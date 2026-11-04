#include "BskSceneController.h"
#include "BskEnvironment.h"
#include "BskEnvironmentBody.h"
#include "IPixelStreaming2Streamer.h"
#include "IPixelStreaming2Module.h"
#include "Components/SceneCaptureComponent2D.h"
#include "BskPreviewCadence.h"
#include "BskUnrealRuntime.h"
#include "BskRenderWorldSubsystem.h"
#include "Scene/BskSceneUtilities.h"
#include "Engine/World.h"
#include "Dom/JsonObject.h"
#include "Misc/CommandLine.h"
#include "Misc/ConfigCacheIni.h"
#include "Misc/Paths.h"
#include "Misc/Parse.h"
#include "Capture/BskCapturePipeline.h"
#include "Capture/BskCaptureDiskWriter.h"
#include "Capture/BskCaptureNetworkSender.h"
#include "BskTcpReceiver.h"
#include "BskReplaySource.h"
#include "BskCameraPawn.h"
#include "Camera/CameraActor.h"
#include "Components/SceneComponent.h"
#include "Components/SpotLightComponent.h"
#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "GameFramework/PlayerController.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformTime.h"
#include "HAL/FileManager.h"
#include "TimerManager.h"
#include "UnrealClient.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Misc/FileHelper.h"

#include "Scene/BskRuntimeIdentity.h"

using namespace BskSceneUtilities;

ABskSceneController::ABskSceneController()
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.TickGroup = TG_PrePhysics;
    SetRootComponent(CreateDefaultSubobject<USceneComponent>(TEXT("Root")));
    VisualKindVisibility.Add(TEXT("css"), false);
    VisualKindVisibility.Add(TEXT("generic_sensor"), false);
    VisualKindVisibility.Add(TEXT("transceiver"), false);
}

ABskSceneController::~ABskSceneController() = default;

void ABskSceneController::BeginPlay()
{
    Super::BeginPlay();
    if (BskValidateCookAssetsIfRequested()) return;
    LoadConfiguration();
    CreateEnvironment();
    ConfigureOfflineSkyLighting();
    bEnvironmentPreviewOnly |= FParse::Param(FCommandLine::Get(), TEXT("BskEnvironmentPreview"))
        || (SavedEnvironment && !SavedEnvironment->bAllowBackendControl);
    if (bEnvironmentPreviewOnly)
    {
        UE_LOG(LogBskUnreal, Display, TEXT("BSK environment: independent preview; backend receiver disabled"));
        return;
    }
    ConfigureCaptureOutput();
    ConfigurePixelStreamingOutput();
    // Do not start accepting authoritative simulation frames until configured
    // visual overlays have finished async static-mesh compilation. Otherwise
    // the first manifest/frame can stall the game thread while the Basilisk
    // sender fills its reliable dataset queue and aborts capture.
    PreloadRuntimeStaticMeshes();
    PreloadConfiguredVisualOverlays();
    if (UBskRenderWorldSubsystem* RenderSubsystem = GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>())
    {
        BuiltinCaptureProvider = MakeShared<FBskBuiltinCaptureProvider>(this);
        if (!RenderSubsystem->RegisterCaptureProvider(TEXT("Bsk.BuiltinSceneCapture"), BuiltinCaptureProvider.ToSharedRef(), 0))
        {
            UE_LOG(LogBskUnreal, Error, TEXT("Could not register the built-in BSK capture provider"));
            BuiltinCaptureProvider.Reset();
        }
    }
    if (!ReplayPath.IsEmpty()) Receiver = MakeUnique<FBskReplaySource>(ReplayPath, ReplayRate);
    else Receiver = MakeUnique<FBskTcpReceiver>(ListenAddress, static_cast<uint16>(ListenPort), MaxPacketBytes,
        !CaptureOutputDirectory.IsEmpty() || CaptureNetworkSender.IsValid());
    Receiver->StartSource();
}

void ABskSceneController::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    CancelCaptureJobs();
    ShutdownPixelStreamingCameras();
    if (BuiltinCaptureProvider && GetWorld())
    {
        if (UBskRenderWorldSubsystem* RenderSubsystem = GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>())
        {
            RenderSubsystem->UnregisterCaptureProvider(TEXT("Bsk.BuiltinSceneCapture"));
        }
        BuiltinCaptureProvider.Reset();
    }
    if (CaptureDiskWriter) CaptureDiskWriter->StopAndWait();
    PollCaptureDiskResults();
    CaptureDiskWriter.Reset();
    CaptureNetworkSender.Reset();
    if (Receiver)
    {
        Receiver->StopSource();
        Receiver.Reset();
    }
    if (bManagesViewportRendering && GEngine && GEngine->GameViewport)
        GEngine->GameViewport->bDisableWorldRendering = bPreviousDisableWorldRendering;
    if (SavedEnvironment) SavedEnvironment->SetBackendControlled(false);
    Super::EndPlay(EndPlayReason);
}

void ABskSceneController::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
    if (SavedEnvironment && !SavedEnvironment->bAllowBackendControl && !bEnvironmentPreviewOnly)
    {
        CancelCaptureJobs();
        if (Receiver) { Receiver->StopSource(); Receiver.Reset(); }
        RestoreSavedEnvironmentPreview();
        bEnvironmentPreviewOnly = true;
        if (APlayerController* Player = GetWorld()->GetFirstPlayerController())
            if (SavedEnvironment->PreviewCamera) Player->SetViewTarget(SavedEnvironment->PreviewCamera);
        UE_LOG(LogBskUnreal, Display, TEXT("BSK environment returned to independent preview; restart Play to enable backend control again"));
    }
    if (bEnvironmentPreviewOnly)
    {
        PollCaptureJobs();
        PollCaptureDiskResults();
        return;
    }
    if (IPixelStreaming2Module::IsAvailable() && IPixelStreaming2Module::Get().IsReady())
    {
        auto& Module = IPixelStreaming2Module::Get();
        RegisterWorkLightInput(Module.FindStreamer(Module.GetDefaultStreamerID()));
        for (const auto& Pair : PixelStreamingCameraStreamers) RegisterWorkLightInput(Pair.Value);
        FBskRelativeViewState View;
        if (GetRelativeViewState(View))
        {
            if (auto* Light = Cast<USpotLightComponent>(ResolveCameraWorkLight(Module.GetDefaultStreamerID())))
            {
                // The overview moves from kilometres to metres. Keep the
                // presentation work light useful throughout that camera range;
                // its old 10 m radius left both spacecraft dark at hold points.
                // This is display illumination, not a spacecraft actuator.
                const double RangeCm = FMath::Max(300.0, FMath::Max(
                    FVector::Distance(Light->GetComponentLocation(), View.PrimaryCentimeters),
                    FVector::Distance(Light->GetComponentLocation(), View.SecondaryCentimeters)));
                const double Scale = FMath::Min(RangeCm / 300.0, 10000.0);
                Light->SetIntensity(static_cast<float>(150000.0 * Scale * Scale));
                Light->SetAttenuationRadius(static_cast<float>(FMath::Max(1000.0, RangeCm * 3.0)));
            }
        }
    }
    // Drain after STOP too: accepted strict jobs retain their original episode.
    PollCaptureJobs();
    PollCaptureDiskResults();
    const double TickStart = bVideoDiagnostics ? FPlatformTime::Seconds() : 0.0;
    check(IsInGameThread());
    if (MaximumStreamingFrameRate > 0)
    {
        // Cache the console variables; resolving them every tick adds measurable
        // overhead at the 90-Hz preview rate and can starve the 30-Hz capture.
        static IConsoleVariable* RequestedRate =
            IConsoleManager::Get().FindConsoleVariable(TEXT("PixelStreaming2.WebRTC.Fps"));
        static IConsoleVariable* RenderRate =
            IConsoleManager::Get().FindConsoleVariable(TEXT("t.MaxFPS"));
        if (RequestedRate && RenderRate)
        {
            const int32 FrameRate = FMath::Clamp(RequestedRate->GetInt(), 1, MaximumStreamingFrameRate);
            if (!FMath::IsNearlyEqual(RenderRate->GetFloat(), static_cast<float>(FrameRate)))
            {
                RenderRate->Set(static_cast<float>(FrameRate), ECVF_SetByConsole);
                UE_LOG(LogBskUnreal, Display, TEXT("Streaming render frame rate changed to %d FPS (launch ceiling %d)"),
                    FrameRate, MaximumStreamingFrameRate);
            }
        }
    }
    if (Receiver)
    {
        FBskSceneManifest Manifest;
        if (Receiver->ConsumeLatestManifest(Manifest)) ApplyManifest(Manifest);
        FBskRenderEvent Event;
        while (Receiver->ConsumeEvent(Event)) ApplyEvent(Event);
        FBskRenderFrame Latest;
        uint64 CaptureBatchByteBudget = 0;
        for (const auto& Camera : ManifestCameras)
        {
            CaptureBatchByteBudget += static_cast<uint64>(FMath::Clamp(Camera.Value.Resolution.X, 64, 4096))
                * FMath::Clamp(Camera.Value.Resolution.Y, 64, 4096) * 4 + 65536;
        }
        const int32 ReservedPackets = PendingCaptureJobs.Num() + ManifestCameras.Num();
        const uint64 ReservedBytes = PendingCaptureBytes + CaptureBatchByteBudget;
        const bool bCaptureHasCapacity = ReservedPackets <= MaxAsyncCaptureJobs
            && ReservedBytes <= MaxAsyncCaptureBytes
            && (!CaptureNetworkSender.IsValid()
                || CaptureNetworkSender->HasReliableCapacity(ReservedPackets, ReservedBytes))
            && (!CaptureDiskWriter.IsValid() || CaptureDiskWriter->HasCapacity(ReservedPackets));
        // Leave authoritative frames in the receiver FIFO until output capacity
        // exists. This applies bounded upstream backpressure without skipping.
        if (Receiver->ConsumeForCapture(Latest, bCaptureHasCapacity))
        {
            const double NowSeconds = FPlatformTime::Seconds();
            FString RejectionReason;
            UBskRenderWorldSubsystem* RenderSubsystem = GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>();
            if (RenderSubsystem && !RenderSubsystem->AcceptFrame(Latest, RejectionReason))
            {
                UE_LOG(LogBskUnreal, Warning, TEXT("Ignoring BSK frame: %s"), *RejectionReason);
            }
            else
            {
                // Dataset products are rendered from the exact received frame before
                // presentation interpolation/extrapolation is allowed to touch it.
                const bool bCaptureActive = (Latest.bVisionCaptureRequested || !Latest.bCaptureOnDemand || !Latest.CaptureEpisodeId.IsEmpty())
                    && (!CaptureOutputDirectory.IsEmpty() || CaptureNetworkSender.IsValid());
                const double PreviewRate = bCaptureActive && RequestedPixelStreamingCameraRateHertz > 30.0
                    ? 5.0 : RequestedPixelStreamingCameraRateHertz;
                if (PixelStreamingCameraRateHertz != PreviewRate)
                {
                    PixelStreamingCameraRateHertz = PreviewRate;
                    UE_LOG(LogBskUnreal, Display, TEXT("Camera preview %.1f Hz; strict capture %s episode=%s"),
                        PreviewRate, bCaptureActive ? TEXT("on") : TEXT("off"), *Latest.CaptureEpisodeId);
                }
                UpdateAuthoritativeDataProductCaptures(Latest);
                if (!PresentationClock.Observe(static_cast<double>(Latest.SimulationTimeNanoseconds) * 1.e-9, NowSeconds))
                {
                    PresentationFrames.Reset();
                }
                if (TimeMode.Equals(TEXT("latest"), ESearchCase::IgnoreCase) || PresentationFrames.IsEmpty())
                {
                    ApplyFrame(Latest);
                }
                PresentationFrames.Add(MoveTemp(Latest));
                if (PresentationFrames.Num() > 64) PresentationFrames.RemoveAt(0, PresentationFrames.Num() - 64, EAllowShrinking::No);
            }
        }
        if (PresentationFrames.Num() >= 2 && !TimeMode.Equals(TEXT("latest"), ESearchCase::IgnoreCase))
        {
            const double Cursor = PresentationClock.Advance(DeltaSeconds, FPlatformTime::Seconds(),
                InterpolationDelaySeconds, MaxExtrapolationSeconds);
            while (PresentationFrames.Num() > 2 && static_cast<double>(PresentationFrames[1].SimulationTimeNanoseconds) * 1.e-9 <= Cursor)
                PresentationFrames.RemoveAt(0, 1, EAllowShrinking::No);
            const FBskRenderFrame& From = PresentationFrames[0];
            const FBskRenderFrame& To = PresentationFrames[1];
            const double FromSeconds = static_cast<double>(From.SimulationTimeNanoseconds) * 1.e-9;
            const double ToSeconds = static_cast<double>(To.SimulationTimeNanoseconds) * 1.e-9;
            if (Cursor <= ToSeconds)
            {
                const double Alpha = FMath::Clamp((Cursor - FromSeconds) / FMath::Max(ToSeconds - FromSeconds, 1.e-9), 0.0, 1.0);
                ApplyFrame(InterpolateFrame(From, To, Alpha));
            }
            else
            {
                const double Rate = FMath::Max(PresentationClock.GetRate(), 1.e-9);
                ApplyFrame(ExtrapolateFrame(From, To, (Cursor - ToSeconds) / Rate, Rate));
            }
        }
    }
    if (!SavedEnvironment || SavedEnvironment->IsBackendControlled) UpdateDecorativeSunPlacement();
    UpdatePreviewViewportRendering();
    UpdatePictureInPictureCaptures();
    UpdatePixelStreamingCameraCaptures();
    if (bVideoDiagnostics)
    {
        const double Now = FPlatformTime::Seconds();
        if (VideoDiagnosticsStart <= 0.0) VideoDiagnosticsStart = TickStart;
        VideoDiagnosticsTickWork += Now - TickStart;
        ++VideoDiagnosticsTicks;
        const double Window = Now - VideoDiagnosticsStart;
        if (Window >= 5.0)
        {
            UE_LOG(LogBskUnreal, Display, TEXT("VideoDiagnostics tick_fps=%.1f scene_tick_ms=%.2f"),
                VideoDiagnosticsTicks / Window, 1000.0 * VideoDiagnosticsTickWork / VideoDiagnosticsTicks);
            for (const auto& Pair : VideoDiagnosticsCaptures)
                UE_LOG(LogBskUnreal, Display, TEXT("VideoDiagnostics camera=%s capture_fps=%.1f viewed=%d"),
                    *Pair.Key, Pair.Value / Window, HasPixelStreamingCameraViewers(Pair.Key) ? 1 : 0);
            VideoDiagnosticsStart = Now;
            VideoDiagnosticsTickWork = 0.0;
            VideoDiagnosticsTicks = 0;
            VideoDiagnosticsCaptures.Reset();
        }
    }
    if (GetWorld() && !PendingCommandIds.IsEmpty())
    {
        const double Now = GetWorld()->GetRealTimeSeconds();
        TArray<FString> TimedOut;
        for (const TPair<FString, double>& Pending : PendingCommandIds)
        {
            if (Now - Pending.Value > 10.0) TimedOut.Add(Pending.Key);
        }
        for (const FString& CommandId : TimedOut)
        {
            PendingCommandIds.Remove(CommandId);
            LastCommandStatus = FString::Printf(TEXT("TIMEOUT %s"), *CommandId);
            FBskMissionEventView Timeout;
            Timeout.Sequence = ++CommandSequence;
            Timeout.Kind = TEXT("command_timeout");
            Timeout.Severity = TEXT("error");
            Timeout.Message = FString::Printf(TEXT("No BSK response for %s"), *CommandId);
            if (UBskRenderWorldSubsystem* Subsystem = GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>()) Timeout.SimulationTimeNanoseconds = Subsystem->GetSimulationTimeNanoseconds();
            MissionEventHistory.Add(MoveTemp(Timeout));
        }
    }
}

FString ABskSceneController::GetReceiverStatus() const
{
    if (bEnvironmentPreviewOnly) return TEXT("Environment preview (backend disabled)");
    const FString Source = Receiver ? Receiver->GetStatus() : TEXT("not started");
    if (!SavedEnvironment) return Source;
    return FString(SavedEnvironment->IsBackendControlled
        ? TEXT("Backend driven | ") : TEXT("Saved environment preview | ")) + Source;
}

bool ABskSceneController::SetReplayPaused(bool bPaused)
{
    return Receiver && Receiver->SetPaused(bPaused);
}

bool ABskSceneController::SetReplayRate(double Rate)
{
    return Receiver && Receiver->SetPlaybackRate(Rate);
}

bool ABskSceneController::StepReplay()
{
    return Receiver && Receiver->StepOnce();
}

bool ABskSceneController::SeekReplayNanoseconds(int64 SimulationTimeNanoseconds)
{
    return Receiver && Receiver->SeekSimulationTime(SimulationTimeNanoseconds);
}

bool ABskSceneController::FocusObject(const FString& ObjectId, bool bFollow)
{
    AActor* Target = BoundActors.FindRef(ObjectId);
    APlayerController* Player = GetWorld() ? GetWorld()->GetFirstPlayerController() : nullptr;
    ABskCameraPawn* Pawn = Player ? Cast<ABskCameraPawn>(Player->GetPawn()) : nullptr;
    if (!Target || !Pawn) return false;
    Pawn->SetCameraTarget(Target, bFollow ? EBskCameraMode::Follow : EBskCameraMode::Orbit);
    return true;
}

void ABskSceneController::GetMissionEvents(TArray<FBskMissionEventView>& OutEvents) const
{
    OutEvents = MissionEventHistory;
}

void ABskSceneController::GetUiCommands(TArray<FBskUiCommandDefinition>& OutCommands) const
{
    OutCommands.Reset();
    if (UBskRenderWorldSubsystem* RenderSubsystem = GetWorld() ? GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>() : nullptr)
    {
        if (const FBskSceneManifest* Manifest = RenderSubsystem->GetLatestManifest()) OutCommands = Manifest->UiCommands;
    }
}

bool ABskSceneController::SendUiCommand(
    const FString& Command,
    const FString& TargetId,
    const FString& PayloadJson,
    FString& OutError)
{
    check(IsInGameThread());
    if (!Receiver)
    {
        OutError = TEXT("message source is not running");
        return false;
    }
    TArray<FBskUiCommandDefinition> Commands;
    GetUiCommands(Commands);
    if (!Commands.ContainsByPredicate([&Command](const FBskUiCommandDefinition& Definition)
        { return Definition.Command == Command; }))
    {
        OutError = FString::Printf(TEXT("command '%s' is not declared by the active scene manifest"), *Command);
        return false;
    }
    TSharedPtr<FJsonObject> Payload;
    if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(PayloadJson.IsEmpty() ? TEXT("{}") : PayloadJson), Payload) || !Payload.IsValid())
    {
        OutError = TEXT("command payload must be a JSON object");
        return false;
    }
    const FString CommandId = FString::Printf(TEXT("ue-%lld"), ++CommandSequence);
    UBskRenderWorldSubsystem* RenderSubsystem = GetWorld() ? GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>() : nullptr;
    const int64 SimTime = RenderSubsystem ? RenderSubsystem->GetSimulationTimeNanoseconds() : 0;
    TSharedPtr<FJsonObject> Root = MakeShared<FJsonObject>();
    Root->SetStringField(TEXT("protocol"), BskProtocol::GenericV2);
    Root->SetStringField(TEXT("type"), TEXT("command"));
    Root->SetStringField(TEXT("session_id"), ActiveSessionId);
    Root->SetStringField(TEXT("command_id"), CommandId);
    Root->SetStringField(TEXT("command"), Command);
    Root->SetStringField(TEXT("target_id"), TargetId);
    Root->SetStringField(TEXT("requested_sim_time_ns"), LexToString(SimTime));
    Root->SetObjectField(TEXT("payload"), Payload);
    FString Json;
    FJsonSerializer::Serialize(Root.ToSharedRef(), TJsonWriterFactory<>::Create(&Json));
    if (!Receiver->SendCommandJson(Json, OutError))
    {
        LastCommandStatus = FString::Printf(TEXT("SEND FAILED: %s"), *OutError);
        return false;
    }
    PendingCommandIds.Add(CommandId, GetWorld()->GetRealTimeSeconds());
    LastCommandStatus = FString::Printf(TEXT("PENDING %s"), *Command);
    FBskMissionEventView Sent;
    Sent.Sequence = CommandSequence;
    Sent.Kind = TEXT("command_sent");
    Sent.Severity = TEXT("command");
    Sent.Message = FString::Printf(TEXT("%s -> %s"), *CommandId, *Command);
    Sent.SimulationTimeNanoseconds = SimTime;
    MissionEventHistory.Add(MoveTemp(Sent));
    constexpr int32 MaxMissionEvents = 128;
    if (MissionEventHistory.Num() > MaxMissionEvents) MissionEventHistory.RemoveAt(0, MissionEventHistory.Num() - MaxMissionEvents, EAllowShrinking::No);
    return true;
}

void ABskSceneController::ClearMissionEvents()
{
    MissionEventHistory.Reset();
}

void ABskSceneController::SetVisualKindVisible(const FString& VisualKind, bool bVisible)
{
    const FString NormalizedKind = VisualKind.TrimStartAndEnd().ToLower();
    if (NormalizedKind.IsEmpty()) return;
    VisualKindVisibility.Add(NormalizedKind, bVisible);
    for (const TPair<FString, FBskVisualDefinition>& Pair : ManifestVisuals)
    {
        if (Pair.Value.Kind.Equals(NormalizedKind, ESearchCase::IgnoreCase)) RefreshVisualVisibility(Pair.Key);
    }
}

bool ABskSceneController::ToggleVisualKindVisible(const FString& VisualKind)
{
    const FString NormalizedKind = VisualKind.TrimStartAndEnd().ToLower();
    const bool bNewVisibility = !IsVisualKindVisible(NormalizedKind);
    SetVisualKindVisible(NormalizedKind, bNewVisibility);
    UE_LOG(LogBskUnreal, Display, TEXT("BSK visual helpers kind=%s visible=%s"),
        *NormalizedKind, bNewVisibility ? TEXT("true") : TEXT("false"));
    return bNewVisibility;
}

bool ABskSceneController::IsVisualKindVisible(const FString& VisualKind) const
{
    const FString NormalizedKind = VisualKind.TrimStartAndEnd().ToLower();
    if (const bool* Visibility = VisualKindVisibility.Find(NormalizedKind)) return *Visibility;
    return true;
}

FVector ABskSceneController::GetConfiguredCameraPositionCentimeters() const
{
    return FVector(Converter.LocalMetersToUnrealCentimeters(CameraPositionMeters));
}

FVector ABskSceneController::GetConfiguredCameraLookAtCentimeters() const
{
    return FVector(Converter.LocalMetersToUnrealCentimeters(CameraLookAtMeters));
}

bool ABskSceneController::LoadConfiguration()
{
    FString ConfigPath = FPaths::Combine(FPaths::ProjectConfigDir(), TEXT("bsk_unreal_scene.json"));
    FParse::Value(FCommandLine::Get(), TEXT("BskConfig="), ConfigPath);
    if (FPaths::IsRelative(ConfigPath))
    {
        ConfigPath = FPaths::ConvertRelativePathToFull(ConfigPath);
    }

    FString Json;
    if (!FFileHelper::LoadFileToString(Json, *ConfigPath))
    {
        UE_LOG(LogBskUnreal, Warning, TEXT("Could not read %s; using built-in defaults"), *ConfigPath);
        return false;
    }
    TSharedPtr<FJsonObject> Root;
    if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Json), Root) || !Root.IsValid())
    {
        UE_LOG(LogBskUnreal, Error, TEXT("Invalid scene configuration: %s"), *ConfigPath);
        return false;
    }

    const TSharedPtr<FJsonObject>* Network = nullptr;
    if (Root->TryGetObjectField(TEXT("network"), Network) && Network != nullptr)
    {
        (*Network)->TryGetStringField(TEXT("listen_address"), ListenAddress);
        double Number = 0.0;
        if ((*Network)->TryGetNumberField(TEXT("port"), Number)) ListenPort = FMath::Clamp(static_cast<int32>(Number), 1, 65535);
        if ((*Network)->TryGetNumberField(TEXT("max_packet_bytes"), Number)) MaxPacketBytes = FMath::Clamp(static_cast<uint32>(Number), 1024u, BskProtocol::DefaultMaxPacketBytes);
    }

    const TSharedPtr<FJsonObject>* Replay = nullptr;
    if (Root->TryGetObjectField(TEXT("replay"), Replay) && Replay != nullptr)
    {
        (*Replay)->TryGetStringField(TEXT("path"), ReplayPath);
        (*Replay)->TryGetNumberField(TEXT("rate"), ReplayRate);
    }
    const TSharedPtr<FJsonObject>* Capture = nullptr;
    if (Root->TryGetObjectField(TEXT("capture"), Capture) && Capture != nullptr)
    {
        (*Capture)->TryGetStringField(TEXT("output_directory"), CaptureOutputDirectory);
        (*Capture)->TryGetStringField(TEXT("network_address"), CaptureNetworkAddress);
        double Number = 0.0;
        if ((*Capture)->TryGetNumberField(TEXT("network_port"), Number)) CaptureNetworkPort = FMath::Clamp(static_cast<int32>(Number), 0, 65535);
        if ((*Capture)->TryGetNumberField(TEXT("rate_override_hz"), Number)) CaptureRateOverrideHertz = FMath::Clamp(Number, 0.0, 60.0);
        const TArray<TSharedPtr<FJsonValue>>* Products = nullptr;
        if ((*Capture)->TryGetArrayField(TEXT("products_override"), Products) && Products != nullptr)
        {
            for (const TSharedPtr<FJsonValue>& Value : *Products)
            {
                FString Product;
                if (Value.IsValid() && Value->TryGetString(Product)) CaptureProductOverride.Add(Product.TrimStartAndEnd().ToLower());
            }
        }
    }
    const TSharedPtr<FJsonObject>* Assets = nullptr;
    if (Root->TryGetObjectField(TEXT("assets"), Assets) && Assets != nullptr)
    {
        (*Assets)->TryGetBoolField(TEXT("allow_external_files"), bAllowExternalAssets);
    }

    double CentimetersPerMeter = 100.0;
    bool bMirrorY = true;
    const TSharedPtr<FJsonObject>* Coordinates = nullptr;
    if (Root->TryGetObjectField(TEXT("coordinates"), Coordinates) && Coordinates != nullptr)
    {
        (*Coordinates)->TryGetNumberField(TEXT("centimeters_per_meter"), CentimetersPerMeter);
        (*Coordinates)->TryGetBoolField(TEXT("mirror_local_y_for_unreal"), bMirrorY);
    }
    Converter = FBskCoordinateConverter(CentimetersPerMeter, bMirrorY);

    const TSharedPtr<FJsonObject>* Scene = nullptr;
    if (Root->TryGetObjectField(TEXT("scene"), Scene) && Scene != nullptr)
    {
        double Number = 0.0;
        if ((*Scene)->TryGetNumberField(TEXT("star_count"), Number)) StarCount = FMath::Clamp(static_cast<int32>(Number), 0, 2000);
        if ((*Scene)->TryGetNumberField(TEXT("star_radius_m"), Number)) StarRadiusMeters = FMath::Max(10.0, Number);
        if ((*Scene)->TryGetNumberField(TEXT("celestial_vault_radius_km"), Number)) CelestialVaultRadiusKilometers = FMath::Clamp(Number, 10000.0, 500000.0);
        if ((*Scene)->TryGetNumberField(TEXT("celestial_background_intensity"), Number)) CelestialBackgroundIntensity = FMath::Clamp(Number, 0.0, 8.0);
        if ((*Scene)->TryGetNumberField(TEXT("sun_intensity_lux"), Number)) SunIntensityLux = FMath::Max(0.0, Number);
        if ((*Scene)->TryGetNumberField(TEXT("sun_illuminance_scale"), Number)) SunIlluminanceScale = FMath::Max(0.0, Number);
        if ((*Scene)->TryGetNumberField(TEXT("fill_light_intensity_lux"), Number)) FillLightIntensityLux = FMath::Max(0.0, Number);
        if ((*Scene)->TryGetNumberField(TEXT("material_exposure_bias"), Number)) MaterialExposureBias = FMath::Clamp(Number, -8.0, 8.0);
        if ((*Scene)->TryGetNumberField(TEXT("sun_visual_distance_m"), Number)) SunVisualDistanceMeters = FMath::Max(1000.0, Number);
        if ((*Scene)->TryGetNumberField(TEXT("sun_visual_angular_diameter_deg"), Number)) SunVisualAngularDiameterDegrees = FMath::Clamp(Number, 0.05, 10.0);
        if ((*Scene)->TryGetNumberField(TEXT("sun_visual_emissive_strength"), Number)) SunVisualEmissiveStrength = FMath::Clamp(Number, 0.0, 1000.0);
        if ((*Scene)->TryGetNumberField(TEXT("sun_visual_effect_scale"), Number)) SunVisualEffectScale = FMath::Clamp(Number, 0.01, 100.0);
        if ((*Scene)->TryGetNumberField(TEXT("decorative_earth_radius_m"), Number)) DecorativeEarthRadiusMeters = FMath::Max(1.0, Number);
        if ((*Scene)->TryGetNumberField(TEXT("earth_cloud_scale"), Number)) EarthCloudScale = FMath::Clamp(Number, 1.0, 1.2);
        if ((*Scene)->TryGetNumberField(TEXT("earth_atmosphere_scale"), Number)) EarthAtmosphereScale = FMath::Clamp(Number, EarthCloudScale, 1.5);
        (*Scene)->TryGetBoolField(TEXT("use_official_celestial_assets"), bUseOfficialCelestialAssets);
        (*Scene)->TryGetBoolField(TEXT("use_textured_star_sphere"), bUseTexturedStarSphere);
        (*Scene)->TryGetBoolField(TEXT("use_earth_sky_atmosphere"), bUseEarthSkyAtmosphere);
        (*Scene)->TryGetBoolField(TEXT("use_manual_exposure"), bUseManualExposure);
        (*Scene)->TryGetBoolField(TEXT("sun_visual_enabled"), bEnableDecorativeSun);
        (*Scene)->TryGetBoolField(TEXT("sun_visual_effects_enabled"), bEnableDecorativeSunEffects);
        (*Scene)->TryGetBoolField(TEXT("decorative_earth_enabled"), bEnableDecorativeEarth);
        (*Scene)->TryGetStringField(TEXT("textured_star_mesh"), TexturedStarMeshPath);
        (*Scene)->TryGetStringField(TEXT("textured_star_material"), TexturedStarMaterialPath);
        (*Scene)->TryGetStringField(TEXT("sun_visual_mesh"), SunVisualMeshPath);
        (*Scene)->TryGetStringField(TEXT("sun_visual_material"), SunVisualMaterialPath);
        (*Scene)->TryGetStringField(TEXT("sun_burst_particle"), SunBurstParticlePath);
        (*Scene)->TryGetStringField(TEXT("sun_halo_particle"), SunHaloParticlePath);
        (*Scene)->TryGetStringField(TEXT("sun_lines_particle"), SunLinesParticlePath);
        (*Scene)->TryGetStringField(TEXT("earth_sphere_mesh"), EarthSphereMeshPath);
        (*Scene)->TryGetStringField(TEXT("earth_surface_material"), EarthSurfaceMaterialPath);
        (*Scene)->TryGetStringField(TEXT("earth_cloud_material"), EarthCloudMaterialPath);
        (*Scene)->TryGetStringField(TEXT("earth_atmosphere_material"), EarthAtmosphereMaterialPath);
        JsonVector3(*Scene, TEXT("camera_position_m"), CameraPositionMeters);
        JsonVector3(*Scene, TEXT("camera_look_at_m"), CameraLookAtMeters);
        JsonVector3(*Scene, TEXT("decorative_earth_position_m"), DecorativeEarthPositionMeters);
        FVector3d Rotation;
        if (JsonVector3(*Scene, TEXT("sun_rotation_deg"), Rotation)) SunRotation = FRotator(Rotation.X, Rotation.Y, Rotation.Z);
        if (JsonVector3(*Scene, TEXT("decorative_earth_rotation_deg"), Rotation)) DecorativeEarthRotation = FRotator(Rotation.X, Rotation.Y, Rotation.Z);
        (*Scene)->TryGetStringField(TEXT("time_mode"), TimeMode);
        double InterpolationDelayMs = InterpolationDelaySeconds * 1000.0;
        double MaxExtrapolationMs = MaxExtrapolationSeconds * 1000.0;
        if ((*Scene)->TryGetNumberField(TEXT("interpolation_delay_ms"), InterpolationDelayMs)) InterpolationDelaySeconds = FMath::Max(0.001, InterpolationDelayMs / 1000.0);
        if ((*Scene)->TryGetNumberField(TEXT("max_extrapolation_ms"), MaxExtrapolationMs)) MaxExtrapolationSeconds = FMath::Max(0.0, MaxExtrapolationMs / 1000.0);
    }

    const TSharedPtr<FJsonObject>* Visuals = nullptr;
    if (Root->TryGetObjectField(TEXT("visuals"), Visuals) && Visuals != nullptr)
    {
        const TSharedPtr<FJsonObject>* VisibilityByKind = nullptr;
        if ((*Visuals)->TryGetObjectField(TEXT("visibility_by_kind"), VisibilityByKind) && VisibilityByKind != nullptr)
        {
            for (const TPair<FString, TSharedPtr<FJsonValue>>& Entry : (*VisibilityByKind)->Values)
            {
                bool bVisible = true;
                if (Entry.Value.IsValid() && Entry.Value->TryGetBool(bVisible))
                {
                    VisualKindVisibility.Add(Entry.Key.TrimStartAndEnd().ToLower(), bVisible);
                }
            }
        }
    }

    const TSharedPtr<FJsonObject>* Objects = nullptr;
    if (Root->TryGetObjectField(TEXT("objects"), Objects) && Objects != nullptr)
    {
        for (const TPair<FString, TSharedPtr<FJsonValue>>& Entry : (*Objects)->Values)
        {
            const TSharedPtr<FJsonObject> Object = Entry.Value->AsObject();
            if (!Object.IsValid()) continue;
            FObjectSpec Spec;
            Object->TryGetStringField(TEXT("asset_type"), Spec.AssetType);
            Object->TryGetStringField(TEXT("asset_path"), Spec.AssetPath);
            Object->TryGetStringField(TEXT("actor_class"), Spec.ActorClass);
            Object->TryGetStringField(TEXT("placeholder_shape"), Spec.PlaceholderShape);
            JsonVector3(Object, TEXT("size_m"), Spec.SizeMeters);
            JsonVector3(Object, TEXT("scale"), Spec.Scale);
            JsonColor(Object, TEXT("color"), Spec.Color);
            ObjectSpecs.Add(Entry.Key, MoveTemp(Spec));
        }
    }

    FString AddressOverride;
    if (FParse::Value(FCommandLine::Get(), TEXT("BskListen="), AddressOverride)) ListenAddress = AddressOverride;
    int32 PortOverride = 0;
    if (FParse::Value(FCommandLine::Get(), TEXT("BskPort="), PortOverride)) ListenPort = FMath::Clamp(PortOverride, 1, 65535);
    FParse::Value(FCommandLine::Get(), TEXT("BskReplay="), ReplayPath);
    FParse::Value(FCommandLine::Get(), TEXT("BskReplayRate="), ReplayRate);
    FParse::Value(FCommandLine::Get(), TEXT("BskAutoCommand="), AutoCommand);
    if (!ReplayPath.IsEmpty() && FPaths::IsRelative(ReplayPath)) ReplayPath = FPaths::ConvertRelativePathToFull(ReplayPath);
    UE_LOG(LogBskUnreal, Display, TEXT("Loaded BSK scene config %s"), *ConfigPath);
    return true;
}

void ABskSceneController::ResetPresentationState()
{
    check(IsInGameThread());
    VisionCaptureFramesRequested = 0;
    VisionCapturePacketsQueued = 0;
    VisionCaptureErrors = 0;
    LastFrameId = -1;
    bHasPresentationFrame = false;
    PresentationFrame = FBskRenderFrame{};
    PresentationClock.Reset();
    PresentationFrames.Reset();
    // Simulation time goes back to zero: old deadlines would stall capture
    // until the new session caught up with the previous one.
    CameraNextDataCaptureSimulationNanoseconds.Reset();
    CameraNextCaptureSeconds.Reset();
    PixelStreamingCameraNextCaptureSeconds.Reset();
    auto CutCaptures = [](auto& Captures)
    {
        for (auto& Pair : Captures)
        {
            if (Pair.Value) Pair.Value->bCameraCutThisFrame = true;
        }
    };
    CutCaptures(CameraCaptureComponents);
    CutCaptures(PixelStreamingCameraCaptures);
}

void ABskSceneController::ApplyEvent(const FBskRenderEvent& Event)
{
    check(IsInGameThread());
    if (!ActiveSessionId.IsEmpty() && Event.SessionId != ActiveSessionId) return;
    FBskMissionEventView View;
    View.Sequence = Event.Sequence;
    View.Kind = Event.EventKind;
    View.Message = Event.EventKind;
    if (!Event.PayloadJson.IsEmpty())
    {
        TSharedPtr<FJsonObject> Payload;
        if (FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Event.PayloadJson), Payload) && Payload.IsValid())
        {
            Payload->TryGetStringField(TEXT("severity"), View.Severity);
            Payload->TryGetStringField(TEXT("message"), View.Message);
            int64 EventSimTime = 0;
            FString EventSimTimeString;
            if (Payload->TryGetStringField(TEXT("sim_time_ns"), EventSimTimeString)) LexTryParseString(EventSimTime, *EventSimTimeString);
            View.SimulationTimeNanoseconds = EventSimTime;
            if (Event.EventKind == TEXT("near_field_status"))
            {
                bool bWorkingView = false;
                if (Payload->TryGetBoolField(TEXT("capture26_working_view"), bWorkingView))
                {
                    APlayerController* Player = GetWorld()->GetFirstPlayerController();
                    if (auto* Pawn = Player ? Cast<ABskCameraPawn>(Player->GetPawn()) : nullptr)
                        Pawn->bCapture26WorkingView = bWorkingView;
                }
            }
            if (Event.EventKind == TEXT("capture26_stage_snapshot"))
            {
                FString Directory, Filename;
                if (FParse::Value(FCommandLine::Get(), TEXT("BskDemoEvidenceDir="), Directory)
                    && Payload->TryGetStringField(TEXT("filename"), Filename)
                    && Filename == FPaths::GetCleanFilename(Filename) && Filename.EndsWith(TEXT(".png")))
                {
                    Directory = FPaths::ConvertRelativePathToFull(Directory);
                    IFileManager::Get().MakeDirectory(*Directory, true);
                    const FString Path = FPaths::Combine(Directory, Filename);
                    FTimerHandle Timer;
                    GetWorld()->GetTimerManager().SetTimer(Timer, [Path]()
                    {
                        FScreenshotRequest::RequestScreenshot(Path, true, false);
                    }, 0.15f, false);
                }
            }
            if (Event.EventKind == TEXT("command_result"))
            {
                FString CommandId;
                FString Command;
                FString Status;
                Payload->TryGetStringField(TEXT("command_id"), CommandId);
                Payload->TryGetStringField(TEXT("command"), Command);
                Payload->TryGetStringField(TEXT("status"), Status);
                PendingCommandIds.Remove(CommandId);
                LastCommandStatus = FString::Printf(TEXT("%s %s"), *Status.ToUpper(), *CommandId);

                // The protocol keeps structured command output in ``result``.  The
                // original HUD displayed only ``message`` (for example
                // "mission.status completed"), which described completion of the
                // query and hid the actual mission phase.  Promote the commonly used
                // mission status fields to a short, human-readable timeline entry.
                const TSharedPtr<FJsonObject>* Result = nullptr;
                if (Status.Equals(TEXT("accepted"), ESearchCase::IgnoreCase)
                    && Payload->TryGetObjectField(TEXT("result"), Result)
                    && Result != nullptr && Result->IsValid())
                {
                    FString Phase;
                    bool bPaused = false;
                    const bool bHasPhase = (*Result)->TryGetStringField(TEXT("phase"), Phase);
                    const bool bHasPaused = (*Result)->TryGetBoolField(TEXT("paused"), bPaused);
                    if (bHasPhase || bHasPaused)
                    {
                        TArray<FString> Details;
                        if (bHasPhase) Details.Add(FString::Printf(TEXT("phase=%s"), *Phase));
                        if (bHasPaused) Details.Add(FString::Printf(TEXT("paused=%s"), bPaused ? TEXT("true") : TEXT("false")));
                        View.Message = FString::Printf(TEXT("%s: %s"), *Command, *FString::Join(Details, TEXT(", ")));
                    }
                    else
                    {
                        View.Message = FString::Printf(TEXT("%s accepted"), *Command);
                    }
                }
            }
        }
    }
    if (View.SimulationTimeNanoseconds == 0)
    {
        if (UBskRenderWorldSubsystem* RenderSubsystem = GetWorld() ? GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>() : nullptr)
        {
            View.SimulationTimeNanoseconds = RenderSubsystem->GetSimulationTimeNanoseconds();
        }
    }
    MissionEventHistory.Add(MoveTemp(View));
    constexpr int32 MaxMissionEvents = 128;
    if (MissionEventHistory.Num() > MaxMissionEvents) MissionEventHistory.RemoveAt(0, MissionEventHistory.Num() - MaxMissionEvents, EAllowShrinking::No);
    if (Event.EventKind == TEXT("scene_reset"))
    {
        ResetPresentationState();
    }
    if (UBskRenderWorldSubsystem* RenderSubsystem = GetWorld() ? GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>() : nullptr)
    {
        RenderSubsystem->NotifyEventApplied(Event);
    }
    UE_LOG(LogBskUnreal, Display, TEXT("BSK event kind=%s sequence=%lld"), *Event.EventKind, Event.Sequence);
}


void ABskSceneController::ApplyFrame(const FBskRenderFrame& Frame, bool bPresentationFrame)
{
    check(IsInGameThread());
    if (bEnvironmentPreviewOnly || (SavedEnvironment && !SavedEnvironment->bAllowBackendControl)) return;
    if (LastFrameId < 0)
    {
        UE_LOG(LogBskUnreal, Display, TEXT("Applying first BSK frame %lld with %d objects on the Game Thread"), Frame.FrameId, Frame.Objects.Num());
    }
    for (const FBskRenderObjectState& State : Frame.Objects)
    {
        GetOrCreateActor(State);
    }
    for (const FBskRenderObjectState& State : Frame.Objects)
    {
        const FString& Key = State.ObjectId.IsEmpty() ? State.Name : State.ObjectId;
        AActor* Actor = BoundActors.FindRef(Key);
        if (!Actor) continue;
        FString ParentId = State.Parent;
        if (const FBskObjectDefinition* Definition = ManifestObjects.Find(Key)) ParentId = Definition->ParentId;
        AActor* DesiredParent = ParentId.IsEmpty() ? nullptr : BoundActors.FindRef(ParentId);
        if (DesiredParent && Actor->GetAttachParentActor() != DesiredParent)
        {
            Actor->AttachToActor(DesiredParent, FAttachmentTransformRules::KeepWorldTransform);
        }
        else if (!DesiredParent && Actor->GetAttachParentActor())
        {
            Actor->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
        }
        const FVector Location(Converter.LocalMetersToUnrealCentimeters(State.PositionMeters));
        const FQuat Rotation(Converter.ActiveLocalWxyzToUnreal(State.OrientationWxyz));
        Actor->SetActorLocationAndRotation(Location, Rotation, false, nullptr, ETeleportType::TeleportPhysics);
    }
    UpdateCelestialBodies(Frame);
    UpdateVisualStates(Frame);
    UpdateLightTargets();
    DrawOrbitLines(Frame);
    if (!bPresentationFrame) return;
    PresentationFrame = Frame;
    bHasPresentationFrame = true;
    LastFrameId = Frame.FrameId;
    FrameAppliedEvent.Broadcast(Frame);
    if (UBskRenderWorldSubsystem* RenderSubsystem = GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>())
    {
        RenderSubsystem->NotifyFrameApplied(Frame);
    }

    if (!bScreenshotRequested)
    {
        FString ScreenshotPath;
        if (FParse::Value(FCommandLine::Get(), TEXT("BskScreenshot="), ScreenshotPath) && !ScreenshotPath.IsEmpty())
        {
            if (FPaths::IsRelative(ScreenshotPath))
            {
                ScreenshotPath = FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir(), ScreenshotPath);
            }
            bScreenshotRequested = true;
            FTimerHandle ScreenshotTimer;
            GetWorldTimerManager().SetTimer(
                ScreenshotTimer,
                [ScreenshotPath]()
                {
                    FScreenshotRequest::RequestScreenshot(ScreenshotPath, false, false);
                    UE_LOG(LogBskUnreal, Display, TEXT("Requested delayed BSK validation screenshot: %s"), *ScreenshotPath);
                },
                1.5f,
                false);
        }
    }
}

FBskRenderFrame ABskSceneController::InterpolateFrame(const FBskRenderFrame& From, const FBskRenderFrame& To, double Alpha) const
{
    FBskRenderFrame Result = To;
    Result.SimulationTimeNanoseconds = FMath::RoundToInt64(FMath::Lerp(
        static_cast<double>(From.SimulationTimeNanoseconds), static_cast<double>(To.SimulationTimeNanoseconds), Alpha));
    Result.OriginInertialMeters = FMath::Lerp(From.OriginInertialMeters, To.OriginInertialMeters, Alpha);
    TMap<FString, const FBskRenderObjectState*> FromObjects;
    for (const FBskRenderObjectState& State : From.Objects)
    {
        FromObjects.Add(State.ObjectId.IsEmpty() ? State.Name : State.ObjectId, &State);
    }
    for (FBskRenderObjectState& State : Result.Objects)
    {
        const FString& Key = State.ObjectId.IsEmpty() ? State.Name : State.ObjectId;
        if (const FBskRenderObjectState* const* Prior = FromObjects.Find(Key))
        {
            State.PositionMeters = FMath::Lerp((*Prior)->PositionMeters, State.PositionMeters, Alpha);
            State.OrientationWxyz = FQuat4d::Slerp((*Prior)->OrientationWxyz, State.OrientationWxyz, Alpha).GetNormalized();
            if ((*Prior)->bHasVelocity && State.bHasVelocity)
            {
                State.VelocityMetersPerSecond = FMath::Lerp((*Prior)->VelocityMetersPerSecond, State.VelocityMetersPerSecond, Alpha);
            }
        }
    }
    TMap<FString, const FBskCelestialBodyState*> FromCelestial;
    for (const FBskCelestialBodyState& State : From.CelestialBodies) FromCelestial.Add(State.BodyId, &State);
    for (FBskCelestialBodyState& State : Result.CelestialBodies)
    {
        if (const FBskCelestialBodyState* const* Prior = FromCelestial.Find(State.BodyId))
        {
            State.PositionMeters = FMath::Lerp((*Prior)->PositionMeters, State.PositionMeters, Alpha);
            State.OrientationWxyz = FQuat4d::Slerp((*Prior)->OrientationWxyz, State.OrientationWxyz, Alpha).GetNormalized();
        }
    }
    TMap<FString, const FBskVisualState*> FromVisuals;
    for (const FBskVisualState& State : From.VisualStates) FromVisuals.Add(State.VisualId, &State);
    for (FBskVisualState& State : Result.VisualStates)
    {
        const FBskVisualState* const* Prior = FromVisuals.Find(State.VisualId);
        if (!Prior) continue;
        State.Value = FMath::Lerp((*Prior)->Value, State.Value, Alpha);
        for (TPair<FString, FBskVisualState::FChannelValue>& Pair : State.Channels)
        {
            const FBskVisualState::FChannelValue* PriorChannel = (*Prior)->Channels.Find(Pair.Key);
            if (PriorChannel && PriorChannel->Type == FBskVisualState::FChannelValue::EType::Number &&
                Pair.Value.Type == FBskVisualState::FChannelValue::EType::Number)
            {
                Pair.Value.Number = FMath::Lerp(PriorChannel->Number, Pair.Value.Number, Alpha);
            }
        }
    }
    return Result;
}

FBskRenderFrame ABskSceneController::ExtrapolateFrame(
    const FBskRenderFrame& From,
    const FBskRenderFrame& To,
    double SecondsBeyondTarget,
    double SimulationRate) const
{
    FBskRenderFrame Result = To;
    const double SourceDeltaSeconds = static_cast<double>(To.SimulationTimeNanoseconds - From.SimulationTimeNanoseconds) * 1.0e-9;
    const double ExtraSeconds = FMath::Clamp(SecondsBeyondTarget, 0.0, MaxExtrapolationSeconds) * FMath::Max(0.0, SimulationRate);
    if (SourceDeltaSeconds <= UE_DOUBLE_SMALL_NUMBER || ExtraSeconds <= 0.0) return Result;
    Result.SimulationTimeNanoseconds += FMath::RoundToInt64(ExtraSeconds * 1.0e9);
    Result.OriginInertialMeters += (To.OriginInertialMeters - From.OriginInertialMeters) * (ExtraSeconds / SourceDeltaSeconds);

    const auto ExtrapolateOrientation = [SourceDeltaSeconds, ExtraSeconds](const FQuat4d& A, const FQuat4d& B)
    {
        FQuat4d Delta = (B * A.Inverse()).GetNormalized();
        if (Delta.W < 0.0) Delta = FQuat4d(-Delta.X, -Delta.Y, -Delta.Z, -Delta.W);
        const double HalfAngle = FMath::Acos(FMath::Clamp(Delta.W, -1.0, 1.0));
        const double SinHalf = FMath::Sin(HalfAngle);
        if (FMath::Abs(SinHalf) <= UE_DOUBLE_SMALL_NUMBER) return B;
        const FVector3d Axis(Delta.X / SinHalf, Delta.Y / SinHalf, Delta.Z / SinHalf);
        const FQuat4d ExtraRotation(Axis, 2.0 * HalfAngle * ExtraSeconds / SourceDeltaSeconds);
        return (ExtraRotation * B).GetNormalized();
    };

    TMap<FString, const FBskRenderObjectState*> FromObjects;
    for (const FBskRenderObjectState& State : From.Objects)
    {
        FromObjects.Add(State.ObjectId.IsEmpty() ? State.Name : State.ObjectId, &State);
    }
    for (FBskRenderObjectState& State : Result.Objects)
    {
        const FString& Key = State.ObjectId.IsEmpty() ? State.Name : State.ObjectId;
        const FBskRenderObjectState* const* Prior = FromObjects.Find(Key);
        if (!Prior) continue;
        // Positions are relative to the moving render origin; wire velocities
        // are inertial velocities expressed in the local axes (also used for
        // orbital telemetry). Integrating those into local positions adds the
        // spacecraft's ~7.6 km/s orbital motion a second time, then snaps back
        // on every received frame. Differentiate the LOCAL positions instead.
        const FVector3d Velocity = (State.PositionMeters - (*Prior)->PositionMeters) / SourceDeltaSeconds;
        State.PositionMeters += Velocity * ExtraSeconds;
        State.OrientationWxyz = ExtrapolateOrientation((*Prior)->OrientationWxyz, State.OrientationWxyz);
    }

    TMap<FString, const FBskCelestialBodyState*> FromCelestial;
    for (const FBskCelestialBodyState& State : From.CelestialBodies) FromCelestial.Add(State.BodyId, &State);
    for (FBskCelestialBodyState& State : Result.CelestialBodies)
    {
        const FBskCelestialBodyState* const* Prior = FromCelestial.Find(State.BodyId);
        if (!Prior) continue;
        // Celestial positions use the same moving origin as ordinary objects.
        // A nonzero inertial ephemeris velocity is not a local position derivative.
        const FVector3d Velocity = (State.PositionMeters - (*Prior)->PositionMeters) / SourceDeltaSeconds;
        State.PositionMeters += Velocity * ExtraSeconds;
        State.OrientationWxyz = ExtrapolateOrientation((*Prior)->OrientationWxyz, State.OrientationWxyz);
    }
    return Result;
}


bool ABskSceneController::GetRelativeViewState(FBskRelativeViewState& Out) const
{
    if (!RelativeView.IsConfigured() || !bHasPresentationFrame) return false;
    const AActor* Primary = BoundActors.FindRef(RelativeView.PrimaryObjectId);
    const AActor* Secondary = BoundActors.FindRef(RelativeView.SecondaryObjectId);
    if (!IsValid(Primary) || !IsValid(Secondary)) return false;
    Out.PrimaryCentimeters = Primary->GetActorTransform().TransformPosition(
        FVector(Converter.LocalMetersToUnrealCentimeters(RelativeView.PrimaryCenterBodyMeters)));
    Out.SecondaryCentimeters = Secondary->GetActorTransform().TransformPosition(
        FVector(Converter.LocalMetersToUnrealCentimeters(RelativeView.SecondaryCenterBodyMeters)));
    Out.PrimaryLabel = RelativeView.PrimaryLabel;
    Out.SecondaryLabel = RelativeView.SecondaryLabel;
    Out.DistanceMeters = FVector::Distance(Out.PrimaryCentimeters, Out.SecondaryCentimeters) / Converter.GetCentimetersPerMeter();
    Out.SimulationSeconds = static_cast<double>(PresentationFrame.SimulationTimeNanoseconds) * 1e-9;
    const auto LocalMeters = [this](const FVector& Position)
    {
        const double YSign = Converter.MirrorsLocalY() ? -1.0 : 1.0;
        return FVector3d(Position.X, Position.Y * YSign, Position.Z) / Converter.GetCentimetersPerMeter();
    };
    Out.PrimarySunlight = SolarVisibilityAt(LocalMeters(Out.PrimaryCentimeters));
    Out.SecondarySunlight = SolarVisibilityAt(LocalMeters(Out.SecondaryCentimeters));
    return true;
}


ACameraActor* ABskSceneController::GetSavedPreviewCamera() const
{
    return SavedEnvironment ? SavedEnvironment->PreviewCamera.Get() : nullptr;
}
