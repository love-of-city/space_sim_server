#include "BskSceneController.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonReader.h"
#include "Async/Async.h"
#include "Components/SceneComponent.h"
#include "Components/SpotLightComponent.h"
#include "IPixelStreaming2InputHandler.h"
#include "BskUnrealRuntime.h"
#include "BskRenderWorldSubsystem.h"
#include "Scene/BskSceneUtilities.h"
#include "Engine/World.h"
#include "Dom/JsonObject.h"
#include "Misc/CommandLine.h"
#include "Misc/ConfigCacheIni.h"
#include "Misc/Paths.h"
#include "Misc/Parse.h"
#include "Streaming/BskPixelStreamingCompatibility.h"
#include "BskPreviewCadence.h"
#include "BskCameraPawn.h"
#include "Camera/CameraComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "Engine/TextureRenderTarget2D.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformTime.h"
#include "IPixelStreaming2Module.h"
#include "IPixelStreaming2Streamer.h"
#include "IPixelStreaming2VideoProducer.h"
#include "PixelStreaming2Delegates.h"
#include "Modules/ModuleManager.h"
#include "GameFramework/PlayerController.h"

using namespace BskSceneUtilities;

bool ABskSceneController::TogglePictureInPictureSlot(int32 Slot)
{
    for (const TPair<FString, FBskCameraDefinition>& Pair : ManifestCameras)
    {
        if (!Pair.Value.bPictureInPicture || Pair.Value.PictureInPictureSlot != Slot) continue;
        const bool bVisible = !CameraPictureInPictureVisibility.FindRef(Pair.Key);
        CameraPictureInPictureVisibility.Add(Pair.Key, bVisible);
        if (bVisible) CameraNextCaptureSeconds.Add(Pair.Key, 0.0);
        return bVisible;
    }
    return false;
}

void ABskSceneController::GetPictureInPictureViews(TArray<FBskPictureInPictureView>& OutViews) const
{
    OutViews.Reset();
    for (const TPair<FString, FBskCameraDefinition>& Pair : ManifestCameras)
    {
        if (!Pair.Value.bPictureInPicture) continue;
        FBskPictureInPictureView View;
        View.CameraId = Pair.Key;
        View.DisplayName = Pair.Value.DisplayName.IsEmpty() ? Pair.Key : Pair.Value.DisplayName;
        View.Slot = Pair.Value.PictureInPictureSlot;
        const bool* bVisible = CameraPictureInPictureVisibility.Find(Pair.Key);
        View.bVisible = bVisible == nullptr || *bVisible;
        View.Texture = CameraRenderTargets.FindRef(Pair.Key);
        OutViews.Add(MoveTemp(View));
    }
    OutViews.Sort([](const FBskPictureInPictureView& Left, const FBskPictureInPictureView& Right)
    {
        if (Left.Slot != Right.Slot) return Left.Slot < Right.Slot;
        return Left.CameraId < Right.CameraId;
    });
}

void ABskSceneController::ConfigurePixelStreamingOutput()
{
    bVideoDiagnostics = FParse::Param(FCommandLine::Get(), TEXT("BskVideoDiagnostics"));
    // Dataset capture uses its own reliable channel and needs no WebRTC producer.
    if (FParse::Param(FCommandLine::Get(), TEXT("BskOfflineCapture"))) return;
    if (FParse::Value(FCommandLine::Get(), TEXT("PixelStreamingWebRTCFps="), MaximumStreamingFrameRate))
        MaximumStreamingFrameRate = FMath::Clamp(MaximumStreamingFrameRate, 1, 120);
    FParse::Value(FCommandLine::Get(), TEXT("BskPixelStreamingURL="), PixelStreamingConnectionUrl);
    FParse::Value(FCommandLine::Get(), TEXT("BskPixelStreamingBaseId="), PixelStreamingBaseId);
    FParse::Value(FCommandLine::Get(), TEXT("BskPixelStreamingCameraWidth="), PixelStreamingCameraWidth);
    FParse::Value(FCommandLine::Get(), TEXT("BskPixelStreamingCameraHeight="), PixelStreamingCameraHeight);
    FParse::Value(FCommandLine::Get(), TEXT("BskPixelStreamingCameraFps="), PixelStreamingCameraRateHertz);
    PixelStreamingCameraWidth = FMath::Clamp(PixelStreamingCameraWidth, 160, 1920);
    PixelStreamingCameraHeight = FMath::Clamp(PixelStreamingCameraHeight, 90, 1080);
    PixelStreamingCameraRateHertz = FMath::Clamp(PixelStreamingCameraRateHertz, 1.0, 120.0);
    RequestedPixelStreamingCameraRateHertz = PixelStreamingCameraRateHertz;

    FString Cameras;
    if (FParse::Value(FCommandLine::Get(), TEXT("BskPixelStreamingCameras="), Cameras))
    {
        Cameras.ReplaceInline(TEXT("+"), TEXT(","));
        TArray<FString> Entries;
        Cameras.ParseIntoArray(Entries, TEXT(","), true);
        for (FString CameraId : Entries)
        {
            CameraId.TrimStartAndEndInline();
            if (CameraId.Equals(TEXT("all"), ESearchCase::IgnoreCase)) bPixelStreamingAllManifestCameras = true;
            else if (!CameraId.IsEmpty()) PixelStreamingRequestedCameras.Add(CameraId);
        }
    }
    if (!PixelStreamingConnectionUrl.IsEmpty())
    {
        UE_LOG(LogBskUnreal, Display,
            TEXT("RenderTarget Pixel Streaming enabled url=%s base=%s cameras=%s resolution=%dx%d fps=%.1f"),
            *PixelStreamingConnectionUrl, *PixelStreamingBaseId,
            bPixelStreamingAllManifestCameras ? TEXT("all") : *FString::Join(PixelStreamingRequestedCameras.Array(), TEXT(",")),
            PixelStreamingCameraWidth, PixelStreamingCameraHeight, PixelStreamingCameraRateHertz);
    }
}

bool ABskSceneController::IsPixelStreamingCameraRequested(const FString& CameraId) const
{
    return !PixelStreamingConnectionUrl.IsEmpty()
        && (bPixelStreamingAllManifestCameras || PixelStreamingRequestedCameras.Contains(CameraId));
}

FString ABskSceneController::PixelStreamingIdForCamera(const FString& CameraId) const
{
    FString Safe = CameraId;
    for (int32 Index = 0; Index < Safe.Len(); ++Index)
    {
        const TCHAR Character = Safe[Index];
        if (!FChar::IsAlnum(Character) && Character != TEXT('_') && Character != TEXT('-')) Safe[Index] = TEXT('_');
    }
    return FString::Printf(TEXT("%s__%s"), *PixelStreamingBaseId, *Safe);
}

void ABskSceneController::ConfigurePixelStreamingCamera(AActor* Actor, const FBskCameraDefinition& Definition)
{
    if (!Actor || !IsPixelStreamingCameraRequested(Definition.CameraId)) return;

    // The HUD picture-in-picture and the standalone Pixel Streaming camera must
    // show the exact same rendered image. Reuse the camera's canonical capture
    // and render target instead of maintaining a second view state/exposure path.
    USceneCaptureComponent2D* Capture = CameraCaptureComponents.FindRef(Definition.CameraId);
    UTextureRenderTarget2D* Target = CameraRenderTargets.FindRef(Definition.CameraId);
    if (!Capture || !Target)
    {
        UE_LOG(LogBskUnreal, Error, TEXT("Shared camera capture is unavailable for Pixel Streaming camera %s"),
            *Definition.CameraId);
        return;
    }
    ConfigureFixedRgbExposure(Capture, &Definition);
    ConfigureRgbRenderTarget(Target);
    Capture->bAlwaysPersistRenderingState = true;
    Capture->TextureTarget = Target;
    PixelStreamingCameraCaptures.Add(Definition.CameraId, Capture);
    PixelStreamingCameraTargets.Add(Definition.CameraId, Target);
    PixelStreamingCameraNextCaptureSeconds.Add(Definition.CameraId, 0.0);

    if (Target->SizeX != PixelStreamingCameraWidth || Target->SizeY != PixelStreamingCameraHeight)
    {
        UE_LOG(LogBskUnreal, Warning,
            TEXT("Camera %s shares its manifest resolution %dx%d with HUD and Pixel Streaming; requested streamer resolution was %dx%d"),
            *Definition.CameraId, Target->SizeX, Target->SizeY,
            PixelStreamingCameraWidth, PixelStreamingCameraHeight);
    }

    if (!PixelStreamingCameraStreamers.Contains(Definition.CameraId))
    {
        const FString StreamerId = PixelStreamingIdForCamera(Definition.CameraId);
        TSharedPtr<IPixelStreaming2Streamer> Streamer = IPixelStreaming2Module::Get().CreateStreamer(StreamerId);
        if (!Streamer)
        {
            UE_LOG(LogBskUnreal, Error, TEXT("Could not create Pixel Streaming camera streamer %s"), *StreamerId);
            return;
        }
        TSharedPtr<IPixelStreaming2VideoProducer> Producer =
            BskCreateRenderTargetVideoProducer(Target);
        if (!Producer)
        {
            IPixelStreaming2Module::Get().DeleteStreamer(Streamer);
            UE_LOG(LogBskUnreal, Error, TEXT("Could not create RenderTarget producer for %s"), *StreamerId);
            return;
        }
        Streamer->SetStreamFPS(FMath::RoundToInt(PixelStreamingCameraRateHertz));
        Streamer->SetCoupleFramerate(true);
        Streamer->SetConnectionURL(PixelStreamingConnectionUrl);
        Streamer->SetVideoProducer(Producer);
        Streamer->StartStreaming();
        PixelStreamingCameraStreamers.Add(Definition.CameraId, Streamer);
        PixelStreamingCameraProducers.Add(Definition.CameraId, Producer);
        UE_LOG(LogBskUnreal, Display, TEXT("Started shared-target camera streamer %s for manifest camera %s"),
            *StreamerId, *Definition.CameraId);
    }

    if (!PixelStreamingNewConnectionHandle.IsValid())
    {
        if (UPixelStreaming2Delegates* Delegates = UPixelStreaming2Delegates::Get())
        {
            PixelStreamingNewConnectionHandle = Delegates->OnNewConnectionNative.AddLambda(
                [this](const FString& StreamerId, const FString& PlayerId)
                {
                    for (const TPair<FString, TSharedPtr<IPixelStreaming2Streamer>>& Pair : PixelStreamingCameraStreamers)
                    {
                        if (Pair.Value && PixelStreamingIdForCamera(Pair.Key) == StreamerId)
                        {
                            Pair.Value->ForceKeyFrame();
                            UE_LOG(LogBskUnreal, Display, TEXT("Pixel Streaming camera peer %s connected to %s"),
                                *PlayerId, *StreamerId);
                            break;
                        }
                    }
                });
        }
    }
}

bool ABskSceneController::HasPixelStreamingCameraViewers(const FString& CameraId) const
{
    const TSharedPtr<IPixelStreaming2Streamer> Streamer = PixelStreamingCameraStreamers.FindRef(CameraId);
    if (!Streamer) return false;
    for (const FString& PlayerId : Streamer->GetConnectedPlayers())
        if (PlayerId != Streamer->GetId()) return true;
    return false;
}

bool ABskSceneController::HasPixelStreamingViewportViewers() const
{
    // A visible local viewport is itself a viewer, even without browser peers.
    if (!FParse::Param(FCommandLine::Get(), TEXT("RenderOffscreen"))) return true;
    const TSharedPtr<IPixelStreaming2Streamer> Streamer = IPixelStreaming2Module::Get().FindStreamer(PixelStreamingBaseId);
    if (!Streamer) return false;
    for (const FString& PlayerId : Streamer->GetConnectedPlayers())
        if (PlayerId != Streamer->GetId()) return true;
    return false;
}

void ABskSceneController::UpdatePreviewViewportRendering()
{
    // In server/offscreen mode a camera subscriber does not need a second full
    // 3D view rendered into an unwatched main viewport. SceneCapture continues
    // independently. Never suppress a local editor/windowed user's viewport.
    if ((PixelStreamingConnectionUrl.IsEmpty() && !FParse::Param(FCommandLine::Get(), TEXT("BskOfflineCapture")))
        || !FParse::Param(FCommandLine::Get(), TEXT("RenderOffscreen"))
        || !GEngine || !GEngine->GameViewport) return;
    if (!bManagesViewportRendering)
    {
        bPreviousDisableWorldRendering = GEngine->GameViewport->bDisableWorldRendering;
        bManagesViewportRendering = true;
    }
    GEngine->GameViewport->bDisableWorldRendering = bPreviousDisableWorldRendering || !HasPixelStreamingViewportViewers();
}

void ABskSceneController::UpdatePixelStreamingCameraCaptures()
{
    if (!GetWorld() || PixelStreamingCameraCaptures.IsEmpty()) return;
    const double Now = FPlatformTime::Seconds();
    for (const TPair<FString, TObjectPtr<USceneCaptureComponent2D>>& Pair : PixelStreamingCameraCaptures)
    {
        if (!Pair.Value || !Pair.Value->TextureTarget) continue;
        const FBskCameraDefinition* Definition = ManifestCameras.Find(Pair.Key);
        const bool bVisibleInHud = Definition && Definition->bPictureInPicture
            && CameraPictureInPictureVisibility.FindRef(Pair.Key);
        // A visible HUD PIP was already captured earlier in this game tick and
        // feeds this same RenderTarget. Capture here only for hidden/non-PIP streams.
        if (bVisibleInHud || !HasPixelStreamingCameraViewers(Pair.Key)) continue;
        double& NextSeconds = PixelStreamingCameraNextCaptureSeconds.FindOrAdd(Pair.Key);
        if (!BskPreviewCaptureDue(Now, PixelStreamingCameraRateHertz, NextSeconds)) continue;
        CapturePreview(Pair.Key, Pair.Value);
        if (bVideoDiagnostics) ++VideoDiagnosticsCaptures.FindOrAdd(Pair.Key);
    }
}

void ABskSceneController::ShutdownPixelStreamingCameras()
{
    if (PixelStreamingNewConnectionHandle.IsValid())
    {
        if (UPixelStreaming2Delegates* Delegates = UPixelStreaming2Delegates::Get())
        {
            Delegates->OnNewConnectionNative.Remove(PixelStreamingNewConnectionHandle);
        }
        PixelStreamingNewConnectionHandle.Reset();
    }
    for (TPair<FString, TSharedPtr<IPixelStreaming2Streamer>>& Pair : PixelStreamingCameraStreamers)
    {
        if (!Pair.Value) continue;
        Pair.Value->StopStreaming();
        Pair.Value->SetVideoProducer(nullptr);
        IPixelStreaming2Module::Get().DeleteStreamer(Pair.Value);
    }
    PixelStreamingCameraStreamers.Reset();
    PixelStreamingCameraProducers.Reset();
    PixelStreamingCameraNextCaptureSeconds.Reset();
    PixelStreamingCameraCaptures.Reset();
    PixelStreamingCameraTargets.Reset();
}


ULightComponent* ABskSceneController::ResolveCameraWorkLight(const FString& StreamerId)
{
    if (auto Existing = CameraWorkLights.FindRef(StreamerId).Get()) return Existing;
    USceneComponent* OpticalComponent = nullptr;
    if (IPixelStreaming2Module::IsAvailable() && StreamerId == IPixelStreaming2Module::Get().GetDefaultStreamerID())
    {
        APlayerController* Player = GetWorld() ? GetWorld()->GetFirstPlayerController() : nullptr;
        ABskCameraPawn* Pawn = Player ? Cast<ABskCameraPawn>(Player->GetPawn()) : nullptr;
        OpticalComponent = Pawn ? Pawn->GetBskCameraComponent() : nullptr;
    }
    else
    {
        for (const auto& Pair : PixelStreamingCameraCaptures)
        {
            if (PixelStreamingIdForCamera(Pair.Key) == StreamerId)
            {
                OpticalComponent = Pair.Value;
                break;
            }
        }
    }
    if (!OpticalComponent) return nullptr;
    auto* Light = NewObject<USpotLightComponent>(OpticalComponent->GetOwner());
    Light->SetupAttachment(OpticalComponent);
    Light->SetRelativeTransform(FTransform::Identity);
    Light->SetMobility(EComponentMobility::Movable);
    // Explicit photometric units: legacy Unitless 1500 is only about 2.4 cd.
    // Inspection-preview settings, not a calibrated flight lamp specification.
    const bool bMainViewport = IPixelStreaming2Module::IsAvailable()
        && StreamerId == IPixelStreaming2Module::Get().GetDefaultStreamerID();
    Light->SetIntensityUnits(ELightUnits::Candelas);
    Light->SetIntensity(bMainViewport ? 150000.0f : 1500.0f);
    Light->SetAttenuationRadius(1000.0f);
    Light->SetInnerConeAngle(35.0f);
    Light->SetOuterConeAngle(65.0f);
    Light->SetCastShadows(false);
    Light->RegisterComponent();
    CameraWorkLights.Add(StreamerId, Light);
    UE_LOG(LogBskUnreal, Display, TEXT("Camera work light mounted streamer=%s parent=%s relative_location=%s relative_rotation=%s"),
        *StreamerId, *OpticalComponent->GetName(), *Light->GetRelativeLocation().ToString(), *Light->GetRelativeRotation().ToString());
    return Light;
}


void ABskSceneController::RegisterWorkLightInput(const TSharedPtr<IPixelStreaming2Streamer>& Streamer)
{
    const auto Handler = Streamer ? Streamer->GetInputHandler().Pin() : nullptr;
    if (!Handler || WorkLightInputHandlers.FindRef(Streamer->GetId()).Pin() == Handler) return;
    const TWeakObjectPtr<ABskSceneController> WeakThis(this);
    const TWeakPtr<IPixelStreaming2Streamer> WeakStreamer(Streamer);
    Handler->SetCommandHandler(TEXT("BskWorkLight"),
        [WeakThis, WeakStreamer](FString SourceId, FString, FString Payload)
        {
            if (Payload.Len() > 256) return;
            const auto Apply = [WeakThis, WeakStreamer, SourceId, Payload]()
            {
                auto* Self = WeakThis.Get();
                const auto Target = WeakStreamer.Pin();
                if (!Self || !Target) return;
                TSharedPtr<FJsonObject> Request;
                if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Payload), Request) || !Request) return;
                if (Request->HasField(TEXT("enabled")) && !Request->HasTypedField<EJson::Boolean>(TEXT("enabled"))) return;
                ULightComponent* Light = Self->ResolveCameraWorkLight(Target->GetId());
                bool bEnabled = false;
                if (Light && Request->TryGetBoolField(TEXT("enabled"), bEnabled))
                {
                    Light->SetVisibility(bEnabled);
                    UE_LOG(LogBskUnreal, Display, TEXT("Camera work light streamer=%s enabled=%d intensity_cd=%.1f (render only)"), *Target->GetId(), bEnabled, Light->Intensity);
                }
                auto Reply = MakeShared<FJsonObject>();
                Reply->SetStringField(TEXT("type"), TEXT("bsk_work_light"));
                Reply->SetBoolField(TEXT("supported"), Light != nullptr);
                Reply->SetBoolField(TEXT("enabled"), Light && Light->IsVisible());
                FString Json;
                FJsonSerializer::Serialize(Reply, TJsonWriterFactory<>::Create(&Json));
                Target->SendPlayerMessage(SourceId, TEXT("Response"), Json);
            };
            if (IsInGameThread()) Apply(); else AsyncTask(ENamedThreads::GameThread, Apply);
        });
    Handler->SetCommandHandler(TEXT("BskPreviewOrbit"),
        [WeakThis, WeakStreamer](FString SourceId, FString, FString Payload)
        {
            if (Payload.Len() > 256) return;
            const auto Apply = [WeakThis, WeakStreamer, SourceId, Payload]()
            {
                auto* Self = WeakThis.Get();
                const auto Target = WeakStreamer.Pin();
                if (!Self || !Target) return;
                TSharedPtr<FJsonObject> Request;
                double Yaw = 0, Pitch = 0;
                if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Payload), Request) || !Request
                    || !Request->TryGetNumberField(TEXT("yaw"), Yaw) || !Request->TryGetNumberField(TEXT("pitch"), Pitch)
                    || !FMath::IsFinite(Yaw) || !FMath::IsFinite(Pitch) || FMath::Abs(Yaw)>360 || FMath::Abs(Pitch)>180) return;
                Self->PreviewOrbitDegrees.Add(Target->GetId(), FVector2D(Yaw, Pitch));
                if (Target->GetId() == IPixelStreaming2Module::Get().GetDefaultStreamerID())
                {
                    APlayerController* Player = Self->GetWorld()->GetFirstPlayerController();
                    auto* Pawn = Player ? Cast<ABskCameraPawn>(Player->GetPawn()) : nullptr;
                    if (Pawn) Pawn->ApplyPreviewOrbit(Yaw, Pitch);
                }
                auto Reply = MakeShared<FJsonObject>();
                Reply->SetStringField(TEXT("type"), TEXT("bsk_preview_orbit"));
                Reply->SetNumberField(TEXT("yaw"), Yaw);
                Reply->SetNumberField(TEXT("pitch"), Pitch);
                FString Json;
                FJsonSerializer::Serialize(Reply, TJsonWriterFactory<>::Create(&Json));
                Target->SendPlayerMessage(SourceId, TEXT("Response"), Json);
            };
            if (IsInGameThread()) Apply(); else AsyncTask(ENamedThreads::GameThread, Apply);
        });
    WorkLightInputHandlers.Add(Streamer->GetId(), Handler);
}


void ABskSceneController::CapturePreview(const FString& CameraId, USceneCaptureComponent2D* Capture)
{
    if (!Capture) return;
    AActor* CameraActor = CameraActors.FindRef(CameraId);
    AActor* Bus = BoundActors.FindRef(TEXT("teleop/cubesat_bus"));
    const FVector2D Offset = PreviewOrbitDegrees.FindRef(PixelStreamingIdForCamera(CameraId));
    if (!CameraActor || !Bus || Offset.IsNearlyZero())
    {
        CaptureRgbCamera(CameraId, Capture);
        return;
    }
    // Only the preview component moves, for this capture call. Restore it before
    // authoritative dataset rendering; neither the camera actor nor its manifest changes.
    const FTransform Saved = Capture->GetRelativeTransform();
    const FTransform Base = CameraActor->GetActorTransform();
    const FBox Bounds = Bus->GetComponentsBoundingBox(true);
    const FVector Center = CameraId == TEXT("teleop/camera/spacecraft_overview")
        ? (Bounds.IsValid ? Bounds.GetCenter() : Bus->GetActorLocation())
        : Base.GetLocation() + Base.GetRotation().GetForwardVector() * 30.0;
    const FQuat Yaw(Base.GetRotation().GetUpVector(), FMath::DegreesToRadians(Offset.X));
    const FQuat Pitch(Base.GetRotation().GetRightVector(), FMath::DegreesToRadians(Offset.Y));
    const FQuat Orbit = Yaw * Pitch;
    Capture->SetWorldLocationAndRotation(Center + Orbit.RotateVector(Base.GetLocation()-Center), Orbit * Base.GetRotation());
    CaptureRgbCamera(CameraId, Capture);
    Capture->SetRelativeTransform(Saved);
}
