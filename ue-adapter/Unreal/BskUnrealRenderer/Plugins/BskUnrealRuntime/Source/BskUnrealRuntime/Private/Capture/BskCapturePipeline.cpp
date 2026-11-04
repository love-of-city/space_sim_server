#include "BskCapturePipeline.h"
#include "BskCaptureRouting.h"
#include "BskCaptureEncoding.h"
#include "BskPreviewCadence.h"
#include "BskSceneController.h"
#include "BskEnvironment.h"
#include "BskEnvironmentBody.h"
#include "BskUnrealRuntime.h"
#include "BskRenderWorldSubsystem.h"
#include "Scene/BskSceneUtilities.h"
#include "Engine/World.h"
#include "Dom/JsonObject.h"
#include "Misc/CommandLine.h"
#include "Misc/ConfigCacheIni.h"
#include "Misc/Paths.h"
#include "Misc/Parse.h"
#include "Capture/BskCaptureDiskWriter.h"
#include "Capture/BskCaptureNetworkSender.h"
#include "BskCameraEffects.h"
#include "BskLeRobotSampling.h"
#include "Camera/CameraActor.h"
#include "Camera/CameraComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/LightComponent.h"
#include "Components/SpotLightComponent.h"
#include "Engine/SpotLight.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Async/Async.h"
#include "HAL/PlatformTime.h"
#include "IImageWrapperModule.h"
#include "Modules/ModuleManager.h"
#include "Serialization/JsonSerializer.h"
#include "RHICommandList.h"

using namespace BskSceneUtilities;
using namespace BskCaptureEncoding;

void ABskSceneController::ConfigureCaptureOutput()
{
    bAsyncCaptureEnabled = !FParse::Param(FCommandLine::Get(), TEXT("BskSynchronousCapture"));
    FParse::Value(FCommandLine::Get(), TEXT("BskCaptureDir="), CaptureOutputDirectory);
    FParse::Value(FCommandLine::Get(), TEXT("BskCaptureHost="), CaptureNetworkAddress);
    FParse::Value(FCommandLine::Get(), TEXT("BskCapturePort="), CaptureNetworkPort);
    FParse::Value(FCommandLine::Get(), TEXT("BskCaptureRate="), CaptureRateOverrideHertz);
    FParse::Value(FCommandLine::Get(), TEXT("BskPreviewRate="), PreviewRateOverrideHertz);
    FString ProductsOverride;
    if (FParse::Value(FCommandLine::Get(), TEXT("BskCaptureProducts="), ProductsOverride))
    {
        CaptureProductOverride.Reset();
        ProductsOverride.ReplaceInline(TEXT("+"), TEXT(","));
        ProductsOverride.ParseIntoArray(CaptureProductOverride, TEXT(","), true);
        for (FString& Product : CaptureProductOverride) Product = Product.TrimStartAndEnd().ToLower();
    }
    const auto IsKnownProduct = [](const FString& Product)
    {
        return Product == TEXT("rgb");
    };
    for (const FString& Product : CaptureProductOverride)
    {
        if (!IsKnownProduct(Product))
        {
            UE_LOG(LogBskUnreal, Error, TEXT("Unsupported -BskCaptureProducts entry '%s'; capture is disabled until corrected"), *Product);
            CaptureOutputDirectory.Reset();
            CaptureNetworkPort = 0;
            CaptureProductOverride.Reset();
            return;
        }
    }
    if (!CaptureOutputDirectory.IsEmpty())
    {
        if (FPaths::IsRelative(CaptureOutputDirectory)) CaptureOutputDirectory = FPaths::ConvertRelativePathToFull(CaptureOutputDirectory);
        FPaths::CollapseRelativeDirectories(CaptureOutputDirectory);
        CaptureDiskWriter = MakeShared<FBskCaptureDiskWriter>();
    }
    CaptureRateOverrideHertz = FMath::Clamp(CaptureRateOverrideHertz, 0.0, 60.0);
    PreviewRateOverrideHertz = FMath::Clamp(PreviewRateOverrideHertz, 0.0, 60.0);
    CaptureNetworkPort = FMath::Clamp(CaptureNetworkPort, 0, 65535);
    if (CaptureNetworkPort > 0)
    {
        CaptureNetworkSender = MakeShared<FBskCaptureNetworkSender>(CaptureNetworkAddress, static_cast<uint16>(CaptureNetworkPort));
    }
    UE_LOG(LogBskUnreal, Display, TEXT("BSK camera products disk=%s network=%s:%d capture_hz=%.1f preview_hz=%.1f override_products=%s"),
        CaptureOutputDirectory.IsEmpty() ? TEXT("disabled") : *CaptureOutputDirectory,
        *CaptureNetworkAddress, CaptureNetworkPort, CaptureRateOverrideHertz, PreviewRateOverrideHertz,
        CaptureProductOverride.IsEmpty() ? TEXT("manifest") : *FString::Join(CaptureProductOverride, TEXT(",")));
}

void ABskSceneController::ConfigureFixedRgbExposure(USceneCaptureComponent2D* Capture, const FBskCameraDefinition* Definition) const
{
    if (!Capture) return;
    Capture->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
    Capture->ShowFlags.SetPostProcessing(true);
    Capture->ShowFlags.SetEyeAdaptation(true);
    Capture->ShowFlags.SetTonemapper(true);
    Capture->PostProcessBlendWeight = 1.0f;
    if (SavedEnvironment)
    {
        // Let the saved unbound volumes control dataset and preview exposure
        // together. Explicit per-camera exposure remains supported below.
        Capture->PostProcessSettings = FPostProcessSettings();
    }
    else
    {
        Capture->PostProcessSettings.bOverride_AutoExposureMethod = true;
        Capture->PostProcessSettings.AutoExposureMethod = EAutoExposureMethod::AEM_Manual;
        Capture->PostProcessSettings.bOverride_AutoExposureApplyPhysicalCameraExposure = true;
        Capture->PostProcessSettings.AutoExposureApplyPhysicalCameraExposure = true;
        Capture->PostProcessSettings.bOverride_CameraISO = true;
        Capture->PostProcessSettings.CameraISO = 100.0f;
        Capture->PostProcessSettings.bOverride_CameraShutterSpeed = true;
        Capture->PostProcessSettings.CameraShutterSpeed = 125.0f;
        Capture->PostProcessSettings.bOverride_DepthOfFieldFstop = true;
        Capture->PostProcessSettings.DepthOfFieldFstop = 16.0f;
        Capture->PostProcessSettings.bOverride_AutoExposureBias = true;
        Capture->PostProcessSettings.AutoExposureBias = static_cast<float>(MaterialExposureBias);
    }
    // Photometric exposure only: this does not integrate moving geometry over
    // the exposure interval or model physical sensor shutter timing.
    if (Definition && (Definition->ExposureTimeSeconds > 0.0 || Definition->ApertureFNumber > 0.0))
    {
        Capture->PostProcessSettings.bOverride_AutoExposureMethod = true;
        Capture->PostProcessSettings.AutoExposureMethod = EAutoExposureMethod::AEM_Manual;
        Capture->PostProcessSettings.bOverride_AutoExposureApplyPhysicalCameraExposure = true;
        Capture->PostProcessSettings.AutoExposureApplyPhysicalCameraExposure = true;
    }
    if (Definition && Definition->ExposureTimeSeconds > 0.0)
    {
        Capture->PostProcessSettings.bOverride_CameraShutterSpeed = true;
        Capture->PostProcessSettings.CameraShutterSpeed = static_cast<float>(1.0 / Definition->ExposureTimeSeconds);
    }
    if (Definition && Definition->ApertureFNumber > 0.0)
    {
        Capture->PostProcessSettings.bOverride_DepthOfFieldFstop = true;
        Capture->PostProcessSettings.DepthOfFieldFstop = static_cast<float>(Definition->ApertureFNumber);
    }
}

void ABskSceneController::CaptureRgbCamera(const FString& CameraId, USceneCaptureComponent2D* Capture)
{
    const FBskCameraDefinition* Definition = ManifestCameras.Find(CameraId);
    if (!Capture || !Capture->TextureTarget || !Definition) return;
    if (Definition->CameraMode == TEXT("rgb"))
    {
        CameraEffectSources.Remove(CameraId);
        Capture->CaptureScene();
        return;
    }
    // Keep the canonical output target shared by HUD, WebRTC, readback and policy.
    // Only the scene render writes to the private intermediate texture.
    UTextureRenderTarget2D* Output = Capture->TextureTarget;
    const bool bFisheye = Definition->IsFisheye();
    const int32 Side = FMath::Min(4096, FMath::Max(Output->SizeX, Output->SizeY) * 2);
    const FIntPoint Size = bFisheye ? FIntPoint(Side, Side) : FIntPoint(Output->SizeX, Output->SizeY);
    UTextureRenderTarget2D* Source = CameraEffectSources.FindRef(CameraId);
    if (!Source || Source->SizeX != Size.X || Source->SizeY != Size.Y)
    {
        Source = NewObject<UTextureRenderTarget2D>(this);
        Source->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8;
        Source->bAutoGenerateMips = false;
        Source->InitAutoFormat(Size.X, Size.Y);
        ConfigureRgbRenderTarget(Source);
        Source->UpdateResourceImmediate(true);
        CameraEffectSources.Add(CameraId, Source);
    }
    Capture->TextureTarget = Source;
    Capture->CaptureScene();
    Capture->TextureTarget = Output;
    BskApplyCameraEffect(Source, Output, bFisheye, Definition->IsMonochrome(),
        static_cast<float>(Definition->FieldOfViewRadians));
}

void ABskSceneController::ConfigureRgbRenderTarget(UTextureRenderTarget2D* Target) const
{
    if (!Target) return;
    Target->TargetGamma = 2.2f;
    Target->ClearColor = FLinearColor::Black;
}

AActor* ABskSceneController::SpawnCamera(const FBskCameraDefinition& Definition)
{
    if (UBskRenderWorldSubsystem* RenderSubsystem = GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>())
    {
        const FBskRenderSpawnContext Context{GetWorld(), this, &Converter};
        if (AActor* Actor = RenderSubsystem->TrySpawnCamera(Context, Definition)) return Actor;
    }

    FActorSpawnParameters Params;
    Params.Name = MakeUniqueObjectName(GetWorld(), ACameraActor::StaticClass(), SafeActorName(Definition.CameraId));
    ACameraActor* Camera = GetWorld()->SpawnActor<ACameraActor>(ACameraActor::StaticClass(), FTransform::Identity, Params);
    if (!Camera) return nullptr;
    Camera->Tags.AddUnique(FName(*FString::Printf(TEXT("BSK.Camera.%s"), *Definition.CameraId)));
    return Camera;
}

void ABskSceneController::ConfigureCamera(AActor* Actor, const FBskCameraDefinition& Definition)
{
    if (!Actor) return;
    const float FieldOfViewDegrees = static_cast<float>(FMath::RadiansToDegrees(Definition.FieldOfViewRadians));
    Actor->SetActorRelativeLocation(FVector(Converter.LocalMetersToUnrealCentimeters(Definition.PositionBodyMeters)));
    Actor->SetActorRelativeRotation(FQuat(Converter.ActiveLocalWxyzToUnreal(Definition.OrientationBodyFromCameraWxyz)));
    if (ACameraActor* CameraActor = Cast<ACameraActor>(Actor))
    {
        CameraActor->GetCameraComponent()->FieldOfView = FieldOfViewDegrees;
    }

    USceneCaptureComponent2D* Capture = CameraCaptureComponents.FindRef(Definition.CameraId);
    const bool bHasDataProducts = !CaptureProductOverride.IsEmpty() || !Definition.CaptureProducts.IsEmpty();
    const bool bPixelStreamingRequested = IsPixelStreamingCameraRequested(Definition.CameraId);
    if (!Definition.bPictureInPicture && !bHasDataProducts && !bPixelStreamingRequested)
    {
        if (Capture) Capture->Deactivate();
        CameraRenderTargets.Remove(Definition.CameraId);
        CameraEffectSources.Remove(Definition.CameraId);
        CameraNextCaptureSeconds.Remove(Definition.CameraId);
        CameraNextDataCaptureSimulationNanoseconds.Remove(Definition.CameraId);
        CameraPictureInPictureVisibility.Remove(Definition.CameraId);
        return;
    }

    if (!Capture)
    {
        Capture = NewObject<USceneCaptureComponent2D>(Actor, MakeUniqueObjectName(Actor, USceneCaptureComponent2D::StaticClass(), TEXT("BskRgbCapture")));
        if (!Capture) return;
        Capture->SetupAttachment(Actor->GetRootComponent());
        Capture->RegisterComponent();
        Capture->SetRelativeTransform(FTransform::Identity);
        Capture->bCaptureEveryFrame = false;
        Capture->bCaptureOnMovement = false;
        Capture->bAlwaysPersistRenderingState = true;
        Capture->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
        Capture->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_RenderScenePrimitives;
        CameraCaptureComponents.Add(Definition.CameraId, Capture);
    }
    ConfigureFixedRgbExposure(Capture, &Definition);
    Capture->bAlwaysPersistRenderingState = true;
    Capture->Activate();
    Capture->FOVAngle = FieldOfViewDegrees;

    UTextureRenderTarget2D* RenderTarget = CameraRenderTargets.FindRef(Definition.CameraId);
    if (!RenderTarget || RenderTarget->SizeX != Definition.Resolution.X || RenderTarget->SizeY != Definition.Resolution.Y)
    {
        RenderTarget = NewObject<UTextureRenderTarget2D>(this, MakeUniqueObjectName(this, UTextureRenderTarget2D::StaticClass(), TEXT("BskSharedCameraTarget")));
        RenderTarget->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8;
        RenderTarget->bAutoGenerateMips = false;
        RenderTarget->InitAutoFormat(Definition.Resolution.X, Definition.Resolution.Y);
        ConfigureRgbRenderTarget(RenderTarget);
        RenderTarget->UpdateResourceImmediate(true);
        CameraRenderTargets.Add(Definition.CameraId, RenderTarget);
    }
    Capture->TextureTarget = RenderTarget;
    if (Definition.bPictureInPicture && !CameraPictureInPictureVisibility.Contains(Definition.CameraId))
    {
        CameraPictureInPictureVisibility.Add(Definition.CameraId, true);
    }
    CameraNextCaptureSeconds.Add(Definition.CameraId, 0.0);
    CameraNextDataCaptureSimulationNanoseconds.Add(Definition.CameraId, 0);

    // Configure the standalone streamer only after the canonical capture and
    // target exist, so the HUD inset and selected camera are pixel-identical.
    ConfigurePixelStreamingCamera(Actor, Definition);
}

void ABskSceneController::UpdatePictureInPictureCaptures()
{
    // Offline capture renders extra views only while held idle before an episode.
    static const bool bOfflineCapture = FParse::Param(FCommandLine::Get(), TEXT("BskOfflineCapture"));
    if (bOfflineCapture)
    {
        UpdateOfflineCameraWarmup(FPlatformTime::Seconds());
        return;
    }
    if (!GetWorld()) return;
    const double NowSeconds = FPlatformTime::Seconds();
    for (const TPair<FString, FBskCameraDefinition>& Pair : ManifestCameras)
    {
        const FBskCameraDefinition& Definition = Pair.Value;
        if (!Definition.bPictureInPicture || !CameraPictureInPictureVisibility.FindRef(Pair.Key)) continue;
        USceneCaptureComponent2D* Capture = CameraCaptureComponents.FindRef(Pair.Key);
        if (!Capture || !Capture->TextureTarget) continue;
        double PreviewRate = PreviewRateOverrideHertz > 0.0 ? PreviewRateOverrideHertz : Definition.CaptureRateHertz;
        if (IsPixelStreamingCameraRequested(Pair.Key))
        {
            // Prioritize the view actually subscribed by a browser. Keep unselected
            // HUD thumbnails at <=15 Hz rather than rendering all three views at 90.
            PreviewRate = BskPreviewRateForViewers(PixelStreamingCameraRateHertz,
                HasPixelStreamingCameraViewers(Pair.Key), HasPixelStreamingViewportViewers());
            if (PreviewRate <= 0.0) continue;
        }
        double& NextSeconds = CameraNextCaptureSeconds.FindOrAdd(Pair.Key);
        if (!BskPreviewCaptureDue(NowSeconds, FMath::Max(1.0, PreviewRate), NextSeconds)) continue;
        CapturePreview(Pair.Key, Capture);
        if (bVideoDiagnostics) ++VideoDiagnosticsCaptures.FindOrAdd(Pair.Key);
        if (PreviewRateOverrideHertz > 0.0 && CaptureNetworkSender && bHasPresentationFrame)
        {
            FBskCaptureRequest Request;
            Request.CameraId = Pair.Key;
            Request.Channels.Add(EBskCaptureChannel::Rgb);
            Request.Resolution = Definition.Resolution;
            Request.SimulationTimeNanoseconds = PresentationFrame.SimulationTimeNanoseconds;
            Request.SourceWallTimeNanoseconds = PresentationFrame.WallTimeNanoseconds;
            Request.FrameId = PresentationFrame.FrameId;
            Request.OriginInertialMeters = PresentationFrame.OriginInertialMeters;
            Request.LocalFromInertial = PresentationFrame.LocalFromInertial;
            Request.Purpose = EBskCapturePurpose::Preview;
            Request.bWriteToDisk = false;
            Request.bReuseExistingRgbTarget = true;
            Request.bSendToNetwork = true;
            FString Error;
            UBskRenderWorldSubsystem* RenderSubsystem = GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>();
            if (!RenderSubsystem || !RenderSubsystem->RequestCapture(Request, Error))
            {
                UE_LOG(LogBskUnreal, Warning, TEXT("Preview capture for %s failed: %s"), *Pair.Key, *Error);
            }
        }
    }
}

void ABskSceneController::UpdateAuthoritativeDataProductCaptures(const FBskRenderFrame& AuthoritativeFrame)
{
    if (bEnvironmentPreviewOnly || (SavedEnvironment && !SavedEnvironment->bAllowBackendControl)) return;
    // Capability alone never triggers dataset readback; only tagged active frames do.
    if (!AuthoritativeFrame.bVisionCaptureRequested
        && AuthoritativeFrame.bCaptureOnDemand && AuthoritativeFrame.CaptureEpisodeId.IsEmpty()) return;
    if (!GetWorld() || (CaptureOutputDirectory.IsEmpty() && !CaptureNetworkSender)) return;
    UBskRenderWorldSubsystem* RenderSubsystem = GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>();
    if (!RenderSubsystem) return;
    TArray<FString> DueCameras;
    TMap<FString, FBskCaptureSelection> Selections;
    for (const TPair<FString, FBskCameraDefinition>& Pair : ManifestCameras)
    {
        FBskCaptureSelection Selection;
        if (!BskSelectCameraCapture(AuthoritativeFrame, Pair.Value,
            CaptureRateOverrideHertz, CaptureProductOverride, Selection)) continue;
        const int64 NextCaptureNanoseconds = CameraNextDataCaptureSimulationNanoseconds.FindRef(Pair.Key);
        if (AuthoritativeFrame.SimulationTimeNanoseconds < NextCaptureNanoseconds) continue;
        DueCameras.Add(Pair.Key);
        Selections.Add(Pair.Key, MoveTemp(Selection));
    }
    if (AuthoritativeFrame.bVisionCaptureRequested)
    {
        ++VisionCaptureFramesRequested;
        for (const FString& CameraId : AuthoritativeFrame.VisionCaptureCameraIds)
        {
            const auto* Definition = ManifestCameras.Find(CameraId);
            if (!Definition || Definition->CaptureConsumer != TEXT("vision"))
            {
                ++VisionCaptureErrors;
                UE_LOG(LogBskUnreal, Error, TEXT("Vision camera is missing or not a vision consumer: %s source_frame=%lld"),
                    *CameraId, AuthoritativeFrame.FrameId);
            }
        }
        if (VisionCaptureFramesRequested == 1 || VisionCaptureFramesRequested % 100 == 0)
            UE_LOG(LogBskUnreal, Display, TEXT("VisionCapture session=%s requested_frames=%lld queued_packets=%lld errors=%lld source_frame=%lld sim_time_ns=%lld"),
                *ActiveSessionId, VisionCaptureFramesRequested, VisionCapturePacketsQueued, VisionCaptureErrors,
                AuthoritativeFrame.FrameId, AuthoritativeFrame.SimulationTimeNanoseconds);
    }
    if (DueCameras.IsEmpty()) return;

    // Snapshot capture and smooth presentation share one UE world, so apply the
    // exact source frame for the whole capture pass, then restore the
    // presentation frame. This never writes data back to BSK/MJScene.
    ApplyFrame(AuthoritativeFrame, false);

    const auto BuildCaptureRequest = [&](const FString& CameraId, const FBskCameraDefinition& Definition,
                                         bool bRenderOnly)
    {
        FBskCaptureRequest Request;
        Request.CameraId = CameraId;
        Request.Resolution = Definition.Resolution;
        Request.SimulationTimeNanoseconds = AuthoritativeFrame.SimulationTimeNanoseconds;
        Request.SourceWallTimeNanoseconds = AuthoritativeFrame.WallTimeNanoseconds;
        Request.FrameId = AuthoritativeFrame.FrameId;
        const FBskCaptureSelection& Selection = Selections.FindChecked(CameraId);
        Request.CaptureEpisodeId = Selection.Purpose == EBskCapturePurpose::AuthoritativeVision
            ? FString() : AuthoritativeFrame.CaptureEpisodeId;
        Request.OriginInertialMeters = AuthoritativeFrame.OriginInertialMeters;
        Request.LocalFromInertial = AuthoritativeFrame.LocalFromInertial;
        Request.Purpose = Selection.Purpose;
        Request.OutputDirectory = CaptureOutputDirectory;
        Request.bWriteToDisk = Selection.Purpose == EBskCapturePurpose::AuthoritativeDataset && !CaptureOutputDirectory.IsEmpty();
        Request.bSendToNetwork = CaptureNetworkSender.IsValid();
        Request.bSkipReadback = bRenderOnly;
        Request.bReuseExistingRgbTarget = !bRenderOnly;
        for (const FString& Product : Selection.Products)
        {
            if (Product == TEXT("rgb")) Request.Channels.Add(EBskCaptureChannel::Rgb);
        }
        return Request;
    };

    // Submit every due camera before reading them individually. This avoids
    // alternating render/readback submissions. The authoritative path normally
    // queues GPU copies; the diagnostic synchronous path uses ReadPixels.
    TArray<FString> RenderedCameras;
    for (const FString& CameraId : DueCameras)
    {
        const FBskCameraDefinition* Definition = ManifestCameras.Find(CameraId);
        if (!Definition) continue;
        FBskCaptureRequest Request = BuildCaptureRequest(CameraId, *Definition, true);
        if (Request.Channels.IsEmpty()) continue;
        FString Error;
        if (!RenderSubsystem->RequestCapture(Request, Error))
        {
            UE_LOG(LogBskUnreal, Error, TEXT("Authoritative capture render for %s frame=%lld failed: %s"),
                *CameraId, AuthoritativeFrame.FrameId, *Error);
            ReportCaptureFailure(Request, Error);
            continue;
        }
        RenderedCameras.Add(CameraId);
    }
    for (const FString& CameraId : RenderedCameras)
    {
        const FBskCameraDefinition* Definition = ManifestCameras.Find(CameraId);
        if (!Definition) continue;
        FBskCaptureRequest Request = BuildCaptureRequest(CameraId, *Definition, false);
        if (Request.Channels.IsEmpty()) continue;
        FString Error;
        if (!RenderSubsystem->RequestCapture(Request, Error))
        {
            UE_LOG(LogBskUnreal, Error, TEXT("Authoritative capture for %s frame=%lld failed: %s"),
                *CameraId, AuthoritativeFrame.FrameId, *Error);
            ReportCaptureFailure(Request, Error);
        }
        CameraNextDataCaptureSimulationNanoseconds.Add(CameraId, AuthoritativeFrame.SimulationTimeNanoseconds + 1);
    }
    if (bHasPresentationFrame) ApplyFrame(PresentationFrame, false);
}

bool ABskSceneController::CaptureCameraDataProducts(const FBskCaptureRequest& Request, FString& OutError)
{
    if (bEnvironmentPreviewOnly || (SavedEnvironment && !SavedEnvironment->bAllowBackendControl))
    {
        OutError = TEXT("authoritative capture is disabled in independent environment preview");
        return false;
    }
    check(IsInGameThread());
    const bool bVision = Request.Purpose == EBskCapturePurpose::AuthoritativeVision;
    const bool bAuthoritative = Request.Purpose != EBskCapturePurpose::Preview;
    const FBskCameraDefinition* Definition = ManifestCameras.Find(Request.CameraId);
    AActor* CameraActor = CameraActors.FindRef(Request.CameraId);
    if (!Definition || !CameraActor)
    {
        OutError = FString::Printf(TEXT("camera '%s' is not registered in the active manifest"), *Request.CameraId);
        return false;
    }
    if (Request.Channels.IsEmpty())
    {
        OutError = TEXT("capture request contains no channels");
        return false;
    }
    if (!Request.bWriteToDisk && !Request.bSendToNetwork)
    {
        OutError = TEXT("capture request has neither disk nor network output enabled");
        return false;
    }
    const FIntPoint Resolution(
        FMath::Clamp(Request.Resolution.X, 64, 4096),
        FMath::Clamp(Request.Resolution.Y, 64, 4096));
    const float FieldOfViewDegrees = static_cast<float>(FMath::RadiansToDegrees(Definition->FieldOfViewRadians));

    const auto EnsureCapture = [&](TMap<FString, TObjectPtr<USceneCaptureComponent2D>>& Components,
                                   TMap<FString, TObjectPtr<UTextureRenderTarget2D>>& Targets,
                                   const TCHAR* ComponentName, ETextureRenderTargetFormat Format,
                                   ESceneCaptureSource Source) -> USceneCaptureComponent2D*
    {
        USceneCaptureComponent2D* Capture = Components.FindRef(Request.CameraId);
        if (!Capture)
        {
            Capture = NewObject<USceneCaptureComponent2D>(CameraActor,
                MakeUniqueObjectName(CameraActor, USceneCaptureComponent2D::StaticClass(), FName(ComponentName)));
            if (!Capture) return nullptr;
            Capture->SetupAttachment(CameraActor->GetRootComponent());
            Capture->RegisterComponent();
            Capture->SetRelativeTransform(FTransform::Identity);
            Capture->bCaptureEveryFrame = false;
            Capture->bCaptureOnMovement = false;
            Capture->bAlwaysPersistRenderingState = false;
            Capture->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_RenderScenePrimitives;
            Components.Add(Request.CameraId, Capture);
        }
        Capture->CaptureSource = Source;
        if (Source == ESceneCaptureSource::SCS_FinalColorLDR) ConfigureFixedRgbExposure(Capture, Definition);
        Capture->FOVAngle = FieldOfViewDegrees;
        Capture->Activate();
        UTextureRenderTarget2D* Target = Targets.FindRef(Request.CameraId);
        if (!Target || Target->SizeX != Resolution.X || Target->SizeY != Resolution.Y || Target->RenderTargetFormat != Format)
        {
            Target = NewObject<UTextureRenderTarget2D>(this,
                MakeUniqueObjectName(this, UTextureRenderTarget2D::StaticClass(), FName(*FString::Printf(TEXT("%sTarget"), ComponentName))));
            Target->RenderTargetFormat = Format;
            Target->ClearColor = FLinearColor::Black;
            Target->bAutoGenerateMips = false;
            Target->InitAutoFormat(Resolution.X, Resolution.Y);
            if (Format == ETextureRenderTargetFormat::RTF_RGBA8) ConfigureRgbRenderTarget(Target);
            Target->UpdateResourceImmediate(true);
            Targets.Add(Request.CameraId, Target);
        }
        Capture->TextureTarget = Target;
        return Capture;
    };

    if (Request.bSkipReadback)
    {
        // Render-only phase for one camera. The matching readback request runs
        // only after every due camera has been rendered.
        for (const EBskCaptureChannel Channel : Request.Channels)
        {
            if (Channel != EBskCaptureChannel::Rgb)
            {
                OutError = TEXT("only RGB capture is supported");
                return false;
            }
            USceneCaptureComponent2D* Capture = EnsureCapture(
                CameraCaptureComponents, CameraRenderTargets, TEXT("BskRgbCapture"),
                ETextureRenderTargetFormat::RTF_RGBA8, ESceneCaptureSource::SCS_FinalColorLDR);
            if (!Capture || !Capture->TextureTarget)
            {
                OutError = TEXT("could not create RGB capture resources");
                return false;
            }
            CaptureRgbCamera(Request.CameraId, Capture);
        }
        return true;
    }

    TArray<FCapturedDataProduct> Products;
    TSharedPtr<FBskAsyncCaptureJob, ESPMode::ThreadSafe> AsyncJob;
    for (const EBskCaptureChannel Channel : Request.Channels)
    {
        if (Channel == EBskCaptureChannel::Rgb)
        {
            USceneCaptureComponent2D* Capture = EnsureCapture(
                CameraCaptureComponents, CameraRenderTargets, TEXT("BskRgbCapture"),
                ETextureRenderTargetFormat::RTF_RGBA8, ESceneCaptureSource::SCS_FinalColorLDR);
            if (!Capture || !Capture->TextureTarget)
            {
                OutError = TEXT("could not create RGB capture resources");
                return false;
            }
            if (!Request.bReuseExistingRgbTarget) CaptureRgbCamera(Request.CameraId, Capture);
            if (bAsyncCaptureEnabled && bAuthoritative)
            {
                const uint64 Bytes = static_cast<uint64>(Resolution.X) * Resolution.Y * sizeof(FColor) + 65536;
                if (PendingCaptureJobs.Num() >= MaxAsyncCaptureJobs || PendingCaptureBytes + Bytes > MaxAsyncCaptureBytes)
                {
                    OutError = TEXT("async capture admission was not reserved");
                    return false;
                }
                FTextureRenderTargetResource* Resource = Capture->TextureTarget->GameThread_GetRenderTargetResource();
                if (!Resource)
                {
                    OutError = TEXT("async RGB readback requires a render-target resource");
                    return false;
                }
                AsyncJob = MakeShared<FBskAsyncCaptureJob, ESPMode::ThreadSafe>();
                AsyncJob->Request = Request;
                AsyncJob->Resolution = Resolution;
                AsyncJob->ReservedBytes = Bytes;
                AsyncJob->SubmittedSeconds = FPlatformTime::Seconds();
                AsyncJob->Readback = MakeShared<FRHIGPUTextureReadback, ESPMode::ThreadSafe>(TEXT("BskStrictRgb"));
                // Load the module/create the wrapper here, never from a worker.
                AsyncJob->Wrapper = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper")).CreateImageWrapper(
                    bVision ? EImageFormat::PNG : EImageFormat::JPEG);
                // Resolve the RHI texture after the queued initialization and
                // scene capture. Reading it on the game thread races frame 0.
                // Resource release is also ordered through the render queue.
                ENQUEUE_RENDER_COMMAND(BskEnqueueRgbReadback)([Job = AsyncJob, Resource](FRHICommandListImmediate& RHICmdList)
                {
                    FTextureRHIRef Texture = Resource->GetRenderTargetTexture();
                    if (!Texture || (Texture->GetFormat() != PF_B8G8R8A8 && Texture->GetFormat() != PF_R8G8B8A8))
                    {
                        Job->Error = TEXT("async RGB readback requires an initialized 8-bit BGRA/RGBA render target");
                    }
                    else
                    {
                        Job->bRgba = Texture->GetFormat() == PF_R8G8B8A8;
                        Job->Readback->EnqueueCopy(RHICmdList, Texture);
                    }
                    Job->bCopySubmitted.Store(true);
                });
                continue;
            }
            TArray<FColor> Pixels;
            FReadSurfaceDataFlags ReadFlags(RCM_UNorm);
            ReadFlags.SetLinearToGamma(false);
            if (!Capture->TextureTarget->GameThread_GetRenderTargetResource()->ReadPixels(Pixels, ReadFlags) || Pixels.Num() != Resolution.X * Resolution.Y)
            {
                OutError = TEXT("RGB render-target readback failed");
                return false;
            }
            const bool bPreview = Request.Purpose == EBskCapturePurpose::Preview;
            // Vision preserves rendered pixels in PNG. Existing training and
            // preview consumers retain their lossy JPEG formats unchanged.
            const int32 JpegQuality = bPreview ? 75 : 95;
            FCapturedDataProduct Product{TEXT("rgb"),
                bVision ? FString(TEXT("png/bgra8_srgb")) : FString::Printf(TEXT("jpeg/bgra8_srgb/quality%d"), JpegQuality),
                bVision ? TEXT("png") : TEXT("jpg")};
            const bool bCompressed = bVision
                ? CompressPng(Pixels, Resolution.X, Resolution.Y, Product.Bytes)
                : CompressJpeg(Pixels, Resolution.X, Resolution.Y, JpegQuality, Product.Bytes);
            if (!bCompressed)
            {
                OutError = TEXT("RGB image encoding failed");
                return false;
            }
            Products.Add(MoveTemp(Product));
        }
        else
        {
            OutError = TEXT("only RGB capture is supported");
            return false;
        }
    }

    const int64 Sequence = ++CaptureSequence;
    const int64 CaptureWallTimeNanoseconds = UnixTimeNanoseconds();
    TSharedPtr<FJsonObject> Metadata = MakeShared<FJsonObject>();
    Metadata->SetStringField(TEXT("protocol"), TEXT("bsk-capture/1"));
    Metadata->SetStringField(TEXT("type"), TEXT("camera_frame"));
    Metadata->SetStringField(TEXT("session_id"), ActiveSessionId);
    Metadata->SetStringField(TEXT("camera_id"), Request.CameraId);
    Metadata->SetStringField(TEXT("camera_mode"), Definition->CameraMode);
    Metadata->SetStringField(TEXT("color_model"), Definition->IsMonochrome() ? TEXT("monochrome_rgb") : TEXT("rgb"));
    Metadata->SetStringField(TEXT("capture_episode_id"), Request.CaptureEpisodeId);
    Metadata->SetBoolField(TEXT("ack_required"), bAuthoritative);
    Metadata->SetStringField(TEXT("stream_kind"),
        bVision ? TEXT("vision") : (bAuthoritative ? TEXT("authoritative") : TEXT("preview")));
    Metadata->SetStringField(TEXT("state_kind"),
        bAuthoritative ? TEXT("authoritative") : TEXT("presentation"));
    if (Request.Purpose == EBskCapturePurpose::AuthoritativeDataset)
    {
        const double DatasetRate = CaptureRateOverrideHertz > 0.0 ? CaptureRateOverrideHertz : Definition->CaptureRateHertz;
        int64 DatasetSampleIndex = 0;
        if (!BskLeRobotSample(Request.SimulationTimeNanoseconds, DatasetRate, DatasetSampleIndex))
        {
            OutError = TEXT("authoritative capture is outside the LeRobot sampling grid");
            return false;
        }
        Metadata->SetStringField(TEXT("dataset_format"), TEXT("lerobot-v3"));
        Metadata->SetNumberField(TEXT("sampling_fps"), DatasetRate);
        Metadata->SetStringField(TEXT("sample_index"), LexToString(DatasetSampleIndex));
        Metadata->SetStringField(TEXT("sampling_clock"), TEXT("simulation"));
    }
    if (bVision)
    {
        Metadata->SetNumberField(TEXT("sampling_fps"), Definition->CaptureRateHertz);
        Metadata->SetStringField(TEXT("sampling_clock"), TEXT("simulation"));
        Metadata->SetStringField(TEXT("sensor_model"), TEXT("render_pinhole"));
        Metadata->SetBoolField(TEXT("temporal_exposure_integration"), false);
        Metadata->SetStringField(TEXT("exposure_semantics"), TEXT("render_photometric_only"));
        Metadata->SetNumberField(TEXT("exposure_time_s"), Definition->ExposureTimeSeconds);
        Metadata->SetNumberField(TEXT("aperture_fnumber"), Definition->ApertureFNumber);
    }
    Metadata->SetStringField(TEXT("capture_sequence"), LexToString(Sequence));
    Metadata->SetStringField(TEXT("source_frame_id"), LexToString(Request.FrameId));
    Metadata->SetStringField(TEXT("sim_time_ns"), LexToString(Request.SimulationTimeNanoseconds));
    Metadata->SetStringField(TEXT("source_wall_time_ns"), LexToString(Request.SourceWallTimeNanoseconds));
    Metadata->SetStringField(TEXT("capture_wall_time_ns"), LexToString(CaptureWallTimeNanoseconds));
    Metadata->SetArrayField(TEXT("resolution"), {
        MakeShared<FJsonValueNumber>(Resolution.X), MakeShared<FJsonValueNumber>(Resolution.Y)});

    const double FovX = Definition->FieldOfViewRadians;
    const bool bFisheye = Definition->IsFisheye();
    const double Fx = bFisheye ? FMath::Min(Resolution.X, Resolution.Y) / FovX
        : 0.5 * Resolution.X / FMath::Tan(0.5 * FovX);
    const double Fy = Fx;
    const double FovY = bFisheye ? FovX : 2.0 * FMath::Atan(0.5 * Resolution.Y / Fy);
    TSharedPtr<FJsonObject> Intrinsics = MakeShared<FJsonObject>();
    Intrinsics->SetStringField(TEXT("model"), bFisheye ? TEXT("equidistant_fisheye") : TEXT("pinhole"));
    Intrinsics->SetStringField(TEXT("distortion_model"), bFisheye ? TEXT("equidistant") : TEXT("none"));
    if (bFisheye) Intrinsics->SetNumberField(TEXT("valid_circle_radius_px"), 0.5 * FMath::Min(Resolution.X, Resolution.Y));
    Intrinsics->SetNumberField(TEXT("fx_px"), Fx);
    Intrinsics->SetNumberField(TEXT("fy_px"), Fy);
    Intrinsics->SetNumberField(TEXT("cx_px"), 0.5 * (Resolution.X - 1));
    Intrinsics->SetNumberField(TEXT("cy_px"), 0.5 * (Resolution.Y - 1));
    Intrinsics->SetNumberField(TEXT("fov_x_rad"), FovX);
    Intrinsics->SetNumberField(TEXT("fov_y_rad"), FovY);
    Metadata->SetObjectField(TEXT("intrinsics"), Intrinsics);

    const FVector3d UnrealPositionCentimeters(CameraActor->GetActorLocation());
    const double MirrorY = Converter.MirrorsLocalY() ? -1.0 : 1.0;
    const FVector3d PositionLocalMeters(
        UnrealPositionCentimeters.X / Converter.GetCentimetersPerMeter(),
        MirrorY * UnrealPositionCentimeters.Y / Converter.GetCentimetersPerMeter(),
        UnrealPositionCentimeters.Z / Converter.GetCentimetersPerMeter());
    const FQuat4d LocalFromCamera = UnrealToActiveLocalWxyz(FQuat4d(CameraActor->GetActorQuat()), Converter.MirrorsLocalY());
    double LocalFromCameraMatrix[3][3];
    QuaternionMatrix(LocalFromCamera, LocalFromCameraMatrix);
    double InertialFromCameraMatrix[3][3];
    for (int32 Row = 0; Row < 3; ++Row) for (int32 Column = 0; Column < 3; ++Column)
    {
        InertialFromCameraMatrix[Row][Column] = LocalFromCameraMatrix[Row][Column];
    }
    FVector3d PositionInertialMeters = PositionLocalMeters;
    const FVector3d OriginInertialMeters = Request.OriginInertialMeters;
    double LocalFromInertialMatrix[3][3];
    for (int32 Row = 0; Row < 3; ++Row) for (int32 Column = 0; Column < 3; ++Column)
    {
        LocalFromInertialMatrix[Row][Column] = Request.LocalFromInertial.M[Row][Column];
    }
    for (int32 Row = 0; Row < 3; ++Row)
    {
        PositionInertialMeters[Row] = OriginInertialMeters[Row];
        for (int32 K = 0; K < 3; ++K) PositionInertialMeters[Row] += LocalFromInertialMatrix[K][Row] * PositionLocalMeters[K];
        for (int32 Column = 0; Column < 3; ++Column)
        {
            InertialFromCameraMatrix[Row][Column] = 0.0;
            for (int32 K = 0; K < 3; ++K) InertialFromCameraMatrix[Row][Column] += LocalFromInertialMatrix[K][Row] * LocalFromCameraMatrix[K][Column];
        }
    }
    TSharedPtr<FJsonObject> Extrinsics = MakeShared<FJsonObject>();
    Extrinsics->SetStringField(TEXT("camera_axes"), TEXT("+X_forward,+Y_left,+Z_up"));
    Extrinsics->SetArrayField(TEXT("position_L_m"), JsonVector(PositionLocalMeters));
    Extrinsics->SetArrayField(TEXT("q_LC_wxyz"), JsonQuaternionWxyz(LocalFromCamera));
    Extrinsics->SetArrayField(TEXT("c_LC"), JsonMatrix3(LocalFromCameraMatrix));
    Extrinsics->SetArrayField(TEXT("position_N_m"), JsonVector(PositionInertialMeters));
    Extrinsics->SetArrayField(TEXT("c_NC"), JsonMatrix3(InertialFromCameraMatrix));
    Metadata->SetObjectField(TEXT("extrinsics"), Extrinsics);
    TSharedPtr<FJsonObject> FloatingOrigin = MakeShared<FJsonObject>();
    FloatingOrigin->SetArrayField(TEXT("origin_N_m"), JsonVector(OriginInertialMeters));
    FloatingOrigin->SetArrayField(TEXT("c_LN"), JsonMatrix3(LocalFromInertialMatrix));
    Metadata->SetObjectField(TEXT("floating_origin"), FloatingOrigin);

    if (AsyncJob)
    {
        AsyncJob->Sequence = Sequence;
        AsyncJob->Metadata = Metadata; // Frozen BEFORE presentation is restored.
        PendingCaptureBytes += AsyncJob->ReservedBytes;
        PendingCaptureJobs.Add(MoveTemp(AsyncJob));
        return true;
    }
    return WriteCaptureProducts(Request, Metadata, Products, Sequence, OutError);
}

bool ABskSceneController::WriteCaptureProducts(const FBskCaptureRequest& Request,
    const TSharedPtr<FJsonObject>& Metadata, const TArray<FCapturedDataProduct>& Products,
    int64 Sequence, FString& OutError)
{
    Metadata->SetNumberField(TEXT("render_queue_frames"), Receiver ? Receiver->GetPendingFrameCount() : 0);
    Metadata->SetNumberField(TEXT("image_send_queue_packets"), CaptureNetworkSender ? CaptureNetworkSender->GetPendingPacketCount() : 0);

    TArray<TSharedPtr<FJsonValue>> ProductMetadata;
    uint64 BlobOffset = 0;
    const FString Stem = FString::Printf(TEXT("%012lld_%s"), Sequence, *SafePathSegment(Request.CameraId));
    for (const FCapturedDataProduct& Product : Products)
    {
        TSharedPtr<FJsonObject> ProductObject = MakeShared<FJsonObject>();
        ProductObject->SetStringField(TEXT("name"), Product.Name);
        ProductObject->SetStringField(TEXT("encoding"), Product.Encoding);
        ProductObject->SetStringField(TEXT("file_name"), FString::Printf(TEXT("%s_%s.%s"), *Stem, *Product.Name, *Product.Extension));
        ProductObject->SetStringField(TEXT("blob_offset"), LexToString(BlobOffset));
        ProductObject->SetStringField(TEXT("byte_length"), LexToString(Product.Bytes.Num()));
        ProductMetadata.Add(MakeShared<FJsonValueObject>(ProductObject));
        BlobOffset += Product.Bytes.Num();
    }
    Metadata->SetArrayField(TEXT("products"), ProductMetadata);
    FString MetadataJson;
    if (!FJsonSerializer::Serialize(Metadata.ToSharedRef(), TJsonWriterFactory<>::Create(&MetadataJson)))
    {
        OutError = TEXT("capture metadata JSON serialization failed");
        return false;
    }
    FTCHARToUTF8 MetadataUtf8(*MetadataJson);

    if (Request.bSendToNetwork)
    {
        if (!CaptureNetworkSender)
        {
            OutError = TEXT("capture network output was requested but no output target is configured");
            return false;
        }
        const uint64 PayloadSize = 4ull + MetadataUtf8.Length() + BlobOffset;
        if (PayloadSize > MAX_uint32)
        {
            OutError = TEXT("capture network packet exceeds the 4 GiB protocol limit");
            return false;
        }
        TArray<uint8> Packet;
        Packet.Reserve(static_cast<int32>(PayloadSize + 4));
        AddBigEndianUint32(Packet, static_cast<uint32>(PayloadSize));
        AddBigEndianUint32(Packet, static_cast<uint32>(MetadataUtf8.Length()));
        Packet.Append(reinterpret_cast<const uint8*>(MetadataUtf8.Get()), MetadataUtf8.Length());
        for (const FCapturedDataProduct& Product : Products) Packet.Append(Product.Bytes);
        if (Request.Purpose != EBskCapturePurpose::Preview)
        {
            if (!CaptureNetworkSender->EnqueueReliable(MoveTemp(Packet)))
            {
                OutError = TEXT("authoritative capture admission failed; output capacity must be reserved before consuming the source frame");
                return false;
            }
            if (Request.Purpose == EBskCapturePurpose::AuthoritativeVision
                && Metadata->GetStringField(TEXT("type")) == TEXT("camera_frame")) ++VisionCapturePacketsQueued;
        }
        else
        {
            CaptureNetworkSender->EnqueueLatest(FString::Printf(TEXT("preview:%s"), *Request.CameraId), MoveTemp(Packet));
        }
    }

    if (Request.bWriteToDisk)
    {
        FString Directory = Request.OutputDirectory.IsEmpty() ? CaptureOutputDirectory : Request.OutputDirectory;
        if (Directory.IsEmpty())
        {
            OutError = TEXT("capture disk output was requested without an output directory");
            return false;
        }
        Directory = FPaths::Combine(Directory, SafePathSegment(Metadata->GetStringField(TEXT("session_id"))), SafePathSegment(Request.CameraId));
        if (!CaptureDiskWriter)
        {
            OutError = TEXT("capture disk output was requested but no disk writer is configured");
            return false;
        }
        FBskCaptureDiskWork Work;
        Work.Request = Request;
        Work.SessionId = Metadata->GetStringField(TEXT("session_id"));
        Work.Sequence = Sequence;
        Work.Directory = MoveTemp(Directory);
        Work.Stem = Stem;
        Work.MetadataJson = MetadataJson;
        Work.Products = Products;
        if (!CaptureDiskWriter->EnqueueReliable(MoveTemp(Work)))
        {
            OutError = TEXT("authoritative capture disk queue is full; frame was not silently replaced");
            return false;
        }
    }
    return true;
}

void ABskSceneController::PollCaptureJobs()
{
    check(IsInGameThread());
    for (const auto& Job : PendingCaptureJobs)
    {
        if (!Job->Readback || !Job->bCopySubmitted.Load()) continue;
        // The submission flag publishes the render-thread result, including
        // failures that have no GPU fence to poll.
        if (!Job->Error.IsEmpty())
        {
            Job->Readback.Reset();
            continue;
        }
        if (!Job->Readback->IsReady()) continue;
        int32 Pitch = 0, Height = 0;
        const FColor* Source = static_cast<const FColor*>(Job->Readback->Lock(Pitch, &Height));
        TArray<FColor> Pixels;
        if (Source && Pitch >= Job->Resolution.X && Height >= Job->Resolution.Y)
        {
            Pixels.SetNumUninitialized(Job->Resolution.X * Job->Resolution.Y);
            for (int32 Y = 0; Y < Job->Resolution.Y; ++Y)
                FMemory::Memcpy(Pixels.GetData() + Y * Job->Resolution.X, Source + Y * Pitch,
                                Job->Resolution.X * sizeof(FColor));
            if (Job->bRgba) for (FColor& Pixel : Pixels) Swap(Pixel.R, Pixel.B);
        }
        if (Source) Job->Readback->Unlock();
        Job->Readback.Reset();
        Job->Metadata->SetStringField(TEXT("readback_completed_wall_time_ns"), LexToString(UnixTimeNanoseconds()));
        if (Pixels.IsEmpty())
        {
            Job->Error = TEXT("GPU RGB readback returned invalid pitch/height/data");
            continue;
        }
        // Independent image wrapper and owned pixels only; no scene/UObject
        // access in the worker. Bound parallel work by MaxAsyncCaptureJobs.
        Job->Encoded = Async(EAsyncExecution::ThreadPool,
            [Pixels = MoveTemp(Pixels), Wrapper = MoveTemp(Job->Wrapper), Size = Job->Resolution,
             bVision = Job->Request.Purpose == EBskCapturePurpose::AuthoritativeVision]() mutable
            {
                FCapturedDataProduct Product{TEXT("rgb"),
                    bVision ? TEXT("png/bgra8_srgb") : TEXT("jpeg/bgra8_srgb/quality95"),
                    bVision ? TEXT("png") : TEXT("jpg")};
                if (Wrapper && Wrapper->SetRaw(Pixels.GetData(), Pixels.Num() * sizeof(FColor),
                                               Size.X, Size.Y, ERGBFormat::BGRA, 8))
                {
                    const TArray64<uint8>& Bytes = Wrapper->GetCompressed(bVision ? -3 : 95);
                    if (Bytes.Num() <= MAX_int32) Product.Bytes.Append(Bytes.GetData(), static_cast<int32>(Bytes.Num()));
                }
                return Product;
            });
    }
    // Workers may finish out of order. Commit in submission order so the
    // backend's per-camera sequences and frame-pairing semantics are unchanged.
    while (!PendingCaptureJobs.IsEmpty())
    {
        const auto Job = PendingCaptureJobs[0];
        if (Job->Readback) break;
        if (Job->Encoded.IsValid())
        {
            if (!Job->Encoded.IsReady()) break;
            FCapturedDataProduct Product = Job->Encoded.Get();
            Job->Encoded = TFuture<FCapturedDataProduct>();
            if (Product.Bytes.IsEmpty()) Job->Error = TEXT("background image encoding failed");
            else Job->Products.Add(MoveTemp(Product));
        }
        uint64 Bytes = 65536;
        for (const auto& Product : Job->Products) Bytes += Product.Bytes.Num();
        if ((Job->Request.bSendToNetwork && CaptureNetworkSender
                && !CaptureNetworkSender->HasReliableCapacity(1, Bytes))
            || (Job->Request.bWriteToDisk && CaptureDiskWriter && !CaptureDiskWriter->HasCapacity(1))) break;
        Job->Metadata->SetStringField(TEXT("capture_wall_time_ns"), LexToString(UnixTimeNanoseconds()));
        Job->Metadata->SetNumberField(TEXT("capture_pipeline_ms"), (FPlatformTime::Seconds() - Job->SubmittedSeconds) * 1000.0);
        Job->Metadata->SetNumberField(TEXT("capture_pending_jobs"), PendingCaptureJobs.Num());
        if (!Job->Error.IsEmpty())
        {
            if (Job->Request.Purpose == EBskCapturePurpose::AuthoritativeVision) ++VisionCaptureErrors;
            Job->Metadata->SetStringField(TEXT("type"), TEXT("capture_error"));
            Job->Metadata->SetStringField(TEXT("error"), Job->Error);
        }
        FString Error;
        if (!WriteCaptureProducts(Job->Request, Job->Metadata, Job->Products, Job->Sequence, Error))
        {
            UE_LOG(LogBskUnreal, Error, TEXT("Async capture frame=%lld failed: %s"), Job->Request.FrameId, *Error);
            ReportCaptureFailure(Job->Request, Error);
        }
        PendingCaptureBytes -= Job->ReservedBytes;
        PendingCaptureJobs.RemoveAt(0, 1, EAllowShrinking::No);
    }
}

void ABskSceneController::CancelCaptureJobs()
{
    // Only session reset/renderer teardown cancels jobs, never normal STOP.
    // Wait for queued RHI commands before releasing their resources.
    if (!PendingCaptureJobs.IsEmpty()) FlushRenderingCommands();
    for (const auto& Job : PendingCaptureJobs)
        if (Job->Encoded.IsValid()) Job->Encoded.Wait();
    PendingCaptureJobs.Reset();
    PendingCaptureBytes = 0;
}

void ABskSceneController::PollCaptureDiskResults()
{
    check(IsInGameThread());
    if (!CaptureDiskWriter) return;
    FBskCaptureDiskResult Result;
    while (CaptureDiskWriter->ConsumeResult(Result))
    {
        if (Result.bSucceeded)
        {
            ++CaptureDiskCompletedCount;
            UE_LOG(LogBskUnreal, Verbose, TEXT("BSK disk capture completed session=%s sequence=%lld marker=%s"),
                *Result.SessionId, Result.Sequence, *Result.CompletionPath);
        }
        else
        {
            ++CaptureDiskFailureCount;
            ReportCaptureFailure(Result.Request, Result.Error, Result.SessionId);
        }
    }
}

void ABskSceneController::ReportCaptureFailure(const FBskCaptureRequest& Request, const FString& Error,
    const FString& SessionId)
{
    const bool bVision = Request.Purpose == EBskCapturePurpose::AuthoritativeVision;
    if (bVision) ++VisionCaptureErrors;
    if (!Request.bSendToNetwork || !CaptureNetworkSender) return;
    auto Metadata = MakeShared<FJsonObject>();
    const int64 Sequence = ++CaptureSequence;
    Metadata->SetStringField(TEXT("protocol"), TEXT("bsk-capture/1"));
    Metadata->SetStringField(TEXT("type"), TEXT("capture_error"));
    Metadata->SetStringField(TEXT("session_id"), SessionId.IsEmpty() ? ActiveSessionId : SessionId);
    Metadata->SetStringField(TEXT("capture_episode_id"), Request.CaptureEpisodeId);
    Metadata->SetStringField(TEXT("camera_id"), Request.CameraId);
    Metadata->SetStringField(TEXT("capture_sequence"), LexToString(Sequence));
    Metadata->SetStringField(TEXT("source_frame_id"), LexToString(Request.FrameId));
    Metadata->SetStringField(TEXT("sim_time_ns"), LexToString(Request.SimulationTimeNanoseconds));
    Metadata->SetStringField(TEXT("stream_kind"), bVision ? TEXT("vision") : TEXT("authoritative"));
    Metadata->SetStringField(TEXT("state_kind"), TEXT("authoritative"));
    Metadata->SetBoolField(TEXT("ack_required"), true);
    Metadata->SetStringField(TEXT("error"), Error);
    FString OutputError;
    WriteCaptureProducts(Request, Metadata, {}, Sequence, OutputError);
}

int32 ABskSceneController::UpdateOfflineCameraWarmup(double NowSeconds)
{
    // Idle wall-clock warmup must submit real views: otherwise on-demand
    // shaders/PSOs first see these materials in the first recorded images.
    // Rendering the held presentation pose does not advance source state or
    // enter the dataset readback, disk, or network paths.
    if (!GetWorld() || !bHasPresentationFrame || !PresentationFrame.bCaptureOnDemand ||
        !PresentationFrame.CaptureEpisodeId.IsEmpty() || PresentationFrame.bVisionCaptureRequested) return 0;
    // A strict frame can already be queued as the interpolation target while
    // the last presentation pose is still idle. Never add warmup work then.
    if (!PresentationFrames.IsEmpty())
    {
        const FBskRenderFrame& Latest = PresentationFrames.Last();
        if (!Latest.bCaptureOnDemand || !Latest.CaptureEpisodeId.IsEmpty() || Latest.bVisionCaptureRequested) return 0;
    }
    int32 RenderedCameras = 0;
    for (const TPair<FString, FBskCameraDefinition>& Pair : ManifestCameras)
    {
        USceneCaptureComponent2D* Capture = CameraCaptureComponents.FindRef(Pair.Key);
        if (!Capture || !Capture->TextureTarget) continue;
        double& NextSeconds = CameraNextCaptureSeconds.FindOrAdd(Pair.Key);
        const bool bFirstWarmup = NextSeconds <= 0.0;
        if (!BskPreviewCaptureDue(NowSeconds, 10.0, NextSeconds)) continue;
        CaptureRgbCamera(Pair.Key, Capture);
        ++RenderedCameras;
        if (bFirstWarmup)
            UE_LOG(LogBskUnreal, Display, TEXT("Warming idle RGB camera %s at 10 Hz without dataset output"), *Pair.Key);
    }
    return RenderedCameras;
}
