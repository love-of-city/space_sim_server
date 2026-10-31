#include "BskSceneController.h"
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
    ConfigureFixedRgbExposure(Capture);
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
        CaptureRgbCamera(Pair.Key, Pair.Value);
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
