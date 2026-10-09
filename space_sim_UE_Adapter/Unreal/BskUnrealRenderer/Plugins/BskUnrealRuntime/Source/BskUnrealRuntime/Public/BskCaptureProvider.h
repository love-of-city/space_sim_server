#pragma once

#include "CoreMinimal.h"

class AActor;

enum class EBskCaptureChannel : uint8
{
    Rgb
};

enum class EBskCapturePurpose : uint8
{
    AuthoritativeDataset,
    Preview
};

struct BSKUNREALRUNTIME_API FBskCaptureRequest
{
    FString CameraId;
    TArray<EBskCaptureChannel> Channels;
    FIntPoint Resolution = FIntPoint(1920, 1080);
    int64 SimulationTimeNanoseconds = 0;
    int64 SourceWallTimeNanoseconds = 0;
    int64 FrameId = -1;
    FString CaptureEpisodeId;
    FVector3d OriginInertialMeters = FVector3d::ZeroVector;
    FMatrix44d LocalFromInertial = FMatrix44d::Identity;
    EBskCapturePurpose Purpose = EBskCapturePurpose::AuthoritativeDataset;
    bool bReuseExistingRgbTarget = false;
    // Render-only phase: run the SceneCapture pass but skip the synchronous
    // GPU readback. Submit every due camera first, then read each render target.
    // ReadPixels remains synchronous; this is not an asynchronous readback API.
    bool bSkipReadback = false;
    FString OutputDirectory;
    bool bWriteToDisk = true;
    bool bSendToNetwork = false;
};

/** Runtime capture extension point. Implementations are invoked only on the Game Thread. */
class BSKUNREALRUNTIME_API IBskCaptureProvider
{
public:
    virtual ~IBskCaptureProvider() = default;
    virtual bool RegisterCamera(const FString& CameraId, TWeakObjectPtr<AActor> CameraActor) = 0;
    virtual bool RequestCapture(const FBskCaptureRequest& Request, FString& OutError) = 0;
    virtual void Shutdown() = 0;
};
