#pragma once

#include "BskProtocolTypes.h"
#include "BskCaptureProvider.h"
#include "BskLeRobotSampling.h"

/** Select a camera consumer without allowing dataset overrides to change vision. */
struct FBskCaptureSelection
{
    EBskCapturePurpose Purpose = EBskCapturePurpose::AuthoritativeDataset;
    double RateHertz = 0.0;
    TArray<FString> Products;
};

inline bool BskSelectCameraCapture(const FBskRenderFrame& Frame, const FBskCameraDefinition& Camera,
    double DatasetRateOverride, const TArray<FString>& DatasetProductOverride, FBskCaptureSelection& Out)
{
    if (Camera.CaptureConsumer == TEXT("vision"))
    {
        if (!Frame.bVisionCaptureRequested || !Frame.VisionCaptureCameraIds.Contains(Camera.CameraId)) return false;
        Out.Purpose = EBskCapturePurpose::AuthoritativeVision;
        Out.RateHertz = Camera.CaptureRateHertz;
        Out.Products = Camera.CaptureProducts;
    }
    else
    {
        if (Frame.bCaptureOnDemand && Frame.CaptureEpisodeId.IsEmpty()) return false;
        Out.Purpose = EBskCapturePurpose::AuthoritativeDataset;
        Out.RateHertz = DatasetRateOverride > 0.0 ? DatasetRateOverride : Camera.CaptureRateHertz;
        Out.Products = DatasetProductOverride.IsEmpty() ? Camera.CaptureProducts : DatasetProductOverride;
    }
    int64 SampleIndex = 0;
    return !Out.Products.IsEmpty() && BskLeRobotSample(Frame.SimulationTimeNanoseconds, Out.RateHertz, SampleIndex);
}
