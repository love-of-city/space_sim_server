#pragma once

#include "CoreMinimal.h"
#include "BskProtocolTypes.h"

/** Transport-neutral source for live TCP, future UDP/binary, or replay files. */
class BSKUNREALRUNTIME_API IBskFrameSource
{
public:
    virtual ~IBskFrameSource() = default;
    virtual bool StartSource() = 0;
    virtual void StopSource() = 0;
    virtual bool ConsumeLatest(FBskRenderFrame& OutFrame) = 0;
    // Live sources can consume previews even when strict image output is full.
    virtual bool ConsumeForCapture(FBskRenderFrame& OutFrame, bool bCaptureHasCapacity)
    {
        return bCaptureHasCapacity && ConsumeLatest(OutFrame);
    }
    virtual bool ConsumeLatestManifest(FBskSceneManifest& OutManifest) { return false; }
    virtual bool ConsumeEvent(FBskRenderEvent& OutEvent) { return false; }
    virtual bool SetPaused(bool bPaused) { return false; }
    virtual bool SetPlaybackRate(double Rate) { return false; }
    virtual bool StepOnce() { return false; }
    virtual bool SeekSimulationTime(int64 SimulationTimeNanoseconds) { return false; }
    virtual bool SendCommandJson(const FString& CommandJson, FString& OutError)
    {
        OutError = TEXT("this message source does not support bidirectional commands");
        return false;
    }
    virtual FString GetStatus() const = 0;
    virtual int32 GetPendingFrameCount() const { return 0; }
};

using IBskMessageSource = IBskFrameSource;
