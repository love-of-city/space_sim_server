#pragma once

#include "CoreMinimal.h"
#include "BskFrameParser.h"
#include "BskFrameSource.h"

/** In-memory deterministic replay source for version 2 ``.bskrec`` files. */
class BSKUNREALRUNTIME_API FBskReplaySource final : public IBskMessageSource
{
public:
    explicit FBskReplaySource(FString InPath, double InPlaybackRate = 1.0);

    virtual bool StartSource() override;
    virtual void StopSource() override;
    virtual bool ConsumeLatest(FBskRenderFrame& OutFrame) override;
    virtual bool ConsumeLatestManifest(FBskSceneManifest& OutManifest) override;
    virtual bool ConsumeEvent(FBskRenderEvent& OutEvent) override;
    virtual bool SetPaused(bool bInPaused) override;
    virtual bool SetPlaybackRate(double Rate) override;
    virtual bool StepOnce() override;
    virtual bool SeekSimulationTime(int64 SimulationTimeNanoseconds) override;
    virtual FString GetStatus() const override { return Status; }

private:
    void ResetClock();

    FString Path;
    FString Status = TEXT("stopped");
    TArray<FBskRenderFrame> Frames;
    TArray<FBskRenderEvent> Events;
    FBskSceneManifest Manifest;
    int32 NextFrameIndex = 0;
    int32 NextEventIndex = 0;
    double PlaybackRate = 1.0;
    double WallStartSeconds = 0.0;
    int64 SimulationStartNanoseconds = 0;
    bool bManifestPending = false;
    bool bPaused = false;
    bool bStepPending = false;
};
