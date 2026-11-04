#include "BskReplaySource.h"

#include "HAL/PlatformTime.h"
#include "Misc/FileHelper.h"

namespace
{
constexpr ANSICHAR RecordingMagic[] = "BSKREC2\n";
}

FBskReplaySource::FBskReplaySource(FString InPath, double InPlaybackRate)
    : Path(MoveTemp(InPath)), PlaybackRate(FMath::Max(InPlaybackRate, 0.001))
{
}

bool FBskReplaySource::StartSource()
{
    TArray<uint8> Bytes;
    if (!FFileHelper::LoadFileToArray(Bytes, *Path))
    {
        Status = FString::Printf(TEXT("could not read replay: %s"), *Path);
        return false;
    }
    constexpr int32 MagicLength = sizeof(RecordingMagic) - 1;
    if (Bytes.Num() < MagicLength || FMemory::Memcmp(Bytes.GetData(), RecordingMagic, MagicLength) != 0)
    {
        Status = TEXT("invalid BSKREC2 recording");
        return false;
    }
    FBskFrameParser Parser;
    TArray<FBskRenderMessage> Messages;
    FString Error;
    if (!Parser.AppendMessages(Bytes.GetData() + MagicLength, Bytes.Num() - MagicLength, Messages, Error))
    {
        Status = FString::Printf(TEXT("invalid replay stream: %s"), *Error);
        return false;
    }
    Frames.Reset();
    Events.Reset();
    for (FBskRenderMessage& Message : Messages)
    {
        if (Message.Type == EBskRenderMessageType::SceneManifest) Manifest = MoveTemp(Message.Manifest);
        else if (Message.Type == EBskRenderMessageType::Frame) Frames.Add(MoveTemp(Message.Frame));
        else if (Message.Type == EBskRenderMessageType::Event) Events.Add(MoveTemp(Message.Event));
    }
    if (Frames.IsEmpty())
    {
        Status = TEXT("replay contains no frames");
        return false;
    }
    NextFrameIndex = 0;
    NextEventIndex = 0;
    SimulationStartNanoseconds = Frames[0].SimulationTimeNanoseconds;
    bManifestPending = Manifest.Revision > 0;
    bPaused = false;
    bStepPending = false;
    ResetClock();
    Status = FString::Printf(TEXT("replaying %d frames from %s"), Frames.Num(), *Path);
    return true;
}

void FBskReplaySource::StopSource()
{
    Status = TEXT("stopped");
}

void FBskReplaySource::ResetClock()
{
    WallStartSeconds = FPlatformTime::Seconds();
    if (Frames.IsValidIndex(NextFrameIndex)) SimulationStartNanoseconds = Frames[NextFrameIndex].SimulationTimeNanoseconds;
}

bool FBskReplaySource::ConsumeLatestManifest(FBskSceneManifest& OutManifest)
{
    if (!bManifestPending) return false;
    OutManifest = Manifest;
    bManifestPending = false;
    return true;
}

bool FBskReplaySource::ConsumeEvent(FBskRenderEvent& OutEvent)
{
    if (!Events.IsValidIndex(NextEventIndex)) return false;
    OutEvent = Events[NextEventIndex++];
    return true;
}

bool FBskReplaySource::ConsumeLatest(FBskRenderFrame& OutFrame)
{
    if (!Frames.IsValidIndex(NextFrameIndex)) return false;
    if (bPaused && !bStepPending) return false;
    const double ElapsedSeconds = (FPlatformTime::Seconds() - WallStartSeconds) * PlaybackRate;
    const int64 TargetNanoseconds = SimulationStartNanoseconds + static_cast<int64>(ElapsedSeconds * 1.0e9);
    int32 SelectedIndex = NextFrameIndex;
    if (!bStepPending)
    {
        if (Frames[SelectedIndex].SimulationTimeNanoseconds > TargetNanoseconds) return false;
        while (Frames.IsValidIndex(SelectedIndex + 1) && Frames[SelectedIndex + 1].SimulationTimeNanoseconds <= TargetNanoseconds) ++SelectedIndex;
    }
    OutFrame = Frames[SelectedIndex];
    NextFrameIndex = SelectedIndex + 1;
    bStepPending = false;
    if (!Frames.IsValidIndex(NextFrameIndex)) Status = TEXT("replay complete; retaining final frame");
    return true;
}

bool FBskReplaySource::SetPaused(bool bInPaused)
{
    bPaused = bInPaused;
    ResetClock();
    return true;
}

bool FBskReplaySource::SetPlaybackRate(double Rate)
{
    if (!FMath::IsFinite(Rate) || Rate <= 0.0) return false;
    PlaybackRate = Rate;
    ResetClock();
    return true;
}

bool FBskReplaySource::StepOnce()
{
    bPaused = true;
    bStepPending = true;
    return Frames.IsValidIndex(NextFrameIndex);
}

bool FBskReplaySource::SeekSimulationTime(int64 SimulationTimeNanoseconds)
{
    NextFrameIndex = 0;
    while (Frames.IsValidIndex(NextFrameIndex) && Frames[NextFrameIndex].SimulationTimeNanoseconds < SimulationTimeNanoseconds) ++NextFrameIndex;
    bManifestPending = true;
    ResetClock();
    return Frames.IsValidIndex(NextFrameIndex);
}
