#include "BskCaptureDiskWriter.h"
#include "BskUnrealRuntime.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"
#include "HAL/PlatformProcess.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Containers/Queue.h"

class FBskCaptureDiskWriter::FWorker final : public FRunnable
{
public:
    // Storage injection is confined to this private type, for deterministic
    // disk-full/partial-write tests without filling or modifying a real volume.
    using FWriteFile = TFunction<bool(const FString&, const TArray<uint8>&)>;
    explicit FWorker(FWriteFile InWriteFile = {}) : WriteFile(MoveTemp(InWriteFile))
    {
        if (!WriteFile) WriteFile = [](const FString& Path, const TArray<uint8>& Bytes)
        {
            return FFileHelper::SaveArrayToFile(Bytes, *Path);
        };
        WakeEvent = FPlatformProcess::GetSynchEventFromPool(false);
        Thread = FRunnableThread::Create(this, TEXT("BskCaptureDiskOutput"));
    }

    virtual ~FWorker() override
    {
        StopAndWait();
        if (WakeEvent) FPlatformProcess::ReturnSynchEventToPool(WakeEvent);
    }

    void StopAndWait()
    {
        Stop();
        if (Thread)
        {
            Thread->WaitForCompletion();
            delete Thread;
            Thread = nullptr;
        }
    }

    bool HasCapacity(int32 Count)
    {
        FScopeLock Lock(&WorkMutex);
        return Thread && !bStopRequested && OutstandingWork + Count <= MaxPendingWork;
    }

    bool EnqueueReliable(FBskCaptureDiskWork&& Work)
    {
        {
            FScopeLock Lock(&WorkMutex);
            if (!Thread || bStopRequested || OutstandingWork >= MaxPendingWork) return false;
            ++OutstandingWork;
            PendingWork.Add(MoveTemp(Work));
        }
        WakeEvent->Trigger();
        return true;
    }

    bool ConsumeResult(FBskCaptureDiskResult& Result)
    {
        if (!Results.Dequeue(Result)) return false;
        FScopeLock Lock(&WorkMutex);
        --OutstandingWork;
        return true;
    }

    virtual uint32 Run() override
    {
        while (true)
        {
            FBskCaptureDiskWork Work;
            bool bHasWork = false;
            {
                FScopeLock Lock(&WorkMutex);
                if (!PendingWork.IsEmpty())
                {
                    Work = MoveTemp(PendingWork[0]);
                    PendingWork.RemoveAt(0, 1, EAllowShrinking::No);
                    bHasWork = true;
                }
            }
            if (!bHasWork)
            {
                if (bStopRequested) break;
                WakeEvent->Wait(100);
                continue;
            }
            FBskCaptureDiskResult Result = WriteCapture(Work);
            if (!Result.bSucceeded)
                UE_LOG(LogBskUnreal, Error, TEXT("BSK disk capture session=%s frame=%lld failed: %s"),
                    *Result.SessionId, Result.Request.FrameId, *Result.Error);
            Results.Enqueue(MoveTemp(Result));
        }
        return 0;
    }

    virtual void Stop() override
    {
        bStopRequested = true;
        if (WakeEvent) WakeEvent->Trigger();
    }

private:
    FBskCaptureDiskResult WriteCapture(const FBskCaptureDiskWork& Work)
    {
        FBskCaptureDiskResult Result;
        Result.Request = Work.Request;
        Result.SessionId = Work.SessionId;
        Result.Sequence = Work.Sequence;
        Result.CompletionPath = FPaths::Combine(Work.Directory, Work.Stem + TEXT(".complete.json"));
        if (!IFileManager::Get().MakeDirectory(*Work.Directory, true))
        {
            Result.Error = TEXT("cannot create capture directory: ") + Work.Directory;
            return Result;
        }
        if (IFileManager::Get().FileExists(*Result.CompletionPath))
        {
            Result.Error = TEXT("refusing to overwrite completed capture: ") + Result.CompletionPath;
            return Result;
        }
        auto CommitFile = [this, &Result](const FString& Path, const TArray<uint8>& Bytes)
        {
            const FString Temporary = Path + TEXT(".partial");
            if (!WriteFile(Temporary, Bytes))
            {
                Result.Error = TEXT("capture write failed: ") + Path;
                IFileManager::Get().Delete(*Temporary, false, true);
                return false;
            }
            if (!IFileManager::Get().Move(*Path, *Temporary, true, true))
            {
                Result.Error = TEXT("capture commit failed: ") + Path;
                IFileManager::Get().Delete(*Temporary, false, true);
                return false;
            }
            return true;
        };
        for (const FCapturedDataProduct& Product : Work.Products)
        {
            const FString Path = FPaths::Combine(Work.Directory,
                FString::Printf(TEXT("%s_%s.%s"), *Work.Stem, *Product.Name, *Product.Extension));
            if (!CommitFile(Path, Product.Bytes)) return Result;
        }
        FTCHARToUTF8 Utf8(*Work.MetadataJson);
        TArray<uint8> MetadataBytes;
        MetadataBytes.Append(reinterpret_cast<const uint8*>(Utf8.Get()), Utf8.Length());
        if (!CommitFile(FPaths::Combine(Work.Directory, Work.Stem + TEXT(".json")), MetadataBytes)) return Result;
        // The final marker is the only publication of a complete disk frame.
        // Partial outputs retain no marker and cannot be mistaken for success.
        if (!CommitFile(Result.CompletionPath, MetadataBytes)) return Result;
        Result.bSucceeded = true;
        return Result;
    }

    FWriteFile WriteFile;
    FThreadSafeBool bStopRequested = false;
    static constexpr int32 MaxPendingWork = 16;
    FCriticalSection WorkMutex;
    int32 OutstandingWork = 0;
    TArray<FBskCaptureDiskWork> PendingWork;
    TQueue<FBskCaptureDiskResult, EQueueMode::Spsc> Results;
    FEvent* WakeEvent = nullptr;
    FRunnableThread* Thread = nullptr;
};
FBskCaptureDiskWriter::FBskCaptureDiskWriter(FWriteFile Write) : Worker(MakeUnique<FWorker>(MoveTemp(Write))) {}
FBskCaptureDiskWriter::~FBskCaptureDiskWriter() = default;
void FBskCaptureDiskWriter::StopAndWait() { Worker->StopAndWait(); }
bool FBskCaptureDiskWriter::HasCapacity(int32 Count) { return Worker->HasCapacity(Count); }
bool FBskCaptureDiskWriter::EnqueueReliable(FBskCaptureDiskWork&& Work) { return Worker->EnqueueReliable(MoveTemp(Work)); }
bool FBskCaptureDiskWriter::ConsumeResult(FBskCaptureDiskResult& Result) { return Worker->ConsumeResult(Result); }
