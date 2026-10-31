#include "Capture/BskCaptureDiskWriter.h"
#include "Misc/AutomationTest.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBskCaptureDiskWriterTest,
    "BskUnreal.Capture.DiskCompletionAndFailures",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ClientContext | EAutomationTestFlags::EngineFilter)

bool FBskCaptureDiskWriterTest::RunTest(const FString& Parameters)
{
    const FString Root = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Automation/CaptureDisk"), FGuid::NewGuid().ToString());
    auto WorkFor = [&Root](const FString& Name)
    {
        FBskCaptureDiskWork Work;
        Work.Directory = FPaths::Combine(Root, Name);
        Work.Stem = TEXT("000000000001_wrist");
        Work.MetadataJson = TEXT("{\"session_id\":\"disk-test\",\"source_frame_id\":\"7\"}");
        Work.SessionId = TEXT("disk-test");
        Work.Sequence = 1;
        Work.Request.FrameId = 7;
        Work.Request.CaptureEpisodeId = TEXT("episode-test");
        Work.Products.Add({TEXT("rgb"), TEXT("test"), TEXT("jpg"), {1, 2, 3}});
        return Work;
    };
    auto RunWork = [this](FBskCaptureDiskWork Work, FBskCaptureDiskWriter::FWriteFile Write = {})
    {
        FBskCaptureDiskWriter Writer(MoveTemp(Write));
        TestTrue(TEXT("work admitted"), Writer.EnqueueReliable(MoveTemp(Work)));
        Writer.StopAndWait();
        FBskCaptureDiskResult Result;
        TestTrue(TEXT("every admitted work returns a result"), Writer.ConsumeResult(Result));
        TestEqual(TEXT("result retains original session"), Result.SessionId, FString(TEXT("disk-test")));
        TestEqual(TEXT("result retains original frame"), Result.Request.FrameId, int64(7));
        FBskCaptureDiskResult Extra;
        TestFalse(TEXT("exactly one completion result"), Writer.ConsumeResult(Extra));
        TestFalse(TEXT("stopped writer rejects new work"), Writer.EnqueueReliable(FBskCaptureDiskWork{}));
        return Result;
    };
    AddExpectedError(TEXT("BSK disk capture"), EAutomationExpectedErrorFlags::Contains, 5);
    const auto Success = RunWork(WorkFor(TEXT("success")));
    TestTrue(TEXT("normal frame succeeds"), Success.bSucceeded);
    TestTrue(TEXT("complete marker published after all files"), IFileManager::Get().FileExists(*Success.CompletionPath));
    const auto Duplicate = RunWork(WorkFor(TEXT("success")));
    TestFalse(TEXT("completed frame cannot be overwritten"), Duplicate.bSucceeded);
    for (const FString& Suffix : {FString(TEXT(".jpg.partial")), FString(TEXT(".json.partial")), FString(TEXT(".complete.json.partial"))})
    {
        const auto Failure = RunWork(WorkFor(Suffix.Replace(TEXT("."), TEXT("_"))),
            [Suffix](const FString& Path, const TArray<uint8>& Bytes)
            {
                if (Path.EndsWith(Suffix))
                {
                    // Simulate a short write / ENOSPC with a partially created file.
                    FFileHelper::SaveArrayToFile(TArray<uint8>{0}, *Path);
                    return false;
                }
                return FFileHelper::SaveArrayToFile(Bytes, *Path);
            });
        TestFalse(TEXT("write failure reported"), Failure.bSucceeded);
        TestFalse(TEXT("failed frame has no completion marker"), IFileManager::Get().FileExists(*Failure.CompletionPath));
        TestFalse(TEXT("failed temporary marker removed"), IFileManager::Get().FileExists(*(Failure.CompletionPath + TEXT(".partial"))));
        TestFalse(TEXT("error explains failed path"), Failure.Error.IsEmpty());
    }
    const FString Blocked = FPaths::Combine(Root, TEXT("blocked"));
    FFileHelper::SaveStringToFile(TEXT("not a directory"), *Blocked);
    const auto Failure = RunWork(WorkFor(TEXT("blocked/child")));
    TestFalse(TEXT("unwritable directory reported"), Failure.bSucceeded);
    IFileManager::Get().DeleteDirectory(*Root, false, true);
    return true;
}

#endif
