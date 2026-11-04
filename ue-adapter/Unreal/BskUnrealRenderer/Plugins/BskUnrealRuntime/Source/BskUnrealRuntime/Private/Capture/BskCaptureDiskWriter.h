#pragma once
#include "CoreMinimal.h"
#include "BskCaptureProvider.h"
#include "BskCaptureProducts.h"

struct FBskCaptureDiskWork
{
    FBskCaptureRequest Request;
    FString SessionId;
    int64 Sequence = 0;
    FString Directory;
    FString Stem;
    FString MetadataJson;
    TArray<FCapturedDataProduct> Products;
};

struct FBskCaptureDiskResult
{
    FBskCaptureRequest Request;
    FString SessionId;
    int64 Sequence = 0;
    bool bSucceeded = false;
    FString CompletionPath;
    FString Error;
};

// Admission includes queued, in-flight and unconsumed completion results.
class FBskCaptureDiskWriter final
{
public:
    using FWriteFile = TFunction<bool(const FString&, const TArray<uint8>&)>;
    explicit FBskCaptureDiskWriter(FWriteFile Write = {});
    ~FBskCaptureDiskWriter();
    void StopAndWait();
    bool HasCapacity(int32 Count);
    bool EnqueueReliable(FBskCaptureDiskWork&& Work);
    bool ConsumeResult(FBskCaptureDiskResult& Result);
private:
    class FWorker;
    TUniquePtr<FWorker> Worker;
};
