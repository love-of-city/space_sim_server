#pragma once
#include "BskSceneController.h"
#include "BskCaptureProvider.h"
#include "BskCaptureProducts.h"
#include "Async/Future.h"
#include "Dom/JsonObject.h"
#include "IImageWrapper.h"
#include "RHIGPUReadback.h"

// Job owns its pixels/encoder/RHI readback; scene access stays on the Game Thread.
struct FBskAsyncCaptureJob
{
    FBskCaptureRequest Request;
    TSharedPtr<FJsonObject> Metadata;
    FIntPoint Resolution;
    int64 Sequence = 0;
    uint64 ReservedBytes = 0;
    double SubmittedSeconds = 0;
    TAtomic<bool> bCopySubmitted{false};
    bool bRgba = false;
    TSharedPtr<FRHIGPUTextureReadback, ESPMode::ThreadSafe> Readback;
    TSharedPtr<IImageWrapper> Wrapper;
    TFuture<FCapturedDataProduct> Encoded;
    TArray<FCapturedDataProduct> Products;
    FString Error;
};

class FBskBuiltinCaptureProvider final : public IBskCaptureProvider
{
public:
    explicit FBskBuiltinCaptureProvider(ABskSceneController* InOwner) : Owner(InOwner) {}
    virtual bool RegisterCamera(const FString&, TWeakObjectPtr<AActor>) override { return Owner.IsValid(); }
    virtual bool RequestCapture(const FBskCaptureRequest& Request, FString& OutError) override
    {
        return Owner.IsValid() && Owner->CaptureCameraDataProducts(Request, OutError);
    }
    virtual void Shutdown() override { Owner.Reset(); }
private:
    TWeakObjectPtr<ABskSceneController> Owner;
};
