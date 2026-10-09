#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "BskCaptureProvider.h"
#include "BskProtocolTypes.h"
#include "BskRenderExtension.h"
#include "BskRenderWorldSubsystem.generated.h"

DECLARE_MULTICAST_DELEGATE_OneParam(FOnBskSceneManifestChanged, const FBskSceneManifest&);
DECLARE_MULTICAST_DELEGATE_OneParam(FOnBskSimulationTimeChanged, int64);
DECLARE_MULTICAST_DELEGATE_OneParam(FOnBskManifestApplied, const FBskSceneManifest&);
DECLARE_MULTICAST_DELEGATE_OneParam(FOnBskEventApplied, const FBskRenderEvent&);
DECLARE_MULTICAST_DELEGATE_OneParam(FOnBskFrameAppliedToWorld, const FBskRenderFrame&);

/** World-scoped authoritative cache for renderer session metadata, never dynamics. */
UCLASS()
class BSKUNREALRUNTIME_API UBskRenderWorldSubsystem : public UWorldSubsystem
{
    GENERATED_BODY()

public:
    virtual void Deinitialize() override;

    void AcceptManifest(const FBskSceneManifest& Manifest);
    bool AcceptFrame(const FBskRenderFrame& Frame, FString& OutReason);
    void NotifyManifestApplied(const FBskSceneManifest& Manifest);
    void NotifyEventApplied(const FBskRenderEvent& Event);
    void NotifyFrameApplied(const FBskRenderFrame& Frame);

    bool RegisterRenderExtension(TSharedRef<IBskRenderExtension> Extension, int32 Priority = 0);
    bool UnregisterRenderExtension(FName ExtensionName);
    AActor* TrySpawnObject(const FBskRenderSpawnContext& Context, const FBskObjectDefinition& Definition);
    AActor* TrySpawnCelestialBody(const FBskRenderSpawnContext& Context, const FBskCelestialBodyDefinition& Definition);
    AActor* TrySpawnVisual(const FBskRenderSpawnContext& Context, const FBskVisualDefinition& Definition);
    AActor* TrySpawnCamera(const FBskRenderSpawnContext& Context, const FBskCameraDefinition& Definition);

    bool RegisterCaptureProvider(FName ProviderName, TSharedRef<IBskCaptureProvider> Provider, int32 Priority = 0);
    bool UnregisterCaptureProvider(FName ProviderName);
    void RegisterCameraForCapture(const FString& CameraId, TWeakObjectPtr<AActor> CameraActor);
    bool RequestCapture(const FBskCaptureRequest& Request, FString& OutError);

    const FBskSceneManifest* GetLatestManifest() const { return bHasManifest ? &LatestManifest : nullptr; }
    const FBskRenderFrame* GetLatestReceivedFrame() const { return bHasReceivedFrame ? &LatestReceivedFrame : nullptr; }
    const FBskRenderFrame* GetLatestAppliedFrame() const { return bHasAppliedFrame ? &LatestAppliedFrame : nullptr; }

    UFUNCTION(BlueprintPure, Category="BSK Renderer")
    FString GetSessionId() const { return SessionId; }

    UFUNCTION(BlueprintPure, Category="BSK Renderer")
    int64 GetManifestRevision() const { return ManifestRevision; }

    UFUNCTION(BlueprintPure, Category="BSK Renderer")
    int64 GetSimulationTimeNanoseconds() const { return SimulationTimeNanoseconds; }

    UFUNCTION(BlueprintPure, Category="BSK Renderer")
    int64 GetLatestReceivedFrameId() const { return bHasReceivedFrame ? LatestReceivedFrame.FrameId : -1; }

    UFUNCTION(BlueprintPure, Category="BSK Renderer")
    int64 GetLatestAppliedFrameId() const { return bHasAppliedFrame ? LatestAppliedFrame.FrameId : -1; }

    FOnBskSceneManifestChanged& OnManifestChanged() { return ManifestChanged; }
    FOnBskSimulationTimeChanged& OnSimulationTimeChanged() { return SimulationTimeChanged; }
    FOnBskManifestApplied& OnManifestApplied() { return ManifestApplied; }
    FOnBskEventApplied& OnEventApplied() { return EventApplied; }
    FOnBskFrameAppliedToWorld& OnFrameApplied() { return FrameApplied; }

private:
    struct FRenderExtensionEntry
    {
        FName Name;
        int32 Priority = 0;
        TSharedPtr<IBskRenderExtension> Extension;
    };

    struct FCaptureProviderEntry
    {
        FName Name;
        int32 Priority = 0;
        TSharedPtr<IBskCaptureProvider> Provider;
    };

    void ResetFrameState();

    FString SessionId;
    int64 ManifestRevision = 0;
    int64 SimulationTimeNanoseconds = 0;
    bool bHasManifest = false;
    bool bHasReceivedFrame = false;
    bool bHasAppliedFrame = false;
    FBskSceneManifest LatestManifest;
    FBskRenderFrame LatestReceivedFrame;
    FBskRenderFrame LatestAppliedFrame;
    TArray<FRenderExtensionEntry> RenderExtensions;
    TArray<FCaptureProviderEntry> CaptureProviders;
    TMap<FString, TWeakObjectPtr<AActor>> CaptureCameras;
    FOnBskSceneManifestChanged ManifestChanged;
    FOnBskSimulationTimeChanged SimulationTimeChanged;
    FOnBskManifestApplied ManifestApplied;
    FOnBskEventApplied EventApplied;
    FOnBskFrameAppliedToWorld FrameApplied;
};
