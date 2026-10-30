#include "BskRenderWorldSubsystem.h"

#include "GameFramework/Actor.h"

namespace
{
template <typename EntryType>
void SortByPriority(TArray<EntryType>& Entries)
{
    Entries.Sort([](const EntryType& A, const EntryType& B)
    {
        if (A.Priority != B.Priority) return A.Priority > B.Priority;
        return A.Name.LexicalLess(B.Name);
    });
}
}

void UBskRenderWorldSubsystem::Deinitialize()
{
    check(IsInGameThread());
    for (FCaptureProviderEntry& Entry : CaptureProviders)
    {
        if (Entry.Provider) Entry.Provider->Shutdown();
    }
    CaptureProviders.Reset();
    CaptureCameras.Reset();
    RenderExtensions.Reset();
    Super::Deinitialize();
}

void UBskRenderWorldSubsystem::ResetFrameState()
{
    check(IsInGameThread());
    bHasReceivedFrame = false;
    bHasAppliedFrame = false;
    LatestReceivedFrame = FBskRenderFrame{};
    LatestAppliedFrame = FBskRenderFrame{};
    SimulationTimeNanoseconds = 0;
    SimulationTimeChanged.Broadcast(0);
}

void UBskRenderWorldSubsystem::AcceptManifest(const FBskSceneManifest& Manifest)
{
    check(IsInGameThread());
    if (SessionId != Manifest.SessionId)
    {
        ResetFrameState();
    }
    SessionId = Manifest.SessionId;
    ManifestRevision = Manifest.Revision;
    LatestManifest = Manifest;
    bHasManifest = true;
    ManifestChanged.Broadcast(Manifest);
}

bool UBskRenderWorldSubsystem::AcceptFrame(const FBskRenderFrame& Frame, FString& OutReason)
{
    check(IsInGameThread());
    if (!SessionId.IsEmpty() && !Frame.SessionId.IsEmpty() && Frame.SessionId != SessionId)
    {
        OutReason = FString::Printf(TEXT("frame session '%s' does not match '%s'"), *Frame.SessionId, *SessionId);
        return false;
    }
    if (ManifestRevision > 0 && Frame.ManifestRevision > 0 && Frame.ManifestRevision != ManifestRevision)
    {
        OutReason = FString::Printf(TEXT("frame manifest revision %lld does not match %lld"), Frame.ManifestRevision, ManifestRevision);
        return false;
    }
    LatestReceivedFrame = Frame;
    bHasReceivedFrame = true;
    SimulationTimeNanoseconds = Frame.SimulationTimeNanoseconds;
    SimulationTimeChanged.Broadcast(SimulationTimeNanoseconds);
    return true;
}

void UBskRenderWorldSubsystem::NotifyManifestApplied(const FBskSceneManifest& Manifest)
{
    check(IsInGameThread());
    for (const FRenderExtensionEntry& Entry : RenderExtensions) if (Entry.Extension) Entry.Extension->OnManifestApplied(Manifest);
    ManifestApplied.Broadcast(Manifest);
}

void UBskRenderWorldSubsystem::NotifyEventApplied(const FBskRenderEvent& Event)
{
    check(IsInGameThread());
    if (Event.EventKind == TEXT("scene_reset")) ResetFrameState();
    for (const FRenderExtensionEntry& Entry : RenderExtensions) if (Entry.Extension) Entry.Extension->OnEventApplied(Event);
    EventApplied.Broadcast(Event);
}

void UBskRenderWorldSubsystem::NotifyFrameApplied(const FBskRenderFrame& Frame)
{
    check(IsInGameThread());
    LatestAppliedFrame = Frame;
    bHasAppliedFrame = true;
    for (const FRenderExtensionEntry& Entry : RenderExtensions) if (Entry.Extension) Entry.Extension->OnFrameApplied(Frame);
    FrameApplied.Broadcast(Frame);
}

bool UBskRenderWorldSubsystem::RegisterRenderExtension(TSharedRef<IBskRenderExtension> Extension, int32 Priority)
{
    check(IsInGameThread());
    const FName Name = Extension->GetExtensionName();
    if (Name.IsNone() || RenderExtensions.ContainsByPredicate([Name](const FRenderExtensionEntry& Entry) { return Entry.Name == Name; })) return false;
    RenderExtensions.Add({Name, Priority, Extension});
    SortByPriority(RenderExtensions);
    return true;
}

bool UBskRenderWorldSubsystem::UnregisterRenderExtension(FName ExtensionName)
{
    check(IsInGameThread());
    return RenderExtensions.RemoveAll([ExtensionName](const FRenderExtensionEntry& Entry) { return Entry.Name == ExtensionName; }) > 0;
}

AActor* UBskRenderWorldSubsystem::TrySpawnObject(const FBskRenderSpawnContext& Context, const FBskObjectDefinition& Definition)
{
    check(IsInGameThread());
    for (const FRenderExtensionEntry& Entry : RenderExtensions) if (Entry.Extension) if (AActor* Actor = Entry.Extension->TrySpawnObject(Context, Definition)) return Actor;
    return nullptr;
}

AActor* UBskRenderWorldSubsystem::TrySpawnCelestialBody(const FBskRenderSpawnContext& Context, const FBskCelestialBodyDefinition& Definition)
{
    check(IsInGameThread());
    for (const FRenderExtensionEntry& Entry : RenderExtensions) if (Entry.Extension) if (AActor* Actor = Entry.Extension->TrySpawnCelestialBody(Context, Definition)) return Actor;
    return nullptr;
}

AActor* UBskRenderWorldSubsystem::TrySpawnVisual(const FBskRenderSpawnContext& Context, const FBskVisualDefinition& Definition)
{
    check(IsInGameThread());
    for (const FRenderExtensionEntry& Entry : RenderExtensions) if (Entry.Extension) if (AActor* Actor = Entry.Extension->TrySpawnVisual(Context, Definition)) return Actor;
    return nullptr;
}

AActor* UBskRenderWorldSubsystem::TrySpawnCamera(const FBskRenderSpawnContext& Context, const FBskCameraDefinition& Definition)
{
    check(IsInGameThread());
    for (const FRenderExtensionEntry& Entry : RenderExtensions) if (Entry.Extension) if (AActor* Actor = Entry.Extension->TrySpawnCamera(Context, Definition)) return Actor;
    return nullptr;
}

bool UBskRenderWorldSubsystem::RegisterCaptureProvider(FName ProviderName, TSharedRef<IBskCaptureProvider> Provider, int32 Priority)
{
    check(IsInGameThread());
    if (ProviderName.IsNone() || CaptureProviders.ContainsByPredicate([ProviderName](const FCaptureProviderEntry& Entry) { return Entry.Name == ProviderName; })) return false;
    CaptureProviders.Add({ProviderName, Priority, Provider});
    SortByPriority(CaptureProviders);
    for (const TPair<FString, TWeakObjectPtr<AActor>>& Camera : CaptureCameras) Provider->RegisterCamera(Camera.Key, Camera.Value);
    return true;
}

bool UBskRenderWorldSubsystem::UnregisterCaptureProvider(FName ProviderName)
{
    check(IsInGameThread());
    const int32 Index = CaptureProviders.IndexOfByPredicate([ProviderName](const FCaptureProviderEntry& Entry) { return Entry.Name == ProviderName; });
    if (Index == INDEX_NONE) return false;
    if (CaptureProviders[Index].Provider) CaptureProviders[Index].Provider->Shutdown();
    CaptureProviders.RemoveAt(Index);
    return true;
}

void UBskRenderWorldSubsystem::RegisterCameraForCapture(const FString& CameraId, TWeakObjectPtr<AActor> CameraActor)
{
    check(IsInGameThread());
    CaptureCameras.Add(CameraId, CameraActor);
    for (const FCaptureProviderEntry& Entry : CaptureProviders) if (Entry.Provider) Entry.Provider->RegisterCamera(CameraId, CameraActor);
}

bool UBskRenderWorldSubsystem::RequestCapture(const FBskCaptureRequest& Request, FString& OutError)
{
    check(IsInGameThread());
    TArray<FString> Errors;
    for (const FCaptureProviderEntry& Entry : CaptureProviders)
    {
        if (!Entry.Provider) continue;
        FString ProviderError;
        if (Entry.Provider->RequestCapture(Request, ProviderError)) return true;
        if (!ProviderError.IsEmpty()) Errors.Add(FString::Printf(TEXT("%s: %s"), *Entry.Name.ToString(), *ProviderError));
    }
    OutError = CaptureProviders.IsEmpty() ? TEXT("no BSK capture provider is registered") : FString::Join(Errors, TEXT("; "));
    return false;
}
