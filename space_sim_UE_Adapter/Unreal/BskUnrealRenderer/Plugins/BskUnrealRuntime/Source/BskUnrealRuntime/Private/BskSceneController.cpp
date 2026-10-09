#include "BskSceneController.h"
#include "BskPreviewCadence.h"
#include "BskLeRobotSampling.h"
#include "BskCelestialLighting.h"

#include "BskTcpReceiver.h"
#include "BskRenderWorldSubsystem.h"
#include "BskReplaySource.h"
#include "BskCameraPawn.h"
#include "BskUnrealRuntime.h"
#include "Camera/CameraActor.h"
#include "Camera/CameraComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/LightComponent.h"
#include "Components/PointLightComponent.h"
#include "Components/SpotLightComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Components/MeshComponent.h"
#include "Components/SceneComponent.h"
#include "Components/SkyAtmosphereComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Particles/ParticleSystem.h"
#include "Particles/ParticleSystemComponent.h"
#include "Dom/JsonObject.h"
#include "DrawDebugHelpers.h"
#include "Engine/DirectionalLight.h"
#include "Engine/Light.h"
#include "Engine/PointLight.h"
#include "Engine/SpotLight.h"
#include "Engine/Texture.h"
#include "Engine/PostProcessVolume.h"
#include "Engine/Scene.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/SkeletalMesh.h"
#include "Animation/SkeletalMeshActor.h"
#include "Async/Async.h"
#include "Engine/StaticMesh.h"
#include "AssetCompilingManager.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "HAL/PlatformTime.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformProcess.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"
#include "HAL/FileManager.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "IPAddress.h"
#include "Kismet/GameplayStatics.h"
#include "GameFramework/PlayerController.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Misc/CommandLine.h"
#include "Misc/ConfigCacheIni.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/Parse.h"
#include "Modules/ModuleManager.h"
#include "IPixelStreaming2Module.h"
#include "IPixelStreaming2Streamer.h"
#include "IPixelStreaming2VideoProducer.h"
#include "PixelStreaming2Delegates.h"
#include "VideoProducerRenderTarget.h"
#include "RHICommandList.h"
#include "RHIGPUReadback.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/MemoryWriter.h"
#include "SocketSubsystem.h"
#include "Sockets.h"
#include "TimerManager.h"
#include "UnrealClient.h"
#include "UObject/UnrealType.h"

class FBskCaptureNetworkSender final : public FRunnable
{
public:
    FBskCaptureNetworkSender(FString InAddress, uint16 InPort)
        : Address(MoveTemp(InAddress)), Port(InPort)
    {
        WakeEvent = FPlatformProcess::GetSynchEventFromPool(false);
        Thread = FRunnableThread::Create(this, TEXT("BskCaptureOutput"));
    }

    virtual ~FBskCaptureNetworkSender() override
    {
        Stop();
        if (Thread)
        {
            Thread->WaitForCompletion();
            delete Thread;
        }
        if (WakeEvent) FPlatformProcess::ReturnSynchEventToPool(WakeEvent);
    }

    void EnqueueLatest(const FString& StreamKey, TArray<uint8>&& Packet)
    {
        {
            FScopeLock Lock(&PacketMutex);
            PendingPreviewPackets.Add(StreamKey, MoveTemp(Packet));
        }
        WakeEvent->Trigger();
    }

    bool HasReliableCapacity(int32 PacketCount, uint64 ByteBudget)
    {
        FScopeLock Lock(&PacketMutex);
        return !bStopRequested && OutstandingReliablePackets + PacketCount <= MaxReliablePackets
            && OutstandingReliableBytes + ByteBudget <= MaxReliableBytes;
    }

    int32 GetPendingPacketCount()
    {
        FScopeLock Lock(&PacketMutex);
        return OutstandingReliablePackets;
    }

    bool EnqueueReliable(TArray<uint8>&& Packet)
    {
        // Admission is checked before consuming a render frame. Never wait for
        // the network on the game thread, and never build an unbounded FIFO.
        {
            FScopeLock Lock(&PacketMutex);
            if (bStopRequested || OutstandingReliablePackets >= MaxReliablePackets
                || OutstandingReliableBytes + Packet.Num() > MaxReliableBytes) return false;
            ++OutstandingReliablePackets;
            OutstandingReliableBytes += Packet.Num();
            PendingReliablePackets.Add(MoveTemp(Packet));
        }
        WakeEvent->Trigger();
        return true;
    }

    virtual uint32 Run() override
    {
        TArray<uint8> Packet;
        bool bReliablePacket = false;
        while (!bStopRequested)
        {
            if (Packet.IsEmpty())
            {
                FScopeLock Lock(&PacketMutex);
                if (!PendingReliablePackets.IsEmpty())
                {
                    Packet = MoveTemp(PendingReliablePackets[0]);
                    PendingReliablePackets.RemoveAt(0, 1, EAllowShrinking::No);
                    bReliablePacket = true;
                }
                else if (!PendingPreviewPackets.IsEmpty())
                {
                    auto Iterator = PendingPreviewPackets.CreateIterator();
                    Packet = MoveTemp(Iterator.Value());
                    Iterator.RemoveCurrent();
                    bReliablePacket = false;
                }
            }
            if (Packet.IsEmpty())
            {
                WakeEvent->Wait(100);
                continue;
            }
            if (!EnsureConnected() || !SendAll(Packet) || (bReliablePacket && !WaitForAcknowledgement()))
            {
                CloseSocket();
                // Retain authoritative packets across connection startup/retry.
                // Preview remains intentionally lossy and latest-wins.
                if (!bReliablePacket) Packet.Reset();
                FPlatformProcess::Sleep(0.1f);
                continue;
            }
            if (bReliablePacket)
            {
                FScopeLock Lock(&PacketMutex);
                --OutstandingReliablePackets;
                OutstandingReliableBytes -= Packet.Num();
            }
            Packet.Reset();
        }
        CloseSocket();
        return 0;
    }

    virtual void Stop() override
    {
        bStopRequested = true;
        if (WakeEvent) WakeEvent->Trigger();
    }

private:
    bool EnsureConnected()
    {
        if (Socket) return true;
        ISocketSubsystem* SocketSubsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
        if (!SocketSubsystem) return false;
        FAddressInfoResult AddressResult = SocketSubsystem->GetAddressInfo(
            *Address, nullptr, EAddressInfoFlags::Default, NAME_None, ESocketType::SOCKTYPE_Streaming);
        if (AddressResult.Results.IsEmpty())
        {
            UE_LOG(LogBskUnreal, Warning, TEXT("BSK capture output could not resolve %s:%u"), *Address, Port);
            return false;
        }
        TSharedPtr<FInternetAddr> InternetAddress = AddressResult.Results[0].Address;
        InternetAddress->SetPort(Port);
        Socket = SocketSubsystem->CreateSocket(NAME_Stream, TEXT("BSK capture output"), InternetAddress->GetProtocolType());
        if (!Socket || !Socket->Connect(*InternetAddress))
        {
            static double LastConnectWarningSeconds = 0.0;
            const double Now = FPlatformTime::Seconds();
            if (Now - LastConnectWarningSeconds >= 5.0)
            {
                UE_LOG(LogBskUnreal, Warning, TEXT("BSK capture output cannot connect to %s:%u"), *Address, Port);
                LastConnectWarningSeconds = Now;
            }
            CloseSocket();
            return false;
        }
        UE_LOG(LogBskUnreal, Display, TEXT("BSK capture output connected to %s:%u"), *Address, Port);
        Socket->SetNoDelay(true);
        Socket->SetSendBufferSize(16 * 1024 * 1024, SendBufferBytes);
        Socket->SetNonBlocking(true);
        return true;
    }

    bool WaitForAcknowledgement()
    {
        if (!Socket || !Socket->Wait(ESocketWaitConditions::WaitForRead, FTimespan::FromSeconds(5.0))) return false;
        uint8 Ack = 0;
        int32 Received = 0;
        return Socket->Recv(&Ack, 1, Received) && Received == 1 && Ack == 1;
    }

    bool SendAll(const TArray<uint8>& Packet)
    {
        int32 Offset = 0;
        double Progress = FPlatformTime::Seconds();
        while (!bStopRequested && Socket && Offset < Packet.Num())
        {
            if (FPlatformTime::Seconds() - Progress > 5.0) return false;
            if (!Socket->Wait(ESocketWaitConditions::WaitForWrite, FTimespan::FromSeconds(0.05))) continue;
            int32 Sent = 0;
            if (!Socket->Send(Packet.GetData() + Offset, Packet.Num() - Offset, Sent))
            {
                if (ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->GetLastErrorCode() == SE_EWOULDBLOCK) continue;
                return false;
            }
            if (Sent <= 0) return false;
            Offset += Sent;
            Progress = FPlatformTime::Seconds();
        }
        return Offset == Packet.Num();
    }

    void CloseSocket()
    {
        if (!Socket) return;
        Socket->Close();
        ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(Socket);
        Socket = nullptr;
    }

    FString Address;
    uint16 Port = 0;
    FThreadSafeBool bStopRequested = false;
    FCriticalSection PacketMutex;
    static constexpr int32 MaxReliablePackets = 128;
    static constexpr uint64 MaxReliableBytes = 256ull * 1024 * 1024;
    int32 OutstandingReliablePackets = 0;
    uint64 OutstandingReliableBytes = 0;
    TArray<TArray<uint8>> PendingReliablePackets;
    TMap<FString, TArray<uint8>> PendingPreviewPackets;
    FEvent* WakeEvent = nullptr;
    FRunnableThread* Thread = nullptr;
    FSocket* Socket = nullptr;
    int32 SendBufferBytes = 0;
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

struct FCapturedDataProduct
{
    FString Name;
    FString Encoding;
    FString Extension;
    TArray<uint8> Bytes;
};

// All UObject/camera access and metadata snapshots stay on the game thread.
// Workers own only pixel buffers and an independent JPEG wrapper. GPU commands
// retain the job/readback until submission; no render target is read after reuse.
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

struct FBskCaptureDiskWork
{
    FString Directory;
    FString Stem;
    FString MetadataJson;
    TArray<FCapturedDataProduct> Products;
};

class FBskCaptureDiskWriter final : public FRunnable
{
public:
    FBskCaptureDiskWriter()
    {
        WakeEvent = FPlatformProcess::GetSynchEventFromPool(false);
        Thread = FRunnableThread::Create(this, TEXT("BskCaptureDiskOutput"));
    }

    virtual ~FBskCaptureDiskWriter() override
    {
        Stop();
        if (Thread)
        {
            Thread->WaitForCompletion();
            delete Thread;
        }
        if (WakeEvent) FPlatformProcess::ReturnSynchEventToPool(WakeEvent);
    }

    bool HasCapacity(int32 Count)
    {
        FScopeLock Lock(&WorkMutex);
        return !bStopRequested && PendingWork.Num() + Count <= MaxPendingWork;
    }

    bool EnqueueReliable(FBskCaptureDiskWork&& Work)
    {
        {
            FScopeLock Lock(&WorkMutex);
            if (PendingWork.Num() >= MaxPendingWork) return false;
            PendingWork.Add(MoveTemp(Work));
        }
        WakeEvent->Trigger();
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
            IFileManager::Get().MakeDirectory(*Work.Directory, true);
            FFileHelper::SaveStringToFile(Work.MetadataJson,
                *FPaths::Combine(Work.Directory, Work.Stem + TEXT(".json")),
                FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
            for (FCapturedDataProduct& Product : Work.Products)
            {
                FFileHelper::SaveArrayToFile(Product.Bytes, *FPaths::Combine(Work.Directory,
                    FString::Printf(TEXT("%s_%s.%s"), *Work.Stem, *Product.Name, *Product.Extension)));
            }
        }
        return 0;
    }

    virtual void Stop() override
    {
        bStopRequested = true;
        if (WakeEvent) WakeEvent->Trigger();
    }

private:
    FThreadSafeBool bStopRequested = false;
    static constexpr int32 MaxPendingWork = 16;
    FCriticalSection WorkMutex;
    TArray<FBskCaptureDiskWork> PendingWork;
    FEvent* WakeEvent = nullptr;
    FRunnableThread* Thread = nullptr;
};

namespace
{

FString SafePathSegment(FString Value)
{
    for (const TCHAR Invalid : FString(TEXT("/\\:*?\"<>|"))) Value.ReplaceCharInline(Invalid, TEXT('_'));
    Value.TrimStartAndEndInline();
    return Value.IsEmpty() ? TEXT("unnamed") : Value;
}

int64 UnixTimeNanoseconds()
{
    static const FDateTime UnixEpoch(1970, 1, 1);
    return (FDateTime::UtcNow().GetTicks() - UnixEpoch.GetTicks()) * ETimespan::NanosecondsPerTick;
}

void AddBigEndianUint32(TArray<uint8>& Bytes, uint32 Value)
{
    Bytes.Add(static_cast<uint8>((Value >> 24) & 0xff));
    Bytes.Add(static_cast<uint8>((Value >> 16) & 0xff));
    Bytes.Add(static_cast<uint8>((Value >> 8) & 0xff));
    Bytes.Add(static_cast<uint8>(Value & 0xff));
}

bool CompressPng(const TArray<FColor>& Pixels, int32 Width, int32 Height, TArray<uint8>& OutBytes)
{
    IImageWrapperModule& Module = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
    const TSharedPtr<IImageWrapper> Wrapper = Module.CreateImageWrapper(EImageFormat::PNG);
    if (!Wrapper.IsValid() || !Wrapper->SetRaw(Pixels.GetData(), Pixels.Num() * sizeof(FColor), Width, Height, ERGBFormat::BGRA, 8)) return false;
    const TArray64<uint8>& Compressed = Wrapper->GetCompressed(-3);
    if (Compressed.Num() > MAX_int32) return false;
    OutBytes.Append(Compressed.GetData(), static_cast<int32>(Compressed.Num()));
    return true;
}

bool CompressJpeg(const TArray<FColor>& Pixels, int32 Width, int32 Height, int32 Quality, TArray<uint8>& OutBytes)
{
    IImageWrapperModule& Module = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
    const TSharedPtr<IImageWrapper> Wrapper = Module.CreateImageWrapper(EImageFormat::JPEG);
    if (!Wrapper.IsValid() || !Wrapper->SetRaw(Pixels.GetData(), Pixels.Num() * sizeof(FColor), Width, Height, ERGBFormat::BGRA, 8)) return false;
    const TArray64<uint8>& Compressed = Wrapper->GetCompressed(FMath::Clamp(Quality, 1, 100));
    if (Compressed.Num() > MAX_int32) return false;
    OutBytes.Append(Compressed.GetData(), static_cast<int32>(Compressed.Num()));
    return true;
}


TArray<TSharedPtr<FJsonValue>> JsonVector(const FVector3d& Value)
{
    return {MakeShared<FJsonValueNumber>(Value.X), MakeShared<FJsonValueNumber>(Value.Y), MakeShared<FJsonValueNumber>(Value.Z)};
}

TArray<TSharedPtr<FJsonValue>> JsonQuaternionWxyz(const FQuat4d& Value)
{
    return {MakeShared<FJsonValueNumber>(Value.W), MakeShared<FJsonValueNumber>(Value.X), MakeShared<FJsonValueNumber>(Value.Y), MakeShared<FJsonValueNumber>(Value.Z)};
}

FQuat4d UnrealToActiveLocalWxyz(FQuat4d Value, bool bMirrorY)
{
    Value.Normalize();
    return bMirrorY ? FQuat4d(-Value.X, Value.Y, -Value.Z, Value.W) : Value;
}

void QuaternionMatrix(const FQuat4d& Q, double Out[3][3])
{
    const double X = Q.X, Y = Q.Y, Z = Q.Z, W = Q.W;
    Out[0][0] = 1.0 - 2.0 * (Y * Y + Z * Z); Out[0][1] = 2.0 * (X * Y - Z * W); Out[0][2] = 2.0 * (X * Z + Y * W);
    Out[1][0] = 2.0 * (X * Y + Z * W); Out[1][1] = 1.0 - 2.0 * (X * X + Z * Z); Out[1][2] = 2.0 * (Y * Z - X * W);
    Out[2][0] = 2.0 * (X * Z - Y * W); Out[2][1] = 2.0 * (Y * Z + X * W); Out[2][2] = 1.0 - 2.0 * (X * X + Y * Y);
}

TArray<TSharedPtr<FJsonValue>> JsonMatrix3(const double Matrix[3][3])
{
    TArray<TSharedPtr<FJsonValue>> Values;
    Values.Reserve(9);
    for (int32 Row = 0; Row < 3; ++Row) for (int32 Column = 0; Column < 3; ++Column) Values.Add(MakeShared<FJsonValueNumber>(Matrix[Row][Column]));
    return Values;
}

bool JsonVector3(const TSharedPtr<FJsonObject>& Object, const TCHAR* Name, FVector3d& Out)
{
    const TArray<TSharedPtr<FJsonValue>>* Values = nullptr;
    if (!Object.IsValid() || !Object->TryGetArrayField(Name, Values) || Values == nullptr || Values->Num() != 3)
    {
        return false;
    }
    Out = FVector3d((*Values)[0]->AsNumber(), (*Values)[1]->AsNumber(), (*Values)[2]->AsNumber());
    return true;
}

bool JsonColor(const TSharedPtr<FJsonObject>& Object, const TCHAR* Name, FLinearColor& Out)
{
    const TArray<TSharedPtr<FJsonValue>>* Values = nullptr;
    if (!Object.IsValid() || !Object->TryGetArrayField(Name, Values) || Values == nullptr || Values->Num() < 3)
    {
        return false;
    }
    Out = FLinearColor(
        static_cast<float>((*Values)[0]->AsNumber()),
        static_cast<float>((*Values)[1]->AsNumber()),
        static_cast<float>((*Values)[2]->AsNumber()),
        Values->Num() >= 4 ? static_cast<float>((*Values)[3]->AsNumber()) : 1.0f);
    return true;
}

FString BindingTag(const FString& Name)
{
    return FString::Printf(TEXT("BSK.%s"), *Name);
}

FName SafeActorName(const FString& Value)
{
    FString Result = Value;
    Result.ReplaceInline(TEXT("/"), TEXT("_"));
    Result.ReplaceInline(TEXT("\\"), TEXT("_"));
    Result.ReplaceInline(TEXT("."), TEXT("_"));
    return FName(*FString::Printf(TEXT("%s_H%08X"), *Result, GetTypeHash(Value)));
}

const TCHAR* PrimitiveMeshPath(const FString& Shape)
{
    if (Shape.Equals(TEXT("sphere"), ESearchCase::IgnoreCase) || Shape.Equals(TEXT("ellipsoid"), ESearchCase::IgnoreCase))
    {
        return TEXT("/Engine/BasicShapes/Sphere.Sphere");
    }
    if (Shape.Equals(TEXT("cylinder"), ESearchCase::IgnoreCase) || Shape.Equals(TEXT("capsule"), ESearchCase::IgnoreCase))
    {
        return TEXT("/Engine/BasicShapes/Cylinder.Cylinder");
    }
    if (Shape.Equals(TEXT("cone"), ESearchCase::IgnoreCase))
    {
        return TEXT("/Engine/BasicShapes/Cone.Cone");
    }
    return TEXT("/Engine/BasicShapes/Cube.Cube");
}

bool PrepareStaticMeshForRuntime(UStaticMesh* Mesh, const FString& AssetPath)
{
    if (!Mesh) return false;
    if (Mesh->IsCompiling())
    {
        TArray<UObject*> Assets;
        Assets.Add(Mesh);
        UE_LOG(LogBskUnreal, Display,
            TEXT("Waiting for static mesh compilation before starting frame intake: %s"), *AssetPath);
        FAssetCompilingManager::Get().FinishCompilationForObjects(Assets);
    }
    if (!Mesh->HasValidRenderData())
    {
        UE_LOG(LogBskUnreal, Warning,
            TEXT("Static mesh has no valid render data after preparation: %s"), *AssetPath);
        return false;
    }
    return true;
}

void PreloadConfiguredVisualOverlays()
{
    if (!GConfig) return;
    bool bEnabled = false;
    GConfig->GetBool(TEXT("Bsk.VisualOverlays"), TEXT("Enabled"), bEnabled, GGameIni);
    if (!bEnabled) return;
    const FConfigSection* Section = GConfig->GetSection(TEXT("Bsk.VisualOverlays"), false, GGameIni);
    if (!Section) return;
    for (const TPair<FName, FConfigValue>& Entry : *Section)
    {
        if (Entry.Key == FName(TEXT("Enabled"))) continue;
        const FString AssetPath = Entry.Value.GetValue();
        if (AssetPath.IsEmpty()) continue;
        UStaticMesh* Mesh = LoadObject<UStaticMesh>(nullptr, *AssetPath);
        if (!Mesh)
        {
            UE_LOG(LogBskUnreal, Warning, TEXT("Configured visual overlay is unavailable during preload: %s"), *AssetPath);
            continue;
        }
        if (PrepareStaticMeshForRuntime(Mesh, AssetPath))
        {
            UE_LOG(LogBskUnreal, Display, TEXT("Preloaded visual overlay before frame intake: %s"), *AssetPath);
        }
    }
}

void PreloadRuntimeStaticMeshes()
{
    // Manifest objects are created on the game thread when the first scene
    // manifest arrives. Loading 100+ generated SARM meshes at that point can
    // block the game thread long enough for the reliable render FIFO to fill,
    // which makes the authoritative simulator abort before any RGB frame is
    // captured. Resolve the packaged .uasset files before starting frame
    // intake instead of relying only on the editor asset-registry cache.
    const FString ContentRoot = FPaths::ConvertRelativePathToFull(FPaths::ProjectContentDir());
    const TArray<FString> Roots = {
        TEXT("BSK/Generated/SARM"),
        TEXT("BSK/VisualOverlays"),
        TEXT("_GENERATED/Hyperlovimia")};

    TSet<FString> SeenPaths;
    TArray<UStaticMesh*> Meshes;
    int32 CandidateCount = 0;
    for (const FString& Root : Roots)
    {
        TArray<FString> Files;
        const FString RootPath = FPaths::Combine(ContentRoot, Root);
        IFileManager::Get().FindFilesRecursive(Files, *RootPath, TEXT("*.uasset"), true, false);
        CandidateCount += Files.Num();
        for (const FString& FilePath : Files)
        {
            FString RelativePath = FilePath;
            if (!FPaths::MakePathRelativeTo(RelativePath, *ContentRoot)) continue;
            RelativePath.ReplaceInline(TEXT("\\"), TEXT("/"));
            if (!RelativePath.EndsWith(TEXT(".uasset"), ESearchCase::IgnoreCase)) continue;
            const FString BaseName = FPaths::GetBaseFilename(RelativePath);
            if (BaseName.StartsWith(TEXT("M_")) || BaseName.StartsWith(TEXT("MI_"))) continue;
            RelativePath.LeftChopInline(7, EAllowShrinking::No);
            const FString ObjectPath = FString::Printf(
                TEXT("/Game/%s.%s"), *RelativePath, *FPaths::GetBaseFilename(RelativePath));
            UStaticMesh* Mesh = LoadObject<UStaticMesh>(nullptr, *ObjectPath);
            if (!Mesh) continue;
            const FString AssetPath = Mesh->GetPathName();
            if (SeenPaths.Contains(AssetPath)) continue;
            SeenPaths.Add(AssetPath);
            Meshes.Add(Mesh);
        }
    }

    if (Meshes.IsEmpty())
    {
        UE_LOG(LogBskUnreal, Warning,
            TEXT("No runtime static meshes were found during preloading (asset candidates=%d)"),
            CandidateCount);
        return;
    }

    TArray<UObject*> CompilingMeshes;
    CompilingMeshes.Reserve(Meshes.Num());
    for (UStaticMesh* Mesh : Meshes)
    {
        if (Mesh && Mesh->IsCompiling()) CompilingMeshes.Add(Mesh);
    }
    if (!CompilingMeshes.IsEmpty())
    {
        UE_LOG(LogBskUnreal, Display,
            TEXT("Waiting for %d runtime static meshes before starting frame intake (candidates=%d)"),
            CompilingMeshes.Num(), CandidateCount);
        FAssetCompilingManager::Get().FinishCompilationForObjects(CompilingMeshes);
    }

    int32 ReadyCount = 0;
    for (UStaticMesh* Mesh : Meshes)
    {
        if (Mesh && Mesh->HasValidRenderData()) ++ReadyCount;
    }
    UE_LOG(LogBskUnreal, Display,
        TEXT("Preloaded runtime static meshes before frame intake: ready=%d total=%d candidates=%d"),
        ReadyCount, Meshes.Num(), CandidateCount);
}

void ApplyUnlitColor(UStaticMeshComponent* Component, UObject* Owner, const FLinearColor& Color)
{
    if (!Component) return;
    if (UMaterialInterface* Base = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/BSK/M_BskUnlitColor.M_BskUnlitColor")))
    {
        UMaterialInstanceDynamic* Material = UMaterialInstanceDynamic::Create(Base, Owner);
        Material->SetVectorParameterValue(TEXT("Color"), Color);
        Component->SetMaterial(0, Material);
    }
}

// Exact mesh-object-path overrides are visual only. They deliberately do not use
// numeric geom indices (which change between MJCF variants) or broad name matches.
bool ApplyConfiguredMaterialOverride(UStaticMeshComponent* Component, const FBskGeometryDefinition& Geometry)
{
    if (!GConfig || Geometry.AssetPath.IsEmpty() || Geometry.RenderRole != TEXT("visual") ||
        FParse::Param(FCommandLine::Get(), TEXT("BskDisableMaterialOverrides"))) return false;
    bool bEnabled = false;
    GConfig->GetBool(TEXT("Bsk.MaterialOverrides"), TEXT("Enabled"), bEnabled, GGameIni);
    if (!bEnabled) return false;
    FString MaterialPath;
    if (!GConfig->GetString(TEXT("Bsk.MaterialOverrides"), *Geometry.AssetPath, MaterialPath, GGameIni) ||
        MaterialPath.IsEmpty()) return false;
    UMaterialInterface* Material = LoadObject<UMaterialInterface>(nullptr, *MaterialPath);
    if (!Material)
    {
        UE_LOG(LogBskUnreal, Warning, TEXT("Material override missing: %s for %s; using original material"),
            *MaterialPath, *Geometry.AssetPath);
        return false;
    }
    const int32 SlotCount = FMath::Max(1, Component->GetNumMaterials());
    for (int32 Slot = 0; Slot < SlotCount; ++Slot) Component->SetMaterial(Slot, Material);
    UE_LOG(LogBskUnreal, Log, TEXT("Applied visual material override %s to %s (%s)"),
        *MaterialPath, *Geometry.AssetPath, *Geometry.GeometryId);
    return true;
}

// Render-only skins are children of the original mesh component. This preserves
// body transforms and component scaling without changing source geometry/physics.
void AttachConfiguredVisualOverlay(UStaticMeshComponent* Parent, const FBskGeometryDefinition& Geometry)
{
    if (!Parent || !GConfig || Geometry.AssetPath.IsEmpty() || Geometry.RenderRole == TEXT("collision") ||
        FParse::Param(FCommandLine::Get(), TEXT("BskDisableVisualOverlays"))) return;
    bool bEnabled = false;
    GConfig->GetBool(TEXT("Bsk.VisualOverlays"), TEXT("Enabled"), bEnabled, GGameIni);
    if (!bEnabled) return;
    FString MeshPath;
    if (!GConfig->GetString(TEXT("Bsk.VisualOverlays"), *Geometry.AssetPath, MeshPath, GGameIni) || MeshPath.IsEmpty()) return;
    UStaticMesh* Mesh = LoadObject<UStaticMesh>(nullptr, *MeshPath);
    if (!Mesh)
    {
        UE_LOG(LogBskUnreal, Warning, TEXT("Visual overlay missing: %s for %s; original mesh retained"), *MeshPath, *Geometry.AssetPath);
        return;
    }
    if (!PrepareStaticMeshForRuntime(Mesh, MeshPath))
    {
        UE_LOG(LogBskUnreal, Warning, TEXT("Visual overlay is not render-ready: %s; original mesh retained"), *MeshPath);
        return;
    }
    AActor* Owner = Parent->GetOwner();
    UStaticMeshComponent* Overlay = NewObject<UStaticMeshComponent>(Owner,
        MakeUniqueObjectName(Owner, UStaticMeshComponent::StaticClass(), TEXT("VisualOverlay")));
    Overlay->SetMobility(EComponentMobility::Movable);
    Overlay->SetStaticMesh(Mesh); // Keep the overlay's own UV material slots.
    Overlay->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Overlay->SetGenerateOverlapEvents(false);
    Overlay->SetCanEverAffectNavigation(false);
    Overlay->SetCastShadow(true);
    Overlay->SetupAttachment(Parent);
    Overlay->SetRelativeTransform(FTransform::Identity);
    Owner->AddInstanceComponent(Overlay);
    Overlay->RegisterComponent();
    UE_LOG(LogBskUnreal, Log, TEXT("Attached visual-only overlay %s to %s (%s)"), *MeshPath, *Geometry.AssetPath, *Geometry.GeometryId);
}

void ApplyGeometryMaterial(UStaticMeshComponent* Component, UObject* Owner, const FBskGeometryDefinition& Geometry, double Ambient)
{
    if (!Component) return;
    if (ApplyConfiguredMaterialOverride(Component, Geometry)) return;
    if (Geometry.bUseAssetMaterials && Geometry.MaterialName.IsEmpty()) return;
    const bool bTranslucent = Geometry.Color.A < 0.999f;
    const TCHAR* MaterialPath = bTranslucent
        ? TEXT("/Game/BSK/M_BskPbrTranslucent.M_BskPbrTranslucent")
        : TEXT("/Game/BSK/M_BskPbrOpaque.M_BskPbrOpaque");
    UMaterialInterface* Base = LoadObject<UMaterialInterface>(nullptr, MaterialPath);
    if (!Base)
    {
        ApplyUnlitColor(Component, Owner, Geometry.Color);
        return;
    }

    UMaterialInstanceDynamic* Material = UMaterialInstanceDynamic::Create(Base, Owner);
    const double BlinnExponent = FMath::Clamp(Geometry.MaterialShininess, 0.0, 1.0) * 128.0;
    const float Roughness = static_cast<float>(FMath::Clamp(FMath::Sqrt(2.0 / (BlinnExponent + 2.0)), 0.08, 1.0));
    const float Specular = static_cast<float>(FMath::Clamp(
        Geometry.MaterialSpecular + Geometry.MaterialReflectance * (1.0 - Geometry.MaterialSpecular), 0.0, 1.0));
    Material->SetVectorParameterValue(TEXT("BaseColor"), Geometry.Color);
    Material->SetScalarParameterValue(TEXT("Roughness"), Roughness);
    Material->SetScalarParameterValue(TEXT("Specular"), Specular);
    Material->SetScalarParameterValue(TEXT("Metallic"), 0.0f);
    Material->SetScalarParameterValue(TEXT("Emission"), static_cast<float>(FMath::Max(0.0, Geometry.MaterialEmission)));
    Material->SetScalarParameterValue(TEXT("Ambient"), static_cast<float>(FMath::Max(0.0, Ambient)));
    Material->SetScalarParameterValue(TEXT("Opacity"), Geometry.Color.A);
    Material->SetScalarParameterValue(TEXT("UseTexture"), 0.0f);
    Material->SetVectorParameterValue(TEXT("TextureRepeat"), FLinearColor(
        static_cast<float>(Geometry.MaterialTextureRepeat.X),
        static_cast<float>(Geometry.MaterialTextureRepeat.Y), 0.0f, 0.0f));
    if (!Geometry.MaterialTextureAssetPath.IsEmpty())
    {
        if (UTexture* Texture = LoadObject<UTexture>(nullptr, *Geometry.MaterialTextureAssetPath))
        {
            Material->SetTextureParameterValue(TEXT("BaseColorTexture"), Texture);
            Material->SetScalarParameterValue(TEXT("UseTexture"), 1.0f);
        }
        else
        {
            UE_LOG(LogBskUnreal, Warning, TEXT("Could not load packaged MJCF texture %s for %s"),
                *Geometry.MaterialTextureAssetPath, *Geometry.GeometryId);
        }
    }
    const int32 SlotCount = FMath::Max(1, Component->GetNumMaterials());
    for (int32 Slot = 0; Slot < SlotCount; ++Slot) Component->SetMaterial(Slot, Material);
}

bool IsSensorFrustumKind(const FString& Kind)
{
    return Kind.Equals(TEXT("css"), ESearchCase::IgnoreCase) ||
        Kind.Equals(TEXT("generic_sensor"), ESearchCase::IgnoreCase) ||
        Kind.Equals(TEXT("transceiver"), ESearchCase::IgnoreCase);
}
}

ABskSceneController::ABskSceneController()
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.TickGroup = TG_PrePhysics;
    SetRootComponent(CreateDefaultSubobject<USceneComponent>(TEXT("Root")));
    VisualKindVisibility.Add(TEXT("css"), false);
    VisualKindVisibility.Add(TEXT("generic_sensor"), false);
    VisualKindVisibility.Add(TEXT("transceiver"), false);
}

ABskSceneController::~ABskSceneController() = default;

void ABskSceneController::BeginPlay()
{
    Super::BeginPlay();
    LoadConfiguration();
    ConfigureCaptureOutput();
    ConfigurePixelStreamingOutput();
    CreateEnvironment();
    // Do not start accepting authoritative simulation frames until configured
    // visual overlays have finished async static-mesh compilation. Otherwise
    // the first manifest/frame can stall the game thread while the Basilisk
    // sender fills its reliable dataset queue and aborts capture.
    PreloadRuntimeStaticMeshes();
    PreloadConfiguredVisualOverlays();
    if (UBskRenderWorldSubsystem* RenderSubsystem = GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>())
    {
        BuiltinCaptureProvider = MakeShared<FBskBuiltinCaptureProvider>(this);
        if (!RenderSubsystem->RegisterCaptureProvider(TEXT("Bsk.BuiltinSceneCapture"), BuiltinCaptureProvider.ToSharedRef(), 0))
        {
            UE_LOG(LogBskUnreal, Error, TEXT("Could not register the built-in BSK capture provider"));
            BuiltinCaptureProvider.Reset();
        }
    }
    if (!ReplayPath.IsEmpty()) Receiver = MakeUnique<FBskReplaySource>(ReplayPath, ReplayRate);
    else Receiver = MakeUnique<FBskTcpReceiver>(ListenAddress, static_cast<uint16>(ListenPort), MaxPacketBytes,
        !CaptureOutputDirectory.IsEmpty() || CaptureNetworkSender.IsValid());
    Receiver->StartSource();
}

void ABskSceneController::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    CancelCaptureJobs();
    ShutdownPixelStreamingCameras();
    if (BuiltinCaptureProvider && GetWorld())
    {
        if (UBskRenderWorldSubsystem* RenderSubsystem = GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>())
        {
            RenderSubsystem->UnregisterCaptureProvider(TEXT("Bsk.BuiltinSceneCapture"));
        }
        BuiltinCaptureProvider.Reset();
    }
    CaptureNetworkSender.Reset();
    CaptureDiskWriter.Reset();
    if (Receiver)
    {
        Receiver->StopSource();
        Receiver.Reset();
    }
    if (bManagesViewportRendering && GEngine && GEngine->GameViewport)
        GEngine->GameViewport->bDisableWorldRendering = bPreviousDisableWorldRendering;
    Super::EndPlay(EndPlayReason);
}

void ABskSceneController::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
    // Drain after STOP too: accepted strict jobs retain their original episode.
    PollCaptureJobs();
    const double TickStart = bVideoDiagnostics ? FPlatformTime::Seconds() : 0.0;
    check(IsInGameThread());
    if (MaximumStreamingFrameRate > 0)
    {
        // Cache the console variables; resolving them every tick adds measurable
        // overhead at the 90-Hz preview rate and can starve the 30-Hz capture.
        static IConsoleVariable* RequestedRate =
            IConsoleManager::Get().FindConsoleVariable(TEXT("PixelStreaming2.WebRTC.Fps"));
        static IConsoleVariable* RenderRate =
            IConsoleManager::Get().FindConsoleVariable(TEXT("t.MaxFPS"));
        if (RequestedRate && RenderRate)
        {
            const int32 FrameRate = FMath::Clamp(RequestedRate->GetInt(), 1, MaximumStreamingFrameRate);
            if (!FMath::IsNearlyEqual(RenderRate->GetFloat(), static_cast<float>(FrameRate)))
            {
                RenderRate->Set(static_cast<float>(FrameRate), ECVF_SetByConsole);
                UE_LOG(LogBskUnreal, Display, TEXT("Streaming render frame rate changed to %d FPS (launch ceiling %d)"),
                    FrameRate, MaximumStreamingFrameRate);
            }
        }
    }
    if (Receiver)
    {
        FBskSceneManifest Manifest;
        if (Receiver->ConsumeLatestManifest(Manifest)) ApplyManifest(Manifest);
        FBskRenderEvent Event;
        while (Receiver->ConsumeEvent(Event)) ApplyEvent(Event);
        FBskRenderFrame Latest;
        uint64 CaptureBatchByteBudget = 0;
        for (const auto& Camera : ManifestCameras)
        {
            CaptureBatchByteBudget += static_cast<uint64>(FMath::Clamp(Camera.Value.Resolution.X, 64, 4096))
                * FMath::Clamp(Camera.Value.Resolution.Y, 64, 4096) * 4 + 65536;
        }
        const int32 ReservedPackets = PendingCaptureJobs.Num() + ManifestCameras.Num();
        const uint64 ReservedBytes = PendingCaptureBytes + CaptureBatchByteBudget;
        const bool bCaptureHasCapacity = ReservedPackets <= MaxAsyncCaptureJobs
            && ReservedBytes <= MaxAsyncCaptureBytes
            && (!CaptureNetworkSender.IsValid()
                || CaptureNetworkSender->HasReliableCapacity(ReservedPackets, ReservedBytes))
            && (!CaptureDiskWriter.IsValid() || CaptureDiskWriter->HasCapacity(ReservedPackets));
        // Leave authoritative frames in the receiver FIFO until output capacity
        // exists. This applies bounded upstream backpressure without skipping.
        if (Receiver->ConsumeForCapture(Latest, bCaptureHasCapacity))
        {
            const double NowSeconds = FPlatformTime::Seconds();
            FString RejectionReason;
            UBskRenderWorldSubsystem* RenderSubsystem = GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>();
            if (RenderSubsystem && !RenderSubsystem->AcceptFrame(Latest, RejectionReason))
            {
                UE_LOG(LogBskUnreal, Warning, TEXT("Ignoring BSK frame: %s"), *RejectionReason);
            }
            else
            {
                // Dataset products are rendered from the exact received frame before
                // presentation interpolation/extrapolation is allowed to touch it.
                const bool bCaptureActive = (!Latest.bCaptureOnDemand || !Latest.CaptureEpisodeId.IsEmpty())
                    && (!CaptureOutputDirectory.IsEmpty() || CaptureNetworkSender.IsValid());
                const double PreviewRate = bCaptureActive && RequestedPixelStreamingCameraRateHertz > 30.0
                    ? 5.0 : RequestedPixelStreamingCameraRateHertz;
                if (PixelStreamingCameraRateHertz != PreviewRate)
                {
                    PixelStreamingCameraRateHertz = PreviewRate;
                    UE_LOG(LogBskUnreal, Display, TEXT("Camera preview %.1f Hz; strict capture %s episode=%s"),
                        PreviewRate, bCaptureActive ? TEXT("on") : TEXT("off"), *Latest.CaptureEpisodeId);
                }
                UpdateAuthoritativeDataProductCaptures(Latest);
                if (TimeMode.Equals(TEXT("latest"), ESearchCase::IgnoreCase) || !bHasTargetFrame)
                {
                    PreviousFrame = Latest;
                    TargetFrame = Latest;
                    bHasTargetFrame = true;
                    ApplyFrame(Latest);
                }
                else
                {
                    PreviousFrame = TargetFrame;
                    TargetFrame = MoveTemp(Latest);
                    BlendElapsedSeconds = 0.0;
                    if (LastFrameArrivalSeconds > 0.0)
                    {
                        BlendDurationSeconds = FMath::Clamp(NowSeconds - LastFrameArrivalSeconds, 1.0 / 240.0, InterpolationDelaySeconds);
                    }
                }
                LastFrameArrivalSeconds = NowSeconds;
            }
        }
        if (bHasTargetFrame && !TimeMode.Equals(TEXT("latest"), ESearchCase::IgnoreCase) && PreviousFrame.FrameId != TargetFrame.FrameId)
        {
            BlendElapsedSeconds += DeltaSeconds;
            const double BlendDuration = FMath::Max(BlendDurationSeconds, UE_DOUBLE_SMALL_NUMBER);
            const double Alpha = BlendElapsedSeconds / BlendDuration;
            if (Alpha <= 1.0)
            {
                ApplyFrame(InterpolateFrame(PreviousFrame, TargetFrame, Alpha));
            }
            else
            {
                const double ExtrapolationSeconds = FMath::Min(BlendElapsedSeconds - BlendDuration, MaxExtrapolationSeconds);
                ApplyFrame(ExtrapolateFrame(PreviousFrame, TargetFrame, ExtrapolationSeconds));
            }
        }
    }
    UpdateDecorativeSunPlacement();
    UpdatePreviewViewportRendering();
    UpdatePictureInPictureCaptures();
    UpdatePixelStreamingCameraCaptures();
    if (bVideoDiagnostics)
    {
        const double Now = FPlatformTime::Seconds();
        if (VideoDiagnosticsStart <= 0.0) VideoDiagnosticsStart = TickStart;
        VideoDiagnosticsTickWork += Now - TickStart;
        ++VideoDiagnosticsTicks;
        const double Window = Now - VideoDiagnosticsStart;
        if (Window >= 5.0)
        {
            UE_LOG(LogBskUnreal, Display, TEXT("VideoDiagnostics tick_fps=%.1f scene_tick_ms=%.2f"),
                VideoDiagnosticsTicks / Window, 1000.0 * VideoDiagnosticsTickWork / VideoDiagnosticsTicks);
            for (const auto& Pair : VideoDiagnosticsCaptures)
                UE_LOG(LogBskUnreal, Display, TEXT("VideoDiagnostics camera=%s capture_fps=%.1f viewed=%d"),
                    *Pair.Key, Pair.Value / Window, HasPixelStreamingCameraViewers(Pair.Key) ? 1 : 0);
            VideoDiagnosticsStart = Now;
            VideoDiagnosticsTickWork = 0.0;
            VideoDiagnosticsTicks = 0;
            VideoDiagnosticsCaptures.Reset();
        }
    }
    if (GetWorld() && !PendingCommandIds.IsEmpty())
    {
        const double Now = GetWorld()->GetRealTimeSeconds();
        TArray<FString> TimedOut;
        for (const TPair<FString, double>& Pending : PendingCommandIds)
        {
            if (Now - Pending.Value > 10.0) TimedOut.Add(Pending.Key);
        }
        for (const FString& CommandId : TimedOut)
        {
            PendingCommandIds.Remove(CommandId);
            LastCommandStatus = FString::Printf(TEXT("TIMEOUT %s"), *CommandId);
            FBskMissionEventView Timeout;
            Timeout.Sequence = ++CommandSequence;
            Timeout.Kind = TEXT("command_timeout");
            Timeout.Severity = TEXT("error");
            Timeout.Message = FString::Printf(TEXT("No BSK response for %s"), *CommandId);
            if (UBskRenderWorldSubsystem* Subsystem = GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>()) Timeout.SimulationTimeNanoseconds = Subsystem->GetSimulationTimeNanoseconds();
            MissionEventHistory.Add(MoveTemp(Timeout));
        }
    }
}

FString ABskSceneController::GetReceiverStatus() const
{
    return Receiver ? Receiver->GetStatus() : TEXT("not started");
}

bool ABskSceneController::SetReplayPaused(bool bPaused)
{
    return Receiver && Receiver->SetPaused(bPaused);
}

bool ABskSceneController::SetReplayRate(double Rate)
{
    return Receiver && Receiver->SetPlaybackRate(Rate);
}

bool ABskSceneController::StepReplay()
{
    return Receiver && Receiver->StepOnce();
}

bool ABskSceneController::SeekReplayNanoseconds(int64 SimulationTimeNanoseconds)
{
    return Receiver && Receiver->SeekSimulationTime(SimulationTimeNanoseconds);
}

bool ABskSceneController::FocusObject(const FString& ObjectId, bool bFollow)
{
    AActor* Target = BoundActors.FindRef(ObjectId);
    APlayerController* Player = GetWorld() ? GetWorld()->GetFirstPlayerController() : nullptr;
    ABskCameraPawn* Pawn = Player ? Cast<ABskCameraPawn>(Player->GetPawn()) : nullptr;
    if (!Target || !Pawn) return false;
    Pawn->SetCameraTarget(Target, bFollow ? EBskCameraMode::Follow : EBskCameraMode::Orbit);
    return true;
}

bool ABskSceneController::TogglePictureInPictureSlot(int32 Slot)
{
    for (const TPair<FString, FBskCameraDefinition>& Pair : ManifestCameras)
    {
        if (!Pair.Value.bPictureInPicture || Pair.Value.PictureInPictureSlot != Slot) continue;
        const bool bVisible = !CameraPictureInPictureVisibility.FindRef(Pair.Key);
        CameraPictureInPictureVisibility.Add(Pair.Key, bVisible);
        if (bVisible) CameraNextCaptureSeconds.Add(Pair.Key, 0.0);
        return bVisible;
    }
    return false;
}

void ABskSceneController::GetPictureInPictureViews(TArray<FBskPictureInPictureView>& OutViews) const
{
    OutViews.Reset();
    for (const TPair<FString, FBskCameraDefinition>& Pair : ManifestCameras)
    {
        if (!Pair.Value.bPictureInPicture) continue;
        FBskPictureInPictureView View;
        View.CameraId = Pair.Key;
        View.DisplayName = Pair.Value.DisplayName.IsEmpty() ? Pair.Key : Pair.Value.DisplayName;
        View.Slot = Pair.Value.PictureInPictureSlot;
        const bool* bVisible = CameraPictureInPictureVisibility.Find(Pair.Key);
        View.bVisible = bVisible == nullptr || *bVisible;
        View.Texture = CameraRenderTargets.FindRef(Pair.Key);
        OutViews.Add(MoveTemp(View));
    }
    OutViews.Sort([](const FBskPictureInPictureView& Left, const FBskPictureInPictureView& Right)
    {
        if (Left.Slot != Right.Slot) return Left.Slot < Right.Slot;
        return Left.CameraId < Right.CameraId;
    });
}

void ABskSceneController::GetMissionEvents(TArray<FBskMissionEventView>& OutEvents) const
{
    OutEvents = MissionEventHistory;
}

void ABskSceneController::GetUiCommands(TArray<FBskUiCommandDefinition>& OutCommands) const
{
    OutCommands.Reset();
    if (UBskRenderWorldSubsystem* RenderSubsystem = GetWorld() ? GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>() : nullptr)
    {
        if (const FBskSceneManifest* Manifest = RenderSubsystem->GetLatestManifest()) OutCommands = Manifest->UiCommands;
    }
}

bool ABskSceneController::SendUiCommand(
    const FString& Command,
    const FString& TargetId,
    const FString& PayloadJson,
    FString& OutError)
{
    check(IsInGameThread());
    if (!Receiver)
    {
        OutError = TEXT("message source is not running");
        return false;
    }
    TArray<FBskUiCommandDefinition> Commands;
    GetUiCommands(Commands);
    if (!Commands.ContainsByPredicate([&Command](const FBskUiCommandDefinition& Definition)
        { return Definition.Command == Command; }))
    {
        OutError = FString::Printf(TEXT("command '%s' is not declared by the active scene manifest"), *Command);
        return false;
    }
    TSharedPtr<FJsonObject> Payload;
    if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(PayloadJson.IsEmpty() ? TEXT("{}") : PayloadJson), Payload) || !Payload.IsValid())
    {
        OutError = TEXT("command payload must be a JSON object");
        return false;
    }
    const FString CommandId = FString::Printf(TEXT("ue-%lld"), ++CommandSequence);
    UBskRenderWorldSubsystem* RenderSubsystem = GetWorld() ? GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>() : nullptr;
    const int64 SimTime = RenderSubsystem ? RenderSubsystem->GetSimulationTimeNanoseconds() : 0;
    TSharedPtr<FJsonObject> Root = MakeShared<FJsonObject>();
    Root->SetStringField(TEXT("protocol"), BskProtocol::GenericV2);
    Root->SetStringField(TEXT("type"), TEXT("command"));
    Root->SetStringField(TEXT("session_id"), ActiveSessionId);
    Root->SetStringField(TEXT("command_id"), CommandId);
    Root->SetStringField(TEXT("command"), Command);
    Root->SetStringField(TEXT("target_id"), TargetId);
    Root->SetStringField(TEXT("requested_sim_time_ns"), LexToString(SimTime));
    Root->SetObjectField(TEXT("payload"), Payload);
    FString Json;
    FJsonSerializer::Serialize(Root.ToSharedRef(), TJsonWriterFactory<>::Create(&Json));
    if (!Receiver->SendCommandJson(Json, OutError))
    {
        LastCommandStatus = FString::Printf(TEXT("SEND FAILED: %s"), *OutError);
        return false;
    }
    PendingCommandIds.Add(CommandId, GetWorld()->GetRealTimeSeconds());
    LastCommandStatus = FString::Printf(TEXT("PENDING %s"), *Command);
    FBskMissionEventView Sent;
    Sent.Sequence = CommandSequence;
    Sent.Kind = TEXT("command_sent");
    Sent.Severity = TEXT("command");
    Sent.Message = FString::Printf(TEXT("%s -> %s"), *CommandId, *Command);
    Sent.SimulationTimeNanoseconds = SimTime;
    MissionEventHistory.Add(MoveTemp(Sent));
    constexpr int32 MaxMissionEvents = 128;
    if (MissionEventHistory.Num() > MaxMissionEvents) MissionEventHistory.RemoveAt(0, MissionEventHistory.Num() - MaxMissionEvents, EAllowShrinking::No);
    return true;
}

void ABskSceneController::ClearMissionEvents()
{
    MissionEventHistory.Reset();
}

void ABskSceneController::SetVisualKindVisible(const FString& VisualKind, bool bVisible)
{
    const FString NormalizedKind = VisualKind.TrimStartAndEnd().ToLower();
    if (NormalizedKind.IsEmpty()) return;
    VisualKindVisibility.Add(NormalizedKind, bVisible);
    for (const TPair<FString, FBskVisualDefinition>& Pair : ManifestVisuals)
    {
        if (Pair.Value.Kind.Equals(NormalizedKind, ESearchCase::IgnoreCase)) RefreshVisualVisibility(Pair.Key);
    }
}

bool ABskSceneController::ToggleVisualKindVisible(const FString& VisualKind)
{
    const FString NormalizedKind = VisualKind.TrimStartAndEnd().ToLower();
    const bool bNewVisibility = !IsVisualKindVisible(NormalizedKind);
    SetVisualKindVisible(NormalizedKind, bNewVisibility);
    UE_LOG(LogBskUnreal, Display, TEXT("BSK visual helpers kind=%s visible=%s"),
        *NormalizedKind, bNewVisibility ? TEXT("true") : TEXT("false"));
    return bNewVisibility;
}

bool ABskSceneController::IsVisualKindVisible(const FString& VisualKind) const
{
    const FString NormalizedKind = VisualKind.TrimStartAndEnd().ToLower();
    if (const bool* Visibility = VisualKindVisibility.Find(NormalizedKind)) return *Visibility;
    return true;
}

FVector ABskSceneController::GetConfiguredCameraPositionCentimeters() const
{
    return FVector(Converter.LocalMetersToUnrealCentimeters(CameraPositionMeters));
}

FVector ABskSceneController::GetConfiguredCameraLookAtCentimeters() const
{
    return FVector(Converter.LocalMetersToUnrealCentimeters(CameraLookAtMeters));
}

bool ABskSceneController::LoadConfiguration()
{
    FString ConfigPath = FPaths::Combine(FPaths::ProjectConfigDir(), TEXT("bsk_unreal_scene.json"));
    FParse::Value(FCommandLine::Get(), TEXT("BskConfig="), ConfigPath);
    if (FPaths::IsRelative(ConfigPath))
    {
        ConfigPath = FPaths::ConvertRelativePathToFull(ConfigPath);
    }

    FString Json;
    if (!FFileHelper::LoadFileToString(Json, *ConfigPath))
    {
        UE_LOG(LogBskUnreal, Warning, TEXT("Could not read %s; using built-in defaults"), *ConfigPath);
        return false;
    }
    TSharedPtr<FJsonObject> Root;
    if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Json), Root) || !Root.IsValid())
    {
        UE_LOG(LogBskUnreal, Error, TEXT("Invalid scene configuration: %s"), *ConfigPath);
        return false;
    }

    const TSharedPtr<FJsonObject>* Network = nullptr;
    if (Root->TryGetObjectField(TEXT("network"), Network) && Network != nullptr)
    {
        (*Network)->TryGetStringField(TEXT("listen_address"), ListenAddress);
        double Number = 0.0;
        if ((*Network)->TryGetNumberField(TEXT("port"), Number)) ListenPort = FMath::Clamp(static_cast<int32>(Number), 1, 65535);
        if ((*Network)->TryGetNumberField(TEXT("max_packet_bytes"), Number)) MaxPacketBytes = FMath::Clamp(static_cast<uint32>(Number), 1024u, BskProtocol::DefaultMaxPacketBytes);
    }

    const TSharedPtr<FJsonObject>* Replay = nullptr;
    if (Root->TryGetObjectField(TEXT("replay"), Replay) && Replay != nullptr)
    {
        (*Replay)->TryGetStringField(TEXT("path"), ReplayPath);
        (*Replay)->TryGetNumberField(TEXT("rate"), ReplayRate);
    }
    const TSharedPtr<FJsonObject>* Capture = nullptr;
    if (Root->TryGetObjectField(TEXT("capture"), Capture) && Capture != nullptr)
    {
        (*Capture)->TryGetStringField(TEXT("output_directory"), CaptureOutputDirectory);
        (*Capture)->TryGetStringField(TEXT("network_address"), CaptureNetworkAddress);
        double Number = 0.0;
        if ((*Capture)->TryGetNumberField(TEXT("network_port"), Number)) CaptureNetworkPort = FMath::Clamp(static_cast<int32>(Number), 0, 65535);
        if ((*Capture)->TryGetNumberField(TEXT("rate_override_hz"), Number)) CaptureRateOverrideHertz = FMath::Clamp(Number, 0.0, 60.0);
        const TArray<TSharedPtr<FJsonValue>>* Products = nullptr;
        if ((*Capture)->TryGetArrayField(TEXT("products_override"), Products) && Products != nullptr)
        {
            for (const TSharedPtr<FJsonValue>& Value : *Products)
            {
                FString Product;
                if (Value.IsValid() && Value->TryGetString(Product)) CaptureProductOverride.Add(Product.TrimStartAndEnd().ToLower());
            }
        }
    }
    const TSharedPtr<FJsonObject>* Assets = nullptr;
    if (Root->TryGetObjectField(TEXT("assets"), Assets) && Assets != nullptr)
    {
        (*Assets)->TryGetBoolField(TEXT("allow_external_files"), bAllowExternalAssets);
    }

    double CentimetersPerMeter = 100.0;
    bool bMirrorY = true;
    const TSharedPtr<FJsonObject>* Coordinates = nullptr;
    if (Root->TryGetObjectField(TEXT("coordinates"), Coordinates) && Coordinates != nullptr)
    {
        (*Coordinates)->TryGetNumberField(TEXT("centimeters_per_meter"), CentimetersPerMeter);
        (*Coordinates)->TryGetBoolField(TEXT("mirror_local_y_for_unreal"), bMirrorY);
    }
    Converter = FBskCoordinateConverter(CentimetersPerMeter, bMirrorY);

    const TSharedPtr<FJsonObject>* Scene = nullptr;
    if (Root->TryGetObjectField(TEXT("scene"), Scene) && Scene != nullptr)
    {
        double Number = 0.0;
        if ((*Scene)->TryGetNumberField(TEXT("star_count"), Number)) StarCount = FMath::Clamp(static_cast<int32>(Number), 0, 2000);
        if ((*Scene)->TryGetNumberField(TEXT("star_radius_m"), Number)) StarRadiusMeters = FMath::Max(10.0, Number);
        if ((*Scene)->TryGetNumberField(TEXT("celestial_vault_radius_km"), Number)) CelestialVaultRadiusKilometers = FMath::Clamp(Number, 10000.0, 500000.0);
        if ((*Scene)->TryGetNumberField(TEXT("celestial_background_intensity"), Number)) CelestialBackgroundIntensity = FMath::Clamp(Number, 0.0, 8.0);
        if ((*Scene)->TryGetNumberField(TEXT("sun_intensity_lux"), Number)) SunIntensityLux = FMath::Max(0.0, Number);
        if ((*Scene)->TryGetNumberField(TEXT("sun_illuminance_scale"), Number)) SunIlluminanceScale = FMath::Max(0.0, Number);
        if ((*Scene)->TryGetNumberField(TEXT("fill_light_intensity_lux"), Number)) FillLightIntensityLux = FMath::Max(0.0, Number);
        if ((*Scene)->TryGetNumberField(TEXT("material_exposure_bias"), Number)) MaterialExposureBias = FMath::Clamp(Number, -8.0, 8.0);
        if ((*Scene)->TryGetNumberField(TEXT("sun_visual_distance_m"), Number)) SunVisualDistanceMeters = FMath::Max(1000.0, Number);
        if ((*Scene)->TryGetNumberField(TEXT("sun_visual_angular_diameter_deg"), Number)) SunVisualAngularDiameterDegrees = FMath::Clamp(Number, 0.05, 10.0);
        if ((*Scene)->TryGetNumberField(TEXT("sun_visual_emissive_strength"), Number)) SunVisualEmissiveStrength = FMath::Clamp(Number, 0.0, 1000.0);
        if ((*Scene)->TryGetNumberField(TEXT("sun_visual_effect_scale"), Number)) SunVisualEffectScale = FMath::Clamp(Number, 0.01, 100.0);
        if ((*Scene)->TryGetNumberField(TEXT("decorative_earth_radius_m"), Number)) DecorativeEarthRadiusMeters = FMath::Max(1.0, Number);
        if ((*Scene)->TryGetNumberField(TEXT("earth_cloud_scale"), Number)) EarthCloudScale = FMath::Clamp(Number, 1.0, 1.2);
        if ((*Scene)->TryGetNumberField(TEXT("earth_atmosphere_scale"), Number)) EarthAtmosphereScale = FMath::Clamp(Number, EarthCloudScale, 1.5);
        (*Scene)->TryGetBoolField(TEXT("use_official_celestial_assets"), bUseOfficialCelestialAssets);
        (*Scene)->TryGetBoolField(TEXT("use_textured_star_sphere"), bUseTexturedStarSphere);
        (*Scene)->TryGetBoolField(TEXT("use_earth_sky_atmosphere"), bUseEarthSkyAtmosphere);
        (*Scene)->TryGetBoolField(TEXT("use_manual_exposure"), bUseManualExposure);
        (*Scene)->TryGetBoolField(TEXT("sun_visual_enabled"), bEnableDecorativeSun);
        (*Scene)->TryGetBoolField(TEXT("sun_visual_effects_enabled"), bEnableDecorativeSunEffects);
        (*Scene)->TryGetBoolField(TEXT("decorative_earth_enabled"), bEnableDecorativeEarth);
        (*Scene)->TryGetStringField(TEXT("textured_star_mesh"), TexturedStarMeshPath);
        (*Scene)->TryGetStringField(TEXT("textured_star_material"), TexturedStarMaterialPath);
        (*Scene)->TryGetStringField(TEXT("sun_visual_mesh"), SunVisualMeshPath);
        (*Scene)->TryGetStringField(TEXT("sun_visual_material"), SunVisualMaterialPath);
        (*Scene)->TryGetStringField(TEXT("sun_burst_particle"), SunBurstParticlePath);
        (*Scene)->TryGetStringField(TEXT("sun_halo_particle"), SunHaloParticlePath);
        (*Scene)->TryGetStringField(TEXT("sun_lines_particle"), SunLinesParticlePath);
        (*Scene)->TryGetStringField(TEXT("earth_sphere_mesh"), EarthSphereMeshPath);
        (*Scene)->TryGetStringField(TEXT("earth_surface_material"), EarthSurfaceMaterialPath);
        (*Scene)->TryGetStringField(TEXT("earth_cloud_material"), EarthCloudMaterialPath);
        (*Scene)->TryGetStringField(TEXT("earth_atmosphere_material"), EarthAtmosphereMaterialPath);
        JsonVector3(*Scene, TEXT("camera_position_m"), CameraPositionMeters);
        JsonVector3(*Scene, TEXT("camera_look_at_m"), CameraLookAtMeters);
        JsonVector3(*Scene, TEXT("decorative_earth_position_m"), DecorativeEarthPositionMeters);
        FVector3d Rotation;
        if (JsonVector3(*Scene, TEXT("sun_rotation_deg"), Rotation)) SunRotation = FRotator(Rotation.X, Rotation.Y, Rotation.Z);
        if (JsonVector3(*Scene, TEXT("decorative_earth_rotation_deg"), Rotation)) DecorativeEarthRotation = FRotator(Rotation.X, Rotation.Y, Rotation.Z);
        (*Scene)->TryGetStringField(TEXT("time_mode"), TimeMode);
        double InterpolationDelayMs = InterpolationDelaySeconds * 1000.0;
        double MaxExtrapolationMs = MaxExtrapolationSeconds * 1000.0;
        if ((*Scene)->TryGetNumberField(TEXT("interpolation_delay_ms"), InterpolationDelayMs)) InterpolationDelaySeconds = FMath::Max(0.001, InterpolationDelayMs / 1000.0);
        if ((*Scene)->TryGetNumberField(TEXT("max_extrapolation_ms"), MaxExtrapolationMs)) MaxExtrapolationSeconds = FMath::Max(0.0, MaxExtrapolationMs / 1000.0);
    }

    const TSharedPtr<FJsonObject>* Visuals = nullptr;
    if (Root->TryGetObjectField(TEXT("visuals"), Visuals) && Visuals != nullptr)
    {
        const TSharedPtr<FJsonObject>* VisibilityByKind = nullptr;
        if ((*Visuals)->TryGetObjectField(TEXT("visibility_by_kind"), VisibilityByKind) && VisibilityByKind != nullptr)
        {
            for (const TPair<FString, TSharedPtr<FJsonValue>>& Entry : (*VisibilityByKind)->Values)
            {
                bool bVisible = true;
                if (Entry.Value.IsValid() && Entry.Value->TryGetBool(bVisible))
                {
                    VisualKindVisibility.Add(Entry.Key.TrimStartAndEnd().ToLower(), bVisible);
                }
            }
        }
    }

    const TSharedPtr<FJsonObject>* Objects = nullptr;
    if (Root->TryGetObjectField(TEXT("objects"), Objects) && Objects != nullptr)
    {
        for (const TPair<FString, TSharedPtr<FJsonValue>>& Entry : (*Objects)->Values)
        {
            const TSharedPtr<FJsonObject> Object = Entry.Value->AsObject();
            if (!Object.IsValid()) continue;
            FObjectSpec Spec;
            Object->TryGetStringField(TEXT("asset_type"), Spec.AssetType);
            Object->TryGetStringField(TEXT("asset_path"), Spec.AssetPath);
            Object->TryGetStringField(TEXT("actor_class"), Spec.ActorClass);
            Object->TryGetStringField(TEXT("placeholder_shape"), Spec.PlaceholderShape);
            JsonVector3(Object, TEXT("size_m"), Spec.SizeMeters);
            JsonVector3(Object, TEXT("scale"), Spec.Scale);
            JsonColor(Object, TEXT("color"), Spec.Color);
            ObjectSpecs.Add(Entry.Key, MoveTemp(Spec));
        }
    }

    FString AddressOverride;
    if (FParse::Value(FCommandLine::Get(), TEXT("BskListen="), AddressOverride)) ListenAddress = AddressOverride;
    int32 PortOverride = 0;
    if (FParse::Value(FCommandLine::Get(), TEXT("BskPort="), PortOverride)) ListenPort = FMath::Clamp(PortOverride, 1, 65535);
    FParse::Value(FCommandLine::Get(), TEXT("BskReplay="), ReplayPath);
    FParse::Value(FCommandLine::Get(), TEXT("BskReplayRate="), ReplayRate);
    FParse::Value(FCommandLine::Get(), TEXT("BskAutoCommand="), AutoCommand);
    if (!ReplayPath.IsEmpty() && FPaths::IsRelative(ReplayPath)) ReplayPath = FPaths::ConvertRelativePathToFull(ReplayPath);
    UE_LOG(LogBskUnreal, Display, TEXT("Loaded BSK scene config %s"), *ConfigPath);
    return true;
}

void ABskSceneController::ConfigureCaptureOutput()
{
    bAsyncCaptureEnabled = !FParse::Param(FCommandLine::Get(), TEXT("BskSynchronousCapture"));
    FParse::Value(FCommandLine::Get(), TEXT("BskCaptureDir="), CaptureOutputDirectory);
    FParse::Value(FCommandLine::Get(), TEXT("BskCaptureHost="), CaptureNetworkAddress);
    FParse::Value(FCommandLine::Get(), TEXT("BskCapturePort="), CaptureNetworkPort);
    FParse::Value(FCommandLine::Get(), TEXT("BskCaptureRate="), CaptureRateOverrideHertz);
    FParse::Value(FCommandLine::Get(), TEXT("BskPreviewRate="), PreviewRateOverrideHertz);
    FString ProductsOverride;
    if (FParse::Value(FCommandLine::Get(), TEXT("BskCaptureProducts="), ProductsOverride))
    {
        CaptureProductOverride.Reset();
        ProductsOverride.ReplaceInline(TEXT("+"), TEXT(","));
        ProductsOverride.ParseIntoArray(CaptureProductOverride, TEXT(","), true);
        for (FString& Product : CaptureProductOverride) Product = Product.TrimStartAndEnd().ToLower();
    }
    const auto IsKnownProduct = [](const FString& Product)
    {
        return Product == TEXT("rgb");
    };
    for (const FString& Product : CaptureProductOverride)
    {
        if (!IsKnownProduct(Product))
        {
            UE_LOG(LogBskUnreal, Error, TEXT("Unsupported -BskCaptureProducts entry '%s'; capture is disabled until corrected"), *Product);
            CaptureOutputDirectory.Reset();
            CaptureNetworkPort = 0;
            CaptureProductOverride.Reset();
            return;
        }
    }
    if (!CaptureOutputDirectory.IsEmpty())
    {
        if (FPaths::IsRelative(CaptureOutputDirectory)) CaptureOutputDirectory = FPaths::ConvertRelativePathToFull(CaptureOutputDirectory);
        FPaths::CollapseRelativeDirectories(CaptureOutputDirectory);
        CaptureDiskWriter = MakeShared<FBskCaptureDiskWriter>();
    }
    CaptureRateOverrideHertz = FMath::Clamp(CaptureRateOverrideHertz, 0.0, 60.0);
    PreviewRateOverrideHertz = FMath::Clamp(PreviewRateOverrideHertz, 0.0, 60.0);
    CaptureNetworkPort = FMath::Clamp(CaptureNetworkPort, 0, 65535);
    if (CaptureNetworkPort > 0)
    {
        CaptureNetworkSender = MakeShared<FBskCaptureNetworkSender>(CaptureNetworkAddress, static_cast<uint16>(CaptureNetworkPort));
    }
    UE_LOG(LogBskUnreal, Display, TEXT("BSK camera products disk=%s network=%s:%d capture_hz=%.1f preview_hz=%.1f override_products=%s"),
        CaptureOutputDirectory.IsEmpty() ? TEXT("disabled") : *CaptureOutputDirectory,
        *CaptureNetworkAddress, CaptureNetworkPort, CaptureRateOverrideHertz, PreviewRateOverrideHertz,
        CaptureProductOverride.IsEmpty() ? TEXT("manifest") : *FString::Join(CaptureProductOverride, TEXT(",")));
}

void ABskSceneController::ConfigurePixelStreamingOutput()
{
    if (FParse::Value(FCommandLine::Get(), TEXT("PixelStreamingWebRTCFps="), MaximumStreamingFrameRate))
        MaximumStreamingFrameRate = FMath::Clamp(MaximumStreamingFrameRate, 1, 120);
    FParse::Value(FCommandLine::Get(), TEXT("BskPixelStreamingURL="), PixelStreamingConnectionUrl);
    FParse::Value(FCommandLine::Get(), TEXT("BskPixelStreamingBaseId="), PixelStreamingBaseId);
    bVideoDiagnostics = FParse::Param(FCommandLine::Get(), TEXT("BskVideoDiagnostics"));
    FParse::Value(FCommandLine::Get(), TEXT("BskPixelStreamingCameraWidth="), PixelStreamingCameraWidth);
    FParse::Value(FCommandLine::Get(), TEXT("BskPixelStreamingCameraHeight="), PixelStreamingCameraHeight);
    FParse::Value(FCommandLine::Get(), TEXT("BskPixelStreamingCameraFps="), PixelStreamingCameraRateHertz);
    PixelStreamingCameraWidth = FMath::Clamp(PixelStreamingCameraWidth, 160, 1920);
    PixelStreamingCameraHeight = FMath::Clamp(PixelStreamingCameraHeight, 90, 1080);
    PixelStreamingCameraRateHertz = FMath::Clamp(PixelStreamingCameraRateHertz, 1.0, 120.0);
    RequestedPixelStreamingCameraRateHertz = PixelStreamingCameraRateHertz;

    FString Cameras;
    if (FParse::Value(FCommandLine::Get(), TEXT("BskPixelStreamingCameras="), Cameras))
    {
        Cameras.ReplaceInline(TEXT("+"), TEXT(","));
        TArray<FString> Entries;
        Cameras.ParseIntoArray(Entries, TEXT(","), true);
        for (FString CameraId : Entries)
        {
            CameraId.TrimStartAndEndInline();
            if (CameraId.Equals(TEXT("all"), ESearchCase::IgnoreCase)) bPixelStreamingAllManifestCameras = true;
            else if (!CameraId.IsEmpty()) PixelStreamingRequestedCameras.Add(CameraId);
        }
    }
    if (!PixelStreamingConnectionUrl.IsEmpty())
    {
        UE_LOG(LogBskUnreal, Display,
            TEXT("RenderTarget Pixel Streaming enabled url=%s base=%s cameras=%s resolution=%dx%d fps=%.1f"),
            *PixelStreamingConnectionUrl, *PixelStreamingBaseId,
            bPixelStreamingAllManifestCameras ? TEXT("all") : *FString::Join(PixelStreamingRequestedCameras.Array(), TEXT(",")),
            PixelStreamingCameraWidth, PixelStreamingCameraHeight, PixelStreamingCameraRateHertz);
    }
}

bool ABskSceneController::IsPixelStreamingCameraRequested(const FString& CameraId) const
{
    return !PixelStreamingConnectionUrl.IsEmpty()
        && (bPixelStreamingAllManifestCameras || PixelStreamingRequestedCameras.Contains(CameraId));
}

FString ABskSceneController::PixelStreamingIdForCamera(const FString& CameraId) const
{
    FString Safe = CameraId;
    for (int32 Index = 0; Index < Safe.Len(); ++Index)
    {
        const TCHAR Character = Safe[Index];
        if (!FChar::IsAlnum(Character) && Character != TEXT('_') && Character != TEXT('-')) Safe[Index] = TEXT('_');
    }
    return FString::Printf(TEXT("%s__%s"), *PixelStreamingBaseId, *Safe);
}

void ABskSceneController::ConfigureFixedRgbExposure(USceneCaptureComponent2D* Capture) const
{
    if (!Capture) return;
    Capture->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
    Capture->ShowFlags.SetPostProcessing(true);
    Capture->ShowFlags.SetEyeAdaptation(true);
    Capture->ShowFlags.SetTonemapper(true);
    Capture->PostProcessBlendWeight = 1.0f;
    Capture->PostProcessSettings.bOverride_AutoExposureMethod = true;
    Capture->PostProcessSettings.AutoExposureMethod = EAutoExposureMethod::AEM_Manual;
    Capture->PostProcessSettings.bOverride_AutoExposureApplyPhysicalCameraExposure = true;
    Capture->PostProcessSettings.AutoExposureApplyPhysicalCameraExposure = true;
    Capture->PostProcessSettings.bOverride_CameraISO = true;
    Capture->PostProcessSettings.CameraISO = 100.0f;
    Capture->PostProcessSettings.bOverride_CameraShutterSpeed = true;
    Capture->PostProcessSettings.CameraShutterSpeed = 125.0f;
    Capture->PostProcessSettings.bOverride_DepthOfFieldFstop = true;
    Capture->PostProcessSettings.DepthOfFieldFstop = 16.0f;
    Capture->PostProcessSettings.bOverride_AutoExposureBias = true;
    Capture->PostProcessSettings.AutoExposureBias = static_cast<float>(MaterialExposureBias);
}

void ABskSceneController::ConfigureRgbRenderTarget(UTextureRenderTarget2D* Target) const
{
    if (!Target) return;
    Target->TargetGamma = 2.2f;
    Target->ClearColor = FLinearColor::Black;
}

void ABskSceneController::ConfigurePixelStreamingCamera(AActor* Actor, const FBskCameraDefinition& Definition)
{
    if (!Actor || !IsPixelStreamingCameraRequested(Definition.CameraId)) return;

    // The HUD picture-in-picture and the standalone Pixel Streaming camera must
    // show the exact same rendered image. Reuse the camera's canonical capture
    // and render target instead of maintaining a second view state/exposure path.
    USceneCaptureComponent2D* Capture = CameraCaptureComponents.FindRef(Definition.CameraId);
    UTextureRenderTarget2D* Target = CameraRenderTargets.FindRef(Definition.CameraId);
    if (!Capture || !Target)
    {
        UE_LOG(LogBskUnreal, Error, TEXT("Shared camera capture is unavailable for Pixel Streaming camera %s"),
            *Definition.CameraId);
        return;
    }
    ConfigureFixedRgbExposure(Capture);
    ConfigureRgbRenderTarget(Target);
    Capture->bAlwaysPersistRenderingState = true;
    Capture->TextureTarget = Target;
    PixelStreamingCameraCaptures.Add(Definition.CameraId, Capture);
    PixelStreamingCameraTargets.Add(Definition.CameraId, Target);
    PixelStreamingCameraNextCaptureSeconds.Add(Definition.CameraId, 0.0);

    if (Target->SizeX != PixelStreamingCameraWidth || Target->SizeY != PixelStreamingCameraHeight)
    {
        UE_LOG(LogBskUnreal, Warning,
            TEXT("Camera %s shares its manifest resolution %dx%d with HUD and Pixel Streaming; requested streamer resolution was %dx%d"),
            *Definition.CameraId, Target->SizeX, Target->SizeY,
            PixelStreamingCameraWidth, PixelStreamingCameraHeight);
    }

    if (!PixelStreamingCameraStreamers.Contains(Definition.CameraId))
    {
        const FString StreamerId = PixelStreamingIdForCamera(Definition.CameraId);
        TSharedPtr<IPixelStreaming2Streamer> Streamer = IPixelStreaming2Module::Get().CreateStreamer(StreamerId);
        if (!Streamer)
        {
            UE_LOG(LogBskUnreal, Error, TEXT("Could not create Pixel Streaming camera streamer %s"), *StreamerId);
            return;
        }
        TSharedPtr<IPixelStreaming2VideoProducer> Producer =
            UE::PixelStreaming2::FVideoProducerRenderTarget::Create(Target);
        if (!Producer)
        {
            IPixelStreaming2Module::Get().DeleteStreamer(Streamer);
            UE_LOG(LogBskUnreal, Error, TEXT("Could not create RenderTarget producer for %s"), *StreamerId);
            return;
        }
        Streamer->SetStreamFPS(FMath::RoundToInt(PixelStreamingCameraRateHertz));
        Streamer->SetCoupleFramerate(true);
        Streamer->SetConnectionURL(PixelStreamingConnectionUrl);
        Streamer->SetVideoProducer(Producer);
        Streamer->StartStreaming();
        PixelStreamingCameraStreamers.Add(Definition.CameraId, Streamer);
        PixelStreamingCameraProducers.Add(Definition.CameraId, Producer);
        UE_LOG(LogBskUnreal, Display, TEXT("Started shared-target camera streamer %s for manifest camera %s"),
            *StreamerId, *Definition.CameraId);
    }

    if (!PixelStreamingNewConnectionHandle.IsValid())
    {
        if (UPixelStreaming2Delegates* Delegates = UPixelStreaming2Delegates::Get())
        {
            PixelStreamingNewConnectionHandle = Delegates->OnNewConnectionNative.AddLambda(
                [this](const FString& StreamerId, const FString& PlayerId)
                {
                    for (const TPair<FString, TSharedPtr<IPixelStreaming2Streamer>>& Pair : PixelStreamingCameraStreamers)
                    {
                        if (Pair.Value && PixelStreamingIdForCamera(Pair.Key) == StreamerId)
                        {
                            Pair.Value->ForceKeyFrame();
                            UE_LOG(LogBskUnreal, Display, TEXT("Pixel Streaming camera peer %s connected to %s"),
                                *PlayerId, *StreamerId);
                            break;
                        }
                    }
                });
        }
    }
}

bool ABskSceneController::HasPixelStreamingCameraViewers(const FString& CameraId) const
{
    const TSharedPtr<IPixelStreaming2Streamer> Streamer = PixelStreamingCameraStreamers.FindRef(CameraId);
    if (!Streamer) return false;
    for (const FString& PlayerId : Streamer->GetConnectedPlayers())
        if (PlayerId != Streamer->GetId()) return true;
    return false;
}

bool ABskSceneController::HasPixelStreamingViewportViewers() const
{
    // A visible local viewport is itself a viewer, even without browser peers.
    if (!FParse::Param(FCommandLine::Get(), TEXT("RenderOffscreen"))) return true;
    const TSharedPtr<IPixelStreaming2Streamer> Streamer = IPixelStreaming2Module::Get().FindStreamer(PixelStreamingBaseId);
    if (!Streamer) return false;
    for (const FString& PlayerId : Streamer->GetConnectedPlayers())
        if (PlayerId != Streamer->GetId()) return true;
    return false;
}

void ABskSceneController::UpdatePreviewViewportRendering()
{
    // In server/offscreen mode a camera subscriber does not need a second full
    // 3D view rendered into an unwatched main viewport. SceneCapture continues
    // independently. Never suppress a local editor/windowed user's viewport.
    if (PixelStreamingConnectionUrl.IsEmpty() || !FParse::Param(FCommandLine::Get(), TEXT("RenderOffscreen"))
        || !GEngine || !GEngine->GameViewport) return;
    if (!bManagesViewportRendering)
    {
        bPreviousDisableWorldRendering = GEngine->GameViewport->bDisableWorldRendering;
        bManagesViewportRendering = true;
    }
    GEngine->GameViewport->bDisableWorldRendering = bPreviousDisableWorldRendering || !HasPixelStreamingViewportViewers();
}

void ABskSceneController::UpdatePixelStreamingCameraCaptures()
{
    if (!GetWorld() || PixelStreamingCameraCaptures.IsEmpty()) return;
    const double Now = FPlatformTime::Seconds();
    for (const TPair<FString, TObjectPtr<USceneCaptureComponent2D>>& Pair : PixelStreamingCameraCaptures)
    {
        if (!Pair.Value || !Pair.Value->TextureTarget) continue;
        const FBskCameraDefinition* Definition = ManifestCameras.Find(Pair.Key);
        const bool bVisibleInHud = Definition && Definition->bPictureInPicture
            && CameraPictureInPictureVisibility.FindRef(Pair.Key);
        // A visible HUD PIP was already captured earlier in this game tick and
        // feeds this same RenderTarget. Capture here only for hidden/non-PIP streams.
        if (bVisibleInHud || !HasPixelStreamingCameraViewers(Pair.Key)) continue;
        double& NextSeconds = PixelStreamingCameraNextCaptureSeconds.FindOrAdd(Pair.Key);
        if (!BskPreviewCaptureDue(Now, PixelStreamingCameraRateHertz, NextSeconds)) continue;
        Pair.Value->CaptureScene();
        if (bVideoDiagnostics) ++VideoDiagnosticsCaptures.FindOrAdd(Pair.Key);
    }
}

void ABskSceneController::ShutdownPixelStreamingCameras()
{
    if (PixelStreamingNewConnectionHandle.IsValid())
    {
        if (UPixelStreaming2Delegates* Delegates = UPixelStreaming2Delegates::Get())
        {
            Delegates->OnNewConnectionNative.Remove(PixelStreamingNewConnectionHandle);
        }
        PixelStreamingNewConnectionHandle.Reset();
    }
    for (TPair<FString, TSharedPtr<IPixelStreaming2Streamer>>& Pair : PixelStreamingCameraStreamers)
    {
        if (!Pair.Value) continue;
        Pair.Value->StopStreaming();
        Pair.Value->SetVideoProducer(nullptr);
        IPixelStreaming2Module::Get().DeleteStreamer(Pair.Value);
    }
    PixelStreamingCameraStreamers.Reset();
    PixelStreamingCameraProducers.Reset();
    PixelStreamingCameraNextCaptureSeconds.Reset();
    PixelStreamingCameraCaptures.Reset();
    PixelStreamingCameraTargets.Reset();
}

ABskSceneController::FObjectSpec ABskSceneController::ResolveSpec(const FBskRenderObjectState& State) const
{
    if (const FBskObjectDefinition* Definition = ManifestObjects.Find(State.ObjectId))
    {
        if (const FObjectSpec* ById = ObjectSpecs.Find(Definition->ObjectId)) return *ById;
        if (const FObjectSpec* ByName = ObjectSpecs.Find(Definition->DisplayName)) return *ByName;
        if (!Definition->AssetPath.IsEmpty())
        {
            FObjectSpec Result;
            Result.AssetType = TEXT("auto");
            Result.AssetPath = Definition->AssetPath;
            return Result;
        }
    }
    if (const FObjectSpec* Configured = ObjectSpecs.Find(State.Name))
    {
        return *Configured;
    }
    FObjectSpec Result;
    if (!State.AssetPath.IsEmpty())
    {
        Result.AssetType = State.AssetPath.EndsWith(TEXT(".usd"), ESearchCase::IgnoreCase) ||
            State.AssetPath.EndsWith(TEXT(".usda"), ESearchCase::IgnoreCase) ||
            State.AssetPath.EndsWith(TEXT(".usdc"), ESearchCase::IgnoreCase) ? TEXT("usd") : TEXT("auto");
        Result.AssetPath = State.AssetPath;
    }
    const uint32 Hash = GetTypeHash(State.Name);
    Result.Color = FLinearColor::MakeFromHSV8(static_cast<uint8>(Hash & 0xff), 190, 240);
    return Result;
}

AActor* ABskSceneController::FindBoundActor(const FString& ObjectName) const
{
    TArray<AActor*> Tagged;
    UGameplayStatics::GetAllActorsWithTag(GetWorld(), FName(*BindingTag(ObjectName)), Tagged);
    if (!Tagged.IsEmpty())
    {
        return Tagged[0];
    }
    TArray<AActor*> AllActors;
    UGameplayStatics::GetAllActorsOfClass(GetWorld(), AActor::StaticClass(), AllActors);
    for (AActor* Actor : AllActors)
    {
        if (Actor && Actor->GetName().Equals(ObjectName, ESearchCase::IgnoreCase))
        {
            return Actor;
        }
    }
    return nullptr;
}

AActor* ABskSceneController::GetOrCreateActor(const FBskRenderObjectState& State)
{
    const FString& Key = State.ObjectId.IsEmpty() ? State.Name : State.ObjectId;
    if (TObjectPtr<AActor>* Existing = BoundActors.Find(Key))
    {
        return Existing->Get();
    }
    AActor* Actor = FindBoundActor(Key);
    if (!Actor)
    {
        if (const FBskObjectDefinition* Definition = ManifestObjects.Find(Key)) Actor = SpawnManifestObject(*Definition);
        if (!Actor) Actor = SpawnFromSpec(Key, ResolveSpec(State));
    }
    if (Actor)
    {
        Actor->Tags.AddUnique(FName(*BindingTag(Key)));
        if (!State.SemanticLabel.IsEmpty()) Actor->Tags.AddUnique(FName(*FString::Printf(TEXT("BSK.Semantic.%s"), *State.SemanticLabel)));
        BoundActors.Add(Key, Actor);
    }
    return Actor;
}

AActor* ABskSceneController::SpawnFromSpec(const FString& ObjectName, const FObjectSpec& Spec)
{
    FActorSpawnParameters Params;
    Params.Name = MakeUniqueObjectName(GetWorld(), AActor::StaticClass(), SafeActorName(ObjectName));
    Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

    if (!Spec.ActorClass.IsEmpty())
    {
        if (UClass* Class = FSoftClassPath(Spec.ActorClass).TryLoadClass<AActor>())
        {
            AActor* Actor = GetWorld()->SpawnActor<AActor>(Class, FTransform::Identity, Params);
            if (Actor) Actor->SetActorScale3D(FVector(Spec.Scale));
            return Actor;
        }
    }
    if (Spec.AssetType.Equals(TEXT("usd"), ESearchCase::IgnoreCase))
    {
        if (!bAllowExternalAssets && !Spec.AssetPath.StartsWith(TEXT("/Game/")))
        {
            UE_LOG(LogBskUnreal, Warning, TEXT("External asset loading is disabled for %s: %s"), *ObjectName, *Spec.AssetPath);
            return SpawnPlaceholder(ObjectName, Spec);
        }
        if (AActor* Actor = SpawnUsdStage(ObjectName, Spec)) return Actor;
    }
    if ((Spec.AssetType.Equals(TEXT("static_mesh"), ESearchCase::IgnoreCase) || Spec.AssetType.Equals(TEXT("auto"), ESearchCase::IgnoreCase)) && !Spec.AssetPath.IsEmpty())
    {
        if (UStaticMesh* Mesh = Cast<UStaticMesh>(FSoftObjectPath(Spec.AssetPath).TryLoad()))
        {
            AStaticMeshActor* Actor = GetWorld()->SpawnActor<AStaticMeshActor>(AStaticMeshActor::StaticClass(), FTransform::Identity, Params);
            Actor->GetStaticMeshComponent()->SetStaticMesh(Mesh);
            Actor->GetStaticMeshComponent()->SetMobility(EComponentMobility::Movable);
            Actor->SetActorScale3D(FVector(Spec.Scale));
            return Actor;
        }
    }
    if (Spec.AssetType.Equals(TEXT("skeletal_mesh"), ESearchCase::IgnoreCase) && !Spec.AssetPath.IsEmpty())
    {
        if (USkeletalMesh* Mesh = Cast<USkeletalMesh>(FSoftObjectPath(Spec.AssetPath).TryLoad()))
        {
            ASkeletalMeshActor* Actor = GetWorld()->SpawnActor<ASkeletalMeshActor>(ASkeletalMeshActor::StaticClass(), FTransform::Identity, Params);
            Actor->GetSkeletalMeshComponent()->SetSkeletalMeshAsset(Mesh);
            Actor->SetActorScale3D(FVector(Spec.Scale));
            return Actor;
        }
    }
    if (!Spec.AssetPath.IsEmpty())
    {
        UE_LOG(LogBskUnreal, Warning, TEXT("Could not load '%s' for %s; using placeholder"), *Spec.AssetPath, *ObjectName);
    }
    return SpawnPlaceholder(ObjectName, Spec);
}

AActor* ABskSceneController::SpawnUsdStage(const FString& ObjectName, const FObjectSpec& Spec)
{
    UClass* StageClass = FSoftClassPath(TEXT("/Script/USDStage.UsdStageActor")).TryLoadClass<AActor>();
    if (!StageClass)
    {
        UE_LOG(LogBskUnreal, Warning, TEXT("USDStage runtime module is unavailable for %s. Enable UE's USD Importer plugin or map an imported asset."), *ObjectName);
        return nullptr;
    }
    const FTransform Transform(FQuat::Identity, FVector::ZeroVector, FVector(Spec.Scale));
    AActor* Actor = GetWorld()->SpawnActorDeferred<AActor>(StageClass, Transform, nullptr, nullptr, ESpawnActorCollisionHandlingMethod::AlwaysSpawn);
    if (!Actor) return nullptr;
    if (FStructProperty* RootLayer = FindFProperty<FStructProperty>(StageClass, TEXT("RootLayer")))
    {
        if (FFilePath* FilePath = RootLayer->ContainerPtrToValuePtr<FFilePath>(Actor)) FilePath->FilePath = Spec.AssetPath;
    }
    UGameplayStatics::FinishSpawningActor(Actor, Transform);
    return Actor;
}

AActor* ABskSceneController::SpawnPlaceholder(const FString& ObjectName, const FObjectSpec& Spec)
{
    FActorSpawnParameters Params;
    Params.Name = MakeUniqueObjectName(GetWorld(), AStaticMeshActor::StaticClass(), SafeActorName(ObjectName));
    Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    AStaticMeshActor* Actor = GetWorld()->SpawnActor<AStaticMeshActor>(AStaticMeshActor::StaticClass(), FTransform::Identity, Params);
    if (!Actor) return nullptr;
    const TCHAR* MeshPath = PrimitiveMeshPath(Spec.PlaceholderShape);
    UStaticMeshComponent* Component = Actor->GetStaticMeshComponent();
    Component->SetMobility(EComponentMobility::Movable);
    UStaticMesh* PlaceholderMesh = LoadObject<UStaticMesh>(nullptr, MeshPath);
    if (!PlaceholderMesh || !Component->SetStaticMesh(PlaceholderMesh))
    {
        UE_LOG(LogBskUnreal, Error, TEXT("Failed to assign placeholder mesh %s to %s"), MeshPath, *ObjectName);
    }
    Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    ApplyUnlitColor(Component, Actor, Spec.Color);
    // Engine basic shapes are one meter across, so a size in meters is also
    // their dimensionless actor scale.
    Actor->SetActorScale3D(FVector(Spec.SizeMeters));
    UE_LOG(LogBskUnreal, Display, TEXT("Created visible placeholder %s shape=%s size_m=%s color=%s"),
        *ObjectName, *Spec.PlaceholderShape, *Spec.SizeMeters.ToString(), *Spec.Color.ToString());
    return Actor;
}

void ABskSceneController::ApplyManifest(const FBskSceneManifest& Manifest)
{
    if (ActiveSessionId != Manifest.SessionId) CancelCaptureJobs();
    check(IsInGameThread());
    if (!ActiveSessionId.IsEmpty() && ActiveSessionId != Manifest.SessionId)
    {
        PendingCommandIds.Reset();
        LastCommandStatus = TEXT("session changed");
        MissionEventHistory.Reset();
        bAutoCommandSent = false;
    }
    if (ActiveSessionId != Manifest.SessionId) ResetPresentationState();
    ActiveSessionId = Manifest.SessionId;
    ActiveManifestRevision = Manifest.Revision;
    bShowOrbitLines = Manifest.bOrbitLines;
    ConfigureManifestLighting(Manifest);
    InterpolationDelaySeconds = FMath::Max(0.001, Manifest.InterpolationDelayMilliseconds / 1000.0);
    MaxExtrapolationSeconds = FMath::Max(0.0, Manifest.MaxExtrapolationMilliseconds / 1000.0);
    ManifestObjects.Reset();
    ManifestCelestialBodies.Reset();
    PrimaryDirectionalLightBodyId.Reset();
    bEphemerisDirectionalLightActive = false;
    bEphemerisEarthLogged = false;
    CurrentSunPositionMeters = FVector3d::ZeroVector;
    CurrentSunRadiusMeters = 0.0;
    CurrentSolarVisibility = 1.0;
    SolarOccluders.Reset();
    CurrentSunAngularDiameterDegrees = SunVisualAngularDiameterDegrees;
    UpdateDecorativeSunScale();
    ManifestVisuals.Reset();
    ManifestCameras.Reset();
    VisualBaseRotations.Reset();
    VisualBaseScales.Reset();
    VisualDynamicVisibility.Reset();
    if (UBskRenderWorldSubsystem* RenderSubsystem = GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>())
    {
        RenderSubsystem->AcceptManifest(Manifest);
    }
    for (const FBskObjectDefinition& Definition : Manifest.Objects)
    {
        ManifestObjects.Add(Definition.ObjectId, Definition);
        if (!BoundActors.Contains(Definition.ObjectId))
        {
            if (AActor* Actor = SpawnManifestObject(Definition))
            {
                Actor->Tags.AddUnique(FName(*BindingTag(Definition.ObjectId)));
                if (!Definition.SemanticLabel.IsEmpty()) Actor->Tags.AddUnique(FName(*FString::Printf(TEXT("BSK.Semantic.%s"), *Definition.SemanticLabel)));
                BoundActors.Add(Definition.ObjectId, Actor);
            }
        }
    }
    const bool bManifestHasEarth = Manifest.CelestialBodies.ContainsByPredicate([](const FBskCelestialBodyDefinition& Definition)
    {
        return Definition.BodyId.Equals(TEXT("earth"), ESearchCase::IgnoreCase) ||
            Definition.DisplayName.Equals(TEXT("earth"), ESearchCase::IgnoreCase);
    });
    if (DecorativeEarthActor)
    {
        DecorativeEarthActor->SetActorHiddenInGame(bManifestHasEarth);
        UE_LOG(LogBskUnreal, Display, TEXT("Decorative Earth %s because the manifest %s an ephemeris Earth"),
            bManifestHasEarth ? TEXT("hidden") : TEXT("visible"),
            bManifestHasEarth ? TEXT("contains") : TEXT("does not contain"));
    }
    for (const TPair<FString, TObjectPtr<AActor>>& Pair : CelestialActors)
    {
        if (Pair.Key.Equals(TEXT("earth"), ESearchCase::IgnoreCase) && Pair.Value)
        {
            Pair.Value->SetActorHiddenInGame(!bManifestHasEarth);
        }
    }
    for (const FBskCelestialBodyDefinition& Definition : Manifest.CelestialBodies)
    {
        ManifestCelestialBodies.Add(Definition.BodyId, Definition);
        if (Definition.bDrivesDirectionalLight)
        {
            if (PrimaryDirectionalLightBodyId.IsEmpty())
            {
                PrimaryDirectionalLightBodyId = Definition.BodyId;
                UE_LOG(LogBskUnreal, Display,
                    TEXT("Celestial body '%s' will drive the primary directional light from ephemeris frames"),
                    *Definition.BodyId);
            }
            else
            {
                UE_LOG(LogBskUnreal, Warning,
                    TEXT("Ignoring additional primary directional-light body '%s'; '%s' is already selected"),
                    *Definition.BodyId, *PrimaryDirectionalLightBodyId);
            }
        }
        const bool bUseDecorativeSunProxy = Definition.bDrivesDirectionalLight;
        if (bUseDecorativeSunProxy)
        {
            if (AActor* ExistingActor = CelestialActors.FindRef(Definition.BodyId))
            {
                ExistingActor->SetActorHiddenInGame(true);
            }
            UE_LOG(LogBskUnreal, Display,
                TEXT("Celestial body '%s' drives lighting and will not spawn at ephemeris distance; decorative proxy=%s"),
                *Definition.BodyId, DecorativeSunActor ? TEXT("enabled") : TEXT("disabled"));
        }
        else if (!CelestialActors.Contains(Definition.BodyId))
        {
            if (AActor* Actor = SpawnCelestialBody(Definition))
            {
                // Celestial surfaces still receive sunlight when the spacecraft
                // is eclipsed. Keep them off the local-object lighting channel.
                TArray<UPrimitiveComponent*> Components;
                Actor->GetComponents<UPrimitiveComponent>(Components);
                for (UPrimitiveComponent* Component : Components)
                {
                    Component->SetLightingChannels(false, true, false);
                    // Planetary umbra uses ephemeris geometry below; a 6,378 km
                    // mesh must not enter the metre-scale arm shadow map.
                    Component->SetCastShadow(false);
                }
                CelestialActors.Add(Definition.BodyId, Actor);
            }
        }
    }
    if (PrimaryDirectionalLightBodyId.IsEmpty())
    {
        CurrentSunSourceDirection = -SunRotation.Vector().GetSafeNormal();
        if (SunLight) SunLight->SetActorRotation(SunRotation);
        UE_LOG(LogBskUnreal, Warning,
            TEXT("Manifest has no ephemeris-driven directional light; using configured fallback rotation %s"),
            *SunRotation.ToCompactString());
    }
    for (const FBskVisualDefinition& Definition : Manifest.Visuals)
    {
        ManifestVisuals.Add(Definition.VisualId, Definition);
        if (!VisualActors.Contains(Definition.VisualId))
        {
            if (AActor* Actor = SpawnVisual(Definition)) VisualActors.Add(Definition.VisualId, Actor);
        }
        if (AActor* Actor = VisualActors.FindRef(Definition.VisualId))
        {
            ApplyVisualMountTransform(Actor, Definition);
            const FQuat BaseRotation = Actor->GetRootComponent()
                ? Actor->GetRootComponent()->GetRelativeRotation().Quaternion()
                : FQuat::Identity;
            VisualBaseRotations.Add(Definition.VisualId, BaseRotation);
            VisualBaseScales.Add(Definition.VisualId, Actor->GetActorRelativeScale3D());
            VisualDynamicVisibility.Add(Definition.VisualId, true);
            RefreshVisualVisibility(Definition.VisualId);
        }
    }
    for (const FBskCameraDefinition& Definition : Manifest.Cameras)
    {
        ManifestCameras.Add(Definition.CameraId, Definition);
        AActor* CameraActor = CameraActors.FindRef(Definition.CameraId);
        if (!CameraActor)
        {
            CameraActor = SpawnCamera(Definition);
            if (CameraActor)
            {
                CameraActors.Add(Definition.CameraId, CameraActor);
                if (UBskRenderWorldSubsystem* RenderSubsystem = GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>())
                {
                    RenderSubsystem->RegisterCameraForCapture(Definition.CameraId, CameraActor);
                }
            }
        }
        if (CameraActor) ConfigureCamera(CameraActor, Definition);
    }
    AttachManifestChildren();
    if (APlayerController* Player = GetWorld() ? GetWorld()->GetFirstPlayerController() : nullptr)
    {
        if (ABskCameraPawn* Pawn = Cast<ABskCameraPawn>(Player->GetPawn()))
        {
            Pawn->SetOrbitDistanceMeters(Manifest.DefaultCameraDistanceMeters);
        }
    }
    const FString CameraTarget = Manifest.DefaultCameraTarget.IsEmpty()
        ? Manifest.OriginObjectId
        : Manifest.DefaultCameraTarget;
    if (!CameraTarget.IsEmpty()) FocusObject(CameraTarget, false);
    if (UBskRenderWorldSubsystem* RenderSubsystem = GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>())
    {
        RenderSubsystem->NotifyManifestApplied(Manifest);
    }
    if (!AutoCommand.IsEmpty() && !bAutoCommandSent)
    {
        const FBskUiCommandDefinition* Definition = Manifest.UiCommands.FindByPredicate(
            [this](const FBskUiCommandDefinition& Item) { return Item.Command == AutoCommand; });
        FString Error;
        if (!Definition || !SendUiCommand(AutoCommand, Definition->TargetId, Definition->PayloadJson, Error))
        {
            UE_LOG(LogBskUnreal, Error, TEXT("Automatic BSK command '%s' failed: %s"), *AutoCommand, *Error);
        }
        else
        {
            bAutoCommandSent = true;
        }
    }
    int32 GeometryCount = 0;
    for (const FBskObjectDefinition& Definition : Manifest.Objects) GeometryCount += Definition.Geometries.Num();
    UE_LOG(LogBskUnreal, Display, TEXT("Applied BSK scene manifest session=%s revision=%lld objects=%d geoms/visuals=%d/%d celestial=%d cameras=%d"),
        *Manifest.SessionId, Manifest.Revision, Manifest.Objects.Num(), GeometryCount,
        Manifest.Visuals.Num(), Manifest.CelestialBodies.Num(), Manifest.Cameras.Num());
}

void ABskSceneController::ResetPresentationState()
{
    check(IsInGameThread());
    LastFrameId = -1;
    bHasTargetFrame = false;
    bHasPresentationFrame = false;
    PreviousFrame = FBskRenderFrame{};
    TargetFrame = FBskRenderFrame{};
    PresentationFrame = FBskRenderFrame{};
    LastFrameArrivalSeconds = 0.0;
    BlendElapsedSeconds = 0.0;
    BlendDurationSeconds = 1.0 / 30.0;
    // Simulation time goes back to zero: old deadlines would stall capture
    // until the new session caught up with the previous one.
    CameraNextDataCaptureSimulationNanoseconds.Reset();
    CameraNextCaptureSeconds.Reset();
    PixelStreamingCameraNextCaptureSeconds.Reset();
    auto CutCaptures = [](auto& Captures)
    {
        for (auto& Pair : Captures)
        {
            if (Pair.Value) Pair.Value->bCameraCutThisFrame = true;
        }
    };
    CutCaptures(CameraCaptureComponents);
    CutCaptures(PixelStreamingCameraCaptures);
}

void ABskSceneController::ApplyEvent(const FBskRenderEvent& Event)
{
    check(IsInGameThread());
    if (!ActiveSessionId.IsEmpty() && Event.SessionId != ActiveSessionId) return;
    FBskMissionEventView View;
    View.Sequence = Event.Sequence;
    View.Kind = Event.EventKind;
    View.Message = Event.EventKind;
    if (!Event.PayloadJson.IsEmpty())
    {
        TSharedPtr<FJsonObject> Payload;
        if (FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Event.PayloadJson), Payload) && Payload.IsValid())
        {
            Payload->TryGetStringField(TEXT("severity"), View.Severity);
            Payload->TryGetStringField(TEXT("message"), View.Message);
            int64 EventSimTime = 0;
            FString EventSimTimeString;
            if (Payload->TryGetStringField(TEXT("sim_time_ns"), EventSimTimeString)) LexTryParseString(EventSimTime, *EventSimTimeString);
            View.SimulationTimeNanoseconds = EventSimTime;
            if (Event.EventKind == TEXT("command_result"))
            {
                FString CommandId;
                FString Command;
                FString Status;
                Payload->TryGetStringField(TEXT("command_id"), CommandId);
                Payload->TryGetStringField(TEXT("command"), Command);
                Payload->TryGetStringField(TEXT("status"), Status);
                PendingCommandIds.Remove(CommandId);
                LastCommandStatus = FString::Printf(TEXT("%s %s"), *Status.ToUpper(), *CommandId);

                // The protocol keeps structured command output in ``result``.  The
                // original HUD displayed only ``message`` (for example
                // "mission.status completed"), which described completion of the
                // query and hid the actual mission phase.  Promote the commonly used
                // mission status fields to a short, human-readable timeline entry.
                const TSharedPtr<FJsonObject>* Result = nullptr;
                if (Status.Equals(TEXT("accepted"), ESearchCase::IgnoreCase)
                    && Payload->TryGetObjectField(TEXT("result"), Result)
                    && Result != nullptr && Result->IsValid())
                {
                    FString Phase;
                    bool bPaused = false;
                    const bool bHasPhase = (*Result)->TryGetStringField(TEXT("phase"), Phase);
                    const bool bHasPaused = (*Result)->TryGetBoolField(TEXT("paused"), bPaused);
                    if (bHasPhase || bHasPaused)
                    {
                        TArray<FString> Details;
                        if (bHasPhase) Details.Add(FString::Printf(TEXT("phase=%s"), *Phase));
                        if (bHasPaused) Details.Add(FString::Printf(TEXT("paused=%s"), bPaused ? TEXT("true") : TEXT("false")));
                        View.Message = FString::Printf(TEXT("%s: %s"), *Command, *FString::Join(Details, TEXT(", ")));
                    }
                    else
                    {
                        View.Message = FString::Printf(TEXT("%s accepted"), *Command);
                    }
                }
            }
        }
    }
    if (View.SimulationTimeNanoseconds == 0)
    {
        if (UBskRenderWorldSubsystem* RenderSubsystem = GetWorld() ? GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>() : nullptr)
        {
            View.SimulationTimeNanoseconds = RenderSubsystem->GetSimulationTimeNanoseconds();
        }
    }
    MissionEventHistory.Add(MoveTemp(View));
    constexpr int32 MaxMissionEvents = 128;
    if (MissionEventHistory.Num() > MaxMissionEvents) MissionEventHistory.RemoveAt(0, MissionEventHistory.Num() - MaxMissionEvents, EAllowShrinking::No);
    if (Event.EventKind == TEXT("scene_reset"))
    {
        ResetPresentationState();
    }
    if (UBskRenderWorldSubsystem* RenderSubsystem = GetWorld() ? GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>() : nullptr)
    {
        RenderSubsystem->NotifyEventApplied(Event);
    }
    UE_LOG(LogBskUnreal, Display, TEXT("BSK event kind=%s sequence=%lld"), *Event.EventKind, Event.Sequence);
}

AActor* ABskSceneController::SpawnManifestObject(const FBskObjectDefinition& Definition)
{
    if (UBskRenderWorldSubsystem* RenderSubsystem = GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>())
    {
        const FBskRenderSpawnContext Context{GetWorld(), this, &Converter};
        if (AActor* Actor = RenderSubsystem->TrySpawnObject(Context, Definition)) return Actor;
    }

    FBskRenderObjectState State;
    State.ObjectId = Definition.ObjectId;
    State.Name = Definition.DisplayName;
    State.AssetPath = Definition.AssetPath;
    const bool bHasConfiguredSpec = ObjectSpecs.Contains(Definition.ObjectId) || ObjectSpecs.Contains(Definition.DisplayName) || !Definition.AssetPath.IsEmpty();
    if (bHasConfiguredSpec) return SpawnFromSpec(Definition.ObjectId, ResolveSpec(State));
    if (Definition.Geometries.IsEmpty()) return nullptr;

    FActorSpawnParameters Params;
    Params.Name = MakeUniqueObjectName(GetWorld(), AActor::StaticClass(), SafeActorName(Definition.ObjectId));
    Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    AActor* Actor = GetWorld()->SpawnActor<AActor>(AActor::StaticClass(), FTransform::Identity, Params);
    if (!Actor) return nullptr;
    USceneComponent* Root = NewObject<USceneComponent>(Actor, TEXT("BodyRoot"));
    Root->SetMobility(EComponentMobility::Movable);
    Root->RegisterComponent();
    Actor->SetRootComponent(Root);
    TMap<int32, UStaticMesh*> ResolvedMeshAssets;
    bool bHasRenderableVisualMesh = false;
    for (int32 Index = 0; Index < Definition.Geometries.Num(); ++Index)
    {
        const FBskGeometryDefinition& Geometry = Definition.Geometries[Index];
        if (!Geometry.Shape.Equals(TEXT("mesh"), ESearchCase::IgnoreCase) || Geometry.AssetPath.IsEmpty()) continue;
        if (UStaticMesh* Mesh = LoadObject<UStaticMesh>(nullptr, *Geometry.AssetPath))
        {
            ResolvedMeshAssets.Add(Index, Mesh);
            if (!Geometry.RenderRole.Equals(TEXT("collision"), ESearchCase::IgnoreCase)) bHasRenderableVisualMesh = true;
        }
        else
        {
            UE_LOG(LogBskUnreal, Warning, TEXT("Unable to load mesh '%s' for geometry '%s'; collision fallback will be used"),
                *Geometry.AssetPath, *Geometry.GeometryId);
        }
    }
    for (int32 Index = 0; Index < Definition.Geometries.Num(); ++Index)
    {
        const FBskGeometryDefinition& Geometry = Definition.Geometries[Index];
        const bool bCollisionGeometry = Geometry.RenderRole.Equals(TEXT("collision"), ESearchCase::IgnoreCase);
        if (bCollisionGeometry && bHasRenderableVisualMesh) continue;
        UStaticMesh* Mesh = ResolvedMeshAssets.FindRef(Index);
        if (!Mesh && Geometry.Shape.Equals(TEXT("mesh"), ESearchCase::IgnoreCase)) continue;
        if (!Mesh) Mesh = LoadObject<UStaticMesh>(nullptr, PrimitiveMeshPath(Geometry.Shape));
        if (!Mesh) continue;
        UStaticMeshComponent* Component = NewObject<UStaticMeshComponent>(Actor, *FString::Printf(TEXT("Geometry_%d"), Index));
        Component->SetMobility(EComponentMobility::Movable);
        Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        Component->SetCastShadow(true);
        Component->SetStaticMesh(Mesh);
        Component->SetupAttachment(Root);
        Component->RegisterComponent();
        Component->SetRelativeLocation(FVector(Converter.LocalMetersToUnrealCentimeters(Geometry.PositionBodyMeters)));
        Component->SetRelativeRotation(FQuat(Converter.ActiveLocalWxyzToUnreal(Geometry.OrientationBodyFromGeometryWxyz)));
        Component->SetRelativeScale3D(FVector(ResolvedMeshAssets.Contains(Index) ? Geometry.Scale : Geometry.DimensionsMeters));
        ApplyGeometryMaterial(Component, Actor, Geometry, ActiveMaterialAmbient);
        AttachConfiguredVisualOverlay(Component, Geometry);
    }
    return Actor;
}

AActor* ABskSceneController::SpawnTexturedEarth(const FString& ActorName, double RadiusMeters)
{
    UStaticMesh* SphereMesh = LoadObject<UStaticMesh>(nullptr, *EarthSphereMeshPath);
    UMaterialInterface* SurfaceMaterial = LoadObject<UMaterialInterface>(nullptr, *EarthSurfaceMaterialPath);
    UMaterialInterface* CloudMaterial = LoadObject<UMaterialInterface>(nullptr, *EarthCloudMaterialPath);
    UMaterialInterface* AtmosphereMaterial = LoadObject<UMaterialInterface>(nullptr, *EarthAtmosphereMaterialPath);
    if (!SphereMesh || !SurfaceMaterial)
    {
        UE_LOG(LogBskUnreal, Warning,
            TEXT("Textured Earth assets unavailable mesh=%s surface=%s; using the celestial fallback"),
            SphereMesh ? TEXT("ok") : TEXT("missing"), SurfaceMaterial ? TEXT("ok") : TEXT("missing"));
        return nullptr;
    }

    const FBoxSphereBounds MeshBounds = SphereMesh->GetBounds();
    const double SourceRadiusCentimeters = FMath::Max(
        static_cast<double>(MeshBounds.BoxExtent.GetMax()), UE_DOUBLE_SMALL_NUMBER);
    const FVector SourceCenterCentimeters = MeshBounds.Origin;
    const double TargetRadiusCentimeters = FMath::Max(RadiusMeters, 1.0) * Converter.GetCentimetersPerMeter();

    FActorSpawnParameters Params;
    Params.Name = MakeUniqueObjectName(GetWorld(), AActor::StaticClass(), SafeActorName(ActorName));
    Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    AActor* Actor = GetWorld()->SpawnActor<AActor>(AActor::StaticClass(), FTransform::Identity, Params);
    if (!Actor) return nullptr;

    USceneComponent* Root = NewObject<USceneComponent>(Actor, TEXT("EarthRoot"));
    Root->SetMobility(EComponentMobility::Movable);
    Root->RegisterComponent();
    Actor->SetRootComponent(Root);

    auto AddLayer = [Actor, Root, SphereMesh, SourceRadiusCentimeters, SourceCenterCentimeters, TargetRadiusCentimeters](
        const TCHAR* Name, UMaterialInterface* Material, double ScaleRatio)
    {
        if (!Material) return static_cast<UStaticMeshComponent*>(nullptr);
        const double MeshScale = TargetRadiusCentimeters * ScaleRatio / SourceRadiusCentimeters;
        UStaticMeshComponent* Component = NewObject<UStaticMeshComponent>(Actor, Name);
        // The original MyProject2 actors are static. Runtime celestial actors must
        // remain movable, but all visual component flags match the source map.
        Component->SetMobility(EComponentMobility::Movable);
        Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        Component->SetCastShadow(true);
        Component->SetAffectDistanceFieldLighting(true);
        Component->SetAffectDynamicIndirectLighting(true);
        Component->SetReceivesDecals(true);
        Component->SetRenderInMainPass(true);
        Component->SetRenderInDepthPass(true);
        Component->SetCanEverAffectNavigation(false);
        Component->SetStaticMesh(SphereMesh);
        Component->SetMaterial(0, Material);
        Component->SetRelativeScale3D(FVector(MeshScale));
        // The migrated Modeling Tools sphere has its pivot at the south pole.
        // Offset its imported bounds origin so the actor transform remains the
        // Basilisk/MJScene celestial centre while preserving the exact source mesh.
        Component->SetRelativeLocation(-SourceCenterCentimeters * MeshScale);
        Component->SetTranslucentSortPriority(0);
        Component->SetupAttachment(Root);
        Component->RegisterComponent();
        return Component;
    };

    AddLayer(TEXT("EarthSurface"), SurfaceMaterial, 1.0);
    if (!AddLayer(TEXT("EarthClouds"), CloudMaterial, EarthCloudScale))
    {
        UE_LOG(LogBskUnreal, Warning, TEXT("Earth cloud material unavailable: %s"), *EarthCloudMaterialPath);
    }
    if (!bUseEarthSkyAtmosphere && !AddLayer(TEXT("EarthAtmosphereShell"), AtmosphereMaterial, EarthAtmosphereScale))
    {
        UE_LOG(LogBskUnreal, Warning, TEXT("Earth atmosphere-shell material unavailable: %s"), *EarthAtmosphereMaterialPath);
    }
    UE_LOG(LogBskUnreal, Display,
        TEXT("Created MyProject2-configured Earth %s mesh=%s radius=%.3f km clouds=%.4fx atmosphere=%.4fx"),
        *ActorName, *EarthSphereMeshPath, RadiusMeters / 1000.0, EarthCloudScale, EarthAtmosphereScale);
    return Actor;
}

AActor* ABskSceneController::SpawnCelestialBody(const FBskCelestialBodyDefinition& Definition)
{
    if (UBskRenderWorldSubsystem* RenderSubsystem = GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>())
    {
        const FBskRenderSpawnContext Context{GetWorld(), this, &Converter};
        if (AActor* Actor = RenderSubsystem->TrySpawnCelestialBody(Context, Definition)) return Actor;
    }

    if (bUseOfficialCelestialAssets && Definition.BodyId.Equals(TEXT("moon"), ESearchCase::IgnoreCase))
    {
        UStaticMesh* MoonPlane = LoadObject<UStaticMesh>(nullptr, TEXT("/CelestialVault/Meshes/SM_Plane_FacingX.SM_Plane_FacingX"));
        UMaterialInterface* MoonMaterial = LoadObject<UMaterialInterface>(nullptr, TEXT("/CelestialVault/Materials/MI_Moon.MI_Moon"));
        if (MoonPlane && MoonMaterial)
        {
            FActorSpawnParameters Params;
            Params.Name = MakeUniqueObjectName(GetWorld(), AStaticMeshActor::StaticClass(), SafeActorName(TEXT("celestial_moon")));
            Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
            AStaticMeshActor* Actor = GetWorld()->SpawnActor<AStaticMeshActor>(AStaticMeshActor::StaticClass(), FTransform::Identity, Params);
            if (Actor)
            {
                UStaticMeshComponent* Component = Actor->GetStaticMeshComponent();
                Component->SetMobility(EComponentMobility::Movable);
                Component->SetStaticMesh(MoonPlane);
                Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
                Component->SetCastShadow(false);
                UMaterialInstanceDynamic* Instance = UMaterialInstanceDynamic::Create(MoonMaterial, Actor);
                Instance->SetScalarParameterValue(TEXT("MoonAge"), 0.5f);
                Instance->SetScalarParameterValue(TEXT("Brightness"), 1.15f);
                Instance->SetScalarParameterValue(TEXT("EarthLightContribution"), 0.06f);
                Component->SetMaterial(0, Instance);
                const double DiameterMeters = 2.0 * FMath::Max(Definition.EquatorialRadiusMeters, 1.0);
                Actor->SetActorScale3D(FVector(DiameterMeters));
                Actor->Tags.AddUnique(FName(*FString::Printf(TEXT("BSK.Celestial.%s"), *Definition.BodyId)));
                CelestialBillboardIds.Add(Definition.BodyId);
                UE_LOG(LogBskUnreal, Display, TEXT("Using Epic Celestial Vault Moon material for %s"), *Definition.BodyId);
                return Actor;
            }
        }
        UE_LOG(LogBskUnreal, Warning, TEXT("Epic Celestial Vault Moon assets unavailable; using the BSK fallback sphere"));
    }

    AActor* Actor = nullptr;
    if (Definition.BodyId.Equals(TEXT("earth"), ESearchCase::IgnoreCase))
    {
        Actor = SpawnTexturedEarth(TEXT("celestial_earth"), Definition.EquatorialRadiusMeters);
    }
    if (!Actor)
    {
        FObjectSpec Spec;
        Spec.PlaceholderShape = TEXT("sphere");
        const double DiameterMeters = 2.0 * FMath::Max(Definition.EquatorialRadiusMeters, 1.0);
        Spec.SizeMeters = FVector3d(DiameterMeters, DiameterMeters, DiameterMeters * Definition.PolarRadiusRatio);
        if (Definition.BodyId.Equals(TEXT("earth"), ESearchCase::IgnoreCase)) Spec.Color = FLinearColor(0.03f, 0.16f, 0.65f);
        else if (Definition.BodyId.Equals(TEXT("moon"), ESearchCase::IgnoreCase)) Spec.Color = FLinearColor(0.45f, 0.45f, 0.48f);
        else if (Definition.bLuminous) Spec.Color = FLinearColor(3.0f, 2.5f, 1.2f);
        else Spec.Color = FLinearColor(0.35f, 0.25f, 0.18f);
        Actor = SpawnPlaceholder(FString::Printf(TEXT("celestial_%s"), *Definition.BodyId), Spec);
    }
    if (Actor) Actor->Tags.AddUnique(FName(*FString::Printf(TEXT("BSK.Celestial.%s"), *Definition.BodyId)));

    if (Actor && bUseEarthSkyAtmosphere && Definition.BodyId.Equals(TEXT("earth"), ESearchCase::IgnoreCase) && !EarthAtmosphere)
    {
        FActorSpawnParameters Params;
        Params.Name = MakeUniqueObjectName(GetWorld(), ASkyAtmosphere::StaticClass(), TEXT("BSK_EarthAtmosphere"));
        Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
        EarthAtmosphere = GetWorld()->SpawnActor<ASkyAtmosphere>(ASkyAtmosphere::StaticClass(), FTransform::Identity, Params);
        if (EarthAtmosphere)
        {
            USkyAtmosphereComponent* Atmosphere = EarthAtmosphere->GetComponent();
            Atmosphere->TransformMode = ESkyAtmosphereTransformMode::PlanetCenterAtComponentTransform;
            Atmosphere->SetBottomRadius(static_cast<float>(Definition.EquatorialRadiusMeters / 1000.0));
            Atmosphere->SetAtmosphereHeight(100.0f);
            Atmosphere->SetMultiScatteringFactor(1.0f);
            EarthAtmosphere->SetActorHiddenInGame(true);
            UE_LOG(LogBskUnreal, Display, TEXT("Created UE SkyAtmosphere for BSK Earth radius %.3f km"), Definition.EquatorialRadiusMeters / 1000.0);
        }
    }
    return Actor;
}

AActor* ABskSceneController::SpawnVisual(const FBskVisualDefinition& Definition)
{
    if (UBskRenderWorldSubsystem* RenderSubsystem = GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>())
    {
        const FBskRenderSpawnContext Context{GetWorld(), this, &Converter};
        if (AActor* Actor = RenderSubsystem->TrySpawnVisual(Context, Definition)) return Actor;
    }

    if (Definition.Kind.Equals(TEXT("light"), ESearchCase::IgnoreCase))
    {
        FActorSpawnParameters Params;
        Params.Name = MakeUniqueObjectName(GetWorld(), AActor::StaticClass(), SafeActorName(Definition.VisualId));
        Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
        AActor* LightActor = nullptr;
        ULightComponent* LightComponent = nullptr;
        if (Definition.LightType.Equals(TEXT("directional"), ESearchCase::IgnoreCase))
        {
            ADirectionalLight* Light = GetWorld()->SpawnActor<ADirectionalLight>(ADirectionalLight::StaticClass(), FTransform::Identity, Params);
            LightActor = Light;
            LightComponent = Light ? Light->GetLightComponent() : nullptr;
            if (LightComponent) LightComponent->SetIntensity(static_cast<float>(10.0 * FMath::Max(0.0, Definition.LightIntensity)));
        }
        else
        {
            ASpotLight* Light = GetWorld()->SpawnActor<ASpotLight>(ASpotLight::StaticClass(), FTransform::Identity, Params);
            LightActor = Light;
            LightComponent = Light ? Light->GetLightComponent() : nullptr;
            if (USpotLightComponent* Spot = Light ? Cast<USpotLightComponent>(Light->GetLightComponent()) : nullptr)
            {
                Spot->SetIntensity(static_cast<float>(650.0 * FMath::Max(0.0, Definition.LightIntensity)));
                Spot->SetAttenuationRadius(static_cast<float>(FMath::Max(Definition.RangeMeters, 20.0) * Converter.GetCentimetersPerMeter()));
                Spot->SetInnerConeAngle(static_cast<float>(FMath::Clamp(Definition.LightCutoffDegrees * 0.65, 0.0, 80.0)));
                Spot->SetOuterConeAngle(static_cast<float>(FMath::Clamp(Definition.LightCutoffDegrees, 1.0, 89.0)));
            }
        }
        if (!LightActor || !LightComponent) return nullptr;
        // MJCF target-body lights are rotated as their target moves. Unreal
        // defaults spawned light components to a non-movable mobility, which
        // otherwise emits a warning on every rendered frame and stalls the
        // Game Thread with log I/O.
        LightComponent->SetMobility(EComponentMobility::Movable);
        const double Peak = FMath::Max3(Definition.LightDiffuseRgb.X, Definition.LightDiffuseRgb.Y, Definition.LightDiffuseRgb.Z);
        const FVector3d Normalized = Peak > UE_DOUBLE_SMALL_NUMBER ? Definition.LightDiffuseRgb / Peak : FVector3d::OneVector;
        LightComponent->SetLightColor(FLinearColor(static_cast<float>(Normalized.X), static_cast<float>(Normalized.Y), static_cast<float>(Normalized.Z)));
        const double SpecularPeak = FMath::Max3(Definition.LightSpecularRgb.X, Definition.LightSpecularRgb.Y, Definition.LightSpecularRgb.Z);
        LightComponent->SetSpecularScale(static_cast<float>(Peak > UE_DOUBLE_SMALL_NUMBER ? SpecularPeak / Peak : 0.0));
        LightComponent->SetCastShadows(Definition.bLightCastShadows);
        ApplyVisualMountTransform(LightActor, Definition);
        LightActor->Tags.AddUnique(FName(*FString::Printf(TEXT("BSK.Light.%s"), *Definition.VisualId)));
        return LightActor;
    }

    FObjectSpec Spec;
    Spec.PlaceholderShape = Definition.Kind == TEXT("reaction_wheel") ? TEXT("cylinder") : TEXT("cone");
    Spec.Color = Definition.Color;
    const double LengthMeters = Definition.RangeMeters > 0.0 ? Definition.RangeMeters : FMath::Max(Definition.SizeMeters, 0.1);
    if (Definition.Kind == TEXT("reaction_wheel"))
    {
        Spec.SizeMeters = FVector3d(
            FMath::Max(Definition.SizeMeters, 0.05),
            FMath::Max(Definition.SizeMeters, 0.05),
            FMath::Max(LengthMeters, 0.02));
    }
    else
    {
        const double FovRadians = Definition.FieldOfViewRadians.X > 0.0 ? Definition.FieldOfViewRadians.X : PI / 6.0;
        const double DiameterMeters = 2.0 * LengthMeters * FMath::Tan(0.5 * FovRadians);
        Spec.SizeMeters = FVector3d(FMath::Max(DiameterMeters, 0.02), FMath::Max(DiameterMeters, 0.02), LengthMeters);
    }
    AActor* Actor = SpawnPlaceholder(FString::Printf(TEXT("visual_%s"), *Definition.VisualId), Spec);
    if (!Actor) return nullptr;
    if (UStaticMeshComponent* Component = Actor->FindComponentByClass<UStaticMeshComponent>())
    {
        Component->SetCastShadow(false);
    }
    ApplyVisualMountTransform(Actor, Definition);
    return Actor;
}

void ABskSceneController::ApplyVisualMountTransform(AActor* Actor, const FBskVisualDefinition& Definition) const
{
    if (!Actor) return;
    FVector Location = FVector(Converter.LocalMetersToUnrealCentimeters(Definition.PositionBodyMeters));
    const FVector3d LocalNormal = Definition.NormalBody.GetSafeNormal();
    const FVector Direction = FVector(Converter.LocalMetersToUnrealCentimeters(LocalNormal)).GetSafeNormal();
    FQuat Rotation = Direction.IsNearlyZero() ? FQuat::Identity : FRotationMatrix::MakeFromZ(Direction).ToQuat();
    if (Definition.Kind.Equals(TEXT("light"), ESearchCase::IgnoreCase))
    {
        Rotation = Direction.IsNearlyZero() ? FQuat::Identity : Direction.Rotation().Quaternion();
    }
    if (!Direction.IsNearlyZero() && IsSensorFrustumKind(Definition.Kind))
    {
        const double LengthMeters = Definition.RangeMeters > 0.0
            ? Definition.RangeMeters
            : FMath::Max(Definition.SizeMeters, 0.1);
        // UE's basic cone is centered and points along +Z. Put its apex at the
        // sensor mount and let the volume expand along the sensor boresight.
        Location += FVector(Converter.LocalMetersToUnrealCentimeters(LocalNormal * (0.5 * LengthMeters)));
        Rotation = FRotationMatrix::MakeFromZ(-Direction).ToQuat();
    }
    Actor->SetActorRelativeLocation(Location);
    Actor->SetActorRelativeRotation(Rotation);
}

void ABskSceneController::ConfigureManifestLighting(const FBskSceneManifest& Manifest)
{
    bUseManifestSceneLighting = Manifest.bUseSceneLighting;
    SceneSunlightIntensityScale = Manifest.SunlightIntensityScale;
    const float InitialSunLux = static_cast<float>(BskCelestialLighting::ScaleIlluminanceLux(
        SunIntensityLux, SceneSunlightIntensityScale));
    UE_LOG(LogBskUnreal, Display, TEXT("Scene sunlight intensity scale=%.4f (rendering only)"),
        SceneSunlightIntensityScale);
    const double AmbientPeak = FMath::Max3(Manifest.HeadlightAmbientRgb.X, Manifest.HeadlightAmbientRgb.Y, Manifest.HeadlightAmbientRgb.Z);
    ActiveMaterialAmbient = Manifest.bUseSceneLighting ? 1.8 * AmbientPeak : 0.18;
    if (SunLight) SunLight->GetLightComponent()->SetIntensity(Manifest.bUseSceneLighting ? 0.0f : InitialSunLux);
    if (CelestialSunLight) CelestialSunLight->GetLightComponent()->SetIntensity(Manifest.bUseSceneLighting ? 0.0f : InitialSunLux);
    const double ManifestFillIntensity = Manifest.FillLightIntensityLux >= 0.0
        ? Manifest.FillLightIntensityLux
        : FillLightIntensityLux;
    if (FillLight) FillLight->GetLightComponent()->SetIntensity(
        Manifest.bUseSceneLighting ? 0.0f : static_cast<float>(ManifestFillIntensity));
    if (!Manifest.bHeadlightEnabled)
    {
        if (Headlight) Headlight->SetActorHiddenInGame(true);
        return;
    }
    if (!Headlight)
    {
        FActorSpawnParameters Params;
        Params.Name = MakeUniqueObjectName(GetWorld(), ASpotLight::StaticClass(), TEXT("BSK_MJCF_Headlight"));
        Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
        Headlight = GetWorld()->SpawnActor<ASpotLight>(ASpotLight::StaticClass(), FTransform::Identity, Params);
        if (Headlight)
        {
            if (APlayerController* Player = GetWorld()->GetFirstPlayerController())
            {
                if (ABskCameraPawn* Pawn = Cast<ABskCameraPawn>(Player->GetPawn()))
                {
                    Headlight->AttachToComponent(Pawn->GetBskCameraComponent(), FAttachmentTransformRules::SnapToTargetNotIncludingScale);
                }
            }
        }
    }
    if (!Headlight) return;
    Headlight->SetActorHiddenInGame(false);
    const double Peak = FMath::Max3(Manifest.HeadlightDiffuseRgb.X, Manifest.HeadlightDiffuseRgb.Y, Manifest.HeadlightDiffuseRgb.Z);
    const FVector3d Normalized = Peak > UE_DOUBLE_SMALL_NUMBER ? Manifest.HeadlightDiffuseRgb / Peak : FVector3d::OneVector;
    USpotLightComponent* Spot = CastChecked<USpotLightComponent>(Headlight->GetLightComponent());
    Spot->SetLightColor(FLinearColor(static_cast<float>(Normalized.X), static_cast<float>(Normalized.Y), static_cast<float>(Normalized.Z)));
    const double SpecularPeak = FMath::Max3(Manifest.HeadlightSpecularRgb.X, Manifest.HeadlightSpecularRgb.Y, Manifest.HeadlightSpecularRgb.Z);
    Spot->SetSpecularScale(static_cast<float>(Peak > UE_DOUBLE_SMALL_NUMBER ? SpecularPeak / Peak : 0.0));
    Spot->SetIntensity(static_cast<float>(1200.0 * Peak));
    Spot->SetAttenuationRadius(10000.0f);
    Spot->SetInnerConeAngle(50.0f);
    Spot->SetOuterConeAngle(80.0f);
    Spot->SetCastShadows(false);
    if (ExposureVolume)
    {
        ExposureVolume->Settings.AutoExposureBias = static_cast<float>(MaterialExposureBias);
    }
}

void ABskSceneController::RefreshVisualVisibility(const FString& VisualId)
{
    AActor* Actor = VisualActors.FindRef(VisualId);
    const FBskVisualDefinition* Definition = ManifestVisuals.Find(VisualId);
    if (!Actor || !Definition) return;
    const bool* DynamicVisibility = VisualDynamicVisibility.Find(VisualId);
    const bool bDynamicallyVisible = DynamicVisibility == nullptr || *DynamicVisibility;
    Actor->SetActorHiddenInGame(!bDynamicallyVisible || !IsVisualKindVisible(Definition->Kind));
}

AActor* ABskSceneController::SpawnCamera(const FBskCameraDefinition& Definition)
{
    if (UBskRenderWorldSubsystem* RenderSubsystem = GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>())
    {
        const FBskRenderSpawnContext Context{GetWorld(), this, &Converter};
        if (AActor* Actor = RenderSubsystem->TrySpawnCamera(Context, Definition)) return Actor;
    }

    FActorSpawnParameters Params;
    Params.Name = MakeUniqueObjectName(GetWorld(), ACameraActor::StaticClass(), SafeActorName(Definition.CameraId));
    ACameraActor* Camera = GetWorld()->SpawnActor<ACameraActor>(ACameraActor::StaticClass(), FTransform::Identity, Params);
    if (!Camera) return nullptr;
    Camera->Tags.AddUnique(FName(*FString::Printf(TEXT("BSK.Camera.%s"), *Definition.CameraId)));
    return Camera;
}

void ABskSceneController::ConfigureCamera(AActor* Actor, const FBskCameraDefinition& Definition)
{
    if (!Actor) return;
    const float FieldOfViewDegrees = static_cast<float>(FMath::RadiansToDegrees(Definition.FieldOfViewRadians));
    Actor->SetActorRelativeLocation(FVector(Converter.LocalMetersToUnrealCentimeters(Definition.PositionBodyMeters)));
    Actor->SetActorRelativeRotation(FQuat(Converter.ActiveLocalWxyzToUnreal(Definition.OrientationBodyFromCameraWxyz)));
    if (ACameraActor* CameraActor = Cast<ACameraActor>(Actor))
    {
        CameraActor->GetCameraComponent()->FieldOfView = FieldOfViewDegrees;
    }

    USceneCaptureComponent2D* Capture = CameraCaptureComponents.FindRef(Definition.CameraId);
    const bool bHasDataProducts = !CaptureProductOverride.IsEmpty() || !Definition.CaptureProducts.IsEmpty();
    const bool bPixelStreamingRequested = IsPixelStreamingCameraRequested(Definition.CameraId);
    if (!Definition.bPictureInPicture && !bHasDataProducts && !bPixelStreamingRequested)
    {
        if (Capture) Capture->Deactivate();
        CameraRenderTargets.Remove(Definition.CameraId);
        CameraNextCaptureSeconds.Remove(Definition.CameraId);
        CameraNextDataCaptureSimulationNanoseconds.Remove(Definition.CameraId);
        CameraPictureInPictureVisibility.Remove(Definition.CameraId);
        return;
    }

    if (!Capture)
    {
        Capture = NewObject<USceneCaptureComponent2D>(Actor, MakeUniqueObjectName(Actor, USceneCaptureComponent2D::StaticClass(), TEXT("BskRgbCapture")));
        if (!Capture) return;
        Capture->SetupAttachment(Actor->GetRootComponent());
        Capture->RegisterComponent();
        Capture->SetRelativeTransform(FTransform::Identity);
        Capture->bCaptureEveryFrame = false;
        Capture->bCaptureOnMovement = false;
        Capture->bAlwaysPersistRenderingState = true;
        Capture->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
        Capture->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_RenderScenePrimitives;
        CameraCaptureComponents.Add(Definition.CameraId, Capture);
    }
    ConfigureFixedRgbExposure(Capture);
    Capture->bAlwaysPersistRenderingState = true;
    Capture->Activate();
    Capture->FOVAngle = FieldOfViewDegrees;

    UTextureRenderTarget2D* RenderTarget = CameraRenderTargets.FindRef(Definition.CameraId);
    if (!RenderTarget || RenderTarget->SizeX != Definition.Resolution.X || RenderTarget->SizeY != Definition.Resolution.Y)
    {
        RenderTarget = NewObject<UTextureRenderTarget2D>(this, MakeUniqueObjectName(this, UTextureRenderTarget2D::StaticClass(), TEXT("BskSharedCameraTarget")));
        RenderTarget->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8;
        RenderTarget->bAutoGenerateMips = false;
        RenderTarget->InitAutoFormat(Definition.Resolution.X, Definition.Resolution.Y);
        ConfigureRgbRenderTarget(RenderTarget);
        RenderTarget->UpdateResourceImmediate(true);
        CameraRenderTargets.Add(Definition.CameraId, RenderTarget);
    }
    Capture->TextureTarget = RenderTarget;
    if (Definition.bPictureInPicture && !CameraPictureInPictureVisibility.Contains(Definition.CameraId))
    {
        CameraPictureInPictureVisibility.Add(Definition.CameraId, true);
    }
    CameraNextCaptureSeconds.Add(Definition.CameraId, 0.0);
    CameraNextDataCaptureSimulationNanoseconds.Add(Definition.CameraId, 0);

    // Configure the standalone streamer only after the canonical capture and
    // target exist, so the HUD inset and selected camera are pixel-identical.
    ConfigurePixelStreamingCamera(Actor, Definition);
}

void ABskSceneController::UpdatePictureInPictureCaptures()
{
    if (!GetWorld()) return;
    const double NowSeconds = FPlatformTime::Seconds();
    for (const TPair<FString, FBskCameraDefinition>& Pair : ManifestCameras)
    {
        const FBskCameraDefinition& Definition = Pair.Value;
        if (!Definition.bPictureInPicture || !CameraPictureInPictureVisibility.FindRef(Pair.Key)) continue;
        USceneCaptureComponent2D* Capture = CameraCaptureComponents.FindRef(Pair.Key);
        if (!Capture || !Capture->TextureTarget) continue;
        double PreviewRate = PreviewRateOverrideHertz > 0.0 ? PreviewRateOverrideHertz : Definition.CaptureRateHertz;
        if (IsPixelStreamingCameraRequested(Pair.Key))
        {
            // Prioritize the view actually subscribed by a browser. Keep unselected
            // HUD thumbnails at <=15 Hz rather than rendering all three views at 90.
            PreviewRate = BskPreviewRateForViewers(PixelStreamingCameraRateHertz,
                HasPixelStreamingCameraViewers(Pair.Key), HasPixelStreamingViewportViewers());
            if (PreviewRate <= 0.0) continue;
        }
        double& NextSeconds = CameraNextCaptureSeconds.FindOrAdd(Pair.Key);
        if (!BskPreviewCaptureDue(NowSeconds, FMath::Max(1.0, PreviewRate), NextSeconds)) continue;
        Capture->CaptureScene();
        if (bVideoDiagnostics) ++VideoDiagnosticsCaptures.FindOrAdd(Pair.Key);
        if (PreviewRateOverrideHertz > 0.0 && CaptureNetworkSender && bHasPresentationFrame)
        {
            FBskCaptureRequest Request;
            Request.CameraId = Pair.Key;
            Request.Channels.Add(EBskCaptureChannel::Rgb);
            Request.Resolution = Definition.Resolution;
            Request.SimulationTimeNanoseconds = PresentationFrame.SimulationTimeNanoseconds;
            Request.SourceWallTimeNanoseconds = PresentationFrame.WallTimeNanoseconds;
            Request.FrameId = PresentationFrame.FrameId;
            Request.OriginInertialMeters = PresentationFrame.OriginInertialMeters;
            Request.LocalFromInertial = PresentationFrame.LocalFromInertial;
            Request.Purpose = EBskCapturePurpose::Preview;
            Request.bReuseExistingRgbTarget = true;
            Request.bSendToNetwork = true;
            FString Error;
            UBskRenderWorldSubsystem* RenderSubsystem = GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>();
            if (!RenderSubsystem || !RenderSubsystem->RequestCapture(Request, Error))
            {
                UE_LOG(LogBskUnreal, Warning, TEXT("Preview capture for %s failed: %s"), *Pair.Key, *Error);
            }
        }
    }
}

void ABskSceneController::UpdateAuthoritativeDataProductCaptures(const FBskRenderFrame& AuthoritativeFrame)
{
    // Capability alone never triggers dataset readback; only tagged active frames do.
    if (AuthoritativeFrame.bCaptureOnDemand && AuthoritativeFrame.CaptureEpisodeId.IsEmpty()) return;
    if (!GetWorld() || (CaptureOutputDirectory.IsEmpty() && !CaptureNetworkSender)) return;
    UBskRenderWorldSubsystem* RenderSubsystem = GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>();
    if (!RenderSubsystem) return;
    TArray<FString> DueCameras;
    for (const TPair<FString, FBskCameraDefinition>& Pair : ManifestCameras)
    {
        const TArray<FString>& Products = CaptureProductOverride.IsEmpty() ? Pair.Value.CaptureProducts : CaptureProductOverride;
        if (Products.IsEmpty()) continue;
        const double DatasetRate = CaptureRateOverrideHertz > 0.0 ? CaptureRateOverrideHertz : Pair.Value.CaptureRateHertz;
        int64 DatasetSampleIndex = 0;
        if (!BskLeRobotSample(AuthoritativeFrame.SimulationTimeNanoseconds, DatasetRate, DatasetSampleIndex)) continue;
        const int64 NextCaptureNanoseconds = CameraNextDataCaptureSimulationNanoseconds.FindRef(Pair.Key);
        if (AuthoritativeFrame.SimulationTimeNanoseconds < NextCaptureNanoseconds) continue;
        DueCameras.Add(Pair.Key);
    }
    if (DueCameras.IsEmpty()) return;

    // Snapshot capture and smooth presentation share one UE world, so apply the
    // exact source frame for the whole capture pass, then restore the
    // presentation frame. This never writes data back to BSK/MJScene.
    ApplyFrame(AuthoritativeFrame, false);

    const auto BuildCaptureRequest = [&](const FString& CameraId, const FBskCameraDefinition& Definition,
                                         bool bRenderOnly)
    {
        FBskCaptureRequest Request;
        Request.CameraId = CameraId;
        Request.Resolution = Definition.Resolution;
        Request.SimulationTimeNanoseconds = AuthoritativeFrame.SimulationTimeNanoseconds;
        Request.SourceWallTimeNanoseconds = AuthoritativeFrame.WallTimeNanoseconds;
        Request.FrameId = AuthoritativeFrame.FrameId;
        Request.CaptureEpisodeId = AuthoritativeFrame.CaptureEpisodeId;
        Request.OriginInertialMeters = AuthoritativeFrame.OriginInertialMeters;
        Request.LocalFromInertial = AuthoritativeFrame.LocalFromInertial;
        Request.Purpose = EBskCapturePurpose::AuthoritativeDataset;
        Request.OutputDirectory = CaptureOutputDirectory;
        Request.bWriteToDisk = !CaptureOutputDirectory.IsEmpty();
        Request.bSendToNetwork = CaptureNetworkSender.IsValid();
        Request.bSkipReadback = bRenderOnly;
        Request.bReuseExistingRgbTarget = !bRenderOnly;
        const TArray<FString>& Products = CaptureProductOverride.IsEmpty()
            ? Definition.CaptureProducts : CaptureProductOverride;
        for (const FString& Product : Products)
        {
            if (Product == TEXT("rgb")) Request.Channels.Add(EBskCaptureChannel::Rgb);
        }
        return Request;
    };

    // Submit every due camera before reading them individually. This avoids
    // alternating render/readback submissions but ReadPixels is still synchronous.
    TArray<FString> RenderedCameras;
    for (const FString& CameraId : DueCameras)
    {
        const FBskCameraDefinition* Definition = ManifestCameras.Find(CameraId);
        if (!Definition) continue;
        FBskCaptureRequest Request = BuildCaptureRequest(CameraId, *Definition, true);
        if (Request.Channels.IsEmpty()) continue;
        FString Error;
        if (!RenderSubsystem->RequestCapture(Request, Error))
        {
            UE_LOG(LogBskUnreal, Error, TEXT("Authoritative capture render for %s frame=%lld failed: %s"),
                *CameraId, AuthoritativeFrame.FrameId, *Error);
            ReportCaptureFailure(Request, Error);
            continue;
        }
        RenderedCameras.Add(CameraId);
    }
    for (const FString& CameraId : RenderedCameras)
    {
        const FBskCameraDefinition* Definition = ManifestCameras.Find(CameraId);
        if (!Definition) continue;
        FBskCaptureRequest Request = BuildCaptureRequest(CameraId, *Definition, false);
        if (Request.Channels.IsEmpty()) continue;
        FString Error;
        if (!RenderSubsystem->RequestCapture(Request, Error))
        {
            UE_LOG(LogBskUnreal, Error, TEXT("Authoritative capture for %s frame=%lld failed: %s"),
                *CameraId, AuthoritativeFrame.FrameId, *Error);
            ReportCaptureFailure(Request, Error);
        }
        CameraNextDataCaptureSimulationNanoseconds.Add(CameraId, AuthoritativeFrame.SimulationTimeNanoseconds + 1);
    }
    if (bHasPresentationFrame) ApplyFrame(PresentationFrame, false);
}

bool ABskSceneController::CaptureCameraDataProducts(const FBskCaptureRequest& Request, FString& OutError)
{
    check(IsInGameThread());
    const FBskCameraDefinition* Definition = ManifestCameras.Find(Request.CameraId);
    AActor* CameraActor = CameraActors.FindRef(Request.CameraId);
    if (!Definition || !CameraActor)
    {
        OutError = FString::Printf(TEXT("camera '%s' is not registered in the active manifest"), *Request.CameraId);
        return false;
    }
    if (Request.Channels.IsEmpty())
    {
        OutError = TEXT("capture request contains no channels");
        return false;
    }
    if (!Request.bWriteToDisk && !Request.bSendToNetwork)
    {
        OutError = TEXT("capture request has neither disk nor network output enabled");
        return false;
    }
    const FIntPoint Resolution(
        FMath::Clamp(Request.Resolution.X, 64, 4096),
        FMath::Clamp(Request.Resolution.Y, 64, 4096));
    const float FieldOfViewDegrees = static_cast<float>(FMath::RadiansToDegrees(Definition->FieldOfViewRadians));

    const auto EnsureCapture = [&](TMap<FString, TObjectPtr<USceneCaptureComponent2D>>& Components,
                                   TMap<FString, TObjectPtr<UTextureRenderTarget2D>>& Targets,
                                   const TCHAR* ComponentName, ETextureRenderTargetFormat Format,
                                   ESceneCaptureSource Source) -> USceneCaptureComponent2D*
    {
        USceneCaptureComponent2D* Capture = Components.FindRef(Request.CameraId);
        if (!Capture)
        {
            Capture = NewObject<USceneCaptureComponent2D>(CameraActor,
                MakeUniqueObjectName(CameraActor, USceneCaptureComponent2D::StaticClass(), FName(ComponentName)));
            if (!Capture) return nullptr;
            Capture->SetupAttachment(CameraActor->GetRootComponent());
            Capture->RegisterComponent();
            Capture->SetRelativeTransform(FTransform::Identity);
            Capture->bCaptureEveryFrame = false;
            Capture->bCaptureOnMovement = false;
            Capture->bAlwaysPersistRenderingState = false;
            Capture->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_RenderScenePrimitives;
            Components.Add(Request.CameraId, Capture);
        }
        Capture->CaptureSource = Source;
        if (Source == ESceneCaptureSource::SCS_FinalColorLDR) ConfigureFixedRgbExposure(Capture);
        Capture->FOVAngle = FieldOfViewDegrees;
        Capture->Activate();
        UTextureRenderTarget2D* Target = Targets.FindRef(Request.CameraId);
        if (!Target || Target->SizeX != Resolution.X || Target->SizeY != Resolution.Y || Target->RenderTargetFormat != Format)
        {
            Target = NewObject<UTextureRenderTarget2D>(this,
                MakeUniqueObjectName(this, UTextureRenderTarget2D::StaticClass(), FName(*FString::Printf(TEXT("%sTarget"), ComponentName))));
            Target->RenderTargetFormat = Format;
            Target->ClearColor = FLinearColor::Black;
            Target->bAutoGenerateMips = false;
            Target->InitAutoFormat(Resolution.X, Resolution.Y);
            if (Format == ETextureRenderTargetFormat::RTF_RGBA8) ConfigureRgbRenderTarget(Target);
            Target->UpdateResourceImmediate(true);
            Targets.Add(Request.CameraId, Target);
        }
        Capture->TextureTarget = Target;
        return Capture;
    };

    if (Request.bSkipReadback)
    {
        // Render-only phase for one camera. The matching readback request runs
        // only after every due camera has been rendered.
        for (const EBskCaptureChannel Channel : Request.Channels)
        {
            if (Channel != EBskCaptureChannel::Rgb)
            {
                OutError = TEXT("only RGB capture is supported");
                return false;
            }
            USceneCaptureComponent2D* Capture = EnsureCapture(
                CameraCaptureComponents, CameraRenderTargets, TEXT("BskRgbCapture"),
                ETextureRenderTargetFormat::RTF_RGBA8, ESceneCaptureSource::SCS_FinalColorLDR);
            if (!Capture || !Capture->TextureTarget)
            {
                OutError = TEXT("could not create RGB capture resources");
                return false;
            }
            Capture->CaptureScene();
        }
        return true;
    }

    TArray<FCapturedDataProduct> Products;
    TSharedPtr<FBskAsyncCaptureJob, ESPMode::ThreadSafe> AsyncJob;
    for (const EBskCaptureChannel Channel : Request.Channels)
    {
        if (Channel == EBskCaptureChannel::Rgb)
        {
            USceneCaptureComponent2D* Capture = EnsureCapture(
                CameraCaptureComponents, CameraRenderTargets, TEXT("BskRgbCapture"),
                ETextureRenderTargetFormat::RTF_RGBA8, ESceneCaptureSource::SCS_FinalColorLDR);
            if (!Capture || !Capture->TextureTarget)
            {
                OutError = TEXT("could not create RGB capture resources");
                return false;
            }
            if (!Request.bReuseExistingRgbTarget) Capture->CaptureScene();
            if (bAsyncCaptureEnabled && Request.Purpose == EBskCapturePurpose::AuthoritativeDataset)
            {
                const uint64 Bytes = static_cast<uint64>(Resolution.X) * Resolution.Y * sizeof(FColor) + 65536;
                if (PendingCaptureJobs.Num() >= MaxAsyncCaptureJobs || PendingCaptureBytes + Bytes > MaxAsyncCaptureBytes)
                {
                    OutError = TEXT("async capture admission was not reserved");
                    return false;
                }
                FTextureRHIRef Texture = Capture->TextureTarget->GameThread_GetRenderTargetResource()->GetRenderTargetTexture();
                if (!Texture || (Texture->GetFormat() != PF_B8G8R8A8 && Texture->GetFormat() != PF_R8G8B8A8))
                {
                    OutError = TEXT("async RGB readback requires an 8-bit BGRA/RGBA render target");
                    return false;
                }
                AsyncJob = MakeShared<FBskAsyncCaptureJob, ESPMode::ThreadSafe>();
                AsyncJob->Request = Request;
                AsyncJob->Resolution = Resolution;
                AsyncJob->ReservedBytes = Bytes;
                AsyncJob->SubmittedSeconds = FPlatformTime::Seconds();
                AsyncJob->bRgba = Texture->GetFormat() == PF_R8G8B8A8;
                AsyncJob->Readback = MakeShared<FRHIGPUTextureReadback, ESPMode::ThreadSafe>(TEXT("BskStrictRgb"));
                // Load the module/create the wrapper here, never from a worker.
                AsyncJob->Wrapper = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper")).CreateImageWrapper(EImageFormat::JPEG);
                ENQUEUE_RENDER_COMMAND(BskEnqueueRgbReadback)([Job = AsyncJob, Texture](FRHICommandListImmediate& RHICmdList)
                {
                    Job->Readback->EnqueueCopy(RHICmdList, Texture);
                    Job->bCopySubmitted.Store(true);
                });
                continue;
            }
            TArray<FColor> Pixels;
            FReadSurfaceDataFlags ReadFlags(RCM_UNorm);
            ReadFlags.SetLinearToGamma(false);
            if (!Capture->TextureTarget->GameThread_GetRenderTargetResource()->ReadPixels(Pixels, ReadFlags) || Pixels.Num() != Resolution.X * Resolution.Y)
            {
                OutError = TEXT("RGB render-target readback failed");
                return false;
            }
            const bool bPreview = Request.Purpose == EBskCapturePurpose::Preview;
            // PNG compression on the game thread can take long enough to block
            // the authoritative render receiver. RGB training frames do not
            // require lossless storage, so use high-quality JPEG for both the
            // preview and authoritative dataset paths. JPEG is lossy; decoding
            // produces an RGB tensor, not pixel-identical lossless RGB.
            const int32 JpegQuality = bPreview ? 75 : 95;
            FCapturedDataProduct Product{
                TEXT("rgb"),
                FString::Printf(TEXT("jpeg/bgra8_srgb/quality%d"), JpegQuality),
                TEXT("jpg")};
            const bool bCompressed = CompressJpeg(
                Pixels, Resolution.X, Resolution.Y, JpegQuality, Product.Bytes);
            if (!bCompressed)
            {
                OutError = TEXT("RGB image encoding failed");
                return false;
            }
            Products.Add(MoveTemp(Product));
        }
        else
        {
            OutError = TEXT("only RGB capture is supported");
            return false;
        }
    }

    const int64 Sequence = ++CaptureSequence;
    const int64 CaptureWallTimeNanoseconds = UnixTimeNanoseconds();
    TSharedPtr<FJsonObject> Metadata = MakeShared<FJsonObject>();
    Metadata->SetStringField(TEXT("protocol"), TEXT("bsk-capture/1"));
    Metadata->SetStringField(TEXT("type"), TEXT("camera_frame"));
    Metadata->SetStringField(TEXT("session_id"), ActiveSessionId);
    Metadata->SetStringField(TEXT("camera_id"), Request.CameraId);
    Metadata->SetStringField(TEXT("capture_episode_id"), Request.CaptureEpisodeId);
    Metadata->SetBoolField(TEXT("ack_required"), Request.Purpose == EBskCapturePurpose::AuthoritativeDataset);
    Metadata->SetStringField(TEXT("stream_kind"),
        Request.Purpose == EBskCapturePurpose::AuthoritativeDataset ? TEXT("authoritative") : TEXT("preview"));
    Metadata->SetStringField(TEXT("state_kind"),
        Request.Purpose == EBskCapturePurpose::AuthoritativeDataset ? TEXT("authoritative") : TEXT("presentation"));
    if (Request.Purpose == EBskCapturePurpose::AuthoritativeDataset)
    {
        const double DatasetRate = CaptureRateOverrideHertz > 0.0 ? CaptureRateOverrideHertz : Definition->CaptureRateHertz;
        int64 DatasetSampleIndex = 0;
        if (!BskLeRobotSample(Request.SimulationTimeNanoseconds, DatasetRate, DatasetSampleIndex))
        {
            OutError = TEXT("authoritative capture is outside the LeRobot sampling grid");
            return false;
        }
        Metadata->SetStringField(TEXT("dataset_format"), TEXT("lerobot-v3"));
        Metadata->SetNumberField(TEXT("sampling_fps"), DatasetRate);
        Metadata->SetStringField(TEXT("sample_index"), LexToString(DatasetSampleIndex));
        Metadata->SetStringField(TEXT("sampling_clock"), TEXT("simulation"));
    }
    Metadata->SetStringField(TEXT("capture_sequence"), LexToString(Sequence));
    Metadata->SetStringField(TEXT("source_frame_id"), LexToString(Request.FrameId));
    Metadata->SetStringField(TEXT("sim_time_ns"), LexToString(Request.SimulationTimeNanoseconds));
    Metadata->SetStringField(TEXT("source_wall_time_ns"), LexToString(Request.SourceWallTimeNanoseconds));
    Metadata->SetStringField(TEXT("capture_wall_time_ns"), LexToString(CaptureWallTimeNanoseconds));
    Metadata->SetArrayField(TEXT("resolution"), {
        MakeShared<FJsonValueNumber>(Resolution.X), MakeShared<FJsonValueNumber>(Resolution.Y)});

    const double FovX = Definition->FieldOfViewRadians;
    const double Fx = 0.5 * Resolution.X / FMath::Tan(0.5 * FovX);
    const double Fy = Fx;
    const double FovY = 2.0 * FMath::Atan(0.5 * Resolution.Y / Fy);
    TSharedPtr<FJsonObject> Intrinsics = MakeShared<FJsonObject>();
    Intrinsics->SetStringField(TEXT("model"), TEXT("pinhole"));
    Intrinsics->SetStringField(TEXT("distortion_model"), TEXT("none"));
    Intrinsics->SetNumberField(TEXT("fx_px"), Fx);
    Intrinsics->SetNumberField(TEXT("fy_px"), Fy);
    Intrinsics->SetNumberField(TEXT("cx_px"), 0.5 * (Resolution.X - 1));
    Intrinsics->SetNumberField(TEXT("cy_px"), 0.5 * (Resolution.Y - 1));
    Intrinsics->SetNumberField(TEXT("fov_x_rad"), FovX);
    Intrinsics->SetNumberField(TEXT("fov_y_rad"), FovY);
    Metadata->SetObjectField(TEXT("intrinsics"), Intrinsics);

    const FVector3d UnrealPositionCentimeters(CameraActor->GetActorLocation());
    const double MirrorY = Converter.MirrorsLocalY() ? -1.0 : 1.0;
    const FVector3d PositionLocalMeters(
        UnrealPositionCentimeters.X / Converter.GetCentimetersPerMeter(),
        MirrorY * UnrealPositionCentimeters.Y / Converter.GetCentimetersPerMeter(),
        UnrealPositionCentimeters.Z / Converter.GetCentimetersPerMeter());
    const FQuat4d LocalFromCamera = UnrealToActiveLocalWxyz(FQuat4d(CameraActor->GetActorQuat()), Converter.MirrorsLocalY());
    double LocalFromCameraMatrix[3][3];
    QuaternionMatrix(LocalFromCamera, LocalFromCameraMatrix);
    double InertialFromCameraMatrix[3][3];
    for (int32 Row = 0; Row < 3; ++Row) for (int32 Column = 0; Column < 3; ++Column)
    {
        InertialFromCameraMatrix[Row][Column] = LocalFromCameraMatrix[Row][Column];
    }
    FVector3d PositionInertialMeters = PositionLocalMeters;
    const FVector3d OriginInertialMeters = Request.OriginInertialMeters;
    double LocalFromInertialMatrix[3][3];
    for (int32 Row = 0; Row < 3; ++Row) for (int32 Column = 0; Column < 3; ++Column)
    {
        LocalFromInertialMatrix[Row][Column] = Request.LocalFromInertial.M[Row][Column];
    }
    for (int32 Row = 0; Row < 3; ++Row)
    {
        PositionInertialMeters[Row] = OriginInertialMeters[Row];
        for (int32 K = 0; K < 3; ++K) PositionInertialMeters[Row] += LocalFromInertialMatrix[K][Row] * PositionLocalMeters[K];
        for (int32 Column = 0; Column < 3; ++Column)
        {
            InertialFromCameraMatrix[Row][Column] = 0.0;
            for (int32 K = 0; K < 3; ++K) InertialFromCameraMatrix[Row][Column] += LocalFromInertialMatrix[K][Row] * LocalFromCameraMatrix[K][Column];
        }
    }
    TSharedPtr<FJsonObject> Extrinsics = MakeShared<FJsonObject>();
    Extrinsics->SetStringField(TEXT("camera_axes"), TEXT("+X_forward,+Y_left,+Z_up"));
    Extrinsics->SetArrayField(TEXT("position_L_m"), JsonVector(PositionLocalMeters));
    Extrinsics->SetArrayField(TEXT("q_LC_wxyz"), JsonQuaternionWxyz(LocalFromCamera));
    Extrinsics->SetArrayField(TEXT("c_LC"), JsonMatrix3(LocalFromCameraMatrix));
    Extrinsics->SetArrayField(TEXT("position_N_m"), JsonVector(PositionInertialMeters));
    Extrinsics->SetArrayField(TEXT("c_NC"), JsonMatrix3(InertialFromCameraMatrix));
    Metadata->SetObjectField(TEXT("extrinsics"), Extrinsics);
    TSharedPtr<FJsonObject> FloatingOrigin = MakeShared<FJsonObject>();
    FloatingOrigin->SetArrayField(TEXT("origin_N_m"), JsonVector(OriginInertialMeters));
    FloatingOrigin->SetArrayField(TEXT("c_LN"), JsonMatrix3(LocalFromInertialMatrix));
    Metadata->SetObjectField(TEXT("floating_origin"), FloatingOrigin);

    if (AsyncJob)
    {
        AsyncJob->Sequence = Sequence;
        AsyncJob->Metadata = Metadata; // Frozen BEFORE presentation is restored.
        PendingCaptureBytes += AsyncJob->ReservedBytes;
        PendingCaptureJobs.Add(MoveTemp(AsyncJob));
        return true;
    }
    return WriteCaptureProducts(Request, Metadata, Products, Sequence, OutError);
}

bool ABskSceneController::WriteCaptureProducts(const FBskCaptureRequest& Request,
    const TSharedPtr<FJsonObject>& Metadata, const TArray<FCapturedDataProduct>& Products,
    int64 Sequence, FString& OutError)
{
    Metadata->SetNumberField(TEXT("render_queue_frames"), Receiver ? Receiver->GetPendingFrameCount() : 0);
    Metadata->SetNumberField(TEXT("image_send_queue_packets"), CaptureNetworkSender ? CaptureNetworkSender->GetPendingPacketCount() : 0);

    TArray<TSharedPtr<FJsonValue>> ProductMetadata;
    uint64 BlobOffset = 0;
    const FString Stem = FString::Printf(TEXT("%012lld_%s"), Sequence, *SafePathSegment(Request.CameraId));
    for (const FCapturedDataProduct& Product : Products)
    {
        TSharedPtr<FJsonObject> ProductObject = MakeShared<FJsonObject>();
        ProductObject->SetStringField(TEXT("name"), Product.Name);
        ProductObject->SetStringField(TEXT("encoding"), Product.Encoding);
        ProductObject->SetStringField(TEXT("file_name"), FString::Printf(TEXT("%s_%s.%s"), *Stem, *Product.Name, *Product.Extension));
        ProductObject->SetStringField(TEXT("blob_offset"), LexToString(BlobOffset));
        ProductObject->SetStringField(TEXT("byte_length"), LexToString(Product.Bytes.Num()));
        ProductMetadata.Add(MakeShared<FJsonValueObject>(ProductObject));
        BlobOffset += Product.Bytes.Num();
    }
    Metadata->SetArrayField(TEXT("products"), ProductMetadata);
    FString MetadataJson;
    if (!FJsonSerializer::Serialize(Metadata.ToSharedRef(), TJsonWriterFactory<>::Create(&MetadataJson)))
    {
        OutError = TEXT("capture metadata JSON serialization failed");
        return false;
    }
    FTCHARToUTF8 MetadataUtf8(*MetadataJson);

    if (Request.bSendToNetwork)
    {
        if (!CaptureNetworkSender)
        {
            OutError = TEXT("capture network output was requested but no output target is configured");
            return false;
        }
        const uint64 PayloadSize = 4ull + MetadataUtf8.Length() + BlobOffset;
        if (PayloadSize > MAX_uint32)
        {
            OutError = TEXT("capture network packet exceeds the 4 GiB protocol limit");
            return false;
        }
        TArray<uint8> Packet;
        Packet.Reserve(static_cast<int32>(PayloadSize + 4));
        AddBigEndianUint32(Packet, static_cast<uint32>(PayloadSize));
        AddBigEndianUint32(Packet, static_cast<uint32>(MetadataUtf8.Length()));
        Packet.Append(reinterpret_cast<const uint8*>(MetadataUtf8.Get()), MetadataUtf8.Length());
        for (const FCapturedDataProduct& Product : Products) Packet.Append(Product.Bytes);
        if (Request.Purpose == EBskCapturePurpose::AuthoritativeDataset)
        {
            if (!CaptureNetworkSender->EnqueueReliable(MoveTemp(Packet)))
            {
                OutError = TEXT("authoritative capture admission failed; output capacity must be reserved before consuming the source frame");
                return false;
            }
        }
        else
        {
            CaptureNetworkSender->EnqueueLatest(FString::Printf(TEXT("preview:%s"), *Request.CameraId), MoveTemp(Packet));
        }
    }

    if (Request.bWriteToDisk)
    {
        FString Directory = Request.OutputDirectory.IsEmpty() ? CaptureOutputDirectory : Request.OutputDirectory;
        if (Directory.IsEmpty())
        {
            OutError = TEXT("capture disk output was requested without an output directory");
            return false;
        }
        Directory = FPaths::Combine(Directory, SafePathSegment(Metadata->GetStringField(TEXT("session_id"))), SafePathSegment(Request.CameraId));
        if (!CaptureDiskWriter)
        {
            OutError = TEXT("capture disk output was requested but no disk writer is configured");
            return false;
        }
        FBskCaptureDiskWork Work;
        Work.Directory = MoveTemp(Directory);
        Work.Stem = Stem;
        Work.MetadataJson = MetadataJson;
        Work.Products = Products;
        if (!CaptureDiskWriter->EnqueueReliable(MoveTemp(Work)))
        {
            OutError = TEXT("authoritative capture disk queue is full; frame was not silently replaced");
            return false;
        }
    }
    return true;
}

void ABskSceneController::PollCaptureJobs()
{
    check(IsInGameThread());
    for (const auto& Job : PendingCaptureJobs)
    {
        if (!Job->Readback || !Job->bCopySubmitted.Load() || !Job->Readback->IsReady()) continue;
        int32 Pitch = 0, Height = 0;
        const FColor* Source = static_cast<const FColor*>(Job->Readback->Lock(Pitch, &Height));
        TArray<FColor> Pixels;
        if (Source && Pitch >= Job->Resolution.X && Height >= Job->Resolution.Y)
        {
            Pixels.SetNumUninitialized(Job->Resolution.X * Job->Resolution.Y);
            for (int32 Y = 0; Y < Job->Resolution.Y; ++Y)
                FMemory::Memcpy(Pixels.GetData() + Y * Job->Resolution.X, Source + Y * Pitch,
                                Job->Resolution.X * sizeof(FColor));
            if (Job->bRgba) for (FColor& Pixel : Pixels) Swap(Pixel.R, Pixel.B);
        }
        if (Source) Job->Readback->Unlock();
        Job->Readback.Reset();
        Job->Metadata->SetStringField(TEXT("readback_completed_wall_time_ns"), LexToString(UnixTimeNanoseconds()));
        if (Pixels.IsEmpty())
        {
            Job->Error = TEXT("GPU RGB readback returned invalid pitch/height/data");
            continue;
        }
        // Independent image wrapper and owned pixels only; no scene/UObject
        // access in the worker. Bound parallel work by MaxAsyncCaptureJobs.
        Job->Encoded = Async(EAsyncExecution::ThreadPool,
            [Pixels = MoveTemp(Pixels), Wrapper = MoveTemp(Job->Wrapper), Size = Job->Resolution]() mutable
            {
                FCapturedDataProduct Product{TEXT("rgb"), TEXT("jpeg/bgra8_srgb/quality95"), TEXT("jpg")};
                if (Wrapper && Wrapper->SetRaw(Pixels.GetData(), Pixels.Num() * sizeof(FColor),
                                               Size.X, Size.Y, ERGBFormat::BGRA, 8))
                {
                    const TArray64<uint8>& Bytes = Wrapper->GetCompressed(95);
                    if (Bytes.Num() <= MAX_int32) Product.Bytes.Append(Bytes.GetData(), static_cast<int32>(Bytes.Num()));
                }
                return Product;
            });
    }
    // Workers may finish out of order. Commit in submission order so the
    // backend's per-camera sequences and frame-pairing semantics are unchanged.
    while (!PendingCaptureJobs.IsEmpty())
    {
        const auto Job = PendingCaptureJobs[0];
        if (Job->Readback) break;
        if (Job->Encoded.IsValid())
        {
            if (!Job->Encoded.IsReady()) break;
            FCapturedDataProduct Product = Job->Encoded.Get();
            Job->Encoded = TFuture<FCapturedDataProduct>();
            if (Product.Bytes.IsEmpty()) Job->Error = TEXT("background JPEG encoding failed");
            else Job->Products.Add(MoveTemp(Product));
        }
        uint64 Bytes = 65536;
        for (const auto& Product : Job->Products) Bytes += Product.Bytes.Num();
        if ((Job->Request.bSendToNetwork && CaptureNetworkSender
                && !CaptureNetworkSender->HasReliableCapacity(1, Bytes))
            || (Job->Request.bWriteToDisk && CaptureDiskWriter && !CaptureDiskWriter->HasCapacity(1))) break;
        Job->Metadata->SetStringField(TEXT("capture_wall_time_ns"), LexToString(UnixTimeNanoseconds()));
        Job->Metadata->SetNumberField(TEXT("capture_pipeline_ms"), (FPlatformTime::Seconds() - Job->SubmittedSeconds) * 1000.0);
        Job->Metadata->SetNumberField(TEXT("capture_pending_jobs"), PendingCaptureJobs.Num());
        if (!Job->Error.IsEmpty())
        {
            Job->Metadata->SetStringField(TEXT("type"), TEXT("capture_error"));
            Job->Metadata->SetStringField(TEXT("error"), Job->Error);
        }
        FString Error;
        if (!WriteCaptureProducts(Job->Request, Job->Metadata, Job->Products, Job->Sequence, Error))
        {
            UE_LOG(LogBskUnreal, Error, TEXT("Async capture frame=%lld failed: %s"), Job->Request.FrameId, *Error);
            ReportCaptureFailure(Job->Request, Error);
        }
        PendingCaptureBytes -= Job->ReservedBytes;
        PendingCaptureJobs.RemoveAt(0, 1, EAllowShrinking::No);
    }
}

void ABskSceneController::CancelCaptureJobs()
{
    // Only session reset/renderer teardown cancels jobs, never normal STOP.
    // Wait for queued RHI commands before releasing their resources.
    if (!PendingCaptureJobs.IsEmpty()) FlushRenderingCommands();
    for (const auto& Job : PendingCaptureJobs)
        if (Job->Encoded.IsValid()) Job->Encoded.Wait();
    PendingCaptureJobs.Reset();
    PendingCaptureBytes = 0;
}

void ABskSceneController::ReportCaptureFailure(const FBskCaptureRequest& Request, const FString& Error)
{
    if (!Request.bSendToNetwork || !CaptureNetworkSender) return;
    auto Metadata = MakeShared<FJsonObject>();
    const int64 Sequence = ++CaptureSequence;
    Metadata->SetStringField(TEXT("protocol"), TEXT("bsk-capture/1"));
    Metadata->SetStringField(TEXT("type"), TEXT("capture_error"));
    Metadata->SetStringField(TEXT("session_id"), ActiveSessionId);
    Metadata->SetStringField(TEXT("capture_episode_id"), Request.CaptureEpisodeId);
    Metadata->SetStringField(TEXT("camera_id"), Request.CameraId);
    Metadata->SetStringField(TEXT("capture_sequence"), LexToString(Sequence));
    Metadata->SetStringField(TEXT("source_frame_id"), LexToString(Request.FrameId));
    Metadata->SetStringField(TEXT("sim_time_ns"), LexToString(Request.SimulationTimeNanoseconds));
    Metadata->SetStringField(TEXT("stream_kind"), TEXT("authoritative"));
    Metadata->SetStringField(TEXT("state_kind"), TEXT("authoritative"));
    Metadata->SetBoolField(TEXT("ack_required"), true);
    Metadata->SetStringField(TEXT("error"), Error);
    FString OutputError;
    WriteCaptureProducts(Request, Metadata, {}, Sequence, OutputError);
}

void ABskSceneController::AttachManifestChildren()
{
    for (const TPair<FString, FBskObjectDefinition>& Pair : ManifestObjects)
    {
        AActor* Actor = BoundActors.FindRef(Pair.Key);
        AActor* Parent = Pair.Value.ParentId.IsEmpty() ? nullptr : BoundActors.FindRef(Pair.Value.ParentId);
        if (Actor && Parent && Actor->GetAttachParentActor() != Parent) Actor->AttachToActor(Parent, FAttachmentTransformRules::KeepWorldTransform);
    }
    for (const TPair<FString, FBskVisualDefinition>& Pair : ManifestVisuals)
    {
        AActor* Actor = VisualActors.FindRef(Pair.Key);
        AActor* Parent = BoundActors.FindRef(Pair.Value.ParentId);
        if (Actor && Parent) Actor->AttachToActor(Parent, FAttachmentTransformRules::KeepRelativeTransform);
    }
    for (const TPair<FString, FBskCameraDefinition>& Pair : ManifestCameras)
    {
        AActor* Actor = CameraActors.FindRef(Pair.Key);
        AActor* Parent = Pair.Value.ParentId.IsEmpty() ? nullptr : BoundActors.FindRef(Pair.Value.ParentId);
        if (Actor && Parent) Actor->AttachToActor(Parent, FAttachmentTransformRules::KeepRelativeTransform);
    }
}

void ABskSceneController::ApplyFrame(const FBskRenderFrame& Frame, bool bPresentationFrame)
{
    check(IsInGameThread());
    if (LastFrameId < 0)
    {
        UE_LOG(LogBskUnreal, Display, TEXT("Applying first BSK frame %lld with %d objects on the Game Thread"), Frame.FrameId, Frame.Objects.Num());
    }
    for (const FBskRenderObjectState& State : Frame.Objects)
    {
        GetOrCreateActor(State);
    }
    for (const FBskRenderObjectState& State : Frame.Objects)
    {
        const FString& Key = State.ObjectId.IsEmpty() ? State.Name : State.ObjectId;
        AActor* Actor = BoundActors.FindRef(Key);
        if (!Actor) continue;
        FString ParentId = State.Parent;
        if (const FBskObjectDefinition* Definition = ManifestObjects.Find(Key)) ParentId = Definition->ParentId;
        AActor* DesiredParent = ParentId.IsEmpty() ? nullptr : BoundActors.FindRef(ParentId);
        if (DesiredParent && Actor->GetAttachParentActor() != DesiredParent)
        {
            Actor->AttachToActor(DesiredParent, FAttachmentTransformRules::KeepWorldTransform);
        }
        else if (!DesiredParent && Actor->GetAttachParentActor())
        {
            Actor->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
        }
        const FVector Location(Converter.LocalMetersToUnrealCentimeters(State.PositionMeters));
        const FQuat Rotation(Converter.ActiveLocalWxyzToUnreal(State.OrientationWxyz));
        Actor->SetActorLocationAndRotation(Location, Rotation, false, nullptr, ETeleportType::TeleportPhysics);
    }
    UpdateCelestialBodies(Frame);
    UpdateVisualStates(Frame);
    UpdateLightTargets();
    DrawOrbitLines(Frame);
    if (!bPresentationFrame) return;
    PresentationFrame = Frame;
    bHasPresentationFrame = true;
    LastFrameId = Frame.FrameId;
    FrameAppliedEvent.Broadcast(Frame);
    if (UBskRenderWorldSubsystem* RenderSubsystem = GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>())
    {
        RenderSubsystem->NotifyFrameApplied(Frame);
    }

    if (!bScreenshotRequested)
    {
        FString ScreenshotPath;
        if (FParse::Value(FCommandLine::Get(), TEXT("BskScreenshot="), ScreenshotPath) && !ScreenshotPath.IsEmpty())
        {
            if (FPaths::IsRelative(ScreenshotPath))
            {
                ScreenshotPath = FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir(), ScreenshotPath);
            }
            bScreenshotRequested = true;
            FTimerHandle ScreenshotTimer;
            GetWorldTimerManager().SetTimer(
                ScreenshotTimer,
                [ScreenshotPath]()
                {
                    FScreenshotRequest::RequestScreenshot(ScreenshotPath, false, false);
                    UE_LOG(LogBskUnreal, Display, TEXT("Requested delayed BSK validation screenshot: %s"), *ScreenshotPath);
                },
                1.5f,
                false);
        }
    }
}

FBskRenderFrame ABskSceneController::InterpolateFrame(const FBskRenderFrame& From, const FBskRenderFrame& To, double Alpha) const
{
    FBskRenderFrame Result = To;
    Result.SimulationTimeNanoseconds = FMath::RoundToInt64(FMath::Lerp(
        static_cast<double>(From.SimulationTimeNanoseconds), static_cast<double>(To.SimulationTimeNanoseconds), Alpha));
    Result.OriginInertialMeters = FMath::Lerp(From.OriginInertialMeters, To.OriginInertialMeters, Alpha);
    TMap<FString, const FBskRenderObjectState*> FromObjects;
    for (const FBskRenderObjectState& State : From.Objects)
    {
        FromObjects.Add(State.ObjectId.IsEmpty() ? State.Name : State.ObjectId, &State);
    }
    for (FBskRenderObjectState& State : Result.Objects)
    {
        const FString& Key = State.ObjectId.IsEmpty() ? State.Name : State.ObjectId;
        if (const FBskRenderObjectState* const* Prior = FromObjects.Find(Key))
        {
            State.PositionMeters = FMath::Lerp((*Prior)->PositionMeters, State.PositionMeters, Alpha);
            State.OrientationWxyz = FQuat4d::Slerp((*Prior)->OrientationWxyz, State.OrientationWxyz, Alpha).GetNormalized();
            if ((*Prior)->bHasVelocity && State.bHasVelocity)
            {
                State.VelocityMetersPerSecond = FMath::Lerp((*Prior)->VelocityMetersPerSecond, State.VelocityMetersPerSecond, Alpha);
            }
        }
    }
    TMap<FString, const FBskCelestialBodyState*> FromCelestial;
    for (const FBskCelestialBodyState& State : From.CelestialBodies) FromCelestial.Add(State.BodyId, &State);
    for (FBskCelestialBodyState& State : Result.CelestialBodies)
    {
        if (const FBskCelestialBodyState* const* Prior = FromCelestial.Find(State.BodyId))
        {
            State.PositionMeters = FMath::Lerp((*Prior)->PositionMeters, State.PositionMeters, Alpha);
            State.OrientationWxyz = FQuat4d::Slerp((*Prior)->OrientationWxyz, State.OrientationWxyz, Alpha).GetNormalized();
        }
    }
    TMap<FString, const FBskVisualState*> FromVisuals;
    for (const FBskVisualState& State : From.VisualStates) FromVisuals.Add(State.VisualId, &State);
    for (FBskVisualState& State : Result.VisualStates)
    {
        const FBskVisualState* const* Prior = FromVisuals.Find(State.VisualId);
        if (!Prior) continue;
        State.Value = FMath::Lerp((*Prior)->Value, State.Value, Alpha);
        for (TPair<FString, FBskVisualState::FChannelValue>& Pair : State.Channels)
        {
            const FBskVisualState::FChannelValue* PriorChannel = (*Prior)->Channels.Find(Pair.Key);
            if (PriorChannel && PriorChannel->Type == FBskVisualState::FChannelValue::EType::Number &&
                Pair.Value.Type == FBskVisualState::FChannelValue::EType::Number)
            {
                Pair.Value.Number = FMath::Lerp(PriorChannel->Number, Pair.Value.Number, Alpha);
            }
        }
    }
    return Result;
}

FBskRenderFrame ABskSceneController::ExtrapolateFrame(
    const FBskRenderFrame& From,
    const FBskRenderFrame& To,
    double SecondsBeyondTarget) const
{
    FBskRenderFrame Result = To;
    const double SourceDeltaSeconds = static_cast<double>(To.SimulationTimeNanoseconds - From.SimulationTimeNanoseconds) * 1.0e-9;
    const double ExtraSeconds = FMath::Clamp(SecondsBeyondTarget, 0.0, MaxExtrapolationSeconds);
    if (SourceDeltaSeconds <= UE_DOUBLE_SMALL_NUMBER || ExtraSeconds <= 0.0) return Result;
    Result.SimulationTimeNanoseconds += FMath::RoundToInt64(ExtraSeconds * 1.0e9);
    Result.OriginInertialMeters += (To.OriginInertialMeters - From.OriginInertialMeters) * (ExtraSeconds / SourceDeltaSeconds);

    const auto ExtrapolateOrientation = [SourceDeltaSeconds, ExtraSeconds](const FQuat4d& A, const FQuat4d& B)
    {
        FQuat4d Delta = (B * A.Inverse()).GetNormalized();
        if (Delta.W < 0.0) Delta = FQuat4d(-Delta.X, -Delta.Y, -Delta.Z, -Delta.W);
        const double HalfAngle = FMath::Acos(FMath::Clamp(Delta.W, -1.0, 1.0));
        const double SinHalf = FMath::Sin(HalfAngle);
        if (FMath::Abs(SinHalf) <= UE_DOUBLE_SMALL_NUMBER) return B;
        const FVector3d Axis(Delta.X / SinHalf, Delta.Y / SinHalf, Delta.Z / SinHalf);
        const FQuat4d ExtraRotation(Axis, 2.0 * HalfAngle * ExtraSeconds / SourceDeltaSeconds);
        return (ExtraRotation * B).GetNormalized();
    };

    TMap<FString, const FBskRenderObjectState*> FromObjects;
    for (const FBskRenderObjectState& State : From.Objects)
    {
        FromObjects.Add(State.ObjectId.IsEmpty() ? State.Name : State.ObjectId, &State);
    }
    for (FBskRenderObjectState& State : Result.Objects)
    {
        const FString& Key = State.ObjectId.IsEmpty() ? State.Name : State.ObjectId;
        const FBskRenderObjectState* const* Prior = FromObjects.Find(Key);
        if (!Prior) continue;
        // Positions are relative to the moving render origin; wire velocities
        // are inertial velocities expressed in the local axes (also used for
        // orbital telemetry). Integrating those into local positions adds the
        // spacecraft's ~7.6 km/s orbital motion a second time, then snaps back
        // on every received frame. Differentiate the LOCAL positions instead.
        const FVector3d Velocity = (State.PositionMeters - (*Prior)->PositionMeters) / SourceDeltaSeconds;
        State.PositionMeters += Velocity * ExtraSeconds;
        State.OrientationWxyz = ExtrapolateOrientation((*Prior)->OrientationWxyz, State.OrientationWxyz);
    }

    TMap<FString, const FBskCelestialBodyState*> FromCelestial;
    for (const FBskCelestialBodyState& State : From.CelestialBodies) FromCelestial.Add(State.BodyId, &State);
    for (FBskCelestialBodyState& State : Result.CelestialBodies)
    {
        const FBskCelestialBodyState* const* Prior = FromCelestial.Find(State.BodyId);
        if (!Prior) continue;
        // Celestial positions use the same moving origin as ordinary objects.
        // A nonzero inertial ephemeris velocity is not a local position derivative.
        const FVector3d Velocity = (State.PositionMeters - (*Prior)->PositionMeters) / SourceDeltaSeconds;
        State.PositionMeters += Velocity * ExtraSeconds;
        State.OrientationWxyz = ExtrapolateOrientation((*Prior)->OrientationWxyz, State.OrientationWxyz);
    }
    return Result;
}

double ABskSceneController::SolarVisibilityAt(const FVector3d& ReceiverMeters) const
{
    double Visibility = 1.0;
    for (const TPair<FVector3d, double>& Occluder : SolarOccluders)
    {
        Visibility = FMath::Min(Visibility, BskCelestialLighting::VisibleSourceFraction(
            CurrentSunPositionMeters, CurrentSunRadiusMeters, Occluder.Key, Occluder.Value, ReceiverMeters));
    }
    return Visibility;
}

void ABskSceneController::UpdateCelestialBodies(const FBskRenderFrame& Frame)
{
    // Read ALL occluders from this frame first, irrespective of message order.
    SolarOccluders.Reset();
    for (const FBskCelestialBodyState& State : Frame.CelestialBodies)
    {
        const FBskCelestialBodyDefinition* Definition = ManifestCelestialBodies.Find(State.BodyId);
        if (Definition && !Definition->bLuminous && Definition->EquatorialRadiusMeters > 0.0)
        {
            SolarOccluders.Emplace(State.PositionMeters, Definition->EquatorialRadiusMeters);
        }
    }
    for (const FBskCelestialBodyState& State : Frame.CelestialBodies)
    {
        AActor* Actor = CelestialActors.FindRef(State.BodyId);
        const FVector Location(Converter.LocalMetersToUnrealCentimeters(State.PositionMeters));
        if (Actor)
        {
            FQuat Rotation(Converter.ActiveLocalWxyzToUnreal(State.OrientationWxyz));
            if (CelestialBillboardIds.Contains(State.BodyId))
            {
                FVector ViewLocation = FVector::ZeroVector;
                FRotator ViewRotation;
                if (APlayerController* Player = GetWorld()->GetFirstPlayerController()) Player->GetPlayerViewPoint(ViewLocation, ViewRotation);
                const FVector DirectionToViewer = (ViewLocation - Location).GetSafeNormal();
                if (!DirectionToViewer.IsNearlyZero()) Rotation = DirectionToViewer.Rotation().Quaternion();
            }
            Actor->SetActorLocationAndRotation(Location, Rotation, false, nullptr, ETeleportType::TeleportPhysics);
        }
        if (EarthAtmosphere && State.BodyId.Equals(TEXT("earth"), ESearchCase::IgnoreCase))
        {
            EarthAtmosphere->SetActorLocation(Location, false, nullptr, ETeleportType::TeleportPhysics);
            EarthAtmosphere->SetActorHiddenInGame(false);
        }
        if (!bEphemerisEarthLogged && State.BodyId.Equals(TEXT("earth"), ESearchCase::IgnoreCase))
        {
            if (const FBskCelestialBodyDefinition* Definition = ManifestCelestialBodies.Find(State.BodyId))
            {
                bEphemerisEarthLogged = true;
                UE_LOG(LogBskUnreal, Display,
                    TEXT("Ephemeris Earth active center_distance_km=%.3f radius_km=%.3f local_origin_altitude_km=%.3f"),
                    State.PositionMeters.Length() / 1000.0, Definition->EquatorialRadiusMeters / 1000.0,
                    (State.PositionMeters.Length() - Definition->EquatorialRadiusMeters) / 1000.0);
            }
        }
        if (State.BodyId == PrimaryDirectionalLightBodyId)
        {
            const FBskCelestialBodyDefinition* Definition = ManifestCelestialBodies.Find(State.BodyId);
            const double SourceDistanceMeters = State.PositionMeters.Length();
            const FVector DirectionTowardSource = Location.GetSafeNormal();
            if (Definition && !DirectionTowardSource.IsNearlyZero())
            {
                CurrentSunSourceDirection = DirectionTowardSource;
                CurrentSunPositionMeters = State.PositionMeters;
                CurrentSunRadiusMeters = Definition->EquatorialRadiusMeters;
                if (DecorativeSunActor)
                {
                    DecorativeSunActor->SetActorRotation(FQuat(Converter.ActiveLocalWxyzToUnreal(State.OrientationWxyz)));
                }
                const double PreviousVisibility = CurrentSolarVisibility;
                CurrentSolarVisibility = SolarVisibilityAt(FVector3d::ZeroVector);
                if (!bEphemerisDirectionalLightActive ||
                    (PreviousVisibility <= 1.0e-6) != (CurrentSolarVisibility <= 1.0e-6))
                {
                    UE_LOG(LogBskUnreal, Display,
                        TEXT("Ephemeris solar visibility at local origin=%.6f occluders=%d"),
                        CurrentSolarVisibility, SolarOccluders.Num());
                }
                if (Definition->EquatorialRadiusMeters > 0.0 &&
                    SourceDistanceMeters > Definition->EquatorialRadiusMeters)
                {
                    const double AngularRadiusRadians = FMath::Asin(FMath::Clamp(
                        Definition->EquatorialRadiusMeters / SourceDistanceMeters, 0.0, 1.0));
                    const double AngularDiameterDegrees = FMath::RadiansToDegrees(2.0 * AngularRadiusRadians);
                    if (FMath::IsFinite(AngularDiameterDegrees) && AngularDiameterDegrees > 0.0)
                    {
                        CurrentSunAngularDiameterDegrees = FMath::Clamp(AngularDiameterDegrees, 0.001, 10.0);
                        UpdateDecorativeSunScale();
                    }
                }

                if (SunLight && !bUseManifestSceneLighting)
                {
                    const FVector LightRayDirection = BskCelestialLighting::DirectionFromSourceToTarget(Location);
                    if (!LightRayDirection.IsNearlyZero())
                    {
                        const bool bWasEphemerisActive = bEphemerisDirectionalLightActive;
                        SunLight->SetActorRotation(LightRayDirection.Rotation());
                        if (UDirectionalLightComponent* Directional = Cast<UDirectionalLightComponent>(SunLight->GetLightComponent()))
                        {
                            Directional->SetLightSourceAngle(static_cast<float>(CurrentSunAngularDiameterDegrees));
                        }
                        SunLight->GetLightComponent()->SetLightColor(FLinearColor(
                            static_cast<float>(Definition->LightColorRgb.X),
                            static_cast<float>(Definition->LightColorRgb.Y),
                            static_cast<float>(Definition->LightColorRgb.Z)));
                        const double BaseIlluminance = Definition->LightIlluminanceLuxAtReferenceDistance > 0.0
                            ? Definition->LightIlluminanceLuxAtReferenceDistance
                            : SunIntensityLux;
                        const double Illuminance = BskCelestialLighting::IlluminanceLux(
                            BaseIlluminance,
                            Definition->LightReferenceDistanceMeters,
                            SourceDistanceMeters);
                        const float UnoccludedLux = static_cast<float>(BskCelestialLighting::ScaleIlluminanceLux(
                            Illuminance * SunIlluminanceScale, SceneSunlightIntensityScale));
                        SunLight->GetLightComponent()->SetIntensity(
                            UnoccludedLux * static_cast<float>(CurrentSolarVisibility));
                        if (CelestialSunLight)
                        {
                            CelestialSunLight->SetActorRotation(LightRayDirection.Rotation());
                            CelestialSunLight->GetLightComponent()->SetLightColor(
                                SunLight->GetLightComponent()->GetLightColor());
                            CelestialSunLight->GetLightComponent()->SetIntensity(UnoccludedLux);
                        }
                        bEphemerisDirectionalLightActive = true;
                        if (!bWasEphemerisActive)
                        {
                            UE_LOG(LogBskUnreal, Display,
                                TEXT("Ephemeris Sun active distance=%.6f AU angular_diameter=%.4f deg direction=%s unoccluded_lux=%.4f local_lux=%.4f visibility=%.6f"),
                                SourceDistanceMeters / 149597870700.0,
                                CurrentSunAngularDiameterDegrees,
                                *CurrentSunSourceDirection.ToCompactString(),
                                static_cast<double>(UnoccludedLux),
                                static_cast<double>(UnoccludedLux) * CurrentSolarVisibility,
                                CurrentSolarVisibility);
                        }
                    }
                }
            }
        }
    }
}

void ABskSceneController::UpdateVisualStates(const FBskRenderFrame& Frame)
{
    for (const FBskVisualState& State : Frame.VisualStates)
    {
        if (AActor* Actor = VisualActors.FindRef(State.VisualId))
        {
            const FBskVisualDefinition* Definition = ManifestVisuals.Find(State.VisualId);
            if (!Definition) continue;
            auto NumberChannel = [&State](const TCHAR* Name, double Fallback)
            {
                if (const FBskVisualState::FChannelValue* Channel = State.Channels.Find(Name))
                {
                    if (Channel->Type == FBskVisualState::FChannelValue::EType::Number) return Channel->Number;
                    if (Channel->Type == FBskVisualState::FChannelValue::EType::Boolean) return Channel->Boolean ? 1.0 : 0.0;
                }
                return Fallback;
            };
            auto BoolChannel = [&State](const TCHAR* Name, bool Fallback)
            {
                if (const FBskVisualState::FChannelValue* Channel = State.Channels.Find(Name))
                {
                    if (Channel->Type == FBskVisualState::FChannelValue::EType::Boolean) return Channel->Boolean;
                    if (Channel->Type == FBskVisualState::FChannelValue::EType::Number) return !FMath::IsNearlyZero(Channel->Number);
                }
                return Fallback;
            };

            bool bVisible = State.bVisible;
            if (Definition->Kind == TEXT("reaction_wheel"))
            {
                const double Angle = NumberChannel(TEXT("angle_rad"), 0.0);
                const FQuat Base = VisualBaseRotations.FindRef(State.VisualId);
                Actor->SetActorRelativeRotation(Base * FQuat(FVector::UpVector, Angle));
                bVisible = BoolChannel(TEXT("enabled"), bVisible);
            }
            else if (Definition->Kind == TEXT("thruster"))
            {
                const double Throttle = FMath::Clamp(NumberChannel(TEXT("throttle"), State.Value), 0.0, 1.0);
                const FVector BaseScale = VisualBaseScales.FindRef(State.VisualId);
                Actor->SetActorRelativeScale3D(FVector(BaseScale.X, BaseScale.Y, BaseScale.Z * FMath::Max(Throttle, 0.02)));
                bVisible = BoolChannel(TEXT("enabled"), bVisible) && Throttle > 1.0e-6;
            }
            else if (Definition->Kind == TEXT("css"))
            {
                const double Signal = FMath::Clamp(NumberChannel(TEXT("normalized_signal"), State.Value), 0.0, 1.0);
                const FVector BaseScale = VisualBaseScales.FindRef(State.VisualId);
                Actor->SetActorRelativeScale3D(BaseScale * (0.85 + 0.15 * Signal));
                bVisible = BoolChannel(TEXT("enabled"), bVisible);
            }
            else if (Definition->Kind == TEXT("generic_sensor") || Definition->Kind == TEXT("light"))
            {
                bVisible = BoolChannel(TEXT("enabled"), bVisible);
            }
            VisualDynamicVisibility.Add(State.VisualId, bVisible);
            RefreshVisualVisibility(State.VisualId);

            const double Intensity = Definition->Kind == TEXT("thruster")
                ? FMath::Clamp(NumberChannel(TEXT("throttle"), State.Value), 0.0, 1.0)
                : 1.0;
            TArray<UStaticMeshComponent*> Components;
            Actor->GetComponents<UStaticMeshComponent>(Components);
            for (UStaticMeshComponent* Component : Components)
            {
                if (UMaterialInstanceDynamic* Material = Cast<UMaterialInstanceDynamic>(Component->GetMaterial(0)))
                {
                    Material->SetVectorParameterValue(TEXT("Color"), Definition->Color * static_cast<float>(0.35 + 0.65 * Intensity));
                }
            }
        }
    }
}

void ABskSceneController::UpdateLightTargets()
{
    for (const TPair<FString, FBskVisualDefinition>& Pair : ManifestVisuals)
    {
        const FBskVisualDefinition& Definition = Pair.Value;
        if (!Definition.Kind.Equals(TEXT("light"), ESearchCase::IgnoreCase) || Definition.LightTargetId.IsEmpty()) continue;
        AActor* LightActor = VisualActors.FindRef(Pair.Key);
        AActor* TargetActor = BoundActors.FindRef(Definition.LightTargetId);
        if (!LightActor || !TargetActor) continue;
        const FVector Direction = (TargetActor->GetActorLocation() - LightActor->GetActorLocation()).GetSafeNormal();
        if (!Direction.IsNearlyZero()) LightActor->SetActorRotation(Direction.Rotation());
    }
}

void ABskSceneController::DrawOrbitLines(const FBskRenderFrame& Frame) const
{
    if (!bShowOrbitLines) return;
    const FBskCelestialBodyState* CentralState = nullptr;
    const FBskCelestialBodyDefinition* CentralDefinition = nullptr;
    for (const FBskCelestialBodyState& State : Frame.CelestialBodies)
    {
        const FBskCelestialBodyDefinition* Definition = ManifestCelestialBodies.Find(State.BodyId);
        if (Definition && (State.BodyId.Equals(TEXT("earth"), ESearchCase::IgnoreCase) ||
            Definition->DisplayName.Equals(TEXT("earth"), ESearchCase::IgnoreCase)))
        {
            CentralState = &State;
            CentralDefinition = Definition;
            break;
        }
    }
    if (!CentralState || !CentralDefinition || CentralDefinition->MuMetersCubedPerSecondSquared <= 0.0) return;
    constexpr int32 SegmentCount = 128;
    for (const FBskRenderObjectState& State : Frame.Objects)
    {
        const FString& Key = State.ObjectId.IsEmpty() ? State.Name : State.ObjectId;
        const FBskObjectDefinition* Definition = ManifestObjects.Find(Key);
        if (!State.bHasVelocity || (Definition && !Definition->ParentId.IsEmpty())) continue;
        const FVector3d R = State.PositionMeters - CentralState->PositionMeters;
        const FVector3d V = State.VelocityMetersPerSecond - CentralState->VelocityMetersPerSecond;
        const double Radius = R.Length();
        const FVector3d H = FVector3d::CrossProduct(R, V);
        const double HSquared = H.SquaredLength();
        if (Radius <= 1.0 || HSquared <= UE_DOUBLE_SMALL_NUMBER) continue;
        const double Mu = CentralDefinition->MuMetersCubedPerSecondSquared;
        const FVector3d EccentricityVector = FVector3d::CrossProduct(V, H) / Mu - R / Radius;
        const double Eccentricity = EccentricityVector.Length();
        const FVector3d P = Eccentricity > 1.0e-8 ? EccentricityVector / Eccentricity : R / Radius;
        const FVector3d Q = FVector3d::CrossProduct(H.GetSafeNormal(), P).GetSafeNormal();
        const double SemiLatusRectum = HSquared / Mu;
        FVector Previous;
        bool bHavePrevious = false;
        const uint32 Hash = GetTypeHash(Key);
        const FColor Color = FLinearColor::MakeFromHSV8(static_cast<uint8>(Hash & 0xff), 180, 255).ToFColor(true);
        for (int32 Index = 0; Index <= SegmentCount; ++Index)
        {
            const double Anomaly = 2.0 * PI * static_cast<double>(Index) / static_cast<double>(SegmentCount);
            const double Denominator = 1.0 + Eccentricity * FMath::Cos(Anomaly);
            if (Denominator <= 1.0e-6)
            {
                bHavePrevious = false;
                continue;
            }
            const double OrbitRadius = SemiLatusRectum / Denominator;
            const FVector3d LocalMeters = CentralState->PositionMeters + OrbitRadius * (P * FMath::Cos(Anomaly) + Q * FMath::Sin(Anomaly));
            const FVector Current(Converter.LocalMetersToUnrealCentimeters(LocalMeters));
            if (bHavePrevious) DrawDebugLine(GetWorld(), Previous, Current, Color, false, 0.0f, 0, 0.0f);
            Previous = Current;
            bHavePrevious = true;
        }
    }
}

bool ABskSceneController::CreateTexturedStarSphere()
{
    if (!bUseTexturedStarSphere) return false;
    UStaticMesh* SphereMesh = LoadObject<UStaticMesh>(nullptr, *TexturedStarMeshPath);
    UMaterialInterface* StarMaterial = LoadObject<UMaterialInterface>(nullptr, *TexturedStarMaterialPath);
    if (!SphereMesh || !StarMaterial)
    {
        UE_LOG(LogBskUnreal, Warning,
            TEXT("Textured star-sphere assets unavailable mesh=%s material=%s; falling back to Celestial Vault"),
            SphereMesh ? TEXT("ok") : TEXT("missing"), StarMaterial ? TEXT("ok") : TEXT("missing"));
        return false;
    }

    const FBoxSphereBounds MeshBounds = SphereMesh->GetBounds();
    const double SourceRadiusCentimeters = FMath::Max(
        static_cast<double>(MeshBounds.BoxExtent.GetMax()), UE_DOUBLE_SMALL_NUMBER);
    const double TargetRadiusCentimeters = CelestialVaultRadiusKilometers * 100000.0;
    const double MeshScale = TargetRadiusCentimeters / SourceRadiusCentimeters;

    DeepSkyComponent = NewObject<UStaticMeshComponent>(this, TEXT("TexturedStarSphereBackground"));
    DeepSkyComponent->SetupAttachment(GetRootComponent());
    DeepSkyComponent->SetMobility(EComponentMobility::Movable);
    DeepSkyComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    DeepSkyComponent->SetCastShadow(true);
    DeepSkyComponent->SetAffectDistanceFieldLighting(true);
    DeepSkyComponent->SetAffectDynamicIndirectLighting(true);
    DeepSkyComponent->SetReceivesDecals(true);
    DeepSkyComponent->SetRenderInMainPass(true);
    DeepSkyComponent->SetRenderInDepthPass(true);
    DeepSkyComponent->SetTranslucentSortPriority(0);
    DeepSkyComponent->SetCanEverAffectNavigation(false);
    DeepSkyComponent->SetStaticMesh(SphereMesh);
    DeepSkyComponent->SetMaterial(0, StarMaterial);
    DeepSkyComponent->SetRelativeScale3D(FVector(MeshScale));
    DeepSkyComponent->SetRelativeLocation(-MeshBounds.Origin * MeshScale);
    DeepSkyComponent->RegisterComponent();
    UE_LOG(LogBskUnreal, Display,
        TEXT("Using MyProject2 star sphere mesh=%s material=%s at %.0f km radius"),
        *TexturedStarMeshPath, *TexturedStarMaterialPath, CelestialVaultRadiusKilometers);
    return true;
}

void ABskSceneController::CreateDecorativeSun()
{
    if (!bEnableDecorativeSun) return;
    UStaticMesh* SphereMesh = LoadObject<UStaticMesh>(nullptr, *SunVisualMeshPath);
    UMaterialInterface* SunMaterial = LoadObject<UMaterialInterface>(nullptr, *SunVisualMaterialPath);
    if (!SphereMesh || !SunMaterial)
    {
        UE_LOG(LogBskUnreal, Warning,
            TEXT("Solar System Scope Sun assets unavailable mesh=%s material=%s"),
            SphereMesh ? TEXT("ok") : TEXT("missing"), SunMaterial ? TEXT("ok") : TEXT("missing"));
        return;
    }

    const double MaximumDistanceMeters = FMath::Max(1000.0, CelestialVaultRadiusKilometers * 1000.0 * 0.92);
    const double EffectiveDistanceMeters = FMath::Min(SunVisualDistanceMeters, MaximumDistanceMeters);
    const FBoxSphereBounds MeshBounds = SphereMesh->GetBounds();
    DecorativeSunSourceRadiusCentimeters = FMath::Max(
        static_cast<double>(MeshBounds.BoxExtent.GetMax()), UE_DOUBLE_SMALL_NUMBER);
    DecorativeSunSourceCenterCentimeters = FVector(MeshBounds.Origin);
    CurrentSunAngularDiameterDegrees = SunVisualAngularDiameterDegrees;

    FActorSpawnParameters Params;
    Params.Name = MakeUniqueObjectName(GetWorld(), AActor::StaticClass(), TEXT("BSK_VisibleSun"));
    Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    DecorativeSunActor = GetWorld()->SpawnActor<AActor>(AActor::StaticClass(), FTransform::Identity, Params);
    if (!DecorativeSunActor) return;

    USceneComponent* Root = NewObject<USceneComponent>(DecorativeSunActor, TEXT("SunVisualRoot"));
    Root->SetMobility(EComponentMobility::Movable);
    Root->RegisterComponent();
    DecorativeSunActor->SetRootComponent(Root);

    UStaticMeshComponent* Component = NewObject<UStaticMeshComponent>(DecorativeSunActor, TEXT("SunSurface"));
    Component->SetMobility(EComponentMobility::Movable);
    Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Component->SetCastShadow(false);
    Component->SetAffectDistanceFieldLighting(false);
    Component->SetAffectDynamicIndirectLighting(false);
    Component->SetReceivesDecals(false);
    Component->SetRenderInMainPass(true);
    Component->SetRenderInDepthPass(true);
    Component->SetCanEverAffectNavigation(false);
    Component->SetStaticMesh(SphereMesh);
    UMaterialInstanceDynamic* Material = UMaterialInstanceDynamic::Create(SunMaterial, DecorativeSunActor);
    Material->SetScalarParameterValue(TEXT("EmissiveStrength"), static_cast<float>(SunVisualEmissiveStrength));
    Component->SetMaterial(0, Material);
    Component->SetupAttachment(Root);
    Component->RegisterComponent();
    DecorativeSunComponent = Component;
    CreateDecorativeSunEffects(Root);
    UpdateDecorativeSunScale();

    DecorativeSunActor->Tags.AddUnique(FName(TEXT("BSK.Environment.SunVisual")));
    DecorativeSunActor->Tags.AddUnique(FName(TEXT("BSK.Semantic.sun")));
    UpdateDecorativeSunPlacement();
    UE_LOG(LogBskUnreal, Display,
        TEXT("Created Solar System Scope Sun visual distance=%.0f km angular_diameter=%.3f deg emissive=%.2f material=%s"),
        EffectiveDistanceMeters / 1000.0,
        CurrentSunAngularDiameterDegrees,
        SunVisualEmissiveStrength,
        *SunVisualMaterialPath);
}

void ABskSceneController::CreateDecorativeSunEffects(USceneComponent* Root)
{
    if (!bEnableDecorativeSunEffects || !DecorativeSunActor || !Root) return;

    const TArray<TPair<FString, FString>> Effects = {
        {TEXT("SunBursts"), SunBurstParticlePath},
        {TEXT("SunHalo"), SunHaloParticlePath},
        {TEXT("SunLines"), SunLinesParticlePath},
    };
    for (const TPair<FString, FString>& Effect : Effects)
    {
        if (Effect.Value.IsEmpty()) continue;
        UParticleSystem* Template = LoadObject<UParticleSystem>(nullptr, *Effect.Value);
        if (!Template)
        {
            UE_LOG(LogBskUnreal, Warning,
                TEXT("SP_space Sun particle asset unavailable name=%s path=%s"),
                *Effect.Key, *Effect.Value);
            continue;
        }

        const FName ComponentName = MakeUniqueObjectName(
            DecorativeSunActor, UParticleSystemComponent::StaticClass(), *FString::Printf(TEXT("%sParticle"), *Effect.Key));
        UParticleSystemComponent* Component = NewObject<UParticleSystemComponent>(DecorativeSunActor, ComponentName);
        if (!Component) continue;
        Component->SetMobility(EComponentMobility::Movable);
        Component->SetTemplate(Template);
        Component->bAutoActivate = true;
        Component->bAutoDestroy = false;
        Component->SetCastShadow(false);
        Component->SetReceivesDecals(false);
        Component->SetCanEverAffectNavigation(false);
        Component->SetupAttachment(Root);
        Component->RegisterComponent();
        Component->ActivateSystem(true);
        DecorativeSunParticleComponents.Add(Component);

        UE_LOG(LogBskUnreal, Display,
            TEXT("Added SP_space Sun particle effect name=%s asset=%s"),
            *Effect.Key, *Effect.Value);
    }
}

void ABskSceneController::UpdateDecorativeSunScale()
{
    if (!DecorativeSunComponent || DecorativeSunSourceRadiusCentimeters <= UE_DOUBLE_SMALL_NUMBER) return;
    const double MaximumDistanceMeters = FMath::Max(1000.0, CelestialVaultRadiusKilometers * 1000.0 * 0.92);
    const double EffectiveDistanceMeters = FMath::Min(SunVisualDistanceMeters, MaximumDistanceMeters);
    const double AngularRadiusRadians = FMath::DegreesToRadians(CurrentSunAngularDiameterDegrees * 0.5);
    const double TargetRadiusMeters = EffectiveDistanceMeters * FMath::Tan(AngularRadiusRadians);
    const double MeshScale = TargetRadiusMeters * Converter.GetCentimetersPerMeter() /
        DecorativeSunSourceRadiusCentimeters;
    DecorativeSunComponent->SetRelativeScale3D(FVector(MeshScale));
    DecorativeSunComponent->SetRelativeLocation(-DecorativeSunSourceCenterCentimeters * MeshScale);
    for (UParticleSystemComponent* Particle : DecorativeSunParticleComponents)
    {
        if (!Particle) continue;
        Particle->SetRelativeScale3D(FVector(MeshScale * SunVisualEffectScale));
    }
}

void ABskSceneController::UpdateDecorativeSunPlacement()
{
    if (!DecorativeSunActor || !SunLight) return;
    FVector ViewLocation = FVector::ZeroVector;
    FRotator ViewRotation = FRotator::ZeroRotator;
    if (APlayerController* Player = GetWorld()->GetFirstPlayerController())
    {
        Player->GetPlayerViewPoint(ViewLocation, ViewRotation);
    }
    FVector3d ViewMeters = FVector3d(ViewLocation) / Converter.GetCentimetersPerMeter();
    if (Converter.MirrorsLocalY()) ViewMeters.Y = -ViewMeters.Y;
    const double Visibility = SolarVisibilityAt(ViewMeters);
    // A near-camera Sun proxy would otherwise draw in FRONT of the Earth
    // during eclipse. Use physical geometry, not the proxy's depth.
    DecorativeSunActor->SetActorHiddenInGame(Visibility <= 1.0e-6);
    if (DecorativeSunComponent)
    {
        if (UMaterialInstanceDynamic* Material = Cast<UMaterialInstanceDynamic>(DecorativeSunComponent->GetMaterial(0)))
        {
            Material->SetScalarParameterValue(TEXT("EmissiveStrength"),
                static_cast<float>(SunVisualEmissiveStrength * Visibility));
        }
    }
    const FVector DirectionTowardSun = CurrentSunRadiusMeters > 0.0
        ? FVector(Converter.LocalMetersToUnrealCentimeters(CurrentSunPositionMeters - ViewMeters)).GetSafeNormal()
        : CurrentSunSourceDirection.GetSafeNormal();
    if (DirectionTowardSun.IsNearlyZero()) return;
    const double MaximumDistanceMeters = FMath::Max(1000.0, CelestialVaultRadiusKilometers * 1000.0 * 0.92);
    const double EffectiveDistanceMeters = FMath::Min(SunVisualDistanceMeters, MaximumDistanceMeters);
    const double DistanceCentimeters = EffectiveDistanceMeters * Converter.GetCentimetersPerMeter();
    const FVector SunLocation = ViewLocation + DirectionTowardSun * DistanceCentimeters;
    // The decorative Sun is a camera-relative render proxy, not a physical
    // body. Avoid moving it by tiny sub-pixel amounts every game tick: at the
    // old astronomical proxy distance this repeatedly invalidated LWC tiles
    // used by the attached Cascade particles and could make the main viewport
    // flash. The auxiliary SceneCapture cameras do not see this proxy, which
    // is why they did not exhibit the same symptom.
    if (!DecorativeSunActor->GetActorLocation().Equals(SunLocation, 1.0f))
    {
        DecorativeSunActor->SetActorLocation(
            SunLocation,
            false,
            nullptr,
            ETeleportType::TeleportPhysics);
    }
}

void ABskSceneController::CreateDecorativeEarth()
{
    if (!bEnableDecorativeEarth) return;
    DecorativeEarthActor = SpawnTexturedEarth(TEXT("decorative_earth"), DecorativeEarthRadiusMeters);
    if (!DecorativeEarthActor)
    {
        UE_LOG(LogBskUnreal, Warning, TEXT("Decorative Earth was requested but its textured assets could not be created"));
        return;
    }
    DecorativeEarthActor->SetActorLocation(
        FVector(Converter.LocalMetersToUnrealCentimeters(DecorativeEarthPositionMeters)),
        false, nullptr, ETeleportType::TeleportPhysics);
    DecorativeEarthActor->SetActorRotation(DecorativeEarthRotation);
    DecorativeEarthActor->Tags.AddUnique(FName(TEXT("BSK.Environment.DecorativeEarth")));
    DecorativeEarthActor->Tags.AddUnique(FName(TEXT("BSK.Semantic.earth")));
    UE_LOG(LogBskUnreal, Display, TEXT("Decorative Earth enabled position_m=%s rotation=%s"),
        *DecorativeEarthPositionMeters.ToString(), *DecorativeEarthRotation.ToCompactString());
}

void ABskSceneController::CreateEnvironment()
{
    CurrentSunSourceDirection = -SunRotation.Vector().GetSafeNormal();
    FActorSpawnParameters Params;
    Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    if (ADirectionalLight* Sun = GetWorld()->SpawnActor<ADirectionalLight>(ADirectionalLight::StaticClass(), FVector::ZeroVector, SunRotation, Params))
    {
        SunLight = Sun;
        ULightComponent* Light = Sun->GetLightComponent();
        // Reproduce the MyProject2 DirectionalLight visual properties. Mobility,
        // rotation, and runtime intensity remain dynamic so Basilisk/SPICE can
        // continue to drive the illumination direction and distance law.
        Light->SetMobility(EComponentMobility::Movable);
        Light->SetIntensity(static_cast<float>(SunIntensityLux));
        Light->SetLightColor(FLinearColor::White);
        Light->SetCastShadows(true);
        Light->CastStaticShadows = true;
        Light->CastDynamicShadows = true;
        Light->SetCastVolumetricShadow(true);
        Light->SetAffectTranslucentLighting(true);
        Light->SetTransmission(false);
        Light->SetUseTemperature(false);
        Light->SetTemperature(6500.0f);
        Light->SetIndirectLightingIntensity(1.0f);
        Light->SetVolumetricScatteringIntensity(1.0f);
        Light->SetSpecularScale(1.0f);
        Light->SetShadowBias(0.5f);
        Light->SetShadowSlopeBias(0.5f);
        Light->ContactShadowLength = 0.0f;
        Light->ContactShadowCastingIntensity = 1.0f;
        Light->ContactShadowNonCastingIntensity = 0.0f;
        if (UDirectionalLightComponent* Directional = Cast<UDirectionalLightComponent>(Light))
        {
            Directional->SetLightSourceAngle(0.5357f);
            Directional->SetLightSourceSoftAngle(0.0f);
            Directional->SetAtmosphereSunLight(true);
            Directional->SetAtmosphereSunLightIndex(0);
            Directional->SetForwardShadingPriority(1);
            Directional->bPerPixelAtmosphereTransmittance = true;
            Directional->CloudScatteredLuminanceScale = FLinearColor::White;
        }
    }

    // MuJoCo's default viewer uses a camera/headlight contribution in addition
    // to its key light.  This low-intensity, shadowless fill preserves the same
    // useful shape readability without pretending to be another physical sun.
    if (ADirectionalLight* CelestialSun = GetWorld()->SpawnActor<ADirectionalLight>(
        FVector::ZeroVector, SunRotation, Params))
    {
        CelestialSunLight = CelestialSun;
        ULightComponent* Light = CelestialSun->GetLightComponent();
        Light->SetMobility(EComponentMobility::Movable);
        Light->SetLightingChannels(false, true, false);
        Light->SetIntensity(static_cast<float>(SunIntensityLux));
        Light->SetCastShadows(false);
        Light->SetIndirectLightingIntensity(0.0f);
        Light->SetAffectTranslucentLighting(true);
        if (UDirectionalLightComponent* Directional = Cast<UDirectionalLightComponent>(Light))
        {
            Directional->SetAtmosphereSunLight(false);
        }
    }

    if (FillLightIntensityLux > 0.0)
    {
        const FRotator FillRotation(-SunRotation.Pitch * 0.5, SunRotation.Yaw + 165.0, 0.0);
        if (ADirectionalLight* Fill = GetWorld()->SpawnActor<ADirectionalLight>(ADirectionalLight::StaticClass(), FVector::ZeroVector, FillRotation, Params))
        {
            FillLight = Fill;
            Fill->GetLightComponent()->SetIntensity(static_cast<float>(FillLightIntensityLux));
            Fill->GetLightComponent()->SetLightColor(FLinearColor(0.55f, 0.68f, 1.0f));
            Fill->GetLightComponent()->SetCastShadows(false);
        }
    }

    // MyProject2 relies on project auto/local exposure and has no overriding
    // PostProcessVolume. Keep the manual path available for other deployments.
    if (bUseManualExposure)
    {
        if (APostProcessVolume* Volume = GetWorld()->SpawnActor<APostProcessVolume>(APostProcessVolume::StaticClass(), FVector::ZeroVector, FRotator::ZeroRotator, Params))
        {
            ExposureVolume = Volume;
            Volume->bUnbound = true;
            Volume->Priority = 1000.0f;
            Volume->Settings.bOverride_AutoExposureMethod = true;
            Volume->Settings.AutoExposureMethod = EAutoExposureMethod::AEM_Manual;
            Volume->Settings.bOverride_AutoExposureApplyPhysicalCameraExposure = true;
            Volume->Settings.AutoExposureApplyPhysicalCameraExposure = true;
            Volume->Settings.bOverride_CameraISO = true;
            Volume->Settings.CameraISO = 100.0f;
            Volume->Settings.bOverride_CameraShutterSpeed = true;
            Volume->Settings.CameraShutterSpeed = 125.0f;
            Volume->Settings.bOverride_DepthOfFieldFstop = true;
            Volume->Settings.DepthOfFieldFstop = 16.0f;
            Volume->Settings.bOverride_AutoExposureBias = true;
            Volume->Settings.AutoExposureBias = static_cast<float>(MaterialExposureBias);
        }
    }

    const bool bUsingTexturedStarSphere = CreateTexturedStarSphere();
    if (!bUsingTexturedStarSphere && bUseOfficialCelestialAssets)
    {
        UStaticMesh* VaultMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/CelestialVault/Meshes/SM_CelestialVault.SM_CelestialVault"));
        UMaterialInterface* VaultMaterial = LoadObject<UMaterialInterface>(nullptr, TEXT("/CelestialVault/Materials/MI_CelestialVault.MI_CelestialVault"));
        if (VaultMesh && VaultMaterial)
        {
            DeepSkyComponent = NewObject<UStaticMeshComponent>(this, TEXT("EpicCelestialVaultBackground"));
            DeepSkyComponent->SetupAttachment(GetRootComponent());
            DeepSkyComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
            DeepSkyComponent->SetCastShadow(false);
            DeepSkyComponent->SetAffectDynamicIndirectLighting(false);
            DeepSkyComponent->SetCanEverAffectNavigation(false);
            DeepSkyComponent->SetStaticMesh(VaultMesh);
            UMaterialInstanceDynamic* VaultInstance = UMaterialInstanceDynamic::Create(VaultMaterial, this);
            VaultInstance->SetScalarParameterValue(TEXT("Global Intensity"), static_cast<float>(CelestialBackgroundIntensity));
            VaultInstance->SetScalarParameterValue(TEXT("Background Intensity"), 1.0f);
            VaultInstance->SetScalarParameterValue(TEXT("Constellations Intensity"), 0.0f);
            VaultInstance->SetScalarParameterValue(TEXT("Grid Intensity"), 0.0f);
            DeepSkyComponent->SetMaterial(0, VaultInstance);
            DeepSkyComponent->SetRelativeScale3D(FVector(CelestialVaultRadiusKilometers * 1000.0));
            DeepSkyComponent->RegisterComponent();
            UE_LOG(LogBskUnreal, Display, TEXT("Using Epic Celestial Vault Milky Way background at %.0f km"), CelestialVaultRadiusKilometers);
        }
        else
        {
            UE_LOG(LogBskUnreal, Warning, TEXT("Epic Celestial Vault background assets unavailable; keeping black background"));
        }
    }

    CreateDecorativeSun();
    CreateDecorativeEarth();

    // MyProject2 uses only its textured star sphere and no additional star
    // instances. A positive StarCount remains an optional renderer extension.
    if (StarCount <= 0)
    {
        UE_LOG(LogBskUnreal, Display, TEXT("No supplemental star instances requested; matching MyProject2"));
        return;
    }

    UInstancedStaticMeshComponent* Stars = NewObject<UInstancedStaticMeshComponent>(this, TEXT("StarField"));
    Stars->SetupAttachment(GetRootComponent());
    Stars->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Stars->SetCastShadow(false);
    UStaticMesh* OfficialStarMesh = bUseOfficialCelestialAssets
        ? LoadObject<UStaticMesh>(nullptr, TEXT("/CelestialVault/Meshes/SM_Plane_FacingX.SM_Plane_FacingX"))
        : nullptr;
    UMaterialInterface* OfficialStarMaterial = bUseOfficialCelestialAssets
        ? LoadObject<UMaterialInterface>(nullptr, TEXT("/CelestialVault/Materials/MI_Stars.MI_Stars"))
        : nullptr;
    const bool bUsingOfficialStars = OfficialStarMesh && OfficialStarMaterial;
    Stars->SetStaticMesh(bUsingOfficialStars ? OfficialStarMesh : LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Sphere.Sphere")));
    if (bUsingOfficialStars)
    {
        Stars->SetNumCustomDataFloats(4);
        UMaterialInstanceDynamic* StarInstance = UMaterialInstanceDynamic::Create(OfficialStarMaterial, this);
        StarInstance->SetScalarParameterValue(TEXT("MagnitudeOffset"), -2.0f);
        StarInstance->SetScalarParameterValue(TEXT("Global Size Factor"), 1.25f);
        Stars->SetMaterial(0, StarInstance);
    }
    else if (UMaterialInterface* StarMaterial = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/BSK/M_BskUnlitColor.M_BskUnlitColor")))
    {
        UMaterialInstanceDynamic* StarInstance = UMaterialInstanceDynamic::Create(StarMaterial, this);
        StarInstance->SetVectorParameterValue(TEXT("Color"), FLinearColor(2.5f, 2.5f, 2.5f, 1.0f));
        Stars->SetMaterial(0, StarInstance);
    }
    Stars->RegisterComponent();
    FRandomStream Random(0x42534b);
    const double RadiusCm = bUsingOfficialStars
        ? CelestialVaultRadiusKilometers * 100000.0 * 0.99
        : StarRadiusMeters * Converter.GetCentimetersPerMeter();
    for (int32 Index = 0; Index < StarCount; ++Index)
    {
        FVector Direction(Random.FRandRange(-1.0f, 1.0f), Random.FRandRange(-1.0f, 1.0f), Random.FRandRange(-1.0f, 1.0f));
        if (!Direction.Normalize()) { --Index; continue; }
        if (bUsingOfficialStars)
        {
            const int32 InstanceIndex = Stars->AddInstance(FTransform(FQuat::Identity, Direction * RadiusCm));
            const float Warmth = Random.FRandRange(0.0f, 1.0f);
            const FLinearColor StarColor = FLinearColor::LerpUsingHSV(FLinearColor(0.62f, 0.75f, 1.0f), FLinearColor(1.0f, 0.78f, 0.55f), Warmth);
            TArray<float> CustomData{StarColor.R, StarColor.G, StarColor.B, static_cast<float>(Random.FRandRange(0.0f, 5.5f))};
            Stars->SetCustomData(InstanceIndex, CustomData);
        }
        else
        {
            const float Scale = Random.FRandRange(0.4f, 1.0f);
            Stars->AddInstance(FTransform(FQuat::Identity, Direction * RadiusCm, FVector(Scale)));
        }
    }
    UE_LOG(LogBskUnreal, Display, TEXT("Created %d %s stars over %s background"), StarCount,
        bUsingOfficialStars ? TEXT("Epic Celestial Vault") : TEXT("fallback"),
        bUsingTexturedStarSphere ? TEXT("textured Milky Way") : TEXT("Celestial Vault/black"));
}
