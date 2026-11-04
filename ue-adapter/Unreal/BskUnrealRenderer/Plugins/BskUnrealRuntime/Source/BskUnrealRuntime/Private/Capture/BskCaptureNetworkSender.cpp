#include "BskCaptureNetworkSender.h"
#include "BskUnrealRuntime.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"
#include "HAL/PlatformTime.h"
#include "HAL/PlatformProcess.h"
#include "IPAddress.h"
#include "SocketSubsystem.h"
#include "Sockets.h"

class FBskCaptureNetworkSender::FWorker final : public FRunnable
{
public:
    FWorker(FString InAddress, uint16 InPort)
        : Address(MoveTemp(InAddress)), Port(InPort)
    {
        WakeEvent = FPlatformProcess::GetSynchEventFromPool(false);
        Thread = FRunnableThread::Create(this, TEXT("BskCaptureOutput"));
    }

    virtual ~FWorker() override
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
                WakeEvent->Wait(100);
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
    struct FResolveState
    {
        TSharedPtr<FInternetAddr> Address;
        TAtomic<bool> bFinished{false};
    };

    bool EnsureConnected()
    {
        if (Socket) return true;
        ISocketSubsystem* SocketSubsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
        if (!SocketSubsystem || bStopRequested) return false;
        TSharedPtr<FInternetAddr> InternetAddress = SocketSubsystem->GetAddressFromString(Address);
        if (!InternetAddress)
        {
            // The callback owns only this shared result, never the sender. A DNS
            // request can finish after cancellation without touching freed state.
            // Keep a pending lookup across retries, so slow DNS cannot grow tasks.
            if (!ResolveState)
            {
                ResolveState = MakeShared<FResolveState, ESPMode::ThreadSafe>();
                SocketSubsystem->GetAddressInfoAsync([State = ResolveState](FAddressInfoResult Result)
                {
                    if (!Result.Results.IsEmpty()) State->Address = Result.Results[0].Address;
                    State->bFinished.Store(true);
                }, *Address, nullptr, EAddressInfoFlags::Default, NAME_None, ESocketType::SOCKTYPE_Streaming);
            }
            const double Deadline = FPlatformTime::Seconds() + 5.0;
            while (!bStopRequested && !ResolveState->bFinished.Load() && FPlatformTime::Seconds() < Deadline)
                WakeEvent->Wait(50);
            if (bStopRequested || !ResolveState->bFinished.Load()) return false;
            InternetAddress = ResolveState->Address;
            ResolveState.Reset();
            if (!InternetAddress) return false;
        }
        InternetAddress->SetPort(Port);
        Socket = SocketSubsystem->CreateSocket(NAME_Stream, TEXT("BSK capture output"), InternetAddress->GetProtocolType());
        if (!Socket) return false;
        Socket->SetNonBlocking(true);
        Socket->SetNoDelay(true);
        Socket->SetSendBufferSize(16 * 1024 * 1024, SendBufferBytes);
        bool bConnected = Socket->Connect(*InternetAddress);
        if (!bConnected)
        {
            const ESocketErrors Error = SocketSubsystem->GetLastErrorCode();
            if (Error == SE_EWOULDBLOCK || Error == SE_EINPROGRESS || Error == SE_EALREADY)
            {
                const double Deadline = FPlatformTime::Seconds() + 5.0;
                while (!bStopRequested && FPlatformTime::Seconds() < Deadline)
                {
                    if (!Socket->Wait(ESocketWaitConditions::WaitForWrite, FTimespan::FromMilliseconds(50))) continue;
                    bConnected = Socket->GetConnectionState() == SCS_Connected;
                    break;
                }
            }
        }
        if (!bConnected || bStopRequested)
        {
            const double Now = FPlatformTime::Seconds();
            if (!bStopRequested && Now - LastConnectWarningSeconds >= 5.0)
            {
                UE_LOG(LogBskUnreal, Warning, TEXT("BSK capture output cannot connect to %s:%u"), *Address, Port);
                LastConnectWarningSeconds = Now;
            }
            CloseSocket();
            return false;
        }
        UE_LOG(LogBskUnreal, Display, TEXT("BSK capture output connected to %s:%u"), *Address, Port);
        return true;
    }

    bool WaitForAcknowledgement()
    {
        const double Deadline = FPlatformTime::Seconds() + 5.0;
        while (!bStopRequested && Socket && FPlatformTime::Seconds() < Deadline)
        {
            if (!Socket->Wait(ESocketWaitConditions::WaitForRead, FTimespan::FromMilliseconds(50))) continue;
            uint8 Ack = 0;
            int32 Received = 0;
            if (Socket->Recv(&Ack, 1, Received)) return Received == 1 && Ack == 1;
            if (ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->GetLastErrorCode() != SE_EWOULDBLOCK) return false;
        }
        return false;
    }

    bool SendAll(const TArray<uint8>& Packet)
    {
        int32 Offset = 0;
        double Progress = FPlatformTime::Seconds();
        const double Deadline = Progress + 30.0;
        while (!bStopRequested && Socket && Offset < Packet.Num())
        {
            if (FPlatformTime::Seconds() - Progress > 5.0 || FPlatformTime::Seconds() > Deadline) return false;
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
    double LastConnectWarningSeconds = 0.0;
    TSharedPtr<FResolveState, ESPMode::ThreadSafe> ResolveState;
};

FBskCaptureNetworkSender::FBskCaptureNetworkSender(FString Address, uint16 Port)
    : Worker(MakeUnique<FWorker>(MoveTemp(Address), Port)) {}
FBskCaptureNetworkSender::~FBskCaptureNetworkSender() = default;
bool FBskCaptureNetworkSender::HasReliableCapacity(int32 Count, uint64 Bytes) { return Worker->HasReliableCapacity(Count, Bytes); }
int32 FBskCaptureNetworkSender::GetPendingPacketCount() { return Worker->GetPendingPacketCount(); }
bool FBskCaptureNetworkSender::EnqueueReliable(TArray<uint8>&& Packet) { return Worker->EnqueueReliable(MoveTemp(Packet)); }
void FBskCaptureNetworkSender::EnqueueLatest(const FString& Key, TArray<uint8>&& Packet) { Worker->EnqueueLatest(Key, MoveTemp(Packet)); }
