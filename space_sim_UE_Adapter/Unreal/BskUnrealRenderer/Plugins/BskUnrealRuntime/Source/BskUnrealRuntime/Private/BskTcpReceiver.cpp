#include "BskTcpReceiver.h"

#include "BskUnrealRuntime.h"
#include "Common/TcpSocketBuilder.h"
#include "HAL/PlatformProcess.h"
#include "HAL/RunnableThread.h"
#include "IPAddress.h"
#include "Interfaces/IPv4/IPv4Address.h"
#include "Interfaces/IPv4/IPv4Endpoint.h"
#include "SocketSubsystem.h"
#include "Sockets.h"

FBskTcpReceiver::FBskTcpReceiver(FString InListenAddress, uint16 InPort, uint32 InMaxPacketBytes, bool bInReliableFrames)
    : ListenAddress(MoveTemp(InListenAddress))
    , Port(InPort)
    , Parser(InMaxPacketBytes)
    , bReliableFrames(bInReliableFrames)
{
}

FBskTcpReceiver::~FBskTcpReceiver()
{
    StopReceiver();
}

bool FBskTcpReceiver::StartReceiver()
{
    if (Thread != nullptr)
    {
        return true;
    }
    bStopRequested.Store(false);
    Thread = FRunnableThread::Create(this, TEXT("BskTcpReceiver"), 0, TPri_BelowNormal);
    if (Thread == nullptr)
    {
        SetStatus(TEXT("failed to create receiver thread"));
        return false;
    }
    return true;
}

void FBskTcpReceiver::StopReceiver()
{
    Stop();
    if (Thread != nullptr)
    {
        Thread->WaitForCompletion();
        delete Thread;
        Thread = nullptr;
    }
}

void FBskTcpReceiver::Stop()
{
    bStopRequested.Store(true);
}

bool FBskTcpReceiver::SendCommandJson(const FString& CommandJson, FString& OutError)
{
    if (!bClientConnected.Load())
    {
        OutError = TEXT("no live BSK sender is connected");
        return false;
    }
    FTCHARToUTF8 Utf8(*CommandJson);
    if (Utf8.Length() <= 0 || Utf8.Length() > static_cast<int32>(BskProtocol::DefaultMaxPacketBytes))
    {
        OutError = TEXT("command JSON has an invalid encoded size");
        return false;
    }
    TArray<uint8> Packet;
    Packet.Reserve(Utf8.Length() + 4);
    const uint32 Length = static_cast<uint32>(Utf8.Length());
    Packet.Add(static_cast<uint8>((Length >> 24) & 0xff));
    Packet.Add(static_cast<uint8>((Length >> 16) & 0xff));
    Packet.Add(static_cast<uint8>((Length >> 8) & 0xff));
    Packet.Add(static_cast<uint8>(Length & 0xff));
    Packet.Append(reinterpret_cast<const uint8*>(Utf8.Get()), Utf8.Length());
    FScopeLock Lock(&OutboundMutex);
    constexpr int32 MaxPendingCommands = 64;
    if (OutboundPackets.Num() >= MaxPendingCommands)
    {
        OutError = TEXT("outbound command queue is full");
        return false;
    }
    OutboundPackets.Add(MoveTemp(Packet));
    return true;
}

FString FBskTcpReceiver::GetStatus() const
{
    FScopeLock Lock(&StatusMutex);
    return Status;
}

void FBskTcpReceiver::SetStatus(const FString& NewStatus)
{
    FScopeLock Lock(&StatusMutex);
    Status = NewStatus;
}

bool FBskTcpReceiver::ConsumeLatest(FBskRenderFrame& OutFrame)
{
    return ConsumeForCapture(OutFrame, true);
}

int32 FBskTcpReceiver::GetPendingFrameCount() const
{
    FScopeLock Lock(&LatestMutex);
    return ReliableFrames.Num();
}

bool FBskTcpReceiver::ConsumeForCapture(FBskRenderFrame& OutFrame, bool bCaptureHasCapacity)
{
    FScopeLock Lock(&LatestMutex);
    if (LatestManifest.IsValid()) return false;
    if (!ReliableFrames.IsEmpty())
    {
        if (!bCaptureHasCapacity) return false;
        OutFrame = MoveTemp(ReliableFrames[0]);
        ReliableFrames.RemoveAt(0, 1, EAllowShrinking::No);
        return true;
    }
    if (LatestManifest.IsValid() || !LatestFrame.IsValid())
    {
        return false;
    }
    if (!bCaptureHasCapacity && !LatestFrame->bCaptureOnDemand) return false;
    OutFrame = MoveTemp(*LatestFrame);
    LatestFrame.Reset();
    return true;
}

bool FBskTcpReceiver::ConsumeLatestManifest(FBskSceneManifest& OutManifest)
{
    FScopeLock Lock(&LatestMutex);
    if (!LatestManifest.IsValid()) return false;
    OutManifest = MoveTemp(*LatestManifest);
    LatestManifest.Reset();
    return true;
}

bool FBskTcpReceiver::ConsumeEvent(FBskRenderEvent& OutEvent)
{
    FScopeLock Lock(&LatestMutex);
    if (LatestManifest.IsValid() || Events.IsEmpty()) return false;
    OutEvent = MoveTemp(Events[0]);
    Events.RemoveAt(0, 1, EAllowShrinking::No);
    return true;
}

void FBskTcpReceiver::PublishLatest(FBskRenderFrame&& Frame)
{
    const bool bStrict = Frame.bCaptureOnDemand ? !Frame.CaptureEpisodeId.IsEmpty() : bReliableFrames;
    if (bStrict)
    {
        // Bounded FIFO applies TCP backpressure instead of overwriting unseen
        // authoritative frames. Release the lock while the game thread drains.
        while (!bStopRequested.Load())
        {
            {
                FScopeLock Lock(&LatestMutex);
                if (!IncomingSessionId.IsEmpty() && !Frame.SessionId.IsEmpty() && Frame.SessionId != IncomingSessionId) return;
                if (ReliableFrames.Num() < 128)
                {
                    // Do not display an older pre-start preview after this FIFO.
                    if (LatestFrame.IsValid())
                    {
                        LatestFrame.Reset();
                        ++OverwrittenFrameCount;
                    }
                    ReliableFrames.Add(MoveTemp(Frame));
                    ++ReceivedFrameCount;
                    return;
                }
            }
            FPlatformProcess::Sleep(0.001f);
        }
        return;
    }
    FScopeLock Lock(&LatestMutex);
    if (!IncomingSessionId.IsEmpty() && !Frame.SessionId.IsEmpty() && Frame.SessionId != IncomingSessionId) return;
    if (LatestFrame.IsValid())
    {
        ++OverwrittenFrameCount;
    }
    LatestFrame = MakeShared<FBskRenderFrame, ESPMode::ThreadSafe>(MoveTemp(Frame));
    ++ReceivedFrameCount;
}

void FBskTcpReceiver::PublishManifest(FBskSceneManifest&& Manifest)
{
    FScopeLock Lock(&LatestMutex);
    if (IncomingSessionId != Manifest.SessionId)
    {
        LatestFrame.Reset();
        ReliableFrames.Reset();
        Events.Reset();
        IncomingSessionId = Manifest.SessionId;
    }
    LatestManifest = MakeShared<FBskSceneManifest, ESPMode::ThreadSafe>(MoveTemp(Manifest));
}

void FBskTcpReceiver::PublishEvent(FBskRenderEvent&& Event)
{
    FScopeLock Lock(&LatestMutex);
    if (!IncomingSessionId.IsEmpty() && Event.SessionId != IncomingSessionId) return;
    if (Event.EventKind == TEXT("scene_reset")) { LatestFrame.Reset(); ReliableFrames.Reset(); }
    constexpr int32 MaxPendingEvents = 64;
    if (Events.Num() >= MaxPendingEvents) Events.RemoveAt(0, 1, EAllowShrinking::No);
    Events.Add(MoveTemp(Event));
}

bool FBskTcpReceiver::CreateListener()
{
    FIPv4Address Address;
    if (!FIPv4Address::Parse(ListenAddress, Address))
    {
        SetStatus(FString::Printf(TEXT("invalid listen address: %s"), *ListenAddress));
        return false;
    }
    ListenSocket = FTcpSocketBuilder(TEXT("BskTcpListener"))
        .AsReusable()
        .AsNonBlocking()
        .BoundToEndpoint(FIPv4Endpoint(Address, Port))
        .Listening(1);
    if (ListenSocket == nullptr)
    {
        SetStatus(FString::Printf(TEXT("failed to listen on %s:%u"), *ListenAddress, Port));
        return false;
    }
    SetStatus(FString::Printf(TEXT("listening on %s:%u"), *ListenAddress, Port));
    UE_LOG(LogBskUnreal, Display, TEXT("BSK receiver listening on %s:%u"), *ListenAddress, Port);
    return true;
}

void FBskTcpReceiver::CloseClient()
{
    bClientConnected.Store(false);
    if (ClientSocket != nullptr)
    {
        ClientSocket->Close();
        ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(ClientSocket);
        ClientSocket = nullptr;
    }
    Parser.Reset();
    {
        FScopeLock Lock(&OutboundMutex);
        OutboundPackets.Reset();
    }
}

void FBskTcpReceiver::CloseSockets()
{
    CloseClient();
    if (ListenSocket != nullptr)
    {
        ListenSocket->Close();
        ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(ListenSocket);
        ListenSocket = nullptr;
    }
}

uint32 FBskTcpReceiver::Run()
{
    if (!CreateListener())
    {
        return 1;
    }

    TArray<uint8> ReadBuffer;
    ReadBuffer.SetNumUninitialized(64 * 1024);
    TArray<uint8> ActiveOutboundPacket;
    int32 ActiveOutboundOffset = 0;
    while (!bStopRequested.Load())
    {
        if (ClientSocket == nullptr)
        {
            bool bPending = false;
            if (ListenSocket->HasPendingConnection(bPending) && bPending)
            {
                ClientSocket = ListenSocket->Accept(TEXT("BskTcpClient"));
                if (ClientSocket != nullptr)
                {
                    ClientSocket->SetNonBlocking(true);
                    ClientSocket->SetNoDelay(true);
                    bClientConnected.Store(true);
                    Parser.Reset();
                    SetStatus(TEXT("client connected"));
                    UE_LOG(LogBskUnreal, Display, TEXT("BSK sender connected"));
                }
            }
            FPlatformProcess::SleepNoStats(0.005f);
            continue;
        }

        uint32 PendingBytes = 0;
        bool bReadAny = false;
        while (ClientSocket->HasPendingData(PendingBytes) && PendingBytes > 0 && !bStopRequested.Load())
        {
            const int32 Requested = FMath::Min<int32>(ReadBuffer.Num(), static_cast<int32>(PendingBytes));
            int32 BytesRead = 0;
            if (!ClientSocket->Recv(ReadBuffer.GetData(), Requested, BytesRead) || BytesRead <= 0)
            {
                break;
            }
            bReadAny = true;
            TArray<FBskRenderMessage> Messages;
            FString Error;
            if (!Parser.AppendMessages(ReadBuffer.GetData(), BytesRead, Messages, Error))
            {
                UE_LOG(LogBskUnreal, Warning, TEXT("Discarding invalid BSK connection: %s"), *Error);
                SetStatus(FString::Printf(TEXT("protocol error: %s"), *Error));
                CloseClient();
                break;
            }
            for (FBskRenderMessage& Message : Messages)
            {
                switch (Message.Type)
                {
                case EBskRenderMessageType::Hello:
                    UE_LOG(LogBskUnreal, Display, TEXT("BSK render session hello: %s"), *Message.SessionId);
                    break;
                case EBskRenderMessageType::SceneManifest:
                    PublishManifest(MoveTemp(Message.Manifest));
                    break;
                case EBskRenderMessageType::Frame:
                    PublishLatest(MoveTemp(Message.Frame));
                    break;
                case EBskRenderMessageType::Event:
                    PublishEvent(MoveTemp(Message.Event));
                    break;
                default:
                    break;
                }
            }
        }

        if (ClientSocket != nullptr)
        {
            if (ActiveOutboundPacket.IsEmpty())
            {
                FScopeLock Lock(&OutboundMutex);
                if (!OutboundPackets.IsEmpty())
                {
                    ActiveOutboundPacket = MoveTemp(OutboundPackets[0]);
                    OutboundPackets.RemoveAt(0, 1, EAllowShrinking::No);
                    ActiveOutboundOffset = 0;
                }
            }
            if (!ActiveOutboundPacket.IsEmpty() &&
                ClientSocket->Wait(ESocketWaitConditions::WaitForWrite, FTimespan::Zero()))
            {
                int32 BytesSent = 0;
                if (!ClientSocket->Send(
                    ActiveOutboundPacket.GetData() + ActiveOutboundOffset,
                    ActiveOutboundPacket.Num() - ActiveOutboundOffset,
                    BytesSent) || BytesSent <= 0)
                {
                    UE_LOG(LogBskUnreal, Warning, TEXT("BSK command channel send failed; closing the client session"));
                    CloseClient();
                    ActiveOutboundPacket.Reset();
                    ActiveOutboundOffset = 0;
                }
                else
                {
                    ActiveOutboundOffset += BytesSent;
                    if (ActiveOutboundOffset >= ActiveOutboundPacket.Num())
                    {
                        ActiveOutboundPacket.Reset();
                        ActiveOutboundOffset = 0;
                    }
                }
            }
        }

        // A graceful TCP close becomes readable with zero payload. On Windows,
        // GetConnectionState() alone can continue to report Connected until a
        // recv observes that FIN, which would prevent accepting a reconnect.
        if (ClientSocket != nullptr && !bReadAny &&
            ClientSocket->Wait(ESocketWaitConditions::WaitForRead, FTimespan::Zero()))
        {
            uint8 Probe = 0;
            int32 ProbeBytes = 0;
            if (!ClientSocket->Recv(&Probe, 1, ProbeBytes, ESocketReceiveFlags::Peek) || ProbeBytes == 0)
            {
                UE_LOG(LogBskUnreal, Display, TEXT("BSK sender disconnected; retaining last rendered frame"));
                CloseClient();
                SetStatus(FString::Printf(TEXT("listening on %s:%u"), *ListenAddress, Port));
            }
        }

        if (ClientSocket != nullptr && ClientSocket->GetConnectionState() != SCS_Connected)
        {
            UE_LOG(LogBskUnreal, Display, TEXT("BSK sender disconnected; retaining last rendered frame"));
            CloseClient();
            SetStatus(FString::Printf(TEXT("listening on %s:%u"), *ListenAddress, Port));
        }
        if (!bReadAny)
        {
            FPlatformProcess::SleepNoStats(0.002f);
        }
    }
    CloseSockets();
    SetStatus(TEXT("stopped"));
    return 0;
}
