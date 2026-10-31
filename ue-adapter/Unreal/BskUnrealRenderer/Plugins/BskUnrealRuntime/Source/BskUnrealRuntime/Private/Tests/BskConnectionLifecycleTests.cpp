#if WITH_DEV_AUTOMATION_TESTS
#include "BskTcpReceiver.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBskConnectionLifecycleTest,
    "BskUnreal.Network.ConnectionRetiresPartialCommands",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ClientContext | EAutomationTestFlags::EngineFilter)

bool FBskConnectionLifecycleTest::RunTest(const FString& Parameters)
{
    FBskTcpReceiver Receiver(TEXT("127.0.0.1"), 0, BskProtocol::DefaultMaxPacketBytes);
    // FIN, protocol error, socket error and a newly accepted connection all use
    // this same retirement boundary. A partial packet must not escape it.
    Receiver.ActiveOutboundPacket = {0, 0, 0, 3, 1, 2, 3};
    Receiver.ActiveOutboundOffset = 5;
    Receiver.OutboundPackets.Add(TArray<uint8>{8, 9});
    Receiver.bClientConnected.Store(true);
    Receiver.CloseClient();
    TestTrue(TEXT("partial outgoing packet retired"), Receiver.ActiveOutboundPacket.IsEmpty());
    TestEqual(TEXT("partial offset retired"), Receiver.ActiveOutboundOffset, 0);
    TestTrue(TEXT("queued commands retired"), Receiver.OutboundPackets.IsEmpty());
    TestFalse(TEXT("connection no longer live"), Receiver.bClientConnected.Load());
    FString Error;
    TestFalse(TEXT("retired session rejects commands"), Receiver.SendCommandJson(TEXT("{}"), Error));
    Receiver.CloseClient();
    TestTrue(TEXT("repeated retirement remains safe"), Receiver.ActiveOutboundPacket.IsEmpty());
    return true;
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
#include "HAL/PlatformTime.h"
#include "HAL/PlatformProcess.h"
#include "SocketSubsystem.h"
#include "Sockets.h"
#include "IPAddress.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBskConnectionReconnectTest,
    "BskUnreal.Network.ReconnectAfterProtocolErrorAndFin",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ClientContext | EAutomationTestFlags::EngineFilter)

bool FBskConnectionReconnectTest::RunTest(const FString& Parameters)
{
    ISocketSubsystem* Sockets = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
    TSharedRef<FInternetAddr> Address = Sockets->CreateInternetAddr();
    bool bValid = false;
    Address->SetIp(TEXT("127.0.0.1"), bValid);
    Address->SetPort(0);
    FSocket* Reservation = Sockets->CreateSocket(NAME_Stream, TEXT("Receiver test port"), false);
    if (!Reservation || !Reservation->Bind(*Address))
    {
        AddError(TEXT("cannot reserve loopback test port"));
        if (Reservation) Sockets->DestroySocket(Reservation);
        return false;
    }
    Reservation->GetAddress(*Address);
    Reservation->Close();
    Sockets->DestroySocket(Reservation);
    FBskTcpReceiver Receiver(TEXT("127.0.0.1"), Address->GetPort(), BskProtocol::DefaultMaxPacketBytes);
    // Simulate a previous connection's partly transmitted command before the
    // first accept. Actual reconnects below exercise protocol error and FIN.
    Receiver.ActiveOutboundPacket = {0, 0, 0, 3, 1, 2, 3};
    Receiver.ActiveOutboundOffset = 5;
    TestTrue(TEXT("receiver thread starts"), Receiver.StartReceiver());
    for (int32 Attempt = 0; Attempt < 3; ++Attempt)
    {
        FSocket* Peer = nullptr;
        const double ConnectDeadline = FPlatformTime::Seconds() + 2.0;
        while (!Peer && FPlatformTime::Seconds() < ConnectDeadline)
        {
            FSocket* Candidate = Sockets->CreateSocket(NAME_Stream, TEXT("Reconnect peer"), false);
            if (Candidate && Candidate->Connect(*Address)) Peer = Candidate;
            else
            {
                if (Candidate) { Candidate->Close(); Sockets->DestroySocket(Candidate); }
                FPlatformProcess::Sleep(0.005f);
            }
        }
        if (!Peer) { AddError(TEXT("loopback peer failed to connect")); break; }
        const double AcceptedDeadline = FPlatformTime::Seconds() + 2.0;
        while (!Receiver.bClientConnected.Load() && FPlatformTime::Seconds() < AcceptedDeadline)
            FPlatformProcess::Sleep(0.005f);
        FString Error;
        const FString Json = FString::Printf(TEXT("{\"connection\":%d}"), Attempt);
        TestTrue(TEXT("fresh command admitted"), Receiver.SendCommandJson(Json, Error));
        FTCHARToUTF8 Utf8(*Json);
        TArray<uint8> Expected{0, 0, 0, static_cast<uint8>(Utf8.Length())};
        Expected.Append(reinterpret_cast<const uint8*>(Utf8.Get()), Utf8.Length());
        TArray<uint8> Received;
        const double ReceiveDeadline = FPlatformTime::Seconds() + 2.0;
        Peer->SetNonBlocking(true);
        while (Received.Num() < Expected.Num() && FPlatformTime::Seconds() < ReceiveDeadline)
        {
            if (!Peer->Wait(ESocketWaitConditions::WaitForRead, FTimespan::FromMilliseconds(20))) continue;
            uint8 Bytes[256];
            int32 Count = 0;
            if (Peer->Recv(Bytes, sizeof(Bytes), Count) && Count > 0) Received.Append(Bytes, Count);
        }
        TestTrue(TEXT("new connection receives only its complete fresh command"), Received == Expected);
        if (Attempt == 0)
        {
            const uint8 InvalidLength[4] = {255, 255, 255, 255};
            int32 Sent = 0;
            TestTrue(TEXT("invalid packet submitted"), Peer->Send(InvalidLength, 4, Sent));
            const double CloseDeadline = FPlatformTime::Seconds() + 2.0;
            while (Receiver.bClientConnected.Load() && FPlatformTime::Seconds() < CloseDeadline)
                FPlatformProcess::Sleep(0.005f);
            TestFalse(TEXT("protocol error closes connection"), Receiver.bClientConnected.Load());
        }
        Peer->Close();
        Sockets->DestroySocket(Peer);
        const double CloseDeadline = FPlatformTime::Seconds() + 2.0;
        while (Receiver.bClientConnected.Load() && FPlatformTime::Seconds() < CloseDeadline)
            FPlatformProcess::Sleep(0.005f);
        TestFalse(TEXT("FIN retires live connection before next accept"), Receiver.bClientConnected.Load());
    }
    Receiver.StopReceiver();
    return true;
}
#endif
