#include "Capture/BskCaptureNetworkSender.h"
#include "Misc/AutomationTest.h"
#include "HAL/PlatformTime.h"
#include "HAL/PlatformProcess.h"
#include "SocketSubsystem.h"
#include "Sockets.h"
#include "IPAddress.h"

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBskCaptureNetworkCancellationTest,
    "BskUnreal.Capture.NetworkCancellation",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ClientContext | EAutomationTestFlags::EngineFilter)

bool FBskCaptureNetworkCancellationTest::RunTest(const FString& Parameters)
{
    ISocketSubsystem* Sockets = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
    FSocket* Listener = Sockets->CreateSocket(NAME_Stream, TEXT("Capture cancellation listener"), false);
    TSharedRef<FInternetAddr> Address = Sockets->CreateInternetAddr();
    bool bValid = false;
    Address->SetIp(TEXT("127.0.0.1"), bValid);
    Address->SetPort(0);
    if (!Listener || !Listener->Bind(*Address) || !Listener->Listen(1))
    {
        AddError(TEXT("cannot create loopback cancellation fixture"));
        if (Listener) Sockets->DestroySocket(Listener);
        return false;
    }
    Listener->GetAddress(*Address);
    Listener->SetNonBlocking(true);
    auto Sender = MakeUnique<FBskCaptureNetworkSender>(TEXT("127.0.0.1"), Address->GetPort());
    TestTrue(TEXT("packet admitted"), Sender->EnqueueReliable(TArray<uint8>{42}));
    FSocket* Peer = nullptr;
    const double ConnectDeadline = FPlatformTime::Seconds() + 2.0;
    while (!Peer && FPlatformTime::Seconds() < ConnectDeadline)
    {
        bool bPending = false;
        if (Listener->HasPendingConnection(bPending) && bPending) Peer = Listener->Accept(TEXT("No ACK peer"));
        if (!Peer) FPlatformProcess::Sleep(0.005f);
    }
    TestNotNull(TEXT("sender connected to loopback peer"), Peer);
    if (Peer)
    {
        TestTrue(TEXT("packet sent before waiting for ACK"), Peer->Wait(ESocketWaitConditions::WaitForRead, FTimespan::FromSeconds(2)));
        uint8 Byte = 0;
        int32 Read = 0;
        Peer->Recv(&Byte, 1, Read);
        TestEqual(TEXT("peer received original packet"), Byte, uint8(42));
    }
    const double Start = FPlatformTime::Seconds();
    Sender.Reset(); // Peer deliberately never sends its acknowledgement.
    const double Elapsed = FPlatformTime::Seconds() - Start;
    AddInfo(FString::Printf(TEXT("Stop while awaiting ACK: %.6f seconds"), Elapsed));
    TestTrue(TEXT("ACK cancellation finishes below two seconds"), Elapsed < 2.0);
    if (Peer) { Peer->Close(); Sockets->DestroySocket(Peer); }
    Listener->Close();
    Sockets->DestroySocket(Listener);
    // The now-closed loopback port exercises retry cancellation without any
    // external network dependency or assumption about a reachable remote host.
    Sender = MakeUnique<FBskCaptureNetworkSender>(TEXT("127.0.0.1"), Address->GetPort());
    Sender->EnqueueReliable(TArray<uint8>{42});
    FPlatformProcess::Sleep(0.02f);
    const double RetryStart = FPlatformTime::Seconds();
    Sender.Reset();
    TestTrue(TEXT("connection retry is cancellable"), FPlatformTime::Seconds() - RetryStart < 2.0);
    return true;
}
#endif
