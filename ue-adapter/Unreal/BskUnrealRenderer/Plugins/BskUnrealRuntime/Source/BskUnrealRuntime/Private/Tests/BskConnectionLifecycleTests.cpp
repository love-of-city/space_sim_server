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
