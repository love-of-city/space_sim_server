#pragma once
#include "CoreMinimal.h"

// Bounded, acknowledged frame transport. The worker owns its socket exclusively.
class FBskCaptureNetworkSender final
{
public:
    FBskCaptureNetworkSender(FString Address, uint16 Port);
    ~FBskCaptureNetworkSender();
    bool HasReliableCapacity(int32 PacketCount, uint64 ByteBudget);
    int32 GetPendingPacketCount();
    bool EnqueueReliable(TArray<uint8>&& Packet);
    void EnqueueLatest(const FString& StreamKey, TArray<uint8>&& Packet);
private:
    class FWorker;
    TUniquePtr<FWorker> Worker;
};
