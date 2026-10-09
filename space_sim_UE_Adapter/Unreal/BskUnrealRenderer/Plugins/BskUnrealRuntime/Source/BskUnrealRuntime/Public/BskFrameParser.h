#pragma once

#include "CoreMinimal.h"
#include "BskProtocolTypes.h"

class BSKUNREALRUNTIME_API FBskFrameParser
{
public:
    explicit FBskFrameParser(uint32 InMaxPacketBytes = BskProtocol::DefaultMaxPacketBytes);

    bool Append(const uint8* Data, int32 NumBytes, TArray<FBskRenderFrame>& OutFrames, FString& OutError);
    bool AppendMessages(const uint8* Data, int32 NumBytes, TArray<FBskRenderMessage>& OutMessages, FString& OutError);
    void Reset();

    static bool ParseJson(const FString& Json, FBskRenderFrame& OutFrame, FString& OutError);
    static bool ParseMessageJson(const FString& Json, FBskRenderMessage& OutMessage, FString& OutError);

private:
    TArray<uint8> Buffer;
    uint32 MaxPacketBytes;
};
