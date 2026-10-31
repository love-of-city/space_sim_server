#pragma once
#include "CoreMinimal.h"

// CPU-owned encoded products, safe to transfer to output worker threads.
struct FCapturedDataProduct
{
    FString Name;
    FString Encoding;
    FString Extension;
    TArray<uint8> Bytes;
};
