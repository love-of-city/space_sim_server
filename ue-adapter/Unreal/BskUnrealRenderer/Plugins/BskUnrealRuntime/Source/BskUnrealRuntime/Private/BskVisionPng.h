#pragma once

#include "CoreMinimal.h"

// Lossless BGRA8 PNG with one reversible row filter and zlib level 1.
// The decoded pixels and full sensor resolution are preserved, including alpha.
bool BskEncodeVisionPng(const TArray<FColor>& Pixels, int32 Width, int32 Height, TArray<uint8>& Output);
