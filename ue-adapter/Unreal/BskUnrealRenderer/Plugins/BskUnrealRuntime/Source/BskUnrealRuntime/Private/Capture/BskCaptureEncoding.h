#pragma once
#include "CoreMinimal.h"
#include "Dom/JsonObject.h"

// Capture serialization, image encoding and camera metadata conversion.
namespace BskCaptureEncoding
{
FString SafePathSegment(FString Value);
int64 UnixTimeNanoseconds();
void AddBigEndianUint32(TArray<uint8>& Bytes, uint32 Value);
bool CompressPng(const TArray<FColor>& Pixels, int32 Width, int32 Height, TArray<uint8>& OutBytes);
bool CompressJpeg(const TArray<FColor>& Pixels, int32 Width, int32 Height, int32 Quality, TArray<uint8>& OutBytes);
TArray<TSharedPtr<FJsonValue>> JsonVector(const FVector3d& Value);
TArray<TSharedPtr<FJsonValue>> JsonQuaternionWxyz(const FQuat4d& Value);
FQuat4d UnrealToActiveLocalWxyz(FQuat4d Value, bool bMirrorY);
void QuaternionMatrix(const FQuat4d& Q, double Out[3][3]);
TArray<TSharedPtr<FJsonValue>> JsonMatrix3(const double Matrix[3][3]);
}
