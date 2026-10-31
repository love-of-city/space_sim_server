#include "BskCaptureEncoding.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Modules/ModuleManager.h"

namespace BskCaptureEncoding
{
FString SafePathSegment(FString Value)
{
    for (const TCHAR Invalid : FString(TEXT("/\\:*?\"<>|"))) Value.ReplaceCharInline(Invalid, TEXT('_'));
    Value.TrimStartAndEndInline();
    return Value.IsEmpty() ? TEXT("unnamed") : Value;
}

int64 UnixTimeNanoseconds()
{
    static const FDateTime UnixEpoch(1970, 1, 1);
    return (FDateTime::UtcNow().GetTicks() - UnixEpoch.GetTicks()) * ETimespan::NanosecondsPerTick;
}

void AddBigEndianUint32(TArray<uint8>& Bytes, uint32 Value)
{
    Bytes.Add(static_cast<uint8>((Value >> 24) & 0xff));
    Bytes.Add(static_cast<uint8>((Value >> 16) & 0xff));
    Bytes.Add(static_cast<uint8>((Value >> 8) & 0xff));
    Bytes.Add(static_cast<uint8>(Value & 0xff));
}

bool CompressPng(const TArray<FColor>& Pixels, int32 Width, int32 Height, TArray<uint8>& OutBytes)
{
    IImageWrapperModule& Module = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
    const TSharedPtr<IImageWrapper> Wrapper = Module.CreateImageWrapper(EImageFormat::PNG);
    if (!Wrapper.IsValid() || !Wrapper->SetRaw(Pixels.GetData(), Pixels.Num() * sizeof(FColor), Width, Height, ERGBFormat::BGRA, 8)) return false;
    const TArray64<uint8>& Compressed = Wrapper->GetCompressed(-3);
    if (Compressed.Num() > MAX_int32) return false;
    OutBytes.Append(Compressed.GetData(), static_cast<int32>(Compressed.Num()));
    return true;
}

bool CompressJpeg(const TArray<FColor>& Pixels, int32 Width, int32 Height, int32 Quality, TArray<uint8>& OutBytes)
{
    IImageWrapperModule& Module = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
    const TSharedPtr<IImageWrapper> Wrapper = Module.CreateImageWrapper(EImageFormat::JPEG);
    if (!Wrapper.IsValid() || !Wrapper->SetRaw(Pixels.GetData(), Pixels.Num() * sizeof(FColor), Width, Height, ERGBFormat::BGRA, 8)) return false;
    const TArray64<uint8>& Compressed = Wrapper->GetCompressed(FMath::Clamp(Quality, 1, 100));
    if (Compressed.Num() > MAX_int32) return false;
    OutBytes.Append(Compressed.GetData(), static_cast<int32>(Compressed.Num()));
    return true;
}


TArray<TSharedPtr<FJsonValue>> JsonVector(const FVector3d& Value)
{
    return {MakeShared<FJsonValueNumber>(Value.X), MakeShared<FJsonValueNumber>(Value.Y), MakeShared<FJsonValueNumber>(Value.Z)};
}

TArray<TSharedPtr<FJsonValue>> JsonQuaternionWxyz(const FQuat4d& Value)
{
    return {MakeShared<FJsonValueNumber>(Value.W), MakeShared<FJsonValueNumber>(Value.X), MakeShared<FJsonValueNumber>(Value.Y), MakeShared<FJsonValueNumber>(Value.Z)};
}

FQuat4d UnrealToActiveLocalWxyz(FQuat4d Value, bool bMirrorY)
{
    Value.Normalize();
    return bMirrorY ? FQuat4d(-Value.X, Value.Y, -Value.Z, Value.W) : Value;
}

void QuaternionMatrix(const FQuat4d& Q, double Out[3][3])
{
    const double X = Q.X, Y = Q.Y, Z = Q.Z, W = Q.W;
    Out[0][0] = 1.0 - 2.0 * (Y * Y + Z * Z); Out[0][1] = 2.0 * (X * Y - Z * W); Out[0][2] = 2.0 * (X * Z + Y * W);
    Out[1][0] = 2.0 * (X * Y + Z * W); Out[1][1] = 1.0 - 2.0 * (X * X + Z * Z); Out[1][2] = 2.0 * (Y * Z - X * W);
    Out[2][0] = 2.0 * (X * Z - Y * W); Out[2][1] = 2.0 * (Y * Z + X * W); Out[2][2] = 1.0 - 2.0 * (X * X + Y * Y);
}

TArray<TSharedPtr<FJsonValue>> JsonMatrix3(const double Matrix[3][3])
{
    TArray<TSharedPtr<FJsonValue>> Values;
    Values.Reserve(9);
    for (int32 Row = 0; Row < 3; ++Row) for (int32 Column = 0; Column < 3; ++Column) Values.Add(MakeShared<FJsonValueNumber>(Matrix[Row][Column]));
    return Values;
}

}
