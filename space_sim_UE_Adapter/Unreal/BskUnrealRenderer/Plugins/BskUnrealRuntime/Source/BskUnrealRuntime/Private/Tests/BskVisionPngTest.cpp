#if WITH_DEV_AUTOMATION_TESTS
#include "BskVisionPng.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Misc/AutomationTest.h"
#include "Modules/ModuleManager.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBskLosslessVisionPng, "BskUnreal.Vision.FastPngPreservesAllPixels",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskLosslessVisionPng::RunTest(const FString& Parameters)
{
    constexpr int32 Width = 97, Height = 61;
    TArray<FColor> Pixels;
    Pixels.SetNumUninitialized(Width * Height);
    auto& Module = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
    for (bool bOpaque : {false, true})
    {
        for (int32 I = 0; I < Pixels.Num(); ++I)
            Pixels[I] = FColor(I % 256, (I * 17) % 256, (I * 31) % 256, bOpaque ? 255 : (I * 7) % 256);
        const TArray<FColor> Original = Pixels;
        TArray<uint8> Png;
        if (!TestTrue(TEXT("Encode full BGRA"), BskEncodeVisionPng(Pixels, Width, Height, Png))) return false;
        auto Decoder = Module.CreateImageWrapper(EImageFormat::PNG);
        if (!TestTrue(TEXT("Decode PNG"), Decoder->SetCompressed(Png.GetData(), Png.Num()))) return false;
        TArray64<uint8> Raw;
        if (!TestTrue(TEXT("Read BGRA8"), Decoder->GetRaw(ERGBFormat::BGRA, 8, Raw))) return false;
        TestEqual(TEXT("Width"), Decoder->GetWidth(), static_cast<int64>(Width));
        TestEqual(TEXT("Height"), Decoder->GetHeight(), static_cast<int64>(Height));
        TestEqual(TEXT("Native byte count"), Raw.Num(), static_cast<int64>(Pixels.Num() * sizeof(FColor)));
        TestTrue(TEXT("Every decoded channel equals the source"),
            FMemory::Memcmp(Raw.GetData(), Original.GetData(), Raw.Num()) == 0);
        TestTrue(TEXT("Encoder leaves source pixels intact"),
            FMemory::Memcmp(Pixels.GetData(), Original.GetData(), Pixels.Num() * sizeof(FColor)) == 0);
        TestFalse(TEXT("Invalid dimensions fail"), BskEncodeVisionPng(Pixels, 0, Height, Png));
    }
    return true;
}
#endif
