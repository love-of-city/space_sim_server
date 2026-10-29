#include "BskVisionPng.h"

THIRD_PARTY_INCLUDES_START
#include "png.h"
#include "zlib.h"
THIRD_PARTY_INCLUDES_END

namespace
{
void WriteVisionPng(png_structp Png, png_bytep Bytes, png_size_t Count)
{
    auto* Output = static_cast<TArray<uint8>*>(png_get_io_ptr(Png));
    if (Count > static_cast<png_size_t>(MAX_int32 - Output->Num()))
        png_error(Png, "vision PNG exceeds packet size");
    Output->Append(Bytes, static_cast<int32>(Count));
}
}

// libpng's documented error API uses setjmp, as does UE's PNG wrapper. This
// function keeps only POD locals in that scope and explicitly frees both exits.
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable:4611)
#endif
bool BskEncodeVisionPng(const TArray<FColor>& Pixels, int32 Width, int32 Height, TArray<uint8>& Output)
{
    Output.Reset();
    if (Width <= 0 || Height <= 0 || static_cast<int64>(Width) * Height != Pixels.Num()) return false;
    // Scene RGB normally has opaque alpha. PNG RGB can represent exactly the
    // same pixels (decoders restore alpha=255), without filtering/compressing a
    // redundant fourth channel. Retain RGBA for any non-opaque source pixel.
    bool bOpaque = true;
    for (const FColor& Pixel : Pixels)
        if (Pixel.A != 255) { bOpaque = false; break; }
    png_structp Png = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!Png) return false;
    png_infop Info = png_create_info_struct(Png);
    if (!Info)
    {
        png_destroy_write_struct(&Png, nullptr);
        return false;
    }
    // Use a raw row table in this setjmp scope; both exit paths free it.
    // There are no C++ owners whose destructors libpng errors could skip.
    auto* Rows = static_cast<png_bytep*>(FMemory::Malloc(static_cast<SIZE_T>(Height) * sizeof(png_bytep)));
    for (int32 Y = 0; Y < Height; ++Y)
        Rows[Y] = reinterpret_cast<png_bytep>(const_cast<FColor*>(Pixels.GetData() + Y * Width));
    if (setjmp(png_jmpbuf(Png)))
    {
        FMemory::Free(Rows);
        png_destroy_write_struct(&Png, &Info);
        Output.Reset();
        return false;
    }
    png_set_write_fn(Png, &Output, WriteVisionPng, nullptr);
    png_set_compression_level(Png, 1);
    // Full-resolution rendered images spend most encoding time in LZ match
    // search. UP-filtered Huffman coding preserves every channel and bounds
    // that CPU work; measured UE images trade about 25% more bytes for 40%
    // less encoder time. Keep normal dataset PNG encoding unchanged.
    png_set_compression_strategy(Png, Z_HUFFMAN_ONLY);
    // The default encoder tries multiple filters on every full-resolution row.
    // UP alone is reversible and avoids that search; compression remains on.
    png_set_filter(Png, 0, PNG_FILTER_UP);
    png_set_IHDR(Png, Info, Width, Height, 8, bOpaque ? PNG_COLOR_TYPE_RGB : PNG_COLOR_TYPE_RGBA,
                 PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
    png_set_rows(Png, Info, Rows);
    png_write_png(Png, Info, PNG_TRANSFORM_BGR | (bOpaque ? PNG_TRANSFORM_STRIP_FILLER_AFTER : 0), nullptr);
    FMemory::Free(Rows);
    png_destroy_write_struct(&Png, &Info);
    return !Output.IsEmpty();
}
#ifdef _MSC_VER
#pragma warning(pop)
#endif
