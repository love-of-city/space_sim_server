#include "BskPixelStreamingCompatibility.h"
#include "Runtime/Launch/Resources/Version.h"
#include "VideoProducerRenderTarget.h"

#if ENGINE_MAJOR_VERSION != 5 || ENGINE_MINOR_VERSION != 6
#error BskPixelStreamingCompatibility requires validation against this Unreal Engine version.
#endif
TSharedPtr<IPixelStreaming2VideoProducer> BskCreateRenderTargetVideoProducer(UTextureRenderTarget2D* Target)
{
    return UE::PixelStreaming2::FVideoProducerRenderTarget::Create(Target);
}
