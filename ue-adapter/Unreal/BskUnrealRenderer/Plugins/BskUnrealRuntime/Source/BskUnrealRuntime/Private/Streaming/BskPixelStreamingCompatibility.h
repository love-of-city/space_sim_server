#pragma once
#include "CoreMinimal.h"
class IPixelStreaming2VideoProducer;
class UTextureRenderTarget2D;
// UE 5.6 internal API is confined here; runtime callers use the public interface.
TSharedPtr<IPixelStreaming2VideoProducer> BskCreateRenderTargetVideoProducer(UTextureRenderTarget2D* Target);
