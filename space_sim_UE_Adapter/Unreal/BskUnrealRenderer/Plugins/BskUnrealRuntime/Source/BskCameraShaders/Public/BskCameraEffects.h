#pragma once
#include "CoreMinimal.h"
class UTextureRenderTarget2D;
// Game-thread entry; queues a GPU pass after the scene capture, before readback.
BSKCAMERASHADERS_API void BskApplyCameraEffect(UTextureRenderTarget2D* Source,
    UTextureRenderTarget2D* Output, bool bFisheye, float FieldOfViewRadians);
