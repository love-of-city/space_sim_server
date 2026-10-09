#pragma once

#include "CoreMinimal.h"

// Browser-relative mouse displacements (not pixels in the encoded video) and
// complete held-key state. Deliberately independent of legacy PlayerInput,
// EnhancedInput mappings, FOV scaling, mouse smoothing and render frequency.
struct BSKUNREALRUNTIME_API FBskCameraInput
{
    bool bActive = false;
    bool bBoost = false;
    FVector Axes = FVector::ZeroVector; // forward, right, world-up
    FVector2D Look = FVector2D::ZeroVector;

    static constexpr double DegreesPerMouseUnit = 0.12;
    static constexpr double SpeedCentimetersPerSecond = 100.0;
    static constexpr double WatchdogSeconds = 0.5;

    static bool Parse(const FString& Json, FBskCameraInput& Out);
    FQuat ApplyLook(const FQuat& Rotation) const;
    FVector MovementDelta(const FQuat& Rotation, double DeltaSeconds) const;
};
