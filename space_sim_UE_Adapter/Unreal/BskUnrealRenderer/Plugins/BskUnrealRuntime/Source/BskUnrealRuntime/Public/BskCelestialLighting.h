#pragma once

#include "CoreMinimal.h"

namespace BskCelestialLighting
{
/** Direction travelled by light rays from an ephemeris source to a local target. */
inline FVector DirectionFromSourceToTarget(const FVector& Source, const FVector& Target = FVector::ZeroVector)
{
    return (Target - Source).GetSafeNormal();
}

/** Visible solar-disc fraction behind a spherical occluder, at one receiver.
 * Uses the standard angular-disc overlap approximation for penumbra. All
 * vectors share one frame and length unit; origin shifts cannot affect it.
 * Full umbra/no-overlap tests are geometric, not shadow-map range dependent.
 */
inline double VisibleSourceFraction(
    const FVector3d& Source, double SourceRadius,
    const FVector3d& Occluder, double OccluderRadius,
    const FVector3d& Receiver = FVector3d::ZeroVector)
{
    const FVector3d ToSource = Source - Receiver;
    const FVector3d ToOccluder = Occluder - Receiver;
    const double SourceDistance = ToSource.Length();
    const double OccluderDistance = ToOccluder.Length();
    if (SourceRadius <= 0.0 || OccluderRadius <= 0.0 || SourceDistance <= SourceRadius)
    {
        return 1.0;
    }
    if (OccluderDistance <= OccluderRadius) return 0.0;
    if (OccluderDistance - OccluderRadius >= SourceDistance + SourceRadius) return 1.0;
    const double A = FMath::Asin(FMath::Clamp(SourceRadius / SourceDistance, 0.0, 1.0));
    const double B = FMath::Asin(FMath::Clamp(OccluderRadius / OccluderDistance, 0.0, 1.0));
    const double D = FMath::Acos(FMath::Clamp(
        FVector3d::DotProduct(ToSource / SourceDistance, ToOccluder / OccluderDistance), -1.0, 1.0));
    if (D >= A + B) return 1.0;
    if (D <= FMath::Abs(A - B))
    {
        return B >= A ? 0.0 : FMath::Clamp(1.0 - B * B / (A * A), 0.0, 1.0);
    }
    const double X = FMath::Clamp((D * D + A * A - B * B) / (2.0 * D * A), -1.0, 1.0);
    const double Y = FMath::Clamp((D * D + B * B - A * A) / (2.0 * D * B), -1.0, 1.0);
    const double Product = (-D + A + B) * (D + A - B) * (D - A + B) * (D + A + B);
    const double Overlap = A * A * FMath::Acos(X) + B * B * FMath::Acos(Y)
        - 0.5 * FMath::Sqrt(FMath::Max(0.0, Product));
    return FMath::Clamp(1.0 - Overlap / (UE_DOUBLE_PI * A * A), 0.0, 1.0);
}

/** A scene multiplier is dimensionless, bounded, and may explicitly be zero. */
inline bool IsValidSunlightIntensityScale(const double Scale)
{
    return FMath::IsFinite(Scale) && Scale >= 0.0 && Scale <= 20000.0;
}

/** Scale illumination without modifying light direction, distance, or occultation. */
inline double ScaleIlluminanceLux(const double Illuminance, const double Scale)
{
    return Illuminance * (IsValidSunlightIntensityScale(Scale) ? Scale : 1.0);
}

/** Inverse-square illuminance with a bounded denominator for malformed frames. */
inline double IlluminanceLux(
    const double IlluminanceAtReference,
    const double ReferenceDistanceMeters,
    const double ActualDistanceMeters)
{
    if (IlluminanceAtReference <= 0.0 || ReferenceDistanceMeters <= 0.0 || ActualDistanceMeters <= 0.0)
    {
        return FMath::Max(0.0, IlluminanceAtReference);
    }
    const double Ratio = ReferenceDistanceMeters / ActualDistanceMeters;
    return FMath::Clamp(IlluminanceAtReference * Ratio * Ratio, 0.0, IlluminanceAtReference * 100.0);
}
}
