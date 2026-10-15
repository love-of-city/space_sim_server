#pragma once

#include "CoreMinimal.h"
#include "Math/RotationMatrix.h"

// Fits physical reference points without moving/scaling either spacecraft.
// The HUD must disclose auto-framing: screen separation alone is not a ruler.
namespace BskRelativeView
{
    inline FTransform Overview(const FVector& Primary, const FVector& Secondary, double HorizontalFovDegrees, double FixedDistanceCentimeters = 0.0)
    {
        FVector Right = (Secondary - Primary).GetSafeNormal();
        if (Right.IsNearlyZero()) Right = FVector::RightVector;
        FVector Up = FVector::UpVector - Right * FVector::DotProduct(FVector::UpVector, Right);
        if (Up.SizeSquared() < 0.01) Up = FVector::ForwardVector - Right * FVector::DotProduct(FVector::ForwardVector, Right);
        Up.Normalize();
        const FVector Forward = FVector::CrossProduct(Right, Up).GetSafeNormal();
        const double HalfFov = FMath::DegreesToRadians(FMath::Clamp(HorizontalFovDegrees, 10.0, 120.0) * 0.5);
        const double Distance = FixedDistanceCentimeters > 0.0 ? FixedDistanceCentimeters
            : FMath::Max(1000.0, (Secondary - Primary).Length() * 0.65 / FMath::Tan(HalfFov));
        return FTransform(FRotationMatrix::MakeFromXZ(Forward, Up).ToQuat(),
                          (Primary + Secondary) * 0.5 - Forward * Distance);
    }

    inline double RulerMeters(double MaximumMeters)
    {
        if (!FMath::IsFinite(MaximumMeters) || MaximumMeters <= 0.0) return 0.0;
        const double Power = FMath::Pow(10.0, FMath::FloorToDouble(FMath::LogX(10.0, MaximumMeters)));
        const double Mantissa = MaximumMeters / Power;
        return (Mantissa >= 5.0 ? 5.0 : Mantissa >= 2.0 ? 2.0 : 1.0) * Power;
    }

    inline FString Distance(double Meters)
    {
        return Meters >= 1000.0 ? FString::Printf(TEXT("%.3f km"), Meters / 1000.0)
                               : FString::Printf(TEXT("%.2f m"), Meters);
    }
}
