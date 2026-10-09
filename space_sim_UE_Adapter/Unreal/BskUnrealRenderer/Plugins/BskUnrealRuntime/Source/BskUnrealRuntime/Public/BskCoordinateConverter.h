#pragma once

#include "CoreMinimal.h"

class BSKUNREALRUNTIME_API FBskCoordinateConverter
{
public:
    FBskCoordinateConverter(double InCentimetersPerMeter = 100.0, bool bInMirrorLocalY = true);

    FVector3d LocalMetersToUnrealCentimeters(const FVector3d& PositionMeters) const;
    FQuat4d ActiveLocalWxyzToUnreal(const FQuat4d& OrientationWxyz) const;

    double GetCentimetersPerMeter() const { return CentimetersPerMeter; }
    bool MirrorsLocalY() const { return bMirrorLocalY; }

private:
    double CentimetersPerMeter;
    bool bMirrorLocalY;
};
