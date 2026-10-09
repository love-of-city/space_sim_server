#include "BskCoordinateConverter.h"

FBskCoordinateConverter::FBskCoordinateConverter(double InCentimetersPerMeter, bool bInMirrorLocalY)
    : CentimetersPerMeter(InCentimetersPerMeter > 0.0 ? InCentimetersPerMeter : 100.0)
    , bMirrorLocalY(bInMirrorLocalY)
{
}

FVector3d FBskCoordinateConverter::LocalMetersToUnrealCentimeters(const FVector3d& PositionMeters) const
{
    const double YSign = bMirrorLocalY ? -1.0 : 1.0;
    return FVector3d(PositionMeters.X, YSign * PositionMeters.Y, PositionMeters.Z) * CentimetersPerMeter;
}

FQuat4d FBskCoordinateConverter::ActiveLocalWxyzToUnreal(const FQuat4d& OrientationWxyz) const
{
    FQuat4d Result = OrientationWxyz;
    if (bMirrorLocalY)
    {
        // Conjugating the active rotation matrix by diag(1,-1,1) maps the
        // right-handed wire frame into UE's left-handed, Z-up convention.
        // Quaternion vector parts are axial, hence (x,y,z)->(-x,y,-z).
        Result = FQuat4d(-OrientationWxyz.X, OrientationWxyz.Y, -OrientationWxyz.Z, OrientationWxyz.W);
    }
    Result.Normalize();
    return Result;
}
