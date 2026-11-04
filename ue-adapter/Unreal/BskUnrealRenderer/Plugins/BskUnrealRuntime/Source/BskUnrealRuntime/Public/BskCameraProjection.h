#pragma once

#include "BskProtocolTypes.h"
#include "Math/PerspectiveMatrix.h"

/** Independent angular spans; raster dimensions do not alter these optics. */
inline FMatrix BskIndependentCameraProjection(const FBskCameraDefinition& Camera, double NearPlane)
{
    return FReversedZPerspectiveMatrix(Camera.FieldOfViewRadians * .5,
        Camera.VerticalFieldOfViewRadians * .5, 1.0, 1.0, NearPlane, NearPlane);
}

inline double BskCameraFocalY(const FBskCameraDefinition& Camera, const FIntPoint& Resolution)
{
    return Camera.VerticalFieldOfViewRadians > 0
        ? .5 * Resolution.Y / FMath::Tan(.5 * Camera.VerticalFieldOfViewRadians)
        : .5 * Resolution.X / FMath::Tan(.5 * Camera.FieldOfViewRadians);
}
