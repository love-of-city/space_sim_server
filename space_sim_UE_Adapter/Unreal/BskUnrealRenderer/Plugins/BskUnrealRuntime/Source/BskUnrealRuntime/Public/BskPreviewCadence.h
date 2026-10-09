#pragma once

#include "CoreMinimal.h"

// Wall-clock preview scheduling only; never used for authoritative data capture.
// Advance from the previous deadline, NOT from the (jittery) tick arrival time.
// Otherwise a 90 Hz capture driven by ~90 Hz ticks regularly skips every other
// tick. Missed periods are skipped, not rendered in an expensive catch-up burst.
inline bool BskPreviewCaptureDue(double NowSeconds, double RateHertz, double& NextSeconds)
{
    if (!FMath::IsFinite(NowSeconds) || !FMath::IsFinite(RateHertz) || RateHertz <= 0.0)
        return false;
    const double Period = 1.0 / RateHertz;
    if (NextSeconds <= 0.0) NextSeconds = NowSeconds;
    if (NowSeconds + 1.e-6 < NextSeconds) return false;
    const double Periods = FMath::FloorToDouble(FMath::Max(0.0, NowSeconds - NextSeconds) / Period) + 1.0;
    NextSeconds += Periods * Period;
    return true;
}

// Full-rate frames for subscribed cameras; only main-view spectators need HUD
// thumbnails. No camera preview work when neither view has a subscriber.
inline double BskPreviewRateForViewers(double TargetRate, bool bCameraViewed, bool bViewportViewed)
{
    return bCameraViewed ? TargetRate : (bViewportViewed ? FMath::Min(15.0, TargetRate) : 0.0);
}
