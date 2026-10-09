#pragma once

#include "CoreMinimal.h"

// Match the server absolute rational-nanosecond grid: physics 240 Hz, IK 120 Hz,
// render 30 Hz. Never accumulate a rounded period or use wall-clock preview time.
inline bool BskLeRobotSample(int64 SimulationTimeNanoseconds, double RateHertz, int64& OutSampleIndex)
{
    const int32 Fps = FMath::RoundToInt(RateHertz);
    if (!FMath::IsNearlyEqual(RateHertz, static_cast<double>(Fps), 1.e-9) ||
        !(Fps == 1 || Fps == 2 || Fps == 5 || Fps == 10 || Fps == 30) ||
        SimulationTimeNanoseconds < 0)
    {
        return false;
    }
    // Avoid multiplying the full nanosecond timestamp (long-running scenes).
    const int64 WholeSeconds = SimulationTimeNanoseconds / 1000000000LL;
    const int64 Fraction = SimulationTimeNanoseconds % 1000000000LL;
    const int64 TickInSecond = (Fraction * Fps + 500000000LL) / 1000000000LL;
    OutSampleIndex = WholeSeconds * Fps + TickInSecond;
    const int64 RoundedFraction = (TickInSecond * 1000000000LL + Fps / 2) / Fps;
    return Fraction == RoundedFraction;
}
