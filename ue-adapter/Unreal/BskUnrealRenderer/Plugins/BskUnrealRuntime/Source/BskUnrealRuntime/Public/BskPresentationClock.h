#pragma once

#include "CoreMinimal.h"

// A continuous preview clock. Packet arrivals update the rate estimate, never
// restart the playhead. Authoritative simulation/capture timestamps bypass it.
class FBskPresentationClock
{
public:
    void Reset() { *this = FBskPresentationClock{}; }

    // False means a new timeline (including a paused/rewound recording).
    bool Observe(double SimulationSeconds, double ArrivalSeconds)
    {
        const double WallDelta = ArrivalSeconds - LastArrivalSeconds;
        if (!bInitialized || SimulationSeconds <= LatestSeconds
            || WallDelta > FMath::Max(0.5, MeanWallInterval * 4.0))
        {
            Reset();
            bInitialized = true;
            LatestSeconds = PositionSeconds = FirstSeconds = SimulationSeconds;
            LastArrivalSeconds = ArrivalSeconds;
            return false;
        }
        const double SampleInterval = FMath::Max(WallDelta, 1.0 / 1000.0);
        const double SimDelta = SimulationSeconds - LatestSeconds;
        if (Rate <= 0.0)
        {
            Rate = SimDelta / SampleInterval;
            MeanWallInterval = SampleInterval;
            MeanSimInterval = SimDelta;
        }
        else
        {
            // Average intervals before dividing; averaging individual rates
            // biases the clock upward when packets arrive in short bursts.
            constexpr double Weight = 0.08;
            MeanWallInterval = FMath::Lerp(MeanWallInterval, SampleInterval, Weight);
            MeanSimInterval = FMath::Lerp(MeanSimInterval, SimDelta, Weight);
            Rate = MeanSimInterval / MeanWallInterval;
        }
        LatestSeconds = SimulationSeconds;
        LastArrivalSeconds = ArrivalSeconds;
        return true;
    }

    double Advance(double DeltaSeconds, double NowSeconds, double DelaySeconds, double MaxExtrapolationSeconds)
    {
        if (!bInitialized || Rate <= 0.0 || DeltaSeconds <= 0.0) return PositionSeconds;
        // Keep at least 1.5 source intervals for irregular low-rate flight
        // packets, with a bounded delay. At 30 Hz the configured delay wins.
        const double Delay = FMath::Max(DelaySeconds, FMath::Min(0.5, 1.5 * MeanWallInterval));
        if (!bPlaying)
        {
            if (LatestSeconds - FirstSeconds < Delay * Rate) return PositionSeconds;
            bPlaying = true;
        }
        const double SinceArrival = FMath::Max(0.0, NowSeconds - LastArrivalSeconds);
        const double Desired = LatestSeconds + FMath::Min(SinceArrival, MaxExtrapolationSeconds) * Rate - Delay * Rate;
        // Gently recover lag instead of jumping on every packet. Keeping the
        // correction small prevents the old fast/slow/fast interpolation cycle.
        const double Correction = FMath::Clamp(1.0 + (Desired - PositionSeconds) / FMath::Max(Rate, 1.e-9), 0.9, 1.1);
        const double Ceiling = LatestSeconds + FMath::Max(0.0, MaxExtrapolationSeconds) * Rate;
        PositionSeconds = FMath::Max(PositionSeconds,
            FMath::Min(Ceiling, PositionSeconds + FMath::Min(DeltaSeconds, 0.25) * Rate * Correction));
        return PositionSeconds;
    }

    double GetRate() const { return Rate; }

private:
    bool bInitialized = false;
    bool bPlaying = false;
    double FirstSeconds = 0.0;
    double LatestSeconds = 0.0;
    double PositionSeconds = 0.0;
    double LastArrivalSeconds = 0.0;
    double MeanWallInterval = 0.0;
    double MeanSimInterval = 0.0;
    double Rate = 0.0;
};
