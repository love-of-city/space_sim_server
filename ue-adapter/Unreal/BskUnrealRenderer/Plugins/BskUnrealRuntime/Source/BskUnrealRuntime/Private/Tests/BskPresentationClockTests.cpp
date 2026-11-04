#include "BskPresentationClock.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBskPresentationJitterTest,
    "BskUnreal.Presentation.ContinuousClockAcrossJitter",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskPresentationJitterTest::RunTest(const FString& Parameters)
{
    for (const double SimulationRate : {0.5, 1.0, 4.5})
    {
        FBskPresentationClock Clock;
        int32 Packet = 0;
        double Previous = 0.0, MinimumRate = 100.0, MaximumRate = 0.0;
        constexpr double Tick = 1.0 / 60.0;
        for (int32 Index = 0; Index < 1200; ++Index)
        {
            const double Now = Index * Tick;
            const double Arrival = Packet / 9.0 + (Packet % 2 ? 0.015 : 0.0);
            if (Now >= Arrival)
            {
                Clock.Observe(Packet * SimulationRate / 9.0, Now);
                ++Packet;
            }
            const double Current = Clock.Advance(Tick, Now, 0.1, 0.1);
            TestTrue(TEXT("arrivals cannot move playback backwards"), Current >= Previous);
            if (Now > 3.0)
            {
                const double Rate = (Current - Previous) / Tick;
                MinimumRate = FMath::Min(MinimumRate, Rate);
                MaximumRate = FMath::Max(MaximumRate, Rate);
            }
            Previous = Current;
        }
        TestTrue(TEXT("no packet-boundary crawl"), MinimumRate > SimulationRate * 0.85);
        TestTrue(TEXT("no packet-boundary leap"), MaximumRate < SimulationRate * 1.15);
        TestTrue(TEXT("bounded latency"), FMath::Abs(Previous - 20.0 * SimulationRate) < SimulationRate * 0.5);
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBskPresentationPauseTest,
    "BskUnreal.Presentation.PauseRewindAndBoundedPrediction",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskPresentationPauseTest::RunTest(const FString& Parameters)
{
    FBskPresentationClock Clock;
    Clock.Observe(100.0, 0.0);
    Clock.Observe(100.5, 0.1);
    Clock.Observe(101.0, 0.2);
    double Position = 0.0;
    for (int32 Index = 0; Index < 600; ++Index)
        Position = Clock.Advance(1.0 / 60.0, 0.2 + Index / 60.0, 0.1, 0.1);
    TestTrue(TEXT("a lost sender cannot predict indefinitely"), FMath::IsNearlyEqual(Position, 101.5, 1.e-9));
    TestFalse(TEXT("a repeated paused timestamp anchors the exact received state"), Clock.Observe(101.0, 11.0));
    TestEqual(TEXT("a paused state stays exact"), Clock.Advance(1.0, 12.0, 0.1, 0.1), 101.0);
    TestFalse(TEXT("rewind starts a fresh timeline"), Clock.Observe(20.0, 13.0));
    TestEqual(TEXT("no extrapolation from an earlier session"), Clock.GetRate(), 0.0);
    TestEqual(TEXT("rewound state stays exact until new frames"), Clock.Advance(1.0, 14.0, 0.1, 0.1), 20.0);
    Clock.Reset();
    TestEqual(TEXT("reset discards playback history"), Clock.Advance(1.0, 15.0, 0.1, 0.1), 0.0);
    Clock.Observe(30.0, 0.0);
    TestFalse(TEXT("long initial model load starts a new preview anchor"), Clock.Observe(30.5, 10.0));
    Clock.Observe(31.0, 10.1);
    TestTrue(TEXT("model loading time is excluded from the rate estimate"), FMath::IsNearlyEqual(Clock.GetRate(), 5.0, 1.e-9));
    TestFalse(TEXT("long stopped stream discards stale playback debt"), Clock.Observe(32.0, 20.0));
    TestEqual(TEXT("resume applies the current authoritative snapshot"), Clock.Advance(0.1, 20.1, 0.1, 0.1), 32.0);
    return true;
}
