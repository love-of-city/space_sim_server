#if WITH_DEV_AUTOMATION_TESTS
#include "BskPreviewCadence.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBskPreviewCadenceTest, "BskUnreal.Preview.FramePacing",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskPreviewCadenceTest::RunTest(const FString& Parameters)
{
    for (const double Rate : { 60.0, 90.0, 120.0 })
    {
        double Next = 0.0;
        int32 Captures = 0;
        // Small alternately late/early ticks must not halve the effective FPS.
        for (int32 Tick = 0; Tick < static_cast<int32>(Rate * 10); ++Tick)
        {
            const double Now = 100.0 + Tick / Rate + (Tick % 2 ? 0.0002 : 0.0);
            if (BskPreviewCaptureDue(Now, Rate, Next)) ++Captures;
        }
        TestEqual(FString::Printf(TEXT("%.0f Hz jitter preserves cadence"), Rate), Captures,
            static_cast<int32>(Rate * 10));
    }
    TestEqual(TEXT("Subscribed camera gets the full target"), BskPreviewRateForViewers(90.0, true, false), 90.0);
    TestEqual(TEXT("Unselected HUD thumbnail has a bounded cost"), BskPreviewRateForViewers(90.0, false, true), 15.0);
    TestEqual(TEXT("No viewers means no preview work"), BskPreviewRateForViewers(90.0, false, false), 0.0);
    TestEqual(TEXT("Thumbnail never exceeds requested FPS"), BskPreviewRateForViewers(10.0, false, true), 10.0);
    double Next = 0.0;
    TestTrue(TEXT("First capture is immediate"), BskPreviewCaptureDue(100.0, 90.0, Next));
    TestFalse(TEXT("Same tick cannot capture twice"), BskPreviewCaptureDue(100.0, 90.0, Next));
    TestTrue(TEXT("Resume after a stall"), BskPreviewCaptureDue(102.0, 90.0, Next));
    TestTrue(TEXT("Missed deadlines skipped"), Next > 102.0 && Next <= 102.0 + 1.0 / 90.0 + 1.e-6);
    TestFalse(TEXT("No catch-up burst"), BskPreviewCaptureDue(102.0, 90.0, Next));
    TestFalse(TEXT("Reject zero rate"), BskPreviewCaptureDue(103.0, 0.0, Next));
    return true;
}
#endif
