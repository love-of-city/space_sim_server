#include "BskRelativeView.h"
#include "BskFrameParser.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBskRelativeOverviewTest,
    "BskUnreal.Camera.RelativeOverviewPreservesPhysicalScale",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskRelativeOverviewTest::RunTest(const FString& Parameters)
{
    for (const FVector Delta : {FVector(3000000, 0, 0), FVector(0, 0, 3000000), FVector(100000000, 120000000, 100000), FVector::ZeroVector})
    {
        const FVector A(80, -120, 50), B = A + Delta;
        const FTransform Camera = BskRelativeView::Overview(A, B, 60.0);
        TestFalse(TEXT("camera orientation is finite"), Camera.ContainsNaN());
        const FVector CameraA = Camera.InverseTransformPosition(A), CameraB = Camera.InverseTransformPosition(B);
        TestTrue(TEXT("rigid camera transform preserves separation"), FMath::IsNearlyEqual((CameraA - CameraB).Length(), Delta.Length(), 1e-6));
        for (const FVector Point : {CameraA, CameraB})
        {
            TestTrue(TEXT("reference is ahead of camera"), Point.X > 0);
            TestTrue(TEXT("reference fits inside horizontal field of view"), FMath::Abs(Point.Y / Point.X) < FMath::Tan(FMath::DegreesToRadians(30.0)));
            TestTrue(TEXT("both reference points lie on horizontal centerline"), FMath::Abs(Point.Z) < 1e-6);
        }
    }
    TestEqual(TEXT("metric ruler uses a readable physical interval"), BskRelativeView::RulerMeters(4200.0), 2000.0);
    TestEqual(TEXT("metre-scale ruler"), BskRelativeView::RulerMeters(0.8), 0.5);
    const FVector Primary = FVector::ZeroVector, Far(3000000, 0, 0), Near = Far * 0.1;
    const FTransform Initial = BskRelativeView::Overview(Primary, Far, 60.0);
    const double LockedDistance = (Initial.GetLocation() - Far * 0.5).Length();
    const FTransform Later = BskRelativeView::Overview(Primary, Near, 60.0, LockedDistance);
    const FVector A = Initial.InverseTransformPosition(Far), B = Later.InverseTransformPosition(Near);
    TestTrue(TEXT("fixed-scale screen separation shrinks with true range"), FMath::IsNearlyEqual((B.Y / B.X) / (A.Y / A.X), 0.1, 1e-9));
    TestTrue(TEXT("fixed scale retains camera distance"), FMath::IsNearlyEqual((Later.GetLocation() - Near * 0.5).Length(), LockedDistance, 1e-6));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBskRelativeManifestTest,
    "BskUnreal.Protocol.RelativeViewRequiresRealObjects",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskRelativeManifestTest::RunTest(const FString& Parameters)
{
    const FString Prefix = TEXT("{\"protocol\":\"bsk-render/2\",\"type\":\"scene_manifest\",\"session_id\":\"test\",\"revision\":\"1\",\"settings\":{\"relative_view\":");
    const FString Suffix = TEXT("},\"objects\":[{\"object_id\":\"a\"},{\"object_id\":\"b\"}]}");
    FBskRenderMessage Message;
    FString Error;
    TestTrue(TEXT("valid references with explicit centers"), FBskFrameParser::ParseMessageJson(
        Prefix + TEXT("{\"primary_object_id\":\"a\",\"secondary_object_id\":\"b\",\"primary_center_body_m\":[0.1,0.2,0.3]}") + Suffix, Message, Error));
    TestEqual(TEXT("center offset remains in metres"), Message.Manifest.RelativeView.PrimaryCenterBodyMeters.X, 0.1);
    TestFalse(TEXT("missing referenced object rejected"), FBskFrameParser::ParseMessageJson(
        Prefix + TEXT("{\"primary_object_id\":\"a\",\"secondary_object_id\":\"missing\"}") + Suffix, Message, Error));
    TestFalse(TEXT("same object twice rejected"), FBskFrameParser::ParseMessageJson(
        Prefix + TEXT("{\"primary_object_id\":\"a\",\"secondary_object_id\":\"a\"}") + Suffix, Message, Error));
    return true;
}
