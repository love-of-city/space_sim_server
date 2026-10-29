#if WITH_DEV_AUTOMATION_TESTS

#include "BskCaptureRouting.h"
#include "BskCameraProjection.h"
#include "BskFrameParser.h"
#include "BskTcpReceiver.h"
#include "Misc/AutomationTest.h"
#include "BskSceneController.h"
#include "Engine/World.h"
#include "Engine/StaticMesh.h"
#include "Components/StaticMeshComponent.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBskVisionCaptureRoutingTest,
    "BskUnreal.Capture.VisionRouting", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskVisionCaptureRoutingTest::RunTest(const FString& Parameters)
{
    FBskRenderFrame Frame;
    Frame.bCaptureOnDemand = true; // no LeRobot episode
    Frame.SimulationTimeNanoseconds = 100000000;
    FBskCameraDefinition Vision;
    Vision.CameraId = TEXT("left");
    Vision.CaptureConsumer = TEXT("vision");
    Vision.CaptureRateHertz = 10;
    Vision.CaptureProducts = {TEXT("rgb")};
    FBskCaptureSelection Selection;
    TestFalse(TEXT("vision is opt-in"), BskSelectCameraCapture(Frame, Vision, 50, {}, Selection));
    Frame.bVisionCaptureRequested = true;
    Frame.VisionCaptureCameraIds = {TEXT("right")};
    TestFalse(TEXT("camera whitelist is required"), BskSelectCameraCapture(Frame, Vision, 50, {}, Selection));
    Frame.VisionCaptureCameraIds.Add(TEXT("left"));
    TestTrue(TEXT("vision works with empty dataset episode"), BskSelectCameraCapture(Frame, Vision, 50, {}, Selection));
    TestTrue(TEXT("independent purpose"), Selection.Purpose == EBskCapturePurpose::AuthoritativeVision);
    TestEqual(TEXT("dataset rate override is ignored"), Selection.RateHertz, 10.0);
    TestEqual(TEXT("own products"), Selection.Products.Num(), 1);
    Vision.CaptureProducts.Reset();
    TestFalse(TEXT("dataset products override cannot activate vision"),
        BskSelectCameraCapture(Frame, Vision, 50, {TEXT("rgb")}, Selection));
    Vision.CaptureProducts = {TEXT("rgb")};
    Frame.SimulationTimeNanoseconds = 120000000;
    TestFalse(TEXT("off-grid trigger never relabelled"), BskSelectCameraCapture(Frame, Vision, 50, {}, Selection));
    Frame.SimulationTimeNanoseconds = 100000000;
    FBskCameraDefinition Dataset = Vision;
    Dataset.CaptureConsumer = TEXT("dataset");
    TestFalse(TEXT("vision whitelist cannot activate dataset camera"), BskSelectCameraCapture(Frame, Dataset, 50, {}, Selection));
    Frame.CaptureEpisodeId = TEXT("episode");
    TestTrue(TEXT("dataset remains supported"), BskSelectCameraCapture(Frame, Dataset, 50, {}, Selection));
    TestTrue(TEXT("dataset purpose retained"), Selection.Purpose == EBskCapturePurpose::AuthoritativeDataset);
    TestEqual(TEXT("dataset override retained"), Selection.RateHertz, 50.0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBskFlushPlaneGeometryTest,
    "BskUnreal.Capture.FlushPlaneGeometry", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskFlushPlaneGeometryTest::RunTest(const FString& Parameters)
{
    UWorld* World = UWorld::CreateWorld(EWorldType::Game, false);
    ABskSceneController* Controller = World->SpawnActor<ABskSceneController>();
    FBskObjectDefinition Definition;
    Definition.ObjectId = TEXT("marker-cell");
    FBskGeometryDefinition Plane;
    Plane.GeometryId = TEXT("marker-cell/face");
    Plane.Shape = TEXT("plane");
    Plane.DimensionsMeters = FVector3d(.03, .02, 0);
    Definition.Geometries.Add(Plane);
    AActor* Actor = Controller->SpawnManifestObject(Definition);
    TestNotNull(TEXT("flat marker anchor"), Actor);
    if (Actor)
    {
        TArray<UStaticMeshComponent*> Meshes;
        Actor->GetComponents(Meshes);
        if (TestEqual(TEXT("one plane, no box backing"), Meshes.Num(), 1))
        {
            TestNotNull(TEXT("plane mesh resolves"), Meshes[0]->GetStaticMesh().Get());
            if (Meshes[0]->GetStaticMesh())
                TestTrue(TEXT("engine plane selected"), Meshes[0]->GetStaticMesh()->GetPathName().Contains(TEXT("BasicShapes/Plane")));
            const FVector Scale = Meshes[0]->GetRelativeScale3D();
            TestEqual(TEXT("full width in meters"), Scale.X, .03);
            TestEqual(TEXT("full height in meters"), Scale.Y, .02);
            TestEqual(TEXT("non-degenerate normal transform"), Scale.Z, 1.0);
            TestTrue(TEXT("marker has no collision"), Meshes[0]->GetCollisionEnabled() == ECollisionEnabled::NoCollision);
        }
    }
    World->DestroyWorld(false);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBskVisionManifestParserTest,
    "BskUnreal.Capture.VisionManifestParser", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskVisionManifestParserTest::RunTest(const FString& Parameters)
{
    const FString Json = TEXT("{\"protocol\":\"bsk-render/2\",\"type\":\"scene_manifest\",\"session_id\":\"S\",\"revision\":\"1\",\"objects\":[],\"cameras\":[{\"camera_id\":\"old\"},{\"camera_id\":\"left\",\"capture_consumer\":\"vision\",\"resolution\":[1920,1200],\"capture_rate_hz\":10,\"exposure_time_s\":0.002,\"aperture_fnumber\":4,\"capture_products\":[\"rgb\"]}]}");
    FTCHARToUTF8 Utf8(*Json);
    TArray<uint8> Packet;
    const uint32 Length = static_cast<uint32>(Utf8.Length());
    for (int32 Shift : {24, 16, 8, 0}) Packet.Add(static_cast<uint8>((Length >> Shift) & 0xff));
    Packet.Append(reinterpret_cast<const uint8*>(Utf8.Get()), Utf8.Length());
    FBskFrameParser Parser;
    TArray<FBskRenderMessage> Messages;
    FString Error;
    TestTrue(*Error, Parser.AppendMessages(Packet.GetData(), Packet.Num(), Messages, Error));
    if (TestEqual(TEXT("one manifest"), Messages.Num(), 1)
        && TestEqual(TEXT("two camera definitions"), Messages[0].Manifest.Cameras.Num(), 2))
    {
        const auto& Legacy = Messages[0].Manifest.Cameras[0];
        const auto& Vision = Messages[0].Manifest.Cameras[1];
        TestEqual(TEXT("old consumer default"), Legacy.CaptureConsumer, FString(TEXT("dataset")));
        TestEqual(TEXT("old exposure default"), Legacy.ExposureTimeSeconds, 0.0);
        TestEqual(TEXT("vision consumer"), Vision.CaptureConsumer, FString(TEXT("vision")));
        TestEqual(TEXT("vision height"), Vision.Resolution.Y, 1200);
        TestEqual(TEXT("vision exposure"), Vision.ExposureTimeSeconds, .002);
        TestEqual(TEXT("vision aperture"), Vision.ApertureFNumber, 4.0);
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBskVisionSourceRetentionTest,
    "BskUnreal.Capture.VisionSourceRetention", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskVisionSourceRetentionTest::RunTest(const FString& Parameters)
{
    FBskTcpReceiver Receiver(TEXT("127.0.0.1"), 0, BskProtocol::DefaultMaxPacketBytes, false);
    for (int64 Id = 0; Id < 4; ++Id)
    {
        FBskRenderFrame Frame;
        Frame.FrameId = Id;
        Frame.bCaptureOnDemand = true;
        Frame.bVisionCaptureRequested = Id == 1 || Id == 2;
        if (Frame.bVisionCaptureRequested) Frame.VisionCaptureCameraIds = {TEXT("left"), TEXT("right")};
        Receiver.PublishLatest(MoveTemp(Frame));
    }
    FBskRenderFrame Frame;
    TestFalse(TEXT("capture capacity gates vision source"), Receiver.ConsumeForCapture(Frame, false));
    TestTrue(TEXT("first vision retained"), Receiver.ConsumeForCapture(Frame, true));
    TestEqual(TEXT("first source id"), Frame.FrameId, int64(1));
    TestTrue(TEXT("second vision retained"), Receiver.ConsumeForCapture(Frame, true));
    TestEqual(TEXT("second source id"), Frame.FrameId, int64(2));
    TestTrue(TEXT("preview remains separate"), Receiver.ConsumeForCapture(Frame, false));
    TestEqual(TEXT("latest preview"), Frame.FrameId, int64(3));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBskVisionFrameParserTest,
    "BskUnreal.Capture.VisionFrameParser", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskVisionFrameParserTest::RunTest(const FString& Parameters)
{
    const FString Json = TEXT("{\"protocol\":\"bsk-render/2\",\"type\":\"frame\",\"session_id\":\"S\",\"frame_id\":\"5\",\"sim_time_ns\":\"100000000\",\"capture_episode_id\":\"\",\"vision_capture_requested\":true,\"vision_capture_camera_ids\":[\"left\",\"right\"],\"origin_N_m\":[0,0,0],\"objects\":[]}");
    FTCHARToUTF8 Utf8(*Json);
    TArray<uint8> Packet;
    const uint32 Length = static_cast<uint32>(Utf8.Length());
    for (int32 Shift : {24, 16, 8, 0}) Packet.Add(static_cast<uint8>((Length >> Shift) & 0xff));
    Packet.Append(reinterpret_cast<const uint8*>(Utf8.Get()), Utf8.Length());
    FBskFrameParser Parser;
    TArray<FBskRenderMessage> Messages;
    FString Error;
    TestTrue(*Error, Parser.AppendMessages(Packet.GetData(), Packet.Num(), Messages, Error));
    if (TestEqual(TEXT("one source frame"), Messages.Num(), 1))
    {
        TestTrue(TEXT("vision request parsed"), Messages[0].Frame.bVisionCaptureRequested);
        TestEqual(TEXT("both camera ids"), Messages[0].Frame.VisionCaptureCameraIds.Num(), 2);
        TestTrue(TEXT("dataset episode remains empty"), Messages[0].Frame.CaptureEpisodeId.IsEmpty());
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBskIndependentCameraProjectionTest,
    "BskUnreal.Capture.IndependentCameraProjection", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskIndependentCameraProjectionTest::RunTest(const FString& Parameters)
{
    FBskCameraDefinition Camera;
    Camera.Resolution = FIntPoint(1920, 1200);
    Camera.FieldOfViewRadians = FMath::DegreesToRadians(100.0);
    Camera.VerticalFieldOfViewRadians = FMath::DegreesToRadians(100.0);
    const FMatrix Projection = BskIndependentCameraProjection(Camera, 1.0);
    const double Edge = FMath::Tan(Camera.FieldOfViewRadians * .5);
    TestTrue(TEXT("horizontal 50 degree edge projects to raster boundary"),
        FMath::IsNearlyEqual(Projection.M[0][0] * Edge, 1.0, 1e-12));
    TestTrue(TEXT("vertical 50 degree edge projects to raster boundary despite 16:10 raster"),
        FMath::IsNearlyEqual(Projection.M[1][1] * Edge, 1.0, 1e-12));
    TestTrue(TEXT("metadata uses independent vertical focal length"),
        FMath::IsNearlyEqual(BskCameraFocalY(Camera, Camera.Resolution), 600.0 / Edge, 1e-9));
    Camera.VerticalFieldOfViewRadians = 0;
    TestTrue(TEXT("legacy cameras retain square pixels"),
        FMath::IsNearlyEqual(BskCameraFocalY(Camera, Camera.Resolution), 960.0 / Edge, 1e-9));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBskVerticalCameraManifestTest,
    "BskUnreal.Capture.VerticalCameraManifest", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskVerticalCameraManifestTest::RunTest(const FString& Parameters)
{
    const FString Json = TEXT("{\"protocol\":\"bsk-render/2\",\"type\":\"scene_manifest\",\"session_id\":\"S\",\"revision\":\"1\",\"objects\":[],\"cameras\":[{\"camera_id\":\"old\"},{\"camera_id\":\"wrist\",\"field_of_view_rad\":1.7453292519943295,\"vertical_field_of_view_rad\":1.7453292519943295,\"resolution\":[1920,1200]}]}");
    FTCHARToUTF8 Utf8(*Json);
    TArray<uint8> Packet;
    const uint32 Length = static_cast<uint32>(Utf8.Length());
    for (int32 Shift : {24, 16, 8, 0}) Packet.Add(static_cast<uint8>((Length >> Shift) & 0xff));
    Packet.Append(reinterpret_cast<const uint8*>(Utf8.Get()), Utf8.Length());
    FBskFrameParser Parser;
    TArray<FBskRenderMessage> Messages;
    FString Error;
    TestTrue(*Error, Parser.AppendMessages(Packet.GetData(), Packet.Num(), Messages, Error));
    if (Messages.Num() == 1 && Messages[0].Manifest.Cameras.Num() == 2)
    {
        TestEqual(TEXT("old vertical FOV remains derived"), Messages[0].Manifest.Cameras[0].VerticalFieldOfViewRadians, 0.0);
        TestTrue(TEXT("explicit vertical FOV parsed"), FMath::IsNearlyEqual(
            Messages[0].Manifest.Cameras[1].VerticalFieldOfViewRadians, FMath::DegreesToRadians(100.0), 1e-12));
    }
    else AddError(TEXT("missing camera definitions"));
    return true;
}


#endif
