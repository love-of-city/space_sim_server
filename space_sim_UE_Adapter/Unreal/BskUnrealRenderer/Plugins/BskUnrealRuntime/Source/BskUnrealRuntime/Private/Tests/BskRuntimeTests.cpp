#if WITH_DEV_AUTOMATION_TESTS

#include "BskCoordinateConverter.h"
#include "BskCelestialLighting.h"
#include "BskFrameParser.h"
#include "BskRenderExtension.h"
#include "BskRenderWorldSubsystem.h"
#include "BskSceneController.h"
#include "BskTcpReceiver.h"
#include "Engine/StaticMesh.h"
#include "Engine/DirectionalLight.h"
#include "Components/LightComponent.h"
#include "Engine/Texture2D.h"
#include "Materials/MaterialInterface.h"
#include "Misc/AutomationTest.h"

namespace
{
TArray<uint8> EncodePacket(const FString& Json)
{
    FTCHARToUTF8 Utf8(*Json);
    const uint32 Length = static_cast<uint32>(Utf8.Length());
    TArray<uint8> Packet;
    Packet.Reserve(static_cast<int32>(Length) + 4);
    Packet.Add(static_cast<uint8>((Length >> 24) & 0xff));
    Packet.Add(static_cast<uint8>((Length >> 16) & 0xff));
    Packet.Add(static_cast<uint8>((Length >> 8) & 0xff));
    Packet.Add(static_cast<uint8>(Length & 0xff));
    Packet.Append(reinterpret_cast<const uint8*>(Utf8.Get()), Utf8.Length());
    return Packet;
}

class FCountingRenderExtension final : public IBskRenderExtension
{
public:
    virtual FName GetExtensionName() const override { return TEXT("Automation.CountingExtension"); }
    virtual void OnManifestApplied(const FBskSceneManifest&) override { ++ManifestCount; }
    virtual void OnEventApplied(const FBskRenderEvent&) override { ++EventCount; }
    virtual void OnFrameApplied(const FBskRenderFrame&) override { ++FrameCount; }

    int32 ManifestCount = 0;
    int32 EventCount = 0;
    int32 FrameCount = 0;
};

class FTestCaptureProvider final : public IBskCaptureProvider
{
public:
    virtual bool RegisterCamera(const FString&, TWeakObjectPtr<AActor>) override { ++CameraCount; return true; }
    virtual bool RequestCapture(const FBskCaptureRequest&, FString&) override { ++CaptureCount; return true; }
    virtual void Shutdown() override { bShutdown = true; }

    int32 CameraCount = 0;
    int32 CaptureCount = 0;
    bool bShutdown = false;
};
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FBskEphemerisLightingTest,
    "BskUnreal.Celestial.EphemerisLightingDirectionAndDistance",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskEphemerisLightingTest::RunTest(const FString& Parameters)
{
    const FVector Source(100.0, -50.0, 25.0);
    const FVector Expected = (-Source).GetSafeNormal();
    TestTrue(TEXT("light rays travel from the source toward the local scene"),
        BskCelestialLighting::DirectionFromSourceToTarget(Source).Equals(Expected, 1.0e-12));
    TestEqual(TEXT("reference-distance illuminance"),
        BskCelestialLighting::IlluminanceLux(8.0, 100.0, 100.0), 8.0);
    TestEqual(TEXT("twice distance gives quarter illuminance"),
        BskCelestialLighting::IlluminanceLux(8.0, 100.0, 200.0), 2.0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FBskCelestialLightManifestTest,
    "BskUnreal.Protocol.CelestialLightManifest",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskCelestialLightManifestTest::RunTest(const FString& Parameters)
{
    FBskRenderMessage Message;
    FString Error;
    const bool bParsed = FBskFrameParser::ParseMessageJson(
        TEXT("{\"protocol\":\"bsk-render/2\",\"type\":\"scene_manifest\",\"session_id\":\"ephemeris\",\"revision\":\"1\",\"objects\":[],\"celestial_bodies\":[{\"body_id\":\"sol\",\"display_name\":\"Primary Star\",\"visual_role\":\"star\",\"luminous\":true,\"drives_directional_light\":true,\"light_color_rgb\":[1,0.97,0.9],\"light_illuminance_lux_at_reference_distance\":8,\"light_reference_distance_m\":149597870693}],\"visuals\":[],\"cameras\":[],\"settings\":{\"fill_light_intensity_lux\":0}}"),
        Message,
        Error);
    TestTrue(*Error, bParsed);
    TestEqual(TEXT("one celestial body"), Message.Manifest.CelestialBodies.Num(), 1);
    if (Message.Manifest.CelestialBodies.Num() == 1)
    {
        const FBskCelestialBodyDefinition& Star = Message.Manifest.CelestialBodies[0];
        TestEqual(TEXT("generic ID does not need to be sun"), Star.BodyId, FString(TEXT("sol")));
        TestEqual(TEXT("explicit star role"), Star.VisualRole, FString(TEXT("star")));
        TestTrue(TEXT("explicit ephemeris light driver"), Star.bDrivesDirectionalLight);
        TestTrue(TEXT("light colour retained"), Star.LightColorRgb.Equals(FVector3d(1.0, 0.97, 0.9)));
        TestEqual(TEXT("reference illuminance"), Star.LightIlluminanceLuxAtReferenceDistance, 8.0);
    }
    TestEqual(TEXT("manifest disables readability fill"), Message.Manifest.FillLightIntensityLux, 0.0);
    return true;
}


IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FBskSceneSunlightIntensityTest,
    "BskUnreal.Celestial.SceneSunlightIntensity",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskSceneSunlightIntensityTest::RunTest(const FString& Parameters)
{
    ABskSceneController* Controller = NewObject<ABskSceneController>();
    Controller->SunLight = NewObject<ADirectionalLight>();
    Controller->CelestialSunLight = NewObject<ADirectionalLight>();
    Controller->FillLight = NewObject<ADirectionalLight>();
    Controller->SunIntensityLux = 8.0;
    Controller->SunIlluminanceScale = 1.5; // Keep the renderer's calibration independent.
    Controller->FillLightIntensityLux = 1.25;
    FBskRenderMessage Message;
    FString Error;
    const FString Prefix = TEXT("{\"protocol\":\"bsk-render/2\",\"type\":\"scene_manifest\",\"revision\":\"1\",\"objects\":[],\"settings\":{");
    for (const double Scale : {0.0, 0.5, 1.0, 2.0, 12500.0, 20000.0})
    {
        Error.Reset();
        const FString Json = Prefix + FString::Printf(TEXT("\"sunlight_intensity_scale\":%.4f}}"), Scale);
        TestTrue(*Error, FBskFrameParser::ParseMessageJson(Json, Message, Error));
        TestEqual(TEXT("scale retained by parser"), Message.Manifest.SunlightIntensityScale, Scale);
        Controller->ConfigureManifestLighting(Message.Manifest);
        TestEqual(TEXT("local sunlight is scaled, including zero"),
            Controller->SunLight->GetLightComponent()->Intensity, static_cast<float>(8.0 * Scale));
        TestEqual(TEXT("planetary sunlight uses same scene multiplier"),
            Controller->CelestialSunLight->GetLightComponent()->Intensity, static_cast<float>(8.0 * Scale));
        TestEqual(TEXT("fill light is not scaled"), Controller->FillLight->GetLightComponent()->Intensity, 1.25f);
        TestEqual(TEXT("renderer calibration is preserved"), Controller->SunIlluminanceScale, 1.5);
        const double AtTwiceDistance = BskCelestialLighting::IlluminanceLux(8.0, 100.0, 200.0);
        const double Scaled = BskCelestialLighting::ScaleIlluminanceLux(AtTwiceDistance * 1.5, Scale);
        TestEqual(TEXT("inverse-square distance and calibration still apply"), Scaled, 3.0 * Scale);
        TestEqual(TEXT("eclipse still extinguishes local sunlight"), Scaled * 0.0, 0.0);
    }
    Error.Reset();
    TestTrue(TEXT("old manifest without multiplier is accepted"),
        FBskFrameParser::ParseMessageJson(Prefix + TEXT("}}"), Message, Error));
    Controller->ConfigureManifestLighting(Message.Manifest);
    TestEqual(TEXT("old manifest resets previous scene multiplier"), Controller->SceneSunlightIntensityScale, 1.0);
    TestEqual(TEXT("default lighting restored"), Controller->SunLight->GetLightComponent()->Intensity, 8.0f);
    for (const FString Invalid : {TEXT("-1"), TEXT("20000.1"), TEXT("\"2\""), TEXT("null"), TEXT("true")})
    {
        Error.Reset();
        TestFalse(*FString::Printf(TEXT("invalid multiplier %s rejected"), *Invalid), FBskFrameParser::ParseMessageJson(
            Prefix + TEXT("\"sunlight_intensity_scale\":") + Invalid + TEXT("}}"), Message, Error));
    }
    return true;
}


IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FBskSolarOccultationTest,
    "BskUnreal.Celestial.SolarOccultationAndRebasing",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskSolarOccultationTest::RunTest(const FString& Parameters)
{
    constexpr double AU = 149597870700.0;
    constexpr double SunRadius = 695000000.0;
    constexpr double EarthRadius = 6378136.6;
    constexpr double EarthDistance = EarthRadius + 500000.0;
    const FVector3d Sun(AU, 0.0, 0.0);
    const FVector3d NightEarth(EarthDistance, 0.0, 0.0);
    TestEqual(TEXT("night-side Earth fully occults Sun"),
        BskCelestialLighting::VisibleSourceFraction(Sun, SunRadius, NightEarth, EarthRadius), 0.0);
    TestEqual(TEXT("day-side Earth is behind observer"),
        BskCelestialLighting::VisibleSourceFraction(Sun, SunRadius, -NightEarth, EarthRadius), 1.0);
    TestEqual(TEXT("body behind Sun cannot eclipse it"),
        BskCelestialLighting::VisibleSourceFraction(Sun, SunRadius, FVector3d(2.0 * AU, 0, 0), EarthRadius), 1.0);

    const double EarthAngle = FMath::Asin(EarthRadius / EarthDistance);
    const double SunAngle = FMath::Asin(SunRadius / AU);
    const FVector3d LimbEarth(EarthDistance * FMath::Cos(EarthAngle), EarthDistance * FMath::Sin(EarthAngle), 0);
    const double Half = BskCelestialLighting::VisibleSourceFraction(Sun, SunRadius, LimbEarth, EarthRadius);
    TestTrue(TEXT("Earth limb produces partial eclipse"), Half > 0.45 && Half < 0.55);
    const FVector3d Shift(7000000, -12000000, 3300000);
    TestTrue(TEXT("receiver/source/occluder translation preserves shadow fraction"),
        FMath::IsNearlyEqual(Half, BskCelestialLighting::VisibleSourceFraction(
            Sun + Shift, SunRadius, LimbEarth + Shift, EarthRadius, Shift), 1.0e-7));
    double Previous = 0.0;
    for (int32 Index = 0; Index <= 100; ++Index)
    {
        const double Angle = EarthAngle - 1.01 * SunAngle + 2.02 * SunAngle * Index / 100.0;
        const FVector3d Earth(EarthDistance * FMath::Cos(Angle), EarthDistance * FMath::Sin(Angle), 0);
        const double Visible = BskCelestialLighting::VisibleSourceFraction(Sun, SunRadius, Earth, EarthRadius);
        TestTrue(TEXT("penumbra remains finite/bounded/monotonic"),
            FMath::IsFinite(Visible) && Visible >= 0.0 && Visible <= 1.0 && Visible + 1.0e-7 >= Previous);
        Previous = Visible;
    }
    TestEqual(TEXT("penumbra exits to full daylight"), Previous, 1.0);
    const double Annular = BskCelestialLighting::VisibleSourceFraction(
        FVector3d(10000, 0, 0), 10, FVector3d(1000, 0, 0), 0.1);
    TestTrue(TEXT("smaller concentric body leaves visible solar annulus"), FMath::IsNearlyEqual(Annular, 0.99, 1.0e-6));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FBskProtocolFragmentationTest,
    "BskUnreal.Protocol.FragmentedLengthPrefixedJson",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskProtocolFragmentationTest::RunTest(const FString& Parameters)
{
    const FString Json = TEXT("{\"protocol\":\"bsk-isaac-render/1\",\"type\":\"frame\",\"frame_id\":7,\"sim_time_ns\":123,\"origin_N_m\":[7000000,0,0],\"objects\":[{\"name\":\"chaser\",\"position_m\":[0,0,0],\"orientation_wxyz\":[1,0,0,0]}]}");
    const TArray<uint8> Packet = EncodePacket(Json);
    FBskFrameParser Parser;
    TArray<FBskRenderFrame> Frames;
    FString Error;
    TestTrue(TEXT("partial header accepted"), Parser.Append(Packet.GetData(), 2, Frames, Error));
    TestEqual(TEXT("no early frame"), Frames.Num(), 0);
    TestTrue(TEXT("remaining bytes accepted"), Parser.Append(Packet.GetData() + 2, Packet.Num() - 2, Frames, Error));
    TestEqual(TEXT("one frame"), Frames.Num(), 1);
    if (Frames.Num() == 1)
    {
        TestEqual(TEXT("frame id"), Frames[0].FrameId, int64(7));
        TestEqual(TEXT("object name"), Frames[0].Objects[0].Name, FString(TEXT("chaser")));
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FBskProtocolGenericIdentifierTest,
    "BskUnreal.Protocol.GenericIdentifier",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskProtocolGenericIdentifierTest::RunTest(const FString& Parameters)
{
    FBskRenderFrame Frame;
    FString Error;
    const bool bParsed = FBskFrameParser::ParseJson(
        TEXT("{\"protocol\":\"bsk-render/1\",\"type\":\"frame\",\"frame_id\":1,\"sim_time_ns\":2,\"origin_N_m\":[0,0,0],\"objects\":[]}"),
        Frame,
        Error);
    TestTrue(*Error, bParsed);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FBskProtocolV2ManifestTest,
    "BskUnreal.Protocol.V2ManifestGeometryAndInt64",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskProtocolV2ManifestTest::RunTest(const FString& Parameters)
{
    FBskRenderMessage Message;
    FString Error;
    const bool bParsed = FBskFrameParser::ParseMessageJson(
        TEXT("{\"protocol\":\"bsk-render/2\",\"type\":\"scene_manifest\",\"session_id\":\"session-a\",\"revision\":\"9007199254740993\",\"objects\":[{\"object_id\":\"primary/hub\",\"display_name\":\"hub\",\"parent_id\":\"\",\"transform_space\":\"world\",\"geometries\":[{\"geometry_id\":\"primary/hub/geom/0\",\"shape\":\"mesh\",\"dimensions_m\":[1,1,1],\"position_body_m\":[0,0,0],\"orientation_body_from_geometry_wxyz\":[1,0,0,0],\"color_rgba\":[1,0,0,0.5],\"asset_type\":\"static_mesh\",\"asset_path\":\"/Game/BSK/Generated/UR5e/base_0.base_0\",\"scale\":[100,100,100],\"render_role\":\"visual\",\"asset_key\":\"base_0.obj\",\"material_name\":\"black\",\"material_specular\":0.6,\"material_shininess\":0.3,\"material_reflectance\":0.1,\"material_emission\":0.0,\"material_texture_repeat\":[2,3]}]}],\"celestial_bodies\":[],\"visuals\":[],\"cameras\":[],\"settings\":{\"origin_object_id\":\"primary/hub\",\"default_camera_distance_m\":2.6,\"orbit_lines\":false,\"trajectory_history\":false}}"),
        Message,
        Error);
    TestTrue(*Error, bParsed);
    TestEqual(TEXT("message type"), Message.Type, EBskRenderMessageType::SceneManifest);
    TestEqual(TEXT("int64 string remains exact"), Message.Manifest.Revision, int64(9007199254740993ll));
    TestEqual(TEXT("one object"), Message.Manifest.Objects.Num(), 1);
    if (Message.Manifest.Objects.Num() == 1)
    {
        TestEqual(TEXT("stable object id"), Message.Manifest.Objects[0].ObjectId, FString(TEXT("primary/hub")));
        TestEqual(TEXT("one geom"), Message.Manifest.Objects[0].Geometries.Num(), 1);
        if (Message.Manifest.Objects[0].Geometries.Num() == 1)
        {
            const FBskGeometryDefinition& Geometry = Message.Manifest.Objects[0].Geometries[0];
            TestEqual(TEXT("mesh shape"), Geometry.Shape, FString(TEXT("mesh")));
            TestEqual(TEXT("mesh asset"), Geometry.AssetPath, FString(TEXT("/Game/BSK/Generated/UR5e/base_0.base_0")));
            TestTrue(TEXT("mesh component scale"), Geometry.Scale.Equals(FVector3d(100.0)));
            TestEqual(TEXT("render role"), Geometry.RenderRole, FString(TEXT("visual")));
            TestEqual(TEXT("material name"), Geometry.MaterialName, FString(TEXT("black")));
            TestEqual(TEXT("material specular"), Geometry.MaterialSpecular, 0.6);
            TestEqual(TEXT("material shininess"), Geometry.MaterialShininess, 0.3);
            TestTrue(TEXT("material texture repeat"), Geometry.MaterialTextureRepeat.Equals(FVector2d(2.0, 3.0)));
        }
    }
    TestEqual(TEXT("camera distance"), Message.Manifest.DefaultCameraDistanceMeters, 2.6);
    TestFalse(TEXT("orbit lines setting"), Message.Manifest.bOrbitLines);
    TestFalse(TEXT("trajectory history setting"), Message.Manifest.bTrajectoryHistory);
    FBskRenderMessage CameraMessage;
    FString CameraError;
    const bool bCameraParsed = FBskFrameParser::ParseMessageJson(
        TEXT("{\"protocol\":\"bsk-render/2\",\"type\":\"scene_manifest\",\"session_id\":\"camera\",\"revision\":\"1\",\"objects\":[],\"celestial_bodies\":[],\"visuals\":[{\"visual_id\":\"scene/light/key\",\"kind\":\"light\",\"parent_id\":\"\",\"properties\":{\"light_type\":\"spot\",\"target_id\":\"primary/hub\",\"intensity\":0.7,\"cutoff_deg\":45,\"cast_shadows\":true,\"diffuse_rgb\":[0.7,0.6,0.5]}}],\"cameras\":[],\"settings\":{\"default_camera_target\":\"primary/hub\",\"use_scene_lighting\":true,\"headlight_enabled\":true,\"headlight_ambient_rgb\":[0.1,0.1,0.1]}}"),
        CameraMessage,
        CameraError);
    TestTrue(*CameraError, bCameraParsed);
    TestEqual(TEXT("camera target"), CameraMessage.Manifest.DefaultCameraTarget, FString(TEXT("primary/hub")));
    TestTrue(TEXT("scene lighting"), CameraMessage.Manifest.bUseSceneLighting);
    TestTrue(TEXT("headlight enabled"), CameraMessage.Manifest.bHeadlightEnabled);
    TestEqual(TEXT("one scene light"), CameraMessage.Manifest.Visuals.Num(), 1);
    if (CameraMessage.Manifest.Visuals.Num() == 1)
    {
        TestEqual(TEXT("light target"), CameraMessage.Manifest.Visuals[0].LightTargetId, FString(TEXT("primary/hub")));
        TestEqual(TEXT("light cutoff"), CameraMessage.Manifest.Visuals[0].LightCutoffDegrees, 45.0);
    }
    FBskRenderMessage CaptureCameraMessage;
    FString CaptureCameraError;
    const bool bCaptureCameraParsed = FBskFrameParser::ParseMessageJson(
        TEXT("{\"protocol\":\"bsk-render/2\",\"type\":\"scene_manifest\",\"session_id\":\"pip\",\"revision\":\"2\",\"objects\":[],\"celestial_bodies\":[],\"visuals\":[],\"cameras\":[{\"camera_id\":\"sat/camera/wrist\",\"display_name\":\"Wrist\",\"parent_id\":\"sat/wrist\",\"position_body_m\":[0,0,0],\"orientation_body_from_camera_wxyz\":[1,0,0,0],\"field_of_view_rad\":1.2,\"resolution\":[480,270],\"picture_in_picture\":true,\"capture_rate_hz\":15,\"picture_in_picture_slot\":2,\"capture_products\":[\"rgb\"]}],\"settings\":{\"ui\":{\"commands\":[{\"command\":\"mission.pause\",\"label\":\"Pause\",\"payload\":{\"mode\":\"hold\"},\"requires_confirmation\":true}]}}}"),
        CaptureCameraMessage,
        CaptureCameraError);
    TestTrue(*CaptureCameraError, bCaptureCameraParsed);
    TestEqual(TEXT("one PIP camera"), CaptureCameraMessage.Manifest.Cameras.Num(), 1);
    if (CaptureCameraMessage.Manifest.Cameras.Num() == 1)
    {
        const FBskCameraDefinition& Camera = CaptureCameraMessage.Manifest.Cameras[0];
        TestEqual(TEXT("PIP camera display name"), Camera.DisplayName, FString(TEXT("Wrist")));
        TestTrue(TEXT("PIP camera enabled"), Camera.bPictureInPicture);
        TestEqual(TEXT("PIP camera resolution"), Camera.Resolution, FIntPoint(480, 270));
        TestEqual(TEXT("PIP camera capture rate"), Camera.CaptureRateHertz, 15.0);
        TestEqual(TEXT("PIP camera slot"), Camera.PictureInPictureSlot, 2);
        TestEqual(TEXT("camera data products"), Camera.CaptureProducts.Num(), 1);
    }
    for (const FString Product : {FString(TEXT("depth")), FString(TEXT("segmentation"))})
    {
        FBskRenderMessage RejectedMessage;
        FString RejectedError;
        const FString Json = FString::Printf(
            TEXT("{\"protocol\":\"bsk-render/2\",\"type\":\"scene_manifest\",\"session_id\":\"rgb-only\",\"revision\":\"1\",\"objects\":[],\"celestial_bodies\":[],\"visuals\":[],\"cameras\":[{\"camera_id\":\"wrist\",\"capture_products\":[\"rgb\",\"%s\"]}],\"settings\":{}}"), *Product);
        TestFalse(TEXT("removed capture product rejects the manifest"),
            FBskFrameParser::ParseMessageJson(Json, RejectedMessage, RejectedError));
        TestTrue(TEXT("rejection identifies the unsupported capture product"), RejectedError.Contains(Product));
    }
    TestEqual(TEXT("one UI command"), CaptureCameraMessage.Manifest.UiCommands.Num(), 1);
    if (CaptureCameraMessage.Manifest.UiCommands.Num() == 1)
    {
        TestEqual(TEXT("UI command name"), CaptureCameraMessage.Manifest.UiCommands[0].Command, FString(TEXT("mission.pause")));
        TestTrue(TEXT("dangerous command confirmation"), CaptureCameraMessage.Manifest.UiCommands[0].bRequiresConfirmation);
        TestTrue(TEXT("command payload retained"), CaptureCameraMessage.Manifest.UiCommands[0].PayloadJson.Contains(TEXT("hold")));
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FBskUr5eAssetLoadTest,
    "BskUnreal.Assets.UR5eStaticMeshLoad",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskUr5eAssetLoadTest::RunTest(const FString& Parameters)
{
    UStaticMesh* Mesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Game/BSK/Generated/UR5e/base_0.base_0"));
    TestNotNull(TEXT("UR5e base mesh is packaged in the project"), Mesh);
    if (Mesh)
    {
        const FVector Extent = Mesh->GetBounds().BoxExtent;
        TestTrue(TEXT("metres-to-centimetres scale is baked while building LOD0"), Extent.GetMax() > 5.0 && Extent.GetMax() < 10.0);
#if WITH_EDITORONLY_DATA
        TestFalse(TEXT("UR5e mesh does not depend on a Nanite fallback"), Mesh->NaniteSettings.bEnabled);
#endif
        const FStaticMeshRenderData* RenderData = Mesh->GetRenderData();
        TestNotNull(TEXT("UR5e mesh has standard render data"), RenderData);
        if (RenderData && RenderData->LODResources.Num() > 0)
        {
            TestEqual(TEXT("base_0 standard LOD0 preserves every OBJ triangle"), RenderData->LODResources[0].GetNumTriangles(), 2240u);
        }
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FBskProtocolV2FrameTest,
    "BskUnreal.Protocol.V2FrameStringTimestamps",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskProtocolV2FrameTest::RunTest(const FString& Parameters)
{
    FBskRenderFrame Frame;
    FString Error;
    const bool bParsed = FBskFrameParser::ParseJson(
        TEXT("{\"protocol\":\"bsk-render/2\",\"type\":\"frame\",\"session_id\":\"session-a\",\"manifest_revision\":\"4\",\"frame_id\":\"8\",\"sim_time_ns\":\"9007199254740993\",\"wall_time_ns\":\"9007199254740995\",\"origin_N_m\":[7000000,0,0],\"c_LN\":[1,0,0,0,1,0,0,0,1],\"objects\":[{\"object_id\":\"primary/hub\",\"position_m\":[0,0,0],\"orientation_wxyz\":[1,0,0,0]}],\"celestial_bodies\":[],\"visual_states\":[]}"),
        Frame,
        Error);
    TestTrue(*Error, bParsed);
    TestEqual(TEXT("simulation timestamp"), Frame.SimulationTimeNanoseconds, int64(9007199254740993ll));
    TestEqual(TEXT("wall timestamp"), Frame.WallTimeNanoseconds, int64(9007199254740995ll));
    TestEqual(TEXT("object id"), Frame.Objects[0].ObjectId, FString(TEXT("primary/hub")));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FBskProtocolTypedVisualChannelsTest,
    "BskUnreal.Protocol.TypedVisualChannels",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskProtocolTypedVisualChannelsTest::RunTest(const FString& Parameters)
{
    FBskRenderMessage ManifestMessage;
    FString Error;
    const bool bManifestParsed = FBskFrameParser::ParseMessageJson(
        TEXT("{\"protocol\":\"bsk-render/2\",\"type\":\"scene_manifest\",\"session_id\":\"devices\",\"revision\":\"2\",\"objects\":[],\"celestial_bodies\":[],\"visuals\":[{\"visual_id\":\"sat/rw/0\",\"kind\":\"reaction_wheel\",\"parent_id\":\"sat\",\"channel_schema\":{\"omega_rad_s\":{\"type\":\"number\",\"unit\":\"rad/s\"},\"saturated\":{\"type\":\"boolean\"}}}],\"cameras\":[],\"settings\":{}}"),
        ManifestMessage,
        Error);
    TestTrue(*Error, bManifestParsed);
    TestEqual(TEXT("one typed visual"), ManifestMessage.Manifest.Visuals.Num(), 1);
    if (ManifestMessage.Manifest.Visuals.Num() == 1)
    {
        const FBskChannelDefinition* Speed = ManifestMessage.Manifest.Visuals[0].ChannelSchema.Find(TEXT("omega_rad_s"));
        TestNotNull(TEXT("speed schema"), Speed);
        if (Speed) TestEqual(TEXT("speed unit"), Speed->Unit, FString(TEXT("rad/s")));
    }

    FBskRenderFrame Frame;
    Error.Reset();
    const bool bFrameParsed = FBskFrameParser::ParseJson(
        TEXT("{\"protocol\":\"bsk-render/2\",\"type\":\"frame\",\"session_id\":\"devices\",\"manifest_revision\":\"2\",\"frame_id\":\"3\",\"sim_time_ns\":\"4\",\"origin_N_m\":[0,0,0],\"objects\":[],\"visual_states\":[{\"visual_id\":\"sat/rw/0\",\"visible\":true,\"channels\":{\"omega_rad_s\":42.5,\"saturated\":false,\"mode\":\"speed\"}}]}"),
        Frame,
        Error);
    TestTrue(*Error, bFrameParsed);
    TestEqual(TEXT("one visual state"), Frame.VisualStates.Num(), 1);
    if (Frame.VisualStates.Num() == 1)
    {
        const FBskVisualState& State = Frame.VisualStates[0];
        const FBskVisualState::FChannelValue* Speed = State.Channels.Find(TEXT("omega_rad_s"));
        const FBskVisualState::FChannelValue* Saturated = State.Channels.Find(TEXT("saturated"));
        const FBskVisualState::FChannelValue* Mode = State.Channels.Find(TEXT("mode"));
        TestTrue(TEXT("numeric channel"), Speed && FMath::IsNearlyEqual(Speed->Number, 42.5));
        TestTrue(TEXT("boolean channel"), Saturated && Saturated->Type == FBskVisualState::FChannelValue::EType::Boolean && !Saturated->Boolean);
        TestTrue(TEXT("string channel"), Mode && Mode->String == TEXT("speed"));
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FBskCoordinateUnitsAndAxesTest,
    "BskUnreal.Coordinates.UnitsAndAxisMapping",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskCoordinateUnitsAndAxesTest::RunTest(const FString& Parameters)
{
    const FBskCoordinateConverter Converter;
    const FVector3d Result = Converter.LocalMetersToUnrealCentimeters(FVector3d(3.0, -2.0, 4.0));
    TestTrue(TEXT("meters to centimeters and Y mirror"), Result.Equals(FVector3d(300.0, 200.0, 400.0), 1.e-12));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FBskActiveAttitudeTest,
    "BskUnreal.Attitude.ActiveQuaternionAndHandedness",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskActiveAttitudeTest::RunTest(const FString& Parameters)
{
    const double HalfSqrt = FMath::Sqrt(0.5);
    const FQuat4d WireQuaternion(0.0, 0.0, HalfSqrt, HalfSqrt); // wire stores wxyz; FQuat constructor is xyzw
    const FQuat4d UnrealQuaternion = FBskCoordinateConverter().ActiveLocalWxyzToUnreal(WireQuaternion);
    TestTrue(TEXT("quaternion remains normalized"), FMath::IsNearlyEqual(UnrealQuaternion.SizeSquared(), 1.0, 1.e-12));
    TestTrue(TEXT("Y reflection flips a positive RH Z rotation"), FMath::IsNearlyEqual(UnrealQuaternion.Z, -HalfSqrt, 1.e-12));
    const FVector3d Rotated = UnrealQuaternion.RotateVector(FVector3d::UnitX());
    TestTrue(TEXT("active rotation maps UE +X to UE -Y"), Rotated.Equals(-FVector3d::UnitY(), 1.e-12));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FBskOfficialCelestialAssetsTest,
    "BskUnreal.Assets.EpicCelestialVault",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskOfficialCelestialAssetsTest::RunTest(const FString& Parameters)
{
    TestNotNull(
        TEXT("Epic Milky Way vault mesh"),
        LoadObject<UStaticMesh>(nullptr, TEXT("/CelestialVault/Meshes/SM_CelestialVault.SM_CelestialVault")));
    TestNotNull(
        TEXT("Epic billboard plane"),
        LoadObject<UStaticMesh>(nullptr, TEXT("/CelestialVault/Meshes/SM_Plane_FacingX.SM_Plane_FacingX")));
    TestNotNull(
        TEXT("Epic Milky Way material"),
        LoadObject<UMaterialInterface>(nullptr, TEXT("/CelestialVault/Materials/MI_CelestialVault.MI_CelestialVault")));
    TestNotNull(
        TEXT("Epic Moon material"),
        LoadObject<UMaterialInterface>(nullptr, TEXT("/CelestialVault/Materials/MI_Moon.MI_Moon")));
    TestNotNull(
        TEXT("Epic star material"),
        LoadObject<UMaterialInterface>(nullptr, TEXT("/CelestialVault/Materials/MI_Stars.MI_Stars")));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FBskTexturedEarthEnvironmentAssetsTest,
    "BskUnreal.Assets.TexturedEarthEnvironment",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskTexturedEarthEnvironmentAssetsTest::RunTest(const FString& Parameters)
{
    TestNotNull(TEXT("MyProject2 generated sphere mesh"),
        LoadObject<UStaticMesh>(nullptr, TEXT("/Game/_GENERATED/Hyperlovimia/Sphere_732702C4.Sphere_732702C4")));
    TestNotNull(TEXT("textured Earth surface material"),
        LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/Planets/Earth/M_Earth.M_Earth")));
    TestNotNull(TEXT("textured Earth cloud material"),
        LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/Planets/Earth/M_Clouds.M_Clouds")));
    TestNotNull(TEXT("textured Earth atmosphere-shell material"),
        LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/Planets/Earth/M_Atmosphere.M_Atmosphere")));
    TestNotNull(TEXT("textured Milky Way material"),
        LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/Planets/Stars/M_Stars.M_Stars")));
    TestNotNull(TEXT("Earth day texture"),
        LoadObject<UTexture2D>(nullptr, TEXT("/Game/Planets/Earth/8k_earth_daymap.8k_earth_daymap")));
    TestNotNull(TEXT("Earth night texture"),
        LoadObject<UTexture2D>(nullptr, TEXT("/Game/Planets/Earth/8k_earth_nightmap.8k_earth_nightmap")));
    TestNotNull(TEXT("Earth normal texture"),
        LoadObject<UTexture2D>(nullptr, TEXT("/Game/Planets/Earth/8k_earth_normal_map.8k_earth_normal_map")));
    TestNotNull(TEXT("Earth specular texture"),
        LoadObject<UTexture2D>(nullptr, TEXT("/Game/Planets/Earth/8k_earth_specular_map.8k_earth_specular_map")));
    TestNotNull(TEXT("Earth cloud texture"),
        LoadObject<UTexture2D>(nullptr, TEXT("/Game/Planets/Earth/8k_earth_clouds.8k_earth_clouds")));
    TestNotNull(TEXT("Milky Way texture"),
        LoadObject<UTexture2D>(nullptr, TEXT("/Game/Planets/Stars/8k_stars_milky_way.8k_stars_milky_way")));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FBskVisualHelperVisibilityTest,
    "BskUnreal.Visuals.HelperVisibilityControls",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskVisualHelperVisibilityTest::RunTest(const FString& Parameters)
{
    ABskSceneController* Controller = NewObject<ABskSceneController>();
    TestNotNull(TEXT("scene controller"), Controller);
    if (!Controller) return false;
    TestFalse(TEXT("CSS helpers hidden by default"), Controller->IsVisualKindVisible(TEXT("css")));
    TestFalse(TEXT("sensor helpers hidden by default"), Controller->IsVisualKindVisible(TEXT("generic_sensor")));
    TestFalse(TEXT("transceiver helpers hidden by default"), Controller->IsVisualKindVisible(TEXT("transceiver")));
    TestTrue(TEXT("ordinary visuals remain visible"), Controller->IsVisualKindVisible(TEXT("reaction_wheel")));
    Controller->SetVisualKindVisible(TEXT("CSS"), true);
    TestTrue(TEXT("kind lookup is case insensitive"), Controller->IsVisualKindVisible(TEXT("css")));
    TestFalse(TEXT("toggle returns and applies new state"), Controller->ToggleVisualKindVisible(TEXT("css")));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FBskRuntimeExtensionRegistryTest,
    "BskUnreal.Extensions.RegistryAndStateCache",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskRuntimeExtensionRegistryTest::RunTest(const FString& Parameters)
{
    UBskRenderWorldSubsystem* Subsystem = NewObject<UBskRenderWorldSubsystem>();
    TSharedRef<FCountingRenderExtension> Extension = MakeShared<FCountingRenderExtension>();
    TestTrue(TEXT("extension registers"), Subsystem->RegisterRenderExtension(Extension, 50));
    TestFalse(TEXT("duplicate extension rejected"), Subsystem->RegisterRenderExtension(Extension, 50));

    FBskSceneManifest Manifest;
    Manifest.SessionId = TEXT("extension-session");
    Manifest.Revision = 7;
    Subsystem->AcceptManifest(Manifest);
    Subsystem->NotifyManifestApplied(Manifest);

    FBskRenderFrame Frame;
    Frame.SessionId = Manifest.SessionId;
    Frame.ManifestRevision = Manifest.Revision;
    Frame.FrameId = 42;
    Frame.SimulationTimeNanoseconds = 123456789;
    FString Reason;
    TestTrue(*Reason, Subsystem->AcceptFrame(Frame, Reason));
    Subsystem->NotifyFrameApplied(Frame);

    FBskRenderEvent Event;
    Event.EventKind = TEXT("configuration_changed");
    Subsystem->NotifyEventApplied(Event);

    TestEqual(TEXT("manifest callback"), Extension->ManifestCount, 1);
    TestEqual(TEXT("frame callback"), Extension->FrameCount, 1);
    TestEqual(TEXT("event callback"), Extension->EventCount, 1);
    TestEqual(TEXT("received frame cache"), Subsystem->GetLatestReceivedFrameId(), int64(42));
    TestEqual(TEXT("applied frame cache"), Subsystem->GetLatestAppliedFrameId(), int64(42));
    TestNotNull(TEXT("manifest cache"), Subsystem->GetLatestManifest());

    TSharedRef<FTestCaptureProvider> CaptureProvider = MakeShared<FTestCaptureProvider>();
    TestTrue(TEXT("capture provider registers"), Subsystem->RegisterCaptureProvider(TEXT("Automation.Capture"), CaptureProvider));
    FBskCaptureRequest Request;
    Request.CameraId = TEXT("primary");
    FString CaptureError;
    TestTrue(*CaptureError, Subsystem->RequestCapture(Request, CaptureError));
    TestEqual(TEXT("capture routed"), CaptureProvider->CaptureCount, 1);
    TestTrue(TEXT("capture provider unregisters"), Subsystem->UnregisterCaptureProvider(TEXT("Automation.Capture")));
    TestTrue(TEXT("capture provider shutdown"), CaptureProvider->bShutdown);

    TestTrue(TEXT("extension unregisters"), Subsystem->UnregisterRenderExtension(Extension->GetExtensionName()));
    Subsystem->NotifyEventApplied(Event);
    TestEqual(TEXT("unregistered extension not notified"), Extension->EventCount, 1);
    return true;
}


IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FBskLocalFrameExtrapolationTest,
    "BskUnreal.Presentation.MovingOriginExtrapolation",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskLocalFrameExtrapolationTest::RunTest(const FString& Parameters)
{
    ABskSceneController* Controller = NewObject<ABskSceneController>();
    Controller->MaxExtrapolationSeconds = 0.05;
    FBskRenderFrame From;
    From.SimulationTimeNanoseconds = 1000000000;
    FBskRenderObjectState Bus;
    Bus.ObjectId = TEXT("teleop/cubesat_bus");
    Bus.PositionMeters = FVector3d::ZeroVector;
    Bus.bHasVelocity = true;
    Bus.VelocityMetersPerSecond = FVector3d(0.0, 7600.0, 0.0);
    From.Objects.Add(Bus);
    FBskRenderObjectState Wrist = Bus;
    Wrist.ObjectId = TEXT("teleop/link6");
    Wrist.PositionMeters = FVector3d(0.0, 0.0, 0.2);
    From.Objects.Add(Wrist);
    FBskCelestialBodyState Sun;
    Sun.BodyId = TEXT("sun");
    Sun.PositionMeters = FVector3d(1.5e11, 0.0, 0.0);
    Sun.VelocityMetersPerSecond = FVector3d(1.0, 20.0, 0.0);
    From.CelestialBodies.Add(Sun);
    FBskRenderFrame To = From;
    To.SimulationTimeNanoseconds += 40000000; // 40 ms source interval.
    To.OriginInertialMeters.Y += 304.0;
    To.Objects[1].PositionMeters.X += 0.004; // 0.1 m/s local wrist motion.
    To.CelestialBodies[0].PositionMeters.Y -= 303.2; // (20 - 7600) m/s.

    const FBskRenderFrame Result = Controller->ExtrapolateFrame(From, To, 0.02);
    TestTrue(TEXT("origin-following bus does not jump 152 metres between packets"),
        Result.Objects[0].PositionMeters.IsNearlyZero());
    TestTrue(TEXT("wrist follows its local motion, not the orbital velocity"),
        Result.Objects[1].PositionMeters.Equals(FVector3d(0.006, 0.0, 0.2), 1.0e-9));
    TestTrue(TEXT("celestial body uses the moving local frame too"),
        FMath::IsNearlyEqual(Result.CelestialBodies[0].PositionMeters.Y, -454.8, 1.0e-6));
    TestEqual(TEXT("inertial velocity remains available for telemetry"),
        Result.Objects[0].VelocityMetersPerSecond.Y, 7600.0);
    TestEqual(TEXT("authoritative target frame is not modified"), To.Objects[1].PositionMeters.X, 0.004);
    TestTrue(TEXT("extrapolation is capped at 50 ms"),
        FMath::IsNearlyEqual(Controller->ExtrapolateFrame(From, To, 1.0).Objects[1].PositionMeters.X, 0.009, 1.0e-9));
    TestTrue(TEXT("duplicate timestamps do not extrapolate"),
        Controller->ExtrapolateFrame(To, To, 0.02).Objects[1].PositionMeters.Equals(To.Objects[1].PositionMeters));
    TestTrue(TEXT("negative prediction duration holds the target"),
        Controller->ExtrapolateFrame(From, To, -1.0).Objects[1].PositionMeters.Equals(To.Objects[1].PositionMeters));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FBskSceneResetTest,
    "BskUnreal.Presentation.SceneReset",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskSceneResetTest::RunTest(const FString& Parameters)
{
    ABskSceneController* Controller = NewObject<ABskSceneController>();
    Controller->ActiveSessionId = TEXT("new-session");
    Controller->LastFrameId = 100;
    Controller->bHasTargetFrame = true;
    Controller->bHasPresentationFrame = true;
    Controller->PreviousFrame.FrameId = 99;
    Controller->TargetFrame.FrameId = 100;
    Controller->BlendElapsedSeconds = 2.0;
    Controller->LastFrameArrivalSeconds = 50.0;
    Controller->CameraNextDataCaptureSimulationNanoseconds.Add(TEXT("wrist"), 9000000000);
    FBskRenderEvent Event;
    Event.SessionId = TEXT("old-session");
    Event.EventKind = TEXT("scene_reset");
    Controller->ApplyEvent(Event);
    TestEqual(TEXT("late old event ignored"), Controller->LastFrameId, int64(100));
    Event.SessionId = TEXT("new-session");
    Controller->ApplyEvent(Event);
    TestEqual(TEXT("frame id cleared"), Controller->LastFrameId, int64(-1));
    TestFalse(TEXT("no interpolation across reset"), Controller->bHasTargetFrame);
    TestFalse(TEXT("no old presentation during capture"), Controller->bHasPresentationFrame);
    TestEqual(TEXT("arrival clock cleared"), Controller->LastFrameArrivalSeconds, 0.0);
    TestEqual(TEXT("capture deadlines cleared"), Controller->CameraNextDataCaptureSimulationNanoseconds.Num(), 0);

    UBskRenderWorldSubsystem* Subsystem = NewObject<UBskRenderWorldSubsystem>();
    FBskSceneManifest Manifest;
    Manifest.SessionId = TEXT("old-session");
    Manifest.Revision = 1;
    Subsystem->AcceptManifest(Manifest);
    FBskRenderFrame Frame;
    Frame.SessionId = Manifest.SessionId;
    Frame.ManifestRevision = 1;
    Frame.FrameId = 100;
    Frame.SimulationTimeNanoseconds = 9000000000;
    FString Reason;
    TestTrue(TEXT("old frame accepted before reset"), Subsystem->AcceptFrame(Frame, Reason));
    Subsystem->NotifyFrameApplied(Frame);
    Manifest.SessionId = TEXT("new-session");
    Subsystem->AcceptManifest(Manifest);
    TestEqual(TEXT("subsystem time cleared"), Subsystem->GetSimulationTimeNanoseconds(), int64(0));
    TestEqual(TEXT("received cache cleared"), Subsystem->GetLatestReceivedFrameId(), int64(-1));
    TestEqual(TEXT("applied cache cleared"), Subsystem->GetLatestAppliedFrameId(), int64(-1));
    TestFalse(TEXT("old session frame rejected"), Subsystem->AcceptFrame(Frame, Reason));
    Frame.SessionId = Manifest.SessionId;
    Frame.FrameId = 0;
    Frame.SimulationTimeNanoseconds = 0;
    TestTrue(TEXT("new frame zero accepted"), Subsystem->AcceptFrame(Frame, Reason));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FBskReceiverResetTest,
    "BskUnreal.Protocol.ResetReceiveBarrier",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskReceiverResetTest::RunTest(const FString& Parameters)
{
    FBskTcpReceiver Receiver(TEXT("127.0.0.1"), 0, 1024 * 1024);
    FBskSceneManifest OldManifest;
    OldManifest.SessionId = TEXT("old");
    Receiver.PublishManifest(MoveTemp(OldManifest));
    FBskSceneManifest Manifest;
    Receiver.ConsumeLatestManifest(Manifest);
    FBskRenderFrame OldFrame;
    OldFrame.SessionId = TEXT("old");
    OldFrame.FrameId = 100;
    Receiver.PublishLatest(MoveTemp(OldFrame));
    FBskRenderEvent OldEvent;
    OldEvent.SessionId = TEXT("old");
    OldEvent.EventKind = TEXT("old-event");
    Receiver.PublishEvent(MoveTemp(OldEvent));

    FBskSceneManifest NewManifest;
    NewManifest.SessionId = TEXT("new");
    Receiver.PublishManifest(MoveTemp(NewManifest));
    FBskRenderEvent Reset;
    Reset.SessionId = TEXT("new");
    Reset.EventKind = TEXT("scene_reset");
    Receiver.PublishEvent(MoveTemp(Reset));
    FBskRenderFrame NewFrame;
    NewFrame.SessionId = TEXT("new");
    NewFrame.FrameId = 0;
    Receiver.PublishLatest(MoveTemp(NewFrame));
    FBskRenderFrame Frame;
    FBskRenderEvent Event;
    TestFalse(TEXT("frame waits for pending manifest"), Receiver.ConsumeLatest(Frame));
    TestFalse(TEXT("event waits for pending manifest"), Receiver.ConsumeEvent(Event));
    TestTrue(TEXT("new manifest available"), Receiver.ConsumeLatestManifest(Manifest));
    TestEqual(TEXT("new manifest session"), Manifest.SessionId, FString(TEXT("new")));
    TestTrue(TEXT("only new reset event survives"), Receiver.ConsumeEvent(Event));
    TestEqual(TEXT("new reset event session"), Event.SessionId, FString(TEXT("new")));
    TestFalse(TEXT("no old events remain"), Receiver.ConsumeEvent(Event));
    TestTrue(TEXT("new frame available"), Receiver.ConsumeLatest(Frame));
    TestEqual(TEXT("frame zero is retained"), Frame.FrameId, int64(0));
    FBskRenderFrame Late;
    Late.SessionId = TEXT("old");
    Receiver.PublishLatest(MoveTemp(Late));
    TestFalse(TEXT("late old frame dropped"), Receiver.ConsumeLatest(Frame));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FBskOnDemandCaptureTest,
    "BskUnreal.Capture.OnDemandBoundaries",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskOnDemandCaptureTest::RunTest(const FString& Parameters)
{
    // Even a capture-capable receiver starts latest-wins for explicitly idle frames.
    FBskTcpReceiver Receiver(TEXT("127.0.0.1"), 0, BskProtocol::DefaultMaxPacketBytes, true);
    const auto Publish = [&](int64 Id, const FString& Episode)
    {
        FBskRenderFrame Frame;
        Frame.FrameId = Id;
        Frame.bCaptureOnDemand = true;
        Frame.CaptureEpisodeId = Episode;
        Receiver.PublishLatest(MoveTemp(Frame));
    };
    FBskRenderFrame Frame;
    Publish(0, TEXT(""));
    Publish(1, TEXT(""));
    TestEqual(TEXT("idle frames do not fill strict FIFO"), Receiver.ReliableFrames.Num(), 0);
    TestTrue(TEXT("preview does not wait for image queue capacity"), Receiver.ConsumeForCapture(Frame, false));
    TestEqual(TEXT("latest preview wins"), Frame.FrameId, int64(1));
    Publish(2, TEXT(""));
    Publish(3, TEXT("episode-A"));
    Publish(4, TEXT("episode-A"));
    Publish(5, TEXT("")); // STOP must not clear accepted strict frames.
    Publish(6, TEXT(""));
    TestFalse(TEXT("strict frames wait for capture capacity"), Receiver.ConsumeForCapture(Frame, false));
    TestTrue(TEXT("first strict frame retained"), Receiver.ConsumeForCapture(Frame, true));
    TestEqual(TEXT("start boundary"), Frame.FrameId, int64(3));
    TestEqual(TEXT("episode propagated"), Frame.CaptureEpisodeId, FString(TEXT("episode-A")));
    TestTrue(TEXT("second strict frame retained"), Receiver.ConsumeForCapture(Frame, true));
    TestEqual(TEXT("strict FIFO ordering"), Frame.FrameId, int64(4));
    TestTrue(TEXT("returns to live preview after draining"), Receiver.ConsumeForCapture(Frame, false));
    TestEqual(TEXT("post-stop preview is latest"), Frame.FrameId, int64(6));
    TestFalse(TEXT("no stale preview remains"), Receiver.ConsumeLatest(Frame));

    FBskFrameParser Parser;
    TArray<FBskRenderMessage> Messages;
    FString Error;
    const TArray<uint8> Packet = EncodePacket(TEXT("{\"protocol\":\"bsk-render/2\",\"type\":\"frame\",\"frame_id\":\"9\",\"sim_time_ns\":\"0\",\"capture_episode_id\":\"episode-B\",\"origin_N_m\":[0,0,0],\"objects\":[]}"));
    TestTrue(*Error, Parser.AppendMessages(Packet.GetData(), Packet.Num(), Messages, Error));
    if (TestEqual(TEXT("one tagged frame parsed"), Messages.Num(), 1))
    {
        TestTrue(TEXT("on-demand marker parsed"), Messages[0].Frame.bCaptureOnDemand);
        TestEqual(TEXT("episode parsed"), Messages[0].Frame.CaptureEpisodeId, FString(TEXT("episode-B")));
    }
    return true;
}

#endif
