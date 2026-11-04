#if WITH_DEV_AUTOMATION_TESTS

#include "BskEnvironment.h"
#include "BskEnvironmentBody.h"
#include "BskSceneController.h"
#include "Camera/CameraActor.h"
#include "Camera/CameraComponent.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/LightComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/SkyAtmosphereComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/DirectionalLight.h"
#include "Engine/Level.h"
#include "Engine/PostProcessVolume.h"
#include "Engine/SpotLight.h"
#include "Engine/StaticMesh.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
#include "Materials/MaterialInterface.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Misc/AutomationTest.h"
#include "NiagaraComponent.h"

namespace
{
ABskEnvironment* MakeSavedEnvironment(UWorld* World)
{
    ABskEnvironment* Environment = World->SpawnActor<ABskEnvironment>();
    // The rig's placement is editorial only: it must never become an extra
    // offset on backend coordinates that already use a floating origin.
    Environment->SetActorLocation(FVector(900000.0, 800000.0, 700000.0));
    Environment->Earth = World->SpawnActor<ABskEnvironmentBody>();
    Environment->StarSphere = World->SpawnActor<AStaticMeshActor>();
    Environment->SunVisual = World->SpawnActor<ABskEnvironmentBody>();
    Environment->SunLight = World->SpawnActor<ADirectionalLight>();
    Environment->CelestialSunLight = World->SpawnActor<ADirectionalLight>();
    Environment->FillLight = World->SpawnActor<ADirectionalLight>();
    Environment->Headlight = World->SpawnActor<ASpotLight>();
    Environment->EarthAtmosphere = World->SpawnActor<ASkyAtmosphere>();
    Environment->ExposureVolume = World->SpawnActor<APostProcessVolume>();
    Environment->PreviewCamera = World->SpawnActor<ACameraActor>();
    Environment->EarthRadiusMeters = 6000000.0;

    UStaticMesh* Sphere = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Sphere.Sphere"));
    UMaterialInterface* Material = LoadObject<UMaterialInterface>(
        nullptr, TEXT("/Engine/EngineMaterials/DefaultMaterial.DefaultMaterial"));
    Environment->Earth->Surface->SetStaticMesh(Sphere);
    Environment->Earth->Surface->SetMaterial(0, Material);
    // Material slots come from the mesh. An override on a meshless component
    // is not returned by GetNumMaterials() and cannot represent a saved sky.
    Environment->StarSphere->GetStaticMeshComponent()->SetStaticMesh(Sphere);
    Environment->SunVisual->Surface->SetStaticMesh(Sphere);
    Environment->SunVisual->Surface->SetMaterial(0, Material);
    Environment->Earth->SetActorTransform(FTransform(
        FRotator(11.0, 22.0, 33.0), FVector(1000.0, 2000.0, -650000000.0), FVector(1.2, 1.3, 1.4)));
    Environment->Earth->Surface->SetRelativeTransform(FTransform(
        FRotator(2.0, 3.0, 4.0), FVector(-100.0, -200.0, -300.0), FVector(5.0)));
    Environment->Earth->Clouds->SetRelativeScale3D(FVector(5.005));
    Environment->StarSphere->GetStaticMeshComponent()->SetMobility(EComponentMobility::Movable);
    Environment->StarSphere->SetActorRotation(FRotator(7.0, 17.0, 27.0));
    Environment->SunVisual->SetActorTransform(FTransform(FRotator(0.0, 25.0, 0.0), FVector(500000.0, 0.0, 0.0)));
    for (ADirectionalLight* Light : {Environment->SunLight.Get(), Environment->CelestialSunLight.Get(), Environment->FillLight.Get()})
        Light->GetLightComponent()->SetMobility(EComponentMobility::Movable);
    Environment->SunLight->SetActorRotation(FRotator(-35.0, -45.0, 0.0));
    Environment->SunLight->GetLightComponent()->SetIntensity(42.0f);
    Environment->SunLight->GetLightComponent()->SetLightColor(FLinearColor(0.8f, 0.6f, 0.4f));
    Environment->FillLight->GetLightComponent()->SetIntensity(3.0f);
    Environment->Headlight->GetLightComponent()->SetMobility(EComponentMobility::Movable);
    Environment->Headlight->SetActorHiddenInGame(true);
    Environment->EarthAtmosphere->GetComponent()->SetMobility(EComponentMobility::Movable);
    Environment->EarthAtmosphere->GetComponent()->SetBottomRadius(6000.0f);
    Environment->PreviewCamera->SetActorTransform(FTransform(FRotator(-10.0, 35.0, 0.0), FVector(1200.0, 3400.0, 5600.0)));
    Environment->PreviewCamera->GetCameraComponent()->SetFieldOfView(73.0f);
    Environment->ExposureVolume->bUnbound = true;
    Environment->ExposureVolume->Settings.bOverride_AutoExposureBias = true;
    Environment->ExposureVolume->Settings.AutoExposureBias = 2.25f;
    return Environment;
}

FBskSceneManifest EarthManifest(const FString& Session, double Radius)
{
    FBskSceneManifest Manifest;
    Manifest.SessionId = Session;
    Manifest.Revision = 1;
    FBskCelestialBodyDefinition Earth;
    Earth.BodyId = TEXT("earth");
    Earth.DisplayName = TEXT("Earth");
    Earth.EquatorialRadiusMeters = Radius;
    Manifest.CelestialBodies.Add(Earth);
    return Manifest;
}

int32 LiveActorCount(const UWorld* World)
{
    int32 Count = 0;
    for (const AActor* Actor : World->PersistentLevel->Actors)
        if (IsValid(Actor)) ++Count;
    return Count;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBskSavedEnvironmentBindingTest,
    "BskUnreal.Environment.SavedActorsSurviveManifests",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskSavedEnvironmentBindingTest::RunTest(const FString& Parameters)
{
    UWorld* World = UWorld::CreateWorld(EWorldType::Game, false);
    ABskEnvironment* Environment = MakeSavedEnvironment(World);
    ABskSceneController* Controller = World->SpawnActor<ABskSceneController>();
    const FTransform PreviewEarth = Environment->Earth->GetActorTransform();
    const FTransform PreviewCamera = Environment->PreviewCamera->GetActorTransform();
    const FTransform Surface = Environment->Earth->Surface->GetRelativeTransform();
    const FTransform Stars = Environment->StarSphere->GetActorTransform();
    UMaterialInterface* EarthMaterial = Environment->Earth->Surface->GetMaterial(0);
    const int32 InitialActorCount = LiveActorCount(World);

    TestTrue(TEXT("the saved level rig is adopted"), Controller->AdoptSavedEnvironment());
    TestEqual(TEXT("saved preview camera is retained"), Controller->GetSavedPreviewCamera(), Environment->PreviewCamera.Get());
    FBskSceneManifest Manifest = EarthManifest(TEXT("first"), 12000000.0);
    Controller->ApplyManifest(Manifest);
    TestTrue(TEXT("backend ownership is explicit"), Environment->IsBackendControlled);
    TestEqual(TEXT("Earth reuses the saved actor"), Controller->CelestialActors.FindRef(TEXT("earth")).Get(), static_cast<AActor*>(Environment->Earth.Get()));
    TestEqual(TEXT("manifest must not spawn duplicate environment actors"), LiveActorCount(World), InitialActorCount);
    TestFalse(TEXT("the adopted Earth remains visible"), Environment->Earth->IsHidden());
    TestTrue(TEXT("physical radius replaces the preview actor's nonuniform scale"),
        Environment->Earth->GetActorScale3D().Equals(FVector(2.0), 1.0e-9));
    TestEqual(TEXT("atmosphere uses the manifest physical radius in kilometres"),
        Environment->EarthAtmosphere->GetComponent()->BottomRadius, 12000.0f);
    TestTrue(TEXT("surface pivot and layer settings remain authored"), Environment->Earth->Surface->GetRelativeTransform().Equals(Surface));
    const FNiagaraVariable AuroraScale(FNiagaraTypeDefinition::GetFloatDef(), TEXT("User.SystemScale"));
    TestEqual(TEXT("aurora uses the physical surface scale exactly once"),
        Environment->Earth->Aurora->GetOverrideParameters().GetParameterValue<float>(AuroraScale), 10.0f);
    TestTrue(TEXT("aurora follows body scale only through its Niagara parameter"),
        Environment->Earth->Aurora->GetComponentScale().Equals(FVector::OneVector));
    TestTrue(TEXT("aurora culling bounds contain the resized surface"),
        Environment->Earth->Aurora->GetSystemFixedBounds().Max.X >= Environment->Earth->Surface->GetStaticMesh()->GetBounds().SphereRadius * 10.0);
    TestEqual(TEXT("saved Earth material survives takeover"), Environment->Earth->Surface->GetMaterial(0), EarthMaterial);
    TestTrue(TEXT("stars keep their authored orientation and scale"), Environment->StarSphere->GetActorTransform().Equals(Stars));
    TestTrue(TEXT("preview camera is not repurposed as a capture camera"), Environment->PreviewCamera->GetActorTransform().Equals(PreviewCamera));
    TestEqual(TEXT("saved camera FOV remains authored"), Environment->PreviewCamera->GetCameraComponent()->FieldOfView, 73.0f);
    TestEqual(TEXT("saved exposure is not replaced by JSON defaults"), Environment->ExposureVolume->Settings.AutoExposureBias, 2.25f);

    USceneCaptureComponent2D* Capture = NewObject<USceneCaptureComponent2D>(Controller);
    FBskCameraDefinition Camera;
    Controller->ConfigureFixedRgbExposure(Capture, &Camera);
    TestFalse(TEXT("default camera retains the saved volume exposure"), Capture->PostProcessSettings.bOverride_CameraShutterSpeed != 0);
    Camera.ExposureTimeSeconds = 0.002;
    Camera.ApertureFNumber = 4.0;
    Controller->ConfigureFixedRgbExposure(Capture, &Camera);
    TestTrue(TEXT("vision shutter override remains enabled with a saved environment"), Capture->PostProcessSettings.bOverride_CameraShutterSpeed != 0);
    TestEqual(TEXT("vision shutter seconds are converted to inverse seconds"), Capture->PostProcessSettings.CameraShutterSpeed, 500.0f);
    TestTrue(TEXT("vision aperture override remains enabled with a saved environment"), Capture->PostProcessSettings.bOverride_DepthOfFieldFstop != 0);
    TestEqual(TEXT("vision aperture retains its manifest value"), Capture->PostProcessSettings.DepthOfFieldFstop, 4.0f);
    Camera.ExposureTimeSeconds = 0.0;
    Camera.ApertureFNumber = 0.0;
    Controller->ConfigureFixedRgbExposure(Capture, &Camera);
    TestFalse(TEXT("removing the camera override returns to the saved exposure"), Capture->PostProcessSettings.bOverride_CameraShutterSpeed != 0);

    ++Manifest.Revision;
    Controller->ApplyManifest(Manifest);
    TestTrue(TEXT("repeated manifest radius scaling does not accumulate"),
        Environment->Earth->GetActorScale3D().Equals(FVector(2.0), 1.0e-9));
    Manifest.SessionId = TEXT("second");
    Controller->ApplyManifest(Manifest);
    TestEqual(TEXT("new session still reuses the saved Earth"), Controller->CelestialActors.FindRef(TEXT("earth")).Get(), static_cast<AActor*>(Environment->Earth.Get()));
    TestEqual(TEXT("new sessions do not duplicate saved actors"), LiveActorCount(World), InitialActorCount);

    Manifest.CelestialBodies.Reset();
    Controller->ApplyManifest(Manifest);
    TestTrue(TEXT("manifest without Earth restores the saved default transform"), Environment->Earth->GetActorTransform().Equals(PreviewEarth));
    TestFalse(TEXT("manifest without Earth keeps the saved default visible"), Environment->Earth->IsHidden());
    TestTrue(TEXT("saved Earth is not destroyed"), IsValid(Environment->Earth));

    // A user may omit the reserved headlight in an edited level. The runtime
    // fallback still needs a stable identity across subsequent manifests.
    Environment->Headlight = nullptr;
    Controller->Headlight = nullptr;
    Manifest.bHeadlightEnabled = true;
    const int32 BeforeFallbackHeadlight = LiveActorCount(World);
    Controller->ApplyManifest(Manifest);
    ASpotLight* FallbackHeadlight = Controller->Headlight;
    TestNotNull(TEXT("missing saved headlight creates one runtime fallback"), FallbackHeadlight);
    TestEqual(TEXT("exactly one fallback headlight is spawned"), LiveActorCount(World), BeforeFallbackHeadlight + 1);
    ++Manifest.Revision;
    Controller->ApplyManifest(Manifest);
    TestEqual(TEXT("repeated manifest reuses the runtime fallback headlight"), Controller->Headlight.Get(), FallbackHeadlight);
    TestEqual(TEXT("repeated manifest does not add another headlight"), LiveActorCount(World), BeforeFallbackHeadlight + 1);
    Manifest.SessionId = TEXT("headlight-new-session");
    Controller->ApplyManifest(Manifest);
    TestEqual(TEXT("new session retains the runtime fallback headlight"), Controller->Headlight.Get(), FallbackHeadlight);
    TestEqual(TEXT("new session does not duplicate runtime lighting"), LiveActorCount(World), BeforeFallbackHeadlight + 1);
    World->DestroyWorld(false);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBskAuroraMaterialScaleTest,
    "BskUnreal.Environment.AuroraMaterialScaleTracksBackendRadius",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskAuroraMaterialScaleTest::RunTest(const FString& Parameters)
{
    UWorld* World = UWorld::CreateWorld(EWorldType::Game, false);
    ABskEnvironment* Environment = MakeSavedEnvironment(World);
    ABskSceneController* Controller = World->SpawnActor<ABskSceneController>();
    const FTransform PreviewEarth = Environment->Earth->GetActorTransform();
    TestTrue(TEXT("the saved rig is adopted before radius updates"), Controller->AdoptSavedEnvironment());

    auto CheckAuroraScale = [this, Environment](const TCHAR* Phase, float ExpectedScale)
    {
        const UNiagaraComponent* Aurora = Environment->Earth->Aurora;
        const float SurfaceScale = static_cast<float>(Environment->Earth->Surface->GetComponentScale().GetAbsMax());
        TestEqual(FString::Printf(TEXT("%s: surface has the requested physical scale"), Phase), SurfaceScale, ExpectedScale);
        const FNiagaraVariable ScaleParameter(FNiagaraTypeDefinition::GetFloatDef(), TEXT("User.SystemScale"));
        TestEqual(FString::Printf(TEXT("%s: Niagara geometry follows the current surface scale"), Phase),
            Aurora->GetOverrideParameters().GetParameterValue<float>(ScaleParameter), SurfaceScale);
        const TArray<float>& PrimitiveData = Aurora->GetCustomPrimitiveData().Data;
        if (TestTrue(FString::Printf(TEXT("%s: aurora material has its scale compensation"), Phase), PrimitiveData.IsValidIndex(0)))
            TestEqual(FString::Printf(TEXT("%s: material coordinates follow the current surface scale"), Phase), PrimitiveData[0], SurfaceScale);
        TestTrue(FString::Printf(TEXT("%s: scale is not applied twice through the Niagara component"), Phase),
            Aurora->GetComponentScale().Equals(FVector::OneVector));
    };

    FBskSceneManifest Manifest = EarthManifest(TEXT("aurora-scale"), 12000000.0);
    Controller->ApplyManifest(Manifest);
    CheckAuroraScale(TEXT("initial takeover"), 10.0f);

    // A stale CPD value can leave valid particles with invisible/incorrect UVs.
    // Exercise the actual backend manifest path rather than calling refresh.
    ++Manifest.Revision;
    Manifest.CelestialBodies[0].EquatorialRadiusMeters = 18000000.0;
    Controller->ApplyManifest(Manifest);
    CheckAuroraScale(TEXT("larger backend radius"), 15.0f);

    ++Manifest.Revision;
    Manifest.CelestialBodies[0].EquatorialRadiusMeters = 6000000.0;
    Controller->ApplyManifest(Manifest);
    CheckAuroraScale(TEXT("smaller backend radius"), 5.0f);

    Environment->SetBackendControlled(false);
    TestTrue(TEXT("releasing backend ownership restores the authored Earth transform"),
        Environment->Earth->GetActorTransform().Equals(PreviewEarth));
    CheckAuroraScale(TEXT("restored preview"), 7.0f);
    World->DestroyWorld(false);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBskSavedEnvironmentFrameTest,
    "BskUnreal.Environment.BackendUsesExistingFloatingOriginConversion",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskSavedEnvironmentFrameTest::RunTest(const FString& Parameters)
{
    UWorld* World = UWorld::CreateWorld(EWorldType::Game, false);
    ABskEnvironment* Environment = MakeSavedEnvironment(World);
    ABskSceneController* Controller = World->SpawnActor<ABskSceneController>();
    Controller->AdoptSavedEnvironment();
    Controller->ApplyManifest(EarthManifest(TEXT("floating-origin"), 6000000.0));
    FBskRenderFrame Frame;
    Frame.SessionId = TEXT("floating-origin");
    Frame.ManifestRevision = 1;
    Frame.FrameId = 0;
    Frame.OriginInertialMeters = FVector3d(7000000.0, 8000000.0, 9000000.0);
    FBskCelestialBodyState Earth;
    Earth.BodyId = TEXT("earth");
    Earth.PositionMeters = FVector3d(12.0, -34.0, -56.0);
    Earth.OrientationWxyz = FQuat4d(FVector3d(1.0, 2.0, 3.0).GetSafeNormal(), 0.3);
    Frame.CelestialBodies.Add(Earth);
    // Both authoritative capture application and interpolated presentation use
    // this path. The inertial origin is metadata, never another translation.
    Controller->ApplyFrame(Frame, false);
    const FVector ExpectedLocation(1200.0, 3400.0, -5600.0);
    TestTrue(TEXT("local metres use centimetres and exactly one Y mirror"), Environment->Earth->GetActorLocation().Equals(ExpectedLocation, 1.0e-8));
    const FQuat ExpectedRotation(-Earth.OrientationWxyz.X, Earth.OrientationWxyz.Y, -Earth.OrientationWxyz.Z, Earth.OrientationWxyz.W);
    TestTrue(TEXT("body orientation retains the established handedness conversion"), Environment->Earth->GetActorQuat().Equals(ExpectedRotation, 1.0e-8));
    TestTrue(TEXT("atmosphere follows the same celestial centre"), Environment->EarthAtmosphere->GetActorLocation().Equals(ExpectedLocation, 1.0e-8));
    TestEqual(TEXT("authoritative capture does not advance the presentation clock"), Controller->LastFrameId, int64(-1));
    TestFalse(TEXT("authoritative capture does not replace the presentation frame"), Controller->bHasPresentationFrame);

    Frame.FrameId = 1;
    Frame.OriginInertialMeters += FVector3d(1.0e9, -2.0e9, 3.0e9);
    Controller->ApplyFrame(Frame);
    TestTrue(TEXT("a changed inertial origin is not subtracted twice"), Environment->Earth->GetActorLocation().Equals(ExpectedLocation, 1.0e-8));
    TestEqual(TEXT("presentation application retains frame identity"), Controller->LastFrameId, int64(1));
    FBskRenderEvent Reset;
    Reset.SessionId = Frame.SessionId;
    Reset.EventKind = TEXT("scene_reset");
    Controller->ApplyEvent(Reset);
    TestTrue(TEXT("scene reset keeps the persistent Earth"), IsValid(Environment->Earth));
    TestEqual(TEXT("scene reset clears presentation identity"), Controller->LastFrameId, int64(-1));

    Controller->Converter = FBskCoordinateConverter(50.0, false);
    const float SavedAtmosphereHeight = Environment->EarthAtmosphere->GetComponent()->AtmosphereHeight;
    Controller->ApplyManifest(EarthManifest(TEXT("custom-units"), 6000000.0));
    TestTrue(TEXT("saved geometry respects custom backend centimetres per metre"),
        Environment->Earth->GetActorScale3D().Equals(FVector(0.5), 1.0e-9));
    TestEqual(TEXT("custom units keep atmosphere and Earth radii consistent"),
        Environment->EarthAtmosphere->GetComponent()->BottomRadius, 3000.0f);
    TestEqual(TEXT("custom units also scale the saved atmosphere height"),
        Environment->EarthAtmosphere->GetComponent()->AtmosphereHeight, SavedAtmosphereHeight * 0.5f);
    Frame.SessionId = TEXT("custom-units");
    Frame.FrameId = 0;
    Controller->ApplyFrame(Frame);
    TestTrue(TEXT("custom units and an unmirrored local Y remain supported"),
        Environment->Earth->GetActorLocation().Equals(FVector(600.0, -1700.0, -2800.0), 1.0e-8));
    const FQuat UnmirroredRotation(Controller->Converter.ActiveLocalWxyzToUnreal(Earth.OrientationWxyz));
    TestTrue(TEXT("custom unit configuration retains its declared orientation conversion"),
        Environment->Earth->GetActorQuat().Equals(UnmirroredRotation, 1.0e-8));
    World->DestroyWorld(false);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBskSavedAtmosphereScaleTest,
    "BskUnreal.Environment.SavedAtmosphereLengthsSurviveBackendScaling",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskSavedAtmosphereScaleTest::RunTest(const FString& Parameters)
{
    UWorld* World = UWorld::CreateWorld(EWorldType::Game, false);
    ABskEnvironment* Environment = MakeSavedEnvironment(World);
    USkyAtmosphereComponent* Atmosphere = Environment->EarthAtmosphere->GetComponent();
    Atmosphere->SetBottomRadius(5990.0f); // An authored 10 km inset below the mesh.
    Atmosphere->SetAtmosphereHeight(100.0f);
    Atmosphere->SetAerialPerspectiveStartDepth(1.25f);
    ABskSceneController* Controller = World->SpawnActor<ABskSceneController>();
    Controller->AdoptSavedEnvironment();

    FBskSceneManifest Manifest = EarthManifest(TEXT("atmosphere-scale"), 12000000.0);
    Controller->ApplyManifest(Manifest);
    TestEqual(TEXT("doubling Earth preserves the relative atmospheric inset"), Atmosphere->BottomRadius, 11980.0f);
    TestEqual(TEXT("atmosphere thickness follows the same radius ratio"), Atmosphere->AtmosphereHeight, 200.0f);
    TestEqual(TEXT("aerial perspective start follows the same radius ratio"), Atmosphere->AerialPerspectiveStartDepth, 2.5f);

    ++Manifest.Revision;
    Manifest.CelestialBodies[0].EquatorialRadiusMeters = 6000000.0;
    Controller->Converter = FBskCoordinateConverter(50.0, false);
    Controller->ApplyManifest(Manifest);
    TestEqual(TEXT("revised radius and units do not accumulate atmospheric scale"), Atmosphere->BottomRadius, 2995.0f);
    TestEqual(TEXT("revised units scale atmosphere height"), Atmosphere->AtmosphereHeight, 50.0f);
    TestEqual(TEXT("revised units scale aerial perspective start"), Atmosphere->AerialPerspectiveStartDepth, 0.625f);

    ++Manifest.Revision;
    Manifest.CelestialBodies.Reset();
    Controller->ApplyManifest(Manifest);
    TestEqual(TEXT("removing backend Earth restores the saved atmospheric inset"), Atmosphere->BottomRadius, 5990.0f);
    TestEqual(TEXT("removing backend Earth restores atmosphere thickness"), Atmosphere->AtmosphereHeight, 100.0f);
    TestEqual(TEXT("removing backend Earth restores aerial perspective start"), Atmosphere->AerialPerspectiveStartDepth, 1.25f);

    Controller->ApplyManifest(EarthManifest(TEXT("atmosphere-new-session"), 12000000.0));
    Environment->SetBackendControlled(false);
    TestEqual(TEXT("preview restore keeps the original atmospheric inset"), Atmosphere->BottomRadius, 5990.0f);
    TestEqual(TEXT("preview restore keeps the original aerial perspective start"), Atmosphere->AerialPerspectiveStartDepth, 1.25f);
    World->DestroyWorld(false);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBskSunVisualDiskTest,
    "BskUnreal.Environment.SunVisualDiskDoesNotChangePhysicalLighting",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskSunVisualDiskTest::RunTest(const FString& Parameters)
{
    UWorld* World = UWorld::CreateWorld(EWorldType::Game, false);
    ABskEnvironment* Environment = MakeSavedEnvironment(World);
    UMaterialInterface* Stars = LoadObject<UMaterialInterface>(nullptr,
        TEXT("/Game/BSK/Environment/IncredibleEarth/MI_Stars_Main.MI_Stars_Main"));
    if (!TestNotNull(TEXT("the migrated sky material is available"), Stars))
    {
        World->DestroyWorld(false);
        return false;
    }
    Environment->StarSphere->GetStaticMeshComponent()->SetMaterial(0, Stars);
    Environment->SunVisualAngularDiameterDegrees = 3.0;
    UDirectionalLightComponent* SunLight = CastChecked<UDirectionalLightComponent>(Environment->SunLight->GetLightComponent());
    SunLight->SetLightSourceAngle(3.0f);
    ABskSceneController* Controller = World->SpawnActor<ABskSceneController>();
    Controller->AdoptSavedEnvironment();
    TestNull(TEXT("independent preview retains the saved material without a runtime instance"), Controller->SunDiskMaterial.Get());

    FBskSceneManifest Manifest;
    Manifest.SessionId = TEXT("sun-disk");
    Manifest.Revision = 1;
    FBskCelestialBodyDefinition Sun;
    Sun.BodyId = TEXT("sun");
    Sun.EquatorialRadiusMeters = 695700000.0;
    Sun.VisualRole = TEXT("star");
    Sun.bLuminous = true;
    Sun.bDrivesDirectionalLight = true;
    Sun.LightIlluminanceLuxAtReferenceDistance = 132821.5;
    Sun.LightReferenceDistanceMeters = 149597870700.0;
    Manifest.CelestialBodies.Add(Sun);
    Controller->ApplyManifest(Manifest);
    UMaterialInstanceDynamic* Disk = Controller->SunDiskMaterial;
    if (!TestNotNull(TEXT("backend creates a runtime instance for the visual disk"), Disk))
    {
        World->DestroyWorld(false);
        return false;
    }
    TestEqual(TEXT("runtime instance derives directly from the saved material"), Disk->Parent.Get(), Stars);

    FBskRenderFrame Frame;
    Frame.SessionId = Manifest.SessionId;
    Frame.ManifestRevision = Manifest.Revision;
    Frame.FrameId = 0;
    FBskCelestialBodyState SunState;
    SunState.BodyId = Sun.BodyId;
    SunState.PositionMeters = FVector3d(Sun.LightReferenceDistanceMeters, 0.0, 0.0);
    Frame.CelestialBodies.Add(SunState);
    Controller->ApplyFrame(Frame, false);
    TestEqual(TEXT("a luminous Sun is not treated as its own occulting body"), Controller->CurrentSolarVisibility, 1.0);
    TestEqual(TEXT("the clear Sun retains the reference-distance illuminance"), SunLight->Intensity, 132821.5f);
    const float ExpectedPhysicalAngle = static_cast<float>(FMath::RadiansToDegrees(
        2.0 * FMath::Asin(Sun.EquatorialRadiusMeters / Sun.LightReferenceDistanceMeters)));
    TestEqual(TEXT("directional-light penumbra retains the physical angular diameter"), SunLight->LightSourceAngle, ExpectedPhysicalAngle);
    TestEqual(TEXT("celestial lighting uses the same physical angular diameter"),
        CastChecked<UDirectionalLightComponent>(Environment->CelestialSunLight->GetLightComponent())->LightSourceAngle,
        ExpectedPhysicalAngle);
    float VisualAngle = 0.0f;
    float RadianceScale = 0.0f;
    TestTrue(TEXT("visual angle parameter is present"), Disk->GetScalarParameterValue(
        FMaterialParameterInfo(TEXT("BSKSunVisualAngularDiameter")), VisualAngle));
    TestTrue(TEXT("disk radiance parameter is present"), Disk->GetScalarParameterValue(
        FMaterialParameterInfo(TEXT("BSKSunDiskRadianceScale")), RadianceScale));
    TestEqual(TEXT("rendered disk retains the authored 3 degree angle"), VisualAngle, 3.0f);
    const double PhysicalSolidAngle = 2.0 * UE_PI * (1.0 - FMath::Cos(FMath::DegreesToRadians(ExpectedPhysicalAngle * 0.5)));
    const double VisualSolidAngle = 2.0 * UE_PI * (1.0 - FMath::Cos(FMath::DegreesToRadians(1.5)));
    TestTrue(TEXT("expanded disk preserves integrated radiance instead of becoming 32 times brighter"),
        FMath::IsNearlyEqual(static_cast<double>(RadianceScale) * VisualSolidAngle, PhysicalSolidAngle, 1.0e-10));

    // A partial eclipse must remain physical when the display disk is edited.
    Controller->SolarOccluders.Emplace(FVector3d(384400000.0, 800000.0, 0.0), 1737400.0);
    const double Visibility = Controller->SolarVisibilityAt(FVector3d::ZeroVector);
    TestTrue(TEXT("the test geometry gives a partial eclipse"), Visibility > 0.0 && Visibility < 1.0);
    const float Lux = SunLight->Intensity;
    Environment->SunVisualAngularDiameterDegrees = 6.0;
    Controller->UpdateSunDiskMaterial();
    TestEqual(TEXT("editing visual diameter does not change the physical source radius"), Controller->CurrentSunRadiusMeters, Sun.EquatorialRadiusMeters);
    TestEqual(TEXT("editing visual diameter does not change penumbra"), SunLight->LightSourceAngle, ExpectedPhysicalAngle);
    TestEqual(TEXT("editing visual diameter does not change illuminance"), SunLight->Intensity, Lux);
    TestEqual(TEXT("editing visual diameter does not change eclipse coverage"), Controller->SolarVisibilityAt(FVector3d::ZeroVector), Visibility);
    Environment->SunVisualAngularDiameterDegrees = 3.0;

    ++Manifest.Revision;
    Controller->ApplyManifest(Manifest);
    TestEqual(TEXT("a repeated manifest does not nest another material instance"), Controller->SunDiskMaterial.Get(), Disk);
    Manifest.SessionId = TEXT("sun-disk-next");
    Controller->ApplyManifest(Manifest);
    TestNotEqual(TEXT("a new session gets a fresh runtime material instance"), Controller->SunDiskMaterial.Get(), Disk);
    if (Controller->SunDiskMaterial)
        TestEqual(TEXT("new sessions derive from the saved material, not the previous runtime instance"), Controller->SunDiskMaterial->Parent.Get(), Stars);
    Controller->RestoreSavedEnvironmentPreview();
    TestEqual(TEXT("preview restores the original saved star material"), Environment->StarSphere->GetStaticMeshComponent()->GetMaterial(0), Stars);
    TestEqual(TEXT("preview restores the saved light's 3 degree angle"), SunLight->LightSourceAngle, 3.0f);
    TestNull(TEXT("preview releases the runtime disk material"), Controller->SunDiskMaterial.Get());
    World->DestroyWorld(false);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBskSavedMoonBindingTest,
    "BskUnreal.Environment.SavedMoonActorReusedByEphemeris",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskSavedMoonBindingTest::RunTest(const FString& Parameters)
{
    UWorld* World = UWorld::CreateWorld(EWorldType::Game, false);
    ABskEnvironment* Environment = MakeSavedEnvironment(World);
    AStaticMeshActor* Moon = World->SpawnActor<AStaticMeshActor>();
    UStaticMesh* Sphere = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Sphere.Sphere"));
    Moon->GetStaticMeshComponent()->SetStaticMesh(Sphere);
    Moon->Tags.Add(FName(TEXT("BSK.Semantic.moon")));
    Moon->Tags.Add(FName(TEXT("BSK.Preview.StaticAsset")));
    Moon->SetActorTransform(FTransform(FRotator(7.0, 8.0, 9.0), FVector(100.0, 200.0, 300.0), FVector(3.0)));
    const FTransform MoonPreview = Moon->GetActorTransform();

    ABskSceneController* Controller = World->SpawnActor<ABskSceneController>();
    TestTrue(TEXT("the saved level rig is adopted"), Controller->AdoptSavedEnvironment());
    const int32 BeforeManifestActorCount = LiveActorCount(World);

    FBskSceneManifest Manifest;
    Manifest.SessionId = TEXT("moon-session");
    Manifest.Revision = 1;
    FBskCelestialBodyDefinition MoonDefinition;
    MoonDefinition.BodyId = TEXT("moon");
    MoonDefinition.DisplayName = TEXT("Moon");
    MoonDefinition.EquatorialRadiusMeters = 2.0;
    Manifest.CelestialBodies.Add(MoonDefinition);
    Controller->ApplyManifest(Manifest);

    TestEqual(TEXT("the tagged saved Moon is reused"),
        Controller->CelestialActors.FindRef(TEXT("moon")).Get(), static_cast<AActor*>(Moon));
    TestEqual(TEXT("Moon reuse does not spawn a duplicate"), LiveActorCount(World), BeforeManifestActorCount);
    TestTrue(TEXT("backend radius updates the saved Moon scale"),
        Moon->GetActorScale3D().Equals(FVector(4.0), 1.0e-6));

    FBskRenderFrame Frame;
    Frame.SessionId = Manifest.SessionId;
    Frame.ManifestRevision = Manifest.Revision;
    Frame.FrameId = 1;
    FBskCelestialBodyState MoonState;
    MoonState.BodyId = TEXT("moon");
    MoonState.PositionMeters = FVector3d(12.0, -34.0, -56.0);
    MoonState.OrientationWxyz = FQuat4d(FVector3d(1.0, 2.0, 3.0).GetSafeNormal(), 0.3);
    Frame.CelestialBodies.Add(MoonState);
    Controller->ApplyFrame(Frame, false);
    TestTrue(TEXT("Moon position uses the established metre-to-centimetre conversion"),
        Moon->GetActorLocation().Equals(FVector(1200.0, 3400.0, -5600.0), 1.0e-8));
    const FQuat ExpectedRotation(Controller->Converter.ActiveLocalWxyzToUnreal(MoonState.OrientationWxyz));
    TestTrue(TEXT("Moon orientation uses the established handedness conversion"),
        Moon->GetActorQuat().Equals(ExpectedRotation, 1.0e-8));

    // Simulate a stale runtime Moon left by an older manifest. A new backend
    // session must destroy that duplicate before rebinding the saved actor.
    AStaticMeshActor* StaleMoon = World->SpawnActor<AStaticMeshActor>();
    StaleMoon->Tags.Add(FName(TEXT("BSK.Celestial.moon")));
    Controller->CelestialActors.Add(TEXT("moon"), StaleMoon);
    ++Manifest.Revision;
    Manifest.SessionId = TEXT("moon-next-session");
    Controller->ApplyManifest(Manifest);
    TestFalse(TEXT("a stale runtime Moon is removed on session takeover"), IsValid(StaleMoon));
    TestEqual(TEXT("the new session still binds the saved Moon"),
        Controller->CelestialActors.FindRef(TEXT("moon")).Get(), static_cast<AActor*>(Moon));
    // A new session restores the authored pose, then its manifest reapplies
    // the physical radius. Comparing the old preview scale (3) to 4 is wrong.
    TestTrue(TEXT("a new backend session restores the authored Moon preview pose"),
        Moon->GetActorLocation().Equals(MoonPreview.GetLocation()) &&
        Moon->GetActorQuat().Equals(MoonPreview.GetRotation()));
    TestTrue(TEXT("a new session reapplies the manifest Moon radius"),
        Moon->GetActorScale3D().Equals(FVector(4.0), 1.0e-6));

    ++Manifest.Revision;
    Manifest.CelestialBodies.Reset();
    Controller->ApplyManifest(Manifest);
    TestTrue(TEXT("removing a manifest Moon preserves the authored level actor"), IsValid(Moon));
    TestTrue(TEXT("removing a manifest Moon restores its authored transform"), Moon->GetActorTransform().Equals(MoonPreview));
    TestFalse(TEXT("removed Moon has no stale backend binding"), Controller->CelestialActors.Contains(TEXT("moon")));
    World->DestroyWorld(false);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBskEnvironmentPreviewTest,
    "BskUnreal.Environment.PreviewOwnershipAndRestore",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskEnvironmentPreviewTest::RunTest(const FString& Parameters)
{
    UWorld* World = UWorld::CreateWorld(EWorldType::Game, false);
    ABskEnvironment* Environment = MakeSavedEnvironment(World);
    ABskSceneController* Controller = World->SpawnActor<ABskSceneController>();
    const FTransform Earth = Environment->Earth->GetActorTransform();
    const FTransform Surface = Environment->Earth->Surface->GetRelativeTransform();
    const FTransform Sun = Environment->SunVisual->GetActorTransform();
    const FLinearColor SunColor = Environment->SunLight->GetLightComponent()->GetLightColor();
    Controller->AdoptSavedEnvironment();
    TestFalse(TEXT("standalone adoption stays in preview"), Environment->IsBackendControlled);
    Controller->UpdateDecorativeSunPlacement();
    Controller->UpdateDecorativeSunScale();
    TestTrue(TEXT("standalone preview does not move the saved Sun"), Environment->SunVisual->GetActorTransform().Equals(Sun));

    Environment->bAllowBackendControl = false;
    Controller->ApplyManifest(EarthManifest(TEXT("blocked"), 12000000.0));
    FBskRenderFrame Frame;
    Frame.FrameId = 123;
    Controller->ApplyFrame(Frame);
    TestFalse(TEXT("preview ownership rejects backend takeover"), Environment->IsBackendControlled);
    TestTrue(TEXT("rejected manifest does not replace the active session"), Controller->ActiveSessionId.IsEmpty());
    TestTrue(TEXT("rejected backend leaves saved Earth untouched"), Environment->Earth->GetActorTransform().Equals(Earth));

    Environment->bAllowBackendControl = true;
    Environment->SetBackendControlled(true);
    Environment->Earth->SetActorLocation(FVector(999.0));
    Environment->Earth->Surface->SetRelativeScale3D(FVector(99.0));
    Environment->SunLight->GetLightComponent()->SetIntensity(999.0f);
    Environment->SunLight->GetLightComponent()->SetLightColor(FLinearColor::Blue);
    Environment->Headlight->AttachToActor(Environment->SunVisual, FAttachmentTransformRules::KeepWorldTransform);
    Environment->EarthAtmosphere->GetComponent()->SetBottomRadius(1000.0f);
    // Capturing twice must never make the backend-modified state the default.
    Environment->CapturePreviewState();
    Environment->SetBackendControlled(false);
    TestFalse(TEXT("restoring preview relinquishes backend ownership"), Environment->IsBackendControlled);
    TestTrue(TEXT("restore returns Earth to saved transform"), Environment->Earth->GetActorTransform().Equals(Earth));
    TestTrue(TEXT("restore returns layer transform to the saved pivot"), Environment->Earth->Surface->GetRelativeTransform().Equals(Surface));
    TestEqual(TEXT("restore returns saved sunlight intensity"), Environment->SunLight->GetLightComponent()->Intensity, 42.0f);
    TestTrue(TEXT("restore returns saved sunlight color"), Environment->SunLight->GetLightComponent()->GetLightColor().Equals(SunColor));
    TestEqual(TEXT("restore returns saved atmosphere radius"), Environment->EarthAtmosphere->GetComponent()->BottomRadius, 6000.0f);
    TestNull(TEXT("restore detaches the backend headlight to its saved parent"), Environment->Headlight->GetAttachParentActor());
    TestTrue(TEXT("restore returns the saved headlight visibility"), Environment->Headlight->IsHidden());

    Controller->ApplyManifest(EarthManifest(TEXT("live-before-opt-out"), 12000000.0));
    TestTrue(TEXT("a running session can take ownership before opt-out"), Environment->IsBackendControlled);
    Environment->bAllowBackendControl = false;
    Controller->Tick(1.0f / 60.0f);
    TestTrue(TEXT("runtime opt-out remains preview-only until restart"), Controller->bEnvironmentPreviewOnly);
    TestFalse(TEXT("runtime opt-out relinquishes backend ownership"), Environment->IsBackendControlled);
    TestTrue(TEXT("runtime opt-out restores saved Earth before any further capture"), Environment->Earth->GetActorTransform().Equals(Earth));
    TestTrue(TEXT("runtime opt-out restores the saved Sun"), Environment->SunVisual->GetActorTransform().Equals(Sun));
    Environment->bAllowBackendControl = true;
    Controller->ApplyManifest(EarthManifest(TEXT("must-not-replay"), 18000000.0));
    Controller->ApplyFrame(Frame);
    Controller->Tick(1.0f / 60.0f);
    TestFalse(TEXT("reenabling the property does not replay queued backend data"), Environment->IsBackendControlled);
    TestTrue(TEXT("preview-only ticks keep the saved Earth"), Environment->Earth->GetActorTransform().Equals(Earth));
    TestTrue(TEXT("preview-only ticks do not reposition the saved Sun"), Environment->SunVisual->GetActorTransform().Equals(Sun));
    World->DestroyWorld(false);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBskEnvironmentLevelAssetTest,
    "BskUnreal.Environment.SavedLevelAsset",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBskEnvironmentLevelAssetTest::RunTest(const FString& Parameters)
{
    UWorld* World = LoadObject<UWorld>(nullptr, TEXT("/Game/BSK/Maps/BskEnvironment.BskEnvironment"));
    if (!TestNotNull(TEXT("project environment map is saved as a loadable asset"), World)) return false;
    if (!TestNotNull(TEXT("project environment map has a persistent level"), World->PersistentLevel.Get())) return false;
    TArray<ABskEnvironment*> Environments;
    for (AActor* Actor : World->PersistentLevel->Actors)
        if (ABskEnvironment* Environment = Cast<ABskEnvironment>(Actor)) Environments.Add(Environment);
    if (!TestEqual(TEXT("saved map contains exactly one environment rig"), Environments.Num(), 1)) return false;
    ABskEnvironment* Environment = Environments[0];
    TestNotNull(TEXT("Earth is serialized in the map, before Play"), Environment->Earth.Get());
    TestNotNull(TEXT("stars are serialized in the map, before Play"), Environment->StarSphere.Get());
    TestNotNull(TEXT("Sun visual is serialized in the map, before Play"), Environment->SunVisual.Get());
    TestNotNull(TEXT("primary sunlight is serialized in the map"), Environment->SunLight.Get());
    TestNotNull(TEXT("planet sunlight is serialized in the map"), Environment->CelestialSunLight.Get());
    TestNotNull(TEXT("atmosphere is serialized in the map"), Environment->EarthAtmosphere.Get());
    TestNotNull(TEXT("exposure is serialized in the map"), Environment->ExposureVolume.Get());
    TestNotNull(TEXT("preview camera is serialized in the map"), Environment->PreviewCamera.Get());
    if (Environment->Earth)
    {
        TestNotNull(TEXT("saved Earth has renderable geometry"), Environment->Earth->Surface->GetStaticMesh().Get());
        TestNotNull(TEXT("saved Earth has a material"), Environment->Earth->Surface->GetMaterial(0));
    }
    if (Environment->StarSphere)
        TestNotNull(TEXT("saved star sphere has renderable geometry"), Environment->StarSphere->GetStaticMeshComponent()->GetStaticMesh().Get());
    TestFalse(TEXT("saved map starts in independent preview ownership"), Environment->IsBackendControlled);
    return true;
}

#endif
