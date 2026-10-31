#include "BskSceneController.h"
#include "Engine/StaticMeshActor.h"
#include "BskCameraPawn.h"
#include "BskUnrealRuntime.h"
#include "BskRenderWorldSubsystem.h"
#include "Scene/BskSceneUtilities.h"
#include "Engine/World.h"
#include "Dom/JsonObject.h"
#include "Misc/CommandLine.h"
#include "Misc/ConfigCacheIni.h"
#include "Misc/Paths.h"
#include "Misc/Parse.h"
#include "BskCelestialLighting.h"
#include "Camera/CameraComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/LightComponent.h"
#include "Components/PointLightComponent.h"
#include "Components/SpotLightComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Components/SkyAtmosphereComponent.h"
#include "Components/SkyLightComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Particles/ParticleSystem.h"
#include "Particles/ParticleSystemComponent.h"
#include "Engine/DirectionalLight.h"
#include "Engine/PointLight.h"
#include "Engine/SpotLight.h"
#include "Engine/SkyLight.h"
#include "Engine/Light.h"
#include "Engine/PostProcessVolume.h"
#include "Engine/StaticMesh.h"
#include "EngineUtils.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "DrawDebugHelpers.h"
#include "HAL/PlatformTime.h"
#include "UObject/UnrealType.h"

using namespace BskSceneUtilities;

AActor* ABskSceneController::SpawnTexturedEarth(const FString& ActorName, double RadiusMeters)
{
    UStaticMesh* SphereMesh = LoadObject<UStaticMesh>(nullptr, *EarthSphereMeshPath);
    UMaterialInterface* SurfaceMaterial = LoadObject<UMaterialInterface>(nullptr, *EarthSurfaceMaterialPath);
    UMaterialInterface* CloudMaterial = LoadObject<UMaterialInterface>(nullptr, *EarthCloudMaterialPath);
    UMaterialInterface* AtmosphereMaterial = LoadObject<UMaterialInterface>(nullptr, *EarthAtmosphereMaterialPath);
    if (!SphereMesh || !SurfaceMaterial)
    {
        UE_LOG(LogBskUnreal, Warning,
            TEXT("Textured Earth assets unavailable mesh=%s surface=%s; using the celestial fallback"),
            SphereMesh ? TEXT("ok") : TEXT("missing"), SurfaceMaterial ? TEXT("ok") : TEXT("missing"));
        return nullptr;
    }

    const FBoxSphereBounds MeshBounds = SphereMesh->GetBounds();
    const double SourceRadiusCentimeters = FMath::Max(
        static_cast<double>(MeshBounds.BoxExtent.GetMax()), UE_DOUBLE_SMALL_NUMBER);
    const FVector SourceCenterCentimeters = MeshBounds.Origin;
    const double TargetRadiusCentimeters = FMath::Max(RadiusMeters, 1.0) * Converter.GetCentimetersPerMeter();

    FActorSpawnParameters Params;
    Params.Name = MakeUniqueObjectName(GetWorld(), AActor::StaticClass(), SafeActorName(ActorName));
    Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    AActor* Actor = GetWorld()->SpawnActor<AActor>(AActor::StaticClass(), FTransform::Identity, Params);
    if (!Actor) return nullptr;

    USceneComponent* Root = NewObject<USceneComponent>(Actor, TEXT("EarthRoot"));
    Root->SetMobility(EComponentMobility::Movable);
    Root->RegisterComponent();
    Actor->SetRootComponent(Root);

    auto AddLayer = [Actor, Root, SphereMesh, SourceRadiusCentimeters, SourceCenterCentimeters, TargetRadiusCentimeters](
        const TCHAR* Name, UMaterialInterface* Material, double ScaleRatio)
    {
        if (!Material) return static_cast<UStaticMeshComponent*>(nullptr);
        const double MeshScale = TargetRadiusCentimeters * ScaleRatio / SourceRadiusCentimeters;
        UStaticMeshComponent* Component = NewObject<UStaticMeshComponent>(Actor, Name);
        // The original MyProject2 actors are static. Runtime celestial actors must
        // remain movable, but all visual component flags match the source map.
        Component->SetMobility(EComponentMobility::Movable);
        Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        Component->SetCastShadow(true);
        Component->SetAffectDistanceFieldLighting(true);
        Component->SetAffectDynamicIndirectLighting(true);
        Component->SetReceivesDecals(true);
        Component->SetRenderInMainPass(true);
        Component->SetRenderInDepthPass(true);
        Component->SetCanEverAffectNavigation(false);
        Component->SetStaticMesh(SphereMesh);
        Component->SetMaterial(0, Material);
        Component->SetRelativeScale3D(FVector(MeshScale));
        // The migrated Modeling Tools sphere has its pivot at the south pole.
        // Offset its imported bounds origin so the actor transform remains the
        // Basilisk/MJScene celestial centre while preserving the exact source mesh.
        Component->SetRelativeLocation(-SourceCenterCentimeters * MeshScale);
        Component->SetTranslucentSortPriority(0);
        Component->SetupAttachment(Root);
        Component->RegisterComponent();
        return Component;
    };

    AddLayer(TEXT("EarthSurface"), SurfaceMaterial, 1.0);
    if (!AddLayer(TEXT("EarthClouds"), CloudMaterial, EarthCloudScale))
    {
        UE_LOG(LogBskUnreal, Warning, TEXT("Earth cloud material unavailable: %s"), *EarthCloudMaterialPath);
    }
    if (!bUseEarthSkyAtmosphere && !AddLayer(TEXT("EarthAtmosphereShell"), AtmosphereMaterial, EarthAtmosphereScale))
    {
        UE_LOG(LogBskUnreal, Warning, TEXT("Earth atmosphere-shell material unavailable: %s"), *EarthAtmosphereMaterialPath);
    }
    UE_LOG(LogBskUnreal, Display,
        TEXT("Created MyProject2-configured Earth %s mesh=%s radius=%.3f km clouds=%.4fx atmosphere=%.4fx"),
        *ActorName, *EarthSphereMeshPath, RadiusMeters / 1000.0, EarthCloudScale, EarthAtmosphereScale);
    return Actor;
}

AActor* ABskSceneController::SpawnCelestialBody(const FBskCelestialBodyDefinition& Definition)
{
    if (UBskRenderWorldSubsystem* RenderSubsystem = GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>())
    {
        const FBskRenderSpawnContext Context{GetWorld(), this, &Converter};
        if (AActor* Actor = RenderSubsystem->TrySpawnCelestialBody(Context, Definition)) return Actor;
    }

    if (bUseOfficialCelestialAssets && Definition.BodyId.Equals(TEXT("moon"), ESearchCase::IgnoreCase))
    {
        UStaticMesh* MoonPlane = LoadObject<UStaticMesh>(nullptr, TEXT("/CelestialVault/Meshes/SM_Plane_FacingX.SM_Plane_FacingX"));
        UMaterialInterface* MoonMaterial = LoadObject<UMaterialInterface>(nullptr, TEXT("/CelestialVault/Materials/MI_Moon.MI_Moon"));
        if (MoonPlane && MoonMaterial)
        {
            FActorSpawnParameters Params;
            Params.Name = MakeUniqueObjectName(GetWorld(), AStaticMeshActor::StaticClass(), SafeActorName(TEXT("celestial_moon")));
            Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
            AStaticMeshActor* Actor = GetWorld()->SpawnActor<AStaticMeshActor>(AStaticMeshActor::StaticClass(), FTransform::Identity, Params);
            if (Actor)
            {
                UStaticMeshComponent* Component = Actor->GetStaticMeshComponent();
                Component->SetMobility(EComponentMobility::Movable);
                Component->SetStaticMesh(MoonPlane);
                Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
                Component->SetCastShadow(false);
                UMaterialInstanceDynamic* Instance = UMaterialInstanceDynamic::Create(MoonMaterial, Actor);
                Instance->SetScalarParameterValue(TEXT("MoonAge"), 0.5f);
                Instance->SetScalarParameterValue(TEXT("Brightness"), 1.15f);
                Instance->SetScalarParameterValue(TEXT("EarthLightContribution"), 0.06f);
                Component->SetMaterial(0, Instance);
                const double DiameterMeters = 2.0 * FMath::Max(Definition.EquatorialRadiusMeters, 1.0);
                Actor->SetActorScale3D(FVector(DiameterMeters));
                Actor->Tags.AddUnique(FName(*FString::Printf(TEXT("BSK.Celestial.%s"), *Definition.BodyId)));
                CelestialBillboardIds.Add(Definition.BodyId);
                UE_LOG(LogBskUnreal, Display, TEXT("Using Epic Celestial Vault Moon material for %s"), *Definition.BodyId);
                return Actor;
            }
        }
        UE_LOG(LogBskUnreal, Warning, TEXT("Epic Celestial Vault Moon assets unavailable; using the BSK fallback sphere"));
    }

    AActor* Actor = nullptr;
    if (Definition.BodyId.Equals(TEXT("earth"), ESearchCase::IgnoreCase))
    {
        Actor = SpawnTexturedEarth(TEXT("celestial_earth"), Definition.EquatorialRadiusMeters);
    }
    if (!Actor)
    {
        FObjectSpec Spec;
        Spec.PlaceholderShape = TEXT("sphere");
        const double DiameterMeters = 2.0 * FMath::Max(Definition.EquatorialRadiusMeters, 1.0);
        Spec.SizeMeters = FVector3d(DiameterMeters, DiameterMeters, DiameterMeters * Definition.PolarRadiusRatio);
        if (Definition.BodyId.Equals(TEXT("earth"), ESearchCase::IgnoreCase)) Spec.Color = FLinearColor(0.03f, 0.16f, 0.65f);
        else if (Definition.BodyId.Equals(TEXT("moon"), ESearchCase::IgnoreCase)) Spec.Color = FLinearColor(0.45f, 0.45f, 0.48f);
        else if (Definition.bLuminous) Spec.Color = FLinearColor(3.0f, 2.5f, 1.2f);
        else Spec.Color = FLinearColor(0.35f, 0.25f, 0.18f);
        Actor = SpawnPlaceholder(FString::Printf(TEXT("celestial_%s"), *Definition.BodyId), Spec);
    }
    if (Actor) Actor->Tags.AddUnique(FName(*FString::Printf(TEXT("BSK.Celestial.%s"), *Definition.BodyId)));

    if (Actor && bUseEarthSkyAtmosphere && Definition.BodyId.Equals(TEXT("earth"), ESearchCase::IgnoreCase) && !EarthAtmosphere)
    {
        FActorSpawnParameters Params;
        Params.Name = MakeUniqueObjectName(GetWorld(), ASkyAtmosphere::StaticClass(), TEXT("BSK_EarthAtmosphere"));
        Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
        EarthAtmosphere = GetWorld()->SpawnActor<ASkyAtmosphere>(ASkyAtmosphere::StaticClass(), FTransform::Identity, Params);
        if (EarthAtmosphere)
        {
            USkyAtmosphereComponent* Atmosphere = EarthAtmosphere->GetComponent();
            Atmosphere->TransformMode = ESkyAtmosphereTransformMode::PlanetCenterAtComponentTransform;
            Atmosphere->SetBottomRadius(static_cast<float>(Definition.EquatorialRadiusMeters / 1000.0));
            Atmosphere->SetAtmosphereHeight(100.0f);
            Atmosphere->SetMultiScatteringFactor(1.0f);
            EarthAtmosphere->SetActorHiddenInGame(true);
            UE_LOG(LogBskUnreal, Display, TEXT("Created UE SkyAtmosphere for BSK Earth radius %.3f km"), Definition.EquatorialRadiusMeters / 1000.0);
        }
    }
    return Actor;
}

void ABskSceneController::ConfigureManifestLighting(const FBskSceneManifest& Manifest)
{
    bUseManifestSceneLighting = Manifest.bUseSceneLighting;
    SceneSunlightIntensityScale = Manifest.SunlightIntensityScale;
    const float InitialSunLux = static_cast<float>(BskCelestialLighting::ScaleIlluminanceLux(
        SunIntensityLux, SceneSunlightIntensityScale));
    UE_LOG(LogBskUnreal, Display, TEXT("Scene sunlight intensity scale=%.4f (rendering only)"),
        SceneSunlightIntensityScale);
    const double AmbientPeak = FMath::Max3(Manifest.HeadlightAmbientRgb.X, Manifest.HeadlightAmbientRgb.Y, Manifest.HeadlightAmbientRgb.Z);
    ActiveMaterialAmbient = Manifest.bUseSceneLighting ? 1.8 * AmbientPeak : 0.18;
    if (SunLight) SunLight->GetLightComponent()->SetIntensity(Manifest.bUseSceneLighting ? 0.0f : InitialSunLux);
    if (CelestialSunLight) CelestialSunLight->GetLightComponent()->SetIntensity(Manifest.bUseSceneLighting ? 0.0f : InitialSunLux);
    const double ManifestFillIntensity = Manifest.FillLightIntensityLux >= 0.0
        ? Manifest.FillLightIntensityLux
        : FillLightIntensityLux;
    if (FillLight) FillLight->GetLightComponent()->SetIntensity(
        Manifest.bUseSceneLighting ? 0.0f : static_cast<float>(ManifestFillIntensity));
    if (!Manifest.bHeadlightEnabled)
    {
        if (Headlight) Headlight->SetActorHiddenInGame(true);
        return;
    }
    if (!Headlight)
    {
        FActorSpawnParameters Params;
        Params.Name = MakeUniqueObjectName(GetWorld(), ASpotLight::StaticClass(), TEXT("BSK_MJCF_Headlight"));
        Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
        Headlight = GetWorld()->SpawnActor<ASpotLight>(ASpotLight::StaticClass(), FTransform::Identity, Params);
        if (Headlight)
        {
            if (APlayerController* Player = GetWorld()->GetFirstPlayerController())
            {
                if (ABskCameraPawn* Pawn = Cast<ABskCameraPawn>(Player->GetPawn()))
                {
                    Headlight->AttachToComponent(Pawn->GetBskCameraComponent(), FAttachmentTransformRules::SnapToTargetNotIncludingScale);
                }
            }
        }
    }
    if (!Headlight) return;
    Headlight->SetActorHiddenInGame(false);
    const double Peak = FMath::Max3(Manifest.HeadlightDiffuseRgb.X, Manifest.HeadlightDiffuseRgb.Y, Manifest.HeadlightDiffuseRgb.Z);
    const FVector3d Normalized = Peak > UE_DOUBLE_SMALL_NUMBER ? Manifest.HeadlightDiffuseRgb / Peak : FVector3d::OneVector;
    USpotLightComponent* Spot = CastChecked<USpotLightComponent>(Headlight->GetLightComponent());
    Spot->SetLightColor(FLinearColor(static_cast<float>(Normalized.X), static_cast<float>(Normalized.Y), static_cast<float>(Normalized.Z)));
    const double SpecularPeak = FMath::Max3(Manifest.HeadlightSpecularRgb.X, Manifest.HeadlightSpecularRgb.Y, Manifest.HeadlightSpecularRgb.Z);
    Spot->SetSpecularScale(static_cast<float>(Peak > UE_DOUBLE_SMALL_NUMBER ? SpecularPeak / Peak : 0.0));
    Spot->SetIntensity(static_cast<float>(1200.0 * Peak));
    Spot->SetAttenuationRadius(10000.0f);
    Spot->SetInnerConeAngle(50.0f);
    Spot->SetOuterConeAngle(80.0f);
    Spot->SetCastShadows(false);
    if (ExposureVolume)
    {
        ExposureVolume->Settings.AutoExposureBias = static_cast<float>(MaterialExposureBias);
    }
}

void ABskSceneController::ConfigureOfflineSkyLighting()
{
#if PLATFORM_LINUX
    if (!FParse::Param(FCommandLine::Get(), TEXT("BskOfflineCapture")) || !GetWorld()) return;
    // Captured-scene sky lighting triggers progressive render hitches in the
    // Linux UE 5.6 offscreen orbital scene, including without real-time capture.
    // Use the established direct-sun lighting for offline datasets. Keep clouds,
    // shadows, camera quality and interactive sky lighting configured as before.
    for (TActorIterator<ASkyLight> It(GetWorld()); It; ++It)
    {
        USkyLightComponent* Sky = It->GetLightComponent();
        if (!Sky || Sky->SourceType != SLS_CapturedScene) continue;
        Sky->SetRealTimeCapture(false);
        Sky->SetVisibility(false);
        UE_LOG(LogBskUnreal, Display, TEXT("Linux offline capture: disabled captured-scene sky light; direct sunlight and scene captures remain enabled"));
    }
#endif
}

double ABskSceneController::SolarVisibilityAt(const FVector3d& ReceiverMeters) const
{
    double Visibility = 1.0;
    for (const TPair<FVector3d, double>& Occluder : SolarOccluders)
    {
        Visibility = FMath::Min(Visibility, BskCelestialLighting::VisibleSourceFraction(
            CurrentSunPositionMeters, CurrentSunRadiusMeters, Occluder.Key, Occluder.Value, ReceiverMeters));
    }
    return Visibility;
}

void ABskSceneController::UpdateCelestialBodies(const FBskRenderFrame& Frame)
{
    // Read ALL occluders from this frame first, irrespective of message order.
    SolarOccluders.Reset();
    for (const FBskCelestialBodyState& State : Frame.CelestialBodies)
    {
        const FBskCelestialBodyDefinition* Definition = ManifestCelestialBodies.Find(State.BodyId);
        if (Definition && !Definition->bLuminous && Definition->EquatorialRadiusMeters > 0.0)
        {
            SolarOccluders.Emplace(State.PositionMeters, Definition->EquatorialRadiusMeters);
        }
    }
    for (const FBskCelestialBodyState& State : Frame.CelestialBodies)
    {
        AActor* Actor = CelestialActors.FindRef(State.BodyId);
        const FVector Location(Converter.LocalMetersToUnrealCentimeters(State.PositionMeters));
        if (Actor)
        {
            FQuat Rotation(Converter.ActiveLocalWxyzToUnreal(State.OrientationWxyz));
            if (CelestialBillboardIds.Contains(State.BodyId))
            {
                FVector ViewLocation = FVector::ZeroVector;
                FRotator ViewRotation;
                if (APlayerController* Player = GetWorld()->GetFirstPlayerController()) Player->GetPlayerViewPoint(ViewLocation, ViewRotation);
                const FVector DirectionToViewer = (ViewLocation - Location).GetSafeNormal();
                if (!DirectionToViewer.IsNearlyZero()) Rotation = DirectionToViewer.Rotation().Quaternion();
            }
            Actor->SetActorLocationAndRotation(Location, Rotation, false, nullptr, ETeleportType::TeleportPhysics);
        }
        if (EarthAtmosphere && State.BodyId.Equals(TEXT("earth"), ESearchCase::IgnoreCase))
        {
            EarthAtmosphere->SetActorLocation(Location, false, nullptr, ETeleportType::TeleportPhysics);
            EarthAtmosphere->SetActorHiddenInGame(false);
        }
        if (!bEphemerisEarthLogged && State.BodyId.Equals(TEXT("earth"), ESearchCase::IgnoreCase))
        {
            if (const FBskCelestialBodyDefinition* Definition = ManifestCelestialBodies.Find(State.BodyId))
            {
                bEphemerisEarthLogged = true;
                UE_LOG(LogBskUnreal, Display,
                    TEXT("Ephemeris Earth active center_distance_km=%.3f radius_km=%.3f local_origin_altitude_km=%.3f"),
                    State.PositionMeters.Length() / 1000.0, Definition->EquatorialRadiusMeters / 1000.0,
                    (State.PositionMeters.Length() - Definition->EquatorialRadiusMeters) / 1000.0);
            }
        }
        if (State.BodyId == PrimaryDirectionalLightBodyId)
        {
            const FBskCelestialBodyDefinition* Definition = ManifestCelestialBodies.Find(State.BodyId);
            const double SourceDistanceMeters = State.PositionMeters.Length();
            const FVector DirectionTowardSource = Location.GetSafeNormal();
            if (Definition && !DirectionTowardSource.IsNearlyZero())
            {
                CurrentSunSourceDirection = DirectionTowardSource;
                CurrentSunPositionMeters = State.PositionMeters;
                CurrentSunRadiusMeters = Definition->EquatorialRadiusMeters;
                if (DecorativeSunActor)
                {
                    DecorativeSunActor->SetActorRotation(FQuat(Converter.ActiveLocalWxyzToUnreal(State.OrientationWxyz)));
                }
                const double PreviousVisibility = CurrentSolarVisibility;
                CurrentSolarVisibility = SolarVisibilityAt(FVector3d::ZeroVector);
                if (!bEphemerisDirectionalLightActive ||
                    (PreviousVisibility <= 1.0e-6) != (CurrentSolarVisibility <= 1.0e-6))
                {
                    UE_LOG(LogBskUnreal, Display,
                        TEXT("Ephemeris solar visibility at local origin=%.6f occluders=%d"),
                        CurrentSolarVisibility, SolarOccluders.Num());
                }
                if (Definition->EquatorialRadiusMeters > 0.0 &&
                    SourceDistanceMeters > Definition->EquatorialRadiusMeters)
                {
                    const double AngularRadiusRadians = FMath::Asin(FMath::Clamp(
                        Definition->EquatorialRadiusMeters / SourceDistanceMeters, 0.0, 1.0));
                    const double AngularDiameterDegrees = FMath::RadiansToDegrees(2.0 * AngularRadiusRadians);
                    if (FMath::IsFinite(AngularDiameterDegrees) && AngularDiameterDegrees > 0.0)
                    {
                        CurrentSunAngularDiameterDegrees = FMath::Clamp(AngularDiameterDegrees, 0.001, 10.0);
                        UpdateDecorativeSunScale();
                    }
                }

                if (SunLight && !bUseManifestSceneLighting)
                {
                    const FVector LightRayDirection = BskCelestialLighting::DirectionFromSourceToTarget(Location);
                    if (!LightRayDirection.IsNearlyZero())
                    {
                        const bool bWasEphemerisActive = bEphemerisDirectionalLightActive;
                        SunLight->SetActorRotation(LightRayDirection.Rotation());
                        if (UDirectionalLightComponent* Directional = Cast<UDirectionalLightComponent>(SunLight->GetLightComponent()))
                        {
                            Directional->SetLightSourceAngle(static_cast<float>(CurrentSunAngularDiameterDegrees));
                        }
                        SunLight->GetLightComponent()->SetLightColor(FLinearColor(
                            static_cast<float>(Definition->LightColorRgb.X),
                            static_cast<float>(Definition->LightColorRgb.Y),
                            static_cast<float>(Definition->LightColorRgb.Z)));
                        const double BaseIlluminance = Definition->LightIlluminanceLuxAtReferenceDistance > 0.0
                            ? Definition->LightIlluminanceLuxAtReferenceDistance
                            : SunIntensityLux;
                        const double Illuminance = BskCelestialLighting::IlluminanceLux(
                            BaseIlluminance,
                            Definition->LightReferenceDistanceMeters,
                            SourceDistanceMeters);
                        const float UnoccludedLux = static_cast<float>(BskCelestialLighting::ScaleIlluminanceLux(
                            Illuminance * SunIlluminanceScale, SceneSunlightIntensityScale));
                        SunLight->GetLightComponent()->SetIntensity(
                            UnoccludedLux * static_cast<float>(CurrentSolarVisibility));
                        if (CelestialSunLight)
                        {
                            CelestialSunLight->SetActorRotation(LightRayDirection.Rotation());
                            CelestialSunLight->GetLightComponent()->SetLightColor(
                                SunLight->GetLightComponent()->GetLightColor());
                            CelestialSunLight->GetLightComponent()->SetIntensity(UnoccludedLux);
                        }
                        bEphemerisDirectionalLightActive = true;
                        if (!bWasEphemerisActive)
                        {
                            UE_LOG(LogBskUnreal, Display,
                                TEXT("Ephemeris Sun active distance=%.6f AU angular_diameter=%.4f deg direction=%s unoccluded_lux=%.4f local_lux=%.4f visibility=%.6f"),
                                SourceDistanceMeters / 149597870700.0,
                                CurrentSunAngularDiameterDegrees,
                                *CurrentSunSourceDirection.ToCompactString(),
                                static_cast<double>(UnoccludedLux),
                                static_cast<double>(UnoccludedLux) * CurrentSolarVisibility,
                                CurrentSolarVisibility);
                        }
                    }
                }
            }
        }
    }
}

void ABskSceneController::UpdateLightTargets()
{
    for (const TPair<FString, FBskVisualDefinition>& Pair : ManifestVisuals)
    {
        const FBskVisualDefinition& Definition = Pair.Value;
        if (!Definition.Kind.Equals(TEXT("light"), ESearchCase::IgnoreCase) || Definition.LightTargetId.IsEmpty()) continue;
        AActor* LightActor = VisualActors.FindRef(Pair.Key);
        AActor* TargetActor = BoundActors.FindRef(Definition.LightTargetId);
        if (!LightActor || !TargetActor) continue;
        const FVector Direction = (TargetActor->GetActorLocation() - LightActor->GetActorLocation()).GetSafeNormal();
        if (!Direction.IsNearlyZero()) LightActor->SetActorRotation(Direction.Rotation());
    }
}

void ABskSceneController::DrawOrbitLines(const FBskRenderFrame& Frame) const
{
    if (!bShowOrbitLines) return;
    const FBskCelestialBodyState* CentralState = nullptr;
    const FBskCelestialBodyDefinition* CentralDefinition = nullptr;
    for (const FBskCelestialBodyState& State : Frame.CelestialBodies)
    {
        const FBskCelestialBodyDefinition* Definition = ManifestCelestialBodies.Find(State.BodyId);
        if (Definition && (State.BodyId.Equals(TEXT("earth"), ESearchCase::IgnoreCase) ||
            Definition->DisplayName.Equals(TEXT("earth"), ESearchCase::IgnoreCase)))
        {
            CentralState = &State;
            CentralDefinition = Definition;
            break;
        }
    }
    if (!CentralState || !CentralDefinition || CentralDefinition->MuMetersCubedPerSecondSquared <= 0.0) return;
    constexpr int32 SegmentCount = 128;
    for (const FBskRenderObjectState& State : Frame.Objects)
    {
        const FString& Key = State.ObjectId.IsEmpty() ? State.Name : State.ObjectId;
        const FBskObjectDefinition* Definition = ManifestObjects.Find(Key);
        if (!State.bHasVelocity || (Definition && !Definition->ParentId.IsEmpty())) continue;
        const FVector3d R = State.PositionMeters - CentralState->PositionMeters;
        const FVector3d V = State.VelocityMetersPerSecond - CentralState->VelocityMetersPerSecond;
        const double Radius = R.Length();
        const FVector3d H = FVector3d::CrossProduct(R, V);
        const double HSquared = H.SquaredLength();
        if (Radius <= 1.0 || HSquared <= UE_DOUBLE_SMALL_NUMBER) continue;
        const double Mu = CentralDefinition->MuMetersCubedPerSecondSquared;
        const FVector3d EccentricityVector = FVector3d::CrossProduct(V, H) / Mu - R / Radius;
        const double Eccentricity = EccentricityVector.Length();
        const FVector3d P = Eccentricity > 1.0e-8 ? EccentricityVector / Eccentricity : R / Radius;
        const FVector3d Q = FVector3d::CrossProduct(H.GetSafeNormal(), P).GetSafeNormal();
        const double SemiLatusRectum = HSquared / Mu;
        FVector Previous;
        bool bHavePrevious = false;
        const uint32 Hash = GetTypeHash(Key);
        const FColor Color = FLinearColor::MakeFromHSV8(static_cast<uint8>(Hash & 0xff), 180, 255).ToFColor(true);
        for (int32 Index = 0; Index <= SegmentCount; ++Index)
        {
            const double Anomaly = 2.0 * PI * static_cast<double>(Index) / static_cast<double>(SegmentCount);
            const double Denominator = 1.0 + Eccentricity * FMath::Cos(Anomaly);
            if (Denominator <= 1.0e-6)
            {
                bHavePrevious = false;
                continue;
            }
            const double OrbitRadius = SemiLatusRectum / Denominator;
            const FVector3d LocalMeters = CentralState->PositionMeters + OrbitRadius * (P * FMath::Cos(Anomaly) + Q * FMath::Sin(Anomaly));
            const FVector Current(Converter.LocalMetersToUnrealCentimeters(LocalMeters));
            if (bHavePrevious) DrawDebugLine(GetWorld(), Previous, Current, Color, false, 0.0f, 0, 0.0f);
            Previous = Current;
            bHavePrevious = true;
        }
    }
}

bool ABskSceneController::CreateTexturedStarSphere()
{
    if (!bUseTexturedStarSphere) return false;
    UStaticMesh* SphereMesh = LoadObject<UStaticMesh>(nullptr, *TexturedStarMeshPath);
    UMaterialInterface* StarMaterial = LoadObject<UMaterialInterface>(nullptr, *TexturedStarMaterialPath);
    if (!SphereMesh || !StarMaterial)
    {
        UE_LOG(LogBskUnreal, Warning,
            TEXT("Textured star-sphere assets unavailable mesh=%s material=%s; falling back to Celestial Vault"),
            SphereMesh ? TEXT("ok") : TEXT("missing"), StarMaterial ? TEXT("ok") : TEXT("missing"));
        return false;
    }

    const FBoxSphereBounds MeshBounds = SphereMesh->GetBounds();
    const double SourceRadiusCentimeters = FMath::Max(
        static_cast<double>(MeshBounds.BoxExtent.GetMax()), UE_DOUBLE_SMALL_NUMBER);
    const double TargetRadiusCentimeters = CelestialVaultRadiusKilometers * 100000.0;
    const double MeshScale = TargetRadiusCentimeters / SourceRadiusCentimeters;

    DeepSkyComponent = NewObject<UStaticMeshComponent>(this, TEXT("TexturedStarSphereBackground"));
    DeepSkyComponent->SetupAttachment(GetRootComponent());
    DeepSkyComponent->SetMobility(EComponentMobility::Movable);
    DeepSkyComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    DeepSkyComponent->SetCastShadow(true);
    DeepSkyComponent->SetAffectDistanceFieldLighting(true);
    DeepSkyComponent->SetAffectDynamicIndirectLighting(true);
    DeepSkyComponent->SetReceivesDecals(true);
    DeepSkyComponent->SetRenderInMainPass(true);
    DeepSkyComponent->SetRenderInDepthPass(true);
    DeepSkyComponent->SetTranslucentSortPriority(0);
    DeepSkyComponent->SetCanEverAffectNavigation(false);
    DeepSkyComponent->SetStaticMesh(SphereMesh);
    DeepSkyComponent->SetMaterial(0, StarMaterial);
    DeepSkyComponent->SetRelativeScale3D(FVector(MeshScale));
    DeepSkyComponent->SetRelativeLocation(-MeshBounds.Origin * MeshScale);
    DeepSkyComponent->RegisterComponent();
    UE_LOG(LogBskUnreal, Display,
        TEXT("Using MyProject2 star sphere mesh=%s material=%s at %.0f km radius"),
        *TexturedStarMeshPath, *TexturedStarMaterialPath, CelestialVaultRadiusKilometers);
    return true;
}

void ABskSceneController::CreateDecorativeSun()
{
    if (!bEnableDecorativeSun) return;
    UStaticMesh* SphereMesh = LoadObject<UStaticMesh>(nullptr, *SunVisualMeshPath);
    UMaterialInterface* SunMaterial = LoadObject<UMaterialInterface>(nullptr, *SunVisualMaterialPath);
    if (!SphereMesh || !SunMaterial)
    {
        UE_LOG(LogBskUnreal, Warning,
            TEXT("Solar System Scope Sun assets unavailable mesh=%s material=%s"),
            SphereMesh ? TEXT("ok") : TEXT("missing"), SunMaterial ? TEXT("ok") : TEXT("missing"));
        return;
    }

    const double MaximumDistanceMeters = FMath::Max(1000.0, CelestialVaultRadiusKilometers * 1000.0 * 0.92);
    const double EffectiveDistanceMeters = FMath::Min(SunVisualDistanceMeters, MaximumDistanceMeters);
    const FBoxSphereBounds MeshBounds = SphereMesh->GetBounds();
    DecorativeSunSourceRadiusCentimeters = FMath::Max(
        static_cast<double>(MeshBounds.BoxExtent.GetMax()), UE_DOUBLE_SMALL_NUMBER);
    DecorativeSunSourceCenterCentimeters = FVector(MeshBounds.Origin);
    CurrentSunAngularDiameterDegrees = SunVisualAngularDiameterDegrees;

    FActorSpawnParameters Params;
    Params.Name = MakeUniqueObjectName(GetWorld(), AActor::StaticClass(), TEXT("BSK_VisibleSun"));
    Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    DecorativeSunActor = GetWorld()->SpawnActor<AActor>(AActor::StaticClass(), FTransform::Identity, Params);
    if (!DecorativeSunActor) return;

    USceneComponent* Root = NewObject<USceneComponent>(DecorativeSunActor, TEXT("SunVisualRoot"));
    Root->SetMobility(EComponentMobility::Movable);
    Root->RegisterComponent();
    DecorativeSunActor->SetRootComponent(Root);

    UStaticMeshComponent* Component = NewObject<UStaticMeshComponent>(DecorativeSunActor, TEXT("SunSurface"));
    Component->SetMobility(EComponentMobility::Movable);
    Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Component->SetCastShadow(false);
    Component->SetAffectDistanceFieldLighting(false);
    Component->SetAffectDynamicIndirectLighting(false);
    Component->SetReceivesDecals(false);
    Component->SetRenderInMainPass(true);
    Component->SetRenderInDepthPass(true);
    Component->SetCanEverAffectNavigation(false);
    Component->SetStaticMesh(SphereMesh);
    UMaterialInstanceDynamic* Material = UMaterialInstanceDynamic::Create(SunMaterial, DecorativeSunActor);
    Material->SetScalarParameterValue(TEXT("EmissiveStrength"), static_cast<float>(SunVisualEmissiveStrength));
    Component->SetMaterial(0, Material);
    Component->SetupAttachment(Root);
    Component->RegisterComponent();
    DecorativeSunComponent = Component;
    CreateDecorativeSunEffects(Root);
    UpdateDecorativeSunScale();

    DecorativeSunActor->Tags.AddUnique(FName(TEXT("BSK.Environment.SunVisual")));
    DecorativeSunActor->Tags.AddUnique(FName(TEXT("BSK.Semantic.sun")));
    UpdateDecorativeSunPlacement();
    UE_LOG(LogBskUnreal, Display,
        TEXT("Created Solar System Scope Sun visual distance=%.0f km angular_diameter=%.3f deg emissive=%.2f material=%s"),
        EffectiveDistanceMeters / 1000.0,
        CurrentSunAngularDiameterDegrees,
        SunVisualEmissiveStrength,
        *SunVisualMaterialPath);
}

void ABskSceneController::CreateDecorativeSunEffects(USceneComponent* Root)
{
    if (!bEnableDecorativeSunEffects || !DecorativeSunActor || !Root) return;

    const TArray<TPair<FString, FString>> Effects = {
        {TEXT("SunBursts"), SunBurstParticlePath},
        {TEXT("SunHalo"), SunHaloParticlePath},
        {TEXT("SunLines"), SunLinesParticlePath},
    };
    for (const TPair<FString, FString>& Effect : Effects)
    {
        if (Effect.Value.IsEmpty()) continue;
        UParticleSystem* Template = LoadObject<UParticleSystem>(nullptr, *Effect.Value);
        if (!Template)
        {
            UE_LOG(LogBskUnreal, Warning,
                TEXT("SP_space Sun particle asset unavailable name=%s path=%s"),
                *Effect.Key, *Effect.Value);
            continue;
        }

        const FName ComponentName = MakeUniqueObjectName(
            DecorativeSunActor, UParticleSystemComponent::StaticClass(), *FString::Printf(TEXT("%sParticle"), *Effect.Key));
        UParticleSystemComponent* Component = NewObject<UParticleSystemComponent>(DecorativeSunActor, ComponentName);
        if (!Component) continue;
        Component->SetMobility(EComponentMobility::Movable);
        Component->SetTemplate(Template);
        Component->bAutoActivate = true;
        Component->bAutoDestroy = false;
        Component->SetCastShadow(false);
        Component->SetReceivesDecals(false);
        Component->SetCanEverAffectNavigation(false);
        Component->SetupAttachment(Root);
        Component->RegisterComponent();
        Component->ActivateSystem(true);
        DecorativeSunParticleComponents.Add(Component);

        UE_LOG(LogBskUnreal, Display,
            TEXT("Added SP_space Sun particle effect name=%s asset=%s"),
            *Effect.Key, *Effect.Value);
    }
}

void ABskSceneController::UpdateDecorativeSunScale()
{
    if (!DecorativeSunComponent || DecorativeSunSourceRadiusCentimeters <= UE_DOUBLE_SMALL_NUMBER) return;
    const double MaximumDistanceMeters = FMath::Max(1000.0, CelestialVaultRadiusKilometers * 1000.0 * 0.92);
    const double EffectiveDistanceMeters = FMath::Min(SunVisualDistanceMeters, MaximumDistanceMeters);
    const double AngularRadiusRadians = FMath::DegreesToRadians(CurrentSunAngularDiameterDegrees * 0.5);
    const double TargetRadiusMeters = EffectiveDistanceMeters * FMath::Tan(AngularRadiusRadians);
    const double MeshScale = TargetRadiusMeters * Converter.GetCentimetersPerMeter() /
        DecorativeSunSourceRadiusCentimeters;
    DecorativeSunComponent->SetRelativeScale3D(FVector(MeshScale));
    DecorativeSunComponent->SetRelativeLocation(-DecorativeSunSourceCenterCentimeters * MeshScale);
    for (UParticleSystemComponent* Particle : DecorativeSunParticleComponents)
    {
        if (!Particle) continue;
        Particle->SetRelativeScale3D(FVector(MeshScale * SunVisualEffectScale));
    }
}

void ABskSceneController::UpdateDecorativeSunPlacement()
{
    if (!DecorativeSunActor || !SunLight) return;
    FVector ViewLocation = FVector::ZeroVector;
    FRotator ViewRotation = FRotator::ZeroRotator;
    if (APlayerController* Player = GetWorld()->GetFirstPlayerController())
    {
        Player->GetPlayerViewPoint(ViewLocation, ViewRotation);
    }
    FVector3d ViewMeters = FVector3d(ViewLocation) / Converter.GetCentimetersPerMeter();
    if (Converter.MirrorsLocalY()) ViewMeters.Y = -ViewMeters.Y;
    const double Visibility = SolarVisibilityAt(ViewMeters);
    // A near-camera Sun proxy would otherwise draw in FRONT of the Earth
    // during eclipse. Use physical geometry, not the proxy's depth.
    DecorativeSunActor->SetActorHiddenInGame(Visibility <= 1.0e-6);
    if (DecorativeSunComponent)
    {
        if (UMaterialInstanceDynamic* Material = Cast<UMaterialInstanceDynamic>(DecorativeSunComponent->GetMaterial(0)))
        {
            Material->SetScalarParameterValue(TEXT("EmissiveStrength"),
                static_cast<float>(SunVisualEmissiveStrength * Visibility));
        }
    }
    const FVector DirectionTowardSun = CurrentSunRadiusMeters > 0.0
        ? FVector(Converter.LocalMetersToUnrealCentimeters(CurrentSunPositionMeters - ViewMeters)).GetSafeNormal()
        : CurrentSunSourceDirection.GetSafeNormal();
    if (DirectionTowardSun.IsNearlyZero()) return;
    const double MaximumDistanceMeters = FMath::Max(1000.0, CelestialVaultRadiusKilometers * 1000.0 * 0.92);
    const double EffectiveDistanceMeters = FMath::Min(SunVisualDistanceMeters, MaximumDistanceMeters);
    const double DistanceCentimeters = EffectiveDistanceMeters * Converter.GetCentimetersPerMeter();
    const FVector SunLocation = ViewLocation + DirectionTowardSun * DistanceCentimeters;
    // The decorative Sun is a camera-relative render proxy, not a physical
    // body. Avoid moving it by tiny sub-pixel amounts every game tick: at the
    // old astronomical proxy distance this repeatedly invalidated LWC tiles
    // used by the attached Cascade particles and could make the main viewport
    // flash. The auxiliary SceneCapture cameras do not see this proxy, which
    // is why they did not exhibit the same symptom.
    if (!DecorativeSunActor->GetActorLocation().Equals(SunLocation, 1.0f))
    {
        DecorativeSunActor->SetActorLocation(
            SunLocation,
            false,
            nullptr,
            ETeleportType::TeleportPhysics);
    }
}

void ABskSceneController::CreateDecorativeEarth()
{
    if (!bEnableDecorativeEarth) return;
    DecorativeEarthActor = SpawnTexturedEarth(TEXT("decorative_earth"), DecorativeEarthRadiusMeters);
    if (!DecorativeEarthActor)
    {
        UE_LOG(LogBskUnreal, Warning, TEXT("Decorative Earth was requested but its textured assets could not be created"));
        return;
    }
    DecorativeEarthActor->SetActorLocation(
        FVector(Converter.LocalMetersToUnrealCentimeters(DecorativeEarthPositionMeters)),
        false, nullptr, ETeleportType::TeleportPhysics);
    DecorativeEarthActor->SetActorRotation(DecorativeEarthRotation);
    DecorativeEarthActor->Tags.AddUnique(FName(TEXT("BSK.Environment.DecorativeEarth")));
    DecorativeEarthActor->Tags.AddUnique(FName(TEXT("BSK.Semantic.earth")));
    UE_LOG(LogBskUnreal, Display, TEXT("Decorative Earth enabled position_m=%s rotation=%s"),
        *DecorativeEarthPositionMeters.ToString(), *DecorativeEarthRotation.ToCompactString());
}

void ABskSceneController::CreateEnvironment()
{
    CurrentSunSourceDirection = -SunRotation.Vector().GetSafeNormal();
    FActorSpawnParameters Params;
    Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    if (ADirectionalLight* Sun = GetWorld()->SpawnActor<ADirectionalLight>(ADirectionalLight::StaticClass(), FVector::ZeroVector, SunRotation, Params))
    {
        SunLight = Sun;
        ULightComponent* Light = Sun->GetLightComponent();
        // Reproduce the MyProject2 DirectionalLight visual properties. Mobility,
        // rotation, and runtime intensity remain dynamic so Basilisk/SPICE can
        // continue to drive the illumination direction and distance law.
        Light->SetMobility(EComponentMobility::Movable);
        Light->SetIntensity(static_cast<float>(SunIntensityLux));
        Light->SetLightColor(FLinearColor::White);
        Light->SetCastShadows(true);
        Light->CastStaticShadows = true;
        Light->CastDynamicShadows = true;
        Light->SetCastVolumetricShadow(true);
        Light->SetAffectTranslucentLighting(true);
        Light->SetTransmission(false);
        Light->SetUseTemperature(false);
        Light->SetTemperature(6500.0f);
        Light->SetIndirectLightingIntensity(1.0f);
        Light->SetVolumetricScatteringIntensity(1.0f);
        Light->SetSpecularScale(1.0f);
        Light->SetShadowBias(0.5f);
        Light->SetShadowSlopeBias(0.5f);
        Light->ContactShadowLength = 0.0f;
        Light->ContactShadowCastingIntensity = 1.0f;
        Light->ContactShadowNonCastingIntensity = 0.0f;
        if (UDirectionalLightComponent* Directional = Cast<UDirectionalLightComponent>(Light))
        {
            Directional->SetLightSourceAngle(0.5357f);
            Directional->SetLightSourceSoftAngle(0.0f);
            Directional->SetAtmosphereSunLight(true);
            Directional->SetAtmosphereSunLightIndex(0);
            Directional->SetForwardShadingPriority(1);
            Directional->bPerPixelAtmosphereTransmittance = true;
            Directional->CloudScatteredLuminanceScale = FLinearColor::White;
        }
    }

    // MuJoCo's default viewer uses a camera/headlight contribution in addition
    // to its key light.  This low-intensity, shadowless fill preserves the same
    // useful shape readability without pretending to be another physical sun.
    if (ADirectionalLight* CelestialSun = GetWorld()->SpawnActor<ADirectionalLight>(
        FVector::ZeroVector, SunRotation, Params))
    {
        CelestialSunLight = CelestialSun;
        ULightComponent* Light = CelestialSun->GetLightComponent();
        Light->SetMobility(EComponentMobility::Movable);
        Light->SetLightingChannels(false, true, false);
        Light->SetIntensity(static_cast<float>(SunIntensityLux));
        Light->SetCastShadows(false);
        Light->SetIndirectLightingIntensity(0.0f);
        Light->SetAffectTranslucentLighting(true);
        if (UDirectionalLightComponent* Directional = Cast<UDirectionalLightComponent>(Light))
        {
            Directional->SetAtmosphereSunLight(false);
        }
    }

    if (FillLightIntensityLux > 0.0)
    {
        const FRotator FillRotation(-SunRotation.Pitch * 0.5, SunRotation.Yaw + 165.0, 0.0);
        if (ADirectionalLight* Fill = GetWorld()->SpawnActor<ADirectionalLight>(ADirectionalLight::StaticClass(), FVector::ZeroVector, FillRotation, Params))
        {
            FillLight = Fill;
            Fill->GetLightComponent()->SetIntensity(static_cast<float>(FillLightIntensityLux));
            Fill->GetLightComponent()->SetLightColor(FLinearColor(0.55f, 0.68f, 1.0f));
            Fill->GetLightComponent()->SetCastShadows(false);
        }
    }

    // MyProject2 relies on project auto/local exposure and has no overriding
    // PostProcessVolume. Keep the manual path available for other deployments.
    if (bUseManualExposure)
    {
        if (APostProcessVolume* Volume = GetWorld()->SpawnActor<APostProcessVolume>(APostProcessVolume::StaticClass(), FVector::ZeroVector, FRotator::ZeroRotator, Params))
        {
            ExposureVolume = Volume;
            Volume->bUnbound = true;
            Volume->Priority = 1000.0f;
            Volume->Settings.bOverride_AutoExposureMethod = true;
            Volume->Settings.AutoExposureMethod = EAutoExposureMethod::AEM_Manual;
            Volume->Settings.bOverride_AutoExposureApplyPhysicalCameraExposure = true;
            Volume->Settings.AutoExposureApplyPhysicalCameraExposure = true;
            Volume->Settings.bOverride_CameraISO = true;
            Volume->Settings.CameraISO = 100.0f;
            Volume->Settings.bOverride_CameraShutterSpeed = true;
            Volume->Settings.CameraShutterSpeed = 125.0f;
            Volume->Settings.bOverride_DepthOfFieldFstop = true;
            Volume->Settings.DepthOfFieldFstop = 16.0f;
            Volume->Settings.bOverride_AutoExposureBias = true;
            Volume->Settings.AutoExposureBias = static_cast<float>(MaterialExposureBias);
        }
    }

    const bool bUsingTexturedStarSphere = CreateTexturedStarSphere();
    if (!bUsingTexturedStarSphere && bUseOfficialCelestialAssets)
    {
        UStaticMesh* VaultMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/CelestialVault/Meshes/SM_CelestialVault.SM_CelestialVault"));
        UMaterialInterface* VaultMaterial = LoadObject<UMaterialInterface>(nullptr, TEXT("/CelestialVault/Materials/MI_CelestialVault.MI_CelestialVault"));
        if (VaultMesh && VaultMaterial)
        {
            DeepSkyComponent = NewObject<UStaticMeshComponent>(this, TEXT("EpicCelestialVaultBackground"));
            DeepSkyComponent->SetupAttachment(GetRootComponent());
            DeepSkyComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
            DeepSkyComponent->SetCastShadow(false);
            DeepSkyComponent->SetAffectDynamicIndirectLighting(false);
            DeepSkyComponent->SetCanEverAffectNavigation(false);
            DeepSkyComponent->SetStaticMesh(VaultMesh);
            UMaterialInstanceDynamic* VaultInstance = UMaterialInstanceDynamic::Create(VaultMaterial, this);
            VaultInstance->SetScalarParameterValue(TEXT("Global Intensity"), static_cast<float>(CelestialBackgroundIntensity));
            VaultInstance->SetScalarParameterValue(TEXT("Background Intensity"), 1.0f);
            VaultInstance->SetScalarParameterValue(TEXT("Constellations Intensity"), 0.0f);
            VaultInstance->SetScalarParameterValue(TEXT("Grid Intensity"), 0.0f);
            DeepSkyComponent->SetMaterial(0, VaultInstance);
            DeepSkyComponent->SetRelativeScale3D(FVector(CelestialVaultRadiusKilometers * 1000.0));
            DeepSkyComponent->RegisterComponent();
            UE_LOG(LogBskUnreal, Display, TEXT("Using Epic Celestial Vault Milky Way background at %.0f km"), CelestialVaultRadiusKilometers);
        }
        else
        {
            UE_LOG(LogBskUnreal, Warning, TEXT("Epic Celestial Vault background assets unavailable; keeping black background"));
        }
    }

    CreateDecorativeSun();
    CreateDecorativeEarth();

    // MyProject2 uses only its textured star sphere and no additional star
    // instances. A positive StarCount remains an optional renderer extension.
    if (StarCount <= 0)
    {
        UE_LOG(LogBskUnreal, Display, TEXT("No supplemental star instances requested; matching MyProject2"));
        return;
    }

    UInstancedStaticMeshComponent* Stars = NewObject<UInstancedStaticMeshComponent>(this, TEXT("StarField"));
    Stars->SetupAttachment(GetRootComponent());
    Stars->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Stars->SetCastShadow(false);
    UStaticMesh* OfficialStarMesh = bUseOfficialCelestialAssets
        ? LoadObject<UStaticMesh>(nullptr, TEXT("/CelestialVault/Meshes/SM_Plane_FacingX.SM_Plane_FacingX"))
        : nullptr;
    UMaterialInterface* OfficialStarMaterial = bUseOfficialCelestialAssets
        ? LoadObject<UMaterialInterface>(nullptr, TEXT("/CelestialVault/Materials/MI_Stars.MI_Stars"))
        : nullptr;
    const bool bUsingOfficialStars = OfficialStarMesh && OfficialStarMaterial;
    Stars->SetStaticMesh(bUsingOfficialStars ? OfficialStarMesh : LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Sphere.Sphere")));
    if (bUsingOfficialStars)
    {
        Stars->SetNumCustomDataFloats(4);
        UMaterialInstanceDynamic* StarInstance = UMaterialInstanceDynamic::Create(OfficialStarMaterial, this);
        StarInstance->SetScalarParameterValue(TEXT("MagnitudeOffset"), -2.0f);
        StarInstance->SetScalarParameterValue(TEXT("Global Size Factor"), 1.25f);
        Stars->SetMaterial(0, StarInstance);
    }
    else if (UMaterialInterface* StarMaterial = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/BSK/M_BskUnlitColor.M_BskUnlitColor")))
    {
        UMaterialInstanceDynamic* StarInstance = UMaterialInstanceDynamic::Create(StarMaterial, this);
        StarInstance->SetVectorParameterValue(TEXT("Color"), FLinearColor(2.5f, 2.5f, 2.5f, 1.0f));
        Stars->SetMaterial(0, StarInstance);
    }
    Stars->RegisterComponent();
    FRandomStream Random(0x42534b);
    const double RadiusCm = bUsingOfficialStars
        ? CelestialVaultRadiusKilometers * 100000.0 * 0.99
        : StarRadiusMeters * Converter.GetCentimetersPerMeter();
    for (int32 Index = 0; Index < StarCount; ++Index)
    {
        FVector Direction(Random.FRandRange(-1.0f, 1.0f), Random.FRandRange(-1.0f, 1.0f), Random.FRandRange(-1.0f, 1.0f));
        if (!Direction.Normalize()) { --Index; continue; }
        if (bUsingOfficialStars)
        {
            const int32 InstanceIndex = Stars->AddInstance(FTransform(FQuat::Identity, Direction * RadiusCm));
            const float Warmth = Random.FRandRange(0.0f, 1.0f);
            const FLinearColor StarColor = FLinearColor::LerpUsingHSV(FLinearColor(0.62f, 0.75f, 1.0f), FLinearColor(1.0f, 0.78f, 0.55f), Warmth);
            TArray<float> CustomData{StarColor.R, StarColor.G, StarColor.B, static_cast<float>(Random.FRandRange(0.0f, 5.5f))};
            Stars->SetCustomData(InstanceIndex, CustomData);
        }
        else
        {
            const float Scale = Random.FRandRange(0.4f, 1.0f);
            Stars->AddInstance(FTransform(FQuat::Identity, Direction * RadiusCm, FVector(Scale)));
        }
    }
    UE_LOG(LogBskUnreal, Display, TEXT("Created %d %s stars over %s background"), StarCount,
        bUsingOfficialStars ? TEXT("Epic Celestial Vault") : TEXT("fallback"),
        bUsingTexturedStarSphere ? TEXT("textured Milky Way") : TEXT("Celestial Vault/black"));
}
