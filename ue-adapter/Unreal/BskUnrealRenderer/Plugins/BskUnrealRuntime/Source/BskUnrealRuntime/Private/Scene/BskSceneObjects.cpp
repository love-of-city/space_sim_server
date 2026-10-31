#include "BskSceneController.h"
#include "BskUnrealRuntime.h"
#include "BskRenderWorldSubsystem.h"
#include "Scene/BskSceneUtilities.h"
#include "Engine/World.h"
#include "Dom/JsonObject.h"
#include "Misc/CommandLine.h"
#include "Misc/ConfigCacheIni.h"
#include "Misc/Paths.h"
#include "Misc/Parse.h"
#include "Capture/BskCapturePipeline.h"
#include "BskCameraPawn.h"
#include "Camera/CameraActor.h"
#include "Camera/CameraComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/PointLightComponent.h"
#include "Components/SpotLightComponent.h"
#include "Components/LightComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Particles/ParticleSystemComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/SkeletalMesh.h"
#include "Animation/SkeletalMeshActor.h"
#include "Engine/SpotLight.h"
#include "Engine/PointLight.h"
#include "Engine/DirectionalLight.h"
#include "Engine/Light.h"
#include "Engine/TextureRenderTarget2D.h"
#include "EngineUtils.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "UObject/UnrealType.h"
#include "Kismet/GameplayStatics.h"

using namespace BskSceneUtilities;

ABskSceneController::FObjectSpec ABskSceneController::ResolveSpec(const FBskRenderObjectState& State) const
{
    if (const FBskObjectDefinition* Definition = ManifestObjects.Find(State.ObjectId))
    {
        if (const FObjectSpec* ById = ObjectSpecs.Find(Definition->ObjectId)) return *ById;
        if (const FObjectSpec* ByName = ObjectSpecs.Find(Definition->DisplayName)) return *ByName;
        if (!Definition->AssetPath.IsEmpty())
        {
            FObjectSpec Result;
            Result.AssetType = TEXT("auto");
            Result.AssetPath = Definition->AssetPath;
            return Result;
        }
    }
    if (const FObjectSpec* Configured = ObjectSpecs.Find(State.Name))
    {
        return *Configured;
    }
    FObjectSpec Result;
    if (!State.AssetPath.IsEmpty())
    {
        Result.AssetType = State.AssetPath.EndsWith(TEXT(".usd"), ESearchCase::IgnoreCase) ||
            State.AssetPath.EndsWith(TEXT(".usda"), ESearchCase::IgnoreCase) ||
            State.AssetPath.EndsWith(TEXT(".usdc"), ESearchCase::IgnoreCase) ? TEXT("usd") : TEXT("auto");
        Result.AssetPath = State.AssetPath;
    }
    const uint32 Hash = GetTypeHash(State.Name);
    Result.Color = FLinearColor::MakeFromHSV8(static_cast<uint8>(Hash & 0xff), 190, 240);
    return Result;
}

AActor* ABskSceneController::FindBoundActor(const FString& ObjectName) const
{
    TArray<AActor*> Tagged;
    UGameplayStatics::GetAllActorsWithTag(GetWorld(), FName(*BindingTag(ObjectName)), Tagged);
    if (!Tagged.IsEmpty())
    {
        return Tagged[0];
    }
    TArray<AActor*> AllActors;
    UGameplayStatics::GetAllActorsOfClass(GetWorld(), AActor::StaticClass(), AllActors);
    for (AActor* Actor : AllActors)
    {
        if (Actor && Actor->GetName().Equals(ObjectName, ESearchCase::IgnoreCase))
        {
            return Actor;
        }
    }
    return nullptr;
}

AActor* ABskSceneController::GetOrCreateActor(const FBskRenderObjectState& State)
{
    const FString& Key = State.ObjectId.IsEmpty() ? State.Name : State.ObjectId;
    if (TObjectPtr<AActor>* Existing = BoundActors.Find(Key))
    {
        return Existing->Get();
    }
    AActor* Actor = FindBoundActor(Key);
    if (!Actor)
    {
        if (const FBskObjectDefinition* Definition = ManifestObjects.Find(Key)) Actor = SpawnManifestObject(*Definition);
        if (!Actor) Actor = SpawnFromSpec(Key, ResolveSpec(State));
        if (Actor) OwnedObjectIds.Add(Key);
    }
    if (Actor)
    {
        Actor->Tags.AddUnique(FName(*BindingTag(Key)));
        if (!State.SemanticLabel.IsEmpty()) Actor->Tags.AddUnique(FName(*FString::Printf(TEXT("BSK.Semantic.%s"), *State.SemanticLabel)));
        BoundActors.Add(Key, Actor);
    }
    return Actor;
}

AActor* ABskSceneController::SpawnFromSpec(const FString& ObjectName, const FObjectSpec& Spec)
{
    FActorSpawnParameters Params;
    Params.Name = MakeUniqueObjectName(GetWorld(), AActor::StaticClass(), SafeActorName(ObjectName));
    Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

    if (!Spec.ActorClass.IsEmpty())
    {
        if (UClass* Class = FSoftClassPath(Spec.ActorClass).TryLoadClass<AActor>())
        {
            AActor* Actor = GetWorld()->SpawnActor<AActor>(Class, FTransform::Identity, Params);
            if (Actor) Actor->SetActorScale3D(FVector(Spec.Scale));
            return Actor;
        }
    }
    if (Spec.AssetType.Equals(TEXT("usd"), ESearchCase::IgnoreCase))
    {
        if (!bAllowExternalAssets && !Spec.AssetPath.StartsWith(TEXT("/Game/")))
        {
            UE_LOG(LogBskUnreal, Warning, TEXT("External asset loading is disabled for %s: %s"), *ObjectName, *Spec.AssetPath);
            return SpawnPlaceholder(ObjectName, Spec);
        }
        if (AActor* Actor = SpawnUsdStage(ObjectName, Spec)) return Actor;
    }
    if ((Spec.AssetType.Equals(TEXT("static_mesh"), ESearchCase::IgnoreCase) || Spec.AssetType.Equals(TEXT("auto"), ESearchCase::IgnoreCase)) && !Spec.AssetPath.IsEmpty())
    {
        if (UStaticMesh* Mesh = Cast<UStaticMesh>(FSoftObjectPath(Spec.AssetPath).TryLoad()))
        {
            AStaticMeshActor* Actor = GetWorld()->SpawnActor<AStaticMeshActor>(AStaticMeshActor::StaticClass(), FTransform::Identity, Params);
            Actor->GetStaticMeshComponent()->SetStaticMesh(Mesh);
            Actor->GetStaticMeshComponent()->SetMobility(EComponentMobility::Movable);
            Actor->SetActorScale3D(FVector(Spec.Scale));
            return Actor;
        }
    }
    if (Spec.AssetType.Equals(TEXT("skeletal_mesh"), ESearchCase::IgnoreCase) && !Spec.AssetPath.IsEmpty())
    {
        if (USkeletalMesh* Mesh = Cast<USkeletalMesh>(FSoftObjectPath(Spec.AssetPath).TryLoad()))
        {
            ASkeletalMeshActor* Actor = GetWorld()->SpawnActor<ASkeletalMeshActor>(ASkeletalMeshActor::StaticClass(), FTransform::Identity, Params);
            Actor->GetSkeletalMeshComponent()->SetSkeletalMeshAsset(Mesh);
            Actor->SetActorScale3D(FVector(Spec.Scale));
            return Actor;
        }
    }
    if (!Spec.AssetPath.IsEmpty())
    {
        UE_LOG(LogBskUnreal, Warning, TEXT("Could not load '%s' for %s; using placeholder"), *Spec.AssetPath, *ObjectName);
    }
    return SpawnPlaceholder(ObjectName, Spec);
}

AActor* ABskSceneController::SpawnUsdStage(const FString& ObjectName, const FObjectSpec& Spec)
{
    UClass* StageClass = FSoftClassPath(TEXT("/Script/USDStage.UsdStageActor")).TryLoadClass<AActor>();
    if (!StageClass)
    {
        UE_LOG(LogBskUnreal, Warning, TEXT("USDStage runtime module is unavailable for %s. Enable UE's USD Importer plugin or map an imported asset."), *ObjectName);
        return nullptr;
    }
    const FTransform Transform(FQuat::Identity, FVector::ZeroVector, FVector(Spec.Scale));
    AActor* Actor = GetWorld()->SpawnActorDeferred<AActor>(StageClass, Transform, nullptr, nullptr, ESpawnActorCollisionHandlingMethod::AlwaysSpawn);
    if (!Actor) return nullptr;
    if (FStructProperty* RootLayer = FindFProperty<FStructProperty>(StageClass, TEXT("RootLayer")))
    {
        if (FFilePath* FilePath = RootLayer->ContainerPtrToValuePtr<FFilePath>(Actor)) FilePath->FilePath = Spec.AssetPath;
    }
    UGameplayStatics::FinishSpawningActor(Actor, Transform);
    return Actor;
}

AActor* ABskSceneController::SpawnPlaceholder(const FString& ObjectName, const FObjectSpec& Spec)
{
    FActorSpawnParameters Params;
    Params.Name = MakeUniqueObjectName(GetWorld(), AStaticMeshActor::StaticClass(), SafeActorName(ObjectName));
    Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    AStaticMeshActor* Actor = GetWorld()->SpawnActor<AStaticMeshActor>(AStaticMeshActor::StaticClass(), FTransform::Identity, Params);
    if (!Actor) return nullptr;
    const TCHAR* MeshPath = PrimitiveMeshPath(Spec.PlaceholderShape);
    UStaticMeshComponent* Component = Actor->GetStaticMeshComponent();
    Component->SetMobility(EComponentMobility::Movable);
    UStaticMesh* PlaceholderMesh = LoadObject<UStaticMesh>(nullptr, MeshPath);
    if (!PlaceholderMesh || !Component->SetStaticMesh(PlaceholderMesh))
    {
        UE_LOG(LogBskUnreal, Error, TEXT("Failed to assign placeholder mesh %s to %s"), MeshPath, *ObjectName);
    }
    Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    ApplyUnlitColor(Component, Actor, Spec.Color);
    // Engine basic shapes are one meter across, so a size in meters is also
    // their dimensionless actor scale.
    Actor->SetActorScale3D(FVector(Spec.SizeMeters));
    UE_LOG(LogBskUnreal, Display, TEXT("Created visible placeholder %s shape=%s size_m=%s color=%s"),
        *ObjectName, *Spec.PlaceholderShape, *Spec.SizeMeters.ToString(), *Spec.Color.ToString());
    return Actor;
}

void ABskSceneController::ApplyManifest(const FBskSceneManifest& Manifest)
{
    const bool bSessionChanged = ActiveSessionId != Manifest.SessionId;
    if (bSessionChanged) CancelCaptureJobs();
    check(IsInGameThread());
    if (!ActiveSessionId.IsEmpty() && ActiveSessionId != Manifest.SessionId)
    {
        PendingCommandIds.Reset();
        LastCommandStatus = TEXT("session changed");
        MissionEventHistory.Reset();
        bAutoCommandSent = false;
    }
    if (bSessionChanged) ResetPresentationState();
    TMap<FString, const FBskObjectDefinition*> IncomingObjects;
    for (const FBskObjectDefinition& Definition : Manifest.Objects) IncomingObjects.Add(Definition.ObjectId, &Definition);
    TMap<FString, FTransform> ReplacedObjectTransforms;
    for (auto It = OwnedObjectIds.CreateIterator(); It; ++It)
    {
        const FString ObjectId = *It;
        const FBskObjectDefinition* const* Incoming = IncomingObjects.Find(ObjectId);
        const FBskObjectDefinition* Previous = ManifestObjects.Find(ObjectId);
        if (!bSessionChanged && Incoming && Previous && SameObjectRenderDefinition(*Previous, **Incoming)) continue;
        if (AActor* Actor = BoundActors.FindRef(ObjectId))
        {
            if (Incoming && !bSessionChanged) ReplacedObjectTransforms.Add(ObjectId, Actor->GetActorTransform());
            // Preserve camera/visual and unchanged child actors until their
            // new manifest mounts are configured and attached below.
            TArray<AActor*> Children;
            Actor->GetAttachedActors(Children);
            for (AActor* Child : Children) Child->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
            Actor->Tags.Remove(FName(*BindingTag(ObjectId)));
            Actor->SetActorHiddenInGame(true);
            Actor->Destroy();
        }
        BoundActors.Remove(ObjectId);
        It.RemoveCurrent();
    }
    TMap<FString, const FBskVisualDefinition*> IncomingVisuals;
    for (const FBskVisualDefinition& Definition : Manifest.Visuals) IncomingVisuals.Add(Definition.VisualId, &Definition);
    for (auto It = VisualActors.CreateIterator(); It; ++It)
    {
        const FBskVisualDefinition* const* Incoming = IncomingVisuals.Find(It.Key());
        const FBskVisualDefinition* Previous = ManifestVisuals.Find(It.Key());
        if (!bSessionChanged && Incoming && Previous && SameVisualActorDefinition(*Previous, **Incoming)) continue;
        if (AActor* Actor = It.Value())
        {
            Actor->SetActorHiddenInGame(true);
            Actor->Destroy();
        }
        It.RemoveCurrent();
    }
    ActiveSessionId = Manifest.SessionId;
    ActiveManifestRevision = Manifest.Revision;
    bShowOrbitLines = Manifest.bOrbitLines;
    ConfigureManifestLighting(Manifest);
    InterpolationDelaySeconds = FMath::Max(0.001, Manifest.InterpolationDelayMilliseconds / 1000.0);
    MaxExtrapolationSeconds = FMath::Max(0.0, Manifest.MaxExtrapolationMilliseconds / 1000.0);
    ManifestObjects.Reset();
    ManifestCelestialBodies.Reset();
    PrimaryDirectionalLightBodyId.Reset();
    bEphemerisDirectionalLightActive = false;
    bEphemerisEarthLogged = false;
    CurrentSunPositionMeters = FVector3d::ZeroVector;
    CurrentSunRadiusMeters = 0.0;
    CurrentSolarVisibility = 1.0;
    SolarOccluders.Reset();
    CurrentSunAngularDiameterDegrees = SunVisualAngularDiameterDegrees;
    UpdateDecorativeSunScale();
    ManifestVisuals.Reset();
    ManifestCameras.Reset();
    VisualBaseRotations.Reset();
    VisualBaseScales.Reset();
    VisualDynamicVisibility.Reset();
    if (UBskRenderWorldSubsystem* RenderSubsystem = GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>())
    {
        RenderSubsystem->AcceptManifest(Manifest);
    }
    for (const FBskObjectDefinition& Definition : Manifest.Objects)
    {
        ManifestObjects.Add(Definition.ObjectId, Definition);
        FBskRenderObjectState State;
        State.ObjectId = Definition.ObjectId;
        State.Name = Definition.DisplayName;
        State.SemanticLabel = Definition.SemanticLabel;
        if (AActor* Actor = GetOrCreateActor(State))
        {
            if (const FTransform* PreviousTransform = ReplacedObjectTransforms.Find(Definition.ObjectId))
            {
                Actor->SetActorTransform(*PreviousTransform, false, nullptr, ETeleportType::TeleportPhysics);
            }
        }
    }
    const bool bManifestHasEarth = Manifest.CelestialBodies.ContainsByPredicate([](const FBskCelestialBodyDefinition& Definition)
    {
        return Definition.BodyId.Equals(TEXT("earth"), ESearchCase::IgnoreCase) ||
            Definition.DisplayName.Equals(TEXT("earth"), ESearchCase::IgnoreCase);
    });
    if (DecorativeEarthActor)
    {
        DecorativeEarthActor->SetActorHiddenInGame(bManifestHasEarth);
        UE_LOG(LogBskUnreal, Display, TEXT("Decorative Earth %s because the manifest %s an ephemeris Earth"),
            bManifestHasEarth ? TEXT("hidden") : TEXT("visible"),
            bManifestHasEarth ? TEXT("contains") : TEXT("does not contain"));
    }
    for (const TPair<FString, TObjectPtr<AActor>>& Pair : CelestialActors)
    {
        if (Pair.Key.Equals(TEXT("earth"), ESearchCase::IgnoreCase) && Pair.Value)
        {
            Pair.Value->SetActorHiddenInGame(!bManifestHasEarth);
        }
    }
    for (const FBskCelestialBodyDefinition& Definition : Manifest.CelestialBodies)
    {
        ManifestCelestialBodies.Add(Definition.BodyId, Definition);
        if (Definition.bDrivesDirectionalLight)
        {
            if (PrimaryDirectionalLightBodyId.IsEmpty())
            {
                PrimaryDirectionalLightBodyId = Definition.BodyId;
                UE_LOG(LogBskUnreal, Display,
                    TEXT("Celestial body '%s' will drive the primary directional light from ephemeris frames"),
                    *Definition.BodyId);
            }
            else
            {
                UE_LOG(LogBskUnreal, Warning,
                    TEXT("Ignoring additional primary directional-light body '%s'; '%s' is already selected"),
                    *Definition.BodyId, *PrimaryDirectionalLightBodyId);
            }
        }
        const bool bUseDecorativeSunProxy = Definition.bDrivesDirectionalLight;
        if (bUseDecorativeSunProxy)
        {
            if (AActor* ExistingActor = CelestialActors.FindRef(Definition.BodyId))
            {
                ExistingActor->SetActorHiddenInGame(true);
            }
            UE_LOG(LogBskUnreal, Display,
                TEXT("Celestial body '%s' drives lighting and will not spawn at ephemeris distance; decorative proxy=%s"),
                *Definition.BodyId, DecorativeSunActor ? TEXT("enabled") : TEXT("disabled"));
        }
        else if (!CelestialActors.Contains(Definition.BodyId))
        {
            if (AActor* Actor = SpawnCelestialBody(Definition))
            {
                // Celestial surfaces still receive sunlight when the spacecraft
                // is eclipsed. Keep them off the local-object lighting channel.
                TArray<UPrimitiveComponent*> Components;
                Actor->GetComponents<UPrimitiveComponent>(Components);
                for (UPrimitiveComponent* Component : Components)
                {
                    Component->SetLightingChannels(false, true, false);
                    // Planetary umbra uses ephemeris geometry below; a 6,378 km
                    // mesh must not enter the metre-scale arm shadow map.
                    Component->SetCastShadow(false);
                }
                CelestialActors.Add(Definition.BodyId, Actor);
            }
        }
    }
    if (PrimaryDirectionalLightBodyId.IsEmpty())
    {
        CurrentSunSourceDirection = -SunRotation.Vector().GetSafeNormal();
        if (SunLight) SunLight->SetActorRotation(SunRotation);
        UE_LOG(LogBskUnreal, Warning,
            TEXT("Manifest has no ephemeris-driven directional light; using configured fallback rotation %s"),
            *SunRotation.ToCompactString());
    }
    for (const FBskVisualDefinition& Definition : Manifest.Visuals)
    {
        ManifestVisuals.Add(Definition.VisualId, Definition);
        if (!VisualActors.Contains(Definition.VisualId))
        {
            if (AActor* Actor = SpawnVisual(Definition)) VisualActors.Add(Definition.VisualId, Actor);
        }
        if (AActor* Actor = VisualActors.FindRef(Definition.VisualId))
        {
            ApplyVisualMountTransform(Actor, Definition);
            const FQuat BaseRotation = Actor->GetRootComponent()
                ? Actor->GetRootComponent()->GetRelativeRotation().Quaternion()
                : FQuat::Identity;
            VisualBaseRotations.Add(Definition.VisualId, BaseRotation);
            VisualBaseScales.Add(Definition.VisualId, Actor->GetActorRelativeScale3D());
            VisualDynamicVisibility.Add(Definition.VisualId, true);
            RefreshVisualVisibility(Definition.VisualId);
        }
    }
    for (const FBskCameraDefinition& Definition : Manifest.Cameras)
    {
        ManifestCameras.Add(Definition.CameraId, Definition);
        AActor* CameraActor = CameraActors.FindRef(Definition.CameraId);
        if (!CameraActor)
        {
            CameraActor = SpawnCamera(Definition);
            if (CameraActor)
            {
                CameraActors.Add(Definition.CameraId, CameraActor);
                if (UBskRenderWorldSubsystem* RenderSubsystem = GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>())
                {
                    RenderSubsystem->RegisterCameraForCapture(Definition.CameraId, CameraActor);
                }
            }
        }
        if (CameraActor) ConfigureCamera(CameraActor, Definition);
    }
    AttachManifestChildren();
    if (APlayerController* Player = GetWorld() ? GetWorld()->GetFirstPlayerController() : nullptr)
    {
        if (ABskCameraPawn* Pawn = Cast<ABskCameraPawn>(Player->GetPawn()))
        {
            Pawn->SetOrbitDistanceMeters(Manifest.DefaultCameraDistanceMeters);
        }
    }
    const FString CameraTarget = Manifest.DefaultCameraTarget.IsEmpty()
        ? Manifest.OriginObjectId
        : Manifest.DefaultCameraTarget;
    if (!CameraTarget.IsEmpty()) FocusObject(CameraTarget, false);
    if (UBskRenderWorldSubsystem* RenderSubsystem = GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>())
    {
        RenderSubsystem->NotifyManifestApplied(Manifest);
    }
    if (!AutoCommand.IsEmpty() && !bAutoCommandSent)
    {
        const FBskUiCommandDefinition* Definition = Manifest.UiCommands.FindByPredicate(
            [this](const FBskUiCommandDefinition& Item) { return Item.Command == AutoCommand; });
        FString Error;
        if (!Definition || !SendUiCommand(AutoCommand, Definition->TargetId, Definition->PayloadJson, Error))
        {
            UE_LOG(LogBskUnreal, Error, TEXT("Automatic BSK command '%s' failed: %s"), *AutoCommand, *Error);
        }
        else
        {
            bAutoCommandSent = true;
        }
    }
    int32 GeometryCount = 0;
    for (const FBskObjectDefinition& Definition : Manifest.Objects) GeometryCount += Definition.Geometries.Num();
    UE_LOG(LogBskUnreal, Display, TEXT("Applied BSK scene manifest session=%s revision=%lld objects=%d geoms/visuals=%d/%d celestial=%d cameras=%d"),
        *Manifest.SessionId, Manifest.Revision, Manifest.Objects.Num(), GeometryCount,
        Manifest.Visuals.Num(), Manifest.CelestialBodies.Num(), Manifest.Cameras.Num());
}

AActor* ABskSceneController::SpawnManifestObject(const FBskObjectDefinition& Definition)
{
    if (UBskRenderWorldSubsystem* RenderSubsystem = GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>())
    {
        const FBskRenderSpawnContext Context{GetWorld(), this, &Converter};
        if (AActor* Actor = RenderSubsystem->TrySpawnObject(Context, Definition)) return Actor;
    }

    FBskRenderObjectState State;
    State.ObjectId = Definition.ObjectId;
    State.Name = Definition.DisplayName;
    State.AssetPath = Definition.AssetPath;
    const bool bHasConfiguredSpec = ObjectSpecs.Contains(Definition.ObjectId) || ObjectSpecs.Contains(Definition.DisplayName) || !Definition.AssetPath.IsEmpty();
    if (bHasConfiguredSpec) return SpawnFromSpec(Definition.ObjectId, ResolveSpec(State));
    // An explicit empty manifest body is a transform anchor, not a missing
    // asset. Keep it mesh-free so joints/sites cannot occlude the scene.

    FActorSpawnParameters Params;
    Params.Name = MakeUniqueObjectName(GetWorld(), AActor::StaticClass(), SafeActorName(Definition.ObjectId));
    Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    AActor* Actor = GetWorld()->SpawnActor<AActor>(AActor::StaticClass(), FTransform::Identity, Params);
    if (!Actor) return nullptr;
    USceneComponent* Root = NewObject<USceneComponent>(Actor, TEXT("BodyRoot"));
    Root->SetMobility(EComponentMobility::Movable);
    Root->RegisterComponent();
    Actor->SetRootComponent(Root);
    TMap<int32, UStaticMesh*> ResolvedMeshAssets;
    bool bHasRenderableVisualMesh = false;
    for (int32 Index = 0; Index < Definition.Geometries.Num(); ++Index)
    {
        const FBskGeometryDefinition& Geometry = Definition.Geometries[Index];
        if (!Geometry.Shape.Equals(TEXT("mesh"), ESearchCase::IgnoreCase) || Geometry.AssetPath.IsEmpty()) continue;
        if (UStaticMesh* Mesh = LoadObject<UStaticMesh>(nullptr, *Geometry.AssetPath))
        {
            ResolvedMeshAssets.Add(Index, Mesh);
            if (!Geometry.RenderRole.Equals(TEXT("collision"), ESearchCase::IgnoreCase)) bHasRenderableVisualMesh = true;
        }
        else
        {
            UE_LOG(LogBskUnreal, Warning, TEXT("Unable to load mesh '%s' for geometry '%s'; collision fallback will be used"),
                *Geometry.AssetPath, *Geometry.GeometryId);
        }
    }
    for (int32 Index = 0; Index < Definition.Geometries.Num(); ++Index)
    {
        const FBskGeometryDefinition& Geometry = Definition.Geometries[Index];
        const bool bCollisionGeometry = Geometry.RenderRole.Equals(TEXT("collision"), ESearchCase::IgnoreCase);
        if (bCollisionGeometry && bHasRenderableVisualMesh) continue;
        UStaticMesh* Mesh = ResolvedMeshAssets.FindRef(Index);
        if (!Mesh && Geometry.Shape.Equals(TEXT("mesh"), ESearchCase::IgnoreCase)) continue;
        if (!Mesh) Mesh = LoadObject<UStaticMesh>(nullptr, PrimitiveMeshPath(Geometry.Shape));
        if (!Mesh) continue;
        UStaticMeshComponent* Component = NewObject<UStaticMeshComponent>(Actor, *FString::Printf(TEXT("Geometry_%d"), Index));
        Component->SetMobility(EComponentMobility::Movable);
        Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        Component->SetCastShadow(true);
        Component->SetStaticMesh(Mesh);
        Component->SetupAttachment(Root);
        Component->RegisterComponent();
        Component->SetRelativeLocation(FVector(Converter.LocalMetersToUnrealCentimeters(Geometry.PositionBodyMeters)));
        Component->SetRelativeRotation(FQuat(Converter.ActiveLocalWxyzToUnreal(Geometry.OrientationBodyFromGeometryWxyz)));
        Component->SetRelativeScale3D(FVector(ResolvedMeshAssets.Contains(Index) ? Geometry.Scale : Geometry.DimensionsMeters));
        ApplyGeometryMaterial(Component, Actor, Geometry, ActiveMaterialAmbient);
        AttachConfiguredVisualOverlay(Component, Geometry);
    }
    return Actor;
}

AActor* ABskSceneController::SpawnVisual(const FBskVisualDefinition& Definition)
{
    if (UBskRenderWorldSubsystem* RenderSubsystem = GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>())
    {
        const FBskRenderSpawnContext Context{GetWorld(), this, &Converter};
        if (AActor* Actor = RenderSubsystem->TrySpawnVisual(Context, Definition)) return Actor;
    }

    if (Definition.Kind.Equals(TEXT("light"), ESearchCase::IgnoreCase))
    {
        FActorSpawnParameters Params;
        Params.Name = MakeUniqueObjectName(GetWorld(), AActor::StaticClass(), SafeActorName(Definition.VisualId));
        Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
        AActor* LightActor = nullptr;
        ULightComponent* LightComponent = nullptr;
        if (Definition.LightType.Equals(TEXT("directional"), ESearchCase::IgnoreCase))
        {
            ADirectionalLight* Light = GetWorld()->SpawnActor<ADirectionalLight>(ADirectionalLight::StaticClass(), FTransform::Identity, Params);
            LightActor = Light;
            LightComponent = Light ? Light->GetLightComponent() : nullptr;
            if (LightComponent) LightComponent->SetIntensity(static_cast<float>(10.0 * FMath::Max(0.0, Definition.LightIntensity)));
        }
        else
        {
            ASpotLight* Light = GetWorld()->SpawnActor<ASpotLight>(ASpotLight::StaticClass(), FTransform::Identity, Params);
            LightActor = Light;
            LightComponent = Light ? Light->GetLightComponent() : nullptr;
            if (USpotLightComponent* Spot = Light ? Cast<USpotLightComponent>(Light->GetLightComponent()) : nullptr)
            {
                // A spawned spot light is Stationary, and USpotLightComponent::
                // SetInnerConeAngle/SetOuterConeAngle silently do nothing for a
                // Stationary light (AreDynamicDataChangesAllowed(false)), which
                // pinned every MJCF spot light to Unreal's default 0/44 degree
                // cone whatever <light cutoff> said. Make it Movable first.
                Spot->SetMobility(EComponentMobility::Movable);
                Spot->SetIntensity(static_cast<float>(650.0 * FMath::Max(0.0, Definition.LightIntensity)));
                const double RangeMeters = Definition.RangeMeters > 0.0 ? Definition.RangeMeters : 20.0;
                Spot->SetAttenuationRadius(static_cast<float>(RangeMeters * Converter.GetCentimetersPerMeter()));
                Spot->SetInnerConeAngle(static_cast<float>(FMath::Clamp(Definition.LightCutoffDegrees * 0.65, 0.0, 80.0)));
                Spot->SetOuterConeAngle(static_cast<float>(FMath::Clamp(Definition.LightCutoffDegrees, 1.0, 89.0)));
            }
        }
        if (!LightActor || !LightComponent) return nullptr;
        // MJCF target-body lights are rotated as their target moves. Unreal
        // defaults spawned light components to a non-movable mobility, which
        // otherwise emits a warning on every rendered frame and stalls the
        // Game Thread with log I/O.
        LightComponent->SetMobility(EComponentMobility::Movable);
        const double Peak = FMath::Max3(Definition.LightDiffuseRgb.X, Definition.LightDiffuseRgb.Y, Definition.LightDiffuseRgb.Z);
        const FVector3d Normalized = Peak > UE_DOUBLE_SMALL_NUMBER ? Definition.LightDiffuseRgb / Peak : FVector3d::OneVector;
        LightComponent->SetLightColor(FLinearColor(static_cast<float>(Normalized.X), static_cast<float>(Normalized.Y), static_cast<float>(Normalized.Z)));
        const double SpecularPeak = FMath::Max3(Definition.LightSpecularRgb.X, Definition.LightSpecularRgb.Y, Definition.LightSpecularRgb.Z);
        LightComponent->SetSpecularScale(static_cast<float>(Peak > UE_DOUBLE_SMALL_NUMBER ? SpecularPeak / Peak : 0.0));
        LightComponent->SetCastShadows(Definition.bLightCastShadows);
        ApplyVisualMountTransform(LightActor, Definition);
        LightActor->Tags.AddUnique(FName(*FString::Printf(TEXT("BSK.Light.%s"), *Definition.VisualId)));
        return LightActor;
    }

    FObjectSpec Spec;
    Spec.PlaceholderShape = Definition.Kind == TEXT("reaction_wheel") ? TEXT("cylinder") : TEXT("cone");
    Spec.Color = Definition.Color;
    const double LengthMeters = Definition.RangeMeters > 0.0 ? Definition.RangeMeters : FMath::Max(Definition.SizeMeters, 0.1);
    if (Definition.Kind == TEXT("reaction_wheel"))
    {
        Spec.SizeMeters = FVector3d(
            FMath::Max(Definition.SizeMeters, 0.05),
            FMath::Max(Definition.SizeMeters, 0.05),
            FMath::Max(LengthMeters, 0.02));
    }
    else
    {
        const double FovRadians = Definition.FieldOfViewRadians.X > 0.0 ? Definition.FieldOfViewRadians.X : PI / 6.0;
        const double DiameterMeters = 2.0 * LengthMeters * FMath::Tan(0.5 * FovRadians);
        Spec.SizeMeters = FVector3d(FMath::Max(DiameterMeters, 0.02), FMath::Max(DiameterMeters, 0.02), LengthMeters);
    }
    AActor* Actor = SpawnPlaceholder(FString::Printf(TEXT("visual_%s"), *Definition.VisualId), Spec);
    if (!Actor) return nullptr;
    if (UStaticMeshComponent* Component = Actor->FindComponentByClass<UStaticMeshComponent>())
    {
        Component->SetCastShadow(false);
    }
    ApplyVisualMountTransform(Actor, Definition);
    return Actor;
}

void ABskSceneController::ApplyVisualMountTransform(AActor* Actor, const FBskVisualDefinition& Definition) const
{
    if (!Actor) return;
    FVector Location = FVector(Converter.LocalMetersToUnrealCentimeters(Definition.PositionBodyMeters));
    const FVector3d LocalNormal = Definition.NormalBody.GetSafeNormal();
    const FVector Direction = FVector(Converter.LocalMetersToUnrealCentimeters(LocalNormal)).GetSafeNormal();
    FQuat Rotation = Direction.IsNearlyZero() ? FQuat::Identity : FRotationMatrix::MakeFromZ(Direction).ToQuat();
    if (Definition.Kind.Equals(TEXT("light"), ESearchCase::IgnoreCase))
    {
        Rotation = Direction.IsNearlyZero() ? FQuat::Identity : Direction.Rotation().Quaternion();
    }
    if (!Direction.IsNearlyZero() && IsSensorFrustumKind(Definition.Kind))
    {
        const double LengthMeters = Definition.RangeMeters > 0.0
            ? Definition.RangeMeters
            : FMath::Max(Definition.SizeMeters, 0.1);
        // UE's basic cone is centered and points along +Z. Put its apex at the
        // sensor mount and let the volume expand along the sensor boresight.
        Location += FVector(Converter.LocalMetersToUnrealCentimeters(LocalNormal * (0.5 * LengthMeters)));
        Rotation = FRotationMatrix::MakeFromZ(-Direction).ToQuat();
    }
    Actor->SetActorRelativeLocation(Location);
    Actor->SetActorRelativeRotation(Rotation);
}

void ABskSceneController::RefreshVisualVisibility(const FString& VisualId)
{
    AActor* Actor = VisualActors.FindRef(VisualId);
    const FBskVisualDefinition* Definition = ManifestVisuals.Find(VisualId);
    if (!Actor || !Definition) return;
    const bool* DynamicVisibility = VisualDynamicVisibility.Find(VisualId);
    const bool bDynamicallyVisible = DynamicVisibility == nullptr || *DynamicVisibility;
    // A camera may disappear while a light descriptor is retained. Do not
    // leave its old light shining from a stale, unregistered camera actor.
    const bool bHasMount = Definition->ParentId.IsEmpty() || ResolveVisualParent(Definition->ParentId) != nullptr;
    Actor->SetActorHiddenInGame(!bDynamicallyVisible || !bHasMount || !IsVisualKindVisible(Definition->Kind));
}

void ABskSceneController::AttachManifestChildren()
{
    for (const TPair<FString, FBskObjectDefinition>& Pair : ManifestObjects)
    {
        AActor* Actor = BoundActors.FindRef(Pair.Key);
        AActor* Parent = Pair.Value.ParentId.IsEmpty() ? nullptr : BoundActors.FindRef(Pair.Value.ParentId);
        if (Actor && Parent && Actor->GetAttachParentActor() != Parent) Actor->AttachToActor(Parent, FAttachmentTransformRules::KeepWorldTransform);
    }
    for (const TPair<FString, FBskCameraDefinition>& Pair : ManifestCameras)
    {
        AActor* Actor = CameraActors.FindRef(Pair.Key);
        AActor* Parent = Pair.Value.ParentId.IsEmpty() ? nullptr : BoundActors.FindRef(Pair.Value.ParentId);
        if (Actor && Parent) Actor->AttachToActor(Parent, FAttachmentTransformRules::KeepRelativeTransform);
        else if (Actor && Actor->GetAttachParentActor()) Actor->DetachFromActor(FDetachmentTransformRules::KeepRelativeTransform);
    }
    for (const TPair<FString, FBskVisualDefinition>& Pair : ManifestVisuals)
    {
        AActor* Actor = VisualActors.FindRef(Pair.Key);
        AActor* Parent = ResolveVisualParent(Pair.Value.ParentId);
        if (Actor && Parent) Actor->AttachToActor(Parent, FAttachmentTransformRules::KeepRelativeTransform);
        else if (Actor && Actor->GetAttachParentActor()) Actor->DetachFromActor(FDetachmentTransformRules::KeepRelativeTransform);
        RefreshVisualVisibility(Pair.Key);
    }
}

AActor* ABskSceneController::ResolveVisualParent(const FString& ParentId) const
{
    if (AActor* Body = BoundActors.FindRef(ParentId)) return Body;
    // CameraActors is a reusable cache; only active-manifest cameras are mounts.
    return ManifestCameras.Contains(ParentId) ? CameraActors.FindRef(ParentId) : nullptr;
}

void ABskSceneController::UpdateVisualStates(const FBskRenderFrame& Frame)
{
    for (const FBskVisualState& State : Frame.VisualStates)
    {
        if (AActor* Actor = VisualActors.FindRef(State.VisualId))
        {
            const FBskVisualDefinition* Definition = ManifestVisuals.Find(State.VisualId);
            if (!Definition) continue;
            auto NumberChannel = [&State](const TCHAR* Name, double Fallback)
            {
                if (const FBskVisualState::FChannelValue* Channel = State.Channels.Find(Name))
                {
                    if (Channel->Type == FBskVisualState::FChannelValue::EType::Number) return Channel->Number;
                    if (Channel->Type == FBskVisualState::FChannelValue::EType::Boolean) return Channel->Boolean ? 1.0 : 0.0;
                }
                return Fallback;
            };
            auto BoolChannel = [&State](const TCHAR* Name, bool Fallback)
            {
                if (const FBskVisualState::FChannelValue* Channel = State.Channels.Find(Name))
                {
                    if (Channel->Type == FBskVisualState::FChannelValue::EType::Boolean) return Channel->Boolean;
                    if (Channel->Type == FBskVisualState::FChannelValue::EType::Number) return !FMath::IsNearlyZero(Channel->Number);
                }
                return Fallback;
            };

            bool bVisible = State.bVisible;
            if (Definition->Kind == TEXT("reaction_wheel"))
            {
                const double Angle = NumberChannel(TEXT("angle_rad"), 0.0);
                const FQuat Base = VisualBaseRotations.FindRef(State.VisualId);
                Actor->SetActorRelativeRotation(Base * FQuat(FVector::UpVector, Angle));
                bVisible = BoolChannel(TEXT("enabled"), bVisible);
            }
            else if (Definition->Kind == TEXT("thruster"))
            {
                const double Throttle = FMath::Clamp(NumberChannel(TEXT("throttle"), State.Value), 0.0, 1.0);
                const FVector BaseScale = VisualBaseScales.FindRef(State.VisualId);
                Actor->SetActorRelativeScale3D(FVector(BaseScale.X, BaseScale.Y, BaseScale.Z * FMath::Max(Throttle, 0.02)));
                bVisible = BoolChannel(TEXT("enabled"), bVisible) && Throttle > 1.0e-6;
            }
            else if (Definition->Kind == TEXT("css"))
            {
                const double Signal = FMath::Clamp(NumberChannel(TEXT("normalized_signal"), State.Value), 0.0, 1.0);
                const FVector BaseScale = VisualBaseScales.FindRef(State.VisualId);
                Actor->SetActorRelativeScale3D(BaseScale * (0.85 + 0.15 * Signal));
                bVisible = BoolChannel(TEXT("enabled"), bVisible);
            }
            else if (Definition->Kind == TEXT("generic_sensor") || Definition->Kind == TEXT("light"))
            {
                bVisible = BoolChannel(TEXT("enabled"), bVisible);
            }
            VisualDynamicVisibility.Add(State.VisualId, bVisible);
            RefreshVisualVisibility(State.VisualId);

            const double Intensity = Definition->Kind == TEXT("thruster")
                ? FMath::Clamp(NumberChannel(TEXT("throttle"), State.Value), 0.0, 1.0)
                : 1.0;
            TArray<UStaticMeshComponent*> Components;
            Actor->GetComponents<UStaticMeshComponent>(Components);
            for (UStaticMeshComponent* Component : Components)
            {
                if (UMaterialInstanceDynamic* Material = Cast<UMaterialInstanceDynamic>(Component->GetMaterial(0)))
                {
                    Material->SetVectorParameterValue(TEXT("Color"), Definition->Color * static_cast<float>(0.35 + 0.65 * Intensity));
                }
            }
        }
    }
}
