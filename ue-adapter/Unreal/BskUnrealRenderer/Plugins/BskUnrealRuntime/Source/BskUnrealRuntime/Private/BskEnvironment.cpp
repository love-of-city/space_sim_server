#include "BskEnvironment.h"

#include "BskEnvironmentBody.h"
#include "Camera/CameraActor.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/LightComponent.h"
#include "Components/MeshComponent.h"
#include "Components/SceneComponent.h"
#include "Components/SkyAtmosphereComponent.h"
#include "Components/SpotLightComponent.h"
#include "Engine/DirectionalLight.h"
#include "Engine/PostProcessVolume.h"
#include "Engine/SpotLight.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"

ABskEnvironment::ABskEnvironment()
{
    PrimaryActorTick.bCanEverTick = false;
    USceneComponent* Root = CreateDefaultSubobject<USceneComponent>(TEXT("EnvironmentRoot"));
    SetRootComponent(Root);
}

void ABskEnvironment::BeginPlay()
{
    Super::BeginPlay();
    CapturePreviewState();
}

void ABskEnvironment::CapturePreviewState()
{
    if (bHasPreviewState || !GetWorld() || !GetWorld()->IsGameWorld()) return;
    bHasPreviewState = true;
    const TArray<AActor*> Actors{
        Earth, StarSphere, SunVisual, SunLight, CelestialSunLight, FillLight,
        Headlight, ExposureVolume, EarthAtmosphere, PreviewCamera};
    TSet<AActor*> Captured;
    for (AActor* Actor : Actors)
    {
        if (!IsValid(Actor) || Captured.Contains(Actor)) continue;
        Captured.Add(Actor);
        USceneComponent* Root = Actor->GetRootComponent();
        ActorPreviewStates.Add({Actor, Actor->GetActorTransform(), Actor->IsHidden(),
            Root ? Root->GetAttachParent() : nullptr, Root ? Root->GetAttachSocketName() : NAME_None});
        TArray<USceneComponent*> Components;
        Actor->GetComponents<USceneComponent>(Components);
        for (USceneComponent* Component : Components)
        {
            ComponentPreviewStates.Add({Component, Component->GetRelativeTransform(),
                Component->IsVisible(), Component->bHiddenInGame != 0});
            if (ULightComponent* Light = Cast<ULightComponent>(Component))
            {
                const UDirectionalLightComponent* Directional = Cast<UDirectionalLightComponent>(Light);
                const USpotLightComponent* Spot = Cast<USpotLightComponent>(Light);
                LightPreviewStates.Add({Light, Light->Intensity, Light->GetLightColor(),
                    Directional ? Directional->LightSourceAngle : 0.0f,
                    Light->SpecularScale, Light->CastShadows != 0,
                    Spot ? Spot->AttenuationRadius : 0.0f,
                    Spot ? Spot->InnerConeAngle : 0.0f,
                    Spot ? Spot->OuterConeAngle : 0.0f});
            }
            if (UMeshComponent* Mesh = Cast<UMeshComponent>(Component))
            {
                for (int32 Index = 0; Index < Mesh->GetNumMaterials(); ++Index)
                {
                    const int32 SavedIndex = PreviewMaterials.Add(Mesh->GetMaterial(Index));
                    MaterialPreviewStates.Add({Mesh, Index, SavedIndex});
                }
            }
        }
    }
    if (EarthAtmosphere)
    {
        PreviewAtmosphereRadius = EarthAtmosphere->GetComponent()->BottomRadius;
        PreviewAtmosphereHeight = EarthAtmosphere->GetComponent()->AtmosphereHeight;
        PreviewAtmosphereAerialPerspectiveStart = EarthAtmosphere->GetComponent()->AerialPerspectiveStartDepth;
    }
}

void ABskEnvironment::RestorePreviewState()
{
    if (!bHasPreviewState) return;
    // A backend headlight may have been attached to the running camera. Restore
    // the saved parent/socket before restoring world and local transforms.
    for (const FActorPreviewState& State : ActorPreviewStates)
    {
        if (AActor* Actor = State.Actor.Get())
        {
            Actor->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
            if (USceneComponent* Parent = State.AttachParent.Get())
                Actor->AttachToComponent(Parent, FAttachmentTransformRules::KeepWorldTransform, State.AttachSocket);
        }
    }
    for (const FActorPreviewState& State : ActorPreviewStates)
    {
        if (AActor* Actor = State.Actor.Get())
        {
            Actor->SetActorTransform(State.Transform, false, nullptr, ETeleportType::TeleportPhysics);
            Actor->SetActorHiddenInGame(State.bHidden);
        }
    }
    for (const FComponentPreviewState& State : ComponentPreviewStates)
    {
        if (USceneComponent* Component = State.Component.Get())
        {
            Component->SetRelativeTransform(State.RelativeTransform, false, nullptr, ETeleportType::TeleportPhysics);
            Component->SetVisibility(State.bVisible);
            Component->SetHiddenInGame(State.bHiddenInGame);
        }
    }
    for (const FLightPreviewState& State : LightPreviewStates)
    {
        if (ULightComponent* Light = State.Component.Get())
        {
            Light->SetIntensity(State.Intensity);
            Light->SetLightColor(State.Color);
            Light->SetSpecularScale(State.SpecularScale);
            Light->SetCastShadows(State.bCastShadows);
            if (UDirectionalLightComponent* Directional = Cast<UDirectionalLightComponent>(Light))
                Directional->SetLightSourceAngle(State.SourceAngle);
            if (USpotLightComponent* Spot = Cast<USpotLightComponent>(Light))
            {
                Spot->SetAttenuationRadius(State.AttenuationRadius);
                Spot->SetInnerConeAngle(State.InnerConeAngle);
                Spot->SetOuterConeAngle(State.OuterConeAngle);
            }
        }
    }
    for (const FMaterialPreviewState& State : MaterialPreviewStates)
    {
        if (UMeshComponent* Mesh = State.Component.Get())
            Mesh->SetMaterial(State.MaterialIndex, PreviewMaterials[State.SavedMaterialIndex]);
    }
    if (EarthAtmosphere)
    {
        EarthAtmosphere->GetComponent()->SetBottomRadius(PreviewAtmosphereRadius);
        EarthAtmosphere->GetComponent()->SetAtmosphereHeight(PreviewAtmosphereHeight);
        EarthAtmosphere->GetComponent()->SetAerialPerspectiveStartDepth(PreviewAtmosphereAerialPerspectiveStart);
    }
    if (Earth) Earth->RefreshVisualScale();
    IsBackendControlled = false;
}

void ABskEnvironment::SetBackendControlled(bool bControlled)
{
    if (bControlled && !bAllowBackendControl) return;
    if (bControlled)
    {
        CapturePreviewState();
        IsBackendControlled = true;
    }
    else RestorePreviewState();
}

FTransform ABskEnvironment::GetPreviewTransform(const AActor* Actor) const
{
    for (const FActorPreviewState& State : ActorPreviewStates)
        if (State.Actor.Get() == Actor) return State.Transform;
    return Actor ? Actor->GetActorTransform() : FTransform::Identity;
}

void ABskEnvironment::StoreEditorPreviewView()
{
#if WITH_EDITOR
    UWorld* World = GetWorld();
    if (!World || World->IsGameWorld() || !PreviewCamera) return;
    World->Modify();
    // ELevelViewportType::LVT_Perspective is 3 in UE 5.6. Avoid an UnrealEd
    // module dependency in this runtime actor; EditorViews is editor-only data.
    if (World->EditorViews.Num() < 8) World->EditorViews.SetNum(8);
    FLevelViewportInfo& View = World->EditorViews[3];
    View.CamPosition = PreviewCamera->GetActorLocation();
    View.CamRotation = PreviewCamera->GetActorRotation();
    View.CamUpdated = true;
#endif
}
