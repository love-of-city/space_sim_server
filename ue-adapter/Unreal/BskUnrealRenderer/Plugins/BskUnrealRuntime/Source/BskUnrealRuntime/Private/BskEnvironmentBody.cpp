#include "BskEnvironmentBody.h"

#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Particles/ParticleSystemComponent.h"
#include "Engine/StaticMesh.h"
#include "NiagaraComponent.h"
#if WITH_EDITOR
#include "NiagaraEmitter.h"
#include "NiagaraMeshRendererProperties.h"
#include "NiagaraScriptSourceBase.h"
#include "NiagaraSystem.h"
#endif

ABskEnvironmentBody::ABskEnvironmentBody()
{
    PrimaryActorTick.bCanEverTick = false;
    BodyRoot = CreateDefaultSubobject<USceneComponent>(TEXT("BodyRoot"));
    BodyRoot->SetMobility(EComponentMobility::Movable);
    SetRootComponent(BodyRoot);

    auto AddLayer = [this](const TCHAR* Name)
    {
        UStaticMeshComponent* Component = CreateDefaultSubobject<UStaticMeshComponent>(Name);
        Component->SetupAttachment(BodyRoot);
        Component->SetMobility(EComponentMobility::Movable);
        Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        Component->SetCanEverAffectNavigation(false);
        return Component;
    };
    Surface = AddLayer(TEXT("Surface"));
    Clouds = AddLayer(TEXT("Clouds"));
    Atmosphere = AddLayer(TEXT("Atmosphere"));

    Aurora = CreateDefaultSubobject<UNiagaraComponent>(TEXT("Aurora"));
    Aurora->SetupAttachment(BodyRoot);
    Aurora->SetMobility(EComponentMobility::Movable);
    // Niagara scales its shell through User.SystemScale. Apply only the body's
    // translation/rotation here, otherwise backend radius changes scale twice.
    Aurora->SetAbsolute(false, false, true);
    Aurora->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Aurora->SetCastShadow(false);
    Aurora->SetCanEverAffectNavigation(false);
    Aurora->bAutoActivate = false;

    auto AddEffect = [this](const TCHAR* Name)
    {
        UParticleSystemComponent* Component = CreateDefaultSubobject<UParticleSystemComponent>(Name);
        Component->SetupAttachment(BodyRoot);
        Component->SetMobility(EComponentMobility::Movable);
        Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        Component->SetCastShadow(false);
        Component->SetCanEverAffectNavigation(false);
        Component->bAutoActivate = false;
        return Component;
    };
    SunBursts = AddEffect(TEXT("SunBursts"));
    SunHalo = AddEffect(TEXT("SunHalo"));
    SunLines = AddEffect(TEXT("SunLines"));
    // Saved meshes, materials and layer transforms remain owned by the level.
}

void ABskEnvironmentBody::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);
    RefreshVisualScale();
}

void ABskEnvironmentBody::BeginPlay()
{
    Super::BeginPlay();
    RefreshVisualScale();
}

void ABskEnvironmentBody::RefreshVisualScale()
{
    if (!Aurora || !Surface || !Surface->GetStaticMesh()) return;
    const double MeshScale = Surface->GetComponentScale().GetAbsMax();
    Aurora->SetVariableFloat(TEXT("User.SystemScale"), static_cast<float>(MeshScale));
    // The source material used the Niagara component's inverse scale to form
    // its polar coordinates. Geometry now scales inside Niagara, so supply
    // that same divisor explicitly without applying a second geometry scale.
    Aurora->SetCustomPrimitiveDataFloat(0, static_cast<float>(MeshScale));
    const double Radius = Surface->GetStaticMesh()->GetBounds().SphereRadius * MeshScale;
    const FBox Bounds(FVector(-Radius * 1.5), FVector(Radius * 1.5));
    Aurora->SetSystemFixedBounds(Bounds);
    Aurora->SetEmitterFixedBounds(TEXT("AuroraSphere"), Bounds);
}

bool ABskEnvironmentBody::PrepareAttachedAuroraAsset(UNiagaraSystem* System, UMaterialInterface* Material)
{
#if WITH_EDITOR
    // Restrict this authoring helper to the migrated copy, never the source asset.
    if (!System || !Material || !System->GetPathName().StartsWith(TEXT("/Game/BSK/Environment/IncredibleEarth/"))) return false;
    System->Modify();
    for (FNiagaraEmitterHandle& Handle : System->GetEmitterHandles())
    {
        FVersionedNiagaraEmitter Instance = Handle.GetInstance();
        FVersionedNiagaraEmitterData* Data = Instance.GetEmitterData();
        if (!Data) return false;
        Instance.Emitter->Modify();
        // The original infinitely lived world-space particles stay behind when
        // a backend frame moves Earth. Local simulation follows its full pose.
        Data->bLocalSpace = true;
        // LocalSpace is a compile-time switch in Initialize Particle. Updating
        // the emitter flag alone leaves cached world-space spawn bytecode.
        if (Data->GraphSource)
        {
            Data->GraphSource->MarkNotSynchronized(TEXT("BSK attached aurora local simulation"));
        }
        Data->FixedBounds = FBox(FVector(-1.0e9), FVector(1.0e9));
        for (UNiagaraRendererProperties* Renderer : Data->GetRenderers())
        {
            if (UNiagaraMeshRendererProperties* Mesh = Cast<UNiagaraMeshRendererProperties>(Renderer))
            {
                Mesh->Modify();
                Mesh->bOverrideMaterials = true;
                Mesh->OverrideMaterials.SetNum(1);
                Mesh->OverrideMaterials[0].ExplicitMat = Material;
            }
        }
        Instance.Emitter->PostEditChange();
    }
    System->PostEditChange();
    System->RequestCompile(true);
    System->WaitForCompilationComplete();
    System->MarkPackageDirty();
    return System->GetEmitterHandles().Num() > 0;
#else
    return false;
#endif
}
