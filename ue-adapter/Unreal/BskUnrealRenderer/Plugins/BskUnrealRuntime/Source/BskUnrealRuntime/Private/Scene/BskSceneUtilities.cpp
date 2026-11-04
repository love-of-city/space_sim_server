#include "Scene/BskSceneUtilities.h"
#include "BskUnrealRuntime.h"
#include "BskProtocolTypes.h"
#include "Engine/StaticMesh.h"
#include "Engine/Texture.h"
#include "Engine/World.h"
#include "Components/StaticMeshComponent.h"
#include "AssetCompilingManager.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "HAL/FileManager.h"
#include "Misc/ConfigCacheIni.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"

namespace BskSceneUtilities
{


bool JsonVector3(const TSharedPtr<FJsonObject>& Object, const TCHAR* Name, FVector3d& Out)
{
    const TArray<TSharedPtr<FJsonValue>>* Values = nullptr;
    if (!Object.IsValid() || !Object->TryGetArrayField(Name, Values) || Values == nullptr || Values->Num() != 3)
    {
        return false;
    }
    Out = FVector3d((*Values)[0]->AsNumber(), (*Values)[1]->AsNumber(), (*Values)[2]->AsNumber());
    return true;
}

bool JsonColor(const TSharedPtr<FJsonObject>& Object, const TCHAR* Name, FLinearColor& Out)
{
    const TArray<TSharedPtr<FJsonValue>>* Values = nullptr;
    if (!Object.IsValid() || !Object->TryGetArrayField(Name, Values) || Values == nullptr || Values->Num() < 3)
    {
        return false;
    }
    Out = FLinearColor(
        static_cast<float>((*Values)[0]->AsNumber()),
        static_cast<float>((*Values)[1]->AsNumber()),
        static_cast<float>((*Values)[2]->AsNumber()),
        Values->Num() >= 4 ? static_cast<float>((*Values)[3]->AsNumber()) : 1.0f);
    return true;
}

FString BindingTag(const FString& Name)
{
    return FString::Printf(TEXT("BSK.%s"), *Name);
}

bool SameObjectRenderDefinition(const FBskObjectDefinition& A, const FBskObjectDefinition& B)
{
    if (A.DisplayName != B.DisplayName || A.ParentId != B.ParentId || A.TransformSpace != B.TransformSpace ||
        A.AssetPath != B.AssetPath || A.SemanticLabel != B.SemanticLabel || A.Geometries.Num() != B.Geometries.Num()) return false;
    for (int32 Index = 0; Index < A.Geometries.Num(); ++Index)
    {
        const FBskGeometryDefinition& X = A.Geometries[Index];
        const FBskGeometryDefinition& Y = B.Geometries[Index];
        if (X.GeometryId != Y.GeometryId || X.Shape != Y.Shape || X.DimensionsMeters != Y.DimensionsMeters ||
            X.PositionBodyMeters != Y.PositionBodyMeters || X.OrientationBodyFromGeometryWxyz != Y.OrientationBodyFromGeometryWxyz ||
            X.Color != Y.Color || X.AssetType != Y.AssetType || X.AssetPath != Y.AssetPath || X.Scale != Y.Scale ||
            X.RenderRole != Y.RenderRole || X.AssetKey != Y.AssetKey || X.MaterialName != Y.MaterialName ||
            X.MaterialSpecular != Y.MaterialSpecular || X.MaterialShininess != Y.MaterialShininess ||
            X.MaterialReflectance != Y.MaterialReflectance || X.MaterialEmission != Y.MaterialEmission ||
            X.MaterialTexture != Y.MaterialTexture || X.MaterialTextureAssetPath != Y.MaterialTextureAssetPath ||
            X.MaterialTextureRepeat != Y.MaterialTextureRepeat || X.bUseAssetMaterials != Y.bUseAssetMaterials) return false;
    }
    return true;
}

bool SameVisualActorDefinition(const FBskVisualDefinition& A, const FBskVisualDefinition& B)
{
    // Channel schemas are applied to frame values, not to the spawned geometry.
    return A.Kind == B.Kind && A.ParentId == B.ParentId && A.PositionBodyMeters == B.PositionBodyMeters &&
        A.OrientationBodyFromVisualWxyz == B.OrientationBodyFromVisualWxyz && A.NormalBody == B.NormalBody &&
        A.FieldOfViewRadians == B.FieldOfViewRadians && A.SizeMeters == B.SizeMeters && A.RangeMeters == B.RangeMeters &&
        A.Color == B.Color && A.Label == B.Label && A.LightType == B.LightType && A.LightTargetId == B.LightTargetId &&
        A.LightDiffuseRgb == B.LightDiffuseRgb && A.LightSpecularRgb == B.LightSpecularRgb &&
        A.LightIntensity == B.LightIntensity && A.LightCutoffDegrees == B.LightCutoffDegrees &&
        A.bLightCastShadows == B.bLightCastShadows;
}

FName SafeActorName(const FString& Value)
{
    FString Result = Value;
    Result.ReplaceInline(TEXT("/"), TEXT("_"));
    Result.ReplaceInline(TEXT("\\"), TEXT("_"));
    Result.ReplaceInline(TEXT("."), TEXT("_"));
    return FName(*FString::Printf(TEXT("%s_H%08X"), *Result, GetTypeHash(Value)));
}

const TCHAR* PrimitiveMeshPath(const FString& Shape)
{
    if (Shape.Equals(TEXT("plane"), ESearchCase::IgnoreCase))
    {
        return TEXT("/Engine/BasicShapes/Plane.Plane");
    }
    if (Shape.Equals(TEXT("sphere"), ESearchCase::IgnoreCase) || Shape.Equals(TEXT("ellipsoid"), ESearchCase::IgnoreCase))
    {
        return TEXT("/Engine/BasicShapes/Sphere.Sphere");
    }
    if (Shape.Equals(TEXT("cylinder"), ESearchCase::IgnoreCase) || Shape.Equals(TEXT("capsule"), ESearchCase::IgnoreCase))
    {
        return TEXT("/Engine/BasicShapes/Cylinder.Cylinder");
    }
    if (Shape.Equals(TEXT("cone"), ESearchCase::IgnoreCase))
    {
        return TEXT("/Engine/BasicShapes/Cone.Cone");
    }
    return TEXT("/Engine/BasicShapes/Cube.Cube");
}

bool PrepareStaticMeshForRuntime(UStaticMesh* Mesh, const FString& AssetPath)
{
    if (!Mesh) return false;
    if (Mesh->IsCompiling())
    {
        TArray<UObject*> Assets;
        Assets.Add(Mesh);
        UE_LOG(LogBskUnreal, Display,
            TEXT("Waiting for static mesh compilation before starting frame intake: %s"), *AssetPath);
        FAssetCompilingManager::Get().FinishCompilationForObjects(Assets);
    }
    if (!Mesh->HasValidRenderData())
    {
        UE_LOG(LogBskUnreal, Warning,
            TEXT("Static mesh has no valid render data after preparation: %s"), *AssetPath);
        return false;
    }
    return true;
}

void PreloadConfiguredVisualOverlays()
{
    if (!GConfig) return;
    bool bEnabled = false;
    GConfig->GetBool(TEXT("Bsk.VisualOverlays"), TEXT("Enabled"), bEnabled, GGameIni);
    if (!bEnabled) return;
    const FConfigSection* Section = GConfig->GetSection(TEXT("Bsk.VisualOverlays"), false, GGameIni);
    if (!Section) return;
    for (const TPair<FName, FConfigValue>& Entry : *Section)
    {
        if (Entry.Key == FName(TEXT("Enabled"))) continue;
        const FString AssetPath = Entry.Value.GetValue();
        if (AssetPath.IsEmpty()) continue;
        UStaticMesh* Mesh = LoadObject<UStaticMesh>(nullptr, *AssetPath);
        if (!Mesh)
        {
            UE_LOG(LogBskUnreal, Warning, TEXT("Configured visual overlay is unavailable during preload: %s"), *AssetPath);
            continue;
        }
        if (PrepareStaticMeshForRuntime(Mesh, AssetPath))
        {
            UE_LOG(LogBskUnreal, Display, TEXT("Preloaded visual overlay before frame intake: %s"), *AssetPath);
        }
    }
}

void PreloadRuntimeStaticMeshes()
{
    // Manifest objects are created on the game thread when the first scene
    // manifest arrives. Loading 100+ generated SARM meshes at that point can
    // block the game thread long enough for the reliable render FIFO to fill,
    // which makes the authoritative simulator abort before any RGB frame is
    // captured. Resolve the packaged .uasset files before starting frame
    // intake instead of relying only on the editor asset-registry cache.
    const FString ContentRoot = FPaths::ConvertRelativePathToFull(FPaths::ProjectContentDir());
    const TArray<FString> Roots = {
        TEXT("BSK/Generated/SARM"),
        TEXT("BSK/VisualOverlays"),
        TEXT("_GENERATED/Hyperlovimia")};

    TSet<FString> SeenPaths;
    TArray<UStaticMesh*> Meshes;
    int32 CandidateCount = 0;
    for (const FString& Root : Roots)
    {
        TArray<FString> Files;
        const FString RootPath = FPaths::Combine(ContentRoot, Root);
        IFileManager::Get().FindFilesRecursive(Files, *RootPath, TEXT("*.uasset"), true, false);
        CandidateCount += Files.Num();
        for (const FString& FilePath : Files)
        {
            FString RelativePath = FilePath;
            if (!FPaths::MakePathRelativeTo(RelativePath, *ContentRoot)) continue;
            RelativePath.ReplaceInline(TEXT("\\"), TEXT("/"));
            if (!RelativePath.EndsWith(TEXT(".uasset"), ESearchCase::IgnoreCase)) continue;
            const FString BaseName = FPaths::GetBaseFilename(RelativePath);
            if (BaseName.StartsWith(TEXT("M_")) || BaseName.StartsWith(TEXT("MI_"))) continue;
            RelativePath.LeftChopInline(7, EAllowShrinking::No);
            const FString ObjectPath = FString::Printf(
                TEXT("/Game/%s.%s"), *RelativePath, *FPaths::GetBaseFilename(RelativePath));
            UStaticMesh* Mesh = LoadObject<UStaticMesh>(nullptr, *ObjectPath);
            if (!Mesh) continue;
            const FString AssetPath = Mesh->GetPathName();
            if (SeenPaths.Contains(AssetPath)) continue;
            SeenPaths.Add(AssetPath);
            Meshes.Add(Mesh);
        }
    }

    if (Meshes.IsEmpty())
    {
        UE_LOG(LogBskUnreal, Warning,
            TEXT("No runtime static meshes were found during preloading (asset candidates=%d)"),
            CandidateCount);
        return;
    }

    TArray<UObject*> CompilingMeshes;
    CompilingMeshes.Reserve(Meshes.Num());
    for (UStaticMesh* Mesh : Meshes)
    {
        if (Mesh && Mesh->IsCompiling()) CompilingMeshes.Add(Mesh);
    }
    if (!CompilingMeshes.IsEmpty())
    {
        UE_LOG(LogBskUnreal, Display,
            TEXT("Waiting for %d runtime static meshes before starting frame intake (candidates=%d)"),
            CompilingMeshes.Num(), CandidateCount);
        FAssetCompilingManager::Get().FinishCompilationForObjects(CompilingMeshes);
    }

    int32 ReadyCount = 0;
    for (UStaticMesh* Mesh : Meshes)
    {
        if (Mesh && Mesh->HasValidRenderData()) ++ReadyCount;
    }
    UE_LOG(LogBskUnreal, Display,
        TEXT("Preloaded runtime static meshes before frame intake: ready=%d total=%d candidates=%d"),
        ReadyCount, Meshes.Num(), CandidateCount);
}

void ApplyUnlitColor(UStaticMeshComponent* Component, UObject* Owner, const FLinearColor& Color)
{
    if (!Component) return;
    if (UMaterialInterface* Base = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/BSK/M_BskUnlitColor.M_BskUnlitColor")))
    {
        UMaterialInstanceDynamic* Material = UMaterialInstanceDynamic::Create(Base, Owner);
        Material->SetVectorParameterValue(TEXT("Color"), Color);
        Component->SetMaterial(0, Material);
    }
}

// Exact mesh-object-path overrides are visual only. They deliberately do not use
// numeric geom indices (which change between MJCF variants) or broad name matches.
bool ApplyConfiguredMaterialOverride(UStaticMeshComponent* Component, const FBskGeometryDefinition& Geometry)
{
    if (!GConfig || Geometry.AssetPath.IsEmpty() || Geometry.RenderRole != TEXT("visual") ||
        FParse::Param(FCommandLine::Get(), TEXT("BskDisableMaterialOverrides"))) return false;
    bool bEnabled = false;
    GConfig->GetBool(TEXT("Bsk.MaterialOverrides"), TEXT("Enabled"), bEnabled, GGameIni);
    if (!bEnabled) return false;
    FString MaterialPath;
    if (!GConfig->GetString(TEXT("Bsk.MaterialOverrides"), *Geometry.AssetPath, MaterialPath, GGameIni) ||
        MaterialPath.IsEmpty()) return false;
    UMaterialInterface* Material = LoadObject<UMaterialInterface>(nullptr, *MaterialPath);
    if (!Material)
    {
        UE_LOG(LogBskUnreal, Warning, TEXT("Material override missing: %s for %s; using original material"),
            *MaterialPath, *Geometry.AssetPath);
        return false;
    }
    const int32 SlotCount = FMath::Max(1, Component->GetNumMaterials());
    for (int32 Slot = 0; Slot < SlotCount; ++Slot) Component->SetMaterial(Slot, Material);
    UE_LOG(LogBskUnreal, Log, TEXT("Applied visual material override %s to %s (%s)"),
        *MaterialPath, *Geometry.AssetPath, *Geometry.GeometryId);
    return true;
}

// Render-only skins are children of the original mesh component. This preserves
// body position/rotation without inheriting source-mesh unit conversion or changing physics.
void AttachConfiguredVisualOverlay(UStaticMeshComponent* Parent, const FBskGeometryDefinition& Geometry)
{
    if (!Parent || !GConfig || Geometry.AssetPath.IsEmpty() || Geometry.RenderRole == TEXT("collision") ||
        FParse::Param(FCommandLine::Get(), TEXT("BskDisableVisualOverlays"))) return;
    bool bEnabled = false;
    GConfig->GetBool(TEXT("Bsk.VisualOverlays"), TEXT("Enabled"), bEnabled, GGameIni);
    if (!bEnabled) return;
    FString MeshPath;
    if (!GConfig->GetString(TEXT("Bsk.VisualOverlays"), *Geometry.AssetPath, MeshPath, GGameIni) || MeshPath.IsEmpty()) return;
    UStaticMesh* Mesh = LoadObject<UStaticMesh>(nullptr, *MeshPath);
    if (!Mesh)
    {
        UE_LOG(LogBskUnreal, Warning, TEXT("Visual overlay missing: %s for %s; original mesh retained"), *MeshPath, *Geometry.AssetPath);
        return;
    }
    if (!PrepareStaticMeshForRuntime(Mesh, MeshPath))
    {
        UE_LOG(LogBskUnreal, Warning, TEXT("Visual overlay is not render-ready: %s; original mesh retained"), *MeshPath);
        return;
    }
    AActor* Owner = Parent->GetOwner();
    UStaticMeshComponent* Overlay = NewObject<UStaticMeshComponent>(Owner,
        MakeUniqueObjectName(Owner, UStaticMeshComponent::StaticClass(), TEXT("VisualOverlay")));
    Overlay->SetMobility(EComponentMobility::Movable);
    Overlay->SetStaticMesh(Mesh); // Keep the overlay's own UV material slots.
    Overlay->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Overlay->SetGenerateOverlapEvents(false);
    Overlay->SetCanEverAffectNavigation(false);
    Overlay->SetCastShadow(true);
    Overlay->SetupAttachment(Parent);
    // Overlay assets are baked in UE centimetres. Geometry scale converts the
    // source mesh units (e.g. millimetres); applying it again shrinks the wrap.
    Overlay->SetAbsolute(false, false, true);
    Overlay->SetRelativeTransform(FTransform::Identity);
    Owner->AddInstanceComponent(Overlay);
    Overlay->RegisterComponent();
    UE_LOG(LogBskUnreal, Log, TEXT("Attached visual-only overlay %s to %s (%s), source scale=%s, physical overlay scale=%s"), *MeshPath, *Geometry.AssetPath, *Geometry.GeometryId, *Parent->GetComponentScale().ToString(), *Overlay->GetComponentScale().ToString());
}



void ApplyGeometryMaterial(UStaticMeshComponent* Component, UObject* Owner, const FBskGeometryDefinition& Geometry, double Ambient)
{
    if (!Component) return;
    if (ApplyConfiguredMaterialOverride(Component, Geometry)) return;
    if (Geometry.bUseAssetMaterials && Geometry.MaterialName.IsEmpty()) return;
    const bool bTranslucent = Geometry.Color.A < 0.999f;
    const TCHAR* MaterialPath = bTranslucent
        ? TEXT("/Game/BSK/M_BskPbrTranslucent.M_BskPbrTranslucent")
        : TEXT("/Game/BSK/M_BskPbrOpaque.M_BskPbrOpaque");
    UMaterialInterface* Base = LoadObject<UMaterialInterface>(nullptr, MaterialPath);
    if (!Base)
    {
        ApplyUnlitColor(Component, Owner, Geometry.Color);
        return;
    }

    UMaterialInstanceDynamic* Material = UMaterialInstanceDynamic::Create(Base, Owner);
    const double BlinnExponent = FMath::Clamp(Geometry.MaterialShininess, 0.0, 1.0) * 128.0;
    const float Roughness = static_cast<float>(FMath::Clamp(FMath::Sqrt(2.0 / (BlinnExponent + 2.0)), 0.08, 1.0));
    const float Specular = static_cast<float>(FMath::Clamp(
        Geometry.MaterialSpecular + Geometry.MaterialReflectance * (1.0 - Geometry.MaterialSpecular), 0.0, 1.0));
    Material->SetVectorParameterValue(TEXT("BaseColor"), Geometry.Color);
    Material->SetScalarParameterValue(TEXT("Roughness"), Roughness);
    Material->SetScalarParameterValue(TEXT("Specular"), Specular);
    Material->SetScalarParameterValue(TEXT("Metallic"), 0.0f);
    Material->SetScalarParameterValue(TEXT("Emission"), static_cast<float>(FMath::Max(0.0, Geometry.MaterialEmission)));
    // Legacy PBR assets add Ambient to Emissive. Scene/headlight ambient is
    // illumination, not material emission: let actual lights illuminate geometry.
    // Keep explicit MJCF emission above for intentionally luminous objects.
    Material->SetScalarParameterValue(TEXT("Ambient"), 0.0f);
    Material->SetScalarParameterValue(TEXT("Opacity"), Geometry.Color.A);
    Material->SetScalarParameterValue(TEXT("UseTexture"), 0.0f);
    Material->SetVectorParameterValue(TEXT("TextureRepeat"), FLinearColor(
        static_cast<float>(Geometry.MaterialTextureRepeat.X),
        static_cast<float>(Geometry.MaterialTextureRepeat.Y), 0.0f, 0.0f));
    if (!Geometry.MaterialTextureAssetPath.IsEmpty())
    {
        if (UTexture* Texture = LoadObject<UTexture>(nullptr, *Geometry.MaterialTextureAssetPath))
        {
            Material->SetTextureParameterValue(TEXT("BaseColorTexture"), Texture);
            Material->SetScalarParameterValue(TEXT("UseTexture"), 1.0f);
        }
        else
        {
            UE_LOG(LogBskUnreal, Warning, TEXT("Could not load packaged MJCF texture %s for %s"),
                *Geometry.MaterialTextureAssetPath, *Geometry.GeometryId);
        }
    }
    const int32 SlotCount = FMath::Max(1, Component->GetNumMaterials());
    for (int32 Slot = 0; Slot < SlotCount; ++Slot) Component->SetMaterial(Slot, Material);
}

bool IsSensorFrustumKind(const FString& Kind)
{
    return Kind.Equals(TEXT("css"), ESearchCase::IgnoreCase) ||
        Kind.Equals(TEXT("generic_sensor"), ESearchCase::IgnoreCase) ||
        Kind.Equals(TEXT("transceiver"), ESearchCase::IgnoreCase);
}

}
