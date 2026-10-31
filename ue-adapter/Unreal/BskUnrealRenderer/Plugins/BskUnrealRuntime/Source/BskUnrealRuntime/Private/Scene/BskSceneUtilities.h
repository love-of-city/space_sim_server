#pragma once
#include "CoreMinimal.h"
#include "BskProtocolTypes.h"
#include "Dom/JsonObject.h"

class UStaticMesh;
class UStaticMeshComponent;

namespace BskSceneUtilities
{
bool JsonVector3(const TSharedPtr<FJsonObject>& Object, const TCHAR* Name, FVector3d& Out);
bool JsonColor(const TSharedPtr<FJsonObject>& Object, const TCHAR* Name, FLinearColor& Out);
FString BindingTag(const FString& Name);
bool SameObjectRenderDefinition(const FBskObjectDefinition& A, const FBskObjectDefinition& B);
bool SameVisualActorDefinition(const FBskVisualDefinition& A, const FBskVisualDefinition& B);
FName SafeActorName(const FString& Value);
const TCHAR* PrimitiveMeshPath(const FString& Shape);
bool PrepareStaticMeshForRuntime(UStaticMesh* Mesh, const FString& AssetPath);
void PreloadConfiguredVisualOverlays();
void PreloadRuntimeStaticMeshes();
void ApplyUnlitColor(UStaticMeshComponent* Component, UObject* Owner, const FLinearColor& Color);
bool ApplyConfiguredMaterialOverride(UStaticMeshComponent* Component, const FBskGeometryDefinition& Geometry);
void AttachConfiguredVisualOverlay(UStaticMeshComponent* Parent, const FBskGeometryDefinition& Geometry);
void ApplyGeometryMaterial(UStaticMeshComponent* Component, UObject* Owner, const FBskGeometryDefinition& Geometry, double Ambient);
bool IsSensorFrustumKind(const FString& Kind);
}
