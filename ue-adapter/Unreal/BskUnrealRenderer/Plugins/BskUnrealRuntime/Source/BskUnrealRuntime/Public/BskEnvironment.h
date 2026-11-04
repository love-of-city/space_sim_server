#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "BskEnvironment.generated.h"

class ABskEnvironmentBody;
class AStaticMeshActor;
class ADirectionalLight;
class ASpotLight;
class APostProcessVolume;
class ASkyAtmosphere;
class ACameraActor;
class ULightComponent;
class UMeshComponent;
class UMaterialInterface;

/** References to saved level objects; does not rebuild their artist settings. */
UCLASS(Blueprintable)
class BSKUNREALRUNTIME_API ABskEnvironment : public AActor
{
    GENERATED_BODY()

public:
    ABskEnvironment();
    virtual void BeginPlay() override;

    UPROPERTY(EditInstanceOnly, BlueprintReadWrite, Category="Environment|Objects")
    TObjectPtr<ABskEnvironmentBody> Earth;
    UPROPERTY(EditInstanceOnly, BlueprintReadWrite, Category="Environment|Objects")
    TObjectPtr<AStaticMeshActor> StarSphere;
    UPROPERTY(EditInstanceOnly, BlueprintReadWrite, Category="Environment|Objects")
    TObjectPtr<ABskEnvironmentBody> SunVisual;
    UPROPERTY(EditInstanceOnly, BlueprintReadWrite, Category="Environment|Objects")
    TObjectPtr<ADirectionalLight> SunLight;
    UPROPERTY(EditInstanceOnly, BlueprintReadWrite, Category="Environment|Objects")
    TObjectPtr<ADirectionalLight> CelestialSunLight;
    UPROPERTY(EditInstanceOnly, BlueprintReadWrite, Category="Environment|Objects")
    TObjectPtr<ADirectionalLight> FillLight;
    UPROPERTY(EditInstanceOnly, BlueprintReadWrite, Category="Environment|Objects")
    TObjectPtr<ASpotLight> Headlight;
    UPROPERTY(EditInstanceOnly, BlueprintReadWrite, Category="Environment|Objects")
    TObjectPtr<APostProcessVolume> ExposureVolume;
    UPROPERTY(EditInstanceOnly, BlueprintReadWrite, Category="Environment|Objects")
    TObjectPtr<ASkyAtmosphere> EarthAtmosphere;
    UPROPERTY(EditInstanceOnly, BlueprintReadWrite, Category="Environment|Objects")
    TObjectPtr<ACameraActor> PreviewCamera;

    /** When disabled the renderer remains an independent preview and rejects backend scenes. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Environment|Ownership")
    bool bAllowBackendControl = true;

    UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly, Transient, Category="Environment|Ownership")
    bool IsBackendControlled = false;

    /** Physical radius represented by the saved Earth geometry before runtime scaling. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Environment|Geometry", meta=(ClampMin="1.0", Units="m"))
    double EarthRadiusMeters = 6371000.0;

    /** Coordinate scale used when authoring the saved mesh geometry. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Environment|Geometry", meta=(ClampMin="0.001"))
    double GeometryCentimetersPerMeter = 100.0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Environment|Sun", meta=(ClampMin="1000.0", Units="m"))
    double SunVisualDistanceMeters = 10000.0;
    /** Authored disk size for compatible sky materials; physical light angles and eclipses remain ephemeris driven. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Environment|Sun", meta=(ClampMin="0.001", ClampMax="179.0", Units="deg"))
    double SunVisualAngularDiameterDegrees = 2.0;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Environment|Sun", meta=(ClampMin="0.0"))
    double SunVisualEmissiveStrength = 1.5;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Environment|Sun", meta=(ClampMin="0.0"))
    double SunVisualEffectScale = 1.0;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Environment|Sun", meta=(ClampMin="0.0"))
    double SunIlluminanceScale = 1.0;

    /** Captures only the running world's copy, once, before backend takeover. */
    void CapturePreviewState();
    void RestorePreviewState();
    void SetBackendControlled(bool bControlled);
    FTransform GetPreviewTransform(const AActor* Actor) const;

    /** Explicit authoring action; also works in headless editor commandlets. */
    UFUNCTION(BlueprintCallable, CallInEditor, Category="Environment|Preview")
    void StoreEditorPreviewView();

private:
    struct FActorPreviewState
    {
        TWeakObjectPtr<AActor> Actor;
        FTransform Transform;
        bool bHidden = false;
        TWeakObjectPtr<USceneComponent> AttachParent;
        FName AttachSocket;
    };
    struct FComponentPreviewState
    {
        TWeakObjectPtr<USceneComponent> Component;
        FTransform RelativeTransform;
        bool bVisible = true;
        bool bHiddenInGame = false;
    };
    struct FLightPreviewState
    {
        TWeakObjectPtr<ULightComponent> Component;
        float Intensity = 0.0f;
        FLinearColor Color;
        float SourceAngle = 0.0f;
        float SpecularScale = 1.0f;
        bool bCastShadows = true;
        float AttenuationRadius = 0.0f;
        float InnerConeAngle = 0.0f;
        float OuterConeAngle = 0.0f;
    };
    struct FMaterialPreviewState
    {
        TWeakObjectPtr<UMeshComponent> Component;
        int32 MaterialIndex = 0;
        int32 SavedMaterialIndex = 0;
    };

    bool bHasPreviewState = false;
    float PreviewAtmosphereRadius = 6371.0f;
    float PreviewAtmosphereHeight = 100.0f;
    float PreviewAtmosphereAerialPerspectiveStart = 0.1f;
    TArray<FActorPreviewState> ActorPreviewStates;
    TArray<FComponentPreviewState> ComponentPreviewStates;
    TArray<FLightPreviewState> LightPreviewStates;
    TArray<FMaterialPreviewState> MaterialPreviewStates;
    UPROPERTY(Transient)
    TArray<TObjectPtr<UMaterialInterface>> PreviewMaterials;
};
