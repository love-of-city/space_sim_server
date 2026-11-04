#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "BskEnvironmentBody.generated.h"

class UStaticMeshComponent;
class UParticleSystemComponent;
class UNiagaraComponent;
class UNiagaraSystem;
class UMaterialInterface;

/** Saved, editor-visible geometry for a planet or the near-camera Sun proxy. */
UCLASS(Blueprintable)
class BSKUNREALRUNTIME_API ABskEnvironmentBody : public AActor
{
    GENERATED_BODY()

public:
    ABskEnvironmentBody();

    virtual void OnConstruction(const FTransform& Transform) override;
    virtual void BeginPlay() override;

    /** Refresh scale-dependent visual parameters after a backend radius change. */
    UFUNCTION(BlueprintCallable, Category="Environment")
    void RefreshVisualScale();

    /** Editor migration bridge for Niagara data not exposed to editor Python. */
    UFUNCTION(BlueprintCallable, Category="Environment|Migration", meta=(DevelopmentOnly))
    static bool PrepareAttachedAuroraAsset(UNiagaraSystem* System, UMaterialInterface* Material);

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Environment")
    TObjectPtr<USceneComponent> BodyRoot;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Environment")
    TObjectPtr<UStaticMeshComponent> Surface;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Environment")
    TObjectPtr<UStaticMeshComponent> Clouds;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Environment")
    TObjectPtr<UStaticMeshComponent> Atmosphere;

    /** Optional local-space aurora; its pose follows the same authoritative body. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Environment")
    TObjectPtr<UNiagaraComponent> Aurora;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Environment")
    TObjectPtr<UParticleSystemComponent> SunBursts;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Environment")
    TObjectPtr<UParticleSystemComponent> SunHalo;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Environment")
    TObjectPtr<UParticleSystemComponent> SunLines;
};
