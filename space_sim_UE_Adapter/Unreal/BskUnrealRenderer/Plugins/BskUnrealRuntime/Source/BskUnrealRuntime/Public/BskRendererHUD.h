#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "BskRendererHUD.generated.h"

/** Lightweight runtime mission overlay with connection and simulation status. */
UCLASS()
class BSKUNREALRUNTIME_API ABskRendererHUD : public AHUD
{
    GENERATED_BODY()

public:
    virtual void DrawHUD() override;
    virtual void NotifyHitBoxClick(FName BoxName) override;

    UFUNCTION(BlueprintCallable, Category="BSK Renderer|Mission UI")
    void ToggleInteractiveMissionUi();

    UFUNCTION(BlueprintCallable, Category="BSK Renderer|Mission UI")
    void ToggleMissionUiVisibility();

    UFUNCTION(BlueprintPure, Category="BSK Renderer|Mission UI")
    bool IsMissionUiInteractive() const { return bMissionUiInteractive; }

    UFUNCTION(BlueprintPure, Category="BSK Renderer|Mission UI")
    bool IsMissionUiVisible() const { return bMissionUiVisible; }

private:
    class ABskSceneController* FindSceneController() const;
    bool bMissionUiVisible = false;
    bool bMissionUiInteractive = false;
    int32 ArmedCommandIndex = INDEX_NONE;
    double ArmedUntilSeconds = 0.0;
    FString UiNotice;
};
