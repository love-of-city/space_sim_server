#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "BskRendererGameMode.generated.h"

UCLASS()
class BSKUNREALRUNTIME_API ABskRendererGameMode : public AGameModeBase
{
    GENERATED_BODY()

public:
    ABskRendererGameMode();
    virtual void StartPlay() override;
};
