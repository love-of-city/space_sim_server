#include "BskRendererGameMode.h"

#include "BskCameraPawn.h"
#include "BskRendererHUD.h"
#include "BskSceneController.h"
#include "BskUnrealRuntime.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"

ABskRendererGameMode::ABskRendererGameMode()
{
    DefaultPawnClass = ABskCameraPawn::StaticClass();
    HUDClass = ABskRendererHUD::StaticClass();
}

void ABskRendererGameMode::StartPlay()
{
    Super::StartPlay();
    ABskSceneController* Scene = GetWorld()->SpawnActor<ABskSceneController>();
    APlayerController* Player = GetWorld()->GetFirstPlayerController();
    APawn* Pawn = Player ? Player->GetPawn() : nullptr;
    if (Scene && Pawn && Player)
    {
        const FVector Eye = Scene->GetConfiguredCameraPositionCentimeters();
        const FVector Target = Scene->GetConfiguredCameraLookAtCentimeters();
        Pawn->SetActorLocation(Eye);
        Player->SetControlRotation((Target - Eye).Rotation());
        if (ABskCameraPawn* BskPawn = Cast<ABskCameraPawn>(Pawn))
        {
            BskPawn->SetMainViewTransform(Eye, (Target - Eye).Rotation());
        }
        Player->SetViewTarget(Pawn);
        Player->bShowMouseCursor = false;
        UE_LOG(LogBskUnreal, Display, TEXT("BSK camera ready: pawn=%s eye_cm=%s target_cm=%s"),
            *Pawn->GetName(), *Eye.ToString(), *Target.ToString());
    }
    else
    {
        UE_LOG(LogBskUnreal, Error, TEXT("BSK camera setup failed: scene=%s player=%s pawn=%s"),
            Scene ? TEXT("valid") : TEXT("null"),
            Player ? TEXT("valid") : TEXT("null"),
            Pawn ? TEXT("valid") : TEXT("null"));
    }
}
