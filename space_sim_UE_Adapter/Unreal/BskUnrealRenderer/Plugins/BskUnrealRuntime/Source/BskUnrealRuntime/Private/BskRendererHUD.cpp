#include "BskRendererHUD.h"

#include "BskRenderWorldSubsystem.h"
#include "BskSceneController.h"
#include "BskCameraPawn.h"
#include "BskRelativeView.h"
#include "Camera/PlayerCameraManager.h"
#include "Misc/CommandLine.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Engine/TextureRenderTarget2D.h"
#include "CanvasItem.h"
#include "GameFramework/PlayerController.h"
#include "Kismet/GameplayStatics.h"

ABskSceneController* ABskRendererHUD::FindSceneController() const
{
    if (!GetWorld()) return nullptr;
    TArray<AActor*> Controllers;
    UGameplayStatics::GetAllActorsOfClass(GetWorld(), ABskSceneController::StaticClass(), Controllers);
    return Controllers.IsEmpty() ? nullptr : Cast<ABskSceneController>(Controllers[0]);
}

void ABskRendererHUD::ToggleInteractiveMissionUi()
{
    if (!bMissionUiVisible)
    {
        bMissionUiVisible = true;
    }
    bMissionUiInteractive = !bMissionUiInteractive;
    if (!PlayerOwner) return;
    PlayerOwner->bShowMouseCursor = bMissionUiInteractive;
    // Canvas HUD hit boxes do not receive NotifyHitBoxClick merely because the
    // cursor is visible. APlayerController click events are disabled by
    // default, so explicitly enable them only while the mission UI is active.
    PlayerOwner->bEnableClickEvents = bMissionUiInteractive;
    PlayerOwner->bEnableMouseOverEvents = bMissionUiInteractive;
    if (bMissionUiInteractive)
    {
        FInputModeGameAndUI Mode;
        Mode.SetHideCursorDuringCapture(false);
        Mode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
        PlayerOwner->SetInputMode(Mode);
        UiNotice = TEXT("Mission UI enabled: click a declared command");
    }
    else
    {
        PlayerOwner->SetInputMode(FInputModeGameOnly());
        ArmedCommandIndex = INDEX_NONE;
        UiNotice = TEXT("Free camera control restored");
    }
}

void ABskRendererHUD::ToggleMissionUiVisibility()
{
    bMissionUiVisible = !bMissionUiVisible;
    if (!bMissionUiVisible && bMissionUiInteractive)
    {
        ToggleInteractiveMissionUi();
    }
    UiNotice = bMissionUiVisible ? TEXT("Mission panel shown") : TEXT("Mission panel hidden; press M to restore");
}

void ABskRendererHUD::NotifyHitBoxClick(FName BoxName)
{
    Super::NotifyHitBoxClick(BoxName);
    if (!bMissionUiInteractive) return;
    UE_LOG(LogTemp, Verbose, TEXT("BSK mission UI clicked hit box: %s"), *BoxName.ToString());
    ABskSceneController* Scene = FindSceneController();
    if (!Scene) return;
    if (BoxName == TEXT("BskClearEvents"))
    {
        Scene->ClearMissionEvents();
        UiNotice = TEXT("Event history cleared");
        return;
    }
    if (BoxName == TEXT("BskCloseMissionUi"))
    {
        ToggleMissionUiVisibility();
        return;
    }
    const FString Name = BoxName.ToString();
    if (!Name.StartsWith(TEXT("BskCommand_"))) return;
    int32 Index = INDEX_NONE;
    if (!LexTryParseString(Index, *Name.RightChop(11))) return;
    TArray<FBskUiCommandDefinition> Commands;
    Scene->GetUiCommands(Commands);
    if (!Commands.IsValidIndex(Index)) return;
    const FBskUiCommandDefinition& Command = Commands[Index];
    const double Now = GetWorld()->GetRealTimeSeconds();
    if (Command.bRequiresConfirmation && (ArmedCommandIndex != Index || Now > ArmedUntilSeconds))
    {
        ArmedCommandIndex = Index;
        ArmedUntilSeconds = Now + 4.0;
        UiNotice = FString::Printf(TEXT("Click '%s' again within 4 s to confirm"), *Command.Label);
        return;
    }
    FString Error;
    if (Scene->SendUiCommand(Command.Command, Command.TargetId, Command.PayloadJson, Error))
    {
        UiNotice = FString::Printf(TEXT("Sent: %s"), *Command.Label);
    }
    else
    {
        UiNotice = FString::Printf(TEXT("Command failed: %s"), *Error);
    }
    ArmedCommandIndex = INDEX_NONE;
}

bool ABskRendererHUD::DrawRelativeView(const ABskSceneController* Scene)
{
    FBskRelativeViewState View;
    if (!Scene || !PlayerOwner || !Scene->GetRelativeViewState(View)) return false;
    const ABskCameraPawn* Pawn = Cast<ABskCameraPawn>(PlayerOwner->GetPawn());
    const FString Mode = Pawn ? Pawn->GetRelativeViewMode() : TEXT("free");
    const bool bFixed = Mode == TEXT("overview-fixed");
    const bool bOverview = Mode == TEXT("overview") || bFixed;
    const FString Name = bFixed ? TEXT("RENDEZVOUS OVERVIEW | FIXED SCALE") : bOverview ? TEXT("RENDEZVOUS OVERVIEW | AUTO-FIT")
        : Mode == TEXT("chaser") ? TEXT("CHASER CLOSE-UP")
        : Mode == TEXT("target") ? TEXT("TARGET CLOSE-UP") : TEXT("FREE VIEW");
    const auto Label = [this](const FString& Text, const FVector2D& At, const FLinearColor& Color)
    {
        FCanvasTextItem Item(At, FText::FromString(Text), GEngine->GetMediumFont(), Color);
        Item.EnableShadow(FLinearColor::Black);
        Canvas->DrawItem(Item);
    };
    Label(Name, FVector2D(24, 22), FLinearColor::White);
    TArray<FBskMissionEventView> Events;
    Scene->GetMissionEvents(Events);
    const bool bNearField = Events.ContainsByPredicate([](const FBskMissionEventView& Event)
    {
        return Event.Kind == TEXT("near_field_status");
    });
    // Relative-view anchors use the catalog's stowed COM offsets. Near-field
    // telemetry below computes the live articulated COM independently.
    Label(FString::Printf(TEXT("%s: %s | %s=%.1f s | models 1:1 | markers not to scale"),
        bNearField ? TEXT("Stowed reference distance") : TEXT("Center distance"),
        *BskRelativeView::Distance(View.DistanceMeters), bNearField ? TEXT("near elapsed") : TEXT("mission t"),
        View.SimulationSeconds), FVector2D(24, 49), FLinearColor(0.75f, 0.85f, 0.92f));
    const bool bDark = View.PrimarySunlight < 0.001 && View.SecondarySunlight < 0.001;
    Label(bNearField ? (bDark ? TEXT("VIEW FILL | ECLIPSE; physical sunlight is zero")
                                             : TEXT("VIEW FILL | physical sunlight plus display lighting"))
                : bDark ? TEXT("ECLIPSE | physical sunlight is zero; position markers remain visible")
                : TEXT("PHYSICAL SUNLIGHT | no observation fill light"), FVector2D(24, 76),
          bDark ? FLinearColor(1.0f, 0.75f, 0.3f) : FLinearColor(0.7f, 0.78f, 0.85f));
    FVector2D Primary, Secondary;
    const auto Visible = [this](const FVector& Point, FVector2D& Screen)
    {
        return PlayerOwner->ProjectWorldLocationToScreen(Point, Screen, true) &&
            Screen.X >= 10 && Screen.X <= Canvas->SizeX - 10 && Screen.Y >= 105 && Screen.Y <= Canvas->SizeY - 50;
    };
    const bool bPrimary = Visible(View.PrimaryCentimeters, Primary);
    const bool bSecondary = Visible(View.SecondaryCentimeters, Secondary);
    const auto Line = [this](const FVector2D& A, const FVector2D& B, const FLinearColor& Color)
    {
        FCanvasLineItem Item(A, B); Item.SetColor(Color); Canvas->DrawItem(Item);
    };
    if (bPrimary && bSecondary) Line(Primary, Secondary, FLinearColor(0.3f, 0.65f, 0.8f, 0.7f));
    const auto Marker = [this, &Label](const FVector2D& Point, const FString& Text, const FLinearColor& Color)
    {
        FCanvasBoxItem Box(Point - FVector2D(6, 6), FVector2D(12, 12)); Box.SetColor(Color); Canvas->DrawItem(Box);
        Label(Text, FVector2D(FMath::Clamp(Point.X + 12.0, 12.0, static_cast<double>(Canvas->SizeX - 180)), Point.Y - 28), Color);
    };
    if (bPrimary) Marker(Primary, View.PrimaryLabel, FLinearColor(0.25f, 0.8f, 1.0f));
    if (bSecondary) Marker(Secondary, View.SecondaryLabel, FLinearColor(1.0f, 0.72f, 0.25f));
    if (!bPrimary || !bSecondary)
    {
        const FString Missing = !bPrimary && !bSecondary ? View.PrimaryLabel + TEXT(" / ") + View.SecondaryLabel
            : !bPrimary ? View.PrimaryLabel : View.SecondaryLabel;
        Label(Missing + TEXT(" outside view | select rendezvous overview to locate both"),
              FVector2D(24, Canvas->SizeY - 55), FLinearColor(1.0f, 0.78f, 0.35f));
    }
    if (bOverview && bPrimary && bSecondary && View.DistanceMeters > 0.001)
    {
        const double PixelsPerMeter = FVector2D::Distance(Primary, Secondary) / View.DistanceMeters;
        const double Meters = BskRelativeView::RulerMeters(160.0 / FMath::Max(PixelsPerMeter, 1e-12));
        const float Width = static_cast<float>(Meters * PixelsPerMeter);
        const float Y = Canvas->SizeY - 28.0f;
        Line(FVector2D(26, Y), FVector2D(26 + Width, Y), FLinearColor::White);
        Line(FVector2D(26, Y - 5), FVector2D(26, Y + 5), FLinearColor::White);
        Line(FVector2D(26 + Width, Y - 5), FVector2D(26 + Width, Y + 5), FLinearColor::White);
        Label(BskRelativeView::Distance(Meters), FVector2D(26, Y - 27), FLinearColor::White);
    }
    return true;
}

void ABskRendererHUD::DrawHUD()
{
    Super::DrawHUD();
    if (!Canvas || !GetWorld()) return;
    const ABskSceneController* Scene = FindSceneController();
    const UBskRenderWorldSubsystem* Subsystem = GetWorld()->GetSubsystem<UBskRenderWorldSubsystem>();
    const FString Status = Scene ? Scene->GetReceiverStatus() : TEXT("renderer not ready");
    const double SimulationSeconds = Subsystem ? static_cast<double>(Subsystem->GetSimulationTimeNanoseconds()) * 1.0e-9 : 0.0;
    const TCHAR* CssStatus = Scene && Scene->IsVisualKindVisible(TEXT("css")) ? TEXT("ON") : TEXT("OFF");
    const TCHAR* SensorStatus = Scene && Scene->IsVisualKindVisible(TEXT("generic_sensor")) ? TEXT("ON") : TEXT("OFF");
    const TCHAR* TransceiverStatus = Scene && Scene->IsVisualKindVisible(TEXT("transceiver")) ? TEXT("ON") : TEXT("OFF");
    TArray<FBskPictureInPictureView> PictureInPictureViews;
    if (Scene) Scene->GetPictureInPictureViews(PictureInPictureViews);
    FString CameraStatus;
    for (const FBskPictureInPictureView& View : PictureInPictureViews)
    {
        if (!CameraStatus.IsEmpty()) CameraStatus += TEXT(" | ");
        CameraStatus += FString::Printf(TEXT("%d %s:%s"), View.Slot + 3, *View.DisplayName, View.bVisible ? TEXT("ON") : TEXT("OFF"));
    }
    const FString Text = FString::Printf(
        TEXT("BSK Unreal Renderer\n%s\nSim: %.3f s   Frame: %lld   Objects: %d   Visuals: %d   Cameras: %d\nW/S A/D Q/E move | Mouse look | M panel | TAB interact\n1 CSS:%s | 2 Sensor:%s | 3 Comms:%s%s%s"),
        *Status,
        SimulationSeconds,
        Scene ? Scene->GetLastFrameId() : -1,
        Scene ? Scene->GetObjectCount() : 0,
        Scene ? Scene->GetVisualCount() : 0,
        Scene ? Scene->GetCameraCount() : 0,
        CssStatus,
        SensorStatus,
        TransceiverStatus,
        CameraStatus.IsEmpty() ? TEXT("") : TEXT("\n"),
        *CameraStatus);
    if (!DrawRelativeView(Scene) || FParse::Param(FCommandLine::Get(), TEXT("BskCameraDiagnostics")))
    {
        Canvas->SetDrawColor(FColor(178, 230, 255));
        Canvas->DrawText(GEngine->GetSmallFont(), Text, 24.0f, 24.0f, 1.0f, 1.0f, FFontRenderInfo());
    }

    // The near-field demonstration status stays visible without opening the
    // interactive mission panel. It carries explicit sensor/capture boundaries.
    if (Scene)
    {
        TArray<FBskMissionEventView> Events;
        Scene->GetMissionEvents(Events);
        TArray<FString> NearLines;
        for (int32 Index = Events.Num() - 1; Index >= 0 && NearLines.Num() < 5; --Index)
        {
            if (Events[Index].Kind == TEXT("near_field_status")) NearLines.Insert(Events[Index].Message, 0);
        }
        if (NearLines.Num() > 0)
        {
            const float X = 24.0f;
            const float Y = FMath::Max(155.0f, static_cast<float>(Canvas->SizeY) - 155.0f);
            FCanvasTileItem Background(FVector2D(X, Y), GWhiteTexture,
                FVector2D(FMath::Min(780.0f, static_cast<float>(Canvas->SizeX) - 48.0f), 112.0f), FLinearColor(0.008f, 0.018f, 0.035f, 0.88f));
            Background.BlendMode = SE_BLEND_Translucent;
            Canvas->DrawItem(Background);
            Canvas->SetDrawColor(FColor(180, 230, 255));
            for (int32 Index = 0; Index < NearLines.Num(); ++Index)
                Canvas->DrawText(GEngine->GetSmallFont(), NearLines[Index], X + 10.0f, Y + 8.0f + Index * 19.0f);
        }
    }

    if (Scene && bMissionUiVisible)
    {
        const float PanelX = 24.0f;
        const float PanelY = 142.0f;
        const float PanelWidth = FMath::Min(520.0f, static_cast<float>(Canvas->SizeX) * 0.43f);
        const float PanelHeight = FMath::Min(520.0f, static_cast<float>(Canvas->SizeY) - PanelY - 24.0f);
        FCanvasTileItem Panel(FVector2D(PanelX, PanelY), GWhiteTexture, FVector2D(PanelWidth, PanelHeight), FLinearColor(0.008f, 0.018f, 0.035f, 0.88f));
        Panel.BlendMode = SE_BLEND_Translucent;
        Canvas->DrawItem(Panel);
        FCanvasBoxItem PanelBorder(FVector2D(PanelX, PanelY), FVector2D(PanelWidth, PanelHeight));
        PanelBorder.SetColor(bMissionUiInteractive ? FLinearColor(0.15f, 0.78f, 1.0f) : FLinearColor(0.12f, 0.32f, 0.45f));
        Canvas->DrawItem(PanelBorder);

        float CursorY = PanelY + 12.0f;
        Canvas->SetDrawColor(FColor(128, 220, 255));
        Canvas->DrawText(GEngine->GetSmallFont(), TEXT("MISSION CONTROL"), PanelX + 12.0f, CursorY);
        const FVector2D ClosePosition(PanelX + PanelWidth - 34.0f, PanelY + 7.0f);
        FCanvasTileItem CloseButton(ClosePosition, GWhiteTexture, FVector2D(24.0f, 22.0f), FLinearColor(0.20f, 0.08f, 0.08f, 0.95f));
        CloseButton.BlendMode = SE_BLEND_Translucent;
        Canvas->DrawItem(CloseButton);
        Canvas->SetDrawColor(FColor(255, 205, 205));
        Canvas->DrawText(GEngine->GetSmallFont(), TEXT("X"), ClosePosition.X + 7.0f, ClosePosition.Y + 2.0f);
        AddHitBox(ClosePosition, FVector2D(24.0f, 22.0f), TEXT("BskCloseMissionUi"), true, 300);
        Canvas->SetDrawColor(FColor(190, 205, 215));
        Canvas->DrawText(GEngine->GetSmallFont(),
            FString::Printf(TEXT("Link: %s | Pending: %d | %s"),
                bMissionUiInteractive ? TEXT("INTERACTIVE") : TEXT("VIEW ONLY"),
                Scene->GetPendingCommandCount(), *Scene->GetLastCommandStatus()),
            PanelX + 155.0f, CursorY);
        CursorY += 25.0f;

        TArray<FBskUiCommandDefinition> Commands;
        Scene->GetUiCommands(Commands);
        const int32 VisibleCommandCount = FMath::Min(Commands.Num(), 6);
        const float ButtonWidth = (PanelWidth - 36.0f) * 0.5f;
        for (int32 Index = 0; Index < VisibleCommandCount; ++Index)
        {
            const int32 Column = Index % 2;
            const int32 Row = Index / 2;
            const FVector2D Position(PanelX + 12.0f + Column * (ButtonWidth + 12.0f), CursorY + Row * 34.0f);
            const FVector2D Size(ButtonWidth, 27.0f);
            const bool bArmed = ArmedCommandIndex == Index && GetWorld()->GetRealTimeSeconds() <= ArmedUntilSeconds;
            const FLinearColor ButtonColor = !bMissionUiInteractive
                ? FLinearColor(0.09f, 0.12f, 0.15f, 0.9f)
                : bArmed ? FLinearColor(0.75f, 0.20f, 0.08f, 0.95f)
                : FLinearColor(0.04f, 0.28f, 0.42f, 0.95f);
            FCanvasTileItem Button(Position, GWhiteTexture, Size, ButtonColor);
            Button.BlendMode = SE_BLEND_Translucent;
            Canvas->DrawItem(Button);
            Canvas->SetDrawColor(FColor(220, 240, 248));
            const FString Label = FString::Printf(TEXT("%s%s"), Commands[Index].bRequiresConfirmation ? TEXT("! ") : TEXT(""), *Commands[Index].Label.Left(28));
            Canvas->DrawText(GEngine->GetSmallFont(), Label, Position.X + 7.0f, Position.Y + 5.0f);
            AddHitBox(Position, Size, FName(*FString::Printf(TEXT("BskCommand_%d"), Index)), true, 100 + Index);
        }
        CursorY += FMath::CeilToInt(VisibleCommandCount / 2.0f) * 34.0f + 8.0f;
        if (!UiNotice.IsEmpty())
        {
            Canvas->SetDrawColor(FColor(255, 207, 96));
            Canvas->DrawText(GEngine->GetSmallFont(), UiNotice.Left(72), PanelX + 12.0f, CursorY);
            CursorY += 22.0f;
        }

        Canvas->SetDrawColor(FColor(128, 220, 255));
        Canvas->DrawText(GEngine->GetSmallFont(), TEXT("EVENT TIMELINE"), PanelX + 12.0f, CursorY);
        const FVector2D ClearPosition(PanelX + PanelWidth - 86.0f, CursorY - 4.0f);
        FCanvasTileItem ClearButton(ClearPosition, GWhiteTexture, FVector2D(72.0f, 23.0f), FLinearColor(0.12f, 0.18f, 0.22f, 0.95f));
        ClearButton.BlendMode = SE_BLEND_Translucent;
        Canvas->DrawItem(ClearButton);
        Canvas->SetDrawColor(FColor(190, 205, 215));
        Canvas->DrawText(GEngine->GetSmallFont(), TEXT("CLEAR"), ClearPosition.X + 16.0f, ClearPosition.Y + 3.0f);
        AddHitBox(ClearPosition, FVector2D(72.0f, 23.0f), TEXT("BskClearEvents"), true, 200);
        CursorY += 25.0f;

        TArray<FBskMissionEventView> Events;
        Scene->GetMissionEvents(Events);
        const int32 MaxVisibleEvents = FMath::Max(1, FMath::FloorToInt((PanelY + PanelHeight - CursorY - 8.0f) / 21.0f));
        const int32 FirstEvent = FMath::Max(0, Events.Num() - MaxVisibleEvents);
        for (int32 Index = FirstEvent; Index < Events.Num(); ++Index)
        {
            const FBskMissionEventView& Event = Events[Index];
            const FColor EventColor = Event.Severity.Equals(TEXT("error"), ESearchCase::IgnoreCase) ? FColor(255, 92, 82)
                : Event.Severity.Equals(TEXT("warning"), ESearchCase::IgnoreCase) ? FColor(255, 196, 74)
                : Event.Severity.Equals(TEXT("command"), ESearchCase::IgnoreCase) ? FColor(125, 205, 255)
                : FColor(180, 214, 198);
            Canvas->SetDrawColor(EventColor);
            Canvas->DrawText(GEngine->GetSmallFont(),
                FString::Printf(TEXT("%8.3f  %-18s %s"), Event.SimulationTimeNanoseconds * 1.0e-9, *Event.Kind.Left(18), *Event.Message.Left(48)),
                PanelX + 12.0f, CursorY);
            CursorY += 21.0f;
        }
    }

    const float Margin = 24.0f;
    const float ViewWidth = FMath::Min(480.0f, static_cast<float>(Canvas->SizeX) * 0.34f);
    const float ViewHeight = ViewWidth * 9.0f / 16.0f;
    const float ViewX = static_cast<float>(Canvas->SizeX) - ViewWidth - Margin;
    int32 VisibleIndex = 0;
    for (const FBskPictureInPictureView& View : PictureInPictureViews)
    {
        if (!View.bVisible || !View.Texture) continue;
        const float ViewY = Margin + VisibleIndex * (ViewHeight + 42.0f);
        if (ViewY + ViewHeight > Canvas->SizeY - Margin) break;
        FCanvasTileItem Tile(FVector2D(ViewX, ViewY), View.Texture->GetResource(), FVector2D(ViewWidth, ViewHeight), FLinearColor::White);
        Tile.BlendMode = SE_BLEND_Opaque;
        Canvas->DrawItem(Tile);
        FCanvasBoxItem Border(FVector2D(ViewX - 1.0f, ViewY - 1.0f), FVector2D(ViewWidth + 2.0f, ViewHeight + 2.0f));
        Border.SetColor(FLinearColor(0.18f, 0.72f, 1.0f, 1.0f));
        Canvas->DrawItem(Border);
        Canvas->SetDrawColor(FColor(178, 230, 255));
        Canvas->DrawText(
            GEngine->GetSmallFont(),
            FString::Printf(TEXT("[%d] %s  LIVE"), View.Slot + 3, *View.DisplayName),
            ViewX,
            ViewY + ViewHeight + 5.0f,
            1.0f,
            1.0f,
            FFontRenderInfo());
        ++VisibleIndex;
    }
}
