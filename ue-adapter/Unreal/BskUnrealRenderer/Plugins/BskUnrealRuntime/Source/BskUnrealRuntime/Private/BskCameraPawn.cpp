#include "BskCameraPawn.h"

#include "BskSceneController.h"
#include "BskRendererHUD.h"
#include "Async/Async.h"
#include "IPixelStreaming2Module.h"
#include "IPixelStreaming2Streamer.h"
#include "Camera/CameraComponent.h"
#include "Camera/PlayerCameraManager.h"
#include "Dom/JsonObject.h"
#include "Misc/CommandLine.h"
#include "Serialization/JsonSerializer.h"
#include "Components/InputComponent.h"
#include "InputCoreTypes.h"
#include "GameFramework/FloatingPawnMovement.h"
#include "GameFramework/PlayerController.h"
#include "Kismet/GameplayStatics.h"

ABskCameraPawn::ABskCameraPawn()
{
    PrimaryActorTick.bCanEverTick = true;
    // Use explicit key state for Pixel Streaming free-flight.  The inherited
    // DefaultPawn axis bindings are not needed and can compete with the
    // browser-controlled robot input.
    bAddDefaultMovementBindings = false;
    Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
    Camera->SetupAttachment(GetRootComponent());
    Camera->bUsePawnControlRotation = true;
    Camera->FieldOfView = 60.0f;
    bUseControllerRotationYaw = true;
    bUseControllerRotationPitch = true;
    if (UFloatingPawnMovement* Movement = Cast<UFloatingPawnMovement>(GetMovementComponent()))
    {
        Movement->MaxSpeed = 5000.0f;
        Movement->Acceleration = 12000.0f;
        Movement->Deceleration = 12000.0f;
    }
}

void ABskCameraPawn::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
    RegisterRemoteCameraInput();
    if (CameraMode == EBskCameraMode::MainView)
    {
        // The main viewport is driven by SetMainViewTransform().  Re-applying
        // the same transform every render tick causes needless camera/LWC
        // updates and can make the viewport visibly flicker.
        return;
    }
    if (CameraMode == EBskCameraMode::Free)
    {
        if (bRemoteCameraControl)
        {
            // A closed tab, lost release, or disconnected data channel must
            // never leave a camera flying. Use wall time, not simulation time.
            if (FPlatformTime::Seconds() - LastRemoteCameraInputSeconds > FBskCameraInput::WatchdogSeconds)
            {
                ReturnToMainView();
                return;
            }
            const FVector Delta = RemoteCameraInput.MovementDelta(FreeCameraOrientation, DeltaSeconds);
            if (!Delta.IsNearlyZero()) SetActorLocation(GetActorLocation() + Delta, false);
            return; // Do not combine remote input with native key/axis fallbacks.
        }
        FVector MoveDirection = FVector::ZeroVector;
        // Poll the PlayerController as a final fallback.  Pixel Streaming
        // injects browser key events into the controller, but depending on the
        // active input component those events may not invoke a pawn BindKey
        // callback.  IsInputKeyDown observes the same authoritative key state
        // and makes held W/A/S/D/Q/E movement independent of that callback.
        const APlayerController* PlayerController = Cast<APlayerController>(Controller);
        const bool bForwardDown = bMoveForward || (PlayerController && PlayerController->IsInputKeyDown(EKeys::W));
        const bool bBackwardDown = bMoveBackward || (PlayerController && PlayerController->IsInputKeyDown(EKeys::S));
        const bool bRightDown = bMoveRight || (PlayerController && PlayerController->IsInputKeyDown(EKeys::D));
        const bool bLeftDown = bMoveLeft || (PlayerController && PlayerController->IsInputKeyDown(EKeys::A));
        const bool bUpDown = bMoveUp || (PlayerController && PlayerController->IsInputKeyDown(EKeys::E));
        const bool bDownDown = bMoveDown || (PlayerController && PlayerController->IsInputKeyDown(EKeys::Q));
        const float Forward = (bForwardDown || bBackwardDown)
            ? (bForwardDown ? 1.0f : 0.0f) - (bBackwardDown ? 1.0f : 0.0f)
            : MoveForwardAxisValue;
        const float Right = (bRightDown || bLeftDown)
            ? (bRightDown ? 1.0f : 0.0f) - (bLeftDown ? 1.0f : 0.0f)
            : MoveRightAxisValue;
        const float Up = (bUpDown || bDownDown)
            ? (bUpDown ? 1.0f : 0.0f) - (bDownDown ? 1.0f : 0.0f)
            : MoveUpAxisValue;
        if (!FMath::IsNearlyZero(Forward)) MoveDirection += FreeCameraOrientation.GetAxisX() * Forward;
        if (!FMath::IsNearlyZero(Right)) MoveDirection += FreeCameraOrientation.GetAxisY() * Right;
        if (!FMath::IsNearlyZero(Up)) MoveDirection += FVector::UpVector * Up;
        if (!MoveDirection.IsNearlyZero())
        {
            const FVector Delta = MoveDirection.GetSafeNormal() * FreeCameraSpeedCentimetersPerSecond * DeltaSeconds;
            // Free-flight is a spectator camera, so do not let scene collision
            // geometry stop the camera.
            SetActorLocation(GetActorLocation() + Delta, false);
        }
        return;
    }
    UpdateTrackingCamera();
}

FRotator ABskCameraPawn::GetViewRotation() const
{
    return bFreeCameraActive ? FreeCameraOrientation.Rotator() : Super::GetViewRotation();
}

void ABskCameraPawn::ApplyFreeLook(const FVector2D& Displacement)
{
    if (Displacement.IsNearlyZero()) return;
    FBskCameraInput Input;
    Input.bActive = true;
    Input.Look = Displacement;
    FreeCameraOrientation = Input.ApplyLook(FreeCameraOrientation);
    SetActorRotation(FreeCameraOrientation);
}

void ABskCameraPawn::UpdateTrackingCamera()
{
    if (!IsValid(TargetActor)) return;
    const FVector TargetLocation = TargetActor->GetActorLocation();
    FVector CameraLocation;
    if (CameraMode == EBskCameraMode::Orbit)
    {
        const FRotator OrbitRotation(OrbitPitchDegrees, OrbitYawDegrees, 0.0);
        CameraLocation = TargetLocation - OrbitRotation.Vector() * OrbitDistanceCentimeters;
    }
    else
    {
        CameraLocation = TargetActor->GetActorTransform().TransformPosition(FollowOffsetCentimeters);
    }
    const FRotator Rotation = (TargetLocation - CameraLocation).Rotation();
    SetActorLocationAndRotation(CameraLocation, Rotation);
    if (Controller) Controller->SetControlRotation(Rotation);
}

void ABskCameraPawn::RegisterRemoteCameraInput()
{
    if (!IPixelStreaming2Module::IsAvailable()) return;
    IPixelStreaming2Module& Module = IPixelStreaming2Module::Get();
    if (!Module.IsReady()) return;
    const TSharedPtr<IPixelStreaming2Streamer> Streamer = Module.FindStreamer(Module.GetDefaultStreamerID());
    const TSharedPtr<IPixelStreaming2InputHandler> Handler = Streamer ? Streamer->GetInputHandler().Pin() : nullptr;
    if (!Handler || RemoteInputHandler.Pin() == Handler) return;
    const TWeakObjectPtr<ABskCameraPawn> WeakPawn(this);
    Handler->SetCommandHandler(TEXT("BskCameraInput"),
        [WeakPawn](FString, FString, FString Payload)
        {
            if (Payload.Len() > 2048) return;
            // Pixel Streaming dispatch/thread details are not a camera contract.
            // Always mutate actors on the game thread, retaining only a weak pawn.
            const auto Apply = [WeakPawn, Payload]()
            {
                if (ABskCameraPawn* Pawn = WeakPawn.Get()) Pawn->ApplyRemoteCameraInput(Payload);
            };
            if (IsInGameThread()) Apply();
            else AsyncTask(ENamedThreads::GameThread, Apply);
        });
    // Read-only runtime probe, deliberately opt-in. Compare accepted input,
    // quaternion state and the actual rendered POV across the WebRTC boundary.
    if (FParse::Param(FCommandLine::Get(), TEXT("BskCameraDiagnostics")))
    {
        const TWeakPtr<IPixelStreaming2Streamer> WeakStreamer(Streamer);
        Handler->SetCommandHandler(TEXT("BskCameraProbe"),
            [WeakPawn, WeakStreamer](FString SourceId, FString, FString)
            {
                const auto Reply = [WeakPawn, WeakStreamer, SourceId]()
                {
                    const ABskCameraPawn* Pawn = WeakPawn.Get();
                    const TSharedPtr<IPixelStreaming2Streamer> TargetStreamer = WeakStreamer.Pin();
                    if (Pawn && TargetStreamer)
                        TargetStreamer->SendPlayerMessage(SourceId, TEXT("Response"), Pawn->GetCameraDiagnostics());
                };
                if (IsInGameThread()) Reply();
                else AsyncTask(ENamedThreads::GameThread, Reply);
            });
    }
    RemoteInputHandler = Handler;
    UE_LOG(LogTemp, Display, TEXT("BSK explicit free-camera input registered on streamer %s"), *Streamer->GetId());
}

FString ABskCameraPawn::GetCameraDiagnostics() const
{
    const TSharedRef<FJsonObject> Data = MakeShared<FJsonObject>();
    Data->SetStringField(TEXT("type"), TEXT("BskCameraDiagnostics"));
    Data->SetBoolField(TEXT("free"), bFreeCameraActive);
    Data->SetBoolField(TEXT("remote"), bRemoteCameraControl);
    Data->SetNumberField(TEXT("packets"), static_cast<double>(RemoteCameraPacketCount));
    Data->SetNumberField(TEXT("look_dx"), TotalRemoteLook.X);
    Data->SetNumberField(TEXT("look_dy"), TotalRemoteLook.Y);
    Data->SetNumberField(TEXT("input_age"), FPlatformTime::Seconds() - LastRemoteCameraInputSeconds);
    const auto AddQuat = [&Data](const TCHAR* Name, const FQuat& Q)
    {
        Data->SetArrayField(Name, { MakeShared<FJsonValueNumber>(Q.X), MakeShared<FJsonValueNumber>(Q.Y),
            MakeShared<FJsonValueNumber>(Q.Z), MakeShared<FJsonValueNumber>(Q.W) });
    };
    AddQuat(TEXT("orientation"), FreeCameraOrientation);
    AddQuat(TEXT("actor"), GetActorQuat());
    AddQuat(TEXT("camera"), Camera->GetComponentQuat());
    Data->SetStringField(TEXT("location"), GetActorLocation().ToString());
    if (const APlayerController* Player = Cast<APlayerController>(Controller))
    {
        AddQuat(TEXT("control"), Player->GetControlRotation().Quaternion());
        if (Player->PlayerCameraManager)
        {
            AddQuat(TEXT("pov"), Player->PlayerCameraManager->GetCameraRotation().Quaternion());
            Data->SetStringField(TEXT("view_target"), GetNameSafe(Player->GetViewTarget()));
        }
    }
    FString Result;
    FJsonSerializer::Serialize(Data, TJsonWriterFactory<>::Create(&Result));
    return Result;
}

bool ABskCameraPawn::ApplyRemoteCameraInput(const FString& Descriptor)
{
    FBskCameraInput Input;
    if (!FBskCameraInput::Parse(Descriptor, Input)) return false;
    ++RemoteCameraPacketCount;
    if (Input.bActive) TotalRemoteLook += Input.Look;
    if (!Input.bActive)
    {
        if (bFreeCameraActive || bRemoteCameraControl) ReturnToMainView();
        return true;
    }
    if (!bFreeCameraActive) ToggleFreeCamera();
    bRemoteCameraControl = true;
    RemoteCameraInput = Input;
    LastRemoteCameraInputSeconds = FPlatformTime::Seconds();
    // A displacement is applied exactly once, without DeltaSeconds, FOV,
    // encoder dimensions, legacy controller scales or mouse smoothing.
    ApplyFreeLook(Input.Look);
    RemoteCameraInput.Look = FVector2D::ZeroVector;
    return true;
}

void ABskCameraPawn::SetMainViewTransform(const FVector& Location, const FRotator& Rotation)
{
    const bool bLocationChanged = !MainViewLocation.Equals(Location, 0.01f);
    const bool bRotationChanged = !MainViewRotation.Equals(Rotation, 0.01f);
    MainViewLocation = Location;
    MainViewRotation = Rotation;
    MainTargetActor = nullptr;
    MainCameraMode = EBskCameraMode::MainView;
    if (!bFreeCameraActive)
    {
        TargetActor = nullptr;
        CameraMode = EBskCameraMode::MainView;
        if (bLocationChanged) SetActorLocation(MainViewLocation);
        if (bRotationChanged && Controller) Controller->SetControlRotation(MainViewRotation);
    }
}

void ABskCameraPawn::SetCameraTarget(AActor* Target, EBskCameraMode Mode)
{
    MainTargetActor = IsValid(Target) ? Target : nullptr;
    MainCameraMode = IsValid(Target) ? Mode : EBskCameraMode::MainView;
    if (bFreeCameraActive) return;
    TargetActor = MainTargetActor;
    CameraMode = MainCameraMode;
}

void ABskCameraPawn::ClearCameraTarget()
{
    MainTargetActor = nullptr;
    MainCameraMode = EBskCameraMode::MainView;
    if (bFreeCameraActive) return;
    TargetActor = nullptr;
    CameraMode = EBskCameraMode::MainView;
}

void ABskCameraPawn::SetOrbitDistanceMeters(double DistanceMeters)
{
    const double DistanceCentimeters = FMath::Clamp(DistanceMeters * 100.0, 100.0, 1.0e9);
    MainOrbitDistanceCentimeters = DistanceCentimeters;
    OrbitDistanceCentimeters = DistanceCentimeters;
}

void ABskCameraPawn::ToggleFreeCamera()
{
    if (bFreeCameraActive)
    {
        ReturnToMainView();
        return;
    }

    // Latch the current view once. Never read Euler ControlRotation back into
    // the free-flight attitude: PlayerCameraManager clamps pitch/roll there.
    FreeCameraOrientation = GetViewRotation().Quaternion().GetNormalized();
    bFreeCameraActive = true;
    TargetActor = nullptr;
    CameraMode = EBskCameraMode::Free;
    bUseControllerRotationPitch = false;
    bUseControllerRotationYaw = false;
    bUseControllerRotationRoll = false;
    Camera->bUsePawnControlRotation = false;
    Camera->SetRelativeRotation(FQuat::Identity);
    SetActorRotation(FreeCameraOrientation);
    UE_LOG(LogTemp, Display, TEXT("BSK camera entered free-flight mode (C toggles, Home restores main view)"));
}

void ABskCameraPawn::ReturnToMainView()
{
    bRemoteCameraControl = false;
    RemoteCameraInput = FBskCameraInput();
    bFreeCameraActive = false;
    bUseControllerRotationPitch = true;
    bUseControllerRotationYaw = true;
    bUseControllerRotationRoll = false;
    Camera->bUsePawnControlRotation = true;
    Camera->SetRelativeRotation(FQuat::Identity);
    TargetActor = MainTargetActor;
    CameraMode = MainCameraMode;
    OrbitDistanceCentimeters = MainOrbitDistanceCentimeters;
    OrbitYawDegrees = MainOrbitYawDegrees;
    OrbitPitchDegrees = MainOrbitPitchDegrees;
    FollowOffsetCentimeters = MainFollowOffsetCentimeters;

    bMoveForward = false;
    bMoveBackward = false;
    bMoveRight = false;
    bMoveLeft = false;
    bMoveUp = false;
    bMoveDown = false;
    MoveForwardAxisValue = 0.0f;
    MoveRightAxisValue = 0.0f;
    MoveUpAxisValue = 0.0f;
    if (GetMovementComponent()) GetMovementComponent()->StopMovementImmediately();
    if (CameraMode == EBskCameraMode::MainView)
    {
        SetActorLocationAndRotation(MainViewLocation, MainViewRotation);
        if (Controller) Controller->SetControlRotation(MainViewRotation);
    }
    else
    {
        UpdateTrackingCamera();
    }
    UE_LOG(LogTemp, Display, TEXT("BSK camera restored main view"));
}

void ABskCameraPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
    Super::SetupPlayerInputComponent(PlayerInputComponent);
    // Pixel Streaming delivers key down/up events reliably, while legacy axis
    // callbacks can be lost when the browser toggles keyboard input at runtime.
    // Track the held keys and apply the translation in Tick instead.
    PlayerInputComponent->BindKey(EKeys::W, IE_Pressed, this, &ABskCameraPawn::MoveForwardPressed);
    PlayerInputComponent->BindKey(EKeys::W, IE_Released, this, &ABskCameraPawn::MoveForwardReleased);
    PlayerInputComponent->BindKey(EKeys::S, IE_Pressed, this, &ABskCameraPawn::MoveBackwardPressed);
    PlayerInputComponent->BindKey(EKeys::S, IE_Released, this, &ABskCameraPawn::MoveBackwardReleased);
    PlayerInputComponent->BindKey(EKeys::D, IE_Pressed, this, &ABskCameraPawn::MoveRightPressed);
    PlayerInputComponent->BindKey(EKeys::D, IE_Released, this, &ABskCameraPawn::MoveRightReleased);
    PlayerInputComponent->BindKey(EKeys::A, IE_Pressed, this, &ABskCameraPawn::MoveLeftPressed);
    PlayerInputComponent->BindKey(EKeys::A, IE_Released, this, &ABskCameraPawn::MoveLeftReleased);
    PlayerInputComponent->BindKey(EKeys::E, IE_Pressed, this, &ABskCameraPawn::MoveUpPressed);
    PlayerInputComponent->BindKey(EKeys::E, IE_Released, this, &ABskCameraPawn::MoveUpReleased);
    PlayerInputComponent->BindKey(EKeys::Q, IE_Pressed, this, &ABskCameraPawn::MoveDownPressed);
    PlayerInputComponent->BindKey(EKeys::Q, IE_Released, this, &ABskCameraPawn::MoveDownReleased);
    // Keep the legacy axis path as a fallback.  Pixel Streaming and the
    // EnhancedInput component do not always deliver BindKey events uniformly
    // across browser focus changes, while axis mappings continue to emit the
    // held-key state.  The Tick() code uses whichever path is available.
    PlayerInputComponent->BindAxis(TEXT("MoveForward"), this, &ABskCameraPawn::MoveForwardAxis);
    PlayerInputComponent->BindAxis(TEXT("MoveRight"), this, &ABskCameraPawn::MoveRightAxis);
    PlayerInputComponent->BindAxis(TEXT("MoveUp"), this, &ABskCameraPawn::MoveUpAxis);
    PlayerInputComponent->BindAxis(TEXT("Turn"), this, &ABskCameraPawn::Turn);
    PlayerInputComponent->BindAxis(TEXT("LookUp"), this, &ABskCameraPawn::LookUp);
    PlayerInputComponent->BindAction(TEXT("ToggleFreeCamera"), IE_Pressed, this, &ABskCameraPawn::ToggleFreeCamera);
    PlayerInputComponent->BindAction(TEXT("ReturnMainView"), IE_Pressed, this, &ABskCameraPawn::ReturnToMainView);
    PlayerInputComponent->BindKey(EKeys::Escape, IE_Pressed, this, &ABskCameraPawn::ReturnToMainView);
    PlayerInputComponent->BindAction(TEXT("ToggleCssVisuals"), IE_Pressed, this, &ABskCameraPawn::ToggleCssVisuals);
    PlayerInputComponent->BindAction(TEXT("ToggleGenericSensorVisuals"), IE_Pressed, this, &ABskCameraPawn::ToggleGenericSensorVisuals);
    PlayerInputComponent->BindAction(TEXT("ToggleTransceiverVisuals"), IE_Pressed, this, &ABskCameraPawn::ToggleTransceiverVisuals);
    PlayerInputComponent->BindAction(TEXT("TogglePictureInPictureOne"), IE_Pressed, this, &ABskCameraPawn::TogglePictureInPictureOne);
    PlayerInputComponent->BindAction(TEXT("TogglePictureInPictureTwo"), IE_Pressed, this, &ABskCameraPawn::TogglePictureInPictureTwo);
    PlayerInputComponent->BindAction(TEXT("ToggleMissionUi"), IE_Pressed, this, &ABskCameraPawn::ToggleMissionUi);
    PlayerInputComponent->BindAction(TEXT("ToggleMissionUiVisibility"), IE_Pressed, this, &ABskCameraPawn::ToggleMissionUiVisibility);
}

void ABskCameraPawn::MoveForwardPressed() { bMoveForward = true; }
void ABskCameraPawn::MoveForwardReleased() { bMoveForward = false; }
void ABskCameraPawn::MoveForwardAxis(float Value) { MoveForwardAxisValue = FMath::Clamp(Value, -1.0f, 1.0f); }
void ABskCameraPawn::MoveBackwardPressed() { bMoveBackward = true; }
void ABskCameraPawn::MoveBackwardReleased() { bMoveBackward = false; }
void ABskCameraPawn::MoveRightPressed() { bMoveRight = true; }
void ABskCameraPawn::MoveRightReleased() { bMoveRight = false; }
void ABskCameraPawn::MoveRightAxis(float Value) { MoveRightAxisValue = FMath::Clamp(Value, -1.0f, 1.0f); }
void ABskCameraPawn::MoveLeftPressed() { bMoveLeft = true; }
void ABskCameraPawn::MoveLeftReleased() { bMoveLeft = false; }
void ABskCameraPawn::MoveUpPressed() { bMoveUp = true; }
void ABskCameraPawn::MoveUpReleased() { bMoveUp = false; }
void ABskCameraPawn::MoveUpAxis(float Value) { MoveUpAxisValue = FMath::Clamp(Value, -1.0f, 1.0f); }
void ABskCameraPawn::MoveDownPressed() { bMoveDown = true; }
void ABskCameraPawn::MoveDownReleased() { bMoveDown = false; }
void ABskCameraPawn::Turn(float Value)
{
    if (bRemoteCameraControl) return;
    if (CameraMode == EBskCameraMode::MainView) return;
    if (CameraMode == EBskCameraMode::Free) ApplyFreeLook(FVector2D(Value / FBskCameraInput::DegreesPerMouseUnit, 0.0));
    else if (CameraMode == EBskCameraMode::Orbit) OrbitYawDegrees += Value;
    else AddControllerYawInput(Value);
}
void ABskCameraPawn::LookUp(float Value)
{
    if (bRemoteCameraControl) return;
    if (CameraMode == EBskCameraMode::MainView) return;
    if (CameraMode == EBskCameraMode::Free) ApplyFreeLook(FVector2D(0.0, -Value / FBskCameraInput::DegreesPerMouseUnit));
    else if (CameraMode == EBskCameraMode::Orbit) OrbitPitchDegrees = FMath::Clamp(OrbitPitchDegrees + Value, -89.0, 89.0);
    else AddControllerPitchInput(Value);
}

void ABskCameraPawn::ToggleVisualKind(const FString& VisualKind)
{
    TArray<AActor*> Controllers;
    UGameplayStatics::GetAllActorsOfClass(GetWorld(), ABskSceneController::StaticClass(), Controllers);
    if (!Controllers.IsEmpty())
    {
        if (ABskSceneController* Scene = Cast<ABskSceneController>(Controllers[0])) Scene->ToggleVisualKindVisible(VisualKind);
    }
}

void ABskCameraPawn::ToggleCssVisuals() { ToggleVisualKind(TEXT("css")); }
void ABskCameraPawn::ToggleGenericSensorVisuals() { ToggleVisualKind(TEXT("generic_sensor")); }
void ABskCameraPawn::ToggleTransceiverVisuals() { ToggleVisualKind(TEXT("transceiver")); }

void ABskCameraPawn::TogglePictureInPicture(int32 Slot)
{
    TArray<AActor*> Controllers;
    UGameplayStatics::GetAllActorsOfClass(GetWorld(), ABskSceneController::StaticClass(), Controllers);
    if (!Controllers.IsEmpty())
    {
        if (ABskSceneController* Scene = Cast<ABskSceneController>(Controllers[0])) Scene->TogglePictureInPictureSlot(Slot);
    }
}

void ABskCameraPawn::TogglePictureInPictureOne() { TogglePictureInPicture(1); }
void ABskCameraPawn::TogglePictureInPictureTwo() { TogglePictureInPicture(2); }

void ABskCameraPawn::ToggleMissionUi()
{
    if (APlayerController* Player = Cast<APlayerController>(Controller))
    {
        if (ABskRendererHUD* Hud = Cast<ABskRendererHUD>(Player->GetHUD())) Hud->ToggleInteractiveMissionUi();
    }
}

void ABskCameraPawn::ToggleMissionUiVisibility()
{
    if (APlayerController* Player = Cast<APlayerController>(Controller))
    {
        if (ABskRendererHUD* Hud = Cast<ABskRendererHUD>(Player->GetHUD())) Hud->ToggleMissionUiVisibility();
    }
}
