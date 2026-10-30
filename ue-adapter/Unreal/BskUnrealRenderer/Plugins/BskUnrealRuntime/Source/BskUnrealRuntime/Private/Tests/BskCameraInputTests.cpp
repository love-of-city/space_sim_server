#if WITH_DEV_AUTOMATION_TESTS
#include "BskCameraInput.h"
#include "BskCameraPawn.h"
#include "Camera/CameraComponent.h"
#include "Camera/PlayerCameraManager.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "HAL/PlatformProcess.h"
#include "Misc/AutomationTest.h"

namespace
{
FString CameraPacket(bool bActive, double Forward = 0.0, double Right = 0.0, double Up = 0.0,
    double Dx = 0.0, double Dy = 0.0, bool bBoost = false)
{
    return FString::Printf(TEXT("{\"version\":1,\"active\":%s,\"forward\":%.6f,\"right\":%.6f,\"up\":%.6f,\"look_dx\":%.6f,\"look_dy\":%.6f,\"boost\":%s}"),
        bActive ? TEXT("true") : TEXT("false"), Forward, Right, Up, Dx, Dy, bBoost ? TEXT("true") : TEXT("false"));
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBskCameraInputValidationTest,
    "BskUnreal.Camera.StrictRemoteInput", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FBskCameraInputValidationTest::RunTest(const FString& Parameters)
{
    FBskCameraInput Input;
    TestTrue(TEXT("valid complete state"), FBskCameraInput::Parse(CameraPacket(true, 1, -1, 1, 30, -20), Input));
    TestTrue(TEXT("correct axes"), Input.Axes.Equals(FVector(1, -1, 1)));
    for (const FString& Json : {
        FString(TEXT("{}")), FString(TEXT("garbage")),
        CameraPacket(true).Replace(TEXT("\"version\":1"), TEXT("\"version\":2")),
        CameraPacket(true).Replace(TEXT("\"active\":true"), TEXT("\"active\":\"true\"")),
        CameraPacket(true).Replace(TEXT("\"forward\":0.000000"), TEXT("\"forward\":\"1\"")),
        CameraPacket(true).Replace(TEXT("\"forward\":0.000000"), TEXT("\"forward\":null")),
        CameraPacket(true).Replace(TEXT("\"forward\":0.000000"), TEXT("\"forward\":1e999")),
        CameraPacket(true, 2), CameraPacket(true, 0, 0, 0, 4097) })
    {
        TestFalse(TEXT("invalid input rejected atomically"), FBskCameraInput::Parse(Json, Input));
        TestTrue(TEXT("previous state preserved"), Input.Axes.Equals(FVector(1, -1, 1)));
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBskCameraRateInvariantTest,
    "BskUnreal.Camera.RateIndependentMotion", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FBskCameraRateInvariantTest::RunTest(const FString& Parameters)
{
    FBskCameraInput Input;
    Input.bActive = true;
    Input.Look = FVector2D(100, 0);
    TestTrue(TEXT("fixed yaw gain and sign"), Input.ApplyLook(FQuat::Identity).Equals(FRotator(0, 12, 0).Quaternion(), 1e-6));
    Input.Look = FVector2D(0, -50);
    TestTrue(TEXT("fixed pitch gain and sign"), Input.ApplyLook(FQuat::Identity).Equals(FRotator(6, 0, 0).Quaternion(), 1e-6));
    Input.Look = FVector2D(100, -50);
    const FQuat Initial = FRotator(120, 35, 170).Quaternion();
    const FQuat Single = Input.ApplyLook(Initial);
    Input.Look /= 10;
    FQuat Batched = Initial;
    for (int I = 0; I < 10; ++I) Batched = Input.ApplyLook(Batched);
    TestTrue(TEXT("collinear event batching does not change sensitivity"), Batched.Equals(Single, 1e-6));
    Input.Axes = FVector(1, 1, 0);
    for (const int Rate : { 15, 30, 60, 144 })
    {
        FVector Displacement = FVector::ZeroVector;
        for (int I = 0; I < Rate; ++I) Displacement += Input.MovementDelta(FQuat::Identity, 1.0 / Rate);
        TestTrue(TEXT("one metre per second, diagonals normalized at all frame rates"), FMath::IsNearlyEqual(Displacement.Size(), 100.0, 1e-6));
    }
    Input.Axes = FVector(1, 0, 0);
    const FVector ForwardDelta = Input.MovementDelta(FRotator(0, 90, 0).Quaternion(), 1);
    TestTrue(*FString::Printf(TEXT("W follows camera yaw: %s"), *ForwardDelta.ToString()), ForwardDelta.Equals(FVector(0, 100, 0), 1e-4));
    const FQuat Inverted = FRotator(180, 0, 0).Quaternion();
    TestTrue(TEXT("W follows the inverted view"), Input.MovementDelta(Inverted, 1).Equals(-FVector::ForwardVector * 100, 1e-4));
    Input.Axes = FVector(0, 1, 0);
    TestTrue(TEXT("D follows screen right even after a flip"), Input.MovementDelta(Initial, 1).Equals(Initial.GetAxisY() * 100, 1e-4));
    Input.Axes = FVector(0, 0, 1);
    TestTrue(TEXT("E retains world up after a flip"), Input.MovementDelta(Inverted, 1).Equals(FVector(0, 0, 100), 1e-6));
    Input.bBoost = true;
    TestTrue(TEXT("Shift boost is five times"), FMath::IsNearlyEqual(Input.MovementDelta(FQuat::Identity, 1).Size(), 500.0, 1e-6));
    Input.bActive = false;
    TestTrue(TEXT("inactive input cannot move"), Input.MovementDelta(FQuat::Identity, 1).IsZero());
    TestTrue(TEXT("inactive input cannot rotate"), Input.ApplyLook(Initial).Equals(Initial));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBskCameraUnboundedRotationTest,
    "BskUnreal.Camera.UnboundedRotation", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FBskCameraUnboundedRotationTest::RunTest(const FString& Parameters)
{
    FBskCameraInput Input;
    Input.bActive = true;
    for (const int Direction : { -1, 1 })
    {
        for (const bool bPitch : { false, true })
        {
            Input.Look = bPitch ? FVector2D(0, -Direction * 5.0 / Input.DegreesPerMouseUnit)
                : FVector2D(Direction * 5.0 / Input.DegreesPerMouseUnit, 0);
            FQuat Rotation = FQuat::Identity;
            for (int Step = 1; Step <= 144; ++Step)
            {
                Rotation = Input.ApplyLook(Rotation);
                const double Angle = FMath::DegreesToRadians(Direction * Step * 5.0);
                const FQuat Expected(bPitch ? FVector::RightVector : FVector::UpVector, bPitch ? -Angle : Angle);
                TestTrue(TEXT("continuous through +/-90, +/-180 and two full turns"), Rotation.Equals(Expected, 1e-6));
            }
            TestTrue(TEXT("two turns return to starting orientation"), Rotation.Equals(FQuat::Identity, 1e-6));
        }
    }
    // At a pole, horizontal motion still turns toward screen-right instead of
    // freezing or switching back to a global yaw axis.
    const FQuat Pole(FVector::RightVector, -UE_DOUBLE_PI / 2.0);
    Input.Look = FVector2D(10.0 / Input.DegreesPerMouseUnit, 0);
    const FQuat Turned = Input.ApplyLook(Pole);
    TestTrue(TEXT("horizontal look remains active at the pole"), !Turned.Equals(Pole, 1e-3));
    TestTrue(TEXT("horizontal turn goes toward camera right at the pole"),
        FVector::DotProduct(Turned.GetAxisX(), Pole.GetAxisY()) > 0.1);
    FQuat Rotation = Pole;
    for (int I = 0; I < 10000; ++I)
    {
        Input.Look = FVector2D(3.0, I % 2 ? 2.0 : -1.0);
        Rotation = Input.ApplyLook(Rotation);
    }
    TestTrue(TEXT("long running free look stays finite and normalized"), !Rotation.ContainsNaN() && Rotation.IsNormalized());
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBskCameraPawnRemoteTest,
    "BskUnreal.Camera.PawnModeMovementAndWatchdog", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FBskCameraPawnRemoteTest::RunTest(const FString& Parameters)
{
    const UWorld::InitializationValues Init = UWorld::InitializationValues().AllowAudioPlayback(false)
        .CreatePhysicsScene(false).CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false);
    UWorld* World = UWorld::CreateWorld(EWorldType::Game, false, NAME_None, nullptr, true, ERHIFeatureLevel::Num, &Init);
    if (!TestNotNull(TEXT("test world"), World)) return false;
    ABskCameraPawn* Pawn = World->SpawnActor<ABskCameraPawn>();
    APlayerController* Player = World->SpawnActor<APlayerController>();
    if (!Pawn || !Player)
    {
        AddError(TEXT("Could not spawn camera and controller"));
        World->DestroyWorld(false);
        return false;
    }
    Player->Possess(Pawn);
    Pawn->SetMainViewTransform(FVector(10, 20, 30), FRotator::ZeroRotator);
    TestTrue(TEXT("enter without a native C key"), Pawn->ApplyRemoteCameraInput(CameraPacket(true, 1)));
    TestTrue(TEXT("free mode active"), Pawn->IsFreeCameraActive());
    Pawn->Tick(0.1f);
    TestTrue(TEXT("actual pawn translates with explicit W state"), Pawn->GetActorLocation().Equals(FVector(20, 20, 30), 1e-4));
    Pawn->SetMainViewTransform(FVector(100, 200, 300), FRotator::ZeroRotator);
    TestTrue(TEXT("streamed main pose does not overwrite free position"), Pawn->GetActorLocation().Equals(FVector(20, 20, 30), 1e-4));
    Pawn->ApplyRemoteCameraInput(CameraPacket(true, 0, 0, 0, 100, -50));
    const FQuat Rotation = Pawn->GetActorQuat();
    TestFalse(TEXT("free camera bypasses control rotation"), Pawn->GetBskCameraComponent()->bUsePawnControlRotation);
    TestFalse(TEXT("mouse rotates the pawn"), Rotation.Equals(FQuat::Identity));
    Pawn->GetBskCameraComponent()->FieldOfView = 110;
    Pawn->Tick(0.1f);
    Pawn->Tick(1.f / 144.f);
    TestTrue(TEXT("look is not replayed by Tick or rescaled by FOV"), Pawn->GetActorQuat().Equals(Rotation, 1e-6));
    // Exercise actual controller processing and the camera view, not just the
    // math helper: stock PlayerCameraManager still clamps ControlRotation.
    // This isolated test world has no StartPlay/PostInitializeComponents pass.
    if (!Player->PlayerCameraManager) Player->SpawnPlayerCameraManager();
    Player->SetViewTarget(Pawn);
    APlayerCameraManager* Manager = Player->PlayerCameraManager;
    if (TestNotNull(TEXT("player camera manager"), Manager))
    {
        const float PitchMin = Manager->ViewPitchMin, PitchMax = Manager->ViewPitchMax;
        const float RollMin = Manager->ViewRollMin, RollMax = Manager->ViewRollMax;
        Manager->bUseClientSideCameraUpdates = false;
        Manager->ViewPitchMin = -5; Manager->ViewPitchMax = 5;
        Manager->ViewRollMin = -5; Manager->ViewRollMax = 5;
        FQuat Expected = Rotation;
        const FQuat PitchStep(FVector::RightVector, -FMath::DegreesToRadians(10.0));
        for (int Step = 0; Step < 72; ++Step)
        {
            Pawn->ApplyRemoteCameraInput(CameraPacket(true, 0, 0, 0, 0, -10.0 / FBskCameraInput::DegreesPerMouseUnit));
            Expected = (Expected * PitchStep).GetNormalized();
            Player->SetControlRotation(FRotator(120, 45, 130));
            Player->UpdateRotation(1.f / 60.f);
            Pawn->Tick(1.f / 60.f);
            FMinimalViewInfo View;
            Pawn->GetBskCameraComponent()->GetCameraView(1.f / 60.f, View);
            Manager->UpdateCamera(1.f / 60.f);
            TestTrue(TEXT("final camera manager POV preserves full orientation"), Manager->GetCameraRotation().Quaternion().Equals(Expected, 1e-5));
            TestTrue(TEXT("rendered view crosses both poles despite engine pitch/roll limits"), View.Rotation.Quaternion().Equals(Expected, 1e-5));
            TestTrue(TEXT("view rotation query follows free orientation"), Pawn->GetViewRotation().Quaternion().Equals(Expected, 1e-5));
        }
        Manager->ViewPitchMin = PitchMin; Manager->ViewPitchMax = PitchMax;
        Manager->ViewRollMin = RollMin; Manager->ViewRollMax = RollMax;
    }
    const FVector Stopped = Pawn->GetActorLocation();
    Pawn->Tick(0.1f);
    TestTrue(TEXT("released movement stays stopped"), Pawn->GetActorLocation().Equals(Stopped));
    Pawn->ApplyRemoteCameraInput(CameraPacket(true, 1));
    FPlatformProcess::Sleep(static_cast<float>(FBskCameraInput::WatchdogSeconds + 0.05));
    Pawn->Tick(0.1f);
    TestFalse(TEXT("lost keepalive exits free mode"), Pawn->IsFreeCameraActive());
    TestTrue(TEXT("main camera control rotation restored"), Pawn->GetBskCameraComponent()->bUsePawnControlRotation);
    TestTrue(TEXT("main pose clears free-flight roll"), Pawn->GetActorQuat().Equals(FQuat::Identity, 1e-6));
    TestTrue(TEXT("lost keepalive returns to main instead of drifting"), Pawn->GetActorLocation().Equals(FVector(100, 200, 300)));
    Pawn->ApplyRemoteCameraInput(CameraPacket(false));
    TestFalse(TEXT("explicit exit resets mode"), Pawn->IsFreeCameraActive());
    TestTrue(TEXT("returns to latest main view"), Pawn->GetActorLocation().Equals(FVector(100, 200, 300)));
    Pawn->ApplyRemoteCameraInput(CameraPacket(false));
    TestFalse(TEXT("duplicate exit never toggles back to free"), Pawn->IsFreeCameraActive());
    // SARM normally uses an orbit target as its main view. Returning from an
    // inverted free view must restore that position/orientation immediately.
    ABskCameraPawn* Target = World->SpawnActor<ABskCameraPawn>();
    if (TestNotNull(TEXT("orbit target"), Target))
    {
        Target->SetActorLocation(FVector(1000, 100, 200));
        Pawn->SetOrbitDistanceMeters(2.8);
        Pawn->SetCameraTarget(Target, EBskCameraMode::Orbit);
        Pawn->Tick(1.f / 60.f);
        const FVector OrbitLocation = Pawn->GetActorLocation();
        const FQuat OrbitOrientation = Pawn->GetActorQuat();
        Pawn->ApplyRemoteCameraInput(CameraPacket(true));
        TestTrue(TEXT("entering free mode preserves the current orbit view"), Pawn->GetActorQuat().Equals(OrbitOrientation, 1e-6));
        Pawn->ApplyRemoteCameraInput(CameraPacket(true, 1, 0, 0, 800, -1500));
        Pawn->Tick(0.1f);
        TestFalse(TEXT("free view moves away from orbit"), Pawn->GetActorLocation().Equals(OrbitLocation));
        Pawn->ApplyRemoteCameraInput(CameraPacket(false));
        TestFalse(TEXT("explicit exit ends free mode"), Pawn->IsFreeCameraActive());
        TestTrue(TEXT("orbit location restored immediately"), Pawn->GetActorLocation().Equals(OrbitLocation, 1e-4));
        TestTrue(TEXT("orbit orientation restored without leftover roll"), Pawn->GetActorQuat().Equals(OrbitOrientation, 1e-6));
    }
    World->DestroyWorld(false);
    return true;
}
#endif
