#include "BskCameraInput.h"

#include "Dom/JsonObject.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

bool FBskCameraInput::Parse(const FString& Json, FBskCameraInput& Out)
{
    if (Json.Len() > 2048) return false;
    TSharedPtr<FJsonObject> Object;
    if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Json), Object) || !Object) return false;
    const auto Number = [&Object](const TCHAR* Name, double Limit, double& Value)
    {
        return Object->HasTypedField<EJson::Number>(Name) && Object->TryGetNumberField(Name, Value)
            && FMath::IsFinite(Value) && FMath::Abs(Value) <= Limit;
    };
    double Version = 0.0;
    FBskCameraInput Input;
    if (!Number(TEXT("version"), 1.0, Version) || Version != 1.0
        || !Object->HasTypedField<EJson::Boolean>(TEXT("active"))
        || !Object->TryGetBoolField(TEXT("active"), Input.bActive)
        || !Object->HasTypedField<EJson::Boolean>(TEXT("boost"))
        || !Object->TryGetBoolField(TEXT("boost"), Input.bBoost)
        || !Number(TEXT("forward"), 1.0, Input.Axes.X)
        || !Number(TEXT("right"), 1.0, Input.Axes.Y)
        || !Number(TEXT("up"), 1.0, Input.Axes.Z)
        || !Number(TEXT("look_dx"), 4096.0, Input.Look.X)
        || !Number(TEXT("look_dy"), 4096.0, Input.Look.Y)) return false;
    Out = Input;
    return true;
}

FQuat FBskCameraInput::ApplyLook(const FQuat& Rotation) const
{
    if (!bActive || Look.IsNearlyZero()) return Rotation;
    // Integrate in camera-local axes, not Euler pitch/yaw. Browser +Y looks
    // down (positive quaternion rotation about local +Y); +X turns right
    // about local +Z. One rotation vector keeps collinear mouse batches
    // equivalent and remains continuous through the poles and full turns.
    const FVector LocalRotationVector(0.0, Look.Y, Look.X);
    const double Magnitude = LocalRotationVector.Size();
    const FQuat Delta(LocalRotationVector / Magnitude,
        FMath::DegreesToRadians(Magnitude * DegreesPerMouseUnit));
    return (Rotation * Delta).GetNormalized();
}

FVector FBskCameraInput::MovementDelta(const FQuat& Rotation, double DeltaSeconds) const
{
    if (!bActive || !FMath::IsFinite(DeltaSeconds) || DeltaSeconds <= 0.0) return FVector::ZeroVector;
    const FVector Direction = Rotation.GetAxisX() * Axes.X
        + Rotation.GetAxisY() * Axes.Y + FVector::UpVector * Axes.Z;
    return Direction.GetClampedToMaxSize(1.0) * SpeedCentimetersPerSecond * (bBoost ? 5.0 : 1.0) * DeltaSeconds;
}
