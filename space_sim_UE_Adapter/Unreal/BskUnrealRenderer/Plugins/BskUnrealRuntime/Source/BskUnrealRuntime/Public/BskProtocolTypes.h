#pragma once

#include "CoreMinimal.h"

enum class EBskRenderMessageType : uint8
{
    Unknown,
    Hello,
    SceneManifest,
    Frame,
    Event
};

struct BSKUNREALRUNTIME_API FBskGeometryDefinition
{
    FString GeometryId;
    FString Shape;
    FVector3d DimensionsMeters = FVector3d::OneVector;
    FVector3d PositionBodyMeters = FVector3d::ZeroVector;
    FQuat4d OrientationBodyFromGeometryWxyz = FQuat4d::Identity;
    FLinearColor Color = FLinearColor::White;
    FString AssetType;
    FString AssetPath;
    FVector3d Scale = FVector3d::OneVector;
    FString RenderRole = TEXT("visual");
    FString AssetKey;
    FString MaterialName;
    double MaterialSpecular = 0.5;
    double MaterialShininess = 0.25;
    double MaterialReflectance = 0.0;
    double MaterialEmission = 0.0;
    FString MaterialTexture;
    FString MaterialTextureAssetPath;
    FVector2d MaterialTextureRepeat = FVector2d(1.0, 1.0);
    bool bUseAssetMaterials = false;
};

struct BSKUNREALRUNTIME_API FBskObjectDefinition
{
    FString ObjectId;
    FString DisplayName;
    FString ParentId;
    FString TransformSpace = TEXT("world");
    FString AssetPath;
    FString SemanticLabel;
    TArray<FBskGeometryDefinition> Geometries;
};

struct BSKUNREALRUNTIME_API FBskCelestialBodyDefinition
{
    FString BodyId;
    FString DisplayName;
    double MuMetersCubedPerSecondSquared = 0.0;
    double EquatorialRadiusMeters = 1.0;
    double PolarRadiusRatio = 1.0;
    FString AssetPath;
    FString VisualRole = TEXT("body");
    bool bLuminous = false;
    bool bDrivesDirectionalLight = false;
    FVector3d LightColorRgb = FVector3d(1.0, 0.98, 0.92);
    double LightIlluminanceLuxAtReferenceDistance = 0.0;
    double LightReferenceDistanceMeters = 149597870693.0;
};

struct BSKUNREALRUNTIME_API FBskChannelDefinition
{
    FString Type = TEXT("number");
    FString Unit;
    double Minimum = 0.0;
    double Maximum = 0.0;
    bool bHasMinimum = false;
    bool bHasMaximum = false;
};

struct BSKUNREALRUNTIME_API FBskVisualDefinition
{
    FString VisualId;
    FString Kind;
    FString ParentId;
    FVector3d PositionBodyMeters = FVector3d::ZeroVector;
    FQuat4d OrientationBodyFromVisualWxyz = FQuat4d::Identity;
    FVector3d NormalBody = FVector3d::UnitX();
    FVector2d FieldOfViewRadians = FVector2d::ZeroVector;
    double SizeMeters = 1.0;
    double RangeMeters = 0.0;
    FLinearColor Color = FLinearColor::White;
    FString Label;
    FString LightType;
    FString LightTargetId;
    FVector3d LightDiffuseRgb = FVector3d(0.7, 0.7, 0.7);
    FVector3d LightSpecularRgb = FVector3d(0.3, 0.3, 0.3);
    double LightIntensity = 1.0;
    double LightCutoffDegrees = 45.0;
    bool bLightCastShadows = true;
    TMap<FString, FBskChannelDefinition> ChannelSchema;
};

struct BSKUNREALRUNTIME_API FBskCameraDefinition
{
    FString CameraId;
    FString DisplayName;
    FString ParentId;
    FVector3d PositionBodyMeters = FVector3d::ZeroVector;
    FQuat4d OrientationBodyFromCameraWxyz = FQuat4d::Identity;
    double FieldOfViewRadians = PI / 3.0;
    FIntPoint Resolution = FIntPoint(1920, 1080);
    FString SemanticLabel;
    bool bPictureInPicture = false;
    double CaptureRateHertz = 15.0;
    int32 PictureInPictureSlot = 0;
    TArray<FString> CaptureProducts;
};

struct BSKUNREALRUNTIME_API FBskUiCommandDefinition
{
    FString Command;
    FString Label;
    FString TargetId;
    FString PayloadJson = TEXT("{}");
    bool bRequiresConfirmation = false;
};

struct BSKUNREALRUNTIME_API FBskSceneManifest
{
    FString Protocol;
    FString SessionId;
    int64 Revision = 0;
    FString OriginObjectId;
    FString DefaultCameraTarget;
    bool bUseSceneLighting = false;
    bool bHeadlightEnabled = false;
    FVector3d HeadlightDiffuseRgb = FVector3d(0.6, 0.6, 0.6);
    FVector3d HeadlightAmbientRgb = FVector3d(0.1, 0.1, 0.1);
    FVector3d HeadlightSpecularRgb = FVector3d::ZeroVector;
    double FillLightIntensityLux = -1.0;
    double SunlightIntensityScale = 1.0; // [-], scene-local multiplier; zero disables direct sunlight
    double InterpolationDelayMilliseconds = 100.0;
    double MaxExtrapolationMilliseconds = 100.0;
    double DefaultCameraDistanceMeters = 25.0;
    bool bOrbitLines = true;
    bool bTrajectoryHistory = true;
    TArray<FBskObjectDefinition> Objects;
    TArray<FBskCelestialBodyDefinition> CelestialBodies;
    TArray<FBskVisualDefinition> Visuals;
    TArray<FBskCameraDefinition> Cameras;
    TArray<FBskUiCommandDefinition> UiCommands;
};

struct BSKUNREALRUNTIME_API FBskRenderObjectState
{
    FString ObjectId;
    FString Name;
    FString Parent;
    FString AssetPath;
    FString SemanticLabel;
    FVector3d PositionMeters = FVector3d::ZeroVector;
    FQuat4d OrientationWxyz = FQuat4d::Identity;
    FVector3d VelocityMetersPerSecond = FVector3d::ZeroVector;
    FVector3d AngularVelocityBodyRadiansPerSecond = FVector3d::ZeroVector;
    bool bHasVelocity = false;
    bool bHasAngularVelocity = false;
};

struct BSKUNREALRUNTIME_API FBskCelestialBodyState
{
    FString BodyId;
    FVector3d PositionMeters = FVector3d::ZeroVector;
    FQuat4d OrientationWxyz = FQuat4d::Identity;
    FVector3d VelocityMetersPerSecond = FVector3d::ZeroVector;
};

struct BSKUNREALRUNTIME_API FBskVisualState
{
    struct FChannelValue
    {
        enum class EType : uint8
        {
            Number,
            Boolean,
            String
        };

        EType Type = EType::Number;
        double Number = 0.0;
        bool Boolean = false;
        FString String;
    };

    FString VisualId;
    double Value = 0.0;
    int32 Status = 0;
    bool bVisible = true;
    TMap<FString, FChannelValue> Channels;
};

struct BSKUNREALRUNTIME_API FBskRenderFrame
{
    FString Protocol;
    FString SessionId;
    int64 ManifestRevision = 0;
    int64 FrameId = 0;
    // Presence distinguishes on-demand peers from legacy always-capture peers.
    bool bCaptureOnDemand = false;
    FString CaptureEpisodeId;
    int64 SimulationTimeNanoseconds = 0;
    int64 WallTimeNanoseconds = 0;
    FVector3d OriginInertialMeters = FVector3d::ZeroVector;
    FMatrix44d LocalFromInertial = FMatrix44d::Identity;
    TArray<FBskRenderObjectState> Objects;
    TArray<FBskCelestialBodyState> CelestialBodies;
    TArray<FBskVisualState> VisualStates;
};

struct BSKUNREALRUNTIME_API FBskRenderEvent
{
    FString Protocol;
    FString SessionId;
    int64 Sequence = 0;
    FString EventKind;
    FString PayloadJson;
};

struct BSKUNREALRUNTIME_API FBskRenderMessage
{
    EBskRenderMessageType Type = EBskRenderMessageType::Unknown;
    FString Protocol;
    FString SessionId;
    TArray<FString> Capabilities;
    TArray<FString> RequiredCapabilities;
    FBskSceneManifest Manifest;
    FBskRenderFrame Frame;
    FBskRenderEvent Event;
};

namespace BskProtocol
{
    inline constexpr TCHAR GenericV1[] = TEXT("bsk-render/1");
    inline constexpr TCHAR LegacyIsaacV1[] = TEXT("bsk-isaac-render/1");
    inline constexpr TCHAR GenericV2[] = TEXT("bsk-render/2");
    inline constexpr uint32 DefaultMaxPacketBytes = 32u * 1024u * 1024u;

    inline bool IsSupported(const FString& Value)
    {
        return Value == GenericV2 || Value == GenericV1 || Value == LegacyIsaacV1;
    }
}
