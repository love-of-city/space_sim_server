#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "BskCoordinateConverter.h"
#include "BskFrameSource.h"
#include "BskProtocolTypes.h"
#include "BskSceneController.generated.h"

class ADirectionalLight;
class APostProcessVolume;
class ASpotLight;
class ASkyAtmosphere;
class UParticleSystemComponent;
class USceneComponent;
class USceneCaptureComponent2D;
class UStaticMeshComponent;
class UTextureRenderTarget2D;
class IPixelStreaming2Streamer;
class IPixelStreaming2VideoProducer;
class IBskCaptureProvider;
class FBskCaptureNetworkSender;
class FBskCaptureDiskWriter;
class FBskBuiltinCaptureProvider;
struct FBskCaptureRequest;
struct FBskAsyncCaptureJob;
struct FCapturedDataProduct;

struct BSKUNREALRUNTIME_API FBskPictureInPictureView
{
    FString CameraId;
    FString DisplayName;
    int32 Slot = 0;
    bool bVisible = true;
    TObjectPtr<UTextureRenderTarget2D> Texture = nullptr;
};

struct BSKUNREALRUNTIME_API FBskMissionEventView
{
    int64 Sequence = 0;
    FString Kind;
    FString Severity = TEXT("info");
    FString Message;
    int64 SimulationTimeNanoseconds = 0;
};

DECLARE_MULTICAST_DELEGATE_OneParam(FOnBskFrameApplied, const FBskRenderFrame&);

UCLASS()
class BSKUNREALRUNTIME_API ABskSceneController : public AActor
{
    GENERATED_BODY()

public:
    ABskSceneController();
    virtual ~ABskSceneController() override;
    virtual void Tick(float DeltaSeconds) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    UFUNCTION(BlueprintCallable, Category="BSK Renderer")
    FString GetReceiverStatus() const;

    UFUNCTION(BlueprintCallable, Category="BSK Renderer")
    int64 GetLastFrameId() const { return LastFrameId; }

    UFUNCTION(BlueprintCallable, Category="BSK Renderer|Replay")
    bool SetReplayPaused(bool bPaused);

    UFUNCTION(BlueprintCallable, Category="BSK Renderer|Replay")
    bool SetReplayRate(double Rate);

    UFUNCTION(BlueprintCallable, Category="BSK Renderer|Replay")
    bool StepReplay();

    UFUNCTION(BlueprintCallable, Category="BSK Renderer|Replay")
    bool SeekReplayNanoseconds(int64 SimulationTimeNanoseconds);

    UFUNCTION(BlueprintCallable, Category="BSK Renderer|Camera")
    bool FocusObject(const FString& ObjectId, bool bFollow = false);

    UFUNCTION(BlueprintCallable, Category="BSK Renderer|Camera")
    bool TogglePictureInPictureSlot(int32 Slot);

    void GetPictureInPictureViews(TArray<FBskPictureInPictureView>& OutViews) const;
    void GetMissionEvents(TArray<FBskMissionEventView>& OutEvents) const;
    void GetUiCommands(TArray<FBskUiCommandDefinition>& OutCommands) const;
    int32 GetObjectCount() const { return BoundActors.Num(); }
    int32 GetVisualCount() const { return VisualActors.Num(); }
    int32 GetCameraCount() const { return CameraActors.Num(); }
    int32 GetPendingCommandCount() const { return PendingCommandIds.Num(); }
    FString GetLastCommandStatus() const { return LastCommandStatus; }

    UFUNCTION(BlueprintCallable, Category="BSK Renderer|Commands")
    bool SendUiCommand(const FString& Command, const FString& TargetId, const FString& PayloadJson, FString& OutError);

    UFUNCTION(BlueprintCallable, Category="BSK Renderer|Commands")
    void ClearMissionEvents();

    /** Control renderer-only helpers such as sensor FOV and antenna cones. */
    UFUNCTION(BlueprintCallable, Category="BSK Renderer|Visual Helpers")
    void SetVisualKindVisible(const FString& VisualKind, bool bVisible);

    UFUNCTION(BlueprintCallable, Category="BSK Renderer|Visual Helpers")
    bool ToggleVisualKindVisible(const FString& VisualKind);

    UFUNCTION(BlueprintPure, Category="BSK Renderer|Visual Helpers")
    bool IsVisualKindVisible(const FString& VisualKind) const;

    FVector GetConfiguredCameraPositionCentimeters() const;
    FVector GetConfiguredCameraLookAtCentimeters() const;

    /** Hook for future recorders, sensor capture, multi-camera, and time synchronization. */
    FOnBskFrameApplied& OnFrameApplied() { return FrameAppliedEvent; }

protected:
    virtual void BeginPlay() override;

private:
    friend class FBskBuiltinCaptureProvider;
    friend class FBskLocalFrameExtrapolationTest;
    friend class FBskSceneResetTest;
    friend class FBskSceneSunlightIntensityTest;
    struct FObjectSpec
    {
        FString AssetType = TEXT("placeholder");
        FString AssetPath;
        FString ActorClass;
        FString PlaceholderShape = TEXT("cube");
        FVector3d SizeMeters = FVector3d(1.5, 1.0, 0.8);
        FVector3d Scale = FVector3d::OneVector;
        FLinearColor Color = FLinearColor(0.25f, 0.55f, 1.0f);
    };

    bool LoadConfiguration();
    void ApplyManifest(const FBskSceneManifest& Manifest);
    void ApplyEvent(const FBskRenderEvent& Event);
    void ResetPresentationState();
    void ApplyFrame(const FBskRenderFrame& Frame, bool bPresentationFrame = true);
    FBskRenderFrame InterpolateFrame(const FBskRenderFrame& From, const FBskRenderFrame& To, double Alpha) const;
    FBskRenderFrame ExtrapolateFrame(const FBskRenderFrame& From, const FBskRenderFrame& To, double SecondsBeyondTarget) const;
    AActor* GetOrCreateActor(const FBskRenderObjectState& State);
    AActor* FindBoundActor(const FString& ObjectName) const;
    AActor* SpawnFromSpec(const FString& ObjectName, const FObjectSpec& Spec);
    AActor* SpawnPlaceholder(const FString& ObjectName, const FObjectSpec& Spec);
    AActor* SpawnManifestObject(const FBskObjectDefinition& Definition);
    AActor* SpawnCelestialBody(const FBskCelestialBodyDefinition& Definition);
    AActor* SpawnTexturedEarth(const FString& ActorName, double RadiusMeters);
    bool CreateTexturedStarSphere();
    void CreateDecorativeSun();
    void CreateDecorativeSunEffects(USceneComponent* Root);
    void UpdateDecorativeSunScale();
    void UpdateDecorativeSunPlacement();
    void CreateDecorativeEarth();
    AActor* SpawnVisual(const FBskVisualDefinition& Definition);
    AActor* SpawnCamera(const FBskCameraDefinition& Definition);
    void ConfigureCamera(AActor* Actor, const FBskCameraDefinition& Definition);
    void ConfigureFixedRgbExposure(USceneCaptureComponent2D* Capture) const;
    void ConfigureRgbRenderTarget(UTextureRenderTarget2D* Target) const;
    void UpdatePictureInPictureCaptures();
    void ConfigurePixelStreamingOutput();
    void ConfigurePixelStreamingCamera(AActor* Actor, const FBskCameraDefinition& Definition);
    void UpdatePixelStreamingCameraCaptures();
    void ShutdownPixelStreamingCameras();
    bool IsPixelStreamingCameraRequested(const FString& CameraId) const;
    bool HasPixelStreamingCameraViewers(const FString& CameraId) const;
    bool HasPixelStreamingViewportViewers() const;
    void UpdatePreviewViewportRendering();
    FString PixelStreamingIdForCamera(const FString& CameraId) const;
    void UpdateAuthoritativeDataProductCaptures(const FBskRenderFrame& AuthoritativeFrame);
    bool CaptureCameraDataProducts(const FBskCaptureRequest& Request, FString& OutError);
    bool WriteCaptureProducts(const FBskCaptureRequest& Request, const TSharedPtr<FJsonObject>& Metadata,
                              const TArray<FCapturedDataProduct>& Products, int64 Sequence, FString& OutError);
    void PollCaptureJobs();
    void CancelCaptureJobs();
    void ReportCaptureFailure(const FBskCaptureRequest& Request, const FString& Error);
    void ConfigureCaptureOutput();
    void ApplyVisualMountTransform(AActor* Actor, const FBskVisualDefinition& Definition) const;
    void RefreshVisualVisibility(const FString& VisualId);
    void AttachManifestChildren();
    void UpdateCelestialBodies(const FBskRenderFrame& Frame);
    double SolarVisibilityAt(const FVector3d& ReceiverMeters) const;
    void UpdateVisualStates(const FBskRenderFrame& Frame);
    void UpdateLightTargets();
    void ConfigureManifestLighting(const FBskSceneManifest& Manifest);
    void DrawOrbitLines(const FBskRenderFrame& Frame) const;
    AActor* SpawnUsdStage(const FString& ObjectName, const FObjectSpec& Spec);
    void CreateEnvironment();
    FObjectSpec ResolveSpec(const FBskRenderObjectState& State) const;

    TUniquePtr<IBskFrameSource> Receiver;
    FOnBskFrameApplied FrameAppliedEvent;
    TMap<FString, TObjectPtr<AActor>> BoundActors;
    TMap<FString, TObjectPtr<AActor>> CelestialActors;
    TMap<FString, TObjectPtr<AActor>> VisualActors;
    TMap<FString, FQuat> VisualBaseRotations;
    TMap<FString, FVector> VisualBaseScales;
    TMap<FString, bool> VisualDynamicVisibility;
    TMap<FString, bool> VisualKindVisibility;
    TMap<FString, TObjectPtr<AActor>> CameraActors;
    UPROPERTY(Transient)
    TMap<FString, TObjectPtr<USceneCaptureComponent2D>> CameraCaptureComponents;
    UPROPERTY(Transient)
    TMap<FString, TObjectPtr<UTextureRenderTarget2D>> CameraRenderTargets;
    UPROPERTY(Transient)
    TMap<FString, TObjectPtr<USceneCaptureComponent2D>> PixelStreamingCameraCaptures;
    UPROPERTY(Transient)
    TMap<FString, TObjectPtr<UTextureRenderTarget2D>> PixelStreamingCameraTargets;
    TMap<FString, TSharedPtr<IPixelStreaming2Streamer>> PixelStreamingCameraStreamers;
    TMap<FString, TSharedPtr<IPixelStreaming2VideoProducer>> PixelStreamingCameraProducers;
    TMap<FString, double> PixelStreamingCameraNextCaptureSeconds;
    FDelegateHandle PixelStreamingNewConnectionHandle;
    TMap<FString, double> CameraNextCaptureSeconds;
    TMap<FString, int64> CameraNextDataCaptureSimulationNanoseconds;
    TMap<FString, bool> CameraPictureInPictureVisibility;
    TObjectPtr<ADirectionalLight> SunLight;
    TObjectPtr<ADirectionalLight> CelestialSunLight;
    TObjectPtr<ADirectionalLight> FillLight;
    TObjectPtr<ASpotLight> Headlight;
    TObjectPtr<APostProcessVolume> ExposureVolume;
    TObjectPtr<ASkyAtmosphere> EarthAtmosphere;
    TObjectPtr<UStaticMeshComponent> DeepSkyComponent;
    TObjectPtr<AActor> DecorativeSunActor;
    TObjectPtr<UStaticMeshComponent> DecorativeSunComponent;
    UPROPERTY(Transient)
    TArray<TObjectPtr<UParticleSystemComponent>> DecorativeSunParticleComponents;
    TObjectPtr<AActor> DecorativeEarthActor;
    TSet<FString> CelestialBillboardIds;
    TMap<FString, FObjectSpec> ObjectSpecs;
    TMap<FString, FBskObjectDefinition> ManifestObjects;
    TMap<FString, FBskCelestialBodyDefinition> ManifestCelestialBodies;
    FString PrimaryDirectionalLightBodyId;
    TMap<FString, FBskVisualDefinition> ManifestVisuals;
    TMap<FString, FBskCameraDefinition> ManifestCameras;
    FBskCoordinateConverter Converter;
    FString ListenAddress = TEXT("127.0.0.1");
    FString ReplayPath;
    int32 ListenPort = 5558;
    double ReplayRate = 1.0;
    uint32 MaxPacketBytes = BskProtocol::DefaultMaxPacketBytes;
    bool bAllowExternalAssets = false;
    bool bShowOrbitLines = true;
    bool bUseOfficialCelestialAssets = true;
    bool bUseTexturedStarSphere = false;
    bool bUseEarthSkyAtmosphere = true;
    bool bUseManualExposure = true;
    bool bEnableDecorativeSun = true;
    bool bEnableDecorativeEarth = false;
    int32 StarCount = 320;
    double StarRadiusMeters = 250.0;
    double CelestialVaultRadiusKilometers = 500000.0;
    double CelestialBackgroundIntensity = 0.35;
    double SunVisualDistanceMeters = 10000.0;
    double SunVisualAngularDiameterDegrees = 2.0;
    double CurrentSunAngularDiameterDegrees = 2.0;
    double SunVisualEmissiveStrength = 1.5;
    bool bEnableDecorativeSunEffects = false;
    double SunVisualEffectScale = 1.0;
    double DecorativeSunSourceRadiusCentimeters = 0.0;
    FVector DecorativeSunSourceCenterCentimeters = FVector::ZeroVector;
    double DecorativeEarthRadiusMeters = 6371000.0;
    double EarthCloudScale = 1.001;
    double EarthAtmosphereScale = 1.001;
    FVector3d DecorativeEarthPositionMeters = FVector3d(0.0, 0.0, -6871000.0);
    FRotator DecorativeEarthRotation = FRotator::ZeroRotator;
    FString TexturedStarMeshPath = TEXT("/Game/_GENERATED/Hyperlovimia/Sphere_732702C4.Sphere_732702C4");
    FString TexturedStarMaterialPath = TEXT("/Game/Planets/Stars/M_Stars.M_Stars");
    FString SunVisualMeshPath = TEXT("/Engine/BasicShapes/Sphere.Sphere");
    FString SunVisualMaterialPath = TEXT("/Game/Planets/Sun/M_Sun.M_Sun");
    FString SunBurstParticlePath = TEXT("/Game/SpaceFX/Particles/P_Sun_Bursts.P_Sun_Bursts");
    FString SunHaloParticlePath = TEXT("/Game/SpaceFX/Particles/P_Sun_Halo.P_Sun_Halo");
    FString SunLinesParticlePath = TEXT("/Game/SpaceFX/Particles/P_Sun_Lines.P_Sun_Lines");
    FString EarthSphereMeshPath = TEXT("/Game/_GENERATED/Hyperlovimia/Sphere_732702C4.Sphere_732702C4");
    FString EarthSurfaceMaterialPath = TEXT("/Game/Planets/Earth/M_Earth.M_Earth");
    FString EarthCloudMaterialPath = TEXT("/Game/Planets/Earth/M_Clouds.M_Clouds");
    FString EarthAtmosphereMaterialPath = TEXT("/Game/Planets/Earth/M_Atmosphere.M_Atmosphere");
    double SunIntensityLux = 8.0;
    double SunIlluminanceScale = 1.0; // Renderer-local calibration, not overwritten by a scene.
    double SceneSunlightIntensityScale = 1.0; // [-], reset on every manifest
    double FillLightIntensityLux = 1.5;
    double MaterialExposureBias = 0.0;
    double ActiveMaterialAmbient = 0.18;
    bool bUseManifestSceneLighting = false;
    bool bEphemerisDirectionalLightActive = false;
    FRotator SunRotation = FRotator(-25.0, -35.0, 15.0);
    FVector CurrentSunSourceDirection = FVector::ForwardVector;
    FVector3d CurrentSunPositionMeters = FVector3d::ZeroVector;
    double CurrentSunRadiusMeters = 0.0;
    double CurrentSolarVisibility = 1.0;
    bool bEphemerisEarthLogged = false;
    TArray<TPair<FVector3d, double>> SolarOccluders;
    FVector3d CameraPositionMeters = FVector3d(10.0, -16.0, 8.0);
    FVector3d CameraLookAtMeters = FVector3d(0.0, 3.0, 0.0);
    int64 LastFrameId = -1;
    FString ActiveSessionId;
    int64 ActiveManifestRevision = 0;
    FString TimeMode = TEXT("interpolated");
    double InterpolationDelaySeconds = 0.1;
    double MaxExtrapolationSeconds = 0.1;
    double LastFrameArrivalSeconds = 0.0;
    double BlendElapsedSeconds = 0.0;
    double BlendDurationSeconds = 1.0 / 30.0;
    bool bHasTargetFrame = false;
    FBskRenderFrame PreviousFrame;
    FBskRenderFrame TargetFrame;
    FBskRenderFrame PresentationFrame;
    bool bHasPresentationFrame = false;
    bool bScreenshotRequested = false;
    TArray<FBskMissionEventView> MissionEventHistory;
    TMap<FString, double> PendingCommandIds;
    int64 CommandSequence = 0;
    FString LastCommandStatus = TEXT("idle");
    FString AutoCommand;
    bool bAutoCommandSent = false;
    FString CaptureOutputDirectory;
    FString CaptureNetworkAddress = TEXT("127.0.0.1");
    int32 CaptureNetworkPort = 0;
    double CaptureRateOverrideHertz = 0.0;
    double PreviewRateOverrideHertz = 0.0;
    FString PixelStreamingConnectionUrl;
    FString PixelStreamingBaseId = TEXT("BskRenderer");
    TSet<FString> PixelStreamingRequestedCameras;
    bool bPixelStreamingAllManifestCameras = false;
    int32 PixelStreamingCameraWidth = 640;
    int32 PixelStreamingCameraHeight = 360;
    double PixelStreamingCameraRateHertz = 90.0;
    double RequestedPixelStreamingCameraRateHertz = 90.0;
    int32 MaximumStreamingFrameRate = 0;
    bool bVideoDiagnostics = false;
    bool bManagesViewportRendering = false;
    bool bPreviousDisableWorldRendering = false;
    double VideoDiagnosticsStart = 0.0;
    double VideoDiagnosticsTickWork = 0.0;
    int32 VideoDiagnosticsTicks = 0;
    TMap<FString, int32> VideoDiagnosticsCaptures;
    TArray<FString> CaptureProductOverride;
    int64 CaptureSequence = 0;
    bool bAsyncCaptureEnabled = true;
    static constexpr int32 MaxAsyncCaptureJobs = 16;
    static constexpr uint64 MaxAsyncCaptureBytes = 256ull * 1024 * 1024;
    uint64 PendingCaptureBytes = 0;
    TArray<TSharedPtr<FBskAsyncCaptureJob, ESPMode::ThreadSafe>> PendingCaptureJobs;
    TSharedPtr<IBskCaptureProvider> BuiltinCaptureProvider;
    TSharedPtr<FBskCaptureNetworkSender> CaptureNetworkSender;
    TSharedPtr<FBskCaptureDiskWriter> CaptureDiskWriter;
};
