#pragma once

#include "CoreMinimal.h"
#include "BskCoordinateConverter.h"
#include "BskProtocolTypes.h"

class AActor;
class UWorld;

/** Game-thread-only context passed to renderer extensions during Actor creation. */
struct BSKUNREALRUNTIME_API FBskRenderSpawnContext
{
    UWorld* World = nullptr;
    AActor* SceneController = nullptr;
    const FBskCoordinateConverter* CoordinateConverter = nullptr;
};

/**
 * Public runtime extension point for project-specific BSK visuals.
 *
 * Extensions never advance simulation state. They may create render Actors and
 * observe already-applied messages. Every method is invoked on the Game Thread.
 * Returning nullptr from a TrySpawn method delegates to the next extension and
 * ultimately to the built-in low-cost factory.
 */
class BSKUNREALRUNTIME_API IBskRenderExtension
{
public:
    virtual ~IBskRenderExtension() = default;
    virtual FName GetExtensionName() const = 0;

    virtual AActor* TrySpawnObject(const FBskRenderSpawnContext&, const FBskObjectDefinition&) { return nullptr; }
    virtual AActor* TrySpawnCelestialBody(const FBskRenderSpawnContext&, const FBskCelestialBodyDefinition&) { return nullptr; }
    virtual AActor* TrySpawnVisual(const FBskRenderSpawnContext&, const FBskVisualDefinition&) { return nullptr; }
    virtual AActor* TrySpawnCamera(const FBskRenderSpawnContext&, const FBskCameraDefinition&) { return nullptr; }

    virtual void OnManifestApplied(const FBskSceneManifest&) {}
    virtual void OnEventApplied(const FBskRenderEvent&) {}
    virtual void OnFrameApplied(const FBskRenderFrame&) {}
};
