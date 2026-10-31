#include "Scene/BskSceneUtilities.h"
#include "BskProtocolTypes.h"
#include "Misc/AutomationTest.h"
#include "Engine/World.h"
#include "Components/StaticMeshComponent.h"

using namespace BskSceneUtilities;

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBskOverlayPhysicalScaleTest,
    "BskUnreal.VisualOverlay.PhysicalUnits", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FBskOverlayPhysicalScaleTest::RunTest(const FString& Parameters)
{
    UWorld* World = UWorld::CreateWorld(EWorldType::Game, false);
    AActor* Owner = World->SpawnActor<AActor>();
    for (const double SourceScale : {1.0, 0.001})
    {
        UStaticMeshComponent* Parent = NewObject<UStaticMeshComponent>(Owner);
        Owner->AddInstanceComponent(Parent);
        Parent->RegisterComponent();
        Parent->SetWorldLocationAndRotation(FVector(123, -456, 789), FRotator(12, 34, 56));
        Parent->SetWorldScale3D(FVector(SourceScale));
        FBskGeometryDefinition Geometry;
        Geometry.AssetPath = SourceScale == 1.0
            ? TEXT("/Game/BSK/Generated/SARM/base_link.base_link")
            : TEXT("/Game/BSK/Generated/Sarm20261012/base_link_mujoco.base_link_mujoco");
        Geometry.Scale = FVector3d(SourceScale);
        AttachConfiguredVisualOverlay(Parent, Geometry);
        TestEqual(TEXT("one configured overlay attached"), Parent->GetNumChildrenComponents(), 1);
        if (Parent->GetNumChildrenComponents() == 1)
        {
            USceneComponent* Overlay = Parent->GetChildComponent(0);
            TestTrue(TEXT("centimetre overlay retains physical size for metre and millimetre source meshes"),
                Overlay->GetComponentScale().Equals(FVector::OneVector, 1.e-6));
            TestTrue(TEXT("overlay follows geometry position"), Overlay->GetComponentLocation().Equals(Parent->GetComponentLocation(), 1.e-6));
            TestTrue(TEXT("overlay follows geometry rotation"), Overlay->GetComponentQuat().Equals(Parent->GetComponentQuat(), 1.e-6));
        }
    }
    World->DestroyWorld(false);
    return true;
}
#endif
