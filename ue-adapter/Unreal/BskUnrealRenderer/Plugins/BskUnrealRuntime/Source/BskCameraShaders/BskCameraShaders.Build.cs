using UnrealBuildTool;
public class BskCameraShaders : ModuleRules
{
    public BskCameraShaders(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        PublicDependencyModuleNames.AddRange(new[] {"Core", "Engine"});
        PrivateDependencyModuleNames.AddRange(new[] {"CoreUObject", "Projects", "RenderCore", "RHI", "Renderer"});
    }
}
