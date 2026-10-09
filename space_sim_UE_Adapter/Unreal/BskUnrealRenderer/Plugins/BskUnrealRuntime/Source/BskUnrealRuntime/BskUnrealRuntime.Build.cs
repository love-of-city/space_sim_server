using UnrealBuildTool;

public class BskUnrealRuntime : ModuleRules
{
    public BskUnrealRuntime(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        PublicDependencyModuleNames.AddRange(new[]
        {
            "Core", "CoreUObject", "Engine", "InputCore", "Json", "Networking", "Sockets", "ImageWrapper", "RenderCore", "RHI",
            "PixelStreaming2", "PixelStreaming2Core", "PixelStreaming2Input", "PixelCapture"
        });
        PrivateDependencyModuleNames.Add("PixelStreaming2RTC");
        PrivateIncludePaths.Add(System.IO.Path.Combine(
            EngineDirectory,
            "Plugins/Media/PixelStreaming2/Source/PixelStreaming2/Internal"));
    }
}
