using UnrealBuildTool;

public class BskUnrealRendererTarget : TargetRules
{
    public BskUnrealRendererTarget(TargetInfo Target) : base(Target)
    {
        Type = TargetType.Game;
        DefaultBuildSettings = BuildSettingsVersion.V5;
        IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_6;
        ExtraModuleNames.Add("BskUnrealRenderer");
    }
}
