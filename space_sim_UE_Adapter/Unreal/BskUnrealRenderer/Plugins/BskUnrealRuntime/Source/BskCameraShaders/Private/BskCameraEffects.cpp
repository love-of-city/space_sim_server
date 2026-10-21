#include "BskCameraEffects.h"
#include "Engine/TextureRenderTarget2D.h"
#include "GlobalShader.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"
#include "RenderGraphBuilder.h"
#include "RenderTargetPool.h"
#include "ScreenPass.h"
#include "ShaderParameterStruct.h"

class FBskCameraEffectPS : public FGlobalShader
{
    DECLARE_GLOBAL_SHADER(FBskCameraEffectPS);
    SHADER_USE_PARAMETER_STRUCT(FBskCameraEffectPS, FGlobalShader);
    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_TEXTURE(Texture2D, SourceTexture)
        SHADER_PARAMETER_SAMPLER(SamplerState, SourceSampler)
        SHADER_PARAMETER(FVector2f, OutputSize)
        SHADER_PARAMETER(float, HalfFov)
        SHADER_PARAMETER(uint32, Fisheye)
        RENDER_TARGET_BINDING_SLOTS()
    END_SHADER_PARAMETER_STRUCT()
};
IMPLEMENT_GLOBAL_SHADER(FBskCameraEffectPS, "/BskCamera/CameraEffect.usf", "MainPS", SF_Pixel);

class FBskCameraShadersModule : public IModuleInterface
{
public:
    virtual void StartupModule() override
    {
        const auto Plugin = IPluginManager::Get().FindPlugin(TEXT("BskUnrealRuntime"));
        check(Plugin.IsValid());
        AddShaderSourceDirectoryMapping(TEXT("/BskCamera"), FPaths::Combine(Plugin->GetBaseDir(), TEXT("Shaders")));
    }
};
IMPLEMENT_MODULE(FBskCameraShadersModule, BskCameraShaders);

void BskApplyCameraEffect(UTextureRenderTarget2D* Source, UTextureRenderTarget2D* Output,
    bool bFisheye, float FieldOfViewRadians)
{
    check(IsInGameThread());
    // Resource initialization and release are ordered on the render thread.
    // A newly created target may not have an RHI texture on the game thread yet.
    FTextureRenderTargetResource* InputResource = Source->GameThread_GetRenderTargetResource();
    FTextureRenderTargetResource* OutputResource = Output->GameThread_GetRenderTargetResource();
    const FVector2f Size(Output->SizeX, Output->SizeY);
    ENQUEUE_RENDER_COMMAND(BskCameraEffect)(
        [InputResource, OutputResource, Size, bFisheye, FieldOfViewRadians](FRHICommandListImmediate& RHICmdList)
        {
            FRDGBuilder Graph(RHICmdList);
            FRDGTextureRef Input = Graph.RegisterExternalTexture(CreateRenderTarget(InputResource->GetRenderTargetTexture(), TEXT("BskCameraSource")));
            FRDGTextureRef Dest = Graph.RegisterExternalTexture(CreateRenderTarget(OutputResource->GetRenderTargetTexture(), TEXT("BskCameraOutput")));
            auto* Params = Graph.AllocParameters<FBskCameraEffectPS::FParameters>();
            Params->SourceTexture = Input;
            Params->SourceSampler = TStaticSamplerState<SF_Bilinear, AM_Clamp, AM_Clamp>::GetRHI();
            Params->OutputSize = Size;
            Params->HalfFov = FieldOfViewRadians * 0.5f;
            Params->Fisheye = bFisheye ? 1u : 0u;
            Params->RenderTargets[0] = FRenderTargetBinding(Dest, ERenderTargetLoadAction::ENoAction);
            TShaderMapRef<FBskCameraEffectPS> PixelShader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
            AddDrawScreenPass(Graph, RDG_EVENT_NAME("BskCameraEffect"), FScreenPassViewInfo(),
                FScreenPassTextureViewport(Dest), FScreenPassTextureViewport(Input), PixelShader, Params);
            Graph.SetTextureAccessFinal(Dest, ERHIAccess::SRVMask);
            Graph.Execute();
        });
}
