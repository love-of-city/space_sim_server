#include "BskUnrealRuntime.h"
#include "Modules/ModuleManager.h"
#include "Scene/BskRuntimeIdentity.h"

DEFINE_LOG_CATEGORY(LogBskUnreal);
class FBskUnrealRuntimeModule final : public IModuleInterface
{
public:
    virtual void StartupModule() override { BskWriteRuntimeIdentity(); }
};
IMPLEMENT_MODULE(FBskUnrealRuntimeModule, BskUnrealRuntime);
