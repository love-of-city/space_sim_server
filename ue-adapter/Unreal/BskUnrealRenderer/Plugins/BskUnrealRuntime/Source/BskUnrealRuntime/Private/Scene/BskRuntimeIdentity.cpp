#include "BskRuntimeIdentity.h"
#include "BskUnrealRuntime.h"
#include "Dom/JsonObject.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformProcess.h"
#include "Misc/CommandLine.h"
#include "Misc/EngineVersion.h"
#include "Misc/FileHelper.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "UObject/UObjectGlobals.h"
#include "UObject/Class.h"

namespace
{
bool WriteJsonReport(const FString& Path, const TSharedRef<FJsonObject>& Json)
{
    FString Text;
    const FString Temporary = Path + TEXT(".partial");
    if (!FJsonSerializer::Serialize(Json, TJsonWriterFactory<>::Create(&Text))
        || !IFileManager::Get().MakeDirectory(*FPaths::GetPath(Path), true)
        || !FFileHelper::SaveStringToFile(Text, *Temporary, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM)
        || !IFileManager::Get().Move(*Path, *Temporary, true, true))
    {
        UE_LOG(LogBskUnreal, Error, TEXT("Cannot write runtime validation report: %s"), *Path);
        IFileManager::Get().Delete(*Temporary, false, true);
        return false;
    }
    return true;
}
}

void BskWriteRuntimeIdentity()
{
    FString ReportPath;
    if (!FParse::Value(FCommandLine::Get(), TEXT("BskRuntimeReport="), ReportPath)) return;
    if (FPaths::IsRelative(ReportPath))
    {
        UE_LOG(LogBskUnreal, Error, TEXT("BskRuntimeReport must be an absolute path"));
        return;
    }
    auto Report = MakeShared<FJsonObject>();
    Report->SetStringField(TEXT("schema"), TEXT("ue-runtime-report/1"));
    Report->SetNumberField(TEXT("pid"), FPlatformProcess::GetCurrentProcessId());
    Report->SetStringField(TEXT("engine_version"), FEngineVersion::Current().ToString());
    Report->SetBoolField(TEXT("monolithic"), IS_MONOLITHIC != 0);
    auto Modules = MakeShared<FJsonObject>();
    for (const FName Name : {FName(TEXT("BskUnrealRuntime")), FName(TEXT("BskCameraShaders"))})
    {
        FModuleManager& Manager = FModuleManager::Get();
        if (Name != FName(TEXT("BskUnrealRuntime"))) Manager.LoadModule(Name);
        FString Path = Manager.GetModuleFilename(Name);
#if IS_MONOLITHIC
        // A monolithic packaged executable contains both linked modules.
        if (Path.IsEmpty()) Path = FPaths::Combine(FPlatformProcess::BaseDir(), FPlatformProcess::ExecutableName(false));
#endif
        if (Path.IsEmpty()) UE_LOG(LogBskUnreal, Error, TEXT("No loaded module path for %s"), *Name.ToString());
        Modules->SetStringField(Name.ToString(), Path.IsEmpty() ? FString() : FPaths::ConvertRelativePathToFull(Path));
    }
    Report->SetObjectField(TEXT("modules"), Modules);
    WriteJsonReport(ReportPath, Report);
}

bool BskValidateCookAssetsIfRequested()
{
    FString ManifestPath;
    if (!FParse::Value(FCommandLine::Get(), TEXT("BskValidateCookAssets="), ManifestPath)) return false;
    FString ReportPath = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("cook-validation-report.json"));
    FParse::Value(FCommandLine::Get(), TEXT("BskCookValidationReport="), ReportPath);
    TArray<TSharedPtr<FJsonValue>> Results;
    TArray<TSharedPtr<FJsonValue>> Errors;
    FString Text;
    TSharedPtr<FJsonObject> Manifest;
    const TArray<TSharedPtr<FJsonValue>>* Assets = nullptr;
    FString Schema;
    if (!FFileHelper::LoadFileToString(Text, *ManifestPath)
        || !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), Manifest) || !Manifest
        || !Manifest->TryGetStringField(TEXT("schema"), Schema) || Schema != TEXT("ue-cook-runtime-assets/1")
        || !Manifest->TryGetArrayField(TEXT("assets"), Assets) || !Assets || Assets->IsEmpty())
    {
        Errors.Add(MakeShared<FJsonValueString>(TEXT("invalid or empty cook runtime asset manifest")));
    }
    else
    {
        TSet<FString> Seen;
        for (const auto& Value : *Assets)
        {
            const TSharedPtr<FJsonObject>* Asset = nullptr;
            FString Path, ExpectedClass;
            if (!Value || !Value->TryGetObject(Asset) || !Asset || !(*Asset)->TryGetStringField(TEXT("object_path"), Path)
                || !Path.StartsWith(TEXT("/")) || Seen.Contains(Path))
            {
                Errors.Add(MakeShared<FJsonValueString>(TEXT("invalid or duplicate asset object_path")));
                continue;
            }
            Seen.Add(Path);
            (*Asset)->TryGetStringField(TEXT("class"), ExpectedClass);
            UObject* Object = LoadObject<UObject>(nullptr, *Path);
            const FString ActualClass = Object ? Object->GetClass()->GetName() : FString();
            const bool bLoaded = Object && (ExpectedClass.IsEmpty() || ExpectedClass == ActualClass);
            auto Result = MakeShared<FJsonObject>();
            Result->SetStringField(TEXT("object_path"), Path);
            Result->SetBoolField(TEXT("loaded"), bLoaded);
            Result->SetStringField(TEXT("class"), ActualClass);
            Results.Add(MakeShared<FJsonValueObject>(Result));
            if (!bLoaded) Errors.Add(MakeShared<FJsonValueString>(TEXT("asset missing or class mismatch: ") + Path));
        }
    }
    auto Report = MakeShared<FJsonObject>();
    Report->SetStringField(TEXT("schema"), TEXT("ue-cook-runtime-validation/1"));
    Report->SetStringField(TEXT("engine_version"), FEngineVersion::Current().ToString());
    Report->SetNumberField(TEXT("pid"), FPlatformProcess::GetCurrentProcessId());
    Report->SetArrayField(TEXT("assets"), Results);
    Report->SetArrayField(TEXT("errors"), Errors);
    Report->SetBoolField(TEXT("passed"), Errors.IsEmpty());
    const bool bWritten = WriteJsonReport(ReportPath, Report);
    UE_LOG(LogBskUnreal, Display, TEXT("Cook runtime assets: %d checked, %d errors; report=%s"), Results.Num(), Errors.Num(), *ReportPath);
    FPlatformMisc::RequestExitWithStatus(false, Errors.IsEmpty() && bWritten ? 0 : 1);
    return true;
}
