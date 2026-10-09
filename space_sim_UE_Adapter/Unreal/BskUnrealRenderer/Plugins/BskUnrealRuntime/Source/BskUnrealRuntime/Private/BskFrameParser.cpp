#include "BskFrameParser.h"
#include "BskCelestialLighting.h"

#include "Dom/JsonObject.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

namespace
{
bool ReadVector3(const TSharedPtr<FJsonObject>& Object, const TCHAR* Field, FVector3d& Out, bool bRequired, FString& Error)
{
    const TArray<TSharedPtr<FJsonValue>>* Values = nullptr;
    if (!Object.IsValid() || !Object->TryGetArrayField(Field, Values))
    {
        if (bRequired) Error = FString::Printf(TEXT("missing vector field '%s'"), Field);
        return false;
    }
    if (Values == nullptr || Values->Num() != 3)
    {
        Error = FString::Printf(TEXT("field '%s' must contain three numbers"), Field);
        return false;
    }
    Out = FVector3d((*Values)[0]->AsNumber(), (*Values)[1]->AsNumber(), (*Values)[2]->AsNumber());
    if (!FMath::IsFinite(Out.X) || !FMath::IsFinite(Out.Y) || !FMath::IsFinite(Out.Z))
    {
        Error = FString::Printf(TEXT("field '%s' contains a non-finite number"), Field);
        return false;
    }
    return true;
}

bool ReadQuaternion(const TSharedPtr<FJsonObject>& Object, const TCHAR* Field, FQuat4d& Out, bool bRequired, FString& Error)
{
    const TArray<TSharedPtr<FJsonValue>>* Values = nullptr;
    if (!Object.IsValid() || !Object->TryGetArrayField(Field, Values))
    {
        if (bRequired) Error = FString::Printf(TEXT("missing quaternion field '%s'"), Field);
        return false;
    }
    if (Values == nullptr || Values->Num() != 4)
    {
        Error = FString::Printf(TEXT("field '%s' must contain four numbers"), Field);
        return false;
    }
    const double W = (*Values)[0]->AsNumber();
    const double X = (*Values)[1]->AsNumber();
    const double Y = (*Values)[2]->AsNumber();
    const double Z = (*Values)[3]->AsNumber();
    Out = FQuat4d(X, Y, Z, W);
    if (!FMath::IsFinite(W) || !FMath::IsFinite(X) || !FMath::IsFinite(Y) || !FMath::IsFinite(Z) ||
        Out.SizeSquared() <= UE_DOUBLE_SMALL_NUMBER)
    {
        Error = FString::Printf(TEXT("field '%s' contains an invalid quaternion"), Field);
        return false;
    }
    Out.Normalize();
    return true;
}

bool ReadColor(const TSharedPtr<FJsonObject>& Object, const TCHAR* Field, FLinearColor& Out, FString& Error)
{
    const TArray<TSharedPtr<FJsonValue>>* Values = nullptr;
    if (!Object.IsValid() || !Object->TryGetArrayField(Field, Values)) return false;
    if (Values == nullptr || (Values->Num() != 3 && Values->Num() != 4))
    {
        Error = FString::Printf(TEXT("field '%s' must contain three or four numbers"), Field);
        return false;
    }
    Out = FLinearColor(
        static_cast<float>((*Values)[0]->AsNumber()),
        static_cast<float>((*Values)[1]->AsNumber()),
        static_cast<float>((*Values)[2]->AsNumber()),
        Values->Num() == 4 ? static_cast<float>((*Values)[3]->AsNumber()) : 1.0f);
    return true;
}

bool ReadVector2(const TSharedPtr<FJsonObject>& Object, const TCHAR* Field, FVector2d& Out, FString& Error)
{
    const TArray<TSharedPtr<FJsonValue>>* Values = nullptr;
    if (!Object.IsValid() || !Object->TryGetArrayField(Field, Values)) return false;
    if (Values == nullptr || Values->Num() != 2)
    {
        Error = FString::Printf(TEXT("field '%s' must contain two numbers"), Field);
        return false;
    }
    Out = FVector2d((*Values)[0]->AsNumber(), (*Values)[1]->AsNumber());
    if (!FMath::IsFinite(Out.X) || !FMath::IsFinite(Out.Y))
    {
        Error = FString::Printf(TEXT("field '%s' contains a non-finite number"), Field);
        return false;
    }
    return true;
}

bool TryReadInt64(const TSharedPtr<FJsonObject>& Object, const TCHAR* Field, int64& Out)
{
    FString Text;
    if (Object->TryGetStringField(Field, Text)) return LexTryParseString(Out, *Text);
    double Number = 0.0;
    if (!Object->TryGetNumberField(Field, Number) || !FMath::IsFinite(Number)) return false;
    Out = static_cast<int64>(Number);
    return true;
}

void ReadStringArray(const TSharedPtr<FJsonObject>& Object, const TCHAR* Field, TArray<FString>& Out)
{
    const TArray<TSharedPtr<FJsonValue>>* Values = nullptr;
    if (!Object->TryGetArrayField(Field, Values) || Values == nullptr) return;
    for (const TSharedPtr<FJsonValue>& Value : *Values)
    {
        FString Text;
        if (Value.IsValid() && Value->TryGetString(Text)) Out.Add(MoveTemp(Text));
    }
}

bool ParseGeometry(const TSharedPtr<FJsonObject>& Object, FBskGeometryDefinition& Out, FString& Error)
{
    if (!Object.IsValid() || !Object->TryGetStringField(TEXT("geometry_id"), Out.GeometryId) || Out.GeometryId.IsEmpty() ||
        !Object->TryGetStringField(TEXT("shape"), Out.Shape) || Out.Shape.IsEmpty())
    {
        Error = TEXT("geometry requires geometry_id and shape");
        return false;
    }
    if (!ReadVector3(Object, TEXT("dimensions_m"), Out.DimensionsMeters, true, Error)) return false;
    ReadVector3(Object, TEXT("position_body_m"), Out.PositionBodyMeters, false, Error);
    if (!Error.IsEmpty()) return false;
    ReadQuaternion(Object, TEXT("orientation_body_from_geometry_wxyz"), Out.OrientationBodyFromGeometryWxyz, false, Error);
    if (!Error.IsEmpty()) return false;
    ReadColor(Object, TEXT("color_rgba"), Out.Color, Error);
    if (!Error.IsEmpty()) return false;
    Object->TryGetStringField(TEXT("asset_type"), Out.AssetType);
    Object->TryGetStringField(TEXT("asset_path"), Out.AssetPath);
    ReadVector3(Object, TEXT("scale"), Out.Scale, false, Error);
    if (!Error.IsEmpty()) return false;
    Object->TryGetStringField(TEXT("render_role"), Out.RenderRole);
    Object->TryGetStringField(TEXT("asset_key"), Out.AssetKey);
    Object->TryGetStringField(TEXT("material_name"), Out.MaterialName);
    Object->TryGetNumberField(TEXT("material_specular"), Out.MaterialSpecular);
    Object->TryGetNumberField(TEXT("material_shininess"), Out.MaterialShininess);
    Object->TryGetNumberField(TEXT("material_reflectance"), Out.MaterialReflectance);
    Object->TryGetNumberField(TEXT("material_emission"), Out.MaterialEmission);
    Object->TryGetStringField(TEXT("material_texture"), Out.MaterialTexture);
    Object->TryGetStringField(TEXT("material_texture_asset_path"), Out.MaterialTextureAssetPath);
    ReadVector2(Object, TEXT("material_texture_repeat"), Out.MaterialTextureRepeat, Error);
    Object->TryGetBoolField(TEXT("use_asset_materials"), Out.bUseAssetMaterials);
    if (!Error.IsEmpty()) return false;
    return true;
}

bool ParseManifest(const TSharedPtr<FJsonObject>& Root, FBskSceneManifest& Out, FString& Error)
{
    Out.Protocol = Root->GetStringField(TEXT("protocol"));
    Root->TryGetStringField(TEXT("session_id"), Out.SessionId);
    if (!TryReadInt64(Root, TEXT("revision"), Out.Revision))
    {
        Error = TEXT("scene_manifest requires an integer revision");
        return false;
    }
    // Reset this optional field even when a caller reuses an output message.
    Out.SunlightIntensityScale = 1.0;
    const TSharedPtr<FJsonObject>* Settings = nullptr;
    if (Root->TryGetObjectField(TEXT("settings"), Settings) && Settings != nullptr)
    {
        (*Settings)->TryGetStringField(TEXT("origin_object_id"), Out.OriginObjectId);
        (*Settings)->TryGetStringField(TEXT("default_camera_target"), Out.DefaultCameraTarget);
        (*Settings)->TryGetBoolField(TEXT("use_scene_lighting"), Out.bUseSceneLighting);
        (*Settings)->TryGetBoolField(TEXT("headlight_enabled"), Out.bHeadlightEnabled);
        ReadVector3(*Settings, TEXT("headlight_diffuse_rgb"), Out.HeadlightDiffuseRgb, false, Error);
        if (!Error.IsEmpty()) return false;
        ReadVector3(*Settings, TEXT("headlight_ambient_rgb"), Out.HeadlightAmbientRgb, false, Error);
        if (!Error.IsEmpty()) return false;
        ReadVector3(*Settings, TEXT("headlight_specular_rgb"), Out.HeadlightSpecularRgb, false, Error);
        if (!Error.IsEmpty()) return false;
        (*Settings)->TryGetNumberField(TEXT("fill_light_intensity_lux"), Out.FillLightIntensityLux);
        if ((*Settings)->HasField(TEXT("sunlight_intensity_scale")) &&
            (!(*Settings)->HasTypedField<EJson::Number>(TEXT("sunlight_intensity_scale")) ||
             !(*Settings)->TryGetNumberField(TEXT("sunlight_intensity_scale"), Out.SunlightIntensityScale) ||
             !BskCelestialLighting::IsValidSunlightIntensityScale(Out.SunlightIntensityScale)))
        {
            Error = TEXT("settings.sunlight_intensity_scale must be a finite number in [0, 20000]");
            return false;
        }
        (*Settings)->TryGetNumberField(TEXT("interpolation_delay_ms"), Out.InterpolationDelayMilliseconds);
        (*Settings)->TryGetNumberField(TEXT("max_extrapolation_ms"), Out.MaxExtrapolationMilliseconds);
        (*Settings)->TryGetNumberField(TEXT("default_camera_distance_m"), Out.DefaultCameraDistanceMeters);
        (*Settings)->TryGetBoolField(TEXT("orbit_lines"), Out.bOrbitLines);
        (*Settings)->TryGetBoolField(TEXT("trajectory_history"), Out.bTrajectoryHistory);
    }

    const TArray<TSharedPtr<FJsonValue>>* Objects = nullptr;
    if (!Root->TryGetArrayField(TEXT("objects"), Objects) || Objects == nullptr)
    {
        Error = TEXT("scene_manifest objects must be an array");
        return false;
    }
    TSet<FString> ObjectIds;
    for (const TSharedPtr<FJsonValue>& Value : *Objects)
    {
        const TSharedPtr<FJsonObject> Object = Value.IsValid() ? Value->AsObject() : nullptr;
        FBskObjectDefinition Definition;
        if (!Object.IsValid() || !Object->TryGetStringField(TEXT("object_id"), Definition.ObjectId) || Definition.ObjectId.IsEmpty())
        {
            Error = TEXT("manifest object requires object_id");
            return false;
        }
        if (ObjectIds.Contains(Definition.ObjectId))
        {
            Error = FString::Printf(TEXT("duplicate manifest object '%s'"), *Definition.ObjectId);
            return false;
        }
        ObjectIds.Add(Definition.ObjectId);
        Object->TryGetStringField(TEXT("display_name"), Definition.DisplayName);
        Object->TryGetStringField(TEXT("parent_id"), Definition.ParentId);
        Object->TryGetStringField(TEXT("transform_space"), Definition.TransformSpace);
        Object->TryGetStringField(TEXT("asset_path"), Definition.AssetPath);
        Object->TryGetStringField(TEXT("semantic_label"), Definition.SemanticLabel);
        const TArray<TSharedPtr<FJsonValue>>* Geometries = nullptr;
        if (Object->TryGetArrayField(TEXT("geometries"), Geometries) && Geometries != nullptr)
        {
            for (const TSharedPtr<FJsonValue>& GeometryValue : *Geometries)
            {
                FBskGeometryDefinition Geometry;
                if (!ParseGeometry(GeometryValue.IsValid() ? GeometryValue->AsObject() : nullptr, Geometry, Error)) return false;
                Definition.Geometries.Add(MoveTemp(Geometry));
            }
        }
        Out.Objects.Add(MoveTemp(Definition));
    }

    const TArray<TSharedPtr<FJsonValue>>* CelestialBodies = nullptr;
    if (Root->TryGetArrayField(TEXT("celestial_bodies"), CelestialBodies) && CelestialBodies != nullptr)
    {
        for (const TSharedPtr<FJsonValue>& Value : *CelestialBodies)
        {
            const TSharedPtr<FJsonObject> Object = Value.IsValid() ? Value->AsObject() : nullptr;
            FBskCelestialBodyDefinition Definition;
            if (!Object.IsValid() || !Object->TryGetStringField(TEXT("body_id"), Definition.BodyId) || Definition.BodyId.IsEmpty())
            {
                Error = TEXT("celestial body requires body_id");
                return false;
            }
            Object->TryGetStringField(TEXT("display_name"), Definition.DisplayName);
            Object->TryGetNumberField(TEXT("mu_m3_s2"), Definition.MuMetersCubedPerSecondSquared);
            Object->TryGetNumberField(TEXT("equatorial_radius_m"), Definition.EquatorialRadiusMeters);
            Object->TryGetNumberField(TEXT("polar_radius_ratio"), Definition.PolarRadiusRatio);
            Object->TryGetStringField(TEXT("asset_path"), Definition.AssetPath);
            Object->TryGetStringField(TEXT("visual_role"), Definition.VisualRole);
            Object->TryGetBoolField(TEXT("luminous"), Definition.bLuminous);
            if (!Object->TryGetBoolField(TEXT("drives_directional_light"), Definition.bDrivesDirectionalLight))
            {
                // bsk-render/2 manifests produced before explicit light roles
                // used luminous=true for the primary star.
                Definition.bDrivesDirectionalLight = Definition.bLuminous;
            }
            ReadVector3(Object, TEXT("light_color_rgb"), Definition.LightColorRgb, false, Error);
            if (!Error.IsEmpty()) return false;
            Object->TryGetNumberField(
                TEXT("light_illuminance_lux_at_reference_distance"),
                Definition.LightIlluminanceLuxAtReferenceDistance);
            Object->TryGetNumberField(TEXT("light_reference_distance_m"), Definition.LightReferenceDistanceMeters);
            Out.CelestialBodies.Add(MoveTemp(Definition));
        }
    }

    const TArray<TSharedPtr<FJsonValue>>* Visuals = nullptr;
    if (Root->TryGetArrayField(TEXT("visuals"), Visuals) && Visuals != nullptr)
    {
        for (const TSharedPtr<FJsonValue>& Value : *Visuals)
        {
            const TSharedPtr<FJsonObject> Object = Value.IsValid() ? Value->AsObject() : nullptr;
            FBskVisualDefinition Definition;
            if (!Object.IsValid() || !Object->TryGetStringField(TEXT("visual_id"), Definition.VisualId) || Definition.VisualId.IsEmpty() ||
                !Object->TryGetStringField(TEXT("kind"), Definition.Kind) || Definition.Kind.IsEmpty())
            {
                Error = TEXT("visual requires visual_id and kind");
                return false;
            }
            Object->TryGetStringField(TEXT("parent_id"), Definition.ParentId);
            if (Definition.ParentId.IsEmpty() && !Definition.Kind.Equals(TEXT("light"), ESearchCase::IgnoreCase))
            {
                Error = TEXT("non-light visual requires parent_id");
                return false;
            }
            ReadVector3(Object, TEXT("position_body_m"), Definition.PositionBodyMeters, false, Error);
            if (!Error.IsEmpty()) return false;
            ReadQuaternion(Object, TEXT("orientation_body_from_visual_wxyz"), Definition.OrientationBodyFromVisualWxyz, false, Error);
            if (!Error.IsEmpty()) return false;
            ReadVector3(Object, TEXT("normal_body"), Definition.NormalBody, false, Error);
            if (!Error.IsEmpty()) return false;
            const TArray<TSharedPtr<FJsonValue>>* Fov = nullptr;
            if (Object->TryGetArrayField(TEXT("field_of_view_rad"), Fov) && Fov != nullptr && Fov->Num() > 0)
            {
                Definition.FieldOfViewRadians.X = (*Fov)[0]->AsNumber();
                Definition.FieldOfViewRadians.Y = Fov->Num() > 1 ? (*Fov)[1]->AsNumber() : Definition.FieldOfViewRadians.X;
            }
            Object->TryGetNumberField(TEXT("size_m"), Definition.SizeMeters);
            Object->TryGetNumberField(TEXT("range_m"), Definition.RangeMeters);
            ReadColor(Object, TEXT("color_rgba"), Definition.Color, Error);
            if (!Error.IsEmpty()) return false;
            Object->TryGetStringField(TEXT("label"), Definition.Label);
            const TSharedPtr<FJsonObject>* Properties = nullptr;
            if (Object->TryGetObjectField(TEXT("properties"), Properties) && Properties != nullptr)
            {
                (*Properties)->TryGetStringField(TEXT("light_type"), Definition.LightType);
                (*Properties)->TryGetStringField(TEXT("target_id"), Definition.LightTargetId);
                (*Properties)->TryGetNumberField(TEXT("intensity"), Definition.LightIntensity);
                (*Properties)->TryGetNumberField(TEXT("cutoff_deg"), Definition.LightCutoffDegrees);
                (*Properties)->TryGetBoolField(TEXT("cast_shadows"), Definition.bLightCastShadows);
                ReadVector3(*Properties, TEXT("diffuse_rgb"), Definition.LightDiffuseRgb, false, Error);
                if (!Error.IsEmpty()) return false;
                ReadVector3(*Properties, TEXT("specular_rgb"), Definition.LightSpecularRgb, false, Error);
                if (!Error.IsEmpty()) return false;
            }
            const TSharedPtr<FJsonObject>* ChannelSchema = nullptr;
            if (Object->TryGetObjectField(TEXT("channel_schema"), ChannelSchema) && ChannelSchema != nullptr)
            {
                for (const TPair<FString, TSharedPtr<FJsonValue>>& Pair : (*ChannelSchema)->Values)
                {
                    const TSharedPtr<FJsonObject> ChannelObject = Pair.Value.IsValid() ? Pair.Value->AsObject() : nullptr;
                    if (!ChannelObject.IsValid()) continue;
                    FBskChannelDefinition Channel;
                    ChannelObject->TryGetStringField(TEXT("type"), Channel.Type);
                    ChannelObject->TryGetStringField(TEXT("unit"), Channel.Unit);
                    Channel.bHasMinimum = ChannelObject->TryGetNumberField(TEXT("minimum"), Channel.Minimum);
                    Channel.bHasMaximum = ChannelObject->TryGetNumberField(TEXT("maximum"), Channel.Maximum);
                    Definition.ChannelSchema.Add(Pair.Key, MoveTemp(Channel));
                }
            }
            Out.Visuals.Add(MoveTemp(Definition));
        }
    }

    const TArray<TSharedPtr<FJsonValue>>* Cameras = nullptr;
    if (Root->TryGetArrayField(TEXT("cameras"), Cameras) && Cameras != nullptr)
    {
        for (const TSharedPtr<FJsonValue>& Value : *Cameras)
        {
            const TSharedPtr<FJsonObject> Object = Value.IsValid() ? Value->AsObject() : nullptr;
            FBskCameraDefinition Definition;
            if (!Object.IsValid() || !Object->TryGetStringField(TEXT("camera_id"), Definition.CameraId) || Definition.CameraId.IsEmpty())
            {
                Error = TEXT("camera requires camera_id");
                return false;
            }
            Object->TryGetStringField(TEXT("parent_id"), Definition.ParentId);
            Object->TryGetStringField(TEXT("display_name"), Definition.DisplayName);
            ReadVector3(Object, TEXT("position_body_m"), Definition.PositionBodyMeters, false, Error);
            if (!Error.IsEmpty()) return false;
            ReadQuaternion(Object, TEXT("orientation_body_from_camera_wxyz"), Definition.OrientationBodyFromCameraWxyz, false, Error);
            if (!Error.IsEmpty()) return false;
            Object->TryGetNumberField(TEXT("field_of_view_rad"), Definition.FieldOfViewRadians);
            const TArray<TSharedPtr<FJsonValue>>* Resolution = nullptr;
            if (Object->TryGetArrayField(TEXT("resolution"), Resolution) && Resolution != nullptr && Resolution->Num() == 2)
            {
                Definition.Resolution = FIntPoint((*Resolution)[0]->AsNumber(), (*Resolution)[1]->AsNumber());
            }
            Object->TryGetStringField(TEXT("semantic_label"), Definition.SemanticLabel);
            Object->TryGetBoolField(TEXT("picture_in_picture"), Definition.bPictureInPicture);
            Object->TryGetNumberField(TEXT("capture_rate_hz"), Definition.CaptureRateHertz);
            double PictureInPictureSlot = 0.0;
            if (Object->TryGetNumberField(TEXT("picture_in_picture_slot"), PictureInPictureSlot))
            {
                Definition.PictureInPictureSlot = FMath::Max(0, FMath::RoundToInt(PictureInPictureSlot));
            }
            const TArray<TSharedPtr<FJsonValue>>* CaptureProducts = nullptr;
            if (Object->TryGetArrayField(TEXT("capture_products"), CaptureProducts) && CaptureProducts != nullptr)
            {
                TSet<FString> SeenProducts;
                for (const TSharedPtr<FJsonValue>& ProductValue : *CaptureProducts)
                {
                    FString Product;
                    if (!ProductValue.IsValid() || !ProductValue->TryGetString(Product))
                    {
                        Error = TEXT("camera capture_products entries must be strings");
                        return false;
                    }
                    Product = Product.TrimStartAndEnd().ToLower();
                    if (Product != TEXT("rgb"))
                    {
                        Error = FString::Printf(TEXT("camera '%s' requests unsupported capture product '%s'"), *Definition.CameraId, *Product);
                        return false;
                    }
                    if (!SeenProducts.Contains(Product))
                    {
                        SeenProducts.Add(Product);
                        Definition.CaptureProducts.Add(MoveTemp(Product));
                    }
                }
            }
            Definition.Resolution.X = FMath::Clamp(Definition.Resolution.X, 64, 4096);
            Definition.Resolution.Y = FMath::Clamp(Definition.Resolution.Y, 64, 4096);
            Definition.CaptureRateHertz = FMath::Clamp(Definition.CaptureRateHertz, 1.0, 60.0);
            if (Definition.DisplayName.IsEmpty()) Definition.DisplayName = Definition.CameraId;
            Out.Cameras.Add(MoveTemp(Definition));
        }
    }
    const TSharedPtr<FJsonObject>* Ui = nullptr;
    const TArray<TSharedPtr<FJsonValue>>* Commands = nullptr;
    if (Settings != nullptr &&
        (*Settings)->TryGetObjectField(TEXT("ui"), Ui) && Ui != nullptr &&
        (*Ui)->TryGetArrayField(TEXT("commands"), Commands) && Commands != nullptr)
    {
        TSet<FString> SeenCommands;
        for (const TSharedPtr<FJsonValue>& Value : *Commands)
        {
            const TSharedPtr<FJsonObject> Object = Value.IsValid() ? Value->AsObject() : nullptr;
            FBskUiCommandDefinition Definition;
            if (!Object.IsValid() || !Object->TryGetStringField(TEXT("command"), Definition.Command) || Definition.Command.IsEmpty())
            {
                Error = TEXT("settings.ui.commands entries require command");
                return false;
            }
            if (SeenCommands.Contains(Definition.Command))
            {
                Error = FString::Printf(TEXT("duplicate UI command '%s'"), *Definition.Command);
                return false;
            }
            SeenCommands.Add(Definition.Command);
            Object->TryGetStringField(TEXT("label"), Definition.Label);
            Object->TryGetStringField(TEXT("target_id"), Definition.TargetId);
            Object->TryGetBoolField(TEXT("requires_confirmation"), Definition.bRequiresConfirmation);
            const TSharedPtr<FJsonObject>* Payload = nullptr;
            if (Object->TryGetObjectField(TEXT("payload"), Payload) && Payload != nullptr)
            {
                FJsonSerializer::Serialize((*Payload).ToSharedRef(), TJsonWriterFactory<>::Create(&Definition.PayloadJson));
            }
            if (Definition.Label.IsEmpty()) Definition.Label = Definition.Command;
            Out.UiCommands.Add(MoveTemp(Definition));
        }
    }
    return true;
}

bool ParseFrame(const TSharedPtr<FJsonObject>& Root, FBskRenderFrame& Out, FString& Error)
{
    Out.Protocol = Root->GetStringField(TEXT("protocol"));
    Root->TryGetStringField(TEXT("session_id"), Out.SessionId);
    if (!TryReadInt64(Root, TEXT("frame_id"), Out.FrameId) || !TryReadInt64(Root, TEXT("sim_time_ns"), Out.SimulationTimeNanoseconds))
    {
        Error = TEXT("frame_id and sim_time_ns must be integers or decimal strings");
        return false;
    }
    Out.bCaptureOnDemand = Root->HasField(TEXT("capture_episode_id"));
    if (Out.bCaptureOnDemand && !Root->TryGetStringField(TEXT("capture_episode_id"), Out.CaptureEpisodeId))
    {
        Error = TEXT("capture_episode_id must be a string (empty means preview only)");
        return false;
    }
    TryReadInt64(Root, TEXT("wall_time_ns"), Out.WallTimeNanoseconds);
    TryReadInt64(Root, TEXT("manifest_revision"), Out.ManifestRevision);
    if (!ReadVector3(Root, TEXT("origin_N_m"), Out.OriginInertialMeters, true, Error)) return false;

    const TArray<TSharedPtr<FJsonValue>>* Basis = nullptr;
    if (Root->TryGetArrayField(TEXT("c_LN"), Basis) && Basis != nullptr)
    {
        if (Basis->Num() != 9)
        {
            Error = TEXT("c_LN must contain nine numbers");
            return false;
        }
        for (int32 Row = 0; Row < 3; ++Row)
        {
            for (int32 Column = 0; Column < 3; ++Column)
            {
                Out.LocalFromInertial.M[Row][Column] = (*Basis)[Row * 3 + Column]->AsNumber();
            }
        }
    }

    const TArray<TSharedPtr<FJsonValue>>* Objects = nullptr;
    if (!Root->TryGetArrayField(TEXT("objects"), Objects) || Objects == nullptr)
    {
        Error = TEXT("objects must be an array");
        return false;
    }
    TSet<FString> ObjectIds;
    for (const TSharedPtr<FJsonValue>& Value : *Objects)
    {
        const TSharedPtr<FJsonObject> Object = Value.IsValid() ? Value->AsObject() : nullptr;
        FBskRenderObjectState State;
        if (!Object.IsValid())
        {
            Error = TEXT("frame object must be a JSON object");
            return false;
        }
        if (!Object->TryGetStringField(TEXT("object_id"), State.ObjectId))
        {
            Object->TryGetStringField(TEXT("name"), State.ObjectId);
        }
        if (State.ObjectId.IsEmpty() || ObjectIds.Contains(State.ObjectId))
        {
            Error = State.ObjectId.IsEmpty() ? TEXT("frame object requires object_id or name") : FString::Printf(TEXT("duplicate object '%s'"), *State.ObjectId);
            return false;
        }
        ObjectIds.Add(State.ObjectId);
        State.Name = State.ObjectId;
        if (!ReadVector3(Object, TEXT("position_m"), State.PositionMeters, true, Error) ||
            !ReadQuaternion(Object, TEXT("orientation_wxyz"), State.OrientationWxyz, true, Error)) return false;
        Object->TryGetStringField(TEXT("parent"), State.Parent);
        Object->TryGetStringField(TEXT("asset_path"), State.AssetPath);
        Object->TryGetStringField(TEXT("semantic_label"), State.SemanticLabel);
        State.bHasVelocity = ReadVector3(Object, TEXT("velocity_mps"), State.VelocityMetersPerSecond, false, Error);
        if (!Error.IsEmpty()) return false;
        State.bHasAngularVelocity = ReadVector3(Object, TEXT("angular_velocity_B_radps"), State.AngularVelocityBodyRadiansPerSecond, false, Error);
        if (!Error.IsEmpty()) return false;
        Out.Objects.Add(MoveTemp(State));
    }

    const TArray<TSharedPtr<FJsonValue>>* Celestial = nullptr;
    if (Root->TryGetArrayField(TEXT("celestial_bodies"), Celestial) && Celestial != nullptr)
    {
        for (const TSharedPtr<FJsonValue>& Value : *Celestial)
        {
            const TSharedPtr<FJsonObject> Object = Value.IsValid() ? Value->AsObject() : nullptr;
            FBskCelestialBodyState State;
            if (!Object.IsValid() || !Object->TryGetStringField(TEXT("body_id"), State.BodyId) || State.BodyId.IsEmpty() ||
                !ReadVector3(Object, TEXT("position_m"), State.PositionMeters, true, Error) ||
                !ReadQuaternion(Object, TEXT("orientation_wxyz"), State.OrientationWxyz, true, Error)) return false;
            ReadVector3(Object, TEXT("velocity_mps"), State.VelocityMetersPerSecond, false, Error);
            if (!Error.IsEmpty()) return false;
            Out.CelestialBodies.Add(MoveTemp(State));
        }
    }

    const TArray<TSharedPtr<FJsonValue>>* VisualStates = nullptr;
    if (Root->TryGetArrayField(TEXT("visual_states"), VisualStates) && VisualStates != nullptr)
    {
        for (const TSharedPtr<FJsonValue>& Value : *VisualStates)
        {
            const TSharedPtr<FJsonObject> Object = Value.IsValid() ? Value->AsObject() : nullptr;
            FBskVisualState State;
            if (!Object.IsValid() || !Object->TryGetStringField(TEXT("visual_id"), State.VisualId) || State.VisualId.IsEmpty()) continue;
            Object->TryGetNumberField(TEXT("value"), State.Value);
            Object->TryGetNumberField(TEXT("status"), State.Status);
            Object->TryGetBoolField(TEXT("visible"), State.bVisible);
            const TSharedPtr<FJsonObject>* Channels = nullptr;
            if (Object->TryGetObjectField(TEXT("channels"), Channels) && Channels != nullptr)
            {
                for (const TPair<FString, TSharedPtr<FJsonValue>>& Pair : (*Channels)->Values)
                {
                    if (!Pair.Value.IsValid()) continue;
                    FBskVisualState::FChannelValue Channel;
                    switch (Pair.Value->Type)
                    {
                    case EJson::Number:
                        Channel.Type = FBskVisualState::FChannelValue::EType::Number;
                        Channel.Number = Pair.Value->AsNumber();
                        break;
                    case EJson::Boolean:
                        Channel.Type = FBskVisualState::FChannelValue::EType::Boolean;
                        Channel.Boolean = Pair.Value->AsBool();
                        break;
                    case EJson::String:
                        Channel.Type = FBskVisualState::FChannelValue::EType::String;
                        Channel.String = Pair.Value->AsString();
                        break;
                    default:
                        continue;
                    }
                    State.Channels.Add(Pair.Key, MoveTemp(Channel));
                }
            }
            Out.VisualStates.Add(MoveTemp(State));
        }
    }
    return true;
}
}

FBskFrameParser::FBskFrameParser(uint32 InMaxPacketBytes)
    : MaxPacketBytes(FMath::Clamp(InMaxPacketBytes, 1u, BskProtocol::DefaultMaxPacketBytes))
{
}

void FBskFrameParser::Reset()
{
    Buffer.Reset();
}

bool FBskFrameParser::AppendMessages(const uint8* Data, int32 NumBytes, TArray<FBskRenderMessage>& OutMessages, FString& OutError)
{
    if (NumBytes < 0 || (NumBytes > 0 && Data == nullptr))
    {
        OutError = TEXT("invalid byte span");
        return false;
    }
    Buffer.Append(Data, NumBytes);
    while (Buffer.Num() >= 4)
    {
        const uint32 Length = (static_cast<uint32>(Buffer[0]) << 24) |
            (static_cast<uint32>(Buffer[1]) << 16) | (static_cast<uint32>(Buffer[2]) << 8) | static_cast<uint32>(Buffer[3]);
        if (Length == 0 || Length > MaxPacketBytes)
        {
            OutError = FString::Printf(TEXT("invalid packet length: %u"), Length);
            Buffer.Reset();
            return false;
        }
        if (static_cast<uint64>(Buffer.Num()) < static_cast<uint64>(Length) + 4ull) break;
        FUTF8ToTCHAR Converted(reinterpret_cast<const ANSICHAR*>(Buffer.GetData() + 4), static_cast<int32>(Length));
        const FString Json(Converted.Length(), Converted.Get());
        FBskRenderMessage Message;
        if (!ParseMessageJson(Json, Message, OutError))
        {
            Buffer.RemoveAt(0, static_cast<int32>(Length) + 4, EAllowShrinking::No);
            return false;
        }
        OutMessages.Add(MoveTemp(Message));
        Buffer.RemoveAt(0, static_cast<int32>(Length) + 4, EAllowShrinking::No);
    }
    return true;
}

bool FBskFrameParser::Append(const uint8* Data, int32 NumBytes, TArray<FBskRenderFrame>& OutFrames, FString& OutError)
{
    TArray<FBskRenderMessage> Messages;
    if (!AppendMessages(Data, NumBytes, Messages, OutError)) return false;
    for (FBskRenderMessage& Message : Messages)
    {
        if (Message.Type == EBskRenderMessageType::Frame) OutFrames.Add(MoveTemp(Message.Frame));
    }
    return true;
}

bool FBskFrameParser::ParseMessageJson(const FString& Json, FBskRenderMessage& OutMessage, FString& OutError)
{
    TSharedPtr<FJsonObject> Root;
    if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Json), Root) || !Root.IsValid())
    {
        OutError = TEXT("packet body is not a JSON object");
        return false;
    }
    if (!Root->TryGetStringField(TEXT("protocol"), OutMessage.Protocol) || !BskProtocol::IsSupported(OutMessage.Protocol))
    {
        OutError = TEXT("unsupported protocol identifier");
        return false;
    }
    FString Type;
    if (!Root->TryGetStringField(TEXT("type"), Type))
    {
        OutError = TEXT("message requires a type");
        return false;
    }
    Root->TryGetStringField(TEXT("session_id"), OutMessage.SessionId);
    if (Type == TEXT("hello"))
    {
        if (OutMessage.Protocol != BskProtocol::GenericV2)
        {
            OutError = TEXT("hello is only valid in bsk-render/2");
            return false;
        }
        OutMessage.Type = EBskRenderMessageType::Hello;
        ReadStringArray(Root, TEXT("capabilities"), OutMessage.Capabilities);
        ReadStringArray(Root, TEXT("required_capabilities"), OutMessage.RequiredCapabilities);
        for (const FString& Required : OutMessage.RequiredCapabilities)
        {
            if (Required != TEXT("scene_manifest"))
            {
                OutError = FString::Printf(TEXT("unsupported required capability '%s'"), *Required);
                return false;
            }
        }
        return true;
    }
    if (Type == TEXT("scene_manifest"))
    {
        OutMessage.Type = EBskRenderMessageType::SceneManifest;
        return ParseManifest(Root, OutMessage.Manifest, OutError);
    }
    if (Type == TEXT("frame"))
    {
        OutMessage.Type = EBskRenderMessageType::Frame;
        return ParseFrame(Root, OutMessage.Frame, OutError);
    }
    if (Type == TEXT("event"))
    {
        OutMessage.Type = EBskRenderMessageType::Event;
        OutMessage.Event.Protocol = OutMessage.Protocol;
        OutMessage.Event.SessionId = OutMessage.SessionId;
        TryReadInt64(Root, TEXT("sequence"), OutMessage.Event.Sequence);
        if (!Root->TryGetStringField(TEXT("event_kind"), OutMessage.Event.EventKind) || OutMessage.Event.EventKind.IsEmpty())
        {
            OutError = TEXT("event requires event_kind");
            return false;
        }
        const TSharedPtr<FJsonObject>* Payload = nullptr;
        if (Root->TryGetObjectField(TEXT("payload"), Payload) && Payload != nullptr)
        {
            const TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&OutMessage.Event.PayloadJson);
            FJsonSerializer::Serialize((*Payload).ToSharedRef(), Writer);
        }
        return true;
    }
    OutError = FString::Printf(TEXT("unsupported message type '%s'"), *Type);
    return false;
}

bool FBskFrameParser::ParseJson(const FString& Json, FBskRenderFrame& OutFrame, FString& OutError)
{
    FBskRenderMessage Message;
    if (!ParseMessageJson(Json, Message, OutError)) return false;
    if (Message.Type != EBskRenderMessageType::Frame)
    {
        OutError = TEXT("message type must be 'frame'");
        return false;
    }
    OutFrame = MoveTemp(Message.Frame);
    return true;
}
