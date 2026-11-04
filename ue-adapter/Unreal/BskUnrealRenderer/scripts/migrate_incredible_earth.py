"""Author IncredibleEarth80K visuals into the existing BSK environment.

Copy Content/IncredibleEarth from the licensed source project first. Build the
Editor target, back up the existing map, then run with UE's Python commandlet.
Only this explicit migration command changes the saved environment. Startup and
backend manifests never re-run it. Source packages are read-only inputs.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import unreal

SOURCE_MAP = '/Game/IncredibleEarth/IncredibleEarth80K'
TARGET_MAP = '/Game/BSK/Maps/BskEnvironment'
OUTPUT = '/Game/BSK/Environment/IncredibleEarth'
MOON_RADIUS_METERS = 1737400.0
# HDR captures confirmed these manual-exposure baselines. Preserve the platform
# camera and physical lighting; adapt authored emissive radiance independently.
SOURCE_REFERENCE_EXPOSURE = 100.0 / (100.0 * 60.0 * 4.0 ** 2) * 2.0 ** 9
TARGET_REFERENCE_EXPOSURE = 100.0 / (100.0 * 125.0 * 16.0 ** 2)
EMISSIVE_EXPOSURE_GAIN = SOURCE_REFERENCE_EXPOSURE / TARGET_REFERENCE_EXPOSURE
SOURCE_SUN_ILLUMINANCE_LUX = 13.0
TARGET_SUN_ILLUMINANCE_LUX = 132821.5
SUN_ILLUMINANCE_GAIN = TARGET_SUN_ILLUMINANCE_LUX / SOURCE_SUN_ILLUMINANCE_LUX
LIB = unreal.MaterialEditingLibrary
ASSETS = unreal.EditorAssetLibrary
ACTORS = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
LEVELS = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
REPORT = {'source_map': SOURCE_MAP, 'target_map': TARGET_MAP, 'materials': {}}
# Explicitly exclude exposure and rendering-pipeline overrides. Some UE
# versions omit or rename settings, so snapshots check each reflected field.
ARTISTIC_POSTPROCESS_FIELDS = (
    'temperature_type', 'white_temp', 'white_tint',
    'color_saturation', 'color_contrast', 'color_gamma', 'color_gain', 'color_offset',
    'color_saturation_shadows', 'color_contrast_shadows', 'color_gamma_shadows',
    'color_gain_shadows', 'color_offset_shadows',
    'color_saturation_midtones', 'color_contrast_midtones', 'color_gamma_midtones',
    'color_gain_midtones', 'color_offset_midtones',
    'color_saturation_highlights', 'color_contrast_highlights', 'color_gamma_highlights',
    'color_gain_highlights', 'color_offset_highlights',
    'color_correction_shadows_max', 'color_correction_highlights_min',
    'color_correction_highlights_max', 'color_grading_intensity', 'color_grading_lut',
    'scene_color_tint', 'blue_correction', 'expand_gamut', 'tone_curve_amount',
    'film_slope', 'film_toe', 'film_shoulder', 'film_black_clip', 'film_white_clip',
    'bloom_method', 'bloom_intensity', 'bloom_threshold', 'bloom_size_scale',
    'bloom1_size', 'bloom1_tint', 'bloom2_size', 'bloom2_tint',
    'bloom3_size', 'bloom3_tint', 'bloom4_size', 'bloom4_tint',
    'bloom5_size', 'bloom5_tint', 'bloom6_size', 'bloom6_tint',
    'bloom_convolution_texture', 'bloom_convolution_size', 'bloom_convolution_center_uv',
    'bloom_convolution_pre_filter_min', 'bloom_convolution_pre_filter_max',
    'bloom_convolution_pre_filter_mult', 'bloom_convolution_buffer_scale',
    'bloom_convolution_scatter_dispersion', 'bloom_dirt_mask',
    'bloom_dirt_mask_intensity', 'bloom_dirt_mask_tint',
    'scene_fringe_intensity', 'chromatic_aberration_start_offset', 'vignette_intensity',
    'motion_blur_amount', 'motion_blur_max', 'motion_blur_per_object_size',
    'motion_blur_target_fps',
)
POSTPROCESS_VOLUME_FIELDS = ('priority', 'blend_weight', 'blend_radius', 'unbound', 'enabled')


def require(value, message):
    if not value:
        raise RuntimeError(message)
    return value


def property_snapshot(obj, name):
    # Reflected structs borrow storage from their UObject owner. Keep an owned
    # copy before LoadLevel unloads that owner (otherwise vectors can become 0).
    value = obj.get_editor_property(name)
    return value.copy() if isinstance(value, unreal.StructBase) else value


def save(asset):
    require(ASSETS.save_loaded_asset(asset, only_if_is_dirty=False), f'Cannot save {asset}')


def duplicate(source, name):
    path = OUTPUT + '/' + name
    return unreal.load_asset(path) or require(ASSETS.duplicate_asset(source.get_path_name(), path), path)


def compensate_stars_without_sun(source, material, gain, visual_angle_degrees=3.0):
    """Adapt stars to exposure and the Sun disk to its remaining exposure gap."""
    def inputs(node, owner):
        return dict(zip(map(str, LIB.get_material_expression_input_names(node)),
                        LIB.get_inputs_for_material_expression(owner, node)))

    source_output = LIB.get_material_property_input_node(source, unreal.MaterialProperty.MP_EMISSIVE_COLOR)
    require(isinstance(source_output, unreal.MaterialExpressionAdd), 'Unexpected source star output')
    source_inputs = inputs(source_output, source)
    disk_inputs = [key for key, node in source_inputs.items()
                   if isinstance(node, unreal.MaterialExpressionSkyAtmosphereLightDiskLuminance)]
    require(len(disk_inputs) == 1, 'Expected one atmospheric Sun disk in source star material')
    disk_input = disk_inputs[0]
    star_input = 'B' if disk_input == 'A' else 'A'
    require(source_inputs.get(star_input), 'Missing source star branch')
    nodes = {node.get_name(): node for node in unreal.ObjectIterator(unreal.MaterialExpression)
             if node.get_outer() == material}
    def scalar_parameter(name, default_value, x, y):
        matches = [node for node in nodes.values()
                   if isinstance(node, unreal.MaterialExpressionScalarParameter)
                   and str(node.get_editor_property('parameter_name')) == name]
        require(len(matches) <= 1, f'Duplicate star material parameter: {name}')
        node = matches[0] if matches else LIB.create_material_expression(
            material, unreal.MaterialExpressionScalarParameter, x, y)
        node.set_editor_property('parameter_name', name)
        node.set_editor_property('default_value', default_value)
        node.set_editor_property('group', 'BSK Sun visual adaptation')
        return node

    output = require(nodes.get(source_output.get_name()), 'Missing preserved star output')
    stars = require(nodes.get(source_inputs[star_input].get_name()), 'Missing preserved star branch')
    disk = require(nodes.get(source_inputs[disk_input].get_name()), 'Missing preserved Sun disk')
    factors = [node for node in nodes.values()
               if isinstance(node, unreal.MaterialExpressionScalarParameter)
               and str(node.get_editor_property('parameter_name')) == 'BSKEmissiveGain']
    require(len(factors) <= 1, 'Duplicate star exposure parameters')
    factor = factors[0] if factors else LIB.create_material_expression(
        material, unreal.MaterialExpressionScalarParameter, 1450, 250)
    factor.set_editor_property('parameter_name', 'BSKEmissiveGain')
    factor.set_editor_property('default_value', gain)
    factor.set_editor_property('group', 'BSK exposure adaptation')
    multipliers = [node for node in nodes.values()
                   if isinstance(node, unreal.MaterialExpressionMultiply)
                   and inputs(node, material).get('B') == factor]
    require(len(multipliers) <= 1, 'Duplicate star exposure multipliers')
    multiply = multipliers[0] if multipliers else LIB.create_material_expression(
        material, unreal.MaterialExpressionMultiply, 1650, 100)
    # Also repairs the old (stars + physical Sun disk) * gain graph in place.
    require(LIB.connect_material_expressions(stars, '', multiply, 'A'), 'star radiance input')
    require(LIB.connect_material_expressions(factor, '', multiply, 'B'), 'star exposure gain')
    require(LIB.connect_material_expressions(multiply, '', output, star_input), 'compensated star branch')
    # Keep the authored disk size independent of the physical source angle used
    # by shadows/eclipses. Runtime supplies the physical/visual solid-angle ratio
    # so making the visible disk larger does not brighten it a second time.
    require(math.isfinite(visual_angle_degrees) and visual_angle_degrees > 0, 'Invalid visual Sun angle')
    visual_angle = scalar_parameter('BSKSunVisualAngularDiameter', visual_angle_degrees, 1450, 450)
    radiance_scale = scalar_parameter('BSKSunDiskRadianceScale', 1.0, 1450, 650)
    # Atmospheric disk luminance already contains the physical Sun lux ratio.
    # Apply only the remaining exposure correction, separately from runtime's
    # solid-angle normalization. Never multiply the disk by the full star gain.
    disk_exposure_gain = gain / SUN_ILLUMINANCE_GAIN
    exposure_gain = scalar_parameter('BSKSunVisualExposureGain', disk_exposure_gain, 1650, 850)
    override_inputs = [name for name in inputs(disk, material)
                       if name.replace(' ', '').lower() == 'diskangulardiameteroverride']
    require(len(override_inputs) == 1, 'Missing Sun disk angular diameter override input')
    require(LIB.connect_material_expressions(visual_angle, '', disk, override_inputs[0]), 'visual Sun diameter')
    disk_multipliers = [node for node in nodes.values()
                        if isinstance(node, unreal.MaterialExpressionMultiply)
                        and inputs(node, material).get('B') == radiance_scale]
    require(len(disk_multipliers) <= 1, 'Duplicate Sun disk radiance multipliers')
    disk_multiply = disk_multipliers[0] if disk_multipliers else LIB.create_material_expression(
        material, unreal.MaterialExpressionMultiply, 1650, 500)
    require(LIB.connect_material_expressions(disk, '', disk_multiply, 'A'), 'physical Sun disk input')
    require(LIB.connect_material_expressions(radiance_scale, '', disk_multiply, 'B'), 'Sun disk solid-angle compensation')
    exposure_multipliers = [node for node in nodes.values()
                            if isinstance(node, unreal.MaterialExpressionMultiply)
                            and inputs(node, material).get('B') == exposure_gain]
    require(len(exposure_multipliers) <= 1, 'Duplicate Sun disk exposure multipliers')
    exposure_multiply = exposure_multipliers[0] if exposure_multipliers else LIB.create_material_expression(
        material, unreal.MaterialExpressionMultiply, 1850, 500)
    require(LIB.connect_material_expressions(disk_multiply, '', exposure_multiply, 'A'), 'Sun disk radiance input')
    require(LIB.connect_material_expressions(exposure_gain, '', exposure_multiply, 'B'), 'Sun disk exposure compensation')
    require(LIB.connect_material_expressions(exposure_multiply, '', output, disk_input), 'separate Sun disk branch')
    require(LIB.connect_material_property(output, '', unreal.MaterialProperty.MP_EMISSIVE_COLOR), 'star output')
    LIB.recompile_material(material)
    save(material)
    REPORT['star_emissive_compensation'] = {
        'gain': gain, 'scope': 'Authored star branch only; atmospheric Sun disk excluded',
        'output': output.get_name(), 'star_multiplier': multiply.get_name(),
        'sun_disk': disk.get_name(), 'sun_disk_multiplier': disk_multiply.get_name(),
        'sun_disk_exposure_multiplier': exposure_multiply.get_name(),
        'sun_visual_exposure_gain': disk_exposure_gain,
        'sun_visual_angular_diameter_degrees': visual_angle_degrees,
        'sun_disk_radiance_scale_preview': 1.0,
    }
    return material


def compensated_master(source, name, gain):
    """Keep diffuse response physical; compensate only authored emissive radiance."""
    material = duplicate(source, name)
    if source.get_path_name().split('.')[0] == '/Game/IncredibleEarth/Materials/Masters/M_Stars':
        return compensate_stars_without_sun(source, material, gain)
    if 'BSKEmissiveGain' not in [str(n) for n in LIB.get_scalar_parameter_names(material)]:
        # Export suffixes are not stable: the source Moon uses _2, Earth _0.
        # Inspect the actual emissive connection instead of a generated name.
        attribute_inputs = []
        for node in unreal.ObjectIterator(unreal.MaterialExpressionMakeMaterialAttributes):
            if node.get_outer() != material:
                continue
            input_names = list(map(str, LIB.get_material_expression_input_names(node)))
            input_nodes = LIB.get_inputs_for_material_expression(material, node)
            emissive = dict(zip(input_names, input_nodes)).get('EmissiveColor')
            if emissive:
                attribute_inputs.append((node, emissive))
        require(len(attribute_inputs) <= 1, f'Multiple emissive attribute branches require explicit handling: {material}')
        make = attribute_inputs[0][0] if attribute_inputs else None
        if make:
            original = attribute_inputs[0][1]
            output = ''
        else:
            original = LIB.get_material_property_input_node(material, unreal.MaterialProperty.MP_EMISSIVE_COLOR)
            output = LIB.get_material_property_input_node_output_name(material, unreal.MaterialProperty.MP_EMISSIVE_COLOR)
        if not original:
            save(material)
            return material
        factor = LIB.create_material_expression(material, unreal.MaterialExpressionScalarParameter, 1450, 250)
        factor.set_editor_property('parameter_name', 'BSKEmissiveGain')
        factor.set_editor_property('default_value', gain)
        factor.set_editor_property('group', 'BSK exposure adaptation')
        multiply = LIB.create_material_expression(material, unreal.MaterialExpressionMultiply, 1650, 100)
        require(LIB.connect_material_expressions(original, output, multiply, 'A'), 'emissive input')
        require(LIB.connect_material_expressions(factor, '', multiply, 'B'), 'emissive gain')
        if make:
            require(LIB.connect_material_expressions(multiply, '', make, 'EmissiveColor'), 'material attributes')
        else:
            require(LIB.connect_material_property(multiply, '', unreal.MaterialProperty.MP_EMISSIVE_COLOR), 'material output')
    else:
        factors = [node for node in unreal.ObjectIterator(unreal.MaterialExpressionScalarParameter)
                   if node.get_outer() == material
                   and str(node.get_editor_property('parameter_name')) == 'BSKEmissiveGain']
        require(len(factors) == 1, f'Expected exactly one emissive gain in {material}')
        # Aurora uses this master directly, so updating only baked instances
        # would leave its previous calibration in place on a repeated run.
        factors[0].set_editor_property('default_value', gain)
    LIB.recompile_material(material)
    save(material)
    return material


def bake(source, master_name, instance_name, gain):
    parent = source
    while not isinstance(parent, unreal.Material):
        parent = parent.get_editor_property('parent')
    master = compensated_master(parent, master_name, gain)
    path = OUTPUT + '/' + instance_name
    result = unreal.load_asset(path)
    if not result:
        result = require(unreal.AssetToolsHelpers.get_asset_tools().create_asset(
            instance_name, OUTPUT, unreal.MaterialInstanceConstant,
            unreal.MaterialInstanceConstantFactoryNew()), path)
    LIB.set_material_instance_parent(result, master)
    copied = {}
    for kind in ['scalar', 'vector', 'texture', 'static_switch']:
        names = getattr(LIB, f'get_{kind}_parameter_names')(source)
        getter = getattr(LIB, f'get_material_instance_{kind}_parameter_value')
        setter = getattr(LIB, f'set_material_instance_{kind}_parameter_value')
        for name in names:
            if isinstance(source, unreal.MaterialInstanceDynamic):
                if kind == 'static_switch':
                    static_parent = source.get_editor_property('parent')
                    if isinstance(static_parent, unreal.Material):
                        value = LIB.get_material_default_static_switch_parameter_value(static_parent, name)
                    else:
                        value = getter(static_parent, name)
                else:
                    value = getattr(source, f'get_{kind}_parameter_value')(name)
            else:
                value = getter(source, name)
            if value is not None:
                # UE 5.6's scalar/vector/texture setters always return false.
                # Read back the resulting value instead of trusting that flag.
                setter(result, name, value)
                actual = getter(result, name)
                if kind == 'scalar': require(math.isclose(actual, value, rel_tol=1e-6, abs_tol=1e-6), f'{name}: {actual} != {value}')
                elif kind == 'static_switch': require(actual == value, f'Static switch {name}')
                elif kind == 'texture': require(actual == value, f'Texture {name}')
                elif kind == 'vector':
                    require(all(math.isclose(getattr(actual,k),getattr(value,k),rel_tol=1e-6,abs_tol=1e-6) for k in 'rgba'), f'Vector {name}')
        copied[kind] = len(names)
    compensation_applied = 'BSKEmissiveGain' in [str(n) for n in LIB.get_scalar_parameter_names(result)]
    if compensation_applied:
        LIB.set_material_instance_scalar_parameter_value(result, 'BSKEmissiveGain', gain)
        require(math.isclose(LIB.get_material_instance_scalar_parameter_value(result, 'BSKEmissiveGain'), gain, rel_tol=1e-6), 'Emissive gain readback')
    if 'BSKSunVisualExposureGain' in [str(n) for n in LIB.get_scalar_parameter_names(result)]:
        disk_exposure_gain = gain / SUN_ILLUMINANCE_GAIN
        LIB.set_material_instance_scalar_parameter_value(result, 'BSKSunVisualExposureGain', disk_exposure_gain)
        require(math.isclose(LIB.get_material_instance_scalar_parameter_value(result, 'BSKSunVisualExposureGain'),
                             disk_exposure_gain, rel_tol=1e-6), 'Sun disk exposure gain readback')
    LIB.update_material_instance(result)
    save(result)
    REPORT['materials'][instance_name] = {'source': source.get_path_name(), 'parameters': copied,
        'emissive_compensation_applied': compensation_applied,
        'emissive_gain': LIB.get_material_instance_scalar_parameter_value(result, 'BSKEmissiveGain') if compensation_applied else None}
    return result


def material_lengths(source):
    """Snapshot authored lengths before unloading the source level."""
    names = {str(n) for n in LIB.get_scalar_parameter_names(source)}
    values = {}
    for name in ['EarthSize', 'Displacement']:
        if name in names:
            if isinstance(source, unreal.MaterialInstanceDynamic):
                values[name] = source.get_scalar_parameter_value(name)
            else:
                values[name] = LIB.get_material_instance_scalar_parameter_value(source, name)
    return values


def scale_material_lengths(material, source_values, enlargement):
    """Scale spatial masks/WPO from the source values, never from a prior run."""
    adapted = {}
    for name, original in source_values.items():
        value = original * enlargement
        LIB.set_material_instance_scalar_parameter_value(material, name, value)
        actual = LIB.get_material_instance_scalar_parameter_value(material, name)
        require(math.isclose(actual, value, rel_tol=1e-6, abs_tol=1e-6), f'{name}: {actual} != {value}')
        adapted[name] = {'source': original, 'adapted': actual}
    LIB.update_material_instance(material)
    save(material)
    REPORT['materials'].setdefault(material.get_name(), {})['length_parameters'] = adapted


def compensate_aurora_component_scale(material):
    """Restore the source component-scale factor after local-space conversion.

    Attached Niagara particles carry the planet scale themselves while their
    component has unit scale. Keep the authored world-to-local transform and
    ObjectScale multiplication; CPD 0 supplies the missing component scale.
    """
    source = require(unreal.load_asset('/Game/IncredibleEarth/Materials/Masters/M_AuroraNiagaraBase'), 'Aurora source material')
    source_transform = require(unreal.find_object(source, 'MaterialExpressionTransform_3'), 'Aurora source transform')
    transform = require(unreal.find_object(material, 'MaterialExpressionTransform_3'), 'Aurora local transform')
    multiply = require(unreal.find_object(material, 'MaterialExpressionMultiply_49'), 'Aurora ObjectScale multiplication')
    for name in ['transform_source_type', 'transform_type']:
        transform.set_editor_property(name, source_transform.get_editor_property(name))
    nodes = [n for n in unreal.ObjectIterator(unreal.MaterialExpression) if n.get_outer() == material]
    factor = next((n for n in nodes if isinstance(n, unreal.MaterialExpressionScalarParameter)
                   and str(n.get_editor_property('parameter_name')) == 'BSKAuroraSystemScale'), None)
    if factor is None:
        factor = LIB.create_material_expression(material, unreal.MaterialExpressionScalarParameter, -11300, 200)
    factor.set_editor_property('parameter_name', 'BSKAuroraSystemScale')
    factor.set_editor_property('default_value', 1.0)
    factor.set_editor_property('use_custom_primitive_data', True)
    factor.set_editor_property('primitive_data_index', 0)
    factor.set_editor_property('group', 'BSK scale adaptation')
    description = 'BSK: restore source Niagara component scale'
    divide = next((n for n in nodes if isinstance(n, unreal.MaterialExpressionDivide)
                   and n.get_editor_property('desc') == description), None)
    if divide is None:
        divide = LIB.create_material_expression(material, unreal.MaterialExpressionDivide, -11000, 100)
        divide.set_editor_property('desc', description)
    require(LIB.connect_material_expressions(transform, '', divide, 'A'), 'Aurora source-space position')
    require(LIB.connect_material_expressions(factor, '', divide, 'B'), 'Aurora component-scale divisor')
    require(LIB.connect_material_expressions(divide, '', multiply, 'A'), 'Aurora scale compensation')
    LIB.recompile_material(material)
    save(material)


def scale_aurora_lengths(material, enlargement):
    """Preserve authored angular texture coverage on the enlarged aurora."""
    require(math.isfinite(enlargement) and enlargement > 0, 'Invalid aurora enlargement')
    source = require(unreal.load_asset('/Game/IncredibleEarth/Materials/Masters/M_AuroraNiagaraBase'), 'Aurora source material')
    require(material != source, 'Aurora size adaptation requires a material copy')
    adapted = {}
    for name in ['MaterialExpressionConstant_2', 'MaterialExpressionConstant_17']:
        source_node = require(unreal.find_object(source, name), f'Aurora source TextureSize: {name}')
        target_node = require(unreal.find_object(material, name), f'Aurora adapted TextureSize: {name}')
        original = source_node.get_editor_property('r')
        value = original * enlargement
        target_node.set_editor_property('r', value)
        actual = target_node.get_editor_property('r')
        require(math.isclose(actual, value, rel_tol=1e-6, abs_tol=1e-6), f'{name}: {actual} != {value}')
        adapted[name] = {'source': original, 'adapted': actual}
    LIB.recompile_material(material)
    save(material)
    REPORT['materials'].setdefault(material.get_name(), {})['length_constants'] = adapted
    return adapted


def scale_aurora_speeds(source_values, enlargement):
    """Keep angular animation speed after division by the larger system scale."""
    require(math.isfinite(enlargement) and enlargement > 0, 'Invalid aurora enlargement')
    adapted = dict(source_values)
    for name in ['Speed', 'Speed2']:
        require(name in source_values, f'Missing source aurora {name}')
        adapted[name] = source_values[name] * enlargement
    return adapted


def snapshot_source_postprocess(source_actors):
    """Capture enabled artistic overrides in the source actor iteration order."""
    snapshots = []
    actors = source_actors.values() if hasattr(source_actors, 'values') else source_actors
    for actor in actors:
        if not isinstance(actor, unreal.PostProcessVolume):
            continue
        settings = actor.get_editor_property('settings')
        fields = {}
        for name in ARTISTIC_POSTPROCESS_FIELDS:
            override = 'override_' + name
            if (hasattr(settings, name) and hasattr(settings, override)
                    and settings.get_editor_property(override)):
                fields[name] = property_snapshot(settings, name)
        snapshots.append({
            'source_name': actor.get_name(),
            'source_label': actor.get_actor_label(),
            'properties': {n: actor.get_editor_property(n) for n in POSTPROCESS_VOLUME_FIELDS},
            'settings': fields,
        })
    return snapshots


def apply_source_postprocess(snapshots):
    """Create/reuse tagged art volumes without editing platform exposure volumes."""
    actors = list(ACTORS.get_all_level_actors())
    applied = []
    for order, snapshot in enumerate(snapshots):
        tag = 'BSK.IncredibleEarth80K.PostProcess.' + snapshot['source_name']
        matches = [a for a in actors if tag in [str(t) for t in a.get_editor_property('tags')]]
        require(len(matches) <= 1, f'Duplicate migrated postprocess tag: {tag}')
        if matches:
            actor = matches[0]
            require(isinstance(actor, unreal.PostProcessVolume), f'Unexpected actor for {tag}')
        else:
            # Source volumes have equal priority: creation order preserves
            # their authored blend order (main volume, then ColorCorrection).
            actor = require(ACTORS.spawn_actor_from_class(
                unreal.PostProcessVolume, unreal.Vector(), unreal.Rotator()), tag)
            actors.append(actor)
            actor.set_editor_property('tags', [tag])
        settings = unreal.PostProcessSettings()
        for name, value in snapshot['settings'].items():
            require(name in ARTISTIC_POSTPROCESS_FIELDS, f'Non-artistic postprocess override: {name}')
            override = 'override_' + name
            require(hasattr(settings, name) and hasattr(settings, override), f'Unsupported postprocess field: {name}')
            settings.set_editor_property(name, value)
            settings.set_editor_property(override, True)
        # Fresh settings also remove stale overrides on our own reused actors.
        actor.set_editor_property('settings', settings)
        for name in POSTPROCESS_VOLUME_FIELDS:
            actor.set_editor_property(name, snapshot['properties'][name])
        actor.set_actor_label('IncredibleEarth80K - ' + snapshot['source_label'])
        applied.append({'source_name': snapshot['source_name'], 'target_actor': actor.get_name(),
                        'tag': tag, 'source_order': order, 'properties': snapshot['properties'],
                        'enabled_artistic_overrides': list(snapshot['settings'])})
    REPORT['postprocess_volumes'] = applied
    return applied


def layer(component, mesh, material, radius_cm, celestial=True, root_rotation=None):
    # The source sphere is centred, unlike the platform's former pole-pivot mesh.
    # Derive scale from measured geometry, never from an assumed UE unit sphere.
    bounds = mesh.get_bounds()
    require(bounds.sphere_radius > 0, f'Invalid sphere bounds: {mesh}')
    scale = radius_cm / bounds.sphere_radius
    is_root = component == component.get_owner().get_editor_property('root_component')
    center = None
    if is_root:
        old_mesh = component.get_editor_property('static_mesh')
        old_origin = old_mesh.get_bounds().origin if old_mesh else unreal.Vector()
        center = unreal.MathLibrary.transform_location(component.get_world_transform(), old_origin)
    component.set_static_mesh(mesh)
    component.set_material(0, material)
    component.set_mobility(unreal.ComponentMobility.MOVABLE)
    if is_root:
        # A StaticMeshActor's root transform is the actor transform: resetting
        # it would move the Moon to Earth and erase the star dome orientation.
        if root_rotation is not None:
            component.set_world_rotation(root_rotation, False, True)
        component.set_world_scale3d(unreal.Vector(scale, scale, scale))
        new_center = unreal.MathLibrary.transform_location(component.get_world_transform(), bounds.origin)
        component.set_world_location(component.get_world_location() + center - new_center, False, True)
    else:
        component.set_relative_rotation(unreal.Rotator(), False, True)
        component.set_relative_scale3d(unreal.Vector(scale, scale, scale))
        component.set_relative_location(unreal.Vector(-bounds.origin.x * scale, -bounds.origin.y * scale, -bounds.origin.z * scale), False, True)
    component.set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)
    component.set_cast_shadow(False)
    component.set_editor_property('affect_distance_field_lighting', False)
    component.set_editor_property('visible_in_ray_tracing', False)
    component.set_visibility(True)
    component.set_hidden_in_game(False)
    if celestial:
        component.set_editor_property('lighting_channels', unreal.LightingChannels(channel0=False, channel1=True, channel2=False))
    return scale


def apply_moon_fill(snapshot, moon_actor, earth, gain):
    """Keep the source Moon-attached artistic fill isolated from spacecraft."""
    tag = 'BSK.IncredibleEarth80K.MoonLight'
    matches = [a for a in ACTORS.get_all_level_actors() if tag in [str(t) for t in a.tags]]
    require(len(matches) <= 1, 'Duplicate IncredibleEarth MoonLight')
    actor = matches[0] if matches else ACTORS.spawn_actor_from_class(unreal.DirectionalLight, unreal.Vector())
    actor.set_editor_property('tags', [tag])
    actor.set_actor_label('IncredibleEarth80K - MoonLight - Earth only')
    light = actor.get_component_by_class(unreal.DirectionalLightComponent)
    light.set_mobility(unreal.ComponentMobility.MOVABLE)
    actor.attach_to_actor(moon_actor, '', unreal.AttachmentRule.KEEP_WORLD,
                          unreal.AttachmentRule.KEEP_WORLD, unreal.AttachmentRule.KEEP_WORLD, False)
    light.set_absolute(False, False, True)
    light.set_world_scale3d(unreal.Vector(1, 1, 1))
    actor.set_actor_relative_location(unreal.Vector(), False, True)
    actor.set_actor_relative_rotation(snapshot['relative_rotation'], False, True)
    for name, value in snapshot['properties'].items():
        light.set_editor_property(name, value * gain if name == 'intensity' else value)
    light.set_editor_property('lighting_channels', unreal.LightingChannels(channel0=False, channel1=False, channel2=True))
    # Source Earth received this artistic fill; its Moon and the platform's
    # spacecraft must not. Channel 2 is reserved here for the Earth layers.
    for name in ['surface', 'clouds', 'atmosphere']:
        earth.get_editor_property(name).set_editor_property('lighting_channels',
            unreal.LightingChannels(channel0=False, channel1=True, channel2=True))
    REPORT['moon_fill'] = {'actor': actor.get_name(), 'source_intensity': snapshot['properties']['intensity'],
                           'adapted_intensity': light.get_editor_property('intensity'), 'earth_only_channel': 2}


def migrate():
    require(LEVELS.load_level(SOURCE_MAP), SOURCE_MAP)
    source_actors = {a.get_actor_label(): a for a in ACTORS.get_all_level_actors()}
    postprocess_snapshots = snapshot_source_postprocess(source_actors)
    source_earth = source_actors['Earth']
    components = {c.get_name(): c for c in source_earth.get_components_by_class(unreal.StaticMeshComponent)}
    sun = source_actors['Sun'].get_component_by_class(unreal.DirectionalLightComponent)
    source_sun_lux = sun.get_editor_property('intensity')
    # Emissive radiance follows the measured camera-exposure ratio, not the
    # independent solar illuminance ratio used by the physical lit surfaces.
    gain = EMISSIVE_EXPOSURE_GAIN
    illumination_gain = TARGET_SUN_ILLUMINANCE_LUX / source_sun_lux
    require(math.isclose(source_sun_lux, SOURCE_SUN_ILLUMINANCE_LUX, rel_tol=1e-6),
            'Source Sun intensity differs from the measured exposure calibration')
    REPORT['exposure_calibration'] = {
        'source_exposure': SOURCE_REFERENCE_EXPOSURE,
        'target_exposure': TARGET_REFERENCE_EXPOSURE,
        'emissive_gain': gain, 'sun_illuminance_gain': illumination_gain,
        'sun_visual_exposure_gain': gain / illumination_gain,
    }
    ground = bake(components['Earth'].get_material(0), 'M_Earth80K_Physical', 'MI_Earth80K_Main', gain)
    clouds = bake(components['Clouds'].get_material(0), 'M_Clouds_Physical', 'MI_Clouds_Main', gain)
    ground_lengths = material_lengths(components['Earth'].get_material(0))
    cloud_lengths = material_lengths(components['Clouds'].get_material(0))
    night = bake(components['Atmosphere'].get_material(0), 'M_NightAtmosphere_Physical', 'MI_NightAtmosphere_Main', gain)
    meshes = {n: c.get_editor_property('static_mesh') for n, c in components.items()}
    source_radius_cm = meshes['Earth'].get_bounds().sphere_radius * components['Earth'].get_world_scale().x
    cloud_ratio = components['Clouds'].get_world_scale().x / components['Earth'].get_world_scale().x
    edge_ratio = (meshes['Atmosphere'].get_bounds().sphere_radius * components['Atmosphere'].get_world_scale().x) / source_radius_cm
    stars_component = source_actors['StarDome'].get_component_by_class(unreal.StaticMeshComponent)
    stars_mesh = stars_component.get_editor_property('static_mesh')
    stars = bake(stars_component.get_material(0), 'M_Stars_Physical', 'MI_Stars_Main', gain)
    stars_rotation = source_actors['StarDome'].get_actor_rotation()
    moon_component = source_actors['Moon'].get_component_by_class(unreal.StaticMeshComponent)
    source_fill = source_actors['MoonLight'].get_component_by_class(unreal.DirectionalLightComponent)
    moon_fill = {
        'relative_rotation': unreal.MathLibrary.inverse_transform_rotation(
            moon_component.get_world_transform(), source_fill.get_world_rotation()),
        'properties': {n: property_snapshot(source_fill, n) for n in [
            'intensity', 'light_color', 'use_temperature', 'temperature',
            'light_source_angle', 'light_source_soft_angle', 'atmosphere_sun_light',
            'cast_shadows', 'cast_translucent_shadows', 'indirect_lighting_intensity',
            'volumetric_scattering_intensity',
        ]},
    }
    moon_mesh = moon_component.get_editor_property('static_mesh')
    moon = bake(moon_component.get_material(0), 'M_Moon_Physical', 'MI_Moon_Main', gain)
    aurora_master = compensated_master(unreal.load_asset('/Game/IncredibleEarth/Materials/Masters/M_AuroraNiagaraBase'), 'M_Aurora_Physical', gain)
    compensate_aurora_component_scale(aurora_master)
    aurora = duplicate(unreal.load_asset('/Game/IncredibleEarth/VFX/NS_AuroraFromTexture'), 'NS_Aurora_Attached')
    require(unreal.BskEnvironmentBody.prepare_attached_aurora_asset(aurora, aurora_master), 'Aurora local simulation conversion')
    save(aurora)
    aurora_scalars = {}
    for name, variable in {
        'AuroraBottomAltitude':'AuroraBottomAltitude', 'AuroraHeight':'AuroraHeight',
        'Brightness':'AuroraBrightness', 'ContrastBottom':'AuroraContrastBottom',
        'ContrastTop':'AuroraContrastTop', 'ManualOffsetX':'AuroraManualOffsetX',
        'ManualOffsetY':'AuroraManualOffsetY', 'MaskScale':'AuroraMaskScale',
        'MaskPoleCut':'AuroraMaskPoleCut', 'Speed':'AuroraSpeed', 'Speed2':'AuroraSpeed2',
        'UVscale':'AuroraUVscale', 'UVscale2':'AuroraUVscale2', 'TimeOffset':'AuroraTimeOffset',
    }.items():
        aurora_scalars[name] = source_earth.get_editor_property(variable)
    aurora_colors = {n: property_snapshot(source_earth, n) for n in ['AuroraAccentColor','AuroraMainColor']}
    sky = source_actors['SkyAtmosphere'].get_component_by_class(unreal.SkyAtmosphereComponent)
    source_sky_bottom_radius_km = sky.get_editor_property('bottom_radius')
    sky_properties = {n: property_snapshot(sky, n) for n in [
        'atmosphere_height','rayleigh_scattering','rayleigh_scattering_scale','rayleigh_exponential_distribution',
        'mie_scattering','mie_scattering_scale','mie_absorption','mie_absorption_scale','mie_anisotropy',
        'mie_exponential_distribution','other_absorption','other_absorption_scale','other_tent_distribution',
        'multi_scattering_factor','sky_and_aerial_perspective_luminance_factor','trace_sample_count_scale',
        'aerial_perspective_start_depth',
    ]}
    sun_properties = {n:property_snapshot(sun, n) for n in [
        'light_source_angle','light_source_soft_angle','atmosphere_sun_disk_color_scale','enable_light_shaft_bloom','bloom_scale','bloom_threshold','bloom_max_brightness','bloom_tint',
        'use_temperature','temperature','cast_translucent_shadows',
    ]}
    require(LEVELS.load_level(TARGET_MAP), TARGET_MAP)
    rigs = [a for a in ACTORS.get_all_level_actors() if isinstance(a, unreal.BskEnvironment)]
    require(len(rigs) == 1, 'Exactly one saved BskEnvironment is required')
    rig = rigs[0]
    apply_source_postprocess(postprocess_snapshots)
    earth = rig.get_editor_property('earth')
    radius_m = rig.get_editor_property('earth_radius_meters')
    cm_per_m = rig.get_editor_property('geometry_centimeters_per_meter')
    require(abs(radius_m - 6371000) < 0.01, f'Unexpected physical Earth radius: {radius_m}')
    radius_cm = radius_m * cm_per_m
    enlargement = radius_cm / source_radius_cm
    scale_material_lengths(ground, ground_lengths, enlargement)
    scale_material_lengths(clouds, cloud_lengths, enlargement)
    scale_aurora_lengths(aurora_master, enlargement)
    REPORT['aurora_source_parameters'] = dict(aurora_scalars)
    aurora_scalars = scale_aurora_speeds(aurora_scalars, enlargement)
    earth.set_actor_scale3d(unreal.Vector(1, 1, 1))
    for field, mesh, material, ratio in [
        ('surface',meshes['Earth'],ground,1), ('clouds',meshes['Clouds'],clouds,cloud_ratio),
        ('atmosphere',meshes['Atmosphere'],night,edge_ratio),
    ]:
        layer(earth.get_editor_property(field), mesh, material, radius_cm * ratio)
    effect = earth.get_editor_property('aurora')
    effect.set_asset(aurora)
    effect.set_absolute(False, False, True)
    effect.set_relative_scale3d(unreal.Vector(1, 1, 1))
    effect.set_editor_property('auto_activate', True)
    effect.set_editor_property('lighting_channels', unreal.LightingChannels(channel0=False, channel1=True, channel2=False))
    for n, v in aurora_scalars.items(): effect.set_variable_float('User.'+n, v)
    for n, v in aurora_colors.items(): effect.set_variable_linear_color('User.'+n, v)
    earth.refresh_visual_scale()
    # The source emitter leaves infinitely lived particles when inactive.
    # Recreate its preview instance so repeated migration cannot stack bursts.
    effect.reinitialize_system()
    star_actor = rig.get_editor_property('star_sphere')
    layer(star_actor.static_mesh_component, stars_mesh, stars, 50000000000.0, False,
          root_rotation=stars_rotation)
    for actor in ACTORS.get_all_level_actors():
        if 'BSK.Semantic.moon' in [str(t) for t in actor.get_editor_property('tags')]:
            component = actor.get_component_by_class(unreal.StaticMeshComponent)
            if component and component.get_editor_property('static_mesh'):
                # Use the same physical lunar radius in preview and simulation.
                # layer() retains the existing world centre and orientation.
                layer(component, moon_mesh, moon, MOON_RADIUS_METERS * cm_per_m)
                actor.set_actor_label('Moon - IncredibleEarth80K - backend ephemeris')
                REPORT['moon_actor'] = actor.get_name()
                REPORT['moon_preview_radius_m'] = MOON_RADIUS_METERS
                apply_moon_fill(moon_fill, actor, earth, illumination_gain)
    sky_actor = rig.get_editor_property('earth_atmosphere')
    target_sky = sky_actor.get_component_by_class(unreal.SkyAtmosphereComponent)
    for n, v in sky_properties.items():
        if n in ['atmosphere_height','rayleigh_exponential_distribution','mie_exponential_distribution',
                 'aerial_perspective_start_depth']:
            v *= enlargement
        if n in ['rayleigh_scattering_scale','mie_scattering_scale','mie_absorption_scale','other_absorption_scale']:
            v /= enlargement
        if n == 'other_tent_distribution':
            v.set_editor_property('tip_altitude', v.get_editor_property('tip_altitude') * enlargement)
            v.set_editor_property('width', v.get_editor_property('width') * enlargement)
        target_sky.set_editor_property(n,v)
    # The source atmosphere's bottom sits slightly inside the surface mesh.
    # Preserve that ratio rather than moving its lower boundary to the surface.
    target_sky.set_bottom_radius(source_sky_bottom_radius_km * enlargement)
    target_sky.set_editor_property('transform_mode', unreal.SkyAtmosphereTransformMode.PLANET_CENTER_AT_COMPONENT_TRANSFORM)
    sky_actor.set_actor_location(earth.get_actor_location(), False, True)
    # IncredibleEarth's Sun is the directional light's atmospheric disk. Keep
    # the platform's bound proxy actor for compatibility, with its old FX off.
    sun_actor = rig.get_editor_property('sun_visual')
    # A lightweight hidden binding avoids retaining the former Sun material's
    # large texture dependencies in the migrated environment.
    sun_actor.get_editor_property('surface').set_static_mesh(unreal.load_asset('/Engine/BasicShapes/Sphere'))
    sun_actor.get_editor_property('surface').set_material(0,unreal.load_asset('/Engine/EngineMaterials/DefaultMaterial'))
    for name in ['surface','clouds','atmosphere','sun_bursts','sun_halo','sun_lines']:
        c = sun_actor.get_editor_property(name)
        c.set_visibility(False)
        c.set_hidden_in_game(True)
        if isinstance(c,unreal.ParticleSystemComponent):
            c.deactivate()
            c.set_editor_property('auto_activate',False)
            c.set_template(None)
    target_sun = rig.get_editor_property('sun_light').get_component_by_class(unreal.DirectionalLightComponent)
    for n,v in sun_properties.items(): target_sun.set_editor_property(n,v)
    target_sun.set_editor_property('atmosphere_sun_light',True)
    target_sun.set_editor_property('atmosphere_sun_light_index',0)
    # Preserve the source's authored preview disk size. The backend still owns
    # physical angular diameter when ephemeris takes control at runtime.
    rig.set_editor_property('sun_visual_angular_diameter_degrees',sun_properties['light_source_angle'])
    tags = list(earth.get_editor_property('tags'))
    if 'BSK.IncredibleEarth80K' not in [str(t) for t in tags]: tags.append('BSK.IncredibleEarth80K')
    earth.set_editor_property('tags',tags)
    earth.set_actor_label('Earth - IncredibleEarth80K - radius 6371 km')
    star_actor.set_actor_label('Stars - IncredibleEarth80K main scene')
    REPORT.update(radius_m=radius_m, geometry_cm_per_m=cm_per_m, enlargement=enlargement,
                  cloud_ratio=cloud_ratio, edge_ratio=edge_ratio, earth_center=str(earth.get_actor_location()),
                  atmosphere_height_km=target_sky.get_editor_property('atmosphere_height'),
                  atmosphere_bottom_radius_km=target_sky.get_editor_property('bottom_radius'),
                  atmosphere_bottom_to_surface_ratio=source_sky_bottom_radius_km / (source_radius_cm / 100000.0),
                  aerial_perspective_start_depth_km=target_sky.get_editor_property('aerial_perspective_start_depth'),
                  aurora_parameters=aurora_scalars, sun='Atmospheric disk; physical backend lighting preserved')
    require(LEVELS.save_current_level(), 'Could not save migrated BskEnvironment')
    report_dir=Path(unreal.Paths.project_saved_dir())/'IncredibleEarthMigration'
    report_dir.mkdir(parents=True,exist_ok=True)
    (report_dir/'migration_report.json').write_text(json.dumps(REPORT,indent=2),encoding='utf-8')
    unreal.log('BSK_INCREDIBLE_EARTH_MIGRATED: radius=6371000m; existing rig and backend bindings retained')


if __name__ == '__main__':
    migrate()
