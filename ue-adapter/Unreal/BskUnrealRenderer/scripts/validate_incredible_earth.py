"""Validate migration state without saving assets (UE editor Python).

When the target map is already open, inspect that world, including unsaved edits.
Only refresh transient visual parameters; never reload the active target map.
Artistic values below validate the freshly migrated baseline, not a live session
whose lighting or grading has been intentionally adjusted.
"""
import json
import math
import runpy
from pathlib import Path
import unreal


def verify_components(label, value, expected, fields='rgba'):
    """Check actual struct contents; enabled overrides alone miss lost values."""
    actual = {field: float(getattr(value, field)) for field in fields}
    for field, reference in zip(fields, expected):
        assert math.isclose(actual[field], reference, rel_tol=1e-6, abs_tol=1e-6), (
            f'{label}.{field}: {actual[field]} != source baseline {reference}')
    return actual


MAP = '/Game/BSK/Maps/BskEnvironment'
levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
current_level = levels.get_current_level()
current_map = current_level.get_path_name().split('.')[0] if current_level else None
if current_map != MAP:
    assert not unreal.EditorLoadingAndSavingUtils.get_dirty_map_packages(), (
        'Open BskEnvironment explicitly after preserving unsaved map edits')
    assert levels.load_level(MAP)
actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem).get_all_level_actors()
rigs = [a for a in actors if isinstance(a, unreal.BskEnvironment)]
assert len(rigs) == 1
rig = rigs[0]
bindings = ['earth','star_sphere','sun_visual','sun_light','celestial_sun_light','fill_light',
            'headlight','exposure_volume','earth_atmosphere','preview_camera']
assert all(rig.get_editor_property(n) for n in bindings)
earth = rig.get_editor_property('earth')
assert earth.get_actor_scale3d() == unreal.Vector(1,1,1)
assert math.isclose(rig.get_editor_property('earth_radius_meters'), 6371000, abs_tol=.001)
radius_cm = rig.get_editor_property('earth_radius_meters') * rig.get_editor_property('geometry_centimeters_per_meter')
report = {'map': MAP, 'inspected_existing_world': current_map == MAP,
          'bindings': {n: rig.get_editor_property(n).get_name() for n in bindings}, 'layers': {}}
# Independent camera baseline: source ISO 100, 1/60 s, f/4, +9 EV; target
# ISO 100, 1/125 s, f/16, +0 EV. HDR captures confirmed .53333 and 3.125e-5.
# This correction is deliberately separate from the physical solar lux ratio.
source_reference_exposure = 100.0 / (100.0 * 60.0 * 4.0 ** 2) * 2.0 ** 9
target_reference_exposure = 100.0 / (100.0 * 125.0 * 16.0 ** 2)
expected_emissive_gain = source_reference_exposure / target_reference_exposure
assert math.isclose(expected_emissive_gain, 17066.666666666668, rel_tol=1e-12)
emissive_layers = {}
for layer_name, asset_name in {
    'surface': 'MI_Earth80K_Main', 'clouds': 'MI_Clouds_Main',
    'night_atmosphere': 'MI_NightAtmosphere_Main', 'stars': 'MI_Stars_Main',
    'moon': 'MI_Moon_Main', 'aurora': 'M_Aurora_Physical',
}.items():
    material = unreal.load_asset('/Game/BSK/Environment/IncredibleEarth/' + asset_name)
    assert material, f'Missing emissive layer: {layer_name}'
    master = material
    while not isinstance(master, unreal.Material):
        master = master.get_editor_property('parent')
        assert master, f'Missing parent for {layer_name}'
    factors = [node for node in unreal.ObjectIterator(unreal.MaterialExpressionScalarParameter)
               if node.get_outer() == master
               and str(node.get_editor_property('parameter_name')) == 'BSKEmissiveGain']
    assert len(factors) == 1, f'{layer_name}: expected one master emissive gain'
    default_gain = factors[0].get_editor_property('default_value')
    assert math.isclose(default_gain, expected_emissive_gain, rel_tol=1e-6), (
        f'{layer_name}: stale master emissive gain {default_gain}')
    actual_gain = (default_gain if material == master else
                   unreal.MaterialEditingLibrary.get_material_instance_scalar_parameter_value(material, 'BSKEmissiveGain'))
    assert math.isclose(actual_gain, expected_emissive_gain, rel_tol=1e-6), (
        f'{layer_name}: emissive gain {actual_gain} != exposure ratio {expected_emissive_gain}')
    assert math.isclose(actual_gain * target_reference_exposure, source_reference_exposure, rel_tol=1e-6), layer_name
    emissive_layers[layer_name] = {'material': material.get_path_name(),
                                   'master_gain': default_gain, 'effective_gain': actual_gain}
report['emissive_exposure_calibration'] = {
    'source_reference_exposure': source_reference_exposure,
    'target_reference_exposure': target_reference_exposure,
    'expected_gain': expected_emissive_gain, 'layers': emissive_layers,
}
# Baseline from the source main level, not from the migration's own output:
# Earth component scale 19672 * root scale 5.5; the source masters use EarthSize
# 1e8, and main-level Blueprint overrides set ground/cloud displacement to 0/4.
source_mesh = unreal.load_asset('/Game/IncredibleEarth/Meshes/SM_SphereMid')
assert source_mesh, 'Missing source mesh for independent scale verification'
enlargement = radius_cm / (source_mesh.get_bounds().sphere_radius * 19672.0 * 5.5)
source_lengths = {
    'surface': {'EarthSize': 100000000.0, 'Displacement': 0.0},
    'clouds': {'EarthSize': 100000000.0, 'Displacement': 4.0},
}
for name in ['surface','clouds','atmosphere']:
    c = earth.get_editor_property(name)
    mesh = c.get_editor_property('static_mesh')
    radius = mesh.get_bounds().sphere_radius * c.get_world_scale().x
    assert c.get_material(0).get_path_name().startswith('/Game/BSK/Environment/IncredibleEarth/')
    assert c.is_visible() and not c.get_editor_property('hidden_in_game')
    assert c.get_collision_enabled() == unreal.CollisionEnabled.NO_COLLISION
    if name == 'surface': assert math.isclose(radius,radius_cm,rel_tol=1e-9)
    else: assert radius > radius_cm
    report['layers'][name] = {'radius_cm':radius,'mesh':mesh.get_path_name(),'material':c.get_material(0).get_path_name()}
    if name in source_lengths:
        material = c.get_material(0)
        available = {str(n) for n in unreal.MaterialEditingLibrary.get_scalar_parameter_names(material)}
        lengths = {}
        for parameter, source_value in source_lengths[name].items():
            assert parameter in available, f'{name}: missing {parameter}'
            actual = unreal.MaterialEditingLibrary.get_material_instance_scalar_parameter_value(material, parameter)
            expected = source_value * enlargement
            assert math.isclose(actual, expected, rel_tol=1e-6, abs_tol=1e-6), (
                f'{name}.{parameter}: {actual} != source {source_value} * {enlargement}')
            lengths[parameter] = {'source': source_value, 'expected': expected, 'actual': actual}
        report['layers'][name]['length_parameters'] = lengths
aurora = earth.get_editor_property('aurora')
assert aurora.get_editor_property('asset').get_path_name().startswith('/Game/BSK/Environment/IncredibleEarth/NS_Aurora_Attached')
assert aurora.get_editor_property('absolute_scale')
assert aurora.get_world_scale() == unreal.Vector(1,1,1)
assert aurora.get_editor_property('auto_activate')
earth.refresh_visual_scale()
assert aurora.get_system_fixed_bounds().max.x > radius_cm
surface_scale = earth.get_editor_property('surface').get_world_scale()
surface_mesh_scale = max(abs(surface_scale.x), abs(surface_scale.y), abs(surface_scale.z))
# UE does not expose transient CustomPrimitiveDataInternal to editor Python.
# Validate the material binding here; the native automation test checks actual
# CPD values through backend radius changes and preview restoration.
aurora_material = unreal.load_asset('/Game/BSK/Environment/IncredibleEarth/M_Aurora_Physical')
source_aurora_material = unreal.load_asset('/Game/IncredibleEarth/Materials/Masters/M_AuroraNiagaraBase')
assert aurora_material and source_aurora_material, 'Missing aurora materials'
aurora_nodes = [n for n in unreal.ObjectIterator(unreal.MaterialExpression) if n.get_outer() == aurora_material]
scale_parameters = [n for n in aurora_nodes if isinstance(n, unreal.MaterialExpressionScalarParameter)
                    and str(n.get_editor_property('parameter_name')) == 'BSKAuroraSystemScale']
assert len(scale_parameters) == 1, 'Expected one BSKAuroraSystemScale material parameter'
scale_parameter = scale_parameters[0]
assert scale_parameter.get_editor_property('use_custom_primitive_data'), 'Aurora scale must read CPD'
assert scale_parameter.get_editor_property('primitive_data_index') == 0, 'Aurora scale must read CPD index 0'
transform_node = unreal.find_object(aurora_material, 'MaterialExpressionTransform_3')
source_transform_node = unreal.find_object(source_aurora_material, 'MaterialExpressionTransform_3')
assert transform_node and source_transform_node, 'Missing authored aurora coordinate transform'
for property_name in ['transform_source_type', 'transform_type']:
    assert transform_node.get_editor_property(property_name) == source_transform_node.get_editor_property(property_name), (
        f'Aurora coordinate transform changed: {property_name}')
assert 'TRANSFORM_LOCAL' in str(transform_node.get_editor_property('transform_type'))
dividers = [n for n in aurora_nodes if isinstance(n, unreal.MaterialExpressionDivide)
            and n.get_editor_property('desc') == 'BSK: restore source Niagara component scale']
assert len(dividers) == 1, 'Expected one aurora component-scale divider'
material_lib = unreal.MaterialEditingLibrary
divider_inputs = dict(zip(map(str, material_lib.get_material_expression_input_names(dividers[0])),
                          material_lib.get_inputs_for_material_expression(aurora_material, dividers[0])))
assert divider_inputs.get('A') == transform_node, 'Aurora divider must receive authored local coordinates'
assert divider_inputs.get('B') == scale_parameter, 'Aurora divider must divide by CPD scale'
multiply_node = unreal.find_object(aurora_material, 'MaterialExpressionMultiply_49')
object_scale_node = unreal.find_object(aurora_material, 'MaterialExpressionMaterialFunctionCall_47')
assert multiply_node and object_scale_node, 'Missing authored aurora ObjectScale multiplication'
multiply_inputs = dict(zip(map(str, material_lib.get_material_expression_input_names(multiply_node)),
                           material_lib.get_inputs_for_material_expression(aurora_material, multiply_node)))
assert multiply_inputs.get('A') == dividers[0], 'Aurora coordinates must pass through scale compensation'
assert multiply_inputs.get('B') == object_scale_node, 'Aurora must preserve authored ObjectScale multiplication'
report['aurora_material_scale_binding'] = {
    'parameter': 'BSKAuroraSystemScale', 'custom_primitive_data_index': 0,
    'expected_surface_scale': surface_mesh_scale, 'graph_verified': True,
    'runtime_value_validation': 'native-test: BskUnreal.Environment.AuroraMaterialScaleTracksBackendRadius',
    'runtime_value_checked_by_this_script': False,
}
stars = rig.get_editor_property('star_sphere')
assert stars.static_mesh_component.get_material(0).get_path_name().startswith('/Game/BSK/Environment/IncredibleEarth/MI_Stars_Main')
source_star_yaw = 23.07902168164
star_yaw = stars.get_actor_rotation().yaw
assert abs((star_yaw - source_star_yaw + 180.0) % 360.0 - 180.0) < 1e-5, (
    f'StarDome orientation changed: {star_yaw} != source {source_star_yaw}')
moons = [a for a in actors if 'BSK.Semantic.moon' in {str(t) for t in a.get_editor_property('tags')}]
assert len(moons) == 1, f'Expected one saved Moon, found {len(moons)}'
moon_component = moons[0].get_component_by_class(unreal.StaticMeshComponent)
assert moon_component and moon_component.get_editor_property('static_mesh'), 'Moon has no geometry'
moon_bounds = moon_component.get_editor_property('static_mesh').get_bounds()
moon_scale = moon_component.get_world_scale()
assert math.isclose(abs(moon_scale.x), abs(moon_scale.y), rel_tol=1e-9)
assert math.isclose(abs(moon_scale.x), abs(moon_scale.z), rel_tol=1e-9)
moon_radius = moon_bounds.sphere_radius * abs(moon_scale.x)
# Preview and backend use the same 1737.4 km physical lunar radius.
expected_moon_radius_cm = 1737400.0 * rig.get_editor_property('geometry_centimeters_per_meter')
assert math.isclose(moon_radius, expected_moon_radius_cm, rel_tol=1e-6), (
    f'Saved Moon preview radius: {moon_radius} != physical radius {expected_moon_radius_cm}')
moon_center = unreal.MathLibrary.transform_location(moon_component.get_world_transform(), moon_bounds.origin)
separation = moon_center - earth.get_actor_location()
expected_center = (38440000000.0, 0.0, -687100000.0)
expected_separation = (38440000000.0, 0.0, 0.0)
for axis, center_value, separation_value in zip('xyz', expected_center, expected_separation):
    assert math.isclose(getattr(moon_center, axis), center_value, rel_tol=0.0, abs_tol=1.0), (
        f'Moon preview centre {axis} drifted: {getattr(moon_center, axis)} != {center_value}')
    assert math.isclose(getattr(separation, axis), separation_value, rel_tol=0.0, abs_tol=1.0), (
        f'Earth-Moon preview offset {axis} drifted: {getattr(separation, axis)} != {separation_value}')
moon_distance = math.sqrt(separation.x ** 2 + separation.y ** 2 + separation.z ** 2)
assert moon_distance > radius_cm + moon_radius, (
    f'Moon intersects Earth: centre distance {moon_distance}, combined radius {radius_cm + moon_radius}')
report.update(enlargement=enlargement,
              star_yaw_degrees=star_yaw,
              moon={'actor': moons[0].get_name(), 'radius_cm': moon_radius,
                    'earth_center_distance_cm': moon_distance, 'center': str(moon_center)})
moon_lights = [a for a in actors if 'BSK.IncredibleEarth80K.MoonLight' in
               {str(t) for t in a.get_editor_property('tags')}]
assert len(moon_lights) == 1, f'Expected one source MoonLight, found {len(moon_lights)}'
moon_light_actor = moon_lights[0]
moon_light = moon_light_actor.get_component_by_class(unreal.DirectionalLightComponent)
assert moon_light, 'Source MoonLight is not a directional light'
assert moon_light_actor.get_attach_parent_actor() == moons[0], 'MoonLight must follow the saved Moon'
channels = moon_light.get_editor_property('lighting_channels')
assert (channels.channel0, channels.channel1, channels.channel2) == (False, False, True)
expected_moon_lux = 0.5 * (132821.5 / 13.0)
assert math.isclose(moon_light.get_editor_property('intensity'), expected_moon_lux, rel_tol=1e-6)
for name in ['surface', 'clouds', 'atmosphere']:
    channels = earth.get_editor_property(name).get_editor_property('lighting_channels')
    assert (channels.channel0, channels.channel1, channels.channel2) == (False, True, True), name
channels = moon_component.get_editor_property('lighting_channels')
assert (channels.channel0, channels.channel1, channels.channel2) == (False, True, False)
# The saved environment may have no spacecraft. Check any explicitly tagged
# local preview models that are present; live manifest lighting is a runtime test.
local_components = []
for actor in actors:
    tags = {str(t) for t in actor.get_editor_property('tags')}
    is_celestial = actor in (earth, moons[0], stars, rig.get_editor_property('sun_visual')) or any(
        t.startswith('BSK.Celestial.') for t in tags)
    is_local_model = 'BSK.Preview.StaticAsset' in tags or 'BSK.Semantic.spacecraft' in tags
    if is_local_model and not is_celestial:
        for component in actor.get_components_by_class(unreal.StaticMeshComponent):
            if not component.get_editor_property('static_mesh'):
                continue
            channels = component.get_editor_property('lighting_channels')
            assert (channels.channel0, channels.channel1, channels.channel2) == (True, False, False), (
                f'Local model receives celestial light: {component.get_path_name()}')
            local_components.append(component.get_path_name())
report['moon_fill'] = {'actor': moon_light_actor.get_name(),
                       'intensity_lux': moon_light.get_editor_property('intensity'),
                       'earth_only_channel': 2, 'local_model_components_checked': local_components}
report['moon_fill']['source_color_rgba'] = verify_components(
    'MoonLight.light_color', moon_light.get_editor_property('light_color'), (177, 205, 255, 255))
sun = rig.get_editor_property('sun_visual')
assert not sun.get_editor_property('surface').is_visible()
assert all(not sun.get_editor_property(n).get_editor_property('template') for n in ['sun_bursts','sun_halo','sun_lines'])
sun_light = rig.get_editor_property('sun_light').get_component_by_class(unreal.DirectionalLightComponent)
appearance_checks = runpy.run_path(str(Path(__file__).with_name('validate_moon_sun_appearance.py')))
report['moon_sun_appearance'] = appearance_checks['validate'](rig, moon_component)
report['sun_disk_color_scale'] = verify_components(
    'Sun.atmosphere_sun_disk_color_scale', sun_light.get_editor_property('atmosphere_sun_disk_color_scale'),
    (1.0, 0.5002009869, 0.2497670054, 1.0))
sky = rig.get_editor_property('earth_atmosphere').get_component_by_class(unreal.SkyAtmosphereComponent)
expected_sky_bottom_km = 1079.5 * enlargement
sky_bottom_km = sky.get_editor_property('bottom_radius')
assert 0 < sky_bottom_km < radius_cm / 100000.0, 'SkyAtmosphere bottom must remain inside the Earth surface'
assert math.isclose(sky_bottom_km, expected_sky_bottom_km, rel_tol=0.0, abs_tol=.001), (
    f'SkyAtmosphere bottom lost source inset: {sky_bottom_km} != {expected_sky_bottom_km} km')
expected_aerial_start_km = 1.0 * enlargement
aerial_start_km = sky.get_editor_property('aerial_perspective_start_depth')
assert math.isclose(aerial_start_km, expected_aerial_start_km, rel_tol=1e-6, abs_tol=1e-6), (
    f'SkyAtmosphere aerial perspective start: {aerial_start_km} != {expected_aerial_start_km} km')
report['sky_length_baselines'] = {
    'source_bottom_radius_km': 1079.5, 'bottom_radius_km': sky_bottom_km,
    'bottom_to_surface_ratio': sky_bottom_km / (radius_cm / 100000.0),
    'surface_inset_km': radius_cm / 100000.0 - sky_bottom_km,
    'source_aerial_perspective_start_depth_km': 1.0,
    'aerial_perspective_start_depth_km': aerial_start_km,
}
assert rig.get_editor_property('earth_atmosphere').get_actor_location() == earth.get_actor_location()
# Source main-level SkyAtmosphereComponent values (plus its recorded default
# white Mie scattering). Colour vectors retain their authored values; only
# scattering lengths and scalar coefficients receive size compensation.
source_sky_colors = {
    'rayleigh_scattering': (0.17528699338436127, 0.4096069931983948, 1.0, 30.21147918701172),
    'mie_scattering': (1.0, 1.0, 1.0, 1.0),
    'mie_absorption': (0.8999999761581421, 1.100000023841858, 0.8999999761581421, 1.0),
    'other_absorption': (0.3455609977245331, 1.0, 0.04518900066614151, 531.632080078125),
}
report['sky_source_color_baselines'] = {
    name: verify_components('SkyAtmosphere.' + name, sky.get_editor_property(name), expected)
    for name, expected in source_sky_colors.items()
}
exposure = rig.get_editor_property('exposure_volume').get_editor_property('settings')
assert exposure.get_editor_property('auto_exposure_method') == unreal.AutoExposureMethod.AEM_MANUAL
assert exposure.get_editor_property('camera_iso') == 100
assert exposure.get_editor_property('camera_shutter_speed') == 125
assert exposure.get_editor_property('depth_of_field_fstop') == 16
# These are the source main level's enabled artistic overrides after excluding
# its exposure and render-pipeline settings. An exact set also catches accidental
# GI, AO, reflection, or exposure overrides on either migrated volume.
base_art_overrides = {
    'bloom_method', 'bloom_intensity', 'bloom_threshold', 'bloom_convolution_texture',
    'bloom_convolution_size', 'bloom_convolution_pre_filter_min',
    'bloom_convolution_pre_filter_max', 'bloom_convolution_pre_filter_mult',
    'bloom_convolution_buffer_scale', 'scene_fringe_intensity', 'vignette_intensity',
    'motion_blur_amount',
}
source_postprocess = {
    'PostProcessVolume_2': base_art_overrides,
    'ColorCorrection': base_art_overrides | {
        'white_temp', 'white_tint', 'color_contrast', 'color_correction_highlights_min',
        'film_slope', 'film_toe', 'film_shoulder',
    },
}
art_volumes = [a for a in actors if any(str(t).startswith('BSK.IncredibleEarth80K.PostProcess.')
                                      for t in a.get_editor_property('tags'))]
assert len(art_volumes) == 2, f'Expected both source art volumes, found {len(art_volumes)}'
report['postprocess_volumes'] = []
for source_name, expected_fields in source_postprocess.items():
    tag = 'BSK.IncredibleEarth80K.PostProcess.' + source_name
    matches = [a for a in art_volumes if tag in {str(t) for t in a.get_editor_property('tags')}]
    assert len(matches) == 1 and isinstance(matches[0], unreal.PostProcessVolume), tag
    volume = matches[0]
    assert volume.get_editor_property('enabled') and volume.get_editor_property('unbound'), tag
    assert math.isclose(volume.get_editor_property('priority'), 0.0, abs_tol=1e-6), tag
    assert math.isclose(volume.get_editor_property('blend_weight'), 1.0, abs_tol=1e-6), tag
    settings = volume.get_editor_property('settings')
    enabled_fields = {name[len('override_'):] for name in dir(settings)
                      if name.startswith('override_') and settings.get_editor_property(name)}
    assert enabled_fields == expected_fields, (
        f'{tag}: missing overrides {expected_fields - enabled_fields}; unexpected overrides {enabled_fields - expected_fields}')
    volume_report = {'source': source_name, 'actor': volume.get_name(),
                     'enabled_artistic_overrides': sorted(enabled_fields)}
    if source_name == 'ColorCorrection':
        source_scalars = {'white_temp': 7000.0, 'white_tint': -0.08,
                          'film_slope': 0.95, 'film_toe': 0.5, 'film_shoulder': 0.15}
        actual_scalars = {}
        for name, expected in source_scalars.items():
            actual = settings.get_editor_property(name)
            assert math.isclose(actual, expected, rel_tol=1e-6, abs_tol=1e-6), (
                f'ColorCorrection.{name}: {actual} != source baseline {expected}')
            actual_scalars[name] = actual
        volume_report['source_scalar_baselines'] = actual_scalars
        volume_report['source_color_contrast'] = verify_components(
            'ColorCorrection.color_contrast', settings.get_editor_property('color_contrast'),
            (1.16, 1.16, 1.16, 1.0), fields='xyzw')
    report['postprocess_volumes'].append(volume_report)
report.update(radius_m=radius_cm/100, atmosphere_height_km=sky.get_editor_property('atmosphere_height'),
              aurora_bounds=str(aurora.get_system_fixed_bounds()), actor_count=len(actors))
registry = unreal.AssetRegistryHelpers.get_asset_registry()
registry.search_all_assets(True)
options = unreal.AssetRegistryDependencyOptions(include_hard_package_references=True, include_soft_package_references=True)
pending, visited, missing = [MAP], set(), []
while pending:
    asset = pending.pop()
    if asset in visited: continue
    visited.add(asset)
    if not unreal.EditorAssetLibrary.does_asset_exist(asset): missing.append(asset)
    for dependency in registry.get_dependencies(asset,options):
        dep = str(dependency)
        if dep.startswith('/Game/') and dep not in visited: pending.append(dep)
assert not missing, missing
report['project_dependency_count'] = len(visited)
report['missing_dependencies'] = missing
out=Path(unreal.Paths.project_saved_dir())/'IncredibleEarthMigration/validation.json'
out.parent.mkdir(parents=True, exist_ok=True)
out.write_text(json.dumps(report,indent=2),encoding='utf8')
unreal.log('BSK_INCREDIBLE_EARTH_VALIDATED: physical geometry, Moon separation, source orientation, material lengths, aurora scale, bindings, exposure and dependency closure')
