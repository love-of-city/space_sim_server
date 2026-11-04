"""Read-only checks for the restored IncredibleEarth Moon and Sun preview.

Call validate(rig, moon_component) inside the editor. This module does not load
levels, change assets, save packages, or claim visual/radiometric equivalence.
The returned dictionary contains only JSON-compatible values; a failed check
raises AssertionError with the property or connection that differs.
"""
from __future__ import annotations

import math

import unreal


LIB = unreal.MaterialEditingLibrary
SOURCE_INSTANCE = '/Game/IncredibleEarth/Materials/MI_MoonFarAway'
TARGET_INSTANCE = '/Game/BSK/Environment/IncredibleEarth/MI_Moon_Main'
# Independent baseline from the source/target cameras and confirmed HDR
# PreExposure captures. Do not derive emissive gain from solar illuminance.
SOURCE_REFERENCE_EXPOSURE = 100.0 / (100.0 * 60.0 * 4.0 ** 2) * 2.0 ** 9
TARGET_REFERENCE_EXPOSURE = 100.0 / (100.0 * 125.0 * 16.0 ** 2)
EXPECTED_GAIN = SOURCE_REFERENCE_EXPOSURE / TARGET_REFERENCE_EXPOSURE
EXPECTED_SUN_VISUAL_EXPOSURE_GAIN = EXPECTED_GAIN / (132821.5 / 13.0)
assert math.isclose(EXPECTED_GAIN, 17066.666666666668, rel_tol=1e-12)
SOURCE_SCALARS = {'EmissiveBoost', 'Exp', 'MaskAdjust', 'Normal Strength'}


def _require(condition, message):
    if not condition:
        raise AssertionError(message)


def _close(actual, expected, label):
    actual = float(actual)
    _require(math.isclose(actual, expected, rel_tol=1e-6, abs_tol=1e-6),
             f'{label}: {actual} != {expected}')
    return actual


def _inputs(material, expression):
    names = [str(name) for name in LIB.get_material_expression_input_names(expression)]
    nodes = list(LIB.get_inputs_for_material_expression(material, expression))
    _require(len(names) == len(nodes), f'Cannot align inputs for {expression.get_path_name()}')
    return dict(zip(names, nodes))


def _master(instance):
    current = instance
    seen = set()
    while not isinstance(current, unreal.Material):
        _require(current is not None, 'Moon material has no master')
        path = current.get_path_name()
        _require(path not in seen, f'Cyclic material parent: {path}')
        seen.add(path)
        current = current.get_editor_property('parent')
    return current


def _expressions(material):
    return {node.get_name(): node for node in unreal.ObjectIterator(unreal.MaterialExpression)
            if node.get_outer() == material}


def _upstream_signature(material, start):
    """Compare original node names/classes, every input edge, and texture assets."""
    signature = {}
    pending = [start] if start else []
    while pending:
        node = pending.pop()
        name = node.get_name()
        if name in signature:
            continue
        inputs = _inputs(material, node)
        entry = {
            'class': node.get_class().get_name(),
            'inputs': {key: value.get_name() if value else None for key, value in inputs.items()},
        }
        if isinstance(node, unreal.MaterialExpressionTextureSample):
            texture = node.get_editor_property('texture')
            entry['texture'] = texture.get_path_name() if texture else None
        signature[name] = entry
        pending.extend(value for value in inputs.values() if value)
    return signature


def _color(value, expected, label):
    return [_close(getattr(value, channel), component, f'{label}.{channel}')
            for channel, component in zip('rgba', expected)]


def _sun_disk_radiance_scale(physical_angle_degrees, visual_angle_degrees):
    for angle in (physical_angle_degrees, visual_angle_degrees):
        _require(math.isfinite(angle) and 0 < angle < 180, 'Invalid Sun disk angular diameter')
    # Solid angle = 4*pi*sin(diameter/4)^2; the ratio avoids cancellation at
    # small angles and is independent of the shared 4*pi factor.
    return (math.sin(math.radians(physical_angle_degrees) / 4.0)
            / math.sin(math.radians(visual_angle_degrees) / 4.0)) ** 2


def _scalar_value(instance, name):
    if isinstance(instance, unreal.MaterialInstanceDynamic):
        return instance.get_scalar_parameter_value(name)
    return LIB.get_material_instance_scalar_parameter_value(instance, name)


def validate_star_sun_compensation(rig, physical_angle_degrees=None):
    """Validate separate star gain and Sun disk size/radiance compensation.

    Omit physical_angle_degrees for the saved preview. Pass the current physical
    source angle to also check a runtime star MID's solid-angle compensation.
    """
    source = unreal.load_asset('/Game/IncredibleEarth/Materials/MI_Stars_Cinematics')
    _require(source is not None, 'Missing source main-scene Stars instance')
    star_actor = rig.get_editor_property('star_sphere')
    _require(star_actor is not None, 'Environment has no star sphere')
    component = star_actor.get_component_by_class(unreal.StaticMeshComponent)
    _require(component is not None, 'Star sphere has no StaticMeshComponent')
    target = component.get_material(0)
    _require(target is not None, 'Star sphere has no assigned material')
    source_master, target_master = _master(source), _master(target)
    _require(source_master.get_path_name().split('.')[0] == '/Game/IncredibleEarth/Materials/Masters/M_Stars',
             'Unexpected source Stars master')
    source_nodes, target_nodes = _expressions(source_master), _expressions(target_master)
    output_property = unreal.MaterialProperty.MP_EMISSIVE_COLOR
    source_output = LIB.get_material_property_input_node(source_master, output_property)
    target_output = LIB.get_material_property_input_node(target_master, output_property)
    _require(isinstance(source_output, unreal.MaterialExpressionAdd), 'Source Stars emissive must end in Add')
    _require(isinstance(target_output, unreal.MaterialExpressionAdd)
             and target_output.get_name() == source_output.get_name(),
             'Stars gain incorrectly multiplies the combined stars and physical Sun disk')
    source_inputs = _inputs(source_master, source_output)
    target_inputs = _inputs(target_master, target_output)
    disk_inputs = [name for name, node in source_inputs.items() if node
                   and node.get_class().get_name() == 'MaterialExpressionSkyAtmosphereLightDiskLuminance']
    _require(len(disk_inputs) == 1 and len(source_inputs) == 2,
             'Source Stars output must add one direct atmospheric Sun disk to the stars')
    disk_input = disk_inputs[0]
    stars_input = next(name for name in source_inputs if name != disk_input)
    source_stars, source_disk = source_inputs[stars_input], source_inputs[disk_input]
    _require(source_stars is not None, 'Source Stars branch is disconnected')
    factor_nodes = [node for node in target_nodes.values()
                    if isinstance(node, unreal.MaterialExpressionScalarParameter)
                    and str(node.get_editor_property('parameter_name')) == 'BSKEmissiveGain']
    _require(len(factor_nodes) == 1, 'Stars must contain exactly one BSKEmissiveGain parameter')
    factor = factor_nodes[0]
    disk_parameters = {}
    for name, expected_default in [('BSKSunVisualAngularDiameter', 3.0), ('BSKSunDiskRadianceScale', 1.0),
                                   ('BSKSunVisualExposureGain', EXPECTED_SUN_VISUAL_EXPOSURE_GAIN)]:
        matches = [node for node in target_nodes.values()
                   if isinstance(node, unreal.MaterialExpressionScalarParameter)
                   and str(node.get_editor_property('parameter_name')) == name]
        _require(len(matches) == 1, f'Expected one {name} parameter')
        _close(matches[0].get_editor_property('default_value'), expected_default, 'Stars master ' + name)
        disk_parameters[name] = matches[0]
    visual_angle_parameter = disk_parameters['BSKSunVisualAngularDiameter']
    radiance_parameter = disk_parameters['BSKSunDiskRadianceScale']
    exposure_parameter = disk_parameters['BSKSunVisualExposureGain']
    target_disk = target_nodes[source_disk.get_name()]
    disk_node_inputs = _inputs(target_master, target_disk)
    angle_inputs = [name for name in disk_node_inputs
                    if name.replace(' ', '').lower() == 'diskangulardiameteroverride']
    _require(len(angle_inputs) == 1, 'Missing Sun disk diameter override')
    angle_input = angle_inputs[0]
    _require(disk_node_inputs[angle_input] == visual_angle_parameter,
             'Sun disk must use the independent visual angular diameter')
    multiplier = target_inputs.get(stars_input)
    _require(isinstance(multiplier, unreal.MaterialExpressionMultiply), 'Stars branch is missing emissive compensation')
    exposure_multiplier = target_inputs.get(disk_input)
    _require(isinstance(exposure_multiplier, unreal.MaterialExpressionMultiply), 'Sun disk is missing exposure compensation')
    exposure_multiplier_inputs = _inputs(target_master, exposure_multiplier)
    disk_multiplier = exposure_multiplier_inputs.get('A')
    _require(isinstance(disk_multiplier, unreal.MaterialExpressionMultiply), 'Sun disk is missing solid-angle compensation')
    _require(exposure_multiplier_inputs.get('B') == exposure_parameter,
             'Sun disk exposure must multiply only the remaining exposure gap, never the full star gain')
    added_names = set(target_nodes) - set(source_nodes)
    _require(not (set(source_nodes) - set(target_nodes)), 'Original Stars expression nodes were removed')
    expected_added = {factor.get_name(), multiplier.get_name(), visual_angle_parameter.get_name(),
                      radiance_parameter.get_name(), disk_multiplier.get_name(),
                      exposure_parameter.get_name(), exposure_multiplier.get_name()}
    _require(added_names == expected_added and len(added_names) == 7,
             f'Expected four parameters and three separate multipliers: {sorted(added_names)}')
    multiplier_inputs = _inputs(target_master, multiplier)
    _require(multiplier_inputs.get('A') == target_nodes[source_stars.get_name()]
             and multiplier_inputs.get('B') == factor,
             'Stars gain must multiply the original star branch exactly once')
    disk_multiplier_inputs = _inputs(target_master, disk_multiplier)
    _require(disk_multiplier_inputs.get('A') == target_disk
             and disk_multiplier_inputs.get('B') == radiance_parameter,
             'Sun disk may multiply only its solid-angle scale, never the star emissive gain')
    _close(factor.get_editor_property('default_value'), EXPECTED_GAIN, 'Stars master gain default')
    gain = _close(_scalar_value(target, 'BSKEmissiveGain'), EXPECTED_GAIN, 'Stars.BSKEmissiveGain')
    visual_angle = _close(_scalar_value(target, 'BSKSunVisualAngularDiameter'),
                          rig.get_editor_property('sun_visual_angular_diameter_degrees'), 'Stars visual Sun diameter')
    expected_radiance_scale = (1.0 if physical_angle_degrees is None else
                               _sun_disk_radiance_scale(physical_angle_degrees, visual_angle))
    radiance_scale = _close(_scalar_value(target, 'BSKSunDiskRadianceScale'),
                            expected_radiance_scale, 'Stars Sun disk radiance scale')
    disk_exposure_gain = _close(_scalar_value(target, 'BSKSunVisualExposureGain'),
                                EXPECTED_SUN_VISUAL_EXPOSURE_GAIN, 'Stars Sun disk exposure gain')
    _close(disk_exposure_gain * (132821.5 / 13.0) * TARGET_REFERENCE_EXPOSURE,
           SOURCE_REFERENCE_EXPOSURE, 'Sun disk exposure and illuminance calibration')
    graphs = {}
    for label, source_start in [('stars', source_stars), ('physical_sun_disk', source_disk)]:
        source_graph = _upstream_signature(source_master, source_start)
        target_graph = _upstream_signature(target_master, target_nodes[source_start.get_name()])
        if label == 'physical_sun_disk':
            # The diameter input is the only intentional change inside the
            # source disk node; every other source edge/texture stays intact.
            _require(source_graph[source_start.get_name()]['inputs'].get(angle_input) is None,
                     'Source Sun disk already has a diameter override; review adaptation')
            target_graph[source_start.get_name()]['inputs'][angle_input] = None
            target_graph.pop(visual_angle_parameter.get_name())
        _require(source_graph == target_graph, f'Original {label} topology or texture changed')
        graphs[label] = target_graph
    parameters = {}
    for kind in ['scalar', 'vector', 'texture', 'static_switch']:
        names = {str(name) for name in getattr(LIB, f'get_{kind}_parameter_names')(source)}
        target_names = {str(name) for name in getattr(LIB, f'get_{kind}_parameter_names')(target)}
        expected_names = names | ({'BSKEmissiveGain', 'BSKSunVisualAngularDiameter', 'BSKSunDiskRadianceScale',
                                   'BSKSunVisualExposureGain'}
                                  if kind == 'scalar' else set())
        _require(target_names == expected_names, f'Stars {kind} parameter set changed')
        getter = getattr(LIB, f'get_material_instance_{kind}_parameter_value')
        values = {}
        for name in sorted(names):
            expected = getter(source, name)
            actual = (_scalar_value(target, name) if kind == 'scalar' else
                      getattr(target, f'get_{kind}_parameter_value')(name)
                      if isinstance(target, unreal.MaterialInstanceDynamic) and kind in ['vector', 'texture']
                      else getter(target, name))
            label = f'Stars.{name}'
            if kind == 'scalar':
                values[name] = _close(actual, expected, label)
            elif kind == 'vector':
                values[name] = _color(actual, [getattr(expected, c) for c in 'rgba'], label)
            else:
                _require(actual == expected, f'{label} changed')
                values[name] = (actual.get_path_name() if actual else None) if kind == 'texture' else bool(actual)
        parameters[kind] = values
    return {'source': source.get_path_name(), 'assigned_material': target.get_path_name(),
            'emissive_gain': gain, 'output_add': target_output.get_name(),
            'stars_gain_multiply': multiplier.get_name(), 'physical_sun_disk_gain': radiance_scale,
            'sun_disk_radiance_multiply': disk_multiplier.get_name(),
            'sun_disk_exposure_multiply': exposure_multiplier.get_name(),
            'sun_visual_exposure_gain': disk_exposure_gain,
            'source_reference_exposure': SOURCE_REFERENCE_EXPOSURE,
            'target_reference_exposure': TARGET_REFERENCE_EXPOSURE,
            'sun_visual_angular_diameter_degrees': visual_angle,
            'physical_angle_degrees_checked': physical_angle_degrees,
            'source_sun_disk': source_disk.get_name(), 'added_expression_names': sorted(added_names),
            'source_parameters_preserved': parameters, 'preserved_graphs': graphs}


def validate(rig, moon_component):
    """Validate the authored preview, before backend angular-size takeover."""
    source = unreal.load_asset(SOURCE_INSTANCE)
    _require(source is not None, f'Missing source instance: {SOURCE_INSTANCE}')
    target = moon_component.get_material(0)
    _require(target is not None, 'Moon has no assigned material')
    _require(target.get_path_name().split('.')[0] == TARGET_INSTANCE,
             f'Unexpected assigned Moon material: {target.get_path_name()}')
    moon_mesh = moon_component.get_editor_property('static_mesh')
    _require(moon_mesh is not None, 'Moon has no mesh')
    moon_scale = moon_component.get_world_scale()
    _close(abs(moon_scale.y), abs(moon_scale.x), 'Moon uniform scale Y')
    _close(abs(moon_scale.z), abs(moon_scale.x), 'Moon uniform scale Z')
    centimeters_per_meter = rig.get_editor_property('geometry_centimeters_per_meter')
    _require(centimeters_per_meter > 0, 'Invalid geometry unit scale')
    moon_radius_m = _close(moon_mesh.get_bounds().sphere_radius * abs(moon_scale.x) / centimeters_per_meter,
                           1737400.0, 'Moon physical preview radius (m)')

    source_names = {str(name) for name in LIB.get_scalar_parameter_names(source)}
    _require(source_names == SOURCE_SCALARS, f'Source Moon scalar set changed: {sorted(source_names)}')
    scalar_values = {}
    for name in sorted(source_names):
        expected = LIB.get_material_instance_scalar_parameter_value(source, name)
        scalar_values[name] = _close(
            LIB.get_material_instance_scalar_parameter_value(target, name), expected, f'Moon.{name}')
    gain = _close(LIB.get_material_instance_scalar_parameter_value(target, 'BSKEmissiveGain'),
                  EXPECTED_GAIN, 'Moon.BSKEmissiveGain')

    source_master, target_master = _master(source), _master(target)
    source_nodes, target_nodes = _expressions(source_master), _expressions(target_master)
    attributes_name = 'MaterialExpressionMakeMaterialAttributes_2'
    original_name = 'MaterialExpressionMultiply_0'
    _require(attributes_name in source_nodes and attributes_name in target_nodes,
             f'Missing original Moon attributes node: {attributes_name}')
    _require(original_name in source_nodes and original_name in target_nodes,
             f'Missing original Moon emissive node: {original_name}')
    source_attributes = _inputs(source_master, source_nodes[attributes_name])
    target_attributes = _inputs(target_master, target_nodes[attributes_name])
    _require(source_attributes.get('EmissiveColor') == source_nodes[original_name],
             'Source Moon emissive topology changed')

    gain_nodes = [node for node in target_nodes.values()
                  if isinstance(node, unreal.MaterialExpressionScalarParameter)
                  and str(node.get_editor_property('parameter_name')) == 'BSKEmissiveGain']
    _require(len(gain_nodes) == 1, f'Expected one BSKEmissiveGain node, found {len(gain_nodes)}')
    factor = gain_nodes[0]
    multiplier = target_attributes.get('EmissiveColor')
    _require(isinstance(multiplier, unreal.MaterialExpressionMultiply),
             'Moon EmissiveColor is not connected through a Multiply')
    added_names = set(target_nodes) - set(source_nodes)
    _require(not (set(source_nodes) - set(target_nodes)), 'Original Moon expression nodes were removed')
    _require(added_names == {factor.get_name(), multiplier.get_name()} and len(added_names) == 2,
             f'Expected exactly one added Multiply and one gain parameter: {sorted(added_names)}')
    multiplied_inputs = _inputs(target_master, multiplier)
    _require(multiplied_inputs.get('A') == target_nodes[original_name]
             and multiplied_inputs.get('B') == factor,
             'Moon gain must multiply the original emissive exactly once')
    _close(factor.get_editor_property('default_value'), EXPECTED_GAIN, 'Moon master gain default')

    preserved_graphs = {}
    for label, source_start, target_start in [
        ('original_emissive', source_nodes[original_name], target_nodes[original_name]),
        ('BaseColor', source_attributes.get('BaseColor'), target_attributes.get('BaseColor')),
        ('Normal', source_attributes.get('Normal'), target_attributes.get('Normal')),
    ]:
        _require(source_start is not None and target_start is not None,
                 f'Missing original Moon {label} connection')
        source_graph = _upstream_signature(source_master, source_start)
        target_graph = _upstream_signature(target_master, target_start)
        _require(source_start.get_name() == target_start.get_name() and source_graph == target_graph,
                 f'Original Moon {label} graph or texture changed')
        preserved_graphs[label] = target_graph

    sun_actor = rig.get_editor_property('sun_light')
    _require(sun_actor is not None, 'Environment has no Sun light')
    sun = sun_actor.get_component_by_class(unreal.DirectionalLightComponent)
    _require(sun is not None, 'Bound Sun has no DirectionalLightComponent')
    sun_values = {
        'sun_visual_angular_diameter_degrees': _close(
            rig.get_editor_property('sun_visual_angular_diameter_degrees'), 3.0, 'Sun preview angle'),
    }
    for name, expected in {
        'light_source_angle': 3.0,
        'light_source_soft_angle': 0.0,
        'intensity': 132821.5,
        'temperature': 6500.0,
        'bloom_scale': 0.10000000149011612,
        'bloom_threshold': 1.5,
        'bloom_max_brightness': 0.16117699444293976,
    }.items():
        sun_values[name] = _close(sun.get_editor_property(name), expected, 'Sun.' + name)
    sun_values['atmosphere_sun_disk_color_scale'] = _color(
        sun.get_editor_property('atmosphere_sun_disk_color_scale'),
        (1.0, 0.5002009868621826, 0.249767005443573, 1.0), 'Sun disk color')
    sun_values['bloom_tint'] = _color(
        sun.get_editor_property('bloom_tint'), (255, 236, 214, 255), 'Sun bloom tint')
    for name, expected in {'enable_light_shaft_bloom': False, 'atmosphere_sun_light': True,
                           'use_temperature': True, 'cast_translucent_shadows': False}.items():
        value = bool(sun.get_editor_property(name))
        _require(value == expected, f'Sun.{name}: {value} != {expected}')
        sun_values[name] = value
    _require(sun.get_editor_property('atmosphere_sun_light_index') == 0,
             'Sun disk must use atmosphere light index 0')
    sun_values['atmosphere_sun_light_index'] = 0

    return {
        'valid': True,
        'scope': 'Saved/editor preview; runtime separates physical source angle from the visual Sun disk',
        'moon': {
            'source': source.get_path_name(),
            'assigned_material': target.get_path_name(),
            'physical_preview_radius_m': moon_radius_m,
            'source_scalars_preserved': scalar_values,
            'emissive_gain': gain,
            'added_expression_names': sorted(added_names),
            'emissive_connection': {
                'original': original_name, 'multiply': multiplier.get_name(),
                'gain': factor.get_name(), 'attributes': attributes_name,
            },
            'preserved_graphs': preserved_graphs,
        },
        'sun': sun_values,
        'stars': validate_star_sun_compensation(rig),
    }
