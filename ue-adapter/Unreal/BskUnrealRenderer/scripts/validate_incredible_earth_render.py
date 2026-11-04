"""Live-editor rendering check after asset/shader completion; never saves.

Run through editor Python after saved-map validation. Realtime and background
rendering must be enabled. Results live under Saved, outside source control.
"""
import json
import time
import traceback
from pathlib import Path
import unreal

actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
rig = next(a for a in actors.get_all_level_actors() if isinstance(a, unreal.BskEnvironment))
earth = rig.get_editor_property('earth')
effect = earth.get_editor_property('aurora')
sky = rig.get_editor_property('earth_atmosphere')
world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
out = Path(unreal.Paths.project_saved_dir()) / 'IncredibleEarthMigration' / ('render_repair_' + time.strftime('%Y%m%d_%H%M%S'))
out.mkdir(parents=True)
original_view = unreal.EditorLevelLibrary.get_level_viewport_camera_info()
original_earth = earth.get_actor_transform()
original_sky = sky.get_actor_transform()
sun_actors = [rig.get_editor_property(n) for n in ['sun_light', 'celestial_sun_light']]
original_sun = [a.get_actor_rotation() for a in sun_actors]
original_visible = effect.is_visible()
original_max_fps = unreal.SystemLibrary.get_console_variable_float_value('t.MaxFPS')
radius = 637100000.0
camera = actors.spawn_actor_from_class(unreal.CameraActor, unreal.Vector(), unreal.Rotator(), transient=True)
camera.camera_component.set_field_of_view(80)
levels.pilot_level_actor(camera)
unreal.SystemLibrary.execute_console_command(world, 't.MaxFPS 30')
# Clear infinite-lived preview particles left by earlier editor activations.
# Activate(reset=True) alone resets age without destroying that instance.
effect.reinitialize_system()
earth.refresh_visual_scale()
report = {'readiness': 'Flush async loading, finish shader/asset compilation, then allow streaming frames at each view.', 'saved_assets_modified': False,
          'vt_residency_counters': 'Not exposed by AutomationLibrary in this editor world; inspect the viewport VirtualTexturing overlay separately.',
          'samples': [], 'screenshots': [], 'particle_checks': []}
state = {'index': 0, 'next': 10.0, 'sample_at': 0.0, 'task': None, 'busy': False, 'ticks': 0}
names = ['polar_ready', 'polar_later', 'polar_hidden', 'day', 'night', 'sun', 'moon', 'earth_moved_rotated']


def write():
    (out / 'report.json').write_text(json.dumps(report, indent=2), encoding='utf8')


def aim(position, center):
    rotation = unreal.MathLibrary.find_look_at_rotation(position, center)
    camera.set_actor_location(position, False, True)
    camera.set_actor_rotation(rotation, True)
    unreal.EditorLevelLibrary.set_level_viewport_camera_info(position, rotation)


def pose(index):
    center = earth.get_actor_location()
    effect.set_visibility(index != 2)
    if index < 3:
        for actor in sun_actors:
            actor.set_actor_rotation(unreal.Rotator(pitch=90, yaw=0, roll=0), True)
        aim(center + unreal.Vector(0, 0, radius * 3), center)
    else:
        for actor, rotation in zip(sun_actors, original_sun):
            actor.set_actor_rotation(rotation, True)
        toward_sun = -sun_actors[0].get_actor_forward_vector()
        if index == 3:
            aim(center + toward_sun * (radius * 3), center)
        elif index == 4:
            aim(center - toward_sun * (radius * 3), center)
        elif index == 5:
            aim(unreal.Vector(), toward_sun * 1.0e10)
        elif index == 6:
            moon = next(a for a in actors.get_all_level_actors() if 'BSK.Semantic.moon' in [str(t) for t in a.tags])
            mesh = moon.get_component_by_class(unreal.StaticMeshComponent)
            bounds = mesh.static_mesh.get_bounds()
            moon_center = unreal.MathLibrary.transform_location(mesh.get_world_transform(), bounds.origin)
            moon_radius = bounds.sphere_radius * mesh.get_world_scale().x
            aim(moon_center + toward_sun * (moon_radius * 3), moon_center)
        elif index == 7:
            earth.set_actor_location(center + unreal.Vector(123000000, -45000000, 10000000), False, True)
            earth.set_actor_rotation(unreal.Rotator(pitch=12, yaw=37, roll=5), True)
            sky.set_actor_location(earth.get_actor_location(), False, True)
            for actor in sun_actors:
                actor.set_actor_rotation(unreal.MathLibrary.find_look_at_rotation(unreal.Vector(), earth.get_actor_up_vector()), True)
            aim(earth.get_actor_location() + earth.get_actor_up_vector() * (radius * 3), earth.get_actor_location())


def particles(name):
    params = unreal.NiagaraSimCacheCreateParameters()
    params.attribute_capture_mode = unreal.NiagaraSimCacheAttributeCaptureMode.ALL
    cache_obj = unreal.NiagaraSimCacheFunctionLibrary.create_niagara_sim_cache(effect)
    cache = unreal.NiagaraSimCacheFunctionLibrary.capture_niagara_sim_cache_immediate(cache_obj, params, effect, True, 1/60)
    assert cache
    for emitter in cache.get_emitter_names():
        positions = cache.read_position_attribute_with_rebase(effect.get_world_transform(), 'Position', emitter)
        error = max((p - effect.get_world_location()).length() for p in positions)
        report['particle_checks'].append({'view': name, 'emitter': str(emitter), 'count': len(positions), 'center_error_cm': error})
        # After a fresh initialization the source's one-shot burst is 100.
        assert len(positions) == 100 and error < 100, report['particle_checks'][-1]


def finish():
    unreal.unregister_slate_post_tick_callback(handle)
    earth.set_actor_transform(original_earth, False, True)
    sky.set_actor_transform(original_sky, False, True)
    for actor, rotation in zip(sun_actors, original_sun):
        actor.set_actor_rotation(rotation, True)
    effect.set_visibility(original_visible)
    earth.refresh_visual_scale()
    levels.eject_pilot_level_actor()
    actors.destroy_actor(camera)
    unreal.EditorLevelLibrary.set_level_viewport_camera_info(*original_view)
    unreal.SystemLibrary.execute_console_command(world, 't.MaxFPS ' + str(original_max_fps))
    unreal.EditorPythonScripting.set_keep_python_script_alive(False)
    write()


def tick(dt):
    if state['busy']:
        return
    try:
        state['ticks'] += 1
        elapsed = time.monotonic() - started
        assert elapsed < 600, 'Graphical validation exceeded 10 minutes'
        if elapsed >= state['sample_at']:
            report['samples'].append({'elapsed_seconds': elapsed, 'ticks': state['ticks'],
                'aurora_active': effect.is_active(),
                'asset_registry_loading': unreal.AssetRegistryHelpers.get_asset_registry().is_loading_assets()})
            state['sample_at'] = elapsed + 5
            write()
        if state['task']:
            if not state['task'].is_task_done():
                return
            state['task'] = None
            state['index'] += 1
            if state['index'] == len(names):
                report['completed_seconds'] = elapsed
                unreal.log('BSK_RENDER_REPAIR_VALIDATED ' + str(out))
                finish()
                return
            pose(state['index'])
            state['next'] = elapsed + 10
        if elapsed < state['next']:
            return
        state['busy'] = True
        unreal.AutomationLibrary.finish_loading_before_screenshot()
        name = names[state['index']]
        if state['index'] in [0, 7]:
            particles(name)
        report['screenshots'].append({'name': name, 'elapsed_seconds': time.monotonic() - started})
        write()
        state['task'] = unreal.AutomationLibrary.take_high_res_screenshot(1280, 720, str(out / (name + '.png')), camera=camera, delay=2)
        state['busy'] = False
    except Exception:
        report['error'] = traceback.format_exc()
        unreal.log_error(report['error'])
        finish()


pose(0)
unreal.AutomationLibrary.finish_loading_before_screenshot()
started = time.monotonic()
unreal.EditorPythonScripting.set_keep_python_script_alive(True)
handle = unreal.register_slate_post_tick_callback(tick)
print('BSK_RENDER_REPAIR_STARTED', str(out))
