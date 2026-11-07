"""Main-scene environment and asset contract for stationary capture collection."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MAIN_COMMIT = 'd47b669edb4f59950062f87975f11cd267b39578'
RENDER_CONFIG = ROOT/'config/capture-main-renderer.json'


def platform_contract():
    from .capture_grasp import VERSION
    from simulation.model_preparation.capture_grasp_model import REFERENCE_MODEL
    return dict(schema='capture-platform-main/2', reference_main_commit=MAIN_COMMIT,
                renderer_config_sha256=hashlib.sha256(RENDER_CONFIG.read_bytes()).hexdigest(),
                task_controller_revision=VERSION,
                task_controller_sha256=hashlib.sha256(
                    Path(__file__).with_name('capture_grasp.py').read_bytes()).hexdigest(),
                state_reader_sha256=hashlib.sha256(
                    Path(__file__).with_name('native_state.py').read_bytes()).hexdigest(),
                carry_planner_sha256=hashlib.sha256(
                    Path(__file__).with_name('capture_carry.py').read_bytes()).hexdigest(),
                payload_initialization_sha256=hashlib.sha256(
                    Path(__file__).with_name('capture_payload.py').read_bytes()).hexdigest(),
                contact_audit_sha256=hashlib.sha256(
                    Path(__file__).with_name('capture_contact_audit.py').read_bytes()).hexdigest(),
                preflight_sha256=hashlib.sha256(
                    Path(__file__).with_name('capture_preflight.py').read_bytes()).hexdigest(),
                reset_preparation_sha256=hashlib.sha256(
                    (ROOT/'simulation/model_preparation/capture_grasp_model.py').read_bytes()).hexdigest(),
                visual_reference_model_sha256=hashlib.sha256(REFERENCE_MODEL.read_bytes()).hexdigest(),
                capture_runtime_sha256=hashlib.sha256(
                    (ROOT/'collection/run_capture_grasp.py').read_bytes()).hexdigest(),
                omitted=['target_aruco_visual_attachment', 'fiducial_pose_estimation'],
                task_camera_revision='capture-global-approach-jaws/6-close-left-global',
                task_camera_sha256=hashlib.sha256(
                    Path(__file__).with_name('capture_cameras.py').read_bytes()).hexdigest())


def verify_main_assets(adapter):
    """Fail before launching if any referenced original main asset is absent/LFS."""
    config = json.loads(RENDER_CONFIG.read_text())
    paths = set()
    def visit(value):
        if isinstance(value, dict):
            for item in value.values(): visit(item)
        elif isinstance(value, list):
            for item in value: visit(item)
        elif isinstance(value, str) and value.startswith('/Game/'):
            paths.add(value.split('.')[0])
    visit(config)
    assets = []
    for name in sorted(paths):
        asset = Path(adapter)/'Unreal/BskUnrealRenderer/Content'/(name[6:]+'.uasset')
        header = b''
        if asset.is_file():
            with asset.open('rb') as stream: header = stream.read(4)
        if header != b'\xc1\x83\x2a\x9e':
            raise FileNotFoundError(f'Original main environment asset missing or LFS pointer: {asset}')
        assets.append(dict(asset=name, file=str(asset.resolve()), bytes=asset.stat().st_size))
    return assets


def verify_capture_render_assets(adapter, catalog):
    """Require every catalog mesh/texture before accepting capture RGB.

    An environment-only check misses gripper meshes. UE may continue after a
    missing mesh warning, producing valid MP4s with invisible robot parts.
    """
    document = json.loads(Path(catalog).read_text())
    paths = set()
    def visit(value):
        if isinstance(value, dict):
            for key, item in value.items():
                visit(key)
                visit(item)
        elif isinstance(value, list):
            for item in value:
                visit(item)
        elif isinstance(value, str) and value.startswith('/Game/'):
            paths.add(value.split('.')[0])
    visit(document)
    if not paths:
        raise ValueError('Capture render catalog contains no /Game assets')
    project = Path(adapter)/'Unreal/BskUnrealRenderer'
    # Existing material/blanket attachments must survive asset migration too.
    ini = project/'Config/DefaultGame.ini'
    if ini.is_file():
        for line in ini.read_text().splitlines():
            if line.strip().startswith('/Game/') and '=' in line:
                for value in line.strip().split('=', 1):
                    visit(value.strip())
    assets, missing = [], []
    for name in sorted(paths):
        asset = project/'Content'/(name[6:]+'.uasset')
        header = b''
        if asset.is_file():
            with asset.open('rb') as stream:
                header = stream.read(4)
        if header != b'\xc1\x83\x2a\x9e':
            missing.append(str(asset))
        else:
            assets.append(dict(asset=name, file=str(asset.resolve()), bytes=asset.stat().st_size))
    if missing:
        raise FileNotFoundError('Capture render assets missing or LFS pointers: '+', '.join(missing))
    return assets


def install_platform_environment(ctx, scene, orbit):
    """Use the same optional modules/conditions as the original teleop session."""
    from simulation.components.plug_retention import install_configured_plug_retention
    from simulation.components.gyroscope import GyroscopeComponent
    from simulation.components.drag import configured_drag_components
    retention = install_configured_plug_retention(ctx)
    drag = (configured_drag_components(orbit.physics.drag, planet_radius_m=float(orbit.earth.radEquator))
            if orbit.physics.drag is not None else [])
    ctx.install(drag)
    flight = None
    handoff = scene.get('rendezvous_handoff')
    if handoff:
        from simulation.components.flight_environment import FlightEnvironmentComponent
        from space_arm_platform.rendezvous_config import FlightOptions
        options = FlightOptions.from_dict(handoff['mission_options'])
        flight = FlightEnvironmentComponent(options, [
            ('capture_target', None, handoff['body_properties']['capture_target']['mass_kg'], options.target_area_m2),
            ('cubesat_bus', None, handoff['body_properties']['cubesat_bus']['mass_kg'], options.chaser_area_m2)])
        ctx.install([flight])
    gyro = GyroscopeComponent(scene.get('sensors', {}).get('service_gyro', {}), seed=scene.get('seed', 0))
    ctx.install([gyro])
    return dict(gyro=gyro, drag=drag, flight=flight, retention=retention)
