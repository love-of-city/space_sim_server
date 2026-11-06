"""Content identity for the current native benchmark dependency closure.

No simulator import or file mutation. A snapshot hashes content again rather
than trusting timestamps or distribution versions. Native free-flight tasks
share one source MJCF and its assets. Rendered tasks additionally bind their
UE build/content, image-estimator dependencies and graphics driver identity.
"""
import hashlib
from concurrent.futures import ThreadPoolExecutor
import importlib.metadata
import importlib.util
import json
import os
from pathlib import Path
import platform
import sys
import sysconfig
import xml.etree.ElementTree as ET

PACKAGES = ('Basilisk', 'numpy', 'scipy', 'pydantic', 'pydantic_core', 'psutil')
NUMERIC_ENVIRONMENT = ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS',
                       'NUMEXPR_NUM_THREADS', 'SPACE_SIM_DYNAMICS_BACKEND', 'SPACE_SIM_LOCAL_SUBSTEPS',
                       'SPACE_SIM_LOCAL_PUBLISH_STRIDE', 'SPACE_SIM_ORBITAL_FORCE_STRIDE')


def digest_file(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def files_under(root):
    if not root.is_dir():
        raise FileNotFoundError(root)
    for directory, children, files in os.walk(root):
        children[:] = sorted(name for name in children if name not in {'__pycache__', '.git', 'site-packages'})
        for name in sorted(files):
            if name.endswith(('.pyc', '.pyo')):
                continue
            path = Path(directory)/name
            if path.is_file():
                yield path


def native_resources(server):
    """All authored inputs read by the current stowed-SARM model builders."""
    model = server/'model/SARM/platform/sarm_ground_target_self_collision.xml'
    root = ET.parse(model).getroot()
    if root.findall('.//include'):
        raise ValueError('native identity does not yet support included MJCF')
    compiler = root.find('compiler')
    meshes = model.parent/(compiler.get('meshdir', '') if compiler is not None else '')
    textures = model.parent/(compiler.get('texturedir', '') if compiler is not None else '')
    paths = {model, model.with_name('attitude_control.json')}
    for element in root.findall('./asset/*[@file]'):
        directory = meshes if element.tag == 'mesh' else textures if element.tag == 'texture' else model.parent
        paths.add((directory/element.get('file')).resolve())
    paths.update(model.parent.glob('*.py'))
    for path in paths:
        if not path.resolve().is_relative_to(server.resolve()):
            raise ValueError('native asset escapes the declared repository resource closure')
        with path.open('rb') as stream:
            if stream.read(64).startswith(b'version https://git-lfs.github.com/spec/v1'):
                raise ValueError('native identity requires hydrated LFS assets: '+str(path))
    return sorted(paths)


def runtime_identity(server, *, rendered=False):
    server = Path(server).resolve()
    files = {}
    errors = {}
    expected_native_sha256 = None

    def record(label, path):
        files[label] = path

    def tree(label, path):
        for item in files_under(path):
            record(label+'/'+item.relative_to(path).as_posix(), item)

    try:
        for path in native_resources(server):
            record('repository/'+path.relative_to(server).as_posix(), path)
    except (OSError, ValueError, ET.ParseError) as error:
        errors['native_assets'] = str(error)
    try:
        manifest = server/'run/native_acceleration/local_mujoco_stepper.json'
        record('native/build_manifest.json', manifest)
        build = json.loads(manifest.read_text(encoding='utf-8'))
        library = (server/build['library']).resolve()
        if not library.is_relative_to(server/'run/native_acceleration'):
            raise ValueError('native library escapes its build directory')
        record('native/library', library)
        expected_native_sha256 = build['library_sha256']
    except (OSError, ValueError, KeyError) as error:
        errors['native_library'] = str(error)
    visual_metadata = None
    if rendered:
        try:
            from .benchmark_vision_resources import rendered_resources
            visual_files, visual_metadata = rendered_resources(server)
            files.update(visual_files)
        except (OSError, ValueError, KeyError, RuntimeError) as error:
            errors['rendered_resources'] = str(error)
    versions = {}
    for name in (*PACKAGES, 'cv2') if rendered else PACKAGES:
        try:
            spec = importlib.util.find_spec(name)
            if spec is None or not spec.origin:
                raise ValueError('package is not installed')
            location = Path(spec.origin).resolve()
            if spec.submodule_search_locations:
                tree('packages/'+name, location.parent)
                sibling = location.parent.with_name(name+'.libs')
                if sibling.is_dir(): tree('packages/'+name+'.libs', sibling)
            else:
                record('packages/'+name+'/'+location.name, location)
            try:
                versions[name] = importlib.metadata.version(name)
            except importlib.metadata.PackageNotFoundError:
                versions[name] = None
        except (ImportError, OSError, ValueError) as error:
            errors['package/'+name] = str(error)
    try:
        record('python/executable', Path(sys.executable))
        base = Path(sys.base_prefix)
        for path in sorted(base.glob('*.dll')):
            record('python/'+path.name, path)
        if (base/'DLLs').is_dir(): tree('python/DLLs', base/'DLLs')
        tree('python/stdlib', Path(sysconfig.get_path('stdlib')))
        tree('adapter', server.parent/'space_sim_UE_Adapter/Adapters/bsk_render_adapter')
    except OSError as error:
        errors['python_or_adapter'] = str(error)
    def hash_entry(entry):
        key, path = entry
        try:
            return key, digest_file(path), None
        except OSError as error:
            return key, None, str(error)

    hashed = {}
    with ThreadPoolExecutor(max_workers=4) as pool:
        for key, value, error in pool.map(hash_entry, sorted(files.items())):
            if error is not None: errors['file/'+key] = error
            else: hashed[key] = value
    if expected_native_sha256 is not None and hashed.get('native/library') != expected_native_sha256:
        errors['native_library'] = 'native library content differs from its build manifest'
    installed = sorted([distribution.metadata.get('Name', ''), distribution.version]
                       for distribution in importlib.metadata.distributions())
    data = dict(schema_version='spacesim-autonomy-native-runtime/1', python=sys.version,
        scope='native V0 stowed SARM; full critical package contents and all installed distribution versions',
        python_implementation=platform.python_implementation(), system=platform.platform(),
        package_versions=versions, installed_distributions=installed, files_sha256=hashed,
        numeric_environment={key: os.environ.get(key) for key in NUMERIC_ENVIRONMENT}, errors=errors)
    if rendered:
        data.update(schema_version='spacesim-autonomy-rendered-runtime/1',
            scope='native SARM plus real UE RGB: built project, asset catalogue, shader/content, editor dependencies, OpenCV and GPU driver',
            rendered_environment=visual_metadata)
    payload = json.dumps(data, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
    return dict(data, fingerprint=hashlib.sha256(payload).hexdigest())


def plan_runtime_identity(plan):
    """Legacy archives remain replayable; every new v2 plan must bind runtime."""
    schema = plan.get('schema_version')
    if schema in (None, 'spacesim-autonomy-batch-plan/1'):
        return None
    if schema != 'spacesim-autonomy-batch-plan/2':
        raise ValueError('unknown benchmark batch plan schema')
    identity = plan.get('runtime_identity')
    if (not isinstance(identity, dict) or not identity.get('fingerprint') or not isinstance(identity.get('files_sha256'), dict)
            or not identity['files_sha256']
            or identity.get('schema_version') not in {'spacesim-autonomy-native-runtime/1','spacesim-autonomy-rendered-runtime/1'}):
        raise ValueError('frozen v2 plan requires a complete runtime identity')
    payload = {key: value for key, value in identity.items() if key != 'fingerprint'}
    digest = hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
    if digest != identity['fingerprint']:
        raise ValueError('frozen runtime identity fingerprint does not match its contents')
    return identity
