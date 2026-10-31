"""Relocation proofs for immutable layout packages; no native imports."""
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

from bsk_render_adapter.portable_assets import package_path

FILE_ATTRIBUTES = {'file', 'fileup', 'filedown', 'fileleft', 'fileright', 'filefront', 'fileback'}


def mjcf_files(path, *, base_directory=None):
    """Resolve explicit MJCF assets; includes and path stripping need dedicated migration."""
    path = Path(path).resolve()
    root = ET.parse(path).getroot()
    compiler = root.find('compiler')
    directories = compiler.attrib if compiler is not None else {}
    if root.find('.//include') is not None or directories.get('strippath', 'false') == 'true':
        raise ValueError('Layout relocation requires a standalone MJCF without strippath')
    files = []
    for element in root.iter():
        for key, value in element.attrib.items():
            if key not in FILE_ATTRIBUTES:
                continue
            if '://' in value:
                raise ValueError('Remote MJCF assets are not portable package inputs')
            directory = directories.get('assetdir', '')
            if element.tag in {'mesh', 'flexcomp', 'skin'}:
                directory = directories.get('meshdir', directory)
            elif element.tag == 'texture':
                directory = directories.get('texturedir', directory)
            files.append((element, key, (Path(base_directory or path.parent) / directory / value).resolve()))
    return root, files


def file_digest(path, verified_hashes=None):
    path = Path(path).resolve()
    if verified_hashes is not None and path in verified_hashes:
        return verified_hashes[path]
    return hashlib.sha256(path.read_bytes()).hexdigest()


def semantic_model_digest(path, *, resolver=None, base_directory=None, verified_hashes=None):
    """Compare all MJCF structure and asset bytes, ignoring only reference path spelling."""
    root, files = mjcf_files(path, base_directory=base_directory)
    for element, key, file in files:
        source = resolver(file) if resolver is not None else file
        element.set(key, 'sha256:' + file_digest(source, verified_hashes))
    compiler = root.find('compiler')
    if compiler is not None:
        for key in ('assetdir', 'meshdir', 'texturedir'):
            compiler.attrib.pop(key, None)
    for element in root.iter():
        element.text = (element.text or '').strip() or None
        element.tail = None
    return hashlib.sha256(ET.tostring(root)).hexdigest()


def validate_relocation(root, entry, *, verified_hashes=None):
    proof = entry['relocation']
    original = package_path(root, proof['original_model'])
    if file_digest(original, verified_hashes) != proof['original_model_sha256']:
        raise ValueError('Original accepted model has changed in relocation evidence')
    original_manifest = package_path(root, proof['original_manifest'])
    if file_digest(original_manifest, verified_hashes) != proof['original_manifest_sha256']:
        raise ValueError('Original accepted manifest changed in relocation evidence')
    mapping = proof['asset_mapping']
    def resolve_old(path):
        if str(path) not in mapping:
            raise ValueError(f'Missing relocation asset mapping: {path}')
        return package_path(root, mapping[str(path)])
    before = semantic_model_digest(original, resolver=resolve_old, base_directory=proof["original_directory"],
                                   verified_hashes=verified_hashes)
    target = package_path(root, entry['model'])
    def resolve_new(path):
        if not path.is_relative_to(root):
            raise ValueError('Relocated MJCF asset escapes package root')
        return package_path(root, path.relative_to(root).as_posix())
    after = semantic_model_digest(target, resolver=resolve_new, verified_hashes=verified_hashes)
    if before != after or after != proof['semantic_model_sha256']:
        raise ValueError('Relocated MJCF changes more than asset path spelling')
    return proof
