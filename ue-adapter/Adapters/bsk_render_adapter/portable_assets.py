"""Resolve portable layout catalogs without accepting paths outside their package."""
from copy import deepcopy
import json
from pathlib import Path, PurePosixPath, PureWindowsPath


def package_path(root, relative):
    root = Path(root).resolve()
    text = str(relative)
    name = PurePosixPath(text)
    if not text or "\\" in text or name.is_absolute() or PureWindowsPath(text).drive or ".." in name.parts:
        raise ValueError(f"Expected a package-relative path: {relative}")
    path = (root / text).resolve()
    if not path.is_relative_to(root):
        raise ValueError(f"Package path escapes through a symlink: {relative}")
    return path


def package_root(document_path, relative):
    document_path = Path(document_path).resolve()
    name = PurePosixPath(str(relative))
    if name.is_absolute() or PureWindowsPath(str(relative)).drive or "\\" in str(relative):
        raise ValueError("Package root must be relative")
    root = (document_path.parent / str(relative)).resolve()
    if not document_path.is_relative_to(root):
        raise ValueError("Package root must contain its document")
    marker = root / "layout-package.json"
    if json.loads(marker.read_text()).get("schema") != "layout-package/1":
        raise ValueError("Missing portable layout package marker")
    return root


def ue_package_relative(object_path):
    """Map a project-local Unreal object path to its package within Content."""
    package = str(object_path).split('.')[0]
    if (not package.startswith('/Game/') or '\\' in package
            or ':' in package or '..' in PurePosixPath(str(object_path)).parts):
        raise ValueError(f'Only project-local UE packages can be relocated: {object_path}')
    relative = package.removeprefix('/Game/')
    if not relative or relative.startswith('/'):
        raise ValueError(f'Invalid UE package path: {object_path}')
    return relative + '.uasset'


def catalog_document(catalog):
    if isinstance(catalog, dict):
        if "package_root" in catalog:
            raise ValueError("Portable catalogs require their file location")
        return catalog
    path = Path(catalog)
    payload = json.loads(path.read_text(encoding="utf-8"))
    if "package_root" not in payload:
        return payload
    root = package_root(path, payload["package_root"])
    result = deepcopy(payload)
    result["source_mjcf"] = str(package_path(root, result["source_mjcf"]))
    for key in ("assets", "textures"):
        result[key] = {str(package_path(root, source)): value for source, value in result.get(key, {}).items()}
    return result
