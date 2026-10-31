"""Copy an accepted legacy layout pool into a new self-contained portable package.

Input originals are never rewritten. Native acceptance is retained as original
proof; relocation verifies XML and every asset byte, without claiming a new run.
"""
from __future__ import annotations
import argparse
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'backend'), str(ROOT.parent / 'ue-adapter/Adapters')]
from space_arm_platform.collection_layouts import LayoutPool, digest
from simulation.layout_package import mjcf_files, semantic_model_digest
from bsk_render_adapter.portable_assets import ue_package_relative


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + '\n')


def migrate(source, output, *, asset_roots, ue_content):
    source, output, ue_content = Path(source).resolve(), Path(output).resolve(), Path(ue_content).resolve()
    pool = LayoutPool(source)
    if pool.document['schema'] != 'collection-layout-pool/1':
        raise ValueError('Migration accepts legacy v1 pools; copy a v2 package as a whole')
    if output.exists():
        raise FileExistsError('Migration output must be a new directory')
    allowed = [source.parent, *(Path(p).resolve() for p in asset_roots)]
    if any(output == p or p.is_relative_to(output) for p in allowed):
        raise ValueError('Migration output cannot contain an input root')
    output.mkdir(parents=True)
    mapping = {}
    entries = []

    def copy_asset(source_path, expected=None):
        path = Path(source_path).resolve()
        if not any(path.is_relative_to(root) for root in allowed):
            raise ValueError(f'Asset outside declared input roots: {path}')
        if not path.is_file():
            raise FileNotFoundError(path)
        sha = digest(path)
        if expected is not None and sha != expected:
            raise ValueError(f'Asset no longer matches its declared source SHA: {path}')
        if str(path) in mapping:
            return mapping[str(path)]
        relative = f'assets/{sha}/{path.name}'
        destination = output / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        if not destination.exists():
            shutil.copyfile(path, destination)
        mapping[str(path)] = relative
        return relative

    try:
        for index, entry in enumerate(pool.document['layouts']):
            directory = output / 'layouts' / f'{index:03d}'
            directory.mkdir(parents=True)
            model = Path(entry['model']).resolve()
            catalog = json.loads(Path(entry['catalog']).read_text())
            record = json.loads(model.with_suffix('.layout.json').read_text())
            original = directory / 'original-model.xml'
            shutil.copyfile(model, original)
            for label, path in [('original-manifest.json', model.with_suffix('.layout.json')),
                                ('original-catalog.json', Path(entry['catalog'])),
                                ('native-geometry.json', Path(entry['native_report']))]:
                shutil.copyfile(path, directory / label)
            tree, files = mjcf_files(model)
            old_mapping = {}
            for element, key, asset in files:
                relative = copy_asset(asset)
                old_mapping[str(asset)] = relative
                element.set(key, os.path.relpath(output / relative, directory).replace(os.sep, '/'))
            compiler = tree.find('compiler')
            if compiler is not None:
                for key in ('assetdir', 'meshdir', 'texturedir'):
                    compiler.attrib.pop(key, None)
            migrated_model = directory / 'layout.xml'
            ET.ElementTree(tree).write(migrated_model, encoding='utf-8', xml_declaration=True)
            if semantic_model_digest(model) != semantic_model_digest(migrated_model):
                raise ValueError('Relocation changed model semantics')
            # Keep acceptance reports intact. The new manifest binds changed XML
            # bytes and separately proves that only paths changed.
            record['package_root'] = '../..'
            record['model'] = migrated_model.relative_to(output).as_posix()
            record['model_sha256'] = digest(migrated_model)
            if record.get('source'):
                record['source'] = copy_asset(record['source'], record.get('source_sha256'))
            record['adapted_base']['visual'] = copy_asset(record['adapted_base']['visual'])
            for visual in record.get('generated_visuals', []):
                visual['file'] = copy_asset(visual['file'])
            record['source_asset_sha256'] = {copy_asset(path, sha): sha for path, sha in record.get('source_asset_sha256', {}).items()}
            write_json(migrated_model.with_suffix('.layout.json'), record)
            migrated_catalog = deepcopy(catalog)
            migrated_catalog.update(schema='bsk-render-asset-catalog/2', package_root='../..',
                                    source_mjcf=migrated_model.relative_to(output).as_posix())
            migrated_catalog['assets'] = {}
            ue_packages = {}
            for asset, raw in catalog.get('assets', {}).items():
                relative = copy_asset(asset, raw.get('source_sha256') if isinstance(raw, dict) else None)
                raw = deepcopy(raw)
                if isinstance(raw, dict) and raw.get('import_source'):
                    raw['import_source'] = copy_asset(raw['import_source'])
                migrated_catalog['assets'][relative] = raw
                package = raw if isinstance(raw, str) else raw.get('asset_path')
                if package:
                    ue_packages[package] = None
            migrated_catalog['textures'] = {copy_asset(path): package for path, package in catalog.get('textures', {}).items()}
            ue_packages.update({p: None for p in migrated_catalog['textures'].values()})
            for package in ue_packages:
                package_rel = ue_package_relative(package)
                package_file = (ue_content / package_rel).resolve()
                if not package_file.is_relative_to(ue_content) or not package_file.is_file():
                    raise ValueError(f'Missing UE package inside declared Content root: {package_file}')
                target = output / 'ue-content' / package_rel
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(package_file, target)
            write_json(directory / 'catalog.json', migrated_catalog)
            companion = model.with_name('attitude_control.json')
            if companion.is_file():
                shutil.copyfile(companion, directory / companion.name)
            new_entry = dict(id=entry['id'], model=migrated_model.relative_to(output).as_posix(),
                             catalog=(directory/'catalog.json').relative_to(output).as_posix(),
                             native_report=(directory/'native-geometry.json').relative_to(output).as_posix(),
                             model_sha256=digest(migrated_model), catalog_sha256=digest(directory/'catalog.json'),
                             manifest_sha256=digest(migrated_model.with_suffix('.layout.json')),
                             native_report_sha256=entry['native_report_sha256'],
                             relocation=dict(original_model=original.relative_to(output).as_posix(),
                                             original_model_sha256=entry['model_sha256'],
                                             original_directory=str(model.parent),
                                             original_manifest=(directory/'original-manifest.json').relative_to(output).as_posix(),
                                             original_manifest_sha256=entry['manifest_sha256'],
                                             semantic_model_sha256=semantic_model_digest(migrated_model),
                                             asset_mapping=old_mapping))
            entries.append(new_entry)
        # The package root marker is stable after copying to another directory.
        write_json(output/'layout-package.json', dict(schema='layout-package/1', source_pool_sha256=pool.sha256))
        files = {p.relative_to(output).as_posix(): digest(p) for p in sorted(output.rglob('*')) if p.is_file()}
        document = dict(schema='collection-layout-pool/2', root='.', layouts=entries,
                        files_sha256=files, source_pool_sha256=pool.sha256,
                        generator='migrate-layout-pool/1')
        write_json(output/'pool.json', document)
        LayoutPool(output/'pool.json')
        write_json(output/'migration-report.json', dict(schema='layout-pool-migration/1',
            source_pool=str(source), source_pool_sha256=pool.sha256, output_pool_sha256=digest(output/'pool.json'),
            layouts=len(entries), file_count=len(files), original_inputs_modified=False,
            native_acceptance='original reports retained; XML/asset equivalence verified; not rerun',
            asset_mapping=mapping))
        return output/'pool.json'
    except BaseException as error:
        write_json(output/'migration-failed.json', dict(error=f'{type(error).__name__}: {error}', publishable=False))
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--asset-root', type=Path, action='append', default=[],
                        help='Explicit allowed mesh/source root; pool directory is included automatically')
    parser.add_argument('--ue-content', type=Path, required=True)
    args = parser.parse_args()
    print(migrate(args.source, args.output, asset_roots=args.asset_root, ue_content=args.ue_content))


if __name__ == '__main__':
    main()
