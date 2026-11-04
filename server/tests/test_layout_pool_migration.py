"""Moving an entire package preserves models, render lookups and original acceptance."""
from copy import deepcopy
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import pytest

from test_collection_layouts import layout_pool  # noqa: F401
from space_arm_platform.collection_layouts import LayoutPool, digest
from tools.assets.migrate_layout_pool import migrate
from bsk_render_adapter.mjcf_assets import load_asset_catalog, resolve_asset
from bsk_render_adapter.portable_assets import package_path
from simulation.layout_package import semantic_model_digest


@pytest.fixture
def package_inputs(layout_pool, tmp_path):
    document = json.loads(layout_pool.read_text(encoding="utf-8"))
    content = tmp_path / 'ue-content'
    for entry in document['layouts']:
        model = Path(entry['model'])
        mesh = model.parent / 'referenced.obj'
        mesh.write_text('v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n')
        tree = ET.parse(model)
        ET.SubElement(ET.SubElement(tree.getroot(), 'asset'), 'mesh', name='shape', file=mesh.name)
        tree.write(model)
        record = json.loads(model.with_suffix('.layout.json').read_text(encoding="utf-8"))
        record['model_sha256'] = digest(model)
        model.with_suffix('.layout.json').write_text(json.dumps(record))
        native = json.loads(Path(entry['native_report']).read_text(encoding="utf-8"))
        native['model_sha256'] = digest(model)
        Path(entry['native_report']).write_text(json.dumps(native))
        catalog = json.loads(Path(entry['catalog']).read_text(encoding="utf-8"))
        for index, raw in enumerate(catalog['assets'].values()):
            raw['asset_path'] = f'/Game/Test/shape_{index}.shape_{index}'
            package = content / 'Test' / f'shape_{index}.uasset'
            package.parent.mkdir(parents=True, exist_ok=True)
            package.write_bytes(f'UE asset {index}'.encode())
        Path(entry['catalog']).write_text(json.dumps(catalog))
        for key in ('model', 'catalog', 'native_report'):
            entry[key + '_sha256'] = digest(entry[key])
        entry['manifest_sha256'] = digest(model.with_suffix('.layout.json'))
    layout_pool.write_text(json.dumps(document))
    return layout_pool, content


def test_migration_can_move_without_original_root_and_resolve_renderer(package_inputs, tmp_path):
    source, content = package_inputs
    original_bytes = source.read_bytes()
    source_models = [semantic_model_digest(e['model']) for e in json.loads(original_bytes)['layouts']]
    result = migrate(source, tmp_path/'package', asset_roots=[], ue_content=content)
    before = LayoutPool(result)
    frozen = tmp_path/'snapshot.json'
    before.write_snapshot(frozen)
    assert LayoutPool(frozen).sha256 == before.sha256
    moved = tmp_path/'different user'/'moved'
    moved.parent.mkdir()
    shutil.copytree(result.parent, moved)
    for entry in json.loads(original_bytes)['layouts']:
        shutil.rmtree(Path(entry['model']).parent)
    after = LayoutPool(moved/'pool.json')
    assert after.sha256 == before.sha256
    for index in range(2):
        model, catalog, record, selection = after.select_index(index, 123)
        assert semantic_model_digest(model) == source_models[index]
        assert model.is_relative_to(moved)
        assert selection['model_sha256'] == digest(model)
        visual = record['generated_visuals'][0]['file']
        assert Path(visual).is_relative_to(moved)
        assert resolve_asset(load_asset_catalog(catalog), visual).asset_path.startswith('/Game/Test/')
        original_report = json.loads((moved/after.document['layouts'][index]['native_report']).read_text(encoding="utf-8"))
        assert original_report['model_sha256'] != digest(model)  # Historical report was not rewritten.
    assert source.read_bytes() == original_bytes


@pytest.mark.parametrize('relative', ['../escape', '/tmp/escape', 'C:/escape', 'dir/../../escape', 'dir\\escape'])
def test_package_rejects_escaping_paths(tmp_path, relative):
    with pytest.raises(ValueError, match='relative'):
        package_path(tmp_path, relative)


def test_package_rejects_symlink_escape(tmp_path):
    root = tmp_path/'package'
    root.mkdir()
    link = root/'link'
    try:
        link.symlink_to(tmp_path, target_is_directory=True)
    except OSError as error:
        if getattr(error, 'winerror', None) != 1314:
            raise
        # Windows directory junctions exercise the same path-escape guard
        # without requiring the account's symbolic-link privilege.
        import _winapi
        _winapi.CreateJunction(str(tmp_path), str(link))
    with pytest.raises(ValueError, match='symlink'):
        package_path(root, 'link/escape')


@pytest.mark.parametrize('kind', ['mesh', 'ue', 'model'])
def test_mutation_after_load_rejected(package_inputs, tmp_path, kind):
    source, content = package_inputs
    path = migrate(source, tmp_path/'package', asset_roots=[], ue_content=content)
    pool = LayoutPool(path)
    names = pool.document['files_sha256']
    relative = (next(n for n in names if n.startswith('assets/')) if kind == 'mesh' else
                next(n for n in names if n.startswith('ue-content/')) if kind == 'ue' else
                pool.document['layouts'][0]['model'])
    (path.parent/relative).write_bytes(b'changed')
    with pytest.raises(ValueError, match='changed file'):
        pool.select_index(0, 1)


def test_untrusted_external_asset_and_existing_output_are_refused(package_inputs, tmp_path):
    source, content = package_inputs
    output = tmp_path/'already-exists'
    output.mkdir()
    with pytest.raises(FileExistsError):
        migrate(source, output, asset_roots=[], ue_content=content)
    outside = tmp_path.parent / (tmp_path.name + '-outside-source.obj')
    outside.write_text('do not copy')
    doc = json.loads(source.read_text(encoding="utf-8"))
    model = Path(doc['layouts'][0]['model'])
    tree = ET.parse(model)
    tree.find('.//mesh').set('file', str(outside))
    tree.write(model)
    # Even a pinned legacy model cannot grant access to undeclared asset roots.
    entry = doc['layouts'][0]
    record = json.loads(model.with_suffix('.layout.json').read_text(encoding="utf-8"))
    record['model_sha256'] = digest(model)
    model.with_suffix('.layout.json').write_text(json.dumps(record))
    native = json.loads(Path(entry['native_report']).read_text(encoding="utf-8"))
    native['model_sha256'] = digest(model)
    Path(entry['native_report']).write_text(json.dumps(native))
    entry.update(model_sha256=digest(model), manifest_sha256=digest(model.with_suffix('.layout.json')),
                 native_report_sha256=digest(entry['native_report']))
    source.write_text(json.dumps(doc))
    with pytest.raises(ValueError, match='outside declared input roots'):
        migrate(source, tmp_path/'refused', asset_roots=[], ue_content=content)
    outside.unlink()


@pytest.mark.parametrize('kind', ['marker', 'model', 'catalog', 'manifest', 'native_report',
                                  'mesh', 'ue', 'original_model', 'original_manifest'])
def test_referenced_file_must_be_in_hash_manifest(package_inputs, tmp_path, kind):
    source, content = package_inputs
    path = migrate(source, tmp_path / 'package', asset_roots=[], ue_content=content)
    document = json.loads(path.read_text(encoding="utf-8"))
    entry = document['layouts'][0]
    names = document['files_sha256']
    if kind == 'marker':
        relative = 'layout-package.json'
    elif kind == 'manifest':
        relative = Path(entry['model']).with_suffix('.layout.json').as_posix()
    elif kind in {'model', 'catalog', 'native_report'}:
        relative = entry[kind]
    elif kind.startswith('original_'):
        relative = entry['relocation'][kind]
    elif kind == 'mesh':
        relative = next(n for n in names if n.endswith('/referenced.obj'))
    else:
        relative = next(n for n in names if n.startswith('ue-content/'))
    del names[relative]
    path.write_text(json.dumps(document))
    with pytest.raises(ValueError, match='missing from file manifest'):
        LayoutPool(path)
