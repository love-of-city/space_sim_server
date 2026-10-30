"""Regression checks for the optional local PBR0110 arm-satellite skin."""
import ast
import configparser
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'ContentSource/PBR0110'


def test_source_roles_and_normal_convention():
    config = json.loads((SOURCE / 'source.json').read_text(encoding='utf-8'))
    assert {f['role'] for f in config['textures']} == {'albedo','normal','roughness','metallic','ao','height'}
    assert all(f['width'] == f['height'] == 2048 for f in config['textures'])
    assert config['normal_flip_green'] is True
    evidence = config['normal_orientation_evidence']
    assert evidence['pbr0110_ny_vs_height_dy'] > 0 > evidence['foil002_dx_ny_vs_height_dy']
    assert set(config['instances']) == {'MliFace','MliHem','MliBacking'}
    assert config['instances']['MliFace']['TextureTiling'] == 3.0


def test_local_sources_match_downloaded_files():
    config = json.loads((SOURCE / 'source.json').read_text(encoding='utf-8'))
    for record in config['textures']:
        assert (SOURCE/record['file']).is_file(), 'Run git lfs pull to restore source textures'
        assert hashlib.sha256((SOURCE/record['file']).read_bytes()).hexdigest() == record['sha256']


def test_material_copy_is_isolated_and_no_displacement():
    source = (ROOT/'scripts/create_pbr0110_material.py').read_text(encoding='utf-8-sig')
    ast.parse(source)
    assert "'destination_path': DEST, 'destination_name': 'SM_SarmMLI_PBR0110'" in source
    assert "subsystem.remove_collisions(mesh)" in source
    assert 'original.set_material' not in source
    assert 'LIB.delete_all_material_expressions' not in source
    assert "'height_displacement': False" in source
    assert 'get_material_instance_texture_parameter_value' in source
    assert "mesh.get_num_triangles(0) != manifest['triangles']" in source
    assert "set_editor_property('flip_green_channel', normal and config['normal_flip_green'])" in source


def test_startup_rebuilds_only_selected_optional_pack():
    prepare = (ROOT/'scripts/prepare_runtime_materials.ps1').read_text(encoding='utf-8-sig')
    assert prepare.index('prepare_sarm_blanket.ps1') < prepare.index('prepare_pbr0110_material.ps1') < prepare.index('if (!$Force')
    assert "$gameConfig -match" in prepare and 'SarmMLI_PBR0110/' in prepare
    build = (ROOT/'scripts/prepare_pbr0110_material.ps1').read_text(encoding='utf-8-sig')
    assert build.index("$content -notmatch 'PBR0110_BUILD_OK'") < build.index('Set-Content -LiteralPath $marker')
    assert '-WindowStyle Hidden' in build and 'Stop-Process -Id $p.Id' in build


def test_configuration_leaves_the_other_satellite_unchanged():
    config = configparser.ConfigParser(strict=False)
    config.optionxform = str
    config.read(ROOT/'Config/DefaultGame.ini',encoding='utf-8-sig')
    assert config['Bsk.MaterialOverrides']['/Game/BSK/Generated/SARM/part_001_color_00.part_001_color_00'] == '/Game/BSK/Materials/Foil002/MI_BskFoil002.MI_BskFoil002'
    assert config['Bsk.VisualOverlays']['/Game/BSK/Generated/SARM/base_link.base_link'] == '/Game/BSK/VisualOverlays/SarmMLI_PBR0110/SM_SarmMLI_PBR0110.SM_SarmMLI_PBR0110'


def test_bundled_runtime_assets_are_materialized():
    assets = [ROOT / 'Content/BSK/Materials/PBR0110' / f'T_PBR0110_{role}.uasset'
              for role in ('albedo', 'normal', 'roughness', 'metallic', 'ao')]
    assets += [ROOT / 'Content/BSK/VisualOverlays/SarmMLI_PBR0110' / f'{name}.uasset'
               for name in ('SM_SarmMLI_PBR0110', 'MI_PBR0110_MliFace',
                            'MI_PBR0110_MliHem', 'MI_PBR0110_MliBacking')]
    assets += [SOURCE / 'Continuous' / f'SM_SarmMLI_PBR0110.{ext}' for ext in ('obj', 'mtl')]
    for path in assets:
        assert path.is_file(), f'Missing bundled asset: {path}'
        with path.open('rb') as stream:
            assert not stream.read(100).startswith(b'version https://git-lfs.github.com/spec/v1'), path
