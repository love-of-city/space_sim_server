"""Duplicate the approved blanket while preserving physical texture density."""
import json
from pathlib import Path
import unreal

ROOT = Path(__file__).resolve().parents[1]
SOURCE = '/Game/BSK/VisualOverlays/SarmMLI_PBR0110'
DEST = '/Game/Generated/DualServiceMLI'
assets = unreal.EditorAssetLibrary
lib = unreal.MaterialEditingLibrary


def duplicate(source, destination):
    result = unreal.load_asset(destination) if assets.does_asset_exist(destination) else assets.duplicate_asset(source, destination)
    if result is None:
        raise RuntimeError('Could not duplicate ' + source)
    return result


mesh = duplicate(SOURCE + '/SM_SarmMLI_PBR0110', DEST + '/SM_DualServiceMLI')
report = {'mesh': mesh.get_path_name(), 'hull_scale': 2., 'materials': {}, 'physics_changed': False}
for index, slot in enumerate(mesh.get_editor_property('static_materials')):
    role = str(slot.get_editor_property('imported_material_slot_name'))
    original = unreal.load_asset(SOURCE + '/MI_PBR0110_' + role)
    if original is None:
        raise RuntimeError('Unknown original slot: ' + role)
    material = duplicate(original.get_path_name(), DEST + '/MI_DualService_' + role)
    tiling = lib.get_material_instance_scalar_parameter_value(original, 'TextureTiling') * 2.
    lib.set_material_instance_scalar_parameter_value(material, 'TextureTiling', tiling)
    lib.update_material_instance(material)
    mesh.set_material(index, material)
    assert abs(lib.get_material_instance_scalar_parameter_value(material, 'TextureTiling') - tiling) < 1e-6
    assert mesh.get_material(index) == material
    report['materials'][role] = {'asset': material.get_path_name(), 'TextureTiling': tiling,
        'NormalStrength': lib.get_material_instance_scalar_parameter_value(material, 'NormalStrength'),
        'RoughnessBias': lib.get_material_instance_scalar_parameter_value(material, 'RoughnessBias')}
    assets.save_loaded_asset(material)
assets.save_loaded_asset(mesh)
(ROOT / 'Saved/AssetImport/dual_service_mli.json').write_text(json.dumps(report, indent=2))
unreal.log('DUAL_SERVICE_MLI_OK: original assets unchanged; 2x texture density compensation')
