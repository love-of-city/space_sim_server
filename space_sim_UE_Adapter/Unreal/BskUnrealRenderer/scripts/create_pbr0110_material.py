"""Build isolated PBR0110 instances and the continuous visual SARM wrap.

Run in UE editor Python. Never writes original Foil002, SARM physics or base_link.
Only the new texture/instance namespace and duplicated overlay are saved.
"""
from pathlib import Path
import hashlib
import json
import unreal

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'ContentSource/PBR0110'
TEXTURES = '/Game/BSK/Materials/PBR0110'
DEST = '/Game/BSK/VisualOverlays/SarmMLI_PBR0110'
ORIGINAL = '/Game/BSK/VisualOverlays/SarmMLI/SM_SarmMLI'
PARENT = '/Game/BSK/VisualOverlays/SarmMLI/M_SarmMLI_UV'
MESH = DEST + '/SM_SarmMLI_PBR0110'
PARAMETERS = {'FoilColor': 'albedo', 'FoilNormalDX': 'normal', 'FoilRoughness': 'roughness', 'FoilMetalness': 'metallic', 'FoilAmbientOcclusion': 'ao'}
LIB = unreal.MaterialEditingLibrary
ASSETS = unreal.EditorAssetLibrary
TOOLS = unreal.AssetToolsHelpers.get_asset_tools()


def save(asset):
    if not ASSETS.save_loaded_asset(asset, only_if_is_dirty=False):
        raise RuntimeError('Failed saving ' + asset.get_path_name())


def build():
    config = json.loads((SOURCE / 'source.json').read_text(encoding='utf-8'))
    records = {item['role']: item for item in config['textures']}
    textures = {}
    for role in PARAMETERS.values():
        record = records[role]
        path = SOURCE / record['file']
        if hashlib.sha256(path.read_bytes()).hexdigest() != record['sha256']:
            raise RuntimeError('Texture checksum mismatch: ' + str(path))
        name = 'T_PBR0110_' + role
        task = unreal.AssetImportTask()
        for key, value in {'filename': str(path), 'destination_path': TEXTURES, 'destination_name': name,
                           'automated': True, 'replace_existing': True, 'save': False,
                           'factory': unreal.TextureFactory()}.items():
            task.set_editor_property(key, value)
        TOOLS.import_asset_tasks([task])
        texture = unreal.load_asset(TEXTURES + '/' + name)
        if not isinstance(texture, unreal.Texture2D):
            raise RuntimeError('Failed importing ' + role + ': ' + str(task.imported_object_paths))
        normal = role == 'normal'
        texture.set_editor_property('srgb', role == 'albedo')
        texture.set_editor_property('compression_settings', unreal.TextureCompressionSettings.TC_NORMALMAP if normal else
                                    unreal.TextureCompressionSettings.TC_DEFAULT if role == 'albedo' else unreal.TextureCompressionSettings.TC_MASKS)
        texture.set_editor_property('flip_green_channel', normal and config['normal_flip_green'])
        texture.set_editor_property('never_stream', True)
        texture.set_editor_property('address_x', unreal.TextureAddress.TA_WRAP)
        texture.set_editor_property('address_y', unreal.TextureAddress.TA_WRAP)
        save(texture)
        textures[role] = texture
        unreal.log('PBR0110_TEXTURE_READY: ' + role)

    parent = unreal.load_asset(PARENT)
    original = unreal.load_asset(ORIGINAL)
    if not isinstance(parent, unreal.Material) or not isinstance(original, unreal.StaticMesh):
        raise RuntimeError('Existing SARM UV blanket/master is missing; prepare it first')
    materials = {}
    for role, values in config['instances'].items():
        name = 'MI_PBR0110_' + role
        mi = unreal.load_asset(DEST + '/' + name) if ASSETS.does_asset_exist(DEST + '/' + name) else None
        if mi is None:
            mi = TOOLS.create_asset(name, DEST, unreal.MaterialInstanceConstant, unreal.MaterialInstanceConstantFactoryNew())
        if not isinstance(mi, unreal.MaterialInstanceConstant):
            raise RuntimeError('Wrong instance type: ' + name)
        LIB.set_material_instance_parent(mi, parent)
        for parameter, texture_role in PARAMETERS.items():
            LIB.set_material_instance_texture_parameter_value(mi, parameter, textures[texture_role])
            if LIB.get_material_instance_texture_parameter_value(mi, parameter) != textures[texture_role]:
                raise RuntimeError('Texture parameter readback failed: ' + parameter)
        for parameter in ['TextureTiling', 'NormalStrength', 'RoughnessBias']:
            LIB.set_material_instance_scalar_parameter_value(mi, parameter, values[parameter])
            if abs(LIB.get_material_instance_scalar_parameter_value(mi, parameter) - values[parameter]) > 1e-5:
                raise RuntimeError('Scalar parameter readback failed: ' + parameter)
        LIB.set_material_instance_vector_parameter_value(mi, 'FoilTint', unreal.LinearColor(*values['FoilTint']))
        LIB.update_material_instance(mi)
        save(mi)
        materials[role] = mi

    # Reimport only the independent PBR0110 visual mesh; original Foil002 mesh stays untouched.
    geometry = SOURCE / 'Continuous'
    manifest = json.loads((geometry / 'mesh_manifest.json').read_text(encoding='utf-8'))
    mesh_file = geometry / 'SM_SarmMLI_PBR0110.obj'
    if hashlib.sha256(mesh_file.read_bytes()).hexdigest() != manifest['mesh_sha256']:
        raise RuntimeError('Continuous wrap checksum mismatch')
    task = unreal.AssetImportTask()
    for key,value in {'filename': str(mesh_file), 'destination_path': DEST, 'destination_name': 'SM_SarmMLI_PBR0110',
                      'automated': True, 'replace_existing': True, 'save': False}.items():
        task.set_editor_property(key,value)
    options = unreal.FbxImportUI()
    for key,value in {'import_mesh': True, 'import_materials': False, 'import_textures': False,
                      'automated_import_should_detect_type': False, 'mesh_type_to_import': unreal.FBXImportType.FBXIT_STATIC_MESH}.items():
        options.set_editor_property(key,value)
    data = options.get_editor_property('static_mesh_import_data')
    for key,value in {'auto_generate_collision': False, 'combine_meshes': True, 'generate_lightmap_u_vs': False,
                      'remove_degenerates': False, 'normal_import_method': unreal.FBXNormalImportMethod.FBXNIM_IMPORT_NORMALS}.items():
        data.set_editor_property(key,value)
    task.set_editor_property('options',options)
    TOOLS.import_asset_tasks([task])
    mesh = unreal.load_asset(MESH)
    if not isinstance(mesh, unreal.StaticMesh):
        raise RuntimeError('Continuous wrap import failed')
    subsystem = unreal.get_editor_subsystem(unreal.StaticMeshEditorSubsystem)
    subsystem.remove_collisions(mesh)
    settings = subsystem.get_lod_build_settings(mesh,0)
    for key,value in {'build_scale3d': unreal.Vector(1,1,1), 'recompute_normals': False, 'recompute_tangents': True,
                      'use_mikk_t_space': True, 'remove_degenerates': False, 'generate_lightmap_u_vs': False,
                      'use_full_precision_u_vs': True}.items():
        settings.set_editor_property(key,value)
    nanite = mesh.get_editor_property('nanite_settings')
    nanite.set_editor_property('enabled',False)
    mesh.set_editor_property('nanite_settings',nanite)
    subsystem.set_lod_build_settings(mesh,0,settings)
    seen = set()
    for index, slot in enumerate(mesh.get_editor_property('static_materials')):
        slot_name = str(slot.get_editor_property('imported_material_slot_name'))
        role = next((name for name in materials if name.casefold() in slot_name.casefold()), slot_name)
        if role not in materials:
            raise RuntimeError('Unexpected blanket material slot: ' + role)
        mesh.set_material(index, materials[role])
        if mesh.get_material(index) != materials[role]:
            raise RuntimeError('Blanket slot readback failed: ' + role)
        seen.add(role)
    if seen != set(materials):
        raise RuntimeError('Incomplete blanket material slots')
    if mesh.get_num_triangles(0) != manifest['triangles']:
        raise RuntimeError('Continuous wrap triangle loss on import')
    uv_count = subsystem.get_num_uv_channels(mesh, 0)
    if uv_count < 1:
        raise RuntimeError('Continuous wrap missing UV channel')
    extent = mesh.get_bounds().box_extent
    expected = [(high-low)*50 for low,high in zip(*manifest['bounds_m'])]
    if any(abs(x-y) > .02 for x,y in zip((extent.x,extent.y,extent.z),expected)):
        raise RuntimeError('Continuous wrap scale/bounds mismatch')
    save(mesh)
    report = {'status': 'PBR0110_BUILD_OK', 'mesh': mesh.get_path_name(), 'original_unchanged': ORIGINAL,
              'parent': PARENT, 'triangles': mesh.get_num_triangles(0), 'uv_channels': uv_count,
              'material_slots': sorted(seen), 'normal_flip_green': textures['normal'].get_editor_property('flip_green_channel'),
              'textures': {role: t.get_path_name() for role,t in textures.items()}, 'settings': config['instances'],
              'height_displacement': False, 'physics_changed': False, 'continuous_wrap': True, 'artificial_seam_gap_m': 0, 'face_sheets': len(manifest['pieces'])}
    saved = ROOT / 'Saved/AssetImport'
    saved.mkdir(parents=True, exist_ok=True)
    (saved / 'pbr0110_validation.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    unreal.log('PBR0110_BUILD_OK: continuous visual wrap; original blanket, XML, base mesh and target foil untouched')


if __name__ == '__main__':
    build()
