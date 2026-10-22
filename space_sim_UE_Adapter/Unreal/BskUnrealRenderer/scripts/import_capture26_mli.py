from pathlib import Path
import unreal
root=Path(unreal.Paths.project_dir())
dest='/Game/BSK/VisualOverlays/Capture26'
name='SM_Capture26_MLI'
t=unreal.AssetImportTask()
for k,v in dict(filename=str(root/'ContentSource/Capture26/SM_Capture26_MLI.obj'),destination_path=dest,destination_name=name,automated=True,replace_existing=True,save=False).items():t.set_editor_property(k,v)
o=unreal.FbxImportUI()
for k,v in dict(import_mesh=True,import_materials=False,import_textures=False,automated_import_should_detect_type=False,mesh_type_to_import=unreal.FBXImportType.FBXIT_STATIC_MESH).items():o.set_editor_property(k,v)
d=o.get_editor_property('static_mesh_import_data')
for k,v in dict(auto_generate_collision=False,combine_meshes=True,generate_lightmap_u_vs=False,remove_degenerates=False,normal_import_method=unreal.FBXNormalImportMethod.FBXNIM_IMPORT_NORMALS).items():d.set_editor_property(k,v)
t.set_editor_property('options',o)
unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([t])
m=unreal.load_asset(dest+'/'+name)
assert isinstance(m,unreal.StaticMesh)
# Collision generation is disabled in import options; overlay is visual-only.
for i,slot in enumerate(m.get_editor_property('static_materials')):
    s=str(slot.get_editor_property('imported_material_slot_name'))
    role=next(r for r in ['MliFace','MliHem','MliBacking'] if r.lower() in s.lower())
    mat=unreal.load_asset('/Game/BSK/VisualOverlays/SarmMLI_PBR0110/MI_PBR0110_'+role)
    assert mat
    m.set_material(i,mat)
unreal.EditorAssetLibrary.save_loaded_asset(m)
unreal.log('CAPTURE26_MLI_OK')
