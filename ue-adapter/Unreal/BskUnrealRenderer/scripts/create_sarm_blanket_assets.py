"""Import the independent UV blanket and build its isolated UE materials.

Requires the existing Foil002 textures. Writes only /Game/BSK/VisualOverlays/SarmMLI.
No change to the original base_link asset or the generic runtime PBR materials.
"""
from pathlib import Path
import sys
import hashlib
import json
import unreal

sys.path.insert(0,str(Path(__file__).resolve().parent))
from create_foil_material import node, connect, scalar, custom, save

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'ContentSource/SarmMLI'
DEST='/Game/BSK/VisualOverlays/SarmMLI'
LIB=unreal.MaterialEditingLibrary
TOOLS=unreal.AssetToolsHelpers.get_asset_tools()


def asset(name, cls, factory):
    path=f'{DEST}/{name}'
    result=unreal.load_asset(path) if unreal.EditorAssetLibrary.does_asset_exist(path) else None
    if result is None:result=TOOLS.create_asset(name,DEST,cls,factory())
    if not isinstance(result,cls):raise RuntimeError(f'Wrong asset type at {path}')
    return result


def material_property(expr, prop, output=''):
    if not LIB.connect_material_property(expr,output,getattr(unreal.MaterialProperty,prop)):
        raise RuntimeError(f'Material output connection failed: {prop}')


def create_materials():
    mat=asset('M_SarmMLI_UV',unreal.Material,unreal.MaterialFactoryNew)
    LIB.delete_all_material_expressions(mat)
    mat.set_editor_property('shading_model',unreal.MaterialShadingModel.MSM_DEFAULT_LIT)
    mat.set_editor_property('blend_mode',unreal.BlendMode.BLEND_OPAQUE)
    mat.set_editor_property('two_sided',False)
    mat.set_editor_property('tangent_space_normal',True)
    uv=node(mat,unreal.MaterialExpressionTextureCoordinate,-1100,-400)
    scale=scalar(mat,'TextureTiling',8.,-1100,-200)
    multiply=node(mat,unreal.MaterialExpressionMultiply,-850,-400)
    connect(uv,multiply,'A');connect(scale,multiply,'B')
    samples={}
    for index,kind in enumerate(('Color','NormalDX','Roughness','Metalness','AmbientOcclusion')):
        texture=unreal.load_asset(f'/Game/BSK/Materials/Foil002/T_Foil002_{kind}')
        if not isinstance(texture,unreal.Texture2D):raise RuntimeError(f'Missing Foil002 texture: {kind}')
        sample=node(mat,unreal.MaterialExpressionTextureSampleParameter2D,-600,index*260-450)
        sample.set_editor_property('parameter_name','Foil'+kind);sample.set_editor_property('texture',texture)
        sample.set_editor_property('sampler_type',unreal.MaterialSamplerType.SAMPLERTYPE_COLOR if kind=='Color' else unreal.MaterialSamplerType.SAMPLERTYPE_NORMAL if kind=='NormalDX' else unreal.MaterialSamplerType.SAMPLERTYPE_MASKS)
        connect(multiply,sample,'UVs');samples[kind]=sample
    tint=node(mat,unreal.MaterialExpressionVectorParameter,-380,-650)
    tint.set_editor_property('parameter_name','FoilTint');tint.set_editor_property('default_value',unreal.LinearColor(1,1,1,1))
    color=node(mat,unreal.MaterialExpressionMultiply,-100,-400)
    connect(samples['Color'],color,'A','RGB');connect(tint,color,'B')
    strength=scalar(mat,'NormalStrength',.35,-400,150)
    normals=custom(mat,'UV tangent normal strength','return normalize(float3(N.xy*S,max(N.z,0.001)));',{'N':samples['NormalDX'],'S':strength},unreal.CustomMaterialOutputType.CMOT_FLOAT3,0,0)
    bias=scalar(mat,'RoughnessBias',.23,-400,500)
    rough=custom(mat,'Foil roughness','return clamp(R.r+B,0.08,0.95);',{'R':samples['Roughness'],'B':bias},unreal.CustomMaterialOutputType.CMOT_FLOAT1,0,250)
    for expr,prop,output in [(color,'MP_BASE_COLOR',''),(normals,'MP_NORMAL',''),(rough,'MP_ROUGHNESS',''),(samples['Metalness'],'MP_METALLIC','R'),(samples['AmbientOcclusion'],'MP_AMBIENT_OCCLUSION','R')]:material_property(expr,prop,output)
    unreal.log('SARM_MLI_GRAPH_READY: all UV shader inputs connected')
    LIB.recompile_material(mat);save(mat)
    result={}
    for role,strength,roughness,tint in [('MliFace',.35,.23,(1.,1.,1.)),('MliHem',.10,.36,(.60,.47,.28)),('MliBacking',.08,.65,(.055,.06,.07))]:
        mi=asset('MI_Sarm'+role,unreal.MaterialInstanceConstant,unreal.MaterialInstanceConstantFactoryNew)
        LIB.set_material_instance_parent(mi,mat)
        for name,value in [('TextureTiling',8.),('NormalStrength',strength),('RoughnessBias',roughness)]:
            LIB.set_material_instance_scalar_parameter_value(mi,name,value)
            if abs(LIB.get_material_instance_scalar_parameter_value(mi,name)-value)>1e-5:raise RuntimeError(f'Parameter readback failed: {name}')
        LIB.set_material_instance_vector_parameter_value(mi,'FoilTint',unreal.LinearColor(*tint,1))
        LIB.update_material_instance(mi);save(mi);result[role]=mi
    return result


def import_mesh(materials):
    mesh_file=SOURCE/'SM_SarmMLI.obj'
    manifest=json.loads((SOURCE/'mesh_manifest.json').read_text(encoding='utf-8'))
    if hashlib.sha256(mesh_file.read_bytes()).hexdigest()!=manifest['mesh_sha256']:raise RuntimeError('MLI mesh checksum mismatch')
    task=unreal.AssetImportTask();task.set_editor_property('filename',str(mesh_file));task.set_editor_property('destination_path',DEST);task.set_editor_property('destination_name','SM_SarmMLI');task.set_editor_property('automated',True);task.set_editor_property('replace_existing',True);task.set_editor_property('save',True)
    options=unreal.FbxImportUI()
    options.set_editor_property('import_mesh',True);options.set_editor_property('import_materials',False);options.set_editor_property('import_textures',False)
    options.set_editor_property('automated_import_should_detect_type',False);options.set_editor_property('mesh_type_to_import',unreal.FBXImportType.FBXIT_STATIC_MESH)
    data=options.get_editor_property('static_mesh_import_data')
    data.set_editor_property('auto_generate_collision',False)
    data.set_editor_property('combine_meshes',True);data.set_editor_property('generate_lightmap_u_vs',False);data.set_editor_property('remove_degenerates',False);data.set_editor_property('normal_import_method',unreal.FBXNormalImportMethod.FBXNIM_IMPORT_NORMALS)
    task.set_editor_property('options',options)
    TOOLS.import_asset_tasks([task])
    mesh=unreal.load_asset(f'{DEST}/SM_SarmMLI')
    if not isinstance(mesh,unreal.StaticMesh):raise RuntimeError(f'MLI mesh import failed: {task.get_editor_property("imported_object_paths")}')
    subsystem=unreal.get_editor_subsystem(unreal.StaticMeshEditorSubsystem)
    subsystem.remove_collisions(mesh)
    settings=subsystem.get_lod_build_settings(mesh,0)
    settings.set_editor_property('build_scale3d',unreal.Vector(1,1,1))
    settings.set_editor_property('recompute_normals',False);settings.set_editor_property('recompute_tangents',True);settings.set_editor_property('use_mikk_t_space',True);settings.set_editor_property('remove_degenerates',False);settings.set_editor_property('generate_lightmap_u_vs',False)
    settings.set_editor_property('use_full_precision_u_vs',True)
    nanite=mesh.get_editor_property('nanite_settings');nanite.set_editor_property('enabled',False);mesh.set_editor_property('nanite_settings',nanite)
    subsystem.set_lod_build_settings(mesh,0,settings)
    slots=mesh.get_editor_property('static_materials');seen=set()
    for index,slot in enumerate(slots):
        name=str(slot.get_editor_property('imported_material_slot_name'))
        role=next((r for r in materials if r.casefold() in name.casefold()),None)
        if role is None:raise RuntimeError(f'Unexpected MLI material slot: {name}')
        mesh.set_material(index,materials[role]);seen.add(role)
        if mesh.get_material(index)!=materials[role]:raise RuntimeError(f'Material slot assignment failed: {role}')
    if seen!=set(materials):raise RuntimeError(f'Incomplete MLI material slots: {seen}')
    save(mesh)
    count=mesh.get_num_triangles(0)
    if count!=manifest['triangles']:raise RuntimeError(f'Blanket triangle loss on import: {count} != {manifest["triangles"]}')
    # Inspect built mesh UV channel and extent, not just the source OBJ.
    uv_count=subsystem.get_num_uv_channels(mesh,0)
    if uv_count<1:raise RuntimeError('Imported blanket has no UV channel')
    bounds=mesh.get_bounds();extent=bounds.box_extent
    expected=[(b-a)*50 for a,b in zip(*manifest['bounds_m'])]
    actual=[extent.x,extent.y,extent.z]
    if any(abs(a-b)>.02 for a,b in zip(sorted(actual),sorted(expected))):raise RuntimeError(f'MLI units/scale mismatch: {actual} vs {expected}')
    report=dict(triangles=count,uv_channels=uv_count,material_slots=sorted(seen),extent_cm=actual,physics_collision='disabled in runtime component; no generated simple collision')
    saved=ROOT/'Saved/AssetImport';saved.mkdir(parents=True,exist_ok=True)
    (saved/'sarm_mli_validation.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    unreal.log('SARM_MLI_BUILD_OK: independent UV mesh, folded hems and material instances validated')


if __name__=='__main__':import_mesh(create_materials())
