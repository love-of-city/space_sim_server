"""Visual-only asset adapter; never supplies physical LRO parameters."""
from copy import deepcopy
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import trimesh

from simulation.scene_library.assembly import _absolutize
from simulation.scene_library.taskbox_mount import _world_pose, _set_pose
from simulation.scene_library.taskbox_recess import clip_surface_pocket


def build(profile, root_dir, output):
    output.mkdir(parents=True, exist_ok=False)
    baseline = root_dir/profile['baseline']
    root = ET.parse(baseline/'preheld-dual.xml').getroot()
    _absolutize(root, baseline/'preheld-dual.xml')
    old_target = root.find('.//body[@name="capture_target"]')
    source = root_dir/profile['source_model']
    lro = ET.parse(source).getroot()
    _absolutize(lro,source)
    scale = profile['scale']['value']
    mount = profile['mount']
    center = np.asarray(mount['center_source'])*scale
    basis = np.asarray(mount['basis_source']).T
    old_box = root.find('.//body[@name="task_box_base"]')
    bp,br = _world_pose(root,old_box)
    work_center = bp+br@np.array([.225,.275,0])
    rotation = br@basis.T
    position = work_center-rotation@center
    target = ET.Element('body',name='capture_target')
    _set_pose(target,position,rotation)
    # Keep only the compiled scene's frame placeholder. It is not an LRO
    # mass/inertia model and the result is explicitly prohibited for dynamics.
    for n in old_target:
        if n.tag in ('inertial','freejoint','joint'):
            target.append(deepcopy(n))
    visual = deepcopy(lro.find('./worldbody/body'))
    changed=[]
    assets={n.get('name'):n for n in lro.find('asset')}
    for node in visual.iter():
        if node.get('name'): node.set('name','variant_'+node.get('name'))
        if node.get('pos'): node.set('pos',' '.join(map(str,np.fromstring(node.get('pos'),sep=' ')*scale)))
        for k in ('mesh','material'):
            if node.get(k): node.set(k,'variant_'+node.get(k))
        if node.tag=='geom':
            node.set('contype','0');node.set('conaffinity','0');node.set('mass','0');node.set('group','2')
    mesh_dir=output/'visual-meshes';mesh_dir.mkdir()
    for oldname,asset in assets.items():
        n=deepcopy(asset);n.set('name','variant_'+oldname)
        if n.tag=='mesh':
            n.set('scale',' '.join([str(scale)]*3))
            if oldname in mount['pocket_meshes']:
                mesh=trimesh.load(n.get('file'),force='mesh',process=False)
                low=np.asarray(mount['pocket_low_m'])/scale
                high=np.asarray(mount['pocket_high_m'])/scale
                cut=clip_surface_pocket(mesh,low,high)
                path=mesh_dir/(oldname+'.stl');cut.export(path)
                n.set('file',str(path.resolve()));changed.append(oldname)
        root.find('asset').append(n)
    target.append(visual)
    for name in ('task_box_base','drawer_base'):
        node=root.find(f'.//body[@name="{name}"]');p,r=_world_pose(root,node)
        copy=deepcopy(node);_set_pose(copy,rotation.T@(p-position),rotation.T@r);target.append(copy)
    world=root.find('worldbody');world.remove(old_target);world.append(target)
    # No false connection of a capture arm to a satellite without a calibrated
    # interface. This is a stationary visual assembly, not a docking scene.
    for section in ('equality','contact'):
        node=root.find(section)
        if node is not None:
            for item in list(node):
                if any('capture_target' in v or 'satellite_' in v for v in item.attrib.values()):node.remove(item)
    # Visual inspection must not load task physics plugins against a different
    # body tree. Keep joint kinematics for the camera/arm pose only.
    ET.indent(root);ET.ElementTree(root).write(output/'preheld-dual.xml',encoding='utf-8',xml_declaration=True)
    for name in ('extension','actuator','sensor','equality','contact','tendon'):
        section=root.find(name)
        if section is not None:root.remove(section)
    for parent in root.iter():
        for node in list(parent):
            if node.tag=='geom' and (node.get('type')=='sdf' or node.find('plugin') is not None):parent.remove(node)
    ET.indent(root);ET.ElementTree(root).write(output/'visual-inspection.xml',encoding='utf-8',xml_declaration=True)
    scene=json.loads((baseline/'initial-scene.json').read_text(encoding='utf-8'))
    from simulation.serial_chain_kinematics import matrix_to_quaternion_wxyz
    scene['randomization'].update(target_position_m=position.tolist(),target_orientation_wxyz=matrix_to_quaternion_wxyz(rotation).tolist())
    scene.pop('preheld_capture',None)
    scene['render_profile']={'visual_attachments':[],'vision_capture':{'fiducial_calibration_valid':False}}
    (output/'initial-scene.json').write_text(json.dumps(scene,indent=2),encoding='utf-8')
    (output/'assembly.json').write_text(json.dumps(dict(visual_only=True,dynamics_allowed=False,
        physical_scale_verified=False,interface_available=False,collision_model_available=False,
        taskbox_world_pose_unchanged=True,modified_visual_meshes=changed,
        limitation='Fixed visual assembly; frame placeholder inertia must not be used for dynamics'),indent=2),encoding='utf-8')
