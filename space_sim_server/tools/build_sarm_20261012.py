"""Compose the delivered SARM assembly with the platform's orbital attachments."""
from pathlib import Path
import copy, hashlib, json, os, math
import xml.etree.ElementTree as E
ROOT=Path(__file__).resolve().parents[1]

def asset_path(path: Path, directory: Path) -> str:
    try:
        return os.path.relpath(path.resolve(), directory.resolve()).replace('\\', '/')
    except ValueError:  # Windows checkouts and temporary output may use different drives.
        return path.resolve().as_posix()

def compose_platform_model(source: Path, platform_reference: Path, output: Path) -> dict:
    source,platform_reference,output=map(Path,(source,platform_reference,output))
    r=E.parse(source).getroot(); old=E.parse(platform_reference).getroot()
    r.set('model','SARM_20261012_platform')
    c=r.find('compiler'); meshdir=c.get('meshdir',''); c.set('meshdir','.'); c.set('fusestatic','false')
    # The platform IK/renderer consumes quaternions. MuJoCo lowercase Euler axes
    # are intrinsic; scipy uses uppercase for the same convention.
    if c.get('eulerseq', 'xyz') != 'xyz':
        raise ValueError('Only source intrinsic xyz Euler sequence is supported')
    for node in r.iter():
        if 'euler' in node.attrib:
            angles = [float(v) for v in node.attrib.pop('euler').split()]
            if c.get('angle', 'degree') == 'degree': angles = list(map(math.radians, angles))
            x,y,z = [v/2 for v in angles]
            cx,cy,cz = map(math.cos,(x,y,z)); sx,sy,sz = map(math.sin,(x,y,z))
            q = (cx*cy*cz-sx*sy*sz, sx*cy*cz+cx*sy*sz, cx*sy*cz-sx*cy*sz, cx*cy*sz+sx*sy*cz)
            node.set('quat', ' '.join(format(v, '.17g') for v in q))
    assets=r.find('asset')
    for mesh in assets.findall('mesh[@file]'):
        mesh.set('file',asset_path(source.parent/meshdir/mesh.get('file'),output.parent))
    world=r.find('worldbody'); bus=world.find('body[@name="base_link"]');bus.set('name','cubesat_bus')
    # A free joint's damping is drag against the world, not damping relative
    # to its socket. It must not brake a seated part during common orbital
    # drift. Preserve all relative mechanism damping and the original file.
    damping_overrides=[]
    for joint in world.findall('.//joint[@type="free"]'):
        damping_overrides.append({'joint':joint.get('name'),'source':float(joint.get('damping','0')),'platform':0.0})
        joint.set('damping','0')
    bus.insert(0,E.Element('freejoint',name='cubesat_free'))
    # The fixed base was at identity; this preserves the drawer's assembly transform.
    drawer=world.find('body[@name="drawer_base"]');world.remove(drawer);bus.append(drawer)
    for e in r.iter():
        for attr in ('body1','body2'):
            if e.get(attr)=='base_link':e.set(attr,'cubesat_bus')
    oldbus=old.find('.//body[@name="cubesat_bus"]')
    for n in ['rw_x','rw_y','rw_z']:bus.append(copy.deepcopy(oldbus.find('body[@name="'+n+'"]')))
    for node in oldbus.findall('camera')+oldbus.findall('site'):bus.append(copy.deepcopy(node))
    wrist=r.find('.//body[@name="ee_base"]')
    for camera in old.findall('.//body[@name="link6"]/camera'):
        camera = copy.deepcopy(camera)
        if camera.get('name') == 'sarm_wrist_cam':
            # New TCP is ee_base+[0,0,.19], not the old link6 frame.
            # Keep the clear side mount; look at [0,0,.23], 4 cm ahead
            # of the fingers. MuJoCo camera forward is local -Z.
            camera.set('pos', '0.22 0 0.12')
            qw = math.sqrt(1 - 1/math.sqrt(5))/2
            qx = math.sqrt(1 + 1/math.sqrt(5))/2
            camera.set('quat', ' '.join(format(v, '.17g') for v in (qw,qx,qx,qw)))
        wrist.append(camera)
    # Ideal, rigid, zero-added-mass load cell after joint6. Keep the joint and
    # camera names on ee_base; move the entire tool inertia/geometry/subtree
    # across the measurement cut without changing any transforms or DOFs.
    tool = E.SubElement(wrist, 'body', name='wrist_ft_tool', pos='0 0 0')
    for child in list(wrist):
        if child is not tool and child.tag in ('inertial', 'geom', 'body'):
            wrist.remove(child)
            tool.append(child)
            if child.tag == 'inertial':
                child.tail = "\n                  "
    # Body-pair exclusions follow the original geoms across the fixed cut.
    # Joint and camera references stay on ee_base.
    for exclusion in r.findall('./contact/exclude'):
        for attribute in ('body1', 'body2'):
            if exclusion.get(attribute) == 'ee_base':
                exclusion.set(attribute, 'wrist_ft_tool')
    E.SubElement(tool, 'site', name='wrist_ft_site', pos='0 0 0',
                 size='0.003', rgba='0 0 0 0', group='4')
    sensors = r.find('sensor')
    if sensors is None:
        sensors = E.SubElement(r, 'sensor')
    E.SubElement(sensors, 'force', name='wrist_ft_force', site='wrist_ft_site')
    E.SubElement(sensors, 'torque', name='wrist_ft_torque', site='wrist_ft_site')
    r.find('.//site[@name="ee_site"]').set('name','sarm_ee')
    # Retain the platform's separate capture target as an explicitly additional body.
    target=copy.deepcopy(old.find('.//body[@name="capture_target"]'));world.append(target)
    used={g.get('mesh') for g in target.findall('.//geom') if g.get('mesh')}
    oldmeshdir=old.find('compiler').get('meshdir','')
    for mesh in old.findall('./asset/mesh'):
        if mesh.get('name') in used:
            m=copy.deepcopy(mesh);m.set('file',asset_path(platform_reference.parent/oldmeshdir/m.get('file'),output.parent));assets.append(m)
    r.append(copy.deepcopy(old.find('custom')))
    for a in old.find('actuator'):
        if a.get('name','').startswith('rw_'):r.find('actuator').append(copy.deepcopy(a))
    for pair in old.findall('./contact/pair'):r.find('contact').append(copy.deepcopy(pair))
    output.parent.mkdir(parents=True,exist_ok=True);output.write_bytes(E.tostring(r,encoding='utf-8',xml_declaration=True))
    record={'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'model_sha256':hashlib.sha256(output.read_bytes()).hexdigest(),'model_revision':'2026-10-12','platform_additions':['free base','reaction wheels (additional mass)','capture target','cameras'],'source_base_inertia_preserved':True,'gripper_type':'four_bar_tendon','root_renames':{'base_link':'cubesat_bus'},'drawer_reparented_to':'cubesat_bus'}
    record['wrist_force_torque_sensor'] = {'site': 'wrist_ft_site', 'parent_body': 'ee_base', 'tool_body': 'wrist_ft_tool', 'added_mass_kg': 0, 'ideal_rigid_sensor': True}
    record['orbital_free_joint_damping_overrides']=damping_overrides
    output.with_suffix('.manifest.json').write_text(json.dumps(record,indent=2)+'\n')
    return record
if __name__=='__main__':
    print(compose_platform_model(ROOT/'model/SARM_mjcf_20261012/SARM_convex.xml',ROOT/'model/SARM/platform/sarm_task_box_plugs.xml',ROOT/'model/SARM/platform/sarm_20261012.xml'))
