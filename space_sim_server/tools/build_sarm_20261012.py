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

# Platform cameras (not part of the delivered model). Positions in the parent
# body frame: overview on cubesat_bus, wrist on ee_base. "right" is the image
# +x direction before orthogonalization. Fully open fingers reach ee_base
# x=+/-.085 m. Square 1K capture preserves the former horizontal field of
# view and expands the vertical view, retaining both fingertips.
# Overview: a virtual camera at the top of a 0.5 m deck mast (the mast itself is
# not modelled). Deck point B on the rear edge, left-behind the arm base, so the
# image "up" follows the arm reaching towards the task box.
OVERVIEW_CAMERA_POS = (-0.18, 0.27, 0.5)
OVERVIEW_CAMERA_AIM = (0.35, 0.34, 0.03)
OVERVIEW_CAMERA_RIGHT = (0.07, -0.53, 0.0)  # horizontal, perpendicular to the line of sight (no roll)
CAMERA_RESOLUTION = "1024 1024"
OVERVIEW_CAMERA_FOVY = math.degrees(2 * math.atan(math.tan(math.radians(51) / 2) * 16 / 9))
WRIST_CAMERA_POS = (0.0, 0.10, 0.12)
WRIST_CAMERA_AIM = (0.0, 0.0, 0.22)
WRIST_CAMERA_RIGHT = (-1.0, 0.0, 0.0)
WRIST_CAMERA_FOVY = math.degrees(2 * math.atan(math.tan(math.radians(70) / 2) * 16 / 9))

def look_at_quat(pos, aim, right):
    """MJCF camera quaternion (w,x,y,z): local -Z towards aim, local +X ~ right."""
    f=[a-p for a,p in zip(aim,pos)]; n=math.sqrt(sum(v*v for v in f)); f=[v/n for v in f]
    k=sum(a*b for a,b in zip(right,f)); x=[a-k*b for a,b in zip(right,f)]
    n=math.sqrt(sum(v*v for v in x)); x=[v/n for v in x]
    z=[-v for v in f]
    y=[z[1]*x[2]-z[2]*x[1], z[2]*x[0]-z[0]*x[2], z[0]*x[1]-z[1]*x[0]]
    # Columns x, y, z are the camera axes in the parent frame.
    m=[[x[0],y[0],z[0]],[x[1],y[1],z[1]],[x[2],y[2],z[2]]]
    t=m[0][0]+m[1][1]+m[2][2]
    if t>0:
        s=math.sqrt(t+1)*2; q=(s/4,(m[2][1]-m[1][2])/s,(m[0][2]-m[2][0])/s,(m[1][0]-m[0][1])/s)
    else:
        i=max(range(3),key=lambda a:m[a][a]); j,k=(i+1)%3,(i+2)%3
        s=math.sqrt(1+m[i][i]-m[j][j]-m[k][k])*2
        q=[0.0]*4; q[0]=(m[k][j]-m[j][k])/s; q[1+i]=s/4; q[1+j]=(m[j][i]+m[i][j])/s; q[1+k]=(m[k][i]+m[i][k])/s
    if q[0]<0: q=[-v for v in q]
    return tuple(q)

def set_look_at(camera, pos, aim, right):
    text=lambda values: ' '.join(repr(float(v)+0.0) for v in values)  # shortest exact, no -0
    camera.set('pos',text(pos))
    camera.set('quat',text(look_at_quat(pos,aim,right)))

def compose_platform_model(source: Path, platform_reference: Path, output: Path, *, fastener_model='captive') -> dict:
    if fastener_model not in ('captive', 'sdf'):
        raise ValueError('fastener_model must be captive or sdf')
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
    for node in oldbus.findall('camera')+oldbus.findall('site'):
        node=copy.deepcopy(node)
        if node.tag == 'camera': node.set('resolution', CAMERA_RESOLUTION)
        if node.tag=='camera' and node.get('name')=='spacecraft_overview':
            # Frame the whole task box (bus-frame AABB x .12-.58, y .06-.62,
            # z -.09-.15 m) with ~8% margin; the arm stays at the image edge.
            set_look_at(node,OVERVIEW_CAMERA_POS,OVERVIEW_CAMERA_AIM,OVERVIEW_CAMERA_RIGHT)
            node.set('fovy',format(OVERVIEW_CAMERA_FOVY,'.12g'))
        bus.append(node)
    wrist=r.find('.//body[@name="ee_base"]')
    for camera in old.findall('.//body[@name="link6"]/camera'):
        camera = copy.deepcopy(camera)
        camera.set("resolution", CAMERA_RESOLUTION)
        if camera.get('name') == 'sarm_wrist_cam':
            # Mounted on ee_base, so it turns with joint6. The fingers open
            # along ee_base x; viewing from the +y side keeps both fingertips
            # apart in the image (a view along x hides one behind the other)
            # and sees over the linkage bars. Aimed 3 cm past the TCP
            # (ee_base z=.19): whole fingers (z .144-.209) in the lower half,
            # the grasped object between them, the workspace ahead above.
            set_look_at(camera,WRIST_CAMERA_POS,WRIST_CAMERA_AIM,WRIST_CAMERA_RIGHT)
            camera.set('fovy',format(WRIST_CAMERA_FOVY,'.12g'))
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
    if fastener_model == 'captive':
        from tools.captive_fastener import apply_captive_fastener
        fastener = apply_captive_fastener(r, output.parent)
    else:
        fastener = {'model': 'sdf'}
    output.parent.mkdir(parents=True,exist_ok=True);output.write_bytes(E.tostring(r,encoding='utf-8',xml_declaration=True))
    record={'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'model_sha256':hashlib.sha256(output.read_bytes()).hexdigest(),'model_revision':'2026-10-12','platform_additions':['free base','reaction wheels (additional mass)','capture target','cameras'],'source_base_inertia_preserved':True,'gripper_type':'four_bar_tendon','root_renames':{'base_link':'cubesat_bus'},'drawer_reparented_to':'cubesat_bus'}
    record['wrist_force_torque_sensor'] = {'site': 'wrist_ft_site', 'parent_body': 'ee_base', 'tool_body': 'wrist_ft_tool', 'added_mass_kg': 0, 'ideal_rigid_sensor': True}
    record['orbital_free_joint_damping_overrides']=damping_overrides
    record['fastener']=fastener
    output.with_suffix('.manifest.json').write_text(json.dumps(record,indent=2)+'\n')
    return record
if __name__=='__main__':
    import argparse, sys
    sys.path.insert(0, str(ROOT))
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fastener-model', choices=('captive','sdf'), default='captive')
    args=parser.parse_args()
    name='sarm_20261012.xml' if args.fastener_model=='captive' else 'sarm_20261012_sdf.xml'
    print(compose_platform_model(ROOT/'model/SARM_mjcf_20261012/SARM_convex.xml',ROOT/'model/SARM/platform/sarm_task_box_plugs.xml',ROOT/'model/SARM/platform'/name,fastener_model=args.fastener_model))
