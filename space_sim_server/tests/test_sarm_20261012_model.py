from pathlib import Path
import xml.etree.ElementTree as E
from tools.assets.build_sarm_20261012 import compose_platform_model
ROOT=Path(__file__).resolve().parents[1]

def test_source_preserved_and_platform_assembly(tmp_path):
 src=ROOT/'model/SARM_mjcf_20261012/SARM_convex.xml'; before=src.read_bytes()
 out=tmp_path/'model.xml'; compose_platform_model(src,ROOT/'model/SARM/platform/sarm_task_box_plugs.xml',out,fastener_model='sdf')
 assert b'\r\n' not in out.read_bytes(), 'Generated XML must have stable LF bytes across platforms'
 a=E.fromstring(before); b=E.parse(out).getroot()
 assert src.read_bytes()==before
 assert E.tostring(a.find('equality'))==E.tostring(b.find('equality'))
 assert E.tostring(a.find('tendon'))==E.tostring(b.find('tendon'))
 assert len(b.findall('.//geom/plugin'))==6  # source five plus the storage-bore female thread
 bus=b.find('.//body[@name="cubesat_bus"]')
 assert bus.find('freejoint') is not None
 assert bus.find('./body[@name="drawer_base"]') is not None
 for n in ['aviation_plug','guide_plug','cylindrical_plug','bolt','fastener']:
  body=b.find('./worldbody/body[@name="'+n+'"]'); assert body is not None
  assert body.find('freejoint') is not None or body.find('joint[@type="free"]') is not None
 assert len(bus.findall('./body[@name="rw_x"]'))==1
 # Normalize the temporary directory's ../ segments before Windows stat:
 # their uncollapsed spelling may exceed MAX_PATH despite a short real path.
 for mesh in b.findall('./asset/mesh[@file]'):assert (out.parent/mesh.get('file')).resolve().is_file()

def test_orbital_free_parts_have_no_world_drag(tmp_path):
 src=ROOT/'model/SARM_mjcf_20261012/SARM_convex.xml'
 out=tmp_path/'model.xml'
 manifest=compose_platform_model(src,ROOT/'model/SARM/platform/sarm_task_box_plugs.xml',out)
 original=E.parse(src); derived=E.parse(out)
 for joint in original.findall('.//joint[@type="free"]'):
  result=derived.find('.//joint[@name="'+joint.get('name')+'"]')
  assert float(result.get('damping','0')) == 0
 # The change concerns absolute free-body drag, not relative mechanism damping.
 for joint in original.findall('.//joint[@damping]'):
  if joint.get('type') != 'free' and joint.get('name'):
   assert derived.find('.//joint[@name="'+joint.get('name')+'"]').get('damping') == joint.get('damping')
 assert {v['joint'] for v in manifest['orbital_free_joint_damping_overrides']} == {
  'aviation_plug_freejoint','guide_plug_freejoint','cylindrical_plug_freejoint'}

def test_all_source_free_parts_inherit_orbit():
 from space_arm_platform.task_box import initialize_free_plugs
 from types import SimpleNamespace
 positions={};velocities={}
 names=['guide_plug','aviation_plug','cylindrical_plug','bolt']
 scene=SimpleNamespace(getBodyNames=lambda:names,getBody=lambda n:SimpleNamespace(setPosition=lambda v:positions.update({n:v}),setVelocity=lambda v:velocities.update({n:v})))
 model=ROOT/'model/SARM/platform/sarm_20261012.xml'
 initialize_free_plugs(scene,model,[100.,200.,300.],[1.,2.,3.])
 assert set(positions)==set(names)==set(velocities)
 for n in names:
  assert velocities[n]==[1.,2.,3.]
  assert min(positions[n])>99.


def _camera_frame(camera):
 import numpy as np
 w,x,y,z=map(float,camera.get('quat').split())
 R=np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],[2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],[2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])
 return np.array(list(map(float,camera.get('pos').split()))),R


def _project(point,position,R,fovy_deg,aspect=1.0):
 import numpy as np
 c=R.T@(np.asarray(point)-position)  # camera frame: -Z forward, +Y up
 depth=-c[2]
 t=np.tan(np.radians(fovy_deg)/2)
 return depth,c[0]/depth/(t*aspect),c[1]/depth/t


def test_wrist_camera_sees_both_fingers_apart_at_every_opening(tmp_path):
 # Mounted on ee_base (turns with joint6). Fingertips open along ee_base x
 # (closed x=+/-.002-.035 m, fully open +/-.053-.085 m, z .144-.209 m). The
 # camera must not look along x, must keep both fingers inside the frame and
 # must put them on opposite image sides with the TCP between them.
 import numpy as np
 out=tmp_path/'model.xml'
 compose_platform_model(ROOT/'model/SARM_mjcf_20261012/SARM_convex.xml',ROOT/'model/SARM/platform/sarm_task_box_plugs.xml',out)
 camera=E.parse(out).find('.//body[@name="ee_base"]/camera[@name="sarm_wrist_cam"]')
 assert camera is not None
 position,R=_camera_frame(camera)
 fovy=float(camera.get('fovy'))
 assert abs(np.dot(-R[:,2],[1,0,0])) < .2  # not along the finger opening axis
 for x in (.002,.035,.053,.085):
  for z in (.144,.209):
   left,right=_project([-x,0,z],position,R,fovy),_project([x,0,z],position,R,fovy)
   for depth,u,v in (left,right):
    assert depth > .05 and abs(u) < 1 and abs(v) < 1
   assert np.sign(left[1]) != np.sign(right[1])
 depth,u,_=_project([0,0,.19],position,R,fovy)
 assert depth > .05 and abs(u) < .05


def test_overview_camera_frames_the_whole_task_box(tmp_path):
 # Task-box AABB in the bus frame, operating pose (measured from the model).
 import itertools
 out=tmp_path/'model.xml'
 compose_platform_model(ROOT/'model/SARM_mjcf_20261012/SARM_convex.xml',ROOT/'model/SARM/platform/sarm_task_box_plugs.xml',out)
 camera=E.parse(out).find('./worldbody/body[@name="cubesat_bus"]/camera[@name="spacecraft_overview"]')
 assert camera is not None
 position,R=_camera_frame(camera)
 # Virtual camera on a 0.5 m mast at deck point B, behind the arm base (-x side).
 assert position[0] < 0 and abs(position[2] - .5) < 1e-9
 assert R[2,1] > 0  # image up points away from -z: the view is not upside down
 assert abs(R[2,0]) < 1e-9  # no roll: image rows stay level with the deck
 # Visible task-box volume: from the deck top (bus z=0; the drawer below it is
 # inside the bus) to the tallest part, with a 10% image margin.
 for corner in itertools.product((.122,.578),(.062,.618),(0.,.149)):
  depth,u,v=_project(corner,position,R,float(camera.get('fovy')))
  assert depth > 0 and abs(u) < .9 and abs(v) < .9


def test_wrist_lamp_is_a_shadowless_spot_light_on_the_camera_axis(tmp_path):
 # Rendering-only lamp: same mount as the wrist camera, shining along its optical
 # axis, cone wide enough for the square image's corners, no shadows (the camera
 # sits at the lamp and never sees them).
 import math
 import numpy as np
 out=tmp_path/'model.xml'
 manifest=compose_platform_model(ROOT/'model/SARM_mjcf_20261012/SARM_convex.xml',ROOT/'model/SARM/platform/sarm_task_box_plugs.xml',out)
 ee=E.parse(out).find('.//body[@name="ee_base"]')
 camera=ee.find('camera[@name="sarm_wrist_cam"]')
 lamps=ee.findall('light')
 assert [l.get('name') for l in lamps]==['wrist_lamp']  # direct child: stays on ee_base across the sensor cut
 lamp=lamps[0]
 position,R=_camera_frame(camera)
 np.testing.assert_allclose([float(v) for v in lamp.get('pos').split()],position,atol=1e-12)
 np.testing.assert_allclose([float(v) for v in lamp.get('dir').split()],-R[:,2],atol=1e-12)  # camera looks along its local -Z
 assert lamp.get('directional') in (None,'false') and lamp.get('mode')=='fixed' and lamp.get('castshadow')=='false'
 corner=math.degrees(math.atan(math.sqrt(2)*math.tan(math.radians(float(camera.get('fovy')))/2)))  # half diagonal of the square image
 cutoff=float(lamp.get('cutoff'))
 assert corner <= cutoff < 90
 # UE spot cone as the adapter builds it: inner 0.65*cutoff, outer cutoff; attenuation
 # = ((cos a - cos outer) / (cos inner - cos outer))^2. The image corners must stay
 # at least half as bright as the centre (a 70 degree cutoff leaves only 19% there).
 c=lambda deg: math.cos(math.radians(deg))
 assert ((c(corner)-c(cutoff))/(c(0.65*cutoff)-c(cutoff)))**2 >= .5
 diffuse=[float(v) for v in lamp.get('diffuse').split()]; specular=[float(v) for v in lamp.get('specular').split()]
 assert min(diffuse)>0 and len(set(diffuse))==1 and all(0<=s<=d for s,d in zip(specular,diffuse))
 assert manifest['wrist_lamp']['rendering_only'] and 'wrist lamp (rendering only)' in manifest['platform_additions']


def test_adapter_turns_the_wrist_lamp_into_a_body_mounted_spot_light(tmp_path):
 import numpy as np
 from bsk_render_adapter.mjcf_assets import parse_mjcf_scene_metadata
 out=tmp_path/'model.xml'
 compose_platform_model(ROOT/'model/SARM_mjcf_20261012/SARM_convex.xml',ROOT/'model/SARM/platform/sarm_task_box_plugs.xml',out)
 metadata=parse_mjcf_scene_metadata(out,'teleop')
 lights={light['visual_id']:light for light in metadata.lights}
 lamp=lights['teleop/light/wrist_lamp']
 camera=next(c for c in metadata.cameras if c['display_name']=='sarm_wrist_cam')
 assert lamp['parent_id']==camera['parent_id']=='teleop/ee_base'
 np.testing.assert_allclose(lamp['position_body_m'],camera['position_body_m'],atol=1e-12)
 from tools.assets.build_sarm_20261012 import WRIST_LIGHT_CUTOFF_DEG,WRIST_LIGHT_DIFFUSE
 assert lamp['properties']['light_type']=='spot' and lamp['properties']['cast_shadows'] is False
 # The cone and brightness reach UE unchanged (UE builds 650 * intensity and cone = cutoff_deg).
 assert lamp['properties']['cutoff_deg']==WRIST_LIGHT_CUTOFF_DEG and lamp['properties']['intensity']==WRIST_LIGHT_DIFFUSE
 # The scene's own lights are untouched: still the two directional ones.
 assert sorted(k for k,v in lights.items() if v['properties']['light_type']=='directional')==['teleop/light/fill_light','teleop/light/main_light']


def test_wrist_sensor_cut_preserves_tool_inertia_and_joint_layout(tmp_path):
 source=ROOT/'model/SARM_mjcf_20261012/SARM_convex.xml'
 out=tmp_path/'sensor.xml'
 manifest=compose_platform_model(source, ROOT/'model/SARM/platform/sarm_task_box_plugs.xml', out)
 original=E.parse(source).find('.//body[@name="ee_base"]')
 derived=E.parse(out)
 flange=derived.find('.//body[@name="ee_base"]')
 tool=flange.find('body[@name="wrist_ft_tool"]')
 assert flange.find('joint').get('name') == 'joint6'
 assert flange.find('inertial') is None
 assert tool.find('joint') is None and tool.find('freejoint') is None
 assert original.find('inertial').attrib == tool.find('inertial').attrib
 assert {g.get('name') for g in original.findall('.//geom')} == {g.get('name') for g in tool.findall('.//geom')}
 for typ in ('force','torque'):
  assert derived.find(f'./sensor/{typ}').get('site') == 'wrist_ft_site'
 assert manifest['wrist_force_torque_sensor']['added_mass_kg'] == 0

 # Every exclusion targeting the moved geometry must still protect that geometry.
 for original_exclusion in E.parse(source).findall('./contact/exclude'):
  if 'ee_base' in (original_exclusion.get('body1'),original_exclusion.get('body2')):
   name=original_exclusion.get('name')
   moved=derived.find(f'./contact/exclude[@name="{name}"]')
   for key in ('body1','body2'):
    assert moved.get(key) == ('wrist_ft_tool' if original_exclusion.get(key)=='ee_base' else original_exclusion.get(key))
