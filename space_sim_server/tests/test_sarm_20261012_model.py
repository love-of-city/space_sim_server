from pathlib import Path
import xml.etree.ElementTree as E
from tools.build_sarm_20261012 import compose_platform_model
ROOT=Path(__file__).resolve().parents[1]

def test_source_preserved_and_platform_assembly(tmp_path):
 src=ROOT/'model/SARM_mjcf_20261012/SARM_convex.xml'; before=src.read_bytes()
 out=tmp_path/'model.xml'; compose_platform_model(src,ROOT/'model/SARM/platform/sarm_task_box_plugs.xml',out,fastener_model='sdf')
 assert b'\r\n' not in out.read_bytes(), 'Generated XML must have stable LF bytes across platforms'
 a=E.fromstring(before); b=E.parse(out).getroot()
 assert src.read_bytes()==before
 assert E.tostring(a.find('equality'))==E.tostring(b.find('equality'))
 assert E.tostring(a.find('tendon'))==E.tostring(b.find('tendon'))
 assert len(b.findall('.//geom/plugin'))==5
 bus=b.find('.//body[@name="cubesat_bus"]')
 assert bus.find('freejoint') is not None
 assert bus.find('./body[@name="drawer_base"]') is not None
 for n in ['aviation_plug','guide_plug','cylindrical_plug','bolt','fastener']:
  body=b.find('./worldbody/body[@name="'+n+'"]'); assert body is not None
  assert body.find('freejoint') is not None or body.find('joint[@type="free"]') is not None
 assert len(bus.findall('./body[@name="rw_x"]'))==1
 for mesh in b.findall('./asset/mesh[@file]'):assert (out.parent/mesh.get('file')).is_file()

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


def test_wrist_camera_centers_new_gripper_working_region(tmp_path):
 import numpy as np
 out=tmp_path/'model.xml'
 compose_platform_model(ROOT/'model/SARM_mjcf_20261012/SARM_convex.xml',ROOT/'model/SARM/platform/sarm_task_box_plugs.xml',out)
 camera=E.parse(out).find('.//body[@name="ee_base"]/camera[@name="sarm_wrist_cam"]')
 w,x,y,z=map(float,camera.get('quat').split())
 forward=-np.array([2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)])
 position=np.array(list(map(float,camera.get('pos').split())))
 # New ee TCP is z=.19 m; center the view 4 cm ahead of the fingers.
 expected=np.array([0.,0.,.23])-position
 expected/=np.linalg.norm(expected)
 assert np.dot(forward,expected) > .9999
 assert float(camera.get('fovy')) == 60
