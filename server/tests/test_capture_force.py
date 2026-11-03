import numpy as np
from simulation.capture_sequence import CaptureSequence

def feedback(force=5., jaws=2, compression=.001):
 return dict(pad_force_n=force,jaw_support_count=jaws,pad_compression_m=compression,jaw_force_n=force)
def invoke(s,t,f):
 q=np.zeros(12);q[6]=.0024;q[8:]=-.012
 return s.command(t,q,np.array([0.,0.,.0001]),np.eye(3),feedback=f)
def test_force_completion_does_not_require_three_mm():
 s=CaptureSequence(force_control=True);s.phase='tighten'
 invoke(s,1,feedback());a,p=invoke(s,2.1,feedback())
 assert p=='hold_force';assert np.allclose(a['end_effector_linear_velocity_body_m_s'],0)
def test_pad_alone_is_not_capture_and_hold_regulates_force():
 s=CaptureSequence(force_control=True);s.phase='tighten'
 invoke(s,1,feedback(jaws=0));_,p=invoke(s,3,feedback(jaws=0));assert p=='tighten'
 s.phase='hold_force';a,_=invoke(s,4,feedback(force=3));assert a['capture_axial_velocity_m_s']>0
 a,_=invoke(s,5,feedback(force=7));assert a['capture_axial_velocity_m_s']<0
def test_overcompression_aborts_and_position_alone_never_succeeds():
 s=CaptureSequence(force_control=True);s.phase='tighten'
 _,p=invoke(s,1,feedback(force=0));assert p=='tighten'
 _,p=invoke(s,2,feedback(compression=.0021));assert p=='abort'
 assert s.failure_reason=='pad_compression_limit'

def test_contact_feedback_is_json_serializable(tmp_path, monkeypatch):
 from simulation import capture_audit
 import json
 model=tmp_path/'model.xml'
 model.write_text('<mujoco><worldbody><body name="cubesat_bus"><body name="jaw_0"><geom name="collision_jaw_0"/></body></body><body name="capture_target"><geom name="capture_x_piece_0"/></body></worldbody></mujoco>')
 monkeypatch.setattr(capture_audit,'__file__',str(tmp_path/'simulation'/'capture_audit.py'))
 class Fake:
  def body_detail(self,n):return np.array([0,0,0,1,0,0,0,0,0,0,0,0,0,0.])
  def _name(self,t,i):return ['collision_jaw_0','capture_x_piece_0'][i]
  def contact_detail(self):
   a=np.zeros((1,23));a[0,:3]=[0,1,-.0001];a[0,6]=2;a[0,12:15]=[0,0,1];a[0,21]=0;return a
 audit=capture_audit.CaptureAudit(model,Fake());audit.record(1,'tighten',np.zeros(12));json.dumps(audit.feedback);assert audit.feedback['jaw_support_count']==1
 assert audit.feedback.get('peak_target_contact_force_n') == 2.
 audit.file.close()
