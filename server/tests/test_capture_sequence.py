import numpy as np
from simulation.capture_sequence import CaptureSequence

def test_open_extend_before_approach_and_no_weld_commands():
 s=CaptureSequence();q=np.zeros(12);q[6]=.02;q[8:]=-.012
 for t in (0,10,19):
  a,phase=s.command(t,q,np.array([0.,0.,.2735]),np.eye(3))
  assert np.allclose(a['end_effector_linear_velocity_body_m_s'],0)
 a,phase=s.command(22,q,np.array([0.,0.,.2735]),np.eye(3));assert phase=='open';assert a['capture_radial_velocity_m_s']>0
 a,phase=s.command(40,q,np.array([0.,0.,.2735]),np.eye(3));assert phase=='abort';assert np.allclose(a['end_effector_linear_velocity_body_m_s'],0)

def test_close_requires_full_insertion():
 s=CaptureSequence();s.phase='approach';s.start=0;q=np.zeros(12);q[6]=-.006
 a,phase=s.command(1,q,np.array([0.,0.,.02]),np.eye(3));assert phase=='approach';assert a['capture_radial_velocity_m_s']==0
 a,phase=s.command(2,q,np.array([0.,0.,.006]),np.eye(3));assert phase=='close'
 assert a['capture_radial_velocity_m_s']==0
 a,phase=s.command(3,q,np.array([0.,0.,.006]),np.eye(3));assert a['capture_radial_velocity_m_s']<0

def test_full_order_uses_measured_tool_positions():
 s=CaptureSequence();q=np.zeros(12);q[6]=.02;q[8:]=-.012
 s.command(20,q,np.array([0,0,.2735]),np.eye(3))
 q[8:]=0
 _,phase=s.command(24,q,np.array([0,0,.2735]),np.eye(3));assert phase=='extend'
 _,phase=s.command(25,q,np.array([0,0,.2735]),np.eye(3));assert phase=='extend'
 q[6]=-.006
 _,phase=s.command(34,q,np.array([0,0,.2735]),np.eye(3));assert phase=='approach'
 _,phase=s.command(100,q,np.array([0,0,.006]),np.eye(3));assert phase=='close'
 q[8:]=-.012
 _,phase=s.command(106,q,np.array([0,0,.006]),np.eye(3));assert phase=='tighten'
 q[6]=.0031
 _,phase=s.command(116,q,np.array([0,0,.006]),np.eye(3));assert phase=='hold'
