"""An intentional mating contact must belong to the active interface only."""
from pathlib import Path
import numpy as np
import pytest
from simulation.scripted_tasks.drawer_initialization import NativePoseProbe


@pytest.mark.parametrize('pickup,geom,expected',[
    (True,'own_pickup',False),(True,'other_pickup',True),(True,'unknown',True),
    (False,'cylindrical_bore_40p5_000',False),(False,'cylindrical_bore_40p1_000',True)])
def test_axial_socket_exemption_is_owned(pickup,geom,expected):
    probe=NativePoseProbe.__new__(NativePoseProbe)
    probe.home=np.zeros(8);probe.q=probe.home.copy()
    probe.names={0:'cylindrical_plug',1:'asm_board2' if pickup else 'asm_board1'}
    probe._mount_ownership={'own_pickup':'cylindrical_pickup','other_pickup':'guide_pickup'}
    probe.geom_name=lambda i:'plug' if i==0 else geom
    probe.contacts=lambda q:[(0,1,0,1,-.001)]
    hit=probe.pose_collision(np.zeros(8),'cylindrical_plug_insert',
        axial_socket=('cylindrical',pickup,'cylindrical_insert_40p5'))
    assert (hit is not None)==expected


@pytest.mark.parametrize('geom,blocked', [
    ('service_cylindrical_bore_40p5_000', False),
    ('cylindrical_bore_40p5_000', True),
    ('service_cylindrical_bore_40p1_000', True),
    ('service_guide_bore_40p5_000', True),
])
def test_service_recipient_exempts_only_selected_bore(geom, blocked):
    probe=NativePoseProbe.__new__(NativePoseProbe)
    probe.home=np.zeros(8);probe.q=probe.home.copy()
    probe.names={0:'cylindrical_plug',1:'service_board1_plug_bore_colliders'}
    probe.geom_name=lambda i:'plug' if i==0 else geom
    probe.contacts=lambda q:[(0,1,0,1,-.001)]
    hit=probe.pose_collision(np.zeros(8),'cylindrical_plug_insert',
        axial_socket=('cylindrical',False,'service_cylindrical_insert_40p5'))
    assert (hit is not None)==blocked
