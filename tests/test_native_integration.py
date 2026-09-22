import pytest
pytest.importorskip('Basilisk')
from simulation import native_integration
from space_arm_platform.sampling import DYNAMICS_HZ, DEFAULT_IK_HZ, RENDER_HZ


def test_integrator_controls_error_without_altering_clock_or_physics(monkeypatch):
    class Integrator:
        def __init__(self,scene):self.scene=scene
        def setRelativeTolerance(self,value):self.relative=value
        def setAbsoluteTolerance(self,value):self.absolute=value
    class Scene:
        def setIntegrator(self,value):self.integrator=value
    monkeypatch.setattr(native_integration,'_create_integrator',Integrator)
    scene=Scene();report=native_integration.configure_scene_integrator(scene)
    assert scene._teleop_integrator is scene.integrator
    assert scene.integrator.scene is scene
    assert scene.integrator.relative==1e-5 and scene.integrator.absolute==1e-6
    assert report['collision_authority']=='running_native_MJScene'
    assert (DYNAMICS_HZ,DEFAULT_IK_HZ,RENDER_HZ)==(240,120,30)
