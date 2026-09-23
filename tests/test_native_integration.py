import pytest
pytest.importorskip('Basilisk')
from simulation import native_integration
from space_arm_platform.sampling import DYNAMICS_HZ, DEFAULT_IK_HZ, RENDER_HZ


def test_integrator_controls_error_without_altering_clock_or_physics(monkeypatch):
    monkeypatch.delenv(native_integration.RELATIVE_TOLERANCE_ENV, raising=False)
    monkeypatch.delenv(native_integration.ABSOLUTE_TOLERANCE_ENV, raising=False)
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
    assert scene.integrator.relative==1e-4 and scene.integrator.absolute==1e-4
    assert report['collision_authority']=='running_native_MJScene'
    assert (DYNAMICS_HZ,DEFAULT_IK_HZ,RENDER_HZ)==(240,120,30)


def test_integrator_tolerances_can_be_overridden_and_reject_bad_values(monkeypatch):
    class Integrator:
        def __init__(self, scene): self.scene = scene
        def setRelativeTolerance(self, value): self.relative = value
        def setAbsoluteTolerance(self, value): self.absolute = value
    class Scene:
        def setIntegrator(self, value): self.integrator = value
    monkeypatch.setattr(native_integration, "_create_integrator", Integrator)
    monkeypatch.setenv(native_integration.RELATIVE_TOLERANCE_ENV, "2e-5")
    monkeypatch.setenv(native_integration.ABSOLUTE_TOLERANCE_ENV, "3e-6")
    scene = Scene()
    report = native_integration.configure_scene_integrator(scene)
    assert report["relative_tolerance"] == 2e-5
    assert report["absolute_tolerance"] == 3e-6
    monkeypatch.setenv(native_integration.RELATIVE_TOLERANCE_ENV, "0")
    with pytest.raises(ValueError, match="finite positive"):
        native_integration.configure_scene_integrator(Scene())
