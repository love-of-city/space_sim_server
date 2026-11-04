import math

from space_arm_platform.scene_targets import capture_target


def test_sdf_and_demo_are_explicit_scene_choices():
    from space_arm_platform.scene_runtime import SCENE_TEMPLATES, template_default_randomization_profile
    from space_arm_platform.control_defaults import ZERO_START_TELEOP_PROFILE
    choices = {item['id'] for item in SCENE_TEMPLATES}
    for name in ('sarm-task-box-20261012-sdf', 'sarm-task-box-20261012-demo'):
        assert name in choices
        assert template_default_randomization_profile(name) == ZERO_START_TELEOP_PROFILE
    assert capture_target('sarm-task-box-20261012-sdf').model_file == 'sarm_20261012_sdf.xml'
    assert capture_target('sarm-task-box-20261012-demo').model_file == 'sarm_20261012.xml'


def test_demo_torque_is_bounded_and_has_release_and_hold_phases():
    from simulation.fastener_demo_schedule import demo_command
    assert demo_command(0) == ('settle', 0.)
    assert demo_command(4)[0] == 'tighten'
    assert demo_command(7) == ('hold_tight', 0.)
    assert demo_command(10)[0] == 'loosen'
    assert demo_command(15) == ('hold_loose', 0.)
    assert demo_command(20) == demo_command(4)
    assert all(math.isfinite(demo_command(i/240)[1]) and -.15 <= demo_command(i/240)[1] <= .06 for i in range(8000))
