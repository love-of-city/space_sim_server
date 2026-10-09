import importlib.util
import inspect
import tempfile
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from bsk_render_adapter import parse_mjcf_scene_metadata


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCENARIO_PATH = PROJECT_ROOT / "examples" / "scenario_orbital_grasp_unreal.py"


def _load_scenario():
    specification = importlib.util.spec_from_file_location("orbital_grasp_contract", SCENARIO_PATH)
    module = importlib.util.module_from_spec(specification)
    assert specification.loader is not None
    specification.loader.exec_module(module)
    return module


class OrbitalGraspContractTests(unittest.TestCase):
    def test_run_exposes_generic_recording_mode(self):
        scenario = _load_scenario()
        parameters = inspect.signature(scenario.run).parameters
        self.assertIn("recording_path", parameters)
        self.assertIn("recording_only", parameters)
        self.assertIsNone(parameters["recording_path"].default)
        self.assertFalse(parameters["recording_only"].default)

        launcher = (PROJECT_ROOT / "scripts" / "run_orbital_grasp.ps1").read_text(encoding="utf-8-sig")
        self.assertIn("[string]$RecordingPath", launcher)
        self.assertIn("[double]$PlaybackRate", launcher)
        self.assertIn("[switch]$ReuseRecording", launcher)
        self.assertIn("--recording $resolvedRecordingPath --record-only", launcher)
        self.assertIn("-ReplayPath $resolvedRecordingPath", launcher)

    def test_orbital_scene_uses_basilisk_sun_ephemeris_for_ue_lighting(self):
        source = SCENARIO_PATH.read_text(encoding="utf-8")
        self.assertIn("gravity_factory.createSun()", source)
        self.assertIn("gravity_factory.createSpiceInterface", source)
        self.assertIn('ephemeris.zeroBase = "Earth"', source)
        self.assertIn("scene.AddModelToDynamicsTask(ephemeris, 75)", source)
        self.assertIn("[earth, sun]", source)
        self.assertIn('"drives_directional_light": True', source)
        self.assertIn("fill_light_intensity_lux=0.0", source)

    def test_rendezvous_reference_is_smooth_and_covers_visible_distance(self):
        scenario = _load_scenario()
        self.assertEqual(scenario.MISSION_TIME_STEP_S, 0.002)
        self.assertEqual(1.0 / scenario.MISSION_TIME_STEP_S, 500.0)
        before = scenario._quintic_rendezvous_reference(
            scenario.RENDEZVOUS_START_S - 1.0
        )
        start = scenario._quintic_rendezvous_reference(scenario.RENDEZVOUS_START_S)
        middle = scenario._quintic_rendezvous_reference(
            0.5 * (scenario.RENDEZVOUS_START_S + scenario.RENDEZVOUS_STOP_S)
        )
        stop = scenario._quintic_rendezvous_reference(scenario.RENDEZVOUS_STOP_S)
        after = scenario._quintic_rendezvous_reference(
            scenario.RENDEZVOUS_STOP_S + 1.0
        )
        self.assertEqual(before, (0.0, 0.0, 0.0))
        self.assertEqual(start, (0.0, 0.0, 0.0))
        self.assertAlmostEqual(middle[0], 0.5 * scenario.RENDEZVOUS_INITIAL_OFFSET_M)
        self.assertGreater(middle[1], 0.0)
        self.assertAlmostEqual(stop[0], scenario.RENDEZVOUS_INITIAL_OFFSET_M)
        self.assertEqual(stop[1:], (0.0, 0.0))
        self.assertEqual(after, stop)
        self.assertGreaterEqual(scenario.RENDEZVOUS_INITIAL_OFFSET_M, 0.5)
        self.assertGreater(
            float(scenario.np.linalg.norm(scenario.EXTENDED_ALIGNED_ARM[1:4])),
            0.05,
        )
        self.assertLessEqual(scenario.TIGHT_GRIP_JOINT_RAD, -0.16)
        self.assertGreater(
            float(scenario.np.linalg.norm(
                scenario.RETRACTED_GRASP_ARM[:5] - scenario.EXTENDED_ALIGNED_ARM[:5]
            )),
            0.45,
        )

    def test_derived_mjcf_adds_native_wheels_and_maneuver_actuator(self):
        scenario = _load_scenario()
        model_root = PROJECT_ROOT.parents[1] / "test" / "model" / "spacecraft_and_arm"
        source = model_root / "assets" / "cubesat_so101_grasp" / "cubesat_so101_grasp.xml"
        mesh_directory = model_root / "assets" / "robotstudio_so101" / "assets"
        self.assertTrue(source.is_file(), "Repository model is missing; check the checkout and Git LFS.")
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "orbital.xml"
            scenario.build_orbital_mjcf(source, output, mesh_directory)
            root = ET.parse(output).getroot()
            metadata = parse_mjcf_scene_metadata(output, "orbital_grasp")
        bodies = {body.get("name") for body in root.findall(".//body")}
        actuators = {actuator.get("name") for actuator in root.findall("./actuator/*")}
        docking_camera = root.find(".//body[@name='cubesat_bus']/camera[@name='cubesat_docking_camera']")
        self.assertTrue({"orbit_rw_x", "orbit_rw_y", "orbit_rw_z"}.issubset(bodies))
        self.assertIsNotNone(docking_camera)
        self.assertEqual(docking_camera.get("pos"), "0.06 -0.08 0.19")
        self.assertEqual(docking_camera.get("resolution"), "1280 720")
        self.assertTrue(
            {
                "orbit_rw_x_motor",
                "orbit_rw_y_motor",
                "orbit_rw_z_motor",
                "docking_approach_thruster",
                "docking_braking_thruster",
            }.issubset(actuators)
        )
        compiler = root.find("./compiler")
        self.assertEqual(Path(compiler.get("meshdir")).resolve(), mesh_directory.resolve())
        self.assertAlmostEqual(float(scenario.np.linalg.norm(scenario.DOCKING_AXIS_BODY)), 1.0)
        camera = next(
            item for item in metadata.cameras
            if item["display_name"] == "cubesat_docking_camera"
        )
        w, x, y, z = camera["orientation_body_from_camera_wxyz"]
        forward_body = np.array([
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y + w * z),
            2.0 * (x * z - w * y),
        ])
        self.assertGreater(float(np.dot(forward_body, scenario.DOCKING_AXIS_BODY)), 0.999999)

    def test_orbit_metric_detects_prograde_energy_gain(self):
        scenario = _load_scenario()
        mu = 3.986004418e14
        radius = 6_878_136.6
        circular_speed = (mu / radius) ** 0.5
        _, before = scenario._specific_orbit(
            scenario.np.array([-radius, 0.0, 0.0]),
            scenario.np.array([0.0, -circular_speed, 0.0]),
            mu,
        )
        _, after = scenario._specific_orbit(
            scenario.np.array([-radius, 0.0, 0.0]),
            scenario.np.array([0.0, -(circular_speed + 0.01), 0.0]),
            mu,
        )
        self.assertGreater(after, before)


if __name__ == "__main__":
    unittest.main()
