"""Run real method processes to test wire boundaries, timeout and ownership."""
import json
import math
from pathlib import Path
import time

import psutil
import pytest

from space_arm_platform.benchmark_attitude import AttitudeContext, AttitudeObservation
from space_arm_platform.benchmark_wheel_recovery import WheelRecoveryContext, WheelRecoveryObservation
from space_arm_platform.benchmark_orbit import OrbitContext, OrbitObservation
from space_arm_platform.benchmark_avoidance import AvoidanceContext, AvoidanceObservation
from space_arm_platform.benchmark_navigation import NavigationContext, GNSSMeasurements
from space_arm_platform.benchmark_navigation_recovery import NavigationRecoveryContext, NavigationRecoveryObservation
from space_arm_platform.benchmark_relative_flight import RelativeFlightContext
from space_arm_platform.benchmark_flyaround import FlyAroundContext
from space_arm_platform.benchmark_formation import FormationContext, FormationObservation
from space_arm_platform.benchmark_pointing import PointingContext, PointingObservation
from space_arm_platform.benchmark_target_pointing import TargetPointingContext, TargetPointingObservation
from space_arm_platform.benchmark_contract import MethodContext, NavigationObservation, SPEC_DIRECTORY, load_task
from space_arm_platform.benchmark_external import (
    ExternalMethod, ExternalMethodDefinition, MAX_STDERR_BYTES, decode_message,
)
from space_arm_platform.benchmark_registry import method_classes, method_ids, parse_action

SERVER = Path(__file__).resolve().parents[1]


def assert_process_exited(identity):
    # Windows job termination is asynchronous. A bare PID check can also see a
    # different process created by another xdist worker after PID reuse.
    try:
        process = psutil.Process(identity["pid"])
        if process.create_time() != identity["created"]:
            return
        process.wait(timeout=5)
    except psutil.NoSuchProcess:
        return


def context_and_observation(family):
    if family == "navigation-recovery":
        task = load_task(SPEC_DIRECTORY/'s23-gnss-outage-recovery-v1.json')
        context = NavigationRecoveryContext(task=task, chaser_mass_kg=170.)
        observation = NavigationRecoveryObservation(episode_id='test', sequence=0, sample_time_ns=0, delivery_time_ns=0,
            chaser_measurement_j2000=(7e6, 0., 0., 0., 7500., 0.), target_measurement_j2000=None, receiver_available=True)
    elif family == "debris-avoidance":
        task = load_task(SPEC_DIRECTORY/'s16-catalog-debris-avoidance-v1.json')
        context = AvoidanceContext(task=task,chaser_mass_kg=170.)
        observation = AvoidanceObservation(episode_id='test',sequence=0,sample_time_ns=0,delivery_time_ns=0,
            chaser_estimate_j2000=(7e6,0.,0.,0.,7500.,0.), debris_catalog_j2000=(7e6+5.,0.,-1500.,0.,7500.,10.),
            catalog_sample_time_ns=0,catalog_delivery_time_ns=0)
    elif family in {"attitude", "pointing", "target-pointing", "wheel-recovery"}:
        task = load_task(SPEC_DIRECTORY / "s02-wheel-attitude-v1.json")
        context = AttitudeContext(task=task, chaser_mass_kg=170., inertia_com_body_kg_m2=((1., 0., 0.), (0., 2., 0.), (0., 0., 3.)),
            wheel_spin_inertia_kg_m2=(.01, .01, .01), hardware_torque_limits_nm=(.2, .2, .2),
            hardware_speed_limits_rad_s=(628., 628., 628.))
        observation = AttitudeObservation(episode_id="test", sequence=0, sample_time_ns=0, delivery_time_ns=0,
            orientation_body_to_j2000_wxyz=(math.cos(.1), math.sin(.1), 0., 0.),
            angular_velocity_body_rad_s=(.001, .002, -.001), wheel_speed_rad_s=(0., 0., 0.), gyro_valid=True)
        if family == "wheel-recovery":
            task = load_task(SPEC_DIRECTORY / "s22-wheel-effectiveness-recovery-v1.json")
            context = WheelRecoveryContext(task=task, **context.model_dump(exclude={"task"}))
            observation = WheelRecoveryObservation(**observation.model_dump(exclude={"schema_version"}),
                measured_motor_torque_xyz_nm=(0., 0., 0.))
        if family in {"pointing", "target-pointing"}:
            task = load_task(SPEC_DIRECTORY / "s06-earth-pointing-v1.json")
            context = PointingContext(task=task, **context.model_dump(exclude={"task"}))
            observation = PointingObservation(**observation.model_dump(exclude={"schema_version"}),
                chaser_gnss_j2000=(6878136.6,0.,0.,0.,7500.,0.),
                ephemeris=dict(earth_j2000_to_fixed=((1.,0.,0.),(0.,1.,0.),(0.,0.,1.)),
                    earth_j2000_to_fixed_rate_s=((0.,0.,0.),(0.,0.,0.),(0.,0.,0.)),
                    sun_geocentric_j2000=(1.5e11,0.,0.,0.,30000.,0.)))
        if family == "target-pointing":
            task = load_task(SPEC_DIRECTORY/'d11-cooperative-target-pointing-v1.json')
            context = TargetPointingContext(task=task, **context.model_dump(exclude={'task'}))
            observation = TargetPointingObservation(**observation.model_dump(exclude={'schema_version'}),
                target_gnss_j2000=(6878136.6,500.,100.,0.,7500.,.4))
    elif family in {"formation", "cooperative-rendezvous"}:
        task = load_task(SPEC_DIRECTORY / ("d19-gnss-cooperative-rendezvous-v1.json" if family == "cooperative-rendezvous" else "m02-gnss-formation-hold-v1.json"))
        context = FormationContext(task=task, spacecraft_masses_kg=(170.,)*task.spacecraft_count)
        observation = FormationObservation(episode_id="test", sequence=0, sample_time_ns=0, delivery_time_ns=0,
            estimates_j2000=((7e6, 0., 0., 0., 7500., 0.), (7e6, -150., 0., 0., 7500., 0.),
                              (7e6+130., 75., 0., 0., 7500., 0.))[:task.spacecraft_count])
    elif family == "navigation":
        task = load_task(SPEC_DIRECTORY / "s14-gnss-state-estimation-v1.json")
        context = NavigationContext(task=task, chaser_mass_kg=170.)
        observation = GNSSMeasurements(episode_id="test", sequence=0, sample_time_ns=0, delivery_time_ns=0,
            chaser_measurement_j2000=(7e6, 0., 0., 0., 7500., 0.), target_measurement_j2000=None)
    elif family in {"orbit", "low-thrust", "orbit-plane"}:
        task = load_task(SPEC_DIRECTORY / ("s10-continuous-low-thrust-v1.json" if family == "low-thrust" else "s12-gnss-orbit-plane-v1.json" if family == "orbit-plane" else "s09-finite-burn-v1.json"))
        context = OrbitContext(task=task, chaser_mass_kg=170.)
        observation = OrbitObservation(episode_id="test", sequence=10, sample_time_ns=5_000_000_000, delivery_time_ns=5_000_000_000,
            chaser_estimate_j2000=(7e6, 0., 0., 0., 7500., 0.))
    else:
        filename, context_type = {
            "relative-flight": ("d29-vbar-gnss-approach-v1.json", RelativeFlightContext),
            "flyaround": ("d33-gnss-flyaround-v1.json", FlyAroundContext),
            "waypoint": ("d32-cooperative-gnss-v1.json", MethodContext),
        }[family]
        task = load_task(SPEC_DIRECTORY / filename)
        context = context_type(task=task, chaser_mass_kg=170.)
        observation = NavigationObservation(episode_id="test", sequence=0, sample_time_ns=0, delivery_time_ns=0,
            target_estimate_j2000=(7e6, 0., 0., 0., 7500., 0.), chaser_estimate_j2000=(7e6, -160., 0., 0., 7500., 0.))
    return context, observation


@pytest.mark.parametrize("family", ["attitude", "waypoint", "orbit", "low-thrust", "orbit-plane", "navigation", "navigation-recovery", "relative-flight", "flyaround", "formation", "cooperative-rendezvous", "pointing", "target-pointing", "wheel-recovery", "debris-avoidance"])
def test_sdk_process_matches_builtin_without_loading_simulator(tmp_path, family):
    context, observation = context_and_observation(family)
    filename = {"target-pointing": "target-pointing.json", "orbit-plane": "orbit-pd.json", "low-thrust": "orbit-pd.json", "navigation": "navigation-estimator.json", "navigation-recovery": "navigation-recovery.json", "relative-flight": "relative-flight.json", "flyaround": "flyaround.json", "formation": "formation.json", "cooperative-rendezvous": "formation.json", "pointing": "pointing.json", "wheel-recovery": "wheel-recovery.json", "debris-avoidance": "debris-avoidance.json"}.get(family, f"{family}-pd.json")
    definition = ExternalMethodDefinition(SERVER / "examples/autonomy_methods" / filename, context.task)
    method = ExternalMethod(definition, tmp_path)
    baseline = method_classes(context.task)[method_ids(context.task)[0]]()
    baseline.reset(context)
    try:
        method.reset(context)
        identity = {"pid": method.process.pid, "created": method.owner.created}
        action = parse_action(context.task, method.act(observation), observation)
        assert action == baseline.act(observation)
    finally:
        method.close()
    assert_process_exited(identity)
    calls = [json.loads(line) for line in (tmp_path / "method_calls.jsonl").read_text().splitlines()]
    assert [row["operation"] for row in calls] == ["reset", "act", "close"]
    assert method.telemetry()["call_count"] == 3
    assert all(row["status"] == "ok" and row["wall_time_s"] >= 0 for row in calls)
    assert method.telemetry()["total_wall_time_s"] > 0


WORKER = r'''
import json, sys, time, subprocess, psutil
from pathlib import Path
mode = sys.argv[1]
for line in sys.stdin:
    request = json.loads(line)
    reply = {"protocol": request["protocol"], "request_id": request["request_id"], "status": "ok", "payload": None}
    if request["operation"] == "reset":
        assert set(request["payload"]) == {"context"}
        assert set(request["payload"]["context"]) == {"task", "chaser_mass_kg", "inertia_com_body_kg_m2", "wheel_spin_inertia_kg_m2", "hardware_torque_limits_nm", "hardware_speed_limits_rad_s"}
    elif request["operation"] == "act":
        assert set(request["payload"]) == {"observation"}
        obs = request["payload"]["observation"]
        assert set(obs) == {"schema_version", "episode_id", "sequence", "sample_time_ns", "delivery_time_ns", "orientation_body_to_j2000_wxyz", "angular_velocity_body_rad_s", "wheel_speed_rad_s", "gyro_valid"}
        if mode == "timeout":
            child = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
            Path("owned-child.json").write_text(json.dumps({"pid": child.pid, "created": psutil.Process(child.pid).create_time()}))
            time.sleep(60)
        if mode == "crash":
            raise SystemExit(7)
        if mode == "oversized":
            print("X" * (4*1024*1024 + 2), flush=True)
            continue
        if mode == "malformed":
            print("not json", flush=True)
            continue
        if mode == "stale":
            reply["request_id"] -= 1
        if mode == "error":
            reply.update(status="error", error="policy has no action")
        else:
            reply["payload"] = {"episode_id": obs["episode_id"], "observation_sequence": obs["sequence"], "motor_torque_xyz_nm": [0., 0., 0.]}
        if mode == "stderr":
            sys.stderr.write("L" * (300*1024))
            sys.stderr.flush()
    print(json.dumps(reply), flush=True)
    if request["operation"] == "close":
        break
'''


def definition_for(tmp_path, mode, task):
    (tmp_path / "worker.py").write_text(WORKER, encoding="utf-8")
    path = tmp_path / "method.json"
    path.write_text(json.dumps({"schema_version": "spacesim-autonomy-python-method/1", "method_id": "test-method-v1",
        "task_schemas": [task.schema_version], "entrypoint": "worker.py", "files": ["worker.py"],
        "arguments": [mode], "response_timeout_s": .3, "initialization_timeout_s": 10.}))
    return ExternalMethodDefinition(path, task)


@pytest.mark.parametrize("mode,reason", [("stale", "identity"), ("crash", "closed"), ("oversized", "size limit"),
                                         ("malformed", "invalid method response"), ("error", "reported error")])
def test_wire_failures_are_rejected_and_process_is_closed(tmp_path, mode, reason):
    context, observation = context_and_observation("attitude")
    method = ExternalMethod(definition_for(tmp_path, mode, context.task), tmp_path)
    try:
        method.reset(context)
        identity = {"pid": method.process.pid, "created": method.owner.created}
        with pytest.raises((ValueError, RuntimeError), match=reason):
            method.act(observation)
    finally:
        method.close()
    assert_process_exited(identity)


def test_timeout_closes_its_descendants_and_keeps_call_failure(tmp_path):
    context, observation = context_and_observation("attitude")
    method = ExternalMethod(definition_for(tmp_path, "timeout", context.task), tmp_path)
    try:
        method.reset(context)
        started = time.monotonic()
        with pytest.raises(TimeoutError, match="wall time"):
            method.act(observation)
    finally:
        method.close()
    assert_process_exited(json.loads((tmp_path / "owned-child.json").read_text()))
    assert time.monotonic() - started < 10
    calls = [json.loads(line) for line in (tmp_path / "method_calls.jsonl").read_text().splitlines()]
    assert calls[-1]["operation"] == "act" and calls[-1]["status"] == "failed"


def test_stderr_is_drained_and_storage_is_bounded(tmp_path):
    context, observation = context_and_observation("attitude")
    method = ExternalMethod(definition_for(tmp_path, "stderr", context.task), tmp_path)
    try:
        method.reset(context)
        parse_action(context.task, method.act(observation), observation)
    finally:
        method.close()
    assert (tmp_path / "method_stderr.log").stat().st_size == MAX_STDERR_BYTES
    assert method.telemetry()["stderr_bytes"] == 300 * 1024


def test_method_source_changes_and_path_escape_rejected(tmp_path):
    context, _ = context_and_observation("attitude")
    definition = definition_for(tmp_path, "stderr", context.task)
    with pytest.raises(ValueError, match="within"):
        definition.file("../outside.py")
    (tmp_path / "worker.py").write_text("raise SystemExit(0)")
    with pytest.raises(ValueError, match="changed"):
        definition.verify_unchanged()
    with pytest.raises(ValueError, match="support"):
        ExternalMethodDefinition(definition.path, context_and_observation("waypoint")[0].task)


def test_same_method_name_cannot_replace_frozen_implementation(tmp_path):
    from space_arm_platform.benchmark_batch import _completed_trial
    context, _ = context_and_observation("attitude")
    run = tmp_path / "run"
    run.mkdir()
    (tmp_path / "status.json").write_text('{"outcome":"method_error"}')
    (run / "result.json").write_text(json.dumps({"seed": 201, "method": "custom-v1"}))
    (run / "manifest.json").write_text(json.dumps({"task_fingerprint": context.task.fingerprint(),
        "external_method": {"files_sha256": {"worker.py": "different"}}}))
    with pytest.raises(ValueError, match="implementation"):
        _completed_trial(tmp_path, {"seed": 201}, context.task, "custom-v1", {"files_sha256": {"worker.py": "frozen"}}, source_sha256={})


@pytest.mark.parametrize("raw", [b'{"value":NaN}\n', b'{"value":Infinity}\n', b'{"value":1,"value":2}\n', b'{}', b'[]\n'])
def test_noncanonical_wire_messages_fail(raw):
    with pytest.raises(ValueError):
        decode_message(raw)
