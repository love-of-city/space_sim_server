"""Accuracy reports are strict about time alignment and finite physics data."""
import json

import pytest

from tools.benchmark_native_acceleration import compare_observations, assert_platform_idle


def observation(time_ns, q):
    return {"sim_time_ns":str(time_ns),"arm_joint_position_rad":q}


def write(path, rows):
    path.write_text("".join(json.dumps(r)+"\n" for r in rows),encoding="utf-8")
    return path


def test_reports_true_max_absolute_state_error(tmp_path):
    a=write(tmp_path/"a.jsonl",[observation(0,[0.,1.]),observation(10,[1.,2.])])
    b=write(tmp_path/"b.jsonl",[observation(0,[0.,1.]),observation(10,[1.001,2.])])
    report=compare_observations(a,b)
    assert report["samples"]==2
    assert report["max_absolute_errors"]["arm_joint_position_rad"]==pytest.approx(.001)


def test_different_simulation_times_are_not_compared_by_row_number(tmp_path):
    a=write(tmp_path/"a.jsonl",[observation(0,[0.])])
    b=write(tmp_path/"b.jsonl",[observation(1,[0.])])
    with pytest.raises(RuntimeError,match="clocks"):
        compare_observations(a,b)


def test_nonfinite_state_cannot_pass_accuracy_check(tmp_path):
    a=write(tmp_path/"a.jsonl",[observation(0,[0.])])
    b=write(tmp_path/"b.jsonl",[observation(0,[float("nan")])])
    with pytest.raises(RuntimeError,match="Invalid state"):
        compare_observations(a,b)


def test_benchmark_refuses_to_compete_with_active_scene(tmp_path):
    (tmp_path/"run").mkdir()
    path=tmp_path/"run/scene_runtime.json"
    for phase in ("launching","starting_renderer","starting_simulation","running"):
        path.write_text(json.dumps({"phase":phase}),encoding="utf-8")
        with pytest.raises(RuntimeError,match="live scene"):
            assert_platform_idle(tmp_path)
    path.write_text(json.dumps({"phase":"stopped"}),encoding="utf-8")
    assert_platform_idle(tmp_path)
