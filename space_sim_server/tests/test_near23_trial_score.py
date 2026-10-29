"""Saved contact evidence must resolve excursions between camera frames."""
import json

import pytest

from tools.score_near23_gyro_trial import contact_metrics, score


def audit(path, *, interrupt=None, gap=False):
    with path.open('w', encoding='utf-8') as stream:
        for tick in range(4801):
            if gap and 2400 <= tick < 2440:
                continue
            stream.write(json.dumps(dict(type='step', t=tick/400, phase='hold_force',
                clamp=dict(pad_force_n=6. if tick == interrupt else 5.,
                           jaw_support_count=4, peak_target_contact_force_n=2.)))+'\n')


def test_physics_tick_force_excursion_breaks_otherwise_valid_image_hold(tmp_path):
    path = tmp_path/'contacts.jsonl'; audit(path)
    result = contact_metrics(path)
    assert result['supported_force_hold_2plus_jaws_s'] == pytest.approx(12.)
    audit(path, interrupt=2401)  # 6.0025 s: missed by 10 Hz images.
    result = contact_metrics(path)
    assert result['supported_force_hold_2plus_jaws_s'] < 10
    assert result['supported_force_hold_all4_jaws_s'] < 10


def test_missing_physics_ticks_break_continuous_hold(tmp_path):
    path = tmp_path/'contacts.jsonl'; audit(path, gap=True)
    assert contact_metrics(path)['supported_force_hold_2plus_jaws_s'] < 10


def test_completed_without_image_and_contact_evidence_cannot_pass(tmp_path):
    for name in ('truth.jsonl', 'measurements.jsonl'):
        (tmp_path/name).write_text('', encoding='utf-8')
    (tmp_path/'completed.json').write_text('{}', encoding='utf-8')
    result = score(tmp_path)
    assert result['completed']
    assert not result['grasp_pass']
    assert not result['all_four_jaws_supported_hold_pass']


def test_mixed_legacy_contact_log_is_reported_without_claiming_a_physical_pass(tmp_path):
    for name in ('truth.jsonl', 'measurements.jsonl'):
        (tmp_path/name).write_text('', encoding='utf-8')
    (tmp_path/'completed.json').write_text('{}', encoding='utf-8')
    (tmp_path/'contacts_400hz.jsonl').write_text('{"type": {"type":\n', encoding='utf-8')
    result = score(tmp_path)
    assert result['physics_tick_contact_audit_error']
    assert result['physics_tick_contact_audit'] is None
    assert not result['physical_grasp_hold_pass']
    assert not result['grasp_pass']
