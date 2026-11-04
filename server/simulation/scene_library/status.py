"""Coverage accounting never promotes a family from one successful subtask."""
import json
from pathlib import Path
from .catalog import catalog, phase_one_scope
from .manifest import digest, mount_inspection_passed, preheld_inspection_passed


def inspection_index(output_root):
    """Keep rejected geometry associated with its exact model revision."""
    index = {}
    paths = {p for p in Path(output_root).rglob('*.json') if 'inspection' in p.name}
    # A run-library output is normally separate from its generated asset folder.
    # Follow the explicitly recorded inspection rather than requiring a copy.
    for instance_path in Path(output_root).rglob('instance.json'):
        instance = json.loads(instance_path.read_text(encoding='utf-8-sig'))
        fixture = instance.get('operation_fixture') or {}
        if instance.get('schema') == 'space-arm-scene-instance/1' and fixture.get('mount_inspection'):
            candidate = Path(fixture['mount_inspection'])
            if candidate.is_file():
                paths.add(candidate)
    for path in sorted(paths):
        record = json.loads(path.read_text(encoding='utf-8-sig'))
        if not str(record.get('schema', '')).startswith('space-arm-mount-inspection/'):
            continue
        sha = record.get('model_sha256')
        if not sha:
            continue
        passed = mount_inspection_passed(record, sha,
            receiver_required=record.get('geometry_counts', {}).get('receiver', 0) > 0)
        penetration_count = sum(len(c.get('penetrations', [])) for c in record.get('checks', []))
        index.setdefault(sha, []).append(dict(report=str(path), schema=record['schema'],
            passed=passed, penetration_pairs=penetration_count,
            coverage_complete=record.get('coverage_complete') is True))
    return index


def geometry_status(reports):
    current = [r for r in reports if r['schema'] == 'space-arm-mount-inspection/2']
    # Conflicting current checks never silently select the successful record.
    if any(not r['passed'] for r in current):
        return 'rejected'
    if current:
        return 'passed'
    return 'not_checked' if not reports else 'incomplete_legacy_check'


def phase_one_evidence(path, record, inspections, item):
    """Reconcile recorded physical results with the user's narrower first phase.

    Historical receipts are immutable. This projection never certifies docking
    or uses an unbound, failed or later geometrically rejected operation.
    """
    try:
        native_path = Path(record['native_report'])
        native = json.loads(native_path.read_text(encoding='utf-8-sig'))
        if item['family_number'] in (2, 4):
            # A rigid service-deck fixture cannot satisfy a two-arm holding
            # contract, even when its operation completed successfully.
            if record.get('evidence_scope') != 'dual_arm_preheld_operation':
                return False
            instance = json.loads(path.with_name('instance.json').read_text(encoding='utf-8-sig'))
            summary = json.loads((native_path.parent / 'run/summary.json').read_text(encoding='utf-8-sig'))
            hold_path = Path(record['preheld_capture_validation'])
            hold = json.loads(hold_path.read_text(encoding='utf-8-sig'))
            model_sha = next(d['sha256'] for d in instance['dependencies'] if Path(d['path']) == Path(instance['model']))
            fixture = instance['operation_fixture']
            from simulation.components.preheld_capture import hold_gains
            expected_gains = hold_gains(instance['initial_scene'].get('preheld_capture', {}).get('hold_gains'))
            inspection_path = Path(fixture['mount_inspection'])
            inspection = json.loads(inspection_path.read_text(encoding='utf-8-sig'))
            hold_checks = ('pose_coverage_200hz', 'contact_coverage_400hz', 'grasp_position_error_under_5mm',
                'grasp_angle_error_under_0p02rad', 'capture_joint_error_under_0p05rad',
                'target_relative_drift_under_5mm', 'no_unintended_capture_external_contact',
                'no_target_service_hardware_collision', 'retention_reactions_bound_to_actual_owners',
                'capture_controller_installed')
            return (record.get('operation_accepted') is True and record.get('returncode') == 0
                    and record.get('evidence_scope') == 'dual_arm_preheld_operation'
                    and instance['recipe']['id'] == item['id']
                    and instance['operation_fixture']['scope'] == record['evidence_scope']
                    and summary.get('model_sha256') == model_sha
                    and summary.get('preheld_capture', {}).get('hold_gains') == expected_gains
                    and digest(inspection_path) == fixture['mount_inspection_sha256']
                    and preheld_inspection_passed(inspection, model_sha, fixture['initial_scene_sha256'])
                    and native.get('accepted') is True and native.get('coverage_complete') is True
                    and native.get('trajectory_clear') is True
                    and digest(hold_path) == record.get('preheld_capture_validation_sha256')
                    and hold.get('model_sha256') == model_sha and hold.get('passed') is True
                    and all(hold.get('checks', {}).get(k) is True for k in hold_checks)
                    and hold.get('scene_sha256') == digest(record.get('resolved_initial_scene', fixture['initial_scene_source']))
                    and hold.get('trace_sha256') == digest(native_path.parent / 'states-200hz.jsonl.gz')
                    and hold.get('contact_trace_sha256') == digest(native_path.parent / 'contacts-400hz.jsonl.gz')
                    and native.get('independent_evaluation', {}).get('success') is True)
        returncode = record.get('returncode')
        if returncode is None and record.get('source') == 'current_run_imported_from_native_validator':
            # Early imported runs bind the native return code by report digest.
            if record.get('report_sha256') == digest(native_path):
                returncode = native.get('native_returncode')
        if record.get('accepted') is not True or returncode != 0:
            return False
        if item['family_number'] == 1:
            return (native.get('accepted') is True and native.get('coverage_complete') is True
                    and native.get('trajectory_clear') is True
                    and native.get('independent_evaluation', {}).get('success') is True)
        if item['family_number'] == 3:
            independent = json.loads(Path(record.get('independent_validation',
                native_path.with_name('independent_validation.json'))).read_text(encoding='utf-8-sig'))
            return bool(native.get('checks')) and all(v is True for v in native['checks'].values()) and independent.get('passed') is True
    except (OSError, ValueError, KeyError, TypeError, StopIteration):
        return False
    return False


def summarize(output_root):
    inspections = inspection_index(output_root)
    scenes = {item["id"]: {**item, "runs": []} for item in catalog()["scenes"]}
    scope = phase_one_scope()
    for item in scenes.values():
        item['phase_one_required'] = item['phase'] == 1 and item['id'] not in scope['deferred_scene_ids']
        item['phase_one_accepted'] = False
    incomplete = []
    for path in sorted(Path(output_root).rglob('instance.json')):
        if path.with_name('receipt.json').exists():
            continue
        instance = json.loads(path.read_text(encoding='utf-8-sig'))
        if instance.get('schema') != 'space-arm-scene-instance/1':
            continue
        progress = None
        log = path.with_name('stdout.log')
        if log.is_file():
            with log.open(encoding='utf-8-sig') as stream:
                for line in stream:
                    try:
                        row = json.loads(line)
                    except json.JSONDecodeError:
                        continue  # A writer may still be appending the final line.
                    if isinstance(row, dict) and row.get('type') in ('task_progress', 'task_result', 'task_stage'):
                        progress = {key: row[key] for key in ('type', 'stage', 'sim_seconds', 'result', 'reason')
                                    if key in row}
        incomplete.append(dict(output=str(path.parent), scene_id=instance['recipe']['id'],
            status='incomplete_output', process_liveness='not_checked', last_progress=progress,
            accepted=False))
    for path in sorted(Path(output_root).rglob("receipt.json")):
        record = json.loads(path.read_text(encoding="utf-8-sig"))
        if record.get("schema") != "space-arm-scene-run/1":
            continue
        item = scenes.get(record.get("scene_id"))
        if item is None:
            continue
        item["runs"].append(dict(receipt=str(path), status=record["status"],
                                 accepted=record.get("accepted") is True,
                                 operation_accepted=record.get("operation_accepted") is True,
                                 evidence_scope=record.get("evidence_scope")))
        scoped_pass = item['phase_one_required'] and phase_one_evidence(path, record, inspections, item)
        item['runs'][-1]['phase_one_accepted'] = bool(scoped_pass)
        item['phase_one_accepted'] |= bool(scoped_pass)
        # Runtime records remain scoped to their declared adapter. Fixtures and
        # logical capture cannot discharge the missing physical docking contract.
        if not item["missing_capabilities"] and record.get("accepted") is True:
            item["accepted"] = True
    families = []
    for number in range(1, 9):
        entries = [s for s in scenes.values() if s["family_number"] == number]
        families.append(dict(number=number, label=entries[0]["label"], tasks=len(entries),
                             accepted_tasks=sum(s["accepted"] for s in entries),
                             accepted=all(s["accepted"] for s in entries)))
    native_operations = []
    for path in sorted(Path(output_root).rglob('acceptance.json')):
        record = json.loads(path.read_text(encoding='utf-8-sig'))
        if record.get('schema') != 'task-trajectory-contacts/3-local-physical':
            continue
        summary_path = path.parent / 'run/summary.json'
        summary = json.loads(summary_path.read_text(encoding='utf-8-sig')) if summary_path.is_file() else {}
        model_checks = inspections.get(summary.get('model_sha256'), [])
        mount_status = geometry_status(model_checks)
        native_operations.append(dict(report=str(path), task=summary.get('task_name'),
            model=summary.get('model'), model_sha256=summary.get('model_sha256'),
            task_reason=record.get('task_reason'), trajectory_clear=record.get('trajectory_clear'),
            operation_accepted=record.get('accepted') is True,
            independent_success=record.get('independent_evaluation', {}).get('success') is True,
            mount_geometry_status=mount_status, mount_inspections=model_checks,
            operation_with_mount_check_passed=(record.get('accepted') is True and mount_status == 'passed'),
            full_scene_accepted=False, scope='raw_native_operation_not_family_completion'))
    phase_families = []
    for number in range(1, 5):
        entries = [s for s in scenes.values() if s['family_number'] == number and s['phase_one_required']]
        phase_families.append(dict(number=number, required_tasks=len(entries),
            accepted_tasks=sum(s['phase_one_accepted'] for s in entries),
            accepted=bool(entries) and all(s['phase_one_accepted'] for s in entries)))
    return dict(schema="space-arm-library-status/1", families=families, scenes=list(scenes.values()),
                phase_one=dict(scope=scope, families=phase_families,
                    complete=all(f['accepted'] for f in phase_families)),
                native_operations=native_operations, incomplete_runs=incomplete,
                first_four_complete=all(f["accepted"] for f in families[:4]))
