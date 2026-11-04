"""Original actuator position/velocity references and their execution dependencies."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import numpy as np

TASKS = ('drawer_cycle', 'aviation_plug_insert', 'guide_plug_insert', 'cylindrical_plug_insert')


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class FrozenTrajectory:
    def __init__(self, document):
        if not isinstance(document,dict):raise ValueError('Trajectory must be a JSON object')
        self.document = deepcopy(document)
        d = self.document
        deps=d.get('dependencies',{})
        if not isinstance(deps,dict) or any(not isinstance(k,str) or not isinstance(v,str) for k,v in deps.items()):
            raise ValueError('Dependencies must be a path to hash mapping')
        if d.get('schema') != 'space-arm-fixed-trajectory/2' or d.get('task') not in TASKS or d.get('frame') != 'cubesat_bus':
            raise ValueError('Unsupported fixed trajectory schema/task/frame')
        if not isinstance(d.get('model_sha256'), str) or len(d['model_sha256']) != 64:
            raise ValueError('A model SHA256 is required')
        try:
            self.times = np.array([r['time_s'] for r in d['samples']], dtype=float)
            self.commands = np.array([r['joint_reference'] for r in d['samples']], dtype=float)
            poses = np.array([r['position_m'] for r in d['samples']], dtype=float)
            rotations = np.array([r['rotation'] for r in d['samples']], dtype=float)
            initial = np.array(d['initial_joints'], dtype=float)
            self.velocities=np.array([r['joint_reference_velocity'] for r in d['samples']],dtype=float)
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError('Malformed reference table') from exc
        n = len(self.times)
        if n < 2 or self.times[0] != 0 or np.any(np.diff(self.times) <= 0) or self.commands.shape != (n,8) or poses.shape != (n,3) or rotations.shape != (n,3,3) or initial.shape != (8,):
            raise ValueError('Invalid table shapes or timestamps')
        if self.velocities.shape!=(n,8) or not all(np.isfinite(a).all() for a in [self.times, self.commands, self.velocities, poses, rotations, initial]):
            raise ValueError('Non-finite trajectory')
        if not np.allclose(rotations @ rotations.transpose(0,2,1), np.eye(3), atol=1e-6) or not np.allclose(np.linalg.det(rotations), 1, atol=1e-6):
            raise ValueError('Invalid rotation matrices')
        if not np.allclose(np.diff(self.times), .005, rtol=0, atol=1e-9):
            raise ValueError('Replay requires consecutive original 200 Hz references')
        self.duration = float(self.times[-1])

    def validate_physics(self, resolved):
        if not self.document.get('physics_identity') or resolved!=self.document['physics_identity']:
            raise ValueError('Frozen resolved physics mismatch or missing identity')

    def reference(self, time_s):
        if not np.isfinite(time_s) or time_s < 0:
            raise ValueError('Invalid playback time')
        i = min(len(self.times)-1, np.searchsorted(self.times, time_s + 1e-10, side='right')-1)
        # Zero-order hold exactly reproduces the original 200 Hz actuator table.
        velocity = self.velocities[i].copy() if time_s<=self.duration+1e-10 else np.zeros(8)
        return self.commands[i].copy(), velocity

    def validate_runtime(self, model, initial, server_root):
        if sha256(model) != self.document['model_sha256']:
            raise ValueError('Frozen trajectory model hash mismatch')
        if not np.allclose(initial, self.document['initial_joints'], rtol=0, atol=1e-12):
            raise ValueError('Fixed-condition playback requires the frozen initial joints')
        root = Path(server_root).resolve()
        for relative, expected in self.document.get('dependencies', {}).items():
            p = Path(relative) if Path(relative).is_absolute() else root / relative
            if sha256(p) != expected:
                raise ValueError(f'Frozen dependency mismatch: {relative}')
        provenance = self.document.get('donor_runtime')
        if provenance is not None:
            import os
            for name, expected in provenance['collision_filter_modes'].items():
                if os.environ.get(name, '0') != expected:
                    raise ValueError(f'Frozen collision mode mismatch: {name}')
            for relative, expected in provenance['sources_sha256'].items():
                if sha256(root / relative) != expected:
                    raise ValueError(f'Frozen source mismatch: {relative}')
            from collection.runtime_provenance import native_identity
            native = native_identity(root)
            for library in provenance['loaded_native']:
                if (library.get('manifest_verified') is not True
                        or native.get(library['name'], {}).get('library_sha256') != library['sha256']):
                    raise ValueError(f'Frozen native library mismatch: {library["name"]}')


def freeze_trace(trace, model, task, output, server_root):
    """Freeze the actual applied references once; donor success is not assumed."""
    from simulation.serial_chain_kinematics import SerialChainKinematics
    from simulation.scene_library.manifest import model_dependencies
    chain = SerialChainKinematics.from_mjcf(model, base_body='cubesat_bus', joint_names=tuple(f'joint{i}' for i in range(1,7)), tool_site='sarm_ee')
    samples=[];initial=None;first=None
    with Path(trace).open(encoding='utf-8') as stream:
        for line in stream:
            row=json.loads(line)
            stamp=int(row['sim_time_ns'])
            if first is None:
                first=stamp;initial=row['joint_position']
            q=np.array(row['joint_reference'], dtype=float)
            p,r=chain.forward(q[:6])
            samples.append(dict(time_s=(stamp-first)*1e-9,joint_reference=q.tolist(),joint_reference_velocity=row['joint_reference_velocity'],position_m=p.tolist(),rotation=r.tolist()))
    server_root=Path(server_root).resolve();model=Path(model).resolve()
    dependencies={}
    files={Path(record['path']) for record in model_dependencies(model)}
    files.add(server_root/'config/plug_retention.json')
    import os
    if os.environ.get('SPACE_SIM_PLUG_RETENTION_CONFIG'):
        files.add(Path(os.environ['SPACE_SIM_PLUG_RETENTION_CONFIG']).expanduser().resolve())
    for file in sorted(files):
        key = file.relative_to(server_root).as_posix() if file.is_relative_to(server_root) else str(file)
        dependencies[key] = sha256(file)
    d=dict(schema='space-arm-fixed-trajectory/2',task=task,frame='cubesat_bus',model_sha256=sha256(model),
           dependencies=dependencies,initial_joints=initial,reference_semantics='original applied 200 Hz actuator position and velocity references',
           donor_trace_sha256=sha256(trace),samples=samples)
    report_path=Path(trace).parent/'summary.json'
    if report_path.is_file():
        report=json.loads(report_path.read_text(encoding='utf-8'))
        if report['model_sha256']!=d['model_sha256'] or report['task_name']!=task:
            raise ValueError('Donor report does not match model/task')
        d.update(donor_result=report['result'],donor_reason=report.get('reason'),donor_config=report['config'],
                 physics_identity={key:report[key] for key in ['substeps','plug_retention','orbital_physics']})
    if not report_path.is_file() or not d.get('physics_identity'):
        raise ValueError('A matching complete donor report is required')
    if report.get('runtime_error') or report.get('simulated_seconds', -1) < samples[-1]['time_s'] - 1e-9:
        raise ValueError('Incomplete donor runtime')
    d.update(model_path=str(model), resolved_scene=report['resolved_scene'],
             donor_task_version=report.get('task_version'), donor_runtime=report['runtime_provenance'])
    frozen = FrozenTrajectory(d)
    frozen.validate_runtime(model, initial, server_root)
    output=Path(output)
    with output.open('x',encoding='utf-8') as stream:json.dump(d,stream,separators=(',',':'))
    return d
