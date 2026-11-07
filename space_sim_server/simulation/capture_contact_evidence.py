"""Read-only contact-force replay, independent of controller phase and feedback.

This is a physical contact sub-audit, not full mission/vision acceptance. Old
aggregate-only files cannot establish a result under this new evidence format.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

PERIOD_NS = 2_500_000
HOLD_NS = 20_000_000_000
JAWS = tuple(f'jaw_{index}' for index in range(4))


def finite(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError('physical numbers must be finite numeric values')
    return value


def vector(value, length):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError('physical vector has the wrong shape')
    return tuple(finite(item) for item in value)


def count(value):
    if type(value) is not int or value < 0:
        raise ValueError('physical counters must be nonnegative integers')
    return value


def unit(value):
    value = vector(value, 3)
    if abs(sum(component*component for component in value)-1.) > 1e-8:
        raise ValueError('physical direction must be unit length')
    return value


def geometry_from_model(model):
    geometry = {}
    bodies = set()
    def visit(body, root):
        name = body.get('name')
        if not name or name in bodies:
            raise ValueError('physical evidence requires unique named model bodies')
        bodies.add(name)
        for geom in body.findall('geom'):
            key = geom.get('name')
            if not key or key in geometry:
                raise ValueError('physical evidence requires unique named model geometry')
            geometry[key] = dict(body=name, root=root)
        for child in body.findall('body'):
            visit(child, root)
    document = ET.parse(model)
    if document.findall('.//include') or document.findall('./worldbody/geom'):
        raise ValueError('capture evidence requires an expanded model with all geometry in named bodies')
    for body in document.findall('./worldbody/body'):
        visit(body, body.get('name'))
    return geometry


def physical_contact_metrics(row, geometry):
    axis = unit(row['gripper_axis_world'])
    w,x,y,z = vector(row['gripper_quaternion_wxyz'], 4)
    if abs(w*w+x*x+y*y+z*z-1.) > 1e-8:
        raise ValueError('physical gripper quaternion must be unit length')
    derived = (2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y))
    if any(abs(a-b) > 1e-8 for a,b in zip(axis,derived,strict=True)):
        raise ValueError('gripper axis differs from the native body quaternion')
    contacts = row['raw_contacts']
    if not isinstance(contacts, list) or count(row['native_contact_count']) != len(contacts):
        raise ValueError('native contact rows are missing')
    jaws = {name: 0. for name in JAWS}
    peak, self_contacts = 0., 0
    constraints = set()
    for item in contacts:
        names = item['geoms']
        if not isinstance(names, list) or len(names) != 2 or any(not isinstance(name,str) or name not in geometry for name in names):
            raise ValueError('unmapped native contact geometry')
        bodies = [geometry[name]['body'] for name in names]
        roots = [geometry[name]['root'] for name in names]
        if item['bodies'] != bodies or item['roots'] != roots:
            raise ValueError('contact role differs from the model geometry')
        distance = finite(item['distance_m'])
        vector(item['position_local_m'], 3)
        force = vector(item['force_contact'], 6)
        frame = vector(item['frame'], 9)
        normal = unit(frame[:3])
        tangent1, tangent2 = unit(frame[3:6]), unit(frame[6:])
        cross = (normal[1]*tangent1[2]-normal[2]*tangent1[1],
            normal[2]*tangent1[0]-normal[0]*tangent1[2],normal[0]*tangent1[1]-normal[1]*tangent1[0])
        if (abs(sum(a*b for a,b in zip(normal,tangent1,strict=True))) > 1e-8
                or any(abs(a-b) > 1e-8 for a,b in zip(cross,tangent2,strict=True))):
            raise ValueError('native contact frame must be a right-handed orthonormal basis')
        if count(item['native_time_ns']) != row['native_time_ns']:
            raise ValueError('native contact and body readback clocks disagree')
        constraint = item['efc_address']
        if type(constraint) is not int or constraint < -1:
            raise ValueError('invalid native constraint address')
        if constraint >= 0:
            if constraint in constraints:
                raise ValueError('duplicate native contact constraint')
            constraints.add(constraint)
            if force[0] < -1e-12:
                raise ValueError('active normal contact force cannot be negative')
        arm = [body.startswith(('bottom_', 'jaw_', 'gripper', 'axial_', 'cam')) for body in bodies]
        if any(arm) and not all(arm) and roots == ['cubesat_bus', 'cubesat_bus'] and distance < -.0001:
            self_contacts += 1
        if set(roots) != {'cubesat_bus', 'capture_target'} or constraint < 0:
            continue
        peak = max(peak, abs(force[0]))
        if not any(name.startswith('capture_x_piece_') for name in names):
            continue
        axial = max(0., force[0])*abs(sum(a*b for a,b in zip(normal, axis, strict=True)))
        for body in bodies:
            if body in jaws:
                jaws[body] += axial
    return dict(jaw_force_by_body_n=jaws, jaw_force_n=sum(jaws.values()),
        jaw_support_count=sum(value > .1 for value in jaws.values()),
        peak_target_contact_force_n=peak, self_contact_count=self_contacts)


class CaptureContactScorer:
    def __init__(self, geometry):
        self.geometry = geometry
        self.count = 0
        self.previous_time = self.native_offset = self.start = None
        self.previous_physics = self.previous_step = self.physics_offset = None
        self.longest = self.peak = 0.
        self.final = None
        self.errors = set()
        self.safety = set()

    def add(self, row):
        try:
            if row['schema_version'] != 'space-sim-capture-contact-evidence/1':
                raise ValueError('unsupported physical contact evidence format')
            sequence, now, native = (count(row[key]) for key in ('sequence','time_ns','native_time_ns'))
            physics, steps = (count(row[key]) for key in ('physics_time_ns','physics_step_count'))
            if type(row['physics_substeps']) is not int or row['physics_substeps'] != 1:
                raise ValueError('contact evidence must sample every 400 Hz native step')
            if sequence != self.count or now % PERIOD_NS or physics % PERIOD_NS:
                raise ValueError('contact sequence or physical clock grid changed')
            if self.previous_time is not None and (now != self.previous_time+PERIOD_NS
                    or physics != self.previous_physics+PERIOD_NS or steps != self.previous_step+1):
                raise ValueError('missing duplicate or reordered physical contact sample')
            if abs(finite(row['time_s'])*1e9-now) > 1.:
                raise ValueError('contact clocks disagree')
            offset = now-physics
            physics_offset = physics-steps*PERIOD_NS
            if self.native_offset is None:
                self.native_offset = offset
                self.physics_offset = physics_offset
            if offset != self.native_offset or physics_offset != self.physics_offset:
                raise ValueError('physical epoch or completed-step counter changed')
            # MuJoCo accumulates binary floating-point dt. Preserve its raw
            # rounded-nanosecond readback; do not snap it onto the exact grid.
            # A conservative floating-addition error bound never permits even
            # a quarter missing step. Hold time uses only the integer clock.
            roundoff = steps*2.**-52
            if roundoff >= .01:
                raise ValueError('native clock precision is outside the evidence horizon')
            bound_ns = 1+math.ceil(steps*PERIOD_NS*roundoff/(1-roundoff))
            if bound_ns >= PERIOD_NS/4 or abs(native-steps*PERIOD_NS) > bound_ns:
                raise ValueError('stale or inconsistent native contact clock')
            if count(row['external_state_writes']):
                raise ValueError('physical state was overwritten after initialization')
            metrics = physical_contact_metrics(row, self.geometry)
        except (ValueError, TypeError, KeyError, OverflowError) as error:
            self.errors.add(str(error))
            self.start = None
            raise ValueError('invalid raw contact evidence: '+str(error)) from error
        self.peak = max(self.peak, metrics['peak_target_contact_force_n'])
        if metrics['self_contact_count']:
            self.safety.add('service_self_contact')
        if metrics['peak_target_contact_force_n'] > 10.:
            self.safety.add('peak_target_contact_force')
        supported = metrics['jaw_support_count'] == 4 and 4.5 <= metrics['jaw_force_n'] <= 5.5
        if supported and not self.errors and not self.safety:
            if self.start is None:
                self.start = physics
            self.longest = max(self.longest, physics-self.start)
        else:
            self.start = None
        self.count += 1
        self.previous_time, self.final = now, metrics
        self.previous_physics, self.previous_step = physics, steps

    def result(self):
        return dict(schema_version='space-sim-capture-contact-score/1',
            scope='four_jaw_physical_contact_sub_audit_not_full_mission_acceptance',
            physical_hold_passed=bool(self.count and self.longest >= HOLD_NS and not self.errors and not self.safety),
            sample_count=self.count, longest_hold_s=self.longest/1e9,
            final_time_s=None if self.previous_time is None else self.previous_time/1e9,
            final_physics_time_ns=self.previous_physics, final_native_step_count=self.previous_step,
            absolute_to_physics_offset_ns=self.native_offset, physics_clock_origin_ns=self.physics_offset,
            peak_target_contact_force_n=self.peak, final_contact_metrics=self.final,
            evidence_errors=sorted(self.errors), safety_violations=sorted(self.safety))


def score_capture_contacts(force_file, model, manifest_file):
    force_file, model, manifest_file = (Path(path) for path in (force_file, model, manifest_file))
    manifest = json.loads(manifest_file.read_text('utf-8'))
    geometry = geometry_from_model(model)
    if (not isinstance(manifest,dict) or manifest.get('schema_version') != 'space-sim-capture-contact-manifest/1'
            or manifest.get('model_sha256') != hashlib.sha256(model.read_bytes()).hexdigest()
            or manifest.get('geometry') != geometry or type(manifest.get('physics_period_ns')) is not int
            or manifest['physics_period_ns'] != PERIOD_NS
            or type(manifest.get('physics_substeps')) is not int or manifest['physics_substeps'] != 1
            or manifest.get('contact_force_frame') != 'MuJoCo contact frame; normal first'
            or manifest.get('force_file') != force_file.name):
        raise ValueError('contact manifest does not bind the actual model and force evidence')
    scorer = CaptureContactScorer(geometry)
    with force_file.open(encoding='utf-8') as stream:
        for line in stream:
            if not line.endswith('\n'):
                raise ValueError('truncated physical contact evidence')
            scorer.add(json.loads(line))
    return scorer.result()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('force_file', type=Path)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--manifest', type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        result = score_capture_contacts(args.force_file, args.model, args.manifest)
    except (ValueError, OSError, ET.ParseError) as error:
        parser.exit(2, str(error)+'\n')
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if result['physical_hold_passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
