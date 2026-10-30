"""One reusable simulation graph owned by a sequential collection worker.

Reuse is conditional on content and configuration identity. Tasks, random
streams and communication sessions remain episode-specific. A failed setup
discards the graph; incompatible inputs take the ordinary construction path.
"""
from copy import deepcopy
import hashlib
import json
import time
import uuid

import numpy as np


class PhysicsGraphCache:
    def __init__(self):
        self.state = None
        self.pending_key = None
        self.miss_reason = "first_episode"

    def compatible(self, values):
        from simulation.compiled_model_cache import model_cache_key
        from simulation.mujoco_runtime import basilisk_mujoco_runtime
        from simulation.local_mujoco_stepper import local_substeps, publish_stride

        args, scene = values['args'], values['scene_instance']
        model = values['model']
        # These components may introduce episode-specific graph wiring.
        if (values['disturbances'].options.external_disturbance.enabled
                or values['orbital_physics'].drag is not None):
            self.close()
            self.pending_key = None
            self.miss_reason = "episode_specific_wiring"
            return False
        fingerprint = model_cache_key(model, basilisk_mujoco_runtime())
        if fingerprint is None:
            self.close()
            self.pending_key = None
            self.miss_reason = "uncacheable_model"
            return False
        attitude = model.with_name('attitude_control.json')
        catalog = args.catalog
        self.pending_key = json.dumps(dict(
            model=fingerprint, environment=scene.get('environment'),
            camera=scene.get('overview_camera'),
            camera_randomization=scene.get('collection_config', {}).get('overview_camera_randomization'),
            wrist_camera_mode=scene.get('wrist_camera_mode', 'rgb'),
            substeps=local_substeps(args.substeps), publish_stride=publish_stride(),
            control_port=args.capture_control_port, render_port=args.capture_render_port,
            attitude=hashlib.sha256(attitude.read_bytes()).hexdigest() if attitude.exists() else None,
            catalog=hashlib.sha256(catalog.read_bytes()).hexdigest() if catalog else None,
        ), sort_keys=True)
        if self.state is not None and self.state['key'] == self.pending_key:
            return True
        self.miss_reason = "inputs_changed" if self.state is not None else "first_episode"
        self.close()
        return False

    def remember(self, values):
        if self.pending_key is None:
            return
        names = ('sim', 'scene', 'models', 'recorders', 'ctx', 'orbit',
                 'plug_retention', 'publisher', 'capture')
        state = {key: values[key] for key in names}
        state.update(key=self.pending_key, identity=uuid.uuid4().hex,
                     orbital_physics=deepcopy(values['report']['orbital_physics']),
                     xml_qpos=np.asarray(state['scene'].assembleFullQpos()).reshape(-1).copy(),
                     xml_qvel=np.asarray(state['scene'].assembleFullQvel()).reshape(-1).copy())
        self.state = state
        values['report'].update(physics_graph_reused=False, physics_graph_id=state['identity'],
                               physics_graph_miss_reason=self.miss_reason)

    def restore(self, values):
        from simulation.scripted_tasks.native_state import STATE_READERS
        from simulation.control_client import SimulationControlClient
        from bsk_render_adapter.protocol import RenderPublisher

        state = self.state
        sim, scene = state['sim'], state['scene']
        stepper = sim.local_stepper
        state['publisher'].reference = lambda t: (values['command'], values['velocity'])
        reader = STATE_READERS[values['args'].task](values['task'], scene, stepper)
        started = time.perf_counter()
        capture = state['capture']
        if capture is not None:
            capture.task, capture.profile = values['task'], values['profile']
            capture.command, capture.scene_instance = values['command'], values['scene_instance']
            capture.state_reader, capture.disturbances = reader, values['disturbances']
            capture.sequence, capture.task_start_ns, capture.task_elapsed_s = 0, None, 0.
            capture._addresses = None
            capture.client = SimulationControlClient('127.0.0.1', values['args'].capture_control_port,
                                                     'sarm-scripted-task')
            capture.client.start()
            capture.client.wait_until_ready(timeout=30.)
            bridge = capture.render.bridge
            bridge.publisher = RenderPublisher('127.0.0.1', values['args'].capture_render_port,
                                               reliable_frames=False)
            bridge.capture_state_provider = capture.client.capture_state
            bridge.session_id = str(uuid.uuid4())
            bridge.last_capture_state = {}
            capture.observation.model.pending.clear()
            capture.observation.model.last_key = None
        phases = values['initialization_seconds']
        phases['capture_transport'] = time.perf_counter() - started
        phases['assemble_components'] = 0.
        started = time.perf_counter()
        stepper._check(stepper._lib.lms_reset_episode(stepper._handle), 'episode reset')
        attitude = sim.attitude_control
        attitude.reference_mrp = attitude.initial_inertia = None
        attitude.control_time_ns = 0
        stepper.ctrl.fill(0.)
        stepper.qacc.fill(0.)
        stepper.actuator_force.fill(0.)
        for name in ('step_count', 'kinematics_publish_count', 'external_writes', 'rebase_count',
                     'orbital_force_evaluations', 'max_contacts', 'max_constraint_rows', 'max_solver_iterations'):
            setattr(stepper, name, 0)
        # Reset scheduling and module state without SelfInit (which recompiles
        # the publication MJScene). Restore every passive/free DOF explicitly.
        sim.TotalSim.ResetSimulation()
        sim.TotalSim.resetInitSimulation()
        manager = scene.dynManager
        manager.getStateObject('mujocoQpos').setState(state['xml_qpos'].reshape(-1, 1))
        manager.getStateObject('mujocoQvel').setState(state['xml_qvel'].reshape(-1, 1))
        scene.postIntegration(0)
        scene.UpdateState(0)
        phases.update(build_physics=0., initialize_simulation=0.,
                      restore_existing_graph=time.perf_counter() - started)
        report = values['report']
        report.update(physics_graph_reused=True, physics_graph_id=state['identity'],
                      publication_scene=getattr(scene, 'publication_preparation', None),
                      orbital_physics=deepcopy(state['orbital_physics']),
                      model_compilation={**stepper.model_compilation, 'graph_reused': True,
                                         'fingerprint_seconds': 0., 'load_seconds': 0.},
                      plug_retention=state['plug_retention'].configuration()
                      if state['plug_retention'] else {'enabled': False})
        if capture is not None:
            report.update(render_model_path=str(values['model']),
                          render_model_sha256=report['model_sha256'])
        return tuple(state[k] for k in ('sim', 'scene', 'models', 'recorders', 'ctx', 'orbit',
                                        'plug_retention', 'capture')) + (stepper, reader)

    def holds(self, sim):
        return self.state is not None and self.state['sim'] is sim

    def close(self):
        state, self.state = self.state, None
        if state is not None:
            try:
                if state['capture'] is not None:
                    state['capture'].close()
            finally:
                try:
                    state['sim'].local_stepper.close()
                finally:
                    if state['orbit'].gravity_factory is not None:
                        state['orbit'].gravity_factory.unloadSpiceKernels()
