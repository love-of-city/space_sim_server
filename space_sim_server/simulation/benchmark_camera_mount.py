"""Read current native articulated-body publications without evolving state."""
from __future__ import annotations

from Basilisk.utilities import RigidBodyKinematics as rbk

from space_arm_platform.benchmark_articulated_camera import CameraMountSample, MountedPinholeCamera


class NativeCameraMountReader:
    def __init__(self, ports, cameras, *, episode_id, epoch_ns=0):
        self.cameras = tuple(MountedPinholeCamera.model_validate(camera) for camera in cameras)
        if not self.cameras or len({camera.camera_id for camera in self.cameras}) != len(self.cameras):
            raise ValueError('native mount reader requires distinct declared cameras')
        self.stepper = ports.stepper
        if self.stepper is None:
            raise ValueError('native camera mounts require the authoritative local stepper')
        self.episode_id, self.epoch_ns = episode_id, epoch_ns
        # Validate identity/clock before registering any readers.
        CameraMountSample(episode_id=episode_id, model_xml_sha256=self.stepper.model_xml_sha256,
            parent_body_name=self.cameras[0].parent_body_name, epoch_ns=epoch_ns, physics_time_ns=0,
            sample_time_ns=epoch_ns, native_step_count=0, external_state_writes=0,
            origin_j2000_m=(0., 0., 0.), orientation_j2000_from_body_wxyz=(1., 0., 0., 0.))
        self.readers = {name: ports.body_state(name).addSubscriber()
                        for name in dict.fromkeys(camera.parent_body_name for camera in self.cameras)}

    def sample(self, sample_time_ns):
        if not self.stepper._handle:
            raise ValueError('native camera mount runtime is closed')
        if type(sample_time_ns) is not int or sample_time_ns < self.epoch_ns:
            raise ValueError('camera exposure must use an integer physical epoch')
        physics_time = sample_time_ns-self.epoch_ns
        if self.stepper._last_nanos != physics_time:
            raise ValueError('camera exposure is not at the current native publication')
        samples = {}
        for name, reader in self.readers.items():
            if not reader.isWritten() or int(reader.timeWritten()) != physics_time:
                raise ValueError('camera parent body publication is missing or stale')
            body = reader()
            samples[name] = CameraMountSample(episode_id=self.episode_id,
                model_xml_sha256=self.stepper.model_xml_sha256, parent_body_name=name,
                epoch_ns=self.epoch_ns, physics_time_ns=physics_time, sample_time_ns=sample_time_ns,
                native_step_count=self.stepper.step_count, external_state_writes=self.stepper.external_writes,
                origin_j2000_m=tuple(body.r_BN_N),
                orientation_j2000_from_body_wxyz=tuple(rbk.MRP2EP(body.sigma_BN)))
        return tuple(samples[camera.parent_body_name] for camera in self.cameras)
