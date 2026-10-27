"""Run capture26 through the formal distant/near flight process interface."""
import argparse
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace

from space_arm_platform.rendezvous_config import FlightOptions
from space_arm_platform.near_field_config import NearFieldOptions
from simulation.capture26_mission import load_mission, MODEL_ID
from simulation.capture26_handoff import load
from simulation.orbital_rendezvous.reference import FlightProfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scenario', choices=('in-orbit', 'orbit-raise', 'near-field'), required=True)
    p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--handoff', type=Path)
    p.add_argument('--catalog', type=Path)
    p.add_argument('--state-port', type=int, default=0)
    p.add_argument('--render-port', type=int, default=5558)
    p.add_argument('--capture-port', type=int, default=8767)
    p.add_argument('--external-vision', action='store_true')
    p.add_argument('--mission-id')
    p.add_argument('--live', action='store_true')
    p.add_argument('--simulation-rate', type=float, default=1.,
                   help='wall-clock pacing multiplier; not a mission acceptance requirement')
    p.add_argument('--duration', type=float, help='diagnostic early stop; cannot pass an unfinished stage')
    p.add_argument('--fault', choices=('', 'image_loss', 'worker_exit', 'gnss_loss', 'communication_loss',
        'service_gyro_loss', 'target_gyro_loss', 'gyro_saturation'), default='')
    p.add_argument('--fault-after', type=float, default=2.)
    p.add_argument('--fault-duration', type=float)
    p.add_argument('--fault-after-release', action='store_true')
    p.add_argument('--post-release-duration', type=float, default=0., help='continue actual control for fault/recovery acceptance')
    cli = p.parse_args()
    if not math.isfinite(cli.simulation_rate) or not 0 < cli.simulation_rate <= 100:
        p.error('simulation-rate must be finite and in (0, 100]')
    if (not math.isfinite(cli.post_release_duration) or cli.post_release_duration < 0
            or not math.isfinite(cli.fault_after) or cli.fault_after < 0
            or cli.fault_duration is not None and (not math.isfinite(cli.fault_duration) or cli.fault_duration <= 0)):
        p.error('invalid fault or continuation duration')
    options = json.loads(cli.config.read_text('utf-8-sig'))
    incoming = None
    if cli.scenario == 'near-field':
        near = NearFieldOptions.from_dict(options)
        if cli.handoff is None:
            p.error('capture26 near flight requires an accepted continuous far-flight result')
        incoming = load(cli.handoff, ROOT)
        if incoming['phase'] != 'FarReady':
            p.error('near flight must resume a FarReady handoff')
        flight = FlightOptions.from_dict(incoming['mission_options'])
        if flight.model_id != near.model_id or flight.seed != near.seed:
            p.error('model or sensor seed changed across phases')
        if near.vision_loss_phase or near.communication_loss_s:
            p.error('capture26 uses timestamped sensor fault injection; legacy fault phases do not apply')
    else:
        flight = FlightOptions.from_dict(options)
        if cli.handoff:
            p.error('distant capture26 flight starts from an explicit initial prior')
    if flight.model_id != MODEL_ID:
        p.error('this entry point requires capture26 model_id')
    duration = (cli.duration if cli.duration is not None else
                6000. if incoming else FlightProfile(scenario=cli.scenario).duration_s+300.)
    if not 0 < duration <= 30000:
        p.error('invalid duration')
    args = SimpleNamespace(output=cli.output_dir, adapter=ROOT.parent/'space_sim_UE_Adapter',
        depth=110., duration=duration, seed=flight.seed,
        gnss_delay=(flight.differential_gnss or {}).get('delay_s', 0.),
        ue=cli.live, catalog=cli.catalog, render_port=cli.render_port, render_rate=10,
        capture_port=cli.capture_port, realtime=True, profile=False, save_all_images=False,
        simulation_rate=cli.simulation_rate,
        pointing_error_deg=[0., 0., 0.], approach=False, fault=cli.fault, fault_after=cli.fault_after, fault_after_ready=False,
        fault_after_release=cli.fault_after_release, fault_duration=cli.fault_duration,
        post_release_duration=cli.post_release_duration,
        mission_stage='near' if incoming else 'far', scenario=cli.scenario, flight_options=flight,
        mission_config=load_mission(ROOT), handoff=cli.handoff, mission_id=cli.mission_id,
        external_vision=cli.external_vision, state_port=cli.state_port)
    os.environ['SPACE_SIM_LOCAL_PUBLISH_STRIDE'] = '8'
    from simulation.capture26_runtime import run
    run(args)


if __name__ == '__main__':
    main()
