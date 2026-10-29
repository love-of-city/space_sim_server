# Gyro-assisted terminal capture (experimental)

This opt-in mode uses the existing shared 50 Hz `service_gyro` to compensate own
spacecraft motion while an eye-in-hand camera guides the capture arm. After a
complete four-marker observation is lost, it predicts the previously observed
interface using encoder/FK and own-motion feedback. Contact closes the jaws and
uses the existing soft-pad force controller. No inter-spacecraft weld is added.

## Enable in a saved dual-capture scene

The existing `sarm-dual-capture26` scene's `randomization` can include:

```json
{
  "capture_window_trial": true,
  "capture_visual_test": {
    "enabled": true,
    "mode": "predictive_grasp",
    "held_target_pose": true,
    "output_directory": "run/capture_gyro_trial"
  }
}
```

Keep the target initially clear of the open/extended jaws and reachable by the
arm. This mode assumes a cooperative, attitude-stable target, no significant
unmodelled relative motion during the terminal prediction interval, a calibrated
camera-to-gripper transform, and a correct service mass model. Do not enable it
for arbitrary tumbling targets. `held_target_pose` is for `predictive_grasp`,
not the legacy `servo` probe; incompatible enabled configurations are rejected
before startup. The legacy probe retains its original contact safety guard.
The predictive mode is disabled by default.

The first 1.5 s estimates common orbit spin together with unknown constant gyro
bias. Keep the arm still during that interval. This is not an independent
absolute bias calibration. The encoder mass model approximates precontact
translation by conserving the service assembly COM; it is not a full navigator.
Original ADCS and dynamics remain unchanged.

## Timing and safety

- The sensor alone samples physical body rates. Capture reads its cached packet;
  it does not create another gyro or use target pose truth for guidance.
- Source timestamps remain on the 20 ms grid. Repeated 400 Hz reads do not redraw
  noise or reweight calibration. Rate hold predicts from the latest earlier
  measurement to the current command tick.
- Missing, invalid, saturated, future, reversed, discontinuous or >40 ms old
  measurements stop approach. A held target stays in bounded existing force
  control, retaining finite-value, compression and contact-overload guards.
- Scene changes require fresh visual/gyro history. Prediction remains limited
  by the existing 90 s age, 180 mm blind-path and 750 mm total-path guards.
- `truth.jsonl` is independent scoring only. `visual_control.jsonl` records the
  source frame, prediction and gyro validity/error. `consumed_service_gyro.jsonl`
  archives every uniquely accepted packet with original timestamp and epoch.
  Ordinary lower-rate observations are not a full-rate gyro archive.
- On a rejected read, diagnostic `sample_age_ns` describes the last accepted
  control read. The outer control time and source timestamp allow current age
  to be recomputed; validity/error explicitly marks the fault.

## Verification and known limits

Portable tests cover timestamp/epoch rejection, unique-sample integration,
held-target prediction, delayed images and force-hold fault handling. Run the
native scheduler tests separately; standalone Python MuJoCo and Basilisk must
not be imported into the same process:

```powershell
$env:PYTHONPATH = 'backend;scripts;.;..\space_sim_UE_Adapter\Adapters'
python -m pytest tests/test_service_gyro_capture.py tests/test_held_pose_capture.py tests/test_visual_predictive_capture.py -q
# In the native Basilisk environment:
python -m pytest tests/test_service_gyro_capture_native.py tests/test_capture26_gyro_native.py tests/test_gyroscope_native.py -q
```

Historical UE-image experiments in the original local experimental workspace
completed five retained initial-state trials with 22.7–23.2 s continuous
four-jaw support near 5 N, no service self-collision, unchanged mass/inertia,
and no fixed inter-spacecraft connection. These are deterministic regression
cases, not a reliability rate, mechanical residual-tolerance envelope, or a
full latest-main end-to-end requalification.

A separate single sensor-fault experiment added service-body-Y bias from zero
at 30 s to 0.01 deg/s at 150 s, then held it. Packets stayed fresh and valid;
the current algorithm did **not** diagnose the drift. Precontact position
prediction error grew from approximately 0.41 mm to 3.83 mm and a jaw made weak
early contact. That case still held with four supports for 22.92 s near 5 N;
target displacement versus independent free flight increased from about
4.11 mm to 5.04 mm. It demonstrates degraded precision, not drift detection.
No bias injector is enabled in normal startup, and no claim is made about
calibrated hardware performance or general fault tolerance.
