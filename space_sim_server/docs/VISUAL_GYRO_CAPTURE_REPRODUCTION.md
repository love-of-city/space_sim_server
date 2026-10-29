# Native visual + gyro terminal grasp reproduction

This opt-in regression runs the current checkout's real UE renderer, main
backend, original ArUco/PnP worker, shared 50Hz service gyro and unchanged capture
controller. The retained nominal scene uses seed26016, adjacent-face dual arms,
COM-coorbital free flight and an initial grip/interface distance of0.5m. It opens
and extends the L jaws, visually approaches, predicts from own encoders/gyro
after marker loss, closes and tightens with the existing soft-pad force control.
Ordinary GUI startup remains manual.

## Requirements and single entry

Clone the unified repository, run `git lfs pull`, and install the server/adapter
as described in README. Supply a native Basilisk/MJScene Python environment,
UE5.6 with its C++ build prerequisites, and a separate worker Python containing
OpenCV-contrib and MuJoCo3.7.0. The worker can use the declared `[vision,posture]`
extras. Never import standalone Python MuJoCo and Basilisk in the same process.

From the server directory (all three machine paths below must be replaced):

```powershell
pwsh -NoProfile -File scripts/run_visual_gyro_capture.ps1 `
  -Python 'C:\your-native-env\Scripts\python.exe' `
  -VisionPython 'C:\your-worker-env\Scripts\python.exe' `
  -UnrealRoot 'D:\UE_5.6' `
  -OutputDirectory 'D:\capture-evidence\nominal-001'
```

`OutputDirectory` must be new. The command builds native helper libraries and
checks/builds the UE module; it derives machine-local catalogs from this
checkout's models and tracked UE assets. It never copies another machine's
Saved catalogs or regenerates physical models. UE runs on the GPU offscreen
without Pixel Streaming; actual1920×1200 PNGs go to the original image-only
vision worker. This is a native image-driven test, not a screenshot or a mock
visual controller. Existing UI camera poses and lights are unchanged.

Default isolated ports are19100(API),19101(control),19102(capture),19103(rendezvous)
and19104(UE); use `-PortBase` for another free range. Occupied ports and another
backend/scene in the same checkout are refused. Use a clean checkout if another
scene is running. The entry temporarily leases `run/vision_runtime.json`, clears
the old observation and enables its export. It closes only its owned processes
and restores previous config/observation bytes on success or ordinary failure.
An interrupted/killed launcher may leave `run/capture_regression.lock`; inspect
the recorded PID before removing a stale lock. Failed writer shutdown retains
both the lease and original config/observation bytes in the reported
`run/capture_regression_restore_*` directory. Stop the recorded owned writers
before restoring those bytes according to `restore.json` and removing the lock.
No global platform stop is used.

## Configuration and evidence

Portable inputs are `config/experiments/visual_gyro_capture/{recipe,scene,vision_runtime}.json`.
The launcher resolves `$OUTPUT` and `$VISION_PYTHON` into separate saved runtime
files. Changing the physical model or enabling formation hold/fixed connections
is outside this regression. The target assembly remains10kg (8kg root plus two
articulated1kg panels), with every inertial tensor and model XML fingerprinted
before and after. The controller consumes image estimates and own-motion
feedback; `truth.jsonl` and the reference trajectory are scorer inputs only.

Artifacts include resolved scene/runtime, source manifests, native build logs,
UE/simulation/backend logs, raw PNGs and metadata, perception stream, control
commands, uniquely consumed gyro packets,400Hz contacts, physical fingerprints,
free-reference trajectory and `result.json`. Failure/timeout returns nonzero and
keeps the failed directory. A new attempt requires a new output directory.

After actual capture, a separate no-contact run uses the same initial target,
orbit, mass model and real backend, with the arms held and grasp/vision disabled.
Only render transport is discarded to speed up that scorer. It records free
target motion; its data is never read by the actual grasp process.

Success requires a fresh four-marker native PNG independently re-decoded through
PnP, matching UE/PnP K, the complete visual→gyro prediction→force-hold phase
sequence, contiguous valid gyro samples, no service self-contact, unchanged
physical fingerprints and at least20s continuous four-jaw support at4.5..5.5N.
The original controller can close through `close` or `contact_seek`; both are
accepted only with recorded inward jaw commands and subsequent physical support.
UE, backend and simulation must remain alive through the scored hold endpoint;
their return codes are saved before intentional teardown. Contact-record gaps
reset the continuous-hold window. Gyro records must match the run session/epoch,
cover the full interval and match packets actually consumed by the controller.
Scoring ends at the verified live endpoint, excluding teardown tail samples.
The independent free
reference must agree before contact within0.05mm/0.005deg; nominal disturbance
gates are10mm/2deg. These are selected regression thresholds for this one
retained initial state, not a hardware tolerance or reliability claim. Target
motion is measured against free flight, not against the recoiling service body.

The100°×100° non-square raster is virtual anisotropic pinhole optics, as described
in CAPTURE_WRIST_OPTICS.md. No new hardware, gyro-bias diagnosis or fault
tolerance is validated by this nominal trial.
