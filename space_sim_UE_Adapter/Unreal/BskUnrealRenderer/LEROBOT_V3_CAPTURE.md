# LeRobot v3 authoritative camera producer

This worktree pairs with `C:\Users\LYH\space_sim_server_lerobot_v3` on
`feat/lerobot-v3-dataset`. It does **not** convert old camera recordings.

- `BskLeRobotSampling.h` selects samples on the simulation-time grid from the
  30 Hz authoritative frame stream. Supported FPS: 1, 2, 5, 10, 30. Physics runs
  at 240 Hz, IK at 120 Hz, rendering every eighth physics step. Timestamps use
  rounded absolute rational times, not accumulated rounded periods. The producer
  and server require the same exact integer-nanosecond sampling grid.
- Authoritative `bsk-capture/1` metadata now includes `dataset_format=lerobot-v3`,
  `sampling_fps`, decimal-string `sample_index`, and `sampling_clock=simulation`.
- RGB is the only supported capture product. Depth and segmentation rendering,
  readback, encoding, and metadata have been removed (2026-09-21). Render-session,
  source-frame, sim-time and calibration are retained. Preview is separate.
- Cameras due on the same grid point capture the exact same authoritative scene
  state. Duplicate timestamps are suppressed. No latest-preview replacement.
- The Python platform owns episode lifecycle and the official LeRobot writer.
  It verifies every required camera and produces Parquet, MP4, task/episode
  metadata and statistics when recording stops. Missing frames fail the dataset
  rather than becoming fabricated samples. Do not use the old main UE binary.

Build using `Unreal/BskUnrealRenderer/scripts/build.ps1`. The automation test is
`BskUnreal.Dataset.LeRobotSampling` (including reset and integer-time rounding).

The server's `docs/LEROBOT_V3_CAPTURE.md` documents all dataset features and the
standalone-per-recording dataset layout, with RGB video and no depth/segmentation attachments.

Dataset-mode `RenderPublisher(reliable_frames=True)` and `FBskTcpReceiver` use
bounded FIFO queues/backpressure rather than latest-wins replacement. Preview
mode retains the legacy behavior. The additional automation test
`BskUnreal.Dataset.ReliableFrames` verifies ordering and reset queue barriers.
