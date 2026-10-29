# Independent capture-wrist projection publication

Selectively ported the local 100° × 100° capture-wrist optics onto main at
`5067bde614bdf430ef959985aecfea1a1e08d539`. The JSON calibration, startup camera
descriptors, PnP intrinsics, optional UE vertical FOV, projection and metadata
ship together. Main's newer camera modes, relative views, rendering cadence,
policy/flight and gyro capture remain on the main implementation.

The original experimental worktree and physical model/configuration remain
unchanged. See `space_sim_server/docs/CAPTURE_WRIST_OPTICS.md` for values,
compatibility and acceptance boundaries. Local publication evidence is retained
outside the repository under
`D:/mingti/outputs/gripper_platform_integration/camera_100_publish_20261029`.
