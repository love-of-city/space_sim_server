# capture26 body stereo and flush fiducial integration

User-approved configuration keeps 50 mm black ArUco codes and allows thin visual film across unused target openings. Four targets are symmetric about the grab reference, with 60/150 mm X/Z offsets, and avoid raised ribs. Geometry has zero thickness, no board, no physics contribution; a 10 um display depth bias avoids z-fighting. Original handoff remains unchanged.

Two explicit body camera descriptors use 1920x1200, nominal 1739.1304348px focal length, parallel 120 mm horizontal baseline, 10 Hz simulation-clock trigger, 2ms/F4 photometric exposure. Existing cameras/preview poses are preserved. Physics XML SHA256 stays 915b175361b47edea8b5d1d173e8b6d1b2f21ad7a74e5e37fdd27d01f1714e2f.

Adapter and UE introduce an opt-in authoritative `vision` consumer, independent of dataset episodes and preview resolution overrides. Source trigger frames and PNG packets are retained/ACKed. Server pairs by session/frame and checks simulation timestamps before dispatching to an isolated image-only worker. It never records these frames as LeRobot samples.

User-provided monocular IPPE/PnP source is byte-preserved and invoked per eye with current marker geometry and static K. An additional stereo geometric baseline performs robust marker selection, triangulation and joint reprojection refinement. Planar points are internally canonicalized because OpenCV RANSAC's EPNP sampling kernel was not invariant to the live Y-constant object frame. Pose maps back to the original target frame; quality thresholds are unchanged.

Validation: 47 adapter tests plus16 subtests,28 scoped server tests,33 image/pose tests, UE build and5 native automation tests pass. Actual UE1920x1200 images detect all4 markers in both eyes, same source frame/time; final shared-frontend initial stereo reprojection RMS .634/.555px, independent frame0 translation error .262mm and rotation error .025deg. These are one ideal-rendered initial frame, not hardware accuracy or a full capture/control validation. Full server suite was not run for this change.

An actual-pair profile measured668.5ms median for the initial redundant workflow; four PNG decodes and four detections dominated. Production now shares the provided frontend's measured corners with stereo geometry, keeping original vendor code unchanged. This is explicitly a shared-frontend geometry comparison. It removes a second decode/detect pass but does not establish10Hz hardware or wall-clock throughput.

Current throughput is below10Hz wall-clock: full rendering/physics and multiple image estimators contend for resources. The service uses a bounded latest-pair queue and reports discarded old pairs, rather than hiding the backlog. Finite exposure integration/noise, physical film deformation and real hardware calibration/synchronization are outside current validation.

Configuration and operation: `space_sim_server/docs/CAPTURE26_STEREO_VISION.md`. Runtime proof is stored outside the repository in `D:/mingti/outputs/gripper_platform_integration/stereo_fiducial_v1`.
