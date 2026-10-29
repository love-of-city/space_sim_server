# Capture-wrist virtual optics

The dual-arm scene with wrist alignment enabled loads
`config/vision/capture_wrist_camera.json` at startup. The image-only PnP worker
loads the same file. The visible capture-wrist camera and separate native vision
stream receive identical optics, without changing their mount, dataset routing
or the other cameras. Main's original operation-arm RGB/fisheye/monochrome modes,
relative views, control APIs and gyro capture logic are retained.

| Parameter | Value |
| --- | --- |
| Camera | One capture-wrist camera, gripper outer end face, existing 0° optical axis |
| Raster | 1920 × 1200 |
| Horizontal / vertical FOV | 100° / 100° |
| fx / fy | 805.5356459301888 / 503.459778706368 px |
| cx / cy | 959.5 / 599.5 px |
| Distortion | None |
| Native vision rate | 10 Hz |
| Exposure descriptor | 0.002 s; photometric exposure only |

This is an ideal anisotropic pinhole simulation. The non-square raster with equal
angular spans implies unequal pixel focal lengths. It is not a physical Basler
or Kowa 6 mm lens calibration, and finite-time motion blur is not simulated.
The configuration records this provenance explicitly. Invalid resolution,
focal lengths, off-center/skew intrinsics, nonzero distortion, inconsistent FOV
or invalid exposure/rate fails rather than silently using another calibration.

The additive `vertical_field_of_view_rad` field travels with the camera
descriptor to UE. Both shared preview and authoritative capture use a custom
reversed-Z projection when this field is nonzero. Native metadata uses the same
vertical focal length. Legacy cameras retain their original projection; fisheye
keeps its existing equidistant model and does not accept this pinhole override.
Rebuild the UE plugin when updating this code; an older receiver ignores the new
field. No MJCF/model, mass, inertia, collision, dynamics or grasp-control change is
required or included.

Verification covers calibration/startup and unchanged mounts, existing camera
modes, legacy projection, malformed inputs, UE projection/parser automation and
native image/metadata checks. Basic CI and optical checks do not by themselves
constitute a new full-grasp, hardware or fault-tolerance qualification.

Publication validation on 2026-10-29: server basic CI 1670 passed; adapter basic
CI 106 passed; native transport 25 passed; frontend 251 passed and Vite build;
signalling 10 passed and syntax check; UE Editor C++ build and 41 BskUnreal
automation tests passed. A separate GPU optical fixture produced native
1920×1200 PNGs with the exact configured K, detected all four 5 cm markers and
recovered camera-target translation within 0.71 mm of the independent fixture
scorer. This fixture used a small volumetric body and bright emissive markers
to isolate projection from lighting; it did not run satellite contact dynamics.
Each raster dimension is validated within UE's supported 64..4096 range.
