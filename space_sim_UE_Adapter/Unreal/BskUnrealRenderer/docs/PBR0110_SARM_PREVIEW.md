# PBR0110 arm-satellite material preview

This optional local material pack replaces only the UV-mapped visual blanket on
`/Game/BSK/Generated/SARM/base_link.base_link` (the arm-carrying satellite).
It does not change MuJoCo XML, base meshes, arm geometry, collisions, mass, joints,
or the other satellite's Foil002 material override.

## Assets and behavior

- Original blanket/master/Foil002 instances remain untouched under
  `/Game/BSK/VisualOverlays/SarmMLI`.
- An independently generated continuous-wrap mesh is stored under
  `/Game/BSK/VisualOverlays/SarmMLI_PBR0110/SM_SarmMLI_PBR0110`.
- Five Textures.com PBR0110 2K TIFFs are imported as textures under
  `/Game/BSK/Materials/PBR0110`: albedo, normal, roughness, metallic and AO.
- Height is retained as a source TIFF but is not used for displacement.
- The former 20 half-panels are replaced by 10 uninterrupted face sheets in a
  single visual mesh (118632 triangles, one UV channel). There are no artificial
  center gaps or uncovered perimeter strips. Adjacent faces meet at the corners;
  the bus underside overlaps the lower-box junction. Hardware openings and the
  open arm/top platform remain. This is a visual envelope, not flight MLI engineering.
- Base color is sRGB. Normal and scalar maps are linear; normal compression is
  BC5/TC_NORMALMAP, scalar maps use TC_MASKS. Normal green-channel inversion is
  enabled after checking the normal/height gradient relationship against the
  known Foil002 DirectX and OpenGL map pair. Source TIFFs are not modified.
- Face instance settings: TextureTiling=3, NormalStrength=0.85,
  RoughnessBias=0.10, neutral tint. Hem/rim settings now match the face to avoid
  artificial dark border bands. Backing remains a separate instance. Side UVs
  wrap continuously across adjacent corners; the texture itself retains folds.

## Local rebuild

The six original TIFFs under `ContentSource/PBR0110/maps/`, continuous OBJ/MTL
sources, and generated UE assets are tracked in this repository. After cloning,
run `git lfs pull` to materialize the textures and UE/OBJ assets.
Names and SHA-256 values are in `ContentSource/PBR0110/source.json`.
Continuous mesh source is generated from `ContentSource/PBR0110/Continuous/recipe.json`
by `scripts/generate_pbr0110_blanket.py`. This generator and its recipe are separate
from the original Foil002 blanket. With the existing SARM master already prepared:

```powershell
.\scripts\prepare_pbr0110_material.ps1 -UnrealRoot 'C:\Program Files\Epic Games\UE_5.6'
```

`prepare_runtime_materials.ps1` calls this only when the PBR0110 overlay is
selected in `Config/DefaultGame.ini`, before the generic material cache return.
Separate geometry and material signature caches prevent regeneration/reimport
of unchanged assets on every startup. Recipe/generator/output changes invalidate
the local cache.
Only the helper process launched by the preparation script is stopped on timeout;
existing user UE processes are not terminated.

Validation output is `Saved/AssetImport/pbr0110_validation.json` and
`Saved/AssetImport/pbr0110_build.log`. Missing source files cause an explicit error; run `git lfs pull` first.
This script never logs into Textures.com or purchases files.

## Selection / rollback

In `[Bsk.VisualOverlays]` of `Config/DefaultGame.ini`, use:

```ini
/Game/BSK/Generated/SARM/base_link.base_link=/Game/BSK/VisualOverlays/SarmMLI_PBR0110/SM_SarmMLI_PBR0110.SM_SarmMLI_PBR0110
```

To restore the old Foil002 blanket, change only that mapping to:

```ini
/Game/BSK/Generated/SARM/base_link.base_link=/Game/BSK/VisualOverlays/SarmMLI/SM_SarmMLI.SM_SarmMLI
```

Restart the UE renderer after changing the selection. Reloading the browser alone
will not reload already instantiated UE meshes/materials. Do not overwrite the
whole config when rolling back; other project settings may have changed.

## Preview and scope

Local static replays and actual UE screenshots are in the server workspace's
`material_previews/PBR0110/`. `render_satellite_preview.ps1 -View front` or
`-View bottom` reuses the existing SARM inspection camera/lights, allocates a
separate receiver port and never connects to the live physics stream. This is
an inspection-light preview, not a claim about lighting in every platform scene.

As of 2026-09-22, the source textures and generated dependent assets are tracked
at the repository owner's request. Inclusion in this repository does not change
the original license or grant additional usage rights. Textures.com free downloads are not
CC0. This change is for local visual evaluation, not an authorization to use the
asset in training datasets, machine learning or redistribution.
