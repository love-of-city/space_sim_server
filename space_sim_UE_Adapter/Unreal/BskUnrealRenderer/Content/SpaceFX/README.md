# SP_space Sun Assets

This directory contains the Unreal Engine assets copied from
`universe/SP_space/Content/SpaceFX` for the BSK Unreal renderer.

The renderer currently uses these two assets for the visible sun:

- `/Game/SpaceFX/Meshes/SM_PlanetSphere.SM_PlanetSphere`
- `/Game/SpaceFX/Materials/M_Star.M_Star`

It also instantiates the three legacy Cascade particle systems used by the
source `BP_Star` presentation:

- `/Game/SpaceFX/Particles/P_Sun_Bursts.P_Sun_Bursts`
- `/Game/SpaceFX/Particles/P_Sun_Halo.P_Sun_Halo`
- `/Game/SpaceFX/Particles/P_Sun_Lines.P_Sun_Lines`

The remaining assets are retained because `BP_Star` and its visual effects
reference the same SpaceFX package. `BP_Star` itself is not spawned as an
independent ticking actor; the runtime creates equivalent visual components
so BSK/SPICE remains the source of truth.

If the effects are too large for a particular camera or GPU configuration,
change `sun_visual_effect_scale` in `Config/bsk_unreal_scene.json`. Set
`sun_visual_effects_enabled` to `false` for authoritative camera captures
that should not include the animated presentation layer.
