"""Validate the migrated SP_space Sun asset package in the UE editor."""

from __future__ import annotations

import unreal


ASSETS = (
    ("mesh", "/Game/SpaceFX/Meshes/SM_PlanetSphere.SM_PlanetSphere"),
    ("material", "/Game/SpaceFX/Materials/M_Star.M_Star"),
    ("blueprint", "/Game/SpaceFX/Blueprints/BP_Star.BP_Star"),
    ("particle", "/Game/SpaceFX/Particles/P_Sun_Bursts.P_Sun_Bursts"),
    ("particle", "/Game/SpaceFX/Particles/P_Sun_Halo.P_Sun_Halo"),
    ("particle", "/Game/SpaceFX/Particles/P_Sun_Lines.P_Sun_Lines"),
)


for kind, path in ASSETS:
    asset = unreal.EditorAssetLibrary.load_asset(path)
    if asset is None:
        raise RuntimeError(f"missing {kind} asset: {path}")
    unreal.log(f"validated {kind}: {path}")

unreal.log("SP_space Sun asset package validated successfully")
