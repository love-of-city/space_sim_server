"""Serializable orbital environment settings; no simulator imports.

Missing settings in OLD scene files mean point-mass gravity. New scenes persist
their settings explicitly, so replay never depends on a changed default.
"""
from __future__ import annotations

import math
from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, FiniteFloat, model_validator

Positive = Annotated[FiniteFloat, Field(gt=0, strict=True)]
# Freeze the value, rather than silently changing saved physics after a BSK upgrade.
DEFAULT_EARTH_J2 = 1.082616e-3  # Basilisk 2.11.1 architecture.astroConstants.J2_EARTH
DEFAULT_J2_SOURCE = "Basilisk 2.11.1 astroConstants.J2_EARTH"


class BallisticBody(BaseModel):
    model_config = ConfigDict(extra="forbid")
    body: str = Field(min_length=1)
    area_m2: Positive
    drag_coefficient: Positive
    parameter_source: str = Field(min_length=1)


class ExponentialDragSettings(BaseModel):
    """Density anchored at an orbital altitude, not an assumed sea-level value.

    Each body receives its own effective area. No measured spacecraft aero data
    is shipped, so every enabled configuration must supply parameters and origin.
    """
    model_config = ConfigDict(extra="forbid")
    density_model: Literal["exponential"] = "exponential"
    reference_altitude_m: Annotated[FiniteFloat, Field(ge=0, strict=True)]
    reference_density_kg_m3: Positive
    scale_height_m: Positive
    density_source: str = Field(min_length=1)
    bodies: list[BallisticBody] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_parameters(self):
        names = [body.body for body in self.bodies]
        if len(names) != len(set(names)):
            raise ValueError("drag bodies must be unique; do not count an area twice")
        if not self.density_source.strip() or any(
            not b.body.strip() or not b.parameter_source.strip() for b in self.bodies
        ):
            raise ValueError("drag body names and parameter sources must not be blank")
        exponent = self.reference_altitude_m / self.scale_height_m
        if exponent > 700 or not math.isfinite(self.reference_density_kg_m3 * math.exp(exponent)):
            raise ValueError("exponential density parameters overflow the BSK base density")
        return self

    @property
    def base_density_kg_m3(self) -> float:
        return self.reference_density_kg_m3 * math.exp(self.reference_altitude_m / self.scale_height_m)


class OrbitalPhysics(BaseModel):
    model_config = ConfigDict(extra="forbid")
    earth_gravity: Literal["point_mass", "j2"] = "j2"
    j2_coefficient: Annotated[FiniteFloat, Field(gt=0, lt=0.01, strict=True)] = DEFAULT_EARTH_J2
    coefficient_source: str = Field(default=DEFAULT_J2_SOURCE, min_length=1)
    drag: ExponentialDragSettings | None = None

    @model_validator(mode="after")
    def validate_source(self):
        if not self.coefficient_source.strip():
            raise ValueError("coefficient_source must not be blank")
        return self

    @classmethod
    def legacy(cls) -> "OrbitalPhysics":
        return cls(earth_gravity="point_mass")
