"""Stable orbital phase stream shared by scene creation and episode sampling."""
import random


def sample_orbit_phase(seed: int) -> float:
    return random.Random(f"space-arm-orbit-phase-v1:{seed}").random() * 360.0
