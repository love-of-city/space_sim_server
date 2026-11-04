"""Rotating orbital coordinates for continuous far-to-near handover.

Columns of the returned frame are along-track (V-bar), orbit normal, radial.
Physical states remain in J2000. Only navigation/control coordinates transform.
"""
import numpy as np


def vbar_frame(target_state):
    state = np.asarray(target_state, dtype=float)
    if state.shape != (6,) or not np.isfinite(state).all():
        raise ValueError("orbital reference requires a finite six-state estimate")
    r, v = state[:3], state[3:]
    radius = np.linalg.norm(r)
    normal = np.cross(r, v)
    momentum = np.linalg.norm(normal)
    if radius < 6_000_000 or momentum < 1:
        raise ValueError("orbital reference requires a nondegenerate Earth orbit")
    radial, normal = r / radius, normal / momentum
    tangent = np.cross(normal, radial)
    return np.column_stack((tangent, normal, radial)), np.cross(r, v) / radius**2


def rotating_relative_state(target_state, relative_inertial):
    frame, omega = vbar_frame(target_state)
    relative = np.asarray(relative_inertial, dtype=float)
    if relative.shape != (6,) or not np.isfinite(relative).all():
        raise ValueError("invalid relative navigation state")
    return np.r_[frame.T @ relative[:3], frame.T @ (relative[3:] - np.cross(omega, relative[:3]))]


def guidance_frame(handoff):
    mode = handoff.get("guidance_frame", "orbital-vbar")
    if mode not in ("orbital-vbar", "inertial-fixture"):
        raise ValueError("unknown near-field guidance frame")
    return mode


def rotating_acceleration(omega, position, rotating_velocity):
    """Centrifugal and Coriolis terms for a locally circular orbital reference."""
    return np.cross(omega, np.cross(omega, position)) + 2*np.cross(omega, rotating_velocity)
