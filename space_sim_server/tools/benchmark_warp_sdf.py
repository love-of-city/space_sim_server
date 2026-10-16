"""Experimental Warp port of native/sarm_thread_sdf.cpp, used only by benchmarks.

No production imports. Keep all five modes and the 2 um normalized finite-
difference gradient; validate against the actual C++ source before stepping.
"""

import warp as wp
from mujoco_warp._src.types import vec_pluginattr


@wp.func
def tooth_radius(p: wp.vec3, phase: float, root: float, height: float) -> float:
    r2 = p[0] * p[0] + p[1] * p[1]
    c2 = float(1.0)
    s2 = float(0.0)
    if r2 > 0.0:
        c2 = (p[0] * p[0] - p[1] * p[1]) / r2
        s2 = 2.0 * p[0] * p[1] / r2
    a = 785.3981633974483 * (p[2] + phase)
    cosine = wp.cos(a) * c2 + wp.sin(a) * s2
    return root + height * 0.5 * (1.0 - cosine)


@wp.func
def distance(p: wp.vec3, mode: int) -> float:
    radius = wp.sqrt(p[0] * p[0] + p[1] * p[1])
    value = float(0.0)
    if mode == 0:
        value = wp.max(
            wp.max(radius - tooth_radius(p, 0.004, 0.0135, 0.0015), -0.060 - p[2]), p[2]
        )
    elif mode == 1:
        value = wp.max(
            wp.max(
                wp.max(
                    tooth_radius(p, 0.004, 0.0135, 0.0015) + 0.00015 - radius,
                    radius - 0.023,
                ),
                -0.020 - p[2],
            ),
            p[2],
        )
    elif mode == 2:
        value = wp.max(
            wp.max(radius - tooth_radius(p, 0.002035, 0.0105, 0.0015), -0.110 - p[2]),
            p[2],
        )
    elif mode == 3:
        value = wp.max(
            wp.max(
                wp.max(
                    tooth_radius(p, 0.001171, 0.0106, 0.0017) - radius, radius - 0.013
                ),
                -0.065 - p[2],
            ),
            p[2],
        )
    else:
        value = wp.max(
            wp.max(wp.max(0.01265 - radius, radius - 0.020), -0.065 - p[2]), p[2]
        )
    return value


@wp.func
def gradient(p: wp.vec3, mode: int) -> wp.vec3:
    eps = float(2e-6)
    g = wp.vec3(0.0)
    for i in range(3):
        plus = wp.vec3(p)
        minus = wp.vec3(p)
        plus[i] += eps
        minus[i] -= eps
        g[i] = (distance(plus, mode) - distance(minus, mode)) / (2.0 * eps)
    norm = wp.length(g)
    if norm > 1e-12:
        g = g / norm
    else:
        g = wp.vec3(1.0, 0.0, 0.0)
    return g


@wp.func
def user_sdf(p: wp.vec3, attr: vec_pluginattr, sdf_type: int) -> float:
    return distance(p, int(attr[0]))


@wp.func
def user_sdf_grad(p: wp.vec3, attr: vec_pluginattr, sdf_type: int) -> wp.vec3:
    return gradient(p, int(attr[0]))


@wp.kernel
def evaluate(
    points: wp.array(dtype=wp.vec3),
    modes: wp.array(dtype=int),
    distances: wp.array(dtype=float),
    gradients: wp.array(dtype=wp.vec3),
):
    i = wp.tid()
    distances[i] = distance(points[i], modes[i])
    gradients[i] = gradient(points[i], modes[i])


def install():
    from mujoco_warp._src import collision_sdf

    collision_sdf.user_sdf = user_sdf
    collision_sdf.user_sdf_grad = user_sdf_grad
