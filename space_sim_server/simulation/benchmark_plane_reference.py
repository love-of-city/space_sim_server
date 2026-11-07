"""Smooth three-dimensional orbit-plane reference from public initial GNSS only."""
import numpy as np

from simulation.orbital_rendezvous.reference import blend
from space_arm_platform.benchmark_orbit import EARTH_MU_M3_S2 as MU, EARTH_RADIUS_M as RADIUS

_X, _W = np.polynomial.legendre.leggauss(32)


def plane_reference(initial_state, target_altitude_m, target_normal, time_s, duration_s):
    state = np.asarray(initial_state)
    r0, r1 = np.linalg.norm(state[:3]), RADIUS+target_altitude_m
    radial0 = state[:3]/r0
    normal0 = np.cross(state[:3], state[3:])
    normal0 /= np.linalg.norm(normal0)
    tangent0 = np.cross(normal0, radial0)
    target = np.asarray(target_normal, dtype=float)
    target = target/np.linalg.norm(target)
    axis = np.cross(normal0, target)
    sine, cosine = np.linalg.norm(axis), np.clip(normal0@target, -1., 1.)
    angle = np.arctan2(sine, cosine)
    axis = axis/sine if sine > 1e-12 else radial0
    u = min(max(0.,time_s),duration_s)
    b, db, ddb = blend(u,duration_s)
    radius, dr, ddr = r0+(r1-r0)*b, (r1-r0)*db, (r1-r0)*ddb
    nodes = .5*u*(1+_X)
    radii = r0+(r1-r0)*blend(nodes,duration_s)[0]
    theta = .5*u*np.dot(_W,np.sqrt(MU/radii**3))+max(0.,time_s-duration_s)*np.sqrt(MU/r1**3)
    radial = np.cos(theta)*radial0+np.sin(theta)*tangent0
    tangent = -np.sin(theta)*radial0+np.cos(theta)*tangent0
    rate = np.sqrt(MU/radius**3)

    def rotate(value):
        turn = angle*b
        return value*np.cos(turn)+np.cross(axis,value)*np.sin(turn)+axis*(axis@value)*(1-np.cos(turn))

    position = rotate(radius*radial)
    orbital_velocity = rotate(dr*radial+radius*rate*tangent)
    omega, alpha = axis*angle*db, axis*angle*ddb
    velocity = orbital_velocity+np.cross(omega,position)
    acceleration = (rotate((ddr-radius*rate**2)*radial+.5*dr*rate*tangent)
        +2*np.cross(omega,orbital_velocity)+np.cross(alpha,position)+np.cross(omega,np.cross(omega,position)))
    return position,velocity,acceleration
