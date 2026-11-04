"""Truth-state, ideal-actuator formation experiment. SI units throughout."""
import numpy as np

def service_body_rotation(quaternion_wxyz=None):
    """Constant body orientation relative to the existing orbital triad."""
    if quaternion_wxyz is None:
        return np.eye(3)
    q = np.asarray(quaternion_wxyz, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-12:
        raise ValueError('service body rotation must be a finite nonzero wxyz quaternion')
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def reference(r, v, separation, *, service_body_rotation_wxyz=None,
              service_com_body_m=None, target_com_body_m=None):
    """Place COMs on one circle, then convert to body-origin states.

    Zero offsets preserve legacy body-origin initialization. Offsets include
    every attached link at the selected initial joint configuration; all joint
    rates must initially be zero. Body spin is the common orbital rate.
    """
    offsets = []
    for value in (service_com_body_m, target_com_body_m):
        offset = np.zeros(3) if value is None else np.asarray(value, dtype=float)
        if offset.shape != (3,) or not np.all(np.isfinite(offset)):
            raise ValueError('COM offset must be a finite three-vector')
        offsets.append(offset)
    r,v=np.asarray(r,dtype=float),np.asarray(v,dtype=float)
    radius=np.linalg.norm(r); er=r/radius
    h=np.cross(r,v); en=h/np.linalg.norm(h); et=np.cross(en,er)
    angle=float(separation)/radius
    c,s=np.cos(angle),np.sin(angle)
    er2=c*er+s*et; et2=-s*er+c*et
    rs=np.column_stack((er,en,-et))
    rs2=np.column_stack((er2,en,-et2))
    rx=np.array([[1.,0,0],[0,0,-1],[0,1,0]])
    omega=h/radius**2
    service_rotation = rs @ service_body_rotation(service_body_rotation_wxyz)
    target_rotation = rs2 @ rx
    service_offset = service_rotation @ offsets[0]
    target_offset = target_rotation @ offsets[1]
    target_com_position = radius * er2
    target_com_velocity = np.dot(v, er) * er2 + np.dot(v, et) * et2
    return dict(service_rotation=service_rotation, target_rotation=target_rotation,
                service_position=r-service_offset,
                service_velocity=v-np.cross(omega, service_offset),
                target_position=target_com_position-target_offset,
                target_velocity=target_com_velocity-np.cross(omega, target_offset),
                service_com_position=r.copy(), service_com_velocity=v.copy(),
                target_com_position=target_com_position, target_com_velocity=target_com_velocity,
                omega=omega)

def limit_norm(x, limit):
    x=np.asarray(x,dtype=float)
    return x*min(1.,float(limit)/max(np.linalg.norm(x),1e-15))

def bounded_force(position_error, velocity_error, mass, limit, feedforward=None):
    a=.04*np.asarray(position_error)+.4*np.asarray(velocity_error)
    if feedforward is not None: a=a+np.asarray(feedforward)
    return limit_norm(float(mass)*a,limit)
