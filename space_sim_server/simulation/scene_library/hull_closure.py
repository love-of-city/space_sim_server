"""Continuous inner shell for a CAD housing with removed payload openings.

All panels stay inside the authored hull envelope. The selected face has one
aperture equal to the task panel footprint; rear clearance never cuts a second
exterior face. Visual panels and collision panels use identical box solids.
"""
import xml.etree.ElementTree as ET

import numpy as np

from .taskbox_recess import subtract_box
from .taskbox_mount import _pose


def taskbox_frame_bounds(root, body_name='task_box_base'):
    """Read the authored mounting frame envelope, including convex proxies.

    Only the fixed root frame is used: moving drawers and protruding service
    tools must not enlarge the installation aperture.
    """
    import trimesh
    body=root.find(f'.//body[@name="{body_name}"]')
    assets={a.get('name'):a for a in root.findall('./asset/mesh')}
    lows=[]; highs=[];cache={}
    for geom in body.findall('geom'):
        p,r=_pose(geom)
        if geom.get('mesh'):
            asset=assets[geom.get('mesh')];path=asset.get('file')
            if any(k in asset.attrib for k in ('refpos','refquat')):
                raise ValueError('Mount frame mesh reference transform needs explicit handling')
            if path not in cache:cache[path]=np.asarray(trimesh.load(path,force='mesh',process=False).vertices)
            vertices=cache[path]*np.fromstring(asset.get('scale','1 1 1'),sep=' ')
            vertices=vertices@r.T+p
        elif geom.get('type')=='box':
            half=np.fromstring(geom.get('size'),sep=' ')
            vertices=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])*half
            vertices=vertices@r.T+p
        else:
            continue
        lows.append(vertices.min(0));highs.append(vertices.max(0))
    if not lows:raise ValueError('Task mounting frame has no inspectable solids')
    return np.array([np.min(lows,axis=0),np.max(highs,axis=0)])


def add_hull_closure(root, hull, aperture_low, aperture_high, *, thickness=.002,
                     inset=.0002, surface_axis=2, owner='capture_target'):
    hull = np.asarray(hull, dtype=float)
    low, high = np.asarray(aperture_low, dtype=float), np.asarray(aperture_high, dtype=float)
    tangents = [i for i in range(3) if i != surface_axis]
    if (hull.shape != (2, 3) or not np.isfinite(hull).all()
            or thickness <= 0 or inset < 0 or np.any(hull[1]-hull[0] <= 2*(thickness+inset))
            or np.any(low[tangents] <= hull[0,tangents]+thickness+inset)
            or np.any(high[tangents] >= hull[1,tangents]-thickness-inset)):
        raise ValueError('Task aperture must leave a continuous rim inside the hull')
    body = root.find(f'.//body[@name="{owner}"]')
    if body is None or any(g.get('name','').startswith('hull_closure_') for g in body):
        raise ValueError('Missing housing or closure already installed')
    panels = []
    fmt = lambda v: ' '.join(format(float(x), '.17g') for x in v)
    for axis in range(3):
        for side in (0,1):
            a,b = hull.copy()
            if side == 0:
                a[axis] += inset; b[axis] = a[axis]+thickness
            else:
                b[axis] -= inset; a[axis] = b[axis]-thickness
            pieces = [(a,b)]
            if axis == surface_axis and side == 0:
                cl,ch = low.copy(),high.copy()
                cl[axis],ch[axis] = a[axis]-1,b[axis]+1
                pieces = subtract_box(a,b,cl,ch)
            for part,(a,b) in enumerate(pieces):
                a,b = np.asarray(a),np.asarray(b)
                name = f'hull_closure_{axis}_{side}_{part}'
                attrs = dict(type='box',pos=fmt((a+b)/2),size=fmt((b-a)/2),mass='0')
                ET.SubElement(body,'geom',dict(attrs,name=name,group='2',contype='0',
                    conaffinity='0',rgba='0.64 0.68 0.73 1'))
                ET.SubElement(body,'geom',dict(attrs,name=name+'_collision',group='3',
                    contype='1',conaffinity='1',rgba='0.6 0.6 0.6 0'))
                panels.append(dict(name=name,axis=axis,side=side,bounds_m=[a.tolist(),b.tolist()]))
    return dict(schema='continuous-hull-shell/1',owner=owner,hull_bounds_m=hull.tolist(),
        aperture_bounds_m=[low.tolist(),high.tolist()],surface_axis=surface_axis,
        thickness_m=thickness,inset_m=inset,panels=panels,
        mass_rule='existing_fixed_hull_mass_and_inertia',original_geometry_preserved=True)


def audit_hull_closure(root, contract):
    """Check actual XML solids using an exact rectangular surface partition.

The check does not trust recorded panel bounds: a missing, moved, undersized or
oversized panel changes coverage. Each cell is constant-membership by design.
"""
    hull=np.asarray(contract['hull_bounds_m']); aperture=np.asarray(contract['aperture_bounds_m'])
    axis0=contract['surface_axis']; body=root.find(f'.//body[@name="{contract["owner"]}"]')
    solids=[]; errors=[]
    for geom in body.findall('geom'):
        name=geom.get('name','')
        if not name.startswith('hull_closure_') or name.endswith('_collision'):continue
        p=np.fromstring(geom.get('pos',''),sep=' '); size=np.fromstring(geom.get('size',''),sep=' ')
        peer=body.find(f'geom[@name="{name}_collision"]')
        if (geom.get('type')!='box' or 'quat' in geom.attrib or p.shape!=(3,) or size.shape!=(3,)
                or np.any(size<=0) or not np.isfinite(np.r_[p,size]).all()):
            errors.append(name+':invalid_box');continue
        if peer is None or any(peer.get(k)!=geom.get(k) for k in ('pos','size','type')) or peer.get('contype')!='1':
            errors.append(name+':collision_mismatch')
        bounds=np.array([p-size,p+size]);solids.append(bounds)
        if np.any(bounds[0]<hull[0]-1e-10) or np.any(bounds[1]>hull[1]+1e-10):errors.append(name+':outside_hull')
    checks=[];missing_area=blocked_area=0.
    for axis in range(3):
        uv=[i for i in range(3) if i!=axis]
        for side in (0,1):
            depth=contract['inset_m']+contract['thickness_m']/2
            plane=hull[side,axis]+(depth if side==0 else -depth)
            rects=[b[:,uv] for b in solids if b[0,axis]-1e-10<=plane<=b[1,axis]+1e-10]
            cuts=[]
            for j,k in enumerate(uv):
                edges=[hull[0,k],hull[1,k]]+[float(v) for b in rects for v in b[:,j]]
                if axis==axis0 and side==0:edges+=aperture[:,k].tolist()
                cuts.append(sorted(set(v for v in edges if hull[0,k]-1e-10<=v<=hull[1,k]+1e-10)))
            absent=blocked=0.
            for x0,x1 in zip(cuts[0],cuts[0][1:]):
                for y0,y1 in zip(cuts[1],cuts[1][1:]):
                    center=np.array([(x0+x1)/2,(y0+y1)/2]);area=(x1-x0)*(y1-y0)
                    if area<1e-16:continue
                    aperture_cell=(axis==axis0 and side==0 and np.all(center>aperture[0,uv]) and np.all(center<aperture[1,uv]))
                    covered=any(np.all(center>=b[0]-1e-12) and np.all(center<=b[1]+1e-12) for b in rects)
                    if not aperture_cell and not covered:absent+=area
                    if aperture_cell and covered:blocked+=area
            missing_area+=absent;blocked_area+=blocked
            checks.append(dict(axis=axis,side=side,uncovered_area_m2=absent,blocked_aperture_area_m2=blocked))
    return dict(schema='continuous-hull-audit/1',passed=not errors and missing_area<1e-10 and blocked_area<1e-10,
        faces=checks,errors=errors,uncovered_area_m2=missing_area,blocked_aperture_area_m2=blocked_area)
