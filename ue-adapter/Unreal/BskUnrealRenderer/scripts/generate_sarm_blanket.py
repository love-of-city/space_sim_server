"""Generate only a separate UV-mapped, closed visual blanket mesh (UE centimetres).

Does not edit source meshes or MJCF. Rebuild from the checked-in fitting recipe.
The recipe's dimensions/keepouts are visual design choices, not flight hardware.
"""
from __future__ import annotations
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'ContentSource/SarmMLI'


def padded_boxes(face, clearance):
    return [(a-clearance,b+clearance,c-clearance,d+clearance) for a,b,c,d in face['hardware_bounds']]


def in_box(u,v,box):
    a,b,c,d=box
    return a < u < b and c < v < d


def edge_distance(u,v,bounds,boxes):
    a,b,c,d=bounds
    distance=min(u-a,b-u,v-c,d-v)
    for x0,x1,y0,y1 in boxes:
        distance=min(distance,math.hypot(max(x0-u,0,u-x1),max(y0-v,0,v-y1)))
    return max(0.,distance)


def height(u,v,bounds,boxes,recipe,phase):
    a,b,c,d=bounds
    distance=edge_distance(u,v,bounds,boxes)
    t=min(distance/.025,1.)
    envelope=t*t*(3-2*t)
    # Millimetre-scale broad folds are geometry, not just a normal-map illusion.
    wave=.0005+.0011*math.sin(21*u+11*v+phase)**2+.0008*math.sin(8*u-24*v+phase*.7)**2
    bead=.00045*math.exp(-((distance-.004)/.002)**2)
    return recipe['standoff']+envelope*wave+bead


def grid_points(low,high,extra,step):
    values=[low,high,*np.linspace(low,high,math.ceil((high-low)/step)+1),*extra]
    return sorted(set(round(float(v),8) for v in values if low <= v <= high))


def generate(recipe):
    vertices=[];uvs=[];faces=[];materials=[];pieces=[]
    for side,face in enumerate(recipe['faces']):
        inset=face.get('outer_inset',recipe['outer_inset']);gap=recipe['seam_gap'];hem=recipe['hem_width']
        l,r=face['urange'];bottom,top=face['vrange'];mid=(l+r)*.5
        boxes=padded_boxes(face,face.get('keepout_clearance',recipe['keepout_clearance']))
        for half,(left,right) in enumerate(((l+inset,mid-gap/2),(mid+gap/2,r-inset))):
            bounds=(left,right,bottom+inset,top-inset);a,b,c,d=bounds
            index=side*2+half
            us=grid_points(a,b,[a+hem,b-hem,*[v+delta for box in boxes for v in box[:2] for delta in (-hem,0,hem)]],recipe['grid_spacing'])
            vs=grid_points(c,d,[c+hem,d-hem,*[v+delta for box in boxes for v in box[2:] for delta in (-hem,0,hem)]],recipe['grid_spacing'])
            grid={};points=[];texcoords=[];triangles=[];mats=[]
            def vertex(i,j):
                if (i,j) in grid:return grid[i,j]
                u,v=us[i],vs[j]
                p=[0.,0.,0.];p[face['uaxis']]=u;p[face['vaxis']]=v
                p[face['axis']]=face['plane']+face['sign']*height(u,v,bounds,boxes,recipe,index)
                n=len(points);grid[i,j]=n;points.append(p)
                # Separate front UV islands; extra rows retain existing side UVs and texel density.
                texcoords.append([.25*(index%4)+.025+(u-a)*recipe['atlas_scale_per_m'],.5*(index//4)+.025+(v-c)*recipe['atlas_scale_per_m']])
                return n
            eu=np.eye(3)[face['uaxis']];ev=np.eye(3)[face['vaxis']]
            positive=np.cross(eu,ev)[face['axis']]*face['sign']>0
            for i in range(len(us)-1):
                for j in range(len(vs)-1):
                    u=(us[i]+us[i+1])*.5;v=(vs[j]+vs[j+1])*.5
                    if any(in_box(u,v,box) for box in boxes):continue
                    aa,bb,cc,dd=vertex(i,j),vertex(i+1,j),vertex(i+1,j+1),vertex(i,j+1)
                    pair=[(aa,bb,cc),(aa,cc,dd)] if positive else [(aa,cc,bb),(aa,dd,cc)]
                    triangles.extend(pair)
                    mats.extend(['MliHem' if edge_distance(u,v,bounds,boxes)<hem+1e-7 else 'MliFace']*2)
            points=np.asarray(points,dtype=float);count=len(points);offset=len(vertices)
            back=points.copy();back[:,face['axis']]-=face['sign']*recipe['visual_thickness']
            vertices.extend(points.tolist());vertices.extend(back.tolist());uvs.extend(texcoords);uvs.extend(texcoords)
            front_count=len(triangles)
            edges=Counter(tuple(sorted((a,b))) for tri in triangles for a,b in zip(tri,(tri[1],tri[2],tri[0])))
            boundaries=[(a,b) for tri in triangles for a,b in zip(tri,(tri[1],tri[2],tri[0])) if edges[tuple(sorted((a,b)))]==1]
            local_faces=list(triangles)+[(a+count,c+count,b+count) for a,b,c in triangles]
            local_mats=mats+['MliBacking']*front_count
            for a,b in boundaries:
                local_faces.extend([(a,a+count,b+count),(a,b+count,b)])
                local_mats.extend(['MliHem']*2)
            faces.extend([tuple(i+offset for i in f) for f in local_faces]);materials.extend(local_mats)
            pieces.append(dict(name=f"{face['name']}_{half}",face=face['name'],bounds_uv_m=list(bounds),vertex_start=offset,front_vertex_count=count,face_start=len(faces)-len(local_faces),front_triangle_count=front_count,triangle_count=len(local_faces),keepouts_m=boxes,uv_island=index))
    return np.array(vertices),np.array(uvs),np.array(faces),materials,pieces


def export(recipe,output):
    vertices,uvs,faces,materials,pieces=generate(recipe)
    triangles=vertices[faces]
    fn=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
    if np.any(np.linalg.norm(fn,axis=1)<1e-14):raise ValueError('Degenerate blanket triangles')
    normals=np.zeros_like(vertices)
    # Smooth front/back surfaces, but give closing rim faces their own normals.
    surface_mask=np.zeros(len(faces),bool)
    for p in pieces:surface_mask[p['face_start']:p['face_start']+2*p['front_triangle_count']]=True
    for corner in range(3):np.add.at(normals,faces[surface_mask,corner],fn[surface_mask])
    normals/=np.maximum(np.linalg.norm(normals,axis=1)[:,None],1e-30)
    if np.any(np.linalg.norm(normals,axis=1)<.99):raise ValueError('Missing surface normal')
    output.mkdir(parents=True,exist_ok=True)
    # Bake metre-to-centimetre conversion into this NEW visual mesh only.
    # Avoid imported-bounds mismatches and tiny-scale tangent degeneracy.
    extra_uvs=[];rim_uv_indices={}
    for i in np.where(~surface_mask)[0]:
        tri=vertices[faces[i]];t=tri[1]-tri[0];t/=np.linalg.norm(t)
        n=fn[i]/np.linalg.norm(fn[i]);b=np.cross(n,t)
        rim_uv_indices[int(i)]=tuple(len(uvs)+len(extra_uvs)+j+1 for j in range(3))
        extra_uvs.extend([[float(np.dot(p-tri[0],t)*recipe['atlas_scale_per_m']),float(np.dot(p-tri[0],b)*recipe['atlas_scale_per_m'])] for p in tri])
    lines=['# Visual-only MLI shell, UE centimetres; recipe is metres; no physical contacts','mtllib SM_SarmMLI.mtl','o SM_SarmMLI']
    lines += ['v '+' '.join(f'{a:.9f}' for a in v*100.) for v in vertices]
    lines += ['vt '+' '.join(f'{a:.9f}' for a in uv) for uv in [*uvs,*extra_uvs]]
    lines += ['vn '+' '.join(f'{a:.9f}' for a in n) for n in normals]
    rim_indices={}
    for i in np.where(~surface_mask)[0]:
        normal=fn[i]/np.linalg.norm(fn[i]);rim_indices[int(i)]=len(normals)+len(rim_indices)+1
        lines.append('vn '+' '.join(f'{a:.9f}' for a in normal))
    # Group by material without splitting the object into unrelated assets.
    for material in ('MliFace','MliHem','MliBacking'):
        lines.append('usemtl '+material)
        for i,(face,mat) in enumerate(zip(faces,materials)):
            if mat != material:continue
            vt=rim_uv_indices.get(i,tuple(v+1 for v in face))
            lines.append('f '+' '.join(f'{v+1}/{tex}/{rim_indices.get(i,v+1)}' for v,tex in zip(face,vt)))
    mesh=output/'SM_SarmMLI.obj';mesh.write_text('\n'.join(lines)+'\n',encoding='utf-8')
    (output/'SM_SarmMLI.mtl').write_text('\n'.join(f'newmtl {name}\nKd {color}\nNs 20\nd 1\n' for name,color in [('MliFace','0.8 0.65 0.2'),('MliHem','0.4 0.25 0.07'),('MliBacking','0.05 0.04 0.03')]),encoding='utf-8')
    manifest=dict(schema='bsk-visual-blanket-mesh/1',obj_length_unit='cm',source_sha256=recipe['source_sha256'],recipe_sha256=hashlib.sha256(json.dumps(recipe,sort_keys=True).encode()).hexdigest(),mesh_sha256=hashlib.sha256(mesh.read_bytes()).hexdigest(),vertices=len(vertices),triangles=len(faces),front_uv_islands=len(pieces),bounds_m=[vertices.min(0).tolist(),vertices.max(0).tolist()],material_triangles=dict(Counter(materials)),physics='none',pieces=pieces)
    (output/'mesh_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    print(f"MLI mesh: {len(pieces)} pieces, {len(vertices)} vertices, {len(faces)} triangles")
    return manifest


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--recipe',type=Path,default=SOURCE/'recipe.json');parser.add_argument('--output',type=Path,default=SOURCE);parser.add_argument('--source-base',type=Path)
    args=parser.parse_args();recipe=json.loads(args.recipe.read_text(encoding='utf-8'))
    if args.source_base and hashlib.sha256(args.source_base.read_bytes()).hexdigest()!=recipe['source_sha256']:raise ValueError('Base mesh changed; refit the visual envelope before building')
    export(recipe,args.output)
