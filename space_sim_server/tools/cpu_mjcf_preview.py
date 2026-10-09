"""CPU z-buffer previews of compiled MuJoCo geometry, without OpenGL or drivers.

Images are geometric previews, NOT screenshots from MuJoCo's OpenGL renderer.
World vertices, normals, triangle indices and material colors come from the
compiled MJCF model after mj_forward. Uses a two-sided, perspective-correct
z-buffer and simple local lighting; no AI, hardware GL, textures or contact sim.
"""
from __future__ import annotations

import math
from pathlib import Path
import time

import mujoco
import numpy as np
from numba import njit
from PIL import Image, ImageDraw, ImageFont


@njit(cache=True)
def rasterize(projected, normals, faces, normal_faces, colors, width, height, toward_eye, key, fill):
    depth = np.zeros((height,width),dtype=np.float32)
    image = np.empty((height,width,3),dtype=np.float32)
    for y in range(height):
        k=y/max(height-1,1)
        for x in range(width):
            image[y,x,0]=0.014-0.006*k
            image[y,x,1]=0.024-0.01*k
            image[y,x,2]=0.04-0.014*k
    for fi in range(len(faces)):
        a,b,c=faces[fi]
        x0,y0,z0=projected[a]
        x1,y1,z1=projected[b]
        x2,y2,z2=projected[c]
        if min(z0,z1,z2)<=0:
            continue
        area=(x1-x0)*(y2-y0)-(y1-y0)*(x2-x0)
        if abs(area)<1e-7:
            continue
        xmin=max(0,int(math.floor(min(x0,x1,x2))))
        xmax=min(width-1,int(math.ceil(max(x0,x1,x2))))
        ymin=max(0,int(math.floor(min(y0,y1,y2))))
        ymax=min(height-1,int(math.ceil(max(y0,y1,y2))))
        if xmin>xmax or ymin>ymax:
            continue
        ia=1.0/area
        na,nb,nc=normal_faces[fi]
        for y in range(ymin,ymax+1):
            py=y+0.5
            for x in range(xmin,xmax+1):
                px=x+0.5
                w0=((x1-px)*(y2-py)-(y1-py)*(x2-px))*ia
                w1=((x2-px)*(y0-py)-(y2-py)*(x0-px))*ia
                w2=1-w0-w1
                if w0 < -1e-7 or w1 < -1e-7 or w2 < -1e-7:
                    continue
                invz=w0*z0+w1*z1+w2*z2
                if invz<=depth[y,x]:
                    continue
                w0*=z0/invz; w1*=z1/invz; w2*=z2/invz
                nx=w0*normals[na,0]+w1*normals[nb,0]+w2*normals[nc,0]
                ny=w0*normals[na,1]+w1*normals[nb,1]+w2*normals[nc,1]
                nz=w0*normals[na,2]+w1*normals[nb,2]+w2*normals[nc,2]
                length=math.sqrt(nx*nx+ny*ny+nz*nz)
                if length<1e-15:
                    continue
                nx/=length; ny/=length; nz/=length
                head=nx*toward_eye[0]+ny*toward_eye[1]+nz*toward_eye[2]
                if head<0:  # Two-sided CAD surfaces, with no invented thickness.
                    nx=-nx;ny=-ny;nz=-nz;head=-head
                lk=max(0.0,nx*key[0]+ny*key[1]+nz*key[2])
                lf=max(0.0,nx*fill[0]+ny*fill[1]+nz*fill[2])
                lighting=0.24+0.47*head+0.27*lk+0.14*lf
                spec=0.07*head**40
                for channel in range(3):
                    image[y,x,channel]=min(1.0,max(0.0,colors[fi,channel]*lighting+spec))
                depth[y,x]=invz
    return image,depth


def gather_geometry(xml):
    model=mujoco.MjModel.from_xml_path(str(xml))
    data=mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    vertices=[];normals=[];faces=[];normal_faces=[];colors=[]
    v_offset=0;n_offset=0
    for gid in np.where(model.geom_group==2)[0]:
        if model.geom_type[gid]!=mujoco.mjtGeom.mjGEOM_MESH:
            raise ValueError('CPU STEP preview expects group-2 mesh geoms')
        mid=model.geom_dataid[gid]
        va,nv=model.mesh_vertadr[mid],model.mesh_vertnum[mid]
        na,nn=model.mesh_normaladr[mid],model.mesh_normalnum[mid]
        fa,nf=model.mesh_faceadr[mid],model.mesh_facenum[mid]
        r=data.geom_xmat[gid].reshape(3,3)
        vertices.append(np.asarray(model.mesh_vert[va:va+nv],dtype=np.float64)@r.T+data.geom_xpos[gid])
        normals.append(np.asarray(model.mesh_normal[na:na+nn],dtype=np.float64)@r.T)
        faces.append(np.asarray(model.mesh_face[fa:fa+nf],dtype=np.int32)+v_offset)
        normal_faces.append(np.asarray(model.mesh_facenormal[fa:fa+nf],dtype=np.int32)+n_offset)
        colors.append(np.tile(model.geom_rgba[gid,:3],(nf,1)))
        v_offset+=nv;n_offset+=nn
    return (np.concatenate(vertices),np.concatenate(normals).astype(np.float32),
            np.concatenate(faces),np.concatenate(normal_faces),np.concatenate(colors).astype(np.float32))


def render_cpu_preview(folder: Path, dimensions):
    started=time.perf_counter()
    vertices,normals,faces,normal_faces,colors=gather_geometry(folder/'ground_validation_satellite.xml')
    print(f'CPU preview: {len(vertices):,} placed vertices, {len(faces):,} triangles',flush=True)
    low,high=vertices.min(0),vertices.max(0)
    center=(low+high)/2
    width,height=1600,1200
    focal=height/(2*math.tan(math.radians(38)/2))
    key=np.array([1.,-2,3]);key/=np.linalg.norm(key)
    fill=np.array([-2.,1,1]);fill/=np.linalg.norm(fill)
    # STEP's original axes are retained. Y is the thin direction in this model.
    views=[('01_isometric',120,24,'Assembly / front oblique'),
           ('02_opposite',-60,24,'Assembly / rear oblique'),
           ('03_front',90,0,'Front (+Y)'),
           ('04_top',0,65,'Upper / side')]
    result=[]
    metadata=[]
    (folder/'preview').mkdir(exist_ok=True)
    for name,az,el,label in views:
        az,el=math.radians(az),math.radians(el)
        radial=np.array([math.cos(el)*math.cos(az),math.cos(el)*math.sin(az),math.sin(el)])
        forward=-radial
        right=np.cross(forward,np.array([0.,0,1.]));right/=np.linalg.norm(right)
        up=np.cross(right,forward)
        relative=vertices-center
        depth_offset=relative@radial
        half_y=math.tan(math.radians(38)/2)*0.86
        half_x=half_y*width/height
        distance=max(float(np.max(depth_offset+np.abs(relative@up)/half_y)),
                     float(np.max(depth_offset+np.abs(relative@right)/half_x)))
        distance+=0.01*float(np.linalg.norm(dimensions))
        eye=center+distance*radial
        cam=(vertices-eye)@np.array([right,up,forward]).T
        projected=np.empty_like(cam)
        projected[:,0]=width/2+focal*cam[:,0]/cam[:,2]
        projected[:,1]=height/2-focal*cam[:,1]/cam[:,2]
        projected[:,2]=1/cam[:,2]
        if np.any(cam[:,2]<=0) or projected[:,0].min()<0 or projected[:,0].max()>=width or projected[:,1].min()<0 or projected[:,1].max()>=height:
            raise AssertionError('Camera fitting clipped model vertices')
        pixels,depth=rasterize(projected,normals,faces,normal_faces,colors,width,height,radial,key,fill)
        foreground=int(np.count_nonzero(depth))
        if foreground<1000:
            raise AssertionError('CPU preview is empty or camera framing is invalid')
        srgb=np.where(pixels<=0.0031308,pixels*12.92,1.055*np.power(np.maximum(pixels,0),1/2.4)-0.055)
        image=Image.fromarray(np.asarray(np.clip(srgb*255,0,255),dtype=np.uint8)).resize((1200,900),Image.Resampling.LANCZOS)
        image.save(folder/'preview'/(name+'.png'))
        result.append((label,image))
        metadata.append(dict(file=name+'.png',foreground_pixels=foreground))
        print(f'  Rendered {name}, foreground pixels={foreground:,}',flush=True)
    sheet=Image.new('RGB',(1600,1330),(17,25,37))
    draw=ImageDraw.Draw(sheet)
    try:
        font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',26)
        small=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',20)
    except OSError:
        font=small=ImageFont.load_default()
    draw.text((24,14),'STEP -> MJCF | Ground-validation satellite',font=font,fill=(225,235,248))
    draw.text((24,49),'CPU geometry preview from compiled MuJoCo poses / original CAD colors / no inferred joints',font=small,fill=(160,182,206))
    for i,(label,image) in enumerate(result):
        x,y=(i%2)*800,86+(i//2)*600
        sheet.paste(image.resize((800,600),Image.Resampling.LANCZOS),(x,y))
        draw.text((x+20,y+12),label,font=small,fill=(230,238,248))
    draw.text((24,1292),f'X/Y/Z = {dimensions[0]:.3f} / {dimensions[1]:.4f} / {dimensions[2]:.4f} m  |  {len(faces):,} placed triangles',font=small,fill=(160,182,206))
    sheet.save(folder/'preview'/'overview.png')
    return dict(overview='preview/overview.png',renderer='Custom CPU z-buffer rasterizer using compiled MuJoCo meshes and world poses',
                mujoco_opengl_renderer=False,ai_generated=False,two_sided=True,
                textures=False,views=metadata,elapsed_seconds=time.perf_counter()-started)
