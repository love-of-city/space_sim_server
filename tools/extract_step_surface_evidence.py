"""Read-only STEP analytic-surface evidence; NOT a kinematic-joint detector."""
import argparse, collections, hashlib, json, pathlib
import numpy as np
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.Interface import Interface_Static
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_Tool,TDF_Label
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE,TopAbs_SOLID
from OCP.TopoDS import TopoDS
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('model_folder',type=pathlib.Path)
args=parser.parse_args()
folder=args.model_folder.resolve()
m=json.loads((folder/'conversion_manifest.json').read_text(encoding='utf-8'))
source=folder.parent/m['source']['filename']
if hashlib.sha256(source.read_bytes()).hexdigest()!=m['source']['sha256']:
    raise RuntimeError('Source STEP changed since conversion')
reader=STEPCAFControl_Reader();reader.SetNameMode(True);reader.SetColorMode(True);reader.SetLayerMode(True);reader.SetSHUOMode(True)
Interface_Static.SetCVal_s('xstep.cascade.unit','MM')
print('Reading original STEP for analytic surfaces...',flush=True)
if reader.ReadFile(str(source))!=IFSelect_RetDone:raise RuntimeError('STEP read failed')
doc=TDocStd_Document(TCollection_ExtendedString('XCAF'))
if not reader.Transfer(doc):raise RuntimeError('STEP transfer failed')
tool=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
def bounds(shape):
    b=Bnd_Box();BRepBndLib.AddOptimal_s(shape,b,False,False)
    return None if b.IsVoid() else [list(b.CornerMin().Coord()),list(b.CornerMax().Coord())]
result={}
for part in m['parts']:
    label=TDF_Label();TDF_Tool.Label_s(doc.GetData(),part['source_label'],label,False)
    if label.IsNull():raise RuntimeError('missing label '+part['source_label'])
    shape=tool.GetShape_s(label)
    solids=[];exp=TopExp_Explorer(shape,TopAbs_SOLID)
    while exp.More():
        solids.append(dict(index=len(solids)+1,bounds_local_mm=bounds(exp.Current())))
        exp.Next()
    counts=collections.Counter();cylinders=[];i=0;exp=TopExp_Explorer(shape,TopAbs_FACE)
    while exp.More():
        i+=1;face=(getattr(TopoDS,'Face_s',None) or TopoDS.Face)(exp.Current());adaptor=BRepAdaptor_Surface(face,True)
        counts[str(adaptor.GetType()).split('.')[-1]]+=1
        if adaptor.GetType()==GeomAbs_Cylinder:
            c=adaptor.Cylinder();props=GProp_GProps();BRepGProp.SurfaceProperties_s(face,props)
            cylinders.append(dict(face_index=i,radius_mm=c.Radius(),axis_point_local_mm=list(c.Axis().Location().Coord()),axis_direction_local=list(c.Axis().Direction().Coord()),bounds_local_mm=bounds(face),area_mm2=float(props.Mass())))
        exp.Next()
    result[part['id']]=dict(source_label=part['source_label'],cad_faces=i,surface_types=dict(counts),bounds_local_mm=bounds(shape),solids=solids,cylinders=cylinders)
    print(part['id'],'solids=',len(solids),'faces=',i,'cylinders=',len(cylinders),flush=True)
path=folder/'articulation_review'/'cad_surface_evidence.json'
path.parent.mkdir(parents=True,exist_ok=True)
result={'schema':'step-analytic-surfaces/1','source_sha256':m['source']['sha256'],'units':'local mm before assembly transform','parts':result}
path.write_text(json.dumps(result,indent=2),encoding='utf-8')
print('Saved',path,flush=True)
