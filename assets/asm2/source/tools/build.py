from pathlib import Path
import json, numpy as np, trimesh
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_Label
from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
from OCP.gp import gp_Trsf, gp_Vec
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
out=(Path(__file__).resolve().parents[1] / 'mujoco')
source=(Path(__file__).resolve().parents[1] / 'cad' / 'asm2.STEP')
doc=TDocStd_Document(TCollection_ExtendedString('asm2'));r=STEPCAFControl_Reader();r.SetNameMode(True);r.ReadFile(str(source));r.Transfer(doc)
st=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main());roots=TDF_LabelSequence();st.GetFreeShapes(roots)
def walk(label):
 ref=TDF_Label()
 if st.IsReference_s(label):st.GetReferredShape_s(label,ref)
 else:ref=label
 children=TDF_LabelSequence();st.GetComponents_s(ref,children)
 if children.Length():
  for j in range(1,children.Length()+1):yield from walk(children.Value(j))
 else:
  attr=TDataStd_Name();ref.FindAttribute(TDataStd_Name.GetID_s(),attr)
  yield attr.Get().ToExtString(),st.GetShape_s(label)
parts=list(walk(roots.Value(1)));weights={'h743':10,'rk3576':15,'rk3576_body':25,'battery':36,'dmotor':6.75,'camera':7,'top2_try':7,'project2':3,'light':3,'connecter':1.5}
(out/'meshes').mkdir(exist_ok=True);records=[];meshes=[]
for i,(name,shape) in enumerate(parts):
 if name == 'battery':
  translation = gp_Trsf()
  translation.SetTranslation(gp_Vec(0.041446297937902325, 0.0, -2.142733906478395))
  shape = BRepBuilderAPI_Transform(shape, translation, True).Shape()
 props=GProp_GProps();BRepGProp.VolumeProperties_s(shape,props)
 center=np.array(props.CentreOfMass().Coord())/1000;mass=weights[name]/1000
 matrix=props.MatrixOfInertia();inertia=np.array([[matrix.Value(j,k) for k in range(1,4)] for j in range(1,4)])*mass/props.Mass()/1e6
 BRepMesh_IncrementalMesh(shape,.08,False,.2,False).Perform();path=out/'meshes'/f'{i}_{name}.stl';w=StlAPI_Writer();w.ASCIIMode=False;w.Write(shape,str(path));mesh=trimesh.load(path,force='mesh');mesh.vertices/=1000;mesh.export(path);meshes.append(mesh)
 records.append(dict(name=name,mass_kg=mass,center_m=center.tolist(),inertia_kg_m2=inertia.tolist(),mesh=str(path.relative_to(out))))
print(json.dumps(records,indent=2));(out/'../metadata/parts.json').write_text(json.dumps(records,indent=2))
