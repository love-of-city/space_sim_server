from pathlib import Path
import sys,json
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'scripts'))
from generate_pbr0110_blanket import export
out=root/'ContentSource/Capture26'
original=out/'SM_Capture26_MLI_sides.obj'
if not original.exists():original.write_bytes((out/'SM_Capture26_MLI.obj').read_bytes())
recipe=json.loads((root/'ContentSource/PBR0110/Continuous/recipe.json').read_text())
recipe['faces']=[
 dict(name='capture_bottom',axis=2,sign=-1,plane=-.49,uaxis=0,vaxis=1,urange=[-.2,.6],vrange=[-.113,.643],hardware_bounds=[[.015,.135,.445,.565]],outer_inset=0,keepout_clearance=.006),
 # Task-box STL bounds + body translation: x=.125..575, y=.065..615.
 # Keep the whole task-box footprint bare, plus the rotated link1 base at origin.
 dict(name='task_face_exposed_bus',axis=2,sign=1,plane=0,uaxis=0,vaxis=1,urange=[-.2,.6],vrange=[-.113,.643],hardware_bounds=[[.125,.575,.065,.615],[-.055,.055,-.055,.055]],outer_inset=0,keepout_clearance=.006),
]
(out/'bottom_recipe.json').write_text(json.dumps(recipe,indent=2))
export(recipe,out/'bottom')
base=original.read_text().splitlines();extra=(out/'bottom/SM_SarmMLI_PBR0110.obj').read_text().splitlines()
counts=[sum(x.startswith(k+' ') for x in base) for k in ['v','vt','vn']]
for line in extra:
 if line.startswith(('mtllib ','o ')):continue
 if line.startswith('f '):
  line='f '+' '.join('/'.join(str(int(v)+counts[j]) for j,v in enumerate(token.split('/'))) for token in line.split()[1:])
 base.append(line)
(out/'SM_Capture26_MLI.obj').write_text('\n'.join(base)+'\n')
print('Combined side, bottom and exposed task-face foil; task box and arm base excluded')
