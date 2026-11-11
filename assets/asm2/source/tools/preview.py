import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import trimesh
p=(Path(__file__).resolve().parents[1] / 'mujoco');r=json.loads((p/'../metadata/mass_properties.json').read_text());fig,axes=plt.subplots(1,2,figsize=(14,7))
for ax,indices,title in zip(axes,[(0,1),(0,2)],['Top view: X forward / Y left','Side view: X forward / Z up']):
 for i,part in enumerate(r['parts']):
  mesh=trimesh.load(p/part['mesh']);v=mesh.vertices*1000;ax.scatter(v[::15,indices[0]],v[::15,indices[1]],s=.2,alpha=.3)
  c=part['center_body_m'];ax.scatter(c[indices[0]]*1000,c[indices[1]]*1000,s=18);ax.annotate(str(i+1),(c[indices[0]]*1000,c[indices[1]]*1000))
 ax.scatter(0,0,c='red',marker='+',s=180,label='Combined COM');ax.set_title(title);ax.set_aspect('equal');ax.set_xlabel('X (mm)');ax.grid();ax.legend()
fig.text(.02,.01,' | '.join(f'{i+1} {x["name"]}: {x["mass_kg"]*1000:g}g' for i,x in enumerate(r['parts'])),fontsize=7,wrap=True)
fig.suptitle('asm2: 142 g; COM origin; CAD uniform-density / carrier surface approximation');fig.tight_layout(rect=(0,.06,1,.95));fig.savefig(p/'../preview/mass_centers.png',dpi=150)
