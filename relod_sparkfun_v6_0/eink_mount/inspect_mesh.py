from pathlib import Path
import json
import numpy as np
from PIL import Image, ImageDraw

SOURCE = Path(r'C:\Users\choro\Documents\relod_projects\relod_solidworks\LidTop_v4_hole_recess_eink_small.STL')
OUT = Path(__file__).parent
data = SOURCE.read_bytes()
mesh = np.frombuffer(data, offset=84, dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('attr','<u2')]))
v = mesh['v'].astype(float)
area = np.linalg.norm(np.cross(v[:,1]-v[:,0],v[:,2]-v[:,0]),axis=1)/2
print('Window wall bounds:')
for axis, value in [(0,60.9704),(0,92.8704),(1,47.5721),(1,102.5721)]:
    sel = np.all(np.abs(v[:,:,axis]-value)<0.001,axis=1)
    print(axis,value,v[sel].min((0,1)),v[sel].max((0,1)))

im=Image.new('RGB',(1000,1060),'#ffffff')
d=ImageDraw.Draw(im)
scale=6.5
def project(p):
    return (80+(p[0]-12)*scale,960-(p[1]-11)*scale)
for i in np.argsort(v[:,:,2].mean(1)):
    z=v[i,:,2].mean()
    color=tuple([int(max(100,220-z*3))]*3)
    d.polygon([project(p) for p in v[i]], fill=color)
# Draw only physical crease/boundary edges, avoiding coplanar triangle diagonals.
edges={}
for i,tri in enumerate(v):
    for j in range(3):
        edge=tuple(sorted((tuple(np.round(tri[j],4)),tuple(np.round(tri[(j+1)%3],4)))))
        edges.setdefault(edge,[]).append(i)
for edge,faces in edges.items():
    if len(faces)!=2 or abs(np.dot(mesh['n'][faces[0]],mesh['n'][faces[1]]))<0.995:
        d.line([project(p) for p in edge], fill='#40464c',width=1)
rect=[project((60.9704,102.5721,0)),project((92.8704,47.5721,0))]
d.rectangle(rect,outline='#007ca8',width=3)
d.text((40,24),'INSIDE VIEW — source STL, interpreted in mm',fill='black',font_size=24)
d.text((rect[0][0]+4,rect[0][1]+30),'Opening',fill='#007ca8',font_size=20)
d.text((rect[0][0]+4,rect[0][1]+58),'31.90 x 55.00',fill='#007ca8',font_size=20)
im.save(OUT/'source_inside_view.png')
report={'source':str(SOURCE),'units':'assumed millimeters (STL is unitless)','bounds':v.min((0,1)).tolist()+v.max((0,1)).tolist(),'triangles':len(v),'opening_mm':[31.9,55.0],'inside_face_z_mm':1.905,'non_two_manifold_edges':sum(len(fs)!=2 for fs in edges.values())}
(OUT/'source_inspection.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
