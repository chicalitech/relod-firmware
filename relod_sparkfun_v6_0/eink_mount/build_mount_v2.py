"""Measured-panel revision. All geometry is in mm; original viewing slot is preserved."""
from pathlib import Path
import sys, json, hashlib
import numpy as np
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'_deps'))
import trimesh
from manifold3d import Manifold, Mesh

SOURCE=Path(r'C:\Users\choro\Documents\relod_projects\relod_solidworks\LidTop_v4_hole_recess_eink_small.STL')
OUT=ROOT/'output_v2'
OUT.mkdir(exist_ok=True)
CX,CY=76.9204387665,75.0721282959
FACE=1.90500164
# USER-MEASURED OUTLINE, superseding the earlier catalog-based design.
PANEL_W,PANEL_H,PANEL_T=36.71,79.05,0.95
CLEARANCE=0.5
XL,XR=CX-PANEL_W/2,CX+PANEL_W/2
YB,YT=CY-PANEL_H/2,CY+PANEL_H/2
IX0,IX1=XL-CLEARANCE,XR+CLEARANCE
IY0,IY1=YB-CLEARANCE,YT+CLEARANCE
WALL=1.6
OX0,OX1=IX0-WALL,IX1+WALL
OY0,OY1=IY0-WALL,IY1+WALL
SEAT=FACE+0.8
FRONT_PAD,REAR_PAD=0.3,0.5
PANEL_Z=SEAT+FRONT_PAD
STOP=PANEL_Z+PANEL_T+REAR_PAD
RT=2.4
AW,AH=31.9,55.0
HOLES=[(x,CY+dy) for x in (OX0-3.2,OX1+3.2) for dy in (-30,30)]
names=['LidTop_measured_panel_mount_v2.stl','Rear_retainer_measured_panel_v2.stl','Mount_fit_coupon_v2.stl']

def box(x0,x1,y0,y1,z0,z1):
    return Manifold.cube((x1-x0,y1-y0,z1-z0)).translate((x0,y0,z0))
def cyl(x,y,r,z0,z1):
    return Manifold.cylinder(z1-z0,r,r,64).translate((x,y,z0))
def tm(m):
    a=m.to_mesh()
    return trimesh.Trimesh(vertices=np.asarray(a.vert_properties)[:,:3],faces=np.asarray(a.tri_verts),process=False)
def record(m,name):
    m=m.simplify(0.0001)
    assert str(m.status()).endswith('NoError'),(name,m.status())
    assert len(m.decompose())==1,(name,'disconnected solids')
    mesh=tm(m)
    assert mesh.is_watertight and mesh.is_winding_consistent and mesh.volume>0,name
    mesh.export(OUT/name)
    reread=trimesh.load_mesh(OUT/name)
    assert reread.is_watertight and reread.is_winding_consistent,name
    return {'file':name,'triangles':len(mesh.faces),'watertight':True,'connected_solids':1,'bounds_mm':mesh.bounds.tolist(),'volume_mm3':float(mesh.volume)}

original=trimesh.load_mesh(SOURCE)
original.merge_vertices(digits_vertex=4)
original.update_faces(original.nondegenerate_faces())
original.remove_unreferenced_vertices()
assert original.is_watertight and original.is_winding_consistent
base=Manifold(Mesh(np.asarray(original.vertices,dtype=np.float32),np.asarray(original.faces,dtype=np.uint32)))

# Supports stand on the existing inside face, entirely outside the original slot.
supports=Manifold()
# Narrow ledges beneath the long outer borders; soft strips go on these ledges.
for a,b in [(XL+0.3,XL+1.7),(XR-1.7,XR-0.3)]:
    supports=supports+box(a,b,YB+10,YT-3,FACE-0.2,SEAT)
# Side guides, continuous upper stop, and two split lower stops.
for a,b in [(OX0,IX0),(IX1,OX1)]:
    supports=supports+box(a,b,IY0,OY1,FACE-0.2,STOP)
supports=supports+box(OX0,OX1,IY1,OY1,FACE-0.2,STOP)
for a,b in [(OX0,XL+2.5),(XR-2.5,OX1)]:
    supports=supports+box(a,b,OY0,IY0,FACE-0.2,STOP)
# Ears are above/below the adjacent sensor feature, so the wider mount clears it.
for x,y in HOLES:
    supports=supports+cyl(x,y,3.2,FACE-0.2,STOP)
    a,b=(x,OX0+0.4) if x<CX else (OX1-0.4,x)
    supports=supports+box(a,b,y-3.2,y+3.2,FACE-0.2,STOP)
lid=base+supports
for x,y in HOLES:
    lid=lid-cyl(x,y,1.0,FACE,STOP+1)

# Rear frame captures the outer borders, open at the short ribbon end.
rear=box(OX0,OX1,IY0,OY1,STOP,STOP+RT)
rear=rear-box(XL+1.7,XR-1.7,IY0-1,YT-1.7,STOP-1,STOP+RT+1)
for x,y in HOLES:
    rear=rear+cyl(x,y,3.2,STOP,STOP+RT)
    a,b=(x,OX0+0.4) if x<CX else (OX1-0.4,x)
    rear=rear+box(a,b,y-3.2,y+3.2,STOP,STOP+RT)
for x,y in HOLES:
    rear=rear-cyl(x,y,1.4,STOP-1,STOP+RT+1)

panel=box(XL,XR,YB,YT,PANEL_Z,PANEL_Z+PANEL_T)
# This is a robustness allowance around the user's measurement, not a datasheet tolerance.
envelope=box(XL-0.1,XR+0.1,YB-0.1,YT+0.1,PANEL_Z,PANEL_Z+PANEL_T+0.1)
insertion=box(XL-0.1,XR+0.1,YB-0.1,YT+0.1,PANEL_Z,35)
aperture=box(CX-AW/2+0.0002,CX+AW/2-0.0002,CY-AH/2+0.0002,CY+AH/2-0.0002,-1,STOP+RT+1)
checks={
    'panel_collision_mm3':(envelope^lid).volume(),
    'panel_retainer_collision_mm3':(envelope^rear).volume(),
    'straight_insertion_collision_mm3':(insertion^lid).volume(),
    'lid_retainer_collision_mm3':(rear^lid).volume(),
    'original_slot_obstruction_mm3':(aperture^lid).volume(),
    'original_material_removed_mm3':(base-lid).volume(),
}
for name,value in checks.items():assert abs(value)<1e-6,(name,value)
# Separate fit piece reproduces all supports over an isolated portion of the flat lid.
coupon_base=box(OX0-6.8,OX1+6.8,OY0-1,OY1+1,0,FACE)
coupon_base=coupon_base-box(CX-AW/2,CX+AW/2,CY-AH/2,CY+AH/2,-1,FACE+1)
coupon=coupon_base+supports
for x,y in HOLES:coupon=coupon-cyl(x,y,1.0,FACE,STOP+1)

records=[record(lid,names[0])]
rear_min=tm(rear).bounds[0]
records.append(record(rear.translate(tuple(-rear_min)),names[1]))
coupon_min=tm(coupon).bounds[0]
records.append(record(coupon.translate(tuple(-coupon_min)),names[2]))
for m,name in [(rear,'rear_assembled'),(panel,'panel_reference'),(lid-base,'added_features'),(coupon,'coupon_assembled')]:
    tm(m).export(ROOT/(name+'_v2.stl'))

report={
    'revision':2,'panel_dimension_source':'User measurements on 2026-09-09, excluding ribbon',
    'panel_width_height_thickness_mm':[PANEL_W,PANEL_H,PANEL_T],
    'pocket_width_height_mm':[PANEL_W+2*CLEARANCE,PANEL_H+2*CLEARANCE],
    'clearance_each_side_mm':CLEARANCE,'original_slot_width_height_mm':[AW,AH],
    'original_slot_preserved':True,'panel_center_xy_mm':[CX,CY],
    'placement_assumption':'Panel outline centered on existing slot; no active-area offset inferred from earlier product datasheet.',
    'ribbon_exit':'short end toward negative Y / bottom in inside preview',
    'front_cushion_installed_mm':FRONT_PAD,'rear_cushion_installed_mm':REAR_PAD,
    'seat_z_mm':SEAT,'panel_front_z_mm':PANEL_Z,'retainer_stop_z_mm':STOP,
    'retainer_thickness_mm':RT,'pilot_diameter_mm':2.0,'pilot_depth_mm':STOP-FACE,
    'retainer_clearance_hole_mm':2.8,'screw_centers_xy_mm':HOLES,
    'hardware':'4 M2.5 x 5 mm plastic thread-forming pan-head screws, each with 0.5 mm washer; verify pilot fit on coupon.',
    'source_stl':str(SOURCE),'source_sha256':hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
    'units':'mm; source STL interpreted in mm',
    'mesh_preprocessing':'Merged source connectivity at 0.0001 mm precision; simplified export slivers at 0.0001 mm tolerance.',
    'validation':checks,'physical_fit_tested':False,'files':records,
}
(OUT/'dimensions_and_checks_v2.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
