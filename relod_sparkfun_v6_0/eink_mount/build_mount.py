"""Build a prototype mount for Adafruit 6383. Coordinates and exports are mm."""
from pathlib import Path
import sys, json, hashlib
import numpy as np
ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / '_deps'))
import trimesh
from manifold3d import Manifold, Mesh

SOURCE = Path(r'C:\Users\choro\Documents\relod_projects\relod_solidworks\LidTop_v4_hole_recess_eink_small.STL')
OUT = ROOT / 'output'
OUT.mkdir(exist_ok=True)
CX, CY = 76.9204387665, 75.0721282959
FACE = 1.90500164
PANEL_W, PANEL_H, PANEL_T = 29.2, 59.2, 1.0
# The active area is 2.625 mm toward the non-ribbon end from panel center.
PX, PY = CX, CY - 2.625
XL, XR = PX-PANEL_W/2, PX+PANEL_W/2
YB, YT = PY-PANEL_H/2, PY+PANEL_H/2
CLEARANCE = 0.30  # per side, prototype FDM allowance
SEAT = FACE + 0.8
FRONT_PAD = 0.30  # installed thickness, not uncompressed foam specification
REAR_PAD = 0.50
PANEL_Z = SEAT + FRONT_PAD
STOP = PANEL_Z + PANEL_T + REAR_PAD
RETAINER_T = 2.4
AW, AH = 25.30, 50.15
OX0, OX1 = CX-17.95, CX+17.95
OY0, OY1 = YB-1.90, YT+2.90
HOLES = [(CX+dx,CY+dy) for dx in (-19.5,19.5) for dy in (-20,20)]

def box(x0,x1,y0,y1,z0,z1):
    return Manifold.cube((x1-x0,y1-y0,z1-z0)).translate((x0,y0,z0))

def cyl(x,y,r,z0,z1):
    return Manifold.cylinder(z1-z0,r,r,64).translate((x,y,z0))

def tm(m):
    a=m.to_mesh()
    return trimesh.Trimesh(vertices=np.asarray(a.vert_properties)[:,:3],faces=np.asarray(a.tri_verts),process=False)

def export(m,name):
    m=m.simplify(0.0001)
    assert str(m.status()).endswith('NoError'), (name,m.status())
    mesh=tm(m)
    assert mesh.is_watertight and mesh.is_winding_consistent and mesh.volume>0, name
    mesh.export(OUT/name)
    reread=trimesh.load_mesh(OUT/name)
    assert reread.is_watertight and reread.is_winding_consistent, (name,len(reread.vertices),len(reread.faces))
    assert len(m.decompose())==1, (name,'disconnected bodies')
    return {'file':name,'triangles':len(mesh.faces),'volume_mm3':float(mesh.volume),'watertight':True,'bodies':1,'bounds_mm':mesh.bounds.tolist()}

original=trimesh.load_mesh(SOURCE)
original.merge_vertices(digits_vertex=4)
original.update_faces(original.nondegenerate_faces())
original.remove_unreferenced_vertices()
assert original.is_watertight and original.is_winding_consistent
base=Manifold(Mesh(np.asarray(original.vertices,dtype=np.float32),np.asarray(original.faces,dtype=np.uint32)))
assert str(base.status()).endswith('NoError'),base.status()

# Flush front bezel extends the existing cutout inward to support bare glass.
seat=box(OX0,OX1,OY0,OY1,0,SEAT)-box(CX-AW/2,CX+AW/2,CY-AH/2,CY+AH/2,-1,SEAT+1)
# Lower the seat beneath the central driver/bonding region near the ribbon.
seat=seat-box(XL+3.5,XR-3.5,OY0-0.1,CY-AH/2+0.01,FACE,SEAT+1)
supports=seat
# Side guides receive the screen by a straight drop from the enclosure interior.
for x0,x1 in [(XL-CLEARANCE-1.6,XL-CLEARANCE),(XR+CLEARANCE,XR+CLEARANCE+1.6)]:
    supports=supports+box(x0,x1,YB-CLEARANCE,YT+CLEARANCE,SEAT-0.2,STOP)
supports=supports+box(XL-CLEARANCE-1.6,XR+CLEARANCE+1.6,YT+CLEARANCE,YT+CLEARANCE+1.6,SEAT-0.2,STOP)
# Split lower ledge: keep a wide open route for the FPC.
for x0,x1 in [(XL-CLEARANCE-1.6,XL+3.5),(XR-3.5,XR+CLEARANCE+1.6)]:
    supports=supports+box(x0,x1,YB-CLEARANCE-1.6,YB-CLEARANCE,FACE-0.2,STOP)
for x,y in HOLES:
    supports=supports+cyl(x,y,3.2,FACE-0.5,STOP)

lid=base+supports
for x,y in HOLES:
    # Blind pilot holes: 2.0 mm starting diameter for M2.5 plastic screws.
    lid=lid-cyl(x,y,1.0,FACE,STOP+1)

# U-shaped retainer, with an open cable end and a clear central back area.
rear=box(OX0,OX1,YB-0.3,OY1,STOP,STOP+RETAINER_T)
rear=rear-box(CX-AW/2,CX+AW/2,YB-1,CY+AH/2,STOP-1,STOP+RETAINER_T+1)
for x,y in HOLES:
    rear=rear+cyl(x,y,3.2,STOP,STOP+RETAINER_T)
for x,y in HOLES:
    rear=rear-cyl(x,y,1.4,STOP-1,STOP+RETAINER_T+1)

# Physical panel envelope excludes its flexible tail; includes maximum tolerance.
panel=box(XL,XR,YB,YT,PANEL_Z,PANEL_Z+PANEL_T)
maxpanel=box(XL-0.1,XR+0.1,YB-0.1,YT+0.1,PANEL_Z,PANEL_Z+1.1)
assert (maxpanel^lid).volume()<1e-6,'Panel collides with lid'
assert (maxpanel^rear).volume()<1e-6,'Panel collides with retainer'
assert (rear^lid).volume()<1e-6,'Retainer collides with lid'
assert (base-lid).volume()<1e-6,'Original lid material was removed'
# Aperture is clear through the front bezel. Panel is deliberately absent.
view=box(CX-AW/2+0.01,CX+AW/2-0.01,CY-AH/2+0.01,CY+AH/2-0.01,-1,PANEL_Z)
assert (view^lid).volume()<1e-6,'Blocked display aperture'

records=[export(lid,'LidTop_Adafruit6383_mount_PROTOTYPE.stl')]
# Retainer is translated only for separate printing, flat on Z=0.
rear_print=rear.translate((-OX0,-(YB-0.3),-STOP))
records.append(export(rear_print,'Adafruit6383_rear_retainer_PROTOTYPE.stl'))
# Small coupon verifies the actual pocket before printing a complete enclosure.
crop=box(OX0-4.8,OX1+4.8,OY0-1,OY1+1,-1,STOP+1)
crop=crop-box(99.50,OX1+5,60,90,FACE,STOP+2)  # omit neighboring sensor wall from coupon
records.append(export(lid^crop,'Adafruit6383_mount_fit_coupon.stl'))
tm(rear).export(ROOT/'rear_assembled.stl')
tm(panel).export(ROOT/'panel_reference_DO_NOT_PRINT.stl')
tm(lid-base).export(ROOT/'added_features.stl')

report={
 'source':str(SOURCE),'source_sha256':hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
 'product':'Adafruit 6383 / ZJY122250-0213BAAMFGN',
 'datasheet':'https://cdn-shop.adafruit.com/product-files/4197/C13256-007_datasheet_ZJY122250-0213BAAMFGN.pdf',
 'units':'mm (source STL interpreted in mm)',
 'panel_mm':[PANEL_W,PANEL_H,PANEL_T], 'pocket_mm':[PANEL_W+2*CLEARANCE,PANEL_H+2*CLEARANCE],
 'aperture_mm':[AW,AH], 'panel_center_xy':[PX,PY], 'ribbon_direction':'negative Y, bottom in inside-view preview',
 'seat_z':SEAT, 'panel_front_z':PANEL_Z,'stop_z':STOP,'retainer_thickness':RETAINER_T,
 'installed_front_pad_mm':FRONT_PAD,'installed_rear_pad_mm':REAR_PAD,
 'hole_centers_xy':HOLES,'pilot_diameter_mm':2.0,'retainer_hole_diameter_mm':2.8,
 'validation':{'max_panel_collision_mm3':(maxpanel^lid).volume(),'retainer_collision_mm3':(rear^lid).volume(),'removed_original_mm3':(base-lid).volume(),'physical_fit_tested':False},
 'mesh_preprocessing':'Merged source vertices at 0.0001 mm coordinate precision for connectivity; simplified output sliver triangles with 0.0001 mm tolerance.',
 'files':records
}
(OUT/'mount_dimensions_and_checks.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
