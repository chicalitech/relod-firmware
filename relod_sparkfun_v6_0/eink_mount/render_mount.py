from pathlib import Path
import json
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'output'
cfg=json.loads((OUT/'mount_dimensions_and_checks.json').read_text())
FONT='C:/Windows/Fonts/segoeui.ttf'
BOLD='C:/Windows/Fonts/segoeuib.ttf'
def font(n,bold=False): return ImageFont.truetype(BOLD if bold else FONT,n)
def read(path):
    return np.frombuffer(Path(path).read_bytes(),offset=84,dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')]))['v'].astype(float).copy()

lid=read(OUT/'LidTop_Adafruit6383_mount_PROTOTYPE.stl')
added=read(ROOT/'added_features.stl')
rear=read(ROOT/'rear_assembled.stl')
panel=read(ROOT/'panel_reference_DO_NOT_PRINT.stl')

def render(parts,size,az=-70,el=60):
    az,el=np.deg2rad([az,el])
    right=np.array([-np.sin(az),np.cos(az),0])
    up=np.array([-np.cos(az)*np.sin(el),-np.sin(az)*np.sin(el),np.cos(el)])
    view=np.cross(right,up)
    verts=np.concatenate([p[0] for p in parts])
    projected=np.stack((verts@right,-verts@up),axis=-1)
    lo=projected.min((0,1)); hi=projected.max((0,1))
    scale=min((size[0]-36)/(hi[0]-lo[0]),(size[1]-36)/(hi[1]-lo[1]))
    offset=(np.array(size)-(hi-lo)*scale)/2
    im=Image.new('RGB',size,'#f4f7fa'); d=ImageDraw.Draw(im)
    faces=[]
    light=np.array([-0.3,-0.5,1]);light/=np.linalg.norm(light)
    for tris,color,bias in parts:
        for tri in tris:
            n=np.cross(tri[1]-tri[0],tri[2]-tri[0]);norm=np.linalg.norm(n)
            if norm<1e-10:continue
            n/=norm
            if np.dot(n,view)<-0.00001:continue
            xy=(np.stack((tri@right,-tri@up),axis=-1)-lo)*scale+offset
            shade=0.70+0.30*max(0,np.dot(n,light))
            c=tuple(int(a*shade) for a in color)
            faces.append((float((tri@view).mean())+bias,xy,c))
    # Rasterized depth avoids incorrect occlusion from long, flat lid triangles.
    pixels=np.full((size[1],size[0],3),(244,247,250),dtype=np.uint8)
    depth=np.full((size[1],size[0]),-np.inf)
    for tris,color,bias in parts:
        for tri in tris:
            n=np.cross(tri[1]-tri[0],tri[2]-tri[0]); norm=np.linalg.norm(n)
            if norm<1e-10:continue
            n/=norm
            if np.dot(n,view)<-0.00001:continue
            xy=(np.stack((tri@right,-tri@up),axis=-1)-lo)*scale+offset
            z=tri@view+bias
            x0=max(0,int(np.floor(xy[:,0].min())));x1=min(size[0]-1,int(np.ceil(xy[:,0].max())))
            y0=max(0,int(np.floor(xy[:,1].min())));y1=min(size[1]-1,int(np.ceil(xy[:,1].max())))
            if x1<x0 or y1<y0:continue
            a,b,c=xy
            den=(b[1]-c[1])*(a[0]-c[0])+(c[0]-b[0])*(a[1]-c[1])
            if abs(den)<1e-10:continue
            xx,yy=np.meshgrid(np.arange(x0,x1+1)+0.5,np.arange(y0,y1+1)+0.5)
            wa=((b[1]-c[1])*(xx-c[0])+(c[0]-b[0])*(yy-c[1]))/den
            wb=((c[1]-a[1])*(xx-c[0])+(a[0]-c[0])*(yy-c[1]))/den
            wc=1-wa-wb
            zz=wa*z[0]+wb*z[1]+wc*z[2]
            target_depth=depth[y0:y1+1,x0:x1+1]
            mask=(wa>=-1e-6)&(wb>=-1e-6)&(wc>=-1e-6)&(zz>=target_depth)
            shade=0.70+0.30*max(0,np.dot(n,light))
            pixels[y0:y1+1,x0:x1+1][mask]=tuple(int(a*shade) for a in color)
            target_depth[mask]=zz[mask]
    im=Image.fromarray(pixels)
    d=ImageDraw.Draw(im)
    if el>1.56:
        # Crease outlines make the stepped seat readable in the orthographic view.
        for tris,color,_ in parts:
            edges={}
            for tri in tris:
                n=np.cross(tri[1]-tri[0],tri[2]-tri[0]); norm=np.linalg.norm(n)
                if norm<1e-10:continue
                n/=norm
                for j in range(3):
                    key=tuple(sorted((tuple(np.round(tri[j],4)),tuple(np.round(tri[(j+1)%3],4)))))
                    edges.setdefault(key,[]).append(n)
            for edge,ns in edges.items():
                if not any(np.dot(n,view)>0.01 for n in ns):continue
                if len(ns)==2 and np.dot(ns[0],ns[1])>0.995:continue
                ep=np.array(edge)
                xy=(np.stack((ep@right,-ep@up),axis=-1)-lo)*scale+offset
                d.line([tuple(p) for p in xy],fill=tuple(int(c*0.55) for c in color),width=1)
    return im

canvas=Image.new('RGB',(1800,1120),'white');d=ImageDraw.Draw(canvas)
d.text((60,32),'E-ink mount for your lid',font=font(38,True),fill='#142b3e')
d.text((60,88),'Adafruit 6383 | Prototype for fit testing | Dimensions in mm',font=font(22),fill='#506576')
left=render([(lid,(193,203,212),0),(added,(30,165,163),0.002)],(830,760),az=-90,el=90)
rp=rear.copy();rp[:,:,2]+=18
pp=panel.copy();pp[:,:,2]+=9
right=render([(lid,(193,203,212),0),(added,(30,165,163),0.002),(pp,(230,181,77),0),(rp,(49,107,171),0)],(830,760),az=-65,el=52)
canvas.paste(left,(60,192));canvas.paste(right,(910,192))
d.text((60,148),'INSIDE VIEW',font=font(23,True),fill='#142b3e')
d.text((910,148),'EXPLODED ASSEMBLY',font=font(23,True),fill='#142b3e')
legend=[('#1ea5a3','Added seat, guides and four screw bosses'),('#e6b54d','Glass panel reference - not a printed part'),('#316bab','Separate rear retainer with open cable end')]
for i,(color,label) in enumerate(legend):
    y=976+i*36
    d.rounded_rectangle((62,y+3,82,y+23),radius=3,fill=color)
    d.text((96,y-3),label,font=font(21),fill='#233e50')
d.text((1060,976),'Pocket: 29.8 x 59.8',font=font(22),fill='#233e50')
d.text((1060,1012),'Viewing opening: 25.3 x 50.15',font=font(22),fill='#233e50')
d.text((1060,1048),'Ribbon exits toward the bottom.',font=font(22),fill='#233e50')
canvas.save(OUT/'mount_preview.png')

# Full-size close-up is useful for reviewing the retaining geometry.
coupon=read(OUT/'Adafruit6383_mount_fit_coupon.stl')
render([(coupon,(30,165,163),0)],(1200,1000),az=-65,el=55).save(OUT/'fit_coupon_preview.png')
