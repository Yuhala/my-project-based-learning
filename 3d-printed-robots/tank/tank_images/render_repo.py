import sys, math, vtk, cadquery as cq
src=open('render.py').read().split("out=sys.argv[1]")[0]; exec(src)
import tank_cad as T
from cadquery import Vector as V, Location as Lc
OUT=sys.argv[1]; import os; os.makedirs(OUT,exist_ok=True)
parts,qty,asm,placed,_=T.build(pan=20)
yc=T.TRACK_YC-T.HY; yfl=yc+T.WHEEL_W/2+0.2+1.6
def csk(): return cq.Solid.makeCone(3.0,1.5,1.7,V(0,yfl,0),V(0,-1,0)).fuse(T.cy(1.5,0,0,yfl-25,yfl-1.6))
def nut(): return (cq.Workplane("XZ").polygon(6,5.5/math.cos(math.radians(30))).extrude(2.4).val().translate(V(0,-T.WALL-0.5,0))).cut(T.cy(1.5,0,0,-10,10))
STEEL=cq.Color(0.75,0.75,0.8)
for s in (1,-1):
    loc=(lambda x,z: Lc(V(x,T.HY,z))) if s>0 else (lambda x,z: Lc(V(x,-T.HY,z))*Lc(V(0,0,0),V(0,0,1),180))
    side='L' if s>0 else 'R'
    for i,(x,z) in enumerate([(x,T.Z_ROAD) for x in T.X_ROAD]+[(T.X_IDLER,T.Z_SPROCKET)]):
        placed.append((f"screw_{side}{i}",csk().moved(loc(x,z)),STEEL,False))
        placed.append((f"nut_{side}{i}",nut().moved(loc(x,z)),STEEL,False))
def lift(c):
    c=c.toTuple()[:3]; return tuple(min(1,0.18+v*1.1) for v in c) if max(c)<0.3 else c
TOP=('deck','SG90_pan','camera_head','camera_board')
def actors(skip=(), xform=None, ghost=()):
    acts=[]
    for n,s,c,p in placed:
        if n in skip or any(n.startswith(k) for k in skip): continue
        if xform: s=xform(n,s)
        a=actor_from(s,lift(c),tol=0.08)
        if any(n.startswith(g) for g in ghost): a.GetProperty().SetOpacity(0.25)
        acts.append(a)
    return acts
def gnd(z=-0.3,ext=150):
    g=vtk.vtkPlaneSource(); g.SetOrigin(-ext,-ext*0.85,z); g.SetPoint1(ext,-ext*0.85,z); g.SetPoint2(-ext,ext*0.85,z)
    m=vtk.vtkPolyDataMapper(); m.SetInputConnection(g.GetOutputPort()); a=vtk.vtkActor(); a.SetMapper(m)
    a.GetProperty().SetColor(0.93,0.93,0.94); a.GetProperty().SetAmbient(0.6); return a
SZ=(2400,1800)
def render_top(acts, fn):
    r=vtk.vtkRenderer(); r.SetBackground(0.96,0.96,0.97)
    for a in acts: r.AddActor(a)
    rw=vtk.vtkRenderWindow(); rw.SetOffScreenRendering(1); rw.SetSize(*SZ); rw.AddRenderer(r); rw.SetMultiSamples(8)
    cam=r.GetActiveCamera(); cam.SetFocalPoint(0,0,30); cam.SetPosition(0,0,800); cam.SetViewUp(0,1,0)
    r.ResetCamera(); cam.Zoom(1.3); r.ResetCameraClippingRange()
    l=vtk.vtkLight(); l.SetLightTypeToCameraLight(); l.SetIntensity(0.55); r.AddLight(l)
    rw.Render(); w=vtk.vtkWindowToImageFilter(); w.SetInput(rw); w.Update()
    pw=vtk.vtkPNGWriter(); pw.SetFileName(fn); pw.SetInputConnection(w.GetOutputPort()); pw.Write()
closed=actors()
render(closed, f"{OUT}/01_closed_front_left.png", -38, 24, size=SZ, zoom=1.2)
render(closed, f"{OUT}/02_closed_rear_right.png", 142, 26, size=SZ, zoom=1.2)
render(closed, f"{OUT}/03_closed_side.png", -90, 0, size=SZ, zoom=1.15)
render_top(closed, f"{OUT}/04_closed_top.png")
render(closed, f"{OUT}/05_closed_front.png", 0, 4, size=SZ, zoom=1.2)
openA=actors(skip=TOP, ghost=('battery',))
render(openA, f"{OUT}/06_open_rear_view.png", -150, 48, size=SZ, zoom=0.95)
render(openA, f"{OUT}/07_open_front_view.png", 30, 50, size=SZ, zoom=0.95)
render_top(openA, f"{OUT}/08_open_top.png")
def lifted(n,s):
    return s.moved(Lc(V(0,0,45))) if n in TOP else s
render(actors(xform=lifted, ghost=('battery',)), f"{OUT}/09_deck_lifted.png", -40, 28, size=SZ, zoom=1.05)
def exploded(n,s):
    side=1 if ('_L' in n) else (-1 if '_R' in n else 0)
    if n in TOP: return s.moved(Lc(V(0,0,70)))
    if n.startswith('motor_cap'): return s.moved(Lc(V(0,0,28)))
    if n.startswith('link'): return s.moved(Lc(V(0,side*70,0)))
    if n.startswith(('road_wheel','idler','sprocket')): return s.moved(Lc(V(0,side*42,0)))
    if n.startswith(('sleeve','screw')): return s.moved(Lc(V(0,side*56,0)))
    if n.startswith(('spacer')): return s.moved(Lc(V(0,side*28,0)))
    if n.startswith('nut'): return s.moved(Lc(V(0,-side*16,0)))
    if n.startswith('battery'): return s.moved(Lc(V(0,0,30)))
    return s
render(actors(xform=exploded), f"{OUT}/10_exploded.png", -35, 30, size=SZ, zoom=0.9)
print('ok')
