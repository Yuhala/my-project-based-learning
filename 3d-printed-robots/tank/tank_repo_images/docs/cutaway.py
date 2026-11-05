import sys, math, vtk, cadquery as cq
src=open('render.py').read().split("out=sys.argv[1]")[0]; exec(src)
import tank_cad as T
from cadquery import Vector as V, Location as Lc
parts,qty,asm,placed,_=T.build(pan=20)
yc=T.TRACK_YC-T.HY
yflange=yc+T.WHEEL_W/2+0.2+1.6                      # sleeve flange outer face (local)
def csk_screw(L=25):
    head=cq.Solid.makeCone(3.0,1.5,1.7,V(0,0,0),V(0,-1,0)).translate(V(0,yflange,0))
    return head.fuse(T.cy(1.5,0,0,yflange-L,yflange-1.6))
def washer(): return T.cy(3.5,0,0,-T.WALL-0.5,-T.WALL).cut(T.cy(1.6,0,0,-5,5))
def nut():
    return (cq.Workplane("XZ").polygon(6,5.5/math.cos(math.radians(30))).extrude(2.4).val()
            .translate(V(0,-T.WALL-0.5,0))).cut(T.cy(1.5,0,0,-10,10))
GREY=(0.55,0.55,0.6); STEEL=(0.75,0.75,0.8)
extra=[]
for s in (1,-1):
    loc=(lambda x,z: Lc(V(x,T.HY,z))) if s>0 else (lambda x,z: Lc(V(x,-T.HY,z))*Lc(V(0,0,0),V(0,0,1),180))
    for x,z in [(x,T.Z_ROAD) for x in T.X_ROAD]+[(T.X_IDLER,T.Z_SPROCKET)]:
        extra+= [(csk_screw().moved(loc(x,z)),STEEL),(washer().moved(loc(x,z)),STEEL),(nut().moved(loc(x,z)),STEEL)]
    extra.append((T.cy(1.0,0,0,1.2,6).moved(loc(T.X_SPROCKET,T.Z_SPROCKET)).moved(Lc(V(0,0,0))),STEEL))
def lift(c):
    c=c.toTuple()[:3]; return tuple(min(1,0.18+v*1.1) for v in c) if max(c)<0.3 else c
items=[(n,s,lift(c)) for n,s,c,p in placed if n not in('deck',)]+[('fast',s,c) for s,c in extra]

# ---------- 1) open tank: deck removed, iso view ----------
acts=[actor_from(s,c) for n,s,c in items]
render(acts, "out/preview/open_tank_iso.png", -35, 38, zoom=1.5)

# ---------- 2) section through the road-wheel axles (cut at X=0) ----------
keep=T.box(-200,0,-200,200,-50,200)
sec=[]
for n,s,c in items:
    bb=s.BoundingBox()
    if bb.xmin>0: continue
    if bb.xmax>0:
        try: s=s.intersect(keep)
        except Exception: continue
    sec.append(actor_from(s,c,tol=0.05))
r=vtk.vtkRenderer(); r.SetBackground(0.96,0.96,0.97)
for a in sec: r.AddActor(a)
rw=vtk.vtkRenderWindow(); rw.SetOffScreenRendering(1); rw.SetSize(1300,900); rw.AddRenderer(r); rw.SetMultiSamples(8)
cam=r.GetActiveCamera(); cam.SetFocalPoint(0,30,22); cam.SetPosition(420,95,120); cam.SetViewUp(0,0,1)
cam.SetViewAngle(18); r.ResetCameraClippingRange()
l=vtk.vtkLight(); l.SetLightTypeToCameraLight(); l.SetIntensity(0.5); r.AddLight(l)
rw.Render(); w=vtk.vtkWindowToImageFilter(); w.SetInput(rw); w.Update()
pw=vtk.vtkPNGWriter(); pw.SetFileName("out/preview/section_wheel_axle.png"); pw.SetInputConnection(w.GetOutputPort()); pw.Write()

# ---------- 3) exploded wheel stack (one road wheel) with labels ----------
stack=[("M3x25 countersunk screw",csk_screw(),STEEL,26),
       ("flanged sleeve (wheel spins on it)",parts["axle_sleeve"],(0.95,0.6,0.15),18),
       ("road wheel",parts["road_wheel"],(0.35,0.35,0.38),9),
       ("spacer",parts["axle_spacer"],(0.95,0.6,0.15),0),
       ("hull wall",T.box(-10,10,-T.WALL,0,-10,10),(0.45,0.5,0.3),-9),
       ("washer",washer(),STEEL,-16),
       ("nut",nut(),STEEL,-23)]
r=vtk.vtkRenderer(); r.SetBackground(1,1,1)
for i,(name,s,c,dy) in enumerate(stack):
    a=actor_from(s.translate(V(0,dy,0)),c,tol=0.03); r.AddActor(a)
    t=vtk.vtkBillboardTextActor3D(); t.SetInput(name)
    bb=s.BoundingBox(); t.SetPosition(0,(bb.ymin+bb.ymax)/2+dy,15+ (i%2)*6)
    t.GetTextProperty().SetFontSize(17); t.GetTextProperty().SetColor(0.1,0.1,0.1); t.GetTextProperty().SetJustificationToCentered()
    r.AddActor(t)
ln=vtk.vtkLineSource(); ln.SetPoint1(0,-40,0); ln.SetPoint2(0,55,0)
lm=vtk.vtkPolyDataMapper(); lm.SetInputConnection(ln.GetOutputPort()); la=vtk.vtkActor(); la.SetMapper(lm)
la.GetProperty().SetColor(0.6,0.2,0.2); la.GetProperty().SetLineStipplePattern(0xF0F0); r.AddActor(la)
rw=vtk.vtkRenderWindow(); rw.SetOffScreenRendering(1); rw.SetSize(1500,700); rw.AddRenderer(r); rw.SetMultiSamples(8)
cam=r.GetActiveCamera(); cam.SetFocalPoint(0,8,4); cam.SetPosition(170,8,95); cam.SetViewUp(0,0,1); cam.SetViewAngle(30)
r.ResetCameraClippingRange(); l=vtk.vtkLight(); l.SetLightTypeToCameraLight(); l.SetIntensity(0.5); r.AddLight(l)
rw.Render(); w=vtk.vtkWindowToImageFilter(); w.SetInput(rw); w.Update()
pw=vtk.vtkPNGWriter(); pw.SetFileName("out/preview/exploded_wheel_stack.png"); pw.SetInputConnection(w.GetOutputPort()); pw.Write()
print("ok")
