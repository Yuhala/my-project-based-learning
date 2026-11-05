"""
Mini FPV Racing Drone - 3-inch, 3D-printed frame (parametric)
=============================================================
Exports STL (print-oriented) + STEP (for FreeCAD) for every printed part
and the full assembly.   pip install cadquery ; python fpv3_cad.py
Units mm.  +X forward, +Z up, Z=0 = underside of bottom plate.
"""
import math, os
import cadquery as cq
from cadquery import Vector as V, Location as L

OUT = os.environ.get("FPV_OUT", "./fpv3_out")

# ============================== PARAMETERS ================================
WHEELBASE   = 150.0                 # motor-to-motor diagonal
MOTOR_XY    = WHEELBASE / 2 / math.sqrt(2)      # 53.0
PROP_D      = 76.2                  # 3-inch props
STACK       = 20.0                  # FC/ESC stack hole spacing (20x20 M2)

MOTOR_PATS  = (9.0, 12.0)           # M2 motor patterns (1404 / 1408 class)
STANDOFF_XY = 20.0                  # M3 standoffs at (+-20, +-20)
ARM_HOLE2   = 28.0                  # second arm-lock screw at (+-28, +-28)
STANDOFF_H  = 27.0                  # printed spacer: M3x40 bolt = 4+5+27+3 + lock washer + nut trap
SPACER_D    = 6.5
STACK_BOSS  = 3.5                   # M2 heat-set insert bosses under the ESC
STACK_SPC   = 5.5                   # printed M2 spacer between ESC and FC carrier
CARRIER_T   = 2.5
M2_INSERT   = 3.2                   # hole for M2 heat-set insert (check your kit)
NUT_AF      = 5.7                   # M3 nut trap across flats (5.5 nut + 0.2)
CSK_D       = 6.4                   # M3 countersunk head diameter (90 deg)
GUARD_T     = 2.5                   # motor guard floor: M2x10 -> 2.5 mm into motor
BOT_T, ARM_T, TOP_T = 4.0, 5.0, 3.0
CAM_W       = 19.0                  # micro FPV camera
CAM_PIVOT   = (30.0, 18.0)          # camera pivot (x, z)
M3, M2      = 3.3, 2.2

Z_ARM  = BOT_T
Z_SO   = BOT_T + ARM_T
Z_TOP  = Z_SO + STANDOFF_H
Z_ESC  = BOT_T + STACK_BOSS
Z_CARRIER = Z_ESC + 1.6 + STACK_SPC

# =============================== HELPERS ==================================
def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))
def cz(r, x, y, z0, z1):
    return cq.Solid.makeCylinder(r, z1 - z0, V(x, y, z0), V(0, 0, 1))
def cyl(r, p, d, h):
    return cq.Solid.makeCylinder(r, h, V(*p), V(*d))
def fuse(*s):
    o = s[0]
    for x in s[1:]: o = o.fuse(x)
    return o.clean()
def cut(b, *t): return b.cut(*t).clean()
def fil_z(s, r): return cq.Workplane().add(s).edges("|Z").fillet(r).val()
def slot_z(x0, y0, x1, y1, r, z0, z1):
    """stadium between two points, extruded in Z"""
    L_ = math.hypot(x1 - x0, y1 - y0); a = math.degrees(math.atan2(y1 - y0, x1 - x0))
    w = (cq.Workplane("XY").slot2D(L_ + 2 * r, 2 * r, 0).extrude(z1 - z0).val()
         .rotate(V(0, 0, 0), V(0, 0, 1), a)
         .translate(V((x0 + x1) / 2, (y0 + y1) / 2, z0)))
    return w
def hex_z(af, x, y, z0, z1):
    return (cq.Workplane("XY").polygon(6, af / math.cos(math.radians(30)))
            .extrude(z1 - z0).val().translate(V(x, y, z0)))
def csk_below(x, y):
    """countersink for an M3 flat head entering from the underside (Z=0)"""
    return cq.Solid.makeCone(CSK_D / 2, 1.6, CSK_D / 2 - 1.6, V(x, y, -0.01), V(0, 0, 1))
def corners(d):
    return [(sx * d, sy * d) for sx in (-1, 1) for sy in (-1, 1)]

# ================================ PARTS ===================================
def bottom_plate():
    body = fil_z(box(-38, 38, -24, 24, 0, BOT_T), 5)
    pads = [slot_z(sx * STANDOFF_XY, sy * STANDOFF_XY, sx * ARM_HOLE2, sy * ARM_HOLE2,
                   6.5, 0, BOT_T) for sx, sy in corners(1)]
    bosses = [cz(3.4, x, y, BOT_T, BOT_T + STACK_BOSS) for x, y in corners(STACK / 2)]
    p = fuse(body, *pads, *bosses)
    t = []
    for x, y in corners(STANDOFF_XY):                  # M3x40 through-bolts, nut trap below
        t += [cz(M3 / 2, x, y, -1, 9), hex_z(NUT_AF, x, y, -1, 2.6)]
    for x, y in corners(ARM_HOLE2):                    # arm lock: countersunk M3x12 from below
        t += [cz(M3 / 2, x, y, -1, 9), csk_below(x, y)]
    for s_ in (-1, 1):                                 # camera mount: countersunk M3x10
        x, y = CAM_PIVOT[0] + 1, s_ * 13.2
        t += [cz(M3 / 2, x, y, -1, 9), csk_below(x, y)]
    for x, y in corners(STACK / 2):                    # M2 heat-set inserts
        t += [cz(M2_INSERT / 2, x, y, BOT_T + STACK_BOSS - 4.0, BOT_T + STACK_BOSS + 1),
              cz(1.0, x, y, -1, 9)]
    t.append(slot_z(-31, -8, -31, 8, 3, -1, 9))        # rear wire/strap slot
    t.append(slot_z(0, -4, 0, 4, 3.5, -1, 9))          # centre wire hole
    for s_ in (-1, 1):
        t.append(slot_z(-9, s_ * 18, 9, s_ * 18, 2.5, -1, 9))
    return cut(p, *t)


def arm():
    """Local: inner (standoff) hole at origin, arm along +X, Z=0 = underside."""
    Lm = (MOTOR_XY - STANDOFF_XY) * math.sqrt(2)                   # 46.7
    h2 = (ARM_HOLE2 - STANDOFF_XY) * math.sqrt(2)                  # 11.3
    root = slot_z(0, 0, h2, 0, 6.0, 0, ARM_T)
    beam = (cq.Workplane("XY").polyline([(h2, -6), (Lm - 6, -4.5), (Lm - 6, 4.5), (h2, 6)])
            .close().extrude(ARM_T).val())
    pad = cz(11.0, Lm, 0, 0, ARM_T)
    p = fuse(root, beam, pad)
    t = [cz(M3 / 2, 0, 0, -1, 9), cz(M3 / 2, h2, 0, -1, 9), cz(2.8, Lm, 0, -1, 9)]
    for pat in MOTOR_PATS:
        r = pat / math.sqrt(2)
        for k in range(4):
            a = math.pi / 4 + k * math.pi / 2
            t.append(cz(M2 / 2, Lm + r * math.cos(a), r * math.sin(a), -1, 9))
    t.append(slot_z(h2 + 9, 0, Lm - 16, 0, 1.6, -1, 9))             # lightening / wire tie
    t.append(box(Lm - 13, Lm - 10.5, -6, 6, ARM_T - 1.0, ARM_T + 1))  # wire groove (top)
    return cut(p, *t)


def top_plate():
    p = fil_z(box(-26, 38, -24, 24, 0, TOP_T), 5)
    t = [cz(M3 / 2, x, y, -1, 5) for x, y in corners(STANDOFF_XY)]
    for x in (-12, 14):                                            # battery strap slots
        t.append(box(x - 1.5, x + 1.5, -11, 11, -1, 5))
    t += [slot_z(-20, -6, -20, 6, 2.6, -1, 5), slot_z(-4, 0, 7, 0, 3.0, -1, 5),
          slot_z(27, -6, 27, 6, 2.6, -1, 5)]                       # lightening
    p = cut(p, *t)
    ribs = []                                                      # battery grip ribs
    for y in (-16, -8, 8, 16):
        for x0, x1 in ((-24, -14), (-10, 12), (16, 34)):
            ribs.append(box(x0, x1, y - 0.5, y + 0.5, TOP_T, TOP_T + 0.6))
    return fuse(p, *ribs)


def camera_mount():
    """Pivoting cradle for a 19 mm micro FPV camera.  Z = 0 on top of bottom plate."""
    cx, cz_ = CAM_PIVOT; zc = cz_ - BOT_T
    yi = CAM_W / 2 + 0.2
    parts = []
    for s in (-1, 1):
        y0, y1 = sorted((s * yi, s * (yi + 2.2)))
        cheek = fuse(box(cx - 7, cx + 7, y0, y1, 0, zc),
                     cyl(7, (cx, y0, zc), (0, 1, 0), y1 - y0))
        fy0, fy1 = sorted((s * yi, s * 16.5))
        fy0, fy1 = sorted((s * yi, s * 16.0))
        foot = fil_z(box(cx - 5, cx + 7, fy0, fy1, 0, 3), 1.5)
        parts += [cheek, foot]
    bridge = box(cx + 4, cx + 7, -yi - 2.2, yi + 2.2, 0, 3)            # front tie-bar
    p = fuse(*parts, bridge)
    t = [cyl(1.1, (cx, -20, zc), (0, 1, 0), 40)]
    t += [cz(M3 / 2, cx + 1, s * 13.2, -1, 5) for s in (-1, 1)]
    return cut(p, *t)


def motor_guard():
    """TPU foot under each motor: protects screws, acts as landing gear."""
    p = cz(11.5, 0, 0, 0, GUARD_T).fuse(cz(11.5, 0, 0, -4, 0).cut(cz(9.8, 0, 0, -6, 1)))
    t = [cz(2.8, 0, 0, -6, 3)]
    for pat in MOTOR_PATS:
        r = pat / math.sqrt(2)
        for k in range(4):
            a = math.pi / 4 + k * math.pi / 2
            t.append(cz(M2 / 2, r * math.cos(a), r * math.sin(a), -6, 3))
    t.append(box(-12, -8, -3, 3, -6, 0))
    return cut(p, *t)


def antenna_mount():
    """TPU clip over the two rear standoffs, holds the VTX antenna tube at 30 deg."""
    sx = -STANDOFF_XY
    ri = SPACER_D / 2 - 0.05
    rings = [cz(ri + 1.8, sx, s * STANDOFF_XY, 0, 7).cut(cz(ri, sx, s * STANDOFF_XY, -1, 8))
             for s in (-1, 1)]
    bar = box(sx - 2, sx + 2, -STANDOFF_XY + 4.5, STANDOFF_XY - 4.5, 0, 7)
    tail = box(sx - 12, sx - 1, -4, 4, 0, 7)
    ang = math.radians(30)
    d = (-math.sin(ang), 0, math.cos(ang))
    base = (sx - 9, 0, 3.5)
    tube = cyl(3.6, base, d, 20).cut(cyl(2.1, (base[0] - d[0] * 2, 0, base[2] - d[2] * 2), d, 30))
    p = fuse(*rings, bar, tail, tube)
    p = cut(p, cz(ri, sx, STANDOFF_XY, -1, 8), cz(ri, sx, -STANDOFF_XY, -1, 8),
            box(sx - 30, sx + 30, -30, 30, -10, 0),
            cyl(2.1, (base[0] - d[0] * 2, 0, base[2] - d[2] * 2), d, 30))
    return p


def spacer():
    """Printed standoff (replaces M3x25 alu).  Clamped by the M3x40 through-bolt."""
    return cz(SPACER_D / 2, 0, 0, 0, STANDOFF_H).cut(cz(M3 / 2, 0, 0, -1, STANDOFF_H + 1))


def stack_spacer():
    """M2 spacer between the 4-in-1 ESC and the FC carrier."""
    return cz(2.4, 0, 0, 0, STACK_SPC).cut(cz(1.2, 0, 0, -1, STACK_SPC + 1))


ESP_BOARD = (22.9, 18.4)          # ESP32-S3 SuperMini 22.5 x 18 (+0.4); XIAO S3 also fits
IMU_BOARD = (21.4, 16.4)          # GY-521 / GY-91 class gyro board


def fc_carrier():
    """Sits on the 20x20 stack above the ESC.  ESP32-S3 at the rear (USB-C facing
    back), gyro board in the centre on foam tape.  Z=0 = carrier underside."""
    ex0 = -36.5; ex1 = ex0 + ESP_BOARD[0]; ey = ESP_BOARD[1] / 2
    p = fil_z(box(-38.5, 16, -16, 16, 0, CARRIER_T), 3)
    w = 1.2; h = CARRIER_T + 2.5
    walls = [box(ex0 - w, ex1 + w, ey, ey + w, CARRIER_T, h),
             box(ex0 - w, ex1 + w, -ey - w, -ey, CARRIER_T, h),
             box(ex1, ex1 + w, -ey, ey, CARRIER_T, h),
             box(ex0 - w, ex0, -ey, ey, CARRIER_T, h)]
    ix, iy = IMU_BOARD[0] / 2, IMU_BOARD[1] / 2
    fences = [box(-6, 6, iy, iy + 1, CARRIER_T, CARRIER_T + 1.5),
              box(-6, 6, -iy - 1, -iy, CARRIER_T, CARRIER_T + 1.5),
              box(ix, ix + 1, -5, 5, CARRIER_T, CARRIER_T + 1.5),
              box(-ix - 1, -ix, -5, 5, CARRIER_T, CARRIER_T + 1.5)]
    p = fuse(p, *walls, *fences)
    t = [cz(1.2, x, y, -1, 5) for x, y in corners(STACK / 2)]
    t.append(box(ex0 - 2, ex0 + 0.1, -5, 5, CARRIER_T, h + 1))          # USB-C opening
    for x in (ex0 + 5, ex1 - 5):                                          # strap / zip-tie slots
        for s_ in (-1, 1):
            t.append(box(x - 1.5, x + 1.5, s_ * (ey + w + 0.6) - 1.5, s_ * (ey + w + 0.6) + 1.5, -1, h + 1))
    for s_ in (-1, 1):                                                    # wire pass-throughs
        t.append(slot_z(-6, s_ * 12.5, 6, s_ * 12.5, 1.8, -1, 5))
    t.append(slot_z(ex0 + 6, 0, ex1 - 6, 0, 2.5, -1, 5))                 # under ESP (airflow/wires)
    return cut(p, *t)


# ========================= reference (bought) parts =========================
def d_motor():
    return fuse(cz(9.0, 0, 0, 0, 3), cz(9.2, 0, 0, 3, 13), cz(2.5, 0, 0, 13, 20))
def d_prop():
    blades = []
    for a in (0, 120, 240):
        b = (cq.Workplane("XY").ellipse(PROP_D / 4, 4.2).extrude(1.2).val()
             .translate(V(PROP_D / 4, 0, 0)).rotate(V(0, 0, 0), V(0, 0, 1), a))
        blades.append(b)
    return fuse(cz(4, 0, 0, -2, 3), *blades)
def d_board(w, h=1.6): return box(-w / 2, w / 2, -w / 2, w / 2, 0, h)
def d_camera():
    w = CAM_W / 2
    return fuse(box(-w, w, -w, w, -w, w), cyl(7, (w, 0, 0), (1, 0, 0), 7))
def d_battery(): return fil_z(box(-28.5, 28.5, -15, 15, 0, 21), 2)
def d_bolt(L_, d=3.0, head=5.5, hh=3.0):
    return fuse(cz(d / 2, 0, 0, -L_, 0), cz(head / 2, 0, 0, 0, hh))


# ================================ ASSEMBLY =================================
def build(cam_tilt=25):
    parts = dict(bottom_plate=bottom_plate(), arm=arm(), top_plate=top_plate(),
                 spacer_M3=spacer(), fc_carrier_ESP32S3=fc_carrier(),
                 stack_spacer_M2=stack_spacer(),
                 camera_mount=camera_mount(), motor_guard_TPU=motor_guard(),
                 antenna_mount_TPU=antenna_mount())
    qty = dict(arm=4, motor_guard_TPU=4, spacer_M3=4, stack_spacer_M2=4)
    C = cq.Color
    frame, blue, tpu = C(0.15, 0.15, 0.17), C(0.1, 0.45, 0.95), C(0.1, 0.4, 0.9)
    asm = cq.Assembly(name="FPV3"); placed = []
    def put(n, s, loc, col, printed=True):
        asm.add(s, name=n, loc=loc, color=col); placed.append((n, s.moved(loc), col, printed))

    put("bottom_plate", parts["bottom_plate"], L(), frame)
    Lm = (MOTOR_XY - STANDOFF_XY) * math.sqrt(2)
    for sx, sy in corners(1):
        ang = math.degrees(math.atan2(sy, sx))
        tag = ("F" if sx > 0 else "R") + ("R" if sy < 0 else "L")
        aloc = L(V(sx * STANDOFF_XY, sy * STANDOFF_XY, Z_ARM), V(0, 0, 1), ang)
        put(f"arm_{tag}", parts["arm"], aloc, frame)
        mloc = aloc * L(V(Lm, 0, ARM_T))
        put(f"motor_guard_{tag}", parts["motor_guard_TPU"], aloc * L(V(Lm, 0, -GUARD_T)), tpu)
        put(f"motor_{tag}", d_motor(), mloc, C(0.1, 0.35, 0.8), False)
        put(f"prop_{tag}", d_prop(), mloc * L(V(0, 0, 15)) * L(V(0, 0, 0), V(0, 0, 1), 30 * sx * sy),
            C(0.25, 0.6, 1.0), False)
        put(f"spacer_{tag}", parts["spacer_M3"], L(V(sx * STANDOFF_XY, sy * STANDOFF_XY, Z_SO)), frame)
        put(f"M3x40_{tag}", d_bolt(40), L(V(sx * STANDOFF_XY, sy * STANDOFF_XY, Z_TOP + TOP_T + 0.8)),
            C(0.05, 0.05, 0.05), False)
        put(f"stack_spacer_{tag}", parts["stack_spacer_M2"],
            L(V(sx * STACK / 2, sy * STACK / 2, Z_ESC + 1.6)), frame)
    put("top_plate", parts["top_plate"], L(V(0, 0, Z_TOP)), frame)
    put("camera_mount", parts["camera_mount"], L(V(0, 0, BOT_T)), blue)
    put("FPV_camera", d_camera(), L(V(CAM_PIVOT[0], 0, CAM_PIVOT[1])) *
        L(V(0, 0, 0), V(0, 1, 0), -cam_tilt), C(0.08, 0.08, 0.08), False)
    put("antenna_mount", parts["antenna_mount_TPU"], L(V(0, 0, Z_SO + 16)), tpu)
    put("ESC_4in1", d_board(27, 1.6).fuse(box(-7, 7, -7, 7, 1.6, 4.5)).cut(
        fuse(*[cz(1.1, x, y, -1, 9) for x, y in corners(STACK / 2)])),
        L(V(0, 0, Z_ESC)), C(0.15, 0.15, 0.15), False)
    put("fc_carrier", parts["fc_carrier_ESP32S3"], L(V(0, 0, Z_CARRIER)), blue)
    ex0 = -36.5
    put("ESP32_S3_SuperMini", box(ex0, ex0 + 22.5, -9, 9, 0, 1.6).fuse(box(ex0 + 3, ex0 + 17, -6, 6, 1.6, 4)),
        L(V(0.2, 0, Z_CARRIER + CARRIER_T)), C(0.1, 0.1, 0.12), False)
    put("gyro_GY521", box(-10.5, 10.5, -8, 8, 0, 1.6).fuse(box(-2, 2, -2, 2, 1.6, 2.5)),
        L(V(0, 0, Z_CARRIER + CARRIER_T + 1.5)), C(0.1, 0.3, 0.75), False)
    put("battery_3S_450", d_battery(), L(V(1, 0, Z_TOP + TOP_T + 0.6)), C(0.15, 0.3, 0.85), False)
    return parts, qty, asm, placed


PRINT_ROT = {"motor_guard_TPU": (V(1, 0, 0), 180)}

def export_all(out=OUT):
    os.makedirs(f"{out}/STL", exist_ok=True); os.makedirs(f"{out}/STEP", exist_ok=True)
    parts, qty, asm, placed = build()
    tol = dict(tolerance=0.01, angularTolerance=0.08)
    for n, s in parts.items():
        q = qty.get(n, 1); fn = f"{n}_x{q}" if q > 1 else n
        cq.exporters.export(cq.Workplane().add(s), f"{out}/STEP/{fn}.step")
        if n in PRINT_ROT: s = s.rotate(V(0, 0, 0), *PRINT_ROT[n])
        bb = s.BoundingBox()
        s = s.translate(V(-(bb.xmin + bb.xmax) / 2, -(bb.ymin + bb.ymax) / 2, -bb.zmin))
        cq.exporters.export(cq.Workplane().add(s), f"{out}/STL/{fn}.stl", **tol)
    pr = cq.Compound.makeCompound([s for _, s, _, p in placed if p])
    al = cq.Compound.makeCompound([s for _, s, _, _ in placed])
    cq.exporters.export(cq.Workplane().add(pr), f"{out}/fpv3_full_assembly.stl", **tol)
    cq.exporters.export(cq.Workplane().add(al), f"{out}/fpv3_full_assembly_with_electronics.stl", **tol)
    asm.save(f"{out}/fpv3_full_assembly.step")
    return parts, placed

if __name__ == "__main__":
    export_all(); print("done ->", OUT)
