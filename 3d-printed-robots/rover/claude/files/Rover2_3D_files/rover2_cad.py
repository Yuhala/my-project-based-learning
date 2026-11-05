"""
Rover 2 (Upgraded) - parametric CAD model
=========================================
Builds every 3D-printable part of the 4WD rover with 4-DOF arm, exports
STL (for slicing) and STEP (for editing in FreeCAD), plus the full assembly.

Run:   pip install cadquery   then   python rover2_cad.py
Units: millimetres.  World frame: +X forward, +Y left->right is +Y = right side,
       +Z up, Z=0 is the ground.

All key dimensions are in the PARAMETERS block - change them to fit your
exact motors / servos / boards and re-run.
"""
import math, os
import cadquery as cq
from cadquery import Vector as V, Location as L

OUT = os.environ.get("ROVER_OUT", "./rover2_out")

# =============================== PARAMETERS ===============================
# --- chassis ---
CH_L, CH_W   = 210.0, 120.0     # base / top plate footprint
PLATE_T      = 4.0              # base and top plate thickness
PANEL_H      = 50.0             # height between base plate and top plate
WALL_T       = 3.0              # panel wall thickness
WHEEL_D      = 80.0
WHEELBASE    = 156.0
AXLE_X       = WHEELBASE / 2

# --- TT gear motor (yellow "DC gear motor") ---
TT_H, TT_T   = 22.5, 18.8       # gearbox height (Z) and thickness (along shaft)
TT_SHAFT_END = 11.0             # shaft centre -> gearbox front end
TT_LEN       = 64.0             # gearbox + motor length
TT_HOLE_DX   = 20.0             # shaft -> mounting holes (along length)
TT_HOLE_DZ   = 17.5             # spacing between the 2 mounting holes
TT_D_DIA, TT_D_FLAT = 5.5, 3.8  # D-shaft bore (with print clearance)

# --- MG996R servo (base, shoulder, elbow) ---
SV_L, SV_W, SV_H = 40.7, 19.7, 37.0   # body (H = bottom -> case top)
SV_SHAFT_FROM_END = 10.2              # shaft centre -> near end of body
SV_TAB_Z     = 26.5                   # bottom -> underside of mounting tabs
SV_TAB_SPAN  = 54.5
SV_TAB_HOLE_DX, SV_TAB_HOLE_DY = 49.5, 10.0
HORN_FACE    = 7.5                    # case top -> outer face of round horn
HORN_D       = 25.6                   # round horn recess (with clearance)
HORN_RECESS  = 2.2
HORN_SCREW_PCD = 14.0                 # 4 horn screws on this circle

# --- SG90 / MG90S servo (gripper) ---
SG_L, SG_W, SG_H = 22.5, 12.2, 22.7
SG_SHAFT_FROM_END = 6.1
SG_TAB_Z     = 15.9
SG_TAB_HOLE_DX = 27.5
SG_HORN_TOP  = 11.8                   # tab underside -> top of single-arm horn

# --- arm ---
ARM_X        = 25.0                   # base-servo axis position on top plate
L_UPPER      = 90.0                   # shoulder U-bracket leg length
L_FORE       = 70.0                   # elbow U-bracket leg length
GEAR_M, GEAR_Z = 1.5, 16              # gripper gears
GEAR_CD      = GEAR_M * GEAR_Z        # centre distance (24 mm)

# --- fasteners ---
M3_CLR, M3_TAP, M3_INSERT = 3.4, 2.8, 4.0

# =========================== DERIVED HEIGHTS ===============================
SHAFT_Z     = WHEEL_D / 2
BRK_FLANGE  = 4.0
Z_BASE_BOT  = SHAFT_Z + TT_H / 2 + BRK_FLANGE
Z_BASE_TOP  = Z_BASE_BOT + PLATE_T
Z_TOP_BOT   = Z_BASE_TOP + PANEL_H
Z_TOP       = Z_TOP_BOT + PLATE_T
DECK_T      = 5.0
TT_UNDER    = DECK_T + (SV_H - SV_TAB_Z) + HORN_FACE - HORN_RECESS  # turntable underside
TURN_T      = 5.0

# ============================== HELPERS ====================================
def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))

def cyl(r, p, d, h):
    """cylinder radius r, from point p along unit direction d, length h"""
    return cq.Solid.makeCylinder(r, h, V(*p), V(*d))

def cz(r, x, y, z0, z1):  return cyl(r, (x, y, z0), (0, 0, 1), z1 - z0)
def cy(r, x, z, y0, y1):  return cyl(r, (x, y0, z), (0, 1, 0), y1 - y0)
def cx(r, y, z, x0, x1):  return cyl(r, (x0, y, z), (1, 0, 0), x1 - x0)

def ring_z(r0, r1, z0, z1, x=0, y=0):
    return cz(r1, x, y, z0, z1).cut(cz(r0, x, y, z0 - 1, z1 + 1))

def fuse(*s):
    out = s[0]
    for o in s[1:]:
        out = out.fuse(o)
    return out.clean()

def cut(base, *tools):
    return base.cut(*tools).clean()

def fillet_vertical(shape, r):
    """round the vertical (Z-parallel) outer edges of a plate"""
    return cq.Workplane().add(shape).edges("|Z").fillet(r).val()

def rbox_y(x, z, w, h, y0, y1, r):
    """rounded rectangle window (in XZ) extruded along Y"""
    b = cq.Workplane().add(box(x - w/2, x + w/2, y0, y1, z - h/2, z + h/2))
    return b.edges("|Y").fillet(r).val()

def rbox_x(y, z, w, h, x0, x1, r):
    b = cq.Workplane().add(box(x0, x1, y - w/2, y + w/2, z - h/2, z + h/2))
    return b.edges("|X").fillet(r).val()

def tri_rib_xz(x0, x1, z0, z1, y0, y1):
    """right-angle triangular gusset in the XZ plane (corner at x0,z0)"""
    w = (cq.Workplane("XZ", origin=(0, y1, 0))
         .polyline([(x0, z0), (x1, z0), (x0, z1)]).close().extrude(y1 - y0))
    return w.val()

def gear_profile(m, z, offset_deg=0.0, backlash=0.15, n=8):
    rp = m * z / 2; rb = rp * math.cos(math.radians(20))
    ra = rp + m;     rf = rp - 1.25 * m
    inv = lambda a: math.tan(a) - a
    ap = math.acos(rb / rp)
    half_p = (math.pi * m / 2 - backlash) / (2 * rp)
    def th(r):
        a = math.acos(min(1.0, rb / r))
        return half_p + inv(ap) - inv(a)
    rs = [rb + (ra - rb) * i / (n - 1) for i in range(n)]
    pts = []
    for k in range(z):
        c = math.radians(offset_deg) + 2 * math.pi * k / z
        a0 = c - th(rb)
        pts.append((rf * math.cos(a0), rf * math.sin(a0)))
        for r in rs:
            a = c - th(r); pts.append((r * math.cos(a), r * math.sin(a)))
        for r in reversed(rs):
            a = c + th(r); pts.append((r * math.cos(a), r * math.sin(a)))
        a1 = c + th(rb)
        pts.append((rf * math.cos(a1), rf * math.sin(a1)))
    return pts

def countersink_y(x, z, y_face, direction, d_head=6.6, d_hole=3.3):
    """cone for an M3 flat-head screw on a face normal to Y"""
    h = (d_head - d_hole) / 2
    return cq.Solid.makeCone(d_head / 2, d_hole / 2, h,
                             V(x, y_face, z), V(0, direction, 0))

# =============================== PARTS =====================================
SIDE_BOT_X = (-45.0, 0.0, 45.0)
SIDE_TOP_X = (-95.0, -45.0, 0.0, 45.0, 95.0)
SIDE_HOLE_Y = 52.0
END_HOLE_X, END_HOLE_Y = 97.0, (-30.0, 30.0)
BRK_HOLE_DX, BRK_HOLE_Y = 16.0, 29.0
CAM_HOLE_Y = (-10.0, 10.0)
HOLDER_X, HOLDER_HOLE_Y = -65.0, 21.0
DECK_HOLE = 40.0


def chassis_base():
    hx, hy = CH_L / 2, CH_W / 2
    b = fillet_vertical(box(-hx, hx, -hy, hy, 0, PLATE_T), 6)
    tools = []
    for s in (-1, 1):
        for x in SIDE_BOT_X:
            tools.append(cz(M3_CLR/2, x, s * SIDE_HOLE_Y, -1, 10))
        for y in END_HOLE_Y:
            tools.append(cz(M3_CLR/2, s * END_HOLE_X, y, -1, 10))
        for ax in (-AXLE_X, AXLE_X):                 # wheel-mount screws
            for dx in (-BRK_HOLE_DX, BRK_HOLE_DX):
                tools.append(cz(M3_CLR/2, ax + dx, s * BRK_HOLE_Y, -1, 10))
        for sx in (-1, 1):                           # motor wire pass-throughs
            tools.append(cz(5.0, sx * 35, s * 38, -1, 10))
    for x in (-10, 20):                              # battery strap slots
        for s in (-1, 1):
            tools.append(box(x - 11, x + 11, s * 24 - 1.5, s * 24 + 1.5, -1, 10))
    for x in (45, 55, 65, 75, 85):                   # M3 accessory grid
        for y in (-20, -10, 0, 10, 20):
            tools.append(cz(M3_CLR/2, x, y, -1, 10))
    tools.append(cz(6, -20, 0, -1, 10))              # central cable hole
    b = cut(b, *tools)
    # L298N standoffs (37 x 37 mm M3 pattern)
    so = []
    for dx in (-18.5, 18.5):
        for dy in (-18.5, 18.5):
            so.append(cz(3.5, -65 + dx, dy, PLATE_T, PLATE_T + 6))
    b = fuse(b, *so)
    holes = [cz(M3_TAP/2, -65 + dx, dy, -1, 20)
             for dx in (-18.5, 18.5) for dy in (-18.5, 18.5)]
    return cut(b, *holes)


def side_panel():
    """C-channel side wall (right side as modelled; the same part flips to the left)"""
    hx = CH_L / 2; yo = CH_W / 2; yi = yo - WALL_T; fl = yi - 10
    fx = hx - WALL_T - 0.2
    wall = box(-hx, hx, yi, yo, 0, PANEL_H)
    fb = box(-fx, fx, fl, yi, 0, 3)
    ft = box(-fx, fx, fl, yi, PANEL_H - 3, PANEL_H)
    p = fuse(wall, fb, ft)
    tools = [cz(M3_CLR/2, x, SIDE_HOLE_Y, -1, 4) for x in SIDE_BOT_X]
    tools += [cz(M3_CLR/2, x, SIDE_HOLE_Y, PANEL_H - 4, PANEL_H + 1) for x in SIDE_TOP_X]
    for x in (-62, 0, 62):                           # windows / cable access
        tools.append(rbox_y(x, PANEL_H / 2, 44, 24, yi - 1, yo + 1, 6))
    return cut(p, *tools)


def _end_panel():
    hx = CH_L / 2; xi = hx - WALL_T; hy = CH_W / 2 - WALL_T - 0.2
    wall = box(xi, hx, -hy, hy, 0, PANEL_H)
    fb = box(xi - 10, xi, -46, 46, 0, 3)
    ft = box(xi - 10, xi, -46, 46, PANEL_H - 3, PANEL_H)
    p = fuse(wall, fb, ft)
    tools = [cz(M3_CLR/2, END_HOLE_X, y, -1, 4) for y in END_HOLE_Y]
    tools += [cz(M3_CLR/2, END_HOLE_X, y, PANEL_H - 4, PANEL_H + 1)
              for y in END_HOLE_Y + CAM_HOLE_Y]
    return cut(p, *tools), xi, hx


def front_sensor_mount():
    p, xi, xo = _end_panel()
    zc = PANEL_H / 2
    tools = [cx(8.3, y, zc, xi - 1, xo + 1) for y in (-13, 13)]        # HC-SR04 eyes
    tools += [cx(0.9, y, zc + dz, xi - 1, xo + 1)                       # board screws
              for y in (-20.5, 20.5) for dz in (-8, 8)]
    return cut(p, *tools)


def rear_panel():
    p, xi, xo = _end_panel()
    zc = PANEL_H / 2
    tools = [box(xi - 1, xo + 1, -25 - 9.6, -25 + 9.6, zc - 6.6, zc + 6.6),  # KCD1 rocker switch
             rbox_x(0, 20, 10, 4.5, xi - 1, xo + 1, 1.5),                    # USB-C charge port
             cx(4.0, 25, zc, xi - 1, xo + 1)]                                # spare (DC jack/LED)
    return cut(p, *tools)


def top_plate():
    hx, hy = CH_L / 2, CH_W / 2
    b = fillet_vertical(box(-hx, hx, -hy, hy, 0, PLATE_T), 6)
    t = []
    for s in (-1, 1):
        for x in SIDE_TOP_X:
            t.append(cz(M3_CLR/2, x, s * SIDE_HOLE_Y, -1, 10))
        for y in END_HOLE_Y:
            t.append(cz(M3_CLR/2, s * END_HOLE_X, y, -1, 10))
    for y in CAM_HOLE_Y:
        t.append(cz(M3_CLR/2, END_HOLE_X, y, -1, 10))
    t.append(cz(4.0, 79, 0, -1, 10))                              # camera wires
    bx = ARM_X - (SV_L / 2 - SV_SHAFT_FROM_END)                    # base servo body
    t.append(box(bx - 21.5, bx + 21.5, -11, 11, -1, 10))
    for sx in (-1, 1):
        for sy in (-1, 1):
            t.append(cz(M3_CLR/2, ARM_X + sx * DECK_HOLE, sy * DECK_HOLE, -1, 10))
    for s in (-1, 1):                                              # ESP32 holder
        t.append(cz(M3_CLR/2, HOLDER_X, s * HOLDER_HOLE_Y, -1, 10))
    t.append(rbox_x(0, 2, 40, 8, HOLDER_X - 4, HOLDER_X + 4, 3.9).rotate(
        V(HOLDER_X, 0, 0), V(HOLDER_X, 0, 1), 90))
    t.append(box(-31, -25, -15, 15, -1, 10))                       # cable slot
    return cut(b, *t)


def wheel_mount():
    """TT-motor bracket.  Local: shaft on Y axis at X=Z=0, right-hand side.
    Symmetric in X, so all 4 corners use the same print."""
    yo = CH_W / 2; mo = yo - WALL_T - 0.2; mi = mo - TT_T
    zt = TT_H / 2
    flange = box(-26, 26, 24, yo, zt, zt + BRK_FLANGE)
    outer = box(-26, 26, yo - WALL_T, yo, -15, zt)
    inner = box(-26, 26, mi - 0.2 - WALL_T, mi - 0.2, -15, zt)
    ribs = []
    for x in (-26, 23):
        ribs.append(box(x, x + 3, 24, mi - 0.2 - WALL_T, zt - 8, zt))
    p = fuse(flange, outer, inner, *ribs)
    t = [cy(8.0, 0, 0, yo - WALL_T - 1, yo + 1),                    # wheel hub clearance
         cy(4.0, 0, 0, mi - 4 - WALL_T, mi + 1)]                     # inner shaft stub
    for dx in (-TT_HOLE_DX, TT_HOLE_DX):
        for dz in (-TT_HOLE_DZ / 2, TT_HOLE_DZ / 2):
            t.append(cy(1.65, dx, dz, mi - 6 - WALL_T, yo + 1))
            t.append(countersink_y(dx, dz, yo, -1))
    for dx in (-BRK_HOLE_DX, BRK_HOLE_DX):
        t.append(cz(M3_CLR/2, dx, BRK_HOLE_Y, zt - 9, zt + 10))
    return cut(p, *t)


def wheel_rim():
    """Local: axis +Y, inner face at Y=0 (towards chassis)."""
    W = 26.0
    rim = cy(30.0, 0, 0, 0, W).cut(cy(27.0, 0, 0, -1, W + 1))
    ridge = cy(30.8, 0, 0, W/2 - 1.5, W/2 + 1.5).cut(cy(29, 0, 0, 0, W))
    web = cy(27.5, 0, 0, W - 4, W).cut(cy(6.5, 0, 0, W - 5, W + 1))
    hub = cy(7.0, 0, 0, -4.5, W)
    p = fuse(rim, ridge, web, hub)
    t = []
    for k in range(6):
        a = 2 * math.pi * k / 6
        t.append(cy(5.0, 17 * math.cos(a), 17 * math.sin(a), W - 5, W + 1))
    bore = cy(TT_D_DIA / 2, 0, 0, -5, 5).cut(
        fuse(box(-5, 5, -6, 6, TT_D_FLAT / 2, 5), box(-5, 5, -6, 6, -5, -TT_D_FLAT / 2)))
    bore = bore.rotate(V(0, 0, 0), V(0, 1, 0), 90)   # flats horizontal on the shaft
    t += [bore, cy(1.2, 0, 0, 4, W + 1), cy(3.0, 0, 0, 6, W + 1)]   # M2 screw + counterbore
    return cut(p, *t)


def wheel_tire():
    """Print in TPU 95A (or PLA for a hard wheel).  Same local frame as rim."""
    W = 26.0
    body = cy(36.5, 0, 0, -1, W + 1).cut(cy(29.8, 0, 0, -2, W + 2))
    body = cq.Workplane().add(body).faces(">Y or <Y").edges().chamfer(1.2).val()
    body = body.cut(cy(31.0, 0, 0, W/2 - 1.7, W/2 + 1.7).cut(cy(29, 0, 0, 0, W)))
    lugs = []
    n = 18
    for k in range(n):
        for half, (y0, y1) in enumerate(((-1, 12.2), (13.8, W + 1))):
            a = 360.0 * k / n + (180.0 / n if half else 0)
            lug = box(-3.5, 3.5, y0, y1, 35.5, 40.0)
            lug = cq.Workplane().add(lug).edges("|Y").fillet(1.0).val()
            lugs.append(lug.rotate(V(0, 0, 0), V(0, 1, 0), a))
    return fuse(body, *lugs)


def arm_base_mount():
    """Local: base-servo axis = Z at X=Y=0, Z=0 = top surface of top plate.
    The MG996R hangs from its tabs through the deck into the chassis."""
    off = SV_L / 2 - SV_SHAFT_FROM_END
    deck = fillet_vertical(box(-48, 48, -48, 48, 0, DECK_T), 8)
    ring_top = TT_UNDER - 0.5
    sup = ring_z(39.5, 43.0, DECK_T, ring_top)
    p = fuse(deck, sup)
    t = [box(-off - 20.6, -off + 20.6, -10.1, 10.1, -1, DECK_T + 1)]
    for dx in (-SV_TAB_HOLE_DX / 2, SV_TAB_HOLE_DX / 2):
        for dy in (-SV_TAB_HOLE_DY / 2, SV_TAB_HOLE_DY / 2):
            t.append(cz(M3_TAP / 2, -off + dx, dy, -1, DECK_T + 1))
    for sx in (-1, 1):
        for sy in (-1, 1):
            t.append(cz(M3_CLR/2, sx * DECK_HOLE, sy * DECK_HOLE, -1, 10))
    for k in range(8):                                   # lightening
        a = 2 * math.pi * k / 8 + math.pi / 8
        t.append(cz(3.5, 33 * math.cos(a), 33 * math.sin(a), -1, DECK_T + 1))
    return cut(p, *t)


def turntable_retainer():
    """Holds the turntable down and takes tipping loads off the base servo.
    Prints upside-down (lip + ears on the bed), no supports."""
    lip_z0 = TT_UNDER + TURN_T + 0.4
    ztop = lip_z0 + 2.5
    wall = ring_z(44.6, 47.5, DECK_T, ztop)
    lip = ring_z(40.0, 47.5, lip_z0, ztop)
    ears, posts, t = [], [], []
    for k in range(4):
        a = 45 + 90 * k
        e = box(44, 62, -6, 6, lip_z0, ztop)
        e = cq.Workplane().add(e).edges("|Z").fillet(2).val()
        ears.append(e.rotate(V(0, 0, 0), V(0, 0, 1), a))
        r = DECK_HOLE * math.sqrt(2)
        px, py = r * math.cos(math.radians(a)), r * math.sin(math.radians(a))
        posts.append(cz(5.0, px, py, DECK_T, ztop))
        t.append(cz(M3_CLR / 2, px, py, 0, ztop + 1))
    p = fuse(wall, lip, *ears, *posts)
    return cut(p, *t)


# servo bracket mounting-hole layout (shared by turntable & U-bracket webs)
RAIL_Y = (-17.0, 3.0)
RAIL_X_TURN = (-30.0, 10.0)
RAIL_X_WEB = (-20.0, 0.0)
SHOULDER_DX, SHOULDER_DY = 10.0, 0.25        # bracket origin rel. turntable centre


def turntable():
    """Local: Z=0 underside, axis Z."""
    p = cz(44.0, 0, 0, 0, TURN_T)
    t = [cz(HORN_D / 2, 0, 0, -1, HORN_RECESS), cz(4.5, 0, 0, -1, 10)]
    for k in range(4):
        a = math.pi / 4 + k * math.pi / 2
        r = HORN_SCREW_PCD / 2
        t.append(cz(1.0, r * math.cos(a), r * math.sin(a), -1, 10))
    for x in RAIL_X_TURN:
        for y in RAIL_Y:
            px, py = x + SHOULDER_DX, y + SHOULDER_DY
            t.append(cz(M3_CLR/2, px, py, -1, 10))
            t.append(cz(3.3, px, py, -1, 2.0))
    for k in range(6):                                   # lightening
        a = 2 * math.pi * k / 6 + 0.3
        t.append(cz(4.0, 36 * math.cos(a), 36 * math.sin(a), -1, 10))
    return cut(p, *t)


def servo_bracket():
    """MG996R bracket.  Local: servo output shaft = +Y axis through origin,
    servo body towards -X, mounting rail (heat-set inserts) on the -Z face."""
    off = SV_L / 2 - SV_SHAFT_FROM_END
    tab_y = -SV_H / 2 + SV_TAB_Z
    x0, x1 = -40.0, 20.0
    wall = box(x0, x1, tab_y - 4, tab_y, -17, 14)
    rail = box(x0, x1, -22.5, tab_y, -17, -11)
    top = box(x0, x1, -22.5, tab_y, 11, 14)
    back = box(x0, x1, -22.5, -19, -17, 14)
    boss = cy(5.0, 0, 0, -24, -19)
    p = fuse(wall, rail, top, back, boss)
    t = [box(-off - 20.6, -off + 20.6, tab_y - 5, tab_y + 1, -10.1, 10.1)]
    for dx in (-SV_TAB_HOLE_DX / 2, SV_TAB_HOLE_DX / 2):
        for dz in (-SV_TAB_HOLE_DY / 2, SV_TAB_HOLE_DY / 2):
            t.append(cy(M3_TAP / 2, -off + dx, dz, tab_y - 5, tab_y + 1))
    t.append(cy(M3_INSERT / 2, 0, 0, -24.5, -18))        # pivot insert
    for x in RAIL_X_TURN + RAIL_X_WEB:
        for y in RAIL_Y:
            t.append(cz(M3_INSERT / 2, x, y, -18, -11.5))
    t.append(rbox_y(-12, 0, 26, 10, -23, -18, 4))        # lighten back plate
    return cut(p, *t)


def u_bracket(L, web_holes):
    """Arm link.  Local: joint axis = Y (horn leg on +Y), legs extend +X."""
    hy0, hy1 = 24.0, 29.0          # horn-side leg
    iy0, iy1 = -29.5, -24.5        # idler leg
    legs = []
    for y0, y1 in ((hy0, hy1), (iy0, iy1)):
        legs += [cy(16.0, 0, 0, y0, y1), box(0, L, y0, y1, -15, 15)]
    web = box(L, L + 5, iy0, hy1, -15, 15)
    p = fuse(*legs, web)
    t = [cy(HORN_D / 2, 0, 0, hy0 - 1, hy0 + HORN_RECESS), cy(4.5, 0, 0, hy0 - 1, hy1 + 1),
         cy(1.65, 0, 0, iy0 - 1, iy1 + 1)]
    for k in range(4):
        a = math.pi / 4 + k * math.pi / 2; r = HORN_SCREW_PCD / 2
        t.append(cy(1.0, r * math.cos(a), r * math.sin(a), hy0 - 1, hy1 + 1))
    for (y, z) in web_holes:
        t.append(cx(M3_CLR / 2, y, z, L - 1, L + 6))
    for y0, y1 in ((hy0, hy1), (iy0, iy1)):
        t.append(rbox_y((22 + L - 12) / 2, 0, L - 34, 12, y0 - 1, y1 + 1, 5.9))
    return cut(p, *t)


def upper_arm():
    return u_bracket(L_UPPER, [(y, -x - 10) for x in RAIL_X_WEB for y in RAIL_Y])


GRIP_HOLES = [(y, z) for y in (-10, 10) for z in (-10, 10)]


def forearm():
    return u_bracket(L_FORE, GRIP_HOLES)


GEAR_Z0 = SG_HORN_TOP - 1.6          # gear underside height above gripper plate


def gripper_base():
    """Local: plate top Z=0, fingers point +X, gears at Y=+-12."""
    off = SG_L / 2 - SG_SHAFT_FROM_END
    sx = -off; sy = -GEAR_CD / 2
    plate = box(-24, 16, -26, 26, -4, 0)
    plate = fillet_vertical(plate, 4)
    flange = box(-28, -24, -22, 22, -16, 16)
    pillar = cz(4.5, 0, GEAR_CD / 2, 0, GEAR_Z0 - 0.5)
    ribs = [tri_rib_xz(-24, -12, 0, 12, y - 1.5, y + 1.5) for y in (-20, 20)]
    p = fuse(plate, flange, pillar, *ribs)
    t = [box(sx - SG_L / 2 - 0.25, sx + SG_L / 2 + 0.25, sy - SG_W / 2 - 0.25,
             sy + SG_W / 2 + 0.25, -5, 1)]
    t += [cz(0.9, sx + d, sy, -5, 1) for d in (-SG_TAB_HOLE_DX / 2, SG_TAB_HOLE_DX / 2)]
    t.append(cz(M3_INSERT / 2, 0, GEAR_CD / 2, GEAR_Z0 - 7, GEAR_Z0))
    t += [cx(M3_CLR / 2, y, z, -29, -23) for (y, z) in GRIP_HOLES]
    return cut(p, *t)


def _finger(top=True):
    """Finger + gear.  Local frame = gripper frame, Z=0 at gear underside."""
    s = 1 if top else -1
    cyg = s * GEAR_CD / 2
    g = (cq.Workplane("XY").polyline(gear_profile(GEAR_M, GEAR_Z,
         0.0 if top else 180.0 / GEAR_Z)).close().extrude(6).val())
    g = g.translate(V(0, cyg, 0))
    arm = [(4, 17.5), (32, 17.5), (58, 7), (58, 0.75), (42, 0.75), (30, 7), (4, 7)]
    arm = [(x, s * y) for x, y in arm]
    if not top:
        arm = list(reversed(arm))
    a = cq.Workplane("XY").polyline(arm).close().extrude(6).val()
    hubdisc = cz(8, 0, cyg, 0, 6)
    p = fuse(g, a, hubdisc)
    t = [box(x, x + 1.0, -1.2 if top else 0.75 - 0.6, 0.75 + 0.6 if top else 1.2, -1, 7)
         for x in (44, 48, 52)]                                   # grip grooves
    if top:
        t.append(cz(1.65, 0, cyg, -1, 7))
    else:
        t.append(cz(2.75, 0, cyg, -1, 7))
        horn = (cq.Workplane("XY").polyline([(0, 2.9), (-16, 2.1), (-16, -2.1), (0, -2.9)])
                .close().extrude(1.6).val().translate(V(0, cyg, 0)))
        t += [horn, cz(3.9, 0, cyg, -1, 1.6)]
        t += [cz(0.75, -r, cyg, -1, 7) for r in (8, 11)]
    return cut(p, *t)


def gripper_finger_idle():  return _finger(True)
def gripper_finger_drive(): return _finger(False)


def camera_mount():
    """ESP32-CAM holder.  Local: board stands in YZ plane facing +X, Z=0 = top plate."""
    flange = box(-14, 12, -18, 18, 0, 3)
    front = box(3, 5, -17.5, 17.5, 0, 44)
    rails = [box(-7, 3, s0, s1, 0, 44) for s0, s1 in ((12.5, 17.5), (-17.5, -12.5))]
    p = fuse(flange, front, *rails)
    t = [box(-4.0, -2.2, 12.5 - 0.1, 13.8, 3, 45), box(-4.0, -2.2, -13.8, -12.4, 3, 45),
         rbox_x(0, 26, 20, 28, 2, 6, 3),
         cz(M3_CLR / 2, 9, -10, -1, 4), cz(M3_CLR / 2, 9, 10, -1, 4),
         cz(4.0, -9, 0, -1, 4)]
    return cut(p, *t)


def electronics_holder():
    """ESP32 DevKit tray (fits ~51-57 x 28 mm boards).  Local Z=0 = top plate."""
    ix, iy, w = 57.0 / 2, 29.0 / 2, 2.0
    ztop = 22.4
    outer = box(-ix - w, ix + w, -iy - w, iy + w, 0, ztop)
    outer = fillet_vertical(outer, 2)
    p = outer.cut(box(-ix, ix, -iy, iy, 2, ztop + 1))
    ledges = [box(-ix, ix, s * iy - (1.0 if s > 0 else 0), s * iy + (0 if s > 0 else 1.0), 17, 20)
              for s in (-1, 1)]
    lips = [box(-ix, ix, s * iy - (0.6 if s > 0 else 0), s * iy + (0 if s > 0 else 0.6),
                21.7, ztop) for s in (-1, 1)]
    ears = [box(-6, 6, s0, s1, 0, 3) for s0, s1 in ((iy + w - 1, 25), (-25, -iy - w + 1))]
    p = fuse(p, *ledges, *lips, *ears)
    t = [box(-20, 20, -4, 4, -1, 3),                          # wire slot
         box(-ix - w - 1, -ix + 1, -6, 6, 6, ztop + 1),       # USB access
         cz(M3_CLR / 2, 0, HOLDER_HOLE_Y, -1, 4), cz(M3_CLR / 2, 0, -HOLDER_HOLE_Y, -1, 4)]
    return cut(p, *t)


# ======================= REFERENCE (non-printed) parts ======================
def dummy_tt_motor():
    mo = CH_W / 2 - WALL_T - 0.2; mi = mo - TT_T
    g = box(-(TT_LEN - TT_SHAFT_END), TT_SHAFT_END, mi, mo, -TT_H / 2, TT_H / 2)
    shaft = cy(2.7, 0, 0, mi - 8.5, mo + 8.5)
    return fuse(g, shaft)

def dummy_mg996r():
    off = SV_L / 2 - SV_SHAFT_FROM_END
    b = box(-off - SV_L / 2, -off + SV_L / 2, -SV_H / 2, SV_H / 2, -SV_W / 2, SV_W / 2)
    ty = -SV_H / 2 + SV_TAB_Z
    tab = box(-off - SV_TAB_SPAN / 2, -off + SV_TAB_SPAN / 2, ty, ty + 2.5, -SV_W / 2, SV_W / 2)
    sp = cy(3, 0, 0, SV_H / 2, SV_H / 2 + HORN_FACE - 2.4)
    horn = cy(12.5, 0, 0, SV_H / 2 + HORN_FACE - 2.4, SV_H / 2 + HORN_FACE)
    return fuse(b, tab, sp, horn)

def dummy_sg90():
    off = SG_L / 2 - SG_SHAFT_FROM_END
    b = box(-off - SG_L / 2, -off + SG_L / 2, -SG_W / 2, SG_W / 2, -SG_TAB_Z, SG_H - SG_TAB_Z)
    tab = box(-off - 16.1, -off + 16.1, -SG_W / 2, SG_W / 2, 0, 2.5)
    return fuse(b, tab, cz(2.5, 0, 0, 0, SG_HORN_TOP - 1.6))

def dummy_board(dx, dy, dz):  return box(-dx / 2, dx / 2, -dy / 2, dy / 2, 0, dz)


# =============================== ASSEMBLY ==================================
def build(pose=(0.0, -60.0, -20.0, 20.0)):
    """pose = (base_yaw, shoulder, elbow, gripper_open) in degrees"""
    yaw, sh, el, op = pose
    parts = {
        "chassis_base": chassis_base(), "side_panel": side_panel(),
        "front_sensor_mount": front_sensor_mount(), "rear_panel": rear_panel(),
        "top_plate": top_plate(), "wheel_mount": wheel_mount(),
        "wheel_rim": wheel_rim(), "wheel_tire_TPU": wheel_tire(),
        "arm_base_mount": arm_base_mount(), "turntable_retainer": turntable_retainer(),
        "turntable": turntable(), "servo_bracket_MG996R": servo_bracket(),
        "arm_upper_link": upper_arm(), "arm_forearm_link": forearm(),
        "gripper_base": gripper_base(), "gripper_finger_drive": gripper_finger_drive(),
        "gripper_finger_idle": gripper_finger_idle(), "camera_mount": camera_mount(),
        "electronics_holder": electronics_holder(),
    }
    qty = {"side_panel": 2, "wheel_mount": 4, "wheel_rim": 4, "wheel_tire_TPU": 4,
           "servo_bracket_MG996R": 2}

    black = cq.Color(0.13, 0.13, 0.14); dgrey = cq.Color(0.22, 0.22, 0.24)
    tire = cq.Color(0.07, 0.07, 0.07); orange = cq.Color(0.95, 0.45, 0.1)
    yellow = cq.Color(0.95, 0.8, 0.1); blue = cq.Color(0.1, 0.3, 0.8)
    green = cq.Color(0.1, 0.45, 0.2); servo_c = cq.Color(0.05, 0.05, 0.05)

    asm = cq.Assembly(name="Rover2")
    placed = []           # (name, shape, color, printed?)

    def put(name, shape, loc, color, printed=True):
        asm.add(shape, name=name, loc=loc, color=color)
        placed.append((name, shape.moved(loc), color, printed))

    Rz180 = L(V(0, 0, 0), V(0, 0, 1), 180)
    put("chassis_base", parts["chassis_base"], L(V(0, 0, Z_BASE_BOT)), dgrey)
    put("side_panel_R", parts["side_panel"], L(V(0, 0, Z_BASE_TOP)), black)
    put("side_panel_L", parts["side_panel"], L(V(0, 0, Z_BASE_TOP)) * Rz180, black)
    put("front_sensor_mount", parts["front_sensor_mount"], L(V(0, 0, Z_BASE_TOP)), black)
    put("rear_panel", parts["rear_panel"], L(V(0, 0, Z_BASE_TOP)) * Rz180, black)
    put("top_plate", parts["top_plate"], L(V(0, 0, Z_TOP_BOT)), dgrey)

    for i, (ax, side) in enumerate(((AXLE_X, 1), (-AXLE_X, 1), (AXLE_X, -1), (-AXLE_X, -1))):
        tag = ("F" if ax > 0 else "R") + ("R" if side > 0 else "L")
        base_loc = L(V(ax, 0, SHAFT_Z)) * (Rz180 if side < 0 else L())
        put(f"wheel_mount_{tag}", parts["wheel_mount"], base_loc, black)
        m = dummy_tt_motor()
        if ax < 0: m = m.mirror("YZ")
        if side < 0: m = m.mirror("XZ")
        put(f"TT_motor_{tag}", m, L(V(ax, 0, SHAFT_Z)), yellow, False)
        wl = L(V(ax, side * (CH_W / 2 + 2), SHAFT_Z)) * (Rz180 if side < 0 else L())
        put(f"wheel_rim_{tag}", parts["wheel_rim"], wl, orange)
        put(f"wheel_tire_{tag}", parts["wheel_tire_TPU"], wl, tire)

    # electronics (reference only)
    put("ESP32_devkit", dummy_board(55, 28, 1.6), L(V(HOLDER_X, 0, Z_TOP + 20)), blue, False)
    put("ESP32_CAM", box(-1.6, 0, -13.5, 13.5, 0, 40.5).fuse(box(0, 6, -4.5, 4.5, 18, 27)),
        L(V(88 - 2.2, 0, Z_TOP + 3)), green, False)
    put("HC_SR04", box(-1.6, 0, -22.5, 22.5, -10, 10).fuse(
        cx(8, -13, 0, 0, 12), cx(8, 13, 0, 0, 12)),
        L(V(CH_L / 2 - 12 + 0.5, 0, Z_BASE_TOP + PANEL_H / 2)), blue, False)
    put("battery_2x18650", dummy_board(77, 41, 20), L(V(5, 0, Z_BASE_TOP)), blue, False)
    put("L298N", dummy_board(43, 43, 1.6).fuse(box(-12, 12, -8, 8, 1.6, 25)),
        L(V(-65, 0, Z_BASE_TOP + 6)), cq.Color(0.8, 0.1, 0.1), False)

    # arm
    base_loc = L(V(ARM_X, 0, Z_TOP))
    put("arm_base_mount", parts["arm_base_mount"], base_loc, black)
    put("turntable_retainer", parts["turntable_retainer"], base_loc, black)
    sv_base = L(V(0, 0, DECK_T + SV_H / 2 - SV_TAB_Z)) * L(V(0, 0, 0), V(1, 0, 0), -90)
    put("MG996R_base", dummy_mg996r(), base_loc * sv_base, servo_c, False)
    tt = base_loc * L(V(0, 0, TT_UNDER)) * L(V(0, 0, 0), V(0, 0, 1), yaw)
    put("turntable", parts["turntable"], tt, dgrey)
    shoulder = tt * L(V(SHOULDER_DX, SHOULDER_DY, TURN_T + 17))
    put("servo_bracket_shoulder", parts["servo_bracket_MG996R"], shoulder, black)
    put("MG996R_shoulder", dummy_mg996r(), shoulder, servo_c, False)
    up = shoulder * L(V(0, 0, 0), V(0, 1, 0), sh)
    put("arm_upper_link", parts["arm_upper_link"], up, black)
    elbow = up * L(V(L_UPPER + 22, 0, -10)) * L(V(0, 0, 0), V(0, 1, 0), 90)
    put("servo_bracket_elbow", parts["servo_bracket_MG996R"], elbow, black)
    put("MG996R_elbow", dummy_mg996r(), elbow, servo_c, False)
    fo = elbow * L(V(0, 0, 0), V(0, 1, 0), el)
    put("arm_forearm_link", parts["arm_forearm_link"], fo, black)
    gr = fo * L(V(L_FORE + 5 + 28, 0, 0))
    put("gripper_base", parts["gripper_base"], gr, black)
    put("SG90_gripper", dummy_sg90(), gr * L(V(0, -GEAR_CD / 2, 0)), blue, False)
    put("gripper_finger_idle", parts["gripper_finger_idle"],
        gr * L(V(0, GEAR_CD / 2, GEAR_Z0)) * L(V(0, 0, 0), V(0, 0, 1), op)
        * L(V(0, -GEAR_CD / 2, 0)), dgrey)
    put("gripper_finger_drive", parts["gripper_finger_drive"],
        gr * L(V(0, -GEAR_CD / 2, GEAR_Z0)) * L(V(0, 0, 0), V(0, 0, 1), -op)
        * L(V(0, GEAR_CD / 2, 0)), dgrey)
    put("camera_mount", parts["camera_mount"], L(V(88, 0, Z_TOP)), black)
    put("electronics_holder", parts["electronics_holder"], L(V(HOLDER_X, 0, Z_TOP)), black)
    return parts, qty, asm, placed


# print orientation (rotation applied before STL export so parts lie flat)
PRINT_ROT = {
    "wheel_mount": (V(1, 0, 0), 180),          # flange on the bed
    "wheel_rim": (V(1, 0, 0), -90), "wheel_tire_TPU": (V(1, 0, 0), 90),
    "side_panel": (V(1, 0, 0), -90),           # wall on the bed
    "front_sensor_mount": (V(0, 1, 0), 90), "rear_panel": (V(0, 1, 0), 90),
    "servo_bracket_MG996R": (V(1, 0, 0), 0),
    "arm_upper_link": (V(0, 1, 0), 90), "arm_forearm_link": (V(0, 1, 0), 90),
    "gripper_base": (V(0, 1, 0), -90),           # flange on the bed
    "turntable_retainer": (V(1, 0, 0), 180),     # lip on the bed
}


def export_all(out=OUT):
    os.makedirs(f"{out}/STL", exist_ok=True); os.makedirs(f"{out}/STEP", exist_ok=True)
    parts, qty, asm, placed = build()
    tol = dict(tolerance=0.02, angularTolerance=0.1)
    for name, shp in parts.items():
        n = qty.get(name, 1)
        fn = f"{name}_x{n}" if n > 1 else name
        cq.exporters.export(cq.Workplane().add(shp), f"{out}/STEP/{fn}.step")
        s = shp
        if name in PRINT_ROT:
            ax, ang = PRINT_ROT[name]
            s = s.rotate(V(0, 0, 0), ax, ang)
        bb = s.BoundingBox()
        s = s.translate(V(-(bb.xmin + bb.xmax) / 2, -(bb.ymin + bb.ymax) / 2, -bb.zmin))
        cq.exporters.export(cq.Workplane().add(s), f"{out}/STL/{fn}.stl", **tol)
    printed = cq.Compound.makeCompound([s for _, s, _, p in placed if p])
    full = cq.Compound.makeCompound([s for _, s, _, _ in placed])
    cq.exporters.export(cq.Workplane().add(printed), f"{out}/rover2_full_assembly.stl", **tol)
    cq.exporters.export(cq.Workplane().add(full), f"{out}/rover2_full_assembly_with_electronics.stl", **tol)
    asm.save(f"{out}/rover2_full_assembly.step")
    return parts, placed


if __name__ == "__main__":
    export_all()
    print("done ->", OUT)
