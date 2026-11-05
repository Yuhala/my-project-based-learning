"""
Mini tank (~157 x 108 mm) - parametric CadQuery model
=====================================================
2 x N20 gear motors (direct drive), 3D-printed hull, tracks, sprockets,
road wheels, idlers, pan-camera head.  ESP32-S3 + SG90/MG90S pan servo.
Only uses: Besitu M2/M3 socket screws + nuts + washers, M3 countersunk
screws (5-30 mm), M2/M3 heat-set inserts, and 1.75 mm filament as track pins.

pip install cadquery scipy ; python tank_cad.py
Units mm.  +X forward, +Y left, Z up, Z=0 = ground.
"""
import math, os
import numpy as np
import cadquery as cq
from cadquery import Vector as V, Location as L

OUT = os.environ.get("TANK_OUT", "./tank_out")

# ================================ PARAMETERS ================================
# --- hull ---
HX, HY = 75.0, 35.0            # hull half-length / half-width (150 x 70)
WALL = 3.0
Z_FLOOR = 9.0                  # underside of hull (ground clearance)
FLOOR_T = 3.0
Z_DECK = 40.0                  # top of walls = underside of deck
DECK_T = 3.0
# --- N20 gear motor ---
N20_W, N20_H = 12.0, 10.0      # across width (X) and flats (Z)
N20_LEN = 24.0                 # gearbox + motor body
N20_SHAFT = 10.0               # shaft length from gearbox face
N20_D, N20_FLAT = 3.1, 2.6     # D-shaft bore (with clearance)
# --- track ---
PITCH = 10.0
N_TEETH = 10
N_LINKS = 34
TRACK_W = 18.0
KN_R, KN_Z = 2.3, 2.3          # knuckle radius / axis height above outer face
PLATE_T = 2.2
PIN_D = 1.95                   # for 1.75 mm filament pins
HORN_Y = (5.5, 6.8)            # guide horns (distance from track centre)
HORN_H = 3.5
# --- wheels / layout ---
RP = PITCH / (2 * math.sin(math.pi / N_TEETH))     # sprocket pitch radius 16.18
RW_R = 12.0                    # road wheel radius
WHEEL_W = 8.0
TRACK_Y0 = HY + 1.0            # track inner edge
TRACK_YC = TRACK_Y0 + TRACK_W / 2                 # track centre line (45)
Z_SPROCKET = KN_Z + 3.0 + RP                      # 21.48
Z_ROAD = KN_Z + KN_R + RW_R                       # 16.6
X_SPROCKET = -60.0
X_ROAD = (-29.0, 0.0, 29.0)
IDLER_SLOT = 4.0                                   # +- tension travel (X_IDLER solved below)
X_IDLER = 59.0
# --- servo / camera ---
SG_L, SG_W, SG_H = 22.5, 12.2, 22.7
SG_SHAFT_FROM_END, SG_TAB_Z, SG_TAB_DX = 6.1, 15.9, 27.5
SG_HORN_TOP = 11.8                                  # tab underside -> top of horn
X_SERVO = 50.0
CAM_W, CAM_H, CAM_T = 27.0, 40.5, 1.6               # camera board (ESP32-CAM-size)
# --- fasteners ---
M3, M2, M3_INS, M2_INS = 3.4, 2.2, 4.0, 3.2

# ================================ HELPERS ===================================
def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))
def cyl(r, p, d, h): return cq.Solid.makeCylinder(r, h, V(*p), V(*d))
def cz(r, x, y, z0, z1): return cyl(r, (x, y, z0), (0, 0, 1), z1 - z0)
def cy(r, x, z, y0, y1): return cyl(r, (x, y0, z), (0, 1, 0), y1 - y0)
def fuse(*s):
    o = s[0]
    for x in s[1:]: o = o.fuse(x)
    return o.clean()
def cut(b, *t): return b.cut(*t).clean()
def fil(s, sel, r): return cq.Workplane().add(s).edges(sel).fillet(r).val()
def slot_y(x0, x1, z, r, y0, y1):
    """stadium in XZ (from x0 to x1 at height z), extruded along Y"""
    return fuse(cy(r, x0, z, y0, y1), cy(r, x1, z, y0, y1), box(x0, x1, y0, y1, z - r, z + r))
def csk_y(x, z, y_face, direction, d_head=6.4):
    """countersink for M3 flat head on a face normal to Y"""
    return cq.Solid.makeCone(d_head / 2, 1.7, d_head / 2 - 1.7, V(x, y_face, z), V(0, direction, 0))

# ================================ PARTS =====================================
MOTOR_BOX_X = (X_SPROCKET - N20_W / 2 - 0.15 - 4.5, X_SPROCKET + N20_W / 2 + 0.15 + 4.5)
DECK_POSTS = [(sx * 45.0, sy * (HY - WALL - 3.5)) for sx in (-1, 1) for sy in (-1, 1)]


def hull():
    zf, zt = Z_FLOOR, Z_FLOOR + FLOOR_T
    shell = box(-HX, HX, -HY, HY, zf, Z_DECK).cut(box(-HX + WALL, HX - WALL, -HY + WALL, HY - WALL, zt, Z_DECK + 1))
    parts = [shell]
    yi = HY - WALL
    mz = Z_SPROCKET
    for s in (-1, 1):
        # --- motor box (built into the side wall) ---
        x0, x1 = MOTOR_BOX_X
        ya, yb = sorted((s * (yi - N20_LEN - 1.0), s * yi))
        ped = box(x0, x1, ya, yb, zt, mz - N20_H / 2)
        w1 = box(x0, x0 + 4.5, ya, yb, zt, mz + N20_H / 2 - 0.15)   # 0.15 below motor top:
        w2 = box(x1 - 4.5, x1, ya, yb, zt, mz + N20_H / 2 - 0.15)   # cap clamps the motor
        parts += [ped, w1, w2]
    for x, y in DECK_POSTS:
        parts.append(cz(4.0, x, y, zt, Z_DECK))
    p = fuse(*parts)
    t = []
    for s in (-1, 1):
        y_out = s * HY
        yw0, yw1 = sorted((s * (HY - WALL - 0.5), s * (HY + 0.5)))
        t.append(cy(2.6, X_SPROCKET, mz, yw0, yw1))                       # motor shaft + boss
        for x in X_ROAD:                                                  # road wheel axles
            t.append(cy(M3 / 2, x, Z_ROAD, yw0, yw1))
        t.append(slot_y(X_IDLER - IDLER_SLOT, X_IDLER + IDLER_SLOT, Z_SPROCKET, M3 / 2, yw0, yw1))
        # motor cap inserts
        for xx in (MOTOR_BOX_X[0] + 2.25, MOTOR_BOX_X[1] - 2.25):
            for yy in (s * (yi - 5), s * (yi - N20_LEN + 4)):
                t.append(cz(M2_INS / 2, xx, yy, mz + N20_H / 2 - 4.5, mz + N20_H / 2 + 1))
    for x, y in DECK_POSTS:
        t.append(cz(M3_INS / 2, x, y, Z_DECK - 6, Z_DECK + 1))
    # floor: battery strap slots, wire hole, rear-wall switch hole
    for x in (-30.0, 15.0):
        for s in (-1, 1):
            t.append(box(x - 10, x + 10, s * 23 - 1.5, s * 23 + 1.5, Z_FLOOR - 1, zt + 1))
    t.append(cyl(6.0, (-HX - 1, 0, 28), (1, 0, 0), WALL + 2))                # Ø12 power switch
    t.append(cyl(3.0, (HX - WALL - 1, 0, 30), (1, 0, 0), WALL + 2))          # front: LED / spare
    return cut(p, *t)


def motor_cap():
    """Clamps an N20 into its box.  Right side (Y>0) as modelled; flip for left."""
    yi = HY - WALL
    x0, x1 = MOTOR_BOX_X
    z0 = Z_SPROCKET + N20_H / 2
    p = box(x0, x1, yi - N20_LEN - 1.0, yi - 0.2, z0, z0 + 2.5)
    p = fil(p, "|Z", 1.5)
    t = [cz(M2 / 2, xx, yy, z0 - 1, z0 + 4)
         for xx in (x0 + 2.25, x1 - 2.25) for yy in (yi - 5, yi - N20_LEN + 4)]
    return cut(p, *t)


def deck():
    z0 = Z_DECK
    p = fil(box(-HX, HX, -HY, HY, z0, z0 + DECK_T), "|Z", 4)
    off = SG_L / 2 - SG_SHAFT_FROM_END
    bx = X_SERVO - off
    tabs = [(bx - SG_TAB_DX / 2, 0.0), (bx + SG_TAB_DX / 2, 0.0)]
    bosses = [cz(3.2, x, y, z0 - 3, z0) for x, y in tabs]
    p = fuse(p, *bosses)
    t = [box(bx - SG_L / 2 - 0.3, bx + SG_L / 2 + 0.3, -SG_W / 2 - 0.3, SG_W / 2 + 0.3, z0 - 4, z0 + 4)]
    t += [cz(M2_INS / 2, x, y, z0 - 3.5, z0 + 4) for x, y in tabs]
    t += [cz(M3 / 2, x, y, z0 - 1, z0 + 4) for x, y in DECK_POSTS]
    t.append(box(bx - SG_L / 2 - 6, bx - SG_L / 2 - 2, -5, 5, z0 - 1, z0 + 4))     # servo/cam cable
    for x in (-55, -20, 15):                                                     # zip-tie / velcro slots
        for s in (-1, 1):
            t.append(box(x - 6, x + 6, s * 14 - 1.5, s * 14 + 1.5, z0 - 1, z0 + 4))
    for x in (-38, -2):                                                          # vents
        t.append(box(x - 8, x + 8, -6, 6, z0 - 1, z0 + 4))
    return cut(p, *t)


def sprocket():
    """Drive sprocket for N20 D-shaft.  Local: axis +Y, Y=0 at hull wall outer face."""
    yc = TRACK_YC - HY
    y0, y1 = yc - WHEEL_W / 2, yc + WHEEL_W / 2
    r_out = RP + 1.6
    disc = cy(r_out, 0, 0, y0, y1)
    hub = cy(5.0, 0, 0, 0.5, y0 + 0.01)
    p = fuse(disc, hub)
    t = []
    for k in range(N_TEETH):
        a = 2 * math.pi * k / N_TEETH
        cx_, cz_ = RP * math.cos(a), RP * math.sin(a)
        t.append(cy(KN_R + 0.25, cx_, cz_, y0 - 1, y1 + 1))
        slot = box(-(KN_R + 0.5), KN_R + 0.5, y0 - 1, y1 + 1, 0, r_out + 2)
        slot = slot.rotate(V(0, 0, 0), V(0, 1, 0), -math.degrees(a) + 90).translate(V(cx_, 0, cz_))
        t.append(slot)
    # D bore from the hull side (shaft enters at Y=0 side ... wall is at Y<0)
    shaft_end = N20_SHAFT - WALL - 0.2
    bore = cy(N20_D / 2, 0, 0, 0, shaft_end + 0.5).cut(box(-3, 3, -1, shaft_end + 2, N20_D / 2 - (N20_D - N20_FLAT), 3))
    t.append(bore)
    t.append(cyl(M2_INS / 2, (0, (0.5 + y0) / 2, 0), (0, 0, 1), 6))              # radial set-screw insert
    for k in range(5):                                                           # lightening
        a = 2 * math.pi * k / 5 + 0.3
        t.append(cy(2.6, 8.6 * math.cos(a), 8.6 * math.sin(a), y0 - 1, y1 + 1))
    return cut(p, *t)


def wheel(r_rim):
    """Road wheel / idler.  Local: axis +Y, Y=0 at hull wall outer face."""
    yc = TRACK_YC - HY
    y0, y1 = yc - WHEEL_W / 2 + 0.1, yc + WHEEL_W / 2 - 0.1
    p = cy(r_rim, 0, 0, y0, y1)
    t = [cy(2.75, 0, 0, y0 - 1, y1 + 1)]                                         # rides on sleeve OD 5.4
    for k in range(5):
        a = 2 * math.pi * k / 5
        rr = (r_rim + 2.75) / 2
        t.append(cy(min(2.6, (r_rim - 4.5) / 3), rr * math.cos(a), rr * math.sin(a), y0 - 1, y1 + 1))
    t.append(cy(r_rim - 1.2, 0, 0, y0 - 1, y0 + 1.2).cut(cy(4.2, 0, 0, y0 - 2, y0 + 2)))   # dish
    t.append(cy(r_rim - 1.2, 0, 0, y1 - 1.2, y1 + 1).cut(cy(4.2, 0, 0, y1 - 2, y1 + 2)))
    return cut(p, *t)


def road_wheel(): return wheel(RW_R)
def idler(): return wheel(RP - KN_Z)


def axle_sleeve():
    """Flanged bearing sleeve: wheel spins on it, countersunk M3x25 clamps it."""
    yc = TRACK_YC - HY
    yb0, yb1 = yc - WHEEL_W / 2, yc + WHEEL_W / 2 + 0.2
    p = fuse(cy(2.6, 0, 0, yb0, yb1), cy(4.4, 0, 0, yb1, yb1 + 1.6))
    return cut(p, cy(M3 / 2, 0, 0, yb0 - 1, yb1 + 3), csk_y(0, 0, yb1 + 1.6, -1))


def axle_spacer():
    """Spacer between hull wall and wheel (wheel inner face)."""
    yc = TRACK_YC - HY
    return cy(3.5, 0, 0, 0.2, yc - WHEEL_W / 2).cut(cy(M3 / 2, 0, 0, -1, 20))


SEGS = [(-9.0, -5.8), (-5.3, -2.1), (-1.6, 1.6), (2.1, 5.3), (5.8, 9.0)]


def track_link():
    """Local: pins at X=+-PITCH/2, Z=KN_Z, along Y.  Z=0 = ground side.
    Front end owns knuckles 0,2,4; rear end owns 1,3."""
    h = PITCH / 2
    plate = box(-h, h, -TRACK_W / 2, TRACK_W / 2, 0, PLATE_T)
    kn = []
    for i, (a, b) in enumerate(SEGS):
        x = h if i % 2 == 0 else -h
        kn.append(cy(KN_R, x, KN_Z, a, b))
        kn.append(box(min(0, x), max(0, x), a, b, 0, KN_Z))        # web to plate
    horns = [box(-1.6, 1.6, s * HORN_Y[0] if s > 0 else -HORN_Y[1],
                 s * HORN_Y[1] if s > 0 else -HORN_Y[0], PLATE_T - 0.01, PLATE_T + HORN_H) for s in (-1, 1)]
    horns = [fil(hh, "|Y", 0.6) for hh in horns]
    p = fuse(plate, *kn, *horns)
    t = []
    for i, (a, b) in enumerate(SEGS):                               # clearance for neighbour knuckles
        x = -h if i % 2 == 0 else h
        t.append(cy(KN_R + 0.45, x, KN_Z, a - 0.25, b + 0.25))
        t.append(box(x - (KN_R + 0.45), x + (KN_R + 0.45), a - 0.25, b + 0.25, KN_Z, 9))
    t.append(cy(PIN_D / 2, h, KN_Z, -TRACK_W, TRACK_W))
    t.append(cy(PIN_D / 2, -h, KN_Z, -TRACK_W, TRACK_W))
    t.append(box(-2.8, 2.8, -4.4, 4.4, -1, KN_Z + KN_R + 1))      # sprocket tooth window
    return cut(p, *t)


def camera_head():
    """Pan head on the servo horn, holds a camera board (CAM_W x CAM_H) tilted 10 deg."""
    base = cz(16, 0, 0, 0, 3)
    hb = CAM_T + 0.3
    tilt = 10
    up = fuse(box(-6, -3, -(CAM_W / 2 + 3), CAM_W / 2 + 3, 0, CAM_H * 0.75),      # back plate
              box(-3, 3.5, -(CAM_W / 2 + 3), -(CAM_W / 2 - 1.2), 0, CAM_H * 0.75),  # side rails
              box(-3, 3.5, CAM_W / 2 - 1.2, CAM_W / 2 + 3, 0, CAM_H * 0.75))
    up = up.cut(box(-3 + 1.2, -3 + 1.2 + hb, -(CAM_W / 2 + 0.2), CAM_W / 2 + 0.2, 1.5, CAM_H))  # board groove
    up = up.cut(box(-6.5, -2.5, -8, 8, 6, CAM_H * 0.75 - 4))                       # back cable window
    up = up.rotate(V(0, 0, 0), V(0, 1, 0), -tilt).translate(V(0, 0, 2.5))
    p = fuse(base, up).cut(box(-30, 30, -30, 30, -10, 0))
    # underside pocket for a single-arm SG90 horn + centre screw access
    horn = (cq.Workplane("XY").polyline([(0, 2.9), (-16, 2.1), (-16, -2.1), (0, -2.9)])
            .close().extrude(1.6).val())
    t = [horn, cz(3.9, 0, 0, -1, 1.6), cz(2.75, 0, 0, -1, 4)]
    t += [cz(0.75, -r, 0, -1, 3) for r in (8, 11)]
    return cut(p, *t)


# ============================ reference parts ===============================
def d_n20():
    yi = HY - WALL
    b = box(-N20_W / 2, N20_W / 2, yi - N20_LEN, yi, -N20_H / 2, N20_H / 2)
    return fuse(b, cy(1.5, 0, 0, yi, yi + N20_SHAFT))
def d_sg90():
    off = SG_L / 2 - SG_SHAFT_FROM_END
    b = box(-off - SG_L / 2, -off + SG_L / 2, -SG_W / 2, SG_W / 2, -SG_TAB_Z, SG_H - SG_TAB_Z)
    return fuse(b, box(-off - 16.1, -off + 16.1, -SG_W / 2, SG_W / 2, 0, 2.5), cz(2.4, 0, 0, 0, SG_HORN_TOP))
def d_battery(): return box(-38.5, 38.5, -20.5, 20.5, 0, 20)          # 2x18650 holder
def d_board(w, l, h=1.6): return box(-l / 2, l / 2, -w / 2, w / 2, 0, h)


# ============================== track path ==================================
def _hull_poly(x_idler):
    from scipy.spatial import ConvexHull
    circles = [(X_SPROCKET, Z_SPROCKET, RP)] + [(x, Z_ROAD, RW_R + KN_R) for x in X_ROAD] + \
              [(x_idler, Z_SPROCKET, RP)]
    pts = []
    for cx_, cz_, r in circles:
        a = np.linspace(0, 2 * np.pi, 4000, endpoint=False)
        pts += list(zip(cx_ + r * np.cos(a), cz_ + r * np.sin(a)))
    pts = np.array(pts); h = ConvexHull(pts)
    poly = pts[h.vertices]                                  # counter-clockwise in (x, z)
    i0 = int(np.argmin(poly[:, 0]))                         # start at rearmost point
    return np.roll(poly, -i0, axis=0), h.area


def _walk(poly, n, pitch):
    """Place n pins along the closed polyline with exact straight spacing = pitch."""
    pins = [poly[0].copy()]; idx = 0; m = len(poly)
    for _ in range(n - 1):
        p = pins[-1]
        while True:
            a, b = poly[idx % m], poly[(idx + 1) % m]
            if np.linalg.norm(b - p) >= pitch:              # crossing in this segment
                d = b - a; f = a - p
                A = d @ d; B = 2 * f @ d; Cc = f @ f - pitch ** 2
                t = (-B + math.sqrt(max(B * B - 4 * A * Cc, 0))) / (2 * A)
                q = a + t * d
                if t >= 0 and (len(pins) < 2 or (q - p) @ (p - pins[-2]) > -1e-9 or True):
                    pins.append(q); break
            idx += 1
    return np.array(pins)


def solve_idler(n=None, pitch=PITCH):
    """Idler X so that n links of exact pitch close the loop."""
    n = n or N_LINKS
    def err(x):
        poly, _ = _hull_poly(x)
        pins = _walk(poly, n, pitch)
        return np.linalg.norm(pins[-1] - pins[0]) - pitch
    lo, hi = 50.0, 70.0
    for _ in range(40):
        mid = (lo + hi) / 2
        if err(mid) > 0: hi = mid
        else: lo = mid
    return (lo + hi) / 2


X_IDLER = round(solve_idler(), 2)


def pins_on_path():
    poly, per = _hull_poly(X_IDLER)
    return _walk(poly, N_LINKS, PITCH), per


# ================================= BUILD ====================================
def build(pan=0.0):
    parts = dict(hull=hull(), motor_cap=motor_cap(), deck=deck(), sprocket=sprocket(),
                 road_wheel=road_wheel(), idler=idler(), axle_sleeve=axle_sleeve(),
                 axle_spacer=axle_spacer(), track_link=track_link(), camera_head=camera_head())
    qty = dict(motor_cap=2, sprocket=2, road_wheel=6, idler=2, axle_sleeve=8, axle_spacer=8,
               track_link=2 * N_LINKS)
    C = cq.Color
    body, dark, acc, trk = C(0.33, 0.38, 0.22), C(0.18, 0.18, 0.2), C(0.85, 0.55, 0.15), C(0.12, 0.12, 0.12)
    asm = cq.Assembly(name="MiniTank"); placed = []
    def put(n, s, loc, col, printed=True):
        asm.add(s, name=n, loc=loc, color=col); placed.append((n, s.moved(loc), col, printed))

    put("hull", parts["hull"], L(), body)
    put("deck", parts["deck"], L(), body)
    Rz = L(V(0, 0, 0), V(0, 0, 1), 180)
    pins, per = pins_on_path()
    for s, side in ((1, "L"), (-1, "R")):
        mir = (lambda shp: shp) if s > 0 else (lambda shp: shp.mirror("XZ"))
        put(f"motor_cap_{side}", mir(parts["motor_cap"]), L(), dark)
        put(f"N20_{side}", mir(d_n20()), L(V(X_SPROCKET, 0, Z_SPROCKET)), C(0.75, 0.75, 0.78), False)
        side_loc = (lambda x, z: L(V(x, HY, z))) if s > 0 else (lambda x, z: L(V(x, -HY, z)) * Rz)
        put(f"sprocket_{side}", parts["sprocket"], side_loc(X_SPROCKET, Z_SPROCKET), acc)
        for i, x in enumerate(X_ROAD):
            put(f"road_wheel_{side}{i}", parts["road_wheel"], side_loc(x, Z_ROAD), dark)
            put(f"sleeve_r_{side}{i}", parts["axle_sleeve"], side_loc(x, Z_ROAD), acc)
            put(f"spacer_r_{side}{i}", parts["axle_spacer"], side_loc(x, Z_ROAD), acc)
        put(f"idler_{side}", parts["idler"], side_loc(X_IDLER, Z_SPROCKET), dark)
        put(f"sleeve_i_{side}", parts["axle_sleeve"], side_loc(X_IDLER, Z_SPROCKET), acc)
        put(f"spacer_i_{side}", parts["axle_spacer"], side_loc(X_IDLER, Z_SPROCKET), acc)
        # track links
        yc = s * TRACK_YC
        cxz = pins.mean(axis=0)
        for k in range(N_LINKS):
            a, b = pins[k], pins[(k + 1) % N_LINKS]
            mid = (a + b) / 2
            d = (b - a) / np.linalg.norm(b - a)
            n = np.array([-d[1], d[0]])                      # left normal (path is CCW -> inward)
            if np.dot(cxz - mid, n) < 0: n = -n
            th = math.degrees(math.atan2(n[0], n[1]))
            loc = L(V(mid[0], yc, mid[1])) * L(V(0, 0, 0), V(0, 1, 0), th) * L(V(0, 0, -KN_Z))
            put(f"link_{side}{k:02d}", parts["track_link"], loc, trk)
    # electronics + servo + camera (reference)
    put("battery_2x18650", d_battery(), L(V(-6, 0, Z_FLOOR + FLOOR_T)), C(0.15, 0.35, 0.8), False)
    off = SG_L / 2 - SG_SHAFT_FROM_END
    put("SG90_pan", d_sg90(), L(V(X_SERVO, 0, Z_DECK + DECK_T)), C(0.1, 0.3, 0.8), False)
    head_z = Z_DECK + DECK_T + SG_HORN_TOP - 1.6
    put("camera_head", parts["camera_head"], L(V(X_SERVO, 0, head_z)) * L(V(0, 0, 0), V(0, 0, 1), pan), acc)
    put("camera_board", box(-3 + 1.35, -3 + 1.35 + CAM_T, -CAM_W / 2, CAM_W / 2, 1.5, 1.5 + CAM_H)
        .rotate(V(0, 0, 0), V(0, 1, 0), -10).translate(V(0, 0, 2.5)),
        L(V(X_SERVO, 0, head_z)) * L(V(0, 0, 0), V(0, 0, 1), pan), C(0.1, 0.45, 0.2), False)
    return parts, qty, asm, placed, per


PRINT_ROT = {"sprocket": (V(1, 0, 0), -90), "road_wheel": (V(1, 0, 0), 90), "idler": (V(1, 0, 0), 90),
             "axle_sleeve": (V(1, 0, 0), -90), "axle_spacer": (V(1, 0, 0), 90),
             "deck": (V(1, 0, 0), 180), "motor_cap": (V(1, 0, 0), 0)}


def export_all(out=OUT):
    os.makedirs(f"{out}/STL", exist_ok=True); os.makedirs(f"{out}/STEP", exist_ok=True)
    parts, qty, asm, placed, per = build()
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
    cq.exporters.export(cq.Workplane().add(pr), f"{out}/tank_full_assembly.stl", **tol)
    cq.exporters.export(cq.Workplane().add(al), f"{out}/tank_full_assembly_with_electronics.stl", **tol)
    asm.save(f"{out}/tank_full_assembly.step")
    return parts, placed, per


if __name__ == "__main__":
    _, _, per = export_all()
    print("done ->", OUT, " track path %.1f mm for %d links" % (per, N_LINKS))
