"""Refit a car's screw fuel cap after filling up: take it off the holder on the back of the open fuel door, start it square on the filler neck at the back of
the recessed pocket, its axis horizontal, turn it clockwise until it clicks, and push the sprung fuel door shut until it latches.

Frame: x forward, y left, z up (TOP: the arms' table top, 0.75 m above the ground). The station's table is cut back to a strip under the arm bases, and a
Volkswagen Golf VII stands parked beyond it, its right rear quarter towards the arms: the body side around the fuel flap at its real height above the ground,
its flat face at the pocket in the plane x = PX facing the robot. The pocket behind its fuel door holds a 44 mm filler neck whose axis is horizontal (along x).
The car (a fixed articulation): frame = panel, pocket, neck. Links: the fuel door, on a vertical hinge at the pocket's -y edge (joint zero is open, about 90 degrees;
its spring opens it; from flush its catch pushes back with up to 0.8 N m, and it latches when pushed 0.8 degrees past flush), with the cap holder on its back; and the cap, on six undriven joints
(three slides, a yaw, a tilt limited to +-80 degrees, its own turn), so until the thread catches it moves as a free body: gravity, the holder, the fingers and the neck act on it, nothing else.
The thread (code): when the cap's plug is 8 to 9 mm into the neck's bore, coaxial within 1.5 mm and square within 4 degrees, the joints are driven to the
neck's axis and the cap's travel into the neck follows its own clockwise turn at the pitch. When it seats against the neck's mouth, the ratchet clicks; past the
click the shell only slips. Backed off past the start, it is free again.

Debug:  python demokit.py task.py OUT --no-record --mode full,none,push_only
Final:  python demokit.py task.py OUT
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demokit as d  # noqa: E402
import mechkit as m  # noqa: E402

SCENE = "station"
INSTRUCTION = "Put the fuel cap back on: start it square on the filler neck, turn it clockwise until it clicks, push the fuel door shut until it latches, let go."
SUCCESS_HOLD_S = 3.0
SCORE_OVER = "end"
HOLD_STEPS = 60  # SUCCESS_HOLD_S at 20 Hz
TOP = d.TABLE_TOP

LOOKS = {
    "body": m.Look(m.rgb(0.62, 0.64, 0.67), metallic=0.6, roughness=0.35),
    "pocket": m.Look(m.rgb(0.10, 0.10, 0.11), roughness=0.85),
    "neck": m.Look(m.rgb(0.05, 0.05, 0.05), roughness=0.6),
    "cap": m.Look(m.rgb(0.06, 0.06, 0.07), roughness=0.55),
    "grip": m.Look(m.rgb(0.16, 0.16, 0.17), roughness=0.6),
    "inner": m.Look(m.rgb(0.20, 0.20, 0.21), roughness=0.8),
    "mark": m.Look(m.rgb(0.85, 0.85, 0.80), roughness=0.5),
    "glass": m.Look(m.rgb(0.04, 0.05, 0.06), metallic=0.2, roughness=0.08),
    "lamp": m.Look(m.rgb(0.55, 0.04, 0.04), roughness=0.2),
    "tyre": m.Look(m.rgb(0.03, 0.03, 0.03), roughness=0.9),
    "rim": m.Look(m.rgb(0.70, 0.71, 0.73), metallic=0.8, roughness=0.3),
    "under": m.Look(m.rgb(0.02, 0.02, 0.02), roughness=0.95),
    "table": m.Look(m.rgb(0.80, 0.68, 0.48), roughness=0.7),
}

# -- the body side at the flap, its pocket and the filler neck (frame coordinates: the world less TOP in z)
H_FLAP = 0.87  # m above the ground: the flap's centre on a Golf VII, measured on a side photograph scaled by the wheelbase (README)
PX = 0.37  # the body side's face at the pocket
PW = 0.075  # the colliders' depth behind it
YC, NZ = -0.03, H_FLAP - TOP  # the pocket's centre (y) and the neck's axis height above the arms' table top
NY = YC + 0.020  # the neck's axis (y): set in the pocket away from the door's hinge
DOOR_W, DOOR_H, DOOR_T = 0.232, 0.140, 0.003  # the Golf VII's flap, measured on the same photograph
OPEN_Y, OPEN_Z = 0.119, 0.0725  # the pocket opening's half sizes
HY = YC - 0.117  # the door's hinge line, at the opening's -y edge (towards the car's front)
FLOOR_X = PX + 0.068  # the pocket's floor
MOUTH_X = PX + 0.050  # the neck's mouth
NECK_RI, NECK_RO = 0.022, 0.026  # 44 mm (1.75 in) bore
LEAD = 0.004  # the mouth's lead-in
# -- the car around the pocket, heights H above the ground: its body side from the sill to the roof, the rear wheel in its arch, the rear lamp
GROUND = -TOP
WHEEL_Y, WHEEL_H, TYRE_R, TYRE_W = YC - 0.154, 0.30, 0.316, 0.205  # the rear wheel's centre 154 mm ahead of the flap's; 205/55 R16
ARCH_R, ARCH_BOX = 0.365, (0.42, 0.72)  # the wheel arch's radius; the patch around it (half width, top H)
SILL_H, BELT_H, GLASS_H, ROOF_H = 0.20, 0.98, 1.36, 1.44
FRONT_Y, REAR_Y, GLASS_Y = -1.30, YC + 0.39, YC - 0.23  # the section's front end, the car's rear end, the side glass's rear edge (the C-pillar behind it)
LAMP = (0.22, 0.86, 0.97)  # the rear lamp: from this y to the rear end, between these heights

# -- the cap: a 67 mm (2.625 in) shell with a raised grip bar, a 40 mm threaded plug behind it
CAP_R, SHELL_T = 0.0335, 0.020
PLUG_R, PLUG_L = 0.020, 0.016
BAR = (0.012, 0.014, 0.050)  # thickness, height (along the axis), length
CAP_MASS = 0.08
TIP = SHELL_T / 2 + PLUG_L  # the plug's end, behind the shell's centre
PITCH = 0.020  # m per turn: a multi-start thread's lead, about a third of a turn from the catch to the click
START = (0.008, 0.009)  # the plug's depth into the bore where the thread catches
TURNED = 0.001  # m: a start counts once turned on this far (about 18 degrees), not only set in place
SEAT = 0.0158  # the depth with the shell's back on the neck's mouth
RUN_T, CLICK_T = 0.15, 1.5  # N m: the turn's drag running, and rising over the last sixth of a turn to the click; past the click the ratchet slips (RUN_T either way)

# the cap as it starts, hung in the holder on the door's back: its outward face (and grip bar) towards +y, its plug against the door
CAP0 = np.array([PX - 0.125, HY + DOOR_T + TIP, NZ - 0.020])

# the door: a light spring holds it open; at flush its push latch's catch resists with up to LATCH_T until pushed to LATCH_AT, where it catches
SPRING = (0.06, 0.02, 0.2)  # N m/rad, N m s/rad, N m: the door's opening spring (stiffness, damping, its torque limit)
LATCH_T = 0.8  # N m about the hinge: 8 N at 10 cm from it, 3.4 N at the door's free edge
LATCH_AT = 90.8  # degrees shut: 0.8 past flush


def _ext_y(outline_xz, y0, y1) -> m.Mesh:
    """A convex prism over an outline in (x, z), from y0 to y1."""
    p = m.prism(np.asarray(outline_xz, float), y0, y1)
    return m.Mesh(p.v[:, [0, 2, 1]], p.f[:, ::-1])


def _ext_x(outline_yz, x0, x1) -> m.Mesh:
    """A convex prism over an outline in (y, z), from x0 to x1."""
    p = m.prism(np.asarray(outline_yz, float), x0, x1)
    return m.Mesh(p.v[:, [2, 0, 1]], p.f)


def _hull(points) -> m.Mesh:
    """The convex hull of points, as a mesh."""
    from scipy.spatial import ConvexHull
    pts = np.asarray(points, float)
    h = ConvexHull(pts)
    f = h.simplices.copy()
    c = pts.mean(axis=0)
    for k, tri in enumerate(f):  # outward winding
        n = np.cross(pts[tri[1]] - pts[tri[0]], pts[tri[2]] - pts[tri[0]])
        if n @ (pts[tri[0]] - c) < 0:
            f[k] = tri[::-1]
    return m.Mesh(pts, f)


def _sector(cy, cz, ri, ro, a0, a1):
    return [(cy + ri * math.cos(a0), cz + ri * math.sin(a0)), (cy + ro * math.cos(a0), cz + ro * math.sin(a0)),
            (cy + ro * math.cos(a1), cz + ro * math.sin(a1)), (cy + ri * math.cos(a1), cz + ri * math.sin(a1))]


def _box(lo, hi):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return tuple(hi - lo), tuple((hi + lo) / 2)


def _frame():
    parts, cols = [], []

    def add(name, lo, hi, look=None):
        size, c = _box(lo, hi)
        if look:
            parts.append((name, m.box(size, c), look))
        cols.append(m.col_box(f"{name}_c", size, c))

    z0, z1 = SILL_H + GROUND, BELT_H + GROUND  # the body side's colliders: plain boxes behind its face, the pocket left open
    add("left", (PX, YC + OPEN_Y, z0), (PX + PW, LAMP[0], z1))
    add("right", (PX, WHEEL_Y - ARCH_BOX[0], z0), (PX + PW, YC - OPEN_Y, z1))
    add("above", (PX, YC - OPEN_Y, NZ + OPEN_Z), (PX + PW, YC + OPEN_Y, z1))
    add("below", (PX, YC - OPEN_Y, z0), (PX + PW, YC + OPEN_Y, NZ - OPEN_Z))
    add("floor", (FLOOR_X, YC - OPEN_Y, NZ - OPEN_Z), (PX + PW, YC + OPEN_Y, NZ + OPEN_Z), "pocket")
    for k, (lo, hi) in enumerate([((PX + 0.002, YC - OPEN_Y, NZ - OPEN_Z), (FLOOR_X, YC + OPEN_Y, NZ - OPEN_Z + 0.002)),
                                  ((PX + 0.002, YC - OPEN_Y, NZ + OPEN_Z - 0.002), (FLOOR_X, YC + OPEN_Y, NZ + OPEN_Z)),
                                  ((PX + 0.002, YC - OPEN_Y, NZ - OPEN_Z), (FLOOR_X, YC - OPEN_Y + 0.002, NZ + OPEN_Z)),
                                  ((PX + 0.002, YC + OPEN_Y - 0.002, NZ - OPEN_Z), (FLOOR_X, YC + OPEN_Y, NZ + OPEN_Z))]):
        size, c = _box(lo, hi)
        parts.append((f"wall{k}", m.box(size, c), "pocket"))
    n = 16
    for k in range(n):
        a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        mesh = _ext_x(_sector(NY, NZ, NECK_RI, NECK_RO, a0, a1), MOUTH_X + LEAD, FLOOR_X)
        parts.append((f"neck{k}", mesh, "neck"))
        cols.append(m.col_hull(f"neck_c{k}", mesh))
        a = _sector(NY, NZ, NECK_RO - 0.0008, NECK_RO, a0, a1)  # the rolled lip: the bore flares from 22 mm to 25 mm radius over its first 4 mm
        b = _sector(NY, NZ, NECK_RI, NECK_RO, a0, a1)
        lip = m.Mesh(np.array([[MOUTH_X, y, z] for y, z in a] + [[MOUTH_X + LEAD, y, z] for y, z in b]), np.zeros((0, 3), int))
        lip = _hull(lip.v)
        parts.append((f"lip{k}", lip, "neck"))
        cols.append(m.col_hull(f"lip_c{k}", lip))
    return parts + _body(), cols


def _side_dx(y, h):
    """How far the body side lies in (+x) from the pocket's face plane at (y, H): flat at the flap, out a little over the wheel, in at the sill,
    the glass and the C-pillar leaning in above the belt line, and the rear corner turning in towards the tailgate."""
    if h <= BELT_H:
        dx = float(np.interp(h, [SILL_H, 0.28, 0.40, 0.55, 0.70, 0.80, 0.95, BELT_H], [0.07, 0.025, 0.005, -0.008, -0.006, 0.0, 0.0, 0.008]))
    else:
        dx = 0.008 + (h - BELT_H) * math.tan(math.radians(28.0))
    return dx + max(0.0, y - 0.16) ** 2 / 0.44


def _surface(points_yh, faces, look, name):
    """A sheet through (y, H) points on the body side, facing the robot (-x)."""
    v = np.array([[PX + _side_dx(y, h), y, h + GROUND] for y, h in points_yh])
    f = np.array(faces, int)
    n = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    f[n[:, 0] > 0] = f[n[:, 0] > 0][:, ::-1]
    return (name, m.Mesh(v, f), look)


def _body():
    """The Golf VII's body side around the pocket, as visual sheets (its colliders are the boxes in _frame): the panel from the sill to the belt
    line with the pocket's opening and the wheel arch cut out, the side glass and C-pillar to the roof, the rear lamp, the rear wheel and the underbody."""
    ax, at = ARCH_BOX
    ys = {FRONT_Y, REAR_Y, GLASS_Y, LAMP[0], WHEEL_Y - ax, WHEEL_Y + ax, YC - OPEN_Y, YC + OPEN_Y, *np.arange(FRONT_Y, REAR_Y, 0.04)}
    ys |= set(np.arange(0.16, REAR_Y, 0.02))
    hs = {SILL_H, at, H_FLAP - OPEN_Z, H_FLAP + OPEN_Z, LAMP[1], LAMP[2], BELT_H, GLASS_H, ROOF_H, *np.arange(SILL_H, ROOF_H, 0.03)}
    ys = sorted(y for y in ys if FRONT_Y <= y <= REAR_Y)
    hs = sorted(h for h in hs if SILL_H <= h <= ROOF_H)
    cells = {"body": ([], []), "glass": ([], []), "lamp": ([], [])}
    for i in range(len(ys) - 1):
        for j in range(len(hs) - 1):
            yc, hc = (ys[i] + ys[i + 1]) / 2, (hs[j] + hs[j + 1]) / 2
            if abs(yc - YC) < OPEN_Y and abs(hc - H_FLAP) < OPEN_Z:
                continue  # the pocket's opening
            if abs(yc - WHEEL_Y) < ax and hc < at:
                continue  # the arch's patch, below
            look = "glass" if BELT_H < hc < GLASS_H and yc < GLASS_Y else "lamp" if yc > LAMP[0] and LAMP[1] < hc < LAMP[2] else "body"
            pts, fs = cells[look]
            k = len(pts)
            pts += [(ys[i], hs[j]), (ys[i + 1], hs[j]), (ys[i + 1], hs[j + 1]), (ys[i], hs[j + 1])]
            fs += [(k, k + 1, k + 2), (k, k + 2, k + 3)]
    parts = [_surface(pts, fs, look, f"side_{look}") for look, (pts, fs) in cells.items()]
    # the arch's patch: rays from the wheel's centre, from the arch's edge out to the patch's sides and top
    lo = math.asin((SILL_H - WHEEL_H) / ARCH_R)
    corner = math.atan2(at - WHEEL_H, ax)
    angles = sorted(set(np.linspace(lo, math.pi - lo, 41)) | {corner, math.pi - corner})
    pts, fs = [], []
    for a in angles:
        c, s_ = math.cos(a), math.sin(a)
        t = min(ax / abs(c) if abs(c) > 1e-9 else 9.0, (at - WHEEL_H) / s_ if s_ > 1e-9 else 9.0)
        pts += [(WHEEL_Y + ARCH_R * c, WHEEL_H + ARCH_R * s_), (WHEEL_Y + t * c, max(SILL_H, WHEEL_H + t * s_))]
    for k in range(len(angles) - 1):
        a, b = 2 * k, 2 * k + 2
        fs += [(a, b, b + 1), (a, b + 1, a + 1)]
    parts.append(_surface(pts, fs, "body", "side_arch"))
    xw = PX + 0.03 + TYRE_W / 2
    parts += [("tyre", m.cylinder(TYRE_R, TYRE_W, (xw, WHEEL_Y, WHEEL_H + GROUND), axis="x", segments=48), "tyre"),
              ("rim", m.cylinder(0.20, 0.012, (PX + 0.026, WHEEL_Y, WHEEL_H + GROUND), axis="x", segments=48), "rim"),
              ("well", m.cylinder(ARCH_R, 0.01, (PX + 0.26, WHEEL_Y, WHEEL_H + GROUND), axis="x", segments=48), "under"),
              ("underbody", m.box((0.8, REAR_Y - 0.10 - FRONT_Y, 0.08), (PX + 0.47, (FRONT_Y + REAR_Y - 0.10) / 2, 0.15 + GROUND)), "under"),
              ("roof", m.box((0.70, REAR_Y - 0.05 - FRONT_Y, 0.02), (PX + _side_dx(0.0, ROOF_H) + 0.35, (FRONT_Y + REAR_Y - 0.05) / 2, ROOF_H + GROUND)), "body"),
              ("handle", m.box((0.018, 0.13, 0.028), (PX - 0.006, -0.62, 0.935 + GROUND)), "body")]
    return parts


def _arms_table():
    """The station's table cut back to a strip under the two arm bases, 0.75 m high, its front edge 0.23 m from the car's side."""
    top_t, x0, x1, yh, leg = 0.06, -0.19, 0.14, 0.65, 0.05
    parts = [("top", m.box((x1 - x0, 2 * yh, top_t), ((x0 + x1) / 2, 0.0, TOP - top_t / 2)), "table")]
    cols = [m.col_box("top_c", (x1 - x0, 2 * yh, top_t), ((x0 + x1) / 2, 0.0, TOP - top_t / 2))]
    for k, (x, y) in enumerate([(x0 + leg / 2, yh - leg / 2), (x0 + leg / 2, -yh + leg / 2), (x1 - leg / 2, yh - leg / 2), (x1 - leg / 2, -yh + leg / 2)]):
        size, c = (leg, leg, TOP - top_t), (x, y, (TOP - top_t) / 2)
        parts.append((f"leg{k}", m.box(size, c), "table"))
        cols.append(m.col_box(f"leg_c{k}", size, c))
    return m.fixed("arms_table", (0.0, 0.0, 0.0), parts, cols, LOOKS)


def _no_station_table(scene) -> None:
    """Take the station's own table out of the room (the arms keep their poses, on the cut-back table above)."""
    if getattr(scene, "_no_station_table", False):
        return
    scene._no_station_table = True
    room = scene.task_room

    def task_room(*a, **k):
        assets = room(*a, **k)
        if (a[0] if a else k.get("scene")) == "station":
            if sum(getattr(x, "name", None) == "table" for x in assets) != 1:
                raise RuntimeError("expected one station table")
            return [x for x in assets if getattr(x, "name", None) != "table"]
        return assets

    scene.task_room = task_room


if "yam_arena.scene" in sys.modules:
    _no_station_table(sys.modules["yam_arena.scene"])
_patch0 = m._Finder.targets["yam_arena.scene"]
m._Finder.targets["yam_arena.scene"] = lambda mod: (_patch0(mod), _no_station_table(mod))


def _door():
    """The fuel door at joint zero (open, standing out towards the robot from its hinge), with the cap holder low on its back (+y face):
    shut, the holder sits in the pocket below the fitted cap. Shutting turns the joint negative."""
    x0, x1 = PX - 0.004 - DOOR_W, PX - 0.004
    c = ((x0 + x1) / 2, HY + DOOR_T / 2, NZ)
    parts = [("door", m.box((x1 - x0, DOOR_T, DOOR_H), c), "body")]
    cols = [m.col_box("door_c", (x1 - x0, DOOR_T, DOOR_H), c)]
    cx, cz = CAP0[0], CAP0[2]
    yb, yf = HY + DOOR_T, HY + DOOR_T + 0.045  # the holder stands 45 mm off the door's back
    phi = math.radians(40.0)
    zb = cz - CAP_R - 0.004
    for s in (-1.0, 1.0):  # a V cradle under the shell's rim
        u = np.array([s * math.sin(phi), -math.cos(phi)])
        v = np.array([math.cos(phi), s * math.sin(phi)]) * s
        T = np.array([cx, cz]) + (CAP_R + 0.0005) * u
        p1, p2 = T - 0.010 * v, T + 0.010 * v
        if p1[1] > p2[1]:
            p1, p2 = p2, p1
        outline = [tuple(p1), tuple(p2), (p2[0], zb), (p1[0], zb)]
        mesh = _ext_y(outline, yb, yf)
        parts.append((f"cradle{int(s > 0)}", mesh, "inner"))
        cols.append(m.col_hull(f"cradle_c{int(s > 0)}", mesh))
    size, c = _box((cx - 0.034, yb, zb - 0.004), (cx + 0.034, yf, zb))
    parts.append(("cradle_base", m.box(size, c), "inner"))
    cols.append(m.col_box("cradle_base_c", size, c))
    size, c = _box((cx - 0.030, yf, zb - 0.004), (cx + 0.030, yf + 0.004, cz - CAP_R + 0.006))
    parts.append(("cradle_lip", m.box(size, c), "inner"))
    cols.append(m.col_box("cradle_lip_c", size, c))
    return m.hinge("door", "Z", (PX, HY, NZ), -92.0, 3.0, parts, cols, spring=SPRING[0], damping=SPRING[1], friction=0.003, effort=SPRING[2], mass=0.15, armature=1e-4)


def _cap_chain():
    at = tuple(CAP0)
    shell = m.cylinder(CAP_R, SHELL_T, at, axis="y", segments=40)
    y0, y1 = SHELL_T / 2, SHELL_T / 2 + PLUG_L
    plug = m.lathe([(0.0, y0), (PLUG_R, y0), (PLUG_R, y1 - 0.004), (PLUG_R - 0.004, y1), (0.0, y1)], 32).turned(90.0, "x").moved(CAP0)  # a chamfered end, towards -y
    bar_c = tuple(CAP0 + [0.0, SHELL_T / 2 + BAR[1] / 2, 0.0])
    bar = m.box((BAR[0], BAR[1], BAR[2]), bar_c)
    ring = m.cylinder(CAP_R - 0.003, 0.001, tuple(CAP0 + [0.0, SHELL_T / 2 + 0.0005, 0.0]), axis="y", segments=40)
    parts = [("shell", shell, "cap"), ("plug", plug, "grip"), ("bar", bar, "grip"), ("face", ring, "inner")]
    cols = [m.col_hull("shell_c", shell), m.col_hull("plug_c", plug), m.col_box("bar_c", (BAR[0], BAR[1], BAR[2]), bar_c)]
    free = dict(spring=0.0, damping=0.0, friction=0.0, effort=0.0, armature=1e-5, mass=0.002, inertia=(1e-7, 1e-7, 1e-7))
    return [m.slider("sx", "X", -0.6, 0.6, [], [], at=at, **free),
            m.slider("sy", "Y", -0.6, 0.6, [], [], at=at, parent="sx", **free),
            m.slider("sz", "Z", -0.6, 0.6, [], [], at=at, parent="sy", **free),
            m.hinge("yaw", "Z", at, -200.0, 200.0, [], [], parent="sz", **free),
            m.hinge("tilt", "X", at, -80.0, 80.0, [], [], parent="yaw", **free),
            m.hinge("cap", "Y", at, -math.inf, math.inf, parts, cols, parent="tilt", spring=0.0, damping=0.0, friction=0.0, effort=0.0, armature=1e-6,
                    mass=CAP_MASS, gravity=True, com=tuple(CAP0 + [0.0, -0.003, 0.0]), inertia=(2.6e-5, 4.5e-5, 2.6e-5))]


_fp, _fc = _frame()
CAR = m.mechanism("car", (0.0, 0.0, TOP), [_door(), *_cap_chain()], LOOKS, frame_parts=_fp, frame_colliders=_fc, friction=(0.8, 0.7), solver_iters=64,
                  self_collisions=True)
OBJECTS = [CAR, _arms_table()]


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def cap_local(st) -> tuple[np.ndarray, np.ndarray]:
    """The cap's shell centre and its outward axis (from the plug to the grip bar), in the car's frame, from the six joints."""
    c = CAP0 + np.array([st.joint("sx"), st.joint("sy"), st.joint("sz")])
    R = _rz(st.joint("yaw")) @ _rx(st.joint("tilt"))
    return c, R @ np.array([0.0, 1.0, 0.0])


def cap_fit(c, out) -> tuple[float, float, float]:
    """(depth of the plug's end into the bore, its centre's distance off the neck's axis, its axis's angle from the neck's)."""
    tip = c - out * TIP
    depth = float(tip[0] - MOUTH_X)
    radial = float(math.hypot(c[1] - NY, c[2] - NZ))
    square = math.degrees(math.acos(float(np.clip(-out[0], -1.0, 1.0))))
    return depth, radial, square


# -- the thread and the ratchet, as code: it catches a cap put square on its start; then the cap's own turn drives its travel to the click
class Thread:
    def reset(self, env=None):
        self.on = False
        self.clicked = False
        self.x0 = 0.0  # the slide sx where the thread caught
        self.turn0 = 0.0
        self.run = 0.0  # travel into the neck since the catch
        self.room = 0.0  # travel from the catch to the seat
        self.gains = {}  # the drive gains last written, per joint
        if env is not None:
            self._free(env)

    def _set(self, env, link, k, c, effort, target=None):
        asset, j = m._joint(env, "car", link)
        m._TOUCHED.add(("car", link))
        if self.gains.get(link) != (k, c, effort):  # write the gains only when they change
            self.gains[link] = (k, c, effort)
            asset.write_joint_stiffness_to_sim(m._tensor(asset, k), joint_ids=[j])
            asset.write_joint_damping_to_sim(m._tensor(asset, c), joint_ids=[j])
            asset.write_joint_effort_limit_to_sim(m._tensor(asset, effort), joint_ids=[j])
        if target is not None:
            asset.set_joint_position_target(m._tensor(asset, target), joint_ids=[j])

    def _free(self, env):
        for link in ("sx", "sy", "sz", "yaw", "tilt", "cap"):
            self._set(env, link, 0.0, 0.0, 0.0)

    def step(self, env):
        st = m.state(env, "car")
        c, out = cap_local(st)
        turn = -st.joint("cap")  # clockwise seen from outside: on the neck the spin joint's axis points out of it
        if not self.on:
            depth, radial, square = cap_fit(c, out)
            if START[0] <= depth <= START[1] and radial <= 0.0015 and square <= 4.0:
                self.on, self.clicked, self.x0, self.turn0 = True, False, st.joint("sx"), turn
                self.room = SEAT - depth - 0.0003
                self._set(env, "sy", 3000.0, 30.0, 40.0, NY - CAP0[1])  # the bore holds the plug on its axis; a few mm of play give a few N
                self._set(env, "sz", 3000.0, 30.0, 40.0, NZ - CAP0[2])
                self._set(env, "yaw", 200.0, 0.5, 20.0, math.pi / 2)
                self._set(env, "tilt", 200.0, 0.5, 20.0, 0.0)
            else:
                return
        self.run = PITCH * (turn - self.turn0) / (2 * math.pi)
        if self.run < -0.0008:  # backed off the start: free again
            self.on, self.clicked, self.run = False, False, 0.0
            self._free(env)
            return
        if self.run > self.room:  # seated: turned on, the ratchet slips and the cap goes no further
            self.turn0 = turn - self.room * 2 * math.pi / PITCH
            self.run = self.room
        self.clicked = self.run >= self.room - 0.0003  # turned back more than about 5 degrees, it is off the click again
        left = self.room - max(self.run, 0.0)
        if self.clicked:  # past the click the ratchet slips freely and turns back freely: only the running drag
            drag = RUN_T
        else:
            drag = RUN_T + (CLICK_T - RUN_T) * float(np.clip(1.0 - left / (PITCH / 6), 0.0, 1.0))
        self._set(env, "sx", 5000.0, 30.0, 100.0, self.x0 + float(np.clip(self.run, 0.0, self.room)))
        self._set(env, "cap", 0.0, 20.0, drag)  # a velocity drive capped at the drag: the turn needs that torque from the hand


THREAD = Thread()
m.on_step(THREAD.step)
m.on_reset(THREAD.reset)


class DoorLatch:
    """The door's push latch: from flush, its catch pushes back with up to LATCH_T (a stiff drive held at flush, its torque capped); pushed to LATCH_AT
    against it, the catch takes the door and holds it flush. Let go short of that, it rests at flush, unlatched; turned back below flush, the spring opens it again."""

    def reset(self, env=None):
        self.on = False
        self.catch = False  # the catch pushing back

    def _drive(self, env, k, c, effort, target):
        asset, j = m._joint(env, "car", "door")
        m._TOUCHED.add(("car", "door"))
        asset.write_joint_stiffness_to_sim(m._tensor(asset, k), joint_ids=[j])
        asset.write_joint_damping_to_sim(m._tensor(asset, c), joint_ids=[j])
        asset.write_joint_effort_limit_to_sim(m._tensor(asset, effort), joint_ids=[j])
        asset.set_joint_position_target(m._tensor(asset, target), joint_ids=[j])

    def step(self, env):
        if self.on:
            return
        shut = -math.degrees(m.state(env, "car").joint("door"))
        if shut >= LATCH_AT:  # pushed past the catch's travel: latched
            self.on = True
            self._drive(env, 2000.0, 5.0, 50.0, math.radians(-90.0))
        elif shut >= 90.0 and not self.catch:  # at flush: the catch pushes back
            self.catch = True
            self._drive(env, 200.0, 0.5, LATCH_T, math.radians(-90.0))
        elif shut < 89.9 and self.catch:  # off the catch: the spring again
            self.catch = False
            self._drive(env, SPRING[0], SPRING[1], SPRING[2], 0.0)


LATCH = DoorLatch()
m.on_step(LATCH.step)
m.on_reset(LATCH.reset)

# -- the scorer


def gripper_points(env) -> np.ndarray:
    """Points over the whole of both grippers (fingers, palm, housing, and the wrist link's end), station frame: a grid on each, at the current opening."""
    base = env.unwrapped
    origin = m._np(base.scene.env_origins[0])
    out = []
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        i = robot.find_bodies("gripper")[0][0]
        pose = m._np(robot.data.body_link_pose_w)[0, i]
        R = m.quat_matrix(pose[3:7])
        opening = abs(float(m._np(robot.data.joint_pos)[0, robot.find_joints("joint7")[0][0]]))
        pts = []
        for sx in (-1.0, 1.0):  # each finger: a 5 x 4 x 2 grid over its plate
            for dy in np.linspace(-0.02, 0.02, 5):
                for dz in np.linspace(0.095, 0.152, 4):
                    for t in (0.0, 0.009):
                        pts.append((sx * (opening + 0.0045 + t), dy, dz))
        for x in np.linspace(-0.05, 0.05, 6):  # the palm and housing: a box about 10 x 6 x 10 cm behind the fingers
            for y in np.linspace(-0.03, 0.03, 4):
                for z in np.linspace(-0.01, 0.095, 6):
                    pts.append((x, y, z))
        out.append(pose[:3] - origin + np.asarray(pts) @ R.T)
    return np.vstack(out)


def door_box(st):
    """The door's plate as (hinge point, its rotation, its box in the door's own frame at joint zero), car frame."""
    return np.array([PX, HY, NZ]), _rz(st.joint("door"))


def now(env, hands: bool = True) -> dict:
    st = m.state(env, "car")
    c, out = cap_local(st)
    depth, radial, square = cap_fit(c, out)
    if not hands:  # not needed for the score now
        return {"depth": depth, "radial": radial, "square": square, "z": float(c[2]), "on": THREAD.on, "run": float(THREAD.run), "clicked": THREAD.clicked,
                "door": -math.degrees(st.joint("door")), "latched": LATCH.on, "hands": 0.0, "c": c}
    pts = st.to_local(gripper_points(env))
    # the cap: a cylinder from the plug's end to the bar's front, its shell's radius
    a, b = c - out * TIP, c + out * (SHELL_T / 2 + BAR[1])
    ab = b - a
    t = np.clip(((pts - a) @ ab) / (ab @ ab), 0.0, 1.0)
    d_cap = float((np.linalg.norm(pts - (a + t[:, None] * ab), axis=1) - CAP_R).min())
    # the door with its holder: a box in its own frame
    h, R = door_box(st)
    loc = (pts - h) @ R  # into the door's frame at joint zero
    lo = np.array([-0.004 - DOOR_W, 0.0, -DOOR_H / 2])
    hi = np.array([-0.004, DOOR_T + 0.050, DOOR_H / 2])
    d_door = float(np.linalg.norm(np.maximum(0.0, np.maximum(lo - loc, loc - hi)), axis=1).min())
    return {"depth": depth, "radial": radial, "square": square, "z": float(c[2]), "on": THREAD.on, "run": float(THREAD.run), "clicked": THREAD.clicked,
            "door": -math.degrees(st.joint("door")), "latched": LATCH.on, "hands": min(d_cap, d_door), "c": c}


def judge(c: dict, dropped: bool, held_steps: int = HOLD_STEPS) -> tuple[bool, float]:
    """The score now: (full, score). 0 if the cap was dropped: its centre fell more than 0.10 m below the pocket's centre (NZ - 0.10, 0.77 m above the ground; no table lies under the car side), latched for the rest of the run.
    0.25 started: the thread caught (the plug 8 to 9 mm into the bore, coaxial within 1.5 mm, square within 4 degrees) and turned on at least 1 mm.
    0.5 threaded: caught, and turned at least a quarter turn on (5 mm of travel).
    0.75 clicked: turned on to the seat and the ratchet's click.
    1.0 shut: clicked, the fuel door latched (and within 2 degrees of flush), and both whole grippers at least 2 cm from the cap and the door, held 3 s."""
    if dropped or not c["on"]:
        return False, 0.0
    if not c["clicked"]:
        return False, 0.5 if c["run"] >= PITCH / 4 else 0.25 if c["run"] >= TURNED else 0.0
    if c["latched"] and c["door"] >= 88.0 and c["hands"] >= 0.02:
        return True, 1.0 if held_steps >= HOLD_STEPS else 0.75
    return False, 0.75


class Trial:
    def reset(self, env=None):
        self.dropped = False
        self.held = 0
        self.full, self.score = False, 0.0

    def step(self, env):
        c = now(env, hands=THREAD.clicked and LATCH.on)
        if c["z"] < NZ - 0.10:  # fell below the pocket
            self.dropped = True
        self.full, self.score = judge(c, self.dropped, self.held + 1)
        self.held = self.held + 1 if self.full else 0


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.full


def score(objects) -> float:
    return TRIAL.score


# -- the demo: reads the cap's pose (the cameras show it), the door's angle and the hands' own poses
GC = d.level(-90.0)  # the cap hand (left) points -y, closing along x: across the grip bar as the cap hangs
GD = d.tilted(60.0, 0.0)  # the door hand (right) is tilted 60 degrees from straight down towards +x, so its fingers point +x, 30 degrees below level: the flap sits low beside the arm's base
AX = (1.0, 0.0, 0.0)
AY = (0.0, 1.0, 0.0)
AZ = (0.0, 0.0, 1.0)
OFF = np.array([0.0, 0.0, TOP])
ROLL_MIN = -115.0  # degrees on the wrist's roll joint (its limit is -120) before the one long turn
STEP = 18.0  # degrees a sub-step of the turn


def demo_script(ctx):
    """Modes: full; none (the arms wait); push_only (the cap is pushed onto the neck square and let go, unturned, then the door is shut);
    no_door (the cap is fitted to the click, the door left open); undo (fitted to the click, turned back 30 degrees, let go, the door shut); door_only (the door is shut with the cap left in its holder); drop (the cap is lifted and let go);
    light_door (as full, but the last push into the door's latch is 2 N); weak_wrist (as full, with the wrist roll's effort limit written down to half the click torque while it turns)."""
    mode = ctx.mode.split("@")[0]
    if mode not in ("full", "none", "push_only", "no_door", "undo", "door_only", "drop", "light_door", "weak_wrist"):
        raise ValueError(f"unknown mode {mode!r}")
    C, D = ctx.left, ctx.right  # C fits the cap, D shuts the door
    env = ctx.emb._env
    dry = getattr(env, "unwrapped", None) is None  # a path check with no simulator: no readings of contact
    if mode == "none":
        yield from C.wait(3.0)
        return

    def car():
        return m.state(env, "car")

    def cap_c():
        if dry and held:  # with no simulator the cap's joints stay put: it rides in the hand
            return C.at() + C.approach_axis() * (SHELL_T / 2 + 0.017)
        return cap_local(car())[0] + OFF

    def cap_out():
        if dry and held:
            return -C.approach_axis()
        return cap_local(car())[1]

    def info(tag):
        if dry:
            return
        c = now(env)
        print(f"[demo] {tag}: cap {np.round(c['c'] + OFF, 4)} depth {c['depth']*1000:.1f} radial {c['radial']*1000:.2f} square {c['square']:.1f} on {c['on']} "
              f"run {c['run']*1000:.2f} clicked {c['clicked']} door {c['door']:.1f} latched {c['latched']} hands {c['hands']*1000:.0f} score {TRIAL.score}", flush=True)

    held = False
    if mode != "door_only":
        bar = lambda: cap_c() + cap_out() * (SHELL_T / 2 + 0.017)  # the TCP with the fingertips 2 mm off the shell's face
        yield from C.move(bar() + cap_out() * 0.07, grip=GC, finger=d.FINGER_OPEN, label="before the cap")
        yield from C.move(bar(), t=1.0, label="round the grip bar")
        yield from C.servo(bar(), timeout=1.0, label="round the grip bar")
        yield from C.close(BAR[0], force=30.0, label="grip the bar")
        held = True
        yield from C.move(C.at() + [0.0, 0.0, 0.020], t=0.6, label="lift out of the holder")
        yield from C.move(C.at() + [0.0, 0.07, 0.0], t=0.8, label="off the door")
        info("off the holder")
        if mode == "drop":
            yield from C.move(C.at() + [0.0, 0.0, 0.05], t=0.6)
            yield from C.open()
            yield from C.wait(2.0)
            info("dropped")
            return
        yield from C.turn(90.0, axis=AZ, t=1.0, label="face the neck")
        roll = math.degrees(float(C.q[5]))  # the wrist roll joint, as commanded
        if mode != "push_only":
            yield from C.turn(ROLL_MIN - roll, axis=AX, t=1.0, label="wind the wrist back for one long turn")
        neck = np.array([MOUTH_X, NY, NZ]) + OFF
        pre = neck + [-(TIP + 0.025), 0.0, 0.0]
        yield from C.move(C.at() + (pre - cap_c()), t=1.6, label="before the neck")
        yield from C.servo(pre, read=cap_c, tol=0.0005, timeout=2.0, label="on the axis")

        def square_up():
            o = cap_out()
            yield from C.turn(-math.degrees(math.atan2(-o[1], -o[0])), axis=AZ, t=0.3, label="square, yaw")
            o = cap_out()
            yield from C.turn(-math.degrees(math.atan2(o[2], -o[0])), axis=AY, t=0.3, label="square, pitch")

        for k in range(3):
            yield from square_up()
            yield from C.servo(pre, read=cap_c, tol=0.0004, timeout=1.5, label="on the axis")
            info("square")
            start = neck + [0.0088 - TIP, 0.0, 0.0]
            yield from C.move(C.at() + (start - cap_c()), t=1.0, label="plug into the bore")
            yield from C.servo(start, read=cap_c, tol=0.0004, timeout=1.5, label="at the thread")
            info("at the thread")
            if dry or THREAD.on:
                break
            res = yield from C.press(AX, 5.0, 0.004, label="press on")
            info(f"pressed ({res['why']})")
            if THREAD.on:
                break
            yield from C.move(C.at() + [-0.02, 0.0, 0.0], t=0.5, label="back off")
        if mode == "weak_wrist" and not dry:  # the roll joint can give only half the click torque
            wrist = env.unwrapped.scene["left_robot"]
            jw = wrist.find_joints("joint6")[0][0]
            lim = wrist.data.joint_effort_limits
            was = float(getattr(lim, "torch", lim)[0, jw])
            wrist.write_joint_effort_limit_to_sim(m._tensor(wrist, 0.5 * CLICK_T), joint_ids=[jw])
        if mode != "push_only":
            for k in range(13):  # one long turn, clockwise seen from outside, the hand following the thread in, until it clicks
                yield from C.turn(STEP, axis=AX, t=0.2, label="turn")
                yield from C.move(C.at() + [PITCH * STEP / 360, 0.0, 0.0], t=0.05, label="follow")
                q = C.q
                if dry:
                    continue
                print(f"[demo] turn {k}: roll {math.degrees(q[5]):.1f} spin {-math.degrees(car().joint('cap')):.1f} run {THREAD.run*1000:.2f} on {THREAD.on} "
                      f"tcp {np.round(C.tcp(), 4)} cap {np.round(cap_c(), 4)}", flush=True)
                if not dry and (THREAD.clicked or not THREAD.on):
                    break
            info("turned")
            if mode == "undo":
                for k in range(3):
                    yield from C.turn(-10.0, axis=AX, t=0.2, label="turn back")
                    yield from C.move(C.at() + [-PITCH * 10 / 360, 0.0, 0.0], t=0.05, label="follow out")
                info("turned back")
        yield from C.close(0.040, force=0.0, t=0.4, label="let go")
        yield from C.move(C.at() + [-0.10, 0.0, 0.0], t=1.0, label="out of the pocket")
        yield from C.move(C.at() + [-0.05, 0.12, 0.0], t=0.8, label="away")
        info("cap done")
        if mode == "weak_wrist" and not dry:  # the wrist limit back, with the cap let go
            wrist.write_joint_effort_limit_to_sim(m._tensor(wrist, was), joint_ids=[jw])
        if mode == "no_door":
            yield from C.wait(4.0)
            info("end")
            return
    # the right hand: push the door round along its swing to flush, reading its angle, then press it past flush into its latch
    hinge = np.array([PX, HY, NZ]) + OFF
    r = 0.10

    def tcp_at(th):  # th: the door's angle shut from open, positive
        u = np.array([-math.cos(th), math.sin(th), 0.0])
        n = np.array([-math.sin(th), -math.cos(th), 0.0])
        return hinge + r * u + n * (0.012 + 0.006 * math.sin(th))

    shut = lambda: -car().joint("door")
    th = shut()
    n0 = np.array([-math.sin(th), -math.cos(th), 0.0])
    yield from D.move(tcp_at(th) + n0 * 0.05, grip=GD, finger=d.FINGER_CLOSED, label="beside the door")
    yield from D.move(tcp_at(th) + n0 * 0.01, t=0.8, label="to the door")
    lead = math.radians(12.0)
    for k in range(30):
        th = shut() if not dry else min(0.12 * k, math.radians(90.0))  # with no simulator, as if the door followed
        if th >= math.radians(89.0):
            break
        yield from D.move(tcp_at(min(th + lead, math.radians(93.0))), t=0.35, label="push the door")
        if not dry:
            print(f"[demo] door push {k}: {math.degrees(th):.1f} -> {math.degrees(shut()):.1f} lead {math.degrees(lead):.0f} tcp {np.round(D.tcp(), 3)}", flush=True)
        if shut() < th + math.radians(3.0):  # not following: push further into it
            lead = min(lead + math.radians(6.0), math.radians(30.0))
    yield from D.move(tcp_at(math.radians(90.0)), t=0.3, label="on the shut door")
    if not dry:  # into the latch: a push along the shut door's normal, by force
        force = 2.0 if mode == "light_door" else 12.0
        res = yield from D.press(AX, force, 0.020, settle=0.6, timeout=4.0, label="into the latch")
        print(f"[demo] latch push {force:.0f} N: {res} door {math.degrees(shut()):.2f} latched {LATCH.on}", flush=True)
    info("door")
    yield from D.move(D.at() + [-0.06, 0.0, 0.0], t=0.6, label="off the door")
    yield from D.move(D.at() + [-0.04, -0.10, 0.04], t=0.8, label="away")
    yield from D.wait(3.6)
    info("end")
