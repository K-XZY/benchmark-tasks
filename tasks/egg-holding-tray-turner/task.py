"""Serve one round egg patty out of a covered holding pan onto a muffin heel with a turner: one arm lifts the cover off by its finger wells and holds it clear, the other
brings the turner's blade in over the near wall tipped steep, its tip on the floor in the 68 mm of flat floor left at the pan's near end, slides it under the patty while
flattening it, carries the patty out over the 65 mm wall and slips it off onto the heel; then the cover goes back.

A 1/3-size polycarbonate food pan (325 x 176 mm, 65 mm deep, walls drawn in to a 267 x 121 mm floor) with its flat lift-off cover (two moulded finger wells at its centre),
two round cooked egg patties (a frozen-food maker's 3 1/2 in, 1.5 oz patty: 89 mm across, 42.5 g, 9 mm thick, rounded edges; rigid, or bending on sprung hinges in
soft_patty) at its far end; a solid stainless offset turner (blade 127 mm long x 102 mm wide) lying on the counter; a toasted English muffin heel (95 mm) on an open
sandwich wrapper.

    python demokit.py task.py OUT --mode full
"""

from __future__ import annotations

import json
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import demokit as d  # noqa: E402
import mechkit as m  # noqa: E402
from yam_arena import ik  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Take the cover off the egg tray, put one egg on the muffin with the turner, and put the cover back on the tray."
TOP = TABLE_TOP[SCENE]

# the tray, long side along x, at (TX, TY) on the counter
TX, TY = 0.40, 0.02
OUT_L, OUT_W, DEPTH = 0.325, 0.176, 0.065  # outside, at the rim
IN_L, IN_W = 0.296, 0.150  # inside, at the rim
WALL, FLOOR, FLANGE_T = 0.003, 0.003, 0.003
# the walls draw in toward the floor: DRAFT a side over the 62 mm inside depth, the bound from the listed 2.5 qt (2.37 L) capacity, which a frustum
# from 296 x 150 mm at the rim fills with a 14.5 mm draw-in (267 x 121 mm at the floor); FILLET the floor-to-wall round, a 45 degree chamfer, estimated
DRAFT, FILLET = 0.0145, (float(os.environ.get("EGG_BIG_ROUND", "0.012")) if os.environ.get("TASK_DEMO", "") == "big_round" else 0.008)  # big_round: the top of the floor round's accepted range (README: 0 to 12 mm)
TRAY_M = 0.31  # the empty pan, loose on the counter: 0.682 lb, the maker's listed weight
IX0, IX1 = TX - IN_L / 2, TX + IN_L / 2  # inside, along x
IY0, IY1 = TY - IN_W / 2, TY + IN_W / 2
ZF = TOP + FLOOR  # the tray's floor
ZRIM = TOP + DEPTH
# eggs
EGG_R, EGG_T, EGG_M, EGG_FILLET = 0.0445, 0.009, 0.0425, 0.003  # Papetti's 3 1/2 in round egg patty, 1.5 oz; 9 mm thick estimated
EGG_FAR = IX1 - DRAFT - FILLET - EGG_R - 0.002  # the patty left in the pan lies on the flat floor at the far end
EGG_X = (EGG_FAR - 2 * EGG_R - 0.003, EGG_FAR)  # the served one first, then the one left at the far wall
# lid
LID_T, LID_M = 0.003, 0.22
# the cover's moulded grip: two finger wells sunk into the cover with a land between them that the fingers pinch, sized from the maker's photo
# scaled to the 324 mm cover (wells 68 x 39 mm, land 15 mm); the wells' depth below the underside, 15 mm, is an estimate (README: ruler check)
WELL_L, WELL_W, WELL_D, LAND_W = 0.068, 0.039, 0.015, 0.015
# turner: its frame has the blade's underside at z = 0, the blade's centre at the origin, the tip at +x; blade 5 in long (to the tip) by 4 in wide,
# 2.5 mm thick (thicker than the 1 to 2 mm of comparable stainless servers: README caliper check) bevelled to 0.6 mm at the tip
BL, BW, BT, BT_TIP = 0.127, 0.102, 0.0025, 0.0006
NECK_DX, NECK_DZ = 0.030, 0.065  # the offset: the handle is 65 mm above the blade, 30 mm behind its back edge
HL, HW, HH = 0.130, 0.022, 0.014
TURNER_M = 0.34  # the listed shipping weight, 0.75 lb: the bare turner weighs less, so the carry is no easier than the real one
TURNER0 = np.array([0.50, -0.16])  # the blade's centre, lying on the counter, tip along +x
# muffin heel on its wrapper
MUF_R, MUF_H, MUF_M = 0.0475, 0.022, 0.030
MUF0 = np.array([0.36, -0.30])
WRAP = 0.18
MU_EGG, MU_STEEL, MU_TRAY, MU_MUF = 0.25, 0.25, 0.35, 0.90  # PhysX averages a pair: patty on steel 0.25, on the pan 0.30
# the patty's friction across the range the real trial accepts (README: tilt test, pair 0.20 to 0.50): the low and high ends as their own runs
MU_EGG = {"fric_lo": 0.15, "fric_hi": 0.75}.get(os.environ.get("TASK_DEMO", ""), MU_EGG)
if os.environ.get("TASK_DEMO", "") in ("shallow_wells", "deep_wells"):  # the ends of the wells' accepted depth (README: ruler check, 12 to 20 mm)
    WELL_D = 0.012 if os.environ.get("TASK_DEMO", "") == "shallow_wells" else 0.020
# the patty's friction on the pan across the range the real trial accepts (README: tilt test, pair 0.20 to 0.50); the pan's on the counter goes with it
MU_TRAY = {"fric_pan_lo": 0.15, "fric_pan_hi": 0.75}.get(os.environ.get("TASK_DEMO", ""), MU_TRAY)
# the toasted heel's friction with the patty across the range the real trial accepts (README: tilt test, pair 0.35 to 1.0)
MU_MUF = {"fric_muf_lo": 0.45, "fric_muf_hi": 1.75}.get(os.environ.get("TASK_DEMO", ""), MU_MUF)
if os.environ.get("TASK_DEMO", "") == "thin_patty":  # the thin end of the patty's thickness (README: 7 to 9 mm from its mass and density)
    EGG_T = 0.007
# a bending patty: five strips across the blade's path on sprung hinges, the hinge stiffness that of a plate of modulus EGG_SOFT_E (Pa)
SOFT_E = float(os.environ.get("EGG_SOFT_E", "27000")) if os.environ.get("TASK_DEMO", "") == "soft_patty" else None

LOOKS = {"pc": m.Look(color=m.rgb(0.86, 0.84, 0.78), roughness=0.25, opacity=0.55) if "opacity" in m.Look.__dataclass_fields__ else m.Look(color=m.rgb(0.86, 0.84, 0.78), roughness=0.25),
         "lid": m.Look(color=m.rgb(0.80, 0.80, 0.76), roughness=0.3),
         "patty": m.Look(color=m.rgb(0.96, 0.87, 0.47), roughness=0.75),
         "steel": m.Look(color=m.rgb(0.72, 0.73, 0.74), metallic=0.85, roughness=0.3), "handle": m.Look(color=m.rgb(0.10, 0.10, 0.11), roughness=0.6),
         "crumb": m.Look(color=m.rgb(0.92, 0.85, 0.68), roughness=0.9), "toast": m.Look(color=m.rgb(0.72, 0.50, 0.26), roughness=0.85),
         "paper": m.Look(color=m.rgb(0.93, 0.90, 0.82), roughness=0.95)}


def _box(parts, cols, name, size, centre, look):
    parts.append((name, m.box(size, centre), look))
    cols.append(m.col_box(f"{name}_c", size, centre))


def _slab(uv, w0, w1, across):
    """A solid from a counter-clockwise outline in (u, height), extruded from w0 to w1; u runs along x for an end wall, along y for a side wall."""
    msh = m.prism(uv, w0, w1).turned(90.0, "x")  # outline (u, v) to (x, z), extruded along -y
    return msh.turned(90.0, "z") if across else msh


def _wall_in(z):
    """How far in from the rim's inside edge the wall's inside face is at height z over the pan's bottom (the draw-in and the floor round)."""
    return DRAFT * (1.0 - (z - FLOOR) / (DEPTH - FLOOR)) + max(0.0, FILLET - (z - FLOOR))


def _tray() -> m.Part:
    parts, cols = [], []
    fl, fw = IN_L - 2 * DRAFT + 2 * WALL, IN_W - 2 * DRAFT + 2 * WALL
    _box(parts, cols, "floor", (fl, fw, FLOOR), (0.0, 0.0, FLOOR / 2), "pc")
    for half, other, across, tag in ((IN_L / 2, IN_W / 2, False, "x"), (IN_W / 2, IN_L / 2, True, "y")):
        wall = [(-half + DRAFT - WALL, FLOOR), (-half + DRAFT, FLOOR), (-half, DEPTH), (-half - WALL, DEPTH)]
        fil = [(-half + DRAFT, FLOOR), (-half + DRAFT + FILLET, FLOOR), (-half + _wall_in(FLOOR + FILLET), FLOOR + FILLET)]
        for k, sgn in enumerate((1, -1)):  # the near (-) wall, then the far one mirrored
            for nm, uv in (("w", wall), ("round", fil)):
                pts = [(sgn * u, v) for u, v in uv]
                pts = pts if sgn > 0 else pts[::-1]
                msh = _slab(pts, -(other + WALL), other + WALL, across)
                parts.append((f"{nm}{tag}{k}", msh, "pc"))
                cols.append(m.col_hull(f"{nm}{tag}{k}_c", msh))
    zf = DEPTH - FLANGE_T / 2  # the flange round the top, out to 325 x 176
    fx, fy = (OUT_L - IN_L) / 2, (OUT_W - IN_W) / 2
    for k, s in enumerate((-1, 1)):
        _box(parts, cols, f"fx{k}", (fx, OUT_W, FLANGE_T), (s * (IN_L / 2 + fx / 2), 0.0, zf), "pc")
        _box(parts, cols, f"fy{k}", (IN_L, fy, FLANGE_T), (0.0, s * (IN_W / 2 + fy / 2), zf), "pc")
    return m.free("tray", (TX, TY, TOP + 0.0003), parts, cols, LOOKS, mass=TRAY_M, friction=(MU_TRAY, MU_TRAY), torsional_patch=None)


def _lid() -> m.Part:
    """Flat lid on the flange, with a 6 mm skirt just inside the walls that keeps it seated, and two finger wells at its centre with the land between them.
    Origin: the lid's underside."""
    parts, cols = [], []
    ends = (OUT_L - WELL_L) / 2
    side = OUT_W / 2 - LAND_W / 2 - WELL_W
    for k, s in enumerate((-1, 1)):  # the plate round the two well openings
        _box(parts, cols, f"plate_x{k}", (ends, OUT_W, LID_T), (s * (WELL_L / 2 + ends / 2), 0.0, LID_T / 2), "lid")
        _box(parts, cols, f"plate_y{k}", (WELL_L, side, LID_T), (0.0, s * (OUT_W / 2 - side / 2), LID_T / 2), "lid")
        yw = s * (LAND_W / 2 + WELL_W / 2)  # a well: floor, outer wall and end walls
        _box(parts, cols, f"well_floor{k}", (WELL_L, WELL_W, 0.002), (0.0, yw, -WELL_D + 0.001), "lid")
        _box(parts, cols, f"well_out{k}", (WELL_L, 0.002, WELL_D), (0.0, s * (LAND_W / 2 + WELL_W - 0.001), -WELL_D / 2), "lid")
        for j, sx in enumerate((-1, 1)):
            _box(parts, cols, f"well_end{k}{j}", (0.002, WELL_W, WELL_D), (sx * (WELL_L / 2 - 0.001), yw, -WELL_D / 2), "lid")
    _box(parts, cols, "land", (WELL_L, LAND_W, WELL_D + LID_T), (0.0, 0.0, (LID_T - WELL_D) / 2), "lid")  # the land the fingers pinch, between the wells
    sk, g = 0.006, 0.0015  # the skirt's 6 mm drop and its 1.5 mm clearance to the walls
    for k, s in enumerate((-1, 1)):
        _box(parts, cols, f"skx{k}", (0.002, IN_W - 2 * g - 0.004, sk), (s * (IN_L / 2 - g - 0.001), 0.0, -sk / 2), "lid")
        _box(parts, cols, f"sky{k}", (IN_L - 2 * g - 0.004, 0.002, sk), (0.0, s * (IN_W / 2 - g - 0.001), -sk / 2), "lid")
    return m.free("lid", (TX, TY, ZRIM + 0.0006), parts, cols, LOOKS, mass=LID_M, friction=(0.6, 0.5), torsional_patch=None)


def _egg_mesh(r=EGG_R, t=EGG_T, f=EGG_FILLET, n=6):
    prof = [(0.0, -t / 2)]
    for k in range(n + 1):  # rounded bottom edge
        a = -np.pi / 2 + k * (np.pi / 2) / n
        prof.append((r - f + f * np.cos(a), -t / 2 + f + f * np.sin(a)))
    for k in range(1, n + 1):  # rounded top edge
        a = k * (np.pi / 2) / n
        prof.append((r - f + f * np.cos(a), t / 2 - f + f * np.sin(a)))
    prof.append((0.0, t / 2))
    return m.lathe(prof, 32)


def _egg(name, x) -> m.Part:
    body = _egg_mesh()
    return m.free(name, (x, TY, ZF + EGG_T / 2 + 0.0007), [("patty", body, "patty")], [m.col_hull("c", _egg_mesh(n=4))], LOOKS,
                  mass=EGG_M, friction=(MU_EGG, MU_EGG), torsional_patch=None)


def _strip(mesh, x0, x1):
    v = mesh.v.copy()
    v[:, 0] = np.clip(v[:, 0], x0, x1)  # the disc cut to x0..x1: points past a cut move onto it, inside the disc
    return m.Mesh(v, mesh.f)


def _soft_egg(name, x) -> m.Part:
    """The patty as five strips across x, each outer strip on a sprung hinge about y: a plate of modulus SOFT_E bending under its own weight."""
    w, gap = 2 * EGG_R / 5, 0.0003
    full, hull = _egg_mesh(), _egg_mesh(n=4)
    xs = np.linspace(-EGG_R, EGG_R, 4001)
    ch = 2 * np.sqrt(np.maximum(EGG_R ** 2 - xs ** 2, 0.0))
    def mass(a, b):
        s_ = (xs >= a) & (xs <= b)
        return float(EGG_M * ch[s_].sum() / ch.sum())
    def k(xh):  # E I / spacing, I = chord width x t^3 / 12 at the hinge
        return float(SOFT_E * 2 * np.sqrt(EGG_R ** 2 - xh ** 2) * EGG_T ** 3 / 12 / w)
    links = []
    for sg, tag in ((1.0, "p"), (-1.0, "n")):
        for j, (a, b) in enumerate(((w / 2, 1.5 * w), (1.5 * w, EGG_R))):
            xa, xb = (a + gap, b) if sg > 0 else (-b, -a - gap)
            mk = mass(a, b)
            links.append(m.hinge(f"{tag}{j}", "Y", (sg * a, 0.0, 0.0), -70.0, 70.0, [("patty", _strip(full, xa, xb), "patty")], [m.col_hull("c", _strip(hull, xa, xb))],
                                 mass=mk, spring=k(a), damping=2.5e-4, friction=0.0, armature=1e-6, gravity=True, parent=None if j == 0 else f"{tag}0",
                                 inertia=(mk * ((b - a) ** 2 / 3 + EGG_T ** 2 / 12),) * 3))
    mf = mass(-w / 2, w / 2)
    return m.mechanism(name, (x, TY, ZF + EGG_T / 2 + 0.0007), links, LOOKS, frame_parts=[("patty", _strip(full, -w / 2, w / 2), "patty")],
                       frame_colliders=[m.col_hull("c", _strip(hull, -w / 2, w / 2))], friction=(MU_EGG, MU_EGG), solver_iters=64, free_mass=mf,
                       frame_inertia=(mf * (2 * EGG_R) ** 2 / 12, mf * w ** 2 / 12, mf * (2 * EGG_R) ** 2 / 12))


def _blade_mesh():
    xb, xt = -BL / 2, BL / 2
    outline = np.array([[xb, 0.0], [xt, 0.0], [xt, BT_TIP], [xt - 0.012, BT], [xb, BT]])  # xz: bevelled front edge
    msh = m.prism(outline, -BW / 2, BW / 2)  # extruded along its third axis
    return msh.turned(90.0, "x")


def _turner_geom():
    xb = -BL / 2
    neck = m.tube([(xb + 0.002, 0.0, BT + 0.004), (xb - NECK_DX * 0.5, 0.0, NECK_DZ * 0.55), (xb - NECK_DX, 0.0, NECK_DZ)], 0.004, sides=10)
    hx = xb - NECK_DX - HL / 2
    handle = m.box((HL, HW, HH), (hx, 0.0, NECK_DZ))
    return neck, hx, handle


def _turner() -> m.Part:
    blade = _blade_mesh()
    neck, hx, handle = _turner_geom()
    parts = [("blade", blade, "steel"), ("neck", neck, "steel"), ("handle", handle, "handle")]
    cols = [m.col_hull("blade_c", blade),
            m.col_hull("neck_c", neck),
            m.col_box("handle_c", (HL, HW, HH), (hx, 0.0, NECK_DZ))]
    # lying on the counter on its blade's back edge and its handle's end (the handle outweighs the thin blade)
    a = np.arctan2(NECK_DZ - HH / 2, (BL / 2 + NECK_DX + HL) - BL / 2)
    q = (0.0, float(np.sin(-a / 2)), 0.0, float(np.cos(-a / 2)))  # tip up by a
    rot = m.quat_matrix(q)
    back = rot @ np.array([-BL / 2, 0.0, 0.0])
    pos = np.array([TURNER0[0], TURNER0[1], TOP + 0.0006]) - np.array([0.0, 0.0, back[2]])
    return m.free("turner", tuple(pos), parts, cols, LOOKS, mass=TURNER_M, friction=(MU_STEEL, MU_STEEL), torsional_patch=None, start_quat=q)


def _muffin() -> m.Part:
    crumb = m.cylinder(MUF_R, MUF_H - 0.004, (0.0, 0.0, (MUF_H - 0.004) / 2), segments=32)
    toast = m.cylinder(MUF_R - 0.003, 0.004, (0.0, 0.0, MUF_H - 0.002), segments=32)
    return m.free("muffin", (MUF0[0], MUF0[1], TOP + 0.0012), [("crumb", crumb, "toast"), ("top", toast, "crumb")], [m.col_cylinder("c", MUF_R, MUF_H, (0.0, 0.0, MUF_H / 2))], LOOKS,
                  mass=MUF_M, friction=(MU_MUF, MU_MUF))


def _wrapper() -> m.Part:
    parts = [("sheet", m.box((WRAP, WRAP, 0.001), (0.0, 0.0, 0.0005)), "paper")]
    return m.fixed("wrapper", (MUF0[0], MUF0[1], TOP), parts, [m.col_box("c", (WRAP, WRAP, 0.001), (0.0, 0.0, 0.0005))], LOOKS, friction=(0.6, 0.5))


OBJECTS = [_tray(), _lid(), (_soft_egg if SOFT_E is not None else _egg)("egg", EGG_X[0]), _egg("egg2", EGG_X[1]), _turner(), _wrapper(), _muffin()]


def blade_points(k):
    """The blade's outline corners on its top face, in the world."""
    return k.to_world(np.array([[sx * BL / 2, sy * BW / 2, BT] for sx in (-1, 1) for sy in (-1, 1)]))


def in_tray(tr, p, margin=0.0):
    """Inside the tray's walls (plus `margin`), in the tray's own frame: the tray is loose on the counter."""
    loc = tr.to_local(np.asarray(p)[None])[0]
    return abs(loc[0]) <= IN_L / 2 + margin and abs(loc[1]) <= IN_W / 2 + margin


def in_outline(tr, p, margin=0.0):
    """Within the pan's outline at the rim (325 x 176 mm) plus `margin`, in the pan's own frame."""
    loc = tr.to_local(np.asarray(p)[None])[0]
    return abs(loc[0]) <= OUT_L / 2 + margin and abs(loc[1]) <= OUT_W / 2 + margin


def below_rim(tr, p):
    return tr.to_local(np.asarray(p)[None])[0][2] < DEPTH


class Tracker:
    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.steps, self.last = 0, {}
        self.void = ""
        self.support, self.off_steps, self.path_ok = "tray", 0, True
        self.bladed, self.blade_h = False, 0.0  # the egg has ridden the blade; its lowest point's height over the heel top at the last blade step
        self.blade_z = None  # the egg's height at its last step on the blade: once off the blade it may only go down until it lands

    def egg_on_blade(self, e, k):
        loc = k.to_local(e.pos[None])[0]
        return abs(loc[0]) <= BL / 2 + 0.01 and abs(loc[1]) <= BW / 2 + 0.01 and abs(loc[2] - (BT + EGG_T / 2)) <= 0.004

    def step(self, env):
        self.steps += 1
        dt = float(env.unwrapped.step_dt)
        e, e2, lid, k, mu, tr = (m.state(env, n) for n in ("egg", "egg2", "lid", "turner", "muffin", "tray"))
        corners = lid.to_world(np.array([[sx * OUT_L / 2, sy * OUT_W / 2, 0.0] for sx in (-1, 1) for sy in (-1, 1)]))  # the plate's corners
        wells = lid.to_world(np.array([[sx * WELL_L / 2, sy * (LAND_W / 2 + WELL_W), -WELL_D] for sx in (-1, 1) for sy in (-1, 1)]))  # the wells' lowest corners
        lid_low = float(min(corners[:, 2].min(), wells[:, 2].min()))
        rim = tr.pos[2] + DEPTH
        lc = tr.to_local(corners)  # the cover's corners in the tray's frame
        off_pan = lc[:, 0].min() > OUT_L / 2 or lc[:, 0].max() < -OUT_L / 2 or lc[:, 1].min() > OUT_W / 2 or lc[:, 1].max() < -OUT_W / 2
        # the cover is clear when held 10 mm or more above the rim anywhere, or when it is wholly off the pan's outline, held or set down on the counter
        lid_held = lid_low >= rim + 0.010 or bool(off_pan)
        ll = tr.to_local(lid.pos[None])[0]
        seated = abs(ll[2] - DEPTH) < 0.002 and np.hypot(ll[0], ll[1]) < 0.008 and lid.tilt_deg() < 1.0  # down on its seat, not propped on a wall
        for egg in (e, e2):
            tl_e = np.radians(min(egg.tilt_deg(), 90.0))
            if egg.pos[2] - EGG_R * np.sin(tl_e) - EGG_T / 2 * np.cos(tl_e) < TOP + 0.004 and not in_tray(tr, egg.pos, 0.01):  # its lowest point, wherever it is outside the pan
                self.void = self.void or "an egg touched the counter"
        if not (in_tray(tr, e2.pos) and below_rim(tr, e2.pos)):
            self.void = self.void or "the second egg left the tray"
        on_blade = self.egg_on_blade(e, k)
        muf_top = mu.pos[2] + MUF_H
        loc = mu.to_local(e.pos[None])[0]
        on_muffin = abs(loc[2] - (MUF_H + EGG_T / 2)) <= 0.004 and np.hypot(loc[0], loc[1]) <= MUF_R
        if in_tray(tr, e.pos) and below_rim(tr, e.pos):
            where = "tray"
        elif on_blade:
            where = "blade"
            self.bladed = True
            tl = np.radians(e.tilt_deg())
            self.blade_h = float(e.pos[2] - EGG_R * np.sin(tl) - EGG_T / 2 * np.cos(tl) - muf_top)  # the patty's lowest point over the heel top
            self.blade_z = float(e.pos[2])
        elif on_muffin or (np.hypot(loc[0], loc[1]) <= MUF_R and MUF_H - 0.002 <= loc[2] - EGG_T / 2 <= MUF_H + 0.03):
            where = "muffin"  # on the heel, or coming down onto it off the blade
            if self.support != "muffin" and (not self.bladed or self.blade_h > 0.02):
                self.path_ok = False  # skipped the blade, or slid off it from more than 20 mm above the heel top (dropped)
            if self.support not in ("blade", "muffin") and self.off_steps * dt > 0.5:
                self.path_ok = False
        else:
            where = "other"
        if where != "tray" and self.support == "tray":  # the moment the egg comes out of the tray
            if not lid_held:
                self.void = self.void or "the egg came out with the cover still on or resting on the pan"
            kl = k.to_local(e.pos[None])[0]
            if not (abs(kl[0]) <= BL / 2 + 0.01 and abs(kl[1]) <= BW / 2 + 0.01 and -0.002 <= kl[2] - BT <= EGG_T + 0.010):
                self.path_ok = False  # it left the pan on something other than the blade under it (fingers): the turner must lift it out
        if where != "blade" and self.support == "blade" and self.blade_z is not None and e.pos[2] > self.blade_z + 0.004:
            self.path_ok = False  # lifted off the blade (fingers): a patty slid off the blade only goes down until it lands
        if where == "other":
            self.off_steps += 1
            if self.off_steps * dt > 0.5:
                self.path_ok = False  # carried, pushed or dropped by anything but the turner's blade
        else:
            self.off_steps = 0
            self.support = where
        off = float(np.hypot(*(e.pos[:2] - mu.pos[:2])))
        out_on_blade = where == "blade" and not in_outline(tr, e.pos, 0.02)
        heel_ok = bool(on_muffin) and where == "muffin" and e.tilt_deg() <= 10.0 and off <= 0.012
        self.last = {"egg_mm": [round(v * 1000, 1) for v in e.pos], "egg_tilt": round(e.tilt_deg(), 1), "where": where, "egg_muffin_mm": round(off * 1000, 1),
                     "muffin_top_mm": round((muf_top - TOP) * 1000, 1), "lid_low_mm": round((lid_low - TOP) * 1000, 1), "lid_seated": bool(seated),
                     "lid_held": bool(lid_held), "out_on_blade": bool(out_on_blade), "path_ok": self.path_ok, "on_muffin": bool(on_muffin), "heel_ok": heel_ok, "void": self.void,
                     "blade_tilt": round(k.tilt_deg(), 1), "egg2_in": bool(in_tray(tr, e2.pos)), "tray_moved_mm": round(float(np.hypot(tr.pos[0] - TX, tr.pos[1] - TY)) * 1000, 1),
                     "lid_in_tray_mm": [round(v * 1000, 1) for v in ll], "lid_tilt": round(lid.tilt_deg(), 1), "tray_tilt": round(tr.tilt_deg(), 1)}
        if self.steps % 20 == 0:
            print("[egg] " + json.dumps({"mode": os.environ.get("TASK_DEMO", ""), "t": round(self.steps * dt, 2), **self.last}), flush=True)


TRACK = Tracker()
m.on_step(TRACK.step)
m.on_reset(TRACK.reset)


def score(objects) -> float:
    """End state only; the void and the path are failures that stay once met. 0.3 the egg on the blade, carried clear of the tray's outline by 2 cm; 0.6 the egg on the heel;
    1.0 that and the lid seated with the other egg still in the tray."""
    r = TRACK.last
    if not r or r["void"] or not r["path_ok"]:
        return 0.0
    if r["heel_ok"]:
        return 1.0 if (r["lid_seated"] and r["egg2_in"]) else 0.6
    return 0.3 if r["out_on_blade"] else 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0


# -- the scripted demo ----------------------------------------------------------------------------

def _tcp_pose(ctx, side):
    return ik.tcp_pose(ctx.observed(side)[:6], ctx.arm(side).base)


def _rot_y(deg):
    a = np.radians(deg)
    return np.array([[np.cos(a), 0.0, np.sin(a)], [0.0, 1.0, 0.0], [-np.sin(a), 0.0, np.cos(a)]])


class Hold:
    """The turner as held: where the TCP sits in the turner's frame, read from the observed pose after the grip."""

    def __init__(self, ctx):
        k = ctx.state("turner")
        p, r = _tcp_pose(ctx, "right")
        rk = k.rotation()
        self.p_rel = rk.T @ (p - k.pos)
        self.r_rel = rk.T @ r

    def tcp_for(self, tip, pitch_down):
        """The TCP pose that puts the blade's tip (underside, centre) at `tip` with the blade pitched `pitch_down` degrees tip-down, square to x."""
        rk = _rot_y(pitch_down)
        origin = np.asarray(tip, float) - rk @ np.array([BL / 2, 0.0, 0.0])
        return origin + rk @ self.p_rel, d._as_grip(rk @ self.r_rel)


def _knob_now(ctx, knob):
    """The finger wells' land: its seated place over the tray where the tray is now (it may have been nudged)."""
    return knob + (ctx.state("tray").pos - np.array([TX, TY, TOP + 0.0003]))


LID_OPEN = -0.020  # the cover hand opens only 40 mm across: the fingers stay inside the finger wells, either side of the land


def _let_go(L, label):
    """Open the fingers to 40 mm, within the wells, and forget the trim."""
    yield from L.move(L.at(), finger=LID_OPEN, t=0.5, label=label)
    L.trim = np.zeros(3)


def _press_seat(ctx, knob):
    """Fingers up out of the wells, closed, and pushed down on the land to a few mm below where it sits when seated: a cover caught on one side
    of the rim settles onto its seat."""
    L = ctx.left
    yield from L.move(L.at() + np.array([0.0, 0.0, 0.012]), t=0.5, label="fingers up out of the wells")
    yield from L.close(label="fingers together")
    seat = _knob_now(ctx, knob)
    yield from L.move(seat + np.array([0.0, 0.0, 0.012]), t=0.6, label="over the land")
    yield from L.move(seat + np.array([0.0, 0.0, float(os.environ.get("EGG_PRESS", "0.008"))]), t=0.8, label="press the cover down onto its seat")
    yield from _let_go(L, "fingers apart, off the land")


def _lid_down(ctx, knob):
    """Line the held lid up over the tray by where it is seen to be (it shifts in the hand when the fingers close), then lower it onto its seat."""
    L = ctx.left
    tgt = _knob_now(ctx, knob) + np.array([0.0, 0.0, 0.04])
    for _ in range(4):
        lid, tr = ctx.state("lid"), ctx.state("tray")
        err = tr.pos[:2] - lid.pos[:2]
        tgt = tgt + np.array([err[0], err[1], 0.0])
        yield from L.move(tgt, t=0.6, label="line the lid up over the tray")
    yield from L.move(tgt - np.array([0.0, 0.0, 0.036]), t=1.2, label="lid down on the tray")


def _lid_grip():
    """The cover arm's grip on the land between the finger wells: fingers closing across it (along y), the hand tilted in the x-z plane, which keeps the lid hold within reach."""
    tilt = float(os.environ.get("EGG_LIDTILT", "25"))
    return d.tilted(abs(tilt), 0.0 if tilt >= 0 else 180.0) if tilt else d.down(0.0)


def _grab_egg(ctx, knob, f=1.0):
    """Lazy route: the lid lifted and held as in the demo, the egg pinched across by the right fingers and set on the heel, the lid put back."""
    L, R = ctx.left, ctx.right
    e = ctx.state("egg").pos
    yield from ctx.together(L.move(knob + np.array([0.0, 0.0, 0.07]), grip=_lid_grip(), finger=LID_OPEN, t=2.5, label="over the knob"),
                            R.move(e + np.array([0.0, 0.0, 0.12]), grip=d.down(0.0), finger=d.FINGER_OPEN, t=2.5, label="lazy: over the egg"))
    yield from L.move(knob, t=1.0, label="round the knob")
    yield from L.close(LAND_W, force=30.0, label="grip the knob")
    yield from L.move(L.at() + np.array([0.0, 0.0, 0.05]), t=1.0, label="lift the lid")
    yield from L.move(np.array([0.34, 0.21, TOP + 0.125]), t=2.0, label="hold the lid clear")
    yield from R.move(e + np.array([0.0, 0.0, 0.004]), t=1.5, label="lazy: fingers down round the egg")
    yield from R.close(2 * EGG_R, force=20.0, label="lazy: pinch the egg")
    mu = ctx.state("muffin").pos
    yield from R.move(R.at() + np.array([0.0, 0.0, 0.10]), t=1.0 * f, label="lazy: lift the egg")
    yield from R.move(np.array([mu[0], mu[1], mu[2] + MUF_H + EGG_T / 2 + 0.05]), t=2.0 * f, label="lazy: over the heel")
    yield from R.move(np.array([mu[0], mu[1], mu[2] + MUF_H + EGG_T / 2 + 0.002]), t=1.0 * f, label="lazy: down on the heel")
    yield from R.open(label="lazy: let go")
    yield from R.move(R.at() + np.array([0.0, 0.0, 0.08]), t=1.0, label="lazy: clear")
    yield from L.move(_knob_now(ctx, knob) + np.array([0.0, 0.0, 0.04]), t=2.5, label="lid back over the tray")
    yield from _lid_down(ctx, knob)
    yield from _let_go(L, "let go of the cover")
    yield from _press_seat(ctx, knob)
    yield from L.move(L.at() + np.array([0.0, 0.0, 0.08]), t=1.0, label="clear")
    yield from L.wait(2.5)
    print("[egg] END " + json.dumps({"mode": ctx.mode, **TRACK.last}), flush=True)


def _min_pitch(x_in):
    """The least tip-down pitch (degrees) at which the blade, its tip on the floor `x_in` in from the near rim's inside edge, clears the near wall
    by 4 mm along its whole underside (the wall draws in toward the floor and rounds into it)."""
    s = np.linspace(0.0, BL, 64)
    for th in np.arange(0.0, 80.0, 0.5):
        c, sn = np.cos(np.radians(th)), np.sin(np.radians(th))
        x, z = x_in - s * c, FLOOR + s * sn
        if all(xx >= _wall_in(zz) + 0.004 for xx, zz in zip(x, z) if zz < DEPTH + 0.004):
            return float(th)
    return 80.0


def _pinch_to_blade(ctx, hold, ix0):
    """Lazy route: the blade held level just past the near wall; the free hand pinches the patty out of the pan and lays it on the blade."""
    L, R = ctx.left, ctx.right
    e = ctx.state("egg").pos
    tip_hold = np.array([ix0 - 0.03 + BL / 2, TY, ZRIM + 0.03])
    p_h, g_h = hold.tcp_for(tip_hold, 2.0)
    yield from ctx.together(R.move(p_h, grip=g_h, t=2.0, label="lazy: blade level beside the pan"),
                            L.move(e + np.array([0.0, 0.0, 0.12]), grip=d.down(0.0), finger=d.FINGER_OPEN, t=2.0, label="lazy: free hand over the patty"))
    yield from L.move(e + np.array([0.0, 0.0, 0.004]), t=1.0, label="lazy: fingers round the patty")
    yield from L.close(2 * EGG_R, force=20.0, label="lazy: pinch the patty")
    yield from L.move(L.at() + np.array([0.0, 0.0, 0.10]), t=0.6, label="lazy: lift it out")
    mid = tip_hold - np.array([BL / 2, 0.0, 0.0])
    yield from L.move(mid + np.array([0.0, 0.0, 0.03]), t=0.6, label="lazy: over the blade")
    yield from L.move(mid + np.array([0.0, 0.0, BT + EGG_T / 2 + 0.002]), t=0.4, label="lazy: down on the blade")
    yield from L.open(label="lazy: let go")
    yield from L.move(L.at() + np.array([0.0, 0.0, 0.08]), t=0.8, label="lazy: clear")


def _scoop(ctx, hold, ix0, xs, pitches, tip_in):
    """The blade in steep at the patty's near edge, then along the floor under it, flattening as the near wall allows; lifted and backed out."""
    L, R = ctx.left, ctx.right
    # over the pan just above the rim, then down and tipped steeper as it goes: a steep blade held high puts the hand out of reach
    for pd, up in ((15.0, DEPTH - FLOOR + 0.02), (0.5 * (15.0 + pitches[0]), 0.045), (pitches[0], 0.02)):
        p_m, g_m = hold.tcp_for(tip_in + np.array([0.0, 0.0, up]), pd)
        yield from R.move(p_m, grip=g_m, t=1.0, label="blade over the pan, tipping down steep")
    p_in, g_in = hold.tcp_for(tip_in, pitches[0])
    yield from R.move(p_in, grip=g_in, t=1.5, label="blade tip down to the floor")
    for x, pd in zip(xs[1:], pitches[1:]):  # in under the patty, flattening as the blade's back end clears the near wall
        p_s, g_s = hold.tcp_for(np.array([x, TY, ZF - 0.0005 + float(os.environ.get("EGG_TIPDZ", "0"))]), pd)
        yield from R.move(p_s, grip=g_s, t=0.6, label="slide under the egg")
    tip_far = np.array([xs[-1], TY, ZF - 0.001])
    p_lift, g_lift = hold.tcp_for(tip_far + np.array([0.0, 0.0, 0.015]), 4.0)
    yield from R.move(p_lift, grip=g_lift, t=1.0, label="flatten and lift")
    if ctx.mode == "near_outline":  # lazy route: the patty lifted on the blade and held just past the pan's near wall
        p_n, _ = hold.tcp_for(tip_far + np.array([0.0, 0.0, 0.085]), 4.0)
        yield from R.move(p_n, t=1.2, label="lazy: lift clear of the wall")
        k = ctx.state("turner")
        on = k.to_local(ctx.state("egg").pos[None])[0][0]  # where the patty rides along the blade
        p_n, _ = hold.tcp_for(np.array([ix0 - (OUT_L - IN_L) / 2 - 0.010 + BL / 2 - on, TY, ZRIM + 0.025]), 4.0)  # its centre 10 mm past the outline
        yield from R.move(p_n, t=1.5, label="lazy: just past the near rim")
        yield from R.wait(3.0)
        print("[egg] END " + json.dumps({"mode": ctx.mode, **TRACK.last}), flush=True)
        return
    p_hi, _ = hold.tcp_for(tip_far + np.array([0.0, 0.0, 0.085]), 4.0)
    yield from R.move(p_hi, t=1.2, label="lift clear of the wall")
    p_back, _ = hold.tcp_for(np.array([ix0 - 0.03, TY, ZRIM + 0.025]) + np.array([BL / 2 + 0.01, 0.0, 0.0]), 4.0)
    yield from R.move(p_back, t=1.5, label="back out over the near wall")


def demo_script(ctx):
    L, R = ctx.left, ctx.right
    lid = ctx.state("lid")
    knob = lid.pos + np.array([0.0, 0.0, LID_T - 0.006])  # the grip point: the fingers down in the wells, either side of the land
    k = ctx.state("turner")
    a = np.degrees(np.arctan2(k.rotation()[2, 0], k.rotation()[0, 0]))  # the handle's rise toward the tip
    grip_pt = k.to_world(np.array([[-BL / 2 - NECK_DX - 0.035, 0.0, NECK_DZ]]))[0]
    if ctx.mode == "none":  # lazy route: nothing done
        yield from L.wait(3.0)
        print("[egg] END " + json.dumps({"mode": ctx.mode, **TRACK.last}), flush=True)
        return
    if ctx.mode in ("grab_egg", "pinch_fast"):  # lazy routes: the egg taken out by the fingers, no turner (fast: quick enough to stay under the off-path time)
        yield from _grab_egg(ctx, knob, 0.2 if ctx.mode == "pinch_fast" else 1.0)
        return
    lean = float(os.environ.get("EGG_LEAN", "60"))  # the hand leans forward of square to the handle: the reach into the tray wants it
    rgrip = d.tilted(a + lean, 0.0)
    yield from ctx.together(L.move(knob + np.array([0.0, 0.0, 0.07]), grip=_lid_grip(), finger=LID_OPEN, t=2.5, label="over the knob"),
                            R.move(grip_pt + 0.07 * np.array([0.0, 0.0, 1.0]), grip=rgrip, finger=d.FINGER_OPEN, t=2.5, label="over the handle"))
    yield from ctx.together(L.move(knob, t=1.0, label="round the knob"), R.move(grip_pt, t=1.0, label="round the handle"))
    yield from ctx.together(L.close(LAND_W, force=30.0, label="grip the knob"), R.close(HW, force=40.0, label="grip the handle"))
    # the lid straight up, then off to the left, held level
    if ctx.mode == "slide_lid":  # lazy route: the lid slid half off along the rim, never lifted
        yield from L.move(L.at() + np.array([0.0, 0.12, 0.001]), t=1.5, label="lazy: slide the lid along the rim")
        yield from L.open(label="lazy: let go")
        yield from L.move(L.at() + np.array([0.0, 0.0, 0.06]), t=1.0, label="lazy: clear")
    else:
        yield from L.move(L.at() + np.array([0.0, 0.0, 0.05]), t=1.0, label="lift the lid")
    hold = Hold(ctx)
    egg = ctx.state("egg").pos
    tr0 = ctx.state("tray").pos
    ix0 = tr0[0] - IN_L / 2  # the near rim's inside edge, where the pan is now
    xs = np.linspace(egg[0] - EGG_R - 0.004, egg[0] + EGG_R - 0.004, 7)  # the tip's way along the floor: from the patty's near edge to under its far edge
    margin = float(os.environ.get("EGG_PMARGIN", "4"))
    pitches = [max(_min_pitch(x - ix0) + margin, 3.0) for x in xs]  # as flat as the near wall allows at each step
    tip_in = np.array([xs[0], TY, ZF + 0.0005])
    yield from ctx.together(L.move(np.array([0.34, 0.21, TOP + 0.125]) if ctx.mode != "slide_lid" else L.at(), t=2.0, label="hold the lid clear"),
                            R.move(R.at() + np.array([0.0, 0.0, 0.08]), t=1.0, label="lift the turner"))
    if ctx.mode in ("lid_counter", "blade_then_pinch", "pinch_to_blade"):  # another route: the cover set down flat on the counter beside the pan, taken up again at the end
        yield from L.move(np.array([0.34, 0.21, TOP + WELL_D + 0.004]), t=1.5, label="cover down on the counter")
        yield from _let_go(L, "let go of the cover")
    if ctx.mode == "pinch_to_blade":  # lazy route: the free hand lifts the patty out of the pan and lays it on the blade held beside the pan
        yield from _pinch_to_blade(ctx, hold, ix0)
    else:
        yield from _scoop(ctx, hold, ix0, xs, pitches, tip_in)
        if ctx.mode == "near_outline":
            return
    # over the heel: the blade's tip over the muffin's near half, then tipped down so the egg slides forward off it onto the heel
    yield from R.wait(0.3)
    mu = ctx.state("muffin").pos
    k, e = ctx.state("turner"), ctx.state("egg").pos
    egg_on_blade = k.to_local(e[None])[0]
    setback = float(os.environ.get("EGG_SETBACK", "-0.035"))  # the tip set 35 mm past the heel: the patty rides 4 cm behind the tip, and the hand stays in reach
    tip_set = np.array([mu[0] - setback, mu[1] - egg_on_blade[1], mu[2] + MUF_H + 0.006])
    if ctx.mode == "drop_high":  # lazy route: the egg tipped off the blade from well above the heel
        tip_set = tip_set + np.array([0.0, 0.0, 0.10])
    p_ov, g_ov = hold.tcp_for(tip_set + np.array([0.0, 0.0, 0.05]), 1.0)
    yield from R.move(p_ov, grip=g_ov, t=2.5, label="carry to over the heel")
    p_set, _ = hold.tcp_for(tip_set, 1.0)
    yield from R.move(p_set, t=1.0, label="down over the heel")
    if ctx.mode == "blade_then_pinch":  # lazy route: the free hand lifts the patty off the blade and sets it on the heel
        e = ctx.state("egg").pos
        yield from L.move(L.at() + np.array([0.0, 0.0, 0.10]), t=1.0, label="lazy: free hand up off the cover")
        yield from L.move(e + np.array([0.0, 0.0, 0.10]), grip=d.down(0.0), finger=d.FINGER_OPEN, t=2.0, label="lazy: free hand over the patty")
        yield from L.move(e + np.array([0.0, 0.0, 0.004]), t=1.0, label="lazy: fingers round the patty")
        yield from L.close(2 * EGG_R, force=20.0, label="lazy: pinch the patty")
        yield from L.move(L.at() + np.array([0.0, 0.0, 0.02]), t=0.3, label="lazy: lift it off the blade")
        yield from R.move(R.at() + np.array([-0.12, 0.0, 0.0]), t=0.8, label="lazy: blade out from under")
        yield from L.move(L.at() + np.array([0.0, 0.0, -0.02]), t=0.3, label="lazy: down on the heel")
        yield from L.open(label="lazy: let go")
        yield from L.move(L.at() + np.array([0.0, 0.0, 0.08]), t=1.0, label="lazy: clear")
        yield from L.wait(2.0)
        print("[egg] END " + json.dumps({"mode": ctx.mode, **TRACK.last}), flush=True)
        return
    tilt = float(os.environ.get("EGG_TILT", {"fric_hi": "30", "fric_lo": "16", "fric_muf_lo": "18", "soft_patty": "20"}.get(ctx.mode, "26")))  # the blade tipped toward the heel past the patty's slide angle: less where the patty slides readily (a slippery patty or heel, a bending patty that lies close on the blade) so it does not shoot past, more on a sticky one
    for a in np.linspace(1.0, tilt, 12)[1:]:
        p_t, g_t = hold.tcp_for(tip_set - np.array([0.0, 0.0, 0.002]), a)
        yield from R.move(p_t, grip=g_t, t=0.12, label="tip the blade down")
    yield from R.wait(0.6)
    # the egg rests on the tipped blade: shift the blade along until the egg's centre is over the heel's, by the egg's observed place
    e, mu = ctx.state("egg").pos, ctx.state("muffin").pos
    bias = float(os.environ.get("EGG_BIAS", "0.005"))  # the egg is drawn back a few mm as the blade comes out
    shift = np.array([float(np.clip(mu[0] + bias - e[0], -0.05, 0.05)), float(np.clip(mu[1] - e[1], -0.02, 0.02)), 0.0])
    tip_set = tip_set + shift
    if ctx.mode == "egg_edge_counter":  # lazy route: the patty slid off with the blade 7 cm past the heel, down onto the counter beside it
        tip_set = tip_set + np.array([float(v) for v in os.environ.get("EGG_EDGE_OFF", "0.07,0.0").split(",")] + [0.0])
    p_s, g_s = hold.tcp_for(tip_set - np.array([0.0, 0.0, 0.002]), tilt)
    yield from R.move(p_s, grip=g_s, t=1.0, label="line the egg up over the heel")
    yield from R.wait(0.4)
    rk = _rot_y(tilt)
    back = -rk @ np.array([1.0, 0.0, 0.0])  # straight back along the blade, out from under the egg's rear
    p_b, _ = hold.tcp_for(tip_set - np.array([0.0, 0.0, 0.002]) + 0.07 * back, tilt)
    yield from R.move(p_b, t=1.2, label="draw the blade back out")
    if ctx.mode in ("lid_counter", "pinch_to_blade"):
        yield from L.close(LAND_W, force=30.0, label="grip the cover again")
        yield from L.move(L.at() + np.array([0.0, 0.0, 0.10]), t=1.0, label="lift the cover off the counter")
    if ctx.mode in ("lid_off", "slide_lid", "drop_high"):  # lazy route: the lid not put back
        yield from R.move(R.at() + np.array([-0.03, 0.0, 0.06]), t=1.0, label="turner up and away")
        yield from L.wait(3.0)
        print("[egg] END " + json.dumps({"mode": ctx.mode, **TRACK.last}), flush=True)
        return
    yield from ctx.together(R.move(R.at() + np.array([-0.03, 0.0, 0.06]), t=1.0, label="turner up and away"),
                            L.move(_knob_now(ctx, knob) + np.array([0.0, 0.0, 0.04]), t=2.5, label="lid back over the tray"))
    yield from _lid_down(ctx, knob)
    yield from _let_go(L, "let go of the cover")
    yield from _press_seat(ctx, knob)
    yield from L.move(L.at() + np.array([0.0, 0.0, 0.08]), t=1.0, label="clear")
    yield from L.wait(2.5)
    print("[egg] END " + json.dumps({"mode": ctx.mode, **TRACK.last}), flush=True)
