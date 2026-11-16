"""Spray a perfume tester on a blotter strip: hold the strip in the nozzle's line, aim the bottle, press once.

A shop's fragrance counter: a glass counter top with a white display block, and on it a 100 mL eau de parfum spray tester, cap off, its nozzle facing
away from the robot. Five white blotter strips (130 x 12 mm card) stand in a slotted acrylic stand; a white melamine tray waits for the sprayed strip.
One hand takes a strip and holds it upright, tip up, about 15 cm in front of the nozzle; the other turns the tester on its block until the nozzle
points at the strip and presses the actuator once from above, through its full stroke. The sprayed strip is then laid on the tray.

Model (declared in code): the actuator is a sprung slide (4 mm stroke, 10 N at full stroke) that fires once each time it is pressed through 90 % of
its stroke and re-arms when it springs back. A firing casts a 30 degree cone of 2,000 droplet rays from the nozzle; a ray that reaches a strip within
22 cm, not blocked by a fingertip, darkens that spot of the card (the perfume soaks through, so the mark shows on both faces).

Scored by the effect only: the mark on a strip, made while a gripper held the strip up in the air (out of the stand, off the counter), that strip on the
tray having touched nothing but the grippers and the tray since it left the stand, the other strips still in the stand, and the tester on its block.
How the strip is held, its distance from the nozzle and the number of presses are not scored.

Debug:  python demokit.py task.py OUT --no-record --stills 20 --mode full
Final:  python demokit.py task.py OUT
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demokit as d  # noqa: E402
import gatekit as gk  # noqa: E402
import mechkit as m  # noqa: E402

SCENE = "station"
INSTRUCTION = ("Try the perfume on a blotter strip. Take one strip from the stand and hold it upright, tip up, about 15 cm in front of the "
               "tester's nozzle. With the other hand, turn the tester on its display block to aim the nozzle at the strip's tip and press the spray "
               "from above, so the perfume wets the strip across its full width within 4 cm of its tip. Then lay the strip on the tray and move both "
               "hands clear. The strip may touch only the grippers and the tray; leave the other strips standing in the stand and the tester on its "
               "block.")
SUCCESS_HOLD_S = 0.0  # the scorer counts its own 1 s hold
HOLD_STEPS = 20  # 1 s at 20 Hz
TABLE = d.TABLE_TOP
TRACE = os.environ.get("PERFUME_TRACE", "1") != "0"

# -- the counter: a 40 x 60 cm glass top, 1 cm thick, on the table ---------------------------------------------------------------------------------
GLASS_T = 0.010
TOP = TABLE + GLASS_T  # the counter's surface
COUNTER = (0.35, 0.0, 0.40, 0.60)  # centre x, y; size x, y
# -- the display block: white acrylic, 10 x 10 cm, 5 cm high -----------------------------------------------------------------------------------------
BLOCK = (0.35, 0.08, 0.10, 0.05)  # centre x, y; side; height
BLOCK_TOP = TOP + BLOCK[3]
# -- the tester: a 100 mL eau de parfum spray bottle (glass body 76 x 38 mm, 74 mm high; 281 g with its actuator), cap off. Its frame: origin at the
# middle of its base, x across its 76 mm width, y through its 38 mm depth (the nozzle points along +y), z up.
B_W, B_D, B_H = 0.076, 0.038, 0.074
COLLAR_R, COLLAR_TOP = 0.0095, 0.080
ACT_R, ACT_LO, ACT_HI = 0.010, 0.084, 0.098  # the actuator cap at rest: 20 mm across, from 84 to 98 mm up
STROKE, FIRE_AT, REARM = 0.004, 0.0036, 0.0015  # its full stroke; fires at 90 % of it; re-arms once back within 1.5 mm of the top
ACT_K = 10.0 / STROKE  # its spring: 10 N at full stroke (a simulation setting)
NOZZLE = np.array([0.0, ACT_R, 0.090])  # the orifice, on the actuator's +y side, 90 mm up at rest
B_MASS = 0.281
B_YAW = -90.0  # the nozzle faces +x at the start: away from the robot
B_AT = (BLOCK[0], BLOCK[1], BLOCK_TOP)
# -- the spray (simulation settings) --------------------------------------------------------------------------------------------------------------
CONE_DEG, N_RAYS, RANGE = 30.0, 2000, 0.22
SHOW_STEPS = 8  # the mist is drawn for 0.4 s after a press
# -- the blotter strips: 130 x 12 mm white card, 0.5 mm, 0.5 g. Strip frame: origin at its middle, x across its width, y through it, z along it; the
# tip (the end that is sprayed) is +z.
S_L, S_W, S_T, S_MASS = 0.130, 0.012, 0.0005, 0.0005
N_STRIPS, PITCH = 5, 0.035
STAND_Y, STAND_X0 = -0.17, 0.32  # the stand's row of slots runs along x; the first slot (nearest the robot) at x 0.32
STAND_H, SLOT_DEPTH, SLOT = 0.025, 0.020, (0.016, 0.0016)  # the stand's height; its slots' depth and opening (along x, across y)
STRIP_X = [STAND_X0 + PITCH * i for i in range(N_STRIPS)]
STRIP_Z0 = TOP + (STAND_H - SLOT_DEPTH) + S_L / 2 + 0.0005  # a strip's middle, standing on its slot's floor
# the mark: the card in cells, 3 across its width and 20 along its length (6.5 mm), row 0 at the tip
MCOLS, MROWS = 3, 20
MARK_HITS = 2  # droplets that darken a cell
# -- the tray: white melamine, 12 x 18 cm with a 1 cm rim, its long side along y ---------------------------------------------------------------------
TRAY = (0.22, -0.20, 0.12, 0.18)  # centre x, y; size x, y
TRAY_FLOOR = TOP + 0.004
TRAY_RIM, TRAY_WALL = 0.010, 0.004

LOOKS = {
    "glass_top": m.Look(m.rgb(0.78, 0.86, 0.84), roughness=0.05, metallic=0.1),
    "block": m.Look(m.rgb(0.95, 0.95, 0.94), roughness=0.35),
    "bottle": m.Look(glass=m.rgb(0.96, 0.95, 0.90)),
    "juice": m.Look(m.rgb(0.99, 0.90, 0.62), roughness=0.25),
    "label": m.Look(m.rgb(0.97, 0.97, 0.96), roughness=0.6),
    "ink": m.Look(m.rgb(0.05, 0.05, 0.05), roughness=0.6),
    "metal": m.Look(m.rgb(0.78, 0.78, 0.80), roughness=0.25, metallic=0.9),
    "orifice": m.Look(m.rgb(0.05, 0.05, 0.05), roughness=0.5),
    "mist": m.Look(m.rgb(0.98, 0.93, 0.78), roughness=0.4, opacity=0.55),
    "card": m.Look(m.rgb(0.98, 0.98, 0.97), roughness=0.9),
    "wet": m.Look(m.rgb(0.80, 0.63, 0.32), roughness=0.5),
    "acrylic": m.Look(m.rgb(0.80, 0.86, 0.90), roughness=0.15, opacity=0.6),
    "tray": m.Look(m.rgb(0.96, 0.96, 0.95), roughness=0.4),
}


def _chamfer_box(w, dpt, z0, z1, c):
    """A box with its four vertical edges chamfered by `c`: the bottle's bevelled glass."""
    hw, hd = w / 2, dpt / 2
    outline = [(-hw + c, -hd), (hw - c, -hd), (hw, -hd + c), (hw, hd - c), (hw - c, hd), (-hw + c, hd), (-hw, hd - c), (-hw, -hd + c)]
    return m.prism(outline, z0, z1)


def _ray_dirs() -> np.ndarray:
    """N_RAYS unit vectors filling a cone of CONE_DEG about +y, evenly in solid angle (a Fibonacci spiral on the cap)."""
    c0 = math.cos(math.radians(CONE_DEG / 2))
    k = np.arange(N_RAYS)
    cz = 1.0 - (1.0 - c0) * (k + 0.5) / N_RAYS
    s = np.sqrt(1.0 - cz ** 2)
    ph = k * math.pi * (3.0 - math.sqrt(5.0))
    return np.stack([s * np.cos(ph), cz, s * np.sin(ph)], axis=1)


RAYS = _ray_dirs()


def _mist() -> m.Mesh:
    """The mist as drawn: 260 fine droplets in the cone, 0.5 to 15 cm out from the orifice, denser near it (actuator frame)."""
    rng = np.random.default_rng(5)
    drops = []
    for _ in range(260):
        dvec = RAYS[rng.integers(N_RAYS)]
        t = 0.005 + 0.145 * rng.random() ** 1.6
        drops.append(m.sphere(0.0005 + 0.0035 * t, NOZZLE + t * dvec, rings=2, segments=5))
    return m.join(*drops)


def _tester() -> m.Part:
    frame_parts = [("glass", _chamfer_box(B_W, B_D, 0.0, B_H, 0.006), "bottle"),
                   ("juice", _chamfer_box(B_W - 0.006, B_D - 0.006, 0.010, 0.062, 0.004), "juice"),
                   ("label", m.box((0.040, 0.0006, 0.024), (0.0, B_D / 2 + 0.0003, 0.036)), "label"),
                   ("label_ink", m.box((0.026, 0.0008, 0.004), (0.0, B_D / 2 + 0.0004, 0.040)), "ink"),
                   ("label_ink2", m.box((0.016, 0.0008, 0.002), (0.0, B_D / 2 + 0.0004, 0.032)), "ink"),
                   ("collar", m.cylinder(COLLAR_R, COLLAR_TOP - B_H, (0.0, 0.0, (B_H + COLLAR_TOP) / 2)), "metal")]
    frame_cols = [m.col_hull("glass_c", _chamfer_box(B_W, B_D, 0.0, B_H, 0.006)),
                  m.col_cylinder("collar_c", COLLAR_R, COLLAR_TOP - B_H, (0.0, 0.0, (B_H + COLLAR_TOP) / 2))]
    cap = m.cylinder(ACT_R, ACT_HI - ACT_LO, (0.0, 0.0, (ACT_LO + ACT_HI) / 2))
    act_parts = [("cap", cap, "metal"),
                 ("stem", m.cylinder(0.003, ACT_LO - COLLAR_TOP + 0.004, (0.0, 0.0, (COLLAR_TOP - 0.004 + ACT_LO) / 2)), "metal"),
                 ("orifice", m.cylinder(0.0018, 0.001, tuple(NOZZLE + np.array([0.0, 0.0002, 0.0])), axis="y", segments=12), "orifice"),
                 ("mist", _mist(), "mist", True)]
    act = m.slider("act", "Z", -STROKE, 0.0, act_parts, (m.col_cylinder("cap_c", ACT_R, ACT_HI - ACT_LO, (0.0, 0.0, (ACT_LO + ACT_HI) / 2)),),
                   mass=0.005, spring=ACT_K, damping=5.0, friction=0.0, effort=200.0)
    inertia = (B_MASS * (B_D ** 2 + B_H ** 2) / 12, B_MASS * (B_W ** 2 + B_H ** 2) / 12, B_MASS * (B_W ** 2 + B_D ** 2) / 12)
    return m.mechanism("tester", B_AT, [act], LOOKS, frame_parts, frame_cols, yaw_deg=B_YAW, friction=(0.5, 0.4), free_mass=B_MASS - 0.005,
                       frame_inertia=inertia)


def _strip(i: int) -> m.Part:
    parts = [("card", m.box((S_W, S_T, S_L)), "card")]
    cw, ch = S_W / MCOLS, S_L / MROWS
    for r in range(MROWS):
        for c in range(MCOLS):
            parts.append((f"m_{r}_{c}", m.box((cw, S_T + 0.0003, ch), (-S_W / 2 + (c + 0.5) * cw, 0.0, S_L / 2 - (r + 0.5) * ch)), "wet", True))
    cols = [m.col_box("card_c", (S_W, S_T, S_L))]
    return m.free(f"strip{i}", (STRIP_X[i], STAND_Y, STRIP_Z0), parts, cols, LOOKS, mass=S_MASS, friction=(0.6, 0.5), torsional_patch=None, solver_iters=48)


def _fixtures() -> m.Part:
    parts, cols = [], []

    def add(name, size, c, look, collide=True):
        parts.append((name, m.box(size, c), look))
        if collide:
            cols.append(m.col_box(name + "_c", size, c))

    cx, cy, sx, sy = COUNTER
    add("counter", (sx, sy, GLASS_T), (cx, cy, TABLE + GLASS_T / 2), "glass_top")
    bx, by, bs, bh = BLOCK
    add("block", (bs, bs, bh), (bx, by, TOP + bh / 2), "block")
    # the strip stand: a base, two long walls either side of the row of slots, and spacers between the slots
    x0, x1 = STRIP_X[0] - 0.020, STRIP_X[-1] + 0.020
    base_h = STAND_H - SLOT_DEPTH
    add("stand_base", (x1 - x0, 0.030, base_h), ((x0 + x1) / 2, STAND_Y, TOP + base_h / 2), "acrylic")
    wall_w = 0.015 - SLOT[1] / 2
    for s in (-1, 1):
        add(f"stand_wall{'pn'[s > 0]}", (x1 - x0, wall_w, SLOT_DEPTH), ((x0 + x1) / 2, STAND_Y + s * (SLOT[1] / 2 + wall_w / 2), TOP + base_h + SLOT_DEPTH / 2), "acrylic")
    edges = [x0] + [x for xs in STRIP_X for x in (xs - SLOT[0] / 2, xs + SLOT[0] / 2)] + [x1]
    for k in range(0, len(edges), 2):
        a, b = edges[k], edges[k + 1]
        add(f"stand_sp{k // 2}", (b - a, SLOT[1], SLOT_DEPTH), ((a + b) / 2, STAND_Y, TOP + base_h + SLOT_DEPTH / 2), "acrylic")
    # the tray: a floor and a rim
    tx, ty, tsx, tsy = TRAY
    add("tray_floor", (tsx, tsy, TRAY_FLOOR - TOP), (tx, ty, (TOP + TRAY_FLOOR) / 2), "tray")
    for s in (-1, 1):
        add(f"tray_x{'pn'[s > 0]}", (TRAY_WALL, tsy, TRAY_RIM), (tx + s * (tsx - TRAY_WALL) / 2, ty, TOP + TRAY_RIM / 2), "tray")
        add(f"tray_y{'pn'[s > 0]}", (tsx, TRAY_WALL, TRAY_RIM), (tx, ty + s * (tsy - TRAY_WALL) / 2, TOP + TRAY_RIM / 2), "tray")
    return m.fixed("counter", (0.0, 0.0, 0.0), parts, cols, LOOKS, friction=(0.5, 0.4))


OBJECTS = [_fixtures(), _tester()] + [_strip(i) for i in range(N_STRIPS)]


# -- the scorer: plain functions of the state, then the trial's record -------------------------------------------------------------------------------
# By effect only. 0.5: a strip's mark crosses its full width within 40 mm of its tip, counting only droplets that landed while a gripper held the strip
# up in the air (jaws shut on it; 1 cm clear of the stand, its lowest point over 1.5 cm above the counter). 0.75: that strip lies on the tray floor, out
# of the grippers. 1.0: and
# since it left the stand it touched nothing but the grippers and the tray, the other strips never left their slots, the tester stands on its block,
# the hands are 3 cm clear and everything is still for 1 s. 0, latched: the tester falls over or comes down off its block, or a strip reaches the floor.

STRIP_GRIP = gk.Grip(half=S_T / 2, across="y", lo=(-S_W / 2 - 0.004, -1.0, -S_L / 2), hi=(S_W / 2 + 0.004, 1.0, S_L / 2))  # across its faces
EDGE_GRIP = gk.Grip(half=S_W / 2, across="x", lo=(-1.0, -0.03, -S_L / 2), hi=(1.0, 0.03, S_L / 2))  # across its 12 mm, by its long edges
TURNED_COS = math.cos(math.radians(45.0))
AIR, AIR_Z = 0.010, 0.015  # in the air: 1 cm clear of the stand, and its lowest point 1.5 cm above the counter (clear of the tray's 1 cm rims)


def TESTER_LINES(tester) -> list:
    """Lines up the tester's glass faces (its corners and the middle of each face) to the top of its actuator: what the hands must be 3 cm clear of."""
    return [gk.along(tester, (x, y, 0.0), (x, y, ACT_HI), 7) for x in (-B_W / 2, 0.0, B_W / 2) for y in (-B_D / 2, B_D / 2)] + \
        [gk.along(tester, (x, 0.0, 0.0), (x, 0.0, ACT_HI), 7) for x in (-B_W / 2, B_W / 2)]
TIP = np.array([0.0, 0.0, S_L / 2])
FOOT = np.array([0.0, 0.0, -S_L / 2])
CORNERS = np.array([[sx * S_W / 2, 0.0, sz * S_L / 2] for sx in (-1, 1) for sz in (-1, 1)])
STRIP_PTS = np.array([[x, 0.0, z] for x in (-S_W / 2, 0.0, S_W / 2) for z in np.linspace(-S_L / 2, S_L / 2, 27)])  # the card, every 5 mm
GLASS = np.array([[sx * B_W / 2, sy * B_D / 2, z] for sx in (-1, 1) for sy in (-1, 1) for z in (0.0, B_H)])  # the tester's glass corners
TOP_ROWS = int(0.040 // (S_L / MROWS))  # the mark's rows within 40 mm of the tip
TOUCH = 0.002  # a strip point this close to a surface touches it
CLEAR = 0.010  # a strip has left the stand once all of it is this far from the stand
STAND_C = ((STRIP_X[0] + STRIP_X[-1]) / 2, STAND_Y)
STAND_HALF = ((STRIP_X[-1] - STRIP_X[0]) / 2 + 0.020, 0.015)


def nozzle(tester) -> tuple[np.ndarray, np.ndarray]:
    """The orifice's world position (with the actuator where it is now) and the spray's direction."""
    q = tester.joint("act")
    return tester.to_world(NOZZLE + np.array([0.0, 0.0, q]))[0], tester.rotation()[:, 1]


def tip_up_deg(strip) -> float:
    return float(np.degrees(np.arccos(np.clip(strip.rotation()[2, 2], -1.0, 1.0))))


def in_stand(strip, i: int) -> bool:
    """The strip's foot still in its own slot (within its opening and below the stand's top)."""
    f = strip.to_world(FOOT)[0]
    return bool(abs(f[0] - STRIP_X[i]) < SLOT[0] / 2 + 0.004 and abs(f[1] - STAND_Y) < 0.006 and f[2] < TOP + STAND_H + 0.003)


def on_tray(strip) -> bool:
    """Lying on the tray: all four corners over its floor inside the rim, within 2 cm of the floor, and the lowest of them down on the floor (within
    3 mm): resting on it, not held above it."""
    c = strip.to_world(CORNERS)
    tx, ty, tsx, tsy = TRAY
    inside = (np.abs(c[:, 0] - tx) <= tsx / 2 - TRAY_WALL) & (np.abs(c[:, 1] - ty) <= tsy / 2 - TRAY_WALL)
    return bool(np.all(inside) and np.all(c[:, 2] < TRAY_FLOOR + 0.020) and c[:, 2].min() < TRAY_FLOOR + 0.003)


def in_air(p) -> bool:
    """Up in the air: every point of the strip (`p`, world) at least 1 cm clear of the stand, and its lowest point more than 1.5 cm above the counter
    (and so above the tray's 1 cm rims and the table)."""
    over_block = bool(np.any((np.abs(p[:, 0] - BLOCK[0]) <= BLOCK[2] / 2) & (np.abs(p[:, 1] - BLOCK[1]) <= BLOCK[2] / 2)))
    floor = BLOCK_TOP if over_block else TOP  # part of the strip over the display block must clear its top too
    return bool(not np.any(near_stand(p, AIR)) and float(p[:, 2].min()) > floor + AIR_Z)


def standing(tester) -> bool:
    """The tester standing upright on its block."""
    p = tester.pos
    over = abs(p[0] - BLOCK[0]) < BLOCK[2] / 2 and abs(p[1] - BLOCK[1]) < BLOCK[2] / 2
    return bool(over and abs(p[2] - BLOCK_TOP) < 0.005 and tester.tilt_deg() < 10.0)


def tester_down(tester) -> bool:
    """The tester fallen over (tipped past 45 degrees and resting on the block) or come down off its block (within 1 cm of the counter, or lower).
    Lifted in a hand and set back, it is neither."""
    low = float(tester.to_world(GLASS)[:, 2].min())
    return bool(low < TOP + 0.010 or (tester.tilt_deg() > 45.0 and low < BLOCK_TOP + 0.005))


def _in_box(p, c, half, z_lo, z_hi, grow) -> np.ndarray:
    return (np.abs(p[:, 0] - c[0]) <= half[0] + grow) & (np.abs(p[:, 1] - c[1]) <= half[1] + grow) & (p[:, 2] >= z_lo - grow) & (p[:, 2] <= z_hi + grow)


def near_stand(p, grow) -> np.ndarray:
    return _in_box(p, STAND_C, STAND_HALF, TOP, TOP + STAND_H, grow)


def contacts(p, j, strips, tester) -> set[str]:
    """What strip j touches, other than the grippers and the tray: `p` its points (world). The counter or the table outside the tray's outline, the
    strip stand, the display block, the tester (glass, collar and actuator with its nozzle) and the other strips."""
    out = set()
    tx, ty, tsx, tsy = TRAY
    cx, cy, sx, sy = COUNTER
    over_tray = (np.abs(p[:, 0] - tx) <= tsx / 2) & (np.abs(p[:, 1] - ty) <= tsy / 2)
    over_counter = (np.abs(p[:, 0] - cx) <= sx / 2) & (np.abs(p[:, 1] - cy) <= sy / 2)
    if np.any(~over_tray & (p[:, 2] < np.where(over_counter, TOP, TABLE) + TOUCH)):
        out.add("counter")
    if np.any(near_stand(p, TOUCH)):
        out.add("stand")
    if np.any(_in_box(p, BLOCK[:2], (BLOCK[2] / 2, BLOCK[2] / 2), TOP, BLOCK_TOP, TOUCH)):
        out.add("block")
    loc = tester.to_local(p)
    q = tester.joint("act")
    glass = _in_box(loc, (0.0, 0.0), (B_W / 2, B_D / 2), 0.0, B_H, TOUCH)
    cap = (np.hypot(loc[:, 0], loc[:, 1]) <= ACT_R + TOUCH) & (loc[:, 2] >= B_H - TOUCH) & (loc[:, 2] <= ACT_HI + q + TOUCH)
    if np.any(glass | cap):
        out.add("tester")
    for k, s in enumerate(strips):
        if k == j:
            continue
        lk = s.to_local(p)
        d = np.sqrt(np.maximum(np.abs(lk[:, 0]) - S_W / 2, 0.0) ** 2 + lk[:, 1] ** 2 + np.maximum(np.abs(lk[:, 2]) - S_L / 2, 0.0) ** 2)
        if np.any(d < TOUCH + S_T):
            out.add(f"strip{k}")
    return out


def cast(origin, direction, strips, tips) -> list[np.ndarray]:
    """One firing: each strip's droplet hits per cell (MROWS x MCOLS). Rays leave `origin` in a CONE_DEG cone about `direction`, go at most RANGE,
    stop at the first strip they meet, and are blocked by a fingertip point (more than 15 mm from the orifice) within 5 mm of their path."""
    a = direction / np.linalg.norm(direction)
    ref = np.array([0.0, 0.0, 1.0]) if abs(a[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(a, ref)
    u /= np.linalg.norm(u)
    w = np.cross(u, a)
    dirs = RAYS[:, 0:1] * u + RAYS[:, 1:2] * a + RAYS[:, 2:3] * w
    best_t = np.full(len(dirs), np.inf)
    best_s = np.full(len(dirs), -1)
    best_p = np.zeros((len(dirs), 3))
    for k, s in enumerate(strips):
        R = s.rotation()
        ol = (origin - s.pos) @ R
        dl = dirs @ R
        with np.errstate(divide="ignore", invalid="ignore"):
            t = -ol[1] / dl[:, 1]
            p = ol[None, :] + t[:, None] * dl
        ok = np.isfinite(t) & (t > 0.0) & (t < RANGE) & (np.abs(p[:, 0]) <= S_W / 2) & (np.abs(p[:, 2]) <= S_L / 2) & (t < best_t)
        best_t[ok], best_s[ok], best_p[ok] = t[ok], k, p[ok]
    tp = np.asarray(tips, float)
    tp = tp[np.linalg.norm(tp - origin, axis=1) > 0.015]
    if len(tp):
        v = tp - origin
        hit = best_s >= 0
        proj = np.clip(v @ dirs[hit].T, 0.0, best_t[hit][None, :])
        dist = np.linalg.norm(v[:, None, :] - proj[:, :, None] * dirs[hit][None, :, :], axis=2)
        blocked = np.zeros(len(dirs), bool)
        blocked[np.flatnonzero(hit)] = np.any(dist < 0.005, axis=0)
        best_s[blocked] = -1
    out = [np.zeros((MROWS, MCOLS), int) for _ in strips]
    for k in range(len(strips)):
        sel = best_s == k
        if not np.any(sel):
            continue
        col = np.clip(((best_p[sel, 0] + S_W / 2) / (S_W / MCOLS)).astype(int), 0, MCOLS - 1)
        row = np.clip(((S_L / 2 - best_p[sel, 2]) / (S_L / MROWS)).astype(int), 0, MROWS - 1)
        np.add.at(out[k], (row, col), 1)
    return out


def mark_reading(hits) -> tuple[bool, float]:
    """What a ruler reads off the darkened card: whether the mark spans the strip's full width somewhere, and its middle's distance from the tip (m;
    inf if no mark). A cell is dark with MARK_HITS droplets or more."""
    dark = np.asarray(hits) >= MARK_HITS
    rows = np.flatnonzero(dark.any(axis=1))
    if not len(rows):
        return False, math.inf
    full = bool(np.any(dark.all(axis=1)))
    ch = S_L / MROWS
    return full, float((rows.min() + rows.max() + 1) / 2 * ch)


def mark_ok(hits) -> bool:
    """The spray's mark crosses the strip's full width somewhere within 40 mm of its tip: a row of dark cells there, all the way across."""
    dark = np.asarray(hits) >= MARK_HITS
    return bool(np.any(dark[:TOP_ROWS].all(axis=1)))


LADDER = gk.Ladder(("marked", 0.5), ("laid", 0.75), ("clean", 1.0))


class Trial:
    """The record of one trial: the marks on each strip, where each strip has touched since it left the stand, whether each has left its slot, the
    latched failures and the end hold."""

    def __init__(self):
        self.fell, self.dropped = gk.Latch("tester fell over or came down off its block"), gk.Latch("strip on the floor")
        self.hold = gk.EndHold(HOLD_STEPS)
        self.reset()

    def reset(self, env=None):
        for latch in (self.fell, self.dropped):
            latch.reset()
        self.hold.reset()
        self.n = 0
        self.armed, self.fires, self.show = True, 0, 0
        self.hits = [np.zeros((MROWS, MCOLS), int) for _ in range(N_STRIPS)]  # every droplet that landed (what the card shows)
        self.counted = [np.zeros((MROWS, MCOLS), int) for _ in range(N_STRIPS)]  # those that landed while the strip was up in the air (what scores)
        self.shown = [np.zeros((MROWS, MCOLS), bool) for _ in range(N_STRIPS)]
        self.left = [False] * N_STRIPS  # 1 cm clear of the stand at some step
        self.touched = [set() for _ in range(N_STRIPS)]  # what each strip touched (besides grippers and tray) after it left the stand
        self.moved = [False] * N_STRIPS  # out of its slot, or tipped past 20 degrees, at some step
        self.marked = [False] * N_STRIPS
        self.laid = [False] * N_STRIPS
        self.chosen = None  # the strip the end state is read on
        self.still_ref = None
        self.reached = {}
        self.full = False

    def step(self, env):
        self.n += 1
        tester = m.state(env, "tester")
        strips = [m.state(env, f"strip{i}") for i in range(N_STRIPS)]
        tips = m.fingertips(env)
        self.fell.trip(tester_down(tester))
        self.dropped.trip(any(s.pos[2] < TABLE - 0.02 for s in strips))
        air = []
        for i, s in enumerate(strips):
            p = s.to_world(STRIP_PTS)
            air.append(in_air(p))
            if not self.left[i] and not np.any(near_stand(p, CLEAR)):
                self.left[i] = True
            if self.left[i]:
                self.touched[i] |= contacts(p, i, strips, tester)
            if not in_stand(s, i) or tip_up_deg(s) > 20.0:
                self.moved[i] = True
        # the actuator: a firing each time it passes 90 % of its stroke, re-armed once it springs back
        q = tester.joint("act")
        if self.armed and q <= -FIRE_AT:
            self.armed = False
            self.fires += 1
            o, a = nozzle(tester)
            new = cast(o, a, strips, tips)
            # held up in the air: a gripper's jaws shut on the strip (across its faces or by its edges) and the strip off every rest. A strip sprayed
            # in its stand, or lying on the tray, its rims, the block or the counter, is not credited
            held = [gk.held(tips, s, STRIP_GRIP, resting=not air[i]) or gk.held(tips, s, EDGE_GRIP, resting=not air[i]) for i, s in enumerate(strips)]
            for i in range(N_STRIPS):
                self.hits[i] += new[i]
                if held[i] and a[0] < TURNED_COS:  # the tester turned at least 45 degrees from its start (nozzle facing +x)
                    self.counted[i] += new[i]
            if TRACE:
                print(f"[perfume] firing {self.fires} at step {self.n}: in the air {air}, held in the air {held}", flush=True)
            self.show = SHOW_STEPS
            self._draw()
        elif not self.armed and q >= -REARM:
            self.armed = True
        if self.show:
            self.show -= 1
        m.set_visible(f"{m.ENV_PATH}/tester/act/visual/mist", self.show > 0)
        # the rungs, read on the state now
        self.marked = [mark_ok(h) for h in self.counted]
        # lying on the tray means out of the grippers: a grasp of it there, across its faces or by its edges, counts as held (resting=False), so a
        # strip pinned on the tray is not laid
        self.laid = [self.marked[i] and on_tray(s) and not gk.holding(tips, s, STRIP_GRIP, resting=False)
                     and not gk.holding(tips, s, EDGE_GRIP, resting=False) for i, s in enumerate(strips)]
        end_ok, self.chosen = False, None
        for i in (k for k in range(N_STRIPS) if self.laid[k]):
            self.chosen = i
            # the other strips never left their slots
            others = not any(self.moved[k] for k in range(N_STRIPS) if k != i)
            clear = gk.hands_clear(tips, [gk.along(strips[i], FOOT, TIP, 9)] + TESTER_LINES(tester), 0.03)
            if not self.touched[i] and others and standing(tester) and clear:
                end_ok = True
                break
        # everything still: every strip and the tester under 1 mm over 1 s
        if end_ok:
            now = np.concatenate([s.pos for s in strips] + [tester.pos])
            if self.still_ref is None or np.max(np.abs(now - self.still_ref)) > 0.001:
                self.still_ref = now
                self.hold.reset()
        else:
            self.still_ref = None
        self.full = self.hold.step(end_ok)
        self.reached = {"marked": any(self.marked), "laid": any(self.laid), "clean": any(self.laid) and end_ok and self.full}
        if TRACE and (self.n % 20 == 0 or self.show == SHOW_STEPS - 1):
            i = self.chosen if self.chosen is not None else 0
            reading = mark_reading(self.counted[i])
            print(f"[perfume] step {self.n}: act {1000 * q:.2f} mm, fires {self.fires}, left {self.left}, moved {self.moved}, touched "
                  f"{[sorted(t) for t in self.touched]}, tip-up {[round(tip_up_deg(s)) for s in strips]}, tester at "
                  f"{np.round(1000 * (tester.pos - [0, 0, BLOCK_TOP]), 1)} mm tilt {tester.tilt_deg():.1f} yaw "
                  f"{math.degrees(math.atan2(tester.rotation()[1, 1], tester.rotation()[0, 1])):.0f}; strip{i} mark ok {self.marked[i]} full {reading[0]} "
                  f"middle {1000 * reading[1]:.0f} mm, dark cells counted {int((self.counted[i] >= MARK_HITS).sum())} of {int((self.hits[i] >= MARK_HITS).sum())}; on tray {on_tray(strips[i])}, end {end_ok}, "
                  f"hold {self.hold.count}; latches {[bool(x) for x in (self.fell, self.dropped)]}; score {self.score()}", flush=True)

    def _draw(self):
        for i in range(N_STRIPS):
            dark = self.hits[i] >= MARK_HITS
            for r, c in zip(*np.nonzero(dark & ~self.shown[i])):
                m.set_visible(f"{m.ENV_PATH}/strip{i}/visual/m_{r}_{c}", True)
            self.shown[i] |= dark

    def score(self) -> float:
        failed = bool(self.fell) or bool(self.dropped)
        return LADDER.score(self.reached, failed=failed)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.score() >= 1.0


def score(objects) -> float:
    return TRIAL.score()


# -- the demo ----------------------------------------------------------------------------------------------------------------------------------------

PINCH = -0.015  # where the right hand pinches the strip, along it from its middle: 80 mm below the tip, so the top 40 mm stays clear
GRIP_Z = 0.055  # the left hand's TCP on the bottle, above its base: across the glass's upper half
TIP_ABOVE_AXIS = 0.010  # the strip's tip held 1 cm above the spray's axis


def demo_script(ctx):
    """Modes: full: the right takes the strip nearest the robot and lifts it out; the left grips the tester across its width and turns it 90 degrees
    on its block, so the nozzle faces the strip's side; the right holds the strip tip up with its tip 15 cm from the nozzle, on the spray's axis; the
    left presses the actuator from above once; the right lays the strip on the tray; both hands clear. noaim: the same without the turn (the spray
    misses the strip). close: the strip held 8 cm from the nozzle (the distance is not scored, only the mark). one_arm: the left hand alone turns
    the tester and presses it, the right never takes a strip (the spray goes into the air). none: the arms wait."""
    l, r = ctx.left, ctx.right
    mode = ctx.mode
    env = ctx.emb._env
    if mode not in ("full", "none", "noaim", "close", "one_arm"):
        raise ValueError(f"unknown mode {mode!r}")
    for _ in range(4):
        yield
    if mode == "none":
        for _ in range(60):
            yield
        return
    g_strip = d.tilted(60.0, 0.0)  # fingers forward and 30 degrees down, closing across the strip's faces
    a_strip = g_strip.rot[:, 2]
    s0 = m.state(env, "strip0")
    pinch = s0.to_world(np.array([[0.0, 0.0, PINCH]]))[0]
    tester = m.state(env, "tester")
    grip_at = tester.to_world(np.array([[0.0, 0.0, GRIP_Z]]))[0]

    def right_take():
        yield from r.move(pinch - 0.07 * a_strip + np.array([0.0, 0.0, 0.03]), grip=g_strip, finger=d.FINGER_OPEN, t=2.0, label="behind the strip, open")
        yield from r.move(pinch - 0.02 * a_strip, t=0.8, label="fingers either side of the strip")
        yield from r.move(pinch, t=0.6, label="in to the strip")
        yield from r.close(S_T, force=5.0, t=0.5, label="pinch the strip")
        yield from r.move(r.at() + np.array([0.0, 0.0, 0.045]), t=1.2, label="lift it out of its slot")

    def left_turn():
        yield from l.move(grip_at + np.array([-0.06, 0.0, 0.045]), grip=d.down(0.0), finger=d.FINGER_OPEN, t=2.0, label="behind and above the tester")
        yield from l.move(grip_at + np.array([0.0, 0.0, 0.045]), t=1.0, label="over the tester, fingers either side of its cap")
        yield from l.move(grip_at, t=1.2, label="fingers either side of the glass")
        yield from l.close(B_W, force=15.0, t=0.5, label="grip the tester across its width")
        if mode != "noaim":
            yield from l.turn(-90.0, t=2.5, label="turn the tester: nozzle to the strip's side")
        yield from l.open(t=0.5, label="let go of the tester")

    def place():
        """The strip into the spray's line: its tip on the nozzle's axis line, 15 cm out (8 cm in mode close), 1 cm above the axis."""
        tester = m.state(env, "tester")
        o, a = nozzle(tester)
        a_flat = np.array([a[0], a[1], 0.0]) / max(1e-6, float(np.hypot(a[0], a[1])))
        dist = 0.08 if mode == "close" else 0.15
        if mode == "noaim":  # the strip held where the nozzle would point after the turn
            a_flat = np.array([0.0, -1.0, 0.0])
            o = tester.pos + np.array([0.0, -ACT_R, NOZZLE[2]])
        want = o + dist * a_flat + np.array([0.0, 0.0, TIP_ABOVE_AXIS])

        def tip():
            return m.state(env, "strip0").to_world(TIP)[0]

        off = tip() - r.at()
        yield from r.move(want - off + np.array([0.0, 0.0, 0.03]), t=2.0, label="the strip towards the spray's line")
        yield from r.move(want - off, t=1.0, label="the strip in the spray's line")
        yield from r.servo(want, tol=0.003, timeout=2.0, gain=0.3, read=tip, label="the strip's tip onto its place")

    if mode == "one_arm":  # the left hand alone turns the tester and presses it; no strip is held in the spray
        yield from left_turn()
    else:
        yield from ctx.together(right_take(), left_turn())
        yield from place()
    # the press: the open hand up past the actuator cap, the fingers closed just above it, then straight down until it fires (the hand keeps the
    # turned grip: the arm cannot reach a straight-down grip much above the cap)
    tester = m.state(env, "tester")
    cap = tester.to_world(np.array([[0.0, 0.0, ACT_HI]]))[0]
    above = np.array([cap[0], cap[1], cap[2] + 0.013])  # the fingers' real undersides about 5 mm over the cap
    yield from l.move(above, t=1.2, label="the open hand up past the cap")
    yield from l.close(t=0.5, label="fingers closed over the actuator")
    for _ in range(50):
        if m.state(env, "tester").joint("act") <= -FIRE_AT - 0.0002:
            break
        yield from l.move(l.at() - np.array([0.0, 0.0, 0.001]), t=0.05, label="press the actuator down")
    for _ in range(3):
        yield
    yield from l.move(above, t=0.6, label="off the actuator")
    yield from l.move(above + np.array([-0.07, 0.05, -0.01]), t=1.0, label="the left hand back from the tester")
    if mode == "one_arm":
        yield from l.wait(2.0)
        return
    # the strip to the tray: its foot down at the tray's +y end, leaned 30 degrees towards -y, let go so that it lies along the tray
    tx, ty, tsx, tsy = TRAY

    def foot():
        return m.state(env, "strip0").to_world(FOOT)[0]

    yield from r.move(r.at() + np.array([0.0, 0.0, 0.04]), t=0.8, label="the strip up")
    f_goal = np.array([tx, ty + tsy / 2 - 0.028, TRAY_FLOOR + 0.006])
    off = foot() - r.at()
    yield from r.move(f_goal - off + np.array([0.0, 0.0, 0.04]), t=2.5, label="the strip over the tray")
    yield from r.servo(f_goal, tol=0.003, timeout=1.5, gain=0.3, read=foot, label="its foot down on the tray")
    yield from r.arc(foot(), (1.0, 0.0, 0.0), 30.0, t=1.0, label="lean it towards the tray's far end")
    yield from r.open(t=0.4, label="let go of the strip")
    yield from r.move(r.at() - 0.06 * r.approach_axis() + np.array([0.0, 0.0, 0.05]), t=1.0, label="the right hand back and up")
    yield from r.wait(2.0)
