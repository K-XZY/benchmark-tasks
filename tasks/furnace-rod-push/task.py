"""On the station table: a clear tube (150 mm bore, 1.2 m) on stands along y, with a plywood door plate across its mouth at y +50 cm. Inside the mouth lies a boat
of 25 wafers (100 mm, 525 um) with its pushing lug at y +52 cm. A 1 m push rod with a white band 630 mm from its furnace end lies in its holder, two V-rests on the
tube's axis line (y -42 and +46 cm), its tip in the boat's pushing cup. The task: push the boat in at about 1 ft/min (5.1 mm/s) with the rod, the hands holding
the rod only behind the band, the push passing from one hand to the other; stop with the band flush with the door plate; draw the rod out without moving the
boat, lay it in both V-rests with the hook up, and move the hands clear.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 4800 --seed 1 --instruction "<INSTRUCTION below>"
The demo:  python demokit.py task.py OUT
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import gatekit as gk  # noqa: E402
import mechkit as m  # noqa: E402  (hooks yam_arena so that the parts below spawn from USD)
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = ("Push the boat to the middle of the furnace tube with the push rod, at about 1 ft per minute (1 inch every 5 seconds), holding the rod only on the cold side of "
               "its white band (the side toward its blunt back end), until the white band is flush with the furnace door plate. The stroke is longer than one arm's reach, "
               "so you may pass the rod from one hand to the other, but never stop the push for long. Do not touch the boat or the wafers. "
               "Then draw the rod straight back out without moving the boat, lay it on both rests of the holder with its hook facing up, and move both hands clear.")
SUCCESS_HOLD_S = 0.0  # the scorer holds the end state itself (HOLD_STEPS)
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
DT = 0.05

# -- the tube, door and boat (m; x forward from the arm bases, y to the robot's left, z above the world floor) ---------------------------------
XR = 0.40  # the tube's axis, the rod's, the boat's
ZF = TOP + 0.075  # the tube floor's top (bore 150 mm, axis 15 cm above the table)
R_BORE = 0.075
YOFF = 0.30  # where the furnace line stands along y: its door plane at y +50 cm
Y_DOOR = 0.20 + YOFF  # the door plate's front face
TUBE_Y = (0.20 + YOFF, 1.40 + YOFF)
LUG_Y0 = 0.22 + YOFF  # the boat's lug (its pushing face) at the start
BOAT_L = 0.150
BOAT_W = 0.110
BOAT_MASS = 0.39  # loaded
W_R = 0.050  # 100 mm wafers
W_T = 0.000525
W_MASS = 0.0096
PITCH = 0.00476
SLOT_Y0 = -0.0595  # the boat frame's y of the first slot's near face
BOAT_FLOOR = 0.004
W_Z = BOAT_FLOOR + W_R + 0.00003  # a wafer's centre above the boat origin (the tube floor)
LOOSE = range(0)  # all 25 wafers are part of the boat (fidelity note)
BOAT_ORIGIN_Y0 = LUG_Y0 + BOAT_L / 2
TARGET_FACE = 0.83 + YOFF  # the lug's front face when the band is flush with the door
TARGET_TOL = 0.006

# -- the rod ---------------------------------------------------------------------------------------------------------------------------------
ROD_L = 1.0
ROD_R = 0.005
ROD_MASS = 0.17
MARK_U = 0.63  # the band's distance from the furnace end
Z_ROD = ZF + 0.0075  # axis height at the start
ROD_Y0 = LUG_Y0 - ROD_L / 2  # centre
REST_Y = -0.72 + YOFF  # the rod's start rest (under its back end)
WALL_IN = 0.057 if "rail2" in os.environ.get("FURN_X", "") else 0.060  # the tube's side rails stand 60 mm either side of the axis, 5 mm off the 110 mm boat: it rides straight like a boat in a tube, it cannot swing crooked
HOOK0 = 0.0  # the rod's hook starts on top
GUIDE_Y = 0.16 + YOFF  # the holder's front V-rest, just before the door: the rod slides through it on the tube's axis
HOLD_Y = (GUIDE_Y, REST_Y)  # the holder's two V-rests: the one in front of the door and the one under the rod's back end, the rod's own start rests
HOLD_X = XR  # the holder's rests stand on the rod's own line
REST_Z = Z_ROD  # a V-rest seats the rod's axis at this height (the V's apex 7.1 mm below it)
V_SIDE = 0.025  # each face of a V-rest, up its 45 degree slope: the mouth is 35 mm across, so a rod lowered anywhere within it slides to the middle

# -- scoring tolerances ----------------------------------------------------------------------------------------------------------------------
V_TARGET = 0.0051  # 1 ft/min
V_MEAN = (0.0038, 0.0064)
V_WINDOW_MAX = 0.012  # over any 2 s
STALL_S = 10.0
FLUSH_TOL = 0.006
MOVED_BY_DRAW = 0.005
CLEAR = 0.08
HOLD_STEPS = 20
FORWARD_MARGIN = 0.003  # a fingertip this far on the furnace side of the band counts as holding past it
HAND_ON_ROD = 0.03  # a fingertip within 3 cm of the rod's axis is on the rod
SEAT_MM = 0.004
SEAT_DEG = 8.0
TIP_DEG = 8.0
PUSH10 = 0.10

LOOKS = {
    "acrylic": m.Look(color=m.rgb(0.80, 0.86, 0.92), roughness=0.15, opacity=0.22),
    "plywood": m.Look(color=m.rgb(0.70, 0.55, 0.34), roughness=0.7),
    "stand": m.Look(color=m.rgb(0.25, 0.25, 0.27), roughness=0.6),
    "quartz": m.Look(color=m.rgb(0.88, 0.92, 0.96), roughness=0.2, opacity=0.55),
    "si": m.Look(color=m.rgb(0.30, 0.32, 0.38), roughness=0.12, metallic=0.9),
    "rod": m.Look(color=m.rgb(0.62, 0.64, 0.68), roughness=0.3, metallic=0.9),
    "white": m.Look(color=m.rgb(0.95, 0.95, 0.95), roughness=0.5),
    "hook": m.Look(color=m.rgb(0.85, 0.35, 0.10), roughness=0.5),
    "red": m.Look(color=m.rgb(0.80, 0.08, 0.08), roughness=0.5),
    "rest": m.Look(color=m.rgb(0.20, 0.35, 0.60), roughness=0.6),
}

SG = {-1: "n", 1: "p"}
TOOTH_T = 0.0010
CO = 0.0005  # contact offset for the thin parts


def col_box(name, size, center):
    return m.col_box(name, size, center).replace("physxCollision:contactOffset = 0.004", f"physxCollision:contactOffset = {CO}")


def slot_y(n: int) -> float:
    """Slot n's wafer centre, in the boat frame (y along the tube, from the boat's middle)."""
    return SLOT_Y0 + (n - 0.5) * PITCH


# -- the tube and the door ---------------------------------------------------------------------------------------------------------------------
def _tube():
    ox = XR
    oz = TOP
    cols, parts = [], []
    ln = TUBE_Y[1] - TUBE_Y[0]  # one long box each, so the floor has no seam
    y = TUBE_Y[0]
    k = 0
    while y < TUBE_Y[1] - 1e-6:
        yc = y + ln / 2
        cols.append(col_box(f"floor{k}", (0.13, ln, 0.006), (0.0, yc, ZF - oz - 0.003)))
        y += ln
        k += 1
    glass = m.cylinder(R_BORE + 0.003, TUBE_Y[1] - TUBE_Y[0], (0.0, (TUBE_Y[0] + TUBE_Y[1]) / 2, ZF - oz + R_BORE), axis="y", segments=40)
    parts.append(("tube", glass, "acrylic"))
    # the hot-zone mark: a red ring where the boat's middle ends up
    ring = m.cylinder(R_BORE + 0.0045, 0.004, (0.0, TARGET_FACE + BOAT_L / 2, ZF - oz + R_BORE), axis="y", segments=40)
    parts.append(("hotzone", ring, "red"))
    # stands under the tube
    for si, ys in enumerate((0.45 + YOFF, 1.05 + YOFF)):
        size = (0.10, 0.04, ZF - 0.006 - 0.0 + 0.0 - 0.0)
        top_z = ZF - oz - 0.006
        h = top_z + oz  # from the floor (z = 0)
        # the stands are drawn only: the floor box carries the boat
        parts.append((f"stand{si}", m.box((0.10, 0.04, h), (0.0, ys, top_z - h / 2)), "stand"))
    return m.fixed("tube", (ox, 0.0, oz), parts, cols, LOOKS, friction=(0.3, 0.25))


def _rails():
    """The tube's two side rails, 60 mm either side of its axis: where a boat that turns a little meets the tube's curved wall. A corner slides along the
    smooth quartz wall rather than catching on it, so the rails are low-friction (0.05)."""
    ln = TUBE_Y[1] - TUBE_Y[0]
    yc = (TUBE_Y[0] + TUBE_Y[1]) / 2
    cols = [col_box(f"wall{SG[s]}", (0.006, ln, R_BORE), (s * (WALL_IN + 0.003), yc, ZF - TOP + R_BORE / 2)) for s in (-1, 1)]
    return m.fixed("rails", (XR, 0.0, TOP), [], cols, LOOKS, friction=(0.05, 0.05))


def _door():
    """A plywood plate across the mouth with a 150 mm hole on the tube's axis; four boxes round the hole."""
    t = 0.012
    yc = Y_DOOR - t / 2
    zc = ZF + R_BORE  # the hole's centre height
    z0, z1 = TOP, TOP + 0.45
    w = 0.30
    boxes = [("bot", (2 * w, t, ZF - z0), (0.0, yc, (z0 + ZF) / 2 - TOP)),
             ("top", (2 * w, t, z1 - zc - R_BORE), (0.0, yc, (z1 + zc + R_BORE) / 2 - TOP)),
             ("l", (w - R_BORE, t, 2 * R_BORE), (R_BORE + (w - R_BORE) / 2, yc, zc - TOP)),
             ("r", (w - R_BORE, t, 2 * R_BORE), (-(R_BORE + (w - R_BORE) / 2), yc, zc - TOP))]
    cols = [col_box(n, s, c) for n, s, c in boxes]
    vis = m.join(*[m.box(s, c) for n, s, c in boxes])
    return m.fixed("door", (XR, 0.0, TOP), [("plate", vis, "plywood")], cols, LOOKS, friction=(0.5, 0.4))


# -- the boat and its wafers --------------------------------------------------------------------------------------------------------------------
def _disc_strips(prefix: str):
    edges = np.linspace(-W_R, W_R, 6)
    out = []
    for i in range(5):
        a, b = edges[i], edges[i + 1]
        xm = 0.5 * (a + b)
        half = math.sqrt(max(W_R ** 2 - xm ** 2, 1e-8))
        out.append((f"{prefix}{i}", (b - a, W_T, 2 * half), (xm, 0.0, 0.0)))
    return out


def _boat():
    parts, cols = [], []
    # floor and side rails
    floor = (BOAT_W, BOAT_L, BOAT_FLOOR)
    parts.append(("floor", m.box(floor, (0.0, 0.0, BOAT_FLOOR / 2)), "quartz"))
    cols.append(col_box("floor", floor, (0.0, 0.0, BOAT_FLOOR / 2)))
    # the pushing lug: a bar across the near end, 9 mm tall about the rod's axis
    lug = (BOAT_W, 0.004, 0.012)
    lug_c = (0.0, -BOAT_L / 2 + 0.002, 0.003 + 0.006)
    parts.append(("lug", m.box(lug, lug_c), "quartz"))
    cols.append(col_box("lug", lug, lug_c))
    # the pushing cup: two cheeks 12 mm apart in front of the lug take the rod's 10 mm end, so the push goes through the boat's axis and the boat cannot yaw about the rod's tip
    for s in (-1, 1):
        ck = (0.004, 0.014, 0.012)
        cc = (s * 0.008, -BOAT_L / 2 - 0.007, 0.003 + 0.006)
        parts.append((f"cup{SG[s]}", m.box(ck, cc), "quartz"))
        cols.append(col_box(f"cup{SG[s]}", ck, cc))
    # the end posts: for the look (wafers stand clear of them)
    for s in (-1, 1):
        post = (0.004, 0.004, 0.045)
        pc = (s * (BOAT_W / 2 - 0.002), -BOAT_L / 2 + 0.002, 0.0225)
        parts.append((f"post_n{SG[s]}", m.box(post, pc), "quartz"))
        cols.append(col_box(f"post_n{SG[s]}", post, pc))
        pc2 = (s * (BOAT_W / 2 - 0.002), BOAT_L / 2 - 0.002, 0.0225)
        parts.append((f"post_f{SG[s]}", m.box(post, pc2), "quartz"))
        cols.append(col_box(f"post_f{SG[s]}", post, pc2))
    # the 25 wafers: one solid block, drawn as separate discs
    blocks = [(1, 25)]
    for a, b in blocks:
        y0 = SLOT_Y0 + (a - 1) * PITCH
        y1 = SLOT_Y0 + b * PITCH
        hull = m.cylinder(W_R, y1 - y0, (0.0, (y0 + y1) / 2, BOAT_FLOOR + W_R), axis="y", segments=24)
        cols.append(m.col_hull(f"block{a}", hull))
        for n in range(a, b + 1):
            parts.append((f"fw{n}", m.cylinder(W_R, W_T, (0.0, slot_y(n), BOAT_FLOOR + W_R), axis="y", segments=32), "si"))
    # slot teeth at the boat's middle
    for k in range(10, 16):
        yb = SLOT_Y0 + k * PITCH
        for s in (-1, 1):
            size = (0.012, TOOTH_T, 0.030)
            c = (s * 0.032, yb, BOAT_FLOOR + 0.015)
            parts.append((f"tooth{k}{SG[s]}", m.box(size, c), "quartz"))
            cols.append(col_box(f"tooth{k}{SG[s]}", size, c))
    return m.free("boat", (XR, BOAT_ORIGIN_Y0, ZF + 0.0008), parts, cols, LOOKS, mass=BOAT_MASS - len(LOOSE) * W_MASS, friction=((0.3, 0.3) if "feq" in os.environ.get("FURN_X", "") else (0.3, 0.25)), torsional_patch=None)


def _wafer(n: int):
    cols = [col_box(nm, s, c) for nm, s, c in _disc_strips("s")]
    disc = m.cylinder(W_R, W_T, (0.0, 0.0, 0.0), axis="y", segments=32)
    return m.free(f"w{n:02d}", (XR, BOAT_ORIGIN_Y0 + slot_y(n), ZF + 0.0008 + W_Z), [("disc", disc, "si")], cols, LOOKS, mass=W_MASS, friction=(0.30, 0.25), torsional_patch=None)


# -- the rod -------------------------------------------------------------------------------------------------------------------------------------
def _rod():
    parts = [("rod", m.cylinder(ROD_R, ROD_L, (0.0, 0.0, 0.0), axis="y", segments=16), "rod"),
             ("band", m.cylinder(ROD_R + 0.0003, 0.012, (0.0, ROD_L / 2 - MARK_U, 0.0), axis="y", segments=16), "white"),
             ("hook", m.box((0.006, 0.006, 0.008), (0.0, ROD_L / 2 - 0.025, ROD_R + 0.004)), "hook")]
    cols = [col_box("hook", (0.006, 0.006, 0.008), (0.0, ROD_L / 2 - 0.025, ROD_R + 0.004))]  # the hook is solid: on a rest with cheeks it stops the rod rolling far
    seg = ROD_L / 5
    for i in range(5):
        yc = -ROD_L / 2 + seg * (i + 0.5)
        cols.append(m.col_hull(f"rod{i}", m.cylinder(ROD_R, seg, (0.0, yc, 0.0), axis="y", segments=12)))
    ph = math.radians(HOOK0)
    return m.free("rod", (XR, ROD_Y0, Z_ROD + 0.00002), parts, cols, LOOKS, mass=ROD_MASS, friction=(0.25, 0.2), torsional_patch=None, start_quat=(0.0, math.sin(ph / 2), 0.0, math.cos(ph / 2)))


def _rests():
    """The rod's two V-rests (its holder): one under its back end, one in front of the door plate. Each is a pedestal with two quartz faces at 45 degrees."""
    parts, cols = [], []
    apex = REST_Z - ROD_R * math.sqrt(2.0) - TOP  # the V's bottom line, in the rests' frame (origin at the table top on the rod's line)
    t, ln = 0.006, 0.030
    for tag, y in (("s", REST_Y), ("g", GUIDE_Y)):
        for s in (-1, 1):
            d = np.array([s * math.sqrt(0.5), 0.0, math.sqrt(0.5)])  # up the face, away from the middle
            n = np.array([-s * math.sqrt(0.5), 0.0, math.sqrt(0.5)])  # the face's normal, toward the rod
            c = np.array([0.0, y, apex]) + d * V_SIDE / 2 - n * t / 2
            slab = m.box((V_SIDE, ln, t)).turned(-45.0 * s, "y").moved(c)
            parts.append((f"v{tag}{SG[s]}", slab, "rest"))
            cols.append(m.col_hull(f"v{tag}{SG[s]}", slab))
        ped_h = apex - 0.004
        ped = (0.05, ln, ped_h)
        pc = (0.0, y, ped_h / 2)
        parts.append((f"p{tag}", m.box(ped, pc), "rest"))
        cols.append(col_box(f"p{tag}", ped, pc))
    return m.fixed("rests", (XR, 0.0, TOP), parts, cols, LOOKS, friction=(0.2, 0.15))


_X = os.environ.get("FURN_X", "")  # debug toggles for the gate: "nowaf"
OBJECTS = [_tube(), _rails()] + ([] if "nodoor" in _X else [_door()]) + [_rests(), _boat()] + [_rod()] + ([] if "nowaf" in _X else [_wafer(n) for n in LOOSE])
LOOSE_NAMES = [f"w{n:02d}" for n in LOOSE] if "nowaf" not in _X else []


# -- readings ------------------------------------------------------------------------------------------------------------------------------------
def rod_end(rod) -> np.ndarray:
    return rod.pos + rod.rotation() @ np.array([0.0, ROD_L / 2, 0.0])


def rod_mark(rod) -> np.ndarray:
    return rod.pos + rod.rotation() @ np.array([0.0, ROD_L / 2 - MARK_U, 0.0])


def boat_face(boat) -> float:
    """The lug's front face (the boat's pushed end), world y."""
    return float((boat.pos + boat.rotation() @ np.array([0.0, -BOAT_L / 2, 0.0]))[1])


class Trial:
    def reset(self, env=None) -> None:
        self.steps = 0
        self.start_face = None
        self.ys = []
        self.t0 = None  # first motion of the boat
        self.t1 = None  # the band first within tolerance of the door while the rod's end is on the lug
        self.d1 = 0.0
        self.stall = 0
        self.max_stall = 0
        self.forward = gk.Latch()  # a hand on the rod's furnace side of the band
        self.touched = gk.Latch()  # a hand on the boat or a wafer
        self.fell = gk.Latch()
        self.fast = gk.Latch()  # 2 s of the boat faster than 12 mm/s
        self.band = gk.Latch()  # band flush with the door while the rod's end is on the lug
        self.e_max = 0.0
        self.y_ret = None
        self.dragged = gk.Latch()
        self.end = gk.EndHold(HOLD_STEPS)
        self.score, self.full = 0.0, False
        self.r = {}

    def step(self, env) -> None:
        self.steps += 1
        boat = m.state(env, "boat")
        rod = m.state(env, "rod")
        ws = [m.state(env, nm) for nm in LOOSE_NAMES]
        face = boat_face(boat)
        if self.start_face is None:
            self.start_face = face
        pushed = face - self.start_face
        e = rod_end(rod)
        mk = rod_mark(rod)
        axis = rod.rotation()[:, 1]
        self.ys.append(face)
        if self.t0 is None and pushed > 0.0005:
            self.t0 = self.steps
        # speed: any 2 s window, and the stall counter while pushing
        if len(self.ys) > 40 and abs(self.ys[-1] - self.ys[-41]) / 2.0 > V_WINDOW_MAX:
            self.fast.trip(True)
        in_push = self.t0 is not None and self.t1 is None
        if in_push:
            if len(self.ys) > 1 and abs(self.ys[-1] - self.ys[-2]) < 0.00002:
                self.stall += 1
            else:
                self.stall = 0
            self.max_stall = max(self.max_stall, self.stall)
        # the band at the door while the end is on the lug
        on_lug = abs(e[1] - face) < 0.004
        flush = abs(mk[1] - Y_DOOR) <= FLUSH_TOL
        if flush and on_lug:
            self.band.trip(True)
            if self.t1 is None:
                self.t1 = self.steps
                self.d1 = pushed
        # retreat of the rod: where the boat is when the rod starts to come back
        self.e_max = max(self.e_max, float(e[1]))
        if self.band and self.y_ret is None and e[1] < self.e_max - 0.005:
            self.y_ret = face
        if self.y_ret is not None and abs(face - self.y_ret) >= MOVED_BY_DRAW:
            self.dragged.trip(True)
        # hands
        tips = m.fingertips(env)
        rel = tips - e
        u = -(rel @ axis)  # behind the furnace end
        radial = np.linalg.norm(rel + np.outer(u, axis), axis=1)
        if bool(((u < MARK_U - FORWARD_MARGIN) & (u > -0.2) & (radial < HAND_ON_ROD)).any()) and not self.dragged_free():
            self.forward.trip(True)
        loc = (tips - boat.pos) @ boat.rotation()
        near_boat = (np.abs(loc[:, 0]) < BOAT_W / 2 + 0.012) & (np.abs(loc[:, 1]) < BOAT_L / 2 + 0.012) & (loc[:, 2] > -0.012) & (loc[:, 2] < 0.11)
        if near_boat.any():
            self.touched.trip(True)
        # the boat upright and the wafers seated
        tl = boat.tilt_deg()
        seated = True
        for n, w in zip(LOOSE, ws):
            rel_w = (w.pos - boat.pos) @ boat.rotation()
            nom = np.array([0.0, slot_y(n), W_Z])
            ang = math.degrees(math.acos(np.clip(abs(float((boat.rotation().T @ w.rotation())[1, 1])), -1.0, 1.0)))
            if np.linalg.norm(rel_w - nom) > SEAT_MM or ang > SEAT_DEG:
                seated = False
        if tl > TIP_DEG:
            self.fell.trip(True)
        # the final conditions
        at_target = abs(face - TARGET_FACE) <= TARGET_TOL
        lines = [gk.along(rod, (0.0, -ROD_L / 2, 0.0), (0.0, ROD_L / 2, 0.0), 12), gk.along(boat, (0.0, -BOAT_L / 2, 0.02), (0.0, BOAT_L / 2, 0.02), 5)]
        clear = bool(gk.hands_clear(tips, lines, CLEAR))
        gap_l, gap_r = (min(float(gk._to_polyline(tips[i:i + 12], ln, True).min()) for ln in lines) for i in (0, 12))
        rod_out = e[1] < Y_DOOR - 0.01
        rr = rod.rotation()
        hook_up = float(rr[2, 2]) > 0.7
        pos_rod = rod.pos
        on_rests = abs(pos_rod[0] - HOLD_X) < 0.02 and abs(pos_rod[2] - REST_Z) < 0.006 and hook_up and abs(float(axis[2])) < 0.05 and abs(float(axis[1])) > 0.98 \
            and (pos_rod[1] - ROD_L / 2 < HOLD_Y[1] - 0.01) and (pos_rod[1] + ROD_L / 2 > HOLD_Y[0] + 0.005)
        speeds = max(float(np.linalg.norm(boat.lin_vel)), float(np.linalg.norm(rod.lin_vel)))
        still = speeds < 0.005
        mean = None
        if self.t1 is not None and self.t0 is not None and self.t1 > self.t0:
            mean = self.d1 / ((self.t1 - self.t0) * DT)
        speed_ok = mean is not None and V_MEAN[0] <= mean <= V_MEAN[1] and self.max_stall * DT <= STALL_S
        done = bool(at_target and seated and tl <= 3.0 and rod_out and on_rests and clear and still and not self.dragged)
        self.end.step(done)
        failed = bool(self.forward) or bool(self.touched) or bool(self.fell) or bool(self.fast)
        if failed:
            sc = 0.0
        elif self.end.done and speed_ok:
            sc = 1.0
        elif self.band and at_target and seated and speed_ok:
            sc = 0.75
        elif self.band and at_target and seated:
            sc = 0.5
        elif pushed >= PUSH10:
            sc = 0.25
        else:
            sc = 0.0
        self.score, self.full = sc, sc >= 1.0
        self.r = dict(pushed_mm=pushed * 1000, face=face, band_to_door_mm=(mk[1] - Y_DOOR) * 1000, mean_mm_s=(mean or 0) * 1000, stall_s=self.max_stall * DT, seated=seated, tilt=tl,
                      rod_end=float(e[1]), rod_out=rod_out, on_rests=on_rests, hook_up=hook_up, clear=clear, still=still, forward=bool(self.forward), touched=bool(self.touched),
                      fast=bool(self.fast), fell=bool(self.fell), dragged=bool(self.dragged), band=bool(self.band),
                      boat_x_mm=(float(boat.pos[0]) - XR) * 1000, boat_yaw=math.degrees(math.atan2(-float(boat.rotation()[0, 1]), float(boat.rotation()[1, 1]))),
                      gap_l_mm=gap_l * 1000, gap_r_mm=gap_r * 1000, roll=math.degrees(math.atan2(float(rr[0, 2]), float(rr[2, 2]))), win_mm_s=(abs(self.ys[-1] - self.ys[-41]) / 2.0 * 1000 if len(self.ys) > 40 else 0.0))
        log = os.environ.get("FURNACE_LOG")
        if log and self.steps % int(log) == 0:
            print(f"[furnace] {self.steps}: " + " ".join(f"{k} {round(v, 2) if isinstance(v, float) else v}" for k, v in self.r.items()) + f" score {sc}", flush=True)
        gate = os.environ.get("FURNACE_GATE")
        if gate:
            self.gate(env, boat, ws, face)

    def dragged_free(self) -> bool:
        return False

    # the gate: no arms, a pusher on the boat that holds 5.1 mm/s by a force; prints the boat's jitter
    def gate(self, env, boat, ws, face) -> None:
        if not hasattr(self, "_g"):
            self._g = dict(f=1.2, log=[])
        g = self._g
        if self.steps < 40:
            return
        k = self.steps - 40
        v_goal = V_TARGET if k < 3200 else 0.0
        y_goal = self.start_face + V_TARGET * DT * min(k, 3200)
        err = y_goal - face
        v = float(boat.lin_vel[1])
        g["f"] = float(np.clip(1.2 + 100.0 * err + 8.0 * (v_goal - v), 0.0, 5.0))  # a spring-damper pusher about the size of the arm at this rate
        m.push(env, "boat", (0.0, g["f"], 0.0))
        g["log"].append((face, v))
        if k in (0, 60):
            print(f"[gate] boat {np.round(boat.pos, 4)} rod {np.round(m.state(env, 'rod').pos, 4)} wafers " + " ".join(f"{np.round(w.pos, 4)}" for w in ws[:3]), flush=True)
        if k % 100 == 0:
            ys = np.array([a for a, _ in g["log"][-40:]])
            vs = np.array([b for _, b in g["log"][-40:]])
            wmax = max(float(np.linalg.norm(w.pos - boat.pos - boat.rotation() @ np.array([0.0, slot_y(n), W_Z]))) for n, w in zip(LOOSE, ws)) if ws else 0.0
            print(f"[gate] k {k}: face {face:.4f} (goal {y_goal:.4f}) v {v * 1000:.2f} mm/s, vmax/vmin over 2 s {1000 * vs.max():.2f}/{1000 * vs.min():.2f}, F {g['f']:.2f} N, "
                  f"tilt {boat.tilt_deg():.2f}, wafer offset max {wmax * 1000:.2f} mm, score {self.score}", flush=True)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.score >= 1.0


def score(objects) -> float:
    return TRIAL.score


# -- the demo ---------------------------------------------------------------------------------------------------------------------------------
U_R, U_L = 0.83, 0.72  # the hands' grips, behind the furnace end
S_HAND, S_END = 0.62 + YOFF, 0.83 + YOFF  # the furnace end at the hand-over, and at the finish
S_OUT = GUIDE_Y + 0.018  # the rod's end when it is drawn out: 1.8 cm past the guide's middle, 1.2 cm short of the door plane
LIFT_OUT = 0.005  # the hand rises this much while the rod's tail comes back over its rest, so the tail drops into the V from above
LIFT_AT = REST_Y + 0.015 + ROD_L + 0.065  # the rod's end when that rise starts (the tail 6.5 cm short of its rest)
LEAD, CAP = 0.005, 0.012  # the paced goal is never more than LEAD ahead of the rod's end, the command never more than CAP ahead of the hand: a stick never winds up into a surge
GR_KEY = (85.0, 30.0, False, 90.0)  # from the IK dry run (gate_ik.py): fingers 5 degrees off level, 30 degrees across the rod, rolled so the jaws close vertically
GL_KEY = (85.0, 30.0, False, 90.0)


def _grips():
    import demokit as d
    from yam_arena.urdf import axis_angle_matrix

    def roll(g, deg):
        return d._as_grip(g.rot @ axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.radians(deg)))

    th, phi, flip, rl = GR_KEY
    gr = roll(d.tilted(th, phi, flip), rl)
    th, phi, flip, rl = GL_KEY
    gl = roll(d.tilted(th, phi, flip), rl)
    return gr, gl


def demo_script(ctx):
    """A scripted reference, not a policy. It reads the rod's and the boat's poses from the simulator to steer, never the scorer.
    Right hand: takes the rod 83 cm behind its end and pushes it at 1 ft/min until the end is at the hand-over; the left takes it 72 cm behind the end and finishes;
    the left draws it back to the hand-over, the right takes it again and draws it out onto the two V-rests, the tail rising over its rest, then lets go.
    TASK_DEMO: full; none; fast (the whole stroke at 30 mm/s); short (stops 3 cm short of the door); forward (the right hand holds 50 cm behind the end, past the band);
    stall (a 15 s stop at the hand-over); nostow (the rod is left lying in the tube mouth)."""
    from yam_arena.constants import FINGER_OPEN
    from yam_arena import ik

    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode in ("still", "none", "gate"):
        yield from l.wait({"still": 1.0, "none": 3.0, "gate": 165.0}[mode])
        return
    gr, gl = _grips()

    def rod():
        return ctx.state("rod")

    def end_pt():
        return rod_end(rod())

    def at_u(u):
        return lambda: end_pt() - rod().rotation()[:, 1] * u

    def report(label):
        e = end_pt()
        b = ctx.state("boat")
        rr = rod().rotation()
        print(f"[demo] {label}: rod pos {np.round(rod().pos, 4)} axis {np.round(rr[:, 1], 3)} hook up {float(rr[2, 2]):.2f} (roll {math.degrees(math.atan2(float(rr[0, 2]), float(rr[2, 2]))):.0f} deg) "
              f"rod end {np.round(e, 4)}, band {rod_mark(rod())[1]:.4f}, boat face {boat_face(b):.4f}, boat x {1000 * (float(b.pos[0]) - XR):.1f} mm", flush=True)

    GAIN = float(os.environ.get("FURN_GAIN", "0.12"))
    v = {"fast": 0.030}.get(mode, V_TARGET)
    u_r = 0.50 if mode == "forward" else U_R

    def grab(arm, grip, u):
        """Come to the rod at `u` behind its end along the fingers' axis (the jaws close vertically, so the rod enters between them from the side), close the jaws on it."""
        p = at_u(u)
        a = arm.approach_axis(grip)
        pre = lambda: p() - 0.10 * a  # noqa: E731
        yield from arm.move(pre() + np.array([0.0, 0.0, 0.05]), grip=grip, finger=FINGER_OPEN, t=3.0, label="approach")
        yield from arm.move(pre(), t=1.0, label="level")
        yield from arm.servo(lambda s: pre(), tol=0.003, timeout=1.5, label="behind")
        yield from arm.move(p(), t=1.5, label="in")
        yield from arm.servo(lambda s: p(), tol=0.002, timeout=1.5, label="on")
        yield from arm.close(ROD_R * 2, 25.0, label="grip")

    def let_go(arm, up=0.15):
        """Open, slide the fingers back off the rod along their own axis, then rise: rising at once would carry the rod up on the lower finger."""
        yield from arm.open(t=0.6)
        yield from arm.move(arm.at() - 0.10 * arm.approach_axis(), t=1.2, label=f"{arm.side}: back off")
        yield from arm.move(arm.at() + np.array([0.0, 0.0, up]), t=1.2, label=f"{arm.side}: clear")

    def drive(arm, y_to, v_, tol=0.0008, steer=None, lift=None, lead=LEAD, cap=CAP):
        """Move the rod's end along y to `y_to` at `v_` m/s. A paced goal for the end advances at v_ but never more than `lead` ahead of where the end is,
        and the command never leads the hand by more than `cap`: if the boat sticks for a moment the push builds only a bounded force, and it does not surge when it frees.
        `steer`: "boat" puts the rod's end on the boat's axis (into its cup); "line" puts the rod at the hand on the rests' line. `lift(end_y)` raises the hand on the way."""
        e0 = end_pt()
        h0 = arm.tcp().copy()
        off = float(e0[1] - h0[1])
        hx, hz = float(h0[0]), float(h0[2])
        if steer == "boat":
            hx += float(ctx.state("boat").pos[0] - e0[0])
        elif steer == "line":
            hx += XR - float(at_u(off)()[0])
        c = ik.tcp_pose(arm.q, arm.base)[0].copy()
        sgn = 1.0 if y_to > e0[1] else -1.0
        ynom = float(e0[1])
        goal = h0.copy()
        good = 0
        bound = np.array([0.02, cap, 0.03])
        for k in range(int((abs(y_to - float(e0[1])) / v_ + 25.0) / DT)):
            e = end_pt()
            h = arm.tcp()
            ynom += sgn * v_ * DT
            ynom = min(ynom, float(e[1]) + lead, y_to) if sgn > 0 else max(ynom, float(e[1]) - lead, y_to)
            goal = np.array([hx, ynom - off, hz + (lift(float(e[1])) if lift else 0.0)])
            c = c + np.clip(GAIN * (goal - h), -0.005, 0.005)
            c = h + np.clip(c - h, -bound, bound)
            arm._solve(c, arm.grip.rot)
            if k % 20 == 0:
                b = ctx.state("boat")
                print(f"[drive] {arm.side} t {k * DT:.0f}: end {float(e[1]):.4f} goal-end {1000 * (ynom - float(e[1])):.1f} mm, cmd-hand {np.round(1000 * (c - h), 1)} mm, hand {np.round(h, 4)}, "
                      f"face {boat_face(b):.4f} boat x {1000 * (float(b.pos[0]) - XR):.1f} mm, rod x at hand {1000 * (float(at_u(off)()[0]) - XR):.1f} mm, finger {float(ctx.observed(arm.side)[6]):.2f}", flush=True)
            yield
            good = good + 1 if abs(float(end_pt()[1]) - y_to) < tol else 0
            if good >= 3:
                break
        arm.trim = c - goal

    yield from l.wait(0.5)
    # -- the right hand takes the rod and pushes it to the hand-over
    yield from grab(r, gr, u_r)
    yield from r.move(r.at() + np.array([0.0, 0.0, 0.004]), t=0.8, label="lift off the tail rest")  # the tail clears its rest, the end stays in the boat's cup
    report("right has the rod")
    loose = dict(lead=0.03, cap=0.03) if mode == "fast" else {}
    yield from drive(r, S_HAND, v, steer="boat", **loose)
    report("hand-over")
    # -- the left takes over, the right lets go
    yield from grab(l, gl, U_L)
    if mode == "stall":
        yield from l.wait(15.0)
    yield from let_go(r)
    report("left has the rod")
    yield from drive(l, S_END - (0.03 if mode == "short" else 0.0), v, steer="boat", **loose)
    report("pushed")
    yield from l.wait(1.5)
    if mode == "nostow":
        return
    # -- draw back: the left to the hand-over, the right takes the rod and draws it out onto its two V-rests
    yield from drive(l, S_HAND, 0.03, lead=0.03, cap=0.03)
    yield from grab(r, gr, U_R)
    yield from let_go(l)
    report("right has it again")
    rise = lambda ey: LIFT_OUT * float(np.clip((LIFT_AT - ey) / 0.03, 0.0, 1.0))  # noqa: E731
    yield from drive(r, S_OUT, 0.03, steer="line", lift=rise, lead=0.03, cap=0.03)
    report("rod out")
    yield from r.move(r.at() + np.array([0.0, 0.0, -LIFT_OUT - 0.002]), t=1.5, label="lay")  # onto the tail rest and the guide
    yield from r.wait(0.5)
    yield from let_go(r)
    yield from r.wait(2.5)
    report("stowed")
