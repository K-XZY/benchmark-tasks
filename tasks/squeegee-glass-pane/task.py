"""Squeegee a wet upright glass pane dry: keep the rubber edge pressed to the glass at a working angle through each stroke, overlapping the strokes, and
wipe the blade on the towel between strokes (the demo holds the towel up in the other hand).

An aluminium-framed glass pane (34 x 35 cm of glass) stands upright on the bench, clamped, its face toward the arms and wet all over. A window squeegee
with a 25 cm channel rests in a bracket at the right; a folded microfibre towel stands in a holder at the left.

Sim (stated, all in code): water is a grid of 2.5 cm wet cells on the glass, a band along each rail left out. A cell clears only while the rubber edge
passes over it touching the glass, pressed with at least BLADE_MIN_N, and with the blade at 30 to 60 degrees to the glass; a flat, light or lifted
blade clears nothing there. The rubber is a sprung slider in the channel, so its compression gives the pressing force. The blade carries the water it
clears: unless it is wiped on the towel, its next contact with the glass after a lift (0.3 s) wets the cells under it again and its edge streaks every cell it passes until
it is wiped; lifting it off mid-pane leaves the cells under it wet. Water running down the glass is left out. The towel is a rigid pad.

    python demokit.py task.py OUT --mode scene --stills 1
    python demokit.py task.py OUT --mode full
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import demokit as d  # noqa: E402
import mechkit as m  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = ("Squeegee the wet glass pane dry, wiping the blade on the towel between strokes, then set the squeegee back on its stand, let go of it "
               "and take your hands away from the pane.")
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0
HOLD_STEPS = 20

# -- the pane: glass face toward the arms (-x) -----------------------------------------------------------------------------------------------------
GX = 0.35  # the glass face's x
GW, GH, GT = 0.34, 0.35, 0.004  # glass width (y), height (z), thickness: two overlapping strokes of the 25 cm blade cover it, within the arms' reach
GZ0 = 0.04  # the glass's bottom edge above the table
FRAME_W, FRAME_D, FRAME_PROUD = 0.025, 0.030, 0.005  # aluminium frame profile: face width, depth, how far it stands proud of the glass on the arms' side
CELL = 0.025
PANE_Y = 0.03  # the pane's middle, 3 cm toward the left arm, so the near stroke's wrist stays clear of the squeegee's stand
# the scored grid leaves out a band along each rail (2 cm at the sides, 2.5 cm at the top and bottom): every scored cell lies inside what a stroke
# sweeps, and the band by the rails is the edge a cleaner details with the cloth afterwards (a real stroke leaves a bead at the bottom rail)
WET_HALF = 0.15  # the wet grid's half width
WET_Z0 = 0.025  # the band left out at the top and at the bottom
NY, NZ = int(round(2 * WET_HALF / CELL)), int(round((GH - 2 * WET_Z0) / CELL))  # 12 x 12 cells
CELL_Y = PANE_Y - WET_HALF + CELL * (np.arange(NY) + 0.5)
CELL_Z = TOP + GZ0 + WET_Z0 + CELL * (np.arange(NZ) + 0.5)

# -- the squeegee, in its own frame: origin at the channel's middle, +x the rubber's edge (toward the glass), +y along the channel ----------------
CH_L = 0.25  # channel length (a 25 cm / 10 in channel)
RUB_OUT = 0.014  # the rubber's edge ahead of the channel's middle, uncompressed
CH_D = 0.008  # the channel's depth across the rubber, shallow enough that the rubber, not the channel's corner, meets the glass
HANDLE_Z = 0.007  # the handle's axis above the channel's, so their undersides are flush on the stand
RUB_TRAVEL = 0.008  # how far the rubber can be pressed back
BLADE_K = 1000.0  # N/m: the rubber's give, so 4 mm of compression is 4 N along the edge (approximation; how to calibrate in the README)
DRIP_S = 0.0  # water running down is left out (stated in fidelity); the top-down order is then not claimed as tested
BLADE_MIN_N = 2.0  # a cell clears only under at least this pressing force (approximation, to be set from a bench check of the real squeegee, README)
ANGLE_BAND = (30.0, 60.0)  # the blade's angle to the glass that clears water
CONTACT_GAP = 0.0015  # the edge within this of the glass counts as touching
LIFT_STEPS = 6  # off the glass this many steps (0.3 s) is a lift; a shorter loss of contact is a bounce within the stroke
RUB_FLEX = 0.004  # a pressed rubber flexes to bear along its edge where the glass is within this much more; a blade tilted further still leaves a streak
GRIP_L = 0.08  # the hand's grip on the handle, behind the rubber's edge
SQ_REST = np.array([0.30, -0.36, TOP + 0.20])  # the channel's middle, resting in its bracket beside the pane's right edge, 20 cm up, where the hand reaches the handle
SQ_YAW = -90.0  # turned a quarter: the rubber toward -y, the channel along x, the handle toward +y, so the right hand takes it reaching forward
TOWEL = (0.020, 0.12, 0.10)  # the folded towel pad: thickness (x), width (y), height (z)
TOWEL_REST = np.array([0.24, 0.22, TOP + 0.030 + TOWEL[2] / 2])  # standing in its holder

LOOKS = {"glass": m.Look(glass=m.rgb(0.92, 0.96, 0.98)), "alu": m.Look(color=m.rgb(0.70, 0.71, 0.73), metallic=0.8, roughness=0.35),
         "water": m.Look(color=m.rgb(0.45, 0.62, 0.80), roughness=0.05, opacity=0.45), "rubber": m.Look(color=m.rgb(0.05, 0.05, 0.05), roughness=0.7),
         "steel": m.Look(color=m.rgb(0.78, 0.79, 0.80), metallic=1.0, roughness=0.25), "handle": m.Look(color=m.rgb(0.95, 0.75, 0.10), roughness=0.5),
         "towel": m.Look(color=m.rgb(0.20, 0.45, 0.75), roughness=0.95), "dark": m.Look(color=m.rgb(0.15, 0.15, 0.16), roughness=0.6),
         "foot": m.Look(color=m.rgb(0.25, 0.25, 0.27), roughness=0.6)}


def _pane() -> m.Part:
    """The framed pane, clamped upright to the bench (fixed), and the wet cells drawn on the glass's face."""
    zc = TOP + GZ0 + GH / 2
    gx = GX + GT / 2
    shapes = [("glass", (GT, GW, GH), (gx, PANE_Y, zc), "glass")]
    fx = GX - FRAME_PROUD + FRAME_D / 2
    for nm, size, c in (("frame_top", (FRAME_D, GW + 2 * FRAME_W, FRAME_W), (fx, PANE_Y, TOP + GZ0 + GH + FRAME_W / 2)),
                        ("frame_bot", (FRAME_D, GW + 2 * FRAME_W, FRAME_W), (fx, PANE_Y, TOP + GZ0 - FRAME_W / 2)),
                        ("frame_l", (FRAME_D, FRAME_W, GH), (fx, PANE_Y + GW / 2 + FRAME_W / 2, zc)), ("frame_r", (FRAME_D, FRAME_W, GH), (fx, PANE_Y - GW / 2 - FRAME_W / 2, zc))):
        shapes.append((nm, size, c, "alu"))
    for k, y in enumerate((PANE_Y - 0.14, PANE_Y + 0.14)):  # two feet clamped to the bench under the bottom rail
        shapes.append((f"foot_{k}", (0.12, 0.04, GZ0 - FRAME_W), (fx, y, TOP + (GZ0 - FRAME_W) / 2), "foot"))
    parts = [(nm, m.box(s, c), look) for nm, s, c, look in shapes]
    for i, y in enumerate(CELL_Y):
        for j, z in enumerate(CELL_Z):
            parts.append((f"wet_{i}_{j}", m.box((0.0006, CELL - 0.001, CELL - 0.001), (GX - 0.0007, y, z)), "water"))
    cols = [m.col_box(f"c_{nm}", s, c) for nm, s, c, _ in shapes]
    return m.fixed("pane", (0.0, 0.0, 0.0), parts, cols, LOOKS, friction=(0.5, 0.4))


def _squeegee() -> m.Part:
    # 0.18 kg in all (0.17 frame and handle, 0.01 rubber): an Unger ErgoTec 25 cm squeegee, 0.4 lb in the maker's table
    # channel and handle 22 mm deep, flush underneath
    frame = [("channel", m.box((0.012, CH_L, CH_D), (-0.006, 0.0, 0.0)), "steel"),
             ("handle", m.box((0.130, 0.022, 0.022), (-0.077, 0.0, HANDLE_Z)), "handle"),
             ("grip_ring", m.box((0.010, 0.024, 0.022), (-0.140, 0.0, HANDLE_Z)), "dark")]
    cols = [_torsional(m.col_box("c_channel", (0.012, CH_L, CH_D), (-0.006, 0.0, 0.0))), _torsional(m.col_box("c_handle", (0.130, 0.022, 0.022), (-0.077, 0.0, HANDLE_Z)))]
    rub = m.box((RUB_OUT, CH_L - 0.004, 0.004), (RUB_OUT / 2, 0.0, 0.0))
    blade = m.slider("rubber", "X", -RUB_TRAVEL, 0.0, [("rubber", rub, "rubber")], [m.col_box("c_rubber", (RUB_OUT, CH_L - 0.004, 0.004), (RUB_OUT / 2, 0.0, 0.0))],
                     mass=0.01, spring=BLADE_K, damping=5.0, friction=0.0, effort=200.0)
    return m.mechanism("squeegee", tuple(SQ_REST), [blade], LOOKS, frame_parts=frame, frame_colliders=cols, free_mass=0.17, friction=(0.9, 0.8),
                       frame_inertia=(2.0e-3, 0.6e-3, 2.4e-3), yaw_deg=SQ_YAW)


def _torsional(col: str, r: float = 0.012) -> str:
    """A collider with a torsional patch, as mechkit gives free parts (a mechanism's colliders get none, and without it the squeegee spins in the
    hand about the pads' normal when pressed to the glass)."""
    key = "float physxCollision:contactOffset = 0.004"
    assert key in col
    return col.replace(key, f"{key}\n        float physxCollision:torsionalPatchRadius = {r}\n        float physxCollision:minTorsionalPatchRadius = {0.8 * r}")


def _bracket() -> m.Part:
    """A U-bracket on a post that holds the squeegee by its channel, and the towel's holder."""
    # a stand that the squeegee lies on by its channel, the handle overhanging free so the hand can close round it; the channel's middle over
    # the stand
    cx, cy = SQ_REST[0], SQ_REST[1] + 0.001
    zb = SQ_REST[2] - CH_D / 2  # the channel's and the handle's undersides
    # two rests, one under the rubber, the channel and the handle's first 4 cm, one under the handle's end, so the fingers close round the handle
    # between them
    ya, wa = SQ_REST[1] + (0.040 - RUB_OUT) / 2, 0.040 + RUB_OUT
    yb, wb = SQ_REST[1] + 0.121, 0.042
    shapes = [("post_a", (0.03, 0.03, zb - 0.006 - TOP), (cx, ya, TOP + (zb - 0.006 - TOP) / 2)), ("rest_a", (0.07, wa, 0.006), (cx + 0.005, ya, zb - 0.003)),  # 7 cm along x, short, out of the right forearm's way
              ("rest_b", (0.06, wb, 0.006), (cx + 0.010, yb, zb - 0.003)),  # carried from rest A by a side arm, no post of its own
              ("arm_b", (0.010, yb - ya, 0.006), (cx + 0.035, (ya + yb) / 2, zb - 0.003)),
              ("base", (0.10, 0.17, 0.010), (cx, (ya + yb) / 2, TOP + 0.005))]
    tx, ty = TOWEL_REST[0], TOWEL_REST[1]
    shapes += [("towel_base", (0.06, 0.15, 0.030), (tx, ty, TOP + 0.015)),
               ("towel_lip_f", (0.004, 0.15, 0.02), (tx + TOWEL[0] / 2 + 0.004, ty, TOP + 0.040)),
               ("towel_lip_b", (0.004, 0.15, 0.02), (tx - TOWEL[0] / 2 - 0.004, ty, TOP + 0.040))]
    parts = [(nm, m.box(s, c), "dark") for nm, s, c in shapes]
    cols = [m.col_box(f"c_{nm}", s, c) for nm, s, c in shapes]
    return m.fixed("bracket", (0.0, 0.0, 0.0), parts, cols, LOOKS, friction=(0.6, 0.5))


def _towel() -> m.Part:
    return m.free("towel", tuple(TOWEL_REST), [("pad", m.box(TOWEL), "towel")], [m.col_box("c_pad", TOWEL)], LOOKS, mass=0.06, friction=(1.0, 0.9))


OBJECTS = [_pane(), _squeegee(), _bracket(), _towel()]


# -- the water and the scorer ------------------------------------------------------------------------------------------------------------------------

def cell_index(pts):
    """The scored cells under points on the glass: {(column, row)}, the same grid the wet cells are drawn on (CELL_Y, CELL_Z); points in the bands
    left out along the rails fall in no cell."""
    pts = np.asarray(pts, dtype=float).reshape(-1, 3)
    iy = np.floor((pts[:, 1] - (PANE_Y - WET_HALF)) / CELL).astype(int)
    iz = np.floor((pts[:, 2] - (TOP + GZ0 + WET_Z0)) / CELL).astype(int)
    return {(int(a), int(b)) for a, b in zip(iy, iz) if 0 <= a < NY and 0 <= b < NZ}


def edge_points(env, n: int = 21):
    """Points along the rubber's edge in the world, the squeegee's rotation, and the rubber's compression (m)."""
    s = m.state(env, "squeegee")
    comp = -float(s.joint("rubber"))
    ys = np.linspace(-(CH_L - 0.004) / 2, (CH_L - 0.004) / 2, n)
    # the rubber's working edge: of its two front corners (it is 4 mm thick), the one nearer the glass
    sides = [s.to_world(np.column_stack([np.full(n, RUB_OUT - comp), ys, np.full(n, zc)])) for zc in (-0.002, 0.002)]
    pts = sides[0] if sides[0][:, 0].mean() > sides[1][:, 0].mean() else sides[1]
    return pts, s.rotation(), comp, s


class Water:
    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.wet = np.ones((NY, NZ), bool)
        self.dry_since = np.zeros((NY, NZ))
        self.steps, self.carry, self.touching, self.towel_on_glass, self.dropped, self.pane_moved = 0, 0, False, False, False, False
        self.hold, self.full, self.last, self.shown = 0, False, {}, np.zeros((NY, NZ), bool)  # every wet cell shown again on the first step
        self.max_force, self.wiped, self.last_cells, self.streak, self.off = 0.0, 0, set(), False, LIFT_STEPS

    def step(self, env):
        self.steps += 1
        dt = float(env.unwrapped.step_dt)
        pts, Rs, comp, s = edge_points(env)
        gap = GX - pts[:, 0]  # the edge's distance in front of the glass (negative: pressed into it)
        over = (np.abs(pts[:, 1] - PANE_Y) < GW / 2) & (pts[:, 2] > TOP + GZ0) & (pts[:, 2] < TOP + GZ0 + GH)  # over the glass, not the frame
        on_glass = over & (gap < CONTACT_GAP) & (gap > -0.002)  # touching the glass's face itself
        # a rubber pressed with 2 N or more and touching the glass flexes to bear along its edge where the glass is up to RUB_FLEX further off; pressed
        # against anything else (the frame stands 5 mm proud) it bears on nothing
        flex = RUB_FLEX if (BLADE_K * comp >= BLADE_MIN_N and on_glass.any()) else 0.0
        on = over & (gap < CONTACT_GAP + flex) & (gap > -0.002)
        # bearing on the frame: an edge point over a rail (outside the glass, within the frame's 2.5 cm) within 1.5 mm of the rail's face, which stands
        # 5 mm proud. The frame then holds the blade off the glass and carries the press: nothing clears that step, wherever else the edge hovers
        dy, zg = np.abs(pts[:, 1] - PANE_Y) - GW / 2, pts[:, 2] - (TOP + GZ0)
        rail = ((dy >= 0.0) & (dy < FRAME_W) & (zg > -FRAME_W) & (zg < GH + FRAME_W)) | ((dy < FRAME_W) & (((zg <= 0.0) & (zg > -FRAME_W)) | ((zg >= GH) & (zg < GH + FRAME_W))))
        on_frame = bool((rail & (gap < FRAME_PROUD + CONTACT_GAP)).any())
        if on_frame:
            on = np.zeros_like(on)
        force = BLADE_K * comp
        self.max_force = max(self.max_force, force)
        angle = math.degrees(math.acos(min(1.0, abs(float(Rs[:, 2] @ np.array([1.0, 0.0, 0.0]))))))  # between the rubber's plane and the glass
        ok = bool(force >= BLADE_MIN_N and ANGLE_BAND[0] <= angle <= ANGLE_BAND[1])
        touch = bool(on.any())
        # the cells under the touching part of the edge
        cells = cell_index(pts[on])
        if touch and self.off >= LIFT_STEPS and self.carry > 0:  # landing after a lift, unwiped: the water it carries goes back where it lands, and its edge streaks
            for a, b in cells:
                self.wet[a, b] = True
            self.carry, self.streak = 0, True
        if touch and self.streak:  # a dirty edge leaves a streak line on every cell it passes, until it is wiped on the towel
            for a, b in cells:
                self.wet[a, b] = True
        elif touch and ok:
            for a, b in cells:
                if self.wet[a, b]:
                    self.wet[a, b] = False
                    self.carry += 1
        if touch:
            self.off, self.last_cells = 0, cells
        else:
            self.off += 1
            if self.off == LIFT_STEPS and self.carry > 0:  # lifted off: the bead along the edge stays where it lifted, unless it ran off the bottom row
                low = self.last_cells and min(b for _, b in self.last_cells) == 0
                if not low:
                    for a, b in self.last_cells:
                        self.wet[a, b] = True
        self.touching = touch
        # the towel: the rubber's edge drawn along its face clears what the blade carries; the towel on the glass is a fault (it dries glass itself)
        t = m.state(env, "towel")
        tl = t.to_local(pts)
        on_towel = (np.abs(tl[:, 0]) < TOWEL[0] / 2 + 0.003) & (np.abs(tl[:, 1]) < TOWEL[1] / 2) & (np.abs(tl[:, 2]) < TOWEL[2] / 2)
        if on_towel.sum() >= 3 and (self.carry > 0 or self.streak):
            self.carry, self.streak, self.wiped = 0, False, self.wiped + 1
        corners = t.to_world(np.array([[sx * TOWEL[0] / 2, sy * TOWEL[1] / 2, sz * TOWEL[2] / 2] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]))
        if np.any((corners[:, 0] > GX - 0.002) & (np.abs(corners[:, 1] - PANE_Y) < GW / 2) & (corners[:, 2] > TOP + GZ0) & (corners[:, 2] < TOP + GZ0 + GH)):
            self.towel_on_glass = True
        if DRIP_S > 0:  # water running down: a wet cell above a dry one wets it after DRIP_S
            below_dry = (~self.wet[:, :-1]) & self.wet[:, 1:]
            self.dry_since[:, :-1] = np.where(below_dry, self.dry_since[:, :-1] + dt, 0.0)
            drip = self.dry_since[:, :-1] >= DRIP_S
            self.wet[:, :-1] |= drip
            self.dry_since[:, :-1][drip] = 0.0
        if s.pos[2] < TOP - 0.05:
            self.dropped = True
        clear = float((~self.wet).mean())
        tips = m.fingertips(env)
        sq_down = bool(np.linalg.norm(s.pos - SQ_REST) < 0.03)
        # hands clear: every fingertip of both hands 5 cm in front of (or behind) the glass's face, and 3 cm from points along the squeegee's channel
        # and handle (let go of it, not held on its stand)
        sq_pts = s.to_world(np.array([[0.0, y, 0.0] for y in np.linspace(-CH_L / 2, CH_L / 2, 7)] + [[x, 0.0, HANDLE_Z] for x in np.linspace(-0.142, -0.01, 6)]))
        let_go = bool(np.min(np.linalg.norm(tips[:, None, :] - sq_pts[None, :, :], axis=2)) >= 0.03)
        hands_clear = bool(np.min(np.abs(tips[:, 0] - GX)) > 0.05) and let_go
        self.last = {"t": round(self.steps * dt, 2), "clear": round(clear, 3), "carry": int(self.carry), "touch": touch, "force_n": round(force, 1),
                     "angle": round(angle, 1), "ok": ok, "on_frame": on_frame, "wiped": self.wiped, "streak": self.streak, "towel_on_glass": self.towel_on_glass, "dropped": self.dropped,
                     "sq_down": sq_down, "hands_clear": hands_clear, "on_cells": len(cells), "edge_gap_mm": [round(float(gap.min()) * 1000, 1), round(float(gap.max()) * 1000, 1)],
                     "sq_from_rest_mm": [round(float(v) * 1000, 1) for v in (s.pos - SQ_REST)], "edge_z_mm": round(float(pts[:, 2].mean() - TOP) * 1000, 1),
                     "rows_clear": [int((~self.wet[:, j]).sum()) for j in range(NZ)]}
        self.hold = self.hold + 1 if _tier(self.last) == 1.0 else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        self.last["hold"] = self.hold
        changed = self.wet != self.shown
        if changed.any():
            for a, b in zip(*np.nonzero(changed)):
                try:
                    m.set_visible(f"{m.ENV_PATH}/pane/visual/wet_{a}_{b}", bool(self.wet[a, b]))
                except Exception:
                    pass
            self.shown = self.wet.copy()
        if self.steps % 10 == 0:
            print("[sq] " + json.dumps({"mode": os.environ.get("TASK_DEMO", ""), **self.last}), flush=True)


def _tier(r) -> float:
    if not r or r["towel_on_glass"] or r["dropped"]:
        return 0.0
    if r["clear"] >= 0.95 and r["sq_down"] and r["hands_clear"]:
        return 1.0
    return round(0.9 * min(r["clear"], 0.95) / 0.95, 3)


WATER = Water()
m.on_step(WATER.step)
m.on_reset(WATER.reset)


def score(objects) -> float:
    return 1.0 if WATER.full else min(_tier(WATER.last), 0.9)


def success(objects) -> bool:
    return WATER.full


# -- the scripted demo -----------------------------------------------------------------------------------------------------------------------------

PITCH = 35.0  # the squeegee's pitch: the rubber at 55 degrees to the glass, and the handle's and channel's top corners stay in front of the top rail
PRESS = 0.006  # the rubber's edge commanded this far past the glass's face


def sq_rot(pitch_deg: float) -> np.ndarray:
    """The squeegee's rotation for a downward stroke: its edge toward the glass and down, the handle back and up, the channel along y."""
    t = math.radians(pitch_deg)
    sx = np.array([math.cos(t), 0.0, -math.sin(t)])
    sy = np.array([0.0, 1.0, 0.0])
    return np.column_stack([sx, sy, np.cross(sx, sy)])


def hand_rot(Rs: np.ndarray) -> d.Grip:
    """The right hand on the handle from the -y side, its fingers closing across the handle (the squeegee's z): every stroke solves."""
    return d._as_grip(np.column_stack([Rs[:, 2], Rs[:, 0], Rs[:, 1]]))


TOWEL_HOLD = np.array([0.29, 0.19, TOP + 0.19])
TOWEL_AWAY = np.array([0.22, 0.28, TOP + 0.20])  # where the left holds the towel pad up while the blade works, beside the pane and out of the strokes' way
COLUMNS = (PANE_Y + 0.03, PANE_Y - 0.03)  # two overlapping strokes of the 25 cm channel over 34 cm of glass (6 cm apart, 19 cm overlap), its ends 17 mm inside the frame, clear of it
Z_TOP, Z_BOT = TOP + GZ0 + GH - 0.016, TOP + GZ0 + 0.030  # the stroke ends 3 cm over the glass's bottom edge: the arm sags up to about 2 cm there, and the edge must sweep the bottom row without coming down onto the bottom rail
Z_AIM = 0.0  # the stroke's start aimed this much higher (the edge starts 16 mm under the top rail, in the top row of cells)


def demo_script(ctx):
    """Modes: full, the solution. flat (cheat check): the same strokes with the blade held nearly flat, 10 degrees to the glass (pitch 80). light:
    the blade only touching, no press. nowipe: no wipe between strokes. keep: the demo, but the squeegee is left in the hand on its stand. frame
    (cheat check): two strokes with one end of the blade riding a side rail of the frame (5 mm proud of the glass), pressed against the rail, the edge
    hovering over the glass. frametip (cheat check): as frame, the blade turned 1.5 degrees so its inner end comes toward the glass."""
    env = ctx.emb._env
    L, R = ctx.left, ctx.right
    mode = ctx.mode
    if mode == "scene":
        yield from L.wait(1.5)
        print("[sq] END " + json.dumps({"mode": mode, **WATER.last}), flush=True)
        return
    pitch = 80.0 if mode == "flat" else PITCH
    press = 0.0 if mode == "light" else PRESS
    face_x = GX - FRAME_PROUD if mode in ("frame", "frametip") else GX  # the frame checks press on the side rails' face, not the glass
    cols = (PANE_Y + 0.055, PANE_Y - 0.055) if mode in ("frame", "frametip") else COLUMNS  # one end of the blade 1 cm onto a side rail
    # the left takes the towel pad from its holder by its top, across its thickness, and holds it up beside the pane
    gl = d.down(-90.0)
    tw = m.state(env, "towel").pos
    t_grip = tw + np.array([0.0, 0.0, TOWEL[2] / 2 - 0.025])
    # the right takes the squeegee by its handle from the -y side, out of its bracket
    sq = m.state(env, "squeegee")
    Rs0 = sq.rotation()
    g0 = hand_rot(Rs0)
    h_grip = sq.pos + Rs0 @ np.array([RUB_OUT - GRIP_L, 0.0, HANDLE_Z])
    yield from ctx.together(L.move(t_grip + np.array([0.0, 0.0, 0.08]), grip=gl, finger=d.FINGER_OPEN, t=2.0, label="over the towel"),
                            R.move(h_grip - 0.07 * g0.rot[:, 2] + np.array([0.0, 0.0, 0.12]), grip=g0, finger=d.FINGER_OPEN, t=2.0,
                                   label="high over the squeegee's handle"))  # high first, so the forearm passes over the channel
    yield from R.move(h_grip - 0.07 * g0.rot[:, 2], t=1.0, label="beside the squeegee's handle")
    print(f"[sq] beside the handle: squeegee {np.round(1000 * (m.state(env, 'squeegee').pos - SQ_REST), 1)} mm from rest", flush=True)
    yield from ctx.together(L.move(t_grip, t=1.0, label="fingers either side of the towel's top"), R.move(h_grip, t=1.0, label="fingers either side of the handle"))
    yield from R.servo(h_grip, tol=0.002, timeout=1.2, gain=0.3, label="the hand's middle on the handle")  # the arm sags under the reach: the servo centres the jaws on the handle
    print(f"[sq] at the handle: TCP off the grip point by {np.round(1000 * (R.tcp() - h_grip), 1)} mm (commanded {np.round(1000 * (R.at() - h_grip), 1)}); squeegee at {np.round(1000 * (m.state(env, 'squeegee').pos - SQ_REST), 1)} mm from rest", flush=True)
    yield from ctx.together(L.close(TOWEL[0], force=15.0, label="take the towel"), R.close(0.022, force=50.0, label="take the squeegee"))
    yield from ctx.together(L.move(L.at() + np.array([0.0, 0.0, 0.12]), t=1.0, label="lift the towel out"),
                            R.move(R.at() + np.array([0.0, 0.0, 0.06]), t=1.0, label="lift the squeegee off its stand"))
    t_up = np.array([0.0, 0.0, TOWEL[2] / 2 - 0.025])
    yield from L.move(TOWEL_AWAY + t_up, t=1.5, label="hold the towel back from the pane")
    # the squeegee's pose in the hand, as taken: strokes are planned from it
    R.trim = np.zeros(3)  # the servo's shift onto the handle is not carried into the strokes, which are planned from the squeegee's pose in the hand
    held = {}

    def remeasure():
        """The squeegee's pose in the hand, read from the hand's observed pose and the squeegee's: strokes are planned from it."""
        tp, Rt = d.ik.tcp_pose(ctx.observed("right")[:6], R.base)
        s1 = m.state(env, "squeegee")
        held.update(p=Rt.T @ (s1.pos - tp), R=Rt.T @ s1.rotation())

    remeasure()
    print(f"[sq] held: squeegee from the TCP {np.round(1000 * held['p'], 1)} mm", flush=True)

    def put(edge_mid, Rs, t, label, finger=None):
        """Move so the rubber's edge middle is at `edge_mid`, the squeegee turned to `Rs`."""
        Rh = Rs @ held["R"].T
        mid = edge_mid - Rs @ np.array([RUB_OUT, 0.0, 0.0])
        yield from R.move(mid - Rh @ held["p"], grip=d._as_grip(Rh), t=t, label=label)

    Rw = sq_rot(pitch)
    off = -Rw[:, 0]  # back from the glass along the squeegee's own axis
    back_mid = np.array([0.17, -0.30, TOP + 0.30])  # the channel's middle drawn back, its far end clear of the pane's frame
    front_edge = np.array([0.25, PANE_Y, TOP + 0.40])  # turned to work in front of the pane

    def via_front(to_work: bool):
        if to_work:
            yield from put(back_mid + Rs0 @ np.array([RUB_OUT, 0.0, 0.0]), Rs0, 1.2, "the squeegee drawn back, clear of the frame")
            yield from put(front_edge, Rw, 2.0, "turned to the pane")
        else:
            yield from put(front_edge, Rw, 1.5, "back in front of the pane")
            yield from put(back_mid + Rs0 @ np.array([RUB_OUT, 0.0, 0.0]), Rs0, 2.0, "turned back beside the stand")

    yield from via_front(True)
    for k, yc in enumerate(cols):
        yield from put(np.array([face_x, yc, Z_TOP + Z_AIM]) + 0.03 * off, Rw, 2.0 if k == 0 else 1.5, f"stroke {k + 1}: the edge before the pane's top")
        yield from R.wait(0.3)
        remeasure()  # the blade squared to the glass again from how it sits in the hand now
        Rk = Rw
        for _ in range(2):  # the edge's ends read against the glass, and the blade turned about the vertical to even them
            yield from put(np.array([face_x, yc, Z_TOP + Z_AIM]) + 0.03 * off, Rk, 0.5, f"stroke {k + 1}: the blade squared to the glass")
            yield from R.wait(0.2)
            pts = edge_points(env)[0]
            phi = float(np.clip(math.atan2(float(pts[-1, 0] - pts[0, 0]), float(pts[-1, 1] - pts[0, 1])), -math.radians(3.0), math.radians(3.0)))
            Rk = d.axis_angle_matrix(np.array([0.0, 0.0, 1.0]), phi) @ Rk
            if mode == "frametip":  # turned 1.5 degrees on the rail, the inner end toward the glass
                Rk = d.axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.radians(1.5 if yc > PANE_Y else -1.5)) @ Rk
            print(f"[sq] stroke {k + 1} squaring: end gaps {1000 * (GX - pts[0, 0]):.1f} and {1000 * (GX - pts[-1, 0]):.1f} mm, edge middle {np.round(1000 * (pts.mean(axis=0) - np.array([GX, yc, Z_TOP]) - 0.03 * off), 1)} mm off its place, turn {math.degrees(phi):.2f} deg", flush=True)
        dz = float((Z_TOP + Z_AIM + 0.03 * off[2]) - edge_points(env)[0][:, 2].mean())  # where the edge sits against its command now, measured
        zt = Z_TOP + Z_AIM + dz  # the stroke starts where the edge actually sits against its command
        cap = 0.0075 if yc > PANE_Y else 0.0055  # the near stroke (on the right arm's side) presses less: deeper, the squeegee turns in the hand near the bottom
        push = press
        yield from put(np.array([face_x + push, yc, zt]), Rk, 0.8, f"stroke {k + 1}: the edge on, pressed")
        print(f"[sq] stroke {k + 1} on: " + json.dumps({kk: WATER.last[kk] for kk in ("touch", "force_n", "angle", "on_cells", "edge_gap_mm", "edge_z_mm")}), flush=True)
        for _ in range(4):  # pressed in at the top until the rubber gives 3 mm, so the top row clears too
            if press <= 0 or m.state(env, "squeegee").joint("rubber") <= -0.003:
                break
            push = min(push + 0.0015, cap)
            yield from put(np.array([face_x + push, yc, zt]), Rk, 0.3, f"stroke {k + 1}: pressed in at the top")
        for j, z in enumerate(np.linspace(zt, Z_BOT, 7)[1:]):  # drawn down in six steps, pressed deeper while the rubber gives less than 3 mm
            if press > 0 and m.state(env, "squeegee").joint("rubber") > -0.003:
                push = min(push + 0.0015, cap)
            yield from put(np.array([face_x + push, yc, z]), Rk, 0.5, f"stroke {k + 1}: draw it down, pressed ({j + 1})")
        yield from put(np.array([face_x, yc, Z_BOT]) + 0.03 * off, Rk, 0.6, f"stroke {k + 1}: off at the bottom rail")
        print(f"[sq] stroke {k + 1}: " + json.dumps(WATER.last), flush=True)
        if mode != "nowipe":  # the towel brought in, the rubber wiped down its face, the towel taken back
            yield from L.move(TOWEL_HOLD + t_up, t=1.0, label="the towel up for the wipe")
            tw = m.state(env, "towel")
            face = tw.pos + tw.rotation() @ np.array([-TOWEL[0] / 2, 0.0, 0.0])
            ztop = tw.pos[2] + TOWEL[2] / 2 - 0.045
            yield from put(np.array([face[0], face[1], ztop]) + 0.03 * off, Rw, 1.5, "the edge before the towel")
            yield from put(np.array([face[0] + 0.003, face[1], ztop]), Rw, 0.5, "the edge on the towel")
            yield from put(np.array([face[0] + 0.003, face[1], ztop - 0.04]), Rw, 0.8, "wipe it down the towel")
            yield from put(np.array([face[0], face[1], ztop - 0.04]) + 0.04 * off, Rw, 0.5, "off the towel")
            print(f"[sq] wiped: carry {WATER.carry}, wipes {WATER.wiped}", flush=True)
            yield from L.move(TOWEL_AWAY + t_up, t=1.0, label="the towel back")
    # the squeegee back on its stand, the towel back in its holder, both hands clear
    yield from via_front(False)
    yield from put(SQ_REST + Rs0 @ np.array([RUB_OUT, 0.0, 0.0]) + np.array([0.0, 0.0, 0.06]), Rs0, 2.0, "over the bracket")
    yield from put(SQ_REST + Rs0 @ np.array([RUB_OUT, 0.0, 0.0]) + np.array([0.0, 0.0, 0.003]), Rs0, 1.0, "into the bracket")
    if mode == "keep":  # keep (lazy check): the squeegee left in the hand on its stand
        yield from L.move(L.at() + np.array([-0.03, 0.0, 0.03]), t=1.0, label="the towel held back from the pane")
    else:
        yield from ctx.together(R.open(label="let go of the squeegee"),
                                L.move(L.at() + np.array([-0.03, 0.0, 0.03]), t=1.0, label="the towel held back from the pane"))  # the towel stays in the hand: stood back in its holder it can tip onto the glass
        yield from R.move(R.at() - 0.07 * R.grip.rot[:, 2], t=1.0, label="right clear")
    yield from L.wait(1.5)
    print("[sq] END " + json.dumps({"mode": mode, **WATER.last}), flush=True)
