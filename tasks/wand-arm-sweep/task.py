"""Screen a mannequin's outstretched arm with a Garrett SuperWand: a sweep over the top, then under the arm, slow at the wrist watch until the wand alarms.

Garrett SuperWand 1165800: 48.3 x 8.3 x 3.2 cm, 0.45 kg (19 in; the manual's 16 oz, spec sheets list 18.6 oz); a 12 cm handle 3.2 cm square; the mass spread through its shape puts the centre of mass
17 cm beyond the grip point; its scan surface is the far 22 cm. The manual (1531500.B): it detects metal only while moving; scan within about two inches (5 cm) of the person; red LED and a tone on an alarm.
Casio MTP-1302D steel watch: case 44.2 x 38.5 x 9.2 mm, 105 g with its band.
Mannequin arm: 70 cm shoulder to fingertip, 10 cm round tapering to 5.5 cm at the wrist, a flat hand beyond it, held out sideways palm down on a stand.
The right hand carries the wand; the left stays at rest. The wand lies flat on a two-post holder under the arm's shoulder end.

Debug:  python demokit.py task.py OUT --no-record --stills 100 --mode full
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
INSTRUCTION = ("Screen the mannequin's arm with the metal-detector wand: take the wand from its holder by its grip and sweep it over the top of the arm from shoulder to fingertips, "
               "then under the arm back to the shoulder, keeping the coil about 5 cm (two inches) off the arm and never touching it. Keep the wand moving, and slow down at the wristwatch until the wand's red lamp lights. "
               "Then lay the wand back flat in its holder, let go and move your hand clear.")
SUCCESS_HOLD_S = 0.0  # the scorer holds the end state itself (HOLD_STEPS)
SCORE_OVER = "end"
TOP = d.TABLE_TOP

# -- the mannequin arm (m; x forward from the arm bases, y to the robot's left, z above the table) ---------------------------------------------
AX = float(os.environ.get("AX", "0.32"))
YS = float(os.environ.get("YS", "-0.45"))
DIR = float(os.environ.get("ARM_DIR", "1.0"))  # the fingertips lie toward +y (1) or -y (-1) of the shoulder
ZA = TOP + float(os.environ.get("ZA", "0.32"))  # the arm's axis above the table
ARM_L, WRIST_S = 0.70, 0.52
R_SH, R_WR = 0.05, 0.0275
HAND_W, HAND_T = 0.08, 0.04
WATCH = (0.0442, 0.0385, 0.0092)  # along the arm, across, high
WATCH_S = WRIST_S - 0.06  # the watch's centre, 6 cm short of the wrist end of the forearm
BAND_R = R_WR + 0.0005

# -- the wand: local frame origin at the grip, +x along the wand toward the coil end, y the wide side, z the thin side ----------------------------
W_LEN, W_WIDE, W_THIN = 0.483, 0.083, 0.032
U_GRIP = 0.10  # the grip's centre, from the handle end (the handle is 12 cm: the pads sit at its far end, nearest the centre of mass)
HANDLE_L = 0.12  # a square 3.2 cm handle; the 8.3 cm body follows it
COIL_U = (0.263, 0.483)  # the scan surface, from the handle end
COIL_C = 0.5 * (COIL_U[0] + COIL_U[1]) - U_GRIP  # its centre in the wand frame
COIL_HALF = np.array([0.5 * (COIL_U[1] - COIL_U[0]), W_WIDE / 2, W_THIN / 2])
W_MASS = 0.45

# -- the holder ("hook"), to the right of the right arm's base --------------------------------------------------------------------------------
HOOK = (float(os.environ.get("HX", "0.32")), -0.62)  # the wand lies along +y from here, its grip point over HOOK
POST_H = float(os.environ.get("POST_H", "0.1035"))  # the grip lies 12 cm above the table
WAND_AT = (HOOK[0], HOOK[1], TOP + POST_H + W_THIN / 2 + 0.0005)

# -- scoring ----------------------------------------------------------------------------------------------------------------------------------
STAND_OFF = 0.05  # the maker's two inches
BAND_MAX = 0.07  # a stretch of the arm counts as screened while the coil's nearest point is within this of its surface
TOUCH = 0.002
FINGER_TOUCH = 0.004  # fingertip to the arm
DROP_H = 0.015  # the wand this far above its holder with no fingertip within 3 cm of it has been dropped
ALARM_RANGE = 0.05  # coil surface to the watch case
MOVING = 0.02  # m/s: the wand detects only while moving
HOLD_STEPS = 40
BINS = 14

LOOKS = {
    "skin": m.Look(m.rgb(0.80, 0.70, 0.62), roughness=0.6),
    "stand": m.Look(m.rgb(0.20, 0.20, 0.22), roughness=0.5),
    "steel": m.Look(m.rgb(0.70, 0.71, 0.74), metallic=0.9, roughness=0.3),
    "dial": m.Look(m.rgb(0.06, 0.07, 0.09), roughness=0.3),
    "black": m.Look(m.rgb(0.05, 0.05, 0.06), roughness=0.6),
    "coil": m.Look(m.rgb(0.10, 0.10, 0.12), roughness=0.5),
    "lamp_on": m.Look(m.rgb(1.0, 0.05, 0.05), roughness=0.3),
    "lamp_off": m.Look(m.rgb(0.25, 0.04, 0.04), roughness=0.4),
    "holder": m.Look(m.rgb(0.55, 0.55, 0.58), metallic=0.5, roughness=0.5),
}


def arm_r(s):
    s = float(np.clip(s, 0.0, WRIST_S))
    return R_SH + (R_WR - R_SH) * s / WRIST_S


def arm_pt(s: float) -> np.ndarray:
    return np.array([AX, YS + DIR * s, ZA])


def _along(mesh: m.Mesh) -> m.Mesh:
    """A mesh built along +z, turned to run along the arm's direction (+y, or -y)."""
    return mesh.turned(-90.0 if DIR > 0 else 90.0, "x")


def _arm():
    fore = _along(m.lathe([(0.0, 0.0), (R_SH, 0.0), (R_WR, WRIST_S), (0.0, WRIST_S)], 40))
    hand_c = (0.0, DIR * (WRIST_S + (ARM_L - WRIST_S) / 2), 0.0)
    hand_size = (HAND_W, ARM_L - WRIST_S, HAND_T)
    wz = R_WR + WATCH[2] / 2
    wc = (0.0, DIR * WATCH_S, wz)
    ws = (WATCH[1], WATCH[0], WATCH[2])
    band = _along(m.cylinder(BAND_R, 0.014, (0.0, 0.0, WATCH_S), axis="z", segments=24))
    bracket_c = (0.07, -DIR * 0.02, 0.0)
    parts = [("forearm", fore, "skin"), ("hand", m.box(hand_size, hand_c), "skin"), ("case", m.box(ws, wc), "steel"),
             ("dial", m.box((ws[0] * 0.8, ws[1] * 0.8, 0.0005), (wc[0], wc[1], wc[2] + ws[2] / 2 + 0.0002)), "dial"), ("band", band, "black"),
             ("bracket", m.box((0.14, 0.03, 0.03), bracket_c), "stand"),
             ("mast", m.box((0.04, 0.04, ZA - TOP), (0.14, -DIR * 0.02, -(ZA - TOP) / 2)), "stand"),
             ("foot", m.box((0.14, 0.14, 0.016), (0.14, -DIR * 0.02, -(ZA - TOP) + 0.008)), "stand")]
    cols = [m.col_hull("fore_c", fore), m.col_box("hand_c", hand_size, hand_c), m.col_box("case_c", ws, wc),
            m.col_box("bracket_c", (0.14, 0.03, 0.03), bracket_c), m.col_box("mast_c", (0.04, 0.04, ZA - TOP), (0.14, -DIR * 0.02, -(ZA - TOP) / 2))]
    return m.fixed("arm", (AX, YS, ZA), parts, cols, LOOKS, friction=(0.8, 0.7))


def _holder():
    cols, parts = [], []
    for nm, y in (("front", -0.06), ("rear", 0.20)):  # under the handle and under the body, either side of the centre of mass
        size = (0.03, 0.03, POST_H - 0.0002)
        c = (0.0, y, (POST_H - 0.0002) / 2)
        cols.append(m.col_box(nm + "_c", size, c))
        parts.append((nm, m.box(size, c), "holder"))
    # side stops: low blocks either side of the handle, so the wand cannot slide off sideways
    for nm, x in (("lstop", 0.026), ("rstop", -0.026)):
        size = (0.006, 0.03, POST_H + 0.012)
        c = (x, -0.06, (POST_H + 0.012) / 2)
        cols.append(m.col_box(nm + "_c", size, c))
        parts.append((nm, m.box(size, c), "holder"))
    return m.fixed("holder", (HOOK[0], HOOK[1], TOP), parts, cols, LOOKS, friction=(0.7, 0.6))


def _wand():
    h0 = -U_GRIP
    body_len = W_LEN - HANDLE_L
    hc = (h0 + HANDLE_L / 2, 0.0, 0.0)
    bc = (h0 + HANDLE_L + body_len / 2, 0.0, 0.0)
    coil_len = COIL_U[1] - COIL_U[0]
    cc = (COIL_C, 0.0, 0.0)
    lamp_c = (h0 + HANDLE_L + 0.03, 0.0, W_THIN / 2 + 0.002)
    parts = [("handle", m.box((HANDLE_L, W_THIN, W_THIN), hc), "black"), ("body", m.box((body_len, W_WIDE, W_THIN), bc), "black"),
             ("coil", m.box((coil_len, W_WIDE * 0.9, W_THIN * 1.02), cc), "coil"),
             ("lamp_off", m.box((0.012, 0.012, 0.004), lamp_c), "lamp_off"), ("lamp_on", m.box((0.014, 0.014, 0.006), lamp_c), "lamp_on", True)]
    cols = [m.col_box("handle_c", (HANDLE_L, W_THIN, W_THIN), hc), m.col_box("body_c", (body_len, W_WIDE, W_THIN), bc)]
    return m.free("wand", WAND_AT, parts, cols, LOOKS, mass=W_MASS, friction=(0.9, 0.8), start_quat=(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)))


OBJECTS = [_arm(), _holder(), _wand()]


# -- geometry of the readings -------------------------------------------------------------------------------------------------------------------

def _grid(half, n=(9, 5, 3)):
    ax = [np.linspace(-h, h, k) for h, k in zip(half, n)]
    return np.array([(a, b, c) for a in ax[0] for b in ax[1] for c in ax[2]])


COIL_PTS = _grid(COIL_HALF) + np.array([COIL_C, 0.0, 0.0])
_h0 = -U_GRIP
BODY_PTS = np.vstack([_grid(np.array([HANDLE_L / 2, W_THIN / 2, W_THIN / 2]), (6, 3, 3)) + np.array([_h0 + HANDLE_L / 2, 0, 0]),
                      _grid(np.array([(W_LEN - HANDLE_L) / 2, W_WIDE / 2, W_THIN / 2]), (16, 5, 3)) + np.array([_h0 + HANDLE_L + (W_LEN - HANDLE_L) / 2, 0, 0])])


def arm_gap(pts: np.ndarray) -> float:
    """Smallest distance from world points to the arm's surface (forearm cone, hand slab, watch case), negative inside."""
    rel = pts - np.array([AX, YS, ZA])
    s = rel[:, 1] * DIR
    dx, dz = rel[:, 0], rel[:, 2]
    best = 9.9
    # forearm: a cone of the arm's radius at each point's own s
    sel = (s >= 0.0) & (s <= WRIST_S)
    if sel.any():
        r = R_SH + (R_WR - R_SH) * s[sel] / WRIST_S
        best = min(best, float((np.hypot(dx[sel], dz[sel]) - r).min()))
    # ends of the forearm (a point before the shoulder or past the wrist end is measured to the nearest edge)
    out = (s < 0.0) & (np.hypot(dx, dz) < R_SH)
    if out.any():
        best = min(best, float((-s[out]).min()))
    # hand slab
    hs = (s >= WRIST_S - 0.02) & (s <= ARM_L)
    if hs.any():
        ex = np.abs(dx[hs]) - HAND_W / 2
        ez = np.abs(dz[hs]) - HAND_T / 2
        outside = np.hypot(np.maximum(ex, 0), np.maximum(ez, 0))
        inside = np.minimum(np.maximum(ex, ez), 0)
        best = min(best, float((outside + inside).min()))
    # watch case
    ws = rel - np.array([0.0, DIR * WATCH_S, R_WR + WATCH[2] / 2])
    e = np.abs(ws) - np.array([WATCH[1], WATCH[0], WATCH[2]]) / 2
    best = min(best, float((np.hypot(np.hypot(np.maximum(e[:, 0], 0), np.maximum(e[:, 1], 0)), np.maximum(e[:, 2], 0)) + np.minimum(e.max(axis=1), 0)).min()))
    return best


def watch_gap(pts: np.ndarray) -> float:
    ws = pts - (np.array([AX, YS + DIR * WATCH_S, ZA + R_WR + WATCH[2] / 2]))
    e = np.abs(ws) - np.array([WATCH[1], WATCH[0], WATCH[2]]) / 2
    return float((np.sqrt((np.maximum(e, 0) ** 2).sum(axis=1)) + np.minimum(e.max(axis=1), 0)).min())


def lamp_path(name: str) -> str:
    return f"{m.ENV_PATH}/wand/{name}"


class Log:
    def reset(self, env=None):
        self.n = 0
        self.prev = None
        self.touched = gk.Latch()
        self.dropped = gk.Latch()
        self.alarm_steps = 0
        self.alarmed = False
        self.lit = False
        self.bins_top = np.zeros(BINS, bool)
        self.bins_under = np.zeros(BINS, bool)
        self.top_done = False
        self.under_done = False
        self.speeds = []
        self.start_pos = None
        self.end = gk.EndHold(HOLD_STEPS)
        self.score, self.full = 0.0, False
        self.r = {}
        self.min_gap = 9.9
        self.gap_log = []

    def step(self, env):
        self.n += 1
        w = m.state(env, "wand")
        R, p = w.rotation(), w.pos
        speed = 0.0 if self.prev is None else float(np.linalg.norm(p - self.prev)) / d.DT
        self.prev = p.copy()
        if self.start_pos is None and self.n > 20:
            self.start_pos = p.copy()
        coil = COIL_PTS @ R.T + p
        body = BODY_PTS @ R.T + p
        cg = arm_gap(coil)
        bg = arm_gap(body)
        if self.n > 20 and bg <= TOUCH:
            if not self.touched:
                gaps = [arm_gap(q[None, :]) for q in body]
                k = int(np.argmin(gaps))
                loc = body[k] - np.array([AX, YS, ZA])
                print(f"[wand] FIRST TOUCH step {self.n}: gap {1000 * bg:.1f} mm at wand point {np.round(BODY_PTS[k], 3)}, arm coords (s {loc[1] * DIR:.3f}, dx {loc[0]:.3f}, dz {loc[2]:.3f}), wand axis {np.round(R[:, 0], 3)}, thin {np.round(R[:, 2], 3)}", flush=True)
            self.touched.trip(True)
        # the fingers and hand: any fingertip on the mannequin's arm is a touch too, as is any point of the wand
        tips = m.fingertips(env)
        if self.n > 20 and arm_gap(gk.all_tips(tips)) <= FINGER_TOUCH:
            self.touched.trip(True)
        # alarm: the coil within range of the watch, and moving
        wg = watch_gap(coil)
        moving = speed >= MOVING
        on = bool(wg <= ALARM_RANGE and moving and not self.touched)
        self.lit = on
        if on:
            self.alarm_steps += 1
        if self.alarm_steps >= 2 and not self.touched:
            self.alarmed = True
        if env is not None:
            try:
                m.set_visible(lamp_path("lamp_on"), on)
                m.set_visible(lamp_path("lamp_off"), not on)
            except Exception:  # the lamp is only seen, the score does not need it
                pass
        # passes: the arm's length in 5 cm stretches; a stretch is screened while the part of the moving coil over it is centred across the arm
        # (its middle within the arm's half width of the arm's centre line), lies wholly higher than the arm's top there (or wholly lower than its
        # underside) and is within the band of the arm: a sweep along the arm's side, or low beside it, does not count
        if self.n > 20 and moving and 0.002 < cg <= BAND_MAX and not self.touched:
            ss = (coil[:, 1] - YS) * DIR
            fore = R_SH + (R_WR - R_SH) * np.clip(ss, 0.0, WRIST_S) / WRIST_S
            half = np.where(ss <= WRIST_S, fore, HAND_T / 2)  # the arm's half height at each point
            wide = np.where(ss <= WRIST_S, fore, HAND_W / 2)  # and its half width
            dz = coil[:, 2] - ZA
            for k in range(BINS):
                lo, hi = k * ARM_L / BINS, (k + 1) * ARM_L / BINS
                inb = (ss >= lo) & (ss <= hi)
                if not inb.any() or arm_gap(coil[inb]) > BAND_MAX or abs(float(coil[inb, 0].mean()) - AX) > float(wide[inb].min()):
                    continue
                if (dz[inb] > half[inb]).all():
                    self.bins_top[k] = True
                elif self.top_done and (-dz[inb] > half[inb]).all():
                    self.bins_under[k] = True
        self.top_done = self.top_done or bool(self.bins_top.all())
        self.under_done = self.under_done or bool(self.top_done and self.bins_under.all())
        # the wand back in its holder, let go
        home = False
        if self.start_pos is not None:
            home = bool(np.linalg.norm(p[:2] - self.start_pos[:2]) < 0.03 and abs(p[2] - self.start_pos[2]) < 0.01 and R[2, 2] > 0.99)
        clear = bool(gk.hands_clear(tips, [gk.along(w, (-U_GRIP, 0, 0), (W_LEN - U_GRIP, 0, 0), 9)], 0.03))
        # a dropped wand: it is off the holder's level with no fingertip near it. Set down means held until it rests on the holder.
        if self.start_pos is not None and clear and p[2] > self.start_pos[2] + DROP_H:
            self.dropped.trip(True)
        self.speeds = (self.speeds + [speed])[-20:]
        still = len(self.speeds) == 20 and max(self.speeds) < 0.01
        done = bool(self.under_done and self.alarmed and home and clear and still and not self.touched and not self.dropped)
        self.end.step(done)
        if self.touched:
            sc = 0.0
        elif self.end.done:
            sc = 1.0
        elif self.alarmed and self.under_done:
            sc = 0.75
        elif self.alarmed and self.top_done:
            sc = 0.5
        elif self.alarmed:
            sc = 0.25
        else:
            sc = 0.0
        self.score, self.full = sc, sc >= 1.0
        self.min_gap = min(self.min_gap, bg) if self.n > 20 else 9.9
        self.r = dict(speed=speed, coil_gap=cg, body_gap=bg, watch_gap=wg, lit=on, alarmed=self.alarmed, top=int(self.bins_top.sum()), under=int(self.bins_under.sum()),
                      touched=bool(self.touched), dropped=bool(self.dropped), home=home, clear=clear, still=still)
        if os.environ.get("WAND_LOG") and self.n % int(os.environ["WAND_LOG"]) == 0:
            print(f"[wand] {self.n}: " + " ".join(f"{k} {round(v, 3) if isinstance(v, float) else v}" for k, v in self.r.items()) + f" score {sc}", flush=True)


LOG = Log()
LOG.reset()
m.on_step(LOG.step)
m.on_reset(LOG.reset)


def success(objects) -> bool:
    return LOG.score >= 1.0


def score(objects) -> float:
    return LOG.score


# -- the demo ---------------------------------------------------------------------------------------------------------------------------------

import demokit as _dk  # noqa: E402
from yam_arena import ik as _ik  # noqa: E402
from yam_arena.urdf import axis_angle_matrix as _aam  # noqa: E402

S0, S1 = 0.06, 0.62  # coil centre along the arm: first and last of a pass
S0U = 0.12  # and where the underside pass ends: the coil's shoulder end still reaches 1 cm past the first 5 cm of the arm
SIDE = float(os.environ.get("SIDE", "-1"))  # +1: the roll goes round the arm's far side (away from the robot), -1 round its near side
RHO = float(os.environ.get("RHO", "0.14"))  # the roll under the arm: the wand, kept parallel to it, circles it at this radius
HZ = COIL_HALF[2]  # the wand is swept flat: half its thickness is below the stand-off
GO = float(os.environ.get("DEMO_STAND", "0.045"))  # the stand-off the demo keeps
V = float(os.environ.get("DEMO_V", "0.10"))  # the sweep speed, m/s
VS = 0.04  # and over the watch
LEVEL_K = float(os.environ.get("LEVEL_K", "0.04"))  # how much of the wand's tilt the hand takes out each step while it sweeps
TRIM_CAP = float(os.environ.get("TRIM_CAP", "0.10"))  # the most the sweep's correction may add to the planned path, m, on each axis: the loaded arm droops up to about 6 cm far out
TRACK_K = float(os.environ.get("TRACK_K", "0.15"))  # the share of the coil's error the correction takes up each step


def _roll(g, deg):
    return _dk._as_grip(g.rot @ _aam(np.array([0.0, 0.0, 1.0]), math.radians(deg)))


def _hook_grip():
    """One grip from the holder to the end: fingers level toward +x, jaws closing vertically on the flat handle, the wand's axis along +y."""
    g = _roll(_dk.tilted(90, 0, False), 90)
    assert g.rot[1, 1] > 0.99
    return g


def _rotvec(R):
    """The rotation vector (axis times angle) of a rotation matrix."""
    ang = math.acos(float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)))
    if ang < 1e-6:
        return np.zeros(3)
    ax = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2.0 * math.sin(ang))
    return ax * ang


BACK = 0.12  # the wand is drawn this far toward the robot before it rises past the arm


def demo_script(ctx):
    """Modes: full (both passes, back on the holder); top (the top pass only); far (both passes at 11 cm); touch (pressing into the arm); hover (held still at the watch);
    side (the top pass, then the second pass along the arm's near side just below its centre line instead of under it, then back on the holder); drop (the full run, but let go 3 cm above the holder); none; pick."""
    if ctx.mode not in ("full", "top", "none", "pick", "far", "touch", "hover", "side", "drop"):
        raise ValueError(f"unknown mode {ctx.mode!r}")
    r = ctx.right
    if ctx.mode == "none":
        yield from r.wait(3.0)
        return
    Rh = _hook_grip()
    go = {"far": 0.11, "touch": -0.02}.get(ctx.mode, GO)  # far: past the maker's two inches; touch: into the arm

    def wand():
        return ctx.state("wand")

    def coil_c():
        w = wand()
        return w.pos + w.rotation() @ np.array([COIL_C, 0.0, 0.0])

    def grip_pt():
        return wand().pos

    def coil_to_tcp(c):
        return c - wand().rotation() @ np.array([COIL_C, 0.0, 0.0])

    def line_r(s):
        return R_SH + (R_WR - R_SH) * s / WRIST_S  # the forearm's top (or bottom) line, carried on past the wrist over the thinner hand

    def top_z(s):
        return ZA + line_r(s) + go + HZ

    def under_z(s):
        return ZA - line_r(s) - go - HZ

    W_des = m.quat_matrix((0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)))  # flat, its axis along +y: the way it lay in the holder
    slope = math.atan2(R_SH - R_WR, WRIST_S)  # the forearm's taper: the wand is held parallel to its surface, so the whole wand keeps the stand-off
    W_top = _aam(np.array([1.0, 0.0, 0.0]), -DIR * slope) @ W_des  # nose down toward the wrist over the top
    W_under = _aam(np.array([1.0, 0.0, 0.0]), DIR * slope) @ W_des  # nose up toward the wrist underneath
    aim = [W_des]

    def tilt():
        return aim[0] @ wand().rotation().T

    def track(goal, T, label, settle=1.0, k=TRACK_K):
        """Carry the coil along goal(t) for T s. The command follows the goal's own motion, plus a correction that closes the coil's error,
        capped at TRIM_CAP so that a wand that cannot follow never winds the command out of reach; the hand takes out a share of the wand's tilt each step."""
        c0 = _ik.tcp_pose(r.q, r.base)[0]
        g0 = np.asarray(goal(0.0), float)
        trim = np.zeros(3)
        rot = r.grip.rot.copy()
        worst = (0.0, 0.0)
        n = round((T + settle) * _dk.CONTROL_HZ)
        for k_ in range(n):
            t = min(k_ * _dk.DT, T)
            g = np.asarray(goal(t), float)
            err = g - coil_c()
            trim = np.clip(trim + np.clip(k * err, -0.004, 0.004), -TRIM_CAP, TRIM_CAP)
            if LEVEL_K > 0:
                rv = _rotvec(tilt())
                rot = _aam(rv / max(np.linalg.norm(rv), 1e-9), LEVEL_K * float(np.linalg.norm(rv))) @ rot if np.linalg.norm(rv) > math.radians(0.5) else rot
            mm, deg = r._solve(c0 + (g - g0) + trim, rot)
            worst = (max(worst[0], mm), max(worst[1], deg))
            if k_ % 20 == 0:
                print(f"[track] {label} t {t:.1f}: err {np.round(1000 * err, 1)} mm, trim {np.round(1000 * trim, 1)} mm, tilt {math.degrees(np.linalg.norm(_rotvec(tilt()))):.1f} deg, ik {mm:.1f} mm {deg:.1f} deg", flush=True)
            yield
        r.grip = _dk._as_grip(rot)
        r._note(*worst, label)

    # take the wand by its grip, the fingers coming level from the robot's side
    yield from r.wait(0.5)
    g0 = ctx.pos("wand")
    a_h = Rh.rot[:, 2]
    pre = g0 - 0.10 * a_h
    yield from r.move(pre, grip=Rh, finger=_dk.FINGER_OPEN, t=3.0, label="beside the grip")
    yield from r.servo(pre, timeout=1.0)
    yield from r.move(g0, t=1.2, label="on the grip")
    yield from r.servo(g0, timeout=1.0)
    yield from r.close(W_THIN, force=float(os.environ.get("WAND_F", "40")), label="take the wand")
    yield from r.move(r.at() + [0, 0, 0.04], t=1.5, label="lift")

    def where(label):
        w = wand()
        R = w.rotation()
        print(f"[demo] {label}: grip {np.round(w.pos, 3)} wand axis {np.round(R[:, 0], 3)} thin {np.round(R[:, 2], 3)} tcp {np.round(r.tcp(), 3)} q {np.round(np.degrees(ctx.observed('right')[:6]), 0)}", flush=True)

    def level(tag, rounds=2):
        """The grip lets the wand's weight tip it a few degrees: turn the hand about its own point until the wand lies flat and along the arm."""
        for k in range(rounds):
            dR = tilt()
            ang = math.degrees(math.acos(np.clip((np.trace(dR) - 1) / 2, -1, 1)))
            print(f"[demo] level {tag} {k}: wand off by {ang:.1f} deg", flush=True)
            if ang < 1.5 or ang > 35.0:
                break
            yield from r.move(r.at(), grip=_dk._as_grip(dR @ r.grip.rot), t=1.0, label=f"level {tag}")
            yield from r.wait(0.4)

    where("lifted")
    yield from level("at the holder")
    where("levelled")
    if ctx.mode == "pick":
        yield from r.wait(1.0)
        return
    up = r.at()
    aim[0] = W_top
    yield from r.move(up + [-BACK, 0, 0], t=2.0, label="draw back")
    c_goal = lambda s, z: np.array([AX, YS + DIR * s, z])  # noqa: E731
    top = coil_to_tcp(c_goal(S0, top_z(S0) + 0.10)) + np.array([-BACK, 0.0, 0.0])
    yield from r.move(np.array([up[0] - BACK, up[1], top[2]]), t=3.0, label="rise")
    yield from r.move(top, t=2.0, label="over the shoulder end")
    where("over the shoulder end")
    yield from level("over the arm")
    yield from r.move(coil_to_tcp(c_goal(S0, top_z(S0) + 0.03)), t=1.5, label="in")
    for _ in range(2):
        yield from r.servo(c_goal(S0, top_z(S0) + 0.03), tol=0.004, timeout=2.0, read=coil_c, label="over the shoulder")
    yield from r.servo(c_goal(S0, top_z(S0)), tol=0.004, timeout=2.0, read=coil_c, label="stand-off")
    # top pass: steady, slow and narrow at the watch
    sw0, sw1 = WATCH_S - 0.10, WATCH_S + 0.06

    def s_of_top(t):
        d1 = (sw0 - S0) / V
        d2 = (sw1 - sw0) / VS
        if t < d1:
            return S0 + V * t
        if t < d1 + d2:
            return sw0 + VS * (t - d1)
        return min(S1, sw1 + V * (t - d1 - d2))

    T1 = (sw0 - S0) / V + (sw1 - sw0) / VS + (S1 - sw1) / V
    if ctx.mode == "hover":  # held still at the watch for as long as the pass takes: a detector that is not moving does not alarm
        yield from r.servo(c_goal(WATCH_S, top_z(WATCH_S)), tol=0.004, timeout=2.0, read=coil_c, label="hover")
        yield from r.wait(T1)
        return
    yield from track(lambda t: c_goal(s_of_top(t), top_z(s_of_top(t))), T1, "top pass")
    where("end of the top pass")
    if ctx.mode == "top":
        return
    # roll under the arm at SR, round the chosen side, the wand kept parallel to it
    SR = float(os.environ.get("SR", "0.5"))
    if SR < S1:  # slide along the top to the roll position first
        yield from track(lambda t: c_goal(max(SR, S1 - V * t), top_z(max(SR, S1 - V * t))), (S1 - SR) / V, "back along the top", settle=0.5)
    T2 = float(os.environ.get("ROLL_T", "8.0"))
    r0 = top_z(SR) - ZA

    side_x = lambda s_: line_r(s_) + go + 0.01 + W_WIDE / 2  # noqa: E731  (side mode: the flat wand's near edge 5.5 cm from the arm's side)
    side_dz = HZ + 0.005  # and its top 5 mm below the centre line
    if ctx.mode == "side":
        phi_end, r_end = math.pi / 2 + math.atan2(side_dz, side_x(SR)), math.hypot(side_dz, side_x(SR))
    else:
        phi_end, r_end = math.pi, ZA - under_z(SR)

    def roll_to(t):
        phi = phi_end * _ik.smoothstep(min(1.0, t / T2))
        rad = r0 + (RHO - r0) * math.sin(phi) if phi < math.pi / 2 else RHO + (r_end - RHO) * (1.0 - math.sin(phi)) / (1.0 - math.sin(phi_end))
        return np.array([AX + SIDE * rad * math.sin(phi), YS + DIR * SR, ZA + rad * math.cos(phi)])

    aim[0] = W_des if ctx.mode == "side" else W_under
    yield from track(roll_to, T2, "roll under")
    where("under the arm")
    yield from level("under the arm")
    # underside pass: out to the fingertip end, then back to the shoulder
    d1 = (S1 - SR) / V

    def s_of_under(t):
        return min(S1, SR + V * t) if t < d1 else max(S0U, S1 - V * (t - d1))

    T3 = d1 + (S1 - S0U) / V
    if ctx.mode == "side":
        side_c = lambda s_: np.array([AX + SIDE * side_x(s_), YS + DIR * s_, ZA - side_dz])  # noqa: E731
        yield from track(lambda t: side_c(s_of_under(t)), T3, "side pass")
        yield from track(lambda t: side_c(S0U) - np.array([0.0, 0.0, 0.08 * min(1.0, t / 1.5)]), 1.5, "down from the side")
    else:
        yield from track(lambda t: c_goal(s_of_under(t), under_z(s_of_under(t))), T3, "underside pass")
    where("end of the underside pass")
    # back into the holder: it stands below the shoulder end, so the wand goes straight down onto it, flat and along y
    aim[0] = W_des
    yield from level("over the holder")
    home = np.array(WAND_AT)
    yield from r.servo(home + [0, 0, 0.03], tol=0.004, timeout=4.0, read=grip_pt, label="over the holder")
    yield from level("over the holder 2")
    if ctx.mode != "drop":  # drop: let go 3 cm above the holder
        yield from r.servo(home + [0, 0, 0.004], tol=0.003, timeout=3.0, read=grip_pt, label="lay")
    where("laid")
    yield from r.open(label="let go")
    yield from r.wait(0.5)
    yield from r.move(r.at() - 0.10 * a_h + [0, 0, 0.02], t=1.5, label="clear")
    yield from r.wait(2.5)
