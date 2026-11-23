"""Hedge trimmer, rear handle to 90 degrees: eject the battery and set it on the table, then, with the front handle held,
pull the rotation lever under the rear grip, start the handle turning, let the lever go and turn on until the detent locks at 90 degrees.

Debug: python demokit.py task.py OUT --no-record --mode full,none
Final: python demokit.py task.py OUT --mode full
"""
from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import demokit as d  # noqa: E402
import mechkit as m  # noqa: E402

SCENE = "station"
INSTRUCTION = ("Take the battery out of the hedge trimmer and set it down, then turn the rear handle until it locks at 90 degrees, and let go.")
SUCCESS_HOLD_S = 0.0
SCORE_OVER = "end"
TOP = d.TABLE_TOP
DT = 0.05

# real sizes (card's size table; manual figures where marked)
BODY_KG = 3.4          # trimmer without battery (manual p.2); split frame + handle
HANDLE_KG = 0.35
BATT_KG = 1.24         # LB60A00, 2.73 lb (battery manual)
BATT = (0.17, 0.083, 0.118)  # 60V 2.0 Ah pack, about 6.8 x 3.3 x 4.7 in (retail listing)
BLADE = 0.66           # 26 in blade (manual p.2)
GRIP_R = 0.0175        # 35 mm rear grip
LEVER_TRAVEL = 0.008   # rotation lever: 15 mm across, 25 mm long, 8 mm travel, about 10 N (estimates)
LEVER_SPRING = 1250.0
TURN_FRICTION = 0.5    # N m to turn the rear handle with the lever pulled (estimate)
BUTTON_SPRING = 1000.0  # release button: 6 N at its 6 mm stop (estimate)
# Range check for the estimated constants, to the ends of their bounds (README, Fidelity): TRIM_RANGE=lo gives lever 5 N, button 3 N,
# turn friction 0.3 N m; TRIM_RANGE=hi gives lever 11 N, button 11 N, 0.6 N m
_K = {"lo": (0.5, 0.6, 0.5), "hi": (1.1, 1.2, 1.83)}.get(os.environ.get("TRIM_RANGE", ""), (1.0, 1.0, 1.0))
LEVER_SPRING, TURN_FRICTION, BUTTON_SPRING = LEVER_SPRING * _K[0], TURN_FRICTION * _K[1], BUTTON_SPRING * _K[2]

O = np.array([0.22, 0.26, TOP + 0.05])  # rear-handle axis at the housing's rear face; trimmer lies along -y (yaw -90)
YAW = -90.0
MAP = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])  # local -> world rotation for yaw -90

PULLED = 0.006         # lever pulled: unlocks the handle
AT_REST = 0.002        # lever let go
DETENTS = (-90.0, 0.0, 90.0)
DETENT_TOL = 2.0
OFF_CENTRE = 15.0
TURNED_LIVE = 3.0      # handle turned this far with the battery still in the bay: void
SLID_M = 0.10
LIFT_M = 0.015
RAIL_UP_M = 0.010      # the pack rises this far off its seat while still over the rails: lifted out, not drawn along them (rails are 8 mm tall)
RAIL_SPAN_M = 0.12     # pack centre within this of the port's middle across the body: still over the rails
TILT_DEG = 6.0
CLEAR_M = 0.02
CLEAR_STEPS = 60       # 3 s at 20 Hz
SET_DOWN_M = 0.03      # pack's lowest corner above the table when the fingers let it go: over this it was dropped
FALL_V = 0.77          # m/s downward, out of the bay: free fall from over 30 mm (v = sqrt(2 g h))
UPRIGHT_DEG = 20.0     # set down on its base: a pack that tipped off the rails lies on its side or end
HELD_M = 0.012         # a fingertip point this near the pack's surface holds it
LATCH_N = 150.0        # the bay latch, as a hold-down on the pack until the release button is pressed
EJECT_N = float(os.environ.get("TRIM_EJECT", "4"))  # the spring eject: eject force is an estimate, no published value: the force to slide the 1.24 kg pack against a rail friction of 0.25 to 0.4 is about 3.0 to 4.9 N; TRIM_EJECT=4 and =14 give the ends of the accepted window
# the spring pushes the pack this far out along its rails, then lets go. Real stroke: no maker value; bound 12-20 mm (the 170 mm pack's seat
# is released and it pops proud of the port by a finger's width, 12 to 20 mm; bench: ruler on the pack's travel from latch release to rest).
# TRIM_STROKE=lo gives 12 mm, =hi gives 20 mm; the default is 15 mm.
EJECT_STROKE_M = {"lo": 0.012, "hi": 0.020}.get(os.environ.get("TRIM_STROKE", ""), 0.015)
# pack-on-rail friction pair coefficient (the pack keeps its own 0.5 material against fingers and table; only the rail pair's two materials set this pair): engineering estimate for plastic on plastic (ABS, PP, nylon) 0.2-0.5, no named source checked;
# a pair above about 0.33 would hold the pack against a 4 N spring, so the bound the scene covers is 0.25 (TRIM_RAILMU=lo) to 0.4 (hi, run with TRIM_EJECT=6, the 4 N spring is held by a pair over about 0.33, so 6 N is used there)
RAIL_MU = {"lo": 0.25, "hi": 0.40}.get(os.environ.get("TRIM_RAILMU", ""), 0.275)
PACK_MU = 0.5  # plastic pack on rubber pads and the table: about 0.5, on its own material
m.ALT_FRICTION = (2 * RAIL_MU - 0.5,) * 2  # the two rail colliders: PhysX averages the two materials of a contact, so pack on rail gives RAIL_MU
EJECT_MAX_STEPS = 40

LOOKS = {
    "body": m.Look(m.rgb(0.40, 0.62, 0.12), metallic=0.0, roughness=0.55),
    "black": m.Look(m.rgb(0.06, 0.06, 0.07), metallic=0.0, roughness=0.6),
    "grey": m.Look(m.rgb(0.25, 0.26, 0.27), metallic=0.1, roughness=0.5),
    "steel": m.Look(m.rgb(0.70, 0.71, 0.73), metallic=0.8, roughness=0.3),
    "cover": m.Look(m.rgb(0.12, 0.12, 0.13), metallic=0.0, roughness=0.7),
    "orange": m.Look(m.rgb(0.85, 0.35, 0.08), metallic=0.0, roughness=0.6),
    "batt": m.Look(m.rgb(0.10, 0.10, 0.11), metallic=0.0, roughness=0.5),
    "stripe": m.Look(m.rgb(0.40, 0.62, 0.12), metallic=0.0, roughness=0.5),
}


def _bx(x0, x1, y0, y1, z0, z1) -> m.Mesh:
    return m.box((x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def _segs(name, pts, r):
    return [m.col_hull(f"{name}{k}", m.tube([pts[k], pts[k + 1]], r, sides=8)) for k in range(len(pts) - 1)]


# ---- trimmer, local frame: x from the rear handle toward the blade tip, z up, origin on the handle axis at the housing's rear face
HOUSING = _bx(0.0, 0.36, -0.05, 0.05, -0.05, 0.045)
NOSE = _bx(0.36, 0.42, -0.035, 0.035, -0.045, 0.0)
BLADE_M = _bx(0.42, 0.42 + BLADE, -0.03, 0.03, -0.040, -0.034)
COVER = _bx(0.42, 0.42 + BLADE, -0.034, 0.034, -0.05, -0.040)
# battery port across the body (manual Fig. 1: the pack's end shows on the body's side, the release button under it): the pack slides
# in and out sideways along two 8 mm rails, 1 mm clear a side
BAY = _bx(0.09, 0.19, -0.085, 0.085, -0.05, 0.045)
RAIL_L = _bx(0.0835, 0.0975, -0.085, 0.085, 0.045, 0.053)
RAIL_R = _bx(0.1825, 0.1965, -0.085, 0.085, 0.045, 0.053)
HOOP_PTS = [(0.27, -0.075, -0.03), (0.27, -0.08, 0.06), (0.27, -0.05, 0.12), (0.27, 0.05, 0.12), (0.27, 0.08, 0.06), (0.27, 0.075, -0.03)]
HOOP = m.tube(HOOP_PTS, 0.0125, sides=12)
HOOP_TOP = np.array([0.27, 0.0, 0.12])

HUB = m.cylinder(0.045, 0.063, (-0.0335, 0.0, 0.0), axis="x")
LOOP_PTS = [(-0.06, 0.0, 0.03), (-0.07, 0.0, 0.10), (-0.18, 0.0, 0.10), (-0.20, 0.0, 0.05), (-0.19, 0.0, -0.03), (-0.04, 0.0, -0.03)]
LOOP = m.tube(LOOP_PTS, 0.014, sides=12)
GRIP_M = m.tube([(-0.065, 0.0, 0.10), (-0.175, 0.0, 0.10)], GRIP_R, sides=16)
TRIGGER = _bx(-0.155, -0.125, -0.006, 0.006, 0.065, 0.083)
GRIP_AT = np.array([-0.125, 0.0, 0.10])
LEVER_C = np.array([-0.025, 0.0, 0.0575])  # paddle centre: on the handle's base just behind the housing, pulled rearward (manual Fig. 4)
LEVER_M = m.box((0.010, 0.015, 0.025), tuple(LEVER_C))  # 10 mm thick along the trimmer, 15 mm across, 25 mm tall
BUTTON_AT = (0.14, -0.085, 0.0)  # on the port's face toward the robot, under the pack's end
BATT_AT = np.array([0.14, 0.0, 0.045 + BATT[2] / 2])


def _trimmer() -> m.Part:
    handle = m.hinge("handle", "X", (0.0, 0.0, 0.0), -90.0, 90.0,
                     [("hub", HUB, "black"), ("loop", LOOP, "black"), ("grip", GRIP_M, "grey"), ("trigger", TRIGGER, "orange")],
                     [m.col_hull("hub_c", HUB), m.col_hull("grip_c", GRIP_M), m.col_hull("trig_c", TRIGGER)] + _segs("loop_c", LOOP_PTS, 0.014),
                     mass=HANDLE_KG, damping=0.3, friction=TURN_FRICTION, effort=100.0, inertia=(0.002, 0.003, 0.003))
    lever = m.slider("lever", "X", -LEVER_TRAVEL, 0.0, [("lever", LEVER_M, "orange")], [m.col_hull("lever_c", LEVER_M)],
                     at=tuple(LEVER_C), mass=0.02, spring=LEVER_SPRING, damping=2.0, friction=0.0, effort=200.0, parent="handle")
    button = m.button("button", BUTTON_AT, "-Y", look="orange", radius=0.009, height=0.01, travel=0.006, spring=BUTTON_SPRING)
    frame_parts = [("housing", HOUSING, "body"), ("bay", BAY, "body"), ("nose", NOSE, "black"), ("blade", BLADE_M, "steel"), ("cover", COVER, "cover"),
                   ("rail_l", RAIL_L, "black"), ("rail_r", RAIL_R, "black"), ("hoop", HOOP, "black")]
    cols = [m.col_hull("housing_c", _bx(0.0, 0.36, -0.05, 0.05, -0.05, 0.040)), m.col_hull("top_a_c", _bx(0.0, 0.0835, -0.05, 0.05, 0.040, 0.045)),
            m.col_hull("seat_c", _bx(0.0835, 0.1965, -0.085, 0.085, 0.040, 0.045), alt=True), m.col_hull("top_b_c", _bx(0.1965, 0.36, -0.05, 0.05, 0.040, 0.045)), m.col_hull("bay_c", _bx(0.09, 0.19, -0.085, 0.085, -0.05, 0.040)), m.col_hull("nose_c", NOSE), m.col_hull("cover_c", m.join(COVER, BLADE_M)),
            m.col_hull("rail_l_c", RAIL_L, alt=True), m.col_hull("rail_r_c", RAIL_R, alt=True)] + _segs("hoop_c", HOOP_PTS, 0.0125)
    return m.mechanism("trimmer", tuple(O + np.array([0.0, 0.0, 0.001])), [handle, lever, button], LOOKS, frame_parts=frame_parts,
                       frame_colliders=cols, yaw_deg=YAW, friction=(0.5, 0.45), solver_iters=64, free_mass=BODY_KG - HANDLE_KG,
                       frame_inertia=(0.01, 0.16, 0.16))


def _battery() -> m.Part:
    L, W, H = BATT
    body = _bx(-L / 2, L / 2, -W / 2, W / 2, -H / 2, H / 2)
    stripe = _bx(-L / 2 + 0.01, L / 2 - 0.01, W / 2, W / 2 + 0.001, H / 2 - 0.025, H / 2 - 0.012)
    stripe2 = _bx(-L / 2 + 0.01, L / 2 - 0.01, -W / 2 - 0.001, -W / 2, H / 2 - 0.025, H / 2 - 0.012)
    pos = O + MAP @ BATT_AT + np.array([0.0, 0.0, 0.002])
    return m.free("battery", tuple(pos), [("pack", body, "batt"), ("stripe", stripe, "stripe"), ("stripe2", stripe2, "stripe")],
                  [m.col_box("pack_c", (L, W, H))], LOOKS, mass=BATT_KG, friction=(PACK_MU, PACK_MU), yaw_deg=0.0)  # plastic pack on plastic rails: the pack-rail pair is RAIL_MU (0.275: 3.3 N holds it at rest, below the 4 N eject); long side across the trimmer, along the rails


OBJECTS = [_trimmer(), _battery()]


def _surface(mesh: m.Mesh, step: float = 0.01) -> np.ndarray:
    out = [mesh.v]
    for a, b, c in mesh.v[mesh.f]:
        n = int(max(np.linalg.norm(b - a), np.linalg.norm(c - a), np.linalg.norm(c - b)) / step) + 1
        if n < 2:
            continue
        i, j = np.meshgrid(np.arange(n + 1), np.arange(n + 1))
        keep = (i + j) <= n
        u, v = i[keep] / n, j[keep] / n
        out.append(a + u[:, None] * (b - a) + v[:, None] * (c - a))
    return np.vstack(out)


FRAME_PTS = np.vstack([_surface(x) for x in (HOUSING, BAY, NOSE, HOOP, RAIL_L, RAIL_R)] + [_surface(m.join(BLADE_M, COVER), 0.03)])
HANDLE_PTS = np.vstack([_surface(x) for x in (HUB, LOOP, GRIP_M, TRIGGER, LEVER_M)])
PACK_HALF = np.array(BATT) / 2
PACK_PTS = _surface(_bx(-PACK_HALF[0], PACK_HALF[0], -PACK_HALF[1], PACK_HALF[1], -PACK_HALF[2], PACK_HALF[2]))
PACK_CORNERS = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * PACK_HALF


def _rot_x(p, deg) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    r = np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
    return np.atleast_2d(np.asarray(p, float)) @ r.T


def _gap(c, b, th, hand) -> float:
    """Least distance from any gripper point (palm, housing, pads, fingers, wrist link) to the trimmer's or the pack's surface."""
    pts = np.vstack([c.to_world(np.vstack([FRAME_PTS, _rot_x(HANDLE_PTS, th)])), b.to_world(PACK_PTS)])
    lo, hi = pts.min(0) - CLEAR_M, pts.max(0) + CLEAR_M
    cand = hand[np.all((hand >= lo) & (hand <= hi), axis=1)]
    if len(cand) == 0:
        return 1.0
    near = pts[np.all((pts >= cand.min(0) - CLEAR_M) & (pts <= cand.max(0) + CLEAR_M), axis=1)]
    if len(near) == 0:
        return 1.0
    return float(np.linalg.norm(cand[:, None, :] - near[None], axis=2).min())


class Trial:
    def reset(self, env=None) -> None:
        self.n = 0
        self.score = 0.0
        self.void = ""
        self.locked = True
        self.latched = True
        self.eject_n = 0
        self.eject_y0 = 0.0
        self.clear_n = 0
        self.held = False
        self.start = None
        self.bz0 = None
        self.last = {}
        if env is not None:
            m.lock(env, "trimmer", "handle")
            m.push(env, "battery", (0.0, 0.0, -LATCH_N))
            self.step(env)

    def step(self, env) -> None:
        self.n += 1
        c = m.state(env, "trimmer")
        b = m.state(env, "battery")
        if self.start is None:
            self.start = np.asarray(c.pos, float).copy()
        th = c.deg("handle")
        lv = -c.joint("lever")  # pulled rearward reads +
        bt = c.joint("button")  # outward -Y: pressed in reads +
        # bay latch: the release button frees the pack and its spring pushes it a little way out along the rails
        if self.latched and bt >= 0.005:
            self.latched = False
            self.eject_n = EJECT_MAX_STEPS
            self.eject_y0 = float(c.to_local(np.asarray(m.state(env, "battery").pos, float))[0][1])
            m.push(env, "battery", tuple(-EJECT_N * c.rotation() @ np.array([0.0, 1.0, 0.0])))  # out along the rails, toward the button side
        elif self.eject_n:
            self.eject_n -= 1
            if abs(float(c.to_local(np.asarray(m.state(env, "battery").pos, float))[0][1]) - self.eject_y0) >= EJECT_STROKE_M:
                self.eject_n = 0
            if self.eject_n == 0:
                m.push(env, "battery", (0.0, 0.0, 0.0))
        # rotation lever and detent
        if self.locked and lv >= PULLED:
            m.unlock(env, "trimmer", "handle")
            self.locked = False
        elif not self.locked and lv <= AT_REST and min(abs(th - x) for x in DETENTS) <= DETENT_TOL:
            m.lock(env, "trimmer", "handle")
            m.set_target(env, "trimmer", "handle", math.radians(min(DETENTS, key=lambda x: abs(th - x))))  # the detent pulls it home
            self.locked = True
        bl = c.to_local(np.asarray(b.pos, float))[0]
        if self.n == 10:
            self.bz0 = float(b.to_world(PACK_CORNERS)[:, 2].min())  # the pack's lowest corner, seated on its rails, after the first steps' settling
        in_bay = abs(bl[0] - BATT_AT[0]) < 0.05 and abs(bl[1]) < 0.17 and 0.08 < bl[2] < 0.14  # on the rails, any part of it still in the port
        out = not in_bay and float(b.pos[2]) - TOP < 0.09 and np.linalg.norm(np.asarray(b.lin_vel, float)) < 0.05 and b.tilt_deg() < UPRIGHT_DEG
        # set down, not dropped: the pack's height when the fingers let it go, and its fall speed out of the bay
        tips = b.to_local(m.fingertips(env))
        held = bool((np.linalg.norm(np.maximum(np.abs(tips) - PACK_HALF, 0.0), axis=1) < HELD_M).any())
        low = float(b.to_world(PACK_CORNERS)[:, 2].min()) - TOP
        dropped = not in_bay and ((self.held and not held and low > SET_DOWN_M) or float(b.lin_vel[2]) < -FALL_V)
        self.held = held
        # void
        pos = np.asarray(c.pos, float)
        if not self.void:
            if dropped:
                self.void = "dropped"
            elif abs(bl[1]) < RAIL_SPAN_M and self.bz0 is not None and low + TOP - self.bz0 > RAIL_UP_M and abs(bl[0] - BATT_AT[0]) < 0.05 and not self.latched:
                self.void = "lifted out"
            elif in_bay and abs(th) > TURNED_LIVE:
                self.void = "turned live"
            elif np.linalg.norm(pos[:2] - self.start[:2]) > SLID_M:
                self.void = "slid"
            elif pos[2] - self.start[2] > LIFT_M or c.tilt_deg() > TILT_DEG:
                self.void = "lifted"
        locked90 = self.locked and abs(abs(th) - 90.0) <= DETENT_TOL and lv <= AT_REST
        gap = 0.0
        if out and locked90 and self.n % 2 == 0:
            gap = _gap(c, b, th, np.vstack([m.hand_body(env), m.fingertips(env)]))
            self.clear_n = self.clear_n + 2 if gap >= CLEAR_M else 0
        elif not (out and locked90):
            self.clear_n = 0
        tier = 0.0
        if out:
            tier = 0.25
            if abs(th) >= OFF_CENTRE:
                tier = 0.5
                if locked90:
                    tier = 0.75
                    if self.clear_n >= CLEAR_STEPS:
                        tier = 1.0
        self.score = 0.0 if self.void else tier
        self.last = dict(n=self.n, th=round(th, 1), lever=round(lv * 1000, 1), btn=round(bt * 1000, 1), latched=self.latched,
                         locked=self.locked, in_bay=in_bay, out=out, held=held, low=round(low, 3), bz=round(float(b.pos[2]) - TOP, 3), rise=round(low + TOP - (self.bz0 or low + TOP), 4), btilt=round(float(b.tilt_deg()), 1), by=round(float(bl[1]), 3), gap=round(gap, 3), clear=self.clear_n, void=self.void,
                         slid=round(float(np.linalg.norm(pos[:2] - self.start[:2])), 3), score=self.score)
        if os.environ.get("TRIAL_TRACE") and self.n % 5 == 0:
            print("trial", self.last, flush=True)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.score >= 1.0


def score(objects) -> float:
    return TRIAL.score


# ---------------- demo ----------------
def _phase(name) -> None:
    print(f"phase {name}", TRIAL.last, flush=True)


def _w(ctx, p_local, th=None) -> np.ndarray:
    """A trimmer point (local frame) in the world, read from the trimmer's pose now; `th` turns it with the rear handle."""
    c = ctx.state("trimmer")
    p = np.asarray(p_local, float) if th is None else _rot_x(p_local, th)[0]
    return c.to_world(p)[0]


def _axis(ctx) -> np.ndarray:
    return ctx.state("trimmer").rotation() @ np.array([1.0, 0.0, 0.0])


UP = np.array([0.0, 0.0, 1.0])
TIP_DROP = 0.008  # so on a grip tilted 60 degrees the ends sit this far below the TCP
MODES = ("full", "none", "nolever", "skipbattery", "nobutton", "holdpack", "droppack", "liftout", "ejectonly")


def demo_script(ctx):
    mode = ctx.mode or "full"
    if mode not in MODES:
        raise ValueError(f"mode {mode} not in {MODES}")
    L, R = ctx.left, ctx.right  # L: battery, lever and rear handle; R: front handle
    if mode == "none":
        yield from L.wait(3.0)
        return
    if mode == "ejectonly":  # press the release button and stop: the eject alone must not score
        btn = _w(ctx, BUTTON_AT) + np.array([0.0, 0.0, TIP_DROP])
        yield from L.move(btn - 0.07 * np.array([1.0, 0.0, 0.0]), grip=d.tilted(60.0, 0.0), finger=d.FINGER_CLOSED, t=2.5, label="near button")
        k = 0
        while ctx.joint("trimmer", "button") < 0.0055 and k < 60:
            k += 1
            yield from L.move(btn + (-0.035 + 0.0012 * k) * np.array([1.0, 0.0, 0.0]), t=0.05, label="press button")
        yield from L.wait(6.0)
        return
    X = np.array([1.0, 0.0, 0.0])
    # the right hand braces the trimmer by the front handle's top bar, fingers across the tube
    hoop = _w(ctx, HOOP_TOP)
    yield from R.move(hoop + 0.07 * UP - 0.05 * X, grip=d.tilted(45.0, 0.0), finger=d.FINGER_OPEN, t=2.5, label="over hoop")
    yield from R.move(hoop, t=1.2, label="onto hoop")
    yield from R.close(0.025, 40.0, label="hold hoop")
    _phase("front held")
    if mode != "skipbattery":
        if mode != "nobutton":
            btn = _w(ctx, BUTTON_AT) + np.array([0.0, 0.0, TIP_DROP])
            g = d.tilted(60.0, 0.0)
            yield from L.move(btn - 0.07 * X, grip=g, finger=d.FINGER_CLOSED, t=2.5, label="near button")
            yield from L.move(btn - 0.035 * X, t=0.8, label="to button")
            k = 0
            while ctx.joint("trimmer", "button") < 0.0055 and k < 60:  # in until the button bottoms
                k += 1
                yield from L.move(btn + (-0.035 + 0.0012 * k) * X, t=0.05, label="press button")
            yield from L.wait(0.3)
            yield from L.move(btn - 0.07 * X, t=0.6, label="off button")
            yield from L.move(btn - 0.07 * X + 0.20 * UP, t=1.2, label="up past the pack")
            _phase("button")
        # grip the pack's sides at its centre of mass, then draw it out along the rails toward the arms, level, and set it on the table
        g = d.tilted(40.0, 0.0)  # leaned toward the trimmer so the wrist camera rides behind the pack, not on its top
        appr = np.array([math.sin(math.radians(40.0)), 0.0, -math.cos(math.radians(40.0))])
        pk = ctx.pos("battery") + np.array([0.0, 0.0, BATT[2] / 2 - 0.025])
        yield from L.move(pk + 0.11 * UP, grip=g, finger=d.FINGER_OPEN, t=1.5, label="over pack")
        yield from L.wait(0.5)
        pk = ctx.pos("battery") + np.array([0.0, 0.0, BATT[2] / 2 - 0.025])  # read again: the eject moved it
        yield from L.move(pk + 0.11 * UP, t=0.6, label="over pack again")
        yield from L.move(pk, t=1.5, label="onto pack")
        yield from L.close(BATT[1], 40.0, label="grip pack")
        out = -_w(ctx, (0.0, 1.0, 0.0)) + _w(ctx, (0.0, 0.0, 0.0))  # the rails' outward direction (trimmer -y), world
        reach = 0.12 if mode == "droppack" else 0.23  # droppack lets go with the pack's centre past the rails' end: it tips off and falls
        if mode == "liftout":  # lift the pack straight up off its rails instead of drawing it along them
            yield from L.move(pk + 0.12 * UP, t=1.5, label="lift pack out")  # the arm sags about 60 mm under the pack
        yield from L.move(pk + reach * out + (0.12 if mode == "liftout" else 0.01) * UP, t=3.0, label="draw pack out")
        _phase("pack out of the port")
        if mode != "droppack":
            place = pk + 0.23 * out
            place[2] = TOP + BATT[2] / 2 + 0.004
            yield from L.move(place + 0.02 * UP, t=2.0, label="lower pack")
            yield from L.move(place, t=1.2, label="set pack")
            yield from L.move(place - 0.012 * UP, t=0.8, label="seat pack")  # press down a little: the pack settles flat on its base in the pinch
            yield from L.wait(0.4)
        yield from L.open(label="let pack go")
        yield from L.move(L.at() + 0.10 * UP, t=0.8, label="off pack")
        _phase("battery out")
    ax = _axis(ctx)
    pivot = _w(ctx, (0.0, 0.0, 0.0))
    # grip the rotation lever from above, one finger in front of it and one behind, and pull it rearward until it stops
    top = _w(ctx, LEVER_C) + 0.022 * UP  # fingertips on the paddle's upper half, clear of the loop's foot behind it
    yield from L.move(top + 0.06 * UP, grip=d.down(0.0), finger=-0.016, t=2.0, label="over lever")
    yield from L.move(top, t=1.0, label="onto lever")
    yield from L.close(0.010, 40.0, label="pinch lever")
    if mode != "nolever":
        yield from L.press(-ax, force=30.0, max_travel=0.03, settle=0.4, timeout=3.0, label="pull lever")  # rearward to its 8 mm stop, about 10 N of spring
        yield from L.wait(0.3)
    _phase("lever pulled")
    # turn the handle through the held lever: the pinched lever swings with the handle about its axis, to about 45 degrees
    yield from L.arc(pivot, ax, 30.0, t=1.5, label="start turn")
    _phase("turn started")
    yield from L.arc(pivot, ax, 15.0, t=1.0, label="turn on")
    yield from L.open(label="let lever go")  # off its detents now, the handle stays free
    yield from L.move(L.at() + 0.06 * UP, t=0.8, label="off lever")
    # push the free grip on round from behind with closed fingers until it stops and clicks at 90
    k = 0
    while ctx.deg("trimmer", "handle") < 88.0 and k < 8:
        k += 1
        th = ctx.deg("trimmer", "handle")
        rot = ctx.state("trimmer").rotation()
        c = _rot_x(GRIP_AT, th)[0]
        tw = rot @ np.array([0.0, -math.cos(math.radians(th)), -math.sin(math.radians(th))])  # the way the grip moves, world
        contact = _w(ctx, c) - (GRIP_R + 0.004) * tw + 0.016 * UP  # fingertips behind the grip, TCP 16 mm above them
        yield from L.move(contact - 0.03 * tw + 0.04 * UP, grip=d.down(-90.0), finger=d.FINGER_CLOSED, t=1.0, label="behind grip")
        yield from L.move(contact - 0.01 * tw, t=0.6, label="to grip")
        yield from L.press(tw, force=25.0, max_travel=0.05, settle=0.3, timeout=2.0, label="push grip")
    yield from L.wait(0.5)
    _phase("turned")
    yield from L.move(L.at() + 0.08 * UP, t=0.8, label="off grip")
    yield from R.open(label="let hoop go")
    yield from R.move(R.at() + 0.08 * UP - 0.06 * X, t=1.0, label="off hoop")
    if mode == "holdpack":  # the left hand goes back and keeps hold of the pack to the end
        pk = ctx.pos("battery") + np.array([0.0, 0.0, BATT[2] / 2 - 0.06])
        yield from L.move(pk + 0.05 * UP, grip=d.down(0.0), finger=d.FINGER_OPEN, t=2.0, label="over pack")
        yield from L.move(pk, t=1.0, label="onto pack")
        yield from L.close(BATT[1], 40.0, label="hold pack")
    else:
        yield from L.move(L.at() + np.array([-0.06, 0.0, 0.06]), t=1.0, label="clear")
    yield from L.wait(4.0)
    _phase("end")
