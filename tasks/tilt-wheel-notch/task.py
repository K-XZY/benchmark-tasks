"""Tilt steering column with a friction-clamp lock (stepless, as the Honda Insight gen3 column): pull the lock lever down,
raise the wheel 6 degrees, push the lever up so the clamp locks the column there.

Range runs: TW_NET_NM sets the unlocked column's net torque at the target (negative droops, positive springs up; default -1.2,
ends -3.0 and +5.9); TW_LEVER_N sets the force at the lever handle that moves the lever either way (default 15, range end 82; the README lists the range runs).
TW_PADDLE_MM, TW_LEVER_OPEN, TW_LEVER_SHUT, TW_DAMP and TW_FRIC set the lever handle size, the lever angles that free and clamp
the column, and the column hinge's damping and friction (defaults and range ends at their definitions).

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
INSTRUCTION = "Pull the lock lever under the steering column down, raise the wheel 6 degrees, push the lever back up to lock it, and let go of the wheel and lever."
SUCCESS_HOLD_S = 0.0
SCORE_OVER = "end"
TOP = d.TABLE_TOP
DT = 0.05
HOLD_STEPS = 20
PUSH_STEPS = 10
PUSH_N = 5.0
PUSH_MOVE_DEG = 1.0
TARGET = 6.0     # raise from the start angle, degrees (read on a digital angle gauge in the real trial)
TARGET_TOL = 1.0
LEVER_OPEN = float(os.environ.get("TW_LEVER_OPEN", "-10"))  # lever angle past which the clamp frees the column (range-run -10 and -20)
LEVER_SHUT = float(os.environ.get("TW_LEVER_SHUT", "-3"))  # lever angle above which the clamp closes (range-run -3 and -1)
SLAM_STEPS = 10  # void window: 0.5 s before the clamp closes
SLAM_DEG = 1.5   # void when the column moved more than this in the window (an average over 3 deg/s): a dropped or swung wheel
CLEAR_M = 0.03
COL_KG = 3.0
G_NM = COL_KG * 9.81 * 0.23 * math.cos(math.radians(23.0 + TARGET))  # gravity torque about the tilt pivot at the target, N m (5.9)
NET_NM = float(os.environ.get("TW_NET_NM", "-1.2"))  # unlocked: net torque at the target, + lifts the wheel (spring-up), - droops
SPRING = 3.4  # tilt spring, N m/rad
SPRING_TARGET = math.radians(TARGET) + (G_NM + NET_NM) / SPRING  # rad: the spring's zero, set so that spring minus gravity = NET_NM at the target
LEVER_ARM_M = 0.068  # lever pivot to the handle's grip point
LEVER_N = float(os.environ.get("TW_LEVER_N", "15"))  # force at the lever handle to move the lever either way (Honda: 82 N max at 10 mm from the end)
LEVER_FRICTION = LEVER_N * LEVER_ARM_M  # N m at the lever pivot; the lever stays where it is left
PADDLE_M = float(os.environ.get("TW_PADDLE_MM", "25")) / 1000.0  # lever handle face, square (Honda figure: about 2x the 10 mm push mark; range-run 20; 45 fails)
COL_DAMP = float(os.environ.get("TW_DAMP", "4"))   # column hinge damping, N m s/rad (range-run 1 and 4)
COL_FRIC = float(os.environ.get("TW_FRIC", "0.3"))  # column hinge friction, N m (range-run 0 and 0.3)

LOOKS = {
    "dash": m.Look(m.rgb(0.20, 0.21, 0.23), metallic=0.0, roughness=0.8),
    "shroud": m.Look(m.rgb(0.12, 0.12, 0.13), metallic=0.0, roughness=0.6),
    "rim": m.Look(m.rgb(0.07, 0.07, 0.08), metallic=0.0, roughness=0.5),
    "hub": m.Look(m.rgb(0.25, 0.25, 0.27), metallic=0.3, roughness=0.4),
    "lever": m.Look(m.rgb(0.45, 0.45, 0.47), metallic=0.2, roughness=0.5),
    "tape": m.Look(m.rgb(0.92, 0.78, 0.15), metallic=0.0, roughness=0.7),
}

A = math.radians(23.0)
U = np.array([-math.cos(A), 0.0, math.sin(A)])  # along the column, toward the driver
V = np.array([math.sin(A), 0.0, math.cos(A)])   # column "up"
P = np.array([0.589, 0.0, 0.218])               # column tilt pivot (mechanism frame)
MAP = np.stack([U, np.array([0.0, -1.0, 0.0]), V], axis=1)  # local (s, w, t) -> mechanism frame, det +1
RIM_R = 0.185
RIM_S = 0.26
C_LOCAL = np.array([0.04, 0.0, -0.025])          # lever pivot (local)


def _loc(p) -> np.ndarray:
    return P + MAP @ np.asarray(p, float)


def _tf(mesh: m.Mesh) -> m.Mesh:
    return m.Mesh(mesh.v @ MAP.T + P, mesh.f)


def _bx(x0, x1, y0, y1, z0, z1) -> m.Mesh:
    return m.box((x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def _rot_y(p, deg, about) -> np.ndarray:
    p = np.atleast_2d(np.asarray(p, float)) - about
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    out = p.copy()
    out[:, 0] = p[:, 0] * c + p[:, 2] * s
    out[:, 2] = -p[:, 0] * s + p[:, 2] * c
    return out + about


def _ring_pts(n=48):
    return [(RIM_S, RIM_R * math.cos(2 * math.pi * k / n), RIM_R * math.sin(2 * math.pi * k / n)) for k in range(n + 1)]


def _rim() -> m.Mesh:
    return _tf(m.tube(_ring_pts(), 0.0125, sides=12))


def _rim_cols():
    pts = _ring_pts(48)
    return [m.col_hull(f"rim_c{k}", _tf(m.tube(pts[4 * k:4 * k + 5], 0.013, sides=8))) for k in range(12)]


def _hub() -> m.Mesh:
    spokes = [m.tube([(RIM_S, 0.0, 0.0), (RIM_S, RIM_R * math.cos(a), RIM_R * math.sin(a))], 0.011, sides=8)
              for a in (0.0, math.pi, 1.5 * math.pi)]
    return _tf(m.join(m.cylinder(0.045, 0.06, (0.23, 0.0, 0.0), axis="x"), *spokes))


def _shroud() -> m.Mesh:
    return _tf(m.cylinder(0.035, 0.17, (0.115, 0.0, 0.0), axis="x"))


def _lever_mesh() -> m.Mesh:
    # arm along the column's underside, then a handle hanging beside it, its faces square to the station x axis
    arm = _tf(_bx(0.03, 0.11, -0.06, -0.045, -0.032, -0.020))
    c = _loc((0.105, -0.0775, -0.045))
    return m.join(arm, m.box((0.010, PADDLE_M, PADDLE_M), tuple(c)))


def _dash() -> m.Mesh:
    return _bx(0.56, 0.70, -0.35, 0.35, 0.0, 0.36)


def _lower_shroud() -> m.Mesh:
    return _tf(m.cylinder(0.05, 0.03, (0.015, 0.0, 0.0), axis="x"))


def _tape() -> m.Mesh:
    # yellow mark on the dash face at the rim-top height for the target angle
    top = _rot_y(_loc((RIM_S, 0.0, RIM_R)), TARGET, P)[0]
    return _bx(0.556, 0.560, 0.03, 0.12, top[2] - 0.003, top[2] + 0.003)


def _steering() -> m.Part:
    com = _loc((0.23, 0.0, 0.0))
    column = m.hinge("column", "Y", tuple(P), 0.0, 13.0,
                     [("shroud", _shroud(), "shroud"), ("hub", _hub(), "hub"), ("rim", _rim(), "rim")],
                     [m.col_hull("shroud_c", _shroud()), m.col_hull("hub_c", _hub())] + _rim_cols(),
                     mass=COL_KG, gravity=True, com=tuple(com), inertia=(0.05, 0.05, 0.05),
                     damping=COL_DAMP, friction=COL_FRIC, effort=200.0, spring=SPRING)
    lever = m.hinge("lever", "Y", tuple(_loc(C_LOCAL)), -25.0, 0.0,
                    [("lever", _lever_mesh(), "lever")], [m.col_hull("lever_c", _lever_mesh())],
                    mass=0.05, damping=0.05, friction=LEVER_FRICTION, effort=20.0, parent="column")
    frame_parts = [("dash", _dash(), "dash"), ("lower", _lower_shroud(), "shroud"), ("tape", _tape(), "tape")]
    return m.mechanism("steering", (0.0, 0.0, TOP), [column, lever], LOOKS, frame_parts=frame_parts,
                       frame_colliders=[m.col_hull("dash_c", _dash())], friction=(0.8, 0.7), solver_iters=64,
                       free_mass=None)


OBJECTS = [_steering()]


def _surface(mesh: m.Mesh, step: float = 0.01) -> np.ndarray:
    """Points over a mesh's whole surface, no two neighbours more than `step` apart."""
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


WHEEL_PTS = np.vstack([_surface(_rim()), _surface(_hub())])
COL_PTS = np.vstack([WHEEL_PTS, _surface(_shroud())])
LEV_PTS = _surface(_lever_mesh(), 0.005)
RIM_TOP = _loc((RIM_S, 0.0, RIM_R))
TAB = _loc((0.105, -0.085, -0.045))  # lever handle grip, on the driver's left of the column (Kia, Mazda)


def _col_world(c, col, lev, pts_col=COL_PTS, pts_lev=LEV_PTS):
    lev_r = _rot_y(_rot_y(pts_lev, lev, _loc(C_LOCAL)), col, P)
    return c.to_world(np.vstack([_rot_y(pts_col, col, P), lev_r]))


class Trial:
    def reset(self, env=None) -> None:
        self.n = 0
        self.same_n = 0
        self.score = 0.0
        self.locked = False
        self.slam = False
        self.th_hist = []
        self.tier_last = 0.0
        self.check_n = 0
        self.check_th = None
        self.check_failed = False
        self.pushing = False
        self.checked = False
        self.last = {}
        if env is not None:
            self.step(env)

    def _push(self, env, c, th, on) -> None:
        if on:
            tan = _rot_y(RIM_TOP, th - 1.0, P)[0] - _rot_y(RIM_TOP, th, P)[0]
            tan /= np.linalg.norm(tan)
            at = c.to_world(_rot_y(RIM_TOP, th, P))[0]
            m.push(env, "steering", tuple(PUSH_N * (c.rotation() @ tan)), at=tuple(at), body="column")
        elif self.pushing:
            m.push(env, "steering", (0.0, 0.0, 0.0), body="column")
        self.pushing = on

    def step(self, env) -> None:
        self.n += 1
        c = m.state(env, "steering")
        th = c.deg("column")
        lv = c.deg("lever")
        self.th_hist = (self.th_hist + [th])[-(SLAM_STEPS + 1):]
        moved = abs(th - self.th_hist[0])
        if self.locked and lv <= LEVER_OPEN:
            m.unlock(env, "steering", "column")
            m.set_target(env, "steering", "column", SPRING_TARGET)
            self.locked = False
        elif not self.locked and lv >= LEVER_SHUT:
            if moved > SLAM_DEG and self.check_n == 0 and not self.check_failed:
                # a dropped or swung wheel: the clamp closes while the column still moves fast
                if os.environ.get("TRIAL_TRACE"):
                    print("lock", dict(n=self.n, col=round(th, 2), moved=round(moved, 2)), flush=True)
                self.slam = True
            m.lock(env, "steering", "column")
            self.locked = True
        at_target = self.locked and abs(th - TARGET) <= TARGET_TOL and lv >= LEVER_SHUT
        gap = 0.0  # the hands-clear distance matters only at the target; skipping it elsewhere keeps the sim fast
        if at_target:
            hand = np.vstack([m.hand_body(env), m.fingertips(env)])
            pts = _col_world(c, th, lv)
            lo, hi = pts.min(0) - CLEAR_M, pts.max(0) + CLEAR_M
            cand = hand[np.all((hand >= lo) & (hand <= hi), axis=1)]
            gap = 1.0 if len(cand) == 0 else float(np.linalg.norm(cand[:, None, :] - pts[None], axis=2).min())
        # 0.5: locked at the target with both hands clear; 1.0: and the lock holds the test push
        tier = 0.5 if (at_target and gap >= CLEAR_M) else 0.0
        if tier != self.tier_last:
            self.same_n = 0
            self.tier_last = tier
        else:
            self.same_n += 1
        if tier < 0.5:
            if self.check_n:
                self._push(env, c, th, False)
            self.check_n = 0
            self.checked = False
        elif self.same_n >= HOLD_STEPS and not self.checked and not self.check_failed:
            if self.check_n == 0:
                self.check_th = th
            self.check_n += 1
            self._push(env, c, th, self.check_n <= PUSH_STEPS)
            if abs(th - self.check_th) >= PUSH_MOVE_DEG:
                self.check_failed = True
                self._push(env, c, th, False)
            elif self.check_n > PUSH_STEPS:
                self.checked = True
        if self.slam:
            self.score = 0.0
        elif self.checked:
            self.score = 1.0
        else:
            self.score = 0.5 if (tier >= 0.5 and self.same_n >= HOLD_STEPS) else 0.0
        self.last = dict(n=self.n, col=round(th, 2), lever=round(lv, 2), locked=self.locked, gap=round(gap, 3),
                         moved=round(moved, 2), score=round(self.score, 3), slam=self.slam, failed=self.check_failed)
        if os.environ.get("TRIAL_TRACE") and self.n % 10 == 0:
            print("trial", self.last, flush=True)


NO_END = False
TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.score >= 1.0 and not NO_END


def score(objects) -> float:
    return TRIAL.score


# ---------------- demo ----------------
GRIP_L = d.tilted(90.0, 0.0)   # rim hand (the right arm) points +x into the rim, jaws across the rim tube
GRIP_R0 = d.tilted(90.0, -90.0)  # lever hand (the left arm) points -y along the lever handle
RIM_AT = math.radians(15.0)   # rim grip, 15 deg above the right-hand spoke


def _w(p) -> np.ndarray:
    return np.asarray(p, float) + np.array([0.0, 0.0, TOP])


def _rim_grip(th) -> np.ndarray:
    p = _loc((RIM_S, RIM_R * math.cos(RIM_AT), RIM_R * math.sin(RIM_AT)))
    return _w(_rot_y(p, th, P)[0])


def _tab_grip(th, lv) -> np.ndarray:
    return _w(_rot_y(_rot_y(TAB, lv, _loc(C_LOCAL)), th, P)[0])


def _roll(g, deg):
    """The grip `g` turned `deg` about the station y axis, as the column and lever turn."""
    c, sn = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return d._as_grip(np.array([[c, 0.0, sn], [0.0, 1.0, 0.0], [-sn, 0.0, c]]) @ g.rot)


GRIP_R = _roll(GRIP_R0, 90.0)  # jaws across the paddle's top and bottom faces, so the lever is pushed, not dragged by friction
PAD_W = PADDLE_M


def _phase(name) -> None:
    print(f"phase {name}", TRIAL.last, flush=True)


def _onto(A, target, grip, back, width, force=40.0, label=""):
    back = np.asarray(back, float)
    yield from A.move(target + back, grip=grip, finger=d.FINGER_OPEN, t=2.5, label="near " + label)
    yield from A.move(target, grip=grip, finger=d.FINGER_OPEN, t=1.2, label="onto " + label)
    yield from A.servo(target, timeout=1.0, label="servo " + label)
    yield from A.close(width, force, label="close " + label)


def _off(A, back, label=""):
    yield from A.open(label="open " + label)
    yield from A.move(A.at() + np.asarray(back, float), t=1.0, label="back " + label)


def _lever_to(ctx, R, goal, over=8.0):
    """Swing the paddle about the lever pivot until the lever reads `goal`, commanding up to `over` degrees past it (the arm lags under load)."""
    th = ctx.deg("steering", "column")
    a = ctx.deg("steering", "lever")
    sgn = -1.0 if goal < a else 1.0
    while abs(a - goal) < abs(goal) + over + 30.0 and sgn * (a - (goal + sgn * over)) < 0:
        if sgn * (ctx.deg("steering", "lever") - goal) >= 0:
            break
        a += sgn
        yield from R.move(_tab_grip(th, a), grip=_roll(GRIP_R, th + a), t=0.12, label="lever")
    yield from R.wait(0.5)


def _lift(ctx, L, goal, over=4.0, step=0.25):
    """Swing the rim grip about the column pivot until the column reads `goal`, commanding up to `over` degrees past it."""
    a = ctx.deg("steering", "column")
    while a < goal + over and ctx.deg("steering", "column") < goal - 0.1:
        a += step
        yield from L.move(_rim_grip(a), grip=_roll(GRIP_L, a), t=0.08, label="lift")
    yield from L.wait(0.5)
    return a


def _hold(ctx, L, goal, a, limit=120):
    """Keep the column at `goal` with the rim hand while the other hand shuts the lever (the lever's push lifts the column)."""
    for _ in range(limit):
        if ctx.deg("steering", "lever") >= LEVER_SHUT + 0.5:
            return
        a += float(np.clip(0.5 * (goal - ctx.deg("steering", "column")), -0.3, 0.3))
        yield from L.move(_rim_grip(a), grip=_roll(GRIP_L, a), t=0.05, label="hold")


MODES = ("full", "none", "liftonly", "leveronly", "overshoot", "leveropen", "restwheel", "drop")
def _fall_to(ctx, R, deg, limit=80):
    """Wait, holding the lever hand still, until the released column has fallen below `deg`."""
    for _ in range(limit):
        if ctx.deg("steering", "column") < deg:
            return
        yield from R.wait(0.05)


def _then(first, second):
    yield from first
    yield from second


def demo_script(ctx):
    mode = ctx.mode or "full"
    if mode not in MODES:
        raise ValueError(f"mode {mode} not in {MODES}")
    R, L = ctx.left, ctx.right  # R works the lever (left arm), L holds the rim (right arm)
    if mode == "none":
        yield from L.wait(3.0)
        return
    goal = 10.0 if mode == "overshoot" else TARGET
    LB, RB = (-0.08, 0.0, 0.0), (0.0, 0.08, 0.0)
    if mode == "drop":
        yield from _onto(L, _rim_grip(0.0), GRIP_L, LB, 0.025, label="rim")
        yield from _onto(R, _tab_grip(0.0, 0.0), GRIP_R, RB, PAD_W, label="paddle")
        yield from _lever_to(ctx, R, -24.0)
        # lazy: raise the wheel, let it go, and shut the lever while it falls through the target with no hand on it
        yield from _off(R, RB, "paddle")
        yield from _lift(ctx, L, 12.0)
        th, lv = ctx.deg("steering", "column"), ctx.deg("steering", "lever")
        yield from _onto(R, _tab_grip(th, lv), _roll(GRIP_R, th + lv), RB, PAD_W, label="paddle again")
        yield from _lever_to(ctx, R, -6.5, over=0.0)
        yield from ctx.together(_off(L, LB, "rim"), _then(_fall_to(ctx, R, TARGET + 0.8), _lever_to(ctx, R, -0.5)))
        yield from _off(R, RB, "paddle")
        _phase("end")
        yield from R.wait(2.0)
        return
    if mode != "leveronly":
        yield from _onto(L, _rim_grip(0.0), GRIP_L, LB, 0.025, label="rim")
        _phase("rim held")
    if mode != "liftonly":
        yield from _onto(R, _tab_grip(0.0, 0.0), GRIP_R, RB, PAD_W, label="paddle")
        yield from _lever_to(ctx, R, -24.0)
        yield from _off(R, RB, "paddle")
        _phase("lever down")
    if mode != "leveronly":
        a_rim = yield from _lift(ctx, L, goal)
        _phase("lifted")
    if mode == "leveropen":
        yield from _off(L, LB, "rim")
        yield from L.wait(3.0)
        _phase("end")
        return
    if mode != "liftonly":
        th, lv = ctx.deg("steering", "column"), ctx.deg("steering", "lever")
        yield from _onto(R, _tab_grip(th, lv), _roll(GRIP_R, th + lv), RB, PAD_W, label="paddle again")
        if mode == "leveronly":
            yield from _lever_to(ctx, R, -0.5)
        else:
            yield from ctx.together(_lever_to(ctx, R, -0.5), _hold(ctx, L, goal, a_rim))
        _phase("lever up")
        yield from _off(R, RB, "paddle")
    if mode not in ("restwheel", "leveronly"):
        yield from _off(L, LB, "rim")
    yield from R.wait(3.0)
    _phase("end")
