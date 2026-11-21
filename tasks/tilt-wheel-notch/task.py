"""Tilt steering column: pull the lock lever down, raise the wheel two notches, close the lever so the column locks.

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
INSTRUCTION = "Pull the tilt lever under the steering column down, raise the wheel two notches, then push the lever back up so the column locks there, and take your hands off the wheel and lever."
SUCCESS_HOLD_S = 0.0
SCORE_OVER = "end"
TOP = d.TABLE_TOP
DT = 0.05
HOLD_STEPS = 20
PUSH_STEPS = 10
PUSH_N = 5.0
PUSH_MOVE_DEG = 1.0
NOTCHES = (0.0, 3.0, 6.0, 9.0, 12.0)
TARGET = 6.0
LOCK_TOL = 0.3
LEVER_OPEN = -10.0
LEVER_SHUT = -3.0
RAISED = 1.5
SLAM_DPS = 3.0
CLEAR_M = 0.03
COL_KG = 3.0
SPRING = 3.4  # counterbalance spring, N m/rad toward 90 deg: leaves about 1.2 N m of droop at 6 deg (gravity alone 6.2)
SPRING_TARGET = 1.571
LEVER_FRICTION = 1.0  # N m at the lever pivot: about 15 N at the paddle to move it either way (estimate, no spring scale available); it stays where it is left

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
    # arm along the column's underside, then a paddle hanging beside it, its faces square to the station x axis
    arm = _tf(_bx(0.03, 0.11, -0.06, -0.045, -0.032, -0.020))
    c = _loc((0.105, -0.0775, -0.045))
    return m.join(arm, m.box((0.010, 0.045, 0.045), tuple(c)))


def _dash() -> m.Mesh:
    return _bx(0.56, 0.70, -0.35, 0.35, 0.0, 0.36)


def _lower_shroud() -> m.Mesh:
    return _tf(m.cylinder(0.05, 0.03, (0.015, 0.0, 0.0), axis="x"))


def _tape() -> m.Mesh:
    # yellow mark on the dash face at the rim-top height for the target notch
    top = _rot_y(_loc((RIM_S, 0.0, RIM_R)), TARGET, P)[0]
    return _bx(0.556, 0.560, 0.03, 0.12, top[2] - 0.003, top[2] + 0.003)


def _steering() -> m.Part:
    com = _loc((0.23, 0.0, 0.0))
    column = m.hinge("column", "Y", tuple(P), 0.0, 13.0,
                     [("shroud", _shroud(), "shroud"), ("hub", _hub(), "hub"), ("rim", _rim(), "rim")],
                     [m.col_hull("shroud_c", _shroud()), m.col_hull("hub_c", _hub())] + _rim_cols(),
                     mass=COL_KG, gravity=True, com=tuple(com), inertia=(0.05, 0.05, 0.05),
                     damping=4.0, friction=0.3, effort=200.0, spring=SPRING)
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
TAB = _loc((0.105, -0.085, -0.045))  # paddle grip, on the driver's left of the column (Kia, Mazda)


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
        self.th_last = None
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
        speed = 0.0 if self.th_last is None else abs(th - self.th_last) / DT
        self.th_last = th
        near = min(abs(th - n) for n in NOTCHES) <= LOCK_TOL
        if self.locked and lv <= LEVER_OPEN:
            m.unlock(env, "steering", "column")
            m.set_target(env, "steering", "column", SPRING_TARGET)
            self.locked = False
        elif not self.locked and lv >= LEVER_SHUT and near:
            if speed > SLAM_DPS and self.check_n == 0 and not self.check_failed:
                # a dropped wheel: the column locks while moving fast with no gripper within CLEAR_M of the wheel (rim or hub)
                hand = np.vstack([m.hand_body(env), m.fingertips(env)])
                pts = _col_world(c, th, lv, pts_col=WHEEL_PTS, pts_lev=LEV_PTS[:0])
                lo, hi = pts.min(0) - CLEAR_M, pts.max(0) + CLEAR_M
                cand = hand[np.all((hand >= lo) & (hand <= hi), axis=1)]
                held = len(cand) > 0 and float(np.linalg.norm(cand[:, None, :] - pts[None], axis=2).min()) < CLEAR_M
                if os.environ.get("TRIAL_TRACE"):
                    print("lock", dict(n=self.n, col=round(th, 2), speed=round(speed, 1), held=held), flush=True)
                if not held:
                    self.slam = True
            m.lock(env, "steering", "column")
            self.locked = True
        at_target = self.locked and abs(th - TARGET) <= LOCK_TOL and lv >= LEVER_SHUT
        gap = 0.0  # the hands-clear distance matters only at the target; skipping it elsewhere keeps the sim fast
        if at_target:
            hand = np.vstack([m.hand_body(env), m.fingertips(env)])
            pts = _col_world(c, th, lv)
            lo, hi = pts.min(0) - CLEAR_M, pts.max(0) + CLEAR_M
            cand = hand[np.all((hand >= lo) & (hand <= hi), axis=1)]
            gap = 1.0 if len(cand) == 0 else float(np.linalg.norm(cand[:, None, :] - pts[None], axis=2).min())
        tier = 0.0
        if th > RAISED:
            tier = 1 / 3
        if at_target:
            tier = 2 / 3
        if tier != self.tier_last:
            self.same_n = 0
            self.tier_last = tier
        else:
            self.same_n += 1
        if tier < 2 / 3 or gap < CLEAR_M:
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
            self.score = 2 / 3 if (tier >= 2 / 3 and self.same_n >= HOLD_STEPS) else (1 / 3 if tier > 0 else 0.0)
        self.last = dict(n=self.n, col=round(th, 2), lever=round(lv, 2), locked=self.locked, gap=round(gap, 3),
                         speed=round(speed, 1), score=round(self.score, 3), slam=self.slam, failed=self.check_failed)
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
GRIP_R = d.tilted(90.0, -90.0)  # lever hand (the left arm) points -y along the paddle, jaws across its faces
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


def _phase(name) -> None:
    print(f"phase {name}", TRIAL.last, flush=True)


def _onto(A, target, grip, back, width, force=40.0, label="", pinch=True):
    back = np.asarray(back, float)
    yield from A.move(target + back, grip=grip, finger=d.FINGER_OPEN, t=2.5, label="near " + label)
    yield from A.move(target, grip=grip, finger=d.FINGER_OPEN, t=1.2, label="onto " + label)
    yield from A.servo(target, timeout=1.0, label="servo " + label)
    if pinch:
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


MODES = ("full", "none", "liftonly", "leveronly", "overshoot", "leveropen", "restwheel", "lightlever", "regrab", "drop", "held_fast")
DROP_DELAY = 0.3  # s after the release before the lever hand shuts the lever
HELD_DELAY = 0.5


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
    goal = 9.0 if mode == "overshoot" else TARGET
    LB, RB = (-0.08, 0.0, 0.0), (0.0, 0.08, 0.0)
    if mode in ("drop", "held_fast"):
        yield from _onto(L, _rim_grip(0.0), GRIP_L, LB, 0.025, label="rim")
        yield from _onto(R, _tab_grip(0.0, 0.0), GRIP_R, RB, 0.010, label="paddle")
        yield from _lever_to(ctx, R, -24.0)
        if mode == "drop":
            # lazy: raise the wheel, let it go, and shut the lever while it falls past a notch with no hand on it
            yield from _lift(ctx, L, 12.0)
            th, lv = ctx.deg("steering", "column"), ctx.deg("steering", "lever")
            yield from _lever_to(ctx, R, -4.5, over=0.0)
            yield from ctx.together(_then(L.open(t=0.2, label="drop"), L.move(L.at() + np.asarray(LB), t=0.3, label="clear")), _then(R.wait(DROP_DELAY), _lever_to(ctx, R, -0.5)))
        else:
            # not a drop: the hand carries the wheel quickly through a notch while the lever shuts
            yield from _lever_to(ctx, R, -4.5, over=0.0)
            yield from ctx.together(_lift(ctx, L, 12.0, step=0.5), _then(R.wait(HELD_DELAY), _lever_to(ctx, R, -0.5)))
        _phase("end")
        yield from R.wait(2.0)
        return
    if mode != "leveronly":
        yield from _onto(L, _rim_grip(0.0), GRIP_L, LB, 0.025, label="rim")
        _phase("rim held")
    if mode != "liftonly":
        yield from _onto(R, _tab_grip(0.0, 0.0), GRIP_R, RB, 0.010, pinch=mode != "lightlever", label="paddle")
        yield from _lever_to(ctx, R, -24.0)
        yield from _off(R, RB, "paddle")
        _phase("lever down")
    if mode != "leveronly":
        yield from _lift(ctx, L, goal)
        _phase("lifted")
    if mode == "leveropen":
        yield from _off(L, LB, "rim")
        yield from L.wait(3.0)
        _phase("end")
        return
    if mode != "liftonly":
        th, lv = ctx.deg("steering", "column"), ctx.deg("steering", "lever")
        yield from _onto(R, _tab_grip(th, lv), _roll(GRIP_R, th + lv), RB, 0.010, label="paddle again")
        yield from _lever_to(ctx, R, -0.5)
        _phase("lever up")
        yield from _off(R, RB, "paddle")
    if mode == "regrab":
        yield from L.open(label="open rim")
        yield from L.wait(1.0)
        yield from L.close(0.025, 40.0, label="close rim again")
    elif mode not in ("restwheel", "leveronly"):
        yield from _off(L, LB, "rim")
    yield from R.wait(3.0)
    _phase("end")
