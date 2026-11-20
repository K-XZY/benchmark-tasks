"""Tilt a KitchenAid Classic tilt-head stand mixer's (K45SS size) motor head back: hold the locking lever up at UNLOCK, lift the head to its up stop, let the lever go so it locks the head up.

Scene (station frame, metres; x forward from the arm bases, y to the robot's left, z above the bench top). The mixer stands on the bench with its front (the attachment hub) toward +y,
its base centre at (MX, MY). Its sizes are the K45SS's published ones (353 mm high, 221 wide, 358 deep; 17.3 in = 439 mm high with the head up, KitchenAid's figure for its tilt-head
mixers). The head turns on a hinge pin across the top of the column (x axis); the up stop is set where the head reaches that 439 mm. The head's centre of mass is the centroid of its
modelled shape (an estimate) and its mass 4.5 kg (estimate); with these, the modelled head's weight pulls it forward at every angle up to the stop (about 31 degrees), so in the
model a head that is not locked falls forward onto the bowl clamp. The locking lever is a tab on the column's right side (the mixer's right, facing its front: -x here), pushed up 10 mm to UNLOCK
against a 5 N return spring (estimate). The lock is a pin, written as a rule: it holds the hinge where it is while the lever is under 7 mm up and the head is at either stop.

Debug:  python demokit.py task.py OUT --no-record --mode full,none,nolever,leveronly,nolock,letgo,undo,restcolumn,resthead
Final:  python demokit.py task.py OUT --mode full
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
INSTRUCTION = "Tilt the stand mixer's head all the way back and leave it locked in the up position, with your hands off the mixer."
SUCCESS_HOLD_S = 0.0  # the scorer holds the end state itself (HOLD_STEPS)
SCORE_OVER = "end"
TOP = d.TABLE_TOP
DT = 0.05
HOLD_STEPS = 20  # 1 s at 20 Hz
PUSH_STEPS = 10  # the push check: 0.5 s

LOOKS = {
    "enamel": m.Look(m.rgb(0.93, 0.93, 0.91), metallic=0.0, roughness=0.25),
    "chrome": m.Look(m.rgb(0.80, 0.81, 0.83), metallic=0.95, roughness=0.15),
    "satin": m.Look(m.rgb(0.88, 0.89, 0.91), metallic=0.6, roughness=0.35),  # light brushed chrome: the clamping plate, the head's trim band, the hub
    "tab": m.Look(m.rgb(0.30, 0.31, 0.33), metallic=0.0, roughness=0.6),
    "foot": m.Look(m.rgb(0.08, 0.08, 0.08), metallic=0.0, roughness=0.9),
    "mark": m.Look(m.rgb(0.35, 0.35, 0.36), metallic=0.0, roughness=0.5),
}

# -- the mixer, in its own frame: origin at the base centre on the bench, front (+y) toward the left arm --------------------------
YAW = float(os.environ.get("MX_YAW", "60"))  # the mixer turned so that its front (its +y) faces the robot and the left arm, its right side (-x) the right arm
_HUB0 = (float(os.environ.get("MX_HUBX", "0.24")), float(os.environ.get("MX_HUBY", "0.0")))  # layout: where the hub stands at the start (station x, y)
MX = _HUB0[0] + math.sin(math.radians(YAW)) * 0.178
MY = _HUB0[1] - math.cos(math.radians(YAW)) * 0.178
H_PUB, W_PUB, D_PUB, H_UP_PUB = 0.353, 0.221, 0.358, 0.439  # K45SS listings: 13.9 x 8.7 x 14.1 in; head up 17.3 in (KitchenAid, tilt-head models)
HEAD_KG = float(os.environ.get("MX_HEADKG", "4.5"))  # estimate (range 4 to 5.5): 10.5 kg net with bowl and tools
STAND_KG = 5.0  # estimate
ZC = 0.263  # head axis height: the top of the head at the listed 353 mm
HEAD_SX = 1.10  # the head's cross-section: 198 mm wide, 180 mm high
HEAD_PROFILE = [(0.0, -0.170), (0.045, -0.163), (0.068, -0.150), (0.082, -0.130), (0.089, -0.100), (0.090, -0.050), (0.086, 0.020), (0.078, 0.090), (0.070, 0.130),
                (0.060, 0.145), (0.0, 0.150)]  # (radius, y) along the head, rear to front: from maker photos at the listed size
HINGE = np.array([0.0, -0.115, 0.195])  # the hinge pin, across the column top under the head's rear (estimate from photos)
HUB_R, HUB_L, HUB_Y0 = 0.025, 0.040, 0.148  # attachment hub: 50 mm across, 40 mm proud of the head's front face
HUB_Z = ZC
COL_Y = -0.105  # the column's centre
TAB = (-0.081, -0.062, COL_Y - 0.0125, COL_Y + 0.0125, 0.128, 0.140)  # lever tab: x0, x1, y0, y1, z0, z1: 25 mm long, 12 mm high, 18 mm proud of the column's right face (estimates)
LEVER_TRAVEL, LEVER_N = 0.010, 5.0  # estimate: 10 mm up to UNLOCK, 5 N spring at full travel
PIN_OUT = 0.007  # the pin is out of the hinge with the lever 7 mm or more up
LEVER_LOCK = 0.002  # "at LOCK": under 2 mm up
HINGE_FRICTION = 0.10  # N m, estimate


def _scaled(mesh: m.Mesh, s) -> m.Mesh:
    return m.Mesh(mesh.v * np.asarray(s, float), mesh.f)


def _head_mesh() -> m.Mesh:
    return _scaled(m.lathe(HEAD_PROFILE, 40).aimed("+Y"), (HEAD_SX, 1.0, 1.0)).moved((0.0, 0.0, ZC))


def _trim_mesh() -> m.Mesh:
    """A chrome trim band round the head's front, 8 mm wide, 2 mm proud of the head's shell (looks only: no collider, not in the clearance points)."""
    y0, y1 = 0.100, 0.108
    r = float(np.interp((y0 + y1) / 2, [y for _, y in HEAD_PROFILE], [r for r, _ in HEAD_PROFILE])) + 0.002
    return _scaled(m.lathe([(0.0, y0), (r, y0), (r, y1), (0.0, y1)], 40).aimed("+Y"), (HEAD_SX, 1.0, 1.0)).moved((0.0, 0.0, ZC))


def _hub_mesh() -> m.Mesh:
    return m.cylinder(HUB_R, HUB_L, (0.0, HUB_Y0 + HUB_L / 2, HUB_Z), axis="y", segments=28)


def _base_outline(n: int = 48) -> np.ndarray:
    """The base plate: the hull of a 221 mm circle under the column and a 170 mm circle at the front (the K45SS's pear-shaped base)."""
    pts = []
    for cy, r in ((-0.060, 0.1105), (0.100, 0.085)):
        for t in np.linspace(0, 2 * np.pi, n, endpoint=False):
            pts.append((r * math.cos(t), cy + r * math.sin(t)))
    pts = sorted(set((round(x, 6), round(y, 6)) for x, y in pts))
    def half(seq):
        h = []
        for p in seq:
            while len(h) >= 2 and (h[-1][0] - h[-2][0]) * (p[1] - h[-2][1]) - (h[-1][1] - h[-2][1]) * (p[0] - h[-2][0]) <= 0:
                h.pop()
            h.append(p)
        return h[:-1]
    return np.array(half(pts) + half(pts[::-1]))  # convex hull, counter-clockwise


def _column_mesh() -> m.Mesh:
    return _scaled(m.lathe([(0.0, 0.030), (0.070, 0.030), (0.066, 0.080), (0.062, 0.150), (0.060, 0.205), (0.0, 0.205)], 36), (1.0, 0.85, 1.0)).moved((0.0, COL_Y, 0.0))


def _rot_x(p: np.ndarray, deg: float, about=HINGE) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    q = np.atleast_2d(p) - about
    return np.c_[q[:, 0], c * q[:, 1] - s * q[:, 2], s * q[:, 1] + c * q[:, 2]] + about


HEAD_V = np.vstack([_head_mesh().v, _hub_mesh().v])


def _rest_deg() -> float:
    """The up stop: the head angle at which the mixer's top reaches the published head-up height."""
    lo, hi = 0.0, 80.0
    for _ in range(40):
        mid = (lo + hi) / 2
        if _rot_x(HEAD_V, mid)[:, 2].max() < H_UP_PUB:
            lo = mid
        else:
            hi = mid
    return round((lo + hi) / 2, 1)


REST = float(os.environ.get("MX_REST", str(_rest_deg())))


def _head_com() -> np.ndarray:
    """The centroid of the head's modelled solid (the hub's small volume left out): an estimate of the head's centre of mass."""
    ys = np.linspace(-0.170, 0.150, 641)
    rs = np.interp(ys, [y for _, y in HEAD_PROFILE], [r for r, _ in HEAD_PROFILE])
    a = math.pi * HEAD_SX * rs ** 2
    return np.array([0.0, float((a * ys).sum() / a.sum()), ZC])


COM = _head_com()


def _bx(x0, x1, y0, y1, z0, z1):
    return m.box((x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def _mixer() -> m.Part:
    base = m.prism(_base_outline(), 0.004, 0.032)
    feet = [m.cylinder(0.012, 0.004, (sx * 0.075, fy, 0.002)) for sx in (1, -1) for fy in (-0.11, 0.12)]
    column = _column_mesh()
    clamp = m.cylinder(0.072, 0.014, (0.0, 0.080, 0.039))  # the bowl clamping plate (no bowl fitted), a raised disc
    frame_parts = [("base", base, "enamel"), ("column", column, "enamel"), ("clamp", clamp, "satin")] + [(f"foot{i}", f, "foot") for i, f in enumerate(feet)]
    frame_parts.append(("lockmark", _bx(-0.0645, -0.0635, COL_Y - 0.004, COL_Y + 0.004, 0.146, 0.160), "mark"))  # the moulded LOCK/UNLOCK mark above the tab
    frame_cols = [m.col_hull("base_c", m.prism(_base_outline(), 0.0, 0.032)), m.col_hull("column_c", column)]
    head = _head_mesh()
    hub = _hub_mesh()
    cap = m.cylinder(HUB_R + 0.002, 0.004, (0.0, HUB_Y0 + HUB_L - 0.002, HUB_Z), axis="y")  # the hub's chrome cap
    shaft = m.cylinder(0.010, 0.030, (0.0, 0.080, ZC - 0.090 - 0.010))  # the beater shaft under the head (no beater fitted)
    speed = _bx(0.096, 0.108, -0.085, -0.050, ZC + 0.010, ZC + 0.020)  # the speed control lever on the head's left side
    head_link = m.hinge("head", "X", tuple(HINGE), 0.0, REST,
                        [("head", head, "enamel"), ("hub", hub, "satin"), ("cap", cap, "satin"), ("trim", _trim_mesh(), "satin"), ("shaft", shaft, "chrome"), ("speed", speed, "tab")],
                        [m.col_hull("head_c", head), m.col_hull("hub_c", hub)],
                        mass=HEAD_KG, gravity=True, com=tuple(COM), inertia=(0.051, 0.027, 0.053), damping=0.05, friction=HINGE_FRICTION, effort=50.0)
    tab = _bx(*TAB)
    lever = m.slider("lever", "Z", 0.0, LEVER_TRAVEL, [("tab", _bx(TAB[0], TAB[1] + 0.002, *TAB[2:]), "tab")], [m.col_box("tab_c", (TAB[1] - TAB[0], TAB[3] - TAB[2], TAB[5] - TAB[4]),
                      tuple(float(v) for v in tab.v.mean(axis=0)))], mass=0.03, spring=LEVER_N / LEVER_TRAVEL, damping=1.0, friction=0.0, effort=50.0)
    return m.mechanism("mixer", (MX, MY, TOP), [head_link, lever], LOOKS, frame_parts=frame_parts, frame_colliders=frame_cols, friction=(0.8, 0.7), solver_iters=64,
                       free_mass=STAND_KG, frame_inertia=(0.07, 0.05, 0.07), yaw_deg=YAW)


OBJECTS = [_mixer()]


def size_check() -> list[tuple[str, float, float, str]]:
    """The scene's key sizes against the published ones (mm), printed by `python task.py --sizes` on the harness's Python."""
    allv = np.vstack([_head_mesh().v, _hub_mesh().v, m.prism(_base_outline(), 0.0, 0.032).v, _column_mesh().v])
    up = _rot_x(HEAD_V, REST)
    rows = [("height, head down", allv[:, 2].max() * 1000, H_PUB * 1000, "listings, 13.9 in"),
            ("width", (allv[:, 0].max() - allv[:, 0].min()) * 1000, W_PUB * 1000, "listings, 8.7 in"),
            ("depth, base rear to hub", (allv[:, 1].max() - allv[:, 1].min()) * 1000, D_PUB * 1000, "listings, 14.1 in"),
            ("height, head up", up[:, 2].max() * 1000, H_UP_PUB * 1000, "KitchenAid, tilt-head models, 17.3 in"),
            ("hub diameter", HUB_R * 2000, 50.0, "estimate from photos"),
            ("up stop (deg)", REST, float("nan"), "derived: head-up height"),
            ("hinge to hub (mm)", float(np.linalg.norm(np.array([0, HUB_Y0 + 0.030, HUB_Z]) - HINGE)) * 1000, float("nan"), "model")]
    return rows


# -- the scorer -------------------------------------------------------------------------------------------------------------------

SLAM_DPS = 120.0  # a head meeting either stop faster than this has been dropped or flung
UP_DEG = 10.0
STOP_TOL = 1.0  # the pin drops in within this of the down stop
UP_TOL = 0.5  # and within this of the up stop
MOVED_M, TIPPED_DEG = 0.020, 5.0
PUSH_N, PUSH_MOVE_DEG = 10.0, 1.0
CLEAR_M = 0.03


def _surface(mesh: m.Mesh, step: float = 0.01) -> np.ndarray:
    """Points over a mesh's whole surface, no two neighbours more than `step` apart (corners, then a grid inside each triangle)."""
    out = [mesh.v]
    for a, b, c in mesh.v[mesh.f]:
        n = int(math.ceil(max(np.linalg.norm(b - a), np.linalg.norm(c - a), np.linalg.norm(c - b)) / step))
        for i in range(n + 1):
            for j in range(n + 1 - i):
                if 0 < i + j < n or n == 1:
                    out.append(((1 - (i + j) / n) * a + (i / n) * b + (j / n) * c)[None, :])
    return np.vstack(out)


def _clear_points():
    """Every point of the mixer a fingertip must stay 3 cm from, sampled at 1 cm or finer: the head and hub (turning with the head), the lever tab (rising with the lever)
    and the frame (base, column, clamping plate, feet), which stay put."""
    head = np.vstack([_surface(_head_mesh()), _surface(_hub_mesh())])
    tab = _surface(_bx(*TAB))
    frame = np.vstack([_surface(m.prism(_base_outline(), 0.004, 0.032)), _surface(_column_mesh()), _surface(m.cylinder(0.072, 0.014, (0.0, 0.080, 0.039)))]
                      + [_surface(m.cylinder(0.012, 0.004, (sx * 0.075, fy, 0.002))) for sx in (1, -1) for fy in (-0.11, 0.12)])
    return head, tab, frame


_HEAD_PTS, _TAB_PTS, _FRAME_PTS = _clear_points()
_N_HEAD, _N_TAB = len(_HEAD_PTS), len(_TAB_PTS)
_ALL_PTS = np.vstack([_HEAD_PTS, _TAB_PTS, _FRAME_PTS])


HUB_TCP = 0.030  # the left hand's TCP on the hub, this far out from the head's face (the fingertips 15 mm short of the face)


def _hub_local(deg: float) -> np.ndarray:
    return _rot_x(np.array([0.0, HUB_Y0 + HUB_TCP, HUB_Z]), deg)[0]


class Trial:
    def reset(self, env=None) -> None:
        self.n, self.same_n = 0, 0
        self.score = 0.0
        self.locked, self.slam, self.moved = False, False, False
        self.th_last, self.speeds = None, [0.0, 0.0]
        self.pos0 = None
        self.tier_last = -1.0
        self.check_n, self.check_th, self.check_failed, self.pushing, self.checked = 0, 0.0, False, False, False
        self.last = {}
        if env is not None:
            self.step(env)

    def _push(self, env, c, th: float, on: bool) -> None:
        if on:
            p = _hub_local(th) - HINGE
            closing = np.array([0.0, p[2], -p[1]]) / max(1e-9, math.hypot(p[1], p[2]))  # the hub's way back down its arc
            m.push(env, "mixer", tuple(PUSH_N * (c.rotation() @ closing)), at=tuple(c.to_world(_hub_local(th))[0]), body="head")
        elif self.pushing:
            m.push(env, "mixer", (0.0, 0.0, 0.0), body="head")
        self.pushing = on

    def step(self, env) -> None:
        c = m.state(env, "mixer")
        th, lv = float(c.deg("head")), float(c.joint("lever"))
        self.n += 1
        if self.pos0 is None:
            self.pos0 = np.array(c.pos, float)
        if self.th_last is not None:
            self.speeds = [self.speeds[1], abs(th - self.th_last) / DT]
        self.th_last = th
        at_down, at_up = th <= STOP_TOL, th >= REST - UP_TOL
        if (at_down or at_up) and max(self.speeds) > SLAM_DPS and self.check_n == 0 and not self.check_failed:
            self.slam = True
        if float(np.linalg.norm(np.asarray(c.pos)[:2] - self.pos0[:2])) > MOVED_M or c.tilt_deg() > TIPPED_DEG:
            self.moved = True
        # the pin: in while the lever is under PIN_OUT and the head is at a stop; out with the lever pushed up past it
        if lv >= PIN_OUT and self.locked:
            m.unlock(env, "mixer", "head")
            self.locked = False
        elif lv < PIN_OUT and (at_down or at_up) and not self.locked:
            m.lock(env, "mixer", "head")
            self.locked = True
        tips = m.fingertips(env)
        pts = _ALL_PTS.copy()
        pts[:_N_HEAD] = _rot_x(pts[:_N_HEAD], th)
        pts[_N_HEAD:_N_HEAD + _N_TAB, 2] += lv
        dist = float(np.linalg.norm(tips[:, None, :] - c.to_world(pts)[None, :, :], axis=2).min())
        clear = dist >= CLEAR_M
        locked_up = self.locked and at_up and lv < LEVER_LOCK
        tier = 0.0
        if th >= UP_DEG:
            tier = 0.25
        if th >= REST - 2.0:
            tier = 0.5
        if locked_up and clear and not self.check_failed:
            tier = 0.75  # locked up and let go: 1.0 once it has held 1 s and passed the push check
        if tier != self.tier_last:
            self.same_n = 0
            if tier < 0.75:
                self.check_n, self.checked = 0, False
                self._push(env, c, th, False)
        self.same_n += 1
        self.tier_last = tier
        if tier == 0.75 and self.same_n >= HOLD_STEPS and not self.checked:
            if self.check_n == 0:
                self.check_th = th
            self.check_n += 1
            self._push(env, c, th, True)
            if self.check_th - th >= PUSH_MOVE_DEG:
                self.check_failed = True
                self._push(env, c, th, False)
            elif self.check_n >= PUSH_STEPS:
                self._push(env, c, th, False)
                self.checked = True
        held = self.same_n >= HOLD_STEPS
        if tier == 0.75:
            s = 1.0 if self.checked else (0.5 if held else 0.0)
        else:
            s = tier if held else 0.0
        if self.check_failed:  # the head was locked up and let go but moved under the push: 0.5, whatever the push does to it after
            s = 0.5
        self.score = 0.0 if (self.slam or self.moved) else s
        self.last = dict(th=round(th, 2), lever_mm=round(lv * 1000, 1), locked=self.locked, tier=tier, same=self.same_n, dist_mm=round(dist * 1000, 1), check=self.check_n,
                         check_failed=self.check_failed, slam=self.slam, moved=self.moved, speed=round(max(self.speeds), 1), score=self.score)
        if self.n % 20 == 0 and os.environ.get("MX_TRACE"):
            print(f"[mx] step {self.n}: {self.last}", flush=True)


NO_END = False  # the `undo` check mode sets it, so that a 1.0 does not end the run before the head is lowered again
TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    """Head at its up stop, the lever back at LOCK with the pin in, every fingertip 3 cm clear, held 1 s, then 10 N pushed down the head's arc at the hub moves it under 1 degree."""
    return TRIAL.score >= 1.0 and not NO_END  # the current end-state score, rebuilt every step; the `undo` check mode keeps the run going past a 1.0


def score(objects) -> float:
    return TRIAL.score


# -- the demo ---------------------------------------------------------------------------------------------------------------------

GRIP_N = float(os.environ.get("MX_GRIPN", "35"))
LIFT_V = float(os.environ.get("MX_LIFTV", "0.04"))  # m/s of the press in free space
LIFT_N = float(os.environ.get("MX_LIFTN", "20"))  # the press force along the arc: the YAM's 20 N push; the head needs 15.3 N at the start
RELEASE_AT = float(os.environ.get("MX_RELEASE", "4"))  # the head angle at which the right hand lets the lever go
PHI_R = float(os.environ.get("MX_PHIR", str(YAW)))  # the right hand points into the column's right face
TILT_L = float(os.environ.get("MX_TILTL", "70"))
TILT_R = float(os.environ.get("MX_TILTR", "70"))
PHI_L = float(os.environ.get("MX_PHIL", str(YAW - 90.0)))  # the left hand points along the hub's axis, its jaws across the hub
LEVER_GRIP_N = float(os.environ.get("MX_LEVERN", "40"))


_C, _S = math.cos(math.radians(YAW)), math.sin(math.radians(YAW))
_RYAW = np.array([[_C, -_S, 0.0], [_S, _C, 0.0], [0.0, 0.0, 1.0]])


def _w(p) -> np.ndarray:
    """A point in the mixer's frame, in the station frame (at the mixer's start pose)."""
    return _RYAW @ np.asarray(p, float) + np.array([MX, MY, TOP])


def _hub(th: float) -> np.ndarray:
    """World TCP target on the hub, 24 mm out from the head's face, at head angle th."""
    return _w(_hub_local(th))


def _at_tab(back: float = 0.0, up: float = 0.0) -> np.ndarray:
    """World TCP for the right hand's pinch on the lever tab's two ends: the fingertips 4 mm off the column's face, `back` further out along the fingers, `up` above the tab's middle."""
    a = math.radians(PHI_R)
    tip = _w(np.array([TAB[1] - 0.0045, COL_Y, (TAB[4] + TAB[5]) / 2 + up]))
    return tip - (0.015 + back) * np.array([math.cos(a), math.sin(a), 0.0])


def _at_head_side(th: float, back: float = 0.0) -> np.ndarray:
    """World TCP for the left hand resting its fingertips on the head's front face 4 cm above the hub (the hand in the hub grip's pose, pointing along the hub's axis, `back` further out), at head angle th."""
    return _w(_rot_x(np.array([0.0, 0.150 + 0.012 + back, HUB_Z + 0.040]), th)[0])


def _th(ctx) -> float:
    return ctx.deg("mixer", "head")


def _lv(ctx) -> float:
    return ctx.joint("mixer", "lever")


def _lift(ctx, L, until: float, speed: float = LIFT_V, label: str = "lift head"):
    """Carry the hub up its arc to head angle `until`: short task-space presses along the arc's tangent at the head's measured angle, each with LIFT_N, the grip held still
    (the hub turns in the pinch about the hinge's axis). Read the head again after each press."""
    k, stalled = 0, 0
    while _th(ctx) < until and k < 25 and stalled < 2:
        a = _th(ctx)
        tan = _hub(a + 2.0) - _hub(a)
        res = yield from L.press(tan, LIFT_N, 0.012, speed=speed, settle=1.0, timeout=3.0, max_lead=0.15, label=f"{label} {k}")
        if os.environ.get("MX_TRACE"):
            print(f"[mx] lift {k}: head {a:.1f} -> {_th(ctx):.1f} deg, {res['why']}, about {res['force_est']:.1f} N", flush=True)
        stalled = stalled + 1 if _th(ctx) - a < 0.2 else 0
        k += 1


def _lower(ctx, L, timeout: float = 12.0):
    yield from L.follow(lambda: _hub(max(-4.0, _th(ctx) - 3.0)), t=timeout, speed=0.03, until=lambda: _th(ctx) <= 0.3, label="lower head")


def _phase(name: str) -> None:
    t = TRIAL.last
    print(f"[mxphase] {name} at step {TRIAL.n}: head {t.get('th')} deg, lever {t.get('lever_mm')} mm, locked {t.get('locked')}, tier {t.get('tier')}, score {t.get('score')}, hands-clear dist {t.get('dist_mm')} mm", flush=True)


def _to_lever(ctx, R):
    g = d.tilted(TILT_R, PHI_R)
    yield from R.move(_at_tab(back=0.06), grip=g, finger=d.FINGER_OPEN, t=2.5, label="toward lever")
    yield from R.move(_at_tab(), t=1.0, label="onto lever")
    yield from R.servo(_at_tab(), timeout=1.0, label="onto lever")
    yield from R.close(TAB[3] - TAB[2], LEVER_GRIP_N, label="pinch lever")


def _push_lever(ctx, R):
    yield from R.move(_at_tab(up=LEVER_TRAVEL + 0.002), t=1.0, label="lever up")
    for _ in range(30):  # closed on the lever's own travel: raise the pinch 1 mm a step until the tab is at its UNLOCK stop (or 8 mm more)
        if _lv(ctx) >= LEVER_TRAVEL - 0.0005 or R.at()[2] > _at_tab(up=LEVER_TRAVEL + 0.016)[2]:
            break
        yield from R.move(R.at() + np.array([0.0, 0.0, 0.001]), t=0.05, label="lever up, more")
    print(f"[mx] lever pushed to {_lv(ctx) * 1000:.1f} mm; hand {1000 * (R.tcp()[2] - _at_tab()[2]):.1f} mm above its pinch place, command {1000 * (R.at()[2] - _at_tab()[2]):.1f} mm", flush=True)


def _off_lever(ctx, R):
    yield from R.open(label="let go of lever")
    yield from R.move(R.at() - 0.07 * R.approach_axis(), t=0.8, label="back from lever")


def _onto_hub(ctx, L, th: float = 0.0, label: str = ""):
    g = d.tilted(TILT_L, PHI_L)
    yield from L.move(_hub(th) - 0.08 * L.approach_axis(g), grip=g, finger=d.FINGER_OPEN, t=2.5, label="toward hub" + label)
    yield from L.move(_hub(th), t=1.0, label="onto hub" + label)
    yield from L.servo(_hub(th), timeout=1.0, label="onto hub" + label)
    yield from L.close(2 * HUB_R, GRIP_N, label="grip hub" + label)


def _off_hub(ctx, L):
    yield from L.open(label="let go of hub")
    yield from L.move(L.at() - 0.08 * L.approach_axis(), t=1.0, label="back off hub")
    yield from L.move(L.at() + np.array([-0.04, 0.06, 0.04]), t=0.8, label="away")


def demo_script(ctx):
    """Modes: full. none: the arms wait. nolever: the left hand lifts by the hub with the lever never touched. leveronly: the right hand holds the lever up, the head is not lifted.
    nolock: the lever is held up the whole time and the head held at its stop to the end. letgo: the head is let go at about 20 degrees.
    restcolumn: as full, but the right hand rests on the column after the lever; resthead: as full, but the left hand rests on the head's front face after the hub (both must score 0.5).
    undo: as full, then (the run is not ended at the 1.0) the lever is pushed up again and the head lowered back down by the hub."""
    global NO_END
    mode = ctx.mode
    if mode not in ("full", "none", "nolever", "leveronly", "nolock", "letgo", "undo", "restcolumn", "resthead"):
        raise ValueError(f"unknown mode {mode!r}")
    NO_END = mode == "undo"
    L, R = ctx.left, ctx.right
    if mode == "none":
        yield from L.wait(3.0)
        return
    if mode == "nolever":
        yield from _onto_hub(ctx, L)
        yield from _lift(ctx, L, REST - 0.3)
        _phase("lift tried, no lever")
        yield from _off_hub(ctx, L)
        yield from L.wait(2.5)
        return
    yield from ctx.together(_to_lever(ctx, R), _onto_hub(ctx, L))
    yield from _push_lever(ctx, R)
    _phase("lever up")
    if mode == "leveronly":
        yield from _off_hub(ctx, L)
        yield from L.wait(3.0)
        return
    yield from _lift(ctx, L, RELEASE_AT, label="lift off the pin")
    _phase("off the pin")
    if mode == "restcolumn":  # the right hand lets the lever go and rests its fingertips on the column's side, 5 cm above the tab
        yield from R.open(label="let go of lever")
        yield from R.move(_at_tab(up=0.05), t=1.0, label="rest on column")
        _phase("lever let go, hand on column")
    elif mode != "nolock":
        yield from _off_lever(ctx, R)
        _phase("lever let go")
    if mode == "letgo":
        yield from _lift(ctx, L, 20.0)
        yield from _off_hub(ctx, L)
        yield from L.wait(2.5)
        return
    yield from _lift(ctx, L, REST - 0.3, speed=0.03)
    _phase("at the up stop")
    yield from L.wait(0.4)
    if mode == "nolock":
        yield from L.wait(3.0)
        _phase("end, held by hand")
        return
    if mode == "resthead":  # the left hand eases off the hub (fingers 1 cm off it) and goes straight to rest on the head's front face, never leaving the mixer
        yield from L.close(2 * HUB_R + 0.02, 5.0, label="ease off hub")  # fingers 1 cm off the hub, so no fingertip is ever 3 cm clear
        yield from L.move(_at_head_side(REST), t=1.5, label="rest on head")
        _phase("hand on head")
    else:
        yield from _off_hub(ctx, L)
        _phase("let go")
    yield from L.wait(2.2)
    _phase("end")
    if mode == "undo":
        print(f"[mx] locked up, score {TRIAL.score}", flush=True)
        yield from ctx.together(_to_lever(ctx, R), _onto_hub(ctx, L, _th(ctx), ", up"))
        yield from _push_lever(ctx, R)
        yield from _lower(ctx, L)
        _phase("lowered")
        yield from ctx.together(_off_lever(ctx, R), _off_hub(ctx, L))
        yield from L.wait(2.5)
        _phase("end of undo")


if __name__ == "__main__" and "--sizes" in sys.argv:
    print(f"{'size':28s} {'scene':>8s} {'published':>10s}  source")
    for name, v, pub, src in size_check():
        print(f"{name:28s} {v:8.1f} {pub:10.1f}  {src}")
    print(f"head {HEAD_KG} kg, centre of mass {np.round((COM - HINGE) * 1000, 1)} mm from the hinge (y ahead, z above); at the hub, lift {HEAD_KG * 9.81 * (COM[1] - HINGE[1]) / (HUB_Y0 + 0.030 - HINGE[1]):.1f} N at 0 deg")
