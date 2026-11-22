"""Bus two full-size porcelain dinner plates from a table into a deep bus tub, then move both hands clear.

A 0.54 kg plate lying flat has nothing to hold: its rim stands under 2 cm off the table, lower than a gripper's fingers can reach
under. One hand presses the far side of the rim so the plate tips up on its foot ring; the other hand closes on the raised near rim,
lifts the plate, carries it over the tub's 17.6 cm wall and lays it on the tub's floor, the second plate on the first.

    python demokit.py task.py OUT --mode full                       final film
    python demokit.py task.py OUT --no-record --stills 40 --mode full,none

Station (positions in metres from the midpoint between the two arm bases, on the table: x forward, y to the robot's left):
- Cambro 21157CBP Cambox deep bus box, 51.3 x 38.6 x 17.6 cm, at the right;
- two Libbey 1502-10235 9 in porcelain plates, face up, where two diners left them.
Maker names are for identification only.
"""

from __future__ import annotations

import collections
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
INSTRUCTION = "Set both plates down in the bus tub, then move both hands clear."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own 1 s hold, so score() and success() agree
SCORE_OVER = "end"
HOLD_STEPS = 20  # 1 s at 20 Hz
TOP = d.TABLE_TOP
TRACE = bool(os.environ.get("BUS_TRACE"))

def _hexa(p) -> m.Mesh:
    """A convex solid from 8 corners: the bottom four counter-clockwise from above, then the top four."""
    f = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    return m.Mesh(np.asarray(p, float), np.array(f))


P_R = 0.1145  # 9 in across
P_H = 0.022  # 7/8 in
P_MASS = 0.54  # 14.35 lb per case of 12, with its packing
P_EDGE_LOW = 0.016  # the rim's underside at its edge, above the table
P_FOOT = (0.058, 0.066)  # the foot ring, inner and outer radius
P_RIM_X = [(0.0794, 0.005), (P_R, P_EDGE_LOW), (P_R, P_H), (0.085, 0.0175), (0.0794, 0.0105)]  # the rim's cross-section, (r, z), round the solid
P_SEGMENTS = 24


def plate_mesh() -> m.Mesh:
    prof = [(0.0, 0.004), (P_FOOT[0] - 0.002, 0.004), (P_FOOT[0], 0.0), (P_FOOT[1], 0.0), (P_FOOT[1] + 0.003, 0.004), (0.0794, 0.005), (0.100, 0.0118), (P_R - 0.001, P_EDGE_LOW),
            (P_R + 0.0012, 0.019), (P_R - 0.001, P_H), (0.100, 0.0200), (0.085, 0.0175), (0.0834, 0.0125), (0.0774, 0.0095), (0.0, 0.009)]
    return m.lathe(prof, 64)


def _wedge(cross, a: float, b: float) -> m.Mesh:
    """A cross-section given as (r, z) swept from angle a to b: a convex prism."""
    n = len(cross)
    pa = [(r * math.cos(a), r * math.sin(a), z) for r, z in cross]
    pb = [(r * math.cos(b), r * math.sin(b), z) for r, z in cross]
    f = [(0, i + 1, i) for i in range(1, n - 1)] + [(n, n + i, n + i + 1) for i in range(1, n - 1)]
    for i in range(n):
        j = (i + 1) % n
        f += [(i, j, n + j), (i, n + j, n + i)]
    return m.Mesh(np.array(pa + pb, float), np.array(f))


def plate_colliders(prefix: str) -> list[str]:
    """The well and foot ring as one solid, and the raised rim as 24 convex segments, so that a finger can go under the rim's edge."""
    cols = [m.col_hull(f"{prefix}_well", m.lathe([(0.0, 0.0), (P_FOOT[1], 0.0), (0.0794, 0.0095), (0.0, 0.0095)], P_SEGMENTS))]
    th = np.linspace(0.0, 2 * np.pi, P_SEGMENTS + 1)
    for k in range(P_SEGMENTS):
        cols.append(m.col_hull(f"{prefix}_r{k:02d}", _wedge(P_RIM_X, th[k], th[k + 1])))
    return cols


TUB_TOP = (0.513, 0.386)  # 20 3/16 x 15 3/16 in at the rim, x by y
TUB_BOTTOM = (0.463, 0.336)
TUB_H = 0.176  # 6 15/16 in
TUB_WALL = 0.004
TUB_FLOOR = 0.005
TUB_MASS = 1.1  # a design value
TUB_AT = np.array([0.587, -0.37])


def _ring_box(bx: float, by: float, tx: float, ty: float, z0: float, z1: float, side: str) -> np.ndarray:
    w = TUB_WALL

    def quad(hx, hy, z, inset):
        hx, hy = hx - inset, hy - inset
        return {"+x": [(hx, -hy, z), (hx, hy, z)], "-x": [(-hx, hy, z), (-hx, -hy, z)], "+y": [(hx, hy, z), (-hx, hy, z)], "-y": [(-hx, -hy, z), (hx, -hy, z)]}[side]

    lo_o, lo_i, hi_o, hi_i = quad(bx, by, z0, 0.0), quad(bx, by, z0, w), quad(tx, ty, z1, 0.0), quad(tx, ty, z1, w)
    return np.array([lo_i[0], lo_o[0], lo_o[1], lo_i[1], hi_i[0], hi_o[0], hi_o[1], hi_i[1]])


def tub_parts():
    bx, by, tx, ty = TUB_BOTTOM[0] / 2, TUB_BOTTOM[1] / 2, TUB_TOP[0] / 2, TUB_TOP[1] / 2
    parts = [("floor", m.box((TUB_BOTTOM[0], TUB_BOTTOM[1], TUB_FLOOR), (0, 0, TUB_FLOOR / 2)), "tub")]
    cols = [m.col_box("floor_c", (TUB_BOTTOM[0], TUB_BOTTOM[1], TUB_FLOOR), (0, 0, TUB_FLOOR / 2))]
    for side in ("+x", "-x", "+y", "-y"):
        wall = _hexa(_ring_box(bx, by, tx, ty, 0.0, TUB_H, side))
        name = "wall_" + side.replace("+", "p").replace("-", "m")
        parts.append((name, wall, "tub"))
        cols.append(m.col_hull(name + "_c", wall))
    lip = 0.012
    for side, size, at in (("px", (lip, TUB_TOP[1] + 2 * lip, 0.004), (tx + lip / 2, 0, TUB_H - 0.002)), ("mx", (lip, TUB_TOP[1] + 2 * lip, 0.004), (-tx - lip / 2, 0, TUB_H - 0.002)),
                           ("py", (TUB_TOP[0], lip, 0.004), (0, ty + lip / 2, TUB_H - 0.002)), ("my", (TUB_TOP[0], lip, 0.004), (0, -ty - lip / 2, TUB_H - 0.002))):
        parts.append((f"lip_{side}", m.box(size, at), "tub"))
    return parts, cols


# -- the scene ----------------------------------------------------------------------------------

LOOKS = {
    "plate": m.Look(m.rgb(0.95, 0.95, 0.93), roughness=0.25),
    "tub": m.Look(m.rgb(0.16, 0.16, 0.17), roughness=0.55),
}
PLATES = {"plate_l": (0.36, 0.19), "plate_r": (0.37, -0.06)}

_PLATE = plate_mesh()
_TUB_PARTS, _TUB_COLS = tub_parts()
# friction: porcelain on the wood table 0.4, on polypropylene 0.4 (0.35 sliding); the tub on the table 0.4
OBJECTS = [m.free("tub", (TUB_AT[0], TUB_AT[1], TOP + 0.0005), _TUB_PARTS, _TUB_COLS, {"tub": LOOKS["tub"]}, mass=TUB_MASS, friction=(0.4, 0.35))]
OBJECTS += [m.free(n, (x, y, TOP + 0.001), [("body", _PLATE, "plate")], plate_colliders("c"), {"plate": LOOKS["plate"]}, mass=P_MASS, friction=(0.4, 0.35)) for n, (x, y) in PLATES.items()]

# -- the scorer, as plain functions of poses -----------------------------------------------------

DROP = 0.015  # m: its centre, from the frame the jaws open, no more than 1.5 cm above where it comes to rest (the open lower jaw under the rim holds the near rim up, so a flat plate settles about 8 mm)
OPEN_MAX = 0.0075  # m: a hand whose finger joint is opened past this (a pinched rim reads 5.8 mm, open 13 mm) is no longer holding a plate
TOUCH = 0.010  # m: any point of a gripper this close to a dish is touching it
CLEAR = 0.03  # m: both grippers at least 3 cm from each plate and the tub
REST_V = 0.02  # m/s: a plate slower than this for REST_STEPS has come to rest
REST_STEPS = 5
STILL = 0.001  # m: nothing moves more than 1 mm over the hold
_RING = np.linspace(0.0, 2 * np.pi, 16, endpoint=False)
P_POINTS = np.vstack([np.c_[P_R * np.cos(_RING), P_R * np.sin(_RING), np.full(16, P_EDGE_LOW)], np.c_[P_FOOT[1] * np.cos(_RING), P_FOOT[1] * np.sin(_RING), np.zeros(16)],
                      np.c_[P_R * np.cos(_RING), P_R * np.sin(_RING), np.full(16, P_H)]])


def world(points: np.ndarray, pos, R) -> np.ndarray:
    return np.asarray(points) @ np.asarray(R).T + np.asarray(pos)


def lowest(pos, R) -> float:
    return float(world(P_POINTS, pos, R)[:, 2].min())


def tip_gap(pos, R, tips) -> float:
    """The nearest hand point's distance to the plate's outer surface, a disc 2.2 cm thick."""
    q = (np.asarray(tips) - np.asarray(pos)) @ np.asarray(R)
    dr = np.maximum(np.linalg.norm(q[:, :2], axis=1) - P_R, 0.0)
    dz = np.maximum(np.maximum(-q[:, 2], q[:, 2] - P_H), 0.0)
    return float(np.hypot(dr, dz).min())


def plate_place(pos, R, tub) -> bool:
    """Every sampled point of the plate (rim top and underside, foot ring) inside the tub: above its floor, below its rim, inside its sloping walls."""
    tp, tR = tub
    q = (world(P_POINTS, pos, R) - np.asarray(tp)) @ np.asarray(tR)
    f = np.clip(q[:, 2] / TUB_H, 0.0, 1.0)
    hx = (TUB_BOTTOM[0] + (TUB_TOP[0] - TUB_BOTTOM[0]) * f) / 2 - TUB_WALL
    hy = (TUB_BOTTOM[1] + (TUB_TOP[1] - TUB_BOTTOM[1]) * f) / 2 - TUB_WALL
    return bool(np.all(q[:, 2] >= TUB_FLOOR - 0.002) and np.all(q[:, 2] <= TUB_H) and np.all(np.abs(q[:, 0]) <= hx) and np.all(np.abs(q[:, 1]) <= hy))


def look(poses: dict, tips) -> dict:
    """One moment, from poses only. `poses`: name -> (pos, R) for the two plates and the tub; `tips`: points over both grippers (palm, housing, fingers) and wrist links."""
    gaps = {n: tip_gap(*poses[n], tips) for n in PLATES}
    tub_gap = _box_gap(tips, poses["tub"], TUB_TOP[0] / 2 + 0.012, TUB_TOP[1] / 2 + 0.012, TUB_H)
    return {"placed": {n: plate_place(*poses[n], poses["tub"]) for n in PLATES}, "gaps": gaps, "clear": min(gaps.values()) >= CLEAR and tub_gap >= CLEAR,
            "off": [n for n in PLATES if poses[n][0][2] < TOP - 0.05], "mid": {n: float((np.asarray(poses[n][0]) + np.asarray(poses[n][1]) @ [0.0, 0.0, P_H / 2])[2]) for n in PLATES}}


def _box_gap(tips, pose, hx: float, hy: float, h: float) -> float:
    q = (np.asarray(tips) - np.asarray(pose[0])) @ np.asarray(pose[1])
    dx = np.maximum(np.abs(q[:, 0]) - hx, 0.0)
    dy = np.maximum(np.abs(q[:, 1]) - hy, 0.0)
    dz = np.maximum(np.maximum(-q[:, 2], q[:, 2] - h), 0.0)
    return float(np.sqrt(dx * dx + dy * dy + dz * dz).min())


def gripper_points(opening: float) -> np.ndarray:
    """Points on one gripper in its `gripper` body's frame: the palm and housing (from the collision meshes' size) and both fingers from root to tip."""
    xs, ys, zs = np.linspace(-0.05, 0.05, 17), np.linspace(-0.033, 0.033, 12), np.linspace(-0.01, 0.064, 13)
    palm = [(x, y, z) for x in xs for y in ys for z in zs if x in (xs[0], xs[-1]) or y in (ys[0], ys[-1]) or z in (zs[0], zs[-1])]
    fingers = [(sx * (opening + 0.0045), y, z) for sx in (-1.0, 1.0) for y in np.linspace(-0.018, 0.018, 13) for z in np.linspace(0.06, 0.147, 30)]
    return np.array(palm + fingers)


def hand_points(env):
    """Both grippers (palm, housing, fingers) and each wrist link down to the gripper, as points in the station's frame."""
    base = env.unwrapped
    origin = m._np(base.scene.env_origins[0])
    out, closed = [], []
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        P = m._np(robot.data.body_link_pose_w)[0]
        g = P[robot.find_bodies("gripper")[0][0]]
        w = P[robot.find_bodies("link5")[0][0], :3] - origin
        opening = abs(float(m._np(robot.data.joint_pos)[0, robot.find_joints("joint7")[0][0]]))
        mine = [gripper_points(opening) @ m.quat_matrix(g[3:7]).T + g[:3] - origin, np.array([w + (g[:3] - origin - w) * t for t in np.linspace(0.0, 1.0, 6)])]
        out += mine
        if opening <= OPEN_MAX:
            closed += mine
    return np.vstack(out), (np.vstack(closed) if closed else np.zeros((0, 3)))


class Trial:
    """What the scorer holds across a trial: drops and falls latch."""

    def reset(self, env=None) -> None:
        self.n = 0
        self.fail = None
        self.dropped: set[str] = set()
        self.held = {n: False for n in PLATES}
        self.release: dict[str, float] = {}  # plate -> its centre's height at the frame the jaws open, until it comes to rest
        self.touch_low: dict[str, float] = {}
        self.slow = {n: 0 for n in PLATES}
        self.hist: collections.deque = collections.deque(maxlen=HOLD_STEPS + 1)
        self.end = gk.EndHold(HOLD_STEPS)
        self.seen: dict = {}
        self.count = 0
        if env is not None:
            self.step(env)

    def update(self, poses: dict, speeds: dict, tips, grip=None) -> None:
        """`tips`: every hand point; `grip`: the points of the hands whose jaws are still closed (all of `tips` when None)."""
        self.n += 1
        seen = look(poses, tips)
        grip = tips if grip is None else grip
        self.seen = seen
        if self.fail is None and seen["off"]:
            self.fail = f"{seen['off'][0]} left the table"
        for n in PLATES:
            touching = (tip_gap(*poses[n], grip) if len(grip) else 9.0) < TOUCH  # held: a closed hand within 1 cm; the frame the jaws open is the release
            if self.held[n] and not touching:
                self.release[n] = self.touch_low[n]
            if touching:
                self.release.pop(n, None)
                self.touch_low[n] = seen["mid"][n]
            self.held[n] = touching
            at_rest = speeds[n] < REST_V and tip_gap(*poses[n], tips) >= TOUCH  # resting on an open hand is not rest: the fall is measured to where the plate ends
            self.slow[n] = self.slow[n] + 1 if at_rest else 0
            if n in self.release and self.slow[n] >= REST_STEPS:
                fall = self.release.pop(n) - seen["mid"][n]
                print(f"[bus] {n} came to rest, its centre {1000 * fall:.1f} mm below the last touch", flush=True)
                if fall > DROP:
                    self.dropped.add(n)
        self.count = sum(1 for n in PLATES if seen["placed"][n] and n not in self.dropped and seen["gaps"][n] >= TOUCH and n not in self.release)
        self.hist.append(np.array([poses[n][0] for n in list(PLATES) + ["tub"]]))
        still = len(self.hist) > HOLD_STEPS and float(np.linalg.norm(self.hist[-1] - self.hist[0], axis=1).max()) <= STILL
        self.end.step(self.fail is None and self.count == 2 and seen["clear"] and still)

    @property
    def holding(self) -> bool:
        """The end state has held for the last 1 s, now: not a latch, so undoing it takes the bonus away."""
        return self.end.count >= HOLD_STEPS

    @property
    def score(self) -> float:
        if self.fail is not None:
            return 0.0
        return round(0.4 * self.count + (0.2 if self.holding else 0.0), 3)

    def step(self, env) -> None:
        states = {n: m.state(env, n) for n in list(PLATES) + ["tub"]}
        poses = {n: (np.asarray(s.pos, float), s.rotation()) for n, s in states.items()}
        speeds = {n: float(np.linalg.norm(states[n].lin_vel)) for n in PLATES}
        self.update(poses, speeds, *hand_points(env))
        if TRACE and self.n % 20 == 0:
            s = self.seen
            print(f"[bus] step {self.n}: placed {[n for n in PLATES if s['placed'][n]]}, held {[n for n in PLATES if self.held[n]]}, "
                  f"dropped {sorted(self.dropped)}, clear {s['clear']}, fail {self.fail}, score {self.score}", flush=True)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    """Both plates in the tub, set down and let go, both grippers clear, and nothing moving, held for 1 s."""
    return TRIAL.fail is None and TRIAL.holding


def score(objects) -> float:
    return TRIAL.score


# -- the demo -----------------------------------------------------------------------------------
# The demo reads object poses to plan (allowed for a scripted demo); it never moves an object itself.

from yam_arena.urdf import axis_angle_matrix  # noqa: E402


def _rot(axis, deg: float) -> np.ndarray:
    return axis_angle_matrix(np.asarray(axis, float) / np.linalg.norm(axis), math.radians(deg))


# plates: the left hand presses the far rim so the plate tips on its foot ring; the right hand pinches the raised near rim
PRESS_F = float(os.environ.get("BUS_PF", "15"))  # N, down on the far rim
PIN_R = float(os.environ.get("BUS_PIN_R", "0.098"))  # the pinch TCP this far from the plate's centre; the fingertips 13 mm further in
GAP = 0.026  # jaws open this far round the rim, pad to pad
BACK = 0.035  # the fingertips start this far outside the pinch, clear of the edge
SQUEEZE = float(os.environ.get("BUS_SQ", "40"))  # N a finger
CARRY_LOW = 0.05  # the plate's lowest point this far over the tub's rim on the way over
LAY_TILT = 4.0  # deg: laid this close to flat, its far edge on the floor, before the jaws open
SINK = 0.012  # then the open hand goes this far down, the near rim resting on the lower jaw, before the jaw slides out
PHI = {"plate_r": 60.0, "plate_l": 60.0}  # the fingers point this way (deg from +x), from the rim toward the plate's centre, at the pick
PHI_TUB = -60.0  # and this way over the tub: the hand turns the held plate about the vertical on the way
TUB_SPOT = np.array([-0.10, 0.05])
OPEN_SPOT = np.array([0.0, 0.0])  # the lazy routes flop and swing lay a plate here, clear of the walls, so it cannot lean on one  # each plate's centre lands here, in the tub's frame


def to_joints(arm, q, t: float = 2.5, label: str = "to joints"):
    """Go to joint angles `q` along a smooth joint-space path, whatever the arm's posture."""
    q0, q = arm.q.copy(), np.asarray(q, float)
    n = max(1, round(t * d.CONTROL_HZ))
    for k in range(1, n + 1):
        s = k / n
        arm.q = q0 + s * s * (3.0 - 2.0 * s) * (q - q0)
        arm.cmd = np.r_[arm.q, float(np.clip(arm.finger / d.FINGER_OPEN, 0.0, 1.0))]
        yield
    arm.trim = np.zeros(3)
    arm.grip = d._as_grip(d.ik.tcp_pose(arm.q, arm.base)[1])


def _under(r: float) -> float:
    return float(np.interp(r, [0.0794, P_R], [0.005, P_EDGE_LOW]))


def _over(r: float) -> float:
    return float(np.interp(r, [0.0794, 0.085, P_R], [0.0105, 0.0175, P_H]))


def rim_grip(phi_deg: float, pitch_deg: float, normal):
    """Fingers pointing `phi_deg` from +x and `pitch_deg` below level, the jaws closing along the plate's normal: one pad under the rim, one on it."""
    phi, p = math.radians(phi_deg), math.radians(pitch_deg)
    a = np.array([math.cos(phi) * math.cos(p), math.sin(phi) * math.cos(p), -math.sin(p)])
    n = np.asarray(normal, float)
    xc = n - n.dot(a) * a
    xc /= np.linalg.norm(xc)
    return d._as_grip(np.c_[xc, np.cross(a, xc), a])


def plate_tilt(ctx, name: str) -> float:
    return float(math.degrees(math.acos(np.clip(ctx.state(name).rotation()[2, 2], -1.0, 1.0))))


def take_plate(ctx, name: str):
    """Left: press the far rim down so the plate tips on its foot ring and its near rim rises. Right: slide level jaws round the raised rim, pinch, lift."""
    l, r = ctx.left, ctx.right
    up = np.array([0.0, 0.0, 1.0])
    a0 = np.array([math.cos(math.radians(PHI[name])), math.sin(math.radians(PHI[name])), 0.0])
    c = ctx.pos(name)
    far = c + 0.104 * a0 + [0.0, 0.0, _over(0.104)]
    yield from l.move(far + 0.03 * up, grip=d.down(0.0), finger=d.FINGER_CLOSED, t=2.5, label=f"{name}: left over the far rim")
    yield from l.move(far + 0.012 * up, t=1.0, label=f"{name}: left onto the far rim")
    res = yield from l.press(-up, force=PRESS_F, max_travel=0.035, label=f"{name}: left presses the far rim down")
    tilt = plate_tilt(ctx, name)
    print(f"[bus] {name}: pressed ({res['why']}, {res['force_est']:.0f} N), tilt {tilt:.1f} deg", flush=True)

    def pinch():
        st = ctx.state(name)
        loc = st.rotation().T @ (-PIN_R * a0)
        loc[2] = 0.5 * (_under(PIN_R) + _over(PIN_R))
        return st.to_world(np.array([loc]))[0], st.rotation()[:, 2]

    p, n = pinch()
    g = rim_grip(PHI[name], plate_tilt(ctx, name), n)
    a = g.rot[:, 2]
    yield from r.move(p - BACK * a + 0.04 * up, grip=g, finger=-GAP / 2, t=3.0, label=f"{name}: right beside the raised rim")
    yield from r.move(p - BACK * a, t=1.0, label=f"{name}: right level with the rim")
    yield from r.servo(p - BACK * a, tol=0.0015, timeout=1.0, label=f"{name}: right level with the rim")
    p, n = pinch()
    yield from r.move(p, t=1.5, label=f"{name}: fingers round the rim")
    yield from r.servo(p, tol=0.0015, timeout=1.0, label=f"{name}: fingers round the rim")
    yield from r.close(0.0, SQUEEZE, t=0.8, label=f"{name}: pinch the rim")
    yield from r.wait(0.3)
    yield from l.move(l.tcp() + 0.08 * up, t=1.0, label=f"{name}: left off the rim")
    yield from r.move(r.at() + 0.08 * up, t=1.5, label=f"{name}: lift")
    yield from r.wait(0.5)
    st = ctx.state(name)
    print(f"[bus] {name}: lifted, lowest {1000 * (lowest(st.pos, st.rotation()) - TOP):.0f} mm over the table, tilt {plate_tilt(ctx, name):.1f} deg", flush=True)


def lay_plate(ctx, name: str, k: int, phi: float | None = None, over: float = 0.0, let_go: bool = True, back: float = 0.06, lay: float | None = None, at=None):
    """Carry the held plate over the tub's wall, lower it till its far edge is on the floor (or on the first plate), lay it nearly flat about that edge, let go.
    Lazy routes: `over` lets go this high over the floor without laying it down; `let_go=False` keeps hold; a small `back` leaves the hand by the plate; `lay` is the tilt it is let go at and `at` the spot in the tub (flop, swing)."""
    at = TUB_SPOT if at is None else at
    lay = LAY_TILT if lay is None else lay
    r = ctx.right
    up = np.array([0.0, 0.0, 1.0])
    tub = ctx.state("tub")
    spot = tub.to_world(np.array([[at[0], at[1], TUB_FLOOR]]))[0]
    st = ctx.state(name)
    rim_z = TOP + TUB_H
    lift = rim_z + CARRY_LOW - lowest(st.pos, st.rotation())
    yield from r.move(r.at() + max(0.0, lift) * up, t=1.5, label=f"{name}: up over the tub's wall")
    st = ctx.state(name)
    turn = _rot((0, 0, 1), PHI_TUB - (PHI[name] if phi is None else phi))
    off = turn @ (np.asarray(st.pos, float) - r.at())
    goal = np.array([spot[0] - off[0], spot[1] - off[1], r.at()[2]])
    yield from r.move(goal, grip=d._as_grip(turn @ r.grip.rot), t=3.5, label=f"{name}: turn it over the tub")
    yield from r.wait(0.3)
    st = ctx.state(name)
    shift = np.array([spot[0] - st.pos[0], spot[1] - st.pos[1], 0.0])
    floor = spot[2] + 0.004 + 0.016 * k + over  # the far edge comes to rest here: the tub's floor, or the first plate's rim
    drop = lowest(st.pos, st.rotation()) - floor
    goal = r.at() + shift - drop * up
    yield from r.move(goal, t=3.0, label=f"{name}: down into the tub")
    yield from r.servo(goal, tol=0.002, timeout=1.0, label=f"{name}: down into the tub")
    for _ in range(2):  # the held plate sags in the pinch as it goes: read its lowest point and lower the rest of the way
        st = ctx.state(name)
        drop = lowest(st.pos, st.rotation()) - floor
        if drop < 0.004:
            break
        goal = r.at() - drop * up
        yield from r.move(goal, t=1.0, label=f"{name}: far edge down")
        yield from r.servo(goal, tol=0.002, timeout=1.0, label=f"{name}: far edge down")
    if over:
        yield from r.move(r.at(), finger=d.FINGER_OPEN, t=0.5, label=f"{name}: let go {over * 100:.0f} cm up")
        yield from r.wait(1.0)
        yield from r.move(r.at() - 0.06 * r.approach_axis() + 0.15 * up, t=1.5, label=f"{name}: up")
        return
    st = ctx.state(name)
    pts = world(P_POINTS, st.pos, st.rotation())
    edge = pts[int(np.argmin(pts[:, 2]))]
    a0 = np.array([math.cos(math.radians(PHI_TUB)), math.sin(math.radians(PHI_TUB)), 0.0])
    tilt = plate_tilt(ctx, name)
    yield from r.arc(edge, np.cross(up, a0), -(tilt - lay), t=2.5, label=f"{name}: lay it down about its far foot ring")
    tilt = plate_tilt(ctx, name)
    if tilt > lay + 1.0:  # the plate turns in the pinch as it goes down: once more for the rest
        yield from r.arc(edge, np.cross(up, a0), -(tilt - lay), t=1.5, label=f"{name}: lay it down the rest of the way")
    st = ctx.state(name)
    print(f"[bus] {name}: let go at tilt {plate_tilt(ctx, name):.1f} deg, lowest {1000 * (lowest(st.pos, st.rotation()) - floor):.0f} mm over its rest", flush=True)
    if not let_go:
        return
    yield from r.move(r.at(), finger=-GAP / 2, t=0.5, label=f"{name}: let go")
    yield from r.move(r.at() - SINK * up, t=1.0, label=f"{name}: near rim down on the open jaw")
    yield from r.move(r.at() - back * r.approach_axis(), t=1.0, label=f"{name}: back off")
    if back >= 0.06:
        yield from r.move(r.at() + 0.15 * up, t=1.0, label=f"{name}: up")


EDGE_AT = np.array([0.02, -0.12])  # one hand alone drags the plate's centre here: its near rim then overhangs the table's near edge (x = -0.05) by 5 cm


def one_hand_take(ctx, name: str):
    """Lazy route: the right hand alone presses the plate's well, drags the plate till its near rim overhangs the table's near edge, and pinches the overhang."""
    r = ctx.right
    up = np.array([0.0, 0.0, 1.0])
    c = ctx.pos(name)
    yield from r.move(c + 0.06 * up, grip=d.down(0.0), finger=d.FINGER_CLOSED, t=3.0, label=f"{name}: one hand over the well")
    yield from r.move(c + 0.015 * up, t=1.0, label=f"{name}: onto the well")
    res = yield from r.press(-up, force=20.0, max_travel=0.03, label=f"{name}: press the well")
    yield from r.move(np.array([EDGE_AT[0], EDGE_AT[1], r.at()[2]]), t=6.0, label=f"{name}: drag it to the near edge")
    print(f"[bus] {name}: pressed {res['force_est']:.0f} N, dragged to {np.round(ctx.pos(name)[:2], 3).tolist()}", flush=True)
    yield from r.move(r.at() + 0.06 * up, t=1.0, label=f"{name}: off the well")
    a0 = np.array([1.0, 0.0, 0.0])
    p = ctx.pos(name) - PIN_R * a0 + [0.0, 0.0, 0.5 * (_under(PIN_R) + _over(PIN_R))]
    g = rim_grip(0.0, 0.0, up)
    yield from r.move(p - BACK * a0 + 0.04 * up, grip=g, finger=-GAP / 2, t=3.0, label=f"{name}: beside the overhang")
    yield from r.move(p - BACK * a0, t=1.0, label=f"{name}: level with the overhang")
    yield from r.move(p, t=1.5, label=f"{name}: fingers round the overhang")
    yield from r.close(0.0, SQUEEZE, t=0.8, label=f"{name}: pinch the overhang")
    yield from r.move(r.at() + 0.08 * up, t=1.5, label=f"{name}: lift")
    yield from r.wait(0.5)
    st = ctx.state(name)
    print(f"[bus] {name}: one hand lifted it, lowest {1000 * (lowest(st.pos, st.rotation()) - TOP):.0f} mm over the table", flush=True)


MODES = ("full", "none", "drop", "hold", "hover", "one_hand", "flop", "swing")


def demo_script(ctx):
    """Modes: full; lazy routes: none, the arms wait; drop, each plate let go 10 cm over the tub's floor; hold, the second plate laid down but held;
    flop, each plate let go at 13 deg from flat, its far edge on the floor; swing, let go at 30 deg; hover, the second plate let go with the jaws backed out only 4.5 cm; one_hand, the right hand alone drags the first plate to the table's near edge and takes it by the overhang."""
    if ctx.mode not in MODES:
        raise ValueError(f"unknown mode {ctx.mode!r}")
    l, r = ctx.left, ctx.right
    for _ in range(5):
        yield
    start = {l: l.q.copy(), r: r.q.copy()}
    if ctx.mode == "none":
        yield from l.wait(3.0)
        return
    if ctx.mode == "one_hand":
        yield from one_hand_take(ctx, "plate_r")
        yield from lay_plate(ctx, "plate_r", 0, phi=0.0)
        yield from to_joints(r, start[r], label="right clear")
        yield from l.wait(2.0)
        return
    for k, name in enumerate(("plate_r", "plate_l")):
        if k:  # the same arm posture for each reach
            yield from to_joints(r, start[r], label="right back to its start")
        yield from take_plate(ctx, name)
        last = k == 1
        yield from lay_plate(ctx, name, k, over=0.10 if ctx.mode == "drop" else 0.0, let_go=not (last and ctx.mode == "hold"), back=0.045 if last and ctx.mode == "hover" else 0.06,
                       lay={"flop": 13.0, "swing": 30.0}.get(ctx.mode), at=OPEN_SPOT if ctx.mode in ("flop", "swing") else None)
    if ctx.mode in ("hold", "hover"):
        yield from to_joints(l, start[l], label="left clear")
    else:
        yield from ctx.together(to_joints(l, start[l], label="left clear"), to_joints(r, start[r], label="right clear"))
    yield from l.wait(2.0)
