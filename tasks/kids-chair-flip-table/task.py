"""Put a preschool chair up on the table: Flash Furniture Whitney YU-YCX-003 stackable school chair and a Community Playthings
Woodcrest rectangular table C241 on its 12 in legs.

Chair (Flash Furniture 2-pack listings 2-YU-YCX-003-*: webstaurantstore.com 354CX003GRN, Flash's Salsify product page
2-YU-YCX-003-RED-GG, quill.com 2YUYCX003BLUE): one moulded polypropylene shell with four splayed plastic legs, no metal;
12.5 in W x 14 in D x 20 in H overall, seat 10 x 10 in, seat height 10.75 in, back 11 in wide rising from the seat's rear edge.
NOT CONFIRMED: mass per chair (the 2-pack pages list "weight 6" with no unit and a 10 lb shipping weight; 1.4 kg used),
shell thickness (12 mm assumed), leg section (25 mm assumed).
Table (communityplaythings.com C241 page, dimension image c241_a881, product guide KX91): solid maple top 30 x 22 in,
1 1/8 in thick, heights 12/14/16/18/20 in (12 in = leg A881), no apron (from the maker's photos). NOT CONFIRMED: leg section
(45 mm assumed), table mass (fixed here).

The robot bench stands for the classroom floor; the table stands on it, its long side across the arms. The chair starts
upright on the floor in front of the table, facing the arms. Both hands pinch the seat at its two side edges, lift, turn it
over through 180 degrees about the line between the hands, and lower it seat down onto the tabletop with its back hanging
over the table's near edge, as classroom chairs are put up.
Debug:  python demokit.py task.py OUT --no-record --size 640x360 --stills 10 --mode full   (checks: --mode drop,none)
"""
from __future__ import annotations

import json
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
INSTRUCTION = (
    "Put the child's chair up on the table: take the chair standing on the floor by the two side edges of its seat, lift "
    "it, turn it over and set it down seat down with its legs up on the tabletop, the seat's back edge on the table's near "
    "edge line or up to 3 cm past it, so that the chair's back hangs over the edge, without a leg touching the top while "
    'you carry it and without dropping it. Then let go and move both hands away from the chair.')
SUCCESS_HOLD_S = 0.0
SCORE_OVER = "end"
TOP = d.TABLE_TOP
IN2M = 0.0254

# chair: Whitney YU-YCX-003
SEAT_Z, SEAT_T, SEAT_X, SEAT_Y = 10.75 * IN2M, 0.012, 10 * IN2M, 10 * IN2M
BACK_W, BACK_TOP, BACK_T = 11 * IN2M, 20 * IN2M, 0.012
FOOT_X, FOOT_Y, LEG_R = 14 * IN2M / 2 - 0.0125, 12.5 * IN2M / 2 - 0.0125, 0.0125
CHAIR_MASS = 1.4
# table: Woodcrest C241, 12 in
TABLE_X, TABLE_Y, TABLE_H, TOP_T, LEG = 22 * IN2M, 30 * IN2M, 12 * IN2M, 1.125 * IN2M, 0.045

X0 = float(os.environ.get("X0", "0.10"))                 # chair origin (floor point under the seat centre) at the start
TX = X0 + 7 * IN2M + 0.02                                 # the table's near edge
D_TURN = int(os.environ.get("DIR", "1"))                  # +1: the chair's top goes away from the arms
ZR, ZHI, XT = (float(os.environ.get(k, v)) for k, v in (("ZR", "0.45"), ("ZHI", "0.48"), ("XT", "0.05")))
GX = float(os.environ.get("GX", "0.051"))                # grip axis behind the seat centre (toward the back): through the centre of mass
ZL = float(os.environ.get("ZL", "0.012"))                # the seat stops this far over the top, on the lower fingers, then drops when they open
GRIP_FORCE = float(os.environ.get("FORCE", "35"))
IN = 0.018                                                # TCP in from the seat's side edge
BACKOFF = float(os.environ.get("BACKOFF", "0.11"))              # the hands' move out along y after letting go
GAP = float(os.environ.get("GAP", "0.016"))               # landed: the back's outer face to the table's near edge

LOOKS = {"chair": m.Look(m.rgb(0.10, 0.35, 0.75), roughness=0.45), "wood": m.Look(m.rgb(0.85, 0.70, 0.50), roughness=0.55)}


def _table():
    parts, cols = [], []
    tc = (TABLE_X / 2, 0.0, TABLE_H - TOP_T / 2)
    parts.append(("top", m.box((TABLE_X, TABLE_Y, TOP_T), tc), "wood"))
    cols.append(m.col_box("top_c", (TABLE_X, TABLE_Y, TOP_T), tc))
    for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        c = (TABLE_X / 2 + sx * (TABLE_X / 2 - 0.05), sy * (TABLE_Y / 2 - 0.05), (TABLE_H - TOP_T) / 2)
        parts.append((f"leg{i}", m.box((LEG, LEG, TABLE_H - TOP_T), c), "wood"))
        cols.append(m.col_box(f"leg{i}_c", (LEG, LEG, TABLE_H - TOP_T), c))
    return m.fixed("woodcrest_table", (TX, 0.0, TOP), parts, cols, LOOKS, friction=(0.5, 0.4))


def _chair():
    """Chair frame: origin on the floor under the seat centre, front at +x (turned 180 deg so it faces the arms)."""
    parts, cols = [], []
    sc = (0.0, 0.0, SEAT_Z - SEAT_T / 2)
    parts.append(("seat", m.box((SEAT_X, SEAT_Y, SEAT_T), sc), "chair"))
    cols.append(m.col_box("seat_c", (SEAT_X, SEAT_Y, SEAT_T), sc))
    bc = (-SEAT_X / 2 - BACK_T / 2, 0.0, (SEAT_Z - SEAT_T + BACK_TOP) / 2)
    parts.append(("back", m.box((BACK_T, BACK_W, BACK_TOP - SEAT_Z + SEAT_T), bc), "chair"))
    cols.append(m.col_box("back_c", (BACK_T, BACK_W, BACK_TOP - SEAT_Z + SEAT_T), bc))
    for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        top = (sx * (SEAT_X / 2 - 0.02), sy * (SEAT_Y / 2 - 0.02), SEAT_Z - SEAT_T)
        foot = (sx * FOOT_X, sy * FOOT_Y, LEG_R)
        leg = m.tube([foot, top], LEG_R, sides=10)
        parts.append((f"leg{i}", leg, "chair"))
        cols.append(m.col_hull(f"leg{i}_c", leg))
    return m.free("chair", (X0, 0.0, TOP + 0.002), parts, cols, LOOKS, mass=CHAIR_MASS, friction=(0.45, 0.35), yaw_deg=180.0)


OBJECTS = [_table(), _chair()]


# -- poses ----------------------------------------------------------------------------------------


def _R(q):
    from yam_arena.urdf import quat_xyzw_to_matrix
    return quat_xyzw_to_matrix(np.asarray(q, float))


def chair_pose(objects):
    s = objects["chair"]
    return np.asarray(s.pos, float), _R(s.quat_xyzw)


def landed(objects) -> dict:
    """Seat-down check from the chair's pose: the seat's up axis against world down, legs up, back over the near edge."""
    p, R = chair_pose(objects)
    up = R[:, 2]                                  # the chair's own up (seat top normal when upright)
    tilt = math.degrees(math.acos(float(np.clip(-up[2], -1, 1))))  # 0 when the seat faces straight down
    s = objects["chair"]
    low, high = np.asarray(s.low), np.asarray(s.high)
    pitch = math.degrees(math.atan2(up[0], -up[2]))     # about y: + when the seat's far side is low
    roll = math.degrees(math.atan2(up[1], -up[2]))      # about x
    return {"seat_down_tilt_deg": round(tilt, 1), "pitch_deg": round(pitch, 1), "roll_deg": round(roll, 1), "pos": [round(float(v), 3) for v in p], "legs_up": bool(up[2] < -0.95), "low_z_over_table": round(float(low[2] - TOP), 3),
            "yaw_deg": round(math.degrees(math.atan2(R[1, 0], R[0, 0])), 1)}


# -- scorer ---------------------------------------------------------------------------------------------------------------------------------------

SURF = TOP + TABLE_H                      # the tabletop's face
TOL_DOWN = 5.0                            # seat-down: the seat face within 5 degrees of straight down
EDGE_TOL = 0.03                           # the seat's rear edge on the table's near edge line or up to 3 cm past it
CLEAR = 0.03                              # every fingertip this far from the chair
LIFT_MIN = 0.08                           # lifted: the chair's lowest point this far off the floor
AIR = 0.015                               # a part of the chair within this of the floor or the top counts as resting on it
DROP_SPEED = 1.2                          # let go, in the air and falling faster than this (m/s) is dropped
DROP_GAP = 0.08                           # let go with the seat's face more than this over what it lands on (the top, or the floor) is dropped
SLIDE_MAX = 0.03                          # the seat may slide this far along the top after touching it
HOLD_STEPS = 40                           # 2 s at 20 Hz
SETTLE_STEPS = 20
TRACE = os.environ.get("CHAIR_TRACE", "0") != "0"
SEAT_GRIP = gk.Grip(half=SEAT_T / 2, across="z", centre=(0.0, 0.0, SEAT_Z - SEAT_T / 2), lo=(-0.14, -0.14, -math.inf), hi=(0.14, 0.14, math.inf))
# key points of the chair, in its frame: the seat's face corners, the four feet, the back's two top corners
SEAT_FACE = np.array([[sx * SEAT_X / 2, sy * SEAT_Y / 2, SEAT_Z] for sx in (-1, 1) for sy in (-1, 1)])
FEET = np.array([[sx * FOOT_X, sy * FOOT_Y, LEG_R] for sx in (-1, 1) for sy in (-1, 1)])
BACK_TIPS = np.array([[-SEAT_X / 2 - BACK_T / 2, sy * BACK_W / 2, BACK_TOP] for sy in (-1, 1)])
KEY = np.vstack([SEAT_FACE, SEAT_FACE - [0, 0, SEAT_T], FEET, BACK_TIPS])
REAR_MID = np.array([-SEAT_X / 2, 0.0, SEAT_Z])


def _over_top(p) -> np.ndarray:
    p = np.atleast_2d(p)
    return (p[:, 0] >= TX) & (p[:, 0] <= TX + TABLE_X) & (np.abs(p[:, 1]) <= TABLE_Y / 2)


def seat_down_deg(chair) -> float:
    """How far the seat's face is from straight down, in degrees (0: seat flat, legs up; 180: upright)."""
    return math.degrees(math.acos(float(np.clip(-chair.rotation()[2, 2], -1.0, 1.0))))


SEAT_GRID = np.array([[x, y, SEAT_Z] for x in np.linspace(-SEAT_X / 2, SEAT_X / 2, 9) for y in np.linspace(-SEAT_Y / 2, SEAT_Y / 2, 9)])


def resting(chair) -> bool:
    """Some part of the chair is within AIR of the floor, or the seat's face is within AIR of the top (over it, or at its edge)."""
    w = chair.to_world(KEY)
    sup = np.where(_over_top(w) & (w[:, 2] > SURF - 0.05), SURF, TOP)
    if bool(np.any(w[:, 2] - sup < AIR)):
        return True
    g = chair.to_world(SEAT_GRID)
    near = (g[:, 0] >= TX - 0.005) & (g[:, 0] <= TX + TABLE_X) & (np.abs(g[:, 1]) <= TABLE_Y / 2)
    return bool(np.any(near & (g[:, 2] - SURF < AIR)))


def on_top(chair) -> bool:
    """Seat down on the tabletop: seat's face within TOL_DOWN of straight down, its centre over the top, the corners over the top within a few mm of the face."""
    if seat_down_deg(chair) > TOL_DOWN or not bool(_over_top(chair.pos)[0]):
        return False
    w = chair.to_world(SEAT_FACE)
    over = w[_over_top(w)]
    return bool(len(over) >= 2 and np.all((over[:, 2] - SURF > -0.004) & (over[:, 2] - SURF < 0.012)))


def edge_gap(chair) -> float:
    """The seat's rear edge (where the back rises) from the table's near edge line, along x: negative past the edge."""
    return float(chair.to_world(REAR_MID)[0, 0] - TX)


def chair_lines(chair):
    """World points along the chair's seat edges and middle lines, its back's outline and its legs, for the clearance gate."""
    a = lambda p, q, n=7: gk.along(chair, p, q, n)
    sx, sy, z, bz = SEAT_X / 2, SEAT_Y / 2, SEAT_Z, SEAT_Z - SEAT_T
    out = [a((sx, sy, z), (sx, -sy, z)), a((-sx, sy, z), (-sx, -sy, z)), a((sx, sy, z), (-sx, sy, z)), a((sx, -sy, z), (-sx, -sy, z)),
           a((sx, 0, z), (-sx, 0, z)), a((0, sy, z), (0, -sy, z)),
           a((-sx - BACK_T / 2, -BACK_W / 2, BACK_TOP), (-sx - BACK_T / 2, BACK_W / 2, BACK_TOP), 9),
           a((-sx - BACK_T / 2, -BACK_W / 2, z), (-sx - BACK_T / 2, -BACK_W / 2, BACK_TOP)), a((-sx - BACK_T / 2, BACK_W / 2, z), (-sx - BACK_T / 2, BACK_W / 2, BACK_TOP))]
    for f, t in zip(FEET, [(px * (sx - 0.02), py * (sy - 0.02), bz) for px in (-1, 1) for py in (-1, 1)]):
        out.append(a(f, t))
    return out


LADDER = gk.Ladder(("lifted", 0.25), ("turned", 0.5), ("on_top", 0.75), ("placed", 1.0))


class Trial:
    """The trial's record: lifted, turned, on the top, placed; and the latched failures."""

    def __init__(self):
        self.dropped, self.scraped, self.dragged = (gk.Latch(w) for w in ("chair dropped", "leg scraped the top", "chair dragged along the top"))
        self.settle, self.hold = gk.EndHold(SETTLE_STEPS), gk.EndHold(HOLD_STEPS)
        self.reset()

    def reset(self, env=None):
        self.n = 0
        self.was_held = self.lifted = self.turned = self.prev_hands = False
        self.touch0 = None
        self.last = {}
        for latch in (self.dropped, self.scraped, self.dragged):
            latch.reset()
        self.settle.reset()
        self.hold.reset()

    def step(self, env):
        self.n += 1
        chair = m.state(env, "chair")
        tips = m.fingertips(env)
        hands = gk.holding(tips, chair, SEAT_GRIP, resting=False)
        both = len(hands) == 2
        self.was_held |= bool(hands)
        w = chair.to_world(KEY)
        down = seat_down_deg(chair)
        # lifted: both hands hold it, its lowest point well off the floor; turned: both hands hold it, seat facing down within 30 degrees
        if both and float(w[:, 2].min()) - TOP >= LIFT_MIN:
            self.lifted = True
        if self.lifted and both and down <= 30.0:
            self.turned = True
        # dropped: it was held, now no hand holds it, no part of it is on the floor or the top, and it falls faster than DROP_SPEED
        if self.was_held and not hands and self.prev_hands:
            g = chair.to_world(np.vstack([SEAT_GRID, KEY]))
            sup = np.where((g[:, 0] >= TX - 0.005) & (g[:, 0] <= TX + TABLE_X) & (np.abs(g[:, 1]) <= TABLE_Y / 2) & (g[:, 2] > SURF - 0.05), SURF, TOP)
            self.dropped.trip(float((g[:, 2] - sup).min()) > DROP_GAP)
        self.prev_hands = bool(hands)
        self.dropped.trip(self.was_held and not hands and not resting(chair) and -float(chair.lin_vel[2]) > DROP_SPEED)
        # a leg's foot or the back's top corners on the tabletop's face while the chair is held or moving, or the seat dragged along the top
        feet, tips_ = chair.to_world(FEET), chair.to_world(BACK_TIPS)
        moving = float(np.linalg.norm(chair.lin_vel)) > 0.02
        touch = (_over_top(feet) & (feet[:, 2] < SURF + LEG_R + 0.004) & (feet[:, 2] > SURF - TOP_T)).any() or (_over_top(tips_) & (tips_[:, 2] < SURF + 0.004) & (tips_[:, 2] > SURF - TOP_T)).any()
        self.scraped.trip(bool(touch and (hands or moving)))
        seat = chair.to_world(SEAT_FACE)
        centre = chair.to_world(np.array([[0.0, 0.0, SEAT_Z]]))[0]
        if np.any(_over_top(seat) & (seat[:, 2] < SURF + 0.004)) and self.touch0 is None:
            self.touch0 = centre[:2].copy()
        if self.touch0 is not None:
            self.dragged.trip(float(np.linalg.norm(centre[:2] - self.touch0)) > SLIDE_MAX)
        still = float(np.linalg.norm(chair.lin_vel)) < 0.01 and float(np.linalg.norm(chair.ang_vel)) < 0.1
        landed = self.turned and on_top(chair) and not hands and still
        clear = gk.hands_clear(tips, chair_lines(chair), CLEAR)
        edge = -EDGE_TOL <= edge_gap(chair) <= 0.0
        self.settle.step(landed)
        self.hold.step(landed and edge and clear)
        self.last = {"lifted": self.lifted, "turned": self.turned, "on_top": self.settle.done, "placed": self.hold.done}
        if TRACE and self.n % 10 == 0:
            print(f"[chair] step {self.n}: held by {hands}, low {1000 * (float(w[:, 2].min()) - TOP):.0f} mm, seat {down:.1f} deg from down, edge gap {1000 * edge_gap(chair):.0f} mm, still {still}, clear {clear}, "
                  f"nearest fingertip {1000 * gk.min_distance(tips, chair_lines(chair)):.0f} mm, scraped {bool(self.scraped)}, dragged {bool(self.dragged)}, settle {self.settle.count}, hold {self.hold.count}; score {self.score()}", flush=True)

    def score(self) -> float:
        failed = any(bool(x) for x in (self.dropped, self.scraped, self.dragged))
        return LADDER.score(self.last, failed=failed) if self.last else 0.0


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.score() >= 1.0


def score(objects) -> float:
    return TRIAL.score()


# -- demo -----------------------------------------------------------------------------------------


def _plan_joints():
    import sweep2 as sw
    sw.SZ, sw.ST, sw.HW, sw.HD, sw.BW, sw.BT, sw.FX, sw.TABLE_H, sw.IN, sw.GAP = SEAT_Z, SEAT_T, SEAT_X / 2, SEAT_X / 2, BACK_W / 2, BACK_TOP, 7 * IN2M, TABLE_H, IN, GAP
    wps, xe = sw.plan(X0, D_TURN, ZR, ZHI, XT, GX)
    wps = wps[:-8] + [(np.array([a[0], 0.0, max(a[2], TABLE_H + SEAT_T / 2 + ZL)]), th) for a, th in wps[-8:]]
    out = {}
    for side, sgn, jaw in (("left", 1.0, -float(D_TURN)), ("right", -1.0, float(D_TURN))):
        res = sw.solve_arm(side, sgn, wps, 0.0, jaw)
        out[side] = (np.radians(np.array(res[3])), res[0], res[1], res[2], sw.hand_rot(sgn, 0.0, jaw), sgn)
    return wps, out


def _jgo(arm, q_goal, t, finger=None):
    q0 = arm.q.copy()
    f = arm.finger if finger is None else finger
    n = max(1, round(t * d.CONTROL_HZ))
    for k in range(1, n + 1):
        s = k / n
        s = s * s * (3 - 2 * s)
        arm.q = q0 + (np.asarray(q_goal) - q0) * s
        arm.cmd = np.r_[arm.q, float(np.clip(f / d.FINGER_OPEN, 0.0, 1.0))]
        yield
    arm.finger = f


def _jpath(arm, qs, t):
    """Through joint waypoints qs (rows), t seconds in all, straight in joint space between them."""
    per = t / max(1, len(qs))
    for q in qs:
        yield from _jgo(arm, q, per)


def _ik(arm, pos, rot, seed):
    from yam_arena import ik
    q, pe, re_ = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.asarray(seed, float), base_pos=arm.base, iters=120)
    return q, pe * 1000, math.degrees(re_)


def demo_script(ctx):
    from yam_arena import ik
    global ZL
    if ctx.mode == "drop":                      # the lazy route: let go 17 cm over the top and let the chair fall onto it
        ZL = 0.17
    if ctx.mode == "none":                      # the arms wait
        yield from ctx.together(ctx.left.wait(15.0), ctx.right.wait(15.0))
        return
    wps, plan = _plan_joints()
    log = {"plan": {s: {"miss_mm": round(v[1], 1), "deg": round(v[2], 1), "max_step_deg": round(v[3], 1)} for s, v in plan.items()}}
    print("[chair] plan", log["plan"], flush=True)
    A, L = ctx.left, ctx.right
    arms = {"left": ctx.left, "right": ctx.right}
    # grasp poses and a stand-off 6 cm out along each approach, then 12 cm above that
    pre = {}
    for side, (qs, *_r, G, sgn) in plan.items():
        arm = arms[side]
        p_grasp, _ = ik.tcp_pose(qs[0], arm.base)
        out = p_grasp - 0.06 * G[:, 2]
        q_out, mm1, _ = _ik(arm, out, G, qs[0])
        q_hi, mm2, _ = _ik(arm, out + [0, 0, 0.12], G, q_out)
        pre[side] = (q_hi, q_out, qs, p_grasp)
        print(f"[chair] {side} stand-off miss {mm1:.1f} / {mm2:.1f} mm", flush=True)
    yield from ctx.together(*(_jgo(arms[s], pre[s][0], 3.0, d.FINGER_OPEN) for s in arms))
    yield from ctx.together(*(_jgo(arms[s], pre[s][1], 1.5) for s in arms))
    yield from ctx.together(*(_jgo(arms[s], pre[s][2][0], 1.5) for s in arms))
    width = SEAT_T
    f_close = float(np.clip(-(width / 2 - GRIP_FORCE / d.GRIPPER["stiffness"]), d.FINGER_OPEN, d.FINGER_CLOSED))
    yield from ctx.together(*(_jgo(arms[s], arms[s].q, 0.8, f_close) for s in arms))
    yield from ctx.together(*(arms[s].wait(0.5) for s in arms))

    # grip-in-chair frame at the close, for slip
    p0, R0 = chair_pose(ctx.states())
    hand_R = lambda s: ik.tcp_pose(ctx.observed(s)[:6], arms[s].base)[1]
    held = {s: R0.T @ hand_R(s) for s in arms}            # each hand's rotation in the chair's frame at the close
    track = {"slip_deg": 0.0, "tilt_off_axis_deg": 0.0, "samples": []}

    def watch():
        p, R = chair_pose(ctx.states())
        for s in arms:   # turn of the chair inside the grip (about the grip axis mostly): slip
            dR = (R.T @ hand_R(s)) @ held[s].T
            track["slip_deg"] = max(track["slip_deg"], math.degrees(math.acos(float(np.clip((np.trace(dR) - 1) / 2, -1, 1)))))
        Rr = R @ R0.T                                  # turn since the grasp; planned: about world y only
        off = math.degrees(math.asin(float(np.clip(abs(Rr[1, 0]) + abs(Rr[1, 2]), 0, 1))))
        track["tilt_off_axis_deg"] = max(track["tilt_off_axis_deg"], off)

    def watched(gen):
        for _ in gen:
            watch()
            yield

    n0, n1, n2 = 7, 7 + 24, 7 + 24 + 8
    segs = [(1, n0, 2.0, "lift"), (n0, n1, 5.0, "turn"), (n1, n2, 1.6, "slide"), (n2, len(wps), 1.8, "lower")]
    for a, b, t, lab in segs:
        yield from watched(ctx.together(*(_jpath(arms[s], plan[s][0][a:b], t) for s in arms)))
        track["samples"].append({"after": lab, "slip_deg": round(track["slip_deg"], 1), "tilt_off_axis_deg": round(track["tilt_off_axis_deg"], 1), **landed(ctx.states())})
        print(f"[chair] after {lab}: {track['samples'][-1]}", flush=True)
    yield from ctx.together(*(arms[s].wait(0.5) for s in arms))
    yield from ctx.together(*(_jgo(arms[s], arms[s].q, 0.8, d.FINGER_OPEN) for s in arms))
    backs = {}
    for s, (qs, *_r, G, sgn) in plan.items():
        p, _ = ik.tcp_pose(arms[s].q, arms[s].base)
        backs[s] = _ik(arms[s], p + np.array([0.0, sgn * BACKOFF, 0.0]), ik.tcp_pose(arms[s].q, arms[s].base)[1], arms[s].q)[0]
    yield from ctx.together(*(_jgo(arms[s], backs[s], 1.2) for s in arms))
    yield from ctx.together(*(arms[s].wait(3.0) for s in arms))
    final = landed(ctx.states())
    p, R = chair_pose(ctx.states())
    final["seat_x_from_edge"] = round(float(p[0] - TX), 3)
    log.update({"slip_deg": round(track["slip_deg"], 1), "tilt_off_axis_deg": round(track["tilt_off_axis_deg"], 1), "samples": track["samples"], "final_after_release_3s": final})
    print("[chair] LOG " + json.dumps(log), flush=True)
