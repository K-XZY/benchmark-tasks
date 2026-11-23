"""Fit a PS5 Slim to its vertical stand and stand it upright: one hand holds the stand square while the other turns its mounting screw snug, then both hands roll the 2.6 kg console a quarter turn up onto the stand.

The console (358 x 80 x 216 mm, 2.6 kg) lies flat on a soft cloth on a media unit, its rear edge toward the arms and 60 mm proud of the unit's front, its stand end 5 mm past the unit's side.
The stand (a 190 mm chrome ring under a 216 x 80 mm plate, 42 mm tall) sits on the console's end face on its started mounting screw. One mechanism, free on the unit:
the console body, the stand on a hinge about the screw's axis, and the screw on a hinge about the same axis. Until the screw is snug, the thread's drag turns the stand with the
screw (HEAD_SHARE of the screw's torque); turning the screw takes DRAG_NM at any speed, rising to SEAT_NM over the last quarter turn; then the screw seats and both lock. Scored in the step hook; `score()` returns what the hook holds.

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
import gatekit as gk  # noqa: E402
import mechkit as m  # noqa: E402

SCENE = "station"
INSTRUCTION = "Fit the stand to the PS5: hold it square while you turn its screw snug by hand, then roll the console upright onto it and let go."
SUCCESS_HOLD_S = 0.0  # the hook counts its own 3 s hold, so score() and success() agree
SCORE_OVER = "end"
TOP = d.TABLE_TOP

# -- real sizes ----------------------------------------------------------------------------
CX, CY, CZ = 0.216, 0.358, 0.080  # console depth (x), width (y), height (z) lying flat: Sony CFI-2000 digital, 358 x 80 x 216 mm
MASS = 2.6  # kg, Sony
UNIT_H = 0.160  # media-unit top over the table
CLOTH = 0.002
_MU = os.environ.get("PS5_MU")  # sweep of the cloth's static friction, dynamic 0.1 less
MU_CLOTH = (float(_MU), float(_MU) - 0.1) if _MU else (0.9, 0.8)  # the cloth on the unit top (estimate); PhysX averages it with the console's: 0.75 static
MU_CONSOLE = (0.6, 0.5)  # the console's shell, stand and screw head, one material (estimate); a tilt test reads the console on the cloth
UNIT_TOP = TOP + UNIT_H + CLOTH
END_Y = 0.080  # the console's stand end, in world y
C0 = np.array([0.250 + CX / 2, END_Y - CY / 2, UNIT_TOP + CZ / 2])  # console centre at the start; rear edge at x 0.25
UNIT_X0, UNIT_X1, UNIT_Y0, UNIT_Y1 = 0.310, 0.640, -0.420, END_Y - 0.005

RING_R, RING_W, RING_T = 0.095, float(os.environ.get("PS5_RINGW", "0.004")), float(os.environ.get("PS5_RINGT", "0.010"))  # chrome ring: 190 mm outside, 4 mm wide, 10 mm thick, pinched across the thickness (photo scaling: width 3.5 to 5 mm, demo 1.0 at 4; thickness estimate 6 to 10 mm; Sony: stand 190 x 190 x 42 mm)
PLATE = (0.216, 0.012, 0.080)  # the plate on the console's end face: its x, along the console's y, z
STAND_H = 0.042
GRIP_N = float(os.environ.get("PS5_GRIP", "35"))  # pinch force per hand, N (the YAM limit is about 40)
STAND_M = 0.15  # kg, estimate (100 to 200 g)
HEAD_R, HEAD_L = 0.009, float(os.environ.get("PS5_HEAD", "0.006"))  # screw head 18 mm across (estimate, 15 to 20), 6 mm proud of the plate (estimate, not visible in Sony's photo: 5 to 8; 4 fails, so a real head under 5 is out of range; calipers in a real trial)
TURNS_SNUG = float(os.environ.get("PS5_TURNS", "3.0"))  # turns from started to seated (estimate, 3 to 5; README)
DRAG_NM = float(os.environ.get("PS5_DRAG", "0.15"))  # running torque of the hand screw before it seats (bound 0.05 to 0.2 N m; README); the stand feels HEAD_SHARE of it
SEAT_NM = float(os.environ.get("PS5_SEAT", "0.25"))  # torque to seat it snug, reached over the last quarter turn (bound 0.15 to 0.35 N m; README)
SEAT_TURNS = 0.25
HEAD_SHARE = 0.5  # share of the screw's torque spent as friction under its head, which turns the stand (about half: bolted-joint torque split)
SCREW_K = 20.0  # N m/rad: the screw's holding drive. Its target stays put until the hand's torque passes the cap, then slides so the drive holds exactly the cap: friction that does not depend on speed
SQUARE_DEG = 5.0
UPRIGHT_DEG = 5.0
HOLD_STEPS = 60  # 3 s at 20 Hz
DROP_V = 0.35  # m/s down, of the console or of its ring: a free fall of about 6 mm (v^2 / 2g) counts as a drop
CLEAR_M = 0.03  # every gripper point this far from the console and stand at the end

YE = CY / 2  # end face, console frame
LOOKS = {"white": m.Look(m.rgb(0.94, 0.94, 0.93), roughness=0.35), "black": m.Look(m.rgb(0.05, 0.05, 0.06), roughness=0.3),
         "chrome": m.Look(m.rgb(0.80, 0.80, 0.82), roughness=0.15, metallic=1.0), "plate": m.Look(m.rgb(0.10, 0.11, 0.14), roughness=0.5),
         "screw": m.Look(m.rgb(0.55, 0.55, 0.57), roughness=0.3, metallic=1.0), "cloth": m.Look(m.rgb(0.25, 0.27, 0.32), roughness=0.95),
         "wood": m.Look(m.rgb(0.42, 0.30, 0.20), roughness=0.7)}


def _unit() -> m.Part:
    sx, sy = UNIT_X1 - UNIT_X0, UNIT_Y1 - UNIT_Y0
    c = ((UNIT_X0 + UNIT_X1) / 2, (UNIT_Y0 + UNIT_Y1) / 2)
    parts = [("body", m.box((sx, sy, UNIT_H), (0.0, 0.0, UNIT_H / 2)), "wood"),
             ("cloth", m.box((sx - 0.04, sy - 0.03, CLOTH), (0.0, 0.0, UNIT_H + CLOTH / 2)), "cloth")]
    cols = [m.col_box("body_c", (sx, sy, UNIT_H + CLOTH), (0.0, 0.0, (UNIT_H + CLOTH) / 2))]
    return m.fixed("unit", (c[0], c[1], TOP), parts, cols, LOOKS, friction=MU_CLOTH)


def _ring():
    meshes, cols, n = [], [], 16
    yc = YE + STAND_H - RING_T / 2
    chord = 2 * (RING_R - RING_W / 2) * math.sin(math.pi / n) + 0.004
    for k in range(n):
        a = 360.0 * k / n
        seg = m.box((chord, RING_T, RING_W)).moved((0.0, 0.0, -(RING_R - RING_W / 2))).turned(a, "y").moved((0.0, yc, 0.0))
        meshes.append(seg)
        cols.append(m.col_hull(f"ring{k}_c", seg))
    return m.join(*meshes), cols


def _stand() -> m.Link:
    ring, cols = _ring()
    py = YE + PLATE[1] / 2
    plate = m.box(PLATE, (0.0, py, 0.0))
    posts = [m.box((0.012, STAND_H - PLATE[1] - RING_T, 0.05), (sx * (RING_R - RING_W / 2), YE + PLATE[1] + (STAND_H - PLATE[1] - RING_T) / 2, 0.0)) for sx in (-1, 1)]
    parts = [("ring", ring, "chrome"), ("plate", plate, "plate"), ("posts", m.join(*posts), "plate")]
    cols = cols + [m.col_box("plate_c", PLATE, (0.0, py, 0.0))] + [m.col_hull(f"post{k}_c", p) for k, p in enumerate(posts)]
    return m.Link("stand", "hinge", "Y", (0.0, YE, 0.0), -math.inf, math.inf, tuple(parts), tuple(cols), mass=STAND_M, spring=2.0, damping=0.05,
                  friction=0.0, effort=HEAD_SHARE * DRAG_NM, gravity=True, com=(0.0, YE + 0.025, 0.0), inertia=(1.5e-3, 2.8e-3, 1.5e-3))


def _screw() -> m.Link:
    y0 = YE + PLATE[1]
    head = m.cylinder(HEAD_R, HEAD_L, (0.0, y0 + HEAD_L / 2, 0.0), axis="y", segments=16)
    slot = m.box((0.012, 0.0015, 0.002), (0.0, y0 + HEAD_L + 0.0004, 0.0))
    return m.Link("screw", "hinge", "Y", (0.0, YE, 0.0), -math.inf, math.inf, (("head", head, "screw"), ("slot", slot, "black")), (m.col_hull("head_c", head),),
                  mass=0.01, spring=SCREW_K, damping=0.02, friction=0.0, effort=DRAG_NM, inertia=(1e-6, 1e-6, 1e-6))


def _console() -> m.Part:
    # Sony's look: a black centre body between two white covers whose edges curve in toward the top and bottom faces
    # (drawn as three steps per cover); the collider is the full 358 x 80 x 216 mm box
    band = 0.030
    parts = [("band", m.box((CX, CY, band), (0.0, 0.0, 0.0)), "black")]
    z = band / 2
    for k, (th, ins) in enumerate(((0.012, 0.003), (0.008, 0.008), (0.005, 0.016))):
        for sgn, name in ((1.0, "upper"), (-1.0, "lower")):
            parts.append((f"{name}{k}", m.box((CX - 2 * ins, CY - 2 * ins, th), (0.0, 0.0, sgn * (z + th / 2))), "white"))
        z += th
    cols = [m.col_box("body_c", (CX, CY, CZ))]
    I = (MASS / 12 * (CY ** 2 + CZ ** 2), MASS / 12 * (CX ** 2 + CZ ** 2), MASS / 12 * (CX ** 2 + CY ** 2))
    return m.mechanism("ps5", tuple(C0), [_stand(), _screw()], LOOKS, frame_parts=parts, frame_colliders=cols, free_mass=MASS,
                       friction=MU_CONSOLE, frame_inertia=I)


OBJECTS = [_unit(), _console()]


# -- the step hook: thread drag, seating, and the score ------------------------------------

def gripper_points(env) -> np.ndarray:
    """Points over both whole grippers: fingers (pads, tips, backs) and the housing and palm round the wrist, from the gripper body's pose."""
    base = env.unwrapped
    origin = m._np(base.scene.env_origins[0])
    out = []
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        i = robot.find_bodies("gripper")[0][0]
        pose = m._np(robot.data.body_link_pose_w)[0, i]
        R = m.quat_matrix(pose[3:7])
        opening = abs(float(m._np(robot.data.joint_pos)[0, robot.find_joints("joint7")[0][0]]))
        for sx in (-1.0, 1.0):
            for dy in (-0.018, 0.0, 0.018):
                for dz in (0.147, 0.132, 0.117, 0.10):
                    for off in (0.0045, 0.0135):
                        out.append(pose[:3] - origin + R @ np.array([sx * (opening + off), dy, dz]))
        for x in (-0.04, 0.0, 0.04):  # housing and palm
            for dy in (-0.03, 0.03):
                for dz in (0.03, 0.06, 0.085):
                    out.append(pose[:3] - origin + R @ np.array([x, dy, dz]))
    return np.array(out)


def _box_dist(p, half):
    q = np.abs(p) - half
    return np.linalg.norm(np.maximum(q, 0.0), axis=-1) + np.minimum(np.max(q, axis=-1), 0.0)


def clearance(s, pts) -> float:
    """Least distance from the points to the console body and the stand (plate and ring disc as boxes), in the console's frame."""
    loc = s.to_local(pts)
    dc = _box_dist(loc, np.array([CX, CY, CZ]) / 2)
    qs = s.joint("stand")
    c, sn = math.cos(qs), math.sin(qs)
    rel = loc - np.array([0.0, YE + STAND_H / 2, 0.0])
    xs, zs = c * rel[:, 0] - sn * rel[:, 2], sn * rel[:, 0] + c * rel[:, 2]  # into the stand's frame (about y)
    ds_plate = _box_dist(np.stack([xs, rel[:, 1], zs], 1), np.array([PLATE[0] / 2, STAND_H / 2, PLATE[2] / 2]))
    rr = np.hypot(xs, zs)
    ds_ring = np.hypot(np.maximum(np.abs(rr - (RING_R - RING_W / 2)) - RING_W / 2, 0.0), np.maximum(np.abs(rel[:, 1]) - STAND_H / 2, 0.0))
    return float(min(dc.min(), ds_plate.min(), ds_ring.min()))


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.prev, self.turned, self.target, self.snug, self.square_at_snug = None, 0.0, 0.0, False, None
        self.prev_s, self.stand_turn, self.cap, self.low_prev, self.low_v = None, 0.0, None, None, 0.0
        self.ts, self.torque, self.seat_peak = None, 0.0, 0.0
        self.upright, self.was_up, self.fell, self.steps, self.hold, self.last = False, False, gk.Latch("console fell or tipped"), 0, gk.EndHold(HOLD_STEPS), {}

    def step(self, env):
        self.steps += 1
        s = m.state(env, "ps5")
        q, qs = s.joint("screw"), s.joint("stand")
        if self.prev is None:
            self.prev = q
        dq = math.atan2(math.sin(q - self.prev), math.cos(q - self.prev))  # unwrapped step
        self.prev = q
        self.turned += -dq  # tightening turns the screw about -y in the console's frame: into the console
        if self.prev_s is None:
            self.prev_s = qs
        self.stand_turn += math.atan2(math.sin(qs - self.prev_s), math.cos(qs - self.prev_s))  # the stand's whole turn, unwrapped: a full spin is not square
        self.prev_s = qs
        if not self.snug:
            # the thread: the screw's drive holds where the screw is, so turning it takes DRAG_NM, rising to SEAT_NM over the last quarter turn
            left = TURNS_SNUG - self.turned / (2 * math.pi)
            cap = DRAG_NM + (SEAT_NM - DRAG_NM) * float(np.clip(1.0 - left / SEAT_TURNS, 0.0, 1.0))
            if self.cap is None or abs(cap - self.cap) > 0.005:
                self.cap = cap
                asset, j = m._joint(env, "ps5", "screw")
                asset.write_joint_effort_limit_to_sim(m._tensor(asset, cap), joint_ids=[j])
                asset, j = m._joint(env, "ps5", "stand")
                asset.write_joint_effort_limit_to_sim(m._tensor(asset, HEAD_SHARE * cap), joint_ids=[j])  # the head's friction drags the stand round
            # Coulomb friction: the drive's target stays put while the hand's torque is under the cap, and slides to hold the cap when it is over, at any speed
            if self.ts is None:
                self.ts = q
            err = q - self.ts
            if abs(SCREW_K * err) > cap:
                self.ts = q - math.copysign(cap / SCREW_K, err)
            self.torque = SCREW_K * abs(q - self.ts)  # the torque the screw resists with now
            if left < 0.1:
                self.seat_peak = max(self.seat_peak, self.torque)
            m.set_target(env, "ps5", "screw", self.ts)
            # the thread's drag: the stand's spring target moves with the screw, the drive capped at the head's share of the screw torque; a held stand stays put
            self.target = qs + float(np.clip(self.target + dq - qs, -0.3, 0.3))
            m.set_target(env, "ps5", "stand", self.target)
            if self.turned >= TURNS_SNUG * 2 * math.pi:
                self.snug = True
                self.square_at_snug = abs(math.degrees(self.stand_turn))
                m.lock(env, "ps5", "screw")
                m.lock(env, "ps5", "stand")
        R = s.rotation()
        axis_up = -R[:, 1]  # the console's long axis, from the stand end to the far end
        tilt = math.degrees(math.acos(float(np.clip(axis_up[2], -1.0, 1.0))))
        square = self.square_at_snug is not None and self.square_at_snug <= SQUARE_DEG
        self.upright = bool(self.snug and square and tilt <= 10.0)  # read now, each step
        self.was_up = self.was_up or self.upright  # only the failure below reads this
        stand_low = float(s.to_world(np.array([[0.0, YE + STAND_H, 0.0]]))[0][2])  # the ring's bottom face, on the stand's axis
        on_unit = abs(stand_low - UNIT_TOP) < 0.01
        if self.low_prev is not None:
            self.low_v = (stand_low - self.low_prev) / float(env.unwrapped.step_dt)  # the ring's speed down, where it lands
        self.low_prev = stand_low
        if s.pos[2] < UNIT_TOP - 0.02 or (self.was_up and tilt > 45.0) or float(s.lin_vel[2]) < -DROP_V or self.low_v < -DROP_V:
            self.fell.trip(True)
        clear = clearance(s, gripper_points(env))
        still = float(np.linalg.norm(s.lin_vel)) < 0.02
        self.hold.step(bool(self.upright and tilt <= UPRIGHT_DEG and on_unit and clear > CLEAR_M and still and not self.fell))
        self.last = {"t": round(self.steps * float(env.unwrapped.step_dt), 2), "turns": round(self.turned / (2 * math.pi), 2), "shift_mm": [round(float(v) * 1000, 1) for v in (s.pos[:2] - C0[:2])], "stand_deg": round(math.degrees(self.stand_turn), 1), "cap": None if self.cap is None else round(self.cap, 3), "torque": round(self.torque, 3), "seat_peak": round(self.seat_peak, 3),
                     "snug": self.snug, "square_at_snug": None if self.square_at_snug is None else round(self.square_at_snug, 1), "tilt": round(tilt, 1),
                     "stand_low_mm": round((stand_low - UNIT_TOP) * 1000, 1), "clear_mm": round(clear * 1000, 1), "upright": self.upright, "fell": bool(self.fell),
                     "done": self.hold.count >= HOLD_STEPS, "score": self.score()}
        if self.steps % 10 == 0:
            print("[ps5] " + json.dumps({"mode": os.environ.get("TASK_DEMO", ""), **self.last}), flush=True)

    def score(self) -> float:
        if self.fell:
            return 0.0
        if self.hold.count >= HOLD_STEPS:  # standing free for the last 3 s, now: not a latch
            return 1.0
        if self.upright:
            return 2 / 3
        if self.snug and self.square_at_snug is not None and self.square_at_snug <= SQUARE_DEG:
            return 1 / 3
        return 0.0


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects) -> float:
    return TRIAL.score()


def success(objects) -> bool:
    return TRIAL.score() >= 1.0


# -- the scripted demo ---------------------------------------------------------------------

TIP = 0.0133  # TCP to the fingertips' middle, along the approach


def _vert(g):
    """A level grip turned a quarter about its own axis, so the fingers close up and down."""
    return d._as_grip(g.rot @ d.axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.pi / 2))


def demo_script(ctx):
    mode = os.environ.get("TASK_DEMO", "full")
    s0 = ctx.state("ps5")
    c0 = np.asarray(s0.pos, float)
    head = s0.to_world(np.array([[0.0, YE + PLATE[1] + HEAD_L / 2, 0.0]]))[0]
    ring_near = s0.to_world(np.array([[-(RING_R - RING_W / 2), YE + STAND_H - RING_T / 2, 0.0]]))[0]
    g_screw, g_ring = d.level(-90.0), d.level(0.0)
    app_s, app_r = g_screw.rot[:, 2], g_ring.rot[:, 2]
    # 1. right hand pinches the ring's near side; left hand takes the screw head from outside the stand
    yield from ctx.together(ctx.right.move(ring_near - (TIP + 0.05) * app_r, grip=g_ring, finger=-0.018, t=2.5, label="to the ring"),
                            ctx.left.move(head - (TIP + 0.035) * app_s, grip=g_screw, finger=-0.02, t=2.5, label="to the screw"))
    yield from ctx.together(ctx.right.move(ring_near - (TIP - 0.004) * app_r, t=1.0, label="fingers round the ring"),
                            ctx.left.move(head - (TIP + 0.001) * app_s, t=1.0, label="fingers round the head"))
    if mode == "nohold":  # lazy: the screw turned with no hand on the stand
        yield from ctx.right.move(ring_near - (TIP + 0.08) * app_r, t=0.5, label="hand off the stand")
    else:
        yield from ctx.right.close(RING_T, force=GRIP_N, label="hold the stand square")
    axis = np.array([0.0, -1.0, 0.0])  # tighten: right-hand turn about the screw's way in
    stroke = float(os.environ.get("PS5_STROKE", "120"))
    for k in range(2 * int(math.ceil(TURNS_SNUG * 360.0 / stroke)) + 8):
        if TRIAL.snug or mode == "noscrew" or (mode == "unseated" and TRIAL.turned / (2 * math.pi) > TURNS_SNUG - 0.4):
            break
        slow = mode == "slowseat" and TRIAL.turned / (2 * math.pi) > TURNS_SNUG - 0.3  # lazy check: the last 0.3 turn at 3 deg/s
        st = 30.0 if slow else stroke
        head_now = ctx.state("ps5").to_world(np.array([[0.0, YE + PLATE[1] + HEAD_L / 2, 0.0]]))[0]  # the head where it is now, as seen
        yield from ctx.left.move(head_now - (TIP + 0.001) * app_s, grip=g_screw, t=0.3, label="fingers round the head")
        yield from ctx.left.close(2 * HEAD_R, force=GRIP_N, t=0.4, label="pinch the head")
        yield from ctx.left.turn(st, axis=axis, t=10.0 if slow else 1.0, label="turn the screw")
        yield from ctx.left.move(ctx.left.at(), finger=-0.02, t=0.4, label="let go of the head")
        yield from ctx.left.turn(-st, axis=axis, t=0.7, label="turn back")
        skew = ctx.deg("ps5", "stand")  # the stand's marks against the console's, as seen
        if abs(skew) > 1.0 and not TRIAL.snug and mode != "nohold":
            yield from ctx.right.arc(head, np.array([0.0, 1.0, 0.0]), -skew, t=0.6, label="square the stand")
    print("[ps5] SCREW " + json.dumps(TRIAL.last), flush=True)
    yield from ctx.together(ctx.left.open(), ctx.right.open())
    yield from ctx.together(ctx.left.move(ctx.left.at() + np.array([0.0, 0.06, 0.0]), t=1.0, label="out of the stand"),
                            ctx.right.move(ctx.right.at() - 0.06 * app_r, t=1.0, label="off the ring"))
    yield from ctx.left.move(np.array([0.17, 0.20, UNIT_TOP + 0.10]), grip=_vert(d.level(0.0)), finger=d.FINGER_OPEN, t=1.5, label="round the stand")
    if mode in ("noroll", "slowseat"):  # lazy: stand fitted, console left flat (slowseat checks only the screw)
        yield from ctx.left.wait(4.0)
        print("[ps5] END " + json.dumps(TRIAL.last), flush=True)
        return
    # 2. both hands pinch the console's rear edge across its 80 mm, either side of its middle
    s1 = ctx.state("ps5")
    gv = _vert(d.level(0.0))
    app = gv.rot[:, 2]
    local = {"left": np.array([-CX / 2 + 0.050, 0.06, 0.0]), "right": np.array([-CX / 2 + 0.050, -0.06, 0.0])}
    pts = {a: s1.to_world(local[a][None])[0] for a in local if mode != "onehand" or a == "left"}
    yield from ctx.together(*[getattr(ctx, a).move(pts[a] - (TIP + 0.07) * app + np.array([0, 0, 0.03]), grip=gv, finger=d.FINGER_OPEN, t=2.0, label="behind the rear edge") for a in pts])
    yield from ctx.together(*[getattr(ctx, a).move(pts[a] - (TIP + 0.07) * app, t=0.8, label="level with it") for a in pts])
    yield from ctx.together(*[getattr(ctx, a).move(pts[a] - TIP * app, t=1.2, label="fingers over and under") for a in pts])
    yield from ctx.together(*[getattr(ctx, a).close(CZ, force=GRIP_N, label="pinch the console") for a in pts])
    for a in pts:
        print(f"[ps5] PINCH {a} tcp-goal mm {np.round((getattr(ctx, a).tcp() - (pts[a] - TIP * app)) * 1000, 1)} finger {getattr(ctx, a).finger:.4f}", flush=True)
    # 3. lift, roll a quarter turn about x (stand end down) while rising so the ring clears the unit, then set the ring on the unit
    c1 = np.asarray(s1.pos, float)
    R1 = s1.rotation()
    g0 = {a: getattr(ctx, a).grip.rot for a in pts}
    tcp0 = {a: getattr(ctx, a).at() for a in pts}

    def pose(th_deg, rise, dy=0.0):
        Rx = d.axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.radians(th_deg))
        c = c1 + np.array([0.0, dy, rise])
        return {a: (c + Rx @ (tcp0[a] - c1), d._as_grip(Rx @ g0[a])) for a in pts}

    path = [(0, 0.10), (-15, 0.16), (-30, 0.21), (-45, 0.24), (-60, 0.25), (-75, 0.25), (-90, 0.235)]
    for th, rise in path:
        p = pose(th, rise)
        yield from ctx.together(*[getattr(ctx, a).move(p[a][0], grip=p[a][1], t=1.0, label=f"roll {th}") for a in pts])
        print(f"[ps5] ROLL {th} console z over unit mm {(ctx.state('ps5').pos[2] - UNIT_TOP) * 1000:.0f} want {(c1[2] + rise - UNIT_TOP) * 1000:.0f} tilt {TRIAL.last.get('tilt')}", flush=True)
    for _ in range(2):  # the load sags in the grips: stand the console's long axis up by its own lean, as seen
        st = ctx.state("ps5")
        ax = -st.rotation()[:, 1]
        lean = math.degrees(math.acos(float(np.clip(ax[2], -1.0, 1.0))))
        if lean < 1.5:
            break
        k = np.cross(ax, np.array([0.0, 0.0, 1.0]))
        yield from ctx.together(*[getattr(ctx, a).arc(np.asarray(st.pos, float), k / np.linalg.norm(k), lean, t=0.8, label="stand it up") for a in pts])
    for _ in range(3):  # lower until the ring sits on the cloth
        st = ctx.state("ps5")
        gap = float(st.to_world(np.array([[0.0, YE + STAND_H, 0.0]]))[0][2]) - UNIT_TOP
        if gap < 0.004:
            break
        yield from ctx.together(*[getattr(ctx, a).move(getattr(ctx, a).at() - np.array([0.0, 0.0, gap - 0.002]), t=1.0, label="lower onto the ring") for a in pts])
    if mode == "drop":  # lazy: let go well over the cloth
        yield from ctx.together(*[getattr(ctx, a).move(getattr(ctx, a).at() + np.array([0.0, 0.0, 0.10]), t=1.0, label="lift 100 mm") for a in pts])
    yield from ctx.left.wait(0.5)
    print("[ps5] SET " + json.dumps(TRIAL.last), flush=True)
    yield from ctx.together(*[getattr(ctx, a).open(t=0.8, label="let go") for a in pts])
    yield from ctx.together(*[getattr(ctx, a).move(getattr(ctx, a).at() - 0.15 * app + np.array([0.0, 0.0, 0.03]), t=1.5, label="hands back") for a in pts])
    yield from ctx.left.wait(4.0)
    print("[ps5] END " + json.dumps(TRIAL.last), flush=True)
    st = ctx.state("ps5")
    print(f"[ps5] HANDS L {np.round(ctx.left.tcp(), 3)} R {np.round(ctx.right.tcp(), 3)} console {np.round(st.pos, 3)} R_col1 {np.round(st.rotation()[:, 1], 3)}", flush=True)
