"""Push a camlock tension rod out to both walls of a pantry shelf at an angle, twist its tubes to lock the length, and swing it level so it wedges as a guard bar.

The rod is a Kenney Twist & Fit No Tools camlock tension rod, 28-48 in (KN633NP): carbon steel, 245 g, outer tube 5/8 in (15.9 mm) across, inner tube 13 mm,
rubber end caps. It has no spring: its length is free until the inner tube is turned against the outer, which sets a cam and holds the length. The maker's
instruction is to extend the rod against both walls at an angle, twist to lock, and level into place. Locked at an angle, the rod is a few millimetres longer
than the gap straight across, so swinging it level squeezes the rubber caps and it wedges; locked short, it drops; unlocked, the caps push it shorter and it drops.

Sim: one free mechanism. The outer tube is its frame; the inner tube slides (inner_slide, along y) and turns (inner, about y) inside it. Each end cap is a 4 mm
slider on a spring (cap_l on the outer tube, cap_r on the inner): the rubber's give. The cam is a coupling in code: turned 80 degrees or more from as found,
the slide is held with up to 150 N; within 20 degrees it slides against 2 N; between, the hold rises linearly. The cam's turning resistance is a drive on
`inner` that holds its angle with at most 0.15 N m.
Estimated values have a real-trial range (README); each can be set by environment variable, and the demo scores 1.0 at both ends.

    python demokit.py task.py OUT --mode full                                   the full-score reference, filmed
    python demokit.py task.py OUT --no-record --mode nolock,squeeze,hold        lazy routes: 0, 0, 0.67
    python demokit.py task.py OUT --no-record --mode skew,tilt                  level rule: level but skewed in depth (commanded 4.0 deg, measured 4.3) 1.0; commanded 4.5 deg off level (measured 5.1) 0.67
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
INSTRUCTION = "Set the tension rod at an angle between the shelf's side walls, twist its tubes to lock the length, then swing it level so it wedges."
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.5

# -- the shelf: an 80 cm cupboard bay, standing on the table -----------------------------------------------------------------------------
BOARD = 0.018  # shelf board and side walls, 18 mm melamine board
S = TOP + BOARD  # the shelf's top face
WALL_Y = 0.40  # inner faces of the side walls, y = +-0.40: an 80 cm clear span
X_FRONT, X_BACK = 0.33, 0.70  # the bay's front and back edges
WALL_H = 0.32
LINE_X, LINE_H = 0.42, 0.12  # where the guard bar goes: 9 cm in from the front edge, 12 cm above the shelf (clear of an 11 cm can)
CAN_R, CAN_H = 0.0335, 0.111  # 15 oz food can, 67 x 111 mm
CANS = [(0.56, -0.15), (0.56, 0.0), (0.56, 0.15)]

# -- the rod, in its own frame (axis along y, left end at +y); joint zero is as found: collapsed to 75 cm ---------------------------------
# Estimated values can be set within their real-trial range by environment variable (README, Sizes): ROD_INNER_MM, ROD_HOLD_N, ROD_CAM_NM, ROD_CAP_N_PER_MM, ROD_LOCK_DEG, ROD_MU.
_ENV = os.environ.get
MU = float(_ENV("ROD_MU", "1.0"))  # rubber cap on painted wall and board: 0.5 to 1.0 (estimate)
R_OUT, R_CAP = 0.00795, 0.015  # 5/8 in outer tube (maker); flared end cap 30 mm across (scaled from the maker's photo)
R_IN = float(_ENV("ROD_INNER_MM", "13.0")) / 2000  # inner tube: slides in the 15.9 mm outer, so 12.7 to 14.3 mm (estimate)
Y_LTIP0 = 0.375  # left cap's face
CAP_T = 0.032  # each cap's length: a 20 mm sleeve, then a flare to the 30 mm face (scaled from the maker's photo)
CAP_PROFILE = [(0.0, 0.0), (0.0098, 0.0), (0.0098, 0.006), (0.0112, 0.008), (0.0098, 0.010), (0.0098, 0.018), (0.0125, 0.025), (0.015, 0.030), (0.015, 0.032), (0.0, 0.032)]
Y_MOUTH = -0.29  # the outer tube's open end, where the inner tube comes out
L_IN = 0.66
Y_RTIP0 = -0.375  # right cap's face at joint zero
CAP_TRAVEL = 0.004  # the rubber cap's give: up to 4 mm (estimate)
CAP_K = float(_ENV("ROD_CAP_N_PER_MM", "25")) * 1000  # its stiffness, 20 to 40 N/mm (estimate)
UNLOCKED = math.radians(20.0)  # cam: free within 20 deg of as found
LOCKED = math.radians(float(_ENV("ROD_LOCK_DEG", "80")))  # and holding from 80 deg, 60 to 120 (estimate)
HOLD_N = float(_ENV("ROD_HOLD_N", "150"))  # the locked cam's axial hold, 80 to 200 N (estimate)
SLIDE_N = 2.0
CAM_T = float(_ENV("ROD_CAM_NM", "0.15"))  # N m to turn the cam, 0.1 to 0.2 (estimate)
M_OUT, M_IN = 0.14, 0.095  # 245 g in all (maker)
PUSH_N = 10.0
Y0 = -0.022  # rod centre start y: its right cap near the right wall
LOOKS = {"white": m.Look(color=m.rgb(0.93, 0.93, 0.92), metallic=0.3, roughness=0.35), "inner": m.Look(color=m.rgb(0.90, 0.90, 0.89), metallic=0.3, roughness=0.3),
         "cap": m.Look(color=m.rgb(0.85, 0.85, 0.84), roughness=0.8), "collar": m.Look(color=m.rgb(0.80, 0.80, 0.79), roughness=0.6),
         "board": m.Look(color=m.rgb(0.86, 0.82, 0.74), roughness=0.7), "can": m.Look(color=m.rgb(0.75, 0.76, 0.78), metallic=0.8, roughness=0.3),
         "label1": m.Look(color=m.rgb(0.75, 0.12, 0.10), roughness=0.6), "label2": m.Look(color=m.rgb(0.15, 0.45, 0.20), roughness=0.6),
         "label3": m.Look(color=m.rgb(0.85, 0.65, 0.12), roughness=0.6)}


def _tube(r, y0, y1, n=32):
    return m.cylinder(r, y1 - y0, (0.0, (y0 + y1) / 2, 0.0), axis="y", segments=n)


def _rod() -> m.Part:
    outer = _tube(R_OUT, Y_MOUTH, Y_LTIP0 - CAP_T)
    collar = _tube(R_OUT + 0.0012, Y_MOUTH, Y_MOUTH + 0.022)  # the cam housing at the outer tube's mouth
    in0 = Y_RTIP0 + CAP_T
    inner = _tube(R_IN, in0, in0 + L_IN)
    cap_l = m.lathe(CAP_PROFILE, 32).aimed("+Y").moved((0.0, Y_LTIP0 - CAP_T, 0.0))
    cap_r = m.lathe(CAP_PROFILE, 32).aimed("-Y").moved((0.0, Y_RTIP0 + CAP_T, 0.0))
    slide = m.slider("inner_slide", "Y", -0.47, 0.04, [], [], mass=0.005, spring=0.0, damping=2.0, friction=0.0, effort=HOLD_N)  # 71 to 122 cm overall
    rot = m.hinge("inner", "Y", (0.0, 0.0, 0.0), -270.0, 270.0, [("inner", inner, "inner")], [m.col_hull("inner_c", _tube(R_IN, in0, Y_MOUTH, 24))],
                  mass=M_IN, spring=20.0, effort=CAM_T, friction=0.0, damping=0.002, parent="inner_slide", gravity=True,
                  inertia=(M_IN * L_IN ** 2 / 12, M_IN * R_IN ** 2, M_IN * L_IN ** 2 / 12))
    cl = m.slider("cap_l", "Y", -CAP_TRAVEL, 0.0, [("cap_l", cap_l, "cap")], [m.col_hull("cap_l_c", cap_l)], mass=0.005, spring=CAP_K, damping=20.0, friction=0.0, effort=400.0)
    cr = m.slider("cap_r", "Y", 0.0, CAP_TRAVEL, [("cap_r", cap_r, "cap")], [m.col_hull("cap_r_c", cap_r)], mass=0.005, spring=CAP_K, damping=20.0, friction=0.0, effort=400.0,
                  parent="inner")
    frame = [("outer", outer, "white"), ("collar", collar, "collar")]
    return m.mechanism("rod", (0.40, Y0, S + R_CAP + 0.0005), [slide, rot, cl, cr], LOOKS, frame_parts=frame,
                       frame_colliders=[m.col_hull("outer_c", _tube(R_OUT, Y_MOUTH, Y_LTIP0 - CAP_T, 24))], free_mass=M_OUT, friction=(MU, 0.9 * MU),
                       frame_inertia=(M_OUT * 0.65 ** 2 / 12, M_OUT * R_OUT ** 2, M_OUT * 0.65 ** 2 / 12))


def _bay() -> m.Part:
    depth, xc = X_BACK - X_FRONT, (X_FRONT + X_BACK) / 2
    parts = [("shelf", m.box((depth, 2 * WALL_Y + 2 * BOARD, BOARD), (xc, 0.0, BOARD / 2)), "board"),
             ("wall_l", m.box((depth, BOARD, WALL_H + BOARD), (xc, WALL_Y + BOARD / 2, (WALL_H + BOARD) / 2)), "board"),
             ("wall_r", m.box((depth, BOARD, WALL_H + BOARD), (xc, -WALL_Y - BOARD / 2, (WALL_H + BOARD) / 2)), "board"),
             ("back", m.box((BOARD, 2 * WALL_Y, WALL_H), (X_BACK - BOARD / 2, 0.0, BOARD + WALL_H / 2)), "board")]
    cols = [m.col_box("shelf_c", (depth, 2 * WALL_Y + 2 * BOARD, BOARD), (xc, 0.0, BOARD / 2)),
            m.col_box("wall_l_c", (depth, BOARD, WALL_H + BOARD), (xc, WALL_Y + BOARD / 2, (WALL_H + BOARD) / 2)),
            m.col_box("wall_r_c", (depth, BOARD, WALL_H + BOARD), (xc, -WALL_Y - BOARD / 2, (WALL_H + BOARD) / 2)),
            m.col_box("back_c", (BOARD, 2 * WALL_Y, WALL_H), (X_BACK - BOARD / 2, 0.0, BOARD + WALL_H / 2))]
    for i, (x, y) in enumerate(CANS):
        parts.append((f"can{i}", m.cylinder(CAN_R, CAN_H, (x, y, BOARD + CAN_H / 2)), "can"))
        parts.append((f"label{i}", m.cylinder(CAN_R + 0.0005, 0.075, (x, y, BOARD + CAN_H / 2)), f"label{i + 1}"))
        cols.append(m.col_cylinder(f"can{i}_c", CAN_R, CAN_H, (x, y, BOARD + CAN_H / 2)))
    return m.fixed("bay", (0.0, 0.0, TOP), parts, cols, LOOKS, friction=(MU, 0.9 * MU))


OBJECTS = [_bay(), _rod()]


# -- reading the rod ------------------------------------------------------------------------------------------------------------------

def tips_local(s) -> tuple[np.ndarray, np.ndarray]:
    """The two cap faces in the rod's frame, from the joints."""
    yl = Y_LTIP0 + s.joint("cap_l")
    yr = Y_RTIP0 + s.joint("inner_slide") + s.joint("cap_r")
    return np.array([0.0, yl, 0.0]), np.array([0.0, yr, 0.0])


def on_wall(p) -> bool:
    """A cap face against a side wall: within 3 mm of its face, and inside the wall's extent."""
    return abs(p[1]) >= WALL_Y - 0.003 and X_FRONT <= p[0] <= X_BACK and S < p[2] < S + WALL_H


def gripper_points(env) -> np.ndarray:
    """The whole of both grippers: the 24 finger points, plus points over the palm and housing (a box about 8 x 8 x 11 cm above the fingers)."""
    base = env.unwrapped
    origin = m._np(base.scene.env_origins[0])
    out = [m.fingertips(env)]
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        i = robot.find_bodies("gripper")[0][0]
        pose = m._np(robot.data.body_link_pose_w)[0, i]
        R = m.quat_matrix(pose[3:7])
        local = np.array([[sx, sy, sz] for sx in (-0.04, 0.0, 0.04) for sy in (-0.04, 0.0, 0.04) for sz in (0.0, 0.04, 0.08, 0.11)])
        out.append(pose[:3] - origin + local @ R.T)
    return np.vstack(out)


def seg_dist(pts, a, b) -> float:
    ab = b - a
    u = np.clip((pts - a) @ ab / max(ab @ ab, 1e-12), 0.0, 1.0)
    return float(np.min(np.linalg.norm(pts - (a + u[:, None] * ab), axis=1)))


class Trial:
    """The cam's coupling and the scorer's record."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.steps, self.cap, self.hold_at, self.cam_at = 0, None, None, None
        self.clear_t, self.push_t, self.push_z0, self.sag_mm, self.passed = 0.0, 0.0, None, None, False
        self.last = {}

    def step(self, env):
        import torch

        self.steps += 1
        dt = float(env.unwrapped.step_dt)
        s = m.state(env, "rod")
        rot = s.joint("inner")
        asset = env.unwrapped.scene["rod"]
        # the cam's turning resistance: a stick-slip hold of its angle, which a torque above CAM_T drags along
        if self.cam_at is None or abs(rot - self.cam_at) > 0.02:
            self.cam_at = rot if self.cam_at is None else rot - math.copysign(0.015, rot - self.cam_at)
        asset.set_joint_position_target(torch.tensor([[self.cam_at]], device=asset.device), joint_ids=[asset.find_joints("inner_joint")[0][0]])
        f = (abs(rot) - UNLOCKED) / (LOCKED - UNLOCKED)
        cap = SLIDE_N + (HOLD_N - SLIDE_N) * float(np.clip(f, 0.0, 1.0))
        if self.cap is None or abs(cap - self.cap) > 1.0:
            j = asset.find_joints("inner_slide_joint")[0][0]
            if cap > SLIDE_N + 1.0:
                if self.hold_at is None:
                    self.hold_at = s.joint("inner_slide")
                m._TOUCHED.add(("rod", "inner_slide"))
                asset.write_joint_stiffness_to_sim(torch.tensor([[50000.0]], device=asset.device), joint_ids=[j])
                asset.set_joint_position_target(torch.tensor([[self.hold_at]], device=asset.device), joint_ids=[j])
            else:
                self.hold_at = None
                asset.write_joint_stiffness_to_sim(torch.tensor([[0.0]], device=asset.device), joint_ids=[j])
            asset.write_joint_effort_limit_to_sim(torch.tensor([[cap]], device=asset.device), joint_ids=[j])
            self.cap = cap
        tl, tr = tips_local(s)
        wl, wr = s.to_world(np.vstack([tl, tr]))
        axis = (wl - wr) / max(np.linalg.norm(wl - wr), 1e-9)
        level_deg = math.degrees(math.asin(min(1.0, abs(axis[2]))))  # the axis's angle to the horizontal: vertical inclination only, a skew in depth is not counted
        mid = (wl + wr) / 2
        gap = seg_dist(gripper_points(env), wl, wr) - R_CAP
        on_walls = on_wall(wl) and on_wall(wr)
        off_shelf = min(wl[2], wr[2]) > S + 0.03
        on_cans = any(seg_dist(np.array([[x, y, S + CAN_H]]), wl, wr) < R_OUT + 0.004 for x, y in CANS)
        locked = abs(rot) >= LOCKED
        placed = on_walls and level_deg <= 3.0 and abs(mid[2] - S - LINE_H) <= 0.015
        self.clear_t = self.clear_t + dt if (gap > 0.03 and placed and locked) else 0.0
        if self.clear_t >= 5.0 and self.sag_mm is None:  # hands off for 5 s: the 10 N push into the shelf at mid-span, for 2 s
            if self.push_z0 is None:
                self.push_z0 = mid[2]
            m.push(env, "rod", (0.0, 0.0, -PUSH_N), at=mid)
            self.push_t += dt
            if self.push_t >= 2.0:
                self.sag_mm = float(self.push_z0 - mid[2]) * 1000
                self.passed = bool(self.sag_mm < 5.0 and on_walls)
                m.push(env, "rod", (0.0, 0.0, 0.0))
        elif self.push_z0 is not None and self.sag_mm is None and self.clear_t < 5.0:  # the hold broke during the push
            self.sag_mm = float(self.push_z0 - mid[2]) * 1000
            m.push(env, "rod", (0.0, 0.0, 0.0))
        self.last = {"t": round(self.steps * dt, 2), "rot_deg": round(math.degrees(rot), 1), "len_mm": round(float(np.linalg.norm(wl - wr)) * 1000, 1),
                     "cap_mm": [round(-s.joint("cap_l") * 1000, 2), round(s.joint("cap_r") * 1000, 2)], "tips": [[round(float(v), 4) for v in wl], [round(float(v), 4) for v in wr]],
                     "on_walls": bool(on_walls), "off_shelf": bool(off_shelf), "on_cans": bool(on_cans), "locked": bool(locked), "level_deg": round(level_deg, 2),
                     "height_mm": round((mid[2] - S) * 1000, 1), "hand_gap_mm": round(gap * 1000, 1), "clear_s": round(self.clear_t, 2),
                     "sag_mm": None if self.sag_mm is None else round(self.sag_mm, 2), "passed": self.passed}
        step_file = os.environ.get("TASK_STEP_FILE")
        if step_file:
            with open(step_file, "w") as fh:
                fh.write(f"{self.steps * dt:.2f}")
        if self.steps % max(1, round(0.25 / dt)) == 0:
            print("[rod] " + json.dumps({"mode": os.environ.get("TASK_DEMO", ""), **self.last}), flush=True)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects) -> float:
    c = TRIAL.last
    if not c or not c["off_shelf"] or c["on_cans"]:
        return 0.0  # void: the rod rests on the shelf or on the cans
    if not c["on_walls"]:
        return 0.0
    if not c["locked"]:
        return 0.33
    if TRIAL.passed and c["level_deg"] <= 3.0 and abs(c["height_mm"] - LINE_H * 1000) <= 15.0 and c["hand_gap_mm"] > 30.0:
        return 1.0
    return 0.67


def success(objects) -> bool:
    return score(objects) >= 1.0


# -- the scripted demo ----------------------------------------------------------------------------------------------------------------

TILT = math.radians(6.0)  # the rod's angle across the bay while its length is set: right end forward
PRESS = 0.002  # each tip aimed this far into its wall as the length is set (the arms stop about 1 mm short of it); level, the caps then give about 1 mm each
Y_GL, Y_GR = 0.12, -0.315  # grip points in the rod's frame: left on the outer tube, right on the inner


def demo_script(ctx):
    l, r = ctx.left, ctx.right

    def rod():
        return ctx.state("rod")

    def tcp_on(y_local, radius):
        """The TCP for a down grip on the rod's tube at y_local: above the axis by the tube's radius."""
        return rod().to_world(np.array([[0.0, y_local, 0.0]]))[0] + np.array([0.0, 0.0, radius - 0.0014])

    def centre(at):
        s = rod()
        a = s.rotation()[:, 1]
        return s.pos + ((at - s.pos) @ a) * a, a

    zl0, zr0 = tcp_on(Y_GL, R_OUT), tcp_on(Y_GR, R_IN)
    up = np.array([0.0, 0.0, 0.08])
    yield from ctx.together(l.move(zl0 + up, grip=d.down(-90), finger=d.FINGER_OPEN, t=2.0, label="over the outer tube"),
                            r.move(zr0 + up, grip=d.down(90), finger=d.FINGER_OPEN, t=2.0, label="over the inner tube"))
    yield from ctx.together(l.move(zl0, t=1.0, label="down"), r.move(zr0, t=1.0, label="down"))
    yield from ctx.together(l.close(2 * R_OUT, force=40.0, label="grip outer"), r.close(2 * R_IN, force=40.0, label="grip inner"))
    z = S + LINE_H + 0.008  # the rod settles about 8 mm as the hands let go
    lift = np.array([0.0, 0.0, z - rod().pos[2]])
    yield from ctx.together(l.move(l.at() + lift, t=1.5, label="lift"), r.move(r.at() + lift, t=1.5, label="lift"))

    tilt = 0.0 if ctx.mode == "squeeze" else TILT
    press = 0.002 if ctx.mode == "squeeze" else PRESS
    v = np.array([-math.sin(tilt), -math.cos(tilt), 0.0])  # from the left tip towards the right tip
    lv = np.array([LINE_X, WALL_Y + press, z])  # where the left tip would be with no wall
    span = (2 * WALL_Y + 2 * press) / math.cos(tilt)
    tl = lv + (Y_LTIP0 - Y_GL) * v + np.array([0.0, 0.0, R_OUT - 0.0014])
    tr = lv + (span - (Y_GR - Y_RTIP0)) * v + np.array([0.0, 0.0, R_IN - 0.0014])
    yaw = -math.degrees(tilt)
    yield from ctx.together(l.move(tl, grip=d.down(-90 + yaw), t=3.5, label="push the outer tube's end to the left wall"),
                            r.move(tr, grip=d.down(90 + yaw), t=3.5, label="draw the inner tube's end to the right wall"))
    yield from ctx.together(l.wait(0.5), r.wait(0.5))
    if ctx.mode == "squeeze":  # lazy: held level and squeezed into both walls, let go without a twist
        yield from ctx.together(l.open(label="let go"), r.open(label="let go"))
        yield from ctx.together(l.move(l.at() + np.array([0.0, 0.0, 0.08]), t=1.0, label="clear"), r.move(r.at() + np.array([0.0, 0.0, 0.08]), t=1.0, label="clear"))
        yield from l.wait(8.0)
        print("[rod] END " + json.dumps({"mode": ctx.mode, **TRIAL.last}), flush=True)
        return

    reach = R_IN - 0.0014

    def lift_level():
        """The right end drops while the right hand regrips (the left pinch is a pivot): lift it back level with the left end."""
        s = rod()
        wl, wr = s.to_world(np.vstack(tips_local(s)))
        yield from r.move(r.at() + np.array([0.0, 0.0, 0.9 * (wl[2] - wr[2])]), t=0.6, label="lift level")

    if ctx.mode != "nolock":
        for k in range(5):  # five eighth-turns of the inner tube, the left hand holding the outer still; the right keeps its last grip
            c, a = centre(r.at())
            yield from r.arc(c, a, -45.0, t=2.4, label=f"twist, eighth {k + 1}")
            if k < 4:
                yield from r.open(t=0.4, label="let go")
                c, a = centre(r.at())
                yield from r.arc(c, a, 45.0, t=0.8, label="turn the hand back")
                c, a = centre(r.at())
                yield from r.move(c - reach * r.approach_axis(), t=0.5, label="onto the tube")
                yield from r.close(2 * R_IN, force=40.0, t=0.4, label="grip again")
                yield from lift_level()
    # the swing: both hands carry the rod level, each grip to its place on the level line, the caps squeezing into the walls
    # range checks of the level rule: "skew" sets the right end commanded 56 mm nearer the front (4.0 deg in depth; measured 60 mm, 4.3 deg; still level: keeps 1.0);
    # "tilt" sets the left end 31 mm above the line and the right 31 mm below (commanded 4.5 deg off level; measured 5.1 deg; middle at height: loses 1.0)
    dx = -0.056 if ctx.mode == "skew" else 0.0
    dz = 0.063 if ctx.mode == "tilt" else 0.0
    left_level = np.array([LINE_X, WALL_Y + PRESS - (Y_LTIP0 - Y_GL), z + R_OUT - 0.0014 + dz / 2])
    right_level = np.array([LINE_X + dx, -WALL_Y - PRESS + (Y_GR - Y_RTIP0), z + R_IN - 0.0014 - dz / 2])
    yield from ctx.together(l.move(left_level, grip=d.down(-90), t=2.5, label="swing level"), r.move(right_level - reach * r.approach_axis() - np.array([0.0, 0.0, reach]), t=2.5, label="swing level"))
    yield from ctx.together(l.wait(0.5), r.wait(0.5))
    for _ in range(4):  # level it by eye: both ends to the bar's height, the right end in line with the left across
        s = rod()
        wl, wr = s.to_world(np.vstack(tips_local(s)))
        el = np.array([0.0, 0.0, S + LINE_H + dz / 2 - wl[2]])
        er = np.array([wl[0] + dx - wr[0], 0.0, S + LINE_H - dz / 2 - wr[2]])
        if max(np.linalg.norm(el), np.linalg.norm(er)) < 0.004:
            break
        yield from ctx.together(l.move(l.at() + 0.6 * el, t=0.8, label="level"), r.move(r.at() + 0.6 * er, t=0.8, label="level"))
    yield from l.open(t=0.4, label="let go")
    yield from l.move(l.at() + np.array([0.0, 0.0, 0.04]), t=0.5, label="lift off")
    yield from l.move(l.at() + np.array([-0.10, 0.0, 0.05]), t=0.8, label="clear")
    for _ in range(4):  # the left end rests on its wall now: the right hand brings its end in line with it
        s = rod()
        wl, wr = s.to_world(np.vstack(tips_local(s)))
        er = np.array([wl[0] + dx - wr[0], 0.0, wl[2] - dz - wr[2]])
        if np.linalg.norm(er) < 0.004:
            break
        yield from r.move(r.at() + 0.8 * er, t=0.8, label="level")
    if ctx.mode == "hold":  # lazy: never lets go
        yield from l.wait(12.0)
        print("[rod] END " + json.dumps({"mode": ctx.mode, **TRIAL.last}), flush=True)
        return
    yield from r.open(t=0.4, label="let go")
    yield from r.move(r.at() + np.array([0.0, 0.0, 0.04]), t=0.5, label="lift off")
    yield from r.move(r.at() + np.array([-0.10, 0.0, 0.05]), t=0.8, label="clear")
    yield from l.wait(9.0)
    print("[rod] END " + json.dumps({"mode": ctx.mode, **TRIAL.last}), flush=True)
