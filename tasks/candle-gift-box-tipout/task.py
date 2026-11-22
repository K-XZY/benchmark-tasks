"""Tip a boxed glass jar candle out of its snug, open gift box onto the table: turn the box over with the jar squeezed in it, ease the grip at the table, lift the box away, and stand the jar upright on its base.

A Diptyque Classic Candle 190 g (Ambre; emea.diptyqueparis.com/en-pf/products/ambre-classic-candle: glass jar 90 mm tall, 77 mm across, 190 g of wax) stands in its
square paperboard gift box, top open: 80 mm square inside (1.5 mm a side round the jar), 100 mm deep, 1.5 mm board. The box is a free articulation: its floor and the two
walls facing the arms are one rigid body; the two walls the right hand squeezes are sprung panels that flex inward up to 3 mm (8 N/mm, from a plate bound for
1.5 mm board pressed at mid-panel), so a firm squeeze presses them onto the glass and holds the jar by friction, and an eased squeeze lets them spring back and the jar slide.

The right hand grips the box across the sprung panels, lifts it, turns it over, lowers the rim to the table, eases the squeeze so the jar slides out rim-down,
lifts the box away and sets it aside. The hand then turns the jar upright and stands it on its base on the table, since a jar tipped out of its box lies on its rim.

    python demokit.py task.py OUT --no-record --mode full,none,ease_5cm,ease_2cm,no_ease,no_upright,upright_drop,slow_release_upright
    python demokit.py task.py OUT          (final: 640 x 360, the GPU to yourself)
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
import mechkit as m  # noqa: E402

SCENE = "station"
INSTRUCTION = "The candle is too snug in its gift box to lift out: tip it out without dropping it, stand it upright on its base, and set the empty box aside."
SUCCESS_HOLD_S = 0.0  # the scorer holds the end state itself (2 s, counted in control steps)
SCORE_OVER = "end"
TOP = d.TABLE_TOP

# -- the jar (maker's sizes) ---------------------------------------------------------------------------------------------------------------------------------------------------------------
JAR_R = 0.0385  # 77 mm across
JAR_H = 0.090
JAR_M = float(os.environ.get("C_JARM", "0.50"))  # 190 g of wax plus the glass: 4 mm soda-lime wall and 8 mm base, pi*(38.5^2-34.5^2)*90 + pi*38.5^2*8 = 120 cm3 at 2.5 g/cm3 = 0.30 kg (shown calculation; trial: kitchen scale, 0.45-0.55 kg)
JAR_MU = float(os.environ.get("C_JMU", "0.45"))  # glass on paperboard (estimate; trial: TAPPI T 815 inclined plane, tan of the slide angle, 0.30-0.60)
GLASS_T = 0.004
WAX_TOP = 0.072  # the wax surface, from the jar's base

# -- the box ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
IN = float(os.environ.get("C_IN", "0.040"))  # half the inside width: 80 mm square, 1.5 mm a side round the jar (estimate; trial: calliper, 79-81 mm)
WALL = 0.0015  # paperboard thickness
DEPTH = 0.100  # inside depth, floor top to rim
FLOOR = 0.0015
RIM = FLOOR + DEPTH  # the rim's height above the box's underside
OUT = IN + WALL  # half the outside width: 83 mm
BOX_M = 0.06  # an empty box: 0.051 m2 of 1.5 mm board at 0.8 g/cm3 = 0.061 kg (shown calculation)
PANEL_K = float(os.environ.get("C_PK", "8000"))  # N/m: inward stiffness of a squeezed side panel at mid-height. Plate bound: D = E t^3/12(1-v^2) = 0.9 N m for 1.5 mm board (E 3 GPa);
# simply supported 80 x 100 mm, k = D/(0.0116 a^2) = 12 kN/m, an upper bound, since the open top edge only softens it: trial range 2-12 kN/m (push gauge at mid-wall)
PANEL_TRAVEL = 0.003  # m it may flex inward: more than the 1-2 mm gap to the glass, so the glass, not this stop, ends the squeeze
BOX_MU = float(os.environ.get("C_BMU", "0.50"))  # paperboard on the table and the gripper pads (estimate; trial: TAPPI T 815 inclined plane, 0.35-0.60)
BOX_XY = (0.40, -0.14)

LOOKS = {"glass": m.Look(m.rgb(0.90, 0.87, 0.76), roughness=0.08, opacity=0.35), "wax": m.Look(m.rgb(0.93, 0.92, 0.84), roughness=0.6),
         "label": m.Look(m.rgb(0.96, 0.96, 0.94), roughness=0.5), "ink": m.Look(m.rgb(0.05, 0.05, 0.05), roughness=0.5),
         "board": m.Look(m.rgb(0.05, 0.08, 0.20), roughness=0.7), "board_in": m.Look(m.rgb(0.85, 0.83, 0.78), roughness=0.9),
         "dots": m.Look(m.rgb(0.75, 0.15, 0.12), roughness=0.6), "gold": m.Look(m.rgb(0.80, 0.62, 0.30), roughness=0.5)}


def _jar() -> m.Part:
    glass = m.lathe([(0.0, 0.0), (JAR_R - 0.002, 0.0), (JAR_R, 0.002), (JAR_R, JAR_H), (JAR_R - GLASS_T, JAR_H), (JAR_R - GLASS_T, 0.008), (0.0, 0.008)], segments=40)
    wax = m.cylinder(JAR_R - GLASS_T - 0.0005, WAX_TOP - 0.0085, (0.0, 0.0, (WAX_TOP + 0.0085) / 2), segments=40)
    wick = m.cylinder(0.0012, 0.008, (0.0, 0.0, WAX_TOP + 0.004), segments=8)
    # the oval label on the glass's front (towards -x, the robot), and its printed oval ring
    th = np.linspace(0, 2 * np.pi, 24, endpoint=False)
    oval = [(0.022 * math.cos(t), 0.032 * math.sin(t)) for t in th]
    label = m.prism(oval, 0.0, 0.0008).turned(90.0, "x").turned(-90.0, "z").moved((-JAR_R - 0.0002, 0.0, 0.045))
    ring = m.tube([(-JAR_R - 0.0012, 0.018 * math.cos(t), 0.045 + 0.027 * math.sin(t)) for t in np.r_[th, th[:1]]], 0.0006, sides=4)
    parts = [("glass", glass, "glass"), ("wax", wax, "wax"), ("wick", wick, "ink"), ("label", label, "label"), ("ring", ring, "ink")]
    cols = [m.col_cylinder("c_jar", JAR_R, JAR_H, (0.0, 0.0, JAR_H / 2))]
    return m.free("jar", (BOX_XY[0], BOX_XY[1], TOP + FLOOR + 0.0003), parts, cols, LOOKS, mass=JAR_M, friction=(JAR_MU, JAR_MU - 0.05), torsional_patch=0.02, solver_iters=48)


def _box() -> m.Part:
    floor = m.box((2 * OUT, 2 * OUT, FLOOR), (0.0, 0.0, FLOOR / 2))
    fixed_walls = [m.box((2 * IN - 0.001, WALL, RIM), (0.0, s * (IN + WALL / 2), RIM / 2)) for s in (1.0, -1.0)]
    frame_parts = [("floor", floor, "board")] + [(f"wall_y{k}", w, "board") for k, w in enumerate(fixed_walls)]
    # a few printed ornaments on the outside of the fixed walls, as on the holiday box
    for k, (x, z, look) in enumerate([(-0.02, 0.07, "dots"), (0.025, 0.035, "gold"), (0.0, 0.085, "gold"), (-0.028, 0.025, "dots"), (0.03, 0.075, "dots")]):
        for s in (1.0, -1.0):
            frame_parts.append((f"orn{k}{'p' if s > 0 else 'n'}", m.cylinder(0.006, 0.0006, (x, s * (OUT + 0.0003), z), axis="y", segments=12), look))
    frame_cols = [m.col_box("c_floor", (2 * OUT, 2 * OUT, FLOOR), (0.0, 0.0, FLOOR / 2))] + \
                 [m.col_box(f"c_wall_y{k}", (2 * IN - 0.001, WALL, RIM), (0.0, s * (IN + WALL / 2), RIM / 2)) for k, s in enumerate((1.0, -1.0))]
    links = []
    for s, nm in ((1.0, "px"), (-1.0, "nx")):
        size, c = (WALL, 2 * IN, DEPTH), (s * (IN + WALL / 2), 0.0, FLOOR + DEPTH / 2)
        lo, hi = (-PANEL_TRAVEL, 0.0) if s > 0 else (0.0, PANEL_TRAVEL)
        links.append(m.slider(f"panel_{nm}", "X", lo, hi, [("board", m.box(size, c), "board")], [m.col_box(f"c_panel_{nm}", size, c)], at=c,
                              mass=0.01, spring=PANEL_K, damping=5.0, friction=0.0, effort=200.0))  # solver values: a light panel, damping
        # that settles it within a control step, no joint friction (the board's own friction is in the contacts); effort far above the hand's 40 N
    return m.mechanism("box", (BOX_XY[0], BOX_XY[1], TOP + 0.0003), links, LOOKS, frame_parts=frame_parts, frame_colliders=frame_cols, friction=(BOX_MU, BOX_MU - 0.05),
                       solver_iters=48, free_mass=BOX_M, frame_inertia=(1e-4, 1e-4, 1e-4))


OBJECTS = [_box(), _jar()]

# -- the scorer -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
HOLD_STEPS = 40  # 2 s at 20 Hz
SQUARE_DEG = 10.0  # the jar stands on its rim (0.5) or on its base (1.0): its axis within this of vertical
TURNED_DEG = 150.0  # the box counts as turned over past this tilt
LIFTED_M = 0.03  # with the jar's lowest point this high above the table
CLEAR_BOX_M = 0.005  # the box clear of the jar
CLEAR_HAND_M = 0.02  # hands off the jar at the end
HELD_M = 0.008  # out of the box: both finger pads of one hand within 8 mm of the bare glass hold it (the pad points are sampled on the finger's centre line; on a 77 mm jar
# the closed pads read 1 to 6 mm from the glass they touch)
SLIP_M = 0.01  # the jar sliding down more than this through the closed fingers is no longer held
DROP_M = 0.03  # the jar comes down more than this from where it last stood still, both against the table and against the box: a drop
TABLE_X, TABLE_Y = (0.0, 0.85), (-0.60, 0.60)


def jar_points() -> np.ndarray:
    """Points over the jar's outer surface, in its own frame (base centre at the origin, z up to the rim)."""
    th = np.linspace(0, 2 * np.pi, 16, endpoint=False)
    ring = np.stack([JAR_R * np.cos(th), JAR_R * np.sin(th)], 1)
    pts = [np.c_[ring, np.full(16, z)] for z in (0.0, JAR_H / 2, JAR_H)]
    pts.append(np.array([[0.0, 0.0, 0.0], [0.0, 0.0, JAR_H]]))
    return np.vstack(pts)


JAR_PTS = jar_points()


def gap_to_box(p_local) -> np.ndarray:
    """A lower bound on each point's distance from the box's outer envelope (box frame, 0 inside it)."""
    p = np.atleast_2d(p_local)
    g = np.stack([np.abs(p[:, 0]) - OUT - PANEL_TRAVEL, np.abs(p[:, 1]) - OUT, -p[:, 2], p[:, 2] - RIM], axis=1)
    return np.maximum(g.max(axis=1), 0.0)


def gap_to_jar(p_local) -> np.ndarray:
    """Each point's distance from the jar's solid cylinder (jar frame)."""
    p = np.atleast_2d(p_local)
    dr = np.linalg.norm(p[:, :2], axis=1) - JAR_R
    dz = np.maximum(-p[:, 2], p[:, 2] - JAR_H)
    out = np.hypot(np.maximum(dr, 0.0), np.maximum(dz, 0.0))
    return np.where((dr <= 0) & (dz <= 0), 0.0, out)


def gripper_points(env) -> np.ndarray:
    """Points filling both grippers: fingers, pads, palm and housing, up to the wrist flange (station frame), left hand's first."""
    base = env.unwrapped
    origin = m._np(base.scene.env_origins[0])
    out = []
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        i = robot.find_bodies("gripper")[0][0]
        pose = m._np(robot.data.body_link_pose_w)[0, i]
        R = m.quat_matrix(pose[3:7])
        op = abs(float(m._np(robot.data.joint_pos)[0, robot.find_joints("joint7")[0][0]]))
        g = np.stack(np.meshgrid(np.linspace(-(op + 0.012), op + 0.012, 5), np.linspace(-0.025, 0.025, 3), np.linspace(-0.04, 0.16, 11), indexing="ij"), -1).reshape(-1, 3)
        out.append(pose[:3] - origin + g @ R.T)
    return np.vstack(out)


def finger_pads(env) -> np.ndarray:
    """The two finger pads of each hand, 3 points each along the finger at the TCP: (2 hands, 2 fingers, 3, 3), left hand first."""
    base = env.unwrapped
    origin = m._np(base.scene.env_origins[0])
    out = []
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        i = robot.find_bodies("gripper")[0][0]
        pose = m._np(robot.data.body_link_pose_w)[0, i]
        R = m.quat_matrix(pose[3:7])
        op = abs(float(m._np(robot.data.joint_pos)[0, robot.find_joints("joint7")[0][0]]))
        out.append([[pose[:3] - origin + R @ np.array([sx * op, 0.0, dz]) for dz in (0.12, 0.134, 0.148)] for sx in (-1.0, 1.0)])
    return np.array(out)


class Trial:
    def reset(self, env=None):
        self.n = 0
        self.void = False
        self.why = ""
        self.turned_ok = False  # the box turned over with the jar still in it, off the table (needed for both tiers)
        self.out_steps = 0
        self.out_ok = False  # tier 0.5: the jar out on the table, on its rim or base, box and hands clear, for 2 s
        self.ok_steps = 0
        self.peak, self.rel_peak = -1.0, -1.0
        self.hand_peak = None
        self.last = {}

    def step(self, env):
        self.n += 1
        jar, box = m.state(env, "jar"), m.state(env, "box")
        Rj = jar.rotation()
        pts_w = JAR_PTS @ Rj.T + jar.pos
        box_gap = float(gap_to_box(box.to_local(pts_w)).min())
        in_box = box_gap <= 0.0
        pads = finger_pads(env)
        pg = gap_to_jar(jar.to_local(pads.reshape(-1, 3))).reshape(2, 2, 3).min(axis=2)
        hands = (pg < HELD_M).all(axis=1)  # which hand has both pads on the glass
        held = (not in_box) and bool(hands.any())  # out of the box, both pads of one hand on the glass
        if held:  # holding means the glass rides with the hand: a jar sliding down through the fingers (a slow release) is not held
            off = float(np.linalg.norm(np.asarray(jar.pos) - pads[int(hands.argmax())].reshape(-1, 3).mean(axis=0)))  # jar centre to the pads' centre: constant while the glass rides in the hand, turns included
            self.hand_peak = off if self.hand_peak is None else min(self.hand_peak, off)
            held = off - self.hand_peak <= SLIP_M
        else:
            self.hand_peak = None
        gp = gripper_points(env)
        hand_gap = float(gap_to_jar(jar.to_local(gp)).min())
        box_hand = float(gap_to_box(box.to_local(gp)).min())
        corners = box.to_world(np.array([[sx * OUT, sy * OUT, z] for sx in (-1, 1) for sy in (-1, 1) for z in (0.0, RIM)]))
        box_down = bool(corners[:, 2].min() < TOP + 0.005 and np.linalg.norm(box.lin_vel) < 0.02 and box_hand >= CLEAR_HAND_M / 2)  # set down on the table and let go
        v = float(np.linalg.norm(jar.lin_vel))
        low = float(pts_w[:, 2].min() - TOP)
        # the drop: since the jar last stood still (or was last in a hand), how far it has come down from its highest, against the table and against
        # the box. A box carried with the jar in it moves the jar with it, and a box lifted off the jar leaves it where it is: neither is a fall.
        rel = float(jar.pos[2] - box.pos[2])
        if held or v < 0.02:
            self.peak, self.rel_peak = low, rel
        else:
            self.peak, self.rel_peak = max(self.peak, low), max(self.rel_peak, rel)
        fall = min(self.peak - low, self.rel_peak - rel)
        resting = low < 0.004 and v < 0.02
        upright = float(np.degrees(np.arccos(np.clip(Rj[2, 2], -1.0, 1.0))))
        if self.n > 10 and not self.void:
            if fall > DROP_M:
                self.void, self.why = True, f"jar came down {100 * fall:.1f} cm from where it last stood still (dropped)"
            on_table = TABLE_X[0] < jar.pos[0] < TABLE_X[1] and TABLE_Y[0] < jar.pos[1] < TABLE_Y[1] and jar.pos[2] > TOP - 0.05
            if not on_table:
                self.void, self.why = True, "jar off the table"
        if self.n > 10 and in_box and box.tilt_deg() >= TURNED_DEG and low >= LIFTED_M:
            self.turned_ok = True
        out = self.turned_ok and box_gap >= CLEAR_BOX_M and resting and min(upright, 180.0 - upright) <= SQUARE_DEG and hand_gap >= CLEAR_HAND_M
        self.out_steps = self.out_steps + 1 if out else 0
        self.out_ok = self.out_ok or self.out_steps >= HOLD_STEPS
        full = out and upright <= SQUARE_DEG and box_down  # standing on its base, opening up
        self.ok_steps = self.ok_steps + 1 if full else 0
        self.last = {"n": self.n, "box_gap": round(box_gap, 3), "in_box": in_box, "held": held, "hand_gap": round(hand_gap, 3), "v": round(v, 3), "fall": round(fall, 3), "low": round(low, 3), "box_low": round(float(corners[:, 2].min() - TOP), 3),
                     "upright": round(upright, 1), "box_tilt": round(box.tilt_deg(), 1), "box_down": box_down, "panels_mm": [round(1000 * x, 1) for x in box.joints.values()],
                     "jar_xy": [round(float(x), 3) for x in jar.pos[:2]], "pads_mm": [round(1000 * float(x), 1) for x in pg.ravel()], "turned": self.turned_ok, "out": self.out_ok, "ok": self.ok_steps, "void": self.why, "tier": self.score()}
        if self.n % 20 == 0 or (self.void and self.why and not getattr(self, "_told", False)):
            self._told = self.void
            print("[candle] " + json.dumps({"mode": os.environ.get("TASK_DEMO", ""), **self.last}), flush=True)

    def score(self) -> float:
        if self.void:
            return 0.0
        return 1.0 if self.ok_steps >= HOLD_STEPS else (0.5 if self.out_steps >= HOLD_STEPS else 0.0)  # both tiers hold at the end of the run


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects) -> float:
    return TRIAL.score()


def success(objects) -> bool:
    return TRIAL.score() >= 1.0


# -- the scripted demo ------------------------------------------------------------------------------------------------------------------------------------------------------------------------
GRIP_Z = float(os.environ.get("C_GRIPZ", "0.040"))  # TCP height on the box's panels, from the box's underside
SQUEEZE_N = float(os.environ.get("C_SQUEEZE", "40"))
SQUEEZE_IN = float(os.environ.get("C_SQIN", "0.0"))  # m the close is commanded past the panels-on-glass width
EASE_N = float(os.environ.get("C_EASE", "2.5"))
RIM_GAP = float(os.environ.get("C_RIMGAP", "-0.020"))  # commanded rim height above the table at the release: the soft arm (wrist joints 10 N m/rad) sags about 2.5 cm under the
# turned-over box and jar, so the command is below the table and the rim comes down to it
ASIDE_XY = (0.53, -0.02)  # where the empty box is set down
SET_XY = (0.53, -0.16)  # where the upright jar is stood: here the rolled hand reaches the table (nearer the arm base the shoulder stalls)
JAR_XY = (float(os.environ.get("C_JARX", "0.396")), float(os.environ.get("C_JARY", "-0.145")))  # where the jar lands on its rim: it slides out from under the box and ends on its rim
JAR_GRIP_Z = 0.035  # TCP height on the glass, from the table, with the jar on its rim: low on the glass, so after the turn the base hangs 5.5 cm below the hand;
# the wrist sags under the held jar as under the box, so the set-down command is 2 cm below the table
JAR_SQUEEZE_N = 30.0


def demo_script(ctx):
    """Modes: full. none: the arm waits. ease_5cm / ease_2cm: the squeeze is eased with the rim about 5 cm / 2 cm above the table (must score 0 / 1.0).
    no_ease: the box is turned over and set back down still squeezed, so the jar never comes out (0). no_upright: the jar is left on its rim (0.5).
    upright_drop: the jar is turned upright and let go well above the table (0). slow_release_upright: the same height, the fingers opened over 6 s so the jar slides down through them (0)."""
    mode = ctx.mode
    if mode not in ("full", "none", "ease_5cm", "ease_2cm", "no_ease", "no_upright", "upright_drop", "slow_release_upright"):
        raise ValueError(f"unknown mode {mode!r}")
    R_ = ctx.right
    if mode == "none":
        yield from R_.wait(3.0)
        return
    # fingers level, pointing +y, closing along x across the two sprung panels; the hand comes down from above
    gr = d.level(90.0)
    g0 = np.array([BOX_XY[0], BOX_XY[1], TOP + GRIP_Z])
    yield from R_.move(g0 + [0.0, 0.0, 0.13], grip=gr, finger=d.FINGER_OPEN, label="above the box")
    yield from R_.move(g0, t=1.5, label="down astride the box")
    yield from R_.close(2 * JAR_R + 2 * WALL - SQUEEZE_IN, SQUEEZE_N, label="squeeze the box onto the glass")
    yield from R_.move(R_.at() + [0.0, 0.0, 0.16], t=1.5, label="lift")
    yield from R_.arc(R_.at(), [0.0, 1.0, 0.0], 180.0, t=4.0, label="turn the box over")
    yield from R_.wait(0.5)
    hz = TOP + (RIM - GRIP_Z) + {"ease_5cm": 0.075, "ease_2cm": 0.045}.get(mode, RIM_GAP)  # after the turn the rim is RIM - GRIP_Z below the TCP
    yield from R_.move([BOX_XY[0], BOX_XY[1], hz], t=2.0, label="rim down to the table")
    yield from R_.wait(0.5)
    if mode != "no_ease":
        yield from R_.close(2 * OUT, EASE_N, t=1.0, label="ease the squeeze")
    yield from R_.wait(1.5)
    yield from R_.move(R_.at() + [0.0, 0.0, 0.13], t=1.5, label="lift the box off the jar")
    yield from R_.move([ASIDE_XY[0], ASIDE_XY[1], R_.at()[2]], t=1.5, label="aside")
    yield from R_.move([ASIDE_XY[0], ASIDE_XY[1], TOP + (RIM - GRIP_Z) + 0.004], t=1.5, label="box down")
    yield from R_.open()
    yield from R_.move(R_.at() + [0.0, 0.0, 0.12], t=1.0, label="hand up")
    yield from R_.move(R_.at() + [-0.08, -0.06, 0.0], t=1.0, label="hand clear")
    if mode in ("no_upright", "no_ease"):
        yield from R_.wait(3.0)
        print("[candle] END " + json.dumps({"mode": mode, **TRIAL.last}), flush=True)
        return
    # turn the jar upright: the hand rolls back over above the jar, takes the glass low on its side, lifts it and rolls over again, as it turned the box
    j0 = np.array([JAR_XY[0], JAR_XY[1], TOP + JAR_GRIP_Z])
    yield from R_.move(j0 + [0.0, 0.0, 0.15], grip=gr, finger=d.FINGER_OPEN, t=2.5, label="hand back over, above the jar")
    yield from R_.move(j0, t=1.5, label="down astride the jar")
    yield from R_.close(2 * JAR_R, JAR_SQUEEZE_N, label="hold the glass")
    yield from R_.move(R_.at() + [0.0, 0.0, 0.10], t=1.2, label="lift the jar")
    yield from R_.arc(R_.at(), [0.0, 1.0, 0.0], 180.0, t=3.0, label="turn the jar upright")
    yield from R_.wait(0.5)
    yield from R_.move([SET_XY[0], SET_XY[1], R_.at()[2]], t=1.5, label="carry the jar")
    yield from R_.move([SET_XY[0], SET_XY[1], TOP + (JAR_H - JAR_GRIP_Z) + (0.11 if mode in ("upright_drop", "slow_release_upright") else -0.02)], t=1.5, label="base down to the table")
    yield from R_.wait(0.5)
    yield from R_.open(t=6.0 if mode == "slow_release_upright" else 0.5)
    yield from R_.move(R_.at() + [0.0, 0.0, 0.15], t=1.5, label="hand up")
    yield from R_.wait(3.0)
    print("[candle] END " + json.dumps({"mode": mode, **TRIAL.last}), flush=True)
