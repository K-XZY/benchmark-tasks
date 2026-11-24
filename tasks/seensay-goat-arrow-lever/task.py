"""Turn a See 'n Say's arrow to the goat and pull its lever without moving the toy.

A Fisher-Price See 'n Say The Farmer Says (Mattel CBN98: barn-shaped body 229 mm tall, 203 mm wide, 51 mm thick, 420 g) lies face up on the table, roof away
from the robot. Its face has ten 36-degree picture sectors round a centre arrow that turns freely; the arrow starts on the cat. A yellow lever on the toy's right
edge, near the roof, swings 75 degrees toward the robot against a return spring (4.4 N at its head at the trigger) and springs home. The toy plays the animal
the arrow points at once the lever passes its trigger, 90 % of the stroke. Sound is not simulated: the scorer reads the arrow's sector and the lever's angle.

    python demokit.py task.py OUT --no-record --mode full,none,cat,no_pin
    python demokit.py task.py OUT          (final: 640 x 360)
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
INSTRUCTION = "Make the See 'n Say play the goat: turn the arrow to the goat and pull the lever."
SUCCESS_HOLD_S = 0.0  # the scorer holds its own 2 s
SCORE_OVER = "end"
TOP = d.TABLE_TOP

# -- the toy, in its own frame: x toward the roof, y to its left, z up from the table -------------------------------------------------------
TOY_XY = (0.30, 0.0)  # tape marks on the table: the toy's centre
L, W, T = 0.229, 0.203, 0.051  # body, maker's listing
EAVE_X = 0.0645  # where the roof's slopes start
MASS = 0.420
DIAL_C = (-0.015, 0.0)  # the dial's centre on the face
DIAL_R = 0.080
SECTOR_DEG = 36.0
ANIMALS = ("goose", "turkey", "mouse", "goat", "music", "quiz", "chick", "owl", "bunny", "cat")  # clockwise seen from above, goose first after the roof


def sector_centre(name: str) -> float:
    """A sector's centre, degrees about +z from the toy's +x (the roof), clockwise seen from above."""
    return -(SECTOR_DEG / 2 + SECTOR_DEG * ANIMALS.index(name))


ARROW_START = sector_centre("cat") + 360.0  # 18: the arrow's direction at joint zero
GOAT_JOINT = (sector_centre("goat") - ARROW_START + 180.0) % 360.0 - 180.0  # -144: the arrow's joint on the goat's centre
ON_SECTOR_DEG = 15.0  # within this of the goat's centre (the sector spans +-18)
ARROW_LEN, ARROW_H = 0.060, 0.010  # centre to tip; blade height over the face
HUB_R, HUB_H = 0.017, 0.018

PIVOT = (0.060, -(W / 2 + 0.0105), T / 2)  # the lever's pivot, beside the right edge near the roof
LEVER_REST = -30.0  # its direction at rest, degrees from +x: the smiley head stands past the roof's right corner
LEVER_LEN = 0.080  # pivot to the paddle's centre
STROKE = float(os.environ.get("SS_STROKE", "75.0"))  # estimate, bound 60 to 90 (README: protractor on the real toy)
TRIGGER = 0.9 * STROKE  # 67.5: the toy plays once the lever passes this
HOME_DEG = 5.0  # back home within this
LEVER_N = float(os.environ.get("SS_LEVER_N", "4.4"))  # N at the head at the trigger: estimate, bound 4 to 8 (README: read on a Pesola 10 N spring scale)
LEVER_K = LEVER_N * 0.080 / math.radians(TRIGGER)  # N m/rad about the pivot, 80 mm to the head: 0.30 at 4.4 N, 75 degrees
TOY_MU = float(os.environ.get("SS_MU", "0.4"))  # toy on the table, static and dynamic: estimate, bound 0.3 to 0.5 (README: spring-scale drag test)
DETENT_NM = float(os.environ.get("SS_DETENT", "0.02"))  # the arrow's detents as joint friction: estimate, bound 0.01 to 0.04 N m

MARK_MM, MARK_DEG = 30.0, 10.0  # the toy within this of its tape marks
CLEAR_M = 0.05
HOLD_STEPS = 40  # 2 s at 20 Hz

LOOKS = {"barn": m.Look(m.rgb(0.80, 0.10, 0.08), roughness=0.5), "face": m.Look(m.rgb(0.96, 0.95, 0.90), roughness=0.5),
         "roof": m.Look(m.rgb(0.95, 0.75, 0.10), roughness=0.5), "arrow": m.Look(m.rgb(0.98, 0.80, 0.05), roughness=0.4),
         "lever": m.Look(m.rgb(0.98, 0.80, 0.05), roughness=0.4),
         "dial": m.Look(roughness=0.6, texture=str(pathlib.Path(__file__).resolve().parent / "dial.png"))}
DIAL_IMG_R = 0.084  # m: the dial picture's half width (dial.png spans the rim)


def _dial_face() -> m.Mesh:
    """The dial's picture face: a flat disc carrying dial.png, the ten picture tiles in the toy's order (image up is the roof)."""
    n = 72
    ang = np.radians(np.arange(n) * 360.0 / n)
    ring = np.stack([DIAL_C[0] + DIAL_IMG_R * np.cos(ang), DIAL_C[1] + DIAL_IMG_R * np.sin(ang), np.full(n, T + 0.0012)], 1)
    v = np.vstack([[DIAL_C[0], DIAL_C[1], T + 0.0012], ring])
    uv = np.stack([0.5 - (v[:, 1] - DIAL_C[1]) / (2 * DIAL_IMG_R), 0.5 + (v[:, 0] - DIAL_C[0]) / (2 * DIAL_IMG_R)], 1)
    f = np.array([[0, 1 + k, 1 + (k + 1) % n] for k in range(n)])
    return m.Mesh(v, f, uv)


def _toy() -> m.Part:
    outline = [(-L / 2, -W / 2), (EAVE_X, -W / 2), (L / 2, 0.0), (EAVE_X, W / 2), (-L / 2, W / 2)]
    body = m.prism(outline, 0.0, T)
    roof = m.prism([(EAVE_X - 0.004, -W / 2 + 0.006), (L / 2 - 0.008, 0.0), (EAVE_X - 0.004, W / 2 - 0.006)], T, T + 0.0015)
    parts = [("body", body, "barn"), ("roof", roof, "roof"), ("rim", m.cylinder(DIAL_R + 0.008, 0.0008, (DIAL_C[0], DIAL_C[1], T + 0.0004)), "roof")]
    parts.append(("dial", _dial_face(), "dial"))  # the ten picture tiles (dial.png: Twemoji icons, CC BY 4.0)
    parts.append(("boss", m.cylinder(0.009, T * 0.7, (PIVOT[0], PIVOT[1] + 0.004, T / 2)), "lever"))
    cols = [m.col_hull("body_c", body)]
    inertia = (MASS * (W * W + T * T) / 12, MASS * (L * L + T * T) / 12, MASS * (L * L + W * W) / 12)
    # the arrow: a hub and a pointed blade, turning freely about the dial's centre (light detents: joint friction)
    a = math.radians(ARROW_START)
    u, v = np.array([math.cos(a), math.sin(a)]), np.array([-math.sin(a), math.cos(a)])
    c0 = np.array(DIAL_C)
    blade = m.prism([tuple(c0 - 0.012 * u - 0.007 * v), tuple(c0 + (ARROW_LEN - 0.012) * u - 0.007 * v), tuple(c0 + ARROW_LEN * u),
                     tuple(c0 + (ARROW_LEN - 0.012) * u + 0.007 * v), tuple(c0 - 0.012 * u + 0.007 * v)], T + 0.0025, T + 0.0025 + ARROW_H)
    hub = m.cylinder(HUB_R, HUB_H, (DIAL_C[0], DIAL_C[1], T + 0.0025 + HUB_H / 2))
    arrow = m.hinge("arrow", "Z", (DIAL_C[0], DIAL_C[1], T), -400.0, 400.0, [("blade", blade, "arrow"), ("hub", hub, "arrow")],
                    [m.col_hull("blade_c", blade), m.col_hull("hub_c", hub)], mass=0.015, damping=0.01, friction=DETENT_NM, spring=0.0)
    # the lever: an arm from the pivot and a paddle at its end, sprung home
    b = math.radians(LEVER_REST)
    e = np.array([math.cos(b), math.sin(b), 0.0])
    p = np.array(PIVOT)
    arm = m.box((0.060, 0.012, 0.014), (0.030, 0.0, 0.0)).turned(LEVER_REST, "z").moved(p)
    paddle = m.cylinder(0.017, 0.020, tuple(p + LEVER_LEN * e))  # the round smiley head
    lever = m.hinge("lever", "Z", tuple(p), -STROKE, 0.0, [("arm", arm, "lever"), ("paddle", paddle, "lever")], [m.col_hull("arm_c", arm), m.col_hull("paddle_c", paddle)],
                    mass=0.02, spring=LEVER_K, damping=0.004, friction=0.002)
    return m.mechanism("toy", (TOY_XY[0], TOY_XY[1], TOP + 0.0005), [arrow, lever], LOOKS, frame_parts=parts, frame_colliders=cols, free_mass=MASS - 0.035,
                       friction=(TOY_MU, TOY_MU), solver_iters=64, frame_inertia=inertia)


OBJECTS = [_toy()]


# -- scoring --------------------------------------------------------------------------------------------------------------------------------


def gripper_points(env) -> np.ndarray:
    """Points over the whole of both grippers (fingers, palm, housing, and the wrist link's end), station frame, at the current opening."""
    base = env.unwrapped
    origin = m._np(base.scene.env_origins[0])
    out = []
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        i = robot.find_bodies("gripper")[0][0]
        pose = m._np(robot.data.body_link_pose_w)[0, i]
        R = m.quat_matrix(pose[3:7])
        opening = abs(float(m._np(robot.data.joint_pos)[0, robot.find_joints("joint7")[0][0]]))
        pts = []
        for sx in (-1.0, 1.0):
            for dy in np.linspace(-0.02, 0.02, 5):
                for dz in np.linspace(0.095, 0.152, 4):
                    for t in (0.0, 0.009):
                        pts.append((sx * (opening + 0.0045 + t), dy, dz))
        for x in np.linspace(-0.05, 0.05, 6):
            for y in np.linspace(-0.03, 0.03, 4):
                for z in np.linspace(-0.01, 0.095, 6):
                    pts.append((x, y, z))
        out.append(pose[:3] - origin + np.asarray(pts) @ R.T)
    return np.vstack(out)


def toy_gap(toy, pts) -> float:
    """Least distance from any gripper point to the toy: its body (as its bounding box), the arrow's hub and blade height above the face, and the lever."""
    loc = toy.to_local(pts)
    lo, hi = np.array([-L / 2, -W / 2, 0.0]), np.array([L / 2, W / 2, T + 0.0025 + HUB_H])
    body = np.linalg.norm(loc - np.clip(loc, lo, hi), axis=1)
    ang = math.radians(LEVER_REST + math.degrees(toy.joint("lever")))
    a = np.array(PIVOT)
    bb = a + (LEVER_LEN + 0.015) * np.array([math.cos(ang), math.sin(ang), 0.0])
    s = np.clip(((loc - a) @ (bb - a)) / float((bb - a) @ (bb - a)), 0.0, 1.0)
    lev = np.linalg.norm(loc - (a + s[:, None] * (bb - a)), axis=1) - 0.012
    return float(min(body.min(), lev.min()))


def _wrap(x: float) -> float:
    return (x + 180.0) % 360.0 - 180.0


def judge(mem: dict, face_up: bool, on_table: bool, off_mm: float, yaw_deg: float, arrow_deg: float, lever_deg: float, gap_m: float) -> float:
    """One step of the scorer, from the state now; `mem` carries the trigger event (`played`), the void and the hold count across steps.
    0.33: the arrow within 15 degrees of the goat's centre. 0.67: the lever has passed its trigger at a moment when the arrow was on the goat and the toy face
    up on its marks (the toy plays the goat), and the arrow is still on the goat. 1.0: as 0.67, and now the lever is home, the arrow on the goat, the toy on its
    marks and every gripper point 50 mm clear, all held 2 s. Void (0, for the rest of the run): the toy tipped more than 25 degrees from face up, or off the table."""
    on_marks = face_up and off_mm <= MARK_MM and abs(_wrap(yaw_deg)) <= MARK_DEG
    on_goat = abs(_wrap(arrow_deg - GOAT_JOINT)) <= ON_SECTOR_DEG
    if lever_deg >= TRIGGER and on_goat and on_marks:
        mem["played"] = True
    if not face_up or not on_table:
        mem["void"] = True
    settled = mem.get("played", False) and lever_deg <= HOME_DEG and on_goat and on_marks and gap_m >= CLEAR_M
    mem["good"] = mem.get("good", 0) + 1 if settled else 0
    if mem.get("void"):
        return 0.0
    if mem.get("played") and mem["good"] >= HOLD_STEPS:
        return 1.0
    if mem.get("played") and on_goat:
        return 0.67
    return 0.33 if on_goat else 0.0


class Trial:
    """Reads the toy every step and passes it to `judge`."""

    def reset(self, env=None):
        self.n = 0
        self.mem = {}
        self.tier = 0.0
        self.last = {}

    def step(self, env):
        toy = m.state(env, "toy")
        self.n += 1
        R = toy.rotation()
        face_up = bool(R[2, 2] > 0.9)  # within about 25.8 degrees of face up
        on_table = bool(toy.pos[2] > TOP - 0.02)
        yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
        off_mm = 1000.0 * float(np.hypot(toy.pos[0] - TOY_XY[0], toy.pos[1] - TOY_XY[1]))
        arrow = math.degrees(toy.joint("arrow"))
        lever = -math.degrees(toy.joint("lever"))
        gap = toy_gap(toy, gripper_points(env))
        self.tier = judge(self.mem, face_up, on_table, off_mm, yaw, arrow, lever, gap)
        self.last = {"n": self.n, "tier": self.tier, "arrow": round(arrow, 1), "lever": round(lever, 1), "played": bool(self.mem.get("played")),
                     "off_mm": round(off_mm, 1), "yaw": round(yaw, 1), "gap_mm": round(gap * 1000, 1), "void": bool(self.mem.get("void"))}
        if self.n % 20 == 0:
            print("[ss] " + json.dumps(self.last), flush=True)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects) -> float:
    return TRIAL.tier


def success(objects) -> bool:
    return TRIAL.tier >= 1.0


# -- the scripted demo ----------------------------------------------------------------------------------------------------------------------

PIN = (float(os.environ.get("SS_PINX", "0.0")), float(os.environ.get("SS_PINY", "0.088")))  # where the left hand presses, toy frame: the left edge beside the dial
PIN_N = float(os.environ.get("SS_PIN_N", "25.0"))
OVER = float(os.environ.get("SS_OVER", "12.0"))  # degrees the hand swings past the stop
GYAW = float(os.environ.get("SS_GYAW", "30.0"))  # the right hand's yaw on the round head, degrees off across-the-lever
FING_R = float(os.environ.get("SS_FR", "0.040"))  # the right fingertips' radius from the dial's centre while turning the arrow
LEAD = float(os.environ.get("SS_LEAD", "10.0"))  # degrees ahead of the arrow the fingertips aim (contact sits about 20 degrees off)
FING_Z = float(os.environ.get("SS_FZ", "0.004"))  # fingertips' underside over the face
PULL_Z = float(os.environ.get("SS_PZ", "0.012"))  # fingertips' underside over the table at the lever's head (the head spans 15.5 to 35.5 mm)


def demo_script(ctx):
    """The left hand turns the arrow, then pins the toy; the right pulls the lever. Modes: full. none: the arms wait. cat: the lever pulled with the arrow left on the cat. no_pin: the arrow turned and the lever pulled with no hand on the toy.
    spin: the arrow pushed fast through the goat and past. half: the lever pulled to half its stroke."""
    mode = ctx.mode
    if mode not in ("full", "none", "cat", "no_pin", "spin", "half"):
        raise ValueError(f"unknown mode {mode!r}")
    Lf, Rt = ctx.left, ctx.right
    if mode == "none":
        yield from Lf.wait(3.0)
        return
    env = ctx.emb._env
    toy = lambda: ctx.state("toy")

    def world(px, py, pz):
        t = toy()
        return np.asarray(t.pos, float) + t.rotation() @ np.array([px, py, pz])

    def tips(side):
        t = m.fingertips(env)[12:] if side == "right" else m.fingertips(env)[:12]
        return np.array([t[:, 0].mean(), t[:, 1].mean(), t[:, 2].min()])

    rtip, ltip = (lambda: tips("right")), (lambda: tips("left"))
    if mode != "cat":
        # the arrow, by the left hand: the closed fingertips behind its blade, pushing it clockwise round the dial until it points at the goat
        def arrow_deg():
            return ctx.deg("toy", "arrow")

        def on_dial(deg, r=FING_R, z=FING_Z):
            a = math.radians(ARROW_START + deg)
            return world(DIAL_C[0] + r * math.cos(a), DIAL_C[1] + r * math.sin(a), T + z)

        start = on_dial(arrow_deg() + 32.0)
        yield from Lf.move(start + [0.0, 0.0, 0.05], grip=d.down(0.0), finger=d.FINGER_CLOSED, label="left over the dial")
        yield from Lf.servo(start, tol=0.003, timeout=3.0, read=ltip, label="left fingertips down on the dial behind the arrow")
        off = Lf.tcp() - ltip()
        goal = GOAT_JOINT - (40.0 if mode == "spin" else 0.0)
        speed = 0.12 if mode == "spin" else 0.03
        yield from Lf.follow(lambda: on_dial(arrow_deg() + LEAD) + off, t=12.0, speed=speed, until=lambda: arrow_deg() <= goal + 4.0, label="left pushes the arrow round")
        yield from Lf.move(on_dial(arrow_deg() + 40.0) + off, t=0.5, label="left fingertips back off the blade")
        print(f"[ss] arrow at {arrow_deg():.1f} deg (goat {GOAT_JOINT:.0f})", flush=True)
        yield from Lf.move(Lf.at() + [0.0, 0.0, 0.04], t=0.6, label="left up off the arrow")
    if mode != "no_pin":
        pin = world(PIN[0], PIN[1], T)
        yield from Lf.move(pin + [0.0, 0.0, 0.05], grip=d.down(-90.0), finger=d.FINGER_CLOSED, label="left over the toy's left edge")
        yield from Lf.move(pin + [0.0, 0.0, 0.025], t=0.8, label="left down")
        res = yield from Lf.press((0.0, 0.0, -1.0), PIN_N, 0.05, settle=0.4, label="left presses the toy flat")
        print(f"[ss] pin {res}", flush=True)
    # the lever, by the right hand: grip its round head and swing it about its pivot toward the robot, past the trigger

    def lever_deg():
        return ctx.deg("toy", "lever")

    def head(deg, z=PULL_Z):
        a = math.radians(LEVER_REST + deg)
        p = np.array(PIVOT[:2]) + LEVER_LEN * np.array([math.cos(a), math.sin(a)])
        return world(p[0], p[1], z)

    h0 = head(lever_deg())
    yield from Rt.move(h0 + [0.0, 0.0, 0.07], grip=d.down(LEVER_REST + GYAW), finger=d.FINGER_OPEN, label="right over the lever's head")
    yield from Rt.servo(h0, tol=0.003, timeout=3.0, read=rtip, label="right fingers down round the head")
    yield from Rt.close(0.034, 25.0, label="right grips the head")
    pull = (-0.5 * STROKE if mode == "half" else -(STROKE + OVER)) - lever_deg()  # past the stop: the arm lags and the grip gives a little
    yield from Rt.arc(world(*PIVOT), (0.0, 0.0, 1.0), pull, t=2.5, label="right pulls the lever round")
    print(f"[ss] lever at {lever_deg():.1f} deg", flush=True)
    yield from Rt.wait(0.3)
    yield from Rt.open(label="right lets go; the lever springs home")
    yield from Rt.move(Rt.at() + [0.0, 0.0, 0.06], t=0.5, label="right up")
    yield from Lf.move(Lf.at() + [0.0, 0.0, 0.08], t=0.8, label="left up off the toy")
    yield from ctx.together(Lf.move((0.15, 0.28, TOP + 0.22), t=1.5, label="left clear"), Rt.move((0.15, -0.28, TOP + 0.22), t=1.5, label="right clear"))
    yield from Lf.wait(3.5)
    print("[ss] END " + json.dumps({"mode": mode, **TRIAL.last}), flush=True)
