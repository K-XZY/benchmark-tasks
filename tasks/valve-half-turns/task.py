"""On the station table: a brass gate valve on a short pipe, with a red 6 cm handwheel on top, a white mark on one spoke and a black knob on the rim that spins freely. It is open by a whole number of turns, and its stem does not rise. The wheel closes clockwise, seen from above, until it stops.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 --seed 1 --instruction "<INSTRUCTION below>"
The demo:  python demokit.py task.py OUT --seed 1
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import mechkit as m  # noqa: E402  (hooks yam_arena so that the parts below spawn from USD)
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Close the valve fully, counting the turns as you go, then open it again by exactly half that number of turns, and let go."
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

VX, VY = 0.30, -0.12
WHEEL_Z, WHEEL_R, RIM_R = 0.09, 0.03, 0.004
KNOB_AT = -90.0  # the knob's direction from the wheel's axis at a whole number of turns, in degrees from +x: towards the right arm
KNOB_R, KNOB_Z0, KNOB_Z1 = 0.008, 0.094, 0.13
MAX_TURNS = 5.0
WINDOW, CLOSED_WITHIN, LEFT_CLOSED_WITHIN = 0.2, 0.02, 0.05  # turns
CLEAR = 0.03

LOOKS = {
    "wheel": m.Look(color=m.rgb(0.75, 0.08, 0.06), roughness=0.5),
    "mark": m.Look(color=m.rgb(0.97, 0.97, 0.97), roughness=0.5, emissive=(0.2, 0.2, 0.2)),
    "knob": m.Look(color=m.rgb(0.04, 0.04, 0.05), roughness=0.5),
    "brass": m.Look(color=m.rgb(0.72, 0.56, 0.25), metallic=0.7, roughness=0.45),
    "pipe": m.Look(color=m.rgb(0.80, 0.80, 0.78), roughness=0.4),
    "board": m.Look(color=m.rgb(0.45, 0.45, 0.47), roughness=0.7),
}


def start_turns(seed: int | None) -> int:
    return 2 + (0 if seed is None else seed) % 3


def _valve_part():
    n = 32
    rim_pts = [(WHEEL_R * math.cos(a), WHEEL_R * math.sin(a), WHEEL_Z) for a in np.linspace(0.0, 2.0 * math.pi, n + 1)]
    rim = m.tube(rim_pts, RIM_R, sides=8)
    hub = m.cylinder(0.009, 0.012, (0.0, 0.0, WHEEL_Z), segments=16)
    a = math.radians(KNOB_AT)

    def spoke(deg: float, half: float = 0.003):
        c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
        return m.box((WHEEL_R, 2.0 * half, 0.005), (0.0, 0.0, 0.0)).turned(deg, "z").moved((0.5 * WHEEL_R * c, 0.5 * WHEEL_R * s, WHEEL_Z))

    spokes = m.join(spoke(KNOB_AT + 120.0), spoke(KNOB_AT + 240.0))
    mark = spoke(KNOB_AT, 0.0045).moved((0.0, 0.0, 0.0006))
    wheel = m.hinge("wheel", "Z", (0.0, 0.0, WHEEL_Z), 0.0, MAX_TURNS * 360.0, [("rim", rim, "wheel"), ("hub", hub, "wheel"), ("spokes", spokes, "wheel"), ("mark", mark, "mark")],
                    [m.col_cylinder("wheel_c", WHEEL_R + RIM_R, 2.0 * RIM_R, (0.0, 0.0, WHEEL_Z))], mass=0.1, damping=0.05, friction=0.03, armature=0.001, effort=50.0)
    kx, ky = WHEEL_R * math.cos(a), WHEEL_R * math.sin(a)
    body = m.cylinder(KNOB_R, KNOB_Z1 - KNOB_Z0, (kx, ky, (KNOB_Z0 + KNOB_Z1) / 2), segments=16)
    knob = m.hinge("knob", "Z", (kx, ky, WHEEL_Z), -36000.0, 36000.0, [("knob", body, "knob")], [m.col_cylinder("knob_c", KNOB_R, KNOB_Z1 - KNOB_Z0, (kx, ky, (KNOB_Z0 + KNOB_Z1) / 2))], parent="wheel", mass=0.02, damping=0.0005, friction=0.0, armature=1e-4, effort=5.0)
    frame_parts = [
        ("board", m.box((0.12, 0.22, 0.008), (0.0, 0.0, 0.004)), "board"),
        ("pipe", m.cylinder(0.012, 0.20, (0.0, 0.0, 0.032), axis="y", segments=20), "pipe"),
        ("body", m.cylinder(0.021, 0.05, (0.0, 0.0, 0.032), axis="y", segments=20), "brass"),
        ("bonnet", m.cylinder(0.014, 0.03, (0.0, 0.0, 0.058), segments=16), "brass"),
        ("stem", m.cylinder(0.004, 0.02, (0.0, 0.0, 0.08), segments=12), "brass"),
    ]
    frame_cols = [m.col_box("board_c", (0.12, 0.22, 0.008), (0.0, 0.0, 0.004)), m.col_box("body_c", (0.042, 0.20, 0.046), (0.0, 0.0, 0.031))]
    return m.mechanism("valve", (VX, VY, TOP), [wheel, knob], LOOKS, frame_parts=frame_parts, frame_colliders=frame_cols, solver_iters=32)


OBJECTS = [_valve_part()]


def gap_to_wheel(points: np.ndarray, knob: np.ndarray) -> float:
    """The least distance from these world points to the whole wheel: rim, spokes and hub within its disc, and the knob."""
    rel = points - np.array([VX, VY, TOP + WHEEL_Z])
    radial = np.maximum(np.hypot(rel[:, 0], rel[:, 1]) - (WHEEL_R + RIM_R), 0.0)
    height = np.maximum(np.abs(rel[:, 2]) - RIM_R, 0.0)
    return min(float(np.hypot(radial, height).min()), float(np.linalg.norm(points - knob, axis=1).min()) - KNOB_R)


def end_ok_now(gap: float, spin: float) -> bool:
    """Every finger clear of the wheel, and the wheel not turning."""
    return bool(gap >= CLEAR and spin < 0.05)


def task_score(closed_first: bool, turns_open: float, start: int, end_ok: bool) -> float:
    """The scoring text, line by line. `turns_open` from closed, `start` the turns open at the start, `end_ok` every finger clear and the wheel still for 1 s."""
    if not closed_first or turns_open <= LEFT_CLOSED_WITHIN:
        return 0.0
    if abs(turns_open - start / 2.0) <= WINDOW and end_ok:
        return 1.0
    return 0.25


class Trial:
    """The turns the wheel stands from closed, and the trial's latches."""

    def __init__(self) -> None:
        self.seed: int | None = None
        self.reset()

    def reset(self, env=None) -> None:
        self.steps = 0
        self.start = start_turns(self.seed)
        self.closed_first = False
        self.held = 0
        self.ready = False
        self.full = False
        self.score = 0.0
        self.max_turns = float(self.start)
        self.min_turns = float(self.start)
        self.report: dict = {}
        if env is not None:
            asset = env.unwrapped.scene["valve"]
            j = asset.find_joints("wheel_joint")[0][0]
            pos = asset.data.default_joint_pos.torch.clone()
            pos[:, j] = self.start * 2.0 * math.pi
            asset.write_joint_position_to_sim_index(position=pos)
            asset.write_joint_velocity_to_sim_index(velocity=asset.data.default_joint_vel.torch.clone() * 0.0)

    def step(self, env) -> None:
        dt = float(env.unwrapped.step_dt)
        self.steps += 1
        valve = m.state(env, "valve")
        turns = valve.joint("wheel") / (2.0 * math.pi)
        if turns <= CLOSED_WITHIN:
            self.closed_first = True
        self.max_turns, self.min_turns = max(self.max_turns, turns), min(self.min_turns, turns)
        pts = m.fingertips(env)
        a = math.radians(KNOB_AT) + valve.joint("wheel")
        knob = np.array([VX + WHEEL_R * math.cos(a), VY + WHEEL_R * math.sin(a), TOP + 0.5 * (KNOB_Z0 + KNOB_Z1)])
        gap = gap_to_wheel(pts, knob)
        spin = abs(float(m._np(env.unwrapped.scene["valve"].data.joint_vel)[0, env.unwrapped.scene["valve"].find_joints("wheel_joint")[0][0]]))
        self.ready = end_ok_now(gap, spin)
        self.held = self.held + 1 if self.ready else 0
        end_ok = self.held * dt >= SUCCESS_HOLD_S
        self.full = bool(self.ready and task_score(self.closed_first, turns, self.start, True) == 1.0)
        self.score = task_score(self.closed_first, turns, self.start, end_ok)
        self.report = {"t": round(self.steps * dt, 2), "seed": self.seed, "start_turns": self.start, "turns_open": round(turns, 3), "closed_first": self.closed_first, "min_turns": round(self.min_turns, 3), "max_turns": round(self.max_turns, 3), "gap_m": round(gap, 3), "spin_rad_s": round(spin, 3), "ready": self.ready, "score": self.score}
        if self.steps % 20 == 0:
            out = os.environ.get("VALVE_SCORE_DIR", "valve_scores")
            os.makedirs(out, exist_ok=True)
            with open(os.path.join(out, f"{os.environ.get('TASK_DEMO', 'run')}_seed{self.seed}.json"), "w") as f:
                json.dump(self.report, f, indent=0)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def _read_seed(module) -> None:
    """The embodiment's reset hands its seed to nothing a task file can read, so note it here. Only the start opening uses it."""
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_valve_seed", False):
        return
    plain = cls.reset

    def reset(self, scene, *, seed=None):
        TRIAL.seed = seed
        return plain(self, scene, seed=seed)

    cls.reset = reset
    cls._valve_seed = True


if "yam_arena.inspect_embodiment" in sys.modules:
    _read_seed(sys.modules["yam_arena.inspect_embodiment"])


def success(objects) -> bool:
    return TRIAL.full


def score(objects) -> float:
    """See `task_score`."""
    return TRIAL.score


# -- the demo ------------------------------------------------------------------------------

LEAD_DEG = 30.0
STALL_S = 0.6


def demo_script(ctx):
    """Closed loop on the wheel's angle modulo one turn, as a camera could read its mark: it never reads how far open the valve is. Closed is found when the wheel stops turning. `TASK_DEMO`: full only; the score lines are checked in check_scorer.py."""
    import demokit as d
    from yam_arena.constants import FINGER_OPEN

    r = ctx.right
    grip = d.down(0.0)
    centre = np.array([VX, VY, TOP + 0.5 * (KNOB_Z0 + KNOB_Z1)])
    up = np.array([0.0, 0.0, 1.0])

    def seen() -> float:
        """The wheel's angle as its mark shows it: degrees, modulo one turn."""
        return ctx.deg("valve", "wheel") % 360.0

    def rim(deg: float) -> np.ndarray:
        a = math.radians(KNOB_AT + deg)
        return centre + np.array([WHEEL_R * math.cos(a), WHEEL_R * math.sin(a), 0.0])

    knob = rim(seen())
    yield from r.move(knob + 0.08 * up, grip=grip, finger=FINGER_OPEN, t=3.0, label="over the knob")
    yield from r.servo(rim(seen()) + 0.08 * up, timeout=1.0, label="over the knob")
    yield from r.move(rim(seen()), t=1.2, label="down to the knob")
    yield from r.servo(rim(seen()), timeout=1.0, label="down to the knob")
    yield from r.close(2.0 * KNOB_R, 15.0, t=0.4, label="pinch the knob")
    last = seen()
    track = {"angle": last, "moved": 0.0, "still": 0.0}

    def update() -> None:
        now = seen()
        step = (now - track["angle"] + 180.0) % 360.0 - 180.0
        track["angle"] = now
        track["moved"] += step
        track["still"] = track["still"] + d.DT if abs(step) < 0.3 else 0.0

    def closing():
        update()
        return rim(track["angle"] - LEAD_DEG)

    yield from r.follow(closing, t=20.0, speed=0.10, until=lambda: track["still"] >= STALL_S and track["moved"] < -10.0, label="close")
    turns = -track["moved"] / 360.0
    print(f"[valve] the wheel stopped after {turns:.2f} turns", flush=True)
    goal = track["moved"] + 0.5 * -track["moved"]
    track["still"] = 0.0

    def opening():
        update()
        return rim(track["angle"] + LEAD_DEG * 0.6)

    yield from r.follow(opening, t=15.0, speed=0.10, until=lambda: goal - track["moved"] < 12.0, label="open by half")
    print(f"[valve] opened {(track['moved'] + turns * 360.0) / 360.0:.3f} turns", flush=True)
    yield from r.open(t=0.4, label="let go")
    yield from r.move(r.at() + 0.10 * up, t=1.0, label="rise")
