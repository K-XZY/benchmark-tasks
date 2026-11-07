"""A camera on a moving head, panned by a long bar, follows a small figure riding a belt rail 50 cm ahead. The belt's moves and stops change by seed, so no planned path fits. When a red lamp comes on, a button out of the bar's arm's reach must be pressed within 1 s, with the camera still on the figure.

Seed: the environment variable TASK_SEED, default 1. In debug runs, a mode may name its own: TASK_DEMO=full@3.

Debug:  python demokit.py task.py OUT --no-record --stills 40 --mode full@1,full@2,sweep@1,...
Final:  python demokit.py task.py OUT
Model:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1000 --instruction "<INSTRUCTION below>"
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
INSTRUCTION = "Hold the bar of the moving head and turn it so that the camera stays on the figure as it moves along its rail. When the red lamp comes on, press the stop button within 1 s while keeping the camera on the figure. When the figure has stopped, let go of the bar."
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
TOP = d.TABLE_TOP
DT = d.DT

LOOKS = {
    "black": m.Look(m.rgb(0.08, 0.08, 0.09), roughness=0.6),
    "metal": m.Look(m.rgb(0.45, 0.47, 0.50), metallic=0.7, roughness=0.4),
    "line": m.Look(m.rgb(1.0, 0.85, 0.05), emissive=(1.2, 0.9, 0.05), roughness=0.5),
    "rail": m.Look(m.rgb(0.35, 0.37, 0.40), roughness=0.6),
    "belt": m.Look(m.rgb(0.10, 0.10, 0.11), roughness=0.8),
    "figure": m.Look(m.rgb(0.10, 0.45, 0.90), roughness=0.5),
    "trousers": m.Look(m.rgb(0.12, 0.13, 0.20), roughness=0.7),
    "face": m.Look(m.rgb(0.95, 0.75, 0.60), roughness=0.5),
    "hair": m.Look(m.rgb(0.20, 0.12, 0.07), roughness=0.7),
    "eye": m.Look(m.rgb(0.03, 0.03, 0.04), roughness=0.3),
    "mouth": m.Look(m.rgb(0.70, 0.10, 0.10), roughness=0.5),
    "lamp_off": m.Look(m.rgb(0.25, 0.04, 0.04), roughness=0.3),
    "lamp_on": m.Look(m.rgb(1.0, 0.0, 0.0), emissive=(2.5, 0.0, 0.0)),
    "stand": m.Look(m.rgb(0.25, 0.26, 0.28), roughness=0.6),
    "button": m.Look(m.rgb(0.85, 0.08, 0.06), roughness=0.4),
}

HEAD = (0.34, 0.0, TOP)
PAN_Z, BAR_Z = 0.10, 0.125
BAR_L, GRIP_R = 0.30, 0.22
PAN_LIMIT = 36.0
AHEAD = 0.50
RAIL_X = HEAD[0] + AHEAD
RAIL_Y = 0.30
START_S = 5.0
LONG_S = 30.0
WINDOW_DEG = 3.0
GOOD_OUT_S, FAIR_OUT_S = 6.0, 12.0  # seconds with the figure outside the band, from the belt's start to the lamp
LAMP_AT = (0.68, 0.38, TOP)
BUTTON_AT = (0.30, 0.45, TOP)
BUTTON_BASE, BUTTON_H, BUTTON_TRAVEL = 0.05, 0.015, 0.008
PRESSED_AT = 0.005
CLEAR = 0.03
_PRESS_ABOVE = 0.02


def schedule(seed: int) -> tuple[np.ndarray, float]:
    """The belt's corner points (t, y), and the lamp's time. Moves of 8 to 15 cm/s, stops of 0.8 to 2.2 s. The lamp comes on just after a move starts, about 30 s in."""
    rng = np.random.default_rng(seed)
    t, y, pts, lamp = START_S, 0.0, [(START_S, 0.0)], None
    while True:
        for _ in range(100):
            ny = float(rng.uniform(-RAIL_Y, RAIL_Y))
            if abs(ny - y) >= 0.12:
                break
        dt = abs(ny - y) / float(rng.uniform(0.08, 0.15))
        if t - START_S >= LONG_S - 4.0:
            lamp = t + min(0.6, 0.3 * dt)
        t, y = t + dt, ny
        pts.append((t, y))
        if lamp is not None:
            return np.array(pts), float(lamp)
        t += float(rng.uniform(0.8, 2.2))
        pts.append((t, y))


def belt_y(pts: np.ndarray, t: float) -> float:
    i = int(np.searchsorted(pts[:, 0], t)) - 1
    if i < 0:
        return float(pts[0, 1])
    if i >= len(pts) - 1:
        return float(pts[-1, 1])
    a, b = pts[i], pts[i + 1]
    s = (t - a[0]) / (b[0] - a[0])
    return float(a[1] + (b[1] - a[1]) * s * s * (3.0 - 2.0 * s))


def _head() -> m.Part:
    bar_x = -BAR_L / 2
    pan = [
        ("deck", m.cylinder(0.035, 0.015, (0, 0, PAN_Z + 0.0075)), "metal"),
        ("bar", m.cylinder(0.009, BAR_L, (bar_x, 0, BAR_Z), axis="x"), "metal"),
        ("bar_line", m.box((BAR_L, 0.006, 0.002), (bar_x, 0, BAR_Z + 0.0095)), "line"),
        ("grip", m.cylinder(0.014, 0.10, (-GRIP_R, 0, BAR_Z), axis="x"), "black"),
        ("camera", m.box((0.14, 0.06, 0.07), (0.07, 0, BAR_Z + 0.035)), "black"),
        ("lens", m.cylinder(0.024, 0.06, (0.17, 0, BAR_Z + 0.035), axis="x"), "black"),
        ("camera_line", m.box((0.20, 0.006, 0.002), (0.10, 0, BAR_Z + 0.072)), "line"),
    ]
    cols = [m.col_box("grip_c", (0.10, 0.028, 0.028), (-GRIP_R, 0, BAR_Z)), m.col_box("bar_c", (BAR_L - 0.10, 0.018, 0.018), ((-BAR_L + 0.10) / 2, 0, BAR_Z)), m.col_box("camera_c", (0.20, 0.06, 0.07), (0.10, 0, BAR_Z + 0.035))]
    link = m.hinge("pan", "Z", (0, 0, 0), -PAN_LIMIT, PAN_LIMIT, pan, cols, mass=0.4, damping=0.6, friction=0.05, armature=0.01)
    column = [("foot", m.cylinder(0.07, 0.012, (0, 0, 0.006)), "black"), ("column", m.cylinder(0.02, PAN_Z - 0.012, (0, 0, (PAN_Z + 0.012) / 2)), "metal")]
    return m.mechanism("head", HEAD, [link], LOOKS, frame_parts=column, frame_colliders=[m.col_cylinder("foot_c", 0.07, 0.012, (0, 0, 0.006)), m.col_cylinder("column_c", 0.02, PAN_Z - 0.012, (0, 0, (PAN_Z + 0.012) / 2))])


def _belt() -> m.Part:
    figure = [
        ("carriage", m.box((0.05, 0.05, 0.012), (0, 0, 0.036)), "metal"),
        ("leg_a", m.cylinder(0.008, 0.04, (0, 0.011, 0.062)), "trousers"),
        ("leg_b", m.cylinder(0.008, 0.04, (0, -0.011, 0.062)), "trousers"),
        ("body", m.box((0.024, 0.042, 0.052), (0, 0, 0.108)), "figure"),
        ("arm_a", m.tube([(0, 0.026, 0.130), (-0.004, 0.033, 0.106), (-0.008, 0.034, 0.086)], 0.0055), "figure"),
        ("arm_b", m.tube([(0, -0.026, 0.130), (-0.004, -0.033, 0.106), (-0.008, -0.034, 0.086)], 0.0055), "figure"),
        ("hand_a", m.sphere(0.0065, (-0.008, 0.034, 0.083)), "face"),
        ("hand_b", m.sphere(0.0065, (-0.008, -0.034, 0.083)), "face"),
        ("neck", m.cylinder(0.006, 0.012, (0, 0, 0.139)), "face"),
        ("skull", m.sphere(0.019, (0, 0, 0.160)), "face"),
        ("hair", m.sphere(0.0195, (0.004, 0, 0.165)), "hair"),
        ("eye_a", m.sphere(0.0035, (-0.0165, 0.007, 0.163)), "eye"),
        ("eye_b", m.sphere(0.0035, (-0.0165, -0.007, 0.163)), "eye"),
        ("mouth", m.box((0.002, 0.011, 0.0025), (-0.018, 0, 0.152)), "mouth"),
    ]
    link = m.slider("carriage", "Y", -RAIL_Y - 0.01, RAIL_Y + 0.01, figure, [m.col_box("carriage_c", (0.05, 0.05, 0.14), (0, 0, 0.10))], mass=0.3, spring=4000.0, damping=200.0, effort=200.0, friction=0.0, armature=0.0)
    rail = [("rail", m.box((0.06, 2 * RAIL_Y + 0.08, 0.02), (0, 0, 0.01)), "rail"), ("belt", m.box((0.03, 2 * RAIL_Y + 0.06, 0.002), (0, 0, 0.021)), "belt")]
    return m.mechanism("belt", (RAIL_X, 0.0, TOP), [link], LOOKS, frame_parts=rail, frame_colliders=[m.col_box("rail_c", (0.06, 2 * RAIL_Y + 0.08, 0.02), (0, 0, 0.01))])


def _lamp() -> m.Part:
    parts = [
        ("foot", m.cylinder(0.03, 0.01, (0, 0, 0.005)), "stand"),
        ("post", m.cylinder(0.006, 0.11, (0, 0, 0.065)), "stand"),
        ("off", m.sphere(0.03, (0, 0, 0.14)), "lamp_off"),
        ("on", m.sphere(0.03, (0, 0, 0.14)), "lamp_on", True),
    ]
    return m.fixed("cue_lamp", LAMP_AT, parts, [m.col_cylinder("c", 0.03, 0.01, (0, 0, 0.005))], LOOKS)


def _button() -> m.Part:
    link = m.button("stop", (0, 0, BUTTON_BASE), "+Z", look="button", radius=0.028, height=BUTTON_H, travel=BUTTON_TRAVEL, spring=300.0, damping=2.0)
    return m.mechanism("button_unit", BUTTON_AT, [link], LOOKS, frame_parts=[("base", m.box((0.09, 0.09, BUTTON_BASE), (0, 0, BUTTON_BASE / 2)), "stand")], frame_colliders=[m.col_box("base_c", (0.09, 0.09, BUTTON_BASE), (0, 0, BUTTON_BASE / 2))])


OBJECTS = [_head(), _belt(), _lamp(), _button()]


def _seed() -> int:
    mode = os.environ.get("TASK_DEMO", "")
    if "@" in mode:
        return int(mode.split("@")[1])
    return int(os.environ.get("TASK_SEED", "1"))


def judge(outside_s: float, good_press: bool, lamp_passed: bool, clear: bool) -> tuple[bool, float]:
    """The score now: (full, score). `outside_s`: seconds with the figure outside the band, from the belt's start to the lamp. Run on hand-built states by check_score.py."""
    if not lamp_passed or outside_s > FAIR_OUT_S:
        return False, 0.0
    if outside_s > GOOD_OUT_S:
        return False, 0.5
    return bool(good_press and clear), 0.75


class Trial:
    """What the scorer holds across a trial."""

    def reset(self, env=None) -> None:
        self.pts, self.lamp_t = schedule(_seed())
        self.end_t = float(self.pts[-1, 0])
        self.t = 0.0
        self.outside_s = 0.0
        self.lamp_on = False
        self.pressed_t = None
        self.good_press = False
        self.stopped_at = None  # a press stops the recording, whenever it comes
        self.full, self.score = False, 0.0
        if env is not None:
            self.lamp_on = None
            self.step(env, count=False)

    def step(self, env, count: bool = True) -> None:
        if count:
            self.t += DT
            if os.environ.get("TASK_STEP_FILE"):  # the simulated time, for anything following the run from outside
                with open(os.environ["TASK_STEP_FILE"], "w") as f:
                    f.write(f"{self.t:.2f}")
        t = self.t
        m.set_target(env, "belt", "carriage", belt_y(self.pts, t))
        on = t >= self.lamp_t
        if on != self.lamp_on:
            self.lamp_on = on
            m.set_visible(f"{m.ENV_PATH}/cue_lamp/visual/on", on)
            m.set_visible(f"{m.ENV_PATH}/cue_lamp/visual/off", not on)
            if on and count:
                print(f"[spot] lamp on at {t:.2f} s; outside the band for {self.outside_s:.1f} s so far", flush=True)
        if not count:
            return
        pan = m.state(env, "head").deg("pan")
        y = m.state(env, "belt").joint("carriage")
        err = abs(pan - math.degrees(math.atan2(y, AHEAD)))
        if START_S <= t <= self.lamp_t and (self.stopped_at is not None or err > WINDOW_DEG):
            self.outside_s += DT  # after an early press, the rest of the time counts as missed
        pressed = m.state(env, "button_unit").joint("stop") <= -PRESSED_AT
        if pressed and self.pressed_t is None:
            self.pressed_t = self.stopped_at = t
            self.good_press = bool(self.lamp_t <= t <= self.lamp_t + 1.0 and err <= WINDOW_DEG)
            print(f"[spot] pressed at {t:.2f} s, {t - self.lamp_t:+.2f} s from the lamp, aim off by {err:.1f} deg: {'good' if self.good_press else 'no credit'}; outside the band for {self.outside_s:.1f} s", flush=True)
        self.full, self.score = judge(self.outside_s, self.good_press, t >= self.lamp_t, self._fingers_clear(env))

    def _fingers_clear(self, env) -> bool:
        tips = m.fingertips(env)
        a = math.radians(m.state(env, "head").deg("pan"))
        along = np.array([-math.cos(a), -math.sin(a), 0.0])
        start = np.array([HEAD[0], HEAD[1], TOP + BAR_Z])
        rel = tips - start
        s = np.clip(rel @ along, 0.0, BAR_L)
        bar = np.linalg.norm(rel - s[:, None] * along, axis=1).min() - 0.014
        button = np.linalg.norm(tips - (np.array(BUTTON_AT) + [0.0, 0.0, BUTTON_BASE + BUTTON_H]), axis=1).min() - 0.028
        return bool(min(bar, button) >= CLEAR)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    """The figure outside the band for at most 6 s, a good press, and the fingers clear of the bar and the button."""
    return TRIAL.full


def score(objects) -> float:
    return TRIAL.score


# -- the demo: reads the figure's position and the pan bar's angle, as a camera could see them, and the lamp from the top camera's picture --------------------------------------


def grip_at(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([HEAD[0] - GRIP_R * math.cos(a), HEAD[1] - GRIP_R * math.sin(a), TOP + BAR_Z])


def lamp_lit(ctx) -> bool:
    """Whether the top camera sees the lamp lit: a bright red pixel where the lamp stands."""
    r = ctx.see(np.array(LAMP_AT) + [0.0, 0.0, 0.14])[0]
    return bool(r[0] > 150.0 and r[0] > 1.5 * r[1])


def figure_y(ctx) -> float:
    return ctx.joint("belt", "carriage")


class Tracker:
    """Keeps the aim on the figure: reads it each step, leads it by its own speed, and servoes out the arm's lag from the bar's angle."""

    def __init__(self, ctx, arm, delay_s: float = 0.0):
        self.ctx, self.arm = ctx, arm
        self.last = figure_y(ctx)
        self.speed = 0.0
        self.trim = 0.0
        self.history: list[float] = []
        self.delay = round(delay_s / DT)
        self.want = 0.0

    def step(self, want_deg: float | None = None) -> None:
        y = figure_y(self.ctx)
        self.speed = 0.7 * self.speed + 0.3 * (y - self.last) / DT
        self.last = y
        self.history.append(y)
        seen = self.history[max(0, len(self.history) - 1 - self.delay)]
        if want_deg is None:
            want_deg = math.degrees(math.atan2(seen + (self.speed * 0.15 if not self.delay else 0.0), AHEAD))
        self.want = want_deg
        now = self.ctx.deg("head", "pan")
        self.trim = float(np.clip(self.trim + np.clip(0.08 * (want_deg - now), -0.3, 0.3), -8.0, 8.0))
        cmd = want_deg + self.trim
        self.arm._solve(grip_at(cmd), d.down(cmd).rot)


MODES = ("full", "none", "fixed", "sweep", "lag", "late", "early", "no_press", "dash", "left_dash", "hold_on")


def demo_script(ctx):
    """Modes, each with an optional @seed: full. none: the arms wait. fixed: holds the bar at the middle, with a good press. sweep: a steady planned sweep, with a good press. lag: tracks 1 s behind. late: presses 1.5 s after the lamp. early: presses before the lamp. no_press: tracks and never presses. dash: the right arm tracks alone, and at the lamp lets go and tries for the button. left_dash: the left arm tracks alone, and at the lamp lets go and presses. hold_on: never lets go of the bar."""
    mode = ctx.mode.split("@")[0]
    if mode not in MODES:
        raise ValueError(f"unknown mode {mode!r}")
    l, r = ctx.left, ctx.right
    if mode == "none":
        yield from l.wait(3.0)
        return
    button_top = np.array(BUTTON_AT) + [0.0, 0.0, BUTTON_BASE + BUTTON_H]
    over = button_top + [0.0, 0.0, 0.013 + _PRESS_ABOVE]  # closed fingertips this far above the button
    press = button_top + [0.0, 0.0, 0.013 - BUTTON_TRAVEL - 0.002]
    bar_arm, other = (l, r) if mode == "left_dash" else (r, l)
    uses_other = mode not in ("dash", "left_dash")
    grip0 = grip_at(0.0)

    def to_bar():
        yield from bar_arm.move(grip0 + [0.0, 0.0, 0.05], grip=d.down(0.0), finger=d.FINGER_OPEN, label="over the bar")
        yield from bar_arm.move(grip0, t=0.8, label="on the bar")
        yield from bar_arm.servo(grip0, timeout=0.6, label="on the bar")
        yield from bar_arm.close(0.028, force=20.0, label="grip the bar")

    def to_button():
        yield from other.move(over, grip=d.down(0.0), finger=d.FINGER_CLOSED, label="over the button")

    yield from ctx.together(to_bar(), *([to_button()] if uses_other else []))
    print(f"[spot] on the bar at {ctx.t:.2f} s", flush=True)
    tracker = Tracker(ctx, bar_arm, delay_s=1.0 if mode == "lag" else 0.0)

    def plan_deg() -> float | None:
        if mode == "fixed":
            return 0.0
        if mode == "sweep":
            # The best steady sweep found offline: period 10 s, 25 degrees each way.
            return 25.0 * (2.0 / math.pi) * math.asin(math.sin(2.0 * math.pi * (ctx.t - START_S) / 10.0)) if ctx.t >= START_S else 0.0
        return None

    def press_it():
        yield from other.move(press, t=0.35, label="press")
        yield from other.wait(0.15)
        yield from other.move(over, t=0.35, label="off the button")

    def watch():
        if mode == "early":
            for _ in range(round((START_S + 8.0 - ctx.t) / DT)):
                yield
            yield from press_it()
            return
        if mode == "no_press":
            return
        while not lamp_lit(ctx):
            yield
        print(f"[spot] lamp seen at {ctx.t:.2f} s; figure off by {ctx.deg('head', 'pan') - math.degrees(math.atan2(figure_y(ctx), AHEAD)):+.1f} deg", flush=True)
        if mode == "late":
            yield from other.wait(1.5)
        yield from press_it()

    if uses_other:
        # Track until the lamp has been dealt with and the figure has stood still for 1 s.
        watcher, ended = watch(), object()
        watching, lit_seen, still = True, False, 0
        while True:
            tracker.step(plan_deg())
            if watching and next(watcher, ended) is ended:
                watching = False
            lit_seen = lit_seen or lamp_lit(ctx)
            still = still + 1 if abs(tracker.speed) < 0.002 else 0
            if lit_seen and not watching and still * DT >= 1.0:
                break
            yield
    else:
        # One arm alone: it tracks until the lamp, then lets go and goes for the button.
        while not lamp_lit(ctx):
            tracker.step()
            yield
        print(f"[spot] {mode}: lamp seen at {ctx.t:.2f} s, letting go", flush=True)
        yield from bar_arm.open(t=0.2)
        yield from bar_arm.move(bar_arm.at() + [0.0, 0.0, 0.05], t=0.3)
        yield from bar_arm.move(over, grip=d.down(0.0), finger=d.FINGER_CLOSED, t=0.5, label="to the button")
        yield from bar_arm.move(press, t=0.2, label="press")
        print(f"[spot] {mode}: tried to press at {ctx.t:.2f} s; TCP {np.round(bar_arm.tcp() - [0.0, 0.0, TOP], 3)}", flush=True)
        yield from bar_arm.move(over, t=0.3)
        yield from bar_arm.wait(3.0)
        return
    print(f"[spot] figure stopped at {ctx.t:.2f} s", flush=True)
    if mode == "hold_on":
        yield from bar_arm.wait(2.0)
        return
    here = bar_arm.tcp()
    yield from bar_arm.open(t=0.5)
    yield from ctx.together(bar_arm.move(here + [0.0, 0.0, 0.07], t=0.8, label="off the bar"), other.move(over + [0.0, 0.0, 0.05], t=0.8, label="clear of the button"))
