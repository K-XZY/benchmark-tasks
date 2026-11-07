"""A glockenspiel in two groups of three bars, 40 cm apart, one group before each arm. A felt mallet lies in a cradle beside each group, and a printed card shows six coloured squares, read from the robot's left. The seed picks the card.

Run a model:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 400 --instruction "..."

The demo:  python demokit.py task.py OUT --mode full --seed 0     the final video
           python demokit.py task.py OUT --fast --mode full --seed 0     debugging, with no cameras
           python check_scorer.py                                 the scoring table, with no simulator
"""

from __future__ import annotations

import dataclasses
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demokit as d  # noqa: E402
import mechkit as m  # noqa: E402

SCENE = "station"
INSTRUCTION = "Play the notes on the card in order, left to right, at one note every half second, with one mallet in each hand, taking turns, then lay the mallets back in their cradles."
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
TOP = d.TABLE_TOP

BAR_X = 0.42  # the bars' middle
GROUP_Y = 0.20
BAR_GAP = 0.05
BAR = (0.11, 0.038, 0.008)
BAR_TOP = 0.035
COLOURS = {  # the bars by name: red, yellow and green for the left arm, blue, white and purple for the right
    "L0": (0.85, 0.05, 0.04), "L1": (0.95, 0.80, 0.05), "L2": (0.05, 0.60, 0.15),
    "R0": (0.05, 0.20, 0.85), "R1": (0.95, 0.95, 0.95), "R2": (0.55, 0.10, 0.70),
}
BARS_Y = {"L0": GROUP_Y + BAR_GAP, "L1": GROUP_Y, "L2": GROUP_Y - BAR_GAP, "R0": -GROUP_Y + BAR_GAP, "R1": -GROUP_Y, "R2": -GROUP_Y - BAR_GAP}
CARDS = (  # left arm first, then turn about
    ("L1", "R1", "L0", "R2", "L2", "R0"),
    ("R0", "L2", "R2", "L1", "R1", "L0"),
    ("L2", "R2", "L1", "R0", "L0", "R1"),
)
CARD_XY = (0.62, 0.0)
SQUARE = 0.05

MALLET_LEN = 0.18
SHAFT_R = 0.005
GRIP = (0.04, 0.012, 0.022)  # a flat grip at the shaft's middle, which the fingers pinch: a round shaft turns in them at each strike
HEAD_R = 0.012
HEAD_LOCAL = np.array([MALLET_LEN / 2, 0.0, 0.0])
CRADLE_X = 0.20
CRADLE_Z = 0.04  # the shaft's axis above the table, in the cradle
MALLET_MASS = 0.06

BEAT = 0.5
BEAT_WITHIN = 0.2 * BEAT  # the gaps' limit, 20 percent of a beat
STRUCK = 0.001  # m: how far a bar must go down to count as struck
HELD_S = 0.3  # a bar kept down this long has been pressed, not struck
HEAD_LOW_WITHIN = 0.005  # the head's lowest point above the bar's top, when the bar goes down
CLEAR_FROM = 0.03
LAID_WITHIN = 0.02
NOTE_SCORE = 0.15

LOOKS = {
    "frame": m.Look(m.rgb(0.13, 0.09, 0.06), roughness=0.7),
    "cradle": m.Look(m.rgb(0.30, 0.20, 0.12), roughness=0.8),
    "card": m.Look(m.rgb(0.95, 0.95, 0.90), roughness=0.9),
    "shaft": m.Look(m.rgb(0.75, 0.60, 0.40), roughness=0.6),
    "felt": m.Look(m.rgb(0.25, 0.25, 0.30), roughness=0.95),
    **{n: m.Look(m.rgb(*c), metallic=0.3, roughness=0.4) for n, c in COLOURS.items()},
}


def card() -> tuple[str, ...]:
    """The card for this trial: the reset seed modulo 3, and the first when there is none. A printed card: read from the scene."""
    seed = m.run_seed()
    return CARDS[0 if seed is None else seed % len(CARDS)]


def _bars(name: str, sign: int) -> m.Part:
    """One group, on a frame: each bar on a stiff spring that gives a few millimetres."""
    names = [n for n in BARS_Y if n[0] == name]
    links = []
    for n in names:
        y = BARS_Y[n] - sign * GROUP_Y
        mesh = m.box(BAR, (0, y, BAR_TOP - BAR[2] / 2))
        links.append(m.slider(n, "Z", -0.004, 0.0, [(n, mesh, n)], [m.col_box(f"{n}_c", BAR, (0, y, BAR_TOP - BAR[2] / 2))], mass=0.03, spring=300.0, damping=1.5, friction=0.0))
    rails = [m.box((0.012, 0.17, 0.024), (sx * 0.04, 0, 0.012)) for sx in (-1, 1)]
    frame = [("rails", m.join(*rails), "frame")]
    cols = [m.col_box(f"rail_{i}", (0.012, 0.17, 0.024), (sx * 0.04, 0, 0.012)) for i, sx in enumerate((-1, 1))]
    return m.mechanism(f"bars_{name}", (BAR_X, sign * GROUP_Y, TOP), links, {k: LOOKS[k] for k in (*names, "frame")}, frame_parts=frame, frame_colliders=cols, friction=(0.8, 0.7))


def _mallet(name: str, y: float) -> m.Part:
    shaft = m.cylinder(SHAFT_R, MALLET_LEN, (0, 0, 0), axis="x", segments=12)
    head = m.sphere(HEAD_R, tuple(HEAD_LOCAL))
    grip = m.box(GRIP, (0, 0, 0))
    return m.free(name, (CRADLE_X, y, TOP + CRADLE_Z), [("shaft", shaft, "shaft"), ("grip", grip, "felt"), ("head", head, "felt")], [m.col_hull("shaft_c", shaft), m.col_box("grip_c", GRIP, (0, 0, 0)), m.col_hull("head_c", head)], {k: LOOKS[k] for k in ("shaft", "felt")}, mass=MALLET_MASS, friction=(1.2, 1.0))


def _cradle(name: str, y: float) -> m.Part:
    """Two forked blocks that the shaft lies in, either side of where the fingers grip it."""
    floor_top = CRADLE_Z - SHAFT_R - 0.0005
    parts, cols = [], []
    for i, dx in enumerate((-0.055, 0.055)):
        block = m.box((0.015, 0.04, floor_top), (dx, 0, floor_top / 2))
        lips = [m.box((0.015, 0.008, 0.012), (dx, sy * 0.012, floor_top + 0.006)) for sy in (-1, 1)]
        parts += [(f"block{i}", block, "cradle"), (f"lips{i}", m.join(*lips), "cradle")]
        cols += [m.col_box(f"block{i}_c", (0.015, 0.04, floor_top), (dx, 0, floor_top / 2))]
        cols += [m.col_box(f"lip{i}{j}_c", (0.015, 0.008, 0.012), (dx, sy * 0.012, floor_top + 0.006)) for j, sy in enumerate((-1, 1))]
    return m.fixed(name, (CRADLE_X, y, TOP), parts, cols, {"cradle": LOOKS["cradle"]})


def _printed() -> m.Part:
    """The printed card, flat on the table: six squares in a row, the first toward the robot's left. Built when the scene is, inside the first reset, so that it follows the seed."""
    sheet = ("sheet", m.box((0.08, 0.40, 0.002), (0, 0, 0.001)), "card")
    looks = {"card": LOOKS["card"], **{n: LOOKS[n] for n in COLOURS}}

    def build() -> str:
        squares = [(f"square{i}", m.box((SQUARE, SQUARE, 0.001), (0.0, (2.5 - i) * (SQUARE + 0.008), 0.0025)), n) for i, n in enumerate(card())]
        return m.fixed("card", (*CARD_XY, TOP), [sheet, *squares], (), looks).build()

    return dataclasses.replace(m.fixed("card", (*CARD_XY, TOP), [sheet], (), looks), build=build)


OBJECTS = [_bars("L", 1), _bars("R", -1), _cradle("cradle_l", GROUP_Y), _cradle("cradle_r", -GROUP_Y), _mallet("mallet_l", GROUP_Y), _mallet("mallet_r", -GROUP_Y), _printed()]


def head_on_bar(head: np.ndarray, bar: str) -> bool:
    """Whether this mallet's head is what put the bar down: over the bar's footprint, and touching its top. A finger or a shaft leaves the head elsewhere."""
    over = abs(head[0] - BAR_X) <= BAR[0] / 2 + HEAD_R and abs(head[1] - BARS_Y[bar]) <= BAR[1] / 2 + HEAD_R
    return bool(over and head[2] - HEAD_R - TOP <= BAR_TOP + HEAD_LOW_WITHIN)


def task_score(wrong: bool, dropped: bool, pressed: bool, notes: int, laid: bool) -> tuple[float, bool]:
    """The scoring text's table, line by line: `notes` is how many were played right, in order and in time, from the first. Returns the score, and whether that is the full one."""
    if wrong or dropped or pressed:
        return 0.0, False
    if notes >= len(CARDS[0]) and laid:
        return 1.0, True
    return round(NOTE_SCORE * min(notes, len(CARDS[0])), 2), False


class Trial:
    """What the scorer holds across a trial."""

    def reset(self, env=None) -> None:
        self.t = 0.0
        self.down = {n: 0.0 for n in BARS_Y}
        self.was_down = {n: False for n in BARS_Y}
        self.played: list[tuple[float, str, str]] = []
        self.good = 0
        self.in_time = True
        self.wrong = self.dropped = self.pressed = False
        self.lifted = {"mallet_l": False, "mallet_r": False}
        self.score, self.full = 0.0, False
        self.rep = {}
        self.n = 0

    def step(self, env) -> None:
        self.t += 1.0 / d.CONTROL_HZ
        self.n += 1
        if os.environ.get("TASK_STEP_FILE"):  # the simulated time, for anything following the run from outside
            with open(os.environ["TASK_STEP_FILE"], "w") as f:
                f.write(f"{self.t:.2f}")
        order = card()
        tips = m.fingertips(env)
        heads, laid, clear = {}, True, True
        for name, cradle_y in (("mallet_l", GROUP_Y), ("mallet_r", -GROUP_Y)):
            s = m.state(env, name)
            heads[name] = s.to_world(HEAD_LOCAL[None])[0]
            on_cradle = bool(np.linalg.norm(s.pos - np.array([CRADLE_X, cradle_y, TOP + CRADLE_Z])) <= LAID_WITHIN)
            laid = laid and on_cradle
            self.lifted[name] = self.lifted[name] or s.pos[2] > TOP + CRADLE_Z + 0.02
            if self.lifted[name] and not on_cradle and s.pos[2] < TOP + 0.03:
                self.dropped = True
            ends = s.to_world(np.array([[-MALLET_LEN / 2, 0.0, 0.0], [MALLET_LEN / 2, 0.0, 0.0]]))
            along = ends[1] - ends[0]
            on_shaft = ends[0] + np.clip((tips - ends[0]) @ along / (along @ along), 0.0, 1.0)[:, None] * along
            clear = clear and float(np.linalg.norm(tips - on_shaft, axis=1).min()) >= CLEAR_FROM
        for group in ("L", "R"):
            s = m.state(env, f"bars_{group}")
            for bar in (n for n in BARS_Y if n[0] == group):
                depth = -s.joint(bar)
                is_down = depth >= STRUCK
                if is_down and not self.was_down[bar]:
                    self._note(bar, heads, order)
                self.down[bar] = self.down[bar] + 1.0 / d.CONTROL_HZ if is_down else 0.0
                if self.down[bar] >= HELD_S:
                    self.pressed = True
                self.was_down[bar] = is_down
        self.score, self.full = task_score(self.wrong, self.dropped, self.pressed, self.good, laid and clear)
        self.rep = {"t": round(self.t, 2), "score": self.score, "good": self.good, "played": [(round(t, 2), b, who) for t, b, who in self.played], "wrong": self.wrong, "dropped": self.dropped, "pressed": self.pressed, "laid": laid, "clear": clear, "card": order}
        if self.n % 20 == 0:
            print(f"[mallet] {self.rep}", flush=True)

    def _note(self, bar: str, heads: dict, order: tuple) -> None:
        own = "mallet_l" if bar[0] == "L" else "mallet_r"
        k = len(self.played)
        if k >= len(order) or bar != order[k] or not head_on_bar(heads[own], bar):
            self.wrong = True
        elif k > 0 and abs(self.t - self.played[-1][0] - BEAT) > BEAT_WITHIN:
            self.in_time = False
        if self.in_time and not self.wrong:
            self.good = k + 1
        self.played.append((self.t, bar, own))


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.full


def score(objects) -> float:
    return TRIAL.score


# -- the demo --------------------------------------------------------------------------------

GRIP_ABOVE = 0.008  # the TCP above the shaft's axis, so that the pads close on the flat grip and the tips clear the cradle's floor
HOVER = 0.028  # the head's underside above a bar's top, waiting
PRESS_PAST = 0.006  # the stroke's bottom, commanded this far below the bar's top
STROKE_S = 0.14  # down, and the same up
PARK = {"left": (0.25, 0.32, TOP + 0.20), "right": (0.25, -0.32, TOP + 0.20)}


def _head(ctx, name: str) -> np.ndarray:
    return ctx.state(name).to_world(HEAD_LOCAL[None])[0]


def _read_card(ctx) -> tuple[str, ...]:
    """The order, from the top camera's pixels at the six squares: each is the bar whose colour it is closest to in hue."""
    points = np.array([[CARD_XY[0], CARD_XY[1] + (2.5 - i) * (SQUARE + 0.008), TOP + 0.003] for i in range(6)])
    seen = ctx.see(points)
    names = list(COLOURS)
    refs = np.array([COLOURS[n] for n in names])
    refs = refs / np.linalg.norm(refs, axis=1, keepdims=True)
    hue = seen / np.maximum(np.linalg.norm(seen, axis=1, keepdims=True), 1.0)
    order = tuple(names[int(np.argmax(refs @ h))] for h in hue)
    print(f"[mallet] the camera reads the card as {order}, from {np.round(seen).astype(int).tolist()}", flush=True)
    return order


def _phrase(ctx, arm, mallet: str, bars: list[tuple[int, str]], start: float):
    """Play one arm's notes. `bars` is (beat number, bar). The head goes over each bar in time for its beat, and rebounds at once. Where the head is against the TCP is read once, with the arm at rest: read on the move, the arm's lag would raise each stroke."""
    offset = _head(ctx, mallet) - arm.tcp()
    over_z = TOP + BAR_TOP + HEAD_R + HOVER - offset[2]
    for beat, bar in bars:
        xy = np.array([BAR_X, BARS_Y[bar]]) - offset[:2]
        when = start + beat * BEAT
        lead = max(0.2, when - ctx.t - STROKE_S - 0.05)
        yield from arm.move((xy[0], xy[1], over_z), t=lead, label=f"over {bar}")
        while ctx.t < when - STROKE_S:
            yield
        yield from arm.move((xy[0], xy[1], over_z - HOVER - PRESS_PAST), t=STROKE_S, label=f"strike {bar}")
        yield from arm.move((xy[0], xy[1], over_z), t=STROKE_S, label=f"rebound {bar}")
        print(f"[mallet] struck {bar}: head at {np.round(_head(ctx, mallet) - [0.0, 0.0, TOP], 3)}, tilt {ctx.state(mallet).tilt_deg():.1f} deg, t {ctx.t:.2f}", flush=True)


def demo_script(ctx):
    """Closed loop on the mallets' seen positions. The order comes from the printed card, which a policy can read; no scorer state is read."""
    mode, left, right = ctx.mode, ctx.left, ctx.right
    if mode == "idle":
        yield from left.wait(4.0)
        return
    order = _read_card(ctx)
    grip = d.down(0.0)

    def take(arm, name):
        at = ctx.pos(name) + [0.0, 0.0, GRIP_ABOVE]
        yield from arm.move(at + [0.0, 0.0, 0.07], grip=grip, finger=-0.02, t=2.5, label=f"over {name}")
        yield from arm.move(at, t=0.8, label=f"down to {name}")
        yield from arm.close(width=GRIP[1], force=30.0, t=0.4, label=f"close on {name}")
        yield from arm.move(at + [0.0, 0.0, 0.05], t=0.8, label=f"lift {name}")

    yield from ctx.together(take(left, "mallet_l"), take(right, "mallet_r"))
    yield from ctx.left.wait(0.3)
    start = ctx.t + 1.6
    notes = {side: [(i, b) for i, b in enumerate(order) if b[0] == side] for side in "LR"}
    yield from ctx.together(_phrase(ctx, left, "mallet_l", notes["L"], start), _phrase(ctx, right, "mallet_r", notes["R"], start))

    def lay(arm, name, y, side):
        here = arm.at()
        down_at = np.array([CRADLE_X, y, TOP + CRADLE_Z + GRIP_ABOVE])
        yield from arm.move((down_at[0], down_at[1], here[2]), t=1.2, label=f"back over the {name} cradle")
        yield from arm.move(down_at + [0.0, 0.0, 0.002], t=0.8, label=f"into the {name} cradle")
        yield from arm.open(t=0.3, label=f"let go of {name}")
        yield from arm.move(down_at + [0.0, 0.0, 0.08], t=0.8, label=f"up off {name}")
        yield from arm.move(PARK[side], finger=d.FINGER_OPEN, t=1.2, label="park")

    yield from ctx.together(lay(left, "mallet_l", GROUP_Y, "left"), lay(right, "mallet_r", -GROUP_Y, "right"))
    yield from left.wait(1.5)
