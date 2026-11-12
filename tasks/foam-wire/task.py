"""A foam block on a thin board, moved and turned past a fixed hot ribbon, so that the kerf follows a curve printed on the block. The ribbon cuts cleanly only along its flat, so the block must turn about it as the curve turns.

Debug:  python demokit.py task.py OUT --fast --mode full
Final:  python demokit.py task.py OUT
Model:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1400 --instruction "<INSTRUCTION below>"
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
INSTRUCTION = "Cut the foam block along the printed curve: hold it in both arms and move and turn it past the fixed hot ribbon. Then move it clear and take both grippers away."
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
HOLD_STEPS = 20  # SUCCESS_HOLD_S at 20 Hz: the harness ends the trial with 1.0 once success() has held this many control steps
TOP = d.TABLE_TOP
DT = d.DT

LOOKS = {
    "foam": m.Look(m.rgb(0.95, 0.95, 0.92), roughness=0.9),
    "board": m.Look(m.rgb(0.55, 0.42, 0.28), roughness=0.8),
    "curve": m.Look(m.rgb(0.05, 0.25, 0.85), roughness=0.7),
    "tick": m.Look(m.rgb(0.05, 0.05, 0.06), roughness=0.7),
    "post": m.Look(m.rgb(0.95, 0.65, 0.05), roughness=0.5),
    "frame": m.Look(m.rgb(0.25, 0.26, 0.28), metallic=0.3, roughness=0.5),
    "ribbon": m.Look(m.rgb(0.90, 0.25, 0.05), emissive=(0.8, 0.15, 0.0), roughness=0.4),
    "kerf": m.Look(m.rgb(0.05, 0.05, 0.05), roughness=0.9),
}

RIBBON_XY = (0.28, 0.0)
BOARD_T, FOAM_H = 0.006, 0.05
BLOCK_D, BLOCK_W = 0.15, 0.20  # along the block's x and y
BOARD_W = 0.31
POST_Y, POST_W, POST_T, POST_H = 0.13, 0.04, 0.02, 0.05
HALF = BLOCK_D / 2
AMP, WAVE = 0.02, BLOCK_D
TICKS = 8  # printed every 1.875 cm along x, to measure from
KERF_DOTS = 90
RAIL_H, RAIL_W, RAIL_Y = 0.012, 0.015, 0.075  # two wooden rails along x hold the foam up off the board, clear of the curve
FOAM_Z0 = BOARD_T + RAIL_H  # the foam's underside
RIBBON_LOW = BOARD_T + 0.001 + 0.006  # the ribbon's lower end: 6 mm above the board, 6 mm below the foam, so it cuts the foam's full depth and not the board
GOOD_OFF, RIBBON_W, THIN_KERF, GOOD_KERF = 0.005, 0.01, 0.002, 0.006
GASH_OFF = 0.015  # a kerf further than this from the curve is a cut that should not be there
START_OFF = 0.01
CLEAR = 0.03
FEED = 0.005  # m/s of the ribbon through the foam, along the curve: 300 mm/min


def curve_y(u: float) -> float:
    """The printed curve in the block's frame: y at x = u, a bend out and back from the far edge to the near edge, square to both edges."""
    return AMP * (1.0 + math.cos(2.0 * math.pi * u / WAVE))


def curve_slope(u: float) -> float:
    return -AMP * 2.0 * math.pi / WAVE * math.sin(2.0 * math.pi * u / WAVE)


def tick_u() -> np.ndarray:
    """Where the ticks stand along the block's x, from the far edge in."""
    return HALF - (np.arange(1, TICKS + 1) - 0.5) * BLOCK_D / TICKS


def kerf_width(angle_rad: float) -> float:
    """How wide the ribbon melts the foam when fed this far from along its flat."""
    return THIN_KERF + RIBBON_W * abs(math.sin(angle_rad))


def judge(good_share: float, through: bool, gash: bool, second: bool, bad_start: bool, board_cut: bool, clear_of_ribbon: bool, fingers_clear: bool, still: bool, held_steps: int = HOLD_STEPS) -> tuple[bool, float]:
    """The score now: (full, score). Run on hand-built states by check_score.py.
    good_share: the share of the block's 15 cm, from the far edge, along which the kerf has stayed within 5 mm of the curve and at most 6 mm wide.
    through: that good run has reached the near edge, and the ribbon has left the foam there, so the cut runs edge to edge.
    full: every 1.0 condition holds at this step. held_steps: for how many control steps they have held, this one included.
    The score is 1.0 once they have held for SUCCESS_HOLD_S, and the lower score of the text until then."""
    if gash or second or bad_start or board_cut:
        return False, 0.0
    full = bool(through and clear_of_ribbon and fingers_clear and still)
    if full and held_steps >= HOLD_STEPS:
        return True, 1.0
    return full, 0.75 * (1.0 if through else min(max(good_share, 0.0), 1.0))


def _ribbon() -> m.Part:
    z_top = 0.27  # high enough to clear the grippers as the block turns
    low = RIBBON_LOW
    parts = [
        ("column", m.box((0.03, 0.03, z_top), (0.34, 0, z_top / 2)), "frame"),
        ("arm", m.box((0.34, 0.02, 0.02), (0.17, 0, z_top)), "frame"),
        ("ribbon", m.box((RIBBON_W, 0.0015, z_top - low), (0, 0, (z_top + low) / 2)), "ribbon"),
    ]
    cols = [m.col_box("column_c", (0.03, 0.03, z_top), (0.34, 0, z_top / 2)), m.col_box("arm_c", (0.34, 0.02, 0.02), (0.17, 0, z_top))]
    return m.fixed("cutter", (*RIBBON_XY, TOP), parts, cols, LOOKS)


def _block() -> m.Part:
    z0 = FOAM_Z0
    top = z0 + FOAM_H
    us = np.linspace(HALF, -HALF, 61)
    line = m.tube([(u, curve_y(u), top + 0.001) for u in us], 0.004, sides=6)
    parts = [
        ("board", m.box((BLOCK_D + 0.04, BOARD_W, BOARD_T), (0, 0, BOARD_T / 2)), "board"),
        ("foam", m.box((BLOCK_D, BLOCK_W, FOAM_H), (0, 0, z0 + FOAM_H / 2)), "foam"),
        ("curve", line, "curve"),
    ]
    for name, sign in (("a", 1.0), ("b", -1.0)):
        parts.append((f"rail_{name}", m.box((BLOCK_D, RAIL_W, RAIL_H), (0, sign * RAIL_Y, BOARD_T + RAIL_H / 2)), "board"))
    for k, u in enumerate(tick_u()):
        parts.append((f"tick_{k}", m.box((0.004, 0.03, 0.001), (u, curve_y(u), top + 0.0015)), "tick"))
    for k in range(KERF_DOTS):
        parts.append((f"dot_{k}", m.box((0.005, 0.009, 0.0012), (0, 0, top + 0.0065)), "kerf", True))  # above the printed curve
    cols = [m.col_box("board_c", (BLOCK_D + 0.04, BOARD_W, BOARD_T), (0, 0, BOARD_T / 2)), m.col_box("foam_c", (BLOCK_D, BLOCK_W, FOAM_H), (0, 0, z0 + FOAM_H / 2))]
    cols += [m.col_box(f"rail_{n}_c", (BLOCK_D, RAIL_W, RAIL_H), (0, sg * RAIL_Y, BOARD_T + RAIL_H / 2)) for n, sg in (("a", 1.0), ("b", -1.0))]
    for name, sign in (("a", 1.0), ("b", -1.0)):
        at = (0, sign * POST_Y, BOARD_T + POST_H / 2)
        parts.append((f"post_{name}", m.box((POST_T, POST_W, POST_H), at), "post"))
        cols.append(m.col_box(f"post_{name}_c", (POST_T, POST_W, POST_H), at))
    return m.free("block", BLOCK_START, parts, cols, LOOKS, mass=0.3, friction=(0.4, 0.3))


# The block starts near the arms, with the ribbon just past its far edge, its printed curve running on from there.
BLOCK_START = (RIBBON_XY[0] - HALF - 0.05, RIBBON_XY[1] - curve_y(HALF), TOP + 0.001)
OBJECTS = [_ribbon(), _block()]


class Kerf:
    """The cut so far, from where the ribbon stands in the block's frame at each step. Plain Python: check_score.py runs it on whole hand-built paths."""

    def __init__(self) -> None:
        self.cutting = False
        self.ever_cut = False
        self.second = False  # the ribbon entered the foam a second time
        self.bad_start = False
        self.gash = False
        self.on_line = True  # the kerf so far has stayed on the line and thin
        self.good_to = HALF  # how far, along the block's x, the good run has reached from the far edge
        self.through = False
        self.events: list[str] = []

    def good_share(self) -> float:
        return 1.0 if self.through else (HALF - self.good_to) / BLOCK_D

    def feed(self, u: float, v: float, yaw: float) -> tuple[bool, float, float]:
        """The ribbon at (u, v) in the block's frame, the block turned yaw radians from the ribbon's flat. Returns (inside the foam, off the curve, kerf width)."""
        inside = abs(u) <= HALF and abs(v) <= BLOCK_W / 2
        if inside and not self.cutting:  # the ribbon enters the foam
            start_off = math.hypot(u - HALF, v - curve_y(HALF))
            if self.ever_cut:
                self.second = True
                self.events.append("the ribbon enters the foam a second time")
            elif start_off > START_OFF:
                self.bad_start = True
                self.events.append(f"the cut starts {start_off * 100:.1f} cm from the curve's start")
            else:
                self.events.append(f"the cut starts {(v - curve_y(HALF)) * 1000:.1f} mm off the line")
            self.ever_cut = True
        if self.cutting and not inside:  # the ribbon leaves the foam
            if self.on_line and u < -HALF:
                self.through = True
            self.events.append(f"the ribbon leaves the foam at ({u * 100:.1f}, {v * 100:.1f}) cm: {'through, on the line' if self.through else 'not through on the line'}")
        self.cutting = inside
        if not inside:
            return False, 0.0, 0.0
        off = abs(v - curve_y(u))
        width = kerf_width(yaw + math.atan(curve_slope(u)))
        if off > GASH_OFF and not self.gash:
            self.gash = True
            self.events.append(f"a gash: the kerf is {off * 1000:.1f} mm from the curve at x {u * 100:.1f} cm")
        if self.on_line and (off > GOOD_OFF or width > GOOD_KERF):
            self.on_line = False
            self.events.append(f"the good run ends at x {u * 100:.1f} cm, share {self.good_share():.3f}: {off * 1000:.1f} mm off, kerf {width * 1000:.1f} mm")
        if self.on_line:
            self.good_to = min(self.good_to, u)
        return True, off, width


class Trial:
    """The kerf, and what the scorer holds across a trial."""

    def reset(self, env=None) -> None:
        self.steps = 0
        self.kerf = Kerf()
        self.board_cut = False
        self.ticks_seen = np.zeros(TICKS, dtype=bool)
        self.dots = 0
        self.last_u = None
        self.full, self.score = False, 0.0
        self.held = 0  # control steps the full state has lasted
        if env is not None:
            for k in range(KERF_DOTS):
                m.set_visible(f"{m.ENV_PATH}/block/visual/dot_{k}", False)

    def _dot(self, u: float, v: float) -> None:
        import isaaclab.sim as sim_utils
        from pxr import Gf, UsdGeom

        if self.dots >= KERF_DOTS:
            return
        path = f"{m.ENV_PATH}/block/visual/dot_{self.dots}"
        xf = UsdGeom.Xformable(sim_utils.get_current_stage().GetPrimAtPath(path))
        ops = xf.GetOrderedXformOps()
        (ops[0] if ops else xf.AddTranslateOp()).Set(Gf.Vec3d(u, v, 0.0))
        m.set_visible(path, True)
        self.dots += 1

    def step(self, env) -> None:
        self.steps += 1
        if os.environ.get("TASK_STEP_FILE"):  # the simulated time, for anything following the run from outside
            with open(os.environ["TASK_STEP_FILE"], "w") as f:
                f.write(f"{self.steps * DT:.2f}")
        block = m.state(env, "block")
        R = block.rotation()
        p = (np.array([*RIBBON_XY, block.pos[2]]) - block.pos) @ R  # the ribbon in the block's frame
        u, v = float(p[0]), float(p[1])
        half_d, half_w = 0.5 * (BLOCK_D + 0.04), BOARD_W / 2
        if abs(u) <= half_d and abs(v) <= half_w and not self.board_cut:
            board_top = float((block.pos + R @ np.array([u, v, BOARD_T]))[2] - TOP)
            if board_top >= RIBBON_LOW:
                self.board_cut = True
                print(f"[foam] the board meets the ribbon: its top {board_top * 1000:.1f} mm up", flush=True)
        yaw = math.atan2(R[1, 0], R[0, 0])  # the ribbon's flat lies along the world's x
        k = self.kerf
        seen = len(k.events)
        inside, off, width = k.feed(u, v, yaw)
        for line in k.events[seen:]:
            print(f"[foam] {line}", flush=True)
        if inside and self.last_u is None:
            self.last_u = u
        if inside:
            if self.last_u - u >= BLOCK_D / KERF_DOTS:
                self._dot(u, v)
                self.last_u = u
            for j, ut in enumerate(tick_u()):
                if not self.ticks_seen[j] and u <= ut:
                    self.ticks_seen[j] = True
                    print(f"[foam] at tick {j + 1}: {off * 1000:.1f} mm off, kerf {width * 1000:.1f} mm, yaw {math.degrees(yaw):.1f} deg", flush=True)
        clear_of_ribbon = abs(u) > half_d + 0.01 or abs(v) > half_w + 0.01
        tips = (m.fingertips(env) - block.pos) @ R
        out = np.maximum(np.maximum(np.array([-half_d, -half_w, 0.0]) - tips, tips - np.array([half_d, half_w, FOAM_Z0 + FOAM_H])), 0.0)
        fingers = float(np.linalg.norm(out, axis=1).min()) >= CLEAR
        self.full, self.score = judge(k.good_share(), k.through, k.gash, k.second, k.bad_start, self.board_cut, clear_of_ribbon, fingers, block.resting(), self.held + 1)
        self.held = self.held + 1 if self.full else 0
        if self.held == HOLD_STEPS:
            print(f"[foam] full state held {HOLD_STEPS * DT:.2f} s: score {self.score}", flush=True)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    """Cut edge to edge on the line, the block at rest clear of the ribbon and the fingers clear of the block, with no gash, second cut, bad start or cut board."""
    return TRIAL.full


def score(objects) -> float:
    """1.0 once the full state has held for SUCCESS_HOLD_S; otherwise the lower score of the scoring text for the state."""
    return TRIAL.score


# -- the demo: reads the block's pose, as a camera could see it -------------------------------------------------------------------------------------------


def demo_script(ctx):
    """Modes: full. slide: the block goes through without turning. half: the cut stops at the block's middle, and the arms let go there.
    gash: on the line almost to the near edge, then 3 cm sideways. none: the arms wait. Other score lines are checked offline: see check_score.py."""
    mode = ctx.mode
    if mode not in ("full", "slide", "half", "gash", "none"):
        raise ValueError(f"unknown mode {mode!r}")
    l, r = ctx.left, ctx.right
    if mode == "none":
        yield from l.wait(3.0)
        return
    wire = np.array([*RIBBON_XY, 0.0])
    grip_up = BOARD_T + POST_H / 2
    z = TOP + 0.001 + grip_up + 0.002  # the board just off the table
    arms = ((l, 1.0, -90.0), (r, -1.0, 90.0))

    def post_at(pos, yaw_deg: float, sign: float) -> np.ndarray:
        a = math.radians(yaw_deg)
        return np.array([pos[0] - sign * POST_Y * math.sin(a), pos[1] + sign * POST_Y * math.cos(a), z])

    def grip(yaw_deg: float, base: float):
        return d.down(base + yaw_deg)

    start = ctx.pos("block")
    yield from ctx.together(*(a.move(post_at(start, 0.0, s) + [0.0, 0.0, 0.06], grip=grip(0.0, b), finger=d.FINGER_OPEN, label="over the post") for a, s, b in arms))
    yield from ctx.together(*(a.move(post_at(start, 0.0, s), t=0.8, label="on the post") for a, s, b in arms))
    yield from ctx.together(*(a.servo(post_at(start, 0.0, s), timeout=0.5, label="on the post") for a, s, b in arms))
    yield from ctx.together(*(a.close(POST_T, force=25.0, label="grip the post") for a, s, b in arms))

    trim = np.zeros(2)
    yaw_trim = 0.0

    def seen():
        st = ctx.state("block")
        return st.pos.copy(), math.degrees(math.atan2(st.rotation()[1, 0], st.rotation()[0, 0]))

    def command(pos_xy, yaw_deg: float):
        nonlocal trim, yaw_trim
        now, now_yaw = seen()
        trim = trim + np.clip(0.15 * (np.asarray(pos_xy) - now[:2]), -0.003, 0.003)
        yaw_trim = yaw_trim + float(np.clip(0.15 * (yaw_deg - now_yaw), -0.5, 0.5))
        for a, s, b in arms:
            mm, deg = a._solve(post_at(np.asarray(pos_xy) + trim, yaw_deg + yaw_trim, s), grip(yaw_deg + yaw_trim, b).rot)
            a._note(mm, deg, "feed the block")

    def pose_for(u: float, turning: bool, side: float = 0.0) -> tuple[np.ndarray, float]:
        """Where the block stands with the ribbon at u on the curve, or `side` off it, its path along the ribbon's flat."""
        on = float(np.clip(u, -HALF, HALF))  # past the block's edges, it runs straight
        yaw = -math.atan(curve_slope(on)) if turning else 0.0
        a = yaw
        point = np.array([u, curve_y(on) + side])
        rot = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
        return wire[:2] - rot @ point, math.degrees(yaw)

    turning = mode != "slide"
    # In to the far edge, with the block already turned for the curve's start.
    first = pose_for(HALF + 0.03, turning)
    now, now_yaw = seen()
    n = round(2.5 / DT)
    for k in range(1, n + 1):
        s = d.ik.smoothstep(k / n)
        command(now[:2] + (first[0] - now[:2]) * s, now_yaw + (first[1] - now_yaw) * s)
        yield
    # Feed: through the foam at FEED along the curve, twice that over the board's bare edges.
    end = 0.0 if mode == "half" else -0.5 * (BLOCK_D + 0.04) - 0.012
    u, side = HALF + 0.03, 0.0
    while u > end:
        inside = -HALF <= u <= HALF
        u -= (FEED if inside else 2.0 * FEED) * DT / math.sqrt(1.0 + curve_slope(float(np.clip(u, -HALF, HALF))) ** 2)
        if mode == "gash" and u < -0.04:
            side = min(0.03, side + FEED * DT)
        command(*pose_for(u, turning, side))
        yield
    print(f"[foam] fed to x {u * 100:.1f} cm; good run {TRIAL.kerf.good_share():.3f} of the block, through {TRIAL.kerf.through}", flush=True)
    for _ in range(round(0.6 / DT)):  # settle where the feed ended
        command(*pose_for(u, turning, side))
        yield
    yield from ctx.together(l.open(), r.open())
    yield from ctx.together(l.move(l.at() + [0.0, 0.0, 0.08], t=0.8, label="off the post"), r.move(r.at() + [0.0, 0.0, 0.08], t=0.8, label="off the post"))
    print(f"[foam] clear: block at {np.round(ctx.pos('block') - [0.0, 0.0, TOP], 3)}", flush=True)
    yield from l.wait(1.5)
