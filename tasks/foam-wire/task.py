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
TICKS = 8
KERF_DOTS = 90
GOOD_OFF, RIBBON_W, THIN_KERF, GOOD_KERF = 0.005, 0.01, 0.002, 0.006
START_OFF, LIFT_MAX = 0.01, 0.015
CLEAR = 0.03


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


def judge(ticks_good: int, lifted: bool, bad_start: bool, clear_of_ribbon: bool, fingers_clear: bool, still: bool) -> tuple[bool, float]:
    """The score now: (full, score). Run on hand-built states by check_score.py."""
    if lifted or bad_start:
        return False, 0.0
    return bool(ticks_good == TICKS and clear_of_ribbon and fingers_clear and still), 0.75 * ticks_good / TICKS


def _ribbon() -> m.Part:
    z_top = 0.27  # high enough to clear the grippers as the block turns
    low = BOARD_T + 0.001 + 0.004  # the ribbon stops just short of the board, which it does not cut
    parts = [
        ("column", m.box((0.03, 0.03, z_top), (0.34, 0, z_top / 2)), "frame"),
        ("arm", m.box((0.34, 0.02, 0.02), (0.17, 0, z_top)), "frame"),
        ("ribbon", m.box((RIBBON_W, 0.0015, z_top - low), (0, 0, (z_top + low) / 2)), "ribbon"),
    ]
    cols = [m.col_box("column_c", (0.03, 0.03, z_top), (0.34, 0, z_top / 2)), m.col_box("arm_c", (0.34, 0.02, 0.02), (0.17, 0, z_top))]
    return m.fixed("cutter", (*RIBBON_XY, TOP), parts, cols, LOOKS)


def _block() -> m.Part:
    z0 = BOARD_T
    top = z0 + FOAM_H
    us = np.linspace(HALF, -HALF, 61)
    line = m.tube([(u, curve_y(u), top + 0.001) for u in us], 0.004, sides=6)
    parts = [
        ("board", m.box((BLOCK_D + 0.04, BOARD_W, BOARD_T), (0, 0, BOARD_T / 2)), "board"),
        ("foam", m.box((BLOCK_D, BLOCK_W, FOAM_H), (0, 0, z0 + FOAM_H / 2)), "foam"),
        ("curve", line, "curve"),
    ]
    for k, u in enumerate(tick_u()):
        parts.append((f"tick_{k}", m.box((0.004, 0.03, 0.001), (u, curve_y(u), top + 0.0015)), "tick"))
    for k in range(KERF_DOTS):
        parts.append((f"dot_{k}", m.box((0.005, 0.009, 0.0012), (0, 0, top + 0.0065)), "kerf", True))  # above the printed curve
    cols = [m.col_box("board_c", (BLOCK_D + 0.04, BOARD_W, BOARD_T), (0, 0, BOARD_T / 2)), m.col_box("foam_c", (BLOCK_D, BLOCK_W, FOAM_H), (0, 0, z0 + FOAM_H / 2))]
    for name, sign in (("a", 1.0), ("b", -1.0)):
        at = (0, sign * POST_Y, BOARD_T + POST_H / 2)
        parts.append((f"post_{name}", m.box((POST_T, POST_W, POST_H), at), "post"))
        cols.append(m.col_box(f"post_{name}_c", (POST_T, POST_W, POST_H), at))
    return m.free("block", BLOCK_START, parts, cols, LOOKS, mass=0.3, friction=(0.4, 0.3))


# The block starts near the arms, with the ribbon just past its far edge, its printed curve running on from there.
BLOCK_START = (RIBBON_XY[0] - HALF - 0.05, RIBBON_XY[1] - curve_y(HALF), TOP + 0.001)
OBJECTS = [_ribbon(), _block()]


class Trial:
    """The kerf, and what the scorer holds across a trial."""

    def reset(self, env=None) -> None:
        self.steps = 0
        self.cutting = False
        self.ever_cut = False
        self.lifted = False
        self.bad_start = False
        self.ticks_seen = np.zeros(TICKS, dtype=bool)
        self.ticks_good = np.zeros(TICKS, dtype=bool)
        self.dots = 0
        self.last_u = None
        self.full, self.score = False, 0.0
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
        inside = abs(u) <= HALF and abs(v) <= BLOCK_W / 2
        height = float(block.pos[2] - TOP)
        if inside and height > LIFT_MAX and not self.lifted:
            self.lifted = True
            print(f"[foam] lifted over the ribbon: {height * 100:.1f} cm up", flush=True)
        if inside and not self.cutting:
            if self.ever_cut:
                self.lifted = True
                print("[foam] a second kerf", flush=True)
            elif abs(v - curve_y(HALF)) > START_OFF:
                self.bad_start = True
                print(f"[foam] the cut starts {abs(v - curve_y(HALF)) * 100:.1f} cm off the line", flush=True)
            self.ever_cut = True
            self.last_u = u
            print(f"[foam] the cut starts: {(v - curve_y(HALF)) * 1000:.1f} mm off the line", flush=True)
        self.cutting = inside
        if inside and self.last_u is not None:
            if self.last_u - u >= BLOCK_D / KERF_DOTS:
                self._dot(u, v)
                self.last_u = u
            # The ribbon's flat lies along the world's x: how far the cut's path in the block turns from it.
            yaw = math.atan2(R[1, 0], R[0, 0])
            tangent = math.atan(curve_slope(u))
            angle = yaw + tangent
            for k, ut in enumerate(tick_u()):
                if not self.ticks_seen[k] and u <= ut:
                    self.ticks_seen[k] = True
                    off = abs(v - curve_y(ut))
                    width = kerf_width(angle)
                    self.ticks_good[k] = off <= GOOD_OFF and width <= GOOD_KERF
                    print(f"[foam] tick {k + 1}: {off * 1000:.1f} mm off, kerf {width * 1000:.1f} mm, yaw {math.degrees(yaw):.1f} deg: {'good' if self.ticks_good[k] else 'bad'}", flush=True)
        half_d, half_w = 0.5 * (BLOCK_D + 0.04), BOARD_W / 2
        clear_of_ribbon = abs(u) > half_d + 0.01 or abs(v) > half_w + 0.01
        tips = (m.fingertips(env) - block.pos) @ R
        out = np.maximum(np.maximum(np.array([-half_d, -half_w, 0.0]) - tips, tips - np.array([half_d, half_w, BOARD_T + POST_H])), 0.0)
        fingers = float(np.linalg.norm(out, axis=1).min()) >= CLEAR
        self.full, self.score = judge(int(self.ticks_good.sum()), self.lifted, self.bad_start, clear_of_ribbon, fingers, block.resting())


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    """All ticks good, the block clear of the ribbon and the fingers clear of the block."""
    return TRIAL.full


def score(objects) -> float:
    return TRIAL.score


# -- the demo: reads the block's pose, as a camera could see it -------------------------------------------------------------------------------------------


def demo_script(ctx):
    """Modes: full. slide: the block goes through without turning. none: the arms wait. Other score lines are checked offline: see check_score.py."""
    mode = ctx.mode
    if mode not in ("full", "slide", "none"):
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

    def pose_for(u: float, turning: bool) -> tuple[np.ndarray, float]:
        """Where the block stands with the ribbon at u on the curve, its path along the ribbon's flat."""
        on = float(np.clip(u, -HALF, HALF))  # past the block's edges, it runs straight
        yaw = -math.atan(curve_slope(on)) if turning else 0.0
        a = yaw
        point = np.array([u, curve_y(on)])
        rot = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
        return wire[:2] - rot @ point, math.degrees(yaw)

    turning = mode == "full"
    # In to the far edge, with the block already turned for the curve's start.
    first = pose_for(HALF + 0.03, turning)
    now, now_yaw = seen()
    n = round(2.5 / DT)
    for k in range(1, n + 1):
        s = d.ik.smoothstep(k / n)
        command(now[:2] + (first[0] - now[:2]) * s, now_yaw + (first[1] - now_yaw) * s)
        yield
    # Feed: the ribbon runs from the far edge to the near edge at a steady speed.
    feed_s = 20.0
    us = np.linspace(HALF + 0.03, -0.5 * (BLOCK_D + 0.04) - 0.012, round(feed_s / DT))
    for u in us:
        pos, yaw = pose_for(float(u), turning)
        command(pos, yaw)
        yield
    print(f"[foam] fed through; ticks good {int(TRIAL.ticks_good.sum())} of {TICKS}", flush=True)
    for _ in range(round(0.6 / DT)):  # the block has come off the ribbon at its near edge: settle there
        command(*pose_for(float(us[-1]), turning))
        yield
    yield from ctx.together(l.open(), r.open())
    yield from ctx.together(l.move(l.at() + [0.0, 0.0, 0.08], t=0.8, label="off the post"), r.move(r.at() + [0.0, 0.0, 0.08], t=0.8, label="off the post"))
    print(f"[foam] clear: block at {np.round(ctx.pos('block') - [0.0, 0.0, TOP], 3)}", flush=True)
    yield from l.wait(1.5)
