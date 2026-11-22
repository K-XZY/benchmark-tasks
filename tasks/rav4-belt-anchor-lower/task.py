"""Lower a Toyota RAV4 front shoulder-belt anchor on its height adjuster (Toyota 73200-12070): hold the lock release button in, slide the anchor down
past its notches to the lowest one, and let the button go so the lock catches there.

Frame: x forward, y left, z up, metres; z = 0 is the ground. The station's table is cut back to a strip under the two arm bases (top 0.75 m). Beyond it
stands a bench board on two aluminium posts, tilted 10 degrees from upright with its top towards the arms (the B-pillar's inward lean), and the bought
adjuster is bolted to it through its rail's two end holes, the way it bolts to the pillar.

The board (a fixed articulation, turned about y by TILT): frame = plywood board, the adjuster's steel rail and its two mounting bolts. Links: the anchor
(slider body, the raised block above the button, the anchor bolt's collar, the belt's D-ring guide and its nut, rigid) on a slide along the rail: joint
zero is its top notch, 5 notches 15 mm apart down to -60 mm. Its child: the white lock release button at the slider's lower edge, under the bolt, on a
slide into the slider, 4 mm of travel, spring 2500 N/m (10 N pressed in full).
The lock (code, the pawl): while the button is in at least 2 mm (5 N), the anchor slides freely, with its mass (0.22 kg) and the rail's joint friction (0.5 N); its weight is left out (gravity off), which makes the act harder. With the button back out
(within 1 mm of its rest), the pawl catches a notch only when the anchor is within 1.5 mm of it (a stiff spring, 50 kN/m, capped at 2000 N); an anchor let go
between notches rides free on the rail's friction and the pawl catches the first notch it passes within 1.5 mm while the button stays out.

Debug:  python demokit.py task.py OUT --no-record --mode full,none,no_button,half,hold,tap,short
Final:  python demokit.py task.py OUT
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
INSTRUCTION = ("Lower the seat belt's shoulder anchor to its lowest notch: hold its release button in while sliding it down, let go so it locks, and clear the hands.")
SUCCESS_HOLD_S = 3.0
SCORE_OVER = "end"
HOLD_STEPS = 60  # SUCCESS_HOLD_S at 20 Hz
TOP = d.TABLE_TOP
TRACE = os.environ.get("ANCHOR_TRACE", "1") == "1"

LOOKS = {
    "board": m.Look(m.rgb(0.72, 0.60, 0.44), roughness=0.8),
    "post": m.Look(m.rgb(0.70, 0.71, 0.72), roughness=0.4),
    "rail": m.Look(m.rgb(0.04, 0.04, 0.045), roughness=0.35),
    "body": m.Look(m.rgb(0.06, 0.06, 0.065), roughness=0.45),
    "metal": m.Look(m.rgb(0.30, 0.30, 0.31), roughness=0.3),
    "dring": m.Look(m.rgb(0.10, 0.10, 0.11), roughness=0.6),
    "button": m.Look(m.rgb(0.93, 0.93, 0.90), roughness=0.5),
    "table": m.Look(m.rgb(0.55, 0.45, 0.35), roughness=0.7),
}

# -- sizes: Toyota 73200-12070, scaled from the maker's part photo by its 7/16 in (11 mm) anchor stud (README, Real items) -------------------------------
# The board's own frame: origin at the anchor stud's axis on the board's face at the top notch; x into the board, y across, z up the rail.
O = np.array([0.30, 0.12, 1.16])  # that origin in the station frame (the stud 1.16 m above the ground at the top notch)
TILT = -10.0  # degrees about y: the board's top leans 10 degrees towards the arms
_a = math.radians(TILT)
R = np.array([[math.cos(_a), 0.0, math.sin(_a)], [0.0, 1.0, 0.0], [-math.sin(_a), 0.0, math.cos(_a)]])  # board to station
QUAT = (0.0, math.sin(_a / 2), 0.0, math.cos(_a / 2))  # xyzw
BOARD = (0.018, 0.30, 0.42)  # plywood: thickness, width, height
BOARD_Z = (-0.25, 0.17)  # its lower and upper edge along the rail
RAIL = (0.008, 0.031, -0.130, 0.062)  # the steel rail's channel: depth, width, lower and upper end of the channel
TAB_Z = (0.076, -0.144)  # its two flat end tabs' hole centres (15 mm holes, each with a mounting bolt)
TAB_R = 0.0155
PITCH = float(os.environ.get("ANCHOR_PITCH", "0.015"))  # notch pitch (range 15 to 20 mm; README, Ranges)
NOTCHES = 5  # top to bottom: 0, -15, -30, -45, -60 mm
LOW = -PITCH * (NOTCHES - 1)  # the lowest notch
SLIDER = (0.014, 0.041, 0.085)  # the slider body: depth, width, height
SLIDER_Z = -0.0205  # its centre below the stud (top 22 mm above, bottom 63 mm below)
FACE = -RAIL[0] - 0.0005 - SLIDER[0]  # the slider's face (it rides 0.5 mm off the rail)
BOSS = (0.010, 0.021, 0.023)  # the raised block above the button: proud of the face, width, height
BOSS_Z = -0.0295  # its centre (18 to 41 mm below the stud)
BTN = (0.006, 0.021, 0.014)  # the release button: depth, width, height
BTN_Z = -0.0616  # its centre below the stud (its lower edge 5.6 mm below the slider's)
BTN_OUT = 0.004  # it stands this far proud of the slider's face: its travel
BTN_K = float(os.environ.get("ANCHOR_BTNK", "2500"))  # N/m: 10 N pressed in full (range 625 to 2500 N/m, 2.5 to 10 N; README, Ranges)
FREE_AT = 0.002  # pressed in this far (5 N), the pawl is lifted
OUT_AT = 0.001  # back within this of rest, the pawl drops
RAIL_F = float(os.environ.get("ANCHOR_RAILF", "0.5"))  # the rail's joint friction on a free anchor (range 0.2 to 2; README, Ranges)
MASS = float(os.environ.get("ANCHOR_MASS", "0.22"))  # kg, the anchor's inertia (gravity off, weight left out): slider, stud, nut, collar, D-ring guide, button (a shown calculation, range 0.15 to 0.35; README)
LOCK_K, LOCK_E = 50000.0, 2000.0  # the pawl in a notch: N/m, N
COLLAR = (0.012, 0.012)  # the anchor bolt's collar and washer on the slider: radius, length
DRING_T = 0.005  # the D-ring guide's plate (steel in its plastic cover): thickness
DRING_DEG = -55.0  # it hangs from the bolt, turned this far from straight down towards the seat (-y), where the webbing runs to the shoulder
NUT = (0.0098, 0.009)  # the anchor nut (17 mm across flats): radius, length
SETTLE = 10  # steps (0.5 s) for the pawl to seat in its notch before FORCED applies
CATCH = float(os.environ.get("ANCHOR_CATCH", "0.0015"))  # the pawl catches a notch only when the anchor is this near it (range 0.5 to 1.5 mm)
FORCED = 0.003  # a latched anchor more than this off its notch was forced past the pawl: the run is void
CLEAR = 0.02  # the end: both whole grippers at least this far from the anchor


def W(p) -> np.ndarray:
    """A point in the board's frame, in the station frame."""
    return O + R @ np.asarray(p, float)


def _frame():
    """The plywood board, the adjuster's rail (channel, two end tabs) and its two mounting bolts: fixed."""
    parts, cols = [], []
    bc = (BOARD[0] / 2, 0.0, sum(BOARD_Z) / 2)
    bs = (BOARD[0], BOARD[1], BOARD_Z[1] - BOARD_Z[0])
    parts.append(("board", m.box(bs, bc), "board"))
    cols.append(m.col_box("board_c", bs, bc))
    rs, rc = (RAIL[0], RAIL[1], RAIL[3] - RAIL[2]), (-RAIL[0] / 2, 0.0, (RAIL[2] + RAIL[3]) / 2)
    parts.append(("rail", m.box(rs, rc), "rail"))
    cols.append(m.col_box("rail_c", rs, rc))
    for k, z in enumerate(TAB_Z):
        tab = m.cylinder(TAB_R, 0.003, (-0.0015, 0.0, z), axis="x")
        neck = m.box((0.003, 0.024, abs(z) - abs(RAIL[3] if z > 0 else RAIL[2]) + 0.004), (-0.0015, 0.0, (z + (RAIL[3] if z > 0 else RAIL[2])) / 2))
        bolt = m.cylinder(0.009, 0.006, (-0.006, 0.0, z), axis="x", segments=6)
        parts += [(f"tab{k}", tab, "rail"), (f"neck{k}", neck, "rail"), (f"mbolt{k}", bolt, "metal")]
    return parts, cols


def _dring():
    """The belt's D-ring guide, built hanging straight down from the bolt (an eye, a neck, a frame round the webbing slot), then turned towards the seat."""
    t = DRING_T
    pieces = [m.cylinder(0.014, t, (0.0, 0.0, 0.0), axis="x"), m.box((t, 0.022, 0.032), (0.0, 0.0, -0.016)),
              m.box((t, 0.060, 0.007), (0.0, 0.0, -0.0335)), m.box((t, 0.060, 0.007), (0.0, 0.0, -0.0505)),
              m.box((t, 0.005, 0.024), (0.0, 0.0275, -0.042)), m.box((t, 0.005, 0.024), (0.0, -0.0275, -0.042))]
    x = FACE - COLLAR[1] - t / 2
    return [p.turned(DRING_DEG, "x").moved((x, 0.0, 0.0)) for p in pieces]


def _anchor():
    """The anchor on the rail: slider body, raised block, collar, D-ring guide, nut and stud. Child: the release button."""
    sc = (FACE + SLIDER[0] / 2, 0.0, SLIDER_Z)
    boc = (FACE - BOSS[0] / 2, 0.0, BOSS_Z)
    collar = m.cylinder(COLLAR[0], COLLAR[1], (FACE - COLLAR[1] / 2, 0.0, 0.0), axis="x")
    nx = FACE - COLLAR[1] - DRING_T
    nut = m.cylinder(NUT[0], NUT[1], (nx - NUT[1] / 2, 0.0, 0.0), axis="x", segments=6)
    stud = m.cylinder(0.0055, 0.006, (nx - NUT[1] - 0.003, 0.0, 0.0), axis="x")
    parts = [("body", m.box(SLIDER, sc), "body"), ("boss", m.box(BOSS, boc), "body"), ("collar", collar, "metal"), ("nut", nut, "metal"), ("stud", stud, "metal")]
    cols = [m.col_box("body_c", SLIDER, sc), m.col_box("boss_c", BOSS, boc), m.col_hull("collar_c", collar), m.col_hull("nut_c", nut)]
    for k, p in enumerate(_dring()):
        parts.append((f"dring{k}", p, "dring"))
        cols.append(m.col_hull(f"dring_c{k}", p))
    anchor = m.slider("anchor", "Z", LOW, 0.0, parts, cols, at=(FACE, 0.0, 0.0), spring=LOCK_K, damping=150.0, friction=RAIL_F, effort=LOCK_E, mass=MASS)
    bc = (FACE - BTN_OUT + BTN[0] / 2, 0.0, BTN_Z)
    cap = m.box(BTN, bc)
    button = m.Link("button", "slide", "X", bc, 0.0, BTN_OUT, (("cap", cap, "button"),), (m.col_box("button_c", BTN, bc),), mass=0.01, spring=BTN_K, damping=2.0,
                    friction=0.0, parent="anchor")
    return [anchor, button]


def _stand():
    """Two 40 mm aluminium posts on feet behind the board, with a standoff to the board's back at two heights."""
    parts, cols = [], []
    px = 0.40
    for k, y in enumerate((O[1] - 0.12, O[1] + 0.12)):
        for name, size, c in ((f"post{k}", (0.04, 0.04, 1.30), (px, y, 0.65)), (f"foot{k}", (0.40, 0.04, 0.04), (px, y, 0.02))):
            parts.append((name, m.box(size, c), "post"))
            cols.append(m.col_box(f"{name}_c", size, c))
        for j, z in enumerate((0.95, 1.25)):
            back = W((BOARD[0], 0.0, (z - O[2]) / math.cos(_a)))[0]
            size, c = (px - 0.02 - back + 0.004, 0.04, 0.03), ((back + px - 0.02) / 2, y, z)
            parts.append((f"standoff{k}{j}", m.box(size, c), "post"))
    return m.fixed("stand", (0.0, 0.0, 0.0), parts, cols, LOOKS)


def _arms_table():
    """The station's table cut back to a strip under the two arm bases."""
    top_t, x0, x1, yh, leg = 0.06, -0.19, 0.14, 0.65, 0.05
    parts = [("top", m.box((x1 - x0, 2 * yh, top_t), ((x0 + x1) / 2, 0.0, TOP - top_t / 2)), "table")]
    cols = [m.col_box("top_c", (x1 - x0, 2 * yh, top_t), ((x0 + x1) / 2, 0.0, TOP - top_t / 2))]
    for k, (x, y) in enumerate([(x0 + leg / 2, yh - leg / 2), (x0 + leg / 2, -yh + leg / 2), (x1 - leg / 2, yh - leg / 2), (x1 - leg / 2, -yh + leg / 2)]):
        size, c = (leg, leg, TOP - top_t), (x, y, (TOP - top_t) / 2)
        parts.append((f"leg{k}", m.box(size, c), "table"))
        cols.append(m.col_box(f"leg_c{k}", size, c))
    return m.fixed("arms_table", (0.0, 0.0, 0.0), parts, cols, LOOKS)


def _no_station_table(scene) -> None:
    """Take the station's own table out of the room (the arms keep their poses, on the cut-back table above)."""
    if getattr(scene, "_no_station_table", False):
        return
    scene._no_station_table = True
    room = scene.task_room

    def task_room(*a, **k):
        assets = room(*a, **k)
        if (a[0] if a else k.get("scene")) == "station":
            if sum(getattr(x, "name", None) == "table" for x in assets) != 1:
                raise RuntimeError("expected one station table")
            return [x for x in assets if getattr(x, "name", None) != "table"]
        return assets

    scene.task_room = task_room


if "yam_arena.scene" in sys.modules:
    _no_station_table(sys.modules["yam_arena.scene"])
_patch0 = m._Finder.targets["yam_arena.scene"]
m._Finder.targets["yam_arena.scene"] = lambda mod: (_patch0(mod), _no_station_table(mod))

_fp, _fc = _frame()
PILLAR = m.mechanism("pillar", tuple(O), _anchor(), LOOKS, frame_parts=_fp, frame_colliders=_fc, friction=(1.0, 0.9), solver_iters=64, start_quat=QUAT)
OBJECTS = [PILLAR, _stand(), _arms_table()]


# -- the lock (the pawl) and the scorer ------------------------------------------------------------------------------------------------------------

def notch_of(z: float) -> int:
    """The nearest notch's index (0 top, 4 lowest) to anchor travel `z` (m, <= 0)."""
    return int(min(NOTCHES - 1, max(0, round(-z / PITCH))))


class Pawl:
    """Latched: the anchor's own stiff spring holds it at a notch. The button in past FREE_AT lifts the pawl: the anchor slides free (rail friction only).
    The button back out within OUT_AT: the pawl drops into the nearest notch once the anchor is within CATCH of it; until then the anchor rides free."""

    def reset(self, env=None):
        self.latched, self.notch = True, 0

    def step(self, env, z: float, b: float):
        if self.latched and b >= FREE_AT:
            m.lock(env, "pillar", "anchor", stiffness=0.0, effort=LOCK_E)
            self.latched = False
        elif not self.latched and b <= OUT_AT and abs(z + PITCH * notch_of(z)) <= CATCH:
            self.notch = notch_of(z)
            m.lock(env, "pillar", "anchor", stiffness=LOCK_K, effort=LOCK_E)
            m.set_target(env, "pillar", "anchor", -PITCH * self.notch)
            self.latched = True


PAWL = Pawl()


def gripper_points(env) -> np.ndarray:
    """Points over the whole of both grippers (fingers, palm, housing, and the wrist link's end), station frame: a grid on each, at the current opening."""
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
        for sx in (-1.0, 1.0):  # each finger: a 5 x 4 x 2 grid over its plate
            for dy in np.linspace(-0.02, 0.02, 5):
                for dz in np.linspace(0.095, 0.152, 4):
                    for t in (0.0, 0.009):
                        pts.append((sx * (opening + 0.0045 + t), dy, dz))
        for x in np.linspace(-0.05, 0.05, 6):  # the palm and housing: a box about 10 x 6 x 10 cm behind the fingers
            for y in np.linspace(-0.03, 0.03, 4):
                for z in np.linspace(-0.01, 0.095, 6):
                    pts.append((x, y, z))
        out.append(pose[:3] - origin + np.asarray(pts) @ R.T)
    return np.vstack(out)


def anchor_gap(pts: np.ndarray, z: float) -> float:
    """The least distance (m) from station points to a box round the whole anchor (slider body, raised block, button, collar, nut, stud and D-ring guide)
    at travel `z`, measured in the board's frame."""
    loc = (pts - O) @ R
    lo = np.array([FACE - COLLAR[1] - DRING_T - NUT[1] - 0.006, -0.065, z + min(BTN_Z - BTN[2] / 2, -0.060)])
    hi = np.array([0.0, SLIDER[1] / 2, z + SLIDER_Z + SLIDER[2] / 2])
    return float(np.linalg.norm(np.maximum(0.0, np.maximum(lo - loc, loc - hi)), axis=1).min())


def judge(c: dict) -> float:
    """The score from the run's record `c`. 0 if void: a latched anchor forced more than 3 mm off its notch (0.5 s after the pawl dropped).
    0.33 moved: the anchor now at least 10 mm below its top notch (it can move only while the button is in, lifting the pawl).
    0.67 at the bottom: the anchor now within 3 mm of the lowest notch.
    1.0 locked low: the button back out, the pawl caught in the lowest notch, both whole grippers at least 2 cm from the anchor, held 3 s."""
    if c["forced"]:
        return 0.0
    if c["locked_low"] and c["clear"] and c["held"] >= HOLD_STEPS:
        return 1.0
    if c["bottom"]:
        return 0.67
    if c["moved"]:
        return 0.33
    return 0.0


class Trial:
    def reset(self, env=None):
        PAWL.reset(env)
        self.c = {"forced": False, "moved": False, "bottom": False, "locked_low": False, "clear": False, "held": 0}
        self.n = 0
        self.since = SETTLE + 1

    def step(self, env):
        self.n += 1
        st = m.state(env, "pillar")
        z, b = st.joint("anchor"), st.joint("button")
        PAWL.step(env, z, b)
        c = self.c
        if PAWL.latched:
            self.since += 1
            c["forced"] = c["forced"] or (self.since > SETTLE and abs(z + PITCH * PAWL.notch) > FORCED)
        else:
            self.since = 0
        c["moved"] = z <= -0.010  # read from the anchor's place now: it moves only while the button holds the pawl up
        c["bottom"] = z <= LOW + 0.003
        c["locked_low"] = PAWL.latched and PAWL.notch == NOTCHES - 1 and b <= OUT_AT
        c["clear"] = c["locked_low"] and anchor_gap(gripper_points(env), z) >= CLEAR
        c["held"] = c["held"] + 1 if (c["locked_low"] and c["clear"]) else 0
        if TRACE and self.n % 20 == 0:
            print(f"[anchor] step {self.n}: z {1000 * z:.1f} mm, button {1000 * b:.2f} mm, latched {PAWL.latched} notch {PAWL.notch}, {c}, score {judge(c)}", flush=True)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return judge(TRIAL.c) >= 1.0


def score(objects) -> float:
    return judge(TRIAL.c)



# -- the demo: the left hand's closed fingertips press the button and, with their top edge under the raised block, carry the anchor down the rail ------
N = R @ np.array([1.0, 0.0, 0.0])  # into the board
TIP = float(os.environ.get("ANCHOR_TIP", "0.015"))  # TCP to the fingertips' ends along the approach
TURN = float(os.environ.get("ANCHOR_TURN", "90"))  # the roll that brings the fingers' closing axis vertical
FY = float(os.environ.get("ANCHOR_FY", "0.016"))  # the fingers' centre across the rail: over the button, clear of the D-ring guide
INTO = float(os.environ.get("ANCHOR_INTO", "0.010"))  # m: the hand's command stands this far into the board while it slides, to keep the button in
SLIDE_T = float(os.environ.get("ANCHOR_SLIDE_T", "2.5"))  # s for the slide down


def demo_script(ctx):
    """Modes: full; none (the arms wait); no_button (the anchor pushed down from on top of the slider, the button never pressed);
    half (pressed, slid two notches, let go); short (let go at 52 mm down, 8 mm above the lowest notch); hold (pressed and slid to the bottom, the button never let go); tap (pressed and let go, no slide)."""
    mode = ctx.mode.split("@")[0]
    if mode not in ("full", "none", "no_button", "half", "hold", "tap", "short"):
        raise ValueError(f"unknown mode {mode!r}")
    P = ctx.left
    env = ctx.emb._env
    dry = getattr(env, "unwrapped", None) is None
    if mode == "none":
        yield from P.wait(3.0)
        return

    def info(tag):
        if dry:
            return
        st = m.state(env, "pillar")
        ft = m.fingertips(env)[:12]
        loc = (ft - O) @ R  # the fingertips in the board's frame
        close = ft[6:].mean(0) - ft[:6].mean(0)
        aim = ft.mean(0) - P.tcp()
        print(f"[demo] {tag}: fingers (board frame) {np.round(loc.min(0), 4)}..{np.round(loc.max(0), 4)} closing {np.round(close / (np.linalg.norm(close) + 1e-9), 2)} "
              f"aim {np.round(aim / (np.linalg.norm(aim) + 1e-9), 3)} (normal {np.round(N, 3)}); z {1000 * st.joint('anchor'):.1f} button {1000 * st.joint('button'):.2f} "
              f"latched {PAWL.latched} notch {PAWL.notch} score {judge(TRIAL.c)}", flush=True)

    G = d.level(0.0)
    if mode == "no_button":  # the fingertips on top of the slider body, beside the rail
        target = W([FACE + 0.006 - TIP, FY, SLIDER_Z + SLIDER[2] / 2 + 0.0135 + 0.003])
    else:  # the fingertips' centre over the button, their top edge just under the raised block
        target = W([FACE - BTN_OUT - TIP, FY, BOSS_Z - BOSS[2] / 2 - 0.0135 - 0.002])
    yield from P.move(target - 0.08 * N, grip=G, finger=d.FINGER_CLOSED, label="before the anchor")
    yield from P.turn(TURN, axis=(1.0, 0.0, 0.0), t=0.8, label="fingers flat, closing along the rail")
    yield from P.move(P.at() + (target - 0.01 * N - P.tcp()), t=0.8, label="at the button")
    if not dry:  # square the fingers to the rail and centre them, from where the fingertips are seen to be
        want = np.array([FY, (target - O) @ R[:, 2]])
        for k in range(3):
            ft = m.fingertips(env)[:12]
            c = R.T @ (ft[6:].mean(0) - ft[:6].mean(0))
            c = c if c[2] > 0 else -c
            ep, er = math.degrees(math.atan2(c[0], c[2])), math.degrees(math.atan2(c[1], c[2]))
            err = want - ((ft - O) @ R).mean(0)[1:]
            print(f"[demo] square {k}: pitch {ep:.1f} deg, roll {er:.1f} deg, off {np.round(1000 * err, 1)} mm", flush=True)
            if abs(ep) > 1.5:
                yield from P.turn(-ep, axis=tuple(R[:, 1]), t=0.4, label="square: pitch")
            if abs(er) > 1.5:
                yield from P.turn(er, axis=tuple(N), t=0.4, label="square: roll")
            if np.abs(err).max() > 0.0015:
                yield from P.move(P.at() + R @ [0.0, err[0], err[1]], t=0.4, label="square: centre")
    info("at the button")
    if mode == "no_button":
        yield from P.move(P.at() + 0.016 * N, t=0.5, label="over the slider")
        res = yield from P.press(tuple(-R[:, 2]), 30.0, 0.07, label="push the anchor down")
        info(f"pushed down ({res['why']})")
    else:
        res = yield from P.press(tuple(N), 14.0, 0.03, label="press the button in")
        info(f"pressed ({res['why']})")
        goal = 0.0 if mode == "tap" else -2 * PITCH if mode == "half" else -0.052 if mode == "short" else LOW
        start = P.tcp().copy()
        for k in range(0 if mode == "tap" else 4):  # keep pushing in (the command stands INTO past the face) while carrying the anchor down by its raised block
            z = m.state(env, "pillar").joint("anchor") if not dry else goal
            if z <= goal + 0.002:
                break
            yield from P.move(start + R @ [INTO, 0.0, goal - (0.006 + 0.008 * k if goal == LOW else 0.0)], t=SLIDE_T, label="slide the anchor down")
            info(f"slid {k}")
        if mode == "hold":
            yield from P.wait(5.0)
            info("held")
            return
    yield from P.move(P.at() - 0.03 * N, t=0.5, label="let the button go")
    info("let go")
    yield from P.move(P.at() - 0.08 * N + [0.0, 0.03, 0.0], t=1.0, label="away")
    yield from P.wait(4.0)
    info("end")
