"""Seat a 100 mm silicon wafer on a contact mask aligner's vacuum chuck: push it by its rim until its flat lies on the two close pre-alignment
pins and its edge on the third, press the vacuum key, then push the transport slide in by its handle to the end stop.

One articulation, `aligner` (fixed in place), holds the base block, the vacuum key (a sprung button) and the transport slide: a plate on rails with
the 120 mm chuck, its three 2 mm pins 0.4 mm proud of the chuck's face, and the handle bar. The wafer is a free 9.6 g rigid disc lying on the chuck.
The part's origin is the chuck's centre on its face when the slide is pulled out, so that a point in the part's frame is a point in the chuck's frame.

Debug:  python demokit.py task.py OUT --no-record --mode full,none,centre,face,slidefirst
Final:  python demokit.py task.py OUT
Model:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1800 --instruction "<INSTRUCTION below>"
"""

from __future__ import annotations

import collections
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demokit as d  # noqa: E402
import gatekit as gk  # noqa: E402
import mechkit as m  # noqa: E402

SCENE = "station"
INSTRUCTION = ("The wafer has been set down on the mask aligner's chuck. Seat it against the pre-alignment pins: push it only by its edge until its flat rests "
               "against the two close pins at the front and its rim against the pin on the left, without pushing it up onto a pin and without touching its "
               "top face more than 3 mm in from the rim. Then press the vacuum key, push the transport slide in by its handle to its end stop, and move both hands clear.")
SUCCESS_HOLD_S = 0.0  # the scorer holds the end state itself (HOLD_STEPS)
SCORE_OVER = "end"
TOP = d.TABLE_TOP
DT = d.DT
HOLD_STEPS = 20  # 1 s at 20 Hz

LOOKS = {
    "alu": m.Look(m.rgb(0.62, 0.64, 0.67), roughness=0.35, metallic=0.9),
    "base": m.Look(m.rgb(0.22, 0.24, 0.28), roughness=0.6),
    "steel": m.Look(m.rgb(0.80, 0.80, 0.82), roughness=0.25, metallic=1.0),
    "key": m.Look(m.rgb(0.75, 0.12, 0.10), roughness=0.5),
    "handle": m.Look(m.rgb(0.10, 0.10, 0.12), roughness=0.7),
    "wafer": m.Look(m.rgb(0.30, 0.34, 0.42), roughness=0.12, metallic=0.6),
}

# -- the aligner, in the chuck's frame: x away from the robot, y to its left, z up; origin on the chuck's face, centre, slide pulled out ----------
CHUCK_AT = (0.30, 0.0)
FACE = TOP + 0.070  # chuck face above the table
R_CHUCK, H_CHUCK = 0.060, 0.010
PLATE = (0.160, 0.140, 0.010)  # top 6.0 cm up
BLOCK = (0.300, 0.250, 0.050)  # centred at x 0.50, top 5.0 cm up
TRAVEL = 0.150
SLIDE_FRICTION = 3.0  # N
M_SLIDE = 1.0  # kg: plate 0.6, chuck 0.3, handle
R_PIN, PIN_H = 0.001, 0.0004
PINS = ((-0.0483, -0.010), (-0.0483, 0.010), (0.0, 0.051))  # centres, chuck frame: the two close pins, then the left pin
R_W, T_W, FLAT = 0.050, 0.000525, 0.0325
D_FLAT = math.sqrt(R_W ** 2 - (FLAT / 2) ** 2)  # 47.28 mm: the flat's distance from the centre
M_W = 0.0096
START_W = (0.004, -0.003, -8.0)  # wafer centre (toward the robot is -x) and turn, from seated; the card's start: on no pin
KEY_AT = (0.40 - CHUCK_AT[0], 0.10)  # on the base's top, beside the slide
KEY_TRAVEL, KEY_N = 0.004, 3.0
HANDLE_X = 0.21 - CHUCK_AT[0]
BASE_TOP = TOP + 0.050 - FACE  # base block's top, chuck frame
GAP_OK = 0.0003  # a feeler of 0.3 mm does not pass
VACUUM_N = 30.0  # the chuck's hold, downward on the wafer once the key is pressed
FACE_FREE = 0.003  # the outer 3 mm of the wafer may be touched
HANDS_CLEAR = 0.05
YAW_MAX = math.radians(1.0)  # the flat faces the close pins within 1 degree
FLICK_SPEED = 0.10  # m/s: a pushed wafer never moves this fast
HANDLE_REACH = 0.020  # a fingertip within 20 mm of the handle bar counts as pushing the slide
BENCH_Y = 0.60


def _wafer_outline(n: int = 96) -> np.ndarray:
    pts = [(R_W * math.cos(a), R_W * math.sin(a)) for a in np.linspace(0.0, 2 * math.pi, n, endpoint=False)]
    out = []
    for x, y in pts:
        out.append((max(x, -D_FLAT), y) if x < -D_FLAT else (x, y))
    keep = [out[0]]
    for p in out[1:]:  # drop the repeated points the cut makes
        if np.hypot(p[0] - keep[-1][0], p[1] - keep[-1][1]) > 1e-6:
            keep.append(p)
    return np.array(keep)


def _slide() -> m.Link:
    px, pz = 0.0, -0.010 - PLATE[2] / 2  # plate centre in the chuck frame
    plate = m.box(PLATE, (px, 0.0, pz))
    chuck = m.cylinder(R_CHUCK, H_CHUCK, (0.0, 0.0, -H_CHUCK / 2), axis="z", segments=64)
    hx = HANDLE_X
    handle = m.box((0.012, 0.080, 0.012), (hx, 0.0, -0.010 + 0.006))
    neck = m.box((0.010, 0.040, 0.006), (hx + 0.008, 0.0, -0.013))
    parts = [("plate", plate, "alu"), ("chuck", chuck, "alu"), ("handle", handle, "handle"), ("neck", neck, "alu")]
    cols = [m.col_box("plate_c", PLATE, (px, 0.0, pz)), m.col_cylinder("chuck_c", R_CHUCK, H_CHUCK, (0.0, 0.0, -H_CHUCK / 2)),
            m.col_box("handle_c", (0.012, 0.080, 0.012), (hx, 0.0, -0.010 + 0.006)), m.col_box("neck_c", (0.010, 0.040, 0.006), (hx + 0.008, 0.0, -0.013))]
    for k, (x, y) in enumerate(PINS):
        parts.append((f"pin{k}", m.cylinder(R_PIN, PIN_H + 0.001, (x, y, PIN_H / 2 - 0.0005), axis="z", segments=12), "steel"))
        cols.append(m.col_cylinder(f"pin{k}_c", R_PIN, 0.003, (x, y, PIN_H - 0.0015)))  # 3 mm tall, its top 0.4 mm proud
    return m.slider("slide", "X", 0.0, TRAVEL, parts, cols, mass=M_SLIDE, spring=0.0, damping=4.0, friction=SLIDE_FRICTION, effort=2000.0, armature=0.0)


def _key() -> m.Link:
    return m.button("key", (KEY_AT[0], KEY_AT[1], BASE_TOP), outward="+Z", look="key", radius=0.008, height=0.008, travel=KEY_TRAVEL,
                    spring=KEY_N / KEY_TRAVEL, damping=2.0, mass=0.02)


def _aligner() -> m.Part:
    bx = 0.50 - CHUCK_AT[0]
    frame = [("block", m.box(BLOCK, (bx, 0.0, BASE_TOP - BLOCK[2] / 2 + 0.0)), "base")]
    cols = [m.col_box("block_c", BLOCK, (bx, 0.0, BASE_TOP - BLOCK[2] / 2))]
    for s in (-1.0, 1.0):  # the two rails along x (looks only: the plate rides on them)
        frame.append((f"rail{0 if s < 0 else 1}", m.box((0.45, 0.012, 0.004), (0.5 * (0.20 + 0.65) - CHUCK_AT[0], s * 0.045, BASE_TOP + 0.002)), "steel"))
    part = m.mechanism("aligner", (*CHUCK_AT, FACE), [_slide(), _key()], LOOKS, frame_parts=frame, frame_colliders=cols, friction=(0.3, 0.3))
    return part


def _wafer_cols(mesh) -> list[str]:
    """The wafer's collider: a convex hull of so thin a disc is cooked with a tilt of 0.6 degrees (README 25), so the disc is 1.5 mm strips of boxes along x, each as long as the
    disc is across at its outer edge (a box is exact where a hull of this thinness is not)."""
    kind = os.environ.get("WAFER_COL", "strips")
    if kind == "hull":
        return [m.col_hull("wafer_c", mesh)]
    w = 0.0015
    cols = []
    edges = np.linspace(-R_W, R_W, int(round(2 * R_W / w)) + 1)
    for k, (y0, y1) in enumerate(zip(edges[:-1], edges[1:])):
        ym = min(abs(y0), abs(y1)) if y0 * y1 > 0 else 0.0
        half = math.sqrt(max(R_W ** 2 - ym ** 2, 0.0))  # the strip's reach at its inner edge, so that it spans the circle
        x0 = -min(half, D_FLAT)
        cols.append(m.col_box(f"wafer_c{k}", (half - x0, w, T_W), ((half + x0) / 2, (y0 + y1) / 2, 0.0)))
    return cols


def _wafer() -> m.Part:
    out = _wafer_outline()
    mesh = m.prism(out, -T_W / 2, T_W / 2)
    return m.free("wafer", (CHUCK_AT[0] + START_W[0], CHUCK_AT[1] + START_W[1], FACE + T_W / 2 + 0.00005), [("disc", mesh, "wafer")], _wafer_cols(mesh), LOOKS,
                  mass=M_W, friction=(0.3, 0.3), yaw_deg=START_W[2])


OBJECTS = [_aligner(), _wafer()]


# -- reading the scene -------------------------------------------------------------------------------------------------------------------------


def chuck_origin(env) -> np.ndarray:
    """Where the chuck's centre on its face is now (station frame): the slide link's origin."""
    base = env.unwrapped
    asset = base.scene["aligner"]
    i = list(asset.body_names).index("slide")
    return m._np(asset.data.body_link_pose_w)[0, i][:3] - m._np(base.scene.env_origins[0])


def dist_to_wafer(P, c, yaw) -> float:
    """Distance from a point P (xy, chuck frame) to the wafer's outline (a 100 mm disc cut by the flat), the wafer centred at c and turned `yaw`."""
    P, c = np.asarray(P, float)[:2], np.asarray(c, float)[:2]
    cs, sn = math.cos(yaw), math.sin(yaw)
    n = np.array([-cs, -sn])  # the flat's outward normal
    t = np.array([-sn, cs])
    q = P - c
    cands = []
    r = np.linalg.norm(q)
    if r > 1e-9:
        q1 = q / r * R_W
        if n @ q1 <= D_FLAT + 1e-9:
            cands.append(np.linalg.norm(q - q1))
    s = float(np.clip(t @ q, -FLAT / 2, FLAT / 2))
    q2 = n * D_FLAT + t * s
    cands.append(np.linalg.norm(q - q2))
    inside = r < R_W and n @ q < D_FLAT
    dd = min(cands)
    return -dd if inside else dd


class Seen:
    """What the scorer reads at one step, in the chuck's frame (metres, radians): the wafer's centre c = (x, y), its turn `yaw`, its centre's height `h` above
    lying on the face, its tilt, the slide's travel, the key's travel, the 24 fingertip points `tips`, and whether the vacuum is on."""

    def __init__(self, c, yaw, h, tilt_deg, slide, key, tips, vacuum=False):
        self.c, self.yaw, self.h, self.tilt_deg = np.asarray(c, float)[:2], float(yaw), float(h), float(tilt_deg)
        self.slide, self.key, self.tips, self.vacuum = float(slide), float(key), np.atleast_2d(np.asarray(tips, float)), bool(vacuum)

    def gaps(self) -> tuple[float, float, float]:
        """Wafer-to-pin gaps (m): the two close pins, then the left pin."""
        return tuple(dist_to_wafer(p, self.c, self.yaw) - R_PIN for p in PINS)

    def touching(self) -> tuple[bool, bool, bool]:
        return tuple(-0.0005 <= g <= GAP_OK for g in self.gaps())

    def flat_on_chuck(self) -> bool:
        return abs(self.h) <= 0.0001 and self.tilt_deg <= 0.3 and abs(self.yaw) <= YAW_MAX

    def face_touch(self) -> bool:
        """A hand's fingertip blade (the centre of its jaws' lower points) inside the wafer's outline shrunk by 3 mm, with its lower points no higher than 0.7 mm over
        the wafer's top face: a finger on the face more than 3 mm in. A blade pushing the rim has its centre outside the outline."""
        for hand in (self.tips[:12], self.tips[12:]):
            if len(hand) < 12:
                continue
            low = hand[0::2]  # the lower row: the finger ends
            zmax = T_W + 0.0007 + max(self.h, 0.0)
            b = low.mean(axis=0)
            if dist_to_wafer(b[:2], self.c, self.yaw) < -FACE_FREE and low[:, 2].min() < zmax:
                return True
            for p in low:  # and no single point of the blade more than 3 mm + the points' spread (3 mm) onto the face
                if dist_to_wafer(p[:2], self.c, self.yaw) < -(FACE_FREE + 0.003) and T_W - 0.0001 <= p[2] < zmax:  # below the top face is inside the solid: not a touch
                    return True
        return False


def key_armed(s: Seen) -> bool:
    """The key counts only with the wafer seated: flat on both close pins and the rim on the left pin."""
    return bool(all(s.touching()) and s.flat_on_chuck())


def handle_touched(tips) -> bool:
    """A fingertip within 20 mm of the handle bar (x = HANDLE_X, |y| <= 4 cm), in plan and below the plate's top plus 3 cm."""
    t = np.atleast_2d(np.asarray(tips, float))
    dx = np.abs(t[:, 0] - HANDLE_X)
    dy = np.maximum(np.abs(t[:, 1]) - 0.04, 0.0)
    return bool(np.any((np.hypot(dx, dy) < HANDLE_REACH) & (t[:, 2] < 0.03)))


def key_free(s: Seen) -> bool:
    return s.key > -0.5 * KEY_TRAVEL  # no hand is holding the key down


def too_fast(speed: float) -> bool:
    return speed > FLICK_SPEED


def failed(s: Seen) -> bool:
    """On a pin (the wafer's centre 0.25 mm up, or tilted over 1 degree), off the chuck, or the face touched more than 3 mm in: latched by the trial."""
    off = float(np.linalg.norm(s.c)) > 0.025
    return bool(s.h > 0.00025 or s.tilt_deg > 1.0 or off or s.face_touch())


LADDER = gk.Ladder(("one_pin", 0.25), ("flat", 0.5), ("seated", 0.75), ("done", 1.0))
SLIDE_DONE = TRAVEL - 0.001


def judge(s: Seen, clear: bool, still: bool, held: int, dead: bool = False, handled: bool = True) -> tuple[bool, float]:
    """(full, score) for one step. clear: every fingertip 5 cm from the wafer and the handle."""
    if dead:
        return False, 0.0
    a, b, left = s.touching()
    one = a or b
    flat = a and b and s.flat_on_chuck()
    seated = flat and left and s.vacuum
    full = bool(seated and s.slide >= SLIDE_DONE and clear and still and handled)
    return full, LADDER.score({"one_pin": one, "flat": flat, "seated": seated, "done": full and held >= HOLD_STEPS})


class Trial:
    def reset(self, env=None) -> None:
        self.steps = 0
        self.vacuum = False
        self.failed = False
        self.risen = 0
        self.full, self.score, self.held = False, 0.0, 0
        self.trail = collections.deque(maxlen=HOLD_STEPS + 1)
        self.seen = None
        self.peak_h = 0.0
        self.handled = False
        self.fast = False

    def read(self, env) -> Seen:
        w = m.state(env, "wafer")
        al = m.state(env, "aligner")
        o = chuck_origin(env)
        c = w.pos - o
        R = w.rotation()
        yaw = math.atan2(R[1, 0], R[0, 0])
        tips = m.fingertips(env) - o
        return Seen(c[:2], yaw, c[2] - T_W / 2, w.tilt_deg(), al.joint("slide"), al.joint("key"), tips, self.vacuum)

    def step(self, env) -> None:
        self.steps += 1
        s = self.seen = self.read(env)
        self.peak_h = max(self.peak_h, s.h)
        if not self.vacuum and s.key <= -0.9 * KEY_TRAVEL and not self.failed and key_armed(s):
            self.vacuum = True
            s.vacuum = True
            print(f"[aligner] vacuum on at step {self.steps}: gaps {np.round(np.array(s.gaps()) * 1000, 2)} mm", flush=True)
        if self.vacuum:
            m.push(env, "wafer", (0.0, 0.0, -VACUUM_N))
        self.risen = self.risen + 1 if (s.h > 0.00025 or s.tilt_deg > 1.0) else 0
        self.fast = self.fast or too_fast(float(np.linalg.norm(m.state(env, "wafer").lin_vel)))
        if s.slide > 0.001 and handle_touched(s.tips):
            self.handled = True
        bad = self.fast or self.risen >= 2 or s.h > 0.0004 or s.tilt_deg > 3.0 or float(np.linalg.norm(s.c)) > 0.025 or s.face_touch() or (s.slide > 0.005 and not self.vacuum)
        if bad and not self.failed and os.environ.get("ALIGNER_LOG"):
            for hand in (s.tips[:12], s.tips[12:]):
                lw = hand[0::2]
                print("[aligner] low pts", [(round(float(dist_to_wafer(p[:2], s.c, s.yaw)) * 1000, 1), round(float(p[2]) * 1000, 1)) for p in lw], flush=True)
            print(f"[aligner] failed at step {self.steps}: fast {self.fast} risen {self.risen} off {float(np.linalg.norm(s.c)) > 0.025} face {s.face_touch()} slide {s.slide:.4f} vacuum {self.vacuum}", flush=True)
        self.failed = self.failed or bad
        wafer_line = np.array([[-R_W, 0.0, 0.0], [0.0, 0.0, 0.0], [R_W, 0.0, 0.0], [0.0, -R_W, 0.0], [0.0, R_W, 0.0]])
        wafer_line = wafer_line + np.r_[s.c, 0.0]
        near = gk.min_distance(s.tips, [wafer_line, np.array([[HANDLE_X, -0.04, 0.0], [HANDLE_X, 0.04, 0.0]])], 0.0)
        clear = bool(near > HANDS_CLEAR and key_free(s))
        self.trail.append(np.r_[s.c, s.slide, s.key])
        arr = np.array(self.trail)
        still = len(arr) > HOLD_STEPS and float(np.abs(arr - arr[-1]).max()) <= 0.0005
        full, _ = judge(s, clear, still, self.held + 1, self.failed, self.handled)
        self.held = self.held + 1 if full else 0
        self.full, self.score = judge(s, clear, still, self.held, self.failed, self.handled)
        if os.environ.get("ALIGNER_LOG") and self.steps % int(os.environ["ALIGNER_LOG"]) == 0:
            g = np.round(np.array(s.gaps()) * 1000, 2)
            print(f"[aligner] {self.steps}: c {np.round(s.c * 1000, 2)} yaw {math.degrees(s.yaw):.2f} h {s.h * 1000:.3f} tilt {s.tilt_deg:.2f} gaps {g} "
                  f"zaxis {np.round(m.state(env, 'wafer').rotation()[:, 2], 4)} lin {np.round(m.state(env, 'wafer').lin_vel * 1000, 3)} slide {s.slide * 1000:.1f} key {s.key * 1000:.1f} vac {self.vacuum} near {near * 1000:.0f} failed {self.failed} score {self.score}", flush=True)
        if self.held == HOLD_STEPS:
            print(f"[aligner] full state held {HOLD_STEPS * DT:.1f} s: score {self.score}", flush=True)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.score >= 1.0


def score(objects) -> float:
    return TRIAL.score


# -- the demo ----------------------------------------------------------------------------------------------------------------------------------

TIP = 0.016  # the closed fingers' ends stand this far past the TCP along the fingers (README 22: the real underside is about 5 mm higher)
BLADE = 0.002  # half the closed fingers' thickness, as the pushes show it (the points `fingertips()` reports lie wider: README 22)
HALF_W = 0.010  # and half the blade's width
LEFT_AWAY = (0.16, 0.30, TOP + 0.20)


def at_chuck(env=None, x=0.0, y=0.0, z=0.0, o=None) -> np.ndarray:
    base = np.array([CHUCK_AT[0], CHUCK_AT[1], FACE]) if o is None else np.asarray(o, float)
    return base + np.array([x, y, z])


def tipinfo(ctx, env) -> str:
    t = m.fingertips(env)[:12] - chuck_origin(env)
    lo = t[:, 2].min()
    low = t[t[:, 2] < lo + 0.004]
    return f"left tips lowest z {lo * 1000:.1f} mm at x {low[:, 0].min() * 1000:.0f}..{low[:, 0].max() * 1000:.0f} y {low[:, 1].min() * 1000:.0f}..{low[:, 1].max() * 1000:.0f}; TCP {np.round((ctx.left.tcp() - chuck_origin(env)) * 1000, 1)}"


def drive(l, grip, path, label, until=None, steps_max=400):
    """Command the TCP along `path` (a generator of positions, one per control step) until `until()` or the path ends."""
    for k, p in enumerate(path):
        mm, deg = l._solve(np.asarray(p, float), grip.rot)
        l._note(mm, deg, label)
        yield
        if os.environ.get("DRIVE_LOG") and k % 4 == 0:
            e = l.ctx.emb._env
            t_ = m.fingertips(e)[:12] - chuck_origin(e)
            print(f"[aligner] {label} {k}: cmd {np.round((p - chuck_origin(e)) * 1000, 1)} tcp {np.round((l.tcp() - chuck_origin(e)) * 1000, 1)} tip z {t_[:, 2].min() * 1000:.1f} wafer {np.round(TRIAL.seen.c * 1000, 1)} yaw {math.degrees(TRIAL.seen.yaw):.1f} h {TRIAL.seen.h * 1e6:.0f}um tilt {TRIAL.seen.tilt_deg:.2f} gaps {np.round(np.array(TRIAL.seen.gaps()) * 1000, 2)}", flush=True)
        if until is not None and until():
            return


def ramp(p0, p1, step):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    n = max(1, int(np.ceil(np.linalg.norm(p1 - p0) / step)))
    for k in range(1, n + 1):
        yield p0 + (p1 - p0) * k / n


AL_Z = float(os.environ.get("AL_Z", "0.0125"))  # TCP height over the chuck face when a fingertip blade rides on it (its real underside is about 5 mm above what fingertips() reports)
HOVER_Z = 0.020


def demo_script(ctx):
    """Modes: full. none: the arms wait. centre: the first push straight at the wafer's centre line. face: a fingertip dragged over the wafer's face.
    slidefirst: the slide pushed in before the key. nokey: seated, the slide pushed in, the key never pressed."""
    mode = ctx.mode
    l = ctx.left
    env = ctx.emb._env
    if mode == "none":
        yield from l.wait(3.0)
        return
    X, Y = d.down(-90.0), d.down(0.0)
    shut = d.FINGER_CLOSED
    ox, oy = CHUCK_AT

    def world(x, y, z):  # a chuck-frame point at the chuck's present place
        return chuck_origin(env) + np.array([x, y, z])

    T = TRIAL
    yield from l.move((0.20, 0.15, FACE + 0.15), grip=X, finger=shut, t=1.2, label="via")
    if mode == "face":  # a fingertip pressed on the wafer's face and dragged toward the robot
        c0 = world(T.seen.c[0] + 0.015, T.seen.c[1], HOVER_Z)
        yield from l.move(c0 + [0, 0, 0.03], t=1.2, label="above the wafer")
        yield from l.move(c0 + [0, 0, AL_Z - HOVER_Z + T_W + 0.0005], t=0.8, label="onto the face")
        yield from drive(l, X, ramp(c0 + [0, 0, AL_Z - HOVER_Z + T_W + 0.0005], c0 + [-0.012, 0, AL_Z - HOVER_Z + T_W + 0.0005], 0.0004), "drag", until=None)
        yield from l.move(l.at() + [0, 0, 0.05], t=0.6, label="up")
    if mode == "slidefirst":
        pass
    else:
        # 1. Push toward the robot from the far side, a little left of the wafer's centre, so that the flat meets the pins and the push turns it onto both.
        def push_x(label, stop):
            """The blade (about 20 mm across, face square to x) rides on the chuck's face with its low-y edge well left of the wafer's centre and moves toward the robot: the push
            then has a long lever on the left, so that it turns the wafer onto both close pins (a lever under about 30 mm jams on the pin's friction)."""
            p1y = T.seen.c[1] + (0.0 if mode == "centre" else float(os.environ.get("AL_P1Y", "0.040")))
            ycon = p1y - HALF_W if mode != "centre" else T.seen.c[1]  # where the blade's low-y edge meets the rim
            rim = T.seen.c[0] + math.sqrt(max(R_W ** 2 - (ycon - T.seen.c[1]) ** 2, 0.0)) + 0.004  # the 4 mm covers the turn
            start = world(rim + 0.0015 + BLADE, p1y, HOVER_Z)
            yield from l.move(start + [0, 0, 0.03], t=1.2, label="above " + label)
            yield from l.move(start, t=0.5, label="hover")
            down = start + [0, 0, AL_Z - HOVER_Z]
            yield from l.move(down, t=0.7, label="down")
            yield from l.wait(0.3)
            yield from drive(l, X, ramp(down, down + [-0.040, 0, 0], float(os.environ.get('AL_STEP', '0.0003'))), label, until=stop)
            yield from l.wait(0.6)
            print(f"[aligner] {label} done: gaps {np.round(np.array(T.seen.gaps()) * 1000, 2)} c {np.round(T.seen.c * 1000, 2)} yaw {math.degrees(T.seen.yaw):.2f}", flush=True)
            yield from l.move(l.at() + [0.02, 0, 0], t=0.5, label="ease off")
            yield from l.move(l.at() + [0, 0, HOVER_Z - AL_Z + 0.03], t=0.6, label="up")

        yield from push_x("push 1", lambda: all(T.seen.touching()[:2]))
        if mode == "p1":
            return
        def push_y(label, stop):
            """Push toward the left pin from the right, along the flat. The blade is turned 15 degrees and moves along its own face normal, which has a small component toward
            the robot, so that the flat stays on the pins while the wafer slides left."""
            psi = 15.0
            u = np.array([-math.sin(math.radians(psi)), math.cos(math.radians(psi))])
            G2 = d.down(psi)
            P = T.seen.c - R_W * u
            s2 = chuck_origin(env) + np.array([P[0] - u[0] * (BLADE + 0.002), P[1] - u[1] * (BLADE + 0.002), HOVER_Z])
            yield from l.move(s2 + [0, 0, 0.03], grip=G2, t=1.2, label="above " + label)
            yield from l.move(s2, t=0.5, label="hover 2")
            down2 = s2 + [0, 0, AL_Z - HOVER_Z]
            yield from l.move(down2, t=0.7, label="down 2")
            yield from l.wait(0.3)
            yield from drive(l, G2, ramp(down2, down2 + np.array([u[0], u[1], 0.0]) * 0.020, 0.0003), label, until=stop)
            yield from l.wait(0.6)
            print(f"[aligner] {label} done: gaps {np.round(np.array(T.seen.gaps()) * 1000, 2)} c {np.round(T.seen.c * 1000, 2)} yaw {math.degrees(T.seen.yaw):.2f}", flush=True)
            yield from l.move(l.at() + np.array([-u[0], -u[1], 0.0]) * 0.02, t=0.5, label="ease off 2")
            yield from l.move(l.at() + [0, 0, HOVER_Z - AL_Z + 0.04], t=0.6, label="up 2")

        # 2. Slide it left along the flat to the left pin; that turns it a little, so push toward the robot again from the left; repeat until the three pins all touch.
        for cycle in range(3):
            if all(T.seen.touching()):
                break
            if not T.seen.touching()[2]:
                yield from push_y(f"push 2.{cycle}", lambda: T.seen.touching()[2])
            if not all(T.seen.touching()[:2]):
                yield from l.move(l.at() + [0.0, 0.0, 0.0], grip=X, t=0.8, label="turn the blade")
                yield from push_x(f"push 3.{cycle}", lambda: all(T.seen.touching()[:2]))
    # 3. The vacuum key.
    kx, ky = CHUCK_AT[0] + KEY_AT[0], CHUCK_AT[1] + KEY_AT[1]
    kz = FACE + BASE_TOP + 0.008 - 0.007 + 0.013 - 0.003  # the cap's top, less 4 mm of travel, the fingers' length past the TCP and their margin

    def press_key():
        yield from l.move((kx, ky, FACE + 0.05), grip=X, t=1.0, label="above key")
        yield from l.move((kx, ky, kz + 0.02), t=0.5, label="to key")
        yield from drive(l, X, ramp((kx, ky, kz + 0.02), (kx, ky, kz - 0.004), 0.0005), "press key", until=lambda: T.vacuum)
        yield from l.wait(0.5)
        yield from l.move((kx, ky, FACE + 0.05), t=0.6, label="off key")

    # 4. The slide, by its handle.
    def push_slide():
        hx = chuck_origin(env)[0] + HANDLE_X
        hz = FACE + 0.001
        face_h = hx - 0.006 - 0.003
        yield from l.move((face_h - BLADE - 0.03, 0.0, FACE + 0.06), grip=X, t=1.0, label="before handle")
        yield from l.move((face_h - BLADE, 0.0, hz + 0.02), t=0.7, label="beside handle")
        yield from l.move((face_h - BLADE, 0.0, hz), t=0.4, label="down at handle")
        done = lambda: T.seen.slide >= TRAVEL - 0.0005
        yield from drive(l, X, ramp((face_h - BLADE, 0.0, hz), (face_h - BLADE + TRAVEL + 0.012, 0.0, hz), 0.0010), "slide in", until=done)
        yield from l.wait(0.5)
        print(f"[aligner] slide {T.seen.slide * 1000:.1f} mm; gaps {np.round(np.array(T.seen.gaps()) * 1000, 2)}", flush=True)

    if mode == "slidefirst":
        yield from push_slide()
        yield from press_key()
    else:
        if mode != "nokey":
            yield from press_key()
        yield from push_slide()
    yield from l.move(l.at() + [-0.03, 0, 0.0], t=0.5, label="off handle")
    yield from l.move(l.at() + [0, 0, 0.08], t=0.6, label="up")
    yield from l.move(LEFT_AWAY, t=1.2, label="away")
    yield from l.wait(1.6)
    print(f"[aligner] done: score {TRIAL.score}", flush=True)
