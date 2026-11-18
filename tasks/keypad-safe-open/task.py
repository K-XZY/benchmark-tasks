"""Open a keypad home safe: key the code, turn the knob within the 5 s the green light gives, swing the door open.

A SentrySafe X055 (35.1 wide, 22.1 high, 24.6 deep cm, 6.44 kg: the maker's listing) stands loose on the table, its door towards the arms. The door (33 x 19 cm, 2.3 kg, estimates) is hinged on its left
edge and held shut by bolts. It carries 12 round keys (10 mm across, 16 mm pitch across and 14 mm down, 2 mm proud, 1 mm of travel, a 2.4 N click) and a round 45 mm knob with a marker line.
Lock (code, as the maker's lock is electronics): the code 1-7-5-0-0 then # opens a 5.0 s window with the green light on; the knob is held at 0 except in that window, and once it passes 80 degrees
inside it the bolts are drawn and the door swings free. A wrong code gives the yellow light and the keys are ignored for 60 s. Every key press is read from the key's own travel past its click.

    python demokit.py task.py OUT --no-record --mode full,none,wrong,late,first,pull,shallow
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
INSTRUCTION = "Open the safe, leave its door open and take your hands away. The code is 1 7 5 0 0, then the # key."
SUCCESS_HOLD_S = 0.0  # the scorer holds its own 1 s
SCORE_OVER = "end"
TOP = d.TABLE_TOP
MM = 1e-3

DX = float(os.environ.get("KS_DX", "0.0"))  # the whole safe is set this far out in x (reach: README 27)
X0 = 0.38 + DX  # the door's front face

# -- the safe (local frame: x into the safe from the door's front face, y to the robot's left, z above the table) ----------------------------------------------------------------------
BODY_W, BODY_H, BODY_D = 0.351, 0.221, 0.246
SAFE_M, DOOR_M = 6.44, 2.3
DOOR_W, DOOR_H, DOOR_T = 0.330, 0.190, 0.018
DOOR_Z0 = 0.016
HINGE_Y = 0.165  # the door's left edge
DOOR_COM = (DOOR_T / 2, 0.0, DOOR_Z0 + DOOR_H / 2)
HINGE_FRICTION = 0.02  # N m (estimate)

KEY_R, KEY_PROUD, KEY_TRAVEL, KEY_CLICK = 0.005, 0.002, 0.001, 0.0007
KEY_SPRING, KEY_PRELOAD = 2000.0, 0.0005  # N/m; the spring rests half a mm beyond zero: 1.0 N at rest, 2.4 N at the click, 3.0 N at the bed
PAD_Y, PAD_Z = -0.05, 0.15
PITCH_Y, PITCH_Z = 0.016, 0.014
LAYOUT = ("1", "2", "3", "4", "5", "6", "7", "8", "9", "*", "0", "#")
KNOB_Y, KNOB_Z = -0.11, float(os.environ.get("KS_KNOB_Z", "0.105"))
KNOB_R, KNOB_L = 0.0225, 0.022
KNOB_FRICTION = 0.1  # N m (estimate)
KNOB_DRAW_DEG = 80.0
LED_Z = 0.19
LED_G_Y, LED_Y_Y = -0.042, -0.058

CODE = "17500"
WINDOW_S = 5.0
LOCKOUT_S = 60.0
DOOR_OPEN_DEG = 70.0
DOOR_MID_DEG = 20.0
CLEAR_M = 0.03
HOLD_STEPS = 20
MOVE_LIMIT, TILT_LIMIT = 0.02, 5.0

LOOKS = {"steel": m.Look(m.rgb(0.10, 0.10, 0.11), metallic=0.5, roughness=0.55), "door": m.Look(m.rgb(0.14, 0.14, 0.15), metallic=0.5, roughness=0.5),
         "key": m.Look(m.rgb(0.78, 0.78, 0.80), metallic=0.2, roughness=0.4), "knob": m.Look(m.rgb(0.55, 0.56, 0.58), metallic=0.8, roughness=0.35),
         "mark": m.Look(m.rgb(0.95, 0.15, 0.10), roughness=0.5), "gon": m.Look(m.rgb(0.1, 0.9, 0.2), emissive=(0.2, 1.0, 0.3)), "goff": m.Look(m.rgb(0.05, 0.18, 0.06), roughness=0.5),
         "yon": m.Look(m.rgb(1.0, 0.8, 0.1), emissive=(1.0, 0.8, 0.0)), "yoff": m.Look(m.rgb(0.2, 0.17, 0.04), roughness=0.5), "panel": m.Look(m.rgb(0.22, 0.22, 0.24), roughness=0.5)}


def key_yz(label: str) -> tuple[float, float]:
    i = LAYOUT.index(label)
    return PAD_Y - (i % 3 - 1) * PITCH_Y, PAD_Z - (i // 3 - 1.5) * PITCH_Z


def _box(size, c):
    return m.box(size, c)


def _door_link() -> m.Link:
    cx, cy, cz = DOOR_COM
    parts = [("plate", _box((DOOR_T, DOOR_W, DOOR_H), (cx, cy, cz)), "door"), ("pad", _box((0.0008, 0.075, 0.075), (-0.0004, PAD_Y, PAD_Z)), "panel"),
             ("led_g_off", m.cylinder(0.0035, 0.001, (-0.0005, LED_G_Y, LED_Z), axis="x"), "goff"), ("led_g_on", m.cylinder(0.0035, 0.001, (-0.0005, LED_G_Y, LED_Z), axis="x"), "gon"),
             ("led_y_off", m.cylinder(0.0035, 0.001, (-0.0005, LED_Y_Y, LED_Z), axis="x"), "yoff"), ("led_y_on", m.cylinder(0.0035, 0.001, (-0.0005, LED_Y_Y, LED_Z), axis="x"), "yon")]
    cols = [m.col_box("plate_c", (DOOR_T, DOOR_W, DOOR_H), (cx, cy, cz))]
    mass_c = DOOR_COM
    ix = DOOR_M / 12 * (DOOR_W**2 + DOOR_H**2)
    iy = DOOR_M / 12 * (DOOR_T**2 + DOOR_H**2)
    iz = DOOR_M / 12 * (DOOR_T**2 + DOOR_W**2)
    return m.hinge("door", "Z", (0.0, HINGE_Y, DOOR_COM[2]), -100.0, 0.0, parts, cols, mass=DOOR_M, damping=0.05, friction=HINGE_FRICTION, armature=1e-3, gravity=True, com=mass_c, inertia=(ix, iy, iz))


def _kname(label: str) -> str:
    return "star" if label == "*" else "hash" if label == "#" else label


def _key_link(label: str) -> m.Link:
    y, z = key_yz(label)
    h = KEY_PROUD + 0.0015  # the cap stands 2 mm proud of the door and sits 1.5 mm into it
    cx = -KEY_PROUD + h / 2
    cap = m.cylinder(KEY_R, h, (cx, y, z), axis="x")
    return m.slider(f"key_{_kname(label)}", "X", 0.0, KEY_TRAVEL, [("cap", cap, "key")], [m.col_hull(f"cap_{_kname(label)}_c", cap)], at=(0.0, y, z),
                    mass=0.02, spring=KEY_SPRING, damping=2.0, friction=0.0, effort=20.0, parent="door")


def _knob_link() -> m.Link:
    cyl = m.cylinder(KNOB_R, KNOB_L, (-KNOB_L / 2, KNOB_Y, KNOB_Z), axis="x")
    mark = _box((0.0006, 0.004, KNOB_R), (-KNOB_L - 0.0001, KNOB_Y, KNOB_Z + KNOB_R / 2))
    return m.hinge("knob", "X", (0.0, KNOB_Y, KNOB_Z), 0.0, 92.0, [("body", cyl, "knob"), ("mark", mark, "mark")], [m.col_hull("knob_c", cyl)], mass=0.1, damping=0.05, friction=KNOB_FRICTION,
                   armature=1e-3, parent="door")


def _safe() -> m.Part:
    fp, fc = [], []
    x_rim = DOOR_T + 0.002

    def b(nm, size, c, look="steel"):
        fp.append((nm, _box(size, c), look))
        fc.append(m.col_box(f"{nm}_c", size, c))

    b("back", (BODY_D - x_rim, BODY_W, BODY_H), (x_rim + (BODY_D - x_rim) / 2, 0.0, BODY_H / 2))
    zt = DOOR_Z0 + DOOR_H + 0.0015  # top rim's underside
    zb = DOOR_Z0 - 0.0015
    b("rim_top", (x_rim, BODY_W, BODY_H - zt), (x_rim / 2, 0.0, (BODY_H + zt) / 2))
    b("rim_bot", (x_rim, BODY_W, zb), (x_rim / 2, 0.0, zb / 2))
    yl_in, yr_in = HINGE_Y + 0.0015, -HINGE_Y - 0.003
    b("rim_l", (x_rim, BODY_W / 2 - yl_in, zt - zb), (x_rim / 2, (BODY_W / 2 + yl_in) / 2, (zt + zb) / 2))
    b("rim_r", (x_rim, BODY_W / 2 + yr_in, zt - zb), (x_rim / 2, (-BODY_W / 2 + yr_in) / 2, (zt + zb) / 2))
    links = [_door_link(), _knob_link()] + [_key_link(k) for k in LAYOUT]
    mass = SAFE_M - DOOR_M
    inertia = (mass / 12 * (BODY_W**2 + BODY_H**2), mass / 12 * (BODY_D**2 + BODY_H**2), mass / 12 * (BODY_D**2 + BODY_W**2))
    return m.mechanism("safe", (X0, 0.0, TOP), links, LOOKS, frame_parts=fp, frame_colliders=fc, friction=(0.4, 0.4), solver_iters=64, free_mass=mass, frame_inertia=inertia)


OBJECTS = [_safe()]

# -- readings ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


def rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


HINGE_W = np.array([X0, HINGE_Y, TOP])
DOOR_C = np.array([DOOR_T / 2, -HINGE_Y, DOOR_Z0 + DOOR_H / 2])  # the door's centre from the hinge, as the door stands shut (z from the table)
DOOR_HALF = np.array([DOOR_T / 2, DOOR_W / 2, DOOR_H / 2])
BODY_C = np.array([X0 + BODY_D / 2, 0.0, TOP + BODY_H / 2])
BODY_HALF = np.array([BODY_D / 2, BODY_W / 2, BODY_H / 2])


def box_dist(p, centre, half) -> np.ndarray:
    q = np.abs(np.atleast_2d(p) - centre) - half
    return np.linalg.norm(np.maximum(q, 0.0), axis=1)


def door_deg(st) -> float:
    return -math.degrees(st.joint("door"))  # positive as it opens


def knob_deg(st) -> float:
    return math.degrees(st.joint("knob"))


def key_mm(st, label: str) -> float:
    return st.joint(f"key_{_kname(label)}") * 1000.0


def knob_world() -> np.ndarray:
    return np.array([X0, KNOB_Y, TOP + KNOB_Z])


def key_world(label: str) -> np.ndarray:
    y, z = key_yz(label)
    return np.array([X0 - KEY_PROUD, y, TOP + z])


def tier(code_ok: bool, drawn: bool, door_max: float, hold_ok: bool, violated: bool) -> float:
    """0.25 code accepted; 0.5 knob past 80 degrees while the green light was on; 0.75 door at 20 degrees or more; 1.0 door at 70 degrees or more, hands 3 cm clear, still 1 s. 0 if the safe moved 2 cm or tipped 5 degrees."""
    if violated:
        return 0.0
    if hold_ok:
        return 1.0
    if drawn and door_max >= DOOR_MID_DEG:
        return 0.75
    if drawn:
        return 0.5
    return 0.25 if code_ok else 0.0


class Trial:
    def reset(self, env=None):
        self.n = 0
        self.buf = ""
        self.armed = {k: True for k in LAYOUT}
        self.lockout_until = -1
        self.window_end = -1
        self.code_ok = False
        self.drawn = False
        self.green = False
        self.yellow = False
        self.door_max = 0.0
        self.quiet = 0
        self.hold_ok = False
        self.violated = False
        self.S0 = None
        self.prev_door = 0.0
        self.presses = []
        self.wrong = 0
        self.clear = False

    def _led(self, green: bool, yellow: bool):
        base = m.ENV_PATH + "/safe/door/visual"
        try:
            m.set_visible(f"{base}/led_g_on", green)
            m.set_visible(f"{base}/led_g_off", not green)
            m.set_visible(f"{base}/led_y_on", yellow)
            m.set_visible(f"{base}/led_y_off", not yellow)
        except RuntimeError as e:
            if self.n == 1:
                print(f"[ks] led prims: {e}", flush=True)

    def step(self, env):
        st = m.state(env, "safe")
        self.n += 1
        n = self.n
        if self.S0 is None:
            self.S0 = np.asarray(st.pos, float).copy()
            m.lock(env, "safe", "door")
            m.lock(env, "safe", "knob")
        for k in LAYOUT:
            m.set_target(env, "safe", f"key_{_kname(k)}", -KEY_PRELOAD)
        kd, dd = knob_deg(st), door_deg(st)
        self.door_max = max(self.door_max, dd)
        # the keys: a press is the key's travel going past its click; it is armed again once it is back under 0.3 mm
        in_lockout = n <= self.lockout_until
        for k in LAYOUT:
            t = key_mm(st, k)
            if self.armed[k] and t >= KEY_CLICK * 1000:
                self.armed[k] = False
                self.presses.append((n, k))
                if not in_lockout and not self.green and not self.drawn:
                    self._press(env, k, n)
            elif not self.armed[k] and t < 0.3:
                self.armed[k] = True
        # the window
        if self.green:
            if kd >= KNOB_DRAW_DEG and not self.drawn:
                self.drawn = True
                m.unlock(env, "safe", "door")
                print(f"[ks] bolts drawn at step {n}, knob {kd:.1f} deg, {(n - self.window_start) / 20:.2f} s into the window", flush=True)
            if n >= self.window_end:
                self.green = False
                if not self.drawn:
                    m.lock(env, "safe", "knob")  # the lock holds the knob where it is
                    print(f"[ks] window closed at step {n}, knob {kd:.1f} deg: the knob is locked again", flush=True)
        if self.yellow and n > self.lockout_until:
            self.yellow = False
        self._led(self.green, self.yellow)
        # the judgement
        tips = m.fingertips(env)
        th = -math.radians(dd)
        dloc = np.array([rz(-th) @ (t - HINGE_W) for t in tips])  # fingertips in the door's frame (heights above the table)
        dist = min(box_dist(dloc, DOOR_C, DOOR_HALF).min(), box_dist(tips, BODY_C, BODY_HALF).min())
        self.clear = bool(dist >= CLEAR_M)
        move = float(np.linalg.norm(np.asarray(st.pos, float) - self.S0))
        if move > MOVE_LIMIT or st.tilt_deg() > TILT_LIMIT:
            self.violated = True
        dsp = abs(dd - self.prev_door) / float(env.unwrapped.step_dt)  # deg/s
        self.prev_door = dd
        still = bool(dsp < 3.0 and float(np.linalg.norm(st.lin_vel)) < 0.02)
        if self.drawn and dd >= DOOR_OPEN_DEG and self.clear and still and not self.violated:
            self.quiet += 1
        else:
            self.quiet = 0
        if self.quiet >= HOLD_STEPS:
            self.hold_ok = True
        if n % 10 == 0:
            print("[ks] " + json.dumps({"mode": os.environ.get("TASK_DEMO", ""), "n": n, "buf": self.buf, "green": self.green, "yellow": self.yellow, "knob": round(kd, 1), "door": round(dd, 1),
                                        "clear": self.clear, "dist_mm": round(dist * 1000), "still": still, "quiet": self.quiet, "moved_mm": round(move * 1000, 1), "tilt": round(st.tilt_deg(), 2),
                                        "score": self.score()}), flush=True)

    def _press(self, env, k: str, n: int):
        if k != "#":
            self.buf += k
            return
        if self.buf == CODE:
            self.code_ok = True
            self.green = True
            self.window_start = n
            self.window_end = n + round(WINDOW_S * 20)
            m.unlock(env, "safe", "knob")
            print(f"[ks] code accepted at step {n}: green light for {WINDOW_S} s", flush=True)
        else:
            self.wrong += 1
            self.yellow = True
            self.lockout_until = n + round(LOCKOUT_S * 20)
            print(f"[ks] wrong code {self.buf!r} at step {n}: five beeps, yellow light, keys ignored for {LOCKOUT_S:.0f} s", flush=True)
        self.buf = ""

    def score(self) -> float:
        return tier(self.code_ok, self.drawn, self.door_max, self.hold_ok, self.violated)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects) -> float:
    return TRIAL.score()


def success(objects) -> bool:
    return bool(TRIAL.hold_ok and not TRIAL.violated)


# -- the scripted demo -----------------------------------------------------------------------------------------------------------------------------------------------------------------

STANDOFF = float(os.environ.get("KS_STANDOFF", "0.04"))
TIP_OFF = float(os.environ.get("KS_TIP_OFF", "0.0"))  # TCP x minus fingertip x along +x; found on the first run
PRESS_N = 8.0
PULL1, PULL2 = float(os.environ.get("KS_PULL1", "20")), float(os.environ.get("KS_PULL2", "82"))
MODES = ("full", "none", "wrong", "late", "first", "pull", "shallow", "one_hand")


def _grip_from(spec: str, roll: float = 0.0):
    kind, args = spec.split(":")
    a = [float(v) for v in args.split(",")]
    g = d.level(*a) if kind == "level" else d.tilted(*a)
    if roll:
        g = d._as_grip(d.axis_angle_matrix(g.rot[:, 2], math.radians(roll)) @ g.rot)
    return g


def knob_at(deg: float) -> np.ndarray:
    """The knob's middle (body centre) with the door open `deg` degrees."""
    return HINGE_W + rz(-math.radians(deg)) @ np.array([-KNOB_L / 2, KNOB_Y - HINGE_Y, 0.0]) + np.array([0.0, 0.0, KNOB_Z])


def _press_key(ctx, arm, label, mode="full"):
    p = key_world(label)
    yield from arm.move(p + [-STANDOFF, 0.0, 0.0], label=f"over {label}")
    yield from arm.servo(p + [-STANDOFF, 0.0, 0.0], tol=0.002, timeout=0.8, label=f"over {label}")
    if mode == "shallow":
        yield from arm.move(p + [-0.025, 0.0, 0.0], t=0.5, label=f"hover {label}")
        return
    res = yield from arm.press((1.0, 0.0, 0.0), PRESS_N, STANDOFF + 0.002 + TIP_OFF, speed=0.06, settle=0.2, timeout=3.0, label=f"press {label}")
    print(f"[ks] key {label}: {res}", flush=True)
    yield from arm.move(arm.at() + [-0.02, 0.0, 0.0], t=0.3, label=f"off {label}")


def _key_code(ctx, arm, code, mode="full"):
    for k in code:
        yield from _press_key(ctx, arm, k, mode)


def _grip_knob(ctx, arm, deg=0.0, back=0.05, grip=None):
    k = knob_at(deg)
    a = np.array([math.cos(math.radians(-deg)), math.sin(math.radians(-deg)), 0.0])  # the fingers' direction: into the door
    yield from arm.move(k - back * a, grip=grip, finger=d.FINGER_OPEN, label="over the knob")
    yield from arm.servo(k - back * a, tol=0.004, timeout=0.5)
    yield from arm.move(k, t=0.45, label="onto the knob")
    yield from arm.close(2 * KNOB_R, 30.0, label="grip the knob")


def demo_script(ctx):
    """Modes: full. none: the arms wait. wrong: a wrong digit then #. late: the code, then the knob only after the light has gone. first: the knob is turned hard before any code is keyed, then the hands stop. pull: code, then the door is pulled by
    the knob without turning it. shallow: every key only touched, never pressed through its click. one_hand: the right hand does the door alone."""
    mode = ctx.mode
    if mode not in MODES:
        raise ValueError(f"unknown mode {mode!r}")
    L, R = ctx.left, ctx.right
    if mode == "none":
        yield from L.wait(3.0)
        return
    grip = d.level(0.0)
    yield from R.move(key_world("5") + [-0.10, 0.0, 0.0], grip=grip, finger=d.FINGER_CLOSED, label="right to the keypad")
    if mode == "first":
        yield from _grip_knob(ctx, R)
        yield from R.arc(R.at(), (1.0, 0.0, 0.0), 90.0, t=1.5, label="turn the knob first")
        print(f"[ks] knob after a forced turn: {ctx.deg('safe', 'knob'):.1f} deg", flush=True)
        yield from R.open()
        yield from R.move(R.at() + [-0.08, 0.0, 0.0], t=0.6)
        yield from R.wait(2.0)
        print("[ks] END " + json.dumps({"mode": mode, "score": TRIAL.score()}), flush=True)
        return
    code = CODE if mode != "wrong" else "17550"
    yield from _key_code(ctx, R, code, "shallow" if mode == "shallow" else "full")
    yield from _press_key(ctx, R, "#", "shallow" if mode == "shallow" else "full")
    if mode == "shallow":
        yield from R.wait(2.0)
        return
    if mode == "late":
        yield from R.move(R.at() + [-0.08, 0.0, 0.0], t=0.6)
        yield from R.wait(5.0)
    kgrip = _grip_from(os.environ.get("KS_KG", "tilted:75,0"), float(os.environ.get("KS_KROLL", "0")))
    yield from _grip_knob(ctx, R, grip=kgrip, back=float(os.environ.get("KS_BACK", "0.05")))
    if mode != "pull":
        yield from R.arc(R.at(), (1.0, 0.0, 0.0), 92.0, t=1.0, label="turn the knob")
        print(f"[ks] knob at {ctx.deg('safe', 'knob'):.1f} deg", flush=True)
    yield from R.arc(HINGE_W, (0.0, 0.0, 1.0), -(PULL2 if mode == "one_hand" else PULL1), t=1.2 if mode != "one_hand" else 3.0, label="right pulls the door")
    print(f"[ks] door at {-ctx.deg('safe', 'door'):.1f} deg", flush=True)
    yield from R.open()
    yield from R.move(R.at() + [-0.08, -0.04, 0.05], t=0.8, label="right clear")
    if mode in ("pull", "one_hand"):
        yield from R.wait(2.0)
        print("[ks] END " + json.dumps({"mode": mode, "score": TRIAL.score()}), flush=True)
        return
    # the left hand takes the knob from the left of the door and swings it on
    a0 = -ctx.deg("safe", "door")
    lgrip = d.tilted(float(os.environ.get("KS_LT", "60")), -a0, flip=bool(int(os.environ.get("KS_LFLIP", "0"))))
    yield from L.move(knob_at(a0) - 0.08 * np.array([math.cos(math.radians(a0)), -math.sin(math.radians(a0)), 0.0]), grip=lgrip, finger=d.FINGER_OPEN, label="left to the knob")
    yield from _grip_knob(ctx, L, a0, grip=lgrip, back=float(os.environ.get("KS_LBACK", "0.05")))
    yield from L.arc(HINGE_W, (0.0, 0.0, 1.0), -(PULL2 - a0), t=2.0, label="left swings the door")
    print(f"[ks] door at {-ctx.deg('safe', 'door'):.1f} deg", flush=True)
    yield from L.open()
    yield from L.move(L.at() + 0.10 * np.array([-math.sin(math.radians(PULL2)) * 0.0, 1.0, 0.0]) + [0.0, 0.0, 0.05], t=1.0, label="left clear")
    yield from L.wait(2.5)
    print("[ks] END " + json.dumps({"mode": mode, "score": TRIAL.score()}), flush=True)
