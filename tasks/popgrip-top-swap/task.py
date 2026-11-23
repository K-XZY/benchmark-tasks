"""Swap the top on a PopSockets PopGrip stuck on the back of an iPhone 15 lying face down on the desk: press and twist the old top 90 degrees off,
put it in the tray, set the new top on the base, press and twist it 90 degrees until it locks, and pull it up to its expanded height.

Frame: x forward, y left, z up, metres; z = 0 is the table top (TOP above the station's floor frame).
The phone (an articulation whose frame is fixed to the table): the phone body slides on x and y and turns about z on undriven joints held only by friction
equal to the table's (0.5 N on each slide: 0.3 x 171 g x 9.81; 0.02 N m on the turn), so it moves as a phone lying on a desk. The PopGrip base is fixed on its back.
Each top hangs on six undriven joints from the phone (three slides, a yaw, a tilt, its own turn) and a seventh slide, the accordion (0 to 16.6 mm), so until the
lock takes it it moves as a free body: gravity, the tray, the fingers and the base act on it, nothing else.
The lock (code): a top whose head rests on the base, coaxial within 1.5 mm, level within 4 degrees and turned to an open slot (within 20 degrees), is seated: the
socket holds it on the base's axis and a 10 N/mm spring holds its head 1 mm above its seat. It turns only while the head is pushed down to within 0.4 mm of
its seat (0.6 mm of spring travel: 6 N); unpressed, it holds
against 3 N m. Turned more than 10 degrees from the open slot, its lugs hold it down against up to 60 N. Within 10 degrees of the slot and lifted 3 mm, it is free.

Debug:  python demokit.py task.py OUT --no-record --mode full,none,one_hand
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
INSTRUCTION = ("Hold the phone, press and twist the old PopGrip top off into the tray, then press and twist the new top on until it locks, and pull it open.")
SUCCESS_HOLD_S = 3.0
SCORE_OVER = "end"
HOLD_STEPS = 60  # SUCCESS_HOLD_S at 20 Hz
TOP = d.TABLE_TOP

LOOKS = {
    "back": m.Look(m.rgb(0.55, 0.62, 0.70), roughness=0.25),  # iPhone 15 blue, frosted glass back
    "band": m.Look(m.rgb(0.62, 0.66, 0.72), metallic=0.7, roughness=0.3),
    "lens": m.Look(m.rgb(0.03, 0.03, 0.04), roughness=0.1),
    "base": m.Look(m.rgb(0.12, 0.12, 0.13), roughness=0.6),
    "head": m.Look(m.rgb(0.70, 0.71, 0.73), metallic=0.8, roughness=0.3),
    "accordion": m.Look(m.rgb(0.10, 0.10, 0.11), roughness=0.7),
    "old": m.Look(m.rgb(0.78, 0.20, 0.28), roughness=0.4),
    "new": m.Look(m.rgb(0.95, 0.80, 0.25), roughness=0.4),
    "tray": m.Look(m.rgb(0.85, 0.85, 0.83), roughness=0.5),
}

# -- real sizes (README, Real items)
PHONE = (0.0716, 0.1476, 0.0078)  # iPhone 15: width (x), height (y), depth; 171 g (Apple)
PHONE_XY = (0.38, -0.05)  # the phone's centre, world
BY = -0.034  # the base's centre on the phone's back, phone frame: 40 mm from the bottom end
DISC_R = 0.0194  # the PopTop: 38.8 mm across
BASE_R, BASE_H = 0.018, 0.002  # the adhesive base on the phone's back
NECK_R, NECK_H = 0.016, 0.0022  # the metal head and the collapsed accordion
DISC_H = 0.0035
SEAT = PHONE[2] + BASE_H  # the head's underside, seated: 9.8 mm
TOP_H = NECK_H + DISC_H  # 5.7 mm; with the base, 7.7 mm proud of the phone's back (maker: 7.7 mm collapsed)
EXT = 0.0166  # the accordion's travel: 24.3 mm expanded less 7.7 mm collapsed
TOP_MASS = 0.012  # kg, an estimate (README fidelity table)
TRAY = (0.30, -0.22)  # world: a trinket dish, 90 x 130 mm inside, 2 mm floor, 6 mm walls
TRAY_IN = (0.090, 0.130)
TRAY_FLOOR = 0.002
NEW_AT = (TRAY[0], TRAY[1] + 0.030)
DROP_AT = (TRAY[0], TRAY[1] - 0.030)

# -- the lock's numbers
CAPTURE_R = 0.0015
CAPTURE_TILT = 4.0
CAPTURE_SLOT = 20.0  # degrees either side of an open slot
LIFT = 0.001  # the head's spring holds it this far up from its seat
SPRING_K = 10000.0  # N/m
PRESSED = 0.0004  # pressed: the head within 0.4 mm of its seat, 0.6 mm down from its 1 mm rest height: 6 N on SPRING_K
TURN_T = 0.08  # N m to turn the pressed top (estimate; the demo passes from 0.03 to 0.3, README)
HOLD_T = 3.0  # N m it holds unpressed: a stop, over the 1.4 N m a 40 N pinch can turn before it slips
LUG_F = 60.0  # N the lugs hold it down
OPEN_BAND = 10.0  # degrees from the slot where the lugs are clear
FREE_UP = 0.003  # lifted this far above the seat at the slot: off the base
LOCKED_WITHIN = 10.0  # degrees from the locked pose
PHONE_SLIDE_F = 0.5  # N, 0.3 x 0.171 kg x 9.81
PHONE_TURN_T = 0.02  # N m
ACC_F = 1.0  # N, the accordion's pull-out friction

# Range checks (README, estimates table): corner runs set these by environment, e.g. POPGRIP_TURN_T=0.03 POPGRIP_SPRING_K=4670.
for _n, _v in [("TURN_T", TURN_T), ("SPRING_K", SPRING_K), ("HOLD_T", HOLD_T), ("LUG_F", LUG_F), ("CAPTURE_SLOT", CAPTURE_SLOT), ("OPEN_BAND", OPEN_BAND),
               ("PHONE_TURN_T", PHONE_TURN_T), ("ACC_F", ACC_F)]:
    globals()[_n] = float(os.environ.get(f"POPGRIP_{_n}", _v))

TOPS = {"old": {"at": (0.0, BY, SEAT), "offset": 0.0, "look": "old"},
        "new": {"at": (NEW_AT[0] - PHONE_XY[0], NEW_AT[1] - PHONE_XY[1], TRAY_FLOOR), "offset": 90.0, "look": "new"}}


def _phone():
    w, h, t = PHONE
    body = m.box((w - 0.002, h - 0.002, t), (0.0, 0.0, t / 2))
    band = m.box((w, h, t - 0.002), (0.0, 0.0, t / 2))
    lens = m.box((0.034, 0.034, 0.0025), (w / 2 - 0.022, h / 2 - 0.022, t + 0.00125))  # the camera plateau, top corner of the back
    base = m.cylinder(BASE_R, BASE_H, (0.0, BY, t + BASE_H / 2), segments=48)
    parts = [("body", body, "back"), ("band", band, "band"), ("lens", lens, "lens"), ("base", base, "base")]
    cols = [m.col_box("body_c", (w, h, t - 0.001), (0.0, 0.0, t / 2 + 0.0005)), m.col_hull("base_c", base)]
    return [m.slider("px", "X", -0.10, 0.10, [], [], friction=PHONE_SLIDE_F, damping=0.0, mass=0.002),
            m.slider("py", "Y", -0.10, 0.10, [], [], parent="px", friction=PHONE_SLIDE_F, damping=0.0, mass=0.002),
            m.hinge("pyaw", "Z", (0.0, 0.0, 0.0), -180.0, 180.0, parts, cols, parent="py", friction=PHONE_TURN_T, damping=0.0, mass=0.171,
                    inertia=(3.2e-4, 7.4e-5, 3.9e-4))]


def _top(name):
    at = np.array(TOPS[name]["at"])
    look = TOPS[name]["look"]
    neck = m.cylinder(NECK_R, NECK_H, tuple(at + [0, 0, NECK_H / 2]), segments=40)
    head = m.cylinder(0.011, 0.0006, tuple(at + [0, 0, 0.0003]), segments=32)
    disc = m.cylinder(DISC_R, DISC_H, tuple(at + [0, 0, NECK_H + DISC_H / 2]), segments=48)
    sleeve = m.cylinder(0.0150, EXT, tuple(at + [0, 0, NECK_H - EXT / 2]), segments=32)  # the accordion, seen only when pulled out
    free = dict(spring=0.0, damping=0.0, friction=0.0, effort=0.0, armature=1e-5, mass=0.002, inertia=(1e-7, 1e-7, 1e-7))
    a = tuple(at)
    p = lambda s: f"{name}_{s}"
    return [m.slider(p("sx"), "X", -0.6, 0.6, [], [], at=a, parent="pyaw", **free),
            m.slider(p("sy"), "Y", -0.6, 0.6, [], [], at=a, parent=p("sx"), **free),
            m.slider(p("sz"), "Z", -0.6, 0.6, [], [], at=a, parent=p("sy"), **free),
            m.hinge(p("yaw"), "Z", a, -720.0, 720.0, [], [], parent=p("sz"), **free),
            m.hinge(p("tilt"), "X", a, -80.0, 80.0, [], [], parent=p("yaw"), **free),
            m.hinge(p("spin"), "Z", a, -math.inf, math.inf, [("neck", neck, "accordion"), ("head", head, "head")], [m.col_hull(p("neck_c"), neck)],
                    parent=p("tilt"), spring=0.0, damping=0.0, friction=0.0, effort=0.0, armature=1e-6, mass=TOP_MASS / 2, gravity=True,
                    com=tuple(at + [0, 0, NECK_H / 2]), inertia=(8e-7, 8e-7, 1.5e-6)),
            m.slider(p("ext"), "Z", 0.0, EXT, [("disc", disc, look), ("sleeve", sleeve, "accordion")], [m.col_hull(p("disc_c"), disc)],
                     parent=p("spin"), friction=ACC_F, damping=2.0, mass=TOP_MASS / 2, gravity=True, com=tuple(at + [0, 0, NECK_H + DISC_H / 2]),
                     inertia=(1.2e-6, 1.2e-6, 2.3e-6))]


def _tray():
    lx, ly = TRAY_IN
    wall, hgt = 0.004, 0.006
    floor = m.box((lx + 2 * wall, ly + 2 * wall, TRAY_FLOOR), (0, 0, TRAY_FLOOR / 2))
    walls = [((lx + 2 * wall, wall, hgt), (0, (ly + wall) / 2 * s, hgt / 2)) for s in (-1, 1)] + [((wall, ly, hgt), ((lx + wall) / 2 * s, 0, hgt / 2)) for s in (-1, 1)]
    parts = [("floor", floor, "tray")] + [(f"wall{i}", m.box(*w), "tray") for i, w in enumerate(walls)]
    cols = [m.col_box("floor_c", (lx + 2 * wall, ly + 2 * wall, TRAY_FLOOR), (0, 0, TRAY_FLOOR / 2))] + [m.col_box(f"wall{i}_c", *w) for i, w in enumerate(walls)]
    return m.fixed("tray", (TRAY[0], TRAY[1], TOP), parts, cols, LOOKS)


PHONE_M = m.mechanism("phone", (PHONE_XY[0], PHONE_XY[1], TOP), [*_phone(), *_top("old"), *_top("new")], LOOKS, friction=(0.9, 0.8), solver_iters=64,
                      self_collisions=True)
OBJECTS = [PHONE_M, _tray()]


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def top_pose(st, name) -> tuple[np.ndarray, float, float, float]:
    """(the head's underside centre in the phone body's frame, its turn in degrees from the locked pose (mod 180, as the lugs repeat), its tilt in degrees, the accordion's travel)."""
    j = lambda s: st.joint(f"{name}_{s}")
    c = np.array(TOPS[name]["at"]) + [j("sx"), j("sy"), j("sz")]
    th = math.degrees(j("yaw") + j("spin")) + TOPS[name]["offset"]
    th = (th + 90.0) % 180.0 - 90.0  # -90..90; 0 locked, +-90 at the open slot
    return c, th, math.degrees(j("tilt")), j("ext")


def phone_world(st) -> tuple[np.ndarray, float]:
    """The phone body's frame: its origin (world, less TOP in z) and its yaw (radians)."""
    return np.array([PHONE_XY[0] + st.joint("px"), PHONE_XY[1] + st.joint("py"), 0.0]), st.joint("pyaw")


class Lock:
    """The base's quarter-turn lock, for both tops."""

    def reset(self, env=None):
        self.seated = {"old": True, "new": False}
        self.gains = {}
        if env is not None:
            self._seat(env, "old")
            self._free(env, "new")

    def _set(self, env, link, k, c, effort, target=None):
        asset, j = m._joint(env, "phone", link)
        m._TOUCHED.add(("phone", link))
        if self.gains.get(link) != (k, c, effort):
            self.gains[link] = (k, c, effort)
            asset.write_joint_stiffness_to_sim(m._tensor(asset, k), joint_ids=[j])
            asset.write_joint_damping_to_sim(m._tensor(asset, c), joint_ids=[j])
            asset.write_joint_effort_limit_to_sim(m._tensor(asset, effort), joint_ids=[j])
        if target is not None:
            asset.set_joint_position_target(m._tensor(asset, target), joint_ids=[j])

    def _free(self, env, name):
        self.seated[name] = False
        for s in ("sx", "sy", "sz", "yaw", "tilt", "spin"):
            self._set(env, f"{name}_{s}", 0.0, 0.0, 0.0)

    def _seat(self, env, name):
        self.seated[name] = True
        at = TOPS[name]["at"]
        st = m.state(env, "phone")
        self._set(env, f"{name}_sx", 3000.0, 30.0, 40.0, 0.0 - at[0])
        self._set(env, f"{name}_sy", 3000.0, 30.0, 40.0, BY - at[1])
        self._set(env, f"{name}_tilt", 200.0, 0.5, 5.0, 0.0)
        self._set(env, f"{name}_yaw", 200.0, 0.5, 5.0, st.joint(f"{name}_yaw"))

    def step(self, env):
        st = m.state(env, "phone")
        for name in ("old", "new"):
            c, th, tilt, _ = top_pose(st, name)
            up = c[2] - SEAT
            if not self.seated[name]:
                if math.hypot(c[0], c[1] - BY) <= CAPTURE_R and -0.0005 <= up <= 0.0015 and abs(tilt) <= CAPTURE_TILT and abs(th) >= 90.0 - CAPTURE_SLOT:
                    self._seat(env, name)
                else:
                    continue
            clear = abs(th) >= 90.0 - OPEN_BAND  # the lugs at the slot
            if clear and up > FREE_UP:
                self._free(env, name)
                continue
            at = TOPS[name]["at"]
            self._set(env, f"{name}_sz", SPRING_K, 20.0, 3.0 if clear else LUG_F, SEAT + LIFT - at[2])
            pressed = up <= PRESSED
            self._set(env, f"{name}_spin", 0.0, 2.0 if pressed else 50.0, TURN_T if pressed else HOLD_T)


LOCK = Lock()
m.on_step(LOCK.step)
m.on_reset(LOCK.reset)

# -- the scorer


def gripper_points(env) -> np.ndarray:
    """Points over the whole of both grippers (fingers, palm, housing, and the wrist link's end), world less the env origin: a grid on each, at the current opening."""
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


def now(env, hands: bool = False) -> dict:
    st = m.state(env, "phone")
    o, yaw = phone_world(st)
    R = _rz(yaw)
    out = {"phone_shift": float(math.hypot(st.joint("px"), st.joint("py"))), "phone_turn": math.degrees(yaw), "hands": 1.0}
    for name in ("old", "new"):
        c, th, tilt, ext = top_pose(st, name)
        w = o + R @ c
        out[name] = {"c": c, "w": w, "turn": th, "tilt": tilt, "ext": ext, "seated": LOCK.seated[name]}
    if hands:
        pts = gripper_points(env) - [0.0, 0.0, TOP]
        loc = (pts - o) @ R  # into the phone's frame
        c = out["new"]["c"]
        top = c[2] + NECK_H + DISC_H + out["new"]["ext"]
        rad = np.hypot(loc[:, 0] - c[0], loc[:, 1] - c[1]) - DISC_R
        dz = np.maximum(0.0, np.maximum(c[2] - loc[:, 2], loc[:, 2] - top))
        out["hands"] = float(np.hypot(np.maximum(rad, 0.0), dz).min())
    return out


def in_tray(w) -> bool:
    return abs(w[0] - TRAY[0]) <= TRAY_IN[0] / 2 and abs(w[1] - TRAY[1]) <= TRAY_IN[1] / 2 and w[2] <= TRAY_FLOOR + 0.006


def judge(c: dict, held_steps: int = HOLD_STEPS) -> tuple[bool, float]:
    """(full, score). 0 if the phone was shoved: moved more than 30 mm or turned more than 30 degrees on the desk.
    0.33: the old top is off the base and lies in the tray. 0.67: and the new top is seated on the base and turned to within 10 degrees of its locked pose
    (its lugs then hold it down against up to 60 N). 1.0: and the new top is pulled up at least 15 of its 16.6 mm, still locked, with both whole grippers at
    least 1 cm from it, held 3 s."""
    if c["phone_shift"] > 0.03 or abs(c["phone_turn"]) > 30.0:
        return False, 0.0
    old, new = c["old"], c["new"]
    if old["seated"] or not in_tray(old["w"]):
        return False, 0.0
    if not (new["seated"] and abs(new["turn"]) <= LOCKED_WITHIN):
        return False, 0.33
    if new["ext"] >= 0.015 and c["hands"] >= 0.01:
        return True, 1.0 if held_steps >= HOLD_STEPS else 0.67
    return False, 0.67


class Trial:
    def reset(self, env=None):
        self.held = 0
        self.full, self.score = False, 0.0

    def step(self, env):
        st = m.state(env, "phone")
        new_ok = LOCK.seated["new"] and abs(top_pose(st, "new")[1]) <= LOCKED_WITHIN and top_pose(st, "new")[3] >= 0.015
        c = now(env, hands=new_ok)
        self.full, self.score = judge(c, self.held + 1)
        self.held = self.held + 1 if self.full else 0


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.full


def score(objects) -> float:
    return TRIAL.score


# -- the demo: reads the tops' poses (the cameras show them) and the hands' own poses
AZ = (0.0, 0.0, 1.0)
TIP = 0.015  # the fingertips' underside below the TCP
RIM_TIP = 0.0100  # the fingertips' underside when pinching a seated top's rim: 2.2 mm above the phone's back
HOLD_Z = 0.019  # the left TCP pinching the phone's sides (tips 4 mm above the table)
STEP = 15.0
PRESS_F = 14.0  # N the right hand presses with: over the 11 N top of the push-button range (README)


def demo_script(ctx):
    """Modes: full; none (the arms wait); one_hand (as full with the left hand idle); no_turn (the old top pulled up without a turn);
    no_press (the turns without the press); set_only (the old top removed, the new one set on the base and let go, unturned)."""
    mode = ctx.mode.split("@")[0]
    if mode not in ("full", "none", "one_hand", "no_turn", "no_press", "set_only"):
        raise ValueError(f"unknown mode {mode!r}")
    L, R = ctx.left, ctx.right
    env = ctx.emb._env
    dry = getattr(env, "unwrapped", None) is None
    if mode == "none":
        yield from L.wait(3.0)
        return

    def info(tag):
        if dry:
            return
        c = now(env)
        nc = c["new"]["c"]
        print(f"[demo] {tag}: new off axis {math.hypot(nc[0], nc[1] - BY)*1000:.2f} mm | phone {c['phone_shift']*1000:.1f} mm {c['phone_turn']:.1f} deg | "
              + " | ".join(f"{n} w {np.round(c[n]['w'], 4)} turn {c[n]['turn']:.1f} tilt {c[n]['tilt']:.1f} ext {c[n]['ext']*1000:.1f} seated {c[n]['seated']}" for n in ("old", "new"))
              + f" | score {TRIAL.score}", flush=True)

    def top_w(name):
        if dry:
            return np.array([PHONE_XY[0], PHONE_XY[1] + BY, SEAT]) if name == "old" else np.array([NEW_AT[0], NEW_AT[1], TRAY_FLOOR])
        return now(env)[name]["w"]

    base_w = np.array([PHONE_XY[0], PHONE_XY[1] + BY, 0.0])
    off = np.array([0.0, 0.0, TOP])
    hold = np.array([PHONE_XY[0], PHONE_XY[1] + 0.065, HOLD_Z]) + off
    press = mode != "no_press"

    def left_hold():
        yield from L.move(hold + [0, 0.04, 0.04], grip=d.tilted(45.0, -90.0), finger=d.FINGER_OPEN, label="above the phone")  # from the phone's top end, fingers pointing -y
        yield from L.move(hold, t=1.0, label="beside the phone")
        yield from L.close(PHONE[0], force=30.0, label="hold the phone")

    def rim(name):
        """The TCP pinching a top's rim, the fingertips RIM_TIP above the table when it is seated."""
        c = top_w(name)
        return np.array([c[0], c[1], c[2] - SEAT + RIM_TIP + TIP]) + off

    def turn(deg):
        n = int(round(abs(deg) / STEP))
        for k in range(n):
            yield from R.turn(math.copysign(STEP, deg), axis=AZ, t=0.3, label="turn")
            if press and not dry:
                yield from R.move(R.at() - [0, 0, 0.0004], t=0.05, label="keep pressing")
        info("turned")
        tcpinfo("after the turn", R.at())

    def tcpinfo(tag, want):
        if dry:
            return
        rob = env.unwrapped.scene["right_robot"]
        w = abs(float(m._np(rob.data.joint_pos)[0, rob.find_joints("joint7")[0][0]]))
        print(f"[demo] {tag}: R tcp {np.round(R.tcp() - off, 4)} want {np.round(np.asarray(want) - off, 4)} opening {w*1000:.1f} | L tcp {np.round(L.tcp() - off, 4)}", flush=True)

    def press_down():
        if press and not dry:
            res = yield from R.press((0.0, 0.0, -1.0), PRESS_F, 0.003, label="press down")
            print(f"[demo] press: {res}", flush=True)

    if mode != "one_hand":
        yield from left_hold()
    info("start")
    # 1. the old top: pinch its rim, press, turn a quarter turn anticlockwise (seen from above) to its slot, pull it off
    p = rim("old")
    yield from R.move(p + [0, 0, 0.05], grip=d.down(0.0), finger=d.FINGER_OPEN, label="above the old top")
    yield from R.move(p, t=1.0, label="round the rim")
    yield from R.close(2 * DISC_R, force=30.0, label="pinch the rim")
    tcpinfo("pinched old", p)
    if mode != "no_turn":
        yield from press_down()
        yield from turn(90.0)
    yield from R.move(R.at() + [0, 0, 0.004], t=0.3, label="ease off")
    yield from R.move(R.at() + [0, 0, 0.040], t=0.8, label="lift it off")
    info("lifted")
    drop = np.array([DROP_AT[0], DROP_AT[1], 0.0]) + off
    yield from R.move(drop + [0, 0, 0.05], t=1.5, label="over the tray")
    yield from R.move(drop + [0, 0, 0.032], t=0.8, label="down into the tray")
    yield from R.open(label="let go")
    yield from R.move(R.at() + [0, 0, 0.05], t=0.6, label="up")
    info("old in the tray")
    # 2. the new top: pinch it in the tray, set it on the base, press, turn a quarter turn clockwise to lock
    n0 = top_w("new")
    q = np.array([n0[0], n0[1], 0.0]) + off + [0, 0, TRAY_FLOOR + NECK_H + DISC_H / 2 - 0.0035 + TIP]
    yield from R.move(q + [0, 0, 0.05], grip=d.down(90.0), finger=d.FINGER_OPEN, t=1.2, label="above the new top")
    yield from R.move(q, t=0.8, label="round its rim")
    yield from R.close(2 * DISC_R, force=30.0, label="pinch the rim")
    yield from R.move(R.at() + [0, 0, 0.04], t=0.6, label="lift")
    above = np.array([base_w[0], base_w[1], 0.0]) + off + [0, 0, SEAT + 0.012 + (q[2] - TOP - TRAY_FLOOR)]
    yield from R.move(above, t=1.5, label="over the base")
    yield from R.move(R.at() - [0, 0, 0.010], t=0.8, label="down to 2 mm above the base")

    def to_axis(tol):
        if dry:
            return
        o, yaw = phone_world(m.state(env, "phone"))
        b = o + _rz(yaw) @ np.array([0.0, BY, 0.0])
        w = now(env)["new"]["w"]
        yield from R.servo(np.array([b[0], b[1], w[2]]) + off, read=lambda: now(env)["new"]["w"] + off, tol=tol, timeout=2.0, label="on the axis")

    yield from to_axis(0.0003)
    yield from R.move(R.at() - [0, 0, 0.0025], t=0.5, label="set it on the base")
    info("set")
    tcpinfo("set", R.at())
    if mode == "set_only":
        yield from R.open(label="let go")
        yield from R.move(R.at() + [0, 0, 0.06], t=0.6, label="away")
        if mode != "one_hand":
            yield from L.open()
            yield from L.move(L.at() + [0, 0, 0.06], t=0.6)
        yield from R.wait(4.0)
        info("end")
        return
    yield from press_down()
    info("pressed")
    yield from turn(-90.0)
    yield from R.turn(-6.0, axis=AZ, t=0.3, label="on to the lock")
    z0 = R.tcp()[2]  # the press leaves the command below the rim: go on from where the hand is
    yield from R.move(np.array([R.at()[0], R.at()[1], z0 + 0.0005]), t=0.3, label="ease off")
    # 3. open the grip: pull the top up to its expanded height
    yield from R.move(np.array([R.at()[0], R.at()[1], z0 + EXT + 0.003]), t=1.0, label="pull it up")  # 3 mm past the travel: the accordion's own stop ends the pull
    info("expanded")
    tcpinfo("expanded", R.at())
    yield from R.open(label="let go")
    yield from R.move(R.at() + [0, 0, 0.06], t=0.6, label="away")
    if mode != "one_hand":
        yield from L.open()
        yield from L.move(L.at() + [0, 0, 0.06], t=0.6, label="off the phone")
    yield from R.wait(3.8)
    info("end")
