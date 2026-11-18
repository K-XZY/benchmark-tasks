"""On the station table, a wall board along x = 0.45 m carries a yellow Tensabarrier-896-type belt unit (belt mouth at y = -0.44 m) and a zinc wall receiver (mouth at
y = +0.46 m), 0.90 m apart; the belt runs along the wall in the plane x = 0.37 m, 18 cm above the table. The belt's end housing (60 mm long x 50 mm tall x 12 mm thick,
0.12 kg) rests against the unit's mouth; the belt is a taut segment from the mouth to the housing, drawn back by a retractor spring (6 N at 0.1 m out, 12 N at 0.6 m)
and its brake. The receiver is out of the right arm's reach and the unit out of the left arm's reach (gate_collide.py reach), so the end changes
hands: the right hand takes the housing from the front by its top and bottom edges and draws the belt out; at y = +0.05 m the left hand
takes the housing's other half by its faces from above and the right lets go; the left squares the housing to the slot and pushes it 20 mm into the receiver, where a
sprung tongue drops into the notch behind its ramp.

Run it:  python demokit.py task.py OUT --no-record --stills 20
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent.parent / "kit"))

import gatekit as gk  # noqa: E402
import mechkit as m  # noqa: E402  (hooks yam_arena so that the parts below spawn from USD)
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Close the belt barrier: pull the belt out and click its end into the wall receiver."
SUCCESS_HOLD_S = 0.0
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
DT = 0.05

# -- layout (m; x forward from the arm bases, y to the robot's left, z above the world floor) -------------------------------------------------------------
XB = float(os.environ.get("BELT_XB", "0.37"))  # the belt's plane and the housing's centre
WALL_X = float(os.environ.get("BELT_WALL", "0.45"))  # the wall board's front face
CASE_D = WALL_X - XB + 0.02  # the case's depth from the wall: the belt leaves it 20 mm behind its front face
ZB = TOP + float(os.environ.get("BELT_ZH", "0.18"))  # belt and housing centre height
Y_MOUTH = float(os.environ.get("BELT_YM", "-0.44"))  # the unit's mouth
Y_RECV = float(os.environ.get("BELT_YR", "0.46"))  # the receiver's mouth
H_L, H_W, H_T = 0.060, 0.050, 0.012  # housing: length (y), height (z), thickness (x)
H_MASS = 0.12
SLOT_DEPTH = float(os.environ.get("BELT_SLOT", "0.020"))  # how far the housing enters the receiver, to its stop face
CHAMFER = 0.004  # the receiver's mouth: its cheeks' edges chamfered this much
Y_DOCK = Y_MOUTH + H_L / 2 + 0.0005
Y_SEAT = Y_RECV + SLOT_DEPTH - H_L / 2
Y_HAND = float(os.environ.get("BELT_YH", "0.05"))
OFF = 0.0225
OFF_R = float(os.environ.get("BELT_OFFR", "0.027"))  # the right grip's centre from the housing's centre, on the +y half
OFF_L = float(os.environ.get("BELT_OFFL", "0.027"))  # the left grip's centre, on the -y half
DZ_L = float(os.environ.get("BELT_DZL", "0.010"))  # the left's TCP this far above the housing's centre: its pads on the upper part of the faces
BETA_R = float(os.environ.get("BELT_BR", "-20"))  # right: fingers level, pointing +x turned this far about z (deg), closing on the top and bottom edges
TH_L = float(os.environ.get("BELT_TH", "10"))  # left: fingers down, tilted this far toward -y (deg), closing on the broad faces
LEAN_L = float(os.environ.get("BELT_LEAN", "15"))  # and leaning this far toward the wall (deg)

# -- the retractor spring -------------------------------------------------------------------------------------------------------------------------------
F0, K_SPR = 4.8, 12.0  # F = 4.8 + 12 * out: 6 N at 0.1 m, 12 N at 0.6 m
V_BRAKE = 0.25
SEG = 0.02  # belt drawn as 2 cm pieces shown out to the housing
N_SEG = int((Y_RECV - Y_MOUTH) / 0.02) + 2
STUB = 0.03  # the belt's piece that rides on the housing
BAR_W = 0.004
OUT_MAX = round(Y_RECV - Y_MOUTH + 0.05, 3)  # the belt's travel out of the case, past the receiver
C_BRAKE = 40.0  # the retractor's brake, viscous (N s/m): the belt let go draws back at about 0.12 m/s at the case, 0.3 m/s at 0.6 m out
PLAY = 0.010
K_PLAY = 400.0
K_YAW = 0.3  # N m/rad
BAR_K = 2700.0  # the latch bar's spring (N/m): 8 N at its 3 mm lift
RECV_MU = float(os.environ.get("BELT_RMU", "0.25"))  # zinc on the steel housing: with the 30 degree ramp, the push past the tongue is about 8 N
BAR_TRAVEL = 0.010  # the latch tongue lifts up to 10 mm
RAMP_FRONT = float(os.environ.get("BELT_RAMPF", "0.0215"))  # the housing's ramp: its top runs from this height (above the centre) at the front face to 25 mm, 6 mm back

LOOKS = {
    "wall": m.Look(color=m.rgb(0.62, 0.60, 0.56), roughness=0.8),
    "case": m.Look(color=m.rgb(0.95, 0.75, 0.05), roughness=0.45),
    "zinc": m.Look(color=m.rgb(0.62, 0.64, 0.67), roughness=0.35, metallic=0.85),
    "belt": m.Look(color=m.rgb(0.95, 0.78, 0.05), roughness=0.7),
    "housing": m.Look(color=m.rgb(0.30, 0.31, 0.33), roughness=0.3, metallic=0.9),
    "dark": m.Look(color=m.rgb(0.08, 0.08, 0.09), roughness=0.6),
}


def _wall():
    t = 0.02
    c = (WALL_X + t / 2, 0.0, TOP + 0.25)
    size = (t, 0.80, 0.50)
    return m.fixed("wall", (0, 0, 0), [("board", m.box(size, c), "wall")], [m.col_box("board", size, c)], LOOKS, friction=(0.6, 0.5))


def _case():
    """Yellow case 100 deep (x) x 115 long (y) x 70 tall (z), its mouth on the +y face; the belt leaves through a slot 52 x 2 mm, 20 mm behind its front."""
    size = (CASE_D, 0.115, 0.070)
    c = (WALL_X - CASE_D / 2, Y_MOUTH - 0.0575, ZB)
    mouth = (0.004, 0.003, 0.056)
    cm = (XB, Y_MOUTH + 0.0015, ZB)
    return m.fixed("case", (0, 0, 0), [("body", m.box(size, c), "case"), ("slot", m.box(mouth, cm), "dark")], [m.col_box("body", size, c)], LOOKS, friction=(0.5, 0.4))


def _receiver():
    """Zinc receiver: a back cheek, a front cheek and a bottom plate round a slot 14 mm wide (x), 33 mm deep, its mouth chamfered 4 mm, with a stop face 20 mm in; and a sprung latch bar (5 mm of a
    zinc tongue) riding the housing's top: it is lifted by the housing's front ramp (detent peak about 8 N) and drops into the notch behind the ramp, and its wall then holds the housing."""
    parts, cols = [], []
    y0, y1 = Y_RECV, Y_RECV + SLOT_DEPTH + 0.013
    yc, yl = (y0 + y1) / 2, y1 - y0
    ch = CHAMFER  # the cheeks' mouth edges are chamfered 4 mm x 45 degrees: a lead-in
    yk, ykl = (y0 + ch + y1) / 2, y1 - y0 - ch
    for nm, outline in (("front_lead", [(XB - 0.019, y0), (XB - 0.007 - ch, y0), (XB - 0.007, y0 + ch), (XB - 0.019, y0 + ch)]),
                        ("back_lead", [(XB + 0.007 + ch, y0), (WALL_X, y0), (WALL_X, y0 + ch), (XB + 0.007, y0 + ch)])):
        mesh = m.prism(outline, ZB - 0.037, ZB + 0.037)
        parts.append((nm, mesh, "zinc"))
        cols.append(m.col_hull(nm, mesh))
    for nm, size, c in (("back", (WALL_X - (XB + 0.007), ykl, 0.074), (XB + 0.007 + (WALL_X - XB - 0.007) / 2, yk, ZB)),
                        ("front", (0.012, ykl, 0.074), (XB - 0.007 - 0.006, yk, ZB)),
                        ("bot", (0.014, yl, 0.011), (XB, yc, ZB - 0.026 - 0.0055)),
                        ("stop", (0.014, y1 - (Y_RECV + SLOT_DEPTH), 0.052), (XB, (y1 + Y_RECV + SLOT_DEPTH) / 2, ZB))):
        parts.append((nm, m.box(size, c), "zinc"))
        cols.append(m.col_box(nm, size, c))
    bar_size = (0.010, BAR_W, 0.012)
    bar_c = (XB, Y_RECV + SLOT_DEPTH - 0.009, ZB + 0.022 + 0.006)
    bar = m.slider("bar", "Z", 0.0, BAR_TRAVEL, [("bar", m.box(bar_size, bar_c), "zinc")], [m.col_box("bar", bar_size, bar_c)], mass=0.02, spring=BAR_K, damping=4.0, friction=0.0)
    return m.mechanism("receiver", (0, 0, 0), [bar], LOOKS, frame_parts=parts, frame_colliders=cols, friction=(RECV_MU, RECV_MU * 0.8))


def _belt():
    """The belt as thin pieces laid along the line from the mouth; the ones beyond the housing are hidden as it comes back."""
    parts = []
    for k in range(N_SEG):
        yc = Y_MOUTH + SEG * (k + 0.5)
        parts.append((f"seg{k}", m.box((0.002, SEG - 0.0004, H_W), (XB, yc, ZB)), "belt", True))
    return m.fixed("beltvis", (0, 0, 0), parts, [], LOOKS)


def _housing():
    """The belt's end: the end housing, a 12 mm plate, 60 long x 50 tall, its front (+y) top edge ramped 6 mm and a 6 mm notch behind the ramp, 3 mm deep (the latch bar's seat).
    It rides on the belt, modelled as a slide along the wall (the belt drawn out: retractor spring, brake) with the webbing's play: 10 mm sideways (x) and up/down (z), 10 degrees of yaw."""
    f = H_L / 2
    c = np.array([XB, Y_DOCK, ZB])
    body = m.box((H_T, 0.054, 0.047), c + (0.0, -0.003, -0.0015))
    ramp = m.prism([(f - 0.006, -0.025), (f, -0.025), (f, RAMP_FRONT), (f - 0.006, 0.025)], -H_T / 2, H_T / 2)
    ramp = m.Mesh(ramp.v[:, [2, 0, 1]], ramp.f).moved(c)  # outline was in (y, z): extruded along x
    land = m.box((H_T, 0.048, 0.003), c + (0.0, -0.006, 0.0235))
    stub = m.box((0.002, STUB, H_W - 0.002), c + (0.0, -H_L / 2 - STUB / 2, 0.0))
    parts = [("body", body, "housing"), ("ramp", ramp, "housing"), ("land", land, "housing"), ("stub", stub, "belt")]
    cols = [m.col_box("body", (H_T, 0.054, 0.047), c + (0.0, -0.003, -0.0015)), m.col_hull("ramp", ramp), m.col_box("land", (H_T, 0.048, 0.003), c + (0.0, -0.006, 0.0235))]
    at = tuple(float(v) for v in c)
    out = m.slider("out", "Y", 0.0, OUT_MAX, [], [], at=at, mass=0.01, spring=K_SPR, damping=C_BRAKE, friction=0.0, effort=200.0)
    dx = m.slider("dx", "X", -PLAY, PLAY, [], [], at=at, mass=0.01, spring=K_PLAY, damping=3.0, friction=0.0, parent="out")
    dz = m.slider("dz", "Z", -PLAY, PLAY, [], [], at=at, mass=0.01, spring=K_PLAY, damping=3.0, friction=0.0, parent="dx")
    yaw = m.hinge("yaw", "Z", at, -10.0, 10.0, parts, cols, mass=H_MASS, spring=K_YAW, damping=0.02, friction=0.0, parent="dz")
    return m.mechanism("belt", (0, 0, 0), [out, dx, dz, yaw], LOOKS, friction=(0.7, 0.6))


OBJECTS = [_wall(), _case(), _receiver(), _belt(), _housing()]
BELT_PULL = F0  # the constant part of the retractor's pull (N); the spring adds K_SPR per metre out

G = 9.81


_LAST = {}


def housing(env):
    """The housing's pose from the belt's joints (the frame is fixed): centre, yaw, and its velocity from the last step."""
    st = m.state(env, "belt")
    pos = np.array([XB + st.joint("dx"), Y_DOCK + st.joint("out"), ZB + st.joint("dz")])
    yaw = st.joint("yaw")
    c, sn = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -sn, 0.0], [sn, c, 0.0], [0.0, 0.0, 1.0]])
    prev = _LAST.get("pos")
    vel = (pos - prev) / DT if prev is not None else np.zeros(3)
    _LAST["pos"] = pos
    return gk.Pose(pos, R, vel)


# -- scoring ----------------------------------------------------------------------------------------------------------------------------------------------------
OUT_PASS = 0.30  # after the belt has passed this far out, it may not come back by more than RETRACT_MAX
RETRACT_MAX = 0.10
OUT_MIN = 0.50  # belt out at the end
SEAT_MIN = SLOT_DEPTH - 0.0015  # the housing's front this far into the slot: the notch is under the bar
BAR_DOWN = 0.0015  # the latch bar this close to its lowest: it sits in the notch
CLEAR = 0.05
HOLD_STEPS = 20
FELL_Z = 0.08  # the housing this far below the belt's height is dropped
# the housing held by its broad faces (jaws across x) or by its top and bottom edges (jaws across z)
GRIPS = (gk.Grip(half=H_T / 2, across="x", lo=(-1.0, -H_L / 2 - 0.004, -H_W / 2 - 0.004), hi=(1.0, H_L / 2 + 0.004, H_W / 2 + 0.004)),
         gk.Grip(half=H_W / 2, across="z", lo=(-0.03, -H_L / 2 - 0.004, -1.0), hi=(0.03, H_L / 2 + 0.004, 1.0)))


def holders(tips, h) -> tuple[str, ...]:
    return tuple(sorted({hand for g in GRIPS for hand in gk.holding(tips, h, g, resting=False)}))


def st_yaw(env) -> float:
    return float(m.state(env, "belt").joint("yaw"))


class Trial:
    def reset(self, env=None) -> None:
        self.steps = 0
        _LAST.clear()
        self.max_out = 0.0
        self.fell = gk.Latch()
        self.retracted = gk.Latch()  # came back more than RETRACT_MAX after passing OUT_PASS
        self.seated = gk.Latch()  # the housing's front went SEAT_MIN into the slot
        self.held_out = gk.Latch()  # belt out OUT_MIN while a hand holds it
        self.both = gk.Latch()  # both hands held the housing at once
        self.end = gk.EndHold(HOLD_STEPS)
        self.score, self.full = 0.0, False
        self.r = {}

    def step(self, env) -> None:
        self.steps += 1
        h = housing(env)
        bar = m.state(env, "receiver")
        R = h.rotation()
        back = h.pos + R @ np.array([0.0, -H_L / 2, 0.0])  # the belt's end on the housing
        front = h.pos + R @ np.array([0.0, H_L / 2, 0.0])
        out = float(back[1] - Y_MOUTH)
        # the retractor: a constant pull plus the spring (the joint's) along the belt, with its brake (the joint's damping); the belt carries the housing's weight (links have no gravity)
        f = BELT_PULL + K_SPR * max(out, 0.0)
        m.push(env, "belt", (0.0, -BELT_PULL, 0.0), body="yaw")
        for k in range(N_SEG):
            m.set_visible(f"{m.ENV_PATH}/beltvis/visual/seg{k}", Y_MOUTH + SEG * (k + 1) <= back[1] - STUB + 0.004 and out > STUB)
        # readings
        self.max_out = max(self.max_out, out)
        tips = m.fingertips(env)
        hands = holders(tips, h)
        if len(hands) == 2:
            self.both.trip(True)
        if self.max_out >= OUT_PASS and self.max_out - out > RETRACT_MAX:
            self.retracted.trip(True)
        if float(h.pos[2]) < ZB - FELL_Z or h.tilt_deg() > 60.0:
            self.fell.trip(True)
        into = float(front[1] - Y_RECV)
        if into >= SEAT_MIN and abs(float(h.pos[0]) - XB) < 0.004 and abs(float(h.pos[2]) - ZB) < 0.005 and h.tilt_deg() < 8.0:
            self.seated.trip(True)
        locked = bool(into >= SEAT_MIN and float(bar.joint("bar")) < BAR_DOWN and abs(float(h.pos[0]) - XB) < 0.004 and abs(float(h.pos[2]) - ZB) < 0.005 and h.tilt_deg() < 8.0)
        if out >= OUT_MIN and hands:
            self.held_out.trip(True)
        clear = bool(gk.hands_clear(tips, gk.along(h, (0.0, -H_L / 2, 0.0), (0.0, H_L / 2, 0.0), 5), CLEAR))
        still = float(np.linalg.norm(h.lin_vel)) < 0.005
        self.end.step(bool(locked and clear and still and out >= OUT_MIN))
        if bool(self.fell):
            sc = 0.0
        elif self.end.done and not self.retracted:
            sc = 1.0
        elif self.end.done:
            sc = 0.6
        elif self.seated:
            sc = 0.4
        elif self.held_out:
            sc = 0.2
        else:
            sc = 0.0
        self.score, self.full = sc, sc >= 1.0
        self.r = dict(dz_mm=float(h.pos[2] - ZB) * 1000, dx_mm=float(h.pos[0] - XB) * 1000, yaw=math.degrees(st_yaw(env)), gapL=gk.jaw_gap(tips, "left") * 1000, gapR=gk.jaw_gap(tips, "right") * 1000, out_mm=out * 1000, into_mm=into * 1000, bar_mm=float(bar.joint("bar")) * 1000, force=f, hands=",".join(hands), locked=locked, clear=clear, retracted=bool(self.retracted),
                      seated=bool(self.seated), both=bool(self.both), fell=bool(self.fell))
        log = os.environ.get("BELT_LOG")
        if log and self.steps % int(log) == 0:
            print(f"[belt] {self.steps}: " + " ".join(f"{k} {round(v, 2) if isinstance(v, float) else v}" for k, v in self.r.items()) + f" score {sc}", flush=True)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.score >= 1.0


def score(objects) -> float:
    return TRIAL.score


# -- the demo -----------------------------------------------------------------------------------------------------------------------------------------------
def _frame(a, c):
    import demokit as d

    a = np.asarray(a, float) / np.linalg.norm(a)
    c = np.asarray(c, float)
    c = c - a * (c @ a)
    c /= np.linalg.norm(c)
    return d._as_grip(np.stack([c, np.cross(a, c), a], axis=1))


def _grip_right():
    """Fingers level, pointing at the wall (turned BETA_R about z), closing on the housing's top and bottom edges; the wrist camera on the +y side."""
    b = math.radians(BETA_R)
    return _frame((math.cos(b), math.sin(b), 0.0), (0.0, 0.0, 1.0))


def _grip_left():
    """Fingers down, tilted TH_L toward -y and LEAN_L toward the wall, closing on the housing's broad faces."""
    t, lean = math.radians(TH_L), math.radians(LEAN_L)
    return _frame((math.sin(lean) * math.cos(t), -math.sin(t), -math.cos(t)), (-1.0, 0.0, 0.0))


R_OPEN, L_OPEN = -0.035, -0.016  # finger joints on approach: jaws 70 mm (round the 50 mm edges) and 32 mm (round the 12 mm faces)
FORCE = float(os.environ.get("BELT_FORCE", "30"))  # N a finger


def demo_script(ctx):
    """A scripted reference, not a policy: it reads the housing's pose from the simulator to steer.
    TASK_DEMO: full; right (right hand only: draws out and waits); letgo (the left lets go at the receiver); nolatch (pushed in 6 mm short of the latch,
    then let go); none; still (waits 1 s)."""
    import demokit as d
    from yam_arena import ik as _ik

    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode in ("still", "none"):
        yield from l.wait({"still": 1.0, "none": 3.0}[mode])
        return
    gr, gl = _grip_right(), _grip_left()

    def hpos():
        st = ctx.state("belt")
        return np.array([XB + st.joint("dx"), Y_DOCK + st.joint("out"), ZB + st.joint("dz")])

    def report(label):
        st = ctx.state("belt")
        p = hpos()
        print(f"[demo] {label}: housing {np.round(p, 4)} out {(p[1] - H_L / 2 - Y_MOUTH) * 1000:.0f} mm yaw {math.degrees(st.joint('yaw')):.1f} "
              f"latch bar {ctx.state('receiver').joint('bar') * 1000:.1f} mm  R tcp {np.round(r.tcp(), 4)}  L tcp {np.round(l.tcp(), 4)}", flush=True)

    def ramp(arm, pos, grip, n=10, t=3.0, finger=None):
        """Move to `pos` in `n` short seeded steps, the grip turned on the way (gate_collide.py seeds each step the same way)."""
        p0, R0 = _ik.tcp_pose(arm.q, arm.base)
        p0 = p0 - arm.trim
        for k in range(1, n + 1):
            sm = _ik.smoothstep(k / n)
            yield from arm.move(p0 + (np.asarray(pos, float) - p0) * sm, grip=d._as_grip(_ik.slerp_matrix(R0, grip.rot, sm)), finger=finger, t=t / n, label=f"ramp {k}")

    def slide(arm, dy, speed, label):
        """Carry the hand along y at `speed`, closing out lag as it goes (the goal moves, then holds)."""
        p0 = arm.at().copy()
        t = abs(dy) / speed

        def goal(s):
            return p0 + np.array([0.0, math.copysign(speed * min(s, t), dy), 0.0])

        yield from arm.servo(goal, tol=0.0015, timeout=t + 2.0, gain=0.15, label=label)

    GAIN, SPEED = float(os.environ.get("BELT_GAIN", "0.3")), float(os.environ.get("BELT_SPEED", "0.06"))

    def steer(arm, y_to, speed, label, tol=0.0015, extra=2.0, dz=0.0, dx=0.0):
        """Bring the housing's centre along the belt's line (x = XB, z = ZB) to y_to, steering the hand by the housing's own pose at every step."""
        y0 = float(hpos()[1])
        t = abs(y_to - y0) / speed

        def goal(s):
            return np.array([XB + dx, y0 + math.copysign(min(s * speed, abs(y_to - y0)), y_to - y0), ZB + dz])

        yield from arm.servo(goal, tol=tol, timeout=t + extra, gain=GAIN, read=hpos, label=label)

    # the right comes to the front of the docked housing while the left waits high over the hand-over point
    r_front = hpos() + np.array([-0.10, OFF_R, 0.0])
    l_high = np.array([XB, Y_HAND - OFF_L, ZB + 0.09])
    yield from ctx.together(ramp(r, r_front, gr, finger=R_OPEN), ramp(l, l_high, gl, finger=L_OPEN, t=3.5))
    yield from r.servo(r_front, tol=0.003, timeout=2.0, label="R front")
    report("right in front")
    yield from r.move(hpos() + np.array([0.0, OFF_R, 0.0]), t=1.2, label="R in")
    yield from r.servo(hpos() + np.array([0.0, OFF_R, 0.0]), tol=0.002, timeout=2.0, label="R on")
    yield from r.close(H_W, FORCE, label="R grip")
    yield from r.wait(0.3)
    report("right has the housing")
    # draw the belt out to the hand-over point
    yield from slide(r, Y_HAND - float(hpos()[1]), 0.08, "R draw")
    report("drawn to the hand-over")
    if mode == "right":
        yield from r.wait(2.0)
        report("right only, end")
        return
    # the left comes down on the housing's other half, by its faces, and closes; then the right lets go and backs out
    yield from l.servo(l_high, tol=0.004, timeout=1.5, label="L high")
    yield from l.move(hpos() + np.array([0.0, -OFF_L, DZ_L + 0.04]), t=1.2, label="L down")
    yield from l.servo(hpos() + np.array([0.0, -OFF_L, DZ_L]), tol=0.002, timeout=2.5, label="L on")
    yield from l.close(H_T, FORCE, label="L grip")
    yield from l.wait(0.4)
    report("both hold")
    shut = float(ctx.observed("left")[6])  # the left's own gripper reading, 0 shut to 1 open: shut on nothing means it missed the housing
    print(f"[demo] left gripper reads {shut:.3f} (12 mm across the jaws is about {0.012 / 0.0939:.3f})", flush=True)
    if shut < 0.06:
        print("[demo] the left missed the housing: the right keeps it", flush=True)
        yield from r.wait(2.0)
        return
    y_hold = float(hpos()[1])
    yield from ctx.together(r.open(t=0.5), steer(l, y_hold, SPEED, "L takes the pull", tol=0.002, extra=1.5))  # the left holds the housing where it is as the right lets go
    yield from r.move(r.at() + np.array([-0.08, 0.0, 0.02]), t=1.2, label="R back out")
    yield from r.move(np.array([0.24, -0.12, ZB + 0.08]), t=1.5, label="R away")
    report("left has it")
    # carry it to the receiver and push it home past the latch
    yield from steer(l, Y_SEAT - 0.025, SPEED, "L carry")
    yield from steer(l, Y_SEAT - 0.025, SPEED, "L line up", tol=0.001, extra=1.5)
    report("at the receiver")
    if mode == "letgo":  # cheat check: let go of the end for half a second at the receiver
        yield from l.open(t=0.3)
        yield from l.wait(0.5)
        report("let go")
        yield from l.wait(3.0)
        return
    for k in range(2):  # square the housing to the slot: turn the hand about the housing's vertical axis by its yaw, read from its pose
        yaw = math.degrees(ctx.state("belt").joint("yaw"))
        if abs(yaw) < 1.5:
            break
        yield from l.arc(hpos(), (0.0, 0.0, 1.0), -yaw, t=0.8, label=f"L square {k}")
        yield from steer(l, Y_SEAT - 0.025, SPEED, "L line up again", tol=0.001, extra=1.0)
        report(f"squared {k}")
    pretilt = float(os.environ.get("BELT_PRETILT", "25"))
    if pretilt:  # lean the fingers toward the slot about the jaws' axis, so the wrist is loaded against the push before it starts (the housing cannot roll)
        jaw, fing = l.grip.rot[:, 0], l.grip.rot[:, 2]
        axis = jaw if np.cross(jaw, fing)[1] > 0 else -jaw  # about the jaws' own axis, the fingertips toward the slot (+y): the pads turn in their plane
        yield from l.turn(pretilt, axis=tuple(axis), t=0.6, label="L lean")
        report("leaned")
        yield from steer(l, Y_SEAT - 0.025, SPEED, "L line up after the lean", tol=0.0015, extra=1.0)
        report("leaned, lined up")
    yaw_bias, dx_bias = float(os.environ.get("BELT_YAWB", "3.0")), float(os.environ.get("BELT_DXB", "0.0015"))
    yaw = math.degrees(ctx.state("belt").joint("yaw"))  # optional: start the housing turned a little against the push
    if yaw_bias and abs(yaw - yaw_bias) > 1.5:
        yield from l.arc(hpos(), (0.0, 0.0, 1.0), yaw_bias - yaw, t=0.6, label="L square before the push")
        report("squared before the push")

    def front():
        st = ctx.state("belt")
        yw = st.joint("yaw")
        return hpos() + np.array([-math.sin(yw), math.cos(yw), 0.0]) * (H_L / 2)

    seat_dz = float(os.environ.get("BELT_SEATDZ", "0.0"))
    for k in range(int(os.environ.get("BELT_PUSHES", "5"))):  # push in short strokes; between them, square the housing again if the push has turned it
        y_goal = Y_SEAT - 0.006 if mode == "nolatch" else Y_SEAT + 0.0015  # cheat check: stop 6 mm short, the tongue still on the ramp
        yield from steer(l, y_goal, 0.015, f"L seat {k}", tol=0.001, extra=1.2, dz=seat_dz, dx=dx_bias)
        if mode == "nolatch":
            break
        into = float(hpos()[1]) + H_L / 2 - Y_RECV
        yaw = math.degrees(ctx.state("belt").joint("yaw"))
        print(f"[demo] push {k}: in {into * 1000:.1f} mm, yaw {yaw:.1f}, tongue {ctx.state('receiver').joint('bar') * 1000:.1f} mm", flush=True)
        if into > SLOT_DEPTH - 0.0005:
            break
        if abs(yaw) > float(os.environ.get("BELT_SQ", "3.0")):  # wedged across the slot: draw it back 4 mm, square it about its front end, push again
            yield from steer(l, float(hpos()[1]) - 0.004, 0.02, f"L back off {k}", tol=0.0015, extra=0.8, dx=dx_bias)
            yaw = math.degrees(ctx.state("belt").joint("yaw"))
            yield from l.arc(front(), (0.0, 0.0, 1.0), yaw_bias - yaw, t=0.6, label=f"L square again {k}")
    yield from l.wait(0.5)
    report("seated")
    yield from l.open(t=0.5)
    yield from l.move(l.at() + np.array([0.0, 0.0, 0.08]), t=1.2, label="L up")
    yield from l.move(l.at() + np.array([-0.08, -0.05, 0.02]), t=1.0, label="L away")
    yield from l.wait(4.0)
    report("end")
