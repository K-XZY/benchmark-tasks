"""Fit a transaxle shifter fork onto its rod over the spring-loaded indexing ball: one hand holds the ball flush in its cross hole with a
flat screwdriver while the other runs the fork's bore up to it; the blade lifts away as the bore takes the ball over; then the fork goes on
until the ball clicks into the neutral groove, seen through the fork's sight hole.

The rod is fixed. The indexing ball (6.35 mm) sits in a cross hole on top of the rod on a spring, a prismatic joint whose zero is the ball
standing half out. The fork is a free body: its hub's bore is built of hull segments, with three internal grooves 7 mm apart (the middle
one neutral) and a sight hole through the top of the hub over the neutral groove. Its bore's entry edge is square: the ball standing proud
meets it about 15 degrees off its equator, inside the friction angle, so a fork pushed against an unpressed ball stops. The screwdriver is
a free body standing in a holder.

Debug:  python demokit.py task.py OUT --no-record --stills 20 --mode full,blocked
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
INSTRUCTION = "Fit the shifter fork onto the shifter rod over the indexing ball and leave it in neutral, then put the screwdriver back in its holder and let go."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own 1 s hold: score() and success() agree
HOLD_STEPS = 20
TOP = d.TABLE_TOP
DT = d.DT
mm = 1e-3

LOOKS = {
    "steel": m.Look(m.rgb(0.62, 0.63, 0.65), metallic=0.9, roughness=0.35),
    "ball": m.Look(m.rgb(0.85, 0.86, 0.88), metallic=1.0, roughness=0.1),
    "cast": m.Look(m.rgb(0.32, 0.33, 0.35), metallic=0.6, roughness=0.55),  # the fork: cast and machined
    "hole": m.Look(m.rgb(0.03, 0.03, 0.03), roughness=0.9),
    "vblock": m.Look(m.rgb(0.20, 0.35, 0.55), roughness=0.5),
    "alu": m.Look(m.rgb(0.66, 0.67, 0.68), metallic=0.5, roughness=0.6),  # the case half: cast aluminium
    "gear": m.Look(m.rgb(0.50, 0.51, 0.53), metallic=0.9, roughness=0.3),
    "handle": m.Look(m.rgb(0.85, 0.15, 0.10), roughness=0.4),
    "blade": m.Look(m.rgb(0.75, 0.76, 0.78), metallic=1.0, roughness=0.25),
    "tray": m.Look(m.rgb(0.25, 0.25, 0.27), roughness=0.7),
    "holder": m.Look(m.rgb(0.15, 0.15, 0.16), roughness=0.6),
}

# -- the rod (a lawn-tractor transaxle's shifter rod: 3/8 in steel), along y; the ball's cross hole on top -----------------------------------------
ROD_X, ROD_Z, ROD_R = 0.30, 0.030, 4.75 * mm  # the axis's x and height above the table; radius
Y_END, Y_FAR = -0.120, 0.112  # the rod's free end, out of the case's open side, and its far end in the case's end wall
BALL_Y, BALL_R, BALL_KG = -0.030, 3.175 * mm, 0.003  # 1/4 in ball, up-weighted from about 1 g for a steady solver (stated)
FLUSH = -BALL_R  # the ball's joint pressed flush: its top level with the rod's surface (joint zero: half out)
SPRING_PRE, SPRING_FLUSH = 2.0, 4.0  # N: the spring's push with the ball half out, and pressed flush (guesses, to measure)
SPRING_K = (SPRING_FLUSH - SPRING_PRE) / BALL_R
SPRING_REST = SPRING_PRE / SPRING_K  # the spring's rest above the joint's top: its preload
# -- the fork: hub on the rod, the shifting arm toward -x, the tines at its end ------------------------------------------------------
BORE_R, HUB_R, HUB_L = 5.6 * mm, 10.0 * mm, 20.0 * mm  # bore 11.2 mm on the 9.5 mm rod: 0.85 mm a side, for the 20-sided collider (stated; the real fit is closer)
GROOVE_R, GROOVES, GROOVE_W = 6.0 * mm, (-7.0 * mm, 0.0, 7.0 * mm), 4.0 * mm  # internal grooves; the middle one is neutral
FORK_KG = 0.15
ARM = (0.046, 0.012, 0.020)  # the shifting arm: from the hub's side toward -x, to the yoke; gripped across its 12 mm
ARM_C = (-0.033, 0.0, 0.0)
YOKE_C, YOKE_RI, YOKE_RO, YOKE_W = -0.070, 0.0105, 0.0145, 0.006  # the yoke's two tines: a half ring round the shift collar on the lay shaft
CASE = (0.195, 0.385, -0.105, 0.115)  # the transaxle case half: x0, x1, y0, y1
LAY_X = ROD_X + YOKE_C  # the lay shaft carrying the shift collar, under the yoke
FORK_START = (0.255, -0.200)  # the hub's middle on its rest in the tray
TRAY_T = 0.003
# -- the screwdriver: a stubby flat one, 105 mm, its frame at the blade's tip, its axis +z ----------------------------------------
DRV_SHAFT, DRV_HANDLE = 0.045, 0.060
DRV_GRIP = DRV_SHAFT + DRV_HANDLE / 2  # the grip from the tip, along the axis
DRV_KG = 0.06
LEAN = 45.0  # the screwdriver's lean from vertical, its handle away from the fork
HOLDER_XY = (0.20, 0.15)
HOLDER_FLOOR = 0.005
CLEAR = 0.03


def _ycyl(r, y0, y1, x=0.0, z=0.0, n=32):
    return m.cylinder(r, abs(y1 - y0), (x, (y0 + y1) / 2, z), axis="y", segments=n)


def _rod() -> m.Part:
    """The shifter rod fixed along y (frame origin on its axis at y 0), carried by the case's bosses, and the ball on its spring."""
    frame = [("rod", _ycyl(ROD_R, Y_END, Y_FAR, n=32), "steel"),
             ("cross_hole", m.cylinder(BALL_R + 0.3 * mm, 0.2 * mm, (0.0, BALL_Y, ROD_R - 0.05 * mm)), "hole")]
    cols = [m.col_hull("c_rod", _ycyl(ROD_R, Y_END, Y_FAR, n=24))]
    k = SPRING_K
    ball = m.slider("ball", "Z", FLUSH - 0.2 * mm, 0.0, [("ball", m.sphere(BALL_R, (0.0, BALL_Y, ROD_R), rings=10, segments=20), "ball")],
                    [m.col_hull("c_ball", m.sphere(BALL_R, (0.0, BALL_Y, ROD_R), rings=8, segments=16))], at=(0.0, BALL_Y, ROD_R),
                    mass=BALL_KG, spring=k, damping=0.5, friction=0.0, effort=50.0)
    return m.mechanism("rod", (ROD_X, 0.0, TOP + ROD_Z), [ball], LOOKS, frame, cols, friction=(0.5, 0.4))


def _ring_segment(r_in, r_out, a0, a1, y0, y1):
    """A piece of the hub's ring between angles a0 and a1 (about y, from +x toward +z) and radii r_in, r_out, from y0 to y1."""
    xz = [(r_in * math.cos(a0), r_in * math.sin(a0)), (r_out * math.cos(a0), r_out * math.sin(a0)),
          (r_out * math.cos(a1), r_out * math.sin(a1)), (r_in * math.cos(a1), r_in * math.sin(a1))]
    outline = np.array([(x, -z) for x, z in xz])  # the prism's y becomes -z once turned about x
    area = 0.5 * np.sum(outline[:, 0] * np.roll(outline[:, 1], -1) - np.roll(outline[:, 0], -1) * outline[:, 1])
    if area < 0:
        outline = outline[::-1]
    return m.prism(outline, y0, y1).turned(-90.0, "x")


def _sections():
    """The hub's axial sections: (y0, y1, inner radius); the grooves are the wider ones."""
    edges = [-HUB_L / 2]
    for g in GROOVES:
        edges += [g - GROOVE_W / 2, g + GROOVE_W / 2]
    edges.append(HUB_L / 2)
    out = []
    for k in range(len(edges) - 1):
        out.append((edges[k], edges[k + 1], GROOVE_R if k % 2 else BORE_R))
    return out


SIGHT_SEG = 5  # with 20 segments from 9 degrees, segment 5 spans 81 to 99 degrees: the top


def _fork() -> m.Part:
    parts, cols, segs = [], [], []
    n = 20
    for s, (y0, y1, r_in) in enumerate(_sections()):
        for k in range(n):
            if r_in == GROOVE_R and abs(0.5 * (y0 + y1)) < 1e-6 and k == SIGHT_SEG:
                continue  # the sight hole through the hub's top over the neutral groove
            a0, a1 = math.radians(9.0 + 18.0 * (k - 1)), math.radians(9.0 + 18.0 * k)
            seg = _ring_segment(r_in, HUB_R, a0, a1, y0, y1)
            segs.append(seg)
            cols.append(m.col_hull(f"c_hub_{s}_{k}", seg))
    parts.append(("hub", m.join(*segs), "cast"))
    parts.append(("arm", m.box(ARM, ARM_C), "cast"))
    cols.append(m.col_box("c_arm", ARM, ARM_C))
    yoke = []
    for k in range(8):  # the yoke: a half ring facing the rod, its two tines above and below the collar
        a0, a1 = math.radians(-90.0 + 22.5 * k), math.radians(-90.0 + 22.5 * (k + 1))
        seg = _ring_segment(YOKE_RI, YOKE_RO, a0, a1, -YOKE_W / 2, YOKE_W / 2).moved((YOKE_C, 0.0, 0.0))
        yoke.append(seg)
        cols.append(m.col_hull(f"c_yoke_{k}", seg))
    parts.append(("yoke", m.join(*yoke), "cast"))
    for k, sz in enumerate((-1, 1)):  # the tines' pads
        parts.append((f"pad_{k}", m.box((0.004, YOKE_W + 0.001, 0.003), (YOKE_C, 0.0, sz * (YOKE_RI + 0.0015))), "steel"))
    return m.free("fork", (FORK_START[0], FORK_START[1], TOP + TRAY_T + YOKE_RO + 0.0005), parts, cols, LOOKS, mass=FORK_KG,
                  friction=(0.5, 0.4), torsional_patch=0.01)


def _driver() -> m.Part:
    parts = [("blade", m.box((0.006, 0.0008, 0.006), (0.0, 0.0, 0.003)), "blade"),
             ("shaft", m.cylinder(0.0025, DRV_SHAFT - 0.006, (0.0, 0.0, 0.006 + (DRV_SHAFT - 0.006) / 2), segments=16), "blade"),
             ("handle", m.cylinder(0.015, DRV_HANDLE, (0.0, 0.0, DRV_SHAFT + DRV_HANDLE / 2), segments=32), "handle")]
    cols = [m.col_box("c_blade", (0.006, 0.0008, 0.006), (0.0, 0.0, 0.003)),
            m.col_cylinder("c_shaft", 0.0025, DRV_SHAFT - 0.006, (0.0, 0.0, 0.006 + (DRV_SHAFT - 0.006) / 2)),
            m.col_cylinder("c_handle", 0.015, DRV_HANDLE, (0.0, 0.0, DRV_SHAFT + DRV_HANDLE / 2))]
    return m.free("driver", (HOLDER_XY[0], HOLDER_XY[1], TOP + HOLDER_FLOOR + 0.0005), parts, cols, LOOKS, mass=DRV_KG, friction=(0.8, 0.7),
                  torsional_patch=0.01)


def _props() -> m.Part:
    parts, cols = [], []

    def box(name, size, c, look):
        parts.append((name, m.box(size, c), look))
        cols.append(m.col_box(f"c_{name}", size, c))

    hx, hy = HOLDER_XY
    box("holder_floor", (0.044, 0.044, HOLDER_FLOOR), (hx, hy, HOLDER_FLOOR / 2), "holder")
    for k, (sx, sy, cx, cy) in enumerate(((0.044, 0.016, 0.0, 0.014), (0.044, 0.016, 0.0, -0.014), (0.016, 0.012, 0.014, 0.0), (0.016, 0.012, -0.014, 0.0))):
        box(f"holder_{k}", (sx, sy, 0.025), (hx + cx, hy + cy, HOLDER_FLOOR + 0.0125), "holder")  # a block round a 12 mm square hole
    fx, fy = FORK_START
    box("tray", (0.110, 0.060, TRAY_T), (fx - 0.040, fy, TRAY_T / 2), "tray")
    rest_h = YOKE_RO - HUB_R  # a rest under the hub and arm, so the fork lies level on its yoke's tine and its hub
    box("tray_rest", (0.060, 0.030, rest_h), (fx - 0.018, fy, TRAY_T + rest_h / 2), "tray")
    # the transaxle case half, open on top and at the near end where the rod comes out
    x0, x1, y0, y1 = CASE
    wall_h = 0.050
    box("case_floor", (x1 - x0, y1 - y0, 0.006), ((x0 + x1) / 2, (y0 + y1) / 2, 0.003), "alu")
    for k, xc in enumerate((x0 + 0.003, x1 - 0.003)):
        box(f"case_side_{k}", (0.006, y1 - y0, wall_h), (xc, (y0 + y1) / 2, wall_h / 2), "alu")
        parts.append((f"flange_side_{k}", m.box((0.012, y1 - y0, 0.004), (xc, (y0 + y1) / 2, wall_h + 0.002)), "alu"))
    box("case_end", (x1 - x0, 0.010, wall_h), ((x0 + x1) / 2, y1 - 0.005, wall_h / 2), "alu")
    parts.append(("flange_end", m.box((x1 - x0, 0.014, 0.004), ((x0 + x1) / 2, y1 - 0.005, wall_h + 0.002)), "alu"))
    box("case_lip", (x1 - x0, 0.006, 0.012), ((x0 + x1) / 2, y0 + 0.003, 0.006), "alu")
    box("case_web", (x1 - x0 - 0.012, 0.008, ROD_Z - ROD_R - 0.0005), ((x0 + x1) / 2, 0.049, (ROD_Z - ROD_R - 0.0005) / 2), "alu")
    for name, (y_a, y_b, r) in (("boss_web", (0.040, 0.058, 0.011)), ("boss_end", (0.092, y1, 0.012))):  # the rod's two bores
        parts.append((name, _ycyl(r, y_a, y_b, x=ROD_X, z=ROD_Z), "alu"))
    for k, (bx, by) in enumerate(((x0 + 0.003, y0 + 0.015), (x0 + 0.003, 0.0), (x0 + 0.003, y1 - 0.02), (x1 - 0.003, y0 + 0.015), (x1 - 0.003, 0.0),
                                  (x1 - 0.003, y1 - 0.02))):  # bolt holes in the mating flange
        parts.append((f"bolt_{k}", m.cylinder(0.003, 0.0004, (bx, by, wall_h + 0.0042)), "hole"))
    rib_x = ROD_X + ARM_C[0]  # a cast rib under the fork's arm, which keeps the fork from swinging round the rod
    box("rib", (0.034, BALL_Y + 0.025 - y0, ROD_Z - ARM[2] / 2 - 0.0005), (rib_x, (BALL_Y + 0.025 + y0) / 2, (ROD_Z - ARM[2] / 2 - 0.0005) / 2), "alu")
    # the lay shaft with the shift collar the yoke straddles at neutral, and its gears (drawn only)
    parts.append(("lay_shaft", _ycyl(0.006, y0 + 0.006, 0.040, x=LAY_X, z=ROD_Z), "steel"))
    parts.append(("collar", _ycyl(0.0098, BALL_Y - 0.006, BALL_Y + 0.006, x=LAY_X, z=ROD_Z), "gear"))
    for name, (gx_, gy, r, w) in (("gear_a", (LAY_X, 0.012, 0.020, 0.010)), ("gear_b", (0.352, 0.006, 0.024, 0.010))):
        parts.append((name, _ycyl(r, gy - w / 2, gy + w / 2, x=gx_, z=ROD_Z if name == "gear_a" else 0.032), "gear"))
        n_t = 18
        for k in range(n_t):  # its teeth
            a = 2 * math.pi * k / n_t
            gz = ROD_Z if name == "gear_a" else 0.032
            tooth = m.box((0.0035, w, 0.003), (0.0, gy, 0.0)).turned(-math.degrees(a), "y").moved((gx_ + (r + 0.001) * math.cos(a), 0.0, gz + (r + 0.001) * math.sin(a)))
            parts.append((f"{name}_t{k}", tooth, "gear"))
    parts.append(("shaft_b", _ycyl(0.005, 0.053, y1, x=0.352, z=0.032), "steel"))
    return m.fixed("props", (0.0, 0.0, TOP), parts, cols, LOOKS, friction=(0.7, 0.6))


OBJECTS = [_rod(), _fork(), _driver(), _props()]


# -- the scorer ------------------------------------------------------------------------------------------------------------------


def judge(lost, dropped, on_rod, reach_ball, over_ball, seated_neutral, home, clear, held):
    """(full now, score now) from the trial's latches and the state now; `held` counts this step."""
    if lost or dropped:
        return False, 0.0
    full = bool(over_ball and seated_neutral and home and clear)
    if full and held >= HOLD_STEPS:
        return True, 1.0
    if over_ball:
        return full, 0.5
    return full, 0.25 if (on_rod and reach_ball) else 0.0


class Fit:
    def reset(self, env=None) -> None:
        self.steps, self.held = 0, 0
        self.lost, self.dropped, self.pressed, self.lifted = False, False, False, False
        self.full, self.score = False, 0.0
        self.min_front_gap = 1.0
        self.last = ""

    def step(self, env) -> None:
        self.steps += 1
        if self.steps == 1:
            m.set_target(env, "rod", "ball", SPRING_REST)  # the spring preloaded: it rests above the ball's top stop
        rod, fork, drv = m.state(env, "rod"), m.state(env, "fork"), m.state(env, "driver")
        ball_q = rod.joint("ball")
        ball_c = np.array([ROD_X, BALL_Y, TOP + ROD_Z + ROD_R + ball_q])
        fl = fork.to_local(ball_c)[0]  # the ball's middle in the fork's frame
        axis_off = float(np.hypot(fork.pos[0] - ROD_X, fork.pos[2] - TOP - ROD_Z))
        fork_axis = fork.rotation()[:, 1]
        on_rod = axis_off < 2.0 * mm and abs(float(fork_axis[1])) > 0.99 and fork.pos[1] + HUB_L / 2 > Y_END
        front = float(fork.pos[1] + HUB_L / 2)
        over_ball = on_rod and abs(float(fl[1])) < HUB_L / 2 - 1.0 * mm
        reach_ball = front > BALL_Y - BALL_R - 1.0 * mm
        g = min(GROOVES, key=lambda c: abs(float(fl[1]) - c))
        risen = ball_q > (BORE_R - BALL_R) - ROD_R + 0.1 * mm  # the ball standing above the bore's surface: up in a groove
        seated_neutral = over_ball and risen and g == 0.0 and abs(float(fl[1])) < 0.5 * mm
        tip = drv.pos
        blade_on = float(np.linalg.norm(tip - ball_c)) < BALL_R + 3.0 * mm
        if ball_q < FLUSH + 1.0 * mm:
            self.pressed = True
        if self.pressed and ball_q > -0.3 * mm and not over_ball and not blade_on:
            self.lost = True  # pressed, then let up with nothing over it: in the shop it springs out
        if fork.pos[2] > TOP + 0.025:
            self.lifted = True
        if self.lifted and fork.pos[2] < TOP + 0.012:
            self.dropped = True
        tilt = math.degrees(math.acos(min(1.0, float(drv.rotation()[2, 2]))))
        home = float(np.hypot(tip[0] - HOLDER_XY[0], tip[1] - HOLDER_XY[1])) < 0.012 and float(tip[2] - TOP) < HOLDER_FLOOR + 0.01 and tilt < 15.0
        tips = m.fingertips(env)
        clear = float(np.min(np.linalg.norm(tips - fork.pos, axis=1))) >= CLEAR + HUB_R
        self.full, self.score = judge(self.lost, self.dropped, on_rod, reach_ball, over_ball, seated_neutral, home, clear, self.held + 1)
        self.held = self.held + 1 if self.full else 0
        if not over_ball and on_rod:
            self.min_front_gap = min(self.min_front_gap, BALL_Y - front)
        self.last = (f"ball {ball_q * 1000:+.2f} mm, ball in fork y {fl[1] * 1000:+.2f} mm (groove {g * 1000:+.0f}), fork front {front * 1000:.1f} mm, "
                     f"axis off {axis_off * 1000:.2f} mm; on rod {on_rod}, over {over_ball}, risen {risen}, neutral {seated_neutral}, home {home}, clear {clear}, "
                     f"lost {self.lost}, dropped {self.dropped}; score {self.score}")
        if self.held == HOLD_STEPS:
            print(f"[fork] neutral, held 1 s: score 1.0; {self.last}", flush=True)
        if os.environ.get("FORK_DEBUG") and self.steps % 10 == 0:
            print(f"[fork] t {self.steps * DT:.1f} s: {self.last}", flush=True)


FIT = Fit()
FIT.reset()
m.on_step(FIT.step)
m.on_reset(FIT.reset)


def success(objects) -> bool:
    return FIT.full and FIT.held >= HOLD_STEPS


def score(objects) -> float:
    return FIT.score


# -- the demo: the left hand holds the ball flush with the screwdriver; the right runs the fork on, the blade lifts, on to neutral -------


def demo_script(ctx):
    """Modes: full. blocked: the fork slid on with the ball not pressed (the first gate: it must stop at the ball). hardpush: the same, then the
    fork's hand driven 15 mm past where the fork stops and held there 4 s, pushing at the arm's limit. none: the arms wait."""
    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode == "none":
        yield from l.wait(3.0)
        return
    if mode not in ("full", "blocked", "hardpush"):
        raise ValueError(f"unknown mode {mode!r}")
    rod_top = TOP + ROD_Z + ROD_R

    def fork_c():
        return ctx.state("fork").pos

    def tip():
        return ctx.state("driver").pos

    # 1. The right hand takes the fork by its shifting arm, the fingers across the arm's 12 mm.
    fk = ctx.state("fork")
    arm_pt = fk.to_world(np.array([ARM_C[0] - 0.003, 0.0, ARM[2] / 2 + 0.002]))[0]  # the TCP over the arm's top, the fingers down its sides
    yield from r.move(arm_pt + [0.0, 0.0, 0.05], grip=d.down(0.0), finger=-0.012, t=2.0, label="over the fork's arm")
    yield from r.move(arm_pt, t=1.0, label="round the arm")
    yield from r.close(ARM[1], force=30.0, label="take the fork")
    hub_from_tcp = fork_c() - r.at()
    yield from r.move(r.at() + [0.0, 0.0, 0.04], t=0.8, label="lift the fork")
    # 2. The bore lined up with the rod's free end, then run on to just short of the ball.
    before = np.array([ROD_X, Y_END - HUB_L / 2 - 0.006, TOP + ROD_Z])
    yield from r.move(before - hub_from_tcp + [0.0, 0.0, 0.01], t=2.0, label="the bore before the rod's end")
    yield from r.servo(before, tol=0.25 * mm, timeout=2.0, read=fork_c, label="the bore on the rod's axis")
    yield from r.servo(lambda s: before + [0.0, min(s / 3.0, 1.0) * (BALL_Y - 0.012 - HUB_L / 2 - before[1]), 0.0], tol=0.4 * mm, timeout=4.0,
                       read=fork_c, label="run the fork on to the ball")
    if mode in ("blocked", "hardpush"):  # on into the ball, unpressed: the fork must stop at it
        yield from r.servo(lambda s: before + [0.0, (BALL_Y - before[1]) * min(1.0, 0.7 + s / 6.0), 0.0], tol=0.4 * mm, timeout=3.0, read=fork_c,
                           label="push the fork into the unpressed ball")
        print(f"[fork] blocked: the fork's front came to {(FIT.min_front_gap) * 1000:.2f} mm short of the ball's middle; {FIT.last}", flush=True)
        if mode == "hardpush":
            ball_min = [0.0]
            yield from r.move(r.tcp() + [0.0, 0.015, 0.0], t=1.0, label="drive the hand 15 mm on past the stop")
            for _ in range(80):  # 4 s at the arm's limit
                ball_min[0] = min(ball_min[0], ctx.state("rod").joint("ball"))
                yield
            hand = (r.at() - r.tcp())[1]
            print(f"[fork] hard push: the hand {hand * 1000:.1f} mm behind its command; the ball's deepest {ball_min[0] * 1000:.2f} mm "
                  f"(flush {FLUSH * 1000:.2f}); front gap {FIT.min_front_gap * 1000:.2f} mm; {FIT.last}", flush=True)
        yield from r.wait(1.0)
        return
    # 3. The left hand takes the screwdriver from its holder by the handle.
    dv = ctx.state("driver")
    grip_pt = dv.pos + np.array([0.0, 0.0, DRV_GRIP])
    yield from l.move(grip_pt + [0.0, 0.0, 0.05], grip=d.down(0.0), finger=d.FINGER_OPEN, t=2.0, label="over the screwdriver")
    yield from l.move(grip_pt, t=1.0, label="round its handle")
    yield from l.close(0.030, force=25.0, label="take the screwdriver")
    yield from l.move(grip_pt + [0.0, 0.0, 0.07], t=1.0, label="out of the holder")
    # 4. Angled, its handle leaning away from the fork, the blade's tip on the ball just past its crown; pressed until flush.
    # the handle pinched across y and leaned about x: the press's twist about x then bears on the pads across the handle, not on the pinch axis
    tilt = d._as_grip(d.axis_angle_matrix(np.array([-1.0, 0.0, 0.0]), math.radians(LEAN)) @ d.down(0.0).rot)
    appr = tilt.rot[:, 2]
    tip_goal = np.array([ROD_X, BALL_Y + 0.8 * mm, rod_top + 0.006])
    yield from l.move(tip_goal - DRV_GRIP * appr + [0.0, 0.0, 0.02], grip=tilt, t=2.5, label="the screwdriver over the ball, angled")
    yield from l.servo(tip_goal, tol=0.4 * mm, timeout=2.0, read=tip, label="the blade over the ball")
    for k in range(5):  # down in steps, the blade re-centred over the ball at each, so that it does not slide off the ball's crown
        yield from l.servo(tip_goal - [0.0, 0.0, (k + 1) / 5 * (0.006 + 0.0030)], tol=0.3 * mm, timeout=0.8, read=tip, label="press the ball flush")
    print(f"[fork] pressed: tip {(tip() - [ROD_X, BALL_Y, rod_top]) * 1000} mm from the ball's top at flush; {FIT.last}", flush=True)
    # 5. The fork's bore run up past the ball's crown, to the blade; the blade lifts away as the bore takes the ball over.
    yield from r.servo(lambda s: before + [0.0, (BALL_Y - 0.012 - HUB_L / 2 - before[1]) + min(s / 1.5, 1.0) * (0.012 + 0.3 * mm), 0.0],
                       tol=0.3 * mm, timeout=2.5, read=fork_c, label="the bore over the ball, to the blade")
    yield from l.move(l.at() + [0.0, 0.02, 0.04], t=0.8, label="the blade lifts away")
    print(f"[fork] handed over: {FIT.last}", flush=True)
    yield from l.move(np.array([0.22, 0.12, TOP + 0.16]), t=1.5, label="the screwdriver well clear of the fork's hand")
    # 6. On to neutral: the hub's middle groove over the ball.
    for _ in range(3):  # through each click: the hand pushed on past where the fork stands, then the fork brought to neutral as read
        short = BALL_Y - fork_c()[1]
        if abs(short) < 0.3 * mm:
            break
        yield from r.move(r.tcp() + [0.0, short + math.copysign(1.0 * mm, short), 0.0], t=1.0, label="push the fork on through the click")  # from where the hand is, 1 mm past neutral
        yield from r.servo(np.array([ROD_X, BALL_Y, TOP + ROD_Z]), tol=0.3 * mm, timeout=1.5, read=fork_c, label="the fork on to neutral")
    yield from r.move(r.at(), finger=-0.012, t=0.5, label="let go of the fork")
    yield from ctx.together(r.move(r.at() + [0.0, -0.02, 0.06], t=1.0, label="right hand clear"),
                            l.move(np.array([HOLDER_XY[0], HOLDER_XY[1], TOP + HOLDER_FLOOR + DRV_GRIP + 0.06]), grip=d.down(0.0), t=2.5, label="the screwdriver over its holder"))
    yield from l.servo(np.array([HOLDER_XY[0], HOLDER_XY[1], TOP + HOLDER_FLOOR + 0.035]), tol=0.5 * mm, timeout=2.0, read=tip, label="over the hole")
    for k in range(3):  # down into the hole in steps, the tip kept over its middle
        yield from l.servo(np.array([HOLDER_XY[0], HOLDER_XY[1], TOP + HOLDER_FLOOR + 0.035 - (k + 1) / 3 * 0.033]), tol=0.6 * mm, timeout=1.0, read=tip,
                           label="into the holder")
    yield from l.open(t=0.5, label="let go of the screwdriver")
    yield from l.move(l.at() + [0.0, 0.0, 0.06], t=0.8, label="left hand clear")
    print(f"[fork] done: {FIT.last}", flush=True)
    yield from l.wait(2.0)
