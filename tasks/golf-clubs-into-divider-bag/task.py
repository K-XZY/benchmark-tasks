"""Put a golf club in a divider-top golf bag with two hands: the left hand takes the 7-iron by its shaft and the right by its grip, off the bench; together they raise it
upright, head up, and stand it on its grip end; the left lets go, and the right carries it over the bag, lowers its grip end into one of the cells at the bag's top and lets go, so it drops
in and stands, head up.

A 7-iron (0.94 m, 0.42 kg, rigid: a 27 cm rubber grip 25 mm across, a steel shaft 12 mm across, a hosel and a 75 x 50 x 20 mm blade head) lies on the bench, across in front of both arms.
A four-way stand bag (Titleist Players S4: top cuff 8.5 in = 21.6 cm, 34.8 in = 88.4 cm tall, 4.8 lb = 2.18 kg, full-length dividers) stands on the floor beside the bench, to the right arm's side:
8 mm wall, its top split into four cells by two full-depth dividers (1 cm thick); any cell counts; its rim is 13 cm above the bench top. The bag is free on the floor. The club's 0.42 kg is one body with
a balance point 63 cm from the grip end (as the simulator computes it, printed at the start of a run); the rubber grip's contact patch is taken as a 12.5 mm radius (torsional_patch). Held level by the grip alone
(22 cm from the grip end), the club needs about 1.7 N.m of grip torque; a 35-40 N two-pad pinch holds about 0.6-0.7 N.m (2/3 x mu x F x r x 2 pads, mu 1.0, r 12.5 mm). While the club's low end is inside the bag,
its velocity is damped by 8 per second (the padded bottom and fabric drag the rigid model leaves out); none in the air. Code: the scorer, which also writes that damping to the club's velocity.

    python demokit.py task.py OUT --no-record --mode full,none,one,lifted,beside,hold,high,toss
    python demokit.py task.py OUT          (final: 640 x 360, the GPU to yourself)
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
INSTRUCTION = "Put the golf club in the golf bag: pick it up off the bench, lower its grip end into one of the cells at the bag's top, then let it go so it stands in the bag, head up."
SUCCESS_HOLD_S = 0.0  # the scorer holds the end state itself (HOLD_STEPS)
SCORE_OVER = "end"
TOP = d.TABLE_TOP
MM = 1e-3

# -- the club: origin at the grip end, shaft along local +x, the blade's toe along local +y, lying flat (local z up) ------------------------
L = 0.94
GRIP_R, GRIP_L = 12.5 * MM, 0.27
SHAFT_R = 6.0 * MM
HOSEL = (0.88, 0.91)
HEAD = ((0.89, 0.94), (-0.010, 0.065), 10.0 * MM)  # x range, y range (toe), half thickness
CLUB_MASS = 0.42
MU = float(os.environ.get("GB_MU", "1.0"))  # rubber grip on the gripper's pads (estimate)
CLUB_AT = (float(os.environ.get("GB_CX", "0.45")), -0.32, TOP + 0.012)  # the grip end; the club lies on the bench along +y, its head towards the left arm's side
CLUB_QUAT = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))  # xyzw: local +x (the shaft) along +y, lying flat

# -- the bag: free, on the floor ------------------------------------------------------------------------------------------------------------
FLOOR = 0.0
BAG_C = (float(os.environ.get("GB_BX", "0.22")), float(os.environ.get("GB_BY", "-0.80")))
BAG_R, WALL, BAG_H = 0.108, 8.0 * MM, 0.884  # 4-way stand bag: top cuff 8.5 in = 21.6 cm, 34.8 in = 88.4 cm tall
BASE_T = 0.04  # the bag's bottom, inside: its top at FLOOR + BASE_T
DIV_T = 10.0 * MM  # four-way dividers run the full depth of the bag
DIV_H = BAG_H - 0.04
BAG_MASS = 2.18  # 4.8 lb; a stand bag, empty; free on the floor
RIM = FLOOR + BAG_H
BOTTOM = FLOOR + BASE_T
R_IN = BAG_R - WALL

LOOKS = {
    "grip": m.Look(m.rgb(0.05, 0.05, 0.05), roughness=0.85),
    "steel": m.Look(m.rgb(0.78, 0.79, 0.81), metallic=1.0, roughness=0.22),
    "head": m.Look(m.rgb(0.62, 0.63, 0.66), metallic=1.0, roughness=0.3),
    "bag": m.Look(m.rgb(0.07, 0.12, 0.30), roughness=0.75),
    "collar": m.Look(m.rgb(0.92, 0.92, 0.90), roughness=0.6),
    "divider": m.Look(m.rgb(0.08, 0.08, 0.09), roughness=0.7),
    "floor": m.Look(m.rgb(0.35, 0.36, 0.34), roughness=0.95),
}


def massed(collider: str, kg: float) -> str:
    """A collider that carries its own mass, so the club's balance point sits where its parts put it."""
    if os.environ.get("GB_UNIFORM"):
        return collider
    c = collider.replace('"MaterialBindingAPI"]', '"MaterialBindingAPI", "PhysicsMassAPI"]', 1)
    return c.replace("\n    {", f"\n    {{\n        float physics:mass = {kg}", 1)


def club() -> m.Part:
    grip = m.cylinder(GRIP_R, GRIP_L, (GRIP_L / 2, 0.0, 0.0), axis="x", segments=16)
    shaft = m.cylinder(SHAFT_R, HOSEL[0] - GRIP_L + 0.01, ((GRIP_L + HOSEL[0]) / 2, 0.0, 0.0), axis="x", segments=12)
    hosel = m.cylinder(8.0 * MM, HOSEL[1] - HOSEL[0], ((HOSEL[0] + HOSEL[1]) / 2, 0.0, 0.0), axis="x", segments=12)
    (x0, x1), (y0, y1), hz = HEAD
    head = m.box((x1 - x0, y1 - y0, 2 * hz), ((x0 + x1) / 2, (y0 + y1) / 2, 0.0))
    parts = [("grip", grip, "grip"), ("shaft", shaft, "steel"), ("hosel", hosel, "steel"), ("head", head, "head")]
    cols = [massed(m.col_hull("grip_c", grip), 0.05), massed(m.col_hull("shaft_c", shaft), 0.10), massed(m.col_hull("hosel_c", hosel), 0.01), massed(m.col_hull("head_c", head), 0.26)]
    return m.free("club", CLUB_AT, parts, cols, {k: LOOKS[k] for k in ("grip", "steel", "head")}, mass=CLUB_MASS, friction=(MU, 0.9 * MU), torsional_patch=float(os.environ.get("GB_PATCH", "0.0125")), start_quat=CLUB_QUAT)


def bag() -> m.Part:
    n = 16
    seg_w = 2 * BAG_R * math.tan(math.pi / n) + 2 * MM
    parts, cols = [], []
    for i in range(n):
        a = 360.0 * i / n
        rc = BAG_R - WALL / 2
        seg = m.box((WALL, seg_w, BAG_H - 0.02), (0.0, 0.0, 0.0)).turned(a, "z").moved((rc * math.cos(math.radians(a)), rc * math.sin(math.radians(a)), (BAG_H + 0.02) / 2))
        parts.append((f"wall{i}", seg, "bag"))
        cols.append(m.col_hull(f"wall{i}_c", seg))
    collar = m.lathe([(R_IN - 1 * MM, BAG_H - 0.06), (BAG_R + 4 * MM, BAG_H - 0.06), (BAG_R + 4 * MM, BAG_H), (R_IN - 1 * MM, BAG_H)], 32)
    parts.append(("collar", collar, "collar"))
    parts.append(("foot", m.cylinder(BAG_R + 3 * MM, 0.02, (0.0, 0.0, 0.01), segments=32), "collar"))
    parts.append(("bottom", m.cylinder(R_IN + 1 * MM, BASE_T - 0.02, (0.0, 0.0, (BASE_T + 0.02) / 2), segments=32), "divider"))
    cols.append(m.col_cylinder("base_c", BAG_R, BASE_T, (0.0, 0.0, BASE_T / 2)))
    for name, size in (("div_x", (2 * R_IN, DIV_T, DIV_H)), ("div_y", (DIV_T, 2 * R_IN, DIV_H))):
        c = (0.0, 0.0, BAG_H - DIV_H / 2)
        parts.append((name, m.box(size, c), "divider"))
        cols.append(m.col_box(name + "_c", size, c))
    return m.free("bag", (BAG_C[0], BAG_C[1], FLOOR), parts, cols, {k: LOOKS[k] for k in ("bag", "collar", "divider")}, mass=BAG_MASS, friction=(0.6, 0.5))


def floor() -> m.Part:
    size, c = (1.4, 1.0, 0.02), (0.0, 0.0, -0.01)
    return m.fixed("floor_slab", (0.30, -1.0, FLOOR), [("floor", m.box(size, c), "floor")], [m.col_box("floor_c", size, c)], {"floor": LOOKS["floor"]})


OBJECTS = [floor(), bag(), club()]

# -- the scorer -----------------------------------------------------------------------------------------------------------------------------
LIFT = 0.08  # 0.33: the whole club 8 cm or more above the bench, where it lay
SEAT = 0.03  # 1.0: the grip end within 3 cm of the bag's bottom, inside the bag, the head over the rim, lean 35 degrees or less,
LEAN_DEG = 35.0  # the bag within 15 degrees of upright, both whole grippers 30 mm or more from the club, still, for 2 s
TILT_DEG = 15.0
CLEAR = 0.030
BAG_DAMPING = 8.0  # 1/s
CONTROL_HZ = 20.0
HOLD_STEPS = 40  # 2 s at 20 Hz
_LAT = np.array([(x, y, z) for x in np.arange(-0.04, 0.0401, 0.01) for y in np.arange(-0.03, 0.0301, 0.01) for z in np.arange(-0.12, 0.1501, 0.01)])


def gripper_points(env) -> np.ndarray:
    """Points through both grippers' whole bodies (palm, housing, fingers) and the fingertips."""
    base = env.unwrapped
    origin = m._np(base.scene.env_origins[0])
    out = []
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        i = robot.find_bodies("gripper")[0][0]
        pose = m._np(robot.data.body_link_pose_w)[0, i]
        out.append(pose[:3] - origin + _LAT @ m.quat_matrix(pose[3:7]).T)
    return np.concatenate([np.concatenate(out), m.fingertips(env)])


def club_gap(pts, grip_end, axis) -> float:
    """The least distance from the points to the club's surface: a capsule along the shaft (grip radius) and a 45 mm ball round the head."""
    t = np.clip((pts - grip_end) @ axis, 0.0, L)
    shaft = np.linalg.norm(pts - (grip_end + t[:, None] * axis), axis=1) - GRIP_R
    head = np.linalg.norm(pts - (grip_end + 0.915 * axis), axis=1) - 0.045
    return float(min(shaft.min(), head.min()))


def moment(c: dict) -> float:
    """The tier from the facts of one moment. Keys: dropped, lifted, in_top, seated_steps."""
    if c["dropped"]:
        return 0.0
    if c["seated_steps"] >= HOLD_STEPS:
        return 1.0
    if c["in_top"]:
        return 0.67
    if c["lifted"]:
        return 0.33
    return 0.0


def fell(low: float, inside: bool, gap: float) -> bool:
    """The one void: the club's low end within 10 cm of the floor, outside the bag, with every part of both grippers CLEAR or more from it (a club still carried does not fall)."""
    return low < FLOOR + 0.10 and not inside and gap >= CLEAR


class Trial:
    def reset(self, env=None) -> None:
        self.dropped = False  # only the failure latches
        self.held = 0
        self.tier_now = 0.0
        self.info = {}
        self.n = 0
        if env is not None:
            self.step(env)

    def step(self, env) -> None:
        cs = m.state(env, "club")
        g = np.asarray(cs.pos, float)
        ax = cs.rotation()[:, 0]
        head = g + L * ax
        low = min(g[2], head[2]) - GRIP_R
        lifted = low >= CLUB_AT[2] - GRIP_R + LIFT
        bs = m.state(env, "bag")  # the bag is free: its base centre and tilt now; the club is read in the bag's own frame
        b, rb = np.asarray(bs.pos, float), bs.rotation()
        gl, hl = rb.T @ (g - b), rb.T @ (head - b)
        r_grip = float(np.hypot(gl[0], gl[1]))
        rim, bottom = RIM - FLOOR, BOTTOM - FLOOR
        inside = r_grip <= R_IN
        tilt = math.degrees(math.acos(min(1.0, max(-1.0, float(rb[2, 2])))))
        gap = club_gap(gripper_points(env), g, ax)
        if fell(low, inside, gap):
            self.dropped = True
        in_top = inside and gl[2] < rim and hl[2] > rim and tilt <= TILT_DEG
        lean = math.degrees(math.acos(min(1.0, max(-1.0, float(ax[2])))))
        still = float(np.linalg.norm(cs.lin_vel)) < 0.02 and float(np.linalg.norm(cs.ang_vel)) < 0.2
        seated = in_top and gl[2] - GRIP_R <= bottom + SEAT and lean <= LEAN_DEG and tilt <= TILT_DEG and gap >= CLEAR and still
        self.held = self.held + 1 if seated else 0
        if inside and gl[2] < rim:  # padded bottom and fabric drag, left out of the rigid bag: 8 per second on the club only while its low end is inside the bag and on the club held or free (none in the air)
            asset = env.unwrapped.scene["club"]
            asset.write_root_velocity_to_sim_index(root_velocity=asset.data.root_vel_w.torch.clone() * math.exp(-BAG_DAMPING / CONTROL_HZ))
        self.tier_now = moment(dict(dropped=self.dropped, lifted=lifted, in_top=in_top, seated_steps=self.held))
        self.info = dict(low=low - TOP, grip_r=r_grip, grip_z=gl[2] - bottom, head_z=hl[2] - rim, lean=lean, gap=gap, still=still, held=self.held, dropped=self.dropped, tilt=tilt)
        self.n += 1


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.tier_now >= 1.0


def score(objects) -> float:
    return TRIAL.tier_now


def report(tag: str) -> None:
    i = TRIAL.info
    if i:
        print(f"[gb] {tag}: tier {TRIAL.tier_now}, club low {i['low'] * 1000:.0f} mm over bench, grip end {i['grip_r'] * 1000:.0f} mm off bag axis, {i['grip_z'] * 1000:.0f} mm over bag bottom, "
              f"head {i['head_z'] * 1000:.0f} mm over rim, lean {i['lean']:.1f} deg, hands {i['gap'] * 1000:.0f} mm from club, tilt {i['tilt']:.1f} deg, still {i['still']}, held {i['held']}, dropped {i['dropped']}", flush=True)


# -- the demo -------------------------------------------------------------------------------------------------------------------------------
MODES = ("full", "none", "one", "lifted", "beside", "hold", "high", "toss")
GRIP_AT = float(os.environ.get("GB_GRIP_AT", "0.22"))  # the right hand closes on the rubber grip this far from the grip end
LEFT_AT = float(os.environ.get("GB_LEFT_AT", "0.50"))  # the left hand closes on the shaft this far from the grip end
FORCE = float(os.environ.get("GB_FORCE", "35"))  # newtons a finger: under the YAM's roughly 40 N
UPRIGHT = np.array([0.30, 0.0])  # where the grip end stands once the club is upright, 4 cm over the bench
KEEP = float(os.environ.get("GB_KEEP", "0.15"))  # the share of the shaft's lean the hand takes out each step
CARRY = float(os.environ.get("GB_CARRY", "0.10"))  # the grip end this far over the bench top while the club is carried upright
DIP = float(os.environ.get("GB_DIP", "0.07"))  # the grip end lowered this far below the rim, into the cell, before the hand lets go
CELL = np.array([BAG_C[0] + 0.05, BAG_C[1] + 0.05, RIM])  # the centre at the rim of the cell next to the bench, on the far side
CARRY_TURN = float(os.environ.get("GB_TURN", "-80"))  # degrees the hand turns about the upright shaft while it carries the club over the bag
CARRY_T = float(os.environ.get("GB_CARRY_T", "7.0"))  # seconds for the carry


def demo_script(ctx):
    """Modes: full (the left hand closes on the shaft and the right on the grip; together they lift the club, roll it a quarter turn about its shaft and raise it upright, head up,
    the hands staying on the shaft's line, and stand it on its grip end on the bench; the left lets go and backs away; the right lifts it and carries it upright over the bag's cell
    next to the bench, turning the hand on the way and keeping the shaft upright from its pose, lowers the grip end into the cell and lets
    go: the club drops to the bag's bottom and leans head up; the hand backs away); none; one (as full with the right hand alone: the club turns in the grasp, 0);
    lifted (lifted upright, 6 cm higher, and held: 0.33); beside (let go beside the bag: the club falls to the floor, void, 0); hold (as full, never lets go: 0.67); high (let go 3 cm over
    the cell, not lowered in: the club falls in at speed and ends on the floor, 0); toss (let go 30 cm over the cell: the club lands on the cuff and stays caught there, not in the bag, 0)."""
    mode = ctx.mode
    if mode not in MODES:
        raise ValueError(f"unknown mode {mode!r}")
    r, l = ctx.right, ctx.left
    asset = ctx.emb._env.unwrapped.scene["club"]
    com = m._np(asset.data.root_com_pos_w)[0] - m._np(asset.data.root_link_pos_w)[0]
    print(f"[gb] club mass {float(m._np(asset.data.default_mass).sum()):.3f} kg, balance point {float(np.linalg.norm(com)) * 100:.1f} cm from the grip end", flush=True)
    if mode == "none":
        yield from r.wait(3.0)
        report("end")
        return

    def grip_end():
        return np.asarray(ctx.state("club").pos, float)

    yield from r.wait(0.5)
    cs = ctx.state("club")
    g0, u0 = np.asarray(cs.pos, float), cs.rotation()[:, 0]
    u0 = np.array([u0[0], u0[1], 0.0]) / np.hypot(u0[0], u0[1])
    z = np.array([0.0, 0.0, 1.0])
    a = np.cross(u0, z)  # level, square to the club: the axis it is raised about
    G0 = d.down(90).rot  # straight down, the fingers closing across the shaft
    s = LEFT_AT - GRIP_AT
    hands = [(r, g0 + GRIP_AT * u0, 2 * GRIP_R)] + ([] if mode == "one" else [(l, g0 + LEFT_AT * u0, 2 * SHAFT_R)])

    def take(h, p, w):
        yield from h.move(p + 0.08 * z, grip=d._as_grip(G0), finger=d.FINGER_OPEN, t=2.5, label="over the club")
        yield from h.move(p, t=1.0, label="down round the club")
        if not (yield from h.servo(p, timeout=1.0, label="round the club")):
            h.trim = np.zeros(3)
        yield from h.close(w, FORCE, label="close on the club")

    yield from ctx.together(*(take(*x) for x in hands))
    yield from ctx.together(*(h.move(p + 0.08 * z, t=1.5, label="lift it off the bench") for h, p, _ in hands))
    for k in range(1, 6):
        Rk = d.axis_angle_matrix(u0, -math.pi / 2 * k / 5)
        yield from ctx.together(*(h.move(p + 0.08 * z, grip=d._as_grip(Rk @ G0), t=0.3, label="roll it a quarter turn") for h, p, _ in hands))
    Gr = Rk @ G0
    r0 = hands[0][1] + 0.08 * z
    r1 = np.array([UPRIGHT[0], UPRIGHT[1], TOP + GRIP_AT + 0.04])
    n = 12
    for k in range(1, n + 1):
        al = math.pi / 2 * k / n
        pr = r0 + (k / n) * (r1 - r0)
        pr[2] = max(pr[2], TOP + GRIP_AT * math.sin(al) + 0.04)
        Ra = d.axis_angle_matrix(a, al)
        steps = [r.move(pr, grip=d._as_grip(Ra @ Gr), t=0.3, label="raise it upright")]
        if mode != "one":
            steps.append(l.move(pr + s * (math.cos(al) * u0 + math.sin(al) * z), grip=d._as_grip(Ra @ Gr), t=0.3, label="raise it upright"))
        yield from ctx.together(*steps)
    report("upright")

    def steady(goal, t, label, turn=0.0):
        """Bring the right hand's TCP to `goal` over `t` s on a smooth path, turning `turn` degrees about the vertical, and each step turning the hand a little to set the
        club's shaft upright again (the wrist sags under the club): the shaft read from the club's pose; at most 1 degree a step and 20 degrees in all."""
        start, R0, Rc = r.at(), r.grip.rot.copy(), np.eye(3)
        n, worst = max(1, round(t * d.CONTROL_HZ)), (0.0, 0.0)
        for k in range(1, n + 1):
            c = ctx.state("club").rotation()[:, 0]
            e = np.cross(c, z)
            sn = float(np.linalg.norm(e))
            w = k / n
            w = w * w * (3 - 2 * w)
            Rz = d.axis_angle_matrix(z, math.radians(turn) * w)
            if 1e-4 < sn < math.sin(math.radians(20.0)):  # the correction turns with the hand
                Rn = Rz.T @ d.axis_angle_matrix(e / sn, min(math.radians(1.0), KEEP * math.asin(sn))) @ Rz @ Rc
                if (np.trace(Rn) - 1.0) / 2.0 > math.cos(math.radians(20.0)):
                    Rc = Rn
            R = Rz @ Rc @ R0
            mm, deg = r._solve(start + w * (goal - start) + r.trim, R)
            worst = (max(worst[0], mm), max(worst[1], deg))
            yield
        r.grip = d._as_grip(R)
        r._note(*worst, label)
        print(f"[gb] {label}: hand turned {math.degrees(math.acos(min(1.0, (np.trace(Rc) - 1.0) / 2.0))):.1f} deg to keep the shaft upright, lean now {math.degrees(math.acos(min(1.0, float(ctx.state('club').rotation()[2, 0])))):.1f} deg", flush=True)

    def aim(spot, t, label):
        """Carry the grip end to `spot`, upright, the hand turning on the way, then re-aim twice from where it is."""
        for tt, turn in ((t, CARRY_TURN), (0.6, 0.0), (0.6, 0.0)):
            yield from steady(r.at() + (spot - grip_end()), tt, label, turn)

    if mode != "one":
        yield from ctx.together(r.move(r.at() - 0.045 * z, t=1.0, label="stand it on its grip end"), l.move(l.at() - 0.045 * z, t=1.0, label="stand it on its grip end"))
        yield from l.open(t=0.8, label="left lets go")
        yield from l.move(l.at() - 0.10 * l.approach_axis(), t=1.0, label="left backs off the shaft")
        yield from l.move(l.at() + np.array([-0.05, 0.15, 0.0]), t=1.0, label="left away")
    yield from steady(np.array([r1[0], r1[1], TOP + CARRY + GRIP_AT]), 2.0, "lift it upright")
    report("lifted")
    if mode == "lifted":
        yield from steady(r.at() + 0.06 * z, 1.5, "lift it higher")
        yield from steady(r.at(), 3.0, "hold it upright")
        report("end")
        return
    spot = (CELL if mode != "beside" else np.array([BAG_C[0] + 0.20, BAG_C[1] + 0.05, RIM])) + np.array([0.0, 0.0, 0.03])
    yield from aim(spot, CARRY_T, "carry over the cell")
    if mode == "toss":
        yield from steady(r.at() + np.array([0.0, 0.0, 0.27]), 1.0, "raise 30 cm over the cell")
    if mode not in ("high", "toss"):
        yield from steady(r.at() - np.array([0.0, 0.0, 0.03 + DIP]), 1.5, "grip end into the cell")
    report("in")
    if mode == "hold":
        yield from steady(r.at(), 4.0, "hold it in the cell")
        report("end")
        return
    yield from r.open(t=0.3, label="let go")
    yield from r.move(r.at() - 0.12 * r.approach_axis(), t=0.5, label="hand back off the shaft")
    yield from r.wait(1.0)
    report("dropped")
    yield from r.move(r.at() + np.array([0.0, 0.0, 0.08]), t=1.0, label="hand away")
    report("backed")
    yield from r.wait(3.5)
    report("end")
