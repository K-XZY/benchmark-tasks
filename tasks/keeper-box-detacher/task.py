"""Keeper box at the checkout detacher: hold a locked clear keeper box flat on the counter's magnetic detacher with one hand while the other swings its lid open,
then take the razor-cartridge pack out and set it in the bagging area.

A clear polycarbonate keeper (190 x 115 x 50 mm, 250 g: a clear base tray with the lock module in a grey cap at one end, and a clear lid hinged along the far end)
lies on the checkout counter, its lock end 3 cm off the magnet of a detacher set flush into the counter (Alpha S3MDKEYST, 146 x 44 x 44 mm). Inside lies a boxed pack of razor cartridges (100 x 70 x 30 mm, 80 g).

The lock is code, stated in the README. While the lid is shut, its free end is held by the lock's spring slider (a joint drive capped at 6.0 N m, 40 N at the lid's lip).
The detacher's magnet pulls the slider back only while the lock block sits over the detacher's magnet (within 10 mm) and the keeper lies flat (its base within 3 mm of the
counter, tilt under 5 degrees). Then only the lid's snap catch holds it: a drive capped at 0.75 N m, 5 N at the lip. Past 8 degrees the lid swings free on a stiff hinge
(0.12 N m of joint friction) that holds it where it is left. Shut again off the detacher, it relocks. The magnet's own pull on the keeper body is not modelled.

    python demokit.py task.py OUT --mode full
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
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
INSTRUCTION = "Free the razor cartridges from the locked keeper: slide its lock end onto the detacher's magnet, hold it flat, open the lid, bag the pack."
SUCCESS_HOLD_S = 0.0
SCORE_OVER = "end"
HOLD_STEPS = 20  # 1 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# the keeper, in its own frame: x along its length (-x the lock end, toward the arms), origin at the base's underside, mid-box
L, W, H = 0.190, 0.115, 0.050
FLOOR, WALL_H, WALL_T = 0.004, 0.016, 0.003  # the base tray: floor, wall top, wall thickness
BLOCK_X = (-0.095, -0.069)  # the grey lock cap on the lock end: full width, full height, 4 mm short of the lid's end wall
BLOCK_H = H
LID_X = (-0.065, 0.095)
LID_T = 0.003
HINGE = (0.095, 0.0, H)  # along y, at the lid's far top edge
LIP_Y = 0.005  # how far the lid's top plate overhangs the base on its right side: a 5 mm lip, 3 mm thick, the lid's whole side edge long
LIP_SIZE = (LID_X[1] - LID_X[0], LIP_Y, LID_T)
LIP = np.array([(LID_X[0] + LID_X[1]) / 2, -(W / 2 + LIP_Y / 2), H - LID_T / 2])
PINCH_X = -0.055  # where the hand pinches the lip, by the lock end
LOCK_PT = np.array([-0.085, 0.0, 0.0])  # under the lock block's middle
PRESS_PT = np.array([-0.082, -0.025, BLOCK_H])  # where a hand holds the block down
BASE_KG, LID_KG = 0.19, 0.06

# where things are on the counter
SEAT = np.array([0.32, 0.0])  # the keeper's middle when its lock cap sits on the detacher's magnet
START_DY = 0.030  # it starts on the counter this far to the left of that, where it was set down
DET_SIZE = (0.146, 0.044, 0.044)  # the detacher housing (Alpha S3MDKEYST, listed 146 x 44 x 44 mm), set into the counter, its face flush
DET_X = (-0.055, 0.091)  # its ends along x, from the magnet's centre
SEAT_Z = TOP  # the detacher's face is flush with the counter, where the keeper lies
MAGNET = SEAT + LOCK_PT[:2]
PROD = (0.100, 0.070, 0.030, 0.080)  # the razor-cartridge pack: x, y, z, kg
PROD_AT = np.array([-0.004, 0.0, FLOOR + PROD[2] / 2 + 0.0005])  # in the keeper frame
BAG = np.array([0.28, 0.21])  # the bagging area's middle
BAG_SIZE = (0.20, 0.16)

LIP_ARM = HINGE[0] - PINCH_X  # 0.15 m from the hinge
LOCK_NM = 40.0 * LIP_ARM  # the slider: 40 N at the lip
CATCH_NM = 5.0 * LIP_ARM  # the snap catch: 5 N at the lip
FREE_DEG = 8.0  # past this the lid is off its catch
SEAT_TOL = (0.010, 0.003, 5.0)  # magnet xy, lift, tilt
OPEN_DEG = 70.0
DROP_V = 0.4  # m/s: the pack falling faster than the hand holding it by more than this (a free fall of about 8 mm) counts as dropped

LOOKS = {"base": m.Look(m.rgb(0.42, 0.43, 0.45), roughness=0.5), "ridge": m.Look(m.rgb(0.28, 0.29, 0.31), roughness=0.6),
         "clear": m.Look(m.rgb(0.80, 0.88, 0.95), roughness=0.05, opacity=0.38), "edge": m.Look(m.rgb(0.70, 0.80, 0.90), roughness=0.05, opacity=0.75),
         "lid": m.Look(m.rgb(0.80, 0.88, 0.95), roughness=0.05, opacity=0.38), "lip": m.Look(m.rgb(0.75, 0.84, 0.92), roughness=0.1, opacity=0.7),
         "housing": m.Look(m.rgb(0.10, 0.10, 0.11), roughness=0.45), "plate": m.Look(m.rgb(0.03, 0.03, 0.035), roughness=0.6),
         "magnet": m.Look(m.rgb(0.88, 0.88, 0.86), metallic=0.6, roughness=0.3), "cable": m.Look(m.rgb(0.04, 0.04, 0.04), roughness=0.7),
         "label": m.Look(m.rgb(0.80, 0.10, 0.10), roughness=0.6), "white": m.Look(m.rgb(0.95, 0.95, 0.95), roughness=0.5), "bag": m.Look(m.rgb(0.62, 0.66, 0.70), roughness=0.8),
         "pack": m.Look(m.rgb(0.10, 0.25, 0.65), roughness=0.4), "band": m.Look(m.rgb(0.95, 0.55, 0.10), roughness=0.4),
         "window": m.Look(m.rgb(0.85, 0.90, 0.95), roughness=0.1, opacity=0.5), "blade": m.Look(m.rgb(0.25, 0.26, 0.28), metallic=0.5, roughness=0.3),
         "strip": m.Look(m.rgb(0.20, 0.75, 0.30), roughness=0.4)}


def _base():
    parts, cols = [], []

    def add(name, size, c, look="clear", col=True):
        parts.append((name, m.box(size, c), look))
        if col:
            cols.append(m.col_box(name + "_c", size, c))

    add("floor", (L, W, FLOOR), (0.0, 0.0, FLOOR / 2))
    for s, nm in ((-1, "n"), (1, "p")):
        add(f"wall_{nm}", (LID_X[1] - LID_X[0], WALL_T, WALL_H), ((LID_X[0] + LID_X[1]) / 2, s * (W / 2 - WALL_T / 2), WALL_H / 2))
    add("wall_far", (WALL_T, W, WALL_H), (L / 2 - WALL_T / 2, 0.0, WALL_H / 2))
    bx = (BLOCK_X[0] + BLOCK_X[1]) / 2
    add("block", (BLOCK_X[1] - BLOCK_X[0], W, BLOCK_H), (bx, 0.0, BLOCK_H / 2), "base")
    for i in range(9):  # grip ridges on the cap's end face
        add(f"ridge{i}", (0.002, W - 0.016, 0.0025), (BLOCK_X[0] - 0.001, 0.0, 0.006 + 0.0048 * i), "ridge", col=False)
    for i in range(4):  # and across its top
        add(f"rtop{i}", (0.0025, W - 0.016, 0.0012), (BLOCK_X[0] + 0.005 + 0.005 * i, 0.0, BLOCK_H + 0.0006), "ridge", col=False)
    add("keyhole", (0.010, 0.022, 0.0013), (bx, 0.030, BLOCK_H + 0.0006), "ridge", col=False)
    for s in (-1, 1):  # the clear body's edges catch the light
        add(f"edge_{s + 1}", (L - 0.026, 0.002, 0.002), (0.013, s * (W / 2 - 0.001), 0.001), "edge", col=False)
    return parts, cols


def _lid() -> m.Link:
    parts, cols = [], []
    xm, lx = (LID_X[0] + LID_X[1]) / 2, LID_X[1] - LID_X[0]
    zh = H - WALL_H
    for name, size, c in (("top", (lx, W, LID_T), (xm, 0.0, H - LID_T / 2)),
                          ("side_n", (lx, LID_T, zh), (xm, -(W / 2 - LID_T / 2), WALL_H + zh / 2)),
                          ("side_p", (lx, LID_T, zh), (xm, W / 2 - LID_T / 2, WALL_H + zh / 2)),
                          ("end_near", (LID_T, W, zh), (LID_X[0] + LID_T / 2, 0.0, WALL_H + zh / 2)),
                          ("end_far", (LID_T, W, zh), (LID_X[1] - LID_T / 2, 0.0, WALL_H + zh / 2))):
        parts.append((name, m.box(size, c), "lid"))
        cols.append(m.col_box(name + "_c", size, c))
    for s in (-1, 1):
        parts.append((f"ledge_{s + 1}", m.box((lx, 0.002, 0.002), (xm, s * (W / 2 - 0.001), H - 0.001)), "edge"))
    parts.append(("ledge_e", m.box((0.002, W, 0.002), (LID_X[0] + 0.001, 0.0, H - 0.001)), "edge"))
    parts.append(("lip", m.box(LIP_SIZE, tuple(LIP)), "lip"))
    cols.append(m.col_box("lip_c", LIP_SIZE, tuple(LIP)))
    return m.hinge("lid", "Y", HINGE, 0.0, 110.0, parts, cols, mass=LID_KG, gravity=True, damping=0.02, friction=0.12, spring=0.0, effort=LOCK_NM)


_bp, _bc = _base()
KEEPER = m.mechanism("keeper", (SEAT[0], SEAT[1] + START_DY, SEAT_Z + 0.0002), [_lid()], LOOKS, frame_parts=_bp, frame_colliders=_bc, friction=(0.4, 0.35),
                     free_mass=BASE_KG, frame_inertia=(2.2e-4, 5.9e-4, 7.8e-4))

_zt = PROD[2] / 2
_pack_parts = [("box", m.box(PROD[:3]), "pack"), ("band", m.box((0.022, PROD[1] + 0.001, PROD[2] + 0.001), (0.036, 0.0, 0.0)), "band"),
               ("logo", m.box((0.030, 0.050, 0.0008), (-0.002, 0.0, _zt + 0.0004)), "white"),
               ("logo_bar", m.box((0.008, 0.040, 0.0009), (-0.002, 0.0, _zt + 0.0006)), "pack"),
               ("window", m.box((0.040, 0.056, 0.0008), (-0.033, 0.0, _zt + 0.0004)), "window"),
               ("count", m.box((0.012, 0.016, 0.0009), (0.036, 0.020, _zt + 0.0006)), "white")]
for _i in range(4):  # four cartridges seen through the window: a grey head with a green lubricating strip
    _y = -0.0195 + 0.013 * _i
    _pack_parts += [(f"cart{_i}", m.box((0.030, 0.010, 0.0010), (-0.033, _y, _zt + 0.0002)), "blade"),
                    (f"strip{_i}", m.box((0.004, 0.010, 0.0011), (-0.020, _y, _zt + 0.0003)), "strip")]
PACK = m.free("pack", (SEAT[0] + PROD_AT[0], SEAT[1] + START_DY + PROD_AT[1], SEAT_Z + 0.0002 + PROD_AT[2]), _pack_parts,
              [m.col_box("box_c", PROD[:3])], {k: LOOKS[k] for k in ("pack", "band", "window", "white", "blade", "strip")}, mass=PROD[3], friction=(0.6, 0.5))

# the magnetic detacher set into the counter, its face flush (origin under the magnet's centre), and the bagging area's mat
_dx = (DET_X[0] + DET_X[1]) / 2
_det_parts = [("rim", m.box((DET_SIZE[0] + 0.006, DET_SIZE[1] + 0.006, 0.0004), (_dx, 0.0, 0.0002)), "plate"),
              ("face", m.box((DET_SIZE[0], DET_SIZE[1], 0.0006), (_dx, 0.0, 0.0003)), "housing"),
              ("magnet", m.cylinder(0.016, 0.0008, (0.0, 0.0, 0.0004)), "magnet"),
              ("label", m.box((0.030, 0.030, 0.0007), (DET_X[0] + 0.019, 0.0, 0.00035)), "label"),
              ("label_s", m.box((0.008, 0.008, 0.0008), (DET_X[0] + 0.012, 0.008, 0.0004)), "white")]
for _s in (-1, 1):  # two screw heads at the far end
    _det_parts.append((f"screw{_s + 1}", m.cylinder(0.003, 0.0007, (DET_X[1] - 0.010, _s * 0.013, 0.00035)), "magnet"))
DETACHER = m.fixed("detacher", (MAGNET[0], MAGNET[1], TOP), _det_parts, (), LOOKS)
BAGMAT = m.fixed("bagging", (BAG[0], BAG[1], TOP), [("mat", m.box((BAG_SIZE[0], BAG_SIZE[1], 0.0010), (0.0, 0.0, 0.0005)), "bag")], (), LOOKS)
OBJECTS = [KEEPER, PACK, DETACHER, BAGMAT]


# -- the lock and the scorer ----------------------------------------------------------------------------


def seated(keeper) -> bool:
    """The lock block over the detacher's magnet and the keeper flat on the counter: the slider is pulled back."""
    p = keeper.to_world(LOCK_PT)[0]
    return (float(np.linalg.norm(p[:2] - MAGNET)) <= SEAT_TOL[0] and p[2] - SEAT_Z <= SEAT_TOL[1] and keeper.tilt_deg() < SEAT_TOL[2])


PACK_CORNERS = np.array([[sx * PROD[0] / 2, sy * PROD[1] / 2, sz * PROD[2] / 2] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])  # the pack's eight corners


def in_bag(pack, keeper) -> bool:
    """The pack at rest on the counter (its lowest corner within 3 mm of the top, any side down), every corner over the bagging area, its middle outside the keeper's outline."""
    c = pack.to_world(PACK_CORNERS)
    q = keeper.to_local(pack.pos)[0]
    return (max(abs(q[0]) - L / 2, abs(q[1]) - W / 2) > 0.0 and bool(np.all(np.abs(c[:, 0] - BAG[0]) <= BAG_SIZE[0] / 2))
            and bool(np.all(np.abs(c[:, 1] - BAG[1]) <= BAG_SIZE[1] / 2)) and float(c[:, 2].min()) < TOP + 0.003
            and float(np.linalg.norm(pack.lin_vel)) < 0.05)


def table_score(failed: bool, seated_shut: bool, open_now: bool, done: bool) -> float:
    """From the current state only. 1.0: the pack resting in the bagging area. 0.5: the lid open 70 degrees or more (it opens only off a seated lock).
    0.25: the keeper seated flat on the detacher with its lid shut. 0: a dropped part (off the counter, or the pack falling faster than 0.4 m/s relative to the hand holding it, so released, slipping or tumbling) voids the run for good."""
    if failed:
        return 0.0
    if done:
        return 1.0
    return 0.5 if open_now else (0.25 if seated_shut else 0.0)


class Trial:
    def reset(self, env=None):
        self.n, self.mode, self.failed = 0, "locked", None
        self.hold, self.score, self.full = 0, 0.0, False
        self.report = {}
        if env is not None:
            m.lock(env, "keeper", "lid", stiffness=2000.0, effort=LOCK_NM)

    def step(self, env):
        self.n += 1
        kp, pack = m.state(env, "keeper"), m.state(env, "pack")
        deg = kp.deg("lid")
        on = seated(kp)
        want = "free" if deg > FREE_DEG else ("catch" if on else ("locked" if deg < 2.0 else self.mode))
        if want != self.mode:
            if want == "free":
                m.unlock(env, "keeper", "lid")
            else:
                m.lock(env, "keeper", "lid", stiffness=2000.0, effort=CATCH_NM if want == "catch" else LOCK_NM)
            self.mode = want
        if self.failed is None and (kp.pos[2] < TOP - 0.05 or pack.pos[2] < TOP - 0.05):
            self.failed = "the keeper or the pack fell off the counter"
        if self.failed is None and pack.lin_vel[2] < -DROP_V:
            near = np.all(np.abs(pack.to_local(m.fingertips(env))) <= np.array(PROD[:3]) / 2 + 0.005, axis=1)
            held = bool(np.any(near & (np.abs(pack.lin_vel[2] - m.fingertip_vz(env)) < DROP_V)))  # a finger near AND moving with it
            if not held:
                self.failed = "the pack was dropped"
        moment = table_score(self.failed is not None, on and deg < 2.0, deg >= OPEN_DEG, in_bag(pack, kp))
        self.hold = self.hold + 1 if moment == 1.0 else 0
        self.full = self.hold >= HOLD_STEPS
        self.score = 1.0 if self.full else (0.5 if moment == 1.0 else moment)
        self.report = {"lid_deg": round(deg, 1), "mode": self.mode, "seated": on, "failed": self.failed, "score": self.score}
        if os.environ.get("KB_TRACE") and self.n % 20 == 0:
            p = kp.to_world(LOCK_PT)[0]
            print(f"[kb] step {self.n}: {self.report} lock_xy_err={np.linalg.norm(p[:2] - MAGNET) * 1000:.1f}mm tilt={kp.tilt_deg():.1f} "
                  f"pack={np.round(pack.pos, 3).tolist()}", flush=True)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.full


def score(objects) -> float:
    return TRIAL.score


def _world(local, dy=0.0):
    """A keeper-frame point, the keeper lying flat at SEAT shifted dy along y."""
    p = np.asarray(local, float)
    return np.array([SEAT[0] + p[0], SEAT[1] + dy + p[1], SEAT_Z + p[2]])


def demo_script(ctx):
    """The left hand slides the keeper by its side onto the detacher and presses its lock block flat; the right hand pinches the lid's lip and swings the lid open
    about its hinge; the left lets go; the left lifts the pack out and sets it in the bagging area. Positions are the scene's nominal ones, except the pack, whose
    pose the demo reads before the left takes it (it is in view); the lid's joint angle is read only for the log. Mode "onehand": the left slides the keeper on and leaves, the right alone lifts the lip. Modes "freefall", "slip_release" and "held_fast": the left lets the pack go in the air, loosens its grip 3 cm above the mat, or lowers it quickly while held. Mode "pry": the keeper stays where it starts, 30 mm off the magnet, and both hands work the lid there."""
    if ctx.mode == "none":
        yield from ctx.left.wait(2.0)
        return
    l, r = ctx.left, ctx.right
    lid = lambda: ctx.deg("keeper", "lid")
    dy = START_DY if ctx.mode == "pry" else 0.0
    # 1. slide it onto the detacher: closed fingertips against its left side, pushed about 41 mm to the right onto the magnet
    if ctx.mode != "pry":
        side = _world((0.0, W / 2 + 0.012, 0.045), START_DY)
        yield from l.move(side + [0.0, 0.0, 0.07], grip=d.down(0.0), finger=d.FINGER_CLOSED, t=2.5, label="left: over the keeper's side")
        yield from l.move(side, t=1.2, label="left: beside it")
        yield from l.move(side - [0.0, START_DY + 0.011, 0.0], t=2.5, label="left: slide it onto the detacher")
        yield from l.move(l.at() + [0.0, 0.02, 0.06], t=1.0, label="left: up")
    # 2. press the lock block flat
    press = _world(PRESS_PT + [0.0, 0.0, 0.016], dy)
    if ctx.mode != "onehand":
        yield from l.move(press + [0.0, 0.0, 0.05], grip=d.down(0.0), t=1.5, label="left: over the lock block")
        yield from l.move(press + [0.0, 0.0, 0.006], t=1.0, label="left: on the block")
        res = yield from l.press((0.0, 0.0, -1.0), force=15.0, max_travel=0.02, label="left: hold it down")
        print(f"[kb] press {res}", flush=True)
    else:
        yield from l.move((0.20, 0.25, TOP + 0.20), t=1.5, label="left: away")
    # 3. the lid's 5 mm lip, from the side, one finger above and one below it
    g = d._as_grip(axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.radians(80.0)) @ d.down(0.0).rot)  # fingers 10 deg below level along +y, closing up and down
    lip = _world(LIP + [PINCH_X - LIP[0], -0.004, 0.0], dy)
    yield from r.move(lip - 0.07 * r.approach_axis(g), grip=g, finger=d.FINGER_OPEN, t=2.5, label="right: beside the lip")
    yield from r.move(lip, t=1.2, label="right: at the lip")
    yield from r.close(LIP_SIZE[2], force=20.0, label="right: pinch the lip")
    # 4. swing the lid open about its hinge: the hand rolls about its own axis, parallel to the hinge
    yield from r.arc(_world(HINGE, dy), (0.0, 1.0, 0.0), 100.0, t=5.0, label="right: swing the lid open")
    print(f"[kb] lid at {lid():.1f} deg", flush=True)
    yield from r.open(label="right: let the lip go")
    yield from r.move(r.at() - 0.06 * r.approach_axis(), t=1.0, label="right: back off")
    yield from r.move(r.at() + [0.0, -0.04, 0.06], t=1.0, label="right: up, clear of the lid")
    yield from r.move((0.20, -0.25, TOP + 0.28), t=1.5, label="right: away")
    # 5. the left lets go of the block, lifts the pack out and sets it in the bagging area
    if ctx.mode != "onehand":
        yield from l.move(l.at() + [0.0, 0.0, 0.06], t=1.0, label="left: let go")
    pk = ctx.pos("pack") + [0.0, 0.0, 0.008]  # the pack as it now lies (in view); pinch its upper half, above the tray walls
    yield from l.move(pk + [0.0, 0.0, 0.08], grip=d.tilted(25.0, 0.0), finger=d.FINGER_OPEN, t=2.5, label="left: over the pack")
    yield from l.move(pk, t=1.2, label="left: at the pack")
    yield from l.close(PROD[1], force=30.0, label="left: grip the pack")
    yield from l.move(l.at() + [0.0, 0.0, 0.10], t=1.2, label="left: lift it out")
    if ctx.mode == "freefall":  # lazy route: let go of the pack in the air
        yield from l.open(label="left: drop it")
        yield from l.wait(2.0)
        return
    drop = np.array([BAG[0], BAG[1], TOP + PROD[2] / 2 + 0.006])
    if ctx.mode == "slip_release":  # lazy route: held 3 cm above the mat, the fingers open 3 mm a side and the pack slides out
        yield from l.move(drop + [0.0, 0.0, 0.10], t=2.0, label="left: over the bagging area")
        yield from l.move(drop + [0.0, 0.0, 0.03], t=1.2, label="left: 3 cm up")
        yield from l.close(PROD[1] + 0.006, force=0.0, label="left: loosen the grip")
        yield from l.wait(2.0)
        return
    yield from l.move(drop + [0.0, 0.0, 0.10], t=2.0, label="left: over the bagging area")
    yield from l.move(drop, t=0.25 if ctx.mode == "held_fast" else 1.2, label="left: down")  # held_fast: lazy route, a quick held descent must not void
    yield from l.open(label="left: let it go")
    yield from l.move(l.at() + [0.0, 0.0, 0.08], t=1.0, label="left: up")
    yield from r.wait(2.0)
    print(f"[kb] end: lid {lid():.1f} deg, {TRIAL.report}", flush=True)
