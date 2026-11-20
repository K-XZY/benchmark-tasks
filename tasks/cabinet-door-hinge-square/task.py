"""Square a crooked cabinet door: turn its top cup hinge's side screw with a screwdriver until the gap is even.

Two frameless cabinets (18 mm white melamine, 300 wide x 300 deep x 450 high) stand side by side on a 30 mm plinth, open sides toward the robot, each with a
19 mm full-overlay door (298 x 448 mm, 1.9 kg) on two Blum CLIP top 110-degree hinges (71T3550) on 0 mm cam plates (173H7100), hinged on the outer sides, with a
bar pull. The right door is square, 2.5 mm from the left. The left door hangs crooked: its top hinge's side screw is backed off, so the gap between the doors
is 1.7 mm 20 mm below the top and 2.6 mm 20 mm above the bottom. A 150 mm PH2 screwdriver lies on a post on the table.

The door is a hinged body on the real 110-degree swing, held open by the hinge's friction above 25 degrees and drawn shut by its self-close spring below that. Each hinge arm's side
screw is a real part with a cross recess; its thread and the bit's grip in the recess are code (declared below): the screwdriver's turn about its own axis, with its
bit's tip in the recess, turns the head and moves that hinge's arm sideways by the screw's pitch, and the door follows through its hinge line. Everything else is physics: the hands, the door, the bar, the driver.

    python demokit.py task.py OUT --mode full
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import gatekit as gk  # noqa: E402
import mechkit as m  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = ("The left cabinet door hangs crooked. Use the screwdriver on its hinges to make the gap between the two doors even from top to bottom, "
               "2 to 3 mm wide, then shut the door and let go.")
SUCCESS_HOLD_S = 0.0  # the scorer counts its own 1 s hold
HOLD_STEPS = 20
TOP = TABLE_TOP[SCENE]

# -- the cabinets, station metres (x forward from the arm bases, y left from the midline, z up from the table top)
XC, CD, CH, PL, T = 0.40, 0.30, 0.45, 0.03, 0.018  # front face, depth, height, plinth, board
Z1 = PL + CH
DT, DW, DH = 0.019, 0.298, 0.448  # the doors
DX1 = XC - 0.002  # the door's back face: 2 mm in front of the carcase (the door bumpers)
DX0 = DX1 - DT
ZD0 = PL + 0.001
REVEAL = 0.0025  # the doors' nominal gap
LY0, LY1 = REVEAL / 2, REVEAL / 2 + DW  # left door, across y
XH, YH = XC - 0.0115, 0.2955  # its hinge line
ZB, ZT = ZD0 + 0.080, ZD0 + DH - 0.080  # the two hinges' heights: 80 mm from the door's bottom and top
SPAN = ZT - ZB
YI = 0.300 - T  # the left side panel's inner face
PLATE = 0.0085
YARM1 = YI - PLATE  # the arm lies on the plate: its inner face
YARM0 = YARM1 - 0.010
SCREW_X = XC + 0.032  # the side-adjust screw on each arm, 32 mm back from the front
DEPTH_X = XC + 0.060  # the depth screw, behind it
HEAD_R, HEAD_L = 0.0045, 0.003
YFACE = YARM0 - 0.0003 - HEAD_L  # the head's face, where the bit seats
GAP_AT = 0.020  # the gap is read 20 mm from the door's top and bottom edges
GAP_Z = (ZD0 + GAP_AT, ZD0 + DH - GAP_AT)
PITCH = 0.0010  # m of side travel per turn: assumed (Blum gives +-2 mm of travel, not the thread)
TRAVEL = 0.0020
START_GAP = (2.6e-3, 1.7e-3)  # gap at the bottom and top, at the start
GAP_LO, GAP_HI, GAP_EVEN = 0.0020, 0.0030, 0.0005
SHUT_DEG = 2.0
HOLD_OPEN_DEG = 25.0  # the hinge's spring draws the door shut below this; above it friction holds the door where it is left
HINGE_K, HINGE_DRAG, HINGE_DAMP = 1.2, 0.15, 1.5  # N m/rad: the self-close spring; N m of friction; N m s/rad
CLEAR = 0.03
STRIP_TURNS = 0.5  # a screw forced this far past its stop strips (the arm comes off its plate)
ENGAGE_DEG = float(os.environ.get("ENGAGE_DEG", "30"))  # the bit seats in the recess with the driver this far off the screw's axis (it comes in through the door opening, angled)
TURN_MIN = 0.25  # tier 1: the top screw turned a quarter turn the right way

# -- the screwdriver: a 150 mm PH2, handle 75 mm (octagonal, 31 mm across the corners) and shaft 75 mm; origin at the handle's back end, along +x
SD_R, SD_L, SD_SR, SD_SL = 0.0155, 0.075, 0.003, 0.075
BIT = SD_L + SD_SL
SD_YAW = float(os.environ.get("SD_YAW", "65"))  # the screwdriver lies pointing 65 degrees left of forward, the way the right arm's level grip points
SD_DIR = np.array([math.cos(math.radians(SD_YAW)), math.sin(math.radians(SD_YAW)), 0.0])
SD_START = np.array([0.281, -0.171, 0.12])  # handle's back end
BLOCK_AT = SD_START[:2] + 0.040 * SD_DIR[:2]  # the post it lies on, under the handle
BLOCK = ((-0.009, 0.009), (-0.030, 0.030), (0.0, 0.1057))

LOOKS = {
    "melamine": m.Look(color=m.rgb(0.93, 0.93, 0.91), roughness=0.4),
    "door": m.Look(color=m.rgb(0.85, 0.84, 0.80), roughness=0.45),
    "ply": m.Look(color=m.rgb(0.74, 0.60, 0.42), roughness=0.75),
    "nickel": m.Look(color=m.rgb(0.76, 0.76, 0.77), metallic=0.9, roughness=0.3),
    "screw": m.Look(color=m.rgb(0.55, 0.55, 0.56), metallic=0.9, roughness=0.35),
    "dark": m.Look(color=m.rgb(0.08, 0.08, 0.09), roughness=0.6),
    "handle": m.Look(color=m.rgb(0.85, 0.15, 0.10), roughness=0.5),
    "steel": m.Look(color=m.rgb(0.70, 0.70, 0.72), metallic=1.0, roughness=0.3),
    "pull": m.Look(color=m.rgb(0.30, 0.30, 0.33), metallic=0.8, roughness=0.35),
    "stand": m.Look(color=m.rgb(0.30, 0.30, 0.32), roughness=0.6),
}


def _bx(x, y, z) -> m.Mesh:
    return m.box((x[1] - x[0], y[1] - y[0], z[1] - z[0]), ((x[0] + x[1]) / 2, (y[0] + y[1]) / 2, (z[0] + z[1]) / 2))


def _cbox(name, x, y, z) -> str:
    return m.col_box(name, (x[1] - x[0], y[1] - y[0], z[1] - z[0]), ((x[0] + x[1]) / 2, (y[0] + y[1]) / 2, (z[0] + z[1]) / 2))


def _pull(prefix: str, y: float, parts, cols):
    """A bar pull on a door's outer face, near its free edge: two standoffs and a 12 mm vertical bar 30 mm off the face."""
    xb = DX0 - 0.030
    zc = 0.255
    parts.append((f"{prefix}_bar", m.cylinder(0.006, 0.120, (xb, y, zc), axis="z", segments=16), "pull"))
    cols.append(m.col_cylinder(f"{prefix}_bar_c", 0.006, 0.120, (xb, y, zc)))
    for dz in (-0.045, 0.045):
        x = (xb, DX0)
        parts.append((f"{prefix}_so{int(dz > 0)}", _bx(x, (y - 0.004, y + 0.004), (zc + dz - 0.004, zc + dz + 0.004)), "pull"))
        cols.append(_cbox(f"{prefix}_so{int(dz > 0)}_c", x, (y - 0.004, y + 0.004), (zc + dz - 0.004, zc + dz + 0.004)))


def _cup(prefix: str, y: float, parts):
    """A hinge's cup on a door's back face, with its boss going back to the arm (looks only)."""
    for i, h in enumerate((ZB, ZT)):
        parts.append((f"{prefix}_cup{i}", m.cylinder(0.0175, 0.0008, (DX1 + 0.0004, y, h), axis="x", segments=28), "nickel"))
        parts.append((f"{prefix}_boss{i}", _bx((DX1, DX1 + 0.016), (y - 0.003, y + 0.003), (h - 0.010, h + 0.010)), "nickel"))


def _cabinets():
    parts, cols = [], []
    X = (XC, XC + CD)
    boxes = {"plinth": (X, (-0.30, 0.30), (0.0, PL)), "top": (X, (-0.30, 0.30), (Z1 - T, Z1)), "bottom": (X, (-0.30, 0.30), (PL, PL + T)),
             "back": ((XC + CD - 0.006, XC + CD), (-0.30, 0.30), (PL, Z1)), "side_l": (X, (YI, 0.30), (PL, Z1)), "side_r": (X, (-0.30, -YI), (PL, Z1)),
             "mid": (X, (-T, T), (PL, Z1))}
    for n, (x, y, z) in boxes.items():
        parts.append((n, _bx(x, y, z), "ply" if n == "plinth" else "melamine"))
        cols.append(_cbox(f"{n}_c", x, y, z))
    # the right door, square: static
    ry = (-LY1, -LY0)
    parts.append(("door_r", _bx((DX0, DX1), ry, (ZD0, ZD0 + DH)), "door"))
    cols.append(_cbox("door_r_c", (DX0, DX1), ry, (ZD0, ZD0 + DH)))
    _pull("pull_r", -0.030, parts, cols)
    _cup("hw_r", -0.2777, parts)
    for i, h in enumerate((ZB, ZT)):  # its plates and arms, on the right side panel's inner face
        parts.append((f"plate_r{i}", _bx((XC + 0.020, XC + 0.070), (-YI, -YI + PLATE), (h - 0.0225, h + 0.0225)), "nickel"))
        parts.append((f"arm_r{i}", _bx((XC + 0.012, XC + 0.082), (-YI + PLATE, -YI + PLATE + 0.010), (h - 0.007, h + 0.007)), "nickel"))
    return m.fixed("cabinets", (0.0, 0.0, TOP), parts, cols, LOOKS, friction=(0.4, 0.3))


def _frame_hardware():
    """The left door's plates and arms on the left side panel's inner face, with the depth screws (fixed, as the thread is not modelled) and the plate cams."""
    parts, cols = [], []
    for i, h in enumerate((ZB, ZT)):
        px, ay = (XC + 0.020, XC + 0.070), (YARM1, YI)
        parts.append((f"plate{i}", _bx(px, (YARM1, YI), (h - 0.0225, h + 0.0225)), "nickel"))
        cols.append(_cbox(f"plate{i}_c", px, (YARM1, YI), (h - 0.0225, h + 0.0225)))
        ax = (XC + 0.012, XC + 0.082)
        parts.append((f"arm{i}", _bx(ax, (YARM0, YARM1), (h - 0.007, h + 0.007)), "nickel"))
        cols.append(_cbox(f"arm{i}_c", ax, (YARM0, YARM1), (h - 0.007, h + 0.007)))
        dy = YARM0 - 0.0003 - HEAD_L / 2
        parts.append((f"depth{i}", m.cylinder(HEAD_R, HEAD_L, (DEPTH_X, dy, h), axis="y", segments=16), "screw"))
        cols.append(m.col_hull(f"depth{i}_c", m.cylinder(HEAD_R, HEAD_L, (DEPTH_X, dy, h), axis="y", segments=12)))
        parts.append((f"cam{i}", m.cylinder(0.0035, 0.0008, (XC + 0.012 + 0.0, YARM1 - 0.0004, h - 0.0), axis="y", segments=16), "screw"))
    return parts, cols


def _screws():
    links = []
    for k, h in (("t", ZT), ("b", ZB)):
        yc = YARM0 - 0.0003 - HEAD_L / 2
        head = m.cylinder(HEAD_R, HEAD_L, (SCREW_X, yc, h), axis="y", segments=24)
        yr = YFACE + 0.0003
        recess = [m.box((0.0055, 0.0006, 0.0012), (SCREW_X, yr, h)), m.box((0.0012, 0.0006, 0.0055), (SCREW_X, yr, h))]
        col = m.col_hull(f"head_{k}_c", m.cylinder(HEAD_R, HEAD_L, (SCREW_X, yc, h), axis="y", segments=16))
        links.append(m.hinge(f"screw_{k}", "Y", (SCREW_X, yc, h), -1e5, 1e5, [(f"head_{k}", head, "screw"), (f"rec_{k}a", recess[0], "dark"), (f"rec_{k}b", recess[1], "dark")],
                             [col], mass=0.004, spring=0.05, damping=0.002, friction=0.0, effort=5.0, inertia=(1e-7, 1e-7, 1e-7)))
    return links


def _door_link():
    cy = (LY0 + LY1) / 2
    parts = [("slab", _bx((DX0, DX1), (LY0, LY1), (ZD0, ZD0 + DH)), "door")]
    cols = [_cbox("slab_c", (DX0, DX1), (LY0, LY1), (ZD0, ZD0 + DH))]
    _pull("pull_l", 0.030, parts, cols)
    _cup("hw_l", 0.2777, parts)
    mass = 1.9
    inertia = (mass * (DW ** 2 + DH ** 2) / 12, mass * (DT ** 2 + DH ** 2) / 12, mass * (DT ** 2 + DW ** 2) / 12)
    return m.hinge("swing", "Z", (XH, YH, ZB), -110.0, 0.0, parts, cols, mass=mass, gravity=True, com=((DX0 + DX1) / 2, cy, ZD0 + DH / 2), spring=HINGE_K, damping=HINGE_DAMP,
                   effort=1.5, friction=0.0, armature=0.002, inertia=inertia, parent="tilt")


def _door():
    hw_parts, hw_cols = _frame_hardware()
    shift = m.slider("shift", "Y", -0.01, 0.01, [], [], at=(XH, YH, ZB), mass=0.2, spring=4e5, damping=2000.0, effort=4000.0, friction=0.0, inertia=(1e-3, 1e-3, 1e-3))
    tilt = m.hinge("tilt", "X", (XH, YH, ZB), -10.0, 10.0, [], [], mass=0.2, spring=3e4, damping=300.0, effort=3000.0, friction=0.0, inertia=(1e-3, 1e-3, 1e-3), parent="shift")
    return m.mechanism("door", (0.0, 0.0, TOP), [shift, tilt, _door_link(), *_screws()], LOOKS, frame_parts=hw_parts, frame_colliders=hw_cols, friction=(0.4, 0.3), solver_iters=64)


def _driver():
    L, sl = SD_L, SD_SL
    parts = [("handle", m.cylinder(SD_R, L, (L / 2, 0.0, 0.0), axis="x", segments=8), "handle"),
             ("ferrule", m.cylinder(0.006, 0.006, (L + 0.003, 0.0, 0.0), axis="x", segments=16), "steel"),
             ("shaft", m.cylinder(SD_SR, sl, (L + sl / 2, 0.0, 0.0), axis="x", segments=12), "steel"),
             ("band", m.cylinder(SD_R + 0.0005, 0.008, (L - 0.012, 0.0, 0.0), axis="x", segments=8), "dark")]
    cols = [m.col_hull("handle_c", m.cylinder(SD_R, L, (L / 2, 0.0, 0.0), axis="x", segments=8)),
            m.col_hull("shaft_c", m.cylinder(SD_SR, sl, (L + sl / 2, 0.0, 0.0), axis="x", segments=8))]
    return parts, cols


def _block():
    x, y, z = BLOCK
    return m.fixed("block", (float(BLOCK_AT[0]), float(BLOCK_AT[1]), TOP), [("block", _bx(x, y, z), "stand")], [_cbox("block_c", x, y, z)], LOOKS, yaw_deg=SD_YAW - 90.0, friction=(0.7, 0.6))


_dp, _dc = _driver()
DRIVER = m.free("driver", tuple(SD_START + np.array([0.0, 0.0, TOP])), _dp, _dc, LOOKS, mass=0.15, friction=(float(os.environ.get("DRV_MU", "0.8")), 0.7), yaw_deg=SD_YAW)
OBJECTS = [_cabinets(), _door(), _block(), DRIVER]


# -- the screws' thread: declared code -----------------------------------------------------------------------------

def _s0() -> tuple[float, float]:
    """The hinges' side offsets at the start (m, positive toward +y: the gap widens): found from the start gaps at the two reading heights."""
    k = lambda z: (z - ZB) / SPAN  # noqa: E731  lateral(z) = sb + (st - sb) k
    a = np.array([[1 - k(GAP_Z[0]), k(GAP_Z[0])], [1 - k(GAP_Z[1]), k(GAP_Z[1])]])
    st_sb = np.linalg.solve(a, np.array(START_GAP) - REVEAL)
    return float(st_sb[0]), float(st_sb[1])  # (sb, st)


SB0, ST0 = _s0()


def lateral(st: float, sb: float, z: float) -> float:
    return sb + (st - sb) * (z - ZB) / SPAN


def gaps_from(st: float, sb: float) -> tuple[float, float]:
    """The gap (bottom, top) from the two hinges' offsets, door shut."""
    return REVEAL + lateral(st, sb, GAP_Z[0]), REVEAL + lateral(st, sb, GAP_Z[1])


def _roll(R) -> float:
    """The screwdriver's turn about its own axis (its local x), from its rotation: positive is clockwise seen from behind the handle."""
    ax, side = R[:, 0], R[:, 1]
    ref = np.array([0.0, 0.0, 1.0]) - ax * ax[2]
    ref = ref / (np.linalg.norm(ref) + 1e-9)
    return math.atan2(float(np.dot(np.cross(ref, side), ax)), float(np.dot(ref, side)))


HEAD = {"t": np.array([SCREW_X, YFACE, TOP + ZT]), "b": np.array([SCREW_X, YFACE, TOP + ZB])}  # each head's face centre, station frame


class Thread:
    """Turns of each side screw from its start, and the door's hinge targets that follow."""

    def __init__(self) -> None:
        self.reset()

    def reset(self, env=None) -> None:
        self.turns = {"t": 0.0, "b": 0.0}
        self.last = None
        self.engaged = None
        self.over = 0.0  # turns forced past a stop
        self.stripped = False

    def s(self, k: str) -> float:
        return (ST0 if k == "t" else SB0) + self.turns[k] * PITCH

    def step(self, env) -> None:
        drv = m.state(env, "driver")
        R = drv.rotation()
        tip = drv.to_world(np.array([[BIT, 0.0, 0.0]]))[0]
        axis = R[:, 0]
        roll = _roll(R)
        self.engaged = None
        for k, f in HEAD.items():
            lat = math.hypot(tip[0] - f[0], tip[2] - f[2])
            ax = tip[1] - f[1]
            if lat < float(os.environ.get("ENG_LAT", "0.0025")) and -0.002 < ax < 0.0005 and float(np.dot(axis, [0.0, 1.0, 0.0])) > math.cos(math.radians(ENGAGE_DEG)):
                self.engaged = k
        if self.last is not None and self.engaged is not None:
            d = (roll - self.last + math.pi) % (2 * math.pi) - math.pi
            k = self.engaged
            new = self.turns[k] + d / (2 * math.pi)
            lo = (-TRAVEL - (ST0 if k == "t" else SB0)) / PITCH
            hi = (TRAVEL - (ST0 if k == "t" else SB0)) / PITCH
            if new > hi or new < lo:  # against the end of the slot: the screw turns no further
                self.over += abs(d) / (2 * math.pi)
                new = min(max(new, lo), hi)
                if self.over > STRIP_TURNS:
                    self.stripped = True
            else:
                self.over = max(0.0, self.over - abs(d) / (2 * math.pi))
            self.turns[k] = new
        self.last = roll
        st, sb = self.s("t"), self.s("b")
        for k in ("t", "b"):
            asset, j = m._joint(env, "door", f"screw_{k}")
            asset.set_joint_position_target(m._tensor(asset, self.turns[k] * 2 * math.pi), joint_ids=[j])
        for name, val in (("shift", sb), ("tilt", -(st - sb) / SPAN)):
            asset, j = m._joint(env, "door", name)
            asset.set_joint_position_target(m._tensor(asset, val), joint_ids=[j])


THREAD = Thread()
m.on_step(THREAD.step)
m.on_reset(THREAD.reset)


class Swing:
    """The hinge's own mechanism: friction holds the door where it is left above HOLD_OPEN_DEG; below it the self-close spring draws it shut."""

    def __init__(self) -> None:
        self.reset()

    def reset(self, env=None) -> None:
        self.rest = 0.0

    def step(self, env) -> None:
        q = -m.state(env, "door").joint("swing")  # the joint turns clockwise (negative about +z) to open: q is the opening
        slip = HINGE_DRAG / HINGE_K
        if math.degrees(q) < HOLD_OPEN_DEG:
            self.rest = 0.0
        elif abs(q - self.rest) > slip:
            self.rest = q - math.copysign(slip, q - self.rest)
        m.set_target(env, "door", "swing", -self.rest)


SWING = Swing()
m.on_step(SWING.step)
m.on_reset(SWING.reset)


# -- the scorer ------------------------------------------------------------------------------------

def link_pose(env, link: str):
    base = env.unwrapped
    asset = base.scene["door"]
    pose = m._np(asset.data.body_link_pose_w)[0, asset.body_names.index(link)]
    return m.quat_matrix(pose[3:7]), pose[:3] - m._np(base.scene.env_origins[0])


def door_gaps(env) -> tuple[float, float]:
    """The gap between the two doors (bottom, top), read at the left door's inner edge 20 mm from its bottom and top, from the door link's own pose."""
    R, t = link_pose(env, "swing")
    pts = np.array([[(DX0 + DX1) / 2, LY0, GAP_Z[0]], [(DX0 + DX1) / 2, LY0, GAP_Z[1]]])
    w = pts @ R.T + t
    return float(w[0, 1] + REVEAL / 2), float(w[1, 1] + REVEAL / 2)


def box_distance(pts: np.ndarray, lo, hi) -> float:
    d = np.maximum(np.maximum(np.asarray(lo) - pts, pts - np.asarray(hi)), 0.0)
    return float(np.linalg.norm(d, axis=1).min())


CAB_BOX = (np.array([XC - 0.060, -0.30, TOP]), np.array([XC + CD, 0.30, TOP + Z1]))  # both cabinets with their doors and pulls


def moment(c: dict) -> float:
    """The scoring text, on one moment's facts."""
    if c["stripped"]:
        return 0.0
    if c["shut"] and c["gaps_ok"]:
        return 1.0 if c["released"] else 0.5
    return 0.25 if c["turned"] else 0.0


class Trial:
    def __init__(self) -> None:
        self.reset()

    def reset(self, env=None) -> None:
        self.steps = 0
        self.end = gk.EndHold(HOLD_STEPS)
        self.score = 0.0
        self.report: dict = {}

    def step(self, env) -> None:
        self.steps += 1
        door = m.state(env, "door")
        drv = m.state(env, "driver")
        gb, gt = door_gaps(env)
        swing = -door.deg("swing")  # the opening, degrees
        shut = abs(swing) <= SHUT_DEG
        gaps_ok = bool(GAP_LO <= gb <= GAP_HI and GAP_LO <= gt <= GAP_HI and abs(gt - gb) <= GAP_EVEN)
        tips = np.concatenate([m.fingertips(env), m.hand_body(env)])  # each whole gripper: fingers, pads, palm, housing and the wrist link
        R, t = link_pose(env, "swing")
        local = (tips - t) @ R  # in the door's frame as authored
        hands = min(box_distance(tips, *CAB_BOX), box_distance(local, (DX0 - 0.060, LY0, ZD0), (DX1 + 0.02, LY1, ZD0 + DH)),
                    box_distance(drv.to_local(tips), (-0.01, -SD_R - 0.0, -SD_R), (BIT + 0.01, SD_R, SD_R)))
        tool_clear = min(float(np.linalg.norm(drv.to_world(np.array([[BIT, 0.0, 0.0]]))[0] - f)) for f in HEAD.values()) > 0.02
        still = abs(float(door.ang_vel[2])) < 0.05 and float(np.linalg.norm(drv.lin_vel)) < 0.02
        released = bool(hands >= CLEAR and tool_clear and still)
        c = {"stripped": THREAD.stripped, "shut": shut, "gaps_ok": gaps_ok, "turned": THREAD.turns["t"] >= TURN_MIN, "released": released}
        mo = moment(c)
        self.end.step(mo == 1.0)
        self.score = 1.0 if self.end.count >= HOLD_STEPS else min(mo, 0.5)  # every tier on the state now; only the gate (a stripped thread) holds
        self.report = {"step": self.steps, **c, "gap_mm": [round(1000 * gb, 2), round(1000 * gt, 2)], "swing": round(swing, 2), "hands_mm": round(1000 * hands, 1),
                       "turns": {k: round(v, 3) for k, v in THREAD.turns.items()}, "engaged": THREAD.engaged, "over": round(THREAD.over, 2), "score": self.score}


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.end.count >= HOLD_STEPS


def score(objects) -> float:
    return TRIAL.score


# -- the demo ------------------------------------------------------------------------------------

def demo_script(ctx):
    """The left hand takes the bar pull and swings the door open 45 degrees (friction holds it there) and keeps hold. The right hand takes the screwdriver from its
    block by the handle, carries it into the cabinet's open front, winds the wrist roll joint back, seats its bit in the top hinge's side screw and turns it with the wrist roll
    (one stroke of up to 224 degrees, the hand's position solved each step so the tip stays on the head; out, wrist back and in again if more is needed) until the head has turned the turns the gap needs. It takes the driver out, the left hand pushes the door shut by its
    pull and lets go, and the demo reads the gap at the door's top and bottom from the door's pose: if it is not yet within the window it opens the door and turns again by what is
    missing. Then the right hand sets the driver down, and both hands move clear. It steers on the door's pose, the head's angle and the driver's pose, all in view.

    Check modes (TASK_DEMO): none; door_only (opens and shuts the door, never touches a screw); hover (the bit seated at the top screw, never turned); bottom (turns the
    bottom hinge's screw instead); reverse (turns the top screw the wrong way); open_end (turns the top screw right but leaves the door open); hold_driver (the right way, but
    keeps hold of the driver at the end); overturn (turns the top screw on past its stop)."""
    import demokit as d
    from yam_arena.urdf import axis_angle_matrix

    env = ctx.emb._env
    l, r = ctx.left, ctx.right
    mode = ctx.mode or "full"
    if mode not in ("full", "none", "door_only", "hover", "bottom", "reverse", "open_end", "hold_driver", "overturn", "probe", "pick"):
        raise ValueError(f"unknown mode {mode!r}")
    for _ in range(10):
        yield
    if mode == "probe":
        for k in range(480):
            if k % 40 == 0:
                p = ctx.state("driver").pos
                print(f"[probe] step {k} driver {np.round(p - [0, 0, TOP], 4)} axis {np.round(ctx.state('driver').rotation()[:, 0], 2)}", flush=True)
            yield
        return
    if mode == "none":
        for _ in range(100):
            yield
        return
    hinge = np.array([XH, YH, TOP])
    up = np.array([0.0, 0.0, 1.0])
    ey = SD_DIR.copy()  # the tool's axis
    dr = lambda: ctx.state("driver")  # noqa: E731
    tip = lambda: dr().to_world(np.array([[BIT, 0.0, 0.0]]))[0]  # noqa: E731
    swing_deg = lambda: -math.degrees(ctx.state("door").joint("swing"))  # noqa: E731
    head_turns = lambda k: ctx.state("door").joint(f"screw_{k}") / (2 * math.pi)  # noqa: E731  the head's own angle, as a camera shows it

    def say(label):
        print(f"[hinge] driver {np.round(dr().pos - [0, 0, TOP], 3)}", flush=True)
        print(f"[hinge] t {ctx.t:5.1f} {label}: door {swing_deg():.1f} deg, {TRIAL.report}", flush=True)

    # -- the left hand: the pull
    def pull_now():
        R, t = link_pose(env, "swing")
        p = R @ np.array([DX0 - 0.030, 0.030, 0.255]) + t
        f = R @ np.array([1.0, 0.0, 0.0])  # the fingers point into the door's outer face, along the door's own x
        return p, d.level(math.degrees(math.atan2(f[1], f[0])))

    def grab_pull():
        p, g = pull_now()
        a = g.rot[:, 2]
        yield from l.move(p - 0.075 * a + 0.0 * up, grip=g, finger=d.FINGER_OPEN, t=2.5, label="left: in front of the pull")
        yield from l.move(p - 0.013 * a, t=1.2, label="left: round the pull")
        yield from l.close(0.012, 30.0, t=0.6, label="left: take the pull")

    def swing_to(deg):
        n = max(1, int(abs(deg - swing_deg()) // 7.5))
        step = (deg - swing_deg()) / n
        for _ in range(n):
            yield from l.arc(hinge, up, -step, t=0.8, label="left: swing the door")

    def let_go_left():
        a = l.grip.rot[:, 2]
        yield from l.open(t=0.5, label="left: let go of the pull")
        yield from l.move(l.at() - 0.07 * a, t=1.0, label="left: off the pull")

    yield from grab_pull()
    say("pull grabbed")
    yield from swing_to(45.0)
    yield from l.wait(0.5)
    say("door open")
    if mode == "door_only":
        yield from let_go_left()
        yield from l.wait(1.0)
        yield from grab_pull()
        yield from swing_to(0.0)
        yield from l.wait(1.0)
        yield from let_go_left()
        yield from l.wait(2.0)
        say("shut")
        return
    # -- the right hand: the screwdriver
    gr = d.level(SD_YAW)
    h0 = dr().to_world(np.array([[0.045, 0.0, 0.0]]))[0]
    rep = lambda lab: print(f"[hinge] {lab}: tilt {math.degrees(math.acos(float(np.clip(r.grip.rot[:, 2] @ dr().rotation()[:, 0], -1, 1)))):.1f} deg, right cmd {np.round(r.at() - [0, 0, TOP], 3)} tcp {np.round(r.tcp() - [0, 0, TOP], 3)} driver {np.round(dr().pos - [0, 0, TOP], 3)} tips z {m.fingertips(env)[12:, 2].min() - TOP:.3f}", flush=True)
    yield from r.move(h0 + np.array([0.0, 0.0, 0.10]), grip=gr, finger=d.FINGER_OPEN, t=3.0, label="right: over the screwdriver")
    rep("over")
    yield from r.move(h0, t=1.2, label="right: fingers either side of the handle")
    rep("down")
    yield from r.servo(h0, tol=0.002, timeout=1.0, label="right: at the handle")
    rep("servo")
    yield from r.close(0.029, float(os.environ.get("GRIP_F", "90")), t=0.8, label="right: grip the handle")
    rep("closed")
    yield from r.move(r.at() + np.array([0.0, 0.0, 0.05]), t=1.0, label="right: lift")
    rep("lift")
    if mode == "pick":
        return
    say("driver lifted")
    print(f"[hinge] driver {np.round(dr().pos - [0, 0, TOP], 3)}, axis {np.round(dr().rotation()[:, 0], 2)}, right tcp {np.round(r.tcp() - [0, 0, TOP], 3)}, tip {np.round(tip() - [0, 0, TOP], 3)}", flush=True)

    def turn_onto(a, b):
        a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
        v, c = np.cross(a, b), float(np.dot(a, b))
        k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        return np.eye(3) + k + k @ k / (1.0 + c)

    def square():
        for _ in range(2):
            Rd = dr().rotation()
            g = d._as_grip(turn_onto(Rd[:, 0], ey) @ r.grip.rot)
            yield from r.move(r.at(), grip=g, t=0.6, label="right: square the screwdriver")

    print(f"[hinge] before square: tilt {math.degrees(math.acos(float(np.clip(r.grip.rot[:, 2] @ dr().rotation()[:, 0], -1, 1)))):.1f} deg", flush=True)
    yield from square()
    k = "b" if mode == "bottom" else "t"
    face = lambda: np.array([SCREW_X, YFACE, TOP + (ZB if k == "b" else ZT)])  # noqa: E731  the head's face, in view
    yield from r.move(r.at() + (face() - 0.05 * ey - tip()), t=4.0, label="right: the bit in line with the screw")
    yield from r.servo(face() - 0.05 * ey, tol=0.002, timeout=2.0, read=tip, label="right: the bit in line with the screw")
    yield from square()
    yield from r.servo(face() - 0.05 * ey, tol=0.001, timeout=2.0, read=tip, label="right: the bit in line")
    say("in line")
    from yam_arena import ik
    from yam_arena.constants import FINGER_OPEN

    sign = -1.0 if mode == "reverse" else 1.0
    # which way joint 6 (the wrist roll, +-120 degrees) turns the hand about its approach axis
    ax = r.grip.rot[:, 2]
    q_try = ik.solve(r.at(), ik.matrix_to_quat_xyzw(axis_angle_matrix(ax, math.radians(10.0)) @ r.grip.rot), r.q, base_pos=r.base, iters=60)[0]
    jdir = sign * (1.0 if q_try[5] > r.q[5] else -1.0)  # joint 6's sense for the right way
    J6 = math.radians(float(os.environ.get("HINGE_J6", "112")))  # each stroke runs joint 6 from -J6 to +J6 the right way

    def roll_to(q6, t):
        """Joint 6 alone, from where it is to `q6`: the wrist roll turns the hand, and the held driver, about the hand's axis."""
        q0 = r.q.copy()
        n = max(1, round(t * d.CONTROL_HZ))
        for i in range(1, n + 1):
            q = q0.copy()
            q[5] = q0[5] + (q6 - q0[5]) * ik.smoothstep(i / n)
            r.q = q
            r.cmd = np.r_[q, float(np.clip(r.finger / FINGER_OPEN, 0.0, 1.0))]
            yield
        r.grip = d._as_grip(ik.tcp_pose(r.q, r.base)[1])

    q6_home = r.q[5]
    yield from roll_to(-jdir * J6, 2.0)  # the wrist wound back before the bit goes in

    def turn_seated(q6, t):
        """Joint 6 to `q6` with the bit held in the head: each step the hand's position is solved so the tip, offset a little from the wrist's
        axis in the grip, stays on the head's face (from where the tip sits in the hand, read when the turn starts, and the tip's observed drift)."""
        qo = ctx.observed("right")[:6]
        p_o, R_o = ik.tcp_pose(qo, r.base)
        v = R_o.T @ (tip() - p_o)  # the tip in the hand's frame
        lag = ik.tcp_pose(r.q, r.base)[0] - p_o  # commanded less observed: the arm's droop and lag here
        q0 = r.q.copy()
        fix = np.zeros(3)
        n = max(1, round(t * d.CONTROL_HZ))
        for i in range(1, n + 1):
            qq = r.q.copy()
            qq[5] = q0[5] + (q6 - q0[5]) * ik.smoothstep(i / n)
            Rn = ik.tcp_pose(qq, r.base)[1]
            fix = fix + np.clip(0.3 * (face() - tip()), -0.001, 0.001)
            q = ik.solve(face() - Rn @ v + fix + lag, ik.matrix_to_quat_xyzw(Rn), qq, base_pos=r.base, iters=60)[0]
            r.q = q
            r.cmd = np.r_[q, float(np.clip(r.finger / FINGER_OPEN, 0.0, 1.0))]
            yield
        r.grip = d._as_grip(ik.tcp_pose(r.q, r.base)[1])

    def seat():
        yield from r.servo(lambda tt: face() - max(0.02 - 0.02 * tt, 0.0010) * ey, tol=0.0006, timeout=3.0, read=tip, label="right: the bit toward the head")
        yield from r.servo(lambda tt: face() - max(0.0010 - 0.003 * tt, 0.0) * ey, tol=0.0004, timeout=2.0, read=tip, label="right: the bit into the head")

    yield from seat()
    say("bit at the head")
    print(f"[hinge] approach vs tool axis {math.degrees(math.acos(float(np.clip(r.grip.rot[:, 2] @ dr().rotation()[:, 0], -1, 1)))):.1f} deg, q {np.round(np.degrees(r.q), 0).tolist()}", flush=True)
    if mode == "hover":
        yield from r.servo(face() - 0.001 * ey, tol=0.0004, timeout=3.0, read=tip, label="hover")
        return
    need = {"overturn": 4.0}.get(mode, float(os.environ.get("HINGE_NEED", "0.63")))  # 0.28 to 0.99 turns puts both gaps in the window; 0.63 squares the door
    for n in range(8):
        left_turns = need - head_turns(k) * sign
        if left_turns < 0.15:
            break
        q6 = r.q[5] + jdir * min(2 * J6, math.radians(left_turns * 360.0))
        yield from turn_seated(q6, 3.0)
        print(f"[hinge] stroke {n}: head {head_turns(k):+.3f} turns, q6 {math.degrees(r.q[5]):.0f}, tilt {math.degrees(math.acos(float(np.clip(r.grip.rot[:, 2] @ dr().rotation()[:, 0], -1, 1)))):.1f}, engaged {THREAD.engaged}, tip to face {np.round(1000 * (tip() - face()), 1)} mm", flush=True)
        if need - head_turns(k) * sign < 0.15:
            break
        yield from r.move(r.at() - 0.008 * ey, t=0.3, label="right: the bit out of the recess")
        yield from roll_to(-jdir * J6, 2.0)
        yield from seat()
    say("turned")
    # -- out, the door shut
    yield from r.move(r.at() - 0.10 * ey, t=1.5, label="right: the bit out")
    yield from roll_to(q6_home, 2.0)  # the wrist unwound: the driver back the way it was taken
    if mode == "open_end":
        yield from r.move(r.at() + np.array([-0.05, -0.05, 0.0]), t=1.0, label="right: clear")
        yield from l.wait(3.0)
        return
    yield from swing_to(0.0)
    yield from l.wait(1.0)
    yield from let_go_left()
    yield from l.wait(1.5)
    say("door shut")
    # -- the driver down on its block, both hands clear
    if mode != "hold_driver":
        yield from r.move(h0 + np.array([0.0, 0.0, 0.05]), t=3.5, label="right: back over the block")
        yield from r.move(h0 + np.array([0.0, 0.0, 0.002]), t=1.2, label="right: down")
        yield from r.open(t=0.6, label="right: let go of the driver")
        yield from r.move(r.at() + np.array([0.0, 0.0, 0.10]), t=1.2, label="right: up")
        yield from r.move(r.at() + np.array([-0.12, -0.04, 0.0]), t=1.2, label="right: back toward its base")
    yield from l.move(l.at() + np.array([-0.15, 0.0, 0.05]), t=1.5, label="left: back toward its base")
    for _ in range(60):
        yield
    say("end")
