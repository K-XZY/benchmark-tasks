"""Turn a loose Asimov 1 leg over on the table until the knee's bearing faces up, then seat the knee side plate flush in its pocket.

Menlo's Asimov 1, Left Leg Step 8, instruction 4: "Turn the leg toward the right so we can see the part of the thigh assembly with the bearing
on the actuator facing upward toward the ceiling. Take part 600_08A and fit it on to the groove on part 600_10C ...". The leg (the thigh
assembly slotted into the knee housing 600_10C, cut off the body) lies loose on the table on its side, the plate's pocket facing away from the
robot. Nothing holds it. At 3.03 kg it is twice the arm's 1.5 kg payload, so it is turned on the table, not lifted: tipped toward the robot
over its lower edge until it lies on its back, the face up. Its knee joint is free, with the knee actuator's own friction (Menlo's actuator
table, EC-A4315-P2-36: 0.70 N m static, 0.02 N m moving, 0.034 kg m^2 reflected inertia, 0 to 1.50 rad). Then the 5 mm aluminium plate
600_08A goes into its 5 mm pocket (0.3 mm a side) while its pad enters the 35.0 mm bore (0.1 mm a side) and its boss the thigh's 12.0 mm
bearing (0.075 mm a side). Scored by effect: the leg on its back, how far the plate's top stands above the face all round, and whether its
holes are over their inserts.

Debug:  python demokit.py task.py OUT --no-record --stills 20 --mode full
Final:  python demokit.py task.py OUT --mode full
Gates:  KNEE_GATE=rest python demokit.py task.py OUT --no-record --mode gate --max-steps 200            (the leg on its side, left alone)
        KNEE_GATE=press KNEE_PUSH=10 KNEE_PUSH2=10 python demokit.py task.py OUT --no-record --mode gate --max-steps 200
        KNEE_GATE=drop KNEE_DROP=0.2,0,7,0,0 python demokit.py task.py OUT --no-record --mode gate --max-steps 200
"""

from __future__ import annotations

import dataclasses
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demokit as d  # noqa: E402
import gatekit as gk  # noqa: E402
import mechkit as m  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
SCENE = "station"
INSTRUCTION = "Turn the leg over on the table until the knee's bearing faces up, then fit the knee side plate: seat it flat in its pocket on the knee housing with its boss in the thigh's bearing and all nine holes over their inserts, leave the leg on its back, and move both hands clear."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own 1 s hold
HOLD_STEPS = 20
TOP = d.TABLE_TOP
TRACE = os.environ.get("KNEE_TRACE", "1") != "0"
GATE = os.environ.get("KNEE_GATE", "")

Z = np.load(HERE / "knee_plate.npz")  # meshes and colliders in the plate frame: origin on the boss axis at the plate's outer face, z out of the housing, x toward the narrow end

LOOKS = {
    "alu": m.Look(m.rgb(0.80, 0.81, 0.83), metallic=0.2, roughness=0.5),  # 7075, machined, bead-blasted
    "pa12": m.Look(m.rgb(0.17, 0.17, 0.18), roughness=0.85),  # MJF PA12, as printed: dark grey, matte
    "thigh": m.Look(m.rgb(0.36, 0.37, 0.39), metallic=0.5, roughness=0.45),
    "steel": m.Look(m.rgb(0.72, 0.72, 0.74), metallic=0.95, roughness=0.25),
    "pom": m.Look(m.rgb(0.92, 0.92, 0.89), roughness=0.5),  # the kitting post, acetal
}


def _rz(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _quat_xyzw(R: np.ndarray) -> tuple:
    w, x, y, z = gk.quat_wxyz(R)
    return (float(x), float(y), float(z), float(w))


# -- the leg (plate frame, m) ----------------------------------------------------------------------------------------------------------------------
# The thigh assembly (URDF left_hip_yaw_link: 1.949 kg, its centre of mass and inertia) and the housing 600_10C (1,069.5 cm3 of PA12 at
# 1.01 g/cm3: 1.080 kg; centre and inertia from its mesh), on the knee joint. The knee's axis is the boss's (URDF: left_knee_joint 0, 0,
# -0.19564 from left_hip_yaw_joint, along its y; here 49.25 mm under the face), so the bearing stays on the pocket's axis at any knee angle.
# The URDF's flexion (0 to 1.50 rad) turns the housing about the plate frame's -z: here the joint runs 0 to -1.50 rad.
THIGH_KG, HOUSING_KG = 1.949, 1.080
LEG_KG = THIGH_KG + HOUSING_KG
THIGH_COM, THIGH_I = (-0.07536, 0.00108, -0.05032), (0.00145, 0.00364, 0.00380)  # URDF, in the plate frame's axes
HOUSING_COM, HOUSING_I = (0.10431, -0.00021, -0.05043), (0.00176, 0.00866, 0.00868)
KNEE_AT = (0.0, 0.0, -0.04925)
KNEE_LO = -86.0  # degrees: 1.50 rad of flexion
# the knee actuator, unpowered (Menlo, Asimov 1 joint design and actuation, actuator 4, left_knee_joint, EC-A4315-P2-36)
STATIC = float(os.environ.get("KNEE_STATIC", "0.70"))  # N m: the torque that starts it turning
DYNAMIC = float(os.environ.get("KNEE_DYNAMIC", "0.02"))  # N m: the lower drag once it turns
ARMATURE = 0.0339552  # kg m^2: the rotor's inertia reflected through the gearbox
PIN_K, PIN_D = 50.0, 0.3  # the drive that stands in for dry friction (kit README: a joint's own friction grows with its load): N m/rad, N m s/rad
MU = float(os.environ.get("KNEE_MU", "0.3"))  # PA12 and aluminium on the table, the post and each other

# Its rest facets on the table, from the colliders below: outward normal n and offset d (n.p = d on the facet). On its
# side: the thigh's and the housing's +y side down, the face toward +x. On its back: face up, 4.3 degrees off level (no pad, nothing added).
# Between them it tips 85.9 degrees over the edge where the two meet, toward the robot; its centre of mass rises 62 -> 79 mm, then falls to 53.
N_SIDE, D_SIDE = np.array([0.04094, 0.99916, -0.00233]), 0.06269
N_BACK, D_BACK = np.array([0.02856, 0.06850, -0.99724]), 0.10309
EDGE_C, EDGE_U = np.array([-0.00749, 0.06282, -0.09928]), np.array([-0.99883, 0.04086, -0.02580])


def _lying(n_down, x_world) -> np.ndarray:
    """The rotation (plate frame into the world) that puts the plate frame's `n_down` straight down, with its x as near `x_world` as that allows."""
    e3 = -np.asarray(n_down, float) / np.linalg.norm(n_down)
    e1 = np.array([1.0, 0.0, 0.0]) - e3[0] * e3
    e1 /= np.linalg.norm(e1)
    w1 = np.asarray(x_world, float) - np.asarray(x_world, float)[2] * np.array([0.0, 0.0, 1.0])
    w1 /= np.linalg.norm(w1)
    W = np.column_stack([w1, np.cross([0.0, 0.0, 1.0], w1), [0.0, 0.0, 1.0]])
    E = np.column_stack([e1, np.cross(e3, e1), e3])
    return W @ E.T


def _axis_rot(u, deg: float) -> np.ndarray:
    return gk.rot(tuple(u), deg)


END_XY = np.array([0.31, -0.06])  # the pocket's axis once the leg is on its back
R_BACK = _lying(N_BACK, (0.0, -1.0, 0.0))  # on its back: the calf toward -y, the face up
O_BACK = np.array([END_XY[0], END_XY[1], TOP + D_BACK])
EDGE_W = R_BACK @ EDGE_U  # the edge in the world, the leg on its back
EDGE_P = O_BACK + R_BACK @ EDGE_C
TIP_DEG = 85.87  # the angle between the two facets' planes
R_SIDE = _axis_rot(EDGE_W, TIP_DEG) @ R_BACK  # the same leg on its side, tipped back over the edge: the face toward +x
O_SIDE = EDGE_P + _axis_rot(EDGE_W, TIP_DEG) @ (O_BACK - EDGE_P)
LEVEL_TOL = 10.0  # degrees: the face within this of level (on its back it rests 4.3 off; on its side, 90)
REST_TOL = 0.012  # m: the face's origin within this of its height on its back: the leg lies on the table, not held up
SLIDE_TOL = 0.015  # m: the leg moves less than this from where it lay when the plate first reached the pocket's mouth
ROCK_TOL = 3.0  # degrees: and its face turns less than this from how it lay then (pressed at the plate's edge it rocks 2 onto its other three points)
LEG0 = os.environ.get("KNEE_LEG0", "back" if GATE in ("seated", "drop", "press", "back") else "side")

POST = np.array([0.20, -0.17])
POST_H, POST_R, SOCKET_R, SOCKET_D = 0.030, 0.025, 0.018, 0.012  # an acetal kitting post, 50 mm across, 30 mm tall, with a 36 mm socket 12 mm deep
PLATE_KG = 0.1198  # 42.62 cm3 (the STEP) of 7075 at 2.81 g/cm3
PLATE0 = np.array([POST[0], POST[1], TOP + POST_H + 0.005])  # boss down in the socket, the sheet's underside on the post's rim
R_POST = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])  # the plate on the post: its x toward -y


def _mesh(key: str) -> m.Mesh:
    return m.Mesh(np.asarray(Z[f"{key}_v"], float), np.asarray(Z[f"{key}_f"], int))


def _hulls(prefix: str, which=None) -> list:
    n = int(Z[f"n_{prefix}"])
    return [m.col_hull(f"c_{prefix}{i}", _mesh(f"{prefix}{i}")) for i in (range(n) if which is None else which)]


def _hull2d(p: np.ndarray) -> np.ndarray:
    """The convex hull of 2D points, counter-clockwise (monotone chain)."""
    p = np.unique(np.round(p, 6), axis=0)
    if len(p) < 3:
        return p

    def half(pts):
        out = []
        for q in pts:
            while len(out) >= 2 and (out[-1][0] - out[-2][0]) * (q[1] - out[-2][1]) - (out[-1][1] - out[-2][1]) * (q[0] - out[-2][0]) <= 0:
                out.pop()
            out.append(q)
        return out

    lo, hi = half(p), half(p[::-1])
    return np.array(lo[:-1] + hi[:-1])


def _slab(name: str, pts: np.ndarray, x0: float, x1: float, keep=None):
    """A convex prism round the leg's points between plate x0 and x1 (their outline across the leg, at most 24 corners), as a collider."""
    sel = (pts[:, 0] >= x0) & (pts[:, 0] <= x1) & (np.ones(len(pts), bool) if keep is None else keep)
    if sel.sum() < 4:
        return None
    h = _hull2d(pts[sel][:, 1:3])
    if len(h) > 24:
        h = h[np.unique(np.linspace(0, len(h) - 1, 24).round().astype(int))]
    pr = m.prism([tuple(c) for c in h], x0, x1)  # built along z from an (a, b) outline: here (y, z) along x
    return m.col_hull(name, m.Mesh(np.c_[pr.v[:, 2], pr.v[:, 0], pr.v[:, 1]], pr.f))


def _housing_colliders() -> list:
    """The housing's outside, for the table and the hands: coarse prisms, kept clear of the pocket's fine colliders (plate x
    -49 to 110 mm, y within 49 mm, z above -16 mm), where the plate goes."""
    hv = np.asarray(Z["housing_v"], float)
    below = hv[:, 2] < -0.0175
    out = [_slab(f"c_hb{i}", hv, a, b, below) for i, (a, b) in enumerate([(-0.062, -0.035), (-0.035, -0.005), (-0.005, 0.025), (0.025, 0.055), (0.055, 0.085), (0.085, 0.112)])]
    for i, (a, b) in enumerate([(-0.050, -0.010), (-0.010, 0.030), (0.030, 0.070), (0.070, 0.112)]):
        out.append(_slab(f"c_hn{i}", hv, a, b, (hv[:, 1] < -0.0505) & ~below))  # the face's rim either side of the pocket
        out.append(_slab(f"c_hf{i}", hv, a, b, (hv[:, 1] > 0.0505) & ~below))
    out += [_slab(f"c_hc{i}", hv, a, b) for i, (a, b) in enumerate([(0.112, 0.150), (0.150, 0.190), (0.190, 0.230), (0.230, 0.266)])]  # the calf
    return [c for c in out if c]


def _thigh_colliders() -> list:
    tv = np.asarray(Z["thigh_v"], float)
    out = [_slab(f"c_th{i}", tv, a, b) for i, (a, b) in enumerate([(-0.273, -0.240), (-0.240, -0.200), (-0.200, -0.160), (-0.160, -0.120), (-0.120, -0.085), (-0.085, -0.050)])]
    return [c for c in out if c]


def _bearing() -> m.Mesh:
    """The flanged bearing 600_11X, moved onto the boss's axis with its 5 mm width from 11 to 16 mm under the face (its STEP mesh lies along x)."""
    b = _mesh("bearing")
    c = (b.v.min(0) + b.v.max(0)) / 2
    return m.Mesh((b.v - c) @ np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]]).T + [0.0, 0.0, -0.0135], b.f)


BEARING_HULLS = range(217, 265)  # of the leg's fine colliders: the bearing's 12.0 mm bore, 11 to 16 mm under the face (the rest are the pocket's)


def _leg():
    if LEG0 == "back":
        R, o = R_BACK, O_BACK + [0.0, 0.0, 0.0005]
    else:
        R, o = R_SIDE, O_SIDE + [0.0, 0.0, 0.0005]
    knee = m.hinge("knee", "Z", KNEE_AT, KNEE_LO, 0.0, [("housing", _mesh("housing"), "pa12")],
                   _hulls("lc", [i for i in range(int(Z["n_lc"])) if i not in BEARING_HULLS]) + _housing_colliders(),
                   mass=HOUSING_KG, spring=PIN_K, damping=PIN_D, effort=STATIC, friction=0.0, armature=ARMATURE, gravity=True, com=HOUSING_COM, inertia=HOUSING_I)
    part = m.mechanism("leg", tuple(o), [knee], LOOKS, frame_parts=[("thigh", _mesh("thigh"), "thigh"), ("bearing", _bearing(), "steel")],
                       frame_colliders=_thigh_colliders() + _hulls("lc", BEARING_HULLS), friction=(MU, MU), solver_iters=64, free_mass=THIGH_KG,
                       start_quat=_quat_xyzw(R), frame_inertia=THIGH_I)
    if part is None:  # check_score's stand-in kit
        return None
    build = part.build

    def with_com() -> str:  # mechkit gives a mechanism's frame no centre of mass of its own: the thigh's, from the URDF
        text = build()
        line = f"float physics:mass = {THIGH_KG}"
        assert text.count(line) == 1
        return text.replace(line, line + f"\n        point3f physics:centerOfMass = ({THIGH_COM[0]:.5f}, {THIGH_COM[1]:.5f}, {THIGH_COM[2]:.5f})")

    return dataclasses.replace(part, build=with_com)


def _post():
    prof = [(0.0, 0.0), (POST_R, 0.0), (POST_R, POST_H), (SOCKET_R, POST_H), (SOCKET_R, POST_H - SOCKET_D), (0.0, POST_H - SOCKET_D)]
    cols = [m.col_cylinder("c_floor", SOCKET_R + 0.001, POST_H - SOCKET_D, (0.0, 0.0, (POST_H - SOCKET_D) / 2))]
    n = 16
    for k in range(n):
        a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        ri = SOCKET_R / math.cos(math.pi / n)
        wedge = [(ri * math.cos(a0), ri * math.sin(a0)), (POST_R * math.cos(a0), POST_R * math.sin(a0)), (POST_R * math.cos(a1), POST_R * math.sin(a1)), (ri * math.cos(a1), ri * math.sin(a1))]
        cols.append(m.col_hull(f"c_rim{k}", m.prism(wedge, 0.0, POST_H)))
    return m.fixed("post", (POST[0], POST[1], TOP), [("post", m.lathe(prof, 48), "pom")], cols, LOOKS, friction=(MU, MU))


def _plate_start():
    """Where the plate starts: on the post; or, for the gates (the leg on its back), seated or dropped over the pocket."""
    if GATE in ("seated", "press"):
        return tuple(O_BACK), _quat_xyzw(R_BACK)
    if GATE == "drop":
        dx, dy, h, tilt, yaw = (float(v) for v in os.environ.get("KNEE_DROP", "0,0,7,0,0").split(","))
        R = R_BACK @ _rz(yaw) @ gk.rot((0.0, 1.0, 0.0), tilt)  # dx, dy in mm along the plate's x and y; tilt about its y, yaw about its axis, degrees
        return tuple(O_BACK + R_BACK @ np.array([dx, dy, h]) * 1e-3), _quat_xyzw(R)
    return tuple(PLATE0), _quat_xyzw(R_POST)


def _plate():
    pos, q = _plate_start()
    return m.free("plate", pos, [("plate", _mesh("plate"), "alu")], _hulls("pc"), LOOKS, mass=PLATE_KG, friction=(MU, MU), torsional_patch=0.015,
                  solver_iters=64, start_quat=q)


OBJECTS = [_leg(), _post(), _plate()]


class Pin:
    """Dry friction in the knee actuator: the drive's rest stays put while its spring holds (up to STATIC); turned harder, the knee slips and
    the rest follows it at DYNAMIC's stretch, so a turning knee meets the lower drag."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.rest = 0.0

    def step(self, env):
        q = m.state(env, "leg").joint("knee")
        if abs(q - self.rest) > STATIC / PIN_K:
            self.rest = q - math.copysign(DYNAMIC / PIN_K, q - self.rest)
            m.set_target(env, "leg", "knee", self.rest)


PIN = Pin()
m.on_step(PIN.step)
m.on_reset(PIN.reset)


def pocket(leg) -> tuple[np.ndarray, np.ndarray]:
    """The pocket's frame in the world, from the leg's state: the housing turned on the knee from the thigh (the frame), about the axis
    through the pocket's centre, which so stays at the frame's origin. Returns its rotation and its origin (the pocket's axis at the face)."""
    return leg.rotation() @ _rz(math.degrees(leg.joint("knee"))), np.asarray(leg.pos, float).copy()


# -- the scorer ------------------------------------------------------------------------------------------------------------------------------------

# points on the plate's top face round its outline (plate frame, m): the round end, the fillets, the neck and its end. Their height above the
# housing's face is what a depth gauge or a straightedge with feeler gauges reads; seated, all are 0.
RIM = np.array([[-0.0385, 0.0, 0.0], [-0.027, 0.027, 0.0], [-0.027, -0.027, 0.0], [0.0, 0.0385, 0.0], [0.0, -0.0385, 0.0], [0.030, 0.021, 0.0],
                [0.030, -0.021, 0.0], [0.065, 0.0185, 0.0], [0.065, -0.0185, 0.0], [0.1085, 0.0, 0.0]])
HOLES = np.c_[np.asarray(Z["holes"], float), np.zeros(9)]  # the nine 3.2 mm holes, over the housing's nine M3 inserts when seated
MOUTH_H, ENTER_H, FLUSH_H = 0.0100, 0.0040, 0.0005  # the rim's height above the face, every point: on the mouth, 1 mm in, flush
ON_AXIS = 0.0015  # the boss's axis within 1.5 mm of the bearing's
HOLE_TOL = 0.00045  # each hole's axis within 0.45 mm of its insert's: a 2.4 mm pin gauge then passes the 3.2 mm hole (0.4 mm either way) into the M3 thread's 2.5 mm minor diameter (0.05 more)
HANDS_CLEAR = 0.10  # both grippers 10 cm from the plate and the face round it, so the last frames show the seated plate
STILL = 0.001  # m/s: under 1 mm in the 1 s hold
LEG_CLEAR = 0.08  # no fingertip within 8 cm of the leg's centre line (thigh's end, knee, calf's end): the leg is not held or propped at the end
LEG_STILL, LEG_SPIN = 0.002, 0.01  # m/s and rad/s: the leg still in the 1 s hold
OUTLINE = np.array([[0.0397 * math.cos(a), 0.0397 * math.sin(a), 0.0] for a in np.linspace(math.radians(37), math.radians(323), 30)]
                   + [[0.05, -0.0197, 0.0], [0.09, -0.0197, 0.0]] + [[0.09 + 0.0197 * math.cos(a), 0.0197 * math.sin(a), 0.0] for a in np.linspace(-math.pi / 2, math.pi / 2, 9)]
                   + [[0.09, 0.0197, 0.0], [0.05, 0.0197, 0.0], [0.0397 * math.cos(math.radians(37)), 0.0397 * math.sin(math.radians(37)), 0.0]])
CLEAR_RING = np.array([[0.055 * math.cos(a), 0.055 * math.sin(a), 0.0] for a in np.linspace(0, 2 * math.pi, 25)])  # the housing's face round the pocket
CALF = np.array([0.235, -0.006, -0.048])  # plate frame, the housing at the knee's zero: the middle of the calf's end
THIGH_END = np.array([-0.250, 0.004, -0.078])  # plate frame, the thigh: the middle of its top end
KNEE_C = np.array(KNEE_AT)  # the knee's centre, on its axis


def knee_now(env) -> dict:
    """What the scorer reads, every step: the plate in the pocket's frame as the leg lies now, and the leg's pose."""
    s = m.state(env, "plate")
    leg = m.state(env, "leg")
    R, o = pocket(leg)
    to_pocket = lambda p: (np.atleast_2d(p) - o) @ R  # noqa: E731
    rim = to_pocket(s.to_world(RIM))
    holes = to_pocket(s.to_world(HOLES))
    axis = to_pocket(s.pos)[0]
    Rp = R.T @ s.rotation()
    yaw = math.degrees(math.atan2(Rp[1, 0], Rp[0, 0]))
    tilt = math.degrees(math.acos(float(np.clip(Rp[2, 2], -1.0, 1.0))))
    face = math.degrees(math.acos(float(np.clip(R[2, 2], -1.0, 1.0))))  # the face's normal from straight up: 4.3 on its back, 90 on its side
    return {"h": rim[:, 2], "off": float(np.hypot(axis[0], axis[1])), "axis": axis, "holes": float(np.max(np.hypot(holes[:, 0] - HOLES[:, 0], holes[:, 1] - HOLES[:, 1]))),
            "yaw": yaw, "tilt": tilt, "tips": m.fingertips(env), "outline": s.to_world(OUTLINE), "ring": o + CLEAR_RING @ R.T, "speed": float(np.linalg.norm(s.lin_vel)),
            "z": float(s.pos[2]), "xy": s.pos[:2].copy(), "face": face, "o": o, "leg_speed": float(np.linalg.norm(leg.lin_vel)), "leg_spin": float(np.linalg.norm(leg.ang_vel)),
            "ends": np.vstack([o + R @ CALF, leg.to_world(THIGH_END)]), "line": np.vstack([leg.to_world(THIGH_END), o + R @ KNEE_C, o + R @ CALF])}


LADDER = gk.Ladder(("turned", 0.25), ("entered", 0.5), ("flush", 0.75), ("done", 1.0))


class Trial:
    """The trial's latches and the score. Zero, latched: the plate or the leg off the table (below its top, or past its edge)."""

    def __init__(self):
        self.fell = gk.Latch("the plate or the leg fell off the table")
        self.hold = gk.EndHold(HOLD_STEPS)
        self.lifted = gk.Latch("a gripper lifted the plate out of the post")
        self.early = gk.Latch("the plate was set in the pocket before the leg was turned on its back")
        self.slid = gk.Latch("the leg slid or rocked once the plate had reached the pocket's mouth")
        self.reset()

    def reset(self, env=None):
        self.n, self.now, self.full, self.best, self.carry, self.mouth_at = 0, None, False, 0.0, 0, None
        self.fell.reset(); self.hold.reset(); self.lifted.reset(); self.early.reset(); self.slid.reset()

    def step(self, env):
        self.n += 1
        self.now = knee_now(env)
        st = self.now
        off_table = lambda p: p[2] < TOP - 0.01 or abs(p[1]) > 0.62 or p[0] < -0.1 or p[0] > 0.75  # noqa: E731
        self.fell.trip(off_table(np.r_[st["xy"], st["z"]]) or off_table(st["o"]) or bool(np.any(st["ends"][:, 2] < TOP - 0.02)))
        # carried: 10 steps in a row 1 cm up out of the post with a fingertip within 3 cm of its outline (a grasp, not a flick or a one-sided prod)
        near = st["z"] > PLATE0[2] + 0.010 and float(gk.min_distance(st["tips"], [st["outline"]])) < 0.03
        self.carry = self.carry + 1 if near else 0
        self.lifted.trip(self.carry >= 10)
        mouth = st["off"] <= ON_AXIS and st["h"].max() <= MOUTH_H and st["h"].min() >= -0.001 and st["tilt"] < 20.0
        # order: the plate on the pocket's mouth while the leg is not on its back (set in first, the leg turned after) is not the act
        self.early.trip(mouth and not self._on_back(st))
        # the leg held still while the plate goes in: from the step the plate first reaches the mouth (never re-anchored), the pocket moves under
        # SLIDE_TOL and the face turns under ROCK_TOL; a leg that slides away or rocks under the press does not count the seat
        if mouth and self.mouth_at is None:
            self.mouth_at = (st["o"].copy(), st["face"])
        if self.mouth_at is not None:
            self.slid.trip(float(np.linalg.norm(st["o"] - self.mouth_at[0])) > SLIDE_TOL or abs(st["face"] - self.mouth_at[1]) > ROCK_TOL or st["face"] > LEVEL_TOL)
        self.full = self.hold.step(self.reached(st)["done"])
        sc = self.score()
        self.best = max(self.best, sc)
        if TRACE and (self.n % 20 == 0 or (GATE and (self.n < 60 and self.n % 5 == 0))):
            print(f"[knee] step {self.n}: face {st['face']:.2f} deg from up, at {np.round(1000 * (st['o'] - [0, 0, TOP]), 1)} mm; rim above the face {1000 * st['h'].min():.2f} to "
                  f"{1000 * st['h'].max():.2f} mm, boss axis off {1000 * st['off']:.3f} mm, holes off {1000 * st['holes']:.2f} mm, tilt {st['tilt']:.2f} deg; score {sc}", flush=True)

    @staticmethod
    def _on_back(st) -> bool:
        """The leg on its back on the table: the face within LEVEL_TOL of level, and at the height it rests at there (not held up off the table)."""
        return st["face"] <= LEVEL_TOL and abs(st["o"][2] - (TOP + D_BACK)) <= REST_TOL

    def reached(self, st) -> dict:
        h = st["h"]
        turned = self._on_back(st)
        mouth = turned and st["off"] <= ON_AXIS and h.max() <= MOUTH_H and h.min() >= -0.001 and st["tilt"] < 20.0  # boss down: a flipped plate (tilt 180) reads nothing
        entered = mouth and h.max() <= ENTER_H
        flush = entered and np.abs(h).max() <= FLUSH_H and st["tilt"] < 1.0
        clear = gk.hands_clear(st["tips"], [st["outline"], st["ring"]], HANDS_CLEAR) and gk.hands_clear(st["tips"], [st["line"]], LEG_CLEAR)
        still = st["speed"] < STILL and st["leg_speed"] < LEG_STILL and st["leg_spin"] < LEG_SPIN
        done = flush and st["holes"] <= HOLE_TOL and clear and still
        return {"turned": turned, "entered": entered, "flush": flush, "done": done}

    def score(self) -> float:
        if self.now is None:
            return 0.0
        r = self.reached(self.now)
        r["done"] = self.full and r["done"]  # the 1 s hold was met, and now: seated, hands clear, plate and leg still
        if (self.early or self.slid) and not GATE:
            r["flush"] = r["done"] = False
        if not (GATE or self.lifted):  # a plate that no gripper ever lifted from the post (thrown, tapped, dropped from far) scores at most 0.5
            r["flush"] = r["done"] = False
        return LADDER.score(r, failed=bool(self.fell))


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    if GATE:  # the gates watch the plate for their whole length
        return False
    return bool(TRIAL.full and TRIAL.lifted and not TRIAL.early and not TRIAL.slid and TRIAL.score() >= 1.0 and not TRIAL.fell)


def score(objects) -> float:
    return TRIAL.score()


# -- the demo ------------------------------------------------------------------------------------------------------------------------------------

PINCH_X = 0.062  # the pinch across the neck's flats (39.4 mm apart), this far along the plate from the boss's axis
RELEASE_H = float(os.environ.get("KNEE_RELEASE", "0.0065"))  # let go with the plate this far above seated: its underside 1.5 mm over the face
PRESS_X = float(os.environ.get("KNEE_PRESS_X", "0.010"))  # press here on the plate's top, between the boss's axis and its centre of mass
PRESS_DEPTH = float(os.environ.get("KNEE_PRESS_DEPTH", "0.020"))  # the fingertips sent this far below flush
RELEASE_TOL = float(os.environ.get("KNEE_RELEASE_TOL", "0.00012"))
GRIP_CALF = np.array([0.235, -0.016, -0.0467])  # plate frame: the right hand's grip on the calf's end, across its 59 mm thickness, 21 mm in from its top side
GRIP_THIGH = np.array([-0.252, -0.028, -0.078])  # plate frame: the left hand's grip on the thigh's top end, across its 60 mm thickness
TURN_T = float(os.environ.get("KNEE_TURN_T", "5.0"))  # seconds for the turn
TURN_DEG = float(os.environ.get("KNEE_TURN_DEG", "66"))  # the hands turn the leg this far (its centre of mass passes over the edge at 38), then
# let go: their wrists cannot follow it the last 20 degrees, and it settles onto its back from there


def _tips(env, side: str = "right") -> np.ndarray:
    """The middle of one hand's two fingertips (the points 147 mm out along the gripper)."""
    t = m.fingertips(env)
    t = t[12:24] if side == "right" else t[0:12]
    return t[0::2].mean(axis=0)


def _check_frame(env):
    """The knee's sign against the simulator's own pose of the housing (once, in the gates and the demo)."""
    asset = env.unwrapped.scene["leg"]
    i = asset.body_names.index("knee")
    pose = asset.data.body_link_pose_w[0, i].detach().cpu().numpy().astype(float)
    R_sim = m.quat_matrix(pose[3:7])
    R_want = pocket(m.state(env, "leg"))[0]
    err = math.degrees(math.acos(float(np.clip((np.trace(R_sim.T @ R_want) - 1.0) / 2.0, -1.0, 1.0))))
    print(f"[knee] frame check: knee {math.degrees(m.state(env, 'leg').joint('knee')):.2f} deg; the simulator's housing differs from the pocket's frame by {err:.3f} deg", flush=True)


def _leg_report(env, label: str) -> None:
    st = knee_now(env)
    leg = m.state(env, "leg")
    print(f"[knee] {label}: face {st['face']:.2f} deg from up, origin {np.round(1000 * (st['o'] - [0, 0, TOP]), 1)} mm (on its back it rests at z {1000 * D_BACK:.1f}), "
          f"knee {math.degrees(leg.joint('knee')):.2f} deg, leg speed {1000 * st['leg_speed']:.1f} mm/s, spin {st['leg_spin']:.3f} rad/s", flush=True)


def demo_script(ctx):
    """Modes: full: both hands grip the leg's ends from above (the right the calf's end, the left the thigh's top end), and turn it together
    over its lower edge, toward the robot, until it lies on its back, then let go; the right hand pinches the plate across its neck, lifts it
    out of the post's socket, brings it over the pocket square and turned to it, lets go 1.5 mm above the face, and presses it home; then both
    hands clear. turn: the turn alone, then a report. gate: the arms wait (KNEE_GATE=press: the leg on its back and the plate seated, a push
    of KNEE_PUSH newtons at the boss for 2 s, then KNEE_PUSH2 at the plate's side, KNEE_PUSH_Y off the axis, for 2 s; the leg's motion after
    each). none: the arms wait."""
    l, r = ctx.left, ctx.right
    env = ctx.emb._env
    mode = ctx.mode
    plate = lambda: m.state(env, "plate")  # noqa: E731
    leg = lambda: m.state(env, "leg")  # noqa: E731
    if mode in ("none", "gate"):
        _check_frame(env)
        n = int(float(os.environ.get("KNEE_WAIT", "8")) * d.CONTROL_HZ)
        p1, p2, py = float(os.environ.get("KNEE_PUSH", "0")), float(os.environ.get("KNEE_PUSH2", "0")), float(os.environ.get("KNEE_PUSH_Y", "0.035"))
        o0 = None
        track = []
        for k in range(n):
            if k == 20:
                _leg_report(env, "settled")
                o0 = knee_now(env)["o"]
            if p1 and k == 20:
                m.push(env, "plate", (0.0, 0.0, -p1), at=plate().to_world(np.array([[PRESS_X, 0.0, 0.0]]))[0])
            if p1 and k == 60:
                m.push(env, "plate", (0.0, 0.0, 0.0))
                st = knee_now(env)
                print(f"[knee] GATE press {p1:.0f} N at the boss for 2 s: leg moved {1000 * np.linalg.norm(st['o'] - o0):.2f} mm, face {st['face']:.2f} deg from up", flush=True)
            if p2 and k == 80:
                m.push(env, "plate", (0.0, 0.0, -p2), at=plate().to_world(np.array([[PRESS_X, py, 0.0]]))[0])
            if p2 and k == 120:
                m.push(env, "plate", (0.0, 0.0, 0.0))
                st = knee_now(env)
                print(f"[knee] GATE press {p2:.0f} N at the plate's side, {1000 * py:.0f} mm off its axis, for 2 s: leg moved {1000 * np.linalg.norm(st['o'] - o0):.2f} mm, face {st['face']:.2f} deg from up", flush=True)
            if k >= 60:
                s = plate()
                track.append((s.pos.copy(), float(np.linalg.norm(s.lin_vel))))
            yield
        if track:
            p = np.array([t[0] for t in track])
            st = knee_now(env)
            print(f"[knee] GATE {GATE or mode} over {len(track) / d.CONTROL_HZ:.1f} s: plate drift {1000 * np.linalg.norm(p - p[0], axis=1).max():.4f} mm, top speed {1000 * max(t[1] for t in track):.3f} mm/s; "
                  f"end: leg moved {1000 * np.linalg.norm(st['o'] - o0):.2f} mm, face {st['face']:.3f} deg from up, rim {1000 * st['h'].min():.3f} to {1000 * st['h'].max():.3f} mm above the face, "
                  f"boss axis off {1000 * st['off']:.3f} mm, holes off {1000 * st['holes']:.3f} mm, tilt {st['tilt']:.3f} deg; score {TRIAL.score()}", flush=True)
        return
    if mode not in ("full", "turn"):
        raise ValueError(f"unknown mode {mode!r}")
    _check_frame(env)
    # 1. the turn. Each hand comes down over one end of the leg, its fingers either side of that end's thickness (the leg lies on its side, the
    # face toward +x, so the thickness runs along x), and closes. Then both swing together about the edge the leg rests on, toward the robot:
    # the leg tips over that edge onto its back, held all the way, so it is lowered, not dropped
    yield from r.wait(0.5)
    _leg_report(env, "at the start")
    R_pk, o0 = pocket(leg())
    gr, gl = o0 + R_pk @ GRIP_CALF, leg().to_world(GRIP_THIGH)[0]
    edge_p, edge_u = leg().to_world(EDGE_C)[0], leg().rotation() @ EDGE_U
    yield from ctx.together(r.move(gr + [0.0, 0.0, 0.10], grip=d.down(90), finger=d.FINGER_OPEN, t=2.5, label="over the calf's end"),
                            l.move(gl + [0.0, 0.0, 0.05], grip=d.down(-90), finger=d.FINGER_OPEN, t=2.5, label="over the thigh's end"))
    yield from ctx.together(r.move(gr, t=1.5, label="fingers either side of the calf's end"), l.move(gl, t=1.0, label="fingers either side of the thigh's end"))
    yield from ctx.together(r.close(0.059, force=45.0, t=0.6, label="grip the calf's end"), l.close(0.060, force=45.0, t=0.6, label="grip the thigh's end"))
    yield from ctx.together(r.arc(edge_p, edge_u, -TURN_DEG, t=TURN_T, label="turn the leg over its edge"), l.arc(edge_p, edge_u, -TURN_DEG, t=TURN_T, label="turn the leg over its edge"))
    yield from r.wait(0.5)
    _leg_report(env, f"turned {TURN_DEG:.0f} deg, held")
    yield from ctx.together(r.open(t=0.4, label="let go of the calf"), l.open(t=0.4, label="let go of the thigh"))
    yield from ctx.together(r.move(r.at() - 0.06 * r.approach_axis(), t=1.0, label="back off the calf"), l.move(l.at() - 0.06 * l.approach_axis(), t=1.0, label="back off the thigh"))
    yield from ctx.together(r.move(r.at() + [0.0, 0.0, 0.10], t=1.0, label="up"), l.move(np.array([0.15, 0.25, TOP + 0.20]), grip=d.down(-90), t=1.5, label="the left hand away"))
    yield from r.wait(0.8)
    _leg_report(env, "turned, let go")
    if mode == "turn":
        yield from r.wait(2.0)
        _leg_report(env, "turned, 2 s later")
        return
    g = d.down(90)  # fingers closing along world x
    # 2. the plate: pinch it across its neck and lift it out of the post's socket
    at = lambda p: plate().to_world(np.array([p]))[0]  # noqa: E731  (a plate-frame point in the world, as the plate is now)
    tips = lambda: _tips(env)  # noqa: E731
    pinch = at([PINCH_X, 0.0, -0.005 - 0.001])
    yield from r.move(pinch + [0.0, 0.0, 0.10], grip=g, finger=d.FINGER_OPEN, t=2.0, label="over the plate's neck")
    yield from r.servo(pinch + [0.0, 0.0, 0.05], tol=0.002, timeout=1.5, gain=0.3, read=tips, label="fingertips over the neck")
    yield from r.move(r.at() + [0.0, 0.0, -0.05], t=1.2, label="fingers down either side of the neck")
    yield from r.servo(pinch, tol=0.001, timeout=1.5, gain=0.3, read=tips, label="fingertips at the neck")
    yield from r.close(0.0394, force=50.0, t=0.6, label="pinch the neck across its flats")
    yield from r.move(r.at() + [0.0, 0.0, 0.045], t=1.2, label="lift it out of the socket")
    # 3. carry over the pocket, then turn the plate to the pocket and bring its axis over the bearing's, read from its pose and the leg's
    pk = lambda: pocket(leg())  # noqa: E731  (the pocket's frame now: rotation, origin)
    goal = lambda: pk()[1] + pk()[0][:, 2] * RELEASE_H  # noqa: E731
    off = r.at() - plate().pos  # the hand from the plate's origin, as held
    high = pk()[1][2] + 0.035  # carried with its underside 3 cm over the housing's face
    yield from r.move(np.r_[r.at()[:2], high + off[2]], t=1.2, label="up, clear of the leg")
    yield from r.move(np.r_[goal()[:2] + off[:2], high + off[2]], t=2.0, label="carry it over the pocket")
    yield from r.move(goal() + off + [0.0, 0.0, 0.020], t=1.0, label="down over the pocket")

    def square():
        """Turn the hand so the plate sits level with the face and turned to the pocket: the pinch lets it pitch a few degrees under its own weight."""
        R = pk()[0].T @ plate().rotation()
        pitch = math.degrees(math.asin(float(np.clip(R[2, 0], -1.0, 1.0))))  # its narrow end up
        roll = math.degrees(math.asin(float(np.clip(-R[2, 1], -1.0, 1.0))))
        yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
        if abs(pitch) > 0.05:
            yield from r.turn(pitch, axis=pk()[0][:, 1], t=0.6, label="level the plate")
        if abs(roll) > 0.05:
            yield from r.turn(roll, axis=pk()[0][:, 0], t=0.6, label="level the plate across")
        if abs(yaw) > 0.05:
            yield from r.turn(-yaw, axis=pk()[0][:, 2], t=0.6, label="turn the plate to the pocket")

    def over(h_goal, tol=0.0001, tries=6):
        """Bring the plate's axis onto the bearing's and its origin to h_goal above seated, read from its pose: short moves, each followed by a
        pause for the held plate to settle (a servo on the moving plate overshoots: the arm lags under the load)."""
        for _ in range(tries):
            yield from r.wait(0.5)
            R, o = pk()
            err = (o + R[:, 2] * h_goal) - plate().pos
            side = err - R[:, 2] * float(err @ R[:, 2])
            if np.linalg.norm(side) < tol and abs(float(err @ R[:, 2])) < 0.0005:
                return
            yield from r.move(r.at() + err, t=0.8, label="the plate's axis over the bearing's")

    yield from square()
    yield from over(RELEASE_H + 0.010, tol=0.0003)
    yield from square()
    yield from over(RELEASE_H)
    # 4. let go: the fingers cannot follow it into the pocket. The held plate trembles by a few tenths of a millimetre, so open the moment its
    # axis is within 0.12 mm of the bearing's
    for _round in range(4):
        hit = False
        for _ in range(int(2.0 * d.CONTROL_HZ)):
            R, o = pk()
            e = plate().pos - o
            if np.linalg.norm(e - R[:, 2] * float(e @ R[:, 2])) < RELEASE_TOL:
                hit = True
                break
            yield
        if hit:
            break
        yield from over(RELEASE_H, tol=0.0001, tries=3)  # still off the bearing's axis: correct once more before letting go
    st = knee_now(env)
    print(f"[knee] at the mouth: axis {np.round(1000 * st['axis'], 2)} mm from the bearing's, yaw {st['yaw']:.2f} tilt {st['tilt']:.2f} deg; face {st['face']:.2f} deg from up", flush=True)
    yield from r.open(t=0.2, label="let go above the pocket")
    yield from r.move(r.at() + [0.0, 0.0, 0.03], t=0.6, label="fingers up")
    yield from r.wait(0.5)
    st = knee_now(env)
    print(f"[knee] let go: rim {1000 * st['h'].min():.2f} to {1000 * st['h'].max():.2f} mm above the face, axis off {1000 * st['off']:.3f} mm", flush=True)
    # 5. press it home with the closed fingertips: where it stands highest if it has cocked, else between the boss and its centre of mass.
    # The arm presses through its soft joints: sent 20 mm below flush, it pushes about 10 N
    for _ in range(4):
        st = knee_now(env)
        if np.abs(st["h"]).max() <= FLUSH_H * 0.5:
            break
        hi = RIM[int(np.argmax(st["h"]))]
        spot = np.array([PRESS_X, 0.0, 0.0]) if st["h"].max() - st["h"].min() < 0.001 else np.clip(0.6 * hi, [-0.025, -0.025, 0.0], [0.090, 0.025, 0.0])
        p = plate().to_world(np.array([spot]))[0]
        yield from r.move(r.at(), finger=d.FINGER_CLOSED, t=0.4, label="close the fingers")
        yield from r.servo(p + [0.0, 0.0, 0.020], tol=0.0015, timeout=2.0, gain=0.3, read=tips, label="fingertips over the plate")
        yield from r.move(r.at() + [0.0, 0.0, -(tips()[2] - (pk()[1][2] - PRESS_DEPTH))], t=1.5, label="press it home")
        yield from r.wait(0.8)
        yield from r.move(r.at() + [0.0, 0.0, 0.03], t=0.6, label="off the plate")
        yield from r.wait(0.3)
        st = knee_now(env)
        print(f"[knee] pressed at {np.round(1000 * spot[:2], 0)} mm: rim {1000 * st['h'].min():.2f} to {1000 * st['h'].max():.2f} mm, axis off {1000 * st['off']:.3f} mm, holes {1000 * st['holes']:.2f} mm; face {st['face']:.2f} deg from up", flush=True)
    # 6. hands clear
    yield from r.move(np.array([0.22, -0.30, TOP + 0.20]), t=1.5, label="the right hand away")
    yield from r.wait(2.0)
