"""Hold a WidowX 250 arm up as its power is cut, and lay it limp in its sleep pose.

A Trossen WidowX 250 6DOF (the arm of BridgeData V2 and the leader arm of ALOHA) stands on its base on the bench in its working pose, its joints held by
their position hold. When both YAM grippers are closed on the arm, 1 s later its torque is cut: the six joints go limp under gravity. The grippers carry the
arm, set it down in Interbotix's sleep pose, and let go.

Model: the arm's six joints and their links from Trossen's interbotix_xsarm_descriptions (wx250s.urdf.xacro and meshes, BSD 3-Clause): joint axes, origins,
limits, link masses and centres of mass; the gripper assembly (gripper body, prop, bar, two fingers at 22 mm) is one rigid body on wrist_rotate (the
gripper's own joint is not modelled). Declared code: the torque cut (the joints' position hold is released) and the scorer. Unpowered, each joint is
back-driven through its Dynamixel gear train: Coulomb friction, viscous damping and the rotor's inertia at the joint (estimates; README). Of the arm's
links only the forearm, wrist and gripper collide with the upper arm, which the sleep pose rests on (SELF_CONTACT).

    python demokit.py task.py OUT --mode full
    python demokit.py task.py OUT --no-record --mode full,none,onehand,wrong,gate
"""

from __future__ import annotations

import math
import os
import pathlib
import struct
import sys

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import gatekit as gk  # noqa: E402
import mechkit as m  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = 'This robot arm is held up by its motors. Close both grippers on it: one second later its power is cut and it goes limp. Lay it down in its sleep pose (shoulder -1.80, elbow 1.55, wrist angle 0.8 rad, the other joints 0) without dropping it, then let go.'
SUCCESS_HOLD_S = 0.0
HOLD_STEPS = 40  # the final state counted by the scorer: 2 s at 20 Hz
V_DROP = 0.5  # m/s: any point of the arm faster than this after the cut is a drop (an unheld fall reaches about 2 m/s)
V_HIT = float(os.environ.get("V_HIT", "0.25"))  # m/s: a link striking the bench, pad or cradle faster than this is a hard landing
TOP = TABLE_TOP[SCENE]
HERE = pathlib.Path(__file__).parent
DT = 0.05

# -- the WidowX 250's kinematics (wx250s.urdf.xacro). Every link frame is aligned with the base's at joint zero.
NAMES = ["waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"]
ORIGIN = {"waist": (0, 0, .072), "shoulder": (0, 0, .11065), "elbow": (.04975, 0, .36065), "forearm_roll": (.22475, 0, .36065),
          "wrist_angle": (.29975, 0, .36065), "wrist_rotate": (.36475, 0, .36065)}
AXIS = {"waist": 2, "shoulder": 1, "elbow": 1, "forearm_roll": 0, "wrist_angle": 1, "wrist_rotate": 0}
LIM = {"waist": (-180, 180), "shoulder": (-108, 114), "elbow": (-123, 92), "forearm_roll": (-180, 180), "wrist_angle": (-100, 123), "wrist_rotate": (-180, 180)}
Q_WORK = np.array([0.0, 0.2, 0.2, 0.0, 0.9, 0.0])  # working pose: waist, shoulder, elbow, forearm roll, wrist angle, wrist rotate (rad)
Q_SLEEP = np.array([0.0, -1.80, 1.55, 0.0, 0.8, 0.0])  # interbotix_xsarm_control/config/wx250s.yaml sleep_positions
BASE = (float(os.environ.get("BASE_X", "0.28")), float(os.environ.get("BASE_Y", "0.10")))  # base centre on the bench; the arm points toward -y
YAW = -90.0
# An unpowered Dynamixel is back-driven through its gear train: Coulomb friction, viscous damping and the motor's rotor inertia times the ratio squared
# (Robotis: XM430-W350 353.5:1, XL430-W250 258.5:1). Trossen: XM430-W350 on waist, shoulder (two), elbow, forearm roll, wrist angle; XL430-W250 on wrist rotate.
SERVO = {"waist": "XM", "shoulder": "XM", "elbow": "XM", "forearm_roll": "XM", "wrist_angle": "XM", "wrist_rotate": "XL"}
N_SERVO = {n: (2 if n == "shoulder" else 1) for n in NAMES}
FRIC = {"XM": float(os.environ.get("WX_FRIC_XM", "0.15")), "XL": float(os.environ.get("WX_FRIC_XL", "0.06"))}  # static friction at the output, N m (estimate; range 0.05 to 0.3)
ROTOR = {"XM": float(os.environ.get("WX_ARM_XM", "0.006")), "XL": float(os.environ.get("WX_ARM_XL", "0.003"))}  # reflected rotor inertia, kg m^2 (estimate: about 0.5 g cm^2 x 353.5^2)
DAMPING = float(os.environ.get("WX_DAMPING", "0.05"))  # viscous, N m s/rad (estimate; range 0.02 to 0.1)
FRICTION_J = {n: FRIC[SERVO[n]] * N_SERVO[n] for n in NAMES}
ARMATURE_J = {n: ROTOR[SERVO[n]] * N_SERVO[n] for n in NAMES}
DAMP_J = {n: DAMPING * N_SERVO[n] for n in NAMES}
HOLD_K, HOLD_D = float(os.environ.get("WX_HOLD_K", "600")), float(os.environ.get("WX_HOLD_D", "12"))  # the position hold of a powered joint
TAU_MAX = {"waist": 10, "shoulder": 20, "elbow": 15, "forearm_roll": 2, "wrist_angle": 5, "wrist_rotate": 1}  # URDF effort limits (N m)
PAD_C = (float(os.environ.get("PAD_X", "0.315")), BASE[1] - 0.35)  # centre; set from the unheld fall (gate)
PAD_S = (float(os.environ.get("PAD_SX", "0.47")), float(os.environ.get("PAD_SY", "0.38")), 0.02)  # x, y, thickness (m)


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


_ROT = [_rx, _ry, _rz]
RS = _rz(math.radians(YAW))  # base frame to station frame
BASE_POS = np.array([BASE[0], BASE[1], TOP])


def to_station(p):
    return np.asarray(p) @ RS.T + BASE_POS


def fk(q):
    """Each joint's frame (4x4, base frame) for joint angles q."""
    T = np.eye(4)
    prev = np.zeros(3)
    out = {}
    for n, a in zip(NAMES, q):
        o = np.array(ORIGIN[n])
        A = np.eye(4)
        A[:3, 3] = o - prev
        B = np.eye(4)
        B[:3, :3] = _ROT[AXIS[n]](a)
        T = T @ A @ B
        out[n] = T.copy()
        prev = o
    return out


def _stl(name):
    raw = (HERE / name).read_bytes()
    n = struct.unpack("<I", raw[80:84])[0]
    return np.frombuffer(raw[84:84 + n * 50], np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))["v"].astype(float) * 0.001  # mm to m


def _mesh(tri, R, t, origin):
    """A triangle soup (link frame, per the URDF visual origin R, t) as a Mesh in the base frame at joint zero (the link frame at `origin`)."""
    v = tri.reshape(-1, 3) @ R.T + np.asarray(t) + np.asarray(origin)
    key = np.round(v * 1e6).astype(np.int64)
    uniq, inv = np.unique(key, axis=0, return_inverse=True)
    return m.Mesh(uniq / 1e6, inv.reshape(-1, 3))


P = math.pi
FOPEN = 0.022  # the fingers' half opening (m); both fixed (the gripper's own joint is not modelled)
GRIP_O = (.36475, 0, .36065)
# (file, rotation, translation in the link frame), per joint
MESH = {
    "waist": [("shoulder.stl", _rz(P / 2), (0, 0, -.003))],
    "shoulder": [("upper_arm.stl", _rz(P / 2), (0, 0, 0))],
    "elbow": [("upper_forearm.stl", np.eye(3), (0, 0, 0))],
    "forearm_roll": [("lower_forearm.stl", _rx(P), (0, 0, 0))],
    "wrist_angle": [("wrist.stl", _rz(P / 2), (0, 0, 0))],
    "wrist_rotate": [("gripper.stl", _rz(P / 2), (-.02, 0, 0)), ("gripper_prop.stl", _rz(P / 2), (.0485 - .0685, 0, 0)),
                     ("gripper_bar.stl", _rz(P / 2), (.043 - .063, 0, 0)),
                     ("gripper_finger.stl", _rz(P), (.066, FOPEN + .005, 0)), ("gripper_finger.stl", _ry(P), (.066, -FOPEN - .005, 0))],
}
# link masses (kg), centres of mass (link frame, m) and principal inertias (kg m^2, along the link's x, y, z) from the URDF
# (the visual origin turned by pi/2 about z swaps x and y)
LINK = {
    "waist": (.480879, (0.0000223, 0.0000415, 0.0066287 - 0.0), (5.557e-4, 3.79e-4, 5.889e-4)),
    "shoulder": (.430811, (0.0171605, 0.0, 0.191323), (3.587e-3, 3.463e-3, 4.566e-4)),
    "elbow": (.234589, (0.107963, 0.000116, 0.0), (3.971e-5, 8.878e-4, 8.88e-4)),
    "forearm_roll": (.220991, (0.0374395, 0.0052225, 0.0), (6.369e-5, 1.677e-4, 1.834e-4)),
    "wrist_angle": (.084957, (0.04236, -0.0000107, 0.010577), (2.822e-5, 3.082e-5, 3.152e-5)),
}
# the gripper assembly, merged: gripper body, prop, bar, two fingers (each: mass, centre in the wrist_rotate link frame, own inertia)
_G = [(.072885, (.021631, 0, .01141), (1.674e-5, 2.537e-5, 1.836e-5)), (.00434, (.0485 + .000846, 0, 0), (5.7e-7, 5.9e-7, 1.1e-6)),
      (.034199, (.043 + .009687, 0, .004962), (7.4e-6, 2.84e-5, 2.86e-5)), (.016246, (.066 + .013816, FOPEN, 0), (1.5e-6, 4.7e-6, 3.7e-6)),
      (.016246, (.066 + .013816, -FOPEN, 0), (1.5e-6, 4.7e-6, 3.7e-6))]
_gm = sum(g[0] for g in _G)
_gc = tuple(sum(g[0] * np.array(g[1]) for g in _G) / _gm)
_gi = np.sum([np.array(g[2]) + g[0] * (np.sum((np.array(g[1]) - _gc) ** 2) - (np.array(g[1]) - _gc) ** 2) for g in _G], axis=0)
LINK["wrist_rotate"] = (_gm, _gc, tuple(_gi))
MASS_MOVING = sum(v[0] for v in LINK.values())
LOOKS = {"black": m.Look(color=m.rgb(0.07, 0.07, 0.08), roughness=0.55), "foam": m.Look(color=m.rgb(0.20, 0.42, 0.62), roughness=0.9),
         "grey": m.Look(color=m.rgb(0.35, 0.36, 0.38), roughness=0.5)}


def _wx():
    base_tri = _stl("base.stl")
    fparts = [("base", _mesh(base_tri, _rz(P / 2), (0, 0, 0), (0, 0, 0)), "black")]
    fcols = [m.col_hull("base_c", _mesh(base_tri, _rz(P / 2), (0, 0, 0), (0, 0, 0)))]
    links = []
    for k, n in enumerate(NAMES):
        parts, cols = [], []
        for i, (f, R, t) in enumerate(MESH[n]):
            mesh = _mesh(_stl(f), R, t, ORIGIN[n])
            parts.append((f"{n}_{i}", mesh, "black" if i == 0 else "grey"))
            cols.append(m.col_hull(f"{n}_{i}_c", mesh))
        mass, com, inertia = LINK[n]
        com = tuple(float(c + o) for c, o in zip(com, ORIGIN[n]))
        ax = "XYZ"[AXIS[n]]
        links.append(m.hinge(n, ax, ORIGIN[n], LIM[n][0], LIM[n][1], parts, cols, parent=None if k == 0 else NAMES[k - 1], mass=float(mass), gravity=True, com=com,
                             spring=HOLD_K, damping=HOLD_D, friction=FRICTION_J[n], effort=TAU_MAX[n], armature=ARMATURE_J[n], inertia=tuple(float(v) for v in inertia)))
    start = [(n, float(a)) for n, a in zip(NAMES, Q_WORK)]
    return m.mechanism("wx250s", tuple(BASE_POS), links, LOOKS, frame_parts=fparts, frame_colliders=fcols, yaw_deg=YAW, friction=(0.8, 0.7), solver_iters=64, start=start)


# Self-contact: in the sleep pose the real arm's forearm rests on its upper arm (the hulls meet at elbow 1.53 rad, the sleep angle is 1.55). Only those pairs
# collide: the forearm, wrist and gripper with the upper arm, the base frame and the waist link; every other pair of the arm's bodies is filtered.
SELF_CONTACT = os.environ.get("WX_SELF", "1") == "1"
_TOUCH = {(a, b) for a in ("shoulder", "frame", "waist") for b in ("forearm_roll", "wrist_angle", "wrist_rotate")}


def _self_contact(txt):
    import re

    root = re.search(r"rel physics:body1 = <(/[^>]*)/waist>", txt).group(1)
    bodies = ["frame"] + NAMES
    txt = txt.replace("bool physxArticulation:enabledSelfCollisions = 0", "bool physxArticulation:enabledSelfCollisions = 1")
    for i, a in enumerate(bodies):
        others = [b for b in bodies[i + 1:] if (a, b) not in _TOUCH and (b, a) not in _TOUCH]
        if not others:
            continue
        pat = re.compile(r'(def Xform "' + a + r'" \(\n\s*prepend apiSchemas = \[)([^\]]*)(\]\n\s*\)\n(\s*)\{)')
        mt = pat.search(txt)
        rel = ", ".join(f"<{root}/{b}>" for b in others)
        txt = txt[:mt.start()] + mt.group(1) + mt.group(2) + ', "PhysicsFilteredPairsAPI"' + mt.group(3) + f"\n{mt.group(4)}    rel physics:filteredPairs = [{rel}]" + txt[mt.end():]
    return txt


def _with_start_state(part):
    """The USD with each joint's start angle authored as its joint state, so the arm is in its working pose from the first frame."""
    import dataclasses
    build0 = part.build

    def build():
        txt = build0()
        for n, a in zip(NAMES, Q_WORK):
            deg = math.degrees(a)
            head = f'def PhysicsRevoluteJoint "{n}_joint" (\n        prepend apiSchemas = ["PhysicsDriveAPI:angular"]'
            k = txt.index(head)
            txt = txt[:k] + head.replace('["PhysicsDriveAPI:angular"]', '["PhysicsDriveAPI:angular", "PhysicsJointStateAPI:angular"]') + txt[k + len(head):]
            anchor = 'uniform token drive:angular:physics:type = "force"'
            j = txt.index(anchor, k)
            txt = txt[:j] + f"float state:angular:physics:position = {deg:.6f}\n        float state:angular:physics:velocity = 0\n        " + txt[j:]
        if SELF_CONTACT:
            txt = _self_contact(txt)
        return txt

    return dataclasses.replace(part, build=build)


WX = _with_start_state(_wx())
PAD = m.fixed("pad", (PAD_C[0], PAD_C[1], TOP), [("pad", m.box((PAD_S[0], PAD_S[1], PAD_S[2]), (0, 0, PAD_S[2] / 2)), "foam")],
              [m.col_box("pad_c", (PAD_S[0], PAD_S[1], PAD_S[2]), (0, 0, PAD_S[2] / 2))], LOOKS, friction=(1.0, 0.9))


def floor_under(pts):
    """The height of what a point would land on: the bench, the pad, or a foam block."""
    f = np.full(len(pts), TOP)
    f = np.where((np.abs(pts[:, 0] - PAD_C[0]) < PAD_S[0] / 2) & (np.abs(pts[:, 1] - PAD_C[1]) < PAD_S[1] / 2), TOP + PAD_S[2], f)
    for cx, cy, sx, sy, h in BLOCKS.values():
        f = np.where((np.abs(pts[:, 0] - cx) < sx / 2) & (np.abs(pts[:, 1] - cy) < sy / 2), TOP + h, f)
    return f


# -- reading the arm -------------------------------------------------------------------------

def q_now(st) -> np.ndarray:
    return np.array([float(st.joint(n)) for n in NAMES])


def _tri_samples(tri, R, t):
    v = tri.reshape(-1, 3, 3) @ R.T + np.asarray(t)
    pts = [v.reshape(-1, 3), v.mean(axis=1), (v[:, 0] + v[:, 1]) / 2, (v[:, 1] + v[:, 2]) / 2, (v[:, 2] + v[:, 0]) / 2]
    return np.unique(np.round(np.concatenate(pts), 4), axis=0)


SURF = {}  # per joint link: the surface's sample points in the link frame (the frame at the joint's origin)
for _n in NAMES:
    SURF[_n] = np.concatenate([_tri_samples(_stl(f), R, t) for f, R, t in MESH[_n]])


def arm_surface(q, from_link: int = 1):
    """The arm's surface sample points (station frame) from the shoulder link out (the waist link, its base cap, is left out)."""
    T = fk(q)
    out = []
    for n in NAMES[from_link:]:
        out.append(SURF[n] @ T[n][:3, :3].T + T[n][:3, 3])
    return to_station(np.concatenate(out))


BASE_PTS = to_station(_tri_samples(_stl("base.stl"), _rz(P / 2), (0, 0, 0)))  # the fixed base's surface; the forearm, wrist and gripper must not pass through it
_BASE_KD = cKDTree(BASE_PTS)


Q_REST = Q_SLEEP.copy()
Q_REST[2] = float(os.environ.get("REST_ELB", math.radians(LIM["elbow"][1])))
Q_REST[4] = float(os.environ.get("REST_WR", Q_SLEEP[4]))  # how the limp arm lies in its sleep pose: gravity holds the forearm on the elbow's hard stop (92 deg, 3 deg past sleep)


Q_HELD = Q_SLEEP.copy()
Q_HELD[2], Q_HELD[4] = float(os.environ.get("GR_ELB", "1.52")), float(os.environ.get("GR_WR", "0.85"))  # how the two hands leave the arm at the end of the fold: the left's fingers on the upper arm keep the forearm off its stop


def _cut(q, cx, cy, sx, sy, link=None):
    """A foam block's height, cut so that the arm at joint angles q just rests on it: the lowest point of the arm (or of one link) over the block."""
    if link is None:
        p = arm_surface(q)
    else:
        T = fk(q)
        p = to_station(SURF[link] @ T[link][:3, :3].T + T[link][:3, 3])
    over = (np.abs(p[:, 0] - cx) < sx / 2) & (np.abs(p[:, 1] - cy) < sy / 2)
    return round(float(p[over, 2].min() - TOP), 3)


# the cradle, foam the limp arm rests on, clear of the base's moving waist link and of the arm at the start (each block's clearance is printed at load):
# rest_a under the upper arm's rear end, cut to the sleep pose; rest_b, in 1 cm strips across y in front of the base, each cut to the gripper's underside as the
# hands leave it, so the gripper lies in a fitted cradle and cannot pitch down off its wrist joint once both hands let go (x, y centre; x, y size; height; m)
GR_Y0, GR_Y1, GR_W = -0.045, float(os.environ.get("GR_Y1", "0.025")), 0.01  # rest_b spans y GR_Y0 to GR_Y1 (the base's front is at 0.023, its waist link at 0.049)
BLOCKS = {"rest_a": (BASE[0], BASE[1] + 0.246, 0.12, 0.07)}
BLOCKS["rest_a"] = (*BLOCKS["rest_a"], _cut(Q_REST, *BLOCKS["rest_a"]))
for _i in range(round((GR_Y1 - GR_Y0) / GR_W)):
    _cy = GR_Y0 + GR_W * (_i + 0.5)
    BLOCKS[f"rest_b{_i}"] = (BASE[0], _cy, 0.12, GR_W, _cut(Q_HELD, BASE[0], _cy, 0.12, GR_W, link="wrist_rotate"))


def _clearance(q):
    """The least gap (m) between the foam blocks and the arm's surface at joint angles q: below zero is a collider overlap."""
    p = arm_surface(q, from_link=0)
    out = {}
    for k, (cx, cy, sx, sy, h) in BLOCKS.items():
        dx = np.maximum(np.abs(p[:, 0] - cx) - sx / 2, 0)
        dy = np.maximum(np.abs(p[:, 1] - cy) - sy / 2, 0)
        dz = np.maximum(p[:, 2] - (TOP + h), 0)
        out[k] = round(float(np.sqrt(dx * dx + dy * dy + dz * dz).min()), 3)
    return out


RESTS = [m.fixed(k, (v[0], v[1], TOP), [(k, m.box((v[2], v[3], v[4]), (0, 0, v[4] / 2)), "foam")], [m.col_box(k + "_c", (v[2], v[3], v[4]), (0, 0, v[4] / 2))], LOOKS, friction=(1.0, 0.9))
         for k, v in BLOCKS.items()]
OBJECTS = [WX, PAD] + RESTS
print(f"[wx] foam blocks: {BLOCKS}; clearance to the arm at the start {_clearance(Q_WORK)}", flush=True)


def link_points(q, name, local):
    T = fk(q)[name]
    return to_station(T[:3, :3] @ np.asarray(local) + T[:3, 3])


def link_axes(q, name):
    """A link's frame axes in the station frame (columns)."""
    return RS @ fk(q)[name][:3, :3]


def sleep_err_deg(q):
    return np.degrees(np.abs(q - Q_SLEEP))


def _gear(asset, j, name):
    """The gear train's static and dynamic friction (N m) and the rotor's reflected inertia, powered or not."""
    f = FRICTION_J[name]
    try:
        asset.write_joint_friction_coefficient_to_sim_index(joint_friction_coeff=m._tensor(asset, f), joint_dynamic_friction_coeff=m._tensor(asset, 0.7 * f), joint_ids=[j])
        asset.write_joint_armature_to_sim_index(armature=m._tensor(asset, ARMATURE_J[name]), joint_ids=[j])
    except (AttributeError, TypeError) as e:
        print(f"[wx] gear friction not written: {e}", flush=True)


def _powered_gains(asset, j, name):
    _gear(asset, j, name)
    asset.write_joint_effort_limit_to_sim(m._tensor(asset, TAU_MAX[name]), joint_ids=[j])
    asset.write_joint_stiffness_to_sim(m._tensor(asset, HOLD_K), joint_ids=[j])
    asset.write_joint_damping_to_sim(m._tensor(asset, HOLD_D), joint_ids=[j])
    asset.set_joint_position_target(m._tensor(asset, float(Q_WORK[NAMES.index(name)])), joint_ids=[j])


_unlock0 = m.unlock


def _unlock(env, part, link):
    """At a reset mechkit gives touched joints their own gains back: here that is the powered hold on the working pose."""
    if part != "wx250s":
        return _unlock0(env, part, link)
    asset, j = m._joint(env, part, link)
    _powered_gains(asset, j, link)


m.unlock = _unlock
m._TOUCHED.update((("wx250s", n) for n in NAMES))  # the first reset gives the joints the powered hold too, with their target on the working pose


_LAT = np.array([(x, y, z) for x in (-0.04, -0.02, 0.0, 0.02, 0.04) for y in (-0.03, 0.0, 0.03) for z in np.arange(-0.12, 0.1471, 0.02)])
_SURF_LINK = np.concatenate([[n] * len(SURF[n]) for n in NAMES])  # the link of each point of arm_surface(q, from_link=0)


def hand_body(env) -> np.ndarray:
    """Points over each whole YAM gripper (station frame): a lattice 2 cm apart, x +-4 cm, y +-3 cm, from 12 cm behind the gripper frame (the wrist link) to the
    fingertips 14.7 cm ahead of it (palm, housing, pads, fingers), both hands."""
    base = env.unwrapped
    origin = m._np(base.scene.env_origins[0])
    lat = _LAT
    out = []
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        i = robot.find_bodies("gripper")[0][0]
        pose = m._np(robot.data.body_link_pose_w)[0, i]
        out.append(pose[:3] - origin + lat @ m.quat_matrix(pose[3:7]).T)
    return np.concatenate(out)


# -- the scorer ---------------------------------------------------------------------------------

class Trial:
    def __init__(self) -> None:
        self.reset()

    def reset(self, env=None) -> None:
        m._TOUCHED.update((("wx250s", n) for n in NAMES))  # the next reset restores the powered hold
        if env is not None:  # the working pose, powered, as the scene's start (mechkit does not write a mechanism's joints at reset)
            import torch
            asset = env.unwrapped.scene["wx250s"]
            ids = [m._joint(env, "wx250s", n)[1] for n in NAMES]
            pos = torch.tensor([list(map(float, Q_WORK))], device=asset.device)
            asset.write_joint_state_to_sim(pos, torch.zeros_like(pos), joint_ids=ids)
            for n in NAMES:
                _powered_gains(asset, m._joint(env, "wx250s", n)[1], n)
        self.t = 0
        self.moved = False
        self.held_steps = 0
        self.cut = False
        self.cut_t = None
        self.powered = True
        self.last_q = None
        self.last_pts = None
        self.latch = False  # a drop: set after the cut
        self.why = ""
        self.peak = 0.0
        self.peak1 = 0.0  # peak joint speed in the first second
        self.clear_steps = 0
        self.dmin, self.near = 9.0, ""
        self.base_gap = 9.0  # least distance (m) from the forearm, wrist and gripper surface to the base surface so far
        self.full = False
        self.score = 0.0
        self.now: dict = {}
        self.held_at_cut = False
        self.set_by_hands = False  # the arm within 10 degrees of the sleep pose at the last step both grippers held it

    def _power(self, env, on: bool) -> None:
        """Power on: the joint holds its working-pose angle (stiff, damped). Power off: no stiffness, only the joint's own damping."""
        if self.powered is on:
            return
        self.powered = on
        for n in NAMES:
            asset, j = m._joint(env, "wx250s", n)
            m._TOUCHED.add(("wx250s", n))
            if on:
                _powered_gains(asset, j, n)
            else:
                asset.write_joint_stiffness_to_sim(m._tensor(asset, 0.0), joint_ids=[j])
                asset.write_joint_damping_to_sim(m._tensor(asset, DAMP_J[n]), joint_ids=[j])

    def step(self, env) -> None:
        self.t += 1
        st = m.state(env, "wx250s")
        q = q_now(st)
        tips = m.fingertips(env)
        pts = arm_surface(q)
        # held: each hand has both fingers' points within 2 cm of the arm's surface and its jaws closed to 7 cm or less
        held = []
        for side, rows in (("left", slice(0, 12)), ("right", slice(12, 24))):
            tt = tips[rows]
            f1, f2 = tt[:6], tt[6:]
            d1 = float(np.min(np.linalg.norm(f1[:, None] - pts[None], axis=2)))
            d2 = float(np.min(np.linalg.norm(f2[:, None] - pts[None], axis=2)))
            gap = float(np.linalg.norm(f1.mean(axis=0) - f2.mean(axis=0)))
            held.append(d1 < 0.025 and d2 < 0.025 and gap < 0.08)
        both = all(held)
        if not self.cut and not self.moved and float(np.abs(q - Q_WORK).max()) > 0.02:
            self.moved = True
            print(f"[wx] first push at t {self.t * DT:.2f} s: q {np.round(q, 3)}; left tips {np.round(tips[:12].mean(axis=0) - BASE_POS, 3)} right tips {np.round(tips[12:].mean(axis=0) - BASE_POS, 3)}; "
                  f"dist left {np.min(np.linalg.norm(tips[:12][:, None] - pts[None], axis=2)):.3f} right {np.min(np.linalg.norm(tips[12:][:, None] - pts[None], axis=2)):.3f}", flush=True)
        if not self.cut:
            self.held_steps = self.held_steps + 1 if both else 0
            if self.held_steps >= 20 or self.t > int(os.environ.get("WX_AUTOCUT", "600")):  # 1 s after both grippers close; 30 s if they never do
                self.cut = True
                self.cut_t = self.t
                self.held_at_cut = both
                self._power(env, False)
                print(f"[wx] torque cut at t {self.t * DT:.1f} s; held by both: {both}; q {np.round(q, 2)}", flush=True)
        # the arm's speed, from its joint readings: every surface point's speed through the URDF's kinematics (a fall moves the links, not a roll joint's few degrees)
        vel = np.zeros(len(pts)) if self.last_pts is None else np.linalg.norm(pts - self.last_pts, axis=1) / DT
        self.last_q, self.last_pts = q.copy(), pts.copy()
        err = sleep_err_deg(q)
        spd = float(vel.max())
        floor = floor_under(pts)
        gap = pts[:, 2] - floor
        resting = bool(gap.min() < 0.006)  # a link on the foam (pad or cradle blocks)
        if self.cut:
            k = self.t - self.cut_t
            self.peak = max(self.peak, spd)
            if k <= 20:
                self.peak1 = max(self.peak1, spd)
            # drop latch: any point of the arm faster than V_DROP, or striking the bench, pad or cradle faster than V_HIT
            hit = gap < 0.006
            v_hit = float(vel[hit].max()) if hit.any() else 0.0
            if spd > V_DROP and not self.latch:
                self.latch, self.why = True, f"a link at {spd:.2f} m/s"
                print(f"[wx] latch at t {self.t * DT:.2f}: a link at {spd:.2f} m/s; q {np.round(q, 2)}; held {held}", flush=True)
            if v_hit > V_HIT and not self.latch:
                self.latch, self.why = True, f"a link struck the foam or bench at {v_hit:.2f} m/s"
                print(f"[wx] latch at t {self.t * DT:.2f}: {self.why}; q {np.round(q, 2)}; held {held}", flush=True)
        # three tiers, one per substep; a drop after the cut voids the run (0)
        sc = 0.0
        if self.cut and both:  # the hands' own lay, read while both still hold it (gravity sagging a joint home after a hand opens does not count)
            self.set_by_hands = bool(err.max() < 10.0)
        if self.cut and self.held_at_cut:  # caught: both hands closed on the arm when the power went
            sc = 1 / 3
            if self.set_by_hands and err.max() < 10.0 and spd < 0.01 and resting:  # laid: within 10 degrees of the sleep pose as the hands let go, and now, the arm still on its foam
                sc = 2 / 3
        # let go: both whole grippers (fingers, pads, palm, housing) at least 5 cm from the arm's surface, the arm still laid, for 2 s
        if sc == 2 / 3:  # the hands-clear distance matters only once the arm is laid; otherwise keep the last reading
            body = hand_body(env)
            dist, idx = cKDTree(arm_surface(q, from_link=0)).query(body)  # each hand point's nearest arm surface point
            bi = int(dist.argmin())
            si = int(idx[bi])
            self.dmin = float(dist[bi])
            self.near = f"{('left', 'right')[bi // len(_LAT)]} lattice {np.round(_LAT[bi % len(_LAT)], 2)} to {_SURF_LINK[si]}"
        dmin, near = self.dmin, self.near
        clear = dmin >= 0.05
        if self.t % 5 == 0:  # diagnostic only, so every fifth step
            self.base_gap = min(self.base_gap, float(_BASE_KD.query(arm_surface(q, from_link=3))[0].min()))
        self.clear_steps = self.clear_steps + 1 if (sc == 2 / 3 and clear) else 0
        self.full = self.clear_steps >= HOLD_STEPS
        if self.latch:
            sc = 0.0
        self.score = 1.0 if (self.full and not self.latch) else sc
        self.now = {"q": np.round(q, 2), "err_deg": np.round(err, 0), "v": round(spd, 3), "peak": round(self.peak, 2), "held": held, "cut": self.cut, "latch": self.why,
                    "peak1": round(self.peak1, 2), "rest": resting, "gap_mm": round(1000 * float(gap.min()), 1), "dmin_cm": round(100 * dmin, 1), "near": near, "clear_steps": self.clear_steps, "set": self.set_by_hands, "base_gap_mm": round(1000 * self.base_gap, 1)}


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.full


def score(objects) -> float:
    return TRIAL.score


def _report(ctx, label=""):
    print(f"[wx] t {ctx.t:5.1f} {label}{TRIAL.now}; score {TRIAL.score:.2f}", flush=True)


# -- the demo -------------------------------------------------------------------------------------


WG = float(os.environ.get("WX_WG", "0.6"))  # the waist's drift: both hands push sideways against it
PIN_L = np.array([0.0, 0.0, float(os.environ.get("PIN_L", "0.06"))])  # the left pinch: on the upper arm's 28 mm tube, 6 cm up from the shoulder axis (shoulder link frame)
PIN_R = np.array([float(os.environ.get("PIN_RX", "0.02")), 0.0, 0.0075])  # the right pinch: across the gripper body (wrist_rotate link frame)


def pinch_points(q):
    """Where each hand pinches, station frame: the left on the upper arm's tube near the elbow, the right on the gripper body."""
    return link_points(q, "shoulder", PIN_L), link_points(q, "wrist_rotate", PIN_R)


def _side_grip(link, axis):
    """A level grip pointing +x (along the arm's pitch axes, from the YAMs' side), rolled so that its jaws close across `axis` of `link` (the link's
    long axis, in the arm's plane) at the working pose."""
    import demokit as d

    g = d.level(0.0).rot
    dvec = link_axes(Q_WORK, link)[:, axis]
    want = np.array([0.0, -dvec[2], dvec[1]])
    want /= np.linalg.norm(want)
    jaw = g[:, 0] if abs(g[:, 0] @ np.array([0, 1.0, 0])) > 0.9 else g[:, 1]  # the jaws close along world y for level(0)
    psi = math.atan2(float(np.cross(jaw, want) @ np.array([1.0, 0, 0])), float(jaw @ want))
    if abs(psi) > math.pi / 2:  # the jaws are symmetric: the smaller roll
        psi -= math.copysign(math.pi, psi)
    return _rx(psi + math.radians(float(os.environ.get("ROLL_" + link.upper(), "0")))) @ g


_REL = {}


def hand_grip(q, link):
    """The hand's grip on `link` at joint angles q: the grip it took at the working pose, turning with the link (a hand closed on a link keeps its pose on it)."""
    import demokit as d

    if link not in _REL:
        _REL[link] = link_axes(Q_WORK, link).T @ _side_grip(link, 2 if link == "shoulder" else 0)
    return d._as_grip(link_axes(q, link) @ _REL[link])


def demo_script(ctx):
    import demokit as d
    from yam_arena.constants import FINGER_OPEN
    from demokit import READY

    env = ctx.emb._env
    l, r = ctx.left, ctx.right
    for _ in range(10):
        yield
    _report(ctx, "start: ")
    mode = ctx.mode
    if mode == "none":  # nothing held: the arm is dropped at WX_AUTOCUT
        for _ in range(int(os.environ.get("WX_AUTOCUT", "600")) + 160):
            yield
            if _ % 100 == 99:
                _report(ctx, "idle: ")
        return
    read = lambda: q_now(m.state(env, "wx250s"))  # the arm's own joint readings
    q = Q_WORK.copy()
    pl, pr = pinch_points(q)
    gl, gr = hand_grip(q, "shoulder"), hand_grip(q, "wrist_rotate")
    yield from ctx.together(l.move(pl - 0.10 * gl.rot[:, 2], grip=gl, finger=FINGER_OPEN, t=2.5, label="left beside the upper arm"),
                            r.move(pr - 0.10 * gr.rot[:, 2], grip=gr, finger=FINGER_OPEN, t=2.5, label="right beside the gripper"))

    def tipoff(label):
        tt = m.fingertips(env)
        a, b = pinch_points(read())
        print(f"[wx] {label}: left tips - pinch {np.round(tt[:12].mean(axis=0) - a, 3)} gap {gk.jaw_gap(tt, 'left'):.3f}; "
              f"right {np.round(tt[12:].mean(axis=0) - b, 3)} gap {gk.jaw_gap(tt, 'right'):.3f}; q {np.round(read(), 2)}", flush=True)

    tipoff("beside")
    yield from ctx.together(l.move(pl, t=1.5, label="left onto the upper arm"), r.move(pr, t=1.5, label="right onto the gripper body"))
    tipoff("onto")
    yield from ctx.together(l.close(0.028, 30.0, t=0.8, label="left closes on the upper arm"), r.close(0.055, 30.0, t=0.8, label="right closes on the gripper body"))
    for _ in range(100):  # until the power is cut (1 s after both grippers hold)
        yield
        if TRIAL.cut:
            break
    for _ in range(10):
        yield
    _report(ctx, "gripped: ")
    if mode == "gate":  # unheld-fall data with both hands on: they let go at the cut
        yield from ctx.together(l.open(t=0.3), r.open(t=0.3))
        for _ in range(200):
            yield
        _report(ctx, "gate end: ")
        return
    if mode == "onehand":  # a lazy route: the left hand lets go at the cut, and the right lays the arm alone by its gripper
        yield from l.open(t=0.3)
        yield from l.move(l.at() - 0.10 * l.approach_axis(), t=1.0)
    n = int(os.environ.get("WX_SEG", "30"))
    q0 = read()
    qt = Q_SLEEP + np.array([0.0, -0.05, float(os.environ.get("WX_ELB", "0.0")), 0.0, 0.0, 0.0])  # aimed past the shoulder's stop: the arm rests on it
    if mode == "wrong":  # a lazy route: laid with the wrist angle 30 degrees short of its sleep angle
        qt = qt + np.array([0.0, 0.0, 0.0, 0.0, -0.52, 0.0])
    qt[4] += float(os.environ.get("WX_WR", "0"))  # aim the wrist angle past its sleep angle, so the gripper is pressed onto its foam before the right hand lets go
    qt[[0, 3, 5]] = 0.0
    offl = offr = np.zeros(3)
    for k in range(1, n + 1):  # closed loop: each step, read the arm's joints and push each hand's goal to make up the gap from the planned pose
        s = k / n
        s = s * s * (3 - 2 * s)
        qk = q0 + (qt - q0) * s
        a, b = pinch_points(qk)
        qm = read()
        ao, bo = pinch_points(qm)
        if k > 1:
            offl = np.clip(offl + 0.5 * (pinch_points(qprev)[0] - ao), -0.03, 0.03)
            offr = np.clip(offr + 0.5 * (pinch_points(qprev)[1] - bo), -0.03, 0.03)
        qprev = qk
        side = np.array([WG * float(qm[0]), 0.0, 0.0])
        ga, gb = a + offl - side * (BASE[1] - a[1]), b + offr - side * (BASE[1] - b[1])
        if mode == "onehand":
            yield from r.move(gb, grip=hand_grip(qk, "wrist_rotate"), t=0.3, label="lay the arm down: right only")
        else:
            yield from ctx.together(l.move(ga, grip=hand_grip(qk, "shoulder"), t=0.3, label="lay the arm down: left"),
                                    r.move(gb, grip=hand_grip(qk, "wrist_rotate"), t=0.3, label="lay the arm down: right"))
        if k % 6 == 0:
            _report(ctx, f"lay {k}/{n}: ")
    for _ in range(10):
        yield
    _report(ctx, "folded: ")
    from yam_arena import ik
    for _ in range(20):
        yield
    _report(ctx, "laid: ")
    order = ((r, "right"), (l, "left")) if os.environ.get("REL_ORDER", "rl") == "rl" else ((l, "left"), (r, "right"))
    for arm, name in order:  # let go one hand at a time, where it is (its joints kept, no new IK solve), and draw it straight back out along its fingers
        arm.grip = d._as_grip(ik.tcp_pose(arm.q, arm.base)[1])
        w = float(os.environ.get("LOPEN", "0.04")) if name == "left" else 0.0  # the left opens only this wide (m) before it slides out, so its upper finger does not lift the forearm
        if w > 0:
            yield from arm.close(w, 5.0, t=1.5, label=f"{name} lets go")
        else:
            yield from arm.open(t=1.5, label=f"{name} lets go")
        for _ in range(6):
            yield
        for _ in range(int(os.environ.get("WX_DRAW", "3"))):
            yield from arm.move(arm.at() - 0.045 * arm.approach_axis(), t=1.0, label=f"{name} draws back")
        lift = float(os.environ.get("WX_LIFT", "0"))
        if lift > 0:  # and up, so the whole gripper and its wrist stand clear of the arm
            yield from arm.move(arm.at() + np.array([0.0, 0.0, lift]), t=1.0, label=f"{name} lifts clear")
        for _ in range(10):
            yield
        _report(ctx, f"{name} drawn back: ")
        if os.environ.get("WX_HOME", "1") == "1":  # then back to its ready pose, by its own joints' path, well clear of the arm
            hq, hr = ik.tcp_pose(np.array(READY, dtype=float), arm.base)
            yield from arm.move(hq, grip=d._as_grip(hr), t=2.0, label=f"{name} back to ready")
            for _ in range(10):
                yield
        _report(ctx, f"{name} clear: ")
    for k in range(60):
        yield
        if k % 20 == 19:
            _report(ctx, "end: ")
