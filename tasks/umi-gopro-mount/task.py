"""Fix a GoPro to a UMI gripper's camera holder: the camera's two mount fingers between the holder's three, the thumbscrew through all five into the nut, snug.

Holder at (34, 0) on the real UMI gripper (its released print files, umi_body.npz: top cover with the GoPro mount and mirror wings, bottom plate, finger holders, soft fingers), held by its handle in a bench vise,
the finger stack's axis along x; the gripper's fingers lie toward -y, so the lens must face -y. The screw goes in from the -x side; the captive nut is behind the far (+x) holder finger. The thread (README
"What is modelled"): once the screw's tip reaches the nut with its axis on the holes' line, a joint leaves the screw only its turn and its slide along that line, and the scorer credits depth past the nut's face
only as the screw is turned, 0.8 mm a turn (the M5 pitch) plus 0.5 mm of play. Seated by its turns, fixed joints hold camera and screw where they stand (a tightened thumbscrew's friction).

    python demokit.py task.py OUT --no-record --mode full,none,camera_only,no_turn,push_seat,light_spin,slow_light_spin
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
INSTRUCTION = ("Fix the GoPro to the gripper's holder, lens toward the gripper's fingers: thumbscrew through all five fingers, snug in the nut. Then let go.")
SUCCESS_HOLD_S = 0.0
SCORE_OVER = "end"
TOP = d.TABLE_TOP
MM = 1e-3

# -- holder (fixed): the finger stack's axis along x ----------------------------------------------------------------------------------------------------------------------------
HX, HY = 0.34, 0.0
ZH = TOP + 0.1364  # the holes' axis: the gripper body's underside is 51.4 mm below it, 20 mm above the vise's jaws
# the real GoPro mount on the UMI gripper's top cover (umi_body.npz, from the UMI print files): three fingers 4.0, 3.3 and 4.0 mm thick with 3.3 and 3.4 mm gaps, 18 mm in all,
# 15.2 mm wide round the holes, their tops 11.6 mm above the holes' axis and their feet 20.3 mm below it
X0 = HX - 0.009  # near face of the first holder finger
FINGERS = ((0.0, 0.004), (0.0073, 0.0106), (0.014, 0.018))  # each holder finger's two faces past X0
PW = 0.0076  # finger half-width in y round the hole
F_TOP, F_FOOT = 0.0116, -0.0203  # finger top and foot, from the holes' axis
FT = 0.0027  # the camera's mounting fingers' thickness: 0.3 mm of play in each of the holder's gaps
HOLE = 0.0058  # sim hole, from the real 5.2 to 5.5 mm: wider holes let the cantilevered knob droop the screw off the hole line
HOLE_END = 0.0054  # the outer two holder fingers' holes, closer to the real 5.2 mm, so a released screw stays level
FUNNEL = HOLE_END  # the first finger's hole at its near face: a plain hole, as on a real mount
SCREW_R = 0.0025
COVER_TOP, BODY_BOT = ZH - 0.0227, ZH - 0.0514  # the top cover's upper face, the bottom plate's underside
VX = HX + 0.0196  # the body's middle in x, over its handle
HANDLE_Y0, HANDLE_Y1 = -0.015, 0.020  # the handle, 35 mm thick, held in the vise's jaws
JAW_TOP, VISE_W, VISE_Y0, JAW_Y1, VISE_Y1 = TOP + 0.065, 0.080, -0.050, 0.060, 0.070  # the vise: jaws 80 mm wide, 50 mm above its base
X_TOUCH = X0 + 0.016  # where the captive nut's thread starts, in the far finger's recess

# -- camera (free): HERO13 71.8 x 50.8 x 33.6 mm, 159 g; origin at the body's centre; x width, y height (mount fingers out of its -y face), z depth (lens on +z) ---------------------------
CW, CH, CD = 0.0718, 0.0508, 0.0336
CF = 0.0205  # finger length out of the body: the body clears the holder fingers' tops by 0.9 mm
CAM_M = 0.159
CAM_START = np.array([0.21, -0.16])
HOLE_LOCAL_Y = -(CH / 2 + CF - 0.008)  # hole 8 mm from the finger tip
CAM_NOM = np.array([HX, HY, ZH - HOLE_LOCAL_Y])  # origin when mounted
RX90 = np.array([[1.0, 0, 0], [0, 0.0, -1.0], [0, 1.0, 0]])  # lens +z -> -y, fingers -y -> -z

# -- thumbscrew (free): Fotodiox GoTough Long, a 45 mm aluminium knob 15 mm across on an 18 mm M5 stainless shaft, 17 g (maker's figures); origin at the knob's face, shaft along +x
LS = 0.018
KNOB_L, KNOB_R = 0.045, 0.0075
SCREW_M = 0.017
KNOB_FLAT = KNOB_R * math.cos(math.pi / 16)  # the 16-sided knob rests on a flat
OVERHANG = 0.018  # the knob's end over the jaw's -x edge, where the fingers take it
SCREW_START = np.array([VX - VISE_W / 2 - OVERHANG + KNOB_L, (HANDLE_Y1 + JAW_Y1) / 2 + 0.010, JAW_TOP + KNOB_FLAT + 0.0004])  # lying on the vise's movable jaw, shaft toward +x
PITCH = 0.0008
SEAT_X = X0 + LS  # tip x when the head seats on the first finger
SNUG_TOL = 0.0004
SNUG_TORQUE = 0.15  # N m, the finger-tight torque of a small thumbscrew (about 0.1 to 0.2): the thread holds the screw against anything lighter
LIGHT_N = 2.0  # N, a light grip for the light_spin lazy mode

LOOKS = {"body": m.Look(m.rgb(0.92, 0.92, 0.90), roughness=0.6), "holder": m.Look(m.rgb(0.15, 0.15, 0.16), roughness=0.5), "cam": m.Look(m.rgb(0.10, 0.10, 0.11), roughness=0.45),
         "lens": m.Look(m.rgb(0.02, 0.03, 0.06), metallic=0.2, roughness=0.15), "steel": m.Look(m.rgb(0.75, 0.75, 0.77), metallic=0.8, roughness=0.35),
         "paddle": m.Look(m.rgb(0.85, 0.45, 0.10), roughness=0.5), "nut": m.Look(m.rgb(0.80, 0.70, 0.35), metallic=0.8, roughness=0.4), "fin": m.Look(m.rgb(0.20, 0.45, 0.85), roughness=0.7), "vise": m.Look(m.rgb(0.12, 0.28, 0.55), roughness=0.5)}


def plate_boxes(xa, xb, ya, yb, za, zb, hy, hz, hole=None):
    """A finger plate (normal x) with a square hole (default HOLE) at (hy, hz), as four boxes: (size, centre)."""
    h = (HOLE if hole is None else hole) / 2
    out = []
    for (y0, y1, z0, z1) in ((ya, yb, za, hz - h), (ya, yb, hz + h, zb), (ya, hy - h, hz - h, hz + h), (hy + h, yb, hz - h, hz + h)):
        if y1 - y0 > 1e-6 and z1 - z0 > 1e-6:
            out.append(((xb - xa, y1 - y0, z1 - z0), ((xa + xb) / 2, (y0 + y1) / 2, (z0 + z1) / 2)))
    return out


def _wedge(name, outline, a0, a1, kind):
    """A convex collider from a polygon outline (x, w) extruded between a0 and a1: kind z: w is z, extruded along y; kind y: w is y, extruded along z."""
    p = m.prism(np.array(outline), a0, a1)
    v = p.v[:, [0, 2, 1]] if kind == "z" else p.v
    return m.col_hull(name, m.Mesh(v, p.f))


def _hull(name, v, f, keep):
    """A convex collider round the faces of a mesh whose corners all pass `keep`."""
    sel = f[np.all(keep(v)[f], axis=1)]
    used, inv = np.unique(sel, return_inverse=True)
    return m.col_hull(name, m.Mesh(v[used], inv.reshape(-1, 3)))


def _holder() -> m.Part:
    parts, cols = [], []

    def add(name, size, c, look, col=True):
        parts.append((name, m.box(size, c), look))
        if col:
            cols.append(m.col_box(name + "_c", size, c))

    # the real UMI gripper, from its print files: top cover with the GoPro mount and the two mirror wings, bottom plate, finger holders and Fin Ray fingers toward -y
    o = np.array([HX, HY, ZH - TOP])
    z = np.load(pathlib.Path(__file__).resolve().parent / "umi_body.npz")
    mesh = {}
    for name, look in (("top_cover", "body"), ("bottom_plate", "body"), ("holder_l", "holder"), ("holder_r", "holder"), ("finger_l", "fin"), ("finger_r", "fin")):
        mesh[name] = (z[name + "_v"].astype(float) + o, z[name + "_f"].astype(np.int64))
        parts.append((name, m.Mesh(*mesh[name]), look))
    cols.append(m.col_box("body_c", (0.172, 0.0914, COVER_TOP - BODY_BOT), (VX, HY - 0.0272, (COVER_TOP + BODY_BOT) / 2 - TOP)))
    cols.append(m.col_box("mount_foot_c", (0.0186, 0.038, F_FOOT + 0.0227), (HX + 0.0002, HY - 0.0005, ZH + (F_FOOT - 0.0227) / 2 - TOP)))
    v, f = mesh["top_cover"]
    above = ZH - 0.0222 - TOP
    cols.append(_hull("wing_near_c", v, f, lambda p: (p[:, 0] < HX - 0.025) & (p[:, 2] > above)))
    cols.append(_hull("wing_far_c", v, f, lambda p: (p[:, 0] > HX + 0.060) & (p[:, 2] > above)))
    # the handle down into the vise's jaws, its trigger toward the fingers
    add("handle", (0.030, HANDLE_Y1 - HANDLE_Y0, BODY_BOT - TOP - 0.030), (VX, (HANDLE_Y0 + HANDLE_Y1) / 2, (BODY_BOT - TOP + 0.030) / 2), "holder")
    add("trigger", (0.012, 0.012, 0.022), (VX, HANDLE_Y0 - 0.006, BODY_BOT - TOP - 0.016), "holder", col=False)
    # a small bench vise on the bench, its jaws along y round the handle; the movable jaw's top is where the thumbscrew was set down
    add("vise_base", (0.10, VISE_Y1 - VISE_Y0, 0.015), (VX, (VISE_Y0 + VISE_Y1) / 2, 0.0075), "vise")
    add("vise_fixed", (VISE_W, HANDLE_Y0 - VISE_Y0, JAW_TOP - TOP - 0.015), (VX, (VISE_Y0 + HANDLE_Y0) / 2, (JAW_TOP - TOP + 0.015) / 2), "vise")
    add("vise_moving", (VISE_W, JAW_Y1 - HANDLE_Y1, JAW_TOP - TOP - 0.015), (VX, (HANDLE_Y1 + JAW_Y1) / 2, (JAW_TOP - TOP + 0.015) / 2), "vise")
    for k, y in enumerate((HANDLE_Y0 - 0.002, HANDLE_Y1 + 0.002)):
        add(f"vise_face{k}", (VISE_W, 0.004, 0.028), (VX, y, JAW_TOP - TOP - 0.014), "steel", col=False)
    parts.append(("vise_spindle", m.cylinder(0.007, VISE_Y1 + 0.05 - JAW_Y1, (VX, (JAW_Y1 + VISE_Y1 + 0.05) / 2, 0.036), axis="y"), "steel"))
    parts.append(("vise_bar", m.cylinder(0.004, 0.10, (VX, VISE_Y1 + 0.05, 0.036), axis="x"), "steel"))
    # the mount's three fingers, as colliders round the real mesh: plates with square holes, chamfered tops
    z0, zt, hz = ZH + F_FOOT - TOP, ZH + F_TOP - TOP, ZH - TOP
    for k, (a, b) in enumerate(FINGERS):
        xa, xb = X0 + a, X0 + b
        if k:
            hole = HOLE_END if k == 2 else HOLE
            for j, (sz, c) in enumerate(plate_boxes(xa, xb, -PW, PW, z0, zt, 0.0, hz, hole)):
                if j != 1:
                    cols.append(m.col_box(f"h{k}_{j}_c", sz, c))
            cols.append(_wedge(f"h{k}_top_c", [(xa, hz + hole / 2), (xb, hz + hole / 2), (xb, zt - 0.003), (xb - 0.0012, zt), (xa + 0.0012, zt), (xa, zt - 0.003)], -PW, PW, "z"))
    # the first finger: a plain square hole of HOLE_END, its colliders as four convex wedges round it
    xa, xb, r1, r2 = X0, X0 + FINGERS[0][1], FUNNEL / 2, HOLE_END / 2
    for name, outline, a0, a1, kind in (
        ("top", [(xa, hz + r1), (xb, hz + r2), (xb, zt - 0.003), (xb - 0.0012, zt), (xa + 0.0012, zt), (xa, zt - 0.003)], -PW, PW, "z"),
        ("bot", [(xa, z0), (xb, z0), (xb, hz - r2), (xa, hz - r1)], -PW, PW, "z"),
        ("left", [(xa, -PW), (xb, -PW), (xb, -r2), (xa, -r1)], hz - r2, hz + r2, "y"),
        ("right", [(xa, r1), (xb, r2), (xb, PW), (xa, PW)], hz - r2, hz + r2, "y"),
    ):
        cols.append(_wedge(f"h0_{name}_c", outline, a0, a1, kind))
    # the captive M5 nut (8 mm across flats, 4 mm thick) in the far finger's recess, its outer 2 mm standing proud (the thread is a rule, README)
    parts.append(("nut", m.cylinder(0.0046, 0.004, (X0 + 0.018, 0.0, hz), axis="x", segments=6), "nut"))
    for j, (sz, c) in enumerate(plate_boxes(X0 + 0.018, X0 + 0.020, -0.004, 0.004, hz - 0.004, hz + 0.004, 0.0, hz, HOLE_END)):
        cols.append(m.col_box(f"nut_{j}_c", sz, c))
    return m.fixed("holder", (0.0, 0.0, TOP), parts, cols, LOOKS, friction=(0.6, 0.6))


def _camera() -> m.Part:
    parts = [("body", m.box((CW, CH, CD)), "cam"), ("lens", m.cylinder(0.014, 0.004, (0.0, 0.006, CD / 2 + 0.002), axis="z"), "lens"),
             ("screen", m.box((0.03, 0.02, 0.001), (0.0, -0.01, -CD / 2 - 0.0005)), "lens")]
    cols = [m.col_box("body_c", (CW, CH, CD))]
    y_tip = -(CH / 2 + CF)
    hy, hz = HOLE_LOCAL_Y, 0.0
    for k, xa in enumerate((-0.0047, 0.00195)):  # centred in the holder's two gaps
        for j, (sz, c) in enumerate(_cam_plate(xa, y_tip, hy)):
            parts.append((f"f{k}_{j}", m.box(sz, c), "cam"))
            cols.append(m.col_box(f"f{k}_{j}_c", sz, c))
        parts.append((f"f{k}_tip", m.box((FT, hy - HOLE / 2 - y_tip, 2 * PW), (xa + FT / 2, (y_tip + hy - HOLE / 2) / 2, 0.0)), "cam"))
        cols.append(_cam_tip(f"f{k}_tip_c", xa, y_tip, hy))
    return m.free("camera", (CAM_START[0], CAM_START[1], TOP + CD / 2 + 0.0004), parts, cols, LOOKS, mass=CAM_M, friction=(0.7, 0.7), torsional_patch=None, solver_iters=64)


def _cam_plate(xa, y_tip, hy):
    """Camera finger: plate in (y, z), hole at (y = hy, z = 0)."""
    h = HOLE / 2
    ya, yb = y_tip, -CH / 2 + 0.001
    out = []
    for (y0, y1, z0, z1) in ((hy + h, yb, -PW, PW), (hy - h, hy + h, -PW, -h), (hy - h, hy + h, h, PW)):
        out.append(((FT, y1 - y0, z1 - z0), (xa + FT / 2, (y0 + y1) / 2, (z0 + z1) / 2)))
    return out


def _cam_tip(name, xa, y_tip, hy):
    h = HOLE / 2
    return _wedge(name, [(xa, y_tip + 0.003), (xa + 0.0012, y_tip), (xa + FT - 0.0012, y_tip), (xa + FT, y_tip + 0.003), (xa + FT, hy - h), (xa, hy - h)], -PW, PW, "y")


def _screw() -> m.Part:
    sh = m.lathe([(0.0, 0.0), (SCREW_R, 0.0), (SCREW_R, LS - 0.0018), (0.0012, LS), (0.0, LS)], 16).turned(90, "y")
    kn = m.lathe([(0.0, -KNOB_L), (KNOB_R - 0.0008, -KNOB_L), (KNOB_R, -KNOB_L + 0.0008), (KNOB_R, -0.0008), (KNOB_R - 0.0008, 0.0), (0.0, 0.0)], 16).turned(11.25, "z").turned(90, "y")
    parts = [("shank", sh, "steel"), ("knob", kn, "paddle")]
    cols = [m.col_hull("shank_c", sh), m.col_hull("knob_c", kn)]
    return m.free("screw", tuple(float(v) for v in SCREW_START), parts, cols, LOOKS, mass=SCREW_M, friction=(0.8, 0.8), torsional_patch=None, solver_iters=64,
                  max_depen=None)


OBJECTS = [_holder(), _camera(), _screw()]


# -- readings -------------------------------------------------------------------------------------------------------------------------------------------------------------------


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0, 0], [0, c, -s], [0, s, c]])


def cam_geom(c):
    return np.asarray(c.pos, float), c.rotation()


def screw_geom(s):
    R = s.rotation()
    head = np.asarray(s.pos, float)
    tip = head + R[:, 0] * LS
    return head, tip, R


def cam_angles(pos, R):
    """(interleave position error m, tilt deg from the mounted pose, rotation about x relative to it (rad), lens angle deg from -y)."""
    rel = R @ RX90.T
    psi = math.atan2(rel[2, 1], rel[1, 1])
    tilt = math.degrees(math.acos(float(np.clip((np.trace(rel) - 1) / 2, -1, 1))))
    lens = math.degrees(math.acos(float(np.clip(-R[1, 2], -1, 1))))
    return pos - CAM_NOM, tilt, psi, lens


def interleaved(pos, R) -> bool:
    e, tilt, _, _ = cam_angles(pos, R)
    return bool(abs(e[0]) < 0.0015 and abs(e[1]) < 0.003 and abs(e[2]) < 0.0025 and tilt < 8.0)


def in_stack(head, tip, R) -> bool:
    """The screw through the stack from the near side: on the holes' line, its knob's face short of the first finger, its tip at the far finger (the nut)."""
    return bool(axis_ok(head, tip, R) and head[0] <= X0 + 0.0005 and tip[0] >= X_TOUCH - 0.0003)


def axis_ok(head, tip, R) -> bool:
    ax = R[:, 0]
    if ax[0] < 0.995:
        return False
    t = (HX - head[0]) / ax[0]
    p = head + ax * t
    return bool(abs(p[1] - HY) < 0.002 and abs(p[2] - ZH) < 0.002)


def box_dist(p, half):
    q = np.abs(np.atleast_2d(p)) - half
    return np.linalg.norm(np.maximum(q, 0.0), axis=1)


CLEAR, QUIET_STEPS = 0.05, 20  # hands 5 cm clear of camera and screw, and both still, for 1 s


def gripper_points(env) -> np.ndarray:
    """Points over the whole of both grippers (fingers, palm and housing), station frame: a grid on each, at the current opening."""
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


def hands_gap(env, cpos, cR, head, tip) -> float:
    """The least distance from any gripper point to the camera's box (body and mount fingers) or to the screw (knob and shaft, as a rod of the knob's radius)."""
    pts = gripper_points(env)
    dc = box_dist((pts - cpos) @ cR, np.array([CW / 2, (CH + 2 * CF) / 2, CD / 2]))
    ax = tip - head
    back = head - ax / np.linalg.norm(ax) * KNOB_L
    seg = tip - back
    t = np.clip(((pts - back) @ seg) / float(seg @ seg), 0.0, 1.0)
    ds = np.linalg.norm(pts - (back + t[:, None] * seg), axis=1) - KNOB_R
    return float(min(dc.min(), ds.min()))


class Trial:
    def reset(self, env=None):
        self.n = 0
        self.ang = None
        self.theta = 0.0
        self.engaged = False
        self.snug = False
        self.quiet = 0
        self.done = False
        self.final = 0.0
        self.last = {}
        self.clamped = False
        self.credit = -1.0
        self.live_tier = 0.0
        if _guide(env, False) | _clamp(env, False):
            _put_back(env)  # a joint left from the last trial held the parts through the reset's settling: start them again

    def live(self, cpos, cR, head, tip, sR):
        """The thread is a rule: depth past the nut's thread counts only as far as the screw has been turned since it reached it, 0.8 mm a turn (and 0.5 mm of play)."""
        inter = interleaved(cpos, cR)
        through = inter and in_stack(head, tip, sR)
        snug = through and self.engaged and tip[0] >= SEAT_X - SNUG_TOL and self.credit >= SEAT_X - SNUG_TOL
        return bool(inter), bool(through), bool(snug)

    def step(self, env):
        cam, scr = m.state(env, "camera"), m.state(env, "screw")
        cpos, cR = cam_geom(cam)
        head, tip, sR = screw_geom(scr)
        self.n += 1
        # the thread rule: count the screw's turns about its own axis from when its tip reaches the far finger (the nut)
        a = math.atan2(sR[2, 1], sR[1, 1])
        if self.ang is None:
            self.ang = a
        da = (a - self.ang + math.pi) % (2 * math.pi) - math.pi
        self.ang = a
        if (not self.engaged) and interleaved(cpos, cR) and in_stack(head, tip, sR):
            # the tip has found the nut's thread: from here the screw only turns and slides on the holes' line (a joint, as a started thread holds it)
            self.engaged = True
            self.theta = 0.0
            _guide(env, True)
        if self.engaged:
            self.theta += da
        self.credit = X_TOUCH + max(self.theta, 0.0) / (2 * math.pi) * PITCH + 0.0005 if self.engaged else -1.0
        inter, through, snug_now = self.live(cpos, cR, head, tip, sR)
        _, tilt, psi, lens = cam_angles(cpos, cR)
        # seated by its turns: the tightened thumbscrew's friction holds camera and screw where they stand (fixed joints)
        if snug_now and not self.clamped:
            _clamp(env, True)
            self.clamped = True
        self.snug = bool(snug_now)
        clear = False
        if self.snug and not self.done:
            # judge: hands clear (the whole gripper) and everything still for 1 s
            clear = hands_gap(env, cpos, cR, head, tip) >= CLEAR
            still = float(np.linalg.norm(cam.lin_vel)) + float(np.linalg.norm(scr.lin_vel)) < 0.01
            self.quiet = self.quiet + 1 if (clear and still) else 0
            if self.quiet >= QUIET_STEPS:
                self.final = 1.0  # the lens already faces the gripper's fingers: the interleave holds the camera within 8 deg of the mounted pose
                self.done = True
                self.last = {"tier": self.final, "lens": round(lens, 1), "turns": round(self.theta / (2 * math.pi), 2)}
                print("[gp] JUDGED " + json.dumps(self.last), flush=True)
        self.live_tier = 0.75 if snug_now else (0.5 if through else (0.25 if inter else 0.0))
        if self.n % 10 == 0:
            print("[gp] " + json.dumps({"mode": os.environ.get("TASK_DEMO", ""), "n": self.n, "inter": bool(inter), "through": bool(through), "tip_mm": round((tip[0] - X0) * 1000, 2), "theta": round(self.theta / (2 * math.pi), 2),
                                        "ax": [round(float(sR[1, 0]) * 1000), round(float(sR[2, 0]) * 1000)], "hd": [round(float(head[1]) * 1000, 2), round(float(head[2] - ZH) * 1000, 2)], "eng": bool(self.engaged), "snug": bool(self.snug), "lens": round(lens, 1), "tilt": round(tilt, 1), "e": [round(float(v) * 1000, 2) for v in cpos - CAM_NOM], "clear": bool(clear), "q": self.quiet, "done": self.done,
                                        "tier": self.final}), flush=True)


def _stage():
    import omni.usd

    return omni.usd.get_context().get_stage()


def _guide(env, on: bool):
    """The started thread (on): a joint from the world to the screw that leaves it only its turn and its slide along the holes' line, from where it stands to the seated depth,
    with a little thread friction on the turn. Off: removed."""
    if env is None or not hasattr(env, "unwrapped"):
        return False
    from pxr import Gf, Sdf, UsdPhysics

    stage = _stage()
    path = Sdf.Path(f"{m.ENV_PATH}/thread_screw")
    had = stage.GetPrimAtPath(path).IsValid()
    if had:
        stage.RemovePrim(path)
    if not on:
        return had
    st = m.state(env, "screw")
    org = m._np(env.unwrapped.scene.env_origins[0])
    x_head = float(st.pos[0])
    j = UsdPhysics.Joint.Define(stage, path)
    j.CreateBody1Rel().SetTargets([Sdf.Path(f"{m.ENV_PATH}/screw")])
    j.CreateLocalPos0Attr(Gf.Vec3f(float(x_head + org[0]), float(HY + org[1]), float(ZH + org[2])))
    j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    prim = j.GetPrim()
    for ax in ("transY", "transZ", "rotY", "rotZ"):  # low above high: locked
        lim = UsdPhysics.LimitAPI.Apply(prim, ax)
        lim.CreateLowAttr(1.0)
        lim.CreateHighAttr(-1.0)
    lim = UsdPhysics.LimitAPI.Apply(prim, "transX")
    lim.CreateLowAttr(float(min(0.0, X_TOUCH - 0.0002 - LS - x_head)))  # a started thread does not let the tip back out of the nut
    lim.CreateHighAttr(float(SEAT_X - LS - x_head + 0.0003))
    drv = UsdPhysics.DriveAPI.Apply(prim, "rotX")
    drv.CreateTypeAttr("force")
    drv.CreateTargetVelocityAttr(0.0)
    drv.CreateStiffnessAttr(0.0)
    drv.CreateDampingAttr(0.05)  # 2.9 N m s/rad (0.05 per degree/s as USD states it): the drive reaches its 0.15 N m cap at 3 deg/s, so the thread resists with that torque at any turning speed above 3 deg/s
    drv.CreateMaxForceAttr(SNUG_TORQUE)
    print(f"[gp] thread: the screw now turns and slides only on the holes' line (head at {(x_head - X0) * 1000:.1f} mm)", flush=True)


def _clamp(env, on: bool):
    """Fix the camera and the screw to the world where they stand (on), or remove the fixings (off): fixed joints on the live stage."""
    if env is None or not hasattr(env, "unwrapped"):
        return False
    from pxr import Gf, Sdf, UsdPhysics

    stage = _stage()
    had = False
    for part in ("camera", "screw"):
        path = Sdf.Path(f"{m.ENV_PATH}/clamp_{part}")
        if stage.GetPrimAtPath(path).IsValid():
            stage.RemovePrim(path)
            had = True
        if not on:
            continue
        st = m.state(env, part)  # the live pose from the simulator (the USD prim is not written back while it runs)
        pw = np.asarray(st.pos, float) + m._np(env.unwrapped.scene.env_origins[0])
        qx, qy, qz, qw = (float(v) for v in st.quat_xyzw)
        j = UsdPhysics.FixedJoint.Define(stage, path)
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{m.ENV_PATH}/{part}")])
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(*(float(v) for v in pw)))
        j.CreateLocalRot0Attr().Set(Gf.Quatf(qw, qx, qy, qz))
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        j.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    if on:
        print("[gp] clamp on: the seated screw holds the camera", flush=True)
    return had


def _put_back(env):
    """Write the camera and the screw back to their start poses, at rest."""
    base = env.unwrapped
    for part in ("camera", "screw"):
        obj = base.scene[part]
        pose = obj.data.default_root_pose.torch.clone()
        pose[:, :3] += base.scene.env_origins
        obj.write_root_pose_to_sim_index(root_pose=pose)
        obj.write_root_velocity_to_sim_index(root_velocity=obj.data.default_root_vel.torch.clone() * 0.0)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects) -> float:
    return TRIAL.final if TRIAL.done else TRIAL.live_tier


def success(objects) -> bool:
    return bool(TRIAL.done and TRIAL.final >= 1.0)


# -- the scripted demo ------------------------------------------------------------------------------------------------------------------------------------------------------------

GRIP_N = 30.0
CAM_PZ = 0.0175  # TCP height above the table when the camera is picked (it is 33.6 mm tall)
CAM_OFF = 0.015  # right hand's grip this far from the camera's centre toward its top: its fingers pass 7 mm over the mirror wings
KT = 0.044  # left TCP this far behind the knob's face, along -x, for the pick and the push: its fingertips stop 4 mm short of the near mirror wing
TCP_IN = 0.0055  # the left TCP this far from the knob's axis toward the hand: its fingertips reach 7.5 mm past the axis
TH = 35.0  # each turning stroke, deg: the left hand swings from beside the knob (+y) toward its top, clear of the cover, the near mirror wing, the camera and the right hand
TURNS_MAX = 30


def _knob_end(ctx):
    """The knob's end, on the screw's axis."""
    s = ctx.state("screw")
    return np.asarray(s.pos, float) - m.quat_matrix(s.quat_xyzw)[:, 0] * KNOB_L


def _screw_tip(ctx):
    s = ctx.state("screw")
    R = m.quat_matrix(s.quat_xyzw)
    return np.asarray(s.pos, float) + R[:, 0] * LS


def _note_screw(ctx, label):
    s = ctx.state("screw")
    ax = m.quat_matrix(s.quat_xyzw)[:, 0]
    print(f"[gp] screw after {label}: pos {np.round((np.asarray(s.pos, float) - np.array([HX, HY, ZH])) * 1000, 1).tolist()} mm of the hole, pitch {math.degrees(math.asin(float(np.clip(ax[2], -1, 1)))):.1f}, yaw {math.degrees(math.asin(float(np.clip(ax[1], -1, 1)))):.1f} deg", flush=True)


def demo_script(ctx):
    """Modes: full. none. camera_only: the camera is mounted, no screw. no_turn: the screw is pushed in to the nut and let go, not turned.
    push_seat: the screw is pushed on until its knob meets the first finger, never turned, and let go.
    light_spin: as full, but the knob is held at only 2 N while it is rolled: too light to turn the snug thread.
    slow_light_spin: as light_spin, the strokes taken at 2 deg/s, under the speed where the thread's resisting torque reaches its cap."""
    mode = ctx.mode
    if mode not in ("full", "none", "camera_only", "no_turn", "push_seat", "light_spin", "slow_light_spin"):
        raise ValueError(f"unknown mode {mode!r}")
    L, R = ctx.left, ctx.right
    if mode == "none":
        yield from L.wait(3.0)
        return
    cam = lambda: ctx.pos("camera")
    def mount_camera():
        # right hand: pick the camera across its width, pitch it to fingers-down, mount it
        g0 = d._as_grip(d.axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.radians(-30.0)) @ d.down(90).rot)  # tool leaning 30 deg toward the base: after the pitch it leans down-forward, its housing rising behind the camera, clear of the mirror wings
        cp = lambda: np.array([cam()[0], cam()[1] + CAM_OFF, TOP + CAM_PZ])
        yield from R.move(cp() + [0.0, 0.0, 0.10], grip=g0, finger=d.FINGER_OPEN, label="right over the camera")
        yield from R.servo(cp() + [0.0, 0.0, 0.10], timeout=1.0)
        yield from R.move(cp(), t=1.2, label="right down")
        yield from R.close(CW, 35.0, label="right grips the camera")
        yield from R.move(R.at() + [0.0, 0.0, 0.07], t=1.2, label="lift")
        above = CAM_NOM + np.array([0.0, 0.0, 0.025])  # its fingers' tips 5 mm over the holder's
        pitch_at = np.array([0.20, -0.08, TOP + 0.09])  # where the right arm reaches the whole pitch with its tool leaning (checked with gpm_plan.py)
        yield from R.move(R.at() + (pitch_at - cam()), t=2.0, label="to where the camera is pitched")
        yield from R.arc(cam(), (1.0, 0.0, 0.0), 90.0, t=2.5, label="pitch fingers down")
        yield from R.move(R.at() + [0.0, 0.0, above[2] - cam()[2]], t=1.2, label="up")
        yield from R.move(R.at() + [0.0, above[1] - cam()[1], 0.0], t=1.5, label="across, short of the gripper's body")
        yield from R.move(R.at() + (above - cam()), t=1.5, label="over the holder, its mount fingers inside the near mirror wing")
        yield from R.servo(above, read=cam, tol=0.0004, timeout=4.0, label="align")
        yield from R.servo(CAM_NOM + [0.0, 0.0, 0.006], read=cam, tol=0.0004, timeout=3.0, label="down 1")
        yield from R.servo(CAM_NOM + [0.0, 0.0, 0.0005], read=cam, tol=0.0004, timeout=3.0, label="down 2")
        yield from R.servo(CAM_NOM + [0.0, 0.0, -0.0008], read=cam, tol=0.0004, timeout=2.0, label="down 3")
        for _ in range(3):  # level by feedback: turn the camera about the hole line by its own measured rotation about x
            c = ctx.state("camera")
            e, tilt, psi, lens = cam_angles(*cam_geom(c))
            print(f"[gp] level: rotation about x {math.degrees(psi):.2f} deg, tilt {tilt:.2f}", flush=True)
            if abs(math.degrees(psi)) < 0.25:
                break
            yield from R.arc(np.array([HX, HY, ZH]), (1.0, 0.0, 0.0), -math.degrees(psi), t=0.8, label="level")
            yield from R.servo(CAM_NOM, read=cam, tol=0.0004, timeout=2.0, label="re-seat")
        c = ctx.state("camera")
        print(f"[gp] camera at {[round(float(v) * 1000, 1) for v in cam() - CAM_NOM]} mm of nominal", flush=True)

    yield from mount_camera()
    if mode == "camera_only":
        yield from R.wait(2.0)
        return
    # left hand: take the knob level from -x, fingers +x, to carry the screw in along the holes' line
    lg = d.tilted(80.0, 0.0)  # fingers 10 deg below level: the only near-level grip the left arm reaches on the vise's jaw; the screw is levelled in hand below
    kt = lambda: ctx.pos("screw") + np.array([-KT, 0.0, 0.0])
    yield from L.move(kt() + [0.0, 0.0, 0.06], grip=lg, finger=-0.016, label="left over the knob")  # opened only 32 mm: wider, its outer pad would reach under the gripper body, 8.5 mm over the jaw
    yield from L.servo(kt() + [0.0, 0.0, 0.06], timeout=1.0)
    yield from L.move(kt(), t=1.5, label="left down round the knob")
    yield from L.servo(kt(), timeout=1.5)
    yield from L.close(2 * KNOB_R, GRIP_N, label="left grips the knob")
    _note_screw(ctx, "left grips the knob")
    yield from L.move(L.at() + [0.0, 0.0, 0.010], t=1.0, label="lift off the vise")
    _note_screw(ctx, "lift off the vise")
    yield from L.move(L.at() + [-0.050, 0.0, 0.0], t=1.5, label="back, clear of the holder")
    _note_screw(ctx, "back, clear of the holder")
    yield from L.move(L.at() + [0.0, 0.0, ZH - _screw_tip(ctx)[2]], t=2.0, label="up to the holes' height")
    _note_screw(ctx, "up to the holes' height")
    for _ in range(3):  # level the screw by its measured axis: turn the hand about the knob it holds
        ax = m.quat_matrix(ctx.state("screw").quat_xyzw)[:, 0]
        pitch, yaw = math.degrees(math.asin(float(np.clip(ax[2], -1, 1)))), math.degrees(math.asin(float(np.clip(ax[1], -1, 1))))
        print(f"[gp] screw level: pitch {pitch:.2f}, yaw {yaw:.2f} deg", flush=True)
        if abs(pitch) < 0.5 and abs(yaw) < 0.5:
            break
        yield from L.turn(pitch, axis=(0.0, 1.0, 0.0), t=0.6, label="level the screw")
        yield from L.turn(-yaw, axis=(0.0, 0.0, 1.0), t=0.6, label="square the screw")
        yield from L.wait(0.3)
    # line the tip up with the holes, then push in along +x, the tip kept on the hole line
    aim = np.array([X0 - 0.012, HY, ZH])
    yield from L.move(L.at() + (aim - _screw_tip(ctx)), t=2.5, label="to the holder")
    yield from L.servo(aim, read=lambda: _screw_tip(ctx), tol=0.0005, timeout=4.0, label="aim")
    goal_x = X_TOUCH + 0.0002
    v = 0.008
    t_in = (goal_x - aim[0]) / v
    yield from L.servo(lambda t: np.array([min(aim[0] + v * t, goal_x), HY, ZH]), read=lambda: _screw_tip(ctx), tol=0.0006, timeout=t_in + 3.0, label="push in")
    for _ in range(2):  # stopped short: draw back 4 mm, re-aim on the hole line, push again
        if TRIAL.engaged:
            break
        back = np.array([_screw_tip(ctx)[0] - 0.004, HY, ZH])
        yield from L.servo(back, read=lambda: _screw_tip(ctx), tol=0.0005, timeout=1.5, label="draw back")
        yield from L.servo(lambda t: np.array([min(back[0] + 0.006 * t, goal_x), HY, ZH]), read=lambda: _screw_tip(ctx), tol=0.0006, timeout=3.0, label="push again")
    print(f"[gp] tip at {(_screw_tip(ctx)[0] - X0) * 1000:.1f} mm of the stack (thread starts at {(X_TOUCH - X0) * 1000:.1f}); live tier {TRIAL.live_tier}", flush=True)
    if mode == "push_seat":
        yield from L.servo(np.array([SEAT_X, HY, ZH]), read=lambda: _screw_tip(ctx), tol=0.0003, timeout=3.0, label="push to the seat")
        print(f"[gp] pushed: tip at {(_screw_tip(ctx)[0] - X0) * 1000:.1f} mm, turns {TRIAL.theta / (2 * math.pi):.2f}, live tier {TRIAL.live_tier}", flush=True)
    yield from L.close(2 * KNOB_R + 0.010, 0.0, t=0.4)
    yield from L.move(L.at() + [-0.04, 0.0, 0.0], t=0.6, label="left off the knob's end")
    if mode in ("no_turn", "push_seat"):
        yield from L.move(L.at() + [0.0, 0.08, 0.06], t=1.0)
        yield from R.open()
        yield from R.move(R.at() + [0.0, -0.10, 0.0], t=1.0)
        yield from R.wait(4.0)
        return
    # turn: strokes of TH degrees about the holes' line, each from beside the knob toward its top; between strokes the hand opens, backs off and swings back
    # take the knob's end again from beside it (+y), the fingers closing on its top and bottom: the one grip that turns it clear of the near mirror wing
    side = d._as_grip(d.axis_angle_matrix(np.array([0.0, 1.0, 0.0]), math.pi / 2) @ d.level(-90).rot)
    yield from L.move(L.at() + [0.0, 0.08, 0.0], t=1.0, label="left out to the side")
    n = 0
    light = mode in ("light_spin", "slow_light_spin")
    grip_n = LIGHT_N if light else GRIP_N
    while n < (6 if light else TURNS_MAX) and not TRIAL.snug:  # until seated by its turns: the tip can run ahead of what the turns allow
        tg = lambda: _knob_end(ctx) + np.array([-0.0055, TCP_IN, 0.0])  # the side pads (40 mm wide along x) end 14.5 mm past the knob end, 3.5 mm short of the camera and the right hand
        yield from L.move(tg() + [0.0, 0.04, 0.0], grip=side, t=0.6, label="beside the knob")
        yield from L.move(tg(), t=0.5, label="in")
        yield from L.close(2 * KNOB_R, grip_n, t=0.3)
        if not TRIAL.engaged:  # backed out: push it to the nut again along the hole line
            yield from L.servo(np.array([goal_x, HY, ZH]), read=lambda: _screw_tip(ctx), tol=0.0005, timeout=2.0, label="re-push")
        # the thread draws the screw in as it turns: the hand pushes it on by what the turns so far allow (the screw's own turn, read from its pose)
        goal = np.array([min(TRIAL.credit - 0.0004, SEAT_X), HY, ZH]) if TRIAL.engaged else _screw_tip(ctx)
        yield from L.servo(goal, read=lambda: _screw_tip(ctx), tol=0.0003, timeout=0.6, label="draw")
        # swing about the holes' line: the commanded centre is the line seen from the hand, shifted by the arm's lag (command minus where the TCP is)
        p_act, p_cmd = L.tcp(), L.at() + L.trim
        a_pt = np.array([p_act[0], HY, ZH])
        yield from L.arc(p_cmd + (a_pt - p_act), (1.0, 0.0, 0.0), TH, t=TH / (2.0 if mode == "slow_light_spin" else 60.0), label="turn")
        yield from L.close(2 * KNOB_R + 0.010, 0.0, t=0.3)  # open 5 mm a side
        out = np.array([0.0, math.cos(math.radians(TH)), math.sin(math.radians(TH))])
        yield from L.move(L.at() + 0.04 * out, t=0.4, label="off")
        yield from L.arc(np.array([L.at()[0], HY, ZH]) + L.trim, (1.0, 0.0, 0.0), -TH, t=TH / 90.0, label="back")
        n += 1
        print(f"[gp] stroke {n}: tip {(_screw_tip(ctx)[0] - X0) * 1000:.2f} mm, turns {TRIAL.theta / (2 * math.pi):.2f}, live {TRIAL.live_tier}", flush=True)
    yield from L.open()
    yield from L.move(L.at() + [0.0, 0.08, 0.0], t=1.0)
    yield from L.move(L.at() + [0.0, 0.0, 0.10], t=1.0)
    yield from R.open()
    yield from R.move(R.at() + [0.0, -0.12, 0.0], t=1.5)
    yield from R.move(R.at() + [0.0, 0.0, 0.10], t=1.0)
    yield from R.wait(5.0)
    print("[gp] END " + json.dumps({"mode": mode, **TRIAL.last}), flush=True)
