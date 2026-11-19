"""Put a Ken doll's head back on: hold the body in the left hand, push the head, neck hole first, onto the neck post's ball until its lip snaps over.

A Barbie Fashionistas Ken #227 (listed about 12 in = 30 cm tall) lies face up on the bench with its head off. Its neck post (a 5 mm stem and a 7 mm ball, 9 mm long) points toward -y.
The head lies on its side nearby, its 5 mm neck hole facing +x, away from the post. The left hand lifts the body by its hips and holds it; the right hand picks the head, turns it a
quarter turn about the vertical so the hole faces the post, lines the hole up with the ball by the two poses, and presses it on along the post, steering the head's axis onto the post's.

Each of the two parts is one free body. The head's neck hole is a real bore (a ring of 16 wedges, 7.5 mm across, with a lead-in funnel 10 mm across at the mouth, 2.5 mm deep) over a ball 7.0 mm
across, so the ball must come in within about 1.5 mm of the hole's axis; it slides in freely to the neck's stop. The snap, which a rigid bore cannot do, is a rule on the robot's real push:
the head counts as snapped once it is home (within 2 mm and 14 degrees of the post's axis) while the hand holding it pushes it at least 8 N along the post for 3 control steps, read from
that arm's joint springs. A snapped head is then held on by a 4 N force along the neck in code, and the scorer's 3 N pull at the end checks that it stayed snapped.

Debug:  python demokit.py task.py OUT --no-record --mode full,none,light,unseated,unheld,crooked
Final:  python demokit.py task.py OUT --mode full
"""

from __future__ import annotations

import math
import os

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demokit as d  # noqa: E402
import mechkit as m  # noqa: E402

SCENE = "station"
INSTRUCTION = "Put the doll's head back on its neck."
SUCCESS_HOLD_S = 0.0  # the scorer holds the end state itself
SCORE_OVER = "end"
TOP = d.TABLE_TOP
MM = 1e-3

LOOKS = {"skin": m.Look(m.rgb(0.97, 0.80, 0.67), metallic=0.0, roughness=0.4), "hair": m.Look(m.rgb(0.30, 0.17, 0.07), metallic=0.0, roughness=0.25),
         "shirt": m.Look(m.rgb(0.16, 0.42, 0.78), metallic=0.0, roughness=0.7), "pants": m.Look(m.rgb(0.62, 0.55, 0.40), metallic=0.0, roughness=0.7),
         "ink": m.Look(m.rgb(0.06, 0.05, 0.05), metallic=0.0, roughness=0.4), "white": m.Look(m.rgb(0.96, 0.96, 0.94), metallic=0.0, roughness=0.3),
         "iris": m.Look(m.rgb(0.10, 0.22, 0.45), metallic=0.0, roughness=0.2), "lips": m.Look(m.rgb(0.72, 0.36, 0.33), metallic=0.0, roughness=0.35),
         "belt": m.Look(m.rgb(0.25, 0.15, 0.08), metallic=0.0, roughness=0.5),
         "shoe": m.Look(m.rgb(0.95, 0.95, 0.95), metallic=0.0, roughness=0.5), "plastic": m.Look(m.rgb(0.97, 0.80, 0.67), metallic=0.0, roughness=0.4)}

# -- the doll (estimates from listing photos: the listed height is 12 in = 30 cm) ------------------------------------------------
BX = float(os.environ.get("DH_BX", "0.34"))  # the body's axis, x
NECK_Y = float(os.environ.get("DH_NECKY", "0.008"))  # the post's root
HX, HY = float(os.environ.get("DH_HX", "0.28")), float(os.environ.get("DH_HY", "-0.16"))  # the head's mouth
STEM, BALL_R, POST_L = 0.0055, 0.0035, 0.009  # the ball's centre 5.5 mm from the root, its tip 9 mm
BORE_A = 0.00375  # the bore's apothem: 7.5 mm across over the 7 mm ball
MOUTH_A, FUN_D, BORE_D = 0.0050, 0.0025, 0.009
HEAD_L = 0.040
M_BODY, M_HEAD = 0.090, 0.012
NW = 16
Z_BODY = TOP + 0.011  # the body's origin (neck root): lies on its back, the post along the mid-thickness of its 22 mm chest

S0, S1 = -0.00245, 0.00445  # ball centre depth in the head's frame (m) where the ball starts to meet the 5 mm hole, and where it clears the 2 mm lip
S_STOP = POST_L - BALL_R  # 5.5 mm: the ball's tip on the bore's floor
F_SNAP = float(os.environ.get("DH_SNAP", "8.0"))
F_SEAT = float(os.environ.get("DH_SEAT", "4.0"))
F_PULL = 3.0
PULL_STEPS, HOLD_STEPS = 20, 20


def _quat(R):
    """xyzw of a rotation matrix (Shepperd: safe at 180 degrees)."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = 2 * math.sqrt(1.0 + t)
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, s / 4)
    i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2 * math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k])
    q = [0.0, 0.0, 0.0]
    q[i], q[j], q[k] = s / 4, (R[j, i] + R[i, j]) / s, (R[k, i] + R[i, k]) / s
    return (q[0], q[1], q[2], (R[k, j] - R[j, k]) / s)


def _wedge(r0, r1, ro, z0, z1, a0, a1):
    """A sector of a ring as one convex solid: the inner flat has apothem r0 at z0 and r1 at z1 (a funnel if they differ), the outer radius ro."""
    half = (a1 - a0) / 2
    pts = []
    for z, r in ((z0, r0), (z1, r1)):
        ri = r / math.cos(half)
        pts += [(ri * math.cos(a0), ri * math.sin(a0), z), (ri * math.cos(a1), ri * math.sin(a1), z), (ro * math.cos(a1), ro * math.sin(a1), z), (ro * math.cos(a0), ro * math.sin(a0), z)]
    f = [(0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6), (0, 5, 1), (0, 4, 5), (1, 6, 2), (1, 5, 6), (2, 7, 3), (2, 6, 7), (3, 4, 0), (3, 7, 4)]
    return m.Mesh(np.array(pts, float), np.array(f))


def _scaled(mesh, sx, sy):
    return m.Mesh(mesh.v * np.array([sx, sy, 1.0]), mesh.f)


# -- the body: local z along the post (out of the neck), x across the shoulders, y to the front -----------------------------------
def _body() -> m.Part:
    # what is seen: a doll body, lathed and flattened front to back, broad-shouldered and narrow-hipped, in a T-shirt with a collar and
    # shorts with a belt, with round joints at the elbows and knees; within about 2 mm of the boxes below (the colliders) at the neck and
    # hips, 3 mm at the shoulders. Our own model, built here; no outside mesh.
    torso = _scaled(m.lathe([(0.0, -0.101), (0.0155, -0.100), (0.0145, -0.085), (0.0180, -0.060), (0.0225, -0.036), (0.0270, -0.020), (0.0288, -0.011),
                             (0.0230, -0.0045), (0.0100, -0.0018), (0.0, -0.0015)], 28), 1.0, 0.42)
    collar = m.lathe([(0.0055, -0.0042), (0.0088, -0.0042), (0.0088, -0.0016), (0.0055, -0.0016)], 20)
    pelvis = _scaled(m.lathe([(0.0, -0.152), (0.0148, -0.150), (0.0155, -0.125), (0.0158, -0.104), (0.0, -0.103)], 28), 1.0, 0.64)
    belt = _scaled(m.lathe([(0.0, -0.1045), (0.0163, -0.1040), (0.0163, -0.0985), (0.0, -0.098)], 28), 1.0, 0.66)
    neck = m.cylinder(0.0045, 0.0055, (0.0, 0.0, -0.0025))
    parts = [("torso", torso, "shirt"), ("collar", collar, "white"), ("pelvis", pelvis, "pants"), ("belt", belt, "belt"), ("neck", neck, "skin"),
             ("stem", m.cylinder(0.0025, STEM, (0.0, 0.0, STEM / 2)), "plastic")]
    for i, sx in enumerate((-1.0, 1.0)):
        lx, ax_ = sx * 0.0074, sx * 0.0278
        parts += [(f"thigh{i}", m.lathe([(0.0, -0.207), (0.0055, -0.206), (0.0068, -0.180), (0.0072, -0.150), (0.0, -0.149)], 18).moved((lx, 0.0, 0.0)), "skin"),
                  (f"knee{i}", m.sphere(0.0062, (lx, 0.0005, -0.207), 8, 14), "skin"),
                  (f"shin{i}", m.lathe([(0.0, -0.263), (0.0040, -0.262), (0.0045, -0.248), (0.0058, -0.228), (0.0055, -0.209), (0.0, -0.208)], 18).moved((lx, 0.0, 0.0)), "skin"),
                  (f"cuff{i}", m.lathe([(0.0, -0.172), (0.0080, -0.171), (0.0082, -0.150), (0.0, -0.149)], 18).moved((lx, 0.0, 0.0)), "pants"),
                  (f"foot{i}", m.box((0.0085, 0.021, 0.007), (lx, 0.006, -0.2645)), "shoe"),
                  (f"upper{i}", _scaled(m.lathe([(0.0, -0.060), (0.0044, -0.059), (0.0052, -0.035), (0.0058, -0.012), (0.0, -0.008)], 14), 1.0, 1.15).moved((ax_, 0.0, 0.0)), "skin"),
                  (f"elbow{i}", m.sphere(0.0050, (ax_, 0.0, -0.0615), 8, 12), "skin"),
                  (f"fore{i}", _scaled(m.lathe([(0.0, -0.108), (0.0034, -0.107), (0.0040, -0.085), (0.0045, -0.064), (0.0, -0.063)], 14), 1.0, 1.15).moved((ax_, 0.0, 0.0)), "skin"),
                  (f"sleeve{i}", _scaled(m.lathe([(0.0, -0.034), (0.0070, -0.033), (0.0075, -0.012), (0.0, -0.006)], 14), 1.0, 1.1).moved((ax_, 0.0, 0.0)), "shirt"),
                  (f"hand{i}", _scaled(m.sphere(0.0055, (0.0, 0.0, 0.0), 6, 12), 0.6, 1.0).moved((ax_, 0.0015, -0.1145)), "skin")]
    ball = m.sphere(BALL_R, (0.0, 0.0, STEM), 8, 16)
    parts.append(("ball", ball, "plastic"))
    cols = [m.col_box("shoulders_c", (0.052, 0.022, 0.030), (0.0, 0.0, -0.017)), m.col_box("chest_c", (0.034, 0.020, 0.070), (0.0, 0.0, -0.065)), m.col_box("hips_c", (0.030, 0.020, 0.040), (0.0, 0.0, -0.135)),
            m.col_box("neck_c", (0.010, 0.010, 0.004), (0.0, 0.0, -0.001)), m.col_cylinder("stem_c", 0.0025, STEM, (0.0, 0.0, STEM / 2)), m.col_hull("ball_c", ball)]
    cols += [m.col_box(f"leg{i}_c", (0.013, 0.014, 0.125), (sx * 0.008, 0.0, -0.2025)) for i, sx in enumerate((-1.0, 1.0))]
    cols += [m.col_box(f"arm{i}_c", (0.008, 0.012, 0.095), (sx * 0.022, 0.0, -0.065)) for i, sx in enumerate((-1.0, 1.0))]
    # lying face up with the post toward -y: local x -> +x, local y -> +z, local z -> -y
    R = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])
    return m.free("body", (BX, NECK_Y, Z_BODY), parts, cols, LOOKS, mass=M_BODY, friction=(0.5, 0.5), torsional_patch=None, solver_iters=192, start_quat=_quat(R))


# -- the head: local z into the head from the neck hole's mouth, y to the face, x ear to ear -----------------------------------
def _head() -> m.Part:
    prof = [(0.0, 0.009), (0.0135, 0.009), (0.015, 0.016), (0.016, 0.024), (0.0155, 0.031), (0.012, 0.036), (0.0, HEAD_L)]
    cap = _scaled(m.lathe(prof, 24), 15 / 16, 17 / 16)
    shell = _scaled(m.lathe([(BORE_A, 0.0), (0.0115, 0.0), (0.0125, 0.004), (0.0135, 0.009)] + prof[2:], 24), 15 / 16, 17 / 16)
    hair = _scaled(m.lathe([(0.0, 0.0245), (0.0168, 0.0245), (0.0180, 0.0300), (0.0162, 0.0365), (0.0110, 0.0412), (0.0, 0.0428)], 24), 15.4 / 16, 17.4 / 16).moved((0.0, -0.0025, 0.0))
    back_hair = _scaled(m.lathe([(0.0, 0.012), (0.0152, 0.012), (0.0168, 0.020), (0.0172, 0.028), (0.0, 0.034)], 24), 15.8 / 16, 17.2 / 16).moved((0.0, -0.0035, 0.0))
    face = []
    for i, s in enumerate((1.0, -1.0)):
        face += [(f"white{i}", _scaled(m.sphere(0.0029, (0.0, 0.0, 0.0), 6, 10), 1.0, 0.5).moved((s * 0.0057, 0.0158, 0.0215)), "white"),
                 (f"iris{i}", m.sphere(0.0017, (s * 0.0057, 0.0166, 0.0213), 6, 10), "iris"),
                 (f"brow{i}", m.box((0.0062, 0.0016, 0.0016), (s * 0.0058, 0.0166, 0.0258)), "hair"),
                 (f"ear{i}", _scaled(m.sphere(0.0034, (0.0, 0.0, 0.0), 6, 10), 0.45, 0.8).moved((s * 0.0148, -0.0005, 0.0200)), "skin")]
    nose = _scaled(m.sphere(0.0020, (0.0, 0.0, 0.0), 6, 10), 0.8, 0.9).moved((0.0, 0.0166, 0.0180))
    lips = m.box((0.0080, 0.0018, 0.0021), (0.0, 0.0160, 0.0137))
    parts = [("shell", shell, "skin"), ("hair", hair, "hair"), ("back_hair", back_hair, "hair"), ("nose", nose, "skin"), ("lips", lips, "lips")] + face
    cols = [m.col_hull("cap_c", cap)]
    for k in range(NW):
        a0, a1 = 2 * math.pi * k / NW, 2 * math.pi * (k + 1) / NW
        cols.append(m.col_hull(f"fun{k}", _wedge(MOUTH_A, BORE_A, 0.0115, 0.0, FUN_D, a0, a1)))
        cols.append(m.col_hull(f"bore{k}", _wedge(BORE_A, BORE_A, 0.0115, FUN_D, BORE_D, a0, a1)))
    # on its side (face down rolled over onto an ear at reset, 7 mm), the neck hole facing +x: local x -> -z (ear down), local y -> -y (face), local z -> -x
    R = np.array([[0.0, 0.0, -1.0], [0.0, -1.0, 0.0], [-1.0, 0.0, 0.0]])
    return m.free("head", (HX, HY, TOP + 0.015 + 0.0005), parts, cols, LOOKS, mass=M_HEAD, friction=(0.5, 0.5), torsional_patch=float(os.environ.get("DH_TORS", "0.012")), solver_iters=192, start_quat=_quat(R))


# a part pressed into another and let go is pushed out at no more than this (PhysX has no cap by default): a 12 g head let go
# while pressed into the neck can otherwise leave it at metres a second
m.MAX_DEPEN.update(head=0.05, body=0.2)
OBJECTS = {"body": [_body()], "head": [_head()]}.get(os.environ.get("DH_ONLY", ""), [_body(), _head()])
BALL_LOCAL = np.array([0.0, 0.0, STEM])


def rel(body, head):
    """The ball in the head's frame: depth s along the head's axis (m), offset from the axis (m), tilt between the post and the head's axis (deg)."""
    q = head.to_local(body.to_world(BALL_LOCAL))[0]
    zh, zb = head.rotation()[:, 2], body.rotation()[:, 2]
    tilt = math.degrees(math.acos(float(np.clip(zh @ zb, -1.0, 1.0))))
    return float(q[2]), float(math.hypot(q[0], q[1])), tilt


HEAD_MID = np.array([0.0, 0.0, 0.020])
HIPS = np.array([0.0, 0.0, -0.13])


def _gripped(tips, c, r):
    """Some hand has both fingers within r of point c, its jaws closed to under 60 mm (the fingers' centre lines; open, they stand about 100 mm apart)."""
    for k in (0, 12):
        a, b = tips[k:k + 6].mean(0), tips[k + 6:k + 12].mean(0)
        if np.linalg.norm(a - c) < r and np.linalg.norm(b - c) < r and np.linalg.norm(a - b) < 0.060:
            return True
    return False


def _holder(tips, c, r):
    """Which hand ("left", "right") has both fingers within r of point c with its jaws closed to under 60 mm, or ""."""
    for k, side in ((0, "left"), (12, "right")):
        a, b = tips[k:k + 6].mean(0), tips[k + 6:k + 12].mean(0)
        if np.linalg.norm(a - c) < r and np.linalg.norm(b - c) < r and np.linalg.norm(a - b) < 0.060:
            return side
    return ""


def _push_along(env, side, direction):
    """The newtons an arm's joint springs push its TCP along `direction`: the TCP of its joint targets less its observed TCP, over its
    compliance J K^-1 J^T that way (taskspace.py; Arm.press's estimate, 10 to 14 % over a spring scale's reading in the kit's test)."""
    import taskspace as ts
    from yam_arena import ik, station

    robot = env.unwrapped.scene[f"{side}_robot"]
    ids = [robot.find_joints(f"joint{k}")[0][0] for k in range(1, 7)]
    q_o = m._np(robot.data.joint_pos)[0, ids]
    q_t = m._np(robot.data.joint_pos_target)[0, ids]
    base = np.asarray(station.BASES[side], dtype=float)
    dvec = np.asarray(direction, dtype=float) / np.linalg.norm(direction)
    lead = float((ik.tcp_pose(q_t, base)[0] - ik.tcp_pose(q_o, base)[0]) @ dvec)
    return lead / float(dvec @ ts.compliance(q_o, base) @ dvec)


def _seg_dist(p, a, b):
    ab = b - a
    t = np.clip(((p - a) @ ab) / (ab @ ab), 0.0, 1.0)
    return np.linalg.norm(p - (a + t[:, None] * ab), axis=1)


class Trial:
    def reset(self, env=None) -> None:
        self.n, self.held, self.pull = 0, 0, 0
        self.dead, self.score, self.full = False, 0.0, False
        self.last = {}
        self.on = False
        self.tier0 = 0.0
        self.latched, self.pushed = False, 0
        if env is not None:
            self.step(env)

    def step(self, env) -> None:
        if os.environ.get("DH_SPEEDS") and self.n % 10 == 0:  # spawn check: each part's speed, no forces from the scorer
            for k in OBJECTS:
                p = m.state(env, k.name)
                print(f"[dh-speed] step {self.n} {k.name} pos_mm {np.round(p.pos * 1000, 1).tolist()} v_mm_s {np.linalg.norm(p.lin_vel) * 1000:.2f} w_rad_s {np.linalg.norm(p.ang_vel):.3f}", flush=True)
        if os.environ.get("DH_ONLY"):
            self.n += 1
            return
        body, head = m.state(env, "body"), m.state(env, "head")
        s, lat, tilt = rel(body, head)
        zh = head.rotation()[:, 2]
        tips = m.fingertips(env)
        # The lip is a rule on the robot's real push, not a force: a code force updated at 20 Hz on two hand-held parts sets the soft
        # arms ringing and lets the head jump the lip between updates. The bore lets the ball in freely to
        # the neck's stop; the head counts as snapped once it is there (within 2 mm and 14 degrees of the axis) while the hand
        # holding it pushes it along the post at 8 N or more for 3 steps, read from that arm's joint springs as Arm.press reads them.
        v_s = -float((head.lin_vel - body.lin_vel) @ zh)  # how fast the ball goes into the head, m/s (trace only)
        home = bool(lat < 0.002 and tilt < 14.0 and s >= S1 - 0.0003)
        side = _holder(tips, head.to_world(HEAD_MID)[0], 0.035)
        push = _push_along(env, side, -zh) if (home and side) else 0.0
        self.pushed = self.pushed + 1 if push >= F_SNAP else 0
        if self.pushed >= 3:
            self.latched = True
        if s < S1 - 0.0015 or lat > 0.002:
            self.latched = False
        f = F_SEAT if (self.latched and s >= S1 - 0.0015) else 0.0  # snapped: the lip holds the head to the neck, the contact taking the force
        # the 3 N pull runs only on a head that is snapped on: pulling two loose parts apart would fling them
        pulling = self.held >= HOLD_STEPS and self.tier0 == 1.0 and self.pull < PULL_STEPS
        if pulling:
            f -= F_PULL
        if f != 0.0 or self.on:
            # f > 0 draws the head on: the head goes on along -zh (its axis points from the mouth into the head), so it gets -f * zh.
            m.push(env, "head", -f * zh)
            m.push(env, "body", f * zh)
        self.on = f != 0.0
        snapped = bool(self.latched and lat < 0.002 and tilt < 14.0 and s >= S1 - 0.0003)
        resting = bool((not snapped) and lat < 0.002 and tilt < 20.0 and -0.0035 < s < S_STOP + 0.0005)  # on the ball at any depth, home included, not snapped
        tier = 1.0 if snapped else 0.3 if resting else 0.0
        self.dead = bool(body.pos[2] < TOP - 0.01 or head.pos[2] < TOP - 0.01)
        clear = bool(_seg_dist(tips, head.pos, head.pos + zh * HEAD_L).min() >= 0.046 and _seg_dist(tips, body.pos, body.pos - body.rotation()[:, 2] * 0.265).min() >= 0.056)
        still = all(np.linalg.norm(x.lin_vel) < 0.005 and np.linalg.norm(x.ang_vel) < 0.3 for x in (body, head))
        if self.pull == 0:
            self.held = self.held + 1 if (clear and still) else 0
            self.tier0 = tier
            self.score = tier if (self.held >= HOLD_STEPS and tier < 1.0) else 0.0  # resting (0.3) or nothing, hands clear and still 1 s; 1.0 only after the pull
        if self.held >= HOLD_STEPS and self.tier0 == 1.0:
            self.pull += 1
        if self.pull >= PULL_STEPS:
            self.score = min(tier, self.tier0)
        if self.dead:
            self.score = 0.0
        self.full = bool(self.pull >= PULL_STEPS and self.score == 1.0)
        self.n += 1
        self.last = dict(s_mm=round(s * 1000, 2), v_mm_s=round(v_s * 1000, 1), push=round(push, 1), hp=round(math.degrees(math.asin(float(np.clip(zh[2], -1, 1)))), 1), bp=round(math.degrees(math.asin(float(np.clip(body.rotation()[2, 2], -1, 1)))), 1), lat_mm=round(lat * 1000, 2), tilt=round(tilt, 1), f=round(f, 2), side=side, latched=self.latched, snapped=snapped, resting=resting, clear=clear, still=still, held=self.held, pull=self.pull, tier=tier)
        if self.n % 20 == 0 and os.environ.get("DH_TRACE"):
            print(f"[dh] step {self.n}: {self.last}", flush=True)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    """The head snapped on (the ball past the hole's lip, within 2 mm of the axis and 14 degrees), both grippers 3 cm clear and all still for 1 s, then a 3 N pull along the neck for 1 s that it survives."""
    return TRIAL.full


def score(objects) -> float:
    return TRIAL.score


# -- the demo ---------------------------------------------------------------------------------------------------------------------

GRIP_N = float(os.environ.get("DH_GRIPN", "35"))
PRESS_N = float(os.environ.get("DH_PRESSN", "13"))
LIFT = float(os.environ.get("DH_LIFT", "0.03"))
HGAP = 0.012  # the mouth's gap before the ball when lined up


def _hips(ctx):
    b = ctx.state("body")
    return b.to_world([0.0, 0.0, -0.13])[0]


def _mid(ctx):
    h = ctx.state("head")
    return h.to_world([0.0, 0.0, 0.020])[0]


def _press(arm, direction, force, max_travel, until, speed=0.02, settle=0.5, timeout=10.0, max_lead=0.25, steer=None):
    """demokit's Arm.press (the same differential-IK step, command lead and force estimate), which also ends as soon as until() is true.
    steer(), if given, returns a world rotation vector (rad) that would turn the held part onto its line: the hand's held orientation turns
    by up to 1.5 degrees a step toward it, so a part pitching in the pinch is steered back by its own pose."""
    import taskspace as ts
    from yam_arena import ik

    dvec = np.asarray(direction, dtype=float) / np.linalg.norm(direction)
    ctl = ts.DiffIK()
    p0_obs = arm.tcp().copy()
    p0_cmd, r_hold = ik.tcp_pose(arm.q, arm.base)
    s_cmd, travel, f_est, good, why, lead = 0.0, 0.0, 0.0, 0, "timeout", 0.0
    for _ in range(max(1, round(timeout * d.CONTROL_HZ))):
        if until():
            why = "until"
            break
        q_obs = arm.ctx.observed(arm.side)[:6]
        travel = float((ik.tcp_pose(q_obs, arm.base)[0] - p0_obs) @ dvec)
        if travel >= max_travel:
            why = "travel"
            break
        c_dd = float(dvec @ ts.compliance(q_obs, arm.base) @ dvec)
        lead = min(force * c_dd, max_lead)
        s_cmd = min(s_cmd + speed * d.DT, travel + lead)
        f_est = max(0.0, s_cmd - travel) / c_dd
        good = good + 1 if f_est >= 0.9 * force else 0
        if good >= settle * d.CONTROL_HZ:
            why = "force"
            break
        if steer is not None:
            w = np.asarray(steer(), dtype=float)
            n = float(np.linalg.norm(w))
            if n > 1e-4:
                r_hold = d.axis_angle_matrix(w / n, min(0.5 * n, math.radians(1.5))) @ r_hold
        for _ in range(2):
            p_cmd, r_cmd = ik.tcp_pose(arm.q, arm.base)
            arm.q = ctl.step(arm.q, arm.base, p0_cmd + dvec * s_cmd - p_cmd, ik._rot_error(r_hold, r_cmd))
        arm.cmd = np.r_[arm.q, float(np.clip(arm.finger / d.FINGER_OPEN, 0.0, 1.0))]
        yield
    print(f"[demokit] {arm.side} 'press': {why}, travel {travel * 1000:.1f} mm, about {f_est:.1f} N, lead {lead * 1000:.1f} mm", flush=True)
    return {"why": why, "travel": travel, "force_est": f_est, "lead": lead}


def demo_script(ctx):
    """Modes: full. none: the arms wait. unheld: the right hand does it all, the body left lying. crooked: the head pushed on 20 degrees off the post's axis. light: the whole demo at a 4 N push (under the lip's 8 N). unseated: pushed 3 mm and let go (on the ball,
    not past the lip). left_only: the left hand holds the body and nothing else."""
    mode = ctx.mode
    if mode not in ("full", "none", "unheld", "crooked", "unseated", "light", "left_only", "pick", "align"):
        raise ValueError(f"unknown mode {mode!r}")
    L, R = ctx.left, ctx.right
    if mode == "none":
        yield from L.wait(3.0)
        return
    if mode != "unheld":
        yield from L.pick(lambda: _hips(ctx) + np.array([0.0, 0.0, 0.006 - 0.011]) + np.array([0.0, 0.0, 0.011]), 0.0, width=0.030, grip=d.down(-90), above=0.10, lift=LIFT, force=GRIP_N, label="hold body")
    if mode == "left_only":
        yield from L.wait(4.0)
        return
    # the head: picked at its middle, the jaws across its 30 mm, then a quarter turn so the hole faces the post
    yield from R.pick(lambda: _mid(ctx) + np.array([0.0, 0.0, 0.003]), 0.0, width=0.034, grip=d.down(0), above=0.10, lift=0.0, force=GRIP_N, label="pick head")
    b = lambda: ctx.state("body")
    tip = lambda: b().to_world([0.0, 0.0, POST_L])[0]
    ax = lambda: b().rotation()[:, 2]  # the post's direction, out of the neck
    mouth = lambda: ctx.state("head").pos
    goal = lambda gap: tip() + ax() * gap
    yield from R.move(R.at() + np.array([0.0, 0.0, 0.035 + LIFT]), t=1.0, label="lift head")
    yield from R.arc(_mid(ctx), (0.0, 0.0, 1.0), 90.0, label="turn hole to post")
    if mode == "pick":
        yield from R.wait(1.0)
        print(f"[dh] picked: {TRIAL.last}", flush=True)
        return
    over = lambda: goal(0.040) + np.array([0.0, 0.0, 0.0])
    yield from R.servo(over(), timeout=3.0, read=mouth, tol=0.0008, label="head to the post")
    for _ in range(2):  # level by feedback: turn the head's axis onto the post's, about its middle
        zh, a = ctx.state("head").rotation()[:, 2], ax()
        c = np.cross(zh, a)
        ang = math.degrees(math.atan2(float(np.linalg.norm(c)), float(zh @ a)))
        print(f"[dh] head axis {ang:.1f} deg off the post's", flush=True)
        if ang < 1.0:
            break
        yield from R.arc(_mid(ctx), c / np.linalg.norm(c), ang, t=max(0.5, ang / 30.0), label="level head")
        yield from R.servo(over(), timeout=1.5, read=mouth, tol=0.0008, label="head to the post")
    if mode == "crooked":
        yield from R.arc(_mid(ctx), (1.0, 0.0, 0.0), 20.0, t=0.6, label="tilt head")
    yield from R.servo(goal(HGAP), timeout=3.0, read=mouth, tol=0.0004, label="mouth on the axis")
    for _ in range(2):
        yield from R.servo(goal(HGAP), timeout=1.5, read=mouth, tol=0.0003, label="mouth on the axis")
    print(f"[dh] lined up: {TRIAL.last} mouth-axis err {np.linalg.norm((mouth() - goal(HGAP)) * 1000):.2f} mm", flush=True)
    if mode == "align":
        yield from R.wait(1.0)
        return
    # the harness's arm is soft along +y here (about 10 to 16 mm of command lead a newton), so the press may lead the hand by up to 25 cm.
    # It stops once the two poses show the ball past the lip: pushed on against the neck, the pinched head pitches and levers off.
    travel = HGAP + 0.003 if mode == "unseated" else HGAP + POST_L + 0.100  # the held body gives way along the post: the hand travels on as the push builds
    push = 4.0 if mode == "light" else PRESS_N
    seated = lambda: False  # the press runs on to its force against the neck: the snap needs 8 N there
    def steer():  # the turn that puts the head's axis on the post's, read from the two poses
        zh, a = ctx.state("head").rotation()[:, 2], ax()
        c = np.cross(zh, a)
        n = float(np.linalg.norm(c))
        return c / n * math.atan2(n, float(zh @ a)) if n > 1e-9 else np.zeros(3)

    res = yield from _press(R, -ax(), push, travel, until=seated, speed=float(os.environ.get("DH_SPEED", "0.012")), timeout=22.0, steer=None if mode == "crooked" else steer)
    print(f"[dh] pressed {res}: {TRIAL.last}", flush=True)
    from yam_arena import ik
    R.grip = d._as_grip(ik.tcp_pose(R.q, R.base)[1])  # keep the orientation the press steered to: the old grip would turn the head back off the ball
    # ease off before opening: the press leaves its command up to 25 cm ahead of the hand, which open fingers would lunge to;
    # keep 5 mm of it (a light push on the seated head)
    yield from R.move(R.tcp() - R.trim - ax() * 0.005, t=0.8, label="ease off")
    print(f"[dh] eased off: {TRIAL.last}", flush=True)
    yield from R.open(t=1.5, label="let go head")
    yield from R.move(R.at() + np.array([0.0, 0.0, 0.08]), t=1.0, label="up")
    yield from R.move(R.at() + np.array([0.0, -0.12, 0.0]), t=1.0, label="away")
    if mode != "unheld":
        # down until the head (16 mm from the neck's axis to the hair) is about 1 mm over the bench: lying flat, the 22 mm chest would put it 5 mm into it
        yield from L.move(L.at() + np.array([0.0, 0.0, -LIFT + 0.006]), t=1.0, label="set body down")
        yield from L.open(t=1.0, label="let go body")
        yield from L.move(L.at() + np.array([0.0, 0.0, 0.10]), t=1.0, label="up")
        yield from L.move(L.at() + np.array([0.0, 0.12, 0.0]), t=1.0, label="away")
    yield from R.wait(4.0)
    print(f"[dh] end {TRIAL.last}", flush=True)
