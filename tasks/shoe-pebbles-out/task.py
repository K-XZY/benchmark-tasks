"""Get the small stones out of a running shoe: hold it by the heel, turn it opening-down with the toe up, shake until all 8 are out, set it back down sole-first.

A men's US 9 road running shoe (modelled on the Nike Pegasus 41, nike.com/t/pegasus-41-mens-road-running-shoes-LMhfRGdO: 297 g, 10 mm heel-to-toe drop, 37/27 mm stack; 290 mm long, collar opening about 100 x 67 mm), one rigid free
body whose shell is a compound of convex pieces with walls 4.5 mm thick (the real mesh upper is thinner; thin walls let 4 mm stones tunnel): sole, two side walls, a closed vamp over the toe box, a solid rigid heel counter 12 mm thick and 120 mm tall at the back (the real counter is foam-padded; one rigid body here). Eight gravel pebbles,
4 to 8 mm, lie inside it, most in the toe box under the vamp. The right hand pinches the heel counter from the side, one finger inside the collar and one behind the heel (fingers leaned 10 degrees off level), lifts the shoe, turns it toe-up
past vertical so the opening faces down, shakes it along its length and sets it back down.

    python demokit.py task.py OUT --no-record --mode full,none,collar_up,heel_up,on_side,drop,drop_low,drop_25mm,toe_first,wide_toe_first,relift_3cm,collar_hook_drop
    python demokit.py task.py OUT          (final: 640 x 360, the GPU to yourself)
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
INSTRUCTION = "There are some small stones in this shoe. Get them all out, then put the shoe back down on the table, sole down."
SUCCESS_HOLD_S = 0.0  # the scorer holds the end state itself (2 s, counted in control steps)
SCORE_OVER = "end"
TOP = d.TABLE_TOP

# -- the shoe, in its own frame: x from the back of the heel (0) to the toe tip, y across, z up from the sole's underside ---------------------------------------------------------
L = 0.290
T = 0.0045  # upper wall thickness
BACK = 0.012  # the solid rigid heel counter (the real one is foam-padded) at the back, solid from the sole to its top edge
THROAT = 0.115  # the collar opening runs from BACK to here; the vamp and tongue close the shoe in front of it
CROWN = 0.008  # the vamp's rise from its sides to its centre line
SHOE_M = 0.297
# key sections: (x, outer half-width, top of the upper: the collar edge behind THROAT, the vamp's centre line in front of it)
KEY = [(0.000, 0.022, 0.120), (0.012, 0.032, 0.118), (0.030, 0.037, 0.100), (0.060, 0.038, 0.090), (0.090, 0.037, 0.089), (0.115, 0.037, 0.092),
       (0.140, 0.040, 0.091), (0.170, 0.045, 0.084), (0.200, 0.050, 0.075), (0.230, 0.050, 0.066), (0.255, 0.045, 0.058), (0.272, 0.036, 0.051),
       (0.283, 0.024, 0.045), (0.290, 0.010, 0.036)]
KX = np.array([k[0] for k in KEY])
KW = np.array([k[1] for k in KEY])
KT = np.array([k[2] for k in KEY])
STATIONS = np.unique(np.r_[KX, np.arange(0.015, 0.28, 0.015)])


def half_w(x):
    return np.interp(x, KX, KW)


def top_z(x):
    return np.interp(x, KX, KT)


def insole_z(x):
    """The insole's top: 37 mm stack at the heel falling to 27 mm under the forefoot."""
    return np.interp(x, [0.0, 0.06, 0.20, L], [0.037, 0.037, 0.027, 0.027])


def _hull_mesh(pts) -> m.Mesh:
    """A convex piece's surface from its 8 corners: [4 at the back section, 4 at the front], each loop going round the section."""
    p = np.asarray(pts, float)
    f = [(0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6)]
    for i in range(4):
        j = (i + 1) % 4
        f += [(i, j, 4 + j), (i, 4 + j, 4 + i)]
    f = np.array(f)
    c = p.mean(axis=0)
    for k, (a, b, e) in enumerate(f):
        n = np.cross(p[b] - p[a], p[e] - p[a])
        if np.dot(n, (p[a] + p[b] + p[e]) / 3 - c) < 0:
            f[k] = (a, e, b)
    return m.Mesh(p, f)


def _quad(x, y0, y1, z0, z1):
    return [(x, y0, z0), (x, y1, z0), (x, y1, z1), (x, y0, z1)]


def shoe_pieces():
    """(name, corners, look) for every convex piece of the shell."""
    out = []
    for i in range(len(STATIONS) - 1):
        a, b = STATIONS[i], STATIONS[i + 1]
        wa, wb, za, zb, ta, tb = half_w(a), half_w(b), insole_z(a), insole_z(b), top_z(a), top_z(b)
        out.append((f"out{i}", _quad(a, -wa, wa, 0.0, 0.006) + _quad(b, -wb, wb, 0.0, 0.006), "outsole"))
        out.append((f"mid{i}", _quad(a, -wa, wa, 0.006, za) + _quad(b, -wb, wb, 0.006, zb), "midsole"))
        if b <= BACK + 1e-9 or a >= 0.283 - 1e-9:  # the heel counter and the toe cap: solid
            out.append((f"end{i}", _quad(a, -wa, wa, za - 0.002, ta) + _quad(b, -wb, wb, zb - 0.002, tb), "upper"))
            continue
        closed = a >= THROAT - 1e-9
        sa, sb = (ta - CROWN, tb - CROWN) if closed else (ta, tb)
        for s, nm in ((1.0, "l"), (-1.0, "r")):
            out.append((f"wall{nm}{i}", _quad(a, s * wa, s * (wa - T), za - 0.003, sa) + _quad(b, s * wb, s * (wb - T), zb - 0.003, sb), "upper"))
            if closed:  # the vamp: two slabs rising from the sides to the centre line
                pa = [(a, 0.0, ta - T), (a, s * wa, sa - T), (a, s * wa, sa), (a, 0.0, ta)]
                pb = [(b, 0.0, tb - T), (b, s * wb, sb - T), (b, s * wb, sb), (b, 0.0, tb)]
                out.append((f"vamp{nm}{i}", pa + pb, "upper"))
    return out


def inside_shoe(p_local) -> np.ndarray:
    """Whether points (shoe frame) lie in the shoe: within its outer envelope and above the sole (the cavity's floor, the insole, is 27 to 37 mm up; a stone on the table under
    the sole's edge is not in). Over the collar opening the envelope reaches 1 cm above the edge, so a stone on the lip is still in."""
    p = np.atleast_2d(p_local)
    x, y, z = p[:, 0], p[:, 1], p[:, 2]
    lid = np.where(x < THROAT, 0.010, 0.0)
    return (x >= 0.0) & (x <= L) & (np.abs(y) <= half_w(x)) & (z >= 0.010) & (z <= top_z(x) + lid)


def gap_to_shoe(p_local) -> np.ndarray:
    """A lower bound on each point's distance from the shoe's outer envelope (0 inside it)."""
    p = np.atleast_2d(p_local)
    x, y, z = p[:, 0], p[:, 1], p[:, 2]
    xc = np.clip(x, 0.0, L)
    top = top_z(xc)
    g = np.stack([-x, x - L, np.abs(y) - half_w(xc), -z, z - top], axis=1)
    return np.maximum(g.max(axis=1), 0.0)


# -- the pebbles: irregular hulls 4 to 8 mm across, quartz 2.65 g/cm3 ---------------------------------------------------------------------------------------------------------------
N_PEB = 8
SIZES = [0.0080, 0.0074, 0.0068, 0.0062, 0.0056, 0.0050, 0.0045, 0.0041]  # longest axis
RHO = 2650.0
# where each starts in the shoe (x, y): 4 in the toe box, 2 at the midfoot, 2 at the heel
START = [(0.238, -0.013), (0.242, 0.015), (0.256, 0.001), (0.222, 0.002), (0.160, -0.011), (0.172, 0.013), (0.050, 0.010), (0.072, -0.010)]


def pebble_mesh(k: int) -> m.Mesh:
    rng = np.random.default_rng(100 + k)
    a = SIZES[k] / 2
    ax = np.array([a, a * rng.uniform(0.65, 0.85), a * rng.uniform(0.45, 0.65)])
    s = m.sphere(1.0, rings=6, segments=10)
    v = s.v / np.linalg.norm(s.v, axis=1, keepdims=True)
    bump = 1.0 + 0.12 * np.sin(3.1 * v[:, 0] + rng.uniform(0, 6)) * np.cos(2.3 * v[:, 1] + rng.uniform(0, 6))
    return m.Mesh(v * ax * bump[:, None], s.f)


def pebble_mass(k: int) -> float:
    mesh = pebble_mesh(k)
    v, f = mesh.v, mesh.f
    vol = abs(float(np.einsum("ij,ij->i", v[f[:, 0]], np.cross(v[f[:, 1]], v[f[:, 2]])).sum()) / 6.0)
    return RHO * vol


# -- the scene ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
GRIP_XY, SHOE_YAW = (0.33, -0.08), 60.0  # where the heel counter stands, and the shoe's heading (heel to toe) from +x
SHOE_X = GRIP_XY[0] - (BACK / 2) * math.cos(math.radians(SHOE_YAW))
SHOE_Y = GRIP_XY[1] - (BACK / 2) * math.sin(math.radians(SHOE_YAW))
LOOKS = {"outsole": m.Look(m.rgb(0.10, 0.10, 0.11), roughness=0.9), "midsole": m.Look(m.rgb(0.93, 0.93, 0.90), roughness=0.7),
         "upper": m.Look(m.rgb(0.16, 0.36, 0.78), roughness=0.75), "lace": m.Look(m.rgb(0.96, 0.96, 0.96), roughness=0.8),
         "collar": m.Look(m.rgb(0.12, 0.12, 0.14), roughness=0.8), "stone": m.Look(m.rgb(0.55, 0.50, 0.44), roughness=0.9)}


def _laces():
    parts = []
    for k, x in enumerate((0.128, 0.143, 0.158, 0.173, 0.188)):
        z = top_z(x) + 0.0015
        w = half_w(x) * 0.55
        parts.append((f"lace{k}", m.tube([(x, -w, z - 0.002), (x, 0.0, z), (x, w, z - 0.002)], 0.0018, sides=6), "lace"))
    for s, nm in ((1.0, "l"), (-1.0, "r")):  # the collar's dark lining along both edges of the opening
        xs = np.linspace(BACK, THROAT, 8)
        parts.append((f"collar{nm}", m.tube([(x, s * (half_w(x) - T / 2), top_z(x) + 0.001) for x in xs], 0.0032, sides=6), "collar"))
    return parts


def _shoe() -> m.Part:
    pieces = shoe_pieces()
    parts = [(n, _hull_mesh(p), look) for n, p, look in pieces] + _laces()
    cols = [m.col_hull(f"c_{n}", _hull_mesh(p)) for n, p, _ in pieces]
    return m.free("shoe", (SHOE_X, SHOE_Y, TOP + 0.0005), parts, cols, LOOKS, mass=SHOE_M, friction=(0.6, 0.5), torsional_patch=0.015, solver_iters=48,
                  yaw_deg=SHOE_YAW, contact_offset=0.002, ccd=True)


def shoe_to_world(p_local) -> np.ndarray:
    c, s = math.cos(math.radians(SHOE_YAW)), math.sin(math.radians(SHOE_YAW))
    R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return np.array([SHOE_X, SHOE_Y, TOP + 0.0005]) + R @ np.asarray(p_local, float)


def _pebble(k: int) -> m.Part:
    x, y = START[k]
    mesh = pebble_mesh(k)
    lift = float(-mesh.v[:, 2].min()) + 0.0015
    pos = shoe_to_world((x, y, float(insole_z(x)) + lift))
    return m.free(f"pebble{k}", tuple(pos), [("body", mesh, "stone")], [m.col_hull("c", mesh)], LOOKS, mass=pebble_mass(k), friction=(0.5, 0.45), torsional_patch=None,
                  solver_iters=32, yaw_deg=37.0 * k, contact_offset=0.0015, ccd=True)


OBJECTS = [_shoe()] + [_pebble(k) for k in range(N_PEB)]


def scene_ccd() -> bool:
    """Turn on the physics scene's CCD flag on the live stage, which the bodies' own CCD (authored in their USD) needs. Returns whether a scene was found."""
    import omni.usd
    from pxr import Sdf

    stage = omni.usd.get_context().get_stage()
    found = False
    for prim in stage.Traverse():
        if prim.GetTypeName() == "PhysicsScene":
            prim.CreateAttribute("physxScene:enableCCD", Sdf.ValueTypeNames.Bool).Set(True)
            found = True
    return found

# -- the scorer -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
HOLD_STEPS = 40  # 2 s at 20 Hz
CLEAR_M = 0.03
LEVEL_DEG = 20.0
TIP_Z = (0.12, 0.134, 0.148)  # fingertip pad heights along the gripper axis from the housing origin: the TCP is at 0.134
MATCH_V = 0.1  # m/s: a fingertip counts as holding only while it slips less than this over the shoe's surface (a finger that merely sits in the collar and lets the shoe slide does not)
DT = 0.05  # s per scorer step (20 Hz)
HELD_M = 0.003  # both fingertips within 3 mm of the shoe
RELEASE_UP = 0.015  # m: no point of the sole may be higher than this above the table when the hand lets go
DROP_SPEED = 0.54  # m/s, with no hand closed on the shoe: what a free fall of 1.5 cm reaches (sqrt(2 g 0.015) = 0.54)
TABLE_X, TABLE_Y = (0.0, 0.85), (-0.60, 0.60)  # the table top's extent in the station frame (estimate, larger than the reach)


def opening(env, side: str) -> float:
    robot = env.unwrapped.scene[f"{side}_robot"]
    return abs(float(m._np(robot.data.joint_pos)[0, robot.find_joints("joint7")[0][0]]))


def gripper_points(env) -> np.ndarray:
    """Points filling both grippers: fingers, pads, palm and housing, and the wrist flange, as they are now (station frame). (2 x 165, 3): the left's, then the right's."""
    base = env.unwrapped
    origin = m._np(base.scene.env_origins[0])
    out = []
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        i = robot.find_bodies("gripper")[0][0]
        pose = m._np(robot.data.body_link_pose_w)[0, i]
        R = m.quat_matrix(pose[3:7])
        opening = abs(float(m._np(robot.data.joint_pos)[0, robot.find_joints("joint7")[0][0]]))
        g = np.stack(np.meshgrid(np.linspace(-(opening + 0.012), opening + 0.012, 5), np.linspace(-0.025, 0.025, 3), np.linspace(-0.04, 0.16, 11), indexing="ij"), -1).reshape(-1, 3)
        out.append(pose[:3] - origin + g @ R.T)
    return np.vstack(out)


def fingertip_points(env) -> np.ndarray:
    """The two finger pads of each hand, as 3 points each at the TCP: (2 hands x 2 fingers x 3, 3) in the station frame, left hand first."""
    base = env.unwrapped
    origin = m._np(base.scene.env_origins[0])
    out = []
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        i = robot.find_bodies("gripper")[0][0]
        pose = m._np(robot.data.body_link_pose_w)[0, i]
        R = m.quat_matrix(pose[3:7])
        opening = abs(float(m._np(robot.data.joint_pos)[0, robot.find_joints("joint7")[0][0]]))
        for sx in (-1.0, 1.0):
            for dz in TIP_Z:
                out.append(pose[:3] - origin + R @ np.array([sx * opening, 0.0, dz]))
    return np.array(out)


def tier(n_out: int, set_down: bool) -> float:
    if n_out >= N_PEB and set_down:
        return 1.0
    if n_out >= 6:
        return 0.75
    if n_out >= 4:
        return 0.5
    if n_out >= 2:
        return 0.25
    return 0.0


class Trial:
    def reset(self, env=None):
        self.n = 0
        self.void = False
        self.why = ""
        self.ok_steps = 0
        self.n_out = 0
        self.set_down = False
        self.peak_v = 0.0
        self.free_steps = 0
        self.was_held = False
        self.prev_loc = None
        self.ever_held = False
        self.held_z = 0.0
        self.last = {}

    def free_air(self, held, high):
        """After a hold: free of both grippers for 2 steps with a sole point more than RELEASE_UP above the table voids (let go in the air, or tipped over after release)."""
        self.free_steps = 0 if held else self.free_steps + 1
        if self.n > 10 and self.ever_held and self.free_steps >= 2 and high > RELEASE_UP and not self.void:
            self.void, self.why = True, f"shoe let go {high:.3f} m above the table"

    def track_hold(self, held: bool, height: float):
        """While held, remember the height of the sole's highest sampled point above the table; on the first step the hand no longer holds it, void if that height was over RELEASE_UP."""
        if held:
            self.held_z = height
        elif self.was_held and self.held_z > RELEASE_UP and self.n > 10 and not self.void:
            self.void, self.why = True, f"shoe let go {self.held_z:.3f} m above the table"
        self.was_held = held
        self.ever_held = self.ever_held or held

    def step(self, env):
        self.n += 1
        if self.n == 1:
            print(f"[shoe] scene CCD on: {scene_ccd()}", flush=True)
        shoe = m.state(env, "shoe")
        R = shoe.rotation()
        peb = np.array([m.state(env, f"pebble{k}").pos for k in range(N_PEB)])
        loc = (peb - shoe.pos) @ R
        self.n_out = int((~inside_shoe(loc)).sum())
        v = float(np.linalg.norm(shoe.lin_vel))
        gaps = gap_to_shoe((gripper_points(env) - shoe.pos) @ R)
        tg = gap_to_shoe((fingertip_points(env) - shoe.pos) @ R).reshape(2, 2, len(TIP_Z))  # (hand, finger, pad point)
        tips = fingertip_points(env)
        loc_tips = (tips - shoe.pos) @ R  # each fingertip point in the shoe's own frame: it stays put while the shoe is held
        slip = np.linalg.norm(loc_tips - self.prev_loc, axis=1) / DT if self.prev_loc is not None else np.zeros(len(tips))
        self.prev_loc = loc_tips
        moving_with = (slip < MATCH_V).reshape(2, 2, len(TIP_Z))
        held = any(bool(((tg[k] < HELD_M) & moving_with[k]).any(axis=1).all()) for k in range(2))  # both fingertips of one hand on the shoe and moving with it
        corners = (np.array([[0.0, -0.03, 0.0], [0.0, 0.03, 0.0], [L, -0.01, 0.0], [L, 0.01, 0.0], [0.20, -0.05, 0.0], [0.20, 0.05, 0.0]]) @ R.T + shoe.pos)[:, 2]
        self.track_hold(held, float(corners.max() - TOP))
        if self.n > 10:  # after the harness's settle
            if not held:
                self.peak_v = max(self.peak_v, v)
            if v > DROP_SPEED and not held and not self.void:
                self.void, self.why = True, f"shoe moved at {v:.2f} m/s with no hand closed on it (dropped or thrown)"
        on_table = TABLE_X[0] < shoe.pos[0] < TABLE_X[1] and TABLE_Y[0] < shoe.pos[1] < TABLE_Y[1] and shoe.pos[2] > TOP - 0.05
        if not on_table and self.n > 10 and not self.void:
            self.void, self.why = True, "shoe off the table"
        level = float(np.degrees(np.arccos(np.clip(R[2, 2], -1.0, 1.0))))
        self.free_air(held, float(corners.max() - TOP))
        resting = bool(corners.min() > TOP - 0.004 and corners.max() < TOP + 0.012 and np.linalg.norm(shoe.lin_vel) < 0.02)
        gap = float(gaps.min())
        open_ = all(opening(env, s) > 0.025 for s in ("left", "right"))
        self.set_down = bool(level <= LEVEL_DEG and resting and gap >= CLEAR_M and open_)
        full = self.n_out >= N_PEB and self.set_down
        self.ok_steps = self.ok_steps + 1 if full else 0
        self.last = {"n": self.n, "out": self.n_out, "level": round(level, 1), "resting": resting, "gap": round(gap, 3), "open": open_, "v": round(v, 3), "peak_v_unheld": round(self.peak_v, 3), "held": held, "held_z": round(self.held_z, 3), "ax": [round(float(R[2, 0]), 2), round(float(R[2, 1]), 2)], "pos": [round(float(c), 3) for c in shoe.pos],
                     "ok": self.ok_steps, "void": self.why, "tier": self.score()}
        if self.n % 20 == 0:
            loc_r = [[round(float(c) * 1000) for c in q] for q in loc]
            print("[shoe] " + json.dumps({"mode": os.environ.get("TASK_DEMO", ""), **self.last, "peb_mm": loc_r}), flush=True)

    def score(self) -> float:
        if self.void:
            return 0.0
        t = tier(self.n_out, False)
        if self.n_out >= N_PEB and self.set_down and self.ok_steps >= HOLD_STEPS:
            return 1.0
        return max(t, 0.75) if self.n_out >= N_PEB else t


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects) -> float:
    return TRIAL.score()


def success(objects) -> bool:
    return TRIAL.score() >= 1.0


# -- the scripted demo ------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TILT = float(os.environ.get("SHOE_TILT", "135"))
TILT2 = float(os.environ.get("SHOE_TILT2", "162"))  # degrees the toe is raised through, past vertical
SHAKES = int(os.environ.get("SHOE_SHAKES", "8"))
AMP = float(os.environ.get("SHOE_AMP", "0.025"))
HALF = float(os.environ.get("SHOE_HALF", "0.25"))  # seconds per stroke
GRIP_Z = float(os.environ.get("SHOE_GRIPZ", "0.114"))  # TCP height on the heel counter, above the sole's underside
GRIP_TH = float(os.environ.get("SHOE_TH", "80"))  # degrees the fingers lean from straight down towards the across-shoe direction (90: level)
PINCH_N = float(os.environ.get("SHOE_PINCH", "40"))  # N a finger
LIFT = float(os.environ.get("SHOE_LIFT", "0.18"))
ROLL = float(os.environ.get("SHOE_ROLL", "40"))  # degrees
SETDZ = float(os.environ.get("SHOE_SETDZ", "0.002"))
TOE_DZ = float(os.environ.get("SHOE_TOEDZ", "0.035"))  # toe_first lowering
TOE_ANG = float(os.environ.get("SHOE_TOEANG", "-8"))  # toe_first pitch, degrees
COMP = float(os.environ.get("SHOE_COMP", "0"))  # degrees the hand turns the heel down after the toe touches: the shoe hangs toe-down in the pinch


def demo_script(ctx):
    """Modes: full. none: the arms wait. collar_up: lift and shake upright. heel_up: turn the opening down with the toe kept low, and shake. on_side: empty it, then lay it on its side.
    drop: empty it, then open the hand 10 cm above the set-down point. drop_low: the same at 3 cm. drop_25mm: at 2.5 cm. toe_first: empty it, roll it level, then set the toe down first with the heel about 4 cm up and let go. wide_toe_first: empty it and set it down as in full, then grip the heel across its sides (opening 7.5 cm), lift 7 cm, pitch the toe down 8 degrees and let go. relift_3cm: the full run, then pinch again, lift 3 cm and let go."""
    mode = ctx.mode
    if mode not in ("full", "none", "collar_up", "heel_up", "on_side", "drop", "drop_low", "drop_25mm", "toe_first", "wide_toe_first", "relift_3cm", "collar_hook_drop"):
        raise ValueError(f"unknown mode {mode!r}")
    R_ = ctx.right
    if mode == "none":
        yield from R_.wait(3.0)
        return
    c, s = math.cos(math.radians(SHOE_YAW)), math.sin(math.radians(SHOE_YAW))
    along = np.array([c, s, 0.0])  # heel to toe
    axis = np.cross(along, [0.0, 0.0, 1.0])  # across the shoe: a positive turn about this raises the toe
    grip = d.tilted(GRIP_TH, SHOE_YAW - 90.0)  # fingers level, pointing across the shoe, closing along its length: one in front of the heel counter's rim, one behind it
    g0 = shoe_to_world((BACK / 2, 0.0, GRIP_Z))
    yield from R_.move(g0 - 0.07 * axis + [0.0, 0.0, 0.03], grip=grip, finger=d.FINGER_OPEN, label="beside the heel")
    yield from R_.move(g0 - 0.07 * axis, t=0.8, label="level with the heel counter")
    yield from R_.move(g0, t=1.2, label="astride the heel counter")
    yield from R_.close(BACK, PINCH_N, label="pinch the heel counter")
    yield from R_.move(R_.at() + [0.0, 0.0, LIFT], t=1.5, label="lift")
    tilt = {"collar_up": 0.0, "heel_up": -100.0}.get(mode, TILT)
    if tilt:
        yield from R_.arc(R_.at(), axis, tilt, t=abs(tilt) / 40.0, label="turn the toe up, opening down")
        yield from R_.wait(0.5)
    stroke = along * math.cos(math.radians(tilt)) + np.array([0.0, 0.0, math.sin(math.radians(tilt))])  # the shoe's length, as it is held now
    home = R_.at().copy()
    for rnd in range(2):
        if rnd == 1 and tilt > 0:  # the last stones catch on the heel counter and the inner finger: turn on until the opening faces straight down, and shake again
            yield from R_.arc(R_.at(), axis, TILT2 - tilt, t=1.0, label="opening straight down")
            tilt = TILT2
            stroke = along * math.cos(math.radians(tilt)) + np.array([0.0, 0.0, math.sin(math.radians(tilt))])
            home = R_.at().copy()
        for k in range(SHAKES):
            yield from R_.move(home + AMP * stroke, t=HALF, label="shake")
            yield from R_.move(home - AMP * stroke, t=HALF, label="shake")
        if rnd == 1:  # a stone jammed between the inner finger and the heel counter: stroke across the shoe too
            for _ in range(3):
                yield from R_.move(home + 0.02 * axis, t=HALF)
                yield from R_.move(home - 0.02 * axis, t=HALF)
        yield from R_.move(home, t=HALF)
        yield from R_.wait(0.6)
    if mode == "wide_toe_first":  # shake on until nothing is left in
        for _ in range(4):
            yield from R_.move(home + 0.03 * axis, t=HALF)
            yield from R_.move(home - 0.03 * axis, t=HALF)
            yield from R_.move(home + AMP * stroke, t=HALF)
            yield from R_.move(home - AMP * stroke, t=HALF)
        yield from R_.move(home, t=HALF)
        yield from R_.wait(0.6)
    if tilt:
        yield from R_.arc(R_.at(), axis, -tilt, t=abs(tilt) / 40.0, label="back upright")
    if ROLL and mode != "on_side":  # the shoe sticks in the pinch rolled about its length after the shake: roll the hand back
        yield from R_.arc(R_.at(), along, ROLL, t=abs(ROLL) / 40.0, label="roll the sole level")
    if mode == "on_side":
        yield from R_.arc(R_.at(), along, 80.0, t=2.0, label="roll onto its side")
    if mode in ("drop", "drop_low", "drop_25mm"):
        yield from R_.move(g0 + [0.0, 0.0, {"drop": 0.10, "drop_low": 0.03, "drop_25mm": 0.025}[mode]], t=1.5)
        yield from R_.open(t=0.2)
        yield from R_.move(R_.at() + [0.0, 0.0, 0.08], t=1.0)
        yield from R_.wait(4.0)
        print("[shoe] END " + json.dumps({"mode": mode, **TRIAL.last}), flush=True)
        return
    if mode == "toe_first":
        yield from R_.move(g0 + [0.0, 0.0, TOE_DZ], t=1.5)
        yield from R_.arc(R_.at(), axis, TOE_ANG, t=1.0, label="toe down first")
        yield from R_.open(t=0.2)
        yield from R_.move(R_.at() + [0.0, 0.0, 0.08], t=1.0)
        yield from R_.wait(4.0)
        print("[shoe] END " + json.dumps({"mode": mode, **TRIAL.last}), flush=True)
        return
    if mode != "on_side":
        yield from R_.move(g0 + [0.0, 0.0, 0.006], t=2.0, label="set it down")
        yield from R_.move(g0 + [0.0, 0.0, SETDZ], t=1.5, label="sole on the table")
        yield from R_.wait(1.0)
        if mode != "toe_first" and COMP:
            yield from R_.arc(R_.at(), axis, COMP, t=1.0, label="heel down, sole flat")
    yield from R_.open()
    if mode == "wide_toe_first":  # the shoe is emptied and set down as in full; then gripped across the heel's two sides, lifted, lowered toe first with the heel about 5 cm up, and let go
        wg = d.tilted(float(os.environ.get("SHOE_WTH", "0")), SHOE_YAW)  # fingers down, closing across the shoe
        wx = float(os.environ.get("SHOE_WX", "0.045"))
        w0 = shoe_to_world((wx, 0.0, float(os.environ.get("SHOE_WZ", "0.05"))))
        yield from R_.move(w0 + [0.0, 0.0, 0.08], grip=wg, finger=d.FINGER_OPEN, t=0.8, label="above the heel")
        yield from R_.move(w0, t=0.8, label="around the heel's sides")
        yield from R_.close(2 * half_w(wx), PINCH_N, label="pinch the heel's sides")
        yield from R_.move(R_.at() + [0.0, 0.0, float(os.environ.get("SHOE_WLIFT", "0.07"))], t=1.0, label="lift")
        yield from R_.arc(R_.at(), np.cross(along, [0.0, 0.0, 1.0]), TOE_ANG, t=1.0, label="toe down first")
        yield from R_.open(t=0.2)
        yield from R_.move(R_.at() + [0.0, 0.0, 0.08], t=1.0)
        yield from R_.wait(4.0)
        print("[shoe] END " + json.dumps({"mode": mode, **TRIAL.last}), flush=True)
        return
    if mode == "collar_hook_drop":  # emptied and set down as in full; then both fingertips go into the collar opening and spread against its inner walls, lift 5 cm, and ease off so the shoe slides off
        wg = d.tilted(0.0, SHOE_YAW)
        wx = 0.06
        w0 = shoe_to_world((wx, 0.0, 0.07))
        inner = 2 * (half_w(wx) - T)
        yield from R_.move(w0 + [0.0, 0.0, 0.08], grip=wg, finger=d.FINGER_OPEN, t=0.8, label="above the collar")
        yield from R_.close(0.03, 5.0, t=0.4, label="fingers together")
        yield from R_.move(w0, t=0.8, label="into the collar")
        yield from R_.close(inner, -PINCH_N, label="spread against the inner walls")
        yield from R_.move(R_.at() + [0.0, 0.0, 0.05], t=1.0, label="lift 5 cm")
        yield from R_.close(0.03, 5.0, t=0.3, label="ease off")
        yield from R_.wait(4.0)
        print("[shoe] END " + json.dumps({"mode": mode, **TRIAL.last}), flush=True)
        return
    if mode == "relift_3cm":
        yield from R_.close(BACK, PINCH_N, label="pinch again")
        yield from R_.move(R_.at() + [0.0, 0.0, 0.03], t=1.0, label="lift 3 cm")
        yield from R_.open(t=0.2)
        yield from R_.wait(3.0)
        print("[shoe] END " + json.dumps({"mode": mode, **TRIAL.last}), flush=True)
        return
    yield from R_.move(R_.at() + [0.0, 0.0, 0.09], t=1.2, label="hand up off the heel")
    yield from R_.move(R_.at() - 0.09 * axis, t=1.0, label="hand clear")
    yield from R_.wait(3.0)
    print("[shoe] END " + json.dumps({"mode": mode, **TRIAL.last}), flush=True)
