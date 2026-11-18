"""Prop a door open with a rubber wedge: hold the door still with one hand and push the wedge under its bottom edge with the other, until its slope bites.

The lower 250 mm of a standard 30 x 80 in hollow-core door (762 x 35 mm, 1.4 kg, estimate) hangs on a vertical hinge line (two 3-1/2 in butt hinges on a 2x4 post), 13 mm above a
vinyl floor tile on the bench. An Everbilt 4-3/4 in rubber door stop (119 x 56 x 36 mm, 5 mm tip, 0.12 kg) lies on the tile, slope up, thin end towards the door.
The door swings freely (0.2 N m of hinge friction, a drive that holds at most that torque, README 15). The scorer is the real trial's pull: once every fingertip of both hands is 3 cm
clear of the door and the wedge and all has been still for 1 s, it pulls the free edge toward closing with 20 N for 2 s.

    python demokit.py task.py OUT --no-record --mode full,none,one_hand,no_wedge,beside,short,loose
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
INSTRUCTION = "Prop the door open with the rubber wedge: push it under the door's bottom edge so the door stays open when pushed, then let go with both hands."
SUCCESS_HOLD_S = 0.0  # the scorer judges itself (one pull, then its tier)
SCORE_OVER = "end"
TOP = d.TABLE_TOP
MM = 1e-3

DX = float(os.environ.get("DW_DX", "0.08"))  # the whole rig is set this far out in x (reach: README 27)
SIGN = float(os.environ.get("DW_SIGN", "1"))  # +1 if a positive hinge angle closes the door (toward -y)

# -- the door section: x from the hinge (0) back to the free edge (-DOOR_W), thickness along y, height along z ---------------------------------------------------------------------
DOOR_W, DOOR_T, DOOR_Z0, DOOR_Z1 = 0.762, 0.035, 0.018, 0.268  # bottom edge 13 mm above the tile's top (5 mm)
FREE_X = 0.22 + DX
X_H = FREE_X + DOOR_W  # the hinge line
DOOR_M = 12.0  # the swing inertia of a full 12 kg, 762 mm leaf: 12 x 0.762^2 / 3 = 2.3 kg m^2 about the hinge (estimate); the visible stub is 250 mm tall
HINGE_FRICTION = 0.2  # N m
HINGE_K = 20.0  # N m/rad of the drive that holds at most HINGE_FRICTION
TILE_TOP = 0.005
PULL_N, PULL_Z, PULL_STEPS = 20.0, TILE_TOP + 0.10, 40
CLEAR_M = 0.03

# -- the wedge (Everbilt 119 x 56 x 36 mm, 5 mm tip): origin at the middle of its underside, thin end +y -----------------------------------------------------------------------------
WL, WW, WH, WTIP = 0.119, 0.056, 0.036, 0.005
SLOPE = (WH - WTIP) / WL
WEDGE_M = 0.12
WEDGE_START = np.array([0.32 + DX, -0.20])

LOOKS = {"door": m.Look(m.rgb(0.93, 0.92, 0.88), roughness=0.5), "post": m.Look(m.rgb(0.76, 0.60, 0.40), roughness=0.7), "steel": m.Look(m.rgb(0.72, 0.72, 0.74), metallic=0.8, roughness=0.35),
         "tile": m.Look(m.rgb(0.62, 0.58, 0.52), roughness=0.6), "rubber": m.Look(m.rgb(0.92, 0.90, 0.84), roughness=0.85)}


def _box(size, c):
    return m.box(size, c)


def _door() -> m.Part:
    cz = (DOOR_Z0 + DOOR_Z1) / 2
    hz = DOOR_Z1 - DOOR_Z0
    leaf = (DOOR_W, DOOR_T, hz)
    centre = (-DOOR_W / 2, 0.0, cz)
    parts = [("leaf", _box(leaf, centre), "door")]
    cols = [m.col_box("leaf_c", leaf, centre)]
    for k, zc in enumerate((0.06, 0.21)):  # hinge leaves on the door's edge (visual)
        parts.append((f"hl{k}", _box((0.0035, 0.032, 0.089), (-0.0018, 0.0, zc)), "steel"))
    mass_c = (-DOOR_W / 2, 0.0, cz)
    inertia = (DOOR_M / 12 * (DOOR_T**2 + hz**2), DOOR_M / 12 * (DOOR_W**2 + hz**2), DOOR_M / 12 * (DOOR_W**2 + DOOR_T**2))
    link = m.hinge("door", "Z", (0.0, 0.0, 0.0), -25.0, 100.0, parts, cols, mass=DOOR_M, spring=HINGE_K, effort=HINGE_FRICTION, damping=0.02, friction=0.0, armature=1e-3,
                   gravity=True, com=mass_c, inertia=inertia)
    post_parts = [("post", _box((0.05, 0.10, 0.32), (0.027, 0.0, 0.16)), "post")]
    post_cols = [m.col_box("post_c", (0.05, 0.10, 0.32), (0.027, 0.0, 0.16))]
    for k, zc in enumerate((0.06, 0.21)):
        post_parts.append((f"knuckle{k}", m.cylinder(0.005, 0.089, (0.0, 0.0, zc)), "steel"))
        post_parts.append((f"pl{k}", _box((0.0035, 0.032, 0.089), (0.0018, 0.0, zc)), "steel"))
    return m.mechanism("door", (X_H, 0.0, TOP), [link], LOOKS, frame_parts=post_parts, frame_colliders=post_cols, friction=(0.1, 0.1), solver_iters=64, frame_inertia=(0.05, 0.05, 0.05))


def _tile() -> m.Part:
    size = (0.610, 0.305, TILE_TOP)
    c = (0.45 + DX, -0.10, TILE_TOP / 2)
    return m.fixed("tile", (0.0, 0.0, TOP), [("tile", _box(size, c), "tile")], [m.col_box("tile_c", size, c)], LOOKS, friction=(1.3, 1.3))


def _wedge_mesh() -> m.Mesh:
    outline = np.array([(-WL / 2, 0.0), (WL / 2, 0.0), (WL / 2, WTIP), (-WL / 2, WH)])  # (y, z) profile
    p = m.prism(outline, -WW / 2, WW / 2)
    return m.Mesh(p.v[:, [2, 0, 1]], p.f)  # extrusion along x


def _wedge() -> m.Part:
    mesh = _wedge_mesh()
    return m.free("wedge", (WEDGE_START[0], WEDGE_START[1], TOP + TILE_TOP + 0.0003), [("body", mesh, "rubber")], [m.col_hull("body_c", mesh)], LOOKS, mass=WEDGE_M, friction=(0.7, 0.7),
                  torsional_patch=None, solver_iters=64, compliance=(1.0e5, 50.0))


OBJECTS = [_tile(), _door(), _wedge()]


# -- geometry readings ------------------------------------------------------------------------------------------------------------------------------------------------------------


def rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


HINGE_W = np.array([X_H, 0.0, TOP])


def door_theta(door) -> float:
    return SIGN * float(door.joint("door"))  # positive closes the door


def door_world(theta: float, p_local) -> np.ndarray:
    return HINGE_W + rz(theta) @ np.asarray(p_local, float)


def edge_point(theta: float) -> np.ndarray:
    """The free edge 10 cm above the floor tile, where the pull is taken."""
    return door_world(theta, (-DOOR_W, 0.0, PULL_Z))


def wedge_geom(w):
    R = w.rotation()
    return np.asarray(w.pos, float), R


def box_dist(p, centre, half) -> np.ndarray:
    q = np.abs(np.atleast_2d(p) - centre) - half
    return np.linalg.norm(np.maximum(q, 0.0), axis=1)


def bite(theta: float, wpos, wR) -> dict:
    """Where the wedge's slope meets the door's bottom edge (its closing-side face, y = -DOOR_T/2): the gap from the edge to the slope (mm, within the wedge's footprint), and how far the tip is past the face."""
    xs = np.linspace(-DOOR_W, -DOOR_W + 0.15, 61)
    edge = np.array([door_world(theta, (x, -DOOR_T / 2, DOOR_Z0)) for x in xs])
    loc = (edge - wpos) @ wR  # in the wedge's frame
    inside = (np.abs(loc[:, 0]) <= WW / 2) & (np.abs(loc[:, 1]) <= WL / 2)
    slope_z = WTIP + (WL / 2 - loc[:, 1]) * SLOPE
    gap = loc[:, 2] - slope_z
    tip_w = wpos + wR @ np.array([0.0, WL / 2, WTIP])
    tip_door = rz(-theta) @ (tip_w - HINGE_W)  # tip in the door's frame: y past the face is tip_door[1] + DOOR_T / 2
    past = float(tip_door[1] + DOOR_T / 2)
    touching = bool(inside.any() and np.min(np.abs(gap[inside])) <= 0.0015 and loc[inside, 2].min() <= WH)
    return {"touch": touching, "past_mm": round(past * 1000, 1), "gap_mm": None if not inside.any() else round(float(np.min(gap[inside])) * 1000, 2)}


def tier(shoved_mm: float, moved_mm: float, still: bool, b: dict) -> float:
    """1.0: slope on the edge, 20 mm or more of the wedge past the face, the door left where it stood (within 8 mm of its start), and it moves under 5 mm in the pull.
    0.5: the wedge stops the door (moves 5 to 30 mm and is still, slope on the edge), or it holds but the door was shoved over 8 mm, or less than 20 mm of the wedge is under.
    0: the door moves over 30 mm, or the wedge is not on the edge."""
    if moved_mm > 30.0 or not b["touch"] or not still:
        return 0.0
    if b["past_mm"] >= 20.0 and shoved_mm <= 8.0 and moved_mm < 5.0:
        return 1.0
    return 0.5


class Trial:
    def reset(self, env=None):
        self.n = 0
        self.tgt = 0.0
        self.engaged = False
        self.quiet = 0
        self.pull = 0  # steps of pull done
        self.done = False
        self.final = 0.0
        self.E0 = None  # the free edge at the first step
        self.E_start = None  # and when the pull starts
        self.last = {}
        self.prev_theta = 0.0
        self.prev_w = None
        self.max_move = 0.0

    def step(self, env):
        door, wedge = m.state(env, "door"), m.state(env, "wedge")
        th = door_theta(door)
        wpos, wR = wedge_geom(wedge)
        self.n += 1
        # the hinge's 0.2 N m: a drive whose target trails the joint, so it holds at most that torque whatever the load (README 15)
        q = float(door.joint("door"))
        slack = HINGE_FRICTION / HINGE_K
        if q - self.tgt > slack:
            self.tgt = q - slack
        elif self.tgt - q > slack:
            self.tgt = q + slack
        m.set_target(env, "door", "door", self.tgt)
        E = edge_point(th)
        if self.E0 is None:
            self.E0 = E.copy()
        tips = m.fingertips(env)
        dc = np.array([-DOOR_W / 2, 0.0, (DOOR_Z0 + DOOR_Z1) / 2])
        dloc = (tips - HINGE_W) @ rz(th)  # fingertips in the door's frame
        dd = box_dist(dloc, dc, np.array([DOOR_W / 2, DOOR_T / 2, (DOOR_Z1 - DOOR_Z0) / 2]))
        wloc = (tips - wpos) @ wR
        wd = box_dist(wloc, np.array([0.0, 0.0, WH / 2]), np.array([WW / 2, WL / 2, WH / 2]))
        clear = bool(min(dd.min(), wd.min()) >= CLEAR_M)
        if min(dd.min(), wd.min()) < 0.02:
            self.engaged = True  # a hand has handled the door or the wedge
        dspeed = abs(th - self.prev_theta) * DOOR_W / float(env.unwrapped.step_dt)
        self.prev_theta = th
        wmove = 0.0 if self.prev_w is None else float(np.linalg.norm(wpos - self.prev_w))  # per control step (the solver's rest jitter shows in the velocity, not the pose)
        self.prev_w = wpos.copy()
        still = bool(dspeed < 0.003 and wmove < 0.0002 and np.linalg.norm(wedge.ang_vel) < 0.5)
        b = bite(th, wpos, wR)
        if not self.done and self.pull == 0:
            self.quiet = self.quiet + 1 if (self.engaged and clear and still) else 0
            if self.quiet >= 20:
                self.pull = 1
                self.E_start = E.copy()
        if self.pull >= 1 and not self.done:
            closing = rz(th) @ np.array([0.0, -1.0, 0.0]) * SIGN
            m.push(env, "door", PULL_N * closing, at=E, body="door")
            moved = float(np.linalg.norm(E[:2] - self.E_start[:2]))
            self.max_move = max(self.max_move, moved)
            self.pull += 1
            if self.pull > PULL_STEPS:
                m.push(env, "door", (0.0, 0.0, 0.0), body="door")
                shoved = float(np.linalg.norm(self.E_start[:2] - self.E0[:2]))
                self.final = tier(shoved * 1000, moved * 1000, still, b)
                self.done = True
                self.last = {"tier": self.final, "shoved_mm": round(shoved * 1000, 1), "moved_mm": round(moved * 1000, 1), "bite": b, "still": still}
                print("[dw] JUDGED " + json.dumps(self.last), flush=True)
        if self.n % 10 == 0:
            print("[dw] " + json.dumps({"mode": os.environ.get("TASK_DEMO", ""), "n": self.n, "theta": round(math.degrees(th), 2), "wedge": [round(float(v), 4) for v in wpos], "bite": b,
                                        "clear": clear, "still": still, "dsp": round(dspeed, 4), "wv": round(float(np.linalg.norm(wedge.lin_vel)), 4), "wa": round(float(np.linalg.norm(wedge.ang_vel)), 3), "eng": self.engaged, "quiet": self.quiet, "pull": self.pull, "done": self.done, "tier": self.final}), flush=True)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects) -> float:
    return TRIAL.final if TRIAL.done else 0.0


def success(objects) -> bool:
    return bool(TRIAL.done and TRIAL.final >= 1.0)


# -- the scripted demo ------------------------------------------------------------------------------------------------------------------------------------------------------------

PICK_Z = float(os.environ.get("DW_PICK_Z", "0.020"))  # TCP height above the wedge's underside when it is picked
GRIP_N = 30.0
SEAT_X = FREE_X + 0.06  # the wedge goes in 6 cm from the free edge
DOOR_FACE_Y = -DOOR_T / 2


def _wedge_tip(ctx):
    s = ctx.state("wedge")
    return np.asarray(s.pos, float) + m.quat_matrix(s.quat_xyzw) @ np.array([0.0, WL / 2, WTIP])


def demo_script(ctx):
    """Modes: full. none: the arms wait. one_hand: only the right hand, the door unheld. no_wedge: the left hand holds the door, the right never brings the wedge. beside: the wedge set down beside the door.
    short: the wedge pushed to 10 mm under the edge only. loose: the wedge laid on its side. drop: one hand sets the wedge down 29 mm under the edge, the door unheld."""
    mode = ctx.mode
    if mode not in ("full", "none", "one_hand", "no_wedge", "beside", "short", "loose", "drop"):
        raise ValueError(f"unknown mode {mode!r}")
    L, R = ctx.left, ctx.right
    if mode == "none":
        yield from L.wait(3.0)
        return
    top_z = TOP + DOOR_Z1
    lg = os.environ.get("DW_LGRIP", "level")
    if lg == "down":
        hold, left_grip, appr = np.array([FREE_X + 0.03, 0.0, top_z - 0.012]), d.down(0), np.array([0.0, 0.0, 0.10])
    else:  # level, fingers +x, closing across the door's thickness on its free edge face, 2.5 cm into the leaf, near its top
        th = float(os.environ.get("DW_LTILT", "60"))
        hold, left_grip, appr = np.array([FREE_X + 0.0133 + 0.025, 0.0, TOP + float(os.environ.get("DW_LZ", "0.21"))]), d.tilted(th, 0.0), np.array([-0.10, 0.0, 0.0])
    rg = os.environ.get("DW_RGRIP", "tilt:40,90")
    right_grip = d.down(float(rg[5:])) if rg.startswith("yaw:") else d.tilted(*[float(v) for v in rg[5:].split(",")])
    if mode not in ("one_hand", "drop"):
        yield from L.move(hold + appr, grip=left_grip, finger=d.FINGER_OPEN, label="left over the free edge")
        yield from L.move(hold, t=1.2, label="left down on the top edge")
        yield from L.close(DOOR_T, GRIP_N, label="left pinches the door")
        print(f"[dw] door held at {ctx.deg('door', 'door'):.2f} deg", flush=True)
    if mode == "no_wedge":
        yield from L.wait(2.0)
        yield from L.open()
        yield from L.move(L.at() + [-0.10, 0.0, 0.0], t=1.0)
        yield from L.wait(2.5)
        return
    # right hand: pick the wedge by its width, near the thick end
    w = ctx.pos("wedge")
    pick = lambda: ctx.pos("wedge") + np.array([0.0, -0.03, PICK_Z])
    yield from R.move(pick() + [0.0, 0.0, 0.10], grip=right_grip, finger=d.FINGER_OPEN, label="right over the wedge")
    yield from R.servo(pick() + [0.0, 0.0, 0.10], timeout=1.0)
    yield from R.move(pick(), t=1.2, label="right down on the wedge")
    yield from R.close(WW, GRIP_N, label="right grips the wedge")
    yield from R.move(R.at() + [0.0, 0.0, 0.025], t=0.8, label="lift")
    if mode == "loose":
        yield from R.move(R.at() + [0.0, 0.08, 0.0], t=1.0)
        yield from R.open()
        yield from R.move(R.at() + [0.0, 0.0, 0.1], t=1.0)
        yield from R.wait(2.5)
        return
    # carry to in front of the door's face at the seat, tip 4 cm short of the face, lower onto the tile
    y_tip_goal = DOOR_FACE_Y - 0.04 if mode != "drop" else DOOR_FACE_Y + 0.029  # drop: set straight down, tip 29 mm under the edge, door unheld
    off = np.array([SEAT_X, 0.0, 0.0]) - np.array([ctx.pos("wedge")[0], 0.0, 0.0])
    yield from R.move(R.at() + [off[0], (y_tip_goal - WL / 2) - ctx.pos("wedge")[1] - (0.0 if mode != "beside" else 0.18), 0.0], t=2.0, label="carry to the door")
    dz = TOP + TILE_TOP + 0.001 - ctx.pos("wedge")[2]
    yield from R.move(R.at() + [0.0, 0.0, dz], t=0.8, label="lower onto the tile")
    if mode in ("beside", "drop"):
        yield from R.open()
        yield from R.move(R.at() + [0.0, 0.0, 0.1], t=1.0)
    else:
        # push in along +y until the slope meets the door's bottom edge and the tip is 30 mm past the face, read from the wedge's own pose
        goal_past = {"short": 0.010}.get(mode, 0.032)
        res = yield from R.press((0.0, 1.0, 0.0), 12.0, 0.04 + goal_past, speed=0.03, settle=0.5, timeout=6.0)
        tip = _wedge_tip(ctx)
        print(f"[dw] pressed {res}; tip y {tip[1] * 1000:.1f} mm, bite {bite(door_theta(ctx.state('door')), *wedge_geom(ctx.state('wedge')))}", flush=True)
        yield from R.open()
        yield from R.move(R.at() + [0.0, -0.06, 0.0], t=1.0, label="right backs off")
        yield from R.move(R.at() + [0.0, 0.0, 0.10], t=1.0, label="right up")
    if mode not in ("one_hand", "drop"):
        yield from L.open()
        yield from L.move(L.at() + [-0.10, 0.0, 0.0], t=1.0, label="left back")
    yield from L.wait(5.0)
    print("[dw] END " + json.dumps({"mode": mode, **TRIAL.last}), flush=True)
