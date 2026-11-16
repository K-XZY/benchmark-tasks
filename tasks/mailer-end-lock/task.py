"""Lock the near end of a roll-end tuck-front mailer (FEFCO 0427, at Uline S-16934's outside size): one hand holds the sprung end wall
upright while the other rolls the wall's inner panel over its double crease and down inside the box, until both its tabs snap into
the holes in the base. Then both hands come away.

The box is one free articulation, its links colliding with one another (only a link and its own parent do not):
  frame: base (with the two tab holes along the near crease), the front and back walls, the far end (already locked), and the lid,
         open to the robot's left with its front edge on the table;
  outer: the near end's outer panel, hinged on the base crease. Its crease springs it back to 75 degrees above the table (joint -15);
  roll:  the inner panel, hinged on the outer panel's top. The double crease is one hinge here, placed so that the rolled panel lies
         17.5 mm inside the outer face, as Uline's outside and inside sizes give, and its spring resists the roll;
  tab_l, tab_r: the two locking tabs on the inner panel's free edge. Each rides a short sprung slide, standing for the board's give as
         the tab is pressed over the base: it springs into its hole when it comes over it;
  ear_l, ear_r: the tuck flaps on the front and back walls' near ends, folded in across the end before the task starts.
A tab latches in its hole once pressed at least 2 mm into the base board inside the hole's outline. With both latched, the end is held
locked, as the seated tabs and the flaps trapped inside the rolled wall hold a real one.

Debug:  python demokit.py task.py OUT --no-record --mode full,unheld,onehand
Final:  python demokit.py task.py OUT
Model:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 --instruction "<INSTRUCTION below>"
"""

from __future__ import annotations

import collections
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

SCENE = "station"
INSTRUCTION = ("Lock the near end of the mailer box, both tabs seated in the holes in the base and the wall's top flat: hold the end wall "
               "upright, roll its inner panel over the top and down inside until both tabs snap in, then move both hands clear.")
SUCCESS_HOLD_S = 0.0  # the scorer holds the end state itself (HOLD_STEPS)
SCORE_OVER = "end"
TOP = d.TABLE_TOP
DT = d.DT
HOLD_STEPS = 20  # 1 s at 20 Hz

LOOKS = {
    "kraft": m.Look(m.rgb(0.66, 0.49, 0.31), roughness=0.92),
    "kraft_in": m.Look(m.rgb(0.74, 0.58, 0.40), roughness=0.95),
    "edge": m.Look(m.rgb(0.55, 0.40, 0.25), roughness=0.95),
    "hole": m.Look(m.rgb(0.16, 0.12, 0.08), roughness=0.95),
    "wood": m.Look(m.rgb(0.45, 0.30, 0.18), roughness=0.7),
}

# -- the box, in its own frame: x away from the robot, y to its left, z up; origin on the table under the near end's crease ----------
BOX_AT = (0.28, 0.0)  # the near crease along y at x 28 cm
T = 0.0032  # B-flute board, 1/8 in
LX, WY, H = 0.340, 0.314, 0.159  # outside 13 3/8 x 12 3/8 x 6 1/4 in (Uline S-16934); the roll ends on the 12 3/8 in sides
ROLLED = 0.0175  # a rolled end's thickness: (13 3/8 - 12) / 2 in, outside less inside
S = ROLLED - T  # outer panel's middle to the rolled inner panel's middle
XA, ZA = (T + S) / 2, H  # the roll's hinge: half way across the rolled top, at the outer panel's top
GAP = 0.002  # the unrolled inner panel starts this far above the outer's top: rolled, its top sits this far below
PANEL_L = H - GAP - T - 0.002  # rolled, its free edge stands 2 mm over the base board
PANEL_W = WY - 2 * T - 0.003  # it rolls down between the front and back walls
P_TOP = H + GAP + PANEL_L  # the unrolled panel's free edge
TAB_W, TAB_OUT, TAB_IN = 0.025, 0.005, 0.004  # a tab: width, how far it stands out of the panel's edge, how far into it it overlaps
TAB_Y = 0.080
HOLE_X = 2 * XA - T / 2  # a hole's middle across the crease, under the rolled panel
HOLE_DX, HOLE_DY = 0.006, 0.030
EAR_W, EAR_Z0, EAR_Z1 = 0.140, 0.005, 0.145  # a tuck flap, folded in across the end behind the outer panel
LID_RUN = math.sqrt(WY ** 2 - (H - T / 2) ** 2)  # the open lid, hinged on the left wall's top, reaches the table this far out
FAR = LX - ROLLED  # the far end's inside face
START_DEG = -15.0  # the outer panel's rest: 75 degrees above the table, leaning toward the robot

K_OUTER = 3.0 * H / math.radians(-START_DEG)  # N m/rad: 3 N at its top edge holds it upright
K_ROLL = float(os.environ.get("MAILER_KROLL", "0.01"))  # N m/rad: 0.2 N at its free edge (1.6 N 2 cm out from the crease) holds it rolled flat inside
K_TAB = float(os.environ.get("MAILER_KTAB", "150"))  # N/m: a tab pressed back 3 mm by the base bears on it with 0.45 N
K_EAR = 0.05
MASS = 0.45  # Uline: 0.99 lb
M_OUTER, M_ROLL, M_TAB, M_EAR = 0.035, 0.033, 0.001, 0.012


def _turn_y(mesh: m.Mesh, deg: float, at) -> m.Mesh:
    a = np.asarray(at, float)
    return mesh.moved(-a).turned(deg, "y").moved(a)


def _outer() -> m.Link:
    panel = m.box((T, WY, H), (T / 2, 0.0, H / 2))
    col = m.col_box("outer_c", (T, WY, H - 0.0015), (T / 2, 0.0, (H + 0.0015) / 2))  # 1.5 mm clear of the table at the crease
    return m.hinge("outer", "Y", (0.0, 0.0, 0.0), START_DEG - 60.0, 0.0, [("outer_panel", panel, "kraft")], [col],
                   mass=M_OUTER, spring=0.0, damping=0.03, friction=0.0, armature=0.0)  # its spring is set at the first step (README 1)


def _roll() -> m.Link:
    zc = H + GAP + PANEL_L / 2
    panel = m.box((T, PANEL_W, PANEL_L), (T / 2, 0.0, zc))
    bead = m.cylinder(XA, PANEL_W + 0.004, (XA, 0.0, ZA), axis="y", segments=20)  # the rolled top's round, the same from either side
    face = m.box((0.0004, PANEL_W - 0.01, PANEL_L - 0.01), (T + 0.0002, 0.0, zc))  # the board's paler inside face
    return m.hinge("roll", "Y", (XA, 0.0, ZA), -10.0, 180.0, [("inner_panel", panel, "kraft"), ("bead", bead, "kraft"), ("inner_face", face, "kraft_in")],
                   [m.col_box("inner_c", (T, PANEL_W, PANEL_L), (T / 2, 0.0, zc))], parent="outer", mass=M_ROLL, spring=K_ROLL, damping=0.01, friction=0.0, armature=0.0)


def _tab(name: str, y: float) -> m.Link:
    size, c = (T, TAB_W, TAB_OUT + TAB_IN), (T / 2, y, P_TOP + (TAB_OUT - TAB_IN) / 2)
    return m.slider(name, "Z", -TAB_OUT, 0.0, [(f"{name}_v", m.box(size, c), "edge")], [m.col_box(f"{name}_c", size, c)], parent="roll",
                    mass=M_TAB, spring=K_TAB, damping=0.5, friction=0.0, armature=0.0)


def _ear(name: str, side: float) -> m.Link:
    y0 = side * (WY / 2 - T)
    size, c = (T, EAR_W, EAR_Z1 - EAR_Z0), (1.5 * T + 0.0003, y0 - side * EAR_W / 2, (EAR_Z0 + EAR_Z1) / 2)
    lo, hi = (-80.0, 0.0) if side > 0 else (0.0, 80.0)  # 0: folded square across the end; opening swings its free edge toward the robot
    return m.hinge(name, "Z", (T + 0.0003, y0, 0.0), lo, hi, [(f"{name}_v", m.box(size, c), "kraft")], [m.col_box(f"{name}_c", size, c)],
                   mass=M_EAR, spring=K_EAR, damping=0.01, friction=0.0, armature=0.0)


def _base_pieces() -> list[tuple[str, tuple, tuple]]:
    """The base board as boxes (name, size, centre), leaving the two tab holes open."""
    x0, x1 = HOLE_X - HOLE_DX / 2, HOLE_X + HOLE_DX / 2
    out = [("base_a", (x0, WY, T), (x0 / 2, 0.0, T / 2)), ("base_c", (LX - x1, WY, T), ((x1 + LX) / 2, 0.0, T / 2))]
    ys = [-WY / 2, -TAB_Y - HOLE_DY / 2, -TAB_Y + HOLE_DY / 2, TAB_Y - HOLE_DY / 2, TAB_Y + HOLE_DY / 2, WY / 2]
    for k, (a, b) in enumerate([(ys[0], ys[1]), (ys[2], ys[3]), (ys[4], ys[5])]):
        out.append((f"base_b{k}", (HOLE_DX, b - a, T), (HOLE_X, (a + b) / 2, T / 2)))
    return out


def _lid() -> tuple[m.Mesh, m.Mesh, m.Mesh]:
    """The lid, hinged on the left wall's top and lying open, its front edge on the table; its tuck flap flat on the table beyond; and
    its collider, a slab 2 cm deep under the lid's top face, kept 2 cm off the hinge and 6 cm off the front edge: a thin plate's hull
    is cooked on the CPU as "oblong", and that one held the box up off the table."""
    ang = math.degrees(math.atan2(H - T / 2, LID_RUN))

    def placed(mesh: m.Mesh) -> m.Mesh:
        return mesh.turned(-ang, "x").moved((0.0, WY / 2, H))

    lid = placed(m.box((LX - 2 * T, WY, T), (LX / 2, WY / 2, 0.0)))
    slab = placed(m.box((LX - 2 * T, WY - 0.08, 0.02), (LX / 2, 0.02 + (WY - 0.08) / 2, T / 2 - 0.01)))
    tuck = m.box((LX - 0.06, 0.045, T), (LX / 2, WY / 2 + LID_RUN + 0.0225, T / 2))
    return lid, tuck, slab


def _mailer() -> m.Part:
    frame, cols = [], []
    for name, size, c in _base_pieces():
        frame.append((name, m.box(size, c), "kraft_in"))
        cols.append(m.col_box(f"{name}_c", size, c))
    for k, (a, b) in enumerate([(-TAB_Y - HOLE_DY / 2, -TAB_Y + HOLE_DY / 2), (TAB_Y - HOLE_DY / 2, TAB_Y + HOLE_DY / 2)]):
        frame.append((f"hole_{k}", m.box((HOLE_DX, b - a, 0.0004), (HOLE_X, (a + b) / 2, 0.0002)), "hole"))
    for name, side in (("wall_l", 1.0), ("wall_r", -1.0)):
        size, c = (FAR - T, T, H), ((T + FAR) / 2, side * (WY / 2 - T / 2), H / 2)
        frame.append((name, m.box(size, c), "kraft"))
        cols.append(m.col_box(f"{name}_c", size, c))
    far = m.box((ROLLED, WY, H), (FAR + ROLLED / 2, 0.0, H / 2))
    frame += [("far_end", far, "kraft"), ("far_bead", m.cylinder(ROLLED / 2, WY, (FAR + ROLLED / 2, 0.0, H), axis="y", segments=20), "kraft")]
    cols.append(m.col_box("far_c", (ROLLED, WY, H), (FAR + ROLLED / 2, 0.0, H / 2)))
    lid, tuck, slab = _lid()
    frame += [("lid", lid, "kraft"), ("tuck", tuck, "kraft")]
    if not os.environ.get("MAILER_NOLID"):  # debug switch
        cols.append(m.col_hull("lid_c", slab))
    cols += [m.col_box("tuck_c", (LX - 0.06, 0.045, T), (LX / 2, WY / 2 + LID_RUN + 0.0225, T / 2))]
    links = [_outer(), _roll(), _tab("tab_l", TAB_Y), _tab("tab_r", -TAB_Y), _ear("ear_l", 1.0), _ear("ear_r", -1.0)]
    frame_mass = MASS
    inertia = (frame_mass / 12 * (WY ** 2 + H ** 2), frame_mass / 12 * (LX ** 2 + H ** 2), frame_mass / 12 * (LX ** 2 + WY ** 2))
    # The box is a free body: the frame carries its whole 0.45 kg (the links have no gravity here), so the table bears the real weight.
    # Table (Isaac Lab default 0.5) and board average to MU, the combined static and dynamic friction of corrugated board on a laminate
    # or wood bench: Park et al. 2019 give board on board 0.38 static; 0.3 to 0.5 is assumed for the bench, 0.4 used.
    fixed = bool(os.environ.get("MAILER_FIXED"))
    free = None if fixed else MASS
    mu = float(os.environ.get("MAILER_MU", "0.4")) * 2 - 0.5
    part = m.mechanism("mailer", (*BOX_AT, TOP), links, LOOKS, frame_parts=frame, frame_colliders=cols, friction=(mu, mu), free_mass=free,
                       frame_inertia=inertia, start=(("outer", math.radians(START_DEG)),))
    build0 = part.build

    def build() -> str:
        txt = build0()
        if not fixed:  # the articulation's root is the frame, so that the reset writes the frame's pose and not a link's
            api = ' (\n    prepend apiSchemas = ["PhysicsArticulationRootAPI", "PhysxArticulationAPI"]\n)'
            attrs = txt[txt.index("    bool physxArticulation:enabledSelfCollisions"):txt.index("    def Material")]
            txt = txt.replace(api, "", 1).replace(attrs, "", 1)
            fh = 'def Xform "frame" (\n        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]\n    )\n    {\n'
            txt = txt.replace(fh, fh.replace('["PhysicsRigidBodyAPI", "PhysicsMassAPI"]', '["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysicsArticulationRootAPI", "PhysxArticulationAPI"]') + attrs.replace("    bool", "        bool").replace("\n    int", "\n        int"), 1)
        if os.environ.get("MAILER_SELFCOL", "1") == "1":
            txt = txt.replace("bool physxArticulation:enabledSelfCollisions = 0", "bool physxArticulation:enabledSelfCollisions = 1")
        head = 'def PhysicsRevoluteJoint "outer_joint" (\n        prepend apiSchemas = ["PhysicsDriveAPI:angular"]'
        k = txt.index(head)
        txt = txt[:k] + head.replace('["PhysicsDriveAPI:angular"]', '["PhysicsDriveAPI:angular", "PhysicsJointStateAPI:angular"]') + txt[k + len(head):]
        anchor = 'uniform token drive:angular:physics:type = "force"'
        j = txt.index(anchor, k)
        return txt[:j] + f"float state:angular:physics:position = {START_DEG:.6f}\n        float state:angular:physics:velocity = 0\n        " + txt[j:]

    return dataclasses.replace(part, build=build)


OBJECTS = [_mailer()]


# -- reading the box ---------------------------------------------------------------------------------------------------------------


def link_pose(env, body: str) -> tuple[np.ndarray, np.ndarray]:
    """A mailer link's pose (station frame): where the part's own origin has gone with that link (README 14)."""
    base = env.unwrapped
    asset = base.scene["mailer"]
    i = list(asset.body_names).index(body)
    pose = m._np(asset.data.body_link_pose_w)[0, i]
    return pose[:3] - m._np(base.scene.env_origins[0]), m.quat_matrix(pose[3:7])


def on_link(env, body: str, p) -> np.ndarray:
    pos, R = link_pose(env, body)
    return R @ np.asarray(p, float) + pos


def roll_axis(env) -> np.ndarray:
    """The roll's hinge line's middle, in the station frame, wherever the outer panel stands."""
    return on_link(env, "outer", (XA, 0.0, ZA))


class Seen:
    """What the scorer reads at one step, in the box's frame: plain numbers, so check_score.py can build them by hand.
    outer_deg: 0 upright, negative leaning out toward the robot. roll_deg: 0 unrolled, 180 rolled flat inside. edge: the inner panel's
    free edge, middle. tips: each tab's tip. ears_deg: each tuck flap's opening, 0 folded in. moved: how far the box has moved from where
    it started. clear: the nearest fingertip to the box."""

    def __init__(self, outer_deg, roll_deg, edge, tips, ears_deg, moved, tilt_deg, clear, yaw_deg=0.0):
        self.outer_deg, self.roll_deg, self.edge, self.tips = float(outer_deg), float(roll_deg), np.asarray(edge, float), tuple(tips)
        self.ears_deg, self.moved, self.tilt_deg, self.clear = tuple(ears_deg), float(moved), float(tilt_deg), float(clear)
        self.yaw_deg = float(yaw_deg)


def in_hole(tip, y: float) -> bool:
    """A tab's tip inside its hole's outline, at least 2 mm down into the base board."""
    return bool(abs(tip[0] - HOLE_X) <= (HOLE_DX - T) / 2 + 0.0005 and abs(tip[1] - y) <= (HOLE_DY - TAB_W) / 2 + 0.0005 and tip[2] <= T - 0.002)


LADDER = gk.Ladder(("rolled", 0.25), ("down", 0.5), ("one_tab", 0.75), ("locked", 1.0))


def failed(s: Seen) -> bool:
    """The box shoved more than 5 cm (any direction, lifting included), turned more than 10 degrees about the vertical, or tipped more
    than 10 degrees; a tuck flap swung out of the end. Latched by the trial: once true at any step, the score stays 0."""
    return bool(s.moved > 0.05 or s.tilt_deg > 10.0 or abs(s.yaw_deg) > 10.0 or max(abs(a) for a in s.ears_deg) > 30.0)


def judge(s: Seen, latched: tuple, still: bool, held: int, dead: bool = False) -> tuple[bool, float]:
    """(full, score) for one step. latched: whether each tab has latched in its hole so far. held: steps the full state has held, this one included."""
    if dead or failed(s):
        return False, 0.0
    rolled = s.roll_deg >= 100.0 and s.edge[0] > T + 0.002 and s.edge[2] < H - 0.005
    down = rolled and s.roll_deg >= 170.0 and s.edge[2] <= T + 0.010
    one = down and any(latched)
    full = bool(down and all(latched) and s.roll_deg >= 178.0 and s.outer_deg >= -5.0 and max(abs(a) for a in s.ears_deg) <= 10.0 and s.clear >= 0.03 and still)
    reached = {"rolled": rolled, "down": down, "one_tab": one, "locked": full and held >= HOLD_STEPS}
    return full, LADDER.score(reached)


class Trial:
    def reset(self, env=None) -> None:
        self.steps = 0
        self.start = None
        self.latched = [False, False]
        self.near = [0, 0]
        self.locked = False
        self.full, self.score, self.held = False, 0.0, 0
        self.failed = False
        self.trail = collections.deque(maxlen=HOLD_STEPS + 1)
        self.peaks = {"outer_Nm": 0.0, "roll_Nm": 0.0, "tab_N": 0.0}
        self.seen = None
        if env is not None and os.environ.get("MAILER_LOG"):
            box = m.state(env, "mailer")
            print(f"[mailer] at reset: box at {np.round((box.pos - [BOX_AT[0], BOX_AT[1], TOP]) * 1000, 1)} mm, x axis {np.round(box.rotation()[:, 0], 3)}, joints "
                  f"{ {k: round(v, 3) for k, v in box.joints.items()} }", flush=True)

    def _spring(self, env, link: str, k: float, rest_deg: float, effort: float) -> None:
        asset, j = m._joint(env, "mailer", link)
        m._TOUCHED.add(("mailer", link))
        asset.write_joint_effort_limit_to_sim(m._tensor(asset, effort), joint_ids=[j])
        asset.write_joint_stiffness_to_sim(m._tensor(asset, k), joint_ids=[j])
        asset.set_joint_position_target(m._tensor(asset, math.radians(rest_deg)), joint_ids=[j])

    def read(self, env) -> Seen:
        box = m.state(env, "mailer")
        if self.start is None:
            self.start = box.pos.copy()
        tips = m.fingertips(env)
        local = box.to_local(tips)
        lo, hi = np.array([-0.025, -WY / 2, 0.0]), np.array([LX, WY / 2 + LID_RUN + 0.045, H + 0.012])  # the box, its rolled top's round and the outer panel's lean, the open lid and its tuck
        gap = np.linalg.norm(np.maximum(np.maximum(lo - local, local - hi), 0.0), axis=1).min()
        edge = box.to_local(on_link(env, "roll", (T / 2, 0.0, P_TOP)))[0]
        tl = box.to_local(on_link(env, "tab_l", (T / 2, TAB_Y, P_TOP + TAB_OUT)))[0]
        tr = box.to_local(on_link(env, "tab_r", (T / 2, -TAB_Y, P_TOP + TAB_OUT)))[0]
        return Seen(box.deg("outer"), box.deg("roll"), edge, (tl, tr), (box.deg("ear_l"), box.deg("ear_r")), float(np.linalg.norm(box.pos - self.start)),
                    box.tilt_deg(), float(gap), math.degrees(math.atan2(box.rotation()[1, 0], box.rotation()[0, 0])))

    def step(self, env) -> None:
        self.steps += 1
        if self.steps == 1:
            self._spring(env, "outer", K_OUTER, START_DEG, 20.0)
        box = m.state(env, "mailer")
        s = self.seen = self.read(env)
        self.failed = self.failed or failed(s)
        for k, y in enumerate((TAB_Y, -TAB_Y)):
            self.near[k] = self.near[k] + 1 if in_hole(s.tips[k], y) else 0
            if self.near[k] >= 3 and not self.latched[k] and not self.failed and s.roll_deg >= 150.0:
                self.latched[k] = True
                m.lock(env, "mailer", ("tab_l", "tab_r")[k])
                print(f"[mailer] {'left' if k == 0 else 'right'} tab latched in its hole at step {self.steps}: tip {np.round(s.tips[k] * 1000, 1)} mm, "
                      f"outer {s.outer_deg:.1f} deg, roll {s.roll_deg:.1f} deg", flush=True)
        if all(self.latched) and not self.locked:
            for link in ("roll", "outer"):
                m.lock(env, "mailer", link)
            self.locked = True
            print(f"[mailer] both tabs in: the end is locked (step {self.steps})", flush=True)
        q = box.joints
        self.peaks["outer_Nm"] = max(self.peaks["outer_Nm"], K_OUTER * math.radians(s.outer_deg - START_DEG))
        self.peaks["roll_Nm"] = max(self.peaks["roll_Nm"], K_ROLL * math.radians(max(0.0, s.roll_deg)))
        self.peaks["tab_N"] = max(self.peaks["tab_N"], K_TAB * max(-q["tab_l_joint"], -q["tab_r_joint"], 0.0))
        self.trail.append(np.r_[box.pos, H * math.radians(s.outer_deg), 0.15 * math.radians(s.roll_deg)])
        arr = np.array(self.trail)
        still = len(arr) > HOLD_STEPS and float(np.abs(arr - arr[-1]).max()) <= 0.001
        full, _ = judge(s, tuple(self.latched), still, self.held + 1, self.failed)
        self.held = self.held + 1 if full else 0
        self.full, self.score = judge(s, tuple(self.latched), still, self.held, self.failed)
        if os.environ.get("MAILER_LOG") and self.steps % int(os.environ["MAILER_LOG"]) == 0:
            yaw = math.degrees(math.atan2(box.rotation()[1, 0], box.rotation()[0, 0]))
            print(f"[mailer] step {self.steps}: box x axis {np.round(box.rotation()[:, 0], 3)} y axis {np.round(box.rotation()[:, 1], 3)}; box at {np.round((box.pos - [BOX_AT[0], BOX_AT[1], TOP]) * 1000, 1)} mm, yaw {yaw:.1f} tilt {s.tilt_deg:.1f} deg, outer {s.outer_deg:.1f} roll {s.roll_deg:.1f} tabs {q['tab_l_joint'] * 1000:.1f} {q['tab_r_joint'] * 1000:.1f} mm "
                  f"edge {np.round(s.edge * 1000, 0)} tips z {s.tips[0][2] * 1000:.1f} {s.tips[1][2] * 1000:.1f} ears {s.ears_deg[0]:.1f} {s.ears_deg[1]:.1f} "
                  f"box moved {s.moved * 1000:.1f} mm, clear {s.clear * 1000:.0f} mm, latched {self.latched}, score {self.score}", flush=True)
        if self.held == HOLD_STEPS:
            print(f"[mailer] full state held {HOLD_STEPS * DT:.1f} s: score {self.score}; spring peaks {self.peaks}", flush=True)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    """Both tabs latched in their holes, the inner panel rolled flat inside, the wall within 5 degrees of upright with both tuck flaps
    inside it, the hands 3 cm clear, nothing moving, held 1 s."""
    return TRIAL.score >= 1.0


def score(objects) -> float:
    return TRIAL.score


# -- the demo's paths ---------------------------------------------------------------------------------------------------------------

TIP = 0.016  # the closed fingers' ends stand this far past the TCP along the fingers
SIDE = 0.020  # and their broad side this far beside it
PRESS = 0.003  # how far into a face the fingers aim, so that they push
HOLD_PRESS = 0.015  # the left hand aims this far past the upright wall's face, so that it holds the wall hard on its stop
DOWN_PRESS, FLAT_PRESS = 0.015, float(os.environ.get("MAILER_FLAT", "0.06"))
LEAD = float(os.environ.get("MAILER_LEAD", "30"))  # degrees: how far ahead of the panel the right hand aims while it presses it down inside
OVER = 2.0  # and past flat at the end, so that it holds the panel on its stop  # the right hand aims deeper as the panel goes down inside, and deepest to press it flat: the arm gives under load
HOLD_Z, HOLD_Y = 0.135, 0.07  # where the left hand holds the outer panel
ROLL_R, ROLL_Y = float(os.environ.get("MAILER_ROLL_R", "0.02")), -0.06  # where the right hand pushes the inner panel: out along it from the hinge (its palm then clears the wall's top), and across
LEFT_VIA, RIGHT_VIA = (0.18, 0.24, TOP + 0.22), (0.18, -0.24, TOP + 0.26)  # from READY, over the box, round the unrolled panel's sides


def rot_y(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def hold_tcp(origin, outer_deg: float, press: float = PRESS) -> np.ndarray:
    """The left TCP (fingers level toward +x) with its fingers' ends on the outer panel's face, the panel at outer_deg."""
    face = rot_y(outer_deg) @ np.array([0.0, HOLD_Y, HOLD_Z])
    normal = rot_y(outer_deg) @ np.array([-1.0, 0.0, 0.0])
    return np.asarray(origin, float) + face + normal * (TIP - press)


TILT_MIN = 30.0  # over the box the fingers stay tilted this far toward +x: straight down, the wrist (joint 4) is at its limit up there


def roll_tilt(theta: float) -> float:
    """The fingers' tilt from straight down toward +x at roll angle theta: with the panel's face down to TILT_MIN, then held, then
    easing to straight down as the panel goes down inside, where the hand is low enough for it."""
    if theta >= TILT_MIN:
        return theta
    if theta >= 0.0:
        return TILT_MIN
    return TILT_MIN * (1.0 + theta / 90.0)


ROLL_YAW = float(os.environ.get("MAILER_YAW", "90"))  # the right hand's turn about its fingers through the roll


def roll_grip(theta: float) -> np.ndarray:
    return rot_y(-roll_tilt(theta)) @ d.down(ROLL_YAW).rot


def roll_tcp(axis, theta: float, press: float = PRESS) -> np.ndarray:
    """The right TCP at roll angle theta (deg; 90: the inner panel straight up, 0: flat over the box, -90: down inside), with the
    closed fingers on the panel's outside face ROLL_R out from the hinge: end-on while the panel is up, then the corner of their
    ends, then their side as it goes down inside. The TCP stands off the face by the fingers' extent toward it."""
    t = math.radians(theta)
    u = np.array([math.cos(t), 0.0, math.sin(t)])
    n = np.array([-math.sin(t), 0.0, math.cos(t)])  # the outside face's normal
    face = np.asarray(axis, float) + np.array([0.0, ROLL_Y, 0.0]) + ROLL_R * u + XA * n
    tau = math.radians(roll_tilt(theta))
    a = np.array([math.sin(tau), 0.0, -math.cos(tau)])  # where the fingers point
    b = np.array([math.cos(tau), 0.0, math.sin(tau)])  # across them, in the same plane
    reach = TIP * max(0.0, float(-n @ a)) + SIDE * abs(float(n @ b))
    return face + n * (reach - press)


HOLD_GRIP = d.level(0.0)
OUT_GRIP = d.tilted(45.0, 0.0)  # the right hand comes up out of the box with its fingers tilted forward
LEFT_AWAY, RIGHT_AWAY = (0.20, 0.25, TOP + 0.25), (0.25, -0.25, TOP + 0.25)


def nominal_axis() -> np.ndarray:
    return np.array([BOX_AT[0] + XA, 0.0, TOP + ZA])


# -- demo ---------------------------------------------------------------------------------------------------------------------------


def demo_script(ctx):
    """Modes: full. look: the arms wait, for a still. unheld: the left hand stands the wall up, holds it 1 s and lets go (it springs
    back). onehand: the right hand's whole roll with no hand on the wall."""
    mode = ctx.mode
    l, r = ctx.left, ctx.right
    env = ctx.emb._env
    if mode == "look":
        yield from l.wait(3.0)
        return
    if mode not in ("full", "unheld", "onehand"):
        raise ValueError(f"unknown mode {mode!r}")
    box = ctx.state("mailer")
    origin = box.pos  # the crease's middle on the table
    shut = d.FINGER_CLOSED  # the fingers close on the way to the box: demokit's close() at READY asks for a straight-down grip there, past joint 4

    if mode in ("full", "unheld"):
        # 1. The left hand comes to the leaning wall's face and stands it upright, pressing it against the tuck flaps.
        yield from l.move(LEFT_VIA, grip=HOLD_GRIP, finger=shut, t=1.2, label="round the panel")
        yield from l.move(hold_tcp(origin, START_DEG, press=-0.03), t=1.0, label="to the wall")
        yield from l.move(hold_tcp(origin, START_DEG, press=0.0), t=0.5, label="on the wall")
        steps = 24
        for k in range(1, steps + 1):
            a = START_DEG * (1.0 - d.ik.smoothstep(k / steps))
            mm, deg = l._solve(hold_tcp(origin, a, press=HOLD_PRESS * k / steps), HOLD_GRIP.rot)
            l._note(mm, deg, "stand the wall up")
            yield
        yield from l.wait(0.6)
        print(f"[mailer] wall held at {ctx.deg('mailer', 'outer'):.1f} deg", flush=True)
        if mode == "unheld":
            yield from l.wait(0.4)
            yield from l.move(hold_tcp(origin, START_DEG, press=-0.06), t=0.4, label="let go")
            for _ in range(3):
                yield from l.wait(0.5)
                print(f"[mailer] let go: wall at {ctx.deg('mailer', 'outer'):.1f} deg", flush=True)
            yield from l.move(LEFT_AWAY, t=1.2, label="away")
            return

    # 2. The right hand comes in front of the inner panel, fingers level toward it, and rolls it over the top: an arc about the hinge.
    axis = roll_axis(env)
    yield from r.move(RIGHT_VIA, grip=d._as_grip(roll_grip(90.0)), finger=shut, t=1.2, label="round the panel")
    yield from r.move(roll_tcp(axis, 90.0, press=-0.05) + [0.0, 0.0, 0.03], t=0.8, label="before the panel")
    yield from r.move(roll_tcp(axis, 90.0, press=-0.01), t=0.6, label="at the panel")
    for k, th in enumerate(np.linspace(90.0, -40.0, 66)):
        axis = roll_axis(env)  # the hinge moves a little as the left hand's hold gives
        press = PRESS if th >= 0.0 else PRESS + (DOWN_PRESS - PRESS) * (-th / 90.0)
        want = roll_tcp(axis, th, press=press)
        mm, deg = r._solve(want, roll_grip(th))
        r._note(mm, deg, "roll the panel")
        yield
        if os.environ.get("MAILER_LOG") and k % 5 == 0:
            local = ctx.state("mailer").to_local(np.vstack([want, r.tcp()]))
            print(f"[mailer] roll {th:.0f}: TCP wanted {np.round(local[0] * 1000, 0)} seen {np.round(local[1] * 1000, 0)} mm (box frame), panel at {ctx.deg('mailer', 'roll'):.1f} deg", flush=True)
    # 3. Down inside, the hand follows the panel as it is: it aims LEAD degrees ahead of where the panel is, pressing into its face
    # along the face's normal, so that the push stays square to the panel and the soft arm's fingers do not slide up it to the crease.
    # Then it holds the panel flat on its hinge's stop until both tabs have snapped into their holes.
    aim = -40.0
    for k in range(round(6.0 / DT)):
        axis = roll_axis(env)
        now = 90.0 - ctx.deg("mailer", "roll")
        if not TRIAL.locked:
            aim = max(now - LEAD, aim - 2.0, -90.0 - OVER)  # 2 degrees a step, up to LEAD ahead of the panel
        press = DOWN_PRESS + (FLAT_PRESS - DOWN_PRESS) * max(0.0, (-aim - 80.0) / (10.0 + OVER))
        want = roll_tcp(axis, aim, press=press)
        mm, deg = r._solve(want, roll_grip(max(aim, -90.0)))
        r._note(mm, deg, "press the panel down inside")
        yield
        if os.environ.get("MAILER_LOG") and k % 5 == 0:
            local = ctx.state("mailer").to_local(np.vstack([want, r.tcp()]))
            print(f"[mailer] follow: aim {aim:.0f}, TCP wanted {np.round(local[0] * 1000, 0)} seen {np.round(local[1] * 1000, 0)} mm, panel at {ctx.deg('mailer', 'roll'):.1f} deg", flush=True)
            if k % 20 == 0:
                tips = ctx.state("mailer").to_local(m.fingertips(env)[12:])
                print(f"[mailer] right fingertips (box frame, mm): x {tips[:, 0].min() * 1000:.0f}..{tips[:, 0].max() * 1000:.0f} "
                      f"y {tips[:, 1].min() * 1000:.0f}..{tips[:, 1].max() * 1000:.0f} z {tips[:, 2].min() * 1000:.0f}..{tips[:, 2].max() * 1000:.0f}; "
                      f"TCP seen {np.round(local[1] * 1000, 0)}", flush=True)
        if TRIAL.locked and k >= 10:
            for _ in range(5):
                yield
            break
    print(f"[mailer] pressed: roll {ctx.deg('mailer', 'roll'):.1f} deg, outer {ctx.deg('mailer', 'outer'):.1f} deg, tabs latched {TRIAL.latched}; "
          f"spring peaks {TRIAL.peaks}", flush=True)
    # 4. The last degrees: straight-down fingers can't bring the panel flat (their hand meets the rolled top first), so the hand
    # backs off and rises with its fingers tilting forward: the back of the hand swings toward the wall and presses the panel's top
    # flat, and both tabs snap in. It waits for that before it leaves.
    yield from r.move(r.at() + [0.03, 0.0, 0.0], t=0.4, label="off the panel")
    yield from r.move(r.at() + [0.03, 0.0, 0.14], grip=OUT_GRIP, t=1.0, label="press it flat with the back of the hand, rising")
    for _ in range(30):
        if TRIAL.locked:
            break
        yield
    print(f"[mailer] after the press: roll {ctx.deg('mailer', 'roll'):.1f} deg, tabs latched {TRIAL.latched}", flush=True)
    if mode == "full":
        yield from ctx.together(r.move(RIGHT_AWAY, t=1.2, label="away"), l.move(l.at() + [-0.06, 0.0, 0.0], t=0.6, label="off the wall"))
        yield from l.move(LEFT_AWAY, t=1.0, label="away")
    else:
        yield from r.move(RIGHT_AWAY, t=1.2, label="away")
    print(f"[mailer] done: outer {ctx.deg('mailer', 'outer'):.1f} deg, roll {ctx.deg('mailer', 'roll'):.1f} deg, latched {TRIAL.latched}, score {TRIAL.score}", flush=True)
    yield from l.wait(1.6)
