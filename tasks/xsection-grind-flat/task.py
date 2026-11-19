"""Grind a mounted chip package's cross-section flat by hand on a turning abrasive disc, down to the line scribed round the mount.

A benchtop grinder-polisher: one 203 mm platen with P180 silicon-carbide paper, turning anticlockwise (seen from above) at 150 rpm in a
splash bowl. A 32 mm epoxy mount, 35 mm tall, holding a strip sawn from a BGA package edge-on, stands face down on a rest, its upper end
clamped in a hand holder: a cup with a flange and a knob with two flats on top. One hand holds the mount by the knob, face down on the
turning paper, presses it, sweeps it between the platen's centre and edge and keeps it flat, until the face reaches the line scribed
0.3 mm above it; then it sets the mount back on its rest and moves clear. Each finger is capped at 40 N.

Removal is a rule (a simulator can't remove material): Preston's law with a threshold pressure, on a 2 mm grid over the face.
  - The push is the force the platen really carries: the platen rides on a stiff vertical spring (20 N/mm), and its deflection gives it.
  - That force is spread over the face by the mount's real tilt on the paper (an elastic paper: 0.1 mm under a 15 N flat push, assumed),
    so a tilted hold presses one edge and grinds a facet there.
  - Each point loses height at K x (pressure - threshold) x its speed over the paper, the speed from the platen's turn at that point.
  - Tuned to the real hand rate of about 0.1 mm per minute: a flat face loses 0.3 mm in about 120 s under 15 N, in about 190 s under 11 N; under 5 N little (threshold 4.5 kPa, about 3.6 N over the face).
The mount's collider keeps its full length; the removed depth is kept in the height map. The scribed line drawn on the mount's side moves
down as the mean stock left shrinks (0.05 mm steps), as the face rises to the line on a real mount.
Scored as a real trial measures it, before and after: the removal at 24 marked stops every 15 degrees round the circle 2 mm in from the rim (the
simulation reads 72, every 5 degrees, so no gap between stops hides a dip), and at the centre. Any of them over the cap fails the trial.

    python demokit.py task.py OUT --no-record --mode full,none,tilted,oneside,light,over,leave,keep,hover
    python demokit.py task.py OUT                                      final film
"""

from __future__ import annotations

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
INSTRUCTION = ("Grind the mounted section on the turning paper down to the line scribed round the mount. Hold the mount face down on the "
               "paper and press it steadily, moving it back and forth between the centre and the edge of the disc and keeping its face flat, "
               "until the face reaches the line all the way round. Then lift it off the paper, set it back on its rest as it started, and move your hand clear.")
SUCCESS_HOLD_S = 0.0  # the scorer holds the end state itself (EndHold)
SCORE_OVER = "end"
TOP = d.TABLE_TOP

# -- the grinder-polisher (positions in the station frame, metres; heights above the table top) ---------------------------------
PLATEN_XY = (0.28, -0.16)
BODY_H = 0.10  # the machine body's top, which is the bowl's floor
PAPER_Z = 0.12  # the paper's top above the table
PLATEN_R = 0.1015  # 8 in
BOWL_R, BOWL_RIM = 0.15, 0.16  # the splash bowl: 30 cm across, its rim 2 cm above the paper
RPM = 150.0
OMEGA = RPM * 2 * math.pi / 60.0  # rad/s, anticlockwise from above
CELL_K = 20000.0  # N/m: the platen's spring, which measures the push on it
FRICTION = 0.4  # the paper on the mount, wet (assumed, 0.3 to 0.7)
PAPER_MAT = MOUNT_MAT = FRICTION

# -- the mount and its rest ------------------------------------------------------------------------------------------------------
MOUNT_R, MOUNT_H, MOUNT_KG = 0.016, 0.035, 0.040
# the hand holder (printed PETG, sizes ours): a cup over the mount's upper 15 mm, closed by a 4 mm top whose 60 mm flange shields the hand
# from spray, and on it a knob 26 mm long and 20 mm across two flats, 18 mm tall. The fingers close on the flats with their tips just over
# the flange, so the push goes down through the flange, not through the pads' friction.
CUP_R, CUP_Z0, FLANGE_R, FLANGE_Z0, FLANGE_Z1 = 0.022, 0.020, 0.030, MOUNT_H, MOUNT_H + 0.004
KNOB = (0.026, 0.020, 0.018)  # x (along the flats), y (across the flats, the jaws' axis), z
KNOB_TOP = FLANGE_Z1 + KNOB[2]
HOLDER_KG = 0.035
FINGER_N_MAX = 40.0  # N: each finger's force is capped here (Robocurve's figure for the YAM)
REST_XY, REST_H, REST_HALF = (0.20, -0.42), 0.04, 0.025
LINE_MM = 0.30  # the scribed line above the face (the last 0.3 mm of the P180 step)

# -- removal (Preston's law with a threshold) ----------------------------------------------------------------------------------------
GRID_MM = 2.0
FACE_A = math.pi * MOUNT_R ** 2
K_WINKLER = (15.0 / FACE_A) / 100e-6  # Pa/m: the paper and its backing give 0.1 mm under a 15 N flat push (assumed)
P_THRESH = 4500.0  # Pa: below this the grains rub and do not cut (about 3.6 N over the face)
V_NOMINAL = 0.9  # m/s, a typical speed over the paper while sweeping 3.5 to 8 cm out
K_PRESTON = 0.0025e-3 / ((15.0 / FACE_A - P_THRESH) * V_NOMINAL)  # 1/Pa: 0.15 mm/min at 15 N, 0.1 mm/min at 11 N
GROUND_MM = 0.05  # a point counts as ground once this much is gone
RING_MM = 14.0  # the ring: the circle 2 mm in from the rim, read at RING_N stops from the index line scribed down the mount's side; the centre is read too
RING_N = 72  # the simulation reads every 5 degrees (the real trial's 24 stops every 15 degrees are every third of these)
INDEX_DEG = float(os.environ.get("GRIND_INDEX_DEG", "0"))  # where the index line sits round the mount: 0 in the demo; the lazy-route run 'spot' sets 15
V_DROP = 0.3  # m/s: a mount this fast with no hand near it has been dropped or thrown
F_MAX = 25.0  # N: a push beyond this is not a hand pressing a mount (machine heads load 18 to 27 N)
CONTACT_GAP = 0.0015  # m: the mount's lowest point this close to the paper is touching it (the paper and the gripper's grip give a little)
LOW_MM, HIGH_MM, SPREAD_MM = 0.23, 0.37, 0.10
CLEAR_M = 0.03  # the hand clear of the mount at the end
TIP_BELOW = 0.004  # fingertips() points sit about 5 mm below the fingers' real underside (kit README 22): 4 mm below the paper is a finger on it
STILL_STEPS = 20  # 1 s at 20 Hz


def _grid():
    """Face points in the mount's frame (mm): a 2 mm grid over the face (the contact model), then the scored points (the ring and the centre)."""
    g = np.arange(-15.0, 16.0, GRID_MM)
    uu, vv = np.meshgrid(g, g)
    keep = uu ** 2 + vv ** 2 <= (MOUNT_R * 1000) ** 2
    grid = np.c_[uu[keep], vv[keep]]
    ang = np.radians(INDEX_DEG + np.arange(RING_N) * 360.0 / RING_N)
    scored = np.vstack([np.c_[RING_MM * np.cos(ang), RING_MM * np.sin(ang)], [[0.0, 0.0]]])  # the ring, then the centre
    return grid, scored


GRID, SCORED = _grid()
POINTS = np.vstack([GRID, SCORED]) / 1000.0  # metres, mount frame
N_GRID, N_SCORED = len(GRID), len(SCORED)  # the last scored point is the centre
AREA = np.r_[np.full(N_GRID, (GRID_MM / 1000) ** 2), np.zeros(len(POINTS) - N_GRID)]
LINE_STEPS = 7  # lines drawn at 0.30, 0.25, ... 0.0 mm above the face; one shown at a time

LOOKS = {
    "body": m.Look(m.rgb(0.80, 0.81, 0.80), roughness=0.5),
    "bowl": m.Look(m.rgb(0.20, 0.22, 0.25), roughness=0.4),
    "alu": m.Look(m.rgb(0.70, 0.72, 0.74), metallic=0.8, roughness=0.35),
    "paper": m.Look(m.rgb(0.16, 0.16, 0.17), roughness=0.95),
    "mark": m.Look(m.rgb(0.85, 0.85, 0.80), roughness=0.9),
    "epoxy": m.Look(m.rgb(0.92, 0.80, 0.45), roughness=0.3),
    "strip": m.Look(m.rgb(0.10, 0.28, 0.12), roughness=0.6),
    "line": m.Look(m.rgb(0.75, 0.05, 0.05), roughness=0.6),
    "pla": m.Look(m.rgb(0.25, 0.45, 0.85), roughness=0.7),
    "holder": m.Look(m.rgb(0.30, 0.30, 0.32), roughness=0.6),
}


def _ring(r0, r1, z0, z1, segments=48):
    return m.lathe([(r0, z0), (r1, z0), (r1, z1), (r0, z1), (r0, z0)], segments=segments)


def _grinder():
    body_z = BODY_H
    frame = [
        ("bowl_floor", m.cylinder(BOWL_RIM, 0.004, (0, 0, 0.002), segments=64), "bowl"),
        ("bowl_wall", _ring(BOWL_R, BOWL_RIM, 0.0, PAPER_Z - body_z + 0.02, 64), "bowl"),
    ]
    cols = [m.col_cylinder("floor_c", BOWL_RIM, 0.004, (0, 0, 0.002))]
    wall_h = PAPER_Z - body_z + 0.02
    c = 0.5 * (BOWL_R + BOWL_RIM)
    for k in range(24):  # the bowl's wall as 24 boxes round it, each a hull so it can be turned
        seg = m.box((BOWL_RIM - BOWL_R, 2 * c * math.tan(math.pi / 24) + 0.002, wall_h), (c, 0, wall_h / 2)).turned(15.0 * k, "z")
        cols.append(m.col_hull(f"wall_{k}", seg))
    paper_t = 0.0005
    disc_h = PAPER_Z - body_z - 0.004 - paper_t  # the aluminium disc from the bowl floor's top up to the paper
    disc = [
        ("disc", m.cylinder(PLATEN_R, disc_h, (0, 0, 0.004 + disc_h / 2), segments=72), "alu"),
        ("paper", m.cylinder(PLATEN_R - 0.0005, paper_t, (0, 0, PAPER_Z - body_z - paper_t / 2), segments=72), "paper"),
        ("mark", m.box((0.07, 0.006, 0.0004), (0.055, 0, PAPER_Z - body_z + 0.0002)), "mark"),
    ]
    platen = m.hinge("platen", "Z", (0, 0, 0), -3.6e6, 3.6e6, disc, [m.col_cylinder("platen_c", PLATEN_R, disc_h + paper_t, (0, 0, 0.004 + (disc_h + paper_t) / 2))],
                     mass=1.0, damping=30.0, effort=20.0, friction=0.0, armature=1e-4, parent="cell",
                     inertia=(0.0026, 0.0026, 0.0052))
    cell = m.slider("cell", "Z", -0.004, 0.001, (), (), mass=0.3, spring=CELL_K, damping=150.0, effort=500.0, friction=0.0, armature=1e-4)
    return m.mechanism("grinder", (*PLATEN_XY, TOP + body_z), [cell, platen], {k: LOOKS[k] for k in ("bowl", "alu", "paper", "mark")},
                       frame_parts=frame, frame_colliders=cols, friction=(PAPER_MAT, PAPER_MAT))


def _mount():
    parts = [("epoxy", m.cylinder(MOUNT_R, MOUNT_H, (0, 0, MOUNT_H / 2), segments=48), "epoxy"),
             ("strip", m.box((0.020, 0.00286, 0.012), (0, 0, 0.0059)), "strip")]
    for k in range(LINE_STEPS):  # line_k sits k twentieths of a mm above the face; line_6 (0.3 mm) shows at the start
        z = k * 0.00005
        parts.append((f"line_{k}", _ring(MOUNT_R - 0.0002, MOUNT_R + 0.00025, z + 0.0003, z + 0.0011), "line", k != LINE_STEPS - 1))
    cup_h = FLANGE_Z0 - CUP_Z0
    parts += [("cup", _ring(MOUNT_R, CUP_R, CUP_Z0, FLANGE_Z0, 48), "holder"),
              ("flange", m.cylinder(FLANGE_R, FLANGE_Z1 - FLANGE_Z0, (0, 0, 0.5 * (FLANGE_Z0 + FLANGE_Z1)), segments=48), "holder"),
              ("knob", m.box(KNOB, (0, 0, FLANGE_Z1 + KNOB[2] / 2)), "holder")]
    cols = [m.col_cylinder("c", MOUNT_R, MOUNT_H, (0, 0, MOUNT_H / 2)),
            m.col_cylinder("cup_c", CUP_R, cup_h, (0, 0, CUP_Z0 + cup_h / 2)),
            m.col_cylinder("flange_c", FLANGE_R, FLANGE_Z1 - FLANGE_Z0, (0, 0, 0.5 * (FLANGE_Z0 + FLANGE_Z1))),
            m.col_box("knob_c", KNOB, (0, 0, FLANGE_Z1 + KNOB[2] / 2))]
    return m.free("mount", (*REST_XY, TOP + REST_H), parts, cols, {k: LOOKS[k] for k in ("epoxy", "strip", "line", "holder")},
                  mass=MOUNT_KG + HOLDER_KG, friction=(MOUNT_MAT, MOUNT_MAT))


OBJECTS = [
    m.fixed("machine", (PLATEN_XY[0] + 0.05, PLATEN_XY[1], TOP), [("body", m.box((0.40, 0.30, BODY_H), (0, 0, BODY_H / 2)), "body")],
            [m.col_box("c", (0.40, 0.30, BODY_H), (0, 0, BODY_H / 2))], {"body": LOOKS["body"]}),
    _grinder(),
    m.fixed("rest", (*REST_XY, TOP), [("block", m.box((2 * REST_HALF, 2 * REST_HALF, REST_H), (0, 0, REST_H / 2)), "pla")],
            [m.col_box("c", (2 * REST_HALF, 2 * REST_HALF, REST_H), (0, 0, REST_H / 2))], {"pla": LOOKS["pla"]}),
    _mount(),
]


# -- the removal rule ------------------------------------------------------------------------------------------------------------------


def contact_pressure(h, area, force):
    """Pressure (Pa) at each face point from its height `h` above the paper (m), for a total push `force` (N): a stiff elastic paper (Winkler),
    each point pressed in by how far the face's contact level `c` lies above it. Points with no area (the score points) take the same law."""
    h = np.asarray(h, float)
    if force <= 0.0 or not np.isfinite(h).any():
        return np.zeros_like(h)
    lo = float(np.nanmin(h[area > 0])) if (area > 0).any() else float(np.nanmin(h))
    hi = lo + force / (K_WINKLER * max(float(area[area > 0].min()), 1e-12))
    a = np.where(np.isfinite(h), area, 0.0)
    hh = np.where(np.isfinite(h), h, np.inf)
    for _ in range(50):
        c = 0.5 * (lo + hi)
        f = float((K_WINKLER * a * np.clip(c - hh, 0.0, None)).sum())
        lo, hi = (c, hi) if f < force else (lo, c)
    c = 0.5 * (lo + hi)
    return K_WINKLER * np.clip(c - hh, 0.0, None)


def removal_rate(pressure, speed):
    """Preston's law with a threshold: m/s lost at each point."""
    return K_PRESTON * np.clip(np.asarray(pressure) - P_THRESH, 0.0, None) * np.asarray(speed)


def rel_speed(v_plat, v_mount):
    """Speed of the paper under each face point: the platen's own speed at that point less the mount's motion, never more than the platen's
    own (shaking the mount against the platen cannot cut faster than the abrasive's own travel)."""
    v_plat, v_mount = np.asarray(v_plat), np.asarray(v_mount)
    return np.minimum(np.hypot(*(v_plat - v_mount).T), np.hypot(*v_plat.T))


def loaded_elsewhere(force, in_contact):
    """The platen loaded by something other than the mount (a push with the mount not touching the paper), or beyond any hand's push."""
    return bool((force > 2.0 and not in_contact) or force > F_MAX)


def dropped(clear, speed, z, over_rest, rest_z):
    """The mount let go: no fingertip near it and either moving fast (dropped or flicked) or sitting higher than the rest off the rest."""
    return bool(clear and (speed > V_DROP or (z > rest_z + 0.01 and not over_rest)))


def scored_points(depth_mm):
    """Removal (mm) at the scored points: the 72 ring stops 2 mm in from the rim (every 5 degrees from the index line), then the centre."""
    return np.asarray(depth_mm)[N_GRID:N_GRID + N_SCORED]


def rungs(depth_mm, on_rest=False, clear=False, still_done=False, thrown=False, touched=False, was_over=False):
    """The scoring text's table from the removal round the ring and at the centre (mm, read from `depth_mm` in POINTS order) and the end-state flags.
    Returns (score, reached, failed)."""
    five = scored_points(depth_mm)  # every ring stop and the centre
    spread = float(five.max() - five.min())
    over = bool(five.max() > HIGH_MM)  # overgrinding anywhere on the ring or at the centre
    failed = over or was_over or thrown or touched  # a failure stays failed
    some = bool(five.max() >= GROUND_MM)
    every = bool(five.min() >= GROUND_MM and spread <= SPREAD_MM)
    line = bool(every and five.min() >= LOW_MM and five.max() <= HIGH_MM)
    done = bool(line and on_rest and clear and still_done)
    reached = {"ground": some, "plane": every, "line": line, "done": done}
    return LADDER.score(reached, failed), reached, failed


LADDER = gk.Ladder(("ground", 0.25), ("plane", 0.5), ("line", 0.75), ("done", 1.0))


class Trial:
    """The grinding rule and the scorer's state across a trial."""

    def reset(self, env=None) -> None:
        self.depth = np.zeros(len(POINTS))  # m removed at each face point
        self.n = 0
        self.off_n = 0
        self.force = 0.0
        self.peak_force = 0.0
        self.thrown, self.touched, self.over = gk.Latch(), gk.Latch(), gk.Latch()
        self.hold = gk.EndHold(STILL_STEPS)
        self.shown = LINE_STEPS - 1
        self.score, self.full = 0.0, False
        self.rep = {}
        if env is not None:
            self._show(env, LINE_STEPS - 1)
            import torch

            for side in ("left", "right"):  # each finger's force capped at 40 N, Robocurve's figure for the YAM (the harness allows 50)
                robot = env.unwrapped.scene[f"{side}_robot"]
                ids = robot.find_joints(["joint7", "joint8"])[0]
                robot.write_joint_effort_limit_to_sim(torch.full((1, len(ids)), FINGER_N_MAX, device=robot.device), joint_ids=ids)

    def _show(self, env, k: int) -> None:
        for j in range(LINE_STEPS):
            m.set_visible(f"{m.ENV_PATH}/mount/visual/line_{j}", j == k)
        self.shown = k

    def step(self, env) -> None:
        self.n += 1
        g = m.state(env, "grinder")
        asset, j = m._joint(env, "grinder", "platen")
        asset.set_joint_velocity_target(m._tensor(asset, OMEGA), joint_ids=[j])
        omega = float(m._np(asset.data.joint_vel)[0, j])
        cell = g.joint("cell")
        self.force = max(0.0, -CELL_K * cell)
        self.peak_force = max(self.peak_force, self.force)
        paper_z = TOP + PAPER_Z + cell
        mount = m.state(env, "mount")
        R = mount.rotation()
        local = np.c_[POINTS, self.depth]
        world = mount.pos + local @ R.T
        h = world[:, 2] - paper_z
        rel = world[:, :2] - np.array(PLATEN_XY)
        on_paper = np.hypot(rel[:, 0], rel[:, 1]) <= PLATEN_R
        face_down = R[2, 2] >= math.cos(math.radians(30.0))
        gap = mount.pos[2] - MOUNT_R * math.sqrt(max(0.0, 1.0 - R[2, 2] ** 2)) - paper_z  # the mount's lowest point over the paper
        in_contact = bool(gap <= CONTACT_GAP and math.hypot(*(mount.pos[:2] - np.array(PLATEN_XY))) <= PLATEN_R + MOUNT_R)
        pressing = in_contact and face_down and self.force > 0.2 and on_paper[:N_GRID].any() and float(np.where(on_paper[:N_GRID], h[:N_GRID], np.inf).min()) < 0.003
        rate = np.zeros(len(POINTS))
        if pressing:
            hh = np.where(on_paper, h, np.nan)
            p = contact_pressure(hh, AREA, self.force)
            v_plat = omega * np.c_[-rel[:, 1], rel[:, 0]]
            v_mount = mount.lin_vel[:2] + np.cross(mount.ang_vel, world - mount.pos)[:, :2]
            speed = rel_speed(v_plat, v_mount)
            rate = removal_rate(p, speed)
            self.depth += rate * d.DT
        mm = self.depth * 1000.0
        left = max(0.0, LINE_MM - float(mm[:N_GRID].mean()))
        k = int(min(LINE_STEPS - 1, math.ceil(round(left * 20.0, 6))))
        if k != self.shown:
            self._show(env, k)
        tips = m.fingertips(env)
        trel = tips[:, :2] - np.array(PLATEN_XY)
        pushed_elsewhere = loaded_elsewhere(self.force, in_contact)
        self.off_n = self.off_n + 1 if pushed_elsewhere else 0  # a lone spike while the mount rocks is not a second body: it must last 0.5 s
        self.touched.trip(self.off_n >= 10)
        self.touched.trip(bool(((np.hypot(trel[:, 0], trel[:, 1]) <= PLATEN_R + 0.01) & (tips[:, 2] < paper_z - TIP_BELOW)).any()))
        near_platen = math.hypot(*(mount.pos[:2] - np.array(PLATEN_XY))) <= BOWL_RIM
        dx0, dy0 = np.abs(mount.pos[:2] - np.array(REST_XY))
        clear0 = gk.hands_clear(tips, gk.along(mount, (0, 0, 0), (0, 0, KNOB_TOP), 9), CLEAR_M, radius=CUP_R)
        self.thrown.trip(dropped(clear0, float(np.linalg.norm(mount.lin_vel)), mount.pos[2], bool(dx0 <= REST_HALF and dy0 <= REST_HALF), TOP + REST_H))
        self.thrown.trip(bool(R[2, 2] < math.cos(math.radians(45.0)) or mount.pos[2] < TOP + REST_H - 0.01
                              or (near_platen and mount.pos[2] < paper_z - 0.005)))
        dx, dy = np.abs(mount.pos[:2] - np.array(REST_XY))
        on_rest = bool(dx <= REST_HALF - MOUNT_R and dy <= REST_HALF - MOUNT_R and abs(mount.pos[2] - (TOP + REST_H)) <= 0.003 and R[2, 2] >= math.cos(math.radians(5.0)))
        clear = gk.hands_clear(tips, gk.along(mount, (0, 0, 0), (0, 0, KNOB_TOP), 9), CLEAR_M, radius=CUP_R)
        still = float(np.linalg.norm(mount.lin_vel)) < 0.01
        _, reached, _ = rungs(mm)
        self.hold.step(bool(reached["line"] and on_rest and clear and still))
        self.over.trip(bool(scored_points(mm).max() > HIGH_MM))  # past the cap anywhere on the ring or at the centre: failed for good
        self.score, reached, failed = rungs(mm, on_rest, clear, self.hold.done, bool(self.thrown), bool(self.touched), bool(self.over))
        self.full = self.score >= 1.0
        five = scored_points(mm)
        self.rep = {"t": round(self.n * d.DT, 2), "score": self.score, "force": round(self.force, 2), "omega": round(omega, 2),
                    "tilt": round(math.degrees(math.acos(min(1.0, R[2, 2]))), 3), "mean_mm": round(float(mm[:N_GRID].mean()), 3),
                    "min_mm": round(float(mm[:N_GRID].min()), 3), "ring_min": round(float(five[:-1].min()), 3), "ring_max": round(float(five[:-1].max()), 3), "ring_at_max_deg": round(float(np.argmax(five[:-1])) * 360.0 / RING_N, 1), "centre": round(float(five[-1]), 3), "quad": [round(float(five[k]), 3) for k in range(0, RING_N, RING_N // 4)],
                    "spread": round(float(five.max() - five.min()), 3), "line_shown": k, "on_rest": on_rest, "clear": clear,
                    "thrown": bool(self.thrown), "touched": bool(self.touched), "gap": round(float(gap), 5), "dxy_mm": [round(float(dx*1000),1), round(float(dy*1000),1)], "dz_mm": round(float((mount.pos[2]-(TOP+REST_H))*1000),1), "r22": round(float(R[2,2]),5), "speed": round(float(np.linalg.norm(mount.lin_vel)), 3), "tips_over_face": round(float(tips[12:, 2].min() - mount.pos[2]), 4), "tips_over_flange": round(float(tips[12:, 2].min() - mount.pos[2] - FLANGE_Z1), 4),
                    "tips_over_paper": round(float(tips[:, 2].min() - paper_z), 4)}
        self.rep["hand_over_face"] = round(float(np.mean(tips[12:], axis=0)[2] - mount.pos[2]), 4)
        self.rep["ang_vel"] = [round(float(w), 2) for w in mount.ang_vel]
        if os.environ.get("GRIND_TRACE") and self.n % int(os.environ.get("GRIND_TRACE")) == 0:
            print(f"[grind] {self.rep}", flush=True)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    """Ground to the line in one plane, the mount back on its rest, the hand clear, held still 1 s; never thrown, never a finger on the paper."""
    return TRIAL.full


def score(objects) -> float:
    return TRIAL.score


def stock_left_mm() -> float:
    """What the scribed line shows: its height above the face now, in 0.05 mm steps. A camera sees this; the demo reads nothing else of the grind."""
    return TRIAL.shown / 20.0


# -- the scripted demo ---------------------------------------------------------------------------------------------------------------

TIP_BELOW_TCP = 0.0124  # the fingers' lowest point below the TCP (the harness's collision meshes)
GRIP_Z = FLANGE_Z1 + TIP_BELOW_TCP + 0.0015  # the TCP above the face: fingertips just over the flange
GRIP_YAW = 0.0  # the jaws close along y, across the knob's flats and along the paper's drag on the sweep line (kit README 24)
FORCE_GOAL = 11.0  # N, under the 15 N cap
GRIP_N = 80.0  # the jaws' close command, past the knob's flats: each finger then presses at the 40 N cap set in Trial.reset
SWEEP_IN, SWEEP_OUT = 0.035, 0.080  # the mount's centre from the platen axis, along +x
SWEEP_PERIOD = 4.0  # s, centre to edge and back
TRIM_GAIN, TRIM_MAX = 0.03, 0.05  # the sweep's trim, from where the mount is seen against where it should be
TURN_MAX = np.radians([12.0, 4.0, 2.0])
TURN_GAIN = 0.25  # the hand's turn, per step, against the tilt seen on the mount
RIGHT_AWAY = (0.14, -0.28, 0.20)
CARRY_Z = PAPER_Z + 0.03 + GRIP_Z  # the TCP while carrying: the mount's face 3 cm above the paper, 1 cm over the rim
SPOT_OVER_MM = 0.002  # the spot mode stops when the highest ring stop is this far over the cap
LEVEL_GAP = 0.008  # the face this far over the paper while the hand levels the mount, before it touches down
SET_GAP = 0.012  # the face about this far over the rest before the slow, read set-down
SET_SPEED = 0.008  # m/s: the set-down's descent
SEATED_MM = 0.5  # the mount stands on the rest: its face within this of the rest's top, or pushed this far up the grip


def _rot(axis, ang):
    from yam_arena.urdf import axis_angle_matrix

    n = float(np.linalg.norm(axis))
    return np.eye(3) if n < 1e-12 or abs(ang) < 1e-12 else axis_angle_matrix(np.asarray(axis, float) / n, ang)


def _level(ctx, r, p0, p1, seconds: float, rv):
    """Go from p0 to p1 in `seconds`, turning the hand against the tilt seen on the mount (the grind's own law), so it touches down
    square. Returns the hand's turn as a rotation vector."""
    grip = d.down(GRIP_YAW)
    n = max(1, int(seconds / d.DT))
    for k in range(1, n + 1):
        axis = ctx.state("mount").rotation()[:, 2]
        rv = np.clip(rv + TURN_GAIN * np.cross(axis, (0.0, 0.0, 1.0)), -TURN_MAX, TURN_MAX)
        r._solve(np.asarray(p0) + (np.asarray(p1) - np.asarray(p0)) * (k / n), _rot(rv, float(np.linalg.norm(rv))) @ grip.rot)
        yield
    return rv


def _set_down(ctx, r, timeout: float = 4.0):
    """Lower the held mount onto its rest by feedback: down at SET_SPEED until it reads seated (its face within SEATED_MM of the rest's top, or
    pushed SEATED_MM up the grip, the hand's reading of contact), then stop the hand where it really is, so it lets go of a mount standing on
    the rest, not one pressed into it, and hold it there at least 0.5 s (at most 1 s) until it reads still and flat."""
    from yam_arena import ik

    rot = d.down(GRIP_YAW).rot
    rest_z = TOP + REST_H
    c = ik.tcp_pose(r.q, r.base)[0]
    hang = float(r.tcp()[2] - ctx.pos("mount")[2])  # how far the face hangs below the hand, free in the air
    seated, n = 0, int(timeout / d.DT)
    for k in range(n):
        mz = float(ctx.pos("mount")[2])
        if mz - rest_z <= SEATED_MM / 1000 or float(r.tcp()[2]) - mz < hang - SEATED_MM / 1000:
            break
        c = c - np.array([0.0, 0.0, SET_SPEED * d.DT])
        r._solve(c, rot)
        yield
    c[2] = max(float(c[2]), float(r.tcp()[2]) - 0.0005)  # hold where the hand is, no push into the rest
    r._solve(c, rot)
    last = np.asarray(ctx.pos("mount"), float)
    for k in range(int(1.0 / d.DT)):  # still and flat before letting go: it moves under 0.1 mm a step, 5 steps running
        yield
        mount = ctx.state("mount")
        now = np.asarray(mount.pos, float)
        seated = seated + 1 if float(np.linalg.norm(now - last)) < 0.0001 and mount.rotation()[2, 2] >= math.cos(math.radians(1.0)) else 0
        last = now
        if seated >= 5 and k >= 9:  # held on the rest at least 0.5 s
            break
    print(f"[demo] set down: face {1000 * (float(ctx.pos('mount')[2]) - rest_z):.1f} mm over the rest, pushed "
          f"{1000 * (hang - float(r.tcp()[2] - ctx.pos('mount')[2])):.1f} mm up the grip, still {seated >= 5}", flush=True)


def _grind(ctx, r, mode: str, rv0=None):
    """Hold the mount on the paper: press to FORCE_GOAL read from the platen's spring, keep its face flat by turning the hand against the
    tilt seen on the mount, and sweep it between SWEEP_IN and SWEEP_OUT along +x, until the line reaches the face (or the mode's own stop)."""
    grip = d.down(GRIP_YAW)
    cx, cy = PLATEN_XY
    from yam_arena import ik

    c0 = ik.tcp_pose(r.q, r.base)[0]  # the command now, with any trim the servo found
    off = c0[:2] - np.array([cx + SWEEP_IN, cy])
    z = c0[2]
    rv = np.zeros(3) if rv0 is None else np.asarray(rv0, float)
    corr = _rot(rv, float(np.linalg.norm(rv)))
    trim = np.zeros(2)  # the soft arm sags back and the drag pushes the mount along +y: the hand aims off by what the mount shows
    goal = {"light": 5.0}.get(mode, FORCE_GOAL)
    tilt_goal = {"tilted": 3.0, "oneside": 3.0}.get(mode, 0.0)
    t, worst = 0.0, 0.0
    limit = {"oneside": 60.0}.get(mode, 270.0)
    x = cx + SWEEP_IN
    while t < limit:
        g = ctx.state("grinder")
        force = max(0.0, -CELL_K * g.joint("cell"))
        z += float(np.clip(-(goal - force) * 0.00008, -0.0008, 0.0008))
        mount = ctx.state("mount")
        axis = mount.rotation()[:, 2]
        want = _rot((1.0, 0.0, 0.0), math.radians(tilt_goal))[:, 2]  # tilted and oneside hold the mount 3 degrees over about x, across the jaws, where the pads hold it rigidly (about the jaws' own axis the knob pivots in the pinch and the face settles flat)
        err = np.cross(axis, want)
        rv = rv + TURN_GAIN * err  # the hand's turn, as a rotation vector, follows the tilt seen on the mount
        rv = np.clip(rv, -TURN_MAX, TURN_MAX)  # at most 12 degrees about x (the drag tips the mount about x) and 4 about y (the arm's reach here)
        corr = _rot(rv, float(np.linalg.norm(rv)))
        s = 0.5 - 0.5 * math.cos(2 * math.pi * t / SWEEP_PERIOD)
        x = cx + SWEEP_IN + (SWEEP_OUT - SWEEP_IN) * s
        trim = np.clip(trim + TRIM_GAIN * (np.array([x, cy]) - mount.pos[:2]), -TRIM_MAX, TRIM_MAX)
        mm, deg = r._solve(np.array([x + off[0] + trim[0], cy + off[1] + trim[1], z]), corr @ grip.rot)
        worst = max(worst, mm)
        yield
        t += d.DT
        if mode == "spot":  # keeps grinding until the highest ring stop is 0.002 mm over the cap, a stretch of the ring between the old four points
            if TRIAL.rep.get("ring_max", 0.0) >= HIGH_MM + SPOT_OVER_MM:
                break
        elif mode not in ("over", "light", "oneside") and stock_left_mm() <= 0.0:
            break
    # ease off: still on the paper, the push let go and the hand's turn unwound together over 1.5 s, so the mount comes off square
    z_free = c0[2]
    corr_axis = np.array([corr[2, 1] - corr[1, 2], corr[0, 2] - corr[2, 0], corr[1, 0] - corr[0, 1]])
    corr_ang = math.acos(max(-1.0, min(1.0, (np.trace(corr) - 1.0) / 2.0)))
    z0, n_ease = z, int(1.5 / d.DT)
    for k in range(1, n_ease + 1):
        u = k / n_ease
        r._solve(np.array([x + off[0] + trim[0], cy + off[1] + trim[1], z0 + (z_free - z0) * u]), _rot(corr_axis, corr_ang * (1.0 - u)) @ grip.rot)
        yield
    r.grip = grip
    ang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(corr) - 1.0) / 2.0))))
    print(f"[demo] grind ended at t {t:.1f} s, push {force:.1f} N, line {stock_left_mm():.1f} mm, hand turned {ang:.1f} deg, trim {np.round(trim * 1000, 1)} mm, worst IK {worst:.1f} mm", flush=True)


def demo_script(ctx):
    """Modes: full; none, the arms wait; tilted, held 3 degrees over to the line (a facet); oneside, held 3 degrees over for 60 s (one side
    ground); light, a 5 N push for 270 s; over, no stop at the line; leave, ground to the line and let go on the turning paper (the platen
    flings the mount off); keep, ground to the line, lifted off and held in the air, never set back; hover, held just above the paper; spot, held flat and ground on past the
    line until one ring stop is 0.002 mm over the cap (run with GRIND_INDEX_DEG=15, which puts that stretch between the old four points)."""
    modes = ("full", "none", "tilted", "oneside", "light", "over", "leave", "keep", "hover", "spot")
    if ctx.mode not in modes:
        raise ValueError(f"unknown mode {ctx.mode!r}")
    r = ctx.right
    if ctx.mode == "none":
        yield from r.wait(3.0)
        return
    grip = d.down(GRIP_YAW)
    paper = TOP + PAPER_Z
    yield from r.pick("mount", GRIP_Z, width=KNOB[1], grip=grip, above=0.04, lift=CARRY_Z - (REST_H + GRIP_Z), force=GRIP_N, t=3.0)  # by the knob's flats; straight up, above the bowl's rim
    start = np.array([PLATEN_XY[0] + SWEEP_IN, PLATEN_XY[1], paper + GRIP_Z])
    yield from r.move(np.array([start[0], start[1], TOP + CARRY_Z]), label="over the platen")
    yield from r.servo(np.array([start[0], start[1], TOP + CARRY_Z]), timeout=1.0, label="over the platen")
    if ctx.mode == "hover":
        yield from r.move(start + [0.0, 0.0, 0.004], t=1.0, label="hover")
        yield from r.wait(30.0)
        return
    hover = start + [0.0, 0.0, LEVEL_GAP]
    yield from r.move(hover, t=1.0, label="down near the paper")
    rv = yield from _level(ctx, r, hover, hover, 1.5, np.zeros(3))  # level by what the mount shows, just clear of the paper
    rv = yield from _level(ctx, r, hover, start, 1.0, rv)  # and touch down square
    yield from _grind(ctx, r, ctx.mode, rv)
    m_at = ctx.pos("mount")
    up = np.array([m_at[0], m_at[1], paper + GRIP_Z + 0.04])  # straight up
    yield from r.move(up, grip=grip, t=0.8, label="lift off the paper")
    if ctx.mode == "keep":
        yield from r.wait(3.0)
        return
    if ctx.mode == "leave":
        yield from r.move(up - [0.0, 0.0, 0.045], t=0.8, label="back down")
        yield from r.open()
        yield from r.move(r.at() + [0.0, 0.0, 0.08], label="clear")
        yield from r.wait(2.0)
        return
    yield from r.move(np.array([up[0], up[1], TOP + CARRY_Z]), t=0.6, label="up over the rim")
    slip = ctx.pos("mount")[:2] - r.at()[:2]  # where the mount sits in the grip now: the hand aims so the mount, not the hand, lands on the rest
    aim = np.array(REST_XY) - slip
    over_rest = np.array([aim[0], aim[1], TOP + CARRY_Z])
    yield from r.move(over_rest, label="carry to the rest")
    yield from r.servo(over_rest, timeout=1.0, label="carry to the rest")
    yield from r.move(np.array([aim[0], aim[1], TOP + REST_H + GRIP_Z + SET_GAP]), t=1.2, label="down over the rest")
    yield from _set_down(ctx, r)  # lowered by feedback until it stands on the rest, unloaded
    yield from r.open(t=0.8)
    yield from r.move(r.at() + [0.0, 0.0, 0.06], t=1.5, label="straight up")
    yield from r.move((RIGHT_AWAY[0], RIGHT_AWAY[1], TOP + RIGHT_AWAY[2]), label="clear")
    yield from r.wait(1.5)
