"""On the station table: an open pine bookshelf. At its left end three books stand packed against the end panel. Beside them a run of three
hardcovers leans about 17 degrees into a 4 cm gap, the end one resting on the top edge of a steel L bookend, whose base runs under the gap
and the run. The three have taken a lean from standing slumped: each case is racked, its boards offset at the tail, so a book stood straight
rests on one board's edge and tips back into the gap when let go. The green book stands on the table in front of the left arm.
The task: push the run upright and hold it there, slide the green book into the gap spine out until its spine is flush with theirs, then let
go and move both hands clear. The run then rests upright against the new book.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1800 --seed 1 --instruction "<INSTRUCTION below>"
The demo:  python demokit.py task.py OUT
"""

from __future__ import annotations

import collections
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import gatekit as gk  # noqa: E402
import mechkit as m  # noqa: E402  (hooks yam_arena so that the parts below spawn from USD)
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = ("Put the green book back on the shelf, in the gap beside the books that lean into it: push those books upright and hold them, "
               "slide the green book in spine out until its spine is flush with theirs, then let go and move both hands clear.")
SUCCESS_HOLD_S = 0.0  # the scorer holds the end state itself (HOLD_STEPS)
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
DT = 0.05

# -- the shelf (lengths in m; x forward from the arm bases, y to the robot's left, z above the table top) --------------------------------
X_F = 0.345  # the board's front edge: clear of the grippers at the arms' ready pose (x 32 cm)
X_S = 0.355  # the books' spine line, 1 cm behind it
DEPTH = 0.26  # the board's depth
Z_B = 0.020  # the board's top
PANEL = 0.018
PANEL_H = 0.30  # the end panels and the back stand this high above the board
Y_R = -0.220  # the board's open right end (the left end panel's inner face is Y_L, below)

# -- the bookend: 1.2 mm steel, an L: the plate stands across the row, the base runs under the gap and the run -----------------------------
STEEL = 0.0012
Y_PLATE = -0.020  # the plate's inner face: the gap's right wall
PLATE_H = 0.130
END_X = 0.140  # the plate's and the base's width along x, from the spine line
BASE_TO = 0.110  # the base's far edge, under the run's first book
END_MASS = 0.34
Z_S = Z_B + STEEL  # where the books in the run and the green book stand

# -- the run: three hardcovers, left to right; the last one (the end book) leans on the plate ---------------------------------------------
RUN_T = (0.026, 0.032, 0.030)  # thickness
RUN_MASS = (0.55, 0.74, 0.70)
RUN_LOOK = ("cloth_maroon", "cloth_navy", "cloth_ochre")
RUN_D, RUN_H = 0.152, 0.229  # 6 x 9 in trade hardcovers
RACK_DEG = 10.0  # each case is racked by this much: stood straight it rests on its left board's tail and tips right
LEAN0_DEG = 17.0  # the run's lean at the start, on the plate's top edge
GAP = 0.0005  # left between the books at the start, so they settle rather than spring apart


# -- the green book ------------------------------------------------------------------------------------------------------------------
NEW_D, NEW_T, NEW_H = 0.130, 0.033, 0.195
NEW_MASS = 0.55
NEW_AT = (0.255, 0.200)  # where it stands on the table in front of the left arm, spine toward the robot

BOOK_FRICTION = (0.4, 0.4)  # cloth-bound boards on cloth, pine and steel
STEEL_FRICTION = (0.5, 0.4)
PINE_FRICTION = (0.5, 0.4)

# -- the scoring lines ----------------------------------------------------------------------------------------------------------------
FLUSH = 0.010  # the spine within 10 mm of the neighbours' spines, proud or sunk
UPRIGHT_DEG = 8.0  # the green book and every book in the run, at 1.0
SEATED_DEG = 12.0  # the green book at 0.75 (a book in a 4 cm gap cannot lean far)
RUN_SEATED_DEG = 10.0  # every book of the run at 0.75: the run at its start leans 17, so an unpushed run cannot reach it
FALLEN_DEG = 35.0  # a book of the run tilted further than this has fallen
ENTERED = 0.03  # the fore-edge at least this far past the neighbours' spines
END_MOVED = 0.006  # the bookend moved (in any direction) this far from its mark: the set-up was changed
END_TURN_DEG = 5.0  # or turned this far from its start pose, about any axis
SLOT_TOL = 0.015  # every run book's spine within this of the spine line, and (plus its lean) its place across the row within 1 cm of its slot
CONTACT_N = 0.1  # the hold: a finger of one hand presses on a run book with at least this force (N), read from the simulator's contacts ...
ENTER_AT = 0.005  # ... at every step from the green book's fore-edge passing this far beyond the run's spines (entering the gap) ...
# ... until its spine is within FLUSH of theirs (seated), with every run book within RUN_SEATED_DEG of upright; latched: one lapse closes credit above 0.5
SPINE_YAW = math.sin(math.radians(6.0))
CLEAR = 0.03  # every fingertip at least this far from every book and the bookend
STILL = 0.01  # m/s: every book and the bookend slower than this ...
STILL_STEPS = 20  # ... for 1 s
HOLD_STEPS = 40  # and the end state held 2 s, hands clear, before it scores

LOOKS = {
    "pine": m.Look(color=m.rgb(0.80, 0.64, 0.42), roughness=0.6),
    "pine_end": m.Look(color=m.rgb(0.74, 0.57, 0.36), roughness=0.65),
    "back": m.Look(color=m.rgb(0.70, 0.56, 0.38), roughness=0.7),
    "steel": m.Look(color=m.rgb(0.62, 0.63, 0.66), metallic=0.8, roughness=0.35),  # brushed steel, so the bookend shows against the books
    "pages": m.Look(color=m.rgb(0.93, 0.90, 0.80), roughness=0.9),
    "gilt": m.Look(color=m.rgb(0.80, 0.66, 0.28), metallic=0.6, roughness=0.35),
}
for _n, _c in {"maroon": (0.42, 0.08, 0.10), "navy": (0.10, 0.14, 0.32), "ochre": (0.70, 0.52, 0.16), "green": (0.10, 0.42, 0.20),
               "green2": (0.16, 0.26, 0.20), "black": (0.07, 0.07, 0.08), "tan": (0.66, 0.56, 0.38), "blue": (0.16, 0.34, 0.58),
               "brown": (0.36, 0.22, 0.12)}.items():
    LOOKS[f"cloth_{_n}"] = m.Look(color=m.rgb(*_c), roughness=0.75)


def rot_x(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rack(t: float) -> float:
    """How far a case's right board stands above its left at the tail: the racking's offset."""
    return t * math.tan(math.radians(RACK_DEG))


def sheared(mesh: m.Mesh, t: float) -> m.Mesh:
    """A mesh built square in the book's frame, racked: the left board (+y) drops by half the offset, the right rises by half."""
    v = mesh.v.copy()
    v[:, 2] -= rack(t) / t * v[:, 1]
    return m.Mesh(v, mesh.f)


def run_start() -> list:
    """Each run book's start: (centre, rotation), all leaning LEAN0_DEG face to face, each resting on its right tail corner, the end book's
    right face on the plate's top edge."""
    th = math.radians(LEAN0_DEG)
    R = rot_x(LEAN0_DEG)
    y_c = Y_PLATE + (Z_B + PLATE_H - Z_S) * math.tan(th) + GAP  # the end book's right tail corner
    corners = [0.0, 0.0, 0.0]
    corners[2] = y_c
    corners[1] = corners[2] + RUN_T[2] / math.cos(th) + GAP
    corners[0] = corners[1] + RUN_T[1] / math.cos(th) + GAP
    out = []
    for i, t in enumerate(RUN_T):
        c_r = np.array([0.0, -t / 2, -RUN_H / 2 + rack(t) / 2])  # the right tail corner, in the book's frame
        w = R @ c_r
        out.append((np.array([X_S + RUN_D / 2, corners[i] - w[1], TOP + Z_S + 0.0005 - w[2]]), R))
    return out


def upright_faces() -> list:
    """Each run book's (right, left) face across the row once pushed upright on its left tail corner without its feet sliding."""
    th = math.radians(LEAN0_DEG)
    a = math.radians(RACK_DEG)
    out = []
    y_c = Y_PLATE + (Z_B + PLATE_H - Z_S) * math.tan(th) + GAP
    cs = [0.0, 0.0, y_c]
    cs[1] = cs[2] + RUN_T[2] / math.cos(th) + GAP
    cs[0] = cs[1] + RUN_T[1] / math.cos(th) + GAP
    for i, t in enumerate(RUN_T):
        left = cs[i] + t / math.cos(a)
        out.append((left - t, left))
    return out


SLOT_Y = 0.5 * (Y_PLATE + upright_faces()[2][0])  # the gap's middle, with the run upright

# -- the three books packed at the left end, against the end panel (fixed in place: they are not part of the act) ------------------------
BLOCK_T = (0.034, 0.028, 0.036)
BLOCK_H = (0.240, 0.215, 0.250)
BLOCK_D = (0.160, 0.150, 0.170)
BLOCK_LOOK = ("cloth_green2", "cloth_tan", "cloth_brown")
Y_BLOCK = upright_faces()[0][1] + 0.0008  # the packed books' right face: the run's first book, upright, stands 0.8 mm from it
Y_L = Y_BLOCK + sum(BLOCK_T)  # the left end panel's inner face


# -- the parts ------------------------------------------------------------------------------------------------------------------------


def _shelf():
    x1 = X_F + DEPTH
    w = Y_L + PANEL - (Y_R - PANEL)
    yc = 0.5 * (Y_L + PANEL + Y_R - PANEL)
    boxes = [
        ("board", (DEPTH, w, Z_B), (X_F + DEPTH / 2, yc, Z_B / 2), "pine"),
        ("back", (0.008, w, Z_B + PANEL_H), (x1 + 0.004, yc, (Z_B + PANEL_H) / 2), "back"),
        ("end_l", (DEPTH, PANEL, Z_B + PANEL_H), (X_F + DEPTH / 2, Y_L + PANEL / 2, (Z_B + PANEL_H) / 2), "pine_end"),
    ]
    parts = [(n, m.box(s, c), k) for n, s, c, k in boxes]
    cols = [m.col_box(f"{n}_c", s, c) for n, s, c, _ in boxes]
    y = Y_L
    for i, (t, h, d, look) in enumerate(zip(BLOCK_T, BLOCK_H, BLOCK_D, BLOCK_LOOK)):
        y -= t
        c = (X_S + d / 2, y + t / 2, Z_B + h / 2)
        parts += [(f"block{i}", m.box((d, t, h), c), look), (f"block{i}_pages", m.box((d - 0.004, t - 0.006, h + 0.0004), (c[0] + 0.003, c[1], c[2])), "pages"),
                  (f"block{i}_label", m.box((0.0008, t * 0.6, 0.04), (X_S - 0.0003, c[1], Z_B + h - 0.06)), "gilt")]
        cols.append(m.col_box(f"block{i}_c", (d, t, h), c))
    return m.fixed("shelf", (0.0, 0.0, TOP), parts, cols, LOOKS, friction=PINE_FRICTION)


def _bookend():
    """Origin: the plate's inner face at the board, midway along x."""
    base_w = BASE_TO - Y_PLATE + STEEL
    parts = [("plate", m.box((END_X, STEEL, PLATE_H), (0.0, -STEEL / 2, PLATE_H / 2)), "steel"),
             ("base", m.box((END_X, base_w, STEEL), (0.0, -STEEL + base_w / 2, STEEL / 2)), "steel")]
    cols = [m.col_box("plate_c", (END_X, STEEL, PLATE_H - STEEL), (0.0, -STEEL / 2, STEEL + (PLATE_H - STEEL) / 2)),
            m.col_box("base_c", (END_X, base_w, STEEL), (0.0, -STEEL + base_w / 2, STEEL / 2))]
    return m.free("bookend", (X_S + END_X / 2, Y_PLATE, TOP + Z_B), parts, cols, LOOKS, mass=END_MASS, friction=STEEL_FRICTION)


def _book_parts(d: float, t: float, h: float, cloth: str, t_rack: float | None = None):
    """A hardcover's visual parts in its own frame (spine toward -x), racked if `t_rack` is given; and its collider's mesh."""
    b = 0.003
    parts = [
        ("board_l", m.box((d, b, h), (0.0, t / 2 - b / 2, 0.0)), cloth),
        ("board_r", m.box((d, b, h), (0.0, -t / 2 + b / 2, 0.0)), cloth),
        ("spine", m.box((b, t, h), (-d / 2 + b / 2, 0.0, 0.0)), cloth),
        ("pages", m.box((d - b - 0.003, t - 2 * b, h - 0.006), (b / 2, 0.0, 0.0)), "pages"),
        ("band_top", m.box((0.0008, t * 0.9, 0.004), (-d / 2 - 0.0002, 0.0, h / 2 - 0.022)), "gilt"),
        ("band_low", m.box((0.0008, t * 0.9, 0.004), (-d / 2 - 0.0002, 0.0, -h / 2 + 0.022)), "gilt"),
        ("label", m.box((0.0008, t * 0.6, 0.045), (-d / 2 - 0.0002, 0.0, h / 2 - 0.065)), "gilt"),
    ]
    hull = m.box((d, t, h))
    if t_rack is not None:
        parts = [(n, sheared(mesh, t_rack), k) for n, mesh, k in parts]
        hull = sheared(hull, t_rack)
    return parts, hull


def _run_book(i: int):
    t = RUN_T[i]
    parts, hull = _book_parts(RUN_D, t, RUN_H, RUN_LOOK[i], t_rack=t)
    pos, R = run_start()[i]
    q = gk.quat_wxyz(R)
    return m.free(f"run{i}", tuple(pos), parts, [m.col_hull("book_c", hull)], LOOKS, mass=RUN_MASS[i], friction=BOOK_FRICTION,
                  torsional_patch=0.01, start_quat=(float(q[1]), float(q[2]), float(q[3]), float(q[0])))


def _new_book():
    parts, _ = _book_parts(NEW_D, NEW_T, NEW_H, "cloth_green")
    return m.free("newbook", (NEW_AT[0], NEW_AT[1], TOP + NEW_H / 2), parts, [m.col_box("book_c", (NEW_D, NEW_T, NEW_H))], LOOKS, mass=NEW_MASS,
                  friction=BOOK_FRICTION, torsional_patch=0.01)


OBJECTS = [_shelf(), _bookend()] + [_run_book(i) for i in range(3)] + [_new_book()]


# -- the scorer ------------------------------------------------------------------------------------------------------------------------


def lean_deg(R) -> float:
    """A book's lean across the row from its boards' height axis, toward the gap (-y) positive."""
    R = np.asarray(R, float)
    return float(math.degrees(math.atan2(-R[1, 2], R[2, 2])))


def tilt_deg(R) -> float:
    """How far a book's boards stand from vertical, any direction."""
    return float(math.degrees(math.acos(float(np.clip(np.asarray(R, float)[2, 2], -1.0, 1.0)))))


def rotvec(R) -> np.ndarray:
    """A rotation matrix as axis times angle (radians)."""
    R = np.asarray(R, float)
    a = math.acos(float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)))
    if a < 1e-9:
        return np.zeros(3)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    n = float(np.linalg.norm(w))
    return w / n * a if n > 1e-12 else np.zeros(3)


def corners(pos, R, size, t_rack: float | None = None) -> np.ndarray:
    d, t, h = size
    c = np.array([[sx * d / 2, sy * t / 2, sz * h / 2] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    if t_rack is not None:
        c[:, 2] -= rack(t_rack) / t_rack * c[:, 1]
    return np.asarray(pos, float) + c @ np.asarray(R, float).T


def box_distance(points, pos, R, half) -> np.ndarray:
    """Each point's distance to a box (centre, rotation, half sizes); 0 inside."""
    local = (np.asarray(points, float) - np.asarray(pos, float)) @ np.asarray(R, float)
    return np.linalg.norm(np.maximum(np.abs(local) - np.asarray(half, float), 0.0), axis=1)


class Seen:
    """What the scorer reads in one step, all of it on the shelf at rest: a photo from the front and above, a ruler and a protractor.
    new: (centre, rotation) of the green book; run: the same for the three; end: the bookend's centre and rotation; end0: where the bookend
    started; tips: the 24 fingertip points; speed: the fastest of the books and the bookend (m/s)."""

    def __init__(self, new, run, end, end0, tips, speed: float, body=None, end_r0=None):
        self.new = (np.asarray(new[0], float), np.asarray(new[1], float))
        self.run = [(np.asarray(p, float), np.asarray(r, float)) for p, r in run]
        self.end = (np.asarray(end[0], float), np.asarray(end[1], float))
        self.end0 = np.asarray(end0, float)
        self.tips = np.asarray(tips, float)
        self.speed = float(speed)
        self.body = self.tips if body is None else np.asarray(body, float)  # palm, wrist and forearm points of both hands
        self.end_r0 = np.eye(3) if end_r0 is None else np.asarray(end_r0, float)


def spine_x(pos, R, d: float) -> float:
    """The middle of a book's spine, along x."""
    return float((np.asarray(pos, float) + np.asarray(R, float) @ np.array([-d / 2, 0.0, 0.0]))[0])


def readings(s: Seen) -> dict:
    """The end-state readings the rungs use."""
    pn, Rn = s.new
    cn = corners(pn, Rn, (NEW_D, NEW_T, NEW_H))
    low = cn[np.argsort(cn[:, 2])[:4]]  # the four lowest corners: the tail
    on_board = (np.abs(low[:, 2] - (TOP + Z_B)) < 0.012) & (low[:, 0] > X_F) & (low[:, 0] < X_F + DEPTH) & (low[:, 1] > Y_R) & (low[:, 1] < Y_L)
    on_shelf = bool(on_board.sum() >= 2)  # it stands on the shelf: at least two corners of its tail on the board (or the bookend's base)
    tail_y = float(low[:, 1].mean())
    in_gap = bool(Y_PLATE - 0.006 < tail_y < upright_faces()[2][0] + 0.006)
    run_spines = [spine_x(p, r, RUN_D) for p, r in s.run]
    run_spine = float(np.mean(run_spines))
    proud = run_spine - spine_x(pn, Rn, NEW_D)  # how far the green book's spine stands out of the run's (negative: sunk deeper)
    proud_far = max(abs(v - spine_x(pn, Rn, NEW_D)) for v in run_spines)  # the worst book of the run
    faces = upright_faces()
    in_place = all(abs(v - X_S) <= SLOT_TOL for v in run_spines) and all(
        abs(p[1] - 0.5 * sum(faces[i])) <= 0.01 + 0.5 * RUN_H * math.sin(math.radians(tilt_deg(r))) for i, (p, r) in enumerate(s.run))  # the run is where it stood
    spine_out = bool(abs(Rn[1, 0]) < SPINE_YAW)  # the spine is square to the shelf, within 6 degrees
    run_tilt = [tilt_deg(r) for _, r in s.run]
    fallen = []
    for (p, r), t in zip(s.run, RUN_T):
        c = corners(p, r, (RUN_D, t, RUN_H), t_rack=t)
        fallen.append(tilt_deg(r) > FALLEN_DEG or float(c[:, 2].min()) < TOP + Z_B - 0.012)
    end_moved = float(np.linalg.norm(s.end[0] - s.end0))
    end_tilt = tilt_deg(s.end[1])
    end_turn = float(math.degrees(math.acos(float(np.clip((np.trace(s.end[1] @ s.end_r0.T) - 1.0) / 2.0, -1.0, 1.0)))))
    half_new = (NEW_D / 2, NEW_T / 2, NEW_H / 2)
    pts = np.vstack([s.tips, s.body])
    gaps = [box_distance(pts, pn, Rn, half_new).min()]
    for (p, r), t in zip(s.run, RUN_T):
        gaps.append(box_distance(pts, p, r, (RUN_D / 2, t / 2, RUN_H / 2 + rack(t) / 2)).min())
    pe, Re = s.end
    gaps.append(box_distance(pts, pe + Re @ np.array([0.0, -STEEL / 2, PLATE_H / 2]), Re, (END_X / 2, STEEL / 2, PLATE_H / 2)).min())
    gaps.append(box_distance(pts, pe + Re @ np.array([0.0, (BASE_TO - Y_PLATE) / 2, STEEL / 2]), Re, (END_X / 2, (BASE_TO - Y_PLATE) / 2, STEEL / 2)).min())
    return {"on_shelf": on_shelf, "in_gap": in_gap, "proud": float(proud), "depth_in": float(NEW_D - proud), "spine_out": spine_out, "proud_far": float(proud_far), "in_place": in_place, "end_turn": end_turn,
            "new_tilt": tilt_deg(Rn), "run_tilt": run_tilt, "run_lean": [lean_deg(r) for _, r in s.run], "fallen": any(fallen),
            "end_moved": end_moved, "end_tilt": end_tilt, "clear": float(min(gaps))}


LADDER = gk.Ladder(("entered", 0.25), ("half_in", 0.5), ("seated", 0.75), ("done", 1.0))


def failed(r: dict) -> bool:
    """Score 0: a book of the run fallen over (past 35 degrees) or off the shelf, or the bookend moved 6 mm or more from its mark, turned 5 degrees or tilted 4 degrees."""
    return bool(r["fallen"] or r["end_moved"] >= END_MOVED or r["end_tilt"] > 4.0 or r["end_turn"] > END_TURN_DEG)


def rungs(r: dict) -> dict:
    """The rungs, each read from the end state at rest: a ruler across the spines, a protractor on the boards, a photo from above."""
    entered = r["on_shelf"] and r["in_gap"] and r["in_place"] and r["depth_in"] >= ENTERED and r["new_tilt"] <= FALLEN_DEG
    half_in = entered and r["depth_in"] >= NEW_D / 2
    seated = (half_in and abs(r["proud"]) <= FLUSH and r["proud_far"] <= FLUSH + 0.005 and r["spine_out"] and r["new_tilt"] <= SEATED_DEG
              and max(r["run_tilt"]) <= RUN_SEATED_DEG)
    done = seated and r["new_tilt"] <= UPRIGHT_DEG and max(r["run_tilt"]) <= UPRIGHT_DEG
    return {"entered": entered, "half_in": half_in, "seated": seated, "done": done}


def judge(s: Seen, still: bool, held: int) -> tuple[float, dict]:
    """(score, readings) for one step. A rung scores only once the end state has been at rest with both hands clear for HOLD_STEPS steps
    (`held`, this step included); before that, and whenever a hand is within 3 cm or anything moves, the score is 0."""
    r = readings(s)
    reached = rungs(r)
    ok = still and r["clear"] >= CLEAR
    r["rung"] = LADDER.score(reached, failed(r))
    r["at_rest"] = ok
    return (r["rung"] if ok and held >= HOLD_STEPS else 0.0), r


class Trial:
    def reset(self, env=None) -> None:
        self.steps = 0
        self.end0 = None
        self.speeds = collections.deque(maxlen=STILL_STEPS)
        self.held, self.last_rung = 0, None
        self.score, self.full = 0.0, False
        self.ever_failed, self.travel_steps, self.lapsed, self.pressed = False, 0, False, 0.0
        self.end_r0 = None
        self.tilts = collections.deque(maxlen=STILL_STEPS)
        self.r: dict = {}

    def step(self, env) -> None:
        self.steps += 1
        new = m.state(env, "newbook")
        run = [m.state(env, f"run{i}") for i in range(3)]
        end = m.state(env, "bookend")
        if self.end0 is None:
            self.end0 = end.pos.copy()
        if self.end_r0 is None:
            self.end_r0 = end.rotation().copy()
        sp = max(float(np.linalg.norm(o.lin_vel)) for o in [new, end] + run)
        spin = max(float(np.linalg.norm(o.ang_vel)) for o in [new, end] + run)
        self.speeds.append(max(sp, spin * 0.1))  # 0.1 rad/s counts as 1 cm/s: a book turning slowly is not at rest
        self.tilts.append([tilt_deg(o.rotation()) for o in run])
        drift = float(np.max(np.ptp(np.array(self.tilts), axis=0)))  # a run book still tipping over the last second
        still = len(self.speeds) == STILL_STEPS and max(self.speeds) < STILL and drift < 0.5
        s = Seen((new.pos, new.rotation()), [(o.pos, o.rotation()) for o in run], (end.pos, end.rotation()), self.end0, m.fingertips(env), sp,
                 body=m.hand_body(env), end_r0=self.end_r0)
        _, r = judge(s, still, 0)
        self.ever_failed |= bool(r["fallen"])  # a run book once knocked over stays failed, even if it is stood up again
        force = m.contact_forces(env, ["run0", "run1", "run2"]).sum(axis=0)  # N the left, then the right fingers press on the run
        self.pressed = float(force.max())
        if ENTER_AT <= r["depth_in"] < NEW_D - FLUSH:  # the green book's travel: entering the gap, not yet seated
            self.travel_steps += 1
            if self.pressed < CONTACT_N or max(r["run_tilt"]) > RUN_SEATED_DEG:
                self.lapsed = True  # a step in the travel with no finger on the run, or the run not held upright: no credit above 0.5, for the rest of the trial
        if r["at_rest"] and r["rung"] == self.last_rung:
            self.held += 1
        else:
            self.held = 1 if r["at_rest"] else 0
        self.last_rung = r["rung"]
        self.score, self.r = judge(s, still, self.held)
        if self.ever_failed or ((self.lapsed or self.travel_steps == 0) and self.score > 0.5):
            self.score = 0.0 if self.ever_failed else 0.5  # no finger on the run through the whole travel, no credit above 0.5
        self.full = self.score >= 1.0
        log = os.environ.get("RESHELVE_LOG")
        if log and self.steps % int(log) == 0:
            print(f"[shelf] step {self.steps}: run lean {[round(v, 1) for v in r['run_lean']]} new tilt {r['new_tilt']:.1f} proud {r['proud'] * 1000:.0f} mm "
                  f"in_gap {r['in_gap']} on_shelf {r['on_shelf']} end moved {r['end_moved'] * 1000:.1f} mm clear {r['clear'] * 1000:.0f} mm "
                  f"speed {sp:.3f} rung {r['rung']} held {self.held} pressed {self.pressed:.2f} N travel {self.travel_steps} lapsed {self.lapsed} score {self.score}", flush=True)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    """The green book in, spine flush within 10 mm, a finger of one hand on the run through its whole travel, it and the run within 8 degrees of upright, nothing fallen, the bookend on its mark,
    held 2 s at rest with both hands 3 cm clear."""
    return TRIAL.score >= 1.0


def score(objects) -> float:
    return TRIAL.score


# -- the demo ---------------------------------------------------------------------------------------------------------------------------

TIP = 0.011  # the closed fingers' real ends stand this far past the TCP along the fingers (kit README 22)
X_HOLD = X_S + 0.075  # the right hand holds the end book here along its depth, clear of the left hand at the spine line
Z_HOLD = Z_B + 0.231  # the middle of the right fingers' ends: their lower edge (1.8 cm down) stays above the green book's head
BOOK_TILT, BOOK_PHI = float(os.environ.get("RS_TILT", "70")), float(os.environ.get("RS_PHI", "0"))  # the left grip on the green book: fingers sloping 20 degrees down from level (level puts joint 5 at its limit)
GRIP_BACK, GRIP_DOWN = float(os.environ.get("RS_BACK", "0.022")), float(os.environ.get("RS_DOWN", "0.020"))  # the left hand's TCP on the green book, in front of its spine and below its head: both pads wholly on the covers
SQUEEZE = 45.0  # N a finger: a pinch is a pivot (kit README 24), and the book's middle is 4.7 cm in front of the pads
LET_GO = X_S - 0.050  # where the green book's spine is when the left hand lets go, its fingers 1 cm short of the gap
PUSH_Z = 0.085  # the left fingers push the spine this high above its tail


def demo_script(ctx):
    """A scripted reference, not a policy. It reads the books' poses from the simulator to steer, never the scorer.
    The right hand comes in level over the bookend, its fingers pointing along the row, pushes the run upright with its fingertips against
    the packed books and holds it there by feedback on the end book's lean. The left hand picks the green book by its top rear corner,
    slides it fore-edge first about 8.5 cm into the gap under the right fingers, lets go, and pushes its spine flush with a fingertip.
    Then the right hand lets go and the run settles against the green book.
    TASK_DEMO: full; still; none; gate1 (push the run upright, hold 4 s, let go: the run falls back); one_hand (one hand at a time: the right
    pushes the run upright and lets go, then the left brings the book); no_hold (the left brings the book with no hold); early_out (the
    right lets go when the book is 4 cm in); hover (the right fingers 2 cm off the run once it is upright, near it and never on it, while the left brings the book); proud (the spine left 3 cm out); hands_on (the left hand stays on the spine)."""
    import demokit as d
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, READY

    l, r = ctx.left, ctx.right
    if ctx.mode in ("still", "none"):
        yield from r.wait(1.0 if ctx.mode == "still" else 3.0)
        return
    flags = {"held": False, "release": False}

    def leans() -> list:
        return [round(lean_deg(ctx.state(f"run{i}").rotation()), 1) for i in range(3)]

    def run_lean() -> float:
        return lean_deg(ctx.state("run2").rotation())

    def book():
        st = ctx.state("newbook")
        return st.pos, st.rotation()

    def on_book(local) -> np.ndarray:
        p, R = book()
        return p + R @ np.asarray(local, float)

    def rest(arm, t: float = 1.5):
        q0, qr = arm.q.copy(), np.array(READY, float)
        n = max(1, round(t * 20))
        for j in range(1, n + 1):
            arm.q = q0 + (qr - q0) * (0.5 - 0.5 * math.cos(math.pi * j / n))
            arm.cmd = np.r_[arm.q, 1.0]
            yield
        arm.finger, arm.grip, arm.trim = FINGER_OPEN, d.down(), np.zeros(3)

    g_hold = d.level(90.0)  # right fingers level, pointing along the row toward the run, closed

    def hold(arm, until):
        """Come in right of the end book's head, push the run upright and hold it there on feedback from the end book's lean."""
        p0, R0 = ctx.state("run2").pos, ctx.state("run2").rotation()
        head = p0 + R0 @ np.array([0.0, -RUN_T[2] / 2, RUN_H / 2 + rack(RUN_T[2]) / 2])  # the end book's top right edge
        start = np.array([X_HOLD, head[1] - 0.025 - TIP, TOP + Z_HOLD])
        yield from arm.move(start + np.array([0.0, -0.03, 0.06]), grip=g_hold, finger=FINGER_CLOSED, t=2.5, label="hold: over")
        yield from arm.move(start, t=1.0, label="hold: in")
        yield from arm.servo(start, timeout=1.0, label="hold: in")
        c = arm.at() + arm.trim
        aim = -0.3  # degrees: the run a little past upright, toward the packed books
        k, y_up, hov = 0, None, None
        while not until():
            e = run_lean() - aim
            c = c + np.array([0.0, float(np.clip(0.0003 * e, -0.002, 0.003)), 0.0])
            if y_up is not None:
                c[1] = float(np.clip(c[1], y_up - 0.006, y_up + 0.008))  # no wind-up against the packed books
            if ctx.mode == "hover" and flags["held"]:  # the lazy route: the fingers 2 cm off the end book's head, near the run but never on it
                hov = c + np.array([0.0, -0.02, 0.0]) if hov is None else hov
                c = hov
            arm._solve(c, g_hold.rot)
            k += 1
            if y_up is None and e < 0.5:
                y_up = float(c[1])
            if not flags["held"] and abs(e) < 1.0 and k > 10:
                flags["held"] = True
                print(f"[demo] run held upright at step {k}: leans {leans()}", flush=True)
            yield

    def let_go(arm):
        yield from arm.move(arm.at() - np.array([0.0, 0.025, 0.0]), t=0.6, label="hold: off")
        yield from arm.move(arm.at() + np.array([0.0, -0.03, 0.06]), t=0.8, label="hold: out")
        yield from rest(arm)

    if ctx.mode == "gate1":
        t0 = [0, 0]

        def until():
            t0[0] += 1
            t0[1] += 1 if flags["held"] else 0
            if t0[1] in (20, 40, 60, 80):
                print(f"[gate1] held {t0[1] * DT:.0f} s: leans {leans()}", flush=True)
            return t0[1] > 80 or t0[0] > 600

        print(f"[gate1] start: leans {leans()}", flush=True)
        yield from hold(r, until)
        yield from r.move(r.at() - np.array([0.0, 0.03, -0.03]), t=0.3, label="hold: off")  # out and up, clear of the run's fall
        yield from r.move(r.at() - np.array([0.0, 0.06, -0.06]), t=0.5, label="hold: away")
        for k in range(3):
            yield from r.wait(1.0)
            print(f"[gate1] {k + 1} s after letting go: leans {leans()}", flush=True)
        yield from rest(r)
        return

    g_book = d.tilted(BOOK_TILT, BOOK_PHI)  # left fingers nearly level, pointing along the book from its spine, closing across its covers; the palm stays behind the spine

    def grip_point() -> np.ndarray:
        return on_book((-NEW_D / 2 + GRIP_BACK, 0.0, NEW_H / 2 - GRIP_DOWN))

    def pick(arm):
        yield from arm.move(grip_point() + np.array([-0.06, 0.0, 0.0]), grip=g_book, finger=FINGER_OPEN, t=2.5, label="pick: behind")
        yield from arm.servo(grip_point() + np.array([-0.06, 0.0, 0.0]), timeout=1.0, label="pick: behind")
        yield from arm.move(grip_point(), t=1.0, label="pick: in")
        yield from arm.servo(grip_point(), timeout=1.0, label="pick: in")
        yield from arm.close(NEW_T, force=SQUEEZE, label="pick: close")
        yield from arm.move(arm.at() + np.array([0.0, 0.0, 0.05]), t=0.8, label="pick: lift")
        print(f"[demo] green book lifted: tilt {tilt_deg(book()[1]):.1f} deg", flush=True)

    def fore_tail():
        return on_book((NEW_D / 2, 0.0, -NEW_H / 2))

    mouth = np.array([X_S - 0.008, SLOT_Y, TOP + Z_S + 0.007])  # the fore-edge's tail, just short of the gap, 7 mm up

    def square_servo(arm, goal, t: float, settle: float = 1.5, label: str = "square"):
        """Bring the book's fore-edge tail to `goal` (a point, or a function of the seconds since the start) with the book square: spine
        toward the room, boards upright. The wrist is soft and the pinch can turn, so the grip's turn is corrected from the book's own pose
        each step (integral), as `servo` corrects the position."""
        from yam_arena import ik
        from yam_arena.urdf import axis_angle_matrix

        G = arm.grip.rot.copy()
        c = ik.tcp_pose(arm.q, arm.base)[0]
        good, err, ang = 0, np.zeros(3), 0.0
        for k in range(round((t + settle) * 20)):
            g = np.asarray(goal(k * DT) if callable(goal) else goal, float)
            R = book()[1]
            rv = rotvec(R.T)  # the turn that would square the book, in the world
            ang = float(np.linalg.norm(rv))
            if ang > 1e-6:
                stp = min(0.3 * ang, math.radians(0.8))
                G = axis_angle_matrix(rv / ang, stp) @ G
            err = g - fore_tail()
            c = c + np.clip(0.2 * err, -0.005, 0.005)
            arm._solve(c, G)
            moving = callable(goal) and k * DT < t
            good = good + 1 if (not moving and np.linalg.norm(err) < 0.003 and ang < math.radians(1.0)) else 0
            yield
            if good >= 3:
                break
        arm.grip = d._as_grip(G)
        if good < 3:
            print(f"[demo] {label}: stopped {np.linalg.norm(err) * 1000:.1f} mm and {math.degrees(ang):.1f} deg short", flush=True)

    def carry(arm):
        p = fore_tail()
        yield from arm.move(arm.at() + (mouth + np.array([-0.03, 0.0, 0.03]) - p), t=2.0, label="carry")
        yield from square_servo(arm, mouth + np.array([-0.03, 0.0, 0.03]), 0.0, settle=2.0, label="carry: line up")
        yield from square_servo(arm, mouth, 0.0, settle=1.5, label="carry: at the gap")
        R = book()[1]
        print(f"[demo] at the gap: fore-edge tail off by {np.round((fore_tail() - mouth) * 1000, 1)} mm, pitch {math.degrees(math.asin(R[2, 0])):.1f} deg, "
              f"lean {lean_deg(R):.1f} deg", flush=True)

    def slide(arm, depth: float, t: float, start=None):
        """Slide the book straight into the gap by `depth`, square, steering its fore-edge's tail along the line."""
        a = mouth.copy() if start is None else np.asarray(start, float)

        def goal(s):
            return a + np.array([min(1.0, s / t) * depth, 0.0, 0.0])

        yield from square_servo(arm, goal, t, settle=1.5, label="slide in")
        R = book()[1]
        print(f"[demo] slid in: spine at {spine_x(*book(), NEW_D) * 1000:.0f} mm, pitch {math.degrees(math.asin(R[2, 0])):.1f} deg, lean {lean_deg(R):.1f} deg",
              flush=True)

    def let_book_go(arm):
        yield from arm.move(arm.at() - np.array([0.0, 0.0, 0.006]), t=0.4, label="down onto the base")
        yield from arm.open(label="let go")
        yield from arm.move(arm.at() - np.array([0.045, 0.0, 0.0]), t=0.8, label="back off the book")
        p, R = book()
        print(f"[demo] let go: spine at {spine_x(p, R, NEW_D) * 1000:.0f} mm, tilt {tilt_deg(R):.1f} deg", flush=True)
        yield from arm.move(arm.at() + np.array([0.0, 0.0, 0.04]), t=0.6, label="up")

    g_push = d.tilted(35.0, 0.0)  # fingers sloping forward onto the spine

    def push_spine(arm, to: float):
        """Push the spine's middle, PUSH_Z up it, until the spine reaches x `to`."""
        ax = g_push.rot[:, 2]
        p = on_book((-NEW_D / 2, 0.0, -NEW_H / 2 + PUSH_Z))
        back = p - np.array([0.025, 0.0, 0.0]) - TIP * ax
        yield from arm.move(back + np.array([0.0, 0.0, 0.06]), grip=g_push, finger=FINGER_CLOSED, t=1.5, label="push: over")
        yield from arm.move(back, t=0.8, label="push: down")
        yield from arm.servo(back, timeout=1.0, label="push: down")
        start = arm.at().copy()
        dist = to - p[0] + 0.025 + 0.006
        steps = [0]

        def goal():
            return start + np.array([min(1.0, steps[0] * DT / 2.0) * dist, 0.0, 0.0])

        for _ in arm.follow(goal, t=3.5, speed=0.04, until=lambda: spine_x(*book(), NEW_D) >= to - 0.001):
            steps[0] += 1
            yield
        print(f"[demo] spine at {spine_x(*book(), NEW_D) * 1000:.0f} mm (line {X_S * 1000:.0f})", flush=True)

    def insert(arm):
        yield from pick(arm)
        yield from carry(arm)
        while not flags["held"]:
            yield
        depth = LET_GO + NEW_D - mouth[0]
        if ctx.mode == "early_out":
            yield from slide(arm, 0.04, 1.2)
            flags["release"] = True
            yield from arm.wait(1.0)
            yield from slide(arm, depth - 0.04, 2.0, start=mouth + np.array([0.04, 0.0, 0.0]))
        else:
            yield from slide(arm, depth, 2.5)
        yield from let_book_go(arm)
        yield from push_spine(arm, X_S - (0.03 if ctx.mode == "proud" else 0.0))
        if ctx.mode == "hands_on":
            flags["release"] = True
            yield from arm.wait(5.0)
            return
        yield from arm.move(arm.at() - 0.05 * g_push.rot[:, 2], t=0.6, label="push: back")
        flags["release"] = True
        yield from rest(arm)

    if ctx.mode == "one_hand":
        yield from hold(r, lambda: flags["held"])
        yield from let_go(r)
        print(f"[one_hand] after letting go: leans {leans()}", flush=True)
        yield from insert(l)
        return

    if ctx.mode == "no_hold":
        flags["held"] = True
        yield from insert(l)
        return

    def right_side():
        yield from hold(r, lambda: flags["release"])
        yield from let_go(r)

    yield from ctx.together(right_side(), insert(l))
    yield from l.wait(1.0)
    print(f"[demo] end: leans {leans()}, green book tilt {tilt_deg(book()[1]):.1f} deg, spine {spine_x(*book(), NEW_D) * 1000:.0f} mm", flush=True)
