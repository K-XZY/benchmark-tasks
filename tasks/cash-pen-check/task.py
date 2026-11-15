"""Check a banknote with a counterfeit-detector pen: pin it flat with one hand, draw one stroke at least 1 cm long on it with the pen, put the pen
back, and sort the note by the ink's colour.

A US $20 note (156 x 66.3 mm, 1 g) lies flat on the counter. A counterfeit-detector pen stands uncapped in a pen cup. The pen's felt tip drags
on the paper harder than a 1 g note holds on the counter, so a stroke with the note unpinned slides it and smears; one hand presses the note's
short edge flat while the other draws. In this task the ink shows amber on genuine note paper and dark on a suspect one (the task's own convention:
the maker's guide says only that the ink detects note paper); the note then goes, slid along the counter under pressing fingertips and
lifted no more than 5 mm, onto the green accept mat (amber) or the red reject mat (dark).

The ink is code: a 3 mm dot shows on the note wherever the felt tip has touched the paper while moving across it.

Debug:  python demokit.py task.py OUT --no-record --stills 20 --mode full
Final:  python demokit.py task.py OUT
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
INSTRUCTION = ("Check the banknote with the counterfeit pen: hold the note flat by one end, draw one stroke at least 1 cm long on it, put the pen back "
               "in its cup, then slide the note onto the green mat if the mark is amber or onto the red mat if it is dark, and move your hands away.")
SUCCESS_HOLD_S = 0.0  # the scorer counts its own 1 s hold
HOLD_STEPS = 20
TABLE = d.TABLE_TOP
COUNTER_T = 0.002  # the counter's laminate top, 2 mm, over the station's table
TOP = TABLE + COUNTER_T  # the counter's surface: the note, the cup and the mats rest on it
DT = d.DT
TRACE = os.environ.get("CASH_TRACE", "1") != "0"
TRACE_NEAR = os.environ.get("CASH_TRACE_NEAR", "0") == "1"  # every step with the felt within 3 mm of the paper or a fingertip within 6 mm of the note: to calibrate TOUCH and PIN
SUSPECT = os.environ.get("CASH_NOTE", "genuine") == "suspect"  # which note this trial uses: genuine (amber mark) or suspect (dark mark)

LOOKS = {
    "note": m.Look(m.rgb(0.80, 0.84, 0.74), roughness=0.9),  # Federal Reserve note paper, its green-grey print
    "note_dark": m.Look(m.rgb(0.30, 0.40, 0.32), roughness=0.9),  # the portrait panel and borders
    "pen": m.Look(m.rgb(0.92, 0.92, 0.90), roughness=0.4),
    "pen_band": m.Look(m.rgb(0.10, 0.25, 0.65), roughness=0.4),
    "felt": m.Look(m.rgb(0.85, 0.80, 0.55), roughness=0.95),
    "cup": m.Look(m.rgb(0.15, 0.15, 0.16), roughness=0.5),
    "laminate": m.Look(m.rgb(0.86, 0.84, 0.80), roughness=0.5),  # the counter's light laminate
    "accept": m.Look(m.rgb(0.10, 0.45, 0.20), roughness=0.95),
    "reject": m.Look(m.rgb(0.65, 0.10, 0.08), roughness=0.95),
    "ink_amber": m.Look(m.rgb(0.85, 0.62, 0.15), roughness=0.9),
    "ink_dark": m.Look(m.rgb(0.20, 0.14, 0.08), roughness=0.9),
}

# -- the note: a US $20 Federal Reserve note, 156 x 66.3 mm, 1 g (BEP). Simulated as a rigid plate 1 mm thick (paper is about 0.11 mm): stated.
NOTE_L, NOTE_W, NOTE_T, NOTE_KG = 0.156, 0.0663, 0.001, 0.001
NOTE_AT = (0.33, 0.0)  # its middle; its length along y
# -- the pen: a counterfeit-detector felt-tip pen, 12 mm across and 136 mm long with its chisel tip, 10 g (estimates)
PEN_R, PEN_L, TIP_L, PEN_KG = 0.006, 0.136, 0.004, 0.010
CUP_AT, CUP_R, CUP_H = (0.24, -0.20), 0.011, 0.045
# -- the mats: green (accept) and red (reject) counting mats, 0.5 mm, flat on the counter
MAT = (0.12, 0.20)
ACCEPT_AT, REJECT_AT = (0.33, -0.27), (0.33, 0.27)
# -- the ink: 3 mm dots on a 4 mm grid over the note's middle (its pinned ends left clear)
DOT_PITCH, DOT_SIZE = 0.004, 0.003
DOT_X = np.arange(-0.024, 0.0241, DOT_PITCH)
DOT_Y = np.arange(-0.048, 0.0481, DOT_PITCH)
DOTS = [(i, j, float(x), float(y)) for i, x in enumerate(DOT_X) for j, y in enumerate(DOT_Y)]


def _note() -> m.Part:
    parts = [("paper", m.box((NOTE_W, NOTE_L, NOTE_T), (0.0, 0.0, NOTE_T / 2)), "note"),
             ("portrait", m.box((NOTE_W * 0.55, NOTE_L * 0.28, 0.0002), (0.0, 0.0, NOTE_T + 0.0001)), "note_dark")]
    for s, y in (("a", -1), ("b", 1)):
        parts.append((f"border_{s}", m.box((NOTE_W - 0.004, 0.003, 0.0002), (0.0, y * (NOTE_L / 2 - 0.004), NOTE_T + 0.0001)), "note_dark"))
    ink = "ink_dark" if SUSPECT else "ink_amber"
    for i, j, x, y in DOTS:
        parts.append((f"ink_{i}_{j}", m.box((DOT_SIZE, DOT_SIZE, 0.0002), (x, y, NOTE_T + 0.0003)), ink, True))
    cols = [m.col_box("c_paper", (NOTE_W, NOTE_L, NOTE_T), (0.0, 0.0, NOTE_T / 2))]
    return m.free("note", (*NOTE_AT, TOP + 0.0002), parts, cols, LOOKS, mass=NOTE_KG, friction=(0.30, 0.25), torsional_patch=0.01)


def _pen() -> m.Part:
    """The pen in its own frame: its axis along z, the felt tip's end at z = 0, the body above."""
    parts = [("body", m.cylinder(PEN_R, PEN_L - TIP_L - 0.006, (0.0, 0.0, TIP_L + 0.006 + (PEN_L - TIP_L - 0.006) / 2)), "pen"),
             ("band", m.cylinder(PEN_R + 0.0003, 0.020, (0.0, 0.0, PEN_L - 0.030)), "pen_band"),
             ("nose", m.cylinder(0.0035, 0.006, (0.0, 0.0, TIP_L + 0.003)), "pen"),
             ("felt", m.box((0.003, 0.0045, TIP_L), (0.0, 0.0, TIP_L / 2)), "felt")]
    cols = [m.col_cylinder("c_body", PEN_R, PEN_L - TIP_L, (0.0, 0.0, TIP_L + (PEN_L - TIP_L) / 2)), m.col_box("c_felt", (0.003, 0.0045, TIP_L), (0.0, 0.0, TIP_L / 2))]
    return m.free("pen", (*CUP_AT, TOP + 0.003), parts, cols, LOOKS, mass=PEN_KG, friction=(0.9, 0.8))


_LAMINATE: list = []


def _fixtures() -> m.Part:
    parts, cols = [], []
    ring = m.lathe([(CUP_R, 0.0), (CUP_R + 0.003, 0.0), (CUP_R + 0.003, CUP_H), (CUP_R, CUP_H)], 32)
    parts += [("cup", ring.moved((*CUP_AT, TOP)), "cup"), ("cup_floor", m.cylinder(CUP_R + 0.003, 0.003, (*CUP_AT, TOP + 0.0015)), "cup")]
    for k in range(12):  # the cup's wall as boxes round it
        a = 2 * math.pi * k / 12
        b = m.box((0.003, 2 * math.pi * (CUP_R + 0.0015) / 12 * 1.15, CUP_H), (CUP_R + 0.0015, 0.0, CUP_H / 2)).turned(math.degrees(a), "z").moved((*CUP_AT, TOP))
        cols.append(m.col_hull(f"c_cup_{k}", b))
    cols.append(m.col_box("c_cup_floor", (2 * CUP_R + 0.006, 2 * CUP_R + 0.006, 0.003), (*CUP_AT, TOP + 0.0015)))
    top = m.box((0.30, 0.80, COUNTER_T), (0.33, 0.0, TABLE + COUNTER_T / 2))  # the counter's laminate top: paper slides on it far more easily than a fingertip slides on paper
    parts.append(("laminate", top, "laminate"))
    lam = m.fixed("laminate", (0.0, 0.0, 0.0), [], [m.col_box("c_laminate", (0.30, 0.80, COUNTER_T), (0.33, 0.0, TABLE + COUNTER_T / 2))], LOOKS, friction=(0.20, 0.18))
    _LAMINATE.append(lam)
    for name, at, look in (("accept", ACCEPT_AT, "accept"), ("reject", REJECT_AT, "reject")):
        parts.append((name, m.box((MAT[0], MAT[1], 0.0005), (at[0], at[1], TOP + 0.00025)), look))  # visual only: the note slides over it
    return m.fixed("counter", (0.0, 0.0, 0.0), parts, cols, LOOKS, friction=(0.5, 0.4))


_FIX = _fixtures()
OBJECTS = [_FIX, _LAMINATE[0], _note(), _pen()]


# -- the scorer ------------------------------------------------------------------------------------------------------------------------------------

STROKE_MIN, STROKE_MOVING = 0.010, 0.005  # a stroke counted for 0.5: its mark spans 10 mm or more on the paper; moving: the tip 5 mm/s or faster across it
TOUCH = (-0.0015, float(os.environ.get("CASH_TOUCH_HI", "0.0006")))  # the felt touching the paper: its lowest point from 1.5 mm into the face to 0.6 mm above it (logged: -0.25 to 0.48 mm in contact during the stroke)
FIRM, FIRM_SHARE = 0.0003, float(os.environ.get("CASH_FIRM_SHARE", "0.5"))  # a counted stroke: at least this share of its touching points read within 0.3 mm of the face (a felt riding 0.4 to 0.9 mm over the paper, as in hoverpen, does not)
PIN = (float(os.environ.get("CASH_PIN_LO", "-0.0015")), float(os.environ.get("CASH_PIN_HI", "0.0000")))  # a fingertip pressing the note: its lowest reported point from 1.5 mm into the face to the face itself (logged: -0.26 to -0.32 mm pressing, 4.2 mm held just over the note)
LIFT_MAX = 0.005  # the note lifted more than 5 mm off the counter: lifted
SLIP_STROKE, SLIP_SMEAR = 0.002, 0.005  # the note may move under 2 mm during a counted stroke; over 5 mm during any stroke is a smear (cap 0.5)
MAX_STROKES = 3
CLEAR = 0.03
PEN_GRIP = gk.Grip(half=PEN_R, axis="z", lo=(-1.0, -1.0, 0.030), hi=(1.0, 1.0, PEN_L))


FELT_CORNERS = np.array([[sx * 0.0015, sy * 0.00225, 0.0] for sx in (-1, 1) for sy in (-1, 1)] + [[0.0, 0.0, 0.0]])


def tip_world(pen) -> np.ndarray:
    """The felt tip's lowest point now (of its end's corners and middle): the point that touches the paper, whatever the pen's lean."""
    pts = pen.to_world(FELT_CORNERS)
    return pts[int(np.argmin(pts[:, 2]))]


def on_paper(note, p) -> tuple[bool, np.ndarray]:
    """Whether a world point is on the note's top face (inside its outline, 1 mm in from the edge, within 1 mm of the face), and its place in the note's frame."""
    q = note.to_local(np.atleast_2d(p))[0]
    inside = abs(q[0]) <= NOTE_W / 2 - 0.001 and abs(q[1]) <= NOTE_L / 2 - 0.001
    return bool(inside and TOUCH[0] <= q[2] - NOTE_T <= TOUCH[1]), q


def pinned(note, tips) -> bool:
    """A fingertip pressing the note flat: a point inside its outline, at one of its ends (the outer 28 mm), not over the stroke area, and at the
    height a fingertip pressing the face reads (PIN)."""
    q = note.to_local(np.asarray(tips))
    inside = (np.abs(q[:, 0]) <= NOTE_W / 2) & (np.abs(q[:, 1]) <= NOTE_L / 2) & (np.abs(q[:, 1]) >= 0.050)
    h = q[:, 2] - NOTE_T
    return bool(np.any(inside & (h >= PIN[0]) & (h <= PIN[1])))


def pin_height(note, tips) -> float:
    """The lowest reported fingertip point over the note's ends, for the trace (inf if none)."""
    q = note.to_local(np.asarray(tips))
    inside = (np.abs(q[:, 0]) <= NOTE_W / 2) & (np.abs(q[:, 1]) <= NOTE_L / 2) & (np.abs(q[:, 1]) >= 0.050)
    return float((q[inside, 2] - NOTE_T).min()) if inside.any() else math.inf


def on_mat(note, at) -> bool:
    """The note flat on a mat: all four of its corners over the mat, tilted under 10 degrees, down on the counter."""
    corners = note.to_world(np.array([[sx * NOTE_W / 2, sy * NOTE_L / 2, 0.0] for sx in (-1, 1) for sy in (-1, 1)]))
    over = np.all((np.abs(corners[:, 0] - at[0]) <= MAT[0] / 2) & (np.abs(corners[:, 1] - at[1]) <= MAT[1] / 2))
    return bool(over and note.tilt_deg() < 10.0 and note.pos[2] < TOP + 0.003)


LADDER = gk.Ladder(("touched", 0.25), ("stroke", 0.5), ("pen_back", 0.75), ("sorted", 1.0))


class Trial:
    """The trial's record: strokes, the note's slip under each, the ink, the latches."""

    def __init__(self):
        self.on_counter, self.too_many, self.lifted, self.dropped, self.wrong_mat = (gk.Latch(w) for w in ("ink on the counter", "more than 3 strokes", "note lifted", "pen dropped", "wrong mat"))
        self.hold = gk.EndHold(HOLD_STEPS)
        self.reset()

    def reset(self, env=None):
        self.n = 0
        self.touching, self.gap, self.path, self.start_pos, self.slip = False, 0, 0.0, None, 0.0
        self.n_touch, self.n_firm = 0, 0
        self.strokes, self.good_stroke, self.touched_pinned, self.smear = 0, False, False, False
        self.marks, self.span = [], 0.0
        self.inked = set()
        self.last_q = None
        self.reached = {}
        self.full = False
        for latch in (self.on_counter, self.too_many, self.lifted, self.dropped, self.wrong_mat):
            latch.reset()
        self.hold.reset()

    def step(self, env):
        self.n += 1
        note, pen = m.state(env, "note"), m.state(env, "pen")
        tips = m.fingertips(env)
        tip = tip_world(pen)
        touch, q = on_paper(note, tip)
        rel_v = float(np.linalg.norm((pen.lin_vel + np.cross(pen.ang_vel, tip - pen.pos)) - note.lin_vel))
        pin = pinned(note, tips)
        held_pen = gk.held(tips, pen, PEN_GRIP, resting=False)  # resting False: the check is only for a falling pen, off any rest
        # ink on the counter: the tip down on the counter, off the note
        if not touch and tip[2] < TOP + 0.001 and not self._in_cup(tip):
            self.on_counter.trip(True)
        if touch:
            self.touched_pinned |= pin
            if not self.touching:
                self.touching, self.path, self.start_pos, self.slip = True, 0.0, note.pos.copy(), 0.0
                self.n_touch, self.n_firm = 0, 0
                self.marks = []
                self.pin_through = pin
            self.gap = 0
            self.n_touch += 1
            self.n_firm += int(q[2] - NOTE_T <= FIRM)
            self.pin_through = self.pin_through and pin
            if self.last_q is not None and rel_v >= STROKE_MOVING:
                self.path += float(np.linalg.norm(q[:2] - self.last_q[:2]))
                self._ink(q, env)
                self.marks.append(q[:2].copy())  # the mark: where ink is laid, the felt moving over the paper
            self.last_q = q
            self.slip = max(self.slip, float(np.linalg.norm(note.pos[:2] - self.start_pos[:2])))
        elif self.touching:
            self.gap += 1
            if self.gap > 3:  # the tip off the paper: the stroke ends
                self.touching, self.last_q = False, None
                self.strokes += 1  # every touch-down is a stroke, a dab included
                self.too_many.trip(self.strokes > MAX_STROKES)
                pts = np.asarray(self.marks)
                self.span = float(np.max(np.linalg.norm(pts[:, None, :] - pts[None, :, :], axis=2))) if len(pts) > 1 else 0.0  # the inked mark's length on the paper
                firm = self.n_firm >= FIRM_SHARE * self.n_touch  # the felt pressed on the paper for most of the stroke, not riding over it
                if self.span >= STROKE_MIN and self.slip < SLIP_STROKE and self.pin_through and firm:
                    self.good_stroke = True
                if self.slip > SLIP_SMEAR:
                    self.smear = True
        self.lifted.trip(note.pos[2] > TOP + LIFT_MAX)
        self.dropped.trip(pen.lin_vel[2] < -0.8 and not held_pen)
        pen_back = self._in_cup(tip) and pen.tilt_deg() < 20.0 and not held_pen
        right_mat, wrong = (REJECT_AT, ACCEPT_AT) if SUSPECT else (ACCEPT_AT, REJECT_AT)
        note_still = float(np.linalg.norm(note.lin_vel)) < 0.005
        self.wrong_mat.trip(on_mat(note, wrong) and note_still and not pinned(note, tips))
        clear = gk.hands_clear(tips, [gk.along(note, (x, -NOTE_L / 2, NOTE_T), (x, NOTE_L / 2, NOTE_T), 9) for x in (-NOTE_W / 2, 0.0, NOTE_W / 2)], CLEAR)  # 3 cm from the note's plate: its two long edges and its middle
        sorted_ok = self.good_stroke and pen_back and on_mat(note, right_mat) and note_still and clear
        self.full = self.hold.step(sorted_ok)
        self.reached = {"touched": self.touched_pinned, "stroke": self.good_stroke, "pen_back": self.good_stroke and pen_back, "sorted": self.full}
        if TRACE_NEAR and (q[2] - NOTE_T < 0.003 or pin_height(note, tips) < 0.006):
            print(f"[cash-near] step {self.n}: felt {1000 * (q[2] - NOTE_T):.2f} mm, pin {1000 * pin_height(note, tips):.2f} mm, touching {touch}, pinned {pin}, v {1000 * rel_v:.0f} mm/s", flush=True)
        if TRACE and self.n % 20 == 0:
            print(f"[cash] step {self.n}: tip {np.round(1000 * (tip - [0, 0, TOP]), 1)} mm, felt {1000 * (q[2] - NOTE_T):.2f} mm over the paper, pin points {1000 * pin_height(note, tips):.2f} mm; touching {touch}, pinned {pin}, pen held {held_pen}; stroke path {1000 * self.path:.1f} mm, span {1000 * self.span:.1f} mm, "
                  f"note slip {1000 * self.slip:.1f} mm, strokes {self.strokes}, good {self.good_stroke}, inked {len(self.inked)}; note at {np.round(1000 * (note.pos - [0, 0, TOP]), 1)}; "
                  f"pen back {pen_back}, on its mat {on_mat(note, right_mat)}, still {note_still}, clear {clear}, held {self.hold.count}; score {self.score()}", flush=True)

    def _in_cup(self, tip) -> bool:
        return math.hypot(tip[0] - CUP_AT[0], tip[1] - CUP_AT[1]) < CUP_R and tip[2] < TOP + 0.010

    def _ink(self, q, env) -> None:
        i = int(round((q[0] - DOT_X[0]) / DOT_PITCH))
        j = int(round((q[1] - DOT_Y[0]) / DOT_PITCH))
        if 0 <= i < len(DOT_X) and 0 <= j < len(DOT_Y) and (i, j) not in self.inked:
            self.inked.add((i, j))
            m.set_visible(f"{m.ENV_PATH}/note/visual/ink_{i}_{j}", True)

    def score(self) -> float:
        failed = any(bool(x) for x in (self.on_counter, self.too_many, self.lifted, self.dropped, self.wrong_mat))
        s = LADDER.score(self.reached, failed=failed)
        return min(s, 0.5) if self.smear else s


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.score() >= 1.0


def score(objects) -> float:
    return TRIAL.score()


# -- the demo ----------------------------------------------------------------------------------------------------------------------------------------

TIP_BELOW_TCP = 0.013  # the fingertips' ends below the TCP
GRIP_FROM_TOP = 0.030  # where the right hand takes the pen, below its top


def demo_script(ctx):
    """Modes: full: the left presses the note's +y end flat; the right takes the pen from its cup, draws one 15 mm stroke across the note's middle,
    puts the pen back; then the right presses the note's middle and slides it onto the mat its mark calls for. nopin: the same, the note not
    pinned (it slides with the pen and smears). none: the arms wait."""
    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode == "none":
        for _ in range(60):
            yield
        return
    if mode not in ("full", "nopin", "hoverpin", "hoverpen"):
        raise ValueError(f"unknown mode {mode!r}")
    note = ctx.state("note")
    pen = ctx.state("pen")
    pin_at = note.to_world(np.array([[0.0, 0.062, NOTE_T]]))[0]
    if mode in ("full", "hoverpin", "hoverpen"):  # 1. the left presses the note's +y end flat with two fingertips 59 mm apart across it, so it cannot pivot about one point
        yield from l.move(pin_at + [0.0, 0.0, TIP_BELOW_TCP + 0.05], grip=d.down(-90.0), finger=d.FINGER_CLOSED, t=2.0, label="over the note's end")
        yield from l.close(0.050, force=0.0, label="fingers apart, across the note")
        down = 0.004 if mode == "hoverpin" else -0.003  # hoverpin: the fingertips stop above the note (the check that a hover is not a pin)
        yield from l.move(pin_at + [0.0, 0.0, TIP_BELOW_TCP + down], t=1.0, label="press its end flat" if mode != "hoverpin" else "fingertips just over its end")
    # 2. the right takes the pen from its cup
    top = pen.to_world(np.array([[0.0, 0.0, PEN_L]]))[0]
    grip_at = top - [0.0, 0.0, GRIP_FROM_TOP]
    yield from r.move(grip_at + [0.0, 0.0, 0.08], grip=d.down(90.0), finger=d.FINGER_OPEN, t=2.0, label="over the pen")
    yield from r.move(grip_at, t=1.0, label="fingers either side of the pen")
    yield from r.close(2 * PEN_R, force=15.0, label="take the pen")
    yield from r.move(r.at() + [0.0, 0.0, 0.16], t=1.2, label="lift it out of the cup")
    tcp_over_tip = PEN_L - GRIP_FROM_TOP  # the TCP behind the pen's tip along the pen, as held
    lean = d.tilted(30.0, 90.0)  # the pen leaning 30 degrees, its tip toward +y: the hand stays 5 cm clear of the left hand pinning the note
    back = -tcp_over_tip * lean.rot[:, 2]  # from the tip to the TCP
    mid = note.to_world(np.array([[0.0, -0.008, NOTE_T]]))[0]
    yield from r.move(mid + back + [0.0, 0.0, 0.03], grip=lean, t=2.0, label="the pen over the note's middle, leaning")
    lift = 0.0015 if mode == "hoverpen" else -0.0015  # hoverpen: the felt stops 1.5 mm over the paper and strokes in the air. Pressed only 0.5 mm in, the felt rode up off the paper within 4 mm of the stroke
    yield from r.move(mid + back + [0.0, 0.0, lift], t=1.0, label="the felt tip down on the paper, lightly" if mode != "hoverpen" else "the felt just over the paper")
    yield from r.move(r.at() + [0.0, -0.015, -0.001 if mode != "hoverpen" else 0.0], t=1.5, label="draw one 15 mm stroke, pressing on")
    yield from r.move(r.at() + [0.0, 0.0, 0.03], t=0.6, label="lift the tip off")
    # 3. the pen back in its cup
    yield from r.move(np.array([CUP_AT[0], CUP_AT[1], TOP + tcp_over_tip + 0.06]), grip=d.down(90.0), t=2.0, label="over the cup, upright")
    yield from r.move(np.array([CUP_AT[0], CUP_AT[1], TOP + 0.003 + tcp_over_tip + 0.005]), t=1.5, label="the pen down into the cup")
    yield from r.open(label="let go of the pen")
    yield from r.move(r.at() + [0.0, 0.0, 0.10], t=1.0, label="up off the pen")
    if mode in ("full", "hoverpin", "hoverpen"):
        yield from l.move(l.at() + [0.0, 0.0, 0.05], t=0.8, label="the left off the note")
        yield from l.move(l.at() + [-0.08, 0.12, 0.12], t=1.2, label="the left out of the way")
    # 4. the note slid along the counter under the pressing fingertips onto the mat its mark calls for (the mark's colour, as seen, chooses the
    # mat): never lifted
    to = REJECT_AT if SUSPECT else ACCEPT_AT
    note = ctx.state("note")
    hand = r if to[1] < 0 else l
    g = d.down(0.0)  # the fingers open along the note's length: two fingertips about 9 cm apart press it, so it slides without turning (one point pressed, it turned off its mat)
    press = note.to_world(np.array([[0.0, 0.0, NOTE_T]]))[0]
    yield from hand.move(press + [0.0, 0.0, TIP_BELOW_TCP + 0.05], grip=g, finger=d.FINGER_OPEN, t=1.5, label="over the note's middle, fingers open along it")
    yield from hand.move(press + [0.0, 0.0, TIP_BELOW_TCP - 0.005], t=0.8, label="both fingertips pressed on the note")
    yield from hand.move(np.array([to[0], to[1], hand.at()[2]]), t=4.0, label="slide the note onto its mat")
    note_z = ctx.state("note").pos[2]
    yield from hand.servo(np.array([to[0], to[1], note_z]), tol=0.004, timeout=3.0, gain=0.3, read=lambda: ctx.state("note").pos,
                          label="the note's middle onto the mat's middle")  # the note lags the hand as the fingertips slip on it: read where it is
    yield from hand.move(hand.at() + [0.0, 0.0, 0.08], t=0.8, label="lift off the note")
    yield from ctx.together(l.move(l.at() + [-0.03, 0.03, 0.03], t=1.0, label="the left away"), r.move(r.at() + [-0.05, -0.05, 0.05], t=1.0, label="the right away"))
    yield from r.wait(1.5)
