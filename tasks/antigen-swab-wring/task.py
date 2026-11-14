"""Wring a throat swab out in a rapid strep test's extraction tube: one hand pinches the soft tube onto the swab head and keeps the pinch while the other
turns the swab five times inside it, then draws the head up and out through the pinch, as the insert says: "Rotate swab 5 times while squeezing, keep
swab in test tube for 1 minute, and remove swab while rotating and squeezing the tube before testing extracted material."

A clinic bench: the extraction tube, buffer in it, stands in a holder by its lowest 10 mm; the throat swab lies on the test kit's box with its handle over
the box's edge; a waste bin stands to the left.

Model (declared in code): the tube is fixed in its holder. Each of its two soft sides is 8 sprung segments, 9 mm each, from the floor to the rim, so a
squeeze dents the tube only where the fingers press (each segment 0.25 N/mm, 4.5 mm travel; with no head between them they close fully). The other two
walls are rigid. The buffer is not simulated; "expressed" is the swab's turning, counted while the segments on its head bear on it. The insert's minute
is timed: the swab must stay in the tube, its head at the bottom, for 60 s after the 5 turns before the draw counts. The draw counts only if, at every
step the head rises in the tube, the segments on it are squeezed: a squeeze left at the bottom while the head rises out of it does not count.

    python demokit.py task.py OUT --mode full
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import mechkit as m  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = ("Do the strep test's swab step: put the swab head down to the bottom of the tube, squeeze the tube on the head and turn the swab 5 times, "
               "leave it in the tube for 1 minute, then take the swab out while squeezing the tube on the head and turning, drop it in the bin, let go of "
               "the tube and move both hands away.")
SUCCESS_HOLD_S = 0.0  # the scorer counts its own hold
HOLD_STEPS = 20  # 1 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# -- the tube (station frame, heights above the table): axis at (TX, TY), its outer bottom at ZB
TX, TY, ZB = 0.28, 0.08, 0.015
T_LEN, R_IN, R_OUT = 0.075, 0.005, 0.0065  # 75 mm long, 10 mm bore; the pads' outer faces at 6.5 mm
PZ0, PZ1 = 0.001, T_LEN  # the soft walls, from the floor to the rim: a rigid ledge anywhere along the bore catches a squeezed head, or stops a squeezing finger
PAD_T, PAD_W = 0.0015, 0.009  # each pad's thickness and width (along x)
PAD_K, PAD_TRAVEL = 250.0, 0.0045  # each wall segment's spring (N/m) and travel: a finger's 36 mm presses four of them, about 2.5 N to reach the head
SEG_N = 8  # each soft wall in 8 segments along the tube, 9 mm each: a squeeze dents the tube only where the fingers are
SEG_L = (PZ1 - PZ0) / SEG_N
HOLD_DEPTH = 0.010  # the tube stands in its holder by its lowest 10 mm
X_IN = 0.0035  # the side walls' inner faces, across the squeeze
WALL_Y = 0.0035  # the rigid side walls' half-width across y: inside the pads' squeezed faces, so fingers meet only the pads
# -- the swab: head (flocked) and shaft; its origin at the head's tip, its axis +z towards the handle
HEAD_R, HEAD_L, SHAFT_R, SWAB_L = 0.0025, 0.020, 0.0015, 0.150
GRIP_PICK, GRIP_TURN = 0.135, 0.105  # where the left hand holds the swab, from the head's tip
PRESS = (0.0023, 0.0034)
BOTTOM = 0.010  # turns count only with the head's tip within 10 mm of the tube's floor
WAIT_STEPS = 1200  # the insert's 1 minute in the tube (20 Hz), after the 5 turns and before the swab is drawn out
DEPTH_BAND = T_LEN - 0.001 - BOTTOM  # a band on the shaft this far from the head's tip: at or below the rim, the tip is within 10 mm of the floor
DRAW_STEP = 0.010  # the demo's draw: 10 mm a lift, the squeeze moved up after each
DRAW_SLACK = 0.001  # during the draw, the head may stand at most 1 mm above the highest point it reached squeezed (an eased squeeze, a regrip)
MAX_RISE = 0.003  # the most the head may rise in one step (60 mm/s) during the draw: a yank between two frames is not a squeezed draw
DRAW_TURN = 0.15  # the draw must turn the swab at least this much (turns) while squeezed
# -- the kit's box and the bin
BOX_C, BOX_S = (0.20, 0.20), (0.10, 0.10, 0.05)
SWAB_START = np.array([0.20, 0.25, BOX_S[2] + HEAD_R + 0.0005])  # the head's tip; the swab lies along -y, its handle 5 cm over the box's edge
BIN_C, BIN_S = (0.07, 0.20), (0.12, 0.12, 0.08)

LOOKS = {
    "tube": m.Look(color=m.rgb(0.92, 0.93, 0.95), roughness=0.35),
    "cap": m.Look(color=m.rgb(0.20, 0.45, 0.80), roughness=0.5),
    "stand": m.Look(color=m.rgb(0.30, 0.30, 0.33), roughness=0.6),
    "box": m.Look(color=m.rgb(0.95, 0.95, 0.97), roughness=0.7),
    "print": m.Look(color=m.rgb(0.75, 0.10, 0.20), roughness=0.6),
    "bin": m.Look(color=m.rgb(0.85, 0.20, 0.15), roughness=0.6),
    "shaft": m.Look(color=m.rgb(0.96, 0.96, 0.96), roughness=0.5),
    "head": m.Look(color=m.rgb(0.95, 0.90, 0.75), roughness=0.95),
    "mark": m.Look(color=m.rgb(0.10, 0.35, 0.80), roughness=0.5),
}


def _b(name, size, c, look, cols=None):
    if cols is not None:
        cols.append(m.col_box(name + "_c", size, c))
    return (name, m.box(size, c), look)


def _fixed():
    """The tube stand, the kit's box and the bin."""
    parts, cols = [], []
    # the holder: a block with a socket the tube stands in by its lowest 10 mm (the tube's lower part is free for the squeeze)
    hz = ZB + HOLD_DEPTH
    hw, hole = 0.040, R_OUT + 0.0005
    parts.append(_b("holder_base", (hw, hw, ZB), (TX, TY, ZB / 2), "stand", cols))
    for s_ in (1, -1):
        parts.append(_b(f"holder_x{'pn'[s_ < 0]}", ((hw / 2 - hole), hw, HOLD_DEPTH), (TX + s_ * (hole + (hw / 2 - hole) / 2), TY, ZB + HOLD_DEPTH / 2), "stand", cols))
        parts.append(_b(f"holder_y{'pn'[s_ < 0]}", (2 * hole, (hw / 2 - hole), HOLD_DEPTH), (TX, TY + s_ * (hole + (hw / 2 - hole) / 2), ZB + HOLD_DEPTH / 2), "stand", cols))
    # the kit's box, its lid printed
    bx, by = BOX_C
    parts.append(_b("box", BOX_S, (bx, by, BOX_S[2] / 2), "box", cols))
    parts.append(_b("box_print", (0.06, 0.03, 0.0006), (bx - 0.01, by + 0.02, BOX_S[2] + 0.0003), "print"))
    # the bin: an open box
    cx, cy = BIN_C
    sx, sy, sz = BIN_S
    t = 0.003
    parts.append(_b("bin_floor", (sx, sy, t), (cx, cy, t / 2), "bin", cols))
    for s in (1, -1):
        parts.append(_b(f"bin_x{'pn'[s < 0]}", (t, sy, sz), (cx + s * (sx - t) / 2, cy, sz / 2), "bin", cols))
        parts.append(_b(f"bin_y{'pn'[s < 0]}", (sx, t, sz), (cx, cy + s * (sy - t) / 2, sz / 2), "bin", cols))
    return parts, cols


_fp, _fc = _fixed()
FIXED = m.fixed("bench", (0.0, 0.0, TOP), _fp, _fc, LOOKS, friction=(0.8, 0.7))


def _tube():
    """The tube's rigid parts (frame) and its two soft-wall pads (links), in station coordinates."""
    parts, cols = [], []
    z0, z1 = ZB, ZB + T_LEN
    zs0, zs1 = ZB + PZ0, ZB + PZ1
    # bottom, and the shell above and below the soft zone (visual: round; colliders: four flat walls)
    parts.append(("shell_lo", m.lathe([(0.0, z0), (R_OUT - 0.0005, z0), (R_OUT - 0.0005, zs0)], 24).moved((TX, TY, 0.0)), "tube"))
    if z1 - zs1 > 1e-4:
        parts.append(("shell_hi", m.lathe([(R_IN, z1), (R_IN, zs1), (R_OUT - 0.0005, zs1), (R_OUT - 0.0005, z1), (R_IN, z1)], 24).moved((TX, TY, 0.0)), "tube"))
    parts.append(("rim", m.lathe([(R_IN, z1), (R_OUT, z1), (R_OUT, z1 + 0.003), (R_IN, z1 + 0.003)], 24).moved((TX, TY, 0.0)), "cap"))
    cols.append(m.col_box("floor_c", (2 * R_OUT, 2 * R_OUT, 0.001), (TX, TY, z0 + 0.0005)))
    for s in (1, -1):
        # the side walls across the squeeze: 3 mm thick, 1 mm from the head, so a squeezed head cannot squirt out sideways (pads may overlap them:
        # links never touch their own frame)
        cols.append(m.col_box(f"wx{'pn'[s < 0]}_c", (0.003, 2 * WALL_Y, T_LEN), (TX + s * (X_IN + 0.0015), TY, z0 + T_LEN / 2)))
        parts.append(_b(f"wxv{'pn'[s < 0]}", (0.001, 2 * WALL_Y, PZ1 - PZ0), (TX + s * (R_IN + 0.0005), TY, (zs0 + zs1) / 2), "tube"))
        for zz0, zz1, nm in ((z0, zs0, "lo"), (zs1, z1, "hi")):
            if zz1 - zz0 < 1e-4:
                continue
            cols.append(m.col_box(f"wy{'pn'[s < 0]}{nm}_c", (2 * R_IN, 0.001, zz1 - zz0), (TX, TY + s * (R_IN + 0.0005), (zz0 + zz1) / 2)))
    links = []
    for s, nm in ((1, "pad_p"), (-1, "pad_n")):
        lo, hi = (-PAD_TRAVEL, 0.0) if s > 0 else (0.0, PAD_TRAVEL)
        for k in range(SEG_N):
            c = (TX, TY + s * (R_OUT - PAD_T / 2), zs0 + (k + 0.5) * SEG_L)
            pad = m.box((PAD_W, PAD_T, SEG_L - 0.0002), c)
            links.append(m.slider(f"{nm}{k}", "Y", lo, hi, [(f"{nm}{k}_v", pad, "tube")], [m.col_box(f"{nm}{k}_c", (PAD_W, PAD_T, SEG_L - 0.0002), c)], at=c,
                                  mass=0.001, spring=PAD_K, damping=1.5, effort=200.0, friction=0.0, inertia=(1e-8, 1e-8, 1e-8)))
    return parts, cols, links


_tp, _tc, _tl = _tube()
TUBE = m.mechanism("tube", (0.0, 0.0, TOP), _tl, LOOKS, frame_parts=_tp, frame_colliders=_tc, friction=(0.10, 0.08), solver_iters=48)  # wet walls on a wet flocked head: slippery


HEAD_PROFILE = [(0.0, 0.0), (0.6 * HEAD_R, 0.0), (HEAD_R, 0.003), (HEAD_R, HEAD_L - 0.004), (SHAFT_R, HEAD_L), (0.0, HEAD_L)]  # rounded tip, tapered into the shaft


def _swab():
    """Origin at the head's tip; the swab along +z. A blue band near the handle's end shows its turning."""
    parts = [("head", m.lathe(HEAD_PROFILE, 16), "head"),
             ("shaft", m.cylinder(SHAFT_R, SWAB_L - HEAD_L, (0.0, 0.0, HEAD_L + (SWAB_L - HEAD_L) / 2), segments=12), "shaft"),
             ("tab", m.box((0.005, 0.0012, 0.010), (SHAFT_R + 0.0024, 0.0, SWAB_L - 0.030)), "mark"),  # a flag on one side: shows the turn
             ("band", m.cylinder(SHAFT_R + 0.0003, 0.002, (0.0, 0.0, DEPTH_BAND + 0.001), segments=12), "mark")]  # the depth band: level with the rim, the tip is at the bottom
    cols = [m.col_hull("head_c", m.lathe(HEAD_PROFILE, 12)),  # a flocked head tapers into the shaft: drawn up, it parts a squeeze rather than catching on it
            m.col_hull("shaft_c", m.cylinder(SHAFT_R, SWAB_L - HEAD_L, (0.0, 0.0, HEAD_L + (SWAB_L - HEAD_L) / 2), segments=8))]
    return parts, cols


_sp, _sc = _swab()
SWAB = m.free("swab", tuple(SWAB_START + np.array([0.0, 0.0, TOP])), _sp, _sc, LOOKS, mass=0.008, friction=(0.3, 0.25),
              start_quat=(math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4)))  # +z along -y: the head towards +y, on the box
OBJECTS = [FIXED, TUBE, SWAB]


# -- the scorer -----------------------------------------------------------------------------------------------------------------------------------
def _spin(R) -> float:
    """The swab's turn about its own axis, from its rotation: its x against the world's x, both seen square to the axis."""
    ax, xx = R[:, 2], R[:, 0]
    ref = np.array([1.0, 0.0, 0.0]) - ax * ax[0]
    ref = ref / (np.linalg.norm(ref) + 1e-9)
    return math.atan2(float(np.dot(np.cross(ref, xx), ax)), float(np.dot(ref, xx)))


def now(env) -> dict:
    s = m.state(env, "swab")
    R = s.rotation()
    tip = s.pos - np.array([0.0, 0.0, TOP])
    axis = R[:, 2]
    tube = m.state(env, "tube")
    travel = [(-tube.joint(f"pad_p{k}"), tube.joint(f"pad_n{k}")) for k in range(SEG_N)]  # each segment's travel in, on both sides
    off = math.hypot(tip[0] - TX, tip[1] - TY)
    head_lo, head_hi = tip[2], tip[2] + HEAD_L * axis[2]
    overlap = min(head_hi, ZB + PZ1) - max(head_lo, ZB + PZ0)
    upright = axis[2] > math.cos(math.radians(15.0))
    in_tube = off < R_IN and upright and head_lo < ZB + T_LEN
    seg_on = [all(PRESS[0] <= t <= PRESS[1] for t in tr) for tr in travel]  # both sides of the segment pressed in onto something
    # the head's length inside segments squeezed onto it: the squeeze is where the fingers are, not along the whole tube
    squeezed = sum(max(0.0, min(head_hi, ZB + PZ0 + (k + 1) * SEG_L) - max(head_lo, ZB + PZ0 + k * SEG_L)) for k in range(SEG_N) if seg_on[k])
    pressed = squeezed > 0.005
    pinched = in_tube and overlap > 0.005 and pressed
    at_bottom = head_lo - (ZB + 0.001) < BOTTOM
    cx, cy = BIN_C
    mid = s.to_world(np.array([[0.0, 0.0, SWAB_L / 2]]))[0] - np.array([0.0, 0.0, TOP])
    along = s.to_world(np.array([[0.0, 0.0, z] for z in np.linspace(0.0, SWAB_L, 7)])) - np.array([0.0, 0.0, TOP])  # points along the whole swab
    in_bin = abs(mid[0] - cx) < BIN_S[0] / 2 and abs(mid[1] - cy) < BIN_S[1] / 2 and mid[2] < BIN_S[2]
    tips = m.fingertips(env) - np.array([0.0, 0.0, TOP])
    to_swab = float(np.min(np.linalg.norm(tips[:, None, :] - along[None, :, :], axis=2)))
    clear = float(np.min(np.linalg.norm(tips[:, :2] - [TX, TY], axis=1))) > 0.05 and to_swab > 0.05
    fell = bool(tip[2] < -0.01 or mid[2] < -0.01)
    return {"spin": _spin(R), "travel": travel, "seg_on": seg_on, "squeezed": squeezed, "pressed": pressed, "pinched": pinched, "in_tube": in_tube, "overlap": overlap, "head_lo": head_lo,
            "in_bin": in_bin, "clear": clear, "fell": fell, "off": off, "at_bottom": at_bottom}


class Trial:
    """Each step: turns counted while pinched at the bottom; then the insert's minute with the swab in the tube; then the draw, the head squeezed wherever
    it is at every step it rises, out of the tube; the final state, held 1 s."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.turns = 0.0
        self.net = 0.0
        self.last = None
        self.was_pinched = False
        self.waited = 0  # steps the swab has stayed in the tube, its head at the bottom, since the 5 turns
        self.wrung = False
        self.drawing = False
        self.draw_turn = 0.0
        self.draw_top = 0.0
        self.last_lo = None
        self.fell = False
        self.prev_bottom = False
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.c = None

    def step(self, env):
        c = now(env)
        self.c = c
        self.fell = self.fell or c["fell"]
        d_turn = 0.0 if self.last is None else ((c["spin"] - self.last + math.pi) % (2 * math.pi) - math.pi) / (2 * math.pi)
        self.last = c["spin"]
        rise = 0.0 if self.last_lo is None else c["head_lo"] - self.last_lo
        self.last_lo = c["head_lo"]
        both = c["pinched"] and self.was_pinched
        if both and c["at_bottom"] and not self.drawing:
            self.net += d_turn  # signed: turning back undoes it; counted only with the head at the tube's bottom
            self.turns = abs(self.net)
        # the minute: after the 5 turns, the swab left in the tube with its head at the bottom, 60 s in a row (squeezed or not); lifted early, it starts again
        if self.turns >= 5.0 and self.waited < WAIT_STEPS:
            self.waited = self.waited + 1 if (c["in_tube"] and c["at_bottom"]) else 0
        waited = self.waited >= WAIT_STEPS
        # the draw: from the head squeezed at the bottom after the minute, up and out, squeezed wherever the head is at every step it rises, turning
        if not self.drawing and waited and self.prev_bottom and c["pinched"] and not c["at_bottom"]:
            self.drawing, self.draw_turn, self.draw_top = True, 0.0, c["head_lo"]  # it starts only from the head squeezed at the bottom
        if self.drawing:
            if both:
                self.draw_turn += d_turn
            if c["pinched"]:
                self.draw_top = max(self.draw_top, c["head_lo"])  # the highest the head has been while squeezed
            in_walls = c["off"] < R_IN and c["overlap"] > 0.005  # the head in the soft walls, upright or leaning
            if rise > MAX_RISE:
                self.drawing = False  # the head jumped further up in one step than a squeeze could be seen on it: no wring
            elif in_walls and not c["pinched"] and c["head_lo"] > self.draw_top + DRAW_SLACK:
                self.drawing = False  # the head has risen, however slowly, past where it was last squeezed: no wring
                if os.environ.get("SWAB_DEBUG"):
                    print(f"[swab] the draw ends: the head at {1000 * (c['head_lo'] - ZB):.1f} mm, last squeezed at {1000 * (self.draw_top - ZB):.1f} mm, "
                          f"{1000 * c['squeezed']:.1f} mm of it squeezed now", flush=True)
            elif c["off"] < R_IN and c["head_lo"] > ZB + PZ1 - 0.006:
                self.wrung = self.wrung or abs(self.draw_turn) >= DRAW_TURN  # out of the soft walls at the mouth
                self.drawing = False
            elif c["at_bottom"]:
                self.drawing = False  # back down: start again
        self.was_pinched = c["pinched"]
        self.prev_bottom = c["pinched"] and c["at_bottom"]
        final = c["in_bin"] and c["clear"]
        self.hold = self.hold + 1 if (final and self.wrung and not self.fell) else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        if self.fell:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        elif self.wrung:
            self.score = 0.8
        elif waited:
            self.score = 0.7
        elif self.turns >= 5.0:
            self.score = 0.6
        else:
            self.score = 0.12 * min(5, int(self.turns))


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.full


def score(objects) -> float:
    return TRIAL.score


def _report(ctx, env, label=""):
    c = now(env)
    print(f"[swab] t {ctx.t:5.1f} {label}in tube {c['in_tube']} off {1000 * c['off']:.1f} mm, head at {1000 * (c['head_lo'] - ZB):+.1f} mm, overlap "
          f"{1000 * c['overlap']:.1f} mm, squeezed on it {1000 * c['squeezed']:.1f} mm, segments in {''.join('x' if o else '.' for o in c['seg_on'])} "
          f"{np.round(np.array(c['travel']).min(1) * 1000, 1)} mm, pinched {c['pinched']}; turns {TRIAL.turns:.2f}, waited {TRIAL.waited / 20:.0f} s, wrung "
          f"{TRIAL.wrung} (drawing {TRIAL.drawing}, turned {TRIAL.draw_turn:+.2f} in the draw); in bin {c['in_bin']}, clear {c['clear']}; score {score(None):.2f}", flush=True)
    if os.environ.get("SWAB_DEBUG"):
        tp = m.fingertips(env) - np.array([0.0, 0.0, TOP])
        print(f"[swab]   left tips {np.round(tp[:12].min(0), 3)}..{np.round(tp[:12].max(0), 3)}; right tips {np.round(tp[12:].min(0), 3)}..{np.round(tp[12:].max(0), 3)}; "
              f"left cmd {np.round(ctx.left.at() - [0, 0, TOP], 3)}, right cmd {np.round(ctx.right.at() - [0, 0, TOP], 3)}", flush=True)


def demo_script(ctx):
    """Left hand: takes the swab by its handle from the box, swings it head down and lowers the head to the tube's bottom. Right hand: squeezes the
    tube's lower part onto the head. Left: lets go, takes the shaft again from above and turns it five times in 120-degree strokes (grip, turn, let go,
    turn back), then draws it up through the squeeze while turning, and drops it in the bin. Right: lets go and backs away.
    Failure checks (--mode): none does nothing; open turns the swab with the tube not squeezed; shallow does it all with the head 2 cm into the tube;
    resqueeze lets the squeeze go after the turns, lifts the swab near the rim and squeezes again for the draw; yank pulls the swab straight out after the turns,
    the squeeze let go first."""
    import demokit as d
    from yam_arena.constants import FINGER_OPEN
    from yam_arena.urdf import axis_angle_matrix

    env = ctx.emb._env
    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode not in ("full", "none", "open", "yank", "shallow", "resqueeze", "holdbin", "nowait", "fixed", "creep"):
        raise ValueError(mode)
    dbg = (lambda w: _report(ctx, env, w)) if os.environ.get("SWAB_DEBUG") else (lambda w: None)
    for _ in range(10):
        yield
    _report(ctx, env, f"{mode} start: ")
    if mode == "none":
        for _ in range(40):
            yield
        return

    def swab_at(z_from_tip):
        return m.state(env, "swab").to_world(np.array([[0.0, 0.0, z_from_tip]]))[0]

    tip_now = lambda: m.state(env, "swab").pos  # noqa: E731

    def joint_move(arm, pos, grip, t, finger):
        """Solve the IK for the goal afresh (from the ready pose and from here, the better), then move the joints straight to it."""
        from yam_arena import ik
        from yam_arena.constants import READY
        best = None
        for seed in (np.array(READY, float), arm.q.copy()):
            q, pe, re_ = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(grip.rot), seed, base_pos=arm.base, iters=200)
            if best is None or pe + 0.01 * re_ < best[1] + 0.01 * best[2]:
                best = (q, pe, re_)
        q1, pe, re_ = best
        arm._note(pe * 1000, math.degrees(re_), "joint move")
        q0 = arm.q.copy()
        n = max(1, round(t * 20))
        fr = float(np.clip(finger / FINGER_OPEN, 0.0, 1.0))
        for k in range(1, n + 1):
            u = k / n
            arm.q = q0 + (q1 - q0) * (u * u * (3 - 2 * u))
            arm.cmd = np.r_[arm.q, fr]
            yield
        arm.grip, arm.finger = grip, finger

    # 1. the swab by its handle, over the box's edge; lifted; swung head down
    g_dn = d.down(-90)
    pick = swab_at(GRIP_PICK)
    yield from l.move(pick + np.array([0.0, 0.0, 0.06]), grip=g_dn, finger=FINGER_OPEN * 0.5, t=2.5, label="over the swab's handle")
    yield from l.move(pick, t=1.0, label="fingers either side of the handle")
    yield from l.close(2 * SHAFT_R, 40.0, t=0.5, label="grip the handle")
    yield from l.move(l.at() + np.array([0.0, 0.0, 0.10]), t=1.0, label="lift the swab")
    g_lv = d._as_grip(axis_angle_matrix(np.array([1.0, 0.0, 0.0]), -math.pi / 2) @ g_dn.rot)  # the head swung down; the fingers point -y
    yield from l.move(np.array([0.25, TY + 0.03, TOP + 0.24]), grip=g_lv, t=2.0, label="swing the swab head down")
    dbg("swung: ")
    # 2. the head over the tube, then down to its bottom
    above = np.array([TX, TY, TOP + ZB + 0.002 + 0.09])
    yield from l.move(l.at() + (above - tip_now()), t=1.5, label="the swab over the tube")
    yield from l.servo(above, tol=0.0008, timeout=1.5, read=tip_now, label="the head over the tube's mouth")
    depth = 0.055 if mode == "shallow" else 0.003  # shallow: the head left 2 cm into the tube's top
    yield from l.servo(lambda tt: np.array([TX, TY, TOP + ZB + max(0.092 - 0.05 * tt, depth)]), tol=0.001, timeout=2.5, read=tip_now,
                       label="the head down to the tube's bottom")
    dbg("head in: ")
    # 3. the squeeze: right hand level from the front, jaws across the tube onto the pads
    zp = TOP + ZB + HOLD_DEPTH + 0.002 + 0.018  # the squeeze just over the holder: the fingers' 36 mm span covers the head's upper half at the bottom
    yield from r.move(np.array([TX - 0.08, TY, zp + 0.03]), grip=d.level(0), finger=FINGER_OPEN * 0.3, t=2.5, label="in front of the tube")
    yield from r.move(np.array([TX - 0.07, TY, zp]), t=0.8, label="level with the tube's soft part")
    yield from r.move(np.array([TX, TY, zp]), t=1.0, label="fingers either side of the tube")
    if mode != "open":
        yield from r.close(2 * (HEAD_R + PAD_T), 6.0, t=0.8, label="squeeze the tube onto the swab head")
    dbg("squeezed: ")
    # 4. the left lets go and takes the shaft again from above
    yield from l.open(t=0.4, label="let go of the swab")
    yield from l.move(l.at() + np.array([0.0, 0.06, 0.06]), t=0.8, label="the left hand back and up")
    zg = TOP + ZB + depth - 0.001 + GRIP_TURN
    yield from l.move(np.array([TX - 0.02, TY + 0.08, TOP + 0.26]), t=1.0, label="the left hand up, clear of the swab")
    # from the swung grip to a down grip the wrist has to flip, which no Cartesian path follows: move the joints straight there, high up
    yield from joint_move(l, np.array([TX - 0.02, TY + 0.04, TOP + 0.25]), d.down(-120), 2.5, FINGER_OPEN * 0.4)
    yield from l.move(np.array([TX, TY, zg + 0.09]), t=0.6, label="above the swab's shaft")
    yield from l.move(np.array([TX, TY, zg]), t=1.0, label="fingers either side of the shaft")
    dbg("over shaft: ")

    # 5. five turns in 120-degree strokes
    def shaft_at(z):
        """The swab's axis where it crosses height z (world)."""
        st = m.state(env, "swab")
        ax = st.rotation()[:, 2]
        return st.pos + ax * (z - st.pos[2]) / max(ax[2], 0.2)

    def stroke(z_from, z_to, label, n=20):
        """Centre the open fingers on the shaft where it stands, close, stand the swab upright over the tube's axis, then turn 120 degrees."""
        p = shaft_at(z_from)
        yield from l.servo(np.array([p[0], p[1], z_from]), tol=0.0006, timeout=0.6, label="the fingers centred on the shaft")
        yield from l.close(2 * SHAFT_R, 40.0, t=0.3, label="grip the shaft")
        yield from l.move(np.array([TX, TY, z_from]), t=0.2, label="the swab upright")
        a0 = -120.0
        for k in range(1, n + 1):
            f = k / n
            yield from l.move(np.array([TX, TY, z_from + (z_to - z_from) * f]), grip=d.down(a0 + 120.0 * f), t=0.05, label=label)
            if os.environ.get("SWAB_DEBUG") and z_to != z_from and k % 5 == 0:
                tl = m.fingertips(env)[:12] - np.array([0.0, 0.0, TOP])
                print(f"[swab]   {label} {k}: tip z {1000 * (tip_now()[2] - TOP - ZB):.1f} mm; left tcp seen {np.round(l.tcp() - [0, 0, TOP], 4)} cmd "
                      f"{np.round(l.at() - [0, 0, TOP], 4)} trim {np.round(l.trim * 1000, 1)}; tips z {tl[:, 2].min():.3f}..{tl[:, 2].max():.3f}, "
                      f"spread {1000 * np.ptp(tl[:, 0]):.0f} x {1000 * np.ptp(tl[:, 1]):.0f} mm", flush=True)

    for k in range(21):  # 21 strokes of 120 degrees: the swab follows about 0.25 turn of each, so about 5.3 turns
        yield from stroke(zg, zg, "turn the swab")
        if k % 3 == 2:
            dbg(f"turn {(k + 1) // 3}: ")
        yield from l.open(t=0.25, label="let go of the shaft")
        yield from l.move(np.array([TX, TY, zg]), grip=d.down(-120), t=0.6, label="the wrist back")
    _report(ctx, env, "turned: ")
    # 6. the insert's minute: the swab left in the tube, the squeeze kept on
    if mode != "nowait":
        for k in range(WAIT_STEPS + 10):
            yield
            if k % 400 == 399:
                _report(ctx, env, "waiting: ")
    _report(ctx, env, "waited: ")
    # 7. out through the squeeze, turning
    if mode == "yank":
        yield from r.open(t=0.4, label="let go of the tube")
        yield from l.close(2 * SHAFT_R, 40.0, t=0.3, label="grip the shaft")
        yield from l.move(np.array([TX, TY, zg + 0.09]), grip=d.down(-120), t=0.8, label="pull the swab straight out")
    elif mode == "resqueeze":  # the squeeze let go, the swab lifted near the rim, squeezed again and drawn the last few mm
        yield from r.open(t=0.4, label="let go of the tube")
        yield from l.close(2 * SHAFT_R, 40.0, t=0.3, label="grip the shaft")
        yield from l.move(np.array([TX, TY, zg + 0.045]), t=1.0, label="lift the swab near the rim")
        yield from r.close(2 * (HEAD_R + PAD_T), 6.0, t=0.8, label="squeeze the tube again")
        for k in range(1, 41):  # the draw from there, turning, without letting go of the swab
            yield from l.move(np.array([TX, TY, zg + 0.045 + 0.035 * k / 40]), grip=d.down(-120 + 120.0 * k / 40), t=0.05, label="draw the swab up, turning")
    elif mode in ("fixed", "shallow", "creep"):
        # one continuous draw, turning, the squeeze left where it was (fixed): the head rises out of the dent the fingers make; creep: the same, slowly
        # (0.27 mm a step)
        yield from stroke(zg, zg + (0.03 if mode == "shallow" else 0.08), "draw the swab up, turning", n=300 if mode == "creep" else 60)
        dbg("drawn: ")
    else:
        # the left hand takes the shaft once and keeps it for the whole draw (let go, the squeeze's upper edge on the head's taper pushes it back down);
        # it lifts 10 mm at a time, turning the swab in lifts 2 to 4. After each lift the right hand eases the squeeze and squeezes again on
        # the head as it is seen, the fingers' upper edge 3 mm under the head's top (so that they do not press on its taper), while the left holds
        p = shaft_at(zg)
        yield from l.servo(np.array([p[0], p[1], zg]), tol=0.0006, timeout=0.6, label="the fingers centred on the shaft")
        yield from l.close(2 * SHAFT_R, 40.0, t=0.3, label="grip the shaft")
        yield from l.move(np.array([TX, TY, zg]), t=0.2, label="the swab upright")
        a = -120.0
        for k in range(14):
            da = 40.0 if 1 <= k <= 3 else 0.0  # the first lift takes the head off the bottom; the next three turn it, -120 to 0 degrees
            z0 = l.at()[2]
            for j in range(1, 21):
                yield from l.move(np.array([TX, TY, z0 + DRAW_STEP * j / 20]), grip=d.down(a + da * j / 20), t=0.05, label="draw the swab up, turning")
            a += da
            dbg(f"draw step {k}: ")
            if tip_now()[2] - TOP - ZB > PZ1 - 0.006:
                break  # the head is out of the soft walls at the mouth
            # the squeeze holds the swab back as the hand lifts, and the arm gives: bring the left hand's command back to where the hand is, so that
            # the swab does not jump up when the squeeze eases
            yield from l.move(np.array([TX, TY, l.tcp()[2]]), t=0.2, label="the left hand's pull relieved")
            yield from r.move(r.at(), finger=FINGER_OPEN * 0.3, t=0.3, label="ease the squeeze")
            yield from r.move(np.array([TX, TY, max(zp, tip_now()[2] + HEAD_L - 0.003 - 0.018)]), t=0.4, label="the fingers up with the head")
            yield from r.close(2 * (HEAD_R + PAD_T), 6.0, t=0.4, label="squeeze the tube on the head again")
    _report(ctx, env, "wrung: ")
    # 8. the right lets go of the tube; the swab lifted clear of the rim (by what it needs: it may have slid in the fingers), into the bin
    if mode != "yank":
        yield from r.open(t=0.4, label="let go of the tube")
    yield from r.move(np.array([TX - 0.12, TY - 0.06, zp + 0.06]), t=1.2, label="the right hand clear")
    dz = (TOP + ZB + T_LEN + 0.03) - tip_now()[2]
    if dz > 0:
        up = min(l.at()[2] + dz, TOP + 0.26) - l.at()[2]
        yield from l.move(l.at() + np.array([0.0, 0.0, up]), t=max(0.6, up / 0.08), label="the swab up out of the tube")
    _report(ctx, env, "out: ")
    yield from l.move(np.array([BIN_C[0] + 0.01, BIN_C[1] - 0.01, l.at()[2]]), t=2.0, label="over the bin")
    yield from l.move(np.array([BIN_C[0] + 0.01, BIN_C[1] - 0.01, TOP + 0.20]), t=0.8, label="down over the bin")
    if mode == "holdbin":  # the swab stood in the bin, still held by its top
        yield from l.move(np.array([BIN_C[0] + 0.01, BIN_C[1] - 0.01, TOP + 0.004 + GRIP_TURN]), t=1.5, label="the swab down into the bin, held")
        for _ in range(60):
            yield
        _report(ctx, env, "held in the bin: ")
        return
    yield from l.open(t=0.4, label="drop the swab in the bin")
    yield from l.move(l.at() + np.array([0.08, -0.02, 0.04]), t=1.0, label="the left hand clear")
    for k in range(60):
        yield
        if k % 30 == 29:
            _report(ctx, env, "end: ")
