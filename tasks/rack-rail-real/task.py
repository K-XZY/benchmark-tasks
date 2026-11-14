"""Hang a server's tool-less sliding rail in a real 4-post rack: seat its back end's pegs in the marked U of the rear flange until its latch
clicks, while the other hand supports the rail's front; then draw the front member out along its slide to the front flange, match the same U,
and seat it until it clicks.

The rack's near posts stand at x 0.28 with their flanges 0.74 m apart along y (rear at +0.37, front at -0.37): square EIA-310 holes, 9.5 mm,
three to a U of 44.45 mm, the target U marked on both. The rail is a free mechanism: its outer member carries the back end piece; its front
member, with the front end piece, slides on it (the depth adjustment), collapsed at 0.63 m. Each end piece has two square pegs, at the bottom
and top holes of a U. The latch is coded: an end piece pushed home with its pegs through the holes must pass a 10 N detent, then is held
against its flange as the latch does. Mass 1.35 kg.

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
import mechkit as m  # noqa: E402

SCENE = "station"
INSTRUCTION = "Hang the rail on the marked U, back end first, then the front, then let go."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own 2 s hold: score() and success() agree
HOLD_STEPS = 40
TOP = d.TABLE_TOP
DT = d.DT
mm = 1e-3

LOOKS = {
    "rack": m.Look(m.rgb(0.10, 0.10, 0.11), metallic=0.5, roughness=0.5),  # black powder-coated steel
    "mark": m.Look(m.rgb(0.95, 0.75, 0.10), roughness=0.5),  # the U marked on the posts
    "zinc": m.Look(m.rgb(0.72, 0.73, 0.75), metallic=0.9, roughness=0.35),  # zinc-plated rail
    "endpiece": m.Look(m.rgb(0.20, 0.45, 0.80), roughness=0.4),  # the rail's blue end pieces
    "latch": m.Look(m.rgb(0.90, 0.45, 0.10), roughness=0.4),
}

# -- the rack: EIA-310 square holes on the near posts' flanges ----------------------------------------------------------------------
HOLE_X = 0.280  # the hole column's x on the near posts
Y_REAR, Y_FRONT = 0.370, -0.370  # the flanges' inner faces
FLANGE_T, FLANGE_W = 0.002, 0.030
HOLE = 9.5 * mm
U, HOLE_PITCH = 44.45 * mm, 15.875 * mm
U0 = 0.006  # the bottom U's lower edge above the table
N_U = 6
TARGET_U = 2  # the marked U (from the bottom, from 0)
FAR_X = 0.280 + 0.480  # the far posts: scenery


def u_holes(k: int):
    """The three hole centres' heights in U k (above the table)."""
    z0 = U0 + k * U
    return [z0 + 6.35 * mm, z0 + 6.35 * mm + HOLE_PITCH, z0 + 6.35 * mm + 2 * HOLE_PITCH]


# -- the rail (a server maker's 1U tool-less sliding rail) -----------------------------------------------------------------------------------
RAIL_L = 0.630  # collapsed, end piece face to end piece face
RAIL_W, RAIL_H = 0.018, 0.040  # the channel across (x) and its height (z)
RAIL_KG, DRAW_KG = 1.00, 0.35  # the outer member with the back end piece; the front member
PEG, PEG_TIP, PEG_L, PEG_TIP_L = 7.5 * mm, 4.5 * mm, 5.0 * mm, 4.0 * mm  # square pegs tapered from 7.5 to 4.5 mm over 9 mm, in 9.5 mm holes
PEG_DZ = (6.35 * mm, 6.35 * mm + 2 * HOLE_PITCH)  # the pegs at the bottom and top holes of a U, from the U's lower edge
END_H = U  # the end piece's height
DRAW_FRICTION = 3.0  # N: the slide's friction (to measure)
LATCH_N = 10.0  # N: the latch's detent as an end piece is pushed home (approximated, to be measured)
LATCH_TRAVEL = 3.0 * mm
RAIL_START = (0.18, 0.0)  # the rail's middle on the bench
CLEAR = 0.03


def _rack() -> m.Part:
    parts, cols = [], []

    def box(name, size, c, look="rack", col=True):
        parts.append((name, m.box(size, c), look))
        if col:
            cols.append(m.col_box(f"c_{name}", size, c))

    z_top = U0 + N_U * U + 0.02
    for side, yf, sgn in (("rear", Y_REAR, 1), ("front", Y_FRONT, -1)):
        yc = yf + sgn * FLANGE_T / 2  # the flange plate, just outside the inner face
        x0, x1 = HOLE_X - FLANGE_W / 2, HOLE_X + FLANGE_W / 2
        hx0, hx1 = HOLE_X - HOLE / 2, HOLE_X + HOLE / 2
        box(f"{side}_strip_a", (hx0 - x0, FLANGE_T, z_top), ((x0 + hx0) / 2, yc, z_top / 2))
        box(f"{side}_strip_b", (x1 - hx1, FLANGE_T, z_top), ((x1 + hx1) / 2, yc, z_top / 2))
        zs = [0.0] + [z for k in range(N_U) for z in u_holes(k)] + [z_top]
        for j in range(len(zs) - 1):  # the bars between the square holes in the hole column
            lo = zs[j] + (HOLE / 2 if j > 0 else 0.0)
            hi = zs[j + 1] - (HOLE / 2 if j + 1 < len(zs) - 1 else 0.0)
            if hi > lo:
                box(f"{side}_bar_{j}", (HOLE, FLANGE_T, hi - lo), (HOLE_X, yc, (lo + hi) / 2))
        # the post's side, behind the flange (an L section), and its U marks
        box(f"{side}_side", (FLANGE_T, 0.030, z_top), (x0 - FLANGE_T / 2, yf + sgn * 0.015, z_top / 2))
        zu = U0 + TARGET_U * U
        for k, (zz, h) in enumerate(((zu, 0.0015), (zu + U, 0.0015))):  # lines at the marked U's edges, on both strips
            for s_, xx in (("a", (x0 + hx0) / 2), ("b", (x1 + hx1) / 2)):
                box(f"{side}_mark_{k}{s_}", (hx0 - x0 + 0.0004, 0.0006, h), (xx, yf - sgn * 0.0003, zz), "mark", col=False)
        box(f"{side}_tab", (0.006, 0.0006, U - 0.004), (x0 + 0.003, yf - sgn * 0.0003, zu + U / 2), "mark", col=False)
        # the far post opposite, drawn only
        box(f"{side}_far", (0.030, 0.030, z_top), (FAR_X, yf + sgn * 0.015, z_top / 2), col=False)
        box(f"{side}_bottom_rail", (FAR_X - HOLE_X + 0.03, 0.030, 0.015), ((HOLE_X + FAR_X) / 2, yf + sgn * 0.015, 0.0075), col=False)
        box(f"{side}_top_rail", (FAR_X - HOLE_X + 0.03, 0.030, 0.015), ((HOLE_X + FAR_X) / 2, yf + sgn * 0.015, z_top - 0.0075), col=False)
    for x_, nm in ((HOLE_X - 0.005, "near"), (FAR_X, "far")):  # the frame's depth rails at the bottom, drawn only
        box(f"depth_{nm}", (0.030, Y_REAR - Y_FRONT + 0.06, 0.012), (x_, 0.0, 0.006), col=False)
    return m.fixed("rack", (0.0, 0.0, TOP), parts, cols, LOOKS, friction=(0.5, 0.4))


def _pegs(prefix, y_face, sgn, z_u0):
    """Two square pegs on an end piece's face at y_face, pointing along sgn*y, tapered from PEG at the face to PEG_TIP at the tip over
    their whole length, so that a push guides them into the holes."""
    meshes, cols = [], []
    length = PEG_L + PEG_TIP_L
    for k, dz in enumerate(PEG_DZ):
        z = z_u0 + dz
        frustum = m.lathe([(0.0, 0.0), (PEG / math.sqrt(2), 0.0), (PEG_TIP / math.sqrt(2), length), (0.0, length)], segments=4).turned(45.0, "z")
        peg = frustum.aimed("+Y" if sgn > 0 else "-Y").moved((0.0, y_face, z))
        meshes.append((f"{prefix}_peg{k}", peg, "zinc"))
        cols.append(m.col_hull(f"c_{prefix}_peg{k}", peg))
    return meshes, cols


def _rail() -> m.Part:
    """In the rail's frame: origin at the outer member's middle, its length along y, the back end at +y. The channel's middle height is
    at z 0; the U the pegs fit runs from z -U/2 to +U/2 (the pegs at its bottom and top holes)."""
    half = RAIL_L / 2
    z_u0 = -U / 2
    frame = [("outer", m.box((RAIL_W, RAIL_L - 0.08, RAIL_H), (0.0, 0.04, 0.0)), "zinc"),
             ("back_end", m.box((0.030, 0.008, END_H), (0.0, half - 0.004, 0.0)), "endpiece"),
             ("back_latch", m.box((0.010, 0.012, 0.010), (-0.012, half - 0.016, 0.0)), "latch")]
    cols = [m.col_box("c_outer", (RAIL_W, RAIL_L - 0.08, RAIL_H), (0.0, 0.04, 0.0)), m.col_box("c_back_end", (0.030, 0.008, END_H), (0.0, half - 0.004, 0.0))]
    pm, pc = _pegs("back", half, 1, z_u0)
    frame += pm
    cols += pc
    # the front member: a thinner channel inside the outer one's front half, with the front end piece
    dm, dc = _pegs("front", -half, -1, z_u0)
    draw_parts = [("inner", m.box((RAIL_W - 0.004, 0.30, RAIL_H - 0.008), (0.0, -half + 0.15, 0.0)), "zinc"),  # its front 8 cm out of the outer member
                  ("front_end", m.box((0.030, 0.008, END_H), (0.0, -half + 0.004, 0.0)), "endpiece"),
                  ("front_latch", m.box((0.010, 0.012, 0.010), (-0.012, -half + 0.016, 0.0)), "latch")] + dm
    draw_cols = [m.col_box("c_front_end", (0.030, 0.008, END_H), (0.0, -half + 0.004, 0.0)),
                 m.col_box("c_inner", (RAIL_W - 0.004, 0.072, RAIL_H - 0.008), (0.0, -half + 0.044, 0.0))] + dc
    draw = m.slider("draw", "Y", -0.25, 0.0, draw_parts, draw_cols, mass=DRAW_KG, spring=0.0, damping=5.0, friction=DRAW_FRICTION, effort=200.0,
                    inertia=(DRAW_KG * 0.3 ** 2 / 12, 1e-4, DRAW_KG * 0.3 ** 2 / 12))
    return m.mechanism("rail", (RAIL_START[0], RAIL_START[1], TOP + END_H / 2 + 0.0005), [draw], LOOKS, frame, cols, friction=(0.6, 0.5),
                       free_mass=RAIL_KG, frame_inertia=(RAIL_KG * RAIL_L ** 2 / 12, RAIL_KG * 0.02 ** 2 / 6, RAIL_KG * RAIL_L ** 2 / 12))


OBJECTS = [_rack(), _rail()]


# -- the latches and the scorer ----------------------------------------------------------------------------------------------------


def judge(dropped, back_ok, front_ok, front_latched, level, clear, held):
    """(full now, score now). `back_ok`: the back end latched at the marked U; `front_ok`: the front too; `front_latched`: the front latched
    at any U; `level`: within 0.5 degrees."""
    if dropped:
        return False, 0.0
    full = bool(back_ok and front_ok and level and clear)
    if full and held >= HOLD_STEPS:
        return True, 1.0
    if back_ok:
        return full, 0.25
    return full, 0.0


class Hang:
    def reset(self, env=None) -> None:
        self.steps, self.held = 0, 0
        self.latched = {"back": None, "front": None}  # the U each end piece latched at, or None
        self.pushing = {"back": False, "front": False}
        self.seat = {"back": None, "front": None}  # where each end piece's face latched, held there by the latch
        self.prev = {"back": None, "front": None}
        self.dropped, self.full, self.score = False, False, 0.0
        self.last = ""

    def _end(self, rail, which):
        """The end piece's face middle and its pegs' heights, in the station frame, and how far its face is from its flange."""
        half = RAIL_L / 2
        draw = rail.joint("draw")
        y_end = half if which == "back" else -half + draw
        face = rail.to_world(np.array([0.0, y_end, 0.0]))[0]
        gap = Y_REAR - face[1] if which == "back" else face[1] - Y_FRONT
        pegs_z = [float(rail.to_world(np.array([0.0, y_end, -U / 2 + dz]))[0][2] - TOP) for dz in PEG_DZ]  # at the end piece itself
        return face, gap, pegs_z

    def _u_of(self, face, pegs_z):
        """The U whose bottom and top holes the pegs are in, or None: the pegs' x within 1 mm of the hole column, their heights within 1 mm."""
        if abs(face[0] - HOLE_X) > 1.0 * mm:
            return None
        for k in range(N_U):
            h = u_holes(k)
            if abs(pegs_z[0] - h[0]) < 1.0 * mm and abs(pegs_z[1] - h[2]) < 1.0 * mm:
                return k
        return None

    def step(self, env) -> None:
        self.steps += 1
        rail = m.state(env, "rail")
        for which, sgn in (("back", 1.0), ("front", -1.0)):
            face, gap, pegs_z = self._end(rail, which)
            body = None if which == "back" else "draw"
            k = self._u_of(face, pegs_z)
            if self.latched[which] is not None:  # the latch's hook: the end piece held where it seated, a stiff spring and damper
                v = np.zeros(3) if self.prev[which] is None else (face - self.prev[which]) / DT
                f = 3000.0 * (self.seat[which] - face) - 30.0 * v
                n = float(np.linalg.norm(f))
                if n > 80.0:
                    f *= 80.0 / n
                m.push(env, "rail", tuple(float(c) for c in f), at=face, body=body)
            self.prev[which] = face
            if self.latched[which] is None:
                if k is not None and gap < LATCH_TRAVEL and gap > 0.5 * mm:
                    if not self.pushing[which]:  # the latch's detent pushes back until the end piece is home
                        m.push(env, "rail", (0.0, -sgn * LATCH_N, 0.0), at=face, body=body)
                        self.pushing[which] = True
                elif k is not None and gap <= 0.5 * mm:
                    self.latched[which] = k  # home: the latch clicks and holds it against its flange
                    self.seat[which] = face.copy()
                    self.pushing[which] = True
                    print(f"[rail] {which} end latched at U {k} (marked U {TARGET_U}), t {self.steps * DT:.1f} s", flush=True)
                elif self.pushing[which]:
                    m.push(env, "rail", (0.0, 0.0, 0.0), body=body)
                    self.pushing[which] = False
        if rail.pos[2] < TOP + 0.005 and self.steps > 20 and (self.latched["back"] is not None or self.latched["front"] is not None):
            self.dropped = True
        ax = rail.rotation()[:, 1]
        slope = math.degrees(math.asin(max(-1.0, min(1.0, float(ax[2])))))
        level = abs(slope) <= 0.5
        tips = m.fingertips(env)
        clear = bool(np.all(np.hypot(tips[:, 0] - rail.pos[0], tips[:, 2] - rail.pos[2]) >= CLEAR))  # every fingertip 3 cm or more from the rail's line
        back_ok = self.latched["back"] == TARGET_U
        front_ok = self.latched["front"] == TARGET_U
        self.full, self.score = judge(self.dropped, back_ok, front_ok, self.latched["front"] is not None, level, clear, self.held + 1)
        self.held = self.held + 1 if self.full else 0
        fb, gb, pb = self._end(rail, "back")
        ff_, gf, pf = self._end(rail, "front")
        roll = math.degrees(math.asin(max(-1.0, min(1.0, float(rail.rotation()[0, 2])))))
        self.last = (f"roll {roll:+.2f} deg; back gap {gb * 1000:.1f} mm x {(fb[0] - HOLE_X) * 1000:+.1f} pegs {pb[0] * 1000:.1f}/{pb[1] * 1000:.1f}, front gap {gf * 1000:.1f} mm x {(ff_[0] - HOLE_X) * 1000:+.1f} pegs {pf[0] * 1000:.1f}/{pf[1] * 1000:.1f} "
                     f"(marked holes {u_holes(TARGET_U)[0] * 1000:.1f}/{u_holes(TARGET_U)[2] * 1000:.1f}); draw {rail.joint('draw') * 1000:.0f} mm; slope {slope:.2f} deg; "
                     f"latched {self.latched}; clear {clear}; score {self.score}")
        if self.held == HOLD_STEPS:
            print(f"[rail] both ends latched at the marked U, level, held 2 s: score 1.0; {self.last}", flush=True)
        if os.environ.get("RAIL_DEBUG") and self.steps % 20 == 0:
            print(f"[rail] t {self.steps * DT:.1f} s: rail at {np.round((rail.pos - [0, 0, TOP]) * 1000, 1).tolist()} mm; {self.last}", flush=True)


HANG = Hang()
HANG.reset()
m.on_step(HANG.step)
m.on_reset(HANG.reset)


def success(objects) -> bool:
    return HANG.full and HANG.held >= HOLD_STEPS


def score(objects) -> float:
    return HANG.score


# -- the demo -------------------------------------------------------------------------------------------------------------------


def demo_script(ctx):
    """Modes: full, the solution. Cheat checks: wrongu (both ends seated one U above the marked one), frontoff (the back at the marked U, the
    front one U above: a sloping rail), partial (the back pegs pushed only part way in, then held), tilted (the rail carried 3.4 degrees out of
    level and pushed home without levelling), onearm (the left hand alone carries the rail by its middle and pushes its back end home).
    none: the arms wait."""
    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode == "none":
        yield from l.wait(3.0)
        return
    if mode not in ("full", "wrongu", "frontoff", "partial", "tilted", "onearm"):
        raise ValueError(f"unknown mode {mode!r}")
    half = RAIL_L / 2

    def z_of(k):
        return TOP + 0.5 * (u_holes(k)[0] + u_holes(k)[2])  # the rail's middle line at U k

    z_mid = z_of(TARGET_U + 1 if mode == "wrongu" else TARGET_U)
    z_front = z_of(TARGET_U + 1 if mode in ("wrongu", "frontoff") else TARGET_U)
    levelled = mode not in ("tilted",)

    def rail():
        return ctx.state("rail")

    def back_face():
        return rail().to_world(np.array([0.0, half, 0.0]))[0]

    def front_face():
        rs = rail()
        return rs.to_world(np.array([0.0, -half + rs.joint("draw"), 0.0]))[0]

    top_off = 0.005  # the TCP 5 mm above the outer channel's middle: the fingers' pads down most of its height, so the rail cannot roll in them
    in_off = 0.004  # and on the front member's channel

    def both(err, t, label, level=False):
        """Both hands moved by `err`; with `level`, the front hand also raised or lowered to bring the rail level, as seen."""
        fix = np.zeros(3)
        if level and levelled:
            ax = rail().rotation()[:, 1]
            sep = float(np.linalg.norm(l.tcp()[:2] - r.tcp()[:2]))
            fix = np.array([float(ax[0]), 0.0, float(ax[2])]) * sep  # the back end high or off to one side: the front hand brings the rail square
        yield from ctx.together(l.move(l.at() + err, t=t, label=label), r.move(r.at() + err + fix, t=t, label=label))

    if mode == "onearm":  # the left hand alone, at the rail's middle; it cannot hold the rail from pitching in its fingers
        mid = rail().to_world(np.array([0.0, -0.013, top_off]))[0]
        yield from l.move(mid + [0.0, 0.0, 0.05], grip=d.down(-90.0), finger=-0.02, t=2.0, label="over the rail's middle")
        yield from l.move(mid, t=1.0, label="round the channel")
        yield from l.close(RAIL_W, force=45.0, label="take the rail")
        yield from l.move(mid + [0.0, 0.0, 0.14], t=1.2, label="lift the rail")
        for _ in range(6):
            err = np.array([HOLE_X, Y_REAR - 0.012, z_mid]) - back_face()
            yield from l.move(l.at() + err, t=0.6, label="the back end at the marked U")
        for _ in range(12):
            if HANG.latched["back"] is not None:
                break
            bf = back_face()
            yield from l.move(l.at() + np.array([HOLE_X, min(bf[1] + 0.003, Y_REAR + 2.5 * mm), z_mid]) - bf, t=0.4, label="push the back end home")
        ax = rail().rotation()[:, 1]
        print(f"[rail] onearm: pitch {math.degrees(math.asin(float(ax[2]))):.1f} deg; {HANG.last}", flush=True)
        yield from l.wait(2.0)
        return
    # 1. Both hands take the rail on the bench: the left on the outer member near its back end, the right on the front member's channel.
    back_g = rail().to_world(np.array([0.0, half - 0.11, top_off]))[0]
    front_g = rail().to_world(np.array([0.0, -half + 0.04, in_off]))[0]
    yield from ctx.together(l.move(back_g + [0.0, 0.0, 0.05], grip=d.down(-90.0), finger=-0.02, t=2.0, label="over the rail's back half"),
                            r.move(front_g + [0.0, 0.0, 0.05], grip=d.down(90.0), finger=-0.02, t=2.0, label="over the rail's front end"))
    yield from ctx.together(l.move(back_g, t=1.0, label="round the channel"), r.move(front_g, t=1.0, label="round the front member"))
    yield from ctx.together(l.close(RAIL_W, force=45.0, label="take the rail"), r.close(RAIL_W - 0.004, force=40.0, label="take its front"))
    yield from both(np.array([0.0, 0.0, 0.14]), 1.2, "lift the rail")
    if mode == "tilted":  # the front hand 25 mm low: the rail 3.4 degrees out of level
        yield from r.move(r.at() + [0.0, 0.0, -0.025], t=0.8, label="the front end low")
    # 2. In between the posts, level, at the marked U, the back pegs 6 mm short of the rear flange's holes.
    y_c = Y_REAR - half - 0.015
    yield from both(np.array([HOLE_X, y_c - 0.010, z_mid + 0.02]) - rail().pos, 2.5, "the rail in between the posts")
    for _ in range(6):  # lined up to the holes, as seen, and levelled: the pegs fit 9.5 mm holes with 0.75 mm a side
        yield from both(np.array([HOLE_X, Y_REAR - 0.012, z_mid]) - back_face(), 0.5, "the back end at the marked U", level=True)
    # 3. Pushed along +y: the back pegs into the marked U's holes, lined up at each step, and home past the latch's click.
    last_gap, wiggle = None, 0
    for k in range(20):
        if HANG.latched["back"] is not None:
            break
        bf = back_face()
        gap = Y_REAR - bf[1]
        if last_gap is not None and last_gap - gap < 0.3 * mm:  # stalled: a hand's wiggle across the hole, as one feels a peg in
            wiggle += 1
        last_gap = gap
        dx = (0.6 * mm if wiggle % 2 else -0.6 * mm) if wiggle and mode != "partial" else 0.0
        goal = np.array([HOLE_X + dx, min(bf[1] + 0.003, Y_REAR - 4.0 * mm if mode == "partial" else Y_REAR + 2.5 * mm), z_mid])
        if mode == "partial" and gap < 4.5 * mm:  # the pegs part way in: stop pushing there
            break
        yield from both(goal - bf, 0.4, "push the back end home", level=True)
    print(f"[rail] back: {HANG.last}", flush=True)
    if mode in ("partial", "tilted") and HANG.latched["back"] is None:  # not latched: the hands keep it there; letting go would drop it
        yield from l.wait(3.0)
        print(f"[rail] {mode}: {HANG.last}", flush=True)
        return
    yield from l.open(t=0.4, label="let go of the rail's middle")
    yield from l.move(l.at() + [-0.04, -0.04, 0.05], t=1.0, label="left hand clear")
    # 4. The right hand draws the front member out to the front flange, at the same U, and pushes it home past the latch.
    for k in range(10):
        if HANG.latched["front"] is not None:
            break
        ff = front_face()
        goal = np.array([HOLE_X, Y_FRONT - 0.5 * mm if k >= 4 else max(ff[1] - 0.04, Y_FRONT + 0.012), z_front])
        yield from r.move(r.at() + (goal - ff) + [0.0, -0.002 if k >= 4 else 0.0, 0.0], t=0.8 if k < 4 else 0.6, label="draw the front out, then home")
    print(f"[rail] front: {HANG.last}", flush=True)
    yield from r.open(t=0.4, label="let go of the front")
    yield from r.move(r.at() + [-0.05, 0.0, 0.10], t=1.0, label="right hand clear")
    yield from l.wait(2.5)
