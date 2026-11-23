"""Fold a sedan's 40% rear seatback down from the trunk: one hand holds the spring-return lock knob pulled, the other tips the seatback past vertical.

Scene (station frame, metres; x forward from the midpoint between the arm bases, y to the robot's left, z above the table top). The robot stands at the
trunk opening. The seatback (40% section, 0.55 m wide, 0.60 m tall, 0.10 m thick, 9 kg) is hinged at its foot at x HINGE_X, z HINGE_Z and starts reclined
25 deg toward the trunk, held by a striker latch at its top outer corner. The lock knob sits on the trunk-side trim beside the seatback top; pulling it
20 mm against its return spring frees the latch. Released, the latch re-catches the seatback unless it has swung LATCH_CLEAR_DEG off the striker.
Pushed past vertical, gravity carries the seatback forward onto the rear cushion (its folded stop).

Debug: python demokit.py task.py OUT --no-record --mode full,none,pushonly,pullonly,letgo,shove
Range check: FRSF_KG, FRSF_SPRING, FRSF_DAMP, FRSF_FRIC, FRSF_CLEAR override the estimates (README, Fidelity).
Final: python demokit.py task.py OUT --mode full
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
INSTRUCTION = "Pull and hold the seatback lock knob in the trunk, push the rear seatback forward past upright, and let it fold down onto the seat cushion."
SUCCESS_HOLD_S = 0.0
SCORE_OVER = "end"
TOP = d.TABLE_TOP

# sizes and physics (README size and fidelity tables)
HINGE_X, HINGE_Z = 0.48, 0.06
RECLINE = float(os.environ.get("FRSF_RECLINE", "25.0"))  # deg from vertical at the start (latched); 23-28 checked
BACK_H, BACK_T = 0.60, 0.10
BACK_Y0, BACK_Y1 = -0.38, 0.17   # 0.55 m wide
BACK_KG = float(os.environ.get("FRSF_KG", "9.0"))   # 8-11 kg bound; the env override is for the range check only
FOLD_STOP = 113.0       # deg of swing from the latched pose to lying on the cushion
HINGE_DAMP = float(os.environ.get("FRSF_DAMP", "2.0"))       # N m s/rad, estimate (0-4 checked)
HINGE_FRICTION = float(os.environ.get("FRSF_FRIC", "0.5"))   # N m, estimate (0-1.5 checked)
KNOB_X, KNOB_Y, KNOB_Z = 0.200, 0.270, 0.52   # knob face centre
KNOB_R = 0.0125          # half the pull tab's height (25 mm tall)
KNOB_TAB_T = 0.012       # pull tab thickness across the pull
KNOB_L = float(os.environ.get("FRSF_HEAD", "0.015"))     # knob head length: low end of 15-20 mm
KNOB_STEM = float(os.environ.get("FRSF_STEM", "0.010"))  # stem between head and trim: low end of 10-20 mm (the knob stands 25-40 mm out of the trim)
KNOB_TRAVEL = 0.020
KNOB_SPRING = float(os.environ.get("FRSF_SPRING", "350.0"))  # N/m: about 7 N at full pull (5-10 N checked)
RELEASE_M = float(os.environ.get("FRSF_RELEASE", "0.015"))  # knob pulled this far frees the latch; 10-19 mm checked
LATCH_CLEAR_DEG = float(os.environ.get("FRSF_CLEAR", "5.0"))  # swing that takes the latch off its striker (3-7 deg checked)
PAST_VERTICAL = RECLINE
FOLDED_TOL = 10.0
LOCK_K = 50000.0

LOOKS = {
    "fabric": m.Look(m.rgb(0.31, 0.30, 0.29), metallic=0.0, roughness=1.0),
    "seam": m.Look(m.rgb(0.12, 0.12, 0.12), metallic=0.0, roughness=1.0),
    "post": m.Look(m.rgb(0.70, 0.70, 0.72), metallic=0.8, roughness=0.3),
    "buckle": m.Look(m.rgb(0.05, 0.05, 0.05), metallic=0.2, roughness=0.5),
    "carpet": m.Look(m.rgb(0.30, 0.30, 0.32), metallic=0.0, roughness=1.0),
    "trim": m.Look(m.rgb(0.24, 0.24, 0.26), metallic=0.0, roughness=0.7),
    "knob": m.Look(m.rgb(0.55, 0.55, 0.57), metallic=0.1, roughness=0.5),
    "head": m.Look(m.rgb(0.22, 0.22, 0.23), metallic=0.0, roughness=1.0),
    "backcarpet": m.Look(m.rgb(0.15, 0.15, 0.16), metallic=0.0, roughness=1.0),
    "rib": m.Look(m.rgb(0.20, 0.20, 0.21), metallic=0.0, roughness=1.0),
    "belt": m.Look(m.rgb(0.08, 0.08, 0.09), metallic=0.0, roughness=0.8),
    "plastic": m.Look(m.rgb(0.11, 0.11, 0.12), metallic=0.0, roughness=0.6),
    "pulltab": m.Look(m.rgb(0.50, 0.30, 0.16), metallic=0.0, roughness=0.6),
    "letter": m.Look(m.rgb(0.92, 0.90, 0.85), metallic=0.0, roughness=0.6),
}


def _rrect(a0, a1, b0, b1, r, n=6):
    """A counter-clockwise rounded rectangle outline in a 2D plane, corner radius r."""
    pts = []
    for ca, cb, t0 in ((a1 - r, b0 + r, -90.0), (a1 - r, b1 - r, 0.0), (a0 + r, b1 - r, 90.0), (a0 + r, b0 + r, 180.0)):
        for k in range(n + 1):
            t = math.radians(t0 + 90.0 * k / n)
            pts.append((ca + r * math.cos(t), cb + r * math.sin(t)))
    return pts


def _slab_x(y0, y1, z0, z1, r, x0, x1) -> m.Mesh:
    """A solid with a rounded (y, z) section, running along x from x0 to x1."""
    pr = m.prism(_rrect(y0, y1, z0, z1, r), x0, x1)
    return m.Mesh(pr.v[:, [2, 0, 1]].copy(), pr.f)


def _bx(x0, x1, y0, y1, z0, z1) -> m.Mesh:
    return m.box((x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def _recline(mesh: m.Mesh, deg: float = RECLINE) -> m.Mesh:
    """An upright mesh (authored about the hinge at x 0, z 0) tipped `deg` back toward the trunk, then placed at the hinge."""
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    v = mesh.v.copy()
    x, z = v[:, 0].copy(), v[:, 2].copy()
    v[:, 0] = x * c - z * s + HINGE_X
    v[:, 2] = x * s + z * c + HINGE_Z
    return m.Mesh(v, mesh.f)


def _back_mesh() -> m.Mesh:
    return _recline(_bx(-BACK_T / 2, BACK_T / 2, BACK_Y0, BACK_Y1, 0.0, BACK_H))


def _back_look() -> m.Mesh:
    """The seatback's upholstered body, visual: inside the collider box, its vertical edges rounded and its top a half-round
    roll (the hand pushes the flat trunk face 8 cm below the top, where the visual is the collider face)."""
    t = BACK_T / 2
    body = m.prism(_rrect(-t, t, BACK_Y0, BACK_Y1, 0.014), 0.0, BACK_H - t)
    roll = m.cylinder(t, BACK_Y1 - BACK_Y0 - 0.004, (0.0, (BACK_Y0 + BACK_Y1) / 2, BACK_H - t), axis="y")
    return _recline(m.join(body, roll))


def _trunk_face_looks() -> list:
    """The trunk face as on the car: a dark carpet back panel with a ribbed nap inside a stitched outline, a seat-belt
    guide at the outer top corner. All at most 1.6 mm proud of the collider's trunk face (where the push hand lands)."""
    t = BACK_T / 2
    ya, yb, za, zb = BACK_Y0 + 0.035, BACK_Y1 - 0.035, 0.04, BACK_H - t
    parts = [("carpet_panel", _recline(_bx(-t - 0.0010, -t + 0.002, ya, yb, za, zb)), "backcarpet")]
    ribs = [_bx(-t - 0.0016, -t, y, y + 0.004, za + 0.01, zb - 0.01) for y in np.arange(ya + 0.01, yb - 0.01, 0.014)]
    parts.append(("carpet_nap", _recline(m.join(*ribs)), "rib"))
    piping = [_bx(-t - 0.0016, -t, ya - 0.008, ya - 0.004, za - 0.006, zb + 0.004), _bx(-t - 0.0016, -t, yb + 0.004, yb + 0.008, za - 0.006, zb + 0.004),
              _bx(-t - 0.0016, -t, ya - 0.008, yb + 0.008, za - 0.008, za - 0.004)]
    parts.append(("piping", _recline(m.join(*piping)), "seam"))
    guide = _bx(-0.022, 0.03, BACK_Y1 - 0.05, BACK_Y1 - 0.012, BACK_H - 0.012, BACK_H + 0.014)
    strap = m.join(_bx(-0.004, 0.052, BACK_Y1 - 0.046, BACK_Y1 - 0.016, BACK_H + 0.014, BACK_H + 0.018),
                   _bx(t + 0.001, t + 0.004, BACK_Y1 - 0.046, BACK_Y1 - 0.016, BACK_H - 0.30, BACK_H + 0.016))
    parts += [("belt_guide", _recline(guide), "plastic"), ("belt", _recline(strap), "belt")]
    return parts


def _back_looks() -> list:
    """Visual dressing on the seatback's cabin face (never touched; the trunk face the hand pushes stays the flat collider face):
    side bolsters, stitched seams, the headrest posts. Within the collider's outline at most 3 mm proud of the cabin face."""
    t = BACK_T / 2
    parts = [("bolster_o", _recline(_bx(t, t + 0.002, BACK_Y0, BACK_Y0 + 0.07, 0.04, BACK_H - 0.02)), "head"),
             ("bolster_i", _recline(_bx(t, t + 0.002, BACK_Y1 - 0.07, BACK_Y1, 0.04, BACK_H - 0.02)), "head")]
    for i, z in enumerate((0.18, 0.36)):
        parts.append((f"seam_h{i}", _recline(_bx(t, t + 0.003, BACK_Y0 + 0.07, BACK_Y1 - 0.07, z, z + 0.004)), "seam"))
    for i, y in enumerate((BACK_Y0 + 0.07, BACK_Y1 - 0.074)):
        parts.append((f"seam_v{i}", _recline(_bx(t, t + 0.003, y, y + 0.004, 0.04, BACK_H - 0.02)), "seam"))
    for i, y in enumerate((-0.17, -0.03)):
        parts.append((f"post{i}", _recline(m.cylinder(0.006, 0.05, (0.0, y, BACK_H + 0.015), axis="z")), "post"))
    return parts + _trunk_face_looks()


def _headrest_look() -> m.Mesh:
    # the headrest pad on its two posts, the posts showing above the seatback top; rounded, inside the headrest collider
    pad = m.prism(_rrect(-0.045, 0.045, -0.22, 0.02, 0.02), BACK_H + 0.035, BACK_H + 0.085)
    top = m.cylinder(0.045, 0.236, (0.0, -0.10, BACK_H + 0.085), axis="y")
    return _recline(m.join(pad, top))


def _buckle() -> m.Mesh:
    # rear belt buckle on its strap, standing at the cushion's inboard edge, clear of the seatback's swing
    return m.join(_bx(HINGE_X + 0.07, HINGE_X + 0.08, BACK_Y0 - 0.045, BACK_Y0 - 0.02, 0.012, 0.06),
                  _bx(HINGE_X + 0.065, HINGE_X + 0.085, BACK_Y0 - 0.048, BACK_Y0 - 0.017, 0.06, 0.12))


def _cushion_looks() -> list:
    """Visual: the 40% cushion top with its seams, and the neighbouring (60%) cushion beside it, a padded bench with a
    rounded edge, so the folded seatback reads as lying on the rear bench. Nothing here is touched."""
    c40 = _slab_x(BACK_Y0, BACK_Y1, 0.0, 0.012, 0.005, HINGE_X + 0.06, HINGE_X + 0.60)
    seams = m.join(*[_bx(HINGE_X + 0.06, HINGE_X + 0.60, y, y + 0.004, 0.012, 0.0135) for y in (BACK_Y0 + 0.07, BACK_Y1 - 0.074)])
    c60 = _slab_x(BACK_Y0 - 0.30, BACK_Y0 - 0.055, 0.0, 0.11, 0.035, HINGE_X + 0.08, HINGE_X + 0.60)
    seams60 = m.join(*[_bx(HINGE_X + 0.08, HINGE_X + 0.60, y, y + 0.004, 0.11, 0.1115) for y in (BACK_Y0 - 0.23, BACK_Y0 - 0.13)])
    return [("cushion", c40, "fabric"), ("cushion_seams", seams, "seam"), ("cushion60", c60, "fabric"), ("cushion60_seams", seams60, "seam")]


def _headrest() -> m.Mesh:
    # outboard headrest in its lowest position, resting on the seatback top
    return _recline(_bx(-0.05, 0.05, -0.22, 0.02, BACK_H, BACK_H + 0.13))


def _cushion() -> m.Mesh:
    return _bx(HINGE_X + 0.06, HINGE_X + 0.60, BACK_Y0, BACK_Y1, 0.0, 0.012)


def _trim() -> m.Mesh:
    # trunk-side trim beside the seatback top (cropped), carrying the lock knob on its trunk face
    return _bx(KNOB_X + KNOB_L + KNOB_STEM, KNOB_X + KNOB_L + KNOB_STEM + 0.060, BACK_Y1 + 0.015, BACK_Y1 + 0.14, 0.40, 0.62)


def _trim_looks() -> list:
    """The trunk-side trim panel, visual: moulded black plastic with rounded edges and a rounded top inside the collider
    block, and a recessed bezel around the release rod."""
    x0, x1 = KNOB_X + KNOB_L + KNOB_STEM, KNOB_X + KNOB_L + KNOB_STEM + 0.060
    y0, y1 = BACK_Y1 + 0.015, BACK_Y1 + 0.14
    body = m.prism(_rrect(x0, x1, y0, y1, 0.012), 0.40, 0.59)
    top = m.cylinder(0.03, y1 - y0 - 0.004, ((x0 + x1) / 2, (y0 + y1) / 2, 0.59), axis="y")
    bezel = _bx(x0 - 0.0015, x0 + 0.001, KNOB_Y - 0.022, KNOB_Y + 0.022, KNOB_Z - 0.028, KNOB_Z + 0.028)
    return [("trim", m.join(body, top), "plastic"), ("bezel", bezel, "trim")]


def _pull_letters() -> m.Mesh:
    """'PULL' in block strokes on both flat faces of the tab (0.4 mm proud)."""
    w, h, s, g = 0.0030, 0.007, 0.0008, 0.0006
    x = KNOB_X + 0.0006
    z0 = KNOB_Z - h / 2
    strokes = []
    def bar(a0, a1, b0, b1):
        strokes.append((a0, a1, b0, b1))
    for k, ch in enumerate("PULL"):
        a = x + k * (w + g)
        if ch == "P":
            bar(a, a + s, z0, z0 + h); bar(a, a + w, z0 + h - s, z0 + h); bar(a, a + w, z0 + h / 2 - s / 2, z0 + h / 2 + s / 2); bar(a + w - s, a + w, z0 + h / 2, z0 + h)
        elif ch == "U":
            bar(a, a + s, z0, z0 + h); bar(a + w - s, a + w, z0, z0 + h); bar(a, a + w, z0, z0 + s)
        else:
            bar(a, a + s, z0, z0 + h); bar(a, a + w, z0, z0 + s)
    t = KNOB_TAB_T / 2
    out = []
    for a0, a1, b0, b1 in strokes:
        out.append(_bx(a0, a1, KNOB_Y - t - 0.0004, KNOB_Y - t, b0, b1))
        out.append(_bx(a0, a1, KNOB_Y + t, KNOB_Y + t + 0.0004, b0, b1))
    return m.join(*out)


def _knob_mesh() -> m.Mesh:
    stem = m.cylinder(0.005, KNOB_STEM, (KNOB_X + KNOB_L + KNOB_STEM / 2, KNOB_Y, KNOB_Z), axis="x")
    # the pull tab: a flat paddle on the end of the release rod, its faces along the pull (owner's manual figure)
    head = m.box((KNOB_L, KNOB_TAB_T, 2 * KNOB_R), (KNOB_X + KNOB_L / 2, KNOB_Y, KNOB_Z))
    return m.join(stem, head)


def _seat() -> m.Part:
    com = _recline(m.Mesh(np.array([[0.0, (BACK_Y0 + BACK_Y1) / 2, BACK_H / 2]]), np.zeros((0, 3), int))).v[0]
    back = m.hinge("back", "Y", (HINGE_X, 0.0, HINGE_Z), -0.5, FOLD_STOP,
                   [("back", _back_look(), "fabric"), ("head", _headrest_look(), "head")] + _back_looks(),
                   [m.col_hull("back_c", _back_mesh()), m.col_hull("head_c", _headrest())],
                   mass=BACK_KG, gravity=True, com=tuple(com), inertia=(0.45, 0.28, 0.30),
                   damping=HINGE_DAMP, friction=HINGE_FRICTION, effort=200.0)
    stem = m.cylinder(0.005, KNOB_STEM, (KNOB_X + KNOB_L + KNOB_STEM / 2, KNOB_Y, KNOB_Z), axis="x")
    tab = m.box((KNOB_L, KNOB_TAB_T, 2 * KNOB_R), (KNOB_X + KNOB_L / 2, KNOB_Y, KNOB_Z))
    knob = m.slider("knob", "X", -KNOB_TRAVEL, 0.0, [("rod", stem, "post"), ("tab", tab, "pulltab"), ("pull", _pull_letters(), "letter")], [m.col_hull("knob_c", _knob_mesh())],
                    at=(KNOB_X, KNOB_Y, KNOB_Z), mass=0.03, spring=KNOB_SPRING, damping=2.0, friction=0.2, effort=100.0)
    frame_parts = _cushion_looks() + _trim_looks() + [("buckle", _buckle(), "buckle")]
    return m.mechanism("seat", (0.0, 0.0, TOP), [back, knob], LOOKS, frame_parts=frame_parts,
                       frame_colliders=[m.col_hull("trim_c", _trim()), m.col_hull("cushion_c", _cushion())],
                       friction=(0.9, 0.8), solver_iters=64, free_mass=None)


OBJECTS = [_seat()]


class Trial:
    def reset(self, env=None) -> None:
        self.n = 0
        self.latched = None
        self.score = 0.0
        self.last = {}
        if env is not None:
            self.step(env)

    def step(self, env) -> None:
        self.n += 1
        c = m.state(env, "seat")
        th = c.deg("back")
        pull = -c.joint("knob")
        if self.latched is None:
            m.lock(env, "seat", "back", stiffness=LOCK_K)
            self.latched = True
        released = pull >= RELEASE_M
        if self.latched:
            if released:
                m.unlock(env, "seat", "back")
                self.latched = False
        elif not released and th < LATCH_CLEAR_DEG:
            m.lock(env, "seat", "back", stiffness=LOCK_K)
            self.latched = True
        tier = 0.0
        if th >= LATCH_CLEAR_DEG:
            tier = 1 / 3
        if th > PAST_VERTICAL:
            tier = 2 / 3
        if th >= FOLD_STOP - FOLDED_TOL:
            tier = 1.0
        self.score = tier
        self.last = dict(n=self.n, back=round(th, 2), pull_mm=round(1000 * pull, 1), latched=self.latched, score=round(self.score, 3))
        if os.environ.get("TRIAL_TRACE") and self.n % 10 == 0:
            print("trial", self.last, flush=True)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.score >= 1.0


def score(objects) -> float:
    return TRIAL.score


# ---------------- demo ----------------
GRIP_K = d.tilted(100.0, 0.0)  # knob hand: fingers pointing +x, 10 deg up
GRIP_X = d.tilted(105.0, 0.0)  # push hand: fingers pointing +x, 15 deg up, near square to the reclined seatback
PUSH_Y = -0.27                 # push point across the seatback, outboard of the headrest
PUSH_H = 0.52                  # push point height along the seatback, 8 cm below its top


def _w(p) -> np.ndarray:
    return np.asarray(p, float) + np.array([0.0, 0.0, TOP])


def _back_pt(th, h=PUSH_H, depth=0.0) -> np.ndarray:
    """The point on the seatback's trunk face at height h along it, `depth` into it, with the seatback swung th deg."""
    phi = math.radians(RECLINE - th)
    n = np.array([math.cos(phi), 0.0, math.sin(phi)])
    up = np.array([-math.sin(phi), 0.0, math.cos(phi)])
    return _w(np.array([HINGE_X, PUSH_Y, HINGE_Z]) + (-BACK_T / 2 + depth) * n + h * up)


KNOB_GRIP = _w((KNOB_X + 0.005, KNOB_Y, KNOB_Z))   # pads on the head's front, fingertips clear of the trim behind it


def _phase(name) -> None:
    print(f"phase {name}", TRIAL.last, flush=True)


def _hold_knob(L):
    # under the trim and in front of it first: from the ready pose a direct move sweeps the hand through the trim block
    yield from L.move(_w((0.17, 0.30, 0.36)), grip=GRIP_K, finger=d.FINGER_OPEN, t=1.5, label="under trim")
    yield from L.move(KNOB_GRIP + np.array([-0.08, 0.0, 0.0]), grip=GRIP_K, finger=d.FINGER_OPEN, t=1.5, label="near knob")
    yield from L.move(KNOB_GRIP, grip=GRIP_K, finger=d.FINGER_OPEN, t=1.2, label="onto knob")
    yield from L.servo(KNOB_GRIP, timeout=1.0, label="servo knob")
    yield from L.close(0.010, 40.0, label="close knob")


def _pull(L):
    yield from L.move(KNOB_GRIP + np.array([-KNOB_TRAVEL - 0.006, 0.0, 0.0]), t=1.0, label="pull knob")
    yield from L.wait(0.4)


def _let_go(L):
    yield from L.open(label="open knob")
    yield from L.move(L.at() + np.array([-0.08, 0.0, 0.0]), t=0.8, label="back knob")


def _push(ctx, R, goal, lead=0.025, step=0.6, h=PUSH_H):
    """Keep the closed fingertips `lead` into the seatback's trunk face, walking the swing forward until the seatback reads `goal` deg."""
    a = ctx.deg("seat", "back")
    k = 0
    while ctx.deg("seat", "back") < goal and k < 250:
        k += 1
        a = min(a + step, ctx.deg("seat", "back") + 3.0)
        yield from R.move(_back_pt(a, h=h, depth=-0.0) + np.array([lead, 0.0, 0.0]), grip=GRIP_X, t=0.08, label="push")


def _then(*gens):
    for g in gens:
        yield from g


MODES = ("full", "none", "pushonly", "pullonly", "letgo", "shove")


def demo_script(ctx):
    mode = ctx.mode or "full"
    if mode not in MODES:
        raise ValueError(f"mode {mode} not in {MODES}")
    L, R = ctx.left, ctx.right   # left arm: knob; right arm: seatback
    if mode == "none":
        yield from L.wait(3.0)
        return
    start = _back_pt(0.0) + np.array([-0.06, 0.0, 0.0])
    if mode == "shove":
        # lazy: no knob, the push hand driven 8 cm into the latched seatback top (the arm's full force)
        yield from R.move(start, grip=GRIP_X, finger=d.FINGER_CLOSED, t=3.0, label="near back")
        yield from R.move(_back_pt(0.0) + np.array([0.08, 0.0, 0.0]), grip=GRIP_X, t=1.5, label="shove")
        yield from R.wait(3.0)
        _phase("end")
        return
    if mode != "pushonly":
        yield from ctx.together(_then(_hold_knob(L), _pull(L)),
                                R.move(start, grip=GRIP_X, finger=d.FINGER_CLOSED, t=3.0, label="near back"))
        _phase("knob pulled")
        if mode == "letgo":
            yield from _let_go(L)   # one hand: knob let go before the push
    else:
        yield from R.move(start, grip=GRIP_X, finger=d.FINGER_CLOSED, t=3.0, label="near back")
    if mode == "pullonly":
        yield from L.wait(3.0)
        _phase("end")
        return
    yield from R.move(_back_pt(0.0) + np.array([0.005, 0.0, 0.0]), grip=GRIP_X, t=1.0, label="onto back")
    yield from _push(ctx, R, LATCH_CLEAR_DEG + 3.0)
    _phase("off striker")
    if mode == "full":
        yield from ctx.together(_let_go(L), _push(ctx, R, 18.0))
    yield from _push(ctx, R, 40.0, h=0.45)
    _phase("past vertical")
    yield from R.move(R.at() + np.array([-0.10, 0.0, 0.0]), t=1.0, label="back off")
    yield from R.wait(3.0)
    _phase("end")
