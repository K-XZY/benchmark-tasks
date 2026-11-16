"""Turn a humanoid's hung upper body round on its gantry hook, then keep it still and fit its back cover.

The open-source Asimov 1 humanoid's upper body (Menlo Research: the torso shell, both arms and the head, 18.3 kg) hangs by its two shoulder
shackles from a bench gantry's swivel hook, facing the arms, its back open. Its maker's build steps 10.6 and 10.7 turn the robot round and put
the back cover (part 100_04C) on. On the swivel it turns freely: a push that starts it also loads it with momentum, which the swivel's drag alone
takes well over a turn to remove, and once it is round any push off the swivel's axis, the cover's press included, turns it again. In the demo
one hand holds the body still while the other carries the cover from its rest and presses it home; a press straight through the axis also
leaves it still, so one hand can do it. Scored by the effect: the cover seated on the back, the body still.

Debug:  python demokit.py task.py OUT --no-record --stills 20 --mode turn
Final:  python demokit.py task.py OUT --mode turn
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

HERE = pathlib.Path(__file__).resolve().parent
SCENE = "station"
INSTRUCTION = "Turn the body hanging from the swivel hook half a turn so its back faces you, then keep it still and press its back cover into place."
SUCCESS_HOLD_S = 0.0
HOLD_STEPS = 20
TOP = d.TABLE_TOP
DT = d.DT
TRACE = os.environ.get("ASIMOV_TRACE", "1") != "0"

LOOKS = {
    "shell": m.Look(m.rgb(0.88, 0.88, 0.86), roughness=0.6),  # MJF PA12, dyed light
    "joint": m.Look(m.rgb(0.20, 0.20, 0.21), roughness=0.5),
    "gantry": m.Look(m.rgb(0.25, 0.27, 0.30), metallic=0.6, roughness=0.4),
    "chain": m.Look(m.rgb(0.70, 0.70, 0.72), metallic=0.9, roughness=0.3),
    "bay": m.Look(m.rgb(0.12, 0.13, 0.14), roughness=0.7),  # the open back: the battery and boards inside, dark
    "cover": m.Look(m.rgb(0.90, 0.90, 0.88), roughness=0.6),  # 100_04C, MJF PA12 like the shell
    "foam": m.Look(m.rgb(0.20, 0.35, 0.55), roughness=0.9),
}

UB = np.load(HERE / "upper_body.npz")
UB_COM = np.asarray(UB["com"], float)  # in the robot's pelvis frame
UB_KG = float(UB["mass"])
UB_I = np.asarray(UB["I"], float)
NAMES = [str(n) for n in UB["names"]]
CV = np.load(HERE / "cover.npz")  # the back cover 100_04C at its place (pelvis frame), its colliders, and the torso opened where it goes
C0 = np.asarray(CV["cover_c0"], float)  # the cover's origin: the centre of its display mesh, pelvis frame
CHECKS = np.asarray(CV["checks"], float)  # 4 points on the cover's outer corners (by the handle bar, and at the bottom), pelvis frame
SEAT_IN = 0.0  # m: pressed home, the cover sits at its fitted place (moved out until it clears the bay), against the bay
COVER_KG = 0.117  # 115.7 cm3 of MJF PA12 (1.01 g/cm3), from the STEP
HOOK = np.array([0.45, 0.0])  # the swivel's axis, over the body's centre of mass
COM_Z = TOP + 0.35  # the centre of mass's height: the hanging hands clear the table by about 3 cm
SWIVEL_FRICTION = 0.005  # PhysX's joint friction coefficient: a free spin slows from 45 to 27.5 deg/s in 10 s; a real hook's spin-down is to be measured
MARK = math.pi  # the swivel's angle at the mark: the body authored facing the arms (angle 0), the mark its back to them


def _to_hung(v) -> np.ndarray:
    """Points from the robot's pelvis frame to the hung frame: the centre of mass on the swivel's axis, at the origin, the body turned to face the
    arms (-x) at the swivel's angle 0."""
    q = np.atleast_2d(np.asarray(v, float)) - UB_COM
    q = q * np.array([-1.0, -1.0, 1.0])
    return q if np.ndim(v) > 1 else q[0]


TORSO_PIECES = ("front", "above", "below")  # the torso shell's colliders around the open back; the bay fills the inside up to the cover


def _body_link():
    parts, cols = [], []
    for n in NAMES:
        v = _to_hung(UB[f"v_{n}"])
        f = np.asarray(CV["torso_f_open"] if n == "waist_yaw_link" else UB[f"f_{n}"], int)  # the torso shell with its back open
        parts.append((n, m.Mesh(v, f), "joint" if "link" in n and ("elbow" in n or "wrist" in n or "neck" in n) else "shell"))
        if n == "waist_yaw_link":
            for k in TORSO_PIECES:
                cols.append(m.col_hull(f"c_torso_{k}", m.Mesh(_to_hung(CV[f"t_{k}_v"]), np.asarray(CV[f"t_{k}_f"], int))))
            continue
        hv, hf = _to_hung(UB[f"hv_{n}"]), np.asarray(UB[f"hf_{n}"], int)
        cols.append(m.col_hull(f"c_{n}", m.Mesh(hv, hf)))
    pieces = {f"band{i}": (f"band{i}_v", f"band{i}_f") for i in range(int(CV["n_bands"]))}
    pieces.update({"bay_l": ("bay_l_v", "bay_l_f"), "bay_r": ("bay_r_v", "bay_r_f")})
    for k, (v, f) in pieces.items():
        mesh = m.Mesh(_to_hung(CV[v]), np.asarray(CV[f], int))
        parts.append((k, mesh, "bay"))
        cols.append(m.col_hull(f"c_{k}", mesh))
    ev = np.linalg.eigvalsh(UB_I)
    return m.hinge("body", "Z", (0.0, 0.0, 0.0), -math.inf, math.inf, parts, cols, mass=UB_KG, damping=0.0, friction=SWIVEL_FRICTION, gravity=True,
                   com=(0.0, 0.0, 0.0), inertia=(float(UB_I[0, 0]), float(UB_I[1, 1]), float(UB_I[2, 2])))


def _gantry_parts():
    """The bench gantry's frame: two posts at the sides and a beam over the body, the swivel hook and two shackle chains down to the shoulders."""
    top = 0.48  # the beam's height over the centre of mass
    parts = [("beam", m.box((0.04, 0.80, 0.04), (0.0, 0.0, top)), "gantry"),
             ("post_l", m.box((0.04, 0.04, top + 0.35 + 0.02), (0.0, 0.40, (top - 0.35) / 2)), "gantry"),
             ("post_r", m.box((0.04, 0.04, top + 0.35 + 0.02), (0.0, -0.40, (top - 0.35) / 2)), "gantry"),
             ("hook", m.cylinder(0.012, 0.05, (0.0, 0.0, top - 0.045)), "chain")]
    cols = [m.col_box("c_beam", (0.04, 0.80, 0.04), (0.0, 0.0, top)), m.col_box("c_post_l", (0.04, 0.04, top + 0.37), (0.0, 0.40, (top - 0.35) / 2)),
            m.col_box("c_post_r", (0.04, 0.04, top + 0.37), (0.0, -0.40, (top - 0.35) / 2))]
    return parts, cols


_gp, _gc = _gantry_parts()


def _ry(b):
    c, s = math.cos(b), math.sin(b)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _slab_meshes(shift=(0.0, 0.0, 0.0)):
    """The cover's colliders: 3 mm boxes along its outer profile, in the cover's frame (pelvis axes, origin C0), the handle slot left open."""
    out = []
    for c, (L, W, H), p in zip(CV["slab_c"], CV["slab_size"], CV["slab_pitch"]):
        b = m.box((H, W, L))
        v = b.v @ _ry(float(p)).T + (np.asarray(c, float) - C0) + np.asarray(shift, float)
        out.append(m.Mesh(v, b.f))
    return out


HANDLE = (np.array([-0.124, 0.003, 0.391]), (0.012, 0.050, 0.022))  # where the pads bear on the handle bar: flat above and below, 22 mm tall (pelvis
# frame, centre and size). The boxes along the profile are pitched, so flat pads would meet them on an edge, a hinge the cover would hang from


def _handle_mesh():
    c, size = HANDLE
    return m.box(size, c - C0)


COVER_MESH = m.Mesh(np.asarray(CV["cover_v"], float) - C0, np.asarray(CV["cover_f"], int))
TILT = math.radians(15.0)  # the cover leans back on its rest at 15 degrees
STAND_XY = np.array([0.26, -0.50])  # the cover rest, by the right arm, clear of the body's sweep (0.215 m round the swivel)


def _stand():
    """A foam rest shaped to the robot's back, leaning back 15 degrees, with a ledge under the cover's bottom edge, on a foam base."""
    R = _ry(TILT)
    rests = [(np.asarray(CV[f"band{i}_v"], float) - C0 - [SEAT_IN, 0.0, 0.0]) @ R.T for i in range(int(CV["n_bands"]))]
    rest = np.vstack(rests)
    bot = float(np.asarray(CV["cover_v"])[:, 2].min()) - C0[2]
    ledge = m.box((0.06, 0.12, 0.012), (-0.117 - C0[0] + 0.022, 0.0, bot - 0.007))
    ledge = m.Mesh(ledge.v @ R.T, ledge.f)
    low = min(rest[:, 2].min(), ledge.v[:, 2].min())
    lo, hi = np.minimum(rest.min(0), ledge.v.min(0)), np.maximum(rest.max(0), ledge.v.max(0))
    base_h = 0.02
    z0 = TOP + base_h - low  # the stand's origin height: the rest and ledge sit on the base
    base = m.box((hi[0] - lo[0] + 0.02, hi[1] - lo[1] + 0.02, base_h + 0.004), ((hi[0] + lo[0]) / 2, (hi[1] + lo[1]) / 2, low - base_h / 2 + 0.002))
    rest_ms = [m.Mesh(v, np.asarray(CV[f"band{i}_f"], int)) for i, v in enumerate(rests)]
    parts = [(f"rest{i}", rm, "foam") for i, rm in enumerate(rest_ms)] + [("ledge", ledge, "foam"), ("base", base, "foam")]
    cols = [m.col_hull(f"c_rest{i}", rm) for i, rm in enumerate(rest_ms)] + [m.col_hull("c_ledge", ledge), m.col_hull("c_base", base)]
    return z0, parts, cols


STAND_Z, _sp, _sc = _stand()
COVER_START = np.array([STAND_XY[0], STAND_XY[1], STAND_Z])
OBJECTS = [m.mechanism("upper", (HOOK[0], HOOK[1], COM_Z), [_body_link()], LOOKS, frame_parts=_gp, frame_colliders=_gc),
           m.fixed("stand", COVER_START, _sp, _sc, LOOKS),
           m.free("cover", COVER_START, [("cover", COVER_MESH, "cover")], [m.col_hull(f"c_slab{i}", s) for i, s in enumerate(_slab_meshes())] + [m.col_hull("c_handle", _handle_mesh())], LOOKS,
                  mass=COVER_KG, friction=(0.6, 0.5), start_quat=(0.0, math.sin(TILT / 2), 0.0, math.cos(TILT / 2)))]


def body_point(p_pelvis, ang) -> np.ndarray:
    """Points on the body (pelvis frame) in the station frame, with the swivel at `ang`."""
    q = _to_hung(np.atleast_2d(p_pelvis))
    c, sn = math.cos(ang), math.sin(ang)
    out = np.stack([HOOK[0] + c * q[:, 0] - sn * q[:, 1], HOOK[1] + sn * q[:, 0] + c * q[:, 1], COM_Z + q[:, 2]], axis=1)
    return out if np.ndim(p_pelvis) > 1 else out[0]


def cover_error(cover_state, ang) -> tuple[float, float]:
    """How far the cover is from its place on the body (pressed home), with the swivel at `ang`: its centre's distance (m) and the angle (rad)
    it is turned from its place's orientation. At the place the cover's frame has the body's axes, the body turned half round from authored."""
    place = body_point(C0 + [SEAT_IN, 0.0, 0.0], ang)
    c, s = math.cos(ang - math.pi), math.sin(ang - math.pi)
    Rp = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    R = Rp.T @ cover_state.rotation()
    turn = math.acos(max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) / 2.0)))
    return float(np.linalg.norm(np.asarray(cover_state.pos, float) - place)), turn


# -- the scorer -----------------------------------------------------------------------------------------------------------------------------------

REST_RATE, SPIN_RATE = math.radians(1.0), math.radians(20.0)  # still (scored); left spinning (a zero)
MARK_TOL, STOP_RATE = math.radians(10.0), math.radians(5.0)  # traced only, not scored: near the half turn; stopped
DRAG_TAU = 20.0  # s: the swivel's spin-down time constant (a free spin from 45 deg/s: 27.5 after 10 s, 13 after 25 s): a body turning at w coasts on about w x 20 s
TURN_MIN = math.radians(90.0)  # traced only, not scored
NEAR = 0.05  # a fingertip point within 5 cm of the body's colliders: a hand (fingers or palm) may be on it
FAST = math.radians(10.0)  # a collision counts while the body turns faster than this
WINDOW = 2  # steps (0.1 s): spin changes are read over this window, so a blow split between two samples still counts whole
BLOW = math.radians(25.0)  # the spin slowing or reversing by more than 25 deg/s within 0.1 s: a blow (about 11 N at 10 cm from the swivel, 0.257 kg m2), a zero
FREE = math.radians(5.0)  # with no fingertip within 5 cm, the spin changing by more than 5 deg/s within 0.1 s is a collision (the swivel's own drag: under 0.2)
HISTORY = 40  # steps of spin rates kept (2 s)


def _planes():
    """The body's collider hulls as plane sets in the hung frame (pelvis frame turned 180 degrees about z, the centre of mass at the origin): a
    point's largest plane value over a hull is a lower bound on its distance outside that hull."""
    out = []
    eqs = [np.asarray(UB[f"heq_{n}"], float) for n in NAMES if n != "waist_yaw_link"]
    eqs += [np.asarray(CV[f"t_{k}_eq"], float) for k in TORSO_PIECES] + [np.asarray(CV[k], float) for k in [f"band{i}_eq" for i in range(int(CV["n_bands"]))] + ["bay_l_eq", "bay_r_eq"]]  # the torso as its colliders are now
    for eq in eqs:
        nrm, off = eq[:, :3], eq[:, 3]
        # pelvis p = flip(h) + com, flip(x, y, z) = (-x, -y, z): n.p + d = (n * flip) . h + (n . com + d)
        out.append((nrm * np.array([-1.0, -1.0, 1.0]), off + nrm @ UB_COM))
    return out


PLANES = _planes()


def body_gap(points, ang) -> np.ndarray:
    """Each point's distance outside the body's colliders (a lower bound; 0 or less inside), with the swivel at `ang`."""
    q = np.atleast_2d(np.asarray(points, float)) - np.array([HOOK[0], HOOK[1], COM_Z])
    c, sn = math.cos(ang), math.sin(ang)
    h = np.stack([c * q[:, 0] + sn * q[:, 1], -sn * q[:, 0] + c * q[:, 1], q[:, 2]], axis=1)  # into the hung frame
    return np.min(np.stack([np.max(h @ nrm.T + off, axis=1) for nrm, off in PLANES], axis=1), axis=1)


SEAT_POS, SEAT_ANG = 0.005, math.radians(5.0)  # seated: the cover's centre within 5 mm of its place on the back, and turned within 5 degrees of it
ON_POS, ON_ANG = 0.015, math.radians(10.0)  # on the back but not seated: within 15 mm and 10 degrees
LADDER = gk.Ladder(("on_back", 0.5), ("seated", 1.0))  # scored by the effect: the turn is what brings the back within reach


class Trial:
    """The trial's record: the swivel's net angle and rate, the latches and the end hold."""

    def __init__(self):
        self.collision, self.struck = gk.Latch("stopped by a collision"), gk.Latch("a hand struck")
        self.hold_mark, self.hold_on, self.hold_seat = gk.EndHold(HOLD_STEPS), gk.EndHold(HOLD_STEPS), gk.EndHold(HOLD_STEPS)
        self.reset()

    def reset(self, env=None):
        self.n, self.last, self.theta, self.rate, self.rates, self.nears = 0, None, 0.0, 0.0, [], []
        self.turned, self.peak, self.over, self.full, self.on_mark_now = False, 0.0, 0.0, False, False
        self.peak_change, self.peak_free, self.gap = 0.0, 0.0, math.inf
        self.cover_gap, self.cover_ang, self.on_now, self.seat_now = math.inf, math.pi, False, False
        for x in (self.collision, self.struck, self.hold_mark, self.hold_on, self.hold_seat):
            x.reset()

    def step(self, env):
        self.n += 1
        ang = m.state(env, "upper").joint("body")
        if self.last is not None:
            d = wrap(ang - self.last)
            self.theta += d
            self.rate = d / DT
        self.last = ang
        gaps = body_gap(m.fingertips(env), ang)
        self.gap = float(gaps.min())
        if self.last is not None and self.n > 1:
            self.rates.append(self.rate)
            self.nears.append(self.gap <= NEAR)
            self.rates, self.nears = self.rates[-(HISTORY + 1):], self.nears[-(HISTORY + 1):]
            if len(self.rates) > WINDOW:
                recent = self.rates[-(WINDOW + 1):]
                change = max(abs(self.rate - r) for r in recent[:-1])
                slowing = max(math.copysign(1.0, r) * (r - self.rate) if r else 0.0 for r in recent[:-1])  # the drop in rate along the earlier motion: a reversal counts whole
                self.peak_change = max(self.peak_change, slowing)
                self.struck.trip(slowing > BLOW, at=self.n)
                if not any(self.nears[-(WINDOW + 1):]) and max(abs(r) for r in recent) > FAST:
                    self.peak_free = max(self.peak_free, change)
                    self.collision.trip(change > FREE, at=self.n)
        self.peak = max(self.peak, abs(self.theta))
        self.over = max(0.0, self.peak - math.pi)
        # traced only, not scored: turned and stopped, and at rest near the half turn
        self.turned |= abs(self.theta) >= TURN_MIN and abs(self.rate) < STOP_RATE and not (self.collision or self.struck)
        coast_to = self.theta + DRAG_TAU * self.rate  # where the drag alone would leave it
        on_mark = (abs(abs(self.theta) - math.pi) < MARK_TOL and abs(abs(coast_to) - math.pi) < MARK_TOL
                   and abs(self.rate) < REST_RATE)  # turned 180 degrees net, either way round, still, and staying there: past it, or a turn more, is not 180
        self.hold_mark.step(on_mark)
        self.on_mark_now = self.hold_mark.count >= HOLD_STEPS
        self.cover_gap, self.cover_ang = cover_error(m.state(env, "cover"), ang)
        still = abs(self.rate) < REST_RATE
        self.hold_on.step(still and self.cover_gap <= ON_POS and self.cover_ang <= ON_ANG)
        self.hold_seat.step(still and self.cover_gap <= SEAT_POS and self.cover_ang <= SEAT_ANG)
        self.on_now = self.hold_on.count >= HOLD_STEPS
        self.seat_now = self.hold_seat.count >= HOLD_STEPS
        self.full = self.score() >= 1.0
        if TRACE and self.n % 10 == 0:
            print(f"[asimov] step {self.n}: swivel {math.degrees(self.theta):.1f} deg (mark 180), rate {math.degrees(self.rate):.1f} deg/s, "
                  f"overshoot {math.degrees(self.over):.1f} deg, nearest fingertip {1000 * self.gap:.0f} mm, peak slowing in 0.1 s {math.degrees(self.peak_change):.1f} "
                  f"deg/s ({math.degrees(self.peak_free):.2f} with no hand near); turned {self.turned}, on the mark {self.hold_mark.count}; "
                  f"cover off its place by {1000 * self.cover_gap:.1f} mm, {math.degrees(self.cover_ang):.1f} deg, seated {self.hold_seat.count}; "
                  f"score {self.score()}", flush=True)

    def score(self) -> float:
        failed = bool(self.collision or self.struck) or abs(self.rate) > SPIN_RATE  # left spinning: read at whatever step the score is taken, the run's end
        return LADDER.score({"on_back": self.on_now, "seated": self.seat_now}, failed=failed)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.full


def score(objects) -> float:
    return TRIAL.score()


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def demo_script(ctx):
    """Modes: turn (the right pushes the torso's front near its left edge, away: the body starts turning; the left meets the back near its left
    edge as it comes round and rides it to rest on the mark, a moving stop; then the left holds the body by its left elbow while the right takes
    the cover from its rest and presses it home), turnonly (the turn, then the hand away: 0), onehand (the turn, then the right fits the cover
    with the body let go), push (the push alone: it keeps turning), none."""
    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode == "pick":  # the cover pick alone, the body untouched: a check of the grip on the handle bar
        yield from cover_phase(ctx, hold=False)
        return
    if mode == "none":
        for _ in range(80):
            yield
        return

    front_l = np.array([0.034, 0.06, 0.25])  # the torso's front near its left edge (the shell curves in further out), 25 cm up the pelvis frame
    back_l = np.array([-0.143, 0.06, 0.25])  # its back near its left edge, on the collider's surface at this height (the back bulges to -0.151 at the middle): pressed from the arms' side, it is the point that slows the turn
    # the left waits outside the body's sweep (0.215 m round the swivel at this height), on its own side
    yield from l.move(np.array([HOOK[0] - 0.05, 0.26, COM_Z - 0.10]), grip=d.level(0.0), finger=d.FINGER_CLOSED, t=2.0, label="the left hand waits outside the body's sweep")
    ang0 = ctx.state("upper").joint("body")
    p = body_point(front_l, ang0)
    yield from r.move(p - [0.06, 0.0, 0.0], grip=d.level(0.0), finger=d.FINGER_CLOSED, t=2.0, label="the right fingertips before the torso's front")
    yield from r.move(p + [0.02, 0.0, 0.0], t=0.4, label="push it away: the body starts turning")
    yield from r.move(r.at() + [-0.15, -0.10, 0.04], t=0.8, label="the right hand back, well out of the sweep")
    if mode == "push":
        for _ in range(int(40.0 / DT)):  # to the run limit
            yield
        return
    # the catch, a moving stop on the rear face of the body's left upper arm. That face is flat (the arm's collider, hung frame: x 0.049 over
    # y -0.14 to -0.18 and z -0.06 to -0.14) and 16 cm out from the swivel, so near the mark it comes straight at a hand held in its path along
    # -x: the left braces it moving only along x, at a fixed height and y, and never slides across it (following the back instead dragged the
    # body on by friction). The hand meets the face 12 degrees before the mark and backs off with it, pressing in only as far as the body turns
    # faster than an even slowing to rest just short of the mark asks for; once the body is nearly stopped the hand stays where it is, unpressed.
    ARM = np.array([0.049, -0.16, -0.10])  # the contact point on the arm's rear face, hung frame
    TIP = 0.013  # the closed fingertips' ends ahead of the TCP, level grip
    meet = MARK - math.radians(12.0)  # a braced hand stops the arm within about 10 degrees
    REST = MARK - math.radians(2.0)

    def face_x(ang, y):
        """Where the arm's rear face crosses the line at height ARM[2] and world y `y`, with the swivel at `ang`."""
        c, sn = math.cos(ang), math.sin(ang)
        px, py = HOOK[0] + c * ARM[0] - sn * ARM[1], HOOK[1] + sn * ARM[0] + c * ARM[1]
        return px - (y - py) * sn / c  # the face's normal is (cos, sin): moving along it by dy shifts x by -dy tan

    Y = HOOK[1] - ARM[1]  # the face's y at the mark
    Z = COM_Z + ARM[2]

    def hand(x):
        return np.array([x, Y, Z])

    for _ in range(300):
        if wrap(MARK - ctx.state("upper").joint("body")) < math.radians(70.0):
            break
        yield
    yield from l.move(hand(face_x(meet, Y) - TIP - 0.01), grip=d.level(0.0), t=0.8, label="the left in the arm's path, 1 cm off its face")
    prev = ctx.state("upper").joint("body")
    rate = 0.0
    for _ in range(200):
        yield
        ang = ctx.state("upper").joint("body")
        rate, prev = (ang - prev) / DT, ang
        if wrap(meet - ang) < math.radians(1.0):
            break
    w0 = max(rate, math.radians(5.0))
    span = max(wrap(REST - ang), math.radians(5.0))
    K = 0.03  # m of press per rad/s of rate above the even slowing
    FF = 0.002  # m: a steady press while slowing
    print(f"[asimov] ride from {math.degrees(ang):.1f} deg at {math.degrees(w0):.1f} deg/s", flush=True)
    frozen, still = None, 0
    for k in range(int(6.0 / DT)):
        left = max(0.0, wrap(REST - ang))
        w_des = w0 * math.sqrt(min(1.0, left / span))
        press = min(0.012, max(0.0, (FF if w_des > 0 else 0.0) + K * (rate - w_des)))
        if frozen is None and k > 8 and abs(rate) < math.radians(2.0) and abs(wrap(MARK - ang)) < math.radians(8.0):
            frozen = face_x(ang, Y) - TIP
        gap = 0.01 * max(0.0, 1.0 - (k + 1) * DT / 0.4)  # the last 1 cm closed in the first 0.4 s
        x = face_x(ang, Y) - TIP - gap + press if frozen is None else frozen
        yield from l.move(hand(x), grip=d.level(0.0), t=DT, label="brace the arm, slowing the body to rest on the mark")
        ang = ctx.state("upper").joint("body")
        rate, prev = (ang - prev) / DT, ang
        still = still + 1 if abs(rate) < math.radians(0.5) else 0
        if TRACE and k % 4 == 0:
            print(f"[asimov] ride {k}: swivel {math.degrees(ang):.1f} deg, rate {math.degrees(rate):.1f} (wanted {math.degrees(w_des):.1f}) deg/s, press {1000 * press:.1f} mm{', held' if frozen is not None else ''}", flush=True)
        if still >= 10 and abs(wrap(MARK - ang)) < math.radians(8.0):
            break
    for _ in range(20):
        yield
    if mode == "turnonly":  # the turn alone (0): the hand leaves the body at rest on the mark, no cover
        yield from l.move(l.at() + [-0.04, 0.0, 0.0], t=1.0, label="the fingertips off the arm")
        yield from l.move(l.at() + [-0.06, 0.05, 0.05], t=1.0, label="the left hand away")
        for _ in range(40):
            yield
        return
    yield from cover_phase(ctx, hold=(mode != "onehand"))


WRIST = np.asarray(UB["v_left_wrist_yaw_link"], float).mean(0)  # the body's left wrist link, 67 mm across, below the torso: the left hand holds it
BAR = np.array([-0.124, 0.003, 0.391])  # where the right hand's TCP takes the handle bar (pelvis frame, the cover at its place): above the slot, 30 mm tall
SQUEEZE = 20.0  # N, each grip


def _rxa(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rx90():
    return np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def bar_grip(R_cover):
    """The right hand on the handle bar: pointing into the cover (its +x), fingers closing up and down across the bar."""
    return d._as_grip(R_cover @ _rx90() @ d.level(0.0).rot)


def cover_phase(ctx, hold=True):
    """The left holds the body still by its left wrist; the right takes the cover from its rest by the handle bar, brings it square to the back,
    and presses it home. `hold=False`: the left lets go and the right fits the cover alone (the check of what one hand alone does: it scores 1.0, since its press goes through the swivel's axis)."""
    l, r = ctx.left, ctx.right
    ang = ctx.state("upper").joint("body")
    # the left: off the upper arm, down in front of it to the hanging wrist, and grip it from the arms' side
    yield from l.move(l.at() + [-0.06, 0.0, 0.0], t=0.8, label="the fingertips off the arm")
    e = body_point(WRIST, ang)
    shut = -(0.067 / 2 - SQUEEZE / d.GRIPPER["stiffness"])
    if hold:
        yield from l.move(e - [0.05, 0.0, -0.03], grip=d.level(0.0), finger=d.FINGER_OPEN, t=1.5, label="the left down before the body's left wrist")
        yield from l.move(e - [0.05, 0.0, 0.0], t=0.6, label="level with the wrist")
        yield from l.move(e, t=1.0, label="the open fingers either side of the wrist")
        yield from l.move(e, finger=shut, t=0.6, label="grip the wrist: the body is held")
    else:
        yield from l.move(l.at() + [-0.06, 0.08, 0.05], t=1.0, label="the left hand away (one-hand check)")
    # the right: to the cover on its rest, grip its handle bar
    cs = ctx.state("cover")
    R0 = cs.rotation()
    g0 = bar_grip(R0)
    bar0 = cs.to_world(BAR - C0)[0]
    into0 = R0[:, 0]
    bar_w = -(0.030 / 2 - SQUEEZE / d.GRIPPER["stiffness"])
    ajar = -0.0195  # fingers 39 mm apart round the 30 mm bar: the lower one in the slot under it
    if TRACE:
        print(f"[asimov] right TCP before the pick {np.round(r.tcp(), 3)}; bar {np.round(bar0, 3)}", flush=True)
    yield from r.move(r.at() + [0.0, 0.0, 0.10], t=0.8, label="the right up, clear of the body")
    hp, hR = d.ik.tcp_pose(np.array(d.READY, float), r.base)  # through the ready pose: from there the wrist can roll to take the bar
    hp = hp + [0.0, 0.0, 0.15]  # above the cover's top
    yield from r.move(hp, grip=d._as_grip(hR), t=1.2, label="the right back over its ready pose, above the cover")
    yield from r.move(hp, grip=g0, finger=ajar, t=1.0, label="turn the hand to take the bar: fingers above and below it")
    yield from r.move(bar0 - 0.10 * into0 + [0.0, 0.0, 0.08], t=1.2, label="the right above and before the cover's handle bar")
    yield from r.move(bar0 - 0.10 * into0 + [0.0, 0.0, 0.02], t=0.6, label="the right before the handle bar")
    yield from r.move(bar0 - 0.10 * into0, t=0.8, label="level with the bar")
    yield from r.move(bar0, t=1.0, label="the open fingers over and under the bar, the lower one in the slot")
    if TRACE:
        print(f"[asimov] at the bar, open: TCP {np.round(r.tcp(), 4)}, bar {np.round(bar0, 4)}", flush=True)
    yield from r.move(bar0, finger=bar_w, t=0.6, label="grip the handle bar")
    if TRACE:
        print(f"[asimov] bar grip: TCP {np.round(r.tcp(), 4)}, bar {np.round(bar0, 4)}, cover at {np.round(ctx.state('cover').pos, 4)}", flush=True)
    yield from r.move(bar0 + [-0.02, 0.0, 0.04], t=0.8, label="lift the cover off its ledge")
    if TRACE:
        print(f"[asimov] lifted: TCP {np.round(r.tcp(), 4)}, cover at {np.round(ctx.state('cover').pos, 4)} (was {np.round(cs.pos, 4)})", flush=True)
    if ctx.mode == "pick":
        for _ in range(20):
            yield
        return
    # carry it square to the back, 10 cm out, then straight in
    ang = ctx.state("upper").joint("body")
    Rb = _rz(ang - math.pi)  # the body's turn from the exact mark
    g1 = bar_grip(Rb)
    into = Rb[:, 0]
    seat = body_point(BAR + [SEAT_IN, 0.0, 0.0], ang)
    yield from r.move(bar0 + [-0.02, 0.0, 0.12], t=1.0, label="up off the rest")
    yield from r.move((bar0 + seat) / 2 + [-0.08, 0.0, 0.08], grip=g1, t=1.5, label="carry it over, turning it upright")
    yield from r.move(seat - 0.10 * into + [0.0, 0.0, 0.03], t=1.2, label="square to the back, 10 cm out")
    yield from r.move(seat - 0.10 * into, t=0.8, label="line it up with the open back")
    # the arm sags under the cover, and the cover sits in the fingers a little off where the hand meant to take it, so the hand steers by the
    # cover itself, as a camera on it would: its handle bar's place, corrected a little each step
    corr = np.zeros(3)
    ang = ctx.state("upper").joint("body")
    rel = _rz(ang - math.pi).T @ ctx.state("cover").rotation()
    lean = math.atan2(rel[0, 2], rel[2, 2])  # how far the cover leans in the fingers about the bar: its foot out (+) or in (-)
    if TRACE:
        print(f"[asimov] lined up: the cover leans {math.degrees(lean):.2f} deg in the fingers; the hand turns it back", flush=True)
    roll = 0.0
    for k in range(200):  # 10 cm straight in over 2 s, following the body if it has moved, then held on its place (the trial ends once it has sat there 1 s)
        ang = ctx.state("upper").joint("body")
        Rb = _rz(ang - math.pi)
        into = Rb[:, 0]
        seat = body_point(BAR + [SEAT_IN, 0.0, 0.0], ang)
        u = min(1.0, (k + 1) / 40)
        goal = seat - 0.10 * (1 - u) * into  # the bar to its place: the cover hangs from it and its weight swings its foot in against the bay
        cs = ctx.state("cover")
        held = cs.to_world(BAR - C0)[0]
        corr += 0.08 * (goal - held) * np.array([0.0 if u < 1.0 else 1.0, 1.0, 1.0])  # along the push only once there
        corr = np.clip(corr, -0.03, 0.03)
        if u >= 1.0:  # on its place: turn the foot in by what the cover itself still leans, a little each step, within 5 degrees
            rel = Rb.T @ cs.rotation()
            lean = float(np.clip(lean + 0.05 * math.atan2(rel[0, 2], rel[2, 2]), -math.radians(5.0), math.radians(5.0)))
            roll = float(np.clip(roll + 0.05 * math.atan2(rel[2, 1], rel[1, 1]), -math.radians(5.0), math.radians(5.0)))  # turned in its own plane
            if TRACE and k % 5 == 0:
                print(f"[asimov] press {k}: cover lean {math.degrees(math.atan2(rel[0, 2], rel[2, 2])):.2f} deg (hand {math.degrees(lean):.2f}), roll {math.degrees(math.atan2(rel[2, 1], rel[1, 1])):.2f} (hand {math.degrees(roll):.2f}), bar off {np.round(1000 * (held - goal), 1)} mm", flush=True)
        yield from r.move(goal + corr, grip=d._as_grip(Rb @ _rxa(-roll) @ _ry(-lean) @ _rx90() @ d.level(0.0).rot), t=DT, label="press the cover home")
        if TRACE and k == 79:
            cs = ctx.state("cover")
            now = cs.to_world(CHECKS - C0)
            place = body_point(CHECKS + [SEAT_IN, 0.0, 0.0], ang)
            print(f"[asimov] pressed: check points off by (mm, station frame) {np.round(1000 * (now - place), 1).tolist()}", flush=True)
    if TRACE:
        cs = ctx.state("cover")
        ang = ctx.state("upper").joint("body")
        now = cs.to_world(CHECKS - C0)
        place = body_point(CHECKS + [SEAT_IN, 0.0, 0.0], ang)
        print(f"[asimov] at the end: check points off by (mm, station frame) {np.round(1000 * (now - place), 1).tolist()}", flush=True)
