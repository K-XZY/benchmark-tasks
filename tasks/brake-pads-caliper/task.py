"""Fit new front brake pads in their retainers either side of the disc, then swing the caliper down over them.

A front corner of a small hatchback (a Honda Jazz class car) stands on a bench stand, wheel off, turned so that its caliper bracket faces the mechanic: a
vented disc 262 mm across and 21 mm thick, its axis across the bench (y), the bracket at the disc's front edge carrying four stainless pad retainers. The
caliper (about 2 kg) hangs on its upper guide pin, swung up and out of the way, its piston already pushed back. Two new pads (131 x 55 x 15 mm, 0.2 kg)
stand in a slotted block on the bench: the outer pad (no shim) and the inner pad (shim and wear indicator).

Each pad goes in from the front, radially, its ears into the two retainers on its side of the disc, until it seats flush against the retainers' ends
(12 mm of travel). The caliper then swings down about its upper pin, its piston on the inboard side and its fingers on the outboard side passing either side
of the pads with 2 mm to spare and its bridge over their edges with 1 mm: a pad that stands proud stops it short.

Model (declared in code): the caliper is a hinge on the upper guide pin, starting 100 degrees open (authored there, so that the joint's zero is the start); the pin's rubber sleeve holds it where it is left, as a drive that resists with at most PIN_HOLD N m (dry friction), above its
weight's 1.4 N m. The lower guide bolt is left out of this model.

    python demokit.py task.py OUT --mode full                                    # the scripted demo, with a video
    python demokit.py task.py OUT --no-record --mode noflush,onepad,noshut,none   # failure modes: the inner pad partway, the inner pad left out, the caliper left up, nothing
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
INSTRUCTION = "Fit the new brake pads, then swing the caliper down over them."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own 2 s hold: score() and success() agree
HOLD_STEPS = 40  # 2 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# -- the disc (fixed): centre, radius, half thickness; its axis along y, the outboard face at -y. The corner's geometry is written with the disc at x 0.35
# and placed OFF_X further out, where a level grip reaches the pads' retainers.
CX, CZ = 0.35, 0.25
OFF_X = 0.03
R_DISC, HALF = 0.131, 0.0105
# -- the pads: radial 55 mm (x when fitted), 131.4 mm long with ears (z), 15.3 mm thick (y): backing plate 5 mm, lining 10.3 mm
PAD_R0, PAD_R1 = 0.088, 0.143  # the lining's radial span, from the disc's axis
PAD_W, PAD_L, PAD_T, BACK_T = PAD_R1 - PAD_R0, 0.1314, 0.0153, 0.005
LINING_L = 0.104
PLATE_L = 0.110
EAR_X = (-0.010, 0.010)  # the ears' radial span in the pad's frame (x inward)
PAD_MASS = 0.2
PAD_X = CX - (PAD_R0 + PAD_R1) / 2  # the pad's centre when flush
PAD_Y = HALF + 0.003 + PAD_T / 2  # lining 3 mm off the disc's face when fitted (it slides to the disc when the piston is pumped out)
TRAVEL = 0.012  # into the retainers, from the ears' first entry to flush
# -- the retainers' channels (fixed), per pad and end
WALL_IN, WALL_OUT = (0.0150, 0.0190), (0.0308, 0.0340)  # |y| of the clip's walls either side of the ear (the backing plate at |y| 0.0236 to 0.0286): 4.5 mm play towards the disc, 2 mm away
CH_X = (0.208, 0.2485)
CH_Z = (0.0575, 0.0705)  # |z - CZ|: the clip's walls start 2.5 mm beyond the backing plate's ends and cover 8 mm of each ear
STOP_Z = 0.0680  # the abutment at the ear's tip (the ear ends at 0.0657)
FLOOR_X = PAD_X + EAR_X[1]  # the channel's end, where the ear stops when the pad is flush
# -- the caliper (a hinge on its upper guide pin, axis y)
PIN_X, PIN_UP, PIN_LO = 0.200, CZ + 0.080, CZ - 0.080
CAL_MASS = 2.0
CAL_START = float(os.environ.get("CAL_START", "100"))  # degrees the caliper starts swung up on its pin: at 100 the inner pad passes under its piston housing to the retainers
PIN_HOLD = float(os.environ.get("PIN_HOLD", "1.6"))  # N m, the most the pin's sleeve holds
PIN_K = 2000.0  # N m/rad: the sleeve's stiffness before it slips
CLOSED_DEG = 1.0

LOOKS = {
    "disc": m.Look(color=m.rgb(0.55, 0.55, 0.56), roughness=0.35, metallic=0.9),
    "vane": m.Look(color=m.rgb(0.20, 0.19, 0.19), roughness=0.8, metallic=0.4),
    "hat": m.Look(color=m.rgb(0.32, 0.30, 0.29), roughness=0.7, metallic=0.5),
    "stud": m.Look(color=m.rgb(0.78, 0.74, 0.55), roughness=0.3, metallic=1.0),
    "cast": m.Look(color=m.rgb(0.22, 0.22, 0.23), roughness=0.75, metallic=0.4),
    "caliper": m.Look(color=m.rgb(0.30, 0.31, 0.33), roughness=0.6, metallic=0.5),
    "steel": m.Look(color=m.rgb(0.74, 0.75, 0.77), roughness=0.25, metallic=1.0),
    "boot": m.Look(color=m.rgb(0.05, 0.05, 0.05), roughness=0.8),
    "stand": m.Look(color=m.rgb(0.10, 0.25, 0.55), roughness=0.5, metallic=0.3),
    "plate": m.Look(color=m.rgb(0.12, 0.12, 0.13), roughness=0.5, metallic=0.6),
    "lining": m.Look(color=m.rgb(0.36, 0.33, 0.30), roughness=0.95),
    "shim": m.Look(color=m.rgb(0.80, 0.80, 0.82), roughness=0.2, metallic=1.0),
    "wood": m.Look(color=m.rgb(0.62, 0.45, 0.26), roughness=0.8),
}


def _b(name, size, c, look):
    return (name, m.box(size, c), look)


def _corner():
    """The fixed corner, in the station's axes with z above the table: disc, hat and studs, hub, knuckle, stand, the caliper bracket and its retainers."""
    parts = [("cheek_o", m.cylinder(R_DISC, 0.007, (CX, -HALF + 0.0035, CZ), axis="y", segments=64), "disc"),
             ("cheek_i", m.cylinder(R_DISC, 0.007, (CX, HALF - 0.0035, CZ), axis="y", segments=64), "disc"),
             ("vanes", m.cylinder(R_DISC - 0.002, 0.0072, (CX, 0.0, CZ), axis="y", segments=64), "vane"),
             ("hat", m.cylinder(0.075, 0.030, (CX, -HALF - 0.015, CZ), axis="y", segments=48), "hat"),
             ("bore", m.cylinder(0.030, 0.004, (CX, -HALF - 0.031, CZ), axis="y"), "cast"),
             ("hub", m.cylinder(0.045, 0.075, (CX, HALF + 0.0375, CZ), axis="y"), "cast")]
    for k in range(4):  # four wheel studs on a 100 mm circle
        a = math.radians(45 + 90 * k)
        parts.append((f"stud{k}", m.cylinder(0.006, 0.030, (CX + 0.05 * math.cos(a), -HALF - 0.045, CZ + 0.05 * math.sin(a)), axis="y", segments=12), "stud"))
    parts += [_b("knuckle", (0.12, 0.05, 0.26), (CX + 0.02, 0.105, CZ), "cast"),
              _b("strut_clamp", (0.06, 0.06, 0.08), (CX + 0.03, 0.105, CZ + 0.16), "cast"),
              _b("steer_arm", (0.10, 0.03, 0.03), (CX + 0.11, 0.105, CZ - 0.06), "cast"),
              _b("post", (0.04, 0.04, CZ - 0.13), (CX + 0.02, 0.105, (CZ - 0.13) / 2), "stand"),
              _b("foot", (0.22, 0.16, 0.010), (CX + 0.03, 0.10, 0.005), "stand"),
              # the bracket: a plate on the knuckle's front, arms over the disc's rim at each end, the guide-pin bores with their boots
              _b("bracket", (0.11, 0.012, 0.19), (0.245, 0.102, CZ), "cast"),
              _b("arm_up", (0.042, 0.122, 0.012), (0.217, 0.029, CZ + 0.076), "cast"),
              _b("arm_lo", (0.042, 0.122, 0.012), (0.217, 0.029, CZ - 0.076), "cast"),
              ("bore_up", m.cylinder(0.011, 0.034, (PIN_X, 0.079, PIN_UP), axis="y"), "cast"),
              ("bore_lo", m.cylinder(0.011, 0.034, (PIN_X, 0.079, PIN_LO), axis="y"), "cast"),
              ("boot_up", m.cylinder(0.009, 0.006, (PIN_X, 0.0635, PIN_UP), axis="y"), "boot"),
              ("boot_lo", m.cylinder(0.009, 0.006, (PIN_X, 0.0635, PIN_LO), axis="y"), "boot")]
    cols = [] if os.environ.get("BRAKE_NODISC") else [m.col_hull("disc_c", m.cylinder(R_DISC, 2 * HALF, (CX, 0.0, CZ), axis="y", segments=48))]
    cols += [
            m.col_box("post_c", (0.04, 0.04, CZ - 0.13), (CX + 0.02, 0.105, (CZ - 0.13) / 2))]  # noqa
    k = 0
    for s in (-1, 1):  # the outer pad's retainers (-y), the inner pad's (+y)
        for e in (-1, 1):  # bottom, top
            zc = CZ + e * (CH_Z[0] + CH_Z[1]) / 2
            hz = CH_Z[1] - CH_Z[0]
            for nm, (y0, y1) in (("in", WALL_IN), ("out", WALL_OUT)):
                size, c = (CH_X[1] - CH_X[0], y1 - y0, hz), ((CH_X[0] + CH_X[1]) / 2, s * (y0 + y1) / 2, zc)
                parts.append(_b(f"ret{k}_{nm}", size, c, "steel"))
                if not os.environ.get("BRAKE_NORET"):
                    cols.append(m.col_box(f"ret{k}_{nm}_c", size, c))
            size, c = (CH_X[1] - CH_X[0], WALL_OUT[1] - WALL_IN[0], CH_Z[1] - STOP_Z), ((CH_X[0] + CH_X[1]) / 2, s * (WALL_IN[0] + WALL_OUT[1]) / 2, CZ + e * (STOP_Z + CH_Z[1]) / 2)
            parts.append(_b(f"ret{k}_end", size, c, "steel"))
            cols.append(m.col_box(f"ret{k}_end_c", size, c))
            size, c = (0.004, WALL_OUT[0] - WALL_IN[1], STOP_Z - CH_Z[0]), (FLOOR_X + 0.002, s * (WALL_IN[1] + WALL_OUT[0]) / 2, CZ + e * (CH_Z[0] + STOP_Z) / 2)
            parts.append(_b(f"ret{k}_floor", size, c, "steel"))
            cols.append(m.col_box(f"ret{k}_floor_c", size, c))
            k += 1
    return parts, cols


def _open(mesh):
    """A mesh of the closed caliper, turned open by CAL_START about the upper pin: the joint's zero."""
    return mesh.moved((-PIN_X, 0.0, -PIN_UP)).turned(CAL_START, "y").moved((PIN_X, 0.0, PIN_UP))


def _caliper():
    """The caliper, written closed in the corner's frame and turned open to its start: bridge over the pads' edges, fingers outboard, piston housing inboard,
    ears on the pins."""
    br = ((0.021, 0.1286, 0.100), (0.1955, 0.0145, CZ))
    fi = ((0.050, 0.018, 0.084), (0.231, -0.0408, CZ))  # 3 mm off the outer pad's back, 1 mm past the clips' outer walls
    ho = ((0.058, 0.047, 0.090), (0.235, 0.0553, CZ - 0.005))  # the piston's face 3 mm off the inner pad's back; its pin end 10 mm short of the bridge's, so that it clears a pad passing under the raised caliper
    ears = [((0.024, 0.020, 0.060), (PIN_X, 0.050, PIN_UP - 0.010)), ((0.024, 0.020, 0.060), (PIN_X, 0.050, PIN_LO + 0.010))]
    parts = [_b("bridge", *br, "caliper"),
             _b("finger_a", (0.050, 0.018, 0.034), (0.231, -0.0408, CZ + 0.025), "caliper"), _b("finger_b", (0.050, 0.018, 0.034), (0.231, -0.0408, CZ - 0.025), "caliper"),
             _b("housing", *ho, "caliper"),
             ("bore", m.cylinder(0.033, 0.022, (0.235, 0.085, CZ), axis="y"), "caliper"),
             ("bleed", m.cylinder(0.004, 0.012, (0.212, 0.070, CZ + 0.056), axis="z", segments=12), "steel"),
             ("hose_port", m.cylinder(0.006, 0.010, (0.245, 0.101, CZ + 0.02), axis="y", segments=12), "steel"),
             _b("ear_up", *ears[0], "caliper"), _b("ear_lo", *ears[1], "caliper"),
             ("bolt_hole", m.cylinder(0.0055, 0.0205, (PIN_X, 0.050, PIN_LO), axis="y", segments=16), "boot")]
    parts = [(n, _open(mesh), look) for n, mesh, look in parts]
    cols = [m.col_hull(f"{n}_c", _open(m.box(*b))) for n, b in (("bridge", br), ("fingers", fi), ("housing", ho), ("ear_up", ears[0]), ("ear_lo", ears[1]))]
    com = _open(m.Mesh(np.array([[0.215, 0.025, CZ + 0.010]]), np.zeros((0, 3), int))).v[0]
    return m.hinge("caliper", "Y", (PIN_X, 0.0, PIN_UP), -CAL_START - 0.5, max(95.0, CAL_START + 0.5) - CAL_START, parts, cols, mass=CAL_MASS, gravity=True, com=tuple(com),
                   inertia=(0.006, 0.010, 0.012), spring=PIN_K, damping=0.5, friction=0.0, effort=PIN_HOLD)


_kp, _kc = _corner()
CORNER = m.mechanism("corner", (OFF_X, 0.0, TOP), [_caliper()], LOOKS, frame_parts=_kp, frame_colliders=_kc, friction=(0.3, 0.25), solver_iters=48)  # greased clips


def _pad(name: str, side: int):
    """A pad in its own frame: x radially inward, y across (lining towards +y for the outer pad, -y for the inner), z along. `side` -1 outer, +1 inner."""
    s = -side  # the lining's side
    yb, yl = s * (-PAD_T / 2 + BACK_T / 2), s * (PAD_T / 2 - (PAD_T - BACK_T) / 2)
    parts = [_b("plate", (PAD_W, BACK_T, PLATE_L), (0.0, yb, 0.0), "plate"),
             _b("ear_a", (EAR_X[1] - EAR_X[0], BACK_T, (PAD_L - PLATE_L) / 2), ((EAR_X[0] + EAR_X[1]) / 2, yb, (PLATE_L + PAD_L) / 4), "plate"),
             _b("ear_b", (EAR_X[1] - EAR_X[0], BACK_T, (PAD_L - PLATE_L) / 2), ((EAR_X[0] + EAR_X[1]) / 2, yb, -(PLATE_L + PAD_L) / 4), "plate"),
             _b("lining_a", (PAD_W - 0.004, PAD_T - BACK_T, LINING_L / 2 - 0.002), (0.0, yl, LINING_L / 4 + 0.001), "lining"),
             _b("lining_b", (PAD_W - 0.004, PAD_T - BACK_T, LINING_L / 2 - 0.002), (0.0, yl, -LINING_L / 4 - 0.001), "lining")]
    if side > 0:  # the inner pad: a stainless shim on its back and the wear indicator's tab at one end
        parts += [_b("shim", (PAD_W - 0.006, 0.0006, PLATE_L - 0.01), (0.0, s * (-PAD_T / 2 - 0.0003), 0.0), "shim"),
                  _b("indicator", (0.006, 0.0008, 0.016), (PAD_W / 2 - 0.006, s * (PAD_T / 2 - 0.004), PLATE_L / 2 - 0.004), "shim")]
    cols = [m.col_box(f"{name}_plate_c", (PAD_W, BACK_T, PLATE_L), (0.0, yb, 0.0)),
            m.col_box(f"{name}_ears_c", (EAR_X[1] - EAR_X[0], BACK_T, PAD_L), ((EAR_X[0] + EAR_X[1]) / 2, yb, 0.0)),
            m.col_box(f"{name}_lining_c", (PAD_W - 0.004, PAD_T - BACK_T, LINING_L), (0.0, yl, 0.0))]
    return parts, cols


# -- the pads stand on end on a parts box, each on one ear in a shallow slot, in the orientation they go in, lining towards the disc's side they belong to
RACK_X = 0.2325  # the pads' centres
RACK_Y = {"outer": -0.11, "inner": -0.15}
SLOT_D = 0.010
GRIP_DZ = -0.035  # the right hand takes each pad 35 mm below its centre, low enough to pass under the raised caliper
RACK_TCP_Z = 0.16
PAD_START_Z = RACK_TCP_Z - GRIP_DZ
FLOOR_Z = PAD_START_Z - PAD_L / 2 - 0.0005


def _block():
    parts, cols = [], []
    base = ((0.07, 0.10, FLOOR_Z), (RACK_X, -0.13, FLOOR_Z / 2))
    parts.append(_b("box", *base, "wood"))
    cols.append(m.col_box("box_c", *base))
    for name, y in RACK_Y.items():  # a slot for the lower ear: 7 mm across the 5 mm backing plate, 24 mm along its 20 mm
        yb = y + (1 if name == "outer" else -1) * (-PAD_T / 2 + BACK_T / 2)
        ex = RACK_X + (EAR_X[0] + EAR_X[1]) / 2
        for k, (sz, c) in enumerate((((0.024, 0.008, SLOT_D), (ex, yb + 0.0075, FLOOR_Z + SLOT_D / 2)), ((0.024, 0.008, SLOT_D), (ex, yb - 0.0075, FLOOR_Z + SLOT_D / 2)),
                                      ((0.006, 0.023, SLOT_D), (ex + 0.015, yb, FLOOR_Z + SLOT_D / 2)), ((0.006, 0.023, SLOT_D), (ex - 0.015, yb, FLOOR_Z + SLOT_D / 2)))):
            parts.append(_b(f"{name}_rib{k}", sz, c, "wood"))
            cols.append(m.col_box(f"{name}_rib{k}_c", sz, c))
        for k, sgn in enumerate((-1, 1)):  # two wire uprights either side of the pad's back half, so that it stands straight
            sz, c = (0.004, 0.003, 0.080), (RACK_X + 0.018, y + sgn * (PAD_T / 2 + 0.0025), FLOOR_Z + 0.040)
            parts.append(_b(f"{name}_post{k}", sz, c, "steel"))
            cols.append(m.col_box(f"{name}_post{k}_c", sz, c))
    return parts, cols


_bp, _bc = _block()
BLOCK_PART = m.fixed("block", (0.0, 0.0, TOP), _bp, _bc, LOOKS, friction=(0.6, 0.5))
_op, _oc = _pad("outer", -1)
_ip, _ic = _pad("inner", 1)
OUTER = m.free("outer", (RACK_X, RACK_Y["outer"], TOP + PAD_START_Z), _op, _oc, LOOKS, mass=PAD_MASS, friction=(0.6, 0.5))
INNER = m.free("inner", (RACK_X, RACK_Y["inner"], TOP + PAD_START_Z), _ip, _ic, LOOKS, mass=PAD_MASS, friction=(0.6, 0.5))
OBJECTS = [CORNER, BLOCK_PART, OUTER, INNER]
FLUSH = {"outer": np.array([PAD_X + OFF_X, -PAD_Y, TOP + CZ]), "inner": np.array([PAD_X + OFF_X, PAD_Y, TOP + CZ])}


# -- the pin's sleeve: a drive that holds the caliper where it is, with at most PIN_HOLD N m (declared code) ----------------------------------------
class Pin:
    """Dry friction: the drive's rest stays put while its spring holds (under PIN_HOLD); pushed harder, the caliper slips and the rest follows it at the
    spring's full stretch, so the sleeve keeps resisting with PIN_HOLD."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.rest = 0.0

    def step(self, env):
        q = m.state(env, "corner").joint("caliper")
        slip = PIN_HOLD / PIN_K
        if abs(q - self.rest) > slip:
            self.rest = q - math.copysign(slip, q - self.rest)
            asset, j = m._joint(env, "corner", "caliper")
            asset.set_joint_position_target(m._tensor(asset, self.rest), joint_ids=[j])


PIN = Pin()
m.on_step(PIN.step)
m.on_reset(PIN.reset)


def pad_now(env, name) -> dict:
    s = m.state(env, name)
    d = s.pos - FLUSH[name]
    tilt = math.degrees(math.acos(np.clip((np.trace(s.rotation()) - 1) / 2, -1, 1)))
    return {"in_mm": 1000 * (TRAVEL + d[0]) if abs(d[0]) < 0.03 else float("nan"), "d_mm": 1000 * d, "tilt": tilt, "flush": bool(abs(d[0]) < 0.001 and abs(d[1]) < 0.005 and abs(d[2]) < 0.003 and tilt < 4.0),
            "dropped": bool(s.pos[2] < TOP - 0.02), "pos": s.pos}


def corner_now(env) -> dict:
    st = m.state(env, "corner")
    tips = m.fingertips(env)
    pads = {n: pad_now(env, n) for n in ("outer", "inner")}
    marks = np.array([pads["outer"]["pos"], pads["inner"]["pos"], [0.19 + OFF_X, 0.0, TOP + CZ]])
    hands = float(np.min(np.linalg.norm(tips[:, None, :] - marks[None, :, :], axis=2)))
    return {"cal": CAL_START + st.deg("caliper"), "pads": pads, "hands": hands}


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.now = None
        self.dropped = False
        self.hold = 0  # consecutive steps in the full state; any break starts it again
        self.full = False

    def step(self, env):
        self.now = corner_now(env)
        self.dropped = self.dropped or any(p["dropped"] for p in self.now["pads"].values())
        self.hold = self.hold + 1 if (not self.dropped and _full(self.now)) else 0
        self.full = self.hold >= HOLD_STEPS


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def _full(c):
    return all(p["flush"] for p in c["pads"].values()) and c["cal"] < CLOSED_DEG and c["hands"] > 0.05


def success(objects) -> bool:
    return TRIAL.full


def score(objects) -> float:
    c = TRIAL.now
    if c is None:
        return 0.0
    n = sum(p["flush"] for p in c["pads"].values())
    s = 1.0 if TRIAL.full else 0.3 if n == 2 else 0.1 if n == 1 else 0.0  # the full state not yet held 2 s scores 0.3
    return min(s, 0.1) if TRIAL.dropped else s


def _report(ctx, env, label=""):
    c = corner_now(env)
    p = "; ".join(f"{n} pad in {q['in_mm']:5.1f} of {1000 * TRAVEL:.0f} mm, off y {q['d_mm'][1]:+5.1f} z {q['d_mm'][2]:+5.1f}, tilt {q['tilt']:4.1f}, flush {q['flush']}" for n, q in c["pads"].items())
    print(f"[brake] t {ctx.t:5.1f} {label}caliper {c['cal']:5.1f} deg; {p}; hands {1000 * c['hands']:.0f} mm; score {score(None):.2f}", flush=True)


def _dbg(env, label, name=None, arm=None):
    """Debug only (BRAKE_DEBUG): a pad's position and yaw/pitch/roll, and an arm's fingertips."""
    if not os.environ.get("BRAKE_DEBUG"):
        return
    out = f"[brake] dbg {label}:"
    if name:
        st = m.state(env, name)
        R = st.rotation()
        yaw, pitch, roll = (math.degrees(math.atan2(R[1, 0], R[0, 0])), math.degrees(math.asin(np.clip(-R[2, 0], -1, 1))), math.degrees(math.atan2(R[2, 1], R[2, 2])))
        out += f" {name} at {np.round(st.pos - [0, 0, TOP], 4)} yaw {yaw:+.1f} pitch {pitch:+.1f} roll {roll:+.1f};"
    if arm:
        tips = m.fingertips(env)[:12] if arm == "left" else m.fingertips(env)[12:]
        out += f" {arm} tips {np.round(tips.min(0) - [0, 0, TOP], 4)} to {np.round(tips.max(0) - [0, 0, TOP], 4)}"
    print(out, flush=True)


def demo_script(ctx):
    """The right hand takes each pad by its front edge (fingers level, pointing +x, closing across the pad's thickness), lifts it out of the block, carries
    it under the raised caliper to its retainers and pushes it 12 mm in, flush. Then the left hand's closed fingertips push the caliper's bridge down round
    its pin until it closes over the pads, and both hands back away."""
    import demokit as d
    from yam_arena.constants import FINGER_OPEN

    env = ctx.emb._env
    l, r = ctx.left, ctx.right
    for _ in range(10):
        yield
    _report(ctx, env, "start: ")
    g = d.level(0)
    LANE_X = 0.166  # TCP x in the lane: the carried pad's back edge 7 mm in front of the fitted outer pad
    if ctx.mode == "none":
        yield from l.wait(2.0)
        return
    for name in (("outer",) if ctx.mode == "onepad" else ("outer", "inner")):
        p = m.state(env, name).pos
        grip = p + np.array([-PAD_W / 2 - 0.005, 0.0, GRIP_DZ])  # TCP 5 mm behind the front edge (8 mm of it in the jaws)
        if name == "outer":
            yield from r.move((0.17, -0.22, TOP + 0.25), grip=g, finger=FINGER_OPEN * 0.6, t=2.0, label="round the parts box, in front of it")
        yield from r.move((0.17, grip[1], grip[2] + 0.03), grip=g, finger=FINGER_OPEN * 0.6, t=2.0, label=f"in front of the {name} pad")
        yield from r.move(grip + np.array([-0.022, 0.0, 0.0]), t=1.0, label="down to its edge's height")
        yield from r.move(grip, t=1.0, label=f"jaws round the {name} pad's edge")
        yield from r.close(PAD_T, 40.0, t=0.8, label=f"grip the {name} pad")
        if os.environ.get("BRAKE_DEBUG"):
            tips = m.fingertips(env)[12:]
            print(f"[brake] {name} grip: TCP {np.round(r.at() - [0, 0, TOP], 4)}, pad {np.round(m.state(env, name).pos - [0, 0, TOP], 4)}, right tips min {np.round(tips.min(0) - [0, 0, TOP], 4)} max {np.round(tips.max(0) - [0, 0, TOP], 4)}", flush=True)
        yield from r.move(r.at() + np.array([0.0, 0.0, 0.03]), t=0.8, label="lift its ear out of the slot")
        _dbg(env, "lifted", name, "right")
        yield from r.move((LANE_X, r.at()[1], r.at()[2]), t=1.0, label="back off the parts box")
        off = r.at() - m.state(env, name).pos
        lane = FLUSH[name][2] + off[2] - 0.045  # the pad's top 3 mm under the raised caliper's fingers, its front clear of the fitted outer pad
        yield from r.move((LANE_X, -0.07 + off[1], r.at()[2]), t=1.2, label="over the parts box's slots, towards the corner")
        yield from r.move((LANE_X, -0.07 + off[1], lane), t=0.8, label="down to the lane under the caliper")
        _dbg(env, "lane", name, "right")
        yield from r.move((LANE_X, FLUSH[name][1] + off[1], lane), t=2.0, label="across, in line with its retainers")
        _dbg(env, "across", name, "right")
        off = r.at() - m.state(env, name).pos
        yield from r.move((LANE_X, FLUSH[name][1] + off[1], FLUSH[name][2] + off[2]), t=1.0, label="up, level with its retainers")
        here = lambda name=name: m.state(env, name).pos  # noqa: E731
        R = m.state(env, name).rotation()
        roll = math.atan2(R[2, 1], R[2, 2])  # the pad's lean in the grip, about the fingers' axis: turn the wrist to stand it upright
        c, sn = math.cos(-roll), math.sin(-roll)
        yield from r.move(r.at(), grip=d._as_grip(np.array([[1, 0, 0], [0, c, -sn], [0, sn, c]]) @ r.grip.rot), t=0.6, label="square the pad up")
        pre = FLUSH[name] - np.array([TRAVEL + 0.004, 0.0, 0.0])
        _dbg(env, "squared", name, "right")
        yield from r.servo(pre - np.array([0.028, 0.0, 0.0]), tol=0.0003, timeout=3.0, read=here, label="in line with the retainers, clear of the disc")
        _dbg(env, "lined up", name, "right")
        x0 = here()[0]
        for k in range(4):
            yield from r.servo(lambda t, k=k: pre + np.array([(x0 - pre[0]) * (1 - (k + min(t / 0.4, 1.0)) / 4), 0.0, 0.0]), tol=0.0004, timeout=0.8, read=here, label="ears into the retainers' mouths")
            _dbg(env, f"going in {k}", name, "right")
        _dbg(env, "at the mouths", name, "right")
        _report(ctx, env, f"{name} at the retainers: ")
        push = 0.006 if (ctx.mode == "noflush" and name == "inner") else TRAVEL + 0.005  # noflush: the inner pad pushed only partway, its ears in the retainers' mouths
        yield from r.servo(lambda t: pre + np.array([min(t / 1.5, 1.0) * push, 0.0, 0.0]), tol=0.0006, timeout=3.0, read=here, label="push it in, flush")
        _report(ctx, env, f"{name} pushed in: ")
        yield from r.move(r.at(), finger=FINGER_OPEN * 0.35, t=0.5, label="let go of the pad")
        yield from r.move(r.at() + np.array([-0.04, 0.0, 0.0]), t=1.0, label="back out")
        yield from r.move((LANE_X - 0.01, r.at()[1], r.at()[2]), t=0.6, label="back out")
    yield from r.move((0.20, -0.20, TOP + 0.20), t=1.5, label="clear")
    _report(ctx, env, "pads in: ")
    if ctx.mode == "noshut":  # the pads in, the caliper left up
        yield from l.wait(3.0)
        return

    def contact(deg, rho=0.10, into=0.004):
        """A point on the bridge's outer face, rho from the pin, `into` the face; and the push direction there."""
        a = math.radians(deg)
        dx, dz = -0.015 + into, -rho
        p = np.array([OFF_X + PIN_X + dx * math.cos(a) + dz * math.sin(a), 0.01, PIN_UP - dx * math.sin(a) + dz * math.cos(a)])
        return p + np.array([0.0, 0.0, TOP]), np.array([math.cos(a), 0.0, -math.sin(a)])

    def tcp(p, g):
        return p - 0.013 * (g.rot @ np.array([0.0, 0.0, 1.0]))  # the closed fingertips on p

    def push_grip(deg):
        return d.tilted(float(np.clip(90.0 - deg, 45.0, 90.0)), -60)

    start = CAL_START + m.state(env, "corner").deg("caliper")
    p0, n0 = contact(start, rho=0.11, into=0.0)
    g0 = push_grip(start)
    yield from l.move(tcp(p0, g0) - 0.03 * n0, grip=g0, finger=0.0, t=2.5, label="over the caliper's bridge")
    for k, deg in enumerate(np.linspace(start - 4, -8.0, 18)):
        p, n = contact(deg, rho=0.11, into=0.010 if deg > 20 else 0.018)
        g = push_grip(deg)
        yield from l.move(tcp(p, g), grip=g, t=0.6, label="push the caliper down round its pin")
        if k % 3 == 2:
            _report(ctx, env, "swinging: ")
        if k < 2:
            _dbg(env, f"push {deg:.0f}: contact {np.round(p - [0, 0, TOP], 3)}", arm="left")
    _report(ctx, env, "caliper down: ")
    yield from l.move(l.at() + np.array([-0.08, 0.0, 0.04]), t=1.2, label="back away")
    for k in range(60):
        yield
        if k % 20 == 19:
            _report(ctx, env, "let go: ")
