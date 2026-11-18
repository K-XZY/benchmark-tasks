"""Open a new iPhone's box: lift the close-fitting lid straight off its base, slowly enough that the air drawn in through the fit does not lift the base with it, and set the lid down beside the box.

An iPhone 16 box stands on the bench with its seal already off: a lid (outside 164 x 86 x 30 mm, 2 mm walls, 55 g) over a base (158 x 80 x 27 mm, 2 mm walls, 260 g with the phone and cable), 1 mm between the walls
on every side and 27 mm of wall overlap. Both are free rigid bodies, built from boxes. Apple publishes no box size: these are estimates from the phone's size and maker photos.
Code: the air in the fit. While lid and base overlap by more than 2 mm, `Trial.step` applies a force of 400 N s/m times the lid's vertical speed relative to the base, up on the base and
(limited, so a loose lid can't be thrown) down on the lid. A base the lid drags faster than about 6 mm/s (its 2.5 N weight over 400 N s/m) comes up with it. The left hand does the lift; the base can't be held.

Debug:  python demokit.py task.py OUT --no-record --mode full,none,yank,hold,right
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
INSTRUCTION = "Open the iPhone box: lift the lid off and put it down on the bench beside the box. The bottom of the box, with the phone in it, stays on the bench."
SUCCESS_HOLD_S = 0.0  # the scorer holds the end state itself (HOLD_STEPS)
SCORE_OVER = "end"
TOP = d.TABLE_TOP
MM = 1e-3
HOLD_STEPS = 20  # 1 s at 20 Hz
DT = d.DT
G = 9.81

LOOKS = {"lid": m.Look(m.rgb(0.96, 0.96, 0.95), metallic=0.0, roughness=0.55), "base": m.Look(m.rgb(0.93, 0.93, 0.92), metallic=0.0, roughness=0.55),
         "phone": m.Look(m.rgb(0.08, 0.09, 0.11), metallic=0.3, roughness=0.3)}

# -- the box (estimates from the phone's listed size and maker photos). x across (short side), y along (long side); origin: the middle of the footprint, on the bench -----------------------
BOX_X, BOX_Y = float(os.environ.get("IB_X", "0.36")), float(os.environ.get("IB_Y", "-0.04"))
B_HX, B_HY, B_H = 0.040, 0.079, 0.027  # base outside 80 x 158 x 27 mm
L_HX, L_HY, L_H = 0.043, 0.082, 0.030  # lid outside 86 x 164 x 30 mm
WALL = 0.002
M_BASE, M_LID = 0.260, 0.055
PH_HX, PH_HY, PH_H = 0.0358, 0.0738, 0.0078  # iPhone 16: Apple 71.6 x 147.6 x 7.80 mm
OVERLAP = B_H  # the lid, shut, covers the base to its top
AIR_C = float(os.environ.get("IB_C", "400"))  # N s/m
V_LIFT = M_BASE * G / AIR_C  # the lid speed at which the drag equals the base's weight: about 6.4 mm/s


def _bx(x0, x1, y0, y1, z0, z1):
    return m.box((x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def _cb(name, x0, x1, y0, y1, z0, z1):
    return m.col_box(name, (x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def _lid():
    hx, hy, h, t = L_HX, L_HY, L_H, WALL
    boxes = [(-hx, hx, -hy, hy, h - t, h), (-hx, -hx + t, -hy, hy, 0.0, h - t), (hx - t, hx, -hy, hy, 0.0, h - t),
             (-hx + t, hx - t, -hy, -hy + t, 0.0, h - t), (-hx + t, hx - t, hy - t, hy, 0.0, h - t)]
    return m.free("lid", (BOX_X, BOX_Y, TOP + 0.0003), [(f"l{i}", _bx(*b), "lid") for i, b in enumerate(boxes)], [_cb(f"lc{i}", *b) for i, b in enumerate(boxes)], LOOKS, mass=M_LID,
                  friction=(0.35, 0.3), torsional_patch=None, solver_iters=64)


def _base():
    hx, hy, h, t = B_HX, B_HY, B_H, WALL
    boxes = [(-hx, hx, -hy, hy, 0.0, t), (-hx, -hx + t, -hy, hy, 0.0, h), (hx - t, hx, -hy, hy, 0.0, h), (-hx + t, hx - t, -hy, -hy + t, 0.0, h), (-hx + t, hx - t, hy - t, hy, 0.0, h)]
    phone = (-PH_HX, PH_HX, -PH_HY, PH_HY, t, t + PH_H)
    parts = [(f"b{i}", _bx(*b), "base") for i, b in enumerate(boxes)] + [("phone", _bx(*phone), "phone")]
    cols = [_cb(f"bc{i}", *b) for i, b in enumerate(boxes)] + [_cb("phone_c", *phone)]
    return m.free("base", (BOX_X, BOX_Y, TOP + 0.0003), parts, cols, LOOKS, mass=M_BASE, friction=(0.5, 0.45), torsional_patch=None, solver_iters=64)


OBJECTS = [_base(), _lid()]

# -- the air, the scorer ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


def _corners(st, hx, hy, h):
    return st.to_world(np.array([[sx * hx, sy * hy, z] for sx in (-1, 1) for sy in (-1, 1) for z in (0.0, h)]))


def _gap_xy(a, b):
    """Gap between two point sets' bounding rectangles in xy: negative when they overlap."""
    gx = max(a[:, 0].min() - b[:, 0].max(), b[:, 0].min() - a[:, 0].max())
    gy = max(a[:, 1].min() - b[:, 1].max(), b[:, 1].min() - a[:, 1].max())
    return max(gx, gy)


def _dist_to_box(points_local, hx, hy, h):
    q = np.abs(points_local - np.array([0.0, 0.0, h / 2])) - np.array([hx, hy, h / 2])
    return np.linalg.norm(np.maximum(q, 0.0), axis=1)


class Trial:
    def reset(self, env=None) -> None:
        self.n, self.held = 0, 0
        self.dead, self.lifted, self.score, self.full = False, False, 0.0, False
        self.p0 = None
        self.u = 0.0
        self.rise_max = 0.0
        self.last = {}
        self.air_on = False
        self.f_air_max = 0.0
        if env is not None:
            self.step(env)

    def _air(self, env, lid, base) -> float:
        """The air in the fit: 400 N s/m on the lid's vertical speed relative to the base, while they overlap by more than 2 mm and sit one in the other."""
        u_raw = float(lid.lin_vel[2] - base.lin_vel[2])
        self.u += 0.5 * (u_raw - self.u)  # the arm moves in control steps: smooth the speed over about 2 of them
        rel = lid.pos - base.pos
        overlap = base.pos[2] + B_H - lid.pos[2]
        nested = abs(rel[0]) < 0.004 and abs(rel[1]) < 0.004 and lid.tilt_deg() < 15.0
        f = 0.0
        fl = 0.0
        if overlap > 0.002 and nested and abs(self.u) > 1e-5:
            u = self.u
            raw = AIR_C * abs(u)
            cap_b = M_BASE * abs(u) / DT + (M_BASE * G if u > 0 else 0.0)  # no more than brings the base level with the lid within a step (stable once the base is off the bench)
            cap_l = M_LID * abs(u) / DT + M_LID * G
            f = math.copysign(min(raw, cap_b), u)
            fl = -math.copysign(min(raw, cap_b, cap_l), u)
        self.f_air_max = max(self.f_air_max, abs(f))
        if f != 0.0 or self.air_on:
            m.push(env, "base", (0.0, 0.0, f))
            m.push(env, "lid", (0.0, 0.0, fl))
        self.air_on = f != 0.0
        return f

    def step(self, env) -> None:
        lid, base = m.state(env, "lid"), m.state(env, "base")
        if self.p0 is None:
            self.p0 = base.pos.copy()
        f_air = self._air(env, lid, base)
        rise = float(base.pos[2] - self.p0[2])
        self.rise_max = max(self.rise_max, rise)
        shift = float(np.linalg.norm(base.pos[:2] - self.p0[:2]))
        btilt = base.tilt_deg()
        if self.rise_max > 3 * MM or shift > 0.02 or btilt > 5.0 or base.pos[2] < TOP - 0.01:
            self.dead = True
        lc = _corners(lid, L_HX, L_HY, L_H)
        bc = _corners(base, B_HX, B_HY, B_H)
        base_top = float(bc[:, 2].max())
        lid_low = float(lc[:, 2].min())
        if lid_low > base_top + 1 * MM and not self.dead:
            self.lifted = True  # the whole rim above the base's top
        up = abs(float(lid.rotation()[2, 2]))
        on_bench = lid_low < TOP + 3 * MM and up > 0.95 and lid.pos[2] < TOP + L_H + 0.003
        gap = _gap_xy(lc, bc)
        set_down = bool(on_bench and gap > 2 * MM)
        tier = 0.0 if self.dead else (0.5 * self.lifted + (0.5 if (self.lifted and set_down) else 0.0))
        tips = m.fingertips(env)
        dist = min(float(_dist_to_box(lid.to_local(tips), L_HX, L_HY, L_H).min()), float(_dist_to_box(base.to_local(tips), B_HX, B_HY, B_H).min()))
        clear = dist >= 0.03
        still = all(np.linalg.norm(x.lin_vel) < 0.005 and np.linalg.norm(x.ang_vel) < 0.2 for x in (lid, base))
        self.held = self.held + 1 if (clear and still) else 0
        self.score = tier
        self.full = bool(self.held >= HOLD_STEPS and tier == 1.0)
        self.n += 1
        self.last = dict(rise_mm=round(rise * 1000, 2), rise_max_mm=round(self.rise_max * 1000, 2), shift_cm=round(shift * 100, 2), btilt=round(btilt, 1), lid_low_mm=round((lid_low - TOP) * 1000, 1),
                         lifted=self.lifted, set_down=set_down, gap_mm=round(gap * 1000, 1), tier=tier, dead=self.dead, clear=clear, still=still, held=self.held, u_mm_s=round(self.u * 1000, 2), f_air=round(f_air, 2))
        if os.environ.get("IB_TRACE") == "2" and int(os.environ.get("IB_T0", "50")) < self.n < int(os.environ.get("IB_T1", "90")):
            lt = lid.to_local(tips)
            print(f"[ib2] {self.n}: base z {1000 * (base.pos[2] - TOP):.2f} lid z {1000 * (lid.pos[2] - TOP):.2f} lid tilt {lid.tilt_deg():.2f} lid dxy {np.round(1000 * (lid.pos[:2] - base.pos[:2]), 1)} "
                  f"tip z min {1000 * (tips[:, 2].min() - TOP):.1f} lid vz {1000 * lid.lin_vel[2]:.1f} tip lid-x {np.round(1000 * lt[:, 0].min(), 1)}..{np.round(1000 * lt[:, 0].max(), 1)} base vz {1000 * base.lin_vel[2]:.1f}", flush=True)
        if self.n % 20 == 0 and os.environ.get("IB_TRACE") == "1":
            print(f"[ib] step {self.n}: {self.last}", flush=True)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    """The lid off the base and down on the bench clear of it, the base never lifted more than 3 mm, moved more than 2 cm or tilted past 5 degrees, hands 3 cm clear, still 1 s."""
    return TRIAL.full


def score(objects) -> float:
    """0.5 once the lid's rim has risen above the base's top with the base down; 0.5 more once the lid is on the bench, clear of the base. 0 if the base ever rose 3 mm, moved 2 cm or tilted 5 degrees."""
    return TRIAL.score


# -- the demo ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

RATE = float(os.environ.get("IB_RATE", "0.0035"))  # m/s: the lid's rise while it overlaps the base (limit about 6.4 mm/s)
PINCH_Z = float(os.environ.get("IB_PINCHZ", "0.022"))  # TCP height over the bench at the pinch
GRIP_N = float(os.environ.get("IB_GRIPN", "20"))
YAW = float(os.environ.get("IB_YAW", "90"))
SET_DY = float(os.environ.get("IB_SETDY", "0.18"))  # the lid set down this far along +y (towards the left hand) from where it stood: 19 mm clear of the base's end
LIFT_Z = 0.075


def demo_script(ctx):
    """Modes: full (the left hand lifts the lid slowly and sets it down). none: the arms wait. yank: the lid lifted at 100 mm/s. tilt: the lid tipped 20 degrees, then yanked. hold: lifted slowly, then held in the air.
    right: the right hand does it instead. Debug: lift (lift slowly and stop at the top)."""
    mode = ctx.mode
    if mode not in ("full", "none", "yank", "hold", "right", "lift", "tilt"):
        raise ValueError(f"unknown mode {mode!r}")
    arm = ctx.right if mode == "right" else ctx.left
    if mode == "none":
        yield from ctx.left.wait(3.0)
        return
    g = d.down(YAW if mode == "right" else -YAW)
    lid = ctx.pos("lid")
    side = 1 if mode == "right" else 0
    env = ctx.emb._env

    def jaws():
        """The middle of this hand's open fingers, from its own fingertip points (xy), at the TCP's height."""
        c = m.fingertips(env)[12 * side:12 * side + 12].mean(axis=0)
        return np.array([c[0], c[1], arm.tcp()[2]])

    def lid_xy(z):
        p = ctx.pos("lid")
        return np.array([p[0], p[1], z])

    over = lid_xy(TOP + PINCH_Z + 0.10)
    yield from arm.move(over, grip=g, finger=d.FINGER_OPEN, t=2.0, label="over the lid")
    yield from arm.servo(lambda t: lid_xy(arm.tcp()[2]), timeout=2.5, read=jaws, tol=0.0008, gain=0.3, label="jaws over the lid")
    yield from arm.move(arm.at() + np.array([0.0, 0.0, TOP + PINCH_Z - arm.at()[2]]), t=1.5, label="down to the lid")
    yield from arm.servo(lambda t: lid_xy(arm.tcp()[2]), timeout=2.0, read=jaws, tol=0.0008, gain=0.3, label="jaws at the lid")
    print(f"[ib] jaws {np.round((jaws() - lid_xy(arm.tcp()[2])) * 1000, 1)} mm off the lid's middle, TCP {np.round((arm.tcp() - ctx.pos('lid')) * 1000, 1)}", flush=True)
    yield from arm.close(2 * L_HX, GRIP_N, t=0.8, label="pinch the lid")
    yield from arm.wait(0.4)
    start_lid_z = ctx.pos("lid")[2]
    if mode == "tilt":  # tip the held lid 20 degrees about its long axis (past the air's 15 degree cutoff), then yank it
        yield from arm.arc(ctx.pos("lid"), (0.0, 1.0, 0.0), 20.0, t=1.0, label="tilt the lid")
        yield from arm.wait(0.3)
        print(f"[ib] tilted: {TRIAL.last} lid tilt {ctx.state('lid').tilt_deg():.1f}", flush=True)
        start_lid_z = ctx.pos("lid")[2]
    hold_at = arm.at().copy()
    rate = 0.10 if mode in ("yank", "tilt") else RATE
    clear_by = L_H * 0.0 + B_H + 0.012  # lid origin this high: the rim 12 mm above the base's top
    t0 = [0.0]

    def target():
        t0[0] += DT
        r_des = rate * t0[0]
        r = ctx.pos("lid")[2] - start_lid_z
        corr = float(np.clip(0.5 * (r_des - r), -0.002, 0.002))
        return hold_at + np.array([0.0, 0.0, r_des + corr])

    yield from arm.follow(target, t=(clear_by / rate) + 3.0, speed=0.15 if mode == "yank" else 0.02, until=lambda: ctx.pos("lid")[2] - TOP >= clear_by, label="lift")
    print(f"[ib] lifted: {TRIAL.last}", flush=True)
    if mode in ("lift", "hold"):
        yield from arm.wait(2.0 if mode == "lift" else 25.0)
        print(f"[ib] end {TRIAL.last}", flush=True)
        return
    top = arm.at().copy()
    yield from arm.move(top + [0.0, 0.0, LIFT_Z - clear_by], t=0.8, label="up")
    yield from arm.move(np.array([arm.at()[0], arm.at()[1] + SET_DY, arm.at()[2]]), t=2.5, label="across")
    down = np.array([arm.at()[0], arm.at()[1], TOP + PINCH_Z + 0.003])
    yield from arm.move(down, t=2.0, label="down")
    yield from arm.wait(0.3)
    yield from arm.open(label="let go")
    yield from arm.move(arm.at() + [0.0, 0.0, 0.08], t=1.0, label="up")
    yield from arm.move(arm.at() + [0.0, -0.06, 0.0], t=1.0, label="away")
    yield from arm.wait(1.5)
    print(f"[ib] end {TRIAL.last}", flush=True)
