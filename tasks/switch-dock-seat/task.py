"""Dock a Nintendo Switch OLED: lower the console straight down its dock's slot until its USB-C port seats on the hidden plug at the slot's bottom.

The console (Nintendo: 102 x 242 x 13.9 mm with Joy-Con attached, 420 g) starts screen down on a fixed 120 mm rest, its top edge overhanging 30 mm toward the robot, port edge
toward the dock. The dock (Nintendo: about 104 x 175 x 51 mm, 313 g) stands 16 cm toward the robot from the console's middle: a slot 16 mm wide and 45 mm deep runs along y; at the middle of the slot's floor
stands a USB-C plug, 8.25 x 2.4 mm and 6.5 mm tall, with a chamfered tip. The console's port is a pocket in its lower edge, 9.25 x 3.4 mm and 6.6 mm deep (0.5 mm to spare each side).
Geometry is real: the plug goes into the pocket or it does not. Code: the seating detent (an 8 N resistance over the last 6 mm of the plug's travel) and the scorer.

Debug:  python demokit.py task.py OUT --no-record --mode full,none,drop,rest,crooked
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
INSTRUCTION = "Put the console in its dock so it connects."
SUCCESS_HOLD_S = 0.0  # the scorer holds the end state itself (HOLD_STEPS)
SCORE_OVER = "end"
TOP = d.TABLE_TOP
HOLD_STEPS = 20  # 1 s at 20 Hz
MM = 1e-3

LOOKS = {"body": m.Look(m.rgb(0.10, 0.10, 0.11), metallic=0.1, roughness=0.35), "screen": m.Look(m.rgb(0.02, 0.03, 0.05), metallic=0.0, roughness=0.15),
         "jc_l": m.Look(m.rgb(0.05, 0.55, 0.85), metallic=0.0, roughness=0.4), "jc_r": m.Look(m.rgb(0.85, 0.12, 0.10), metallic=0.0, roughness=0.4),
         "dock": m.Look(m.rgb(0.93, 0.93, 0.92), metallic=0.0, roughness=0.4), "slot": m.Look(m.rgb(0.12, 0.12, 0.13), metallic=0.0, roughness=0.5),
         "plug": m.Look(m.rgb(0.75, 0.76, 0.78), metallic=0.9, roughness=0.25), "rest": m.Look(m.rgb(0.55, 0.40, 0.25), metallic=0.0, roughness=0.7)}

# -- the console (Nintendo: 102 x 242 x 13.9 mm with Joy-Con, 0.93 lb = 420 g). Frame: origin at its middle; x the thickness (the screen faces -x), y the width, z the height,
#    the lower edge with the port at z = -51 mm.
CT, CW, CH = 0.0139, 0.242, 0.102
M_CON = 0.420
PORT_Y, PORT_X, PORT_D = 0.00925, 0.0034, 0.0066  # the port pocket: width, thickness, depth (estimates)
JC = 0.0345  # a Joy-Con's width, for the picture only

# -- the dock (Nintendo: about 4.1 x 6.9 x 2.0 in = 104 x 175 x 51 mm; 0.69 lb = 313 g). Frame: origin at the middle of its footprint on the bench; x depth, y length, z up.
DX, DY, DZ = 0.104, 0.175, 0.051
SLOT_W, FLOOR = 0.016, 0.006  # the slot, 45 mm deep from the top (estimates)
PLUG_W, PLUG_T, PLUG_H = 0.00825, 0.0024, 0.0065  # USB Type-C plug
M_DOCK = 0.313

# -- the layout (metres, station frame)
DOCK_XY = (float(os.environ.get('SW_DX', '0.30')), -0.10)
CON_X, CON_Y = float(os.environ.get('SW_CX', '0.46')), -0.10
REST_H = float(os.environ.get('SW_RH', '0.12'))
REST_X = (CON_X - 0.021, CON_X + 0.045)  # the console's top edge is 51 mm toward the robot from its middle: it overhangs the rest by 30 mm
REST_Y = 0.07

SEAT_Z = TOP + FLOOR + CH / 2  # the console's middle height when its lower edge is on the slot's floor


def _bx(x0, x1, y0, y1, z0, z1):
    return m.box((x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def _cb(name, x0, x1, y0, y1, z0, z1):
    return m.col_box(name, (x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def _console() -> m.Part:
    hx, hy, hz = CT / 2, CW / 2, CH / 2
    pz = -hz + PORT_D
    boxes = [(-hx, hx, -hy, hy, pz, hz), (-hx, hx, -hy, -PORT_Y / 2, -hz, pz), (-hx, hx, PORT_Y / 2, hy, -hz, pz), (-hx, -PORT_X / 2, -PORT_Y / 2, PORT_Y / 2, -hz, pz),
             (PORT_X / 2, hx, -PORT_Y / 2, PORT_Y / 2, -hz, pz)]
    cols = [_cb(f"c{i}", *b) for i, b in enumerate(boxes)]
    # the picture: the back, the screen face, two Joy-Con
    jx = hx + 0.0001
    parts = [("body", _bx(-hx, hx, -hy + JC, hy - JC, -hz, hz), "body"), ("screen", _bx(-hx - 0.0004, -hx, -hy + JC + 0.004, hy - JC - 0.004, -hz + 0.004, hz - 0.004), "screen"),
             ("jcl", _bx(-hx, hx, hy - JC, hy, -hz, hz), "jc_l"), ("jcr", _bx(-hx, hx, -hy, -hy + JC, -hz, hz), "jc_r")]
    # the pocket shows as a dark notch: it is cut from the body's lower edge by drawing the body in two parts
    parts[0] = ("body_u", _bx(-hx, hx, -hy + JC, hy - JC, pz, hz), "body")
    parts += [("body_l1", _bx(-hx, hx, -hy + JC, -PORT_Y / 2, -hz, pz), "body"), ("body_l2", _bx(-hx, hx, PORT_Y / 2, hy - JC, -hz, pz), "body"),
              ("body_f", _bx(-hx, -PORT_X / 2, -PORT_Y / 2, PORT_Y / 2, -hz, pz), "body"), ("body_b", _bx(PORT_X / 2, hx, -PORT_Y / 2, PORT_Y / 2, -hz, pz), "body")]
    # lying screen down, its port edge toward +x: x_c -> +z, z_c -> -x, y_c -> y: -90 degrees about y (xyzw)
    s2 = math.sqrt(0.5)
    return m.free("console", (CON_X, CON_Y, TOP + REST_H + hx + 0.0002), parts, cols, LOOKS, mass=M_CON, friction=(0.4, 0.4), torsional_patch=None, solver_iters=96, start_quat=(0.0, -s2, 0.0, s2))


def _plug_mesh() -> m.Mesh:
    w, t = PLUG_W / 2, PLUG_T / 2
    big = np.array([(t, -w), (t, w), (-t, w), (-t, -w)])
    small = np.array([(t - 0.0006, -w + 0.0008), (t - 0.0006, w - 0.0008), (-t + 0.0006, w - 0.0008), (-t + 0.0006, -w + 0.0008)])
    return m.join(m.prism(big, FLOOR, FLOOR + PLUG_H - 0.0015), m.prism(small, FLOOR + PLUG_H - 0.0015, FLOOR + PLUG_H))


def _dock() -> m.Part:
    hw = SLOT_W / 2
    wall = [(-DX / 2, -hw, -DY / 2, DY / 2, FLOOR, DZ), (hw, DX / 2, -DY / 2, DY / 2, FLOOR, DZ)]
    floor = (-DX / 2, DX / 2, -DY / 2, DY / 2, 0.0, FLOOR)
    parts = [("floor", _bx(*floor), "dock"), ("w0", _bx(*wall[0]), "dock"), ("w1", _bx(*wall[1]), "dock"), ("plug", _plug_mesh(), "plug"), ("slotfloor", _bx(-hw, hw, -DY / 2, DY / 2, FLOOR, FLOOR + 0.0002), "slot")]
    cols = [_cb("cf", *floor), _cb("cw0", *wall[0]), _cb("cw1", *wall[1]), m.col_hull("plug_c", _plug_mesh())]
    return m.free("dock", (DOCK_XY[0], DOCK_XY[1], TOP + 0.0002), parts, cols, LOOKS, mass=M_DOCK, friction=(0.4, 0.4), torsional_patch=None, solver_iters=64)


def _rest() -> m.Part:
    x0, x1 = REST_X
    b = (x0, x1, -REST_Y, REST_Y, 0.0, REST_H)
    return m.fixed("rest", (0.0, CON_Y, TOP), [("rest", _bx(*b), "rest")], [_cb("c", *b)], LOOKS, friction=(0.4, 0.4))


OBJECTS = [_rest(), _dock(), _console()]

DETENT_N = 8.0  # the plug's grip on the port over the last 6 mm (estimate)
SEATED_DEPTH = PORT_D - 0.0004  # plug in to within 0.4 mm of the pocket's roof
SLAM_V = 0.25  # a console arriving faster than this at the plug is dropped in, not lowered (a real plug's shell could be damaged; estimate)
TILT_SEAT, TILT_REST = 2.0, 5.0  # degrees


def _geom(env):
    con, dock = m.state(env, "console"), m.state(env, "dock")
    R = con.rotation()
    tip = R.T @ (dock.to_world(np.array([[0.0, 0.0, FLOOR + PLUG_H]]))[0] - con.pos)  # the plug's tip in the console's frame
    depth = tip[2] + CH / 2  # how far the tip stands above the lower edge: the plug's travel into the port
    aligned = bool(abs(tip[1]) < PORT_Y / 2 and abs(tip[0]) < PORT_X / 2)
    tilt = math.degrees(math.acos(float(np.clip(R[2, 2], -1.0, 1.0))))
    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    facing = bool(R[0, 0] > 0.0)  # the screen toward the robot
    corners = con.to_world(np.array([[sx * CT / 2, sy * CW / 2, -CH / 2] for sx in (-1, 1) for sy in (-1, 1)]))
    return con, dock, R, tip, depth, aligned, tilt, yaw, facing, corners


class Trial:
    def reset(self, env=None) -> None:
        self.n, self.held = 0, 0
        self.dead, self.score, self.full = False, 0.0, False
        self.d0 = None
        self.impact, self.latched, self.armed = 0.0, False, False
        self.last = {}
        if env is not None:
            self.step(env)

    def step(self, env) -> None:
        con, dock, R, tip, depth, aligned, tilt, yaw, facing, corners = _geom(env)
        if self.d0 is None:
            self.d0 = dock.pos.copy()
        # the detent: over the plug's last 6 mm of travel, a pull out of the port along the console's up axis, 8 N; once in, nothing
        if aligned and -0.0005 < depth < SEATED_DEPTH and tilt < 8.0 and not self.latched:
            m.push(env, "console", R[:, 2] * DETENT_N)
        else:
            m.push(env, "console", (0.0, 0.0, 0.0))
        if aligned and depth >= SEATED_DEPTH:
            self.latched = True
        if aligned and -0.004 < depth < 0.0005 and not self.latched:
            self.impact = max(self.impact, float(-con.lin_vel[2]))
        seated = bool(self.latched and aligned and depth >= SEATED_DEPTH - 0.0003 and tilt <= TILT_SEAT and abs(yaw) <= TILT_SEAT and facing)
        low = float(corners[:, 2].min())
        in_slot = bool(low < dock.pos[2] + DZ - 0.004 and abs(con.pos[0] - dock.pos[0]) < SLOT_W)
        upright = bool(tilt <= TILT_REST and abs(yaw) <= TILT_REST and facing)
        kept = bool(np.linalg.norm(dock.pos[:2] - self.d0[:2]) <= 0.02 and dock.tilt_deg() <= 5.0)
        fell = con.pos[2] < TOP - 0.01 or dock.pos[2] < TOP - 0.01
        if fell or not kept:
            self.dead = True
        slammed = self.impact > SLAM_V
        if self.dead:
            tier = 0.0
        elif seated and not slammed:
            tier = 1.0
        elif (seated or (in_slot and upright)) and kept:
            tier = 0.5
        elif in_slot:
            tier = 0.25
        else:
            tier = 0.0
        tips = m.fingertips(env)
        tl = con.to_local(tips)
        q = np.abs(tl) - np.array([CT / 2, CW / 2, CH / 2])
        clear = bool(np.linalg.norm(np.maximum(q, 0.0), axis=1).min() >= 0.03)
        still = all(np.linalg.norm(x.lin_vel) < 0.005 and np.linalg.norm(x.ang_vel) < 0.2 for x in (con, dock))
        self.held = self.held + 1 if (clear and still) else 0
        if self.held >= HOLD_STEPS:
            self.score = tier
        if self.dead:
            self.score = 0.0
        self.full = bool(self.held >= HOLD_STEPS and tier == 1.0)
        self.n += 1
        self.last = dict(depth_mm=round(depth * 1000, 2), aligned=aligned, tip_xy_mm=(round(tip[0] * 1000, 2), round(tip[1] * 1000, 2)), tilt=round(tilt, 2), yaw=round(yaw, 2), seated=seated,
                         latched=self.latched, impact=round(self.impact, 3), in_slot=in_slot, upright=upright, kept=kept, tier=tier, clear=clear, still=still, held=self.held)
        if self.n % 20 == 0 and os.environ.get("SW_TRACE"):
            print(f"[sw] step {self.n}: {self.last}", flush=True)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    """The port fully on the plug (in to within 0.7 mm of the pocket's stop, the plug inside the pocket), the console upright within 2 degrees with its screen toward the robot, arrived slower
    than 0.25 m/s, the dock within 2 cm and 5 degrees of where it stood, hands 3 cm clear, all still 1 s."""
    return TRIAL.full


def score(objects) -> float:
    return TRIAL.score


# -- the demo ---------------------------------------------------------------------------------------------------------------------

GRIP_N = float(os.environ.get("SW_GRIP", "35"))
LIFT = float(os.environ.get("SW_LIFT", "0.07"))
EDGE_IN = float(os.environ.get("SW_IN", "0.016"))  # the TCP this far in from the top edge
STAND_Z = float(os.environ.get("SW_STAND", "0.15"))
GAP = float(os.environ.get("SW_GAP", "0.034"))  # the jaws' opening on the way in: under the 30 mm overhang


def _g0():
    """Fingers level pointing +x, jaws closing up and down: a down grip with its jaws along x (right arm +90), pitched so its fingers point +x."""
    base = d.down(float(os.environ.get('SW_YAW', '90')))
    r = d.axis_angle_matrix(np.array([0.0, 1.0, 0.0]), math.radians(-90.0)) @ base.rot
    return d._as_grip(r)


def _con(ctx):
    return ctx.pos("console")


def _port(ctx):
    """The point on the console's lower edge where its port is (read from the console's pose), with the console's middle height: what the servo steers over the plug."""
    st = ctx.state("console")
    p = st.pos + st.rotation() @ np.array([0.0, 0.0, -CH / 2])
    return np.array([p[0], p[1], st.pos[2]])


def _unit(v):
    return v / np.linalg.norm(v)


def _level(ctx, arm):
    """Level the held console by feedback: turn the hand about the console's middle to bring its up axis vertical and its thickness along x."""
    for _ in range(2):
        R = ctx.state("console").rotation()
        z = R[:, 2]
        tilt = math.degrees(math.acos(float(np.clip(z[2], -1, 1))))
        if tilt > 0.7:
            n = _unit(np.cross(z, [0.0, 0.0, 1.0]))
            yield from arm.arc(_con(ctx), n, tilt, t=0.6, label="level")
        R = ctx.state("console").rotation()
        yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
        if abs(yaw) > 0.7:
            yield from arm.arc(_con(ctx), (0.0, 0.0, 1.0), -yaw, t=0.6, label="yaw")
    R = ctx.state("console").rotation()
    print(f"[sw] leveled: tilt {math.degrees(math.acos(float(np.clip(R[2, 2], -1, 1)))):.2f} deg, yaw {math.degrees(math.atan2(R[1, 0], R[0, 0])):.2f} deg", flush=True)


def demo_script(ctx):
    """Modes: full. none: the arms wait. drop: let go 8 cm above the plug. rest: let go just above the plug. crooked: lowered with a 10 degree roll across its width."""
    mode = ctx.mode
    if mode not in ("full", "none", "drop", "rest", "crooked"):
        raise ValueError(f"unknown mode {mode!r}")
    R = ctx.right
    if mode == "none":
        yield from R.wait(3.0)
        return
    g0 = _g0()
    c = _con(ctx)
    edge = np.array([c[0] - CH / 2, c[1], c[2]])  # the top edge's middle, on the console's mid-thickness
    tcp = edge + np.array([EDGE_IN, 0.0, 0.0])
    yield from R.move(tcp + [-0.10, 0.0, 0.0], grip=g0, finger=-GAP / 2, label="approach")
    yield from R.servo(tcp + [-0.10, 0.0, 0.0], timeout=1.0, label="approach")
    yield from R.move(tcp, t=1.2, label="in")
    yield from R.servo(tcp, timeout=1.0, label="in")
    yield from R.close(CT, GRIP_N, label="grip the top edge")
    yield from R.move(R.at() + [0.0, 0.0, LIFT], t=1.2, label="lift")
    dk = ctx.pos("dock")
    # carry back level to over the dock, high enough that the hanging console clears it, then stand it up there
    hi = np.array([dk[0], dk[1], TOP + STAND_Z])
    yield from R.move(hi, t=2.5, label="carry")
    yield from R.servo(hi, timeout=1.5, label="carry")
    yield from R.turn(90.0, axis=(0.0, 1.0, 0.0), t=2.0, label="stand up")
    yield from R.wait(0.4)
    print(f"[sw] stood: console {np.round(_con(ctx), 3)}, {TRIAL.last}", flush=True)
    over = np.array([dk[0], dk[1], _con(ctx)[2]])
    yield from R.servo(over, timeout=2.0, read=lambda: _port(ctx), tol=0.0004, label="over slot")
    yield from _level(ctx, R)
    if mode == "crooked":
        yield from R.arc(_con(ctx), (1.0, 0.0, 0.0), float(os.environ.get('SW_CROOK', '10')), t=0.6, label="crook")
    yield from R.servo(over, timeout=2.0, read=lambda: _port(ctx), tol=0.0003, label="over slot 2")
    print(f"[sw] over slot: tip {TRIAL.last['tip_xy_mm']} depth {TRIAL.last['depth_mm']} tilt {TRIAL.last['tilt']}", flush=True)
    if mode == "drop":
        yield from R.servo(np.array([dk[0], dk[1], SEAT_Z + 0.080]), timeout=2.0, read=lambda: _con(ctx), tol=0.002, label="to 8 cm")
        yield from R.open(t=0.3, label="drop")
        yield from R.wait(2.0)
        return
    # lower in a straight line: 12 mm/s, steered by the console's own pose, until the plug is 3 mm from the port, then the push to home
    z0 = _port(ctx)[2]

    def goal(t):
        z = max(SEAT_Z + 0.003, z0 - 0.03 * t) if z0 - 0.03 * t > SEAT_Z + 0.030 else max(SEAT_Z + 0.003, SEAT_Z + 0.030 - 0.010 * (t - (z0 - SEAT_Z - 0.030) / 0.03))
        return np.array([dk[0], dk[1], z])

    yield from R.servo(goal, timeout=14.0, read=lambda: _port(ctx), tol=0.0006, label="lower")
    print(f"[sw] lowered: {TRIAL.last}", flush=True)
    if mode == "rest":
        yield from R.open(t=0.3, label="let go")
        yield from R.wait(2.0)
        return
    ok = yield from R.servo(np.array([dk[0], dk[1], SEAT_Z - 0.001]), timeout=5.0, read=lambda: _port(ctx), tol=0.0004, gain=0.1, label="push home")
    print(f"[sw] pushed: ok={ok} {TRIAL.last}", flush=True)
    yield from R.open(label="release")
    yield from R.wait(0.5)
    yield from R.move(R.at() + [0.0, 0.0, 0.10], t=1.0, label="up")
    yield from R.wait(2.5)
    print(f"[sw] end {TRIAL.last}", flush=True)
