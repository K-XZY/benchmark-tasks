"""Dock a Nintendo Switch OLED: lower the console straight down its dock's slot until its USB-C port seats on the hidden plug at the slot's bottom.

The console (Nintendo: 102 x 242 x 13.9 mm with Joy-Con attached, 420 g) starts screen down on two fixed 150 mm rest blocks (a staging rest standing in for the hand or shelf the console comes from), one under each Joy-Con (the screen glass touches nothing), its top edge overhanging 30 mm toward the robot, port edge
toward the dock. The dock (ChargerLAB's measurement: 103.7 mm high, 174 mm long, 47.4 mm deep, 286 g) stands upright 16 cm toward the robot from the console's middle, its length along y: a slot 16 mm wide and
45 mm deep opens in its top along its length, 9 mm behind its front face; at the middle of the slot's floor stands a USB-C plug, 8.25 x 2.4 mm and 6.5 mm tall, with a chamfered tip. The console's port is a pocket in its lower edge, 9.25 x 3.4 mm and 6.6 mm deep (0.5 mm to spare each side).
Geometry is real: the plug goes into the pocket or it does not. Code: the plug's resistance (an 8 N friction along the console's axis while the plug is in the port, read and set every physics step, inside the USB Type-C mating range of 5 to 20 N) and the scorer.

Debug:  python demokit.py task.py OUT --no-record --mode full,none,drop,rest,drop10,redrop,crooked,beside,pastleft,pastright,undo
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
INSTRUCTION = "Lower the console into its dock, its screen facing the way the dock's front faces, so it connects, then let go."
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
SCR_W, SCR_H = 0.1549, 0.0872  # the 7-inch 16:9 screen (Nintendo: 7.0 in OLED): 154.9 x 87.2 mm, so a 7.4 mm bezel along the top and lower edges

# -- the dock, standing upright, at ChargerLAB's caliper measurement of the OLED model's dock (103.69 mm high, 17.4 cm long, 47.43 mm deep, 286 g; Nintendo rounds it to about 4.1 x 6.9 x 2.0 in,
#    0.69 lb). Frame: origin at the middle of its footprint on the bench; x depth (the front face toward the robot, at -x), y length, z up.
DX, DY, DZ = 0.0474, 0.174, 0.1037
SLOT_W, SLOT_DEPTH, FRONT = 0.016, 0.045, 0.009  # the slot: 16 mm wide (the console is 13.9 mm), 45 mm deep from the top, behind a 9 mm front wall (estimates: no published figure; real range in the README)
FLOOR = DZ - SLOT_DEPTH  # the slot's floor, 58.7 mm above the bench
SLOT_X = -DX / 2 + FRONT + SLOT_W / 2  # the slot's middle across the dock's depth: 6.7 mm in front of the dock's middle
PLUG_W, PLUG_T, PLUG_H = 0.00825, 0.0024, 0.0065  # USB Type-C plug (the receptacle it enters is 8.34 x 2.56 mm, 6.2 mm deep)
M_DOCK = 0.286

# -- the layout (metres, station frame)
DOCK_XY = (float(os.environ.get('SW_DX', '0.30')), -0.10)
CON_X, CON_Y = float(os.environ.get('SW_CX', '0.46')), -0.10
REST_H = float(os.environ.get('SW_RH', '0.15'))  # high enough that the hands reach the console's top edge over the upright dock
REST_X = (CON_X - 0.021, CON_X + 0.045)  # the console's top edge is 51 mm toward the robot from its middle: it overhangs the rest by 30 mm
REST_Y = (0.088, 0.118)  # the rest is two blocks, one under each Joy-Con (y from the middle, both sides): the screen face touches nothing

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
    parts = [("body", _bx(-hx, hx, -hy + JC, hy - JC, -hz, hz), "body"), ("screen", _bx(-hx - 0.0004, -hx, -SCR_W / 2, SCR_W / 2, -SCR_H / 2, SCR_H / 2), "screen"),
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
    big = np.array([(t, -w), (t, w), (-t, w), (-t, -w)]) + [SLOT_X, 0.0]
    small = np.array([(t - 0.0006, -w + 0.0008), (t - 0.0006, w - 0.0008), (-t + 0.0006, w - 0.0008), (-t + 0.0006, -w + 0.0008)]) + [SLOT_X, 0.0]
    return m.join(m.prism(big, FLOOR, FLOOR + PLUG_H - 0.0015), m.prism(small, FLOOR + PLUG_H - 0.0015, FLOOR + PLUG_H))


def _dock() -> m.Part:
    hw = SLOT_W / 2
    wall = [(-DX / 2, SLOT_X - hw, -DY / 2, DY / 2, FLOOR, DZ), (SLOT_X + hw, DX / 2, -DY / 2, DY / 2, FLOOR, DZ)]  # the front wall (9 mm) and the back (26 mm)
    floor = (-DX / 2, DX / 2, -DY / 2, DY / 2, 0.0, FLOOR)
    parts = [("floor", _bx(*floor), "dock"), ("w0", _bx(*wall[0]), "dock"), ("w1", _bx(*wall[1]), "dock"), ("plug", _plug_mesh(), "plug"), ("slotfloor", _bx(SLOT_X - hw, SLOT_X + hw, -DY / 2, DY / 2, FLOOR, FLOOR + 0.0002), "slot")]
    cols = [_cb("cf", *floor), _cb("cw0", *wall[0]), _cb("cw1", *wall[1]), m.col_hull("plug_c", _plug_mesh())]
    return m.free("dock", (DOCK_XY[0], DOCK_XY[1], TOP + 0.0002), parts, cols, LOOKS, mass=M_DOCK, friction=(0.4, 0.4), torsional_patch=None, solver_iters=64)


def _rest() -> m.Part:
    x0, x1 = REST_X
    bs = [(x0, x1, CON_Y + sg * REST_Y[0], CON_Y + sg * REST_Y[1], 0.0, REST_H) for sg in (-1, 1)]
    bs = [(b[0], b[1], min(b[2], b[3]), max(b[2], b[3]), b[4], b[5]) for b in bs]
    return m.fixed("rest", (0.0, 0.0, TOP), [(f"rest{i}", _bx(*b), "rest") for i, b in enumerate(bs)], [_cb(f"c{i}", *b) for i, b in enumerate(bs)], LOOKS, friction=(0.4, 0.4))


OBJECTS = [_rest(), _dock(), _console()]

DETENT_N = float(os.environ.get("SW_DETENT", "8.0"))  # the plug's resistance while it is in the port: inside the USB Type-C mating range (insertion 5 to 20 N, extraction 8 to 20 N); SW_DETENT for the range's ends
G = 9.81
CLEAR_OFF = 0.004  # a console lifted this far off the plug's tip is judged afresh on its next arrival (a fall from 4 mm arrives at 0.28 m/s, over SLAM_V)
SEATED_DEPTH = PORT_D - 0.0004  # plug in to within 0.4 mm of the pocket's roof
SLAM_V = 0.25  # a console arriving faster than this at the plug is dropped in, not lowered (a real plug's shell could be damaged; estimate)
TILT_SEAT, TILT_REST, TILT_JAM = 2.0, 5.0, 15.0  # degrees: seated, upright in the slot, jammed in the slot (a console leaning further is not in the dock)


def size_check() -> str:
    """The scene's key sizes against the published ones (python -c "import task; print(task.size_check())"). Published: Nintendo's 102 x 242 x 13.9 mm and 420 g for the console; ChargerLAB's measured 103.69 x 174 x 47.43 mm and 286 g, and Nintendo's about 4.1 x 6.9 x 2.0 in and 0.69 lb, for the dock; a 7-inch 16:9 screen; USB Type-C plug 8.25 x 2.4 mm;
    the USB Type-C mating force range, 5 to 20 N (the seating row shows the scene's value against the range's ends); the slot against the console's 13.9 mm (the slot has no published width)."""
    rows = [("console height x width x thickness, mm", (CH * 1e3, CW * 1e3, CT * 1e3), (102.0, 242.0, 13.9)), ("console mass, g", (M_CON * 1e3,), (420.0,)),
            ("dock height x length x depth, mm (ChargerLAB, measured)", (DZ * 1e3, DY * 1e3, DX * 1e3), (103.69, 174.0, 47.43)), ("dock mass, g (ChargerLAB, weighed)", (M_DOCK * 1e3,), (286.0,)),
            ("dock height x length x depth, mm (Nintendo, about)", (DZ * 1e3, DY * 1e3, DX * 1e3), (4.1 * 25.4, 6.9 * 25.4, 2.0 * 25.4)), ("dock mass, g (Nintendo, about)", (M_DOCK * 1e3,), (0.69 * 453.6,)),
            ("screen diagonal, in", (math.hypot(SCR_W, SCR_H) / 0.0254,), (7.0,)), ("plug width x thickness, mm", (PLUG_W * 1e3, PLUG_T * 1e3), (8.25, 2.4)),
            ("seating resistance, N", (DETENT_N,), (5.0, 20.0)), ("slot width against the console's thickness, mm", (SLOT_W * 1e3,), (CT * 1e3,))]
    return "\n".join(f"{n}: scene {tuple(round(v, 1) for v in a)}, published {tuple(round(v, 1) for v in b)}" for n, a, b in rows)


def _geom(env):
    con, dock = m.state(env, "console"), m.state(env, "dock")
    R = con.rotation()
    Rr = dock.rotation().T @ R  # tilt, turn and facing are read against the dock's own frame, so a dock nudged round on its mark keeps its standard
    tip = R.T @ (dock.to_world(np.array([[SLOT_X, 0.0, FLOOR + PLUG_H]]))[0] - con.pos)  # the plug's tip in the console's frame
    depth = tip[2] + CH / 2  # how far the tip stands above the lower edge: the plug's travel into the port
    aligned = bool(abs(tip[1]) < PORT_Y / 2 and abs(tip[0]) < PORT_X / 2)
    tilt = math.degrees(math.acos(float(np.clip(Rr[2, 2], -1.0, 1.0))))
    yaw = math.degrees(math.atan2(Rr[1, 0], Rr[0, 0]))
    facing = bool(Rr[0, 0] > 0.0)  # the screen the way the dock's front faces: toward the robot
    corners = con.to_world(np.array([[sx * CT / 2, sy * CW / 2, -CH / 2] for sx in (-1, 1) for sy in (-1, 1)]))
    return con, dock, R, tip, depth, aligned, tilt, yaw, facing, corners


def _arm_points(env) -> np.ndarray:
    """Points on both arms besides the fingers (station frame): every link's origin, and 9 points through each gripper's housing above the fingers."""
    base = env.unwrapped
    origin = m._np(base.scene.env_origins[0])
    out = []
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        poses = m._np(robot.data.body_link_pose_w)[0]
        out += [p[:3] - origin for p in poses]
        g = poses[robot.find_bodies("gripper")[0][0]]
        Rg = m.quat_matrix(g[3:7])
        out += [g[:3] - origin + Rg @ np.array([0.0, dy, dz]) for dy in (-0.03, 0.0, 0.03) for dz in (0.0, 0.045, 0.09)]
    return np.array(out)


def _hook_physics(env) -> None:
    """Run TRIAL.substep before every physics step (240 Hz), not only every 50 ms control step: the plug's resistance and the arrival speed are read where they happen."""
    scene = env.unwrapped.scene
    if getattr(scene, "_sw_substep", False):
        return
    write = scene.write_data_to_sim

    def write_with_plug(*a, **k):
        TRIAL.substep(env)
        return write(*a, **k)

    scene.write_data_to_sim = write_with_plug
    scene._sw_substep = True


class Trial:
    def reset(self, env=None) -> None:
        self.n, self.held = 0, 0
        self.dead, self.score, self.full = False, 0.0, False
        self.d0 = None
        self.impact = 0.0
        self.prev = None  # the last physics step's (depth, inward speed, resistance)
        self.screen_steps = 0
        self.last = {}
        if env is not None:
            _hook_physics(env)
            self.step(env)

    def substep(self, env) -> None:
        """Every physics step, with no memory beyond the last physics step. The plug's resistance: whenever the plug is in the port (its tip past the port's mouth), it acts as friction
        along the console's axis, up to DETENT_N either way, home or not: it stops the console when the other forces on it (its weight along the axis, the hands, the walls) are under
        DETENT_N, and takes DETENT_N off them when they are over it, going in or coming out. The other forces are read from the last physics step's change of speed. The arrival speed:
        the console's speed into the plug at the step its lower edge reaches the plug's tip, under the port or anywhere beside it along the edge. Only a console lifted CLEAR_OFF
        off the plug is judged afresh on its next arrival."""
        con, dock = m.state(env, "console"), m.state(env, "dock")
        R = con.rotation()
        Rr = dock.rotation().T @ R  # the console's turn against the dock
        tip = R.T @ (dock.to_world(np.array([[SLOT_X, 0.0, FLOOR + PLUG_H]]))[0] - con.pos)
        depth = tip[2] + CH / 2
        aligned = bool(abs(tip[1]) < PORT_Y / 2 and abs(tip[0]) < PORT_X / 2)
        under = bool(abs(tip[1]) < CW / 2 and abs(tip[0]) < CT / 2)  # the plug's tip anywhere under the console's lower edge, at the port or beside it
        tilt = math.degrees(math.acos(float(np.clip(Rr[2, 2], -1.0, 1.0))))
        v_in = float(-(con.lin_vel @ R[:, 2]))  # speed into the plug, down the console's own axis
        dt = float(getattr(env.unwrapped, "physics_dt", 1.0 / 240.0))
        if depth < -CLEAR_OFF:
            self.impact = 0.0
        elif self.prev is not None and under and self.prev[0] < -0.0005 <= depth:
            self.impact = max(self.impact, v_in, self.prev[1])
        f = 0.0
        if aligned and depth >= 0.0 and tilt < 8.0:
            w = M_CON * G * max(0.0, float(R[2, 2]))
            other = w if self.prev is None or self.prev[2] is None else M_CON * (v_in - self.prev[1]) / dt + self.prev[2]  # the other forces into the plug, from the last step: m dv/dt = other - f
            f = float(np.clip(other + M_CON * v_in / dt, -DETENT_N, DETENT_N))
            m.push(env, "console", R[:, 2] * f)  # along the console's axis, out of the plug: the resistance
            m.push(env, "dock", -R[:, 2] * f)  # and its reaction on the dock, through the plug
            self.prev = (depth, v_in, f)
        else:
            m.push(env, "console", (0.0, 0.0, 0.0))
            m.push(env, "dock", (0.0, 0.0, 0.0))
            self.prev = (depth, v_in, None)

    def step(self, env) -> None:
        con, dock, R, tip, depth, aligned, tilt, yaw, facing, corners = _geom(env)
        if self.d0 is None:
            self.d0 = dock.pos.copy()
        # the plug's resistance and the arrival speed are read every physics step (substep); seated is the state now: the port within 0.7 mm of its stop
        seated = bool(aligned and depth >= SEATED_DEPTH - 0.0003 and tilt <= TILT_SEAT and abs(yaw) <= TILT_SEAT and facing)
        low = float(corners[:, 2].min())
        # in the slot: the console's lower edge below the dock's rim AND its connector edge over the dock's opening: the middle of the lower edge (where the port is) inside the opening's width (the slot, 16 mm) and
        # within 34 mm of the dock's middle along its length (the 242 mm console then spans the 174 mm dock): a console beside the dock, off to one end or past either end is never in the slot
        edge = dock.to_local(con.to_world(np.array([[0.0, 0.0, -CH / 2]])))[0]
        over_opening = bool(abs(edge[0] - SLOT_X) <= SLOT_W / 2 and abs(edge[1]) <= (CW - DY) / 2)  # along: within 34 mm of the dock's middle, so the console spans the whole dock, both Joy-Con past its ends
        in_slot = bool(edge[2] < DZ - 0.004 and low < dock.pos[2] + DZ - 0.004 and over_opening)  # the lower edge's middle and its lowest corner both 4 mm below the rim
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
        elif in_slot and tilt <= TILT_JAM:
            tier = 0.25
        else:
            tier = 0.0
        tips = m.fingertips(env)
        tl = con.to_local(tips)
        q = np.abs(con.to_local(np.vstack([tips, _arm_points(env)]))) - np.array([CT / 2, CW / 2, CH / 2])
        clear = bool(np.linalg.norm(np.maximum(q, 0.0), axis=1).min() >= 0.03)  # every finger point, each gripper's housing and every arm link 3 cm from the console
        # screen guard (checked, not scored): any of the 24 points on the fingers (the pads) over the screen's glass, at its face or beyond it
        on_screen = (np.abs(tl[:, 1]) < SCR_W / 2) & (np.abs(tl[:, 2]) < SCR_H / 2) & (tl[:, 0] < -CT / 2 + 0.002) & (tl[:, 0] > -CT / 2 - 0.007)
        self.screen_steps += int(on_screen.any())
        if on_screen.any() and self.screen_steps in (1, 60) and os.environ.get("SW_TRACE_SCREEN"):
            print(f"[sw] screen step {self.n}: points {np.round(tl[on_screen] * 1000, 1).tolist()}", flush=True)
        still = all(np.linalg.norm(x.lin_vel) < 0.005 and np.linalg.norm(x.ang_vel) < 0.2 for x in (con, dock))
        self.held = self.held + 1 if (clear and still) else 0
        # every tier is read from the state now, held still and clear for 1 s; a run not settled at its end scores 0. Only failures stick (self.dead: the console fallen, the dock moved or tipped)
        self.score = 0.0 if self.dead else (tier if self.held >= HOLD_STEPS else 0.0)
        self.full = bool(self.held >= HOLD_STEPS and tier == 1.0)
        self.n += 1
        self.last = dict(depth_mm=round(depth * 1000, 2), aligned=aligned, tip_xy_mm=(round(tip[0] * 1000, 2), round(tip[1] * 1000, 2)), tilt=round(tilt, 2), yaw=round(yaw, 2), seated=seated,
                         impact=round(self.impact, 3), in_slot=in_slot, over_opening=over_opening, screen_steps=self.screen_steps, upright=upright, kept=kept, tier=tier, clear=clear, still=still, held=self.held)
        if os.environ.get("SW_TRACE") and (self.n % 20 == 0 or (os.environ["SW_TRACE"] == "2" and int(os.environ.get("SW_TRACE_FROM", "540")) <= self.n <= int(os.environ.get("SW_TRACE_FROM", "540")) + 60)):
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
GY = float(os.environ.get("SW_GY", "0.104"))  # each hand's offset from the console's middle along its width: on a Joy-Con's end of the top edge (the screen's glass ends at 77 mm), off the screen
STAND_Z = float(os.environ.get("SW_STAND", "0.21"))  # the TCPs' height over the dock when the console is stood up: its lower edge 19 mm over the dock's top
TILT = float(os.environ.get("SW_TILT", "30"))  # each grip turned this far outward about the console's thickness: the fingers point out past the console's end, the pads away from the screen
GAP = float(os.environ.get("SW_GAP", "0.034"))  # the jaws' opening on the way in: under the 30 mm overhang


def _g0(yaw):
    """Fingers level pointing +x, jaws closing up and down: a down grip with its jaws along x (right arm +90, left arm -90), pitched so its fingers point +x."""
    base = d.down(yaw)
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


def _both(ctx, make):
    """Run one primitive on both hands in lockstep: make(arm) gives its generator."""
    yield from ctx.together(make(ctx.right), make(ctx.left))


def _level(ctx):
    """Level the held console by feedback: turn both hands about the console's middle to bring its up axis vertical and its thickness along x."""
    for _ in range(2):
        R = ctx.state("console").rotation()
        z = R[:, 2]
        tilt = math.degrees(math.acos(float(np.clip(z[2], -1, 1))))
        if tilt > 0.7:
            n = _unit(np.cross(z, [0.0, 0.0, 1.0]))
            yield from _both(ctx, lambda a: a.arc(_con(ctx), n, tilt, t=0.6, label="level"))
        R = ctx.state("console").rotation()
        yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
        if abs(yaw) > 0.7:
            yield from _both(ctx, lambda a: a.arc(_con(ctx), (0.0, 0.0, 1.0), -yaw, t=0.6, label="yaw"))
    R = ctx.state("console").rotation()
    print(f"[sw] leveled: tilt {math.degrees(math.acos(float(np.clip(R[2, 2], -1, 1)))):.2f} deg, yaw {math.degrees(math.atan2(R[1, 0], R[0, 0])):.2f} deg", flush=True)


def _steer(ctx, goal, timeout, tol, label, gain=0.1):
    """Both hands steer the console's port to `goal` (a point or a function of time); each takes half the correction."""
    yield from _both(ctx, lambda a: a.servo(goal, timeout=timeout, read=lambda: _port(ctx), tol=tol, gain=gain, label=label))


def _frames(ctx):
    """The console's connector edge against the dock's plug, from both observed poses: E, the middle of the console's lower edge (the port's mouth); P, the plug's tip; Rd, the dock's axes
    (x across the slot, y along it, z the slot's axis); e, E - P in the dock's frame (across, along, height above the plug's tip); ang, the console's turn from the slot's plane in radians (roll about
    the slot's across axis, which tilts the width; pitch about its length; yaw about its axis)."""
    env = ctx.emb._env
    con, dock = m.state(env, "console"), m.state(env, "dock")
    Rc, Rd = con.rotation(), dock.rotation()
    E = con.pos + Rc @ np.array([0.0, 0.0, -CH / 2])
    P = dock.pos + Rd @ np.array([SLOT_X, 0.0, FLOOR + PLUG_H])
    e = Rd.T @ (E - P)
    yc, zc = Rd.T @ Rc[:, 1], Rd.T @ Rc[:, 2]
    ang = np.array([math.atan2(-zc[1], zc[2]), math.atan2(zc[0], zc[2]), math.atan2(-yc[0], yc[1])])
    return E, P, Rd, e, ang


ALIGN_TOL = 0.0004  # the port's mouth over the plug to 0.4 mm across and along the slot: the port pocket clears the plug by 0.5 mm a side
ALIGN_DEG = 0.3  # and square to the slot to 0.3 degrees: at 0.3 degrees of yaw the console's ends move 0.63 mm across the slot, inside its 1.05 mm clearance a side


def _hold_at(ctx, h, tol=ALIGN_TOL, tol_h=0.001, tol_deg=ALIGN_DEG, timeout=1.0, gain=0.15, turn=True, until=None, label="hold"):
    """Both hands, closed loop on the observed poses: each step, read the console's connector edge against the plug, move both TCPs by a share of the error (the edge to `h` above the plug's tip
    along the slot's axis, centred across and along it) and turn both grips together about the edge by a share of the console's turn from the slot's plane. Ends once the edge has been within `tol`
    across and along, `tol_h` in height and `tol_deg` square for 3 steps, or `until()` holds, or after `timeout` s. Returns whether it ended within tolerance."""
    arms = (ctx.right, ctx.left)
    cs = [d.ik.tcp_pose(a.q, a.base)[0] for a in arms]
    good, worst = 0, [(0.0, 0.0), (0.0, 0.0)]
    for _ in range(max(1, round(timeout * d.CONTROL_HZ))):
        E, P, Rd, e, ang = _frames(ctx)
        err = P + Rd[:, 2] * h - E
        el = Rd.T @ err
        ok = abs(el[0]) < tol and abs(el[1]) < tol and abs(el[2]) < tol_h and (not turn or float(np.abs(ang).max()) < math.radians(tol_deg))
        good = good + 1 if ok else 0
        if good >= 3 or (until is not None and until(e)):
            for a, w in zip(arms, worst):
                a._note(*w, label)
            return True
        dp = np.clip(gain * err, -0.002, 0.002)
        rv = Rd @ (-ang) * 0.3 if turn else np.zeros(3)
        n = float(np.linalg.norm(rv))
        if n > math.radians(0.5):
            rv, n = rv * math.radians(0.5) / n, math.radians(0.5)
        Rk = d.axis_angle_matrix(rv / n, n) if n > 1e-7 else np.eye(3)
        for i, a in enumerate(arms):
            cs[i] = E + Rk @ (cs[i] - E) + dp
            rot = Rk @ a.grip.rot
            a.grip = d._as_grip(rot)
            mm, deg = a._solve(cs[i], rot)
            worst[i] = (max(worst[i][0], mm), max(worst[i][1], deg))
        yield
    for a, w in zip(arms, worst):
        a._note(*w, label)
    E, P, Rd, e, ang = _frames(ctx)
    print(f"[sw] '{label}' at {h * 1000:.1f} mm: ended outside tolerance, edge {np.round(e * 1000, 2).tolist()} mm, turn {np.round(np.degrees(ang), 2).tolist()} deg", flush=True)
    return False


def _seat(ctx, crook=0.0):
    """Align the connector edge over the plug at the hover height, then lower it along the slot's axis in short steps (4 mm above the slot's rim, 3 mm inside it, then 1 mm over the last 8 mm),
    correcting the edge's place and the console's squareness at each step, then push the port home onto the plug. `crook` (the crooked check): after the alignment, roll the console that many
    degrees across its width and lower it so, correcting its place but not its roll."""
    turn = True
    h = float(_frames(ctx)[3][2])
    for _ in range(3):
        if (yield from _hold_at(ctx, h, tol=ALIGN_TOL * 0.75, tol_deg=ALIGN_DEG * 0.75, timeout=2.0, turn=turn, label="align over the opening")):
            break
    E, P, Rd, e, ang = _frames(ctx)
    print(f"[sw] aligned over the opening: edge {np.round(e * 1000, 2).tolist()} mm, turn {np.round(np.degrees(ang), 2).tolist()} deg, dock yaw {math.degrees(math.atan2(Rd[1, 0], Rd[0, 0])):.2f} deg", flush=True)
    if crook:
        yield from _both(ctx, lambda a: a.arc(E, Rd[:, 0], crook, t=0.6, label="crook"))
        turn = False
    rim = DZ - FLOOR - PLUG_H  # the slot's rim, above the plug's tip
    stop = 0.010 if ctx.mode == "drop10" else 0.0012  # drop10: let go aligned 10 mm above the plug
    while h > stop:
        h = max(stop, h - (0.004 if h > rim + 0.004 else 0.003 if h > 0.008 else 0.001))
        yield from _hold_at(ctx, h, timeout=1.2, turn=turn, label="lower")
    E, P, Rd, e, ang = _frames(ctx)
    print(f"[sw] lowered: edge {np.round(e * 1000, 2).tolist()} mm, turn {np.round(np.degrees(ang), 2).tolist()} deg, {TRIAL.last}", flush=True)
    if ctx.mode in ("rest", "drop10"):
        return
    home = -PLUG_H + 0.0003  # the port's floor on the plug's tip: the edge 6.2 mm below the tip
    yield from _hold_at(ctx, -PLUG_H - 0.001, tol_h=0.0002, timeout=6.0, gain=0.1, turn=turn, until=lambda e: e[2] <= home, label="push home")
    yield from ctx.right.wait(0.3)
    print(f"[sw] pushed: {TRIAL.last}", flush=True)


def _lazy_beside(ctx, dk, off):
    """The lazy routes that skip the dock: stand the console beside it on the bench, or with its port past one end of the slot, lower it there and let go."""
    tgt = np.array([dk[0] + off[0], dk[1] + off[1], _con(ctx)[2]])
    yield from _steer(ctx, tgt, 3.0, 0.001, "over the bench")
    yield from _level(ctx)
    floor = TOP + CH / 2 if ctx.mode == "beside" else SEAT_Z  # beside: the bench; past an end: the console's lower edge goes down into the slot's floor, its port off the plug
    yield from _both(ctx, lambda a: a.servo(np.array([tgt[0], tgt[1], floor + 0.002]), timeout=8.0, read=lambda: _con(ctx), tol=0.002, label="to the bench"))
    yield from _both(ctx, lambda a: a.open(t=0.3, label="let go"))
    yield from _both(ctx, lambda a: a.move(a.at() + [0.0, 0.0, 0.10], t=1.0, label="up"))
    yield from ctx.right.wait(2.5)
    print(f"[sw] end {TRIAL.last}", flush=True)


def demo_script(ctx):
    """Both hands, one on each Joy-Con end of the top edge (the screen's glass is never touched). Modes: full. none: the arms wait. drop: let go over the plug from the hover height, the port 64 mm above it. rest: let go 1.2 mm above the plug. drop10: let go 10 mm above it. redrop: seated, lifted 3 cm off the plug and dropped back in. undo: the drop, settled 2 s, then gripped again and lifted 10 cm back out.
    crooked: lowered with a 10 degree roll across its width. beside: set down upright on the bench 8 cm beyond the dock (away from the robot). pastleft, pastright: lowered at the dock's x with the console's
    port 3 cm past the dock's left or right end, into the slot there."""
    mode = ctx.mode
    if mode not in ("full", "none", "drop", "rest", "drop10", "redrop", "crooked", "beside", "pastleft", "pastright", "undo"):
        raise ValueError(f"unknown mode {mode!r}")
    R, L = ctx.right, ctx.left
    if mode == "none":
        yield from R.wait(3.0)
        return
    # the grip points from the console's observed pose: on its mid-thickness, EDGE_IN in from the top edge, GY either side of the middle; both grips turned with the console's yaw on the rest
    st = m.state(ctx.emb._env, "console")
    Rc = st.rotation()
    yaw_c = math.atan2(-Rc[0, 1], Rc[1, 1])
    Rz = d.axis_angle_matrix(np.array([0.0, 0.0, 1.0]), yaw_c)
    Rt = [d.axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.radians(sg * TILT)) for sg in (1.0, -1.0)]  # right: fingers toward +y, left: toward -y (both outward when stood up)
    gr, gl = d._as_grip(Rz @ Rt[0] @ _g0(90.0).rot), d._as_grip(Rz @ Rt[1] @ _g0(-90.0).rot)
    tr, tl = (st.pos + Rc @ np.array([0.0, sy * GY, CH / 2 - EDGE_IN]) for sy in (-1.0, 1.0))
    print(f"[sw] console on the rest: {np.round(st.pos, 4).tolist()}, yaw {math.degrees(yaw_c):.2f} deg", flush=True)
    br, bl = (Rz @ R_ @ np.array([-0.10, 0.0, 0.0]) for R_ in Rt)  # 10 cm back along each hand's fingers
    yield from ctx.together(R.move(tr + br, grip=gr, finger=-GAP / 2, label="approach"), L.move(tl + bl, grip=gl, finger=-GAP / 2, label="approach"))
    yield from ctx.together(R.servo(tr + br, timeout=1.0, label="approach"), L.servo(tl + bl, timeout=1.0, label="approach"))
    yield from ctx.together(R.move(tr, t=1.2, label="in"), L.move(tl, t=1.2, label="in"))
    yield from ctx.together(R.servo(tr, timeout=1.0, label="in"), L.servo(tl, timeout=1.0, label="in"))
    yield from _both(ctx, lambda a: a.close(CT, GRIP_N, label="grip the top edge"))
    yield from _both(ctx, lambda a: a.move(a.at() + [0.0, 0.0, LIFT], t=1.2, label="lift"))
    dk = ctx.pos("dock")
    # carry back level to over the dock, high enough that the hanging console clears it, then stand it up there
    hz_ = TOP + STAND_Z
    sx = dk[0] + SLOT_X  # over the slot
    yield from ctx.together(R.move(np.array([sx, dk[1] - GY, hz_]), t=2.5, label="carry"), L.move(np.array([sx, dk[1] + GY, hz_]), t=2.5, label="carry"))
    yield from ctx.together(R.servo(np.array([sx, dk[1] - GY, hz_]), timeout=1.5, label="carry"), L.servo(np.array([sx, dk[1] + GY, hz_]), timeout=1.5, label="carry"))
    yield from _both(ctx, lambda a: a.turn(90.0, axis=(0.0, 1.0, 0.0), t=2.0, label="stand up"))
    yield from R.wait(0.4)
    print(f"[sw] stood: console {np.round(_con(ctx), 3)}, {TRIAL.last}", flush=True)
    off = {"beside": (0.08, 0.0), "pastleft": (SLOT_X, DY / 2 + 0.03), "pastright": (SLOT_X, -DY / 2 - 0.03)}.get(mode)
    if off is not None:
        yield from _lazy_beside(ctx, dk, off)
        return
    if mode == "crooked":
        yield from _seat(ctx, crook=float(os.environ.get("SW_CROOK", "10")))
    elif mode in ("drop", "undo"):
        yield from _hold_at(ctx, float(_frames(ctx)[3][2]), timeout=2.0, label="over the opening")
        print(f"[sw] drop from: {TRIAL.last}", flush=True)
        z_held = float(_con(ctx)[2])
        yield from _both(ctx, lambda a: a.open(t=0.3, label="drop"))
        yield from R.wait(2.0)
        if mode == "undo":  # the dropped-in console has settled at its tier: grip it again where the hands let go, lift it 10 cm back out of the slot and hold it there to the end
            print(f"[sw] settled before the undo: score {TRIAL.score}, {TRIAL.last}", flush=True)
            fall = z_held - float(_con(ctx)[2])
            yield from _both(ctx, lambda a: a.move(a.at() - [0.0, 0.0, fall], t=1.5, label="down to the top edge"))
            yield from _both(ctx, lambda a: a.close(CT, GRIP_N, label="grip again"))
            yield from _both(ctx, lambda a: a.move(a.at() + [0.0, 0.0, 0.10], t=1.5, label="lift out"))
            yield from R.wait(3.0)
            print(f"[sw] end {TRIAL.last}, score {TRIAL.score}", flush=True)
        return
    else:
        yield from _seat(ctx)
    if mode in ("rest", "drop10"):
        yield from _both(ctx, lambda a: a.open(t=0.3, label="let go"))
        yield from R.wait(1.0)
        yield from _both(ctx, lambda a: a.move(a.at() + [0.0, 0.0, 0.10], t=1.0, label="up"))
        yield from R.wait(2.5)
        print(f"[sw] end {TRIAL.last}", flush=True)
        return
    if mode == "redrop":  # seated gently, then lifted 3 cm (SW_REDROP, metres) in the hands and dropped back in
        print(f"[sw] seated before the redrop: {TRIAL.last}", flush=True)
        up = float(os.environ.get("SW_REDROP", "0.03"))
        yield from _both(ctx, lambda a: a.move(a.at() + [0.0, 0.0, up], t=1.0, label="lift off the plug"))
        yield from R.wait(0.5)
        yield from _both(ctx, lambda a: a.open(t=0.3, label="drop"))
        yield from R.wait(1.0)
        yield from _both(ctx, lambda a: a.move(a.at() + [0.0, 0.0, 0.10], t=1.0, label="up"))
        yield from R.wait(2.5)
        print(f"[sw] end {TRIAL.last}", flush=True)
        return
    yield from _both(ctx, lambda a: a.open(label="release"))
    yield from R.wait(0.5)
    yield from _both(ctx, lambda a: a.move(a.at() + [0.0, 0.0, 0.10], t=1.0, label="up"))
    yield from R.wait(2.5)
    print(f"[sw] end {TRIAL.last}", flush=True)
