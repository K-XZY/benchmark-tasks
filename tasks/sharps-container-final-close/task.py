"""Close a full 1.5 qt phlebotomy sharps container for pickup: one hand holds it by its moulded side handle while the other swings the hinged clear cap
over and presses it down, through a light first click (temporary closure) to a firm second click (final lock); then the holding hand lifts it by the
handle and sets it upright in the open pickup carton beside it.

The container (127 x 145 mm footprint, 229 mm tall, 0.5 kg filled to its fill line) stands on the counter, cap open and leaning towards the robot on its
hinge along the front top edge. Under the cap's back edge sit two sprung catches (one near each side). The cap meets them when it is swung shut; pressed
2 mm down (about 1.4 N per catch, 3 N at the cap) they give the temporary click; pressed 6 mm down (about 4.2 N per catch, 9.4 N at the cap) they give the
final lock, which then holds the cap shut for good. The catches, the hinge and every contact are bodies, joints and collisions; the code stands in only
for the snap of the final lock (`Trial.step`: once both catches are 6 mm down with the cap shut, the hinge is held where it is) and the scorer.

Debug:  python demokit.py task.py OUT --no-record --mode full,none,light_press,no_press,free_press,drop_high,drop_low,drop_2cm,cap_fall,held_fast_lower
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
INSTRUCTION = "This sharps container is full. Close its cap and press it down until it locks for good, then put the container upright in the pickup box."
SUCCESS_HOLD_S = 0.0  # the scorer holds the end state itself (HOLD_STEPS)
SCORE_OVER = "end"
TOP = d.TABLE_TOP
HOLD_STEPS = 20  # 1 s at 20 Hz
HELD_MM = 3.0  # a fall is not a drop only when a gripper's fingertips are within 3 mm of the container and the gripper falls with it
HOLD_SLACK = 0.005  # the gripper may fall up to 5 mm less than the container and still hold it
DROP3_MM = 6.0  # or more than 6 mm in three steps (0.04 m/s)
DROP_MM = 4.0  # a fall of more than 4 mm in one 0.05 s step (0.08 m/s) that no gripper goes down with counts as a drop; the demo lowers at about 3 mm per step, held
ENV = os.environ.get

# -- the container, in its own frame: origin at its footprint's centre on the counter, x away from the robot, y to the robot's left
CX, CY = float(ENV("SC_X", "0.36")), float(ENV("SC_Y", "-0.03"))
HX, HY = 0.0725, 0.0635  # half depth (145 mm) and half width (127 mm)
T = 0.003  # wall
RED_TOP, WALL_TOP, RIM_TOP = 0.150, 0.204, 0.2145  # red body, clear collar (collider top), the collar's visual rim
CAP_Z0, CAP_Z1 = 0.2155, 0.229  # the cap's underside and top when shut (229 mm tall)
HXC = -HX - 0.002  # the hinge line, along y at the cap's front underside edge
OPEN = float(ENV("SC_OPEN", "100.0"))  # the cap at joint zero: 100 degrees open, leaning 10 degrees towards the robot onto its stop
CATCH_X, CATCH_Y = 0.063, 0.040  # the two catches: under the cap's back edge, one near each side
CATCH_K = float(ENV("SC_K", "700.0"))  # N/m each
TEMP_MM, FINAL_MM, TRAVEL_MM = 2.0, 6.0, 8.0
LEVER = CATCH_X - HXC  # from the hinge to the catches
H_Y0, H_Y1, H_Z0, H_Z1, H_X = HY + 0.032, HY + 0.046, 0.136, 0.154, 0.040  # the side handle's bar: 14 mm across, 18 mm tall, 80 mm long, 32 mm off the wall
MASS = 0.5

# -- the pickup carton (open, cardboard): its centre, outer size, wall height
BX, BY = float(ENV("SC_BX", "0.38")), float(ENV("SC_BY", "0.27"))
BL, BW, BH, BT = 0.30, 0.28, 0.12, 0.005

LOOKS = {"red": m.Look(m.rgb(0.75, 0.06, 0.05), metallic=0.0, roughness=0.45), "clear": m.Look(m.rgb(0.92, 0.94, 0.95), roughness=0.15, opacity=0.35),
         "cap": m.Look(m.rgb(0.90, 0.92, 0.93), roughness=0.2, opacity=0.55), "port": m.Look(m.rgb(0.35, 0.35, 0.37), roughness=0.5),
         "label": m.Look(m.rgb(0.95, 0.80, 0.10), roughness=0.6), "ink": m.Look(m.rgb(0.08, 0.08, 0.08), roughness=0.6),
         "barrel": m.Look(m.rgb(0.93, 0.93, 0.90), roughness=0.3), "plunger": m.Look(m.rgb(0.20, 0.25, 0.70), roughness=0.4), "hub": m.Look(m.rgb(0.95, 0.55, 0.10), roughness=0.4),
         "white": m.Look(m.rgb(0.96, 0.96, 0.96), roughness=0.5), "card": m.Look(m.rgb(0.62, 0.47, 0.30), roughness=0.9), "bio": m.Look(m.rgb(0.80, 0.10, 0.08), roughness=0.6)}


def _rot(x, z, x0, z0, deg):
    """(x, z) turned `deg` about the line along y through (x0, z0), right-hand rule about +y: negative lifts points on the +x side."""
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return x0 + (x - x0) * c + (z - z0) * s, z0 - (x - x0) * s + (z - z0) * c


def _bx(x0, x1, y0, y1, z0, z1):
    return m.box((x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def _about_y(mesh, deg, x0, z0):
    return m.Mesh(mesh.v - [x0, 0, z0], mesh.f).turned(deg, "y").moved([x0, 0, z0])


def _col(name, b):
    return m.col_box(name, (b[1] - b[0], b[3] - b[2], b[5] - b[4]), ((b[0] + b[1]) / 2, (b[2] + b[3]) / 2, (b[4] + b[5]) / 2))


def _col_turned(name, b, deg, x0, z0):
    xa, xb, ya, yb, za, zb = b
    cx, cz = _rot((xa + xb) / 2, (za + zb) / 2, x0, z0, deg)
    return f"""
    def Cube "{name}" {m._COL_HEAD}
        double size = 1
        double3 xformOp:translate = ({cx:.6f}, {(ya + yb) / 2:.6f}, {cz:.6f})
        float xformOp:rotateY = {deg:.4f}
        float3 xformOp:scale = ({xb - xa:.6f}, {yb - ya:.6f}, {zb - za:.6f})
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateY", "xformOp:scale"]
    }}"""


def _walls(x0, x1, y0, y1, z0, z1, t):
    return [(x0, x0 + t, y0, y1, z0, z1), (x1 - t, x1, y0, y1, z0, z1), (x0 + t, x1 - t, y0, y0 + t, z0, z1), (x0 + t, x1 - t, y1 - t, y1, z0, z1)]


def _syringes():
    """The contents seen through the clear collar: syringes and blood-collection holders standing and leaning up to the fill line."""
    out, rng = [], np.random.default_rng(3)
    for k in range(14):
        x, y = rng.uniform(-HX + 0.015, HX - 0.015), rng.uniform(-HY + 0.015, HY - 0.015)
        lean, yaw, h = rng.uniform(5, 35), rng.uniform(0, 360), rng.uniform(0.06, 0.09)
        z0 = 0.165 - h
        barrel = m.cylinder(0.0055, h, (0.0, 0.0, h / 2)).turned(lean, "x").turned(yaw, "z").moved([x, y, z0])
        hub = m.cylinder(0.004, 0.010, (0.0, 0.0, h + 0.005)).turned(lean, "x").turned(yaw, "z").moved([x, y, z0])
        out += [(f"syr{k}", barrel, "barrel"), (f"hub{k}", hub, "hub" if k % 3 else "plunger")]
    return out


def _container():
    red = [b for b in _walls(-HX, HX, -HY, HY, 0.0, RED_TOP, T)] + [(-HX, HX, -HY, HY, 0.0, T)]
    clear = _walls(-HX, HX, -HY, HY, RED_TOP, WALL_TOP, T) + _walls(-HX, HX, -HY, HY, WALL_TOP, RIM_TOP, T)
    handle = [(-H_X, H_X, H_Y0, H_Y1, H_Z0, H_Z1), (-H_X, -H_X + 0.012, HY, H_Y0, H_Z0, H_Z1), (H_X - 0.012, H_X, HY, H_Y0, H_Z0, H_Z1)]
    parts = [(f"red{i}", _bx(*b), "red") for i, b in enumerate(red)] + [(f"clr{i}", _bx(*b), "clear") for i, b in enumerate(clear)]
    parts += [(f"hdl{i}", _bx(*b), "red") for i, b in enumerate(handle)]
    parts += [("label", _bx(-HX - 0.0008, -HX, -0.050, 0.050, 0.045, 0.125), "label"), ("bio", m.cylinder(0.016, 0.0006, (-HX - 0.0011, 0.0, 0.095), axis="x"), "ink"),
              ("fill", _bx(-HX - 0.0009, -HX, -0.050, 0.050, 0.138, 0.141), "ink")] + _syringes()
    cols = [_col(f"wc{i}", b) for i, b in enumerate(_walls(-HX, HX, -HY, HY, 0.0, WALL_TOP, T) + [(-HX, HX, -HY, HY, 0.0, T)] + handle)]
    # the cap, modelled shut and turned open to joint zero: a clear moulded top with its needle ports, a skirt over the collar's rim
    cx0, cx1, cy = -HX - 0.002, HX + 0.002, HY + 0.002
    cap_boxes = [(cx0, cx1, -cy, cy, CAP_Z1 - 0.003, CAP_Z1)] + _walls(cx0, cx1, -cy, cy, CAP_Z0 - 0.006, CAP_Z1 - 0.003, 0.002)
    cap_parts = [(f"cap{i}", _about_y(_bx(*b), -OPEN, HXC, CAP_Z0), "cap") for i, b in enumerate(cap_boxes)]
    cap_parts += [(f"port{i}", _about_y(_bx(px - 0.012, px + 0.012, -0.004, 0.004, CAP_Z1, CAP_Z1 + 0.001), -OPEN, HXC, CAP_Z0), "port") for i, px in enumerate((-0.030, 0.0))]
    cap_parts += [("tab", _about_y(_bx(cx1, cx1 + 0.010, -0.020, 0.020, CAP_Z1 - 0.006, CAP_Z1 - 0.002), -OPEN, HXC, CAP_Z0), "cap")]
    cap = m.hinge("cap", "Y", (HXC, 0.0, CAP_Z0), -2.0, OPEN + math.degrees(math.atan(TRAVEL_MM / 1000 / LEVER)) + 0.2, cap_parts,
                  [_col_turned("capc", (cx0, cx1, -cy, cy, CAP_Z0, CAP_Z1), -OPEN, HXC, CAP_Z0)], mass=0.04, damping=0.002, friction=0.002, gravity=True, inertia=(7e-5, 7e-5, 1.4e-4))
    catches = []
    for k, yc in enumerate((-CATCH_Y, CATCH_Y)):
        b = (CATCH_X - 0.0055, CATCH_X + 0.0055, yc - 0.007, yc + 0.007, CAP_Z0 - 0.016, CAP_Z0)
        catches.append(m.slider(f"catch{k}", "Z", -TRAVEL_MM / 1000, 0.0, [(f"cat{k}", _bx(*b), "white")], [_col(f"catc{k}", b)], at=(CATCH_X, yc, CAP_Z0),
                                mass=0.005, spring=CATCH_K, damping=4.0, friction=0.05, inertia=(2e-7, 2e-7, 2e-7)))
    return m.mechanism("sharps", (CX, CY, TOP), [cap, *catches], LOOKS, frame_parts=parts, frame_colliders=cols, friction=(0.7, 0.6), solver_iters=64,
                       self_collisions=True, free_mass=MASS, frame_inertia=(0.0029, 0.0031, 0.0016))


def _carton():
    boxes = [(-BL / 2, BL / 2, -BW / 2, BW / 2, 0.0, BT)] + _walls(-BL / 2, BL / 2, -BW / 2, BW / 2, 0.0, BH, BT)
    parts = [(f"w{i}", _bx(*b), "card") for i, b in enumerate(boxes)]
    flaps = [(-BL / 2 - 0.004, -BL / 2, -BW / 2, BW / 2, BH - 0.10, BH), (BL / 2, BL / 2 + 0.004, -BW / 2, BW / 2, BH - 0.10, BH),
             (-BL / 2, BL / 2, -BW / 2 - 0.004, -BW / 2, BH - 0.10, BH), (-BL / 2, BL / 2, BW / 2, BW / 2 + 0.004, BH - 0.10, BH)]
    parts += [(f"f{i}", _bx(*b), "card") for i, b in enumerate(flaps)]  # the flaps folded down outside the walls
    parts += [("lab", _bx(-BL / 2 - 0.0050, -BL / 2 - 0.004, -0.05, 0.05, 0.03, 0.09), "bio")]
    return m.fixed("carton", (BX, BY, TOP), parts, [_col(f"c{i}", b) for i, b in enumerate(boxes)], LOOKS, friction=(0.6, 0.5))


OBJECTS = [_container(), _carton()]


# -- the scorer ---------------------------------------------------------------------------------------------------------------------------------------------

def judge(temp: bool, final: bool, boxed: bool, released: bool, dead: bool, dropped: bool = False) -> float:
    """0 if the container tipped past 30 degrees or fell off the counter; 1.0 final-locked, upright in the carton, let go and never dropped;
    0.67 final-locked (also the cap on a run that dropped the container); 0.33 the temporary click."""
    if dead:
        return 0.0
    if final and boxed and released and not dropped:
        return 1.0
    if final:
        return 0.67
    return 0.33 if temp else 0.0


def in_carton(pos, tilt: float) -> bool:
    """The container upright (within 10 degrees), its footprint inside the carton's walls and its base on the carton's floor."""
    x, y = pos[0] - BX, pos[1] - BY
    return bool(tilt < 10.0 and abs(x) + HX <= BL / 2 - BT + 0.002 and abs(y) + HY <= BW / 2 - BT + 0.002 and pos[2] - TOP < BT + 0.012)


def hand_points(env) -> np.ndarray:
    """Points over both whole grippers (station frame): from 6 cm behind the gripper link's origin (wrist side) to the fingertips, 10 cm across, 6 cm wide:
    fingers, pads, palm and housing."""
    base = env.unwrapped
    origin = m._np(base.scene.env_origins[0])
    out = []
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        i = robot.find_bodies("gripper")[0][0]
        pose = m._np(robot.data.body_link_pose_w)[0, i]
        R = m.quat_matrix(pose[3:7])
        g = np.array([(sx, dy, dz) for sx in np.linspace(-0.05, 0.05, 7) for dy in np.linspace(-0.03, 0.03, 5) for dz in np.linspace(-0.06, 0.147, 14)])
        out.append(pose[:3] - origin + g @ R.T)
    return np.vstack(out)


BOUND = np.array([[-HX - 0.002, HX + 0.002], [-HY - 0.002, H_Y1], [0.0, CAP_Z1 + 0.001]])  # the container's whole outline (body, handle, cap shut) in its frame


def grip_z(env) -> list:
    """Height of each gripper link (left, right)."""
    base = env.unwrapped
    return [float(m._np(base.scene[f"{side}_robot"].data.body_link_pose_w)[0, base.scene[f"{side}_robot"].find_bodies("gripper")[0][0]][2]) for side in ("left", "right")]


def clearance(local_pts: np.ndarray) -> float:
    """Smallest distance from any point to the container's outline box (0 inside it)."""
    gap = np.maximum(BOUND[:, 0] - local_pts, 0.0) + np.maximum(local_pts - BOUND[:, 1], 0.0)
    return float(np.min(np.linalg.norm(gap, axis=1))) if len(local_pts) else 1.0


class Trial:
    def reset(self, env=None) -> None:
        self.n, self.held, self.dead, self.dropped, self.temp, self.final, self.score, self.full = 0, 0, False, False, False, False, 0.0, False
        self.last, self.prev, self.why, self.zs, self.hz = {}, None, "", [], []

    def step(self, env) -> None:
        st = m.state(env, "sharps")
        open_deg = OPEN - st.deg("cap")
        depth = [-1000.0 * st.joint(f"catch{k}") for k in range(2)]
        shut = open_deg <= 0.5  # the cap down on the catches (it rests on them at 0)
        if shut and min(depth) >= TEMP_MM:
            self.temp = True
        if not self.final and shut and min(depth) >= FINAL_MM:
            self.final = True
            m.lock(env, "sharps", "cap")  # the final lock's snap: the cap is held shut from here on
        tilt = st.tilt_deg()
        if tilt > 30.0 or st.pos[2] < TOP - 0.05:
            self.dead, self.why = True, self.why or f"container tipped {tilt:.0f} deg or fell"
        boxed = in_carton(st.pos, tilt)
        dist = clearance(st.to_local(np.vstack([m.fingertips(env), hand_points(env)]))) if (self.final and boxed) else 0.0
        now = np.r_[st.pos]
        tips = m.fingertips(env)
        aclr = [clearance(st.to_local(tips[12 * k:12 * k + 12])) for k in range(2)]  # each gripper's fingertips to the container's outline
        self.zs = (getattr(self, "zs", []) + [now[2]])[-4:]  # the last four heights of the container and of each gripper link
        self.hz = (getattr(self, "hz", []) + [grip_z(env)])[-4:]
        cf = {w: self.zs[-1 - w] - self.zs[-1] for w in (1, 3) if len(self.zs) > w}  # container fall over 1 and 3 steps (positive = down)
        if (cf.get(1, 0.0) > DROP_MM / 1000 or cf.get(3, 0.0) > DROP3_MM / 1000):
            def held(w: int) -> bool:  # a gripper holds it: its fingertips are within 3 mm and it fell with the container
                return any(aclr[k] < HELD_MM / 1000 and self.hz[-1 - w][k] - self.hz[-1][k] >= cf[w] - HOLD_SLACK for k in range(2))

            if (cf.get(1, 0.0) > DROP_MM / 1000 and not held(1)) or (cf.get(3, 0.0) > DROP3_MM / 1000 and not held(3)):  # a fall no gripper goes down with: dropped, not placed
                if not self.dropped:
                    print(f"[sc] drop at step {self.n}: z {now[2] - TOP:.3f} fingertip clear {min(aclr) * 1000:.1f} mm", flush=True)
                self.dropped = True
        moving = self.prev is not None and np.linalg.norm(now - self.prev) > 0.0005
        self.prev = now
        self.held = self.held + 1 if (dist >= 0.02 and not moving) else 0
        self.score = judge(self.temp and open_deg <= 2.0, self.final, boxed, self.held >= HOLD_STEPS, self.dead, self.dropped)  # the temporary click counts only while the cap is still shut
        self.full = bool(self.score >= 1.0)
        self.n += 1
        self.last = dict(open=round(open_deg, 1), depth_mm=[round(v, 1) for v in depth], temp=self.temp, final=self.final, tilt=round(tilt, 1), pos=np.round(st.pos, 3).tolist(),
                         boxed=boxed, dist_mm=round(dist * 1000, 1), held=self.held, dead=self.dead, dropped=self.dropped, why=self.why, score=self.score)
        if self.n % 20 == 0 and ENV("SC_TRACE"):
            print(f"[sc] step {self.n}: {self.last}", flush=True)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    """Both catches pressed to the final lock, the container upright inside the carton, both grippers 2 cm clear and all still for 1 s, never tipped past 30 degrees and never dropped."""
    return TRIAL.full


def score(objects) -> float:
    """1.0 final-locked, upright in the carton, let go, not dropped; else 0.67 final-locked (a dropped container stays at 0.67), 0.33 temporary click on both catches
    with the cap still shut. 0 if the container tipped past 30 degrees or fell off the counter."""
    return TRIAL.score


# -- the demo -------------------------------------------------------------------------------------------------------------------------------------------------

RCAP = float(ENV("SC_RCAP", "0.11"))  # where the right hand pushes the cap: distance from the hinge
STAND = float(ENV("SC_STAND", "0.004"))  # from the cap's outer face to the fingertips
TIP = 0.015  # from the TCP to the fingertips, along the fingers
GL_TILT = float(ENV("SC_GLTILT", "45"))
GR_TILT = float(ENV("SC_GRTILT", "60"))
GP_TILT = float(ENV("SC_GPTILT", "40"))  # the right hand's grip for the press: steeper, so the arm is stiff downwards
LEAD = float(ENV("SC_LEAD", "6.0"))  # degrees the hand leads the cap while swinging it
PRESS_X = float(ENV("SC_PRESSX", "0.050"))  # where the press lands (container x)
PRESS_N = float(ENV("SC_PRESSN", "16.0"))
LIGHT_N = float(ENV("SC_LIGHTN", "4.0"))
LIFT = float(ENV("SC_LIFT", "0.18"))  # clears the carton wall (12.5 cm) with the container base
DROP_LOW_H = float(ENV("SC_DL", "0.085"))  # drop_low: the hand lets go 8.5 cm above the carton floor; the base is then about 3.7 cm up (under load the base sits about 4.8 cm below the commanded hand height, measured at release)
DROP_H = 0.15  # drop_high: the hand lets go 15 cm above its grip height; the base is then about 10 cm up (measured at release)
DROP_2CM_H = float(ENV("SC_D2", "0.065"))  # drop_2cm: the hand lets go with the base about 2 cm above the carton floor
HF_T = float(ENV("SC_HFT", "0.7"))  # held_fast_lower: seconds for the 17 cm descent
GRIP_N = float(ENV("SC_GRIPN", "35.0"))


def demo_script(ctx):
    """Modes: full (left holds the handle, right swings the cap over and presses it to the final lock, left lifts the container into the carton and lets go).
    none: the arms wait. light_press: as full, but the press stops at the temporary click. no_press: the cap is swung shut and the container carried with no press.
    free_press: the right hand swings and presses with the container not held, then the left carries it. drop_high: as full, but the left hand lets go with the base about 10 cm above the carton
    floor. drop_low: as drop_high, but with the base about 3.7 cm above the carton floor. cap_fall: the right hand lifts the cap to 60 degrees open and lets it fall shut under its own weight; the left then carries the container. drop_2cm: as drop_low, with the base about 2 cm up. held_fast_lower (a check mode): as full, but the left hand lowers the gripped container into the carton in 0.7 s (about 0.24 m/s on average, over 8 mm a step), which is not a drop."""
    mode = ctx.mode
    if mode not in ("full", "none", "light_press", "no_press", "free_press", "drop_high", "drop_low", "drop_2cm", "cap_fall", "held_fast_lower"):
        raise ValueError(f"unknown mode {mode!r}")
    L, R = ctx.left, ctx.right
    if mode == "none":
        yield from L.wait(3.0)
        return
    cp = ctx.pos("sharps")

    def W(x, y, z):  # a point in the container's frame, as laid out at reset
        return cp + np.array([x, y, z])

    gl, gr = d.tilted(GL_TILT, 0.0), d.tilted(GR_TILT, 0.0)  # both pointing forward and down, fingers closing along y
    al = np.array([math.sin(math.radians(GL_TILT)), 0.0, -math.cos(math.radians(GL_TILT))])  # the left fingers' pointing direction
    ar = np.array([math.sin(math.radians(GR_TILT)), 0.0, -math.cos(math.radians(GR_TILT))])

    def open_deg():
        return OPEN - ctx.deg("sharps", "cap")

    def pad(phi):  # the right TCP that puts the fingertips STAND off the cap's outer face, RCAP from the hinge, with the cap open phi degrees
        x, z = _rot(HXC + RCAP, CAP_Z1 + STAND, HXC, CAP_Z0, -phi)
        return W(x, 0.0, z) - TIP * ar

    hy = (H_Y0 + H_Y1) / 2
    hc = W(0.0, hy, (H_Z0 + H_Z1) / 2)

    def take_handle():
        yield from L.move(hc - 0.10 * al + np.array([0.0, 0.0, 0.03]), grip=gl, finger=-0.022, t=2.0, label="left behind the handle")
        yield from L.move(hc - 0.04 * al, t=1.0, label="left in line with the handle")
        yield from L.move(hc, t=1.0, label="left round the handle")
        yield from L.close(force=GRIP_N, t=0.6, label="left grips the handle")

    if mode != "free_press":
        yield from take_handle()
    # right: in front of the open cap, then swing it over
    yield from R.move(pad(open_deg() + 10.0) - 0.05 * ar, grip=gr, finger=d.FINGER_CLOSED, t=2.0, label="right in front of the cap")
    yield from R.move(pad(open_deg() + 3.0), t=1.0, label="right to the cap")
    if mode == "cap_fall":
        yield from R.follow(lambda: pad(max(open_deg() - LEAD, -1.0)), t=8.0, speed=0.10, until=lambda: open_deg() <= 60.0, label="lift the cap to 60 degrees")
        yield from R.move(R.at() + np.array([-0.02, 0.0, 0.10]), t=0.6, label="right away from the cap")
        yield from R.wait(3.0)
    else:
        yield from R.follow(lambda: pad(max(open_deg() - LEAD, -1.0)), t=8.0, speed=0.10, until=lambda: open_deg() <= 0.5, label="swing the cap over")
    print(f"[sc] cap down {TRIAL.last}", flush=True)
    yield from R.move(R.at() + np.array([-0.02, 0.0, 0.04]), t=0.6, label="right up")
    ap = np.array([math.sin(math.radians(GP_TILT)), 0.0, -math.cos(math.radians(GP_TILT))])
    yield from R.move(W(PRESS_X, 0.0, CAP_Z1 + 0.02) - TIP * ap, grip=d.tilted(GP_TILT, 0.0), t=1.2, label="right over the catches")
    if mode not in ("no_press", "cap_fall"):
        res = yield from R.press((0.0, 0.0, -1.0), LIGHT_N if mode == "light_press" else PRESS_N, 0.030, speed=0.02, max_lead=0.10, label="press the cap down")
    print(f"[sc] pressed {TRIAL.last}", flush=True)
    yield from R.move(R.at() + np.array([-0.03, 0.0, 0.08]), t=0.8, label="right up")
    yield from R.move(R.at() + np.array([-0.10, -0.10, 0.0]), t=1.0, label="right away")
    if mode == "free_press":
        yield from take_handle()
    # left: lift by the handle, over the carton wall, down onto the carton floor, let go
    h0 = L.at()
    yield from L.move(h0 + np.array([0.0, 0.0, LIFT]), t=2.0, label="lift by the handle")
    yield from L.move(np.array([BX + h0[0] - cp[0], BY + h0[1] - cp[1], h0[2] + LIFT]), t=2.5, label="over the carton")
    yield from L.move(np.array([BX + h0[0] - cp[0], BY + h0[1] - cp[1], h0[2] + (DROP_H if mode == "drop_high" else BT + DROP_LOW_H if mode == "drop_low" else BT + DROP_2CM_H if mode == "drop_2cm" else BT + 0.006)]), t=HF_T if mode == "held_fast_lower" else 3.5, label="down into the carton")
    yield from L.open(t=0.6, label="let go")
    yield from L.move(L.at() - 0.06 * al, t=1.0, label="left back off the handle")
    yield from L.move(L.at() + np.array([-0.06, 0.04, 0.10]), t=1.2, label="left away")
    yield from L.wait(2.5)
    print(f"[sc] end {TRIAL.last}", flush=True)
