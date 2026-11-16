"""Lock a free-rolling 3-tier utility cart in place: press down the brake tabs on its two locking casters until each locks, then move both
hands clear. The cart is judged by effect: once locked, an 8 N push at its bottom shelf must not move it.

The cart (a Mind Reader LIDCART3T-BLK 3-tier metal cart, 15.5 x 11 x 28.25 in, 7.85 lb, four casters, two of them locking) stands on the
table top, which stands for the floor: the arms cannot reach floor level. It is one free articulation, authored at joint zero:
  frame: four corner posts, the perforated bottom shelf (10 cm up), the drawer tier, the covered top compartment, the casters' top plates;
  sw_nl, sw_nr, sw_fl, sw_fr: each caster's fork on its swivel (a vertical axis), near (toward the robot) and far, left and right;
  wh_nl, ...: each wheel on its axle, 50 mm across, trailing 20 mm behind its swivel axis (away from the robot at the start); it meets the
    floor as a ball of its radius, a round tread the same at every angle of the wheel, of rubber (0.8, 0.7 sliding) with a 5 mm contact patch;
  tab_l, tab_r: the near casters' brake tabs, each a short sprung slide on its fork, its pressed end 2 cm in front of the bottom shelf.
Rolling resistance and swivel friction are dry friction, held in code at each step (kit README 15): each free wheel and swivel has a drive
whose target stays put until the joint has slipped past effort / stiffness from it, and then follows that far behind, so it resists with at
most a fixed torque: 0.5 N at each wheel's rim (the free cart starts rolling at about 2.5 N) and 0.03 N m at each swivel. A tab pressed down at least 7 mm of its 8 mm travel (about 9 N on its spring) latches down, and its caster is then a total lock:
wheel and swivel held. A tab pressed less springs back up, and its caster stays free.

The scorer judges by effect: with both tabs latched and every fingertip and arm joint 3 cm clear of the cart, it pushes the cart at the
bottom shelf's front edge, 10 cm up, along +x, then along +y at the right front post, 10 cm up, each push rising to 8 N over 0.5 s and held
1 s, as a hand pulls a spring gauge; each must move it under 5 mm. Then the cart must stand still (1 mm) for 1 s with the hands clear.

Debug:  python demokit.py task.py OUT --no-record --stills 20 --mode full
Final:  python demokit.py task.py OUT
Gates:  python demokit.py task.py OUT --no-record --mode onehand     (one hand presses one tab, nothing steadies the cart, the other hand idle)
        python demokit.py task.py OUT --no-record --mode nosteady    (both tabs, one at a time, nothing steadying the cart)
        python demokit.py task.py OUT --no-record --mode together    (both tabs pressed at once)
        python demokit.py task.py OUT --no-record --mode half        (each tab pressed only 4 mm: the casters stay free)
        python demokit.py task.py OUT --no-record --mode g_roll      (no hands: the push that starts the free cart rolling)
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
INSTRUCTION = ("Lock the cart in place: press down the brake tabs on both of its locking wheels until each locks, then move both hands clear.")
SUCCESS_HOLD_S = 0.0  # the scorer holds the end state itself (HOLD_STEPS)
TOP = d.TABLE_TOP
DT = d.DT
HOLD_STEPS = 20  # 1 s at 20 Hz
TRACE = os.environ.get("CART_TRACE", "1") != "0"
TRACE_EVERY = int(os.environ.get("CART_EVERY", "10"))

# -- the cart, in its own frame: x away from the robot, y to its left, z up; origin on the table under the cart's middle -------------------
CART_AT = (float(os.environ.get("CART_X", "0.45")), 0.0)
W, L, H = 0.279, 0.394, 0.718  # 11 x 15.5 x 28.25 in (Quill listing); its length along y
MASS = 3.56  # 7.85 lb
POST_R = 0.0095  # round tube about 19 mm across (assumed)
PX, PY = W / 2 - POST_R, L / 2 - POST_R  # the posts' centres: their outsides on the cart's outline
INSET = 0.025  # each caster's swivel axis 2.5 cm in from its corner (design)
SX, SY = W / 2 - INSET, L / 2 - INSET  # swivel axes at (+-0.1145, +-0.172)
WHEEL_R, WHEEL_W, TRAIL = 0.025, 0.018, 0.020  # 50 mm wheel (assumed, 40 to 65 mm), its tread width, the axle's trail behind the swivel axis
PLATE_Z = 0.072  # the casters' top plates: 72 mm overall caster height
SHELF_Z = 0.100  # the bottom shelf's top, 10 cm up (design)
DRAWER_Z = (0.33, 0.47)  # the drawer tier: two drawers side by side
BOX_Z = (0.585, 0.700)  # the covered top compartment, its lid to the cart's full height
TAB_W, TAB_T = 0.020, 0.006  # each brake tab: 20 mm wide, 6 mm thick (assumed)
TAB_X = (-0.008, -0.045)  # it stands out toward the robot, from 8 to 45 mm in front of its swivel axis: its pressed end 2 cm in front of the shelf
TAB_TOP = 0.065  # its top, 6.5 cm up
TAB_TRAVEL = 0.008  # pressed down 8 mm (assumed, 5 to 12 mm)
LOCK_AT = 0.007  # it latches past 7 mm: a tab only part way down leaves the caster free
K_TAB = 10.0 / TAB_TRAVEL  # 10 N at full travel (assumed, 6 to 15 N): about 9 N at the latch
TAU_ROLL = 0.5 * WHEEL_R  # 0.5 N at each wheel's rim: the free cart starts rolling at about 2.5 N (assumed, 1 to 4 N)
FINGER_ON_TAB = 0.03  # a fingertip within this of the tab's top as the tab is down past the latch
TAU_SWIVEL = 0.03  # N m at each swivel
K_DRY = float(os.environ.get("CART_KDRY", "5.0"))  # the dry-friction drives' stiffness, N m/rad: a wheel gives 2.5 mrad (0.06 mm at its rim) before it slips
M_WHEEL, M_FORK, M_TAB = 0.06, 0.04, 0.01
M_FRAME = MASS - 4 * (M_WHEEL + M_FORK) - 2 * M_TAB
PUSH_N = float(os.environ.get("CART_PUSH", "8.0"))  # the scorer's test push: twice the top of the free cart's assumed rolling resistance (1 to 4 N), so it tells locked from free
PUSH_STEPS = 20  # each held 1 s
RAMP_STEPS = 10  # after rising from 0 over 0.5 s, as a hand pulls a spring gauge (a step from 0 to 8 N rocks the tall cart on its wheels)
MOVE_LIMIT, STEADY_LIMIT, TEST_LIMIT = 0.05, 0.02, 0.005
CLEAR = 0.03
LOCK_K = float(os.environ.get("CART_LOCK_K", "50000"))  # a locked wheel's or swivel's hold, N m/rad
LOCK_E = float(os.environ.get("CART_LOCK_E", "2000"))
ANG_STILL = 0.05  # rad/s: the cart's spin when the test pushes may start

CASTERS = {"nl": (-1, 1), "nr": (-1, -1), "fl": (1, 1), "fr": (1, -1)}  # near/far (x sign), left/right (y sign)
NEAR = {"left": "nl", "right": "nr"}

LOOKS = {
    "powder": m.Look(m.rgb(0.07, 0.07, 0.075), roughness=0.45, metallic=0.4),
    "shelf": m.Look(m.rgb(0.10, 0.10, 0.105), roughness=0.5, metallic=0.4),
    "hole": m.Look(m.rgb(0.015, 0.015, 0.015), roughness=0.9),
    "drawer": m.Look(m.rgb(0.12, 0.12, 0.125), roughness=0.4, metallic=0.4),
    "handle": m.Look(m.rgb(0.62, 0.63, 0.65), roughness=0.3, metallic=0.8),
    "zinc": m.Look(m.rgb(0.70, 0.72, 0.75), roughness=0.3, metallic=0.85),
    "tyre": m.Look(m.rgb(0.13, 0.13, 0.14), roughness=0.85),
    "hub": m.Look(m.rgb(0.78, 0.78, 0.80), roughness=0.4),
    "tab": m.Look(m.rgb(0.78, 0.12, 0.10), roughness=0.5),
}


def axis_of(c: str) -> np.ndarray:
    sx, sy = CASTERS[c]
    return np.array([sx * SX, sy * SY, 0.0])


TREAD_MU = (0.8, 0.7)  # the tyres' rubber tread on the floor, static and sliding (setup text)
TREAD_COMBINE = "max"  # the tread's own friction holds where it meets the table top, whose default material is 0.5
TREAD_PATCH = 0.005  # m: the tread's contact patch, which gives the wheel its turning (scrub) friction on the floor


def col_ball(name: str, radius: float, center) -> str:
    """A ball collider: a wheel's round tread where it meets the floor, the same at every angle of the wheel, with the tread's material."""
    head = getattr(m, "_COL_HEAD", "").replace("/PhysicsMaterial>", "/TreadMaterial>")
    patch = f"""
        float physxCollision:torsionalPatchRadius = {TREAD_PATCH:.4f}
        float physxCollision:minTorsionalPatchRadius = {TREAD_PATCH:.4f}"""
    return f"""
    def Sphere "{name}" {head}{patch}
        double radius = {radius:.6f}
        double3 xformOp:translate = ({center[0]:.6f}, {center[1]:.6f}, {center[2]:.6f})
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }}"""


def _caster(c: str) -> list[m.Link]:
    a = axis_of(c)
    ax, ay = float(a[0]), float(a[1])
    wx = ax + TRAIL  # the wheel trails toward +x: the cart was last rolled toward the robot
    fork = [("housing", m.cylinder(0.018, 0.006, (ax, ay, PLATE_Z - 0.009)), "zinc"),
            ("crown", m.box((0.030 + TRAIL, WHEEL_W + 0.010, 0.004), (ax + TRAIL / 2, ay, PLATE_Z - 0.014)), "zinc")]
    for s in (-1, 1):
        fork.append((f"leg{'ab'[s > 0]}", m.box((0.024, 0.0025, PLATE_Z - 0.016 - WHEEL_R + 0.008), (wx, ay + s * (WHEEL_W / 2 + 0.003), (PLATE_Z - 0.016 + WHEEL_R - 0.008) / 2)), "zinc"))
    fork.append(("axle", m.cylinder(0.005, WHEEL_W + 0.012, (wx, ay, WHEEL_R), axis="y", segments=12), "zinc"))
    sw = m.hinge(f"sw_{c}", "Z", (ax, ay, 0.0), -math.inf, math.inf, fork, [], mass=M_FORK, spring=K_DRY, damping=0.001, effort=TAU_SWIVEL, friction=0.0,
                 armature=1e-5, inertia=(2e-5, 2e-5, 2e-5))
    tyre = m.cylinder(WHEEL_R, WHEEL_W, (wx, ay, WHEEL_R), axis="y", segments=28)
    hub = m.cylinder(0.013, WHEEL_W + 0.002, (wx, ay, WHEEL_R), axis="y", segments=16)
    i_axis, i_across = 0.5 * M_WHEEL * WHEEL_R ** 2, M_WHEEL * (3 * WHEEL_R ** 2 + WHEEL_W ** 2) / 12
    tread = col_ball(f"wh_{c}_c", WHEEL_R, (wx, ay, WHEEL_R))  # the round tread, the same at every angle of the wheel (a faceted collider would roll and lock unlike a wheel)
    wh = m.hinge(f"wh_{c}", "Y", (wx, ay, WHEEL_R), -math.inf, math.inf, [("tyre", tyre, "tyre"), ("hub", hub, "hub")], [tread],
                 mass=M_WHEEL, spring=K_DRY, damping=0.0002, effort=TAU_ROLL, friction=0.0, armature=1e-6, parent=f"sw_{c}", inertia=(i_across, i_axis, i_across))
    links = [sw, wh]
    if c in NEAR.values():
        x0, x1 = ax + TAB_X[0], ax + TAB_X[1]
        cx, cz = (x0 + x1) / 2, TAB_TOP - TAB_T / 2
        pedal = [("pedal", m.box((x0 - x1, TAB_W, TAB_T), (cx, ay, cz)), "tab")]
        for k in range(4):  # the tread ribs across its pressed end
            pedal.append((f"rib{k}", m.box((0.002, TAB_W - 0.002, 0.0012), (x1 + 0.004 + 0.005 * k, ay, TAB_TOP + 0.0006)), "tab"))
        side = "l" if CASTERS[c][1] > 0 else "r"
        links.append(m.slider(f"tab_{side}", "Z", -TAB_TRAVEL - 0.002, 0.0, pedal, [m.col_box(f"tab_{side}_c", (x0 - x1, TAB_W, TAB_T), (cx, ay, cz))],
                              at=(cx, ay, cz), mass=M_TAB, spring=K_TAB, damping=2.0, effort=200.0, friction=0.0, armature=1e-5, parent=f"sw_{c}"))
    return links


def _frame():
    parts, cols = [], []
    for sx in (-1, 1):
        for sy in (-1, 1):
            n = f"{'nf'[sx > 0]}{'rl'[sy > 0]}"
            parts.append((f"post_{n}", m.cylinder(POST_R, H - PLATE_Z, (sx * PX, sy * PY, (H + PLATE_Z) / 2), segments=20), "powder"))
            cols.append(m.col_cylinder(f"post_{n}_c", POST_R, H - PLATE_Z, (sx * PX, sy * PY, (H + PLATE_Z) / 2)))
            parts.append((f"plate_{n}", m.box((0.050, 0.050, 0.004), (sx * SX, sy * SY, PLATE_Z - 0.002)), "zinc"))
            parts.append((f"mount_{n}", m.box((0.030, 0.030, SHELF_Z - PLATE_Z - 0.003), (sx * SX, sy * SY, (SHELF_Z + PLATE_Z) / 2 - 0.0015)), "powder"))
    # the bottom shelf: a perforated tray with a 25 mm lip all round
    t = 0.003
    parts.append(("shelf", m.box((W - 0.004, L - 0.004, t), (0.0, 0.0, SHELF_Z - t / 2)), "shelf"))
    holes = [m.box((0.008, 0.008, 0.0006), (x, y, SHELF_Z + 0.0003)) for x in np.linspace(-W / 2 + 0.03, W / 2 - 0.03, 10) for y in np.linspace(-L / 2 + 0.03, L / 2 - 0.03, 14)]
    parts.append(("perforation", m.join(*holes), "hole"))
    lip = 0.025
    for s in (-1, 1):
        parts.append((f"lip_x{'ab'[s > 0]}", m.box((0.0015, L - 0.004, lip), (s * (W / 2 - 0.002), 0.0, SHELF_Z + lip / 2 - t)), "powder"))
        parts.append((f"lip_y{'ab'[s > 0]}", m.box((W - 0.004, 0.0015, lip), (0.0, s * (L / 2 - 0.002), SHELF_Z + lip / 2 - t)), "powder"))
    cols.append(m.col_box("shelf_c", (W - 0.004, L - 0.004, 0.012), (0.0, 0.0, SHELF_Z - 0.006)))
    cols.append(m.col_box("lip_front_c", (0.004, L - 0.004, lip), (-W / 2 + 0.003, 0.0, SHELF_Z + lip / 2 - t)))
    # the drawer tier: a shell with two drawers side by side, each with a pull
    z0, z1 = DRAWER_Z
    parts.append(("drawer_shell", m.box((W - 0.006, L - 0.006, z1 - z0), (0.0, 0.0, (z0 + z1) / 2)), "powder"))
    for s in (-1, 1):
        parts.append((f"drawer_{'rl'[s > 0]}", m.box((0.006, L / 2 - 0.016, z1 - z0 - 0.016), (-W / 2 + 0.001, s * L / 4, (z0 + z1) / 2)), "drawer"))
        parts.append((f"pull_{'rl'[s > 0]}", m.box((0.010, 0.060, 0.008), (-W / 2 - 0.004, s * L / 4, (z0 + z1) / 2 + 0.02)), "handle"))
    cols.append(m.col_box("drawers_c", (W, L, z1 - z0), (0.0, 0.0, (z0 + z1) / 2)))
    # the covered top compartment and its lid
    b0, b1 = BOX_Z
    parts.append(("bin", m.box((W - 0.004, L - 0.004, b1 - b0), (0.0, 0.0, (b0 + b1) / 2)), "powder"))
    parts.append(("lid", m.box((W, L, H - b1), (0.0, 0.0, (b1 + H) / 2)), "drawer"))
    parts.append(("lid_pull", m.box((0.010, 0.080, 0.006), (-W / 2 - 0.004, 0.0, (b1 + H) / 2)), "handle"))
    cols.append(m.col_box("bin_c", (W, L, H - b0), (0.0, 0.0, (b0 + H) / 2)))
    return parts, cols


def _cart() -> m.Part:
    links = []
    for c in CASTERS:
        links += _caster(c)
    fp, fc = _frame()
    # a 0.28 x 0.39 x 0.72 m frame of 3.14 kg: principal inertia from its box, about its middle
    inertia = (M_FRAME * (L ** 2 + H ** 2) / 12, M_FRAME * (W ** 2 + H ** 2) / 12, M_FRAME * (W ** 2 + L ** 2) / 12)
    return m.mechanism("cart", (CART_AT[0], CART_AT[1], TOP + 0.0005), links, LOOKS, fp, fc, friction=(float(os.environ.get("CART_MU", "0.8")), float(os.environ.get("CART_MU", "0.8")) - 0.1), free_mass=M_FRAME,
                       frame_inertia=inertia, solver_iters=int(os.environ.get("CART_ITERS", "48")))


def _with_tread(part):
    """The cart's USD with a second physics material, the tyres' tread, which the wheel colliders use."""
    if part is None:
        return part
    import dataclasses

    mat = f"""    def Material "TreadMaterial" (
        prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]
    )
    {{
        float physics:staticFriction = {TREAD_MU[0]}
        float physics:dynamicFriction = {TREAD_MU[1]}
        float physics:restitution = 0
        uniform token physxMaterial:frictionCombineMode = "{TREAD_COMBINE}"
    }}

"""
    build = part.build

    def with_tread() -> str:
        text = build()
        anchor = '    def Scope "Looks"'
        if text.count(anchor) != 1:
            raise RuntimeError("cart USD: no single Looks scope to put the tread material before")
        return text.replace(anchor, mat + anchor, 1)

    return dataclasses.replace(part, build=with_tread)


OBJECTS = [_with_tread(_cart())]
SWIVELS = [f"sw_{c}" for c in CASTERS]
WHEELS = [f"wh_{c}" for c in CASTERS]

# -- the brakes, rolling resistance and swivel friction ------------------------------------------------------------------------------------------


class Brakes:
    """Each near caster's brake: its tab latches down once pressed past LOCK_AT, and the caster is then a total lock (its wheel and its swivel
    held stiff where they are). Every other wheel and swivel gets dry friction: its drive's target follows the joint each step."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.latched = {"left": False, "right": False}
        self.depth = {"left": 0.0, "right": 0.0}
        self.peak = {"left": 0.0, "right": 0.0}
        self.held: set[str] = set()
        self.target: dict[str, float] = {}
        self.down = {"left": 0, "right": 0}  # steps the tab has been down past the latch with a fingertip on it

    def latch(self, env, side: str):
        self.latched[side] = True
        m.lock(env, "cart", f"tab_{side[0]}", stiffness=20000.0, effort=300.0)  # the tab stays down
        for link in (f"sw_{NEAR[side]}", f"wh_{NEAR[side]}"):
            m.lock(env, "cart", link, stiffness=LOCK_K, effort=LOCK_E)  # total lock: wheel and swivel
            self.held.add(link)

    def step(self, env):
        c = m.state(env, "cart")
        tips = m.fingertips(env)
        for side in ("left", "right"):
            dep = -c.joint(f"tab_{side[0]}")
            a = axis_of(NEAR[side])
            pedal = gk.along(c, (a[0] + TAB_X[1], a[1], TAB_TOP), (a[0] + TAB_X[0], a[1], TAB_TOP), 6)
            fingered = gk.min_distance(tips, pedal) < FINGER_ON_TAB  # a fingertip is on the tab: an arm link, a knock or the cart's own weight does not lock it
            self.down[side] = self.down[side] + 1 if (dep >= LOCK_AT and fingered) else 0
            self.depth[side] = dep
            self.peak[side] = max(self.peak[side], dep)
            if not self.latched[side] and self.down[side] >= 1:
                self.latch(env, side)
                if TRACE:
                    print(f"[cart] the {side} tab latched at {1000 * dep:.1f} mm: {side} near caster locked (wheel and swivel)", flush=True)
        for link in SWIVELS + WHEELS:  # dry friction: the target moves only once the joint has slipped past effort / stiffness from it (kit README 15)
            if link in self.held:
                continue
            q, tgt = c.joint(link), self.target.get(link, 0.0)
            lim = (TAU_ROLL if link.startswith("wh") else TAU_SWIVEL) / K_DRY
            if abs(q - tgt) > lim:
                self.target[link] = q - math.copysign(lim, q - tgt)
                m.set_target(env, "cart", link, self.target[link])
        return c


BRAKES = Brakes()

# -- the scorer ---------------------------------------------------------------------------------------------------------------------------------

LADDER = gk.Ladder(("one", 0.25), ("both", 0.5), ("steady", 0.75), ("done", 1.0))
FRONT_MID = (-W / 2, 0.0, SHELF_Z)  # the bottom shelf's front edge, 10 cm up: the +x push
RIGHT_POST = (-PX, -L / 2, SHELF_Z)  # the right front post's outside, 10 cm up: the +y push


def cart_lines(c) -> list[np.ndarray]:
    """The cart for the hands-clear check: its four posts, the bottom shelf's edges, and the near casters' tabs."""
    lines = [gk.along(c, (sx * PX, sy * PY, PLATE_Z - 0.02), (sx * PX, sy * PY, H), 13) for sx in (-1, 1) for sy in (-1, 1)]
    lines.append(gk.along(c, (-W / 2, -L / 2, SHELF_Z), (-W / 2, L / 2, SHELF_Z), 11))
    lines.append(gk.along(c, (W / 2, -L / 2, SHELF_Z), (W / 2, L / 2, SHELF_Z), 11))
    for side in ("left", "right"):
        a = axis_of(NEAR[side])
        lines.append(gk.along(c, (a[0] + TAB_X[1], a[1], TAB_TOP), (a[0] + TRAIL, a[1], WHEEL_R), 6))
    for z in (SHELF_Z, 0.18, 0.26, 0.40, 0.52, 0.65, H):  # the near face (drawers, pulls, bin, lid) and both sides, at several heights
        lines.append(gk.along(c, (-W / 2, -L / 2, z), (-W / 2, L / 2, z), 9))
        for sy in (-1, 1):
            lines.append(gk.along(c, (-W / 2, sy * L / 2, z), (W / 2, sy * L / 2, z), 7))
    for sy in (-1, 1):  # the far casters' wheels
        lines.append(gk.along(c, (W / 2 - INSET + TRAIL, sy * SY, WHEEL_R), (W / 2 - INSET + TRAIL, sy * SY, PLATE_Z), 3))
    return lines


KEYS = [FRONT_MID, RIGHT_POST, (-W / 2, L / 2, SHELF_Z), (W / 2, -L / 2, SHELF_Z)]  # points whose motion shows a slide or a yaw


def key_points(c) -> np.ndarray:
    return c.to_world(np.array(KEYS))


def arm_points(env) -> np.ndarray:
    """Every body of both arms (station frame), wrist and forearm included, not only the fingers."""
    base = env.unwrapped
    origin = np.asarray(base.scene.env_origins[0].cpu())
    return np.vstack([np.asarray(base.scene[f"{side}_robot"].data.body_link_pos_w[0].cpu()) - origin for side in ("left", "right")])


class Trial:
    def __init__(self):
        self.fell = gk.Latch("cart tipped, left the table or moved more than 5 cm")
        self.hold = gk.EndHold(HOLD_STEPS)
        self.reset()

    def reset(self, env=None):
        BRAKES.reset(env)
        self.fell.reset()
        self.hold.reset()
        self.n = 0
        self.start = None
        self.peak = 0.0  # the cart's farthest from its start, outside the test pushes
        self.phase, self.k, self.ready = "wait", 0, 0
        self.ref = None
        self.test = {"x": None, "y": None}
        self.passed = False
        self.still_ref = None
        self.reached = {}
        self.clear = False

    def _push(self, env, c, force, local):
        """Start a test push: it rises to `force` over RAMP_STEPS steps at the point `local` (cart frame, fixed where it is now), then holds."""
        self.push_force, self.push_at = np.asarray(force, float), c.to_world(np.array([local]))[0]
        self._ramp(env, 0)

    def _ramp(self, env, k):
        m.push(env, "cart", tuple(self.push_force * min(1.0, (k + 1) / RAMP_STEPS)), at=self.push_at)

    def step(self, env):
        self.n += 1
        c = BRAKES.step(env)
        if self.start is None:
            self.start = c.pos.copy()
        moved = float(np.linalg.norm((c.pos - self.start)[:2]))
        pushing = self.phase in ("px", "gap", "py")
        if not pushing:
            self.peak = max(self.peak, moved)
        tilt = c.tilt_deg()
        off = abs(float(c.pos[2] - self.start[2])) > 0.02
        self.fell.trip(tilt > 10.0 or off or self.peak > MOVE_LIMIT)
        locked = sum(BRAKES.latched.values())
        tips = m.fingertips(env)
        lines = cart_lines(c)
        arm = arm_points(env)
        self.clear = gk.hands_clear(tips, lines, CLEAR) and min(float(gk._to_polyline(arm, L, True).min()) for L in lines) > CLEAR
        speed = float(np.linalg.norm(c.lin_vel))
        spin = float(np.linalg.norm(c.ang_vel))
        # the test: PUSH_N along +x at the shelf's front edge, then along +y at the right front post, each rising over 0.5 s and held 1 s, with the hands clear
        if self.phase == "wait":
            self.ready = self.ready + 1 if (locked == 2 and self.clear and speed < 0.01 and spin < ANG_STILL) else 0
            if self.ready >= 4:
                self.phase, self.k, self.ref, self.test["x"] = "px", 0, key_points(c), 0.0
                self._push(env, c, (PUSH_N, 0.0, 0.0), FRONT_MID)
        elif self.phase in ("px", "py"):
            axis = self.phase[1]
            self.k += 1
            if self.k < RAMP_STEPS:
                self._ramp(env, self.k)
            self.test[axis] = max(self.test[axis], float(np.linalg.norm((key_points(c) - self.ref)[:, :2], axis=1).max()))
            if not self.clear:  # a hand came back: the test starts again once the hands are clear
                m.push(env, "cart", (0.0, 0.0, 0.0))
                self.phase, self.ready, self.test = "wait", 0, {"x": None, "y": None}
            elif self.k >= RAMP_STEPS + PUSH_STEPS:
                m.push(env, "cart", (0.0, 0.0, 0.0))
                if axis == "x":
                    self.phase, self.k = "gap", 0
                else:
                    self.phase = "tested"
                    self.passed = max(self.test["x"], self.test["y"]) < TEST_LIMIT
                    if TRACE:
                        print(f"[cart] test pushes: +x moved {1000 * self.test['x']:.2f} mm, +y moved {1000 * self.test['y']:.2f} mm: {'held' if self.passed else 'MOVED'}", flush=True)
        elif self.phase == "gap":  # one reference from the start of the +x push to the end of the +y push: creep adds up
            self.k += 1
            self.test["x"] = max(self.test["x"], float(np.linalg.norm((key_points(c) - self.ref)[:, :2], axis=1).max()))
            if not self.clear:
                self.phase, self.ready, self.test = "wait", 0, {"x": None, "y": None}
            elif self.k >= 6:
                self.phase, self.k, self.test["y"] = "py", 0, 0.0
                self._push(env, c, (0.0, PUSH_N, 0.0), RIGHT_POST)
        steady = locked == 2 and self.peak < STEADY_LIMIT
        end_ok = steady and self.passed and self.clear
        if end_ok:
            if self.still_ref is None or np.max(np.abs(key_points(c) - self.still_ref)) > 0.001:
                self.still_ref = key_points(c)
                self.hold.reset()
        else:
            self.still_ref = None
        done = self.hold.step(end_ok) and end_ok  # the end state must still hold now: hands coming back or a shove after the hold drops it
        self.reached = {"one": locked >= 1, "both": locked == 2, "steady": steady, "done": done}
        if TRACE and self.n % TRACE_EVERY == 0:
            print(f"[cart] step {self.n}: moved {1000 * moved:.1f} mm (peak {1000 * self.peak:.1f}), tilt {tilt:.2f}, tabs L {1000 * BRAKES.depth['left']:.1f} "
                  f"R {1000 * BRAKES.depth['right']:.1f} mm (peak {1000 * BRAKES.peak['left']:.1f}, {1000 * BRAKES.peak['right']:.1f}), latched {BRAKES.latched}, "
                  f"swivels {[round(c.deg(s), 1) for s in SWIVELS]}, clear {self.clear}, phase {self.phase}, hold {self.hold.count}, score {self.score()}", flush=True)

    def score(self) -> float:
        return LADDER.score(self.reached, failed=bool(self.fell))


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.score() >= 1.0


def score(objects) -> float:
    return TRIAL.score()


# -- the demo -----------------------------------------------------------------------------------------------------------------------------------

TILT = float(os.environ.get("CART_TILT", "30"))  # the pressing fingers point down and this far forward (+x), so the hand stays clear of the shelf
TURN = {"left": float(os.environ.get("CART_TURN_L", "90")), "right": float(os.environ.get("CART_TURN_R", "-90"))}  # each pressing hand turned about its fingers: its finger width along y, its wrist camera outboard
TIP = 0.012  # the closed fingers' ends stand about this far past the TCP along the fingers
PRESS_IN = 0.010  # the pressed point: this far in from the tab's end
STEP_MM = 1.5  # each step of the press, straight down
CLEAR_AT = {"left": (0.20, 0.30, 0.30), "right": (0.20, -0.30, 0.30)}  # where each hand waits once clear (x, y, height above the table)


def press_grip(side: str) -> d.Grip:
    """Closed fingers pointing down and TILT degrees forward, turned TURN[side] about their own axis (which side the wrist camera rides on)."""
    g = d.tilted(TILT, 0.0) if TILT else d.down(0.0)
    return d._as_grip(g.rot @ d.axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.radians(TURN[side])))


def tab_point(c, side: str) -> np.ndarray:
    """The point on `side`'s tab top where the fingers press (world), with the cart where it is now (the tab on its fork, as authored)."""
    a = axis_of(NEAR[side])
    return c.to_world(np.array([[a[0] + TAB_X[1] + PRESS_IN, a[1], TAB_TOP]]))[0]


def press_tcp(c, side: str, g: d.Grip) -> np.ndarray:
    return tab_point(c, side) - TIP * g.rot[:, 2]


def press(ctx, arm, side: str):
    """`arm` presses `side`'s brake tab straight down with its closed fingers until the tab latches (or 9 cm of command), then lifts off."""
    g = press_grip(side)
    c = ctx.state("cart")
    tcp = press_tcp(c, side, g)
    a = g.rot[:, 2]
    yield from arm.move(tcp - 0.10 * a + np.array([-0.02, 0.0, 0.04]), grip=g, finger=d.FINGER_CLOSED, t=1.8, label=f"over the {side} tab")
    yield from arm.move(tcp + np.array([0.0, 0.0, 0.02]), t=0.8, label=f"above the {side} tab")
    c0 = ctx.state("cart").pos.copy()
    peak = 0.0
    for k in range(60):
        if BRAKES.latched[side]:
            break
        yield from arm.move(arm.at() - np.array([0.0, 0.0, STEP_MM / 1000]), t=DT, label=f"press the {side} tab")
        peak = max(peak, float(np.linalg.norm((ctx.state("cart").pos - c0)[:2])))
    for _ in range(3):
        yield
    if TRACE:
        print(f"[cart] {arm.side} hand on the {side} tab: latched {BRAKES.latched[side]}, tab down {1000 * BRAKES.peak[side]:.1f} mm at most, "
              f"cart moved {1000 * peak:.1f} mm at most during the press, command {1000 * (tcp[2] - arm.at()[2]):.0f} mm below the tab top's TCP", flush=True)
    yield from arm.move(arm.at() + np.array([-0.03, 0.0, 0.08]), t=0.8, label=f"up off the {side} tab")


POST_Z = float(os.environ.get("CART_POST_Z", "0.22"))  # where a steadying hand takes its front post: this high above the table


def post_point(c, side: str) -> np.ndarray:
    """The middle of `side`'s front post at POST_Z (world)."""
    s = 1.0 if side == "left" else -1.0
    return c.to_world(np.array([[-PX, s * PY, POST_Z]]))[0]


def take_post(ctx, arm, side: str):
    """`arm` takes its own side's front post: fingers level toward +x, jaws closing across the 19 mm tube."""
    g = d.level(0.0)
    p = post_point(ctx.state("cart"), side)
    yield from arm.move(p + np.array([-0.10, 0.0, 0.02]), grip=g, finger=d.FINGER_OPEN, t=1.6, label=f"before the {side} post")
    yield from arm.move(p + np.array([-0.002, 0.0, 0.0]), t=0.8, label=f"round the {side} post")
    yield from arm.close(2 * POST_R, force=20.0, t=0.5, label=f"hold the {side} post")


def leave_post(arm, side: str):
    yield from arm.open(t=0.4, label=f"let go of the {side} post")
    yield from arm.move(arm.at() + np.array([-0.10, 0.0, 0.0]), t=0.8, label=f"back off the {side} post")


def clear(arm, side: str, t: float = 1.4):
    x, y, z = CLEAR_AT[side]
    yield from arm.move(np.array([x, y, TOP + z]), t=t, label=f"{side} hand clear")


def demo_script(ctx):
    """Modes: full: the left hand holds the left front post while the right hand presses the right tab, then the right hand holds the right
    post while the left presses the left tab; both come clear, and the scorer pushes the cart. nosteady: the same presses one at a time with
    nothing steadying the cart. together: each hand presses its own tab, both at once, nothing steadying the cart.
    look: nothing moves, for a still. onehand: the left hand alone presses the left tab, nothing steadying the cart, then comes clear.
    half: each hand presses its tab only 4 mm (a lazy route: the casters stay free). g_roll: no hands; a push at the bottom shelf's front
    edge rises 0.1 N a step, to find the force that starts the free cart rolling."""
    mode = ctx.mode
    env = ctx.emb._env
    l, r = ctx.left, ctx.right
    for _ in range(4):
        yield
    if mode == "look":
        yield from l.wait(3.0)
        return
    if mode == "nosteady":
        # one tab at a time, nothing steadying the cart: the right hand presses the right tab and comes clear, then the left hand the left
        # (a lone press rolls the free cart about 2 cm; both at once about 2.7 cm)
        yield from press(ctx, r, "right")
        yield from ctx.together(clear(r, "right"), press(ctx, l, "left"))
        yield from clear(l, "left")
        yield from l.wait(4.5)
        return
    if mode == "full":
        # the left hand holds the left front post while the right hand presses the right tab; then the right hand holds the right post
        # while the left hand presses the left tab (each press then moved the cart 2 to 4 mm); then both let go and come clear, and the
        # scorer pushes the cart
        yield from ctx.together(take_post(ctx, l, "left"), press(ctx, r, "right"))
        yield from take_post(ctx, r, "right")
        yield from leave_post(l, "left")
        yield from press(ctx, l, "left")
        yield from ctx.together(leave_post(r, "right"), clear(l, "left"))
        yield from clear(r, "right", t=1.0)
        yield from l.wait(4.5)
        return
    if mode == "together":
        yield from ctx.together(press(ctx, l, "left"), press(ctx, r, "right"))
        yield from ctx.together(clear(l, "left"), clear(r, "right"))
        yield from l.wait(4.5)
        return
    if mode == "onehand":
        yield from press(ctx, l, "left")
        yield from clear(l, "left")
        yield from l.wait(1.0)
        return
    if mode == "half":
        def half(arm, side):
            g = press_grip(side)
            tcp = press_tcp(ctx.state("cart"), side, g)
            yield from arm.move(tcp - 0.10 * g.rot[:, 2] + np.array([-0.02, 0.0, 0.04]), grip=g, finger=d.FINGER_CLOSED, t=1.8)
            yield from arm.move(tcp + np.array([0.0, 0.0, 0.02]), t=0.8)
            for _ in range(60):
                if BRAKES.depth[side] >= 0.004:
                    break
                yield from arm.move(arm.at() - np.array([0.0, 0.0, STEP_MM / 1000]), t=DT)
            yield from arm.move(arm.at() + np.array([-0.03, 0.0, 0.08]), t=0.8)
        yield from ctx.together(half(l, "left"), half(r, "right"))
        yield from ctx.together(clear(l, "left"), clear(r, "right"))
        yield from l.wait(2.0)
        return
    if mode == "g_roll":
        c0 = ctx.state("cart")
        start = c0.pos.copy()
        f = 0.0
        for k in range(80):
            c = ctx.state("cart")
            f = 0.1 * (k + 1)
            m.push(env, "cart", (f, 0.0, 0.0), at=c.to_world(np.array([FRONT_MID]))[0])
            yield
            moved = float(np.linalg.norm((ctx.state("cart").pos - start)[:2]))
            if k % 5 == 0:
                print(f"[gate] g_roll: push {f:.1f} N, cart moved {1000 * moved:.2f} mm", flush=True)
            if moved > 0.002:
                print(f"[gate] RESULT g_roll: the free cart starts rolling at {f:.1f} N (moved {1000 * moved:.1f} mm)", flush=True)
                break
        m.push(env, "cart", (0.0, 0.0, 0.0))
        for _ in range(20):
            yield
        return
    raise ValueError(f"unknown mode {mode!r}")
