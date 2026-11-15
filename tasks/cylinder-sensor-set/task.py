"""Set a pneumatic cylinder's slot sensor: drag it along the slot by a hex driver seated in its clamp screw, find its switching band by the LED, centre it, clamp it.

A compact pneumatic cylinder lies fixed on the bench along x, exhausted, its sensor slot up. A slot sensor (a magnetic cylinder sensor,
5 x 4.6 x 29.3 mm) sits loose in the slot, its clamp screw up. One hand pushes the rod fully home by its clevis. The other takes a hex driver from its
holder, seats its tip in the sensor's clamp screw, and drags the sensor along the slot by it until the yellow LED lights, on past the band's far edge, back
to the band's middle, and clamps it with a turn. The scorer then cycles the rod out and back: the LED goes off and on.

Sim (stated, all in code): the cylinder is a fixed mechanism. The rod is a slider whose seal friction is a drive holding it where it is with at most ROD_N.
The sensor is a slider in the slot with friction SLOT_N the same way; its clamp screw's socket is a square opening, so the driver's tip pushes it along.
The LED is lit in code from the piston magnet's distance to the sensor's middle (on inside BAND_ON, off outside BAND_OFF: hysteresis); a lit LED stands
out of the sensor's face, an unlit one sinks into it. The clamp is a code gate: a quarter turn of the driver clockwise with its tip seated locks the
sensor in the slot (the real screw takes about 0.6 N m). The driver's tip is 2 mm across, not the real 1.5 mm hex (stated).

    python demokit.py task.py OUT --mode full
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import demokit as d  # noqa: E402
import mechkit as m  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Push the rod fully in, set the sensor so it switches in the middle of its switching range, clamp it, then let go."
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0
HOLD_STEPS = 40

# the cylinder, in the station frame (x forward, y left, heights above the table)
CY = 0.0  # the cylinder's axis, y
X_REAR, X_FRONT = 0.22, 0.32  # its body's ends
Z_TOP = 0.050  # the body's top face, where the slot opens
BODY_H, BODY_W = 0.040, 0.040
SLOT_W, SLOT_D = 0.0052, 0.0050  # the T-slot's mouth width and depth
STROKE = 0.040
X_MAG_HOME = 0.252  # the piston magnet with the rod home (retracted)
CLEVIS_HOME = 0.36  # the clevis's middle with the rod home
ROD_N, SLOT_N = 8.0, 1.5  # N: the rod's seal friction (exhausted) and the sensor's slot friction (both to measure)
BAND_ON, BAND_OFF = 0.0035, 0.0045  # m: the LED lights within BAND_ON of the magnet and goes out beyond BAND_OFF (the maker's band and hysteresis, to measure)
SENS_L, SENS_W, SENS_H = 0.0293, 0.0050, 0.0046
X_SENS0 = 0.240  # the sensor's middle at the start, loose in the slot near the retracted end (unlit either way)
SOCK = 0.0030  # the opening in the sensor's top round the clamp screw's hex socket
SCREW_T = 0.05  # N m: the clamp screw turning loose
CLAMP_DEG, STRIP_DEG = 75.0, 200.0  # a turn this far clockwise clamps the sensor (the real screw takes about 0.6 N m); past STRIP_DEG it counts as stripped
DRV = np.array([0.20, 0.15])  # the driver's holder
HANDLE_R, HANDLE_L, BLADE_L = 0.007, 0.040, 0.060

LOOKS = {"alu": m.Look(color=m.rgb(0.80, 0.81, 0.83), metallic=0.85, roughness=0.3), "dark": m.Look(color=m.rgb(0.12, 0.12, 0.13), roughness=0.5),
         "steel": m.Look(color=m.rgb(0.72, 0.73, 0.75), metallic=1.0, roughness=0.25), "sensor": m.Look(color=m.rgb(0.10, 0.10, 0.11), roughness=0.5),
         "led": m.Look(color=m.rgb(1.0, 0.85, 0.1), roughness=0.3, emissive=m.rgb(1.0, 0.8, 0.0)), "cable": m.Look(color=m.rgb(0.1, 0.1, 0.12), roughness=0.7),
         "handle": m.Look(color=m.rgb(0.1, 0.35, 0.75), roughness=0.6), "wood": m.Look(color=m.rgb(0.55, 0.40, 0.25), roughness=0.7),
         "plate": m.Look(color=m.rgb(0.35, 0.36, 0.38), metallic=0.7, roughness=0.5)}


def _box(parts, cols, name, size, center, look, collide=True):
    parts.append((name, m.box(size, center), look))
    if collide:
        cols.append(m.col_box(f"{name}_c", size, center))


def _cylinder() -> m.Part:
    """In the part's frame: the station frame, shifted to y CY (its origin on the table under the cylinder's axis at x 0)."""
    parts, cols = [], []
    L = X_FRONT - X_REAR
    xc = (X_REAR + X_FRONT) / 2
    zb = Z_TOP - BODY_H  # the body's underside
    _box(parts, cols, "mount", (L + 0.02, 0.08, zb), (xc, 0.0, zb / 2), "plate")
    # the body: two halves either side of the slot, and the slot's floor
    half = (BODY_W - SLOT_W) / 2
    for k, s in enumerate((-1, 1)):
        _box(parts, cols, f"body{k}", (L, half, BODY_H), (xc, s * (SLOT_W / 2 + half / 2), zb + BODY_H / 2), "alu")
    _box(parts, cols, "slot_floor", (L, SLOT_W, BODY_H - SLOT_D), (xc, 0.0, zb + (BODY_H - SLOT_D) / 2), "alu")
    _box(parts, cols, "cap_rear", (0.008, BODY_W + 0.004, BODY_H + 0.004), (X_REAR - 0.004, 0.0, zb + BODY_H / 2), "dark")
    _box(parts, cols, "cap_front", (0.008, BODY_W + 0.004, BODY_H + 0.004), (X_FRONT + 0.004, 0.0, zb + BODY_H / 2), "dark")
    zr = zb + BODY_H / 2  # the rod's axis
    # the rod and its clevis: a slider along x, authored out (joint 0 = fully out, as found: start joints are not applied at a reset); home is -STROKE
    rod_len = CLEVIS_HOME + STROKE - X_FRONT - 0.008
    rod = m.cylinder(0.005, rod_len, (X_FRONT + 0.008 + rod_len / 2 - STROKE * 0.0, 0.0, zr), axis="x", segments=16)
    clevis = m.box((0.020, 0.016, 0.016), (CLEVIS_HOME + STROKE, 0.0, zr))
    rod_link = m.slider("rod", "X", -STROKE - 0.001, 0.001, [("rod", rod, "steel"), ("clevis", clevis, "steel")],
                        [m.col_box("clevis_c", (0.020, 0.016, 0.016), (CLEVIS_HOME + STROKE, 0.0, zr))], mass=0.08, spring=20000.0, damping=40.0, friction=0.0, effort=ROD_N)
    # the sensor in the slot: a slider along x; its clamp screw's socket a square opening on top; the LED on a small slider of its own
    zs = Z_TOP - SENS_H / 2 - 0.0002
    sens_parts = [("body", m.box((SENS_L, SENS_W - 0.0004, SENS_H), (X_SENS0, 0.0, zs)), "sensor"),
                  ("cable", m.box((0.012, 0.0035, 0.0035), (X_SENS0 - SENS_L / 2 - 0.006, 0.0, zs + 0.0005)), "cable")]
    s, w = SOCK / 2, SENS_W - 0.0004
    sx = X_SENS0 + 0.008
    t = 0.0035  # the socket's depth: the top layer it is cut through
    zt = zs + SENS_H / 2 - t / 2
    sens_cols = [m.col_box("body_lo", (SENS_L, w, SENS_H - t), (X_SENS0, 0.0, zs - t / 2)),
                 m.col_box("body_a", (sx - s - (X_SENS0 - SENS_L / 2), w, t), ((X_SENS0 - SENS_L / 2 + sx - s) / 2, 0.0, zt)),
                 m.col_box("body_b", (X_SENS0 + SENS_L / 2 - sx - s, w, t), ((sx + s + X_SENS0 + SENS_L / 2) / 2, 0.0, zt)),
                 m.col_box("sock_l", (SOCK, (w - SOCK) / 2, t), (sx, (SOCK + w) / 4, zt)),
                 m.col_box("sock_r", (SOCK, (w - SOCK) / 2, t), (sx, -(SOCK + w) / 4, zt))]
    sensor = m.slider("sensor", "X", X_REAR + 0.016 - X_SENS0, X_FRONT - 0.016 - X_SENS0, sens_parts, sens_cols, mass=0.01, spring=20000.0, damping=20.0,
                      friction=0.0, effort=SLOT_N)
    # the clamp screw's head: a hinge in the sensor, turned in code by the driver's turn while its tip sits in the socket (stated)
    screw = m.hinge("screw", "Z", (sx, 0.0, zt), -math.inf, math.inf, [("head", m.cylinder(0.0018, 0.0006, (sx, 0.0, Z_TOP + 0.0001), segments=6), "steel")],
                    [], mass=0.001, spring=2.0, damping=0.001, friction=0.0, effort=SCREW_T, parent="sensor",
                    inertia=(1e-8, 1e-8, 1e-8))
    led = m.slider("led", "Z", -0.0012, 0.0002, [("led", m.box((0.0025, 0.0020, 0.0012), (X_SENS0 - 0.010, 0.0, Z_TOP - 0.0006)), "led")], [],
                   mass=0.001, spring=200.0, damping=0.5, friction=0.0, effort=5.0, parent="sensor")
    return m.mechanism("cylinder", (0.0, CY, TOP), [rod_link, sensor, screw, led], LOOKS, frame_parts=parts, frame_colliders=cols, friction=(0.6, 0.5))


def _driver() -> m.Part:
    """In the driver's frame: origin at its tip; the blade up +z into the handle. It stands tip-down in its holder."""
    rc = 0.001 / math.cos(math.radians(30.0))  # a 2 mm hex (across flats), flats facing 0, 60, ... degrees
    blade = m.prism([(rc * math.cos(math.radians(30 + 60 * k)), rc * math.sin(math.radians(30 + 60 * k))) for k in range(6)], 0.0, BLADE_L)
    handle = m.cylinder(HANDLE_R, HANDLE_L, (0.0, 0.0, BLADE_L + HANDLE_L / 2), segments=20)
    parts = [("blade", blade, "steel"), ("handle", handle, "handle")]
    cols = [m.col_hull("blade_c", blade), m.col_hull("handle_c", handle)]
    return m.free("driver", (DRV[0], DRV[1], TOP + 0.002), parts, cols, LOOKS, mass=0.03, friction=(0.8, 0.7))


def _props() -> m.Part:
    parts, cols = [], []
    for k, (dx, dy) in enumerate(((0.004, 0.0), (-0.004, 0.0), (0.0, 0.004), (0.0, -0.004))):  # the driver's holder: a block with a 5 mm hole
        _box(parts, cols, f"holder{k}", (0.003 if dx else 0.011, 0.011 if dx else 0.003, 0.030), (DRV[0] + dx, DRV[1] + dy, 0.015), "wood")
    return m.fixed("props", (0.0, 0.0, TOP), parts, cols, LOOKS, friction=(0.8, 0.7))


OBJECTS = [_cylinder(), _driver(), _props()]


def _plain(o):
    return o.item() if hasattr(o, "item") else str(o)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class Setting:
    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.steps, self.lit, self.prev_yaw, self.turned, self.clamped, self.stripped, self.push_ok, self.push_from = 0, False, None, 0.0, False, False, None, None
        self.rod_hold, self.sens_hold, self.out_slot, self.edges = None, None, False, []
        self.cycle_t, self.cycle_ok_off, self.cycle_ok_on, self.cycle_done, self.clear_for = 0.0, False, False, False, 0
        self.hold, self.last, self.full = 0, {}, False

    def _drive(self, asset, name, target, cap):
        import torch

        j = asset.find_joints(f"{name}_joint")[0][0]
        asset.set_joint_position_target(torch.tensor([[target]], device=asset.device), joint_ids=[j])
        asset.write_joint_effort_limit_to_sim(torch.tensor([[cap]], device=asset.device), joint_ids=[j])
        m._TOUCHED.add(("cylinder", name))

    def step(self, env):
        self.steps += 1
        dt = float(env.unwrapped.step_dt)
        cy = m.state(env, "cylinder")
        asset = env.unwrapped.scene["cylinder"]
        rod_j = cy.joint("rod")
        rod = rod_j + STROKE  # how far out: 0 home, STROKE fully out
        sens = cy.joint("sensor")
        x_s = X_SENS0 + sens
        x_m = X_MAG_HOME + rod
        dist = abs(x_s - x_m)
        if self.lit and dist > BAND_OFF:
            self.lit = False
            self.edges.append(("off", x_s))
        elif not self.lit and dist < BAND_ON:
            self.lit = True
            self.edges.append(("on", x_s))
        self._drive(asset, "led", 0.0 if self.lit else -0.0011, 5.0)
        # the rod's seal friction: held where it is with at most ROD_N; during the scorer's cycle, driven out and back
        if self.cycle_t > 0.0 and not self.cycle_done:
            self.cycle_t += dt
            out = 0.020 if self.cycle_t < 1.5 else 0.0
            self._drive(asset, "rod", out - STROKE, 200.0)
            if 1.0 < self.cycle_t < 1.5 and not self.lit:
                self.cycle_ok_off = True
            if 2.8 < self.cycle_t < 3.6:  # the push check: 10 N along the slot on the clamped sensor
                if self.push_from is None:
                    self.push_from = x_s
                m.push(env, "cylinder", (10.0, 0.0, 0.0), body="sensor")
            if self.cycle_t >= 3.6:
                m.push(env, "cylinder", (0.0, 0.0, 0.0), body="sensor")
                self.push_ok = abs(x_s - self.push_from) < 0.0005
                self.cycle_ok_on = self.lit
                self.cycle_done = True
        else:
            self._drive(asset, "rod", rod_j, ROD_N)
        # the driver's tip in the clamp screw's socket: a turn clockwise from above clamps the sensor
        dv = m.state(env, "driver")
        tip = dv.pos
        sock = np.array([x_s + 0.008, CY, TOP + Z_TOP])
        seated = bool(np.hypot(tip[0] - sock[0], tip[1] - sock[1]) < 0.0015 and -0.004 < tip[2] - sock[2] < 0.0015 and dv.rotation()[2, 2] > 0.98)
        yaw = math.atan2(dv.rotation()[1, 0], dv.rotation()[0, 0])
        if seated and self.prev_yaw is not None:
            self.turned += -_wrap(yaw - self.prev_yaw)  # clockwise from above tightens
        self.prev_yaw = yaw if seated else None
        if not self.clamped and self.turned >= math.radians(CLAMP_DEG):
            self.clamped, self.sens_clamp = True, (self.grip_at if getattr(self, "grip_at", None) is not None else sens)
        if self.turned >= math.radians(STRIP_DEG):
            self.stripped = True
        self._drive(asset, "screw", -self.turned, 1.0)  # the screw's head follows (seen turning)
        if self.turned > math.radians(10.0) and getattr(self, "grip_at", None) is None:
            self.grip_at = sens  # the screw starts to bite: from here the sensor is held where it is (a tightening screw grips progressively)
        if self.turned < math.radians(5.0):
            self.grip_at = None
        hold_at = self.sens_clamp if self.clamped else (self.grip_at if getattr(self, "grip_at", None) is not None else sens)
        self._drive(asset, "sensor", hold_at, 500.0 if (self.clamped or getattr(self, "grip_at", None) is not None) else SLOT_N)
        # the scorer's check: once clamped and the hands are clear for 1 s, cycle the rod
        tips = m.fingertips(env)
        clear = bool(np.min(np.linalg.norm(tips - np.array([x_s, CY, TOP + Z_TOP]), axis=1)) > 0.06) and bool(np.min(np.linalg.norm(tips - np.array([CLEVIS_HOME + rod, CY, TOP + 0.03]), axis=1)) > 0.05)
        clear = clear and not seated  # let go: the driver's tip out of the clamp screw too, so a hand on the driver's handle top does not count as clear
        self.clear_for = self.clear_for + 1 if (clear and self.clamped) else 0
        if self.clear_for * dt >= 1.0 and self.cycle_t == 0.0 and abs(rod) < 0.002:
            self.cycle_t = dt
            self.home_dist = dist
        self.last = {"t": round(self.steps * dt, 2), "rod_mm": round(rod * 1000, 1), "sensor_x_mm": round(x_s * 1000, 1), "off_centre_mm": round((x_s - x_m) * 1000, 2),
                     "lit": self.lit, "seated": seated, "turned_deg": round(math.degrees(self.turned), 1), "clamped": self.clamped, "clear": clear,
                     "cycle": [round(self.cycle_t, 2), self.cycle_ok_off, self.cycle_ok_on, self.cycle_done], "push_ok": self.push_ok, "stripped": self.stripped}
        self.hold = self.hold + 1 if _tier(self) == 1.0 else 0
        self.full = self.hold >= HOLD_STEPS
        self.last["hold"] = self.hold
        step_file = os.environ.get("TASK_STEP_FILE")
        if step_file:
            with open(step_file, "w") as f:
                f.write(f"{self.steps * dt:.2f}")
        if self.steps % 5 == 0:
            print("[sensor] " + json.dumps({"mode": os.environ.get("TASK_DEMO", ""), **self.last}, default=_plain), flush=True)


def _tier(st) -> float:
    """1.0: clamped within 1 mm of the band's middle with the rod home, the scorer's rod cycle turning the LED off and on, the clamp holding a 10 N push,
    hands clear. 0.6: clamped in the band, off its middle. 0.3: clamped in the band with the rod not home. 0: not clamped, clamped outside the band,
    or the screw stripped. (The sensor cannot leave the slot in this model: a slider holds it, stated.)"""
    r = st.last
    if not r or st.stripped or not r["clamped"]:
        return 0.0
    off = abs(r["off_centre_mm"]) / 1000.0
    if not st.cycle_done:
        if off > BAND_ON:
            return 0.0
        return 0.3 if abs(r["rod_mm"]) > 2.0 else 0.6
    home = getattr(st, "home_dist", 1.0)
    if home > BAND_ON:
        return 0.0
    if home <= 0.001 and st.cycle_ok_off and st.cycle_ok_on and st.push_ok and r["clear"]:
        return 1.0
    return 0.6


SET = Setting()
m.on_step(SET.step)
m.on_reset(SET.reset)


def score(objects) -> float:
    if SET.full:
        return 1.0
    return min(_tier(SET), 0.6)


def success(objects) -> bool:
    return SET.full


# -- the scripted demo ----------------------------------------------------------------------------

def demo_script(ctx):
    env = ctx.emb._env
    L, R = ctx.left, ctx.right
    cyl = lambda: m.state(env, "cylinder")  # noqa: E731
    drv = lambda: m.state(env, "driver")  # noqa: E731
    if ctx.mode == "scene":
        yield from L.wait(1.0)
        print("[sensor] END " + json.dumps({"mode": ctx.mode, **SET.last}, default=_plain), flush=True)
        return
    zr = Z_TOP - BODY_H / 2
    # the right hand pushes the rod fully home by its clevis
    clev = np.array([CLEVIS_HOME + cyl().joint("rod") + STROKE, CY, TOP + zr])
    yield from R.move(clev + np.array([0.0, 0.0, 0.06]), grip=d.down(0.0), finger=d.FINGER_OPEN, t=1.5, label="over the clevis")
    yield from R.move(clev + np.array([0.0, 0.0, 0.0133 - 0.004]), t=0.8, label="round the clevis")
    yield from R.close(0.016, force=40.0, label="grip the clevis")
    yield from R.servo(lambda s: np.array([CLEVIS_HOME + max(0.0, STROKE - s * 0.03) - 0.002, CY, TOP + zr + 0.0093]), tol=0.001, timeout=2.5,
                       label="push the rod home")
    yield from R.open(label="let go of the clevis")
    yield from R.move(R.at() + np.array([0.04, -0.04, 0.06]), t=1.0, label="right clear")
    print(f"[sensor] rod home: {json.dumps(SET.last, default=_plain)}", flush=True)
    # the left hand takes the driver from its holder by the handle (fingers closing along x, the drag's direction) and seats its tip in the clamp screw
    h = drv().to_world(np.array([0.0, 0.0, BLADE_L + HANDLE_L / 2]))[0]
    g = d.down(-30.0)  # a 90-degree clamp turn clockwise then ends at yaw -120, inside the left wrist's range
    yield from L.move(h + np.array([0.0, 0.0, 0.05]), grip=g, finger=d.FINGER_OPEN, t=1.5, label="over the driver")
    yield from L.move(h, t=0.8, label="round the handle")
    yield from L.close(2 * HANDLE_R, force=40.0, label="grip the driver")
    yield from L.move(L.at() + np.array([0.0, 0.0, 0.08]), t=1.0, label="the driver out of its holder")
    tip = lambda: drv().pos  # noqa: E731
    sock = lambda: np.array([X_SENS0 + cyl().joint("sensor") + 0.008, CY, TOP + Z_TOP])  # noqa: E731
    above = sock() + np.array([0.0, 0.0, 0.010])
    yield from L.move(L.at() + (above - tip()), t=1.5, label="the tip over the clamp screw")
    yield from L.servo(above, tol=0.0003, timeout=3.0, gain=0.4, read=tip, label="the tip on the screw's axis")
    print(f"[sensor] over the screw: tip minus socket {np.round(1000 * (tip() - sock()), 2)} mm, driver axis {np.round(drv().rotation()[:, 2], 3)}", flush=True)
    yield from L.servo(lambda s: sock() + np.array([0.0, 0.0, 0.010 - min(s / 1.0, 1.0) * 0.007]), tol=0.0003, timeout=2.0, gain=0.4, read=tip, label="the tip down to the screw")
    yield from L.servo(sock() + np.array([0.0, 0.0, 0.003]), tol=0.00025, timeout=2.0, gain=0.4, read=tip, label="the tip on the screw's axis again, 3 mm above")
    yield from L.servo(lambda s: sock() + np.array([0.0, 0.0, 0.003 - min(s / 0.6, 1.0) * 0.006]), tol=0.0004, timeout=1.5, gain=0.4, read=tip, label="the tip into the screw")
    print(f"[sensor] tip seated: {json.dumps(SET.last, default=_plain)}", flush=True)
    print(f"[sensor] tip minus socket {np.round(1000 * (tip() - sock()), 2)} mm, driver axis {np.round(drv().rotation()[:, 2], 3)}, TCP minus tip {np.round(1000 * (L.tcp() - tip()), 1)} mm", flush=True)
    if ctx.mode == "probe":
        return

    def drag_to(x_goal, label, until=None):
        """Drag the sensor along the slot by the seated tip: each step the tip is servoed to just ahead of the socket and 2.2 mm down in it, so it pushes the
        socket's wall, and finds its way back in if it rides out (the goal follows the socket, not the hand)."""
        for _ in range(220):
            x_s = X_SENS0 + cyl().joint("sensor")
            if until is not None and until():
                return
            if until is None and abs(x_s - x_goal) < 0.0003:
                return
            lim = 0.0003 if SET.lit else 0.0008  # slow inside the band, so the LED's edges are caught
            step = float(np.clip(x_goal - x_s, -lim, lim))
            push = step + math.copysign((SOCK - 0.002) / 2, step)  # past the tip's play in the socket, so it bears on the wall it drags by
            yield from L.servo(sock() + np.array([push, 0.0, -0.0022]), tol=0.0003, timeout=0.1, gain=0.5, read=tip, label=label)

    # toward the magnet until the LED lights; on past its far edge until it goes out; back to the band's middle
    if ctx.mode != "offband":  # cheat check offband: clamped where it started, never dragged into the band
        yield from drag_to(X_MAG_HOME + 0.02, "drag toward the light", until=lambda: SET.lit)
        near = SET.edges[-1][1] if SET.edges and SET.edges[-1][0] == "on" else X_SENS0 + cyl().joint("sensor")  # where the LED came on
        print(f"[sensor] lit at {near * 1000:.1f} mm", flush=True)
        yield from drag_to(X_MAG_HOME + 0.02, "drag through the band", until=lambda: not SET.lit)
        far = SET.edges[-1][1] if SET.edges and SET.edges[-1][0] == "off" else X_SENS0 + cyl().joint("sensor")  # where it went out
        print(f"[sensor] out at {far * 1000:.1f} mm", flush=True)
        yield from drag_to((near + far) / 2 - 0.0005 * 0.0, "back to the band's middle")  # the middle of the on and off edges
    print(f"[sensor] centred: {json.dumps(SET.last, default=_plain)}", flush=True)
    # clamp: the tip pressed back down into the screw, then a quarter turn clockwise from above
    t0 = tip()
    if ctx.mode == "unseated":  # cheat check unseated: the same turn with the tip lifted 6 mm, out of the screw
        yield from L.move(L.at() + np.array([0.0, 0.0, 0.006]), t=0.5, label="the tip up out of the screw")
    else:
        yield from L.servo(np.array([t0[0], t0[1], sock()[2] - 0.003]), tol=0.0004, timeout=1.0, gain=0.4, read=tip, label="the tip down in the screw")  # straight down only
    if ctx.mode != "noclamp":  # cheat check noclamp: centred in the band, never clamped
        yield from L.turn(-90.0, t=1.2, label="clamp the screw")
    print(f"[sensor] clamped: {json.dumps(SET.last, default=_plain)}", flush=True)
    yield from L.move(L.at() + np.array([0.0, 0.0, 0.03]), t=0.8, label="the tip straight up out of the screw")
    if ctx.mode != "noclamp":
        yield from L.turn(90.0, t=1.0, label="the wrist back")
    yield from L.move(np.array([DRV[0], DRV[1], TOP + 0.002]) + (L.at() - tip()) + np.array([0.0, 0.0, 0.04]), t=1.5, label="the driver over its holder")
    yield from L.servo(np.array([DRV[0], DRV[1], TOP + 0.034]), tol=0.0008, timeout=1.5, read=tip, label="the tip over the holder")
    yield from L.servo(lambda s: np.array([DRV[0], DRV[1], TOP + 0.034 - min(s / 1.0, 1.0) * 0.030]), tol=0.001, timeout=1.8, read=tip, label="the driver into its holder")
    yield from L.open(label="let go of the driver")
    yield from L.move(L.at() + np.array([0.0, 0.0, 0.08]), t=1.0, label="left clear")
    yield from R.move(R.at(), finger=d.FINGER_CLOSED, t=0.5, label="right fingers closed")  # open, one fingertip hangs within 5 cm of the clevis
    yield from L.wait(6.0)  # the scorer cycles the rod
    tips = m.fingertips(env)
    cy_ = m.state(env, "cylinder")
    sens_pt = np.array([X_SENS0 + cy_.joint("sensor"), CY, TOP + Z_TOP])
    clev_pt = np.array([CLEVIS_HOME + cy_.joint("rod") + STROKE, CY, TOP + 0.03])
    for side, tt in (("left", tips[:12]), ("right", tips[12:])):
        print(f"[sensor] {side} hand: closest fingertip {np.min(np.linalg.norm(tt - sens_pt, axis=1)) * 1000:.0f} mm from the sensor (needs 60), "
              f"{np.min(np.linalg.norm(tt - clev_pt, axis=1)) * 1000:.0f} mm from the clevis (needs 50)", flush=True)
    print("[sensor] END " + json.dumps({"mode": ctx.mode, **SET.last}, default=_plain), flush=True)
