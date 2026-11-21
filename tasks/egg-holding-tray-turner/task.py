"""Serve one round egg out of a covered holding tray onto a muffin heel with a turner: one arm holds the lid off, the other lowers the turner's blade into the
113 mm left at the tray's near end, slides it under the egg along the floor, carries it out over the 65 mm wall and slips it off onto the heel; then the lid goes back.

A 1/3-size gastronorm polycarbonate food pan (325 x 176 mm, 65 mm deep) with its flat lift-off lid (a moulded grip bar at its centre), two round cooked eggs (89 mm across, 12 mm thick,
rounded edges, rigid) at its far end; a solid stainless offset turner (blade 102 mm long x 127 mm wide) lying on the counter; a toasted English muffin heel (95 mm)
on an open sandwich wrapper.

    python demokit.py task.py OUT --mode full
"""

from __future__ import annotations

import json
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import demokit as d  # noqa: E402
import mechkit as m  # noqa: E402
from yam_arena import ik  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Take the cover off the egg tray, put one egg on the muffin with the turner, and put the cover back on the tray."
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 2.0

# the tray, long side along x, at (TX, TY) on the counter
TX, TY = 0.40, 0.02
OUT_L, OUT_W, DEPTH = 0.325, 0.176, 0.065  # outside, at the rim
IN_L, IN_W = 0.296, 0.150  # inside the walls
WALL, FLOOR, FLANGE_T = 0.003, 0.003, 0.003
TRAY_M = 0.36  # the empty pan, loose on the counter
IX0, IX1 = TX - IN_L / 2, TX + IN_L / 2  # inside, along x
IY0, IY1 = TY - IN_W / 2, TY + IN_W / 2
ZF = TOP + FLOOR  # the tray's floor
ZRIM = TOP + DEPTH
# eggs
EGG_R, EGG_T, EGG_M, EGG_FILLET = 0.0445, 0.012, 0.055, 0.003
EGG_X = (IX1 - EGG_R - 0.002 - 2 * EGG_R - 0.003, IX1 - EGG_R - 0.002)  # the served one first, then the one left at the far wall
# lid
LID_T, KNOB_R, KNOB_H, LID_M = 0.003, 0.0125, 0.030, 0.22
KNOB_L = 0.060  # the lid's moulded grip bar: 60 mm along the pan, 25 mm across (2 x KNOB_R), 30 mm high
# turner: its frame has the blade's underside at z = 0, the blade's centre at the origin, the tip at +x
BL, BW, BT, BT_TIP = 0.102, 0.127, 0.0025, 0.0006
NECK_DX, NECK_DZ = 0.030, 0.065  # the offset: the handle is 65 mm above the blade, 30 mm behind its back edge
HL, HW, HH = 0.130, 0.022, 0.014
TURNER_M = 0.17
TURNER0 = np.array([0.50, -0.16])  # the blade's centre, lying on the counter, tip along +x
# muffin heel on its wrapper
MUF_R, MUF_H, MUF_M = 0.0475, 0.022, 0.030
MUF0 = np.array([0.36, -0.30])
WRAP = 0.18
MU_EGG, MU_STEEL, MU_TRAY, MU_MUF = 0.25, 0.25, 0.35, 0.90

LOOKS = {"pc": m.Look(color=m.rgb(0.86, 0.84, 0.78), roughness=0.25, opacity=0.55) if "opacity" in m.Look.__dataclass_fields__ else m.Look(color=m.rgb(0.86, 0.84, 0.78), roughness=0.25),
         "lid": m.Look(color=m.rgb(0.80, 0.80, 0.76), roughness=0.3),
         "white": m.Look(color=m.rgb(0.97, 0.96, 0.92), roughness=0.6), "yolk": m.Look(color=m.rgb(0.98, 0.72, 0.18), roughness=0.5),
         "steel": m.Look(color=m.rgb(0.72, 0.73, 0.74), metallic=0.85, roughness=0.3), "handle": m.Look(color=m.rgb(0.10, 0.10, 0.11), roughness=0.6),
         "crumb": m.Look(color=m.rgb(0.92, 0.85, 0.68), roughness=0.9), "toast": m.Look(color=m.rgb(0.72, 0.50, 0.26), roughness=0.85),
         "paper": m.Look(color=m.rgb(0.93, 0.90, 0.82), roughness=0.95)}


def _box(parts, cols, name, size, centre, look):
    parts.append((name, m.box(size, centre), look))
    cols.append(m.col_box(f"{name}_c", size, centre))


def _tray() -> m.Part:
    parts, cols = [], []
    ow, ol = IN_W + 2 * WALL, IN_L + 2 * WALL
    _box(parts, cols, "floor", (ol, ow, FLOOR), (0.0, 0.0, FLOOR / 2), "pc")
    h = DEPTH - FLOOR - FLANGE_T
    for k, s in enumerate((-1, 1)):
        _box(parts, cols, f"wx{k}", (WALL, ow, h), (s * (IN_L / 2 + WALL / 2), 0.0, FLOOR + h / 2), "pc")
        _box(parts, cols, f"wy{k}", (ol, WALL, h), (0.0, s * (IN_W / 2 + WALL / 2), FLOOR + h / 2), "pc")
    zf = DEPTH - FLANGE_T / 2  # the flange round the top, out to 325 x 176
    fx, fy = (OUT_L - IN_L) / 2, (OUT_W - IN_W) / 2
    for k, s in enumerate((-1, 1)):
        _box(parts, cols, f"fx{k}", (fx, OUT_W, FLANGE_T), (s * (IN_L / 2 + fx / 2), 0.0, zf), "pc")
        _box(parts, cols, f"fy{k}", (IN_L, fy, FLANGE_T), (0.0, s * (IN_W / 2 + fy / 2), zf), "pc")
    return m.free("tray", (TX, TY, TOP + 0.0003), parts, cols, LOOKS, mass=TRAY_M, friction=(MU_TRAY, MU_TRAY), torsional_patch=None)


def _lid() -> m.Part:
    """Flat lid on the flange, with a 6 mm skirt just inside the walls that keeps it seated, and a grip bar moulded at its centre. Origin: the lid's underside."""
    parts, cols = [], []
    _box(parts, cols, "plate", (OUT_L, OUT_W, LID_T), (0.0, 0.0, LID_T / 2), "lid")
    sk, g = 0.006, 0.004  # the skirt's 6 mm drop and its clearance to the walls
    for k, s in enumerate((-1, 1)):
        _box(parts, cols, f"skx{k}", (0.002, IN_W - 2 * g - 0.004, sk), (s * (IN_L / 2 - g - 0.001), 0.0, -sk / 2), "lid")
        _box(parts, cols, f"sky{k}", (IN_L - 2 * g - 0.004, 0.002, sk), (0.0, s * (IN_W / 2 - g - 0.001), -sk / 2), "lid")
    _box(parts, cols, "grip", (KNOB_L, 2 * KNOB_R, KNOB_H), (0.0, 0.0, LID_T + KNOB_H / 2), "lid")
    return m.free("lid", (TX, TY, ZRIM + 0.0006), parts, cols, LOOKS, mass=LID_M, friction=(0.6, 0.5), torsional_patch=None)


def _egg_mesh(r=EGG_R, t=EGG_T, f=EGG_FILLET, n=6):
    prof = [(0.0, -t / 2)]
    for k in range(n + 1):  # rounded bottom edge
        a = -np.pi / 2 + k * (np.pi / 2) / n
        prof.append((r - f + f * np.cos(a), -t / 2 + f + f * np.sin(a)))
    for k in range(1, n + 1):  # rounded top edge
        a = k * (np.pi / 2) / n
        prof.append((r - f + f * np.cos(a), t / 2 - f + f * np.sin(a)))
    prof.append((0.0, t / 2))
    return m.lathe(prof, 32)


def _egg(name, x) -> m.Part:
    white = _egg_mesh()
    yolk = m.cylinder(0.024, 0.0015, (0.006, 0.004, EGG_T / 2 + 0.0004), segments=24)
    return m.free(name, (x, TY, ZF + EGG_T / 2 + 0.0007), [("white", white, "white"), ("yolk", yolk, "yolk")], [m.col_hull("c", _egg_mesh(n=4))], LOOKS,
                  mass=EGG_M, friction=(MU_EGG, MU_EGG), torsional_patch=None)


def _blade_mesh():
    xb, xt = -BL / 2, BL / 2
    outline = np.array([[xb, 0.0], [xt, 0.0], [xt, BT_TIP], [xt - 0.012, BT], [xb, BT]])  # xz: bevelled front edge
    msh = m.prism(outline, -BW / 2, BW / 2)  # extruded along its third axis
    return msh.turned(90.0, "x")


def _turner_geom():
    xb = -BL / 2
    neck = m.tube([(xb + 0.002, 0.0, BT + 0.004), (xb - NECK_DX * 0.5, 0.0, NECK_DZ * 0.55), (xb - NECK_DX, 0.0, NECK_DZ)], 0.004, sides=10)
    hx = xb - NECK_DX - HL / 2
    handle = m.box((HL, HW, HH), (hx, 0.0, NECK_DZ))
    return neck, hx, handle


def _turner() -> m.Part:
    blade = _blade_mesh()
    neck, hx, handle = _turner_geom()
    parts = [("blade", blade, "steel"), ("neck", neck, "steel"), ("handle", handle, "handle")]
    cols = [m.col_hull("blade_c", blade),
            m.col_hull("neck_c", neck),
            m.col_box("handle_c", (HL, HW, HH), (hx, 0.0, NECK_DZ))]
    # lying on the counter on its blade's back edge and its handle's end (the handle outweighs the thin blade)
    a = np.arctan2(NECK_DZ - HH / 2, (BL / 2 + NECK_DX + HL) - BL / 2)
    q = (0.0, float(np.sin(-a / 2)), 0.0, float(np.cos(-a / 2)))  # tip up by a
    rot = m.quat_matrix(q)
    back = rot @ np.array([-BL / 2, 0.0, 0.0])
    pos = np.array([TURNER0[0], TURNER0[1], TOP + 0.0006]) - np.array([0.0, 0.0, back[2]])
    return m.free("turner", tuple(pos), parts, cols, LOOKS, mass=TURNER_M, friction=(MU_STEEL, MU_STEEL), torsional_patch=None, start_quat=q)


def _muffin() -> m.Part:
    crumb = m.cylinder(MUF_R, MUF_H - 0.004, (0.0, 0.0, (MUF_H - 0.004) / 2), segments=32)
    toast = m.cylinder(MUF_R - 0.003, 0.004, (0.0, 0.0, MUF_H - 0.002), segments=32)
    return m.free("muffin", (MUF0[0], MUF0[1], TOP + 0.0012), [("crumb", crumb, "toast"), ("top", toast, "crumb")], [m.col_cylinder("c", MUF_R, MUF_H, (0.0, 0.0, MUF_H / 2))], LOOKS,
                  mass=MUF_M, friction=(MU_MUF, MU_MUF))


def _wrapper() -> m.Part:
    parts = [("sheet", m.box((WRAP, WRAP, 0.001), (0.0, 0.0, 0.0005)), "paper")]
    return m.fixed("wrapper", (MUF0[0], MUF0[1], TOP), parts, [m.col_box("c", (WRAP, WRAP, 0.001), (0.0, 0.0, 0.0005))], LOOKS, friction=(0.6, 0.5))


OBJECTS = [_tray(), _lid(), _egg("egg", EGG_X[0]), _egg("egg2", EGG_X[1]), _turner(), _wrapper(), _muffin()]


def blade_points(k):
    """The blade's outline corners on its top face, in the world."""
    return k.to_world(np.array([[sx * BL / 2, sy * BW / 2, BT] for sx in (-1, 1) for sy in (-1, 1)]))


def in_tray(tr, p, margin=0.0):
    """Inside the tray's walls (plus `margin`), in the tray's own frame: the tray is loose on the counter."""
    loc = tr.to_local(np.asarray(p)[None])[0]
    return abs(loc[0]) <= IN_L / 2 + margin and abs(loc[1]) <= IN_W / 2 + margin


def below_rim(tr, p):
    return tr.to_local(np.asarray(p)[None])[0][2] < DEPTH


class Tracker:
    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.steps, self.last = 0, {}
        self.void = ""
        self.support, self.off_steps, self.path_ok = "tray", 0, True
        self.bladed, self.blade_h = False, 0.0  # the egg has ridden the blade; its bottom height over the heel top at the last blade step

    def egg_on_blade(self, e, k):
        loc = k.to_local(e.pos[None])[0]
        return abs(loc[0]) <= BL / 2 + 0.01 and abs(loc[1]) <= BW / 2 + 0.01 and abs(loc[2] - (BT + EGG_T / 2)) <= 0.004

    def step(self, env):
        self.steps += 1
        dt = float(env.unwrapped.step_dt)
        e, e2, lid, k, mu, tr = (m.state(env, n) for n in ("egg", "egg2", "lid", "turner", "muffin", "tray"))
        corners = lid.to_world(np.array([[sx * OUT_L / 2, sy * OUT_W / 2, 0.0] for sx in (-1, 1) for sy in (-1, 1)]))
        lid_low = float(corners[:, 2].min())
        rim = tr.pos[2] + DEPTH
        lid_held = lid_low >= rim + 0.010 and np.hypot(*(lid.pos[:2] - tr.pos[:2])) >= 0.10  # off its seat, clear of the tray, now
        if lid_low < TOP + 0.020 and not in_tray(tr, lid.pos, 0.03):
            self.void = self.void or "lid set down off the tray"
        ll = tr.to_local(lid.pos[None])[0]
        seated = abs(ll[2] - DEPTH) < 0.002 and abs(ll[0]) < 0.008 and abs(ll[1]) < 0.008 and lid.tilt_deg() < 1.0  # down on its seat, not propped on a wall
        for egg in (e, e2):
            if egg.pos[2] - EGG_T / 2 < TOP + 0.004 and not in_tray(tr, egg.pos, 0.01) and np.hypot(*(egg.pos[:2] - mu.pos[:2])) > MUF_R:
                self.void = self.void or "an egg touched the counter"
        if not (in_tray(tr, e2.pos) and below_rim(tr, e2.pos)):
            self.void = self.void or "the second egg left the tray"
        on_blade = self.egg_on_blade(e, k)
        muf_top = mu.pos[2] + MUF_H
        loc = mu.to_local(e.pos[None])[0]
        on_muffin = abs(loc[2] - (MUF_H + EGG_T / 2)) <= 0.004 and np.hypot(loc[0], loc[1]) <= MUF_R
        if in_tray(tr, e.pos) and below_rim(tr, e.pos):
            where = "tray"
        elif on_blade:
            where = "blade"
            self.bladed = True
            self.blade_h = float(e.pos[2] - EGG_T / 2 - muf_top)
        elif on_muffin or (np.hypot(loc[0], loc[1]) <= MUF_R and MUF_H - 0.002 <= loc[2] - EGG_T / 2 <= MUF_H + 0.03):
            where = "muffin"  # on the heel, or coming down onto it off the blade
            if self.support != "muffin" and (not self.bladed or self.blade_h > 0.03):
                self.path_ok = False  # skipped the blade, or slid off it from more than 3 cm above the heel top (dropped)
            if self.support not in ("blade", "muffin") and self.off_steps * dt > 0.5:
                self.path_ok = False
        else:
            where = "other"
        if where != "tray" and self.support == "tray" and not lid_held:  # the moment the egg comes out of the tray
            self.void = self.void or "the egg came out with the lid not held clear"
        if where == "other":
            self.off_steps += 1
            if self.off_steps * dt > 0.5:
                self.path_ok = False  # carried, pushed or dropped by anything but the turner's blade
        else:
            self.off_steps = 0
            self.support = where
        off = float(np.hypot(*(e.pos[:2] - mu.pos[:2])))
        out_on_blade = where == "blade" and not in_tray(tr, e.pos, 0.02)
        heel_ok = bool(on_muffin) and where == "muffin" and e.tilt_deg() <= 10.0 and off <= 0.012
        self.last = {"egg_mm": [round(v * 1000, 1) for v in e.pos], "egg_tilt": round(e.tilt_deg(), 1), "where": where, "egg_muffin_mm": round(off * 1000, 1),
                     "muffin_top_mm": round((muf_top - TOP) * 1000, 1), "lid_low_mm": round((lid_low - TOP) * 1000, 1), "lid_seated": bool(seated),
                     "lid_held": bool(lid_held), "out_on_blade": bool(out_on_blade), "path_ok": self.path_ok, "on_muffin": bool(on_muffin), "heel_ok": heel_ok, "void": self.void,
                     "blade_tilt": round(k.tilt_deg(), 1), "egg2_in": bool(in_tray(tr, e2.pos)), "tray_moved_mm": round(float(np.hypot(tr.pos[0] - TX, tr.pos[1] - TY)) * 1000, 1),
                     "lid_in_tray_mm": [round(v * 1000, 1) for v in ll], "lid_tilt": round(lid.tilt_deg(), 1), "tray_tilt": round(tr.tilt_deg(), 1)}
        if self.steps % 20 == 0:
            print("[egg] " + json.dumps({"mode": os.environ.get("TASK_DEMO", ""), "t": round(self.steps * dt, 2), **self.last}), flush=True)


TRACK = Tracker()
m.on_step(TRACK.step)
m.on_reset(TRACK.reset)


def score(objects) -> float:
    """End state only; the void and the path are failures that stay once met. 0.3 the egg out of the tray on the blade; 0.6 the egg on the heel;
    1.0 that and the lid seated with the other egg still in the tray."""
    r = TRACK.last
    if not r or r["void"] or not r["path_ok"]:
        return 0.0
    if r["heel_ok"]:
        return 1.0 if (r["lid_seated"] and r["egg2_in"]) else 0.6
    return 0.3 if r["out_on_blade"] else 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0


# -- the scripted demo ----------------------------------------------------------------------------

def _tcp_pose(ctx, side):
    return ik.tcp_pose(ctx.observed(side)[:6], ctx.arm(side).base)


def _rot_y(deg):
    a = np.radians(deg)
    return np.array([[np.cos(a), 0.0, np.sin(a)], [0.0, 1.0, 0.0], [-np.sin(a), 0.0, np.cos(a)]])


class Hold:
    """The turner as held: where the TCP sits in the turner's frame, read from the observed pose after the grip."""

    def __init__(self, ctx):
        k = ctx.state("turner")
        p, r = _tcp_pose(ctx, "right")
        rk = k.rotation()
        self.p_rel = rk.T @ (p - k.pos)
        self.r_rel = rk.T @ r

    def tcp_for(self, tip, pitch_down):
        """The TCP pose that puts the blade's tip (underside, centre) at `tip` with the blade pitched `pitch_down` degrees tip-down, square to x."""
        rk = _rot_y(pitch_down)
        origin = np.asarray(tip, float) - rk @ np.array([BL / 2, 0.0, 0.0])
        return origin + rk @ self.p_rel, d._as_grip(rk @ self.r_rel)


def _knob_now(ctx, knob):
    """The grip bar's seated place over the tray where the tray is now (it may have been nudged)."""
    return knob + (ctx.state("tray").pos - np.array([TX, TY, TOP + 0.0003]))


def _lid_down(ctx, knob):
    """Line the held lid up over the tray by where it is seen to be (it shifts in the hand when the fingers close), then lower it onto its seat."""
    L = ctx.left
    tgt = _knob_now(ctx, knob) + np.array([0.0, 0.0, 0.04])
    for _ in range(2):
        lid, tr = ctx.state("lid"), ctx.state("tray")
        err = tr.pos[:2] - lid.pos[:2]
        tgt = tgt + np.array([err[0], err[1], 0.0])
        yield from L.move(tgt, t=0.6, label="line the lid up over the tray")
    yield from L.move(tgt - np.array([0.0, 0.0, 0.036]), t=1.2, label="lid down on the tray")


def _lid_grip():
    """The lid arm's grip on the grip bar: fingers closing across it (along y), the hand tilted in the x-z plane, which keeps the lid hold within reach."""
    tilt = float(os.environ.get("EGG_LIDTILT", "25"))
    return d.tilted(abs(tilt), 0.0 if tilt >= 0 else 180.0) if tilt else d.down(0.0)


def _grab_egg(ctx, knob, f=1.0):
    """Lazy route: the lid lifted and held as in the demo, the egg pinched across by the right fingers and set on the heel, the lid put back."""
    L, R = ctx.left, ctx.right
    e = ctx.state("egg").pos
    yield from ctx.together(L.move(knob + np.array([0.0, 0.0, 0.07]), grip=_lid_grip(), finger=d.FINGER_OPEN, t=2.5, label="over the knob"),
                            R.move(e + np.array([0.0, 0.0, 0.12]), grip=d.down(0.0), finger=d.FINGER_OPEN, t=2.5, label="lazy: over the egg"))
    yield from L.move(knob, t=1.0, label="round the knob")
    yield from L.close(2 * KNOB_R, force=30.0, label="grip the knob")
    yield from L.move(L.at() + np.array([0.0, 0.0, 0.05]), t=1.0, label="lift the lid")
    yield from L.move(np.array([0.34, 0.21, TOP + 0.125]), t=2.0, label="hold the lid clear")
    yield from R.move(e + np.array([0.0, 0.0, 0.004]), t=1.5, label="lazy: fingers down round the egg")
    yield from R.close(2 * EGG_R, force=20.0, label="lazy: pinch the egg")
    mu = ctx.state("muffin").pos
    yield from R.move(R.at() + np.array([0.0, 0.0, 0.10]), t=1.0 * f, label="lazy: lift the egg")
    yield from R.move(np.array([mu[0], mu[1], mu[2] + MUF_H + EGG_T / 2 + 0.05]), t=2.0 * f, label="lazy: over the heel")
    yield from R.move(np.array([mu[0], mu[1], mu[2] + MUF_H + EGG_T / 2 + 0.002]), t=1.0 * f, label="lazy: down on the heel")
    yield from R.open(label="lazy: let go")
    yield from R.move(R.at() + np.array([0.0, 0.0, 0.08]), t=1.0, label="lazy: clear")
    yield from L.move(_knob_now(ctx, knob) + np.array([0.0, 0.0, 0.04]), t=2.5, label="lid back over the tray")
    yield from _lid_down(ctx, knob)
    yield from L.open(label="let go of the knob")
    yield from L.move(L.at() + np.array([0.0, 0.0, 0.08]), t=1.0, label="clear")
    yield from L.wait(2.5)
    print("[egg] END " + json.dumps({"mode": ctx.mode, **TRACK.last}), flush=True)


def demo_script(ctx):
    L, R = ctx.left, ctx.right
    lid = ctx.state("lid")
    knob = lid.pos + np.array([0.0, 0.0, LID_T + KNOB_H - 0.010])
    k = ctx.state("turner")
    a = np.degrees(np.arctan2(k.rotation()[2, 0], k.rotation()[0, 0]))  # the handle's rise toward the tip
    grip_pt = k.to_world(np.array([[-BL / 2 - NECK_DX - 0.035, 0.0, NECK_DZ]]))[0]
    if ctx.mode == "none":  # lazy route: nothing done
        yield from L.wait(3.0)
        print("[egg] END " + json.dumps({"mode": ctx.mode, **TRACK.last}), flush=True)
        return
    if ctx.mode in ("grab_egg", "pinch_fast"):  # lazy routes: the egg taken out by the fingers, no turner (fast: quick enough to stay under the off-path time)
        yield from _grab_egg(ctx, knob, 0.2 if ctx.mode == "pinch_fast" else 1.0)
        return
    lean = float(os.environ.get("EGG_LEAN", "40"))  # the hand leans forward of square to the handle: the reach into the tray wants it
    rgrip = d.tilted(a + lean, 0.0)
    yield from ctx.together(L.move(knob + np.array([0.0, 0.0, 0.07]), grip=_lid_grip(), finger=d.FINGER_OPEN, t=2.5, label="over the knob"),
                            R.move(grip_pt + 0.07 * np.array([0.0, 0.0, 1.0]), grip=rgrip, finger=d.FINGER_OPEN, t=2.5, label="over the handle"))
    yield from ctx.together(L.move(knob, t=1.0, label="round the knob"), R.move(grip_pt, t=1.0, label="round the handle"))
    yield from ctx.together(L.close(2 * KNOB_R, force=30.0, label="grip the knob"), R.close(HW, force=40.0, label="grip the handle"))
    # the lid straight up, then off to the left, held level
    if ctx.mode == "slide_lid":  # lazy route: the lid slid half off along the rim, never lifted
        yield from L.move(L.at() + np.array([0.0, 0.12, 0.001]), t=1.5, label="lazy: slide the lid along the rim")
        yield from L.open(label="lazy: let go")
        yield from L.move(L.at() + np.array([0.0, 0.0, 0.06]), t=1.0, label="lazy: clear")
    else:
        yield from L.move(L.at() + np.array([0.0, 0.0, 0.05]), t=1.0, label="lift the lid")
    hold = Hold(ctx)
    pitch = float(os.environ.get("EGG_PITCH", "20"))
    tip_in = np.array([IX0 + 0.106, TY, ZF + 0.0005])
    p_up, g_up = hold.tcp_for(tip_in + np.array([0.0, 0.0, 0.10]), pitch)
    yield from ctx.together(L.move(np.array([0.34, 0.21, TOP + 0.125]) if ctx.mode != "slide_lid" else L.at(), t=2.0, label="hold the lid clear"),
                            R.move(R.at() + np.array([0.0, 0.0, 0.08]), t=1.0, label="lift the turner"))
    if ctx.mode == "lid_counter":  # lazy route: the lid set down on the counter
        yield from L.move(np.array([0.34, 0.21, TOP + 0.028]), t=1.5, label="lazy: lid down on the counter")
        yield from L.open(label="lazy: let go")
    yield from R.move(p_up, grip=g_up, t=2.0, label="blade over the tray's empty end")
    p_in, g_in = hold.tcp_for(tip_in, pitch)
    yield from R.move(p_in, grip=g_in, t=1.5, label="blade down to the floor")
    egg = ctx.state("egg").pos
    tip_far = np.array([egg[0] + EGG_R - 0.004, TY, ZF - 0.001])
    p_far, g_far = hold.tcp_for(tip_far, pitch)
    yield from R.move(p_far, grip=g_far, t=2.5, label="slide under the egg")
    p_lift, g_lift = hold.tcp_for(tip_far + np.array([0.0, 0.0, 0.015]), 4.0)
    yield from R.move(p_lift, grip=g_lift, t=1.0, label="flatten and lift")
    p_hi, _ = hold.tcp_for(tip_far + np.array([0.0, 0.0, 0.085]), 4.0)
    yield from R.move(p_hi, t=1.2, label="lift clear of the wall")
    p_back, _ = hold.tcp_for(np.array([IX0 - 0.03, TY, ZRIM + 0.025]) + np.array([BL / 2 + 0.01, 0.0, 0.0]), 4.0)
    yield from R.move(p_back, t=1.5, label="back out over the near wall")
    # over the heel: the blade's tip over the muffin's near half, then tipped down so the egg slides forward off it onto the heel
    yield from R.wait(0.3)
    mu = ctx.state("muffin").pos
    k, e = ctx.state("turner"), ctx.state("egg").pos
    egg_on_blade = k.to_local(e[None])[0]
    setback = float(os.environ.get("EGG_SETBACK", "0.0"))
    tip_set = np.array([mu[0] - setback, mu[1] - egg_on_blade[1], mu[2] + MUF_H + 0.006])
    if ctx.mode == "drop_high":  # lazy route: the egg tipped off the blade from well above the heel
        tip_set = tip_set + np.array([0.0, 0.0, 0.10])
    p_ov, g_ov = hold.tcp_for(tip_set + np.array([0.0, 0.0, 0.05]), 1.0)
    yield from R.move(p_ov, grip=g_ov, t=2.5, label="carry to over the heel")
    p_set, _ = hold.tcp_for(tip_set, 1.0)
    yield from R.move(p_set, t=1.0, label="down over the heel")
    tilt = float(os.environ.get("EGG_TILT", "26"))
    for a in np.linspace(1.0, tilt, 12)[1:]:
        p_t, g_t = hold.tcp_for(tip_set - np.array([0.0, 0.0, 0.002]), a)
        yield from R.move(p_t, grip=g_t, t=0.12, label="tip the blade down")
    yield from R.wait(0.6)
    # the egg rests on the tipped blade: shift the blade along until the egg's centre is over the heel's, by the egg's observed place
    e, mu = ctx.state("egg").pos, ctx.state("muffin").pos
    bias = float(os.environ.get("EGG_BIAS", "0.005"))  # the egg is drawn back a few mm as the blade comes out
    shift = np.array([float(np.clip(mu[0] + bias - e[0], -0.05, 0.05)), float(np.clip(mu[1] - e[1], -0.02, 0.02)), 0.0])
    tip_set = tip_set + shift
    p_s, g_s = hold.tcp_for(tip_set - np.array([0.0, 0.0, 0.002]), tilt)
    yield from R.move(p_s, grip=g_s, t=1.0, label="line the egg up over the heel")
    yield from R.wait(0.4)
    rk = _rot_y(tilt)
    back = -rk @ np.array([1.0, 0.0, 0.0])  # straight back along the blade, out from under the egg's rear
    p_b, _ = hold.tcp_for(tip_set - np.array([0.0, 0.0, 0.002]) + 0.07 * back, tilt)
    yield from R.move(p_b, t=1.2, label="draw the blade back out")
    if ctx.mode in ("lid_off", "slide_lid", "lid_counter", "drop_high"):  # lazy route: the lid not put back
        yield from R.move(R.at() + np.array([-0.03, 0.0, 0.06]), t=1.0, label="turner up and away")
        yield from L.wait(3.0)
        print("[egg] END " + json.dumps({"mode": ctx.mode, **TRACK.last}), flush=True)
        return
    yield from ctx.together(R.move(R.at() + np.array([-0.03, 0.0, 0.06]), t=1.0, label="turner up and away"),
                            L.move(_knob_now(ctx, knob) + np.array([0.0, 0.0, 0.04]), t=2.5, label="lid back over the tray"))
    yield from _lid_down(ctx, knob)
    yield from L.open(label="let go of the knob")
    yield from L.move(L.at() + np.array([0.0, 0.0, 0.08]), t=1.0, label="clear")
    yield from L.wait(2.5)
    print("[egg] END " + json.dumps({"mode": ctx.mode, **TRACK.last}), flush=True)
