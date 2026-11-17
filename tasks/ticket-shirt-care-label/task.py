"""Ticket a garment through its care label only: lift the label clear of the body fabric, push a tagging gun's needle through the label alone, fire one
fastener and draw the needle back.

A child's cotton T-shirt lies flat on the table, front up, and the care label sewn into its side seam has been drawn out onto the front panel (30 x 60 mm,
sewn along its short edge, its free edge curled up about 8 mm). A standard tagging gun (Avery Dennison Mark III class: 125 x 25 x 130 mm, 100 g by a
retailer's listing) lies on its side beside it with a card price ticket on its needle.

Sim (stated, all in code): the shirt is two sewn layers of native surface-FEM cloth (front and back, 0.6 mm, 180 g/m2; gravity, friction and self-contact), and the label
a small rigid flap hinged along its sewn edge, its curl a light spring (0.01 N m/rad toward 8 degrees). The needle has no collider: the scorer checks its tip against the
shirt's outline and top face (a pierce, recorded as a failure)
and against the label's (the passage: its tip must cross the label's face inside the outline, gated on the label standing at least 2 cm clear of the
body; a needle slid in sideways never counts, and one dragged sideways out through the label's edge tears it). The trigger is a sprung squeeze lever along the grip's
front (25 N preload); a fastener fires when it travels 2.5 mm with the needle through the label.

Full score: one fastener through the label alone, the body never pierced, the needle drawn back out, the label let go (down under 30 degrees, every
fingertip of both hands 5 cm from it), held 1 s. Partial: 0.5 the needle through the label alone; 0.25 the label lifted 2 cm at its free edge. A tip
inside the shirt's body, or within 4 mm above its top face, is a pierce.

    python demokit.py task.py OUT --mode scene --stills 1
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
INSTRUCTION = ("Put the price ticket on this shirt through its care label only: lift the label clear of the shirt with one hand, push the tagging gun's "
               "needle through the label alone with the other, fire one fastener, draw the needle back, then let the label down and let go of it.")
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0
HOLD_STEPS = 20

# The T-shirt lies flat, front up: two sewn surface-FEM layers (README). x runs from the hem (X_HEM) to the shoulders (X_SHO), y across; the side seam is at y = Y_SIDE.
X_HEM, X_SHO, Y_SIDE, BODY_W = 0.18, 0.48, -0.005, 0.28
SLEEVE_X0, SLEEVE_L = 0.38, 0.09  # the sleeves run out sideways from the shoulder end, 10 cm along x, 9 cm out
NECK = (0.44, 0.095, 0.175)  # the neck opening: x from 0.44 to the shoulder edge, y from 0.095 to 0.175
SHIRT = np.array([0.32, 0.005])  # the mechanism frame's origin: the label's sewn edge, mid-width
BODY_T = float(os.environ.get("SHIRT_TOP", "0.0025"))  # the shirt's settled top face over the table (two layers, measured in the settle test)
SHIRT_SIZE = (X_SHO - X_HEM, BODY_W, BODY_T)
LABEL_W, LABEL_L, LABEL_T = 0.030, 0.060, 0.0006
SEAM = np.array([0.32, 0.005, SHIRT_SIZE[2]])  # the label's sewn edge, along x, on the side seam, drawn out onto the front panel
GUN0 = np.array([0.12, -0.16])  # where the gun lies, its grip's middle
FWD, UP = 0.115, 0.06
NEEDLE_L = 0.025  # the needle's length beyond the gun's nose
PIERCE_U = 0.037  # where the demo pierces the label: 37 mm out from its sewn edge, 3.7 cm clear of the body with the label held at about 82 degrees (LIFT_DEG asks 60; the pinch carries it to 82 in the run)
HOLD_U = 0.048  # where the left pads pinch the label: 12 mm in from its free edge
CURL_DEG, CURL_K = 8.0, 0.01  # the free edge's curl: the label rests 8 degrees up (its free edge about 8 mm up) and springs back toward it at 0.01 N m/rad (unmeasured: an approximation of a satin label's set)

LOOKS = {"cotton": m.Look(color=m.rgb(0.10, 0.18, 0.36), roughness=0.95), "hem": m.Look(color=m.rgb(0.08, 0.15, 0.31), roughness=0.95),
         "label": m.Look(color=m.rgb(0.94, 0.94, 0.90), roughness=0.45), "print": m.Look(color=m.rgb(0.35, 0.35, 0.38), roughness=0.6),
         "gun": m.Look(color=m.rgb(0.82, 0.83, 0.85), roughness=0.5), "grip": m.Look(color=m.rgb(0.20, 0.21, 0.23), roughness=0.6),
         "trigger": m.Look(color=m.rgb(0.75, 0.12, 0.10), roughness=0.5), "steel": m.Look(color=m.rgb(0.75, 0.76, 0.78), metallic=1.0, roughness=0.25),
         "ticket": m.Look(color=m.rgb(0.97, 0.96, 0.92), roughness=0.7), "price": m.Look(color=m.rgb(0.80, 0.10, 0.10), roughness=0.6),
         "fastener": m.Look(color=m.rgb(0.92, 0.92, 0.92), roughness=0.4)}


SHIRT_GSM, SHIRT_THICK = 0.18, 0.0006  # 180 g/m2 cotton jersey, 0.6 mm per layer
SHIRT_YOUNG = float(os.environ.get("SHIRT_YOUNG", "2e6"))  # Pa, effective in-plane (unmeasured: an approximation of knitted cotton)
SHIRT_BEND = float(os.environ.get("SHIRT_BEND", "1.8e5"))
LAYER_GAP = 0.0016
CELL = 0.01


def _tee_cells():
    """The tee's outline on a 1 cm grid: the body, two short sleeves out from the shoulder end, the neck notch cut away."""
    cells = set()
    nx = int(round((X_SHO - X_HEM) / CELL))
    y_lo, y_hi = Y_SIDE - SLEEVE_L, Y_SIDE + BODY_W + SLEEVE_L
    ny = int(round((y_hi - y_lo) / CELL))
    for i in range(nx):
        x = X_HEM + (i + 0.5) * CELL
        for j in range(ny):
            y = y_lo + (j + 0.5) * CELL
            body = Y_SIDE < y < Y_SIDE + BODY_W
            sleeve = x > SLEEVE_X0
            neck = x > NECK[0] and NECK[1] < y < NECK[2]
            if (body or sleeve) and not neck:
                cells.add((i, j))
    return cells, nx, ny, y_lo


def _build_cloth(usd_path: str) -> None:
    """Write the shirt: a front and a back layer of a flat tee, each a triangle mesh in Isaac's surface-deformable (FEM) body, sewn along the shoulders, sleeve
    undersides and side seams by narrow strips joining their edges; the neck, cuffs and hem stay open. One body, so the layers collide with each other."""
    from collections import Counter

    from omni.physx.scripts import deformableUtils as du
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdShade

    cells, nx, ny, y_lo = _tee_cells()
    ids, uv = {}, []

    def vid(i, j):
        if (i, j) not in ids:
            ids[(i, j)] = len(uv)
            uv.append((X_HEM + i * CELL, y_lo + j * CELL))
        return ids[(i, j)]

    tris = []
    for (i, j) in sorted(cells):
        a, b, c, d = vid(i, j), vid(i + 1, j), vid(i + 1, j + 1), vid(i, j + 1)
        tris += [(a, b, c), (a, c, d)]
    n = len(uv)
    edges = Counter(tuple(sorted((a, b))) for t in tris for a, b in zip(t, t[1:] + t[:1]))
    directed = {(a, b) for t in tris for a, b in zip(t, t[1:] + t[:1])}
    seams = []
    for (a, b), cnt in edges.items():
        if cnt != 1:
            continue
        mx, my = (uv[a][0] + uv[b][0]) / 2, (uv[a][1] + uv[b][1]) / 2
        hem = mx < X_HEM + 1e-6
        cuff = (my < y_lo + 1e-6) or (my > y_lo + ny * CELL - 1e-6)
        neck = NECK[0] - 1e-6 <= mx and NECK[1] - 1e-6 <= my <= NECK[2] + 1e-6 and not (mx > X_SHO - 1e-6)
        if not (hem or cuff or neck):
            seams.append((a, b))
    z_back = 0.0012
    pts = [(x, y, z_back + LAYER_GAP) for x, y in uv] + [(x, y, z_back) for x, y in uv]  # front layer, then back layer
    faces = list(tris) + [(a + n, c + n, b + n) for a, b, c in tris]
    for a, b in seams:
        if (a, b) not in directed:
            a, b = b, a
        faces += [(b, a, a + n), (b, a + n, b + n)]
    idx = [k for t in faces for k in t]

    stage = Usd.Stage.CreateNew(usd_path)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(stage, "/P_shirt_cloth")
    stage.SetDefaultPrim(root.GetPrim())
    mesh = UsdGeom.Mesh.Define(stage, "/P_shirt_cloth/mesh")
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    mesh.CreateFaceVertexCountsAttr([3] * len(faces))
    mesh.CreateFaceVertexIndicesAttr(idx)
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateDisplayColorAttr([Gf.Vec3f(0.10, 0.18, 0.36)])
    ok = du.set_physics_surface_deformable_body(stage, mesh.GetPath())
    prim = mesh.GetPrim()
    print("[shirt] rest attrs: " + ", ".join(f"{a.GetName()}={a.Get() if a.GetName().endswith('Default') else '...'}" for a in prim.GetAttributes() if "rest" in a.GetName().lower() or "bend" in a.GetName().lower()), flush=True)
    if os.environ.get("SHIRT_RESTBEND", "1") == "1":
        # the sewn seams fold the layers 90 degrees at their edges: tell the solver that fold is the unstressed bend, or its default (flat) inflates the edges
        for name, val in (("omniphysics:restBendAnglesDefault", "restShapeDefault"),):
            a = prim.GetAttribute(name)
            print(f"[shirt] {name}: attribute {'present' if a else 'missing'}; set {a.Set(val) if a else False}", flush=True)
    col = PhysxSchema.PhysxCollisionAPI.Apply(prim)  # the deformable default keeps contacts 2 cm off everything (README, kit section 11)
    col.CreateContactOffsetAttr(float(os.environ.get("SHIRT_CONTACT", "0.002")))
    col.CreateRestOffsetAttr(float(os.environ.get("SHIRT_REST", "0.0006")))
    prim.CreateAttribute("physxDeformableBody:selfCollision", Sdf.ValueTypeNames.Bool).Set(os.environ.get("SHIRT_SELF", "1") == "1")
    if os.environ.get("SHIRT_FILTER", "0.0012"):
        prim.CreateAttribute("physxDeformableBody:selfCollisionFilterDistance", Sdf.ValueTypeNames.Float).Set(float(os.environ.get("SHIRT_FILTER", "0.0012")))
    if os.environ.get("SHIRT_CCD", "0") == "1":  # speculative CCD and frequent collision-pair updates keep thin layers from passing through each other
        prim.CreateAttribute("physxDeformableBody:enableSpeculativeCCD", Sdf.ValueTypeNames.Bool).Set(True)
        prim.CreateAttribute("physxDeformableBody:collisionPairUpdateFrequency", Sdf.ValueTypeNames.Int).Set(int(os.environ.get("SHIRT_PAIRS", "8")))
    iters = os.environ.get("SHIRT_ITERS", "")
    if iters:
        prim.CreateAttribute("physxDeformableBody:solverPositionIterationCount", Sdf.ValueTypeNames.Int).Set(int(iters))
    damp = float(os.environ.get("SHIRT_DAMP", "0.5"))
    if damp > 0:
        prim.CreateAttribute("physxDeformableBody:linearDamping", Sdf.ValueTypeNames.Float).Set(damp)
    mat = UsdShade.Material.Define(stage, "/P_shirt_cloth/fabric")
    ok2 = du.add_surface_deformable_material(stage, mat.GetPath(), density=SHIRT_GSM / SHIRT_THICK, static_friction=0.8, dynamic_friction=0.6, youngs_modulus=SHIRT_YOUNG,
                                             poissons_ratio=0.3, surface_thickness=SHIRT_THICK, surface_bend_stiffness=SHIRT_BEND)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")
    look = UsdShade.Material.Define(stage, "/P_shirt_cloth/look")  # the rendered cloth: navy cotton (the display colour alone renders grey)
    shader = UsdShade.Shader.Define(stage, "/P_shirt_cloth/look/shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.07, 0.13, 0.30))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.95)
    look.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(look)
    stage.GetRootLayer().Save()
    print(f"[shirt] built {len(pts)} nodes, {len(faces)} triangles, {len(seams)} seam edges; body {ok}, material {ok2}", flush=True)


CLOTH = m.Part("shirt_cloth", pos=(0.0, 0.0, TOP), size=(0.1, 0.1, 0.1), role="cloth", build=_build_cloth)
_VIEW: list = [None]


def nodes(env):
    """The shirt's simulation nodes now, in the station frame, through PhysX's tensor view; None if the view cannot be made."""
    if _VIEW[0] is None:
        try:
            import omni.physics.tensors as tensors

            sim_view = tensors.create_simulation_view("torch")
            sim_view.set_subspace_roots("/")
            _VIEW[0] = (sim_view, sim_view.create_surface_deformable_body_view(f"{m.ENV_PATH}/shirt_cloth*/mesh"))
        except Exception as e:  # noqa: BLE001
            print(f"[shirt] no view: {e!r}", flush=True)
            _VIEW[0] = (None, None)
    view = _VIEW[0][1]
    if view is None or view.count == 0:
        return None
    p = view.get_simulation_nodal_positions()
    p = p.cpu().numpy() if hasattr(p, "cpu") else np.asarray(p)
    return p.reshape(-1, 3) - env.unwrapped.scene.env_origins[0].cpu().numpy()


def on_shirt(x: float, y: float) -> bool:
    """Whether a point is over the tee's outline (body and sleeves, the neck notch ignored)."""
    return bool(X_HEM <= x <= X_SHO and ((Y_SIDE <= y <= Y_SIDE + BODY_W) or (x >= SLEEVE_X0 and Y_SIDE - SLEEVE_L <= y <= Y_SIDE + BODY_W + SLEEVE_L)))


def _shirt() -> m.Part:
    """The care label, a small rigid patch hinged along its sewn edge at the side seam (a fixed, visual-only stitch strip carries the hinge: the cloth under it is the FEM shirt)."""
    sz = SHIRT_SIZE[2]
    frame = [("seam", m.box((LABEL_W + 0.006, 0.006, 0.0008), (0.0, 0.0, sz - 0.0004)), "hem")]
    at = (SEAM[0] - SHIRT[0], SEAM[1] - SHIRT[1], sz + 0.001)
    lab = m.box((LABEL_W, LABEL_L, LABEL_T), (at[0], at[1] + LABEL_L / 2, at[2] + LABEL_T / 2))
    curl = m.box((LABEL_W, 0.004, LABEL_T), (at[0], at[1] + LABEL_L - 0.0005, at[2] + 0.002))  # a lip along the free edge; the label itself rests 8 degrees up on its curl spring
    prints = [(f"print{k}", m.box((0.020, 0.0015, 0.0002), (at[0], at[1] + 0.012 + 0.008 * k, at[2] + LABEL_T + 0.0001)), "print") for k in range(5)]
    # once fired: the fastener's T-end through the label, its barb running out past the free edge to the card ticket, which lies on the shirt beyond the label
    tag = [("tag_fastener", m.box((0.0015, 0.0015, 0.006), (at[0], at[1] + PIERCE_U, at[2])), "fastener", True),
           ("tag_barb", m.box((0.0012, 0.040, 0.0012), (at[0], at[1] + PIERCE_U + 0.020, at[2] + LABEL_T + 0.0012)), "fastener", True),
           ("tag_ticket", m.box((0.030, 0.050, 0.0006), (at[0], at[1] + PIERCE_U + 0.065, at[2] + LABEL_T + 0.0015)), "ticket", True),
           ("tag_price", m.box((0.016, 0.010, 0.0003), (at[0], at[1] + PIERCE_U + 0.060, at[2] + LABEL_T + 0.002)), "price", True)]
    label = m.hinge("label", "X", at, 0.0, 120.0, [("label", lab, "label"), ("curl", curl, "label"), *prints, *tag],
                    [m.col_box("label_c", (LABEL_W, LABEL_L, 0.0015), (at[0], at[1] + LABEL_L / 2, at[2] + 0.00075))], mass=0.002, spring=CURL_K, damping=0.003, friction=0.0)
    return m.mechanism("shirt", (SHIRT[0], SHIRT[1], TOP), [label], LOOKS, frame_parts=frame, frame_colliders=[], friction=(0.9, 0.8))


TRIG_TRAVEL, TRIG_FIRE = 0.004, 0.0025  # the trigger's travel back into the grip, and how far it must go to fire (unmeasured: approximations)
TRIG_K, TRIG_PRE = 2000.0, 25.0  # its spring: 25 N preload at rest, 33 N at full travel (unmeasured): a carrying pinch of 20 N does not move it, a 35 N squeeze fires it


def _gun() -> m.Part:
    """The tagging gun in its own frame: origin at the grip's middle, +y forward along the barrel and needle, +x up from the grip to the barrel, z its thickness.
    It lies on its side, so its frame is the table's. The trigger slides back into the grip against its spring; the needle has no collider."""
    t = 0.025
    frame = [("barrel", m.box((0.040, 0.120, t), (UP, 0.030, 0.0)), "gun"),
             ("grip", m.box((0.090, 0.030, t * 0.9), (0.0, -0.012, 0.0)), "grip"),
             ("nose", m.cylinder(0.006, 0.010, (UP, 0.095, 0.0), axis="y"), "steel"),
             ("needle", m.cylinder(0.001, 0.025, (UP, FWD - 0.0125, 0.0), axis="y"), "steel"),
             ("clip", m.box((0.010, 0.040, t * 0.7), (UP + 0.024, 0.0, 0.0)), "fastener"),
             ("ticket", m.box((0.050, 0.0006, 0.030), (UP - 0.022, 0.104, 0.0)), "ticket"),  # the card ticket on the needle
             ("price", m.box((0.016, 0.0003, 0.010), (UP - 0.030, 0.1035, 0.0)), "price")]
    cols = [m.col_box("barrel_c", (0.040, 0.120, t), (UP, 0.030, 0.0)), m.col_box("grip_c", (0.090, 0.030, t * 0.9), (0.0, -0.012, 0.0))]
    trig = m.box((0.080, 0.010, t * 0.9), (0.0, 0.012, 0.0))  # a squeeze lever along the grip's whole front: a finger on the front always bears on it
    trigger = m.slider("trigger", "Y", -TRIG_TRAVEL, 0.0, [("trigger", trig, "trigger")], [m.col_box("trigger_c", (0.080, 0.010, t * 0.9), (0.0, 0.012, 0.0))],
                       mass=0.004, spring=TRIG_K, damping=2.0, friction=0.0, effort=60.0)
    return m.mechanism("gun", (GUN0[0], GUN0[1], TOP + t / 2 + 0.0005), [trigger], LOOKS, frame_parts=frame, frame_colliders=cols, free_mass=0.10,
                       friction=(0.8, 0.7), frame_inertia=(1.6e-4, 0.6e-4, 2.0e-4))


OBJECTS = [CLOTH, _shirt(), _gun()]


def _rotx(a: float) -> np.ndarray:
    c, sn = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -sn], [0.0, sn, c]])


def label_frame(env):
    """The label's hinge point and rotation in the world (the shirt is fixed), and its angle."""
    sh = m.state(env, "shirt")
    a = float(sh.joint("label"))
    hinge = sh.to_world(np.array([[SEAM[0] - SHIRT[0], SEAM[1] - SHIRT[1], SHIRT_SIZE[2] + 0.001]]))[0]
    return hinge, sh.rotation() @ _rotx(a), a


def needle_tip(env):
    g = m.state(env, "gun")
    return g.to_world(np.array([[UP, FWD, 0.0]]))[0], g


def let_go(tips, mid, a) -> bool:
    """The label let down under 30 degrees and let go: every fingertip of both hands 5 cm or more from the label's middle."""
    return bool(np.min(np.linalg.norm(np.asarray(tips) - mid, axis=1)) > 0.05 and a < math.radians(30.0))


def needle_through(tip, base_pt, hinge, Rl, top):
    """Whether the needle is through the label now: its tip and its base on opposite sides of the label's face, crossing it inside its outline,
    where the label stands 2 cm or more over the shirt (`top`, the height of the label's seam). Returns that and the label's clearance there."""
    rt, rb = Rl.T @ (np.asarray(tip) - hinge), Rl.T @ (np.asarray(base_pt) - hinge)
    ht, hb = rt[2] - LABEL_T / 2, rb[2] - LABEL_T / 2
    if ht * hb >= 0.0:
        return False, 0.0
    c = rb + (rt - rb) * (hb / (hb - ht))  # where the needle crosses the label's plane
    clear = float((hinge + Rl @ np.array([0.0, c[1], 0.0]))[2] - top)
    return bool(0.0 < c[1] < LABEL_L and abs(c[0]) < LABEL_W / 2 and clear >= 0.02), clear


def track_entry(entered, prev_rt, tip, base_pt, hinge, Rl):
    """The needle's entry into the label, which needs memory: the needle has no collider, so it could straddle the label's plane beside the label and
    slide in sideways. Entered: the tip has crossed the label's face inside its outline, and the needle has straddled the face, crossing it inside the
    outline, at every step since. Returns (entered, the tip in label coordinates for the next step, torn): torn when an entered needle's crossing
    leaves the outline sideways while it still straddles the face (dragged out through the label's edge)."""
    rt, rb = Rl.T @ (np.asarray(tip) - hinge), Rl.T @ (np.asarray(base_pt) - hinge)
    ht, hb = rt[2] - LABEL_T / 2, rb[2] - LABEL_T / 2
    inside = lambda q: bool(0.0 < q[1] < LABEL_L and abs(q[0]) < LABEL_W / 2)  # noqa: E731
    straddle = ht * hb < 0.0
    crossing_in = straddle and inside(rb + (rt - rb) * (hb / (hb - ht)))
    torn = False
    if prev_rt is not None:
        hp = prev_rt[2] - LABEL_T / 2
        if hp * ht < 0.0 and inside(prev_rt + (rt - prev_rt) * (hp / (hp - ht))) and crossing_in:
            entered = True  # the tip went through the face, inside the outline, this step
    if entered and straddle and not crossing_in:
        entered, torn = False, True
    elif entered and not straddle:
        entered = False  # drawn back out of the label
    return entered, rt, torn


class Tag:
    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.steps, self.pierced, self.torn, self.through, self.fired, self.fastened, self.lifted, self.gun_fell = 0, False, False, False, False, False, 0.0, False
        self.entered, self.prev_rt, self._prev = False, None, None
        self.hold, self.full, self.last, self.shown = 0, False, {}, False
        if env is None:  # at import, before the simulator runs: the parts start hidden by themselves (touching the stage here breaks the app's start)
            return
        for path, vis in [(f"{m.ENV_PATH}/shirt/label/visual/{nm}", False) for nm in ("tag_fastener", "tag_barb", "tag_ticket", "tag_price")] + \
                         [(f"{m.ENV_PATH}/gun/frame/visual/{nm}", True) for nm in ("ticket", "price")]:
            try:  # the ticket on the gun's needle, nothing on the label
                m.set_visible(path, vis)
            except Exception:
                pass

    def step(self, env):
        self.steps += 1
        m.set_target(env, "gun", "trigger", TRIG_PRE / TRIG_K)  # the spring's rest beyond the stop: its preload
        m.set_target(env, "shirt", "label", math.radians(CURL_DEG))  # the label's curl
        hinge, Rl, a = label_frame(env)
        tip, g = needle_tip(env)
        top = TOP + SHIRT_SIZE[2] + 0.001
        # the body: a tip over the shirt's outline and inside it, or within 4 mm above its top face, is a pierce, at any moment
        if on_shirt(tip[0], tip[1]) and tip[2] < TOP + SHIRT_SIZE[2] + 0.004:  # in the shirt, or within 4 mm above its top face
            self.pierced = True
        if a > math.radians(110.0):
            self.torn = True  # pulled past what its sewn edge allows
        self.lifted = max(self.lifted, LABEL_L * math.sin(a))
        # through now: the needle's tip and its base on opposite sides of the label's face, the needle crossing it inside its outline, where the
        # label stands 2 cm or more clear of the shirt
        base_pt = tip - NEEDLE_L * (g.rotation() @ np.array([0.0, 1.0, 0.0]))
        in_label, clear_cm = needle_through(tip, base_pt, hinge, Rl, top)
        self.entered, self.prev_rt, ripped = track_entry(self.entered, self.prev_rt, tip, base_pt, hinge, Rl)
        if ripped:
            self.torn = True  # the needle dragged sideways out through the label's edge
        in_label = bool(in_label and self.entered)  # through only by way of the label's face
        rt = Rl.T @ (tip - hinge)
        u, w, h = rt[1], rt[0], rt[2] - LABEL_T / 2
        self.through = self.through or in_label
        travel = -float(g.joint("trigger"))
        if travel >= TRIG_FIRE and not self.fired:
            self.fired = True
            self.fastened = bool(in_label and not self.pierced)  # only the first squeeze counts: a fastener through the label alone if the needle is through it now, else wasted
        out = not in_label
        gun_down = bool(g.pos[2] - TOP < 0.03 and 0.0 < g.pos[0] < 0.6 and abs(g.pos[1]) < 0.6)
        if g.pos[2] < TOP - 0.05:
            self.gun_fell = True  # off the table
        mid = hinge + Rl @ np.array([0.0, LABEL_L / 2, 0.0])
        out = bool(out and np.linalg.norm(tip - mid) > 0.04)  # the needle drawn back out of the label
        clear = let_go(m.fingertips(env), mid, a)
        if self.shown != self.fastened:
            self.shown = self.fastened
            for nm in ("tag_fastener", "tag_barb", "tag_ticket", "tag_price"):
                try:
                    m.set_visible(f"{m.ENV_PATH}/shirt/label/visual/{nm}", self.fastened)
                except Exception:
                    pass
            for nm in ("ticket", "price"):
                try:
                    m.set_visible(f"{m.ENV_PATH}/gun/frame/visual/{nm}", not self.fastened)
                except Exception:
                    pass
        self.last = {"t": round(self.steps * float(env.unwrapped.step_dt), 2), "label_deg": round(math.degrees(a), 1), "lifted_mm": round(self.lifted * 1000, 1),
                     "tip_label_mm": [round(u * 1000, 1), round(w * 1000, 1), round(h * 1000, 1)], "clear_of_body_mm": round(clear_cm * 1000, 1),
                     "through": self.through, "in_label": bool(in_label), "trigger_mm": round(travel * 1000, 2), "fired": self.fired, "fastened": self.fastened,
                     "pierced": self.pierced, "torn": self.torn, "gun_fell": self.gun_fell, "out": bool(out), "gun_down": gun_down, "clear": clear,
                     "tip_above_body_mm": round((tip[2] - top) * 1000, 1)}
        self.hold = self.hold + 1 if _tier(self.last) == 1.0 else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        self.last["hold"] = self.hold
        if self.steps % 10 == 0 and os.environ.get("SHIRT_LOG", "1") == "1":
            n = nodes(env)
            if n is not None:
                spd = 0.0 if self._prev is None or self._prev.shape != n.shape else float(np.linalg.norm(n - self._prev, axis=1).max() / (10 * float(env.unwrapped.step_dt)))
                self._prev = n.copy()
                z = (n[:, 2] - TOP) * 1000
                print(f"[shirt] t {self.last['t']} nodes {len(n)} nonfinite {int((~np.isfinite(n)).any(axis=1).sum())} z {np.nanmin(z):.1f}..{np.nanmax(z):.1f} mm "
                      f"x {np.nanmin(n[:, 0]):.3f}..{np.nanmax(n[:, 0]):.3f} y {np.nanmin(n[:, 1]):.3f}..{np.nanmax(n[:, 1]):.3f} max node speed {spd:.2f} m/s", flush=True)
        if self.steps % 10 == 0:
            print("[tag] " + json.dumps({"mode": os.environ.get("TASK_DEMO", ""), **self.last}), flush=True)


def _tier(r) -> float:
    if not r or r["pierced"] or r["torn"] or r["gun_fell"]:
        return 0.0
    if r["fastened"] and r["out"] and r["clear"]:
        return 1.0
    if r["through"]:
        return 0.5
    if r["lifted_mm"] >= 20.0:
        return 0.25
    return 0.0


TAG = Tag()
m.on_step(TAG.step)
m.on_reset(TAG.reset)


def score(objects) -> float:
    return 1.0 if TAG.full else min(_tier(TAG.last), 0.5)


def success(objects) -> bool:
    return TAG.full


# -- the scripted demo ----------------------------------------------------------------------------

G_DOWN = 0.0  # the needle level as it goes in
LIFT_DEG = 60.0  # the label swung up to this, pinched: the barrel's lower front edge then stays about 12 mm off it
PUSH_PAST = 0.004  # how far the needle's tip goes beyond the label's face before the squeeze
CLEAR_MIN = 0.024  # pierce where the label stands at least this far above the body (the scorer gates the passage at 2 cm)


def demo_script(ctx):
    env = ctx.emb._env
    L, R = ctx.left, ctx.right
    if ctx.mode == "scene":
        yield from L.wait(float(os.environ.get("SHIRT_SETTLE_S", "4.0")))
        n = nodes(env)
        if n is not None:
            z = (n[:, 2] - TOP) * 1000
            hi = np.flatnonzero(z > 12.0)
            print(f"[shirt] settled: z percentiles 5/50/95/100 {np.percentile(z, [5, 50, 95, 100]).round(1).tolist()} mm; {len(hi)} nodes above 12 mm; "
                  f"their x {n[hi, 0].min() if len(hi) else 0:.3f}..{n[hi, 0].max() if len(hi) else 0:.3f} y {n[hi, 1].min() if len(hi) else 0:.3f}..{n[hi, 1].max() if len(hi) else 0:.3f}", flush=True)
            print(f"[shirt] {int((z > 1.2).sum())} of {len(z)} nodes above 1.2 mm (the front layer, if the two layers keep apart); {int((z > 12).sum())} above 12 mm", flush=True)
            for xs in (0.22, 0.30, 0.34, 0.42, 0.46):  # the section across the body at some x
                sel = np.abs(n[:, 0] - xs) < 0.006
                print(f"[shirt] x {xs}: {sel.sum()} nodes, z {np.sort(z[sel])[[0, len(z[sel]) // 2, -1]].round(1).tolist() if sel.any() else None}", flush=True)
        print("[tag] END " + json.dumps({"mode": ctx.mode, **TAG.last}), flush=True)
        return
    nolift = ctx.mode == "nolift"  # failure check: the label left lying on the shirt, the needle pushed at it as in the demo
    hinge = label_frame(env)[0]

    def edge(deg, u=LABEL_L):
        return hinge + u * np.array([0.0, math.cos(math.radians(deg)), math.sin(math.radians(deg))])

    # the left hand: pads round the curled free edge, 12 mm in, the fingers pointing along the label toward its seam (tilted 60 degrees from straight down, rolled
    # to close across the label: the wrist leans well away from the right hand's); a light pinch, then the label swung up about its seam
    gl0 = d.tilted(60.0, -90.0)
    gl = d._as_grip(gl0.rot @ d.axis_angle_matrix(np.array([0.0, 0.0, 1.0]), -math.pi / 2))
    a_rest = label_frame(env)[2]
    back = -L.approach_axis(gl)
    P = edge(math.degrees(a_rest), HOLD_U) + np.array([0.0, 0.0, 0.001])
    if not nolift:
        yield from L.move(P + 0.06 * back + np.array([0.0, 0.0, 0.02]), grip=gl, finger=d.FINGER_OPEN, t=2.0, label="behind the label's curled free edge")
        yield from L.move(P + 0.03 * back, t=0.8, label="jaws either side of the free edge")
        yield from L.move(P, t=0.8, label="pads 12 mm in from the edge")
        yield from L.close(label="pinch the label")
        yield from L.arc(hinge, (1.0, 0.0, 0.0), LIFT_DEG - math.degrees(a_rest), t=2.0, label=f"swing the label up about its seam to {LIFT_DEG:.0f} degrees")
    print("[tag] lifted: " + json.dumps(TAG.last), flush=True)
    # the right hand: the gun from the table, the fingers across the grip's back and the trigger, a light pinch under the trigger's preload
    g0 = d.down(0.0)
    grip = m.state(env, "gun").pos
    yield from R.move(grip + np.array([0.0, 0.0, 0.07]), grip=g0, finger=d.FINGER_OPEN, t=2.0, label="over the gun's grip")
    yield from R.move(grip, t=1.0, label="fingers either side of the grip and trigger")
    yield from R.close(0.044, force=20.0, label="hold the gun at 20 N, under the trigger's 25 N preload")
    yield from R.move(R.at() + np.array([0.0, 0.0, 0.08]), t=1.0, label="lift the gun")
    tip, _ = needle_tip(env)
    in_hand = R.grip.rot.T @ (tip - R.at())  # the needle's tip in the hand's frame
    n = np.array([0.0, math.cos(math.radians(G_DOWN)), -math.sin(math.radians(G_DOWN))])
    aim = d._as_grip(_rotx(-math.radians(G_DOWN)) @ g0.rot)
    tip_off = aim.rot @ in_hand
    h0, R0, a0 = label_frame(env)  # the label as the left hand holds it now
    u = float(np.clip(CLEAR_MIN / max(math.sin(a0), 0.2), PIERCE_U, 0.040))  # far enough out to stand clear, short of the pads
    pierce = h0 + R0 @ np.array([0.0, u, 0.0])
    print("[tag] aim: " + json.dumps({"label_deg": round(math.degrees(a0), 1), "pierce_u_mm": round(u * 1000, 1), "left_tcp_rel_hinge_mm": [round(float(v) * 1000, 1) for v in L.at() - h0]}), flush=True)
    yield from R.move(pierce - 0.05 * n - tip_off, grip=aim, t=2.0, label="aim the needle at the label")
    yield from R.servo(pierce - 0.05 * n, tol=0.0015, timeout=1.5, read=lambda: needle_tip(env)[0], label="the needle's tip 5 cm short of the label")
    yield from R.servo(lambda s: pierce + (-0.05 + min(s / 1.5, 1.0) * (0.05 + PUSH_PAST)) * n, tol=0.0015, timeout=2.5, read=lambda: needle_tip(env)[0],
                       label="push the needle through the label")
    print("[tag] through: " + json.dumps(TAG.last), flush=True)
    # squeeze until the fastener fires, then ease off at once: held on its stop, the trigger would let the gun slip in the hand, and the jolt would
    # shake the label out of the left hand's pinch
    for _ in R.close(0.030, force=35.0, t=1.0, label="squeeze at up to 35 N: the trigger fires a fastener"):
        yield
        if TAG.fired:
            break
    yield from R.close(0.044, force=20.0, t=0.3, label="ease off the trigger as it fires")
    print("[tag] fired: " + json.dumps(TAG.last), flush=True)
    yield from R.move(R.at() - 0.06 * n, t=1.2, label="draw the needle back")
    yield from R.move(R.at() + np.array([-0.06, -0.08, 0.04]), t=1.5, label="the gun away from the shirt, held")
    # the left lets the label down with the ticket on it
    # let it down to 25 degrees, open the jaws 12 mm (fully open, the lower jaw would go into the shirt), and draw them out level so the
    # label slips off the lower jaw's end onto the shirt
    if not nolift:
        yield from L.arc(hinge, (1.0, 0.0, 0.0), 25.0 - LIFT_DEG, t=1.5, label="let the label down to 25 degrees")
        yield from L.move(L.at(), finger=-0.006, t=0.5, label="let go of the label: jaws 12 mm apart")
        yield from L.move(L.at() + np.array([0.0, 0.05, 0.0]), t=1.2, label="draw the jaws out level, off the label's edge")
        yield from L.move(L.at() + np.array([0.0, 0.02, 0.06]), t=1.0, label="left clear")
    yield from L.wait(1.5)
    print("[tag] END " + json.dumps({"mode": ctx.mode, **TAG.last}), flush=True)
