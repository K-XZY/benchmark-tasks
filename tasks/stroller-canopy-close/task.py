"""Draw an infant carrier's cloth canopy forward over a doll's face zone until it clicks into its open stop, steadying the rocking carrier by its handle.

A Graco SnugRide 35 Lite LX infant carrier at its listed size (69.6 x 39.4 cm shell, canopy top 45.9 cm) stands on the bench on its curved rocker base, its open
front facing the arms, the carry handle turned back behind the head end, a 1 kg doll (about 45 cm long) lying in it. Three canopy bows hinge at the shell's sides; a
sewn surface-FEM cloth canopy (native Isaac surface deformable, 0.6 mm, 160 g/m2, friction, self-contact) is sewn to the bows, pleated between them. The fabric
pulls the front bow back toward closed (a spring of 1.5 N m/rad) until it clicks into its open stop 45 degrees forward of upright. The left hand steadies the
handle by its side arm and keeps the carrier level; the right hand takes the canopy's front bow and draws it forward through its arc to the stop.

Sim (stated): the carrier is a rigid body on a hinge at the rockers' contact with the bench, with a spring giving a rocker's restoring torque (m g (R - h), R = 0.42 m,
h = 0.17 m), damping and friction; it does not roll along as it rocks. Bows are hinged links with joint friction and end stops. The cloth is attached to the bows only.
The right hand holds the front bow by friction alone (HOLD_RULE = False); setting HOLD_RULE = True instead adds a diagnostic rule, a spring of 800 N/m (at most 30 N) between the jaws and the bow.
An unsteadied carrier (rock 3 degrees or more at any time, or 1 degree or more at the end) scores at most 0.25.

    python demokit.py task.py OUT --mode full                    # the filmed demo
    python demokit.py task.py OUT --no-record --mode noleft      # the lazy routes, one per session: noleft, none
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
INSTRUCTION = ("Shade the baby: with your left hand hold the carrier steady by its carry handle, and with your right hand take the canopy's front bow and draw it forward until it stops, its front edge over the doll's face, without the fabric touching the face.")
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0

CX, CY = float(os.environ.get("CAN_CX", "0.16")), float(os.environ.get("CAN_CY", "0.0"))  # the carrier's origin on the bench: its middle
YAW = float(os.environ.get("CAN_YAW", "-90"))  # the carrier's turn about z: -90 puts its head end (local +y) away from the robot, its open front facing the arms
W2, L2 = 0.197, 0.348  # shell half width (x) and half length (y), Graco's listed 39.4 x 69.6 cm; +y is the head end
R_ROCK = float(os.environ.get("CAN_R", "0.42"))  # the rockers' radius of curvature
ROCK_Z = 0.0  # the carrier rocks about the rockers' contact with the bench
FLOOR_Z = 0.11  # underside of the shell floor over the bench
RIM = float(os.environ.get("CAN_RIM", "0.28"))  # shell rim height, and the bows' pivot height
YH = 0.22  # the bows' pivot, along the carrier (toward the head end)
H_BOW = float(os.environ.get("CAN_H", "0.179"))  # bow arch height over its pivot axis: the canopy's top at the listed 45.9 cm
MASS = 4.6  # carrier 3.6 kg (listing) + doll 1 kg
K_ROCK = MASS * 9.81 * (R_ROCK - 0.17)  # N m/rad, the rocker's restoring torque for a centre of mass 0.17 m up
Q0 = {"bow_r": -60.0, "bow_m": -30.0, "bow_f": 0.0}  # the bows' start angles (deg, positive = forward = toward -y)
STOP = float(os.environ.get("CAN_STOP", "45"))  # the canopy's open stop: the front bow 45 degrees forward of upright
CANOPY_K = float(os.environ.get("CAN_K", "1.5"))  # N m/rad: the canopy's pull back toward closed, about 1.2 N m at the 45 degree stop
LATCH = os.environ.get("CAN_LATCH", "1") == "1"  # the front bow clicks into its open stop and stays there
HUB_FRICTION = float(os.environ.get("CAN_HUB", "0.15"))  # N m: the canopy hub's friction on the front bow (a 0.3 kg canopy at 15 cm needs about 0.45 N m to stay where it is set)
LIM = {"bow_r": (-80.0, -5.0), "bow_m": (-60.0, 35.0), "bow_f": (-25.0, STOP)}  # each bow's travel (deg): the front bow ends at the stop, the others fan out behind it
GA = float(os.environ.get("CAN_GA", "25"))  # where the hand takes the front bow: degrees round the arch from its top (positive: the right hand's side)
GBETA = float(os.environ.get("CAN_GBETA", "270"))  # the hand's approach round the bow tube (see bow_grip)
GFLIP = os.environ.get("CAN_GFLIP", "1") == "1"
LINE_Y = round(YH - H_BOW * math.sin(math.radians(STOP - 5.0)), 3)  # the visor line: the front edge must pass it (y at or below), in the carrier's frame; 5 degrees short of the stop
FACE_C, FACE_HALF = np.array([0.0, 0.14, 0.27]), np.array([0.06, 0.07, 0.012])  # the face zone's keep-out box (carrier frame at rest)
ROCK_LIMIT = 3.0  # deg

LOOKS = {"shell": m.Look(color=m.rgb(0.62, 0.64, 0.68), roughness=0.5), "pad": m.Look(color=m.rgb(0.80, 0.82, 0.86), roughness=0.9),
         "handle": m.Look(color=m.rgb(0.10, 0.10, 0.11), roughness=0.5), "bow": m.Look(color=m.rgb(0.55, 0.57, 0.60), roughness=0.4),
         "tab": m.Look(color=m.rgb(0.85, 0.30, 0.10), roughness=0.5), "doll": m.Look(color=m.rgb(0.93, 0.78, 0.66), roughness=0.8),
         "face": m.Look(color=m.rgb(0.95, 0.15, 0.55), roughness=0.6), "blanket": m.Look(color=m.rgb(0.30, 0.55, 0.85), roughness=0.95),
         "rocker": m.Look(color=m.rgb(0.15, 0.15, 0.17), roughness=0.6)}

# -- the cloth -----------------------------------------------------------------------------

GSM, THICK = 0.16, 0.0006  # 160 g/m2 polyester canopy fabric, 0.6 mm
YOUNG = float(os.environ.get("CAN_YOUNG", "2e6"))
BEND = float(os.environ.get("CAN_BEND", "1.8e5"))
NU, NV = 24, 24  # columns across the arch, rows from the rear bow to the front bow
PLEAT = float(os.environ.get("CAN_PLEAT", "0.25"))  # fabric between bows is pleated: radial +-25 % in one full wave, the slack that lets the bows spread
ALPHA = float(os.environ.get("CAN_ALPHA", "72"))  # the cloth runs round the arch to this many degrees either side of its top


_CY, _SY = math.cos(math.radians(YAW)), math.sin(math.radians(YAW))
RZ = np.array([[_CY, -_SY, 0.0], [_SY, _CY, 0.0], [0.0, 0.0, 1.0]])  # the carrier's turn


def W(p) -> np.ndarray:
    """A point in the carrier's frame to the station frame (the carrier at rest, turned about z by YAW)."""
    return np.array([CX, CY, TOP]) + RZ @ np.asarray(p, float)


def local(pts, rock_deg: float) -> np.ndarray:
    """Station-frame points to the carrier's frame at rest, undoing its turn and its rock."""
    loc = (np.asarray(pts, float).reshape(-1, 3) - np.array([CX, CY, TOP])) @ RZ  # RZ^T applied to each row
    th = math.radians(rock_deg)
    yy, zz = loc[:, 1], loc[:, 2] - ROCK_Z
    return np.stack([loc[:, 0], yy * math.cos(th) + zz * math.sin(th), -yy * math.sin(th) + zz * math.cos(th) + ROCK_Z], axis=1)


def bow_grip(q_deg: float, rock_deg: float = 0.0):
    """Where the hand takes the front bow, turned q forward, in the station frame: the TCP and its rotation. The fingers straddle the bow tube and close across it;
    GBETA turns the approach round the tube (0: from outside the arch inward, 90: along the bow's swing)."""
    a, q = math.radians(GA), math.radians(q_deg)
    P = np.array([W2 * math.sin(a), YH - H_BOW * math.cos(a) * math.sin(q), RIM + H_BOW * math.cos(a) * math.cos(q)])
    T = np.array([W2 * math.cos(a), H_BOW * math.sin(a) * math.sin(q), -H_BOW * math.sin(a) * math.cos(q)])
    T /= np.linalg.norm(T)
    out = P - np.array([P[0], YH, RIM])
    out /= np.linalg.norm(out)
    n = np.cross(T, out)
    b = math.radians(GBETA)
    A = -math.cos(b) * out + math.sin(b) * n
    th = math.radians(rock_deg)  # the rock turns the carrier about x through (0, 0, ROCK_Z)
    Rr = np.array([[1.0, 0.0, 0.0], [0.0, math.cos(th), -math.sin(th)], [0.0, math.sin(th), math.cos(th)]])
    P = Rr @ (P - np.array([0.0, 0.0, ROCK_Z])) + np.array([0.0, 0.0, ROCK_Z])
    T, A = RZ @ Rr @ T, RZ @ Rr @ A
    C = np.cross(A, T)
    C /= np.linalg.norm(C)
    rot = np.zeros((3, 3))
    rot[:, 0], rot[:, 2] = C, A
    rot[:, 1] = np.cross(A, C)
    if np.linalg.det(rot) < 0:
        rot[:, 1] *= -1
    if GFLIP:
        rot = rot @ np.diag([-1.0, -1.0, 1.0])
    return W(P), rot


def arch(alpha_deg: float, q_deg: float, eps: float = 0.0) -> np.ndarray:
    """A point of the arch at angle alpha from its top, with the bow turned q forward about its pivot axis, in the carrier frame at rest."""
    a, q = math.radians(alpha_deg), math.radians(q_deg)
    x, h = W2 * math.sin(a), H_BOW * math.cos(a) * (1.0 + eps)
    return np.array([x, YH - h * math.sin(q), RIM + h * math.cos(q)])


def _build_cloth(usd_path: str) -> None:
    from omni.physx.scripts import deformableUtils as du
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdShade

    pts, faces = [], []
    qs = [Q0["bow_r"] + (Q0["bow_f"] - Q0["bow_r"]) * j / NV for j in range(NV + 1)]
    for j in range(NV + 1):
        for i in range(NU + 1):
            seg = j / (NV / 2)
            eps = PLEAT * math.sin(2 * math.pi * (seg - math.floor(seg)))
            p = arch(-ALPHA + 2 * ALPHA * i / NU, qs[j], eps)
            w = W(p + np.array([0.0, 0.0, 0.002]))  # 2 mm above the bow centreline plane's outside
            pts.append((w[0], w[1], w[2] - TOP))
    for j in range(NV):
        for i in range(NU):
            a, b, c, e = j * (NU + 1) + i, j * (NU + 1) + i + 1, (j + 1) * (NU + 1) + i + 1, (j + 1) * (NU + 1) + i
            faces += [(a, b, c), (a, c, e)]
    idx = [k for t in faces for k in t]
    stage = Usd.Stage.CreateNew(usd_path)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(stage, "/P_canopy_cloth")
    stage.SetDefaultPrim(root.GetPrim())
    mesh = UsdGeom.Mesh.Define(stage, "/P_canopy_cloth/mesh")
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    mesh.CreateFaceVertexCountsAttr([3] * len(faces))
    mesh.CreateFaceVertexIndicesAttr(idx)
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateDisplayColorAttr([Gf.Vec3f(0.55, 0.20, 0.25)])
    ok = du.set_physics_surface_deformable_body(stage, mesh.GetPath())
    prim = mesh.GetPrim()
    col = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    col.CreateContactOffsetAttr(float(os.environ.get("CAN_CONTACT", "0.003")))
    col.CreateRestOffsetAttr(float(os.environ.get("CAN_REST", "0.001")))
    prim.CreateAttribute("physxDeformableBody:selfCollision", Sdf.ValueTypeNames.Bool).Set(True)
    prim.CreateAttribute("physxDeformableBody:selfCollisionFilterDistance", Sdf.ValueTypeNames.Float).Set(0.002)
    prim.CreateAttribute("physxDeformableBody:linearDamping", Sdf.ValueTypeNames.Float).Set(float(os.environ.get("CAN_DAMP", "0.5")))
    mat = UsdShade.Material.Define(stage, "/P_canopy_cloth/fabric")
    ok2 = du.add_surface_deformable_material(stage, mat.GetPath(), density=GSM / THICK, static_friction=0.8, dynamic_friction=0.6, youngs_modulus=YOUNG,
                                             poissons_ratio=0.3, surface_thickness=THICK, surface_bend_stiffness=BEND)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")
    look = UsdShade.Material.Define(stage, "/P_canopy_cloth/look")
    shader = UsdShade.Shader.Define(stage, "/P_canopy_cloth/look/shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.55, 0.14, 0.20))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
    look.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(look)
    stage.GetRootLayer().Save()
    print(f"[canopy] cloth built {len(pts)} nodes, {len(faces)} triangles; body {ok}, material {ok2}", flush=True)


CLOTH = m.Part("canopy_cloth", pos=(0.0, 0.0, TOP), size=(0.1, 0.1, 0.1), role="cloth", build=_build_cloth)
_VIEW: list = [None]
_ATTACHED: list = [False]
APEX = NV * (NU + 1) + NU // 2  # the front row's middle node


def nodes(env):
    if _VIEW[0] is None:
        try:
            import omni.physics.tensors as tensors

            sim_view = tensors.create_simulation_view("torch")
            sim_view.set_subspace_roots("/")
            _VIEW[0] = (sim_view, sim_view.create_surface_deformable_body_view(f"{m.ENV_PATH}/canopy_cloth*/mesh"))
        except Exception as e:  # noqa: BLE001
            print(f"[canopy] no view: {e!r}", flush=True)
            _VIEW[0] = (None, None)
    view = _VIEW[0][1]
    if view is None or view.count == 0:
        return None
    p = view.get_simulation_nodal_positions()
    p = p.cpu().numpy() if hasattr(p, "cpu") else np.asarray(p)
    return p.reshape(-1, 3) - env.unwrapped.scene.env_origins[0].cpu().numpy()


def attach_bows(env) -> None:
    """Sew the cloth to the three bows on the live stage, once (Isaac's attachment helper needs the running stage)."""
    if _ATTACHED[0]:
        return
    import omni.usd
    from omni.physx.scripts import deformableUtils as du
    from pxr import Sdf

    stage = omni.usd.get_context().get_stage()
    base = f"{m.ENV_PATH}/canopy_cloth"
    for name in ("bow_r", "bow_m", "bow_f"):
        path = f"{m.ENV_PATH}/canopy/{name}"
        ok = du.create_auto_deformable_attachment(stage, Sdf.Path(f"{base}/attach_{name}"), Sdf.Path(f"{base}/mesh"), Sdf.Path(path))
        print(f"[canopy] {name} attached on the live stage: {ok} ({path})", flush=True)
    _ATTACHED[0] = True


m.on_reset(attach_bows)

# -- the carrier ---------------------------------------------------------------------------


def _bow(name: str) -> m.Link:
    pts = [arch(a, Q0[name]) for a in np.linspace(-ALPHA - 8, ALPHA + 8, 21)]  # authored at its start pose: joint zero is the start
    tube = m.tube(pts, 0.008, 8)
    cols = [m.col_hull(f"c{k}", m.tube([pts[k], pts[k + 1]], 0.008, 6)) for k in range(len(pts) - 1)]
    parts = [("tube", tube, "bow")]
    lo, hi = LIM[name]
    fr = HUB_FRICTION if name == "bow_f" else 0.15
    k = CANOPY_K if name == "bow_f" else 0.0  # the stretched fabric pulls the front bow back toward closed until it clicks into its open stop
    return m.hinge(name, "X", (0.0, YH, RIM), lo - Q0[name], hi - Q0[name], parts, cols, parent="rock", mass=0.05, damping=0.1, friction=fr, spring=k, effort=20.0,
                   inertia=(0.003, 0.003, 0.003))


HY = float(os.environ.get("CAN_HY", "0.44"))  # the carry handle turned back behind the head end: its grip bar this far from the middle, 0.20 m up
HX = W2 + 0.05  # its side arms run 5 cm outside the shell's walls, from the hubs


def _carrier() -> m.Part:
    wz = (FLOOR_Z + RIM) / 2
    vis = [("floor", m.box((2 * W2, 2 * L2, 0.015), (0.0, 0.0, FLOOR_Z + 0.0075)), "shell"),
           ("wall_l", m.box((0.02, 2 * L2, RIM - FLOOR_Z), (W2 - 0.01, 0.0, wz)), "shell"), ("wall_r", m.box((0.02, 2 * L2, RIM - FLOOR_Z), (-W2 + 0.01, 0.0, wz)), "shell"),
           ("wall_h", m.box((2 * W2 - 0.04, 0.02, RIM - FLOOR_Z), (0.0, L2 - 0.01, wz)), "shell"), ("wall_f", m.box((2 * W2 - 0.04, 0.02, RIM - FLOOR_Z), (0.0, -L2 + 0.01, wz)), "shell"),
           ("pad", m.box((2 * W2 - 0.06, 2 * L2 - 0.06, 0.02), (0.0, 0.0, FLOOR_Z + 0.025)), "pad"),
           ("torso", m.box((0.16, 0.30, 0.07), (0.0, -0.08, FLOOR_Z + 0.07)), "blanket"), ("head", m.sphere(0.07, (0.0, 0.14, FLOOR_Z + 0.115)), "doll"),
           ("face", m.cylinder(0.05, 0.003, (0.0, 0.14, FLOOR_Z + 0.1845)), "face"),
           ("cross", m.box((2 * HX + 0.02, 0.02, 0.02), (0.0, HY, 0.20)), "handle"), ("arm_l", m.box((0.02, 0.26, 0.02), (HX, HY - 0.13, 0.20)), "handle"),
           ("arm_r", m.box((0.02, 0.26, 0.02), (-HX, HY - 0.13, 0.20)), "handle")]
    for sgn, nm in ((1, "rk_l"), (-1, "rk_r")):  # the rocker arcs under the floor (visual: a circle turns into itself)
        ys = np.linspace(-min(0.9 * L2, 0.95 * R_ROCK), min(0.9 * L2, 0.95 * R_ROCK), 13)
        pts = [(sgn * 0.17, y, R_ROCK - math.sqrt(R_ROCK**2 - y**2) - 0.0) for y in ys]
        vis.append((nm, m.tube(pts, 0.012, 8), "rocker"))
    cols = [m.col_box("floor_c", (2 * W2, 2 * L2, 0.015), (0.0, 0.0, FLOOR_Z + 0.0075)),
            m.col_box("wl_c", (0.02, 2 * L2, RIM - FLOOR_Z), (W2 - 0.01, 0.0, wz)), m.col_box("wr_c", (0.02, 2 * L2, RIM - FLOOR_Z), (-W2 + 0.01, 0.0, wz)),
            m.col_box("wh_c", (2 * W2 - 0.04, 0.02, RIM - FLOOR_Z), (0.0, L2 - 0.01, wz)), m.col_box("wf_c", (2 * W2 - 0.04, 0.02, RIM - FLOOR_Z), (0.0, -L2 + 0.01, wz)),
            m.col_box("torso_c", (0.16, 0.30, 0.07), (0.0, -0.08, FLOOR_Z + 0.07)), m.col_hull("head_c", m.sphere(0.07, (0.0, 0.14, FLOOR_Z + 0.115))),
            m.col_box("cross_c", (2 * HX + 0.02, 0.02, 0.02), (0.0, HY, 0.20)), m.col_box("al_c", (0.02, 0.26, 0.02), (HX, HY - 0.13, 0.20)),
            m.col_box("ar_c", (0.02, 0.26, 0.02), (-HX, HY - 0.13, 0.20))]
    rock = m.hinge("rock", "X", (0.0, 0.0, ROCK_Z), -25.0, 25.0, vis, cols, mass=MASS, spring=K_ROCK, damping=0.8, friction=0.05, effort=300.0, inertia=(0.35, 0.35, 0.35))
    return m.mechanism("canopy", (CX, CY, TOP), [rock, _bow("bow_r"), _bow("bow_m"), _bow("bow_f")], LOOKS, yaw_deg=YAW, frame_parts=[], frame_colliders=[],
                       frame_inertia=(0.1, 0.1, 0.1))


OBJECTS = [CLOTH, _carrier()]

# -- log and score -------------------------------------------------------------------------

HEAD_C, HEAD_R = np.array([0.0, 0.14, FLOOR_Z + 0.115]), 0.07  # the doll's head, carrier frame
TORSO_C, TORSO_HALF = np.array([0.0, -0.08, FLOOR_Z + 0.07]), np.array([0.08, 0.15, 0.035])
DOLL_CLEAR = 0.01  # no fingertip within 1 cm of the doll
AT_STOP = 2.0  # deg: the front bow counts as at its stop within this


def box_gap(pts, c, half) -> np.ndarray:
    return np.linalg.norm(np.maximum(np.abs(pts - c) - half, 0.0), axis=1)


def doll_gap(tips_local) -> float:
    """The nearest fingertip of either hand to the doll (head and body), metres."""
    head = np.linalg.norm(tips_local - HEAD_C, axis=1) - HEAD_R
    return float(min(head.min(), box_gap(tips_local, TORSO_C, TORSO_HALF).min()))


def tier(passed: bool, at_stop: bool, travel: float, touched: bool, doll: bool, peak: float, end: float, held: bool) -> float:
    """The score from what happened: plain values, so hand-built states can be checked.
    An unsteadied carrier (rock 3 degrees or more at any time, or 1 degree or more at the end) earns no canopy credit: at most 0.25."""
    if touched or doll or peak > 6.0:
        return 0.0
    drawn = travel >= 0.05
    if peak >= ROCK_LIMIT or end >= 1.0:
        return 0.25 if drawn else 0.0
    if passed and at_stop and held:
        return 1.0
    if passed and at_stop:
        return 0.5
    return 0.25 if drawn else 0.0


HOLD_RULE = False  # False: the hand holds the bow by friction alone (no rule); True adds an 800 N/m hold, for diagnosis
HOLD = os.environ.get("CAN_HOLD", "1" if HOLD_RULE else "0") == "1"  # the hand's hold on the front bow as a rule, engaged only while both fingers close on the bow
HOLD_K, HOLD_MAX = 800.0, 30.0  # N/m and N: the hold's spring and its most force
PAD_INSET = 0.0045  # mechkit's fingertip points sit this far outside each pad


def jaws_on_bow(tips_right, point) -> tuple[bool, np.ndarray]:
    """Whether the right hand's two fingers close on the bow tube at `point` (station frame): the pads within 3 cm of each other, the tube between them,
    and the pads' midpoint within 2.5 cm of it. Returns that and the midpoint."""
    a, b = tips_right[:6], tips_right[6:]
    gap = float(np.mean(np.linalg.norm(a - b, axis=1))) - 2 * PAD_INSET
    ca, cb = a.mean(axis=0), b.mean(axis=0)
    mid = (ca + cb) / 2
    u = cb - ca
    u = u / max(np.linalg.norm(u), 1e-9)
    between = float(np.dot(point - ca, u)) > -0.004 and float(np.dot(point - cb, u)) < 0.004
    return gap < 0.03 and between and float(np.linalg.norm(mid - point)) < 0.025, mid


class Canopy:
    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.steps, self.prev, self.apex0 = 0, None, None
        self.max_rock, self.max_speed, self.max_travel, self.min_gap, self.bad, self.min_doll = 0.0, 0.0, 0.0, 9.0, 0, 9.0
        self.passed, self.hold, self.full, self.last, self.contact_steps, self.touched, self.doll = False, 0, False, {}, 0, False, False
        self.rock, self.at_stop = 0.0, False
        self.hold_off, self.pushing, self.hold_steps = None, False, 0
        self.latched = False

    def step(self, env):
        self.steps += 1
        c = m.state(env, "canopy")
        rock = math.degrees(c.joint("rock"))
        q = {k: round(math.degrees(c.joint(k)) + Q0[k], 1) for k in Q0}
        n = nodes(env)
        r = {"t": round(self.steps * float(env.unwrapped.step_dt), 2), "rock_deg": round(rock, 2), "bows_deg": q}
        if HOLD:
            self._hold(env, c, rock)
        try:
            tl = local(m.fingertips(env), rock)
            dg = doll_gap(tl)
            self.min_doll = min(self.min_doll, dg)
            self.doll = self.doll or dg < DOLL_CLEAR
            r["doll_gap_mm"] = round(dg * 1000, 1)
        except Exception as e:  # noqa: BLE001
            r["tips"] = repr(e)[:60]
        if n is not None:
            nf = int((~np.isfinite(n)).any(axis=1).sum())
            self.bad = max(self.bad, nf)
            spd = 0.0 if self.prev is None or self.prev.shape != n.shape else float(np.nanmax(np.linalg.norm(n - self.prev, axis=1)) * float(env.unwrapped.step_dt) ** -1)
            self.prev = n.copy()
            lo = local(n, rock)
            lx, ly, lz = lo[:, 0], lo[:, 1], lo[:, 2]
            inside = (np.abs(lx - FACE_C[0]) < FACE_HALF[0] + 0.01) & (np.abs(ly - FACE_C[1]) < FACE_HALF[1] + 0.01) & (np.abs(lz - FACE_C[2]) < FACE_HALF[2] + 0.01)
            gap = float(np.nanmin(box_gap(lo, FACE_C, FACE_HALF)))
            touching = int(inside.sum())
            ay = float(ly[APEX])
            if self.apex0 is None:
                self.apex0 = ay
            travel = self.apex0 - ay
            self.max_speed, self.min_gap, self.max_travel = max(self.max_speed, spd), min(self.min_gap, gap), max(self.max_travel, travel)
            self.contact_steps += int(touching > 0)
            r.update({"nodes": len(n), "nonfinite": nf, "node_speed": round(spd, 2), "apex_y": round(ay, 3), "travel_cm": round(travel * 100, 1), "face_gap_mm": round(gap * 1000, 1),
                      "face_touch_nodes": touching})
            if ay <= LINE_Y:
                self.passed = True
            self.touched = self.touched or touching > 0
        self.max_rock = max(self.max_rock, abs(rock))
        self.rock = abs(rock)
        self.at_stop = q["bow_f"] >= STOP - AT_STOP
        if LATCH and not self.latched and q["bow_f"] >= STOP - 0.5:  # the open stop's click
            m.lock(env, "canopy", "bow_f", stiffness=200.0, effort=20.0)
            self.latched = True
            print(f"[canopy] latched at {q['bow_f']} deg", flush=True)
        line_now = r.get("apex_y", 9.0) <= LINE_Y
        ok = line_now and self.at_stop and not self.touched and not self.doll and self.max_rock < ROCK_LIMIT and self.rock < 1.0
        self.hold = self.hold + 1 if ok else 0
        self.full = self.full or self.hold >= 20  # held 1 s at 20 Hz
        r.update({"max_rock": round(self.max_rock, 2), "passed": self.passed, "at_stop": self.at_stop, "hold": self.hold, "held_bow": self.hold_off is not None})
        self.last = r
        if self.steps % 10 == 0:
            print("[canopy] " + json.dumps(r), flush=True)

    def _hold(self, env, c, rock: float) -> None:
        tips = m.fingertips(env)
        p, _ = bow_grip(math.degrees(c.joint("bow_f")) + Q0["bow_f"], rock)
        on, mid = jaws_on_bow(tips[12:], p)
        if on and self.hold_off is None:
            self.hold_off = mid - p  # where the bow sits in the jaws when they close on it
        if not on:
            self.hold_off = None
        if self.hold_off is not None:
            f = HOLD_K * (mid - self.hold_off - p)
            n = float(np.linalg.norm(f))
            if n > HOLD_MAX:
                f *= HOLD_MAX / n
            m.push(env, "canopy", tuple(f), at=p, body="bow_f")
            self.pushing, self.hold_steps = True, self.hold_steps + 1
        elif self.pushing:
            m.push(env, "canopy", (0.0, 0.0, 0.0), body="bow_f")
            self.pushing = False

    def score(self) -> float:
        return tier(self.passed, self.at_stop, self.max_travel, self.touched, self.doll, self.max_rock, self.rock, self.full)


CAN = Canopy()
m.on_step(CAN.step)
m.on_reset(CAN.reset)


def score(objects) -> float:
    return CAN.score()


def success(objects) -> bool:
    return CAN.score() >= 1.0


# -- the scripted demo ---------------------------------------------------------------------

HANDLE_GRIP = (-HX, HY - 0.06, 0.20)  # where the left hand takes the handle: its left side arm, near the grip bar (carrier frame)


def demo_script(ctx):
    L, R = ctx.left, ctx.right
    mode = ctx.mode
    yield from L.wait(float(os.environ.get("CAN_SETTLE_S", "3.0")))
    print("[canopy] settled: " + json.dumps(CAN.last), flush=True)
    if mode in ("scene", "none"):  # none: the lazy route of doing nothing
        print("[canopy] END " + json.dumps({"mode": mode, "max_rock": CAN.max_rock, "max_speed": CAN.max_speed, "min_gap_mm": CAN.min_gap * 1000, "bad": CAN.bad}), flush=True)
        return
    if mode == "drive":  # diagnostic only, no hands: drive the front bow's joint to the stop, to see whether the carrier and cloth let it get there
        env = ctx.emb._env
        asset, j = m._joint(env, "canopy", "bow_f")
        asset.write_joint_effort_limit_to_sim(m._tensor(asset, 20.0), joint_ids=[j])
        asset.write_joint_stiffness_to_sim(m._tensor(asset, 40.0), joint_ids=[j])
        for k in range(160):
            asset.set_joint_position_target(m._tensor(asset, math.radians((STOP + 2.0) * min(1.0, k / 120))), joint_ids=[j])
            yield
        print("[canopy] END " + json.dumps({"mode": mode, "bows": CAN.last.get("bows_deg"), "min_gap_mm": CAN.min_gap * 1000, "max_rock": CAN.max_rock}), flush=True)
        return
    # left hand: the handle's side arm, fingers closing across it, tilted 45 degrees toward the far end
    one_hand = mode in ("noleft", "noleft_release")  # lazy routes: the right hand alone; noleft_release also lets go once the bow has clicked in
    if not one_hand:
        hg = d.tilted(45.0, float(os.environ.get("CAN_LPHI", "-45")))  # leaning out, away from the shell's wall
        if os.environ.get("CAN_LMODE", "") == "side":  # level, from outside the handle's arm, fingers above and below it
            A = RZ @ np.array([1.0, 0.0, 0.0])  # pointing in across the carrier (local +x is the left hand's outside for this turn)
            if np.dot(A, [0.0, 1.0, 0.0]) > 0:
                A = -A
            C = np.array([0.0, 0.0, 1.0])
            rot = np.zeros((3, 3))
            rot[:, 0], rot[:, 2] = C, A
            rot[:, 1] = np.cross(A, C)
            hg = d._as_grip(rot)
        ax = hg.rot[:, 2]
        h = W(HANDLE_GRIP)
        part_open = float(os.environ.get("CAN_LOPEN", "-0.024"))  # the fingers opened 48 mm, not the full 94: the shell wall is 4 cm inside the arm
        h = h + np.array([0.0, 0.0, float(os.environ.get("CAN_LDZ", "0.01"))])  # the pads' middle a little over the arm's middle: the hand does not press the handle down
        yield from L.move(h - 0.08 * ax, grip=hg, finger=part_open, t=2.5, label="behind the handle's side arm")
        yield from L.move(h, t=1.5, label="fingers either side of the arm")
        if os.environ.get("CAN_LSERVO", "0") == "1":
            yield from L.servo(h, timeout=0.8, label="on the handle")
        yield from L.close(0.020, force=float(os.environ.get("CAN_LF", "35.0")), label="steady the handle")
        for _ in range(int(os.environ.get("CAN_LEVEL_N", "3"))):  # level the carrier: read its rock and lift or lower the handle to undo it
            th = math.radians(ctx.deg("canopy", "rock")) if hasattr(ctx, "deg") else ctx.joint("canopy", "rock")
            dz = -HANDLE_GRIP[1] * math.sin(th)
            if abs(dz) < 0.001:
                break
            yield from L.move(L.at() + np.array([0.0, 0.0, dz]), t=0.6, label="level the carrier")
            yield from L.wait(0.3)
        print("[canopy] levelled: " + json.dumps(CAN.last), flush=True)
    # right hand: the front bow's tube, read where it is now
    rock = lambda: math.degrees(ctx.joint("canopy", "rock"))  # noqa: E731
    qf = lambda: math.degrees(ctx.joint("canopy", "bow_f")) + Q0["bow_f"]  # noqa: E731
    p, rot = bow_grip(qf(), rock())
    g = d._as_grip(rot)
    ax = rot[:, 2]
    yield from R.move(p - 0.08 * ax, grip=g, finger=d.FINGER_OPEN, t=2.5, label="behind the front bow")
    p, rot = bow_grip(qf(), rock())
    yield from R.move(p - 0.08 * rot[:, 2], grip=d._as_grip(rot), t=0.6, label="lined up on the bow")
    yield from R.move(p, t=1.0, label="fingers either side of the bow")
    yield from R.servo(p, timeout=1.0, label="on the bow")
    yield from R.close(0.016, force=float(os.environ.get("CAN_FORCE", "25.0")), label="take the front bow")
    print("[canopy] gripped: " + json.dumps(CAN.last), flush=True)
    if mode == "grip":
        return
    hinge = W([0.0, YH, RIM])
    axis = RZ @ np.array([1.0, 0.0, 0.0])
    if os.environ.get("CAN_SMOOTH", "1") == "1":  # one smooth swing to the stop, at about 7 degrees a second
        dq = STOP + float(os.environ.get("CAN_OVER", "3")) - qf()  # a little past the stop: the bow lags the hand by a few degrees
        frac = float(os.environ.get("CAN_ROTFRAC", "0.3"))  # the hand turns this share of the bow's swing: the round tube rolls in the pads for the rest
        from yam_arena import ik as _ik
        from yam_arena.urdf import axis_angle_matrix as _aam

        p0, r0 = _ik.tcp_pose(R.q, R.base)
        n = max(1, round(max(3.0, dq / float(os.environ.get("CAN_DEGS", "7"))) * d.CONTROL_HZ))
        worst = [0.0]
        last = [r0]

        def draw():
            for k in range(1, n + 1):
                a = math.radians(dq) * _ik.smoothstep(k / n)
                pos = hinge + _aam(axis, a) @ (p0 - hinge)
                last[0] = _aam(axis, frac * a) @ r0
                mm, _ = R._solve(pos, last[0])
                worst[0] = max(worst[0], mm)
                yield

        def level():  # the left hand keeps the carrier level through the draw: it reads the rock and lifts or lowers the handle a little each step
            if one_hand:
                return
            for _ in range(n + round(1.5 * d.CONTROL_HZ)):
                th = math.radians(ctx.deg("canopy", "rock"))
                pl, rl = _ik.tcp_pose(L.q, L.base)
                L._solve(pl + np.array([0.0, 0.0, -float(os.environ.get("CAN_LGAIN", "0.3")) * HANDLE_GRIP[1] * math.sin(th)]), rl)
                yield

        yield from ctx.together(draw(), level())
        rot = last[0]
        worst = worst[0]
        R.grip = d._as_grip(rot)
        print(f"[canopy] pulled (IK worst {worst:.1f} mm): " + json.dumps(CAN.last), flush=True)
    step = float(os.environ.get("CAN_STEP", "10"))
    q_hand = qf()  # where the hand has been sent along the bow's arc
    for k in range(0 if os.environ.get("CAN_SMOOTH", "1") == "1" else 14):  # stepwise: the front bow drawn a few degrees at a time, reading the bow after each
        q_now = qf()
        if q_now >= STOP - 1.0:
            break
        if q_hand - q_now > 8.0:  # the bow lags the hand: the grip has slipped, so stop sending the hand on
            print(f"[canopy] bow lags the hand: hand {q_hand:.0f}, bow {q_now:.0f} deg", flush=True)
            break
        dq = min(step, STOP + 1.0 - q_hand)
        yield from R.arc(hinge, axis, dq, t=float(os.environ.get("CAN_ARC_T", "1.5")), label=f"draw the canopy forward ({q_now:.0f} deg)")
        q_hand += dq
        print(f"[canopy] pull {k}: " + json.dumps(CAN.last), flush=True)
    yield from R.wait(1.6)
    if mode == "noleft_release":  # let go of the clicked-in bow, draw the hand back, and let the carrier settle
        yield from R.open(t=0.6, label="let go of the bow")
        yield from R.move(R.at() - 0.08 * R.approach_axis(), t=1.0, label="hand back")
        yield from R.wait(4.0)
    print("[canopy] closed: " + json.dumps(CAN.last), flush=True)
    print("[canopy] END " + json.dumps({"mode": mode, "score": CAN.score(), "max_rock": CAN.max_rock, "end_rock": CAN.rock, "max_speed": CAN.max_speed, "max_travel_cm": CAN.max_travel * 100,
                                         "min_gap_mm": CAN.min_gap * 1000, "min_doll_mm": CAN.min_doll * 1000, "contact_steps": CAN.contact_steps, "bad": CAN.bad, "passed": CAN.passed,
                                         "at_stop": CAN.at_stop, "full": CAN.full, "hold_steps": CAN.hold_steps}), flush=True)
