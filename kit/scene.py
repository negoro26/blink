"""Blink Kit scene: a forklift reverses into a rack's end upright and the boxes above an ANYmal-C fall into the aisle.

One deterministic scenario (CPU PhysX, fixed dt, fixed seed, pinned box paths) run by hand-stepped physics:
SimulationManager.step() per physics tick, the ANYmal policy in the post-physics callback, renders only when asked
(record mode: every 1/30 s of sim time; live mode: on a snapshot request). The protocol server from
SIM_PROTOCOL.md runs in the same loop (newline JSON over TCP, the catch-up time rule, blink_log.jsonl).

Usage (Isaac Sim python): scene.py --out RUN_DIR [--serve] [--record] [--port 47800] [--until T]
  --serve   listen on 127.0.0.1:$BLINK_PORT and wait for a client before stepping
  --record  1080p chase + 720p mast frames from t_event - pre to the end, with a per-frame sim-time index
"""

import argparse
import json
import math
import os
import select
import socket
import sys
import time
from concurrent.futures import ThreadPoolExecutor

T0 = time.time()
ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
ap.add_argument("--cfg", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "shot.json"))
ap.add_argument("--serve", action="store_true")
ap.add_argument("--port", type=int, default=int(os.environ.get("BLINK_PORT", "47800")))
ap.add_argument("--wait_client_s", type=float, default=900.0)
ap.add_argument("--record", action="store_true")
ap.add_argument("--workers", type=int, default=3)
ap.add_argument("--no_ira", action="store_true", help="open the stage directly (no IRA, no workers)")
ap.add_argument("--until", type=float, default=0.0, help="stop at this sim time (0 = scenario end)")
ap.add_argument("--stills", default="", help="comma list of sim times to save chase stills (no full record)")
ap.add_argument("--hold", default="", help="scripted command for tests: 't0,t1,vx,vy,wz' (robot frame)")
ap.add_argument("--set", action="append", default=[], help="override a shot.json value: dotted.key=<json>")
ap.add_argument("--reflex", action="store_true", help="the robot's local safety layer evades on its own (no brain)")
args, _ = ap.parse_known_args()
OUT = args.out
os.makedirs(OUT, exist_ok=True)
SHOT = json.load(open(args.cfg))
for kv in args.set:
    key, val = kv.split("=", 1)
    node = SHOT
    for part in key.split(".")[:-1]:
        node = node[part]
    node[key.split(".")[-1]] = json.loads(val)
json.dump(SHOT, open(os.path.join(OUT, "shot_used.json"), "w"), indent=1)

from isaacsim import SimulationApp

app = SimulationApp(
    {
        "headless": True,
        "width": 1920,
        "height": 1080,
        "renderer": "RayTracedLighting",
        "active_cuda_gpus": [0],
        "multi_gpu": False,
    }
)
LOGF = open(os.path.join(OUT, "scene.log"), "a")


def log(*a):
    msg = "SCENE " + " ".join(str(x) for x in a)
    print(msg, flush=True)
    LOGF.write(msg + "\n")
    LOGF.flush()


log(f"app_up_s={time.time() - T0:.1f}")
from isaacsim.core.utils.extensions import enable_extension

for ext in ["omni.anim.navigation.core", "isaacsim.replicator.agent.core", "isaacsim.robot.policy.examples"]:
    enable_extension(ext)
for _ in range(5):
    app.update()

import carb
import numpy as np
import omni.kit.app
import omni.replicator.core as rep
import omni.timeline
import omni.usd
import yaml
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.simulation_manager.impl.isaac_events import IsaacEvents
from isaacsim.robot.policy.examples import RobotPolicyRunner, get_anymal_spec
from isaacsim.storage.native import get_assets_root_path
from omni.physx import get_physx_interface, get_physx_scene_query_interface, get_physx_simulation_interface
from PIL import Image
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

ROOT = get_assets_root_path()
DT = float(SHOT["physics_dt"])

# ---------------------------------------------------------------- stage + workers through IRA (Isaac Replicator Agent)
ira_ver = omni.kit.app.get_app().get_extension_manager().get_enabled_extension_id("isaacsim.replicator.agent.core")
ira_ver = ira_ver.split("-")[-1].split("+")[0]
cfg = {
    "version": ".".join(ira_ver.split(".")[:2]) + ".0",
    "seed": int(SHOT["seed"]),
    "simulation_duration": 600.0,
    "environment": {"base_stage_asset_path": SHOT["stage"]},
}
if args.workers:
    cfg["character"] = {
        "groups": {
            f"worker_{i}": {
                "num": 1,
                "asset_path": "Isaac/People/Characters/",
                "spawn_positions": [w["path"][0]],
                "triggers": [
                    {
                        "time_trigger": {
                            "time": 0.0,
                            "priority": 10,
                            "behavior": [
                                {"patrol": {"speed_range": [w.get("speed", 1.1)] * 2, "path_points": w["path"]}}
                            ],
                        }
                    }
                ],
            }
            for i, w in enumerate(SHOT["workers"][: args.workers])
        }
    }
cfg_path = os.path.join(OUT, "ira_config.yaml")
yaml.safe_dump({"isaacsim.replicator.agent": cfg}, open(cfg_path, "w"), sort_keys=False)

import contextlib

import isaacsim.replicator.agent.core.scene_assembly.environment_loader as _el
import omni.anim.navigation.core as nav
import omni.metropolis.pipeline.simulation_util as _su
from isaacsim.replicator.agent.core import api as ira_api


async def _navmesh_ready_wait() -> bool:
    # omni.metropolis.pipeline gives the navmesh bake 100 app frames (~2 s headless); the warehouse bake takes ~4 s,
    # so IRA "fails" and spawns nobody (KitProto, 2026-11-21). Wait on the bake itself.
    inav = nav.acquire_interface()
    if inav.get_navmesh() is not None:
        return True
    if not inav.start_navmesh_baking():
        return False
    t_ = time.time()
    while inav.get_navmesh() is None and time.time() - t_ < 300:
        await omni.kit.app.get_app().next_update_async()
    return inav.get_navmesh() is not None


_su.ensure_navmesh_ready = _navmesh_ready_wait
_el.ensure_navmesh_ready = _navmesh_ready_wait
# IRA picks characters at random from Isaac/People/Characters/, which holds a grey mannequin; keep construction workers.
carb.settings.get_settings().set(
    "/ext/isaacsim.replicator.agent/character_asset_exclude",
    ["biped_demo", "Biped", "original_", "Reallusion", "police", "Medical", "Business"],
)
if args.no_ira:
    omni.usd.get_context().open_stage(f"{get_assets_root_path()}/{SHOT['stage']}")
else:
    assert ira_api.load_config_file(cfg_path), "IRA config rejected"
    app.run_coroutine(ira_api.setup_simulation())
for _ in range(10):
    app.update()
stage = omni.usd.get_context().get_stage()
log(f"ira_setup_s={time.time() - T0:.1f}")

# ---------------------------------------------------------------- look: ACES + NVIDIA warehouse lighting recipe
s = carb.settings.get_settings()
L = SHOT["look"]
s.set("/rtx/post/tonemap/op", 4)
s.set("/rtx/post/tonemap/filmIso", float(L["film_iso"]))
s.set("/rtx/post/tonemap/whitepoint", 6500.0)
s.set("/rtx/post/tonemap/enabled", True)
s.set("/rtx/post/aa/op", int(L.get("aa_op", 3)))
if L.get("recipe", True):
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from warehouse_lighting import add_warehouse_lighting

    add_warehouse_lighting(stage, n_lights=len(L["highbay_xy"]), settings=s if L.get("fog", True) else None)
    for i, (x, y) in enumerate(L["highbay_xy"]):  # the recipe leaves its high-bays unplaced
        UsdGeom.Xformable(stage.GetPrimAtPath(f"/World/L/HB{i}")).AddTranslateOp().Set(Gf.Vec3d(x, y, L["highbay_z"]))
        stage.GetPrimAtPath(f"/World/L/HB{i}").GetAttribute("inputs:intensity").Set(float(L["highbay_intensity"]))
    stage.GetPrimAtPath("/World/L/Dome").GetAttribute("inputs:intensity").Set(float(L["dome"]))

# ---------------------------------------------------------------- set dressing: two loaded pallets fence the robot's spot
bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])


def ref_asset(prim_path):
    for spec in stage.GetPrimAtPath(prim_path).GetPrimStack():
        items = list(spec.referenceList.prependedItems) + list(spec.referenceList.explicitItems)
        if items:
            return spec.layer.ComputeAbsolutePath(items[0].assetPath)
    raise RuntimeError(f"no reference on {prim_path}")


def place(path, asset, xyz, yaw_deg=0.0):
    p = stage.DefinePrim(path, "Xform")
    p.GetReferences().AddReference(asset)
    xf = UsdGeom.Xformable(p)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*xyz))
    xf.AddRotateZOp().Set(float(yaw_deg))
    return p


pallet_asset = ref_asset(SHOT["pallet_ref"])
for i, pl in enumerate(SHOT["pallets"]):
    px, py = pl["pos"]
    place(f"/World/Blink/Pallet{i}", pallet_asset, (px, py, 0.0))
    for j, (src, (ox, oy)) in enumerate(pl["boxes"]):
        place(f"/World/Blink/Pallet{i}_Box{j}", ref_asset(src), (px + ox, py + oy, 0.212), 90.0)

# ---------------------------------------------------------------- forklift: kinematic, scripted schedule
F = SHOT["forklift"]
fk = place("/World/Blink/Forklift", f"{ROOT}/{F['usd']}", (F["x"], 0.0, 0.0), F["yaw_deg"])
fk_rb = UsdPhysics.RigidBodyAPI.Apply(fk)
fk_rb.CreateKinematicEnabledAttr(True)
fk_translate = next(
    op for op in UsdGeom.Xformable(fk).GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeTranslate
)
T_EVENT = float(F["t_event"])
Y_HIT = F["y_rear_hit"] - F["rear"]  # forklift origin y when its rear meets the end upright (yaw +90: rear is +y)
Y_REV0 = Y_HIT - F["reverse_d"]  # origin y when it starts reversing (the EVENT)
Y_PARK = Y_REV0 + F["pull_out_d"]
T_PULL0 = T_EVENT - F["pause_s"] - F["pull_out_s"]
TA = F["reverse_v"] / F["reverse_a"]
SA = 0.5 * F["reverse_a"] * TA * TA
T_HIT = T_EVENT + (
    math.sqrt(2 * F["reverse_d"] / F["reverse_a"])
    if F["reverse_d"] <= SA
    else TA + (F["reverse_d"] - SA) / F["reverse_v"]
)


def forklift_y(t):
    if t < T_PULL0:
        return Y_PARK
    if t < T_EVENT - F["pause_s"]:  # pull out forks-first (south), smoothstep
        u = (t - T_PULL0) / F["pull_out_s"]
        return Y_PARK - F["pull_out_d"] * (3 * u * u - 2 * u * u * u)
    if t < T_EVENT:
        return Y_REV0
    tau = t - T_EVENT
    if t < T_HIT:
        sd = 0.5 * F["reverse_a"] * tau * tau if tau < TA else SA + F["reverse_v"] * (tau - TA)
        return Y_REV0 + sd
    u = min((t - T_HIT) / 0.15, 1.0)  # small rebound off the upright
    return Y_HIT - 0.06 * math.sin(math.pi * 0.5 * u)


def forklift_vy(t):
    return (forklift_y(t + 1e-3) - forklift_y(t - 1e-3)) / 2e-3


fk_translate.Set(Gf.Vec3d(F["x"], forklift_y(0.0), 0.0))
log(f"forklift t_event={T_EVENT:.3f} t_hit={T_HIT:.3f} y_rev0={Y_REV0:.3f} y_hit={Y_HIT:.3f}")

# ---------------------------------------------------------------- hazard boxes: the incident-tagged loose items of the end bay
B = SHOT["boxes"]
LOOSE = "IsaacSim_Replicator_Incident_Attr:LooseItem"
box_paths = sorted(
    {
        "/".join(str(p.GetPath()).split("/")[:4])
        for p in stage.Traverse()
        if p.HasAttribute(LOOSE) and str(p.GetPath()).startswith(B["root"] + "/")
    }
)
rng = np.random.default_rng(int(SHOT["seed"]))
BOX = {}
cardboard = UsdShade.Material.Define(stage, "/World/Blink/Cardboard")
cb_api = UsdPhysics.MaterialAPI.Apply(cardboard.GetPrim())
cb_api.CreateStaticFrictionAttr(float(B["friction"]))
cb_api.CreateDynamicFrictionAttr(float(B["friction"]))
cb_api.CreateRestitutionAttr(0.0)
# The jolted shelf deck is slick (combine "min" outranks the boxes' "average"), so a modest shove slides the boxes off
# and the floor (average with the boxes' friction) stops them near the rack instead of across the aisle.
slick = UsdShade.Material.Define(stage, "/World/Blink/SlickShelf")
sl_api = UsdPhysics.MaterialAPI.Apply(slick.GetPrim())
sl_api.CreateStaticFrictionAttr(float(B["shelf_friction"]))
sl_api.CreateDynamicFrictionAttr(float(B["shelf_friction"]))
PhysxSchema.PhysxMaterialAPI.Apply(slick.GetPrim()).CreateFrictionCombineModeAttr("min")
for sp in B["shelves"]:
    for q in Usd.PrimRange(stage.GetPrimAtPath(sp)):
        if q.HasAPI(UsdPhysics.CollisionAPI):
            UsdShade.MaterialBindingAPI.Apply(q).Bind(slick, UsdShade.Tokens.strongerThanDescendants, "physics")
for path in box_paths:
    p = stage.GetPrimAtPath(path)
    r = bbc.ComputeWorldBound(p).ComputeAlignedRange()
    size = r.GetMax() - r.GetMin()
    for q in Usd.PrimRange(p):
        if q.HasAPI(UsdPhysics.MeshCollisionAPI) or q.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.MeshCollisionAPI.Apply(q).CreateApproximationAttr().Set("convexHull")
            UsdShade.MaterialBindingAPI.Apply(q).Bind(cardboard, UsdShade.Tokens.strongerThanDescendants, "physics")
    UsdPhysics.RigidBodyAPI.Apply(p)
    big = size[0] * size[1] * size[2] > 0.05
    UsdPhysics.MassAPI.Apply(p).CreateMassAttr(float(B["mass_big"] if big else B["mass_small"]))
    PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr(0.0)
    sv = B["shove"]
    d = np.array(sv["dir"], float)
    n = np.array([-d[1], d[0]])
    vh = rng.uniform(*sv["vx"]) * d + rng.uniform(*sv["vy"]) * n
    BOX[path] = {
        "r": 0.5 * float(np.linalg.norm([size[0], size[1], size[2]])),
        "v": [float(vh[0]), float(vh[1]), float(rng.uniform(*sv["vz"]))],
        "w": [float(v) for v in rng.uniform(-sv["w_deg"], sv["w_deg"], 3)],
    }
log(f"hazard_boxes={len(BOX)}")

# ---------------------------------------------------------------- physics scene (CPU, fixed dt, enhanced determinism)
scenes = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
scene_prim = scenes[0] if scenes else UsdPhysics.Scene.Define(stage, "/World/PhysicsScene").GetPrim()
PhysxSchema.PhysxSceneAPI.Apply(scene_prim).CreateEnableEnhancedDeterminismAttr(True)
SimulationManager.set_physics_sim_device("cpu")

# ---------------------------------------------------------------- ANYmal-C (Isaac Sim 6.2's bundled policy example)
RB = SHOT["robot"]
yaw0 = math.radians(RB["yaw_deg"])
anymal = RobotPolicyRunner(
    get_anymal_spec(),
    prim_path="/World/Anymal",
    position=RB["start"],
    orientation=[math.cos(yaw0 / 2), 0.0, 0.0, math.sin(yaw0 / 2)],
)
anymal.spawn()
SimulationManager.set_physics_dt(DT)
CMD = np.zeros(3, dtype=np.float32)
POL = {"first": True, "err": None}


def on_post_physics(step_size, context):
    if POL["err"]:
        return
    try:
        if POL["first"]:
            anymal.restart_from_default_state(CMD)
            POL["first"] = False
        else:
            anymal.step(step_size, CMD)
    except Exception as e:
        POL["err"] = repr(e)
        carb.log_error(f"anymal step failed: {e}")


_cb = SimulationManager.register_callback(on_post_physics, IsaacEvents.POST_PHYSICS_STEP)

# ---------------------------------------------------------------- cameras
C = SHOT["cameras"]


def make_cam(path, focal):
    c = UsdGeom.Camera.Define(stage, path)
    c.CreateFocalLengthAttr(float(focal))
    c.CreateClippingRangeAttr(Gf.Vec2f(0.05, 500.0))
    xf = UsdGeom.Xformable(c.GetPrim())
    xf.ClearXformOpOrder()
    return xf.AddTransformOp()


def look(op, eye, target):
    op.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse())


chase_op = make_cam("/World/Blink/ChaseCam", C["chase"]["focal"])
look(chase_op, C["chase"]["eye"], C["chase"]["target"])
mast_op = make_cam("/World/Blink/MastCam", C["mast"]["focal"])
want_chase = args.record or bool(args.stills)
rp_mast = rep.create.render_product("/World/Blink/MastCam", tuple(C["mast"]["res"]))
ann_mast = rep.AnnotatorRegistry.get_annotator("rgb")
ann_mast.attach([rp_mast])
if want_chase:
    rp_chase = rep.create.render_product("/World/Blink/ChaseCam", tuple(C["chase"]["res"]))
    ann_chase = rep.AnnotatorRegistry.get_annotator("rgb")
    ann_chase.attach([rp_chase])
rep.orchestrator.set_capture_on_play(False)
for _ in range(60 if want_chase else 5):
    app.update()

# ---------------------------------------------------------------- start physics; from here the loop owns every physics step
timeline = omni.timeline.get_timeline_interface()
timeline.play()
app.update()
s.set_bool("/app/player/playSimulations", False)  # renders no longer step physics (RenderingManager.render pattern)
t_sim = SimulationManager.get_simulation_time()
log(f"physics_started t_sim={t_sim:.3f} policy_err={POL['err']} setup_s={time.time() - T0:.1f}")
physx = get_physx_interface()
sim_if = get_physx_simulation_interface()
sq = get_physx_scene_query_interface()


def robot_pose():
    p, q = anymal.articulation.get_world_poses()
    p = p.numpy().reshape(-1)[:3].astype(float)
    w, x, y, z = (float(v) for v in q.numpy().reshape(-1)[:4])
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    up_z = 1 - 2 * (x * x + y * y)  # world z of the body z axis
    return p, yaw, math.degrees(math.acos(max(-1.0, min(1.0, up_z))))


def to_robot(vx, vy, yaw):
    c, s_ = math.cos(yaw), math.sin(yaw)
    return c * vx + s_ * vy, -s_ * vx + c * vy


DIRS = {"front": (1.0, 0.0), "back": (-1.0, 0.0), "left": (0.0, 1.0), "right": (0.0, -1.0)}
ZONE = SHOT["zone"]
DYNAMIC_PREFIX = ("/World/Anymal", "/World/Blink/Forklift", "/World/Characters", "/World/Robots", *tuple(BOX))


def free_m(p, yaw):
    """Distance along each robot-frame direction from the footprint edge to the first static collider (raycasts)."""
    out = {}
    c, s_ = math.cos(yaw), math.sin(yaw)
    for name, (fx, fy) in DIRS.items():
        ux, uy = c * fx - s_ * fy, s_ * fx + c * fy  # world direction
        px, py = -uy, ux  # world perpendicular
        edge = RB["half_len"] if fx else RB["half_wid"]
        side = RB["half_wid"] if fx else RB["half_len"]
        best = 10.0
        for off in (-0.85 * side, 0.0, 0.85 * side):
            for hz in (0.15, 0.45):
                o = carb.Float3(p[0] + ux * edge + px * off, p[1] + uy * edge + py * off, hz)
                hits = []

                def rep_fn(h, hits=hits):
                    if not str(h.collision).startswith(DYNAMIC_PREFIX) and not str(h.rigid_body).startswith(
                        DYNAMIC_PREFIX
                    ):
                        hits.append(h.distance)
                    return True

                sq.raycast_all(o, carb.Float3(ux, uy, 0.0), 10.0, rep_fn)
                if hits:
                    best = min(best, min(hits))
        out[name] = round(best, 3)
    return out


def footprint_aabb(p, yaw):
    c, s_ = abs(math.cos(yaw)), abs(math.sin(yaw))
    hx = RB["half_len"] * c + RB["half_wid"] * s_
    hy = RB["half_len"] * s_ + RB["half_wid"] * c
    return p[0] - hx, p[1] - hy, p[0] + hx, p[1] + hy


def exit_m(p, yaw):
    """Distance along each robot-frame direction until the footprint no longer overlaps the predicted landing zone."""
    x0, y0, x1, y1 = footprint_aabb(p, yaw)
    inside = x1 > ZONE[0] and x0 < ZONE[2] and y1 > ZONE[1] and y0 < ZONE[3]
    c, s_ = math.cos(yaw), math.sin(yaw)
    out = {}
    for name, (fx, fy) in DIRS.items():
        if not inside:
            out[name] = 0.0
            continue
        ux, uy = c * fx - s_ * fy, s_ * fx + c * fy
        need = []
        if ux > 1e-6:
            need.append((ZONE[2] - x0) / ux)
        if ux < -1e-6:
            need.append((x1 - ZONE[0]) / -ux)
        if uy > 1e-6:
            need.append((ZONE[3] - y0) / uy)
        if uy < -1e-6:
            need.append((y1 - ZONE[1]) / -uy)
        out[name] = round(min(need), 3) if need else None
    return out


T_IMPACT = SHOT.get("t_impact_nobrain") or (T_HIT + 0.5)


def hazard_state(p, yaw, t):
    zx, zy = 0.5 * (ZONE[0] + ZONE[2]) - p[0], 0.5 * (ZONE[1] + ZONE[3]) - p[1]
    hx, hy = to_robot(zx, zy, yaw)
    return {
        "kind": "falling_boxes",
        "x": round(hx, 3),
        "y": round(hy, 3),
        "vx": 0.0,
        "vy": 0.0,
        "time_to_impact_s": round(max(0.0, T_IMPACT - t), 3),
        "exit_m": exit_m(p, yaw),
    }


# ---------------------------------------------------------------- protocol server
BLOG = open(os.path.join(OUT, "blink_log.jsonl"), "a")


def blog(rec):
    rec.setdefault("wall_ns", time.time_ns())
    rec.setdefault("t_sim", round(t_sim, 4))
    BLOG.write(json.dumps(rec) + "\n")
    BLOG.flush()


class Server:
    def __init__(self, port):
        self.ls = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ls.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.ls.bind(("127.0.0.1", port))
        self.ls.listen(4)
        self.ls.setblocking(False)
        self.clients, self.buf, self.queue = [], {}, []

    def poll(self):
        r, _, _ = select.select([self.ls, *self.clients], [], [], 0)
        for sock in r:
            if sock is self.ls:
                c, _ = self.ls.accept()
                c.setblocking(True)
                c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.clients.append(c)
                self.buf[c] = b""
                blog({"dir": "info", "msg": "client connected"})
                if EV["msg"]:
                    self.send(c, EV["msg"])
                continue
            data = sock.recv(65536)
            if not data:
                self.clients.remove(sock)
                self.buf.pop(sock, None)
                blog({"dir": "info", "msg": "client closed"})
                continue
            self.buf[sock] += data
            while b"\n" in self.buf[sock]:
                line, self.buf[sock] = self.buf[sock].split(b"\n", 1)
                if line.strip():
                    try:
                        msg = json.loads(line)
                    except Exception as e:
                        self.send(sock, {"id": None, "error": f"bad json: {e}"})
                        continue
                    blog({"dir": "in", "msg": msg})
                    self.queue.append([sock, msg, None])

    def send(self, sock, obj):
        with contextlib.suppress(OSError):
            sock.sendall((json.dumps(obj) + "\n").encode())
        rec = {
            "dir": "out",
            "msg": obj if "jpeg_b64" not in obj else {**obj, "jpeg_b64": f"<{len(obj['jpeg_b64'])} b64 chars>"},
        }
        blog(rec)

    def broadcast(self, obj):
        if not self.clients:
            blog({"dir": "out", "msg": obj, "note": "no client"})
        for c in list(self.clients):
            self.send(c, obj)


EV = {"msg": None, "t": None, "wall": None}
srv = Server(args.port) if args.serve else None
if srv:
    log(f"listening 127.0.0.1:{args.port}")
    t_w = time.time()
    while not srv.clients and time.time() - t_w < args.wait_client_s:
        srv.poll()
        time.sleep(0.01)
    log(f"client_connected={bool(srv.clients)} waited_s={time.time() - t_w:.1f}")

# ---------------------------------------------------------------- run state
REC = SHOT["record"]
FPS = int(REC["fps"])
T_OUTCOME = T_HIT + float(SHOT["t_end_after_impact"])
T_END = args.until if args.until > 0 else T_OUTCOME + (REC["post"] if args.record else 0.5)
T_REC0 = T_EVENT - REC["pre"]
STILLS = sorted(float(x) for x in args.stills.split(",") if x)
MODE = {"kind": "hold", "x": float(RB["start"][0]), "y": float(RB["start"][1]), "yaw": yaw0}
HOLD = [float(v) for v in args.hold.split(",")] if args.hold else None
ST = {
    "contact_t": None,
    "max_force": 0.0,
    "min_gap": 99.0,
    "min_z": 9.0,
    "max_tilt": 0.0,
    "shoved": False,
    "n": 0,
    "last_pose": None,
    "vel": (0.0, 0.0, 0.0),
    "frame": 0,
    "next_frame_t": 0.0,
    "outcome_sent": False,
    "contacts": [],
    "render_ms": [],
    "step_ms": [],
    "landed": [99.0, 99.0, -99.0, -99.0],
}  # xy AABB swept by box centres below robot height after the shove
TRACK = open(os.path.join(OUT, "track.jsonl"), "w")
FR_IDX = open(os.path.join(OUT, "frames.jsonl"), "w") if args.record else None
if args.record:
    os.makedirs(os.path.join(OUT, "chase"), exist_ok=True)
    os.makedirs(os.path.join(OUT, "mast"), exist_ok=True)
pool = ThreadPoolExecutor(6)
ROBOT_PREFIX = "/World/Anymal"


def mast_pose(p, yaw):
    o = C["mast"]["offset"]
    c, s_ = math.cos(yaw), math.sin(yaw)
    eye = (p[0] + c * o[0] - s_ * o[1], p[1] + s_ * o[0] + c * o[1], p[2] + o[2])
    pt = math.radians(C["mast"]["pitch_deg"])
    tgt = (eye[0] + c * math.cos(pt), eye[1] + s_ * math.cos(pt), eye[2] - math.sin(pt))
    look(mast_op, eye, tgt)


def render():
    p, yaw, _ = robot_pose()
    mast_pose(p, yaw)
    t_ = time.perf_counter()
    rep.orchestrator.step(delta_time=1.0 / FPS, pause_timeline=False, rt_subframes=1)
    ST["render_ms"].append((time.perf_counter() - t_) * 1000)
    ST["mast_t"] = t_sim


def mast_jpeg():
    import base64
    import io

    img = Image.fromarray(ann_mast.get_data()[:, :, :3]).resize((640, 360), Image.BILINEAR)
    b = io.BytesIO()
    img.save(b, "JPEG", quality=80)
    return base64.b64encode(b.getvalue()).decode()


def hold_here(p, yaw):
    MODE.clear()
    MODE.update(kind="hold", x=float(p[0]), y=float(p[1]), yaw=float(yaw))


def command_for(t, p, yaw):
    """Base-frame [vx, vy, wz] for this control tick. 'hold' station-keeps (the standing policy drifts ~4 cm/s)."""
    if HOLD and HOLD[0] <= t < HOLD[1]:
        MODE["after_hold"] = True
        return HOLD[2:5]
    if MODE.pop("after_hold", False):
        hold_here(p, yaw)
    k = MODE["kind"]
    if k == "evade":
        if t < MODE["until"]:
            return MODE["v"]
        hold_here(p, yaw)
        k = "hold"
    ex, ey = to_robot(MODE["x"] - p[0], MODE["y"] - p[1], yaw)
    dist = math.hypot(ex, ey)
    eyaw = math.atan2(math.sin(MODE["yaw"] - yaw), math.cos(MODE["yaw"] - yaw))
    if k == "goal":
        if dist < 0.15 and abs(eyaw) < 0.1:
            hold_here(p, yaw)
            return [0.0, 0.0, 0.0]
        sp = min(1.0, dist) / max(dist, 1e-6)
        return [ex * sp, ey * sp, max(-1.0, min(1.0, 1.5 * eyaw))]
    sp = min(0.3, 3.0 * dist) / max(dist, 1e-6)
    return [ex * sp, ey * sp, max(-0.5, min(0.5, 3.0 * eyaw))]


def physics_step():
    """One physics tick of the whole scenario at the current command."""
    global t_sim
    t_next = t_sim + DT
    fk_translate.Set(Gf.Vec3d(F["x"], forklift_y(t_next), 0.0))
    if not ST["shoved"] and t_next >= T_HIT:
        for path, b in BOX.items():
            rb = UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(path))
            rb.GetVelocityAttr().Set(Gf.Vec3f(*b["v"]))
            rb.GetAngularVelocityAttr().Set(Gf.Vec3f(*b["w"]))
        ST["shoved"] = True
        blog({"dir": "info", "msg": "boxes shoved", "t_shove": round(t_next, 4)})
    t_ = time.perf_counter()
    SimulationManager.step(steps=1)
    ST["step_ms"].append((time.perf_counter() - t_) * 1000)
    t_sim = SimulationManager.get_simulation_time()
    ST["n"] += 1
    # contacts between hazard boxes and the robot
    headers, data = sim_if.get_contact_report()
    f_step = 0.0
    for h in headers:
        a0, a1 = str(PhysicsSchemaTools.intToSdfPath(h.actor0)), str(PhysicsSchemaTools.intToSdfPath(h.actor1))
        if (a0 in BOX and a1.startswith(ROBOT_PREFIX)) or (a1 in BOX and a0.startswith(ROBOT_PREFIX)):
            imp = 0.0
            for k in range(h.contact_data_offset, h.contact_data_offset + h.num_contact_data):
                iv = data[k].impulse
                imp += math.sqrt(iv[0] ** 2 + iv[1] ** 2 + iv[2] ** 2)
            if imp > 1e-4:
                f_step += imp / DT
                if ST["contact_t"] is None:
                    ST["contact_t"] = round(t_sim, 4)
                if len(ST["contacts"]) < 400:
                    ST["contacts"].append(
                        [round(t_sim, 4), a0 if a0 in BOX else a1, a1 if a0 in BOX else a0, round(imp / DT, 1)]
                    )
    ST["max_force"] = max(ST["max_force"], f_step)
    if ST["n"] % 4 == 0:  # control-rate bookkeeping (50 Hz)
        p, yaw, tilt = robot_pose()
        if ST["last_pose"] is not None:
            lp, lyaw, lt = ST["last_pose"]
            vxw, vyw = (p[0] - lp[0]) / (t_sim - lt), (p[1] - lp[1]) / (t_sim - lt)
            vx, vy = to_robot(vxw, vyw, yaw)
            ST["vel"] = (vx, vy, math.atan2(math.sin(yaw - lyaw), math.cos(yaw - lyaw)) / (t_sim - lt))
        ST["last_pose"] = (p, yaw, t_sim)
        CMD[:] = command_for(t_sim, p, yaw)
        gap = 99.0
        c, s_ = math.cos(yaw), math.sin(yaw)
        for path, b in BOX.items():
            tr = physx.get_rigidbody_transformation(path)
            if not tr["ret_val"]:
                continue
            bx, by, bz = tr["position"]
            lx, ly = c * (bx - p[0]) + s_ * (by - p[1]), -s_ * (bx - p[0]) + c * (by - p[1])
            dx, dy = max(abs(lx) - RB["half_len"], 0.0), max(abs(ly) - RB["half_wid"], 0.0)
            dz = max(bz - RB["height"], 0.0)
            gap = min(gap, max(0.0, math.sqrt(dx * dx + dy * dy + dz * dz) - b["r"]))
            if ST["shoved"] and bz < RB["height"]:
                lz = ST["landed"]
                lz[0], lz[1], lz[2], lz[3] = min(lz[0], bx), min(lz[1], by), max(lz[2], bx), max(lz[3], by)
        if ST["shoved"]:
            ST["min_gap"] = min(ST["min_gap"], gap)
        ST["min_z"] = min(ST["min_z"], p[2])
        ST["max_tilt"] = max(ST["max_tilt"], tilt)
        TRACK.write(
            json.dumps(
                {
                    "t": round(t_sim, 4),
                    "x": round(p[0], 4),
                    "y": round(p[1], 4),
                    "z": round(p[2], 4),
                    "yaw": round(yaw, 4),
                    "tilt": round(tilt, 2),
                    "cmd": [round(float(v), 3) for v in CMD],
                    "gap": round(gap, 3),
                    "fk_y": round(forklift_y(t_sim), 3),
                    "f": round(f_step, 1),
                }
            )
            + "\n"
        )


def state_reply(rid):
    p, yaw, _ = robot_pose()
    vx, vy, wz = ST["vel"]
    return {
        "id": rid,
        "t_sim": round(t_sim, 4),
        "late_s": 0.0,
        "pose": {"x": round(p[0], 4), "y": round(p[1], 4), "yaw": round(yaw, 4)},
        "vel": {"vx": round(vx, 3), "vy": round(vy, 3), "wz": round(wz, 3)},
        "hazard": hazard_state(p, yaw, t_sim),
        "free_m": free_m(p, yaw),
    }


EVADE = {"forward": (1.0, 0.0), "back": (-1.0, 0.0), "left": (0.0, 1.0), "right": (0.0, -1.0), "stop": (0.0, 0.0)}
SNAP_MAX_AGE = 0.5


def apply_request(sock, msg, late):
    rid, op = msg.get("id"), msg.get("op")
    if op == "get_state":
        r = state_reply(rid)
        r["late_s"] = late
        srv.send(sock, r)
    elif op == "evade":
        d = msg.get("direction")
        if d not in EVADE:
            srv.send(sock, {"id": rid, "error": f"bad direction {d!r}"})
            return
        dur = float(msg.get("duration_ms", 1500)) / 1000.0
        MODE.clear()
        MODE.update(kind="evade", v=[EVADE[d][0], EVADE[d][1], 0.0], until=t_sim + dur)
        srv.send(sock, {"id": rid, "applied_at_t_sim": round(t_sim, 4), "late_s": late})
        blog({"dir": "cmd", "msg": {"evade": d, "v": MODE["v"], "until": round(MODE["until"], 4)}})
    elif op == "set_goal":
        try:
            MODE.clear()
            MODE.update(kind="goal", x=float(msg["x"]), y=float(msg["y"]), yaw=float(msg.get("yaw", 0.0)))
        except (KeyError, TypeError, ValueError) as e:
            srv.send(sock, {"id": rid, "error": f"bad goal: {e}"})
            return
        srv.send(sock, {"id": rid, "applied_at_t_sim": round(t_sim, 4), "late_s": late})
        blog({"dir": "cmd", "msg": {"goal": [MODE["x"], MODE["y"], MODE["yaw"]]}})
    elif op == "snapshot":
        # Live mode renders on demand (~0.45 s of wall time with no stepping); a frame up to SNAP_MAX_AGE sim-seconds
        # old is reused so a snapshot does not cost the brain a render. t_sim is the frame's own time.
        if not args.record and (ST.get("mast_t") is None or t_sim - ST["mast_t"] > SNAP_MAX_AGE):
            render()
        srv.send(sock, {"id": rid, "t_sim": round(ST["mast_t"], 4), "late_s": late, "jpeg_b64": mast_jpeg()})
    else:
        srv.send(sock, {"id": rid, "error": f"unknown op {op!r}"})


def t_map(wall_ns):
    return EV["t"] + (wall_ns - EV["wall"]) / 1e9


def maybe_frame():
    if not (args.record or STILLS) or t_sim + 1e-9 < ST["next_frame_t"]:
        return
    if args.record:
        render()
        if t_sim >= T_REC0 - 1e-9:
            k = ST["frame"]
            ST["frame"] += 1
            a, m = ann_chase.get_data()[:, :, :3].copy(), ann_mast.get_data()[:, :, :3].copy()
            pool.submit(lambda a=a, k=k: Image.fromarray(a).save(f"{OUT}/chase/{k:05d}.png", compress_level=1))
            pool.submit(lambda m=m, k=k: Image.fromarray(m).save(f"{OUT}/mast/{k:05d}.png", compress_level=1))
            FR_IDX.write(json.dumps({"i": k, "t_sim": round(t_sim, 4)}) + "\n")
    elif STILLS and t_sim >= STILLS[0] - 1e-9:
        STILLS.pop(0)
        render()
        render()
        Image.fromarray(ann_chase.get_data()[:, :, :3]).save(f"{OUT}/still_{t_sim:06.2f}.png")
        Image.fromarray(ann_mast.get_data()[:, :, :3]).save(f"{OUT}/mast_{t_sim:06.2f}.png")
    ST["next_frame_t"] += 1.0 / FPS


def emit_event():
    if srv and not args.record:
        render()  # fresh mast frame for the first snapshot, rendered before the event's wall stamp
    p, yaw, _ = robot_pose()
    EV["t"], EV["wall"] = t_sim, time.time_ns()
    EV["msg"] = {
        "type": "event",
        "t_sim": round(t_sim, 4),
        "t_wall_ns": EV["wall"],
        "hazard": hazard_state(p, yaw, t_sim),
        "time_to_impact_s": round(T_IMPACT - t_sim, 3),
    }
    if srv:
        srv.broadcast(EV["msg"])
    else:
        blog({"dir": "out", "msg": EV["msg"], "note": "no server"})
    log(f"EVENT t_sim={t_sim:.4f} time_to_impact_s={T_IMPACT - t_sim:.3f}")


def emit_outcome():
    ST["outcome_sent"] = True
    out = {
        "type": "outcome",
        "t_sim": round(t_sim, 4),
        "collision": ST["contact_t"] is not None,
        "t_contact": ST["contact_t"],
        "min_gap_m": round(ST["min_gap"], 3),
        "max_force_n": round(ST["max_force"], 1),
    }
    if srv:
        srv.broadcast(out)
    else:
        blog({"dir": "out", "msg": out, "note": "no server"})
    p, _yaw, _tilt = robot_pose()
    extra = {
        **out,
        "t_event": EV["t"],
        "t_hit_forklift": round(T_HIT, 4),
        "robot_final": [round(p[0], 3), round(p[1], 3), round(p[2], 3)],
        "robot_min_z": round(ST["min_z"], 3),
        "robot_max_tilt_deg": round(ST["max_tilt"], 1),
        "first_contacts": ST["contacts"][:12],
        "landed_centres_xy": [round(v, 3) for v in ST["landed"]],
        "policy_err": POL["err"],
    }
    json.dump(extra, open(os.path.join(OUT, "outcome.json"), "w"), indent=1)
    log("OUTCOME " + json.dumps(extra))


def local_reflex():
    """The robot's own safety layer: at EVENT + latency, step out along the one direction whose static clearance
    exceeds the distance to leave the impact zone (free_m > exit_m, ground truth), for exit_m + margin metres."""
    p, yaw, _ = robot_pose()
    free, ex = free_m(p, yaw), exit_m(p, yaw)
    ok = sorted((free[d] - ex[d], d) for d in DIRS if ex[d] is not None and free[d] > ex[d])
    ST["reflex_done"] = True
    if not ok:
        blog({"dir": "reflex", "msg": {"escape": None, "free_m": free, "exit_m": ex}})
        return
    d = ok[-1][1]
    dur = min(ex[d] + float(SHOT["reflex"]["margin_m"]), free[d] - 0.3)  # metres at 1.0 m/s, stop short of the rack
    MODE.clear()
    MODE.update(kind="evade", v=[*DIRS[d], 0.0], until=t_sim + dur)
    blog({"dir": "reflex", "msg": {"escape": d, "until": round(t_sim + dur, 4), "free_m": free, "exit_m": ex}})
    log(f"REFLEX t_sim={t_sim:.4f} escape={d} free_m={free} exit_m={ex}")


# ---------------------------------------------------------------- main loop
wall0 = time.time()
CTRL = 4 * DT
while t_sim < T_END - 1e-9:
    if srv:
        srv.poll()
    if EV["msg"] is None and t_sim >= T_EVENT - 1e-9:
        emit_event()
    if args.reflex and not ST.get("reflex_done") and t_sim >= T_EVENT + float(SHOT["reflex"]["latency_s"]) - 1e-9:
        local_reflex()
    if not ST["outcome_sent"] and t_sim >= T_OUTCOME - 1e-9:
        emit_outcome()
    if srv and srv.queue:
        item = srv.queue[0]
        sock, msg = item[0], item[1]
        tt = t_sim if EV["t"] is None or "t_wall_ns" not in msg else t_map(int(msg["t_wall_ns"]))
        if item[2] is None:  # first look: is the sim already past the request's mapped time?
            item[2] = round(t_sim - tt, 4) if t_sim > tt + 1e-9 else 0.0
        if t_sim < tt - 1e-9:
            physics_step()
            maybe_frame()
            continue
        srv.queue.pop(0)
        blog({"dir": "map", "id": msg.get("id"), "t_target": round(tt, 4), "late_s": item[2]})
        apply_request(sock, msg, item[2])
        continue
    if EV["t"] is not None and t_sim + DT > t_map(time.time_ns()) + CTRL:
        time.sleep(0.0005)
        continue  # never run ahead of the wall mapping by more than one control step
    physics_step()
    maybe_frame()
if not ST["outcome_sent"]:
    emit_outcome()
pool.shutdown(wait=True)
for fh in (TRACK, FR_IDX, BLOG, LOGF):
    if fh:
        fh.flush()
sm = np.array(ST["step_ms"])
rm = np.array(ST["render_ms"] or [0.0])
log(
    f"done t_sim={t_sim:.3f} steps={ST['n']} wall_s={time.time() - wall0:.1f} step_ms median={np.median(sm):.2f} p90={np.percentile(sm, 90):.2f} "
    f"render_ms median={np.median(rm):.1f} frames={ST['frame']} rtf={(t_sim) / (time.time() - wall0):.2f}"
)
timeline.stop()
os._exit(0)  # skip the Kit shutdown (known teardown hang, isaac-sim-troubleshooting 1a)
