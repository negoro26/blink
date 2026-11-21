"""Pose an NVIDIA Isaac People character (`Isaac/People/Characters/*`) standing, and wrap it into the scenario's worker.

The characters are skinned in a T pose and Isaac Sim animates them through Kit's retargeting, which a Kit-less run
does not have (their Reallusion skeleton is not the one of `People/Animations`). This script poses the skeleton itself,
upper arms lowered beside the body and elbows a little bent, bakes the skinning into static meshes (UsdSkel's
BakeSkinning) and writes two layers:

    <out>_posed.usda    the character flattened, its meshes holding the posed points, no skeleton left
    <out>               /Worker (default prim, Xform): Visual references the posed layer, turned so the worker faces
                        +x (z 0 at his soles); Collider is a capsule (CollisionAPI, invisible) around him

so the scenario places him like the forklift, by (x, y, yaw), and the forklift and the robot meet a collider. The
character's own files are only referenced through anchored asset paths, never copied or changed. Run it in the project
env (pxr):

    python demo/build_worker.py <character.usd> <out.usda> [--radius 0.28]

It prints the worker's size, the capsule and both outputs' sha256, for the scenario.
"""

import argparse
import hashlib
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdSkel

POSE = {"L_Upperarm": 72.0, "R_Upperarm": -72.0, "L_Forearm": 12.0, "R_Forearm": -12.0}
"""Joint (last path part) to its turn [deg] about the character's forward axis, at its own origin, from the T pose.
Signs are for a character facing -y in its file; the script flips them for one facing +y."""


def joint_world(skel, rest_local):
    """World (skeleton space) transforms of every joint from local ones, in the skeleton's joint order."""
    topo = UsdSkel.Topology(skel.GetJointsAttr().Get())
    world = []
    for i, local in enumerate(rest_local):
        p = topo.GetParent(i)
        world.append(local * world[p] if p >= 0 else local)
    return topo, world


def facing(skel, world):
    """The character's forward direction in the file (+1 for +y, -1 for -y), from heel to toe of the left foot."""
    names = [str(j).rsplit("/", 1)[-1] for j in skel.GetJointsAttr().Get()]
    foot = world[names.index("L_Foot")].ExtractTranslation()
    toe = world[names.index("L_ToeBase")].ExtractTranslation()
    return 1.0 if toe[1] > foot[1] else -1.0


def posed_locals(skel, forward):
    """Local joint transforms of the standing pose: POSE's turns laid on the rest pose, in world space at each joint."""
    rest = list(skel.GetRestTransformsAttr().Get())
    topo, world = joint_world(skel, rest)
    names = [str(j).rsplit("/", 1)[-1] for j in skel.GetJointsAttr().Get()]
    axis = Gf.Vec3d(0.0, forward, 0.0)
    posed_world = list(world)
    for i in range(len(rest)):
        p = topo.GetParent(i)
        local = rest[i]
        w = local * posed_world[p] if p >= 0 else local
        turn = POSE.get(names[i])
        if turn is not None:
            o = w.ExtractTranslation()
            spin = Gf.Matrix4d().SetTranslate(-o) * Gf.Matrix4d().SetRotate(Gf.Rotation(axis, turn * forward))
            w = w * spin * Gf.Matrix4d().SetTranslate(o)
        posed_world[i] = w
    return [posed_world[i] * (posed_world[p].GetInverse() if p >= 0 else Gf.Matrix4d(1.0)) for i, p in
            enumerate(topo.GetParent(k) for k in range(len(rest)))]  # fmt: skip


def bake(src, posed_path):
    """Write the posed, baked, skeleton-free character to posed_path; return its forward sign."""
    stage = Usd.Stage.Open(str(src))
    skel_prim = next(p for p in stage.Traverse() if p.IsA(UsdSkel.Skeleton))
    skel = UsdSkel.Skeleton(skel_prim)
    forward = facing(skel, joint_world(skel, list(skel.GetRestTransformsAttr().Get()))[1])
    anim = UsdSkel.Animation.Define(stage, skel_prim.GetPath().AppendChild("StandPose"))
    anim.CreateJointsAttr(skel.GetJointsAttr().Get())
    locals_ = posed_locals(skel, forward)
    trans, rots, scales = UsdSkel.DecomposeTransforms(locals_)
    anim.CreateTranslationsAttr(trans)
    anim.CreateRotationsAttr(rots)
    anim.CreateScalesAttr(scales)
    UsdSkel.BindingAPI.Apply(skel_prim).CreateAnimationSourceRel().SetTargets([anim.GetPath()])
    root = next(p for p in stage.Traverse() if p.IsA(UsdSkel.Root))
    if not UsdSkel.BakeSkinning(UsdSkel.Root(root), Gf.Interval(0.0, 0.0)):
        raise SystemExit("build_worker: BakeSkinning failed")
    layer = stage.Flatten()
    for prim_spec in list(iter_specs(layer.pseudoRoot)):
        unskin(prim_spec)
    layer.Export(str(posed_path))
    return forward


def iter_specs(spec):
    """Every prim spec under `spec`, depth first."""
    for child in spec.nameChildren:
        yield child
        yield from iter_specs(child)


def unskin(spec):
    """Make a baked prim static: points and transforms at their baked sample, skeleton bindings and prims gone."""
    if spec.typeName in ("Skeleton", "SkelAnimation"):
        spec.active = False
        return
    if spec.typeName == "SkelRoot":
        spec.typeName = "Xform"
    for name in list(spec.properties.keys()):
        prop = spec.properties[name]
        if name.startswith(("skel:", "primvars:skel:")):
            spec.RemoveProperty(prop)
            continue
        if isinstance(prop, Sdf.AttributeSpec) and prop.layer.ListTimeSamplesForPath(prop.path):
            t = prop.layer.ListTimeSamplesForPath(prop.path)[0]
            prop.default = prop.layer.QueryTimeSample(prop.path, t)
            for s in list(prop.layer.ListTimeSamplesForPath(prop.path)):
                prop.layer.EraseTimeSample(prop.path, s)
    schemas = spec.GetInfo("apiSchemas") if spec.HasInfo("apiSchemas") else None
    if schemas is not None:
        kept = [s for s in schemas.GetAddedOrExplicitItems() if not s.startswith("SkelBindingAPI")]
        spec.SetInfo("apiSchemas", Sdf.TokenListOp.CreateExplicit(kept))


def main():
    """Pose, bake and wrap; print the sizes and digests."""
    ap = argparse.ArgumentParser()
    ap.add_argument("character")
    ap.add_argument("out")
    ap.add_argument("--radius", type=float, default=0.28, help="collider capsule radius [m]")
    args = ap.parse_args()
    out = Path(args.out).resolve()
    posed = out.with_name(out.stem + "_posed.usda")
    forward = bake(Path(args.character).resolve(), posed)
    stage = Usd.Stage.CreateNew(str(out))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Worker").GetPrim()
    stage.SetDefaultPrim(root)
    visual = UsdGeom.Xform.Define(stage, "/Worker/Visual")
    visual.AddRotateZOp().Set(-90.0 * forward)
    src = Sdf.Layer.FindOrOpen(str(posed))
    visual.GetPrim().GetReferences().AddReference(f"./{posed.name}", src.defaultPrim and f"/{src.defaultPrim}")
    box = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeWorldBound(visual.GetPrim())
    rng = box.ComputeAlignedRange()
    lo, hi = rng.GetMin(), rng.GetMax()
    height = hi[2] - lo[2]
    cap = UsdGeom.Capsule.Define(stage, "/Worker/Collider")
    cap.CreateAxisAttr("Z")
    cap.CreateRadiusAttr(args.radius)
    cap.CreateHeightAttr(height - 2 * args.radius)
    cap.MakeInvisible()
    cap.AddTranslateOp().Set(Gf.Vec3d((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2] + height / 2))
    UsdPhysics.CollisionAPI.Apply(cap.GetPrim())
    stage.GetRootLayer().Save()
    print(
        f"worker: facing {'+y' if forward > 0 else '-y'} in the file, posed x {lo[0]:.3f}..{hi[0]:.3f} "
        f"y {lo[1]:.3f}..{hi[1]:.3f} z {lo[2]:.3f}..{hi[2]:.3f}; capsule radius {args.radius} at "
        f"({(lo[0] + hi[0]) / 2:.3f}, {(lo[1] + hi[1]) / 2:.3f})"
    )
    for p in (posed, out):
        print(f"sha256 {p.name} {hashlib.sha256(p.read_bytes()).hexdigest()}")


if __name__ == "__main__":
    main()
