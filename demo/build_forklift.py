"""Wrap NVIDIA's Isaac Sim forklift (`Isaac/Props/Forklift/forklift.usd`, CC-BY-4.0) into the scenario's rigid body.

The prop is a visual: its body and fork meshes carry mesh collision, it has no rigid body, and its forks point along
-y. This script writes a layer that references it, turned so the vehicle frame is the scenario's (x forward, toward
the forks; z 0 on the floor), with the prop's mesh collision removed and one box collider over the body in its place:

    /Forklift            Xform, RigidBodyAPI, MassAPI (mass_kg), the default prim
    /Forklift/Visual     the prop's /Root, rotated 90 deg about z, render-only
    /Forklift/Collider   Cube over the body (counterweight to mast, not the forks), from `clearance_m` up, CollisionAPI,
                         invisible

Reversing, the counterweight leads, so the box is the face that meets the robot. The prop is only referenced (by a
path relative to the output), never copied or changed. Run it in the project env (pxr):

    python demo/build_forklift.py <forklift.usd> <out.usda> [--clearance 0.08] [--mass 3000]

It prints the forklift's length and width (with the forks), the collider and the output's sha256, for the scenario.
"""

import argparse
import hashlib
import os
from pathlib import Path

from pxr import Gf, Usd, UsdGeom, UsdPhysics

BODY = "S_ForkliftBody"
"""The prop's body mesh under /Root; the collider covers its footprint."""


def main():
    """Write the wrapper, print the sizes and its sha256."""
    ap = argparse.ArgumentParser()
    ap.add_argument("prop")
    ap.add_argument("out")
    ap.add_argument("--clearance", type=float, default=0.08, help="collider bottom above the floor [m]")
    ap.add_argument("--mass", type=float, default=3000.0)
    args = ap.parse_args()
    stage = Usd.Stage.CreateNew(args.out)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Forklift").GetPrim()
    stage.SetDefaultPrim(root)
    UsdPhysics.RigidBodyAPI.Apply(root)
    UsdPhysics.MassAPI.Apply(root).CreateMassAttr(args.mass)
    visual = UsdGeom.Xform.Define(stage, "/Forklift/Visual")
    visual.AddRotateZOp().Set(90.0)
    rel = os.path.relpath(os.path.abspath(args.prop), os.path.dirname(os.path.abspath(args.out)))
    visual.GetPrim().GetReferences().AddReference(rel, "/Root")
    for prim in Usd.PrimRange(visual.GetPrim()):
        for api in (UsdPhysics.MeshCollisionAPI, UsdPhysics.CollisionAPI):
            if prim.HasAPI(api):
                prim.RemoveAPI(api)
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    whole = bbox.ComputeWorldBound(visual.GetPrim()).ComputeAlignedRange()
    body = bbox.ComputeWorldBound(stage.GetPrimAtPath(f"/Forklift/Visual/{BODY}")).ComputeAlignedRange()
    lo, hi = body.GetMin(), body.GetMax()
    hz = hi[2] - args.clearance
    col = UsdGeom.Cube.Define(stage, "/Forklift/Collider")
    col.CreateSizeAttr(1.0)
    col.MakeInvisible()
    col.AddTranslateOp().Set(Gf.Vec3d((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, args.clearance + hz / 2))
    col.AddScaleOp().Set(Gf.Vec3f(float(hi[0] - lo[0]), float(hi[1] - lo[1]), float(hz)))
    UsdPhysics.CollisionAPI.Apply(col.GetPrim())
    stage.GetRootLayer().Save()
    size = whole.GetMax() - whole.GetMin()
    print(
        f"forklift: length {size[0]:.3f} width {size[1]:.3f} height {size[2]:.3f} m with the forks, "
        f"x {whole.GetMin()[0]:.3f}..{whole.GetMax()[0]:.3f}; collider x {lo[0]:.3f}..{hi[0]:.3f} "
        f"y {lo[1]:.3f}..{hi[1]:.3f} z {args.clearance:.2f}..{hi[2]:.3f}"
    )
    print(f"sha256 {hashlib.sha256(Path(args.out).read_bytes()).hexdigest()}")


if __name__ == "__main__":
    main()
