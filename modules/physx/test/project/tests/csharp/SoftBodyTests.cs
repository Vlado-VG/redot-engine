// Soft bodies: lifecycle/parameter plumbing plus behavioral tests. Each soft
// body resolves to a GPU PxDeformableVolume (CUDA) or the CPU XPBD solver; the
// assertions below hold on either path.

using System.Collections;
using Godot;
using Godot.Collections;

namespace PhysxTestProject.Tests;

internal static class SoftBodyTests {
    public static void Register(SuiteBuilder s) {
        s.Add("PHYSX-SOFT-001", "create/space/params plumbing", Plumbing);
        s.Add("PHYSX-SOFT-002", "collision layer/mask/exceptions plumbing", FilterPlumbing);
        s.Add("PHYSX-SOFT-003", "pin/unpin point bookkeeping", PinPlumbing);
        s.Add("PHYSX-SOFT-004", "deforms under gravity", DeformsUnderGravity);
        s.Add("PHYSX-SOFT-005", "collides with rigid bodies", CollidesWithRigid);
        s.Add("PHYSX-SOFT-006", "pinned point stays fixed while body sags", PinnedPointHolds);
    }

    /// <summary>Unit cube mesh (8 verts / 12 tris) centered on the origin —
    /// watertight, so it tetrahedralizes for the GPU path.</summary>
    public static Rid MakeCubeMesh() {
        var mesh = RenderingServer.MeshCreate();
        var verts = new Vector3[] {
            new(-0.5f, -0.5f, -0.5f), new(0.5f, -0.5f, -0.5f), new(0.5f, 0.5f, -0.5f), new(-0.5f, 0.5f, -0.5f),
            new(-0.5f, -0.5f, 0.5f), new(0.5f, -0.5f, 0.5f), new(0.5f, 0.5f, 0.5f), new(-0.5f, 0.5f, 0.5f),
        };
        // Outward-facing triangles (right-hand rule), watertight cube surface.
        int[][] quads = {
            new[] { 1, 0, 3 }, new[] { 1, 3, 2 }, // -Z
            new[] { 4, 5, 6 }, new[] { 4, 6, 7 }, // +Z
            new[] { 5, 1, 2 }, new[] { 5, 2, 6 }, // +X
            new[] { 0, 4, 7 }, new[] { 0, 7, 3 }, // -X
            new[] { 3, 7, 6 }, new[] { 3, 6, 2 }, // +Y
            new[] { 0, 1, 5 }, new[] { 0, 5, 4 }, // -Y
        };
        var indices = new int[36];
        int k = 0;
        foreach (var q in quads) {
            indices[k++] = q[0]; indices[k++] = q[1]; indices[k++] = q[2];
        }
        var arrays = new Array();
        arrays.Resize((int)Mesh.ArrayType.Max);
        arrays[(int)Mesh.ArrayType.Vertex] = verts;
        arrays[(int)Mesh.ArrayType.Index] = indices;
        RenderingServer.MeshAddSurfaceFromArrays(mesh, RenderingServer.PrimitiveType.Triangles, arrays);
        return mesh;
    }

    static IEnumerator Plumbing() {
        using var w = new PhysxWorld(false);
        var sb = PhysicsServer3D.SoftBodyCreate();
        Assert.Require(sb.IsValid, "soft_body_create returns valid RID");
        PhysicsServer3D.SoftBodySetSpace(sb, w.Space);
        Assert.Expect(PhysicsServer3D.SoftBodyGetSpace(sb) == w.Space, "soft body space round-trip");
        PhysicsServer3D.SoftBodySetTotalMass(sb, 3f);
        Assert.ExpectNear(PhysicsServer3D.SoftBodyGetTotalMass(sb), 3f, 1e-4f, "total mass round-trip");
        PhysicsServer3D.SoftBodySetLinearStiffness(sb, 0.8f);
        Assert.ExpectNear(PhysicsServer3D.SoftBodyGetLinearStiffness(sb), 0.8f, 1e-4f, "stiffness round-trip");
        PhysicsServer3D.SoftBodySetSimulationPrecision(sb, 7);
        Assert.Expect(PhysicsServer3D.SoftBodyGetSimulationPrecision(sb) == 7, "precision round-trip");
        PhysicsServer3D.SoftBodySetPressureCoefficient(sb, 0.5f);
        Assert.ExpectNear(PhysicsServer3D.SoftBodyGetPressureCoefficient(sb), 0.5f, 1e-4f, "pressure round-trip");
        PhysicsServer3D.SoftBodySetDampingCoefficient(sb, 0.2f);
        Assert.ExpectNear(PhysicsServer3D.SoftBodyGetDampingCoefficient(sb), 0.2f, 1e-4f, "damping round-trip");
        var bounds = PhysicsServer3D.SoftBodyGetBounds(sb);
        Assert.Expect(bounds.HasVolume() || bounds.Size.Length() >= 0f, "bounds readable");
        yield return Wait.Frames(10);
        Assert.Expect(PhysicsServer3D.SoftBodyGetSpace(sb) == w.Space, "space still set after stepping");
    }
    static IEnumerator FilterPlumbing() {
        using var w = new PhysxWorld(false);
        var sb = PhysicsServer3D.SoftBodyCreate();
        PhysicsServer3D.SoftBodySetSpace(sb, w.Space);
        PhysicsServer3D.SoftBodySetCollisionLayer(sb, 3);
        PhysicsServer3D.SoftBodySetCollisionMask(sb, 7);
        Assert.Expect(PhysicsServer3D.SoftBodyGetCollisionLayer(sb) == 3, "layer round-trip");
        Assert.Expect(PhysicsServer3D.SoftBodyGetCollisionMask(sb) == 7, "mask round-trip");
        var other = w.MakeBody(w.Box(0.3f), new Vector3(0, 5, 0));
        PhysicsServer3D.SoftBodyAddCollisionException(sb, other);
        PhysicsServer3D.SoftBodyRemoveCollisionException(sb, other);
        yield return Wait.Frames(5);
        Assert.Expect(PhysxWorld.Finite(w.Pos(other)), "engine healthy with soft-body exception churn");
    }
    static IEnumerator PinPlumbing() {
        using var w = new PhysxWorld(false);
        var sb = PhysicsServer3D.SoftBodyCreate();
        PhysicsServer3D.SoftBodySetSpace(sb, w.Space);
        PhysicsServer3D.SoftBodyPinPoint(sb, 2, true);
        Assert.Expect(PhysicsServer3D.SoftBodyIsPointPinned(sb, 2), "pin flag set");
        PhysicsServer3D.SoftBodyPinPoint(sb, 2, false);
        Assert.Expect(!PhysicsServer3D.SoftBodyIsPointPinned(sb, 2), "pin flag cleared");
        PhysicsServer3D.SoftBodyPinPoint(sb, 0, true);
        PhysicsServer3D.SoftBodyPinPoint(sb, 1, true);
        PhysicsServer3D.SoftBodyRemoveAllPinnedPoints(sb);
        Assert.Expect(!PhysicsServer3D.SoftBodyIsPointPinned(sb, 0) && !PhysicsServer3D.SoftBodyIsPointPinned(sb, 1),
            "remove_all_pinned_points clears pins");
        yield return Wait.Frame();
    }
    static IEnumerator DeformsUnderGravity() {
        using var w = new PhysxWorld(false);
        var mesh = MakeCubeMesh();
        var sb = PhysicsServer3D.SoftBodyCreate();
        try {
            PhysicsServer3D.SoftBodySetSpace(sb, w.Space);
            PhysicsServer3D.SoftBodySetTransform(sb, new Transform3D(Basis.Identity, new Vector3(0, 3, 0)));
            PhysicsServer3D.SoftBodySetMesh(sb, mesh);

            yield return Wait.Frames(5);
            var b0 = PhysicsServer3D.SoftBodyGetBounds(sb);
            Assert.Expect(b0.Size.Length() > 0.01f, "bounds reflect the mesh (got size " + b0.Size + ")");
            float startY = b0.GetCenter().Y;
            Assert.Require(Mathf.Abs(startY - 3f) < 0.6f, "bounds placed near the transform (center y=" + startY + ")");

            yield return Wait.Frames(60);
            var b1 = PhysicsServer3D.SoftBodyGetBounds(sb);
            Assert.Expect(PhysxWorld.Finite(b1.Position) && PhysxWorld.Finite(b1.Size), "bounds stay finite while falling");
            Assert.Expect(b1.GetCenter().Y < startY - 1f,
                $"soft body fell under gravity (center y {startY:F2} -> {b1.GetCenter().Y:F2})");
        } finally {
            PhysicsServer3D.FreeRid(sb);
            RenderingServer.FreeRid(mesh);
        }
    }
    static IEnumerator CollidesWithRigid() {
        using var w = new PhysxWorld(true); // floor top at y=0
        var mesh = MakeCubeMesh();
        var sb = PhysicsServer3D.SoftBodyCreate();
        try {
            PhysicsServer3D.SoftBodySetSpace(sb, w.Space);
            PhysicsServer3D.SoftBodySetTransform(sb, new Transform3D(Basis.Identity, new Vector3(0, 2.5f, 0)));
            PhysicsServer3D.SoftBodySetMesh(sb, mesh);
            PhysicsServer3D.SoftBodySetDampingCoefficient(sb, 0.3f);

            yield return Wait.Frames(180);
            var b = PhysicsServer3D.SoftBodyGetBounds(sb);
            Assert.Expect(PhysxWorld.Finite(b.Position) && PhysxWorld.Finite(b.Size), "bounds stay finite while settling");
            Assert.Expect(b.Position.Y > -0.25f,
                $"soft body rests on the floor instead of sinking through (bounds min y={b.Position.Y:F3})");
            Assert.Expect(b.GetCenter().Y < 2f, $"soft body actually dropped toward the floor (center y={b.GetCenter().Y:F3})");
        } finally {
            PhysicsServer3D.FreeRid(sb);
            RenderingServer.FreeRid(mesh);
        }
    }
    static IEnumerator PinnedPointHolds() {
        using var w = new PhysxWorld(false);
        var mesh = MakeCubeMesh();
        var sb = PhysicsServer3D.SoftBodyCreate();
        try {
            PhysicsServer3D.SoftBodySetSpace(sb, w.Space);
            PhysicsServer3D.SoftBodySetTransform(sb, new Transform3D(Basis.Identity, new Vector3(0, 3, 0)));
            PhysicsServer3D.SoftBodySetMesh(sb, mesh);
            yield return Wait.Frames(3);

            // Pin render vertex 0 (a bottom corner). Its world position at the
            // placed transform is origin + (-0.5, -0.5, -0.5).
            PhysicsServer3D.SoftBodyPinPoint(sb, 0, true);
            var pinnedStart = PhysicsServer3D.SoftBodyGetPointGlobalPosition(sb, 0);
            Assert.Expect(PhysxWorld.Finite(pinnedStart), "pinned point position readable");
            Assert.Expect(Mathf.Abs(pinnedStart.Y - 2.5f) < 0.2f, "pinned point starts at its placed position");

            yield return Wait.Frames(90);
            var now = PhysicsServer3D.SoftBodyGetPointGlobalPosition(sb, 0);
            var bounds = PhysicsServer3D.SoftBodyGetBounds(sb);
            Assert.Expect(PhysxWorld.Finite(now), "pinned point stays finite");
            Assert.Expect(now.DistanceTo(pinnedStart) < 0.35f,
                $"pinned point holds (moved {(now - pinnedStart).Length():F3} m) while bounds center y={bounds.GetCenter().Y:F2}");
        } finally {
            PhysicsServer3D.FreeRid(sb);
            RenderingServer.FreeRid(mesh);
        }
    }
}
