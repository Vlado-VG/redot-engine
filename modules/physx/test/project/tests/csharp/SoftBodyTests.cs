// Soft bodies: the module currently ships an API skeleton (no
// PxDeformableVolume is created; nothing simulates). Per the test charter,
// API-only functionality must NOT be reported as fully implemented: lifecycle
// and parameter plumbing is verified, while behavioral tests SKIP with an
// explicit reason so the gap stays visible in every report.

using System.Collections;

namespace PhysxTestProject.Tests;

internal static class SoftBodyTests {
    public static void Register(SuiteBuilder s) {
        s.Add("PHYSX-SOFT-001", "create/space/params plumbing [API skeleton]", Plumbing);
        s.Add("PHYSX-SOFT-002", "collision layer/mask/exceptions plumbing [API skeleton]", FilterPlumbing);
        s.Add("PHYSX-SOFT-003", "pin/unpin point bookkeeping [API skeleton]", PinPlumbing);
        s.Add("PHYSX-SOFT-004", "deforms under gravity", DeformsUnderGravity);
        s.Add("PHYSX-SOFT-005", "collides with rigid bodies", CollidesWithRigid);
        s.Add("PHYSX-SOFT-006", "pinned point stays fixed while body sags", PinnedPointHolds);
    }

    static IEnumerator Plumbing() {
        using var w = new PhysxWorld(false);
        var sb = PhysicsServer3D.SoftBodyCreate();
        Assert.Require(sb.IsValid, "soft_body_create returns valid RID");
        PhysicsServer3D.SoftBodySetSpace(sb, w.Space);
        Assert.Expect(PhysicsServer3D.SoftBodyGetSpace(sb) == w.Space, "soft body space round-trip");
        PhysicsServer3D.SoftBodySetTotalMass(sb, 3f);
        Assert.ExpectNear(PhysicsServer3D.SoftBodyGetTotalMass(sb), 3f, 1e-4f, "total mass round-trip (not simulated)");
        PhysicsServer3D.SoftBodySetLinearStiffness(sb, 0.8f);
        Assert.ExpectNear(PhysicsServer3D.SoftBodyGetLinearStiffness(sb), 0.8f, 1e-4f, "stiffness round-trip (not simulated)");
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
        Assert.Skip("module ships an API skeleton only — no PxDeformableVolume is created, soft bodies do not simulate yet");
        yield return Wait.Frame();
    }
    static IEnumerator CollidesWithRigid() {
        Assert.Skip("soft-body simulation not implemented in the module (no deformable actor to collide with)");
        yield return Wait.Frame();
    }
    static IEnumerator PinnedPointHolds() {
        Assert.Skip("soft-body simulation not implemented in the module (pinning has no simulated effect)");
        yield return Wait.Frame();
    }
}
