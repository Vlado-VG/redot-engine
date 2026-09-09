// Motion (body_test_motion): sweeps used by character controllers — hit
// fractions, normals, collider ids, depenetration recovery, slopes, corners,
// and multi-collision results.

using System;
using System.Collections;

namespace PhysxTestProject.Tests;

internal static class MotionTests {
    public static void Register(SuiteBuilder s) {
        s.Add("PHYSX-MOVE-001", "sweep hits floor at sane fraction with up normal", SweepHitsFloor);
        s.Add("PHYSX-MOVE-002", "free sweep reports no collision", SweepFree);
        s.Add("PHYSX-MOVE-003", "sweep reports collider rid, safe/unsafe fractions, travel", SweepFields);
        s.Add("PHYSX-MOVE-004", "penetrating start recovers upward", SweepDepenetration);
        s.Add("PHYSX-MOVE-005", "initial penetration reports remainder collision", SweepInitialPenetration);
        s.Add("PHYSX-MOVE-006", "sweep into wall is blocked laterally", SweepWall);
        s.Add("PHYSX-MOVE-007", "sweep down a slope hits with slanted normal", SweepSlope);
        s.Add("PHYSX-MOVE-008", "corner sweep between two walls picks a deflection", SweepCorner);
        s.Add("PHYSX-MOVE-009", "sweep respects collision exceptions", SweepException);
        s.Add("PHYSX-MOVE-010", "kinematic body sweeps use its own shapes", SweepOwnShape);
    }

    static IEnumerator SweepHitsFloor() {
        using var w = new PhysxWorld();
        var body = w.MakeKinematic(w.Box(0.4f), new Vector3(3, 5, 0));
        var (hit, r) = w.TestMotion(body, new Transform3D(Basis.Identity, new Vector3(3, 2, 0)), new Vector3(0, -3, 0));
        Assert.Expect(hit, "sweep hits floor");
        Assert.Expect(r.GetCollisionSafeFraction() > 0.1f && r.GetCollisionSafeFraction() < 0.9f,
            $"safe fraction mid-range (got {r.GetCollisionSafeFraction():F3})");
        Assert.Expect(r.GetCollisionNormal(0).Y > 0.7f, "floor normal up");
        Assert.Expect(r.GetColliderRid(0) == w.FloorRid, "collider rid is floor");
        yield return Wait.Frame();
    }
    static IEnumerator SweepFree() {
        using var w = new PhysxWorld();
        var body = w.MakeKinematic(w.Box(0.4f), new Vector3(3, 5, 0));
        var (hit, _) = w.TestMotion(body, new Transform3D(Basis.Identity, new Vector3(3, 2, 0)), new Vector3(0, 0.5f, 0));
        Assert.Expect(!hit, "upward sweep in open air reports no collision");
        yield return Wait.Frame();
    }
    static IEnumerator SweepFields() {
        using var w = new PhysxWorld();
        var body = w.MakeKinematic(w.Box(0.4f), new Vector3(3, 5, 0));
        var (hit, r) = w.TestMotion(body, new Transform3D(Basis.Identity, new Vector3(3, 2, 0)), new Vector3(0, -3, 0));
        Assert.Require(hit, "hit expected");
        Assert.Expect(r.GetCollisionUnsafeFraction() >= r.GetCollisionSafeFraction(), "unsafe >= safe fraction");
        Assert.Expect(r.GetTravel().Y < 0f, "travel is downward");
        Assert.Expect(Math.Abs(r.GetTravel().Y + 3f * r.GetCollisionSafeFraction()) < 0.25f, "travel consistent with safe fraction");
        Assert.Expect(r.GetColliderShape(0) >= 0, "collider shape index valid");
        Assert.Expect(PhysxWorld.Finite(r.GetRemainder()), "remainder finite");
        yield return Wait.Frame();
    }
    static IEnumerator SweepDepenetration() {
        using var w = new PhysxWorld();
        var body = w.MakeKinematic(w.Box(0.4f), new Vector3(3, 5, 0));
        var (hit, r) = w.TestMotion(body, new Transform3D(Basis.Identity, new Vector3(3, 0.2f, 0)), new Vector3(0, 0.5f, 0));
        Assert.Expect(r.GetTravel().Y > 0.52f, $"recovery adds depenetration travel (travel.y={r.GetTravel().Y:F2})");
        Assert.Expect(hit, "penetrating motion reports collision");
        yield return Wait.Frame();
    }
    static IEnumerator SweepInitialPenetration() {
        using var w = new PhysxWorld();
        var body = w.MakeKinematic(w.Box(0.4f), new Vector3(3, 5, 0));
        // Start half-inside the floor; the remainder of the motion must still resolve.
        var (hit, r) = w.TestMotion(body, new Transform3D(Basis.Identity, new Vector3(3, 0.1f, 0)), new Vector3(0, -0.5f, 0));
        Assert.Expect(hit, "initially-penetrating sweep reports collision");
        Assert.Expect(PhysxWorld.Finite(r.GetTravel()), "travel finite after initial penetration");
        yield return Wait.Frame();
    }
    static IEnumerator SweepWall() {
        using var w = new PhysxWorld();
        w.MakeStatic(w.Box(0.5f, 4, 4), new Vector3(4, 2, 0));
        var body = w.MakeKinematic(w.Box(0.4f), new Vector3(0, 2, 0));
        var (hit, r) = w.TestMotion(body, new Transform3D(Basis.Identity, new Vector3(0, 2, 0)), new Vector3(4, 0, 0));
        Assert.Expect(hit, "sweep into wall blocked");
        Assert.Expect(r.GetCollisionSafeFraction() > 0.1f && r.GetCollisionSafeFraction() < 1f, "fraction sane");
        Assert.Expect(Mathf.Abs(r.GetCollisionNormal(0).X) > 0.7f, "wall normal horizontal");
        yield return Wait.Frame();
    }
    static IEnumerator SweepSlope() {
        using var w = new PhysxWorld(false);
        var rot = Basis.FromEuler(new Vector3(0, 0, Mathf.DegToRad(30)));
        w.MakeStatic(w.Box(10, 0.5f, 4), new Vector3(0, 1, 0), shapeXf: new Transform3D(rot, Vector3.Zero));
        var body = w.MakeKinematic(w.Box(0.3f), new Vector3(0, 5, 0));
        var (hit, r) = w.TestMotion(body, new Transform3D(Basis.Identity, new Vector3(-3, 5, 0)), new Vector3(0, -6, 0));
        Assert.Require(hit, "sweep reaches slope");
        var n = r.GetCollisionNormal(0);
        Assert.Expect(n.Y > 0.5f && n.Y < 0.99f, $"slope normal is slanted (got {n})");
        Assert.Expect(Mathf.Abs(n.X) > 0.15f, "slope normal has lateral component");
        yield return Wait.Frame();
    }
    static IEnumerator SweepCorner() {
        using var w = new PhysxWorld(false);
        w.MakeStatic(w.Box(0.25f, 3, 3), new Vector3(2, 1.5f, 0));  // wall ahead (+x)
        w.MakeStatic(w.Box(3, 3, 0.25f), new Vector3(1, 1.5f, 2));   // wall right (+z)
        var body = w.MakeKinematic(w.Box(0.4f), new Vector3(0, 1, 0));
        var (hit, r) = w.TestMotion(body, new Transform3D(Basis.Identity, new Vector3(0, 1, 0)), new Vector3(4, 0, 2));
        Assert.Expect(hit, "corner sweep collides");
        Assert.Expect(r.GetCollisionCount() >= 1, "at least one collision reported");
        Assert.Expect(PhysxWorld.Finite(r.GetTravel()), "corner travel finite");
        yield return Wait.Frame();
    }
    static IEnumerator SweepException() {
        using var w = new PhysxWorld();
        var body = w.MakeKinematic(w.Box(0.4f), new Vector3(3, 5, 0));
        PhysicsServer3D.BodyAddCollisionException(body, w.FloorRid);
        var (hit, _) = w.TestMotion(body, new Transform3D(Basis.Identity, new Vector3(3, 2, 0)), new Vector3(0, -3, 0));
        Assert.Expect(!hit, "sweep ignores excepted floor");
        PhysicsServer3D.BodyRemoveCollisionException(body, w.FloorRid);
        var (hit2, _) = w.TestMotion(body, new Transform3D(Basis.Identity, new Vector3(3, 2, 0)), new Vector3(0, -3, 0));
        Assert.Expect(hit2, "sweep hits floor after removing exception");
        yield return Wait.Frame();
    }
    static IEnumerator SweepOwnShape() {
        using var w = new PhysxWorld();
        var tall = w.MakeKinematic(w.Capsule(0.3f, 1.8f), new Vector3(6, 5, 0));
        var (hit, r) = w.TestMotion(tall, new Transform3D(Basis.Identity, new Vector3(6, 2, 0)), new Vector3(0, -3, 0));
        Assert.Expect(hit, "capsule sweep hits floor");
        Assert.Expect(r.GetCollisionSafeFraction() > 0.05f && r.GetCollisionSafeFraction() < 0.6f,
            "capsule (long shape) contacts earlier than a box would");
        yield return Wait.Frame();
    }
}
