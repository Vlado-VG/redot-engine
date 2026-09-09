// Spaces: lifecycle, activation semantics (measured, not just flagged),
// solver parameter plumbing, multi-space isolation, and space destruction
// with live contents.

using System.Collections;

namespace PhysxTestProject.Tests;

internal static class SpaceTests {
    public static void Register(SuiteBuilder s) {
        s.Add("PHYSX-SPACE-001", "create/destroy space; RID validity lifecycle", CreateDestroy);
        s.Add("PHYSX-SPACE-002", "inactive space does not simulate (body frozen mid-air)", InactiveSpaceFrozen);
        s.Add("PHYSX-SPACE-003", "reactivating a space resumes simulation", ReactivateResumes);
        s.Add("PHYSX-SPACE-004", "SOLVER_ITERATIONS round-trip (documented storage param)", SolverIterationsRoundtrip);
        s.Add("PHYSX-SPACE-005", "unsupported space params are dropped without corrupting the space", UnsupportedParamsNoCorrupt);
        s.Add("PHYSX-SPACE-006", "direct states of two spaces are distinct objects", DistinctDirectStates);
        s.Add("PHYSX-SPACE-007", "space A objects invisible to queries in space B", QueryIsolation);
        s.Add("PHYSX-SPACE-008", "equivalent bodies in two spaces evolve identically", ParallelEvolution);
        s.Add("PHYSX-SPACE-009", "destroying a space with live objects leaves other spaces intact", DestroySpaceWithObjects);
        s.Add("PHYSX-SPACE-010", "repeated create/destroy cycles (40x) stay stable", RepeatedCreateDestroy);
        s.Add("PHYSX-SPACE-011", "empty space steps 120 frames without issue", EmptySpaceSteps);
        s.Add("PHYSX-SPACE-012", "moving a body between spaces transfers its query presence", BodyMovedBetweenSpaces);
        s.Add("PHYSX-SPACE-013", "freed space returns null direct state", FreedSpaceDirectState);
    }

    static IEnumerator CreateDestroy() {
        var sp = PhysicsServer3D.SpaceCreate();
        PhysicsServer3D.SpaceSetActive(sp, true);
        Assert.Expect(sp.IsValid, "created space RID valid");
        PhysicsServer3D.FreeRid(sp);
        yield return Wait.Frame();
        using var w = new PhysxWorld();
        var b = w.MakeBody(w.Box(0.4f), new Vector3(0, 2, 0));
        yield return Wait.Frames(30);
        Assert.Expect(w.Pos(b).Origin.Y < 1.9f, "simulation healthy after space free");
    }

    static IEnumerator InactiveSpaceFrozen() {
        var sp = PhysicsServer3D.SpaceCreate();
        var shape = PhysicsServer3D.BoxShapeCreate();
        PhysicsServer3D.ShapeSetData(shape, new Vector3(0.5f, 0.5f, 0.5f));
        var b = PhysicsServer3D.BodyCreate();
        PhysicsServer3D.BodySetSpace(b, sp);
        PhysicsServer3D.BodyAddShape(b, shape, Transform3D.Identity);
        PhysicsServer3D.BodySetState(b, PhysicsServer3D.BodyState.Transform, new Transform3D(Basis.Identity, new Vector3(0, 6, 0)));
        yield return Wait.Frames(60);
        Assert.Expect((PhysicsServer3D.BodyGetState(b, PhysicsServer3D.BodyState.Transform).AsTransform3D().Origin - new Vector3(0, 6, 0)).Length() < 1e-4f,
            "body in never-activated space does not fall");
        PhysicsServer3D.FreeRid(b);
        PhysicsServer3D.FreeRid(shape);
        PhysicsServer3D.FreeRid(sp);
    }

    static IEnumerator ReactivateResumes() {
        var sp = PhysicsServer3D.SpaceCreate();
        var shape = PhysicsServer3D.BoxShapeCreate();
        PhysicsServer3D.ShapeSetData(shape, new Vector3(0.5f, 0.5f, 0.5f));
        var b = PhysicsServer3D.BodyCreate();
        PhysicsServer3D.BodySetSpace(b, sp);
        PhysicsServer3D.BodyAddShape(b, shape, Transform3D.Identity);
        PhysicsServer3D.BodySetState(b, PhysicsServer3D.BodyState.Transform, new Transform3D(Basis.Identity, new Vector3(0, 6, 0)));
        yield return Wait.Frames(30);
        float frozenY = PhysicsServer3D.BodyGetState(b, PhysicsServer3D.BodyState.Transform).AsTransform3D().Origin.Y;
        PhysicsServer3D.SpaceSetActive(sp, true);
        yield return Wait.Frames(30);
        float thawedY = PhysicsServer3D.BodyGetState(b, PhysicsServer3D.BodyState.Transform).AsTransform3D().Origin.Y;
        Assert.ExpectNear(frozenY, 6f, 1e-3f, "body frozen while space inactive");
        Assert.Expect(thawedY < frozenY - 0.5f, $"body resumes falling after activation (fell {frozenY - thawedY:F2} m)");
        PhysicsServer3D.FreeRid(b);
        PhysicsServer3D.FreeRid(shape);
        PhysicsServer3D.FreeRid(sp);
    }

    static IEnumerator SolverIterationsRoundtrip() {
        using var w = new PhysxWorld(false);
        PhysicsServer3D.SpaceSetParam(w.Space, PhysicsServer3D.SpaceParameter.SolverIterations, 12f);
        Assert.ExpectNear(PhysicsServer3D.SpaceGetParam(w.Space, PhysicsServer3D.SpaceParameter.SolverIterations), 12f, 1e-4f,
            "solver iterations round-trip");
        PhysicsServer3D.SpaceSetParam(w.Space, PhysicsServer3D.SpaceParameter.SolverIterations, 4f);
        yield return Wait.Frame();
    }

    static IEnumerator UnsupportedParamsNoCorrupt() {
        using var w = new PhysxWorld();
        // These params are documented as ignored by the module; setting them must
        // not corrupt the space or the simulation.
        PhysicsServer3D.SpaceSetParam(w.Space, PhysicsServer3D.SpaceParameter.BodyTimeToSleep, 0.7f);
        PhysicsServer3D.SpaceSetParam(w.Space, PhysicsServer3D.SpaceParameter.ContactRecycleRadius, 0.1f);
        PhysicsServer3D.SpaceSetParam(w.Space, PhysicsServer3D.SpaceParameter.ContactMaxAllowedPenetration, 0.01f);
        var b = w.MakeBody(w.Box(0.4f), new Vector3(0, 2, 0));
        yield return Wait.UntilOrFail(() => w.Pos(b).Origin.Y < 0.5f, 300, "body falls and rests on floor");
        Assert.ExpectNear(w.Pos(b).Origin.Y, 0.4f, 0.15f, "body rests at expected height after unsupported-param writes");
    }

    static IEnumerator DistinctDirectStates() {
        using var a = new PhysxWorld(false);
        using var b = new PhysxWorld(false);
        var sa = PhysicsServer3D.SpaceGetDirectState(a.Space);
        var sb = PhysicsServer3D.SpaceGetDirectState(b.Space);
        Assert.Expect(sa != null && sb != null, "both direct states exist");
        Assert.Expect(sa != sb, "direct states are distinct objects");
        yield return Wait.Frame();
    }

    static IEnumerator QueryIsolation() {
        using var a = new PhysxWorld(); // has floor at y=0
        using var b = new PhysxWorld(false); // empty
        var body = a.MakeBody(a.Box(0.4f), new Vector3(0, 3, 0));
        yield return Wait.Frames(10);
        var hitA = a.Ray(new Vector3(0, 5, 0), new Vector3(0, -5, 0));
        var hitB = b.Ray(new Vector3(0, 5, 0), new Vector3(0, -5, 0));
        Assert.Expect(hitA.Count > 0, "ray in space A hits A's floor");
        Assert.Expect(hitB.Count == 0, "ray in space B does not see A's floor or bodies");
        yield return Wait.Frames(10);
        Assert.Expect(a.Pos(body).Origin.Y < 2.9f, "A's body keeps falling in A");
    }

    static IEnumerator ParallelEvolution() {
        using var a = new PhysxWorld(false);
        using var b = new PhysxWorld(false);
        var ba = a.MakeBody(a.Box(0.5f), new Vector3(0, 20, 0));
        var bb = b.MakeBody(b.Box(0.5f), new Vector3(0, 20, 0));
        for (int i = 0; i < 5; i++) {
            yield return Wait.Frames(30);
            Assert.ExpectNear(a.Pos(ba).Origin.Y, b.Pos(bb).Origin.Y, 1e-5f,
                $"independent spaces evolve identically (checkpoint {i})");
        }
    }

    static IEnumerator DestroySpaceWithObjects() {
        using var keeper = new PhysxWorld();
        var victim = PhysicsServer3D.SpaceCreate();
        PhysicsServer3D.SpaceSetActive(victim, true);
        var shape = PhysicsServer3D.BoxShapeCreate();
        PhysicsServer3D.ShapeSetData(shape, new Vector3(0.5f, 0.5f, 0.5f));
        for (int i = 0; i < 4; i++) {
            var b = PhysicsServer3D.BodyCreate();
            PhysicsServer3D.BodySetSpace(b, victim);
            PhysicsServer3D.BodyAddShape(b, shape, Transform3D.Identity);
            PhysicsServer3D.BodySetState(b, PhysicsServer3D.BodyState.Transform,
                new Transform3D(Basis.Identity, new Vector3(i * 2f, 5f, 0)));
        }
        yield return Wait.Frames(20);
        var canary = keeper.MakeBody(keeper.Box(0.4f), new Vector3(0, 3, 0));
        float canaryY0 = keeper.Pos(canary).Origin.Y;
        PhysicsServer3D.FreeRid(victim); // free while it still contains live bodies
        PhysicsServer3D.FreeRid(shape); // shared shape freed too
        // The canary falls 2.6 m to touchdown (~44 frames); a fixed 40-frame
        // wait sampled it mid-air at 0.82 and looked like a phantom. Wait for
        // actual touchdown + settling instead.
        yield return Wait.UntilOrFail(
            () => Mathf.Abs(keeper.Vel(canary).Y) < 0.2f && Mathf.Abs(keeper.Pos(canary).Origin.Y - 0.4f) < 0.25f, 300,
            "canary falls onto keeper floor");
        Assert.Expect(keeper.Pos(canary).Origin.Y < canaryY0 - 0.5f, "canary in other space still simulates after space destruction");
        Assert.ExpectNear(keeper.Pos(canary).Origin.Y, 0.4f, 0.15f, "canary rests correctly after space destruction");
    }

    static IEnumerator RepeatedCreateDestroy() {
        using var w = new PhysxWorld();
        for (int i = 0; i < 40; i++) {
            var sp = PhysicsServer3D.SpaceCreate();
            PhysicsServer3D.SpaceSetActive(sp, true);
            var b = PhysicsServer3D.BodyCreate();
            PhysicsServer3D.BodySetSpace(b, sp);
            var sh = w.Box(0.3f);
            PhysicsServer3D.BodyAddShape(b, sh, Transform3D.Identity);
            PhysicsServer3D.FreeRid(b);
            PhysicsServer3D.FreeRid(sh);
            PhysicsServer3D.FreeRid(sp);
            if (i % 10 == 9) {
                yield return Wait.Frames(5);
                var probe = w.Ray(new Vector3(0, 5, 0), new Vector3(0, -5, 0));
                Assert.Expect(probe.Count > 0, $"floor still hit after {i + 1} space create/destroy cycles");
            }
        }
        yield return Wait.Frame();
    }

    static IEnumerator EmptySpaceSteps() {
        var sp = PhysicsServer3D.SpaceCreate();
        PhysicsServer3D.SpaceSetActive(sp, true);
        yield return Wait.Frames(120);
        PhysicsServer3D.FreeRid(sp);
        using var w = new PhysxWorld();
        var b = w.MakeBody(w.Box(0.4f), new Vector3(0, 2, 0));
        yield return Wait.UntilOrFail(() => w.Pos(b).Origin.Y < 0.6f, 200, "simulation alive after empty-space stepping");
    }

    static IEnumerator BodyMovedBetweenSpaces() {
        using var a = new PhysxWorld();
        using var b = new PhysxWorld(false);
        var body = a.MakeBody(a.Box(0.5f), new Vector3(0, 0, 0), 1f, 1, 0xFFFFFFFF, PhysicsServer3D.BodyMode.Static);
        yield return Wait.Frames(5);
        bool SeesA() => a.Ray(new Vector3(0, 5, 0), new Vector3(0, -5, 0)) is { Count: > 0 } h && h["rid"].AsRid() == body;
        Assert.Expect(SeesA(), "body visible in space A");
        PhysicsServer3D.BodySetSpace(body, b.Space);
        yield return Wait.Frames(5);
        Assert.Expect(!SeesA(), "body gone from space A after move");
        var hitB = b.Ray(new Vector3(0, 5, 0), new Vector3(0, -5, 0));
        Assert.Expect(hitB.Count > 0 && hitB["rid"].AsRid() == body, "body visible in space B after move");
    }

    static IEnumerator FreedSpaceDirectState() {
        var sp = PhysicsServer3D.SpaceCreate();
        PhysicsServer3D.SpaceSetActive(sp, true);
        PhysicsServer3D.FreeRid(sp);
        yield return Wait.Frame();
        var st = PhysicsServer3D.SpaceGetDirectState(sp);
        Assert.Expect(st == null, "freed space yields null direct state");
    }
}
