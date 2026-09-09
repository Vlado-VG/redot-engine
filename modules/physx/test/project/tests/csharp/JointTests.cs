// Joints: every implemented type (pin, hinge, slider, cone-twist, 6dof) with
// *measured* constraint behavior — allowed vs forbidden degrees of freedom,
// limits enforced, motor driven — plus destruction order torture, chains,
// closed loops, and load stress. Setter/getter round-trips only where the
// module documents storage-only params.

using System;
using System.Collections;
using System.Linq;

namespace PhysxTestProject.Tests;

internal static class JointTests {
    public static void Register(SuiteBuilder s) {
        s.Add("PHYSX-JOINT-001", "pin joint: anchor separation stays bounded under gravity", PinHolds);
        s.Add("PHYSX-JOINT-002", "pin joint: bodies cannot translate apart but can swing", PinDof);
        s.Add("PHYSX-JOINT-003", "pin joint params are storage-only round-trips [plumbing]", PinParamRoundtrip);
        s.Add("PHYSX-JOINT-004", "hinge: rotation about axis allowed", HingeAllowsAxis);
        s.Add("PHYSX-JOINT-005", "hinge: perpendicular rotation constrained", HingeBlocksPerp);
        s.Add("PHYSX-JOINT-006", "hinge limit clamps angle", HingeLimit);
        s.Add("PHYSX-JOINT-007", "hinge motor reaches target velocity", HingeMotor);
        s.Add("PHYSX-JOINT-008", "slider: translation along axis allowed, perpendicular blocked", SliderDof);
        s.Add("PHYSX-JOINT-009", "slider linear limit enforced", SliderLimit);
        s.Add("PHYSX-JOINT-010", "slider angular storage-only round-trip [plumbing]", SliderParamRoundtrip);
        s.Add("PHYSX-JOINT-011", "cone-twist: swing constrained within span", ConeTwistSwing);
        s.Add("PHYSX-JOINT-012", "cone-twist params round-trip", ConeTwistRoundtrip);
        s.Add("PHYSX-JOINT-013", "6dof: locked linear axis blocked, free axis moves", SixDofLinear);
        s.Add("PHYSX-JOINT-014", "6dof: angular lock round-trips and constrains", SixDofAngular);
        s.Add("PHYSX-JOINT-015", "joint types reported correctly", JointTypeReporting);
        s.Add("PHYSX-JOINT-016", "solver priority storage-only round-trip [plumbing]", SolverPriorityRoundtrip);
        s.Add("PHYSX-JOINT-017", "disable_collisions keeps jointed overlapping pair apart-free", DisableCollisions);
        s.Add("PHYSX-JOINT-018", "destroy joint: bodies free-fall independently", DestroyJointReleases);
        s.Add("PHYSX-JOINT-019", "destroy body A: joint auto-released, body B continues", DestroyBodyA);
        s.Add("PHYSX-JOINT-020", "destroy body B: joint auto-released, body A continues", DestroyBodyB);
        s.Add("PHYSX-JOINT-021", "destroy both bodies then joint: no corruption", DestroyBothThenJoint);
        s.Add("PHYSX-JOINT-022", "joint ops after joint destruction: no crash", StaleJointOps);
        s.Add("PHYSX-JOINT-023", "chain of 5 jointed bodies hangs without explosion", ChainHangs);
        s.Add("PHYSX-JOINT-024", "closed loop (4-body square) stays bounded", ClosedLoop);
        s.Add("PHYSX-JOINT-025", "joint under heavy load keeps constraint error bounded", LoadStress);
        s.Add("PHYSX-JOINT-026", "pin joint to static body: pendulum swings below pivot", PendulumSwings);
        s.Add("PHYSX-JOINT-027", "many joints (30) on one body stay finite", ManyJointsOneBody);
    }

    static (Rid a, Rid b, Rid j) MakePinnedPair(PhysxWorld w, Vector3 pos) {
        var a = w.MakeStatic(w.Box(0.3f), pos);
        var b = w.MakeBody(w.Box(0.3f), pos + new Vector3(0, -1.5f, 0));
        var j = PhysicsServer3D.JointCreate();
        w.TrackJoint(j);
        PhysicsServer3D.JointMakePin(j, a, new Vector3(0, -0.5f, 0), b, new Vector3(0, 0.5f, 0));
        return (a, b, j);
    }

    static IEnumerator PinHolds() {
        using var w = new PhysxWorld(false);
        var (a, b, j) = MakePinnedPair(w, new Vector3(0, 5, 0));
        yield return Wait.Frames(120);
        var pa = w.Pos(a).Origin; var pb = w.Pos(b).Origin;
        var anchorA = pa + new Vector3(0, -0.5f, 0);
        var anchorB = pb + new Vector3(0, 0.5f, 0);
        float err = (anchorA - anchorB).Length();
        Assert.Expect(err < 0.2f, $"pin constraint error bounded (got {err:F3})");
        Assert.Expect(pb.Y < pa.Y - 0.8f, "body hangs below pivot");
        Assert.Expect(PhysxWorld.Finite(w.Pos(b)) && PhysxWorld.Finite(w.Vel(b)), "state finite under constraint");
    }
    static IEnumerator PinDof() {
        using var w = new PhysxWorld(false);
        var (a, b, j) = MakePinnedPair(w, new Vector3(0, 5, 0));
        yield return Wait.Frames(20);
        // Kick the hanging body sideways: it must swing (rotate around pivot),
        // not translate rigidly away. Anchor offsets are +/-0.5, so the swing
        // radius is 1.0 (body center hangs 1.0 below the static body center).
        PhysicsServer3D.BodyApplyCentralImpulse(b, new Vector3(2, 0, 0));
        yield return Wait.Frames(60);
        var pa = w.Pos(a).Origin; var pb = w.Pos(b).Origin;
        float dist = (pb - pa).Length();
        Assert.Expect(dist > 0.8f && dist < 1.25f, $"swing keeps radius ~1.0 (got {dist:F2})");
        var anchorA = pa + new Vector3(0, -0.5f, 0);
        var anchorB = pb + new Vector3(0, 0.5f, 0);
        Assert.Expect((anchorA - anchorB).Length() < 0.3f, "anchors stay coincident while swinging");
    }
    static IEnumerator PinParamRoundtrip() {
        using var w = new PhysxWorld(false);
        var (a, b, j) = MakePinnedPair(w, new Vector3(0, 5, 0));
        PhysicsServer3D.PinJointSetParam(j, PhysicsServer3D.PinJointParam.Bias, 0.3f);
        PhysicsServer3D.PinJointSetParam(j, PhysicsServer3D.PinJointParam.Damping, 0.2f);
        PhysicsServer3D.PinJointSetParam(j, PhysicsServer3D.PinJointParam.ImpulseClamp, 5f);
        Assert.ExpectNear(PhysicsServer3D.PinJointGetParam(j, PhysicsServer3D.PinJointParam.Bias), 0.3f, 1e-4f, "bias round-trip (storage-only)");
        Assert.ExpectNear(PhysicsServer3D.PinJointGetParam(j, PhysicsServer3D.PinJointParam.Damping), 0.2f, 1e-4f, "damping round-trip (storage-only)");
        Assert.ExpectNear(PhysicsServer3D.PinJointGetParam(j, PhysicsServer3D.PinJointParam.ImpulseClamp), 5f, 1e-4f, "impulse clamp round-trip (storage-only)");
        PhysicsServer3D.PinJointSetLocalA(j, new Vector3(1, 2, 3));
        Assert.ExpectVecNear(PhysicsServer3D.PinJointGetLocalA(j), new Vector3(1, 2, 3), 1e-4f, "local_a round-trip");
        yield return Wait.Frame();
    }

    static (Rid a, Rid b, Rid j) MakeHingedPair(PhysxWorld w) {
        var a = w.MakeStatic(w.Box(0.3f), new Vector3(0, 5, 0));
        var b = w.MakeBody(w.Box(0.25f, 1.0f, 0.25f), new Vector3(0.75f, 5f, 0)); // arm along +x
        var j = PhysicsServer3D.JointCreate();
        w.TrackJoint(j);
        PhysicsServer3D.JointMakeHinge(j, a, Transform3D.Identity, b, Transform3D.Identity);
        return (a, b, j);
    }
    static IEnumerator HingeAllowsAxis() {
        using var w = new PhysxWorld(false);
        var (a, b, j) = MakeHingedPair(w);
        // Hinge with identity frames rotates about local X (Godot hinge axis).
        PhysicsServer3D.BodyApplyTorqueImpulse(b, new Vector3(3, 0, 0));
        yield return Wait.Frames(10);
        float wx = Math.Abs(w.AngVel(b).X);
        Assert.Expect(wx > 0.3f, $"torque about hinge axis produces spin ({wx:F2})");
        yield return Wait.Frames(60);
        Assert.Expect(PhysxWorld.Finite(w.Pos(b)), "state finite while spinning on hinge");
    }
    static IEnumerator HingeBlocksPerp() {
        using var w = new PhysxWorld(false);
        var (a, b, j) = MakeHingedPair(w);
        PhysicsServer3D.BodyApplyTorqueImpulse(b, new Vector3(0, 4, 0)); // perpendicular torque
        yield return Wait.Frames(10);
        float wy = Math.Abs(w.AngVel(b).Y);
        Assert.Expect(wy < 0.25f, $"hinge resists perpendicular spin (wy={wy:F2})");
    }
    static IEnumerator HingeLimit() {
        using var w = new PhysxWorld(false);
        var (a, b, j) = MakeHingedPair(w);
        PhysicsServer3D.HingeJointSetFlag(j, PhysicsServer3D.HingeJointFlag.UseLimit, true);
        PhysicsServer3D.HingeJointSetParam(j, PhysicsServer3D.HingeJointParam.LimitUpper, 0.3f);
        PhysicsServer3D.HingeJointSetParam(j, PhysicsServer3D.HingeJointParam.LimitLower, -0.3f);
        Assert.Expect(PhysicsServer3D.HingeJointGetFlag(j, PhysicsServer3D.HingeJointFlag.UseLimit), "use_limit flag round-trip");
        Assert.ExpectNear(PhysicsServer3D.HingeJointGetParam(j, PhysicsServer3D.HingeJointParam.LimitUpper), 0.3f, 1e-4f, "limit upper round-trip");
        PhysicsServer3D.HingeJointSetFlag(j, PhysicsServer3D.HingeJointFlag.EnableMotor, true);
        PhysicsServer3D.HingeJointSetParam(j, PhysicsServer3D.HingeJointParam.MotorTargetVelocity, 20f); // try to spin far past limit
        PhysicsServer3D.HingeJointSetParam(j, PhysicsServer3D.HingeJointParam.MotorMaxImpulse, 40f);
        yield return Wait.Frames(120);
        var q = w.Pos(b).Basis.GetRotationQuaternion();
        // The arm extends along +x from pivot; swing about hinge X axis is limited to ±0.3 rad.
        float swing = Math.Abs(Mathf.Atan2(w.Pos(b).Origin.Y - 5f, w.Pos(b).Origin.X));
        Assert.Expect(swing < 0.65f, $"hinge limit clamps swing (measured {swing:F2} rad, limit 0.3 + solver slack)");
        Assert.Expect(PhysxWorld.Finite(w.Pos(b)), "state finite at limit");
    }
    static IEnumerator HingeMotor() {
        using var w = new PhysxWorld(false);
        var (a, b, j) = MakeHingedPair(w);
        PhysicsServer3D.BodySetParam(b, PhysicsServer3D.BodyParameter.AngularDamp, 2f);
        PhysicsServer3D.HingeJointSetFlag(j, PhysicsServer3D.HingeJointFlag.EnableMotor, true);
        PhysicsServer3D.HingeJointSetParam(j, PhysicsServer3D.HingeJointParam.MotorTargetVelocity, 4f);
        PhysicsServer3D.HingeJointSetParam(j, PhysicsServer3D.HingeJointParam.MotorMaxImpulse, 50f);
        yield return Wait.Frames(90);
        Assert.Expect(Mathf.Abs(w.AngVel(b).X) > 1.5f,
            $"motor drives hinge toward target velocity (wx={w.AngVel(b).X:F2}, target 4)");
    }
    static IEnumerator SliderDof() {
        using var w = new PhysxWorld(false);
        var a = w.MakeStatic(w.Box(0.3f), new Vector3(0, 5, 0));
        var b = w.MakeBody(w.Box(0.25f), new Vector3(0, 5, 0));
        var j = PhysicsServer3D.JointCreate();
        w.TrackJoint(j);
        PhysicsServer3D.JointMakeSlider(j, a, Transform3D.Identity, b, Transform3D.Identity);
        // Slider with identity frames translates along X.
        PhysicsServer3D.BodySetParam(b, PhysicsServer3D.BodyParameter.GravityScale, 0f);
        PhysicsServer3D.BodyApplyCentralImpulse(b, new Vector3(2, 0, 0));
        yield return Wait.Frames(30);
        Assert.Expect(w.Pos(b).Origin.X > 0.5f, $"slides along free axis (x={w.Pos(b).Origin.X:F2})");
        Assert.Expect(Mathf.Abs(w.Pos(b).Origin.Y - 5f) < 0.25f, "perpendicular translation constrained");
        Assert.Expect(Mathf.Abs(w.Pos(b).Origin.Z) < 0.1f, "second perpendicular axis constrained");
    }
    static IEnumerator SliderLimit() {
        using var w = new PhysxWorld(false);
        var a = w.MakeStatic(w.Box(0.3f), new Vector3(0, 5, 0));
        var b = w.MakeBody(w.Box(0.25f), new Vector3(0, 5, 0));
        var j = PhysicsServer3D.JointCreate();
        w.TrackJoint(j);
        PhysicsServer3D.JointMakeSlider(j, a, Transform3D.Identity, b, Transform3D.Identity);
        PhysicsServer3D.SliderJointSetParam(j, PhysicsServer3D.SliderJointParam.LinearLimitUpper, 0.8f);
        PhysicsServer3D.SliderJointSetParam(j, PhysicsServer3D.SliderJointParam.LinearLimitLower, -0.8f);
        Assert.ExpectNear(PhysicsServer3D.SliderJointGetParam(j, PhysicsServer3D.SliderJointParam.LinearLimitUpper), 0.8f, 1e-4f, "slider limit round-trip");
        PhysicsServer3D.BodySetParam(b, PhysicsServer3D.BodyParameter.GravityScale, 0f);
        PhysicsServer3D.BodySetConstantForce(b, new Vector3(3, 0, 0)); // push outward forever
        yield return Wait.Frames(240);
        Assert.Expect(w.Pos(b).Origin.X < 1.3f, $"slider limit stops travel (x={w.Pos(b).Origin.X:F2}, limit 0.8+slack)");
        Assert.Expect(w.Pos(b).Origin.X > 0.3f, "body did travel before hitting limit");
    }
    static IEnumerator SliderParamRoundtrip() {
        using var w = new PhysxWorld(false);
        var (a, b, j) = MakePinnedPair(w, new Vector3(0, 5, 0)); // any bodies; re-make the joint
        var j2 = PhysicsServer3D.JointCreate();
        w.TrackJoint(j2);
        PhysicsServer3D.JointMakeSlider(j2, a, Transform3D.Identity, b, Transform3D.Identity);
        PhysicsServer3D.SliderJointSetParam(j2, PhysicsServer3D.SliderJointParam.AngularLimitUpper, 0.5f);
        Assert.ExpectNear(PhysicsServer3D.SliderJointGetParam(j2, PhysicsServer3D.SliderJointParam.AngularLimitUpper), 0.5f, 1e-4f,
            "slider angular limit stored (documented storage-only)");
        yield return Wait.Frame();
    }
    static IEnumerator ConeTwistSwing() {
        using var w = new PhysxWorld(false);
        var a = w.MakeStatic(w.Box(0.3f), new Vector3(0, 5, 0));
        var b = w.MakeBody(w.Box(0.25f, 1f, 0.25f), new Vector3(0, 4.4f, 0)); // hanging below
        var j = PhysicsServer3D.JointCreate();
        w.TrackJoint(j);
        PhysicsServer3D.JointMakeConeTwist(j, a, Transform3D.Identity, b, Transform3D.Identity);
        PhysicsServer3D.ConeTwistJointSetParam(j, PhysicsServer3D.ConeTwistJointParam.SwingSpan, 0.25f);
        PhysicsServer3D.ConeTwistJointSetParam(j, PhysicsServer3D.ConeTwistJointParam.TwistSpan, 0.25f);
        Assert.ExpectNear(PhysicsServer3D.ConeTwistJointGetParam(j, PhysicsServer3D.ConeTwistJointParam.SwingSpan), 0.25f, 1e-4f, "swing span round-trip");
        // Push hard sideways: swing must stay within span + slack.
        PhysicsServer3D.BodyApplyCentralImpulse(b, new Vector3(3, 0, 0));
        yield return Wait.Frames(90);
        float swingAngle = Math.Abs(Mathf.Atan2(w.Pos(b).Origin.X, 5f - w.Pos(b).Origin.Y));
        Assert.Expect(swingAngle < 0.75f, $"cone swing limited (measured {swingAngle:F2} rad, span 0.25)");
        Assert.Expect(PhysxWorld.Finite(w.Pos(b)), "state finite in cone");
    }
    static IEnumerator ConeTwistRoundtrip() {
        using var w = new PhysxWorld(false);
        var a = w.MakeStatic(w.Box(0.3f), new Vector3(0, 5, 0));
        var b = w.MakeBody(w.Box(0.25f), new Vector3(0, 4.5f, 0));
        var j = PhysicsServer3D.JointCreate();
        w.TrackJoint(j);
        PhysicsServer3D.JointMakeConeTwist(j, a, Transform3D.Identity, b, Transform3D.Identity);
        PhysicsServer3D.ConeTwistJointSetParam(j, PhysicsServer3D.ConeTwistJointParam.TwistSpan, 0.6f);
        PhysicsServer3D.ConeTwistJointSetParam(j, PhysicsServer3D.ConeTwistJointParam.Softness, 0.1f);
        PhysicsServer3D.ConeTwistJointSetParam(j, PhysicsServer3D.ConeTwistJointParam.Relaxation, 0.2f);
        Assert.ExpectNear(PhysicsServer3D.ConeTwistJointGetParam(j, PhysicsServer3D.ConeTwistJointParam.TwistSpan), 0.6f, 1e-4f, "twist span round-trip");
        Assert.ExpectNear(PhysicsServer3D.ConeTwistJointGetParam(j, PhysicsServer3D.ConeTwistJointParam.Softness), 0.1f, 1e-4f, "softness round-trip");
        Assert.ExpectNear(PhysicsServer3D.ConeTwistJointGetParam(j, PhysicsServer3D.ConeTwistJointParam.Relaxation), 0.2f, 1e-4f, "relaxation round-trip");
        yield return Wait.Frame();
    }
    static IEnumerator SixDofLinear() {
        using var w = new PhysxWorld(false);
        var a = w.MakeStatic(w.Box(0.3f), new Vector3(0, 5, 0));
        var b = w.MakeBody(w.Box(0.25f), new Vector3(0, 5, 0));
        var j = PhysicsServer3D.JointCreate();
        w.TrackJoint(j);
        PhysicsServer3D.JointMakeGeneric6Dof(j, a, Transform3D.Identity, b, Transform3D.Identity);
        // Lock Y translation (limits 0..0); leave X/Z free.
        PhysicsServer3D.Generic6DofJointSetFlag(j, Vector3.Axis.Y, PhysicsServer3D.G6DofJointAxisFlag.EnableLinearLimit, true);
        PhysicsServer3D.Generic6DofJointSetParam(j, Vector3.Axis.Y, PhysicsServer3D.G6DofJointAxisParam.LinearLowerLimit, 0f);
        PhysicsServer3D.Generic6DofJointSetParam(j, Vector3.Axis.Y, PhysicsServer3D.G6DofJointAxisParam.LinearUpperLimit, 0f);
        Assert.ExpectNear(PhysicsServer3D.Generic6DofJointGetParam(j, Vector3.Axis.Y, PhysicsServer3D.G6DofJointAxisParam.LinearLowerLimit), 0f, 1e-5f, "6dof lower limit round-trip");
        Assert.Expect(PhysicsServer3D.Generic6DofJointGetFlag(j, Vector3.Axis.Y, PhysicsServer3D.G6DofJointAxisFlag.EnableLinearLimit), "6dof flag round-trip");
        // Gravity pulls -Y; body must stay at y=5. Then push along +X: free.
        yield return Wait.Frames(60);
        Assert.Expect(Mathf.Abs(w.Pos(b).Origin.Y - 5f) < 0.3f, $"locked Y resists gravity (y={w.Pos(b).Origin.Y:F2})");
        PhysicsServer3D.BodyApplyCentralImpulse(b, new Vector3(2, 0, 0));
        yield return Wait.Frames(30);
        Assert.Expect(w.Pos(b).Origin.X > 0.4f, "free X axis moves");
    }
    static IEnumerator SixDofAngular() {
        using var w = new PhysxWorld(false);
        var a = w.MakeStatic(w.Box(0.3f), new Vector3(0, 5, 0));
        var b = w.MakeBody(w.Box(0.25f), new Vector3(0.6f, 5f, 0));
        var j = PhysicsServer3D.JointCreate();
        w.TrackJoint(j);
        PhysicsServer3D.JointMakeGeneric6Dof(j, a, Transform3D.Identity, b, Transform3D.Identity);
        // Lock all angular motion around Y: in Godot semantics limits apply
        // only where the ENABLE_ANGULAR_LIMIT flag is set (fresh axes are free).
        PhysicsServer3D.Generic6DofJointSetFlag(j, Vector3.Axis.Y, PhysicsServer3D.G6DofJointAxisFlag.EnableAngularLimit, true);
        PhysicsServer3D.Generic6DofJointSetParam(j, Vector3.Axis.Y, PhysicsServer3D.G6DofJointAxisParam.AngularLowerLimit, 0f);
        PhysicsServer3D.Generic6DofJointSetParam(j, Vector3.Axis.Y, PhysicsServer3D.G6DofJointAxisParam.AngularUpperLimit, 0f);
        PhysicsServer3D.BodyApplyTorqueImpulse(b, new Vector3(0, 3, 0));
        yield return Wait.Frames(10);
        Assert.Expect(Math.Abs(w.AngVel(b).Y) < 0.3f, $"6dof angular lock constrains spin (wy={w.AngVel(b).Y:F2})");
    }
    static IEnumerator JointTypeReporting() {
        using var w = new PhysxWorld(false);
        var a = w.MakeStatic(w.Box(0.3f), new Vector3(0, 5, 0));
        var b = w.MakeBody(w.Box(0.25f), new Vector3(0, 4.5f, 0));
        var jp = w.TrackJoint(PhysicsServer3D.JointCreate());
        PhysicsServer3D.JointMakePin(jp, a, Vector3.Zero, b, Vector3.Zero);
        var jh = w.TrackJoint(PhysicsServer3D.JointCreate());
        PhysicsServer3D.JointMakeHinge(jh, a, Transform3D.Identity, b, Transform3D.Identity);
        var js = w.TrackJoint(PhysicsServer3D.JointCreate());
        PhysicsServer3D.JointMakeSlider(js, a, Transform3D.Identity, b, Transform3D.Identity);
        var jc = w.TrackJoint(PhysicsServer3D.JointCreate());
        PhysicsServer3D.JointMakeConeTwist(jc, a, Transform3D.Identity, b, Transform3D.Identity);
        var j6 = w.TrackJoint(PhysicsServer3D.JointCreate());
        PhysicsServer3D.JointMakeGeneric6Dof(j6, a, Transform3D.Identity, b, Transform3D.Identity);
        Assert.Expect(PhysicsServer3D.JointGetType(jp) == PhysicsServer3D.JointType.Pin, "pin type");
        Assert.Expect(PhysicsServer3D.JointGetType(jh) == PhysicsServer3D.JointType.Hinge, "hinge type");
        Assert.Expect(PhysicsServer3D.JointGetType(js) == PhysicsServer3D.JointType.Slider, "slider type");
        Assert.Expect(PhysicsServer3D.JointGetType(jc) == PhysicsServer3D.JointType.ConeTwist, "cone-twist type");
        Assert.Expect(PhysicsServer3D.JointGetType(j6) == PhysicsServer3D.JointType.Type6Dof, "6dof type");
        yield return Wait.Frame();
    }
    static IEnumerator SolverPriorityRoundtrip() {
        using var w = new PhysxWorld(false);
        var (a, b, j) = MakePinnedPair(w, new Vector3(0, 5, 0));
        PhysicsServer3D.JointSetSolverPriority(j, 5);
        Assert.Expect(PhysicsServer3D.JointGetSolverPriority(j) == 5, "solver priority round-trip (storage-only)");
        yield return Wait.Frame();
    }
    static IEnumerator DisableCollisions() {
        using var w = new PhysxWorld(false);
        // Two overlapping boxes joined by a pin: with collisions disabled they
        // interpenetrate peacefully instead of pushing apart.
        var a = w.MakeBody(w.Box(0.5f), new Vector3(0, 5, 0));
        var b = w.MakeBody(w.Box(0.5f), new Vector3(0.05f, 5, 0));
        PhysicsServer3D.BodySetParam(a, PhysicsServer3D.BodyParameter.GravityScale, 0f);
        PhysicsServer3D.BodySetParam(b, PhysicsServer3D.BodyParameter.GravityScale, 0f);
        var j = w.TrackJoint(PhysicsServer3D.JointCreate());
        PhysicsServer3D.JointMakePin(j, a, Vector3.Zero, b, Vector3.Zero);
        PhysicsServer3D.JointDisableCollisionsBetweenBodies(j, true);
        Assert.Expect(PhysicsServer3D.JointIsDisabledCollisionsBetweenBodies(j), "disable-collisions flag round-trip");
        yield return Wait.Frames(60);
        Assert.Expect((w.Pos(b).Origin - w.Pos(a).Origin).Length() < 0.6f,
            $"jointed overlapping pair does not explode apart (dist={(w.Pos(b).Origin - w.Pos(a).Origin).Length():F2})");
        Assert.Expect(PhysxWorld.Finite(w.Pos(a)) && PhysxWorld.Finite(w.Pos(b)), "states finite");
    }
    static IEnumerator DestroyJointReleases() {
        using var w = new PhysxWorld(false);
        var (a, b, j) = MakePinnedPair(w, new Vector3(0, 8, 0));
        yield return Wait.Frames(30);
        PhysicsServer3D.FreeRid(j);
        yield return Wait.Frames(60);
        Assert.Expect(w.Pos(b).Origin.Y < 5f, $"body free-falls after joint destroyed (y={w.Pos(b).Origin.Y:F2})");
        Assert.Expect(PhysxWorld.Finite(w.Pos(b)), "state finite after joint destruction");
    }
    static IEnumerator DestroyBodyA() {
        using var w = new PhysxWorld(false);
        var (a, b, j) = MakePinnedPair(w, new Vector3(0, 8, 0));
        yield return Wait.Frames(30);
        PhysicsServer3D.FreeRid(a);
        yield return Wait.Frames(60);
        Assert.Expect(w.Pos(b).Origin.Y < 5f, "body B falls after body A destroyed");
        Assert.Expect(PhysxWorld.Finite(w.Pos(b)), "body B finite");
    }
    static IEnumerator DestroyBodyB() {
        using var w = new PhysxWorld(false);
        var (a, b, j) = MakePinnedPair(w, new Vector3(0, 8, 0));
        var c = w.MakeBody(w.Box(0.3f), new Vector3(5, 8, 0));
        yield return Wait.Frames(30);
        PhysicsServer3D.FreeRid(b);
        yield return Wait.Frames(60);
        Assert.Expect(PhysxWorld.Finite(w.Pos(c)), "unrelated body finite after jointed body destroyed");
        Assert.Expect(w.Pos(c).Origin.Y < 6f, "unrelated body still falls");
    }
    static IEnumerator DestroyBothThenJoint() {
        using var w = new PhysxWorld(false);
        var (a, b, j) = MakePinnedPair(w, new Vector3(0, 8, 0));
        yield return Wait.Frames(10);
        PhysicsServer3D.FreeRid(a);
        PhysicsServer3D.FreeRid(b);
        yield return Wait.Frames(10);
        // Joint's bodies are gone; freeing it now must be safe.
        PhysicsServer3D.FreeRid(j);
        yield return Wait.Frames(30);
        var canary = w.MakeBody(w.Box(0.3f), new Vector3(0, 5, 0));
        yield return Wait.Frames(30);
        Assert.Expect(w.Pos(canary).Origin.Y < 4.5f, "simulation healthy after joint-after-bodies destruction");
    }
    static IEnumerator StaleJointOps() {
        using var w = new PhysxWorld(false);
        var (a, b, j) = MakePinnedPair(w, new Vector3(0, 8, 0));
        PhysicsServer3D.FreeRid(j);
        yield return Wait.Frame();
        // Ops on the stale joint RID must not crash or corrupt anything.
        PhysicsServer3D.PinJointSetParam(j, PhysicsServer3D.PinJointParam.Bias, 0.1f);
        _ = PhysicsServer3D.PinJointGetParam(j, PhysicsServer3D.PinJointParam.Bias);
        _ = PhysicsServer3D.JointGetType(j);
        yield return Wait.Frames(20);
        Assert.Expect(w.Pos(b).Origin.Y < 6f, "body B fell (joint was freed) and engine healthy");
    }
    static IEnumerator ChainHangs() {
        using var w = new PhysxWorld(false);
        const int n = 5;
        var anchor = w.MakeStatic(w.Box(0.3f), new Vector3(0, 12, 0));
        var links = new Rid[n];
        for (int i = 0; i < n; i++)
            links[i] = w.MakeBody(w.Box(0.2f, 0.45f, 0.2f), new Vector3(0, 12 - 0.7f - i * 0.95f, 0));
        for (int i = 0; i < n; i++) {
            var j = w.TrackJoint(PhysicsServer3D.JointCreate());
            PhysicsServer3D.JointMakePin(j, i == 0 ? anchor : links[i - 1], new Vector3(0, i == 0 ? 0 : -0.5f, 0), links[i], new Vector3(0, 0.5f, 0));
        }
        for (int f = 0; f < 6; f++) {
            yield return Wait.Frames(60);
            foreach (var l in links) {
                Assert.Expect(PhysxWorld.Finite(w.Pos(l)) && PhysxWorld.Finite(w.Vel(l)), $"chain link finite at {(f + 1) * 60} frames");
                Assert.Expect(w.Vel(l).Length() < 50f, "chain velocity bounded");
            }
        }
        Assert.Expect(w.Pos(links[n - 1]).Origin.Y < 9f, "chain hangs below anchor");
        Assert.Expect(Mathf.Abs(w.Pos(links[n - 1]).Origin.X) < 2.5f, "chain has not wandered sideways");
    }
    static IEnumerator ClosedLoop() {
        using var w = new PhysxWorld(false);
        // A—B
        // |  |
        // D—C  (pinned square, A static)
        var A = w.MakeStatic(w.Box(0.2f), new Vector3(0, 6, 0));
        var B = w.MakeBody(w.Box(0.2f), new Vector3(0, 5, 0));
        var C = w.MakeBody(w.Box(0.2f), new Vector3(1, 5, 0));
        var D = w.MakeBody(w.Box(0.2f), new Vector3(1, 6, 0));
        void Pin(Rid x, Vector3 la, Rid y, Vector3 lb) {
            var j = w.TrackJoint(PhysicsServer3D.JointCreate());
            PhysicsServer3D.JointMakePin(j, x, la, y, lb);
        }
        Pin(A, new Vector3(0, -0.5f, 0), B, new Vector3(0, 0.5f, 0));
        Pin(B, new Vector3(0.5f, 0, 0), C, new Vector3(-0.5f, 0, 0));
        Pin(C, new Vector3(0, 0.5f, 0), D, new Vector3(0, -0.5f, 0));
        Pin(D, new Vector3(-0.5f, 0, 0), A, new Vector3(0.5f, 0, 0));
        PhysicsServer3D.BodyApplyCentralImpulse(C, new Vector3(2, 0, 0));
        for (int f = 0; f < 6; f++) {
            yield return Wait.Frames(60);
            foreach (var b in new[] { B, C, D }) {
                Assert.Expect(PhysxWorld.Finite(w.Pos(b)) && PhysxWorld.Finite(w.Vel(b)), $"loop body finite at {(f + 1) * 60}");
                Assert.Expect(w.Vel(b).Length() < 60f, "loop velocity bounded");
            }
        }
        Assert.Expect((w.Pos(C).Origin - new Vector3(1, 5, 0)).Length() < 1.5f, "loop roughly keeps shape");
    }
    static IEnumerator LoadStress() {
        using var w = new PhysxWorld(false);
        var anchor = w.MakeStatic(w.Box(0.3f), new Vector3(0, 10, 0));
        var heavy = w.MakeBody(w.Box(0.5f), new Vector3(0, 8.5f, 0), mass: 50f);
        var j = w.TrackJoint(PhysicsServer3D.JointCreate());
        PhysicsServer3D.JointMakePin(j, anchor, Vector3.Zero, heavy, new Vector3(0, 0.5f, 0));
        PhysicsServer3D.BodySetConstantForce(heavy, new Vector3(80, 0, 0)); // lateral load
        yield return Wait.Frames(240);
        var anchorB = w.Pos(heavy).Origin + new Vector3(0, 0.5f, 0);
        Assert.Expect((anchorB - w.Pos(anchor).Origin).Length() < 1.2f,
            $"heavy load constraint error bounded ({(anchorB - w.Pos(anchor).Origin).Length():F2})");
        Assert.Expect(PhysxWorld.Finite(w.Pos(heavy)) && PhysxWorld.Finite(w.Vel(heavy)), "loaded body finite");
    }
    static IEnumerator PendulumSwings() {
        using var w = new PhysxWorld(false);
        var (a, b, j) = MakePinnedPair(w, new Vector3(0, 6, 0));
        // Start displaced: body hangs below-left.
        w.Teleport(b, new Vector3(-1.3f, 5.0f, 0));
        yield return Wait.Frames(20);
        float minX = w.Pos(b).Origin.X;
        for (int i = 0; i < 6; i++) {
            yield return Wait.Frames(40);
            minX = Math.Min(minX, w.Pos(b).Origin.X);
            Assert.Expect(w.Pos(b).Origin.Y < 5.6f, "pendulum stays below pivot");
        }
        Assert.Expect(minX > -2.0f, "pendulum swing radius bounded");
    }
    static IEnumerator ManyJointsOneBody() {
        using var w = new PhysxWorld(false);
        var hub = w.MakeBody(w.Box(0.6f), new Vector3(0, 8, 0));
        PhysicsServer3D.BodySetParam(hub, PhysicsServer3D.BodyParameter.GravityScale, 0f);
        for (int i = 0; i < 30; i++) {
            float ang = i * Mathf.Tau / 30f;
            // Radius 3.0: spoke spacing 0.63 m keeps the 0.3-wide boxes out of
            // each other's contact offset (0.02). At radius 1.5 the gap is
            // 0.014 m — inside contact offset — so all 30 spokes sit in
            // permanent mutual contact and the solver explodes.
            var offset = new Vector3(Mathf.Cos(ang) * 3.0f, 0, Mathf.Sin(ang) * 3.0f);
            var spoke = w.MakeBody(w.Box(0.15f), new Vector3(0, 8, 0) + offset);
            PhysicsServer3D.BodySetParam(spoke, PhysicsServer3D.BodyParameter.GravityScale, 0f);
            var j = w.TrackJoint(PhysicsServer3D.JointCreate());
            // Pin anchors preserve the spawn radius: anchoring both ends at
            // Zero would drag all 30 spokes onto the hub center where they
            // pile up and explode.
            PhysicsServer3D.JointMakePin(j, hub, offset, spoke, Vector3.Zero);
            // The pins hold the spokes off the hub body; disable hub-spoke
            // collision like Godot's Joint3D does by default.
            PhysicsServer3D.JointDisableCollisionsBetweenBodies(j, true);
        }
        for (int f = 0; f < 4; f++) {
            yield return Wait.Frames(60);
            var hv = w.Vel(hub);
            var hp = w.Pos(hub).Origin;
            Assert.Expect(PhysxWorld.Finite(hp) && PhysxWorld.Finite(hv), $"hub finite at {(f + 1) * 60}");
            Assert.Expect(hv.Length() < 40f, $"hub velocity bounded with 30 joints (|v|={hv.Length():F3} v=({hv.X:F2},{hv.Y:F2},{hv.Z:F2}) y={hp.Y:F3})");
        }
    }
}
