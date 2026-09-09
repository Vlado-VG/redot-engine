/**
 * @file physx_joint_3d.h
 * @brief Joint wrapper — bridges Godot's PhysicsServer3D joint API to PhysX.
 *
 * PhysXJoint3D is a single class that wraps a PxJoint* (either PxRevoluteJoint,
 * PxPrismaticJoint, PxSphericalJoint, PxD6Joint, etc.) and dispatches parameter
 * access based on the Godot-side JointType. This avoids the boilerplate of
 * maintaining one class per joint type while keeping PhysX API usage idiomatic.
 *
 * Joint type mapping:
 *   JOINT_TYPE_PIN       → PxD6Joint (linear locked, angular free — pure ball joint)
 *   JOINT_TYPE_HINGE     → PxRevoluteJoint
 *   JOINT_TYPE_SLIDER    → PxPrismaticJoint
 *   JOINT_TYPE_CONE_TWIST→ PxD6Joint (swing/twist configured)
 *   JOINT_TYPE_6DOF      → PxD6Joint (fully configurable)
 *
 * Godot-specific parameters that have no PhysX equivalent:
 *   - PinJoint: PIN_JOINT_BIAS, PIN_JOINT_DAMPING, PIN_JOINT_IMPULSE_CLAMP
 *     Stored on the wrapper for round-trip get_param, but have no/negligible
 *     effect (PhysX solver stiffness is global, not per-joint).
 *   - HingeJoint: HINGE_JOINT_BIAS — no PhysX equivalent.
 *   - HingeJoint: HINGE_JOINT_LIMIT_BIAS — maps to PxJointLimitParameters::bounceThreshold.
 *   - HingeJoint: HINGE_JOINT_LIMIT_SOFTNESS — maps to PxJointLimitParameters::stiffness.
 *   - HingeJoint: HINGE_JOINT_LIMIT_RELAXATION — maps to PxJointLimitParameters::damping.
 *   - SliderJoint: linear/angular SLIDER_JOINT_* parameters map to
 *     PxJointLinearLimitPair/PxJointAngularLimitPair equivalents.
 *   - ConeTwistJoint: CONE_TWIST_JOINT_BIAS — maps to PxJointLimitCone::bounceThreshold.
 *   - ConeTwistJoint: CONE_TWIST_JOINT_SOFTNESS — maps to PxJointLimitCone::stiffness.
 *   - ConeTwistJoint: CONE_TWIST_JOINT_RELAXATION — maps to PxJointLimitCone::damping.
 */

#ifndef PHYSX_JOINT_3D_H
#define PHYSX_JOINT_3D_H

#include "core/variant/variant.h"
#include "core/math/transform_3d.h"
#include "core/math/vector3.h"
#include "core/templates/rid.h"
#include "servers/physics_3d/physics_server_3d.h"

#include "physx_rid_owner.h"

#include <PxPhysicsAPI.h>
#include "extensions/PxRevoluteJoint.h"
#include "extensions/PxPrismaticJoint.h"
#include "extensions/PxSphericalJoint.h"
#include "extensions/PxD6Joint.h"
#include "extensions/PxJointLimit.h"

namespace physx {
	class PxJoint;
	class PxRigidActor;
	class PxPhysics;
}

/**
 * @brief Godot-side metadata for a pin joint — stored on wrapper, no PhysX equivalent.
 */
struct PinJointParams {
	real_t bias = 0.0f;
	real_t damping = 0.0f;
	real_t impulse_clamp = 0.0f;
};

/**
 * @brief Godot-side metadata for a hinge joint — stored for round-trip access.
 */
struct HingeJointParams {
	real_t bias = 0.0f;
	real_t limit_upper = 0.0f;
	real_t limit_lower = 0.0f;
	real_t limit_bias = 0.0f;
	real_t limit_softness = 0.0f;
	real_t limit_relaxation = 0.0f;
	real_t motor_target_velocity = 0.0f;
	real_t motor_max_impulse = 0.0f;
	bool use_limit = false;
	bool enable_motor = false;
};

/**
 * @brief Godot-side metadata for a slider joint — stored for round-trip access.
 */
struct SliderJointParams {
	// Linear
	real_t linear_limit_upper = 0.0f;
	real_t linear_limit_lower = 0.0f;
	real_t linear_limit_softness = 0.0f;
	real_t linear_limit_restitution = 0.0f;
	real_t linear_limit_damping = 0.0f;
	real_t linear_motion_softness = 0.0f;
	real_t linear_motion_restitution = 0.0f;
	real_t linear_motion_damping = 0.0f;
	real_t linear_orthogonal_softness = 0.0f;
	real_t linear_orthogonal_restitution = 0.0f;
	real_t linear_orthogonal_damping = 0.0f;

	// Angular
	real_t angular_limit_upper = 0.0f;
	real_t angular_limit_lower = 0.0f;
	real_t angular_limit_softness = 0.0f;
	real_t angular_limit_restitution = 0.0f;
	real_t angular_limit_damping = 0.0f;
	real_t angular_motion_softness = 0.0f;
	real_t angular_motion_restitution = 0.0f;
	real_t angular_motion_damping = 0.0f;
	real_t angular_orthogonal_softness = 0.0f;
	real_t angular_orthogonal_restitution = 0.0f;
	real_t angular_orthogonal_damping = 0.0f;
};

/**
 * @brief Godot-side metadata for a cone/twist joint — stored for round-trip access.
 */
struct ConeTwistJointParams {
	real_t swing_span = 0.0f;
	real_t twist_span = 0.0f;
	real_t bias = 0.0f;
	real_t softness = 0.0f;
	real_t relaxation = 0.0f;
};

/**
 * @brief Godot-side metadata for a 6DOF joint — stored for round-trip access.
 */
struct G6DOFJointAxisParams {
	real_t linear_lower_limit = 0.0f;
	real_t linear_upper_limit = 0.0f;
	real_t linear_limit_softness = 0.0f;
	real_t linear_restitution = 0.0f;
	real_t linear_damping = 0.0f;
	real_t linear_motor_target_velocity = 0.0f;
	real_t linear_motor_force_limit = 0.0f;
	real_t linear_spring_stiffness = 0.0f;
	real_t linear_spring_damping = 0.0f;
	real_t linear_spring_equilibrium_point = 0.0f;

	real_t angular_lower_limit = 0.0f;
	real_t angular_upper_limit = 0.0f;
	real_t angular_limit_softness = 0.0f;
	real_t angular_restitution = 0.0f;
	real_t angular_damping = 0.0f;
	real_t angular_force_limit = 0.0f;
	real_t angular_erp = 0.0f;
	real_t angular_motor_target_velocity = 0.0f;
	real_t angular_motor_force_limit = 0.0f;
	real_t angular_spring_stiffness = 0.0f;
	real_t angular_spring_damping = 0.0f;
	real_t angular_spring_equilibrium_point = 0.0f;
};

class PhysXJoint3D : public PhysXRIDOwner {
public:
	/// Identifies which PhysX joint subclass we wrap.
	enum JointKind {
		JOINT_KIND_NONE = -1,
		JOINT_KIND_PIN,
		JOINT_KIND_HINGE,
		JOINT_KIND_SLIDER,
		JOINT_KIND_CONE_TWIST,
		JOINT_KIND_6DOF,
	};

private:
	physx::PxJoint *px_joint = nullptr;
	JointKind kind = JOINT_KIND_PIN;
	physx::PxRigidActor *body_a = nullptr;
	physx::PxRigidActor *body_b = nullptr;

	// Godot-side parameter storage — only used when the parameter has no
	// PhysX equivalent or when we need round-trip get_param.
	PinJointParams pin_params;
	HingeJointParams hinge_params;
	SliderJointParams slider_params;
	ConeTwistJointParams cone_twist_params;
	G6DOFJointAxisParams g6dof_params[6]; // per-axis

	// --- Private helpers (deduplicate limit/drive reconfiguration) ---
	/// Rebuilds and applies the hinge angular limit from hinge_params.
	/// No-op if use_limit is false.
	void _apply_hinge_limit();
	/// Rebuilds and applies the cone-twist swing + twist limits from cone_twist_params.
	void _apply_cone_twist_limits();
	/// Applies the g6dof angular limit to the correct PhysX limit slot
	/// (twist -> setTwistLimit, swing -> setSwingLimit) for the given axis.
	void _apply_g6dof_angular_limit(physx::PxD6Joint *p_d6, Vector3::Axis p_axis);

	// Cached 6DOF drive velocity — accumulated across all axes
	physx::PxVec3 cached_g6dof_lin_drive_vel{0.0f, 0.0f, 0.0f};
	physx::PxVec3 cached_g6dof_ang_drive_vel{0.0f, 0.0f, 0.0f};

	/// Solver priority, stored for round-trip access. PhysX 5.x exposes no
	/// per-constraint solver priority, so the value is not applied to the
	/// PxConstraint.
	int solver_priority = 0;

public:
	PhysXJoint3D() = default;
	~PhysXJoint3D();

	// --- Factory ---
	static physx::PxJoint *create_px_joint(physx::PxPhysics &p_physics,
			PhysicsServer3D::JointType p_type,
			physx::PxRigidActor *p_body_a,
			const physx::PxTransform &p_local_a,
			physx::PxRigidActor *p_body_b,
			const physx::PxTransform &p_local_b);

	// --- Adoption ---
	void adopt(physx::PxJoint *p_joint, JointKind p_kind,
			physx::PxRigidActor *p_a, physx::PxRigidActor *p_b);

	// --- Release ---
	void release();

	// --- Identity ---
	JointKind get_kind() const { return kind; }
	physx::PxJoint *get_px_joint() const { return px_joint; }
	physx::PxRigidActor *get_body_a() const { return body_a; }
	physx::PxRigidActor *get_body_b() const { return body_b; }

	// --- 6DOF drive velocity caching (per-axis motors must not overwrite each other) ---
	void cache_g6dof_drive_velocity(const physx::PxVec3 &p_lin, const physx::PxVec3 &p_ang);
	void apply_cached_g6dof_drive_velocity(physx::PxD6Joint *p_d6);

	// --- 6DOF linear axis mapping helper ---
	static physx::PxD6Axis::Enum _px_linear_axis(Vector3::Axis p_axis);
	static physx::PxD6Axis::Enum _px_angular_axis(Vector3::Axis p_axis);

	// --- Internal state (for joint_make_* to take ownership) ---
	void set_body_a(physx::PxRigidActor *p_actor, const physx::PxTransform &p_local_a);
	void set_body_b(physx::PxRigidActor *p_actor, const physx::PxTransform &p_local_b);

	// --- Static helpers for joint creation (used by physx_server.cpp) ---
	static physx::PxTransform compute_joint_frame(const physx::PxVec3 &p_pivot, const physx::PxVec3 &p_axis);
	static physx::PxTransform to_physx_transform(const Transform3D &p_transform);
	static physx::PxVec3 to_physx_vec3(const Vector3 &p_vec);
	static Vector3 to_godot_vec3(const physx::PxVec3 &p_vec);

	// --- Local frame access ---
	void set_local_a(const physx::PxTransform &p_transform);
	void set_local_b(const physx::PxTransform &p_transform);
	physx::PxTransform get_local_a_transform() const;
	physx::PxTransform get_local_b_transform() const;

	// --- Solver priority (PxConstraint solver priority) ---
	void set_solver_priority(int p_priority);
	int get_solver_priority() const;

	// --- Collision between bodies ---
	void set_disable_collisions(bool p_disable);
	bool is_disabled_collisions() const;

	// --- Pin joint params ---
	void set_pin_param(PhysicsServer3D::PinJointParam p_param, real_t p_value);
	real_t get_pin_param(PhysicsServer3D::PinJointParam p_param) const;
	void set_local_a(const Vector3 &p_local_a);
	Vector3 get_local_a() const;
	void set_local_b(const Vector3 &p_local_b);
	Vector3 get_local_b() const;

	// --- Hinge joint params ---
	void set_hinge_param(PhysicsServer3D::HingeJointParam p_param, real_t p_value);
	real_t get_hinge_param(PhysicsServer3D::HingeJointParam p_param) const;
	void set_hinge_flag(PhysicsServer3D::HingeJointFlag p_flag, bool p_enabled);
	bool get_hinge_flag(PhysicsServer3D::HingeJointFlag p_flag) const;

	// --- Slider joint params ---
	void set_slider_param(PhysicsServer3D::SliderJointParam p_param, real_t p_value);
	real_t get_slider_param(PhysicsServer3D::SliderJointParam p_param) const;

	// --- Cone/twist joint params ---
	void set_cone_twist_param(PhysicsServer3D::ConeTwistJointParam p_param, real_t p_value);
	real_t get_cone_twist_param(PhysicsServer3D::ConeTwistJointParam p_param) const;

	// --- 6DOF joint params ---
	void set_g6dof_param(Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisParam p_param, real_t p_value);
	real_t get_g6dof_param(Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisParam p_param) const;
	void set_g6dof_flag(Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisFlag p_flag, bool p_enable);
	bool get_g6dof_flag(Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisFlag p_flag) const;
};

#endif // PHYSX_JOINT_3D_H
