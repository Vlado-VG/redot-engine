/**
 * @file physx_joint_3d.cpp
 * @brief Implementation of PhysXJoint3D — type-dispatched wrapper for PhysX joints.
 */

#include "physx_joint_3d.h"
#include "../physx_conversions.h"

#include "core/error/error_macros.h"
#include "core/math/math_funcs.h"

#include "foundation/PxTransform.h"
#include "foundation/PxMat33.h"
#include "common/PxTolerancesScale.h"
#include "PxPhysics.h"
#include "extensions/PxRevoluteJoint.h"
#include "extensions/PxPrismaticJoint.h"
#include "extensions/PxD6Joint.h"
#include "extensions/PxJointLimit.h"

// ---------------------------------------------------------------------------
// Static helpers — convert between Godot and PhysX types
// (delegating to the module-wide conversions header, which clamps degenerate
// transforms before they can assert inside PhysX)
// ---------------------------------------------------------------------------

physx::PxTransform PhysXJoint3D::to_physx_transform(const Transform3D &p_transform) {
	return physx_to_px(p_transform);
}

physx::PxVec3 PhysXJoint3D::to_physx_vec3(const Vector3 &p_vec) {
	return physx_to_px(p_vec);
}

Vector3 PhysXJoint3D::to_godot_vec3(const physx::PxVec3 &p_vec) {
	return physx_to_godot(p_vec);
}

/**
 * @brief Builds a PxTransform from a pivot point and axis direction.
 *
 * The axis becomes the X-axis of the joint frame (PhysX convention).
 * The Y and Z axes are orthogonalized via Gram-Schmidt.
 */
physx::PxTransform PhysXJoint3D::compute_joint_frame(const physx::PxVec3 &p_pivot, const physx::PxVec3 &p_axis) {
	// X = axis direction
	physx::PxVec3 x = p_axis;
	x.normalize();

	// Y = arbitrary orthogonal vector (cross with world-up, fallback to world-right)
	physx::PxVec3 world_up(0.0f, 1.0f, 0.0f);
	physx::PxVec3 y;
	if (fabsf(x.dot(world_up)) > 0.99f) {
		// Axis is nearly parallel to world-up, use world-right instead.
		physx::PxVec3 world_right(1.0f, 0.0f, 0.0f);
		y = x.cross(world_right);
	} else {
		y = x.cross(world_up);
	}
	y.normalize();

	// Z = x × y
	physx::PxVec3 z = x.cross(y);

	// Build a 3x3 rotation matrix from the orthogonal basis and convert to quaternion.
	physx::PxMat33 rotmat(x, y, z);
	physx::PxQuat px_quat(rotmat);
	return physx::PxTransform(p_pivot, px_quat);
}

// ---------------------------------------------------------------------------
// PhysXJoint3D destructor — releases the PhysX joint
// ---------------------------------------------------------------------------
PhysXJoint3D::~PhysXJoint3D() {
	release();
}

// ---------------------------------------------------------------------------
// 6DOF helpers — axis mapping and drive velocity caching
// ---------------------------------------------------------------------------
physx::PxD6Axis::Enum PhysXJoint3D::_px_linear_axis(Vector3::Axis p_axis) {
	switch (p_axis) {
		case Vector3::AXIS_X: return physx::PxD6Axis::eX;
		case Vector3::AXIS_Y: return physx::PxD6Axis::eY;
		case Vector3::AXIS_Z: return physx::PxD6Axis::eZ;
		default: return physx::PxD6Axis::eX;
	}
}

physx::PxD6Axis::Enum PhysXJoint3D::_px_angular_axis(Vector3::Axis p_axis) {
	switch (p_axis) {
		case Vector3::AXIS_X: return physx::PxD6Axis::eTWIST;
		case Vector3::AXIS_Y: return physx::PxD6Axis::eSWING1;
		case Vector3::AXIS_Z: return physx::PxD6Axis::eSWING2;
		default: return physx::PxD6Axis::eTWIST;
	}
}

void PhysXJoint3D::cache_g6dof_drive_velocity(const physx::PxVec3 &p_lin, const physx::PxVec3 &p_ang) {
	cached_g6dof_lin_drive_vel = p_lin;
	cached_g6dof_ang_drive_vel = p_ang;
}

void PhysXJoint3D::apply_cached_g6dof_drive_velocity(physx::PxD6Joint *p_d6) {
	p_d6->setDriveVelocity(cached_g6dof_lin_drive_vel, cached_g6dof_ang_drive_vel);
}

// ---------------------------------------------------------------------------
// Factory — creates the correct PhysX joint type based on Godot joint type
// ---------------------------------------------------------------------------
physx::PxJoint *PhysXJoint3D::create_px_joint(physx::PxPhysics &p_physics,
		PhysicsServer3D::JointType p_type,
		physx::PxRigidActor *p_body_a,
		const physx::PxTransform &p_local_a,
		physx::PxRigidActor *p_body_b,
		const physx::PxTransform &p_local_b) {

	physx::PxJoint *px_joint = nullptr;

	switch (p_type) {
		case PhysicsServer3D::JOINT_TYPE_PIN: {
			// PinJoint = pure ball joint: all 3 linear DOFs locked, all 3
			// angular DOFs free. PxD6Joint is the cleanest way to express this.
			px_joint = physx::PxD6JointCreate(p_physics, p_body_a, p_local_a, p_body_b, p_local_b);
			if (px_joint) {
				physx::PxD6Joint *d6 = static_cast<physx::PxD6Joint *>(px_joint);
				// Lock all linear DOFs (X, Y, Z)
				d6->setMotion(physx::PxD6Axis::eX, physx::PxD6Motion::eLOCKED);
				d6->setMotion(physx::PxD6Axis::eY, physx::PxD6Motion::eLOCKED);
				d6->setMotion(physx::PxD6Axis::eZ, physx::PxD6Motion::eLOCKED);
				// Free all angular DOFs (twist, swing1, swing2)
				d6->setMotion(physx::PxD6Axis::eTWIST, physx::PxD6Motion::eFREE);
				d6->setMotion(physx::PxD6Axis::eSWING1, physx::PxD6Motion::eFREE);
				d6->setMotion(physx::PxD6Axis::eSWING2, physx::PxD6Motion::eFREE);
			}
			break;
		}
		case PhysicsServer3D::JOINT_TYPE_HINGE: {
			px_joint = physx::PxRevoluteJointCreate(p_physics, p_body_a, p_local_a, p_body_b, p_local_b);
			break;
		}
		case PhysicsServer3D::JOINT_TYPE_SLIDER: {
			px_joint = physx::PxPrismaticJointCreate(p_physics, p_body_a, p_local_a, p_body_b, p_local_b);
			break;
		}
		case PhysicsServer3D::JOINT_TYPE_CONE_TWIST: {
			px_joint = physx::PxD6JointCreate(p_physics, p_body_a, p_local_a, p_body_b, p_local_b);
			if (px_joint) {
				physx::PxD6Joint *d6 = static_cast<physx::PxD6Joint *>(px_joint);
				// ConeTwist: linear DOFs locked, twist free, swing limited
				d6->setMotion(physx::PxD6Axis::eX, physx::PxD6Motion::eLOCKED);
				d6->setMotion(physx::PxD6Axis::eY, physx::PxD6Motion::eLOCKED);
				d6->setMotion(physx::PxD6Axis::eZ, physx::PxD6Motion::eLOCKED);
				d6->setMotion(physx::PxD6Axis::eTWIST, physx::PxD6Motion::eLIMITED);
				d6->setMotion(physx::PxD6Axis::eSWING1, physx::PxD6Motion::eLIMITED);
				d6->setMotion(physx::PxD6Axis::eSWING2, physx::PxD6Motion::eLIMITED);
			}
			break;
		}
		case PhysicsServer3D::JOINT_TYPE_6DOF: {
			px_joint = physx::PxD6JointCreate(p_physics, p_body_a, p_local_a, p_body_b, p_local_b);
			break;
		}
		default:
			ERR_FAIL_V_MSG(nullptr, "PhysX: unsupported joint type.");
	}

	if (!px_joint) {
		ERR_FAIL_V_MSG(nullptr, "PhysX: failed to create joint.");
	}

	return px_joint;
}

// ---------------------------------------------------------------------------
// Adoption — take ownership of an existing PxJoint from the factory
// ---------------------------------------------------------------------------
void PhysXJoint3D::adopt(physx::PxJoint *p_joint, JointKind p_kind,
		physx::PxRigidActor *p_a, physx::PxRigidActor *p_b) {
	release();
	px_joint = p_joint;
	kind = p_kind;
	body_a = p_a;
	body_b = p_b;
}

// ---------------------------------------------------------------------------
// Release — release the PhysX joint reference
// ---------------------------------------------------------------------------
void PhysXJoint3D::release() {
	if (px_joint) {
		// Wake the connected dynamics before destroying the constraint: a body
		// that fell asleep while constrained has gravity integration skipped
		// (see PhysXBody3D::on_pre_step), so it would keep floating in place
		// after the joint disappears. Actors already removed from a scene
		// (their body is being destroyed) are skipped — wakeUp() is only
		// valid for simulating actors. Kinematic dynamics (e.g. a frozen
		// RigidBody3D joint anchor, as in the parity joints test) never sleep
		// and reject wakeUp() ("Body must be non-kinematic!"), so they are
		// skipped as well.
		for (physx::PxRigidActor *actor : { body_a, body_b }) {
			if (actor && actor->getScene() && actor->is<physx::PxRigidDynamic>()) {
				physx::PxRigidDynamic *dyn = static_cast<physx::PxRigidDynamic *>(actor);
				if (!(dyn->getRigidBodyFlags() & physx::PxRigidBodyFlag::eKINEMATIC)) {
					dyn->wakeUp();
				}
			}
		}
		px_joint->release();
		px_joint = nullptr;
	}
	kind = JOINT_KIND_NONE;
	body_a = nullptr;
	body_b = nullptr;
}

// ---------------------------------------------------------------------------
// Joint configuration — set actors and local poses
// ---------------------------------------------------------------------------
void PhysXJoint3D::set_body_a(physx::PxRigidActor *p_actor, const physx::PxTransform &p_local_a) {
	if (!px_joint) {
		// No PhysX joint yet: Joint3D configures the joint as soon as one node
		// path is assigned and re-applies every param/flag once both are set.
		// Cache-only calls before that point must not raise errors.
		return;
	}
	body_a = p_actor;
	// PhysX allows nullptr actors (joint attached to world frame).
	px_joint->setLocalPose(physx::PxJointActorIndex::eACTOR0, p_local_a);
}

void PhysXJoint3D::set_body_b(physx::PxRigidActor *p_actor, const physx::PxTransform &p_local_b) {
	if (!px_joint) {
		// No PhysX joint yet: Joint3D configures the joint as soon as one node
		// path is assigned and re-applies every param/flag once both are set.
		// Cache-only calls before that point must not raise errors.
		return;
	}
	body_b = p_actor;
	// PhysX allows nullptr actors (joint attached to world frame).
	px_joint->setLocalPose(physx::PxJointActorIndex::eACTOR1, p_local_b);
}

void PhysXJoint3D::set_local_a(const physx::PxTransform &p_transform) {
	if (!px_joint) {
		// No PhysX joint yet: Joint3D configures the joint as soon as one node
		// path is assigned and re-applies every param/flag once both are set.
		// Cache-only calls before that point must not raise errors.
		return;
	}
	px_joint->setLocalPose(physx::PxJointActorIndex::eACTOR0, p_transform);
}

void PhysXJoint3D::set_local_b(const physx::PxTransform &p_transform) {
	if (!px_joint) {
		// No PhysX joint yet: Joint3D configures the joint as soon as one node
		// path is assigned and re-applies every param/flag once both are set.
		// Cache-only calls before that point must not raise errors.
		return;
	}
	px_joint->setLocalPose(physx::PxJointActorIndex::eACTOR1, p_transform);
}

physx::PxTransform PhysXJoint3D::get_local_a_transform() const {
	if (!px_joint) {
		// No PhysX joint yet -- return the fallback quietly (Joint3D
		// re-applies state once the joint has been created).
		return physx::PxTransform();
	}
	return px_joint->getLocalPose(physx::PxJointActorIndex::eACTOR0);
}

physx::PxTransform PhysXJoint3D::get_local_b_transform() const {
	if (!px_joint) {
		// No PhysX joint yet -- return the fallback quietly (Joint3D
		// re-applies state once the joint has been created).
		return physx::PxTransform();
	}
	return px_joint->getLocalPose(physx::PxJointActorIndex::eACTOR1);
}

// ---------------------------------------------------------------------------
// Solver priority — stored for round-trip access only.
// ---------------------------------------------------------------------------
// Note: PhysX 5.x doesn't expose setPriority/getPriority on PxConstraint.
void PhysXJoint3D::set_solver_priority(int p_priority) {
	solver_priority = p_priority;
}

int PhysXJoint3D::get_solver_priority() const {
	return solver_priority;
}

// ---------------------------------------------------------------------------
// Collision between bodies — PxConstraintFlag::eCOLLISION_ENABLED
// ---------------------------------------------------------------------------
void PhysXJoint3D::set_disable_collisions(bool p_disable) {
	if (!px_joint) {
		// No PhysX joint yet: Joint3D configures the joint as soon as one node
		// path is assigned and re-applies every param/flag once both are set.
		// Cache-only calls before that point must not raise errors.
		return;
	}
	physx::PxConstraint *constraint = px_joint->getConstraint();
	if (constraint) {
		constraint->setFlag(physx::PxConstraintFlag::eCOLLISION_ENABLED, !p_disable);
	}
}

bool PhysXJoint3D::is_disabled_collisions() const {
	if (!px_joint) {
		// No PhysX joint yet -- return the fallback quietly (Joint3D
		// re-applies state once the joint has been created).
		return false;
	}
	physx::PxConstraint *constraint = px_joint->getConstraint();
	if (constraint) {
		return !constraint->getFlags().isSet(physx::PxConstraintFlag::eCOLLISION_ENABLED);
	}
	return false;
}

// ---------------------------------------------------------------------------
// Pin joint — local frames, params
// ---------------------------------------------------------------------------
void PhysXJoint3D::set_local_a(const Vector3 &p_local_a) {
	if (!px_joint) {
		// No PhysX joint yet: Joint3D configures the joint as soon as one node
		// path is assigned and re-applies every param/flag once both are set.
		// Cache-only calls before that point must not raise errors.
		return;
	}
	physx::PxTransform local_a = get_local_a_transform();
	local_a.p = physx::PxVec3(p_local_a.x, p_local_a.y, p_local_a.z);
	px_joint->setLocalPose(physx::PxJointActorIndex::eACTOR0, local_a);
}

Vector3 PhysXJoint3D::get_local_a() const {
	if (!px_joint) {
		// No PhysX joint yet -- return the fallback quietly (Joint3D
		// re-applies state once the joint has been created).
		return Vector3();
	}
	physx::PxTransform local_a = px_joint->getLocalPose(physx::PxJointActorIndex::eACTOR0);
	return to_godot_vec3(local_a.p);
}

void PhysXJoint3D::set_local_b(const Vector3 &p_local_b) {
	if (!px_joint) {
		// No PhysX joint yet: Joint3D configures the joint as soon as one node
		// path is assigned and re-applies every param/flag once both are set.
		// Cache-only calls before that point must not raise errors.
		return;
	}
	physx::PxTransform local_b = get_local_b_transform();
	local_b.p = physx::PxVec3(p_local_b.x, p_local_b.y, p_local_b.z);
	px_joint->setLocalPose(physx::PxJointActorIndex::eACTOR1, local_b);
}

Vector3 PhysXJoint3D::get_local_b() const {
	if (!px_joint) {
		// No PhysX joint yet -- return the fallback quietly (Joint3D
		// re-applies state once the joint has been created).
		return Vector3();
	}
	physx::PxTransform local_b = px_joint->getLocalPose(physx::PxJointActorIndex::eACTOR1);
	return to_godot_vec3(local_b.p);
}

void PhysXJoint3D::set_pin_param(PhysicsServer3D::PinJointParam p_param, real_t p_value) {
	// PinJoint parameters have no PhysX equivalent — stored for round-trip.
	switch (p_param) {
		case PhysicsServer3D::PIN_JOINT_BIAS:
			pin_params.bias = (float)p_value;
			break;
		case PhysicsServer3D::PIN_JOINT_DAMPING:
			pin_params.damping = (float)p_value;
			break;
		case PhysicsServer3D::PIN_JOINT_IMPULSE_CLAMP:
			pin_params.impulse_clamp = (float)p_value;
			break;
		default:
			break;
	}
}

real_t PhysXJoint3D::get_pin_param(PhysicsServer3D::PinJointParam p_param) const {
	switch (p_param) {
		case PhysicsServer3D::PIN_JOINT_BIAS:
			return (real_t)pin_params.bias;
		case PhysicsServer3D::PIN_JOINT_DAMPING:
			return (real_t)pin_params.damping;
		case PhysicsServer3D::PIN_JOINT_IMPULSE_CLAMP:
			return (real_t)pin_params.impulse_clamp;
		default:
			return 0.0f;
	}
}

// ---------------------------------------------------------------------------
// Hinge joint — params and flags
// ---------------------------------------------------------------------------

// Rebuilds and applies the hinge angular limit from hinge_params. No-op when
// the limit is disabled. Centralizes the limit construction that was previously
// duplicated across 6 param cases + the USE_LIMIT flag case.
void PhysXJoint3D::_apply_hinge_limit() {
	if (!hinge_params.use_limit || !px_joint) {
		return;
	}
	physx::PxRevoluteJoint *revolute = static_cast<physx::PxRevoluteJoint *>(px_joint);
	physx::PxJointAngularLimitPair limit(hinge_params.limit_lower, hinge_params.limit_upper);
	limit.stiffness = hinge_params.limit_softness;
	limit.damping = hinge_params.limit_relaxation;
	limit.bounceThreshold = hinge_params.limit_bias;
	revolute->setLimit(limit);
}

void PhysXJoint3D::set_hinge_param(PhysicsServer3D::HingeJointParam p_param, real_t p_value) {
	if (!px_joint) {
		// No PhysX joint yet: Joint3D configures the joint as soon as one node
		// path is assigned and re-applies every param/flag once both are set.
		// Cache-only calls before that point must not raise errors.
		return;
	}
	DEV_ASSERT(kind == JOINT_KIND_HINGE); // unchecked static_cast below

	switch (p_param) {
		case PhysicsServer3D::HINGE_JOINT_BIAS: {
			// No PhysX equivalent — stored for round-trip.
			hinge_params.bias = (float)p_value;
			break;
		}
		case PhysicsServer3D::HINGE_JOINT_LIMIT_UPPER: {
			hinge_params.limit_upper = (float)p_value;
			_apply_hinge_limit();
			break;
		}
		case PhysicsServer3D::HINGE_JOINT_LIMIT_LOWER: {
			hinge_params.limit_lower = (float)p_value;
			_apply_hinge_limit();
			break;
		}
		case PhysicsServer3D::HINGE_JOINT_LIMIT_BIAS: {
			hinge_params.limit_bias = (float)p_value;
			_apply_hinge_limit();
			break;
		}
		case PhysicsServer3D::HINGE_JOINT_LIMIT_SOFTNESS: {
			hinge_params.limit_softness = (float)p_value;
			_apply_hinge_limit();
			break;
		}
		case PhysicsServer3D::HINGE_JOINT_LIMIT_RELAXATION: {
			hinge_params.limit_relaxation = (float)p_value;
			_apply_hinge_limit();
			break;
		}
		case PhysicsServer3D::HINGE_JOINT_MOTOR_TARGET_VELOCITY: {
			hinge_params.motor_target_velocity = (float)p_value;
			if (hinge_params.enable_motor) {
				static_cast<physx::PxRevoluteJoint *>(px_joint)->setDriveVelocity(hinge_params.motor_target_velocity);
			}
			break;
		}
		case PhysicsServer3D::HINGE_JOINT_MOTOR_MAX_IMPULSE: {
			hinge_params.motor_max_impulse = (float)p_value;
			if (hinge_params.enable_motor) {
				static_cast<physx::PxRevoluteJoint *>(px_joint)->setDriveForceLimit(hinge_params.motor_max_impulse);
			}
			break;
		}
		default:
			break;
	}
}

real_t PhysXJoint3D::get_hinge_param(PhysicsServer3D::HingeJointParam p_param) const {
	if (!px_joint) {
		// No PhysX joint yet -- return the fallback quietly (Joint3D
		// re-applies state once the joint has been created).
		return 0.0f;
	}
	DEV_ASSERT(kind == JOINT_KIND_HINGE); // unchecked static_cast below
	const physx::PxRevoluteJoint *revolute = static_cast<const physx::PxRevoluteJoint *>(px_joint);

	switch (p_param) {
		case PhysicsServer3D::HINGE_JOINT_BIAS:
			return (real_t)hinge_params.bias;
		case PhysicsServer3D::HINGE_JOINT_LIMIT_UPPER:
			return (real_t)hinge_params.limit_upper;
		case PhysicsServer3D::HINGE_JOINT_LIMIT_LOWER:
			return (real_t)hinge_params.limit_lower;
		case PhysicsServer3D::HINGE_JOINT_LIMIT_BIAS:
			return (real_t)hinge_params.limit_bias;
		case PhysicsServer3D::HINGE_JOINT_LIMIT_SOFTNESS:
			return (real_t)hinge_params.limit_softness;
		case PhysicsServer3D::HINGE_JOINT_LIMIT_RELAXATION:
			return (real_t)hinge_params.limit_relaxation;
		case PhysicsServer3D::HINGE_JOINT_MOTOR_TARGET_VELOCITY:
			return (real_t)hinge_params.motor_target_velocity;
		case PhysicsServer3D::HINGE_JOINT_MOTOR_MAX_IMPULSE:
			return (real_t)hinge_params.motor_max_impulse;
		default:
			return 0.0f;
	}
}

	void PhysXJoint3D::set_hinge_flag(PhysicsServer3D::HingeJointFlag p_flag, bool p_enabled) {
	if (!px_joint) {
		// No PhysX joint yet: Joint3D configures the joint as soon as one node
		// path is assigned and re-applies every param/flag once both are set.
		// Cache-only calls before that point must not raise errors.
		return;
	}
	DEV_ASSERT(kind == JOINT_KIND_HINGE); // unchecked static_cast below
	physx::PxRevoluteJoint *revolute = static_cast<physx::PxRevoluteJoint *>(px_joint);

	switch (p_flag) {
		case PhysicsServer3D::HINGE_JOINT_FLAG_USE_LIMIT: {
			hinge_params.use_limit = p_enabled;
			if (p_enabled) {
				revolute->setRevoluteJointFlag(physx::PxRevoluteJointFlag::eLIMIT_ENABLED, true);
				_apply_hinge_limit();
			} else {
				revolute->setRevoluteJointFlag(physx::PxRevoluteJointFlag::eLIMIT_ENABLED, false);
			}
			break;
		}
		case PhysicsServer3D::HINGE_JOINT_FLAG_ENABLE_MOTOR: {
			hinge_params.enable_motor = p_enabled;
			if (p_enabled) {
				revolute->setRevoluteJointFlag(physx::PxRevoluteJointFlag::eDRIVE_ENABLED, true);
				revolute->setDriveVelocity(hinge_params.motor_target_velocity);
				revolute->setDriveForceLimit(hinge_params.motor_max_impulse);
			} else {
				revolute->setRevoluteJointFlag(physx::PxRevoluteJointFlag::eDRIVE_ENABLED, false);
			}
			break;
		}
		default:
			break;
	}
}

bool PhysXJoint3D::get_hinge_flag(PhysicsServer3D::HingeJointFlag p_flag) const {
	if (!px_joint) {
		// No PhysX joint yet -- return the fallback quietly (Joint3D
		// re-applies state once the joint has been created).
		return false;
	}
	DEV_ASSERT(kind == JOINT_KIND_HINGE); // unchecked static_cast below
	const physx::PxRevoluteJoint *revolute = static_cast<const physx::PxRevoluteJoint *>(px_joint);

	switch (p_flag) {
		case PhysicsServer3D::HINGE_JOINT_FLAG_USE_LIMIT:
			return hinge_params.use_limit;
		case PhysicsServer3D::HINGE_JOINT_FLAG_ENABLE_MOTOR:
			return hinge_params.enable_motor;
		default:
			return false;
	}
}

// ---------------------------------------------------------------------------
// Slider joint — params
// ---------------------------------------------------------------------------
void PhysXJoint3D::set_slider_param(PhysicsServer3D::SliderJointParam p_param, real_t p_value) {
	if (!px_joint) {
		// No PhysX joint yet: Joint3D configures the joint as soon as one node
		// path is assigned and re-applies every param/flag once both are set.
		// Cache-only calls before that point must not raise errors.
		return;
	}
	DEV_ASSERT(kind == JOINT_KIND_SLIDER); // unchecked static_cast below
	physx::PxPrismaticJoint *prismatic = static_cast<physx::PxPrismaticJoint *>(px_joint);

	switch (p_param) {
		// Linear limit upper
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_LIMIT_UPPER:
		{
			slider_params.linear_limit_upper = (float)p_value;
			physx::PxJointLinearLimitPair limit(physx::PxTolerancesScale(), slider_params.linear_limit_lower, slider_params.linear_limit_upper);
			limit.stiffness = slider_params.linear_limit_softness;
			limit.damping = slider_params.linear_limit_damping;
			limit.restitution = slider_params.linear_limit_restitution;
			prismatic->setLimit(limit);
			prismatic->setPrismaticJointFlag(physx::PxPrismaticJointFlag::eLIMIT_ENABLED, true);
			break;
		}
		// Linear limit lower
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_LIMIT_LOWER:
		{
			slider_params.linear_limit_lower = (float)p_value;
			physx::PxJointLinearLimitPair limit(physx::PxTolerancesScale(), slider_params.linear_limit_lower, slider_params.linear_limit_upper);
			limit.stiffness = slider_params.linear_limit_softness;
			limit.damping = slider_params.linear_limit_damping;
			limit.restitution = slider_params.linear_limit_restitution;
			prismatic->setLimit(limit);
			prismatic->setPrismaticJointFlag(physx::PxPrismaticJointFlag::eLIMIT_ENABLED, true);
			break;
		}
		// Linear limit softness
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_LIMIT_SOFTNESS:
		{
			slider_params.linear_limit_softness = (float)p_value;
			physx::PxJointLinearLimitPair limit(physx::PxTolerancesScale(), slider_params.linear_limit_lower, slider_params.linear_limit_upper);
			limit.stiffness = slider_params.linear_limit_softness;
			limit.damping = slider_params.linear_limit_damping;
			limit.restitution = slider_params.linear_limit_restitution;
			prismatic->setLimit(limit);
			break;
		}
		// Linear limit restitution
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_LIMIT_RESTITUTION:
		{
			slider_params.linear_limit_restitution = (float)p_value;
			physx::PxJointLinearLimitPair limit(physx::PxTolerancesScale(), slider_params.linear_limit_lower, slider_params.linear_limit_upper);
			limit.stiffness = slider_params.linear_limit_softness;
			limit.damping = slider_params.linear_limit_damping;
			limit.restitution = slider_params.linear_limit_restitution;
			prismatic->setLimit(limit);
			break;
		}
		// Linear limit damping
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_LIMIT_DAMPING:
		{
			slider_params.linear_limit_damping = (float)p_value;
			physx::PxJointLinearLimitPair limit(physx::PxTolerancesScale(), slider_params.linear_limit_lower, slider_params.linear_limit_upper);
			limit.stiffness = slider_params.linear_limit_softness;
			limit.damping = slider_params.linear_limit_damping;
			limit.restitution = slider_params.linear_limit_restitution;
			prismatic->setLimit(limit);
			break;
		}
		// Angular limit upper
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_LIMIT_UPPER:
		{
			slider_params.angular_limit_upper = (float)p_value;
			// PhysX prismatic joint has no angular limits — store for round-trip.
			break;
		}
		// Angular limit lower
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_LIMIT_LOWER:
		{
			slider_params.angular_limit_lower = (float)p_value;
			break;
		}
		// Angular limit softness
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_LIMIT_SOFTNESS:
		{
			slider_params.angular_limit_softness = (float)p_value;
			break;
		}
		// Angular limit restitution
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_LIMIT_RESTITUTION:
		{
			slider_params.angular_limit_restitution = (float)p_value;
			break;
		}
		// Angular limit damping
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_LIMIT_DAMPING:
		{
			slider_params.angular_limit_damping = (float)p_value;
			break;
		}
		// Linear motion softness
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_MOTION_SOFTNESS:
		{
			slider_params.linear_motion_softness = (float)p_value;
			break;
		}
		// Linear motion restitution
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_MOTION_RESTITUTION:
		{
			slider_params.linear_motion_restitution = (float)p_value;
			break;
		}
		// Linear motion damping
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_MOTION_DAMPING:
		{
			slider_params.linear_motion_damping = (float)p_value;
			break;
		}
		// Linear orthogonal softness
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_ORTHOGONAL_SOFTNESS:
		{
			slider_params.linear_orthogonal_softness = (float)p_value;
			break;
		}
		// Linear orthogonal restitution
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_ORTHOGONAL_RESTITUTION:
		{
			slider_params.linear_orthogonal_restitution = (float)p_value;
			break;
		}
		// Linear orthogonal damping
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_ORTHOGONAL_DAMPING:
		{
			slider_params.linear_orthogonal_damping = (float)p_value;
			break;
		}
		// Angular motion softness
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_MOTION_SOFTNESS:
		{
			slider_params.angular_motion_softness = (float)p_value;
			break;
		}
		// Angular motion restitution
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_MOTION_RESTITUTION:
		{
			slider_params.angular_motion_restitution = (float)p_value;
			break;
		}
		// Angular motion damping
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_MOTION_DAMPING:
		{
			slider_params.angular_motion_damping = (float)p_value;
			break;
		}
		// Angular orthogonal softness
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_ORTHOGONAL_SOFTNESS:
		{
			slider_params.angular_orthogonal_softness = (float)p_value;
			break;
		}
		// Angular orthogonal restitution
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_ORTHOGONAL_RESTITUTION:
		{
			slider_params.angular_orthogonal_restitution = (float)p_value;
			break;
		}
		// Angular orthogonal damping
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_ORTHOGONAL_DAMPING:
		{
			slider_params.angular_orthogonal_damping = (float)p_value;
			break;
		}
		default:
			break;
	}
}

real_t PhysXJoint3D::get_slider_param(PhysicsServer3D::SliderJointParam p_param) const {
	if (!px_joint) {
		// No PhysX joint yet -- return the fallback quietly (Joint3D
		// re-applies state once the joint has been created).
		return 0.0f;
	}
	DEV_ASSERT(kind == JOINT_KIND_SLIDER); // unchecked static_cast below
	const physx::PxPrismaticJoint *prismatic = static_cast<const physx::PxPrismaticJoint *>(px_joint);

	switch (p_param) {
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_LIMIT_UPPER:
			return (real_t)slider_params.linear_limit_upper;
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_LIMIT_LOWER:
			return (real_t)slider_params.linear_limit_lower;
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_LIMIT_SOFTNESS:
			return (real_t)slider_params.linear_limit_softness;
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_LIMIT_RESTITUTION:
			return (real_t)slider_params.linear_limit_restitution;
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_LIMIT_DAMPING:
			return (real_t)slider_params.linear_limit_damping;
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_MOTION_SOFTNESS:
			return (real_t)slider_params.linear_motion_softness;
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_MOTION_RESTITUTION:
			return (real_t)slider_params.linear_motion_restitution;
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_MOTION_DAMPING:
			return (real_t)slider_params.linear_motion_damping;
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_ORTHOGONAL_SOFTNESS:
			return (real_t)slider_params.linear_orthogonal_softness;
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_ORTHOGONAL_RESTITUTION:
			return (real_t)slider_params.linear_orthogonal_restitution;
		case PhysicsServer3D::SLIDER_JOINT_LINEAR_ORTHOGONAL_DAMPING:
			return (real_t)slider_params.linear_orthogonal_damping;
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_LIMIT_UPPER:
			return (real_t)slider_params.angular_limit_upper;
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_LIMIT_LOWER:
			return (real_t)slider_params.angular_limit_lower;
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_LIMIT_SOFTNESS:
			return (real_t)slider_params.angular_limit_softness;
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_LIMIT_RESTITUTION:
			return (real_t)slider_params.angular_limit_restitution;
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_LIMIT_DAMPING:
			return (real_t)slider_params.angular_limit_damping;
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_MOTION_SOFTNESS:
			return (real_t)slider_params.angular_motion_softness;
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_MOTION_RESTITUTION:
			return (real_t)slider_params.angular_motion_restitution;
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_MOTION_DAMPING:
			return (real_t)slider_params.angular_motion_damping;
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_ORTHOGONAL_SOFTNESS:
			return (real_t)slider_params.angular_orthogonal_softness;
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_ORTHOGONAL_RESTITUTION:
			return (real_t)slider_params.angular_orthogonal_restitution;
		case PhysicsServer3D::SLIDER_JOINT_ANGULAR_ORTHOGONAL_DAMPING:
			return (real_t)slider_params.angular_orthogonal_damping;
		default:
			return 0.0f;
	}
}

// ---------------------------------------------------------------------------
// Cone/twist joint — params
// ---------------------------------------------------------------------------

// Rebuilds and applies both the swing (cone) and twist (angular pair) limits
// from cone_twist_params. Centralizes the limit construction that was
// previously duplicated across 5 param cases.
void PhysXJoint3D::_apply_cone_twist_limits() {
	if (!px_joint) {
		return;
	}
	physx::PxD6Joint *d6 = static_cast<physx::PxD6Joint *>(px_joint);

	// PhysX validates cone/pair limits strictly (angles must lie in (0, PI],
	// pair min < pair max). Godot allows degenerate values (a 0 span is a
	// locked joint; the stock 0/0 g6dof ranges), so clamp into the valid range
	// — a hair-thin limit reads as locked, which is the Godot semantic.
	const real_t swing = CLAMP(cone_twist_params.swing_span, 1.0e-4, (real_t)Math::PI);
	physx::PxJointLimitCone swing_limit(swing, swing);
	swing_limit.stiffness = cone_twist_params.softness;
	swing_limit.damping = cone_twist_params.relaxation;
	swing_limit.restitution = cone_twist_params.bias;
	d6->setSwingLimit(swing_limit);

	const real_t twist = CLAMP(cone_twist_params.twist_span, 1.0e-4, (real_t)Math::PI);
	physx::PxJointAngularLimitPair twist_limit(-twist, twist);
	twist_limit.stiffness = cone_twist_params.softness;
	twist_limit.damping = cone_twist_params.relaxation;
	twist_limit.bounceThreshold = cone_twist_params.bias;
	d6->setTwistLimit(twist_limit);
}

void PhysXJoint3D::set_cone_twist_param(PhysicsServer3D::ConeTwistJointParam p_param, real_t p_value) {
	if (!px_joint) {
		// No PhysX joint yet: Joint3D configures the joint as soon as one node
		// path is assigned and re-applies every param/flag once both are set.
		// Cache-only calls before that point must not raise errors.
		return;
	}
	DEV_ASSERT(kind == JOINT_KIND_CONE_TWIST); // unchecked static_cast below

	switch (p_param) {
		case PhysicsServer3D::CONE_TWIST_JOINT_SWING_SPAN:
			cone_twist_params.swing_span = (float)p_value;
			_apply_cone_twist_limits();
			break;
		case PhysicsServer3D::CONE_TWIST_JOINT_TWIST_SPAN:
			cone_twist_params.twist_span = (float)p_value;
			_apply_cone_twist_limits();
			break;
		case PhysicsServer3D::CONE_TWIST_JOINT_BIAS:
			cone_twist_params.bias = (float)p_value;
			_apply_cone_twist_limits();
			break;
		case PhysicsServer3D::CONE_TWIST_JOINT_SOFTNESS:
			cone_twist_params.softness = (float)p_value;
			_apply_cone_twist_limits();
			break;
		case PhysicsServer3D::CONE_TWIST_JOINT_RELAXATION:
			cone_twist_params.relaxation = (float)p_value;
			_apply_cone_twist_limits();
			break;
		default:
			break;
	}
}

real_t PhysXJoint3D::get_cone_twist_param(PhysicsServer3D::ConeTwistJointParam p_param) const {
	if (!px_joint) {
		// No PhysX joint yet -- return the fallback quietly (Joint3D
		// re-applies state once the joint has been created).
		return 0.0f;
	}
	DEV_ASSERT(kind == JOINT_KIND_CONE_TWIST); // unchecked static_cast below
	const physx::PxD6Joint *d6 = static_cast<const physx::PxD6Joint *>(px_joint);

	switch (p_param) {
		case PhysicsServer3D::CONE_TWIST_JOINT_SWING_SPAN:
			return (real_t)cone_twist_params.swing_span;
		case PhysicsServer3D::CONE_TWIST_JOINT_TWIST_SPAN:
			return (real_t)cone_twist_params.twist_span;
		case PhysicsServer3D::CONE_TWIST_JOINT_BIAS:
			return (real_t)cone_twist_params.bias;
		case PhysicsServer3D::CONE_TWIST_JOINT_SOFTNESS:
			return (real_t)cone_twist_params.softness;
		case PhysicsServer3D::CONE_TWIST_JOINT_RELAXATION:
			return (real_t)cone_twist_params.relaxation;
		default:
			return 0.0f;
	}
}

// ---------------------------------------------------------------------------
// 6DOF joint — params and flags (per-axis)
// ---------------------------------------------------------------------------

// Applies the per-axis angular limit to the correct PhysX slot:
//   AXIS_X (twist)  -> setTwistLimit (PxJointAngularLimitPair, lower/upper)
//   AXIS_Y (swing1) } setSwingLimit (PxJointLimitCone, yAngle/zAngle)
//   AXIS_Z (swing2) }
// Swing1 and swing2 share a single cone limit in PhysX, so configuring either
// axis updates the same setSwingLimit using the swing spans of both.
void PhysXJoint3D::_apply_g6dof_angular_limit(physx::PxD6Joint *p_d6, Vector3::Axis p_axis) {
	if (!p_d6) {
		return;
	}
	const G6DOFJointAxisParams &params = g6dof_params[p_axis];

	if (p_axis == Vector3::AXIS_X) {
		// Twist: angular limit pair. PhysX requires lower < upper; a degenerate
		// Godot range (the stock 0/0 defaults on an axis whose limit is off) is
		// nudged open so setTwistLimit stays valid. Whether the axis actually
		// constrains is governed separately by setMotion.
		float lo = (float)params.angular_lower_limit;
		float hi = (float)params.angular_upper_limit;
		if (hi <= lo) {
			hi = lo + 1.0e-4f;
		}
		physx::PxJointAngularLimitPair limit(lo, hi);
		limit.stiffness = params.angular_limit_softness;
		limit.damping = params.angular_damping;
		limit.restitution = params.angular_restitution;
		p_d6->setTwistLimit(limit);
	} else {
		// Swing1 (Y) / Swing2 (Z) share a PxJointLimitCone. Use the lower of
		// the two spans as both yAngle and zAngle extents of the cone, clamped
		// into PhysX's valid (0, PI] range (0 = locked in Godot).
		const real_t y_angle = g6dof_params[Vector3::AXIS_Y].angular_lower_limit;
		const real_t z_angle = g6dof_params[Vector3::AXIS_Z].angular_lower_limit;
		const real_t cone_extent = CLAMP(MIN(y_angle, z_angle), 1.0e-4, (real_t)Math::PI);
		physx::PxJointLimitCone cone_limit(cone_extent, cone_extent);
		cone_limit.stiffness = params.angular_limit_softness;
		cone_limit.damping = params.angular_damping;
		cone_limit.restitution = params.angular_restitution;
		p_d6->setSwingLimit(cone_limit);
	}
}

void PhysXJoint3D::set_g6dof_param(Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisParam p_param, real_t p_value) {
	if (!px_joint) {
		// No PhysX joint yet: Joint3D configures the joint as soon as one node
		// path is assigned and re-applies every param/flag once both are set.
		// Cache-only calls before that point must not raise errors.
		return;
	}
	DEV_ASSERT(kind == JOINT_KIND_6DOF); // unchecked static_cast below
	physx::PxD6Joint *d6 = static_cast<physx::PxD6Joint *>(px_joint);
	G6DOFJointAxisParams &params = g6dof_params[p_axis];
	physx::PxD6Axis::Enum lin_axis = _px_linear_axis(p_axis);
	physx::PxD6Axis::Enum ang_axis = _px_angular_axis(p_axis);
	physx::PxD6Drive::Enum lin_drive = static_cast<physx::PxD6Drive::Enum>(lin_axis);
	physx::PxD6Drive::Enum ang_drive = static_cast<physx::PxD6Drive::Enum>(ang_axis);

	switch (p_param) {
		// Linear limits
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_LOWER_LIMIT:
		{
			params.linear_lower_limit = (float)p_value;
			physx::PxJointLinearLimitPair limit(physx::PxTolerancesScale(), params.linear_lower_limit, params.linear_upper_limit);
			limit.stiffness = params.linear_limit_softness;
			limit.damping = params.linear_damping;
			limit.restitution = params.linear_restitution;
			d6->setLinearLimit(lin_axis, limit);
			d6->setMotion(lin_axis, physx::PxD6Motion::eLIMITED);
			break;
		}
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_UPPER_LIMIT:
		{
			params.linear_upper_limit = (float)p_value;
			physx::PxJointLinearLimitPair limit(physx::PxTolerancesScale(), params.linear_lower_limit, params.linear_upper_limit);
			limit.stiffness = params.linear_limit_softness;
			limit.damping = params.linear_damping;
			limit.restitution = params.linear_restitution;
			d6->setLinearLimit(lin_axis, limit);
			d6->setMotion(lin_axis, physx::PxD6Motion::eLIMITED);
			break;
		}
		// Linear limit softness
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_LIMIT_SOFTNESS:
		{
			params.linear_limit_softness = (float)p_value;
			physx::PxJointLinearLimitPair limit(physx::PxTolerancesScale(), params.linear_lower_limit, params.linear_upper_limit);
			limit.stiffness = params.linear_limit_softness;
			limit.damping = params.linear_damping;
			limit.restitution = params.linear_restitution;
			d6->setLinearLimit(lin_axis, limit);
			break;
		}
		// Linear restitution
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_RESTITUTION:
		{
			params.linear_restitution = (float)p_value;
			physx::PxJointLinearLimitPair limit(physx::PxTolerancesScale(), params.linear_lower_limit, params.linear_upper_limit);
			limit.stiffness = params.linear_limit_softness;
			limit.damping = params.linear_damping;
			limit.restitution = params.linear_restitution;
			d6->setLinearLimit(lin_axis, limit);
			break;
		}
		// Linear damping
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_DAMPING:
		{
			params.linear_damping = (float)p_value;
			physx::PxJointLinearLimitPair limit(physx::PxTolerancesScale(), params.linear_lower_limit, params.linear_upper_limit);
			limit.stiffness = params.linear_limit_softness;
			limit.damping = params.linear_damping;
			limit.restitution = params.linear_restitution;
			d6->setLinearLimit(lin_axis, limit);
			break;
		}
		// Linear motor target velocity � accumulate into cached drive velocity
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_MOTOR_TARGET_VELOCITY:
		{
			params.linear_motor_target_velocity = (float)p_value;
			switch (p_axis) {
				case Vector3::AXIS_X: cached_g6dof_lin_drive_vel.x = params.linear_motor_target_velocity; break;
				case Vector3::AXIS_Y: cached_g6dof_lin_drive_vel.y = params.linear_motor_target_velocity; break;
				case Vector3::AXIS_Z: cached_g6dof_lin_drive_vel.z = params.linear_motor_target_velocity; break;
				default: break;
			}
			d6->setDriveVelocity(cached_g6dof_lin_drive_vel, cached_g6dof_ang_drive_vel);
			{
				physx::PxD6JointDrive drive;
				drive.stiffness = 0.0f;
				drive.damping = 0.0f;
				drive.forceLimit = params.linear_motor_force_limit;
				d6->setDrive(lin_drive, drive);
			}
			break;
		}
		// Linear motor force limit
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_MOTOR_FORCE_LIMIT:
		{
			params.linear_motor_force_limit = (float)p_value;
			physx::PxD6JointDrive drive;
			drive.stiffness = 0.0f;
			drive.damping = 0.0f;
			drive.forceLimit = params.linear_motor_force_limit;
			d6->setDrive(lin_drive, drive);
			break;
		}
		// Linear spring stiffness
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_SPRING_STIFFNESS:
		{
			params.linear_spring_stiffness = (float)p_value;
			physx::PxD6JointDrive drive;
			drive.stiffness = params.linear_spring_stiffness;
			drive.damping = params.linear_spring_damping;
			drive.forceLimit = params.linear_motor_force_limit;
			d6->setDrive(lin_drive, drive);
			break;
		}
		// Linear spring damping
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_SPRING_DAMPING:
		{
			params.linear_spring_damping = (float)p_value;
			physx::PxD6JointDrive drive;
			drive.stiffness = params.linear_spring_stiffness;
			drive.damping = params.linear_spring_damping;
			drive.forceLimit = params.linear_motor_force_limit;
			d6->setDrive(lin_drive, drive);
			break;
		}
		// Linear spring equilibrium point
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_SPRING_EQUILIBRIUM_POINT:
		{
			params.linear_spring_equilibrium_point = (float)p_value;
			// PxD6Joint::setDrivePosition is used for positional drives.
			break;
		}
		// Angular limits
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_LOWER_LIMIT:
		{
			params.angular_lower_limit = (float)p_value;
			_apply_g6dof_angular_limit(d6, p_axis);
			d6->setMotion(ang_axis, physx::PxD6Motion::eLIMITED);
			break;
		}
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_UPPER_LIMIT:
		{
			params.angular_upper_limit = (float)p_value;
			_apply_g6dof_angular_limit(d6, p_axis);
			break;
		}
		// Angular limit softness
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_LIMIT_SOFTNESS:
		{
			params.angular_limit_softness = (float)p_value;
			_apply_g6dof_angular_limit(d6, p_axis);
			break;
		}
		// Angular restitution
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_RESTITUTION:
		{
			params.angular_restitution = (float)p_value;
			_apply_g6dof_angular_limit(d6, p_axis);
			break;
		}
		// Angular damping
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_DAMPING:
		{
			params.angular_damping = (float)p_value;
			_apply_g6dof_angular_limit(d6, p_axis);
			break;
		}
		// Angular force limit (max torque for motor/limit correction)
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_FORCE_LIMIT:
		{
			params.angular_force_limit = (float)p_value;
			// PxD6Joint doesn't have a direct max torque parameter for limits.
			// This is stored for round-trip but has no effect on the solver.
			break;
		}
		// Angular ERP (error reduction parameter for positional drift)
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_ERP:
		{
			params.angular_erp = (float)p_value;
			// PxD6Joint doesn't have an ERP parameter � stored for round-trip.
			break;
		}
		// Angular motor target velocity � accumulate into cached drive velocity
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_MOTOR_TARGET_VELOCITY:
		{
			params.angular_motor_target_velocity = (float)p_value;
			switch (p_axis) {
				case Vector3::AXIS_X: cached_g6dof_ang_drive_vel.x = params.angular_motor_target_velocity; break;
				case Vector3::AXIS_Y: cached_g6dof_ang_drive_vel.y = params.angular_motor_target_velocity; break;
				case Vector3::AXIS_Z: cached_g6dof_ang_drive_vel.z = params.angular_motor_target_velocity; break;
				default: break;
			}
			d6->setDriveVelocity(cached_g6dof_lin_drive_vel, cached_g6dof_ang_drive_vel);
			{
				physx::PxD6JointDrive drive;
				drive.stiffness = 0.0f;
				drive.damping = 0.0f;
				drive.forceLimit = params.angular_motor_force_limit;
				d6->setDrive(ang_drive, drive);
			}
			break;
		}
		// Angular motor force limit
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_MOTOR_FORCE_LIMIT:
		{
			params.angular_motor_force_limit = (float)p_value;
			physx::PxD6JointDrive drive;
			drive.stiffness = 0.0f;
			drive.damping = 0.0f;
			drive.forceLimit = params.angular_motor_force_limit;
			d6->setDrive(ang_drive, drive);
			break;
		}
		// Angular spring stiffness
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_SPRING_STIFFNESS:
		{
			params.angular_spring_stiffness = (float)p_value;
			physx::PxD6JointDrive drive;
			drive.stiffness = params.angular_spring_stiffness;
			drive.damping = params.angular_spring_damping;
			drive.forceLimit = params.angular_motor_force_limit;
			d6->setDrive(ang_drive, drive);
			break;
		}
		// Angular spring damping
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_SPRING_DAMPING:
		{
			params.angular_spring_damping = (float)p_value;
			physx::PxD6JointDrive drive;
			drive.stiffness = params.angular_spring_stiffness;
			drive.damping = params.angular_spring_damping;
			drive.forceLimit = params.angular_motor_force_limit;
			d6->setDrive(ang_drive, drive);
			break;
		}
		// Angular spring equilibrium point
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_SPRING_EQUILIBRIUM_POINT:
		{
			params.angular_spring_equilibrium_point = (float)p_value;
			// PxD6Joint::setDrivePosition for positional drives.
			break;
		}
		default:
			break;
	}
}

real_t PhysXJoint3D::get_g6dof_param(Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisParam p_param) const {
	if (!px_joint) {
		// No PhysX joint yet -- return the fallback quietly (Joint3D
		// re-applies state once the joint has been created).
		return 0.0f;
	}
	DEV_ASSERT(kind == JOINT_KIND_6DOF); // unchecked static_cast below
	const physx::PxD6Joint *d6 = static_cast<const physx::PxD6Joint *>(px_joint);

	const G6DOFJointAxisParams &params = g6dof_params[p_axis];

	switch (p_param) {
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_LOWER_LIMIT:
			return (real_t)params.linear_lower_limit;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_UPPER_LIMIT:
			return (real_t)params.linear_upper_limit;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_LIMIT_SOFTNESS:
			return (real_t)params.linear_limit_softness;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_RESTITUTION:
			return (real_t)params.linear_restitution;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_DAMPING:
			return (real_t)params.linear_damping;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_MOTOR_TARGET_VELOCITY:
			return (real_t)params.linear_motor_target_velocity;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_MOTOR_FORCE_LIMIT:
			return (real_t)params.linear_motor_force_limit;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_SPRING_STIFFNESS:
			return (real_t)params.linear_spring_stiffness;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_SPRING_DAMPING:
			return (real_t)params.linear_spring_damping;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_SPRING_EQUILIBRIUM_POINT:
			return (real_t)params.linear_spring_equilibrium_point;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_LOWER_LIMIT:
			return (real_t)params.angular_lower_limit;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_UPPER_LIMIT:
			return (real_t)params.angular_upper_limit;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_LIMIT_SOFTNESS:
			return (real_t)params.angular_limit_softness;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_RESTITUTION:
			return (real_t)params.angular_restitution;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_DAMPING:
			return (real_t)params.angular_damping;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_FORCE_LIMIT:
			return (real_t)params.angular_force_limit;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_ERP:
			return (real_t)params.angular_erp;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_MOTOR_TARGET_VELOCITY:
			return (real_t)params.angular_motor_target_velocity;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_MOTOR_FORCE_LIMIT:
			return (real_t)params.angular_motor_force_limit;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_SPRING_STIFFNESS:
			return (real_t)params.angular_spring_stiffness;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_SPRING_DAMPING:
			return (real_t)params.angular_spring_damping;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_SPRING_EQUILIBRIUM_POINT:
			return (real_t)params.angular_spring_equilibrium_point;
		default:
			return 0.0f;
	}
}

void PhysXJoint3D::set_g6dof_flag(Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisFlag p_flag, bool p_enable) {
	if (!px_joint) {
		// No PhysX joint yet: Joint3D configures the joint as soon as one node
		// path is assigned and re-applies every param/flag once both are set.
		// Cache-only calls before that point must not raise errors.
		return;
	}
	DEV_ASSERT(kind == JOINT_KIND_6DOF); // unchecked static_cast below
	physx::PxD6Joint *d6 = static_cast<physx::PxD6Joint *>(px_joint);
	G6DOFJointAxisParams &params = g6dof_params[p_axis];
	physx::PxD6Axis::Enum lin_axis = _px_linear_axis(p_axis);
	physx::PxD6Axis::Enum ang_axis = _px_angular_axis(p_axis);
	physx::PxD6Drive::Enum lin_drive = static_cast<physx::PxD6Drive::Enum>(lin_axis);
	physx::PxD6Drive::Enum ang_drive = static_cast<physx::PxD6Drive::Enum>(ang_axis);

	switch (p_flag) {
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_LINEAR_LIMIT:
		{
			// Godot semantics: the flag alone governs the axis — enabled means
			// limited, disabled means FREE (not locked).
			d6->setMotion(lin_axis, p_enable ? physx::PxD6Motion::eLIMITED : physx::PxD6Motion::eFREE);
			break;
		}
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_ANGULAR_LIMIT:
		{
			d6->setMotion(ang_axis, p_enable ? physx::PxD6Motion::eLIMITED : physx::PxD6Motion::eFREE);
			break;
		}
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_ANGULAR_SPRING:
		{
			params.angular_spring_equilibrium_point = 0.0f;
			{
				physx::PxD6JointDrive drive;
				drive.stiffness = params.angular_spring_stiffness;
				drive.damping = params.angular_spring_damping;
				drive.forceLimit = params.angular_motor_force_limit;
				d6->setDrive(ang_drive, drive);
			}
			break;
		}
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_LINEAR_SPRING:
		{
			params.linear_spring_equilibrium_point = 0.0f;
			{
				physx::PxD6JointDrive drive;
				drive.stiffness = params.linear_spring_stiffness;
				drive.damping = params.linear_spring_damping;
				drive.forceLimit = params.linear_motor_force_limit;
				d6->setDrive(lin_drive, drive);
			}
			break;
		}
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_MOTOR:
		{
			// Update cached angular drive velocity for this axis
			switch (p_axis) {
				case Vector3::AXIS_Y: cached_g6dof_ang_drive_vel.y = params.angular_motor_target_velocity; break;
				case Vector3::AXIS_Z: cached_g6dof_ang_drive_vel.z = params.angular_motor_target_velocity; break;
				default: break;
			}
			d6->setDriveVelocity(cached_g6dof_lin_drive_vel, cached_g6dof_ang_drive_vel);
			{
				physx::PxD6JointDrive drive;
				drive.stiffness = 0.0f;
				drive.damping = 0.0f;
				drive.forceLimit = params.angular_motor_force_limit;
				d6->setDrive(ang_drive, drive);
			}
			break;
		}
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_LINEAR_MOTOR:
		{
			// Update cached linear drive velocity for this axis
			switch (p_axis) {
				case Vector3::AXIS_X: cached_g6dof_lin_drive_vel.x = params.linear_motor_target_velocity; break;
				case Vector3::AXIS_Y: cached_g6dof_lin_drive_vel.y = params.linear_motor_target_velocity; break;
				case Vector3::AXIS_Z: cached_g6dof_lin_drive_vel.z = params.linear_motor_target_velocity; break;
				default: break;
			}
			d6->setDriveVelocity(cached_g6dof_lin_drive_vel, cached_g6dof_ang_drive_vel);
			{
				physx::PxD6JointDrive drive;
				drive.stiffness = 0.0f;
				drive.damping = 0.0f;
				drive.forceLimit = params.linear_motor_force_limit;
				d6->setDrive(lin_drive, drive);
			}
			break;
		}
		default:
			break;
	}
}

bool PhysXJoint3D::get_g6dof_flag(Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisFlag p_flag) const {
	if (!px_joint) {
		// No PhysX joint yet -- return the fallback quietly (Joint3D
		// re-applies state once the joint has been created).
		return false;
	}
	DEV_ASSERT(kind == JOINT_KIND_6DOF); // unchecked static_cast below
	const physx::PxD6Joint *d6 = static_cast<const physx::PxD6Joint *>(px_joint);
	const G6DOFJointAxisParams &params = g6dof_params[p_axis];

	switch (p_flag) {
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_LINEAR_LIMIT:
		{
			physx::PxD6Axis::Enum lin_axis = static_cast<physx::PxD6Axis::Enum>(_px_linear_axis(p_axis));
			return d6->getMotion(lin_axis) == physx::PxD6Motion::eLIMITED;
		}
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_ANGULAR_LIMIT:
		{
			physx::PxD6Axis::Enum ang_axis = static_cast<physx::PxD6Axis::Enum>(_px_angular_axis(p_axis));
			return d6->getMotion(ang_axis) == physx::PxD6Motion::eLIMITED;
		}
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_ANGULAR_SPRING:
			return params.angular_spring_stiffness > 0.0f;
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_LINEAR_SPRING:
			return params.linear_spring_stiffness > 0.0f;
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_MOTOR:
			return params.angular_motor_target_velocity != 0.0f;
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_LINEAR_MOTOR:
			return params.linear_motor_target_velocity != 0.0f;
		default:
			return false;
	}
}
