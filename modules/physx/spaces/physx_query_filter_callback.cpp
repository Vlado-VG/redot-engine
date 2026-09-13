#include "physx_query_filter_callback.h"

#include "../objects/physx_body_3d.h"
#include "../objects/physx_object_3d.h"
#include "../shapes/physx_shape_3d.h"
#include "../shapes/physx_user_data.h"

physx::PxQueryHitType::Enum PhysXQueryFilterCallback::preFilter(
		const physx::PxFilterData &filterData,
		const physx::PxShape *shape,
		const physx::PxRigidActor *actor,
		physx::PxHitFlags &queryFlags) {
	// Every actor this module owns carries a PhysXActorUserData. If it doesn't,
	// the actor is not one of ours (or was misconfigured) — skip it.
	if (!actor->userData) {
		return physx::PxQueryHitType::eNONE;
	}
	const PhysXActorUserData *actor_data = static_cast<const PhysXActorUserData *>(actor->userData);
	const PhysXObject3D *obj = actor_data->object;
	if (!obj) {
		return physx::PxQueryHitType::eNONE;
	}

	// Godot excludes separation-ray shapes from scene queries entirely: they
	// only participate in the owning body's test_body_motion (recover/collide,
	// which issues its own raycasts), never as a query target. Without this,
	// a raycast/overlap/sweep reports the thin box that emulates the ray.
	// (Mirrors the type check in the _body_motion_cast path.)
	if (shape->userData) {
		const PhysXShape3D *shape_bp = static_cast<const PhysXShape3D *>(shape->userData);
		if (shape_bp && shape_bp->get_type() == PhysicsServer3D::SHAPE_SEPARATION_RAY) {
			return physx::PxQueryHitType::eNONE;
		}
	}

	// Godot semantics: query mask AND object layer must be nonzero to collide.
	if ((collision_mask & obj->get_collision_layer()) == 0) {
		return physx::PxQueryHitType::eNONE;
	}

	// Bodies vs areas toggle.
	const PhysXObject3D::ObjectType type = obj->get_type();
	if (type == PhysXObject3D::OBJECT_TYPE_AREA && !collide_with_areas) {
		return physx::PxQueryHitType::eNONE;
	}
	if (type == PhysXObject3D::OBJECT_TYPE_BODY && !collide_with_bodies) {
		return physx::PxQueryHitType::eNONE;
	}

	// Raycasts honor ray_pickable (intersect_ray sets pick_ray; all other
	// queries leave it false and never consult the flag).
	if (pick_ray && !obj->is_ray_pickable()) {
		return physx::PxQueryHitType::eNONE;
	}

	// Collision exceptions (body_test_motion paths, mirroring godot's
	// _cull_aabb_for_body): skip bodies excepted against the moving body.
	if (motion_body && type == PhysXObject3D::OBJECT_TYPE_BODY) {
		const PhysXBody3D *other = static_cast<const PhysXBody3D *>(obj);
		if (other->get_collision_exception_set().has(motion_body->get_rid()) ||
				motion_body->get_collision_exception_set().has(other->get_rid())) {
			return physx::PxQueryHitType::eNONE;
		}
	}

	// Exclusion lists.
	if (exclude_rids && exclude_rids->has(obj->get_rid())) {
		return physx::PxQueryHitType::eNONE;
	}
	if (exclude_objects && exclude_objects->has(obj->get_instance_id())) {
		return physx::PxQueryHitType::eNONE;
	}

	// Multi-hit queries use eTOUCH to collect all overlaps up to buffer size;
	// single-hit queries use eBLOCK to stop at the first hit.
	return multi_hit ? physx::PxQueryHitType::eTOUCH : physx::PxQueryHitType::eBLOCK;
}

physx::PxQueryHitType::Enum PhysXQueryFilterCallback::postFilter(
		const physx::PxFilterData &filterData,
		const physx::PxQueryHit &hit,
		const physx::PxShape *shape,
		const physx::PxRigidActor *actor) {
	// Pre-filter already applied all Godot semantics; no post-filter needed.
	return physx::PxQueryHitType::eBLOCK;
}
