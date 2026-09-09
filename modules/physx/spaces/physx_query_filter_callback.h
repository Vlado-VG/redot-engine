/**
 * @file physx_query_filter_callback.h
 * @brief PxQueryFilterCallback for scene queries (raycast, overlap, sweep).
 *
 * Applies Godot's query semantics to PhysX scene queries:
 *   - collision_mask vs object's collision_layer (bitwise AND must be nonzero)
 *   - collide_with_bodies / collide_with_areas toggle
 *   - RID and ObjectID exclusion lists
 *   - eTOUCH vs eBLOCK for multi-hit vs single-hit queries
 *
 * All filtering reads from the PhysXActorUserData attached to each actor, so
 * every actor this module puts into a scene must carry one.
 *
 * Note: PxShape::userData points at the shared PhysXShape3D* blueprint.
 */

#ifndef PHYSX_QUERY_FILTER_CALLBACK_H
#define PHYSX_QUERY_FILTER_CALLBACK_H

#include "core/object/object.h"
#include "core/templates/hash_set.h"
#include "core/templates/rid.h"
#include "PxPhysicsAPI.h"

// PxQueryFilterCallback implementation that applies Godot's query semantics
// (collision mask vs layer, bodies-vs-areas toggle, RID/ObjectID exclusions)
// to PhysX scene queries (raycast / overlap / sweep).
//
// All filtering reads from the PhysXActorUserData attached to each actor, so
// every actor this module puts into a scene must carry one.
//
// multi_hit flag: when true, preFilter returns eTOUCH so PhysX collects all
// overlapping shapes up to the buffer size (intersect_point, intersect_shape,
// collide_shape, motion overlaps). When false, returns eBLOCK for the single
// closest hit (intersect_ray, cast_motion).
class PhysXBody3D;

class PhysXQueryFilterCallback : public physx::PxQueryFilterCallback {
public:
	uint32_t collision_mask = 0;
	bool collide_with_bodies = true;
	bool collide_with_areas = false;
	const HashSet<RID> *exclude_rids = nullptr;
	const HashSet<ObjectID> *exclude_objects = nullptr;
	/// When set (body_test_motion paths), pairs connected by a collision
	/// exception with this body are rejected — Godot's test_body_motion
	/// respects exceptions (see godot_space_3d.cpp _cull_aabb_for_body).
	const PhysXBody3D *motion_body = nullptr;
	bool multi_hit = false; ///< true = return eTOUCH (collect all); false = return eBLOCK (single hit)

	virtual physx::PxQueryHitType::Enum preFilter(
			const physx::PxFilterData &filterData,
			const physx::PxShape *shape,
			const physx::PxRigidActor *actor,
			physx::PxHitFlags &queryFlags) override;

	virtual physx::PxQueryHitType::Enum postFilter(
			const physx::PxFilterData &filterData,
			const physx::PxQueryHit &hit,
			const physx::PxShape *shape,
			const physx::PxRigidActor *actor) override;
};
#endif // PHYSX_QUERY_FILTER_CALLBACK_H
