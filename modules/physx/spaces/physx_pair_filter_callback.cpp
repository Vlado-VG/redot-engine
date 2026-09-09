/**
 * @file physx_pair_filter_callback.cpp
 * @brief PxSimulationFilterCallback implementation — enforces collision exceptions.
 */

#include "physx_pair_filter_callback.h"
#include "../objects/physx_body_3d.h"
#include "../shapes/physx_user_data.h"

physx::PxFilterFlags PhysXPairFilterCallback::pairFound(
	physx::PxU64,
	physx::PxFilterObjectAttributes, physx::PxFilterData,
	const physx::PxActor* a0, const physx::PxShape*,
	physx::PxFilterObjectAttributes, physx::PxFilterData,
	const physx::PxActor* a1, const physx::PxShape*,
	physx::PxPairFlags& pairFlags) {
	// Collision exceptions only apply between bodies. If either side is not a
	// body (e.g. an area), there's nothing to check — let the pair through.
	if (!a0->userData || !a1->userData) {
		return physx::PxFilterFlags();
	}

	const PhysXActorUserData* ud0 = static_cast<const PhysXActorUserData*>(a0->userData);
	const PhysXActorUserData* ud1 = static_cast<const PhysXActorUserData*>(a1->userData);
	if (!ud0->object || !ud1->object) {
		return physx::PxFilterFlags();
	}

	const PhysXObject3D* obj0 = ud0->object;
	const PhysXObject3D* obj1 = ud1->object;

	// Both must be bodies — areas don't have collision exceptions.
	if (obj0->get_type() != PhysXObject3D::OBJECT_TYPE_BODY ||
			obj1->get_type() != PhysXObject3D::OBJECT_TYPE_BODY) {
		return physx::PxFilterFlags();
	}

	// Check: does body0 have an exception for body1, or vice versa?
	const PhysXBody3D* body0 = static_cast<const PhysXBody3D*>(obj0);
	const PhysXBody3D* body1 = static_cast<const PhysXBody3D*>(obj1);

	if (body0->get_collision_exception_set().has(body1->get_rid()) ||
			body1->get_collision_exception_set().has(body0->get_rid())) {
		// Suppress this pair — collision exception found.
		return physx::PxFilterFlag::eKILL;
	}

	return physx::PxFilterFlags();
}
