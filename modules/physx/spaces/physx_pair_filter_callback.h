/**
 * @file physx_pair_filter_callback.h
 * @brief PxSimulationFilterCallback that enforces collision exceptions.
 *
 * The simulation filter shader cannot consult Godot objects (it runs stateless
 * on the sim thread), so collision exceptions are enforced here — in the
 * post-shader filter callback, which has access to both actors' userData and
 * can look up the wrapper objects to check their collision_exceptions sets.
 */

#ifndef PHYSX_PAIR_FILTER_CALLBACK_H
#define PHYSX_PAIR_FILTER_CALLBACK_H

#include <PxPhysicsAPI.h>

class PhysXPairFilterCallback : public physx::PxSimulationFilterCallback {
public:
	physx::PxFilterFlags pairFound(
		physx::PxU64 pairID,
		physx::PxFilterObjectAttributes attributes0, physx::PxFilterData filterData0,
		const physx::PxActor* a0, const physx::PxShape* s0,
		physx::PxFilterObjectAttributes attributes1, physx::PxFilterData filterData1,
		const physx::PxActor* a1, const physx::PxShape* s1,
		physx::PxPairFlags& pairFlags) override;

	virtual void pairLost(
		physx::PxU64 pairID,
		physx::PxFilterObjectAttributes attributes0, physx::PxFilterData filterData0,
		physx::PxFilterObjectAttributes attributes1, physx::PxFilterData filterData1,
		bool objectRemoved) override {
		// No-op — we don't need to track pair lifecycles for exceptions.
	}

	virtual bool statusChange(
		physx::PxU64& pairID,
		physx::PxPairFlags& pairFlags,
		physx::PxFilterFlags& filterFlags) override {
		return false; // No dynamic status changes needed.
	}
};

#endif // PHYSX_PAIR_FILTER_CALLBACK_H
