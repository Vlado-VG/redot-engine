/**
 * @file physx_simulation_event_callback.h
 * @brief PxSimulationEventCallback implementation for one PhysX scene.
 *
 * Translates PhysX simulation events into Godot-side data:
 *   - onContact: populates per-body contact buffers (PhysXBody3D::contacts),
 *     read by PhysXDirectBodyState3D's contact getters.
 *   - onTrigger: maintains area overlap state and defers Area3D monitor
 *     callbacks to flush_queries() (AREA_BODY_ADDED / AREA_BODY_REMOVED).
 *
 * The callback resolves actors back to PhysXBody3D / PhysXArea3D via the
 * PhysXActorUserData attached to each PxActor's userData field.
 */

#ifndef PHYSX_SIMULATION_EVENT_CALLBACK_H
#define PHYSX_SIMULATION_EVENT_CALLBACK_H

#include "PxSimulationEventCallback.h"

class PhysXSpace3D;

// PxSimulationEventCallback implementation for one PhysX scene. Translates
// PhysX contact and trigger events into Godot-side data:
//   - onContact populates per-body contact buffers (read by DirectBodyState).
//   - onTrigger dispatches Area3D monitor callbacks.
//
// The callback owns no state of its own; it resolves actors back to
// PhysXBody3D / PhysXArea3D via the userData attached to each PxActor.
class PhysXSimulationEventCallback : public physx::PxSimulationEventCallback {
public:
	explicit PhysXSimulationEventCallback(PhysXSpace3D *p_space) : space(p_space) {}

	virtual void onConstraintBreak(physx::PxConstraintInfo *constraints, physx::PxU32 count) override;
	virtual void onWake(physx::PxActor **actors, physx::PxU32 count) override;
	virtual void onSleep(physx::PxActor **actors, physx::PxU32 count) override;
	virtual void onContact(const physx::PxContactPairHeader &pairHeader, const physx::PxContactPair *pairs, physx::PxU32 nbPairs) override;
	virtual void onTrigger(physx::PxTriggerPair *pairs, physx::PxU32 count) override;
	virtual void onAdvance(const physx::PxRigidBody *const *bodyBuffer, const physx::PxTransform *poseBuffer, physx::PxU32 count) override;

private:
	PhysXSpace3D *space = nullptr;
};
#endif // PHYSX_SIMULATION_EVENT_CALLBACK_H
