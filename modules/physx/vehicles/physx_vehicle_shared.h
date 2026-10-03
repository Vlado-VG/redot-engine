/**
 * @file physx_vehicle_shared.h
 * @brief Composition helpers shared by BOTH vehicle stacks (the server-RID
 *        assembly in physx_vehicle_server.cpp and the node-level PxVehicle2
 *        compositions in physx_vehicle4w/2w/track.h).
 *
 * This exists to keep the two stacks' vehicle conventions from drifting again:
 * the left-handed-frame regression (VEHN-2) happened exactly because the 4W
 * composition fixed its frame while 2W/track kept their own copies. Anything
 * here must stay a pure inline helper — no Godot node types, no state.
 *
 * Conventions (Godot, matching physx_conversions.h's 1:1 axis mapping):
 *   forward = -Z, right = +X, up = +Y.
 */

#ifndef PHYSX_VEHICLE_SHARED_H
#define PHYSX_VEHICLE_SHARED_H

#include "core/math/vector3.h"

#include <PxPhysicsAPI.h>
#include <vehicle/PxVehicleAPI.h>

// ---------------------------------------------------------------------------
// Frame: Godot's forward = -Z requires the RIGHT-handed SDK triple
// (lng x lat = vrt): lngAxis = eNegZ, latAxis = eNegX, vrtAxis = ePosY.
// latAxis = ePosX makes a left-handed triple — PxVehicleFrame::isValid()
// builds a quaternion from the basis and a mirrored basis has determinant -1,
// which is not a unit rotation, so the frame is illegal (checked builds warn;
// unchecked builds run with mirrored drive/lateral behavior).
// ---------------------------------------------------------------------------
inline physx::PxVehicleFrame physx_vehicle_frame() {
	physx::PxVehicleFrame f;
	f.setToDefault();
	f.lngAxis = physx::PxVehicleAxes::eNegZ;
	f.latAxis = physx::PxVehicleAxes::eNegX;
	f.vrtAxis = physx::PxVehicleAxes::ePosY;
	return f;
}

// Simulation-context gravity from the owning scene (VEHN-5 / VEH-5: the
// hardcoded -9.81 made custom scene gravity wrong on the node stack; the
// server path already read the scene).
inline physx::PxVec3 physx_vehicle_scene_gravity(const physx::PxScene &p_scene, const physx::PxVehicleFrame &p_frame) {
	const physx::PxVec3 g = p_scene.getGravity();
	return p_frame.getVrtAxis() * -g.magnitude();
}

// ---------------------------------------------------------------------------
// Command clamps (VEHN-7): Godot exposes [0,1] throttle/brake and [-1,1]
// steer/ratio conventions; out-of-range script values otherwise produce
// proportionally out-of-range torque through the linear response model.
// ---------------------------------------------------------------------------
inline physx::PxReal physx_vehicle_clamp01(real_t p_v) {
	return (physx::PxReal)CLAMP(p_v, 0.0, 1.0);
}

inline physx::PxReal physx_vehicle_clamp_sym(real_t p_v) {
	return (physx::PxReal)CLAMP(p_v, -1.0, 1.0);
}

// Explicit chassis gravity-off (VEHN-4): PxVehicle2 applies gravity itself
// through the simulation context (PxVehicleRigidBodyComponent), so the chassis
// actor must have scene gravity disabled or it is applied twice. The node
// stack relied on PxVehiclePhysXActorCreate doing this implicitly — set it
// explicitly so an SDK behavior change cannot silently double-apply gravity.
inline void physx_vehicle_chassis_gravity_off(physx::PxRigidActor &p_actor) {
	p_actor.setActorFlag(physx::PxActorFlag::eDISABLE_GRAVITY, true);
}

#endif // PHYSX_VEHICLE_SHARED_H
