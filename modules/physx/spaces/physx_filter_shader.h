#ifndef PHYSX_FILTER_SHADER_H
#define PHYSX_FILTER_SHADER_H

#include "PxFiltering.h"
#include <atomic>

// PxFilterData word conventions for this module:
//   word0 = collision_layer   (Godot layer bits, "I belong to these layers")
//   word1 = collision_mask    (Godot mask bits, "I collide with these layers")
//   word2 = unused (reserved for future use, e.g. group indices)
//   word3 = module flags:
//           bit 0 (0x1): contact notification requested
//                        (set when body has max_contacts_reported > 0)
//           bit 1 (0x2): shape belongs to an area (trigger or detection
//                        shape; areas must never generate solver contacts)
#define PHYSX_FILTER_FLAG_CONTACT_NOTIFY 0x1u
#define PHYSX_FILTER_FLAG_IS_AREA 0x2u

// Module-global flag: when true (debug-contacts enabled on any space), the
// simulation filter shader requests contact-point notifications for every
// colliding pair so onContact can record them for the "Visible Collision
// Shapes" overlay — even when no body has max_contacts_reported > 0.
// Toggled by PhysXSpace3D::set_debug_contacts via the server.
extern std::atomic<bool> g_physx_debug_contacts_enabled;

// Custom simulation filter shader implementing Godot's collision semantics.
//
// Implements the Godot layer/mask test directly — an asymmetric OR:
//     collide iff (layer0 & mask1) | (layer1 & mask0) != 0
// The stock PxDefaultSimulationFilterShader cannot be used because it applies
// a symmetric AND plus reserved-bit conventions that conflict with this
// module's word0=layer / word1=mask packing.
//
// Also: marks trigger pairs for eTRIGGER_DEFAULT, and requests contact-point
// notifications (eNOTIFY_TOUCH_FOUND/LOST + eNOTIFY_CONTACT_POINTS) for any
// pair where at least one side carries PHYSX_FILTER_FLAG_CONTACT_NOTIFY.
//
physx::PxFilterFlags physx_simulation_filter_shader(
		physx::PxFilterObjectAttributes attributes0, physx::PxFilterData filterData0,
		physx::PxFilterObjectAttributes attributes1, physx::PxFilterData filterData1,
		physx::PxPairFlags &pairFlags,
		const void *constantBlock, physx::PxU32 constantBlockSize);

#endif // PHYSX_FILTER_SHADER_H
