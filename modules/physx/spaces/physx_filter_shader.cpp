#include "physx_filter_shader.h"

#include "PxShape.h"
#include "PxRigidActor.h"

// Module-global debug-contacts flag. Read by the (stateless) filter shader on
// the simulation thread; toggled by the server when a space enables debug contacts.
std::atomic<bool> g_physx_debug_contacts_enabled{ false };

/**
 * Custom simulation filter shader implementing Godot's collision semantics.
 *
 * Godot rule (asymmetric OR): two objects collide if EITHER side wants the
 * other, i.e.
 *
 *     (layer0 & mask1) | (layer1 & mask0)   !=  0
 *
 * This is deliberately an OR, not an AND — A can detect B even when B does not
 * detect A. The stock PxDefaultSimulationFilterShader uses a symmetric AND of
 * both directions and applies reserved-bit conventions that do not match the
 * way this module packs word0=layer / word1=mask, so it cannot be used here.
 *
 * PxFilterData word layout (set by PhysXShapedObject3D::update_shapes_collision_filter):
 *   word0 = collision_layer   ("I belong to these layers")
 *   word1 = collision_mask    ("I collide with these layers")
 *   word2 = unused (reserved)
 *   word3 = module flags — bit 0 (PHYSX_FILTER_FLAG_CONTACT_NOTIFY) requests
 *           contact-point notifications for this shape.
 *
 * The shader is stateless and runs on the simulation thread; it must not touch
 * Godot objects. Pair flags are set so that:
 *   - colliding simulation shapes get solver contacts,
 *   - any pair where at least one side is a trigger gets trigger notifications,
 *   - pairs where at least one side requested contact reporting get the
 *     NOTIFY_TOUCH_* + NOTIFY_CONTACT_POINTS flags consumed by onContact,
 *   - any non-trigger pair involving an area shape is killed: monitorable
 *     areas carry a plain (non-trigger) "detection" simulation shape whose
 *     only purpose is to let another area's trigger fire against it (PhysX 5
 *     does not report trigger-vs-trigger pairs), and areas are never solid.
 *
 * NOTE (REG-0011): area actors are kinematic rigid bodies, not static —
 * PhysX never calls this shader for pairs of two static rigid actors, so
 * static areas could never produce area-vs-area trigger events.
 */
physx::PxFilterFlags physx_simulation_filter_shader(
		physx::PxFilterObjectAttributes attributes0, physx::PxFilterData filterData0,
		physx::PxFilterObjectAttributes attributes1, physx::PxFilterData filterData1,
		physx::PxPairFlags &pairFlags,
		const void *constantBlock, physx::PxU32 constantBlockSize) {
	// --- Godot layer/mask test (asymmetric OR) ---
	const bool layer_match = (filterData0.word0 & filterData1.word1) || (filterData1.word0 & filterData0.word1);

	// --- Trigger detection (either side marked as a trigger shape) ---
	const bool is_trigger_pair = physx::PxFilterObjectIsTrigger(attributes0) || physx::PxFilterObjectIsTrigger(attributes1);

	// --- Areas never collide (REG-0011) ---
	// A monitorable area owns a second, non-trigger simulation "detection"
	// shape so that another area's trigger shape can fire against it (PhysX 5
	// does not report trigger-vs-trigger pairs). Those detection shapes must
	// never produce solver contacts — areas are non-solid — so any pair where
	// NEITHER side is a trigger but one side is an area shape is killed.
	// (Trigger pairs involving an area are handled by the trigger branch below,
	// which still applies the layer/mask test.)
	const bool area0 = (filterData0.word3 & PHYSX_FILTER_FLAG_IS_AREA) != 0;
	const bool area1 = (filterData1.word3 & PHYSX_FILTER_FLAG_IS_AREA) != 0;
	if (!is_trigger_pair && (area0 || area1)) {
		pairFlags = physx::PxPairFlags();
		return physx::PxFilterFlag::eKILL;
	}

	// Pairs that neither match nor are triggers never interact.
	if (!layer_match && !is_trigger_pair) {
		pairFlags = physx::PxPairFlags();
		return physx::PxFilterFlag::eKILL;
	}

	if (is_trigger_pair) {
		// Trigger pairs do not solve; they only notify on overlap begin/end.
		// Godot's area detection is one-directional: the area (trigger) checks
		// its mask against the other side's layer. Keep the pair only when the
		// trigger side's mask matches the other side's layer; otherwise kill it.
		// Area-vs-area: the trigger side is one area's trigger shape, the other
		// side the other area's non-trigger detection shape — same one-directional
		// rule (the reverse direction is its own pair/event, REG-0011).
		bool trigger_match = false;
		if (physx::PxFilterObjectIsTrigger(attributes0)) {
			trigger_match = (filterData0.word1 & filterData1.word0) != 0;
		}
		if (!trigger_match && physx::PxFilterObjectIsTrigger(attributes1)) {
			trigger_match = (filterData1.word1 & filterData0.word0) != 0;
		}
		if (!trigger_match) {
			pairFlags = physx::PxPairFlags();
			return physx::PxFilterFlag::eKILL;
		}
		pairFlags = physx::PxPairFlag::eTRIGGER_DEFAULT;
		return physx::PxFilterFlags();
	}

	// --- Simulation (solver) pair ---
	pairFlags = physx::PxPairFlag::eCONTACT_DEFAULT;
	pairFlags |= physx::PxPairFlag::eDETECT_DISCRETE_CONTACT;
	// Route every simulation pair through the contact-modify callback so the
	// Godot material combiner (absorbent/rough → sum-clamped / min-abs) can
	// override the per-contact restitution/friction. PhysX's built-in combine
	// modes can't express Godot's "sum, clamped to [0,1]" rule.
	pairFlags |= physx::PxPairFlag::eMODIFY_CONTACTS;
	// CCD pairs are only resolved if the pair flag is set; the per-body
	// eENABLE_CCD flag on the actor gates which bodies actually sweep.
	pairFlags |= physx::PxPairFlag::eDETECT_CCD_CONTACT;

	// Contact-point notifications if either side asked for them
	// (i.e. a body with max_contacts_reported > 0 is involved), OR when debug
	// contacts are globally enabled (the "Visible Collision Shapes" overlay
	// wants every contact point regardless of per-body reporting).
	const bool want_contact_notify =
			(filterData0.word3 & PHYSX_FILTER_FLAG_CONTACT_NOTIFY) ||
			(filterData1.word3 & PHYSX_FILTER_FLAG_CONTACT_NOTIFY) ||
			g_physx_debug_contacts_enabled.load(std::memory_order_relaxed);
	if (want_contact_notify) {
		pairFlags |= physx::PxPairFlag::eNOTIFY_TOUCH_FOUND;
		pairFlags |= physx::PxPairFlag::eNOTIFY_TOUCH_LOST;
		// PERSIST reports contacts every step while shapes stay in contact —
		// Godot's contact list is live while touching, not transition-only.
		pairFlags |= physx::PxPairFlag::eNOTIFY_TOUCH_PERSISTS;
		pairFlags |= physx::PxPairFlag::eNOTIFY_CONTACT_POINTS;
	}

	// Return eNOTIFY so the filter callback gets invoked — it checks
	// collision exceptions between bodies (which the shader cannot do).
	return physx::PxFilterFlag::eNOTIFY;
}
