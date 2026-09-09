/**
 * @file physx_rid_owner.h
 * @brief RID ownership mixin for PhysX-side objects.
 *
 * Every PhysX object that Godot creates (shapes, bodies, areas, spaces) is
 * tracked through a Resource ID (RID) so the engine can hand opaque handles
 * to scripts and the scene tree. This mixin holds that RID and enforces the
 * "assign once at registration time" invariant.
 */

#ifndef PHYSX_RID_OWNER_H
#define PHYSX_RID_OWNER_H

#include "core/templates/rid.h"
#include <core/error/error_macros.h>

/**
 * @brief Holds the RID under which a PhysX-side object is registered.
 *
 * The RID is assigned exactly once at registration time. The DEV_ASSERT in
 * set_rid() prevents the same object from being silently re-registered under
 * a new RID, which would orphan the previous entry in the RID owner table.
 */
class PhysXRIDOwner {
	RID self;

public:
	PhysXRIDOwner() = default;

	/**
	 * @brief Assign the RID. Must be called exactly once.
	 * Asserts that the RID has not already been set.
	 */
	void set_rid(const RID &p_rid) {
		DEV_ASSERT(!self.is_valid());
		self = p_rid;
	}

	/** @brief Returns the RID assigned to this object. */
	_FORCE_INLINE_ const RID &get_rid() const { return self; }

	/** @brief True if set_rid() has been called. */
	_FORCE_INLINE_ bool is_valid() const { return self.is_valid(); }
};
#endif // PHYSX_RID_OWNER_H