/**
 * @file physx_user_data.h
 * @brief userData bridge structures between PhysX and Godot.
 *
 * PhysX provides a raw `void* userData` field on PxActor and PxShape. This
 * module uses it to store a back-pointer to the Godot-side wrapper object so
 * that query results and simulation callbacks can recover the Godot RID and
 * ObjectID without maintaining a separate lookup table.
 *
 * Convention: every PxActor/PxShape created by this module MUST have its
 * userData set to the appropriate struct below. Consumers MUST cast to the
 * documented type — never to PhysXObject3D* or PhysXBody3D* directly.
 *
 * Note: PxShape::userData points at the shared PhysXShape3D* blueprint
 * (the same pointer returned by PhysXShape3D::create_shape's caller).
 * The shape_index for each PxShape is tracked in the owning body/area's
 * shapes[] array, not stored in the shape itself.
 */

#ifndef PHYSX_USER_DATA_H
#define PHYSX_USER_DATA_H

#include "core/object/object.h"
#include "core/templates/rid.h"

// physx_user_data.h is pulled into the simulation-event and direct-space-state
// translation units, both of which already include the full PhysX and shaped-
// object definitions transitively. Include them here so physx_resolve_shape_index
// (which calls PhysXShapedObject3D::find_shape_index) compiles regardless of
// preceding include order.
#include "../objects/physx_shaped_object_3d.h"
#include <PxPhysicsAPI.h>

class PhysXObject3D;

/**
 * @brief Attached to PxActor::userData.
 *
 * This is the single source of truth that bridges a PhysX actor back to
 * Godot-side data during queries (raycast, overlap, sweep) and simulation
 * callbacks (onContact, onTrigger).
 *
 * bounce/friction carry the Godot-side SIGNED values (negative = absorbent /
 * rough) so the contact-modify callback — which runs on a worker thread and
 * must not touch Godot objects — can implement Godot's material combiner. They
 * are written from the main thread before simulate() (in on_pre_step /
 * _apply_params_to_actor) and only read during simulate(), so plain floats are
 * safe under the synchronous step model.
 */
struct PhysXActorUserData {
	RID rid;                     ///< Godot RID of the owning body/area.
	ObjectID object_id;          ///< Godot ObjectID of the scene-tree node.
	PhysXObject3D *object = nullptr; ///< Back-pointer to the wrapper (for filter/callback use).
	float bounce = 0.0f;         ///< Signed bounce (negative = absorbent). Read by contact-modify.
	float friction = 1.0f;       ///< Signed friction (negative = rough). Read by contact-modify.
};

// Resolves a (actor, shape) query hit to the body-local shape index, or -1
// when the hit cannot be mapped (soft-body/articulation actors without
// userData, or a shape the object does not carry). -1 is the "invalid"
// sentinel: 0 is a legitimate shape index and must not double as the failure
// value.
// Both PhysXBody3D and PhysXArea3D are shaped objects; areas/bodies both
// support find_shape_index, so we check the type and cast accordingly.
static inline int physx_resolve_shape_index(const physx::PxRigidActor *p_actor,
                                             const physx::PxShape *p_shape) {
    if (!p_actor || !p_actor->userData || !p_shape) return -1;
    const auto *ad = static_cast<const PhysXActorUserData *>(p_actor->userData);
    if (!ad->object) return -1;
    // Bodies and areas are both shaped objects; areas/bodies both support this.
    if (ad->object->get_type() != PhysXObject3D::OBJECT_TYPE_BODY &&
        ad->object->get_type() != PhysXObject3D::OBJECT_TYPE_AREA) {
        return -1;
    }
    const PhysXShapedObject3D *shaped = static_cast<const PhysXShapedObject3D *>(ad->object);
    const int idx = shaped->find_shape_index(p_shape);
    return idx;
}

#endif // PHYSX_USER_DATA_H
