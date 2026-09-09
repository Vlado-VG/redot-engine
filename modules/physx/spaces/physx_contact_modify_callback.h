/**
 * @file physx_contact_modify_callback.h
 * @brief PxContactModifyCallback implementing Godot's material combiner.
 *
 * Godot's engine-wide material combine contract (shared by godot_physics_3d and
 * jolt_physics) is:
 *   combined_bounce    = CLAMP(bounceA + bounceB, 0, 1)
 *   combined_friction  = abs(min(frictionA, frictionB))
 *
 * The negative-sign trick (PhysicsMaterial::computed_bounce returns -bounce when
 * "absorbent", computed_friction returns -friction when "rough") is how Godot
 * signals "drive the combined value toward zero / toward the minimum": a
 * negative bounce cancels the other side's bounce, a negative friction is
 * equivalent to its absolute value under min().
 *
 * PhysX 5 cannot express "sum, clamped" with its built-in PxCombineMode enum
 * (eAVERAGE/eMIN/eMULTIPLY/eMAX), AND it misinterprets a negative restitution
 * as a compliant (spring-damper) contact — which is why an absorbent body
 * falls through the floor. The contact-modify callback is PhysX's sanctioned
 * mechanism for custom per-pair material combining: it runs after narrowphase
 * computes contacts but before the solver, letting us overwrite the per-contact
 * restitution/friction with the Godot-combined values.
 *
 * The body's SIGNED bounce/friction are read from the PxActor userData (which
 * bridges back to PhysXBody3D). Bodies clamp to [0,1] when writing to their
 * PxMaterial so no negative restitution ever reaches PhysX's compliant path.
 */

#ifndef PHYSX_CONTACT_MODIFY_CALLBACK_H
#define PHYSX_CONTACT_MODIFY_CALLBACK_H

#include "PxContactModifyCallback.h"

class PhysXContactModifyCallback : public physx::PxContactModifyCallback {
public:
    virtual void onContactModify(physx::PxContactModifyPair *p_pairs, physx::PxU32 p_count) override;
};

#endif // PHYSX_CONTACT_MODIFY_CALLBACK_H
