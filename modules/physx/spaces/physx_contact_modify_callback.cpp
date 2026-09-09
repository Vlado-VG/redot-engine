/**
 * @file physx_contact_modify_callback.cpp
 * @brief Implements Godot's material combiner via PhysX contact modification.
 */

#include "physx_contact_modify_callback.h"
#include "../shapes/physx_user_data.h"

#include "core/math/math_funcs.h"

// Reads the signed bounce/friction from each actor's userData and applies
// Godot's combine contract to every contact point in the pair:
//   combined_bounce   = CLAMP(bounceA + bounceB, 0, 1)
//   combined_friction = abs(min(frictionA, frictionB))
void PhysXContactModifyCallback::onContactModify(physx::PxContactModifyPair *p_pairs, physx::PxU32 p_count) {
    for (physx::PxU32 p = 0; p < p_count; p++) {
        physx::PxContactModifyPair &pair = p_pairs[p];

        // PxContactModifyPair::actor[] can reference deleted actors in an
        // async pipeline, but this engine steps synchronously (PxScene::simulate
        // blocks the main thread, and actors are only released from it between
        // steps), so the actors are guaranteed alive for the whole solve. The
        // null checks below guard the "no userData attached" case, not deletion.
        // Recover each side's signed bounce/friction from the actor userData.
        float bounce[2] = { 0.0f, 0.0f };
        float friction[2] = { 1.0f, 1.0f };
        for (int s = 0; s < 2; s++) {
            if (pair.actor[s] && pair.actor[s]->userData) {
                const auto *ud = static_cast<const PhysXActorUserData *>(pair.actor[s]->userData);
                bounce[s] = ud->bounce;
                friction[s] = ud->friction;
            }
        }

        // Godot's engine-wide material combiner (matches godot_physics_3d and
        // jolt_physics): bounce is summed then clamped (a negative/absorbent
        // value cancels the other side); friction is the minimum then absolute
        // (a negative/rough value behaves like its magnitude under min()).
        const float combined_bounce = CLAMP(bounce[0] + bounce[1], 0.0f, 1.0f);
        const float combined_friction = Math::abs(MIN(friction[0], friction[1]));

        physx::PxContactSet &contacts = pair.contacts;
        const physx::PxU32 nb = contacts.size();
        for (physx::PxU32 i = 0; i < nb; i++) {
            contacts.setRestitution(i, combined_bounce);
            contacts.setStaticFriction(i, combined_friction);
            contacts.setDynamicFriction(i, combined_friction);
        }
    }
}
