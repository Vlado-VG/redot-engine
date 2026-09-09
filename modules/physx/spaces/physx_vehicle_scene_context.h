/**
 * @file physx_vehicle_scene_context.h
 * @brief Per-scene context for vehicle2 — road-geometry query buffer + tire-friction table.
 *
 * PhysXVehicleSceneContext is owned by PhysXSpace3D. It is a STUB: it is
 * allocated per space but not yet consumed by anything. It is the designated
 * home for two planned vehicle features:
 *   - A batched road-geometry query buffer (64 vehicles x 4 wheels) for
 *     PxVehiclePhysXRoadGeometrySceneQueryComponent.
 *   - A tire-friction table mapping PxMaterial* to per-wheel friction values.
 *
 * Wire it up in PhysXVehicle3D::update()/post_step() when those features land.
 */

#ifndef PHYSX_VEHICLE_SCENE_CONTEXT_H
#define PHYSX_VEHICLE_SCENE_CONTEXT_H

#include "core/templates/local_vector.h"
#include "core/templates/hash_map.h"

#include "foundation/PxVec3.h"

// PhysX types are used only as pointers/keys here — forward-declare them
// instead of pulling in headers that don't live at the include root
// (e.g. PxConvexMesh.h is under geomutils/, not the include root).
namespace physx {
class PxRigidActor;
class PxShape;
class PxScene;
class PxMaterial;
class PxConvexMesh;
}

class PhysXVehicle3D;

/** Maximum number of vehicles that can be queried per space at once. */
static constexpr int VEHICLE_MAX_VEHICLES_PER_SPACE = 64;
/** Maximum wheels per vehicle (PhysX constant). */
static constexpr int VEHICLE_MAX_WHEELS = 4;

/**
 * @brief Batched road-geometry query result for a single wheel.
 *
 * Filled by PxVehiclePhysXRoadGeometryQueryUpdate during the scene query phase.
 * The batch is processed by the PhysXVehicleSceneContext and then read back
 * by each vehicle's PhysXActorEndComponent.
 */
struct PhysXVehicleRoadGeometryQueryResult {
    physx::PxRigidActor *actor = nullptr;   ///< The actor that got hit by the query.
    physx::PxShape *shape = nullptr;        ///< The shape that got hit by the query.
    physx::PxMaterial *material = nullptr;  ///< The material at the hit point.
    physx::PxVec3 hit_position = physx::PxVec3(0, 0, 0); ///< The hit position in world space.
};

class PhysXVehicleSceneContext {
public:
    PhysXVehicleSceneContext();
    ~PhysXVehicleSceneContext();

    /// Get the batched query results buffer (capacity = VEHICLE_MAX_VEHICLES_PER_SPACE x VEHICLE_MAX_WHEELS).
    const LocalVector<PhysXVehicleRoadGeometryQueryResult> &get_query_results() const { return query_results; }

    /// Get the tire-friction table.
    const HashMap<physx::PxMaterial *, float> &get_tire_friction_table() const { return tire_friction_table; }

    /// Add an entry to the tire-friction table.
    void set_tire_friction(physx::PxMaterial *p_material, float p_friction) {
        tire_friction_table[p_material] = p_friction;
    }

    /// Remove an entry from the tire-friction table.
    void remove_tire_friction(physx::PxMaterial *p_material) {
        tire_friction_table.erase(p_material);
    }

    /// Clear all query results (called before each scene query phase).
    void clear_query_results() { query_results.clear(); }

    /// True if the context has been initialized (has a scene pointer).
    bool is_valid() const { return px_scene != nullptr; }

    /// Get the PxScene this context belongs to.
    physx::PxScene *get_px_scene() const { return px_scene; }

    /// Get the unit cylinder sweep mesh (cached during initialization).
    const physx::PxConvexMesh *get_unit_cylinder_sweep_mesh() const { return unit_cylinder_sweep_mesh; }

private:
    physx::PxScene *px_scene = nullptr;
    const physx::PxConvexMesh *unit_cylinder_sweep_mesh = nullptr;

    /// Batched road-geometry query results (capacity = 64 x 4 = 256).
    LocalVector<PhysXVehicleRoadGeometryQueryResult> query_results;

    /// Tire friction mapping: PxMaterial* to friction value.
    HashMap<physx::PxMaterial *, float> tire_friction_table;
};

#endif // PHYSX_VEHICLE_SCENE_CONTEXT_H
