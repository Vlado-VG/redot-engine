/**
 * @file physx_space_3d.h
 * @brief A physics world — owns one PxScene and its supporting infrastructure.
 *
 * PhysXSpace3D represents a single physics simulation context. It owns:
 *   - A PxScene (the PhysX world)
 *   - A CPU dispatcher (worker threads)
 *   - A PhysXSimulationEventCallback (contacts, triggers, wake/sleep)
 *   - A PhysXDirectSpaceState3D (query interface)
 *
 * The step() method drives the simulation: pre-step hooks → simulate() →
 * fetchResults() → post-step hooks. In the current synchronous model, the
 * entire step blocks until physics is computed.
 */

#ifndef PHYSX_SPACE_3D_H
#define PHYSX_SPACE_3D_H
#include "servers/physics_3d/physics_server_3d.h"
#include "core/templates/local_vector.h"
#include "core/math/math_funcs.h"
// Forward declarations of PhysX types to keep the header clean
namespace physx {
class PxScene;
class PxActor;
class PxPhysics;
class PxDefaultCpuDispatcher;
class PxCudaContextManager;
}

class PhysXArea3D;
class PhysXObject3D;
class PhysXBody3D;
class PhysXJoint3D;
class PhysXDirectSpaceState3D;
class PhysXVehicle3D;
class PhysXVehicleSceneContext;
class PhysXSimulationEventCallback;
class PhysXShapedObject3D;
class PhysXSoftBody3D;
class PhysXPairFilterCallback;
class PhysXContactModifyCallback;
class PhysXGPUParticleFluid3D;
class PhysXGPUCloth3D;

class PhysXSpace3D {
public:
    PhysXSpace3D();
    ~PhysXSpace3D();

    // Simulation Loop
    void step(float p_step);
    bool is_stepping() const { return stepping; }
    float get_last_step() const { return last_step; }

    RID get_rid() const { return rid; }
    void set_rid(const RID &p_rid) { rid = p_rid; }

    bool is_active() const { return active; }
    void set_active(bool p_active);

    // Godot Space Parameters
    double get_param(PhysicsServer3D::SpaceParameter p_param) const;
    void set_param(PhysicsServer3D::SpaceParameter p_param, double p_value);

    int get_solver_iteration_count() const { return solver_iteration_count; }

    // --------------------------------------------------------------------
    // Sleep policy (Godot SpaceParameters, mapped onto PhysX's energy-based
    // sleep threshold + wake counter).
    // Godot expresses sleep as separate linear/angular velocity thresholds
    // (m/s and rad/s) plus a time-to-sleep; PhysX takes a mass-normalized
    // kinetic-energy threshold below which a body may sleep, plus a wake
    // counter in seconds.
    // --------------------------------------------------------------------
    real_t get_sleep_threshold_linear() const { return sleep_threshold_linear; }
    real_t get_sleep_threshold_angular() const { return sleep_threshold_angular; }
    real_t get_time_before_sleep() const { return time_before_sleep; }
    real_t get_sleep_energy_threshold() const {
        return 0.5 * (sleep_threshold_linear * sleep_threshold_linear + sleep_threshold_angular * sleep_threshold_angular);
    }

    PhysXDirectSpaceState3D *get_direct_state() { return direct_state; }

    // Actor Management
    void add_actor(physx::PxActor *p_actor);
    void remove_actor(physx::PxActor *p_actor);

    // PhysX-specific getter for helper classes (like DirectSpaceState)
    physx::PxScene *get_px_scene() const { return px_scene; }

    PhysXArea3D *get_default_area() const { return default_area; }
    void set_default_area(PhysXArea3D *p_area) { default_area = p_area; }

    // --------------------------------------------------------------------
    // Debug contacts — drives the "Visible Collision Shapes" contact overlay.
    //
    // space_set_debug_contacts() allocates the buffer; onContact appends up to
    // the buffer's capacity each step; the count is reset at the start of step.
    // Enabling debug contacts on any space flips a module-global flag so the
    // stateless simulation filter shader requests contact-point notifications
    // for all pairs (not just those with max_contacts_reported > 0).
    // Matches GodotPhysics' GodotSpace3D debug-contact semantics.
    // --------------------------------------------------------------------
    void set_debug_contacts(int p_amount);
    _FORCE_INLINE_ bool is_debugging_contacts() const { return !debug_contacts_buffer.is_empty(); }
    _FORCE_INLINE_ void add_debug_contact(const Vector3 &p_contact) {
        if (debug_contacts_count < (int)debug_contacts_buffer.size()) {
            debug_contacts_buffer.write[debug_contacts_count++] = p_contact;
        }
    }
    _FORCE_INLINE_ Vector<Vector3> get_debug_contacts() const { return debug_contacts_buffer; }
    _FORCE_INLINE_ int get_debug_contact_count() const { return debug_contacts_count; }

    // --------------------------------------------------------------------
    // Deferred monitor/event dispatch.
    //
    // onTrigger/onContact run inside fetchResults() during step(), i.e. outside
    // Godot's "safe" callback window. Godot expects Area3D monitor callbacks
    // (body_entered/area_entered/...) to fire during PhysicsServer3D::flush_queries,
    // which runs AFTER sync, with the world unlocked. We therefore record the
    // events here during step() and dispatch them in flush_queries().
    // --------------------------------------------------------------------

    /// A deferred area/body monitor event (from onTrigger).
    struct TriggerEvent {
        PhysXArea3D *area = nullptr;            ///< The trigger area (dispatches the callback).
        PhysXBody3D *body = nullptr;            ///< The other body, if this is a body-monitor event.
        PhysXArea3D *other_area = nullptr;      ///< The other area, if this is an area-vs-area event.
        int status = 0;                         ///< AREA_BODY_ADDED / AREA_BODY_REMOVED.
        int other_shape = 0;                    ///< Shape index of the other actor.
        int area_shape = 0;                     ///< Shape index of this area.
        bool is_area_vs_area = false;           ///< Selects area_monitor vs body_monitor dispatch.
        /// Identity is cached at queue time: the body/area may be freed before
        /// flush_queries() dispatches (body_exited must still fire), so the
        /// dispatch must never dereference body/other_area.
        RID body_rid;
        ObjectID body_id;
        RID other_area_rid;
        ObjectID other_area_id;
    };

    /// A deferred contact event (from onContact). Points at the body whose
    /// contact buffer was already filled during step(); nothing to do at flush
    /// time today, but the queue reserves the hook for state-sync dispatch.
    struct ContactEvent {
        PhysXBody3D *body = nullptr;
    };

    /// Records a trigger event for later dispatch (called by onTrigger).
    void queue_trigger(const TriggerEvent &p_event);
    /// Dispatches all queued trigger events and clears the queue. Called by
    /// PhysXServer3D::flush_queries().
    void flush_pending_callbacks();

    bool is_flushing_callbacks() const { return flushing_callbacks; }

    // --------------------------------------------------------------------
    // Body / Area registration
    // --------------------------------------------------------------------

    void register_body(PhysXBody3D *p_body);
    void unregister_body(PhysXBody3D *p_body);

    void register_area(PhysXArea3D *p_area);
    void unregister_area(PhysXArea3D *p_area);

    // --------------------------------------------------------------------
    // Vehicle registration
    // --------------------------------------------------------------------

    void register_vehicle(PhysXVehicle3D *p_vehicle);
    void unregister_vehicle(PhysXVehicle3D *p_vehicle);

    const LocalVector<PhysXVehicle3D *> &get_vehicles() const {
        return vehicles;
    }

    // --------------------------------------------------------------------
    // GPU fluid / cloth registration (PhysXGPUParticleFluid3D/PhysXGPUCloth3D)
    // --------------------------------------------------------------------

    void register_fluid(PhysXGPUParticleFluid3D *p_fluid);
    void unregister_fluid(PhysXGPUParticleFluid3D *p_fluid);
    void register_cloth(PhysXGPUCloth3D *p_cloth);
    void unregister_cloth(PhysXGPUCloth3D *p_cloth);

    const LocalVector<PhysXGPUParticleFluid3D *> &get_fluids() const {
        return fluids;
    }
    const LocalVector<PhysXGPUCloth3D *> &get_cloths() const {
        return cloths;
    }

    // --------------------------------------------------------------------
    // Soft body registration (PhysXSoftBody3D — the soft_body_* server API).
    // --------------------------------------------------------------------

    void register_soft_body(PhysXSoftBody3D *p_sb) {
        if (soft_bodies.find(p_sb) == -1) {
            soft_bodies.push_back(p_sb);
        }
    }
    void unregister_soft_body(PhysXSoftBody3D *p_sb) {
        int idx = soft_bodies.find(p_sb);
        if (idx != -1) {
            soft_bodies.remove_at(idx);
        }
    }

    // Convenience accessors for the GPU object types (resolve via the server).
    physx::PxPhysics *get_px_physics() const;
    physx::PxCudaContextManager *get_px_cuda() const;

    const LocalVector<PhysXBody3D *> &get_bodies() const {
        return bodies;
    }

    const LocalVector<PhysXArea3D *> &get_areas() const {
        return areas;
    }
private:
    // Core PhysX elements
    physx::PxScene *px_scene = nullptr;
    physx::PxDefaultCpuDispatcher *px_dispatcher = nullptr;

    // Simulation event callback (contacts, triggers, wake/sleep).
    PhysXSimulationEventCallback *event_callback = nullptr;

    // Pair filter callback (collision exceptions).
    PhysXPairFilterCallback *pair_filter_callback = nullptr;

    // Contact modify callback (Godot material combiner: absorbent/rough).
    PhysXContactModifyCallback *contact_modify_callback = nullptr;

    // Godot/Redot tracking
    PhysXDirectSpaceState3D *direct_state = nullptr;
    RID rid;
    bool active = false;
    bool stepping = false;
    float last_step = 0.0f;
	int solver_iteration_count = 8; // Godot default (physics/3d/solver/solver_iterations)

	// Godot sleep policy defaults (physics/3d/sleep_threshold_linear etc.).
	real_t sleep_threshold_linear = 0.1;
	real_t sleep_threshold_angular = Math::deg_to_rad(8.0);
	real_t time_before_sleep = 0.5;

    // Provides world-default gravity/damp
    PhysXArea3D *default_area = nullptr;

    // Vehicle scene context (batched road-geometry query buffer + tire-friction table).
    PhysXVehicleSceneContext *vehicle_scene_context = nullptr;

    // Object registration
    LocalVector<PhysXBody3D *> bodies;
    LocalVector<PhysXArea3D *> areas;
    LocalVector<PhysXVehicle3D *> vehicles;
    LocalVector<PhysXGPUParticleFluid3D *> fluids;
    LocalVector<PhysXGPUCloth3D *> cloths;
    LocalVector<PhysXSoftBody3D *> soft_bodies;

    // Deferred monitor events (filled during step, drained in flush_queries).
    LocalVector<TriggerEvent> pending_trigger_events;
    bool flushing_callbacks = false;

    // Debug-contact buffer for the "Visible Collision Shapes" overlay.
    // Reset (count = 0) at the start of each step; filled by onContact.
    Vector<Vector3> debug_contacts_buffer;
    int debug_contacts_count = 0;

    // Private helper to initialize/destroy the scene + vehicle context
    void _initialize_scene();
    void _terminate_scene();
    /// CPU soft bodies only: resolves the effective gravity from overlapping
    /// areas (AABB approximation of the reference's broadphase pairs) with the
    /// default area as the additive fallback — mirrors godot_physics_3d's
    /// GodotSoftBody3D::predict_motion. GPU deformables keep the scene gravity
    /// (no per-body gravity injection path).
    Vector3 _resolve_soft_body_gravity(const PhysXSoftBody3D &p_sb) const;
    /// Re-applies the sleep policy to every registered body (called when a
    /// sleep-related SpaceParameter changes).
    void _refresh_body_sleep_policies();
};
#endif // PHYSX_SPACE_3D_H