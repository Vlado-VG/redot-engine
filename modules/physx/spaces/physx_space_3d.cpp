/**
 * @file physx_space_3d.cpp
 * @brief Implementation of PhysXSpace3D — the physics world.
 */

#include "physx_space_3d.h"
#include "../physx_server.h"
#include "../physx_project_settings.h"
#include "../objects/physx_object_3d.h"
#include "../objects/physx_area_3d.h"
#include "../objects/physx_body_3d.h"
#include "../objects/physx_shaped_object_3d.h"
#include "../objects/physx_soft_body_3d.h"
#include "../joints/physx_joint_3d.h"
#include "physx_direct_space_state_3d.h"
#include "physx_filter_shader.h"
#include "physx_pair_filter_callback.h"
#include "physx_simulation_event_callback.h"
#include "physx_contact_modify_callback.h"
#include "../vehicles/physx_vehicle_3d.h"
#include "../objects/physx_gpu_cloth_3d.h"
#include "../objects/physx_gpu_particle_fluid_3d.h"

#include "PxPhysicsAPI.h"                // PhysX SDK
#include "extensions/PxDefaultCpuDispatcher.h"

#include "core/config/project_settings.h"
#include "core/os/os.h"

// Refcount of spaces currently recording debug contacts. The simulation filter
// shader is stateless, so it reads a module-global flag; we keep it accurate by
// counting how many spaces have a non-empty debug buffer.
static int s_debug_spaces_refcount = 0;

PhysXSpace3D::PhysXSpace3D() {
    pair_filter_callback = new PhysXPairFilterCallback();
    contact_modify_callback = new PhysXContactModifyCallback();
    event_callback = memnew(PhysXSimulationEventCallback(this));
    _initialize_scene();
    // Only build the query wrapper if the scene was created successfully;
    // otherwise direct_state stays null and queries safely early-out.
    if (px_scene) {
        direct_state = memnew(PhysXDirectSpaceState3D(this));
    }
}

PhysXSpace3D::~PhysXSpace3D() {
    // Detach soft bodies while this space (and its PxScene) is still valid --
    // they may outlive it if their RID is never explicitly freed. Runs before
    // _terminate_scene() so a GPU soft volume can still leave the scene.
    for (PhysXSoftBody3D *sb : soft_bodies) {
        sb->notify_space_destroyed();
    }
    soft_bodies.clear();
    if (direct_state) {
        memdelete(direct_state);
    }
    _terminate_scene();
    if (event_callback) {
        memdelete(event_callback);
        event_callback = nullptr;
    }
    if (pair_filter_callback) {
        // Allocated with plain `new` in the ctor (see :34), so free with
        // `delete` — not memdelete (CR-06: new must pair with delete).
        delete pair_filter_callback;
        pair_filter_callback = nullptr;
    }
    if (contact_modify_callback) {
        delete contact_modify_callback;
        contact_modify_callback = nullptr;
    }
    // Drop any deferred monitor events; their area/body pointers may be gone
    // by the time anyone could flush them.
    pending_trigger_events.clear();

    // Release this space's claim on the global debug-contacts flag.
    if (!debug_contacts_buffer.is_empty()) {
        debug_contacts_buffer.clear();
        if (--s_debug_spaces_refcount == 0) {
            g_physx_debug_contacts_enabled.store(false, std::memory_order_relaxed);
        }
    }
}

void PhysXSpace3D::_initialize_scene() {
    PhysXServer3D *server = PhysXServer3D::get_singleton();
    physx::PxPhysics *physics = server ? server->try_get_physics() : nullptr;
    ERR_FAIL_NULL_MSG(physics, "PhysX: server not initialized before space creation");

    // One CPU dispatcher is shared by every space (owned by the server, sized
    // once at init() from physics/physx_3d/simulation/cpu_worker_threads).
    px_dispatcher = server->get_cpu_dispatcher();
    ERR_FAIL_NULL_MSG(px_dispatcher, "PhysX: no shared CPU dispatcher (server init() failed?)");

    physx::PxSceneDesc scene_desc(physics->getTolerancesScale());

    // Gravity comes from project settings: a magnitude (physics/3d/default_gravity,
    // default 9.8) times a direction (physics/3d/default_gravity_vector, default
    // (0,-1,0)). Their product is the actual world gravity vector, which must
    // match the space's default-area gravity so that PhysX's native scene gravity
    // equals the resolved default gravity (PhysXBody3D::on_pre_step applies only
    // the area-override delta on top of it — zero in the default case).
    const real_t g = GLOBAL_GET("physics/3d/default_gravity");
    const Vector3 g_dir = GLOBAL_GET("physics/3d/default_gravity_vector");
    scene_desc.gravity = physx::PxVec3(
            (float)(g_dir.x * g),
            (float)(g_dir.y * g),
            (float)(g_dir.z * g));

    scene_desc.cpuDispatcher = px_dispatcher;

    // Simulation event callback (contacts, triggers, wake/sleep).
    scene_desc.simulationEventCallback = event_callback;

    // Custom filter shader: wraps PxDefaultSimulationFilterShader (which
    // implements Godot's layer/mask test) and additionally requests contact-
    // point notifications for shape pairs where either side has the
    // PHYSX_FILTER_FLAG_CONTACT_NOTIFY marker (set when a body has
    // max_contacts_reported > 0).
    // Returns eNOTIFY for simulation pairs so the filter callback gets invoked
    // to check collision exceptions.
    scene_desc.filterShader = physx_simulation_filter_shader;

    // Pair filter callback: enforces collision exceptions between bodies.
    // Runs after the filter shader; has access to both actors' userData.
    scene_desc.filterCallback = pair_filter_callback;

    // Contact modify callback: implements Godot's material combiner (absorbent /
    // rough) per-pair. Runs on worker threads during simulate(); reads only the
    // pre-step bounce/friction cached in each actor's userData.
    scene_desc.contactModifyCallback = contact_modify_callback;

    // Solver: PGS (PhysX's classic) by default; TGS is opt-in via
    // physics/physx_3d/simulation/solver_type — it holds joint chains steadier
    // under sustained external forces like wind, at some joint looseness cost
    // in large mixed piles.
    scene_desc.solverType = PhysXProjectSettings::solver_type == 1
            ? physx::PxSolverType::eTGS
            : physx::PxSolverType::ePGS;

    // Scene flags required for correct Godot integration:
    //   eENABLE_ACTIVE_ACTORS  - efficient body transform sync back to nodes
    //   eENABLE_CCD            - per-body continuous collision detection
    scene_desc.flags |= physx::PxSceneFlag::eENABLE_ACTIVE_ACTORS;
    scene_desc.flags |= physx::PxSceneFlag::eENABLE_CCD;

    if (PhysXProjectSettings::enhanced_determinism) {
        // Same-binary/same-platform determinism, independent of worker count
        // and API call order (not cross-platform). GPU dynamics is disabled
        // entirely in this mode (see PhysXServer3D::init()).
        scene_desc.flags |= physx::PxSceneFlag::eENABLE_ENHANCED_DETERMINISM;
    }

    // eENABLE_STABILIZATION is CPU-path only; PhysX rejects it alongside GPU
    // dynamics. See physx_project_settings.h for why it defaults to off here.
    if (PhysXProjectSettings::stabilization && !server->is_gpu_dynamics_enabled()) {
        scene_desc.flags |= physx::PxSceneFlag::eENABLE_STABILIZATION;
    }

    // GPU dynamics (GODOT_PHYSX_GPU build + usable CUDA device): the whole
    // scene simulates on the GPU. Buffer capacities sized for tens of
    // thousands of colliding rigid bodies plus a particle-contact budget;
    // PhysX grows some of these on demand but warns when the initial
    // capacity is exceeded.
    if (server->is_gpu_dynamics_enabled()) {
        physx::PxCudaContextManager *cuda = server->get_cuda_context();
        if (cuda) {
            scene_desc.cudaContextManager = cuda;
            scene_desc.flags |= physx::PxSceneFlag::eENABLE_GPU_DYNAMICS;
            scene_desc.broadPhaseType = physx::PxBroadPhaseType::eGPU;
            scene_desc.gpuMaxNumPartitions = 8;
            scene_desc.gpuDynamicsConfig.tempBufferCapacity = 64 * 1024 * 1024;
            scene_desc.gpuDynamicsConfig.maxRigidContactCount = 4 * 1024 * 1024;
            scene_desc.gpuDynamicsConfig.maxRigidPatchCount = 1024 * 1024;
            scene_desc.gpuDynamicsConfig.heapCapacity = 256 * 1024 * 1024;
            scene_desc.gpuDynamicsConfig.foundLostPairsCapacity = 4 * 1024 * 1024;
            scene_desc.gpuDynamicsConfig.collisionStackSize = 256 * 1024 * 1024;
            // Non-zero so PxDeformableSurface (cloth) / PxDeformableVolume (GPU
            // soft bodies) can generate contacts; either touching anything with
            // its budget at 0 silently drops the contacts and the deformable
            // falls through the world.
            scene_desc.gpuDynamicsConfig.maxDeformableSurfaceContacts = 512 * 1024;
            scene_desc.gpuDynamicsConfig.maxDeformableVolumeContacts = 1024 * 1024;
            scene_desc.gpuDynamicsConfig.maxParticleContacts = 1 * 1024 * 1024;
        }
    }

    px_scene = physics->createScene(scene_desc);
    ERR_FAIL_NULL_MSG(px_scene, "PhysX: createScene failed");
}

void PhysXSpace3D::_terminate_scene() {
    if (stepping && px_scene) {
        // The space died with a solve in flight (World3D freed between an
        // async step and its sync). Fetch so nothing releases into a live
        // solve, then let the flush below apply queued mutations.
        px_scene->fetchResults(true);
        stepping = false;
    }
    _flush_actor_ops();
    if (px_scene) {
        px_scene->release();
        px_scene = nullptr;
    }
    // NOTE: px_dispatcher is shared and owned by PhysXServer3D (released in
    // finish()); it is only borrowed here and must not be released per-space.
    px_dispatcher = nullptr;
}

void PhysXSpace3D::set_debug_contacts(int p_amount) {
    const bool was_debugging = !debug_contacts_buffer.is_empty();
    debug_contacts_buffer.resize(p_amount);
    const bool is_debugging = !debug_contacts_buffer.is_empty();

    // Keep the global refcount in sync so the filter shader requests contact
    // notifications exactly while at least one space is debugging.
    if (!was_debugging && is_debugging) {
        if (++s_debug_spaces_refcount == 1) {
            g_physx_debug_contacts_enabled.store(true, std::memory_order_relaxed);
        }
    } else if (was_debugging && !is_debugging) {
        if (--s_debug_spaces_refcount == 0) {
            g_physx_debug_contacts_enabled.store(false, std::memory_order_relaxed);
        }
    }
}

void PhysXSpace3D::step(float p_step) {
    if (!active || !px_scene) {
        return;
    }
    // PhysX forbids calling simulate() while a solve is in flight. A leftover
    // in-flight step can only happen when the async flag was flipped off
    // mid-flight or a sync was skipped — fetch defensively instead of dying.
    if (stepping) {
        WARN_PRINT_ONCE("PhysX: step() called with a solve in flight; fetching it first.");
        px_scene->fetchResults(true);
        stepping = false;
        _finish_step();
    }

    last_step = p_step;

    // Reset the debug-contact buffer for this step (onContact refills it).
    debug_contacts_count = 0;

    // Pre-step: apply constant forces, clear contact buffers, resolve area
    // gravity/damp overrides, and run custom integrators. Iterate the space's
    // own body registration list (maintained by register_body/unregister_body)
    // rather than querying the scene — this avoids the per-step getActors()
    // round-trip, has no 1024-actor cap, and skips the userData round-trip
    // since we already hold the typed wrapper.
    for (PhysXBody3D *body : bodies) {
        body->on_pre_step(p_step);
    }

    // Batched multi-fluid isosurface readback: with more than one fluid,
    // onPostSolve kicks each fluid's smoothing kernel WITHOUT syncing and the
    // sync + outlier clamp + extraction happen in finish_isosurface_extraction()
    // after fetchResults (below) -- syncing inline would serialize N GPU stalls
    // mid-solve, because PhysX invokes each fluid's onPostSolve in turn and
    // system 2's kernel would not be issued until system 1's callback finished
    // blocking. Single-fluid spaces keep the inline path.
    batched_isosurface = fluids.size() > 1;
    for (PhysXGPUParticleFluid3D *fluid : fluids) {
        fluid->set_deferred_extraction(batched_isosurface);
    }

    // Vehicle update (pre-step): read state from PhysX actor, apply commands,
    // write state back. This runs before simulate so the vehicle2 state is
    // consistent during the physics step.
    for (PhysXVehicle3D *vehicle : vehicles) {
        vehicle->update(p_step);
    }

    px_scene->simulate(p_step);

    // Synchronous mode (default): simulate + block in the same call — the
    // historic behavior. Async mode (physics/physx_3d/simulation/async_step):
    // return with the solve in flight; sync() — called by the engine at the
    // START of the next tick, before scripts — does the fetch and the whole
    // post-solve pipeline, so the solve overlaps the rest of this frame.
    // stepping marks the exact simulate→fetch span: pre-step hooks run before
    // it (queries there see the idle scene), and _finish_step runs after it is
    // cleared — _finish_step itself issues scene queries (CPU soft bodies), and
    // a re-entrant sync() while stepping is still set would recurse.
    stepping = true;
    px_scene->simulate(p_step);
    if (!PhysXServer3D::get_singleton()->is_async_stepping()) {
        px_scene->fetchResults(true);
        stepping = false;
        _finish_step();
    }
}

void PhysXSpace3D::sync() {
    if (!stepping || !px_scene) {
        return;
    }
    px_scene->fetchResults(true);
    // Clear stepping BEFORE _finish_step: the post pipeline issues scene
    // queries (CPU soft bodies), and a re-entrant sync() while stepping is
    // still set would fetch-and-recurse.
    stepping = false;
    _finish_step();
}

// Everything after fetchResults, shared by the sync and async paths. Runs on
// the server thread with the solve complete.
void PhysXSpace3D::_finish_step() {
    // Mutations queued during the in-flight window (deferred calls, node
    // destruction): apply removes/adds against the now-idle scene and run any
    // deferred releases. Flush BEFORE the readbacks so a mid-flight-destroyed
    // GPU object is out of the picture first.
    _flush_actor_ops();

    // GPU fluid/cloth read-back: copy particle/vertex positions GPU -> host so
    // the nodes can render them. Must run while the scene is still valid
    // (after fetchResults, before the next simulate). In batched mode the
    // finish pass completes each fluid's deferred isosurface extraction first
    // (sync + clamp + extraction kicks) -- every fluid's smoothing kernel was
    // kicked during the solve, so the first sync absorbs the whole batch.
    if (batched_isosurface) {
        for (PhysXGPUParticleFluid3D *fluid : fluids) {
            fluid->finish_isosurface_extraction();
        }
    }
    for (PhysXGPUParticleFluid3D *fluid : fluids) {
        fluid->read_back();
    }
    for (PhysXGPUCloth3D *cloth : cloths) {
        cloth->read_back();
    }
    // GPU soft bodies (PxDeformableVolume) simulated inside px_scene -- pull
    // their deformed state off the device. CPU soft bodies advance here, after
    // the rigid solve, so their per-vertex world query sees this step's final
    // rigid poses.
    for (PhysXSoftBody3D *sb : soft_bodies) {
        sb->read_back();
        Vector3 gravity;
        if (px_scene) {
            const physx::PxVec3 g = px_scene->getGravity();
            gravity = Vector3(g.x, g.y, g.z);
        }
        if (!sb->is_gpu()) {
            // Reference parity (GodotSoftBody3D::predict_motion): CPU soft
            // bodies resolve area gravity overrides. GPU deformables keep the
            // scene gravity — the solver has no per-body gravity injection.
            gravity = _resolve_soft_body_gravity(*sb);
        }
        sb->step(last_step, gravity);
    }

    // Post-step: derive kinematic velocities and fire state-sync callbacks so
    // Godot nodes read the new transforms.
    for (PhysXBody3D *body : bodies) {
        body->on_post_step(last_step);
    }

    // Vehicle post-step (post-step): sync the PhysX actor pose/velocity from
    // vehicle2 state after the simulation step is complete.
    for (PhysXVehicle3D *vehicle : vehicles) {
        vehicle->post_step(last_step);
    }

    // Active-actor count for INFO_ACTIVE_OBJECTS (reflects the solve that
    // just completed; in async mode it is read one phase later than the
    // historic inline path).
    if (px_scene) {
        physx::PxU32 nb_active = 0;
        px_scene->getActiveActors(nb_active);
        active_objects = (int)nb_active;
    }
}

void PhysXSpace3D::set_active(bool p_active) { active = p_active; }

void PhysXSpace3D::add_actor(physx::PxActor *p_actor) {
    ERR_FAIL_NULL(p_actor);
    ERR_FAIL_NULL(px_scene);
    if (stepping) {
        // Async in-flight window: PhysX forbids scene mutation during the
        // solve. Apply right after the fetch (see _flush_actor_ops).
        pending_actor_ops.push_back({ p_actor, true, false, false });
        return;
    }
    px_scene->addActor(*p_actor);
}

void PhysXSpace3D::remove_actor(physx::PxActor *p_actor) {
    ERR_FAIL_NULL(p_actor);
    // Tolerate a null scene silently: during teardown a body/area destructor
    // may run after the space's PxScene was already released (e.g. the World3D
    // is freed before the body nodes). There's nothing to remove in that case.
    if (!px_scene) {
        return;
    }
    if (stepping) {
        pending_actor_ops.push_back({ p_actor, false, true, false });
        return;
    }
    px_scene->removeActor(*p_actor);
}

// Takes ownership of a PxActor release: while a solve is in flight the
// release is queued until after the fetch (the solve still references the
// actor), otherwise it runs inline. Wrappers that destroy their actor route
// through this so a mid-flight node free never releases into a live solve.
void PhysXSpace3D::release_actor(physx::PxActor *p_actor) {
    ERR_FAIL_NULL(p_actor);
    if (stepping && px_scene) {
        pending_actor_ops.push_back({ p_actor, false, false, true });
        return;
    }
    p_actor->release();
}

void PhysXSpace3D::_flush_actor_ops() {
    for (const PendingActorOp &op : pending_actor_ops) {
        if (op.add && px_scene) {
            px_scene->addActor(*op.actor);
        } else if (op.remove && px_scene) {
            px_scene->removeActor(*op.actor);
        }
        if (op.release) {
            op.actor->release();
        }
    }
    pending_actor_ops.clear();
}

void PhysXSpace3D::set_param(PhysicsServer3D::SpaceParameter p_param, double p_value) {
    if (!px_scene) {
        return;
    }

    switch (p_param) {
        case PhysicsServer3D::SPACE_PARAM_SOLVER_ITERATIONS: {
            solver_iteration_count = (int)p_value;
            for (PhysXBody3D *body : bodies) {
                physx::PxRigidActor *actor = body->get_px_actor();
                if (actor && actor->is<physx::PxRigidDynamic>()) {
                    physx::PxRigidDynamic *dyn = static_cast<physx::PxRigidDynamic *>(actor);
                    dyn->setSolverIterationCounts(solver_iteration_count, solver_iteration_count);
                }
            }
        } break;
        case PhysicsServer3D::SPACE_PARAM_BODY_LINEAR_VELOCITY_SLEEP_THRESHOLD: {
            sleep_threshold_linear = p_value;
            _refresh_body_sleep_policies();
        } break;
        case PhysicsServer3D::SPACE_PARAM_BODY_ANGULAR_VELOCITY_SLEEP_THRESHOLD: {
            sleep_threshold_angular = p_value;
            _refresh_body_sleep_policies();
        } break;
        case PhysicsServer3D::SPACE_PARAM_BODY_TIME_TO_SLEEP: {
            time_before_sleep = p_value;
            _refresh_body_sleep_policies();
        } break;
        case PhysicsServer3D::SPACE_PARAM_CONTACT_RECYCLE_RADIUS: {
            // No PhysX counterpart: PhysX's contact-offset/bias machinery
            // supersedes Godot's pair-recycling knob. Cached for get_param.
            param_contact_recycle_radius = p_value;
            WARN_PRINT_ONCE("PhysX: CONTACT_RECYCLE_RADIUS is intentionally unmapped (cached for get_param only).");
        } break;
        case PhysicsServer3D::SPACE_PARAM_CONTACT_MAX_SEPARATION: {
            // The PhysX-native knob for contact generation distance is the
            // per-shape contactOffset (driven by the shape margin); a
            // per-space separation has no clean mapping without disturbing
            // it. Cached for get_param only.
            param_contact_max_separation = p_value;
        } break;
        case PhysicsServer3D::SPACE_PARAM_CONTACT_MAX_ALLOWED_PENETRATION: {
            // Godot: contacts with penetration < allowed produce no
            // positional correction, so bodies rest at `allowed` penetration.
            // PhysX bodies rest at the SUM of a pair's rest offsets — apply
            // half per shape. Only an explicit set maps the parameter; the
            // Godot default (0.01) stays unmapped so default resting
            // behavior is unchanged. Shapes attached afterwards pick the
            // value up (see PhysXShapedObject3D's rest-offset application).
            param_contact_max_allowed_penetration = p_value;
            allowed_penetration_set = true;
            shape_rest_offset = MAX(0.0, p_value) * 0.5;
        } break;
        case PhysicsServer3D::SPACE_PARAM_CONTACT_DEFAULT_BIAS: {
            // No PhysX counterpart: solver bias is a global solver property.
            // Cached for get_param only.
            param_contact_default_bias = p_value;
            WARN_PRINT_ONCE("PhysX: CONTACT_DEFAULT_BIAS is intentionally unmapped (cached for get_param only).");
        } break;
        default: {
            WARN_PRINT_ONCE("PhysX: unknown SpaceParameter, ignored.");
        } break;
    }
}

double PhysXSpace3D::get_param(PhysicsServer3D::SpaceParameter p_param) const {
    switch (p_param) {
        case PhysicsServer3D::SPACE_PARAM_SOLVER_ITERATIONS:
            return static_cast<double>(solver_iteration_count);
        case PhysicsServer3D::SPACE_PARAM_BODY_LINEAR_VELOCITY_SLEEP_THRESHOLD:
            return sleep_threshold_linear;
        case PhysicsServer3D::SPACE_PARAM_BODY_ANGULAR_VELOCITY_SLEEP_THRESHOLD:
            return sleep_threshold_angular;
        case PhysicsServer3D::SPACE_PARAM_BODY_TIME_TO_SLEEP:
            return time_before_sleep;
        case PhysicsServer3D::SPACE_PARAM_CONTACT_RECYCLE_RADIUS:
            return param_contact_recycle_radius;
        case PhysicsServer3D::SPACE_PARAM_CONTACT_MAX_SEPARATION:
            return param_contact_max_separation;
        case PhysicsServer3D::SPACE_PARAM_CONTACT_MAX_ALLOWED_PENETRATION:
            return param_contact_max_allowed_penetration;
        case PhysicsServer3D::SPACE_PARAM_CONTACT_DEFAULT_BIAS:
            return param_contact_default_bias;
        default:
            return 0.0;
    }
}

void PhysXSpace3D::_refresh_body_sleep_policies() {
    for (PhysXBody3D *body : bodies) {
        body->refresh_sleep_policy();
    }
}

// --------------------------------------------------------------------
// Body / Area registration
// --------------------------------------------------------------------

void PhysXSpace3D::register_body(PhysXBody3D *p_body) {
    ERR_FAIL_NULL(p_body);

    if (bodies.find(p_body) != -1) {
        return;
    }

    bodies.push_back(p_body);

    // Apply cached solver iterations to newly registered bodies.
    physx::PxRigidActor *actor = p_body->get_px_actor();
    if (actor && actor->is<physx::PxRigidDynamic>()) {
        physx::PxRigidDynamic *dyn = static_cast<physx::PxRigidDynamic *>(actor);
        dyn->setSolverIterationCounts(solver_iteration_count, solver_iteration_count);
    }
    // Adopt this space's sleep policy (thresholds may differ from the previous
    // space, and the body was created before it had one).
    p_body->refresh_sleep_policy();
}

void PhysXSpace3D::unregister_body(PhysXBody3D *p_body) {
    ERR_FAIL_NULL(p_body);

    for (unsigned int i = 0; i < bodies.size(); i++) {
        if (bodies[i] == p_body) {
            bodies.remove_at(i);
            return;
        }
    }
}

void PhysXSpace3D::register_area(PhysXArea3D *p_area) {
    ERR_FAIL_NULL(p_area);

    if (areas.find(p_area) != -1) {
        return;
    }

    areas.push_back(p_area);
}

void PhysXSpace3D::unregister_area(PhysXArea3D *p_area) {
    ERR_FAIL_NULL(p_area);

    for (unsigned int i = 0; i < areas.size(); i++) {
        if (areas[i] == p_area) {
            areas.remove_at(i);
            return;
        }
    }
}

// --------------------------------------------------------------------
// Vehicle registration
// --------------------------------------------------------------------

void PhysXSpace3D::register_vehicle(PhysXVehicle3D *p_vehicle) {
    ERR_FAIL_NULL(p_vehicle);

    if (vehicles.find(p_vehicle) != -1) {
        return;
    }

    vehicles.push_back(p_vehicle);
}

void PhysXSpace3D::unregister_vehicle(PhysXVehicle3D *p_vehicle) {
    ERR_FAIL_NULL(p_vehicle);

    for (unsigned int i = 0; i < vehicles.size(); i++) {
        if (vehicles[i] == p_vehicle) {
            vehicles.remove_at(i);
            return;
        }
    }
}

// ---------------------------------------------------------------------------
// GPU fluid / cloth registration
// ---------------------------------------------------------------------------

void PhysXSpace3D::register_fluid(PhysXGPUParticleFluid3D *p_fluid) {
    ERR_FAIL_NULL(p_fluid);
    if (fluids.find(p_fluid) != -1) {
        return;
    }
    fluids.push_back(p_fluid);
}

void PhysXSpace3D::unregister_fluid(PhysXGPUParticleFluid3D *p_fluid) {
    ERR_FAIL_NULL(p_fluid);
    for (unsigned int i = 0; i < fluids.size(); i++) {
        if (fluids[i] == p_fluid) {
            fluids.remove_at(i);
            return;
        }
    }
}

void PhysXSpace3D::register_cloth(PhysXGPUCloth3D *p_cloth) {
    ERR_FAIL_NULL(p_cloth);
    if (cloths.find(p_cloth) != -1) {
        return;
    }
    cloths.push_back(p_cloth);
}

void PhysXSpace3D::unregister_cloth(PhysXGPUCloth3D *p_cloth) {
    ERR_FAIL_NULL(p_cloth);
    for (unsigned int i = 0; i < cloths.size(); i++) {
        if (cloths[i] == p_cloth) {
            cloths.remove_at(i);
            return;
        }
    }
}

physx::PxPhysics *PhysXSpace3D::get_px_physics() const {
    PhysXServer3D *server = PhysXServer3D::get_singleton();
    return server ? server->try_get_physics() : nullptr;
}

physx::PxCudaContextManager *PhysXSpace3D::get_px_cuda() const {
    PhysXServer3D *server = PhysXServer3D::get_singleton();
    return server ? server->get_cuda_context() : nullptr;
}

// ---------------------------------------------------------------------------
// Deferred monitor/event dispatch.
//
// onTrigger/onContact run inside fetchResults() during step(), i.e. on the
// physics thread and outside Godot's unlocked callback window. Area3D monitor
// callbacks (body_entered/area_entered/...) must fire during flush_queries()
// instead, so onTrigger records a TriggerEvent here and flush_queries()
// dispatches it. The overlap-list bookkeeping (add_overlapping_body/area) still
// happens during onTrigger because that state is needed for the next step's
// gravity/damp resolution — only the *Godot callbacks* are deferred.
// ---------------------------------------------------------------------------

void PhysXSpace3D::queue_trigger(const TriggerEvent &p_event) {
    pending_trigger_events.push_back(p_event);
}

void PhysXSpace3D::flush_pending_callbacks() {
    // Dispatch the trigger events queued during the last step. Reentrancy guard:
    // a monitor callback could in theory mutate the world, but it cannot re-enter
    // flush_queries() (it runs on the main thread, single-threaded server).
    flushing_callbacks = true;

    for (unsigned int i = 0; i < pending_trigger_events.size(); i++) {
        const TriggerEvent &ev = pending_trigger_events[i];
        // Skip events whose dispatching area was freed between step() and now.
        // The body/area identity is cached in the event, so nothing else is
        // dereferenced here — removal events MUST still dispatch after the
        // body/area has been freed (Godot emits body_exited on removal).
        if (!ev.area || areas.find(ev.area) == -1) {
            continue;
        }
        if (ev.is_area_vs_area) {
            ev.area->dispatch_area_monitor(
                    ev.status,
                    ev.other_area_rid,
                    ev.other_area_id,
                    ev.other_shape,
                    ev.area_shape);
        } else {
            ev.area->dispatch_body_monitor(
                    ev.status,
                    ev.body_rid,
                    ev.body_id,
                    ev.other_shape,
                    ev.area_shape);
        }
    }

    pending_trigger_events.clear();
    flushing_callbacks = false;
}

Vector3 PhysXSpace3D::_resolve_soft_body_gravity(const PhysXSoftBody3D &p_sb) const {
	// Mirror of godot_physics_3d GodotSoftBody3D::predict_motion's gravity
	// resolution: overlapping areas resolve by priority (highest first) with
	// the per-channel override modes, and the space's default area is the
	// additive fallback. The reference tracks true broadphase pairs; here the
	// soft body's world AABB vs each area's shape AABB approximates the overlap.
	const AABB bounds = p_sb.get_bounds();

	LocalVector<const PhysXArea3D *> overlapping;
	for (const PhysXArea3D *area : areas) {
		const physx::PxBounds3 pb = area->get_world_bounds();
		if (pb.isEmpty()) {
			continue;
		}
		const AABB area_bounds(
				Vector3(pb.minimum.x, pb.minimum.y, pb.minimum.z),
				Vector3(pb.maximum.x - pb.minimum.x, pb.maximum.y - pb.minimum.y, pb.maximum.z - pb.minimum.z));
		if (area_bounds.intersects(bounds)) {
			overlapping.push_back(area);
		}
	}

	// Insertion sort by priority ascending; iterate descending (highest first)
	// — same ordering the rigid-body pre-step uses.
	for (int i = 1; i < (int)overlapping.size(); i++) {
		const PhysXArea3D *key = overlapping[i];
		int j = i - 1;
		while (j >= 0 && overlapping[j]->get_priority() > key->get_priority()) {
			overlapping[j + 1] = overlapping[j];
			j--;
		}
		overlapping[j + 1] = key;
	}

	const Vector3 center = bounds.get_center();
	Vector3 gravity;
	bool gravity_done = false;
	for (int i = (int)overlapping.size() - 1; i >= 0 && !gravity_done; i--) {
		const PhysXArea3D *area = overlapping[i];
		gravity_done = physx_apply_area_override(gravity, area->get_gravity_override_mode(),
				[&] { return physx_area_gravity_at(*area, center); });
	}
	if (!gravity_done && default_area) {
		gravity += physx_area_gravity_at(*default_area, center);
	}
	return gravity;
}
