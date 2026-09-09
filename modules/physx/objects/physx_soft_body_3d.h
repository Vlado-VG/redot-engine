#ifndef PHYSX_SOFT_BODY_3D_H
#define PHYSX_SOFT_BODY_3D_H

#include "core/math/aabb.h"
#include "core/math/transform_3d.h"
#include "core/object/object_id.h"
#include "core/variant/variant.h"
#include "core/templates/hash_set.h"
#include "servers/physics_server_3d.h"

#include "physx_object_3d.h"
#include "../shapes/physx_user_data.h"

// Forward declarations. PhysX 5's soft body is PxDeformableVolume; the
// PxSoftBody name survives only as a deprecated typedef.
namespace physx {
class PxActor;
class PxDeformableVolume;
class PxCudaContextManager;
}

/**
 * @brief Skeleton for the soft-body implementation (Godot SoftBody3D).
 *
 * STATUS: skeleton. The Godot-side lifecycle is complete and safe — RID
 * registration, space membership (works for any PxActor once one exists),
 * collision layer/mask, collision exceptions, parameter storage and
 * round-trip getters. No PhysX actor is created and nothing is simulated yet;
 * the unimplemented entry points are marked below.
 *
 * Wiring it up (API names verified against thirdparty/physx 5.8 headers):
 *
 *  1. PxDeformableVolume is GPU-ONLY (see PxDeformableVolume.h): the scene
 *     needs PxSceneFlag::eENABLE_GPU_DYNAMICS and PxBroadPhaseType::eGPU.
 *     Add both to PhysXSpace3D::_initialize_scene() behind a project setting.
 *  2. soft_body_set_mesh(): cook the mesh into a tetrahedral
 *     PxDeformableVolumeMesh (PxCooking), then load it via
 *     PxPhysics::createDeformableVolumeMesh(PxInputStream&).
 *  3. Create the actor and attach its collision shape via
 *     PxDeformableBody::attachShape, then store it as px_actor — set_space()
 *     below already adds/removes a non-null actor from the PxScene.
 *  4. Rendering sync / kinematic targets: per-step read
 *     PxDeformableVolume::getSimPositionInvMassBufferD() and write
 *     setKinematicTargetBufferD() (device buffers; a PxCudaContextManager
 *     is required, threaded through the space like the vehicle context).
 *  5. Map the stored params onto PxDeformableVolumeMaterial /
 *     solver iteration counts; implement pinned points with
 *     PxDeformableAttachment (pinned_indices below is the Godot-side set).
 *
 * Related future features (same GPU pipeline): PxDeformableSurface (cloth),
 * PxParticleBuffer (particles), PxArticulationReducedCoordinate
 * (articulations) — all follow this same wrapper pattern.
 */
class PhysXSoftBody3D : public PhysXObject3D {
public:
	PhysXSoftBody3D();
	~PhysXSoftBody3D() override;

	// --- Space Management (overrides PhysXObject3D pure virtual) ---
	virtual void set_space(PhysXSpace3D *p_space) override;

	// --- Transform & Bounds ---
	void set_transform(const Transform3D &p_transform);
	Transform3D get_transform() const;

	void set_bounds(const AABB &p_bounds);
	AABB get_bounds() const;

	// --- Collision Filtering (inherited set_collision_layer/set_collision_mask) ---
	// _update_shapes() override handles propagation to PhysX.

	// --- Ray Pickable ---
	void set_ray_pickable(bool p_enable);
	bool is_ray_pickable() const;

	// --- Collision Exceptions ---
	void add_collision_exception(const RID &p_excepted_body);
	void remove_collision_exception(const RID &p_excepted_body);
	void get_collision_exceptions(List<RID> *p_exceptions) const;
	const HashSet<RID> &get_collision_exception_set() const;

	// --- State ---
	void set_state(PhysicsServer3D::BodyState p_state, const Variant &p_variant);
	Variant get_state(PhysicsServer3D::BodyState p_state) const;

	// --- Soft Body Parameters (stored; TODO: apply to the deformable) ---
	void set_mass(real_t p_mass);
	real_t get_mass() const;

	void set_linear_stiffness(real_t p_stiffness);
	real_t get_linear_stiffness() const;

	void set_pressure_coefficient(real_t p_pressure);
	real_t get_pressure_coefficient() const;

	void set_damping_coefficient(real_t p_damping);
	real_t get_damping_coefficient() const;

	void set_drag_coefficient(real_t p_drag);
	real_t get_drag_coefficient() const;

	void set_simulation_precision(int p_precision);
	int get_simulation_precision() const;

	// --- PhysX Access ---
	physx::PxActor *get_px_actor() const;
	physx::PxDeformableVolume *get_px_deformable() const;

	/// Re-syncs actor_user_data with the soft body's current RID/ObjectID.
	void refresh_user_data();

	/// Visual mesh RID (assigned via soft_body_set_mesh; TODO: cook from it).
	void set_mesh_rid(RID p_mesh) { mesh_rid = p_mesh; }
	RID get_mesh_rid() const { return mesh_rid; }

	/// Point indices pinned via soft_body_pin_point (TODO: PxDeformableAttachment).
	void pin_point(int p_point_index, bool p_pin);
	bool is_point_pinned(int p_point_index) const { return pinned_indices.has(p_point_index); }
	void clear_pinned_points() { pinned_indices.clear(); }

	// ------------------------------------------------------------------
	// FEM SKELETON (PhysX 5.10). A first, honest slice of the deformable-
	// volume pipeline built on PxDeformableVolumeExt:
	//
	//   build_fem_box(size, voxels): voxelizes an axis-aligned box through
	//     PxDeformableVolumeExt::createDeformableVolumeBox, attaches the
	//     actor to the current space (GPU dynamics required) and allocates
	//     pinned host mirrors for the simulation positions/velocities.
	//   get_fem_positions(): on-demand GPU -> host readback of the collision
	//     positions (the points that match the render surface).
	//
	// Not wired yet (TODO, in rough order): parameter mapping (the stored
	// mass/stiffness/damping values belong on PxDeformableVolumeMaterial and
	// the solver iteration counts), PxDeformableAttachment for pinned
	// points, kinematic targets, a proper tet-mesh path for arbitrary
	// meshes (createDeformableVolumeMesh), and per-step automatic readback
	// (readback is currently pull-based for the lab).
	// ------------------------------------------------------------------
	bool build_fem_box(const Vector3 &p_size, int p_voxels = 10);
	bool is_fem_built() const { return px_actor != nullptr; }
	int get_fem_point_count() const { return fem_point_count; }
	/// On-demand GPU -> host readback of the current collision positions.
	Vector<Vector3> get_fem_positions();

protected:
	virtual void _update_shapes() override;

private:
	/// Bridges PxActor->userData back to this object's RID/ObjectID/type.
	PhysXActorUserData actor_user_data;

	physx::PxActor *px_actor = nullptr;

	// FEM skeleton state (valid after build_fem_box).
	physx::PxCudaContextManager *fem_cuda = nullptr;
	void *fem_sim_pos_pinned = nullptr;  // PxVec4* pinned host mirror (sim pos/inv-mass)
	void *fem_sim_vel_pinned = nullptr;  // PxVec4* pinned host mirror (sim velocity)
	void *fem_coll_pos_pinned = nullptr; // PxVec4* pinned host mirror (collision pos/inv-mass)
	void *fem_rest_pinned = nullptr;     // PxVec4* pinned host mirror (rest positions)
	int fem_point_count = 0;
	bool fem_simulated_once = false;

	Transform3D transform;
	AABB bounds;

	// Visual mesh (cooked into a PxDeformableVolumeMesh in a future phase).
	RID mesh_rid;

	/// Godot 4 SoftBody properties
	real_t mass = 1.0;
	real_t linear_stiffness = 0.5;
	real_t pressure_coefficient = 0.0;
	real_t damping_coefficient = 0.01;
	real_t drag_coefficient = 0.0;
	int simulation_precision = 5;

	bool ray_pickable = false;

	/// Pinned point indices (applied via PxDeformableAttachment later).
	HashSet<int> pinned_indices;

	/// Collision exceptions (consulted by query filter callback).
	HashSet<RID> collision_exceptions;
};
#endif // PHYSX_SOFT_BODY_3D_H
