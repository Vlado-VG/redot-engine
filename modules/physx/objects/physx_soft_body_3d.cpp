#include "physx_soft_body_3d.h"
#include "../physx_server.h"
#include "../physx_conversions.h"
#include "../spaces/physx_space_3d.h"

#include "core/error/error_macros.h"

#include "PxPhysicsAPI.h"
#include "extensions/PxCudaHelpersExt.h"
#include "extensions/PxDeformableVolumeExt.h"
#include "geometry/PxTetrahedronMesh.h"

// ============================================================================
// Lifecycle
// ============================================================================

PhysXSoftBody3D::PhysXSoftBody3D()
		: PhysXObject3D(OBJECT_TYPE_SOFT_BODY) {
	// px_actor stays null until the deformable volume is created (see the
	// header's wiring notes). All Godot-side state below is stored eagerly so
	// the server API round-trips even before simulation exists.
}

PhysXSoftBody3D::~PhysXSoftBody3D() {
	if (space) {
		set_space(nullptr);
	}

	if (px_actor) {
		px_actor->release();
		px_actor = nullptr;
	}
	if (fem_cuda) {
		if (fem_sim_pos_pinned) {
			physx::PxVec4 *p = static_cast<physx::PxVec4 *>(fem_sim_pos_pinned);
			PX_EXT_PINNED_MEMORY_FREE(*fem_cuda, p);
			fem_sim_pos_pinned = nullptr;
		}
		if (fem_sim_vel_pinned) {
			physx::PxVec4 *p = static_cast<physx::PxVec4 *>(fem_sim_vel_pinned);
			PX_EXT_PINNED_MEMORY_FREE(*fem_cuda, p);
			fem_sim_vel_pinned = nullptr;
		}
		if (fem_coll_pos_pinned) {
			physx::PxVec4 *p = static_cast<physx::PxVec4 *>(fem_coll_pos_pinned);
			PX_EXT_PINNED_MEMORY_FREE(*fem_cuda, p);
			fem_coll_pos_pinned = nullptr;
		}
		if (fem_rest_pinned) {
			physx::PxVec4 *p = static_cast<physx::PxVec4 *>(fem_rest_pinned);
			PX_EXT_PINNED_MEMORY_FREE(*fem_cuda, p);
			fem_rest_pinned = nullptr;
		}
	}
}

// ============================================================================
// Space Management
// ============================================================================

void PhysXSoftBody3D::set_space(PhysXSpace3D *p_space) {
	if (space == p_space) {
		return;
	}

	if (space && px_actor) {
		space->remove_actor(px_actor);
	}

	space = p_space;

	if (space && px_actor) {
		space->add_actor(px_actor);
	}
}

// ============================================================================
// Transform & Bounds
// ============================================================================

void PhysXSoftBody3D::set_transform(const Transform3D &p_transform) {
	transform = p_transform;

	// TODO: apply to the PxDeformableVolume once it exists (the actor-level
	// pose API or the kinematic target buffer, depending on attachment mode).
}

Transform3D PhysXSoftBody3D::get_transform() const {
	return transform;
}

void PhysXSoftBody3D::set_bounds(const AABB &p_bounds) {
	bounds = p_bounds;
}

AABB PhysXSoftBody3D::get_bounds() const {
	return bounds;
}

// ============================================================================
// Ray Pickable
// ============================================================================

void PhysXSoftBody3D::set_ray_pickable(bool p_enable) {
	ray_pickable = p_enable;
}

bool PhysXSoftBody3D::is_ray_pickable() const {
	return ray_pickable;
}

// ============================================================================
// Collision Exceptions
// ============================================================================

void PhysXSoftBody3D::add_collision_exception(const RID &p_excepted_body) {
	collision_exceptions.insert(p_excepted_body);
}

void PhysXSoftBody3D::remove_collision_exception(const RID &p_excepted_body) {
	collision_exceptions.erase(p_excepted_body);
}

void PhysXSoftBody3D::get_collision_exceptions(List<RID> *p_exceptions) const {
	for (const RID &rid : collision_exceptions) {
		p_exceptions->push_back(rid);
	}
}

const HashSet<RID> &PhysXSoftBody3D::get_collision_exception_set() const {
	return collision_exceptions;
}

// ============================================================================
// State
// ============================================================================

void PhysXSoftBody3D::set_state(PhysicsServer3D::BodyState p_state, const Variant &p_variant) {
	switch (p_state) {
		case PhysicsServer3D::BODY_STATE_TRANSFORM:
			set_transform(p_variant);
			break;
		default:
			break;
	}
}

Variant PhysXSoftBody3D::get_state(PhysicsServer3D::BodyState p_state) const {
	switch (p_state) {
		case PhysicsServer3D::BODY_STATE_TRANSFORM:
			return get_transform();
		default:
			return Variant();
	}
}

// ============================================================================
// Soft Body Parameters — stored for round-trip; TODO: apply to the
// PxDeformableVolumeMaterial / solver once simulation exists.
// ============================================================================

void PhysXSoftBody3D::set_mass(real_t p_mass) {
	mass = MAX(p_mass, 0.001f);
}

real_t PhysXSoftBody3D::get_mass() const {
	return mass;
}

void PhysXSoftBody3D::set_linear_stiffness(real_t p_stiffness) {
	linear_stiffness = p_stiffness;
}

real_t PhysXSoftBody3D::get_linear_stiffness() const {
	return linear_stiffness;
}

void PhysXSoftBody3D::set_pressure_coefficient(real_t p_pressure) {
	pressure_coefficient = p_pressure;
}

real_t PhysXSoftBody3D::get_pressure_coefficient() const {
	return pressure_coefficient;
}

void PhysXSoftBody3D::set_damping_coefficient(real_t p_damping) {
	damping_coefficient = p_damping;
}

real_t PhysXSoftBody3D::get_damping_coefficient() const {
	return damping_coefficient;
}

void PhysXSoftBody3D::set_drag_coefficient(real_t p_drag) {
	drag_coefficient = p_drag;
}

real_t PhysXSoftBody3D::get_drag_coefficient() const {
	return drag_coefficient;
}

void PhysXSoftBody3D::set_simulation_precision(int p_precision) {
	simulation_precision = p_precision;
}

int PhysXSoftBody3D::get_simulation_precision() const {
	return simulation_precision;
}

// ============================================================================
// PhysX Access
// ============================================================================

physx::PxActor *PhysXSoftBody3D::get_px_actor() const {
	return px_actor;
}

physx::PxDeformableVolume *PhysXSoftBody3D::get_px_deformable() const {
	// px_actor will be the PxDeformableVolume once created; the typed accessor
	// exists so future code doesn't sprinkle static_casts.
	return static_cast<physx::PxDeformableVolume *>(px_actor);
}

void PhysXSoftBody3D::refresh_user_data() {
	actor_user_data.rid = get_rid();
	actor_user_data.object_id = get_instance_id();
	// TODO: set px_actor->userData = &actor_user_data when the actor exists.
}

// ============================================================================
// Pinned points — TODO: apply via PxDeformableAttachment.
// ============================================================================

void PhysXSoftBody3D::pin_point(int p_point_index, bool p_pin) {
	if (p_pin) {
		pinned_indices.insert(p_point_index);
	} else {
		pinned_indices.erase(p_point_index);
	}
}

// ============================================================================
// FEM SKELETON — deformable volume from a voxelized box.
// ============================================================================

bool PhysXSoftBody3D::build_fem_box(const Vector3 &p_size, int p_voxels) {
	ERR_FAIL_NULL_V_MSG(px_actor, false,
			"PhysX: this soft body already has a deformable volume.");
	PhysXServer3D *server = PhysXServer3D::get_singleton();
	ERR_FAIL_NULL_V_MSG(server, false, "PhysX: server not initialized.");
	PhysXSpace3D *sp = space;
	ERR_FAIL_NULL_V_MSG(sp, false, "PhysX: soft body has no space; set_space() first.");
	physx::PxCudaContextManager *cuda = sp->get_px_cuda();
	ERR_FAIL_NULL_V_MSG(cuda, false,
			"PhysX: FEM soft bodies are GPU-only (no CUDA context; use a physx_gpu build with a device).");

	physx::PxPhysics &physics = server->get_physics();

	// Material: map the stored Godot-side params onto the FEM material.
	// (Skeleton mapping: linear_stiffness 0..1 -> Young's modulus range.)
	const float youngs = 1.0e4f + 4.0e8f * CLAMP((float)linear_stiffness, 0.0f, 1.0f);
	physx::PxDeformableVolumeMaterial *material =
			physics.createDeformableVolumeMaterial(youngs, 0.3f, 0.5f, 0.02f);
	ERR_FAIL_NULL_V_MSG(material, false, "PhysX: createDeformableVolumeMaterial failed.");

	// Voxelize + mesh + create the actor in one call (surface box -> voxels ->
	// tetrahedral deformable volume mesh -> collision shape + sim FEM data).
	physx::PxDeformableVolume *volume = physx::PxDeformableVolumeExt::createDeformableVolumeBox(
			physx_to_px(transform),
			physx::PxVec3((physx::PxReal)p_size.x, (physx::PxReal)p_size.y, (physx::PxReal)p_size.z),
			*material, *cuda, -1.0f,
			(float)MAX(mass, 0.001) / MAX(p_size.x * p_size.y * p_size.z, 0.001f),
			(physx::PxU32)CLAMP(p_voxels, 4, 64));
	ERR_FAIL_NULL_V_MSG(volume, false, "PhysX: createDeformableVolumeBox failed.");

	px_actor = volume;
	refresh_user_data();
	volume->userData = &actor_user_data;
	// Attach to the scene (set_space with an equal pointer short-circuits, so
	// force the add through a remove/add cycle).
	PhysXSpace3D *current = space;
	space = nullptr;
	set_space(current);

	// Pinned host mirrors for the readback.
	physx::PxVec4 *hp = nullptr;
	physx::PxVec4 *hv = nullptr;
	physx::PxVec4 *hcp = nullptr;
	physx::PxVec4 *rp = nullptr;
	physx::PxDeformableVolumeExt::allocateAndInitializeHostMirror(*volume, cuda, hp, hv, hcp, rp);
	fem_sim_pos_pinned = hp;
	fem_sim_vel_pinned = hv;
	fem_coll_pos_pinned = hcp;
	fem_rest_pinned = rp;
	fem_cuda = cuda;
	fem_simulated_once = false;

	// Point count: the collision tet mesh's vertex count (matches the coll
	// mirror buffer).
	physx::PxU32 count = 0;
	if (const physx::PxTetrahedronMesh *coll = volume->getCollisionMesh()) {
		count = coll->getNbVertices();
	}

	if (count == 0) {
		// Fall back to the sim position buffer size inferred from the mirror
		// init: the coll mirror is always populated for at least one point.
		count = 1;
	}
	fem_point_count = (int)count;
	return true;
}

Vector<Vector3> PhysXSoftBody3D::get_fem_positions() {
	Vector<Vector3> out;
	if (!px_actor || !fem_cuda || fem_point_count <= 0) {
		return out;
	}
	auto *volume = static_cast<physx::PxDeformableVolume *>(px_actor);
	if (fem_simulated_once) {
		// GPU -> host: sim positions have been integrated at least once; read
		// the collision buffer (matches the render surface) like the cloth's
		// read_back does.
		physx::PxVec4 *hp = static_cast<physx::PxVec4 *>(fem_coll_pos_pinned);
		physx::Ext::PxCudaHelpersExt::copyDToH(*fem_cuda, hp,
				volume->getPositionInvMassBufferD(), (physx::PxU32)fem_point_count);
	}
	out.resize(fem_point_count);
	physx::PxVec4 *hp = static_cast<physx::PxVec4 *>(fem_coll_pos_pinned);
	for (int i = 0; i < fem_point_count; i++) {
		out.write[i] = Vector3(hp[i].x, hp[i].y, hp[i].z);
	}
	fem_simulated_once = true;
	return out;
}

// ============================================================================
// Inherited Overrides
// ============================================================================

void PhysXSoftBody3D::_update_shapes() {
	// Collision layer/mask propagation — wire to the deformable's filter data
	// when the actor exists.
}
