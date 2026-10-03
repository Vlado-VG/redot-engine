#include "physx_world_boundary_shape_3d.h"

void PhysXWorldBoundaryShape3D::set_data(const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::PLANE);
	
	plane = p_data;
	
	_notify_shape_changed();
}

Variant PhysXWorldBoundaryShape3D::get_data() const {
	return plane;
}

AABB PhysXWorldBoundaryShape3D::get_aabb() const {
	// World boundaries are technically infinite. 
	// In Godot standard implementations, this is often represented by a massive AABB.
	const float size = 1e6f; 
	const float half_size = size / 2.0f;
	return AABB(Vector3(-half_size, -half_size, -half_size), Vector3(size, size, size));
}

physx::PxTransform PhysXWorldBoundaryShape3D::get_local_pose(const physx::PxVec3 &p_scale) const {
	// Define the normals using Godot's native Vector3
	Vector3 default_normal(1.0f, 0.0f, 0.0f);
	Vector3 n = plane.normal;
	
	// Use Godot's Quaternion to calculate the shortest arc rotation
	Quaternion q(default_normal, n);
	
	// Convert Godot math to PhysX math
	physx::PxQuat px_rotation(q.x, q.y, q.z, q.w);
	
	// Calculate translation (Normal * Distance)
	physx::PxVec3 px_translation(n.x * plane.d, n.y * plane.d, n.z * plane.d);
	
	return physx::PxTransform(px_translation, px_rotation);
}

bool PhysXWorldBoundaryShape3D::get_physx_geometry(physx::PxGeometryHolder &p_geometry_holder, const physx::PxVec3 &p_scale) const {
	// PhysX planes do not take parameters in their geometry constructor.
	// The plane equation (normal and distance) MUST be handled by setting 
	// the PxTransform on the PxShape when attaching it to a PxRigidActor.
	
	physx::PxPlaneGeometry plane_geometry;
	p_geometry_holder.storeAny(plane_geometry);
	
	return true;
}