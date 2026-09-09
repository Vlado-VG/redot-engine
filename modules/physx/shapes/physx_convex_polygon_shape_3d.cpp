#include "physx_convex_polygon_shape_3d.h"
#include "../physx_server.h"
#include <cooking/PxCooking.h>
#include "core/templates/local_vector.h"

PhysXConvexPolygonShape3D::~PhysXConvexPolygonShape3D() {
	_release_convex_mesh();
}

AABB PhysXConvexPolygonShape3D::_calculate_aabb() const {
	AABB result;
	for (int i = 0; i < points.size(); ++i) {
		const Vector3 &vertex = points[i];
		if (i == 0) {
			result.position = vertex;
		} else {
			result.expand_to(vertex);
		}
	}
	return result;
}

void PhysXConvexPolygonShape3D::set_data(const Variant &p_data) {
	// Godot's ConvexPolygon data is just a PackedVector3Array of point cloud vertices,
	// though sometimes it's wrapped in a dictionary depending on the server call context.
	// We handle both just to be safe, but typically it is passed as a direct array or a dict with "points".
	
	PackedVector3Array new_points;

	if (p_data.get_type() == Variant::DICTIONARY) {
		const Dictionary data = p_data;
		const Variant maybe_points = data.get("points", Variant());
		ERR_FAIL_COND(maybe_points.get_type() != Variant::PACKED_VECTOR3_ARRAY);
		new_points = maybe_points;
	} else if (p_data.get_type() == Variant::PACKED_VECTOR3_ARRAY) {
		new_points = p_data;
	} else {
		ERR_FAIL_MSG("Invalid data type for PhysXConvexPolygonShape3D.");
	}

	points = new_points;
	aabb = _calculate_aabb();

	_release_convex_mesh();
	_notify_shape_changed();
}

Variant PhysXConvexPolygonShape3D::get_data() const {
	return points;
}

bool PhysXConvexPolygonShape3D::get_physx_geometry(physx::PxGeometryHolder &holder, const physx::PxVec3 &scale) const {
	if (!_ensure_convex_mesh()) {
		return false;
	}

	holder.storeAny(
		physx::PxConvexMeshGeometry(
			convex_mesh,
			physx::PxMeshScale(scale)
		)
	);

	return true;
}

void PhysXConvexPolygonShape3D::_release_convex_mesh() {
	if (!convex_mesh) {
		return;
	}

	convex_mesh->release();
	convex_mesh = nullptr;
}

bool PhysXConvexPolygonShape3D::_ensure_convex_mesh() const {
	if (convex_mesh) {
		return true;
	}

	const int vertex_count = points.size();

	if (unlikely(vertex_count == 0)) {
		return false;
	}

	// A 3D convex hull requires at least 4 non-coplanar points, but PhysX can 
	// compute planar convex hulls with 3.
	ERR_FAIL_COND_V_MSG(vertex_count < 3, false, "Failed to build PhysX convex polygon: vertex count < 3.");

	// 1. Prepare PhysX data arrays
	LocalVector<physx::PxVec3> px_vertices;
	px_vertices.resize(vertex_count);

	for (int i = 0; i < vertex_count; ++i) {
		const Vector3 &v = points[i];
		px_vertices[i] = physx::PxVec3(v.x, v.y, v.z);
	}

	// 2. Set up the mesh descriptor
	physx::PxConvexMeshDesc mesh_desc;
	mesh_desc.points.count = vertex_count;
	mesh_desc.points.stride = sizeof(physx::PxVec3);
	mesh_desc.points.data = px_vertices.ptr();
	
	// eCOMPUTE_CONVEX tells PhysX to generate the hull from our point cloud.
	mesh_desc.flags = physx::PxConvexFlag::eCOMPUTE_CONVEX;

	// 3. Fetch singletons
	const physx::PxCookingParams &cooking_params = PhysXServer3D::get_singleton()->get_cooking_params();
	physx::PxPhysics &physics = PhysXServer3D::get_singleton()->get_physics();

	ERR_FAIL_NULL_V_MSG(&physics, false, "PhysX PxPhysics is not initialized.");

	// 4. Cook the convex mesh
	convex_mesh = PxCreateConvexMesh(cooking_params, mesh_desc, physics.getPhysicsInsertionCallback());

	if (!convex_mesh) {
		ERR_PRINT("PhysX failed to create convex mesh.");
		return false;
	}

	return true;
}