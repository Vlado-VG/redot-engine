#include "physx_sphere_shape_3d.h"

void PhysXSphereShape3D::set_data(const Variant &p_data) {
	// Godot sends the radius as a simple float for SHAPE_SPHERE
	radius = p_data;
	_notify_shape_changed();
	// You might want to trigger a shape update listener here if you have one set up
    // configure_shape(); 
}

Variant PhysXSphereShape3D::get_data() const {
	return radius;
}

AABB PhysXSphereShape3D::get_aabb() const {
	// AABB is local to the shape. 
	// Center is 0,0,0, extent is radius in all directions.
	return AABB(Vector3(-radius, -radius, -radius), Vector3(radius * 2.0, radius * 2.0, radius * 2.0));
}

bool PhysXSphereShape3D::get_physx_geometry(physx::PxGeometryHolder &p_geometry_holder, const physx::PxVec3 &p_scale) const {
	// PhysX spheres are perfectly round. They do not support non-uniform scaling (ellipsoids).
	// Standard practice is to use the maximum scale component to ensure the collider 
	// fully encompasses the visual mesh, or just assume uniform scaling.
	
	// We use PxAbs to handle negative scales (flipping) correctly.
	float max_scale = physx::PxMax(physx::PxAbs(p_scale.x), physx::PxMax(physx::PxAbs(p_scale.y), physx::PxAbs(p_scale.z)));
	
	// Create the geometry
	physx::PxSphereGeometry sphere_geometry(radius * max_scale);
	
	// Store it in the holder
	p_geometry_holder.storeAny(sphere_geometry);
	
	return true;
}