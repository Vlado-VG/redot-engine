// modules/physx/shapes/physx_separation_ray_shape_3d.cpp
#include "physx_separation_ray_shape_3d.h"
#include <geometry/PxBoxGeometry.h>

void PhysXSeparationRayShape3D::set_data(const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::DICTIONARY);

	const Dictionary data = p_data;

	const Variant maybe_length = data.get("length", Variant());
	ERR_FAIL_COND(maybe_length.get_type() != Variant::FLOAT);

	const Variant maybe_slide_on_slope = data.get("slide_on_slope", Variant());
	ERR_FAIL_COND(maybe_slide_on_slope.get_type() != Variant::BOOL);

	length = maybe_length;
	slide_on_slope = maybe_slide_on_slope;

	_notify_shape_changed();
}

Variant PhysXSeparationRayShape3D::get_data() const {
	Dictionary data;
	data["length"] = length;
	data["slide_on_slope"] = slide_on_slope;
	return data;
}

AABB PhysXSeparationRayShape3D::get_aabb() const {
	// Godot SeparationRays project down the local Z-axis
	constexpr float size_xy = 0.1f;
	constexpr float half_size_xy = size_xy * 0.5f;
	return AABB(Vector3(-half_size_xy, -half_size_xy, 0.0f), Vector3(size_xy, size_xy, length));
}

bool PhysXSeparationRayShape3D::get_physx_geometry(physx::PxGeometryHolder &holder, const physx::PxVec3 &scale) const {
	// Emulate the ray using a very thin Box Geometry.
	// A Box is defined by its half-extents in PhysX.
	float half_length = (length * scale.z) * 0.5f;
	float thickness = 0.01f; 

	holder.storeAny(physx::PxBoxGeometry(thickness, thickness, half_length));

	return true;
}