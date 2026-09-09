#include "physx_capsule_shape_3d.h"

void PhysXCapsuleShape3D::set_data(const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::DICTIONARY);

	const Dictionary data = p_data;

	const Variant maybe_height = data.get("height", Variant());
	ERR_FAIL_COND(maybe_height.get_type() != Variant::FLOAT);

	const Variant maybe_radius = data.get("radius", Variant());
	ERR_FAIL_COND(maybe_radius.get_type() != Variant::FLOAT);

	height = maybe_height;
	radius = maybe_radius;

	_notify_shape_changed();
	// Trigger shape update listener here if needed
}

Variant PhysXCapsuleShape3D::get_data() const {
	Dictionary data;
	data["height"] = height;
	data["radius"] = radius;
	return data;
}

AABB PhysXCapsuleShape3D::get_aabb() const {
	// Capsule is aligned to the Y axis in Godot
	const Vector3 half_extents(radius, height / 2.0f, radius);
	return AABB(-half_extents, half_extents * 2.0f);
}

physx::PxTransform PhysXCapsuleShape3D::get_local_pose() const {
    // Rotate PhysX's local X-axis to match Redot's Y-axis
    physx::PxQuat rot_z_90(physx::PxHalfPi, physx::PxVec3(0.0f, 0.0f, 1.0f));
    return physx::PxTransform(physx::PxVec3(0.0f), rot_z_90);
}

bool PhysXCapsuleShape3D::get_physx_geometry(physx::PxGeometryHolder &p_geometry_holder, const physx::PxVec3 &p_scale) const {
	// PhysX doesn't support non-uniform scaling on capsules. 
	// We extract a uniform max scale just like the sphere implementation.
	float max_scale = physx::PxMax(physx::PxAbs(p_scale.x), physx::PxMax(physx::PxAbs(p_scale.y), physx::PxAbs(p_scale.z)));
	
	float scaled_radius = radius * max_scale;
	float scaled_height = height * max_scale;

	// PhysX expects the half-height of the *cylindrical* part, not the whole capsule.
	// Formula: (Total Height / 2) - Radius
	float half_cylinder_height = (scaled_height / 2.0f) - scaled_radius;
	
	// Failsafe: if height < radius * 2, PhysX will assert/crash. Clamp it.
	if (half_cylinder_height < 0.0f) {
		half_cylinder_height = 0.0f;
	}

	physx::PxCapsuleGeometry capsule_geometry(scaled_radius, half_cylinder_height);
	p_geometry_holder.storeAny(capsule_geometry);

	return true;
}