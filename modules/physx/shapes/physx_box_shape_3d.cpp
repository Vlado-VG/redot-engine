#include "physx_box_shape_3d.h"

PhysXBoxShape3D::PhysXBoxShape3D() {
    // Server contract for SHAPE_BOX data is HALF-EXTENTS (core BoxShape3D
    // passes size / 2, and godot_physics / Jolt both treat the data as
    // half-extents). Godot's default BoxShape3D size is (1, 1, 1), so the
    // contract-correct default here is (0.5, 0.5, 0.5).
    half_extents = Vector3(0.5, 0.5, 0.5);
}

void PhysXBoxShape3D::set_data(const Variant &p_data) {
    // SHAPE_BOX data is HALF-EXTENTS, per the server contract: core
    // BoxShape3D::set_size passes size / 2 to the physics server, and both
    // reference backends (godot_physics_3d's GodotBoxShape3D and
    // jolt_physics_3d) interpret the data as half-extents. Treating it as
    // full size and halving it here shrinks every box to a quarter of its
    // volume (REG: P5 probe measured boxes resting at exactly half height
    // and a 2 m cube reporting a 1 m cube's inertia).
    half_extents = Vector3(p_data);
    _notify_shape_changed();
}

Variant PhysXBoxShape3D::get_data() const {
    // Return the half-extents as stored (mirrors set_data and the contract).
    return half_extents;
}

AABB PhysXBoxShape3D::get_aabb() const {
    return AABB(-half_extents, half_extents * 2.0);
}

bool PhysXBoxShape3D::get_physx_geometry(physx::PxGeometryHolder &p_geometry_holder, const physx::PxVec3 &p_scale) const {
	// Calculate final scaled half-extents
    physx::PxVec3 scaled_half_extents;
    scaled_half_extents.x = half_extents.x * p_scale.x;
    scaled_half_extents.y = half_extents.y * p_scale.y;
    scaled_half_extents.z = half_extents.z * p_scale.z;

    // Safety: PhysX doesn't like zero or negative dimensions
    // Using a tiny epsilon ensures the physics engine doesn't crash on flat boxes
    const float EPSILON = 0.0001f;
    scaled_half_extents.x = MAX(scaled_half_extents.x, EPSILON);
    scaled_half_extents.y = MAX(scaled_half_extents.y, EPSILON);
    scaled_half_extents.z = MAX(scaled_half_extents.z, EPSILON);

    // Store in the holder — scaled_half_extents is already half the full size,
    // matching PxBoxGeometry's half-extent expectation.
    p_geometry_holder.storeAny(physx::PxBoxGeometry(scaled_half_extents));
    
    return true;
}
