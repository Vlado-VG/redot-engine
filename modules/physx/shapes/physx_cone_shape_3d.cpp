#include "physx_cone_shape_3d.h"

PhysXConeShape3D::PhysXConeShape3D() {}

PhysXConeShape3D::~PhysXConeShape3D() {}

void PhysXConeShape3D::set_data(const Variant &p_data) {
    Dictionary d = p_data;
    radius = d["radius"];
    height = d["height"];

    _invalidate_scale();

    if (callbacks) {
        callbacks->setRadius(radius);
        callbacks->setHeight(height);
    }

    _notify_shape_changed();
}

Variant PhysXConeShape3D::get_data() const {
    Dictionary data;
    data["height"] = height;
    data["radius"] = radius;
    return data;
}

AABB PhysXConeShape3D::get_aabb() const {
    return AABB(
        Vector3(-radius, -height * 0.5f, -radius),
        Vector3(radius * 2, height, radius * 2)
    );
}

physx::PxCustomGeometryExt::ConeCallbacks* PhysXConeShape3D::_create_callbacks() const {
    return new physx::PxCustomGeometryExt::ConeCallbacks(height, radius, 1); // 1 is Y-axis alignment
}

void PhysXConeShape3D::_apply_scale_to_callbacks(const physx::PxVec3& scale) const {
    if (!callbacks) return;

    // Strongly-typed access is now directly resolved via templates; no casts!
    float scaled_radius = radius * physx::PxMax(physx::PxAbs(scale.x), physx::PxAbs(scale.z));
    float scaled_height = height * physx::PxAbs(scale.y);

    callbacks->setRadius(scaled_radius);
    callbacks->setHeight(scaled_height);
}