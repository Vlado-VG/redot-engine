#include "physx_cylinder_shape_3d.h"

PhysXCylinderShape3D::PhysXCylinderShape3D() {}

PhysXCylinderShape3D::~PhysXCylinderShape3D() {}

void PhysXCylinderShape3D::set_data(const Variant &p_data) {
    Dictionary d = p_data;
    radius = d["radius"];
    height = d["height"];

    _invalidate_scale();

    // Look at how clean this is now! No static_cast required.
    if (callbacks) {
        callbacks->setRadius(radius);
        callbacks->setHeight(height);
    }

    _notify_shape_changed();
}

Variant PhysXCylinderShape3D::get_data() const {
    Dictionary data;
    data["height"] = height;
    data["radius"] = radius;
    return data;
}

AABB PhysXCylinderShape3D::get_aabb() const {
    return AABB(
        Vector3(-radius, -height * 0.5f, -radius),
        Vector3(radius * 2, height, radius * 2)
    );
}

physx::PxCustomGeometryExt::CylinderCallbacks* PhysXCylinderShape3D::_create_callbacks() const {
    return new physx::PxCustomGeometryExt::CylinderCallbacks(height, radius, 1); 
}

void PhysXCylinderShape3D::_apply_scale_to_callbacks(const physx::PxVec3& scale) const {
    if (!callbacks) return;

    float scaled_radius = radius * physx::PxMax(physx::PxAbs(scale.x), physx::PxAbs(scale.z));
    float scaled_height = height * physx::PxAbs(scale.y);

    callbacks->setRadius(scaled_radius);
    callbacks->setHeight(scaled_height);
}