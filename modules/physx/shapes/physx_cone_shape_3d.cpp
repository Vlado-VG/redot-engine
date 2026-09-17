#include "physx_cone_shape_3d.h"

PhysXConeShape3D::PhysXConeShape3D() {}

PhysXConeShape3D::~PhysXConeShape3D() {}

void PhysXConeShape3D::set_data(const Variant &p_data) {
    Dictionary d = p_data;
    radius = d["radius"];
    height = d["height"];

    // Push the new params into every live per-scale instance.
    _refresh_instances();

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

void PhysXConeShape3D::_apply_scale_to_callbacks(physx::PxCustomGeometryExt::ConeCallbacks &cb, const physx::PxVec3& scale) const {
    float scaled_radius = radius * physx::PxMax(physx::PxAbs(scale.x), physx::PxAbs(scale.z));
    float scaled_height = height * physx::PxAbs(scale.y);

    cb.setRadius(scaled_radius);
    cb.setHeight(scaled_height);
}

void PhysXConeShape3D::_apply_params_to_callbacks(physx::PxCustomGeometryExt::ConeCallbacks &cb) const {
    cb.setRadius(radius);
    cb.setHeight(height);
}