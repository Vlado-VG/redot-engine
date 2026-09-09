#ifndef PHYSX_CONE_SHAPE_3D_H
#define PHYSX_CONE_SHAPE_3D_H

#include "physx_custom_geometry_callback.h"

class PhysXConeShape3D : public PhysXCustomGeometryCallback<physx::PxCustomGeometryExt::ConeCallbacks> {
public:
    PhysXConeShape3D();
    ~PhysXConeShape3D() override;

    virtual PhysicsServer3D::ShapeType get_type() const override {
        return PhysicsServer3D::SHAPE_CUSTOM;
    }

    virtual bool is_convex() const override {
        return true;
    }

    virtual void set_data(const Variant &p_data) override;
    virtual Variant get_data() const override;

    virtual AABB get_aabb() const override;

protected:
    virtual physx::PxCustomGeometryExt::ConeCallbacks* _create_callbacks() const override;
    virtual void _apply_scale_to_callbacks(const physx::PxVec3& scale) const override;

private:
    float radius = 0.5f;
    float height = 1.0f;
};
#endif // PHYSX_CONE_SHAPE_3D_H