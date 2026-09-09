#ifndef PHYSX_CYLINDER_SHAPE_3D_H
#define PHYSX_CYLINDER_SHAPE_3D_H

#include "physx_custom_geometry_callback.h"

// Inherit from the instantiated template!
class PhysXCylinderShape3D : public PhysXCustomGeometryCallback<physx::PxCustomGeometryExt::CylinderCallbacks> {
public:
    PhysXCylinderShape3D();
    ~PhysXCylinderShape3D() override;

    virtual PhysicsServer3D::ShapeType get_type() const override {
        return PhysicsServer3D::SHAPE_CYLINDER;
    }

    virtual bool is_convex() const override {
        return true;
    }

    virtual void set_data(const Variant &p_data) override;
    virtual Variant get_data() const override;

    virtual AABB get_aabb() const override;

protected:
    // Return types now match the template signature explicitly
    virtual physx::PxCustomGeometryExt::CylinderCallbacks* _create_callbacks() const override;
    virtual void _apply_scale_to_callbacks(const physx::PxVec3& scale) const override;

private:
    float radius = 0.5f;
    float height = 1.0f;
};
#endif // PHYSX_CYLINDER_SHAPE_3D_H