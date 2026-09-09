#ifndef PHYSX_BOX_SHAPE_3D_H
#define PHYSX_BOX_SHAPE_3D_H

#include "physx_shape_3d.h"

// --- PhysX API ---
#include <PxPhysicsAPI.h>

class PhysXBoxShape3D : public PhysXShape3D {

    Vector3 half_extents; // Stored as half-extents (PxBoxGeometry expects half-extents; set_data() receives full-size extents and halves them)
public:
    PhysXBoxShape3D();
    virtual ~PhysXBoxShape3D(){}

    virtual PhysicsServer3D::ShapeType get_type() const override { return PhysicsServer3D::SHAPE_BOX; }
    virtual bool is_convex() const override { return true; }

    virtual void set_data(const Variant &p_data) override;
    virtual Variant get_data() const override;
    
    virtual AABB get_aabb() const override;

    // The key function that generates the PhysX geometry
    virtual bool get_physx_geometry(physx::PxGeometryHolder &p_geometry_holder, const physx::PxVec3 &p_scale) const override;

};
#endif // PHYSX_BOX_SHAPE_3D_H