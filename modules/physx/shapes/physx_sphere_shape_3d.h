#ifndef PHYSX_SPHERE_SHAPE_3D_H
#define PHYSX_SPHERE_SHAPE_3D_H

#include "physx_shape_3d.h"

// --- PhysX API ---
#include <PxPhysicsAPI.h>

class PhysXSphereShape3D : public PhysXShape3D {
	float radius = 0.0f;

public:
	PhysXSphereShape3D() {}
	virtual ~PhysXSphereShape3D() {}

	virtual PhysicsServer3D::ShapeType get_type() const override { return PhysicsServer3D::SHAPE_SPHERE; }
	virtual bool is_convex() const override { return true; }

	virtual void set_data(const Variant &p_data) override;
	virtual Variant get_data() const override;

	virtual AABB get_aabb() const override;

	// Generates the PxSphereGeometry
	virtual bool get_physx_geometry(physx::PxGeometryHolder &p_geometry_holder, const physx::PxVec3 &p_scale) const override;
};
#endif // PHYSX_SPHERE_SHAPE_3D_H