// modules/physx/shapes/physx_separation_ray_shape_3d.h
#ifndef PHYSX_SEPARATION_RAY_SHAPE_3D_H
#define PHYSX_SEPARATION_RAY_SHAPE_3D_H

#include "physx_shape_3d.h"

class PhysXSeparationRayShape3D : public PhysXShape3D {
public:
	PhysXSeparationRayShape3D() {}
	virtual ~PhysXSeparationRayShape3D() {}

	virtual PhysicsServer3D::ShapeType get_type() const override { return PhysicsServer3D::SHAPE_SEPARATION_RAY; }
	virtual bool is_convex() const override { return true; }

	virtual void set_data(const Variant &p_data) override;
	virtual Variant get_data() const override;

	virtual AABB get_aabb() const override;

	virtual bool get_physx_geometry(physx::PxGeometryHolder &p_geometry_holder, const physx::PxVec3 &p_scale) const override;

	virtual physx::PxTransform get_local_pose(const physx::PxVec3 &p_scale = physx::PxVec3(1.0f)) const override;

	float get_length() const { return length; }
	bool get_slide_on_slope() const { return slide_on_slope; }

private:
	float length = 1.0f;
	bool slide_on_slope = false;
};
#endif // PHYSX_SEPARATION_RAY_SHAPE_3D_H