#ifndef PHYSX_WORLD_BOUNDARY_SHAPE_3D_H
#define PHYSX_WORLD_BOUNDARY_SHAPE_3D_H

#include "physx_shape_3d.h"

class PhysXWorldBoundaryShape3D : public PhysXShape3D {
	Plane plane;

public:
	PhysXWorldBoundaryShape3D() {}
	virtual ~PhysXWorldBoundaryShape3D() {}

	virtual PhysicsServer3D::ShapeType get_type() const override { return PhysicsServer3D::SHAPE_WORLD_BOUNDARY; }
	virtual bool is_convex() const override { return false; }

	virtual void set_data(const Variant &p_data) override;
	virtual Variant get_data() const override;

	virtual AABB get_aabb() const override;

    virtual physx::PxTransform get_local_pose() const override;

	// Generates the PxPlaneGeometry
	virtual bool get_physx_geometry(physx::PxGeometryHolder &p_geometry_holder, const physx::PxVec3 &p_scale) const override;

	// Expose the plane so the shape owner can apply the correct local transform
	_FORCE_INLINE_ Plane get_plane() const { return plane; }
};
#endif // PHYSX_WORLD_BOUNDARY_SHAPE_3D_H