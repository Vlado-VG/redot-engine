#ifndef PHYSX_CONVEX_POLYGON_SHAPE_3D_H
#define PHYSX_CONVEX_POLYGON_SHAPE_3D_H

#include "physx_shape_3d.h"

class PhysXConvexPolygonShape3D : public PhysXShape3D {
public:
	PhysXConvexPolygonShape3D() {}
	virtual ~PhysXConvexPolygonShape3D();

	virtual PhysicsServer3D::ShapeType get_type() const override { return PhysicsServer3D::SHAPE_CONVEX_POLYGON; }
	virtual bool is_convex() const override { return true; }

	virtual void set_data(const Variant &p_data) override;
	virtual Variant get_data() const override;

	virtual AABB get_aabb() const override { return aabb; }
	
	virtual bool get_physx_geometry(physx::PxGeometryHolder &p_geometry_holder, const physx::PxVec3 &p_scale) const override;

private:
	AABB aabb;
	PackedVector3Array points;

	mutable physx::PxConvexMesh *convex_mesh = nullptr;

	bool _ensure_convex_mesh() const;
	void _release_convex_mesh();

	AABB _calculate_aabb() const;
};
#endif // PHYSX_CONVEX_POLYGON_SHAPE_3D_H