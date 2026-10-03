#ifndef PHYSX_CONCAVE_POLYGON_SHAPE_3D_H
#define PHYSX_CONCAVE_POLYGON_SHAPE_3D_H

#include "physx_shape_3d.h"

class PhysXConcavePolygonShape3D : public PhysXShape3D {
	

public:
	PhysXConcavePolygonShape3D() {}
	virtual ~PhysXConcavePolygonShape3D();

	virtual PhysicsServer3D::ShapeType get_type() const override { return PhysicsServer3D::SHAPE_CONCAVE_POLYGON; }
	virtual bool is_convex() const override { return false; }

	/// Backface cooking duplicates every triangle (forward + reversed winding):
	/// raw PhysX face indices address the DOUBLED mesh, Godot consumers expect
	/// indices into their own faces array.
	virtual int translate_face_index(int p_face_index) const override {
		return back_face_collision ? p_face_index / 2 : p_face_index;
	}

	virtual void set_data(const Variant &p_data) override;
	virtual Variant get_data() const override;

	virtual AABB get_aabb() const override { return aabb; }
	
	// Generates a Concave Polygon as a fast fallback
	virtual bool get_physx_geometry(physx::PxGeometryHolder &p_geometry_holder, const physx::PxVec3 &p_scale) const override;

private:
    AABB aabb;
    PackedVector3Array faces;
    bool back_face_collision = false;

    mutable physx::PxTriangleMesh *triangle_mesh = nullptr;

    bool _ensure_triangle_mesh() const;
    void _release_triangle_mesh();

    AABB _calculate_aabb() const;
};
#endif // PHYSX_CONCAVE_POLYGON_SHAPE_3D_H