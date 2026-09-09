/**
 * @file physx_heightmap_shape_3d.h
 * @brief Heightfield collision shape (terrain).
 *
 * Builds a PxHeightField from Godot's PackedFloat32Array height data. The
 * heightfield is cooked lazily on first use and cached until the data changes.
 *
 * Note: PhysX heightfields originate at a local corner, while Godot expects
 * them centered. get_center_offset() provides the translation that the body
 * must apply when attaching the shape.
 */

#ifndef PHYSX_HEIGHT_MAP_SHAPE_3D_H
#define PHYSX_HEIGHT_MAP_SHAPE_3D_H

#include "physx_shape_3d.h"

class PhysXHeightMapShape3D : public PhysXShape3D {
public:
	PhysXHeightMapShape3D() {}
	virtual ~PhysXHeightMapShape3D();

	virtual PhysicsServer3D::ShapeType get_type() const override { return PhysicsServer3D::SHAPE_HEIGHTMAP; }
	virtual bool is_convex() const override { return false; }

	virtual void set_data(const Variant &p_data) override;
	virtual Variant get_data() const override;

	virtual AABB get_aabb() const override { return aabb; }

	virtual bool get_physx_geometry(physx::PxGeometryHolder &p_geometry_holder, const physx::PxVec3 &p_scale) const override;

	virtual physx::PxTransform get_local_pose() const override;

private:
	AABB aabb;
	int width = 0;
	int depth = 0;
	PackedFloat32Array heights;
	float min_height = 0.0f;
	float max_height = 0.0f;
	float height_scale = 1.0f;

	mutable physx::PxHeightField *height_field = nullptr;

	bool _ensure_physx_height_field() const;
	void _release_height_field();

	AABB _calculate_aabb() const;
};
#endif // PHYSX_HEIGHT_MAP_SHAPE_3D_H