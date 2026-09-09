/**
 * @file physx_capsule_shape_3d.h
 * @brief Capsule collision shape (radius + height).
 *
 * Notable: PhysX capsules are aligned along the X axis, while Godot capsules
 * are Y-axis aligned. The get_local_pose() override applies a 90-degree
 * rotation around Z to compensate.
 */

#ifndef PHYSX_CAPSULE_SHAPE_3D_H
#define PHYSX_CAPSULE_SHAPE_3D_H

#include "physx_shape_3d.h"

class PhysXCapsuleShape3D : public PhysXShape3D {
	float height = 2.0f;
	float radius = 0.5f;

public:
	PhysXCapsuleShape3D() {}
	virtual ~PhysXCapsuleShape3D() {}

	virtual PhysicsServer3D::ShapeType get_type() const override { return PhysicsServer3D::SHAPE_CAPSULE; }
	virtual bool is_convex() const override { return true; }

	virtual void set_data(const Variant &p_data) override;
	virtual Variant get_data() const override;

	virtual AABB get_aabb() const override;
	
	// Applies the 90-degree rotation
    virtual physx::PxTransform get_local_pose() const override;

	// Generates the PxCapsuleGeometry
	virtual bool get_physx_geometry(physx::PxGeometryHolder &p_geometry_holder, const physx::PxVec3 &p_scale) const override;
};
#endif // PHYSX_CAPSULE_SHAPE_3D_H