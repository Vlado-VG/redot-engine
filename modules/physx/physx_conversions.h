/**
 * @file physx_conversions.h
 * @brief Inline Godot ↔ PhysX math conversions shared across the module.
 *
 * Every conversion lives in
 * one header so objects/shapes/spaces/joints stop maintaining private copies,
 * and Transform3D → PxTransform clamps degenerate input instead of passing
 * NaN / zero-length quaternions into PhysX (which asserts or corrupts the
 * scene in checked builds).
 */

#ifndef PHYSX_CONVERSIONS_H
#define PHYSX_CONVERSIONS_H

#include "core/math/basis.h"
#include "core/math/transform_3d.h"
#include "core/math/vector3.h"

#include <foundation/PxQuat.h>
#include <foundation/PxTransform.h>
#include <foundation/PxVec3.h>

_FORCE_INLINE_ physx::PxVec3 physx_to_px(const Vector3 &p_v) {
	return physx::PxVec3((physx::PxReal)p_v.x, (physx::PxReal)p_v.y, (physx::PxReal)p_v.z);
}

_FORCE_INLINE_ Vector3 physx_to_godot(const physx::PxVec3 &p_v) {
	return Vector3((real_t)p_v.x, (real_t)p_v.y, (real_t)p_v.z);
}

_FORCE_INLINE_ physx::PxQuat physx_to_px(const Quaternion &p_q) {
	return physx::PxQuat((physx::PxReal)p_q.x, (physx::PxReal)p_q.y, (physx::PxReal)p_q.z, (physx::PxReal)p_q.w);
}

_FORCE_INLINE_ Quaternion physx_to_godot(const physx::PxQuat &p_q) {
	return Quaternion((real_t)p_q.x, (real_t)p_q.y, (real_t)p_q.z, (real_t)p_q.w);
}

/**
 * Rigid (rotation + translation) conversion. PhysX transforms cannot carry
 * scale — callers bake scale into the geometry (see
 * PhysXShapedObject3D::_shape_geometry_scale). A degenerate/NaN transform
 * from script would assert or crash PhysX, so it is clamped to identity
 * rotation (position kept when finite).
 */
_FORCE_INLINE_ physx::PxTransform physx_to_px(const Transform3D &p_xform) {
	physx::PxVec3 p = physx_to_px(p_xform.origin);
	physx::PxQuat q = physx_to_px(p_xform.basis.get_rotation_quaternion());
	if (!p.isFinite() || !q.isFinite() || q.magnitudeSquared() < 1e-6f) {
		return physx::PxTransform(p.isFinite() ? p : physx::PxVec3(0.0f), physx::PxQuat(physx::PxIdentity));
	}
	return physx::PxTransform(p, q.getNormalized());
}

_FORCE_INLINE_ Transform3D physx_to_godot(const physx::PxTransform &p_xform) {
	return Transform3D(Basis(physx_to_godot(p_xform.q)), physx_to_godot(p_xform.p));
}

#endif // PHYSX_CONVERSIONS_H
