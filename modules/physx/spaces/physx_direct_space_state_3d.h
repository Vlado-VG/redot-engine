/**
 * @file physx_direct_space_state_3d.h
 * @brief PhysicsDirectSpaceState3D implementation for PhysX scene queries.
 *
 * Provides the query surface that Godot uses for:
 *   - intersect_ray (raycasting)
 *   - intersect_point (point/overlap queries)
 *   - intersect_shape (shape overlap queries)
 *   - cast_motion (swept shape queries)
 *   - collide_shape (contact-point generation)
 *   - rest_info (nearest surface info)
 *   - body_test_motion (CharacterBody3D move_and_slide)
 *
 * All queries use PhysX's native PxScene::raycast/overlap/sweep with a
 * PhysXQueryFilterCallback that applies Godot's collision mask/exclusion rules.
 */

#ifndef PHYSX_DIRECT_SPACE_STATE_3D_H
#define PHYSX_DIRECT_SPACE_STATE_3D_H

#include "servers/physics_3d/physics_server_3d.h"
#include "physx_space_3d.h"
#include "core/templates/local_vector.h"

#include "PxPhysicsAPI.h"

class PhysXBody3D;
class PhysXShape3D;
class PhysXSpace3D;

class PhysXDirectSpaceState3D final : public PhysicsDirectSpaceState3D {
	GDCLASS(PhysXDirectSpaceState3D, PhysicsDirectSpaceState3D)

public:
    PhysXDirectSpaceState3D() = default;
	explicit PhysXDirectSpaceState3D(PhysXSpace3D *p_space);

    virtual bool intersect_ray(const RayParameters &p_parameters, RayResult &r_result) override;
	virtual int intersect_point(const PointParameters &p_parameters, ShapeResult *r_results, int p_result_max) override;
	virtual int intersect_shape(const ShapeParameters &p_parameters, ShapeResult *r_results, int p_result_max) override;
	virtual bool cast_motion(const ShapeParameters &p_parameters, real_t &r_closest_safe, real_t &r_closest_unsafe, ShapeRestInfo *r_info = nullptr) override;
	virtual bool collide_shape(const ShapeParameters &p_parameters, Vector3 *r_results, int p_result_max, int &r_result_count) override;
	virtual bool rest_info(const ShapeParameters &p_parameters, ShapeRestInfo *r_info) override;
	virtual Vector3 get_closest_point_to_object_volume(RID p_object, Vector3 p_point) const override;

    bool body_test_motion(const PhysXBody3D &p_body, const PhysicsServer3D::MotionParameters &p_parameters, PhysicsServer3D::MotionResult *r_result) const;

private:
    PhysXSpace3D *space = nullptr;

    bool _body_motion_recover(const PhysXBody3D &p_body, const Transform3D &p_transform, float p_margin, const HashSet<RID> &p_self_and_excluded, const HashSet<ObjectID> &p_excluded_objects, Vector3 &r_recovery) const;
    bool _body_motion_cast(const PhysXBody3D &p_body, const Transform3D &p_transform, const Vector3 &p_motion, bool p_collide_separation_ray, const HashSet<RID> &p_self_and_excluded, const HashSet<ObjectID> &p_excluded_objects, real_t &r_safe_fraction, real_t &r_unsafe_fraction, Vector3 &r_hit_position, Vector3 &r_hit_normal, const physx::PxRigidActor *&r_hit_actor, const physx::PxShape *&r_hit_shape) const;
    bool _body_motion_collide(const PhysXBody3D &p_body, const Transform3D &p_transform, const Vector3 &p_motion, int p_max_collisions, const HashSet<RID> &p_self_and_excluded, const HashSet<ObjectID> &p_excluded_objects, PhysicsServer3D::MotionResult *r_result) const;
};
#endif // PHYSX_DIRECT_SPACE_STATE_3D_H