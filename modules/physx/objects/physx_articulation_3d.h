/**
 * @file physx_articulation_3d.h
 * @brief SKELETON implementation -- reduced-coordinate articulations.
 *
 * Wraps PhysX 5's PxArticulationReducedCoordinate: an articulation owns its
 * links (PxArticulationLink) and the reduced-coordinate joints between them,
 * and is simulated as a single actor inside a PxScene. This skeleton wires
 * the full lifecycle (create / add links / drives / limits / free) plus
 * world-pose readback for rendering. It is NOT a complete feature set:
 *tendons, mimics, aggregates, per-link collision filtering and link-level
 * velocity queries are not exposed yet (marked TODO).
 *
 * API surface (via PhysXServer3D, all indices 0-based):
 *   articulation_create()                       -> RID
 *   articulation_set_space(rid, space)          -> attach to a space
 *   articulation_add_link(rid, parent, parent_frame, child_frame,
 *                         joint_type, density, box_half_extents) -> link index
 *       parent -1 creates the base link (no inbound joint; joint_type ignored).
 *       joint_type: 0 = FIX, 1 = PRISMATIC, 2 = REVOLUTE, 3 = REVOLUTE_UNWRAPPED,
 *                   4 = SPHERICAL (PxArticulationJointType values).
 *       Each link gets a box collision shape of the given half extents; mass
 *       properties are computed from the shape at the given density.
 *   articulation_set_drive(rid, link, axis, stiffness, damping,
 *                          drive_target, drive_velocity, drive_type)
 *       axis: 0 = TWIST, 1 = SWING1, 2 = SWING2, 3 = X, 4 = Y, 5 = Z.
 *       drive_type: 0 = FORCE, 1 = ACCELERATION. Also unlocks the axis motion.
 *   articulation_set_limit(rid, link, axis, low, high)
 *   articulation_wake(rid)
 *   articulation_get_link_count(rid) / articulation_get_link_transform(rid, i)
 *
 * Rendering is the caller's job: poll articulation_get_link_transform and
 * mirror it onto visual meshes (see the lab when this gets a scenario).
 */

#ifndef PHYSX_ARTICULATION_3D_H
#define PHYSX_ARTICULATION_3D_H

#include "physx_rid_owner.h"
#include "core/math/transform_3d.h"
#include "core/math/vector3.h"
#include "core/templates/local_vector.h"

namespace physx {
class PxArticulationReducedCoordinate;
class PxArticulationLink;
} // namespace physx

class PhysXSpace3D;

class PhysXArticulation3D : public PhysXRIDOwner {
public:
	PhysXArticulation3D();
	~PhysXArticulation3D();

	void set_space(PhysXSpace3D *p_space);
	PhysXSpace3D *get_space() const { return space; }

	/// Adds a link. parent_index -1 creates the base link. Returns the new
	/// link's index, or -1 on failure. See the file header for conventions.
	int add_link(int p_parent_index,
			const Transform3D &p_parent_frame,
			const Transform3D &p_child_frame,
			int p_joint_type,
			float p_density,
			const Vector3 &p_box_half_extents);

	/// Unlocks the axis, configures the implicit PD drive and sets targets.
	void set_drive(int p_link_index, int p_axis,
			float p_stiffness, float p_damping,
			float p_drive_target, float p_drive_velocity,
			int p_drive_type);

	/// Unlocks the axis and clamps it with a joint limit.
	void set_limit(int p_link_index, int p_axis, float p_low, float p_high);

	void set_fix_base(bool p_fix);
	void wake_up();
	void put_to_sleep();

	int get_link_count() const;
	/// World-space pose of the link (poll for rendering).
	Transform3D get_link_transform(int p_index) const;
	/// True while any link of the articulation is awake.
	bool is_sleeping() const;

private:
	PhysXSpace3D *space = nullptr;
	physx::PxArticulationReducedCoordinate *px_articulation = nullptr;
	LocalVector<physx::PxArticulationLink *> links;

	void _destroy();
	physx::PxArticulationLink *_link(int p_index) const;
};

#endif // PHYSX_ARTICULATION_3D_H
