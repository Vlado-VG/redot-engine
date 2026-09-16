/**
 * @file physx_articulation_3d.cpp
 * @brief SKELETON implementation of the reduced-coordinate articulation wrapper.
 */

#include "physx_articulation_3d.h"
#include "physx_server.h"
#include "../physx_conversions.h"
#include "../spaces/physx_space_3d.h"

#include "core/error/error_macros.h"
#include "core/math/math_funcs.h"

#include "PxPhysicsAPI.h"
#include "extensions/PxRigidBodyExt.h"
#include "../shapes/physx_shape_3d.h"

using namespace physx;

// Godot-facing enums (see the header for the tables):
//   joint_type: 0 FIX, 1 PRISMATIC, 2 REVOLUTE, 3 REVOLUTE_UNWRAPPED, 4 SPHERICAL
//   axis:       0 TWIST, 1 SWING1, 2 SWING2, 3 X, 4 Y, 5 Z
//   drive_type: 0 FORCE, 1 ACCELERATION
static PxArticulationJointType::Enum _map_joint_type(int p_type) {
	switch (p_type) {
		case 1: return PxArticulationJointType::ePRISMATIC;
		case 2: return PxArticulationJointType::eREVOLUTE;
		case 3: return PxArticulationJointType::eREVOLUTE_UNWRAPPED;
		case 4: return PxArticulationJointType::eSPHERICAL;
		default: return PxArticulationJointType::eFIX;
	}
}

static PxArticulationAxis::Enum _map_axis(int p_axis) {
	if (p_axis < 0 || p_axis > 5) {
		return PxArticulationAxis::eTWIST;
	}
	return static_cast<PxArticulationAxis::Enum>(p_axis);
}

static PxArticulationDriveType::Enum _map_drive_type(int p_type) {
	return p_type == 1 ? PxArticulationDriveType::eACCELERATION : PxArticulationDriveType::eFORCE;
}

PhysXArticulation3D::PhysXArticulation3D() {
	PhysXServer3D *server = PhysXServer3D::get_singleton();
	ERR_FAIL_NULL_MSG(server, "PhysX: articulation_create() called before PhysX initialization.");
	physx::PxPhysics &physics = server->get_physics();
	px_articulation = physics.createArticulationReducedCoordinate();
	ERR_FAIL_NULL_MSG(px_articulation, "PhysX: createArticulationReducedCoordinate failed.");
}

PhysXArticulation3D::~PhysXArticulation3D() {
	_destroy();
}

void PhysXArticulation3D::_destroy() {
	if (px_articulation) {
		// removeArticulation() mutates the scene — fetch an in-flight solve
		// first (async stepping).
		if (space) {
			space->ensure_synced();
			if (PxScene *scene = space->get_px_scene()) {
				scene->removeArticulation(*px_articulation);
			}
		}
		px_articulation->release();
		px_articulation = nullptr;
	}
	links.clear();
}

void PhysXArticulation3D::set_space(PhysXSpace3D *p_space) {
	if (space == p_space) {
		return;
	}
	// addArticulation/removeArticulation below mutate the scene — fetch any
	// in-flight solve on both spaces first (async stepping).
	if (space) {
		space->ensure_synced();
	}
	if (p_space) {
		p_space->ensure_synced();
	}
	if (space && px_articulation) {
		if (PxScene *scene = space->get_px_scene()) {
			scene->removeArticulation(*px_articulation);
		}
	}
	space = p_space;
	if (space && px_articulation) {
		if (PxScene *scene = space->get_px_scene()) {
			scene->addArticulation(*px_articulation);
		}
	}
}

int PhysXArticulation3D::add_link(int p_parent_index,
		const Transform3D &p_parent_frame,
		const Transform3D &p_child_frame,
		int p_joint_type,
		float p_density,
		const Vector3 &p_box_half_extents) {
	ERR_FAIL_NULL_V(px_articulation, -1);
	PhysXServer3D *server = PhysXServer3D::get_singleton();
	ERR_FAIL_NULL_V(server, -1);

	physx::PxArticulationLink *parent_link = nullptr;
	if (p_parent_index >= 0) {
		ERR_FAIL_INDEX_V(p_parent_index, (int)links.size(), -1);
		parent_link = links[p_parent_index].link;
	}

	// The link's pose is relative to its parent; the base link's pose is in
	// world space (parent_frame carries the chain layout, the child frame the
	// inbound joint anchor inside the new link).
	physx::PxArticulationLink *link = px_articulation->createLink(
			parent_link, physx_to_px(p_child_frame));
	ERR_FAIL_NULL_V(link, -1);

	// Box collision shape so the link interacts with the world; mass
	// properties derive from it at the requested density.
	const float he_x = MAX(p_box_half_extents.x, 0.01f);
	const float he_y = MAX(p_box_half_extents.y, 0.01f);
	const float he_z = MAX(p_box_half_extents.z, 0.01f);
	const physx::PxMaterial *material = server->get_default_material();
	physx::PxShape *shape = server->get_physics().createShape(
			physx::PxBoxGeometry(he_x, he_y, he_z), *material, true,
			physx::PxShapeFlag::eVISUALIZATION | physx::PxShapeFlag::eSCENE_QUERY_SHAPE |
					physx::PxShapeFlag::eSIMULATION_SHAPE);
	if (shape) {
		link->attachShape(*shape);
		shape->release();
	}
	physx::PxRigidBodyExt::updateMassAndInertia(*link, MAX(p_density, 0.001f));

	if (parent_link) {
		auto *joint = static_cast<physx::PxArticulationJointReducedCoordinate *>(link->getInboundJoint());
		joint->setJointType(_map_joint_type(p_joint_type));
		joint->setParentPose(physx_to_px(p_parent_frame));
		joint->setChildPose(physx::PxTransform(physx::PxIdentity));
	}

	LinkRecord rec;
	rec.link = link;
	rec.shape = shape;
	rec.density = MAX(p_density, 0.001f);
	links.push_back(rec);
	// Per-link mass properties are set by updateMassAndInertia() above; the
	// reduced-coordinate solver aggregates them when the articulation is added
	// to a scene.
	return (int)links.size() - 1;
}

void PhysXArticulation3D::set_link_shape(int p_link_index, PhysXShape3D *p_shape, const Transform3D &p_transform) {
	LinkRecord *rec = _record(p_link_index);
	ERR_FAIL_NULL_MSG(rec, "PhysX: articulation_set_link_shape on invalid link index.");
	ERR_FAIL_NULL_MSG(p_shape, "PhysX: articulation_set_link_shape passed an invalid shape.");
	if (!rec->link || !px_articulation) {
		return;
	}

	// detachShape/attachShape below mutate the scene's broadphase — fetch an
	// in-flight solve first (async stepping).
	if (space) {
		space->ensure_synced();
	}

	if (rec->shape) {
		rec->link->detachShape(*rec->shape);
		rec->shape = nullptr;
	}

	// Same dispatch as PhysXShapedObject3D::_shape_geometry_scale_for: convex
	// polygon meshes consume the signed (mirror-capable) scale — the blueprint
	// bakes the mirror — every other geometry the absolute scale. Links have
	// no separate body scale.
	const bool convex_poly = p_shape->get_type() == PhysicsServer3D::SHAPE_CONVEX_POLYGON;
	const Vector3 scale = (convex_poly && !Math::is_zero_approx(p_transform.basis.determinant()))
			? p_transform.basis.get_scale()
			: p_transform.basis.get_scale_abs();

	// Links simulate, so the REG-0014 rule applies: concave geometries
	// (triangle mesh/heightfield/plane) must be query-only on them.
	physx::PxShapeFlags flags = physx::PxShapeFlag::eVISUALIZATION | physx::PxShapeFlag::eSCENE_QUERY_SHAPE |
			physx::PxShapeFlag::eSIMULATION_SHAPE;
	if (!p_shape->is_convex()) {
		flags = physx::PxShapeFlag::eVISUALIZATION | physx::PxShapeFlag::eSCENE_QUERY_SHAPE;
	}

	PhysXServer3D *server = PhysXServer3D::get_singleton();
	physx::PxShape *shape = p_shape->create_shape(
			server->get_physics(),
			physx::PxVec3(scale.x, scale.y, scale.z),
			server->get_default_material(),
			flags);
	ERR_FAIL_NULL_MSG(shape, "PhysX: failed to create articulation link shape.");
	shape->setLocalPose(physx_to_px(p_transform) * p_shape->get_local_pose());
	rec->link->attachShape(*shape);
	// The link owns the shape after attach (refcount 2 -> 1 on release).
	shape->release();
	rec->shape = shape;
	_apply_link_filter(*rec);

	// Geometry changed: recompute mass properties at the stored density.
	physx::PxRigidBodyExt::updateMassAndInertia(*rec->link, rec->density);
}

void PhysXArticulation3D::set_link_collision_layer(int p_link_index, uint32_t p_layer) {
	LinkRecord *rec = _record(p_link_index);
	ERR_FAIL_NULL_MSG(rec, "PhysX: articulation link layer on invalid link index.");
	rec->collision_layer = p_layer;
	_apply_link_filter(*rec);
}

void PhysXArticulation3D::set_link_collision_mask(int p_link_index, uint32_t p_mask) {
	LinkRecord *rec = _record(p_link_index);
	ERR_FAIL_NULL_MSG(rec, "PhysX: articulation link mask on invalid link index.");
	rec->collision_mask = p_mask;
	_apply_link_filter(*rec);
}

uint32_t PhysXArticulation3D::get_link_collision_layer(int p_index) const {
	const LinkRecord *rec = _record(p_index);
	return rec ? rec->collision_layer : 0;
}

uint32_t PhysXArticulation3D::get_link_collision_mask(int p_index) const {
	const LinkRecord *rec = _record(p_index);
	return rec ? rec->collision_mask : 0;
}

Dictionary PhysXArticulation3D::get_link_velocity(int p_index) const {
	Dictionary d;
	physx::PxArticulationLink *link = const_cast<PhysXArticulation3D *>(this)->_link(p_index);
	if (!link) {
		return d;
	}
	const physx::PxVec3 lv = link->getLinearVelocity();
	const physx::PxVec3 av = link->getAngularVelocity();
	d["linear"] = Vector3(lv.x, lv.y, lv.z);
	d["angular"] = Vector3(av.x, av.y, av.z);
	return d;
}

void PhysXArticulation3D::set_drive(int p_link_index, int p_axis,
		float p_stiffness, float p_damping,
		float p_drive_target, float p_drive_velocity,
		int p_drive_type) {
	physx::PxArticulationLink *link = _link(p_link_index);
	ERR_FAIL_NULL_MSG(link, "PhysX: articulation drive on invalid link index.");
	auto *joint = static_cast<physx::PxArticulationJointReducedCoordinate *>(link->getInboundJoint());
	// The base link has no inbound joint — drives/limits only apply to child links.
	ERR_FAIL_NULL_MSG(joint, "PhysX: articulation drive on the base link (no inbound joint).");
	const physx::PxArticulationAxis::Enum axis = _map_axis(p_axis);

	// A driven axis must be unlocked; the drive spring then pulls it toward
	// the target at the requested velocity.
	joint->setMotion(axis, physx::PxArticulationMotion::eFREE);
	physx::PxArticulationDrive drive(
			p_stiffness, p_damping, PX_MAX_F32, _map_drive_type(p_drive_type));
	joint->setDriveParams(axis, drive);
	joint->setDriveTarget(axis, p_drive_target);
	joint->setDriveVelocity(axis, p_drive_velocity);
}

void PhysXArticulation3D::set_limit(int p_link_index, int p_axis, float p_low, float p_high) {
	physx::PxArticulationLink *link = _link(p_link_index);
	ERR_FAIL_NULL_MSG(link, "PhysX: articulation limit on invalid link index.");
	auto *joint = static_cast<physx::PxArticulationJointReducedCoordinate *>(link->getInboundJoint());
	// The base link has no inbound joint — drives/limits only apply to child links.
	ERR_FAIL_NULL_MSG(joint, "PhysX: articulation limit on the base link (no inbound joint).");
	const physx::PxArticulationAxis::Enum axis = _map_axis(p_axis);
	joint->setMotion(axis, physx::PxArticulationMotion::eLIMITED);
	joint->setLimitParams(axis, physx::PxArticulationLimit(p_low, p_high));
}

void PhysXArticulation3D::set_fix_base(bool p_fix) {
	ERR_FAIL_NULL(px_articulation);
	px_articulation->setArticulationFlag(physx::PxArticulationFlag::eFIX_BASE, p_fix);
}

void PhysXArticulation3D::wake_up() {
	if (px_articulation) {
		px_articulation->wakeUp();
	}
}

void PhysXArticulation3D::put_to_sleep() {
	if (px_articulation) {
		px_articulation->putToSleep();
	}
}

int PhysXArticulation3D::get_link_count() const {
	return (int)links.size();
}

Transform3D PhysXArticulation3D::get_link_transform(int p_index) const {
	physx::PxArticulationLink *link = const_cast<PhysXArticulation3D *>(this)->_link(p_index);
	if (!link) {
		return Transform3D();
	}
	const physx::PxTransform pose = link->getGlobalPose();
	Transform3D t;
	t.origin = Vector3(pose.p.x, pose.p.y, pose.p.z);
	t.basis = Basis(Quaternion(pose.q.x, pose.q.y, pose.q.z, pose.q.w));
	return t;
}

bool PhysXArticulation3D::is_sleeping() const {
	if (!px_articulation) {
		return true;
	}
	return px_articulation->isSleeping();
}

physx::PxArticulationLink *PhysXArticulation3D::_link(int p_index) const {
	if (p_index < 0 || p_index >= (int)links.size()) {
		return nullptr;
	}
	return links[p_index].link;
}

PhysXArticulation3D::LinkRecord *PhysXArticulation3D::_record(int p_index) {
	if (p_index < 0 || p_index >= (int)links.size()) {
		return nullptr;
	}
	return &links[p_index];
}

const PhysXArticulation3D::LinkRecord *PhysXArticulation3D::_record(int p_index) const {
	if (p_index < 0 || p_index >= (int)links.size()) {
		return nullptr;
	}
	return &links[p_index];
}

void PhysXArticulation3D::_apply_link_filter(LinkRecord &p_rec) const {
	if (!p_rec.shape) {
		return;
	}
	physx::PxFilterData filter_data;
	filter_data.word0 = p_rec.collision_layer;
	filter_data.word1 = p_rec.collision_mask;
	p_rec.shape->setSimulationFilterData(filter_data);
	p_rec.shape->setQueryFilterData(filter_data);
}
