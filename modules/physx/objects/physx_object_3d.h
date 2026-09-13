/**
 * @file physx_object_3d.h
 * @brief Base class for all physics objects (bodies, areas, soft bodies).
 *
 * PhysXObject3D is the common ancestor that every physics object inherits
 * from. It holds identity (RID, ObjectID), space membership, and collision
 * layer/mask bits. It does NOT own a PxActor — that is the derived class's
 * responsibility, because bodies, areas, and soft bodies use different
 * PxActor subtypes.
 */

#ifndef PHYSX_OBJECT_3D_H
#define PHYSX_OBJECT_3D_H

#include "core/object/object.h"
#include "physx_rid_owner.h"
#include "physx_server.h"

#include <PxPhysicsAPI.h>

class PhysXSpace3D;
class PxActor;

class PhysXObject3D {
public:
	/// Identifies the concrete subtype for safe downcasting (e.g. in
	/// PhysXQueryFilterCallback and PhysXSimulationEventCallback).
	enum ObjectType {
		OBJECT_TYPE_INVALID,
		OBJECT_TYPE_BODY,
		OBJECT_TYPE_AREA,
		OBJECT_TYPE_SOFT_BODY
	};

	explicit PhysXObject3D(ObjectType p_type) : type(p_type) {}
	virtual ~PhysXObject3D() {}

	// --- Identity ---
	ObjectType get_type() const { return type; }

	RID get_rid() const { return physx_rid.get_rid(); }
	void set_rid(const RID &p_rid) { physx_rid.set_rid(p_rid); }

	ObjectID get_instance_id() const { return instance_id; }
	void set_instance_id(ObjectID p_id) { instance_id = p_id; }

	// --- Space Management ---
	PhysXSpace3D *get_space() const { return space; }
	/// Pure virtual: bodies and areas handle space transitions differently
	/// (bodies add/remove a PxActor; areas manage a kinematic trigger actor).
	virtual void set_space(PhysXSpace3D *p_space) = 0;

	// --- Collision Layers ---
	// Stored in PxShape's SimulationFilterData as word0 (layer) / word1 (mask)
	// by PhysXShapedObject3D::update_shapes_collision_filter().
	uint32_t get_collision_layer() const { return collision_layer; }
	void set_collision_layer(uint32_t p_layer);

	uint32_t get_collision_mask() const { return collision_mask; }
	void set_collision_mask(uint32_t p_mask);

	// --- Ray pickable ---
	// Whether intersect_ray may report this object. Godot applies the flag to
	// raycasts ONLY — point/shape/sweep queries ignore it (reference:
	// godot_physics_3d gathers pickable objects for _intersect_ray alone).
	// Bodies default true; areas default false (each wrapper holds the member).
	virtual bool is_ray_pickable() const { return true; }

protected:
	ObjectType type = OBJECT_TYPE_INVALID;
	PhysXRIDOwner physx_rid;
	ObjectID instance_id;

	/// The physics world this object belongs to (null if not in a scene).
	PhysXSpace3D *space = nullptr;

	// Godot collision bits: layer = "what I am", mask = "what I collide with".
	uint32_t collision_layer = 1;
	uint32_t collision_mask = 1;

	/// Re-applies collision filter data to all shapes. Called when
	/// layer/mask/contact-notify changes. Implemented by shaped objects.
	virtual void _update_shapes() = 0;
};
#endif // PHYSX_OBJECT_3D_H
