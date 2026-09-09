/**
 * @file physx_shaped_object_3d.h
 * @brief Base class for objects that own collision shapes (bodies, areas).
 *
 * PhysXShapedObject3D extends PhysXObject3D with the ability to attach
 * PhysXShape3D resources. Each attached shape creates a PxShape on the
 * underlying PxRigidActor. The class manages the shape lifecycle:
 * creation, transform updates, body-scale application, collision filter
 * propagation, and recreation when the actor is replaced.
 *
 * Shape ownership model:
 *   - PhysXShape3D (the RID-tracked resource) is shared — many bodies can
 *     reference the same shape definition.
 *   - PxShape (the PhysX instance) is per-actor — each body gets its own
 *     copy, created by PhysXShape3D::create_shape().
 *   - After attachShape(), the local PxShape reference is released (refcount
 *     drops to 1, owned by the actor). Detaching or releasing the actor
 *     destroys it.
 */

#ifndef PHYSX_SHAPED_OBJECT_3D_H
#define PHYSX_SHAPED_OBJECT_3D_H

#include "physx_object_3d.h"
#include "shapes/physx_shape_3d.h"
#include "core/templates/local_vector.h"
#include "core/math/transform_3d.h"

#include <PxPhysicsAPI.h>

class PhysXShapedObject3D : public PhysXObject3D {
public:
    explicit PhysXShapedObject3D(ObjectType p_type);
	virtual ~PhysXShapedObject3D();

	/** @brief Converts a Godot Transform3D to a PhysX PxTransform. */
    static physx::PxTransform to_physx_transform(const Transform3D &p_transform);

	/**
	 * @brief Computes the total geometry scale for an attached shape: the
	 * object's body scale composed with the per-shape transform scale.
	 *
	 * Godot's server contract carries per-shape scale inside the shape
	 * transform basis (the standard idiom: a 1x1 trimesh quad whose shape
	 * offset scales it (50,1,50) into a floor), and PhysX requires that scale
	 * to be baked into the geometry itself. Mirrored (negative) scales are
	 * absorbed with abs(), since PhysX geometries reject negative scales.
	 */
	Vector3 _shape_geometry_scale(const Transform3D &p_shape_transform) const;

	// --- Shape Observer ---
	// Called by PhysXShape3D::_notify_shape_changed when a shape's geometry
	// is modified (e.g. user changes box size in the editor). Finds all
	// attached instances of that shape and recreates their PxGeometry.
	virtual void shape_changed(PhysXShape3D *p_shape);

	// --- Shape Destruction Notification ---
	// Called by PhysXShape3D destructor to nullify the shape pointer,
	// preventing use-after-free when the body is destroyed after the shape.
	void nullify_shape(PhysXShape3D *p_shape);

	// --- Shape Detachment ---
	// Called by PhysXShape3D::detach_from_owners() for custom-geometry shapes
	// to detach and release all PxShape instances referencing a specific
	// PhysXShape3D, preventing use-after-free when the shape holds external
	// pointers (e.g. PxCustomGeometry callbacks).
	void detach_shape(PhysXShape3D *p_shape);

	// --- Shape Management ---
	void add_shape(PhysXShape3D *p_shape, const Transform3D &p_transform, bool p_disabled);
	void remove_shape(int p_index);
	void remove_shape(PhysXShape3D *p_shape);
	void set_shape_transform(int p_index, const Transform3D &p_transform);

	int get_shape_count() const { return (int)shapes.size(); }
	RID get_shape_rid(int p_index) const;
	Transform3D get_shape_transform(int p_index) const;

	/**
	 * @brief Resolves a PhysX PxShape pointer to its body-local index, or -1.
	 *
	 * Used by the simulation event callback to map a trigger/contact shape back
	 * to the Godot-side shape index without relying on PxShape::userData (which
	 * points at the shared PhysXShape3D blueprint, not a per-instance struct).
	 */
	int find_shape_index(const physx::PxShape *p_px_shape) const;

	// --- PhysX Access ---
	physx::PxRigidActor *get_rigid_actor() const { return px_actor; }

protected:
	/** @brief Re-applies collision layer/mask/notify to all attached PxShapes. */
    void update_shapes_collision_filter();

	/** @brief Computes the word3 module flags (contact-notify / IS_AREA) for this object's shapes. */
	uint32_t shape_filter_flags() const;

	/** @brief Toggles the contact-notify marker (written to PxFilterData.word3). */
	void set_contact_notify(bool p_enable) { contact_notify = p_enable; }

	/**
	 * @brief Returns the PxMaterial to attach to newly-created PxShapes.
	 *
	 * The base implementation returns the server's shared default material. The
	 * body override lazily creates and owns a private material so that per-body
	 * bounce/friction mutations don't leak across bodies that would otherwise
	 * all share the one default material.
	 */
	virtual physx::PxMaterial *_get_shape_material();

	/**
	 * @brief Re-creates all PxShape instances against the current px_actor.
	 *
	 * Call this after the actor has been replaced (e.g. static↔dynamic mode
	 * switch), because releasing the old PxRigidActor releases its attached
	 * PxShapes, leaving the AttachedShape records with dangling pointers.
	 */
    void rebuild_shapes();

	// --- Shape lifecycle hooks (overridden by derived classes like PhysXBody3D) ---
	virtual void _on_shape_added();
	virtual void _on_shape_removed();
	virtual void _on_shape_geometry_changed();
	virtual void _on_shape_transform_changed();

	/// The underlying PhysX actor (PxRigidStatic or PxRigidDynamic).
	/// Placed in protected so derived classes (bodies, areas) share a single
	/// actor reference instead of shadowing with a separate member.
	physx::PxRigidActor *px_actor = nullptr;

	/// Per-object material (body-owned). nullptr for objects (e.g. areas) that
	/// use the server default. Set by overriding _get_shape_material().
	physx::PxMaterial *px_material = nullptr;

	/// Tracks one shape instance attached to this object.
	/// Protected so derived classes (areas) can access shape fields for
	/// trigger flag configuration.
	struct AttachedShape {
		PhysXShape3D *shareable_shape = nullptr; ///< The shared RID resource.
		physx::PxShape *px_shape = nullptr;      ///< The per-actor PhysX instance.
		/// Optional second, non-trigger simulation shape (areas only, REG-0011):
		/// PhysX 5 does not report trigger-vs-trigger pairs, so a monitorable
		/// area needs a plain simulation shape for other areas' triggers to
		/// fire against. nullptr for bodies and for non-monitorable areas.
		physx::PxShape *detection_shape = nullptr;
		Transform3D relative_transform;          ///< Local transform relative to body center.
		bool disabled = false;
		bool trigger = false;
	};

	/// All shapes attached to this object.
	LocalVector<AttachedShape> shapes;

	/// Cached body scale (applied to shape geometry + local pose origins).
	Vector3 body_scale = Vector3(1, 1, 1);

	/// True when contact-point notifications are requested.
	bool contact_notify = false;
};
#endif // PHYSX_SHAPED_OBJECT_3D_H
