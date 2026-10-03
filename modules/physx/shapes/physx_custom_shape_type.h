#ifndef PHYSX_CUSTOM_SHAPE_TYPE_H
#define PHYSX_CUSTOM_SHAPE_TYPE_H

#include "core/string/string_name.h"
#include "core/templates/hash_map.h"

#include "physx_shape_3d.h"

#include <functional>
#include <memory>

class PhysXCustomShapeType : public PhysXShape3D {
private:
	StringName current_type;
	std::unique_ptr<PhysXShape3D, void (*)(PhysXShape3D *)> internal_shape{ nullptr, memdelete_shape };

	static void memdelete_shape(PhysXShape3D *p_shape) {
		if (p_shape) {
			memdelete(p_shape);
		}
	}

public:

	using FactoryFunc = std::function<std::unique_ptr<PhysXShape3D, void (*)(PhysXShape3D *)>()>;
// Static registry so it's shared across all custom shapes
    static HashMap<StringName, FactoryFunc> shape_factories;

    /// Registers the built-in custom shapes ("cone"). Called once from
    /// PhysXServer3D::init(); idempotent.
    static void register_builtin_shapes();



    PhysXCustomShapeType();
    ~PhysXCustomShapeType() override;

    // To Godot, this is a custom shape
    virtual PhysicsServer3D::ShapeType get_type() const override {
        return PhysicsServer3D::SHAPE_CUSTOM;
    }

    virtual bool is_convex() const override;

    virtual void set_data(const Variant &p_data) override;
    virtual Variant get_data() const override;

    virtual AABB get_aabb() const override;

    virtual bool get_physx_geometry(physx::PxGeometryHolder& holder, const physx::PxVec3& scale) const override;

    /// Keeps the inner shape's margin in sync with the wrapper's (the wrapper
    /// margin governs the attached PxShape's contact offset).
    virtual void set_margin(float p_margin) override;
    
    // Backend access to check the exact shape type internally
    StringName get_custom_type() const { return current_type; }
};
#endif // PHYSX_CUSTOM_SHAPE_TYPE_H