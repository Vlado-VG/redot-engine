#include "physx_custom_shape_type.h"
#include "physx_cone_shape_3d.h" // Register new custom shapes here

PhysXCustomShapeType::PhysXCustomShapeType() {}

PhysXCustomShapeType::~PhysXCustomShapeType() {}

// Define the static registry
HashMap<StringName, PhysXCustomShapeType::FactoryFunc> PhysXCustomShapeType::shape_factories;

void PhysXCustomShapeType::register_builtin_shapes() {
	if (!shape_factories.has("cone")) {
		shape_factories["cone"] = []() -> std::unique_ptr<PhysXShape3D, void (*)(PhysXShape3D *)> {
			return std::unique_ptr<PhysXShape3D, void (*)(PhysXShape3D *)>(memnew(PhysXConeShape3D), memdelete_shape);
		};
	}
}

bool PhysXCustomShapeType::is_convex() const {
    if (internal_shape) {
        return internal_shape->is_convex();
    }
    return false;
}

void PhysXCustomShapeType::set_data(const Variant &p_data) {
    ERR_FAIL_COND(p_data.get_type() != Variant::DICTIONARY);
    Dictionary d = p_data;

    // Default to empty StringName if not found
    StringName new_type = d.get("type", StringName());

    if (new_type != current_type) {
        current_type = new_type;

        // Instant factory dispatch
        if (shape_factories.has(current_type)) {
            internal_shape = shape_factories[current_type]();
        } else {
            internal_shape.reset();
        }
    }

    if (internal_shape) {
        internal_shape->set_data(p_data);
    }

    _notify_shape_changed();
}

Variant PhysXCustomShapeType::get_data() const {
    if (internal_shape) {
        Dictionary data = internal_shape->get_data();
        data["type"] = current_type; // Cleanly inserts the StringName
        return data;
    }
    
    Dictionary empty_data;
    empty_data["type"] = StringName(); // Represents our "Unknown" state
    return empty_data;
}

AABB PhysXCustomShapeType::get_aabb() const {
    if (internal_shape) {
        return internal_shape->get_aabb();
    }
    return AABB();
}

bool PhysXCustomShapeType::get_physx_geometry(physx::PxGeometryHolder& holder, const physx::PxVec3& scale) const {
    if (internal_shape) {
        return internal_shape->get_physx_geometry(holder, scale);
    }
    return false;
}