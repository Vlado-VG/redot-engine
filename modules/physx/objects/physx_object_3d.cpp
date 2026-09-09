#include "physx_object_3d.h"
#include "spaces/physx_space_3d.h"

void PhysXObject3D::set_collision_layer(uint32_t p_layer) {
    if (collision_layer == p_layer) {
        return;
    }
    collision_layer = p_layer;
    _update_shapes(); // Tell the derived class to update its PxShapes
}

void PhysXObject3D::set_collision_mask(uint32_t p_mask) {
    if (collision_mask == p_mask) {
        return;
    }
    collision_mask = p_mask;
    _update_shapes(); // Tell the derived class to update its PxShapes
}