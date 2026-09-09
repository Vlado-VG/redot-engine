/**
 * @file physx_custom_geometry_callback.h
 * @brief Template base for shapes using PhysX's PxCustomGeometry system.
 *
 * PhysX 5.x supports PxCustomGeometry, which delegates collision and query
 * operations to a user-provided callback object (PxCustomGeometryExt::Callbacks).
 * This template provides RAII management of that callback: it lazily creates
 * the callback on first use, applies scale changes, and owns it via unique_ptr.
 *
 * Concrete subclasses (e.g. the cylinder shape) specialize this template with
 * their specific callback type and implement _create_callbacks() /
 * _apply_scale_to_callbacks().
 */

#ifndef PHYSX_CUSTOM_GEOMETRY_CALLBACK_H
#define PHYSX_CUSTOM_GEOMETRY_CALLBACK_H

#include "physx_shape_3d.h"
#include "../objects/physx_shaped_object_3d.h"
#include <extensions/PxCustomGeometryExt.h>
#include <memory> // For std::unique_ptr

template <typename CallbackT>
class PhysXCustomGeometryCallback : public PhysXShape3D {

public:
    PhysXCustomGeometryCallback() : PhysXShape3D() {}
    
    virtual ~PhysXCustomGeometryCallback() override {
        _release_geometry();
    }

    virtual bool get_physx_geometry(physx::PxGeometryHolder& holder, const physx::PxVec3& scale) const override {
        if (!_ensure_geometry()) {
            return false;
        }
        
        if (current_scale != scale) {
            current_scale = scale;
            _apply_scale_to_callbacks(scale);
        }
        
        holder.storeAny(geometry);
        return true;
    }

protected:
    // Memory automatically managed!
    mutable std::unique_ptr<CallbackT> callbacks;
    
    mutable physx::PxCustomGeometry geometry;
    mutable bool geometry_initialized = false;
    mutable physx::PxVec3 current_scale = physx::PxVec3(-1.0f, -1.0f, -1.0f);

    // Subclasses return a dynamically allocated, strongly-typed callback pointer
    virtual CallbackT* _create_callbacks() const = 0;
    virtual void _apply_scale_to_callbacks(const physx::PxVec3& scale) const = 0;

    bool _ensure_geometry() const {
        if (geometry_initialized && callbacks) {
            return true;
        }
        _release_geometry();

        CallbackT* new_callbacks = _create_callbacks();
        if (new_callbacks) {
            callbacks.reset(new_callbacks); // Take ownership
            geometry = physx::PxCustomGeometry(*callbacks);
            geometry_initialized = true;
            return true;
        }
        return false;
    }

    void _release_geometry() const {
        callbacks.reset(); // Safely deletes the callback, or does nothing if already null
        geometry_initialized = false;
        _invalidate_scale();
    }

    void _invalidate_scale() const { 
        current_scale = physx::PxVec3(-1.0f, -1.0f, -1.0f); 
    }

    // PhysXShape3D interface — detach PxShape from owners before callbacks are destroyed.
    // The callbacks are destroyed after this method returns (during derived-class
    // destructor), so the PxShape must be detached and released while the
    // callback pointer is still valid.
    virtual void detach_from_owners() override {
        // Ensure geometry is initialized so we have PxShape instances to detach.
        _ensure_geometry();

        // For each owner, detach the PxShape from the actor and nullify pointers.
        // The PxShape is released (refcount drops to 0, destroyed) here while
        // callbacks still exist. After this, the PxCustomGeometry's callback
        // pointer is no longer accessed, even though it will become dangling
        // when the PxShape is destroyed.
        MutexLock lock(owners_mutex);
        for (KeyValue<PhysXShapedObject3D *, int> &E : owners) {
            PhysXShapedObject3D *owner = E.key;
            owner->detach_shape(this);
        }
    }
};
#endif // PHYSX_CUSTOM_GEOMETRY_CALLBACK_H