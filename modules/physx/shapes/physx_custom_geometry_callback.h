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
 *
 * GPU DYNAMICS NOTE (verified — do not re-investigate without cause):
 * PhysX documentation historically described PxCustomGeometry as unsupported
 * by the GPU pipeline, suggesting bodies with custom geometry could lose
 * collision in GODOT_PHYSX_GPU scenes. Tested against the vendored SDK with
 * GPU dynamics active (CUDA device present, scene with eENABLE_GPU_DYNAMICS +
 * eGPU broadphase): a dynamic cylinder and a dynamic cone both fall, contact,
 * and rest at their exact half-heights on a static floor — PhysX generates
 * such pairs' contacts on the CPU while they remain in the GPU broadphase.
 * The suite's shape-family pair matrix (PHYSX-SHAPE-P-*) guards this
 * permanently. If custom-geometry contact cost ever matters in GPU scenes,
 * cooking a convex approximation is the available optimization — a behavior
 * change (faceted silhouette), not a correctness fix.
 */

#ifndef PHYSX_CUSTOM_GEOMETRY_CALLBACK_H
#define PHYSX_CUSTOM_GEOMETRY_CALLBACK_H

#include "physx_shape_3d.h"
#include "../objects/physx_shaped_object_3d.h"
#include <extensions/PxCustomGeometryExt.h>
#include <memory> // For std::unique_ptr

#include "core/templates/local_vector.h"

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
        // One callback instance per distinct scale: the shape blueprint is
        // shared across bodies, and a single shared callback mutated per
        // request made two differently-scaled owners fight over its
        // radius/height (last write won for both). Each PxShape now holds a
        // PxCustomGeometry bound to its own scale's instance.
        CallbackT* cb = _callback_for_scale(scale);
        if (!cb) {
            return false;
        }
        holder.storeAny(physx::PxCustomGeometry(*cb));
        return true;
    }

protected:
    // Memory automatically managed!
    mutable std::unique_ptr<CallbackT> callbacks;

    // Additional per-scale instances (the primary `callbacks` anchors the
    // geometry-initialized state; these serve owners at other scales).
    mutable LocalVector<std::pair<physx::PxVec3, CallbackT*>> scaled_instances;

    mutable physx::PxCustomGeometry geometry;
    mutable bool geometry_initialized = false;

    // Subclasses return a dynamically allocated, strongly-typed callback pointer
    virtual CallbackT* _create_callbacks() const = 0;
    virtual void _apply_scale_to_callbacks(CallbackT& cb, const physx::PxVec3& scale) const = 0;
    // Unscaled params (radius/height) from the current data members.
    virtual void _apply_params_to_callbacks(CallbackT& cb) const = 0;

    CallbackT* _callback_for_scale(const physx::PxVec3& scale) const {
        for (uint32_t i = 0; i < scaled_instances.size(); i++) {
            if (scaled_instances[i].first == scale) {
                return scaled_instances[i].second;
            }
        }
        CallbackT* cb = _create_callbacks();
        if (!cb) {
            return nullptr;
        }
        _apply_scale_to_callbacks(*cb, scale);
        scaled_instances.push_back({ scale, cb });
        return cb;
    }

    // Re-push the current data (then each instance's own scale) into every
    // live instance -- called by derived set_data() implementations.
    void _refresh_instances() const {
        if (callbacks) {
            _apply_params_to_callbacks(*callbacks);
        }
        for (uint32_t i = 0; i < scaled_instances.size(); i++) {
            _apply_scale_to_callbacks(*scaled_instances[i].second, scaled_instances[i].first);
        }
    }

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
        for (uint32_t i = 0; i < scaled_instances.size(); i++) {
            delete scaled_instances[i].second;
        }
        scaled_instances.clear();
        geometry_initialized = false;
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