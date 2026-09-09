#include "physx_heightmap_shape_3d.h"
#include "../physx_server.h"
#include <cooking/PxCooking.h>
#include "core/templates/local_vector.h"

PhysXHeightMapShape3D::~PhysXHeightMapShape3D() {
	_release_height_field();
}

AABB PhysXHeightMapShape3D::_calculate_aabb() const {
	if (width <= 1 || depth <= 1) {
		return AABB();
	}

	// Godot heightmaps are centered around the local origin
	float x_size = (float)(width - 1);
	float z_size = (float)(depth - 1);
	float y_size = max_height - min_height;

	Vector3 position(-x_size * 0.5f, min_height, -z_size * 0.5f);
	Vector3 size(x_size, y_size, z_size);

	return AABB(position, size);
}

physx::PxTransform PhysXHeightMapShape3D::get_local_pose() const {
	// PhysX heightfields originate at a corner (0,0,0). Godot expects them
	// centered on the local origin. Apply the offset so the shape is centered
	// when the body places it at the origin.
	float x_offset = -(float)(width - 1) * 0.5f;
	float z_offset = -(float)(depth - 1) * 0.5f;
	return physx::PxTransform(physx::PxVec3(x_offset, 0.0f, z_offset));
}

void PhysXHeightMapShape3D::set_data(const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::DICTIONARY);
	Dictionary data = p_data;

	ERR_FAIL_COND(!data.has("width") || !data.has("depth") || !data.has("heights"));

	width = data["width"];
	depth = data["depth"];
	heights = data["heights"];

	ERR_FAIL_COND(width <= 1 || depth <= 1);
	ERR_FAIL_COND(heights.size() != width * depth);

	// Calculate bounds and quantization scale
	min_height = heights[0];
	max_height = heights[0];
	float max_abs_height = 0.0f;

	for (int i = 0; i < heights.size(); ++i) {
		float h = heights[i];
		if (h < min_height) min_height = h;
		if (h > max_height) max_height = h;

		float abs_h = Math::abs(h);
		if (abs_h > max_abs_height) max_abs_height = abs_h;
	}

	// Quantize float domain into 16-bit integer domain safely
	// We map the absolute maximum height to 32767 to ensure Y=0 exactly equals PxI16(0)
	if (max_abs_height > 0.0f) {
		height_scale = max_abs_height / 32767.0f;
	} else {
		height_scale = 1.0f; // Completely flat
	}

	aabb = _calculate_aabb();
	_release_height_field();
}

Variant PhysXHeightMapShape3D::get_data() const {
	Dictionary data;
	data["width"] = width;
	data["depth"] = depth;
	data["heights"] = heights;
	return data;
}

bool PhysXHeightMapShape3D::get_physx_geometry(physx::PxGeometryHolder &holder, const physx::PxVec3 &scale) const {
	if (!_ensure_physx_height_field()) {
		return false;
	}
    ERR_FAIL_COND_V(scale.x <= 0.0f, false);
    ERR_FAIL_COND_V(scale.y <= 0.0f, false);
    ERR_FAIL_COND_V(scale.z <= 0.0f, false);

	holder.storeAny(
		physx::PxHeightFieldGeometry(
			height_field,
			physx::PxMeshGeometryFlags(),
			height_scale * scale.y, // Apply quantization restore scale + godot scale
			scale.x,
			scale.z
		)
	);

	return true;
}

void PhysXHeightMapShape3D::_release_height_field() {
	if (height_field) {
		height_field->release();
		height_field = nullptr;
        _notify_shape_changed();
	}
}

bool PhysXHeightMapShape3D::_ensure_physx_height_field() const {
	if (height_field) {
		return true;
	}

	if (width <= 1 || depth <= 1 || heights.size() == 0) {
		return false;
	}

	// 1. Prepare PhysX data array
	// PhysX heightfield grid uses 16-bit samples
	LocalVector<physx::PxHeightFieldSample> samples;
	samples.resize(width * depth);

	// Remap Godot's Z*width+X into PhysX's Row*columns+column
	// In PhysX, X maps to rows, Z maps to columns.
	for (int column = 0; column < depth; ++column) {
		for (int row = 0; row < width; ++row) {
			int godot_index = column * width + row;
			int physx_index = row * depth + column; // Swapped iteration alignment

			float h = heights[godot_index];
			
			// Quantize height
			physx::PxI16 h_quantized = (physx::PxI16)CLAMP(Math::round(h / height_scale), -32768.0, 32767.0);

			samples[physx_index].height = h_quantized;
			samples[physx_index].materialIndex0 = 0;
			samples[physx_index].materialIndex1 = 0;
			samples[physx_index].clearTessFlag(); 
		}
	}

	// 2. Set up the mesh descriptor
	physx::PxHeightFieldDesc hf_desc;
	hf_desc.format = physx::PxHeightFieldFormat::eS16_TM;
	hf_desc.nbRows = width;
	hf_desc.nbColumns = depth;
	hf_desc.samples.data = samples.ptr();
	hf_desc.samples.stride = sizeof(physx::PxHeightFieldSample);
	// thickness removed for PhysX 5 compatibility

	physx::PxPhysics &physics = PhysXServer3D::get_singleton()->get_physics();

	ERR_FAIL_NULL_V_MSG(&physics, false, "PhysX PxPhysics is not initialized.");
    ERR_FAIL_COND_V_MSG(!hf_desc.isValid(), false, "Invalid PhysX heightfield descriptor.");

	// 3. Create the HeightField (Stateless global function)
	height_field = PxCreateHeightField(hf_desc, physics.getPhysicsInsertionCallback());

	if (!height_field) {
		ERR_PRINT("PhysX failed to create height map.");
		return false;
	}

	return true;
}