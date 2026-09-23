/**************************************************************************/
/*  physx_flow_simulation_3d.cpp                                          */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,         */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.*/
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                  */
/**************************************************************************/

#include "physx_flow_simulation_3d.h"

#ifdef GODOT_PHYSX_FLOW

#include "physx_flow_collider_3d.h"
#include "physx_flow_emitter_3d.h"

#include "core/io/image.h"
#include "core/object/class_db.h"
#include "core/os/os.h"
#include "scene/3d/fog_volume.h"
#include "scene/3d/physics/collision_object_3d.h"
#include "scene/3d/physics/collision_shape_3d.h"
#include "scene/main/viewport.h"
#include "scene/resources/3d/box_shape_3d.h"
#include "scene/resources/3d/concave_polygon_shape_3d.h"
#include "scene/resources/3d/convex_polygon_shape_3d.h"
#include "scene/resources/3d/sphere_shape_3d.h"
#include "scene/resources/3d/world_3d.h"
#include "scene/resources/environment.h"
#include "scene/resources/image_texture.h"
#include "scene/resources/material.h"
#include "scene/resources/shader.h"
#include "servers/physics_3d/physics_server_3d.h"
#include "servers/rendering/rendering_server.h"

namespace {
// Fog shader for the Flow volume. The readback texture packs the Flow grid's
// density texture as RGBA half: A = smoke density, RGB = the solver's
// colorized channels (fire colormap output when combustion is active -- see
// flow_simulation.cpp's decode; channel semantics validated by the readback
// test). Emission fires only where RGB is non-zero, so a pure-smoke scene
// shades as plain albedo fog and a burning one glows without any extra pass.
const char *FLOW_FOG_SHADER_SOURCE = R"(
shader_type fog;

uniform sampler3D flow_tex;
uniform float density_scale = 24.0;
uniform vec3 smoke_albedo = vec3(0.9);
uniform bool fire_emission = true;
uniform float emission_strength = 6.0;

void fog() {
	vec4 s = texture(flow_tex, UVW);
	float d = s.a;
	float edge = pow(clamp(-2.0 * SDF / min(min(SIZE.x, SIZE.y), SIZE.z), 0.0, 1.0), 1.0);
	DENSITY = d * density_scale * edge;
	ALBEDO = smoke_albedo;
	if (fire_emission) {
		EMISSION = s.rgb * emission_strength * edge;
	}
}
)";
} // namespace

PhysXFlowSimulation3D::~PhysXFlowSimulation3D() {
	if (simulation != nullptr) {
		memdelete(simulation);
		simulation = nullptr;
	}
}

uint64_t PhysXFlowSimulation3D::_luid_for(ObjectID p_id) {
	if (uint64_t *existing = node_luids.getptr(p_id)) {
		return *existing;
	}
	const uint64_t luid = next_luid++;
	node_luids[p_id] = luid;
	return luid;
}

void PhysXFlowSimulation3D::_ensure_simulation() {
	if (simulation == nullptr) {
		simulation = memnew(FlowSimulation);
	}
	if (sim_settings_dirty) {
		FlowSimGridConfig config;
		config.max_blocks = (uint32_t)MAX(64, max_blocks);
		simulation->configure(config);
		sim_settings_dirty = false;
	}
	available = simulation->is_available();
}

// ---- Emitters -------------------------------------------------------------

void PhysXFlowSimulation3D::_resolve_emitters(LocalVector<FlowEmitterData> &r_emitters) {
	for (int i = 0; i < emitters.size(); i++) {
		Node *n = get_node_or_null(emitters[i]);
		Node3D *node = Object::cast_to<Node3D>(n);
		if (node == nullptr) {
			continue;
		}
		if (PhysXFlowEmitter3D *fe = Object::cast_to<PhysXFlowEmitter3D>(node)) {
			if (!fe->enabled) {
				continue;
			}
			FlowEmitterData e;
			e.luid = _luid_for(fe->get_instance_id());
			e.enabled = true;
			e.is_box = fe->shape == PhysXFlowEmitter3D::SHAPE_BOX;
			e.world_xform = fe->get_global_transform();
			e.velocity = fe->get_global_transform().basis.get_rotation_quaternion().xform(fe->velocity);
			e.radius = fe->radius;
			e.half_size = fe->size * 0.5f;
			e.divergence = fe->divergence;
			e.temperature = fe->temperature;
			e.fuel = fe->fuel;
			e.smoke = fe->smoke;
			e.couple_rate = fe->couple_rate;
			e.sub_steps = (uint32_t)fe->sub_steps;
			r_emitters.push_back(e);
		} else {
			// Plain Node3D fallback: a small smoke sphere at the node, aimed
			// by its transform (same pattern as PhysXGas3D's emitter list).
			FlowEmitterData e;
			e.luid = _luid_for(node->get_instance_id());
			e.enabled = true;
			e.is_box = false;
			e.world_xform = node->get_global_transform();
			e.velocity = node->get_global_transform().basis.get_rotation_quaternion().xform(Vector3(0, 2, 0));
			e.radius = 0.4f;
			e.temperature = 0.0f;
			e.fuel = 0.0f;
			e.smoke = 1.0f;
			r_emitters.push_back(e);
		}
	}
}

// ---- Colliders (analytic nodes + PhysicsServer3D bridge) -------------------

// Builds a triangle-soup/mesh or analytic-box record for a physics-server
// shape. Meshes are cached by shape RID: the soup is built once per shape
// resource and reused across bodies/frames (static geometry never rebuilds).
bool PhysXFlowSimulation3D::_build_server_shape_mesh(const RID &p_shape_rid, CachedShapeMesh &r_cache) {
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	const PhysicsServer3D::ShapeType type = ps->shape_get_type(p_shape_rid);
	// Data layouts follow THIS fork's PhysXServer3D::shape_get_data contract
	// (verified against modules/physx/shapes/*.cpp): box = half-extents,
	// convex = raw point array, concave = {faces: triangle soup}, capsule /
	// cylinder = {height, radius}.
	switch (type) {
		case PhysicsServer3D::SHAPE_BOX: {
			const Vector3 half_extents = ps->shape_get_data(p_shape_rid);
			r_cache.analytic_box = true;
			r_cache.box_half_extents = half_extents;
			return true;
		}
		case PhysicsServer3D::SHAPE_SPHERE: {
			const real_t radius = ps->shape_get_data(p_shape_rid);
			// Flow has no native sphere collider -- generate an icosphere
			// soup once per shape (shared generator).
			r_cache.analytic_box = false;
			PhysXFlowCollider3D::build_icosphere_soup((float)radius, 2, r_cache.positions);
			return r_cache.positions.size() > 0;
		}
		case PhysicsServer3D::SHAPE_CONVEX_POLYGON: {
			// Raw hull point cloud (this server stores points, not faces):
			// approximate with its bounding box -- an honest v1 limitation
			// (quickhull on the cloud is future polish; see implementation
			// notes). A wrong-silhouette box beats no collision for smoke.
			const PackedVector3Array points = ps->shape_get_data(p_shape_rid);
			if (points.size() < 3) {
				return false;
			}
			AABB aabb(points[0], Vector3());
			for (int i = 1; i < points.size(); i++) {
				aabb.expand_to(points[i]);
			}
			r_cache.analytic_box = true;
			r_cache.box_half_extents = aabb.size * 0.5f;
			return true;
		}
		case PhysicsServer3D::SHAPE_CONCAVE_POLYGON: {
			const Dictionary data = ps->shape_get_data(p_shape_rid);
			const PackedVector3Array faces = data["faces"]; // triangle soup a,b,c per face
			if (faces.size() < 3 || faces.size() % 3 != 0) {
				return false;
			}
			r_cache.analytic_box = false;
			r_cache.positions.clear();
			r_cache.positions.reserve(faces.size() * 3);
			for (const Vector3 &v : faces) {
				r_cache.positions.push_back(v.x);
				r_cache.positions.push_back(v.y);
				r_cache.positions.push_back(v.z);
			}
			return true;
		}
		case PhysicsServer3D::SHAPE_CAPSULE:
		case PhysicsServer3D::SHAPE_CYLINDER: {
			const Dictionary data = ps->shape_get_data(p_shape_rid);
			const float radius = (float)(double)data["radius"];
			const float height = (float)(double)data["height"];
			r_cache.analytic_box = true;
			r_cache.box_half_extents = Vector3(radius, height * 0.5f, radius);
			return true;
		}
		default:
			// World-boundary planes are infinite and heightmaps have no
			// compact representation here -- skip both (documented).
			return false;
	}
}

void PhysXFlowSimulation3D::_resolve_colliders(LocalVector<FlowColliderData> &r_colliders, double p_delta) {
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();

	for (int i = 0; i < colliders.size(); i++) {
		Node *n = get_node_or_null(colliders[i]);
		if (n == nullptr) {
			continue;
		}

		if (PhysXFlowCollider3D *fc = Object::cast_to<PhysXFlowCollider3D>(n)) {
			if (!fc->enabled) {
				continue;
			}
			const Transform3D xform = fc->get_global_transform();
			// Finite-difference velocity estimate between steps.
			if (fc->auto_velocity) {
				if (fc->velocity_tracked && p_delta > 0.0) {
					const Vector3 dp = xform.origin - fc->last_world_xform.origin;
					fc->tracked_linear_velocity = dp / (float)p_delta;
					// Angular estimate from basis change (small-angle): axis
					// of the rotation between the two bases.
					const Basis db = xform.basis * fc->last_world_xform.basis.transposed();
					const Quaternion dq = db.get_rotation_quaternion();
					const float angle = 2.0f * Math::acos(CLAMP(Math::abs(dq.w), -1.0f, 1.0f));
					if (angle > 0.0001f && angle < 3.0f) {
						const Vector3 axis = Vector3(dq.x, dq.y, dq.z) / MAX(Math::sin(angle * 0.5f), 1e-6f);
						fc->tracked_angular_velocity = axis * (angle / (float)p_delta);
					} else {
						fc->tracked_angular_velocity = Vector3();
					}
				}
				fc->last_world_xform = xform;
				fc->velocity_tracked = true;
			}

			FlowColliderData c;
			c.luid = _luid_for(fc->get_instance_id());
			c.world_xform = xform;
			c.linear_velocity = fc->auto_velocity ? fc->tracked_linear_velocity : fc->manual_linear_velocity;
			c.angular_velocity = fc->auto_velocity ? fc->tracked_angular_velocity : fc->manual_angular_velocity;
			if (fc->shape == PhysXFlowCollider3D::SHAPE_BOX) {
				c.is_mesh = false;
				c.half_size = fc->size * 0.5f;
			} else {
				if (!fc->_ensure_mesh()) {
					continue;
				}
				c.is_mesh = true;
				c.mesh_positions = fc->mesh_positions; // copied into the record's storage
			}
			r_colliders.push_back(c);
			continue;
		}

		if (CollisionObject3D *co = Object::cast_to<CollisionObject3D>(n)) {
			const RID body = co->get_rid();
			if (!body.is_valid()) {
				continue;
			}
			// Real velocities from the server (works with any physics backend;
			// with this module's PhysX backend active this is PhysX state).
			const Transform3D body_xform = ps->body_get_state(body, PhysicsServer3D::BODY_STATE_TRANSFORM);
			Vector3 linear = ps->body_get_state(body, PhysicsServer3D::BODY_STATE_LINEAR_VELOCITY);
			Vector3 angular = ps->body_get_state(body, PhysicsServer3D::BODY_STATE_ANGULAR_VELOCITY);
			const PhysicsServer3D::BodyMode body_mode = ps->body_get_mode(body);
			if (body_mode == PhysicsServer3D::BODY_MODE_STATIC) {
				linear = Vector3();
				angular = Vector3();
			} else if (body_mode == PhysicsServer3D::BODY_MODE_KINEMATIC && linear.is_zero_approx() && angular.is_zero_approx()) {
				// Kinematic bodies don't always report velocity: estimate by
				// finite difference on the body transform.
				if (HashMap<ObjectID, Transform3D>::Iterator it = kinematic_last_transforms.find(co->get_instance_id()); it) {
					const double dt = MAX(p_delta, 1e-4);
					linear = (body_xform.origin - it->value.origin) / (float)dt;
					// (angular estimate omitted for kinematics -- translation
					// coupling is the case that matters for stirring smoke)
				}
				kinematic_last_transforms.insert(co->get_instance_id(), body_xform);
			}

			const int shape_count = ps->body_get_shape_count(body);
			for (int s = 0; s < shape_count; s++) {
				const RID shape = ps->body_get_shape(body, s);
				if (!shape.is_valid()) {
					continue;
				}
				CachedShapeMesh *cache = shape_mesh_cache.getptr(shape);
				if (cache == nullptr) {
					CachedShapeMesh built;
					if (!_build_server_shape_mesh(shape, built)) {
						continue;
					}
					shape_mesh_cache.insert(shape, built);
					cache = shape_mesh_cache.getptr(shape); // refetch: insert may rehash
				}
				FlowColliderData c;
				c.luid = ((uint64_t)shape.get_id() << 1) ^ 0x100000000ull; // stable per shape
				const Transform3D shape_local = ps->body_get_shape_transform(body, s);
				c.world_xform = body_xform * shape_local;
				c.linear_velocity = linear;
				c.angular_velocity = angular;
				if (cache->analytic_box) {
					c.is_mesh = false;
					c.half_size = cache->box_half_extents;
				} else {
					c.is_mesh = true;
					c.mesh_positions = cache->positions;
				}
				r_colliders.push_back(c);
			}
			continue;
		}

		if (CollisionShape3D *cs = Object::cast_to<CollisionShape3D>(n)) {
			// Standalone CollisionShape3D (no body): static analytic box from
			// the resource-side shape AABB (the server exposes no shape AABB
			// query in this fork; Shape3D::get_aabb is the same data).
			Ref<Shape3D> s = cs->get_shape();
			if (s.is_null()) {
				continue;
			}
			const float enclosing = (float)s->get_enclosing_radius();
			FlowColliderData c;
			c.luid = _luid_for(cs->get_instance_id());
			c.world_xform = cs->get_global_transform();
			c.is_mesh = false;
			c.half_size = Vector3(enclosing, enclosing, enclosing); // bound cube (Shape3D exposes only an enclosing radius here)
			r_colliders.push_back(c);
			continue;
		}

		// Fallback: small box at the node.
		Node3D *node = Object::cast_to<Node3D>(n);
		if (node == nullptr) {
			continue;
		}
		FlowColliderData c;
		c.luid = _luid_for(node->get_instance_id());
		c.world_xform = node->get_global_transform();
		c.is_mesh = false;
		c.half_size = Vector3(collider_fallback_size, collider_fallback_size, collider_fallback_size);
		r_colliders.push_back(c);
	}
}

// ---- Transient (event) emitters ------------------------------------------------

void PhysXFlowSimulation3D::add_transient_emitter(const Vector3 &p_world_position, float p_radius,
		float p_smoke, float p_temperature, float p_fuel, float p_velocity, float p_lifetime) {
	TransientEmitter t;
	t.position = p_world_position;
	t.radius = MAX(p_radius, 0.01f);
	t.smoke = MAX(p_smoke, 0.0f);
	t.temperature = MAX(p_temperature, 0.0f);
	t.fuel = MAX(p_fuel, 0.0f);
	t.velocity = p_velocity;
	t.lifetime = MAX(p_lifetime, 0.05f);
	t.age = 0.0f;
	t.luid = next_transient_luid++;
	// Ring capacity: destructive scenes can emit hundreds of events per
	// second; cap the working set (oldest dropped -- visually the right
	// degradation) rather than grow unbounded.
	if (transient_emitters.size() >= 256) {
		transient_emitters.remove_at(0);
	}
	transient_emitters.push_back(t);
}

// Ages transient emitters (in SIMULATED time -- paused sims freeze their
// events) and feeds the live ones into the resolved emitter list with a
// linear decay so bursts fade rather than pop.
void PhysXFlowSimulation3D::_resolve_transients(double p_delta, LocalVector<FlowEmitterData> &r_emitters) {
	for (uint32_t i = 0; i < transient_emitters.size();) {
		TransientEmitter &t = transient_emitters[i];
		t.age += (float)p_delta;
		if (t.age >= t.lifetime) {
			transient_emitters.remove_at_unordered(i);
			continue; // do not advance: another element swapped into place
		}
		const float fade = 1.0f - t.age / t.lifetime;
		FlowEmitterData e;
		e.luid = t.luid; // stable per event: Flow matches injected params by luid
		e.enabled = true;
		e.is_box = false;
		e.world_xform = Transform3D(Basis(), t.position);
		e.velocity = Vector3(0, t.velocity, 0);
		e.radius = t.radius * (1.0f + 0.5f * (1.0f - fade)); // puff spreads as it fades
		e.temperature = t.temperature * fade;
		e.fuel = t.fuel * fade;
		e.smoke = t.smoke * fade;
		e.couple_rate = 2.0f;
		e.sub_steps = 1;
		r_emitters.push_back(e);
		i++;
	}
}

// ---- Step -------------------------------------------------------------------

void PhysXFlowSimulation3D::_step(double p_delta) {
	_ensure_simulation();
	if (simulation == nullptr || !simulation->is_available() || !enabled) {
		return;
	}

	FlowSimSettings settings;
	settings.cell_size = cell_size;
	settings.steps_per_second = steps_per_second;
	settings.time_scale = time_scale;
	settings.max_steps_per_simulate = (uint32_t)MAX(1, max_steps_per_simulate);
	settings.velocity_substeps = (uint32_t)MAX(1, velocity_substeps);
	settings.block_min_lifetime = (uint32_t)MAX(0, block_min_lifetime);
	settings.simulate_when_paused = simulate_when_paused;
	settings.gravity = gravity;
	settings.buoyancy_per_temp = buoyancy_per_temp;
	settings.buoyancy_per_smoke = buoyancy_per_smoke;
	settings.cooling_rate = cooling_rate;
	settings.smoke_fade = smoke_fade;
	settings.temperature_fade = temperature_fade;
	settings.fuel_fade = fuel_fade;
	settings.combustion_enabled = combustion_enabled;
	settings.vorticity_force_scale = vorticity;
	settings.pressure_enabled = pressure_projection;

	LocalVector<FlowEmitterData> resolved_emitters;
	LocalVector<FlowColliderData> resolved_colliders;
	if (!paused || simulate_when_paused) {
		_resolve_emitters(resolved_emitters);
		_resolve_transients(p_delta, resolved_emitters);
	}
	_resolve_colliders(resolved_colliders, p_delta);

	sim_time_accum += p_delta; // monotonic sim clock owned by this node
	simulation->step(sim_time_accum, (float)p_delta,
			resolved_emitters.ptr(), resolved_emitters.size(),
			resolved_colliders.ptr(), resolved_colliders.size(),
			settings, false, paused);

	// Consume any completed readback (typically 1-2 frames behind).
	FlowReadbackFrame frame;
	if (simulation->poll_readback(frame)) {
		last_frame = frame;
	}
	if (debug_stats) {
		simulation->set_diag_flags(FlowSimulation::DIAG_SPARSE_LAYOUT | FlowSimulation::DIAG_READBACK_STATS);
	} else {
		simulation->set_diag_flags(FlowSimulation::DIAG_NONE);
	}
}

// ---- Render -------------------------------------------------------------------

void PhysXFlowSimulation3D::_ensure_fog_volume() {
	if (fog_volume != nullptr) {
		return;
	}
	if (flow_shader.is_null()) {
		flow_shader.instantiate();
		flow_shader->set_code(FLOW_FOG_SHADER_SOURCE);
	}
	flow_material.instantiate();
	flow_material->set_shader(flow_shader);
	flow_material->set_shader_parameter("density_scale", fog_density);
	flow_material->set_shader_parameter("smoke_albedo", smoke_albedo);
	flow_material->set_shader_parameter("fire_emission", fire_emission);
	flow_material->set_shader_parameter("emission_strength", fire_emission_strength);

	fog_volume = memnew(FogVolume);
	fog_volume->set_shape(RS::FOG_VOLUME_SHAPE_BOX);
	fog_volume->set_material(flow_material);
	// The volume follows the active sparse block bounds in world space (set
	// per update), so keep it out of this node's transform chain.
	fog_volume->set_as_top_level(true);
	add_child(fog_volume, false, INTERNAL_MODE_BACK);
}

void PhysXFlowSimulation3D::_update_volumetric_render() {
	if (!last_frame.valid || last_frame.dims.x <= 0) {
		return;
	}
	const Vector3i dims = last_frame.dims;

	// One Image slice per Z layer; voxels are already RGBA half (4x16-bit).
	Vector<Ref<Image>> slices;
	slices.resize(dims.z);
	const int slice_voxels = dims.x * dims.y;
	const int slice_u16 = slice_voxels * 4;
	for (int z = 0; z < dims.z; z++) {
		Vector<uint8_t> bytes;
		bytes.resize(slice_u16 * (int)sizeof(uint16_t));
		memcpy(bytes.ptrw(), last_frame.voxels_rgbaH.ptr() + (size_t)z * slice_u16, bytes.size());
		slices.write[z] = Image::create_from_data(dims.x, dims.y, false, Image::FORMAT_RGBAH, bytes);
	}

	if (density_texture.is_null() || density_texture_dims != dims) {
		density_texture.instantiate();
		density_texture->create(Image::FORMAT_RGBAH, dims.x, dims.y, dims.z, false, slices);
		density_texture_dims = dims;
		flow_material->set_shader_parameter("flow_tex", density_texture);
	} else {
		density_texture->update(slices);
	}

	fog_volume->set_size(last_frame.world_size.maxf(0.05f));
	fog_volume->set_position(last_frame.world_min + last_frame.world_size * 0.5f);
}

void PhysXFlowSimulation3D::_update_render() {
	if (volumetric_render && simulation != nullptr && simulation->is_available()) {
		_ensure_fog_volume();
		_update_volumetric_render();
	}
}

// ---- Notifications -------------------------------------------------------------

void PhysXFlowSimulation3D::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_WORLD: {
			_ensure_simulation();
			set_physics_process_internal(true);
		} break;
		case NOTIFICATION_EXIT_WORLD: {
			set_physics_process_internal(false);
			if (fog_volume != nullptr) {
				fog_volume->queue_free();
				fog_volume = nullptr;
			}
			density_texture.unref();
			density_texture_dims = Vector3i();
			// The FlowSimulation (and its grid) is kept alive: blocks and GPU
			// state survive scene reloads, matching how the shared runtime
			// does. reset_simulation() clears them on demand.
		} break;
		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
			_step(get_physics_process_delta_time());
			_update_render();
		} break;
		default:
			break;
	}
}

// ---- Public control ---------------------------------------------------------------

void PhysXFlowSimulation3D::reset_simulation() {
	if (simulation != nullptr) {
		simulation->reset();
	}
	last_frame = FlowReadbackFrame();
}

void PhysXFlowSimulation3D::step_once() {
	if (!paused) {
		return; // already stepping every tick
	}
	const double dt = 1.0 / 60.0;
	_step(dt);
	_update_render();
}

bool PhysXFlowSimulation3D::is_flow_available() const {
	return simulation != nullptr && simulation->is_available();
}

Dictionary PhysXFlowSimulation3D::get_diagnostics() const {
	Dictionary out;
	out["available"] = simulation != nullptr && simulation->is_available();
	if (simulation == nullptr || !simulation->is_available()) {
		out["unavailable_reason"] = simulation != nullptr ? simulation->get_unavailable_reason() : String("not initialized");
		return out;
	}
	FlowRuntime::Stats stats;
	if (FlowRuntime *rt = FlowRuntime::get_singleton()) {
		stats = rt->get_stats();
	}
	out["backend"] = stats.backend_name;
	out["simulations"] = stats.simulation_count;
	out["active_blocks"] = simulation->get_active_block_count();
	out["max_blocks"] = max_blocks;
	out["frames_simulated"] = simulation->get_simulated_frame_count();
	out["sim_time"] = simulation->get_sim_time();
	out["device_memory_mb"] = stats.device_memory_bytes / (1024.0 * 1024.0);
	out["readback_memory_mb"] = stats.readback_memory_bytes / (1024.0 * 1024.0);
	const FlowSimulation::TimingStats timing = simulation->get_timing_stats();
	out["last_submit_usec"] = timing.last_submit_usec;
	out["last_flush_usec"] = timing.last_flush_usec;
	out["last_decode_usec"] = timing.last_decode_usec;
	return out;
}

int PhysXFlowSimulation3D::get_active_block_count() const {
	return simulation != nullptr ? (int)simulation->get_active_block_count() : 0;
}

PackedStringArray PhysXFlowSimulation3D::get_configuration_warnings() const {
	PackedStringArray warnings = Node3D::get_configuration_warnings();
	if (volumetric_render && OS::get_singleton()->get_current_rendering_method() != "forward_plus") {
		warnings.push_back(RTR("PhysXFlowSimulation3D's volumetric render needs the Forward+ renderer (it draws through a FogVolume)."));
	}
	return warnings;
}

// ---- Property plumbing ---------------------------------------------------------

void PhysXFlowSimulation3D::set_enabled(bool p_enabled) {
	enabled = p_enabled;
	if (simulation != nullptr && !enabled) {
		simulation->reset();
	}
}

void PhysXFlowSimulation3D::set_paused(bool p_paused) { paused = p_paused; }

void PhysXFlowSimulation3D::set_max_blocks(int p_blocks) {
	p_blocks = CLAMP(p_blocks, 64, 262144);
	if (p_blocks != max_blocks) {
		max_blocks = p_blocks;
		sim_settings_dirty = true;
	}
}

void PhysXFlowSimulation3D::set_cell_size(float p_size) {
	cell_size = MAX(p_size, 0.0f);
}
void PhysXFlowSimulation3D::set_steps_per_second(float p_sps) { steps_per_second = MAX(p_sps, 1.0f); }
void PhysXFlowSimulation3D::set_time_scale(float p_scale) { time_scale = MAX(p_scale, 0.0f); }
void PhysXFlowSimulation3D::set_max_steps_per_simulate(int p_steps) { max_steps_per_simulate = CLAMP(p_steps, 1, 16); }
void PhysXFlowSimulation3D::set_velocity_substeps(int p_steps) { velocity_substeps = CLAMP(p_steps, 1, 8); }
void PhysXFlowSimulation3D::set_block_min_lifetime(int p_seconds) { block_min_lifetime = CLAMP(p_seconds, 0, 64); }
void PhysXFlowSimulation3D::set_simulate_when_paused(bool p_enabled) { simulate_when_paused = p_enabled; }
void PhysXFlowSimulation3D::set_combustion_enabled(bool p_enabled) { combustion_enabled = p_enabled; }
void PhysXFlowSimulation3D::set_gravity(const Vector3 &p_gravity) { gravity = p_gravity; }
void PhysXFlowSimulation3D::set_buoyancy_per_temp(float p_b) { buoyancy_per_temp = MAX(p_b, 0.0f); }
void PhysXFlowSimulation3D::set_buoyancy_per_smoke(float p_b) { buoyancy_per_smoke = MAX(p_b, 0.0f); }
void PhysXFlowSimulation3D::set_cooling_rate(float p_rate) { cooling_rate = MAX(p_rate, 0.0f); }
void PhysXFlowSimulation3D::set_smoke_fade(float p_fade) { smoke_fade = MAX(p_fade, 0.0f); }
void PhysXFlowSimulation3D::set_temperature_fade(float p_fade) { temperature_fade = MAX(p_fade, 0.0f); }
void PhysXFlowSimulation3D::set_fuel_fade(float p_fade) { fuel_fade = MAX(p_fade, 0.0f); }
void PhysXFlowSimulation3D::set_vorticity(float p_strength) { vorticity = MAX(p_strength, 0.0f); }
void PhysXFlowSimulation3D::set_pressure_projection(bool p_enabled) { pressure_projection = p_enabled; }

void PhysXFlowSimulation3D::set_volumetric_render(bool p_enabled) {
	volumetric_render = p_enabled;
	if (!volumetric_render && fog_volume != nullptr) {
		fog_volume->queue_free();
		fog_volume = nullptr;
		density_texture.unref();
		density_texture_dims = Vector3i();
	}
}

void PhysXFlowSimulation3D::set_fog_density(float p_density) {
	fog_density = MAX(p_density, 0.0f);
	if (flow_material.is_valid()) {
		flow_material->set_shader_parameter("density_scale", fog_density);
	}
}

void PhysXFlowSimulation3D::set_smoke_albedo(const Color &p_color) {
	smoke_albedo = p_color;
	if (flow_material.is_valid()) {
		flow_material->set_shader_parameter("smoke_albedo", Vector3(p_color.r, p_color.g, p_color.b));
	}
}

void PhysXFlowSimulation3D::set_fire_emission(bool p_enabled) {
	fire_emission = p_enabled;
	if (flow_material.is_valid()) {
		flow_material->set_shader_parameter("fire_emission", fire_emission);
	}
}

void PhysXFlowSimulation3D::set_fire_emission_strength(float p_strength) {
	fire_emission_strength = MAX(p_strength, 0.0f);
	if (flow_material.is_valid()) {
		flow_material->set_shader_parameter("emission_strength", fire_emission_strength);
	}
}

void PhysXFlowSimulation3D::set_debug_stats(bool p_enabled) { debug_stats = p_enabled; }

void PhysXFlowSimulation3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_enabled", "enabled"), &PhysXFlowSimulation3D::set_enabled);
	ClassDB::bind_method(D_METHOD("get_enabled"), &PhysXFlowSimulation3D::get_enabled);
	ClassDB::bind_method(D_METHOD("set_paused", "paused"), &PhysXFlowSimulation3D::set_paused);
	ClassDB::bind_method(D_METHOD("get_paused"), &PhysXFlowSimulation3D::get_paused);
	ClassDB::bind_method(D_METHOD("reset_simulation"), &PhysXFlowSimulation3D::reset_simulation);
	ClassDB::bind_method(D_METHOD("step_once"), &PhysXFlowSimulation3D::step_once);
	ClassDB::bind_method(D_METHOD("add_transient_emitter", "world_position", "radius", "smoke", "temperature", "fuel", "velocity", "lifetime"),
			&PhysXFlowSimulation3D::add_transient_emitter, DEFVAL(1.5));
	ClassDB::bind_method(D_METHOD("is_flow_available"), &PhysXFlowSimulation3D::is_flow_available);
	ClassDB::bind_method(D_METHOD("get_diagnostics"), &PhysXFlowSimulation3D::get_diagnostics);
	ClassDB::bind_method(D_METHOD("get_active_block_count"), &PhysXFlowSimulation3D::get_active_block_count);
	ClassDB::bind_method(D_METHOD("get_active_bounds_min"), &PhysXFlowSimulation3D::get_active_bounds_min);
	ClassDB::bind_method(D_METHOD("get_active_bounds_size"), &PhysXFlowSimulation3D::get_active_bounds_size);
	ClassDB::bind_method(D_METHOD("get_last_max_smoke"), &PhysXFlowSimulation3D::get_last_max_smoke);
	ClassDB::bind_method(D_METHOD("get_last_max_temperature"), &PhysXFlowSimulation3D::get_last_max_temperature);

	ClassDB::bind_method(D_METHOD("set_max_blocks", "blocks"), &PhysXFlowSimulation3D::set_max_blocks);
	ClassDB::bind_method(D_METHOD("get_max_blocks"), &PhysXFlowSimulation3D::get_max_blocks);
	ClassDB::bind_method(D_METHOD("set_cell_size", "size"), &PhysXFlowSimulation3D::set_cell_size);
	ClassDB::bind_method(D_METHOD("get_cell_size"), &PhysXFlowSimulation3D::get_cell_size);
	ClassDB::bind_method(D_METHOD("set_steps_per_second", "steps"), &PhysXFlowSimulation3D::set_steps_per_second);
	ClassDB::bind_method(D_METHOD("get_steps_per_second"), &PhysXFlowSimulation3D::get_steps_per_second);
	ClassDB::bind_method(D_METHOD("set_time_scale", "scale"), &PhysXFlowSimulation3D::set_time_scale);
	ClassDB::bind_method(D_METHOD("get_time_scale"), &PhysXFlowSimulation3D::get_time_scale);
	ClassDB::bind_method(D_METHOD("set_max_steps_per_simulate", "steps"), &PhysXFlowSimulation3D::set_max_steps_per_simulate);
	ClassDB::bind_method(D_METHOD("get_max_steps_per_simulate"), &PhysXFlowSimulation3D::get_max_steps_per_simulate);
	ClassDB::bind_method(D_METHOD("set_velocity_substeps", "steps"), &PhysXFlowSimulation3D::set_velocity_substeps);
	ClassDB::bind_method(D_METHOD("get_velocity_substeps"), &PhysXFlowSimulation3D::get_velocity_substeps);
	ClassDB::bind_method(D_METHOD("set_block_min_lifetime", "seconds"), &PhysXFlowSimulation3D::set_block_min_lifetime);
	ClassDB::bind_method(D_METHOD("get_block_min_lifetime"), &PhysXFlowSimulation3D::get_block_min_lifetime);
	ClassDB::bind_method(D_METHOD("set_simulate_when_paused", "enabled"), &PhysXFlowSimulation3D::set_simulate_when_paused);
	ClassDB::bind_method(D_METHOD("get_simulate_when_paused"), &PhysXFlowSimulation3D::get_simulate_when_paused);
	ClassDB::bind_method(D_METHOD("set_combustion_enabled", "enabled"), &PhysXFlowSimulation3D::set_combustion_enabled);
	ClassDB::bind_method(D_METHOD("get_combustion_enabled"), &PhysXFlowSimulation3D::get_combustion_enabled);
	ClassDB::bind_method(D_METHOD("set_gravity", "gravity"), &PhysXFlowSimulation3D::set_gravity);
	ClassDB::bind_method(D_METHOD("get_gravity"), &PhysXFlowSimulation3D::get_gravity);
	ClassDB::bind_method(D_METHOD("set_buoyancy_per_temp", "buoyancy"), &PhysXFlowSimulation3D::set_buoyancy_per_temp);
	ClassDB::bind_method(D_METHOD("get_buoyancy_per_temp"), &PhysXFlowSimulation3D::get_buoyancy_per_temp);
	ClassDB::bind_method(D_METHOD("set_buoyancy_per_smoke", "buoyancy"), &PhysXFlowSimulation3D::set_buoyancy_per_smoke);
	ClassDB::bind_method(D_METHOD("get_buoyancy_per_smoke"), &PhysXFlowSimulation3D::get_buoyancy_per_smoke);
	ClassDB::bind_method(D_METHOD("set_cooling_rate", "rate"), &PhysXFlowSimulation3D::set_cooling_rate);
	ClassDB::bind_method(D_METHOD("get_cooling_rate"), &PhysXFlowSimulation3D::get_cooling_rate);
	ClassDB::bind_method(D_METHOD("set_smoke_fade", "fade"), &PhysXFlowSimulation3D::set_smoke_fade);
	ClassDB::bind_method(D_METHOD("get_smoke_fade"), &PhysXFlowSimulation3D::get_smoke_fade);
	ClassDB::bind_method(D_METHOD("set_temperature_fade", "fade"), &PhysXFlowSimulation3D::set_temperature_fade);
	ClassDB::bind_method(D_METHOD("get_temperature_fade"), &PhysXFlowSimulation3D::get_temperature_fade);
	ClassDB::bind_method(D_METHOD("set_fuel_fade", "fade"), &PhysXFlowSimulation3D::set_fuel_fade);
	ClassDB::bind_method(D_METHOD("get_fuel_fade"), &PhysXFlowSimulation3D::get_fuel_fade);
	ClassDB::bind_method(D_METHOD("set_vorticity", "strength"), &PhysXFlowSimulation3D::set_vorticity);
	ClassDB::bind_method(D_METHOD("get_vorticity"), &PhysXFlowSimulation3D::get_vorticity);
	ClassDB::bind_method(D_METHOD("set_pressure_projection", "enabled"), &PhysXFlowSimulation3D::set_pressure_projection);
	ClassDB::bind_method(D_METHOD("get_pressure_projection"), &PhysXFlowSimulation3D::get_pressure_projection);

	ClassDB::bind_method(D_METHOD("set_emitters", "paths"), &PhysXFlowSimulation3D::set_emitters);
	ClassDB::bind_method(D_METHOD("get_emitters"), &PhysXFlowSimulation3D::get_emitters);
	ClassDB::bind_method(D_METHOD("set_colliders", "paths"), &PhysXFlowSimulation3D::set_colliders);
	ClassDB::bind_method(D_METHOD("get_colliders"), &PhysXFlowSimulation3D::get_colliders);
	ClassDB::bind_method(D_METHOD("set_collider_fallback_size", "size"), &PhysXFlowSimulation3D::set_collider_fallback_size);
	ClassDB::bind_method(D_METHOD("get_collider_fallback_size"), &PhysXFlowSimulation3D::get_collider_fallback_size);

	ClassDB::bind_method(D_METHOD("set_volumetric_render", "enabled"), &PhysXFlowSimulation3D::set_volumetric_render);
	ClassDB::bind_method(D_METHOD("get_volumetric_render"), &PhysXFlowSimulation3D::get_volumetric_render);
	ClassDB::bind_method(D_METHOD("set_fog_density", "density"), &PhysXFlowSimulation3D::set_fog_density);
	ClassDB::bind_method(D_METHOD("get_fog_density"), &PhysXFlowSimulation3D::get_fog_density);
	ClassDB::bind_method(D_METHOD("set_smoke_albedo", "color"), &PhysXFlowSimulation3D::set_smoke_albedo);
	ClassDB::bind_method(D_METHOD("get_smoke_albedo"), &PhysXFlowSimulation3D::get_smoke_albedo);
	ClassDB::bind_method(D_METHOD("set_fire_emission", "enabled"), &PhysXFlowSimulation3D::set_fire_emission);
	ClassDB::bind_method(D_METHOD("get_fire_emission"), &PhysXFlowSimulation3D::get_fire_emission);
	ClassDB::bind_method(D_METHOD("set_fire_emission_strength", "strength"), &PhysXFlowSimulation3D::set_fire_emission_strength);
	ClassDB::bind_method(D_METHOD("get_fire_emission_strength"), &PhysXFlowSimulation3D::get_fire_emission_strength);
	ClassDB::bind_method(D_METHOD("set_debug_stats", "enabled"), &PhysXFlowSimulation3D::set_debug_stats);
	ClassDB::bind_method(D_METHOD("get_debug_stats"), &PhysXFlowSimulation3D::get_debug_stats);

	ADD_GROUP("Simulation", "");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "enabled"), "set_enabled", "get_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "paused"), "set_paused", "get_paused");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "max_blocks", PROPERTY_HINT_RANGE, "64,262144,256"), "set_max_blocks", "get_max_blocks");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "cell_size", PROPERTY_HINT_RANGE, "0.0,4.0,0.01,suffix:m"), "set_cell_size", "get_cell_size");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "steps_per_second", PROPERTY_HINT_RANGE, "10,240,1"), "set_steps_per_second", "get_steps_per_second");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "time_scale", PROPERTY_HINT_RANGE, "0.0,4.0,0.05"), "set_time_scale", "get_time_scale");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "max_steps_per_simulate", PROPERTY_HINT_RANGE, "1,16,1"), "set_max_steps_per_simulate", "get_max_steps_per_simulate");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "velocity_substeps", PROPERTY_HINT_RANGE, "1,8,1"), "set_velocity_substeps", "get_velocity_substeps");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "block_min_lifetime", PROPERTY_HINT_RANGE, "0,64,1,suffix:s"), "set_block_min_lifetime", "get_block_min_lifetime");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "simulate_when_paused"), "set_simulate_when_paused", "get_simulate_when_paused");
	ADD_GROUP("Combustion", "");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "combustion_enabled"), "set_combustion_enabled", "get_combustion_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "gravity", PROPERTY_HINT_NONE, "suffix:m/s^2"), "set_gravity", "get_gravity");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "buoyancy_per_temp", PROPERTY_HINT_RANGE, "0.0,10.0,0.05"), "set_buoyancy_per_temp", "get_buoyancy_per_temp");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "buoyancy_per_smoke", PROPERTY_HINT_RANGE, "0.0,10.0,0.05"), "set_buoyancy_per_smoke", "get_buoyancy_per_smoke");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "cooling_rate", PROPERTY_HINT_RANGE, "0.0,10.0,0.05"), "set_cooling_rate", "get_cooling_rate");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "smoke_fade", PROPERTY_HINT_RANGE, "0.0,2.0,0.01"), "set_smoke_fade", "get_smoke_fade");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "temperature_fade", PROPERTY_HINT_RANGE, "0.0,2.0,0.01"), "set_temperature_fade", "get_temperature_fade");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "fuel_fade", PROPERTY_HINT_RANGE, "0.0,2.0,0.01"), "set_fuel_fade", "get_fuel_fade");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "vorticity", PROPERTY_HINT_RANGE, "0.0,4.0,0.05"), "set_vorticity", "get_vorticity");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "pressure_projection"), "set_pressure_projection", "get_pressure_projection");
	ADD_GROUP("Emitters", "emitter");
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "emitters", PROPERTY_HINT_ARRAY_TYPE, "NodePath"), "set_emitters", "get_emitters");
	ADD_GROUP("Colliders", "collider");
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "colliders", PROPERTY_HINT_ARRAY_TYPE, "NodePath"), "set_colliders", "get_colliders");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "collider_fallback_size", PROPERTY_HINT_RANGE, "0.001,8.0,0.01,suffix:m"), "set_collider_fallback_size", "get_collider_fallback_size");
	ADD_GROUP("Rendering", "");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "volumetric_render"), "set_volumetric_render", "get_volumetric_render");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "fog_density", PROPERTY_HINT_RANGE, "0.0,64.0,0.1"), "set_fog_density", "get_fog_density");
	ADD_PROPERTY(PropertyInfo(Variant::COLOR, "smoke_albedo"), "set_smoke_albedo", "get_smoke_albedo");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "fire_emission"), "set_fire_emission", "get_fire_emission");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "fire_emission_strength", PROPERTY_HINT_RANGE, "0.0,32.0,0.1"), "set_fire_emission_strength", "get_fire_emission_strength");
	ADD_GROUP("Debug", "debug");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "debug_stats"), "set_debug_stats", "get_debug_stats");
}

#endif // GODOT_PHYSX_FLOW
