// Foam advection pass -- one thread per diffuse slot. Foam never joins the MPM
// solve: inside the fluid (grid mass present) it is strongly dragged toward the
// fluid velocity while buoyancy lifts it (bubbles/foam rising); outside it
// flies ballistically under gravity with mild air drag (spray). Lifetime decays
// in real seconds; spent slots park at life <= 0 until the spawn pass recycles
// them. The alive count feeds the readback so the node shows only live foam.
//
// Runs once per frame (frame dt from Params.foam_b.y), after the spawn pass.
#[compute]
#version 450

#include "mpm_fluid_inc.glsl"

layout(local_size_x = GROUP) in;

void main() {
	uint id = gl_GlobalInvocationID.x;
	if (id >= uint(max(FOAM_CAP, 1.0))) {
		return;
	}
	float life = foam[id].pos_life.w;
	if (life <= 0.0) {
		return;
	}
	vec3 p = foam[id].pos_life.xyz;
	vec3 v = foam[id].vel_kind.xyz;

	// Kill foam that escaped the sim domain entirely (it would hang in the air
	// sampling a zero grid forever).
	if (any(lessThan(p, bmin.xyz)) || any(greaterThan(p, bmax.xyz))) {
		foam[id].pos_life.w = 0.0;
		return;
	}

	float dt = max(FRAME_DT, 1e-4);
	if (sample_grid_mass(p) > 0.25 * PMASS) {
		// In the fluid: relative-velocity drag toward the fluid motion plus a
		// buoyant rise (FOAM_BUOY is a fraction of the gravity magnitude).
		vec3 target = sample_grid_v(p) - GRAV * FOAM_BUOY;
		float blend = 1.0 - exp(-max(FOAM_DRAG, 0.01) * 8.0 * dt);
		v = mix(v, target, blend);
	} else {
		// Air: ballistic with mild aerodynamic drag.
		v += GRAV * dt;
		v *= 1.0 / (1.0 + 0.2 * dt);
	}
	p += v * dt;
	life -= dt;

	if (life <= 0.0) {
		foam[id].pos_life.w = 0.0; // park; the spawn pass recycles the slot
		return;
	}
	foam[id].pos_life = vec4(p, life);
	foam[id].vel_kind.xyz = v;
	atomicAdd(foam_meta.y, 1u);
}
