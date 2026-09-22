// Foam spawn pass -- potential-based diffuse-particle generation, evaluated per
// fluid particle off this frame's final MPM state. Mirrors NVIDIA's
// PxDiffuseParticleParams model (and Ihmsen et al. 2013, "Unified Spray, Foam
// and Air Bubbles"): a splash/foam site shows high kinetic energy, a density
// error against the rest value, and particles flying apart (positive grid
// divergence). One thread per fluid particle; spawns land in a fixed-capacity
// ring -- foam_meta.x is a monotonic counter, so heavy spawn pressure recycles
// the oldest slots round-robin instead of flooding past the budget.
//
// Runs once per frame after the substep loop (visual layer, 60 Hz is plenty);
// never touches the MPM grid or particle state.
#[compute]
#version 450

#include "mpm_fluid_inc.glsl"

layout(local_size_x = GROUP) in;

// Weight set matching the PBD path's PxDiffuseParticleParams (kinetic energy /
// pressure / divergence). Kept as constants in v1 -- the exposed tuning knobs
// are threshold/lifetime/buoyancy, shared with the CUDA backend's properties.
const float W_KE = 0.1;
const float W_PRESSURE = 1.0;
const float W_DIVERGENCE = 0.1;

void main() {
	uint id = gl_GlobalInvocationID.x;
	if (id >= uint(PCOUNT)) {
		return;
	}
	vec3 x = particles[id].x_d.xyz;
	vec3 v = particles[id].v.xyz;

	// Grid velocity divergence by central differences of the trilinear field.
	float e = DX;
	float div = (dot(sample_grid_v(x + vec3(e, 0.0, 0.0)) - sample_grid_v(x - vec3(e, 0.0, 0.0)), vec3(1.0, 0.0, 0.0)) +
			dot(sample_grid_v(x + vec3(0.0, e, 0.0)) - sample_grid_v(x - vec3(0.0, e, 0.0)), vec3(0.0, 1.0, 0.0)) +
			dot(sample_grid_v(x + vec3(0.0, 0.0, e)) - sample_grid_v(x - vec3(0.0, 0.0, e)), vec3(0.0, 0.0, 1.0))) /
			(2.0 * e);

	// x_d.w carries this frame's sampled grid density (written by P2G mass).
	float dens_err = abs(RHO0 - particles[id].x_d.w) / max(RHO0, 1e-3);
	float potential = W_KE * 0.5 * dot(v, v) + W_PRESSURE * dens_err + W_DIVERGENCE * abs(div);
	if (potential <= FOAM_THRESH) {
		return;
	}

	uint slot = atomicAdd(foam_meta.x, 1u) % uint(max(FOAM_CAP, 1.0));
	foam[slot].pos_life = vec4(x, FOAM_LIFE);
	foam[slot].vel_kind = vec4(v, 0.0);
}
