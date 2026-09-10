/**
 * @file physx_project_settings.h
 * @brief Module settings under Project Settings → Physics → PhysX 3D.
 *
 * Settings are registered at MODULE_INITIALIZATION_LEVEL_SERVERS
 * (register_types.cpp) and read once in PhysXServer3D::init(), before any
 * PxScene exists.
 */

#ifndef PHYSX_PROJECT_SETTINGS_H
#define PHYSX_PROJECT_SETTINGS_H

class PhysXProjectSettings {
public:
	// physics/physx_3d/simulation/enhanced_determinism
	//
	// Sets PxSceneFlag::eENABLE_ENHANCED_DETERMINISM: the CPU simulation then
	// produces identical results across runs on the same binary/platform,
	// independent of the CPU worker count and API call order (it is NOT
	// cross-platform deterministic). Has a performance cost.
	//
	// GPU dynamics is never deterministic, so enabling this forces the CPU
	// solver even when a CUDA device is available.
	inline static bool enhanced_determinism = false;

	// physics/physx_3d/simulation/solver_type
	//
	// 0 = PGS (Projected Gauss-Seidel) -- the DEFAULT. PhysX's classic solver
	// and the stable, well-understood choice for this backend.
	// 1 = TGS (Temporal Gauss-Seidel) -- EXPERIMENTAL in this module. It can
	// hold joint chains steadier under sustained external forces (wind,
	// thrusters), but can be looser on joints in large mixed rigid-body
	// scenes and has seen far less validation here. Opt in per project if
	// you want to play with it; default remains PGS.
	inline static int solver_type = 0;

	// physics/physx_3d/simulation/allow_sleep
	//
	// When false, no rigid body ever sleeps (equivalent to RigidBody3D.can_sleep
	// off on every body). Useful for debugging and for setups that need every
	// body integrated every step.
	inline static bool allow_sleep = true;

	// physics/physx_3d/simulation/stabilization
	//
	// PxSceneFlag::eENABLE_STABILIZATION. Damps low-mass stacked/piled bodies
	// toward rest so they settle and can sleep instead of jittering; the same
	// mechanism Unity and Unreal call "stabilization".
	//
	// Default OFF in this module: the flag
	// applies an extra damping force to low-kinetic-energy bodies, which on a
	// sphere sliding down a ramp suppresses the angular acceleration from
	// contact friction — the sphere slides instead of rolling. Stacks are
	// already handled by the solver iteration count, so only enable this when
	// pile jitter is the greater evil. CPU-path only; silently ignored on the
	// GPU dynamics path (PhysX does not support it there).
	inline static bool stabilization = false;

	// physics/physx_3d/simulation/cpu_worker_threads
	//
	// 0 = auto (most of the machine for the CPU solver, a small pool for the
	// GPU path). A fixed value is useful for reproducible profiling. The
	// dispatcher is shared by every PxScene the server creates.
	inline static int cpu_worker_threads = 0;

	// physics/physx_3d/soft_body/mode
	//
	// Resolution for stock SoftBody3D bodies: 0 = Auto (GPU PxDeformableVolume
	// when the mesh tetrahedralizes and CUDA is available, else the CPU XPBD
	// solver — decided per body), 1 = CPU, 2 = GPU. A per-body override is
	// available via the node metadata "physx_soft_mode" = "cpu" / "gpu".
	// enhanced_determinism forces every soft body to CPU (no CUDA context).
	inline static int soft_body_mode = 0;

	static void register_settings();
	static void read_settings();
};

#endif // PHYSX_PROJECT_SETTINGS_H
