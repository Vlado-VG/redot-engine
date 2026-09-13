/**
 * @file physx_project_settings.cpp
 * @brief Registers and reads the physics/physx_3d/* project settings.
 */

#include "physx_project_settings.h"

#include "core/config/project_settings.h"

void PhysXProjectSettings::register_settings() {
	GLOBAL_DEF(PropertyInfo(Variant::BOOL, "physics/physx_3d/simulation/enhanced_determinism"), false);
	GLOBAL_DEF(PropertyInfo(Variant::INT, "physics/physx_3d/simulation/solver_type", PROPERTY_HINT_ENUM, "PGS,TGS"), 0);
	GLOBAL_DEF(PropertyInfo(Variant::BOOL, "physics/physx_3d/simulation/allow_sleep"), true);
	GLOBAL_DEF(PropertyInfo(Variant::BOOL, "physics/physx_3d/simulation/stabilization"), false);
	// Optional async stepping: step() only kicks PxScene::simulate() and the
	// fetch happens in sync() at the START of the next tick, so the solve
	// overlaps the rest of the frame instead of blocking it. Default off —
	// see the header for the behavioral caveats. Read live (per server step),
	// so it can be toggled at runtime.
	GLOBAL_DEF(PropertyInfo(Variant::BOOL, "physics/physx_3d/simulation/async_step"), false);
	GLOBAL_DEF(PropertyInfo(Variant::INT, "physics/physx_3d/simulation/cpu_worker_threads", PROPERTY_HINT_RANGE, U"0,32,1"), 0);
	GLOBAL_DEF(PropertyInfo(Variant::INT, "physics/physx_3d/soft_body/mode", PROPERTY_HINT_ENUM, "Auto,CPU,GPU"), 0);
}

void PhysXProjectSettings::read_settings() {
	enhanced_determinism = GLOBAL_GET("physics/physx_3d/simulation/enhanced_determinism");
	solver_type = GLOBAL_GET("physics/physx_3d/simulation/solver_type");
	allow_sleep = GLOBAL_GET("physics/physx_3d/simulation/allow_sleep");
	stabilization = GLOBAL_GET("physics/physx_3d/simulation/stabilization");
	cpu_worker_threads = GLOBAL_GET("physics/physx_3d/simulation/cpu_worker_threads");
	soft_body_mode = GLOBAL_GET("physics/physx_3d/soft_body/mode");
}
