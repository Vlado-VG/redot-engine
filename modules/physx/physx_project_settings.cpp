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
