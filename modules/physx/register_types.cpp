/**
 * @file register_types.cpp
 * @brief Module entry point — registers PhysX as a selectable 3D physics backend.
 *
 * At MODULE_INITIALIZATION_LEVEL_SERVERS, this file registers the PhysX server
 * factory with PhysicsServer3DManager. The user selects it in Project Settings
 * under "Physics > 3D > Physics Engine" (the dropdown is populated automatically
 * by PhysicsServer3DManager::on_servers_changed()).
 */

#include "register_types.h"

#include "core/object/class_db.h"
#include "core/config/project_settings.h"

#include "physx_server.h"
#include "physx_project_settings.h"
#include "servers/physics_3d/physics_server_3d_wrap_mt.h"
#include "objects/physx_direct_body_state_3d.h"
#include "nodes/physx_chunk_emitter_3d.h"
#include "nodes/physx_cloth_3d.h"
#include "nodes/physx_particle_fluid_3d.h"

#ifdef TOOLS_ENABLED
#include "editor/physx_editor_plugin.h"
#endif

/**
 * @brief Factory function that creates the PhysX physics server.
 *
 * The returned PhysicsServer3DWrapMT is owned by PhysicsServer3DManager after
 * registration; do not memdelete it here.
 */
static PhysicsServer3D *create_physx_server() {

#ifdef THREADS_ENABLED
	bool run_on_separate_thread = GLOBAL_GET("physics/3d/run_on_separate_thread");
#else
	bool run_on_separate_thread = false;
#endif

	// The inner PhysXServer3D is created with a default constructor; threading
	// is handled by the wrapper, not by the server itself.
	PhysXServer3D *physics_server = memnew(PhysXServer3D);

	return memnew(PhysicsServer3DWrapMT(physics_server, run_on_separate_thread));
}

void initialize_physx_module(ModuleInitializationLevel p_level) {
	if (p_level == MODULE_INITIALIZATION_LEVEL_SERVERS) {
		// Register the physics/physx_3d/* settings before the server factory
		// runs, so init() reads the user's values (not the defaults).
		PhysXProjectSettings::register_settings();

		// Class registration for Godot (and GDScript/Doctool)
		ClassDB::register_class<PhysXServer3D>();
		ClassDB::register_class<PhysXDirectBodyState3D>();

		// "PhysX" registration for appearance in the Project Settings inside physics engine dropdown.
		PhysicsServer3DManager::get_singleton()->register_server("PhysX", callable_mp_static(&create_physx_server));
	}

	if (p_level == MODULE_INITIALIZATION_LEVEL_SCENE) {
		// Scene-level node classes provided by the module.
		ClassDB::register_class<PhysXChunkEmitter3D>();
		ClassDB::register_class<PhysXCloth3D>();
		ClassDB::register_class<PhysXParticleFluid3D>();
	}

#ifdef TOOLS_ENABLED
	if (p_level == MODULE_INITIALIZATION_LEVEL_EDITOR) {
		// Viewport gizmos for the fluid/cloth nodes.
		EditorPlugins::add_by_type<PhysXEditorPlugin>();
	}
#endif
}

void uninitialize_physx_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SERVERS) {
		return;
	}
	// Inner PhysXServer3D lifetime is managed by PhysicsServer3DWrapMT,
	// which the PhysicsServer3DManager owns and tears down at shutdown.
}
