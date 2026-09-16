//////////////////////////////////////////////////////////////////////////
/*  physx_blast_context_menu_plugin.h                                   */
//////////////////////////////////////////////////////////////////////////
/*                         This file is part of:                          */
/*                             REDOT ENGINE                               */
/*                        https://redotengine.org                         */
/* Copyright (c) 2024-present Redot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */


#pragma once

#ifdef GODOT_PHYSX_BLAST

#include "editor/inspector/editor_context_menu_plugin.h"

class PhysXBlastFractureDialog;

// Adds a "Fracture with Blast..." item to two right-click menus -- a
// MeshInstance3D node in the Scene tree dock (CONTEXT_SLOT_SCENE_TREE) and a
// Mesh resource file in the FileSystem dock (CONTEXT_SLOT_FILESYSTEM) -- both
// opening the same PhysXBlastFractureDialog instance. One plugin instance
// handles exactly one slot (EditorContextMenuPluginManager::add_plugin sets
// the slot per-Ref), so PhysXEditorPlugin constructs two of these, one per
// slot, both pointed at the same dialog.
class PhysXBlastFractureMenuPlugin : public EditorContextMenuPlugin {
	GDCLASS(PhysXBlastFractureMenuPlugin, EditorContextMenuPlugin);

	ContextMenuSlot target_slot;
	PhysXBlastFractureDialog *dialog = nullptr; // not owned, outlives this plugin

	void _on_scene_tree_option(Array p_nodes);
	void _on_filesystem_option(PackedStringArray p_paths);

public:
	void get_options(const Vector<String> &p_paths) override;

	PhysXBlastFractureMenuPlugin(ContextMenuSlot p_target_slot, PhysXBlastFractureDialog *p_dialog);
};

#endif // GODOT_PHYSX_BLAST
