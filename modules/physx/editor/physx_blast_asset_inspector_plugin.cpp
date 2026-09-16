//////////////////////////////////////////////////////////////////////////
/*  physx_blast_asset_inspector_plugin.cpp                              */
//////////////////////////////////////////////////////////////////////////
/*                         This file is part of:                          */
/*                             REDOT ENGINE                               */
/*                        https://redotengine.org                         */
/* Copyright (c) 2024-present Redot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */


#include "physx_blast_asset_inspector_plugin.h"

#ifdef GODOT_PHYSX_BLAST

#include "../blast/physx_blast_asset.h"
#include "physx_blast_preview.h"

#include "editor/scene/3d/mesh_editor_plugin.h"

bool EditorInspectorPluginPhysXBlastAsset::can_handle(Object *p_object) {
	return Object::cast_to<PhysXBlastAsset>(p_object) != nullptr;
}

void EditorInspectorPluginPhysXBlastAsset::parse_begin(Object *p_object) {
	PhysXBlastAsset *asset = Object::cast_to<PhysXBlastAsset>(p_object);
	if (!asset) {
		return;
	}
	Ref<PhysXBlastAsset> a(asset);

	MeshEditor *editor = memnew(MeshEditor);
	editor->edit(physx_blast_build_preview_mesh(a));
	add_custom_control(editor);
}

#endif // GODOT_PHYSX_BLAST
