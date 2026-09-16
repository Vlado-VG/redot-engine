//////////////////////////////////////////////////////////////////////////
/*  physx_blast_asset_inspector_plugin.h                                */
//////////////////////////////////////////////////////////////////////////
/*                         This file is part of:                          */
/*                             REDOT ENGINE                               */
/*                        https://redotengine.org                         */
/* Copyright (c) 2024-present Redot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */


#pragma once

#ifdef GODOT_PHYSX_BLAST

#include "editor/inspector/editor_inspector.h"

// Gives a PhysXBlastAsset the same live-rotate mesh preview a plain Mesh
// resource gets (EditorInspectorPluginMesh/MeshEditor) -- otherwise its
// Inspector is just raw asset_bytes/chunk_points arrays, no visual at all,
// unlike double-clicking the source Mesh it was fractured from.
class EditorInspectorPluginPhysXBlastAsset : public EditorInspectorPlugin {
	GDCLASS(EditorInspectorPluginPhysXBlastAsset, EditorInspectorPlugin);

public:
	virtual bool can_handle(Object *p_object) override;
	virtual void parse_begin(Object *p_object) override;
};

#endif // GODOT_PHYSX_BLAST
