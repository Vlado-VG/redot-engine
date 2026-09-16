//////////////////////////////////////////////////////////////////////////
/*  physx_blast_preview.h                                               */
//////////////////////////////////////////////////////////////////////////
/*                         This file is part of:                          */
/*                             REDOT ENGINE                               */
/*                        https://redotengine.org                         */
/* Copyright (c) 2024-present Redot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */


#pragma once

#ifdef GODOT_PHYSX_BLAST

#include "core/object/ref_counted.h"

class ArrayMesh;
class PhysXBlastAsset;

// Builds one surface per leaf chunk of p_asset (index 0 is the unfractured
// root chunk -- never rendered on its own, same convention
// PhysXDestructible3D already uses), each a flat,
// distinct color so the fracture cells read clearly regardless of scene
// lighting. Shared by PhysXBlastFractureDialog's live preview and the
// Inspector's PhysXBlastAsset preview (EditorInspectorPluginPhysXBlastAsset)
// -- both want the exact same "here is what actually broke" picture.
Ref<ArrayMesh> physx_blast_build_preview_mesh(const Ref<PhysXBlastAsset> &p_asset);

#endif // GODOT_PHYSX_BLAST
