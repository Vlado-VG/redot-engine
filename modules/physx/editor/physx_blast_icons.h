//////////////////////////////////////////////////////////////////////////
/*  physx_blast_icons.h                                                 */
//////////////////////////////////////////////////////////////////////////
/*                         This file is part of:                          */
/*                             REDOT ENGINE                               */
/*                        https://redotengine.org                         */
/* Copyright (c) 2024-present Redot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */


#pragma once

#ifdef GODOT_PHYSX_BLAST

#include "core/object/ref_counted.h"

class Texture2D;

// A small flat-color "cracked square" icon for PhysXBlastAsset -- same
// square-resource silhouette and accent color (#ffca5f) core's own
// ArrayMesh/Mesh icons use, split by a jagged gap so it reads as "fractured"
// rather than a plain mesh at the FileSystem dock's icon size. Built from an
// inline SVG string (Image::load_svg_from_string) rather than a loose file:
// this module ships as part of the engine binary, not a project/addon, so
// there's no res:// path a loose .svg under modules/ could be loaded from.
// Null on failure (e.g. an editor build without the svg module) -- callers
// should treat that as "no custom icon, keep the engine's generic fallback".
Ref<Texture2D> physx_blast_asset_make_icon();

#endif // GODOT_PHYSX_BLAST
