//////////////////////////////////////////////////////////////////////////
/*  physx_blast_asset.h                                                 */
//////////////////////////////////////////////////////////////////////////
/*                         This file is part of:                          */
/*                             REDOT ENGINE                               */
/*                        https://redotengine.org                         */
/* Copyright (c) 2024-present Redot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */


#pragma once

#include "core/io/resource.h"
#include "core/variant/array.h"
#include "core/variant/variant.h"

// A fractured Blast asset, saveable/loadable as a normal Godot Resource
// (.tres/.res). Produced by PhysXBlastAuthoring::fracture_mesh(); consumed
// by PhysXDestructible3D, which prefers it over the legacy two-file
// asset_path/chunks_path convention (still supported as a fallback).
class PhysXBlastAsset : public Resource {
	GDCLASS(PhysXBlastAsset, Resource);

protected:
	static void _bind_methods();

public:
	// The raw NvBlastAsset bytes -- a single relocatable memory block per
	// NvBlast's own design, so this is genuinely just "the asset", no
	// framing of our own on top.
	void set_asset_bytes(const PackedByteArray &p_bytes) { asset_bytes = p_bytes; }
	PackedByteArray get_asset_bytes() const { return asset_bytes; }

	// chunk_points[i] is a PackedVector3Array: chunk i's render-mesh
	// triangle-soup positions (object-local space), the same triangle-soup
	// format PhysXDestructible3D's legacy .chunks text fallback parses --
	// here it's real Resource-serialized data, no text parsing needed.
	void set_chunk_points(const Array &p_points) { chunk_points = p_points; }
	Array get_chunk_points() const { return chunk_points; }

	int get_chunk_count() const { return chunk_points.size(); }

private:
	PackedByteArray asset_bytes;
	Array chunk_points;
};
