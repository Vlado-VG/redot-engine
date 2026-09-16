//////////////////////////////////////////////////////////////////////////
/*  physx_blast_asset.cpp                                               */
//////////////////////////////////////////////////////////////////////////
/*                         This file is part of:                          */
/*                             REDOT ENGINE                               */
/*                        https://redotengine.org                         */
/* Copyright (c) 2024-present Redot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */


#include "physx_blast_asset.h"

#include "core/object/class_db.h"

void PhysXBlastAsset::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_asset_bytes", "bytes"), &PhysXBlastAsset::set_asset_bytes);
	ClassDB::bind_method(D_METHOD("get_asset_bytes"), &PhysXBlastAsset::get_asset_bytes);
	ClassDB::bind_method(D_METHOD("set_chunk_points", "points"), &PhysXBlastAsset::set_chunk_points);
	ClassDB::bind_method(D_METHOD("get_chunk_points"), &PhysXBlastAsset::get_chunk_points);
	ClassDB::bind_method(D_METHOD("get_chunk_count"), &PhysXBlastAsset::get_chunk_count);

	ADD_PROPERTY(PropertyInfo(Variant::PACKED_BYTE_ARRAY, "asset_bytes"), "set_asset_bytes", "get_asset_bytes");
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "chunk_points"), "set_chunk_points", "get_chunk_points");
}
