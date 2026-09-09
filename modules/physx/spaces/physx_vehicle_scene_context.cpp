/**
 * @file physx_vehicle_scene_context.cpp
 * @brief Implementation of PhysXVehicleSceneContext — per-scene vehicle2 context.
 *
 * P3: Skeleton — no runtime change. The context owns a batched road-geometry
 * query buffer and a tire-friction table, both populated in later phases
 * (P7 batched road queries, P12 friction mapping).
 */

#include "physx_vehicle_scene_context.h"

PhysXVehicleSceneContext::PhysXVehicleSceneContext() = default;

PhysXVehicleSceneContext::~PhysXVehicleSceneContext() = default;
