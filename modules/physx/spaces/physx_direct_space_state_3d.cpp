/**
 * @file physx_direct_space_state_3d.cpp
 * @brief Scene query implementations (raycast, overlap, sweep, test_motion).
 *
 * Each query builds a PhysXQueryFilterCallback with Godot's mask/exclusion
 * parameters, invokes the corresponding PxScene method, and maps PhysX hit
 * results back to Godot's RayResult / ShapeResult / ShapeRestInfo / MotionResult
 * via the PhysXActorUserData attached to each actor (PxShape::userData points at
 * the shared PhysXShape3D* blueprint).
 */

#include "physx_direct_space_state_3d.h"
#include "../physx_server.h"
#include "physx_space_3d.h"
#include "../objects/physx_body_3d.h"
#include "../shapes/physx_shape_3d.h"
#include "../shapes/physx_separation_ray_shape_3d.h"
#include "physx_query_filter_callback.h"
#include "../shapes/physx_user_data.h"

#include "geometry/PxGeometryQuery.h"

// Upper bound on the number of results a single scene query can return.
// Caller-provided p_result_max is clamped to this so a hostile or buggy
// caller cannot exhaust the stack/heap. Godot caps its own result arrays
// at 64, so 256 leaves comfortable headroom.
static constexpr int PHYSX_QUERY_MAX_RESULTS = 256;

PhysXDirectSpaceState3D::PhysXDirectSpaceState3D(PhysXSpace3D *p_space) {
    space = p_space;
}

// ---------------------------------------------------------------------------
// _build_query_shape — shared geometry/pose builder for shape-based queries.
//
// Resolves a Godot shape RID to its PhysXShape3D blueprint, generates the
// scaled PxGeometry via the shape's own get_physx_geometry() (so every shape
// type — box/sphere/capsule/convex/concave/heightmap/cylinder/cone — is
// supported, with scale and axis alignment baked in), and composes the query
// pose as world_transform * shape_local_pose. This mirrors how shapes are
// placed on a body in PhysXShapedObject3D::add_shape().
//
// Returns false (with an error) if the RID is invalid or geometry generation
// failed; callers return "no hit" in that case.
// ---------------------------------------------------------------------------
static bool _build_query_shape(const RID &p_shape_rid,
                               const Transform3D &p_transform,
                               physx::PxGeometryHolder &r_geometry,
                               physx::PxTransform &r_pose) {
    PhysXServer3D *server = PhysXServer3D::get_singleton();
    ERR_FAIL_NULL_V(server, false);
    const PhysXShape3D *shape = server->get_shape(p_shape_rid);
    if (!shape) {
        ERR_PRINT_ONCE("PhysX: query referenced an invalid shape RID.");
        return false;
    }

    // Bake the query transform's scale into the geometry (PhysX shapes carry
    // no scale of their own). The pose uses the unscaled rotation/translation.
    const Vector3 scale = p_transform.basis.get_scale_abs();
    const physx::PxVec3 px_scale(scale.x, scale.y, scale.z);
    if (!shape->get_physx_geometry(r_geometry, px_scale)) {
        ERR_PRINT_ONCE("PhysX: query shape failed to generate geometry.");
        return false;
    }

    // Compose world pose * shape-local alignment pose (e.g. capsule Y->X).
    // Orthonormalize the basis so the PxTransform quaternion is valid even if
    // the input basis carried shear from the scale we just stripped out.
    Transform3D unscaled = p_transform;
    unscaled.basis.orthonormalize();
    r_pose = PhysXShapedObject3D::to_physx_transform(unscaled) * shape->get_local_pose();
    return true;
}

// -----------------------------------------------------------------------
// INTERSECT RAY (Raycast)
// -----------------------------------------------------------------------
bool PhysXDirectSpaceState3D::intersect_ray(const RayParameters &p_parameters, RayResult &r_result) {
    if (!space || !space->get_px_scene()) {
        return false;
    }

    physx::PxScene *scene = space->get_px_scene();

    // Calculate origin, direction, and distance
    Vector3 godot_dir = p_parameters.to - p_parameters.from;
    real_t length = godot_dir.length();
    if (length == 0.0) {
        return false;
    }
    godot_dir.normalize();

    physx::PxVec3 origin(p_parameters.from.x, p_parameters.from.y, p_parameters.from.z);
    physx::PxVec3 dir(godot_dir.x, godot_dir.y, godot_dir.z);

    // Setup custom filter callback
    PhysXQueryFilterCallback filter_cb;
    filter_cb.collision_mask = p_parameters.collision_mask;
    filter_cb.collide_with_bodies = p_parameters.collide_with_bodies;
    filter_cb.collide_with_areas = p_parameters.collide_with_areas;
    filter_cb.exclude_rids = &p_parameters.exclude;
    // intersect_ray is single-hit — keep eBLOCK for the closest hit.

    // 2. Configure PhysX to USE the preFilter callback
    physx::PxQueryFilterData filter_data;
    filter_data.flags = physx::PxQueryFlag::eDYNAMIC | physx::PxQueryFlag::eSTATIC | physx::PxQueryFlag::ePREFILTER;

    // 3. Map Godot's backface/inside flags to PhysX Hit Flags.
    //   - hit_back_faces: let raycasts hit the back side of triangles
    //     (PhysX: eMESH_BOTH_SIDES).
    //   - hit_from_inside: Godot's contract is that when the ray origin is
    //     inside a shape, a hit at distance 0 (position = origin, zero normal)
    //     is returned. PhysX already returns a distance-0 hit when starting
    //     inside a convex shape, so hit_from_inside=true needs no extra flag.
    //     When hit_from_inside=false, Godot ignores shapes the origin is inside;
    //     we emulate that by dropping zero-distance hits below.
    //   (eMTD is for overlap/sweep penetration queries, NOT raycasts — using it
    //   here was a no-op.)
    physx::PxHitFlags hit_flags = physx::PxHitFlag::ePOSITION | physx::PxHitFlag::eNORMAL | physx::PxHitFlag::eFACE_INDEX;

    if (p_parameters.hit_back_faces) {
        hit_flags |= physx::PxHitFlag::eMESH_BOTH_SIDES;
    }

    // When hit_from_inside is true, the origin may be inside a shape. PhysX
    // raycasts don't reliably report zero-distance inside hits (especially for
    // mesh shapes), so we do a tiny-sphere overlap to detect if the origin
    // is inside any shape and synthesize the Godot origin-hit if so.
    physx::PxTransform pose(origin);
    if (p_parameters.hit_from_inside) {
        constexpr float INSIDE_RADIUS = 1e-4f;
        physx::PxSphereGeometry inside_geo(INSIDE_RADIUS);
        // multi_hit returns eTOUCH from the pre-filter: overlap queries warn
        // when the filter returns eBLOCK.
        filter_cb.multi_hit = true;
        LocalVector<physx::PxOverlapHit> overlap_storage;
        overlap_storage.resize(1);
        physx::PxOverlapBuffer overlap_buf(overlap_storage.ptr(), 1);
        if (scene->overlap(inside_geo, pose, overlap_buf, filter_data, &filter_cb)) {
            const physx::PxOverlapHit &overlap = overlap_buf.getAnyHit(0);

            // Synthesize the origin hit (distance 0, zero normal) for the
            // first shape that contains the origin.
            r_result.position = p_parameters.from;
            r_result.normal = Vector3();

            if (overlap.actor && overlap.actor->userData) {
                auto *actor_data = static_cast<PhysXActorUserData*>(overlap.actor->userData);
                r_result.rid = actor_data->rid;
                r_result.collider_id = actor_data->object_id;
                r_result.collider = ObjectDB::get_instance(actor_data->object_id);
            } else {
                r_result.rid = RID();
                r_result.collider_id = ObjectID();
                r_result.collider = nullptr;
            }
            if (overlap.shape && overlap.shape->userData) {
                r_result.shape = physx_resolve_shape_index(overlap.actor, overlap.shape);
            } else {
                r_result.shape = 0;
            }
            // face_index is undefined for origin-inside hits; leave as default.
            return true;
        }
    }

    physx::PxRaycastBuffer hit;
    bool has_hit = scene->raycast(origin, dir, length, hit, hit_flags, filter_data, &filter_cb);

    if (has_hit && hit.hasBlock) {
        const physx::PxRaycastHit &block = hit.block;

               const bool is_backface = block.normal.dot(dir) > 0.0f;

        if (!p_parameters.hit_from_inside) {
            // Drop hits where the origin is inside the shape (convex: distance <= 0)
            // or where the ray hit a backface (mesh: origin is inside the mesh).
            if (block.distance <= 0.0f || is_backface) {
                return false;
            }
        } else {
            // hit_from_inside == true: report a synthetic from-inside hit.
            if (block.distance <= 0.0f || is_backface) {
                r_result.position = p_parameters.from;
                r_result.normal = Vector3();
                r_result.face_index = block.faceIndex;

                // Map the PhysX hit back to Godot via the actor/shape userData.
                if (block.actor && block.actor->userData) {
                    auto *actor_data = static_cast<PhysXActorUserData*>(block.actor->userData);
                    r_result.rid = actor_data->rid;
                    r_result.collider_id = actor_data->object_id;
                    r_result.collider = ObjectDB::get_instance(actor_data->object_id);
                } else {
                    r_result.rid = RID();
                    r_result.collider_id = ObjectID();
                    r_result.collider = nullptr;
                }
                if (block.shape && block.shape->userData) {
                    r_result.shape = physx_resolve_shape_index(block.actor, block.shape);
                } else {
                    r_result.shape = 0;
                }
                return true;
            }
        }

        r_result.position = Vector3(block.position.x, block.position.y, block.position.z);
        r_result.normal = Vector3(block.normal.x, block.normal.y, block.normal.z);
        r_result.face_index = block.faceIndex;

        // Map the PhysX hit back to Godot via the actor/shape userData.
        if (block.actor && block.actor->userData) {
            auto *actor_data = static_cast<PhysXActorUserData*>(block.actor->userData);
            r_result.rid = actor_data->rid;
            r_result.collider_id = actor_data->object_id;
            r_result.collider = ObjectDB::get_instance(actor_data->object_id);
        } else {
            r_result.rid = RID();
            r_result.collider_id = ObjectID();
            r_result.collider = nullptr;
        }
        if (block.shape && block.shape->userData) {
            r_result.shape = physx_resolve_shape_index(block.actor, block.shape);
        } else {
            r_result.shape = 0;
        }
        return true;
    }
return false;
}
// -----------------------------------------------------------------------
// INTERSECT POINT (Overlap)
// -----------------------------------------------------------------------
int PhysXDirectSpaceState3D::intersect_point(const PointParameters &p_parameters, ShapeResult *r_results, int p_result_max) {
    if (!space || !space->get_px_scene() || p_result_max <= 0) {
        return 0;
    }
    physx::PxScene *scene = space->get_px_scene();
    constexpr float POINT_QUERY_RADIUS = 1e-4f;
    // Setup a tiny sphere geometry to simulate a "point"
    physx::PxSphereGeometry point_geo(POINT_QUERY_RADIUS);
    physx::PxTransform pose(physx::PxVec3(p_parameters.position.x, p_parameters.position.y, p_parameters.position.z));

    // Heap-allocate the hit buffer sized by the caller's request. alloca with
    // a caller-controlled count is a stack-exhaustion vector; the result_max
    // caps below also bound the engine-facing result arrays.
    const int result_max = MIN(p_result_max, PHYSX_QUERY_MAX_RESULTS);
    LocalVector<physx::PxOverlapHit> hit_storage;
    hit_storage.resize(result_max);
    physx::PxOverlapBuffer hit(hit_storage.ptr(), result_max);
    PhysXQueryFilterCallback filter_cb;
    filter_cb.collision_mask = p_parameters.collision_mask;
    filter_cb.collide_with_bodies = p_parameters.collide_with_bodies;
    filter_cb.collide_with_areas = p_parameters.collide_with_areas;
    filter_cb.exclude_rids = &p_parameters.exclude;
    filter_cb.multi_hit = true; ///< intersect_point: collect all overlaps

    physx::PxQueryFilterData filter_data;
    filter_data.flags = physx::PxQueryFlag::eDYNAMIC | physx::PxQueryFlag::eSTATIC | physx::PxQueryFlag::ePREFILTER;

    // Execute Overlap
    bool has_hit = scene->overlap(point_geo, pose, hit, filter_data, &filter_cb);
    
    if (!has_hit) {
        return 0;
    }

    // Map results back to Godot via userData.
    const int count = MIN((int)hit.getNbAnyHits(), result_max);
    for (int i = 0; i < count; ++i) {
        const physx::PxOverlapHit &overlap = hit.getAnyHit(i);

        if (overlap.actor && overlap.actor->userData) {
            auto *actor_data = static_cast<PhysXActorUserData *>(overlap.actor->userData);
            r_results[i].rid = actor_data->rid;
            r_results[i].collider_id = actor_data->object_id;
            r_results[i].collider = ObjectDB::get_instance(actor_data->object_id);
        } else {
            r_results[i].rid = RID();
            r_results[i].collider_id = ObjectID();
            r_results[i].collider = nullptr;
        }

        if (overlap.shape && overlap.shape->userData) {
            r_results[i].shape = physx_resolve_shape_index(overlap.actor, overlap.shape);
        } else {
            r_results[i].shape = 0;
        }
    }

    return count;
}

int PhysXDirectSpaceState3D::intersect_shape(const ShapeParameters &p_parameters, ShapeResult *r_results, int p_result_max) {
    if (!space || !space->get_px_scene() || p_result_max <= 0) {
        return 0;
    }

    // Build the query geometry + pose from the shape resource (supports every
    // shape type via the shape's own get_physx_geometry / get_local_pose).
    physx::PxGeometryHolder geometry;
    physx::PxTransform pose(physx::PxIdentity);
    if (!_build_query_shape(p_parameters.shape_rid, p_parameters.transform, geometry, pose)) {
        return 0;
    }

    // Setup our custom filter callback
    PhysXQueryFilterCallback filter_cb;
    filter_cb.collision_mask = p_parameters.collision_mask;
    filter_cb.collide_with_bodies = p_parameters.collide_with_bodies;
    filter_cb.collide_with_areas = p_parameters.collide_with_areas;
    filter_cb.exclude_rids = &p_parameters.exclude;
    filter_cb.multi_hit = true; ///< intersect_shape: collect all overlaps

    physx::PxQueryFilterData filter_data;
    filter_data.flags = physx::PxQueryFlag::eDYNAMIC | physx::PxQueryFlag::eSTATIC | physx::PxQueryFlag::ePREFILTER;

    // Heap-allocate the hit buffer; clamp to PHYSX_QUERY_MAX_RESULTS so a
    // caller-controlled count can't exhaust the stack.
    const int result_max = MIN(p_result_max, PHYSX_QUERY_MAX_RESULTS);
    LocalVector<physx::PxOverlapHit> hit_storage;
    hit_storage.resize(result_max);
    physx::PxOverlapBuffer hit(hit_storage.ptr(), result_max);

    // Fire the overlap query
    bool has_hit = space->get_px_scene()->overlap(geometry.any(), pose, hit, filter_data, &filter_cb);

    if (!has_hit) {
        return 0;
    }

    // Map the results using our flattened UserData
    const int count = MIN((int)hit.getNbAnyHits(), result_max);
    for (int i = 0; i < count; ++i) {
        const physx::PxOverlapHit &overlap = hit.getAnyHit(i);

        if (overlap.actor && overlap.actor->userData) {
            auto *actor_data = static_cast<PhysXActorUserData*>(overlap.actor->userData);
            r_results[i].rid = actor_data->rid;
            r_results[i].collider_id = actor_data->object_id;
            r_results[i].collider = ObjectDB::get_instance(actor_data->object_id);
        } else {
            r_results[i].rid = RID();
            r_results[i].collider_id = ObjectID();
            r_results[i].collider = nullptr;
        }

        if (overlap.shape && overlap.shape->userData) {
            r_results[i].shape = physx_resolve_shape_index(overlap.actor, overlap.shape);
        } else {
            r_results[i].shape = 0;
        }
    }

    return count;
}

bool PhysXDirectSpaceState3D::cast_motion(const ShapeParameters &p_parameters, real_t &r_closest_safe, real_t &r_closest_unsafe, ShapeRestInfo *r_info) {
    if (!space || !space->get_px_scene()) return false;

    // Build the query geometry + pose from the shape resource.
    physx::PxGeometryHolder geometry;
    physx::PxTransform pose(physx::PxIdentity);
    if (!_build_query_shape(p_parameters.shape_rid, p_parameters.transform, geometry, pose)) {
        r_closest_safe = 1.0;
        r_closest_unsafe = 1.0;
        return false;
    }

    Vector3 motion = p_parameters.motion;
    real_t length = motion.length();
    if (length == 0.0) {
        r_closest_safe = 1.0;
        r_closest_unsafe = 1.0;
        return false;
    }

    Vector3 dir = motion / length;
    physx::PxVec3 px_dir(dir.x, dir.y, dir.z);

    PhysXQueryFilterCallback filter_cb;
    filter_cb.collision_mask = p_parameters.collision_mask;
    filter_cb.collide_with_bodies = p_parameters.collide_with_bodies;
    filter_cb.collide_with_areas = p_parameters.collide_with_areas;
    filter_cb.exclude_rids = &p_parameters.exclude;
    // cast_motion is single-hit — keep eBLOCK for the closest hit.

    physx::PxQueryFilterData filter_data;
    filter_data.flags = physx::PxQueryFlag::eDYNAMIC | physx::PxQueryFlag::eSTATIC | physx::PxQueryFlag::ePREFILTER;

    physx::PxSweepBuffer hit;
    // eMTD makes initial-overlap sweeps report a well-defined (negative)
    // distance/normal/position instead of distance==0 with garbage fields.
    physx::PxHitFlags hit_flags = physx::PxHitFlag::ePOSITION | physx::PxHitFlag::eNORMAL |
                                  physx::PxHitFlag::eFACE_INDEX | physx::PxHitFlag::eMTD;

    bool has_hit = space->get_px_scene()->sweep(geometry.any(), pose, px_dir, length, hit, hit_flags, filter_data, &filter_cb);

    if (has_hit && hit.hasBlock) {
        // Godot's cast_motion contract: shapes that the query is ALREADY
        // overlapping are completely disregarded. An initial overlap shows up
        // as distance <= 0 (hadInitialOverlap), so skip it and report an
        // unobstructed full motion. A real forward hit has distance > 0.
        if (!hit.block.hadInitialOverlap()) {
            real_t hit_fraction = hit.block.distance / length;
            real_t margin_fraction = p_parameters.margin / length;

            r_closest_safe = MAX(0.0, hit_fraction - margin_fraction);
            r_closest_unsafe = hit_fraction;
            return true;
        }
    }

    r_closest_safe = 1.0;
    r_closest_unsafe = 1.0;
    return false;
}

bool PhysXDirectSpaceState3D::collide_shape(const ShapeParameters &p_parameters, Vector3 *r_results, int p_result_max, int &r_result_count) {
    r_result_count = 0;
    if (!space || !space->get_px_scene() || p_result_max <= 0) return false;

    // Build the query geometry + pose from the shape resource.
    physx::PxGeometryHolder query_geom;
    physx::PxTransform query_pose(physx::PxIdentity);
    if (!_build_query_shape(p_parameters.shape_rid, p_parameters.transform, query_geom, query_pose)) {
        return false;
    }

    // Filter fields
    PhysXQueryFilterCallback filter_cb;
    filter_cb.collision_mask = p_parameters.collision_mask;
    filter_cb.collide_with_bodies = p_parameters.collide_with_bodies;
    filter_cb.collide_with_areas = p_parameters.collide_with_areas;
    filter_cb.exclude_rids = &p_parameters.exclude;
    filter_cb.multi_hit = true; ///< collide_shape: collect all overlaps

    // Perform Overlap
    physx::PxQueryFilterData filter_data;
    filter_data.flags = physx::PxQueryFlag::eDYNAMIC | physx::PxQueryFlag::eSTATIC | physx::PxQueryFlag::ePREFILTER;

    const int result_max = MIN(p_result_max, PHYSX_QUERY_MAX_RESULTS);
    LocalVector<physx::PxOverlapHit> hit_storage;
    hit_storage.resize(result_max);
    physx::PxOverlapBuffer hit(hit_storage.ptr(), result_max);

    if (!space->get_px_scene()->overlap(query_geom.any(), query_pose, hit, filter_data, &filter_cb)) return false;

    // Process contacts using computePenetration. Godot's collide_shape contract
    // is that r_results holds PAIRS of contact points per collision: index
    // (i*2+0) is the contact point on the QUERY shape's surface, (i*2+1) is the
    // contact point on the COLLIDER's surface. r_result_count counts contacts
    // (pairs), not individual points. (Buffer is pre-sized p_result_max*2 by the
    // caller.) Collisions are processed nearest-first by depth.
    int written = 0;
    for (physx::PxU32 i = 0; i < hit.getNbAnyHits() && written < result_max; ++i) {
        const physx::PxOverlapHit &overlap = hit.getAnyHit(i);

        physx::PxGeometryHolder hit_geom = overlap.shape->getGeometry();
        physx::PxTransform hit_pose = physx::PxShapeExt::getGlobalPose(*overlap.shape, *overlap.actor);

        physx::PxVec3 mtd_dir;
        physx::PxReal penetration_depth = 0.0f;
        const bool is_pen = physx::PxGeometryQuery::computePenetration(
            mtd_dir, penetration_depth,
            query_geom.any(), query_pose,
            hit_geom.any(), hit_pose);

        if (!is_pen) {
            continue;
        }

        // Contact points must lie on the shapes' surfaces, not near the query
        // shape's centroid. The collider-side point is the closest point on the
        // collider's surface to the query shape's center; the query-side point
        // sits one penetration depth further along the MTD (push-out vector).
        physx::PxVec3 closest_on_collider;
        physx::PxGeometryQuery::pointDistance(query_pose.p, hit_geom.any(), hit_pose, &closest_on_collider);
        const physx::PxVec3 point_a = closest_on_collider + mtd_dir * penetration_depth; // query side
        const physx::PxVec3 point_b = closest_on_collider; // collider side
        r_results[written * 2 + 0] = Vector3(point_a.x, point_a.y, point_a.z);
        r_results[written * 2 + 1] = Vector3(point_b.x, point_b.y, point_b.z);
        written++;
    }

    r_result_count = written;
    return r_result_count > 0;
}

bool PhysXDirectSpaceState3D::rest_info(const ShapeParameters &p_parameters, ShapeRestInfo *r_info) {
    // Returns the nearest-surface contact info for the query shape: overlaps
    // the scene, computes penetration against every hit, and reports the one
    // with the SMALLEST penetration depth (i.e. the collider the query shape
    // is just barely touching/penetrating). Used by cast_motion when the sweep
    // starts already overlapping, and by direct rest_info queries.
    if (!r_info || !space || !space->get_px_scene()) {
        return false;
    }

    physx::PxGeometryHolder query_geom;
    physx::PxTransform query_pose(physx::PxIdentity);
    if (!_build_query_shape(p_parameters.shape_rid, p_parameters.transform, query_geom, query_pose)) {
        return false;
    }

    PhysXQueryFilterCallback filter_cb;
    filter_cb.collision_mask = p_parameters.collision_mask;
    filter_cb.collide_with_bodies = p_parameters.collide_with_bodies;
    filter_cb.collide_with_areas = p_parameters.collide_with_areas;
    filter_cb.exclude_rids = &p_parameters.exclude;
    filter_cb.multi_hit = true; ///< rest_info: collect all overlaps

    physx::PxQueryFilterData filter_data;
    filter_data.flags = physx::PxQueryFlag::eDYNAMIC | physx::PxQueryFlag::eSTATIC | physx::PxQueryFlag::ePREFILTER;

    constexpr int MAX_REST_HITS = 32;
    physx::PxOverlapHit hit_buffer[MAX_REST_HITS];
    physx::PxOverlapBuffer hit(hit_buffer, MAX_REST_HITS);

    if (!space->get_px_scene()->overlap(query_geom.any(), query_pose, hit, filter_data, &filter_cb)) {
        return false;
    }

    // Find the overlapping shape with the minimum penetration depth.
    bool found = false;
    physx::PxReal min_depth = PX_MAX_F32;
    const physx::PxOverlapHit *best = nullptr;
    physx::PxVec3 best_mtd(0, 0, 0);
    physx::PxGeometryHolder best_geom;
    physx::PxTransform best_pose(physx::PxIdentity);

    for (physx::PxU32 i = 0; i < hit.getNbAnyHits(); ++i) {
        const physx::PxOverlapHit &overlap = hit.getAnyHit(i);

        physx::PxGeometryHolder hit_geom = overlap.shape->getGeometry();
        physx::PxTransform hit_pose = physx::PxShapeExt::getGlobalPose(*overlap.shape, *overlap.actor);

        physx::PxVec3 mtd_dir;
        physx::PxReal penetration_depth = 0.0f;
        if (!physx::PxGeometryQuery::computePenetration(
                mtd_dir, penetration_depth,
                query_geom.any(), query_pose,
                hit_geom.any(), hit_pose)) {
            continue;
        }

        if (penetration_depth < min_depth) {
            min_depth = penetration_depth;
            best = &overlap;
            best_mtd = mtd_dir;
            best_geom = hit_geom;
            best_pose = hit_pose;
            found = true;
        }
    }

    if (!found || !best) {
        return false;
    }

    // MTD points from the collider toward the query shape; Godot's rest normal
    // points toward the query shape (away from the surface it rests on). The
    // rest point is on the collider's surface (see collide_shape).
    r_info->normal = Vector3(best_mtd.x, best_mtd.y, best_mtd.z);
    physx::PxVec3 rest_pt;
    physx::PxGeometryQuery::pointDistance(query_pose.p, best_geom.any(), best_pose, &rest_pt);
    r_info->point = Vector3(rest_pt.x, rest_pt.y, rest_pt.z);

    if (best->actor && best->actor->userData) {
        const auto *actor_data = static_cast<PhysXActorUserData *>(best->actor->userData);
        r_info->rid = actor_data->rid;
        r_info->collider_id = actor_data->object_id;
    } else {
        r_info->rid = RID();
        r_info->collider_id = ObjectID();
    }
    r_info->shape = physx_resolve_shape_index(best->actor, best->shape);

    // Velocity at the rest point (for dynamic colliders).
    r_info->linear_velocity = Vector3();
    if (best->actor && best->actor->is<physx::PxRigidDynamic>()) {
        const physx::PxRigidDynamic *dyn = best->actor->is<physx::PxRigidDynamic>();
        const physx::PxVec3 v = physx::PxRigidBodyExt::getVelocityAtPos(*const_cast<physx::PxRigidDynamic *>(dyn), rest_pt);
        r_info->linear_velocity = Vector3(v.x, v.y, v.z);
    }

    return true;
}

Vector3 PhysXDirectSpaceState3D::get_closest_point_to_object_volume(RID p_object, Vector3 p_point) const {
    // Resolve the RID to the internal PhysXBody3D wrapper.
    PhysXBody3D *body = PhysXServer3D::get_singleton()->get_body(p_object);

    if (!body) {
        return p_point;
    }

    physx::PxRigidActor *actor = body->get_px_actor();
    if (!actor) {
        return p_point;
    }

    physx::PxVec3 query_point(p_point.x, p_point.y, p_point.z);
    physx::PxVec3 closest_point(p_point.x, p_point.y, p_point.z); // Default to origin
    physx::PxReal min_distance = PX_MAX_F32;

    // Get all shapes from the actor.
    const physx::PxU32 nb_shapes = actor->getNbShapes();
    if (nb_shapes == 0) return p_point;

    LocalVector<physx::PxShape *> shapes;
    shapes.resize(nb_shapes);
    actor->getShapes(shapes.ptr(), nb_shapes);

    for (physx::PxU32 i = 0; i < nb_shapes; ++i) {
        physx::PxShape *shape = shapes[i];
        physx::PxGeometryHolder geom = shape->getGeometry();
        physx::PxTransform pose = physx::PxShapeExt::getGlobalPose(*shape, *actor);

        physx::PxVec3 local_closest;
        // 0.0f means we don't care about inflation
        physx::PxReal dist = physx::PxGeometryQuery::pointDistance(query_point, geom.any(), pose, &local_closest);

        if (dist < min_distance) {
            min_distance = dist;
            closest_point = local_closest;
        }
    }

    return Vector3(closest_point.x, closest_point.y, closest_point.z);
}

// Fills collisions[0] from a sweep blocking hit when the overlap-based collide
// phase came back empty (at exact touch there is no penetration to report).
static void _fill_collision_from_sweep(PhysicsServer3D::MotionResult *r_result, const PhysXBody3D &p_body,
        const Vector3 &p_position, const Vector3 &p_normal, const physx::PxRigidActor *p_actor, const physx::PxShape *p_shape) {
    if (!r_result || r_result->collision_count > 0 || !p_actor) {
        return;
    }
    PhysicsServer3D::MotionCollision &col = r_result->collisions[0];
    col.position = p_position;
    col.normal = p_normal;
    col.depth = 0.0;
    if (p_actor->userData) {
        const auto *actor_data = static_cast<const PhysXActorUserData *>(p_actor->userData);
        col.collider = actor_data->rid;
        col.collider_id = actor_data->object_id;
    } else {
        col.collider = RID();
        col.collider_id = ObjectID();
    }
    col.collider_shape = physx_resolve_shape_index(p_actor, p_shape);
    const int local_idx = p_body.find_shape_index(p_shape);
    col.local_shape = local_idx >= 0 ? local_idx : 0;
    if (const physx::PxRigidDynamic *dyn = p_actor->is<physx::PxRigidDynamic>()) {
        const physx::PxVec3 lv = dyn->getLinearVelocity();
        const physx::PxVec3 av = dyn->getAngularVelocity();
        col.collider_velocity = Vector3(lv.x, lv.y, lv.z);
        col.collider_angular_velocity = Vector3(av.x, av.y, av.z);
    } else {
        col.collider_velocity = Vector3();
        col.collider_angular_velocity = Vector3();
    }
    r_result->collision_count = 1;
}

bool PhysXDirectSpaceState3D::body_test_motion(const PhysXBody3D &p_body, const PhysicsServer3D::MotionParameters &p_parameters, PhysicsServer3D::MotionResult *r_result) const {
    if (!space) return false;

    Transform3D transform = p_parameters.from;
    Vector3 motion = p_parameters.motion;
    float margin = p_parameters.margin;

    // Build exclusion set with the body's own RID (prevents self-collision at PhysX level)
    HashSet<RID> self_and_excluded;
    if (!p_parameters.exclude_bodies.is_empty()) {
        self_and_excluded = p_parameters.exclude_bodies;
    }
    self_and_excluded.insert(p_body.get_rid());

    // RECOVER (Depenetrate initial overlaps)
    Vector3 recovery;
    bool recovered = _body_motion_recover(p_body, transform, margin, self_and_excluded, p_parameters.exclude_objects, recovery);
    if (!p_parameters.recovery_as_collision) {
        // The MTD is folded into travel below; lift the pose used for the
        // subsequent cast so the sweep starts from the depenetrated position.
        transform.origin += recovery;
    }

    // CAST (Sweep the motion)
    real_t safe_fraction = 1.0;
    real_t unsafe_fraction = 1.0;
    Vector3 hit_position;
    Vector3 hit_normal;
    const physx::PxRigidActor *hit_actor = nullptr;
    const physx::PxShape *hit_shape = nullptr;

    bool hit = _body_motion_cast(p_body, transform, motion, p_parameters.collide_separation_ray, self_and_excluded, p_parameters.exclude_objects, safe_fraction, unsafe_fraction, hit_position, hit_normal, hit_actor, hit_shape);

    if (r_result) {
        // When recovery_as_collision is true and recovery occurred, we still run the cast.
        // The collision result depends on whether the cast hit AND whether a rest contact
        // exists at the recovered position (mirrors godot_space_3d.cpp:929).
        if (p_parameters.recovery_as_collision && recovered) {
            if (hit) {
                // Cast hit: collide at the unsafe position and combine with recovery.
                Transform3D hit_transform = transform;
                hit_transform.origin += motion * unsafe_fraction;
                _body_motion_collide(p_body, hit_transform, motion, p_parameters.max_collisions, self_and_excluded, p_parameters.exclude_objects, r_result);
                _fill_collision_from_sweep(r_result, p_body, hit_position, hit_normal, hit_actor, hit_shape);

                r_result->travel = motion * safe_fraction + recovery;
                r_result->remainder = motion - motion * safe_fraction;
                r_result->collision_unsafe_fraction = unsafe_fraction;
                r_result->collision_safe_fraction = safe_fraction;
            } else {
                // No cast hit: recovery alone can be the collision, but only if
                // the contact pass at the recovered pose actually found
                // touching/overlapping shapes — Godot never reports a hit with
                // an empty collision list (CharacterBody3D would hand out
                // slide collisions with no contacts).
                _body_motion_collide(p_body, transform, Vector3(), p_parameters.max_collisions, self_and_excluded, p_parameters.exclude_objects, r_result);
                r_result->travel = motion + recovery;
                r_result->remainder = Vector3();
                r_result->collision_safe_fraction = 1.0;
                r_result->collision_unsafe_fraction = 1.0;
            }
        } else {
            if (!hit) {
                // travel = full motion + recovery offset (from + travel == final pos).
                r_result->travel = motion + recovery;
                r_result->remainder = Vector3();
                r_result->collision_depth = 0;
                r_result->collision_count = 0;
                r_result->collision_safe_fraction = 1.0;
                r_result->collision_unsafe_fraction = 1.0;
            } else {
                // COLLIDE (Generate detailed manifold at the hit location)
                Transform3D hit_transform = transform;
                hit_transform.origin += motion * unsafe_fraction;
                _body_motion_collide(p_body, hit_transform, motion, p_parameters.max_collisions, self_and_excluded, p_parameters.exclude_objects, r_result);

                // The collide phase overlaps the shape at the contact pose; at
                // exact touch there is no penetration, so it can come back
                // empty. Godot still reports the blocking collision —
                // synthesize it from the sweep hit.
                _fill_collision_from_sweep(r_result, p_body, hit_position, hit_normal, hit_actor, hit_shape);

                // travel = swept motion up to the safe fraction, plus the recovery offset
                // applied to the transform above (matches Godot: from + travel == final pos).
                r_result->travel = motion * safe_fraction + recovery;
                r_result->remainder = motion - motion * safe_fraction;
                r_result->collision_unsafe_fraction = unsafe_fraction;
                r_result->collision_safe_fraction = safe_fraction;
            }
        }
    }

    // An initial penetration counts as a collision only when the caller asked
    // for it (recovery_as_collision) AND the contact pass actually recorded
    // contacts. Returning true with an empty collision list violates Godot's
    // contract — body_test_motion must never report a hit without collisions
    // (CharacterBody3D exposes those as slide collisions, and reading them
    // errors with "index out of bounds"). (REG-0012)
    return hit || (p_parameters.recovery_as_collision && recovered && r_result != nullptr && r_result->collision_count > 0);
}

bool PhysXDirectSpaceState3D::_body_motion_recover(const PhysXBody3D &p_body, const Transform3D &p_transform, float p_margin, 
    const HashSet<RID> &p_self_and_excluded, const HashSet<ObjectID> &p_excluded_objects, Vector3 &r_recovery) const {

    r_recovery = Vector3();

    if (!space || !space->get_px_scene()) {
        return false;
    }

    physx::PxRigidActor *actor = p_body.get_px_actor();
    if (!actor) {
        return false;
    }

    const physx::PxU32 nb_shapes = actor->getNbShapes();
    if (nb_shapes == 0) {
        return false;
    }

    // Heap-allocate the body's shape pointer list.
    LocalVector<physx::PxShape *> shapes;
    shapes.resize(nb_shapes);
    actor->getShapes(shapes.ptr(), nb_shapes);

    constexpr int MAX_RECOVER_ITERATIONS = 4;
    constexpr float MIN_PENETRATION_THRESHOLD = 1e-5f;

    Vector3 total_recovery;
    bool recovered = false;

    // Filter setup for environment overlap query
    PhysXQueryFilterCallback filter_cb;
    filter_cb.collision_mask = p_body.get_collision_mask();
    filter_cb.collide_with_bodies = true;
    filter_cb.collide_with_areas = false;
    filter_cb.exclude_rids = &p_self_and_excluded;
    filter_cb.exclude_objects = &p_excluded_objects;
    filter_cb.motion_body = &p_body;
    filter_cb.multi_hit = true; ///< _body_motion_recover: collect all overlaps

    physx::PxQueryFilterData filter_data;
    filter_data.flags = physx::PxQueryFlag::eDYNAMIC | physx::PxQueryFlag::eSTATIC | physx::PxQueryFlag::ePREFILTER;

    for (int iter = 0; iter < MAX_RECOVER_ITERATIONS; ++iter) {
        Transform3D current_transform = p_transform;
        current_transform.origin += total_recovery;

        Vector3 step_recovery;
        bool penetrating_in_this_iter = false;

        for (physx::PxU32 s = 0; s < nb_shapes; ++s) {
            physx::PxShape *body_shape = shapes[s];

            // Build body pose for this iteration
            Quaternion q = current_transform.basis.get_rotation_quaternion();
            physx::PxTransform body_pose(
                physx::PxVec3(current_transform.origin.x, current_transform.origin.y, current_transform.origin.z),
                physx::PxQuat(q.x, q.y, q.z, q.w)
            );

            // Handle separation-ray shapes as raycasts against the floor.
            if (body_shape->userData) {
                const PhysXShape3D *shape_bp = static_cast<const PhysXShape3D *>(body_shape->userData);
                if (shape_bp && shape_bp->get_type() == PhysicsServer3D::SHAPE_SEPARATION_RAY) {
                    const PhysXSeparationRayShape3D *sep_ray = static_cast<const PhysXSeparationRayShape3D *>(shape_bp);
                    const real_t ray_length = sep_ray->get_length();

                    physx::PxTransform local_pose = body_shape->getLocalPose();
                    physx::PxTransform shape_pose = body_pose * local_pose;

                    // Ray direction = the body-transformed local +Z axis. Don't
                    // hand-roll quaternion basis extraction — xform() is exact.
                    const Vector3 shape_dir = current_transform.basis.xform(Vector3(0, 0, 1)).normalized();
                    const physx::PxVec3 px_shape_dir(shape_dir.x, shape_dir.y, shape_dir.z);

                    // Adjust the ray origin to the base of the separation-ray shape.
                    // The PhysX box geometry is centered at the shape's local pose,
                    // but the ray should start from the shape's base (body origin),
                    // not its center. Subtract half-length along the body's +Z axis.
                    shape_pose.p -= px_shape_dir * (ray_length * 0.5f);

                    physx::PxRaycastBuffer ray_hit;
                    if (space->get_px_scene()->raycast(shape_pose.p, px_shape_dir, ray_length, ray_hit,
                            physx::PxHitFlag::ePOSITION | physx::PxHitFlag::eNORMAL, filter_data, &filter_cb)) {
                        if (ray_hit.hasBlock) {
                            // Depth is how far the ray tip has passed the surface.
                            // block.distance is measured along the ray from its origin,
                            // so depth = ray_length - distance is positive while penetrating.
                            const float penetration_depth = ray_length - ray_hit.block.distance;
                            const physx::PxVec3 &hit_normal_px = ray_hit.block.normal;

                            if (penetration_depth > MIN_PENETRATION_THRESHOLD) {
                                penetrating_in_this_iter = true;
                                const Vector3 pen_vec(hit_normal_px.x * penetration_depth,
                                                      hit_normal_px.y * penetration_depth,
                                                      hit_normal_px.z * penetration_depth);
                                if (step_recovery.length_squared() == 0.0f) {
                                    step_recovery = pen_vec;
                                } else {
                                    const float dot = step_recovery.normalized().dot(
                                            Vector3(hit_normal_px.x, hit_normal_px.y, hit_normal_px.z));
                                    if (dot < 0.0f) {
                                        const Vector3 sub = Vector3(hit_normal_px.x, hit_normal_px.y, hit_normal_px.z)
                                                * dot * step_recovery.length();
                                        step_recovery += pen_vec - sub;
                                    } else {
                                        step_recovery += pen_vec;
                                    }
                                }
                            }
                        }
                    }
                    continue;
                }
            }

            // Regular shapes: overlap-based recovery
            physx::PxGeometryHolder body_geom = body_shape->getGeometry();
            physx::PxTransform global_shape_pose = body_pose * body_shape->getLocalPose();

            constexpr int MAX_OVERLAPS = 32;
            physx::PxOverlapHit hit_buffer[MAX_OVERLAPS];
            physx::PxOverlapBuffer hits(hit_buffer, MAX_OVERLAPS);

            if (space->get_px_scene()->overlap(body_geom.any(), global_shape_pose, hits, filter_data, &filter_cb)) {
                for (physx::PxU32 i = 0; i < hits.getNbAnyHits(); ++i) {
                    const physx::PxOverlapHit &overlap = hits.getAnyHit(i);

                    physx::PxGeometryHolder hit_geom = overlap.shape->getGeometry();
                    physx::PxTransform hit_pose = physx::PxShapeExt::getGlobalPose(*overlap.shape, *overlap.actor);

                    physx::PxVec3 mtd_dir;
                    physx::PxReal penetration_depth = 0.0f;

                    bool is_pen = physx::PxGeometryQuery::computePenetration(
                        mtd_dir, penetration_depth,
                        body_geom.any(), global_shape_pose,
                        hit_geom.any(), hit_pose
                    );

                    if (is_pen && penetration_depth > MIN_PENETRATION_THRESHOLD) {
                        penetrating_in_this_iter = true;
                        Vector3 recovery_dir(mtd_dir.x, mtd_dir.y, mtd_dir.z);
                        Vector3 pen_vector = recovery_dir * penetration_depth;

                        if (step_recovery.length_squared() == 0.0f) {
                            step_recovery = pen_vector;
                        } else {
                            float dot = step_recovery.normalized().dot(recovery_dir);
                            if (dot < 0.0f) {
                                pen_vector -= recovery_dir * dot * step_recovery.length();
                            }
                            step_recovery += pen_vector;
                        }
                    }
                }
            }
        }

        if (!penetrating_in_this_iter || step_recovery.length_squared() < MIN_PENETRATION_THRESHOLD) {
            break;
        }

        total_recovery += step_recovery;
        recovered = true;
    }

    r_recovery = total_recovery;
    return recovered;
}

bool PhysXDirectSpaceState3D::_body_motion_cast(const PhysXBody3D &p_body, const Transform3D &p_transform,
    const Vector3 &p_motion, bool p_collide_separation_ray, const HashSet<RID> &p_self_and_excluded, const HashSet<ObjectID> &p_excluded_objects,
    real_t &r_safe_fraction, real_t &r_unsafe_fraction, Vector3 &r_hit_position, Vector3 &r_hit_normal,
    const physx::PxRigidActor *&r_hit_actor, const physx::PxShape *&r_hit_shape) const {

    r_safe_fraction = 1.0;
    r_unsafe_fraction = 1.0;

    if (!space || !space->get_px_scene()) {
        return false;
    }

    physx::PxRigidActor *actor = p_body.get_px_actor();
    if (!actor) {
        return false;
    }

    const physx::PxU32 nb_shapes = actor->getNbShapes();
    if (nb_shapes == 0) {
        return false;
    }

    // Heap-allocate the body's shape pointer list.
    LocalVector<physx::PxShape *> shapes;
    shapes.resize(nb_shapes);
    actor->getShapes(shapes.ptr(), nb_shapes);

    Vector3 dir = p_motion;
    real_t motion_length = dir.length();
    if (motion_length == 0.0) {
        return false;
    }
    dir /= motion_length;
    physx::PxVec3 px_dir(dir.x, dir.y, dir.z);

    // Setup filter callback
    PhysXQueryFilterCallback filter_cb;
    filter_cb.collision_mask = p_body.get_collision_mask();
    filter_cb.collide_with_bodies = true;
    filter_cb.collide_with_areas = false;
    filter_cb.exclude_rids = &p_self_and_excluded;
    filter_cb.exclude_objects = &p_excluded_objects;
    filter_cb.motion_body = &p_body;
    // _body_motion_cast is single-hit — keep eBLOCK for the closest hit.

    physx::PxQueryFilterData filter_data;
    filter_data.flags = physx::PxQueryFlag::eDYNAMIC | physx::PxQueryFlag::eSTATIC | physx::PxQueryFlag::ePREFILTER;

    // eMTD makes initial-contact sweeps report a well-defined MTD distance
    // (negative while penetrating, zero while merely touching) together with
    // the MTD normal/position, instead of distance==0 with undefined fields.
    // The initial-contact logic below relies on both being well-defined.
    physx::PxHitFlags sweep_hit_flags = physx::PxHitFlag::ePOSITION | physx::PxHitFlag::eNORMAL | physx::PxHitFlag::eFACE_INDEX | physx::PxHitFlag::eMTD;

    bool hit_any = false;
    float min_hit_distance = motion_length;
    Vector3 best_position;
    Vector3 best_normal;
    const physx::PxRigidActor *best_actor = nullptr;
    const physx::PxShape *best_shape = nullptr;

    for (physx::PxU32 s = 0; s < nb_shapes; ++s) {
        physx::PxShape *shape = shapes[s];
        
        // Calculate body pose once per shape iteration
        Quaternion q = p_transform.basis.get_rotation_quaternion();
        physx::PxTransform body_pose(
            physx::PxVec3(p_transform.origin.x, p_transform.origin.y, p_transform.origin.z),
            physx::PxQuat(q.x, q.y, q.z, q.w)
        );

        // Separation rays are not swept as a volume in the cast phase — they
        // participate only in recover/collide. When collide_separation_ray is
        // false they're skipped entirely. When true we fall through to the
        // generic sweep using the ray's thin-box representation, because a
        // downward raycast distance is not comparable to a horizontal sweep
        // distance and would incorrectly clamp horizontal motion.
        if (shape->userData) {
            const PhysXShape3D *shape_bp = static_cast<const PhysXShape3D *>(shape->userData);
            if (shape_bp && shape_bp->get_type() == PhysicsServer3D::SHAPE_SEPARATION_RAY) {
                if (!p_collide_separation_ray) {
                    continue;
                }
                // fall through to the generic sweep below
            }
        }

        // Regular shapes: sweep as before
        physx::PxGeometryHolder geom = shape->getGeometry();
        physx::PxTransform shape_pose = body_pose * shape->getLocalPose();

        physx::PxSweepBuffer sweep_hit;
        if (space->get_px_scene()->sweep(geom.any(), shape_pose, px_dir, motion_length, sweep_hit, sweep_hit_flags, filter_data, &filter_cb)) {
            if (sweep_hit.hasBlock) {
                // Initial-contact handling (REG-0012): the recover phase already
                // depenetrated the body before this cast, so a sweep may start
                // merely touching (distance == 0) or still penetrating
                // (distance < 0 under eMTD). PhysX reports that as a
                // zero/negative-distance block hit, which would clamp travel to
                // the recovery MTD alone and freeze the motion. Only count it
                // as a blocking hit when the motion drives the shape INTO the
                // surface (against the MTD normal); separating or tangential
                // motion passes through — the MTD is already folded into travel
                // by the caller, so the shape is ejected as expected.
                if (sweep_hit.block.distance <= 0.0f) {
                    const Vector3 mtd_normal(sweep_hit.block.normal.x, sweep_hit.block.normal.y, sweep_hit.block.normal.z);
                    if (mtd_normal.dot(dir) >= 0.0f) {
                        continue; // separating or tangential: not a forward block
                    }
                }

                if (sweep_hit.block.distance < min_hit_distance) {
                    min_hit_distance = sweep_hit.block.distance;
                    hit_any = true;
                    best_position = Vector3(sweep_hit.block.position.x, sweep_hit.block.position.y, sweep_hit.block.position.z);
                    best_normal = Vector3(sweep_hit.block.normal.x, sweep_hit.block.normal.y, sweep_hit.block.normal.z);
                    best_actor = sweep_hit.block.actor;
                    best_shape = sweep_hit.block.shape;
                }
            }
        }
    }

    if (hit_any) {
        // Initial-contact hits carry distance <= 0 (eMTD: negative while
        // penetrating); clamp the fraction so safe/unsafe stay within [0, 1].
        real_t hit_fraction = CLAMP(min_hit_distance / motion_length, 0.0, 1.0);

        r_unsafe_fraction = hit_fraction;
        // Back off slightly from the exact collision point to prevent starting inside geometry on the next frame
        r_safe_fraction = MAX(0.0, hit_fraction - (1e-4f / motion_length));
        r_hit_position = best_position;
        r_hit_normal = best_normal;
        r_hit_actor = best_actor;
        r_hit_shape = best_shape;
        return true;
    }
    return false;
}

bool PhysXDirectSpaceState3D::_body_motion_collide(const PhysXBody3D &p_body, const Transform3D &p_transform, const Vector3 &p_motion,
    int p_max_collisions, const HashSet<RID> &p_self_and_excluded, const HashSet<ObjectID> &p_excluded_objects,
    PhysicsServer3D::MotionResult *r_result) const {
        
    if (!r_result || !space || !space->get_px_scene() || p_max_collisions <= 0) {
        return false;
    }

    physx::PxRigidActor *actor = p_body.get_px_actor();
    if (!actor) {
        return false;
    }

    const physx::PxU32 nb_shapes = actor->getNbShapes();
    if (nb_shapes == 0) {
        return false;
    }

    // Heap-allocate the body's shape pointer list.
    LocalVector<physx::PxShape *> shapes;
    shapes.resize(nb_shapes);
    actor->getShapes(shapes.ptr(), nb_shapes);

    // Filter setup
    PhysXQueryFilterCallback filter_cb;
    filter_cb.collision_mask = p_body.get_collision_mask();
    filter_cb.collide_with_bodies = true;
    filter_cb.collide_with_areas = false;
    filter_cb.exclude_rids = &p_self_and_excluded;
    filter_cb.exclude_objects = &p_excluded_objects;
    filter_cb.motion_body = &p_body;
    filter_cb.multi_hit = true; ///< _body_motion_collide: collect all overlaps

    physx::PxQueryFilterData filter_data;
    filter_data.flags = physx::PxQueryFlag::eDYNAMIC | physx::PxQueryFlag::eSTATIC | physx::PxQueryFlag::ePREFILTER;

    r_result->collision_count = 0;

    const int max_cols = MIN(p_max_collisions, PhysicsServer3D::MotionResult::MAX_COLLISIONS);

    for (physx::PxU32 s = 0; s < nb_shapes; ++s) {
        if (r_result->collision_count >= max_cols) {
            break;
        }

        physx::PxShape *body_shape = shapes[s];

        // Handle separation-ray shapes in collide phase (when collide_separation_ray is true)
        if (body_shape->userData) {
            const PhysXShape3D *shape_bp = static_cast<const PhysXShape3D *>(body_shape->userData);
            if (shape_bp && shape_bp->get_type() == PhysicsServer3D::SHAPE_SEPARATION_RAY) {
                const PhysXSeparationRayShape3D *sep_ray = static_cast<const PhysXSeparationRayShape3D *>(shape_bp);
                const real_t ray_length = sep_ray->get_length();

                Quaternion q = p_transform.basis.get_rotation_quaternion();
                physx::PxTransform body_pose(
                    physx::PxVec3(p_transform.origin.x, p_transform.origin.y, p_transform.origin.z),
                    physx::PxQuat(q.x, q.y, q.z, q.w)
                );
                physx::PxTransform local_pose = body_shape->getLocalPose();
                physx::PxTransform shape_pose = body_pose * local_pose;

                // Ray direction = body-transformed local +Z axis (see recover phase).
                const Vector3 shape_dir = p_transform.basis.xform(Vector3(0, 0, 1)).normalized();
                const physx::PxVec3 px_shape_dir(shape_dir.x, shape_dir.y, shape_dir.z);

                // Adjust the ray origin to the base of the separation-ray shape.
                // The PhysX box geometry is centered at the shape's local pose,
                // but the ray should start from the shape's base (body origin),
                // not its center. Subtract half-length along the body's +Z axis.
                shape_pose.p -= px_shape_dir * (ray_length * 0.5f);

                physx::PxRaycastBuffer ray_hit;
                if (space->get_px_scene()->raycast(shape_pose.p, px_shape_dir, ray_length, ray_hit, physx::PxHitFlag::ePOSITION | physx::PxHitFlag::eNORMAL, filter_data, &filter_cb)) {
                    if (ray_hit.hasBlock) {
                        PhysicsServer3D::MotionCollision &col = r_result->collisions[r_result->collision_count];

                        col.position = Vector3(ray_hit.block.position.x, ray_hit.block.position.y, ray_hit.block.position.z);
                        col.normal = Vector3(ray_hit.block.normal.x, ray_hit.block.normal.y, ray_hit.block.normal.z);
                        // Depth = how far the ray tip has passed the surface (positive while penetrating).
                        col.depth = ray_length - ray_hit.block.distance;

                        if (ray_hit.block.actor && ray_hit.block.actor->userData) {
                            auto *actor_data = static_cast<PhysXActorUserData*>(ray_hit.block.actor->userData);
                            col.collider = actor_data->rid;
                            col.collider_id = actor_data->object_id;
                        } else {
                            col.collider = RID();
                            col.collider_id = ObjectID();
                        }

                        if (ray_hit.block.shape && ray_hit.block.shape->userData) {
                            col.collider_shape = physx_resolve_shape_index(ray_hit.block.actor, ray_hit.block.shape);
                        } else {
                            col.collider_shape = 0;
                        }

                        const int local_idx = p_body.find_shape_index(body_shape);
                        col.local_shape = local_idx >= 0 ? local_idx : (int)s;

                        col.collider_velocity = Vector3();
                        col.collider_angular_velocity = Vector3();
                        r_result->collision_count++;
                    }
                }
                continue;
            }
        }

        // Regular shapes: overlap-based collide
        if (r_result->collision_count >= max_cols) {
            break;
        }

        physx::PxShape *regular_shape = shapes[s];
        physx::PxGeometryHolder body_geom = regular_shape->getGeometry();

        Quaternion q = p_transform.basis.get_rotation_quaternion();
        physx::PxTransform body_pose(
            physx::PxVec3(p_transform.origin.x, p_transform.origin.y, p_transform.origin.z),
            physx::PxQuat(q.x, q.y, q.z, q.w)
        );
        physx::PxTransform global_shape_pose = body_pose * body_shape->getLocalPose();

        constexpr int MAX_OVERLAPS = 16;
        physx::PxOverlapHit hit_buffer[MAX_OVERLAPS];
        physx::PxOverlapBuffer hits(hit_buffer, MAX_OVERLAPS);

        if (space->get_px_scene()->overlap(body_geom.any(), global_shape_pose, hits, filter_data, &filter_cb)) {
            for (physx::PxU32 i = 0; i < hits.getNbAnyHits(); ++i) {
                if (r_result->collision_count >= max_cols) {
                    break;
                }

                const physx::PxOverlapHit &overlap = hits.getAnyHit(i);

                physx::PxGeometryHolder hit_geom = overlap.shape->getGeometry();
                physx::PxTransform hit_pose = physx::PxShapeExt::getGlobalPose(*overlap.shape, *overlap.actor);

                physx::PxVec3 mtd_dir;
                physx::PxReal penetration_depth = 0.0f;

                bool is_pen = physx::PxGeometryQuery::computePenetration(
                    mtd_dir, penetration_depth,
                    body_geom.any(), global_shape_pose,
                    hit_geom.any(), hit_pose
                );

                if (is_pen) {
                    PhysicsServer3D::MotionCollision &col = r_result->collisions[r_result->collision_count];

                    col.normal = Vector3(mtd_dir.x, mtd_dir.y, mtd_dir.z);
                    
                    physx::PxVec3 contact_pt = global_shape_pose.p - (mtd_dir * (penetration_depth * 0.5f));
                    col.position = Vector3(contact_pt.x, contact_pt.y, contact_pt.z);
                    
                    col.depth = penetration_depth;

                    if (overlap.actor && overlap.actor->userData) {
                        auto *actor_data = static_cast<PhysXActorUserData*>(overlap.actor->userData);
                        col.collider = actor_data->rid;
                        col.collider_id = actor_data->object_id;
                    } else {
                        col.collider = RID();
                        col.collider_id = ObjectID();
                    }

                    if (overlap.shape && overlap.shape->userData) {
                        col.collider_shape = physx_resolve_shape_index(overlap.actor, overlap.shape);
                    } else {
                        col.collider_shape = 0;
                    }

                    const int local_idx = p_body.find_shape_index(body_shape);
                    col.local_shape = local_idx >= 0 ? local_idx : (int)s;

                    if (overlap.actor->is<physx::PxRigidDynamic>()) {
                        physx::PxRigidDynamic *dyn = static_cast<physx::PxRigidDynamic*>(overlap.actor);
                        physx::PxVec3 lin_vel = dyn->getLinearVelocity();
                        physx::PxVec3 ang_vel = dyn->getAngularVelocity();

                        col.collider_velocity = Vector3(lin_vel.x, lin_vel.y, lin_vel.z);
                        col.collider_angular_velocity = Vector3(ang_vel.x, ang_vel.y, ang_vel.z);
                    } else {
                        col.collider_velocity = Vector3();
                        col.collider_angular_velocity = Vector3();
                    }
                    r_result->collision_count++;
                }
            }
        }
    }
    return r_result->collision_count > 0;
}
