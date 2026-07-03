// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// World core: bodies, shapes, gravity integration, broadphase update,
// snapshot/restore/hash. Every array is POD and lives in the snapshot;
// the rollback gate byte-compares all of it. Allocation goes through
// malloc for now - the world-def allocator hooks are a recorded pending
// item, due before any public release (topic-10).

#include "world_internal.h"

#include "maul2d/base.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define M2_MAX_WORLDS       16
#define M2_WORLD_COOKIE     (M2_COOKIE ^ ((int32_t)sizeof(m2WorldDef) << 8) ^ 1)
#define M2_BODY_COOKIE      (M2_COOKIE ^ ((int32_t)sizeof(m2BodyDef) << 8) ^ 2)
#define M2_SHAPE_COOKIE     (M2_COOKIE ^ ((int32_t)sizeof(m2ShapeDef) << 8) ^ 3)
#define M2_DJOINT_COOKIE    (M2_COOKIE ^ ((int32_t)sizeof(m2DistanceJointDef) << 8) ^ 4)
#define M2_RJOINT_COOKIE    (M2_COOKIE ^ ((int32_t)sizeof(m2RevoluteJointDef) << 8) ^ 5)
#define M2_PJOINT_COOKIE    (M2_COOKIE ^ ((int32_t)sizeof(m2PrismaticJointDef) << 8) ^ 6)
#define M2_WJOINT_COOKIE    (M2_COOKIE ^ ((int32_t)sizeof(m2WeldJointDef) << 8) ^ 7)
#define M2_WHJOINT_COOKIE   (M2_COOKIE ^ ((int32_t)sizeof(m2WheelJointDef) << 8) ^ 8)
#define M2_SNAPSHOT_MAGIC   0x4D32534Eu // 'M2SN'
#define M2_SNAPSHOT_VERSION 11u

// Fat margin in meters (topic-02 §3; harness-tuned later, F-T2-1).
#define M2_AABB_MARGIN 0.1

static m2World* s_worlds[M2_MAX_WORLDS];
static uint16_t s_worldGenerations[M2_MAX_WORLDS];

static m2World* GetWorld(m2WorldId id)
{
    if (id.index1 < 1 || id.index1 > M2_MAX_WORLDS)
    {
        return NULL;
    }
    m2World* world = s_worlds[id.index1 - 1];
    if (world == NULL || s_worldGenerations[id.index1 - 1] != id.generation)
    {
        return NULL;
    }
    return world;
}

m2World* m2World_GetInternal(m2WorldId worldId)
{
    return GetWorld(worldId);
}

static m2World* WorldFromIndex(uint16_t world0)
{
    if (world0 < 1 || world0 > M2_MAX_WORLDS)
    {
        return NULL;
    }
    return s_worlds[world0 - 1];
}

static m2World* GetBodyWorld(m2BodyId id)
{
    return WorldFromIndex(id.world0);
}

static int32_t BodySlot(const m2World* world, m2BodyId id)
{
    int32_t index = id.index1 - 1;
    if (index < 0 || index >= world->bodyCapacity)
    {
        return -1;
    }
    if (world->alive[index] == 0 || world->generations[index] != id.generation)
    {
        return -1;
    }
    return index;
}

static int32_t ShapeSlot(const m2World* world, m2ShapeId id)
{
    int32_t index = id.index1 - 1;
    if (index < 0 || index >= world->shapeCapacity)
    {
        return -1;
    }
    if (world->shapeAlive[index] == 0 || world->shapeGenerations[index] != id.generation)
    {
        return -1;
    }
    return index;
}

// --- Broadphase helpers ------------------------------------------------------

static m2AABB Fatten(m2AABB aabb)
{
    aabb.lowerBound.x -= M2_AABB_MARGIN;
    aabb.lowerBound.y -= M2_AABB_MARGIN;
    aabb.upperBound.x += M2_AABB_MARGIN;
    aabb.upperBound.y += M2_AABB_MARGIN;
    return aabb;
}

static m2AABB ShapeTightAABB(const m2World* world, int32_t shapeIndex)
{
    int32_t body = world->shapeBody[shapeIndex];
    return m2ComputeShapeAABB(&world->shapeGeometry[shapeIndex], world->transforms[body]);
}

static int32_t ShapeTreeIndex(const m2World* world, int32_t shapeIndex)
{
    return world->types[world->shapeBody[shapeIndex]];
}

// ALL moved proxies enter the moved set - shapes of dynamic, kinematic,
// and static bodies alike (topic-02 §4.2).
static void PushMoved(m2World* world, int32_t shapeIndex)
{
    if (world->inMoved[shapeIndex] != 0)
    {
        return;
    }
    world->inMoved[shapeIndex] = 1;
    world->moved[world->movedCount] = shapeIndex;
    world->movedCount += 1;
}

static uint64_t PairKey(int32_t a, int32_t b)
{
    uint64_t lo = (uint64_t)(a < b ? a : b);
    uint64_t hi = (uint64_t)(a < b ? b : a);
    return (lo << 32) | hi;
}

static int CompareU64(const void* a, const void* b)
{
    uint64_t ua = *(const uint64_t*)a;
    uint64_t ub = *(const uint64_t*)b;
    return ua < ub ? -1 : (ua > ub ? 1 : 0);
}

static int CompareI32(const void* a, const void* b)
{
    int32_t ia = *(const int32_t*)a;
    int32_t ib = *(const int32_t*)b;
    return ia < ib ? -1 : (ia > ib ? 1 : 0);
}

static bool ShapeIsDynamic(const m2World* world, int32_t shapeIndex)
{
    return world->types[world->shapeBody[shapeIndex]] == (uint8_t)m2_dynamicBody;
}

static m2ShapeId MakeShapeId(const m2World* world, int32_t shapeIndex)
{
    m2ShapeId id = {shapeIndex + 1, world->worldIndex0, world->shapeGenerations[shapeIndex]};
    return id;
}

static void EmitEnd(m2World* world, int32_t shapeA, int32_t shapeB)
{
    if (world->endEventCount >= world->pairCapacity)
    {
        return;
    }
    m2ContactEndEvent* e = &world->endEvents[world->endEventCount++];
    e->shapeIdA = MakeShapeId(world, shapeA);
    e->shapeIdB = MakeShapeId(world, shapeB);
    e->step = world->stepCount;
}

static void EmitBegin(m2World* world, int32_t shapeA, int32_t shapeB)
{
    if (world->beginEventCount >= world->pairCapacity)
    {
        return;
    }
    m2ContactBeginEvent* e = &world->beginEvents[world->beginEventCount++];
    e->shapeIdA = MakeShapeId(world, shapeA);
    e->shapeIdB = MakeShapeId(world, shapeB);
    e->step = world->stepCount;
}

// Re-derive pairs touched by the moved set, then batch-merge with the
// untouched remainder (topic-02 §4.2, RT1-PERF-3).
static void UpdatePairs(m2World* world)
{
    if (world->movedCount == 0)
    {
        return;
    }

    qsort(world->moved, (size_t)world->movedCount, sizeof(int32_t), CompareI32);

    int32_t collected = 0;
    int32_t queryResults[256];
    for (int32_t m = 0; m < world->movedCount; ++m)
    {
        int32_t shapeIndex = world->moved[m];
        if (world->shapeAlive[shapeIndex] == 0 || world->proxyIds[shapeIndex] == M2_NULL_NODE)
        {
            continue;
        }
        int32_t treeIndex = ShapeTreeIndex(world, shapeIndex);
        m2AABB fat = world->treeNodes[treeIndex][world->proxyIds[shapeIndex]].aabb;

        bool moverDynamic = ShapeIsDynamic(world, shapeIndex);
        int32_t firstTree = moverDynamic ? 0 : m2_dynamicBody;
        int32_t lastTree = moverDynamic ? M2_TREE_COUNT - 1 : m2_dynamicBody;
        for (int32_t t = firstTree; t <= lastTree; ++t)
        {
            int32_t hits =
                m2Tree_Query(&world->trees[t], world->treeNodes[t], fat, queryResults, 256);
            M2_ASSERT(hits <= 256);
            hits = hits <= 256 ? hits : 256;
            for (int32_t h = 0; h < hits; ++h)
            {
                int32_t other = queryResults[h];
                if (other == shapeIndex || world->shapeAlive[other] == 0)
                {
                    continue;
                }
                if (world->shapeBody[other] == world->shapeBody[shapeIndex])
                {
                    continue; // same-body shapes never pair
                }
                if (!moverDynamic && !ShapeIsDynamic(world, other))
                {
                    continue;
                }
                if (collected < world->pairCapacity)
                {
                    world->pairScratch[collected] = PairKey(shapeIndex, other);
                }
                collected += 1;
            }
        }
    }
    M2_ASSERT(collected <= world->pairCapacity);
    collected = collected <= world->pairCapacity ? collected : world->pairCapacity;

    qsort(world->pairScratch, (size_t)collected, sizeof(uint64_t), CompareU64);

    // Old set stash for the end-event diff and the touching carry.
    memcpy(world->oldPairScratch, world->pairKeys, (size_t)world->pairCount * sizeof(uint64_t));
    memcpy(world->touchingScratch, world->pairTouching, (size_t)world->pairCount * sizeof(uint8_t));
    int32_t oldCount = world->pairCount;

    int32_t kept = 0;
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        int32_t a = (int32_t)(world->pairKeys[i] >> 32);
        int32_t b = (int32_t)(world->pairKeys[i] & 0xFFFFFFFFu);
        if (world->inMoved[a] == 0 && world->inMoved[b] == 0 && world->shapeAlive[a] != 0 &&
            world->shapeAlive[b] != 0)
        {
            world->pairKeys[kept] = world->pairKeys[i];
            kept += 1;
        }
    }

    int32_t i = 0;
    int32_t j = 0;
    int32_t outCount = 0;
    uint64_t previous = 0;
    bool hasPrevious = false;
    while ((i < kept || j < collected) && outCount < world->pairCapacity)
    {
        uint64_t next;
        if (i < kept && (j >= collected || world->pairKeys[i] <= world->pairScratch[j]))
        {
            next = world->pairKeys[i];
            i += 1;
        }
        else
        {
            next = world->pairScratch[j];
            j += 1;
        }
        if (hasPrevious && next == previous)
        {
            continue;
        }
        world->pairScratch[world->pairCapacity - 1 - outCount] = next;
        previous = next;
        hasPrevious = true;
        outCount += 1;
    }
    for (int32_t k = 0; k < outCount; ++k)
    {
        world->pairKeys[k] = world->pairScratch[world->pairCapacity - 1 - (outCount - 1 - k)];
    }
    world->pairCount = outCount;

    // Diff old vs new (both sorted): vanished-and-touching pairs emit
    // their end events here (M19: pair loss is a contact-killing path);
    // surviving pairs carry their touching flag to the new slot.
    {
        int32_t oi = 0;
        int32_t ni = 0;
        while (oi < oldCount || ni < world->pairCount)
        {
            uint64_t ok = oi < oldCount ? world->oldPairScratch[oi] : UINT64_MAX;
            uint64_t nk = ni < world->pairCount ? world->pairKeys[ni] : UINT64_MAX;
            if (ok == nk)
            {
                world->pairTouching[ni] = world->touchingScratch[oi];
                oi += 1;
                ni += 1;
            }
            else if (ok < nk)
            {
                if (world->touchingScratch[oi] != 0)
                {
                    int32_t a = (int32_t)(ok >> 32);
                    int32_t b = (int32_t)(ok & 0xFFFFFFFFu);
                    if (world->shapeAlive[a] != 0 && world->shapeAlive[b] != 0)
                    {
                        EmitEnd(world, a, b); // destroy path emits its own
                    }
                }
                oi += 1;
            }
            else
            {
                world->pairTouching[ni] = 0;
                ni += 1;
            }
        }
    }

    for (int32_t m = 0; m < world->movedCount; ++m)
    {
        world->inMoved[world->moved[m]] = 0;
    }
    world->movedCount = 0;
}

static void PrunePairsOfShape(m2World* world, int32_t shapeIndex)
{
    int32_t kept = 0;
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        int32_t a = (int32_t)(world->pairKeys[i] >> 32);
        int32_t b = (int32_t)(world->pairKeys[i] & 0xFFFFFFFFu);
        if (a != shapeIndex && b != shapeIndex)
        {
            world->pairKeys[kept] = world->pairKeys[i];
            world->pairTouching[kept] = world->pairTouching[i];
            world->manifolds[kept] = world->manifolds[i];
            kept += 1;
        }
    }
    world->pairCount = kept;
}

// --- Mass ---------------------------------------------------------------------

// Deterministic: walks the body's shape list in stored (insertion) order,
// which the snapshot preserves. Dynamic bodies get the minimum-mass floor
// (RT1-NUM-2: a zero-density dynamic body must not mint an infinite
// inverse mass).
static void RecomputeMass(m2World* world, int32_t bodyIndex)
{
    if (world->types[bodyIndex] != (uint8_t)m2_dynamicBody)
    {
        world->invMass[bodyIndex] = 0.0f;
        world->invInertia[bodyIndex] = 0.0f;
        world->localCenters[bodyIndex] = (m2Vec2){0.0f, 0.0f};
        return;
    }

    float mass = 0.0f;
    float inertiaOrigin = 0.0f;
    m2Vec2 center = {0.0f, 0.0f};
    for (int32_t s = world->bodyShapeHead[bodyIndex]; s != -1; s = world->shapeNext[s])
    {
        m2MassData data = m2ComputeShapeMass(&world->shapeGeometry[s], world->shapeDensity[s]);
        mass += data.mass;
        inertiaOrigin += data.rotationalInertia;
        center.x += data.mass * data.center.x;
        center.y += data.mass * data.center.y;
    }

    if (!(mass > 0.0f))
    {
        // Floor: shapeless or zero-density dynamic bodies weigh 1 kg and
        // do not rotate from impulses (deterministic fallback, no NaN).
        world->invMass[bodyIndex] = 1.0f;
        world->invInertia[bodyIndex] = 0.0f;
        world->localCenters[bodyIndex] = (m2Vec2){0.0f, 0.0f};
        return;
    }

    float invMass = 1.0f / mass;
    center.x *= invMass;
    center.y *= invMass;
    float inertiaCenter = inertiaOrigin - mass * (center.x * center.x + center.y * center.y);
    world->invMass[bodyIndex] = invMass;
    world->invInertia[bodyIndex] = inertiaCenter > 0.0f ? 1.0f / inertiaCenter : 0.0f;
    // The body now rotates about this point: velocities, solver
    // anchors and integration all live in the COM frame.
    world->localCenters[bodyIndex] = center;
}

// --- Contacts (topic-04) -------------------------------------------------------

// Relative pose of shape B's body in shape A's body frame: the single
// f64 -> f32 crossing for the contact stage (RT1-NUM-3).
static m2RelativePose MakeRelativePose(const m2World* world, int32_t bodyA, int32_t bodyB)
{
    m2Transform xfA = world->transforms[bodyA];
    m2Transform xfB = world->transforms[bodyB];
    float dx = (float)(xfB.p.x - xfA.p.x);
    float dy = (float)(xfB.p.y - xfA.p.y);
    m2RelativePose pose;
    pose.p = (m2Vec2){xfA.q.c * dx + xfA.q.s * dy, -xfA.q.s * dx + xfA.q.c * dy};
    m2Rot rel = {xfA.q.c * xfB.q.c + xfA.q.s * xfB.q.s, xfA.q.c * xfB.q.s - xfA.q.s * xfB.q.c};
    pose.q = m2NormalizeRot(rel); // every composition renormalizes
    return pose;
}

// Convert a manifold computed with swapped shape roles back into the
// canonical frame (A = lower shape index).
static m2Manifold FlipManifold(m2Manifold in, m2RelativePose poseOfBInA)
{
    m2Manifold out = in;
    for (int32_t k = 0; k < in.pointCount; ++k)
    {
        out.points[k].anchorA = in.points[k].anchorB;
        out.points[k].anchorB = in.points[k].anchorA;
    }
    m2Vec2 n = {poseOfBInA.q.c * in.normal.x - poseOfBInA.q.s * in.normal.y,
                poseOfBInA.q.s * in.normal.x + poseOfBInA.q.c * in.normal.y};
    out.normal = (m2Vec2){-n.x, -n.y};
    return out;
}

static m2RelativePose InvertPose(m2RelativePose pose)
{
    m2RelativePose inv;
    inv.q = (m2Rot){pose.q.c, -pose.q.s};
    m2Vec2 r = {inv.q.c * pose.p.x - inv.q.s * pose.p.y, inv.q.s * pose.p.x + inv.q.c * pose.p.y};
    inv.p = (m2Vec2){-r.x, -r.y};
    return inv;
}

static m2Manifold ComputeManifold(const m2World* world, int32_t shapeA, int32_t shapeB,
                                  m2RelativePose pose)
{
    const m2ShapeGeometry* ga = &world->shapeGeometry[shapeA];
    const m2ShapeGeometry* gb = &world->shapeGeometry[shapeB];

    if (ga->type == m2_circleShape && gb->type == m2_circleShape)
    {
        return m2CollideCircles(&ga->circle, &gb->circle, pose);
    }
    if (ga->type == m2_polygonShape && gb->type == m2_circleShape)
    {
        return m2CollidePolygonAndCircle(&ga->polygon, &gb->circle, pose);
    }
    if (ga->type == m2_circleShape && gb->type == m2_polygonShape)
    {
        m2Manifold m = m2CollidePolygonAndCircle(&gb->polygon, &ga->circle, InvertPose(pose));
        return FlipManifold(m, pose);
    }

    // Capsules and segments become 2-vertex rounded polygons; every
    // remaining combination goes through the SAT+clip kernel.
    m2Polygon proxyA;
    m2Polygon proxyB;
    const m2Polygon* pa = NULL;
    const m2Polygon* pb = NULL;
    if (ga->type == m2_polygonShape)
    {
        pa = &ga->polygon;
    }
    else if (ga->type == m2_capsuleShape)
    {
        proxyA = m2MakeSegmentProxy(ga->capsule.point1, ga->capsule.point2, ga->capsule.radius);
        pa = &proxyA;
    }
    else if (ga->type == m2_segmentShape)
    {
        proxyA = m2MakeSegmentProxy(ga->segment.point1, ga->segment.point2, 0.0f);
        pa = &proxyA;
    }
    if (gb->type == m2_polygonShape)
    {
        pb = &gb->polygon;
    }
    else if (gb->type == m2_capsuleShape)
    {
        proxyB = m2MakeSegmentProxy(gb->capsule.point1, gb->capsule.point2, gb->capsule.radius);
        pb = &proxyB;
    }
    else if (gb->type == m2_segmentShape)
    {
        proxyB = m2MakeSegmentProxy(gb->segment.point1, gb->segment.point2, 0.0f);
        pb = &proxyB;
    }

    if (pa != NULL && gb->type == m2_circleShape)
    {
        return m2CollidePolygonAndCircle(pa, &gb->circle, pose);
    }
    if (ga->type == m2_circleShape && pb != NULL)
    {
        m2Manifold m = m2CollidePolygonAndCircle(pb, &ga->circle, InvertPose(pose));
        return FlipManifold(m, pose);
    }
    M2_ASSERT(pa != NULL && pb != NULL);
    return m2CollidePolygons(pa, pb, pose);
}

// Stash old (key, manifold) rows, then rebuild aligned to the new pair
// array, carrying warm-start impulses across by pair key and point id.
static void StashContacts(m2World* world)
{
    memcpy(world->oldPairScratch, world->pairKeys, (size_t)world->pairCount * sizeof(uint64_t));
    memcpy(world->manifoldScratch, world->manifolds, (size_t)world->pairCount * sizeof(m2Manifold));
}

typedef struct m2UpdateContactsCtx
{
    m2World* world;
} m2UpdateContactsCtx;

// Each pair writes only its own manifold slot, so the range splits
// freely across workers without touching the arithmetic.
static void UpdateContactsRange(int32_t begin, int32_t end, void* userCtx)
{
    m2World* world = ((m2UpdateContactsCtx*)userCtx)->world;
    for (int32_t i = begin; i < end; ++i)
    {
        int32_t shapeA = (int32_t)(world->pairKeys[i] >> 32);
        int32_t shapeB = (int32_t)(world->pairKeys[i] & 0xFFFFFFFFu);
        m2RelativePose pose =
            MakeRelativePose(world, world->shapeBody[shapeA], world->shapeBody[shapeB]);
        m2Manifold fresh = ComputeManifold(world, shapeA, shapeB, pose);

        // Locate the previous manifold for this pair (both lists sorted;
        // binary search keeps this O(P log P) worst case).
        const m2Manifold* previous = NULL;
        int32_t lo = 0;
        int32_t hi = world->oldPairCount - 1;
        while (lo <= hi)
        {
            int32_t mid = (lo + hi) / 2;
            if (world->oldPairScratch[mid] == world->pairKeys[i])
            {
                previous = &world->manifoldScratch[mid];
                break;
            }
            if (world->oldPairScratch[mid] < world->pairKeys[i])
            {
                lo = mid + 1;
            }
            else
            {
                hi = mid - 1;
            }
        }

        if (previous != NULL)
        {
            for (int32_t k = 0; k < fresh.pointCount; ++k)
            {
                for (int32_t o = 0; o < previous->pointCount; ++o)
                {
                    if (previous->points[o].id == fresh.points[k].id)
                    {
                        fresh.points[k].normalImpulse = previous->points[o].normalImpulse;
                        fresh.points[k].tangentImpulse = previous->points[o].tangentImpulse;
                        fresh.points[k].flags |= 1; // persisted
                        break;
                    }
                }
            }
        }
        world->manifolds[i] = fresh;
    }
}

static void UpdateContacts(m2World* world)
{
    m2UpdateContactsCtx ctx = {world};
    m2ThreadPoolRun(world->pool, UpdateContactsRange, &ctx, world->pairCount);
}

// --- Defs & world lifecycle ----------------------------------------------------

m2WorldDef m2DefaultWorldDef(void)
{
    m2WorldDef def;
    memset(&def, 0, sizeof(def));
    def.gravity = (m2Vec2){0.0f, -10.0f};
    def.bodyCapacity = 1024;
    def.shapeCapacity = 2048;
    def.jointCapacity = 256;
    def.internalValue = M2_WORLD_COOKIE;
    return def;
}

m2WorldId m2CreateWorld(const m2WorldDef* def)
{
    if (def == NULL || def->internalValue != M2_WORLD_COOKIE || def->bodyCapacity < 1 ||
        def->shapeCapacity < 1 || def->jointCapacity < 1)
    {
        M2_ASSERT(false);
        return m2_nullWorldId;
    }

    int32_t slot = -1;
    for (int32_t i = 0; i < M2_MAX_WORLDS; ++i)
    {
        if (s_worlds[i] == NULL)
        {
            slot = i;
            break;
        }
    }
    if (slot < 0)
    {
        return m2_nullWorldId;
    }

    m2World* world = m2AllocZeroed(sizeof(m2World));
    if (world == NULL)
    {
        return m2_nullWorldId;
    }

    int32_t cap = def->bodyCapacity;
    int32_t shapeCap = def->shapeCapacity;
    int32_t jointCap = def->jointCapacity;
    world->gravity = def->gravity;
    world->bodyCapacity = cap;
    world->shapeCapacity = shapeCap;
    world->jointCapacity = jointCap;
    world->treeNodeCapacity = 2 * shapeCap;
    world->pairCapacity = 8 * shapeCap;

    bool ok = true;
#define M2_ALLOC(field, count, type)                                                               \
    do                                                                                             \
    {                                                                                              \
        world->field = m2AllocZeroed((size_t)(count) * sizeof(type));                              \
        ok = ok && world->field != NULL;                                                           \
    } while (0)
    M2_ALLOC(transforms, cap, m2Transform);
    M2_ALLOC(linearVelocities, cap, m2Vec2);
    M2_ALLOC(angularVelocities, cap, float);
    M2_ALLOC(gravityScales, cap, float);
    M2_ALLOC(userData, cap, uint64_t);
    M2_ALLOC(types, cap, uint8_t);
    M2_ALLOC(alive, cap, uint8_t);
    M2_ALLOC(bodyShapeHead, cap, int32_t);
    M2_ALLOC(invMass, cap, float);
    M2_ALLOC(invInertia, cap, float);
    M2_ALLOC(localCenters, cap, m2Vec2);
    M2_ALLOC(asleep, cap, uint8_t);
    M2_ALLOC(sleepTimes, cap, float);
    M2_ALLOC(bullets, cap, uint8_t);
    M2_ALLOC(ccdPrevPositions, cap, m2Pos2);
    M2_ALLOC(islandParent, cap, int32_t);
    M2_ALLOC(islandDisturbed, cap, uint8_t);
    M2_ALLOC(generations, cap, uint16_t);
    M2_ALLOC(freeQueue, cap, int32_t);
    M2_ALLOC(shapeGeometry, shapeCap, m2ShapeGeometry);
    M2_ALLOC(shapeDensity, shapeCap, float);
    M2_ALLOC(shapeFriction, shapeCap, float);
    M2_ALLOC(shapeRestitution, shapeCap, float);
    M2_ALLOC(shapeUserData, shapeCap, uint64_t);
    M2_ALLOC(shapeBody, shapeCap, int32_t);
    M2_ALLOC(shapeNext, shapeCap, int32_t);
    M2_ALLOC(shapeAlive, shapeCap, uint8_t);
    M2_ALLOC(shapeGenerations, shapeCap, uint16_t);
    M2_ALLOC(shapeFreeQueue, shapeCap, int32_t);
    M2_ALLOC(proxyIds, shapeCap, int32_t);
    M2_ALLOC(inMoved, shapeCap, uint8_t);
    M2_ALLOC(moved, shapeCap, int32_t);
    M2_ALLOC(jointType, jointCap, uint8_t);
    M2_ALLOC(jointAlive, jointCap, uint8_t);
    M2_ALLOC(jointBodyA, jointCap, int32_t);
    M2_ALLOC(jointBodyB, jointCap, int32_t);
    M2_ALLOC(jointLocalAnchorA, jointCap, m2Vec2);
    M2_ALLOC(jointLocalAnchorB, jointCap, m2Vec2);
    M2_ALLOC(jointLength, jointCap, float);
    M2_ALLOC(jointHertz, jointCap, float);
    M2_ALLOC(jointDamping, jointCap, float);
    M2_ALLOC(jointImpulse, jointCap, m2Vec2);
    M2_ALLOC(jointFlags, jointCap, uint8_t);
    M2_ALLOC(jointMotorSpeed, jointCap, float);
    M2_ALLOC(jointMaxMotor, jointCap, float);
    M2_ALLOC(jointLower, jointCap, float);
    M2_ALLOC(jointUpper, jointCap, float);
    M2_ALLOC(jointLocalAxisA, jointCap, m2Vec2);
    M2_ALLOC(jointRefAngle, jointCap, float);
    M2_ALLOC(jointMotorImpulse, jointCap, float);
    M2_ALLOC(jointLowerImpulse, jointCap, float);
    M2_ALLOC(jointUpperImpulse, jointCap, float);
    M2_ALLOC(jointGenerations, jointCap, uint16_t);
    M2_ALLOC(jointFreeQueue, jointCap, int32_t);
    M2_ALLOC(pairKeys, world->pairCapacity, uint64_t);
    M2_ALLOC(pairTouching, world->pairCapacity, uint8_t);
    M2_ALLOC(touchingScratch, world->pairCapacity, uint8_t);
    M2_ALLOC(queryScratch, cap, int32_t);
    M2_ALLOC(colorMasks, cap, uint32_t);
    M2_ALLOC(constraintColors, cap, uint8_t);
    M2_ALLOC(colorOrder, cap, int32_t);

    world->pool = m2ThreadPoolCreate(def->workerCount);
    M2_ALLOC(beginEvents, world->pairCapacity, m2ContactBeginEvent);
    M2_ALLOC(endEvents, world->pairCapacity, m2ContactEndEvent);
    M2_ALLOC(pendingEndEvents, world->pairCapacity, m2ContactEndEvent);
    M2_ALLOC(pairScratch, world->pairCapacity, uint64_t);
    M2_ALLOC(manifolds, world->pairCapacity, m2Manifold);
    M2_ALLOC(oldPairScratch, world->pairCapacity, uint64_t);
    M2_ALLOC(manifoldScratch, world->pairCapacity, m2Manifold);
    M2_ALLOC(deltaPositions, cap, m2Vec2);
    M2_ALLOC(deltaRotations, cap, m2Rot);
    world->constraintScratch =
        m2AllocZeroed((size_t)world->pairCapacity * (size_t)m2ContactConstraintSize());
    ok = ok && world->constraintScratch != NULL;
#undef M2_ALLOC
    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        world->treeNodes[t] = m2AllocZeroed((size_t)world->treeNodeCapacity * sizeof(m2TreeNode));
        ok = ok && world->treeNodes[t] != NULL;
    }
    if (!ok)
    {
        m2WorldId failed = {(uint16_t)(slot + 1), s_worldGenerations[slot]};
        s_worlds[slot] = world;
        m2DestroyWorld(failed);
        return m2_nullWorldId;
    }

    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        m2Tree_Init(&world->trees[t], world->treeNodes[t], world->treeNodeCapacity);
    }
    for (int32_t i = 0; i < cap; ++i)
    {
        world->freeQueue[i] = i;
        world->bodyShapeHead[i] = -1;
    }
    for (int32_t i = 0; i < shapeCap; ++i)
    {
        world->shapeFreeQueue[i] = i;
        world->shapeNext[i] = -1;
        world->proxyIds[i] = M2_NULL_NODE;
    }
    for (int32_t i = 0; i < jointCap; ++i)
    {
        world->jointFreeQueue[i] = i;
    }
    world->jointFreeCount = jointCap;
    world->freeHead = 0;
    world->freeTail = 0;
    world->freeCount = cap;
    world->shapeFreeHead = 0;
    world->shapeFreeTail = 0;
    world->shapeFreeCount = shapeCap;

    s_worldGenerations[slot] += 1;
    world->worldGeneration = s_worldGenerations[slot];
    world->worldIndex0 = (uint16_t)(slot + 1);
    s_worlds[slot] = world;

    m2WorldId id = {(uint16_t)(slot + 1), world->worldGeneration};
    return id;
}

void m2DestroyWorld(m2WorldId worldId)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL && worldId.index1 >= 1 && worldId.index1 <= M2_MAX_WORLDS)
    {
        world = s_worlds[worldId.index1 - 1]; // failed-allocation path
    }
    if (world == NULL)
    {
        return;
    }
    m2Free(world->transforms);
    m2Free(world->linearVelocities);
    m2Free(world->angularVelocities);
    m2Free(world->gravityScales);
    m2Free(world->userData);
    m2Free(world->types);
    m2Free(world->alive);
    m2Free(world->bodyShapeHead);
    m2Free(world->invMass);
    m2Free(world->invInertia);
    m2Free(world->localCenters);
    m2Free(world->asleep);
    m2Free(world->sleepTimes);
    m2Free(world->bullets);
    m2Free(world->ccdPrevPositions);
    m2Free(world->islandParent);
    m2Free(world->islandDisturbed);
    m2Free(world->generations);
    m2Free(world->freeQueue);
    m2Free(world->shapeGeometry);
    m2Free(world->shapeDensity);
    m2Free(world->shapeFriction);
    m2Free(world->shapeRestitution);
    m2Free(world->shapeUserData);
    m2Free(world->shapeBody);
    m2Free(world->shapeNext);
    m2Free(world->shapeAlive);
    m2Free(world->shapeGenerations);
    m2Free(world->shapeFreeQueue);
    m2Free(world->proxyIds);
    m2Free(world->inMoved);
    m2Free(world->moved);
    m2Free(world->jointType);
    m2Free(world->jointAlive);
    m2Free(world->jointBodyA);
    m2Free(world->jointBodyB);
    m2Free(world->jointLocalAnchorA);
    m2Free(world->jointLocalAnchorB);
    m2Free(world->jointLength);
    m2Free(world->jointHertz);
    m2Free(world->jointDamping);
    m2Free(world->jointImpulse);
    m2Free(world->jointFlags);
    m2Free(world->jointMotorSpeed);
    m2Free(world->jointMaxMotor);
    m2Free(world->jointLower);
    m2Free(world->jointUpper);
    m2Free(world->jointLocalAxisA);
    m2Free(world->jointRefAngle);
    m2Free(world->jointMotorImpulse);
    m2Free(world->jointLowerImpulse);
    m2Free(world->jointUpperImpulse);
    m2Free(world->jointGenerations);
    m2Free(world->jointFreeQueue);
    m2Free(world->pairKeys);
    m2Free(world->pairTouching);
    m2Free(world->touchingScratch);
    m2Free(world->queryScratch);
    m2Free(world->colorMasks);
    m2Free(world->constraintColors);
    m2Free(world->colorOrder);
    m2ThreadPoolDestroy(world->pool);
    m2Free(world->beginEvents);
    m2Free(world->endEvents);
    m2Free(world->pendingEndEvents);
    m2Free(world->pairScratch);
    m2Free(world->manifolds);
    m2Free(world->oldPairScratch);
    m2Free(world->manifoldScratch);
    m2Free(world->deltaPositions);
    m2Free(world->deltaRotations);
    m2Free(world->constraintScratch);
    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        m2Free(world->treeNodes[t]);
    }
    m2Free(world);
    s_worlds[worldId.index1 - 1] = NULL;
}

bool m2World_IsValid(m2WorldId worldId)
{
    return GetWorld(worldId) != NULL;
}

void m2World_Step(m2WorldId worldId, float dt, int32_t substepCount)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL || !(dt > 0.0f) || substepCount < 1)
    {
        M2_ASSERT(world != NULL && dt > 0.0f && substepCount >= 1);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            float dt;
            int32_t substepCount;
        } marker;
        memset(&marker, 0, sizeof(marker));
        marker.dt = dt;
        marker.substepCount = substepCount;
        m2JournalRecord(world, m2_opStep, &marker, (int32_t)sizeof(marker));
    }

    // Fresh event window: clear the public buffers, then flush ends
    // queued by between-step destroys (they belong to this window).
    world->beginEventCount = 0;
    world->endEventCount = 0;
    for (int32_t i = 0; i < world->pendingEndCount && i < world->pairCapacity; ++i)
    {
        world->endEvents[world->endEventCount++] = world->pendingEndEvents[i];
    }
    world->pendingEndCount = 0;

    // Wall-clock diagnostics only; never fed back into simulation.
    uint64_t tStart = m2TimeNowNs();

    // Collide first (reference order): broadphase + narrowphase produce
    // fresh manifolds from current positions, then the solver moves the
    // world. Warm-start impulses arrive via the manifold carry.
    // Broadphase update: single-threaded, fixed body order, shape-list
    // order within a body (both snapshot-deterministic).
    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        if (world->alive[i] == 0 || world->types[i] == (uint8_t)m2_staticBody)
        {
            continue;
        }
        for (int32_t s = world->bodyShapeHead[i]; s != -1; s = world->shapeNext[s])
        {
            m2AABB tight = ShapeTightAABB(world, s);
            int32_t tree = ShapeTreeIndex(world, s);
            if (!m2AABB_Contains(world->treeNodes[tree][world->proxyIds[s]].aabb, tight))
            {
                m2Tree_Move(&world->trees[tree], world->treeNodes[tree], world->proxyIds[s],
                            Fatten(tight));
                PushMoved(world, s);
            }
        }
    }
    world->oldPairCount = world->pairCount;
    StashContacts(world);
    UpdatePairs(world);
    uint64_t tPairs = m2TimeNowNs();
    UpdateContacts(world);
    uint64_t tContacts = m2TimeNowNs();

    // Touch transitions in canonical contact order (serial compaction:
    // the topic-08 event law, scalar edition).
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        uint8_t touchingNow = world->manifolds[i].pointCount > 0 ? 1 : 0;
        if (touchingNow != world->pairTouching[i])
        {
            int32_t a = (int32_t)(world->pairKeys[i] >> 32);
            int32_t b = (int32_t)(world->pairKeys[i] & 0xFFFFFFFFu);
            if (touchingNow != 0)
            {
                EmitBegin(world, a, b);
            }
            else
            {
                EmitEnd(world, a, b);
            }
            world->pairTouching[i] = touchingNow;
        }
    }

    m2UpdateIslandsAndWake(world);
    uint64_t tIslands = m2TimeNowNs();
    m2SolveStep(world, dt, substepCount);
    uint64_t tSolve = m2TimeNowNs();
    m2UpdateSleep(world, dt);
    uint64_t tEnd = m2TimeNowNs();

    world->profile.stepMs = (float)((double)(tEnd - tStart) * 1.0e-6);
    world->profile.pairsMs = (float)((double)(tPairs - tStart) * 1.0e-6);
    world->profile.contactsMs = (float)((double)(tContacts - tPairs) * 1.0e-6);
    world->profile.solveMs = (float)((double)(tSolve - tIslands) * 1.0e-6);
    world->profile.sleepMs =
        (float)((double)(tIslands - tContacts) * 1.0e-6 + (double)(tEnd - tSolve) * 1.0e-6);

    world->stepCount += 1;
}

uint64_t m2World_GetStepCount(m2WorldId worldId)
{
    m2World* world = GetWorld(worldId);
    return world != NULL ? world->stepCount : 0;
}

// --- Snapshot -------------------------------------------------------------------

typedef struct m2SnapshotHeader
{
    uint32_t magic;
    uint32_t version;
    int32_t bodyCapacity;
    int32_t maxBodyIndex;
    uint64_t stepCount;
    m2Vec2 gravity;
    int32_t freeHead;
    int32_t freeTail;
    int32_t freeCount;
    int32_t retiredCount;
    int32_t movedCount;
    int32_t pairCount;
    int32_t shapeCapacity;
    int32_t maxShapeIndex;
    int32_t shapeFreeHead;
    int32_t shapeFreeTail;
    int32_t shapeFreeCount;
    int32_t shapeRetiredCount;
} m2SnapshotHeader;

_Static_assert(sizeof(m2SnapshotHeader) == 80, "snapshot header must be padding-free");

static int32_t WalkBlocks(m2World* world, uint8_t* out, const uint8_t* in, int direction);

// Single source of truth: the size IS the walk (measure mode). The
// duplicated byte formula died here after its third drift (assert-caught
// every time; root cause now removed).
static int32_t BlockBytes(const m2World* world)
{
    return WalkBlocks((m2World*)world, NULL, NULL, 2);
}

int32_t m2World_SnapshotSize(m2WorldId worldId)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL)
    {
        return 0;
    }
    return (int32_t)sizeof(m2SnapshotHeader) + BlockBytes(world);
}

// The block walk is shared by Snapshot and Restore so the layouts can
// never drift apart (direction 0 = write, 1 = read).
static int32_t WalkBlocks(m2World* world, uint8_t* out, const uint8_t* in, int direction)
{
    int32_t cursor = 0;
    size_t cap = (size_t)world->bodyCapacity;
    size_t shapeCap = (size_t)world->shapeCapacity;
#define M2_BLOCK(ptr, bytes)                                                                       \
    do                                                                                             \
    {                                                                                              \
        if (direction == 0)                                                                        \
        {                                                                                          \
            memcpy(out + cursor, ptr, (size_t)(bytes));                                            \
        }                                                                                          \
        else if (direction == 1)                                                                   \
        {                                                                                          \
            memcpy(ptr, in + cursor, (size_t)(bytes));                                             \
        }                                                                                          \
        cursor += (int32_t)(bytes);                                                                \
    } while (0)
    M2_BLOCK(world->transforms, cap * sizeof(m2Transform));
    M2_BLOCK(world->linearVelocities, cap * sizeof(m2Vec2));
    M2_BLOCK(world->angularVelocities, cap * sizeof(float));
    M2_BLOCK(world->gravityScales, cap * sizeof(float));
    M2_BLOCK(world->invMass, cap * sizeof(float));
    M2_BLOCK(world->invInertia, cap * sizeof(float));
    M2_BLOCK(world->localCenters, cap * sizeof(m2Vec2));
    M2_BLOCK(world->asleep, cap * sizeof(uint8_t));
    M2_BLOCK(world->sleepTimes, cap * sizeof(float));
    M2_BLOCK(world->bullets, cap * sizeof(uint8_t));
    M2_BLOCK(world->userData, cap * sizeof(uint64_t));
    M2_BLOCK(world->types, cap * sizeof(uint8_t));
    M2_BLOCK(world->alive, cap * sizeof(uint8_t));
    M2_BLOCK(world->bodyShapeHead, cap * sizeof(int32_t));
    M2_BLOCK(world->generations, cap * sizeof(uint16_t));
    M2_BLOCK(world->freeQueue, cap * sizeof(int32_t));
    M2_BLOCK(world->shapeGeometry, shapeCap * sizeof(m2ShapeGeometry));
    M2_BLOCK(world->shapeDensity, shapeCap * sizeof(float));
    M2_BLOCK(world->shapeFriction, shapeCap * sizeof(float));
    M2_BLOCK(world->shapeRestitution, shapeCap * sizeof(float));
    M2_BLOCK(world->shapeUserData, shapeCap * sizeof(uint64_t));
    M2_BLOCK(world->shapeBody, shapeCap * sizeof(int32_t));
    M2_BLOCK(world->shapeNext, shapeCap * sizeof(int32_t));
    M2_BLOCK(world->shapeAlive, shapeCap * sizeof(uint8_t));
    M2_BLOCK(world->shapeGenerations, shapeCap * sizeof(uint16_t));
    M2_BLOCK(world->shapeFreeQueue, shapeCap * sizeof(int32_t));
    M2_BLOCK(world->proxyIds, shapeCap * sizeof(int32_t));
    M2_BLOCK(world->inMoved, shapeCap * sizeof(uint8_t));
    M2_BLOCK(world->moved, shapeCap * sizeof(int32_t));
    M2_BLOCK(&world->maxJointIndex, sizeof(int32_t));
    M2_BLOCK(&world->jointFreeHead, sizeof(int32_t));
    M2_BLOCK(&world->jointFreeTail, sizeof(int32_t));
    M2_BLOCK(&world->jointFreeCount, sizeof(int32_t));
    M2_BLOCK(&world->jointRetiredCount, sizeof(int32_t));
    M2_BLOCK(world->jointType, (size_t)world->jointCapacity * sizeof(uint8_t));
    M2_BLOCK(world->jointAlive, (size_t)world->jointCapacity * sizeof(uint8_t));
    M2_BLOCK(world->jointBodyA, (size_t)world->jointCapacity * sizeof(int32_t));
    M2_BLOCK(world->jointBodyB, (size_t)world->jointCapacity * sizeof(int32_t));
    M2_BLOCK(world->jointLocalAnchorA, (size_t)world->jointCapacity * sizeof(m2Vec2));
    M2_BLOCK(world->jointLocalAnchorB, (size_t)world->jointCapacity * sizeof(m2Vec2));
    M2_BLOCK(world->jointLength, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointHertz, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointDamping, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointImpulse, (size_t)world->jointCapacity * sizeof(m2Vec2));
    M2_BLOCK(world->jointFlags, (size_t)world->jointCapacity * sizeof(uint8_t));
    M2_BLOCK(world->jointMotorSpeed, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointMaxMotor, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointLower, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointUpper, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointLocalAxisA, (size_t)world->jointCapacity * sizeof(m2Vec2));
    M2_BLOCK(world->jointRefAngle, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointMotorImpulse, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointLowerImpulse, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointUpperImpulse, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointGenerations, (size_t)world->jointCapacity * sizeof(uint16_t));
    M2_BLOCK(world->jointFreeQueue, (size_t)world->jointCapacity * sizeof(int32_t));
    M2_BLOCK(world->trees, M2_TREE_COUNT * sizeof(m2DynamicTree));
    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        M2_BLOCK(world->treeNodes[t], (size_t)world->treeNodeCapacity * sizeof(m2TreeNode));
    }
    M2_BLOCK(world->pairKeys, (size_t)world->pairCapacity * sizeof(uint64_t));
    M2_BLOCK(world->pairTouching, (size_t)world->pairCapacity * sizeof(uint8_t));
    M2_BLOCK(world->manifolds, (size_t)world->pairCapacity * sizeof(m2Manifold));
#undef M2_BLOCK
    return cursor;
}

int32_t m2World_Snapshot(m2WorldId worldId, void* buffer, int32_t capacity)
{
    m2World* world = GetWorld(worldId);
    int32_t size = m2World_SnapshotSize(worldId);
    if (world == NULL || buffer == NULL || capacity < size)
    {
        return 0;
    }

    m2SnapshotHeader header;
    memset(&header, 0, sizeof(header));
    header.magic = M2_SNAPSHOT_MAGIC;
    header.version = M2_SNAPSHOT_VERSION;
    header.bodyCapacity = world->bodyCapacity;
    header.maxBodyIndex = world->maxBodyIndex;
    header.stepCount = world->stepCount;
    header.gravity = world->gravity;
    header.freeHead = world->freeHead;
    header.freeTail = world->freeTail;
    header.freeCount = world->freeCount;
    header.retiredCount = world->retiredCount;
    header.movedCount = world->movedCount;
    header.pairCount = world->pairCount;
    header.shapeCapacity = world->shapeCapacity;
    header.maxShapeIndex = world->maxShapeIndex;
    header.shapeFreeHead = world->shapeFreeHead;
    header.shapeFreeTail = world->shapeFreeTail;
    header.shapeFreeCount = world->shapeFreeCount;
    header.shapeRetiredCount = world->shapeRetiredCount;

    uint8_t* out = buffer;
    memcpy(out, &header, sizeof(header));
    int32_t cursor = (int32_t)sizeof(header) + WalkBlocks(world, out + sizeof(header), NULL, 0);
    M2_ASSERT(cursor == size);
    return cursor;
}

bool m2World_Restore(m2WorldId worldId, const void* buffer, int32_t size)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL || buffer == NULL || size < (int32_t)sizeof(m2SnapshotHeader))
    {
        return false;
    }
    m2SnapshotHeader header;
    memcpy(&header, buffer, sizeof(header));
    if (header.magic != M2_SNAPSHOT_MAGIC || header.version != M2_SNAPSHOT_VERSION ||
        header.bodyCapacity != world->bodyCapacity ||
        header.shapeCapacity != world->shapeCapacity ||
        size != (int32_t)sizeof(header) + BlockBytes(world))
    {
        return false;
    }

    world->maxBodyIndex = header.maxBodyIndex;
    world->stepCount = header.stepCount;
    world->gravity = header.gravity;
    world->freeHead = header.freeHead;
    world->freeTail = header.freeTail;
    world->freeCount = header.freeCount;
    world->retiredCount = header.retiredCount;
    world->movedCount = header.movedCount;
    world->pairCount = header.pairCount;
    world->maxShapeIndex = header.maxShapeIndex;
    world->shapeFreeHead = header.shapeFreeHead;
    world->shapeFreeTail = header.shapeFreeTail;
    world->shapeFreeCount = header.shapeFreeCount;
    world->shapeRetiredCount = header.shapeRetiredCount;

    const uint8_t* in = buffer;
    int32_t cursor = (int32_t)sizeof(header) + WalkBlocks(world, NULL, in + sizeof(header), 1);
    M2_ASSERT(cursor == size);
    (void)cursor;

    // Restore during recording stops the journal (recorded limitation).
    world->journalActive = 0;

    // Events are an observer stream from an abandoned timeline: cleared
    // on restore, re-emitted by re-simulation (RT1-ROLL-3 / RT1-API-3).
    world->beginEventCount = 0;
    world->endEventCount = 0;
    world->pendingEndCount = 0;
    return true;
}

uint64_t m2World_Hash(m2WorldId worldId)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL)
    {
        return 0;
    }
    uint64_t h = M2_HASH_INIT;
    h = m2Hash64(h, &world->stepCount, (int32_t)sizeof(world->stepCount));
    h = m2Hash64(h, &world->gravity, (int32_t)sizeof(world->gravity));
    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        if (world->alive[i] == 0)
        {
            continue;
        }
        h = m2Hash64(h, &world->transforms[i], (int32_t)sizeof(m2Transform));
        h = m2Hash64(h, &world->linearVelocities[i], (int32_t)sizeof(m2Vec2));
        h = m2Hash64(h, &world->angularVelocities[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->invMass[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->invInertia[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->localCenters[i], (int32_t)sizeof(m2Vec2));
        h = m2Hash64(h, &world->types[i], (int32_t)sizeof(uint8_t));
        h = m2Hash64(h, &world->asleep[i], (int32_t)sizeof(uint8_t));
        h = m2Hash64(h, &world->sleepTimes[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->bullets[i], (int32_t)sizeof(uint8_t));
    }
    h = m2Hash64(h, world->pairKeys, world->pairCount * (int32_t)sizeof(uint64_t));
    h = m2Hash64(h, world->manifolds, world->pairCount * (int32_t)sizeof(m2Manifold));
    for (int32_t i = 0; i < world->maxJointIndex; ++i)
    {
        if (world->jointAlive[i] == 0)
        {
            continue;
        }
        h = m2Hash64(h, &world->jointImpulse[i], (int32_t)sizeof(m2Vec2));
        h = m2Hash64(h, &world->jointMotorImpulse[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->jointLowerImpulse[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->jointUpperImpulse[i], (int32_t)sizeof(float));
    }
    return h;
}

// --- Bodies ---------------------------------------------------------------------

m2BodyDef m2DefaultBodyDef(void)
{
    m2BodyDef def;
    memset(&def, 0, sizeof(def));
    def.type = m2_staticBody;
    def.rotation = (m2Rot){1.0f, 0.0f};
    def.gravityScale = 1.0f;
    def.internalValue = M2_BODY_COOKIE;
    return def;
}

m2BodyId m2CreateBody(m2WorldId worldId, const m2BodyDef* def)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_BODY_COOKIE)
    {
        M2_ASSERT(false);
        return m2_nullBodyId;
    }
    if (world->freeCount == 0)
    {
        return m2_nullBodyId;
    }

    int32_t index = world->freeQueue[world->freeHead];
    world->freeHead = (world->freeHead + 1) % world->bodyCapacity;
    world->freeCount -= 1;

    world->transforms[index].p = def->position;
    world->transforms[index].q = m2NormalizeRot(def->rotation);
    world->linearVelocities[index] = def->linearVelocity;
    world->angularVelocities[index] = def->angularVelocity;
    world->gravityScales[index] = def->gravityScale;
    world->userData[index] = def->userData;
    world->types[index] = (uint8_t)def->type;
    world->alive[index] = 1;
    world->bodyShapeHead[index] = -1;
    world->invMass[index] = def->type == m2_dynamicBody ? 1.0f : 0.0f; // shapeless floor
    world->localCenters[index] = (m2Vec2){0.0f, 0.0f};
    world->invInertia[index] = 0.0f;
    world->asleep[index] = 0;
    world->sleepTimes[index] = 0.0f;
    world->bullets[index] = def->isBullet ? 1 : 0;
    if (index + 1 > world->maxBodyIndex)
    {
        world->maxBodyIndex = index + 1;
    }

    m2BodyId id = {index + 1, worldId.index1, world->generations[index]};

    if (world->journalActive != 0)

    {

        struct

        {

            m2BodyDef def;

            m2BodyId expected;

        } record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = id;

        m2JournalRecord(world, m2_opCreateBody, &record, (int32_t)sizeof(record));
    }
    return id;
}

static void DestroyShapeInternal(m2World* world, int32_t shapeIndex)
{
    // M19 bookending: a destroyed shape ends every touching contact it
    // had, id captured before the generation bump. Between steps these
    // land in the pending queue and flush into the next step's buffers.
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        if (world->pairTouching[i] == 0)
        {
            continue;
        }
        int32_t a = (int32_t)(world->pairKeys[i] >> 32);
        int32_t b = (int32_t)(world->pairKeys[i] & 0xFFFFFFFFu);
        if (a != shapeIndex && b != shapeIndex)
        {
            continue;
        }
        if (world->pendingEndCount < world->pairCapacity)
        {
            m2ContactEndEvent* e = &world->pendingEndEvents[world->pendingEndCount++];
            e->shapeIdA = MakeShapeId(world, a);
            e->shapeIdB = MakeShapeId(world, b);
            e->step = world->stepCount;
        }
    }

    if (world->proxyIds[shapeIndex] != M2_NULL_NODE)
    {
        int32_t tree = ShapeTreeIndex(world, shapeIndex);
        m2Tree_Remove(&world->trees[tree], world->treeNodes[tree], world->proxyIds[shapeIndex]);
        world->proxyIds[shapeIndex] = M2_NULL_NODE;
    }
    PrunePairsOfShape(world, shapeIndex);

    world->shapeAlive[shapeIndex] = 0;
    if (world->shapeGenerations[shapeIndex] == UINT16_MAX)
    {
        world->shapeRetiredCount += 1;
        return;
    }
    world->shapeGenerations[shapeIndex] += 1;
    world->shapeFreeQueue[world->shapeFreeTail] = shapeIndex;
    world->shapeFreeTail = (world->shapeFreeTail + 1) % world->shapeCapacity;
    world->shapeFreeCount += 1;
}

void m2DestroyShape(m2ShapeId shapeId)
{
    m2World* world = WorldFromIndex(shapeId.world0);
    if (world == NULL)
    {
        return;
    }
    int32_t index = shapeId.index1 - 1;
    if (index < 0 || index >= world->shapeCapacity || world->shapeAlive[index] == 0 ||
        world->shapeGenerations[index] != shapeId.generation)
    {
        return;
    }
    m2JournalRecord(world, m2_opDestroyShape, &shapeId, (int32_t)sizeof(shapeId));

    int32_t bodyIndex = world->shapeBody[index];
    // Unlink from the body's shape list (insertion-ordered, singly
    // linked - the walk is canonical).
    if (world->bodyShapeHead[bodyIndex] == index)
    {
        world->bodyShapeHead[bodyIndex] = world->shapeNext[index];
    }
    else
    {
        for (int32_t s = world->bodyShapeHead[bodyIndex]; s != -1; s = world->shapeNext[s])
        {
            if (world->shapeNext[s] == index)
            {
                world->shapeNext[s] = world->shapeNext[index];
                break;
            }
        }
    }
    world->shapeNext[index] = -1;

    DestroyShapeInternal(world, index);
    RecomputeMass(world, bodyIndex);
    if (world->types[bodyIndex] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[bodyIndex] = 0;
        world->sleepTimes[bodyIndex] = 0.0f;
    }
}

void m2DestroyBody(m2BodyId bodyId)
{
    m2World* world = GetBodyWorld(bodyId);
    if (world == NULL)
    {
        return;
    }
    int32_t index = BodySlot(world, bodyId);
    if (index < 0)
    {
        return;
    }
    m2JournalRecord(world, m2_opDestroyBody, &bodyId, (int32_t)sizeof(bodyId));

    // Joints attached to this body die with it; the counterpart body
    // wakes (a support vanished).
    for (int32_t j = 0; j < world->maxJointIndex; ++j)
    {
        if (world->jointAlive[j] == 0 ||
            (world->jointBodyA[j] != index && world->jointBodyB[j] != index))
        {
            continue;
        }
        int32_t other = world->jointBodyA[j] == index ? world->jointBodyB[j] : world->jointBodyA[j];
        if (world->types[other] == (uint8_t)m2_dynamicBody)
        {
            world->asleep[other] = 0;
            world->sleepTimes[other] = 0.0f;
        }
        world->jointAlive[j] = 0;
        if (world->jointGenerations[j] == UINT16_MAX)
        {
            world->jointRetiredCount += 1;
        }
        else
        {
            world->jointGenerations[j] += 1;
            world->jointFreeQueue[world->jointFreeTail] = j;
            world->jointFreeTail = (world->jointFreeTail + 1) % world->jointCapacity;
            world->jointFreeCount += 1;
        }
    }

    // Cascade: destroy the shape list (the contact-killing path that event
    // bookending will hang end-touch emission on, registry M19).
    int32_t s = world->bodyShapeHead[index];
    while (s != -1)
    {
        int32_t next = world->shapeNext[s];
        DestroyShapeInternal(world, s);
        s = next;
    }
    world->bodyShapeHead[index] = -1;

    world->alive[index] = 0;
    if (world->generations[index] == UINT16_MAX)
    {
        world->retiredCount += 1;
        return;
    }
    world->generations[index] += 1;
    world->freeQueue[world->freeTail] = index;
    world->freeTail = (world->freeTail + 1) % world->bodyCapacity;
    world->freeCount += 1;
}

bool m2Body_IsValid(m2BodyId bodyId)
{
    m2World* world = GetBodyWorld(bodyId);
    return world != NULL && BodySlot(world, bodyId) >= 0;
}

#define M2_BODY_GETTER(returnType, name, expr, fallback)                                           \
    returnType name(m2BodyId bodyId)                                                               \
    {                                                                                              \
        m2World* world = GetBodyWorld(bodyId);                                                     \
        int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;                              \
        if (index < 0)                                                                             \
        {                                                                                          \
            M2_ASSERT(false);                                                                      \
            return fallback;                                                                       \
        }                                                                                          \
        return expr;                                                                               \
    }

M2_BODY_GETTER(m2Transform, m2Body_GetTransform, world->transforms[index],
               ((m2Transform){{0.0, 0.0}, {1.0f, 0.0f}}))
M2_BODY_GETTER(m2Pos2, m2Body_GetPosition, world->transforms[index].p, ((m2Pos2){0.0, 0.0}))
M2_BODY_GETTER(m2Rot, m2Body_GetRotation, world->transforms[index].q, ((m2Rot){1.0f, 0.0f}))
M2_BODY_GETTER(m2Vec2, m2Body_GetLinearVelocity, world->linearVelocities[index],
               ((m2Vec2){0.0f, 0.0f}))
M2_BODY_GETTER(float, m2Body_GetAngularVelocity, world->angularVelocities[index], 0.0f)
M2_BODY_GETTER(uint64_t, m2Body_GetUserData, world->userData[index], 0)

float m2Body_GetMass(m2BodyId bodyId)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return 0.0f;
    }
    return world->invMass[index] > 0.0f ? 1.0f / world->invMass[index] : 0.0f;
}

void m2Body_SetLinearVelocity(m2BodyId bodyId, m2Vec2 velocity)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return;
    }
    world->linearVelocities[index] = velocity;
    world->asleep[index] = 0;
    world->sleepTimes[index] = 0.0f;
    if (world->journalActive != 0)
    {
        struct
        {
            m2BodyId body;
            m2Vec2 value;
        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.value = velocity;
        m2JournalRecord(world, m2_opSetLinearVelocity, &record, (int32_t)sizeof(record));
    }
}

void m2Body_SetAngularVelocity(m2BodyId bodyId, float velocity)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return;
    }
    world->angularVelocities[index] = velocity;
    world->asleep[index] = 0;
    world->sleepTimes[index] = 0.0f;
    if (world->journalActive != 0)
    {
        struct
        {
            m2BodyId body;
            float value;
        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.value = velocity;
        m2JournalRecord(world, m2_opSetAngularVelocity, &record, (int32_t)sizeof(record));
    }
}

void m2Body_ApplyLinearImpulse(m2BodyId bodyId, m2Vec2 impulse, m2Pos2 worldPoint)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0 || world->types[index] != (uint8_t)m2_dynamicBody)
    {
        M2_ASSERT(false);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2BodyId body;
            m2Vec2 impulse;
            m2Pos2 point;
        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.impulse = impulse;
        record.point = worldPoint;
        m2JournalRecord(world, m2_opApplyLinearImpulse, &record, (int32_t)sizeof(record));
    }
    // Arm from the center of mass; the single f64 crossing.
    m2Transform xf = world->transforms[index];
    m2Vec2 rlc = {xf.q.c * world->localCenters[index].x - xf.q.s * world->localCenters[index].y,
                  xf.q.s * world->localCenters[index].x + xf.q.c * world->localCenters[index].y};
    m2Vec2 r = {(float)(worldPoint.x - xf.p.x) - rlc.x, (float)(worldPoint.y - xf.p.y) - rlc.y};
    world->linearVelocities[index].x += world->invMass[index] * impulse.x;
    world->linearVelocities[index].y += world->invMass[index] * impulse.y;
    world->angularVelocities[index] +=
        world->invInertia[index] * (r.x * impulse.y - r.y * impulse.x);
    world->asleep[index] = 0;
    world->sleepTimes[index] = 0.0f;
}

void m2Body_ApplyAngularImpulse(m2BodyId bodyId, float impulse)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0 || world->types[index] != (uint8_t)m2_dynamicBody)
    {
        M2_ASSERT(false);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2BodyId body;
            float value;
        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.value = impulse;
        m2JournalRecord(world, m2_opApplyAngularImpulse, &record, (int32_t)sizeof(record));
    }
    world->angularVelocities[index] += world->invInertia[index] * impulse;
    world->asleep[index] = 0;
    world->sleepTimes[index] = 0.0f;
}

bool m2Body_IsAwake(m2BodyId bodyId)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        return false;
    }
    return world->types[index] == (uint8_t)m2_dynamicBody ? world->asleep[index] == 0 : true;
}

// --- Shapes ---------------------------------------------------------------------

m2ShapeDef m2DefaultShapeDef(void)
{
    m2ShapeDef def;
    memset(&def, 0, sizeof(def));
    def.density = 1.0f;
    def.friction = 0.6f;
    def.restitution = 0.0f;
    def.internalValue = M2_SHAPE_COOKIE;
    return def;
}

static m2ShapeId CreateShape(m2BodyId bodyId, const m2ShapeDef* def,
                             const m2ShapeGeometry* geometry)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t bodyIndex = world != NULL ? BodySlot(world, bodyId) : -1;
    if (bodyIndex < 0 || def == NULL || def->internalValue != M2_SHAPE_COOKIE ||
        !(def->density >= 0.0f) || !(def->friction >= 0.0f) ||
        !(def->restitution >= 0.0f && def->restitution <= 1.0f))
    {
        M2_ASSERT(false);
        return m2_nullShapeId;
    }
    if (world->shapeFreeCount == 0)
    {
        return m2_nullShapeId;
    }

    int32_t index = world->shapeFreeQueue[world->shapeFreeHead];
    world->shapeFreeHead = (world->shapeFreeHead + 1) % world->shapeCapacity;
    world->shapeFreeCount -= 1;

    // memset first: deterministic union tail bytes in the snapshot.
    memset(&world->shapeGeometry[index], 0, sizeof(m2ShapeGeometry));
    world->shapeGeometry[index] = *geometry;
    world->shapeDensity[index] = def->density;
    world->shapeFriction[index] = def->friction;
    world->shapeRestitution[index] = def->restitution;
    world->shapeUserData[index] = def->userData;
    world->shapeBody[index] = bodyIndex;
    world->shapeNext[index] = world->bodyShapeHead[bodyIndex];
    world->bodyShapeHead[bodyIndex] = index;
    world->shapeAlive[index] = 1;
    if (index + 1 > world->maxShapeIndex)
    {
        world->maxShapeIndex = index + 1;
    }

    int32_t tree = world->types[bodyIndex];
    world->proxyIds[index] = m2Tree_Insert(&world->trees[tree], world->treeNodes[tree],
                                           Fatten(ShapeTightAABB(world, index)), index);
    if (world->proxyIds[index] == M2_NULL_NODE)
    {
        // Node pool exhausted: undo everything; capacity error, not UB.
        world->bodyShapeHead[bodyIndex] = world->shapeNext[index];
        world->shapeAlive[index] = 0;
        world->shapeFreeHead =
            (world->shapeFreeHead + world->shapeCapacity - 1) % world->shapeCapacity;
        world->shapeFreeQueue[world->shapeFreeHead] = index;
        world->shapeFreeCount += 1;
        return m2_nullShapeId;
    }
    PushMoved(world, index);
    RecomputeMass(world, bodyIndex);

    m2ShapeId id = {index + 1, bodyId.world0, world->shapeGenerations[index]};

    if (world->journalActive != 0)

    {

        struct

        {

            m2BodyId body;

            m2ShapeDef def;

            m2ShapeGeometry geometry;

            m2ShapeId expected;

        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.def = *def;
        record.geometry = *geometry;
        record.expected = id;

        m2JournalRecord(world, m2_opCreateShape, &record, (int32_t)sizeof(record));
    }
    return id;
}

#define M2_SHAPE_CTOR(name, geomType, enumValue, validator, member)                                \
    m2ShapeId name(m2BodyId bodyId, const m2ShapeDef* def, const geomType* geom)                   \
    {                                                                                              \
        if (!validator(geom))                                                                      \
        {                                                                                          \
            M2_ASSERT(false);                                                                      \
            return m2_nullShapeId;                                                                 \
        }                                                                                          \
        m2ShapeGeometry geometry;                                                                  \
        memset(&geometry, 0, sizeof(geometry));                                                    \
        geometry.type = enumValue;                                                                 \
        geometry.member = *geom;                                                                   \
        return CreateShape(bodyId, def, &geometry);                                                \
    }

M2_SHAPE_CTOR(m2CreateCircleShape, m2Circle, m2_circleShape, m2ValidateCircle, circle)
M2_SHAPE_CTOR(m2CreateCapsuleShape, m2Capsule, m2_capsuleShape, m2ValidateCapsule, capsule)
M2_SHAPE_CTOR(m2CreatePolygonShape, m2Polygon, m2_polygonShape, m2ValidatePolygon, polygon)
M2_SHAPE_CTOR(m2CreateSegmentShape, m2Segment, m2_segmentShape, m2ValidateSegment, segment)

bool m2Shape_IsValid(m2ShapeId shapeId)
{
    m2World* world = WorldFromIndex(shapeId.world0);
    return world != NULL && ShapeSlot(world, shapeId) >= 0;
}

m2BodyId m2Shape_GetBody(m2ShapeId shapeId)
{
    m2World* world = WorldFromIndex(shapeId.world0);
    int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return m2_nullBodyId;
    }
    int32_t bodyIndex = world->shapeBody[index];
    m2BodyId id = {bodyIndex + 1, shapeId.world0, world->generations[bodyIndex]};
    return id;
}

uint64_t m2Shape_GetUserData(m2ShapeId shapeId)
{
    m2World* world = WorldFromIndex(shapeId.world0);
    int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return 0;
    }
    return world->shapeUserData[index];
}

m2ContactEvents m2World_GetContactEvents(m2WorldId worldId)
{
    m2ContactEvents events = {NULL, NULL, 0, 0};
    m2World* world = GetWorld(worldId);
    if (world == NULL)
    {
        return events;
    }
    events.beginEvents = world->beginEvents;
    events.beginCount = world->beginEventCount;
    events.endEvents = world->endEvents;
    events.endCount = world->endEventCount;
    return events;
}

m2Profile m2World_GetProfile(m2WorldId worldId)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL)
    {
        m2Profile zero = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        return zero;
    }
    return world->profile;
}

double m2World_GetKineticEnergy(m2WorldId worldId)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL)
    {
        return 0.0;
    }
    double energy = 0.0;
    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        if (world->alive[i] == 0 || world->types[i] != (uint8_t)m2_dynamicBody ||
            world->invMass[i] == 0.0f)
        {
            continue;
        }
        double vx = (double)world->linearVelocities[i].x;
        double vy = (double)world->linearVelocities[i].y;
        energy += 0.5 * (1.0 / (double)world->invMass[i]) * (vx * vx + vy * vy);
        if (world->invInertia[i] > 0.0f)
        {
            double w = (double)world->angularVelocities[i];
            energy += 0.5 * (1.0 / (double)world->invInertia[i]) * w * w;
        }
    }
    return energy;
}

m2Counters m2World_GetCounters(m2WorldId worldId)
{
    m2Counters counters;
    memset(&counters, 0, sizeof(counters));
    m2World* world = GetWorld(worldId);
    if (world == NULL)
    {
        return counters;
    }
    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        if (world->alive[i] == 0)
        {
            continue;
        }
        counters.bodies += 1;
        counters.awakeBodies += world->asleep[i] == 0 ? 1 : 0;
    }
    for (int32_t i = 0; i < world->maxShapeIndex; ++i)
    {
        counters.shapes += world->shapeAlive[i] != 0 ? 1 : 0;
    }
    for (int32_t i = 0; i < world->maxJointIndex; ++i)
    {
        counters.joints += world->jointAlive[i] != 0 ? 1 : 0;
    }
    counters.pairs = world->pairCount;
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        counters.touchingPairs += world->pairTouching[i] != 0 ? 1 : 0;
    }
    counters.constraints = world->lastConstraintCount;
    counters.graphColors = world->lastGraphColors;
    counters.overflowConstraints = world->lastOverflow;
    counters.stepCount = world->stepCount;
    return counters;
}

static int32_t AllocateJoint(m2World* world)
{
    if (world->jointFreeCount == 0)
    {
        return -1;
    }
    int32_t index = world->jointFreeQueue[world->jointFreeHead];
    world->jointFreeHead = (world->jointFreeHead + 1) % world->jointCapacity;
    world->jointFreeCount -= 1;
    if (index + 1 > world->maxJointIndex)
    {
        world->maxJointIndex = index + 1;
    }
    return index;
}

// Relative angle of B vs A from their rotations (own trig: ADR-0010).
static float RelativeJointAngle(m2Rot qA, m2Rot qB)
{
    float sin = qA.c * qB.s - qA.s * qB.c;
    float cos = qA.c * qB.c + qA.s * qB.s;
    return m2Atan2(sin, cos);
}

static m2JointId FinishJoint(m2World* world, m2WorldId worldId, int32_t index, uint8_t type,
                             int32_t bodyA, int32_t bodyB, m2Vec2 anchorA, m2Vec2 anchorB,
                             float length, float hertz, float damping)
{
    world->jointType[index] = type;
    world->jointBodyA[index] = bodyA;
    world->jointBodyB[index] = bodyB;
    world->jointLocalAnchorA[index] = anchorA;
    world->jointLocalAnchorB[index] = anchorB;
    world->jointLength[index] = length;
    world->jointHertz[index] = hertz;
    world->jointDamping[index] = damping;
    world->jointImpulse[index] = (m2Vec2){0.0f, 0.0f};
    world->jointFlags[index] = 0;
    world->jointMotorSpeed[index] = 0.0f;
    world->jointMaxMotor[index] = 0.0f;
    world->jointLower[index] = 0.0f;
    world->jointUpper[index] = 0.0f;
    world->jointLocalAxisA[index] = (m2Vec2){1.0f, 0.0f};
    world->jointRefAngle[index] = 0.0f;
    world->jointMotorImpulse[index] = 0.0f;
    world->jointLowerImpulse[index] = 0.0f;
    world->jointUpperImpulse[index] = 0.0f;
    world->jointAlive[index] = 1;
    // A new constraint wakes both ends.
    world->asleep[bodyA] = 0;
    world->sleepTimes[bodyA] = 0.0f;
    world->asleep[bodyB] = 0;
    world->sleepTimes[bodyB] = 0.0f;
    m2JointId id = {index + 1, worldId.index1, world->jointGenerations[index]};
    return id;
}

m2DistanceJointDef m2DefaultDistanceJointDef(void)
{
    m2DistanceJointDef def;
    memset(&def, 0, sizeof(def));
    def.internalValue = M2_DJOINT_COOKIE;
    return def;
}

m2RevoluteJointDef m2DefaultRevoluteJointDef(void)
{
    m2RevoluteJointDef def;
    memset(&def, 0, sizeof(def));
    def.internalValue = M2_RJOINT_COOKIE;
    return def;
}

m2JointId m2CreateDistanceJoint(m2WorldId worldId, const m2DistanceJointDef* def)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_DJOINT_COOKIE)
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    int32_t bodyA = BodySlot(world, def->bodyIdA);
    int32_t bodyB = BodySlot(world, def->bodyIdB);
    if (bodyA < 0 || bodyB < 0 || bodyA == bodyB)
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    int32_t index = AllocateJoint(world);
    if (index < 0)
    {
        return m2_nullJointId;
    }
    float length = def->length;
    if (!(length > 0.0f))
    {
        // Derive from spawn poses: the single f64 crossing.
        m2Transform xfA = world->transforms[bodyA];
        m2Transform xfB = world->transforms[bodyB];
        m2Vec2 wA = {xfA.q.c * def->localAnchorA.x - xfA.q.s * def->localAnchorA.y,
                     xfA.q.s * def->localAnchorA.x + xfA.q.c * def->localAnchorA.y};
        m2Vec2 wB = {xfB.q.c * def->localAnchorB.x - xfB.q.s * def->localAnchorB.y,
                     xfB.q.s * def->localAnchorB.x + xfB.q.c * def->localAnchorB.y};
        float dx = (float)(xfB.p.x - xfA.p.x) + wB.x - wA.x;
        float dy = (float)(xfB.p.y - xfA.p.y) + wB.y - wA.y;
        length = sqrtf(dx * dx + dy * dy);
    }
    m2JointId jointId = FinishJoint(world, worldId, index, 0, bodyA, bodyB, def->localAnchorA,
                                    def->localAnchorB, length, def->hertz, def->dampingRatio);
    if (world->journalActive != 0)
    {
        struct
        {
            m2DistanceJointDef def;
            m2JointId expected;
        } record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateDistanceJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

m2JointId m2CreateRevoluteJoint(m2WorldId worldId, const m2RevoluteJointDef* def)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_RJOINT_COOKIE)
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    int32_t bodyA = BodySlot(world, def->bodyIdA);
    int32_t bodyB = BodySlot(world, def->bodyIdB);
    if (bodyA < 0 || bodyB < 0 || bodyA == bodyB)
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    int32_t index = AllocateJoint(world);
    if (index < 0)
    {
        return m2_nullJointId;
    }
    m2JointId jointId = FinishJoint(world, worldId, index, 1, bodyA, bodyB, def->localAnchorA,
                                    def->localAnchorB, 0.0f, def->hertz, def->dampingRatio);
    world->jointFlags[index] = (def->enableMotor ? 1u : 0u) | (def->enableLimit ? 2u : 0u);
    world->jointMotorSpeed[index] = def->motorSpeed;
    world->jointMaxMotor[index] = def->maxMotorTorque;
    world->jointLower[index] = def->lowerAngle;
    world->jointUpper[index] = def->upperAngle;
    world->jointRefAngle[index] =
        RelativeJointAngle(world->transforms[bodyA].q, world->transforms[bodyB].q);
    if (world->journalActive != 0)
    {
        struct
        {
            m2RevoluteJointDef def;
            m2JointId expected;
        } record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateRevoluteJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

m2PrismaticJointDef m2DefaultPrismaticJointDef(void)
{
    m2PrismaticJointDef def;
    memset(&def, 0, sizeof(def));
    def.localAxisA = (m2Vec2){1.0f, 0.0f};
    def.internalValue = M2_PJOINT_COOKIE;
    return def;
}

m2JointId m2CreatePrismaticJoint(m2WorldId worldId, const m2PrismaticJointDef* def)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_PJOINT_COOKIE)
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    int32_t bodyA = BodySlot(world, def->bodyIdA);
    int32_t bodyB = BodySlot(world, def->bodyIdB);
    if (bodyA < 0 || bodyB < 0 || bodyA == bodyB)
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    float axisLength =
        sqrtf(def->localAxisA.x * def->localAxisA.x + def->localAxisA.y * def->localAxisA.y);
    if (!(axisLength > 1.19209290e-7f))
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    int32_t index = AllocateJoint(world);
    if (index < 0)
    {
        return m2_nullJointId;
    }
    m2JointId jointId = FinishJoint(world, worldId, index, 2, bodyA, bodyB, def->localAnchorA,
                                    def->localAnchorB, 0.0f, def->hertz, def->dampingRatio);
    world->jointFlags[index] = (def->enableMotor ? 1u : 0u) | (def->enableLimit ? 2u : 0u);
    world->jointMotorSpeed[index] = def->motorSpeed;
    world->jointMaxMotor[index] = def->maxMotorForce;
    world->jointLower[index] = def->lowerTranslation;
    world->jointUpper[index] = def->upperTranslation;
    world->jointLocalAxisA[index] =
        (m2Vec2){def->localAxisA.x / axisLength, def->localAxisA.y / axisLength};
    world->jointRefAngle[index] =
        RelativeJointAngle(world->transforms[bodyA].q, world->transforms[bodyB].q);
    if (world->journalActive != 0)
    {
        struct
        {
            m2PrismaticJointDef def;
            m2JointId expected;
        } record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreatePrismaticJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

m2WeldJointDef m2DefaultWeldJointDef(void)
{
    m2WeldJointDef def;
    memset(&def, 0, sizeof(def));
    def.internalValue = M2_WJOINT_COOKIE;
    return def;
}

m2JointId m2CreateWeldJoint(m2WorldId worldId, const m2WeldJointDef* def)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_WJOINT_COOKIE)
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    int32_t bodyA = BodySlot(world, def->bodyIdA);
    int32_t bodyB = BodySlot(world, def->bodyIdB);
    if (bodyA < 0 || bodyB < 0 || bodyA == bodyB)
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    int32_t index = AllocateJoint(world);
    if (index < 0)
    {
        return m2_nullJointId;
    }
    m2JointId jointId = FinishJoint(world, worldId, index, 3, bodyA, bodyB, def->localAnchorA,
                                    def->localAnchorB, 0.0f, def->hertz, def->dampingRatio);
    world->jointRefAngle[index] =
        RelativeJointAngle(world->transforms[bodyA].q, world->transforms[bodyB].q);
    if (world->journalActive != 0)
    {
        struct
        {
            m2WeldJointDef def;
            m2JointId expected;
        } record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateWeldJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

m2WheelJointDef m2DefaultWheelJointDef(void)
{
    m2WheelJointDef def;
    memset(&def, 0, sizeof(def));
    def.localAxisA = (m2Vec2){0.0f, 1.0f};
    def.enableSpring = true;
    def.hertz = 2.0f;
    def.dampingRatio = 0.7f;
    def.internalValue = M2_WHJOINT_COOKIE;
    return def;
}

m2JointId m2CreateWheelJoint(m2WorldId worldId, const m2WheelJointDef* def)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_WHJOINT_COOKIE)
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    int32_t bodyA = BodySlot(world, def->bodyIdA);
    int32_t bodyB = BodySlot(world, def->bodyIdB);
    if (bodyA < 0 || bodyB < 0 || bodyA == bodyB)
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    float axisLength =
        sqrtf(def->localAxisA.x * def->localAxisA.x + def->localAxisA.y * def->localAxisA.y);
    if (!(axisLength > 1.19209290e-7f))
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    int32_t index = AllocateJoint(world);
    if (index < 0)
    {
        return m2_nullJointId;
    }
    m2JointId jointId = FinishJoint(world, worldId, index, 4, bodyA, bodyB, def->localAnchorA,
                                    def->localAnchorB, 0.0f, def->hertz, def->dampingRatio);
    world->jointFlags[index] =
        (def->enableMotor ? 1u : 0u) | (def->enableLimit ? 2u : 0u) | (def->enableSpring ? 4u : 0u);
    world->jointMotorSpeed[index] = def->motorSpeed;
    world->jointMaxMotor[index] = def->maxMotorTorque;
    world->jointLower[index] = def->lowerTranslation;
    world->jointUpper[index] = def->upperTranslation;
    world->jointLocalAxisA[index] =
        (m2Vec2){def->localAxisA.x / axisLength, def->localAxisA.y / axisLength};
    if (world->journalActive != 0)
    {
        struct
        {
            m2WheelJointDef def;
            m2JointId expected;
        } record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateWheelJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

// One journaled channel for every runtime joint parameter: replay
// re-drives the same setters through the same records.
void m2SetJointParamInternal(m2World* world, m2JointId jointId, uint8_t param, float value)
{
    int32_t index = jointId.index1 - 1;
    if (index < 0 || index >= world->jointCapacity || world->jointAlive[index] == 0 ||
        world->jointGenerations[index] != jointId.generation)
    {
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2JointId joint;
            float value;
            uint8_t param;
        } record;
        memset(&record, 0, sizeof(record));
        record.joint = jointId;
        record.value = value;
        record.param = param;
        m2JournalRecord(world, m2_opSetJointParam, &record, (int32_t)sizeof(record));
    }
    switch (param)
    {
    case m2_jointParamMotorSpeed:
        world->jointMotorSpeed[index] = value;
        break;
    case m2_jointParamMaxMotor:
        world->jointMaxMotor[index] = value;
        break;
    case m2_jointParamEnableMotor:
        world->jointFlags[index] =
            value != 0.0f ? (world->jointFlags[index] | 1u) : (world->jointFlags[index] & ~1u);
        break;
    case m2_jointParamEnableLimit:
        world->jointFlags[index] =
            value != 0.0f ? (world->jointFlags[index] | 2u) : (world->jointFlags[index] & ~2u);
        break;
    case m2_jointParamLower:
        world->jointLower[index] = value;
        break;
    default:
        world->jointUpper[index] = value;
        break;
    }
    // Any parameter change wakes both ends.
    int32_t bodyA = world->jointBodyA[index];
    int32_t bodyB = world->jointBodyB[index];
    if (world->types[bodyA] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[bodyA] = 0;
        world->sleepTimes[bodyA] = 0.0f;
    }
    if (world->types[bodyB] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[bodyB] = 0;
        world->sleepTimes[bodyB] = 0.0f;
    }
}

void m2Joint_SetMotorSpeed(m2JointId jointId, float speed)
{
    m2World* world = WorldFromIndex(jointId.world0);
    if (world != NULL)
    {
        m2SetJointParamInternal(world, jointId, m2_jointParamMotorSpeed, speed);
    }
}

void m2Joint_SetMaxMotor(m2JointId jointId, float maxTorqueOrForce)
{
    m2World* world = WorldFromIndex(jointId.world0);
    if (world != NULL)
    {
        m2SetJointParamInternal(world, jointId, m2_jointParamMaxMotor, maxTorqueOrForce);
    }
}

void m2Joint_EnableMotor(m2JointId jointId, bool enable)
{
    m2World* world = WorldFromIndex(jointId.world0);
    if (world != NULL)
    {
        m2SetJointParamInternal(world, jointId, m2_jointParamEnableMotor, enable ? 1.0f : 0.0f);
    }
}

void m2Joint_EnableLimit(m2JointId jointId, bool enable)
{
    m2World* world = WorldFromIndex(jointId.world0);
    if (world != NULL)
    {
        m2SetJointParamInternal(world, jointId, m2_jointParamEnableLimit, enable ? 1.0f : 0.0f);
    }
}

void m2Joint_SetLimits(m2JointId jointId, float lower, float upper)
{
    m2World* world = WorldFromIndex(jointId.world0);
    if (world != NULL)
    {
        m2SetJointParamInternal(world, jointId, m2_jointParamLower, lower);
        m2SetJointParamInternal(world, jointId, m2_jointParamUpper, upper);
    }
}

void m2DestroyJoint(m2JointId jointId)
{
    m2World* world = WorldFromIndex(jointId.world0);
    if (world == NULL)
    {
        return;
    }
    int32_t index = jointId.index1 - 1;
    if (index < 0 || index >= world->jointCapacity || world->jointAlive[index] == 0 ||
        world->jointGenerations[index] != jointId.generation)
    {
        return;
    }
    m2JournalRecord(world, m2_opDestroyJoint, &jointId, (int32_t)sizeof(jointId));
    // Both ends wake: a constraint vanished.
    int32_t bodyA = world->jointBodyA[index];
    int32_t bodyB = world->jointBodyB[index];
    if (world->types[bodyA] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[bodyA] = 0;
        world->sleepTimes[bodyA] = 0.0f;
    }
    if (world->types[bodyB] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[bodyB] = 0;
        world->sleepTimes[bodyB] = 0.0f;
    }
    world->jointAlive[index] = 0;
    if (world->jointGenerations[index] == UINT16_MAX)
    {
        world->jointRetiredCount += 1;
        return;
    }
    world->jointGenerations[index] += 1;
    world->jointFreeQueue[world->jointFreeTail] = index;
    world->jointFreeTail = (world->jointFreeTail + 1) % world->jointCapacity;
    world->jointFreeCount += 1;
}

bool m2Joint_IsValid(m2JointId jointId)
{
    m2World* world = WorldFromIndex(jointId.world0);
    if (world == NULL)
    {
        return false;
    }
    int32_t index = jointId.index1 - 1;
    return index >= 0 && index < world->jointCapacity && world->jointAlive[index] != 0 &&
           world->jointGenerations[index] == jointId.generation;
}
