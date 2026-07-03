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

#include <stdlib.h>
#include <string.h>

#define M2_MAX_WORLDS       16
#define M2_WORLD_COOKIE     (M2_COOKIE ^ ((int32_t)sizeof(m2WorldDef) << 8) ^ 1)
#define M2_BODY_COOKIE      (M2_COOKIE ^ ((int32_t)sizeof(m2BodyDef) << 8) ^ 2)
#define M2_SHAPE_COOKIE     (M2_COOKIE ^ ((int32_t)sizeof(m2ShapeDef) << 8) ^ 3)
#define M2_SNAPSHOT_MAGIC   0x4D32534Eu // 'M2SN'
#define M2_SNAPSHOT_VERSION 3u

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
        return;
    }

    float invMass = 1.0f / mass;
    center.x *= invMass;
    center.y *= invMass;
    float inertiaCenter = inertiaOrigin - mass * (center.x * center.x + center.y * center.y);
    world->invMass[bodyIndex] = invMass;
    world->invInertia[bodyIndex] = inertiaCenter > 0.0f ? 1.0f / inertiaCenter : 0.0f;
}

// --- Defs & world lifecycle ----------------------------------------------------

m2WorldDef m2DefaultWorldDef(void)
{
    m2WorldDef def;
    memset(&def, 0, sizeof(def));
    def.gravity = (m2Vec2){0.0f, -10.0f};
    def.bodyCapacity = 1024;
    def.shapeCapacity = 2048;
    def.internalValue = M2_WORLD_COOKIE;
    return def;
}

m2WorldId m2CreateWorld(const m2WorldDef* def)
{
    if (def == NULL || def->internalValue != M2_WORLD_COOKIE || def->bodyCapacity < 1 ||
        def->shapeCapacity < 1)
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

    m2World* world = calloc(1, sizeof(m2World));
    if (world == NULL)
    {
        return m2_nullWorldId;
    }

    int32_t cap = def->bodyCapacity;
    int32_t shapeCap = def->shapeCapacity;
    world->gravity = def->gravity;
    world->bodyCapacity = cap;
    world->shapeCapacity = shapeCap;
    world->treeNodeCapacity = 2 * shapeCap;
    world->pairCapacity = 8 * shapeCap;

    bool ok = true;
#define M2_ALLOC(field, count, type)                                                               \
    do                                                                                             \
    {                                                                                              \
        world->field = calloc((size_t)(count), sizeof(type));                                      \
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
    M2_ALLOC(generations, cap, uint16_t);
    M2_ALLOC(freeQueue, cap, int32_t);
    M2_ALLOC(shapeGeometry, shapeCap, m2ShapeGeometry);
    M2_ALLOC(shapeDensity, shapeCap, float);
    M2_ALLOC(shapeUserData, shapeCap, uint64_t);
    M2_ALLOC(shapeBody, shapeCap, int32_t);
    M2_ALLOC(shapeNext, shapeCap, int32_t);
    M2_ALLOC(shapeAlive, shapeCap, uint8_t);
    M2_ALLOC(shapeGenerations, shapeCap, uint16_t);
    M2_ALLOC(shapeFreeQueue, shapeCap, int32_t);
    M2_ALLOC(proxyIds, shapeCap, int32_t);
    M2_ALLOC(inMoved, shapeCap, uint8_t);
    M2_ALLOC(moved, shapeCap, int32_t);
    M2_ALLOC(pairKeys, world->pairCapacity, uint64_t);
    M2_ALLOC(pairScratch, world->pairCapacity, uint64_t);
#undef M2_ALLOC
    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        world->treeNodes[t] = calloc((size_t)world->treeNodeCapacity, sizeof(m2TreeNode));
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
    world->freeHead = 0;
    world->freeTail = 0;
    world->freeCount = cap;
    world->shapeFreeHead = 0;
    world->shapeFreeTail = 0;
    world->shapeFreeCount = shapeCap;

    s_worldGenerations[slot] += 1;
    world->worldGeneration = s_worldGenerations[slot];
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
    free(world->transforms);
    free(world->linearVelocities);
    free(world->angularVelocities);
    free(world->gravityScales);
    free(world->userData);
    free(world->types);
    free(world->alive);
    free(world->bodyShapeHead);
    free(world->invMass);
    free(world->invInertia);
    free(world->generations);
    free(world->freeQueue);
    free(world->shapeGeometry);
    free(world->shapeDensity);
    free(world->shapeUserData);
    free(world->shapeBody);
    free(world->shapeNext);
    free(world->shapeAlive);
    free(world->shapeGenerations);
    free(world->shapeFreeQueue);
    free(world->proxyIds);
    free(world->inMoved);
    free(world->moved);
    free(world->pairKeys);
    free(world->pairScratch);
    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        free(world->treeNodes[t]);
    }
    free(world);
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

    float h = dt / (float)substepCount;
    for (int32_t sub = 0; sub < substepCount; ++sub)
    {
        // Fixed body-index order everywhere: the ordering law (ADR-0010).
        for (int32_t i = 0; i < world->maxBodyIndex; ++i)
        {
            if (world->alive[i] == 0 || world->types[i] != (uint8_t)m2_dynamicBody)
            {
                continue;
            }
            world->linearVelocities[i].x += world->gravity.x * world->gravityScales[i] * h;
            world->linearVelocities[i].y += world->gravity.y * world->gravityScales[i] * h;
        }
        for (int32_t i = 0; i < world->maxBodyIndex; ++i)
        {
            if (world->alive[i] == 0 || world->types[i] == (uint8_t)m2_staticBody)
            {
                continue;
            }
            world->transforms[i].p.x += (double)world->linearVelocities[i].x * (double)h;
            world->transforms[i].p.y += (double)world->linearVelocities[i].y * (double)h;
            float dAngle = world->angularVelocities[i] * h;
            world->transforms[i].q = m2MulRot(world->transforms[i].q, m2MakeRot(dAngle));
        }
    }

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
    UpdatePairs(world);

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

static int32_t BlockBytes(const m2World* world)
{
    size_t cap = (size_t)world->bodyCapacity;
    size_t shapeCap = (size_t)world->shapeCapacity;
    size_t bytes = 0;
    bytes += cap * (sizeof(m2Transform) + sizeof(m2Vec2) + 4 * sizeof(float) + sizeof(uint64_t) +
                    2 * sizeof(uint8_t) + sizeof(uint16_t) + 2 * sizeof(int32_t));
    bytes += shapeCap * (sizeof(m2ShapeGeometry) + sizeof(float) + sizeof(uint64_t) +
                         5 * sizeof(int32_t) + 2 * sizeof(uint8_t) + sizeof(uint16_t));
    bytes += M2_TREE_COUNT * sizeof(m2DynamicTree);
    bytes += (size_t)M2_TREE_COUNT * (size_t)world->treeNodeCapacity * sizeof(m2TreeNode);
    bytes += (size_t)world->pairCapacity * sizeof(uint64_t);
    return (int32_t)bytes;
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
        else                                                                                       \
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
    M2_BLOCK(world->userData, cap * sizeof(uint64_t));
    M2_BLOCK(world->types, cap * sizeof(uint8_t));
    M2_BLOCK(world->alive, cap * sizeof(uint8_t));
    M2_BLOCK(world->bodyShapeHead, cap * sizeof(int32_t));
    M2_BLOCK(world->generations, cap * sizeof(uint16_t));
    M2_BLOCK(world->freeQueue, cap * sizeof(int32_t));
    M2_BLOCK(world->shapeGeometry, shapeCap * sizeof(m2ShapeGeometry));
    M2_BLOCK(world->shapeDensity, shapeCap * sizeof(float));
    M2_BLOCK(world->shapeUserData, shapeCap * sizeof(uint64_t));
    M2_BLOCK(world->shapeBody, shapeCap * sizeof(int32_t));
    M2_BLOCK(world->shapeNext, shapeCap * sizeof(int32_t));
    M2_BLOCK(world->shapeAlive, shapeCap * sizeof(uint8_t));
    M2_BLOCK(world->shapeGenerations, shapeCap * sizeof(uint16_t));
    M2_BLOCK(world->shapeFreeQueue, shapeCap * sizeof(int32_t));
    M2_BLOCK(world->proxyIds, shapeCap * sizeof(int32_t));
    M2_BLOCK(world->inMoved, shapeCap * sizeof(uint8_t));
    M2_BLOCK(world->moved, shapeCap * sizeof(int32_t));
    M2_BLOCK(world->trees, M2_TREE_COUNT * sizeof(m2DynamicTree));
    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        M2_BLOCK(world->treeNodes[t], (size_t)world->treeNodeCapacity * sizeof(m2TreeNode));
    }
    M2_BLOCK(world->pairKeys, (size_t)world->pairCapacity * sizeof(uint64_t));
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
        h = m2Hash64(h, &world->types[i], (int32_t)sizeof(uint8_t));
    }
    h = m2Hash64(h, world->pairKeys, world->pairCount * (int32_t)sizeof(uint64_t));
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
    world->invInertia[index] = 0.0f;
    if (index + 1 > world->maxBodyIndex)
    {
        world->maxBodyIndex = index + 1;
    }

    m2BodyId id = {index + 1, worldId.index1, world->generations[index]};
    return id;
}

static void DestroyShapeInternal(m2World* world, int32_t shapeIndex)
{
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
}

// --- Shapes ---------------------------------------------------------------------

m2ShapeDef m2DefaultShapeDef(void)
{
    m2ShapeDef def;
    memset(&def, 0, sizeof(def));
    def.density = 1.0f;
    def.internalValue = M2_SHAPE_COOKIE;
    return def;
}

static m2ShapeId CreateShape(m2BodyId bodyId, const m2ShapeDef* def,
                             const m2ShapeGeometry* geometry)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t bodyIndex = world != NULL ? BodySlot(world, bodyId) : -1;
    if (bodyIndex < 0 || def == NULL || def->internalValue != M2_SHAPE_COOKIE ||
        !(def->density >= 0.0f))
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
