// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// World core: body storage, gravity integration, broadphase update, and
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
#define M2_SNAPSHOT_MAGIC   0x4D32534Eu // 'M2SN'
#define M2_SNAPSHOT_VERSION 2u

// Fat margin in meters (topic-02 §3; harness-tuned later, F-T2-1).
#define M2_AABB_MARGIN 0.1

// Placeholder body extent until shapes land (topic-03): every body is a
// half-extent-0.5 box around its position, rotation ignored. Replaced by
// per-shape AABBs in the next slice; clearly throwaway, deliberately so.
#define M2_PLACEHOLDER_EXTENT 0.5

// The world registry is the single sanctioned process-global (ADR-0011
// amendment): it maps id.world0 to a world and is not simulation state -
// snapshots and rollback never touch it.
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

static m2World* GetBodyWorld(m2BodyId id)
{
    m2WorldId worldId = {id.world0, 0};
    if (id.world0 < 1 || id.world0 > M2_MAX_WORLDS)
    {
        return NULL;
    }
    worldId.generation = s_worldGenerations[id.world0 - 1];
    return GetWorld(worldId);
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

// --- Broadphase helpers ------------------------------------------------------

static m2AABB TightAABB(const m2World* world, int32_t index)
{
    m2Pos2 p = world->transforms[index].p;
    m2AABB aabb;
    aabb.lowerBound = (m2Pos2){p.x - M2_PLACEHOLDER_EXTENT, p.y - M2_PLACEHOLDER_EXTENT};
    aabb.upperBound = (m2Pos2){p.x + M2_PLACEHOLDER_EXTENT, p.y + M2_PLACEHOLDER_EXTENT};
    return aabb;
}

static m2AABB Fatten(m2AABB aabb)
{
    aabb.lowerBound.x -= M2_AABB_MARGIN;
    aabb.lowerBound.y -= M2_AABB_MARGIN;
    aabb.upperBound.x += M2_AABB_MARGIN;
    aabb.upperBound.y += M2_AABB_MARGIN;
    return aabb;
}

// ALL moved proxies enter the moved set - dynamic, kinematic, and static
// alike (topic-02 §4.2: this is the mechanism that lets a moved kinematic
// or static body wake and pair against everything it now touches).
static void PushMoved(m2World* world, int32_t index)
{
    if (world->inMoved[index] != 0)
    {
        return;
    }
    world->inMoved[index] = 1;
    world->moved[world->movedCount] = index;
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

// Re-derive pairs touched by the moved set, then batch-merge with the
// untouched remainder (topic-02 §4.2, RT1-PERF-3: never per-item inserts).
static void UpdatePairs(m2World* world)
{
    if (world->movedCount == 0)
    {
        return;
    }

    // Canonical consumption order: ascending body index, not insertion order.
    qsort(world->moved, (size_t)world->movedCount, sizeof(int32_t), CompareI32);

    int32_t collected = 0;
    int32_t queryResults[256];
    for (int32_t m = 0; m < world->movedCount; ++m)
    {
        int32_t index = world->moved[m];
        if (world->alive[index] == 0 || world->proxyIds[index] == M2_NULL_NODE)
        {
            continue;
        }
        m2AABB fat = world->treeNodes[world->types[index]][world->proxyIds[index]].aabb;

        // Dynamic bodies pair against everything; non-dynamic movers pair
        // against dynamics only (a static-static overlap is not a pair).
        int32_t firstTree = world->types[index] == (uint8_t)m2_dynamicBody ? 0 : m2_dynamicBody;
        int32_t lastTree =
            world->types[index] == (uint8_t)m2_dynamicBody ? M2_TREE_COUNT - 1 : m2_dynamicBody;
        for (int32_t t = firstTree; t <= lastTree; ++t)
        {
            int32_t hits =
                m2Tree_Query(&world->trees[t], world->treeNodes[t], fat, queryResults, 256);
            M2_ASSERT(hits <= 256);
            hits = hits <= 256 ? hits : 256;
            for (int32_t h = 0; h < hits; ++h)
            {
                int32_t other = queryResults[h];
                if (other == index || world->alive[other] == 0)
                {
                    continue;
                }
                if (world->types[index] != (uint8_t)m2_dynamicBody &&
                    world->types[other] != (uint8_t)m2_dynamicBody)
                {
                    continue;
                }
                if (collected < world->pairCapacity)
                {
                    world->pairScratch[collected] = PairKey(index, other);
                }
                collected += 1;
            }
        }
    }
    M2_ASSERT(collected <= world->pairCapacity);
    collected = collected <= world->pairCapacity ? collected : world->pairCapacity;

    qsort(world->pairScratch, (size_t)collected, sizeof(uint64_t), CompareU64);

    // Keep pairs with no moved endpoint; every moved-endpoint pair is
    // re-derived above if it still overlaps.
    int32_t kept = 0;
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        int32_t a = (int32_t)(world->pairKeys[i] >> 32);
        int32_t b = (int32_t)(world->pairKeys[i] & 0xFFFFFFFFu);
        if (world->inMoved[a] == 0 && world->inMoved[b] == 0 && world->alive[a] != 0 &&
            world->alive[b] != 0)
        {
            world->pairKeys[kept] = world->pairKeys[i];
            kept += 1;
        }
    }

    // Merge (kept is sorted; scratch is sorted): classic two-way merge with
    // dedup into the tail scratch space, then copy back.
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
        world->pairScratch[world->pairCapacity - 1 - outCount] = next; // build in tail
        previous = next;
        hasPrevious = true;
        outCount += 1;
    }
    for (int32_t k = 0; k < outCount; ++k)
    {
        world->pairKeys[k] = world->pairScratch[world->pairCapacity - 1 - (outCount - 1 - k)];
    }
    world->pairCount = outCount;

    // Consume the moved set.
    for (int32_t m = 0; m < world->movedCount; ++m)
    {
        world->inMoved[world->moved[m]] = 0;
    }
    world->movedCount = 0;
}

// --- Defs & world lifecycle --------------------------------------------------

m2WorldDef m2DefaultWorldDef(void)
{
    m2WorldDef def;
    memset(&def, 0, sizeof(def));
    def.gravity = (m2Vec2){0.0f, -10.0f};
    def.bodyCapacity = 1024;
    def.internalValue = M2_WORLD_COOKIE;
    return def;
}

m2WorldId m2CreateWorld(const m2WorldDef* def)
{
    if (def == NULL || def->internalValue != M2_WORLD_COOKIE || def->bodyCapacity < 1)
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
    world->gravity = def->gravity;
    world->bodyCapacity = cap;
    world->treeNodeCapacity = 2 * cap;
    world->pairCapacity = 8 * cap;
    world->transforms = calloc((size_t)cap, sizeof(m2Transform));
    world->linearVelocities = calloc((size_t)cap, sizeof(m2Vec2));
    world->angularVelocities = calloc((size_t)cap, sizeof(float));
    world->gravityScales = calloc((size_t)cap, sizeof(float));
    world->userData = calloc((size_t)cap, sizeof(uint64_t));
    world->types = calloc((size_t)cap, sizeof(uint8_t));
    world->alive = calloc((size_t)cap, sizeof(uint8_t));
    world->generations = calloc((size_t)cap, sizeof(uint16_t));
    world->freeQueue = calloc((size_t)cap, sizeof(int32_t));
    world->proxyIds = calloc((size_t)cap, sizeof(int32_t));
    world->inMoved = calloc((size_t)cap, sizeof(uint8_t));
    world->moved = calloc((size_t)cap, sizeof(int32_t));
    world->pairKeys = calloc((size_t)world->pairCapacity, sizeof(uint64_t));
    world->pairScratch = calloc((size_t)world->pairCapacity, sizeof(uint64_t));
    bool treesOk = true;
    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        world->treeNodes[t] = calloc((size_t)world->treeNodeCapacity, sizeof(m2TreeNode));
        treesOk = treesOk && world->treeNodes[t] != NULL;
    }
    if (world->transforms == NULL || world->linearVelocities == NULL ||
        world->angularVelocities == NULL || world->gravityScales == NULL ||
        world->userData == NULL || world->types == NULL || world->alive == NULL ||
        world->generations == NULL || world->freeQueue == NULL || world->proxyIds == NULL ||
        world->inMoved == NULL || world->moved == NULL || world->pairKeys == NULL ||
        world->pairScratch == NULL || !treesOk)
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
        world->proxyIds[i] = M2_NULL_NODE;
    }
    world->freeHead = 0;
    world->freeTail = 0;
    world->freeCount = cap;

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
        // Allow the failed-allocation path in m2CreateWorld to reuse this.
        world = s_worlds[worldId.index1 - 1];
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
    free(world->generations);
    free(world->freeQueue);
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
            // Hybrid precision: f32 velocity advances an f64 position; the
            // f64 -> f32 crossing happens only in future solver stages.
            world->transforms[i].p.x += (double)world->linearVelocities[i].x * (double)h;
            world->transforms[i].p.y += (double)world->linearVelocities[i].y * (double)h;
            float dAngle = world->angularVelocities[i] * h;
            world->transforms[i].q = m2MulRot(world->transforms[i].q, m2MakeRot(dAngle));
        }
    }

    // Broadphase update: single-threaded, fixed body-index order. A proxy
    // is re-inserted only when its tight AABB escapes its fat one.
    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        if (world->alive[i] == 0 || world->proxyIds[i] == M2_NULL_NODE ||
            world->types[i] == (uint8_t)m2_staticBody)
        {
            continue;
        }
        m2AABB tight = TightAABB(world, i);
        int32_t tree = world->types[i];
        if (!m2AABB_Contains(world->treeNodes[tree][world->proxyIds[i]].aabb, tight))
        {
            m2Tree_Move(&world->trees[tree], world->treeNodes[tree], world->proxyIds[i],
                        Fatten(tight));
            PushMoved(world, i);
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

// --- Snapshot ---------------------------------------------------------------

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
} m2SnapshotHeader;

_Static_assert(sizeof(m2SnapshotHeader) == 56, "snapshot header must be padding-free");

static int32_t BlockBytes(const m2World* world)
{
    int32_t cap = world->bodyCapacity;
    int32_t bodyBlocks =
        (int32_t)(cap * sizeof(m2Transform) + cap * sizeof(m2Vec2) + cap * sizeof(float) +
                  cap * sizeof(float) + cap * sizeof(uint64_t) + cap * sizeof(uint8_t) +
                  cap * sizeof(uint8_t) + cap * sizeof(uint16_t) + cap * sizeof(int32_t));
    int32_t broadphaseBlocks =
        (int32_t)(M2_TREE_COUNT * sizeof(m2DynamicTree) +
                  M2_TREE_COUNT * (size_t)world->treeNodeCapacity * sizeof(m2TreeNode) +
                  cap * sizeof(int32_t) + cap * sizeof(uint8_t) + cap * sizeof(int32_t) +
                  (size_t)world->pairCapacity * sizeof(uint64_t));
    return bodyBlocks + broadphaseBlocks;
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

    uint8_t* out = buffer;
    int32_t cursor = 0;
    int32_t cap = world->bodyCapacity;
#define M2_COPY(src, bytes)                                                                        \
    do                                                                                             \
    {                                                                                              \
        memcpy(out + cursor, src, (size_t)(bytes));                                                \
        cursor += (int32_t)(bytes);                                                                \
    } while (0)
    M2_COPY(&header, sizeof(header));
    M2_COPY(world->transforms, cap * sizeof(m2Transform));
    M2_COPY(world->linearVelocities, cap * sizeof(m2Vec2));
    M2_COPY(world->angularVelocities, cap * sizeof(float));
    M2_COPY(world->gravityScales, cap * sizeof(float));
    M2_COPY(world->userData, cap * sizeof(uint64_t));
    M2_COPY(world->types, cap * sizeof(uint8_t));
    M2_COPY(world->alive, cap * sizeof(uint8_t));
    M2_COPY(world->generations, cap * sizeof(uint16_t));
    M2_COPY(world->freeQueue, cap * sizeof(int32_t));
    M2_COPY(world->trees, M2_TREE_COUNT * sizeof(m2DynamicTree));
    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        M2_COPY(world->treeNodes[t], (size_t)world->treeNodeCapacity * sizeof(m2TreeNode));
    }
    M2_COPY(world->proxyIds, cap * sizeof(int32_t));
    M2_COPY(world->inMoved, cap * sizeof(uint8_t));
    M2_COPY(world->moved, cap * sizeof(int32_t));
    M2_COPY(world->pairKeys, (size_t)world->pairCapacity * sizeof(uint64_t));
#undef M2_COPY
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

    const uint8_t* in = buffer;
    int32_t cursor = (int32_t)sizeof(header);
    int32_t cap = world->bodyCapacity;
#define M2_READ(dst, bytes)                                                                        \
    do                                                                                             \
    {                                                                                              \
        memcpy(dst, in + cursor, (size_t)(bytes));                                                 \
        cursor += (int32_t)(bytes);                                                                \
    } while (0)
    M2_READ(world->transforms, cap * sizeof(m2Transform));
    M2_READ(world->linearVelocities, cap * sizeof(m2Vec2));
    M2_READ(world->angularVelocities, cap * sizeof(float));
    M2_READ(world->gravityScales, cap * sizeof(float));
    M2_READ(world->userData, cap * sizeof(uint64_t));
    M2_READ(world->types, cap * sizeof(uint8_t));
    M2_READ(world->alive, cap * sizeof(uint8_t));
    M2_READ(world->generations, cap * sizeof(uint16_t));
    M2_READ(world->freeQueue, cap * sizeof(int32_t));
    M2_READ(world->trees, M2_TREE_COUNT * sizeof(m2DynamicTree));
    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        M2_READ(world->treeNodes[t], (size_t)world->treeNodeCapacity * sizeof(m2TreeNode));
    }
    M2_READ(world->proxyIds, cap * sizeof(int32_t));
    M2_READ(world->inMoved, cap * sizeof(uint8_t));
    M2_READ(world->moved, cap * sizeof(int32_t));
    M2_READ(world->pairKeys, (size_t)world->pairCapacity * sizeof(uint64_t));
#undef M2_READ
    M2_ASSERT(cursor == size);
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
        h = m2Hash64(h, &world->types[i], (int32_t)sizeof(uint8_t));
    }
    // Pair set is sim-visible state (future contacts feed on it): hash it.
    h = m2Hash64(h, world->pairKeys, world->pairCount * (int32_t)sizeof(uint64_t));
    return h;
}

// --- Bodies -----------------------------------------------------------------

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
        return m2_nullBodyId; // capacity exhausted (or all slots retired)
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
    if (index + 1 > world->maxBodyIndex)
    {
        world->maxBodyIndex = index + 1;
    }

    // Eager proxy insertion (topic-02 §4.1): the body is visible to the
    // broadphase the moment it exists, and pairs against everything it
    // overlaps on the next pair update.
    int32_t tree = (int32_t)def->type;
    world->proxyIds[index] = m2Tree_Insert(&world->trees[tree], world->treeNodes[tree],
                                           Fatten(TightAABB(world, index)), index);
    if (world->proxyIds[index] == M2_NULL_NODE)
    {
        // Node pool exhausted: undo the body, fail as a capacity error.
        world->alive[index] = 0;
        world->freeHead = (world->freeHead + world->bodyCapacity - 1) % world->bodyCapacity;
        world->freeQueue[world->freeHead] = index;
        world->freeCount += 1;
        return m2_nullBodyId;
    }
    PushMoved(world, index);

    m2BodyId id = {index + 1, worldId.index1, world->generations[index]};
    return id;
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

    if (world->proxyIds[index] != M2_NULL_NODE)
    {
        int32_t tree = world->types[index];
        m2Tree_Remove(&world->trees[tree], world->treeNodes[tree], world->proxyIds[index]);
        world->proxyIds[index] = M2_NULL_NODE;
    }
    // Drop pairs involving this body immediately (event bookending will
    // hang end-touch emission on exactly this path, registry M19).
    int32_t kept = 0;
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        int32_t a = (int32_t)(world->pairKeys[i] >> 32);
        int32_t b = (int32_t)(world->pairKeys[i] & 0xFFFFFFFFu);
        if (a != index && b != index)
        {
            world->pairKeys[kept] = world->pairKeys[i];
            kept += 1;
        }
    }
    world->pairCount = kept;

    world->alive[index] = 0;
    if (world->generations[index] == UINT16_MAX)
    {
        // Retire: staleness detection stays absolute (topic-10). The slot
        // never re-enters the free queue; compaction is out of v1.
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
