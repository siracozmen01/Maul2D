// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Slice 0: world/body storage, gravity integration, snapshot/restore/hash.
// Every array here is POD and lives in the snapshot; the rollback gate
// byte-compares all of it. Allocation goes through malloc for now - the
// world-def allocator hooks are a recorded pending item, due before any
// public release (topic-10).

#include "maul2d/world.h"
#include "maul2d/body.h"

#include "maul2d/base.h"

#include <stdlib.h>
#include <string.h>

#define M2_MAX_WORLDS       16
#define M2_WORLD_COOKIE     (M2_COOKIE ^ ((int32_t)sizeof(m2WorldDef) << 8) ^ 1)
#define M2_BODY_COOKIE      (M2_COOKIE ^ ((int32_t)sizeof(m2BodyDef) << 8) ^ 2)
#define M2_SNAPSHOT_MAGIC   0x4D32534Eu // 'M2SN'
#define M2_SNAPSHOT_VERSION 1u

typedef struct m2World
{
    // World-global mutable block (snapshot state).
    m2Vec2 gravity;
    uint64_t stepCount;

    // Body storage: parallel POD arrays, fixed capacity (slice 0).
    int32_t bodyCapacity;
    int32_t maxBodyIndex; // high-water mark of used slots
    m2Transform* transforms;
    m2Vec2* linearVelocities;
    float* angularVelocities;
    float* gravityScales;
    uint64_t* userData;
    uint8_t* types;
    uint8_t* alive;

    // Id pool: FIFO free queue + generations. Saturated slots retire.
    uint16_t* generations;
    int32_t* freeQueue;
    int32_t freeHead;
    int32_t freeTail;
    int32_t freeCount;
    int32_t retiredCount;

    uint16_t worldGeneration;
} m2World;

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
    world->transforms = calloc((size_t)cap, sizeof(m2Transform));
    world->linearVelocities = calloc((size_t)cap, sizeof(m2Vec2));
    world->angularVelocities = calloc((size_t)cap, sizeof(float));
    world->gravityScales = calloc((size_t)cap, sizeof(float));
    world->userData = calloc((size_t)cap, sizeof(uint64_t));
    world->types = calloc((size_t)cap, sizeof(uint8_t));
    world->alive = calloc((size_t)cap, sizeof(uint8_t));
    world->generations = calloc((size_t)cap, sizeof(uint16_t));
    world->freeQueue = calloc((size_t)cap, sizeof(int32_t));
    if (world->transforms == NULL || world->linearVelocities == NULL ||
        world->angularVelocities == NULL || world->gravityScales == NULL ||
        world->userData == NULL || world->types == NULL || world->alive == NULL ||
        world->generations == NULL || world->freeQueue == NULL)
    {
        m2WorldId failed = {(uint16_t)(slot + 1), s_worldGenerations[slot]};
        s_worlds[slot] = world;
        m2DestroyWorld(failed);
        return m2_nullWorldId;
    }

    // FIFO free queue starts holding every slot in index order -
    // deterministic id minting from the first create (topic-09 contract).
    for (int32_t i = 0; i < cap; ++i)
    {
        world->freeQueue[i] = i;
    }
    world->freeHead = 0;
    world->freeTail = 0; // tail == head with full count means "all free"
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
} m2SnapshotHeader;

_Static_assert(sizeof(m2SnapshotHeader) == 48, "snapshot header must be padding-free");

static int32_t BlockBytes(const m2World* world)
{
    int32_t cap = world->bodyCapacity;
    return (int32_t)(cap * sizeof(m2Transform) + cap * sizeof(m2Vec2) + cap * sizeof(float) +
                     cap * sizeof(float) + cap * sizeof(uint64_t) + cap * sizeof(uint8_t) +
                     cap * sizeof(uint8_t) + cap * sizeof(uint16_t) + cap * sizeof(int32_t));
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

#define M2_COPY_BLOCK(dst, src, bytes)                                                             \
    do                                                                                             \
    {                                                                                              \
        memcpy(dst, src, (size_t)(bytes));                                                         \
        cursor += (bytes);                                                                         \
    } while (0)

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

    uint8_t* out = buffer;
    int32_t cursor = 0;
    int32_t cap = world->bodyCapacity;
    M2_COPY_BLOCK(out + cursor, &header, (int32_t)sizeof(header));
    M2_COPY_BLOCK(out + cursor, world->transforms, cap * (int32_t)sizeof(m2Transform));
    M2_COPY_BLOCK(out + cursor, world->linearVelocities, cap * (int32_t)sizeof(m2Vec2));
    M2_COPY_BLOCK(out + cursor, world->angularVelocities, cap * (int32_t)sizeof(float));
    M2_COPY_BLOCK(out + cursor, world->gravityScales, cap * (int32_t)sizeof(float));
    M2_COPY_BLOCK(out + cursor, world->userData, cap * (int32_t)sizeof(uint64_t));
    M2_COPY_BLOCK(out + cursor, world->types, cap * (int32_t)sizeof(uint8_t));
    M2_COPY_BLOCK(out + cursor, world->alive, cap * (int32_t)sizeof(uint8_t));
    M2_COPY_BLOCK(out + cursor, world->generations, cap * (int32_t)sizeof(uint16_t));
    M2_COPY_BLOCK(out + cursor, world->freeQueue, cap * (int32_t)sizeof(int32_t));
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

    const uint8_t* in = buffer;
    int32_t cursor = (int32_t)sizeof(header);
    int32_t cap = world->bodyCapacity;
#define M2_READ_BLOCK(dst, bytes)                                                                  \
    do                                                                                             \
    {                                                                                              \
        memcpy(dst, in + cursor, (size_t)(bytes));                                                 \
        cursor += (bytes);                                                                         \
    } while (0)
    M2_READ_BLOCK(world->transforms, cap * (int32_t)sizeof(m2Transform));
    M2_READ_BLOCK(world->linearVelocities, cap * (int32_t)sizeof(m2Vec2));
    M2_READ_BLOCK(world->angularVelocities, cap * (int32_t)sizeof(float));
    M2_READ_BLOCK(world->gravityScales, cap * (int32_t)sizeof(float));
    M2_READ_BLOCK(world->userData, cap * (int32_t)sizeof(uint64_t));
    M2_READ_BLOCK(world->types, cap * (int32_t)sizeof(uint8_t));
    M2_READ_BLOCK(world->alive, cap * (int32_t)sizeof(uint8_t));
    M2_READ_BLOCK(world->generations, cap * (int32_t)sizeof(uint16_t));
    M2_READ_BLOCK(world->freeQueue, cap * (int32_t)sizeof(int32_t));
#undef M2_READ_BLOCK
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
