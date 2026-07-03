// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Broadphase gate: tree structural invariants vs a brute-force oracle
// (own PRNG, registry M22), the pair pipeline (eager creation pairing,
// kinematic-sweep pairing - the RT1-STAB-1 mechanism - and destroy
// pruning), rollback identity over broadphase state, and the pair
// evolution hash compared across CI cells.

#include "world_internal.h"

static m2ShapeId AttachUnitBox(m2BodyId body)
{
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon box = m2MakeBox(0.5f, 0.5f);
    return m2CreatePolygonShape(body, &sd, &box);
}

#include "maul2d/base.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int s_failures = 0;

#define CHECK(cond, msg)                                                                           \
    do                                                                                             \
    {                                                                                              \
        if (!(cond))                                                                               \
        {                                                                                          \
            printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__);                                 \
            s_failures += 1;                                                                       \
        }                                                                                          \
    } while (0)

// Deterministic test PRNG (splitmix64): platform rand() is banned (M22).
static uint64_t s_rngState = 0x9E3779B97F4A7C15ULL;
static uint64_t NextRandom(void)
{
    s_rngState += 0x9E3779B97F4A7C15ULL;
    uint64_t z = s_rngState;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

static double RandomInRange(double lo, double hi)
{
    double t = (double)(NextRandom() >> 11) * (1.0 / 9007199254740992.0);
    return lo + t * (hi - lo);
}

static m2AABB RandomAabb(void)
{
    m2AABB aabb;
    aabb.lowerBound.x = RandomInRange(-100.0, 100.0);
    aabb.lowerBound.y = RandomInRange(-100.0, 100.0);
    aabb.upperBound.x = aabb.lowerBound.x + RandomInRange(0.1, 8.0);
    aabb.upperBound.y = aabb.lowerBound.y + RandomInRange(0.1, 8.0);
    return aabb;
}

static void TestTreeStructure(void)
{
    enum
    {
        N = 200,
        CAPACITY = 2 * N
    };
    m2DynamicTree tree;
    m2TreeNode* nodes = calloc(CAPACITY, sizeof(m2TreeNode));
    m2AABB boxes[N];
    int32_t proxies[N];
    m2Tree_Init(&tree, nodes, CAPACITY);

    for (int32_t i = 0; i < N; ++i)
    {
        boxes[i] = RandomAabb();
        proxies[i] = m2Tree_Insert(&tree, nodes, boxes[i], i);
        CHECK(proxies[i] != M2_NULL_NODE, "tree insert must succeed within capacity");
    }
    CHECK(m2Tree_Validate(&tree, nodes), "tree valid after inserts");

    // Query oracle: tree results must equal brute force exactly.
    for (int32_t q = 0; q < 50; ++q)
    {
        m2AABB query = RandomAabb();
        int32_t results[N];
        int32_t hits = m2Tree_Query(&tree, nodes, query, results, N);
        int32_t brute = 0;
        for (int32_t i = 0; i < N; ++i)
        {
            brute += m2AABB_Overlaps(boxes[i], query) ? 1 : 0;
        }
        CHECK(hits == brute, "tree query must match brute force");
    }

    // Remove every other proxy; the survivors must stay intact.
    for (int32_t i = 0; i < N; i += 2)
    {
        m2Tree_Remove(&tree, nodes, proxies[i]);
    }
    CHECK(m2Tree_Validate(&tree, nodes), "tree valid after removals");
    for (int32_t q = 0; q < 25; ++q)
    {
        m2AABB query = RandomAabb();
        int32_t results[N];
        int32_t hits = m2Tree_Query(&tree, nodes, query, results, N);
        int32_t brute = 0;
        for (int32_t i = 1; i < N; i += 2)
        {
            brute += m2AABB_Overlaps(boxes[i], query) ? 1 : 0;
        }
        CHECK(hits == brute, "tree query after removal must match brute force");
    }

    free(nodes);
}

static m2AABB ShapeFat(const m2World* world, int32_t s)
{
    int32_t tree = world->types[world->shapeBody[s]];
    return world->treeNodes[tree][world->proxyIds[s]].aabb;
}

static int32_t BrutePairCount(const m2World* world)
{
    int32_t count = 0;
    for (int32_t a = 0; a < world->maxShapeIndex; ++a)
    {
        if (world->shapeAlive[a] == 0)
        {
            continue;
        }
        for (int32_t b = a + 1; b < world->maxShapeIndex; ++b)
        {
            if (world->shapeAlive[b] == 0 || world->shapeBody[a] == world->shapeBody[b])
            {
                continue;
            }
            if (world->types[world->shapeBody[a]] != (uint8_t)m2_dynamicBody &&
                world->types[world->shapeBody[b]] != (uint8_t)m2_dynamicBody)
            {
                continue;
            }
            count += m2AABB_Overlaps(ShapeFat(world, a), ShapeFat(world, b)) ? 1 : 0;
        }
    }
    return count;
}

static void TestPairPipeline(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 128;
    def.gravity = (m2Vec2){0.0f, 0.0f}; // isolate broadphase from motion
    m2WorldId worldId = m2CreateWorld(&def);
    m2World* world = m2World_GetInternal(worldId);

    // A sparse grid: no overlaps, so no pairs after the first step.
    m2BodyId grid[16];
    for (int32_t i = 0; i < 16; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){3.0 * (double)(i % 4), 3.0 * (double)(i / 4)};
        grid[i] = m2CreateBody(worldId, &bd);
        AttachUnitBox(grid[i]);
    }
    m2World_Step(worldId, 1.0f / 60.0f, 4);
    CHECK(world->pairCount == 0, "sparse grid has no pairs");
    CHECK(world->pairCount == BrutePairCount(world), "pair set matches brute force (sparse)");

    // Two overlapping dynamics pair up on creation (eager proxies).
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.4, 0.0};
    m2BodyId intruder = m2CreateBody(worldId, &bd);
    AttachUnitBox(intruder);
    m2World_Step(worldId, 1.0f / 60.0f, 4);
    CHECK(world->pairCount == BrutePairCount(world), "pair set matches brute force (intruder)");
    CHECK(world->pairCount >= 1, "creation overlap must produce a pair");

    // Kinematic sweep (RT1-STAB-1): a kinematic body plows through the
    // grid; every overlap it reaches must become a pair.
    m2BodyDef kd = m2DefaultBodyDef();
    kd.type = m2_kinematicBody;
    kd.position = (m2Pos2){-6.0, 0.0};
    kd.linearVelocity = (m2Vec2){30.0f, 0.0f};
    m2BodyId sweeper = m2CreateBody(worldId, &kd);
    m2ShapeId sweeperShape = AttachUnitBox(sweeper);
    bool sweeperPaired = false;
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(worldId, 1.0f / 60.0f, 4);
        CHECK(world->pairCount == BrutePairCount(world), "pair set tracks the kinematic sweep");
        for (int32_t p = 0; p < world->pairCount; ++p)
        {
            int32_t a = (int32_t)(world->pairKeys[p] >> 32);
            int32_t b = (int32_t)(world->pairKeys[p] & 0xFFFFFFFFu);
            if (a == sweeperShape.index1 - 1 || b == sweeperShape.index1 - 1)
            {
                sweeperPaired = true;
            }
        }
    }
    CHECK(sweeperPaired, "kinematic sweep must generate pairs (RT1-STAB-1)");

    // Destroy pruning: removing a body removes its pairs the same call.
    m2DestroyBody(intruder);
    CHECK(world->pairCount == BrutePairCount(world), "destroy prunes pairs immediately");
    (void)grid;

    m2DestroyWorld(worldId);
}

static void TestBroadphaseRollback(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 64;
    m2WorldId worldId = m2CreateWorld(&def);

    // A falling column over a static floor: pairs evolve as bodies drop.
    m2BodyDef floorDef = m2DefaultBodyDef();
    floorDef.position = (m2Pos2){0.0, -2.0};
    AttachUnitBox(m2CreateBody(worldId, &floorDef));
    for (int32_t i = 0; i < 20; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){0.25 * (double)(i % 2), 1.5 * (double)i};
        AttachUnitBox(m2CreateBody(worldId, &bd));
    }

    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(worldId, 1.0f / 60.0f, 4);
    }

    int32_t size = m2World_SnapshotSize(worldId);
    void* snapA = malloc((size_t)size);
    void* snapB = malloc((size_t)size);
    CHECK(m2World_Snapshot(worldId, snapA, size) == size, "snapshot");
    CHECK(m2World_Restore(worldId, snapA, size), "restore");
    CHECK(m2World_Snapshot(worldId, snapB, size) == size, "re-snapshot");
    CHECK(memcmp(snapA, snapB, (size_t)size) == 0,
          "broadphase state (trees, moved set, pairs) survives byte-compare");

    uint64_t hashes[40];
    for (int32_t i = 0; i < 40; ++i)
    {
        m2World_Step(worldId, 1.0f / 60.0f, 4);
        hashes[i] = m2World_Hash(worldId);
    }
    CHECK(m2World_Restore(worldId, snapA, size), "restore for replay");
    for (int32_t i = 0; i < 40; ++i)
    {
        m2World_Step(worldId, 1.0f / 60.0f, 4);
        CHECK(m2World_Hash(worldId) == hashes[i], "pair evolution replays bit-exactly");
    }

    free(snapA);
    free(snapB);
    m2DestroyWorld(worldId);
}

static uint64_t BroadphaseSweepHash(void)
{
    // Deterministic rain: bodies spawn on a fixed schedule, fall through a
    // static field, pairs churn constantly. The rolling hash of the pair
    // sets is compared across every CI platform cell.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 256;
    m2WorldId worldId = m2CreateWorld(&def);
    m2World* world = m2World_GetInternal(worldId);

    for (int32_t i = 0; i < 12; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.position = (m2Pos2){2.5 * (double)i - 15.0, -4.0};
        AttachUnitBox(m2CreateBody(worldId, &bd));
    }

    uint64_t h = M2_HASH_INIT;
    for (int32_t step = 0; step < 240; ++step)
    {
        if (step % 4 == 0)
        {
            m2BodyDef bd = m2DefaultBodyDef();
            bd.type = m2_dynamicBody;
            bd.position = (m2Pos2){-14.0 + 0.47 * (double)(step % 60), 6.0 + 0.03 * (double)step};
            bd.linearVelocity = (m2Vec2){(float)(step % 7) - 3.0f, 0.0f};
            AttachUnitBox(m2CreateBody(worldId, &bd));
        }
        m2World_Step(worldId, 1.0f / 60.0f, 4);
        h = m2Hash64(h, world->pairKeys, world->pairCount * (int32_t)sizeof(uint64_t));
        h = m2Hash64(h, &world->pairCount, (int32_t)sizeof(world->pairCount));
    }
    m2DestroyWorld(worldId);
    return h;
}

int main(void)
{
    TestTreeStructure();
    TestPairPipeline();
    TestBroadphaseRollback();

    uint64_t hash = BroadphaseSweepHash();
    printf("M2_BP_HASH=%016llx\n", (unsigned long long)hash);

    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
