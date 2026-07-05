// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Red-team round 3, executable form: id-discipline attacks (stale ids
// across slot reuse), capacity exhaustion, journal header defenses and
// double replay, slot-reuse replay, query edge rays, multi-world
// isolation - and a chaos scene that churns creation, destruction,
// joints, rollback and journal replay into the 13th gated hash line.

#include "world_internal.h"

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

static m2BodyId AddBox(m2WorldId world, double x, double y)
{
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){x, y};
    m2BodyId body = m2CreateBody(world, &bd);
    if (body.index1 != 0)
    {
        m2ShapeDef sd = m2DefaultShapeDef();
        m2Polygon box = m2MakeBox(0.4f, 0.4f);
        m2CreatePolygonShape(body, &sd, &box);
    }
    return body;
}

static void TestStaleIds(void)
{
    // The O(1) id discipline: destroyed ids stay dead forever, even
    // after their slot is reborn under a new generation.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);

    m2BodyId a = AddBox(world, 0.0, 10.0);
    m2BodyId b = AddBox(world, 2.0, 10.0);
    m2DistanceJointDef jd = m2DefaultDistanceJointDef();
    jd.bodyIdA = a;
    jd.bodyIdB = b;
    m2JointId rod = m2CreateDistanceJoint(world, &jd);
    CHECK(m2Body_IsValid(a) && m2Joint_IsValid(rod), "everything valid while alive");

    m2DestroyBody(a);
    CHECK(!m2Body_IsValid(a), "destroyed body id dies immediately");
    CHECK(!m2Joint_IsValid(rod), "body destroy cascades to its joints");
    CHECK(m2Body_IsValid(b), "the counterpart lives");

    // Churn until a's slot is reused; the stale id must stay dead.
    for (int32_t i = 0; i < 8; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        m2BodyId fresh = AddBox(world, 4.0 + (double)i, 10.0);
        if (fresh.index1 == a.index1)
        {
            CHECK(fresh.generation != a.generation, "reused slot carries a new generation");
            CHECK(m2Body_IsValid(fresh) && !m2Body_IsValid(a),
                  "old id stays dead after slot rebirth");
            break;
        }
    }

    m2DestroyWorld(world);
    CHECK(!m2Body_IsValid(b), "world destruction kills every id");
    CHECK(!m2World_IsValid(world), "world id itself dies");
}

static void TestCapacityExhaustion(void)
{
    // Filling the world must fail loudly-but-safely: null ids out,
    // no corruption, and slots come back after the retire flush.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 4;
    def.shapeCapacity = 4;
    m2WorldId world = m2CreateWorld(&def);

    m2BodyId ids[8];
    int32_t created = 0;
    for (int32_t i = 0; i < 8; ++i)
    {
        ids[i] = AddBox(world, (double)i * 2.0, 5.0);
        created += ids[i].index1 != 0 ? 1 : 0;
    }
    CHECK(created == 4, "exactly capacity bodies fit");
    CHECK(ids[4].index1 == 0 && ids[7].index1 == 0, "overflow returns the null id");

    m2World_Step(world, 1.0f / 60.0f, 4);
    uint64_t before = m2World_Hash(world);
    m2World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m2World_Hash(world) != before, "full world still simulates");

    m2DestroyBody(ids[0]);
    for (int32_t i = 0; i < 4; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4); // flush the retire queue
    }
    m2BodyId reborn = AddBox(world, 0.0, 8.0);
    CHECK(reborn.index1 != 0, "slot returns after destroy and flush");

    m2DestroyWorld(world);
}

static void TestJournalDefenses(void)
{
    // Header attacks bounce before the world is touched; a healthy
    // journal replays twice with identical end bits.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);
    uint8_t buffer[1 << 16];
    m2World_StartJournal(world, buffer, (int32_t)sizeof(buffer));

    m2BodyId box = AddBox(world, 0.0, 4.0);
    m2Body_SetLinearVelocity(box, (m2Vec2){1.5f, 0.0f});
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    int32_t bytes = m2World_StopJournal(world);
    CHECK(bytes > 0, "journal captured");
    uint64_t recordedHash = m2World_Hash(world);
    m2DestroyWorld(world);

    m2WorldId fresh = m2CreateWorld(&def);

    uint8_t corrupt[1 << 16];
    memcpy(corrupt, buffer, (size_t)bytes);
    corrupt[0] ^= 0xFF; // magic
    CHECK(!m2World_ReplayJournal(fresh, corrupt, bytes), "corrupt magic rejected");

    memcpy(corrupt, buffer, (size_t)bytes);
    corrupt[4] ^= 0x01; // version
    CHECK(!m2World_ReplayJournal(fresh, corrupt, bytes), "wrong version rejected");

    CHECK(!m2World_ReplayJournal(fresh, buffer, 16), "truncated header rejected");

    m2WorldDef small = m2DefaultWorldDef();
    small.bodyCapacity = 4;
    small.shapeCapacity = 4;
    m2WorldId wrongCapacity = m2CreateWorld(&small);
    CHECK(!m2World_ReplayJournal(wrongCapacity, buffer, bytes), "capacity mismatch rejected");
    m2DestroyWorld(wrongCapacity);

    CHECK(m2World_ReplayJournal(fresh, buffer, bytes), "healthy replay works");
    CHECK(m2World_Hash(fresh) == recordedHash, "replay lands on the recorded bits");
    CHECK(m2World_ReplayJournal(fresh, buffer, bytes), "second replay works");
    CHECK(m2World_Hash(fresh) == recordedHash, "second replay lands on the same bits");

    m2DestroyWorld(fresh);
}

static void TestJournalSlotReuse(void)
{
    // The nasty replay path: a session whose ids die and whose slots
    // are reborn under new generations must still rebind cleanly.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);
    uint8_t buffer[1 << 16];
    m2World_StartJournal(world, buffer, (int32_t)sizeof(buffer));

    m2BodyId doomed = AddBox(world, 0.0, 3.0);
    for (int32_t i = 0; i < 10; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2DestroyBody(doomed);
    for (int32_t i = 0; i < 6; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4); // let the slot retire
    }
    m2BodyId reborn = AddBox(world, 1.0, 3.0); // may reuse the slot
    m2Body_SetLinearVelocity(reborn, (m2Vec2){-2.0f, 1.0f});
    for (int32_t i = 0; i < 20; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }

    int32_t bytes = m2World_StopJournal(world);
    CHECK(bytes > 0, "reuse journal captured");
    uint64_t recordedHash = m2World_Hash(world);
    m2DestroyWorld(world);

    m2WorldId fresh = m2CreateWorld(&def);
    CHECK(m2World_ReplayJournal(fresh, buffer, bytes), "reuse journal replays");
    CHECK(m2World_Hash(fresh) == recordedHash, "slot-reuse session replays bit-exactly");
    m2DestroyWorld(fresh);
}

static void TestQueryEdges(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);

    // Empty world: everything misses quietly.
    m2RayCastResult miss = m2World_CastRayClosest(world, (m2Pos2){0.0, 0.0}, (m2Vec2){5.0f, 0.0f},
                                                  m2DefaultQueryFilter());
    CHECK(!miss.hit, "empty world misses");
    m2ShapeId results[4];
    CHECK(m2World_OverlapAABB(world, (m2Pos2){-1.0, -1.0}, (m2Pos2){1.0, 1.0}, results, 4,
                              m2DefaultQueryFilter()) == 0,
          "empty world overlaps nothing");

    m2BodyId box = AddBox(world, 0.0, 0.0);

    // Zero-length ray inside a shape: initial-overlap convention.
    m2RayCastResult inside = m2World_CastRayClosest(world, (m2Pos2){0.0, 0.0}, (m2Vec2){0.0f, 0.0f},
                                                    m2DefaultQueryFilter());
    CHECK(inside.hit && inside.fraction == 0.0f, "zero ray inside reports initial overlap");
    CHECK(inside.normal.x == 0.0f && inside.normal.y == 0.0f, "initial overlap has no normal");

    // Destroyed shapes drop out of query results after the flush.
    m2DestroyBody(box);
    m2World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m2World_OverlapAABB(world, (m2Pos2){-1.0, -1.0}, (m2Pos2){1.0, 1.0}, results, 4,
                              m2DefaultQueryFilter()) == 0,
          "destroyed shape leaves the query space");

    m2DestroyWorld(world);
}

static void TestMultiWorldIsolation(void)
{
    // Two identical worlds march in lockstep; killing one must not
    // move a single bit in the other.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m2WorldId w1 = m2CreateWorld(&def);
    m2WorldId w2 = m2CreateWorld(&def);
    for (int32_t i = 0; i < 5; ++i)
    {
        AddBox(w1, (double)i, 6.0 + 0.7 * (double)i);
        AddBox(w2, (double)i, 6.0 + 0.7 * (double)i);
    }
    for (int32_t i = 0; i < 40; ++i)
    {
        m2World_Step(w1, 1.0f / 60.0f, 4);
        m2World_Step(w2, 1.0f / 60.0f, 4);
    }
    CHECK(m2World_Hash(w1) == m2World_Hash(w2), "twin worlds march in lockstep");

    uint64_t expected[20];
    {
        // Reference trajectory from a third twin, no destruction around.
        m2WorldId w3 = m2CreateWorld(&def);
        for (int32_t i = 0; i < 5; ++i)
        {
            AddBox(w3, (double)i, 6.0 + 0.7 * (double)i);
        }
        for (int32_t i = 0; i < 40; ++i)
        {
            m2World_Step(w3, 1.0f / 60.0f, 4);
        }
        for (int32_t i = 0; i < 20; ++i)
        {
            m2World_Step(w3, 1.0f / 60.0f, 4);
            expected[i] = m2World_Hash(w3);
        }
        m2DestroyWorld(w3);
    }

    m2DestroyWorld(w1); // kill the twin mid-flight
    for (int32_t i = 0; i < 20; ++i)
    {
        m2World_Step(w2, 1.0f / 60.0f, 4);
        CHECK(m2World_Hash(w2) == expected[i], "survivor unaffected by the neighbor's death");
    }
    m2DestroyWorld(w2);
}

static int32_t s_failAfter = -1; // -1 = never fail
static int32_t s_liveBlocks = 0;
static int32_t s_allocCalls = 0;

static void* FlakyAlloc(size_t bytes)
{
    s_allocCalls += 1;
    if (s_failAfter >= 0 && s_allocCalls > s_failAfter)
    {
        return NULL;
    }
    s_liveBlocks += 1;
    return calloc(1, bytes);
}

static void FlakyFree(void* memory)
{
    if (memory != NULL)
    {
        s_liveBlocks -= 1;
    }
    free(memory);
}

static void TestAllocationFailure(void)
{
    // The allocator fails mid-creation at several depths: the world
    // must come back null, and every block taken must be returned.
    m2SetAllocator(FlakyAlloc, FlakyFree);
    int32_t depths[4] = {1, 5, 20, 50};
    for (int32_t d = 0; d < 4; ++d)
    {
        s_allocCalls = 0;
        s_liveBlocks = 0;
        s_failAfter = depths[d];
        m2WorldDef def = m2DefaultWorldDef();
        def.bodyCapacity = 32;
        def.shapeCapacity = 32;
        m2WorldId world = m2CreateWorld(&def);
        CHECK(world.index1 == 0, "starved creation returns the null world");
        CHECK(s_liveBlocks == 0, "starved creation leaks nothing");
    }
    s_failAfter = -1;
    m2SetAllocator(NULL, NULL);
}

static void TestFrozenEraRollback(void)
{
    // A long-asleep stack runs the frozen-pair fast path; snapshot and
    // replay inside that era must still be bit-exact (sleepStreak is
    // snapshot state, and the skip is a pure function of it).
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m2WorldId world = m2CreateWorld(&def);
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = m2CreateBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(15.0f, 0.5f);
    m2CreatePolygonShape(floor, &fs, &slab);
    for (int32_t i = 0; i < 5; ++i)
    {
        AddBox(world, 0.0, 0.5 + 0.9 * (double)i);
    }
    for (int32_t i = 0; i < 160; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4); // settle deep into the frozen era
    }

    int32_t size = m2World_SnapshotSize(world);
    void* snap = malloc((size_t)size);
    CHECK(m2World_Snapshot(world, snap, size) == size, "snapshot");
    uint64_t hashes[50];
    for (int32_t i = 0; i < 50; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        hashes[i] = m2World_Hash(world);
    }
    CHECK(m2World_Restore(world, snap, size), "restore");
    for (int32_t i = 0; i < 50; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        CHECK(m2World_Hash(world) == hashes[i], "frozen era replays bit-exactly");
    }
    free(snap);
    m2DestroyWorld(world);
}

static void TestWakeAfterFrozenEra(void)
{
    // No stale-manifold ghosts: a body that slept through many frozen
    // steps must move immediately when shoved, then earn sleep again.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = m2CreateBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(15.0f, 0.5f);
    m2CreatePolygonShape(floor, &fs, &slab);
    m2BodyId box = AddBox(world, 0.0, 0.5);
    for (int32_t i = 0; i < 150; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m2Body_IsAwake(box), "asleep deep in the frozen era");
    double x0 = m2Body_GetPosition(box).x;

    m2Body_ApplyLinearImpulse(box, (m2Vec2){2.0f, 0.0f}, m2Body_GetPosition(box));
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(box).x > x0 + 0.5, "the shoved sleeper actually moves");

    bool sleptAgain = false;
    for (int32_t i = 0; i < 300 && !sleptAgain; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        sleptAgain = !m2Body_IsAwake(box);
    }
    CHECK(sleptAgain, "and earns its sleep back");
    m2DestroyWorld(world);
}

static uint64_t ChurnTrajectory(int32_t workerCount)
{
    // Lifecycle churn + joints + sleep phases under a worker count:
    // the thread law must hold beyond static scenes.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 32;
    def.shapeCapacity = 32;
    def.jointCapacity = 16;
    def.workerCount = workerCount;
    m2WorldId world = m2CreateWorld(&def);
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){2.2e5, -0.5};
    m2BodyId floor = m2CreateBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(20.0f, 0.5f);
    m2CreatePolygonShape(floor, &fs, &slab);

    m2BodyId ring[8] = {0};
    int32_t head = 0;
    int32_t count = 0;
    uint64_t h = M2_HASH_INIT;
    for (int32_t step = 0; step < 240; ++step)
    {
        if (step % 9 == 0 && count < 8)
        {
            m2BodyId body = AddBox(world, 2.2e5 - 3.0 + 0.8 * (double)((step / 9) % 9),
                                   1.0 + 0.4 * (double)(step % 3));
            if (body.index1 != 0)
            {
                ring[(head + count) % 8] = body;
                count += 1;
            }
        }
        if (step % 23 == 11 && count > 3)
        {
            m2DestroyBody(ring[head]);
            head = (head + 1) % 8;
            count -= 1;
        }
        if (step == 120 && count >= 2)
        {
            m2DistanceJointDef jd = m2DefaultDistanceJointDef();
            jd.bodyIdA = ring[head];
            jd.bodyIdB = ring[(head + 1) % 8];
            m2CreateDistanceJoint(world, &jd);
        }
        m2World_Step(world, 1.0f / 60.0f, 4);
        uint64_t worldHash = m2World_Hash(world);
        h = m2Hash64(h, &worldHash, (int32_t)sizeof(worldHash));
    }
    m2DestroyWorld(world);
    return h;
}

static void TestThreadedChurnParity(void)
{
    uint64_t one = ChurnTrajectory(1);
    uint64_t four = ChurnTrajectory(4);
    CHECK(one == four, "the thread law survives lifecycle churn");
}

static void TestTeleportOntoSleepers(void)
{
    // Teleport a heavy box INTO a sleeping stack: overlap must resolve
    // through the ordinary pushout without an explosion, and the stack
    // must wake.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m2WorldId world = m2CreateWorld(&def);
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = m2CreateBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(15.0f, 0.5f);
    m2CreatePolygonShape(floor, &fs, &slab);
    m2BodyId a = AddBox(world, 0.0, 0.5);
    m2BodyId b = AddBox(world, 0.0, 1.42);
    m2BodyId intruder = AddBox(world, 8.0, 0.5);
    for (int32_t i = 0; i < 200; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m2Body_IsAwake(a) && !m2Body_IsAwake(b), "stack asleep");

    m2Body_SetTransform(intruder, (m2Pos2){0.1, 0.9}, (m2Rot){1.0f, 0.0f}); // into the stack
    bool exploded = false;
    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        m2Vec2 v = m2Body_GetLinearVelocity(b);
        exploded = exploded || v.x * v.x + v.y * v.y > 100.0f;
    }
    CHECK(!exploded, "overlap resolves without an explosion");
    CHECK(m2Body_GetPosition(b).y > 0.3, "the stack sorts itself out");
    m2DestroyWorld(world);
}

static void TestSetTypeEdges(void)
{
    // Convert a sleeping body, a jointed body, and a bullet; flip-flop
    // types under twin-world scrutiny.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.jointCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = m2CreateBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(15.0f, 0.5f);
    m2CreatePolygonShape(floor, &fs, &slab);

    // Sleeping conversion round trip.
    m2BodyId sleeper = AddBox(world, 0.0, 0.5);
    for (int32_t i = 0; i < 150; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m2Body_IsAwake(sleeper), "asleep before conversion");
    m2Body_SetType(sleeper, m2_staticBody);
    m2Body_SetType(sleeper, m2_dynamicBody);
    m2World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m2Body_IsAwake(sleeper), "round-trip conversion wakes it");
    bool resleeps = false;
    for (int32_t i = 0; i < 200 && !resleeps; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        resleeps = !m2Body_IsAwake(sleeper);
    }
    CHECK(resleeps, "and it earns sleep again with real mass");

    // A pendulum whose pivot end turns static mid-swing.
    m2BodyDef ad = m2DefaultBodyDef();
    ad.type = m2_dynamicBody;
    ad.position = (m2Pos2){5.0, 4.0};
    m2BodyId pivot = m2CreateBody(world, &ad);
    m2ShapeDef ps = m2DefaultShapeDef();
    m2Circle pc = {{0.0f, 0.0f}, 0.2f};
    m2CreateCircleShape(pivot, &ps, &pc);
    m2BodyId bob = AddBox(world, 6.5, 4.0);
    m2RevoluteJointDef jd = m2DefaultRevoluteJointDef();
    jd.bodyIdA = pivot;
    jd.bodyIdB = bob;
    jd.localAnchorB = (m2Vec2){-1.5f, 0.0f};
    m2JointId hinge = m2CreateRevoluteJoint(world, &jd);
    m2Body_SetType(pivot, m2_staticBody); // becomes a wall anchor
    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Joint_IsValid(hinge), "the joint survives its anchor turning static");
    m2Pos2 pp = m2Body_GetPosition(pivot);
    CHECK(pp.x == 5.0 && pp.y == 4.0, "the static anchor holds its ground");
    double dx = m2Body_GetPosition(bob).x - pp.x;
    double dy = m2Body_GetPosition(bob).y - pp.y;
    double rod = dx * dx + dy * dy;
    CHECK(rod > 1.9 && rod < 2.6, "the bob still hangs from it");

    // A bullet that turns static mid-flight simply stops mattering.
    m2BodyDef bulletDef = m2DefaultBodyDef();
    bulletDef.type = m2_dynamicBody;
    bulletDef.isBullet = true;
    bulletDef.position = (m2Pos2){-8.0, 2.0};
    bulletDef.linearVelocity = (m2Vec2){60.0f, 0.0f};
    m2BodyId bullet = m2CreateBody(world, &bulletDef);
    m2ShapeDef bs = m2DefaultShapeDef();
    m2Circle bc = {{0.0f, 0.0f}, 0.1f};
    m2CreateCircleShape(bullet, &bs, &bc);
    m2World_Step(world, 1.0f / 60.0f, 4);
    m2Body_SetType(bullet, m2_staticBody);
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4); // must not crash the sweep
    }
    CHECK(m2Body_GetLinearVelocity(bullet).x == 0.0f, "a static bullet is just a pebble");

    m2DestroyWorld(world);
}

static uint64_t FlipFlopTrajectory(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = m2CreateBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(10.0f, 0.5f);
    m2CreatePolygonShape(floor, &fs, &slab);
    m2BodyId flipper = AddBox(world, 0.0, 2.0);
    m2BodyId witness = AddBox(world, 0.9, 3.0);
    (void)witness;
    uint64_t h = M2_HASH_INIT;
    for (int32_t i = 0; i < 120; ++i)
    {
        if (i % 7 == 3)
        {
            m2Body_SetType(flipper, i % 14 == 3 ? m2_kinematicBody : m2_dynamicBody);
        }
        m2World_Step(world, 1.0f / 60.0f, 4);
        uint64_t wh = m2World_Hash(world);
        h = m2Hash64(h, &wh, (int32_t)sizeof(wh));
    }
    m2DestroyWorld(world);
    return h;
}

static void TestFlipFlopDeterminism(void)
{
    CHECK(FlipFlopTrajectory() == FlipFlopTrajectory(), "type flip-flop is twin-deterministic");
}

static void TestLoopChainBowl(void)
{
    // A closed loop chain forming a diamond bowl: a ball dropped in
    // settles inside; degenerate chain defs bounce loudly.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 16;
    m2WorldId world = m2CreateWorld(&def);
    m2BodyDef gd = m2DefaultBodyDef();
    gd.position = (m2Pos2){0.0, 0.0};
    m2BodyId frame = m2CreateBody(world, &gd);
    // Diamond wound clockwise so the solid side faces INWARD.
    m2Vec2 diamond[4] = {{0.0f, 3.0f}, {3.0f, 0.0f}, {0.0f, -3.0f}, {-3.0f, 0.0f}};
    m2ChainDef chain = m2DefaultChainDef();
    chain.points = diamond;
    chain.count = 4;
    chain.isLoop = true;
    CHECK(m2CreateChain(frame, &chain) == 4, "a loop of four points makes four segments");

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.3, 1.5};
    m2BodyId ball = m2CreateBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Circle c = {{0.0f, 0.0f}, 0.3f};
    m2CreateCircleShape(ball, &sd, &c);
    bool rested = false;
    for (int32_t i = 0; i < 600 && !rested; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        rested = !m2Body_IsAwake(ball);
    }
    CHECK(rested, "the ball settles inside the bowl");
    CHECK(m2Body_GetPosition(ball).y > -3.0, "and did not leak out the bottom");

    m2DestroyWorld(world);
}

static void TestBreakStorm(void)
{
    // Every rope in a curtain breaks on the same overloaded step: all
    // events arrive that step, in canonical joint order, and a
    // pre-break snapshot resurrects the whole curtain.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.jointCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);
    m2JointId ropes[4];
    for (int32_t i = 0; i < 4; ++i)
    {
        m2BodyDef ad = m2DefaultBodyDef();
        ad.position = (m2Pos2){(double)i * 3.0, 6.0};
        m2BodyId hook = m2CreateBody(world, &ad);
        m2BodyDef cd = m2DefaultBodyDef();
        cd.type = m2_dynamicBody;
        cd.position = (m2Pos2){(double)i * 3.0, 4.0};
        m2BodyId crate = m2CreateBody(world, &cd);
        m2ShapeDef sd = m2DefaultShapeDef();
        sd.density = 5.0f;
        m2Polygon box = m2MakeBox(0.5f, 0.5f);
        m2CreatePolygonShape(crate, &sd, &box);
        m2DistanceJointDef jd = m2DefaultDistanceJointDef();
        jd.bodyIdA = hook;
        jd.bodyIdB = crate;
        ropes[i] = m2CreateDistanceJoint(world, &jd);
        m2Joint_SetBreakLimits(ropes[i], 30.0f, 0.0f);
    }

    int32_t size = m2World_SnapshotSize(world);
    void* snap = malloc((size_t)size);
    CHECK(m2World_Snapshot(world, snap, size) == size, "snapshot");

    int32_t stormStep = -1;
    for (int32_t i = 0; i < 60 && stormStep < 0; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        m2JointEvents events = m2World_GetJointEvents(world);
        if (events.breakCount > 0)
        {
            stormStep = i;
            CHECK(events.breakCount == 4, "the whole curtain goes at once");
            for (int32_t k = 1; k < events.breakCount; ++k)
            {
                CHECK(events.breakEvents[k - 1].jointId.index1 <
                          events.breakEvents[k].jointId.index1,
                      "break events arrive in canonical joint order");
            }
        }
    }
    CHECK(stormStep >= 0, "the storm came");

    CHECK(m2World_Restore(world, snap, size), "rollback before the storm");
    for (int32_t i = 0; i < 4; ++i)
    {
        CHECK(m2Joint_IsValid(ropes[i]), "restore resurrects the curtain");
    }
    int32_t stormAgain = -1;
    for (int32_t i = 0; i < 60 && stormAgain < 0; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        if (m2World_GetJointEvents(world).breakCount > 0)
        {
            stormAgain = i;
        }
    }
    CHECK(stormAgain == stormStep, "and the storm returns on the identical step");

    free(snap);
    m2DestroyWorld(world);
}

static uint64_t SetterChurnTrajectory(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m2WorldId world = m2CreateWorld(&def);
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = m2CreateBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(15.0f, 0.5f);
    m2ShapeId floorShape = m2CreatePolygonShape(floor, &fs, &slab);
    m2ShapeId boxes[4];
    for (int32_t i = 0; i < 4; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){-3.0 + 2.0 * (double)i, 1.0 + 0.5 * (double)i};
        m2BodyId body = m2CreateBody(world, &bd);
        m2ShapeDef sd = m2DefaultShapeDef();
        m2Polygon unit = m2MakeBox(0.4f, 0.4f);
        boxes[i] = m2CreatePolygonShape(body, &sd, &unit);
    }
    uint64_t h = M2_HASH_INIT;
    for (int32_t step = 0; step < 180; ++step)
    {
        if (step % 13 == 4)
        {
            m2World_SetGravity(world, (m2Vec2){0.0f, step % 26 == 4 ? -10.0f : -6.0f});
        }
        if (step % 17 == 8)
        {
            m2Shape_SetFriction(floorShape, step % 34 == 8 ? 0.9f : 0.1f);
        }
        if (step % 23 == 11)
        {
            m2Shape_SetFilter(boxes[step % 4], step % 46 == 11 ? 0x4u : 0x1u, 0xFFFFFFFFu, 0);
        }
        m2World_Step(world, 1.0f / 60.0f, 4);
        uint64_t wh = m2World_Hash(world);
        h = m2Hash64(h, &wh, (int32_t)sizeof(wh));
    }
    m2DestroyWorld(world);
    return h;
}

static void TestSetterChurnDeterminism(void)
{
    CHECK(SetterChurnTrajectory() == SetterChurnTrajectory(), "setter churn is twin-deterministic");
}

static void TestKinematicSensorHibernation(void)
{
    // A kinematic at rest inside a static sensor must not hold the
    // world out of hibernation; when it moves again, the exit is
    // witnessed on the sensor stream.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = m2CreateBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(10.0f, 0.5f);
    m2CreatePolygonShape(floor, &fs, &slab);
    m2BodyId sleeperBox = AddBox(world, 3.0, 0.5);

    m2BodyDef zd = m2DefaultBodyDef();
    zd.position = (m2Pos2){-3.0, 1.0};
    m2BodyId zoneBody = m2CreateBody(world, &zd);
    m2ShapeDef zs = m2DefaultShapeDef();
    zs.isSensor = true;
    m2Polygon zone = m2MakeBox(1.5f, 1.5f);
    m2ShapeId zoneId = m2CreatePolygonShape(zoneBody, &zs, &zone);

    m2BodyDef kd = m2DefaultBodyDef();
    kd.type = m2_kinematicBody;
    kd.position = (m2Pos2){-3.0, 1.0};
    m2BodyId guard = m2CreateBody(world, &kd);
    m2ShapeDef gs = m2DefaultShapeDef();
    m2Circle body = {{0.0f, 0.0f}, 0.3f};
    m2CreateCircleShape(guard, &gs, &body);

    for (int32_t i = 0; i < 250; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m2Body_IsAwake(sleeperBox), "the dynamic box sleeps");
    m2ShapeId inside[2];
    CHECK(m2Shape_GetSensorOverlaps(zoneId, inside, 2) == 1,
          "the resting guard still shows in the roll call");
    m2Profile profile = m2World_GetProfile(world);
    CHECK(profile.contactsMs == 0.0f && profile.solveMs == 0.0f,
          "a resting guard does not hold the world out of hibernation");

    m2Body_SetLinearVelocity(guard, (m2Vec2){4.0f, 0.0f});
    bool sawExit = false;
    for (int32_t i = 0; i < 60 && !sawExit; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        sawExit = m2World_GetSensorEvents(world).endCount > 0;
    }
    CHECK(sawExit, "the guard's exit is witnessed");

    m2DestroyWorld(world);
}

static void TestBulletThroughOneWayChain(void)
{
    // A one-way platform must be one-way for bullets too: from the
    // ghost side the bullet keeps its speed; from the solid side it
    // stops on the platform.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 16;
    m2WorldId world = m2CreateWorld(&def);
    m2BodyDef gd = m2DefaultBodyDef();
    gd.position = (m2Pos2){0.0, 2.0};
    m2BodyId platform = m2CreateBody(world, &gd);
    m2Vec2 pts[5] = {{6.0f, 0.0f}, {3.0f, 0.0f}, {0.0f, 0.0f}, {-3.0f, 0.0f}, {-6.0f, 0.0f}};
    m2ChainDef chain = m2DefaultChainDef();
    chain.points = pts;
    chain.count = 5; // right-to-left: solid side up
    CHECK(m2CreateChain(platform, &chain) == 2, "two live segments");

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.isBullet = true;
    bd.position = (m2Pos2){0.5, -6.0};
    bd.linearVelocity = (m2Vec2){0.0f, 80.0f}; // straight up through the ghost side
    m2BodyId riser = m2CreateBody(world, &bd);
    m2ShapeDef bs = m2DefaultShapeDef();
    m2Circle ball = {{0.0f, 0.0f}, 0.1f};
    m2CreateCircleShape(riser, &bs, &ball);

    for (int32_t i = 0; i < 12; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(riser).y > 4.0, "the rising bullet passed the one-way platform");
    CHECK(m2Body_GetLinearVelocity(riser).y > 60.0f, "without losing its speed to the ghost");

    bd.position = (m2Pos2){-0.5, 10.0};
    bd.linearVelocity = (m2Vec2){0.0f, -80.0f};
    m2BodyId diver = m2CreateBody(world, &bd);
    m2CreateCircleShape(diver, &bs, &ball);
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(diver).y > 1.9, "the diving bullet stopped on the solid side");

    m2DestroyWorld(world);
}

static uint64_t ChaosHash(void)
{
    // Lifecycle churn far from the origin: bodies and joints created
    // and destroyed on deterministic cycles, a mid-flight rollback, and
    // a journal sub-session replayed into a fresh world - all folded
    // into one line.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 64;
    def.shapeCapacity = 64;
    def.jointCapacity = 32;
    m2WorldId world = m2CreateWorld(&def);

    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){7.7e5, -0.5};
    m2BodyId floor = m2CreateBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(25.0f, 0.5f);
    m2CreatePolygonShape(floor, &fs, &slab);

    m2BodyId ring[16];
    int32_t ringHead = 0;
    int32_t ringCount = 0;
    m2JointId joints[8];
    int32_t jointHead = 0;
    int32_t jointCount = 0;

    void* snapshot = NULL;
    int32_t snapshotSize = 0;
    uint8_t journal[1 << 16];
    int32_t journalBytes = 0;
    uint64_t h = M2_HASH_INIT;

    for (int32_t step = 0; step < 220; ++step)
    {
        if (step % 7 == 0 && ringCount < 16)
        {
            double x = 7.7e5 - 8.0 + 0.9 * (double)((step / 7) % 18);
            m2BodyId body = AddBox(world, x, 2.0 + 0.6 * (double)(step % 5));
            if (body.index1 != 0)
            {
                ring[(ringHead + ringCount) % 16] = body;
                ringCount += 1;
            }
        }
        if (step % 11 == 3 && ringCount > 2)
        {
            m2DestroyBody(ring[ringHead]);
            ringHead = (ringHead + 1) % 16;
            ringCount -= 1;
        }
        if (step % 13 == 5 && ringCount >= 2 && jointCount < 8)
        {
            m2BodyId a = ring[(ringHead + ringCount - 2) % 16];
            m2BodyId b = ring[(ringHead + ringCount - 1) % 16];
            if (m2Body_IsValid(a) && m2Body_IsValid(b))
            {
                m2DistanceJointDef jd = m2DefaultDistanceJointDef();
                jd.bodyIdA = a;
                jd.bodyIdB = b;
                m2JointId joint = m2CreateDistanceJoint(world, &jd);
                if (joint.index1 != 0)
                {
                    joints[(jointHead + jointCount) % 8] = joint;
                    jointCount += 1;
                }
            }
        }
        if (step % 19 == 7 && ringCount > 1 && m2Body_IsValid(ring[ringHead]))
        {
            // Teleport the oldest ring member to a fresh drop point.
            m2Body_SetTransform(ring[ringHead],
                                (m2Pos2){7.7e5 + 6.0 + 0.3 * (double)(step % 5), 4.0},
                                m2MakeRot(0.1f * (float)(step % 7)));
        }
        if (step % 29 == 13 && ringCount > 2 && m2Body_IsValid(ring[(ringHead + 1) % 16]))
        {
            // Flip a member's nature and back a few steps later via the
            // deterministic schedule.
            m2Body_SetType(ring[(ringHead + 1) % 16],
                           step % 58 == 13 ? m2_kinematicBody : m2_dynamicBody);
        }
        if (step % 17 == 9 && jointCount > 0)
        {
            if (m2Joint_IsValid(joints[jointHead]))
            {
                m2DestroyJoint(joints[jointHead]);
            }
            jointHead = (jointHead + 1) % 8;
            jointCount -= 1;
        }

        if (step == 40)
        {
            snapshotSize = m2World_SnapshotSize(world);
            snapshot = malloc((size_t)snapshotSize);
            m2World_Snapshot(world, snapshot, snapshotSize);
        }
        if (step == 60 && snapshot != NULL)
        {
            // Rollback under churn; ids taken since the snapshot die
            // with the restore, so the rings restart from scratch.
            m2World_Restore(world, snapshot, snapshotSize);
            ringCount = 0;
            ringHead = 0;
            jointCount = 0;
            jointHead = 0;
        }
        if (step == 100)
        {
            m2World_StartJournal(world, journal, (int32_t)sizeof(journal));
        }
        if (step == 160)
        {
            journalBytes = m2World_StopJournal(world);
        }

        m2World_Step(world, 1.0f / 60.0f, 4);
        uint64_t worldHash = m2World_Hash(world);
        h = m2Hash64(h, &worldHash, (int32_t)sizeof(worldHash));
    }

    if (journalBytes > 0)
    {
        m2WorldId fresh = m2CreateWorld(&def);
        if (m2World_ReplayJournal(fresh, journal, journalBytes))
        {
            uint64_t replayHash = m2World_Hash(fresh);
            h = m2Hash64(h, &replayHash, (int32_t)sizeof(replayHash));
        }
        m2DestroyWorld(fresh);
    }

    free(snapshot);
    m2DestroyWorld(world);
    return h;
}

int main(void)
{
    TestStaleIds();
    TestCapacityExhaustion();
    TestJournalDefenses();
    TestJournalSlotReuse();
    TestQueryEdges();
    TestMultiWorldIsolation();
    TestAllocationFailure();
    TestFrozenEraRollback();
    TestWakeAfterFrozenEra();
    TestThreadedChurnParity();
    TestTeleportOntoSleepers();
    TestSetTypeEdges();
    TestFlipFlopDeterminism();
    TestLoopChainBowl();
    TestBreakStorm();
    TestSetterChurnDeterminism();
    TestKinematicSensorHibernation();
    TestBulletThroughOneWayChain();

    uint64_t hash = ChaosHash();
    printf("M2_CHAOS_HASH=%016llx\n", (unsigned long long)hash);

    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
