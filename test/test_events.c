// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Event gate: begin/end symmetry across the full contact lifecycle
// (land, launch, destroy-while-touching, between-step destroy), canonical
// order, restore-clears-buffers with identical re-emission on replay,
// and the event stream hash across CI cells.

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

static m2BodyId AddSlab(m2WorldId world)
{
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId body = m2CreateBody(world, &fd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(20.0f, 0.5f);
    m2CreatePolygonShape(body, &sd, &slab);
    return body;
}

static void TestLifecycleSymmetry(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);
    AddSlab(world);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 2.0};
    m2BodyId ball = m2CreateBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Circle circle = {{0.0f, 0.0f}, 0.4f};
    m2ShapeId ballShape = m2CreateCircleShape(ball, &sd, &circle);

    // Fall until begin fires; exactly one begin, no end yet.
    int32_t begins = 0;
    int32_t ends = 0;
    int32_t beginStep = -1;
    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        m2ContactEvents ev = m2World_GetContactEvents(world);
        begins += ev.beginCount;
        ends += ev.endCount;
        if (ev.beginCount > 0 && beginStep < 0)
        {
            beginStep = i;
            CHECK(ev.beginEvents[0].shapeIdB.index1 == ballShape.index1 ||
                      ev.beginEvents[0].shapeIdA.index1 == ballShape.index1,
                  "begin event names the ball shape");
        }
    }
    CHECK(begins == 1 && ends == 0, "landing produces exactly one begin");

    // Launch it away: exactly one end.
    m2Body_SetLinearVelocity(ball, (m2Vec2){0.0f, 30.0f});
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        m2ContactEvents ev = m2World_GetContactEvents(world);
        begins += ev.beginCount;
        ends += ev.endCount;
    }
    CHECK(begins == 1 && ends == 1, "launch produces exactly one end");

    m2DestroyWorld(world);
}

static void TestDestroyBookending(void)
{
    // M19: destroying a touching body must emit the end event, with the
    // ids as they were - and a between-step destroy must surface in the
    // next drained window, never vanish.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);
    AddSlab(world);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 0.85};
    m2BodyId box = m2CreateBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon unit = m2MakeBox(0.5f, 0.5f);
    m2ShapeId boxShape = m2CreatePolygonShape(box, &sd, &unit);

    int32_t begins = 0;
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        begins += m2World_GetContactEvents(world).beginCount;
    }
    CHECK(begins == 1, "box landed");

    // Destroy BETWEEN steps, while touching.
    uint16_t generationAtDestroy = boxShape.generation;
    m2DestroyBody(box);
    m2World_Step(world, 1.0f / 60.0f, 4);
    m2ContactEvents ev = m2World_GetContactEvents(world);
    CHECK(ev.endCount == 1, "destroy-while-touching emits the end event (M19)");
    if (ev.endCount == 1)
    {
        bool namesBox = ev.endEvents[0].shapeIdA.index1 == boxShape.index1 ||
                        ev.endEvents[0].shapeIdB.index1 == boxShape.index1;
        CHECK(namesBox, "end event names the destroyed shape");
        uint16_t gen = ev.endEvents[0].shapeIdA.index1 == boxShape.index1
                           ? ev.endEvents[0].shapeIdA.generation
                           : ev.endEvents[0].shapeIdB.generation;
        CHECK(gen == generationAtDestroy, "end event carries the pre-destroy generation");
    }

    m2DestroyWorld(world);
}

static void TestRestoreClearsAndReplays(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m2WorldId world = m2CreateWorld(&def);
    AddSlab(world);
    m2ShapeDef sd = m2DefaultShapeDef();
    sd.restitution = 0.6f;
    for (int32_t i = 0; i < 4; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){-2.0 + 1.4 * (double)i, 1.0 + 0.8 * (double)i};
        m2BodyId body = m2CreateBody(world, &bd);
        m2Circle c = {{0.0f, 0.0f}, 0.35f};
        m2CreateCircleShape(body, &sd, &c);
    }

    int32_t size = m2World_SnapshotSize(world);
    void* snap = malloc((size_t)size);
    CHECK(m2World_Snapshot(world, snap, size) == size, "snapshot");

    // Record the event stream (bouncy balls churn begins/ends).
    uint64_t streamA = M2_HASH_INIT;
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        m2ContactEvents ev = m2World_GetContactEvents(world);
        streamA =
            m2Hash64(streamA, ev.beginEvents, ev.beginCount * (int32_t)sizeof(m2ContactBeginEvent));
        streamA = m2Hash64(streamA, ev.endEvents, ev.endCount * (int32_t)sizeof(m2ContactEndEvent));
    }

    CHECK(m2World_Restore(world, snap, size), "restore");
    m2ContactEvents cleared = m2World_GetContactEvents(world);
    CHECK(cleared.beginCount == 0 && cleared.endCount == 0, "restore clears event buffers");

    uint64_t streamB = M2_HASH_INIT;
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        m2ContactEvents ev = m2World_GetContactEvents(world);
        streamB =
            m2Hash64(streamB, ev.beginEvents, ev.beginCount * (int32_t)sizeof(m2ContactBeginEvent));
        streamB = m2Hash64(streamB, ev.endEvents, ev.endCount * (int32_t)sizeof(m2ContactEndEvent));
    }
    CHECK(streamA == streamB, "re-simulation re-emits the identical event stream");

    free(snap);
    m2DestroyWorld(world);
}

static uint64_t EventSweepHash(void)
{
    // Bouncy churn far from the origin: begins and ends every few steps,
    // a destroy wave in the middle (bookended ends included).
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 128;
    def.shapeCapacity = 128;
    m2WorldId world = m2CreateWorld(&def);
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){5.5e5, -0.5};
    m2BodyId floor = m2CreateBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(25.0f, 0.5f);
    m2CreatePolygonShape(floor, &fs, &slab);

    m2BodyId spawned[64];
    int32_t spawnCount = 0;
    uint64_t h = M2_HASH_INIT;
    for (int32_t step = 0; step < 240; ++step)
    {
        if (step % 8 == 0 && spawnCount < 24)
        {
            m2BodyDef bd = m2DefaultBodyDef();
            bd.type = m2_dynamicBody;
            bd.position = (m2Pos2){5.5e5 - 8.0 + 0.71 * (double)spawnCount, 3.0};
            m2BodyId b = m2CreateBody(world, &bd);
            m2ShapeDef sd = m2DefaultShapeDef();
            sd.restitution = 0.5f;
            m2Circle c = {{0.0f, 0.0f}, 0.3f};
            m2CreateCircleShape(b, &sd, &c);
            spawned[spawnCount++] = b;
        }
        if (step == 150)
        {
            for (int32_t d = 0; d < spawnCount; d += 2)
            {
                m2DestroyBody(spawned[d]); // between-step destroy wave
            }
        }
        m2World_Step(world, 1.0f / 60.0f, 4);
        m2ContactEvents ev = m2World_GetContactEvents(world);
        h = m2Hash64(h, ev.beginEvents, ev.beginCount * (int32_t)sizeof(m2ContactBeginEvent));
        h = m2Hash64(h, ev.endEvents, ev.endCount * (int32_t)sizeof(m2ContactEndEvent));
        h = m2Hash64(h, &ev.beginCount, (int32_t)sizeof(int32_t));
        h = m2Hash64(h, &ev.endCount, (int32_t)sizeof(int32_t));
    }
    m2DestroyWorld(world);
    return h;
}

int main(void)
{
    TestLifecycleSymmetry();
    TestDestroyBookending();
    TestRestoreClearsAndReplays();

    uint64_t hash = EventSweepHash();
    printf("M2_EVENT_HASH=%016llx\n", (unsigned long long)hash);

    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
