// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Island/sleep gate: a settled pyramid must sleep with zeroed
// velocities and frozen positions; waking is island-transitive; a
// kinematic sweep into a sleeping stack wakes it (RT1-STAB-1 end to
// end); sleep counters survive rollback bit-exactly; and the sleep and
// wake cycle hash crosses CI cells.

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

static m2BodyId AddSlab(m2WorldId world, double y)
{
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, y};
    m2BodyId body = m2CreateBody(world, &fd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(20.0f, 0.5f);
    m2CreatePolygonShape(body, &sd, &slab);
    return body;
}

static m2BodyId AddBox(m2WorldId world, double x, double y)
{
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){x, y};
    m2BodyId body = m2CreateBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon unit = m2MakeBox(0.5f, 0.5f);
    m2CreatePolygonShape(body, &sd, &unit);
    return body;
}

static void TestPyramidSleeps(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 32;
    def.shapeCapacity = 32;
    m2WorldId world = m2CreateWorld(&def);
    AddSlab(world, -0.5);

    m2BodyId boxes[6];
    int32_t count = 0;
    for (int32_t row = 0; row < 3; ++row)
    {
        for (int32_t col = row; col < 3; ++col)
        {
            boxes[count++] = AddBox(world, (double)col - 0.5 * (double)(3 + row) + 0.5,
                                    0.55 + 1.02 * (double)row);
        }
    }

    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    for (int32_t i = 0; i < count; ++i)
    {
        CHECK(!m2Body_IsAwake(boxes[i]), "pyramid box sleeps");
        m2Vec2 v = m2Body_GetLinearVelocity(boxes[i]);
        CHECK(v.x == 0.0f && v.y == 0.0f, "sleeping velocity zeroed");
    }

    // Frozen: positions must not change at all while asleep.
    m2Pos2 before = m2Body_GetPosition(boxes[0]);
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2Pos2 after = m2Body_GetPosition(boxes[0]);
    CHECK(before.x == after.x && before.y == after.y, "sleeping bodies are frozen");

    // Island-transitive wake: shove the top box; the whole pile wakes.
    m2Body_SetLinearVelocity(boxes[count - 1], (m2Vec2){2.0f, 0.0f});
    m2World_Step(world, 1.0f / 60.0f, 4);
    int32_t awakeCount = 0;
    for (int32_t i = 0; i < count; ++i)
    {
        awakeCount += m2Body_IsAwake(boxes[i]) ? 1 : 0;
    }
    CHECK(awakeCount == count, "waking is island-transitive");

    m2DestroyWorld(world);
}

static void TestKinematicWakesSleepers(void)
{
    // RT1-STAB-1, end to end: a stack sleeps, a kinematic platform
    // sweeps into it, the stack must wake and get pushed.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m2WorldId world = m2CreateWorld(&def);
    AddSlab(world, -0.5);
    m2BodyId a = AddBox(world, 0.0, 0.55);
    m2BodyId b = AddBox(world, 0.0, 1.57);

    for (int32_t i = 0; i < 200; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m2Body_IsAwake(a) && !m2Body_IsAwake(b), "stack asleep before the sweep");

    m2BodyDef kd = m2DefaultBodyDef();
    kd.type = m2_kinematicBody;
    kd.position = (m2Pos2){-6.0, 0.55};
    kd.linearVelocity = (m2Vec2){8.0f, 0.0f};
    m2BodyId sweeper = m2CreateBody(world, &kd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon blade = m2MakeBox(0.5f, 0.4f);
    m2CreatePolygonShape(sweeper, &sd, &blade);

    bool woke = false;
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        if (m2Body_IsAwake(a))
        {
            woke = true;
            break;
        }
    }
    CHECK(woke, "kinematic sweep wakes the sleeping stack (RT1-STAB-1)");
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(a).x > 0.2, "woken box actually gets pushed");

    m2DestroyWorld(world);
}

static void TestSleepRollback(void)
{
    // Snapshot mid-doze (counters running), restore, replay: the sleep
    // moment must land on the identical step - the reference CI's own
    // oracle, now under rollback.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m2WorldId world = m2CreateWorld(&def);
    AddSlab(world, -0.5);
    m2BodyId box = AddBox(world, 0.0, 1.2);

    for (int32_t i = 0; i < 45; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4); // landed, counters counting
    }
    CHECK(m2Body_IsAwake(box), "still dozing off at snapshot time");

    int32_t size = m2World_SnapshotSize(world);
    void* snap = malloc((size_t)size);
    CHECK(m2World_Snapshot(world, snap, size) == size, "snapshot");

    int32_t sleepStepA = -1;
    uint64_t hashes[120];
    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        hashes[i] = m2World_Hash(world);
        if (sleepStepA < 0 && !m2Body_IsAwake(box))
        {
            sleepStepA = i;
        }
    }
    CHECK(sleepStepA >= 0, "box eventually sleeps");

    CHECK(m2World_Restore(world, snap, size), "restore");
    int32_t sleepStepB = -1;
    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        CHECK(m2World_Hash(world) == hashes[i], "sleep cycle replays bit-exactly");
        if (sleepStepB < 0 && !m2Body_IsAwake(box))
        {
            sleepStepB = i;
        }
    }
    CHECK(sleepStepA == sleepStepB, "sleep lands on the identical step after rollback");

    free(snap);
    m2DestroyWorld(world);
}

static uint64_t IslandSweepHash(void)
{
    // Sleep/wake churn far from the origin: piles doze, a patrolling
    // kinematic wakes them on a cycle.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 128;
    def.shapeCapacity = 128;
    m2WorldId world = m2CreateWorld(&def);

    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){3.3e5, -0.5};
    m2BodyId floor = m2CreateBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(25.0f, 0.5f);
    m2CreatePolygonShape(floor, &fs, &slab);

    for (int32_t i = 0; i < 24; ++i)
    {
        AddBox(world, 3.3e5 - 10.0 + 0.9 * (double)i, 0.55 + 1.1 * (double)(i % 3));
    }

    m2BodyDef kd = m2DefaultBodyDef();
    kd.type = m2_kinematicBody;
    kd.position = (m2Pos2){3.3e5 - 16.0, 0.5};
    m2BodyId patrol = m2CreateBody(world, &kd);
    m2ShapeDef ps = m2DefaultShapeDef();
    m2Polygon blade = m2MakeBox(0.4f, 0.3f);
    m2CreatePolygonShape(patrol, &ps, &blade);

    uint64_t h = M2_HASH_INIT;
    for (int32_t step = 0; step < 420; ++step)
    {
        if (step == 200)
        {
            m2Body_SetLinearVelocity(patrol, (m2Vec2){6.0f, 0.0f}); // start the sweep
        }
        m2World_Step(world, 1.0f / 60.0f, 4);
        uint64_t worldHash = m2World_Hash(world);
        h = m2Hash64(h, &worldHash, (int32_t)sizeof(worldHash));
    }
    m2DestroyWorld(world);
    return h;
}

int main(void)
{
    TestPyramidSleeps();
    TestKinematicWakesSleepers();
    TestSleepRollback();

    uint64_t hash = IslandSweepHash();
    printf("M2_ISLAND_HASH=%016llx\n", (unsigned long long)hash);

    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
