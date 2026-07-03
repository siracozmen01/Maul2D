// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The threading law (topic-08): worker count is NON-SEMANTIC. The same
// busy scene - stacks, joints, a motorized car, bullets - must produce
// bit-identical trajectories at 1, 2 and 4 workers, and rollback must
// hold under a parallel solver. The 4-worker trajectory is the 14th
// gated line, so the law also holds across compilers and platforms.

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

static m2WorldId BuildBusyScene(int32_t workerCount)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 128;
    def.shapeCapacity = 128;
    def.jointCapacity = 32;
    def.workerCount = workerCount;
    m2WorldId world = m2CreateWorld(&def);

    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){5.5e5, -0.5};
    m2BodyId floor = m2CreateBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    fs.friction = 0.8f;
    m2Polygon slab = m2MakeBox(40.0f, 0.5f);
    m2CreatePolygonShape(floor, &fs, &slab);

    // A pyramid: plenty of contacts sharing bodies = many colors.
    m2ShapeDef bs = m2DefaultShapeDef();
    bs.friction = 0.6f;
    m2Polygon unit = m2MakeBox(0.5f, 0.5f);
    int32_t rows = 8;
    for (int32_t i = 0; i < rows; ++i)
    {
        for (int32_t j = i; j < rows; ++j)
        {
            m2BodyDef bd = m2DefaultBodyDef();
            bd.type = m2_dynamicBody;
            bd.position =
                (m2Pos2){5.5e5 + (double)j - 0.5 * (double)(rows + i), 0.55 + 1.05 * (double)i};
            m2BodyId body = m2CreateBody(world, &bd);
            m2CreatePolygonShape(body, &bs, &unit);
        }
    }

    // A revolute chain off a tower.
    m2BodyDef ad = m2DefaultBodyDef();
    ad.position = (m2Pos2){5.5e5 - 14.0, 8.0};
    m2BodyId anchor = m2CreateBody(world, &ad);
    m2BodyId prev = anchor;
    m2Polygon link = m2MakeBox(0.4f, 0.1f);
    for (int32_t i = 0; i < 6; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){5.5e5 - 14.0 + 0.8 * (double)(i + 1), 8.0};
        m2BodyId linkBody = m2CreateBody(world, &bd);
        m2CreatePolygonShape(linkBody, &bs, &link);
        m2RevoluteJointDef jd = m2DefaultRevoluteJointDef();
        jd.bodyIdA = prev;
        jd.bodyIdB = linkBody;
        jd.localAnchorA =
            prev.index1 == anchor.index1 ? (m2Vec2){0.0f, 0.0f} : (m2Vec2){0.4f, 0.0f};
        jd.localAnchorB = (m2Vec2){-0.4f, 0.0f};
        m2CreateRevoluteJoint(world, &jd);
        prev = linkBody;
    }

    // A motorized car heading for the pyramid.
    m2BodyDef cd = m2DefaultBodyDef();
    cd.type = m2_dynamicBody;
    cd.position = (m2Pos2){5.5e5 - 20.0, 1.0};
    m2BodyId chassis = m2CreateBody(world, &cd);
    m2Polygon deck = m2MakeBox(0.9f, 0.15f);
    m2CreatePolygonShape(chassis, &bs, &deck);
    for (int32_t i = 0; i < 2; ++i)
    {
        m2BodyDef wd = m2DefaultBodyDef();
        wd.type = m2_dynamicBody;
        wd.position = (m2Pos2){5.5e5 - 20.0 + (i == 0 ? -0.6 : 0.6), 0.55};
        m2BodyId wheel = m2CreateBody(world, &wd);
        m2ShapeDef ws = m2DefaultShapeDef();
        ws.friction = 0.9f;
        m2Circle tire = {{0.0f, 0.0f}, 0.3f};
        m2CreateCircleShape(wheel, &ws, &tire);
        m2WheelJointDef ride = m2DefaultWheelJointDef();
        ride.bodyIdA = chassis;
        ride.bodyIdB = wheel;
        ride.localAnchorA = (m2Vec2){i == 0 ? -0.6f : 0.6f, -0.45f};
        ride.localAxisA = (m2Vec2){0.0f, 1.0f};
        ride.hertz = 4.0f;
        ride.dampingRatio = 0.7f;
        ride.enableLimit = true;
        ride.lowerTranslation = -0.2f;
        ride.upperTranslation = 0.2f;
        ride.enableMotor = true;
        ride.motorSpeed = -12.0f;
        ride.maxMotorTorque = 20.0f;
        m2CreateWheelJoint(world, &ride);
    }

    return world;
}

static void FireBullet(m2WorldId world, int32_t step)
{
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.isBullet = true;
    bd.position = (m2Pos2){5.5e5 + 25.0, 2.0 + 0.5 * (double)(step % 3)};
    bd.linearVelocity = (m2Vec2){-90.0f, 0.0f};
    m2BodyId bullet = m2CreateBody(world, &bd);
    if (bullet.index1 != 0)
    {
        m2ShapeDef sd = m2DefaultShapeDef();
        sd.density = 3.0f;
        m2Circle ball = {{0.0f, 0.0f}, 0.15f};
        m2CreateCircleShape(bullet, &sd, &ball);
    }
}

static uint64_t RunTrajectory(int32_t workerCount, int32_t steps)
{
    m2WorldId world = BuildBusyScene(workerCount);
    uint64_t h = M2_HASH_INIT;
    for (int32_t i = 0; i < steps; ++i)
    {
        if (i % 45 == 20)
        {
            FireBullet(world, i);
        }
        m2World_Step(world, 1.0f / 60.0f, 4);
        uint64_t worldHash = m2World_Hash(world);
        h = m2Hash64(h, &worldHash, (int32_t)sizeof(worldHash));
    }
    m2DestroyWorld(world);
    return h;
}

static void TestWorkerCountIsNonSemantic(void)
{
    uint64_t serial = RunTrajectory(1, 150);
    uint64_t two = RunTrajectory(2, 150);
    uint64_t four = RunTrajectory(4, 150);
    CHECK(serial == two, "2 workers match the serial bits");
    CHECK(serial == four, "4 workers match the serial bits");
}

static void TestParallelRollback(void)
{
    // Rollback under a 4-worker solver: same contract as always.
    m2WorldId world = BuildBusyScene(4);
    for (int32_t i = 0; i < 40; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    int32_t size = m2World_SnapshotSize(world);
    void* snap = malloc((size_t)size);
    CHECK(m2World_Snapshot(world, snap, size) == size, "snapshot");

    uint64_t hashes[60];
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        hashes[i] = m2World_Hash(world);
    }
    CHECK(m2World_Restore(world, snap, size), "restore");
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        CHECK(m2World_Hash(world) == hashes[i], "parallel solver replays bit-exactly");
    }
    free(snap);
    m2DestroyWorld(world);
}

int main(void)
{
    TestWorkerCountIsNonSemantic();
    TestParallelRollback();

    uint64_t hash = RunTrajectory(4, 150);
    printf("M2_THREAD_HASH=%016llx\n", (unsigned long long)hash);

    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
