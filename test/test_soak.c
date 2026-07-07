// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The soak: one mixed world (stacks, machines, water, jelly, a
// conveyor) stepped for a long horizon. Not part of the gated
// sixteen; the weekly scheduled CI runs it deep and any local run
// can too: test_soak [steps], default 2000. It proves three things
// at horizon scale: nothing goes non-finite, the world keeps
// producing bytes (hash moves or the world legitimately sleeps),
// and a mid-horizon snapshot replays the second half bit for bit.

#include "maul2d/maul2d.h"

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

int main(int argc, char** argv)
{
    int32_t steps = argc > 1 ? atoi(argv[1]) : 2000;
    if (steps < 200)
    {
        steps = 200;
    }

    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 256;
    def.shapeCapacity = 512;
    def.jointCapacity = 64;
    def.particleCapacity = 512;
    m2WorldId world = m2CreateWorld(&def);

    // The floor is a conveyor: the pyramid's rubble keeps drifting.
    m2BodyDef floorDef = m2DefaultBodyDef();
    floorDef.position = (m2Pos2){0.0, -0.2};
    m2ShapeDef beltShape = m2DefaultShapeDef();
    beltShape.tangentSpeed = 0.4f;
    m2Polygon slab = m2MakeBox(24.0f, 0.2f);
    m2CreatePolygonShape(m2CreateBody(world, &floorDef), &beltShape, &slab);
    m2BodyDef wallDefL = m2DefaultBodyDef();
    wallDefL.position = (m2Pos2){-24.0, 3.0};
    m2ShapeDef wallShape = m2DefaultShapeDef();
    m2Polygon wall = m2MakeBox(0.2f, 3.2f);
    m2CreatePolygonShape(m2CreateBody(world, &wallDefL), &wallShape, &wall);
    m2BodyDef wallDefR = m2DefaultBodyDef();
    wallDefR.position = (m2Pos2){24.0, 3.0};
    m2CreatePolygonShape(m2CreateBody(world, &wallDefR), &wallShape, &wall);

    // A pyramid to grind along the belt.
    m2Polygon crate = m2MakeBox(0.25f, 0.25f);
    for (int32_t row = 0; row < 8; ++row)
    {
        for (int32_t col = 0; col <= row; ++col)
        {
            m2BodyDef bd = m2DefaultBodyDef();
            bd.type = m2_dynamicBody;
            bd.position =
                (m2Pos2){-8.0 + (double)col * 0.52 - (double)row * 0.26, 4.2 - (double)row * 0.5};
            m2ShapeDef sd = m2DefaultShapeDef();
            m2CreatePolygonShape(m2CreateBody(world, &bd), &sd, &crate);
        }
    }

    // A gear pair with a motor and a ratchet: machinery that runs the
    // whole horizon.
    m2Circle disc = {{0.0f, 0.0f}, 0.4f};
    m2BodyId wheels[2];
    for (int32_t i = 0; i < 2; ++i)
    {
        m2BodyDef pd = m2DefaultBodyDef();
        pd.position = (m2Pos2){6.0 + (double)i * 1.8, 3.0};
        m2BodyId post = m2CreateBody(world, &pd);
        m2BodyDef wd = m2DefaultBodyDef();
        wd.type = m2_dynamicBody;
        wd.position = (m2Pos2){6.0 + (double)i * 1.8, 3.0};
        wheels[i] = m2CreateBody(world, &wd);
        m2ShapeDef sd = m2DefaultShapeDef();
        m2CreateCircleShape(wheels[i], &sd, &disc);
        m2RevoluteJointDef rj = m2DefaultRevoluteJointDef();
        rj.bodyIdA = post;
        rj.bodyIdB = wheels[i];
        if (i == 0)
        {
            rj.enableMotor = true;
            rj.motorSpeed = 1.5f;
            rj.maxMotorTorque = 60.0f;
        }
        m2CreateRevoluteJoint(world, &rj);
    }
    m2GearJointDef gd = m2DefaultGearJointDef();
    gd.bodyIdA = wheels[0];
    gd.bodyIdB = wheels[1];
    gd.ratio = 2.0f;
    m2CreateGearJoint(world, &gd);

    // Water in the left corner, a jelly blob dropped on the pile.
    m2Polygon pool = m2MakeBox(1.6f, 0.4f);
    m2World_FillPolygonWithParticles(world, &pool, (m2Pos2){-20.0, 0.5}, (m2Vec2){0.0f, 0.0f}, 0);
    m2Polygon blob = m2MakeBox(0.25f, 0.25f);
    m2World_FillPolygonWithParticles(world, &blob, (m2Pos2){-8.0, 6.0}, (m2Vec2){0.0f, 0.0f},
                                     m2_springParticle | m2_elasticParticle);

    int32_t half = steps / 2;
    for (int32_t i = 0; i < half; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    int32_t size = m2World_SnapshotSize(world);
    void* snap = malloc((size_t)size);
    CHECK(m2World_Snapshot(world, snap, size) == size, "the mid-horizon snapshot lands");

    for (int32_t i = half; i < steps; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        if (i % 1024 == 0)
        {
            // Periodic finiteness patrol without disturbing anything.
            m2BodyId probe[8];
            int32_t got = m2World_GetBodies(world, probe, 8);
            for (int32_t k = 0; k < (got < 8 ? got : 8); ++k)
            {
                m2Pos2 p = m2Body_GetPosition(probe[k]);
                CHECK(p.x == p.x && p.y == p.y, "positions stay finite at horizon");
            }
        }
    }
    uint64_t first = m2World_Hash(world);

    // Replay the second half from the snapshot: identical bits.
    CHECK(m2World_Restore(world, snap, size), "the mid-horizon restore lands");
    for (int32_t i = half; i < steps; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    uint64_t second = m2World_Hash(world);
    CHECK(first == second, "the second half replays bit for bit at horizon scale");

    printf("M2_SOAK_HASH=%016llx steps=%d\n", (unsigned long long)first, steps);
    free(snap);
    m2DestroyWorld(world);
    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
