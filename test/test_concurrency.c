// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The multi-world concurrency proof (integration audit B2): two
// DISTINCT worlds stepped on two host threads land bit-identical
// to their serial twins. One writer per world stays the law;
// create/destroy stay host-serialized. POSIX-only referee; the
// contract holds everywhere.
#include "maul2d/maul2d.h"

#include <pthread.h>
#include <stdio.h>
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

static m2WorldId BuildWorld(int32_t seed)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 128;
    def.shapeCapacity = 128;
    m2WorldId world = m2CreateWorld(&def);
    m2BodyDef gd = m2DefaultBodyDef();
    m2BodyId ground = m2CreateBody(world, &gd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(8.0f, 0.3f);
    m2CreatePolygonShape(ground, &sd, &slab);
    for (int32_t i = 0; i < 80; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){0.8 * (double)(i % 10) - 3.6 + 0.01 * (double)seed,
                               0.8 + 0.7 * (double)(i / 10)};
        m2BodyId b = m2CreateBody(world, &bd);
        if (i % 2 == 0)
        {
            m2Polygon box = m2MakeBox(0.3f, 0.3f);
            m2CreatePolygonShape(b, &sd, &box);
        }
        else
        {
            m2Circle c = {{0.0f, 0.0f}, 0.3f};
            m2CreateCircleShape(b, &sd, &c);
        }
    }
    return world;
}

typedef struct StepJob
{
    m2WorldId world;
    int32_t steps;
} StepJob;

static void* StepMain(void* arg)
{
    StepJob* job = (StepJob*)arg;
    for (int32_t i = 0; i < job->steps; ++i)
    {
        m2World_Step(job->world, 1.0f / 60.0f, 4);
    }
    return NULL;
}

static void TestParallelWorldsMatchSerial(void)
{
    m2WorldId serialA = BuildWorld(1);
    m2WorldId serialB = BuildWorld(2);
    for (int32_t i = 0; i < 300; ++i)
    {
        m2World_Step(serialA, 1.0f / 60.0f, 4);
        m2World_Step(serialB, 1.0f / 60.0f, 4);
    }
    uint64_t wantA = m2World_Hash(serialA);
    uint64_t wantB = m2World_Hash(serialB);
    m2DestroyWorld(serialA);
    m2DestroyWorld(serialB);

    m2WorldId parA = BuildWorld(1);
    m2WorldId parB = BuildWorld(2);
    StepJob jobA = {parA, 300};
    StepJob jobB = {parB, 300};
    pthread_t tA;
    pthread_t tB;
    pthread_create(&tA, NULL, StepMain, &jobA);
    pthread_create(&tB, NULL, StepMain, &jobB);
    pthread_join(tA, NULL);
    pthread_join(tB, NULL);
    CHECK(m2World_Hash(parA) == wantA, "world A is thread-placement blind");
    CHECK(m2World_Hash(parB) == wantB, "world B is thread-placement blind");
    m2DestroyWorld(parA);
    m2DestroyWorld(parB);
}

int main(void)
{
    TestParallelWorldsMatchSerial();
    if (s_failures == 0)
    {
        printf("test_concurrency: all green\n");
        return 0;
    }
    return 1;
}
