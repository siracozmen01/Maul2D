// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Joint gate: a distance-joint pendulum holds its rod length through the
// swing; a revolute chain keeps its pivots pinned; joints couple sleep
// islands without any contact; destroying a body kills its joints and
// wakes the counterpart; joint impulses replay under rollback; and the
// chain-swing hash crosses CI cells.

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

static m2BodyId AddBall(m2WorldId world, double x, double y, float radius)
{
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){x, y};
    m2BodyId body = m2CreateBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Circle c = {{0.0f, 0.0f}, radius};
    m2CreateCircleShape(body, &sd, &c);
    return body;
}

static double Distance(m2Pos2 a, m2Pos2 b)
{
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    return dx * dx + dy * dy;
}

static void TestDistancePendulum(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);

    m2BodyDef ad = m2DefaultBodyDef();
    ad.position = (m2Pos2){0.0, 5.0};
    m2BodyId anchor = m2CreateBody(world, &ad);
    m2BodyId bob = AddBall(world, 2.0, 5.0, 0.2f); // horizontal start: big swing

    m2DistanceJointDef jd = m2DefaultDistanceJointDef();
    jd.bodyIdA = anchor;
    jd.bodyIdB = bob;
    m2JointId joint = m2CreateDistanceJoint(world, &jd);
    CHECK(m2Joint_IsValid(joint), "distance joint created");

    double worst = 0.0;
    for (int32_t i = 0; i < 300; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        double lenSq = Distance(m2Body_GetPosition(anchor), m2Body_GetPosition(bob));
        double err = lenSq - 4.0; // rod length 2 derived at create
        err = err < 0.0 ? -err : err;
        worst = err > worst ? err : worst;
    }
    // |len^2 - 4| < 0.12 keeps the rod within ~1.5% of its length.
    CHECK(worst < 0.12, "pendulum rod length holds through the swing");

    m2DestroyWorld(world);
}

static void TestRevoluteChain(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m2WorldId world = m2CreateWorld(&def);

    m2BodyDef ad = m2DefaultBodyDef();
    ad.position = (m2Pos2){0.0, 8.0};
    m2BodyId previous = m2CreateBody(world, &ad);

    m2BodyId links[4];
    for (int32_t i = 0; i < 4; ++i)
    {
        links[i] = AddBall(world, 1.0 + (double)i, 8.0, 0.15f);
        m2RevoluteJointDef jd = m2DefaultRevoluteJointDef();
        jd.bodyIdA = previous;
        jd.bodyIdB = links[i];
        jd.localAnchorA = i == 0 ? (m2Vec2){0.5f, 0.0f} : (m2Vec2){0.5f, 0.0f};
        jd.localAnchorB = (m2Vec2){-0.5f, 0.0f};
        CHECK(m2Joint_IsValid(m2CreateRevoluteJoint(world, &jd)), "link joint created");
        previous = links[i];
    }

    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    // Pivot integrity: consecutive links stay anchor-to-anchor (1m apart
    // center to center) within a soft tolerance while swinging.
    for (int32_t i = 1; i < 4; ++i)
    {
        double gapSq = Distance(m2Body_GetPosition(links[i - 1]), m2Body_GetPosition(links[i]));
        CHECK(gapSq > 0.8 && gapSq < 1.2, "revolute pivots hold the chain together");
    }

    m2DestroyWorld(world);
}

static void TestJointsCoupleIslands(void)
{
    // Two balls joined by a rod, never touching each other, resting on
    // separate pillars: they must sleep as one island and wake as one.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m2WorldId world = m2CreateWorld(&def);

    for (int32_t i = 0; i < 2; ++i)
    {
        m2BodyDef pd = m2DefaultBodyDef();
        pd.position = (m2Pos2){3.0 * (double)i, 0.0};
        m2BodyId pillar = m2CreateBody(world, &pd);
        m2ShapeDef sd = m2DefaultShapeDef();
        m2Polygon top = m2MakeBox(0.4f, 0.4f);
        m2CreatePolygonShape(pillar, &sd, &top);
    }
    m2BodyId left = AddBall(world, 0.0, 0.62, 0.2f);
    m2BodyId right = AddBall(world, 3.0, 0.62, 0.2f);
    m2DistanceJointDef jd = m2DefaultDistanceJointDef();
    jd.bodyIdA = left;
    jd.bodyIdB = right;
    m2CreateDistanceJoint(world, &jd);

    for (int32_t i = 0; i < 200; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m2Body_IsAwake(left) && !m2Body_IsAwake(right), "joined pair sleeps together");

    m2Body_SetLinearVelocity(left, (m2Vec2){0.5f, 0.0f});
    m2World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m2Body_IsAwake(right), "wake crosses the joint, no contact needed");

    // Destroy one end: the joint dies, the survivor wakes.
    for (int32_t i = 0; i < 200; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2DestroyBody(left);
    CHECK(m2Body_IsAwake(right), "destroying a jointed body wakes the counterpart");

    m2DestroyWorld(world);
}

static void TestJointRollback(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m2WorldId world = m2CreateWorld(&def);
    m2BodyDef ad = m2DefaultBodyDef();
    ad.position = (m2Pos2){0.0, 6.0};
    m2BodyId anchor = m2CreateBody(world, &ad);
    m2BodyId bob = AddBall(world, 1.5, 6.0, 0.2f);
    m2DistanceJointDef jd = m2DefaultDistanceJointDef();
    jd.bodyIdA = anchor;
    jd.bodyIdB = bob;
    m2CreateDistanceJoint(world, &jd);

    for (int32_t i = 0; i < 40; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4); // mid-swing, impulses loaded
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
        CHECK(m2World_Hash(world) == hashes[i], "pendulum replays bit-exactly");
    }
    free(snap);
    m2DestroyWorld(world);
}

static uint64_t JointSweepHash(void)
{
    // A bridge of revolute links with cargo dropped on it, far from the
    // origin: joint impulses, contacts, and sleep all in one stream.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 128;
    def.shapeCapacity = 128;
    m2WorldId world = m2CreateWorld(&def);

    m2BodyDef ad = m2DefaultBodyDef();
    ad.position = (m2Pos2){-9.1e5, 4.0};
    m2BodyId left = m2CreateBody(world, &ad);
    m2BodyId previous = left;
    for (int32_t i = 0; i < 10; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){-9.1e5 + 0.8 * (double)(i + 1), 4.0};
        m2BodyId plank = m2CreateBody(world, &bd);
        m2ShapeDef sd = m2DefaultShapeDef();
        m2Polygon shape = m2MakeBox(0.4f, 0.1f);
        m2CreatePolygonShape(plank, &sd, &shape);
        m2RevoluteJointDef jd = m2DefaultRevoluteJointDef();
        jd.bodyIdA = previous;
        jd.bodyIdB = plank;
        jd.localAnchorA =
            previous.index1 == left.index1 ? (m2Vec2){0.0f, 0.0f} : (m2Vec2){0.4f, 0.0f};
        jd.localAnchorB = (m2Vec2){-0.4f, 0.0f};
        m2CreateRevoluteJoint(world, &jd);
        previous = plank;
    }
    m2BodyDef rd = m2DefaultBodyDef();
    rd.position = (m2Pos2){-9.1e5 + 8.8, 4.0};
    m2BodyId right = m2CreateBody(world, &rd);
    m2RevoluteJointDef last = m2DefaultRevoluteJointDef();
    last.bodyIdA = previous;
    last.bodyIdB = right;
    last.localAnchorA = (m2Vec2){0.4f, 0.0f};
    last.localAnchorB = (m2Vec2){0.0f, 0.0f};
    m2CreateRevoluteJoint(world, &last);

    uint64_t h = M2_HASH_INIT;
    for (int32_t step = 0; step < 300; ++step)
    {
        if (step % 40 == 10)
        {
            m2BodyDef bd = m2DefaultBodyDef();
            bd.type = m2_dynamicBody;
            bd.position = (m2Pos2){-9.1e5 + 1.5 + 0.9 * (double)(step / 40), 7.0};
            m2BodyId cargo = m2CreateBody(world, &bd);
            m2ShapeDef sd = m2DefaultShapeDef();
            sd.density = 2.0f;
            m2Circle c = {{0.0f, 0.0f}, 0.25f};
            m2CreateCircleShape(cargo, &sd, &c);
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
    TestDistancePendulum();
    TestRevoluteChain();
    TestJointsCoupleIslands();
    TestJointRollback();

    uint64_t hash = JointSweepHash();
    printf("M2_JOINT_HASH=%016llx\n", (unsigned long long)hash);

    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
