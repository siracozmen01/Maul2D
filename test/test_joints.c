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
        // This scene predates collideConnected and its gap band assumes
        // adjacent links bounce off each other; opt into that explicitly.
        jd.collideConnected = true;
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

static float BodyAngle(m2BodyId body)
{
    m2Rot q = m2Body_GetRotation(body);
    return m2Atan2(q.s, q.c);
}

static void TestRevoluteMotor(void)
{
    // A strong motor spins a pinned wheel up to its target speed; a
    // starved motor cannot (the torque budget is real).
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);

    m2BodyDef ad = m2DefaultBodyDef();
    ad.position = (m2Pos2){0.0, 3.0};
    m2BodyId base = m2CreateBody(world, &ad);
    m2BodyId wheel = AddBall(world, 0.0, 3.0, 0.5f);
    m2BodyId starvedWheel = AddBall(world, 5.0, 3.0, 0.5f);
    m2BodyDef ad2 = m2DefaultBodyDef();
    ad2.position = (m2Pos2){5.0, 3.0};
    m2BodyId base2 = m2CreateBody(world, &ad2);

    m2RevoluteJointDef jd = m2DefaultRevoluteJointDef();
    jd.bodyIdA = base;
    jd.bodyIdB = wheel;
    jd.enableMotor = true;
    jd.motorSpeed = 4.0f;
    jd.maxMotorTorque = 50.0f;
    m2CreateRevoluteJoint(world, &jd);

    m2RevoluteJointDef weak = m2DefaultRevoluteJointDef();
    weak.bodyIdA = base2;
    weak.bodyIdB = starvedWheel;
    weak.enableMotor = true;
    weak.motorSpeed = 4.0f;
    weak.maxMotorTorque = 0.0001f;
    m2CreateRevoluteJoint(world, &weak);

    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    float w = m2Body_GetAngularVelocity(wheel);
    CHECK(w > 3.9f && w < 4.1f, "motor reaches target speed");
    float starved = m2Body_GetAngularVelocity(starvedWheel);
    CHECK(starved < 1.0f, "starved motor stays under budget");

    m2DestroyWorld(world);
}

static void TestRevoluteLimit(void)
{
    // A horizontal rod on a limited hinge: gravity swings it down, the
    // lower limit catches it and it rests there without sagging through.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);

    m2BodyDef ad = m2DefaultBodyDef();
    ad.position = (m2Pos2){0.0, 5.0};
    m2BodyId pivot = m2CreateBody(world, &ad);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.6, 5.0};
    m2BodyId rod = m2CreateBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon bar = m2MakeBox(0.6f, 0.05f);
    m2CreatePolygonShape(rod, &sd, &bar);

    m2RevoluteJointDef jd = m2DefaultRevoluteJointDef();
    jd.bodyIdA = pivot;
    jd.bodyIdB = rod;
    jd.localAnchorB = (m2Vec2){-0.6f, 0.0f};
    jd.enableLimit = true;
    jd.lowerAngle = -0.5f;
    jd.upperAngle = 0.5f;
    m2CreateRevoluteJoint(world, &jd);

    float minAngle = 0.0f;
    for (int32_t i = 0; i < 400; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        float angle = BodyAngle(rod);
        minAngle = angle < minAngle ? angle : minAngle;
    }
    CHECK(minAngle > -0.55f, "limit never overshoots badly");
    float rest = BodyAngle(rod);
    CHECK(rest > -0.55f && rest < -0.45f, "rod rests on the lower limit");

    m2DestroyWorld(world);
}

static void TestPrismaticSlider(void)
{
    // Gravity elevator: the axis lock kills sideways drift and spin,
    // the lower translation limit catches the fall.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);

    m2BodyDef ad = m2DefaultBodyDef();
    ad.position = (m2Pos2){0.0, 4.0};
    m2BodyId frame = m2CreateBody(world, &ad);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 4.0};
    m2BodyId platform = m2CreateBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon deck = m2MakeBox(0.5f, 0.1f);
    m2CreatePolygonShape(platform, &sd, &deck);

    m2PrismaticJointDef jd = m2DefaultPrismaticJointDef();
    jd.bodyIdA = frame;
    jd.bodyIdB = platform;
    jd.localAxisA = (m2Vec2){0.0f, 1.0f};
    jd.enableLimit = true;
    jd.lowerTranslation = -1.5f;
    jd.upperTranslation = 2.0f;
    m2CreatePrismaticJoint(world, &jd);

    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2Pos2 pos = m2Body_GetPosition(platform);
    CHECK(pos.y > 4.0 - 1.55 && pos.y < 4.0 - 1.42, "platform rests on the lower stop");
    CHECK(pos.x > -0.01 && pos.x < 0.01, "axis lock kills sideways drift");
    m2Rot q = m2Body_GetRotation(platform);
    CHECK(q.s > -0.01f && q.s < 0.01f, "angle lock kills spin");

    m2DestroyWorld(world);
}

static void TestPrismaticMotor(void)
{
    // The motor hoists the platform against gravity to the upper stop
    // and holds it there.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);

    m2BodyDef ad = m2DefaultBodyDef();
    ad.position = (m2Pos2){0.0, 1.0};
    m2BodyId frame = m2CreateBody(world, &ad);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 1.0};
    m2BodyId platform = m2CreateBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon deck = m2MakeBox(0.5f, 0.1f);
    m2CreatePolygonShape(platform, &sd, &deck);

    m2PrismaticJointDef jd = m2DefaultPrismaticJointDef();
    jd.bodyIdA = frame;
    jd.bodyIdB = platform;
    jd.localAxisA = (m2Vec2){0.0f, 1.0f};
    jd.enableMotor = true;
    jd.motorSpeed = 1.5f;
    jd.maxMotorForce = 50.0f;
    jd.enableLimit = true;
    jd.lowerTranslation = -0.5f;
    jd.upperTranslation = 2.0f;
    m2CreatePrismaticJoint(world, &jd);

    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2Vec2 vel = m2Body_GetLinearVelocity(platform);
    CHECK(vel.y > 1.3f && vel.y < 1.7f, "elevator cruises at motor speed");

    for (int32_t i = 0; i < 200; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2Pos2 pos = m2Body_GetPosition(platform);
    CHECK(pos.y > 1.0 + 1.95 && pos.y < 1.0 + 2.05, "elevator holds at the upper stop");

    m2DestroyWorld(world);
}

static void TestMotorRollback(void)
{
    // Motors, limits and the slider all carry warm-start accumulators:
    // snapshot mid-drive, replay, demand bit-exact hashes.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m2WorldId world = m2CreateWorld(&def);

    m2BodyDef ad = m2DefaultBodyDef();
    ad.position = (m2Pos2){0.0, 3.0};
    m2BodyId tower = m2CreateBody(world, &ad);
    m2BodyDef bladeDef = m2DefaultBodyDef();
    bladeDef.type = m2_dynamicBody;
    bladeDef.position = (m2Pos2){0.0, 3.0};
    m2BodyId blade = m2CreateBody(world, &bladeDef);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon bar = m2MakeBox(0.8f, 0.05f);
    m2CreatePolygonShape(blade, &sd, &bar);
    m2RevoluteJointDef mill = m2DefaultRevoluteJointDef();
    mill.bodyIdA = tower;
    mill.bodyIdB = blade;
    mill.enableMotor = true;
    mill.motorSpeed = 3.0f;
    mill.maxMotorTorque = 20.0f;
    mill.enableLimit = false;
    m2CreateRevoluteJoint(world, &mill);

    m2BodyDef pd = m2DefaultBodyDef();
    pd.type = m2_dynamicBody;
    pd.position = (m2Pos2){3.0, 3.0};
    m2BodyId piston = m2CreateBody(world, &pd);
    m2Polygon head = m2MakeBox(0.2f, 0.2f);
    m2CreatePolygonShape(piston, &sd, &head);
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){3.0, 3.0};
    m2BodyId cylinder = m2CreateBody(world, &fd);
    m2PrismaticJointDef rig = m2DefaultPrismaticJointDef();
    rig.bodyIdA = cylinder;
    rig.bodyIdB = piston;
    rig.localAxisA = (m2Vec2){0.0f, 1.0f};
    rig.enableMotor = true;
    rig.motorSpeed = -0.8f;
    rig.maxMotorForce = 30.0f;
    rig.enableLimit = true;
    rig.lowerTranslation = -1.0f;
    rig.upperTranslation = 1.0f;
    m2CreatePrismaticJoint(world, &rig);

    for (int32_t i = 0; i < 30; ++i)
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
        CHECK(m2World_Hash(world) == hashes[i], "motorized machinery replays bit-exactly");
    }

    free(snap);
    m2DestroyWorld(world);
}

static void TestWeldRigid(void)
{
    // Two boxes welded into a bar, dropped onto a slab: the weld holds
    // through the impact and the pair lands as one rigid piece.
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

    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon box = m2MakeBox(0.4f, 0.4f);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){-0.4, 3.0};
    m2BodyId left = m2CreateBody(world, &bd);
    m2CreatePolygonShape(left, &sd, &box);
    bd.position = (m2Pos2){0.4, 3.0};
    m2BodyId right = m2CreateBody(world, &bd);
    m2CreatePolygonShape(right, &sd, &box);

    m2WeldJointDef wd = m2DefaultWeldJointDef();
    wd.bodyIdA = left;
    wd.bodyIdB = right;
    wd.localAnchorA = (m2Vec2){0.4f, 0.0f};
    wd.localAnchorB = (m2Vec2){-0.4f, 0.0f};
    m2CreateWeldJoint(world, &wd);

    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    float relAngle = BodyAngle(right) - BodyAngle(left);
    CHECK(relAngle > -0.01f && relAngle < 0.01f, "weld holds the relative angle");
    m2Pos2 pl = m2Body_GetPosition(left);
    m2Pos2 pr = m2Body_GetPosition(right);
    double gap = pr.x - pl.x;
    CHECK(gap > 0.78 && gap < 0.82, "weld holds the separation");
    CHECK(pl.y > 0.35 && pl.y < 0.45, "welded bar rests on the slab");

    m2DestroyWorld(world);
}

static void TestWheelSuspension(void)
{
    // A chassis on one sprung wheel: drop it, the spring absorbs the
    // fall and the chassis settles above the wheel inside the travel
    // limits instead of slamming rigidly.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);

    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = m2CreateBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(20.0f, 0.5f);
    m2CreatePolygonShape(floor, &fs, &slab);

    m2BodyDef cd = m2DefaultBodyDef();
    cd.type = m2_dynamicBody;
    cd.position = (m2Pos2){0.0, 2.5};
    m2BodyId chassis = m2CreateBody(world, &cd);
    m2ShapeDef cs = m2DefaultShapeDef();
    cs.density = 2.0f;
    m2Polygon deck = m2MakeBox(0.8f, 0.15f);
    m2CreatePolygonShape(chassis, &cs, &deck);

    m2BodyId wheel = AddBall(world, 0.0, 1.9, 0.3f);

    m2WheelJointDef wd = m2DefaultWheelJointDef();
    wd.bodyIdA = chassis;
    wd.bodyIdB = wheel;
    wd.localAnchorA = (m2Vec2){0.0f, -0.6f};
    wd.localAxisA = (m2Vec2){0.0f, 1.0f};
    wd.hertz = 3.0f;
    wd.dampingRatio = 0.7f;
    wd.enableLimit = true;
    wd.lowerTranslation = -0.25f;
    wd.upperTranslation = 0.25f;
    m2CreateWheelJoint(world, &wd);

    for (int32_t i = 0; i < 300; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2Pos2 wheelPos = m2Body_GetPosition(wheel);
    m2Pos2 chassisPos = m2Body_GetPosition(chassis);
    CHECK(wheelPos.y > 0.25 && wheelPos.y < 0.35, "wheel rests on the ground");
    double ride = chassisPos.y - wheelPos.y;
    CHECK(ride > 0.3 && ride < 0.9, "spring carries the chassis inside travel");
    CHECK(chassisPos.x > -0.05 && chassisPos.x < 0.05, "suspension does not walk sideways");

    m2DestroyWorld(world);
}

static void TestWheelCar(void)
{
    // The showcase: a two-wheel car drives itself with wheel-joint
    // motors - suspension, friction and the motor all working together.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);

    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId road = m2CreateBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    fs.friction = 0.9f;
    m2Polygon slab = m2MakeBox(60.0f, 0.5f);
    m2CreatePolygonShape(road, &fs, &slab);

    m2BodyDef cd = m2DefaultBodyDef();
    cd.type = m2_dynamicBody;
    cd.position = (m2Pos2){0.0, 1.0};
    m2BodyId chassis = m2CreateBody(world, &cd);
    m2ShapeDef cs = m2DefaultShapeDef();
    cs.density = 1.0f;
    m2Polygon deck = m2MakeBox(0.9f, 0.15f);
    m2CreatePolygonShape(chassis, &cs, &deck);

    m2BodyId wheels[2];
    double xs[2] = {-0.6, 0.6};
    for (int32_t i = 0; i < 2; ++i)
    {
        wheels[i] = AddBall(world, xs[i], 0.55, 0.3f);
        m2WheelJointDef wd = m2DefaultWheelJointDef();
        wd.bodyIdA = chassis;
        wd.bodyIdB = wheels[i];
        wd.localAnchorA = (m2Vec2){(float)xs[i], -0.45f};
        wd.localAxisA = (m2Vec2){0.0f, 1.0f};
        wd.hertz = 4.0f;
        wd.dampingRatio = 0.7f;
        wd.enableLimit = true;
        wd.lowerTranslation = -0.2f;
        wd.upperTranslation = 0.2f;
        wd.enableMotor = true;
        wd.motorSpeed = -10.0f; // clockwise spin rolls the car +x
        wd.maxMotorTorque = 15.0f;
        m2CreateWheelJoint(world, &wd);
    }

    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4); // settle onto the road
    }
    double startX = m2Body_GetPosition(chassis).x;
    for (int32_t i = 0; i < 300; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    double traveled = m2Body_GetPosition(chassis).x - startX;
    CHECK(traveled > 3.0, "the car drives itself forward");
    m2Rot q = m2Body_GetRotation(chassis);
    CHECK(q.c > 0.95f, "chassis stays upright while driving");

    m2DestroyWorld(world);
}

static void TestJointRuntimeTuning(void)
{
    // Setters retarget a running motor, toggle limits, and wake
    // sleeping machines.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);

    m2BodyDef ad = m2DefaultBodyDef();
    ad.position = (m2Pos2){0.0, 3.0};
    m2BodyId base = m2CreateBody(world, &ad);
    m2BodyId wheel = AddBall(world, 0.0, 3.0, 0.5f);
    m2RevoluteJointDef jd = m2DefaultRevoluteJointDef();
    jd.bodyIdA = base;
    jd.bodyIdB = wheel;
    jd.enableMotor = true;
    jd.motorSpeed = 3.0f;
    jd.maxMotorTorque = 50.0f;
    m2JointId motor = m2CreateRevoluteJoint(world, &jd);

    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    float w1 = m2Body_GetAngularVelocity(wheel);
    CHECK(w1 > 2.9f && w1 < 3.1f, "initial motor speed reached");

    m2Joint_SetMotorSpeed(motor, -5.0f);
    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    float w2 = m2Body_GetAngularVelocity(wheel);
    CHECK(w2 > -5.1f && w2 < -4.9f, "retargeted motor speed reached");

    m2Joint_EnableMotor(motor, false);
    m2Joint_EnableLimit(motor, true);
    m2Joint_SetLimits(motor, -0.2f, 0.2f);
    bool settled = false;
    for (int32_t i = 0; i < 300 && !settled; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        settled = !m2Body_IsAwake(wheel);
    }
    CHECK(settled, "limited free wheel eventually sleeps inside its limits");

    // A setter on a sleeping machine wakes it.
    m2Joint_SetMotorSpeed(motor, 1.0f);
    m2Joint_EnableMotor(motor, true);
    m2World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m2Body_IsAwake(wheel), "runtime tuning wakes the machine");

    m2DestroyWorld(world);
}

static void TestSoftWeld(void)
{
    // Twin cantilevers off static walls: the rigid weld holds its
    // angle, the angular-soft weld droops under the same load like the
    // spring it now is.
    m2WorldDef def = m2DefaultWorldDef();
    def.gravity = (m2Vec2){0.0f, -10.0f};
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);

    for (int32_t soft = 0; soft < 2; ++soft)
    {
        double x = soft == 0 ? 0.0 : 10.0;
        m2BodyDef wd = m2DefaultBodyDef();
        wd.position = (m2Pos2){x, 4.0};
        m2BodyId wall = m2CreateBody(world, &wd);
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){x + 0.9, 4.0};
        m2BodyId beam = m2CreateBody(world, &bd);
        m2ShapeDef sd = m2DefaultShapeDef();
        sd.density = 2.0f;
        m2Polygon bar = m2MakeBox(0.9f, 0.06f);
        m2CreatePolygonShape(beam, &sd, &bar);
        m2WeldJointDef weld = m2DefaultWeldJointDef();
        weld.bodyIdA = wall;
        weld.bodyIdB = beam;
        weld.localAnchorB = (m2Vec2){-0.9f, 0.0f};
        if (soft == 1)
        {
            weld.angularHertz = 1.5f;
            weld.angularDampingRatio = 0.3f;
        }
        m2CreateWeldJoint(world, &weld);
    }

    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }

    // Compare the beam tips by looking the bodies up via queries.
    m2RayCastResult rigid = m2World_CastRayClosest(world, (m2Pos2){1.7, 6.0}, (m2Vec2){0.0f, -6.0f},
                                                   m2DefaultQueryFilter());
    m2RayCastResult soft = m2World_CastRayClosest(world, (m2Pos2){11.7, 6.0}, (m2Vec2){0.0f, -6.0f},
                                                  m2DefaultQueryFilter());
    CHECK(rigid.hit, "rigid beam tip is where it was built");
    CHECK(rigid.point.y > 3.85, "the rigid weld holds its angle");
    CHECK(!soft.hit || soft.point.y < rigid.point.y - 0.05,
          "the angular-soft weld droops under the same load");

    m2DestroyWorld(world);
}

// The getters hand back the same numbers the scissors compare: read
// the load, set a limit just under it, and the joint MUST snap; set
// one well over it and it MUST hold. Plus analytic checks: a hanging
// crate reads its own weight, a cantilever reads its lever torque.
static void TestReactionGetters(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.jointCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);

    m2JointId ropes[2];
    for (int32_t i = 0; i < 2; ++i)
    {
        double x = (double)i * 6.0;
        m2BodyDef ad = m2DefaultBodyDef();
        ad.position = (m2Pos2){x, 6.0};
        m2BodyId hook = m2CreateBody(world, &ad);
        m2BodyDef cd = m2DefaultBodyDef();
        cd.type = m2_dynamicBody;
        cd.position = (m2Pos2){x, 4.0};
        m2BodyId crate = m2CreateBody(world, &cd);
        m2ShapeDef sd = m2DefaultShapeDef();
        sd.density = 5.0f;
        m2Polygon box = m2MakeBox(0.5f, 0.5f);
        m2CreatePolygonShape(crate, &sd, &box);
        m2DistanceJointDef jd = m2DefaultDistanceJointDef();
        jd.bodyIdA = hook;
        jd.bodyIdB = crate;
        ropes[i] = m2CreateDistanceJoint(world, &jd);
    }
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    // Load = m*g = 5 * 1 * 10 = 50 N steady.
    float load = m2Joint_GetReactionForce(ropes[0]);
    CHECK(load > 45.0f && load < 55.0f, "hanging crate reads its own weight");
    CHECK(m2Joint_GetReactionTorque(ropes[0]) == 0.0f, "a rope carries no torque");

    // Agreement with the scissors, both directions.
    m2Joint_SetBreakLimits(ropes[0], load * 0.95f, 0.0f);
    m2Joint_SetBreakLimits(ropes[1], load * 2.0f, 0.0f);
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m2Joint_IsValid(ropes[0]), "a limit under the reading snaps");
    CHECK(m2Joint_IsValid(ropes[1]), "a limit over the reading holds");
    CHECK(m2Joint_GetReactionForce(ropes[0]) == 0.0f, "a dead id reads zero");

    // Cantilever: beam mass 4*2*0.16 = 1.28 kg, COM one meter out from
    // the anchor: torque = 12.8 N*m, force = the 12.8 N weight.
    m2BodyDef wd = m2DefaultBodyDef();
    wd.position = (m2Pos2){12.0, 4.0};
    m2BodyId wall = m2CreateBody(world, &wd);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){13.0, 4.0};
    m2BodyId beam = m2CreateBody(world, &bd);
    m2ShapeDef bs = m2DefaultShapeDef();
    bs.density = 4.0f;
    m2Polygon bar = m2MakeBox(1.0f, 0.08f);
    m2CreatePolygonShape(beam, &bs, &bar);
    m2WeldJointDef weld = m2DefaultWeldJointDef();
    weld.bodyIdA = wall;
    weld.bodyIdB = beam;
    weld.localAnchorB = (m2Vec2){-1.0f, 0.0f};
    m2JointId cantilever = m2CreateWeldJoint(world, &weld);
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    float tipTorque = m2Joint_GetReactionTorque(cantilever);
    float holdForce = m2Joint_GetReactionForce(cantilever);
    CHECK(tipTorque > 10.0f && tipTorque < 16.0f, "cantilever reads its lever torque");
    CHECK(holdForce > 10.0f && holdForce < 16.0f, "and carries the beam weight");

    m2DestroyWorld(world);
}

// collideConnected (parity sprint 3a): the reference default is that
// jointed bodies pass through each other; the flag restores contact,
// and the filter joint is nothing but this switch with a lifetime.
static void TestCollideConnectedAndFilterJoint(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.jointCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);
    m2BodyDef gd = m2DefaultBodyDef();
    gd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = m2CreateBody(world, &gd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(12.0f, 0.5f);
    m2CreatePolygonShape(floor, &fs, &slab);

    // A crate resting on a pedestal; a filter joint makes the pedestal
    // intangible to it and the crate falls through to the floor.
    m2BodyDef pd = m2DefaultBodyDef();
    pd.position = (m2Pos2){0.0, 0.5};
    m2BodyId pedestal = m2CreateBody(world, &pd);
    m2Polygon top = m2MakeBox(1.0f, 0.5f);
    m2CreatePolygonShape(pedestal, &fs, &top);
    m2BodyDef cd = m2DefaultBodyDef();
    cd.type = m2_dynamicBody;
    cd.position = (m2Pos2){0.0, 1.45};
    m2BodyId crate = m2CreateBody(world, &cd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon unit = m2MakeBox(0.4f, 0.4f);
    m2CreatePolygonShape(crate, &sd, &unit);
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(crate).y > 1.2, "the crate rests on the pedestal");

    m2FilterJointDef fj = m2DefaultFilterJointDef();
    fj.bodyIdA = pedestal;
    fj.bodyIdB = crate;
    m2JointId ghostPact = m2CreateFilterJoint(world, &fj);
    CHECK(m2Joint_GetType(ghostPact) == m2_filterJoint, "the pact reads as a filter joint");
    CHECK(!m2Joint_GetCollideConnected(ghostPact), "and never collides");
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(crate).y < 0.9, "the crate falls through the pedestal");
    CHECK(m2Body_GetPosition(crate).y > 0.2, "and lands on the floor");
    CHECK(m2Joint_GetReactionForce(ghostPact) == 0.0f, "a filter joint carries nothing");

    // Destroy the pact: collision returns and the crate pops back out
    // on top the next time it is dropped onto the pedestal.
    m2DestroyJoint(ghostPact);
    m2Body_SetTransform(crate, (m2Pos2){0.0, 2.0}, (m2Rot){1.0f, 0.0f});
    m2Body_SetLinearVelocity(crate, (m2Vec2){0.0f, 0.0f});
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(crate).y > 1.2, "with the pact gone it rests on top again");

    // The flag on a real joint: two touching crates tied by a slack
    // rope. Default false lets them interpenetrate; true keeps them
    // apart. Same scene, one flag.
    for (int32_t pass = 0; pass < 2; ++pass)
    {
        m2BodyDef ad = m2DefaultBodyDef();
        ad.type = m2_dynamicBody;
        ad.position = (m2Pos2){6.0 + 4.0 * (double)pass, 0.45};
        m2BodyId left = m2CreateBody(world, &ad);
        m2CreatePolygonShape(left, &sd, &unit);
        m2BodyDef bd2 = ad;
        bd2.position = (m2Pos2){6.0 + 4.0 * (double)pass, 1.25};
        m2BodyId right = m2CreateBody(world, &bd2);
        m2CreatePolygonShape(right, &sd, &unit);
        m2DistanceJointDef dj = m2DefaultDistanceJointDef();
        dj.bodyIdA = left;
        dj.bodyIdB = right;
        dj.length = 0.1f; // a short soft rod pulling them together
        dj.hertz = 5.0f;
        dj.dampingRatio = 0.7f;
        dj.collideConnected = pass == 1;
        m2CreateDistanceJoint(world, &dj);
        for (int32_t i = 0; i < 120; ++i)
        {
            m2World_Step(world, 1.0f / 60.0f, 4);
        }
        double gap = m2Body_GetPosition(right).y - m2Body_GetPosition(left).y;
        if (pass == 0)
        {
            CHECK(gap < 0.5, "default false: the top crate sinks into the bottom one");
        }
        else
        {
            CHECK(gap > 0.7, "true: contact keeps them stacked");
        }
    }
    m2DestroyWorld(world);
}

// Motor and mouse joints (parity sprint 3b): a platform that chases
// offsets under gravity, and a crate dragged around by a soft target
// spring. Reference solves on Maul's delta tracking.
static void TestMotorAndMouseJoints(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.jointCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);

    // Motor platform: anchor at the origin, platform driven to an
    // offset and held there against gravity.
    m2BodyDef ad = m2DefaultBodyDef();
    ad.position = (m2Pos2){0.0, 4.0};
    m2BodyId anchor = m2CreateBody(world, &ad);
    m2BodyDef pd = m2DefaultBodyDef();
    pd.type = m2_dynamicBody;
    pd.position = (m2Pos2){0.0, 4.0};
    m2BodyId platform = m2CreateBody(world, &pd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon deck = m2MakeBox(0.8f, 0.1f);
    m2CreatePolygonShape(platform, &sd, &deck);
    m2MotorJointDef mj = m2DefaultMotorJointDef();
    mj.bodyIdA = anchor;
    mj.bodyIdB = platform;
    mj.linearOffset = (m2Vec2){2.0f, 1.0f};
    mj.maxForce = 200.0f;
    mj.maxTorque = 50.0f;
    m2JointId mover = m2CreateMotorJoint(world, &mj);
    for (int32_t i = 0; i < 180; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2Pos2 at = m2Body_GetPosition(platform);
    CHECK(at.x > 1.8 && at.x < 2.2, "the platform chases the x offset");
    CHECK(at.y > 4.7 && at.y < 5.2, "and holds the y offset against gravity");
    m2Vec2 off = m2MotorJoint_GetLinearOffset(mover);
    CHECK(off.x == 2.0f && off.y == 1.0f, "offsets read back");
    CHECK(m2MotorJoint_GetMaxForce(mover) == 200.0f, "and the force budget too");

    // Retarget: the platform follows.
    m2MotorJoint_SetOffsets(mover, (m2Vec2){-2.0f, 0.5f}, 0.0f);
    for (int32_t i = 0; i < 180; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    at = m2Body_GetPosition(platform);
    CHECK(at.x < -1.7, "retargeted platform crosses over");

    // Mouse drag: a crate on the floor gets pulled up and sideways.
    m2BodyDef gd = m2DefaultBodyDef();
    gd.position = (m2Pos2){10.0, -0.5};
    m2BodyId floor = m2CreateBody(world, &gd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(6.0f, 0.5f);
    m2CreatePolygonShape(floor, &fs, &slab);
    m2BodyDef cd = m2DefaultBodyDef();
    cd.type = m2_dynamicBody;
    cd.position = (m2Pos2){10.0, 0.45};
    m2BodyId crate = m2CreateBody(world, &cd);
    m2Polygon unit = m2MakeBox(0.4f, 0.4f);
    m2CreatePolygonShape(crate, &sd, &unit);
    m2MouseJointDef sj = m2DefaultMouseJointDef();
    sj.bodyIdA = floor;
    sj.bodyIdB = crate;
    sj.target = (m2Pos2){10.0, 0.45};
    sj.maxForce = 200.0f;
    m2JointId grip = m2CreateMouseJoint(world, &sj);
    m2MouseJoint_SetTarget(grip, (m2Pos2){12.0, 3.0});
    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    at = m2Body_GetPosition(crate);
    CHECK(at.x > 11.3 && at.x < 12.7, "the drag pulls the crate sideways");
    CHECK(at.y > 2.2 && at.y < 3.6, "and lifts it off the floor");
    m2Pos2 target = m2MouseJoint_GetTarget(grip);
    CHECK(target.x == 12.0 && target.y == 3.0, "the target reads back");
    CHECK(m2Joint_GetReactionForce(grip) > 1.0f, "the spring carries real load");
    CHECK(m2Joint_GetType(mover) == m2_motorJoint && m2Joint_GetType(grip) == m2_mouseJoint,
          "types read back");

    m2DestroyWorld(world);
}

// The richer distance joint (parity bucket 2): a hard length range
// over the spring, runtime retargeting, and runtime softness.
static void TestDistanceRange(void)
{
    // The hard range rows run on the stiff default softness, so when
    // the range disagrees with a soft rod, the range wins. That is
    // the observable contract: a squishy rope reeled hard.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.jointCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);

    m2JointId ropes[2];
    m2BodyId crates[2];
    for (int32_t i = 0; i < 2; ++i)
    {
        double x = (double)i * 8.0;
        m2BodyDef ad = m2DefaultBodyDef();
        ad.position = (m2Pos2){x, 8.0};
        m2BodyId hook = m2CreateBody(world, &ad);
        m2BodyDef cd = m2DefaultBodyDef();
        cd.type = m2_dynamicBody;
        cd.position = (m2Pos2){x, 6.0};
        crates[i] = m2CreateBody(world, &cd);
        m2ShapeDef sd = m2DefaultShapeDef();
        sd.density = 5.0f;
        m2Polygon crate = m2MakeBox(0.5f, 0.5f);
        m2CreatePolygonShape(crates[i], &sd, &crate);
        m2DistanceJointDef jd = m2DefaultDistanceJointDef();
        jd.bodyIdA = hook;
        jd.bodyIdB = crates[i];
        jd.length = 2.0f;
        jd.hertz = 0.5f; // a squishy rod that converges near 2
        jd.dampingRatio = 0.7f;
        if (i == 1)
        {
            jd.minLength = 0.5f;
            jd.maxLength = 1.5f; // fights the rod, and must win
        }
        ropes[i] = m2CreateDistanceJoint(world, &jd);
    }
    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    double hangFree = 8.0 - m2Body_GetPosition(crates[0]).y;
    double hangClamped = 8.0 - m2Body_GetPosition(crates[1]).y;
    CHECK(hangFree > 1.9, "the free rope settles at its length");
    CHECK(hangClamped < 1.7, "the ranged rope is held at max length instead");
    float lo = 0.0f;
    float hi = 0.0f;
    m2Joint_GetLimits(ropes[1], &lo, &hi);
    CHECK(lo == 0.5f && hi == 1.5f, "the range reads back through GetLimits");

    // Runtime: reel the free one in the same way.
    m2DistanceJoint_SetLengthRange(ropes[0], 0.5f, 1.5f);
    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    double reeled = 8.0 - m2Body_GetPosition(crates[0]).y;
    CHECK(reeled < 1.7, "a runtime range reels the crate in");

    // Min side: force the clamped one OUT beyond its rod.
    m2DistanceJoint_SetLengthRange(ropes[1], 2.6f, 5.0f);
    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    double pushed = 8.0 - m2Body_GetPosition(crates[1]).y;
    CHECK(pushed > 2.4, "the min bound pushes the load out past the rod");

    // Runtime spring retuning is journaled and takes effect: soften
    // the free rope hugely and give it a shove; it wobbles (peak
    // deviation from its clamp grows), then stiffen and it pins.
    m2Joint_SetSpringHertz(ropes[0], 0.1f);
    m2Joint_SetSpringDampingRatio(ropes[0], 0.05f);
    m2Body_ApplyLinearImpulseToCenter(crates[0], (m2Vec2){30.0f, 0.0f});
    double wobble = 0.0;
    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        double sway = m2Body_GetPosition(crates[0]).x;
        sway = sway < 0.0 ? -sway : sway;
        wobble = sway > wobble ? sway : wobble;
    }
    CHECK(wobble > 0.5, "a shoved pendulum sways");

    // Retarget a rigid rod: the crate follows the new length.
    m2BodyDef ad2 = m2DefaultBodyDef();
    ad2.position = (m2Pos2){16.0, 8.0};
    m2BodyId hook2 = m2CreateBody(world, &ad2);
    m2BodyDef cd2 = m2DefaultBodyDef();
    cd2.type = m2_dynamicBody;
    cd2.position = (m2Pos2){16.0, 7.0};
    m2BodyId weight = m2CreateBody(world, &cd2);
    m2ShapeDef sd2 = m2DefaultShapeDef();
    m2Polygon unit = m2MakeBox(0.4f, 0.4f);
    m2CreatePolygonShape(weight, &sd2, &unit);
    m2DistanceJointDef rig = m2DefaultDistanceJointDef();
    rig.bodyIdA = hook2;
    rig.bodyIdB = weight;
    m2JointId rod = m2CreateDistanceJoint(world, &rig); // rigid, length 1
    m2DistanceJoint_SetLength(rod, 2.5f);
    for (int32_t i = 0; i < 180; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    double drop = 8.0 - m2Body_GetPosition(weight).y;
    CHECK(drop > 2.3 && drop < 2.7, "a retargeted rigid rod lowers the load");

    m2DestroyWorld(world);
}

// The pulley: a rope over two fixed world points. Heavy side sinks,
// light side rises, the rope total is conserved; a ratio-2 machine
// balances double the mass; a live retune recaptures the total and
// the machine re-decides without snapping.
static void TestPulleyJoint(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.jointCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);
    m2World_SetGravity(world, (m2Vec2){0.0f, -10.0f});

    m2Circle disc = {{0.0f, 0.0f}, 0.4f};

    // Machine 1: equal ratio, unequal masses. The heavy side wins.
    m2BodyDef hd = m2DefaultBodyDef();
    hd.type = m2_dynamicBody;
    hd.position = (m2Pos2){-2.0, 2.0};
    m2BodyId heavy = m2CreateBody(world, &hd);
    m2ShapeDef hs = m2DefaultShapeDef();
    hs.density = 4.0f;
    m2CreateCircleShape(heavy, &hs, &disc);
    m2BodyDef ld = m2DefaultBodyDef();
    ld.type = m2_dynamicBody;
    ld.position = (m2Pos2){2.0, 2.0};
    m2BodyId light = m2CreateBody(world, &ld);
    m2ShapeDef ls = m2DefaultShapeDef();
    ls.density = 1.0f;
    m2CreateCircleShape(light, &ls, &disc);
    m2PulleyJointDef pd = m2DefaultPulleyJointDef();
    pd.bodyIdA = heavy;
    pd.bodyIdB = light;
    pd.groundAnchorA = (m2Pos2){-2.0, 4.0};
    pd.groundAnchorB = (m2Pos2){2.0, 4.0};
    m2JointId rope = m2CreatePulleyJoint(world, &pd);
    CHECK(m2PulleyJoint_GetRatio(rope) == 1.0f, "the ratio reads back");
    float spawnA = m2PulleyJoint_GetLengthA(rope);
    CHECK(spawnA > 1.9f && spawnA < 2.1f, "the A rope measures its spawn length");
    m2Pos2 gA = m2PulleyJoint_GetGroundAnchorA(rope);
    CHECK(gA.x == -2.0 && gA.y == 4.0, "the ground anchor reads back");

    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2Pos2 pHeavy = m2Body_GetPosition(heavy);
    m2Pos2 pLight = m2Body_GetPosition(light);
    CHECK(pHeavy.y < 1.7, "the heavy side sinks");
    CHECK(pLight.y > 2.3, "the light side rises");
    float total = m2PulleyJoint_GetLengthA(rope) + m2PulleyJoint_GetLengthB(rope);
    CHECK(total > 3.95f && total < 4.05f, "the rope total is conserved");
    CHECK(m2Joint_GetReactionForce(rope) > 0.0f, "the rope carries tension");

    // Machine 2: ratio 2 balances double the mass (the B side feels
    // twice the A-side tension).
    m2BodyDef ad = m2DefaultBodyDef();
    ad.type = m2_dynamicBody;
    ad.position = (m2Pos2){6.0, 2.0};
    m2BodyId counter = m2CreateBody(world, &ad);
    m2ShapeDef as = m2DefaultShapeDef();
    as.density = 1.0f;
    m2CreateCircleShape(counter, &as, &disc);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){10.0, 2.0};
    m2BodyId load = m2CreateBody(world, &bd);
    m2ShapeDef bs = m2DefaultShapeDef();
    bs.density = 2.0f;
    m2CreateCircleShape(load, &bs, &disc);
    m2PulleyJointDef bal = m2DefaultPulleyJointDef();
    bal.bodyIdA = counter;
    bal.bodyIdB = load;
    bal.groundAnchorA = (m2Pos2){6.0, 4.0};
    bal.groundAnchorB = (m2Pos2){10.0, 4.0};
    bal.ratio = 2.0f;
    m2JointId scale = m2CreatePulleyJoint(world, &bal);
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2Pos2 pCounter = m2Body_GetPosition(counter);
    m2Pos2 pLoad = m2Body_GetPosition(load);
    CHECK(pCounter.y > 1.85 && pCounter.y < 2.15, "the counterweight holds its height");
    CHECK(pLoad.y > 1.85 && pLoad.y < 2.15, "double mass balances at ratio 2");

    // Live retune: at ratio 1 the double-mass side wins; the total is
    // recaptured at the retune so the machine re-decides, no snap.
    m2PulleyJoint_SetRatio(scale, 1.0f);
    CHECK(m2PulleyJoint_GetRatio(scale) == 1.0f, "the new ratio reads back");
    float c0 = m2PulleyJoint_GetLengthA(scale) + m2PulleyJoint_GetLengthB(scale);
    for (int32_t i = 0; i < 40; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2Pos2 pCounter2 = m2Body_GetPosition(counter);
    m2Pos2 pLoad2 = m2Body_GetPosition(load);
    CHECK(pLoad2.y < pLoad.y - 0.1, "the heavy side wins after the retune");
    CHECK(pCounter2.y > pCounter.y + 0.1, "the counterweight gives way");
    float c1 = m2PulleyJoint_GetLengthA(scale) + m2PulleyJoint_GetLengthB(scale);
    CHECK(c1 > c0 - 0.05f && c1 < c0 + 0.05f, "the recaptured total is conserved");

    m2DestroyWorld(world);
}

// The gear joint: two pinned flywheels coupled at a ratio; drive
// one, the other counter-spins scaled; retune the ratio live; the
// phase survives many full turns.
static void TestGearJoint(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.jointCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);
    m2World_SetGravity(world, (m2Vec2){0.0f, 0.0f});

    m2BodyId wheels[2];
    m2JointId pins[2];
    for (int32_t i = 0; i < 2; ++i)
    {
        double x = (double)i * 3.0;
        m2BodyDef pd = m2DefaultBodyDef();
        pd.position = (m2Pos2){x, 2.0};
        m2BodyId post = m2CreateBody(world, &pd);
        m2BodyDef wd = m2DefaultBodyDef();
        wd.type = m2_dynamicBody;
        wd.position = (m2Pos2){x, 2.0};
        wheels[i] = m2CreateBody(world, &wd);
        m2ShapeDef sd = m2DefaultShapeDef();
        m2Circle disc = {{0.0f, 0.0f}, 0.5f};
        m2CreateCircleShape(wheels[i], &sd, &disc);
        m2RevoluteJointDef rj = m2DefaultRevoluteJointDef();
        rj.bodyIdA = post;
        rj.bodyIdB = wheels[i];
        if (i == 0)
        {
            rj.enableMotor = true;
            rj.motorSpeed = 3.0f;
            rj.maxMotorTorque = 50.0f;
        }
        pins[i] = m2CreateRevoluteJoint(world, &rj);
    }
    (void)pins;
    m2GearJointDef gj = m2DefaultGearJointDef();
    gj.bodyIdA = wheels[0];
    gj.bodyIdB = wheels[1];
    gj.ratio = 2.0f;
    m2JointId gear = m2CreateGearJoint(world, &gj);

    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    float wA = m2Body_GetAngularVelocity(wheels[0]);
    float wB = m2Body_GetAngularVelocity(wheels[1]);
    CHECK(wA > 2.7f && wA < 3.3f, "the driven wheel reaches its motor speed");
    CHECK(wB < -5.4f && wB > -6.6f, "the geared wheel counter-spins at twice the rate");
    CHECK(m2Joint_GetReactionTorque(gear) > 0.0f, "the gear carries torque");
    CHECK(m2GearJoint_GetRatio(gear) == 2.0f, "the ratio reads back");

    // Many full turns later the coupling still holds exactly: cut the
    // motor and the ratio persists through the coast-down.
    m2Joint_EnableMotor(pins[0], false);
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    wA = m2Body_GetAngularVelocity(wheels[0]);
    wB = m2Body_GetAngularVelocity(wheels[1]);
    CHECK(wA > 0.5f, "the pair coasts");
    CHECK(wB / wA < -1.9f && wB / wA > -2.1f, "still locked at the ratio");

    // Live retune halves the coupling.
    m2GearJoint_SetRatio(gear, 1.0f);
    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    wA = m2Body_GetAngularVelocity(wheels[0]);
    wB = m2Body_GetAngularVelocity(wheels[1]);
    CHECK(wA != 0.0f && wB / wA < -0.9f && wB / wA > -1.1f, "the new ratio takes over");

    m2DestroyWorld(world);
}

static void TestJointBreaking(void)
{
    // Twin ropes carry the same heavy crate; one has a break limit
    // under the load, one above it. The weak one snaps, reports the
    // force that did it, and the crate falls. Then a weld cantilever
    // breaks by torque alone. Snapping replays bit-exactly through a
    // pre-break snapshot because it is a pure function of state.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.jointCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);

    m2BodyId crates[2];
    m2JointId ropes[2];
    for (int32_t i = 0; i < 2; ++i)
    {
        double x = (double)i * 6.0;
        m2BodyDef ad = m2DefaultBodyDef();
        ad.position = (m2Pos2){x, 6.0};
        m2BodyId hook = m2CreateBody(world, &ad);
        m2BodyDef cd = m2DefaultBodyDef();
        cd.type = m2_dynamicBody;
        cd.position = (m2Pos2){x, 4.0};
        crates[i] = m2CreateBody(world, &cd);
        m2ShapeDef sd = m2DefaultShapeDef();
        sd.density = 5.0f;
        m2Polygon crate = m2MakeBox(0.5f, 0.5f);
        m2CreatePolygonShape(crates[i], &sd, &crate);
        m2DistanceJointDef jd = m2DefaultDistanceJointDef();
        jd.bodyIdA = hook;
        jd.bodyIdB = crates[i];
        ropes[i] = m2CreateDistanceJoint(world, &jd);
    }
    // Load = m*g = 5*1*10 = 50 N steady. Weak rope: 30 N. Strong: 500 N.
    m2Joint_SetBreakLimits(ropes[0], 30.0f, 0.0f);
    m2Joint_SetBreakLimits(ropes[1], 500.0f, 0.0f);

    bool snapped = false;
    float reportedForce = 0.0f;
    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        m2JointEvents events = m2World_GetJointEvents(world);
        if (events.breakCount > 0 && !snapped)
        {
            snapped = true;
            reportedForce = events.breakEvents[0].force;
            CHECK(events.breakEvents[0].jointId.index1 == ropes[0].index1,
                  "the weak rope is the one that snapped");
        }
    }
    CHECK(snapped, "the overloaded rope snaps");
    CHECK(reportedForce > 30.0f, "the event reports the force that broke it");
    CHECK(!m2Joint_IsValid(ropes[0]), "the snapped rope is gone");
    CHECK(m2Joint_IsValid(ropes[1]), "the strong rope holds");
    CHECK(m2Body_GetPosition(crates[0]).y < 2.0, "the crate fell");
    CHECK(m2Body_GetPosition(crates[1]).y > 3.0, "the safe crate hangs");

    // Torque break: a weld cantilever with a heavy tip.
    m2BodyDef wd = m2DefaultBodyDef();
    wd.position = (m2Pos2){12.0, 4.0};
    m2BodyId wall = m2CreateBody(world, &wd);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){13.0, 4.0};
    m2BodyId beam = m2CreateBody(world, &bd);
    m2ShapeDef bs = m2DefaultShapeDef();
    bs.density = 4.0f;
    m2Polygon bar = m2MakeBox(1.0f, 0.08f);
    m2CreatePolygonShape(beam, &bs, &bar);
    m2WeldJointDef weld = m2DefaultWeldJointDef();
    weld.bodyIdA = wall;
    weld.bodyIdB = beam;
    weld.localAnchorB = (m2Vec2){-1.0f, 0.0f};
    m2JointId cantilever = m2CreateWeldJoint(world, &weld);
    m2Joint_SetBreakLimits(cantilever, 0.0f, 4.0f); // gravity torque ~ 6.4 Nm
    bool cracked = false;
    for (int32_t i = 0; i < 120 && !cracked; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        cracked = m2World_GetJointEvents(world).breakCount > 0;
    }
    CHECK(cracked, "the weld cracks under torque alone");
    CHECK(!m2Joint_IsValid(cantilever), "and is gone");

    m2DestroyWorld(world);
}

static void TestBreakRollback(void)
{
    // Snapshot before the snap, replay through it: the joint must
    // break on the identical step with identical bits.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.jointCapacity = 4;
    m2WorldId world = m2CreateWorld(&def);
    m2BodyDef ad = m2DefaultBodyDef();
    ad.position = (m2Pos2){0.0, 6.0};
    m2BodyId hook = m2CreateBody(world, &ad);
    m2BodyDef cd = m2DefaultBodyDef();
    cd.type = m2_dynamicBody;
    cd.position = (m2Pos2){0.0, 4.0};
    m2BodyId crate = m2CreateBody(world, &cd);
    m2ShapeDef sd = m2DefaultShapeDef();
    sd.density = 5.0f;
    m2Polygon box = m2MakeBox(0.5f, 0.5f);
    m2CreatePolygonShape(crate, &sd, &box);
    m2DistanceJointDef jd = m2DefaultDistanceJointDef();
    jd.bodyIdA = hook;
    jd.bodyIdB = crate;
    m2JointId rope = m2CreateDistanceJoint(world, &jd);
    m2Joint_SetBreakLimits(rope, 30.0f, 0.0f);

    int32_t size = m2World_SnapshotSize(world);
    void* snap = malloc((size_t)size);
    CHECK(m2World_Snapshot(world, snap, size) == size, "snapshot before the snap");

    int32_t breakStepA = -1;
    uint64_t hashes[60];
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        hashes[i] = m2World_Hash(world);
        if (breakStepA < 0 && m2World_GetJointEvents(world).breakCount > 0)
        {
            breakStepA = i;
        }
    }
    CHECK(breakStepA >= 0, "the rope snapped in the recorded run");

    CHECK(m2World_Restore(world, snap, size), "restore");
    int32_t breakStepB = -1;
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        CHECK(m2World_Hash(world) == hashes[i], "the snap replays bit-exactly");
        if (breakStepB < 0 && m2World_GetJointEvents(world).breakCount > 0)
        {
            breakStepB = i;
        }
    }
    CHECK(breakStepA == breakStepB, "and lands on the identical step");

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

    // A motorized windmill and a limited piston churn beside the
    // bridge: motors, limits and the slider feed the same hash stream.
    m2BodyDef td = m2DefaultBodyDef();
    td.position = (m2Pos2){-9.1e5 + 12.0, 5.0};
    m2BodyId tower = m2CreateBody(world, &td);
    m2BodyDef wd = m2DefaultBodyDef();
    wd.type = m2_dynamicBody;
    wd.position = (m2Pos2){-9.1e5 + 12.0, 5.0};
    m2BodyId blade = m2CreateBody(world, &wd);
    m2ShapeDef ws = m2DefaultShapeDef();
    m2Polygon bladeBox = m2MakeBox(1.0f, 0.08f);
    m2CreatePolygonShape(blade, &ws, &bladeBox);
    m2RevoluteJointDef mill = m2DefaultRevoluteJointDef();
    mill.bodyIdA = tower;
    mill.bodyIdB = blade;
    mill.enableMotor = true;
    mill.motorSpeed = 2.5f;
    mill.maxMotorTorque = 30.0f;
    m2CreateRevoluteJoint(world, &mill);

    m2BodyDef cd = m2DefaultBodyDef();
    cd.position = (m2Pos2){-9.1e5 - 3.0, 5.0};
    m2BodyId cylinder = m2CreateBody(world, &cd);
    m2BodyDef hd = m2DefaultBodyDef();
    hd.type = m2_dynamicBody;
    hd.position = (m2Pos2){-9.1e5 - 3.0, 5.0};
    m2BodyId ram = m2CreateBody(world, &hd);
    m2Polygon ramBox = m2MakeBox(0.25f, 0.25f);
    m2CreatePolygonShape(ram, &ws, &ramBox);
    m2PrismaticJointDef press = m2DefaultPrismaticJointDef();
    press.bodyIdA = cylinder;
    press.bodyIdB = ram;
    press.localAxisA = (m2Vec2){0.0f, 1.0f};
    press.enableMotor = true;
    press.motorSpeed = -1.2f;
    press.maxMotorForce = 40.0f;
    press.enableLimit = true;
    press.lowerTranslation = -1.2f;
    press.upperTranslation = 0.5f;
    m2CreatePrismaticJoint(world, &press);

    // A welded counterweight hangs off the right pier; a sprung buggy
    // drives across the far apron: weld and wheel bytes join the hash.
    m2BodyDef counterDef = m2DefaultBodyDef();
    counterDef.type = m2_dynamicBody;
    counterDef.position = (m2Pos2){-9.1e5 + 9.6, 3.2};
    m2BodyId counter = m2CreateBody(world, &counterDef);
    m2ShapeDef counterShape = m2DefaultShapeDef();
    m2Polygon counterBox = m2MakeBox(0.3f, 0.3f);
    m2CreatePolygonShape(counter, &counterShape, &counterBox);
    m2WeldJointDef weld = m2DefaultWeldJointDef();
    weld.bodyIdA = right;
    weld.bodyIdB = counter;
    weld.localAnchorB = (m2Vec2){0.0f, 0.8f};
    m2CreateWeldJoint(world, &weld);

    m2BodyDef apronDef = m2DefaultBodyDef();
    apronDef.position = (m2Pos2){-9.1e5 + 20.0, 0.0};
    m2BodyId apron = m2CreateBody(world, &apronDef);
    m2ShapeDef apronShape = m2DefaultShapeDef();
    apronShape.friction = 0.9f;
    m2Polygon apronSlab = m2MakeBox(8.0f, 0.5f);
    m2CreatePolygonShape(apron, &apronShape, &apronSlab);
    m2BodyDef buggyDef = m2DefaultBodyDef();
    buggyDef.type = m2_dynamicBody;
    buggyDef.position = (m2Pos2){-9.1e5 + 15.0, 1.4};
    m2BodyId buggy = m2CreateBody(world, &buggyDef);
    m2Polygon buggyDeck = m2MakeBox(0.7f, 0.12f);
    m2CreatePolygonShape(buggy, &apronShape, &buggyDeck);
    for (int32_t i = 0; i < 2; ++i)
    {
        m2BodyDef wheelDef = m2DefaultBodyDef();
        wheelDef.type = m2_dynamicBody;
        wheelDef.position = (m2Pos2){-9.1e5 + 15.0 + (i == 0 ? -0.5 : 0.5), 1.0};
        m2BodyId tire = m2CreateBody(world, &wheelDef);
        m2ShapeDef tireShape = m2DefaultShapeDef();
        tireShape.friction = 0.9f;
        m2Circle tireCircle = {{0.0f, 0.0f}, 0.25f};
        m2CreateCircleShape(tire, &tireShape, &tireCircle);
        m2WheelJointDef ride = m2DefaultWheelJointDef();
        ride.bodyIdA = buggy;
        ride.bodyIdB = tire;
        ride.localAnchorA = (m2Vec2){i == 0 ? -0.5f : 0.5f, -0.4f};
        ride.localAxisA = (m2Vec2){0.0f, 1.0f};
        ride.hertz = 4.0f;
        ride.dampingRatio = 0.7f;
        ride.enableLimit = true;
        ride.lowerTranslation = -0.15f;
        ride.upperTranslation = 0.15f;
        ride.enableMotor = true;
        ride.motorSpeed = -6.0f;
        ride.maxMotorTorque = 10.0f;
        m2CreateWheelJoint(world, &ride);
    }

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
    TestRevoluteMotor();
    TestRevoluteLimit();
    TestPrismaticSlider();
    TestPrismaticMotor();
    TestMotorRollback();
    TestWeldRigid();
    TestWheelSuspension();
    TestWheelCar();
    TestJointRuntimeTuning();
    TestSoftWeld();
    TestReactionGetters();
    TestCollideConnectedAndFilterJoint();
    TestMotorAndMouseJoints();
    TestDistanceRange();
    TestGearJoint();
    TestPulleyJoint();
    TestJointBreaking();
    TestBreakRollback();

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
