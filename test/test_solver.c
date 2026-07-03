// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Solver gate: restitution bounce ratio, friction stick vs slide,
// pyramid settling without explosion or NaN, rollback replay with the
// solver active, and the solver evolution hash across CI cells.

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

static m2BodyId AddSlab(m2WorldId world, double y, float friction)
{
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, y};
    m2BodyId body = m2CreateBody(world, &fd);
    m2ShapeDef sd = m2DefaultShapeDef();
    sd.friction = friction;
    m2Polygon slab = m2MakeBox(20.0f, 0.5f);
    m2CreatePolygonShape(body, &sd, &slab);
    return body;
}

static void TestRestitution(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);
    AddSlab(world, -0.5, 0.6f);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 4.0};
    m2BodyId ball = m2CreateBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    sd.restitution = 0.8f;
    m2Circle circle = {{0.0f, 0.0f}, 0.5f};
    m2CreateCircleShape(ball, &sd, &circle);

    // Drop, bounce, find the apex after the first bounce.
    double apex = 0.0;
    bool bounced = false;
    double previousY = 4.0;
    bool descending = true;
    for (int32_t i = 0; i < 400; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        double y = m2Body_GetPosition(ball).y;
        if (descending && y > previousY + 1.0e-6)
        {
            descending = false;
            bounced = true;
        }
        if (!descending)
        {
            apex = y > apex ? y : apex;
            if (y < previousY - 1.0e-6 && apex > 1.0)
            {
                break; // second descent underway
            }
        }
        previousY = y;
    }
    CHECK(bounced, "ball bounces");
    // Drop height ~3.5m; e=0.8 => energy ratio 0.64 => apex ~2.24 above
    // the surface. Generous band: solver softness eats a little energy.
    double reboundHeight = apex - 0.5; // center at rest sits at y=0.5
    CHECK(reboundHeight > 1.4 && reboundHeight < 2.6, "restitution rebound in band");

    m2DestroyWorld(world);
}

static void TestFriction(void)
{
    // Two slabs and two boxes given the same shove: high friction stops
    // short, frictionless keeps going.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);
    AddSlab(world, -0.5, 1.0f);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){-10.0, 0.55};
    bd.linearVelocity = (m2Vec2){4.0f, 0.0f};
    m2BodyId gripBox = m2CreateBody(world, &bd);
    m2ShapeDef gripShape = m2DefaultShapeDef();
    gripShape.friction = 0.9f;
    m2Polygon unit = m2MakeBox(0.5f, 0.5f);
    m2CreatePolygonShape(gripBox, &gripShape, &unit);

    m2BodyDef bd2 = m2DefaultBodyDef();
    bd2.type = m2_dynamicBody;
    bd2.position = (m2Pos2){10.0, 0.55};
    bd2.linearVelocity = (m2Vec2){4.0f, 0.0f};
    m2BodyId iceBox = m2CreateBody(world, &bd2);
    m2ShapeDef iceShape = m2DefaultShapeDef();
    iceShape.friction = 0.0f;
    m2CreatePolygonShape(iceBox, &iceShape, &unit);

    for (int32_t i = 0; i < 180; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    double gripTravel = m2Body_GetPosition(gripBox).x - (-10.0);
    double iceTravel = m2Body_GetPosition(iceBox).x - 10.0;
    CHECK(gripTravel < 3.0, "high friction stops the box");
    CHECK(iceTravel > 8.0, "frictionless box keeps sliding");
    m2Vec2 gripVel = m2Body_GetLinearVelocity(gripBox);
    CHECK(gripVel.x * gripVel.x + gripVel.y * gripVel.y < 1.0e-2f, "gripped box at rest");

    m2DestroyWorld(world);
}

static void TestPyramid(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 64;
    def.shapeCapacity = 64;
    m2WorldId world = m2CreateWorld(&def);
    AddSlab(world, -0.5, 0.6f);

    enum
    {
        ROWS = 4
    };
    m2BodyId boxes[ROWS * (ROWS + 1) / 2];
    int32_t count = 0;
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon unit = m2MakeBox(0.5f, 0.5f);
    for (int32_t row = 0; row < ROWS; ++row)
    {
        for (int32_t col = row; col < ROWS; ++col)
        {
            m2BodyDef bd = m2DefaultBodyDef();
            bd.type = m2_dynamicBody;
            bd.position =
                (m2Pos2){(double)col - 0.5 * (double)(ROWS + row) + 0.5, 0.55 + 1.02 * (double)row};
            boxes[count] = m2CreateBody(world, &bd);
            m2CreatePolygonShape(boxes[count], &sd, &unit);
            count += 1;
        }
    }

    for (int32_t i = 0; i < 300; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }

    float maxSpeed = 0.0f;
    double minY = 100.0;
    for (int32_t i = 0; i < count; ++i)
    {
        m2Vec2 v = m2Body_GetLinearVelocity(boxes[i]);
        float speed = v.x * v.x + v.y * v.y;
        maxSpeed = m2MaxF(maxSpeed, speed);
        double y = m2Body_GetPosition(boxes[i]).y;
        CHECK(y == y, "no NaN positions in the pyramid");
        minY = y < minY ? y : minY;
    }
    CHECK(maxSpeed < 0.05f, "pyramid settles (no popcorn)");
    CHECK(minY > 0.3, "no box sank through the slab");

    m2DestroyWorld(world);
}

static void TestSolverRollback(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 32;
    def.shapeCapacity = 32;
    m2WorldId world = m2CreateWorld(&def);
    AddSlab(world, -0.5, 0.6f);
    m2ShapeDef sd = m2DefaultShapeDef();
    sd.restitution = 0.4f;
    for (int32_t i = 0; i < 6; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){-2.0 + 0.9 * (double)i, 1.0 + 0.7 * (double)i};
        m2BodyId body = m2CreateBody(world, &bd);
        m2Circle c = {{0.0f, 0.0f}, 0.4f};
        m2CreateCircleShape(body, &sd, &c);
    }
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    int32_t size = m2World_SnapshotSize(world);
    void* snap = malloc((size_t)size);
    CHECK(m2World_Snapshot(world, snap, size) == size, "snapshot");
    uint64_t hashes[40];
    for (int32_t i = 0; i < 40; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        hashes[i] = m2World_Hash(world);
    }
    CHECK(m2World_Restore(world, snap, size), "restore");
    for (int32_t i = 0; i < 40; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        CHECK(m2World_Hash(world) == hashes[i], "solver replays bit-exactly");
    }
    free(snap);
    m2DestroyWorld(world);
}

static uint64_t SolverSweepHash(void)
{
    // Mixed pile far from the origin: stacking, bouncing, sliding.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 128;
    def.shapeCapacity = 128;
    m2WorldId world = m2CreateWorld(&def);
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){-1.0e5, -0.5};
    m2BodyId floor = m2CreateBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(25.0f, 0.5f);
    m2CreatePolygonShape(floor, &fs, &slab);

    uint64_t h = M2_HASH_INIT;
    for (int32_t step = 0; step < 240; ++step)
    {
        if (step % 6 == 0 && step < 160)
        {
            m2BodyDef bd = m2DefaultBodyDef();
            bd.type = m2_dynamicBody;
            bd.position = (m2Pos2){-1.0e5 - 6.0 + 0.83 * (double)(step / 6), 5.0};
            bd.angularVelocity = 0.7f * (float)(step % 5) - 1.4f;
            m2BodyId body = m2CreateBody(world, &bd);
            m2ShapeDef sd = m2DefaultShapeDef();
            sd.friction = 0.2f + 0.2f * (float)(step % 3);
            sd.restitution = 0.15f * (float)(step % 4);
            if (step % 2 == 0)
            {
                m2Polygon box = m2MakeBox(0.35f, 0.25f);
                m2CreatePolygonShape(body, &sd, &box);
            }
            else
            {
                m2Circle c = {{0.0f, 0.0f}, 0.3f};
                m2CreateCircleShape(body, &sd, &c);
            }
        }
        m2World_Step(world, 1.0f / 60.0f, 4);
        uint64_t worldHash = m2World_Hash(world);
        h = m2Hash64(h, &worldHash, (int32_t)sizeof(worldHash));
    }
    m2DestroyWorld(world);
    return h;
}

static void TestSpinsAboutCenterOfMass(void)
{
    // A body whose shape sits away from its origin must rotate about
    // the shape's center of mass: the COM stays put, the origin orbits.
    m2WorldDef def = m2DefaultWorldDef();
    def.gravity = (m2Vec2){0.0f, 0.0f};
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 0.0};
    m2BodyId body = m2CreateBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Circle offset = {{2.0f, 0.0f}, 0.5f}; // COM at local (2, 0)
    m2CreateCircleShape(body, &sd, &offset);
    m2Body_SetAngularVelocity(body, 4.0f);

    double comX0 = 2.0;
    double comY0 = 0.0;
    double maxComDrift = 0.0;
    double maxOriginTravel = 0.0;
    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        m2Transform xf = m2Body_GetTransform(body);
        double comX = xf.p.x + (double)(xf.q.c * 2.0f);
        double comY = xf.p.y + (double)(xf.q.s * 2.0f);
        double dx = comX - comX0;
        double dy = comY - comY0;
        double drift = dx * dx + dy * dy;
        maxComDrift = drift > maxComDrift ? drift : maxComDrift;
        double travel = xf.p.x * xf.p.x + xf.p.y * xf.p.y;
        maxOriginTravel = travel > maxOriginTravel ? travel : maxOriginTravel;
    }
    CHECK(maxComDrift < 1.0e-4, "the center of mass holds still while spinning");
    CHECK(maxOriginTravel > 1.0, "the origin orbits the center of mass");
    float w = m2Body_GetAngularVelocity(body);
    CHECK(w > 3.99f && w < 4.01f, "free spin keeps its rate");

    // Counterweight at (-2, 0) pulls the COM back onto the origin.
    m2Circle balance = {{-2.0f, 0.0f}, 0.5f};
    m2CreateCircleShape(body, &sd, &balance);
    m2Transform mark = m2Body_GetTransform(body);
    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2Transform after = m2Body_GetTransform(body);
    double ox = after.p.x - mark.p.x;
    double oy = after.p.y - mark.p.y;
    CHECK(ox * ox + oy * oy < 1.0e-4, "balanced body spins in place again");

    m2DestroyWorld(world);
}

static void TestOffCenterRollback(void)
{
    // The new localCenter block rides the snapshot: an off-center
    // tumbling body must replay bit-exactly.
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

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 3.0};
    m2BodyId hammer = m2CreateBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon handle = m2MakeBox(0.6f, 0.08f);
    m2CreatePolygonShape(hammer, &sd, &handle);
    m2Circle head = {{0.7f, 0.0f}, 0.25f};
    sd.density = 4.0f;
    m2CreateCircleShape(hammer, &sd, &head);
    m2Body_SetAngularVelocity(hammer, 3.0f);

    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    int32_t size = m2World_SnapshotSize(world);
    void* snap = malloc((size_t)size);
    CHECK(m2World_Snapshot(world, snap, size) == size, "snapshot");
    uint64_t hashes[90];
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        hashes[i] = m2World_Hash(world);
    }
    CHECK(m2World_Restore(world, snap, size), "restore");
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        CHECK(m2World_Hash(world) == hashes[i], "off-center tumble replays bit-exactly");
    }
    free(snap);
    m2DestroyWorld(world);
}

static void TestImpulses(void)
{
    // Analytic: J at the COM moves without spin; J at an offset arm
    // spins by invI * cross(r, J); pure angular matches invI * L.
    m2WorldDef def = m2DefaultWorldDef();
    def.gravity = (m2Vec2){0.0f, 0.0f};
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 0.0};
    m2BodyId ball = m2CreateBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Circle circle = {{0.0f, 0.0f}, 0.5f};
    m2CreateCircleShape(ball, &sd, &circle);

    float mass = 3.14159265f * 0.25f; // rho=1, r=0.5
    m2Body_ApplyLinearImpulse(ball, (m2Vec2){mass, 0.0f}, (m2Pos2){0.0, 0.0});
    m2Vec2 v = m2Body_GetLinearVelocity(ball);
    CHECK(v.x > 0.99f && v.x < 1.01f, "COM impulse gives J/m");
    CHECK(m2Body_GetAngularVelocity(ball) == 0.0f, "COM impulse does not spin");

    // Arm (0, 0.5): J=(mass,0) -> torque = -0.5*mass, w = -0.5*mass*invI.
    m2Body_ApplyLinearImpulse(ball, (m2Vec2){mass, 0.0f}, (m2Pos2){0.0, 0.5});
    float w = m2Body_GetAngularVelocity(ball);
    CHECK(w < -0.1f, "offset impulse spins the ball");

    m2Body_ApplyAngularImpulse(ball, -w / m2Body_GetAngularVelocity(ball) * 0.0f);
    float before = m2Body_GetAngularVelocity(ball);
    m2Body_ApplyAngularImpulse(ball, 0.5f);
    CHECK(m2Body_GetAngularVelocity(ball) > before, "angular impulse adds spin");

    m2DestroyWorld(world);
}

int main(void)
{
    TestImpulses();
    TestSpinsAboutCenterOfMass();
    TestOffCenterRollback();
    TestRestitution();
    TestFriction();
    TestPyramid();
    TestSolverRollback();

    uint64_t hash = SolverSweepHash();
    printf("M2_SOLVER_HASH=%016llx\n", (unsigned long long)hash);

    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
