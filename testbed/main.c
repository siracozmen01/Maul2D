// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The Maul2D testbed: an interactive playground over the debug-draw
// interface. Everything here is a VIEWER; the simulation stays inside
// the engine's determinism contract and the testbed only calls the
// public API. Controls:
//   left drag   grab bodies (a mouse joint, of course)
//   right drag  pan          wheel  zoom
//   space       pause        S      single step
//   tab         next scene   F5     restart scene
//   R (hold)    REWIND TIME through the snapshot ring
//   E           explosion at the cursor
//   B           drop a box at the cursor
//   arrows      drive (scenes with a car)

#include "maul2d/maul2d.h"
#include "raylib.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ---------------------------------------------------------------- camera
// World positions are f64; the camera subtracts its center in double
// and only then drops to float, so scenes far from the origin render
// rock steady (the same hybrid-precision idea the engine uses).
typedef struct tbCamera
{
    double cx;
    double cy;
    float pixelsPerMeter;
} tbCamera;

static tbCamera s_camera;
static int s_screenWidth = 1280;
static int s_screenHeight = 720;

static Vector2 tbToScreen(m2Pos2 p)
{
    float x = (float)(p.x - s_camera.cx) * s_camera.pixelsPerMeter + (float)s_screenWidth * 0.5f;
    float y = (float)s_screenHeight * 0.5f - (float)(p.y - s_camera.cy) * s_camera.pixelsPerMeter;
    return (Vector2){x, y};
}

static m2Pos2 tbToWorld(Vector2 s)
{
    double x =
        s_camera.cx + (double)((s.x - (float)s_screenWidth * 0.5f) / s_camera.pixelsPerMeter);
    double y =
        s_camera.cy + (double)(((float)s_screenHeight * 0.5f - s.y) / s_camera.pixelsPerMeter);
    return (m2Pos2){x, y};
}

static Color tbColor(uint32_t rgb)
{
    return (Color){(unsigned char)(rgb >> 16), (unsigned char)(rgb >> 8), (unsigned char)rgb, 255};
}

// ------------------------------------------------------------ draw bridge
static void tbDrawPolygon(const m2Vec2* localVertices, int32_t count, m2Pos2 origin, m2Rot q,
                          uint32_t color, void* context)
{
    (void)context;
    Vector2 pts[16];
    for (int32_t i = 0; i < count && i < 16; ++i)
    {
        m2Vec2 r = {q.c * localVertices[i].x - q.s * localVertices[i].y,
                    q.s * localVertices[i].x + q.c * localVertices[i].y};
        pts[i] = tbToScreen((m2Pos2){origin.x + (double)r.x, origin.y + (double)r.y});
    }
    for (int32_t i = 0; i < count; ++i)
    {
        DrawLineV(pts[i], pts[(i + 1) % count], tbColor(color));
    }
}

static void tbDrawCircle(m2Pos2 center, float radius, m2Rot q, uint32_t color, void* context)
{
    (void)context;
    Vector2 c = tbToScreen(center);
    DrawCircleLinesV(c, radius * s_camera.pixelsPerMeter, tbColor(color));
    // Radius line so rotation is visible.
    m2Pos2 rim = {center.x + (double)(q.c * radius), center.y + (double)(q.s * radius)};
    DrawLineV(c, tbToScreen(rim), tbColor(color));
}

static void tbDrawCapsule(m2Pos2 p1, m2Pos2 p2, float radius, uint32_t color, void* context)
{
    (void)context;
    Vector2 a = tbToScreen(p1);
    Vector2 b = tbToScreen(p2);
    float r = radius * s_camera.pixelsPerMeter;
    DrawCircleLinesV(a, r, tbColor(color));
    DrawCircleLinesV(b, r, tbColor(color));
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    float len = sqrtf(dx * dx + dy * dy);
    if (len > 0.0f)
    {
        float nx = -dy / len * r;
        float ny = dx / len * r;
        DrawLineV((Vector2){a.x + nx, a.y + ny}, (Vector2){b.x + nx, b.y + ny}, tbColor(color));
        DrawLineV((Vector2){a.x - nx, a.y - ny}, (Vector2){b.x - nx, b.y - ny}, tbColor(color));
    }
}

static void tbDrawSegment(m2Pos2 p1, m2Pos2 p2, uint32_t color, void* context)
{
    (void)context;
    DrawLineV(tbToScreen(p1), tbToScreen(p2), tbColor(color));
}

static void tbDrawPoint(m2Pos2 p, float size, uint32_t color, void* context)
{
    (void)context;
    DrawCircleV(tbToScreen(p), size, tbColor(color));
}

// ---------------------------------------------------------------- scenes
typedef struct tbScene
{
    const char* name;
    void (*setup)(m2WorldId world);
    void (*tick)(m2WorldId world, double simTime); // optional per-step logic
    bool hasCar;
    bool hasCharacter; // the mover-kit kinematic character
} tbScene;

static m2BodyId s_driveWheelA;
static m2BodyId s_driveWheelB;
static m2JointId s_driveJointA;
static m2JointId s_driveJointB;

static m2BodyId tbAddBox(m2WorldId world, double x, double y, float half, float density)
{
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){x, y};
    m2BodyId body = m2CreateBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    sd.density = density;
    m2Polygon box = m2MakeBox(half, half);
    m2CreatePolygonShape(body, &sd, &box);
    return body;
}

static m2BodyId tbAddFloor(m2WorldId world, double x, double y, float halfW)
{
    m2BodyDef gd = m2DefaultBodyDef();
    gd.position = (m2Pos2){x, y};
    m2BodyId floor = m2CreateBody(world, &gd);
    m2ShapeDef fs = m2DefaultShapeDef();
    fs.friction = 0.8f;
    m2Polygon slab = m2MakeBox(halfW, 0.5f);
    m2CreatePolygonShape(floor, &fs, &slab);
    return floor;
}

static void SceneWelcome(m2WorldId world)
{
    tbAddFloor(world, 0.0, -0.5, 30.0);
    // A pyramid to shove, a chain shelf to bounce under, seesaw, and a
    // breakable rope holding a wrecking ball: one of everything that
    // reads well on screen.
    for (int32_t row = 0; row < 10; ++row)
    {
        for (int32_t col = 0; col <= row; ++col)
        {
            tbAddBox(world, (double)col * 0.85 - (double)row * 0.425 - 6.0,
                     0.4 + (double)(9 - row) * 0.81, 0.4f, 1.0f);
        }
    }
    // One-way shelf: approach from below, land from above.
    m2BodyDef cd = m2DefaultBodyDef();
    cd.position = (m2Pos2){8.0, 3.0};
    m2BodyId shelfBody = m2CreateBody(world, &cd);
    m2Vec2 pts[6] = {{4.0f, 0.0f},  {2.5f, 0.0f},  {1.0f, 0.0f},
                     {-1.0f, 0.0f}, {-2.5f, 0.0f}, {-4.0f, 0.0f}};
    m2ChainDef chain = m2DefaultChainDef();
    chain.points = pts;
    chain.count = 6;
    m2CreateChain(shelfBody, &chain);

    // Wrecking ball on a breakable rope over the shelf.
    m2BodyDef hd = m2DefaultBodyDef();
    hd.position = (m2Pos2){8.0, 9.0};
    m2BodyId hook = m2CreateBody(world, &hd);
    m2BodyDef wd = m2DefaultBodyDef();
    wd.type = m2_dynamicBody;
    wd.position = (m2Pos2){8.0, 6.0};
    m2BodyId ball = m2CreateBody(world, &wd);
    m2ShapeDef bs = m2DefaultShapeDef();
    bs.density = 6.0f;
    m2Circle heavy = {{0.0f, 0.0f}, 0.6f};
    m2CreateCircleShape(ball, &bs, &heavy);
    m2DistanceJointDef dj = m2DefaultDistanceJointDef();
    dj.bodyIdA = hook;
    dj.bodyIdB = ball;
    m2JointId rope = m2CreateDistanceJoint(world, &dj);
    m2Joint_SetBreakLimits(rope, 260.0f, 0.0f); // yank it hard and it snaps
}

static void SceneCar(m2WorldId world)
{
    tbAddFloor(world, 0.0, -0.5, 18.0);
    // Rolling terrain out of a one-sided chain, then a car with two
    // motored wheel joints; arrows drive it.
    m2BodyDef td = m2DefaultBodyDef();
    td.position = (m2Pos2){44.0, -1.0};
    m2BodyId terrain = m2CreateBody(world, &td);
    m2Vec2 hills[12];
    for (int32_t i = 0; i < 12; ++i)
    {
        // Right to left: the solid side of the chain faces up.
        hills[i].x = (float)(28.0 - (double)i * 5.0);
        hills[i].y = (float)(1.2 * sin((double)i * 0.9) + 1.0);
    }
    m2ChainDef chain = m2DefaultChainDef();
    chain.points = hills;
    chain.count = 12;
    chain.friction = 0.9f;
    m2CreateChain(terrain, &chain);

    m2BodyDef cd = m2DefaultBodyDef();
    cd.type = m2_dynamicBody;
    cd.position = (m2Pos2){-8.0, 1.4};
    m2BodyId chassis = m2CreateBody(world, &cd);
    m2ShapeDef cs = m2DefaultShapeDef();
    cs.density = 1.6f;
    m2Polygon hull = m2MakeBox(1.1f, 0.3f);
    m2CreatePolygonShape(chassis, &cs, &hull);

    m2ShapeDef ws = m2DefaultShapeDef();
    ws.density = 1.0f;
    ws.friction = 1.1f;
    m2Circle tire = {{0.0f, 0.0f}, 0.38f};
    m2BodyId wheels[2];
    m2JointId axles[2];
    for (int32_t i = 0; i < 2; ++i)
    {
        m2BodyDef wd = m2DefaultBodyDef();
        wd.type = m2_dynamicBody;
        wd.position = (m2Pos2){-8.0 + (i == 0 ? -0.75 : 0.75), 1.0};
        wheels[i] = m2CreateBody(world, &wd);
        m2CreateCircleShape(wheels[i], &ws, &tire);
        m2WheelJointDef wj = m2DefaultWheelJointDef();
        wj.bodyIdA = chassis;
        wj.bodyIdB = wheels[i];
        wj.localAnchorA = (m2Vec2){i == 0 ? -0.75f : 0.75f, -0.4f};
        wj.localAxisA = (m2Vec2){0.0f, 1.0f};
        wj.enableSpring = true;
        wj.hertz = 4.0f;
        wj.dampingRatio = 0.7f;
        wj.enableMotor = true;
        wj.motorSpeed = 0.0f;
        wj.maxMotorTorque = 24.0f;
        axles[i] = m2CreateWheelJoint(world, &wj);
    }
    s_driveWheelA = wheels[0];
    s_driveWheelB = wheels[1];
    s_driveJointA = axles[0];
    s_driveJointB = axles[1];
}

static void SceneDemolition(m2WorldId world)
{
    tbAddFloor(world, 0.0, -0.5, 30.0);
    // A tower and a motorized platform ferrying crates: press E near
    // the tower and watch it come down; the platform keeps its route
    // via m2MotorJoint_SetOffsets from the input loop below.
    for (int32_t floor = 0; floor < 8; ++floor)
    {
        tbAddBox(world, -6.0, 0.45 + (double)floor * 0.92, 0.45f, 1.0f);
        tbAddBox(world, -4.6, 0.45 + (double)floor * 0.92, 0.45f, 1.0f);
        if (floor % 2 == 1)
        {
            tbAddBox(world, -5.3, 0.95 + (double)floor * 0.92, 0.45f, 0.8f);
        }
    }
    m2BodyDef ad = m2DefaultBodyDef();
    ad.position = (m2Pos2){6.0, 2.0};
    m2BodyId anchor = m2CreateBody(world, &ad);
    m2BodyDef pd = m2DefaultBodyDef();
    pd.type = m2_dynamicBody;
    pd.position = (m2Pos2){6.0, 2.0};
    m2BodyId platform = m2CreateBody(world, &pd);
    m2ShapeDef ps = m2DefaultShapeDef();
    ps.density = 2.0f;
    m2Polygon deck = m2MakeBox(1.4f, 0.15f);
    m2CreatePolygonShape(platform, &ps, &deck);
    m2MotorJointDef mj = m2DefaultMotorJointDef();
    mj.bodyIdA = anchor;
    mj.bodyIdB = platform;
    mj.maxForce = 400.0f;
    mj.maxTorque = 150.0f;
    mj.correctionFactor = 0.25f;
    s_driveJointA = m2CreateMotorJoint(world, &mj); // reused as the platform handle
    tbAddBox(world, 6.0, 2.6, 0.35f, 0.8f);
}

// A ragdoll out of capsules and limited revolutes; jointed bodies do
// not self-collide by default, which is exactly what a ragdoll wants.
static m2BodyId tbCapsulePart(m2WorldId world, double x, double y, float hx, float r)
{
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){x, y};
    m2BodyId part = m2CreateBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    sd.density = 1.0f;
    sd.friction = 0.4f;
    m2Capsule cap = {{-hx, 0.0f}, {hx, 0.0f}, r};
    m2CreateCapsuleShape(part, &sd, &cap);
    return part;
}

static void tbPin(m2WorldId world, m2BodyId a, m2BodyId b, m2Vec2 anchorA, m2Vec2 anchorB,
                  float lower, float upper)
{
    m2RevoluteJointDef jd = m2DefaultRevoluteJointDef();
    jd.bodyIdA = a;
    jd.bodyIdB = b;
    jd.localAnchorA = anchorA;
    jd.localAnchorB = anchorB;
    jd.enableLimit = true;
    jd.lowerAngle = lower;
    jd.upperAngle = upper;
    m2CreateRevoluteJoint(world, &jd);
}

static void tbMakeRagdoll(m2WorldId world, double x, double y)
{
    m2BodyId torso = tbCapsulePart(world, x, y, 0.02f, 0.34f);
    m2BodyDef hd = m2DefaultBodyDef();
    hd.type = m2_dynamicBody;
    hd.position = (m2Pos2){x, y + 0.62};
    m2BodyId head = m2CreateBody(world, &hd);
    m2ShapeDef hs = m2DefaultShapeDef();
    hs.density = 0.8f;
    m2Circle skull = {{0.0f, 0.0f}, 0.2f};
    m2CreateCircleShape(head, &hs, &skull);
    tbPin(world, torso, head, (m2Vec2){0.0f, 0.4f}, (m2Vec2){0.0f, -0.22f}, -0.5f, 0.5f);

    for (int32_t side = 0; side < 2; ++side)
    {
        double dir = side == 0 ? -1.0 : 1.0;
        m2BodyId upperArm = tbCapsulePart(world, x + dir * 0.45, y + 0.25, 0.2f, 0.09f);
        tbPin(world, torso, upperArm, (m2Vec2){(float)dir * 0.1f, 0.3f},
              (m2Vec2){(float)-dir * 0.25f, 0.0f}, -2.2f, 2.2f);
        m2BodyId lowerArm = tbCapsulePart(world, x + dir * 0.95, y + 0.25, 0.2f, 0.08f);
        tbPin(world, upperArm, lowerArm, (m2Vec2){(float)dir * 0.25f, 0.0f},
              (m2Vec2){(float)-dir * 0.25f, 0.0f}, side == 0 ? -2.4f : 0.0f,
              side == 0 ? 0.0f : 2.4f);
        m2BodyId thigh = tbCapsulePart(world, x + dir * 0.12, y - 0.62, 0.24f, 0.11f);
        tbPin(world, torso, thigh, (m2Vec2){(float)dir * 0.1f, -0.32f},
              (m2Vec2){(float)-dir * 0.28f, 0.0f}, -1.2f, 1.2f);
        m2BodyId shin = tbCapsulePart(world, x + dir * 0.12, y - 1.14, 0.22f, 0.1f);
        tbPin(world, thigh, shin, (m2Vec2){(float)dir * 0.28f, 0.0f},
              (m2Vec2){(float)-dir * 0.26f, 0.0f}, side == 0 ? 0.0f : -2.4f,
              side == 0 ? 2.4f : 0.0f);
    }
}

static void SceneRagdolls(m2WorldId world)
{
    tbAddFloor(world, 0.0, -0.5, 30.0);
    // A staircase for them to crumple down.
    for (int32_t s = 0; s < 7; ++s)
    {
        m2BodyDef sd = m2DefaultBodyDef();
        sd.position = (m2Pos2){-8.0 + (double)s * 2.0, (double)s * 0.7};
        m2BodyId step = m2CreateBody(world, &sd);
        m2ShapeDef ss = m2DefaultShapeDef();
        ss.friction = 0.6f;
        m2Polygon slab = m2MakeBox(1.0f, 0.35f);
        m2CreatePolygonShape(step, &ss, &slab);
    }
    for (int32_t i = 0; i < 3; ++i)
    {
        tbMakeRagdoll(world, 4.0 + (double)i * 0.4, 7.0 + (double)i * 2.4);
    }
}

static void SceneDemolitionTick(m2WorldId world, double simTime)
{
    (void)world;
    if (m2Joint_IsValid(s_driveJointA) && m2Joint_GetType(s_driveJointA) == m2_motorJoint)
    {
        float ferry = (float)(2.5 * sin(simTime * 0.7));
        m2MotorJoint_SetOffsets(s_driveJointA, (m2Vec2){ferry, 0.0f}, 0.0f);
    }
}

static void SceneDominoes(m2WorldId world)
{
    tbAddFloor(world, 0.0, -0.5, 40.0);
    // Forty dominoes, a starter ramp, and a ball to set it all off.
    for (int32_t i = 0; i < 40; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){-24.0 + (double)i * 1.1, 0.9};
        m2BodyId tile = m2CreateBody(world, &bd);
        m2ShapeDef sd = m2DefaultShapeDef();
        sd.density = 1.0f;
        sd.friction = 0.5f;
        m2Polygon slab = m2MakeBox(0.12f, 0.9f);
        m2CreatePolygonShape(tile, &sd, &slab);
    }
    m2BodyDef rd = m2DefaultBodyDef();
    rd.position = (m2Pos2){-28.5, 2.6};
    rd.rotation = (m2Rot){0.94f, -0.34f}; // ramp tilts toward the row
    m2BodyId ramp = m2CreateBody(world, &rd);
    m2ShapeDef rs = m2DefaultShapeDef();
    rs.friction = 0.1f;
    m2Polygon board = m2MakeBox(2.4f, 0.1f);
    m2CreatePolygonShape(ramp, &rs, &board);
    m2BodyDef md = m2DefaultBodyDef();
    md.type = m2_dynamicBody;
    md.position = (m2Pos2){-30.2, 4.4};
    m2BodyId marble = m2CreateBody(world, &md);
    m2ShapeDef ms = m2DefaultShapeDef();
    ms.density = 3.0f;
    m2Circle ball = {{0.0f, 0.0f}, 0.45f};
    m2CreateCircleShape(marble, &ms, &ball);
}

static void SceneOneWayCourse(m2WorldId world)
{
    tbAddFloor(world, 0.0, -0.5, 20.0);
    // Three one-way shelves at rising heights: drop boxes with B and
    // toss balls upward through them; they pass rising, land falling.
    for (int32_t level = 0; level < 3; ++level)
    {
        m2BodyDef cd = m2DefaultBodyDef();
        cd.position = (m2Pos2){-4.0 + (double)level * 5.0, 2.2 + (double)level * 2.2};
        m2BodyId shelf = m2CreateBody(world, &cd);
        m2Vec2 pts[5] = {{2.5f, 0.0f}, {1.2f, 0.0f}, {0.0f, 0.0f}, {-1.2f, 0.0f}, {-2.5f, 0.0f}};
        m2ChainDef chain = m2DefaultChainDef();
        chain.points = pts;
        chain.count = 5;
        m2CreateChain(shelf, &chain);
    }
    // A bouncy ball ready to be flung around.
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){-4.0, 0.6};
    m2BodyId bouncy = m2CreateBody(world, &bd);
    m2ShapeDef bs = m2DefaultShapeDef();
    bs.density = 0.8f;
    bs.restitution = 0.85f;
    m2Circle orb = {{0.0f, 0.0f}, 0.35f};
    m2CreateCircleShape(bouncy, &bs, &orb);
}

static void SceneJointZoo(m2WorldId world)
{
    tbAddFloor(world, 0.0, -0.5, 30.0);
    // One rig per joint type, left to right: distance pendulum,
    // motored revolute spinner, prismatic elevator, weld tee, wheel
    // rig, a motor-joint follower, and a breakable rope.
    m2ShapeDef sd = m2DefaultShapeDef();
    sd.density = 1.0f;
    m2Polygon unit = m2MakeBox(0.35f, 0.35f);
    m2Circle disc = {{0.0f, 0.0f}, 0.35f};

    m2BodyDef ad = m2DefaultBodyDef();
    ad.position = (m2Pos2){-12.0, 5.0};
    m2BodyId hookA = m2CreateBody(world, &ad);
    m2BodyDef p1 = m2DefaultBodyDef();
    p1.type = m2_dynamicBody;
    p1.position = (m2Pos2){-10.5, 5.0};
    m2BodyId bob = m2CreateBody(world, &p1);
    m2CreateCircleShape(bob, &sd, &disc);
    m2DistanceJointDef dj = m2DefaultDistanceJointDef();
    dj.bodyIdA = hookA;
    dj.bodyIdB = bob;
    m2CreateDistanceJoint(world, &dj);

    m2BodyDef sp = m2DefaultBodyDef();
    sp.position = (m2Pos2){-7.0, 3.0};
    m2BodyId pivot = m2CreateBody(world, &sp);
    m2BodyDef bl = m2DefaultBodyDef();
    bl.type = m2_dynamicBody;
    bl.position = (m2Pos2){-7.0, 3.0};
    m2BodyId blade = m2CreateBody(world, &bl);
    m2ShapeDef bs2 = m2DefaultShapeDef();
    bs2.density = 0.6f;
    m2Polygon bar = m2MakeBox(1.4f, 0.08f);
    m2CreatePolygonShape(blade, &bs2, &bar);
    m2RevoluteJointDef rj = m2DefaultRevoluteJointDef();
    rj.bodyIdA = pivot;
    rj.bodyIdB = blade;
    rj.enableMotor = true;
    rj.motorSpeed = 2.5f;
    rj.maxMotorTorque = 60.0f;
    m2CreateRevoluteJoint(world, &rj);

    m2BodyDef ed = m2DefaultBodyDef();
    ed.position = (m2Pos2){-3.0, 3.0};
    m2BodyId rail = m2CreateBody(world, &ed);
    m2BodyDef cd2 = m2DefaultBodyDef();
    cd2.type = m2_dynamicBody;
    cd2.position = (m2Pos2){-3.0, 3.0};
    m2BodyId cab = m2CreateBody(world, &cd2);
    m2CreatePolygonShape(cab, &sd, &unit);
    m2PrismaticJointDef pj = m2DefaultPrismaticJointDef();
    pj.bodyIdA = rail;
    pj.bodyIdB = cab;
    pj.localAxisA = (m2Vec2){0.0f, 1.0f};
    pj.enableLimit = true;
    pj.lowerTranslation = -2.0f;
    pj.upperTranslation = 2.0f;
    pj.enableMotor = true;
    pj.motorSpeed = 1.5f;
    pj.maxMotorForce = 80.0f;
    s_driveJointB = m2CreatePrismaticJoint(world, &pj); // elevator handle

    m2BodyDef wd = m2DefaultBodyDef();
    wd.type = m2_dynamicBody;
    wd.position = (m2Pos2){1.0, 2.0};
    m2BodyId post = m2CreateBody(world, &wd);
    m2CreatePolygonShape(post, &sd, &unit);
    m2BodyDef fd2 = m2DefaultBodyDef();
    fd2.type = m2_dynamicBody;
    fd2.position = (m2Pos2){1.7, 2.0};
    m2BodyId flag = m2CreateBody(world, &fd2);
    m2ShapeDef fs2 = m2DefaultShapeDef();
    fs2.density = 0.4f;
    m2Polygon pennant = m2MakeBox(0.35f, 0.08f);
    m2CreatePolygonShape(flag, &fs2, &pennant);
    m2WeldJointDef wj = m2DefaultWeldJointDef();
    wj.bodyIdA = post;
    wj.bodyIdB = flag;
    wj.localAnchorA = (m2Vec2){0.35f, 0.2f};
    wj.localAnchorB = (m2Vec2){-0.35f, 0.0f};
    wj.linearHertz = 4.0f;
    wj.linearDampingRatio = 0.4f;
    wj.angularHertz = 4.0f;
    wj.angularDampingRatio = 0.4f;
    m2CreateWeldJoint(world, &wj);

    m2BodyDef sd2 = m2DefaultBodyDef();
    sd2.position = (m2Pos2){5.5, 3.5};
    m2BodyId strut = m2CreateBody(world, &sd2);
    m2BodyDef hd2 = m2DefaultBodyDef();
    hd2.type = m2_dynamicBody;
    hd2.position = (m2Pos2){5.5, 2.2};
    m2BodyId hub = m2CreateBody(world, &hd2);
    m2CreateCircleShape(hub, &sd, &disc);
    m2WheelJointDef whj = m2DefaultWheelJointDef();
    whj.bodyIdA = strut;
    whj.bodyIdB = hub;
    whj.localAxisA = (m2Vec2){0.0f, 1.0f};
    whj.enableSpring = true;
    whj.hertz = 2.0f;
    whj.dampingRatio = 0.4f;
    m2CreateWheelJoint(world, &whj);

    m2BodyDef md2 = m2DefaultBodyDef();
    md2.position = (m2Pos2){9.5, 3.0};
    m2BodyId beacon = m2CreateBody(world, &md2);
    m2BodyDef fd3 = m2DefaultBodyDef();
    fd3.type = m2_dynamicBody;
    fd3.position = (m2Pos2){9.5, 3.0};
    m2BodyId chaser = m2CreateBody(world, &fd3);
    m2CreatePolygonShape(chaser, &sd, &unit);
    m2MotorJointDef mj = m2DefaultMotorJointDef();
    mj.bodyIdA = beacon;
    mj.bodyIdB = chaser;
    mj.maxForce = 120.0f;
    mj.maxTorque = 40.0f;
    s_driveJointA = m2CreateMotorJoint(world, &mj); // follower handle

    m2BodyDef hd3 = m2DefaultBodyDef();
    hd3.position = (m2Pos2){13.0, 6.0};
    m2BodyId gallows = m2CreateBody(world, &hd3);
    m2BodyDef ld = m2DefaultBodyDef();
    ld.type = m2_dynamicBody;
    ld.position = (m2Pos2){13.0, 4.0};
    m2BodyId weight = m2CreateBody(world, &ld);
    m2ShapeDef ws2 = m2DefaultShapeDef();
    ws2.density = 4.0f;
    m2CreatePolygonShape(weight, &ws2, &unit);
    m2DistanceJointDef frail = m2DefaultDistanceJointDef();
    frail.bodyIdA = gallows;
    frail.bodyIdB = weight;
    m2JointId thread = m2CreateDistanceJoint(world, &frail);
    m2Joint_SetBreakLimits(thread, 45.0f, 0.0f); // one extra box snaps it
}

static void SceneJointZooTick(m2WorldId world, double simTime)
{
    (void)world;
    // The elevator shuttles, the follower orbits its beacon.
    if (m2Joint_IsValid(s_driveJointB) && m2Joint_GetType(s_driveJointB) == m2_prismaticJoint)
    {
        float dir = fmod(simTime, 6.0) < 3.0 ? 1.5f : -1.5f;
        m2Joint_SetMotorSpeed(s_driveJointB, dir);
    }
    if (m2Joint_IsValid(s_driveJointA) && m2Joint_GetType(s_driveJointA) == m2_motorJoint)
    {
        float ox = (float)(1.6 * cos(simTime * 0.9));
        float oy = (float)(1.0 * sin(simTime * 1.7));
        m2MotorJoint_SetOffsets(s_driveJointA, (m2Vec2){ox, oy}, (float)(0.6 * sin(simTime * 0.5)));
    }
}

static void SceneCurtain(m2WorldId world)
{
    tbAddFloor(world, 0.0, -0.5, 30.0);
    // A hanging curtain of small plates tied by breakable ropes; drop
    // the anvil (or grab and throw things) and watch it tear.
    m2BodyDef rd = m2DefaultBodyDef();
    rd.position = (m2Pos2){0.0, 10.0};
    m2BodyId rail = m2CreateBody(world, &rd);
    m2ShapeDef ps = m2DefaultShapeDef();
    ps.density = 0.5f;
    m2Polygon plate = m2MakeBox(0.28f, 0.28f);
    m2BodyId prev[9];
    for (int32_t col = 0; col < 9; ++col)
    {
        prev[col] = rail;
    }
    for (int32_t row = 0; row < 6; ++row)
    {
        for (int32_t col = 0; col < 9; ++col)
        {
            m2BodyDef bd = m2DefaultBodyDef();
            bd.type = m2_dynamicBody;
            bd.position = (m2Pos2){-3.2 + (double)col * 0.8, 9.2 - (double)row * 0.75};
            m2BodyId cell = m2CreateBody(world, &bd);
            m2CreatePolygonShape(cell, &ps, &plate);
            m2DistanceJointDef dj = m2DefaultDistanceJointDef();
            dj.bodyIdA = prev[col];
            dj.bodyIdB = cell;
            dj.hertz = 8.0f;
            dj.dampingRatio = 0.6f;
            m2JointId link = m2CreateDistanceJoint(world, &dj);
            m2Joint_SetBreakLimits(link, 55.0f, 0.0f);
            prev[col] = cell;
        }
    }
    // The anvil, waiting above for a nudge.
    m2BodyDef av = m2DefaultBodyDef();
    av.type = m2_dynamicBody;
    av.position = (m2Pos2){-8.0, 13.0};
    m2BodyId anvil = m2CreateBody(world, &av);
    m2ShapeDef as2 = m2DefaultShapeDef();
    as2.density = 8.0f;
    m2Polygon block = m2MakeBox(0.7f, 0.5f);
    m2CreatePolygonShape(anvil, &as2, &block);
}

// ------------------------------------------- the mover-kit character
// A kinematic character that is NOT a body: just a capsule, a pose
// and a velocity living in the viewer, driven entirely by the public
// mover kit (CollideMover -> SolvePlanes -> ClipVector). Standing on
// a moving platform inherits its velocity, one-way shelves work from
// both sides, and none of it perturbs the simulation.
static m2Pos2 s_charPos;
static m2Pos2 s_charPrevPos; // for render interpolation
static m2Vec2 s_charVel;
static bool s_charGrounded;
static const m2Capsule s_charShape = {{0.0f, -0.35f}, {0.0f, 0.35f}, 0.3f};

static void CharacterReset(double x, double y)
{
    s_charPos = (m2Pos2){x, y};
    s_charPrevPos = s_charPos;
    s_charVel = (m2Vec2){0.0f, 0.0f};
    s_charGrounded = false;
}

static void CharacterUpdate(m2WorldId world, float dt)
{
    // Input: run and jump.
    float wish = 0.0f;
    if (IsKeyDown(KEY_RIGHT) || IsKeyDown(KEY_D))
    {
        wish += 8.0f;
    }
    if (IsKeyDown(KEY_LEFT) || IsKeyDown(KEY_A))
    {
        wish -= 8.0f;
    }
    float blend = s_charGrounded ? 0.25f : 0.06f; // air control is weaker
    s_charVel.x += (wish - s_charVel.x) * blend;
    s_charVel.y -= 24.0f * dt; // snappier-than-physics platformer gravity
    if (s_charVel.y < -30.0f)
    {
        s_charVel.y = -30.0f;
    }
    if (s_charGrounded && (IsKeyPressed(KEY_UP) || IsKeyPressed(KEY_W) || IsKeyPressed(KEY_SPACE)))
    {
        s_charVel.y = 11.0f;
    }

    s_charPrevPos = s_charPos;

    // Gather planes at the current pose.
    m2Transform pose = {s_charPos, {1.0f, 0.0f}};
    m2PlaneResult found[16];
    int32_t n = m2World_CollideMover(world, &s_charShape, pose, found, 16, m2DefaultQueryFilter());
    n = n < 16 ? n : 16;
    m2CollisionPlane planes[16];
    m2Vec2 carry = {0.0f, 0.0f};
    for (int32_t i = 0; i < n; ++i)
    {
        planes[i].normal = found[i].normal;
        planes[i].separation = found[i].separation;
        planes[i].pushLimit = 3.4e38f;
        planes[i].push = 0.0f;
        planes[i].clipVelocity = true;
        if (found[i].normal.y > 0.7f)
        {
            // Ride whatever we stand on (the ferry moment).
            m2BodyId under = m2Shape_GetBody(found[i].shapeId);
            m2Vec2 vUnder = m2Body_GetLinearVelocity(under);
            carry = vUnder;
        }
    }

    m2Vec2 delta = {(s_charVel.x + carry.x) * dt, (s_charVel.y + carry.y) * dt};
    m2PlaneSolverResult solved = m2SolvePlanes(delta, planes, n);
    s_charPos.x += (double)solved.translation.x;
    s_charPos.y += (double)solved.translation.y;
    s_charVel = m2ClipVector(s_charVel, planes, n);

    s_charGrounded = false;
    for (int32_t i = 0; i < n; ++i)
    {
        if (planes[i].push > 0.0f && planes[i].normal.y > 0.7f)
        {
            s_charGrounded = true;
        }
    }
    if (s_charPos.y < -8.0)
    {
        CharacterReset(-10.0, 2.0); // fell off the world
    }
}

static void CharacterDraw(double alpha)
{
    // The fixed-step presentation rule integrators need: draw
    // between the previous and current poses by the accumulator
    // fraction. alpha 1 = raw current pose.
    m2Pos2 at = {s_charPrevPos.x + (s_charPos.x - s_charPrevPos.x) * alpha,
                 s_charPrevPos.y + (s_charPos.y - s_charPrevPos.y) * alpha};
    m2Pos2 p1 = {at.x + (double)s_charShape.point1.x, at.y + (double)s_charShape.point1.y};
    m2Pos2 p2 = {at.x + (double)s_charShape.point2.x, at.y + (double)s_charShape.point2.y};
    tbDrawCapsule(p1, p2, s_charShape.radius, s_charGrounded ? 0x50E080u : 0xE0B050u, NULL);
}

static void ScenePlatformer(m2WorldId world)
{
    tbAddFloor(world, 0.0, -0.5, 26.0);
    CharacterReset(-10.0, 2.0);

    // A slope up to the shelf run.
    m2BodyDef sd = m2DefaultBodyDef();
    sd.position = (m2Pos2){-5.0, 0.8};
    sd.rotation = (m2Rot){0.94f, 0.34f};
    m2BodyId ramp = m2CreateBody(world, &sd);
    m2ShapeDef rs = m2DefaultShapeDef();
    rs.friction = 0.9f;
    m2Polygon board = m2MakeBox(2.6f, 0.2f);
    m2CreatePolygonShape(ramp, &rs, &board);

    // Three one-way shelves, stair-cased: jump up through them.
    for (int32_t level = 0; level < 3; ++level)
    {
        m2BodyDef cd = m2DefaultBodyDef();
        cd.position = (m2Pos2){1.0 + (double)level * 4.0, 2.4 + (double)level * 2.0};
        m2BodyId shelf = m2CreateBody(world, &cd);
        m2Vec2 pts[5] = {{2.2f, 0.0f}, {1.0f, 0.0f}, {0.0f, 0.0f}, {-1.0f, 0.0f}, {-2.2f, 0.0f}};
        m2ChainDef chain = m2DefaultChainDef();
        chain.points = pts;
        chain.count = 5;
        m2CreateChain(shelf, &chain);
    }

    // The ferry: a motor-joint platform patrolling under the far gap.
    m2BodyDef ad = m2DefaultBodyDef();
    ad.position = (m2Pos2){15.0, 4.0};
    m2BodyId beacon = m2CreateBody(world, &ad);
    m2BodyDef pd = m2DefaultBodyDef();
    pd.type = m2_dynamicBody;
    pd.position = (m2Pos2){15.0, 4.0};
    m2BodyId deckBody = m2CreateBody(world, &pd);
    m2ShapeDef ps = m2DefaultShapeDef();
    ps.density = 3.0f;
    ps.friction = 0.9f;
    m2Polygon deck = m2MakeBox(1.5f, 0.15f);
    m2CreatePolygonShape(deckBody, &ps, &deck);
    m2MotorJointDef mj = m2DefaultMotorJointDef();
    mj.bodyIdA = beacon;
    mj.bodyIdB = deckBody;
    mj.maxForce = 500.0f;
    mj.maxTorque = 200.0f;
    s_driveJointA = m2CreateMotorJoint(world, &mj);

    // Crates to shoulder around at ground level.
    for (int32_t i = 0; i < 3; ++i)
    {
        tbAddBox(world, 6.0 + (double)i * 1.1, 0.45, 0.4f, 0.6f);
    }
}

static void ScenePlatformerTick(m2WorldId world, double simTime)
{
    (void)world;
    if (m2Joint_IsValid(s_driveJointA) && m2Joint_GetType(s_driveJointA) == m2_motorJoint)
    {
        float ferry = (float)(3.0 * sin(simTime * 0.6));
        m2MotorJoint_SetOffsets(s_driveJointA, (m2Vec2){ferry, 0.0f}, 0.0f);
    }
}

static int32_t s_faucetTicks = 0;
static bool s_faucetTensile = false;

static void SceneWater(m2WorldId world)
{
    s_faucetTicks = 0;
    m2ShapeDef sd = m2DefaultShapeDef();
    m2BodyDef floorDef = m2DefaultBodyDef();
    floorDef.position = (m2Pos2){0.0, -0.1};
    m2Polygon floorBox = m2MakeBox(3.0f, 0.1f);
    m2CreatePolygonShape(m2CreateBody(world, &floorDef), &sd, &floorBox);
    m2Polygon wallBox = m2MakeBox(0.1f, 1.6f);
    m2BodyDef leftDef = m2DefaultBodyDef();
    leftDef.position = (m2Pos2){-3.0, 1.5};
    m2CreatePolygonShape(m2CreateBody(world, &leftDef), &sd, &wallBox);
    m2BodyDef rightDef = m2DefaultBodyDef();
    rightDef.position = (m2Pos2){3.0, 1.5};
    m2CreatePolygonShape(m2CreateBody(world, &rightDef), &sd, &wallBox);

    // Debris: two light crates that will float, one dense one that
    // will sink, and a plank.
    m2Polygon crate = m2MakeBox(0.15f, 0.15f);
    for (int32_t i = 0; i < 2; ++i)
    {
        m2BodyDef cd = m2DefaultBodyDef();
        cd.type = m2_dynamicBody;
        cd.position = (m2Pos2){-1.5 + (double)i * 1.2, 0.6};
        m2ShapeDef cs = m2DefaultShapeDef();
        cs.density = 0.15f;
        m2CreatePolygonShape(m2CreateBody(world, &cd), &cs, &crate);
    }
    m2BodyDef hd = m2DefaultBodyDef();
    hd.type = m2_dynamicBody;
    hd.position = (m2Pos2){1.6, 0.6};
    m2ShapeDef hs = m2DefaultShapeDef();
    hs.density = 4.0f;
    m2CreatePolygonShape(m2CreateBody(world, &hd), &hs, &crate);
    m2BodyDef pd = m2DefaultBodyDef();
    pd.type = m2_dynamicBody;
    pd.position = (m2Pos2){0.5, 0.8};
    m2ShapeDef ps = m2DefaultShapeDef();
    ps.density = 0.1f;
    m2Polygon plank = m2MakeBox(0.6f, 0.06f);
    m2CreatePolygonShape(m2CreateBody(world, &pd), &ps, &plank);
}

static void SceneWaterTick(m2WorldId world, double simTime)
{
    (void)simTime;
    // The faucet: three drops per tick until the pool is deep. The
    // spread pattern is a pure function of the tick counter, so the
    // pour is deterministic and the rewind ring replays it exactly.
    if (m2World_GetParticleCount(world) < 1500)
    {
        for (int32_t j = 0; j < 3; ++j)
        {
            int32_t lane = (s_faucetTicks + j * 5) % 7;
            double x = -0.27 + (double)lane * 0.09;
            uint32_t flags = s_faucetTensile ? m2_tensileParticle : 0;
            m2World_EmitParticle(world, (m2Pos2){x, 3.5}, (m2Vec2){0.0f, -2.0f}, flags);
        }
    }
    s_faucetTicks += 1;
}

// A rewind can resurrect a mouse grip that lived inside an old
// snapshot while the hand that held it is long gone. The engine is
// right to bring it back (restores are bit-exact by contract); the
// viewer owns its props, so when a rewind ends it sweeps any mouse
// joint it does not hold, along with the throwaway static anchor
// that grips hang from.
static void tbDropGhostGrips(m2WorldId world, m2JointId held)
{
    m2JointId joints[64];
    int32_t total = m2World_GetJoints(world, joints, 64);
    total = total <= 64 ? total : 64;
    for (int32_t i = 0; i < total; ++i)
    {
        if (m2Joint_GetType(joints[i]) != m2_mouseJoint)
        {
            continue;
        }
        if (held.index1 == joints[i].index1 && held.generation == joints[i].generation)
        {
            continue;
        }
        m2BodyId anchor = m2Joint_GetBodyA(joints[i]);
        m2DestroyJoint(joints[i]);
        if (m2Body_GetType(anchor) == m2_staticBody && m2Body_GetShapes(anchor, NULL, 0) == 0)
        {
            m2DestroyBody(anchor); // the grip's shapeless anchor post
        }
    }
}

static void SceneMachinery(m2WorldId world)
{
    tbAddFloor(world, 0.0, -0.5, 30.0);
    m2ShapeDef sd = m2DefaultShapeDef();
    sd.density = 1.0f;

    // Gear rig: two flywheels with visible spokes, motor on the left
    // one, gear ratio 2: the right wheel counter-spins twice as fast.
    m2Circle hub = {{0.0f, 0.0f}, 0.5f};
    m2Polygon spoke = m2MakeBox(0.45f, 0.06f);
    m2BodyId wheels[2];
    for (int32_t i = 0; i < 2; ++i)
    {
        double x = -10.0 + (double)i * 2.2;
        m2BodyDef postDef = m2DefaultBodyDef();
        postDef.position = (m2Pos2){x, 3.0};
        m2BodyId post = m2CreateBody(world, &postDef);
        m2BodyDef wheelDef = m2DefaultBodyDef();
        wheelDef.type = m2_dynamicBody;
        wheelDef.position = (m2Pos2){x, 3.0};
        wheels[i] = m2CreateBody(world, &wheelDef);
        m2CreateCircleShape(wheels[i], &sd, &hub);
        m2CreatePolygonShape(wheels[i], &sd, &spoke);
        m2RevoluteJointDef pin = m2DefaultRevoluteJointDef();
        pin.bodyIdA = post;
        pin.bodyIdB = wheels[i];
        if (i == 0)
        {
            pin.enableMotor = true;
            pin.motorSpeed = 2.5f;
            pin.maxMotorTorque = 80.0f;
        }
        m2CreateRevoluteJoint(world, &pin);
    }
    m2GearJointDef cog = m2DefaultGearJointDef();
    cog.bodyIdA = wheels[0];
    cog.bodyIdB = wheels[1];
    cog.ratio = 2.0f;
    m2CreateGearJoint(world, &cog);

    // Pulley elevator: two platforms on vertical prismatic rails,
    // coupled by a rope over two anchors. Drop the crate onto a
    // platform (or grab it) and watch the counterweight answer.
    m2Polygon platform = m2MakeBox(0.8f, 0.1f);
    m2BodyId cars[2];
    for (int32_t i = 0; i < 2; ++i)
    {
        double x = -3.5 + (double)i * 3.0;
        m2BodyDef railDef = m2DefaultBodyDef();
        railDef.position = (m2Pos2){x, 0.0};
        m2BodyId rail = m2CreateBody(world, &railDef);
        m2BodyDef carDef = m2DefaultBodyDef();
        carDef.type = m2_dynamicBody;
        carDef.position = (m2Pos2){x, 2.0};
        cars[i] = m2CreateBody(world, &carDef);
        m2ShapeDef ps = m2DefaultShapeDef();
        ps.density = 1.0f;
        m2CreatePolygonShape(cars[i], &ps, &platform);
        m2PrismaticJointDef rails = m2DefaultPrismaticJointDef();
        rails.bodyIdA = rail;
        rails.bodyIdB = cars[i];
        rails.localAxisA = (m2Vec2){0.0f, 1.0f};
        m2CreatePrismaticJoint(world, &rails);
    }
    m2PulleyJointDef rope = m2DefaultPulleyJointDef();
    rope.bodyIdA = cars[0];
    rope.bodyIdB = cars[1];
    rope.groundAnchorA = (m2Pos2){-3.5, 6.0};
    rope.groundAnchorB = (m2Pos2){-0.5, 6.0};
    m2CreatePulleyJoint(world, &rope);
    m2BodyDef cargoDef = m2DefaultBodyDef();
    cargoDef.type = m2_dynamicBody;
    cargoDef.position = (m2Pos2){-3.5, 2.5};
    m2BodyId cargo = m2CreateBody(world, &cargoDef);
    m2Polygon cargoBox = m2MakeBox(0.25f, 0.25f);
    m2ShapeDef cargoShape = m2DefaultShapeDef();
    cargoShape.density = 2.0f;
    m2CreatePolygonShape(cargo, &cargoShape, &cargoBox);

    // Sprung paddle: an angular spring holds it level; drop of balls
    // to bat away, or drag it down and let it snap back.
    m2BodyDef paddlePostDef = m2DefaultBodyDef();
    paddlePostDef.position = (m2Pos2){3.5, 1.6};
    m2BodyId paddlePost = m2CreateBody(world, &paddlePostDef);
    m2BodyDef paddleDef = m2DefaultBodyDef();
    paddleDef.type = m2_dynamicBody;
    paddleDef.position = (m2Pos2){4.6, 1.6};
    m2BodyId paddle = m2CreateBody(world, &paddleDef);
    m2Polygon blade = m2MakeBox(1.1f, 0.08f);
    m2CreatePolygonShape(paddle, &sd, &blade);
    m2RevoluteJointDef springPin = m2DefaultRevoluteJointDef();
    springPin.bodyIdA = paddlePost;
    springPin.bodyIdB = paddle;
    springPin.localAnchorB = (m2Vec2){-1.1f, 0.0f};
    springPin.springHertz = 5.0f;
    springPin.springDampingRatio = 0.4f;
    m2CreateRevoluteJoint(world, &springPin);
    m2Circle pebble = {{0.0f, 0.0f}, 0.15f};
    for (int32_t i = 0; i < 3; ++i)
    {
        m2BodyDef ballDef = m2DefaultBodyDef();
        ballDef.type = m2_dynamicBody;
        ballDef.position = (m2Pos2){4.0 + (double)i * 0.5, 4.0 + (double)i * 0.7};
        m2BodyId ball = m2CreateBody(world, &ballDef);
        m2ShapeDef bs = m2DefaultShapeDef();
        bs.density = 0.4f;
        bs.restitution = 0.5f;
        m2CreateCircleShape(ball, &bs, &pebble);
    }

    // Shrapnel twins: the same plus outline, once as a single body
    // of decomposed pieces, once pre-shattered into one body per
    // piece. Shove the right one and it crumbles.
    m2Vec2 plus[12] = {{0.6f, -1.2f}, {0.6f, -0.4f},  {1.4f, -0.4f},  {1.4f, 0.4f},
                       {0.6f, 0.4f},  {0.6f, 1.2f},   {-0.6f, 1.2f},  {-0.6f, 0.4f},
                       {-1.4f, 0.4f}, {-1.4f, -0.4f}, {-0.6f, -0.4f}, {-0.6f, -1.2f}};
    m2Polygon pieces[8];
    int32_t pieceCount = m2DecomposeOutline(plus, 12, pieces, 8);
    m2BodyDef statueDef = m2DefaultBodyDef();
    statueDef.type = m2_dynamicBody;
    statueDef.position = (m2Pos2){8.5, 0.9};
    m2BodyId statue = m2CreateBody(world, &statueDef);
    for (int32_t i = 0; i < pieceCount; ++i)
    {
        m2ShapeDef ss = m2DefaultShapeDef();
        m2CreatePolygonShape(statue, &ss, &pieces[i]);
    }
    for (int32_t i = 0; i < pieceCount; ++i)
    {
        m2BodyDef shardDef = m2DefaultBodyDef();
        shardDef.type = m2_dynamicBody;
        shardDef.position = (m2Pos2){12.0, 0.9};
        m2BodyId shard = m2CreateBody(world, &shardDef);
        m2ShapeDef ss = m2DefaultShapeDef();
        m2CreatePolygonShape(shard, &ss, &pieces[i]);
    }
}

static void SceneGoo(m2WorldId world)
{
    // A basin, a jelly cube dropped in, a powder heap poured beside
    // it, sticky spray above, and a crate to throw at all of them.
    m2ShapeDef sd = m2DefaultShapeDef();
    m2BodyDef floorDef = m2DefaultBodyDef();
    floorDef.position = (m2Pos2){0.0, -0.2};
    m2Polygon slab = m2MakeBox(6.0f, 0.2f);
    m2CreatePolygonShape(m2CreateBody(world, &floorDef), &sd, &slab);
    m2Polygon wallBox = m2MakeBox(0.2f, 2.0f);
    m2BodyDef leftDef = m2DefaultBodyDef();
    leftDef.position = (m2Pos2){-6.0, 1.8};
    m2CreatePolygonShape(m2CreateBody(world, &leftDef), &sd, &wallBox);
    m2BodyDef rightDef = m2DefaultBodyDef();
    rightDef.position = (m2Pos2){6.0, 1.8};
    m2CreatePolygonShape(m2CreateBody(world, &rightDef), &sd, &wallBox);

    m2Polygon jellyBox = m2MakeBox(0.45f, 0.45f);
    m2World_FillPolygonWithParticles(world, &jellyBox, (m2Pos2){-3.0, 1.8}, (m2Vec2){0.0f, 0.0f},
                                     m2_springParticle | m2_elasticParticle);
    m2Polygon heap = m2MakeBox(0.7f, 0.5f);
    m2World_FillPolygonWithParticles(world, &heap, (m2Pos2){0.5, 1.2}, (m2Vec2){0.0f, 0.0f},
                                     m2_powderParticle);
    m2Polygon spray = m2MakeBox(0.5f, 0.2f);
    m2World_FillPolygonWithParticles(world, &spray, (m2Pos2){3.5, 3.2}, (m2Vec2){0.0f, 0.0f},
                                     m2_tensileParticle);

    m2BodyDef crateDef = m2DefaultBodyDef();
    crateDef.type = m2_dynamicBody;
    crateDef.position = (m2Pos2){4.5, 0.4};
    m2BodyId crate = m2CreateBody(world, &crateDef);
    m2ShapeDef cs = m2DefaultShapeDef();
    cs.density = 0.4f;
    m2Polygon crateBox = m2MakeBox(0.25f, 0.25f);
    m2CreatePolygonShape(crate, &cs, &crateBox);
}

static const tbScene s_scenes[] = {
    {"welcome: pyramid, one-way shelf, wrecking ball", SceneWelcome, NULL, false, false},
    {"car: arrows to drive, terrain is a chain", SceneCar, NULL, true, false},
    {"demolition: E to blast, motor platform ferries", SceneDemolition, SceneDemolitionTick, false,
     false},
    {"ragdolls: grab and drag; hold R to rewind time", SceneRagdolls, NULL, false, false},
    {"dominoes: one marble, forty tiles", SceneDominoes, NULL, false, false},
    {"one-way course: fling the ball up through shelves", SceneOneWayCourse, NULL, false, false},
    {"joint zoo: every joint, one rig each", SceneJointZoo, SceneJointZooTick, false, false},
    {"curtain: breakable ropes, one anvil", SceneCurtain, NULL, false, false},
    {"platformer: A/D run, W jump, ride the ferry (mover kit)", ScenePlatformer,
     ScenePlatformerTick, false, true},
    {"water: faucet fills the basin, crates float; T = sticky water", SceneWater, SceneWaterTick,
     false, false},
    {"machinery: gears, pulley elevator, sprung paddle, shrapnel twins", SceneMachinery, NULL,
     false, false},
    {"goo: jelly cube, powder heap, sticky spray; grab and wreck", SceneGoo, NULL, false, false},
};
#define TB_SCENE_COUNT ((int32_t)(sizeof(s_scenes) / sizeof(s_scenes[0])))

// ------------------------------------------------------------------ main
int main(void)
{
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(s_screenWidth, s_screenHeight, "Maul2D testbed");
    SetTargetFPS(60);

    m2WorldId world = {0, 0};
    int32_t sceneIndex = 0;
    bool paused = false;
    bool showHelp = false;
    int32_t simMode = 0; // 0 = 60 Hz, 1 = 30 Hz snapped, 2 = 30 Hz interpolated
    int32_t frameParity = 0;
    double simTime = 0.0;
    m2JointId grip = m2_nullJointId;

    // THE REWIND RING: bit-exact snapshots of recent history. Hold R
    // and time runs backward; release and the simulation resumes from
    // wherever you let go. This is the rollback contract, visible.
    uint8_t* ring = NULL;
    int32_t ringEntry = 0; // bytes per snapshot
    int32_t ringCapacity = 0;
    int32_t ringCount = 0; // valid entries behind the present
    int32_t ringStart = 0; // oldest entry (true circular ring, no sliding)
    bool wasRewinding = false;
    int32_t ringStride = 2; // capture every Nth step
    int32_t stepParity = 0;

    m2DebugDraw draw = {0};
    draw.drawPolygon = tbDrawPolygon;
    draw.drawCircle = tbDrawCircle;
    draw.drawCapsule = tbDrawCapsule;
    draw.drawSegment = tbDrawSegment;
    draw.drawPoint = tbDrawPoint;
    draw.drawShapes = true;
    draw.drawJoints = true;
    draw.drawContacts = false;

    // Scene loader inline (recreate the world, run setup).
    int32_t pendingScene = 0;
    bool reload = true;

    while (!WindowShouldClose())
    {
        if (reload)
        {
            if (world.index1 != 0)
            {
                m2DestroyWorld(world);
            }
            m2WorldDef def = m2DefaultWorldDef();
            def.bodyCapacity = 512;
            def.shapeCapacity = 1024;
            def.jointCapacity = 64;
            def.particleCapacity = 2048; // the water scene pours into this
            world = m2CreateWorld(&def);
            free(ring);
            ringEntry = m2World_SnapshotSize(world);
#ifdef __EMSCRIPTEN__
            // Browsers get a slimmer rewind ring; the desktop
            // keeps its quarter gigabyte of history.
            int64_t budget = 64ll * 1024ll * 1024ll;
#else
            int64_t budget = 256ll * 1024ll * 1024ll;
#endif
            int64_t fit = budget / (int64_t)ringEntry;
            ringCapacity = fit < 30 ? 30 : (fit > 400 ? 400 : (int32_t)fit);
            ring = malloc((size_t)ringCapacity * (size_t)ringEntry);
            ringCount = 0;
            ringStart = 0;
            stepParity = 0;
            s_driveWheelA = m2_nullBodyId;
            s_driveWheelB = m2_nullBodyId;
            s_driveJointA = m2_nullJointId;
            s_driveJointB = m2_nullJointId;
            grip = m2_nullJointId;
            sceneIndex = pendingScene;
            s_scenes[sceneIndex].setup(world);
            s_camera = (tbCamera){0.0, 4.0, 40.0f};
            simTime = 0.0;
            reload = false;
        }

        // ---- input ----
        if (IsKeyPressed(KEY_TAB))
        {
            pendingScene = (sceneIndex + 1) % TB_SCENE_COUNT;
            reload = true;
        }
        if (IsKeyPressed(KEY_F5))
        {
            pendingScene = sceneIndex;
            reload = true;
        }
        if (IsKeyPressed(KEY_SPACE))
        {
            paused = !paused;
        }
        if (IsKeyPressed(KEY_C))
        {
            draw.drawContacts = !draw.drawContacts;
        }
        if (IsKeyPressed(KEY_V))
        {
            draw.drawAABBs = !draw.drawAABBs;
        }
        if (IsKeyPressed(KEY_H))
        {
            showHelp = !showHelp;
        }
        if (IsKeyPressed(KEY_T))
        {
            s_faucetTensile = !s_faucetTensile;
        }
        if (IsKeyPressed(KEY_I))
        {
            simMode = (simMode + 1) % 3;
        }
        float wheel = GetMouseWheelMove();
        if (wheel != 0.0f)
        {
            float factor = wheel > 0.0f ? 1.1f : 1.0f / 1.1f;
            s_camera.pixelsPerMeter *= factor;
            if (s_camera.pixelsPerMeter < 4.0f)
            {
                s_camera.pixelsPerMeter = 4.0f;
            }
        }
        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT))
        {
            Vector2 d = GetMouseDelta();
            s_camera.cx -= (double)(d.x / s_camera.pixelsPerMeter);
            s_camera.cy += (double)(d.y / s_camera.pixelsPerMeter);
        }

        m2Pos2 cursor = tbToWorld(GetMousePosition());

        // Grab: a mouse joint, naturally.
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && world.index1 != 0)
        {
            m2Circle probe = {{0.0f, 0.0f}, 0.05f};
            m2Transform pose = {cursor, {1.0f, 0.0f}};
            m2ShapeId hit[1];
            if (m2World_OverlapCircle(world, &probe, pose, hit, 1, m2DefaultQueryFilter()) > 0)
            {
                m2BodyId body = m2Shape_GetBody(hit[0]);
                if (m2Body_GetType(body) == m2_dynamicBody)
                {
                    m2MouseJointDef sj = m2DefaultMouseJointDef();
                    sj.bodyIdA = body; // bookkeeping anchor only
                    sj.bodyIdB = body;
                    // Two distinct bodies are required; grab against a
                    // throwaway static anchor instead.
                    m2BodyDef ad = m2DefaultBodyDef();
                    ad.position = cursor;
                    m2BodyId anchor = m2CreateBody(world, &ad);
                    sj.bodyIdA = anchor;
                    sj.target = cursor;
                    sj.hertz = 6.0f;
                    sj.maxForce = 120.0f * m2Body_GetMass(body);
                    grip = m2CreateMouseJoint(world, &sj);
                }
            }
        }
        if (grip.index1 != 0 && IsMouseButtonDown(MOUSE_BUTTON_LEFT))
        {
            m2MouseJoint_SetTarget(grip, cursor);
        }
        if (grip.index1 != 0 && IsMouseButtonReleased(MOUSE_BUTTON_LEFT))
        {
            if (m2Joint_IsValid(grip))
            {
                m2BodyId anchor = m2Joint_GetBodyA(grip);
                m2DestroyJoint(grip);
                m2DestroyBody(anchor);
            }
            grip = m2_nullJointId;
        }

        if (IsKeyPressed(KEY_E) && world.index1 != 0)
        {
            m2ExplosionDef boom = m2DefaultExplosionDef();
            boom.position = cursor;
            boom.radius = 2.0f;
            boom.falloff = 1.5f;
            boom.impulse = 8.0f;
            m2World_Explode(world, &boom);
        }
        if (IsKeyPressed(KEY_B) && world.index1 != 0)
        {
            tbAddBox(world, cursor.x, cursor.y, 0.4f, 1.0f);
        }

        // Drive: wheel motors on the car scene, platform offsets on the
        // demolition scene.
        if (s_scenes[sceneIndex].hasCar && m2Joint_IsValid(s_driveJointA))
        {
            float speed = 0.0f;
            if (IsKeyDown(KEY_RIGHT))
            {
                speed = -22.0f;
            }
            if (IsKeyDown(KEY_LEFT))
            {
                speed = 22.0f;
            }
            m2Joint_SetMotorSpeed(s_driveJointA, speed);
            m2Joint_SetMotorSpeed(s_driveJointB, speed);
        }

        // ---- step or rewind ----
        bool rewinding = IsKeyDown(KEY_R) && ring != NULL && ringCount > 0;
        bool single = IsKeyPressed(KEY_S);
        frameParity = (frameParity + 1) % 2;
        bool halfRateSkip = simMode != 0 && frameParity != 0;
        if (rewinding)
        {
            // Pop the newest snapshot and live there. Local handles into
            // the future (the grip and its anchor) are now stale by
            // definition; drop them and let the registries be the truth.
            grip = m2_nullJointId;
            ringCount -= 1;
            int32_t slot = (ringStart + ringCount) % ringCapacity;
            m2World_Restore(world, ring + (size_t)slot * (size_t)ringEntry, ringEntry);
            simTime -= (double)ringStride / 60.0;
        }
        else if ((!paused || single) && !halfRateSkip && world.index1 != 0)
        {
            m2World_Step(world, simMode == 0 ? 1.0f / 60.0f : 1.0f / 30.0f, 4);
            simTime += 1.0 / 60.0;
            if (s_scenes[sceneIndex].tick != NULL)
            {
                s_scenes[sceneIndex].tick(world, simTime);
            }
            if (s_scenes[sceneIndex].hasCharacter)
            {
                // The character lives in the viewer, not the world:
                // rewind rewinds the world under its feet only.
                CharacterUpdate(world, 1.0f / 60.0f);
            }
            stepParity = (stepParity + 1) % ringStride;
            if (stepParity == 0 && ring != NULL)
            {
                if (ringCount == ringCapacity)
                {
                    // Forget the oldest by moving the ring start: O(1).
                    ringStart = (ringStart + 1) % ringCapacity;
                    ringCount -= 1;
                }
                int32_t slot = (ringStart + ringCount) % ringCapacity;
                m2World_Snapshot(world, ring + (size_t)slot * (size_t)ringEntry, ringEntry);
                ringCount += 1;
            }
        }

        if (wasRewinding && !rewinding && world.index1 != 0)
        {
            tbDropGhostGrips(world, grip);
        }
        wasRewinding = rewinding;

        // ---- render ----
        BeginDrawing();
        ClearBackground((Color){24, 26, 32, 255});
        if (world.index1 != 0)
        {
            m2World_Draw(world, &draw);
        }
        if (world.index1 != 0 && m2World_GetParticleCount(world) > 0)
        {
            static m2ParticleId s_drops[2048];
            int32_t drops = m2World_GetParticles(world, s_drops, 2048);
            drops = drops <= 2048 ? drops : 2048;
            float dotRadius = 0.05f * s_camera.pixelsPerMeter;
            for (int32_t i = 0; i < drops; ++i)
            {
                Vector2 at = tbToScreen(m2Particle_GetPosition(s_drops[i]));
                uint32_t flags = m2Particle_GetFlags(s_drops[i]);
                Color tint = (Color){96, 156, 245, 200}; // water blue
                if ((flags & m2_powderParticle) != 0)
                {
                    tint = (Color){214, 181, 110, 220}; // sand
                }
                else if ((flags & (m2_springParticle | m2_elasticParticle)) != 0)
                {
                    tint = (Color){120, 220, 130, 220}; // jelly green
                }
                else if ((flags & m2_tensileParticle) != 0)
                {
                    tint = (Color){110, 220, 235, 210}; // sticky cyan
                }
                else if ((flags & m2_viscousParticle) != 0)
                {
                    tint = (Color){230, 170, 90, 210}; // syrup amber
                }
                DrawCircleV(at, dotRadius, tint);
            }
        }
        if (s_scenes[sceneIndex].hasCharacter)
        {
            double alpha = simMode == 2 && frameParity != 0 ? 0.5 : 1.0;
            CharacterDraw(alpha);
        }
        m2Counters counters = m2World_GetCounters(world);
        m2Profile profile = m2World_GetProfile(world);
        DrawText(
            TextFormat("[%d/%d] %s", sceneIndex + 1, TB_SCENE_COUNT, s_scenes[sceneIndex].name), 12,
            10, 20, RAYWHITE);
        DrawText(TextFormat(
                     "bodies %d (awake %d)  shapes %d  joints %d  step %.2f ms%s", counters.bodies,
                     counters.awakeBodies, counters.shapes, counters.joints, profile.stepMs,
                     simMode == 0
                         ? (paused ? "  [paused]" : "")
                         : (simMode == 1 ? "  [30 Hz snapped: I]" : "  [30 Hz interpolated: I]")),
                 12, 36, 16, GRAY);
        DrawText("drag: grab   rmb: pan   wheel: zoom   tab: scene   f5: reset   space: pause  "
                 " s: step   e: boom   b: box   HOLD R: REWIND TIME",
                 12, s_screenHeight - 26, 14, GRAY);
        if (ring != NULL && ringCapacity > 0)
        {
            float frac = (float)ringCount / (float)ringCapacity;
            int barWidth = (int)(260.0f * frac);
            DrawRectangle(s_screenWidth - 280, s_screenHeight - 30, 260, 10,
                          (Color){60, 60, 70, 255});
            DrawRectangle(s_screenWidth - 280, s_screenHeight - 30, barWidth, 10,
                          rewinding ? (Color){240, 120, 60, 255} : (Color){90, 170, 240, 255});
            DrawText(rewinding ? "<< rewinding" : "history", s_screenWidth - 280,
                     s_screenHeight - 48, 14, rewinding ? ORANGE : GRAY);
        }
        DrawText(TextFormat("step %llu  hash %016llx",
                            (unsigned long long)m2World_GetStepCount(world),
                            (unsigned long long)m2World_Hash(world)),
                 12, 58, 14, (Color){120, 200, 140, 255});
        if (showHelp)
        {
            DrawRectangle(s_screenWidth / 2 - 260, 90, 520, 300, (Color){10, 12, 16, 235});
            DrawRectangleLines(s_screenWidth / 2 - 260, 90, 520, 300, GRAY);
            const char* lines[] = {
                "Maul2D testbed",
                "",
                "left drag      grab bodies (mouse joint)",
                "right drag     pan            wheel   zoom",
                "space          pause          s       single step",
                "tab            next scene     f5      restart",
                "R (hold)       REWIND TIME    h       this panel",
                "e              explosion      b       drop a box",
                "c              contact points v       fat AABBs",
                "arrows         drive, where there is a car",
            };
            for (int32_t i = 0; i < 10; ++i)
            {
                DrawText(lines[i], s_screenWidth / 2 - 236, 116 + i * 26, 18,
                         i == 0 ? RAYWHITE : LIGHTGRAY);
            }
        }
        EndDrawing();
    }

    free(ring);
    if (world.index1 != 0)
    {
        m2DestroyWorld(world);
    }
    CloseWindow();
    return 0;
}
