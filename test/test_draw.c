// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Debug-draw gate: m2World_Draw is a read-only walk that hands a
// renderer one primitive per shape, the right coupling per joint, a
// point per contact and a box per proxy. This suite records every
// callback and checks the counts, the colors that encode body state,
// that every emitted coordinate is finite, that the walk moves no
// bits, that it repeats identically, and that the special joints
// (filter draws nothing, gear and ratchet draw hub to hub, pulley
// draws its ropes and crossbar, mouse draws from its target) come out
// exactly as drawn. Draw carries no gated hash: it produces no state,
// so it is a pure pass/fail suite.

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

// A finite check without libm: NaN fails x == x, and the magnitude
// bound rejects both infinities.
static int FiniteD(double x)
{
    return x == x && x <= 1e300 && x >= -1e300;
}
static int FiniteF(float x)
{
    return x == x && x <= 1e30f && x >= -1e30f;
}

// The recorder: every callback lands here, tallying primitives, the
// last color seen per kind, and any non-finite coordinate. A rolling
// FNV-1a over the whole primitive stream proves the walk repeats.
typedef struct
{
    int polygons;
    int circles;
    int capsules;
    int segments;
    int points;
    int nonFinite;
    uint32_t lastPolygonColor;
    uint32_t lastCircleColor;
    uint32_t lastCapsuleColor;
    uint32_t lastSegmentColor;
    uint64_t streamHash;
} Rec;

static void MixU64(uint64_t* h, uint64_t v)
{
    *h ^= v;
    *h *= 0x100000001B3ull;
}
static void MixColor(Rec* r, uint32_t color)
{
    MixU64(&r->streamHash, 0x9E3779B9ull ^ (uint64_t)color);
}

static void OnPolygon(const m2Vec2* verts, int32_t count, m2Pos2 origin, m2Rot rot, uint32_t color,
                      void* ctx)
{
    Rec* r = (Rec*)ctx;
    r->polygons += 1;
    r->lastPolygonColor = color;
    if (!FiniteD(origin.x) || !FiniteD(origin.y) || !FiniteF(rot.c) || !FiniteF(rot.s) ||
        count <= 0)
    {
        r->nonFinite += 1;
    }
    for (int32_t i = 0; i < count; ++i)
    {
        if (!FiniteF(verts[i].x) || !FiniteF(verts[i].y))
        {
            r->nonFinite += 1;
        }
    }
    MixColor(r, color);
}
static void OnCircle(m2Pos2 c, float radius, m2Rot rot, uint32_t color, void* ctx)
{
    Rec* r = (Rec*)ctx;
    r->circles += 1;
    r->lastCircleColor = color;
    if (!FiniteD(c.x) || !FiniteD(c.y) || !FiniteF(radius) || radius <= 0.0f || !FiniteF(rot.c))
    {
        r->nonFinite += 1;
    }
    MixColor(r, color);
}
static void OnCapsule(m2Pos2 p1, m2Pos2 p2, float radius, uint32_t color, void* ctx)
{
    Rec* r = (Rec*)ctx;
    r->capsules += 1;
    r->lastCapsuleColor = color;
    if (!FiniteD(p1.x) || !FiniteD(p2.y) || !FiniteF(radius) || radius <= 0.0f)
    {
        r->nonFinite += 1;
    }
    MixColor(r, color);
}
static void OnSegment(m2Pos2 p1, m2Pos2 p2, uint32_t color, void* ctx)
{
    Rec* r = (Rec*)ctx;
    r->segments += 1;
    r->lastSegmentColor = color;
    if (!FiniteD(p1.x) || !FiniteD(p1.y) || !FiniteD(p2.x) || !FiniteD(p2.y))
    {
        r->nonFinite += 1;
    }
    MixColor(r, color);
}
static void OnPoint(m2Pos2 p, float size, uint32_t color, void* ctx)
{
    Rec* r = (Rec*)ctx;
    r->points += 1;
    if (!FiniteD(p.x) || !FiniteD(p.y) || !FiniteF(size))
    {
        r->nonFinite += 1;
    }
    MixColor(r, color);
}

static m2DebugDraw FullDraw(Rec* r)
{
    memset(r, 0, sizeof(*r));
    r->streamHash = 0xCBF29CE484222325ull;
    m2DebugDraw d;
    memset(&d, 0, sizeof(d));
    d.drawPolygon = OnPolygon;
    d.drawCircle = OnCircle;
    d.drawCapsule = OnCapsule;
    d.drawSegment = OnSegment;
    d.drawPoint = OnPoint;
    d.drawShapes = true;
    d.drawJoints = true;
    d.drawContacts = true;
    d.drawAABBs = true;
    d.context = r;
    return d;
}

// These mirror draw.c's palette; the test owns the expected values so
// a silent palette change is caught here.
enum
{
    kStatic = 0x7f7f7f,
    kKinematic = 0x5f9fd8,
    kAwake = 0xd8b25f,
    kSleeping = 0x6f6a55,
    kSensor = 0x5fd88a,
    kBullet = 0xd8755f,
};

static m2BodyId AddBody(m2WorldId world, m2BodyType type, double x, double y)
{
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = type;
    bd.position = (m2Pos2){x, y};
    return m2CreateBody(world, &bd);
}

static void TestShapeCountsAndColors(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    m2WorldId world = m2CreateWorld(&def);

    // One of every geometry, on its own dynamic body, plus a static
    // box, a kinematic box, a sensor circle and a bullet box so every
    // color branch is exercised.
    m2ShapeDef sd = m2DefaultShapeDef();

    m2BodyId circleBody = AddBody(world, m2_dynamicBody, 0.0, 5.0);
    m2Circle circle = {{0.0f, 0.0f}, 0.5f};
    m2CreateCircleShape(circleBody, &sd, &circle);

    m2BodyId boxBody = AddBody(world, m2_dynamicBody, 2.0, 5.0);
    m2Polygon box = m2MakeBox(0.4f, 0.4f);
    m2CreatePolygonShape(boxBody, &sd, &box);

    m2BodyId capBody = AddBody(world, m2_dynamicBody, 4.0, 5.0);
    m2Capsule capsule = {{-0.3f, 0.0f}, {0.3f, 0.0f}, 0.2f};
    m2CreateCapsuleShape(capBody, &sd, &capsule);

    m2BodyId segBody = AddBody(world, m2_staticBody, 6.0, 5.0);
    m2Segment segment = {{-1.0f, 0.0f}, {1.0f, 0.0f}};
    m2CreateSegmentShape(segBody, &sd, &segment);

    m2BodyId kinBody = AddBody(world, m2_kinematicBody, 8.0, 5.0);
    m2Polygon kbox = m2MakeBox(0.3f, 0.3f);
    m2CreatePolygonShape(kinBody, &sd, &kbox);

    m2ShapeDef sensorDef = m2DefaultShapeDef();
    sensorDef.isSensor = true;
    m2BodyId senBody = AddBody(world, m2_dynamicBody, 10.0, 5.0);
    m2Circle senCircle = {{0.0f, 0.0f}, 0.4f};
    m2CreateCircleShape(senBody, &sensorDef, &senCircle);

    m2BodyDef bulletDef = m2DefaultBodyDef();
    bulletDef.type = m2_dynamicBody;
    bulletDef.isBullet = true;
    bulletDef.position = (m2Pos2){12.0, 5.0};
    m2BodyId bulletBody = m2CreateBody(world, &bulletDef);
    m2Polygon bbox = m2MakeBox(0.3f, 0.3f);
    m2CreatePolygonShape(bulletBody, &sd, &bbox);

    // Isolate the shape walk: joints/contacts/aabbs off.
    Rec r = {0};
    m2DebugDraw d = FullDraw(&r);
    d.drawJoints = false;
    d.drawContacts = false;
    d.drawAABBs = false;
    m2World_Draw(world, &d);

    CHECK(r.circles == 2, "two circle shapes draw two circles"); // dynamic + sensor
    CHECK(r.capsules == 1, "one capsule shape draws one capsule");
    CHECK(r.segments == 1, "one segment shape draws one segment");      // no joints/aabbs here
    CHECK(r.polygons == 3, "three polygon shapes draw three polygons"); // box + kin + bullet
    CHECK(r.nonFinite == 0, "every emitted coordinate is finite");

    // Colors: the static segment is gray, the kinematic box blue, the
    // sensor circle green, the bullet box orange, a plain awake body tan.
    CHECK(r.lastSegmentColor == kStatic, "static body draws gray");
    CHECK(r.lastCapsuleColor == kAwake, "awake dynamic body draws tan");
    // The sensor circle is created after the plain circle, so the last
    // circle color recorded is the sensor's green.
    CHECK(r.lastCircleColor == kSensor, "sensor shape draws green");
    CHECK(r.lastPolygonColor == kBullet, "bullet body draws orange");

    // A kinematic body's color: draw only its box by walking a fresh
    // recorder with just that shape reachable is awkward; instead assert
    // the palette carries a distinct kinematic value by drawing a world
    // that holds only the kinematic body.
    m2WorldId kworld = m2CreateWorld(&def);
    m2BodyId onlyKin = AddBody(kworld, m2_kinematicBody, 0.0, 0.0);
    m2Polygon kb = m2MakeBox(0.5f, 0.5f);
    m2CreatePolygonShape(onlyKin, &sd, &kb);
    Rec kr = {0};
    m2DebugDraw kd = FullDraw(&kr);
    kd.drawJoints = false;
    kd.drawContacts = false;
    kd.drawAABBs = false;
    m2World_Draw(kworld, &kd);
    CHECK(kr.polygons == 1 && kr.lastPolygonColor == kKinematic, "kinematic body draws blue");
    m2DestroyWorld(kworld);

    m2DestroyWorld(world);
}

static void TestSleepingColor(void)
{
    // A settled dynamic body sleeps and must draw dim. Give it a floor
    // to rest on and step until it sleeps.
    m2WorldDef def = m2DefaultWorldDef();
    m2WorldId world = m2CreateWorld(&def);
    m2ShapeDef sd = m2DefaultShapeDef();

    m2BodyId floor = AddBody(world, m2_staticBody, 0.0, 0.0);
    m2Polygon slab = m2MakeBox(10.0f, 0.5f);
    m2CreatePolygonShape(floor, &sd, &slab);

    m2BodyId box = AddBody(world, m2_dynamicBody, 0.0, 1.0);
    m2Polygon b = m2MakeBox(0.5f, 0.5f);
    m2CreatePolygonShape(box, &sd, &b);

    for (int i = 0; i < 200 && m2Body_IsAwake(box); ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m2Body_IsAwake(box), "the box settles and sleeps");

    Rec r = {0};
    m2DebugDraw d = FullDraw(&r);
    d.drawJoints = false;
    d.drawContacts = false;
    d.drawAABBs = false;
    m2World_Draw(world, &d);
    // Two polygons: static floor (gray) and the sleeping box (dim).
    CHECK(r.polygons == 2, "floor and box both draw");
    CHECK(r.lastPolygonColor == kSleeping, "a sleeping body draws dim");
    m2DestroyWorld(world);
}

static m2BodyId PairBody(m2WorldId world, double x)
{
    m2BodyId body = AddBody(world, m2_dynamicBody, x, 5.0);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon box = m2MakeBox(0.3f, 0.3f);
    m2CreatePolygonShape(body, &sd, &box);
    return body;
}

static void TestJointDrawing(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    m2WorldId world = m2CreateWorld(&def);

    m2BodyId a = PairBody(world, 0.0);
    m2BodyId b = PairBody(world, 1.0);

    // Filter joint draws nothing.
    m2FilterJointDef fj = m2DefaultFilterJointDef();
    fj.bodyIdA = a;
    fj.bodyIdB = b;
    m2CreateFilterJoint(world, &fj);

    // Distance joint: one segment, two anchor points.
    m2DistanceJointDef dj = m2DefaultDistanceJointDef();
    dj.bodyIdA = a;
    dj.bodyIdB = b;
    dj.localAnchorA = (m2Vec2){0.1f, 0.0f};
    dj.localAnchorB = (m2Vec2){-0.1f, 0.0f};
    m2CreateDistanceJoint(world, &dj);

    // Gear: hub to hub, one segment, no points.
    m2GearJointDef gj = m2DefaultGearJointDef();
    gj.bodyIdA = a;
    gj.bodyIdB = b;
    gj.ratio = 1.5f;
    m2CreateGearJoint(world, &gj);

    // Ratchet: hub to hub, one segment, no points.
    m2RatchetJointDef rj = m2DefaultRatchetJointDef();
    rj.bodyIdA = a;
    rj.bodyIdB = b;
    rj.ratchet = 0.3f;
    m2CreateRatchetJoint(world, &rj);

    // Mouse: from the world target to B, one segment, two points.
    m2MouseJointDef mj = m2DefaultMouseJointDef();
    mj.bodyIdA = a;
    mj.bodyIdB = b;
    m2Pos2 bp = m2Body_GetPosition(b);
    mj.target = (m2Pos2){bp.x + 0.5, bp.y + 0.5};
    mj.maxForce = 50.0f;
    m2CreateMouseJoint(world, &mj);

    // Pulley: three segments (two ropes + crossbar) and two ground points.
    m2PulleyJointDef pj = m2DefaultPulleyJointDef();
    pj.bodyIdA = a;
    pj.bodyIdB = b;
    m2Pos2 pa = m2Body_GetPosition(a);
    m2Pos2 pbp = m2Body_GetPosition(b);
    pj.groundAnchorA = (m2Pos2){pa.x, pa.y + 3.0};
    pj.groundAnchorB = (m2Pos2){pbp.x, pbp.y + 3.0};
    pj.ratio = 1.0f;
    m2CreatePulleyJoint(world, &pj);

    // Isolate the joint walk.
    Rec r = {0};
    m2DebugDraw d = FullDraw(&r);
    d.drawShapes = false;
    d.drawContacts = false;
    d.drawAABBs = false;
    m2World_Draw(world, &d);

    // Segments: distance 1 + gear 1 + ratchet 1 + mouse 1 + pulley 3 = 7.
    // Points: distance 2 + mouse 2 + pulley 2 = 6. Filter contributes none.
    CHECK(r.segments == 7, "the joint set draws seven segments");
    CHECK(r.points == 6, "the joint set draws six anchor and ground points");
    CHECK(r.circles == 0 && r.polygons == 0 && r.capsules == 0, "joints emit no shape primitives");
    CHECK(r.nonFinite == 0, "every joint coordinate is finite");

    // With no segment callback the whole joint pass is skipped (draw.c
    // guards on drawSegment) and nothing is emitted.
    Rec r2 = {0};
    m2DebugDraw d2 = FullDraw(&r2);
    d2.drawShapes = false;
    d2.drawContacts = false;
    d2.drawAABBs = false;
    d2.drawSegment = NULL;
    m2World_Draw(world, &d2);
    CHECK(r2.points == 0, "no segment sink means the joint pass is skipped entirely");

    m2DestroyWorld(world);
}

static void TestChainDrawing(void)
{
    // A one-sided loop chain of four points becomes four chain-segment
    // shapes, each drawn as a segment; on a static body they are gray.
    m2WorldDef def = m2DefaultWorldDef();
    m2WorldId world = m2CreateWorld(&def);
    m2BodyId ground = AddBody(world, m2_staticBody, 0.0, 0.0);
    m2Vec2 loop[4] = {{-2.0f, -2.0f}, {2.0f, -2.0f}, {2.0f, 2.0f}, {-2.0f, 2.0f}};
    m2ChainDef cd = m2DefaultChainDef();
    cd.points = loop;
    cd.count = 4;
    cd.isLoop = true;
    m2CreateChain(ground, &cd);

    Rec r = {0};
    m2DebugDraw d = FullDraw(&r);
    d.drawJoints = false;
    d.drawContacts = false;
    d.drawAABBs = false;
    m2World_Draw(world, &d);
    CHECK(r.segments == 4, "a four-point loop chain draws four chain segments");
    CHECK(r.lastSegmentColor == kStatic, "chain segments on a static body draw gray");
    CHECK(r.nonFinite == 0, "every chain-segment endpoint is finite");
    CHECK(r.polygons == 0 && r.circles == 0, "a chain emits no other primitive");
    m2DestroyWorld(world);
}

static void TestContactsAndAABBs(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    m2WorldId world = m2CreateWorld(&def);
    m2ShapeDef sd = m2DefaultShapeDef();

    m2BodyId floor = AddBody(world, m2_staticBody, 0.0, 0.0);
    m2Polygon slab = m2MakeBox(10.0f, 0.5f);
    m2CreatePolygonShape(floor, &sd, &slab);

    m2BodyId box = AddBody(world, m2_dynamicBody, 0.0, 0.9);
    m2Polygon b = m2MakeBox(0.5f, 0.5f);
    m2CreatePolygonShape(box, &sd, &b);

    // A sensor sitting over the box: its overlap is a touching pair the
    // contact pass must SKIP (sensors are not contacts), exercising the
    // sensor guard.
    m2ShapeDef sensorDef = m2DefaultShapeDef();
    sensorDef.isSensor = true;
    m2BodyId sensor = AddBody(world, m2_staticBody, 0.0, 0.9);
    m2Polygon sbox = m2MakeBox(0.6f, 0.6f);
    m2CreatePolygonShape(sensor, &sensorDef, &sbox);

    // Settle enough to make a real touching manifold.
    for (int i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }

    Rec r = {0};
    m2DebugDraw d = FullDraw(&r);
    d.drawShapes = false;
    d.drawJoints = false;
    d.drawAABBs = false;
    m2World_Draw(world, &d);
    CHECK(r.points >= 1, "a resting box draws at least one contact point");
    CHECK(r.nonFinite == 0, "every contact point is finite");

    // A disabled body's shape leaves the broadphase, so its proxy is null
    // and the AABB pass must skip it (the null-proxy guard). Three alive
    // shapes remain in the tree: floor, box, sensor.
    m2BodyId ghost = AddBody(world, m2_dynamicBody, 5.0, 5.0);
    m2Polygon gb = m2MakeBox(0.3f, 0.3f);
    m2CreatePolygonShape(ghost, &sd, &gb);
    m2Body_Disable(ghost);

    Rec ra = {0};
    m2DebugDraw da = FullDraw(&ra);
    da.drawShapes = false;
    da.drawJoints = false;
    da.drawContacts = false;
    m2World_Draw(world, &da);
    CHECK(ra.segments == 12,
          "three in-tree proxies draw twelve AABB edges, the disabled one skipped");
    CHECK(ra.nonFinite == 0, "every AABB edge is finite");

    m2DestroyWorld(world);
}

static void TestContactForces(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    m2WorldId world = m2CreateWorld(&def);
    m2ShapeDef sd = m2DefaultShapeDef();

    m2BodyId floor = AddBody(world, m2_staticBody, 0.0, 0.0);
    m2Polygon slab = m2MakeBox(10.0f, 0.5f);
    m2CreatePolygonShape(floor, &sd, &slab);
    m2BodyId box = AddBody(world, m2_dynamicBody, 0.0, 0.9);
    m2Polygon b = m2MakeBox(0.5f, 0.5f);
    m2CreatePolygonShape(box, &sd, &b);
    for (int i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }

    // Only the force arrows: a resting box pushes on the floor, so each
    // contact point draws a normal-impulse arrow (plus a friction arrow,
    // near zero here). Two segments per point.
    Rec r = {0};
    m2DebugDraw d = FullDraw(&r);
    d.drawShapes = false;
    d.drawJoints = false;
    d.drawContacts = false;
    d.drawAABBs = false;
    d.drawContactForces = true;
    d.forceScale = 5.0f;
    m2World_Draw(world, &d);
    CHECK(r.segments >= 2, "a resting box draws force arrows at its contact");
    CHECK(r.nonFinite == 0, "every force arrow is finite");

    // Off unless asked: FullDraw leaves drawContactForces false.
    Rec r2 = {0};
    m2DebugDraw d2 = FullDraw(&r2);
    d2.drawShapes = false;
    d2.drawJoints = false;
    d2.drawContacts = false;
    d2.drawAABBs = false;
    m2World_Draw(world, &d2);
    CHECK(r2.segments == 0, "force arrows are off unless asked");

    m2DestroyWorld(world);
}

static void TestReadOnlyAndRepeatable(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    m2WorldId world = m2CreateWorld(&def);
    m2ShapeDef sd = m2DefaultShapeDef();

    m2BodyId floor = AddBody(world, m2_staticBody, 0.0, 0.0);
    m2Polygon slab = m2MakeBox(6.0f, 0.5f);
    m2CreatePolygonShape(floor, &sd, &slab);
    for (int i = 0; i < 6; ++i)
    {
        m2BodyId box =
            AddBody(world, m2_dynamicBody, (double)i * 0.3 - 0.75, 1.0 + (double)i * 0.6);
        m2Polygon b = m2MakeBox(0.25f, 0.25f);
        m2CreatePolygonShape(box, &sd, &b);
    }
    for (int i = 0; i < 40; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }

    uint64_t before = m2World_Hash(world);
    Rec r1 = {0};
    m2DebugDraw d = FullDraw(&r1);
    m2World_Draw(world, &d);
    uint64_t after = m2World_Hash(world);
    CHECK(before == after, "a draw walk moves no bits (read-only law)");

    // A second identical walk emits the identical primitive stream.
    Rec r2 = {0};
    m2DebugDraw d2 = FullDraw(&r2);
    m2World_Draw(world, &d2);
    CHECK(r1.streamHash == r2.streamHash, "two draws emit an identical primitive stream");
    CHECK(r1.polygons == r2.polygons && r1.segments == r2.segments && r1.points == r2.points,
          "two draws emit identical counts");
    CHECK(r1.nonFinite == 0, "a churned world still draws only finite coordinates");

    m2DestroyWorld(world);
}

static void TestNullAndEmpty(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    m2WorldId world = m2CreateWorld(&def);

    // A null draw pointer and a null world are both no-ops, not crashes.
    m2World_Draw(world, NULL);
    Rec r = {0};
    m2DebugDraw d = FullDraw(&r);
    m2World_Draw(m2_nullWorldId, &d);
    CHECK(r.polygons == 0 && r.segments == 0, "drawing a null world emits nothing");

    // An empty world draws nothing.
    m2World_Draw(world, &d);
    CHECK(r.polygons == 0 && r.circles == 0 && r.segments == 0 && r.points == 0,
          "an empty world emits nothing");

    // Partial vtables: only a circle sink present, a lone circle shape.
    m2ShapeDef sd = m2DefaultShapeDef();
    m2BodyId body = AddBody(world, m2_dynamicBody, 0.0, 0.0);
    m2Circle circle = {{0.0f, 0.0f}, 0.5f};
    m2CreateCircleShape(body, &sd, &circle);
    m2BodyId boxBody = AddBody(world, m2_dynamicBody, 2.0, 0.0);
    m2Polygon box = m2MakeBox(0.3f, 0.3f);
    m2CreatePolygonShape(boxBody, &sd, &box);

    Rec r2 = {0};
    m2DebugDraw partial;
    memset(&partial, 0, sizeof(partial));
    partial.drawCircle = OnCircle;
    partial.drawShapes = true;
    partial.context = &r2;
    r2.streamHash = 0xCBF29CE484222325ull;
    m2World_Draw(world, &partial); // no polygon sink: the box is silently skipped
    CHECK(r2.circles == 1 && r2.polygons == 0, "a missing sink skips its primitive, no crash");

    m2DestroyWorld(world);
}

int main(void)
{
    TestShapeCountsAndColors();
    TestSleepingColor();
    TestJointDrawing();
    TestChainDrawing();
    TestContactsAndAABBs();
    TestContactForces();
    TestReadOnlyAndRepeatable();
    TestNullAndEmpty();

    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
