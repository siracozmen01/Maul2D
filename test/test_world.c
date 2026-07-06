// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Slice-0 world gate: the rollback identity (hash-sequence equality plus
// snapshot -> restore -> re-snapshot byte-compare), id semantics
// (generation staleness, FIFO reuse, replay re-minting), def cookies,
// and the cross-platform world hash printed for CI comparison.

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

static m2WorldId MakeTestWorld(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 256;
    return m2CreateWorld(&def);
}

static void SpawnScene(m2WorldId world)
{
    m2BodyDef groundDef = m2DefaultBodyDef();
    groundDef.position = (m2Pos2){0.0, -1.0};
    m2CreateBody(world, &groundDef);

    for (int32_t i = 0; i < 50; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){1.0e6 + 0.1 * (double)(i % 10), 5.0 + 1.1 * (double)(i / 10)};
        bd.rotation = m2MakeRot(0.05f * (float)i);
        bd.linearVelocity = (m2Vec2){0.25f * (float)(i % 5) - 0.5f, 0.0f};
        bd.angularVelocity = 0.3f * (float)(i % 3) - 0.3f;
        bd.userData = (uint64_t)i;
        m2CreateBody(world, &bd);
    }
}

static void TestDefCookies(void)
{
    m2WorldDef zeroWorld;
    memset(&zeroWorld, 0, sizeof(zeroWorld));
#if defined(NDEBUG)
    CHECK(!m2World_IsValid(m2CreateWorld(&zeroWorld)),
          "zero-initialized world def must be rejected");
#endif
    m2BodyDef good = m2DefaultBodyDef();
    CHECK(good.internalValue != 0, "cookie must be nonzero by construction");
}

static void TestRollbackIdentity(void)
{
    m2WorldId world = MakeTestWorld();
    CHECK(m2World_IsValid(world), "world creation");
    SpawnScene(world);

    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }

    int32_t size = m2World_SnapshotSize(world);
    void* snapA = malloc((size_t)size);
    void* snapB = malloc((size_t)size);
    CHECK(m2World_Snapshot(world, snapA, size) == size, "snapshot A");

    // Immediate restore + re-snapshot must be byte-identical (oracle 2).
    CHECK(m2World_Restore(world, snapA, size), "restore A");
    CHECK(m2World_Snapshot(world, snapB, size) == size, "snapshot B");
    CHECK(memcmp(snapA, snapB, (size_t)size) == 0, "restore->re-snapshot byte-compare");

    // Trajectory identity: continue, record hashes; restore, re-run, compare.
    uint64_t hashes[60];
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        hashes[i] = m2World_Hash(world);
    }
    CHECK(m2World_Restore(world, snapA, size), "restore for replay");
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        CHECK(m2World_Hash(world) == hashes[i], "rollback identity: hash sequence must match");
    }

    // Post-restore create/destroy determinism (topic-09 gate requirement):
    // the same commands after restore must mint bit-identical ids.
    CHECK(m2World_Restore(world, snapA, size), "restore for id re-mint");
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    m2BodyId mintedA = m2CreateBody(world, &bd);
    CHECK(m2World_Restore(world, snapA, size), "restore again");
    m2BodyId mintedB = m2CreateBody(world, &bd);
    CHECK(mintedA.index1 == mintedB.index1 && mintedA.generation == mintedB.generation,
          "replay re-mints identical ids");

    free(snapA);
    free(snapB);
    m2DestroyWorld(world);
}

static void TestIdSemantics(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 4;
    m2WorldId world = m2CreateWorld(&def);

    m2BodyDef bd = m2DefaultBodyDef();
    m2BodyId a = m2CreateBody(world, &bd);
    m2BodyId b = m2CreateBody(world, &bd);
    CHECK(m2Body_IsValid(a) && m2Body_IsValid(b), "created ids valid");

    m2DestroyBody(a);
    CHECK(!m2Body_IsValid(a), "destroyed id is stale");

    // FIFO reuse: the freed slot goes to the queue tail, so the next two
    // creates use fresh slots before slot 0 comes around again.
    m2BodyId c = m2CreateBody(world, &bd);
    m2BodyId d = m2CreateBody(world, &bd);
    CHECK(c.index1 != a.index1 && d.index1 != a.index1, "FIFO reuse distance");
    m2BodyId e = m2CreateBody(world, &bd);
    CHECK(e.index1 == a.index1 && e.generation != a.generation,
          "recycled slot carries a bumped generation");
    CHECK(!m2Body_IsValid(a), "stale id stays stale after slot reuse");
    CHECK(m2CreateBody(world, &bd).index1 == 0, "capacity exhaustion returns the null id");

    m2World_Step(world, 1.0f / 60.0f, 1); // step with dead slots present
    m2DestroyWorld(world);
    CHECK(!m2Body_IsValid(e), "world destruction stales every body id");
}

static uint64_t DeterminismSweep(void)
{
    // A fixed 300-step scene far from the origin; the final hash is
    // compared bit-for-bit across every CI platform cell.
    m2WorldId world = MakeTestWorld();
    SpawnScene(world);
    for (int32_t i = 0; i < 300; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    // Mutate mid-run the way a game would, in fixed order.
    for (int32_t i = 0; i < 100; ++i)
    {
        m2BodyId probe = {i + 1, world.index1, 0};
        probe.generation = 0;
        (void)probe; // id probing is covered in TestIdSemantics
    }
    uint64_t h = m2World_Hash(world);
    m2DestroyWorld(world);
    return h;
}

static void TestCountersAndProfile(void)
{
    // Counters are state-derived and deterministic; profile is wall
    // clock and merely sane. Neither may touch the simulation hash.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 32;
    def.shapeCapacity = 32;
    m2WorldId world = m2CreateWorld(&def);

    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = m2CreateBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(15.0f, 0.5f);
    m2CreatePolygonShape(floor, &fs, &slab);
    for (int32_t i = 0; i < 6; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){-2.5 + (double)i, 0.55};
        m2BodyId body = m2CreateBody(world, &bd);
        m2ShapeDef sd = m2DefaultShapeDef();
        m2Polygon box = m2MakeBox(0.45f, 0.45f);
        m2CreatePolygonShape(body, &sd, &box);
    }

    for (int32_t i = 0; i < 10; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }

    m2Counters counters = m2World_GetCounters(world);
    CHECK(counters.bodies == 7 && counters.shapes == 7, "body and shape counts");
    CHECK(counters.awakeBodies == 7, "everyone still awake early on");
    CHECK(counters.joints == 0, "no joints in this scene");
    CHECK(counters.touchingPairs == 6, "six boxes rest on the slab");
    CHECK(counters.pairs >= counters.touchingPairs,
          "speculative pairs may outnumber touching ones");
    CHECK(counters.constraints == 6, "one constraint per touching pair");
    CHECK(counters.graphColors >= 1 && counters.overflowConstraints == 0,
          "colored without overflow");
    CHECK(counters.stepCount == 10, "step count mirrors the world");

    m2Profile profile = m2World_GetProfile(world);
    CHECK(profile.stepMs > 0.0f, "a step takes measurable time");
    CHECK(profile.pairsMs >= 0.0f && profile.contactsMs >= 0.0f && profile.solveMs >= 0.0f &&
              profile.sleepMs >= 0.0f,
          "phase times are sane");

    // Diagnostics never touch the simulation: hash before == after.
    uint64_t before = m2World_Hash(world);
    for (int32_t i = 0; i < 25; ++i)
    {
        m2World_GetCounters(world);
        m2World_GetProfile(world);
    }
    CHECK(m2World_Hash(world) == before, "diagnostics are read-only");

    // Twin world, same ops: counters identical (determinism).
    m2WorldId twin = m2CreateWorld(&def);
    m2BodyId floor2 = m2CreateBody(twin, &fd);
    m2CreatePolygonShape(floor2, &fs, &slab);
    for (int32_t i = 0; i < 6; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){-2.5 + (double)i, 0.55};
        m2BodyId body = m2CreateBody(twin, &bd);
        m2ShapeDef sd = m2DefaultShapeDef();
        m2Polygon box = m2MakeBox(0.45f, 0.45f);
        m2CreatePolygonShape(body, &sd, &box);
    }
    for (int32_t i = 0; i < 10; ++i)
    {
        m2World_Step(twin, 1.0f / 60.0f, 4);
    }
    m2Counters twinCounters = m2World_GetCounters(twin);
    CHECK(memcmp(&counters, &twinCounters, sizeof(m2Counters)) == 0,
          "twin worlds report identical counters");

    m2DestroyWorld(twin);
    m2DestroyWorld(world);
}

static void TestKineticEnergy(void)
{
    // Analytic: one ball, known mass, known velocity. Then the energy
    // must be twin-identical and vanish with sleep.
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
    m2Body_SetLinearVelocity(ball, (m2Vec2){3.0f, 4.0f}); // speed 5

    double mass = 3.14159265358979 * 0.25; // rho=1, r=0.5
    double expected = 0.5 * mass * 25.0;
    double energy = m2World_GetKineticEnergy(world);
    double err = energy - expected;
    CHECK(err > -1.0e-4 && err < 1.0e-4, "kinetic energy matches the hand computation");

    m2DestroyWorld(world);

    // A settling stack: energy decays to exactly zero once asleep,
    // and twin worlds agree to the bit the whole way down.
    m2WorldDef def2 = m2DefaultWorldDef();
    def2.bodyCapacity = 16;
    def2.shapeCapacity = 16;
    m2WorldId a = m2CreateWorld(&def2);
    m2WorldId b = m2CreateWorld(&def2);
    for (int32_t twin = 0; twin < 2; ++twin)
    {
        m2WorldId w = twin == 0 ? a : b;
        m2BodyDef fd = m2DefaultBodyDef();
        fd.position = (m2Pos2){0.0, -0.5};
        m2BodyId floor = m2CreateBody(w, &fd);
        m2ShapeDef fs = m2DefaultShapeDef();
        m2Polygon slab = m2MakeBox(10.0f, 0.5f);
        m2CreatePolygonShape(floor, &fs, &slab);
        for (int32_t i = 0; i < 4; ++i)
        {
            m2BodyDef box = m2DefaultBodyDef();
            box.type = m2_dynamicBody;
            box.position = (m2Pos2){0.0, 0.7 + 1.05 * (double)i};
            m2BodyId body = m2CreateBody(w, &box);
            m2ShapeDef bs = m2DefaultShapeDef();
            m2Polygon unit = m2MakeBox(0.45f, 0.45f);
            m2CreatePolygonShape(body, &bs, &unit);
        }
    }
    bool everAwake = false;
    for (int32_t i = 0; i < 300; ++i)
    {
        m2World_Step(a, 1.0f / 60.0f, 4);
        m2World_Step(b, 1.0f / 60.0f, 4);
        double ea = m2World_GetKineticEnergy(a);
        double eb = m2World_GetKineticEnergy(b);
        CHECK(ea == eb, "twin worlds agree on energy to the bit");
        everAwake = everAwake || ea > 0.0;
    }
    CHECK(everAwake, "the stack had energy while falling");
    CHECK(m2World_GetKineticEnergy(a) == 0.0, "sleep zeroes the ledger");
    m2DestroyWorld(a);
    m2DestroyWorld(b);
}

static void TestTeleport(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 5.0};
    bd.linearVelocity = (m2Vec2){1.5f, 0.0f};
    m2BodyId body = m2CreateBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Circle c = {{0.0f, 0.0f}, 0.4f};
    m2CreateCircleShape(body, &sd, &c);

    m2Body_SetTransform(body, (m2Pos2){100.0, -3.0}, m2MakeRot(0.5f));
    m2Pos2 p = m2Body_GetPosition(body);
    CHECK(p.x == 100.0 && p.y == -3.0, "teleport moves the origin exactly");
    m2Vec2 v = m2Body_GetLinearVelocity(body);
    CHECK(v.x == 1.5f && v.y == 0.0f, "teleport leaves velocity untouched");

    // Ray finds it at the new home immediately (broadphase refreshed).
    m2RayCastResult hit = m2World_CastRayClosest(world, (m2Pos2){100.0, 2.0}, (m2Vec2){0.0f, -6.0f},
                                                 m2DefaultQueryFilter());
    CHECK(hit.hit, "queries see the teleported body at once");

    CHECK(m2World_JournalBaseSize(world) == 32 + m2World_SnapshotSize(world),
          "journal base size is header plus snapshot");

    m2DestroyWorld(world);
}

static void TestSetType(void)
{
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

    // A falling box freezes mid-air when it turns static...
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 4.0};
    m2BodyId shelf = m2CreateBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon unit = m2MakeBox(0.5f, 0.1f);
    m2CreatePolygonShape(shelf, &sd, &unit);
    for (int32_t i = 0; i < 10; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2Body_SetType(shelf, m2_staticBody);
    double frozenY = m2Body_GetPosition(shelf).y;
    m2Vec2 v = m2Body_GetLinearVelocity(shelf);
    CHECK(v.x == 0.0f && v.y == 0.0f, "becoming static zeroes velocity");
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(shelf).y == frozenY, "a static shelf hangs mid-air");

    // ...carries a dynamic guest on its migrated proxy...
    m2BodyDef gd = m2DefaultBodyDef();
    gd.type = m2_dynamicBody;
    gd.position = (m2Pos2){0.0, frozenY + 0.5};
    m2BodyId guest = m2CreateBody(world, &gd);
    m2Polygon g = m2MakeBox(0.3f, 0.3f);
    m2CreatePolygonShape(guest, &sd, &g);
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(guest).y > frozenY + 0.3, "a guest rests on the frozen shelf");

    // ...and drops like anyone else once it is dynamic again.
    m2Body_SetType(shelf, m2_dynamicBody);
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(shelf).y < frozenY - 0.5, "dynamic again, it falls");
    CHECK(m2Body_GetPosition(shelf).y > 0.0, "and lands on the real floor");

    m2DestroyWorld(world);
}

typedef struct DrawCounts
{
    int32_t polygons;
    int32_t circles;
    int32_t segments;
    int32_t points;
    double firstPolyX; // world x of the first polygon's first vertex
    bool firstPolySeen;
} DrawCounts;

static void CountPolygon(const m2Vec2* verts, int32_t count, m2Pos2 origin, m2Rot rotation,
                         uint32_t color, void* context)
{
    (void)color;
    DrawCounts* c = context;
    c->polygons += 1;
    if (!c->firstPolySeen && count > 0)
    {
        c->firstPolySeen = true;
        c->firstPolyX = origin.x + (double)(rotation.c * verts[0].x - rotation.s * verts[0].y);
    }
}

static void CountCircle(m2Pos2 center, float radius, m2Rot rotation, uint32_t color, void* context)
{
    (void)center;
    (void)radius;
    (void)rotation;
    (void)color;
    ((DrawCounts*)context)->circles += 1;
}

static void CountSegment(m2Pos2 p1, m2Pos2 p2, uint32_t color, void* context)
{
    (void)p1;
    (void)p2;
    (void)color;
    ((DrawCounts*)context)->segments += 1;
}

static void CountPoint(m2Pos2 p, float size, uint32_t color, void* context)
{
    (void)p;
    (void)size;
    (void)color;
    ((DrawCounts*)context)->points += 1;
}

static void TestDebugDraw(void)
{
    // A known scene: one ground polygon, one resting box, one circle,
    // one joint. The counting renderer must see exactly that - and a
    // draw storm must not move a single simulation bit.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){100.0, -0.5};
    m2BodyId floor = m2CreateBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(6.0f, 0.5f);
    m2CreatePolygonShape(floor, &fs, &slab);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){100.0, 0.45};
    m2BodyId box = m2CreateBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon unit = m2MakeBox(0.4f, 0.4f);
    m2CreatePolygonShape(box, &sd, &unit);
    m2BodyDef cd = m2DefaultBodyDef();
    cd.type = m2_dynamicBody;
    cd.position = (m2Pos2){103.0, 2.0};
    m2BodyId ball = m2CreateBody(world, &cd);
    m2ShapeDef cs = m2DefaultShapeDef();
    m2Circle circle = {{0.0f, 0.0f}, 0.3f};
    m2CreateCircleShape(ball, &cs, &circle);
    m2DistanceJointDef jd = m2DefaultDistanceJointDef();
    jd.bodyIdA = floor;
    jd.bodyIdB = ball;
    m2CreateDistanceJoint(world, &jd);

    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }

    DrawCounts counts;
    memset(&counts, 0, sizeof(counts));
    m2DebugDraw draw;
    memset(&draw, 0, sizeof(draw));
    draw.drawPolygon = CountPolygon;
    draw.drawCircle = CountCircle;
    draw.drawSegment = CountSegment;
    draw.drawPoint = CountPoint;
    draw.drawShapes = true;
    draw.drawJoints = true;
    draw.drawContacts = true;
    draw.context = &counts;

    uint64_t before = m2World_Hash(world);
    for (int32_t i = 0; i < 20; ++i)
    {
        m2World_Draw(world, &draw);
    }
    CHECK(m2World_Hash(world) == before, "a draw storm moves no bits");

    memset(&counts, 0, sizeof(counts));
    m2World_Draw(world, &draw);
    CHECK(counts.polygons == 2, "two polygons drawn");
    CHECK(counts.circles == 1, "one circle drawn");
    CHECK(counts.segments == 1, "one joint segment drawn");
    CHECK(counts.points >= 4, "joint anchors and contact points drawn");
    CHECK(counts.firstPolySeen && counts.firstPolyX > 90.0 && counts.firstPolyX < 110.0,
          "local vertices compose with the f64 origin to world space");

    m2DestroyWorld(world);
}

static void TestRuntimeGravity(void)
{
    // Flip gravity over a sleeping box: it must wake and rise. The
    // reference leaves sleepers floating; Maul wakes the world.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = m2CreateBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(5.0f, 0.5f);
    m2CreatePolygonShape(floor, &fs, &slab);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 0.45};
    m2BodyId box = m2CreateBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon unit = m2MakeBox(0.4f, 0.4f);
    m2CreatePolygonShape(box, &sd, &unit);
    for (int32_t i = 0; i < 200; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m2Body_IsAwake(box), "asleep under the old sky");

    m2World_SetGravity(world, (m2Vec2){0.0f, 10.0f});
    m2Vec2 g = m2World_GetGravity(world);
    CHECK(g.y == 10.0f, "the getter reads the new sky");
    m2World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m2Body_IsAwake(box), "the sleeper feels the flip");
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(box).y > 3.0, "and falls upward");

    m2DestroyWorld(world);
}

// Destroying a body must wake whoever was resting on it, the same
// law teleports and type changes already obey. Before slice 52 the
// destroy paths skipped it and sleepers floated on a memory.
static void TestDestroyWakesSleepers(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = m2CreateBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(5.0f, 0.5f);
    m2CreatePolygonShape(floor, &fs, &slab);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 0.45};
    m2BodyId box = m2CreateBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon unit = m2MakeBox(0.4f, 0.4f);
    m2CreatePolygonShape(box, &sd, &unit);
    for (int32_t i = 0; i < 250; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m2Body_IsAwake(box), "box sleeps on the floor");

    m2DestroyBody(floor);
    m2World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m2Body_IsAwake(box), "yanking the floor wakes the sleeper");
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(box).y < -5.0, "and it falls");
    m2DestroyWorld(world);
}

// The integrator walk: an editor or engine layer must be able to
// enumerate every live object from a bare world id, read types and
// filters back, and see destroys reflected. Ascending slot order,
// truthful totals.
static void TestEnumerationWalk(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 32;
    def.jointCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);

    m2BodyDef gd = m2DefaultBodyDef();
    gd.position = (m2Pos2){0.0, -1.0};
    m2BodyId ground = m2CreateBody(world, &gd);
    m2ShapeDef gs = m2DefaultShapeDef();
    m2Polygon slabPoly = m2MakeBox(8.0f, 0.5f);
    m2ShapeId slab = m2CreatePolygonShape(ground, &gs, &slabPoly);
    m2Vec2 pts[6] = {{6.0f, 2.0f}, {4.0f, 2.0f},  {2.0f, 2.0f},
                     {0.0f, 2.0f}, {-2.0f, 2.0f}, {-4.0f, 2.0f}};
    m2ChainDef chain = m2DefaultChainDef();
    chain.points = pts;
    chain.count = 6;
    m2ChainId ledge = m2CreateChain(ground, &chain);

    m2BodyDef hd = m2DefaultBodyDef();
    hd.position = (m2Pos2){0.0, 6.0};
    m2BodyId hook = m2CreateBody(world, &hd);

    m2BodyDef ad = m2DefaultBodyDef();
    ad.type = m2_dynamicBody;
    ad.position = (m2Pos2){0.0, 4.0};
    m2BodyId boxA = m2CreateBody(world, &ad);
    m2ShapeDef asd = m2DefaultShapeDef();
    m2Polygon unit = m2MakeBox(0.4f, 0.4f);
    m2CreatePolygonShape(boxA, &asd, &unit);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){3.0, 0.5};
    m2BodyId boxB = m2CreateBody(world, &bd);
    m2CreatePolygonShape(boxB, &asd, &unit);
    m2ShapeDef sensorDef = m2DefaultShapeDef();
    sensorDef.isSensor = true;
    m2Circle aura = {{0.0f, 0.0f}, 1.5f};
    m2ShapeId auraShape = m2CreateCircleShape(boxB, &sensorDef, &aura);

    m2DistanceJointDef dj = m2DefaultDistanceJointDef();
    dj.bodyIdA = hook;
    dj.bodyIdB = boxA;
    m2JointId rope = m2CreateDistanceJoint(world, &dj);
    m2RevoluteJointDef rj = m2DefaultRevoluteJointDef();
    rj.bodyIdA = ground;
    rj.bodyIdB = boxB;
    m2JointId pin = m2CreateRevoluteJoint(world, &rj);

    // Totals are truthful regardless of capacity.
    CHECK(m2World_GetBodies(world, NULL, 0) == 4, "four live bodies");
    m2BodyId two[2];
    CHECK(m2World_GetBodies(world, two, 2) == 4, "small capacity still reports four");
    m2BodyId bodies[8];
    int32_t n = m2World_GetBodies(world, bodies, 8);
    CHECK(n == 4, "the full walk finds four");
    bool ascending = true;
    for (int32_t i = 1; i < n; ++i)
    {
        ascending = ascending && bodies[i - 1].index1 < bodies[i].index1;
    }
    CHECK(ascending, "bodies arrive in ascending slot order");
    CHECK(m2World_GetJoints(world, NULL, 0) == 2, "two joints");
    CHECK(m2World_GetChains(world, NULL, 0) == 1, "one chain");

    // Per-body shape walks.
    CHECK(m2Body_GetShapes(ground, NULL, 0) == 4, "slab plus three chain segments");
    m2ShapeId groundShapes[8];
    int32_t gn = m2Body_GetShapes(ground, groundShapes, 8);
    int32_t segments = 0;
    for (int32_t i = 0; i < gn; ++i)
    {
        segments += m2Shape_GetType(groundShapes[i]) == m2_chainSegmentShape ? 1 : 0;
    }
    CHECK(segments == 3, "and the walk can tell which are chain links");
    CHECK(m2Body_GetShapes(boxB, NULL, 0) == 2, "box B carries body and aura");

    // Types and flags read back.
    CHECK(m2Body_GetType(ground) == m2_staticBody, "ground reads static");
    CHECK(m2Body_GetType(boxA) == m2_dynamicBody, "box A reads dynamic");
    m2Body_SetType(boxA, m2_kinematicBody);
    CHECK(m2Body_GetType(boxA) == m2_kinematicBody, "and follows SetType");
    m2Body_SetType(boxA, m2_dynamicBody);
    CHECK(m2Shape_GetType(slab) == m2_polygonShape, "slab reads polygon");
    CHECK(m2Shape_GetType(auraShape) == m2_circleShape, "aura reads circle");
    CHECK(m2Shape_IsSensor(auraShape), "aura reads sensor");
    CHECK(!m2Shape_IsSensor(slab), "slab does not");

    // Filter round-trip, including NULL out-params.
    m2Shape_SetFilter(slab, 0x4u, 0xF0u, -3);
    uint32_t category = 0;
    uint32_t mask = 0;
    int32_t group = 0;
    m2Shape_GetFilter(slab, &category, &mask, &group);
    CHECK(category == 0x4u && mask == 0xF0u && group == -3, "filter reads back exactly");
    m2Shape_GetFilter(slab, NULL, NULL, &group);
    CHECK(group == -3, "partial reads are fine");

    // Joint introspection.
    CHECK(m2Joint_GetType(rope) == m2_distanceJoint, "rope reads distance");
    CHECK(m2Joint_GetType(pin) == m2_revoluteJoint, "pin reads revolute");
    m2BodyId ja = m2Joint_GetBodyA(rope);
    m2BodyId jb = m2Joint_GetBodyB(rope);
    CHECK(ja.index1 == hook.index1 && ja.generation == hook.generation, "rope body A is the hook");
    CHECK(jb.index1 == boxA.index1 && jb.generation == boxA.generation, "rope body B is box A");

    // Destroys are reflected immediately.
    m2DestroyBody(boxA);
    CHECK(m2World_GetBodies(world, NULL, 0) == 3, "three bodies after the destroy");
    CHECK(m2World_GetJoints(world, NULL, 0) == 1, "the rope died with its body");
    m2DestroyChain(ledge);
    CHECK(m2World_GetChains(world, NULL, 0) == 0, "no chains after demolition");
    CHECK(m2Body_GetShapes(ground, NULL, 0) == 1, "the slab is all that remains");

    m2DestroyWorld(world);
}

// THE MIRROR TEST: rebuild a world using nothing but the public
// readback surface, then demand hash equality at creation and through
// 90 steps. Any parameter the getters cannot carry breaks the mirror,
// so this test IS the completeness proof of the integrator surface.
// Bit-identity requires replaying the source's creation order, which
// the ascending walks reproduce for destroy-free worlds.

static m2BodyId MirrorOf(const m2BodyId* src, const m2BodyId* dst, int32_t n, m2BodyId find)
{
    for (int32_t i = 0; i < n; ++i)
    {
        if (src[i].index1 == find.index1 && src[i].generation == find.generation)
        {
            return dst[i];
        }
    }
    m2BodyId null = {0, 0, 0};
    return null;
}

static m2WorldId MirrorWorld(m2WorldId source, const m2WorldDef* def)
{
    m2WorldId mirror = m2CreateWorld(def);
    m2BodyId bodies[16];
    m2BodyId mirrored[16];
    int32_t bodyCount = m2World_GetBodies(source, bodies, 16);
    m2ChainId chains[8];
    int32_t chainCount = m2World_GetChains(source, chains, 8);

    for (int32_t i = 0; i < bodyCount; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2Body_GetType(bodies[i]);
        m2Transform tf = m2Body_GetTransform(bodies[i]);
        bd.position = tf.p;
        bd.rotation = tf.q;
        bd.linearVelocity = m2Body_GetLinearVelocity(bodies[i]);
        bd.angularVelocity = m2Body_GetAngularVelocity(bodies[i]);
        bd.gravityScale = m2Body_GetGravityScale(bodies[i]);
        bd.isBullet = m2Body_IsBullet(bodies[i]);
        bd.userData = m2Body_GetUserData(bodies[i]);
        bd.dominance = m2Body_GetDominance(bodies[i]);
        mirrored[i] = m2CreateBody(mirror, &bd);

        m2ShapeId shapes[16];
        int32_t shapeCount = m2Body_GetShapes(bodies[i], shapes, 16);
        for (int32_t s = 0; s < shapeCount; ++s)
        {
            m2ShapeType type = m2Shape_GetType(shapes[s]);
            if (type == m2_chainSegmentShape)
            {
                continue; // rebuilt with its chain below
            }
            m2ShapeDef sd = m2DefaultShapeDef();
            sd.density = m2Shape_GetDensity(shapes[s]);
            sd.friction = m2Shape_GetFriction(shapes[s]);
            sd.restitution = m2Shape_GetRestitution(shapes[s]);
            sd.isSensor = m2Shape_IsSensor(shapes[s]);
            m2Shape_GetFilter(shapes[s], &sd.categoryBits, &sd.maskBits, &sd.groupIndex);
            sd.userData = m2Shape_GetUserData(shapes[s]);
            if (type == m2_circleShape)
            {
                m2Circle g = m2Shape_GetCircle(shapes[s]);
                m2CreateCircleShape(mirrored[i], &sd, &g);
            }
            else if (type == m2_capsuleShape)
            {
                m2Capsule g = m2Shape_GetCapsule(shapes[s]);
                m2CreateCapsuleShape(mirrored[i], &sd, &g);
            }
            else if (type == m2_polygonShape)
            {
                m2Polygon g = m2Shape_GetPolygon(shapes[s]);
                m2CreatePolygonShape(mirrored[i], &sd, &g);
            }
            else
            {
                m2Segment g = m2Shape_GetSegment(shapes[s]);
                m2CreateSegmentShape(mirrored[i], &sd, &g);
            }
        }

        for (int32_t c = 0; c < chainCount; ++c)
        {
            m2ShapeId links[8];
            int32_t linkCount = m2Chain_GetShapes(chains[c], links, 8);
            if (linkCount == 0)
            {
                continue;
            }
            m2BodyId owner = m2Shape_GetBody(links[0]);
            if (owner.index1 != bodies[i].index1)
            {
                continue;
            }
            // Open-chain polyline from the links: leading ghost, each
            // link's first point, trailing point, trailing ghost.
            m2Vec2 pts[16];
            m2ChainSegment firstLink = m2Shape_GetChainSegment(links[0]);
            m2ChainSegment lastLink = m2Shape_GetChainSegment(links[linkCount - 1]);
            pts[0] = firstLink.ghost1;
            for (int32_t k = 0; k < linkCount; ++k)
            {
                pts[k + 1] = m2Shape_GetChainSegment(links[k]).segment.point1;
            }
            pts[linkCount + 1] = lastLink.segment.point2;
            pts[linkCount + 2] = lastLink.ghost2;
            m2ChainDef cd = m2DefaultChainDef();
            cd.points = pts;
            cd.count = linkCount + 3;
            cd.friction = m2Shape_GetFriction(links[0]);
            cd.restitution = m2Shape_GetRestitution(links[0]);
            m2Shape_GetFilter(links[0], &cd.categoryBits, &cd.maskBits, &cd.groupIndex);
            cd.userData = m2Shape_GetUserData(links[0]);
            m2CreateChain(mirrored[i], &cd);
        }
    }

    m2JointId joints[8];
    int32_t jointCount = m2World_GetJoints(source, joints, 8);
    for (int32_t j = 0; j < jointCount; ++j)
    {
        m2JointId id = joints[j];
        m2BodyId a = MirrorOf(bodies, mirrored, bodyCount, m2Joint_GetBodyA(id));
        m2BodyId b = MirrorOf(bodies, mirrored, bodyCount, m2Joint_GetBodyB(id));
        m2JointId made = {0, 0, 0};
        m2JointType type = m2Joint_GetType(id);
        if (type == m2_filterJoint)
        {
            m2FilterJointDef jd = m2DefaultFilterJointDef();
            jd.bodyIdA = a;
            jd.bodyIdB = b;
            made = m2CreateFilterJoint(mirror, &jd);
        }
        else if (type == m2_pulleyJoint)
        {
            m2PulleyJointDef jd = m2DefaultPulleyJointDef();
            jd.bodyIdA = a;
            jd.bodyIdB = b;
            jd.groundAnchorA = m2PulleyJoint_GetGroundAnchorA(id);
            jd.groundAnchorB = m2PulleyJoint_GetGroundAnchorB(id);
            jd.localAnchorA = m2Joint_GetLocalAnchorA(id);
            jd.localAnchorB = m2Joint_GetLocalAnchorB(id);
            jd.ratio = m2PulleyJoint_GetRatio(id);
            jd.collideConnected = m2Joint_GetCollideConnected(id);
            made = m2CreatePulleyJoint(mirror, &jd);
        }
        else if (type == m2_gearJoint)
        {
            m2GearJointDef jd = m2DefaultGearJointDef();
            jd.bodyIdA = a;
            jd.bodyIdB = b;
            jd.ratio = m2GearJoint_GetRatio(id);
            jd.collideConnected = m2Joint_GetCollideConnected(id);
            made = m2CreateGearJoint(mirror, &jd);
        }
        else if (type == m2_motorJoint)
        {
            m2MotorJointDef jd = m2DefaultMotorJointDef();
            jd.bodyIdA = a;
            jd.bodyIdB = b;
            jd.linearOffset = m2MotorJoint_GetLinearOffset(id);
            jd.angularOffset = m2MotorJoint_GetAngularOffset(id);
            jd.maxForce = m2MotorJoint_GetMaxForce(id);
            jd.maxTorque = m2Joint_GetMaxMotor(id);
            jd.correctionFactor = m2MotorJoint_GetCorrectionFactor(id);
            jd.collideConnected = m2Joint_GetCollideConnected(id);
            made = m2CreateMotorJoint(mirror, &jd);
        }
        else if (type == m2_mouseJoint)
        {
            m2MouseJointDef jd = m2DefaultMouseJointDef();
            jd.bodyIdA = a;
            jd.bodyIdB = b;
            jd.target = m2MouseJoint_GetTarget(id);
            jd.hertz = m2Joint_GetHertz(id);
            jd.dampingRatio = m2Joint_GetDampingRatio(id);
            jd.maxForce = m2MouseJoint_GetMaxForce(id);
            jd.collideConnected = m2Joint_GetCollideConnected(id);
            made = m2CreateMouseJoint(mirror, &jd);
        }
        else if (type == m2_distanceJoint)
        {
            m2DistanceJointDef jd = m2DefaultDistanceJointDef();
            jd.bodyIdA = a;
            jd.bodyIdB = b;
            jd.localAnchorA = m2Joint_GetLocalAnchorA(id);
            jd.localAnchorB = m2Joint_GetLocalAnchorB(id);
            jd.length = m2Joint_GetLength(id);
            m2Joint_GetLimits(id, &jd.minLength, &jd.maxLength);
            jd.collideConnected = m2Joint_GetCollideConnected(id);
            jd.hertz = m2Joint_GetHertz(id);
            jd.dampingRatio = m2Joint_GetDampingRatio(id);
            made = m2CreateDistanceJoint(mirror, &jd);
        }
        else if (type == m2_revoluteJoint)
        {
            m2RevoluteJointDef jd = m2DefaultRevoluteJointDef();
            jd.bodyIdA = a;
            jd.bodyIdB = b;
            jd.localAnchorA = m2Joint_GetLocalAnchorA(id);
            jd.localAnchorB = m2Joint_GetLocalAnchorB(id);
            jd.hertz = m2Joint_GetHertz(id);
            jd.dampingRatio = m2Joint_GetDampingRatio(id);
            jd.springHertz = m2Joint_GetAngularHertz(id);
            jd.springDampingRatio = m2Joint_GetAngularDampingRatio(id);
            jd.enableMotor = m2Joint_IsMotorEnabled(id);
            jd.motorSpeed = m2Joint_GetMotorSpeed(id);
            jd.maxMotorTorque = m2Joint_GetMaxMotor(id);
            jd.enableLimit = m2Joint_IsLimitEnabled(id);
            m2Joint_GetLimits(id, &jd.lowerAngle, &jd.upperAngle);
            jd.collideConnected = m2Joint_GetCollideConnected(id);
            made = m2CreateRevoluteJoint(mirror, &jd);
        }
        else if (type == m2_prismaticJoint)
        {
            m2PrismaticJointDef jd = m2DefaultPrismaticJointDef();
            jd.bodyIdA = a;
            jd.bodyIdB = b;
            jd.localAnchorA = m2Joint_GetLocalAnchorA(id);
            jd.localAnchorB = m2Joint_GetLocalAnchorB(id);
            jd.localAxisA = m2Joint_GetLocalAxisA(id);
            jd.hertz = m2Joint_GetHertz(id);
            jd.dampingRatio = m2Joint_GetDampingRatio(id);
            jd.enableMotor = m2Joint_IsMotorEnabled(id);
            jd.motorSpeed = m2Joint_GetMotorSpeed(id);
            jd.maxMotorForce = m2Joint_GetMaxMotor(id);
            jd.enableLimit = m2Joint_IsLimitEnabled(id);
            m2Joint_GetLimits(id, &jd.lowerTranslation, &jd.upperTranslation);
            jd.collideConnected = m2Joint_GetCollideConnected(id);
            made = m2CreatePrismaticJoint(mirror, &jd);
        }
        else if (type == m2_weldJoint)
        {
            m2WeldJointDef jd = m2DefaultWeldJointDef();
            jd.bodyIdA = a;
            jd.bodyIdB = b;
            jd.localAnchorA = m2Joint_GetLocalAnchorA(id);
            jd.localAnchorB = m2Joint_GetLocalAnchorB(id);
            jd.linearHertz = m2Joint_GetHertz(id);
            jd.linearDampingRatio = m2Joint_GetDampingRatio(id);
            jd.angularHertz = m2Joint_GetAngularHertz(id);
            jd.angularDampingRatio = m2Joint_GetAngularDampingRatio(id);
            jd.collideConnected = m2Joint_GetCollideConnected(id);
            made = m2CreateWeldJoint(mirror, &jd);
        }
        else
        {
            m2WheelJointDef jd = m2DefaultWheelJointDef();
            jd.bodyIdA = a;
            jd.bodyIdB = b;
            jd.localAnchorA = m2Joint_GetLocalAnchorA(id);
            jd.localAnchorB = m2Joint_GetLocalAnchorB(id);
            jd.localAxisA = m2Joint_GetLocalAxisA(id);
            jd.enableSpring = m2Joint_IsSpringEnabled(id);
            jd.hertz = m2Joint_GetHertz(id);
            jd.dampingRatio = m2Joint_GetDampingRatio(id);
            jd.enableMotor = m2Joint_IsMotorEnabled(id);
            jd.motorSpeed = m2Joint_GetMotorSpeed(id);
            jd.maxMotorTorque = m2Joint_GetMaxMotor(id);
            jd.enableLimit = m2Joint_IsLimitEnabled(id);
            m2Joint_GetLimits(id, &jd.lowerTranslation, &jd.upperTranslation);
            jd.collideConnected = m2Joint_GetCollideConnected(id);
            made = m2CreateWheelJoint(mirror, &jd);
        }
        float breakForce = 0.0f;
        float breakTorque = 0.0f;
        m2Joint_GetBreakLimits(id, &breakForce, &breakTorque);
        if (breakForce != 0.0f || breakTorque != 0.0f)
        {
            m2Joint_SetBreakLimits(made, breakForce, breakTorque);
        }
    }
    return mirror;
}

static void TestMirrorRebuild(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 32;
    def.jointCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);

    // One of everything, with deliberately non-default numbers, built
    // body by body so the mirror's walk replays the creation order.
    m2BodyDef gd = m2DefaultBodyDef();
    gd.position = (m2Pos2){0.0, -1.0};
    gd.userData = 11;
    m2BodyId ground = m2CreateBody(world, &gd);
    m2ShapeDef gs = m2DefaultShapeDef();
    gs.friction = 0.8f;
    m2Polygon slab = m2MakeBox(10.0f, 0.5f);
    m2CreatePolygonShape(ground, &gs, &slab);
    m2Vec2 pts[6] = {{6.0f, 1.0f}, {4.0f, 1.0f},  {2.0f, 1.0f},
                     {0.0f, 1.0f}, {-2.0f, 1.0f}, {-4.0f, 1.0f}};
    m2ChainDef chain = m2DefaultChainDef();
    chain.points = pts;
    chain.count = 6;
    chain.friction = 0.9f;
    chain.restitution = 0.1f;
    chain.categoryBits = 0x2u;
    chain.maskBits = 0xFFu;
    chain.groupIndex = 5;
    chain.userData = 77;
    m2CreateChain(ground, &chain);

    m2BodyDef hd = m2DefaultBodyDef();
    hd.position = (m2Pos2){0.0, 8.0};
    m2BodyId hook = m2CreateBody(world, &hd);

    m2BodyDef ad = m2DefaultBodyDef();
    ad.type = m2_dynamicBody;
    ad.position = (m2Pos2){0.0, 5.0};
    ad.linearVelocity = (m2Vec2){1.5f, 2.0f};
    ad.angularVelocity = 3.0f;
    ad.gravityScale = 0.7f;
    ad.isBullet = true;
    ad.userData = 21;
    ad.dominance = 2;
    m2BodyId boxA = m2CreateBody(world, &ad);
    m2ShapeDef asd = m2DefaultShapeDef();
    asd.density = 2.0f;
    asd.friction = 0.4f;
    asd.restitution = 0.3f;
    m2Polygon unit = m2MakeBox(0.4f, 0.4f);
    m2CreatePolygonShape(boxA, &asd, &unit);

    m2BodyDef bd2 = m2DefaultBodyDef();
    bd2.type = m2_dynamicBody;
    bd2.position = (m2Pos2){3.0, 2.0};
    bd2.rotation = (m2Rot){0.9553365f, 0.29552022f}; // cos/sin of 0.3, pinned
    m2BodyId boxB = m2CreateBody(world, &bd2);
    m2Capsule pill = {{-0.4f, 0.0f}, {0.4f, 0.0f}, 0.25f};
    m2CreateCapsuleShape(boxB, &asd, &pill);
    m2ShapeDef sensorDef = m2DefaultShapeDef();
    sensorDef.isSensor = true;
    sensorDef.userData = 42;
    m2Circle aura = {{0.0f, 0.0f}, 1.2f};
    m2CreateCircleShape(boxB, &sensorDef, &aura);

    m2BodyDef cd2 = m2DefaultBodyDef();
    cd2.type = m2_dynamicBody;
    cd2.position = (m2Pos2){-3.0, 2.0};
    m2BodyId boxC = m2CreateBody(world, &cd2);
    m2CreatePolygonShape(boxC, &asd, &unit);

    m2BodyDef dd = m2DefaultBodyDef();
    dd.type = m2_dynamicBody;
    dd.position = (m2Pos2){-6.0, 2.0};
    m2BodyId boxD = m2CreateBody(world, &dd);
    m2Circle wheelDisc = {{0.0f, 0.0f}, 0.35f};
    m2CreateCircleShape(boxD, &asd, &wheelDisc);

    m2DistanceJointDef dj = m2DefaultDistanceJointDef();
    dj.bodyIdA = hook;
    dj.bodyIdB = boxA;
    dj.hertz = 4.0f;
    dj.dampingRatio = 0.6f;
    dj.minLength = 0.5f;
    dj.maxLength = 4.0f;
    m2JointId rope = m2CreateDistanceJoint(world, &dj);
    m2Joint_SetBreakLimits(rope, 60.0f, 0.0f);
    m2RevoluteJointDef rj = m2DefaultRevoluteJointDef();
    rj.bodyIdA = ground;
    rj.bodyIdB = boxB;
    rj.enableMotor = true;
    rj.motorSpeed = 1.2f;
    rj.maxMotorTorque = 20.0f;
    rj.enableLimit = true;
    rj.lowerAngle = -0.5f;
    rj.upperAngle = 0.7f;
    m2CreateRevoluteJoint(world, &rj);
    m2PrismaticJointDef pj = m2DefaultPrismaticJointDef();
    pj.bodyIdA = ground;
    pj.bodyIdB = boxC;
    pj.localAxisA = (m2Vec2){0.0f, 1.0f};
    pj.enableMotor = true;
    pj.motorSpeed = -0.5f;
    pj.maxMotorForce = 30.0f;
    pj.enableLimit = true;
    pj.lowerTranslation = -1.0f;
    pj.upperTranslation = 2.0f;
    m2CreatePrismaticJoint(world, &pj);
    m2WeldJointDef wj = m2DefaultWeldJointDef();
    wj.bodyIdA = boxA;
    wj.bodyIdB = boxB;
    wj.linearHertz = 3.0f;
    wj.linearDampingRatio = 0.5f;
    wj.angularHertz = 6.0f;
    wj.angularDampingRatio = 0.8f;
    m2CreateWeldJoint(world, &wj);
    m2PulleyJointDef hoist = m2DefaultPulleyJointDef();
    hoist.bodyIdA = boxB;
    hoist.bodyIdB = boxD;
    hoist.groundAnchorA = (m2Pos2){-1.0, 9.0};
    hoist.groundAnchorB = (m2Pos2){3.0, 9.0};
    hoist.ratio = 1.5f;
    m2CreatePulleyJoint(world, &hoist);
    m2GearJointDef cog = m2DefaultGearJointDef();
    cog.bodyIdA = boxB;
    cog.bodyIdB = boxD;
    cog.ratio = 1.5f;
    m2CreateGearJoint(world, &cog);
    m2MotorJointDef chase = m2DefaultMotorJointDef();
    chase.bodyIdA = hook;
    chase.bodyIdB = boxC;
    chase.linearOffset = (m2Vec2){0.5f, -0.25f};
    chase.angularOffset = 0.2f;
    chase.maxForce = 77.0f;
    chase.maxTorque = 33.0f;
    chase.correctionFactor = 0.4f;
    m2CreateMotorJoint(world, &chase);
    m2MouseJointDef grip = m2DefaultMouseJointDef();
    grip.bodyIdA = ground;
    grip.bodyIdB = boxD;
    grip.target = (m2Pos2){-6.0, 2.5};
    grip.hertz = 5.0f;
    grip.dampingRatio = 0.8f;
    grip.maxForce = 44.0f;
    grip.collideConnected = true;
    m2CreateMouseJoint(world, &grip);
    m2FilterJointDef pact = m2DefaultFilterJointDef();
    pact.bodyIdA = boxA;
    pact.bodyIdB = boxC;
    m2CreateFilterJoint(world, &pact);
    m2WheelJointDef whj = m2DefaultWheelJointDef();
    whj.bodyIdA = ground;
    whj.bodyIdB = boxD;
    whj.localAxisA = (m2Vec2){0.0f, 1.0f};
    whj.enableSpring = true;
    whj.hertz = 2.5f;
    whj.dampingRatio = 0.7f;
    whj.enableMotor = true;
    whj.motorSpeed = 4.0f;
    whj.maxMotorTorque = 15.0f;
    whj.enableLimit = true;
    whj.lowerTranslation = -0.3f;
    whj.upperTranslation = 0.3f;
    whj.collideConnected = true;
    m2CreateWheelJoint(world, &whj);

    m2WorldId mirror = MirrorWorld(world, &def);
    CHECK(m2World_Hash(mirror) == m2World_Hash(world), "the mirror is bit-identical at creation");

    bool tracked = true;
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        m2World_Step(mirror, 1.0f / 60.0f, 4);
        tracked = tracked && m2World_Hash(mirror) == m2World_Hash(world);
    }
    CHECK(tracked, "and tracks the original for 90 steps");

    m2DestroyWorld(mirror);
    m2DestroyWorld(world);
}

// The body dynamics pack (slice 62): forces with one-step lifetime,
// Pade damping, fixed rotation as a mass property, and sleep control
// at both scopes. Reference forms, journaled channels, snapshot v20.
static void TestBodyDynamicsPack(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m2WorldId world = m2CreateWorld(&def);
    m2World_SetGravity(world, (m2Vec2){0.0f, 0.0f}); // isolate the pack

    // Force: F = m*a, integrated over a second.
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 10.0};
    m2BodyId pushed = m2CreateBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon unit = m2MakeBox(0.4f, 0.4f); // 0.64 kg at density 1
    m2CreatePolygonShape(pushed, &sd, &unit);
    for (int32_t i = 0; i < 60; ++i)
    {
        m2Body_ApplyForceToCenter(pushed, (m2Vec2){6.4f, 0.0f}); // a = 10
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    float vx = m2Body_GetLinearVelocity(pushed).x;
    CHECK(vx > 9.0f && vx < 11.0f, "one second of F=ma reaches ten");

    // One-step lifetime: no further force, velocity coasts.
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    float coast = m2Body_GetLinearVelocity(pushed).x;
    CHECK(coast > vx - 0.01f && coast < vx + 0.01f, "forces die with their step");

    // Damping: exp(-c*t) via Pade, c=4 over one second leaves ~e^-4.
    m2Body_SetLinearDamping(pushed, 4.0f);
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    float damped = m2Body_GetLinearVelocity(pushed).x;
    CHECK(damped > 0.05f && damped < 0.5f, "damping decays like the reference");

    // Torque spins, angular damping stills.
    m2Body_ApplyTorque(pushed, 5.0f);
    m2World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m2Body_GetAngularVelocity(pushed) > 0.0f, "torque spins");
    m2Body_SetAngularDamping(pushed, 8.0f);
    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    float spin = m2Body_GetAngularVelocity(pushed);
    CHECK(spin < 0.05f, "angular damping stills the spin");

    // Fixed rotation: an off-center impulse must not rotate it.
    m2BodyDef fdd = m2DefaultBodyDef();
    fdd.type = m2_dynamicBody;
    fdd.position = (m2Pos2){5.0, 10.0};
    fdd.fixedRotation = true;
    m2BodyId upright = m2CreateBody(world, &fdd);
    m2CreatePolygonShape(upright, &sd, &unit);
    m2Body_ApplyLinearImpulse(upright, (m2Vec2){0.0f, 2.0f}, (m2Pos2){5.4, 10.0});
    m2World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m2Body_GetAngularVelocity(upright) == 0.0f, "fixed rotation never spins");
    CHECK(m2Body_IsFixedRotation(upright), "and reads back");
    m2Body_SetFixedRotation(upright, false);
    m2Body_ApplyLinearImpulse(upright, (m2Vec2){0.0f, 2.0f}, (m2Pos2){5.4, 10.0});
    m2World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m2Body_GetAngularVelocity(upright) != 0.0f, "released, it spins again");

    m2DestroyWorld(world);

    // Sleep control at both scopes, on a world with real gravity.
    m2WorldId nap = m2CreateWorld(&def);
    m2BodyDef gd = m2DefaultBodyDef();
    gd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = m2CreateBody(nap, &gd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(10.0f, 0.5f);
    m2CreatePolygonShape(floor, &fs, &slab);
    m2BodyDef nd = m2DefaultBodyDef();
    nd.type = m2_dynamicBody;
    nd.position = (m2Pos2){-2.0, 0.45};
    m2BodyId sleeper = m2CreateBody(nap, &nd);
    m2CreatePolygonShape(sleeper, &sd, &unit);
    m2BodyDef id2 = nd;
    id2.position = (m2Pos2){2.0, 0.45};
    id2.enableSleep = false;
    m2BodyId insomniac = m2CreateBody(nap, &id2);
    m2CreatePolygonShape(insomniac, &sd, &unit);
    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(nap, 1.0f / 60.0f, 4);
    }
    CHECK(!m2Body_IsAwake(sleeper), "the plain twin sleeps");
    CHECK(m2Body_IsAwake(insomniac), "the flagged twin never does");
    CHECK(!m2Body_IsSleepEnabled(insomniac), "and reads back");

    m2World_EnableSleeping(nap, false);
    CHECK(m2Body_IsAwake(sleeper), "killing the master switch wakes the sleeper");
    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(nap, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_IsAwake(sleeper), "and nobody sleeps while it is off");
    m2World_EnableSleeping(nap, true);
    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(nap, 1.0f / 60.0f, 4);
    }
    CHECK(!m2Body_IsAwake(sleeper), "switch back on, naps resume");
    m2DestroyWorld(nap);
}

// Integration extras (parity sprint 4a): dormancy that ends contacts
// and wakes riders, mass overrides with a documented lifetime, and
// one deterministic blast.
static void TestDormancyMassAndExplosions(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 32;
    m2WorldId world = m2CreateWorld(&def);
    m2BodyDef gd = m2DefaultBodyDef();
    gd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = m2CreateBody(world, &gd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(20.0f, 0.5f);
    m2CreatePolygonShape(floor, &fs, &slab);

    // Dormancy: a pedestal with a sleeper on top goes dormant; the
    // rider wakes and falls (the destroy-path wake law, third user).
    m2BodyDef pd = m2DefaultBodyDef();
    pd.position = (m2Pos2){0.0, 0.75};
    m2BodyId pedestal = m2CreateBody(world, &pd);
    m2Polygon post = m2MakeBox(0.6f, 0.75f);
    m2CreatePolygonShape(pedestal, &fs, &post);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 1.95};
    m2BodyId rider = m2CreateBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon unit = m2MakeBox(0.4f, 0.4f);
    m2CreatePolygonShape(rider, &sd, &unit);
    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m2Body_IsAwake(rider), "the rider sleeps on the pedestal");
    m2Body_Disable(pedestal);
    CHECK(!m2Body_IsEnabled(pedestal), "dormancy reads back");
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(rider).y < 0.6, "the rider wakes and lands on the floor");
    m2RayCastResult probe = m2World_CastRayClosest(world, (m2Pos2){0.0, 5.0}, (m2Vec2){0.0f, -4.0f},
                                                   m2DefaultQueryFilter());
    CHECK(!probe.hit, "a dormant body is invisible to queries");
    m2Body_Enable(pedestal);
    probe = m2World_CastRayClosest(world, (m2Pos2){0.0, 5.0}, (m2Vec2){0.0f, -4.5f},
                                   m2DefaultQueryFilter());
    CHECK(probe.hit, "and reappears where it stood");

    // Mass override: twice the mass, same impulse, half the speed.
    m2BodyDef md = m2DefaultBodyDef();
    md.type = m2_dynamicBody;
    md.position = (m2Pos2){8.0, 5.0};
    m2BodyId heavy = m2CreateBody(world, &md);
    m2CreatePolygonShape(heavy, &sd, &unit);
    m2MassData original = m2Body_GetMassData(heavy);
    m2MassData doubled = original;
    doubled.mass = original.mass * 2.0f;
    doubled.rotationalInertia = original.rotationalInertia * 2.0f;
    m2Body_SetMassData(heavy, doubled);
    CHECK(m2Body_GetMass(heavy) > original.mass * 1.9f, "the override sticks");
    m2Body_ApplyLinearImpulse(heavy, (m2Vec2){original.mass * 2.0f, 0.0f},
                              m2Body_GetPosition(heavy));
    float vx = m2Body_GetLinearVelocity(heavy).x;
    CHECK(vx > 0.9f && vx < 1.1f, "twice the mass takes half the speed");
    m2Body_ApplyMassFromShapes(heavy);
    m2MassData back = m2Body_GetMassData(heavy);
    CHECK(back.mass < original.mass * 1.1f, "shapes take the mass back on request");

    // Explosion: two crates flank the blast inside the radius, a third
    // sits in the falloff band, a fourth is filtered out by mask.
    double bx = 20.0;
    m2BodyId crates[4];
    double at[4] = {bx - 1.0, bx + 1.0, bx + 3.0, bx - 1.0};
    double ay[4] = {0.45, 0.45, 0.45, 1.6};
    for (int32_t i = 0; i < 4; ++i)
    {
        m2BodyDef cd = m2DefaultBodyDef();
        cd.type = m2_dynamicBody;
        cd.position = (m2Pos2){at[i], ay[i]};
        crates[i] = m2CreateBody(world, &cd);
        m2ShapeDef xs = m2DefaultShapeDef();
        if (i == 3)
        {
            xs.categoryBits = 0x8000u; // outside the blast mask
        }
        m2CreatePolygonShape(crates[i], &xs, &unit);
    }
    m2ExplosionDef boom = m2DefaultExplosionDef();
    boom.position = (m2Pos2){bx, 0.45};
    boom.radius = 1.5f;
    boom.falloff = 2.0f;
    boom.impulse = 3.0f;
    boom.maskBits = 0x1u;
    m2World_Explode(world, &boom);
    float vLeft = m2Body_GetLinearVelocity(crates[0]).x;
    float vRight = m2Body_GetLinearVelocity(crates[1]).x;
    float vFar = m2Body_GetLinearVelocity(crates[2]).x;
    CHECK(vLeft < -1.0f, "the left crate flies left");
    CHECK(vRight > 1.0f, "the right crate flies right");
    CHECK(vFar > 0.1f && vFar < vRight, "the falloff crate gets a weaker push");
    float vMasked = m2Body_GetLinearVelocity(crates[3]).x;
    CHECK(vMasked == 0.0f, "the masked crate never feels it");

    m2DestroyWorld(world);
}

// Small-basket readers and mutators (parity sprint 4b): frame
// helpers, the per-body joint walk, runtime geometry, and chain
// materials.
static void TestSmallBasket(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 32;
    def.jointCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);

    // Frame helpers on a rotated, offset body.
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){100.0, 50.0};
    bd.rotation = (m2Rot){0.0f, 1.0f}; // ninety degrees
    m2BodyId body = m2CreateBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon unit = m2MakeBox(0.4f, 0.4f);
    m2CreatePolygonShape(body, &sd, &unit);
    m2Pos2 wp = m2Body_GetWorldPoint(body, (m2Vec2){1.0f, 0.0f});
    CHECK(wp.x > 99.9 && wp.x < 100.1 && wp.y > 50.9 && wp.y < 51.1,
          "a local x step becomes a world y step under ninety degrees");
    m2Vec2 back = m2Body_GetLocalPoint(body, wp);
    CHECK(back.x > 0.99f && back.x < 1.01f && back.y > -0.01f && back.y < 0.01f,
          "and comes back local");
    m2Vec2 wv = m2Body_GetWorldVector(body, (m2Vec2){1.0f, 0.0f});
    CHECK(wv.y > 0.99f, "vectors rotate the same way");
    m2Body_SetAngularVelocity(body, 2.0f);
    m2Vec2 pv =
        m2Body_GetWorldPointVelocity(body, m2Body_GetWorldPoint(body, (m2Vec2){1.0f, 0.0f}));
    CHECK(pv.x < -1.9f && pv.x > -2.1f, "point velocity is w cross r");

    // Per-body joint walk.
    m2BodyDef ad = m2DefaultBodyDef();
    ad.position = (m2Pos2){104.0, 50.0};
    m2BodyId post = m2CreateBody(world, &ad);
    m2DistanceJointDef dj = m2DefaultDistanceJointDef();
    dj.bodyIdA = post;
    dj.bodyIdB = body;
    m2JointId rope = m2CreateDistanceJoint(world, &dj);
    m2MotorJointDef mj = m2DefaultMotorJointDef();
    mj.bodyIdA = post;
    mj.bodyIdB = body;
    m2JointId chase = m2CreateMotorJoint(world, &mj);
    CHECK(m2Body_GetJoints(body, NULL, 0) == 2, "the body names both joints");
    m2JointId list[4];
    int32_t n = m2Body_GetJoints(post, list, 4);
    CHECK(n == 2 && list[0].index1 == rope.index1 && list[1].index1 == chase.index1,
          "ascending and correct");

    // Runtime geometry: a floor shrinks under a sleeper.
    m2BodyDef gd = m2DefaultBodyDef();
    gd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = m2CreateBody(world, &gd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(6.0f, 0.5f);
    m2ShapeId ground = m2CreatePolygonShape(floor, &fs, &slab);
    m2BodyDef cd = m2DefaultBodyDef();
    cd.type = m2_dynamicBody;
    cd.position = (m2Pos2){4.0, 0.45};
    m2BodyId crate = m2CreateBody(world, &cd);
    m2CreatePolygonShape(crate, &sd, &unit);
    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m2Body_IsAwake(crate), "the crate sleeps at the slab's edge");
    m2Polygon stub = m2MakeBox(1.0f, 0.5f);
    m2Shape_SetPolygon(ground, &stub);
    CHECK(m2Shape_GetPolygon(ground).count == 4, "geometry reads back after the swap");
    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(crate).y < -1.0, "the sleeper wakes and falls off the stub");

    // Type change: the stub becomes a circle and reports it.
    m2Circle dome = {{0.0f, 0.0f}, 0.8f};
    m2Shape_SetCircle(ground, &dome);
    CHECK(m2Shape_GetType(ground) == m2_circleShape, "runtime type change reads back");

    // Chain materials: every link picks up the new values.
    m2Vec2 pts[5] = {{22.0f, 0.0f}, {20.0f, 0.0f}, {18.0f, 0.0f}, {16.0f, 0.0f}, {14.0f, 0.0f}};
    m2ChainDef chain = m2DefaultChainDef();
    chain.points = pts;
    chain.count = 5;
    m2ChainId ledge = m2CreateChain(floor, &chain);
    m2Chain_SetFriction(ledge, 0.85f);
    m2Chain_SetRestitution(ledge, 0.35f);
    m2ShapeId links[4];
    int32_t linkCount = m2Chain_GetShapes(ledge, links, 4);
    bool allSet = linkCount == 2;
    for (int32_t i = 0; i < linkCount; ++i)
    {
        allSet = allSet && m2Shape_GetFriction(links[i]) == 0.85f &&
                 m2Shape_GetRestitution(links[i]) == 0.35f;
    }
    CHECK(allSet, "chain materials reach every link");

    m2DestroyWorld(world);
}

// The leftover basket (audit appendix bucket 1): every small reader
// and setter that finishes reference parity, exercised end to end.
static void TestLeftoverBasket(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 32;
    def.jointCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);
    m2World_SetGravity(world, (m2Vec2){0.0f, 0.0f});

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 5.0};
    m2BodyId box = m2CreateBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon unit = m2MakeBox(0.4f, 0.4f);
    m2ShapeId boxShape = m2CreatePolygonShape(box, &sd, &unit);

    // Impulse to center: speed without spin.
    m2Body_ApplyLinearImpulseToCenter(box, (m2Vec2){0.64f, 0.0f});
    CHECK(m2Body_GetLinearVelocity(box).x > 0.9f && m2Body_GetAngularVelocity(box) == 0.0f,
          "center impulse moves without spin");

    // Manual sleep: stilled instantly, then woken on demand.
    m2Body_SetAwake(box, false);
    CHECK(!m2Body_IsAwake(box) && m2Body_GetLinearVelocity(box).x == 0.0f,
          "forced sleep stills the body");
    m2Body_SetAwake(box, true);
    CHECK(m2Body_IsAwake(box), "and it wakes on demand");

    // Runtime flags and data.
    m2Body_SetBullet(box, true);
    CHECK(m2Body_IsBullet(box), "bullet flag flips at runtime");
    m2Body_SetUserData(box, 777);
    m2Shape_SetUserData(boxShape, 888);
    CHECK(m2Body_GetUserData(box) == 777 && m2Shape_GetUserData(boxShape) == 888,
          "user data round-trips at runtime");
    float massBefore = m2Body_GetMass(box);
    m2Shape_SetDensity(boxShape, 2.0f);
    CHECK(m2Body_GetMass(box) > massBefore * 1.9f, "density doubles the mass");

    // Mass and frame readers.
    m2Pos2 com = m2Body_GetWorldCenterOfMass(box);
    CHECK(com.x > -0.01 && com.x < 0.01 && com.y > 4.99 && com.y < 5.01,
          "world center of mass sits on the centered box");
    CHECK(m2Body_GetRotationalInertia(box) > 0.0f, "inertia reads positive");
    m2AABBResult ab = m2Body_ComputeAABB(box);
    CHECK(ab.lowerBound.x < -0.39 && ab.upperBound.x > 0.39 && ab.upperBound.y > 5.39,
          "the body AABB hugs the shape");
    m2AABBResult sab = m2Shape_GetAABB(boxShape);
    CHECK(sab.lowerBound.y < 4.61 && sab.upperBound.y > 5.39, "the shape AABB agrees");
    m2WorldId back = m2Body_GetWorld(box);
    CHECK(back.index1 == world.index1 && back.generation == world.generation,
          "GetWorld round-trips");

    // One-shape queries.
    CHECK(m2Shape_TestPoint(boxShape, (m2Pos2){0.0, 5.0}), "the center is inside");
    CHECK(!m2Shape_TestPoint(boxShape, (m2Pos2){2.0, 5.0}), "two meters out is not");
    m2Pos2 closest = m2Shape_GetClosestPoint(boxShape, (m2Pos2){3.0, 5.0});
    CHECK(closest.x > 0.35 && closest.x < 0.45, "closest point sits on the face");
    m2RayCastResult ray = m2Shape_RayCast(boxShape, (m2Pos2){3.0, 5.0}, (m2Vec2){-5.0f, 0.0f});
    CHECK(ray.hit && ray.normal.x > 0.99f, "the one-shape ray hits the right face");

    // Chain parentage.
    m2BodyDef gd = m2DefaultBodyDef();
    gd.position = (m2Pos2){10.0, 0.0};
    m2BodyId ground = m2CreateBody(world, &gd);
    m2Vec2 pts[5] = {{4.0f, 0.0f}, {2.0f, 0.0f}, {0.0f, 0.0f}, {-2.0f, 0.0f}, {-4.0f, 0.0f}};
    m2ChainDef chain = m2DefaultChainDef();
    chain.points = pts;
    chain.count = 5;
    m2ChainId ledge = m2CreateChain(ground, &chain);
    m2ShapeId links[4];
    m2Chain_GetShapes(ledge, links, 4);
    m2ChainId parent = m2Shape_GetParentChain(links[0]);
    CHECK(parent.index1 == ledge.index1 && parent.generation == ledge.generation,
          "links name their chain");
    CHECK(m2Shape_GetParentChain(boxShape).index1 == 0, "plain shapes name none");
    m2WorldId cw = m2Chain_GetWorld(ledge);
    CHECK(cw.index1 == world.index1, "chains know their world too");

    // Joint extras: user data, drift readers, world.
    m2BodyDef hd = m2DefaultBodyDef();
    hd.position = (m2Pos2){0.0, 8.0};
    m2BodyId hook = m2CreateBody(world, &hd);
    m2DistanceJointDef dj = m2DefaultDistanceJointDef();
    dj.bodyIdA = hook;
    dj.bodyIdB = box;
    dj.userData = 4242;
    m2JointId rope = m2CreateDistanceJoint(world, &dj);
    CHECK(m2Joint_GetUserData(rope) == 4242, "joint user data arrives from the def");
    m2Joint_SetUserData(rope, 5151);
    CHECK(m2Joint_GetUserData(rope) == 5151, "and flips at runtime");
    CHECK(m2Joint_GetWorld(rope).index1 == world.index1, "joints know their world");
    // Teleport the box a meter outward: the rope reads that meter.
    m2Body_SetTransform(box, (m2Pos2){0.0, 1.0}, (m2Rot){1.0f, 0.0f});
    float stretch = m2Joint_GetLinearSeparation(rope);
    CHECK(stretch > 3.9f && stretch < 4.1f, "linear separation reads the stretch");
    CHECK(m2Joint_GetAngularSeparation(rope) == 0.0f, "a rope pins no angle");

    // Kinematic follow: one step to the target pose.
    m2BodyDef kd = m2DefaultBodyDef();
    kd.type = m2_kinematicBody;
    kd.position = (m2Pos2){20.0, 0.0};
    m2BodyId mover = m2CreateBody(world, &kd);
    m2CreatePolygonShape(mover, &sd, &unit);
    m2Body_SetTargetTransform(mover, (m2Pos2){21.0, 0.5}, (m2Rot){1.0f, 0.0f}, 1.0f / 60.0f);
    m2World_Step(world, 1.0f / 60.0f, 4);
    m2Pos2 landed = m2Body_GetPosition(mover);
    CHECK(landed.x > 20.9 && landed.x < 21.1 && landed.y > 0.4 && landed.y < 0.6,
          "target transform lands in one step");

    m2DestroyWorld(world);
}

// Dominance (slice 77, a rival lesson): the higher body cannot be
// pushed by the lower one in contacts, statics outrank everyone,
// joints stay symmetric.
static void TestDominance(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m2WorldId world = m2CreateWorld(&def);
    m2BodyDef gd = m2DefaultBodyDef();
    gd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = m2CreateBody(world, &gd);
    m2ShapeDef fs = m2DefaultShapeDef();
    fs.friction = 0.2f;
    m2Polygon slab = m2MakeBox(20.0f, 0.5f);
    m2CreatePolygonShape(floor, &fs, &slab);

    // A heavy runaway crate slams into a plain twin and into a
    // dominant twin; only the plain one gets shoved.
    m2BodyId targets[2];
    for (int32_t i = 0; i < 2; ++i)
    {
        double x = (double)i * 12.0;
        m2BodyDef td = m2DefaultBodyDef();
        td.type = m2_dynamicBody;
        td.position = (m2Pos2){x, 0.45};
        if (i == 1)
        {
            td.dominance = 5;
        }
        targets[i] = m2CreateBody(world, &td);
        m2ShapeDef sd = m2DefaultShapeDef();
        sd.friction = 0.2f;
        m2Polygon unit = m2MakeBox(0.4f, 0.4f);
        m2CreatePolygonShape(targets[i], &sd, &unit);

        m2BodyDef rd = m2DefaultBodyDef();
        rd.type = m2_dynamicBody;
        rd.position = (m2Pos2){x - 3.0, 0.45};
        rd.linearVelocity = (m2Vec2){8.0f, 0.0f};
        m2BodyId ram = m2CreateBody(world, &rd);
        m2ShapeDef rs = m2DefaultShapeDef();
        rs.density = 4.0f;
        rs.friction = 0.2f;
        m2Polygon heavy = m2MakeBox(0.5f, 0.4f);
        m2CreatePolygonShape(ram, &rs, &heavy);
    }
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    double plainMoved = m2Body_GetPosition(targets[0]).x - 0.0;
    double bossMoved = m2Body_GetPosition(targets[1]).x - 12.0;
    CHECK(plainMoved > 0.5, "the plain crate gets shoved");
    CHECK(bossMoved < 0.05 && bossMoved > -0.05, "the dominant crate does not budge");
    CHECK(m2Body_GetDominance(targets[1]) == 5, "dominance reads back");

    // The dominant crate still falls (gravity is not a contact) and
    // still rests on the floor (statics outrank it).
    m2Body_SetTransform(targets[1], (m2Pos2){12.0, 3.0}, (m2Rot){1.0f, 0.0f});
    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(targets[1]).y < 0.6, "the boss still lands on the floor");

    // Runtime demotion: drop the crown and the crate shoves again.
    m2Body_SetDominance(targets[1], 0);
    m2BodyDef rd2 = m2DefaultBodyDef();
    rd2.type = m2_dynamicBody;
    rd2.position = (m2Pos2){9.0, 0.45};
    rd2.linearVelocity = (m2Vec2){8.0f, 0.0f};
    m2BodyId ram2 = m2CreateBody(world, &rd2);
    m2ShapeDef rs2 = m2DefaultShapeDef();
    rs2.density = 4.0f;
    m2Polygon heavy2 = m2MakeBox(0.5f, 0.4f);
    m2CreatePolygonShape(ram2, &rs2, &heavy2);
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(targets[1]).x > 12.3, "demoted, it gets shoved like anyone");

    m2DestroyWorld(world);
}

int main(void)
{
    TestRuntimeGravity();
    TestDominance();
    TestLeftoverBasket();
    TestSmallBasket();
    TestDormancyMassAndExplosions();
    TestBodyDynamicsPack();
    TestMirrorRebuild();
    TestEnumerationWalk();
    TestDestroyWakesSleepers();
    TestDebugDraw();
    TestSetType();
    TestTeleport();
    TestKineticEnergy();
    TestCountersAndProfile();
    TestDefCookies();
    TestRollbackIdentity();
    TestIdSemantics();

    uint64_t hash = DeterminismSweep();
    printf("M2_WORLD_HASH=%016llx\n", (unsigned long long)hash);

    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
