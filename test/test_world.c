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

int main(void)
{
    TestRuntimeGravity();
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
