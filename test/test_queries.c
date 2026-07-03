// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Query gate: the closest ray cast picks the right shape with an
// analytically exact fraction, works on every geometry, and stays
// exact far from the origin; AABB overlap returns exactly the
// expected shapes in canonical order with a truthful total; queries
// never move the world hash; and a query storm's results hash
// identically across CI cells.

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

static m2ShapeId AddStaticBox(m2WorldId world, double x, double y, float hw, float hh)
{
    m2BodyDef bd = m2DefaultBodyDef();
    bd.position = (m2Pos2){x, y};
    m2BodyId body = m2CreateBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon box = m2MakeBox(hw, hh);
    return m2CreatePolygonShape(body, &sd, &box);
}

static m2ShapeId AddStaticCircle(m2WorldId world, double x, double y, float radius)
{
    m2BodyDef bd = m2DefaultBodyDef();
    bd.position = (m2Pos2){x, y};
    m2BodyId body = m2CreateBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Circle c = {{0.0f, 0.0f}, radius};
    return m2CreateCircleShape(body, &sd, &c);
}

static bool SameShape(m2ShapeId a, m2ShapeId b)
{
    return a.index1 == b.index1 && a.generation == b.generation;
}

static void TestRayClosest(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m2WorldId world = m2CreateWorld(&def);

    // Three walls along +x; the ray must pick the nearest.
    AddStaticBox(world, 8.0, 0.0, 0.5, 2.0);
    m2ShapeId nearWall = AddStaticBox(world, 4.0, 0.0, 0.5, 2.0);
    AddStaticBox(world, 12.0, 0.0, 0.5, 2.0);

    m2RayCastResult hit = m2World_CastRayClosest(world, (m2Pos2){0.0, 0.0}, (m2Vec2){20.0f, 0.0f});
    CHECK(hit.hit, "ray hits");
    CHECK(SameShape(hit.shapeId, nearWall), "closest wall wins");
    // Wall face at x = 3.5, ray length 20: fraction = 0.175 exactly.
    CHECK(hit.fraction == 0.175f, "box fraction is analytically exact");
    CHECK(hit.normal.x == -1.0f && hit.normal.y == 0.0f, "face normal points back");
    double px = hit.point.x - 3.5;
    CHECK(px > -1.0e-6 && px < 1.0e-6 && hit.point.y == 0.0, "hit point lands on the face");

    // A miss stays a miss.
    m2RayCastResult miss =
        m2World_CastRayClosest(world, (m2Pos2){0.0, 10.0}, (m2Vec2){20.0f, 0.0f});
    CHECK(!miss.hit, "ray over the walls misses");

    // Circle fraction: center (4,-6) r=1, vertical ray from (4,-3)
    // (below the wall) down 10: hits at y=-5, fraction 0.2.
    AddStaticCircle(world, 4.0, -6.0, 1.0f);
    m2RayCastResult round =
        m2World_CastRayClosest(world, (m2Pos2){4.0, -3.0}, (m2Vec2){0.0f, -10.0f});
    CHECK(round.hit, "circle hit");
    CHECK(round.fraction == 0.2f, "circle fraction is analytically exact");
    CHECK(round.normal.y == 1.0f, "circle normal points up");

    m2DestroyWorld(world);
}

static void TestRayGeometries(void)
{
    // Capsule and segment kernels answer too.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.position = (m2Pos2){0.0, 0.0};
    m2BodyId body = m2CreateBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Capsule capsule = {{-1.0f, 0.0f}, {1.0f, 0.0f}, 0.5f};
    m2CreateCapsuleShape(body, &sd, &capsule);

    m2RayCastResult hit = m2World_CastRayClosest(world, (m2Pos2){0.0, 5.0}, (m2Vec2){0.0f, -10.0f});
    CHECK(hit.hit, "capsule hit");
    CHECK(hit.fraction == 0.45f, "capsule top at y=0.5: fraction exact");

    m2BodyDef gd = m2DefaultBodyDef();
    gd.position = (m2Pos2){10.0, 0.0};
    m2BodyId ground = m2CreateBody(world, &gd);
    m2Segment segment = {{-2.0f, 0.0f}, {2.0f, 0.0f}};
    m2CreateSegmentShape(ground, &sd, &segment);

    m2RayCastResult seg = m2World_CastRayClosest(world, (m2Pos2){10.0, 3.0}, (m2Vec2){0.0f, -6.0f});
    CHECK(seg.hit, "segment hit");
    CHECK(seg.fraction == 0.5f, "segment fraction exact");

    m2DestroyWorld(world);
}

static void TestRayFarFromOrigin(void)
{
    // The f64 discipline: the same cast, 3e5 meters out, same numbers.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);

    double bx = 3.0e5;
    AddStaticBox(world, bx + 4.0, 0.0, 0.5, 2.0);
    m2RayCastResult hit = m2World_CastRayClosest(world, (m2Pos2){bx, 0.0}, (m2Vec2){20.0f, 0.0f});
    CHECK(hit.hit, "far cast hits");
    CHECK(hit.fraction == 0.175f, "far fraction identical to the origin scene");
    double farPx = hit.point.x - (bx + 3.5);
    CHECK(farPx > -1.0e-5 && farPx < 1.0e-5, "far hit point exact in f64");

    m2DestroyWorld(world);
}

static void TestOverlap(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m2WorldId world = m2CreateWorld(&def);

    m2ShapeId a = AddStaticBox(world, 0.0, 0.0, 0.5, 0.5);
    m2ShapeId b = AddStaticBox(world, 2.0, 0.0, 0.5, 0.5);
    m2ShapeId c = AddStaticCircle(world, 4.0, 0.0, 0.5f);
    AddStaticBox(world, 40.0, 0.0, 0.5, 0.5); // far away

    m2ShapeId results[8];
    int32_t total =
        m2World_OverlapAABB(world, (m2Pos2){-1.0, -1.0}, (m2Pos2){4.2, 1.0}, results, 8);
    CHECK(total == 3, "overlap finds exactly the three");
    CHECK(SameShape(results[0], a) && SameShape(results[1], b) && SameShape(results[2], c),
          "overlap order is canonical (creation order)");

    // Truthful total under a tight capacity; the kept ones are the
    // lowest indices.
    m2ShapeId two[2];
    int32_t clamped = m2World_OverlapAABB(world, (m2Pos2){-1.0, -1.0}, (m2Pos2){4.2, 1.0}, two, 2);
    CHECK(clamped == 3, "total reported even beyond capacity");
    CHECK(SameShape(two[0], a) && SameShape(two[1], b), "capacity keeps the canonical head");

    // Empty region.
    int32_t none =
        m2World_OverlapAABB(world, (m2Pos2){100.0, 100.0}, (m2Pos2){101.0, 101.0}, results, 8);
    CHECK(none == 0, "empty region reports zero");

    m2DestroyWorld(world);
}

static void TestQueriesAreReadOnly(void)
{
    // A query storm between steps must not move the simulation hash.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 32;
    def.shapeCapacity = 32;
    m2WorldId world = m2CreateWorld(&def);

    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = m2CreateBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(10.0f, 0.5f);
    m2CreatePolygonShape(floor, &fs, &slab);
    for (int32_t i = 0; i < 8; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){-3.0 + 0.9 * (double)i, 1.0 + 0.5 * (double)(i % 3)};
        m2BodyId body = m2CreateBody(world, &bd);
        m2ShapeDef sd = m2DefaultShapeDef();
        m2Circle circle = {{0.0f, 0.0f}, 0.3f};
        m2CreateCircleShape(body, &sd, &circle);
    }
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }

    uint64_t before = m2World_Hash(world);
    m2ShapeId results[32];
    for (int32_t i = 0; i < 50; ++i)
    {
        m2World_CastRayClosest(world, (m2Pos2){-5.0 + 0.2 * (double)i, 3.0}, (m2Vec2){0.3f, -6.0f});
        m2World_OverlapAABB(world, (m2Pos2){-4.0, -1.0}, (m2Pos2){0.1 * (double)i, 2.0}, results,
                            32);
    }
    CHECK(m2World_Hash(world) == before, "queries never touch simulation state");

    uint64_t hashesA[20];
    for (int32_t i = 0; i < 20; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        hashesA[i] = m2World_Hash(world);
    }
    m2DestroyWorld(world);

    // Same scene, no query storm: identical trajectory.
    world = m2CreateWorld(&def);
    floor = m2CreateBody(world, &fd);
    m2CreatePolygonShape(floor, &fs, &slab);
    for (int32_t i = 0; i < 8; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){-3.0 + 0.9 * (double)i, 1.0 + 0.5 * (double)(i % 3)};
        m2BodyId body = m2CreateBody(world, &bd);
        m2ShapeDef sd = m2DefaultShapeDef();
        m2Circle circle = {{0.0f, 0.0f}, 0.3f};
        m2CreateCircleShape(body, &sd, &circle);
    }
    for (int32_t i = 0; i < 50; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    uint64_t tail = m2World_Hash(world);
    CHECK(tail == hashesA[19], "trajectory with and without queries is identical");
    m2DestroyWorld(world);
}

static uint64_t QuerySweepHash(void)
{
    // A debris field far from the origin, swept by a fan of rays and a
    // sliding overlap window; the answers themselves are the gated
    // bytes.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 64;
    def.shapeCapacity = 64;
    m2WorldId world = m2CreateWorld(&def);

    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){-4.4e5, -0.5};
    m2BodyId floor = m2CreateBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(30.0f, 0.5f);
    m2CreatePolygonShape(floor, &fs, &slab);

    for (int32_t i = 0; i < 24; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){-4.4e5 - 10.0 + 0.9 * (double)i, 0.6 + 0.8 * (double)(i % 4)};
        m2BodyId body = m2CreateBody(world, &bd);
        m2ShapeDef sd = m2DefaultShapeDef();
        if (i % 3 == 0)
        {
            m2Circle circle = {{0.0f, 0.0f}, 0.25f + 0.02f * (float)(i % 5)};
            m2CreateCircleShape(body, &sd, &circle);
        }
        else if (i % 3 == 1)
        {
            m2Polygon box = m2MakeBox(0.3f, 0.2f);
            m2CreatePolygonShape(body, &sd, &box);
        }
        else
        {
            m2Capsule capsule = {{-0.2f, 0.0f}, {0.2f, 0.0f}, 0.15f};
            m2CreateCapsuleShape(body, &sd, &capsule);
        }
    }

    uint64_t h = M2_HASH_INIT;
    m2ShapeId results[64];
    for (int32_t step = 0; step < 120; ++step)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        // Ray fan.
        for (int32_t r = 0; r < 8; ++r)
        {
            m2RayCastResult hit =
                m2World_CastRayClosest(world, (m2Pos2){-4.4e5 - 12.0 + 3.0 * (double)r, 4.0},
                                       (m2Vec2){0.5f * (float)(r - 4), -8.0f});
            h = m2Hash64(h, &hit.hit, 1);
            if (hit.hit)
            {
                h = m2Hash64(h, &hit.shapeId.index1, (int32_t)sizeof(int32_t));
                h = m2Hash64(h, &hit.fraction, (int32_t)sizeof(float));
                h = m2Hash64(h, &hit.normal, (int32_t)sizeof(m2Vec2));
            }
        }
        // Sliding overlap window.
        double x = -4.4e5 - 12.0 + 0.2 * (double)step;
        int32_t total =
            m2World_OverlapAABB(world, (m2Pos2){x, -1.0}, (m2Pos2){x + 4.0, 3.0}, results, 64);
        h = m2Hash64(h, &total, (int32_t)sizeof(int32_t));
        for (int32_t i = 0; i < total && i < 64; ++i)
        {
            h = m2Hash64(h, &results[i].index1, (int32_t)sizeof(int32_t));
        }
    }
    m2DestroyWorld(world);
    return h;
}

int main(void)
{
    TestRayClosest();
    TestRayGeometries();
    TestRayFarFromOrigin();
    TestOverlap();
    TestQueriesAreReadOnly();

    uint64_t hash = QuerySweepHash();
    printf("M2_QUERY_HASH=%016llx\n", (unsigned long long)hash);

    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
