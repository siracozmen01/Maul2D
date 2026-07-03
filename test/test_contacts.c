// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Contact gate, slice 3a: analytic manifold checks (circle kernels,
// Voronoi regions, feature ids, the flip path), speculative existence
// before touch, feature-id persistence with warm-start impulse carry,
// rollback over the contact block, and the manifold evolution hash.

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

#define CHECK_NEAR(value, expected, tolerance, msg)                                                \
    CHECK((value) > (expected) - (tolerance) && (value) < (expected) + (tolerance), msg)

static m2RelativePose IdentityPose(void)
{
    m2RelativePose pose = {{0.0f, 0.0f}, {1.0f, 0.0f}};
    return pose;
}

static void TestCircleKernels(void)
{
    // Overlapping circles on the x axis: separation = dist - r1 - r2.
    m2Circle a = {{0.0f, 0.0f}, 0.5f};
    m2Circle b = {{0.8f, 0.0f}, 0.5f};
    m2Manifold m = m2CollideCircles(&a, &b, IdentityPose());
    CHECK(m.pointCount == 1, "overlapping circles produce one point");
    CHECK_NEAR(m.points[0].separation, -0.2f, 1.0e-5f, "circle separation");
    CHECK_NEAR(m.normal.x, 1.0f, 1.0e-5f, "circle normal points A to B");
    CHECK_NEAR(m.points[0].anchorA.x, 0.4f, 1.0e-5f, "midpoint anchor");

    // Speculative: apart but within the margin - manifold exists with
    // positive separation (topic-07 D1).
    m2Circle far = {{1.01f, 0.0f}, 0.5f};
    m = m2CollideCircles(&a, &far, IdentityPose());
    CHECK(m.pointCount == 1 && m.points[0].separation > 0.0f, "speculative point before touch");

    // Beyond the margin: no manifold.
    m2Circle beyond = {{1.1f, 0.0f}, 0.5f};
    CHECK(m2CollideCircles(&a, &beyond, IdentityPose()).pointCount == 0, "beyond margin: none");

    // Coincident centers: canonical (0, 1) fallback, never NaN.
    m2Circle same = {{0.0f, 0.0f}, 0.5f};
    m = m2CollideCircles(&a, &same, IdentityPose());
    CHECK(m.normal.x == 0.0f && m.normal.y == 1.0f, "coincident fallback normal");
    CHECK(m.points[0].separation == m.points[0].separation, "no NaN on coincidence");
}

static void TestPolygonCircleRegions(void)
{
    m2Polygon box = m2MakeBox(1.0f, 1.0f);

    // Face region: circle above the top edge (edge index 2 for MakeBox
    // winding). Feature id encodes the edge, low byte 0.
    m2Circle above = {{0.0f, 1.4f}, 0.5f};
    m2Manifold m = m2CollidePolygonAndCircle(&box, &above, IdentityPose());
    CHECK(m.pointCount == 1, "face region hit");
    CHECK_NEAR(m.points[0].separation, -0.1f, 1.0e-5f, "face separation");
    CHECK_NEAR(m.normal.y, 1.0f, 1.0e-5f, "face normal");
    CHECK((m.points[0].id & 0xFF) == 0, "face region id");

    // Vertex region: circle overlapping the top-right corner along the
    // diagonal (corner distance 0.283 < radius 0.3).
    m2Circle corner = {{1.2f, 1.2f}, 0.3f};
    m = m2CollidePolygonAndCircle(&box, &corner, IdentityPose());
    CHECK(m.pointCount == 1, "vertex region hit");
    CHECK((m.points[0].id & 0xFF) != 0, "vertex region id marks a vertex");
    CHECK_NEAR(m.normal.x, 0.7071f, 1.0e-3f, "diagonal normal x");
    CHECK_NEAR(m.normal.y, 0.7071f, 1.0e-3f, "diagonal normal y");
    CHECK(m.points[0].separation < 0.0f, "corner overlap penetrates");

    // And just beyond the margin at the corner: correctly no manifold.
    m2Circle outside = {{1.4f, 1.4f}, 0.3f};
    CHECK(m2CollidePolygonAndCircle(&box, &outside, IdentityPose()).pointCount == 0,
          "beyond corner margin: none");

    // Deep containment must still produce a face answer, never NaN.
    m2Circle inside = {{0.1f, 0.0f}, 0.2f};
    m = m2CollidePolygonAndCircle(&box, &inside, IdentityPose());
    CHECK(m.pointCount == 1 && m.points[0].separation < -0.5f, "contained circle manifold");
}

static void TestPersistenceAndCarry(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.gravity = (m2Vec2){0.0f, 0.0f};
    m2WorldId worldId = m2CreateWorld(&def);
    m2World* world = m2World_GetInternal(worldId);

    // A dynamic circle resting overlapped on a static polygon: the pair
    // is stable, so the manifold point must persist by feature id.
    m2BodyDef floorDef = m2DefaultBodyDef();
    m2BodyId floorBody = m2CreateBody(worldId, &floorDef);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon box = m2MakeBox(2.0f, 0.5f);
    m2CreatePolygonShape(floorBody, &sd, &box);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 0.8};
    m2BodyId ball = m2CreateBody(worldId, &bd);
    m2Circle circle = {{0.0f, 0.0f}, 0.4f};
    m2CreateCircleShape(ball, &sd, &circle);

    m2World_Step(worldId, 1.0f / 60.0f, 4);
    CHECK(world->pairCount == 1 && world->manifolds[0].pointCount == 1, "resting contact");
    uint16_t firstId = world->manifolds[0].points[0].id;

    // Inject a warm-start impulse (white box) and step: the fresh
    // manifold must inherit it through the id match.
    world->manifolds[0].points[0].normalImpulse = 3.5f;
    m2World_Step(worldId, 1.0f / 60.0f, 4);
    CHECK(world->manifolds[0].points[0].id == firstId, "feature id stable across steps");
    CHECK(world->manifolds[0].points[0].normalImpulse == 3.5f, "impulse carried by id match");
    CHECK((world->manifolds[0].points[0].flags & 1) != 0, "persisted flag set");

    m2DestroyWorld(worldId);
}

static void TestContactRollback(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 32;
    def.shapeCapacity = 32;
    m2WorldId worldId = m2CreateWorld(&def);

    m2BodyDef floorDef = m2DefaultBodyDef();
    floorDef.position = (m2Pos2){0.0, -0.5};
    m2BodyId floorBody = m2CreateBody(worldId, &floorDef);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(10.0f, 0.5f);
    m2CreatePolygonShape(floorBody, &sd, &slab);

    for (int32_t i = 0; i < 8; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){-3.5 + 1.0 * (double)i, 1.0 + 0.5 * (double)i};
        m2BodyId body = m2CreateBody(worldId, &bd);
        m2Circle circle = {{0.0f, 0.0f}, 0.45f};
        m2CreateCircleShape(body, &sd, &circle);
    }

    for (int32_t i = 0; i < 25; ++i)
    {
        m2World_Step(worldId, 1.0f / 60.0f, 4);
    }

    int32_t size = m2World_SnapshotSize(worldId);
    void* snapA = malloc((size_t)size);
    void* snapB = malloc((size_t)size);
    CHECK(m2World_Snapshot(worldId, snapA, size) == size, "snapshot");
    uint64_t hashes[30];
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(worldId, 1.0f / 60.0f, 4);
        hashes[i] = m2World_Hash(worldId);
    }
    CHECK(m2World_Restore(worldId, snapA, size), "restore");
    CHECK(m2World_Snapshot(worldId, snapB, size) == size, "re-snapshot");
    CHECK(memcmp(snapA, snapB, (size_t)size) == 0, "contact block survives byte-compare");
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(worldId, 1.0f / 60.0f, 4);
        CHECK(m2World_Hash(worldId) == hashes[i], "manifold evolution replays bit-exactly");
    }

    free(snapA);
    free(snapB);
    m2DestroyWorld(worldId);
}

static uint64_t ContactSweepHash(void)
{
    // Circle rain onto a wide polygon slab far from the origin: manifold
    // birth, persistence, and speculative points all churn; the rolling
    // world hash (which includes the contact block) crosses CI cells.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 128;
    def.shapeCapacity = 128;
    m2WorldId worldId = m2CreateWorld(&def);

    m2BodyDef floorDef = m2DefaultBodyDef();
    floorDef.position = (m2Pos2){2.0e5, 0.0};
    m2BodyId floorBody = m2CreateBody(worldId, &floorDef);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(20.0f, 0.5f);
    m2CreatePolygonShape(floorBody, &sd, &slab);

    uint64_t h = M2_HASH_INIT;
    for (int32_t step = 0; step < 200; ++step)
    {
        if (step % 5 == 0 && step < 150)
        {
            m2BodyDef bd = m2DefaultBodyDef();
            bd.type = m2_dynamicBody;
            bd.position = (m2Pos2){2.0e5 - 15.0 + 1.03 * (double)(step / 5), 4.0};
            bd.linearVelocity = (m2Vec2){0.5f * (float)(step % 3) - 0.5f, -2.0f};
            m2BodyId body = m2CreateBody(worldId, &bd);
            m2Circle circle = {{0.0f, 0.0f}, 0.3f + 0.04f * (float)(step % 4)};
            m2CreateCircleShape(body, &sd, &circle);
        }
        m2World_Step(worldId, 1.0f / 60.0f, 4);
        uint64_t worldHash = m2World_Hash(worldId);
        h = m2Hash64(h, &worldHash, (int32_t)sizeof(worldHash));
    }
    m2DestroyWorld(worldId);
    return h;
}

int main(void)
{
    TestCircleKernels();
    TestPolygonCircleRegions();
    TestPersistenceAndCarry();
    TestContactRollback();

    uint64_t hash = ContactSweepHash();
    printf("M2_CONTACT_HASH=%016llx\n", (unsigned long long)hash);

    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
