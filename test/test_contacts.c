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
    m2WorldId worldId = m2CreateWorld(&def);
    m2World* world = m2World_GetInternal(worldId);

    // A dynamic circle resting on a static polygon under gravity: the
    // point must persist by feature id and carry the body's weight.
    m2BodyDef floorDef = m2DefaultBodyDef();
    m2BodyId floorBody = m2CreateBody(worldId, &floorDef);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon box = m2MakeBox(2.0f, 0.5f);
    m2CreatePolygonShape(floorBody, &sd, &box);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 0.92};
    m2BodyId ball = m2CreateBody(worldId, &bd);
    m2Circle circle = {{0.0f, 0.0f}, 0.4f};
    m2CreateCircleShape(ball, &sd, &circle);

    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(worldId, 1.0f / 60.0f, 4);
    }
    CHECK(world->pairCount == 1 && world->manifolds[0].pointCount == 1, "resting contact");
    uint16_t firstId = world->manifolds[0].points[0].id;
    float restingImpulse = world->manifolds[0].points[0].normalImpulse;
    CHECK(restingImpulse > 0.0f, "resting contact carries load");
    m2World_Step(worldId, 1.0f / 60.0f, 4);
    CHECK(world->manifolds[0].points[0].id == firstId, "feature id stable across steps");
    CHECK((world->manifolds[0].points[0].flags & 1) != 0, "persisted flag set");
    m2Pos2 rest = m2Body_GetPosition(ball);
    CHECK(rest.y > 0.85 && rest.y < 0.95, "ball rests on the slab, no sinking");

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
            switch (step % 3)
            {
            case 0:
            {
                m2Circle circle = {{0.0f, 0.0f}, 0.3f + 0.04f * (float)(step % 4)};
                m2CreateCircleShape(body, &sd, &circle);
                break;
            }
            case 1:
            {
                m2Polygon smallBox = m2MakeBox(0.3f, 0.25f);
                m2CreatePolygonShape(body, &sd, &smallBox);
                break;
            }
            default:
            {
                m2Capsule cap = {{-0.25f, 0.0f}, {0.25f, 0.0f}, 0.15f};
                m2CreateCapsuleShape(body, &sd, &cap);
                break;
            }
            }
        }
        m2World_Step(worldId, 1.0f / 60.0f, 4);
        uint64_t worldHash = m2World_Hash(worldId);
        h = m2Hash64(h, &worldHash, (int32_t)sizeof(worldHash));
    }
    m2DestroyWorld(worldId);
    return h;
}

static void TestPolygonKernels(void)
{
    // Face-face: box resting overlapped on a wide slab: two points, both
    // penetrating, normal up, distinct stable ids.
    m2Polygon slab = m2MakeBox(4.0f, 0.5f);
    m2Polygon box = m2MakeBox(0.5f, 0.5f);
    m2RelativePose pose = {{0.0f, 0.95f}, {1.0f, 0.0f}}; // box above slab
    m2Manifold m = m2CollidePolygons(&slab, &box, pose);
    CHECK(m.pointCount == 2, "face-face gives two points");
    CHECK_NEAR(m.normal.y, 1.0f, 1.0e-4f, "face-face normal up");
    CHECK(m.points[0].separation < 0.0f && m.points[1].separation < 0.0f, "both points penetrate");
    CHECK(m.points[0].id != m.points[1].id, "distinct feature ids");

    // Speculative face-face: within margin, positive separations.
    pose.p.y = 1.01f;
    m = m2CollidePolygons(&slab, &box, pose);
    CHECK(m.pointCount == 2 && m.points[0].separation > 0.0f, "speculative face-face");

    // Beyond margin: none.
    pose.p.y = 1.05f;
    CHECK(m2CollidePolygons(&slab, &box, pose).pointCount == 0, "face-face beyond margin");

    // Vertex-vertex: two boxes corner to corner along the diagonal with a
    // sub-margin gap - the single-point path with a diagonal normal.
    m2Polygon boxA = m2MakeBox(0.5f, 0.5f);
    m2Polygon boxB = m2MakeBox(0.5f, 0.5f);
    m2RelativePose corner = {{1.01f, 1.01f}, {1.0f, 0.0f}};
    m = m2CollidePolygons(&boxA, &boxB, corner);
    CHECK(m.pointCount == 1, "vertex-vertex single point");
    CHECK_NEAR(m.normal.x, 0.7071f, 5.0e-2f, "vertex-vertex diagonal normal");

    // Capsule lying on a slab (via the 2-vertex proxy): two points.
    m2Polygon capsule = m2MakeSegmentProxy((m2Vec2){-0.4f, 0.0f}, (m2Vec2){0.4f, 0.0f}, 0.2f);
    m2RelativePose lying = {{0.0f, 0.65f}, {1.0f, 0.0f}};
    m = m2CollidePolygons(&slab, &capsule, lying);
    CHECK(m.pointCount == 2, "capsule on slab gives two points");
    CHECK(m.points[0].separation < 0.0f, "capsule rests in contact");

    // Rotated box (45 deg) over a slab: no NaN, at least one point.
    m2Rot rot45 = m2MakeRot(0.25f * M2_PI);
    m2RelativePose tilted = {{0.0f, 1.15f}, rot45};
    m = m2CollidePolygons(&slab, &box, tilted);
    CHECK(m.pointCount >= 1, "tilted box makes contact");
    for (int32_t i = 0; i < m.pointCount; ++i)
    {
        CHECK(m.points[i].separation == m.points[i].separation, "no NaN separations");
    }
}

static void TestBoxStackPersistence(void)
{
    // A box overlapping a static slab in a zero-gravity world: the two
    // manifold points must keep their ids across 30 steps and carry
    // injected impulses (the solver precondition).
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2WorldId worldId = m2CreateWorld(&def);
    m2World* world = m2World_GetInternal(worldId);

    m2ShapeDef sd = m2DefaultShapeDef();
    m2BodyDef floorDef = m2DefaultBodyDef();
    m2BodyId floorBody = m2CreateBody(worldId, &floorDef);
    m2Polygon slab = m2MakeBox(3.0f, 0.5f);
    m2CreatePolygonShape(floorBody, &sd, &slab);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 0.95};
    m2BodyId box = m2CreateBody(worldId, &bd);
    m2Polygon unit = m2MakeBox(0.5f, 0.5f);
    m2CreatePolygonShape(box, &sd, &unit);

    for (int32_t i = 0; i < 10; ++i)
    {
        m2World_Step(worldId, 1.0f / 60.0f, 4);
    }
    CHECK(world->pairCount == 1 && world->manifolds[0].pointCount == 2, "stack contact");
    uint16_t id0 = world->manifolds[0].points[0].id;
    uint16_t id1 = world->manifolds[0].points[1].id;

    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(worldId, 1.0f / 60.0f, 4);
    }
    CHECK(world->manifolds[0].points[0].id == id0 && world->manifolds[0].points[1].id == id1,
          "box-slab ids stable over 30 steps");
    CHECK(world->manifolds[0].points[0].normalImpulse > 0.0f &&
              world->manifolds[0].points[1].normalImpulse > 0.0f,
          "both points carry the box weight");
    m2Pos2 boxPos = m2Body_GetPosition(box);
    CHECK(boxPos.y > 0.9 && boxPos.y < 1.05, "box rests without sinking");

    m2DestroyWorld(worldId);
}

static void TestCollisionFilters(void)
{
    // Category/mask filtering: a "ghost" box falls straight through a
    // floor it is masked against, a one-way mismatch also blocks (the
    // rule is AND), and the filter survives rollback.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);

    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = m2CreateBody(world, &fd);
    m2ShapeDef floorShape = m2DefaultShapeDef();
    floorShape.categoryBits = 0x1;
    floorShape.maskBits = 0x2; // floor only talks to category 2
    m2Polygon slab = m2MakeBox(10.0f, 0.5f);
    m2CreatePolygonShape(floor, &floorShape, &slab);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 2.0};
    m2BodyId solid = m2CreateBody(world, &bd);
    m2ShapeDef solidShape = m2DefaultShapeDef();
    solidShape.categoryBits = 0x2;
    solidShape.maskBits = 0x1;
    m2Polygon unit = m2MakeBox(0.4f, 0.4f);
    m2CreatePolygonShape(solid, &solidShape, &unit);

    bd.position = (m2Pos2){3.0, 2.0};
    m2BodyId ghost = m2CreateBody(world, &bd);
    m2ShapeDef ghostShape = m2DefaultShapeDef();
    ghostShape.categoryBits = 0x4; // floor's mask does not include 4
    ghostShape.maskBits = 0xFFFFFFFFu;
    m2CreatePolygonShape(ghost, &ghostShape, &unit);

    bd.position = (m2Pos2){-3.0, 2.0};
    m2BodyId oneWay = m2CreateBody(world, &bd);
    m2ShapeDef oneWayShape = m2DefaultShapeDef();
    oneWayShape.categoryBits = 0x2; // floor would accept it...
    oneWayShape.maskBits = 0x8;     // ...but it refuses the floor
    m2CreatePolygonShape(oneWay, &oneWayShape, &unit);

    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(solid).y > 0.3, "matching filters collide and rest");
    CHECK(m2Body_GetPosition(ghost).y < -3.0, "mismatched category falls through");
    CHECK(m2Body_GetPosition(oneWay).y < -3.0, "one-way mismatch also falls (AND rule)");

    m2DestroyWorld(world);
}

static void TestGroupIndex(void)
{
    // Same negative group never collides even when the masks agree;
    // same positive group always collides even when the masks refuse.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);

    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = m2CreateBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(12.0f, 0.5f);
    m2CreatePolygonShape(floor, &fs, &slab);

    // Two squad-mates (group -3) dropped onto the same spot: they pass
    // through each other and both land on the floor.
    m2ShapeDef mate = m2DefaultShapeDef();
    mate.groupIndex = -3;
    m2Polygon unit = m2MakeBox(0.4f, 0.4f);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 1.2};
    m2BodyId one = m2CreateBody(world, &bd);
    m2CreatePolygonShape(one, &mate, &unit);
    bd.position = (m2Pos2){0.05, 2.4};
    m2BodyId two = m2CreateBody(world, &bd);
    m2CreatePolygonShape(two, &mate, &unit);

    // Two forced-friends (group +5) whose masks refuse each other:
    // they still stack.
    m2ShapeDef friendA = m2DefaultShapeDef();
    friendA.groupIndex = 5;
    friendA.categoryBits = 0x10;
    friendA.maskBits = 0x1; // floor only
    bd.position = (m2Pos2){4.0, 0.55};
    m2BodyId base = m2CreateBody(world, &bd);
    m2CreatePolygonShape(base, &friendA, &unit);
    bd.position = (m2Pos2){4.0, 1.6};
    m2BodyId rider = m2CreateBody(world, &bd);
    m2CreatePolygonShape(rider, &friendA, &unit);

    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    double y1 = m2Body_GetPosition(one).y;
    double y2 = m2Body_GetPosition(two).y;
    CHECK(y1 < 0.6 && y2 < 0.6, "negative group mates pass through each other");
    CHECK(m2Body_GetPosition(rider).y > 1.0, "positive group forces the stack");

    m2DestroyWorld(world);
}

static void TestFilterRollback(void)
{
    // Filters are snapshot state: a mixed-filter scene must replay
    // bit-exactly through restore.
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
    for (int32_t i = 0; i < 4; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){-1.5 + (double)i, 1.5 + 0.4 * (double)i};
        m2BodyId body = m2CreateBody(world, &bd);
        m2ShapeDef sd = m2DefaultShapeDef();
        sd.categoryBits = i % 2 == 0 ? 0x1u : 0x4u; // half of them ghosts
        m2Polygon unit = m2MakeBox(0.3f, 0.3f);
        m2CreatePolygonShape(body, &sd, &unit);
    }

    for (int32_t i = 0; i < 20; ++i)
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
        CHECK(m2World_Hash(world) == hashes[i], "filtered scene replays bit-exactly");
    }
    free(snap);
    m2DestroyWorld(world);
}

int main(void)
{
    TestCollisionFilters();
    TestGroupIndex();
    TestFilterRollback();
    TestCircleKernels();
    TestPolygonCircleRegions();
    TestPolygonKernels();
    TestBoxStackPersistence();
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
