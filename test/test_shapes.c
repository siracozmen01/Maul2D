// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Shape gate: validation thresholds (the RT1-NUM-1 relative-threshold
// family, adversarial inputs included), rotation-aware AABBs against
// analytic expectations, mass properties against closed forms, mixed
// shapes through the pair pipeline, rollback over shape state, and the
// shape sweep hash compared across CI cells.

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

static void TestValidation(void)
{
    // Degenerate capsule: points one ulp apart must be rejected (the axis
    // normalization would divide by ~1e-38).
    m2Capsule tiny = {{1.0f, 0.0f}, {1.0000001f, 0.0f}, 0.3f};
    CHECK(!m2ValidateCapsule(&tiny), "1-ulp capsule must be rejected");
    m2Capsule good = {{-0.5f, 0.0f}, {0.5f, 0.0f}, 0.3f};
    CHECK(m2ValidateCapsule(&good), "ordinary capsule must pass");

    // Sliver polygon: large area but pathological thinness (scale-free
    // rejection; an absolute epsilon would pass this one).
    m2Vec2 sliver[3] = {{0.0f, 0.0f}, {100.0f, 0.0f}, {100.0f, 0.002f}};
    m2Polygon sliverPoly = m2MakePolygon(sliver, 3, 0.0f);
    CHECK(sliverPoly.count == 0, "sliver polygon must be rejected");

    // Winding: clockwise input violates the CCW contract - rejected, not
    // silently repaired (topic-03 D4).
    m2Vec2 clockwise[3] = {{0.0f, 0.0f}, {0.0f, 1.0f}, {1.0f, 0.0f}};
    CHECK(m2MakePolygon(clockwise, 3, 0.0f).count == 0, "CW winding must be rejected");

    // NaN screening at the rim.
    uint32_t nanBits = 0x7FC00000u;
    float qnan;
    memcpy(&qnan, &nanBits, sizeof(qnan));
    m2Circle nanCircle = {{qnan, 0.0f}, 0.5f};
    CHECK(!m2ValidateCircle(&nanCircle), "NaN geometry must be rejected");

    m2Polygon box = m2MakeBox(0.5f, 0.5f);
    CHECK(box.count == 4 && m2ValidatePolygon(&box), "unit box must validate");
}

static void TestAabbAndMass(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 16;
    m2WorldId worldId = m2CreateWorld(&def);
    m2World* world = m2World_GetInternal(worldId);

    // A body rotated 90 degrees with an offset circle: the AABB must
    // follow the rotation (the slice-1 placeholder ignored it).
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){10.0, 20.0};
    bd.rotation = m2MakeRot(0.5f * M2_PI);
    m2BodyId body = m2CreateBody(worldId, &bd);

    m2ShapeDef sd = m2DefaultShapeDef();
    sd.density = 2.0f;
    m2Circle circle = {{3.0f, 0.0f}, 0.5f};
    m2ShapeId shape = m2CreateCircleShape(body, &sd, &circle);
    CHECK(m2Shape_IsValid(shape), "circle attaches");

    // Rotating (3,0) by +90deg lands at (0,3): center ~(10, 23).
    int32_t shapeIndex = shape.index1 - 1;
    m2AABB tight =
        m2ComputeShapeAABB(&world->shapeGeometry[shapeIndex], world->transforms[body.index1 - 1]);
    CHECK_NEAR(tight.lowerBound.x, 10.0 - 0.5, 1.0e-3, "rotated AABB lower x");
    CHECK_NEAR(tight.upperBound.y, 23.0 + 0.5, 1.0e-3, "rotated AABB upper y");

    // Mass: circle m = rho * pi * r^2.
    CHECK_NEAR(m2Body_GetMass(body), 2.0f * 3.14159f * 0.25f, 1.0e-3f, "circle mass");

    // Box inertia about origin for a centered box: m*(w^2+h^2)/12.
    m2BodyDef bd2 = m2DefaultBodyDef();
    bd2.type = m2_dynamicBody;
    m2BodyId boxBody = m2CreateBody(worldId, &bd2);
    m2ShapeDef sd2 = m2DefaultShapeDef();
    sd2.density = 1.0f;
    m2Polygon box = m2MakeBox(0.5f, 0.5f);
    m2CreatePolygonShape(boxBody, &sd2, &box);
    CHECK_NEAR(m2Body_GetMass(boxBody), 1.0f, 1.0e-4f, "unit box mass");
    float expectedInvI = 1.0f / (1.0f * (1.0f + 1.0f) / 12.0f);
    CHECK_NEAR(world->invInertia[boxBody.index1 - 1], expectedInvI, 1.0e-2f, "box inertia");

    // Zero-density dynamic body: the minimum-mass floor, never NaN.
    m2BodyDef bd3 = m2DefaultBodyDef();
    bd3.type = m2_dynamicBody;
    m2BodyId floored = m2CreateBody(worldId, &bd3);
    m2ShapeDef sd3 = m2DefaultShapeDef();
    sd3.density = 0.0f;
    m2CreateCircleShape(floored, &sd3, &circle);
    CHECK(m2Body_GetMass(floored) == 1.0f, "zero-density dynamic body floors to 1 kg");

    m2DestroyWorld(worldId);
}

static void TestCompoundAndRollback(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 32;
    def.shapeCapacity = 64;
    m2WorldId worldId = m2CreateWorld(&def);
    m2World* world = m2World_GetInternal(worldId);

    // Compound body (two shapes) falling onto a segment floor: same-body
    // shapes must never pair; floor pairs arrive as it lands.
    m2BodyDef floorDef = m2DefaultBodyDef();
    floorDef.position = (m2Pos2){0.0, 0.0};
    m2BodyId floorBody = m2CreateBody(worldId, &floorDef);
    m2ShapeDef floorShape = m2DefaultShapeDef();
    m2Segment segment = {{-20.0f, 0.0f}, {20.0f, 0.0f}};
    m2CreateSegmentShape(floorBody, &floorShape, &segment);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 3.0};
    m2BodyId compound = m2CreateBody(worldId, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Capsule capsule = {{-0.4f, 0.0f}, {0.4f, 0.0f}, 0.2f};
    m2CreateCapsuleShape(compound, &sd, &capsule);
    m2Circle topper = {{0.0f, 0.4f}, 0.25f};
    m2CreateCircleShape(compound, &sd, &topper);

    for (int32_t i = 0; i < 20; ++i)
    {
        m2World_Step(worldId, 1.0f / 60.0f, 4);
        for (int32_t p = 0; p < world->pairCount; ++p)
        {
            int32_t a = (int32_t)(world->pairKeys[p] >> 32);
            int32_t b = (int32_t)(world->pairKeys[p] & 0xFFFFFFFFu);
            CHECK(world->shapeBody[a] != world->shapeBody[b], "same-body shapes never pair");
        }
    }

    // Rollback identity over shape + mass + broadphase state.
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
    CHECK(memcmp(snapA, snapB, (size_t)size) == 0, "shape state survives byte-compare");
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(worldId, 1.0f / 60.0f, 4);
        CHECK(m2World_Hash(worldId) == hashes[i], "shape world replays bit-exactly");
    }

    // Destroying the compound body cascades both shapes and their pairs.
    m2DestroyBody(compound);
    CHECK(world->pairCount == 0, "body destroy cascades shape pairs away");

    free(snapA);
    free(snapB);
    m2DestroyWorld(worldId);
}

static uint64_t ShapeSweepHash(void)
{
    // Mixed-shape rain over a segment terrain, rotated spawns, far from
    // the origin: AABB, mass, and pair evolution all feed the hash.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 256;
    def.shapeCapacity = 512;
    m2WorldId worldId = m2CreateWorld(&def);
    m2World* world = m2World_GetInternal(worldId);

    m2BodyDef floorDef = m2DefaultBodyDef();
    floorDef.position = (m2Pos2){-3.0e5, 0.0};
    m2BodyId floorBody = m2CreateBody(worldId, &floorDef);
    m2ShapeDef floorShape = m2DefaultShapeDef();
    m2Segment segment = {{-25.0f, 0.0f}, {25.0f, 0.0f}};
    m2CreateSegmentShape(floorBody, &floorShape, &segment);

    uint64_t h = M2_HASH_INIT;
    for (int32_t step = 0; step < 200; ++step)
    {
        if (step % 3 == 0)
        {
            m2BodyDef bd = m2DefaultBodyDef();
            bd.type = m2_dynamicBody;
            bd.position = (m2Pos2){-3.0e5 - 20.0 + 0.61 * (double)(step % 66), 5.0};
            bd.rotation = m2MakeRot(0.13f * (float)step);
            bd.angularVelocity = 0.5f * (float)(step % 5) - 1.0f;
            m2BodyId body = m2CreateBody(worldId, &bd);
            m2ShapeDef sd = m2DefaultShapeDef();
            sd.density = 1.0f + 0.25f * (float)(step % 4);
            switch (step % 3)
            {
            case 0:
            {
                m2Circle c = {{0.1f, 0.0f}, 0.3f + 0.05f * (float)(step % 4)};
                m2CreateCircleShape(body, &sd, &c);
                break;
            }
            case 1:
            {
                m2Capsule cap = {{-0.3f, 0.0f}, {0.3f, 0.0f}, 0.2f};
                m2CreateCapsuleShape(body, &sd, &cap);
                break;
            }
            default:
            {
                m2Polygon box = m2MakeBox(0.35f, 0.2f);
                m2CreatePolygonShape(body, &sd, &box);
                break;
            }
            }
        }
        m2World_Step(worldId, 1.0f / 60.0f, 4);
        uint64_t worldHash = m2World_Hash(worldId);
        h = m2Hash64(h, &worldHash, (int32_t)sizeof(worldHash));
    }
    (void)world;
    m2DestroyWorld(worldId);
    return h;
}

// Editor round-trip: what you created is what you read back, to the
// exact stored bit, for every geometry kind plus the body state that
// gizmos draw from.
static void TestGeometryReadback(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 32;
    m2WorldId world = m2CreateWorld(&def);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 4.0};
    bd.isBullet = true;
    bd.gravityScale = 0.5f;
    m2BodyId body = m2CreateBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    sd.density = 2.5f;

    m2Circle circle = {{1.0f, 0.25f}, 0.4f};
    m2ShapeId circleShape = m2CreateCircleShape(body, &sd, &circle);
    m2Capsule capsule = {{-0.5f, 0.0f}, {0.5f, 0.0f}, 0.3f};
    m2ShapeId capsuleShape = m2CreateCapsuleShape(body, &sd, &capsule);
    m2Polygon poly = m2MakeBox(0.6f, 0.2f);
    m2ShapeId polyShape = m2CreatePolygonShape(body, &sd, &poly);

    m2Circle rc = m2Shape_GetCircle(circleShape);
    CHECK(rc.center.x == circle.center.x && rc.center.y == circle.center.y &&
              rc.radius == circle.radius,
          "circle reads back exactly");
    m2Capsule rcap = m2Shape_GetCapsule(capsuleShape);
    CHECK(rcap.point1.x == capsule.point1.x && rcap.point2.x == capsule.point2.x &&
              rcap.radius == capsule.radius,
          "capsule reads back exactly");
    m2Polygon rp = m2Shape_GetPolygon(polyShape);
    bool polySame = rp.count == poly.count;
    for (int32_t i = 0; polySame && i < rp.count; ++i)
    {
        polySame = rp.vertices[i].x == poly.vertices[i].x && rp.vertices[i].y == poly.vertices[i].y;
    }
    CHECK(polySame, "polygon vertices read back exactly");
    CHECK(m2Shape_GetDensity(polyShape) == 2.5f, "density reads back");

    // Body state for gizmos: the compound is symmetric about x != 0,
    // so the local center must sit off the origin, and the def flags
    // read back as set.
    m2Vec2 com = m2Body_GetLocalCenter(body);
    CHECK(com.x != 0.0f, "offset shapes move the local center");
    CHECK(m2Body_IsBullet(body), "bullet flag reads back");
    CHECK(m2Body_GetGravityScale(body) == 0.5f, "gravity scale reads back");

    // Chain links: enumerable per chain, each one a chain segment,
    // and the middle link's stored points echo the source polyline.
    m2BodyDef gd = m2DefaultBodyDef();
    m2BodyId ground = m2CreateBody(world, &gd);
    m2Vec2 pts[6] = {{6.0f, 0.0f}, {4.0f, 0.0f},  {2.0f, 0.0f},
                     {0.0f, 0.0f}, {-2.0f, 0.0f}, {-4.0f, 0.0f}};
    m2ChainDef chain = m2DefaultChainDef();
    chain.points = pts;
    chain.count = 6;
    m2ChainId ledge = m2CreateChain(ground, &chain);
    CHECK(m2Chain_GetShapes(ledge, NULL, 0) == m2Chain_GetSegmentCount(ledge),
          "chain shape walk agrees with the segment count");
    m2ShapeId links[8];
    int32_t linkCount = m2Chain_GetShapes(ledge, links, 8);
    CHECK(linkCount == 3, "three links");
    bool allSegments = true;
    bool ascending = true;
    for (int32_t i = 0; i < linkCount; ++i)
    {
        allSegments = allSegments && m2Shape_GetType(links[i]) == m2_chainSegmentShape;
        ascending = ascending && (i == 0 || links[i - 1].index1 < links[i].index1);
    }
    CHECK(allSegments && ascending, "all links are segments, ascending");
    m2ChainSegment mid = m2Shape_GetChainSegment(links[1]);
    CHECK(mid.segment.point1.x == 2.0f && mid.segment.point2.x == 0.0f && mid.ghost1.x == 4.0f &&
              mid.ghost2.x == -2.0f,
          "the middle link's points echo the polyline");

    m2DestroyWorld(world);
}

// m2ComputeHull: welding, collinear rejection, and a noisy cloud
// collapsing to the square it always was.
static float PieceArea(const m2Polygon* p)
{
    float area2 = 0.0f;
    for (int32_t i = 0; i < p->count; ++i)
    {
        m2Vec2 a = p->vertices[i];
        m2Vec2 b = p->vertices[(i + 1) % p->count];
        area2 += a.x * b.y - b.x * a.y;
    }
    return 0.5f * area2;
}

// The sprite-outline road: concave outlines split into convex pieces
// that keep the area, stay inside the 8-vertex law, and validate
// through the ordinary polygon road. Bad outlines bounce loudly.
static void TestDecomposeOutline(void)
{
    // An L: one reflex corner, two pieces, area 4.
    m2Vec2 ell[6] = {{0.0f, 0.0f}, {2.0f, 0.0f}, {2.0f, 1.0f},
                     {1.0f, 1.0f}, {1.0f, 3.0f}, {0.0f, 3.0f}};
    m2Polygon pieces[16];
    int32_t n = m2DecomposeOutline(ell, 6, pieces, 16);
    CHECK(n == 2, "an L splits into two convex pieces");
    float area = 0.0f;
    for (int32_t i = 0; i < n; ++i)
    {
        CHECK(pieces[i].count >= 3 && pieces[i].count <= 8, "pieces obey the vertex law");
        area += PieceArea(&pieces[i]);
    }
    CHECK(area > 3.999f && area < 4.001f, "the L keeps its area");

    // A plus: four reflex corners, area 20, a handful of pieces.
    m2Vec2 plus[12] = {{3.0f, -1.0f},  {3.0f, 1.0f},   {1.0f, 1.0f},  {1.0f, 3.0f},
                       {-1.0f, 3.0f},  {-1.0f, 1.0f},  {-3.0f, 1.0f}, {-3.0f, -1.0f},
                       {-1.0f, -1.0f}, {-1.0f, -3.0f}, {1.0f, -3.0f}, {1.0f, -1.0f}};
    n = m2DecomposeOutline(plus, 12, pieces, 16);
    CHECK(n >= 3 && n <= 5, "a plus needs a handful of pieces");
    area = 0.0f;
    for (int32_t i = 0; i < n; ++i)
    {
        area += PieceArea(&pieces[i]);
    }
    CHECK(area > 19.99f && area < 20.01f, "the plus keeps its area");

    // Truthful totals beyond capacity, the enumeration contract.
    m2Polygon one[1];
    int32_t total = m2DecomposeOutline(plus, 12, one, 1);
    CHECK(total == n, "capacity pressure never lies about the total");

    // A convex outline is returned whole.
    m2Vec2 box[4] = {{0.0f, 0.0f}, {1.0f, 0.0f}, {1.0f, 1.0f}, {0.0f, 1.0f}};
    n = m2DecomposeOutline(box, 4, pieces, 16);
    CHECK(n == 1 && pieces[0].count == 4, "a convex outline stays one piece");

    // The pieces build real bodies end to end.
    m2WorldDef wd = m2DefaultWorldDef();
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 16;
    wd.jointCapacity = 4;
    m2WorldId world = m2CreateWorld(&wd);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 5.0};
    m2BodyId body = m2CreateBody(world, &bd);
    int32_t made = m2DecomposeOutline(ell, 6, pieces, 16);
    for (int32_t i = 0; i < made; ++i)
    {
        m2ShapeDef sd = m2DefaultShapeDef();
        m2ShapeId sid = m2CreatePolygonShape(body, &sd, &pieces[i]);
        CHECK(m2Shape_IsValid(sid), "every piece becomes a live shape");
    }
    m2DestroyWorld(world);
}

static void TestComputeHull(void)
{
    // A square with duplicates, an interior point, and jitter on one
    // edge that should weld or merge away.
    m2Vec2 cloud[8] = {{-1.0f, -1.0f}, {1.0f, -1.0f}, {1.0f, 1.0f},  {-1.0f, 1.0f},
                       {0.0f, 0.2f},   {1.0f, -1.0f}, {0.0f, -1.0f}, {-1.0f, -0.999f}};
    m2Polygon hull = m2ComputeHull(cloud, 8, 0.0f);
    CHECK(hull.count == 4, "the noisy cloud is a square");
    float loX = 3.4e38f;
    float hiX = -3.4e38f;
    for (int32_t i = 0; i < hull.count; ++i)
    {
        loX = hull.vertices[i].x < loX ? hull.vertices[i].x : loX;
        hiX = hull.vertices[i].x > hiX ? hull.vertices[i].x : hiX;
    }
    CHECK(loX == -1.0f && hiX == 1.0f, "with the original extremes");

    // The hull feeds shape creation directly.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 4;
    def.shapeCapacity = 4;
    m2WorldId world = m2CreateWorld(&def);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 5.0};
    m2BodyId body = m2CreateBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2ShapeId shape = m2CreatePolygonShape(body, &sd, &hull);
    CHECK(m2Shape_IsValid(shape), "and the world accepts it");
    m2DestroyWorld(world);

    // Degenerates are loud, not quiet.
    m2Vec2 line[4] = {{0.0f, 0.0f}, {1.0f, 0.0f}, {2.0f, 0.0f}, {3.0f, 0.0f}};
    CHECK(m2ComputeHull(line, 4, 0.0f).count == 0, "collinear input returns count zero");
    m2Vec2 blob[3] = {{0.0f, 0.0f}, {0.001f, 0.001f}, {0.002f, 0.0f}};
    CHECK(m2ComputeHull(blob, 3, 0.0f).count == 0, "a welded-away blob returns count zero");
}

int main(void)
{
    TestValidation();
    TestComputeHull();
    TestDecomposeOutline();
    TestGeometryReadback();
    TestAabbAndMass();
    TestCompoundAndRollback();

    uint64_t hash = ShapeSweepHash();
    printf("M2_SHAPE_HASH=%016llx\n", (unsigned long long)hash);

    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
