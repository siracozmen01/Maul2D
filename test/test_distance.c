// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Distance gate: the single GJK kernel (m2ShapeDistance) that every
// convex proxy shares. This suite drives its simplex solver through
// each Voronoi region a two- and three-vertex simplex can land in, and
// checks correctness with a general oracle: for a separated pair the
// returned normal must TRULY separate the cores, so the projected gap
// along it equals the reported distance and the witnesses are the
// supports. A wrong closest point cannot satisfy that. Analytic cases
// pin exact numbers, an orbit sweep walks the regions, degenerate and
// overlapping inputs must stay finite, and the shape cast is exercised
// both ways. White-box (the proxy kernel is internal); no gated hash.

#include "world_internal.h"

#include "maul2d/base.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

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

static m2DistanceProxy Proxy(const m2Vec2* pts, int32_t n, float radius)
{
    m2DistanceProxy p;
    p.count = n;
    p.radius = radius;
    for (int32_t i = 0; i < n; ++i)
    {
        p.points[i] = pts[i];
    }
    return p;
}

static float Dot(m2Vec2 a, m2Vec2 b)
{
    return a.x * b.x + a.y * b.y;
}

// The separating-axis invariant. For a correct GJK distance on
// separated cores, the returned normal separates them: every point of A
// projects at or below the witness on A, every point of B at or above
// the witness on B, and the projected gap equals the reported distance.
// A wrong closest point yields a normal that does not fully separate, so
// the gap comes out short of the claimed distance and this catches it.
static void CheckSeparation(const m2DistanceProxy* a, const m2DistanceProxy* b, m2DistanceResult r,
                            const char* msg)
{
    if (r.distance <= 1.0e-4f)
    {
        return; // overlap or touching: checked by the caller instead
    }
    float nlen = sqrtf(r.normal.x * r.normal.x + r.normal.y * r.normal.y);
    CHECK(fabsf(nlen - 1.0f) < 1.0e-3f, msg);
    float maxA = -1.0e30f;
    for (int32_t i = 0; i < a->count; ++i)
    {
        float d = Dot(a->points[i], r.normal);
        if (d > maxA)
        {
            maxA = d;
        }
    }
    float minB = 1.0e30f;
    for (int32_t i = 0; i < b->count; ++i)
    {
        float d = Dot(b->points[i], r.normal);
        if (d < minB)
        {
            minB = d;
        }
    }
    CHECK(fabsf((minB - maxA) - r.distance) < 3.0e-3f, msg);
    CHECK(Dot(r.pointA, r.normal) - maxA < 3.0e-3f, msg);
    CHECK(minB - Dot(r.pointB, r.normal) < 3.0e-3f, msg);
    CHECK(r.distance == r.distance, msg); // never NaN
}

static void TestAnalyticDistances(void)
{
    // Two points: a clean 3-4-5.
    m2Vec2 pa[1] = {{0.0f, 0.0f}};
    m2Vec2 pb[1] = {{3.0f, 4.0f}};
    m2DistanceProxy a = Proxy(pa, 1, 0.0f);
    m2DistanceProxy b = Proxy(pb, 1, 0.0f);
    m2DistanceResult r = m2ShapeDistance(&a, &b);
    CHECK(fabsf(r.distance - 5.0f) < 1.0e-4f, "point-point distance is 5");
    CHECK(fabsf(r.normal.x - 0.6f) < 1.0e-3f && fabsf(r.normal.y - 0.8f) < 1.0e-3f,
          "point-point normal is (0.6,0.8)");

    // Point to a vertical segment: closest is the segment interior.
    m2Vec2 sp[2] = {{5.0f, -1.0f}, {5.0f, 1.0f}};
    m2DistanceProxy seg = Proxy(sp, 2, 0.0f);
    m2DistanceProxy pt = Proxy(pa, 1, 0.0f);
    r = m2ShapeDistance(&pt, &seg);
    CHECK(fabsf(r.distance - 5.0f) < 1.0e-4f, "point-segment interior distance is 5");
    CHECK(fabsf(r.normal.x - 1.0f) < 1.0e-3f, "point-segment normal is +x");

    // Box to box, face to face: the 2-unit gap between x=1 and x=3.
    m2Vec2 boxA[4] = {{-1.0f, -1.0f}, {1.0f, -1.0f}, {1.0f, 1.0f}, {-1.0f, 1.0f}};
    m2Vec2 boxB[4] = {{3.0f, -1.0f}, {5.0f, -1.0f}, {5.0f, 1.0f}, {3.0f, 1.0f}};
    m2DistanceProxy ba = Proxy(boxA, 4, 0.0f);
    m2DistanceProxy bb = Proxy(boxB, 4, 0.0f);
    r = m2ShapeDistance(&ba, &bb);
    CHECK(fabsf(r.distance - 2.0f) < 1.0e-4f, "box face-face distance is 2");
    CheckSeparation(&ba, &bb, r, "box face-face separation");

    // Box to box, corner to corner: vertex (1,1) to vertex (3,3).
    m2Vec2 boxC[4] = {{3.0f, 3.0f}, {5.0f, 3.0f}, {5.0f, 5.0f}, {3.0f, 5.0f}};
    m2DistanceProxy bc = Proxy(boxC, 4, 0.0f);
    r = m2ShapeDistance(&ba, &bc);
    CHECK(fabsf(r.distance - sqrtf(8.0f)) < 1.0e-3f, "box corner-corner distance is sqrt(8)");
    CheckSeparation(&ba, &bc, r, "box corner-corner separation");
}

static void TestSimplexRegionSweep(void)
{
    // A fixed triangle, and a small triangle orbited around it at three
    // radii: far (2-simplex closest features), mid, and near/overlapping
    // (3-simplex, every vertex and edge region of the triangle solver).
    m2Vec2 triA[3] = {{0.0f, 0.0f}, {2.0f, 0.0f}, {1.0f, 2.0f}};
    m2DistanceProxy a = Proxy(triA, 3, 0.0f);

    const float radii[3] = {4.0f, 2.0f, 1.2f};
    int32_t separated = 0;
    int32_t overlapped = 0;
    for (int32_t ri = 0; ri < 3; ++ri)
    {
        for (int32_t step = 0; step < 48; ++step)
        {
            float ang = (float)step * (2.0f * M2_PI / 48.0f);
            // Rotate the direction with a pinned rotation so the sweep is
            // deterministic and libm-free.
            m2Rot q = m2MakeRot(ang);
            float cx = 1.0f + radii[ri] * q.c; // orbit about the triangle's rough center
            float cy = 0.7f + radii[ri] * q.s;
            m2Vec2 triB[3] = {{cx - 0.3f, cy - 0.3f}, {cx + 0.3f, cy - 0.2f}, {cx, cy + 0.4f}};
            m2DistanceProxy b = Proxy(triB, 3, 0.0f);
            // Both argument orders: swapping negates the Minkowski
            // difference, so the simplex solver lands in the mirror Voronoi
            // regions and both halves of each branch get walked.
            for (int32_t swap = 0; swap < 2; ++swap)
            {
                const m2DistanceProxy* pa = swap ? &b : &a;
                const m2DistanceProxy* pb = swap ? &a : &b;
                m2DistanceResult r = m2ShapeDistance(pa, pb);
                CHECK(r.distance == r.distance, "sweep distance is never NaN");
                CHECK(r.distance >= 0.0f, "sweep distance is non-negative");
                if (r.distance > 1.0e-4f)
                {
                    CheckSeparation(pa, pb, r, "sweep separation invariant");
                    separated += 1;
                }
                else
                {
                    CHECK(r.normal.x == 0.0f && r.normal.y == 0.0f,
                          "an overlapping pair reports a zero normal");
                    overlapped += 1;
                }
            }
        }
    }

    // A second family with different simplex structure: a box orbiting a
    // segment, both argument orders, to reach the vertex and edge regions
    // the triangle sweep does not land in.
    m2Vec2 segA[2] = {{-1.5f, 0.0f}, {1.5f, 0.0f}};
    m2DistanceProxy sa = Proxy(segA, 2, 0.0f);
    for (int32_t ri = 0; ri < 3; ++ri)
    {
        for (int32_t step = 0; step < 40; ++step)
        {
            m2Rot q = m2MakeRot((float)step * (2.0f * M2_PI / 40.0f));
            float cx = radii[ri] * q.c;
            float cy = radii[ri] * q.s;
            m2Vec2 box[4] = {{cx - 0.4f, cy - 0.4f},
                             {cx + 0.4f, cy - 0.4f},
                             {cx + 0.4f, cy + 0.4f},
                             {cx - 0.4f, cy + 0.4f}};
            m2DistanceProxy bx = Proxy(box, 4, 0.0f);
            for (int32_t swap = 0; swap < 2; ++swap)
            {
                const m2DistanceProxy* pa = swap ? &bx : &sa;
                const m2DistanceProxy* pb = swap ? &sa : &bx;
                m2DistanceResult r = m2ShapeDistance(pa, pb);
                CHECK(r.distance == r.distance, "segment-box sweep is never NaN");
                if (r.distance > 1.0e-4f)
                {
                    CheckSeparation(pa, pb, r, "segment-box separation invariant");
                    separated += 1;
                }
                else
                {
                    overlapped += 1;
                }
            }
        }
    }
    // The sweep must actually exercise both outcomes, or it proves nothing.
    CHECK(separated > 0, "the sweep produced separated configurations");
    CHECK(overlapped > 0, "the sweep produced overlapping configurations");
}

static void TestOverlapAndContainment(void)
{
    // Two overlapping boxes: the origin is enclosed in the Minkowski
    // difference, so distance 0 and a zero normal.
    m2Vec2 boxA[4] = {{-1.0f, -1.0f}, {1.0f, -1.0f}, {1.0f, 1.0f}, {-1.0f, 1.0f}};
    m2Vec2 boxB[4] = {{0.0f, 0.0f}, {2.0f, 0.0f}, {2.0f, 2.0f}, {0.0f, 2.0f}};
    m2DistanceProxy a = Proxy(boxA, 4, 0.0f);
    m2DistanceProxy b = Proxy(boxB, 4, 0.0f);
    m2DistanceResult r = m2ShapeDistance(&a, &b);
    CHECK(r.distance == 0.0f, "overlapping boxes report distance 0");
    CHECK(r.normal.x == 0.0f && r.normal.y == 0.0f, "overlapping boxes report a zero normal");

    // A small box fully inside a large one: still an overlap.
    m2Vec2 big[4] = {{-5.0f, -5.0f}, {5.0f, -5.0f}, {5.0f, 5.0f}, {-5.0f, 5.0f}};
    m2Vec2 small[4] = {{-0.5f, -0.5f}, {0.5f, -0.5f}, {0.5f, 0.5f}, {-0.5f, 0.5f}};
    m2DistanceProxy bg = Proxy(big, 4, 0.0f);
    m2DistanceProxy sm = Proxy(small, 4, 0.0f);
    r = m2ShapeDistance(&bg, &sm);
    CHECK(r.distance == 0.0f, "a contained box reports distance 0");
}

static void TestDegenerate(void)
{
    // Three collinear points as a proxy: the triangle solver's normal
    // goes to zero, and the closest point must still be the near end.
    m2Vec2 lineA[3] = {{0.0f, 0.0f}, {1.0f, 0.0f}, {2.0f, 0.0f}};
    m2Vec2 above[1] = {{0.0f, 5.0f}};
    m2DistanceProxy a = Proxy(lineA, 3, 0.0f);
    m2DistanceProxy b = Proxy(above, 1, 0.0f);
    m2DistanceResult r = m2ShapeDistance(&a, &b);
    CHECK(r.distance == r.distance, "collinear proxy distance is not NaN");
    CHECK(fabsf(r.distance - 5.0f) < 1.0e-3f,
          "collinear proxy closest is the near end, distance 5");

    // Coincident single points: touching cores, distance 0.
    m2Vec2 same[1] = {{1.0f, 1.0f}};
    m2DistanceProxy p = Proxy(same, 1, 0.0f);
    r = m2ShapeDistance(&p, &p);
    CHECK(r.distance == 0.0f, "coincident points touch at distance 0");
    CHECK(r.normal.x == 0.0f && r.normal.y == 0.0f, "coincident points report a zero normal");

    // A duplicated-vertex "triangle" (two coincident points) must not
    // break the solver.
    m2Vec2 dup[3] = {{0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 2.0f}};
    m2Vec2 far[1] = {{6.0f, 1.0f}};
    m2DistanceProxy da = Proxy(dup, 3, 0.0f);
    m2DistanceProxy fb = Proxy(far, 1, 0.0f);
    r = m2ShapeDistance(&da, &fb);
    CHECK(r.distance == r.distance, "duplicate-vertex proxy is not NaN");
    CHECK(r.distance > 5.0f && r.distance < 7.0f, "duplicate-vertex proxy distance is sane");
}

static void TestShapeCast(void)
{
    // A unit box cast toward a static unit box two units away: it hits
    // partway. Then cast away: it misses.
    m2Vec2 boxA[4] = {{2.0f, -1.0f}, {4.0f, -1.0f}, {4.0f, 1.0f}, {2.0f, 1.0f}};
    m2Vec2 boxB[4] = {{-1.0f, -1.0f}, {1.0f, -1.0f}, {1.0f, 1.0f}, {-1.0f, 1.0f}};
    m2DistanceProxy a = Proxy(boxA, 4, 0.0f);
    m2DistanceProxy b = Proxy(boxB, 4, 0.0f);

    m2CastResult hit = m2ShapeCastProxy(&a, &b, (m2Vec2){4.0f, 0.0f}, 1.0f);
    CHECK(hit.hit, "a box cast into another box hits");
    CHECK(hit.fraction > 0.0f && hit.fraction < 1.0f, "the hit is partway along the cast");
    CHECK(fabsf(hit.fraction - 0.25f) < 1.0e-2f,
          "the gap of 1 over a translation of 4 hits at 0.25");

    m2CastResult miss = m2ShapeCastProxy(&a, &b, (m2Vec2){-4.0f, 0.0f}, 1.0f);
    CHECK(!miss.hit, "a box cast away from another box misses");

    // Already overlapping at the start: hit at fraction 0 with a zero
    // normal, the ray convention.
    m2Vec2 boxC[4] = {{0.5f, -1.0f}, {2.5f, -1.0f}, {2.5f, 1.0f}, {0.5f, 1.0f}};
    m2DistanceProxy c = Proxy(boxC, 4, 0.0f);
    m2CastResult initial = m2ShapeCastProxy(&c, &b, (m2Vec2){1.0f, 0.0f}, 1.0f);
    CHECK(initial.hit && initial.fraction == 0.0f, "initial overlap casts as a hit at fraction 0");
    CHECK(initial.normal.x == 0.0f && initial.normal.y == 0.0f,
          "initial overlap has a zero normal");
}

static void TestFromGeometry(void)
{
    // The proxy builder must round-trip a real polygon geometry: a box
    // built through the shape path should measure the same as the hand
    // proxy. This keeps m2GeometryProxy honest.
    m2Polygon poly = m2MakeBox(1.0f, 1.0f);
    m2ShapeGeometry geom;
    geom.type = m2_polygonShape;
    geom.polygon = poly;
    m2DistanceProxy a = m2GeometryProxy(&geom);
    m2Vec2 pb[1] = {{5.0f, 0.0f}};
    m2DistanceProxy b = Proxy(pb, 1, 0.0f);
    m2DistanceResult r = m2ShapeDistance(&a, &b);
    CHECK(fabsf(r.distance - 4.0f) < 1.0e-3f, "a geometry box measures 4 to a point at x=5");
}

int main(void)
{
    TestAnalyticDistances();
    TestSimplexRegionSweep();
    TestOverlapAndContainment();
    TestDegenerate();
    TestShapeCast();
    TestFromGeometry();

    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
