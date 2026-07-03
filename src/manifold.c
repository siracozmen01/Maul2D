// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Narrowphase manifold kernels, slice 3a: circle-vs-circle and
// polygon-vs-circle (Voronoi regions). Polygon-vs-polygon (SAT + clip,
// covering capsules and segments as 2-vertex rounded polygons) lands in
// slice 3b. Structure follows Box2D v3's manifold functions (Copyright
// 2023 Erin Catto, MIT). Pure functions of their inputs: no state, only
// allowed ops, evaluated in canonical pair order by the world.

#include "shape_internal.h"

#include "maul2d/base.h"
#include "maul2d/math.h"

static m2Vec2 RotateVec(m2Rot q, m2Vec2 v)
{
    return (m2Vec2){q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y};
}

static m2Vec2 InvRotateVec(m2Rot q, m2Vec2 v)
{
    return (m2Vec2){q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y};
}

static m2Vec2 PoseTransform(m2RelativePose pose, m2Vec2 v)
{
    m2Vec2 r = RotateVec(pose.q, v);
    return (m2Vec2){r.x + pose.p.x, r.y + pose.p.y};
}

// Express a point given in A's frame back in B's frame.
static m2Vec2 PoseUntransform(m2RelativePose pose, m2Vec2 pointInA)
{
    m2Vec2 d = {pointInA.x - pose.p.x, pointInA.y - pose.p.y};
    return InvRotateVec(pose.q, d);
}

m2Manifold m2CollideCircles(const m2Circle* a, const m2Circle* b, m2RelativePose pose)
{
    m2Manifold manifold;
    memset(&manifold, 0, sizeof(manifold));

    m2Vec2 centerB = PoseTransform(pose, b->center);
    m2Vec2 d = {centerB.x - a->center.x, centerB.y - a->center.y};
    float distSq = d.x * d.x + d.y * d.y;
    float radiusSum = a->radius + b->radius;
    float maxDist = radiusSum + M2_SPECULATIVE_DISTANCE;
    if (distSq > maxDist * maxDist)
    {
        return manifold;
    }

    float dist = sqrtf(distSq);
    m2Vec2 normal;
    if (dist > 1.19209290e-7f)
    {
        float inv = 1.0f / dist;
        normal = (m2Vec2){d.x * inv, d.y * inv};
    }
    else
    {
        // Coincident centers: the canonical fallback normal (topic-05
        // NaN-free law) - deterministic, never NaN.
        normal = (m2Vec2){0.0f, 1.0f};
    }

    m2Vec2 pointInA = {a->center.x + (0.5f * (dist + a->radius - b->radius)) * normal.x,
                       a->center.y + (0.5f * (dist + a->radius - b->radius)) * normal.y};

    manifold.normal = normal;
    manifold.pointCount = 1;
    manifold.points[0].anchorA = pointInA;
    manifold.points[0].anchorB = PoseUntransform(pose, pointInA);
    manifold.points[0].separation = dist - radiusSum;
    manifold.points[0].id = 0;
    return manifold;
}

m2Manifold m2CollidePolygonAndCircle(const m2Polygon* a, const m2Circle* b, m2RelativePose pose)
{
    m2Manifold manifold;
    memset(&manifold, 0, sizeof(manifold));

    m2Vec2 center = PoseTransform(pose, b->center);
    float radiusSum = a->radius + b->radius;
    float maxDist = radiusSum + M2_SPECULATIVE_DISTANCE;

    // Deepest-penetration / closest-feature edge by support separation.
    int32_t bestEdge = 0;
    float bestSeparation = -3.4e38f;
    for (int32_t i = 0; i < a->count; ++i)
    {
        m2Vec2 n = a->normals[i];
        m2Vec2 v = a->vertices[i];
        float separation = n.x * (center.x - v.x) + n.y * (center.y - v.y);
        if (separation > bestSeparation)
        {
            bestSeparation = separation;
            bestEdge = i;
        }
    }
    if (bestSeparation > maxDist)
    {
        return manifold;
    }

    m2Vec2 v1 = a->vertices[bestEdge];
    m2Vec2 v2 = a->vertices[(bestEdge + 1) % a->count];

    m2Vec2 normal;
    m2Vec2 closest;
    uint16_t id;
    // Voronoi region of the reference edge: vertex, vertex, or face.
    float u1 = (center.x - v1.x) * (v2.x - v1.x) + (center.y - v1.y) * (v2.y - v1.y);
    float u2 = (center.x - v2.x) * (v1.x - v2.x) + (center.y - v2.y) * (v1.y - v2.y);
    if (bestSeparation > 1.19209290e-7f && u1 <= 0.0f)
    {
        closest = v1;
        id = (uint16_t)((bestEdge << 8) | 1);
    }
    else if (bestSeparation > 1.19209290e-7f && u2 <= 0.0f)
    {
        closest = v2;
        id = (uint16_t)((bestEdge << 8) | 2);
    }
    else
    {
        closest = (m2Vec2){0.0f, 0.0f}; // set below via face projection
        id = (uint16_t)(bestEdge << 8);
    }

    float separation;
    if ((id & 0xFF) != 0)
    {
        m2Vec2 d = {center.x - closest.x, center.y - closest.y};
        float dist = sqrtf(d.x * d.x + d.y * d.y);
        if (dist > maxDist)
        {
            return manifold;
        }
        if (dist > 1.19209290e-7f)
        {
            float inv = 1.0f / dist;
            normal = (m2Vec2){d.x * inv, d.y * inv};
        }
        else
        {
            normal = a->normals[bestEdge];
        }
        separation = dist - radiusSum;
    }
    else
    {
        normal = a->normals[bestEdge];
        separation = bestSeparation - radiusSum;
        closest =
            (m2Vec2){center.x - bestSeparation * normal.x, center.y - bestSeparation * normal.y};
    }

    m2Vec2 pointInA = {closest.x + a->radius * normal.x, closest.y + a->radius * normal.y};
    // Midpoint between the two surfaces along the normal.
    pointInA.x += 0.5f * (separation)*normal.x;
    pointInA.y += 0.5f * (separation)*normal.y;

    manifold.normal = normal;
    manifold.pointCount = 1;
    manifold.points[0].anchorA = pointInA;
    manifold.points[0].anchorB = PoseUntransform(pose, pointInA);
    manifold.points[0].separation = separation;
    manifold.points[0].id = id;
    return manifold;
}

// --- Polygon vs polygon (SAT + clip; adapted from Box2D v3) ------------------

#define M2_MAKE_ID(a, b) ((uint16_t)(((a) << 8) | (b)))

typedef struct m2SegmentDistanceResult
{
    m2Vec2 closest1;
    m2Vec2 closest2;
    float fraction1;
    float fraction2;
    float distanceSquared;
} m2SegmentDistanceResult;

static m2SegmentDistanceResult SegmentDistance(m2Vec2 p1, m2Vec2 q1, m2Vec2 p2, m2Vec2 q2)
{
    m2SegmentDistanceResult result = {0};
    m2Vec2 d1 = {q1.x - p1.x, q1.y - p1.y};
    m2Vec2 d2 = {q2.x - p2.x, q2.y - p2.y};
    m2Vec2 r = {p1.x - p2.x, p1.y - p2.y};
    float dd1 = d1.x * d1.x + d1.y * d1.y;
    float dd2 = d2.x * d2.x + d2.y * d2.y;
    float rd1 = r.x * d1.x + r.y * d1.y;
    float rd2 = r.x * d2.x + r.y * d2.y;
    const float epsSqr = 1.19209290e-7f * 1.19209290e-7f;

    if (dd1 < epsSqr || dd2 < epsSqr)
    {
        if (dd1 >= epsSqr)
        {
            result.fraction1 = m2ClampF(-rd1 / dd1, 0.0f, 1.0f);
        }
        else if (dd2 >= epsSqr)
        {
            result.fraction2 = m2ClampF(rd2 / dd2, 0.0f, 1.0f);
        }
    }
    else
    {
        float d12 = d1.x * d2.x + d1.y * d2.y;
        float denominator = dd1 * dd2 - d12 * d12;
        float f1 = 0.0f;
        if (denominator != 0.0f)
        {
            f1 = m2ClampF((d12 * rd2 - rd1 * dd2) / denominator, 0.0f, 1.0f);
        }
        float f2 = (d12 * f1 + rd2) / dd2;
        if (f2 < 0.0f)
        {
            f2 = 0.0f;
            f1 = m2ClampF(-rd1 / dd1, 0.0f, 1.0f);
        }
        else if (f2 > 1.0f)
        {
            f2 = 1.0f;
            f1 = m2ClampF((d12 - rd1) / dd1, 0.0f, 1.0f);
        }
        result.fraction1 = f1;
        result.fraction2 = f2;
    }
    result.closest1 = (m2Vec2){p1.x + result.fraction1 * d1.x, p1.y + result.fraction1 * d1.y};
    result.closest2 = (m2Vec2){p2.x + result.fraction2 * d2.x, p2.y + result.fraction2 * d2.y};
    float cx = result.closest1.x - result.closest2.x;
    float cy = result.closest1.y - result.closest2.y;
    result.distanceSquared = cx * cx + cy * cy;
    return result;
}

static float FindMaxSeparation(int32_t* edgeIndex, const m2Polygon* poly1, const m2Polygon* poly2)
{
    int32_t bestIndex = 0;
    float maxSeparation = -3.4e38f;
    for (int32_t i = 0; i < poly1->count; ++i)
    {
        m2Vec2 n = poly1->normals[i];
        m2Vec2 v1 = poly1->vertices[i];
        float si = 3.4e38f;
        for (int32_t j = 0; j < poly2->count; ++j)
        {
            float sij = n.x * (poly2->vertices[j].x - v1.x) + n.y * (poly2->vertices[j].y - v1.y);
            si = m2MinF(si, sij);
        }
        if (si > maxSeparation)
        {
            maxSeparation = si;
            bestIndex = i;
        }
    }
    *edgeIndex = bestIndex;
    return maxSeparation;
}

static m2Manifold ClipPolygons(const m2Polygon* polyA, const m2Polygon* polyB, int32_t edgeA,
                               int32_t edgeB, bool flip)
{
    m2Manifold manifold;
    memset(&manifold, 0, sizeof(manifold));

    const m2Polygon* poly1 = flip ? polyB : polyA;
    const m2Polygon* poly2 = flip ? polyA : polyB;
    int32_t i11 = flip ? edgeB : edgeA;
    int32_t i12;
    int32_t i21 = flip ? edgeA : edgeB;
    int32_t i22;
    i12 = i11 + 1 < poly1->count ? i11 + 1 : 0;
    i22 = i21 + 1 < poly2->count ? i21 + 1 : 0;

    m2Vec2 normal = poly1->normals[i11];
    m2Vec2 v11 = poly1->vertices[i11];
    m2Vec2 v12 = poly1->vertices[i12];
    m2Vec2 v21 = poly2->vertices[i21];
    m2Vec2 v22 = poly2->vertices[i22];
    m2Vec2 tangent = {-normal.y, normal.x}; // CCW perp

    float lower1 = 0.0f;
    float upper1 = (v12.x - v11.x) * tangent.x + (v12.y - v11.y) * tangent.y;
    float upper2 = (v21.x - v11.x) * tangent.x + (v21.y - v11.y) * tangent.y;
    float lower2 = (v22.x - v11.x) * tangent.x + (v22.y - v11.y) * tangent.y;

    if (upper2 < lower1 || upper1 < lower2)
    {
        return manifold;
    }

    m2Vec2 vLower;
    if (lower2 < lower1 && upper2 - lower2 > 1.19209290e-7f)
    {
        float t = (lower1 - lower2) / (upper2 - lower2);
        vLower = (m2Vec2){v22.x + t * (v21.x - v22.x), v22.y + t * (v21.y - v22.y)};
    }
    else
    {
        vLower = v22;
    }
    m2Vec2 vUpper;
    if (upper2 > upper1 && upper2 - lower2 > 1.19209290e-7f)
    {
        float t = (upper1 - lower2) / (upper2 - lower2);
        vUpper = (m2Vec2){v22.x + t * (v21.x - v22.x), v22.y + t * (v21.y - v22.y)};
    }
    else
    {
        vUpper = v21;
    }

    float separationLower = (vLower.x - v11.x) * normal.x + (vLower.y - v11.y) * normal.y;
    float separationUpper = (vUpper.x - v11.x) * normal.x + (vUpper.y - v11.y) * normal.y;
    float r1 = poly1->radius;
    float r2 = poly2->radius;
    vLower = (m2Vec2){vLower.x + 0.5f * (r1 - r2 - separationLower) * normal.x,
                      vLower.y + 0.5f * (r1 - r2 - separationLower) * normal.y};
    vUpper = (m2Vec2){vUpper.x + 0.5f * (r1 - r2 - separationUpper) * normal.x,
                      vUpper.y + 0.5f * (r1 - r2 - separationUpper) * normal.y};
    float radius = r1 + r2;

    if (!flip)
    {
        manifold.normal = normal;
        manifold.points[0].anchorA = vLower;
        manifold.points[0].separation = separationLower - radius;
        manifold.points[0].id = M2_MAKE_ID(i11, i22);
        manifold.points[1].anchorA = vUpper;
        manifold.points[1].separation = separationUpper - radius;
        manifold.points[1].id = M2_MAKE_ID(i12, i21);
        manifold.pointCount = 2;
    }
    else
    {
        manifold.normal = (m2Vec2){-normal.x, -normal.y};
        manifold.points[0].anchorA = vUpper;
        manifold.points[0].separation = separationUpper - radius;
        manifold.points[0].id = M2_MAKE_ID(i21, i12);
        manifold.points[1].anchorA = vLower;
        manifold.points[1].separation = separationLower - radius;
        manifold.points[1].id = M2_MAKE_ID(i22, i11);
        manifold.pointCount = 2;
    }
    return manifold;
}

m2Manifold m2CollidePolygons(const m2Polygon* a, const m2Polygon* b, m2RelativePose pose)
{
    // Shift everything near A's first vertex for round-off (reference
    // technique; our positions are already A-relative f32, this tightens
    // the last bits).
    m2Vec2 origin = a->vertices[0];

    m2Polygon localA;
    memset(&localA, 0, sizeof(localA));
    localA.count = a->count;
    localA.radius = a->radius;
    for (int32_t i = 0; i < a->count; ++i)
    {
        localA.vertices[i] = (m2Vec2){a->vertices[i].x - origin.x, a->vertices[i].y - origin.y};
        localA.normals[i] = a->normals[i];
    }
    m2Polygon localB;
    memset(&localB, 0, sizeof(localB));
    localB.count = b->count;
    localB.radius = b->radius;
    for (int32_t i = 0; i < b->count; ++i)
    {
        m2Vec2 r = {pose.q.c * b->vertices[i].x - pose.q.s * b->vertices[i].y,
                    pose.q.s * b->vertices[i].x + pose.q.c * b->vertices[i].y};
        localB.vertices[i] = (m2Vec2){r.x + pose.p.x - origin.x, r.y + pose.p.y - origin.y};
        localB.normals[i] = (m2Vec2){pose.q.c * b->normals[i].x - pose.q.s * b->normals[i].y,
                                     pose.q.s * b->normals[i].x + pose.q.c * b->normals[i].y};
    }

    int32_t edgeA = 0;
    float separationA = FindMaxSeparation(&edgeA, &localA, &localB);
    int32_t edgeB = 0;
    float separationB = FindMaxSeparation(&edgeB, &localB, &localA);
    float radius = localA.radius + localB.radius;

    m2Manifold manifold;
    memset(&manifold, 0, sizeof(manifold));
    if (separationA > M2_SPECULATIVE_DISTANCE + radius ||
        separationB > M2_SPECULATIVE_DISTANCE + radius)
    {
        return manifold;
    }

    bool flip = separationA < separationB;
    const m2Polygon* searchPoly = flip ? &localB : &localA;
    const m2Polygon* incidentPoly = flip ? &localA : &localB;
    int32_t* incidentEdge = flip ? &edgeA : &edgeB;
    m2Vec2 searchDirection = searchPoly->normals[flip ? edgeB : edgeA];
    float minDot = 3.4e38f;
    *incidentEdge = 0;
    for (int32_t i = 0; i < incidentPoly->count; ++i)
    {
        float dot = searchDirection.x * incidentPoly->normals[i].x +
                    searchDirection.y * incidentPoly->normals[i].y;
        if (dot < minDot)
        {
            minDot = dot;
            *incidentEdge = i;
        }
    }

    const float linearSlop = 0.005f;
    if (separationA > 0.1f * linearSlop || separationB > 0.1f * linearSlop)
    {
        // Disjoint edges: closest points decide vertex-vertex vs clip.
        int32_t i11 = edgeA;
        int32_t i12 = edgeA + 1 < localA.count ? edgeA + 1 : 0;
        int32_t i21 = edgeB;
        int32_t i22 = edgeB + 1 < localB.count ? edgeB + 1 : 0;
        m2SegmentDistanceResult result = SegmentDistance(
            localA.vertices[i11], localA.vertices[i12], localB.vertices[i21], localB.vertices[i22]);
        M2_ASSERT(result.distanceSquared > 0.0f);
        float distance = sqrtf(result.distanceSquared);
        if (distance - radius > M2_SPECULATIVE_DISTANCE)
        {
            return manifold; // vertex-vertex beyond the margin
        }
        manifold = ClipPolygons(&localA, &localB, edgeA, edgeB, flip);

        float minSeparation = 3.4e38f;
        for (int32_t i = 0; i < manifold.pointCount; ++i)
        {
            minSeparation = m2MinF(minSeparation, manifold.points[i].separation);
        }
        if (distance - radius + 0.1f * linearSlop < minSeparation)
        {
            // Vertex-vertex beats the clip: single-point manifold.
            bool f1 = result.fraction1 > 0.5f;
            bool f2 = result.fraction2 > 0.5f;
            m2Vec2 pA = f1 ? localA.vertices[i12] : localA.vertices[i11];
            m2Vec2 pB = f2 ? localB.vertices[i22] : localB.vertices[i21];
            float invDistance = 1.0f / distance;
            m2Vec2 normal = {(pB.x - pA.x) * invDistance, (pB.y - pA.y) * invDistance};
            m2Vec2 c1 = {pA.x + localA.radius * normal.x, pA.y + localA.radius * normal.y};
            m2Vec2 c2 = {pB.x - localB.radius * normal.x, pB.y - localB.radius * normal.y};
            manifold.normal = normal;
            manifold.points[0].anchorA = (m2Vec2){0.5f * (c1.x + c2.x), 0.5f * (c1.y + c2.y)};
            manifold.points[0].separation = distance - radius;
            manifold.points[0].id = M2_MAKE_ID(f1 ? i12 : i11, f2 ? i22 : i21);
            manifold.points[0].normalImpulse = 0.0f;
            manifold.points[0].tangentImpulse = 0.0f;
            manifold.points[0].flags = 0;
            manifold.pointCount = 1;
        }
    }
    else
    {
        manifold = ClipPolygons(&localA, &localB, edgeA, edgeB, flip);
    }

    // Undo the origin shift and fill B-frame anchors.
    m2RelativePose inverse;
    inverse.q = (m2Rot){pose.q.c, -pose.q.s};
    {
        m2Vec2 r = {inverse.q.c * pose.p.x - inverse.q.s * pose.p.y,
                    inverse.q.s * pose.p.x + inverse.q.c * pose.p.y};
        inverse.p = (m2Vec2){-r.x, -r.y};
    }
    for (int32_t i = 0; i < manifold.pointCount; ++i)
    {
        m2Vec2 pointInA = {manifold.points[i].anchorA.x + origin.x,
                           manifold.points[i].anchorA.y + origin.y};
        manifold.points[i].anchorA = pointInA;
        m2Vec2 rb = {inverse.q.c * pointInA.x - inverse.q.s * pointInA.y,
                     inverse.q.s * pointInA.x + inverse.q.c * pointInA.y};
        manifold.points[i].anchorB = (m2Vec2){rb.x + inverse.p.x, rb.y + inverse.p.y};
    }
    return manifold;
}

// --- Point-to-shape distance (bullet CCD kernel) ------------------------------

static float PointSegmentDistance(m2Vec2 p, m2Vec2 a, m2Vec2 b)
{
    m2Vec2 d = {b.x - a.x, b.y - a.y};
    m2Vec2 r = {p.x - a.x, p.y - a.y};
    float dd = d.x * d.x + d.y * d.y;
    float t = dd > 0.0f ? m2ClampF((r.x * d.x + r.y * d.y) / dd, 0.0f, 1.0f) : 0.0f;
    m2Vec2 c = {a.x + t * d.x - p.x, a.y + t * d.y - p.y};
    return sqrtf(c.x * c.x + c.y * c.y);
}

float m2PointShapeDistance(const m2ShapeGeometry* geometry, m2Vec2 point)
{
    switch (geometry->type)
    {
    case m2_circleShape:
    {
        m2Vec2 d = {point.x - geometry->circle.center.x, point.y - geometry->circle.center.y};
        return sqrtf(d.x * d.x + d.y * d.y) - geometry->circle.radius;
    }
    case m2_capsuleShape:
    {
        return PointSegmentDistance(point, geometry->capsule.point1, geometry->capsule.point2) -
               geometry->capsule.radius;
    }
    case m2_segmentShape:
    {
        return PointSegmentDistance(point, geometry->segment.point1, geometry->segment.point2);
    }
    default:
    {
        const m2Polygon* poly = &geometry->polygon;
        float maxSeparation = -3.4e38f;
        for (int32_t i = 0; i < poly->count; ++i)
        {
            float sep = poly->normals[i].x * (point.x - poly->vertices[i].x) +
                        poly->normals[i].y * (point.y - poly->vertices[i].y);
            maxSeparation = m2MaxF(maxSeparation, sep);
        }
        if (maxSeparation <= 0.0f)
        {
            return maxSeparation - poly->radius; // inside the core
        }
        float best = 3.4e38f;
        for (int32_t i = 0; i < poly->count; ++i)
        {
            float d = PointSegmentDistance(point, poly->vertices[i],
                                           poly->vertices[(i + 1) % poly->count]);
            best = m2MinF(best, d);
        }
        return best - poly->radius;
    }
    }
}
