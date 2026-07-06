// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Shape validation, world-space AABBs, and mass properties. Validation
// uses relative/geometric thresholds (topic-03 D4, RT1-NUM-1): these
// guarantees ARE the non-zero-divisor preconditions the sim path relies
// on. Mass math follows the standard area/inertia integrals (reference:
// Box2D's ComputeMass lineage, MIT).

#include "shape_internal.h"

#include "maul2d/base.h"
#include "maul2d/math.h"

// Slop-scaled geometric floors (the b2_linearSlop model). Fixed constants
// for now; harness-tuned alongside F-T2-1.
#define M2_LINEAR_SLOP     0.005f
#define M2_MIN_EDGE_LENGTH (2.0f * M2_LINEAR_SLOP)
// Thinness bound: area must exceed this fraction of perimeter^2 (a
// scale-free sliver rejector; a square scores 1/16 = 0.0625).
#define M2_MIN_THINNESS 0.001f

static bool IsFiniteF(float x)
{
    return x == x && x < 3.4e38f && x > -3.4e38f;
}

static bool IsFiniteVec(m2Vec2 v)
{
    return IsFiniteF(v.x) && IsFiniteF(v.y);
}

static float EdgeLength(m2Vec2 a, m2Vec2 b)
{
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    // sqrtf is IEEE-exact (allowed op set).
    return sqrtf(dx * dx + dy * dy);
}

bool m2ValidateCircle(const m2Circle* circle)
{
    return circle != NULL && IsFiniteVec(circle->center) && IsFiniteF(circle->radius) &&
           circle->radius >= M2_LINEAR_SLOP;
}

bool m2ValidateCapsule(const m2Capsule* capsule)
{
    if (capsule == NULL || !IsFiniteVec(capsule->point1) || !IsFiniteVec(capsule->point2) ||
        !IsFiniteF(capsule->radius) || capsule->radius < M2_LINEAR_SLOP)
    {
        return false;
    }
    // Relative floor (RT1-NUM-1): 1-ulp-apart points must not pass. The
    // axis normalization divides by this length.
    float length = EdgeLength(capsule->point1, capsule->point2);
    float scale = m2MaxF(m2AbsF(capsule->point1.x) + m2AbsF(capsule->point1.y),
                         m2AbsF(capsule->point2.x) + m2AbsF(capsule->point2.y));
    return length >= m2MaxF(M2_MIN_EDGE_LENGTH, 1.0e-5f * scale);
}

bool m2ValidateSegment(const m2Segment* segment)
{
    return segment != NULL && IsFiniteVec(segment->point1) && IsFiniteVec(segment->point2) &&
           EdgeLength(segment->point1, segment->point2) >= M2_MIN_EDGE_LENGTH;
}

bool m2ValidatePolygon(const m2Polygon* polygon)
{
    if (polygon == NULL || polygon->count < 3 || polygon->count > M2_MAX_POLYGON_VERTICES ||
        !IsFiniteF(polygon->radius) || polygon->radius < 0.0f)
    {
        return false;
    }

    float area = 0.0f;
    float perimeter = 0.0f;
    for (int32_t i = 0; i < polygon->count; ++i)
    {
        m2Vec2 a = polygon->vertices[i];
        m2Vec2 b = polygon->vertices[(i + 1) % polygon->count];
        if (!IsFiniteVec(a))
        {
            return false;
        }
        float edge = EdgeLength(a, b);
        if (edge < M2_MIN_EDGE_LENGTH)
        {
            return false; // near-coincident vertices: cancellation in normals
        }
        perimeter += edge;
        area += 0.5f * (a.x * b.y - a.y * b.x);
        // Convexity + CCW winding: every cross product must be positive.
        m2Vec2 c = polygon->vertices[(i + 2) % polygon->count];
        float cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
        if (cross <= 0.0f)
        {
            return false;
        }
    }
    // Scale-free sliver rejection (an absolute area epsilon would be
    // scale-dependent, RT1-NUM-1).
    return area > M2_MIN_THINNESS * perimeter * perimeter;
}

// Quickhull with aggressive welding and collinear merging, ported
// from the reference (hull.c) under Maul's loud-invalid convention.
static int32_t RecurseHull(m2Vec2 p1, m2Vec2 p2, const m2Vec2* ps, int32_t count, m2Vec2* out)
{
    if (count == 0)
    {
        return 0;
    }
    float ex = p2.x - p1.x;
    float ey = p2.y - p1.y;
    float len = sqrtf(ex * ex + ey * ey);
    if (!(len > 0.0f))
    {
        return 0;
    }
    ex /= len;
    ey /= len;

    m2Vec2 rightPoints[M2_MAX_POLYGON_VERTICES];
    int32_t rightCount = 0;
    int32_t bestIndex = 0;
    float bestDistance = (ps[0].x - p1.x) * ey - (ps[0].y - p1.y) * ex;
    if (bestDistance > 0.0f)
    {
        rightPoints[rightCount++] = ps[0];
    }
    for (int32_t i = 1; i < count; ++i)
    {
        float distance = (ps[i].x - p1.x) * ey - (ps[i].y - p1.y) * ex;
        if (distance > bestDistance)
        {
            bestIndex = i;
            bestDistance = distance;
        }
        if (distance > 0.0f)
        {
            rightPoints[rightCount++] = ps[i];
        }
    }
    if (bestDistance < 2.0f * M2_LINEAR_SLOP)
    {
        return 0;
    }
    m2Vec2 bestPoint = ps[bestIndex];
    int32_t n1 = RecurseHull(p1, bestPoint, rightPoints, rightCount, out);
    out[n1] = bestPoint;
    int32_t n2 = RecurseHull(bestPoint, p2, rightPoints, rightCount, out + n1 + 1);
    return n1 + 1 + n2;
}

m2Polygon m2ComputeHull(const m2Vec2* points, int32_t count, float radius)
{
    m2Polygon invalid;
    memset(&invalid, 0, sizeof(invalid));
    if (points == NULL || count < 3)
    {
        return invalid; // check your data: count == 0 is the loud sign
    }
    count = count < M2_MAX_POLYGON_VERTICES ? count : M2_MAX_POLYGON_VERTICES;

    // Aggressive welding; track the bounds for the seed pick.
    m2Vec2 ps[M2_MAX_POLYGON_VERTICES];
    int32_t n = 0;
    float loX = 3.4e38f;
    float loY = 3.4e38f;
    float hiX = -3.4e38f;
    float hiY = -3.4e38f;
    float tolSqr = 16.0f * M2_LINEAR_SLOP * M2_LINEAR_SLOP;
    for (int32_t i = 0; i < count; ++i)
    {
        loX = points[i].x < loX ? points[i].x : loX;
        loY = points[i].y < loY ? points[i].y : loY;
        hiX = points[i].x > hiX ? points[i].x : hiX;
        hiY = points[i].y > hiY ? points[i].y : hiY;
        bool unique = true;
        for (int32_t j = 0; j < n; ++j)
        {
            float dx = points[i].x - ps[j].x;
            float dy = points[i].y - ps[j].y;
            if (dx * dx + dy * dy < tolSqr)
            {
                unique = false;
                break;
            }
        }
        if (unique)
        {
            ps[n++] = points[i];
        }
    }
    if (n < 3)
    {
        return invalid; // welded away: scale problem, be loud
    }

    // Seed with the point farthest from the bounds center, then its
    // farthest partner; split the rest left/right of that line.
    m2Vec2 c = {0.5f * (loX + hiX), 0.5f * (loY + hiY)};
    int32_t f1 = 0;
    float dsq1 = (ps[0].x - c.x) * (ps[0].x - c.x) + (ps[0].y - c.y) * (ps[0].y - c.y);
    for (int32_t i = 1; i < n; ++i)
    {
        float dsq = (ps[i].x - c.x) * (ps[i].x - c.x) + (ps[i].y - c.y) * (ps[i].y - c.y);
        if (dsq > dsq1)
        {
            f1 = i;
            dsq1 = dsq;
        }
    }
    m2Vec2 p1 = ps[f1];
    ps[f1] = ps[n - 1];
    n -= 1;

    int32_t f2 = 0;
    float dsq2 = (ps[0].x - p1.x) * (ps[0].x - p1.x) + (ps[0].y - p1.y) * (ps[0].y - p1.y);
    for (int32_t i = 1; i < n; ++i)
    {
        float dsq = (ps[i].x - p1.x) * (ps[i].x - p1.x) + (ps[i].y - p1.y) * (ps[i].y - p1.y);
        if (dsq > dsq2)
        {
            f2 = i;
            dsq2 = dsq;
        }
    }
    m2Vec2 p2 = ps[f2];
    ps[f2] = ps[n - 1];
    n -= 1;

    m2Vec2 rightPoints[M2_MAX_POLYGON_VERTICES - 2];
    int32_t rightCount = 0;
    m2Vec2 leftPoints[M2_MAX_POLYGON_VERTICES - 2];
    int32_t leftCount = 0;
    float ex = p2.x - p1.x;
    float ey = p2.y - p1.y;
    float elen = sqrtf(ex * ex + ey * ey);
    ex /= elen;
    ey /= elen;
    for (int32_t i = 0; i < n; ++i)
    {
        float d = (ps[i].x - p1.x) * ey - (ps[i].y - p1.y) * ex;
        if (d >= 2.0f * M2_LINEAR_SLOP)
        {
            rightPoints[rightCount++] = ps[i];
        }
        else if (d <= -2.0f * M2_LINEAR_SLOP)
        {
            leftPoints[leftCount++] = ps[i];
        }
    }

    m2Vec2 hull[M2_MAX_POLYGON_VERTICES];
    int32_t hullCount = 0;
    hull[hullCount++] = p1;
    int32_t n1 = RecurseHull(p1, p2, rightPoints, rightCount, hull + hullCount);
    hullCount += n1;
    hull[hullCount++] = p2;
    int32_t n2 = RecurseHull(p2, p1, leftPoints, leftCount, hull + hullCount);
    hullCount += n2;
    if (n1 == 0 && n2 == 0)
    {
        return invalid; // all collinear
    }

    // Merge collinear runs until stable.
    bool searching = true;
    while (searching && hullCount > 2)
    {
        searching = false;
        for (int32_t i = 0; i < hullCount; ++i)
        {
            int32_t i1 = i;
            int32_t i2 = (i + 1) % hullCount;
            int32_t i3 = (i + 2) % hullCount;
            m2Vec2 s1 = hull[i1];
            m2Vec2 s2 = hull[i2];
            m2Vec2 s3 = hull[i3];
            float rx = s3.x - s1.x;
            float ry = s3.y - s1.y;
            float rlen = sqrtf(rx * rx + ry * ry);
            if (!(rlen > 0.0f))
            {
                continue;
            }
            rx /= rlen;
            ry /= rlen;
            float distance = (s2.x - s1.x) * ry - (s2.y - s1.y) * rx;
            if (distance <= 2.0f * M2_LINEAR_SLOP)
            {
                for (int32_t j = i2; j < hullCount - 1; ++j)
                {
                    hull[j] = hull[j + 1];
                }
                hullCount -= 1;
                searching = true;
                break;
            }
        }
    }
    if (hullCount < 3)
    {
        return invalid;
    }
    // The existing constructor does the rest: normals, centroid, and
    // its own loud validation.
    return m2MakePolygon(hull, hullCount, radius);
}

m2Polygon m2MakePolygon(const m2Vec2* points, int32_t count, float radius)
{
    m2Polygon polygon;
    memset(&polygon, 0, sizeof(polygon)); // deterministic bytes in unions
    if (points == NULL || count < 3 || count > M2_MAX_POLYGON_VERTICES)
    {
        return polygon; // count == 0 marks invalid
    }
    for (int32_t i = 0; i < count; ++i)
    {
        polygon.vertices[i] = points[i];
    }
    polygon.count = count;
    polygon.radius = radius;
    for (int32_t i = 0; i < count; ++i)
    {
        m2Vec2 a = polygon.vertices[i];
        m2Vec2 b = polygon.vertices[(i + 1) % count];
        float dx = b.x - a.x;
        float dy = b.y - a.y;
        float length = sqrtf(dx * dx + dy * dy);
        if (length < M2_MIN_EDGE_LENGTH)
        {
            memset(&polygon, 0, sizeof(polygon));
            return polygon;
        }
        float inv = 1.0f / length;
        polygon.normals[i] = (m2Vec2){dy * inv, -dx * inv};
    }
    if (!m2ValidatePolygon(&polygon))
    {
        memset(&polygon, 0, sizeof(polygon));
    }
    return polygon;
}

m2Polygon m2MakeBox(float halfWidth, float halfHeight)
{
    m2Vec2 points[4] = {{-halfWidth, -halfHeight},
                        {halfWidth, -halfHeight},
                        {halfWidth, halfHeight},
                        {-halfWidth, halfHeight}};
    return m2MakePolygon(points, 4, 0.0f);
}

// --- World-space AABBs (rotation-aware; f64 crossing at body position) -------

static m2Pos2 WorldPoint(m2Transform xf, m2Vec2 local)
{
    // The single f64 crossing for this stage: rotate in f32, then add to
    // the f64 body position (topic-01 D1 discipline).
    float x = xf.q.c * local.x - xf.q.s * local.y;
    float y = xf.q.s * local.x + xf.q.c * local.y;
    return (m2Pos2){xf.p.x + (double)x, xf.p.y + (double)y};
}

m2AABB m2ComputeShapeAABB(const m2ShapeGeometry* geometry, m2Transform xf)
{
    m2AABB aabb;
    switch (geometry->type)
    {
    case m2_circleShape:
    {
        m2Pos2 c = WorldPoint(xf, geometry->circle.center);
        double r = (double)geometry->circle.radius;
        aabb.lowerBound = (m2Pos2){c.x - r, c.y - r};
        aabb.upperBound = (m2Pos2){c.x + r, c.y + r};
        return aabb;
    }
    case m2_capsuleShape:
    {
        m2Pos2 p1 = WorldPoint(xf, geometry->capsule.point1);
        m2Pos2 p2 = WorldPoint(xf, geometry->capsule.point2);
        double r = (double)geometry->capsule.radius;
        aabb.lowerBound =
            (m2Pos2){(p1.x < p2.x ? p1.x : p2.x) - r, (p1.y < p2.y ? p1.y : p2.y) - r};
        aabb.upperBound =
            (m2Pos2){(p1.x > p2.x ? p1.x : p2.x) + r, (p1.y > p2.y ? p1.y : p2.y) + r};
        return aabb;
    }
    case m2_polygonShape:
    {
        m2Pos2 first = WorldPoint(xf, geometry->polygon.vertices[0]);
        aabb.lowerBound = first;
        aabb.upperBound = first;
        for (int32_t i = 1; i < geometry->polygon.count; ++i)
        {
            m2Pos2 p = WorldPoint(xf, geometry->polygon.vertices[i]);
            aabb.lowerBound.x = p.x < aabb.lowerBound.x ? p.x : aabb.lowerBound.x;
            aabb.lowerBound.y = p.y < aabb.lowerBound.y ? p.y : aabb.lowerBound.y;
            aabb.upperBound.x = p.x > aabb.upperBound.x ? p.x : aabb.upperBound.x;
            aabb.upperBound.y = p.y > aabb.upperBound.y ? p.y : aabb.upperBound.y;
        }
        double r = (double)geometry->polygon.radius;
        aabb.lowerBound.x -= r;
        aabb.lowerBound.y -= r;
        aabb.upperBound.x += r;
        aabb.upperBound.y += r;
        return aabb;
    }
    default:
    {
        M2_ASSERT(geometry->type == m2_segmentShape || geometry->type == m2_chainSegmentShape);
        const m2Segment* seg = geometry->type == m2_segmentShape ? &geometry->segment
                                                                 : &geometry->chainSegment.segment;
        m2Pos2 p1 = WorldPoint(xf, seg->point1);
        m2Pos2 p2 = WorldPoint(xf, seg->point2);
        aabb.lowerBound = (m2Pos2){p1.x < p2.x ? p1.x : p2.x, p1.y < p2.y ? p1.y : p2.y};
        aabb.upperBound = (m2Pos2){p1.x > p2.x ? p1.x : p2.x, p1.y > p2.y ? p1.y : p2.y};
        return aabb;
    }
    }
}

// --- Mass properties ----------------------------------------------------------

m2MassData m2ComputeShapeMass(const m2ShapeGeometry* geometry, float density)
{
    m2MassData data = {0};
    switch (geometry->type)
    {
    case m2_chainSegmentShape: // chains are massless, like segments
        return data;

    case m2_circleShape:
    {
        float r = geometry->circle.radius;
        data.mass = density * M2_PI * r * r;
        data.center = geometry->circle.center;
        // I about center + parallel axis handled by the caller.
        data.rotationalInertia =
            data.mass * (0.5f * r * r + geometry->circle.center.x * geometry->circle.center.x +
                         geometry->circle.center.y * geometry->circle.center.y);
        return data;
    }
    case m2_capsuleShape:
    {
        // Rectangle + two half discs (reference formulas).
        m2Vec2 p1 = geometry->capsule.point1;
        m2Vec2 p2 = geometry->capsule.point2;
        float r = geometry->capsule.radius;
        float length = EdgeLength(p1, p2);
        float rectMass = density * 2.0f * r * length;
        float discMass = density * M2_PI * r * r;
        data.mass = rectMass + discMass;
        data.center = (m2Vec2){0.5f * (p1.x + p2.x), 0.5f * (p1.y + p2.y)};
        float h = 0.5f * length;
        float rectInertia = rectMass * (4.0f * h * h + 4.0f * r * r) * (1.0f / 12.0f);
        float discInertia = discMass * (0.5f * r * r + h * h);
        data.rotationalInertia =
            rectInertia + discInertia +
            data.mass * (data.center.x * data.center.x + data.center.y * data.center.y);
        return data;
    }
    case m2_polygonShape:
    {
        // Standard polygon integrals about the origin, then shifted.
        float area = 0.0f;
        float inertia = 0.0f;
        m2Vec2 center = {0.0f, 0.0f};
        const m2Polygon* poly = &geometry->polygon;
        for (int32_t i = 0; i < poly->count; ++i)
        {
            m2Vec2 a = poly->vertices[i];
            m2Vec2 b = poly->vertices[(i + 1) % poly->count];
            float cross = a.x * b.y - a.y * b.x;
            float triangleArea = 0.5f * cross;
            area += triangleArea;
            center.x += triangleArea * (a.x + b.x) * (1.0f / 3.0f);
            center.y += triangleArea * (a.y + b.y) * (1.0f / 3.0f);
            float intx2 = a.x * a.x + a.x * b.x + b.x * b.x;
            float inty2 = a.y * a.y + a.y * b.y + b.y * b.y;
            inertia += (0.25f * (1.0f / 3.0f) * cross) * (intx2 + inty2);
        }
        data.mass = density * area;
        M2_ASSERT(area > 0.0f); // validation guarantees this
        float invArea = 1.0f / area;
        center.x *= invArea;
        center.y *= invArea;
        data.center = center;
        // The integral is already about the body origin.
        data.rotationalInertia = density * inertia;
        return data;
    }
    default:
    {
        // Segments are one-dimensional: no mass contribution.
        return data;
    }
    }
}

m2Polygon m2MakeSegmentProxy(m2Vec2 p1, m2Vec2 p2, float radius)
{
    m2Polygon proxy;
    memset(&proxy, 0, sizeof(proxy));
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    float length = sqrtf(dx * dx + dy * dy);
    M2_ASSERT(length >= M2_MIN_EDGE_LENGTH); // shape validation guarantees
    float inv = 1.0f / length;
    m2Vec2 axis = {dx * inv, dy * inv};
    proxy.vertices[0] = p1;
    proxy.vertices[1] = p2;
    proxy.normals[0] = (m2Vec2){axis.y, -axis.x};
    proxy.normals[1] = (m2Vec2){-axis.y, axis.x};
    proxy.count = 2;
    proxy.radius = radius;
    return proxy;
}
