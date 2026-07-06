// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// World queries (topic-09 API surface): closest ray cast and AABB
// overlap over the broadphase trees. Read-only by construction - no
// world state is touched, so a query storm between steps cannot move
// the simulation hash. Results are canonical: the ray tie-breaks equal
// fractions on the lower shape index, overlap lists sort ascending.
//
// Shape kernels are adapted from Box2D v3.1.1 geometry.c (MIT,
// Copyright Erin Catto), reworked for Maul's frames: the ray drops
// into each body's local frame through one f64->f32 crossing, exactly
// like the narrowphase, so casts stay exact far from the origin.

#include "world_internal.h"

#include "maul2d/base.h"

#include <math.h>
#include <stdlib.h>

m2QueryFilter m2DefaultQueryFilter(void)
{
    m2QueryFilter filter = {1u, 0xFFFFFFFFu};
    return filter;
}

static bool QueryShouldSee(const m2World* world, int32_t shapeIndex, m2QueryFilter filter)
{
    return (filter.categoryBits & world->shapeMask[shapeIndex]) != 0 &&
           (world->shapeCategory[shapeIndex] & filter.maskBits) != 0;
}

typedef struct m2CastHit
{
    m2Vec2 point; // body-local
    m2Vec2 normal;
    float fraction; // of the local ray length parameter (0..maxFraction)
    bool hit;
} m2CastHit;

static float LengthAndNormalize(m2Vec2* out, m2Vec2 v)
{
    float length = sqrtf(v.x * v.x + v.y * v.y);
    if (length < 1.19209290e-7f)
    {
        *out = (m2Vec2){0.0f, 0.0f};
        return 0.0f;
    }
    out->x = v.x / length;
    out->y = v.y / length;
    return length;
}

// Circle kernel (reference structure).
static m2CastHit RayCastCircle(m2Vec2 p1, m2Vec2 d, float maxFraction, m2Vec2 center, float radius)
{
    m2CastHit output = {{0.0f, 0.0f}, {0.0f, 0.0f}, 0.0f, false};
    m2Vec2 s = {p1.x - center.x, p1.y - center.y};
    float rr = radius * radius;

    m2Vec2 unit;
    float length = LengthAndNormalize(&unit, d);
    if (length == 0.0f)
    {
        if (s.x * s.x + s.y * s.y < rr)
        {
            output.point = p1;
            output.hit = true;
        }
        return output;
    }

    float t = -(s.x * unit.x + s.y * unit.y);
    m2Vec2 c = {s.x + t * unit.x, s.y + t * unit.y};
    float cc = c.x * c.x + c.y * c.y;
    if (cc > rr)
    {
        return output;
    }

    float h = sqrtf(rr - cc);
    float fraction = t - h;
    if (fraction < 0.0f || maxFraction * length < fraction)
    {
        if (s.x * s.x + s.y * s.y < rr)
        {
            output.point = p1;
            output.hit = true;
        }
        return output;
    }

    m2Vec2 hitPoint = {s.x + fraction * unit.x, s.y + fraction * unit.y};
    float invRadius = radius > 0.0f ? 1.0f / radius : 0.0f;
    output.fraction = fraction / length;
    output.normal = (m2Vec2){hitPoint.x * invRadius, hitPoint.y * invRadius};
    output.point =
        (m2Vec2){center.x + radius * output.normal.x, center.y + radius * output.normal.y};
    output.hit = true;
    return output;
}

// Two-sided segment kernel (reference structure).
static m2CastHit RayCastSegment(m2Vec2 p1, m2Vec2 d, float maxFraction, m2Vec2 v1, m2Vec2 v2)
{
    m2CastHit output = {{0.0f, 0.0f}, {0.0f, 0.0f}, 0.0f, false};
    m2Vec2 e = {v2.x - v1.x, v2.y - v1.y};
    m2Vec2 eUnit;
    float length = LengthAndNormalize(&eUnit, e);
    if (length == 0.0f)
    {
        return output;
    }

    m2Vec2 normal = {eUnit.y, -eUnit.x}; // right perp
    float numerator = normal.x * (v1.x - p1.x) + normal.y * (v1.y - p1.y);
    float denominator = normal.x * d.x + normal.y * d.y;
    if (denominator == 0.0f)
    {
        return output;
    }

    float t = numerator / denominator;
    if (t < 0.0f || maxFraction < t)
    {
        return output;
    }

    m2Vec2 p = {p1.x + t * d.x, p1.y + t * d.y};
    float s = (p.x - v1.x) * eUnit.x + (p.y - v1.y) * eUnit.y;
    if (s < 0.0f || length < s)
    {
        return output;
    }
    if (numerator > 0.0f)
    {
        normal = (m2Vec2){-normal.x, -normal.y};
    }
    output.fraction = t;
    output.point = p;
    output.normal = normal;
    output.hit = true;
    return output;
}

// Capsule kernel (reference structure).
static m2CastHit RayCastCapsule(m2Vec2 p1, m2Vec2 d, float maxFraction, m2Vec2 v1, m2Vec2 v2,
                                float radius)
{
    m2CastHit output = {{0.0f, 0.0f}, {0.0f, 0.0f}, 0.0f, false};
    m2Vec2 a;
    float capsuleLength = LengthAndNormalize(&a, (m2Vec2){v2.x - v1.x, v2.y - v1.y});
    if (capsuleLength == 0.0f)
    {
        return RayCastCircle(p1, d, maxFraction, v1, radius);
    }

    m2Vec2 q = {p1.x - v1.x, p1.y - v1.y};
    float qa = q.x * a.x + q.y * a.y;
    m2Vec2 qp = {q.x - qa * a.x, q.y - qa * a.y};

    if (qp.x * qp.x + qp.y * qp.y < radius * radius)
    {
        if (qa < 0.0f)
        {
            return RayCastCircle(p1, d, maxFraction, v1, radius);
        }
        if (qa > capsuleLength)
        {
            return RayCastCircle(p1, d, maxFraction, v2, radius);
        }
        output.point = p1;
        output.hit = true;
        return output;
    }

    m2Vec2 n = {a.y, -a.x};
    m2Vec2 u;
    float rayLength = LengthAndNormalize(&u, d);
    if (rayLength == 0.0f)
    {
        return output;
    }

    float den = -a.x * u.y + u.x * a.y;
    if (den > -1.19209290e-7f && den < 1.19209290e-7f)
    {
        return output; // parallel and outside
    }

    m2Vec2 b1 = {q.x - radius * n.x, q.y - radius * n.y};
    m2Vec2 b2 = {q.x + radius * n.x, q.y + radius * n.y};
    float invDen = 1.0f / den;
    float s21 = (a.x * b1.y - b1.x * a.y) * invDen;
    float s22 = (a.x * b2.y - b2.x * a.y) * invDen;

    float s2;
    m2Vec2 b;
    if (s21 < s22)
    {
        s2 = s21;
        b = b1;
    }
    else
    {
        s2 = s22;
        b = b2;
        n = (m2Vec2){-n.x, -n.y};
    }

    if (s2 < 0.0f || maxFraction * rayLength < s2)
    {
        return output;
    }

    float s1 = (-b.x * u.y + u.x * b.y) * invDen;
    if (s1 < 0.0f)
    {
        return RayCastCircle(p1, d, maxFraction, v1, radius);
    }
    if (capsuleLength < s1)
    {
        return RayCastCircle(p1, d, maxFraction, v2, radius);
    }

    float lerp = s1 / capsuleLength;
    output.fraction = s2 / rayLength;
    output.point = (m2Vec2){v1.x + lerp * (v2.x - v1.x) + radius * n.x,
                            v1.y + lerp * (v2.y - v1.y) + radius * n.y};
    output.normal = n;
    output.hit = true;
    return output;
}

// Sharp polygon kernel (reference structure, radius == 0 path).
static m2CastHit RayCastSharpPolygon(m2Vec2 p1In, m2Vec2 d, float maxFraction,
                                     const m2Polygon* shape)
{
    m2CastHit output = {{0.0f, 0.0f}, {0.0f, 0.0f}, 0.0f, false};
    // Shift the math to the first vertex (the polygon may sit far from
    // the body origin).
    m2Vec2 base = shape->vertices[0];
    m2Vec2 p1 = {p1In.x - base.x, p1In.y - base.y};

    float lower = 0.0f;
    float upper = maxFraction;
    int32_t index = -1;

    for (int32_t i = 0; i < shape->count; ++i)
    {
        m2Vec2 vertex = {shape->vertices[i].x - base.x, shape->vertices[i].y - base.y};
        float numerator =
            shape->normals[i].x * (vertex.x - p1.x) + shape->normals[i].y * (vertex.y - p1.y);
        float denominator = shape->normals[i].x * d.x + shape->normals[i].y * d.y;

        if (denominator == 0.0f)
        {
            if (numerator < 0.0f)
            {
                return output;
            }
        }
        else
        {
            if (denominator < 0.0f && numerator < lower * denominator)
            {
                lower = numerator / denominator;
                index = i;
            }
            else if (denominator > 0.0f && numerator < upper * denominator)
            {
                upper = numerator / denominator;
            }
        }

        if (upper < lower)
        {
            return output;
        }
    }

    if (index >= 0)
    {
        output.fraction = lower;
        output.normal = shape->normals[index];
        output.point = (m2Vec2){p1In.x + lower * d.x, p1In.y + lower * d.y};
        output.hit = true;
    }
    else
    {
        output.point = p1In;
        output.hit = true;
    }
    return output;
}

static void TakeBetter(m2CastHit* best, m2CastHit candidate)
{
    if (candidate.hit && (!best->hit || candidate.fraction < best->fraction))
    {
        *best = candidate;
    }
}

// Rounded polygons cast as the union of offset edges and vertex
// circles: exact, and built from kernels that are already exact.
static m2CastHit RayCastPolygon(m2Vec2 p1, m2Vec2 d, float maxFraction, const m2Polygon* shape)
{
    if (shape->radius == 0.0f)
    {
        return RayCastSharpPolygon(p1, d, maxFraction, shape);
    }
    m2CastHit best = {{0.0f, 0.0f}, {0.0f, 0.0f}, 0.0f, false};
    for (int32_t i = 0; i < shape->count; ++i)
    {
        int32_t j = i + 1 < shape->count ? i + 1 : 0;
        m2Vec2 offset = {shape->normals[i].x * shape->radius, shape->normals[i].y * shape->radius};
        m2Vec2 e1 = {shape->vertices[i].x + offset.x, shape->vertices[i].y + offset.y};
        m2Vec2 e2 = {shape->vertices[j].x + offset.x, shape->vertices[j].y + offset.y};
        TakeBetter(&best, RayCastSegment(p1, d, maxFraction, e1, e2));
        TakeBetter(&best, RayCastCircle(p1, d, maxFraction, shape->vertices[i], shape->radius));
    }
    return best;
}

static m2CastHit RayCastGeometry(const m2ShapeGeometry* geometry, m2Vec2 p1, m2Vec2 d,
                                 float maxFraction)
{
    switch (geometry->type)
    {
    case m2_circleShape:
        return RayCastCircle(p1, d, maxFraction, geometry->circle.center, geometry->circle.radius);
    case m2_capsuleShape:
        return RayCastCapsule(p1, d, maxFraction, geometry->capsule.point1,
                              geometry->capsule.point2, geometry->capsule.radius);
    case m2_segmentShape:
        return RayCastSegment(p1, d, maxFraction, geometry->segment.point1,
                              geometry->segment.point2);
    case m2_chainSegmentShape:
    {
        // One-sided, like the collision: rays from the ghost side miss.
        const m2Segment* seg = &geometry->chainSegment.segment;
        m2Vec2 e = {seg->point2.x - seg->point1.x, seg->point2.y - seg->point1.y};
        float offset = (p1.x - seg->point1.x) * e.y - (p1.y - seg->point1.y) * e.x;
        if (offset < 0.0f) // reference sign: skip rays from the ghost side
        {
            m2CastHit missHit = {{0.0f, 0.0f}, {0.0f, 0.0f}, 0.0f, false};
            return missHit;
        }
        return RayCastSegment(p1, d, maxFraction, seg->point1, seg->point2);
    }
    default:
        return RayCastPolygon(p1, d, maxFraction, &geometry->polygon);
    }
}

// Ray vs one shape in the body frame: one f64 subtraction per body is
// the only precision crossing, same law as the narrowphase.
static m2CastHit RayCastShape(const m2World* world, int32_t shapeIndex, m2Pos2 origin, m2Vec2 d,
                              float maxFraction)
{
    int32_t body = world->shapeBody[shapeIndex];
    m2Transform xf = world->transforms[body];
    m2Vec2 rel = {(float)(origin.x - xf.p.x), (float)(origin.y - xf.p.y)};
    // Inverse-rotate into the body frame.
    m2Vec2 pLocal = {xf.q.c * rel.x + xf.q.s * rel.y, -xf.q.s * rel.x + xf.q.c * rel.y};
    m2Vec2 dLocal = {xf.q.c * d.x + xf.q.s * d.y, -xf.q.s * d.x + xf.q.c * d.y};
    m2CastHit hit = RayCastGeometry(&world->shapeGeometry[shapeIndex], pLocal, dLocal, maxFraction);
    if (hit.hit)
    {
        // Rotate the normal back out; the point is rebuilt in f64 by
        // the caller from the fraction.
        m2Vec2 n = hit.normal;
        hit.normal = (m2Vec2){xf.q.c * n.x - xf.q.s * n.y, xf.q.s * n.x + xf.q.c * n.y};
    }
    return hit;
}

typedef struct m2RayState
{
    m2Pos2 origin;
    m2Vec2 translation;
    m2QueryFilter filter;
    float fraction; // current best (starts at 1)
    int32_t shapeIndex;
    m2Vec2 normal;
    bool hit;
    bool initialOverlap;
} m2RayState;

// Node cull: segment AABB overlap plus the reference's separating-line
// test, all in f64 so far-from-origin worlds cull exactly.
static bool RayMissesNode(const m2RayState* ray, m2AABB aabb)
{
    double p1x = ray->origin.x;
    double p1y = ray->origin.y;
    double p2x = p1x + (double)ray->fraction * (double)ray->translation.x;
    double p2y = p1y + (double)ray->fraction * (double)ray->translation.y;
    double segLoX = p1x < p2x ? p1x : p2x;
    double segHiX = p1x < p2x ? p2x : p1x;
    double segLoY = p1y < p2y ? p1y : p2y;
    double segHiY = p1y < p2y ? p2y : p1y;
    if (segLoX > aabb.upperBound.x || segHiX < aabb.lowerBound.x || segLoY > aabb.upperBound.y ||
        segHiY < aabb.lowerBound.y)
    {
        return true;
    }
    // Separating line through the ray direction's perpendicular.
    double cx = 0.5 * (aabb.lowerBound.x + aabb.upperBound.x);
    double cy = 0.5 * (aabb.lowerBound.y + aabb.upperBound.y);
    double hx = 0.5 * (aabb.upperBound.x - aabb.lowerBound.x);
    double hy = 0.5 * (aabb.upperBound.y - aabb.lowerBound.y);
    double vx = -(p2y - p1y);
    double vy = p2x - p1x;
    double separation = fabs(vx * (p1x - cx) + vy * (p1y - cy)) - (fabs(vx) * hx + fabs(vy) * hy);
    return separation > 0.0;
}

static void RayCastTree(const m2World* world, int32_t treeIndex, m2RayState* ray)
{
    const m2DynamicTree* tree = &world->trees[treeIndex];
    const m2TreeNode* nodes = world->treeNodes[treeIndex];
    int32_t stack[256];
    int32_t top = 0;
    if (tree->root != M2_NULL_NODE)
    {
        stack[top++] = tree->root;
    }
    while (top > 0)
    {
        int32_t index = stack[--top];
        if (RayMissesNode(ray, nodes[index].aabb))
        {
            continue;
        }
        if (nodes[index].height > 0)
        {
            M2_ASSERT(top + 2 <= 256);
            // Push child2 first so child1 is visited first: canonical.
            stack[top++] = nodes[index].child2;
            stack[top++] = nodes[index].child1;
            continue;
        }
        int32_t shapeIndex = nodes[index].userData;
        if (world->shapeAlive[shapeIndex] == 0 || !QueryShouldSee(world, shapeIndex, ray->filter))
        {
            continue;
        }
        m2CastHit hit =
            RayCastShape(world, shapeIndex, ray->origin, ray->translation, ray->fraction);
        if (!hit.hit)
        {
            continue;
        }
        // Canonical winner: strictly closer, or same fraction and a
        // lower shape index.
        if (!ray->hit || hit.fraction < ray->fraction ||
            (hit.fraction == ray->fraction && shapeIndex < ray->shapeIndex))
        {
            ray->hit = true;
            ray->fraction = hit.fraction;
            ray->shapeIndex = shapeIndex;
            ray->normal = hit.normal;
            ray->initialOverlap = hit.normal.x == 0.0f && hit.normal.y == 0.0f;
        }
    }
}

m2RayCastResult m2World_CastRayClosest(m2WorldId worldId, m2Pos2 origin, m2Vec2 translation,
                                       m2QueryFilter filter)
{
    m2RayCastResult result;
    result.shapeId = m2_nullShapeId;
    result.point = origin;
    result.normal = (m2Vec2){0.0f, 0.0f};
    result.fraction = 0.0f;
    result.hit = false;

    m2World* world = m2World_GetInternal(worldId);
    if (world == NULL)
    {
        M2_ASSERT(false);
        return result;
    }

    m2RayState ray;
    ray.origin = origin;
    ray.translation = translation;
    ray.filter = filter;
    ray.fraction = 1.0f;
    ray.shapeIndex = -1;
    ray.normal = (m2Vec2){0.0f, 0.0f};
    ray.hit = false;
    ray.initialOverlap = false;
    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        RayCastTree(world, t, &ray);
    }
    if (!ray.hit)
    {
        return result;
    }

    result.shapeId.index1 = ray.shapeIndex + 1;
    result.shapeId.world0 = worldId.index1;
    result.shapeId.generation = world->shapeGenerations[ray.shapeIndex];
    result.fraction = ray.initialOverlap ? 0.0f : ray.fraction;
    result.point = (m2Pos2){origin.x + (double)result.fraction * (double)translation.x,
                            origin.y + (double)result.fraction * (double)translation.y};
    result.normal = ray.normal;
    result.hit = true;
    return result;
}

static int CompareShapeIndex(const void* a, const void* b)
{
    int32_t ia = *(const int32_t*)a;
    int32_t ib = *(const int32_t*)b;
    return ia < ib ? -1 : (ia > ib ? 1 : 0);
}

int32_t m2World_OverlapAABB(m2WorldId worldId, m2Pos2 lower, m2Pos2 upper, m2ShapeId* results,
                            int32_t capacity, m2QueryFilter filter)
{
    m2World* world = m2World_GetInternal(worldId);
    if (world == NULL || upper.x < lower.x || upper.y < lower.y)
    {
        M2_ASSERT(false);
        return 0;
    }

    m2AABB aabb = {lower, upper};
    int32_t total = 0;
    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        int32_t base = total;
        int32_t hits = m2Tree_Query(&world->trees[t], world->treeNodes[t], aabb,
                                    world->queryScratch + base, world->shapeCapacity - base);
        for (int32_t h = 0; h < hits; ++h)
        {
            // In-place compaction: the write cursor can never pass the
            // read cursor because kept <= scanned.
            int32_t shapeIndex = world->queryScratch[base + h];
            if (world->shapeAlive[shapeIndex] == 0 || !QueryShouldSee(world, shapeIndex, filter))
            {
                continue;
            }
            // Tight filter: the fat tree AABB over-reports.
            int32_t body = world->shapeBody[shapeIndex];
            m2AABB tight =
                m2ComputeShapeAABB(&world->shapeGeometry[shapeIndex], world->transforms[body]);
            if (!m2AABB_Overlaps(tight, aabb))
            {
                continue;
            }
            world->queryScratch[total] = shapeIndex;
            total += 1;
        }
    }

    qsort(world->queryScratch, (size_t)total, sizeof(int32_t), CompareShapeIndex);

    int32_t stored = total < capacity ? total : capacity;
    for (int32_t i = 0; i < stored; ++i)
    {
        int32_t shapeIndex = world->queryScratch[i];
        results[i].index1 = shapeIndex + 1;
        results[i].world0 = worldId.index1;
        results[i].generation = world->shapeGenerations[shapeIndex];
    }
    return total;
}

// ---------------------------------------------------------------
// Convex sweeps and overlaps (parity sprint, slice 63). One generic
// walk serves circle, capsule and polygon: the cast geometry becomes
// a m2DistanceProxy in its own local frame, and per candidate shape
// both proxies meet in the TARGET's body-local frame (the same single
// f64 crossing as rays).

m2DistanceProxy m2GeometryProxy(const m2ShapeGeometry* g)
{
    m2DistanceProxy p;
    p.count = 1;
    p.radius = 0.0f;
    p.points[0] = (m2Vec2){0.0f, 0.0f};
    switch (g->type)
    {
    case m2_circleShape:
        p.points[0] = g->circle.center;
        p.radius = g->circle.radius;
        break;
    case m2_capsuleShape:
        p.points[0] = g->capsule.point1;
        p.points[1] = g->capsule.point2;
        p.count = 2;
        p.radius = g->capsule.radius;
        break;
    case m2_polygonShape:
        for (int32_t i = 0; i < g->polygon.count; ++i)
        {
            p.points[i] = g->polygon.vertices[i];
        }
        p.count = g->polygon.count;
        p.radius = g->polygon.radius;
        break;
    case m2_segmentShape:
        p.points[0] = g->segment.point1;
        p.points[1] = g->segment.point2;
        p.count = 2;
        break;
    default: // chain segment
        p.points[0] = g->chainSegment.segment.point1;
        p.points[1] = g->chainSegment.segment.point2;
        p.count = 2;
        break;
    }
    return p;
}

typedef struct m2ProxyQuery
{
    m2DistanceProxy castLocal; // cast geometry in its own frame
    m2Transform pose;          // world pose of that frame (f64 p)
    m2Vec2 translation;        // world sweep, zero for overlaps
    float boundRadius;         // bounding circle of the cast geometry
} m2ProxyQuery;

static m2ProxyQuery MakeProxyQuery(const m2DistanceProxy* castLocal, m2Transform pose,
                                   m2Vec2 translation)
{
    m2ProxyQuery q;
    q.castLocal = *castLocal;
    q.pose = pose;
    q.translation = translation;
    float ext = 0.0f;
    for (int32_t i = 0; i < castLocal->count; ++i)
    {
        float d2 = castLocal->points[i].x * castLocal->points[i].x +
                   castLocal->points[i].y * castLocal->points[i].y;
        float d = sqrtf(d2);
        ext = d > ext ? d : ext;
    }
    q.boundRadius = ext + castLocal->radius;
    return q;
}

// Both proxies in the target's body frame; also reports the pose
// origin there (the chain one-sided reference point).
static void ProxiesInBodyFrame(const m2World* world, int32_t shapeIndex, const m2ProxyQuery* q,
                               m2DistanceProxy* target, m2DistanceProxy* cast,
                               m2Vec2* translationLocal, m2Vec2* poseOriginLocal)
{
    int32_t body = world->shapeBody[shapeIndex];
    m2Transform xf = world->transforms[body];
    *target = m2GeometryProxy(&world->shapeGeometry[shapeIndex]);

    m2Vec2 rel = {(float)(q->pose.p.x - xf.p.x), (float)(q->pose.p.y - xf.p.y)};
    m2Vec2 relLocal = {xf.q.c * rel.x + xf.q.s * rel.y, -xf.q.s * rel.x + xf.q.c * rel.y};
    *poseOriginLocal = relLocal;

    // Combined rotation: cast local -> world (pose.q), world -> body
    // local (inverse xf.q).
    float rc = xf.q.c * q->pose.q.c + xf.q.s * q->pose.q.s;
    float rs = xf.q.c * q->pose.q.s - xf.q.s * q->pose.q.c;
    cast->count = q->castLocal.count;
    cast->radius = q->castLocal.radius;
    for (int32_t i = 0; i < q->castLocal.count; ++i)
    {
        m2Vec2 pt = q->castLocal.points[i];
        cast->points[i] =
            (m2Vec2){rc * pt.x - rs * pt.y + relLocal.x, rs * pt.x + rc * pt.y + relLocal.y};
    }
    translationLocal->x = xf.q.c * q->translation.x + xf.q.s * q->translation.y;
    translationLocal->y = -xf.q.s * q->translation.x + xf.q.c * q->translation.y;
}

// Sweeps from the ghost side pass through, same sign law as rays.
static bool ChainGhostSide(const m2World* world, int32_t shapeIndex, m2Vec2 startLocal)
{
    const m2ShapeGeometry* g = &world->shapeGeometry[shapeIndex];
    if (g->type != m2_chainSegmentShape)
    {
        return false;
    }
    const m2Segment* seg = &g->chainSegment.segment;
    m2Vec2 e = {seg->point2.x - seg->point1.x, seg->point2.y - seg->point1.y};
    float offset = (startLocal.x - seg->point1.x) * e.y - (startLocal.y - seg->point1.y) * e.x;
    return offset < 0.0f;
}

static m2RayCastResult CastProxyClosest(m2WorldId worldId, const m2DistanceProxy* castLocal,
                                        m2Transform pose, m2Vec2 translation, m2QueryFilter filter)
{
    m2RayCastResult result;
    result.shapeId = m2_nullShapeId;
    result.point = pose.p;
    result.normal = (m2Vec2){0.0f, 0.0f};
    result.fraction = 0.0f;
    result.hit = false;

    m2World* world = m2World_GetInternal(worldId);
    if (world == NULL)
    {
        M2_ASSERT(false);
        return result;
    }
    m2ProxyQuery q = MakeProxyQuery(castLocal, pose, translation);

    // Swept bounding circle in f64: exact culling far from the origin.
    double lox = q.pose.p.x - (double)q.boundRadius;
    double loy = q.pose.p.y - (double)q.boundRadius;
    double hix = q.pose.p.x + (double)q.boundRadius;
    double hiy = q.pose.p.y + (double)q.boundRadius;
    double tx = (double)translation.x;
    double ty = (double)translation.y;
    m2AABB aabb;
    aabb.lowerBound.x = tx < 0.0 ? lox + tx : lox;
    aabb.lowerBound.y = ty < 0.0 ? loy + ty : loy;
    aabb.upperBound.x = tx > 0.0 ? hix + tx : hix;
    aabb.upperBound.y = ty > 0.0 ? hiy + ty : hiy;

    float bestFraction = 1.0f;
    int32_t bestShape = -1;
    m2Vec2 bestNormalLocal = {0.0f, 0.0f};
    m2Vec2 bestPointLocal = {0.0f, 0.0f};
    bool bestOverlap = false;

    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        int32_t hits = m2Tree_Query(&world->trees[t], world->treeNodes[t], aabb,
                                    world->queryScratch, world->shapeCapacity);
        for (int32_t h = 0; h < hits; ++h)
        {
            int32_t shapeIndex = world->queryScratch[h];
            if (world->shapeAlive[shapeIndex] == 0 || !QueryShouldSee(world, shapeIndex, filter))
            {
                continue;
            }
            m2DistanceProxy target;
            m2DistanceProxy cast;
            m2Vec2 translationLocal;
            m2Vec2 startLocal;
            ProxiesInBodyFrame(world, shapeIndex, &q, &target, &cast, &translationLocal,
                               &startLocal);
            if (ChainGhostSide(world, shapeIndex, startLocal))
            {
                continue;
            }
            m2CastResult hit = m2ShapeCastProxy(&target, &cast, translationLocal, bestFraction);
            if (!hit.hit)
            {
                continue;
            }
            if (bestShape < 0 || hit.fraction < bestFraction ||
                (hit.fraction == bestFraction && shapeIndex < bestShape))
            {
                bestShape = shapeIndex;
                bestFraction = hit.fraction;
                bestNormalLocal = hit.normal;
                bestPointLocal = hit.pointA;
                bestOverlap = hit.normal.x == 0.0f && hit.normal.y == 0.0f;
            }
        }
    }
    if (bestShape < 0)
    {
        return result;
    }
    int32_t body = world->shapeBody[bestShape];
    m2Transform xf = world->transforms[body];
    result.shapeId.index1 = bestShape + 1;
    result.shapeId.world0 = worldId.index1;
    result.shapeId.generation = world->shapeGenerations[bestShape];
    result.fraction = bestOverlap ? 0.0f : bestFraction;
    result.hit = true;
    if (bestOverlap)
    {
        result.point = pose.p; // the ray convention: origin on overlap
        return result;
    }
    result.normal = (m2Vec2){xf.q.c * bestNormalLocal.x - xf.q.s * bestNormalLocal.y,
                             xf.q.s * bestNormalLocal.x + xf.q.c * bestNormalLocal.y};
    result.point =
        (m2Pos2){xf.p.x + (double)(xf.q.c * bestPointLocal.x - xf.q.s * bestPointLocal.y),
                 xf.p.y + (double)(xf.q.s * bestPointLocal.x + xf.q.c * bestPointLocal.y)};
    return result;
}

static int32_t OverlapProxy(m2WorldId worldId, const m2DistanceProxy* castLocal, m2Transform pose,
                            m2ShapeId* ids, int32_t capacity, m2QueryFilter filter)
{
    m2World* world = m2World_GetInternal(worldId);
    if (world == NULL)
    {
        M2_ASSERT(false);
        return 0;
    }
    m2ProxyQuery q = MakeProxyQuery(castLocal, pose, (m2Vec2){0.0f, 0.0f});
    m2AABB aabb;
    aabb.lowerBound.x = q.pose.p.x - (double)q.boundRadius;
    aabb.lowerBound.y = q.pose.p.y - (double)q.boundRadius;
    aabb.upperBound.x = q.pose.p.x + (double)q.boundRadius;
    aabb.upperBound.y = q.pose.p.y + (double)q.boundRadius;

    int32_t total = 0;
    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        int32_t base = total;
        int32_t hits = m2Tree_Query(&world->trees[t], world->treeNodes[t], aabb,
                                    world->queryScratch + base, world->shapeCapacity - base);
        for (int32_t h = 0; h < hits; ++h)
        {
            int32_t shapeIndex = world->queryScratch[base + h];
            if (world->shapeAlive[shapeIndex] == 0 || !QueryShouldSee(world, shapeIndex, filter))
            {
                continue;
            }
            m2DistanceProxy target;
            m2DistanceProxy cast;
            m2Vec2 translationLocal;
            m2Vec2 startLocal;
            ProxiesInBodyFrame(world, shapeIndex, &q, &target, &cast, &translationLocal,
                               &startLocal);
            if (ChainGhostSide(world, shapeIndex, startLocal))
            {
                continue;
            }
            m2DistanceResult d = m2ShapeDistance(&target, &cast);
            // Touching within the engine's linear slop skin counts as
            // overlap: the same tolerance the solver's speculative
            // margin uses, and it absorbs GJK witness noise on deeply
            // contained pairs.
            if (d.distance - target.radius - cast.radius > 0.005f)
            {
                continue;
            }
            world->queryScratch[total] = shapeIndex;
            total += 1;
        }
    }
    // Ascending slot order, whatever order the trees reported.
    for (int32_t i = 1; i < total; ++i)
    {
        int32_t key = world->queryScratch[i];
        int32_t j = i - 1;
        while (j >= 0 && world->queryScratch[j] > key)
        {
            world->queryScratch[j + 1] = world->queryScratch[j];
            j -= 1;
        }
        world->queryScratch[j + 1] = key;
    }
    int32_t fill = total < capacity ? total : capacity;
    for (int32_t i = 0; i < fill && ids != NULL; ++i)
    {
        int32_t shapeIndex = world->queryScratch[i];
        ids[i].index1 = shapeIndex + 1;
        ids[i].world0 = worldId.index1;
        ids[i].generation = world->shapeGenerations[shapeIndex];
    }
    return total;
}

m2RayCastResult m2World_CastCircleClosest(m2WorldId worldId, const m2Circle* circle,
                                          m2Transform origin, m2Vec2 translation,
                                          m2QueryFilter filter)
{
    m2DistanceProxy p;
    p.points[0] = circle->center;
    p.count = 1;
    p.radius = circle->radius;
    return CastProxyClosest(worldId, &p, origin, translation, filter);
}

m2RayCastResult m2World_CastCapsuleClosest(m2WorldId worldId, const m2Capsule* capsule,
                                           m2Transform origin, m2Vec2 translation,
                                           m2QueryFilter filter)
{
    m2DistanceProxy p;
    p.points[0] = capsule->point1;
    p.points[1] = capsule->point2;
    p.count = 2;
    p.radius = capsule->radius;
    return CastProxyClosest(worldId, &p, origin, translation, filter);
}

m2RayCastResult m2World_CastPolygonClosest(m2WorldId worldId, const m2Polygon* polygon,
                                           m2Transform origin, m2Vec2 translation,
                                           m2QueryFilter filter)
{
    m2DistanceProxy p;
    for (int32_t i = 0; i < polygon->count; ++i)
    {
        p.points[i] = polygon->vertices[i];
    }
    p.count = polygon->count;
    p.radius = polygon->radius;
    return CastProxyClosest(worldId, &p, origin, translation, filter);
}

int32_t m2World_OverlapCircle(m2WorldId worldId, const m2Circle* circle, m2Transform origin,
                              m2ShapeId* ids, int32_t capacity, m2QueryFilter filter)
{
    m2DistanceProxy p;
    p.points[0] = circle->center;
    p.count = 1;
    p.radius = circle->radius;
    return OverlapProxy(worldId, &p, origin, ids, capacity, filter);
}

int32_t m2World_OverlapCapsule(m2WorldId worldId, const m2Capsule* capsule, m2Transform origin,
                               m2ShapeId* ids, int32_t capacity, m2QueryFilter filter)
{
    m2DistanceProxy p;
    p.points[0] = capsule->point1;
    p.points[1] = capsule->point2;
    p.count = 2;
    p.radius = capsule->radius;
    return OverlapProxy(worldId, &p, origin, ids, capacity, filter);
}

int32_t m2World_OverlapPolygon(m2WorldId worldId, const m2Polygon* polygon, m2Transform origin,
                               m2ShapeId* ids, int32_t capacity, m2QueryFilter filter)
{
    m2DistanceProxy p;
    for (int32_t i = 0; i < polygon->count; ++i)
    {
        p.points[i] = polygon->vertices[i];
    }
    p.count = polygon->count;
    p.radius = polygon->radius;
    return OverlapProxy(worldId, &p, origin, ids, capacity, filter);
}

// One shape, the world ray conventions, the one-sided chain law: the
// The particle projection pass borrows the per-shape kernel (chain
// one-sided law included) without the tree walk.
struct m2CastHitInternal m2RayCastShapeIndex(const m2World* world, int32_t shapeIndex,
                                             m2Pos2 origin, m2Vec2 translation, float maxFraction)
{
    m2CastHit hit = RayCastShape(world, shapeIndex, origin, translation, maxFraction);
    struct m2CastHitInternal out;
    out.point = hit.point;
    out.normal = hit.normal;
    out.fraction = hit.fraction;
    out.hit = hit.hit;
    return out;
}

// same RayCastShape the world walk uses, minus the walk.
m2RayCastResult m2Shape_RayCast(m2ShapeId shapeId, m2Pos2 origin, m2Vec2 translation)
{
    m2RayCastResult result;
    result.shapeId = m2_nullShapeId;
    result.point = origin;
    result.normal = (m2Vec2){0.0f, 0.0f};
    result.fraction = 0.0f;
    result.hit = false;

    m2World* world = m2WorldFromIndex0(shapeId.world0);
    if (world == NULL)
    {
        M2_ASSERT(false);
        return result;
    }
    int32_t index = shapeId.index1 - 1;
    if (index < 0 || index >= world->shapeCapacity || world->shapeAlive[index] == 0 ||
        world->shapeGenerations[index] != shapeId.generation)
    {
        M2_ASSERT(false);
        return result;
    }
    m2CastHit hit = RayCastShape(world, index, origin, translation, 1.0f);
    if (!hit.hit)
    {
        return result;
    }
    result.shapeId = shapeId;
    result.hit = true;
    bool initialOverlap = hit.normal.x == 0.0f && hit.normal.y == 0.0f;
    result.fraction = initialOverlap ? 0.0f : hit.fraction;
    result.normal = hit.normal;
    result.point = (m2Pos2){origin.x + (double)result.fraction * (double)translation.x,
                            origin.y + (double)result.fraction * (double)translation.y};
    return result;
}

// -------------------------------------------------------- all-hits
// Bounded keep-the-closest insertion: candidates stream in, the hits
// array holds the best `capacity` in ascending (fraction, shapeIndex)
// order, and the TRUE total keeps counting past it. No extra memory,
// deterministic regardless of tree visit order.
static int32_t InsertHitSorted(m2RayHit* hits, int32_t kept, int32_t capacity, m2RayHit candidate,
                               int32_t shapeIndex, const int32_t* keptShapes,
                               int32_t* keptShapesOut)
{
    (void)keptShapes;
    int32_t at = kept;
    while (at > 0)
    {
        bool after =
            hits[at - 1].fraction < candidate.fraction ||
            (hits[at - 1].fraction == candidate.fraction && keptShapesOut[at - 1] < shapeIndex);
        if (after)
        {
            break;
        }
        at -= 1;
    }
    if (at >= capacity)
    {
        return kept; // worse than everything kept
    }
    int32_t last = kept < capacity ? kept : capacity - 1;
    for (int32_t i = last; i > at; --i)
    {
        hits[i] = hits[i - 1];
        keptShapesOut[i] = keptShapesOut[i - 1];
    }
    hits[at] = candidate;
    keptShapesOut[at] = shapeIndex;
    return kept < capacity ? kept + 1 : kept;
}

int32_t m2World_CastRayAll(m2WorldId worldId, m2Pos2 origin, m2Vec2 translation, m2RayHit* hits,
                           int32_t capacity, m2QueryFilter filter)
{
    m2World* world = m2World_GetInternal(worldId);
    if (world == NULL)
    {
        M2_ASSERT(false);
        return 0;
    }
    int32_t keptShapes[64];
    int32_t cap = capacity < 64 ? capacity : 64;
    cap = hits != NULL ? cap : 0;
    int32_t kept = 0;
    int32_t total = 0;

    m2RayState ray;
    ray.origin = origin;
    ray.translation = translation;
    ray.filter = filter;
    ray.fraction = 1.0f; // no pruning: every hit counts
    ray.shapeIndex = -1;
    ray.normal = (m2Vec2){0.0f, 0.0f};
    ray.hit = false;
    ray.initialOverlap = false;

    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        const m2DynamicTree* tree = &world->trees[t];
        const m2TreeNode* nodes = world->treeNodes[t];
        int32_t stack[256];
        int32_t top = 0;
        if (tree->root != M2_NULL_NODE)
        {
            stack[top++] = tree->root;
        }
        while (top > 0)
        {
            int32_t index = stack[--top];
            if (RayMissesNode(&ray, nodes[index].aabb))
            {
                continue;
            }
            if (nodes[index].height > 0)
            {
                M2_ASSERT(top + 2 <= 256);
                stack[top++] = nodes[index].child2;
                stack[top++] = nodes[index].child1;
                continue;
            }
            int32_t shapeIndex = nodes[index].userData;
            if (world->shapeAlive[shapeIndex] == 0 || !QueryShouldSee(world, shapeIndex, filter))
            {
                continue;
            }
            m2CastHit hit = RayCastShape(world, shapeIndex, origin, translation, 1.0f);
            if (!hit.hit)
            {
                continue;
            }
            bool initialOverlap = hit.normal.x == 0.0f && hit.normal.y == 0.0f;
            m2RayHit out;
            out.shapeId.index1 = shapeIndex + 1;
            out.shapeId.world0 = worldId.index1;
            out.shapeId.generation = world->shapeGenerations[shapeIndex];
            out.fraction = initialOverlap ? 0.0f : hit.fraction;
            out.normal = hit.normal;
            out.point = (m2Pos2){origin.x + (double)out.fraction * (double)translation.x,
                                 origin.y + (double)out.fraction * (double)translation.y};
            total += 1;
            if (cap > 0)
            {
                kept = InsertHitSorted(hits, kept, cap, out, shapeIndex, NULL, keptShapes);
            }
        }
    }
    return total;
}

static int32_t CastProxyAll(m2WorldId worldId, const m2DistanceProxy* castLocal, m2Transform pose,
                            m2Vec2 translation, m2RayHit* hits, int32_t capacity,
                            m2QueryFilter filter)
{
    m2World* world = m2World_GetInternal(worldId);
    if (world == NULL)
    {
        M2_ASSERT(false);
        return 0;
    }
    m2ProxyQuery q = MakeProxyQuery(castLocal, pose, translation);

    double lox = q.pose.p.x - (double)q.boundRadius;
    double loy = q.pose.p.y - (double)q.boundRadius;
    double hix = q.pose.p.x + (double)q.boundRadius;
    double hiy = q.pose.p.y + (double)q.boundRadius;
    double tx = (double)translation.x;
    double ty = (double)translation.y;
    m2AABB aabb;
    aabb.lowerBound.x = tx < 0.0 ? lox + tx : lox;
    aabb.lowerBound.y = ty < 0.0 ? loy + ty : loy;
    aabb.upperBound.x = tx > 0.0 ? hix + tx : hix;
    aabb.upperBound.y = ty > 0.0 ? hiy + ty : hiy;

    int32_t keptShapes[64];
    int32_t cap = capacity < 64 ? capacity : 64;
    cap = hits != NULL ? cap : 0;
    int32_t kept = 0;
    int32_t total = 0;

    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        int32_t found = m2Tree_Query(&world->trees[t], world->treeNodes[t], aabb,
                                     world->queryScratch, world->shapeCapacity);
        for (int32_t h = 0; h < found; ++h)
        {
            int32_t shapeIndex = world->queryScratch[h];
            if (world->shapeAlive[shapeIndex] == 0 || !QueryShouldSee(world, shapeIndex, filter))
            {
                continue;
            }
            m2DistanceProxy target;
            m2DistanceProxy cast;
            m2Vec2 translationLocal;
            m2Vec2 startLocal;
            ProxiesInBodyFrame(world, shapeIndex, &q, &target, &cast, &translationLocal,
                               &startLocal);
            if (ChainGhostSide(world, shapeIndex, startLocal))
            {
                continue;
            }
            m2CastResult hit = m2ShapeCastProxy(&target, &cast, translationLocal, 1.0f);
            if (!hit.hit)
            {
                continue;
            }
            int32_t body = world->shapeBody[shapeIndex];
            m2Transform xf = world->transforms[body];
            bool initialOverlap = hit.normal.x == 0.0f && hit.normal.y == 0.0f;
            m2RayHit out;
            out.shapeId.index1 = shapeIndex + 1;
            out.shapeId.world0 = worldId.index1;
            out.shapeId.generation = world->shapeGenerations[shapeIndex];
            out.fraction = initialOverlap ? 0.0f : hit.fraction;
            if (initialOverlap)
            {
                out.normal = (m2Vec2){0.0f, 0.0f};
                out.point = pose.p;
            }
            else
            {
                out.normal = (m2Vec2){xf.q.c * hit.normal.x - xf.q.s * hit.normal.y,
                                      xf.q.s * hit.normal.x + xf.q.c * hit.normal.y};
                out.point =
                    (m2Pos2){xf.p.x + (double)(xf.q.c * hit.pointA.x - xf.q.s * hit.pointA.y),
                             xf.p.y + (double)(xf.q.s * hit.pointA.x + xf.q.c * hit.pointA.y)};
            }
            total += 1;
            if (cap > 0)
            {
                kept = InsertHitSorted(hits, kept, cap, out, shapeIndex, NULL, keptShapes);
            }
        }
    }
    return total;
}

int32_t m2World_CastCircleAll(m2WorldId worldId, const m2Circle* circle, m2Transform origin,
                              m2Vec2 translation, m2RayHit* hits, int32_t capacity,
                              m2QueryFilter filter)
{
    m2DistanceProxy p;
    p.points[0] = circle->center;
    p.count = 1;
    p.radius = circle->radius;
    return CastProxyAll(worldId, &p, origin, translation, hits, capacity, filter);
}

int32_t m2World_CastCapsuleAll(m2WorldId worldId, const m2Capsule* capsule, m2Transform origin,
                               m2Vec2 translation, m2RayHit* hits, int32_t capacity,
                               m2QueryFilter filter)
{
    m2DistanceProxy p;
    p.points[0] = capsule->point1;
    p.points[1] = capsule->point2;
    p.count = 2;
    p.radius = capsule->radius;
    return CastProxyAll(worldId, &p, origin, translation, hits, capacity, filter);
}

int32_t m2World_CastPolygonAll(m2WorldId worldId, const m2Polygon* polygon, m2Transform origin,
                               m2Vec2 translation, m2RayHit* hits, int32_t capacity,
                               m2QueryFilter filter)
{
    m2DistanceProxy p;
    for (int32_t i = 0; i < polygon->count; ++i)
    {
        p.points[i] = polygon->vertices[i];
    }
    p.count = polygon->count;
    p.radius = polygon->radius;
    return CastProxyAll(worldId, &p, origin, translation, hits, capacity, filter);
}

// ------------------------------------------------- the mover kit
// Collision planes for a posed capsule mover: one GJK query per
// nearby shape, plane normal from the shape toward the mover,
// separation measured along it (negative = penetration), the
// speculative collar included so a controller sees walls before it
// clips them. Reference architecture (mover.c), Maul frames.
int32_t m2World_CollideMover(m2WorldId worldId, const m2Capsule* mover, m2Transform origin,
                             m2PlaneResult* results, int32_t capacity, m2QueryFilter filter)
{
    m2World* world = m2World_GetInternal(worldId);
    if (world == NULL || mover == NULL)
    {
        M2_ASSERT(false);
        return 0;
    }
    m2DistanceProxy moverLocal;
    memset(&moverLocal, 0, sizeof(moverLocal));
    moverLocal.points[0] = mover->point1;
    moverLocal.points[1] = mover->point2;
    moverLocal.count = 2;
    moverLocal.radius = mover->radius;
    m2ProxyQuery q = MakeProxyQuery(&moverLocal, origin, (m2Vec2){0.0f, 0.0f});

    float collar = 0.02f; // 4x linear slop, the speculative margin
    m2AABB aabb;
    aabb.lowerBound.x = q.pose.p.x - (double)(q.boundRadius + collar);
    aabb.lowerBound.y = q.pose.p.y - (double)(q.boundRadius + collar);
    aabb.upperBound.x = q.pose.p.x + (double)(q.boundRadius + collar);
    aabb.upperBound.y = q.pose.p.y + (double)(q.boundRadius + collar);

    int32_t total = 0;
    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        int32_t base = total; // scratch reuse discipline, per tree
        int32_t found = m2Tree_Query(&world->trees[t], world->treeNodes[t], aabb,
                                     world->queryScratch + base, world->shapeCapacity - base);
        for (int32_t h = 0; h < found; ++h)
        {
            int32_t shapeIndex = world->queryScratch[base + h];
            if (world->shapeAlive[shapeIndex] == 0 || !QueryShouldSee(world, shapeIndex, filter))
            {
                continue;
            }
            m2DistanceProxy target;
            m2DistanceProxy cast;
            m2Vec2 translationLocal;
            m2Vec2 startLocal;
            ProxiesInBodyFrame(world, shapeIndex, &q, &target, &cast, &translationLocal,
                               &startLocal);
            if (ChainGhostSide(world, shapeIndex, startLocal))
            {
                continue;
            }
            m2DistanceResult d = m2ShapeDistance(&target, &cast);
            float separation = d.distance - target.radius - cast.radius;
            if (separation > collar)
            {
                continue;
            }
            // Deep overlap loses the normal; fall back to pushing the
            // mover toward its own pose origin side, or skip if even
            // that is degenerate (dead-centered).
            m2Vec2 normalLocal = d.normal;
            if (d.distance <= 0.0f && normalLocal.x == 0.0f && normalLocal.y == 0.0f)
            {
                float len = sqrtf(startLocal.x * startLocal.x + startLocal.y * startLocal.y);
                if (!(len > 0.0f))
                {
                    continue;
                }
                normalLocal = (m2Vec2){startLocal.x / len, startLocal.y / len};
            }
            int32_t body = world->shapeBody[shapeIndex];
            m2Transform xf = world->transforms[body];
            m2Vec2 surf = {d.pointA.x + target.radius * normalLocal.x,
                           d.pointA.y + target.radius * normalLocal.y};
            if (results != NULL && total < capacity)
            {
                m2PlaneResult* out = results + total;
                out->shapeId.index1 = shapeIndex + 1;
                out->shapeId.world0 = worldId.index1;
                out->shapeId.generation = world->shapeGenerations[shapeIndex];
                out->normal = (m2Vec2){xf.q.c * normalLocal.x - xf.q.s * normalLocal.y,
                                       xf.q.s * normalLocal.x + xf.q.c * normalLocal.y};
                out->separation = separation;
                out->point = (m2Pos2){xf.p.x + (double)(xf.q.c * surf.x - xf.q.s * surf.y),
                                      xf.p.y + (double)(xf.q.s * surf.x + xf.q.c * surf.y)};
            }
            total += 1;
        }
    }
    // Ascending shape order for the filled portion (canonical).
    int32_t filled = results != NULL ? (total < capacity ? total : capacity) : 0;
    for (int32_t i = 1; i < filled; ++i)
    {
        m2PlaneResult key = results[i];
        int32_t j = i - 1;
        while (j >= 0 && results[j].shapeId.index1 > key.shapeId.index1)
        {
            results[j + 1] = results[j];
            j -= 1;
        }
        results[j + 1] = key;
    }
    return total;
}

// The reference plane solver, verbatim algorithm: iterate the planes,
// push the delta out along each normal with a clamped accumulator,
// stop when the total push falls under the slop tolerance.
m2PlaneSolverResult m2SolvePlanes(m2Vec2 targetDelta, m2CollisionPlane* planes, int32_t count)
{
    for (int32_t i = 0; i < count; ++i)
    {
        planes[i].push = 0.0f;
    }
    m2Vec2 delta = targetDelta;
    float tolerance = 0.005f; // linear slop

    int32_t iteration = 0;
    for (; iteration < 20; ++iteration)
    {
        float totalPush = 0.0f;
        for (int32_t i = 0; i < count; ++i)
        {
            m2CollisionPlane* plane = planes + i;
            // Separation of the moved mover from this plane, slopped
            // to prevent jitter.
            float separation =
                plane->separation + delta.x * plane->normal.x + delta.y * plane->normal.y + 0.005f;
            float push = -separation;
            float accumulated = plane->push;
            float next = accumulated + push;
            next = next < 0.0f ? 0.0f : (next > plane->pushLimit ? plane->pushLimit : next);
            plane->push = next;
            push = next - accumulated;
            delta.x += push * plane->normal.x;
            delta.y += push * plane->normal.y;
            totalPush += push < 0.0f ? -push : push;
        }
        if (totalPush < tolerance)
        {
            break;
        }
    }
    m2PlaneSolverResult result = {delta, iteration};
    return result;
}

m2Vec2 m2ClipVector(m2Vec2 vector, const m2CollisionPlane* planes, int32_t count)
{
    m2Vec2 v = vector;
    for (int32_t i = 0; i < count; ++i)
    {
        const m2CollisionPlane* plane = planes + i;
        if (plane->push == 0.0f || plane->clipVelocity == false)
        {
            continue;
        }
        float vn = v.x * plane->normal.x + v.y * plane->normal.y;
        if (vn < 0.0f)
        {
            v.x -= vn * plane->normal.x;
            v.y -= vn * plane->normal.y;
        }
    }
    return v;
}
