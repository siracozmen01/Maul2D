// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Convex distance and translation casts (parity sprint, slice 63).
// Every convex Maul shape is a point set plus a radius, so ONE GJK
// distance kernel and one conservative-advancement cast serve circle,
// capsule, polygon, segment and chain segment alike. The algorithm is
// the reference's (distance.c: b2ShapeDistance / b2ShapeCast), rewritten
// against Maul's frame discipline: callers put both proxies into a
// single body-local float frame first, so the kernel carries no
// transforms at all. Iteration caps are fixed constants and all
// arithmetic is plain IEEE, so results are bit-identical everywhere.

#include "world_internal.h"

#include <math.h>

typedef struct m2SimplexVertex
{
    m2Vec2 wA;      // support on proxy A
    m2Vec2 wB;      // support on proxy B
    m2Vec2 w;       // wB - wA
    float bary;     // barycentric weight
    int32_t indexA; // support indices, for duplicate detection
    int32_t indexB;
} m2SimplexVertex;

static int32_t SupportIndex(const m2DistanceProxy* proxy, m2Vec2 d)
{
    int32_t best = 0;
    float bestDot = proxy->points[0].x * d.x + proxy->points[0].y * d.y;
    for (int32_t i = 1; i < proxy->count; ++i)
    {
        float dot = proxy->points[i].x * d.x + proxy->points[i].y * d.y;
        if (dot > bestDot)
        {
            bestDot = dot;
            best = i;
        }
    }
    return best;
}

// Closest point on a one-vertex simplex is the vertex itself; two and
// three vertices reduce by barycentric regions, reference-style.
static void SolveSimplex2(m2SimplexVertex* v, int32_t* count)
{
    m2Vec2 w1 = v[0].w;
    m2Vec2 w2 = v[1].w;
    m2Vec2 e = {w2.x - w1.x, w2.y - w1.y};
    float d12_2 = -(w1.x * e.x + w1.y * e.y);
    if (d12_2 <= 0.0f)
    {
        v[0].bary = 1.0f;
        *count = 1;
        return;
    }
    float d12_1 = w2.x * e.x + w2.y * e.y;
    if (d12_1 <= 0.0f)
    {
        v[0] = v[1];
        v[0].bary = 1.0f;
        *count = 1;
        return;
    }
    float inv = 1.0f / (d12_1 + d12_2);
    v[0].bary = d12_1 * inv;
    v[1].bary = d12_2 * inv;
    *count = 2;
}

static void SolveSimplex3(m2SimplexVertex* v, int32_t* count)
{
    m2Vec2 w1 = v[0].w;
    m2Vec2 w2 = v[1].w;
    m2Vec2 w3 = v[2].w;

    m2Vec2 e12 = {w2.x - w1.x, w2.y - w1.y};
    float w1e12 = w1.x * e12.x + w1.y * e12.y;
    float w2e12 = w2.x * e12.x + w2.y * e12.y;
    float d12_1 = w2e12;
    float d12_2 = -w1e12;

    m2Vec2 e13 = {w3.x - w1.x, w3.y - w1.y};
    float w1e13 = w1.x * e13.x + w1.y * e13.y;
    float w3e13 = w3.x * e13.x + w3.y * e13.y;
    float d13_1 = w3e13;
    float d13_2 = -w1e13;

    m2Vec2 e23 = {w3.x - w2.x, w3.y - w2.y};
    float w2e23 = w2.x * e23.x + w2.y * e23.y;
    float w3e23 = w3.x * e23.x + w3.y * e23.y;
    float d23_1 = w3e23;
    float d23_2 = -w2e23;

    float n123 = e12.x * e13.y - e12.y * e13.x;
    float d123_1 = n123 * (w2.x * w3.y - w2.y * w3.x);
    float d123_2 = n123 * (w3.x * w1.y - w3.y * w1.x);
    float d123_3 = n123 * (w1.x * w2.y - w1.y * w2.x);

    if (d12_2 <= 0.0f && d13_2 <= 0.0f)
    {
        v[0].bary = 1.0f;
        *count = 1;
        return;
    }
    if (d12_1 > 0.0f && d12_2 > 0.0f && d123_3 <= 0.0f)
    {
        float inv = 1.0f / (d12_1 + d12_2);
        v[0].bary = d12_1 * inv;
        v[1].bary = d12_2 * inv;
        *count = 2;
        return;
    }
    if (d13_1 > 0.0f && d13_2 > 0.0f && d123_2 <= 0.0f)
    {
        float inv = 1.0f / (d13_1 + d13_2);
        v[0].bary = d13_1 * inv;
        v[1] = v[2];
        v[1].bary = d13_2 * inv;
        *count = 2;
        return;
    }
    if (d12_1 <= 0.0f && d23_2 <= 0.0f)
    {
        v[0] = v[1];
        v[0].bary = 1.0f;
        *count = 1;
        return;
    }
    if (d13_1 <= 0.0f && d23_1 <= 0.0f)
    {
        v[0] = v[2];
        v[0].bary = 1.0f;
        *count = 1;
        return;
    }
    if (d23_1 > 0.0f && d23_2 > 0.0f && d123_1 <= 0.0f)
    {
        float inv = 1.0f / (d23_1 + d23_2);
        v[0] = v[1];
        v[1] = v[2];
        v[0].bary = d23_1 * inv;
        v[1].bary = d23_2 * inv;
        *count = 2;
        return;
    }
    // Origin inside the triangle: overlap.
    float inv = 1.0f / (d123_1 + d123_2 + d123_3);
    v[0].bary = d123_1 * inv;
    v[1].bary = d123_2 * inv;
    v[2].bary = d123_3 * inv;
    *count = 3;
}

static m2Vec2 SimplexClosest(const m2SimplexVertex* v, int32_t count)
{
    if (count == 1)
    {
        return v[0].w;
    }
    if (count == 2)
    {
        return (m2Vec2){v[0].bary * v[0].w.x + v[1].bary * v[1].w.x,
                        v[0].bary * v[0].w.y + v[1].bary * v[1].w.y};
    }
    return (m2Vec2){0.0f, 0.0f};
}

static void SimplexWitness(const m2SimplexVertex* v, int32_t count, m2Vec2* pA, m2Vec2* pB)
{
    if (count == 1)
    {
        *pA = v[0].wA;
        *pB = v[0].wB;
        return;
    }
    if (count == 2)
    {
        pA->x = v[0].bary * v[0].wA.x + v[1].bary * v[1].wA.x;
        pA->y = v[0].bary * v[0].wA.y + v[1].bary * v[1].wA.y;
        pB->x = v[0].bary * v[0].wB.x + v[1].bary * v[1].wB.x;
        pB->y = v[0].bary * v[0].wB.y + v[1].bary * v[1].wB.y;
        return;
    }
    // Overlapping: both witnesses collapse to the same blended point.
    pA->x = v[0].bary * v[0].wA.x + v[1].bary * v[1].wA.x + v[2].bary * v[2].wA.x;
    pA->y = v[0].bary * v[0].wA.y + v[1].bary * v[1].wA.y + v[2].bary * v[2].wA.y;
    *pB = *pA;
}

// Core-shape distance (radii NOT applied) with witness points; the
// callers subtract radii. distance == 0 means the cores overlap and
// the normal is (0,0).
m2DistanceResult m2ShapeDistance(const m2DistanceProxy* proxyA, const m2DistanceProxy* proxyB)
{
    m2SimplexVertex v[3];
    int32_t count = 1;
    v[0].indexA = 0;
    v[0].indexB = 0;
    v[0].wA = proxyA->points[0];
    v[0].wB = proxyB->points[0];
    v[0].w = (m2Vec2){v[0].wB.x - v[0].wA.x, v[0].wB.y - v[0].wA.y};
    v[0].bary = 1.0f;

    for (int32_t iter = 0; iter < 20; ++iter)
    {
        // Reference termination: remember the pre-solve simplex; a new
        // support that was JUST dropped means a degenerate cycle, and
        // bouncing between the same vertices means convergence. Both
        // exit with the current (correct) closest point.
        int32_t saveCount = count;
        int32_t saveA[3];
        int32_t saveB[3];
        for (int32_t i = 0; i < count; ++i)
        {
            saveA[i] = v[i].indexA;
            saveB[i] = v[i].indexB;
        }
        if (count == 2)
        {
            SolveSimplex2(v, &count);
        }
        else if (count == 3)
        {
            SolveSimplex3(v, &count);
        }
        if (count == 3)
        {
            break; // origin enclosed: overlap
        }
        m2Vec2 closest = SimplexClosest(v, count);
        float dist2 = closest.x * closest.x + closest.y * closest.y;
        if (dist2 == 0.0f)
        {
            break; // exactly touching cores
        }
        m2Vec2 d = {-closest.x, -closest.y};
        int32_t ia = SupportIndex(proxyA, (m2Vec2){-d.x, -d.y});
        int32_t ib = SupportIndex(proxyB, d);
        bool seen = false;
        for (int32_t i = 0; i < saveCount; ++i)
        {
            if (saveA[i] == ia && saveB[i] == ib)
            {
                seen = true;
                break;
            }
        }
        if (seen)
        {
            break; // no progress possible: converged
        }
        m2SimplexVertex* nv = &v[count];
        nv->indexA = ia;
        nv->indexB = ib;
        nv->wA = proxyA->points[ia];
        nv->wB = proxyB->points[ib];
        nv->w = (m2Vec2){nv->wB.x - nv->wA.x, nv->wB.y - nv->wA.y};
        nv->bary = 0.0f;
        count += 1;
    }

    m2DistanceResult result;
    if (count == 3)
    {
        SimplexWitness(v, count, &result.pointA, &result.pointB);
        result.distance = 0.0f;
        result.normal = (m2Vec2){0.0f, 0.0f};
        return result;
    }
    SimplexWitness(v, count, &result.pointA, &result.pointB);
    m2Vec2 gap = {result.pointB.x - result.pointA.x, result.pointB.y - result.pointA.y};
    float dist = sqrtf(gap.x * gap.x + gap.y * gap.y);
    result.distance = dist;
    result.normal = dist > 0.0f ? (m2Vec2){gap.x / dist, gap.y / dist} : (m2Vec2){0.0f, 0.0f};
    return result;
}

// Conservative advancement, translation only: proxy B slides along
// `translation` toward the static proxy A. Reference algorithm with a
// fixed 20-iteration cap. Initial overlap reports hit at fraction 0
// with a (0,0) normal, matching the ray convention.
m2CastResult m2ShapeCastProxy(const m2DistanceProxy* proxyA, const m2DistanceProxy* proxyB,
                              m2Vec2 translation, float maxFraction)
{
    m2CastResult out;
    out.fraction = 0.0f;
    out.normal = (m2Vec2){0.0f, 0.0f};
    out.pointA = (m2Vec2){0.0f, 0.0f};
    out.hit = false;

    float slop = 0.005f; // M2_LINEAR_SLOP (shape.c pins the same value)
    float totalRadius = proxyA->radius + proxyB->radius;
    float target = totalRadius - slop > slop ? totalRadius - slop : slop;
    float tolerance = 0.25f * slop;

    m2DistanceProxy moved = *proxyB;
    float fraction = 0.0f;

    for (int32_t iter = 0; iter < 20; ++iter)
    {
        m2DistanceResult d = m2ShapeDistance(proxyA, &moved);
        if (d.distance < target + tolerance)
        {
            if (iter == 0)
            {
                out.hit = true; // initial overlap: fraction 0, no normal
                out.pointA = d.pointA;
                return out;
            }
            out.hit = true;
            out.fraction = fraction;
            // Reference convention: the normal points from A toward B,
            // i.e. it faces the incoming shape, exactly like a ray's
            // surface normal faces the ray origin.
            out.normal = d.normal;
            out.pointA = (m2Vec2){d.pointA.x + proxyA->radius * d.normal.x,
                                  d.pointA.y + proxyA->radius * d.normal.y};
            return out;
        }
        // d.normal points from A to B (pointB - pointA); approaching
        // means the translation runs against it.
        float denom = -(translation.x * d.normal.x + translation.y * d.normal.y);
        if (denom <= 0.0f)
        {
            return out; // separating or parallel: no hit ever
        }
        float advance = (d.distance - target) / denom;
        fraction += advance;
        if (fraction >= maxFraction)
        {
            return out; // out of range
        }
        for (int32_t i = 0; i < moved.count; ++i)
        {
            moved.points[i].x = proxyB->points[i].x + fraction * translation.x;
            moved.points[i].y = proxyB->points[i].y + fraction * translation.y;
        }
    }
    return out;
}
