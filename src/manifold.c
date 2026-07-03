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
