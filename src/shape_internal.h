// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Internal shape layout and geometry kernels.

#ifndef MAUL2D_SHAPE_INTERNAL_H
#define MAUL2D_SHAPE_INTERNAL_H

#include "dynamic_tree.h"
#include "maul2d/shape.h"

#include <math.h> // sqrtf: IEEE-exact, allowed
#include <string.h>

// Tagged geometry union. Writers must memset the whole struct first so
// union tail bytes are deterministic in snapshots (topic-01 D2).
typedef struct m2ShapeGeometry
{
    int32_t type; // m2ShapeType
    union
    {
        m2Circle circle;
        m2Capsule capsule;
        m2Polygon polygon;
        m2Segment segment;
    };
} m2ShapeGeometry;

_Static_assert(sizeof(m2Polygon) == 136, "polygon layout is ABI");
_Static_assert(sizeof(m2ShapeGeometry) == 140, "geometry union must be padding-free");

typedef struct m2MassData
{
    float mass;
    m2Vec2 center;           // body-local centroid
    float rotationalInertia; // about the body origin
} m2MassData;

bool m2ValidateCircle(const m2Circle* circle);
bool m2ValidateCapsule(const m2Capsule* capsule);
bool m2ValidateSegment(const m2Segment* segment);
// m2ValidatePolygon is declared here (not public yet: F-T10 pre-freeze
// review decides whether the validator family joins the ABI).
bool m2ValidatePolygon(const m2Polygon* polygon);

m2AABB m2ComputeShapeAABB(const m2ShapeGeometry* geometry, m2Transform xf);
m2MassData m2ComputeShapeMass(const m2ShapeGeometry* geometry, float density);

// --- Contact manifolds (topic-04) -------------------------------------------
// Everything is expressed in body A's local frame (topic-04/RT1-NUM-3):
// the f64 body positions are differenced exactly once to build the
// relative transform, then all contact math runs in f32 near the origin.

typedef struct m2ManifoldPoint
{
    m2Vec2 anchorA;      // contact point in body A's frame
    m2Vec2 anchorB;      // same point in body B's frame
    float separation;    // negative = penetration; positive = speculative
    float normalImpulse; // accumulated - the warm-start payload
    float tangentImpulse;
    uint16_t id;    // feature id, stable across steps for matching
    uint16_t flags; // bit 0: persisted (inherited impulses this step)
} m2ManifoldPoint;

typedef struct m2Manifold
{
    m2ManifoldPoint points[2];
    m2Vec2 normal; // in body A's frame, from A toward B
    int32_t pointCount;
    int32_t padding; // keeps sizeof == sum-of-members
} m2Manifold;

_Static_assert(sizeof(m2ManifoldPoint) == 32, "manifold point must be padding-free");
_Static_assert(sizeof(m2Manifold) == 80, "manifold must be padding-free");

// Relative pose of B in A's frame. Built by the world (owns the single
// f64 crossing); kernels never see world coordinates.
typedef struct m2RelativePose
{
    m2Vec2 p; // B origin in A frame
    m2Rot q;  // B rotation in A frame
} m2RelativePose;

// Speculative distance: manifolds exist slightly before touch
// (topic-07 D1). Constant shared with the fat margin family (F-T2-1).
#define M2_SPECULATIVE_DISTANCE (4.0f * 0.005f)

m2Manifold m2CollideCircles(const m2Circle* a, const m2Circle* b, m2RelativePose pose);
m2Manifold m2CollidePolygonAndCircle(const m2Polygon* a, const m2Circle* b, m2RelativePose pose);
m2Manifold m2CollidePolygons(const m2Polygon* a, const m2Polygon* b, m2RelativePose pose);

// Capsules and segments enter the polygon kernels as 2-vertex rounded
// polygons (the Box2D v3 model): one kernel table, fewer edge cases.
m2Polygon m2MakeSegmentProxy(m2Vec2 p1, m2Vec2 p2, float radius);

#endif // MAUL2D_SHAPE_INTERNAL_H
