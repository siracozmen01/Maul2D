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

#endif // MAUL2D_SHAPE_INTERNAL_H
