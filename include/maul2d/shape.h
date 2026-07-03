// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#ifndef MAUL2D_SHAPE_H
#define MAUL2D_SHAPE_H

#include "maul2d/body.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define M2_MAX_POLYGON_VERTICES 8

    typedef struct m2ShapeId
    {
        int32_t index1; // 1-based, 0 = null
        uint16_t world0;
        uint16_t generation;
    } m2ShapeId;

    /// All geometry is authored in body-local space, f32 (topic-01 D1).
    typedef struct m2Circle
    {
        m2Vec2 center;
        float radius;
    } m2Circle;

    typedef struct m2Capsule
    {
        m2Vec2 point1;
        m2Vec2 point2;
        float radius;
    } m2Capsule;

    /// Convex, counter-clockwise, at most 8 vertices. radius > 0 makes a
    /// rounded polygon (first-class, topic-03 D2). Build via m2MakePolygon
    /// or m2MakeBox so normals and validity are computed for you.
    typedef struct m2Polygon
    {
        m2Vec2 vertices[M2_MAX_POLYGON_VERTICES];
        m2Vec2 normals[M2_MAX_POLYGON_VERTICES];
        int32_t count;
        float radius;
    } m2Polygon;

    typedef struct m2Segment
    {
        m2Vec2 point1;
        m2Vec2 point2;
    } m2Segment;

    typedef enum m2ShapeType
    {
        m2_circleShape = 0,
        m2_capsuleShape = 1,
        m2_polygonShape = 2,
        m2_segmentShape = 3,
    } m2ShapeType;

    typedef struct m2ShapeDef
    {
        float density;     // kg/m^2; dynamic bodies get a minimum-mass floor
        float friction;    // Coulomb; pairs mix by geometric mean
        float restitution; // bounce in [0,1]; pairs mix by maximum
        uint64_t userData;
        int32_t internalValue;
    } m2ShapeDef;

    m2ShapeDef m2DefaultShapeDef(void);

    /// Validated constructors (topic-03 D4: relative thresholds, reject
    /// loudly). A returned polygon with count == 0 is invalid input.
    m2Polygon m2MakePolygon(const m2Vec2* points, int32_t count, float radius);
    m2Polygon m2MakeBox(float halfWidth, float halfHeight);

    /// Attach a shape to a body. Validation failure or exhausted capacity
    /// returns the null id; nothing is half-constructed. Dynamic bodies
    /// recompute mass from all attached shapes (explicit override comes
    /// with the solver slice). Thread class: writer.
    m2ShapeId m2CreateCircleShape(m2BodyId bodyId, const m2ShapeDef* def, const m2Circle* circle);
    m2ShapeId m2CreateCapsuleShape(m2BodyId bodyId, const m2ShapeDef* def,
                                   const m2Capsule* capsule);
    m2ShapeId m2CreatePolygonShape(m2BodyId bodyId, const m2ShapeDef* def,
                                   const m2Polygon* polygon);
    m2ShapeId m2CreateSegmentShape(m2BodyId bodyId, const m2ShapeDef* def,
                                   const m2Segment* segment);

    bool m2Shape_IsValid(m2ShapeId shapeId);
    m2BodyId m2Shape_GetBody(m2ShapeId shapeId);
    uint64_t m2Shape_GetUserData(m2ShapeId shapeId);

    /// Body mass derived from attached shape densities (0 for non-dynamic).
    float m2Body_GetMass(m2BodyId bodyId);

    static const m2ShapeId m2_nullShapeId = {0, 0, 0};

    /// Queries are read-only: they never touch simulation state, and
    /// their results are canonical (closest hit with lowest-shape-index
    /// tie break; overlap lists in ascending creation order).
    typedef struct m2RayCastResult
    {
        m2ShapeId shapeId; // null when hit is false
        m2Pos2 point;      // world hit point (ray origin on initial overlap)
        m2Vec2 normal;     // world surface normal ((0,0) on initial overlap)
        float fraction;    // hit = origin + fraction * translation
        bool hit;
    } m2RayCastResult;

    /// Closest hit along origin + t * translation, t in [0, 1].
    /// Thread class: reader.
    m2RayCastResult m2World_CastRayClosest(m2WorldId worldId, m2Pos2 origin, m2Vec2 translation);

    /// Fills results with up to capacity alive shapes whose tight AABB
    /// overlaps [lower, upper], ascending shape order. Returns the total
    /// number of overlapping shapes even when it exceeds capacity.
    /// Thread class: reader.
    int32_t m2World_OverlapAABB(m2WorldId worldId, m2Pos2 lower, m2Pos2 upper, m2ShapeId* results,
                                int32_t capacity);

#ifdef __cplusplus
}
#endif

#endif // MAUL2D_SHAPE_H
