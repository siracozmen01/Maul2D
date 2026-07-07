// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Debug drawing: a read-only walk over the world that hands your
// renderer everything it needs. Colors encode state - static gray,
// awake tan, sleeping dim, sensors green, bullets orange - and every
// world position leaves here as f64 so the renderer can subtract the
// camera before dropping to floats.

#include "world_internal.h"

#include "maul2d/base.h"

enum
{
    m2_colorStatic = 0x7f7f7f,
    m2_colorKinematic = 0x5f9fd8,
    m2_colorAwake = 0xd8b25f,
    m2_colorSleeping = 0x6f6a55,
    m2_colorSensor = 0x5fd88a,
    m2_colorBullet = 0xd8755f,
    m2_colorJoint = 0x9f5fd8,
    m2_colorContact = 0xd85f5f,
    m2_colorFriction = 0x5fd8d8,
    m2_colorAABB = 0x3f4f3f,
};

static uint32_t BodyColor(const m2World* world, int32_t body)
{
    if (world->types[body] == (uint8_t)m2_staticBody)
    {
        return m2_colorStatic;
    }
    if (world->types[body] == (uint8_t)m2_kinematicBody)
    {
        return m2_colorKinematic;
    }
    if (world->asleep[body] != 0)
    {
        return m2_colorSleeping;
    }
    return world->bullets[body] != 0 ? m2_colorBullet : m2_colorAwake;
}

static m2Pos2 LocalToWorld(m2Transform xf, m2Vec2 local)
{
    m2Pos2 p;
    p.x = xf.p.x + (double)(xf.q.c * local.x - xf.q.s * local.y);
    p.y = xf.p.y + (double)(xf.q.s * local.x + xf.q.c * local.y);
    return p;
}

static void DrawShape(const m2World* world, const m2DebugDraw* draw, int32_t shape)
{
    int32_t body = world->shapeBody[shape];
    m2Transform xf = world->transforms[body];
    uint32_t color = world->shapeSensor[shape] != 0 ? m2_colorSensor : BodyColor(world, body);
    const m2ShapeGeometry* geometry = &world->shapeGeometry[shape];

    switch (geometry->type)
    {
    case m2_circleShape:
        if (draw->drawCircle != NULL)
        {
            draw->drawCircle(LocalToWorld(xf, geometry->circle.center), geometry->circle.radius,
                             xf.q, color, draw->context);
        }
        break;
    case m2_capsuleShape:
        if (draw->drawCapsule != NULL)
        {
            draw->drawCapsule(LocalToWorld(xf, geometry->capsule.point1),
                              LocalToWorld(xf, geometry->capsule.point2), geometry->capsule.radius,
                              color, draw->context);
        }
        break;
    case m2_segmentShape:
        if (draw->drawSegment != NULL)
        {
            draw->drawSegment(LocalToWorld(xf, geometry->segment.point1),
                              LocalToWorld(xf, geometry->segment.point2), color, draw->context);
        }
        break;
    case m2_chainSegmentShape:
        if (draw->drawSegment != NULL)
        {
            draw->drawSegment(LocalToWorld(xf, geometry->chainSegment.segment.point1),
                              LocalToWorld(xf, geometry->chainSegment.segment.point2), color,
                              draw->context);
        }
        break;
    default:
        if (draw->drawPolygon != NULL)
        {
            draw->drawPolygon(geometry->polygon.vertices, geometry->polygon.count, xf.p, xf.q,
                              color, draw->context);
        }
        break;
    }
}

void m2World_Draw(m2WorldId worldId, const m2DebugDraw* draw)
{
    m2World* world = m2World_GetInternal(worldId);
    if (world == NULL || draw == NULL)
    {
        return;
    }

    if (draw->drawShapes)
    {
        for (int32_t i = 0; i < world->maxShapeIndex; ++i)
        {
            if (world->shapeAlive[i] != 0)
            {
                DrawShape(world, draw, i);
            }
        }
    }

    if (draw->drawJoints && draw->drawSegment != NULL)
    {
        for (int32_t j = 0; j < world->maxJointIndex; ++j)
        {
            if (world->jointAlive[j] == 0)
            {
                continue;
            }
            uint8_t type = world->jointType[j];
            if (type == 5)
            {
                continue; // a filter joint is the absence of contact: nothing to draw
            }
            if (type == 8 || type == 10)
            {
                // Gear and ratchet: the anchor slots carry phase-tracking
                // rotation state, not anchors; draw the coupling hub to hub.
                m2Pos2 ca = world->transforms[world->jointBodyA[j]].p;
                m2Pos2 cb = world->transforms[world->jointBodyB[j]].p;
                draw->drawSegment(ca, cb, m2_colorJoint, draw->context);
                continue;
            }
            m2Pos2 a =
                LocalToWorld(world->transforms[world->jointBodyA[j]], world->jointLocalAnchorA[j]);
            m2Pos2 b =
                LocalToWorld(world->transforms[world->jointBodyB[j]], world->jointLocalAnchorB[j]);
            if (type == 9)
            {
                // Pulley: two ropes up to the ground anchors and the
                // crossbar between them, the machine as you drew it.
                m2Pos2 ga = world->jointTargets[j];
                m2Pos2 gb = world->jointTargetsB[j];
                draw->drawSegment(a, ga, m2_colorJoint, draw->context);
                draw->drawSegment(b, gb, m2_colorJoint, draw->context);
                draw->drawSegment(ga, gb, m2_colorJoint, draw->context);
                if (draw->drawPoint != NULL)
                {
                    draw->drawPoint(ga, 4.0f, m2_colorJoint, draw->context);
                    draw->drawPoint(gb, 4.0f, m2_colorJoint, draw->context);
                }
                continue;
            }
            if (type == 7)
            {
                // Mouse: the spring runs from the world target to the
                // grab point on B.
                a = world->jointTargets[j];
            }
            draw->drawSegment(a, b, m2_colorJoint, draw->context);
            if (draw->drawPoint != NULL)
            {
                draw->drawPoint(a, 4.0f, m2_colorJoint, draw->context);
                draw->drawPoint(b, 4.0f, m2_colorJoint, draw->context);
            }
        }
    }

    if (draw->drawContacts && draw->drawPoint != NULL)
    {
        for (int32_t i = 0; i < world->pairCount; ++i)
        {
            if (world->pairTouching[i] == 0)
            {
                continue;
            }
            int32_t shapeA = (int32_t)(world->pairKeys[i] >> 32);
            int32_t shapeB = (int32_t)(world->pairKeys[i] & 0xFFFFFFFFu);
            if (world->shapeSensor[shapeA] != 0 || world->shapeSensor[shapeB] != 0)
            {
                continue;
            }
            const m2Manifold* manifold = &world->manifolds[i];
            m2Transform xfA = world->transforms[world->shapeBody[shapeA]];
            for (int32_t k = 0; k < manifold->pointCount; ++k)
            {
                draw->drawPoint(LocalToWorld(xfA, manifold->points[k].anchorA), 5.0f,
                                m2_colorContact, draw->context);
            }
        }
    }

    if (draw->drawContactForces && draw->drawSegment != NULL)
    {
        // An arrow per contact point: the normal impulse along the world
        // normal, the friction impulse along the tangent. The stored
        // impulses are the warm-start payload the last solve settled on.
        float scale = draw->forceScale > 0.0f ? draw->forceScale : 1.0f;
        for (int32_t i = 0; i < world->pairCount; ++i)
        {
            if (world->pairTouching[i] == 0)
            {
                continue;
            }
            int32_t shapeA = (int32_t)(world->pairKeys[i] >> 32);
            int32_t shapeB = (int32_t)(world->pairKeys[i] & 0xFFFFFFFFu);
            if (world->shapeSensor[shapeA] != 0 || world->shapeSensor[shapeB] != 0)
            {
                continue;
            }
            const m2Manifold* manifold = &world->manifolds[i];
            m2Transform xfA = world->transforms[world->shapeBody[shapeA]];
            m2Vec2 n = {xfA.q.c * manifold->normal.x - xfA.q.s * manifold->normal.y,
                        xfA.q.s * manifold->normal.x + xfA.q.c * manifold->normal.y};
            m2Vec2 t = {-n.y, n.x};
            for (int32_t k = 0; k < manifold->pointCount; ++k)
            {
                m2Pos2 p = LocalToWorld(xfA, manifold->points[k].anchorA);
                float ni = manifold->points[k].normalImpulse * scale;
                m2Pos2 nEnd = {p.x + (double)(n.x * ni), p.y + (double)(n.y * ni)};
                draw->drawSegment(p, nEnd, m2_colorContact, draw->context);
                float ti = manifold->points[k].tangentImpulse * scale;
                m2Pos2 tEnd = {p.x + (double)(t.x * ti), p.y + (double)(t.y * ti)};
                draw->drawSegment(p, tEnd, m2_colorFriction, draw->context);
            }
        }
    }

    if (draw->drawAABBs && draw->drawSegment != NULL)
    {
        for (int32_t i = 0; i < world->maxShapeIndex; ++i)
        {
            if (world->shapeAlive[i] == 0 || world->proxyIds[i] == M2_NULL_NODE)
            {
                continue;
            }
            const m2TreeNode* node =
                &world->treeNodes[world->types[world->shapeBody[i]]][world->proxyIds[i]];
            m2AABB box = node->aabb;
            m2Pos2 c1 = {box.lowerBound.x, box.lowerBound.y};
            m2Pos2 c2 = {box.upperBound.x, box.lowerBound.y};
            m2Pos2 c3 = {box.upperBound.x, box.upperBound.y};
            m2Pos2 c4 = {box.lowerBound.x, box.upperBound.y};
            draw->drawSegment(c1, c2, m2_colorAABB, draw->context);
            draw->drawSegment(c2, c3, m2_colorAABB, draw->context);
            draw->drawSegment(c3, c4, m2_colorAABB, draw->context);
            draw->drawSegment(c4, c1, m2_colorAABB, draw->context);
        }
    }
}
