// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Soft Step contact solver, scalar path. Adapted from Box2D v3's contact
// solver (Copyright 2023 Erin Catto, MIT), reconciled against topic-05:
// prepare -> per substep [integrate velocities, warm start, solve(bias),
// integrate positions, relax(no bias)] -> restitution once -> store.
// Friction resolves in the relax pass, exactly like the reference.
// Separations are re-evaluated inside substeps from accumulated f32
// deltas (never fresh world-space math): the RT1-NUM-3 discipline.
//
// v1 simplification, recorded: the body origin stands in for the center
// of mass (inertia is about the true centroid, anchors are about the
// origin). Centered shapes are exact; offset-COM rigs come with the
// proper COM state in a later slice.

#include "world_internal.h"

#include "maul2d/base.h"

#include <string.h>

#define M2_CONTACT_HERTZ          30.0f
#define M2_CONTACT_DAMPING_RATIO  10.0f
#define M2_CONTACT_PUSH_MAX_SPEED 3.0f
#define M2_RESTITUTION_THRESHOLD  1.0f

typedef struct m2Softness
{
    float biasRate;
    float massScale;
    float impulseScale;
} m2Softness;

// Reference formula (b2MakeSoft): bias = w/(2z+hw),
// massScale = hw(2z+hw)/(1+hw(2z+hw)), impulseScale = 1/(1+hw(2z+hw)).
static m2Softness MakeSoft(float hertz, float zeta, float h)
{
    if (hertz == 0.0f)
    {
        return (m2Softness){0.0f, 0.0f, 0.0f};
    }
    float omega = 2.0f * M2_PI * hertz;
    float a1 = 2.0f * zeta + h * omega;
    float a2 = h * omega * a1;
    float a3 = 1.0f / (1.0f + a2);
    return (m2Softness){omega / a1, a2 * a3, a3};
}

static m2Vec2 Rotate(m2Rot q, m2Vec2 v)
{
    return (m2Vec2){q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y};
}

static float Cross(m2Vec2 a, m2Vec2 b)
{
    return a.x * b.y - a.y * b.x;
}

typedef struct m2ConstraintPoint
{
    m2Vec2 rA; // anchor relative to body origin, world-rotated at prepare
    m2Vec2 rB;
    float baseSeparation;
    float relativeVelocity; // normal speed at prepare (restitution input)
    float normalMass;
    float tangentMass;
    float normalImpulse;
    float tangentImpulse;
    uint16_t id;
    uint16_t persisted;
} m2ConstraintPoint;

typedef struct m2ContactConstraint
{
    int32_t pairIndex;
    int32_t bodyA;
    int32_t bodyB;
    m2Vec2 normal; // world frame
    float friction;
    float restitution;
    m2Softness softness;
    int32_t pointCount;
    m2ConstraintPoint points[2];
} m2ContactConstraint;

// The whole solver scratch lives on world arrays sized at creation; the
// constraint list is rebuilt every step (step-transient, snapshot-benign).

int32_t m2ContactConstraintSize(void)
{
    return (int32_t)sizeof(m2ContactConstraint);
}

static int32_t PrepareContacts(m2World* world, m2ContactConstraint* constraints, float h)
{
    m2Softness soft = MakeSoft(M2_CONTACT_HERTZ, M2_CONTACT_DAMPING_RATIO, h);
    int32_t count = 0;
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        m2Manifold* manifold = &world->manifolds[i];
        if (manifold->pointCount == 0)
        {
            continue;
        }
        int32_t shapeA = (int32_t)(world->pairKeys[i] >> 32);
        int32_t shapeB = (int32_t)(world->pairKeys[i] & 0xFFFFFFFFu);
        int32_t bodyA = world->shapeBody[shapeA];
        int32_t bodyB = world->shapeBody[shapeB];
        float mA = world->invMass[bodyA];
        float iA = world->invInertia[bodyA];
        float mB = world->invMass[bodyB];
        float iB = world->invInertia[bodyB];
        if (mA + mB == 0.0f && iA + iB == 0.0f)
        {
            continue; // both non-dynamic
        }
        bool asleepA = world->types[bodyA] != (uint8_t)m2_dynamicBody || world->asleep[bodyA] != 0;
        bool asleepB = world->types[bodyB] != (uint8_t)m2_dynamicBody || world->asleep[bodyB] != 0;
        if (asleepA && asleepB)
        {
            continue; // frozen contact inside a sleeping island
        }

        m2ContactConstraint* c = constraints + count;
        count += 1;
        c->pairIndex = i;
        c->bodyA = bodyA;
        c->bodyB = bodyB;
        // Geometric-mean friction, max restitution (reference mixing).
        c->friction = sqrtf(world->shapeFriction[shapeA] * world->shapeFriction[shapeB]);
        float restA = world->shapeRestitution[shapeA];
        float restB = world->shapeRestitution[shapeB];
        c->restitution = m2MaxF(restA, restB);
        c->softness = soft;
        c->pointCount = manifold->pointCount;

        m2Rot qA = world->transforms[bodyA].q;
        m2Rot qB = world->transforms[bodyB].q;
        c->normal = Rotate(qA, manifold->normal);
        m2Vec2 tangent = {-c->normal.y, c->normal.x};

        m2Vec2 vA = world->linearVelocities[bodyA];
        float wA = world->angularVelocities[bodyA];
        m2Vec2 vB = world->linearVelocities[bodyB];
        float wB = world->angularVelocities[bodyB];

        for (int32_t k = 0; k < manifold->pointCount; ++k)
        {
            m2ManifoldPoint* mp = &manifold->points[k];
            m2ConstraintPoint* cp = &c->points[k];
            cp->rA = Rotate(qA, mp->anchorA);
            cp->rB = Rotate(qB, mp->anchorB);
            cp->baseSeparation = mp->separation;
            cp->normalImpulse = mp->normalImpulse;
            cp->tangentImpulse = mp->tangentImpulse;
            cp->id = mp->id;
            cp->persisted = (uint16_t)(mp->flags & 1);

            float kNormal = mA + mB + iA * Cross(cp->rA, c->normal) * Cross(cp->rA, c->normal) +
                            iB * Cross(cp->rB, c->normal) * Cross(cp->rB, c->normal);
            cp->normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f; // row-skip law
            float kTangent = mA + mB + iA * Cross(cp->rA, tangent) * Cross(cp->rA, tangent) +
                             iB * Cross(cp->rB, tangent) * Cross(cp->rB, tangent);
            cp->tangentMass = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

            m2Vec2 vrA = {vA.x - wA * cp->rA.y, vA.y + wA * cp->rA.x};
            m2Vec2 vrB = {vB.x - wB * cp->rB.y, vB.y + wB * cp->rB.x};
            cp->relativeVelocity = (vrB.x - vrA.x) * c->normal.x + (vrB.y - vrA.y) * c->normal.y;
        }
    }
    return count;
}

static void WarmStart(m2World* world, m2ContactConstraint* constraints, int32_t count)
{
    for (int32_t i = 0; i < count; ++i)
    {
        m2ContactConstraint* c = constraints + i;
        float mA = world->invMass[c->bodyA];
        float iA = world->invInertia[c->bodyA];
        float mB = world->invMass[c->bodyB];
        float iB = world->invInertia[c->bodyB];
        m2Vec2 tangent = {-c->normal.y, c->normal.x};
        for (int32_t k = 0; k < c->pointCount; ++k)
        {
            m2ConstraintPoint* cp = &c->points[k];
            m2Vec2 P = {cp->normalImpulse * c->normal.x + cp->tangentImpulse * tangent.x,
                        cp->normalImpulse * c->normal.y + cp->tangentImpulse * tangent.y};
            world->linearVelocities[c->bodyA].x -= mA * P.x;
            world->linearVelocities[c->bodyA].y -= mA * P.y;
            world->angularVelocities[c->bodyA] -= iA * Cross(cp->rA, P);
            world->linearVelocities[c->bodyB].x += mB * P.x;
            world->linearVelocities[c->bodyB].y += mB * P.y;
            world->angularVelocities[c->bodyB] += iB * Cross(cp->rB, P);
        }
    }
}

static void SolveContacts(m2World* world, m2ContactConstraint* constraints, int32_t count,
                          float invH, bool useBias)
{
    for (int32_t i = 0; i < count; ++i)
    {
        m2ContactConstraint* c = constraints + i;
        float mA = world->invMass[c->bodyA];
        float iA = world->invInertia[c->bodyA];
        float mB = world->invMass[c->bodyB];
        float iB = world->invInertia[c->bodyB];
        m2Vec2 vA = world->linearVelocities[c->bodyA];
        float wA = world->angularVelocities[c->bodyA];
        m2Vec2 vB = world->linearVelocities[c->bodyB];
        float wB = world->angularVelocities[c->bodyB];
        m2Vec2 normal = c->normal;
        m2Vec2 tangent = {-normal.y, normal.x};

        // Separation drift from accumulated deltas (never fresh
        // world-space math inside the step).
        m2Vec2 dp = {world->deltaPositions[c->bodyB].x - world->deltaPositions[c->bodyA].x,
                     world->deltaPositions[c->bodyB].y - world->deltaPositions[c->bodyA].y};

        for (int32_t k = 0; k < c->pointCount; ++k)
        {
            m2ConstraintPoint* cp = &c->points[k];
            m2Vec2 drB = Rotate(world->deltaRotations[c->bodyB], cp->rB);
            m2Vec2 drA = Rotate(world->deltaRotations[c->bodyA], cp->rA);
            m2Vec2 ds = {dp.x + drB.x - cp->rB.x - (drA.x - cp->rA.x),
                         dp.y + drB.y - cp->rB.y - (drA.y - cp->rA.y)};
            float s = cp->baseSeparation + ds.x * normal.x + ds.y * normal.y;

            float velocityBias = 0.0f;
            float massScale = 1.0f;
            float impulseScale = 0.0f;
            if (s > 0.0f)
            {
                velocityBias = s * invH; // speculative: prevent crossing
            }
            else if (useBias)
            {
                velocityBias = m2MaxF(c->softness.massScale * c->softness.biasRate * s,
                                      -M2_CONTACT_PUSH_MAX_SPEED);
                massScale = c->softness.massScale;
                impulseScale = c->softness.impulseScale;
            }

            m2Vec2 vrA = {vA.x - wA * cp->rA.y, vA.y + wA * cp->rA.x};
            m2Vec2 vrB = {vB.x - wB * cp->rB.y, vB.y + wB * cp->rB.x};
            float vn = (vrB.x - vrA.x) * normal.x + (vrB.y - vrA.y) * normal.y;

            float impulse = -cp->normalMass * (massScale * vn + velocityBias) -
                            impulseScale * cp->normalImpulse;
            float newImpulse = m2MaxF(cp->normalImpulse + impulse, 0.0f);
            impulse = newImpulse - cp->normalImpulse;
            cp->normalImpulse = newImpulse;

            m2Vec2 P = {impulse * normal.x, impulse * normal.y};
            vA.x -= mA * P.x;
            vA.y -= mA * P.y;
            wA -= iA * Cross(cp->rA, P);
            vB.x += mB * P.x;
            vB.y += mB * P.y;
            wB += iB * Cross(cp->rB, P);
        }

        if (!useBias)
        {
            // Friction rides the relax pass, exactly like the reference.
            for (int32_t k = 0; k < c->pointCount; ++k)
            {
                m2ConstraintPoint* cp = &c->points[k];
                m2Vec2 vrA = {vA.x - wA * cp->rA.y, vA.y + wA * cp->rA.x};
                m2Vec2 vrB = {vB.x - wB * cp->rB.y, vB.y + wB * cp->rB.x};
                float vt = (vrB.x - vrA.x) * tangent.x + (vrB.y - vrA.y) * tangent.y;
                float impulse = -cp->tangentMass * vt;
                float maxFriction = c->friction * cp->normalImpulse;
                float newImpulse =
                    m2ClampF(cp->tangentImpulse + impulse, -maxFriction, maxFriction);
                impulse = newImpulse - cp->tangentImpulse;
                cp->tangentImpulse = newImpulse;

                m2Vec2 P = {impulse * tangent.x, impulse * tangent.y};
                vA.x -= mA * P.x;
                vA.y -= mA * P.y;
                wA -= iA * Cross(cp->rA, P);
                vB.x += mB * P.x;
                vB.y += mB * P.y;
                wB += iB * Cross(cp->rB, P);
            }
        }

        world->linearVelocities[c->bodyA] = vA;
        world->angularVelocities[c->bodyA] = wA;
        world->linearVelocities[c->bodyB] = vB;
        world->angularVelocities[c->bodyB] = wB;
    }
}

static void ApplyRestitution(m2World* world, m2ContactConstraint* constraints, int32_t count)
{
    for (int32_t i = 0; i < count; ++i)
    {
        m2ContactConstraint* c = constraints + i;
        if (c->restitution == 0.0f)
        {
            continue;
        }
        float mA = world->invMass[c->bodyA];
        float iA = world->invInertia[c->bodyA];
        float mB = world->invMass[c->bodyB];
        float iB = world->invInertia[c->bodyB];
        m2Vec2 vA = world->linearVelocities[c->bodyA];
        float wA = world->angularVelocities[c->bodyA];
        m2Vec2 vB = world->linearVelocities[c->bodyB];
        float wB = world->angularVelocities[c->bodyB];
        for (int32_t k = 0; k < c->pointCount; ++k)
        {
            m2ConstraintPoint* cp = &c->points[k];
            // Only points that arrived fast and actually carried load.
            if (cp->relativeVelocity > -M2_RESTITUTION_THRESHOLD || cp->normalImpulse == 0.0f)
            {
                continue;
            }
            m2Vec2 vrA = {vA.x - wA * cp->rA.y, vA.y + wA * cp->rA.x};
            m2Vec2 vrB = {vB.x - wB * cp->rB.y, vB.y + wB * cp->rB.x};
            float vn = (vrB.x - vrA.x) * c->normal.x + (vrB.y - vrA.y) * c->normal.y;
            float impulse = -cp->normalMass * (vn + c->restitution * cp->relativeVelocity);
            float newImpulse = m2MaxF(cp->normalImpulse + impulse, 0.0f);
            impulse = newImpulse - cp->normalImpulse;
            cp->normalImpulse = newImpulse;
            m2Vec2 P = {impulse * c->normal.x, impulse * c->normal.y};
            vA.x -= mA * P.x;
            vA.y -= mA * P.y;
            wA -= iA * Cross(cp->rA, P);
            vB.x += mB * P.x;
            vB.y += mB * P.y;
            wB += iB * Cross(cp->rB, P);
        }
        world->linearVelocities[c->bodyA] = vA;
        world->angularVelocities[c->bodyA] = wA;
        world->linearVelocities[c->bodyB] = vB;
        world->angularVelocities[c->bodyB] = wB;
    }
}

static void StoreImpulses(m2World* world, m2ContactConstraint* constraints, int32_t count)
{
    for (int32_t i = 0; i < count; ++i)
    {
        m2ContactConstraint* c = constraints + i;
        m2Manifold* manifold = &world->manifolds[c->pairIndex];
        for (int32_t k = 0; k < c->pointCount; ++k)
        {
            manifold->points[k].normalImpulse = c->points[k].normalImpulse;
            manifold->points[k].tangentImpulse = c->points[k].tangentImpulse;
        }
    }
}

void m2SolveStep(m2World* world, float dt, int32_t substepCount)
{
    float h = dt / (float)substepCount;
    float invH = h > 0.0f ? 1.0f / h : 0.0f;

    // Deltas are step-transient: zero at prepare, folded into f64
    // positions at each integrate-positions stage.
    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        world->deltaPositions[i] = (m2Vec2){0.0f, 0.0f};
        world->deltaRotations[i] = (m2Rot){1.0f, 0.0f};
    }

    m2ContactConstraint* constraints = (m2ContactConstraint*)world->constraintScratch;
    int32_t constraintCount = PrepareContacts(world, constraints, h);

    for (int32_t sub = 0; sub < substepCount; ++sub)
    {
        // Integrate velocities (fixed body order).
        for (int32_t i = 0; i < world->maxBodyIndex; ++i)
        {
            if (world->alive[i] == 0 || world->types[i] != (uint8_t)m2_dynamicBody ||
                world->asleep[i] != 0)
            {
                continue;
            }
            world->linearVelocities[i].x += world->gravity.x * world->gravityScales[i] * h;
            world->linearVelocities[i].y += world->gravity.y * world->gravityScales[i] * h;
        }

        WarmStart(world, constraints, constraintCount);
        SolveContacts(world, constraints, constraintCount, invH, true);

        // Integrate positions: f64 positions advance, f32 deltas track.
        for (int32_t i = 0; i < world->maxBodyIndex; ++i)
        {
            if (world->alive[i] == 0 || world->types[i] == (uint8_t)m2_staticBody ||
                world->asleep[i] != 0)
            {
                continue;
            }
            world->transforms[i].p.x += (double)world->linearVelocities[i].x * (double)h;
            world->transforms[i].p.y += (double)world->linearVelocities[i].y * (double)h;
            world->deltaPositions[i].x += world->linearVelocities[i].x * h;
            world->deltaPositions[i].y += world->linearVelocities[i].y * h;
            m2Rot dq = m2MakeRot(world->angularVelocities[i] * h);
            world->transforms[i].q = m2MulRot(world->transforms[i].q, dq);
            world->deltaRotations[i] = m2MulRot(world->deltaRotations[i], dq);
        }

        SolveContacts(world, constraints, constraintCount, invH, false);
    }

    ApplyRestitution(world, constraints, constraintCount);
    StoreImpulses(world, constraints, constraintCount);
}
