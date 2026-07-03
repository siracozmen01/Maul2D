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

typedef struct m2JointConstraint
{
    int32_t jointIndex;
    int32_t bodyA;
    int32_t bodyB;
    uint8_t type;  // 0 distance, 1 revolute, 2 prismatic, 3 weld, 4 wheel
    uint8_t flags; // bit0 motor, bit1 limit
    m2Vec2 rA;     // world-rotated anchors at prepare
    m2Vec2 rB;
    m2Vec2 axis;         // distance/prismatic: unit axis at prepare
    m2Vec2 perp;         // prismatic: left-perp of axis
    float baseC;         // distance: C0; prismatic: translation0
    m2Vec2 baseCVec;     // revolute: C0; prismatic: (perpC0, unused)
    float baseAngle;     // relative angle at prepare minus reference
    float a1, a2;        // prismatic axial torque arms
    float s1, s2;        // prismatic perpendicular torque arms
    float axialMass;     // distance/prismatic axial; revolute 1/(iA+iB)
    float k11, k12, k22; // revolute/prismatic 2x2 effective mass
    float motorSpeed;
    float maxMotorImpulse; // h * maxMotorTorque(Force), clamp budget
    float lower;
    float upper;
    m2Softness softness;       // constraint rows (stiff for wheel)
    m2Softness springSoftness; // wheel suspension (real spring)
    m2Vec2 impulse;            // point/axial; prismatic (perp, angle); wheel (perp, spring)
    float motorImpulse;        // motor accumulator; weld reuses it for the angle lock
    float lowerImpulse;
    float upperImpulse;
} m2JointConstraint;

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
    // Joint constraints ride in the tail of the same scratch block.
    return (int32_t)sizeof(m2ContactConstraint) + (int32_t)sizeof(m2JointConstraint);
}

static int32_t PrepareContacts(m2World* world, m2ContactConstraint* constraints, float h)
{
    // Stiffer for static contacts, exactly like the reference: a soft
    // ground row is an energy reservoir under a tall stack.
    m2Softness soft = MakeSoft(M2_CONTACT_HERTZ, M2_CONTACT_DAMPING_RATIO, h);
    m2Softness staticSoft = MakeSoft(2.0f * M2_CONTACT_HERTZ, M2_CONTACT_DAMPING_RATIO, h);
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
        c->softness = world->types[bodyA] != (uint8_t)m2_dynamicBody ||
                              world->types[bodyB] != (uint8_t)m2_dynamicBody
                          ? staticSoft
                          : soft;
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
            // Anchors relative to each body's center of mass: the arm
            // the impulse actually torques about (bit-neutral when the
            // COM sits on the origin).
            m2Vec2 lcA = world->localCenters[bodyA];
            m2Vec2 lcB = world->localCenters[bodyB];
            cp->rA = Rotate(qA, (m2Vec2){mp->anchorA.x - lcA.x, mp->anchorA.y - lcA.y});
            cp->rB = Rotate(qB, (m2Vec2){mp->anchorB.x - lcB.x, mp->anchorB.y - lcB.y});
            // Reference factoring: fold the prepare-time anchor gap in,
            // then track the ABSOLUTE rotated gap during solve - the
            // incremental (rs - r0) form cancels catastrophically at
            // small rotations and feeds the bias noise.
            cp->baseSeparation = mp->separation - ((cp->rB.x - cp->rA.x) * c->normal.x +
                                                   (cp->rB.y - cp->rA.y) * c->normal.y);
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

// Write-backs are guarded on dynamic bodies: a static or kinematic
// body shared across graph colors must never be written, even with an
// unchanged value - concurrent identical writes are still a race.
static void StoreBodyVelocities(m2World* world, const m2ContactConstraint* c, m2Vec2 vA, float wA,
                                m2Vec2 vB, float wB)
{
    if (world->types[c->bodyA] == (uint8_t)m2_dynamicBody)
    {
        world->linearVelocities[c->bodyA] = vA;
        world->angularVelocities[c->bodyA] = wA;
    }
    if (world->types[c->bodyB] == (uint8_t)m2_dynamicBody)
    {
        world->linearVelocities[c->bodyB] = vB;
        world->angularVelocities[c->bodyB] = wB;
    }
}

static void WarmStartOne(m2World* world, m2ContactConstraint* c)
{
    float mA = world->invMass[c->bodyA];
    float iA = world->invInertia[c->bodyA];
    float mB = world->invMass[c->bodyB];
    float iB = world->invInertia[c->bodyB];
    m2Vec2 vA = world->linearVelocities[c->bodyA];
    float wA = world->angularVelocities[c->bodyA];
    m2Vec2 vB = world->linearVelocities[c->bodyB];
    float wB = world->angularVelocities[c->bodyB];
    m2Vec2 tangent = {-c->normal.y, c->normal.x};
    for (int32_t k = 0; k < c->pointCount; ++k)
    {
        m2ConstraintPoint* cp = &c->points[k];
        m2Vec2 P = {cp->normalImpulse * c->normal.x + cp->tangentImpulse * tangent.x,
                    cp->normalImpulse * c->normal.y + cp->tangentImpulse * tangent.y};
        vA.x -= mA * P.x;
        vA.y -= mA * P.y;
        wA -= iA * Cross(cp->rA, P);
        vB.x += mB * P.x;
        vB.y += mB * P.y;
        wB += iB * Cross(cp->rB, P);
    }
    StoreBodyVelocities(world, c, vA, wA, vB, wB);
}

static void SolveContactOne(m2World* world, m2ContactConstraint* c, float invH, float minBiasVel,
                            bool useBias)
{
    {
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
            // Reference discipline: FIXED prepare-time anchors for the
            // Jacobian and the applied torque; anchors re-rotated by
            // the substep deltas only measure the current separation.
            m2Vec2 rsA = Rotate(world->deltaRotations[c->bodyA], cp->rA);
            m2Vec2 rsB = Rotate(world->deltaRotations[c->bodyB], cp->rB);
            m2Vec2 ds = {dp.x + rsB.x - rsA.x, dp.y + rsB.y - rsA.y};
            float s = cp->baseSeparation + ds.x * normal.x + ds.y * normal.y;

            // Reference bias selection: the push clamp lands BEFORE the
            // mass scale, and the whole (vn + bias) is scaled together.
            float bias = 0.0f;
            float massScale = 1.0f;
            float impulseScale = 0.0f;
            if (s > 0.0f)
            {
                bias = s * invH; // speculative: prevent crossing
            }
            else if (useBias)
            {
                bias = m2MaxF(c->softness.biasRate * s, minBiasVel);
                massScale = c->softness.massScale;
                impulseScale = c->softness.impulseScale;
            }

            m2Vec2 vrA = {vA.x - wA * cp->rA.y, vA.y + wA * cp->rA.x};
            m2Vec2 vrB = {vB.x - wB * cp->rB.y, vB.y + wB * cp->rB.x};
            float vn = (vrB.x - vrA.x) * normal.x + (vrB.y - vrA.y) * normal.y;

            float impulse =
                -cp->normalMass * massScale * (vn + bias) - impulseScale * cp->normalImpulse;
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

        {
            // Friction solves in BOTH passes, like the reference. This
            // was unstable for twenty slices - because the scrambled
            // pair order was silently dropping warm-start carries and
            // amplifying bias contamination in the accumulators. With
            // the sort fixed, the reference schedule is strictly best
            // (F-T8 closed: pyramid settles 74 vs the reference's 69).
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

        StoreBodyVelocities(world, c, vA, wA, vB, wB);
    }
}

static void RestitutionOne(m2World* world, m2ContactConstraint* c)
{
    {
        if (c->restitution == 0.0f)
        {
            return;
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
        StoreBodyVelocities(world, c, vA, wA, vB, wB);
    }
}

// --- Graph coloring (topic-08): constraints in one color share no
// dynamic body, so a color solves in parallel with bit-identical
// results at ANY worker count. The color assignment itself is greedy
// over canonical constraint order - fully deterministic. The colored
// order is used even when serial, so worker count can never change
// the arithmetic sequence.

#define M2_GRAPH_COLORS 24 // colors 0..23; 24 = overflow, solved serially

static void ColorConstraints(m2World* world, m2ContactConstraint* constraints, int32_t count,
                             int32_t* colorStart)
{
    uint32_t* masks = world->colorMasks;
    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        masks[i] = 0;
    }
    int32_t counts[M2_GRAPH_COLORS + 1];
    for (int32_t c = 0; c <= M2_GRAPH_COLORS; ++c)
    {
        counts[c] = 0;
    }

    for (int32_t i = 0; i < count; ++i)
    {
        int32_t bodyA = constraints[i].bodyA;
        int32_t bodyB = constraints[i].bodyB;
        bool dynA = world->types[bodyA] == (uint8_t)m2_dynamicBody;
        bool dynB = world->types[bodyB] == (uint8_t)m2_dynamicBody;
        uint32_t used = (dynA ? masks[bodyA] : 0u) | (dynB ? masks[bodyB] : 0u);
        int32_t color = 0;
        while (color < M2_GRAPH_COLORS && (used & (1u << color)) != 0)
        {
            color += 1;
        }
        if (color < M2_GRAPH_COLORS && counts[color] >= 256)
        {
            color = M2_GRAPH_COLORS; // full color: spill to the scalar bucket
        }
        if (color < M2_GRAPH_COLORS)
        {
            if (dynA)
            {
                masks[bodyA] |= 1u << color;
            }
            if (dynB)
            {
                masks[bodyB] |= 1u << color;
            }
        }
        world->constraintColors[i] = (uint8_t)color;
        counts[color] += 1;
    }

    colorStart[0] = 0;
    for (int32_t c = 0; c <= M2_GRAPH_COLORS; ++c)
    {
        colorStart[c + 1] = colorStart[c] + counts[c];
    }
    int32_t cursor[M2_GRAPH_COLORS + 1];
    for (int32_t c = 0; c <= M2_GRAPH_COLORS; ++c)
    {
        cursor[c] = colorStart[c];
    }
    for (int32_t i = 0; i < count; ++i)
    {
        int32_t color = world->constraintColors[i];
        world->colorOrder[cursor[color]] = i;
        cursor[color] += 1;
    }
}

typedef enum m2ContactStage
{
    m2_stageWarmStart,
    m2_stageSolve,
    m2_stageRestitution,
    m2_stageStore,
} m2ContactStage;

typedef struct m2ContactStageCtx
{
    m2World* world;
    m2ContactConstraint* constraints;
    const int32_t* order;
    m2ContactStage stage;
    float invH;
    float minBiasVel;
    bool useBias;
} m2ContactStageCtx;

static void ContactStageRange(int32_t begin, int32_t end, void* userCtx)
{
    m2ContactStageCtx* ctx = (m2ContactStageCtx*)userCtx;
    for (int32_t k = begin; k < end; ++k)
    {
        m2ContactConstraint* c = ctx->constraints + ctx->order[k];
        switch (ctx->stage)
        {
        case m2_stageWarmStart:
            WarmStartOne(ctx->world, c);
            break;
        case m2_stageSolve:
            SolveContactOne(ctx->world, c, ctx->invH, ctx->minBiasVel, ctx->useBias);
            break;
        case m2_stageRestitution:
            RestitutionOne(ctx->world, c);
            break;
        default:
        {
            m2Manifold* manifold = &ctx->world->manifolds[c->pairIndex];
            for (int32_t j = 0; j < c->pointCount; ++j)
            {
                manifold->points[j].normalImpulse = c->points[j].normalImpulse;
                manifold->points[j].tangentImpulse = c->points[j].tangentImpulse;
            }
            break;
        }
        }
    }
}

// --- Wide-lane contact solving (topic-08 phase 2) --------------------------
//
// Blocks are SoA bundles of M2_LANES constraints from one graph color,
// homogeneous in point count. Every lane executes the same scalar IEEE
// sequence as the per-item path - the kernels below are transliterations
// of WarmStartOne/SolveContactOne/RestitutionOne - so the lane layout is
// pure mechanical sympathy: compilers vectorize the lane loops, and the
// bits cannot move. Padding lanes point at the dummy body slot
// (index bodyCapacity, zero mass, never scattered).

typedef struct m2ContactBlock
{
    int32_t lanes;      // live lanes; the rest are dummy-padded
    int32_t pointCount; // homogeneous: 1 or 2 for every live lane
    int32_t bodyA[M2_LANES];
    int32_t bodyB[M2_LANES];
    int32_t pairIndex[M2_LANES];
    float invMassA[M2_LANES];
    float invIA[M2_LANES];
    float invMassB[M2_LANES];
    float invIB[M2_LANES];
    float normalX[M2_LANES];
    float normalY[M2_LANES];
    float friction[M2_LANES];
    float restitution[M2_LANES];
    float biasRate[M2_LANES];
    float massScale[M2_LANES];
    float impulseScale[M2_LANES];
    float rAX[2][M2_LANES];
    float rAY[2][M2_LANES];
    float rBX[2][M2_LANES];
    float rBY[2][M2_LANES];
    float baseSep[2][M2_LANES];
    float relVel[2][M2_LANES];
    float normalMass[2][M2_LANES];
    float tangentMass[2][M2_LANES];
    float normalImp[2][M2_LANES];
    float tangentImp[2][M2_LANES];
} m2ContactBlock;

int32_t m2ContactBlockScratchBytes(int32_t pairCapacity)
{
    // Worst case: every block half-full plus two partial blocks per
    // color (one per point-count class).
    int32_t blocks = pairCapacity / M2_LANES + 2 * (M2_GRAPH_COLORS + 1) + 2;
    return blocks * (int32_t)sizeof(m2ContactBlock);
}

// Pack one color's constraints (already partitioned by point count)
// into blocks. Returns the new block count.
static int32_t PackRun(m2World* world, m2ContactConstraint* constraints, const int32_t* order,
                       int32_t count, int32_t pointCount, int32_t blockCount)
{
    m2ContactBlock* blocks = (m2ContactBlock*)world->contactBlocks;
    int32_t dummy = world->bodyCapacity;
    for (int32_t base = 0; base < count; base += M2_LANES)
    {
        m2ContactBlock* block = blocks + blockCount;
        blockCount += 1;
        int32_t lanes = count - base < M2_LANES ? count - base : M2_LANES;
        block->lanes = lanes;
        block->pointCount = pointCount;
        for (int32_t lane = 0; lane < M2_LANES; ++lane)
        {
            if (lane >= lanes)
            {
                block->bodyA[lane] = dummy;
                block->bodyB[lane] = dummy;
                block->pairIndex[lane] = -1;
                block->invMassA[lane] = 0.0f;
                block->invIA[lane] = 0.0f;
                block->invMassB[lane] = 0.0f;
                block->invIB[lane] = 0.0f;
                block->normalX[lane] = 0.0f;
                block->normalY[lane] = 1.0f;
                block->friction[lane] = 0.0f;
                block->restitution[lane] = 0.0f;
                block->biasRate[lane] = 0.0f;
                block->massScale[lane] = 1.0f;
                block->impulseScale[lane] = 0.0f;
                for (int32_t k = 0; k < 2; ++k)
                {
                    block->rAX[k][lane] = 0.0f;
                    block->rAY[k][lane] = 0.0f;
                    block->rBX[k][lane] = 0.0f;
                    block->rBY[k][lane] = 0.0f;
                    block->baseSep[k][lane] = 1.0f; // separated: speculative no-op
                    block->relVel[k][lane] = 0.0f;
                    block->normalMass[k][lane] = 0.0f;
                    block->tangentMass[k][lane] = 0.0f;
                    block->normalImp[k][lane] = 0.0f;
                    block->tangentImp[k][lane] = 0.0f;
                }
                continue;
            }
            m2ContactConstraint* c = constraints + order[base + lane];
            block->bodyA[lane] = c->bodyA;
            block->bodyB[lane] = c->bodyB;
            block->pairIndex[lane] = c->pairIndex;
            block->invMassA[lane] = world->invMass[c->bodyA];
            block->invIA[lane] = world->invInertia[c->bodyA];
            block->invMassB[lane] = world->invMass[c->bodyB];
            block->invIB[lane] = world->invInertia[c->bodyB];
            block->normalX[lane] = c->normal.x;
            block->normalY[lane] = c->normal.y;
            block->friction[lane] = c->friction;
            block->restitution[lane] = c->restitution;
            block->biasRate[lane] = c->softness.biasRate;
            block->massScale[lane] = c->softness.massScale;
            block->impulseScale[lane] = c->softness.impulseScale;
            for (int32_t k = 0; k < 2; ++k)
            {
                const m2ConstraintPoint* cp = &c->points[k < c->pointCount ? k : 0];
                bool live = k < c->pointCount;
                block->rAX[k][lane] = live ? cp->rA.x : 0.0f;
                block->rAY[k][lane] = live ? cp->rA.y : 0.0f;
                block->rBX[k][lane] = live ? cp->rB.x : 0.0f;
                block->rBY[k][lane] = live ? cp->rB.y : 0.0f;
                block->baseSep[k][lane] = live ? cp->baseSeparation : 1.0f;
                block->relVel[k][lane] = live ? cp->relativeVelocity : 0.0f;
                block->normalMass[k][lane] = live ? cp->normalMass : 0.0f;
                block->tangentMass[k][lane] = live ? cp->tangentMass : 0.0f;
                block->normalImp[k][lane] = live ? cp->normalImpulse : 0.0f;
                block->tangentImp[k][lane] = live ? cp->tangentImpulse : 0.0f;
            }
        }
    }
    return blockCount;
}

// Partition each full color by point count (order within a color is
// free: its constraints share no dynamic body), then pack. Returns
// block count; blockStart[c]..blockStart[c+1] are color c's blocks.
static int32_t PackContactBlocks(m2World* world, m2ContactConstraint* constraints,
                                 const int32_t* colorStart, int32_t* blockStart)
{
    int32_t blockCount = 0;
    int32_t scratch[2][256];
    for (int32_t color = 0; color < M2_GRAPH_COLORS; ++color)
    {
        blockStart[color] = blockCount;
        int32_t begin = colorStart[color];
        int32_t end = colorStart[color + 1];
        int32_t twos = 0;
        int32_t ones = 0;
        for (int32_t k = begin; k < end; ++k)
        {
            int32_t index = world->colorOrder[k];
            if (constraints[index].pointCount == 2)
            {
                if (twos < 256)
                {
                    scratch[0][twos] = index;
                }
                twos += 1;
            }
            else
            {
                if (ones < 256)
                {
                    scratch[1][ones] = index;
                }
                ones += 1;
            }
            // Colors are capped by the per-body u32 mask, but a color
            // can hold more than 256 constraints in huge worlds; spill
            // the excess back into the overflow-style scalar path by
            // marking it - handled below via the spill list.
        }
        M2_ASSERT(twos <= 256 && ones <= 256);
        blockCount =
            PackRun(world, constraints, scratch[0], twos <= 256 ? twos : 256, 2, blockCount);
        blockCount =
            PackRun(world, constraints, scratch[1], ones <= 256 ? ones : 256, 1, blockCount);
    }
    blockStart[M2_GRAPH_COLORS] = blockCount;
    return blockCount;
}

typedef struct m2BlockStageCtx
{
    m2World* world;
    m2ContactBlock* blocks;
    m2ContactStage stage;
    float invH;
    float minBiasVel;
    bool useBias;
} m2BlockStageCtx;

static void WarmStartBlock(m2World* world, m2ContactBlock* b)
{
    float vAx[M2_LANES];
    float vAy[M2_LANES];
    float wA[M2_LANES];
    float vBx[M2_LANES];
    float vBy[M2_LANES];
    float wB[M2_LANES];
    for (int32_t lane = 0; lane < M2_LANES; ++lane)
    {
        vAx[lane] = world->linearVelocities[b->bodyA[lane]].x;
        vAy[lane] = world->linearVelocities[b->bodyA[lane]].y;
        wA[lane] = world->angularVelocities[b->bodyA[lane]];
        vBx[lane] = world->linearVelocities[b->bodyB[lane]].x;
        vBy[lane] = world->linearVelocities[b->bodyB[lane]].y;
        wB[lane] = world->angularVelocities[b->bodyB[lane]];
    }
    for (int32_t k = 0; k < b->pointCount; ++k)
    {
        for (int32_t lane = 0; lane < M2_LANES; ++lane)
        {
            float tangentX = -b->normalY[lane];
            float tangentY = b->normalX[lane];
            float Px = b->normalImp[k][lane] * b->normalX[lane] + b->tangentImp[k][lane] * tangentX;
            float Py = b->normalImp[k][lane] * b->normalY[lane] + b->tangentImp[k][lane] * tangentY;
            vAx[lane] -= b->invMassA[lane] * Px;
            vAy[lane] -= b->invMassA[lane] * Py;
            wA[lane] -= b->invIA[lane] * (b->rAX[k][lane] * Py - b->rAY[k][lane] * Px);
            vBx[lane] += b->invMassB[lane] * Px;
            vBy[lane] += b->invMassB[lane] * Py;
            wB[lane] += b->invIB[lane] * (b->rBX[k][lane] * Py - b->rBY[k][lane] * Px);
        }
    }
    for (int32_t lane = 0; lane < M2_LANES; ++lane)
    {
        if (world->types[b->bodyA[lane]] == (uint8_t)m2_dynamicBody)
        {
            world->linearVelocities[b->bodyA[lane]] = (m2Vec2){vAx[lane], vAy[lane]};
            world->angularVelocities[b->bodyA[lane]] = wA[lane];
        }
        if (world->types[b->bodyB[lane]] == (uint8_t)m2_dynamicBody)
        {
            world->linearVelocities[b->bodyB[lane]] = (m2Vec2){vBx[lane], vBy[lane]};
            world->angularVelocities[b->bodyB[lane]] = wB[lane];
        }
    }
}

static void SolveBlock(m2World* world, m2ContactBlock* b, float invH, float minBiasVel,
                       bool useBias)
{
    float vAx[M2_LANES];
    float vAy[M2_LANES];
    float wA[M2_LANES];
    float vBx[M2_LANES];
    float vBy[M2_LANES];
    float wB[M2_LANES];
    float dpx[M2_LANES];
    float dpy[M2_LANES];
    float dqAc[M2_LANES];
    float dqAs[M2_LANES];
    float dqBc[M2_LANES];
    float dqBs[M2_LANES];
    for (int32_t lane = 0; lane < M2_LANES; ++lane)
    {
        int32_t iA = b->bodyA[lane];
        int32_t iB = b->bodyB[lane];
        vAx[lane] = world->linearVelocities[iA].x;
        vAy[lane] = world->linearVelocities[iA].y;
        wA[lane] = world->angularVelocities[iA];
        vBx[lane] = world->linearVelocities[iB].x;
        vBy[lane] = world->linearVelocities[iB].y;
        wB[lane] = world->angularVelocities[iB];
        dpx[lane] = world->deltaPositions[iB].x - world->deltaPositions[iA].x;
        dpy[lane] = world->deltaPositions[iB].y - world->deltaPositions[iA].y;
        dqAc[lane] = world->deltaRotations[iA].c;
        dqAs[lane] = world->deltaRotations[iA].s;
        dqBc[lane] = world->deltaRotations[iB].c;
        dqBs[lane] = world->deltaRotations[iB].s;
    }

    for (int32_t k = 0; k < b->pointCount; ++k)
    {
        for (int32_t lane = 0; lane < M2_LANES; ++lane)
        {
            float rsAx = dqAc[lane] * b->rAX[k][lane] - dqAs[lane] * b->rAY[k][lane];
            float rsAy = dqAs[lane] * b->rAX[k][lane] + dqAc[lane] * b->rAY[k][lane];
            float rsBx = dqBc[lane] * b->rBX[k][lane] - dqBs[lane] * b->rBY[k][lane];
            float rsBy = dqBs[lane] * b->rBX[k][lane] + dqBc[lane] * b->rBY[k][lane];
            float dsx = dpx[lane] + rsBx - rsAx;
            float dsy = dpy[lane] + rsBy - rsAy;
            float sep = b->baseSep[k][lane] + dsx * b->normalX[lane] + dsy * b->normalY[lane];

            // Value selects instead of branches: both candidates are pure
            // values, the selection is bit-identical to the branchy
            // form, and the vectorizer gets a straight-line loop.
            bool speculative = sep > 0.0f;
            float softBias = m2MaxF(b->biasRate[lane] * sep, minBiasVel);
            float bias = speculative ? sep * invH : (useBias ? softBias : 0.0f);
            float massScale = !speculative && useBias ? b->massScale[lane] : 1.0f;
            float impulseScale = !speculative && useBias ? b->impulseScale[lane] : 0.0f;

            float vrAx = vAx[lane] - wA[lane] * b->rAY[k][lane];
            float vrAy = vAy[lane] + wA[lane] * b->rAX[k][lane];
            float vrBx = vBx[lane] - wB[lane] * b->rBY[k][lane];
            float vrBy = vBy[lane] + wB[lane] * b->rBX[k][lane];
            float vn = (vrBx - vrAx) * b->normalX[lane] + (vrBy - vrAy) * b->normalY[lane];

            float impulse = -b->normalMass[k][lane] * massScale * (vn + bias) -
                            impulseScale * b->normalImp[k][lane];
            float newImpulse = m2MaxF(b->normalImp[k][lane] + impulse, 0.0f);
            impulse = newImpulse - b->normalImp[k][lane];
            b->normalImp[k][lane] = newImpulse;

            float Px = impulse * b->normalX[lane];
            float Py = impulse * b->normalY[lane];
            vAx[lane] -= b->invMassA[lane] * Px;
            vAy[lane] -= b->invMassA[lane] * Py;
            wA[lane] -= b->invIA[lane] * (b->rAX[k][lane] * Py - b->rAY[k][lane] * Px);
            vBx[lane] += b->invMassB[lane] * Px;
            vBy[lane] += b->invMassB[lane] * Py;
            wB[lane] += b->invIB[lane] * (b->rBX[k][lane] * Py - b->rBY[k][lane] * Px);
        }
    }

    for (int32_t k = 0; k < b->pointCount; ++k)
    {
        for (int32_t lane = 0; lane < M2_LANES; ++lane)
        {
            float tangentX = -b->normalY[lane];
            float tangentY = b->normalX[lane];
            float vrAx = vAx[lane] - wA[lane] * b->rAY[k][lane];
            float vrAy = vAy[lane] + wA[lane] * b->rAX[k][lane];
            float vrBx = vBx[lane] - wB[lane] * b->rBY[k][lane];
            float vrBy = vBy[lane] + wB[lane] * b->rBX[k][lane];
            float vt = (vrBx - vrAx) * tangentX + (vrBy - vrAy) * tangentY;
            float impulse = -b->tangentMass[k][lane] * vt;
            float maxFriction = b->friction[lane] * b->normalImp[k][lane];
            float newImpulse =
                m2ClampF(b->tangentImp[k][lane] + impulse, -maxFriction, maxFriction);
            impulse = newImpulse - b->tangentImp[k][lane];
            b->tangentImp[k][lane] = newImpulse;

            float Px = impulse * tangentX;
            float Py = impulse * tangentY;
            vAx[lane] -= b->invMassA[lane] * Px;
            vAy[lane] -= b->invMassA[lane] * Py;
            wA[lane] -= b->invIA[lane] * (b->rAX[k][lane] * Py - b->rAY[k][lane] * Px);
            vBx[lane] += b->invMassB[lane] * Px;
            vBy[lane] += b->invMassB[lane] * Py;
            wB[lane] += b->invIB[lane] * (b->rBX[k][lane] * Py - b->rBY[k][lane] * Px);
        }
    }

    for (int32_t lane = 0; lane < M2_LANES; ++lane)
    {
        if (world->types[b->bodyA[lane]] == (uint8_t)m2_dynamicBody)
        {
            world->linearVelocities[b->bodyA[lane]] = (m2Vec2){vAx[lane], vAy[lane]};
            world->angularVelocities[b->bodyA[lane]] = wA[lane];
        }
        if (world->types[b->bodyB[lane]] == (uint8_t)m2_dynamicBody)
        {
            world->linearVelocities[b->bodyB[lane]] = (m2Vec2){vBx[lane], vBy[lane]};
            world->angularVelocities[b->bodyB[lane]] = wB[lane];
        }
    }
}

static void RestitutionBlock(m2World* world, m2ContactBlock* b)
{
    // Per-lane branches on purpose: the scalar path skips whole
    // updates, and a masked add of a signed zero would move bits.
    for (int32_t lane = 0; lane < b->lanes; ++lane)
    {
        if (b->restitution[lane] == 0.0f)
        {
            continue;
        }
        float vAx = world->linearVelocities[b->bodyA[lane]].x;
        float vAy = world->linearVelocities[b->bodyA[lane]].y;
        float wA = world->angularVelocities[b->bodyA[lane]];
        float vBx = world->linearVelocities[b->bodyB[lane]].x;
        float vBy = world->linearVelocities[b->bodyB[lane]].y;
        float wB = world->angularVelocities[b->bodyB[lane]];
        for (int32_t k = 0; k < b->pointCount; ++k)
        {
            if (b->relVel[k][lane] > -M2_RESTITUTION_THRESHOLD || b->normalImp[k][lane] == 0.0f)
            {
                continue;
            }
            float vrAx = vAx - wA * b->rAY[k][lane];
            float vrAy = vAy + wA * b->rAX[k][lane];
            float vrBx = vBx - wB * b->rBY[k][lane];
            float vrBy = vBy + wB * b->rBX[k][lane];
            float vn = (vrBx - vrAx) * b->normalX[lane] + (vrBy - vrAy) * b->normalY[lane];
            float impulse =
                -b->normalMass[k][lane] * (vn + b->restitution[lane] * b->relVel[k][lane]);
            float newImpulse = m2MaxF(b->normalImp[k][lane] + impulse, 0.0f);
            impulse = newImpulse - b->normalImp[k][lane];
            b->normalImp[k][lane] = newImpulse;
            float Px = impulse * b->normalX[lane];
            float Py = impulse * b->normalY[lane];
            vAx -= b->invMassA[lane] * Px;
            vAy -= b->invMassA[lane] * Py;
            wA -= b->invIA[lane] * (b->rAX[k][lane] * Py - b->rAY[k][lane] * Px);
            vBx += b->invMassB[lane] * Px;
            vBy += b->invMassB[lane] * Py;
            wB += b->invIB[lane] * (b->rBX[k][lane] * Py - b->rBY[k][lane] * Px);
        }
        if (world->types[b->bodyA[lane]] == (uint8_t)m2_dynamicBody)
        {
            world->linearVelocities[b->bodyA[lane]] = (m2Vec2){vAx, vAy};
            world->angularVelocities[b->bodyA[lane]] = wA;
        }
        if (world->types[b->bodyB[lane]] == (uint8_t)m2_dynamicBody)
        {
            world->linearVelocities[b->bodyB[lane]] = (m2Vec2){vBx, vBy};
            world->angularVelocities[b->bodyB[lane]] = wB;
        }
    }
}

static void StoreBlock(m2World* world, m2ContactBlock* b)
{
    for (int32_t lane = 0; lane < b->lanes; ++lane)
    {
        m2Manifold* manifold = &world->manifolds[b->pairIndex[lane]];
        for (int32_t k = 0; k < b->pointCount; ++k)
        {
            manifold->points[k].normalImpulse = b->normalImp[k][lane];
            manifold->points[k].tangentImpulse = b->tangentImp[k][lane];
        }
    }
}

static void BlockStageRange(int32_t begin, int32_t end, void* userCtx)
{
    m2BlockStageCtx* ctx = (m2BlockStageCtx*)userCtx;
    for (int32_t i = begin; i < end; ++i)
    {
        m2ContactBlock* block = ctx->blocks + i;
        switch (ctx->stage)
        {
        case m2_stageWarmStart:
            WarmStartBlock(ctx->world, block);
            break;
        case m2_stageSolve:
            SolveBlock(ctx->world, block, ctx->invH, ctx->minBiasVel, ctx->useBias);
            break;
        case m2_stageRestitution:
            RestitutionBlock(ctx->world, block);
            break;
        default:
            StoreBlock(ctx->world, block);
            break;
        }
    }
}

// Full colors run wide; the overflow bucket stays on the per-item path.
static void RunContactStageWide(m2World* world, m2ContactConstraint* constraints,
                                const int32_t* colorStart, const int32_t* blockStart,
                                m2ContactStage stage, float invH, float minBiasVel, bool useBias)
{
    m2BlockStageCtx ctx;
    ctx.world = world;
    ctx.blocks = (m2ContactBlock*)world->contactBlocks;
    ctx.stage = stage;
    ctx.invH = invH;
    ctx.minBiasVel = minBiasVel;
    ctx.useBias = useBias;
    for (int32_t color = 0; color < M2_GRAPH_COLORS; ++color)
    {
        int32_t begin = blockStart[color];
        int32_t end = blockStart[color + 1];
        if (begin == end)
        {
            continue;
        }
        ctx.blocks = (m2ContactBlock*)world->contactBlocks + begin;
        m2ThreadPoolRun(world->pool, BlockStageRange, &ctx, end - begin);
    }

    // Overflow bucket: serial per-item path, canonical order.
    int32_t begin = colorStart[M2_GRAPH_COLORS];
    int32_t end = colorStart[M2_GRAPH_COLORS + 1];
    if (begin != end)
    {
        m2ContactStageCtx overflow;
        overflow.world = world;
        overflow.constraints = constraints;
        overflow.order = world->colorOrder + begin;
        overflow.stage = stage;
        overflow.invH = invH;
        overflow.minBiasVel = minBiasVel;
        overflow.useBias = useBias;
        ContactStageRange(0, end - begin, &overflow);
    }
}

// Relative angle of B vs A (own trig per ADR-0010).
static float RelativeRotAngle(m2Rot qA, m2Rot qB)
{
    float sin = qA.c * qB.s - qA.s * qB.c;
    float cos = qA.c * qB.c + qA.s * qB.s;
    return m2Atan2(sin, cos);
}

static int32_t PrepareJoints(m2World* world, m2JointConstraint* joints, float h)
{
    // Stiff default softness for hertz==0 (F-T5-4 surface pending).
    int32_t count = 0;
    for (int32_t j = 0; j < world->maxJointIndex; ++j)
    {
        if (world->jointAlive[j] == 0)
        {
            continue;
        }
        int32_t bodyA = world->jointBodyA[j];
        int32_t bodyB = world->jointBodyB[j];
        if ((world->types[bodyA] != (uint8_t)m2_dynamicBody || world->asleep[bodyA] != 0) &&
            (world->types[bodyB] != (uint8_t)m2_dynamicBody || world->asleep[bodyB] != 0))
        {
            continue; // both ends inert this step
        }
        m2JointConstraint* c = joints + count;
        count += 1;
        c->jointIndex = j;
        c->bodyA = bodyA;
        c->bodyB = bodyB;
        c->type = world->jointType[j];
        c->flags = world->jointFlags[j];
        float hertz = world->jointHertz[j] > 0.0f ? world->jointHertz[j] : 60.0f;
        float damping = world->jointHertz[j] > 0.0f ? world->jointDamping[j] : 2.0f;
        c->softness = MakeSoft(hertz, damping, h);
        c->impulse = world->jointImpulse[j];
        c->motorSpeed = world->jointMotorSpeed[j];
        c->maxMotorImpulse = h * world->jointMaxMotor[j];
        c->lower = world->jointLower[j];
        c->upper = world->jointUpper[j];
        c->motorImpulse = world->jointMotorImpulse[j];
        c->lowerImpulse = world->jointLowerImpulse[j];
        c->upperImpulse = world->jointUpperImpulse[j];

        m2Rot qA = world->transforms[bodyA].q;
        m2Rot qB = world->transforms[bodyB].q;
        m2Vec2 lcA = world->localCenters[bodyA];
        m2Vec2 lcB = world->localCenters[bodyB];
        c->rA = Rotate(qA, (m2Vec2){world->jointLocalAnchorA[j].x - lcA.x,
                                    world->jointLocalAnchorA[j].y - lcA.y});
        c->rB = Rotate(qB, (m2Vec2){world->jointLocalAnchorB[j].x - lcB.x,
                                    world->jointLocalAnchorB[j].y - lcB.y});
        m2Vec2 comA = Rotate(qA, lcA);
        m2Vec2 comB = Rotate(qB, lcB);
        float dx = (float)(world->transforms[bodyB].p.x - world->transforms[bodyA].p.x) +
                   (comB.x - comA.x) + c->rB.x - c->rA.x;
        float dy = (float)(world->transforms[bodyB].p.y - world->transforms[bodyA].p.y) +
                   (comB.y - comA.y) + c->rB.y - c->rA.y;
        float mA = world->invMass[bodyA];
        float iA = world->invInertia[bodyA];
        float mB = world->invMass[bodyB];
        float iB = world->invInertia[bodyB];

        if (c->type == 0)
        {
            float length = sqrtf(dx * dx + dy * dy);
            c->axis = length > 1.19209290e-7f ? (m2Vec2){dx / length, dy / length}
                                              : (m2Vec2){0.0f, 1.0f}; // canonical fallback
            c->baseC = length - world->jointLength[j];
            float crA = Cross(c->rA, c->axis);
            float crB = Cross(c->rB, c->axis);
            float k = mA + mB + iA * crA * crA + iB * crB * crB;
            c->axialMass = k > 0.0f ? 1.0f / k : 0.0f;
        }
        else if (c->type == 1 || c->type == 3)
        {
            c->baseCVec = (m2Vec2){dx, dy};
            c->k11 = mA + mB + iA * c->rA.y * c->rA.y + iB * c->rB.y * c->rB.y;
            c->k12 = -iA * c->rA.x * c->rA.y - iB * c->rB.x * c->rB.y;
            c->k22 = mA + mB + iA * c->rA.x * c->rA.x + iB * c->rB.x * c->rB.x;
            float k = iA + iB;
            c->axialMass = k > 0.0f ? 1.0f / k : 0.0f;
            c->baseAngle = m2UnwindAngle(RelativeRotAngle(qA, qB) - world->jointRefAngle[j]);
        }
        else if (c->type == 2)
        {
            // Prismatic frame at prepare: Jacobians frozen for the
            // substep like the revolute point block (recorded
            // adaptation of the reference's per-iteration re-rotation).
            c->axis = Rotate(qA, world->jointLocalAxisA[j]);
            c->perp = (m2Vec2){-c->axis.y, c->axis.x};
            c->baseC = dx * c->axis.x + dy * c->axis.y; // translation0
            c->baseCVec = (m2Vec2){dx * c->perp.x + dy * c->perp.y, 0.0f};
            c->baseAngle = m2UnwindAngle(RelativeRotAngle(qA, qB) - world->jointRefAngle[j]);
            m2Vec2 dPlusRA = {dx + c->rA.x, dy + c->rA.y};
            c->a1 = Cross(dPlusRA, c->axis);
            c->a2 = Cross(c->rB, c->axis);
            c->s1 = Cross(dPlusRA, c->perp);
            c->s2 = Cross(c->rB, c->perp);
            float ka = mA + mB + iA * c->a1 * c->a1 + iB * c->a2 * c->a2;
            c->axialMass = ka > 0.0f ? 1.0f / ka : 0.0f;
            c->k11 = mA + mB + iA * c->s1 * c->s1 + iB * c->s2 * c->s2;
            c->k12 = iA * c->s1 + iB * c->s2;
            float k22 = iA + iB;
            c->k22 = k22 > 0.0f ? k22 : 1.0f; // fixed-rotation guard (reference)
        }
        else
        {
            // Wheel: slider frame like the prismatic, but rotation is
            // free. The user's hertz shapes the suspension spring; the
            // constraint rows stay on the stiff default.
            c->springSoftness = c->softness;
            c->softness = MakeSoft(60.0f, 2.0f, h);
            c->axis = Rotate(qA, world->jointLocalAxisA[j]);
            c->perp = (m2Vec2){-c->axis.y, c->axis.x};
            c->baseC = dx * c->axis.x + dy * c->axis.y; // translation0
            c->baseCVec = (m2Vec2){dx * c->perp.x + dy * c->perp.y, 0.0f};
            m2Vec2 dPlusRA = {dx + c->rA.x, dy + c->rA.y};
            c->a1 = Cross(dPlusRA, c->axis);
            c->a2 = Cross(c->rB, c->axis);
            c->s1 = Cross(dPlusRA, c->perp);
            c->s2 = Cross(c->rB, c->perp);
            float ka = mA + mB + iA * c->a1 * c->a1 + iB * c->a2 * c->a2;
            c->axialMass = ka > 0.0f ? 1.0f / ka : 0.0f;
            float kp = mA + mB + iA * c->s1 * c->s1 + iB * c->s2 * c->s2;
            c->k11 = kp > 0.0f ? 1.0f / kp : 0.0f; // perp effective mass
            float km = iA + iB;
            c->k22 = km > 0.0f ? 1.0f / km : 0.0f; // motor effective mass
        }
    }
    return count;
}

static void ApplyJointImpulse(m2World* world, const m2JointConstraint* c, m2Vec2 P)
{
    float mA = world->invMass[c->bodyA];
    float iA = world->invInertia[c->bodyA];
    float mB = world->invMass[c->bodyB];
    float iB = world->invInertia[c->bodyB];
    world->linearVelocities[c->bodyA].x -= mA * P.x;
    world->linearVelocities[c->bodyA].y -= mA * P.y;
    world->angularVelocities[c->bodyA] -= iA * Cross(c->rA, P);
    world->linearVelocities[c->bodyB].x += mB * P.x;
    world->linearVelocities[c->bodyB].y += mB * P.y;
    world->angularVelocities[c->bodyB] += iB * Cross(c->rB, P);
}

// Prismatic impulses act along frozen Jacobians, not through rA/rB.
static void ApplyPrismaticImpulse(m2World* world, const m2JointConstraint* c, m2Vec2 P, float LA,
                                  float LB)
{
    float mA = world->invMass[c->bodyA];
    float iA = world->invInertia[c->bodyA];
    float mB = world->invMass[c->bodyB];
    float iB = world->invInertia[c->bodyB];
    world->linearVelocities[c->bodyA].x -= mA * P.x;
    world->linearVelocities[c->bodyA].y -= mA * P.y;
    world->angularVelocities[c->bodyA] -= iA * LA;
    world->linearVelocities[c->bodyB].x += mB * P.x;
    world->linearVelocities[c->bodyB].y += mB * P.y;
    world->angularVelocities[c->bodyB] += iB * LB;
}

static void WarmStartJoints(m2World* world, m2JointConstraint* joints, int32_t count)
{
    for (int32_t i = 0; i < count; ++i)
    {
        m2JointConstraint* c = joints + i;
        if (c->type == 0)
        {
            ApplyJointImpulse(world, c,
                              (m2Vec2){c->impulse.x * c->axis.x, c->impulse.x * c->axis.y});
        }
        else if (c->type == 1 || c->type == 3)
        {
            // Weld's angle-lock accumulator rides the motor slot.
            ApplyJointImpulse(world, c, c->impulse);
            float axial = c->motorImpulse + c->lowerImpulse - c->upperImpulse;
            world->angularVelocities[c->bodyA] -= world->invInertia[c->bodyA] * axial;
            world->angularVelocities[c->bodyB] += world->invInertia[c->bodyB] * axial;
        }
        else if (c->type == 2)
        {
            float axial = c->motorImpulse + c->lowerImpulse - c->upperImpulse;
            m2Vec2 P = {axial * c->axis.x + c->impulse.x * c->perp.x,
                        axial * c->axis.y + c->impulse.x * c->perp.y};
            float LA = axial * c->a1 + c->impulse.x * c->s1 + c->impulse.y;
            float LB = axial * c->a2 + c->impulse.x * c->s2 + c->impulse.y;
            ApplyPrismaticImpulse(world, c, P, LA, LB);
        }
        else
        {
            // Wheel: spring rides impulse.y, the motor is pure angular.
            float axial = c->impulse.y + c->lowerImpulse - c->upperImpulse;
            m2Vec2 P = {axial * c->axis.x + c->impulse.x * c->perp.x,
                        axial * c->axis.y + c->impulse.x * c->perp.y};
            float LA = axial * c->a1 + c->impulse.x * c->s1 + c->motorImpulse;
            float LB = axial * c->a2 + c->impulse.x * c->s2 + c->motorImpulse;
            ApplyPrismaticImpulse(world, c, P, LA, LB);
        }
    }
}

// One axial sub-constraint (motor or limit) on the shared axial
// Jacobian; oneSided clamps the accumulator at zero (limits), otherwise
// it clamps symmetrically at the motor budget. Returns the delta.
static float SolveAxial(m2JointConstraint* c, float cdot, float bias, float massScale,
                        float impulseScale, float* accumulated, bool oneSided)
{
    float old = *accumulated;
    float impulse = -massScale * c->axialMass * (cdot + bias) - impulseScale * old;
    float next = old + impulse;
    if (oneSided)
    {
        next = m2MaxF(next, 0.0f);
    }
    else
    {
        next = m2ClampF(next, -c->maxMotorImpulse, c->maxMotorImpulse);
    }
    *accumulated = next;
    return next - old;
}

static void SolveJoints(m2World* world, m2JointConstraint* joints, int32_t count, bool useBias,
                        float invH)
{
    for (int32_t i = 0; i < count; ++i)
    {
        m2JointConstraint* c = joints + i;
        m2Vec2 vA = world->linearVelocities[c->bodyA];
        float wA = world->angularVelocities[c->bodyA];
        m2Vec2 vB = world->linearVelocities[c->bodyB];
        float wB = world->angularVelocities[c->bodyB];
        m2Vec2 dp = {world->deltaPositions[c->bodyB].x - world->deltaPositions[c->bodyA].x,
                     world->deltaPositions[c->bodyB].y - world->deltaPositions[c->bodyA].y};
        m2Vec2 drB = Rotate(world->deltaRotations[c->bodyB], c->rB);
        m2Vec2 drA = Rotate(world->deltaRotations[c->bodyA], c->rA);
        m2Vec2 ds = {dp.x + (drB.x - c->rB.x) - (drA.x - c->rA.x),
                     dp.y + (drB.y - c->rB.y) - (drA.y - c->rA.y)};

        if (c->type == 0)
        {
            float bias = 0.0f;
            float massScale = 1.0f;
            float impulseScale = 0.0f;
            if (useBias)
            {
                float C = c->baseC + ds.x * c->axis.x + ds.y * c->axis.y;
                bias = c->softness.massScale * c->softness.biasRate * C;
                massScale = c->softness.massScale;
                impulseScale = c->softness.impulseScale;
            }
            m2Vec2 vrA = {vA.x - wA * c->rA.y, vA.y + wA * c->rA.x};
            m2Vec2 vrB = {vB.x - wB * c->rB.y, vB.y + wB * c->rB.x};
            float cdot = (vrB.x - vrA.x) * c->axis.x + (vrB.y - vrA.y) * c->axis.y;
            float impulse = -c->axialMass * (massScale * cdot + bias) - impulseScale * c->impulse.x;
            c->impulse.x += impulse;
            ApplyJointImpulse(world, c, (m2Vec2){impulse * c->axis.x, impulse * c->axis.y});
        }
        else if (c->type == 1 || c->type == 3)
        {
            if (c->type == 3 && c->axialMass > 0.0f)
            {
                // Weld angle lock (reference weld_joint.c): a full
                // constraint row on relative rotation, accumulator in
                // the motor slot.
                float bias = 0.0f;
                float massScale = 1.0f;
                float impulseScale = 0.0f;
                if (useBias)
                {
                    float C = m2UnwindAngle(c->baseAngle +
                                            RelativeRotAngle(world->deltaRotations[c->bodyA],
                                                             world->deltaRotations[c->bodyB]));
                    bias = c->softness.biasRate * C;
                    massScale = c->softness.massScale;
                    impulseScale = c->softness.impulseScale;
                }
                float cdot = wB - wA;
                float impulse =
                    -massScale * c->axialMass * (cdot + bias) - impulseScale * c->motorImpulse;
                c->motorImpulse += impulse;
                wA -= world->invInertia[c->bodyA] * impulse;
                wB += world->invInertia[c->bodyB] * impulse;
            }
            if (c->type == 1 && (c->flags & 1u) != 0 && c->axialMass > 0.0f)
            {
                // Motor: drive relative spin toward motorSpeed within
                // the per-step torque budget (reference formulation).
                float cdot = wB - wA - c->motorSpeed;
                float delta = SolveAxial(c, cdot, 0.0f, 1.0f, 0.0f, &c->motorImpulse, false);
                wA -= world->invInertia[c->bodyA] * delta;
                wB += world->invInertia[c->bodyB] * delta;
            }
            if (c->type == 1 && (c->flags & 2u) != 0 && c->axialMass > 0.0f)
            {
                float jointAngle =
                    m2UnwindAngle(c->baseAngle + RelativeRotAngle(world->deltaRotations[c->bodyA],
                                                                  world->deltaRotations[c->bodyB]));
                { // lower: open limits speculate, violated limits go soft
                    float C = jointAngle - c->lower;
                    float bias = 0.0f;
                    float massScale = 1.0f;
                    float impulseScale = 0.0f;
                    if (C > 0.0f)
                    {
                        bias = C * invH;
                    }
                    else if (useBias)
                    {
                        bias = c->softness.biasRate * C;
                        massScale = c->softness.massScale;
                        impulseScale = c->softness.impulseScale;
                    }
                    float delta = SolveAxial(c, wB - wA, bias, massScale, impulseScale,
                                             &c->lowerImpulse, true);
                    wA -= world->invInertia[c->bodyA] * delta;
                    wB += world->invInertia[c->bodyB] * delta;
                }
                { // upper: signs flipped so C stays positive when satisfied
                    float C = c->upper - jointAngle;
                    float bias = 0.0f;
                    float massScale = 1.0f;
                    float impulseScale = 0.0f;
                    if (C > 0.0f)
                    {
                        bias = C * invH;
                    }
                    else if (useBias)
                    {
                        bias = c->softness.biasRate * C;
                        massScale = c->softness.massScale;
                        impulseScale = c->softness.impulseScale;
                    }
                    float delta = SolveAxial(c, wA - wB, bias, massScale, impulseScale,
                                             &c->upperImpulse, true);
                    wA += world->invInertia[c->bodyA] * delta;
                    wB -= world->invInertia[c->bodyB] * delta;
                }
            }
            world->angularVelocities[c->bodyA] = wA;
            world->angularVelocities[c->bodyB] = wB;

            m2Vec2 bias = {0.0f, 0.0f};
            float massScale = 1.0f;
            float impulseScale = 0.0f;
            if (useBias)
            {
                m2Vec2 C = {c->baseCVec.x + ds.x, c->baseCVec.y + ds.y};
                bias.x = c->softness.massScale * c->softness.biasRate * C.x;
                bias.y = c->softness.massScale * c->softness.biasRate * C.y;
                massScale = c->softness.massScale;
                impulseScale = c->softness.impulseScale;
            }
            m2Vec2 vrA = {vA.x - wA * c->rA.y, vA.y + wA * c->rA.x};
            m2Vec2 vrB = {vB.x - wB * c->rB.y, vB.y + wB * c->rB.x};
            m2Vec2 cdot = {vrB.x - vrA.x, vrB.y - vrA.y};
            m2Vec2 b = {massScale * cdot.x + bias.x, massScale * cdot.y + bias.y};
            // Solve K * impulse = -b (2x2, guarded determinant: row-skip law).
            float det = c->k11 * c->k22 - c->k12 * c->k12;
            if (det <= 0.0f)
            {
                continue;
            }
            float invDet = 1.0f / det;
            m2Vec2 impulse = {-invDet * (c->k22 * b.x - c->k12 * b.y) - impulseScale * c->impulse.x,
                              -invDet * (c->k11 * b.y - c->k12 * b.x) -
                                  impulseScale * c->impulse.y};
            c->impulse.x += impulse.x;
            c->impulse.y += impulse.y;
            ApplyJointImpulse(world, c, impulse);
        }
        else if (c->type == 2)
        {
            float mA = world->invMass[c->bodyA];
            float iA = world->invInertia[c->bodyA];
            float mB = world->invMass[c->bodyB];
            float iB = world->invInertia[c->bodyB];
            float translation = c->baseC + c->axis.x * ds.x + c->axis.y * ds.y;

            if ((c->flags & 1u) != 0 && c->axialMass > 0.0f)
            {
                float cdot =
                    c->axis.x * (vB.x - vA.x) + c->axis.y * (vB.y - vA.y) + c->a2 * wB - c->a1 * wA;
                float delta =
                    SolveAxial(c, cdot - c->motorSpeed, 0.0f, 1.0f, 0.0f, &c->motorImpulse, false);
                vA.x -= mA * delta * c->axis.x;
                vA.y -= mA * delta * c->axis.y;
                wA -= iA * delta * c->a1;
                vB.x += mB * delta * c->axis.x;
                vB.y += mB * delta * c->axis.y;
                wB += iB * delta * c->a2;
            }
            if ((c->flags & 2u) != 0 && c->axialMass > 0.0f)
            {
                { // lower translation limit
                    float C = translation - c->lower;
                    float bias = 0.0f;
                    float massScale = 1.0f;
                    float impulseScale = 0.0f;
                    if (C > 0.0f)
                    {
                        bias = C * invH;
                    }
                    else if (useBias)
                    {
                        bias = c->softness.biasRate * C;
                        massScale = c->softness.massScale;
                        impulseScale = c->softness.impulseScale;
                    }
                    float cdot = c->axis.x * (vB.x - vA.x) + c->axis.y * (vB.y - vA.y) +
                                 c->a2 * wB - c->a1 * wA;
                    float delta =
                        SolveAxial(c, cdot, bias, massScale, impulseScale, &c->lowerImpulse, true);
                    vA.x -= mA * delta * c->axis.x;
                    vA.y -= mA * delta * c->axis.y;
                    wA -= iA * delta * c->a1;
                    vB.x += mB * delta * c->axis.x;
                    vB.y += mB * delta * c->axis.y;
                    wB += iB * delta * c->a2;
                }
                { // upper limit: signs flipped, impulse stays positive
                    float C = c->upper - translation;
                    float bias = 0.0f;
                    float massScale = 1.0f;
                    float impulseScale = 0.0f;
                    if (C > 0.0f)
                    {
                        bias = C * invH;
                    }
                    else if (useBias)
                    {
                        bias = c->softness.biasRate * C;
                        massScale = c->softness.massScale;
                        impulseScale = c->softness.impulseScale;
                    }
                    float cdot = c->axis.x * (vA.x - vB.x) + c->axis.y * (vA.y - vB.y) +
                                 c->a1 * wA - c->a2 * wB;
                    float delta =
                        SolveAxial(c, cdot, bias, massScale, impulseScale, &c->upperImpulse, true);
                    vA.x += mA * delta * c->axis.x;
                    vA.y += mA * delta * c->axis.y;
                    wA += iA * delta * c->a1;
                    vB.x -= mB * delta * c->axis.x;
                    vB.y -= mB * delta * c->axis.y;
                    wB -= iB * delta * c->a2;
                }
            }
            { // perpendicular + angle lock, block form (reference)
                float cdotPerp =
                    c->perp.x * (vB.x - vA.x) + c->perp.y * (vB.y - vA.y) + c->s2 * wB - c->s1 * wA;
                float cdotAngle = wB - wA;
                m2Vec2 bias = {0.0f, 0.0f};
                float massScale = 1.0f;
                float impulseScale = 0.0f;
                if (useBias)
                {
                    float perpC = c->baseCVec.x + c->perp.x * ds.x + c->perp.y * ds.y;
                    float angleC = m2UnwindAngle(c->baseAngle +
                                                 RelativeRotAngle(world->deltaRotations[c->bodyA],
                                                                  world->deltaRotations[c->bodyB]));
                    bias.x = c->softness.biasRate * perpC;
                    bias.y = c->softness.biasRate * angleC;
                    massScale = c->softness.massScale;
                    impulseScale = c->softness.impulseScale;
                }
                float det = c->k11 * c->k22 - c->k12 * c->k12;
                if (det > 0.0f)
                {
                    float invDet = 1.0f / det;
                    m2Vec2 b = {cdotPerp + bias.x, cdotAngle + bias.y};
                    m2Vec2 impulse = {-massScale * invDet * (c->k22 * b.x - c->k12 * b.y) -
                                          impulseScale * c->impulse.x,
                                      -massScale * invDet * (c->k11 * b.y - c->k12 * b.x) -
                                          impulseScale * c->impulse.y};
                    c->impulse.x += impulse.x;
                    c->impulse.y += impulse.y;
                    m2Vec2 P = {impulse.x * c->perp.x, impulse.x * c->perp.y};
                    float LA = impulse.x * c->s1 + impulse.y;
                    float LB = impulse.x * c->s2 + impulse.y;
                    vA.x -= mA * P.x;
                    vA.y -= mA * P.y;
                    wA -= iA * LA;
                    vB.x += mB * P.x;
                    vB.y += mB * P.y;
                    wB += iB * LB;
                }
            }
            world->linearVelocities[c->bodyA] = vA;
            world->angularVelocities[c->bodyA] = wA;
            world->linearVelocities[c->bodyB] = vB;
            world->angularVelocities[c->bodyB] = wB;
        }
        else
        {
            // Wheel (reference wheel_joint.c): rotational motor, real
            // suspension spring (biased even in relax), translation
            // limits, single perpendicular row. Rotation stays free.
            float mA = world->invMass[c->bodyA];
            float iA = world->invInertia[c->bodyA];
            float mB = world->invMass[c->bodyB];
            float iB = world->invInertia[c->bodyB];
            float translation = c->baseC + c->axis.x * ds.x + c->axis.y * ds.y;

            if ((c->flags & 1u) != 0 && c->k22 > 0.0f)
            {
                float cdot = wB - wA - c->motorSpeed;
                float unclamped = c->motorImpulse - c->k22 * cdot;
                float clamped = m2ClampF(unclamped, -c->maxMotorImpulse, c->maxMotorImpulse);
                float delta = clamped - c->motorImpulse;
                c->motorImpulse = clamped;
                wA -= iA * delta;
                wB += iB * delta;
            }
            if ((c->flags & 4u) != 0 && c->axialMass > 0.0f)
            {
                float C = translation;
                float bias = c->springSoftness.biasRate * C;
                float cdot =
                    c->axis.x * (vB.x - vA.x) + c->axis.y * (vB.y - vA.y) + c->a2 * wB - c->a1 * wA;
                float impulse = -c->springSoftness.massScale * c->axialMass * (cdot + bias) -
                                c->springSoftness.impulseScale * c->impulse.y;
                c->impulse.y += impulse;
                vA.x -= mA * impulse * c->axis.x;
                vA.y -= mA * impulse * c->axis.y;
                wA -= iA * impulse * c->a1;
                vB.x += mB * impulse * c->axis.x;
                vB.y += mB * impulse * c->axis.y;
                wB += iB * impulse * c->a2;
            }
            if ((c->flags & 2u) != 0 && c->axialMass > 0.0f)
            {
                { // lower travel stop
                    float C = translation - c->lower;
                    float bias = 0.0f;
                    float massScale = 1.0f;
                    float impulseScale = 0.0f;
                    if (C > 0.0f)
                    {
                        bias = C * invH;
                    }
                    else if (useBias)
                    {
                        bias = c->softness.biasRate * C;
                        massScale = c->softness.massScale;
                        impulseScale = c->softness.impulseScale;
                    }
                    float cdot = c->axis.x * (vB.x - vA.x) + c->axis.y * (vB.y - vA.y) +
                                 c->a2 * wB - c->a1 * wA;
                    float delta =
                        SolveAxial(c, cdot, bias, massScale, impulseScale, &c->lowerImpulse, true);
                    vA.x -= mA * delta * c->axis.x;
                    vA.y -= mA * delta * c->axis.y;
                    wA -= iA * delta * c->a1;
                    vB.x += mB * delta * c->axis.x;
                    vB.y += mB * delta * c->axis.y;
                    wB += iB * delta * c->a2;
                }
                { // upper travel stop, signs flipped
                    float C = c->upper - translation;
                    float bias = 0.0f;
                    float massScale = 1.0f;
                    float impulseScale = 0.0f;
                    if (C > 0.0f)
                    {
                        bias = C * invH;
                    }
                    else if (useBias)
                    {
                        bias = c->softness.biasRate * C;
                        massScale = c->softness.massScale;
                        impulseScale = c->softness.impulseScale;
                    }
                    float cdot = c->axis.x * (vA.x - vB.x) + c->axis.y * (vA.y - vB.y) +
                                 c->a1 * wA - c->a2 * wB;
                    float delta =
                        SolveAxial(c, cdot, bias, massScale, impulseScale, &c->upperImpulse, true);
                    vA.x += mA * delta * c->axis.x;
                    vA.y += mA * delta * c->axis.y;
                    wA += iA * delta * c->a1;
                    vB.x -= mB * delta * c->axis.x;
                    vB.y -= mB * delta * c->axis.y;
                    wB -= iB * delta * c->a2;
                }
            }
            if (c->k11 > 0.0f)
            { // point-to-line row
                float cdot =
                    c->perp.x * (vB.x - vA.x) + c->perp.y * (vB.y - vA.y) + c->s2 * wB - c->s1 * wA;
                float bias = 0.0f;
                float massScale = 1.0f;
                float impulseScale = 0.0f;
                if (useBias)
                {
                    float C = c->baseCVec.x + c->perp.x * ds.x + c->perp.y * ds.y;
                    bias = c->softness.biasRate * C;
                    massScale = c->softness.massScale;
                    impulseScale = c->softness.impulseScale;
                }
                float impulse = -massScale * c->k11 * (cdot + bias) - impulseScale * c->impulse.x;
                c->impulse.x += impulse;
                vA.x -= mA * impulse * c->perp.x;
                vA.y -= mA * impulse * c->perp.y;
                wA -= iA * impulse * c->s1;
                vB.x += mB * impulse * c->perp.x;
                vB.y += mB * impulse * c->perp.y;
                wB += iB * impulse * c->s2;
            }
            world->linearVelocities[c->bodyA] = vA;
            world->angularVelocities[c->bodyA] = wA;
            world->linearVelocities[c->bodyB] = vB;
            world->angularVelocities[c->bodyB] = wB;
        }
    }
}

static void StoreJointImpulses(m2World* world, m2JointConstraint* joints, int32_t count)
{
    for (int32_t i = 0; i < count; ++i)
    {
        int32_t j = joints[i].jointIndex;
        world->jointImpulse[j] = joints[i].impulse;
        world->jointMotorImpulse[j] = joints[i].motorImpulse;
        world->jointLowerImpulse[j] = joints[i].lowerImpulse;
        world->jointUpperImpulse[j] = joints[i].upperImpulse;
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
    m2JointConstraint* joints =
        (m2JointConstraint*)((uint8_t*)world->constraintScratch +
                             (size_t)world->pairCapacity * sizeof(m2ContactConstraint));
    int32_t jointCount = PrepareJoints(world, joints, h);

    int32_t colorStart[M2_GRAPH_COLORS + 2];
    ColorConstraints(world, constraints, constraintCount, colorStart);
    world->lastConstraintCount = constraintCount;
    world->lastOverflow = colorStart[M2_GRAPH_COLORS + 1] - colorStart[M2_GRAPH_COLORS];
    world->lastGraphColors = 0;
    for (int32_t c = 0; c < M2_GRAPH_COLORS; ++c)
    {
        world->lastGraphColors += colorStart[c + 1] > colorStart[c] ? 1 : 0;
    }

    // The push clamp is pre-divided by the static mass scale so the
    // later massScale multiply cannot weaken it (reference detail).
    m2Softness staticSoft = MakeSoft(2.0f * M2_CONTACT_HERTZ, M2_CONTACT_DAMPING_RATIO, h);
    float minBiasVel = -M2_CONTACT_PUSH_MAX_SPEED / staticSoft.massScale;

    int32_t blockStart[M2_GRAPH_COLORS + 1];
    PackContactBlocks(world, constraints, colorStart, blockStart);

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

        WarmStartJoints(world, joints, jointCount);
        RunContactStageWide(world, constraints, colorStart, blockStart, m2_stageWarmStart, invH,
                            minBiasVel, true);
        SolveJoints(world, joints, jointCount, true, invH); // joints before contacts (topic-05 §5)
        RunContactStageWide(world, constraints, colorStart, blockStart, m2_stageSolve, invH,
                            minBiasVel, true);

        // Bullet substep origins, captured before positions move.
        for (int32_t i = 0; i < world->maxBodyIndex; ++i)
        {
            if (world->bullets[i] != 0)
            {
                world->ccdPrevPositions[i] = world->transforms[i].p;
            }
        }

        // Integrate positions: f64 positions advance, f32 deltas track.
        for (int32_t i = 0; i < world->maxBodyIndex; ++i)
        {
            if (world->alive[i] == 0 || world->types[i] == (uint8_t)m2_staticBody ||
                world->asleep[i] != 0)
            {
                continue;
            }
            // The center of mass is what the velocity moves; the origin
            // swings around it. With the COM on the origin both extra
            // terms are exact zeros and the old bits fall out.
            m2Vec2 lc = world->localCenters[i];
            m2Vec2 rlcOld = Rotate(world->transforms[i].q, lc);
            world->transforms[i].p.x += (double)world->linearVelocities[i].x * (double)h;
            world->transforms[i].p.y += (double)world->linearVelocities[i].y * (double)h;
            world->deltaPositions[i].x += world->linearVelocities[i].x * h;
            world->deltaPositions[i].y += world->linearVelocities[i].y * h;
            m2Rot dq = m2MakeRot(world->angularVelocities[i] * h);
            world->transforms[i].q = m2MulRot(world->transforms[i].q, dq);
            world->deltaRotations[i] = m2MulRot(world->deltaRotations[i], dq);
            m2Vec2 rlcNew = Rotate(world->transforms[i].q, lc);
            world->transforms[i].p.x += (double)(rlcOld.x - rlcNew.x);
            world->transforms[i].p.y += (double)(rlcOld.y - rlcNew.y);
        }

        m2SolveContinuous(world); // the last transform-mutating pass (M13)
        SolveJoints(world, joints, jointCount, false, invH);
        RunContactStageWide(world, constraints, colorStart, blockStart, m2_stageSolve, invH,
                            minBiasVel, false);
    }

    RunContactStageWide(world, constraints, colorStart, blockStart, m2_stageRestitution, invH,
                        minBiasVel, false);
    RunContactStageWide(world, constraints, colorStart, blockStart, m2_stageStore, invH, minBiasVel,
                        false);
    StoreJointImpulses(world, joints, jointCount);
}
