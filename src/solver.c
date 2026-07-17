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

#include "simd.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <string.h>

#define M2_CONTACT_HERTZ          30.0f
#define M2_CONTACT_DAMPING_RATIO  10.0f
#define M2_CONTACT_PUSH_MAX_SPEED 3.0f
// Reference (b2DefaultWorldDef) default maximum linear speed, 400 m/s. It
// is a SAFETY bound, not a gameplay knob: an over-constrained or
// near-degenerate configuration is bounded here instead of pumping speed
// exponentially to infinity and then a NaN. Hardcoded like the angular cap
// (M2_PI quarter-turn) so the guard stays off the determinism-sensitive
// worldDef surface; 400 m/s leaves ample headroom over any real 2D motion.
#define M2_MAX_LINEAR_SPEED      400.0f
#define M2_RESTITUTION_THRESHOLD 1.0f

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
    m2Softness softness;       // constraint rows (stiff for wheel); weld linear
    m2Softness springSoftness; // wheel suspension (real spring)
    m2Softness softness2;      // weld angular row
    bool linearSpring;         // weld: nonzero hertz = biased even in relax
    bool angularSpring;
    m2Vec2 impulse;     // point/axial; prismatic (perp, angle); wheel (perp, spring)
    float motorImpulse; // motor accumulator; weld reuses it for the angle lock
    float lowerImpulse;
    float upperImpulse;
    float springImpulse; // revolute angular spring
} m2JointConstraint;

typedef struct m2ContactConstraint
{
    int32_t pairIndex;
    int32_t bodyA;
    int32_t bodyB;
    // Pair-effective masses: usually the bodies' own, but dominance
    // zeroes one side so the winner cannot be pushed in this pair.
    float invMassA;
    float invIA;
    float invMassB;
    float invIB;
    m2Vec2 normal; // world frame
    float friction;
    float restitution;
    float tangentSpeed; // conveyor: sum of both shapes (reference mixing)
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
        if (world->shapeSensor[shapeA] != 0 || world->shapeSensor[shapeB] != 0)
        {
            continue; // sensors observe, never push
        }
        int32_t bodyA = world->shapeBody[shapeA];
        int32_t bodyB = world->shapeBody[shapeB];
        float mA = world->invMass[bodyA];
        float iA = world->invInertia[bodyA];
        float mB = world->invMass[bodyB];
        float iB = world->invInertia[bodyB];
        // Dominance (contacts only): the higher side is unmovable in
        // this pair; statics outrank every dynamic by construction.
        int32_t domA = world->types[bodyA] == (uint8_t)m2_dynamicBody
                           ? (int32_t)world->dominances[bodyA]
                           : 128;
        int32_t domB = world->types[bodyB] == (uint8_t)m2_dynamicBody
                           ? (int32_t)world->dominances[bodyB]
                           : 128;
        if (domA > domB)
        {
            mA = 0.0f;
            iA = 0.0f;
        }
        else if (domB > domA)
        {
            mB = 0.0f;
            iB = 0.0f;
        }
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
        c->invMassA = mA;
        c->invIA = iA;
        c->invMassB = mB;
        c->invIB = iB;
        // Geometric-mean friction, max restitution (reference mixing).
        c->friction = sqrtf(world->shapeFriction[shapeA] * world->shapeFriction[shapeB]);
        c->tangentSpeed = world->shapeTangentSpeed[shapeA] + world->shapeTangentSpeed[shapeB];
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
    float mA = c->invMassA;
    float iA = c->invIA;
    float mB = c->invMassB;
    float iB = c->invIB;
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
        float mA = c->invMassA;
        float iA = c->invIA;
        float mB = c->invMassB;
        float iB = c->invIB;
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
                // Our tangent is the LEFT perp of the normal (the
                // reference uses the right perp), so the belt term
                // ADDS: positive tangentSpeed drives riders toward
                // +x on an upward-facing floor, the reference's
                // observable convention.
                float vt =
                    (vrB.x - vrA.x) * tangent.x + (vrB.y - vrA.y) * tangent.y + c->tangentSpeed;
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
        float mA = c->invMassA;
        float iA = c->invIA;
        float mB = c->invMassB;
        float iB = c->invIB;
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
    float tangentSpeed[M2_LANES];
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
                block->tangentSpeed[lane] = 0.0f;
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
            block->invMassA[lane] = c->invMassA;
            block->invIA[lane] = c->invIA;
            block->invMassB[lane] = c->invMassB;
            block->invIB[lane] = c->invIB;
            block->normalX[lane] = c->normal.x;
            block->normalY[lane] = c->normal.y;
            block->friction[lane] = c->friction;
            block->tangentSpeed[lane] = c->tangentSpeed;
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
    // Vector body (simd.h bit law): tangent = (-ny, nx), P = nImp*n +
    // tImp*t, velocities pick up the usual +-invMass * P terms.
    m2f8 nX = m2F8Load(b->normalX);
    m2f8 nY = m2F8Load(b->normalY);
    m2f8 imA = m2F8Load(b->invMassA);
    m2f8 iiA = m2F8Load(b->invIA);
    m2f8 imB = m2F8Load(b->invMassB);
    m2f8 iiB = m2F8Load(b->invIB);
    m2f8 avAx = m2F8Load(vAx);
    m2f8 avAy = m2F8Load(vAy);
    m2f8 awA = m2F8Load(wA);
    m2f8 avBx = m2F8Load(vBx);
    m2f8 avBy = m2F8Load(vBy);
    m2f8 awB = m2F8Load(wB);
    for (int32_t k = 0; k < b->pointCount; ++k)
    {
        m2f8 nImp = m2F8Load(b->normalImp[k]);
        m2f8 tImp = m2F8Load(b->tangentImp[k]);
        m2f8 rAXk = m2F8Load(b->rAX[k]);
        m2f8 rAYk = m2F8Load(b->rAY[k]);
        m2f8 rBXk = m2F8Load(b->rBX[k]);
        m2f8 rBYk = m2F8Load(b->rBY[k]);
        m2f8 Px = m2F8NegMulAdd(tImp, nY, m2F8Mul(nImp, nX));
        m2f8 Py = m2F8MulAdd(tImp, nX, m2F8Mul(nImp, nY));
        avAx = m2F8NegMulAdd(imA, Px, avAx);
        avAy = m2F8NegMulAdd(imA, Py, avAy);
        m2f8 crossA = m2F8NegMulAdd(rAYk, Px, m2F8Mul(rAXk, Py));
        awA = m2F8NegMulAdd(iiA, crossA, awA);
        avBx = m2F8MulAdd(imB, Px, avBx);
        avBy = m2F8MulAdd(imB, Py, avBy);
        m2f8 crossB = m2F8NegMulAdd(rBYk, Px, m2F8Mul(rBXk, Py));
        awB = m2F8MulAdd(iiB, crossB, awB);
    }
    m2F8Store(vAx, avAx);
    m2F8Store(vAy, avAy);
    m2F8Store(wA, awA);
    m2F8Store(vBx, avBx);
    m2F8Store(vBy, avBy);
    m2F8Store(wB, awB);
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

    // Vector body (simd.h bit law). The selects mirror the scalar
    // ternaries exactly; useBias is uniform per call, so its branch is
    // resolved once outside the lane math.
    m2f8 nX = m2F8Load(b->normalX);
    m2f8 nY = m2F8Load(b->normalY);
    m2f8 imA = m2F8Load(b->invMassA);
    m2f8 iiA = m2F8Load(b->invIA);
    m2f8 imB = m2F8Load(b->invMassB);
    m2f8 iiB = m2F8Load(b->invIB);
    m2f8 avAx = m2F8Load(vAx);
    m2f8 avAy = m2F8Load(vAy);
    m2f8 awA = m2F8Load(wA);
    m2f8 avBx = m2F8Load(vBx);
    m2f8 avBy = m2F8Load(vBy);
    m2f8 awB = m2F8Load(wB);
    m2f8 vdpx = m2F8Load(dpx);
    m2f8 vdpy = m2F8Load(dpy);
    m2f8 vqAc = m2F8Load(dqAc);
    m2f8 vqAs = m2F8Load(dqAs);
    m2f8 vqBc = m2F8Load(dqBc);
    m2f8 vqBs = m2F8Load(dqBs);
    m2f8 zero = m2F8Zero();
    m2f8 one = m2F8Set1(1.0f);
    m2f8 vInvH = m2F8Set1(invH);
    m2f8 vMinBias = m2F8Set1(minBiasVel);
    m2f8 biasRate = m2F8Load(b->biasRate);
    m2f8 msIn = useBias ? m2F8Load(b->massScale) : one;
    m2f8 isIn = useBias ? m2F8Load(b->impulseScale) : zero;

    for (int32_t k = 0; k < b->pointCount; ++k)
    {
        m2f8 rAXk = m2F8Load(b->rAX[k]);
        m2f8 rAYk = m2F8Load(b->rAY[k]);
        m2f8 rBXk = m2F8Load(b->rBX[k]);
        m2f8 rBYk = m2F8Load(b->rBY[k]);
        m2f8 rsAx = m2F8NegMulAdd(vqAs, rAYk, m2F8Mul(vqAc, rAXk));
        m2f8 rsAy = m2F8MulAdd(vqAs, rAXk, m2F8Mul(vqAc, rAYk));
        m2f8 rsBx = m2F8NegMulAdd(vqBs, rBYk, m2F8Mul(vqBc, rBXk));
        m2f8 rsBy = m2F8MulAdd(vqBs, rBXk, m2F8Mul(vqBc, rBYk));
        m2f8 dsx = m2F8Add(vdpx, m2F8Sub(rsBx, rsAx));
        m2f8 dsy = m2F8Add(vdpy, m2F8Sub(rsBy, rsAy));
        m2f8 sep = m2F8MulAdd(dsy, nY, m2F8MulAdd(dsx, nX, m2F8Load(b->baseSep[k])));

        m2f8 spec = m2F8GT(sep, zero);
        m2f8 softBias = m2F8Max(m2F8Mul(biasRate, sep), vMinBias);
        m2f8 nonSpecBias = useBias ? softBias : zero;
        m2f8 bias = m2F8Select(spec, m2F8Mul(sep, vInvH), nonSpecBias);
        m2f8 massScale = m2F8Select(spec, one, msIn);
        m2f8 impulseScale = m2F8Select(spec, zero, isIn);

        m2f8 vrAx = m2F8NegMulAdd(awA, rAYk, avAx);
        m2f8 vrAy = m2F8MulAdd(awA, rAXk, avAy);
        m2f8 vrBx = m2F8NegMulAdd(awB, rBYk, avBx);
        m2f8 vrBy = m2F8MulAdd(awB, rBXk, avBy);
        m2f8 vn = m2F8MulAdd(m2F8Sub(vrBy, vrAy), nY, m2F8Mul(m2F8Sub(vrBx, vrAx), nX));

        m2f8 nImp = m2F8Load(b->normalImp[k]);
        m2f8 scaledMass = m2F8Mul(m2F8Load(b->normalMass[k]), massScale);
        m2f8 drag = m2F8Mul(impulseScale, nImp);
        m2f8 impulse = m2F8NegMulAdd(scaledMass, m2F8Add(vn, bias), m2F8Neg(drag));
        m2f8 newImpulse = m2F8Max(m2F8Add(nImp, impulse), zero);
        impulse = m2F8Sub(newImpulse, nImp);
        m2F8Store(b->normalImp[k], newImpulse);

        m2f8 Px = m2F8Mul(impulse, nX);
        m2f8 Py = m2F8Mul(impulse, nY);
        avAx = m2F8NegMulAdd(imA, Px, avAx);
        avAy = m2F8NegMulAdd(imA, Py, avAy);
        awA = m2F8NegMulAdd(iiA, m2F8NegMulAdd(rAYk, Px, m2F8Mul(rAXk, Py)), awA);
        avBx = m2F8MulAdd(imB, Px, avBx);
        avBy = m2F8MulAdd(imB, Py, avBy);
        awB = m2F8MulAdd(iiB, m2F8NegMulAdd(rBYk, Px, m2F8Mul(rBXk, Py)), awB);
    }

    for (int32_t k = 0; k < b->pointCount; ++k)
    {
        m2f8 rAXk = m2F8Load(b->rAX[k]);
        m2f8 rAYk = m2F8Load(b->rAY[k]);
        m2f8 rBXk = m2F8Load(b->rBX[k]);
        m2f8 rBYk = m2F8Load(b->rBY[k]);
        // tangent = (-nY, nX)
        m2f8 vrAx = m2F8NegMulAdd(awA, rAYk, avAx);
        m2f8 vrAy = m2F8MulAdd(awA, rAXk, avAy);
        m2f8 vrBx = m2F8NegMulAdd(awB, rBYk, avBx);
        m2f8 vrBy = m2F8MulAdd(awB, rBXk, avBy);
        m2f8 vt = m2F8MulAdd(m2F8Sub(vrBy, vrAy), nX, m2F8Mul(m2F8Sub(vrBx, vrAx), m2F8Neg(nY)));
        vt = m2F8Add(vt, m2F8Load(b->tangentSpeed)); // left-perp tangent: belt term adds
        m2f8 tImp = m2F8Load(b->tangentImp[k]);
        m2f8 impulse = m2F8Neg(m2F8Mul(m2F8Load(b->tangentMass[k]), vt));
        m2f8 maxFriction = m2F8Mul(m2F8Load(b->friction), m2F8Load(b->normalImp[k]));
        m2f8 newImpulse =
            m2F8Min(m2F8Max(m2F8Add(tImp, impulse), m2F8Neg(maxFriction)), maxFriction);
        impulse = m2F8Sub(newImpulse, tImp);
        m2F8Store(b->tangentImp[k], newImpulse);

        m2f8 Px = m2F8Mul(impulse, m2F8Neg(nY));
        m2f8 Py = m2F8Mul(impulse, nX);
        avAx = m2F8NegMulAdd(imA, Px, avAx);
        avAy = m2F8NegMulAdd(imA, Py, avAy);
        awA = m2F8NegMulAdd(iiA, m2F8NegMulAdd(rAYk, Px, m2F8Mul(rAXk, Py)), awA);
        avBx = m2F8MulAdd(imB, Px, avBx);
        avBy = m2F8MulAdd(imB, Py, avBy);
        awB = m2F8MulAdd(iiB, m2F8NegMulAdd(rBYk, Px, m2F8Mul(rBXk, Py)), awB);
    }
    m2F8Store(vAx, avAx);
    m2F8Store(vAy, avAy);
    m2F8Store(wA, awA);
    m2F8Store(vBx, avBx);
    m2F8Store(vBy, avBy);
    m2F8Store(wB, awB);

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
        m2RunParallel(world, BlockStageRange, &ctx, end - begin, 1);
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
        if (world->jointType[j] == 5)
        {
            continue; // filter joints have no rows at all
        }
        if (world->jointType[j] == 7 &&
            (world->types[world->jointBodyB[j]] != (uint8_t)m2_dynamicBody ||
             world->asleep[world->jointBodyB[j]] != 0))
        {
            continue; // a mouse joint only ever moves body B
        }
        if (world->disabled[world->jointBodyA[j]] != 0 ||
            world->disabled[world->jointBodyB[j]] != 0)
        {
            continue; // a dormant end pauses the whole joint
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
        c->springImpulse = world->jointSpringImpulse[j];

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
            // The rod's spawn-time length rides the (unused) motor
            // speed slot so the limit rows can track absolute length;
            // flag bit3 marks an active hard range.
            c->motorSpeed = length;
            if (world->jointLower[j] > 0.0f || world->jointUpper[j] < 3.0e38f)
            {
                c->flags |= 8;
                // Hard stops stay hard even on a soft rope: the limit
                // rows run on the stiff default, reference-style.
                c->springSoftness = MakeSoft(60.0f, 2.0f, h);
            }
            if ((c->flags & 16u) != 0 && world->jointHertz[j] == 0.0f)
            {
                // enableSpring with zero stiffness is a free rope or rod:
                // skip the rest-length row so only the limits act, and
                // drop any rest impulse so warm start carries nothing.
                c->flags |= 32u;
                c->impulse.x = 0.0f;
            }
            float crA = Cross(c->rA, c->axis);
            float crB = Cross(c->rB, c->axis);
            float k = mA + mB + iA * crA * crA + iB * crB * crB;
            c->axialMass = k > 0.0f ? 1.0f / k : 0.0f;
        }
        else if (c->type == 1 || c->type == 3 || c->type == 6)
        {
            c->baseCVec = (m2Vec2){dx, dy};
            c->k11 = mA + mB + iA * c->rA.y * c->rA.y + iB * c->rB.y * c->rB.y;
            c->k12 = -iA * c->rA.x * c->rA.y - iB * c->rB.x * c->rB.y;
            c->k22 = mA + mB + iA * c->rA.x * c->rA.x + iB * c->rB.x * c->rB.x;
            if (c->type == 6)
            {
                // Motor: separations are measured against the offsets.
                // linearOffset lives in A's frame (the documented
                // contract; the reference's code disagrees with its own
                // comment and we side with the comment).
                m2Vec2 offset = Rotate(qA, world->jointLocalAxisA[j]);
                c->baseCVec = (m2Vec2){dx - offset.x, dy - offset.y};
                c->baseAngle = m2UnwindAngle(RelativeRotAngle(qA, qB) - world->jointRefAngle[j]);
                float ki = iA + iB;
                c->axialMass = ki > 0.0f ? 1.0f / ki : 0.0f;
                // correctionFactor rides the motorSpeed slot into the
                // solve; the linear budget rides lower.
                c->motorSpeed = world->jointDamping[j];
                c->lower = h * world->jointLength[j];
                if (world->jointHertz2[j] > 0.0f)
                {
                    // Spring drive: a soft softness replaces the hard
                    // correctionFactor bias in both rows (flag bit 32).
                    c->flags |= 32u;
                    c->softness = MakeSoft(world->jointHertz2[j], world->jointDamping2[j], h);
                }
            }
            float k = iA + iB;
            c->axialMass = k > 0.0f ? 1.0f / k : 0.0f;
            c->baseAngle = m2UnwindAngle(RelativeRotAngle(qA, qB) - world->jointRefAngle[j]);
            c->linearSpring = c->type == 3 && world->jointHertz[j] > 0.0f;
            c->angularSpring = (c->type == 1 || c->type == 3) && world->jointHertz2[j] > 0.0f;
            c->softness2 = c->angularSpring
                               ? MakeSoft(world->jointHertz2[j], world->jointDamping2[j], h)
                               : MakeSoft(60.0f, 2.0f, h);
        }
        else if (c->type == 8)
        {
            // Gear: accumulate the phase by how far each body actually
            // rotated since last prepare (per-step deltas stay far from
            // the wrap, so many full turns remain exact), then couple
            // the spins. ratio rides jointLength -> loaded fields.
            float ratio = world->jointLength[j];
            m2Rot prevA = {world->jointLocalAnchorA[j].x, world->jointLocalAnchorA[j].y};
            m2Rot prevB = {world->jointLocalAnchorB[j].x, world->jointLocalAnchorB[j].y};
            float phase = world->jointRefAngle[j];
            phase += ratio * RelativeRotAngle(prevA, qA) + RelativeRotAngle(prevB, qB);
            world->jointRefAngle[j] = phase;
            world->jointLocalAnchorA[j] = (m2Vec2){qA.c, qA.s};
            world->jointLocalAnchorB[j] = (m2Vec2){qB.c, qB.s};
            c->baseAngle = phase;
            c->motorSpeed = ratio; // carried into the solve
            float k = ratio * ratio * iA + iB;
            c->axialMass = k > 0.0f ? 1.0f / k : 0.0f;
        }
        else if (c->type == 10)
        {
            // Ratchet (Chipmunk cpRatchetJoint reconciliation): track
            // the relative angle multi-turn exact via previous-rotation
            // slots, click the engaged tooth forward when the angle
            // passes it, and hold a one-sided row against back-spin.
            float ratchet = world->jointLength[j];
            float phase = world->jointRefAngle[j];
            m2Rot prevA = {world->jointLocalAnchorA[j].x, world->jointLocalAnchorA[j].y};
            m2Rot prevB = {world->jointLocalAnchorB[j].x, world->jointLocalAnchorB[j].y};
            float angle = world->jointUpper[j];
            angle += RelativeRotAngle(prevB, qB) - RelativeRotAngle(prevA, qA);
            world->jointUpper[j] = angle;
            world->jointLocalAnchorA[j] = (m2Vec2){qA.c, qA.s};
            world->jointLocalAnchorB[j] = (m2Vec2){qB.c, qB.s};
            float engaged = world->jointLower[j];
            float diff = engaged - angle;
            if (!(diff * ratchet > 0.0f))
            {
                // Free direction: click to the tooth at or behind us.
                engaged = floorf((angle - phase) / ratchet) * ratchet + phase;
                world->jointLower[j] = engaged;
            }
            c->baseAngle = angle - engaged; // C0, sign-adjusted in solve
            c->motorSpeed = ratchet;        // carries the free direction
            float k = iA + iB;
            c->axialMass = k > 0.0f ? 1.0f / k : 0.0f;
        }
        else if (c->type == 9)
        {
            // Pulley (b2 v2.4 reconciliation): equality constraint
            // C = total - lengthA - ratio * lengthB with unit rope
            // directions frozen at prepare. uA rides the axis slot,
            // uB the perp slot, ratio the motorSpeed slot. A side
            // shorter than 10 slop goes limp (zero direction), the
            // reference's slack guard.
            float ratio = world->jointLength[j];
            m2Vec2 armA = Rotate(qA, world->jointLocalAnchorA[j]);
            m2Vec2 armB = Rotate(qB, world->jointLocalAnchorB[j]);
            m2Vec2 uA = {(float)(world->transforms[bodyA].p.x - world->jointTargets[j].x) + armA.x,
                         (float)(world->transforms[bodyA].p.y - world->jointTargets[j].y) + armA.y};
            m2Vec2 uB = {(float)(world->transforms[bodyB].p.x - world->jointTargetsB[j].x) + armB.x,
                         (float)(world->transforms[bodyB].p.y - world->jointTargetsB[j].y) +
                             armB.y};
            float lengthA = sqrtf(uA.x * uA.x + uA.y * uA.y);
            float lengthB = sqrtf(uB.x * uB.x + uB.y * uB.y);
            c->axis =
                lengthA > 0.05f ? (m2Vec2){uA.x / lengthA, uA.y / lengthA} : (m2Vec2){0.0f, 0.0f};
            c->perp =
                lengthB > 0.05f ? (m2Vec2){uB.x / lengthB, uB.y / lengthB} : (m2Vec2){0.0f, 0.0f};
            c->baseC = world->jointRefAngle[j] - lengthA - ratio * lengthB;
            c->motorSpeed = ratio; // carried into the solve
            float ruA = Cross(c->rA, c->axis);
            float ruB = Cross(c->rB, c->perp);
            float k = mA + iA * ruA * ruA + ratio * ratio * (mB + iB * ruB * ruB);
            c->axialMass = k > 0.0f ? 1.0f / k : 0.0f;
        }
        else if (c->type == 7)
        {
            // Mouse: only body B has rows. Grab arm rB comes from the
            // generic anchors; the target separation base is the f64
            // crossing between B's center and the stored target.
            c->k11 = mB + iB * c->rB.y * c->rB.y;
            c->k12 = -iB * c->rB.x * c->rB.y;
            c->k22 = mB + iB * c->rB.x * c->rB.x;
            m2Vec2 comB2 = Rotate(qB, lcB);
            float cbx = (float)(world->transforms[bodyB].p.x - world->jointTargets[j].x);
            float cby = (float)(world->transforms[bodyB].p.y - world->jointTargets[j].y);
            c->baseCVec = (m2Vec2){cbx + comB2.x, cby + comB2.y};
            c->softness2 = MakeSoft(0.5f, 0.1f, h); // reference spin damper
            c->lower = h * world->jointLength[j];   // force budget
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
            float axial = (c->flags & 8) != 0 ? c->impulse.x + c->lowerImpulse - c->upperImpulse
                                              : c->impulse.x;
            ApplyJointImpulse(world, c, (m2Vec2){axial * c->axis.x, axial * c->axis.y});
        }
        else if (c->type == 1 || c->type == 3 || c->type == 6)
        {
            // Weld's angle-lock and the motor joint's torque both ride
            // the motor slot; limits are zero where unused. The
            // revolute angular spring joins the axial sum.
            ApplyJointImpulse(world, c, c->impulse);
            float axial = c->springImpulse + c->motorImpulse + c->lowerImpulse - c->upperImpulse;
            world->angularVelocities[c->bodyA] -= world->invInertia[c->bodyA] * axial;
            world->angularVelocities[c->bodyB] += world->invInertia[c->bodyB] * axial;
        }
        else if (c->type == 8)
        {
            // Gear: one angular impulse, ratio-weighted on side A.
            float L = c->impulse.x;
            world->angularVelocities[c->bodyA] += world->invInertia[c->bodyA] * (c->motorSpeed * L);
            world->angularVelocities[c->bodyB] += world->invInertia[c->bodyB] * L;
        }
        else if (c->type == 10)
        {
            // Ratchet: one-sided angular impulse in the hold direction.
            float s = c->motorSpeed > 0.0f ? 1.0f : -1.0f;
            float L = s * c->impulse.x;
            world->angularVelocities[c->bodyA] -= world->invInertia[c->bodyA] * L;
            world->angularVelocities[c->bodyB] += world->invInertia[c->bodyB] * L;
        }
        else if (c->type == 9)
        {
            // Pulley: PA = -L*uA on A, PB = -ratio*L*uB on B.
            float L = c->impulse.x;
            m2Vec2 PA = {-L * c->axis.x, -L * c->axis.y};
            m2Vec2 PB = {-c->motorSpeed * L * c->perp.x, -c->motorSpeed * L * c->perp.y};
            float mA = world->invMass[c->bodyA];
            float iA = world->invInertia[c->bodyA];
            float mB = world->invMass[c->bodyB];
            float iB = world->invInertia[c->bodyB];
            world->linearVelocities[c->bodyA].x += mA * PA.x;
            world->linearVelocities[c->bodyA].y += mA * PA.y;
            world->angularVelocities[c->bodyA] += iA * Cross(c->rA, PA);
            world->linearVelocities[c->bodyB].x += mB * PB.x;
            world->linearVelocities[c->bodyB].y += mB * PB.y;
            world->angularVelocities[c->bodyB] += iB * Cross(c->rB, PB);
        }
        else if (c->type == 7)
        {
            // Mouse: body B only.
            int32_t bodyB = c->bodyB;
            float mB = world->invMass[bodyB];
            float iB = world->invInertia[bodyB];
            world->linearVelocities[bodyB].x += mB * c->impulse.x;
            world->linearVelocities[bodyB].y += mB * c->impulse.y;
            world->angularVelocities[bodyB] += iB * (Cross(c->rB, c->impulse) + c->motorImpulse);
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
            if ((c->flags & 32u) == 0)
            {
                // Rest-length row (skipped for a free rope or rod).
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
                float impulse =
                    -c->axialMass * (massScale * cdot + bias) - impulseScale * c->impulse.x;
                c->impulse.x += impulse;
                ApplyJointImpulse(world, c, (m2Vec2){impulse * c->axis.x, impulse * c->axis.y});
            }

            if ((c->flags & 8) != 0)
            {
                // Hard length range (reference distance_joint.c): one
                // one-sided row per bound on the shared axis, reading
                // fresh world velocities after each apply.
                float lengthNow = c->motorSpeed + ds.x * c->axis.x + ds.y * c->axis.y;
                {
                    m2Vec2 vA2 = world->linearVelocities[c->bodyA];
                    float wA2 = world->angularVelocities[c->bodyA];
                    m2Vec2 vB2 = world->linearVelocities[c->bodyB];
                    float wB2 = world->angularVelocities[c->bodyB];
                    m2Vec2 vrA2 = {vA2.x - wA2 * c->rA.y, vA2.y + wA2 * c->rA.x};
                    m2Vec2 vrB2 = {vB2.x - wB2 * c->rB.y, vB2.y + wB2 * c->rB.x};
                    float cdotL = (vrB2.x - vrA2.x) * c->axis.x + (vrB2.y - vrA2.y) * c->axis.y;
                    float C = lengthNow - c->lower;
                    float biasL = 0.0f;
                    float massL = 1.0f;
                    float scaleL = 0.0f;
                    if (C > 0.0f)
                    {
                        biasL = C * invH; // speculative
                    }
                    else if (useBias)
                    {
                        biasL = c->springSoftness.biasRate * C;
                        massL = c->springSoftness.massScale;
                        scaleL = c->springSoftness.impulseScale;
                    }
                    float imp = -massL * c->axialMass * (cdotL + biasL) - scaleL * c->lowerImpulse;
                    float next = c->lowerImpulse + imp;
                    next = next > 0.0f ? next : 0.0f;
                    imp = next - c->lowerImpulse;
                    c->lowerImpulse = next;
                    ApplyJointImpulse(world, c, (m2Vec2){imp * c->axis.x, imp * c->axis.y});
                }
                {
                    m2Vec2 vA2 = world->linearVelocities[c->bodyA];
                    float wA2 = world->angularVelocities[c->bodyA];
                    m2Vec2 vB2 = world->linearVelocities[c->bodyB];
                    float wB2 = world->angularVelocities[c->bodyB];
                    m2Vec2 vrA2 = {vA2.x - wA2 * c->rA.y, vA2.y + wA2 * c->rA.x};
                    m2Vec2 vrB2 = {vB2.x - wB2 * c->rB.y, vB2.y + wB2 * c->rB.x};
                    // Upper bound: the constraint runs the other way.
                    float cdotU = (vrA2.x - vrB2.x) * c->axis.x + (vrA2.y - vrB2.y) * c->axis.y;
                    float C = c->upper - lengthNow;
                    float biasU = 0.0f;
                    float massU = 1.0f;
                    float scaleU = 0.0f;
                    if (C > 0.0f)
                    {
                        biasU = C * invH;
                    }
                    else if (useBias)
                    {
                        biasU = c->springSoftness.biasRate * C;
                        massU = c->springSoftness.massScale;
                        scaleU = c->springSoftness.impulseScale;
                    }
                    float imp = -massU * c->axialMass * (cdotU + biasU) - scaleU * c->upperImpulse;
                    float next = c->upperImpulse + imp;
                    next = next > 0.0f ? next : 0.0f;
                    imp = next - c->upperImpulse;
                    c->upperImpulse = next;
                    ApplyJointImpulse(world, c, (m2Vec2){-imp * c->axis.x, -imp * c->axis.y});
                }
            }
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
                // Nonzero angular hertz = a real spring: biased even
                // during relax (reference semantics).
                if (useBias || c->angularSpring)
                {
                    float C = m2UnwindAngle(c->baseAngle +
                                            RelativeRotAngle(world->deltaRotations[c->bodyA],
                                                             world->deltaRotations[c->bodyB]));
                    bias = c->softness2.biasRate * C;
                    massScale = c->softness2.massScale;
                    impulseScale = c->softness2.impulseScale;
                }
                float cdot = wB - wA;
                float impulse =
                    -massScale * c->axialMass * (cdot + bias) - impulseScale * c->motorImpulse;
                c->motorImpulse += impulse;
                wA -= world->invInertia[c->bodyA] * impulse;
                wB += world->invInertia[c->bodyB] * impulse;
            }
            if (c->type == 1 && c->angularSpring && c->axialMass > 0.0f)
            {
                // Revolute angular spring (reference revolute_joint.c):
                // a real spring toward the creation angle, biased in
                // every pass by definition.
                float C =
                    m2UnwindAngle(c->baseAngle + RelativeRotAngle(world->deltaRotations[c->bodyA],
                                                                  world->deltaRotations[c->bodyB]));
                float bias = c->softness2.biasRate * C;
                float massScale = c->softness2.massScale;
                float impulseScale = c->softness2.impulseScale;
                float cdot = wB - wA;
                float impulse =
                    -massScale * c->axialMass * (cdot + bias) - impulseScale * c->springImpulse;
                c->springImpulse += impulse;
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
            if (useBias || (c->type == 3 && c->linearSpring))
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
        else if (c->type == 6)
        {
            // Motor joint (reference solve, always biased): angular row
            // first, then the clamped linear block. correctionFactor
            // rides the motorSpeed slot, the force budget rides lower.
            float mA = world->invMass[c->bodyA];
            float iA = world->invInertia[c->bodyA];
            float mB = world->invMass[c->bodyB];
            float iB = world->invInertia[c->bodyB];
            bool motorSpring = (c->flags & 32u) != 0;
            {
                float angC =
                    m2UnwindAngle(c->baseAngle + RelativeRotAngle(world->deltaRotations[c->bodyA],
                                                                  world->deltaRotations[c->bodyB]));
                float impulse;
                if (motorSpring)
                {
                    // Soft drive: the spring biasRate replaces the hard
                    // correctionFactor, with its mass and impulse scales.
                    float angBias = c->softness.biasRate * angC;
                    impulse = -c->softness.massScale * c->axialMass * ((wB - wA) + angBias) -
                              c->softness.impulseScale * c->motorImpulse;
                }
                else
                {
                    float angBias = invH * c->motorSpeed * angC;
                    impulse = -c->axialMass * ((wB - wA) + angBias);
                }
                float old = c->motorImpulse;
                c->motorImpulse = m2ClampF(old + impulse, -c->maxMotorImpulse, c->maxMotorImpulse);
                impulse = c->motorImpulse - old;
                wA -= iA * impulse;
                wB += iB * impulse;
            }
            {
                m2Vec2 sep = {c->baseCVec.x + ds.x, c->baseCVec.y + ds.y};
                float biasMul = motorSpring ? c->softness.biasRate : invH * c->motorSpeed;
                m2Vec2 bias = {biasMul * sep.x, biasMul * sep.y};
                m2Vec2 vrA = {vA.x - wA * c->rA.y, vA.y + wA * c->rA.x};
                m2Vec2 vrB = {vB.x - wB * c->rB.y, vB.y + wB * c->rB.x};
                m2Vec2 cdot = {vrB.x - vrA.x + bias.x, vrB.y - vrA.y + bias.y};
                // Solve K impulse = -cdot with the 2x2 from prepare.
                float det = c->k11 * c->k22 - c->k12 * c->k12;
                float invDet = det != 0.0f ? 1.0f / det : 0.0f;
                m2Vec2 raw = {-invDet * (c->k22 * cdot.x - c->k12 * cdot.y),
                              -invDet * (c->k11 * cdot.y - c->k12 * cdot.x)};
                m2Vec2 impulse;
                if (motorSpring)
                {
                    impulse.x =
                        c->softness.massScale * raw.x - c->softness.impulseScale * c->impulse.x;
                    impulse.y =
                        c->softness.massScale * raw.y - c->softness.impulseScale * c->impulse.y;
                }
                else
                {
                    impulse = raw;
                }
                m2Vec2 old = c->impulse;
                c->impulse.x += impulse.x;
                c->impulse.y += impulse.y;
                float budget = c->lower;
                float mag2 = c->impulse.x * c->impulse.x + c->impulse.y * c->impulse.y;
                if (mag2 > budget * budget)
                {
                    float mag = sqrtf(mag2);
                    float scale = mag > 0.0f ? budget / mag : 0.0f;
                    c->impulse.x *= scale;
                    c->impulse.y *= scale;
                }
                impulse.x = c->impulse.x - old.x;
                impulse.y = c->impulse.y - old.y;
                vA.x -= mA * impulse.x;
                vA.y -= mA * impulse.y;
                wA -= iA * Cross(c->rA, impulse);
                vB.x += mB * impulse.x;
                vB.y += mB * impulse.y;
                wB += iB * Cross(c->rB, impulse);
            }
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
        else if (c->type == 8)
        {
            // Gear: C = ratio*angleA + angleB - phase0, tracked through
            // the substep delta rotations; stiff-biased like the weld
            // angle row.
            float ratio = c->motorSpeed;
            float iA = world->invInertia[c->bodyA];
            float iB = world->invInertia[c->bodyB];
            float bias = 0.0f;
            float massScale = 1.0f;
            float impulseScale = 0.0f;
            if (useBias)
            {
                float dA =
                    m2Atan2(world->deltaRotations[c->bodyA].s, world->deltaRotations[c->bodyA].c);
                float dB =
                    m2Atan2(world->deltaRotations[c->bodyB].s, world->deltaRotations[c->bodyB].c);
                float C = c->baseAngle + ratio * dA + dB;
                bias = c->softness.biasRate * C;
                massScale = c->softness.massScale;
                impulseScale = c->softness.impulseScale;
            }
            float cdot = ratio * wA + wB;
            float impulse = -c->axialMass * (massScale * cdot + bias) - impulseScale * c->impulse.x;
            c->impulse.x += impulse;
            world->angularVelocities[c->bodyA] = wA + iA * (ratio * impulse);
            world->angularVelocities[c->bodyB] = wB + iB * impulse;
        }
        else if (c->type == 10)
        {
            // Ratchet: the revolute limit row, sign-folded so the free
            // direction never feels it: C' = s*(angle - engaged) >= 0,
            // speculative when open, stiff-soft when violated.
            float s = c->motorSpeed > 0.0f ? 1.0f : -1.0f;
            float iA = world->invInertia[c->bodyA];
            float iB = world->invInertia[c->bodyB];
            float angleNow = c->baseAngle + RelativeRotAngle(world->deltaRotations[c->bodyA],
                                                             world->deltaRotations[c->bodyB]);
            float C = s * angleNow;
            float bias = 0.0f;
            float massScale = 1.0f;
            float impulseScale = 0.0f;
            if (C > 0.0f)
            {
                bias = C * invH; // speculative: stop exactly at the tooth
            }
            else if (useBias)
            {
                bias = c->softness.biasRate * C;
                massScale = c->softness.massScale;
                impulseScale = c->softness.impulseScale;
            }
            float cdot = s * (wB - wA);
            float impulse = -massScale * c->axialMass * (cdot + bias) - impulseScale * c->impulse.x;
            float next = c->impulse.x + impulse;
            next = next > 0.0f ? next : 0.0f;
            impulse = next - c->impulse.x;
            c->impulse.x = next;
            world->angularVelocities[c->bodyA] = wA - iA * (s * impulse);
            world->angularVelocities[c->bodyB] = wB + iB * (s * impulse);
        }
        else if (c->type == 9)
        {
            // Pulley: Cdot = -dot(uA, vpA) - ratio * dot(uB, vpB),
            // stiff-biased with C tracked through per-body position
            // deltas against the frozen rope directions.
            float ratio = c->motorSpeed;
            float mA = world->invMass[c->bodyA];
            float iA = world->invInertia[c->bodyA];
            float mB = world->invMass[c->bodyB];
            float iB = world->invInertia[c->bodyB];
            float bias = 0.0f;
            float massScale = 1.0f;
            float impulseScale = 0.0f;
            if (useBias)
            {
                m2Vec2 dsA = {world->deltaPositions[c->bodyA].x + (drA.x - c->rA.x),
                              world->deltaPositions[c->bodyA].y + (drA.y - c->rA.y)};
                m2Vec2 dsB = {world->deltaPositions[c->bodyB].x + (drB.x - c->rB.x),
                              world->deltaPositions[c->bodyB].y + (drB.y - c->rB.y)};
                float C = c->baseC - (dsA.x * c->axis.x + dsA.y * c->axis.y) -
                          ratio * (dsB.x * c->perp.x + dsB.y * c->perp.y);
                bias = c->softness.massScale * c->softness.biasRate * C;
                massScale = c->softness.massScale;
                impulseScale = c->softness.impulseScale;
            }
            m2Vec2 vpA = {vA.x - wA * c->rA.y, vA.y + wA * c->rA.x};
            m2Vec2 vpB = {vB.x - wB * c->rB.y, vB.y + wB * c->rB.x};
            float cdot = -(vpA.x * c->axis.x + vpA.y * c->axis.y) -
                         ratio * (vpB.x * c->perp.x + vpB.y * c->perp.y);
            float impulse = -c->axialMass * (massScale * cdot + bias) - impulseScale * c->impulse.x;
            c->impulse.x += impulse;
            m2Vec2 PA = {-impulse * c->axis.x, -impulse * c->axis.y};
            m2Vec2 PB = {-ratio * impulse * c->perp.x, -ratio * impulse * c->perp.y};
            world->linearVelocities[c->bodyA].x = vA.x + mA * PA.x;
            world->linearVelocities[c->bodyA].y = vA.y + mA * PA.y;
            world->angularVelocities[c->bodyA] = wA + iA * Cross(c->rA, PA);
            world->linearVelocities[c->bodyB].x = vB.x + mB * PB.x;
            world->linearVelocities[c->bodyB].y = vB.y + mB * PB.y;
            world->angularVelocities[c->bodyB] = wB + iB * Cross(c->rB, PB);
        }
        else if (c->type == 7)
        {
            // Mouse joint (reference solve, always biased): a soft spin
            // damper, then the soft pull toward the target, clamped to
            // the force budget in lower.
            float mB = world->invMass[c->bodyB];
            float iB = world->invInertia[c->bodyB];
            {
                float impulse = iB > 0.0f ? -wB / iB : 0.0f;
                impulse =
                    c->softness2.massScale * impulse - c->softness2.impulseScale * c->motorImpulse;
                c->motorImpulse += impulse;
                wB += iB * impulse;
            }
            {
                m2Vec2 sep = {c->baseCVec.x + world->deltaPositions[c->bodyB].x + drB.x,
                              c->baseCVec.y + world->deltaPositions[c->bodyB].y + drB.y};
                m2Vec2 bias = {c->softness.biasRate * sep.x, c->softness.biasRate * sep.y};
                m2Vec2 cdot = {vB.x - wB * drB.y + bias.x, vB.y + wB * drB.x + bias.y};
                float det = c->k11 * c->k22 - c->k12 * c->k12;
                float invDet = det != 0.0f ? 1.0f / det : 0.0f;
                m2Vec2 raw = {invDet * (c->k22 * cdot.x - c->k12 * cdot.y),
                              invDet * (c->k11 * cdot.y - c->k12 * cdot.x)};
                m2Vec2 impulse = {
                    -c->softness.massScale * raw.x - c->softness.impulseScale * c->impulse.x,
                    -c->softness.massScale * raw.y - c->softness.impulseScale * c->impulse.y};
                m2Vec2 old = c->impulse;
                c->impulse.x += impulse.x;
                c->impulse.y += impulse.y;
                float budget = c->lower;
                float mag2 = c->impulse.x * c->impulse.x + c->impulse.y * c->impulse.y;
                if (mag2 > budget * budget)
                {
                    float mag = sqrtf(mag2);
                    float scale = mag > 0.0f ? budget / mag : 0.0f;
                    c->impulse.x *= scale;
                    c->impulse.y *= scale;
                }
                impulse.x = c->impulse.x - old.x;
                impulse.y = c->impulse.y - old.y;
                vB.x += mB * impulse.x;
                vB.y += mB * impulse.y;
                wB += iB * Cross(drB, impulse);
            }
            world->linearVelocities[c->bodyB] = vB;
            world->angularVelocities[c->bodyB] = wB;
        }
        else if (c->type == 2)
        {
            float mA = world->invMass[c->bodyA];
            float iA = world->invInertia[c->bodyA];
            float mB = world->invMass[c->bodyB];
            float iB = world->invInertia[c->bodyB];
            float translation = c->baseC + c->axis.x * ds.x + c->axis.y * ds.y;

            // Fresh effective mass per substep (reference b2 #981): the axial
            // and perpendicular torque arms a1/s1 track the current
            // separation, so recompute them and the effective masses here
            // instead of leaving them frozen at prepare. A stressed slider
            // whose geometry rotates within the step no longer solves against
            // a stale mass and diverges. cross is linear, so the fresh arm is
            // a1_prepare + cross(ds, axis); a2/s2 and the angle row do not
            // depend on the separation. The prepare-time values are swapped
            // back in at the end so the next substep recomputes from them.
            float a1Frozen = c->a1;
            float s1Frozen = c->s1;
            float axialMassFrozen = c->axialMass;
            float k11Frozen = c->k11;
            float k12Frozen = c->k12;
            c->a1 = a1Frozen + Cross(ds, c->axis);
            c->s1 = s1Frozen + Cross(ds, c->perp);
            float kaFresh = mA + mB + iA * c->a1 * c->a1 + iB * c->a2 * c->a2;
            c->axialMass = kaFresh > 0.0f ? 1.0f / kaFresh : 0.0f;
            c->k11 = mA + mB + iA * c->s1 * c->s1 + iB * c->s2 * c->s2;
            c->k12 = iA * c->s1 + iB * c->s2;

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
                        // Clamp the speculative distance to a safe span (b2
                        // #981): a slider far inside its range cannot inject
                        // a huge corrective velocity toward the limit.
                        bias = m2MinF(C, 1.0f) * invH;
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
                        // Speculative distance clamped to a safe span (b2 #981).
                        bias = m2MinF(C, 1.0f) * invH;
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
            // Swap the prepare-time arms and masses back so the next substep
            // rebuilds the fresh values from them, not from this substep's.
            c->a1 = a1Frozen;
            c->s1 = s1Frozen;
            c->axialMass = axialMassFrozen;
            c->k11 = k11Frozen;
            c->k12 = k12Frozen;
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
        world->jointSpringImpulse[j] = joints[i].springImpulse;
    }
}

void m2JointReactionMagnitudes(const m2World* world, int32_t j, float invH, float* force,
                               float* torque)
{
    m2Vec2 impulse = world->jointImpulse[j];
    float axialTrio = world->jointSpringImpulse[j] + world->jointMotorImpulse[j] +
                      world->jointLowerImpulse[j] - world->jointUpperImpulse[j];
    *force = 0.0f;
    *torque = 0.0f;
    switch (world->jointType[j])
    {
    case 0: // distance: axial row, plus range rows when bounded
    {
        float axial = impulse.x;
        if (world->jointLower[j] > 0.0f || world->jointUpper[j] < 3.0e38f)
        {
            axial += world->jointLowerImpulse[j] - world->jointUpperImpulse[j];
        }
        *force = m2AbsF(axial) * invH;
        break;
    }
    case 1: // revolute: point block linear, motor/limits as torque
        *force = sqrtf(impulse.x * impulse.x + impulse.y * impulse.y) * invH;
        *torque = m2AbsF(axialTrio) * invH;
        break;
    case 2: // prismatic: (perp, angle) + axial trio
    {
        float linear = sqrtf(impulse.x * impulse.x + axialTrio * axialTrio);
        *force = linear * invH;
        *torque = m2AbsF(impulse.y) * invH;
        break;
    }
    case 3: // weld: point block linear, angle row in the motor slot
        *force = sqrtf(impulse.x * impulse.x + impulse.y * impulse.y) * invH;
        *torque = m2AbsF(world->jointMotorImpulse[j]) * invH;
        break;
    case 5: // filter: no rows, no loads, by definition
        break;
    case 6: // motor: linear block + pure torque in the motor slot
    case 7: // mouse: same accumulator shape, body B only
        *force = sqrtf(impulse.x * impulse.x + impulse.y * impulse.y) * invH;
        *torque = m2AbsF(world->jointMotorImpulse[j]) * invH;
        break;
    case 8: // gear: pure torque coupling
        *torque = m2AbsF(impulse.x) * invH;
        break;
    case 9: // pulley: A-side rope tension (B side feels ratio times this)
        *force = m2AbsF(impulse.x) * invH;
        break;
    case 10: // ratchet: pure holding torque
        *torque = m2AbsF(impulse.x) * invH;
        break;
    default: // wheel: (perp, spring) linear, motor as torque
    {
        float axial = impulse.y + world->jointLowerImpulse[j] - world->jointUpperImpulse[j];
        *force = sqrtf(impulse.x * impulse.x + axial * axial) * invH;
        *torque = m2AbsF(world->jointMotorImpulse[j]) * invH;
        break;
    }
    }
}

void m2SolveStep(m2World* world, float dt, int32_t substepCount)
{
    float h = dt / (float)substepCount;
    float invH = h > 0.0f ? 1.0f / h : 0.0f;
    world->lastInvH = invH;

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
                world->asleep[i] != 0 || world->disabled[i] != 0)
            {
                continue;
            }
            // Reference form: v = lvd + damp * v, with the Pade damping
            // 1/(1+h*d) and lvd = h*invM*force + h*gScale*g. Torque and
            // angular damping mirror it.
            float linDamp = 1.0f / (1.0f + h * world->linearDampings[i]);
            float angDamp = 1.0f / (1.0f + h * world->angularDampings[i]);
            float lvdx = h * world->invMass[i] * world->forces[i].x +
                         h * world->gravityScales[i] * world->gravity.x;
            float lvdy = h * world->invMass[i] * world->forces[i].y +
                         h * world->gravityScales[i] * world->gravity.y;
            world->linearVelocities[i].x = lvdx + linDamp * world->linearVelocities[i].x;
            world->linearVelocities[i].y = lvdy + linDamp * world->linearVelocities[i].y;
            world->angularVelocities[i] = h * world->invInertia[i] * world->torques[i] +
                                          angDamp * world->angularVelocities[i];
            // Motion locks (reference b2 #950): a locked axis holds still,
            // so its velocity is zeroed here, before the constraint solve,
            // and again at integrate-positions below (angular is locked via
            // the mass, invInertia = 0). Off the locked axes are untouched.
            uint8_t locks = world->motionLocks[i];
            if (locks & 1u)
            {
                world->linearVelocities[i].x = 0.0f;
            }
            if (locks & 2u)
            {
                world->linearVelocities[i].y = 0.0f;
            }
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
            if (world->bullets[i] != 0 && world->disabled[i] == 0)
            {
                world->ccdPrevPositions[i] = world->transforms[i].p;
            }
        }

        // Integrate positions: f64 positions advance, f32 deltas track.
        for (int32_t i = 0; i < world->maxBodyIndex; ++i)
        {
            if (world->alive[i] == 0 || world->types[i] == (uint8_t)m2_staticBody ||
                world->asleep[i] != 0 || world->disabled[i] != 0)
            {
                continue;
            }
            // Reference velocity caps (b2 maximumLinearSpeed 400 m/s and
            // B2_MAX_ROTATION, a quarter turn per substep), applied to the
            // STORED dynamic velocity at the point the integrator consumes
            // it. The reference caps in IntegrateVelocities; Maul caps here
            // because a warm-started joint accumulation can spike a body past
            // the cap AFTER that stage, and the guard must bound what
            // m2MakeRot and the f64 position actually see. Capping the stored
            // value each substep also breaks an over-constrained scene's
            // exponential velocity growth at the source, so it can never
            // reach inf and hand a kinematic neighbour 0 * inf = NaN. Ratio
            // scaling matches the reference bit-form; tame scenes reach
            // neither cap, so the gated hashes are untouched.
            if (world->types[i] == (uint8_t)m2_dynamicBody)
            {
                float vx = world->linearVelocities[i].x;
                float vy = world->linearVelocities[i].y;
                float v2 = vx * vx + vy * vy;
                if (v2 > M2_MAX_LINEAR_SPEED * M2_MAX_LINEAR_SPEED)
                {
                    float ratio = M2_MAX_LINEAR_SPEED / sqrtf(v2);
                    world->linearVelocities[i].x = vx * ratio;
                    world->linearVelocities[i].y = vy * ratio;
                }
                float maxW = 0.25f * M2_PI * invH;
                float wv = world->angularVelocities[i];
                if (wv * wv > maxW * maxW)
                {
                    float ratio = maxW / m2AbsF(wv);
                    world->angularVelocities[i] = wv * ratio;
                }
            }
            // Motion locks again at the point the position consumes the
            // velocity (reference b2 #950): whatever the solve pushed along
            // a locked axis, the body does not move along it.
            uint8_t plocks = world->motionLocks[i];
            if (plocks & 1u)
            {
                world->linearVelocities[i].x = 0.0f;
            }
            if (plocks & 2u)
            {
                world->linearVelocities[i].y = 0.0f;
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

    // Break pass: reaction magnitudes come straight from the stored
    // impulses, so breaking is a pure function of state - twins snap
    // on the same step and replays never disagree. Canonical joint
    // order; the destroyed id is reported with the generation it had.
    for (int32_t j = 0; j < world->maxJointIndex; ++j)
    {
        if (world->jointAlive[j] == 0)
        {
            continue;
        }
        float breakForce = world->jointBreakForce[j];
        float breakTorque = world->jointBreakTorque[j];
        if (breakForce == 0.0f && breakTorque == 0.0f)
        {
            continue;
        }

        float force = 0.0f;
        float torque = 0.0f;
        m2JointReactionMagnitudes(world, j, invH, &force, &torque);

        bool snapped = (breakForce > 0.0f && force > breakForce) ||
                       (breakTorque > 0.0f && torque > breakTorque);
        if (!snapped)
        {
            continue;
        }

        if (world->jointBreakEventCount < world->jointCapacity)
        {
            m2JointBreakEvent* e = &world->jointBreakEvents[world->jointBreakEventCount++];
            memset(e, 0, sizeof(*e));
            e->jointId.index1 = j + 1;
            e->jointId.world0 = world->worldIndex0;
            e->jointId.generation = world->jointGenerations[j];
            e->step = world->stepCount;
            e->force = force;
            e->torque = torque;
        }
        m2DestroyJointInternal(world, j);
    }
}
