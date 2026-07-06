// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Fluids, the neighbor structure (chapter slice 2): a uniform grid
// with cell size = particle diameter, rebuilt from positions every
// step (history-free, the island precedent: derived structures
// never enter the snapshot). The pair list is canonical by
// construction: proxies sorted by (cell key, particle index), one
// sweep visiting the east neighbor on the same row and the
// west/center/east cells on the row above, every pair emitted
// exactly once in sweep order.
//
// Adapted from LiquidFun's b2ParticleSystem contact pass (see
// THIRD_PARTY.md) with three argued deviations:
//   1. Cell keys are 64-bit with BIASED coordinates
//      ((int64)floor(x/d) + 2^31 packed as (y << 32) | x). The
//      reference's 32-bit offset tag wraps close to the origin
//      axes; a wrap between adjacent columns would silently drop
//      seam pairs. With the bias the seam sits ~2*10^8 m out, and
//      the distance test guards even that.
//   2. Coincident particles (distance exactly zero) get the
//      canonical fallback normal (0, 1) and full weight instead of
//      the reference's NaN (0 * inf); the NaN-free invariant is
//      constitution.
//   3. The pair buffer is fixed (12 per particle of capacity);
//      overflow truncates DETERMINISTICALLY in sweep order and
//      counts loudly in particlePairOverflow. LiquidFun grows
//      buffers; Maul's fixed-capacity law does not.

#include "world_internal.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static m2Vec2 Rotate(m2Rot q, m2Vec2 v)
{
    return (m2Vec2){q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y};
}

typedef struct m2ParticleProxy
{
    uint64_t key;
    int32_t index;
    int32_t pad; // explicit, never read
} m2ParticleProxy;

static uint64_t ParticleCellKey(m2Pos2 position, double inverseDiameter)
{
    uint32_t cx = (uint32_t)((int64_t)floor(position.x * inverseDiameter) + 2147483648LL);
    uint32_t cy = (uint32_t)((int64_t)floor(position.y * inverseDiameter) + 2147483648LL);
    return ((uint64_t)cy << 32) | (uint64_t)cx;
}

static uint64_t ShiftKey(uint64_t key, int32_t dx, int32_t dy)
{
    uint32_t cx = (uint32_t)key + (uint32_t)dx;
    uint32_t cy = (uint32_t)(key >> 32) + (uint32_t)dy;
    return ((uint64_t)cy << 32) | (uint64_t)cx;
}

static int CompareProxies(const void* lhs, const void* rhs)
{
    const m2ParticleProxy* a = (const m2ParticleProxy*)lhs;
    const m2ParticleProxy* b = (const m2ParticleProxy*)rhs;
    if (a->key != b->key)
    {
        return a->key < b->key ? -1 : 1;
    }
    return a->index < b->index ? -1 : (a->index > b->index ? 1 : 0);
}

// First proxy at or after key, searching [lo, count).
static int32_t LowerBound(const m2ParticleProxy* proxies, int32_t lo, int32_t count, uint64_t key)
{
    int32_t hi = count;
    while (lo < hi)
    {
        int32_t mid = lo + (hi - lo) / 2;
        if (proxies[mid].key < key)
        {
            lo = mid + 1;
        }
        else
        {
            hi = mid;
        }
    }
    return lo;
}

static void TryPair(m2World* world, int32_t a, int32_t b, float diameter)
{
    float dx = (float)(world->particlePositions[b].x - world->particlePositions[a].x);
    float dy = (float)(world->particlePositions[b].y - world->particlePositions[a].y);
    float distSq = dx * dx + dy * dy;
    if (distSq >= diameter * diameter)
    {
        return;
    }
    if (world->particlePairCount >= world->particlePairCapacity)
    {
        world->particlePairOverflow += 1; // deterministic truncation, counted
        return;
    }
    int32_t n = world->particlePairCount;
    world->particlePairA[n] = a;
    world->particlePairB[n] = b;
    world->particlePairFlags[n] = world->particleFlags[a] | world->particleFlags[b];
    world->particleFlagsUnion |= world->particlePairFlags[n];
    if (distSq > 1.0e-12f)
    {
        float dist = sqrtf(distSq);
        float inv = 1.0f / dist;
        world->particlePairWeight[n] = 1.0f - dist / diameter;
        world->particlePairNormal[n] = (m2Vec2){dx * inv, dy * inv};
    }
    else
    {
        // Coincident: canonical fallback, never NaN (deviation 2).
        world->particlePairWeight[n] = 1.0f;
        world->particlePairNormal[n] = (m2Vec2){0.0f, 1.0f};
    }
    world->particlePairCount = n + 1;
}

void m2UpdateParticlePairs(m2World* world)
{
    world->particlePairCount = 0;
    world->particlePairOverflow = 0;
    world->particleFlagsUnion = 0;
    if (world->particleCount == 0)
    {
        return;
    }
    float diameter = 2.0f * world->particleRadius;
    double inverseDiameter = 1.0 / (2.0 * (double)world->particleRadius);

    m2ParticleProxy* proxies = (m2ParticleProxy*)world->particleProxies;
    int32_t proxyCount = 0;
    for (int32_t i = 0; i < world->maxParticleIndex; ++i)
    {
        if (world->particleAlive[i] == 0)
        {
            continue;
        }
        proxies[proxyCount].key = ParticleCellKey(world->particlePositions[i], inverseDiameter);
        proxies[proxyCount].index = i;
        proxies[proxyCount].pad = 0;
        proxyCount += 1;
    }
    qsort(proxies, (size_t)proxyCount, sizeof(m2ParticleProxy), CompareProxies);

    for (int32_t a = 0; a < proxyCount; ++a)
    {
        uint64_t key = proxies[a].key;
        // Same row: this cell's remainder plus the east neighbor.
        uint64_t eastKey = ShiftKey(key, 1, 0);
        for (int32_t b = a + 1; b < proxyCount && proxies[b].key <= eastKey; ++b)
        {
            TryPair(world, proxies[a].index, proxies[b].index, diameter);
        }
        // Row above: west, center and east cells as one key range.
        uint64_t rowLo = ShiftKey(key, -1, 1);
        uint64_t rowHi = ShiftKey(key, 1, 1);
        for (int32_t b = LowerBound(proxies, a + 1, proxyCount, rowLo);
             b < proxyCount && proxies[b].key <= rowHi; ++b)
        {
            TryPair(world, proxies[a].index, proxies[b].index, diameter);
        }
    }
}

// Particle-vs-body contacts (chapter slice 4): for every particle,
// every shape whose surface sits within one diameter contributes a
// contact carrying the reference fields (weight, outward normal,
// pair-effective mass). Candidates come from the three trees like
// the CCD sweep does: sensors never touch water, one-way chain
// links ignore particles on their ghost side, order is canonical
// (particles in index order, candidate shapes ascending). One-way
// for now: bodies push water, the water pushes back in the next
// slice.
#define M2_PARTICLE_CANDIDATES 32

static void UpdateParticleBodyContacts(m2World* world)
{
    world->particleBodyCount = 0;
    world->particleBodyOverflow = 0;
    float diameter = 2.0f * world->particleRadius;
    float stride = 0.75f * diameter;
    float particleMass = world->particleDensity * stride * stride;
    float invAm = 1.0f / particleMass;

    for (int32_t i = 0; i < world->maxParticleIndex; ++i)
    {
        if (world->particleAlive[i] == 0)
        {
            continue;
        }
        m2Pos2 pp = world->particlePositions[i];
        m2AABB box;
        box.lowerBound.x = pp.x - (double)diameter;
        box.lowerBound.y = pp.y - (double)diameter;
        box.upperBound.x = pp.x + (double)diameter;
        box.upperBound.y = pp.y + (double)diameter;

        int32_t candidates[M2_PARTICLE_CANDIDATES];
        int32_t candidateCount = 0;
        for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
        {
            int32_t results[M2_PARTICLE_CANDIDATES];
            int32_t hits = m2Tree_Query(&world->trees[t], world->treeNodes[t], box, results,
                                        M2_PARTICLE_CANDIDATES);
            hits = hits <= M2_PARTICLE_CANDIDATES ? hits : M2_PARTICLE_CANDIDATES;
            for (int32_t h = 0; h < hits && candidateCount < M2_PARTICLE_CANDIDATES; ++h)
            {
                int32_t shape = results[h];
                if (world->shapeSensor[shape] != 0)
                {
                    continue; // sensors never touch water
                }
                candidates[candidateCount++] = shape;
            }
        }
        // Canonical order regardless of tree traversal.
        for (int32_t a = 1; a < candidateCount; ++a)
        {
            int32_t key = candidates[a];
            int32_t j = a - 1;
            while (j >= 0 && candidates[j] > key)
            {
                candidates[j + 1] = candidates[j];
                j -= 1;
            }
            candidates[j + 1] = key;
        }

        for (int32_t c = 0; c < candidateCount; ++c)
        {
            int32_t shape = candidates[c];
            int32_t body = world->shapeBody[shape];
            m2Transform xf = world->transforms[body];
            float dx = (float)(pp.x - xf.p.x);
            float dy = (float)(pp.y - xf.p.y);
            m2Vec2 local = {xf.q.c * dx + xf.q.s * dy, -xf.q.s * dx + xf.q.c * dy};

            const m2ShapeGeometry* g = &world->shapeGeometry[shape];
            if (g->type == m2_chainSegmentShape)
            {
                // Ghost side: the sign law shared with contacts, rays
                // and CCD; water on the pass-through side ignores it.
                const m2ChainSegment* link = &g->chainSegment;
                m2Vec2 e = {link->segment.point2.x - link->segment.point1.x,
                            link->segment.point2.y - link->segment.point1.y};
                float offset = (local.x - link->segment.point1.x) * e.y -
                               (local.y - link->segment.point1.y) * e.x;
                if (offset < 0.0f)
                {
                    continue;
                }
            }

            m2DistanceProxy shapeProxy = m2GeometryProxy(g);
            m2DistanceProxy pointProxy;
            memset(&pointProxy, 0, sizeof(pointProxy));
            pointProxy.points[0] = local;
            pointProxy.count = 1;
            pointProxy.radius = 0.0f;
            m2DistanceResult dist = m2ShapeDistance(&shapeProxy, &pointProxy);
            float separation = dist.distance - shapeProxy.radius;
            if (separation >= diameter)
            {
                continue;
            }
            m2Vec2 n;
            if (dist.normal.x != 0.0f || dist.normal.y != 0.0f)
            {
                n = Rotate(xf.q, dist.normal); // shape toward particle
            }
            else
            {
                // Deep overlap: push from the body origin, the mover
                // kit's fallback; coincident falls back canonically.
                float len = sqrtf(dx * dx + dy * dy);
                n = len > 1.0e-6f ? (m2Vec2){dx / len, dy / len} : (m2Vec2){0.0f, 1.0f};
            }
            if (world->particleBodyCount >= world->particleBodyCapacity)
            {
                world->particleBodyOverflow += 1;
                continue;
            }
            m2Vec2 lc = world->localCenters[body];
            m2Vec2 comArm = Rotate(xf.q, lc);
            float rpx = dx - comArm.x;
            float rpy = dy - comArm.y;
            float rpn = rpx * n.y - rpy * n.x;
            float invM = invAm + world->invMass[body] + world->invInertia[body] * rpn * rpn;
            if (world->types[body] == (uint8_t)m2_dynamicBody)
            {
                // Water touching a body wakes it (reference semantics:
                // every body impulse wakes); the pass runs before the
                // island update so the wake spreads island-wide.
                world->asleep[body] = 0;
                world->sleepTimes[body] = 0.0f;
            }
            int32_t k = world->particleBodyCount;
            world->particleBodyParticle[k] = i;
            world->particleBodyBody[k] = body;
            world->particleBodyWeight[k] = 1.0f - separation / diameter;
            world->particleBodyNormal[k] = n;
            world->particleBodyMass[k] = invM > 0.0f ? 1.0f / invM : 0.0f;
            world->particleBodyCount = k + 1;
        }
    }
}

// The water pass (chapter slice 3), the reference relaxation solver
// on the frozen pair list, once per step before the rigid solve:
// weight (dimensionless density) -> viscosity (system-level strength;
// zero means plain water) -> gravity -> pressure -> damping ->
// velocity limit -> advance. All scalars f32 through the pinned op
// whitelist (sqrtf + divide, never a fast inverse sqrt), min/max as
// compare+select, every loop in fixed index or pair order.
void m2SolveParticles(m2World* world, float dt)
{
    m2UpdateParticlePairs(world);
    UpdateParticleBodyContacts(world);
    if (dt <= 0.0f)
    {
        return;
    }
    float stride = 0.75f * 2.0f * world->particleRadius;
    float particleInvMass = 1.0f / (world->particleDensity * stride * stride);
    int32_t bodyContactCount = world->particleBodyCount;
    float invDt = 1.0f / dt;
    float diameter = 2.0f * world->particleRadius;
    int32_t pairCount = world->particlePairCount;

    // Dimensionless density: the sum of contact weights.
    for (int32_t i = 0; i < world->maxParticleIndex; ++i)
    {
        world->particleWeights[i] = 0.0f;
    }
    for (int32_t k = 0; k < bodyContactCount; ++k)
    {
        world->particleWeights[world->particleBodyParticle[k]] += world->particleBodyWeight[k];
    }
    for (int32_t k = 0; k < pairCount; ++k)
    {
        float w = world->particlePairWeight[k];
        world->particleWeights[world->particlePairA[k]] += w;
        world->particleWeights[world->particlePairB[k]] += w;
    }

    // Viscosity kills relative velocity across every pair. The
    // reference gates this on a per-particle flag; Maul's v1 makes
    // it a system property (zero strength = plain water), the
    // per-particle flag can arrive with the behavior flags later.
    float viscous = world->particleViscousStrength;
    if (viscous > 0.0f)
    {
        for (int32_t k = 0; k < bodyContactCount; ++k)
        {
            int32_t a = world->particleBodyParticle[k];
            int32_t body = world->particleBodyBody[k];
            float w = world->particleBodyWeight[k];
            float m = world->particleBodyMass[k];
            m2Pos2 pp = world->particlePositions[a];
            m2Vec2 lc = world->localCenters[body];
            m2Vec2 comArm = Rotate(world->transforms[body].q, lc);
            float rx = (float)(pp.x - world->transforms[body].p.x) - comArm.x;
            float ry = (float)(pp.y - world->transforms[body].p.y) - comArm.y;
            float wb = world->angularVelocities[body];
            float bvx = world->linearVelocities[body].x - wb * ry;
            float bvy = world->linearVelocities[body].y + wb * rx;
            float fx = viscous * m * w * (bvx - world->particleVelocities[a].x);
            float fy = viscous * m * w * (bvy - world->particleVelocities[a].y);
            world->particleVelocities[a].x += particleInvMass * fx;
            world->particleVelocities[a].y += particleInvMass * fy;
            world->linearVelocities[body].x -= world->invMass[body] * fx;
            world->linearVelocities[body].y -= world->invMass[body] * fy;
            world->angularVelocities[body] -= world->invInertia[body] * (rx * fy - ry * fx);
        }
        for (int32_t k = 0; k < pairCount; ++k)
        {
            int32_t a = world->particlePairA[k];
            int32_t b = world->particlePairB[k];
            float w = world->particlePairWeight[k];
            float fx =
                viscous * w * (world->particleVelocities[b].x - world->particleVelocities[a].x);
            float fy =
                viscous * w * (world->particleVelocities[b].y - world->particleVelocities[a].y);
            world->particleVelocities[a].x += fx;
            world->particleVelocities[a].y += fy;
            world->particleVelocities[b].x -= fx;
            world->particleVelocities[b].y -= fy;
        }
    }

    // Surface tension (reference SolveTensile): each tensile pair
    // first votes a weighted surface normal, then pairs attract by
    // local density above 2 plus the normal disagreement, capped at
    // half the critical velocity. Droplets bead up and cling.
    if ((world->particleFlagsUnion & m2_tensileParticle) != 0)
    {
        for (int32_t i = 0; i < world->maxParticleIndex; ++i)
        {
            world->particleAccumulation2[i] = (m2Vec2){0.0f, 0.0f};
        }
        for (int32_t k = 0; k < pairCount; ++k)
        {
            if ((world->particlePairFlags[k] & m2_tensileParticle) == 0)
            {
                continue;
            }
            int32_t a = world->particlePairA[k];
            int32_t b = world->particlePairB[k];
            float w = world->particlePairWeight[k];
            m2Vec2 n = world->particlePairNormal[k];
            float wn = (1.0f - w) * w;
            world->particleAccumulation2[a].x -= wn * n.x;
            world->particleAccumulation2[a].y -= wn * n.y;
            world->particleAccumulation2[b].x += wn * n.x;
            world->particleAccumulation2[b].y += wn * n.y;
        }
        float tensilePressure = world->particleTensilePressure * (diameter * invDt);
        float tensileNormal = world->particleTensileNormal * (diameter * invDt);
        float maxVariation = 0.5f * (diameter * invDt);
        for (int32_t k = 0; k < pairCount; ++k)
        {
            if ((world->particlePairFlags[k] & m2_tensileParticle) == 0)
            {
                continue;
            }
            int32_t a = world->particlePairA[k];
            int32_t b = world->particlePairB[k];
            float w = world->particlePairWeight[k];
            m2Vec2 n = world->particlePairNormal[k];
            float h = world->particleWeights[a] + world->particleWeights[b];
            float sx = world->particleAccumulation2[b].x - world->particleAccumulation2[a].x;
            float sy = world->particleAccumulation2[b].y - world->particleAccumulation2[a].y;
            float fn = m2MinF(tensilePressure * (h - 2.0f) + tensileNormal * (sx * n.x + sy * n.y),
                              maxVariation) *
                       w;
            world->particleVelocities[a].x -= fn * n.x;
            world->particleVelocities[a].y -= fn * n.y;
            world->particleVelocities[b].x += fn * n.x;
            world->particleVelocities[b].y += fn * n.y;
        }
    }

    // Gravity, full step, fixed index order.
    float gx = dt * world->particleGravityScale * world->gravity.x;
    float gy = dt * world->particleGravityScale * world->gravity.y;
    for (int32_t i = 0; i < world->maxParticleIndex; ++i)
    {
        if (world->particleAlive[i] != 0)
        {
            world->particleVelocities[i].x += gx;
            world->particleVelocities[i].y += gy;
        }
    }

    // Pressure as a linear function of density above the rest weight
    // (reference constants: min weight 1, pressure cap 0.25 of the
    // critical pressure; the critical velocity is one diameter per
    // step, the scale that keeps neighbors discoverable).
    float criticalVelocity = diameter * invDt;
    float criticalPressure = world->particleDensity * criticalVelocity * criticalVelocity;
    float pressurePerWeight = world->particlePressureStrength * criticalPressure;
    float maxPressure = 0.25f * criticalPressure;
    for (int32_t i = 0; i < world->maxParticleIndex; ++i)
    {
        float w = world->particleWeights[i];
        float h = pressurePerWeight * m2MaxF(0.0f, w - 1.0f);
        world->particleAccumulation[i] = m2MinF(h, maxPressure);
    }
    float velocityPerPressure = dt / (world->particleDensity * diameter);
    for (int32_t k = 0; k < bodyContactCount; ++k)
    {
        int32_t a = world->particleBodyParticle[k];
        int32_t body = world->particleBodyBody[k];
        float w = world->particleBodyWeight[k];
        float m = world->particleBodyMass[k];
        m2Vec2 n = world->particleBodyNormal[k];
        float h = world->particleAccumulation[a] + pressurePerWeight * w;
        float f = velocityPerPressure * w * m * h;
        // n points from the shape toward the particle: push out, and
        // push the body the other way (buoyancy is exactly this).
        world->particleVelocities[a].x += particleInvMass * f * n.x;
        world->particleVelocities[a].y += particleInvMass * f * n.y;
        m2Pos2 pp = world->particlePositions[a];
        m2Vec2 lc = world->localCenters[body];
        m2Vec2 comArm = Rotate(world->transforms[body].q, lc);
        float rx = (float)(pp.x - world->transforms[body].p.x) - comArm.x;
        float ry = (float)(pp.y - world->transforms[body].p.y) - comArm.y;
        world->linearVelocities[body].x -= world->invMass[body] * f * n.x;
        world->linearVelocities[body].y -= world->invMass[body] * f * n.y;
        world->angularVelocities[body] -= world->invInertia[body] * (rx * f * n.y - ry * f * n.x);
    }
    for (int32_t k = 0; k < pairCount; ++k)
    {
        int32_t a = world->particlePairA[k];
        int32_t b = world->particlePairB[k];
        float w = world->particlePairWeight[k];
        m2Vec2 n = world->particlePairNormal[k];
        float h = world->particleAccumulation[a] + world->particleAccumulation[b];
        float f = velocityPerPressure * w * h;
        world->particleVelocities[a].x -= f * n.x;
        world->particleVelocities[a].y -= f * n.y;
        world->particleVelocities[b].x += f * n.x;
        world->particleVelocities[b].y += f * n.y;
    }

    // Damping eats approach speed only (vn < 0), linear plus
    // quadratic, capped at half the approach per pass.
    float linearDamping = world->particleDampingStrength;
    float quadraticDamping = 1.0f / criticalVelocity;
    for (int32_t k = 0; k < bodyContactCount; ++k)
    {
        int32_t a = world->particleBodyParticle[k];
        int32_t body = world->particleBodyBody[k];
        float w = world->particleBodyWeight[k];
        float m = world->particleBodyMass[k];
        m2Vec2 n = world->particleBodyNormal[k];
        m2Pos2 pp = world->particlePositions[a];
        m2Vec2 lc = world->localCenters[body];
        m2Vec2 comArm = Rotate(world->transforms[body].q, lc);
        float rx = (float)(pp.x - world->transforms[body].p.x) - comArm.x;
        float ry = (float)(pp.y - world->transforms[body].p.y) - comArm.y;
        float wb = world->angularVelocities[body];
        float bvx = world->linearVelocities[body].x - wb * ry;
        float bvy = world->linearVelocities[body].y + wb * rx;
        float relx = bvx - world->particleVelocities[a].x;
        float rely = bvy - world->particleVelocities[a].y;
        float vn = relx * n.x + rely * n.y;
        if (vn > 0.0f)
        {
            // Approaching along the outward normal from the particle's
            // side; eat it on both ends like the pair pass does.
            float damping = m2MaxF(linearDamping * w, m2MinF(quadraticDamping * vn, 0.5f));
            float f = damping * m * vn;
            world->particleVelocities[a].x += particleInvMass * f * n.x;
            world->particleVelocities[a].y += particleInvMass * f * n.y;
            world->linearVelocities[body].x -= world->invMass[body] * f * n.x;
            world->linearVelocities[body].y -= world->invMass[body] * f * n.y;
            world->angularVelocities[body] -=
                world->invInertia[body] * (rx * f * n.y - ry * f * n.x);
        }
    }
    for (int32_t k = 0; k < pairCount; ++k)
    {
        int32_t a = world->particlePairA[k];
        int32_t b = world->particlePairB[k];
        float w = world->particlePairWeight[k];
        m2Vec2 n = world->particlePairNormal[k];
        float vx = world->particleVelocities[b].x - world->particleVelocities[a].x;
        float vy = world->particleVelocities[b].y - world->particleVelocities[a].y;
        float vn = vx * n.x + vy * n.y;
        if (vn < 0.0f)
        {
            float damping = m2MaxF(linearDamping * w, m2MinF(-quadraticDamping * vn, 0.5f));
            float f = damping * vn;
            world->particleVelocities[a].x += f * n.x;
            world->particleVelocities[a].y += f * n.y;
            world->particleVelocities[b].x -= f * n.x;
            world->particleVelocities[b].y -= f * n.y;
        }
    }

    // The velocity limit is the stability law: nothing crosses more
    // than one diameter per step, so neighbors stay discoverable and
    // dense clusters cannot explode.
    float criticalSq = criticalVelocity * criticalVelocity;
    for (int32_t i = 0; i < world->maxParticleIndex; ++i)
    {
        if (world->particleAlive[i] == 0)
        {
            continue;
        }
        float vx = world->particleVelocities[i].x;
        float vy = world->particleVelocities[i].y;
        float v2 = vx * vx + vy * vy;
        if (v2 > criticalSq)
        {
            float scale = sqrtf(criticalSq / v2);
            world->particleVelocities[i].x = vx * scale;
            world->particleVelocities[i].y = vy * scale;
        }
    }

    // The projection pass (reference SolveCollision): whoever would
    // land inside a body this step gets its velocity redirected to a
    // point one slop above the surface instead. Anti-tunneling for
    // water; the chain one-sided law rides inside the ray kernel.
    for (int32_t i = 0; i < world->maxParticleIndex; ++i)
    {
        if (world->particleAlive[i] == 0)
        {
            continue;
        }
        m2Pos2 p1 = world->particlePositions[i];
        m2Vec2 move = {world->particleVelocities[i].x * dt, world->particleVelocities[i].y * dt};
        m2AABB box;
        box.lowerBound.x = p1.x + (move.x < 0.0f ? (double)move.x : 0.0);
        box.lowerBound.y = p1.y + (move.y < 0.0f ? (double)move.y : 0.0);
        box.upperBound.x = p1.x + (move.x > 0.0f ? (double)move.x : 0.0);
        box.upperBound.y = p1.y + (move.y > 0.0f ? (double)move.y : 0.0);

        float bestFraction = 1.0f;
        int32_t bestShape = -1;
        m2Vec2 bestNormal = {0.0f, 0.0f};
        for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
        {
            int32_t results[M2_PARTICLE_CANDIDATES];
            int32_t hits = m2Tree_Query(&world->trees[t], world->treeNodes[t], box, results,
                                        M2_PARTICLE_CANDIDATES);
            hits = hits <= M2_PARTICLE_CANDIDATES ? hits : M2_PARTICLE_CANDIDATES;
            for (int32_t h = 0; h < hits; ++h)
            {
                int32_t shape = results[h];
                if (world->shapeSensor[shape] != 0)
                {
                    continue;
                }
                struct m2CastHitInternal hit = m2RayCastShapeIndex(world, shape, p1, move, 1.0f);
                if (!hit.hit || (hit.normal.x == 0.0f && hit.normal.y == 0.0f))
                {
                    continue; // misses and initial overlaps (pressure owns those)
                }
                if (hit.fraction < bestFraction ||
                    (hit.fraction == bestFraction && shape < bestShape))
                {
                    bestFraction = hit.fraction;
                    bestShape = shape;
                    bestNormal = hit.normal;
                }
            }
        }
        if (bestShape >= 0)
        {
            double tx = p1.x + (double)(bestFraction * move.x) + (double)(0.005f * bestNormal.x);
            double ty = p1.y + (double)(bestFraction * move.y) + (double)(0.005f * bestNormal.y);
            world->particleVelocities[i].x = (float)(tx - p1.x) * invDt;
            world->particleVelocities[i].y = (float)(ty - p1.y) * invDt;
            // The projection stays a pure clamp on the particle, like
            // the reference's SolveCollision (its body push arrives
            // through the contact loops above); the reference's
            // lost-momentum re-add force is deliberately omitted.
        }
    }

    // Advance, single f64 crossing per axis.
    for (int32_t i = 0; i < world->maxParticleIndex; ++i)
    {
        if (world->particleAlive[i] == 0)
        {
            continue;
        }
        world->particlePositions[i].x += (double)(world->particleVelocities[i].x * dt);
        world->particlePositions[i].y += (double)(world->particleVelocities[i].y * dt);
    }
}
