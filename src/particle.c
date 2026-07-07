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

#include "maul2d/base.h"

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

// Stable LSD radix over the 64-bit cell key, eight 8-bit passes.
// The proxy array is built in ascending particle index order and
// stability preserves it, so the result is exactly the (key, index)
// order the old comparison sort produced: the gated pair digest is
// the proof, byte for byte. O(n) instead of n log n, and the pair
// pass stops being sort-bound as pools grow.
static void SortProxies(m2ParticleProxy* proxies, m2ParticleProxy* scratch, int32_t count)
{
    m2ParticleProxy* src = proxies;
    m2ParticleProxy* dst = scratch;
    for (int32_t pass = 0; pass < 8; ++pass)
    {
        int32_t shift = pass * 8;
        int32_t histogram[256];
        memset(histogram, 0, sizeof(histogram));
        for (int32_t i = 0; i < count; ++i)
        {
            histogram[(src[i].key >> shift) & 0xFF] += 1;
        }
        int32_t running = 0;
        for (int32_t b = 0; b < 256; ++b)
        {
            int32_t n = histogram[b];
            histogram[b] = running;
            running += n;
        }
        for (int32_t i = 0; i < count; ++i)
        {
            dst[histogram[(src[i].key >> shift) & 0xFF]++] = src[i];
        }
        m2ParticleProxy* swap = src;
        src = dst;
        dst = swap;
    }
    // Eight passes: the data ends where it started.
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
    SortProxies(proxies, (m2ParticleProxy*)world->particleProxiesTmp, proxyCount);

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
#define M2_FILL_COLUMNS        256

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

    // Viscosity drags flagged particles toward their neighbors'
    // velocities (the reference's per-particle gate, adopted now
    // that behavior flags exist).
    float viscous = world->particleViscousStrength;
    if (viscous > 0.0f)
    {
        for (int32_t k = 0; k < bodyContactCount; ++k)
        {
            int32_t a = world->particleBodyParticle[k];
            if ((world->particleFlags[a] & m2_viscousParticle) == 0)
            {
                continue;
            }
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
            if ((world->particlePairFlags[k] & m2_viscousParticle) == 0)
            {
                continue;
            }
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

    // Powder (reference SolvePowder): grains packed tighter than the
    // rest stride push apart and never cohere; that is the whole law
    // of sand.
    if ((world->particleFlagsUnion & m2_powderParticle) != 0)
    {
        float powder = world->particlePowderStrength * (diameter * invDt);
        float minWeight = 1.0f - 0.75f; // one minus the particle stride
        for (int32_t k = 0; k < pairCount; ++k)
        {
            if ((world->particlePairFlags[k] & m2_powderParticle) == 0)
            {
                continue;
            }
            float w = world->particlePairWeight[k];
            if (w <= minWeight)
            {
                continue;
            }
            int32_t a = world->particlePairA[k];
            int32_t b = world->particlePairB[k];
            m2Vec2 n = world->particlePairNormal[k];
            float f = powder * (w - minWeight);
            world->particleVelocities[a].x -= f * n.x;
            world->particleVelocities[a].y -= f * n.y;
            world->particleVelocities[b].x += f * n.x;
            world->particleVelocities[b].y += f * n.y;
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
        // Powder and tensile particles produce no dynamic pressure
        // (the reference's k_noPressureFlags): they carry their own
        // repulsion and cohesion laws instead.
        if ((world->particleFlags[i] & (m2_powderParticle | m2_tensileParticle)) != 0)
        {
            world->particleAccumulation[i] = 0.0f;
        }
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

    // Jelly, late for stability like the reference: elastic triads
    // pull toward their spawn shape through a best-fit rotation, then
    // springs pull toward their spawn lengths. Both read predicted
    // positions (p + dt*v) and correct velocities.
    if (world->particleTriadCount > 0)
    {
        float strength = invDt * world->particleElasticStrength;
        for (int32_t k = 0; k < world->particleTriadCount; ++k)
        {
            int32_t a = world->particleTriadA[k];
            int32_t b = world->particleTriadB[k];
            int32_t c = world->particleTriadC[k];
            m2Vec2 oa = world->particleTriadPA[k];
            m2Vec2 ob = world->particleTriadPB[k];
            m2Vec2 oc = world->particleTriadPC[k];
            double pax =
                world->particlePositions[a].x + (double)(dt * world->particleVelocities[a].x);
            double pay =
                world->particlePositions[a].y + (double)(dt * world->particleVelocities[a].y);
            double pbx =
                world->particlePositions[b].x + (double)(dt * world->particleVelocities[b].x);
            double pby =
                world->particlePositions[b].y + (double)(dt * world->particleVelocities[b].y);
            double pcx =
                world->particlePositions[c].x + (double)(dt * world->particleVelocities[c].x);
            double pcy =
                world->particlePositions[c].y + (double)(dt * world->particleVelocities[c].y);
            double midx = (pax + pbx + pcx) / 3.0;
            double midy = (pay + pby + pcy) / 3.0;
            float ax = (float)(pax - midx);
            float ay = (float)(pay - midy);
            float bx = (float)(pbx - midx);
            float by = (float)(pby - midy);
            float cx = (float)(pcx - midx);
            float cy = (float)(pcy - midy);
            float rs = oa.x * ay - oa.y * ax + (ob.x * by - ob.y * bx) + (oc.x * cy - oc.y * cx);
            float rc = oa.x * ax + oa.y * ay + (ob.x * bx + ob.y * by) + (oc.x * cx + oc.y * cy);
            float r2 = rs * rs + rc * rc;
            if (r2 <= 1.0e-12f)
            {
                continue; // degenerate triad this step: never a NaN
            }
            float inv = 1.0f / sqrtf(r2);
            rs *= inv;
            rc *= inv;
            world->particleVelocities[a].x += strength * (rc * oa.x - rs * oa.y - ax);
            world->particleVelocities[a].y += strength * (rs * oa.x + rc * oa.y - ay);
            world->particleVelocities[b].x += strength * (rc * ob.x - rs * ob.y - bx);
            world->particleVelocities[b].y += strength * (rs * ob.x + rc * ob.y - by);
            world->particleVelocities[c].x += strength * (rc * oc.x - rs * oc.y - cx);
            world->particleVelocities[c].y += strength * (rs * oc.x + rc * oc.y - cy);
        }
    }
    if (world->particleSpringCount > 0)
    {
        float strength = invDt * world->particleSpringStrength;
        for (int32_t k = 0; k < world->particleSpringCount; ++k)
        {
            int32_t a = world->particleSpringA[k];
            int32_t b = world->particleSpringB[k];
            double pax =
                world->particlePositions[a].x + (double)(dt * world->particleVelocities[a].x);
            double pay =
                world->particlePositions[a].y + (double)(dt * world->particleVelocities[a].y);
            double pbx =
                world->particlePositions[b].x + (double)(dt * world->particleVelocities[b].x);
            double pby =
                world->particlePositions[b].y + (double)(dt * world->particleVelocities[b].y);
            float dx = (float)(pbx - pax);
            float dy = (float)(pby - pay);
            float r1 = sqrtf(dx * dx + dy * dy);
            if (r1 <= 1.0e-6f)
            {
                continue;
            }
            float f = strength * (world->particleSpringRest[k] - r1) / r1;
            world->particleVelocities[a].x -= f * dx;
            world->particleVelocities[a].y -= f * dy;
            world->particleVelocities[b].x += f * dx;
            world->particleVelocities[b].y += f * dy;
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

// Fill a convex polygon with particles at the reference stride,
// row-major from the bottom-left of its bounds: a pool in one call.
int32_t m2World_FillPolygonWithParticles(m2WorldId worldId, const m2Polygon* polygon,
                                         m2Pos2 position, m2Vec2 velocity, uint32_t flags)
{
    m2World* world = m2World_GetInternal(worldId);
    if (world == NULL || polygon == NULL || polygon->count < 3 || world->particleCapacity == 0)
    {
        M2_ASSERT(false);
        return 0;
    }
    float stride = 0.75f * 2.0f * world->particleRadius;
    float minX = polygon->vertices[0].x;
    float minY = polygon->vertices[0].y;
    float maxX = minX;
    float maxY = minY;
    for (int32_t i = 1; i < polygon->count; ++i)
    {
        minX = m2MinF(minX, polygon->vertices[i].x);
        minY = m2MinF(minY, polygon->vertices[i].y);
        maxX = m2MaxF(maxX, polygon->vertices[i].x);
        maxY = m2MaxF(maxY, polygon->vertices[i].y);
    }
    if ((maxX - minX) / stride >= (float)M2_FILL_COLUMNS)
    {
        M2_ASSERT(false); // wider than the fill lattice allows
        return 0;
    }

    // The whole fill is ONE journal op (inner emits suppressed, the
    // chain-create precedent): replay must rebuild the springs and
    // triads too, and those are captured here, not in the emits.
    uint8_t journalWas = world->journalActive;
    world->journalActive = 0;

    int32_t* emitted = (int32_t*)world->particleProxies; // borrowed scratch
    int32_t prevRow[M2_FILL_COLUMNS];
    int32_t currRow[M2_FILL_COLUMNS];
    for (int32_t i = 0; i < M2_FILL_COLUMNS; ++i)
    {
        prevRow[i] = -1;
        currRow[i] = -1;
    }
    bool wantSprings = (flags & m2_springParticle) != 0;
    bool wantTriads = (flags & m2_elasticParticle) != 0;
    int32_t count = 0;
    bool full = false;
    for (float y = minY + 0.5f * stride; y < maxY && !full; y += stride)
    {
        for (int32_t i = 0; i < M2_FILL_COLUMNS; ++i)
        {
            currRow[i] = -1;
        }
        int32_t col = 0;
        for (float x = minX + 0.5f * stride; x < maxX && !full; x += stride, ++col)
        {
            bool inside = true;
            for (int32_t i = 0; i < polygon->count && inside; ++i)
            {
                m2Vec2 v = polygon->vertices[i];
                m2Vec2 n = polygon->normals[i];
                inside = n.x * (x - v.x) + n.y * (y - v.y) <= 0.0f;
            }
            if (!inside)
            {
                continue;
            }
            m2ParticleId id = m2World_EmitParticle(
                worldId, (m2Pos2){position.x + (double)x, position.y + (double)y}, velocity, flags);
            if (id.index1 == 0)
            {
                full = true; // pool full: a quiet runtime fact
                break;
            }
            int32_t slot = id.index1 - 1;
            emitted[count] = slot;
            count += 1;
            currRow[col] = slot;
            if (wantTriads)
            {
                // Two triangles per complete lattice cell, rest shape
                // centered on each triad's spawn centroid.
                int32_t left = col > 0 ? currRow[col - 1] : -1;
                int32_t below = prevRow[col];
                int32_t belowLeft = col > 0 ? prevRow[col - 1] : -1;
                if (left >= 0 && below >= 0 && belowLeft >= 0 &&
                    world->particleTriadCount + 2 <= world->particleTriadCapacity)
                {
                    int32_t t = world->particleTriadCount;
                    world->particleTriadA[t] = belowLeft;
                    world->particleTriadB[t] = below;
                    world->particleTriadC[t] = left;
                    world->particleTriadPA[t] = (m2Vec2){-stride / 3.0f, -stride / 3.0f};
                    world->particleTriadPB[t] = (m2Vec2){2.0f * stride / 3.0f, -stride / 3.0f};
                    world->particleTriadPC[t] = (m2Vec2){-stride / 3.0f, 2.0f * stride / 3.0f};
                    world->particleTriadA[t + 1] = below;
                    world->particleTriadB[t + 1] = slot;
                    world->particleTriadC[t + 1] = left;
                    world->particleTriadPA[t + 1] = (m2Vec2){stride / 3.0f, -2.0f * stride / 3.0f};
                    world->particleTriadPB[t + 1] = (m2Vec2){stride / 3.0f, stride / 3.0f};
                    world->particleTriadPC[t + 1] = (m2Vec2){-2.0f * stride / 3.0f, stride / 3.0f};
                    world->particleTriadCount = t + 2;
                }
            }
        }
        for (int32_t i = 0; i < M2_FILL_COLUMNS; ++i)
        {
            prevRow[i] = currRow[i];
        }
    }

    if (wantSprings)
    {
        // Every batch pair inside one diameter becomes a spring that
        // remembers its spawn length; ascending order, canonical.
        float diameter = 2.0f * world->particleRadius;
        for (int32_t i = 0; i < count; ++i)
        {
            for (int32_t j = i + 1; j < count; ++j)
            {
                int32_t a = emitted[i];
                int32_t b = emitted[j];
                float dx = (float)(world->particlePositions[b].x - world->particlePositions[a].x);
                float dy = (float)(world->particlePositions[b].y - world->particlePositions[a].y);
                float distSq = dx * dx + dy * dy;
                if (distSq >= diameter * diameter || distSq <= 0.0f)
                {
                    continue;
                }
                if (world->particleSpringCount >= world->particleSpringCapacity)
                {
                    continue; // deterministic truncation, documented
                }
                int32_t k = world->particleSpringCount;
                world->particleSpringA[k] = a;
                world->particleSpringB[k] = b;
                world->particleSpringRest[k] = sqrtf(distSq);
                world->particleSpringCount = k + 1;
            }
        }
    }

    world->journalActive = journalWas;
    if (world->journalActive != 0)
    {
        struct
        {
            m2Polygon polygon;
            m2Pos2 position;
            m2Vec2 velocity;
            uint32_t flags;
            int32_t expected;
        } record;
        memset(&record, 0, sizeof(record));
        record.polygon = *polygon;
        record.position = position;
        record.velocity = velocity;
        record.flags = flags;
        record.expected = count;
        m2JournalRecord(world, m2_opFillParticles, &record, (int32_t)sizeof(record));
    }
    return count;
}

// Region query over the pool: a plain ascending scan. Particles
// carry no tree (their grid is step-transient); an honest linear
// walk over a fixed-capacity pool is deterministic and cheap.
int32_t m2World_OverlapParticlesAABB(m2WorldId worldId, m2Pos2 lower, m2Pos2 upper,
                                     m2ParticleId* ids, int32_t capacity)
{
    m2World* world = m2World_GetInternal(worldId);
    if (world == NULL || world->particleCapacity == 0)
    {
        return 0;
    }
    int32_t total = 0;
    for (int32_t i = 0; i < world->maxParticleIndex; ++i)
    {
        if (world->particleAlive[i] == 0)
        {
            continue;
        }
        m2Pos2 p = world->particlePositions[i];
        if (p.x < lower.x || p.x > upper.x || p.y < lower.y || p.y > upper.y)
        {
            continue;
        }
        if (ids != NULL && total < capacity)
        {
            ids[total] = (m2ParticleId){i + 1, worldId.index1, world->particleGenerations[i]};
        }
        total += 1;
    }
    return total;
}
