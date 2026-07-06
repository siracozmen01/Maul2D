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
    if (dt <= 0.0f)
    {
        return;
    }
    float invDt = 1.0f / dt;
    float diameter = 2.0f * world->particleRadius;
    int32_t pairCount = world->particlePairCount;

    // Dimensionless density: the sum of contact weights.
    for (int32_t i = 0; i < world->maxParticleIndex; ++i)
    {
        world->particleWeights[i] = 0.0f;
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
