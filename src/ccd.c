// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Bullet continuous collision (topic-07 D2/D3): conservative advancement
// per substep for bullet-flagged bodies. Deterministic by construction:
// bullets advance in body-index order, candidates come from tree queries
// filtered and processed in canonical shape order, iteration counts are
// capped, and hitting the cap is documented behavior (slight overlap
// next substep, which the speculative solver then resolves). Bullets do
// not sweep against other bullets (F-T7-1, reference behavior).
//
// v1 simplifications, recorded: the bullet sweeps as its bounding circle
// (exact for circle shapes, conservative for the rest), and rotation
// during the sweep is ignored (F-T7-3 territory).

#include "world_internal.h"

#include "maul2d/base.h"

#define M2_TOI_ITERATIONS 20
#define M2_TOI_TARGET     0.005f // stop one slop short of the surface
#define M2_TOI_CANDIDATES 64

static float BulletBoundingRadius(const m2World* world, int32_t body)
{
    float radius = 0.0f;
    for (int32_t s = world->bodyShapeHead[body]; s != -1; s = world->shapeNext[s])
    {
        const m2ShapeGeometry* g = &world->shapeGeometry[s];
        switch (g->type)
        {
        case m2_circleShape:
        {
            float r = sqrtf(g->circle.center.x * g->circle.center.x +
                            g->circle.center.y * g->circle.center.y) +
                      g->circle.radius;
            radius = m2MaxF(radius, r);
            break;
        }
        case m2_capsuleShape:
        {
            float r1 = sqrtf(g->capsule.point1.x * g->capsule.point1.x +
                             g->capsule.point1.y * g->capsule.point1.y);
            float r2 = sqrtf(g->capsule.point2.x * g->capsule.point2.x +
                             g->capsule.point2.y * g->capsule.point2.y);
            radius = m2MaxF(radius, m2MaxF(r1, r2) + g->capsule.radius);
            break;
        }
        default:
        {
            for (int32_t v = 0; v < g->polygon.count; ++v)
            {
                float r = sqrtf(g->polygon.vertices[v].x * g->polygon.vertices[v].x +
                                g->polygon.vertices[v].y * g->polygon.vertices[v].y);
                radius = m2MaxF(radius, r + g->polygon.radius);
            }
            break;
        }
        }
    }
    return radius;
}

// Distance from the bullet's bounding circle (center at world point wp)
// to a target shape, in the target body's frame (single f64 crossing).
static float DistanceToShape(const m2World* world, int32_t shape, m2Pos2 wp, float bulletRadius)
{
    int32_t body = world->shapeBody[shape];
    m2Transform xf = world->transforms[body];
    float dx = (float)(wp.x - xf.p.x);
    float dy = (float)(wp.y - xf.p.y);
    m2Vec2 local = {xf.q.c * dx + xf.q.s * dy, -xf.q.s * dx + xf.q.c * dy};
    return m2PointShapeDistance(&world->shapeGeometry[shape], local) - bulletRadius;
}

// Sweep one bullet from p0 toward its current position; clamp on impact.
static void SweepBullet(m2World* world, int32_t body, m2Pos2 p0)
{
    m2Pos2 p1 = world->transforms[body].p;
    double mx = p1.x - p0.x;
    double my = p1.y - p0.y;
    float moveLen = sqrtf((float)(mx * mx + my * my));
    float bulletRadius = BulletBoundingRadius(world, body);
    if (moveLen < 0.5f * bulletRadius || bulletRadius == 0.0f)
    {
        return; // slow enough for the speculative pipeline
    }

    // Swept AABB over the whole motion, fattened by the bounding circle.
    m2AABB sweep;
    sweep.lowerBound.x = (p0.x < p1.x ? p0.x : p1.x) - (double)bulletRadius;
    sweep.lowerBound.y = (p0.y < p1.y ? p0.y : p1.y) - (double)bulletRadius;
    sweep.upperBound.x = (p0.x > p1.x ? p0.x : p1.x) + (double)bulletRadius;
    sweep.upperBound.y = (p0.y > p1.y ? p0.y : p1.y) + (double)bulletRadius;

    int32_t candidates[M2_TOI_CANDIDATES];
    int32_t candidateCount = 0;
    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        int32_t results[M2_TOI_CANDIDATES];
        int32_t hits =
            m2Tree_Query(&world->trees[t], world->treeNodes[t], sweep, results, M2_TOI_CANDIDATES);
        hits = hits <= M2_TOI_CANDIDATES ? hits : M2_TOI_CANDIDATES;
        for (int32_t h = 0; h < hits && candidateCount < M2_TOI_CANDIDATES; ++h)
        {
            int32_t shape = results[h];
            int32_t other = world->shapeBody[shape];
            if (other == body || world->bullets[other] != 0)
            {
                continue; // self, or bullet-vs-bullet (excluded, F-T7-1)
            }
            // Collision filters apply to bullets too: keep the
            // candidate only if some bullet shape may hit it.
            bool mayCollide = false;
            for (int32_t bs = world->bodyShapeHead[body]; bs != -1; bs = world->shapeNext[bs])
            {
                mayCollide =
                    mayCollide || ((world->shapeCategory[bs] & world->shapeMask[shape]) != 0 &&
                                   (world->shapeCategory[shape] & world->shapeMask[bs]) != 0);
            }
            if (!mayCollide)
            {
                continue;
            }
            candidates[candidateCount++] = shape;
        }
    }
    if (candidateCount == 0)
    {
        return;
    }
    // Canonical order regardless of tree traversal.
    for (int32_t i = 1; i < candidateCount; ++i)
    {
        int32_t key = candidates[i];
        int32_t j = i - 1;
        while (j >= 0 && candidates[j] > key)
        {
            candidates[j + 1] = candidates[j];
            j -= 1;
        }
        candidates[j + 1] = key;
    }

    // Conservative advancement to the earliest impact across candidates.
    float minT = 1.0f;
    for (int32_t c = 0; c < candidateCount; ++c)
    {
        float t = 0.0f;
        for (int32_t iter = 0; iter < M2_TOI_ITERATIONS; ++iter)
        {
            m2Pos2 wp = {p0.x + (double)t * mx, p0.y + (double)t * my};
            float distance = DistanceToShape(world, candidates[c], wp, bulletRadius);
            if (distance < M2_TOI_TARGET)
            {
                if (t < minT)
                {
                    minT = t;
                }
                break;
            }
            float advance = (distance - 0.5f * M2_TOI_TARGET) / moveLen;
            t += advance;
            if (t >= minT || t >= 1.0f)
            {
                break; // no earlier impact on this candidate
            }
        }
    }

    if (minT < 1.0f)
    {
        // Clamp position to the impact time. Velocity is left alone: the
        // next substep's speculative contact resolves the collision.
        world->transforms[body].p = (m2Pos2){p0.x + (double)minT * mx, p0.y + (double)minT * my};
        world->deltaPositions[body].x -= (1.0f - minT) * (float)mx;
        world->deltaPositions[body].y -= (1.0f - minT) * (float)my;
    }
}

// Called after each substep's position integration (the last
// transform-mutating pass ordering, registry M13).
void m2SolveContinuous(m2World* world)
{
    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        if (world->alive[i] == 0 || world->bullets[i] == 0 || world->asleep[i] != 0 ||
            world->types[i] != (uint8_t)m2_dynamicBody)
        {
            continue;
        }
        SweepBullet(world, i, world->ccdPrevPositions[i]);
    }
}
