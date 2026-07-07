// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Particle-free water: buoyancy volumes. An axis-aligned activation
// region gates which awake dynamic bodies are in the water; a
// horizontal surface line decides how much of each shape is below
// it. The physics is the LiquidFun buoyancy controller (see
// THIRD_PARTY.md): Archimedes lift on the submerged area at its
// centroid, plus linear and angular drag, plus an optional flow
// current. The submerged area is exact for circles and polygons and
// a bounding-box fraction for capsules and segments, a documented
// approximation for the rounded and thin shapes that rarely float.
//
// Forces feed the ordinary per-body accumulators, so they integrate
// alongside gravity and die with the step. Only awake dynamic
// bodies are touched, exactly as gravity is, so a body that settles
// at the waterline and sleeps simply floats.

#include "world_internal.h"

#include "maul2d/base.h"

#include <math.h>

// acos(x) = atan2(sqrt(1 - x^2), x), built on the engine's own
// deterministic atan2 because libm acosf is not bit-identical
// across platforms (the determinism contract, ADR-0010).
static float FvAcos(float x)
{
    float s = 1.0f - x * x;
    s = s > 0.0f ? sqrtf(s) : 0.0f;
    return m2Atan2(s, x);
}

static m2Vec2 FvRotate(m2Rot q, m2Vec2 v)
{
    return (m2Vec2){q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y};
}

// Submerged area and world centroid of a circle below y = surface.
static float SubmergedCircle(m2Vec2 center, float radius, double surface, m2Vec2* centroidOut)
{
    float depth = (float)(surface - (double)center.y); // >0: center is below the line
    if (depth >= radius)
    {
        *centroidOut = center;
        return (float)M2_PI * radius * radius;
    }
    if (depth <= -radius)
    {
        return 0.0f;
    }
    // The dry cap sits above the line at signed height depth from the
    // center; the submerged area is the disc minus that cap.
    float r2 = radius * radius;
    float root = sqrtf(r2 - depth * depth);
    float dryArea = r2 * FvAcos(depth / radius) - depth * root;
    float area = (float)M2_PI * r2 - dryArea;
    if (area <= 0.0f)
    {
        return 0.0f;
    }
    // Segment centroids: the dry cap centroid sits above the center by
    // (2/3) root^3 / dryArea; the submerged centroid balances it.
    float dryCentroidY = dryArea > 1.0e-9f ? (2.0f / 3.0f) * root * root * root / dryArea : 0.0f;
    float subCentroidY = -dryCentroidY * dryArea / area;
    *centroidOut = (m2Vec2){center.x, center.y + subCentroidY};
    return area;
}

// Signed area and area-weighted centroid of a polygon (world verts).
static float PolygonAreaCentroid(const m2Vec2* v, int32_t n, m2Vec2* centroidOut)
{
    float area2 = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;
    for (int32_t i = 0; i < n; ++i)
    {
        m2Vec2 a = v[i];
        m2Vec2 b = v[(i + 1) % n];
        float cross = a.x * b.y - b.x * a.y;
        area2 += cross;
        cx += (a.x + b.x) * cross;
        cy += (a.y + b.y) * cross;
    }
    float area = 0.5f * area2;
    if (area > 1.0e-9f || area < -1.0e-9f)
    {
        cx /= 3.0f * area2;
        cy /= 3.0f * area2;
    }
    *centroidOut = (m2Vec2){cx, cy};
    return area < 0.0f ? -area : area;
}

// Submerged area of a convex polygon below y = surface: clip the
// polygon by the half-plane, then measure the clipped piece.
static float SubmergedPolygon(const m2Vec2* verts, int32_t count, double surface,
                              m2Vec2* centroidOut)
{
    m2Vec2 clipped[2 * M2_MAX_POLYGON_VERTICES];
    int32_t out = 0;
    for (int32_t i = 0; i < count; ++i)
    {
        m2Vec2 a = verts[i];
        m2Vec2 b = verts[(i + 1) % count];
        bool aIn = (double)a.y <= surface;
        bool bIn = (double)b.y <= surface;
        if (aIn)
        {
            clipped[out++] = a;
        }
        if (aIn != bIn)
        {
            float t = (float)((surface - (double)a.y) / ((double)b.y - (double)a.y));
            clipped[out++] = (m2Vec2){a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
        }
    }
    if (out < 3)
    {
        return 0.0f;
    }
    return PolygonAreaCentroid(clipped, out, centroidOut);
}

// One shape's submerged area and world centroid.
static float SubmergedShape(const m2World* world, int32_t shape, m2Transform xf, double surface,
                            m2Vec2* centroidOut)
{
    const m2ShapeGeometry* g = &world->shapeGeometry[shape];
    switch (g->type)
    {
    case m2_circleShape:
    {
        m2Vec2 c = FvRotate(xf.q, g->circle.center);
        c.x += (float)xf.p.x;
        c.y += (float)xf.p.y;
        return SubmergedCircle(c, g->circle.radius, surface, centroidOut);
    }
    case m2_polygonShape:
    {
        m2Vec2 verts[M2_MAX_POLYGON_VERTICES];
        for (int32_t i = 0; i < g->polygon.count; ++i)
        {
            m2Vec2 w = FvRotate(xf.q, g->polygon.vertices[i]);
            verts[i] = (m2Vec2){w.x + (float)xf.p.x, w.y + (float)xf.p.y};
        }
        return SubmergedPolygon(verts, g->polygon.count, surface, centroidOut);
    }
    default:
        break;
    }
    // Capsule, segment, chain segment: a bounding-box fraction. The
    // rounded and thin shapes rarely float and never need the exact
    // waterline; the box keeps the result finite and monotone.
    m2AABB box = m2ComputeShapeAABB(g, xf);
    double lo = box.lowerBound.y;
    double hi = box.upperBound.y;
    if (surface <= lo || hi <= lo)
    {
        return 0.0f;
    }
    float frac = surface >= hi ? 1.0f : (float)((surface - lo) / (hi - lo));
    float fullArea = m2ShapeArea(g);
    float area = fullArea * frac;
    float midY = (float)(lo + 0.5 * (double)frac * (hi - lo));
    *centroidOut = (m2Vec2){(float)(0.5 * (box.lowerBound.x + box.upperBound.x)), midY};
    return area;
}

m2FluidVolumeDef m2DefaultFluidVolumeDef(void)
{
    m2FluidVolumeDef def;
    memset(&def, 0, sizeof(def));
    def.density = 2.0f;
    def.linearDrag = 1.0f;
    def.angularDrag = 0.5f;
    def.internalValue = M2_FVOLUME_COOKIE;
    return def;
}

m2FluidVolumeId m2World_CreateFluidVolume(m2WorldId worldId, const m2FluidVolumeDef* def)
{
    m2World* world = m2World_GetInternal(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_FVOLUME_COOKIE ||
        world->fvCapacity == 0)
    {
        M2_ASSERT(false);
        return m2_nullFluidVolumeId;
    }
    if (!(def->density >= 0.0f) || !(def->linearDrag >= 0.0f) || !(def->angularDrag >= 0.0f) ||
        !(def->surface == def->surface))
    {
        M2_ASSERT(false);
        return m2_nullFluidVolumeId;
    }
    if (world->fvFreeCount == 0)
    {
        return m2_nullFluidVolumeId; // pool full: a runtime fact
    }
    int32_t index = world->fvFreeQueue[world->fvFreeHead];
    world->fvFreeHead = (world->fvFreeHead + 1) % world->fvCapacity;
    world->fvFreeCount -= 1;
    if (index + 1 > world->maxFvIndex)
    {
        world->maxFvIndex = index + 1;
    }
    world->fvLower[index] = def->regionLower;
    world->fvUpper[index] = def->regionUpper;
    world->fvSurface[index] = def->surface;
    world->fvDensity[index] = def->density;
    world->fvLinearDrag[index] = def->linearDrag;
    world->fvAngularDrag[index] = def->angularDrag;
    world->fvFlow[index] = def->flow;
    world->fvUserData[index] = def->userData;
    world->fvAlive[index] = 1;
    m2FluidVolumeId id = {index + 1, worldId.index1, world->fvGenerations[index]};
    if (world->journalActive != 0)
    {
        struct
        {
            m2FluidVolumeDef def;
            m2FluidVolumeId expected;
        } record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = id;
        m2JournalRecord(world, m2_opCreateFluidVolume, &record, (int32_t)sizeof(record));
    }
    return id;
}

static int32_t FvSlot(const m2World* world, m2FluidVolumeId id)
{
    int32_t index = id.index1 - 1;
    if (world == NULL || index < 0 || index >= world->fvCapacity || world->fvAlive[index] == 0 ||
        world->fvGenerations[index] != id.generation)
    {
        return -1;
    }
    return index;
}

void m2World_DestroyFluidVolume(m2FluidVolumeId volumeId)
{
    m2World* world = m2WorldFromIndex0(volumeId.world0);
    int32_t index = FvSlot(world, volumeId);
    if (index < 0)
    {
        M2_ASSERT(false);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2FluidVolumeId id;
        } record;
        memset(&record, 0, sizeof(record));
        record.id = volumeId;
        m2JournalRecord(world, m2_opDestroyFluidVolume, &record, (int32_t)sizeof(record));
    }
    world->fvAlive[index] = 0;
    world->fvGenerations[index] += 1;
    world->fvFreeQueue[(world->fvFreeHead + world->fvFreeCount) % world->fvCapacity] = index;
    world->fvFreeCount += 1;
}

bool m2FluidVolume_IsValid(m2FluidVolumeId volumeId)
{
    m2World* world = m2WorldFromIndex0(volumeId.world0);
    return FvSlot(world, volumeId) >= 0;
}

void m2FluidVolume_SetSurface(m2FluidVolumeId volumeId, double surface)
{
    m2World* world = m2WorldFromIndex0(volumeId.world0);
    int32_t index = FvSlot(world, volumeId);
    if (index < 0 || !(surface == surface))
    {
        M2_ASSERT(false);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2FluidVolumeId id;
            double surface;
        } record;
        memset(&record, 0, sizeof(record));
        record.id = volumeId;
        record.surface = surface;
        m2JournalRecord(world, m2_opSetFluidSurface, &record, (int32_t)sizeof(record));
    }
    world->fvSurface[index] = surface;
}

double m2FluidVolume_GetSurface(m2FluidVolumeId volumeId)
{
    m2World* world = m2WorldFromIndex0(volumeId.world0);
    int32_t index = FvSlot(world, volumeId);
    return index >= 0 ? world->fvSurface[index] : 0.0;
}

uint64_t m2FluidVolume_GetUserData(m2FluidVolumeId volumeId)
{
    m2World* world = m2WorldFromIndex0(volumeId.world0);
    int32_t index = FvSlot(world, volumeId);
    return index >= 0 ? world->fvUserData[index] : 0;
}

// The per-step pass: buoyancy and drag into the force accumulators,
// in canonical volume-then-body order.
void m2ApplyFluidVolumes(m2World* world, float dt)
{
    if (dt <= 0.0f)
    {
        return;
    }
    for (int32_t v = 0; v < world->maxFvIndex; ++v)
    {
        if (world->fvAlive[v] == 0)
        {
            continue;
        }
        m2Pos2 lo = world->fvLower[v];
        m2Pos2 hi = world->fvUpper[v];
        double surface = world->fvSurface[v];
        float density = world->fvDensity[v];
        float linearDrag = world->fvLinearDrag[v];
        float angularDrag = world->fvAngularDrag[v];
        m2Vec2 flow = world->fvFlow[v];

        for (int32_t b = 0; b < world->maxBodyIndex; ++b)
        {
            if (world->alive[b] == 0 || world->types[b] != (uint8_t)m2_dynamicBody ||
                world->asleep[b] != 0 || world->disabled[b] != 0)
            {
                continue;
            }
            m2Transform xf = world->transforms[b];
            // Region gate: the body origin must sit in the activation
            // box (cheap and canonical; the surface does the physics).
            if (xf.p.x < lo.x || xf.p.x > hi.x || xf.p.y < lo.y || xf.p.y > hi.y)
            {
                continue;
            }

            m2Vec2 lc = world->localCenters[b];
            m2Vec2 comArm = FvRotate(xf.q, lc);
            float comX = (float)xf.p.x + comArm.x;
            float comY = (float)xf.p.y + comArm.y;

            float totalArea = 0.0f;
            m2Vec2 areaCentroid = {0.0f, 0.0f};
            for (int32_t s = world->bodyShapeHead[b]; s != -1; s = world->shapeNext[s])
            {
                m2Vec2 c = {0.0f, 0.0f};
                float area = SubmergedShape(world, s, xf, surface, &c);
                if (area <= 0.0f)
                {
                    continue;
                }
                totalArea += area;
                areaCentroid.x += area * c.x;
                areaCentroid.y += area * c.y;
            }
            if (totalArea <= 1.19209290e-7f)
            {
                continue;
            }
            areaCentroid.x /= totalArea;
            areaCentroid.y /= totalArea;

            // Buoyancy: displaced weight, up, at the submerged centroid.
            float bfx = -density * totalArea * world->gravity.x;
            float bfy = -density * totalArea * world->gravity.y;
            // Drag: opposes the centroid's velocity relative to the flow.
            float w = world->angularVelocities[b];
            float rcx = areaCentroid.x - comX;
            float rcy = areaCentroid.y - comY;
            float vcx = world->linearVelocities[b].x - w * rcy;
            float vcy = world->linearVelocities[b].y + w * rcx;
            float dfx = -linearDrag * totalArea * (vcx - flow.x);
            float dfy = -linearDrag * totalArea * (vcy - flow.y);

            float fx = bfx + dfx;
            float fy = bfy + dfy;
            world->forces[b].x += fx;
            world->forces[b].y += fy;
            world->torques[b] += rcx * fy - rcy * fx;
            // Angular drag scales with the reference's inertia-per-mass.
            float invM = world->invMass[b];
            float invI = world->invInertia[b];
            if (invM > 0.0f && invI > 0.0f)
            {
                float inertiaPerMass = invM / invI;
                world->torques[b] -= inertiaPerMass * totalArea * w * angularDrag;
            }
        }
    }
}

// Global wind: an area-weighted linear drag toward the world wind
// velocity, into the same force accumulators as gravity and buoyancy,
// in canonical body order. Runs on the FULL shape area (wind acts in
// air, not on a submerged part) and only on dynamic awake enabled
// bodies; frozen bodies skip it exactly as they skip gravity. Uniform
// wind is applied at the center of mass, so it adds no spurious torque.
// The world gates on a positive drag before calling this (opt-in).
void m2ApplyWind(m2World* world, float dt)
{
    if (dt <= 0.0f || !(world->windLinearDrag > 0.0f))
    {
        return;
    }
    float drag = world->windLinearDrag;
    m2Vec2 wind = world->windVelocity;
    for (int32_t b = 0; b < world->maxBodyIndex; ++b)
    {
        if (world->alive[b] == 0 || world->types[b] != (uint8_t)m2_dynamicBody ||
            world->asleep[b] != 0 || world->disabled[b] != 0)
        {
            continue;
        }
        // Full shape area is rotation invariant; mass at unit density
        // reuses the tested area math (circles, rounded polygons, caps).
        float area = 0.0f;
        for (int32_t s = world->bodyShapeHead[b]; s != -1; s = world->shapeNext[s])
        {
            area += m2ComputeShapeMass(&world->shapeGeometry[s], 1.0f).mass;
        }
        if (!(area > 0.0f))
        {
            continue;
        }
        m2Vec2 v = world->linearVelocities[b];
        world->forces[b].x += -drag * area * (v.x - wind.x);
        world->forces[b].y += -drag * area * (v.y - wind.y);
    }
}
