// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// World core: bodies, shapes, gravity integration, broadphase update,
// snapshot/restore/hash. Every array is POD and lives in the snapshot;
// the rollback gate byte-compares all of it. Allocation goes through
// malloc for now - the world-def allocator hooks are a recorded pending
// item, due before any public release (topic-10).

#include "world_internal.h"

#include "maul2d/base.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define M2_MAX_WORLDS       16
#define M2_WORLD_COOKIE     (M2_COOKIE ^ ((int32_t)sizeof(m2WorldDef) << 8) ^ 1)
#define M2_BODY_COOKIE      (M2_COOKIE ^ ((int32_t)sizeof(m2BodyDef) << 8) ^ 2)
#define M2_SHAPE_COOKIE     (M2_COOKIE ^ ((int32_t)sizeof(m2ShapeDef) << 8) ^ 3)
#define M2_DJOINT_COOKIE    (M2_COOKIE ^ ((int32_t)sizeof(m2DistanceJointDef) << 8) ^ 4)
#define M2_RJOINT_COOKIE    (M2_COOKIE ^ ((int32_t)sizeof(m2RevoluteJointDef) << 8) ^ 5)
#define M2_PJOINT_COOKIE    (M2_COOKIE ^ ((int32_t)sizeof(m2PrismaticJointDef) << 8) ^ 6)
#define M2_WJOINT_COOKIE    (M2_COOKIE ^ ((int32_t)sizeof(m2WeldJointDef) << 8) ^ 7)
#define M2_WHJOINT_COOKIE   (M2_COOKIE ^ ((int32_t)sizeof(m2WheelJointDef) << 8) ^ 8)
#define M2_CHAIN_COOKIE     (M2_COOKIE ^ ((int32_t)sizeof(m2ChainDef) << 8) ^ 9)
#define M2_FJOINT_COOKIE    (M2_COOKIE ^ ((int32_t)sizeof(m2FilterJointDef) << 8) ^ 10)
#define M2_MOJOINT_COOKIE   (M2_COOKIE ^ ((int32_t)sizeof(m2MotorJointDef) << 8) ^ 11)
#define M2_MSJOINT_COOKIE   (M2_COOKIE ^ ((int32_t)sizeof(m2MouseJointDef) << 8) ^ 12)
#define M2_EXPLODE_COOKIE   (M2_COOKIE ^ ((int32_t)sizeof(m2ExplosionDef) << 8) ^ 13)
#define M2_GJOINT_COOKIE    (M2_COOKIE ^ ((int32_t)sizeof(m2GearJointDef) << 8) ^ 14)
#define M2_PLJOINT_COOKIE   (M2_COOKIE ^ ((int32_t)sizeof(m2PulleyJointDef) << 8) ^ 15)
#define M2_RTJOINT_COOKIE   (M2_COOKIE ^ ((int32_t)sizeof(m2RatchetJointDef) << 8) ^ 16)
#define M2_SNAPSHOT_MAGIC   0x4D32534Eu // 'M2SN'
#define M2_SNAPSHOT_VERSION 35u

// Fat margin in meters (topic-02 §3; harness-tuned later, F-T2-1).
#define M2_AABB_MARGIN 0.1

static m2World* s_worlds[M2_MAX_WORLDS];
static uint16_t s_worldGenerations[M2_MAX_WORLDS];

static m2World* GetWorld(m2WorldId id)
{
    if (id.index1 < 1 || id.index1 > M2_MAX_WORLDS)
    {
        return NULL;
    }
    m2World* world = s_worlds[id.index1 - 1];
    if (world == NULL || s_worldGenerations[id.index1 - 1] != id.generation)
    {
        return NULL;
    }
    return world;
}

m2World* m2World_GetInternal(m2WorldId worldId)
{
    return GetWorld(worldId);
}

static m2World* WorldFromIndex(uint16_t world0)
{
    if (world0 < 1 || world0 > M2_MAX_WORLDS)
    {
        return NULL;
    }
    return s_worlds[world0 - 1];
}

static m2World* GetBodyWorld(m2BodyId id)
{
    return WorldFromIndex(id.world0);
}

static int32_t BodySlot(const m2World* world, m2BodyId id)
{
    int32_t index = id.index1 - 1;
    if (index < 0 || index >= world->bodyCapacity)
    {
        return -1;
    }
    if (world->alive[index] == 0 || world->generations[index] != id.generation)
    {
        return -1;
    }
    return index;
}

static int32_t ShapeSlot(const m2World* world, m2ShapeId id)
{
    int32_t index = id.index1 - 1;
    if (index < 0 || index >= world->shapeCapacity)
    {
        return -1;
    }
    if (world->shapeAlive[index] == 0 || world->shapeGenerations[index] != id.generation)
    {
        return -1;
    }
    return index;
}

// --- Broadphase helpers ------------------------------------------------------

static m2AABB Fatten(m2AABB aabb)
{
    aabb.lowerBound.x -= M2_AABB_MARGIN;
    aabb.lowerBound.y -= M2_AABB_MARGIN;
    aabb.upperBound.x += M2_AABB_MARGIN;
    aabb.upperBound.y += M2_AABB_MARGIN;
    return aabb;
}

static m2AABB ShapeTightAABB(const m2World* world, int32_t shapeIndex)
{
    int32_t body = world->shapeBody[shapeIndex];
    return m2ComputeShapeAABB(&world->shapeGeometry[shapeIndex], world->transforms[body]);
}

static int32_t ShapeTreeIndex(const m2World* world, int32_t shapeIndex)
{
    return world->types[world->shapeBody[shapeIndex]];
}

// ALL moved proxies enter the moved set - shapes of dynamic, kinematic,
// and static bodies alike (topic-02 §4.2).
static float RelativeJointAngle(m2Rot qA, m2Rot qB);
static int32_t JointSlotChecked(const m2World* world, m2JointId jointId);
static int32_t TypedJointSlot(m2World* world, m2JointId jointId, uint8_t type);
static float PulleyLiveLength(m2World* world, int32_t index, int32_t side);

static void PushMoved(m2World* world, int32_t shapeIndex)
{
    if (world->inMoved[shapeIndex] != 0)
    {
        return;
    }
    world->inMoved[shapeIndex] = 1;
    world->moved[world->movedCount] = shapeIndex;
    world->movedCount += 1;
}

// Reference default: jointed bodies do not collide. A linear scan is
// fine at Maul's joint counts; worlds without joints skip it via the
// maxJointIndex fast path at the call site.
static bool JointsForbidPair(const m2World* world, int32_t bodyA, int32_t bodyB)
{
    for (int32_t j = 0; j < world->maxJointIndex; ++j)
    {
        if (world->jointAlive[j] == 0 || world->jointCollide[j] != 0)
        {
            continue;
        }
        if ((world->jointBodyA[j] == bodyA && world->jointBodyB[j] == bodyB) ||
            (world->jointBodyA[j] == bodyB && world->jointBodyB[j] == bodyA))
        {
            return true;
        }
    }
    return false;
}

// The filter takes effect through the normal rebuild road: wake both
// ends and push their shapes, and the pair diff emits the M19 ends.
static void RefilterJointedBodies(m2World* world, int32_t bodyA, int32_t bodyB)
{
    if (world->types[bodyA] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[bodyA] = 0;
        world->sleepTimes[bodyA] = 0.0f;
    }
    if (world->types[bodyB] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[bodyB] = 0;
        world->sleepTimes[bodyB] = 0.0f;
    }
    for (int32_t s = world->bodyShapeHead[bodyA]; s != -1; s = world->shapeNext[s])
    {
        PushMoved(world, s);
    }
    for (int32_t s = world->bodyShapeHead[bodyB]; s != -1; s = world->shapeNext[s])
    {
        PushMoved(world, s);
    }
}

static uint64_t PairKey(int32_t a, int32_t b)
{
    uint64_t lo = (uint64_t)(a < b ? a : b);
    uint64_t hi = (uint64_t)(a < b ? b : a);
    return (lo << 32) | hi;
}

static int CompareU64(const void* a, const void* b)
{
    uint64_t ua = *(const uint64_t*)a;
    uint64_t ub = *(const uint64_t*)b;
    return ua < ub ? -1 : (ua > ub ? 1 : 0);
}

static int CompareI32(const void* a, const void* b)
{
    int32_t ia = *(const int32_t*)a;
    int32_t ib = *(const int32_t*)b;
    return ia < ib ? -1 : (ia > ib ? 1 : 0);
}

static bool ShapeIsDynamic(const m2World* world, int32_t shapeIndex)
{
    return world->types[world->shapeBody[shapeIndex]] == (uint8_t)m2_dynamicBody;
}

static m2ShapeId MakeShapeId(const m2World* world, int32_t shapeIndex)
{
    m2ShapeId id = {shapeIndex + 1, world->worldIndex0, world->shapeGenerations[shapeIndex]};
    return id;
}

static void EmitEnd(m2World* world, int32_t shapeA, int32_t shapeB)
{
    if (world->endEventCount >= world->pairCapacity)
    {
        return;
    }
    m2ContactEndEvent* e = &world->endEvents[world->endEventCount++];
    e->shapeIdA = MakeShapeId(world, shapeA);
    e->shapeIdB = MakeShapeId(world, shapeB);
    e->step = world->stepCount;
}

// Shared begin-event geometry: the manifold's world normal, the world
// hit points, and the first-point closing speed, all in body A's frame.
// Used by the solid-contact and the sensor begin streams alike, so a
// sensor overlap reports WHERE it was entered and how fast (b2 #945).
static void FillBeginGeometry(m2World* world, m2ContactBeginEvent* e, int32_t pairIndex,
                              int32_t shapeA, int32_t shapeB)
{
    const m2Manifold* manifold = &world->manifolds[pairIndex];
    int32_t bodyA = world->shapeBody[shapeA];
    int32_t bodyB = world->shapeBody[shapeB];
    m2Transform xfA = world->transforms[bodyA];
    m2Rot qA = xfA.q;
    e->normal = (m2Vec2){qA.c * manifold->normal.x - qA.s * manifold->normal.y,
                         qA.s * manifold->normal.x + qA.c * manifold->normal.y};
    e->pointCount = manifold->pointCount;
    for (int32_t k = 0; k < manifold->pointCount && k < 2; ++k)
    {
        m2Vec2 anchor = manifold->points[k].anchorA;
        e->points[k].x = xfA.p.x + (double)(qA.c * anchor.x - qA.s * anchor.y);
        e->points[k].y = xfA.p.y + (double)(qA.s * anchor.x + qA.c * anchor.y);
    }

    if (manifold->pointCount > 0)
    {
        // Closing speed at the first point: how hard the hit landed.
        m2Vec2 lcA = world->localCenters[bodyA];
        m2Vec2 lcB = world->localCenters[bodyB];
        m2Vec2 anchor = manifold->points[0].anchorA;
        m2Vec2 rA = {qA.c * (anchor.x - lcA.x) - qA.s * (anchor.y - lcA.y),
                     qA.s * (anchor.x - lcA.x) + qA.c * (anchor.y - lcA.y)};
        m2Rot qB = world->transforms[bodyB].q;
        // The same world point measured from B's center of mass.
        m2Vec2 rB = {
            (float)(e->points[0].x - world->transforms[bodyB].p.x) - (qB.c * lcB.x - qB.s * lcB.y),
            (float)(e->points[0].y - world->transforms[bodyB].p.y) - (qB.s * lcB.x + qB.c * lcB.y)};
        m2Vec2 vA = world->linearVelocities[bodyA];
        m2Vec2 vB = world->linearVelocities[bodyB];
        float wA = world->angularVelocities[bodyA];
        float wB = world->angularVelocities[bodyB];
        m2Vec2 velA = {vA.x - wA * rA.y, vA.y + wA * rA.x};
        m2Vec2 velB = {vB.x - wB * rB.y, vB.y + wB * rB.x};
        float vn = (velB.x - velA.x) * e->normal.x + (velB.y - velA.y) * e->normal.y;
        e->approachSpeed = vn < 0.0f ? -vn : 0.0f;
    }
}

static void EmitBegin(m2World* world, int32_t shapeA, int32_t shapeB, int32_t pairIndex)
{
    if (world->beginEventCount >= world->pairCapacity)
    {
        return;
    }
    m2ContactBeginEvent* e = &world->beginEvents[world->beginEventCount++];
    memset(e, 0, sizeof(*e)); // no stack garbage in observer payloads
    e->shapeIdA = MakeShapeId(world, shapeA);
    e->shapeIdB = MakeShapeId(world, shapeB);
    e->step = world->stepCount;
    FillBeginGeometry(world, e, pairIndex, shapeA, shapeB);
}

static void EmitSensorEnd(m2World* world, int32_t shapeA, int32_t shapeB)
{
    if (world->sensorEndCount < world->pairCapacity)
    {
        m2ContactEndEvent* e = &world->sensorEndEvents[world->sensorEndCount++];
        e->shapeIdA = MakeShapeId(world, shapeA);
        e->shapeIdB = MakeShapeId(world, shapeB);
        e->step = world->stepCount;
    }
}

static void EmitSensorBegin(m2World* world, int32_t shapeA, int32_t shapeB, int32_t pairIndex)
{
    if (world->sensorBeginCount < world->pairCapacity)
    {
        m2ContactBeginEvent* e = &world->sensorBeginEvents[world->sensorBeginCount++];
        memset(e, 0, sizeof(*e));
        e->shapeIdA = MakeShapeId(world, shapeA);
        e->shapeIdB = MakeShapeId(world, shapeB);
        e->step = world->stepCount;
        // The overlap manifold carries the hit point and normal (b2 #945).
        FillBeginGeometry(world, e, pairIndex, shapeA, shapeB);
    }
}

// Re-derive pairs touched by the moved set, then batch-merge with the
// untouched remainder (topic-02 §4.2, RT1-PERF-3).
static void UpdatePairs(m2World* world)
{
    if (world->movedCount == 0)
    {
        return;
    }

    qsort(world->moved, (size_t)world->movedCount, sizeof(int32_t), CompareI32);

    int32_t collected = 0;
    int32_t queryResults[256];
    for (int32_t m = 0; m < world->movedCount; ++m)
    {
        int32_t shapeIndex = world->moved[m];
        if (world->shapeAlive[shapeIndex] == 0 || world->proxyIds[shapeIndex] == M2_NULL_NODE)
        {
            continue;
        }
        int32_t treeIndex = ShapeTreeIndex(world, shapeIndex);
        m2AABB fat = world->treeNodes[treeIndex][world->proxyIds[shapeIndex]].aabb;

        bool moverDynamic = ShapeIsDynamic(world, shapeIndex);
        // Kinematic movers sweep every tree: a kinematic character
        // walking through a static trigger zone is the most ordinary
        // sensor story there is. Static movers only exist via
        // teleport and keep the narrow scan.
        bool moverKinematic =
            world->types[world->shapeBody[shapeIndex]] == (uint8_t)m2_kinematicBody;
        int32_t firstTree = moverDynamic || moverKinematic ? 0 : m2_dynamicBody;
        int32_t lastTree = moverDynamic || moverKinematic ? M2_TREE_COUNT - 1 : m2_dynamicBody;
        for (int32_t t = firstTree; t <= lastTree; ++t)
        {
            int32_t hits =
                m2Tree_Query(&world->trees[t], world->treeNodes[t], fat, queryResults, 256);
            M2_ASSERT(hits <= 256);
            hits = hits <= 256 ? hits : 256;
            for (int32_t h = 0; h < hits; ++h)
            {
                int32_t other = queryResults[h];
                if (other == shapeIndex || world->shapeAlive[other] == 0)
                {
                    continue;
                }
                if (world->shapeBody[other] == world->shapeBody[shapeIndex])
                {
                    continue; // same-body shapes never pair
                }
                if (!moverDynamic && !ShapeIsDynamic(world, other) &&
                    world->shapeSensor[shapeIndex] == 0 && world->shapeSensor[other] == 0)
                {
                    continue; // two non-dynamics only pair through a sensor
                }
                if (world->shapeSensor[shapeIndex] != 0 && world->shapeSensor[other] != 0)
                {
                    continue; // two sensors never detect each other
                }
                int32_t groupA = world->shapeGroup[shapeIndex];
                if (groupA != 0 && groupA == world->shapeGroup[other])
                {
                    if (groupA < 0)
                    {
                        continue; // same negative group: never collide
                    }
                }
                else if ((world->shapeCategory[shapeIndex] & world->shapeMask[other]) == 0 ||
                         (world->shapeCategory[other] & world->shapeMask[shapeIndex]) == 0)
                {
                    continue; // filtered out (category/mask, both ways)
                }
                if (world->maxJointIndex > 0 &&
                    JointsForbidPair(world, world->shapeBody[shapeIndex], world->shapeBody[other]))
                {
                    continue; // connected without collideConnected
                }
                if (collected < world->pairCapacity)
                {
                    world->pairScratch[collected] = PairKey(shapeIndex, other);
                }
                collected += 1;
            }
        }
    }
    M2_ASSERT(collected <= world->pairCapacity);
    collected = collected <= world->pairCapacity ? collected : world->pairCapacity;

    qsort(world->pairScratch, (size_t)collected, sizeof(uint64_t), CompareU64);

    // Old set stash for the end-event diff and the touching carry.
    memcpy(world->oldPairScratch, world->pairKeys, (size_t)world->pairCount * sizeof(uint64_t));
    memcpy(world->touchingScratch, world->pairTouching, (size_t)world->pairCount * sizeof(uint8_t));
    int32_t oldCount = world->pairCount;

    int32_t kept = 0;
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        int32_t a = (int32_t)(world->pairKeys[i] >> 32);
        int32_t b = (int32_t)(world->pairKeys[i] & 0xFFFFFFFFu);
        if (world->inMoved[a] == 0 && world->inMoved[b] == 0 && world->shapeAlive[a] != 0 &&
            world->shapeAlive[b] != 0)
        {
            world->pairKeys[kept] = world->pairKeys[i];
            kept += 1;
        }
    }

    int32_t i = 0;
    int32_t j = 0;
    int32_t outCount = 0;
    uint64_t previous = 0;
    bool hasPrevious = false;
    while ((i < kept || j < collected) && outCount < world->pairCapacity)
    {
        uint64_t next;
        if (i < kept && (j >= collected || world->pairKeys[i] <= world->pairScratch[j]))
        {
            next = world->pairKeys[i];
            i += 1;
        }
        else
        {
            next = world->pairScratch[j];
            j += 1;
        }
        if (hasPrevious && next == previous)
        {
            continue;
        }
        world->pairScratch[world->pairCapacity - 1 - outCount] = next;
        previous = next;
        hasPrevious = true;
        outCount += 1;
    }
    // The merge writes the m-th smallest key to scratch[cap-1-m]; the
    // read-back must mirror that, or the whole list comes back
    // reversed and every downstream both-sorted walk (warm-start
    // carry, end-event diff) silently degrades. It did, for twenty
    // slices - deterministically, so no hash gate ever saw it.
    for (int32_t k = 0; k < outCount; ++k)
    {
        world->pairKeys[k] = world->pairScratch[world->pairCapacity - 1 - k];
    }
    world->pairCount = outCount;

#ifndef NDEBUG
    // The invariant that hid a reversed read-back for twenty slices:
    // the pair list must be strictly ascending after every rebuild.
    for (int32_t v = 1; v < world->pairCount; ++v)
    {
        M2_ASSERT(world->pairKeys[v - 1] < world->pairKeys[v]);
    }
#endif

    // Diff old vs new (both sorted): vanished-and-touching pairs emit
    // their end events here (M19: pair loss is a contact-killing path);
    // surviving pairs carry their touching flag to the new slot.
    {
        int32_t oi = 0;
        int32_t ni = 0;
        while (oi < oldCount || ni < world->pairCount)
        {
            uint64_t ok = oi < oldCount ? world->oldPairScratch[oi] : UINT64_MAX;
            uint64_t nk = ni < world->pairCount ? world->pairKeys[ni] : UINT64_MAX;
            if (ok == nk)
            {
                world->pairTouching[ni] = world->touchingScratch[oi];
                oi += 1;
                ni += 1;
            }
            else if (ok < nk)
            {
                if (world->touchingScratch[oi] != 0)
                {
                    int32_t a = (int32_t)(ok >> 32);
                    int32_t b = (int32_t)(ok & 0xFFFFFFFFu);
                    if (world->shapeAlive[a] != 0 && world->shapeAlive[b] != 0)
                    {
                        EmitEnd(world, a, b); // destroy path emits its own
                    }
                }
                oi += 1;
            }
            else
            {
                world->pairTouching[ni] = 0;
                ni += 1;
            }
        }
    }

    for (int32_t m = 0; m < world->movedCount; ++m)
    {
        world->inMoved[world->moved[m]] = 0;
    }
    world->movedCount = 0;
}

static void PrunePairsOfShape(m2World* world, int32_t shapeIndex)
{
    int32_t kept = 0;
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        int32_t a = (int32_t)(world->pairKeys[i] >> 32);
        int32_t b = (int32_t)(world->pairKeys[i] & 0xFFFFFFFFu);
        if (a != shapeIndex && b != shapeIndex)
        {
            world->pairKeys[kept] = world->pairKeys[i];
            world->pairTouching[kept] = world->pairTouching[i];
            world->manifolds[kept] = world->manifolds[i];
            kept += 1;
        }
    }
    world->pairCount = kept;
}

// --- Mass ---------------------------------------------------------------------

// Deterministic: walks the body's shape list in stored (insertion) order,
// which the snapshot preserves. Dynamic bodies get the minimum-mass floor
// (RT1-NUM-2: a zero-density dynamic body must not mint an infinite
// inverse mass).
static void RecomputeMass(m2World* world, int32_t bodyIndex)
{
    if (world->types[bodyIndex] != (uint8_t)m2_dynamicBody)
    {
        world->invMass[bodyIndex] = 0.0f;
        world->invInertia[bodyIndex] = 0.0f;
        world->localCenters[bodyIndex] = (m2Vec2){0.0f, 0.0f};
        return;
    }

    // First pass: total mass and the mass-weighted center. Shape mass now
    // reports inertia about the SHAPE centroid, so the shift to the body
    // COM below is a sum of non-negative parallel-axis terms with no
    // big-minus-big cancellation (reference b2 #955).
    float mass = 0.0f;
    m2Vec2 center = {0.0f, 0.0f};
    for (int32_t s = world->bodyShapeHead[bodyIndex]; s != -1; s = world->shapeNext[s])
    {
        m2MassData data = m2ComputeShapeMass(&world->shapeGeometry[s], world->shapeDensity[s]);
        mass += data.mass;
        center.x += data.mass * data.center.x;
        center.y += data.mass * data.center.y;
    }

    if (!(mass > 0.0f))
    {
        // Floor: shapeless or zero-density dynamic bodies weigh 1 kg and
        // do not rotate from impulses (deterministic fallback, no NaN).
        world->invMass[bodyIndex] = 1.0f;
        world->invInertia[bodyIndex] = 0.0f;
        world->localCenters[bodyIndex] = (m2Vec2){0.0f, 0.0f};
        return;
    }

    float invMass = 1.0f / mass;
    center.x *= invMass;
    center.y *= invMass;

    // Second pass: accumulate inertia about the body COM. Each shape's
    // centroid inertia plus mass times the (small) offset from the COM,
    // in the same canonical shape-list order.
    float inertiaCenter = 0.0f;
    for (int32_t s = world->bodyShapeHead[bodyIndex]; s != -1; s = world->shapeNext[s])
    {
        m2MassData data = m2ComputeShapeMass(&world->shapeGeometry[s], world->shapeDensity[s]);
        float ox = center.x - data.center.x;
        float oy = center.y - data.center.y;
        inertiaCenter += data.rotationalInertia + data.mass * (ox * ox + oy * oy);
    }
    world->invMass[bodyIndex] = invMass;
    world->invInertia[bodyIndex] =
        world->fixedRotations[bodyIndex] == 0 && inertiaCenter > 0.0f ? 1.0f / inertiaCenter : 0.0f;
    // The body now rotates about this point: velocities, solver
    // anchors and integration all live in the COM frame.
    world->localCenters[bodyIndex] = center;
}

// --- Contacts (topic-04) -------------------------------------------------------

// Relative pose of shape B's body in shape A's body frame: the single
// f64 -> f32 crossing for the contact stage (RT1-NUM-3).
static m2RelativePose MakeRelativePose(const m2World* world, int32_t bodyA, int32_t bodyB)
{
    m2Transform xfA = world->transforms[bodyA];
    m2Transform xfB = world->transforms[bodyB];
    float dx = (float)(xfB.p.x - xfA.p.x);
    float dy = (float)(xfB.p.y - xfA.p.y);
    m2RelativePose pose;
    pose.p = (m2Vec2){xfA.q.c * dx + xfA.q.s * dy, -xfA.q.s * dx + xfA.q.c * dy};
    m2Rot rel = {xfA.q.c * xfB.q.c + xfA.q.s * xfB.q.s, xfA.q.c * xfB.q.s - xfA.q.s * xfB.q.c};
    pose.q = m2NormalizeRot(rel); // every composition renormalizes
    return pose;
}

// Convert a manifold computed with swapped shape roles back into the
// canonical frame (A = lower shape index).
static m2Manifold FlipManifold(m2Manifold in, m2RelativePose poseOfBInA)
{
    m2Manifold out = in;
    for (int32_t k = 0; k < in.pointCount; ++k)
    {
        out.points[k].anchorA = in.points[k].anchorB;
        out.points[k].anchorB = in.points[k].anchorA;
    }
    m2Vec2 n = {poseOfBInA.q.c * in.normal.x - poseOfBInA.q.s * in.normal.y,
                poseOfBInA.q.s * in.normal.x + poseOfBInA.q.c * in.normal.y};
    out.normal = (m2Vec2){-n.x, -n.y};
    return out;
}

static m2RelativePose InvertPose(m2RelativePose pose)
{
    m2RelativePose inv;
    inv.q = (m2Rot){pose.q.c, -pose.q.s};
    m2Vec2 r = {inv.q.c * pose.p.x - inv.q.s * pose.p.y, inv.q.s * pose.p.x + inv.q.c * pose.p.y};
    inv.p = (m2Vec2){-r.x, -r.y};
    return inv;
}

// Chain laws, applied in the chain's own frame after the ordinary SAT
// pipeline has spoken. DELIBERATE DEVIATION from the reference (argued
// per CONTRIBUTING): Box2D integrates ghost handling into a GJK-based
// collider; Maul reuses its bit-proven SAT+clip pipeline and enforces
// one-sidedness and ghost Voronoi rejection as an explicit post-pass.
// Buys: no new collider subsystem, laws visible and testable in one
// place. Cost: seam behavior lives or dies by the crossing tests.
static void ApplyChainLaws(m2Manifold* manifold, const m2ChainSegment* chain)
{
    if (manifold->pointCount == 0)
    {
        return;
    }
    m2Vec2 p1 = chain->segment.point1;
    m2Vec2 p2 = chain->segment.point2;
    m2Vec2 e = {p2.x - p1.x, p2.y - p1.y};
    m2Vec2 rightPerp = {e.y, -e.x};

    // One-sided: solid on the right of point1 -> point2 only.
    if (manifold->normal.x * rightPerp.x + manifold->normal.y * rightPerp.y <= 0.0f)
    {
        manifold->pointCount = 0;
        return;
    }

    float ee = e.x * e.x + e.y * e.y;
    int32_t kept = 0;
    for (int32_t k = 0; k < manifold->pointCount; ++k)
    {
        m2Vec2 pos = manifold->points[k].anchorA;
        float v = e.x * (pos.x - p1.x) + e.y * (pos.y - p1.y);
        bool drop = false;
        if (v < 0.0f)
        {
            // Behind point1: the previous edge's Voronoi region owns
            // anything out here; this segment lets go.
            m2Vec2 prevEdge = {p1.x - chain->ghost1.x, p1.y - chain->ghost1.y};
            float uPrev = prevEdge.x * (pos.x - p1.x) + prevEdge.y * (pos.y - p1.y);
            drop = uPrev <= 0.0f;
        }
        else if (v > ee)
        {
            m2Vec2 nextEdge = {chain->ghost2.x - p2.x, chain->ghost2.y - p2.y};
            float vNext = nextEdge.x * (pos.x - p2.x) + nextEdge.y * (pos.y - p2.y);
            drop = vNext > 0.0f;
        }
        if (!drop)
        {
            manifold->points[kept] = manifold->points[k];
            kept += 1;
        }
    }
    manifold->pointCount = kept;
}

static m2Manifold ComputeManifoldRaw(const m2World* world, int32_t shapeA, int32_t shapeB,
                                     m2RelativePose pose);

static m2Manifold ComputeManifold(const m2World* world, int32_t shapeA, int32_t shapeB,
                                  m2RelativePose pose)
{
    const m2ShapeGeometry* ga = &world->shapeGeometry[shapeA];
    const m2ShapeGeometry* gb = &world->shapeGeometry[shapeB];
    if (gb->type == m2_chainSegmentShape && ga->type != m2_chainSegmentShape)
    {
        // Canonical: the chain plays shape A so its laws apply in its
        // own frame; the manifold flips back on the way out.
        m2Manifold m = ComputeManifold(world, shapeB, shapeA, InvertPose(pose));
        return FlipManifold(m, pose);
    }
    m2Manifold m = ComputeManifoldRaw(world, shapeA, shapeB, pose);
    if (ga->type == m2_chainSegmentShape)
    {
        ApplyChainLaws(&m, &ga->chainSegment);
    }
    return m;
}

static m2Manifold ComputeManifoldRaw(const m2World* world, int32_t shapeA, int32_t shapeB,
                                     m2RelativePose pose)
{
    const m2ShapeGeometry* ga = &world->shapeGeometry[shapeA];
    const m2ShapeGeometry* gb = &world->shapeGeometry[shapeB];

    if (ga->type == m2_circleShape && gb->type == m2_circleShape)
    {
        return m2CollideCircles(&ga->circle, &gb->circle, pose);
    }
    if (ga->type == m2_polygonShape && gb->type == m2_circleShape)
    {
        return m2CollidePolygonAndCircle(&ga->polygon, &gb->circle, pose);
    }
    if (ga->type == m2_circleShape && gb->type == m2_polygonShape)
    {
        m2Manifold m = m2CollidePolygonAndCircle(&gb->polygon, &ga->circle, InvertPose(pose));
        return FlipManifold(m, pose);
    }

    // Capsules and segments become 2-vertex rounded polygons; every
    // remaining combination goes through the SAT+clip kernel.
    m2Polygon proxyA;
    m2Polygon proxyB;
    const m2Polygon* pa = NULL;
    const m2Polygon* pb = NULL;
    if (ga->type == m2_polygonShape)
    {
        pa = &ga->polygon;
    }
    else if (ga->type == m2_capsuleShape)
    {
        proxyA = m2MakeSegmentProxy(ga->capsule.point1, ga->capsule.point2, ga->capsule.radius);
        pa = &proxyA;
    }
    else if (ga->type == m2_segmentShape)
    {
        proxyA = m2MakeSegmentProxy(ga->segment.point1, ga->segment.point2, 0.0f);
        pa = &proxyA;
    }
    else if (ga->type == m2_chainSegmentShape)
    {
        proxyA = m2MakeSegmentProxy(ga->chainSegment.segment.point1,
                                    ga->chainSegment.segment.point2, 0.0f);
        pa = &proxyA;
    }
    if (gb->type == m2_polygonShape)
    {
        pb = &gb->polygon;
    }
    else if (gb->type == m2_capsuleShape)
    {
        proxyB = m2MakeSegmentProxy(gb->capsule.point1, gb->capsule.point2, gb->capsule.radius);
        pb = &proxyB;
    }
    else if (gb->type == m2_segmentShape)
    {
        proxyB = m2MakeSegmentProxy(gb->segment.point1, gb->segment.point2, 0.0f);
        pb = &proxyB;
    }
    else if (gb->type == m2_chainSegmentShape)
    {
        proxyB = m2MakeSegmentProxy(gb->chainSegment.segment.point1,
                                    gb->chainSegment.segment.point2, 0.0f);
        pb = &proxyB;
    }

    if (pa != NULL && gb->type == m2_circleShape)
    {
        return m2CollidePolygonAndCircle(pa, &gb->circle, pose);
    }
    if (ga->type == m2_circleShape && pb != NULL)
    {
        m2Manifold m = m2CollidePolygonAndCircle(pb, &ga->circle, InvertPose(pose));
        return FlipManifold(m, pose);
    }
    M2_ASSERT(pa != NULL && pb != NULL);
    return m2CollidePolygons(pa, pb, pose);
}

// Stash old (key, manifold) rows, then rebuild aligned to the new pair
// array, carrying warm-start impulses across by pair key and point id.
static void StashContacts(m2World* world)
{
    memcpy(world->oldPairScratch, world->pairKeys, (size_t)world->pairCount * sizeof(uint64_t));
    memcpy(world->manifoldScratch, world->manifolds, (size_t)world->pairCount * sizeof(m2Manifold));
}

typedef struct m2UpdateContactsCtx
{
    m2World* world;
} m2UpdateContactsCtx;

// Each pair writes only its own manifold slot, so the range splits
// freely across workers without touching the arithmetic.
static void UpdateContactsRange(int32_t begin, int32_t end, void* userCtx)
{
    m2World* world = ((m2UpdateContactsCtx*)userCtx)->world;
    for (int32_t i = begin; i < end; ++i)
    {
        int32_t shapeA = (int32_t)(world->pairKeys[i] >> 32);
        int32_t shapeB = (int32_t)(world->pairKeys[i] & 0xFFFFFFFFu);

        // Frozen pair: both ends static or sleeping, so transforms and
        // geometry are untouched and the stored manifold is exactly
        // what ComputeManifold would return - skip the arithmetic,
        // keep the bits. A kinematic end never freezes (its velocity
        // can change without stepping).
        int32_t frozenBodyA = world->shapeBody[shapeA];
        int32_t frozenBodyB = world->shapeBody[shapeB];
        bool frozenA = world->types[frozenBodyA] == (uint8_t)m2_staticBody ||
                       (world->types[frozenBodyA] == (uint8_t)m2_dynamicBody &&
                        world->asleep[frozenBodyA] != 0 && world->sleepStreak[frozenBodyA] >= 2);
        bool frozenB = world->types[frozenBodyB] == (uint8_t)m2_staticBody ||
                       (world->types[frozenBodyB] == (uint8_t)m2_dynamicBody &&
                        world->asleep[frozenBodyB] != 0 && world->sleepStreak[frozenBodyB] >= 2);
        if (frozenA && frozenB)
        {
            int32_t flo = 0;
            int32_t fhi = world->oldPairCount - 1;
            while (flo <= fhi)
            {
                int32_t mid = (flo + fhi) / 2;
                if (world->oldPairScratch[mid] == world->pairKeys[i])
                {
                    world->manifolds[i] = world->manifoldScratch[mid];
                    // Recompute would match every id against itself and
                    // set the persisted bit; the copy owes the same.
                    for (int32_t k = 0; k < world->manifolds[i].pointCount; ++k)
                    {
                        world->manifolds[i].points[k].flags |= 1;
                    }
                    break;
                }
                if (world->oldPairScratch[mid] < world->pairKeys[i])
                {
                    flo = mid + 1;
                }
                else
                {
                    fhi = mid - 1;
                }
            }
            if (flo <= fhi)
            {
                continue; // found and copied
            }
            // Brand-new pair between frozen bodies (restore edges):
            // fall through and compute once.
        }

        m2RelativePose pose =
            MakeRelativePose(world, world->shapeBody[shapeA], world->shapeBody[shapeB]);
        m2Manifold fresh = ComputeManifold(world, shapeA, shapeB, pose);

        // Locate the previous manifold for this pair (both lists sorted;
        // binary search keeps this O(P log P) worst case).
        const m2Manifold* previous = NULL;
        int32_t lo = 0;
        int32_t hi = world->oldPairCount - 1;
        while (lo <= hi)
        {
            int32_t mid = (lo + hi) / 2;
            if (world->oldPairScratch[mid] == world->pairKeys[i])
            {
                previous = &world->manifoldScratch[mid];
                break;
            }
            if (world->oldPairScratch[mid] < world->pairKeys[i])
            {
                lo = mid + 1;
            }
            else
            {
                hi = mid - 1;
            }
        }

        if (previous != NULL)
        {
            for (int32_t k = 0; k < fresh.pointCount; ++k)
            {
                for (int32_t o = 0; o < previous->pointCount; ++o)
                {
                    if (previous->points[o].id == fresh.points[k].id)
                    {
                        fresh.points[k].normalImpulse = previous->points[o].normalImpulse;
                        fresh.points[k].tangentImpulse = previous->points[o].tangentImpulse;
                        fresh.points[k].flags |= 1; // persisted
                        break;
                    }
                }
            }
        }
        world->manifolds[i] = fresh;
    }
}

static void UpdateContacts(m2World* world)
{
    m2UpdateContactsCtx ctx = {world};
    m2ThreadPoolRun(world->pool, UpdateContactsRange, &ctx, world->pairCount);
}

// --- Defs & world lifecycle ----------------------------------------------------

m2WorldDef m2DefaultWorldDef(void)
{
    m2WorldDef def;
    memset(&def, 0, sizeof(def));
    def.gravity = (m2Vec2){0.0f, -10.0f};
    def.bodyCapacity = 1024;
    def.shapeCapacity = 2048;
    def.jointCapacity = 256;
    def.particleCapacity = 0;    // fluids are opt-in
    def.fluidVolumeCapacity = 0; // buoyancy volumes are opt-in
    def.particleRadius = 0.05f;
    def.particleDensity = 1.0f;
    def.particleGravityScale = 1.0f;
    def.particlePressureStrength = 0.05f;
    def.particleDampingStrength = 1.0f;
    def.particleViscousStrength = 0.25f; // used by viscous-flagged particles only
    def.particlePowderStrength = 0.5f;
    def.particleSpringStrength = 0.25f;  // overlapping nets sum; reference value
    def.particleElasticStrength = 0.25f; // stiff blobs combine spring|elastic flags
    def.particleTensilePressureStrength = 0.2f;
    def.particleTensileNormalStrength = 0.2f;
    def.internalValue = M2_WORLD_COOKIE;
    return def;
}

m2WorldId m2CreateWorld(const m2WorldDef* def)
{
    // Before any solver kernel runs, make sure this CPU can execute the
    // backend the binary was built for: a clear abort beats a bare
    // illegal-instruction trap on pre-Haswell hardware.
    m2VerifyCpuBackend();
    if (def == NULL || def->internalValue != M2_WORLD_COOKIE || def->bodyCapacity < 1 ||
        def->shapeCapacity < 1 || def->jointCapacity < 1)
    {
        M2_ASSERT(false);
        return m2_nullWorldId;
    }
    if (def->fluidVolumeCapacity < 0)
    {
        M2_ASSERT(false);
        return m2_nullWorldId;
    }
    if (def->particleCapacity < 0 ||
        (def->particleCapacity > 0 &&
         (!(def->particleRadius >= 0.02f) || !(def->particleDensity > 0.0f) ||
          !(def->particleGravityScale == def->particleGravityScale) ||
          !(def->particlePressureStrength >= 0.0f) || !(def->particleDampingStrength >= 0.0f) ||
          !(def->particleViscousStrength >= 0.0f) ||
          !(def->particleTensilePressureStrength >= 0.0f) ||
          !(def->particleTensileNormalStrength >= 0.0f) || !(def->particlePowderStrength >= 0.0f) ||
          !(def->particleSpringStrength >= 0.0f) || !(def->particleElasticStrength >= 0.0f))))
    {
        // Fluids config is validated loudly: the radius floor is 4x
        // linear slop so the skin laws keep meaning.
        M2_ASSERT(false);
        return m2_nullWorldId;
    }

    int32_t slot = -1;
    for (int32_t i = 0; i < M2_MAX_WORLDS; ++i)
    {
        if (s_worlds[i] == NULL)
        {
            slot = i;
            break;
        }
    }
    if (slot < 0)
    {
        return m2_nullWorldId;
    }

    m2World* world = m2AllocZeroed(sizeof(m2World));
    if (world == NULL)
    {
        return m2_nullWorldId;
    }

    int32_t cap = def->bodyCapacity;
    int32_t shapeCap = def->shapeCapacity;
    int32_t jointCap = def->jointCapacity;
    world->gravity = def->gravity;
    world->windVelocity = (m2Vec2){0.0f, 0.0f};
    world->windLinearDrag = 0.0f; // wind is opt-in via m2World_SetWind
    world->bodyCapacity = cap;
    world->shapeCapacity = shapeCap;
    world->jointCapacity = jointCap;
    world->treeNodeCapacity = 2 * shapeCap;
    world->pairCapacity = 8 * shapeCap;
    world->particleCapacity = def->particleCapacity;
    world->fvCapacity = def->fluidVolumeCapacity;
    world->particleRadius = def->particleRadius;
    world->particleDensity = def->particleDensity;
    world->particleGravityScale = def->particleGravityScale;
    world->particlePressureStrength = def->particlePressureStrength;
    world->particleDampingStrength = def->particleDampingStrength;
    world->particleViscousStrength = def->particleViscousStrength;
    world->particleTensilePressure = def->particleTensilePressureStrength;
    world->particlePowderStrength = def->particlePowderStrength;
    world->particleSpringStrength = def->particleSpringStrength;
    world->particleElasticStrength = def->particleElasticStrength;
    world->particleTensileNormal = def->particleTensileNormalStrength;

    bool ok = true;
#define M2_ALLOC(field, count, type)                                                               \
    do                                                                                             \
    {                                                                                              \
        world->field = m2AllocZeroed((size_t)(count) * sizeof(type));                              \
        ok = ok && world->field != NULL;                                                           \
    } while (0)
    M2_ALLOC(transforms, cap, m2Transform);
    M2_ALLOC(linearVelocities, cap + 1, m2Vec2); // +1: wide-lane dummy slot
    M2_ALLOC(angularVelocities, cap + 1, float); // +1: wide-lane dummy slot
    M2_ALLOC(gravityScales, cap, float);
    M2_ALLOC(userData, cap, uint64_t);
    M2_ALLOC(types, cap + 1, uint8_t); // +1: wide-lane dummy slot
    M2_ALLOC(alive, cap, uint8_t);
    M2_ALLOC(bodyShapeHead, cap, int32_t);
    M2_ALLOC(invMass, cap, float);
    M2_ALLOC(invInertia, cap, float);
    M2_ALLOC(localCenters, cap, m2Vec2);
    M2_ALLOC(asleep, cap, uint8_t);
    M2_ALLOC(sleepTimes, cap, float);
    M2_ALLOC(sleepStreak, cap, uint8_t);
    M2_ALLOC(bullets, cap, uint8_t);
    M2_ALLOC(ccdPrevPositions, cap, m2Pos2);
    M2_ALLOC(islandParent, cap, int32_t);
    M2_ALLOC(islandDisturbed, cap, uint8_t);
    M2_ALLOC(generations, cap, uint16_t);
    M2_ALLOC(freeQueue, cap, int32_t);
    M2_ALLOC(shapeGeometry, shapeCap, m2ShapeGeometry);
    M2_ALLOC(shapeDensity, shapeCap, float);
    M2_ALLOC(shapeFriction, shapeCap, float);
    M2_ALLOC(shapeRestitution, shapeCap, float);
    M2_ALLOC(shapeTangentSpeed, shapeCap, float);
    M2_ALLOC(shapeUserData, shapeCap, uint64_t);
    M2_ALLOC(shapeBody, shapeCap, int32_t);
    M2_ALLOC(shapeNext, shapeCap, int32_t);
    M2_ALLOC(shapeAlive, shapeCap, uint8_t);
    M2_ALLOC(shapeGenerations, shapeCap, uint16_t);
    M2_ALLOC(shapeCategory, shapeCap, uint32_t);
    M2_ALLOC(shapeMask, shapeCap, uint32_t);
    M2_ALLOC(shapeGroup, shapeCap, int32_t);
    M2_ALLOC(shapeSensor, shapeCap, uint8_t);
    M2_ALLOC(shapeFreeQueue, shapeCap, int32_t);
    M2_ALLOC(proxyIds, shapeCap, int32_t);
    M2_ALLOC(inMoved, shapeCap, uint8_t);
    M2_ALLOC(moved, shapeCap, int32_t);
    M2_ALLOC(jointType, jointCap, uint8_t);
    M2_ALLOC(jointAlive, jointCap, uint8_t);
    M2_ALLOC(jointBodyA, jointCap, int32_t);
    M2_ALLOC(jointBodyB, jointCap, int32_t);
    M2_ALLOC(jointLocalAnchorA, jointCap, m2Vec2);
    M2_ALLOC(jointLocalAnchorB, jointCap, m2Vec2);
    M2_ALLOC(jointLength, jointCap, float);
    M2_ALLOC(jointHertz, jointCap, float);
    M2_ALLOC(jointDamping, jointCap, float);
    M2_ALLOC(jointHertz2, jointCap, float);
    M2_ALLOC(jointDamping2, jointCap, float);
    M2_ALLOC(jointImpulse, jointCap, m2Vec2);
    M2_ALLOC(jointFlags, jointCap, uint8_t);
    M2_ALLOC(jointMotorSpeed, jointCap, float);
    M2_ALLOC(jointMaxMotor, jointCap, float);
    M2_ALLOC(jointLower, jointCap, float);
    M2_ALLOC(jointUpper, jointCap, float);
    M2_ALLOC(jointLocalAxisA, jointCap, m2Vec2);
    M2_ALLOC(jointRefAngle, jointCap, float);
    M2_ALLOC(jointMotorImpulse, jointCap, float);
    M2_ALLOC(jointLowerImpulse, jointCap, float);
    M2_ALLOC(jointUpperImpulse, jointCap, float);
    M2_ALLOC(jointSpringImpulse, jointCap, float);
    M2_ALLOC(jointBreakForce, jointCap, float);
    M2_ALLOC(jointCollide, jointCap, uint8_t);
    M2_ALLOC(jointTargets, jointCap, m2Pos2);
    M2_ALLOC(jointTargetsB, jointCap, m2Pos2);
    if (def->particleCapacity > 0)
    {
        int32_t particleCap = def->particleCapacity;
        M2_ALLOC(particlePositions, particleCap, m2Pos2);
        M2_ALLOC(particleVelocities, particleCap, m2Vec2);
        M2_ALLOC(particleAlive, particleCap, uint8_t);
        M2_ALLOC(particleGenerations, particleCap, uint16_t);
        M2_ALLOC(particleFlags, particleCap, uint32_t);
        M2_ALLOC(particleLifetime, particleCap, float);
        M2_ALLOC(particleUserData, particleCap, uint64_t);
        M2_ALLOC(particleFreeQueue, particleCap, int32_t);
        world->particlePairCapacity = 12 * particleCap;
        world->particleProxies = m2AllocZeroed((size_t)particleCap * 16);
        ok = ok && world->particleProxies != NULL;
        world->particleProxiesTmp = m2AllocZeroed((size_t)particleCap * 16);
        ok = ok && world->particleProxiesTmp != NULL;
        M2_ALLOC(particlePairA, world->particlePairCapacity, int32_t);
        M2_ALLOC(particlePairB, world->particlePairCapacity, int32_t);
        M2_ALLOC(particlePairWeight, world->particlePairCapacity, float);
        M2_ALLOC(particlePairFlags, world->particlePairCapacity, uint32_t);
        M2_ALLOC(particlePairNormal, world->particlePairCapacity, m2Vec2);
        M2_ALLOC(particleWeights, particleCap, float);
        M2_ALLOC(particleAccumulation, particleCap, float);
        M2_ALLOC(particleAccumulation2, particleCap, m2Vec2);
        world->particleSpringCapacity = 4 * particleCap;
        M2_ALLOC(particleSpringA, world->particleSpringCapacity, int32_t);
        M2_ALLOC(particleSpringB, world->particleSpringCapacity, int32_t);
        M2_ALLOC(particleSpringRest, world->particleSpringCapacity, float);
        world->particleTriadCapacity = 2 * particleCap;
        M2_ALLOC(particleTriadA, world->particleTriadCapacity, int32_t);
        M2_ALLOC(particleTriadB, world->particleTriadCapacity, int32_t);
        M2_ALLOC(particleTriadC, world->particleTriadCapacity, int32_t);
        M2_ALLOC(particleTriadPA, world->particleTriadCapacity, m2Vec2);
        M2_ALLOC(particleTriadPB, world->particleTriadCapacity, m2Vec2);
        M2_ALLOC(particleTriadPC, world->particleTriadCapacity, m2Vec2);
        world->particleBodyCapacity = 4 * particleCap;
        M2_ALLOC(particleBodyParticle, world->particleBodyCapacity, int32_t);
        M2_ALLOC(particleBodyBody, world->particleBodyCapacity, int32_t);
        M2_ALLOC(particleBodyWeight, world->particleBodyCapacity, float);
        M2_ALLOC(particleBodyNormal, world->particleBodyCapacity, m2Vec2);
        M2_ALLOC(particleBodyMass, world->particleBodyCapacity, float);
        M2_ALLOC(particleBodyStageBody, 4 * particleCap, int32_t);
        M2_ALLOC(particleBodyStageWeight, 4 * particleCap, float);
        M2_ALLOC(particleBodyStageNormal, 4 * particleCap, m2Vec2);
        M2_ALLOC(particleBodyStageMass, 4 * particleCap, float);
        M2_ALLOC(particlePairWorkCount, particleCap + 1, int32_t);
        M2_ALLOC(particleBodyStageDrops, particleCap, int32_t);
    }
    M2_ALLOC(jointUserData, jointCap, uint64_t);
    M2_ALLOC(jointBreakTorque, jointCap, float);
    M2_ALLOC(jointGenerations, jointCap, uint16_t);
    M2_ALLOC(jointFreeQueue, jointCap, int32_t);
    M2_ALLOC(linearDampings, cap, float);
    M2_ALLOC(angularDampings, cap, float);
    M2_ALLOC(fixedRotations, cap, uint8_t);
    M2_ALLOC(motionLocks, cap, uint8_t);
    M2_ALLOC(sleepEnables, cap, uint8_t);
    M2_ALLOC(forces, cap, m2Vec2);
    M2_ALLOC(torques, cap, float);
    M2_ALLOC(disabled, cap, uint8_t);
    M2_ALLOC(dominances, cap, int8_t);
    if (def->fluidVolumeCapacity > 0)
    {
        int32_t fvCap = def->fluidVolumeCapacity;
        M2_ALLOC(fvLower, fvCap, m2Pos2);
        M2_ALLOC(fvUpper, fvCap, m2Pos2);
        M2_ALLOC(fvSurface, fvCap, double);
        M2_ALLOC(fvDensity, fvCap, float);
        M2_ALLOC(fvLinearDrag, fvCap, float);
        M2_ALLOC(fvAngularDrag, fvCap, float);
        M2_ALLOC(fvFlow, fvCap, m2Vec2);
        M2_ALLOC(fvUserData, fvCap, uint64_t);
        M2_ALLOC(fvAlive, fvCap, uint8_t);
        M2_ALLOC(fvGenerations, fvCap, uint16_t);
        M2_ALLOC(fvFreeQueue, fvCap, int32_t);
    }
    M2_ALLOC(shapeChain, shapeCap, int32_t);
    M2_ALLOC(chainAlive, shapeCap, uint8_t);
    M2_ALLOC(chainBody, shapeCap, int32_t);
    M2_ALLOC(chainGenerations, shapeCap, uint16_t);
    M2_ALLOC(chainFreeQueue, shapeCap, int32_t);
    M2_ALLOC(pairKeys, world->pairCapacity, uint64_t);
    M2_ALLOC(pairTouching, world->pairCapacity, uint8_t);
    M2_ALLOC(touchingScratch, world->pairCapacity, uint8_t);
    M2_ALLOC(queryScratch, cap, int32_t);
    M2_ALLOC(colorMasks, cap, uint32_t);
    // Indexed by CONSTRAINT, not body: one slot per potential pair.
    // (Sized by body capacity until the pyramid30 perf scene found the
    // overflow - constraints outnumber bodies in dense stacks.)
    M2_ALLOC(constraintColors, world->pairCapacity, uint8_t);
    M2_ALLOC(colorOrder, world->pairCapacity, int32_t);

    world->pool = m2ThreadPoolCreate(def->workerCount);
    M2_ALLOC(beginEvents, world->pairCapacity, m2ContactBeginEvent);
    M2_ALLOC(endEvents, world->pairCapacity, m2ContactEndEvent);
    M2_ALLOC(pendingEndEvents, world->pairCapacity, m2ContactEndEvent);
    M2_ALLOC(sensorBeginEvents, world->pairCapacity, m2ContactBeginEvent);
    M2_ALLOC(sensorEndEvents, world->pairCapacity, m2ContactEndEvent);
    M2_ALLOC(pendingSensorEnd, world->pairCapacity, m2ContactEndEvent);
    M2_ALLOC(jointBreakEvents, world->jointCapacity, m2JointBreakEvent);
    M2_ALLOC(pairScratch, world->pairCapacity, uint64_t);
    M2_ALLOC(manifolds, world->pairCapacity, m2Manifold);
    M2_ALLOC(oldPairScratch, world->pairCapacity, uint64_t);
    M2_ALLOC(manifoldScratch, world->pairCapacity, m2Manifold);
    M2_ALLOC(deltaPositions, cap + 1, m2Vec2); // +1: wide-lane dummy slot
    M2_ALLOC(deltaRotations, cap + 1, m2Rot);  // +1: wide-lane dummy slot
    world->constraintScratch =
        m2AllocZeroed((size_t)world->pairCapacity * (size_t)m2ContactConstraintSize());
    world->contactBlocks = m2AllocZeroed((size_t)m2ContactBlockScratchBytes(world->pairCapacity));
    ok = ok && world->contactBlocks != NULL;
    ok = ok && world->constraintScratch != NULL;
#undef M2_ALLOC
    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        world->treeNodes[t] = m2AllocZeroed((size_t)world->treeNodeCapacity * sizeof(m2TreeNode));
        ok = ok && world->treeNodes[t] != NULL;
    }
    if (!ok)
    {
        m2WorldId failed = {(uint16_t)(slot + 1), s_worldGenerations[slot]};
        s_worlds[slot] = world;
        m2DestroyWorld(failed);
        return m2_nullWorldId;
    }

    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        m2Tree_Init(&world->trees[t], world->treeNodes[t], world->treeNodeCapacity);
    }
    for (int32_t i = 0; i < cap; ++i)
    {
        world->freeQueue[i] = i;
        world->bodyShapeHead[i] = -1;
    }
    for (int32_t i = 0; i < shapeCap; ++i)
    {
        world->shapeFreeQueue[i] = i;
        world->shapeNext[i] = -1;
        world->proxyIds[i] = M2_NULL_NODE;
        world->shapeChain[i] = -1;
        world->chainFreeQueue[i] = i;
    }
    for (int32_t i = 0; i < jointCap; ++i)
    {
        world->jointFreeQueue[i] = i;
    }
    world->jointFreeCount = jointCap;
    for (int32_t i = 0; i < world->particleCapacity; ++i)
    {
        world->particleFreeQueue[i] = i;
    }
    world->particleFreeCount = world->particleCapacity;
    for (int32_t i = 0; i < world->fvCapacity; ++i)
    {
        world->fvFreeQueue[i] = i;
    }
    world->fvFreeCount = world->fvCapacity;
    world->freeHead = 0;
    world->freeTail = 0;
    world->freeCount = cap;
    world->shapeFreeHead = 0;
    world->shapeFreeTail = 0;
    world->shapeFreeCount = shapeCap;
    world->chainFreeCount = shapeCap;

    s_worldGenerations[slot] += 1;
    world->worldGeneration = s_worldGenerations[slot];
    world->worldIndex0 = (uint16_t)(slot + 1);
    world->sleepEnabled = 1;
    s_worlds[slot] = world;

    m2WorldId id = {(uint16_t)(slot + 1), world->worldGeneration};
    return id;
}

void m2DestroyWorld(m2WorldId worldId)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL && worldId.index1 >= 1 && worldId.index1 <= M2_MAX_WORLDS)
    {
        world = s_worlds[worldId.index1 - 1]; // failed-allocation path
    }
    if (world == NULL)
    {
        return;
    }
    m2Free(world->transforms);
    m2Free(world->linearVelocities);
    m2Free(world->angularVelocities);
    m2Free(world->gravityScales);
    m2Free(world->userData);
    m2Free(world->types);
    m2Free(world->alive);
    m2Free(world->bodyShapeHead);
    m2Free(world->invMass);
    m2Free(world->invInertia);
    m2Free(world->localCenters);
    m2Free(world->asleep);
    m2Free(world->sleepTimes);
    m2Free(world->sleepStreak);
    m2Free(world->bullets);
    m2Free(world->ccdPrevPositions);
    m2Free(world->islandParent);
    m2Free(world->islandDisturbed);
    m2Free(world->generations);
    m2Free(world->freeQueue);
    m2Free(world->shapeGeometry);
    m2Free(world->shapeDensity);
    m2Free(world->shapeFriction);
    m2Free(world->shapeRestitution);
    m2Free(world->shapeTangentSpeed);
    m2Free(world->shapeUserData);
    m2Free(world->shapeBody);
    m2Free(world->shapeNext);
    m2Free(world->shapeAlive);
    m2Free(world->shapeGenerations);
    m2Free(world->shapeCategory);
    m2Free(world->shapeMask);
    m2Free(world->shapeGroup);
    m2Free(world->shapeSensor);
    m2Free(world->shapeFreeQueue);
    m2Free(world->proxyIds);
    m2Free(world->inMoved);
    m2Free(world->moved);
    m2Free(world->jointType);
    m2Free(world->jointAlive);
    m2Free(world->jointBodyA);
    m2Free(world->jointBodyB);
    m2Free(world->jointLocalAnchorA);
    m2Free(world->jointLocalAnchorB);
    m2Free(world->jointLength);
    m2Free(world->jointHertz);
    m2Free(world->jointDamping);
    m2Free(world->jointHertz2);
    m2Free(world->jointDamping2);
    m2Free(world->jointImpulse);
    m2Free(world->jointFlags);
    m2Free(world->jointMotorSpeed);
    m2Free(world->jointMaxMotor);
    m2Free(world->jointLower);
    m2Free(world->jointUpper);
    m2Free(world->jointLocalAxisA);
    m2Free(world->jointRefAngle);
    m2Free(world->jointMotorImpulse);
    m2Free(world->jointLowerImpulse);
    m2Free(world->jointUpperImpulse);
    m2Free(world->jointSpringImpulse);
    m2Free(world->jointBreakForce);
    m2Free(world->jointCollide);
    m2Free(world->jointTargets);
    m2Free(world->jointTargetsB);
    m2Free(world->particlePositions);
    m2Free(world->particleVelocities);
    m2Free(world->particleAlive);
    m2Free(world->particleGenerations);
    m2Free(world->particleFlags);
    m2Free(world->particleLifetime);
    m2Free(world->particleUserData);
    m2Free(world->particleFreeQueue);
    m2Free(world->fvLower);
    m2Free(world->fvUpper);
    m2Free(world->fvSurface);
    m2Free(world->fvDensity);
    m2Free(world->fvLinearDrag);
    m2Free(world->fvAngularDrag);
    m2Free(world->fvFlow);
    m2Free(world->fvUserData);
    m2Free(world->fvAlive);
    m2Free(world->fvGenerations);
    m2Free(world->fvFreeQueue);
    m2Free(world->particleProxies);
    m2Free(world->particleProxiesTmp);
    m2Free(world->particlePairA);
    m2Free(world->particlePairB);
    m2Free(world->particlePairWeight);
    m2Free(world->particlePairFlags);
    m2Free(world->particlePairNormal);
    m2Free(world->particleWeights);
    m2Free(world->particleAccumulation);
    m2Free(world->particleAccumulation2);
    m2Free(world->particleSpringA);
    m2Free(world->particleSpringB);
    m2Free(world->particleSpringRest);
    m2Free(world->particleTriadA);
    m2Free(world->particleTriadB);
    m2Free(world->particleTriadC);
    m2Free(world->particleTriadPA);
    m2Free(world->particleTriadPB);
    m2Free(world->particleTriadPC);
    m2Free(world->particleBodyParticle);
    m2Free(world->particleBodyBody);
    m2Free(world->particleBodyWeight);
    m2Free(world->particleBodyNormal);
    m2Free(world->particleBodyMass);
    m2Free(world->particleBodyStageBody);
    m2Free(world->particleBodyStageWeight);
    m2Free(world->particleBodyStageNormal);
    m2Free(world->particleBodyStageMass);
    m2Free(world->particlePairWorkCount);
    m2Free(world->particleBodyStageDrops);
    m2Free(world->jointUserData);
    m2Free(world->jointBreakTorque);
    m2Free(world->jointGenerations);
    m2Free(world->jointFreeQueue);
    m2Free(world->linearDampings);
    m2Free(world->angularDampings);
    m2Free(world->fixedRotations);
    m2Free(world->motionLocks);
    m2Free(world->sleepEnables);
    m2Free(world->forces);
    m2Free(world->torques);
    m2Free(world->disabled);
    m2Free(world->dominances);
    m2Free(world->shapeChain);
    m2Free(world->chainAlive);
    m2Free(world->chainBody);
    m2Free(world->chainGenerations);
    m2Free(world->chainFreeQueue);
    m2Free(world->pairKeys);
    m2Free(world->pairTouching);
    m2Free(world->touchingScratch);
    m2Free(world->queryScratch);
    m2Free(world->colorMasks);
    m2Free(world->constraintColors);
    m2Free(world->colorOrder);
    m2ThreadPoolDestroy(world->pool);
    m2Free(world->beginEvents);
    m2Free(world->endEvents);
    m2Free(world->pendingEndEvents);
    m2Free(world->sensorBeginEvents);
    m2Free(world->sensorEndEvents);
    m2Free(world->pendingSensorEnd);
    m2Free(world->jointBreakEvents);
    m2Free(world->pairScratch);
    m2Free(world->manifolds);
    m2Free(world->oldPairScratch);
    m2Free(world->manifoldScratch);
    m2Free(world->deltaPositions);
    m2Free(world->deltaRotations);
    m2Free(world->constraintScratch);
    m2Free(world->contactBlocks);
    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        m2Free(world->treeNodes[t]);
    }
    m2Free(world);
    s_worlds[worldId.index1 - 1] = NULL;
}

bool m2World_IsValid(m2WorldId worldId)
{
    return GetWorld(worldId) != NULL;
}

void m2World_Step(m2WorldId worldId, float dt, int32_t substepCount)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL || !(dt > 0.0f) || substepCount < 1)
    {
        M2_ASSERT(world != NULL && dt > 0.0f && substepCount >= 1);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            float dt;
            int32_t substepCount;
        } marker;
        memset(&marker, 0, sizeof(marker));
        marker.dt = dt;
        marker.substepCount = substepCount;
        m2JournalRecord(world, m2_opStep, &marker, (int32_t)sizeof(marker));
    }

    // Fresh event window: clear the public buffers, then flush ends
    // queued by between-step destroys (they belong to this window).
    world->beginEventCount = 0;
    world->endEventCount = 0;
    for (int32_t i = 0; i < world->pendingEndCount && i < world->pairCapacity; ++i)
    {
        world->endEvents[world->endEventCount++] = world->pendingEndEvents[i];
    }
    world->pendingEndCount = 0;
    world->sensorBeginCount = 0;
    world->sensorEndCount = 0;
    for (int32_t i = 0; i < world->pendingSensorEndCount && i < world->pairCapacity; ++i)
    {
        world->sensorEndEvents[world->sensorEndCount++] = world->pendingSensorEnd[i];
    }
    world->pendingSensorEndCount = 0;
    world->jointBreakEventCount = 0;

    // Wall-clock diagnostics only; never fed back into simulation.
    uint64_t tStart = m2TimeNowNs();

    // Hibernation: when every dynamic body sleeps, no kinematic is
    // moving and nothing was teleported, the full pipeline provably
    // changes no state at all (frozen pairs copy themselves, islands
    // rebuild to the same roots, the solver has no constraints). Skip
    // it wholesale - bit-identical by construction, and a sleeping
    // city costs what a sleeping city should.
    if (world->movedCount == 0 && world->particleCount == 0)
    {
        bool anyoneStirring = false;
        for (int32_t i = 0; i < world->maxBodyIndex && !anyoneStirring; ++i)
        {
            if (world->alive[i] == 0)
            {
                continue;
            }
            if (world->types[i] == (uint8_t)m2_dynamicBody)
            {
                // A body that JUST fell asleep still owes one manifold
                // refresh (its stash can be one solve stale - the same
                // freshness rule the frozen-pair skip lives by).
                anyoneStirring =
                    world->disabled[i] == 0 && (world->asleep[i] == 0 || world->sleepStreak[i] < 2);
            }
            else if (world->types[i] == (uint8_t)m2_kinematicBody)
            {
                anyoneStirring = world->linearVelocities[i].x != 0.0f ||
                                 world->linearVelocities[i].y != 0.0f ||
                                 world->angularVelocities[i] != 0.0f;
            }
        }
        if (!anyoneStirring)
        {
            world->profile.stepMs = (float)((double)(m2TimeNowNs() - tStart) * 1.0e-6);
            world->profile.pairsMs = 0.0f;
            world->profile.contactsMs = 0.0f;
            world->profile.solveMs = 0.0f;
            world->profile.sleepMs = 0.0f;
            world->stepCount += 1;
            return;
        }
    }

    // Collide first (reference order): broadphase + narrowphase produce
    // fresh manifolds from current positions, then the solver moves the
    // world. Warm-start impulses arrive via the manifold carry.
    // Broadphase update: single-threaded, fixed body order, shape-list
    // order within a body (both snapshot-deterministic).
    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        if (world->alive[i] == 0 || world->disabled[i] != 0 ||
            world->types[i] == (uint8_t)m2_staticBody ||
            (world->types[i] == (uint8_t)m2_dynamicBody && world->asleep[i] != 0))
        {
            continue;
        }
        for (int32_t s = world->bodyShapeHead[i]; s != -1; s = world->shapeNext[s])
        {
            m2AABB tight = ShapeTightAABB(world, s);
            int32_t tree = ShapeTreeIndex(world, s);
            if (!m2AABB_Contains(world->treeNodes[tree][world->proxyIds[s]].aabb, tight))
            {
                m2Tree_Move(&world->trees[tree], world->treeNodes[tree], world->proxyIds[s],
                            Fatten(tight));
                PushMoved(world, s);
            }
        }
    }
    world->oldPairCount = world->pairCount;
    StashContacts(world);
    UpdatePairs(world);
    uint64_t tPairs = m2TimeNowNs();
    UpdateContacts(world);
    uint64_t tContacts = m2TimeNowNs();

    // Touch transitions in canonical contact order (serial compaction:
    // the topic-08 event law, scalar edition).
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        uint8_t touchingNow = world->manifolds[i].pointCount > 0 ? 1 : 0;
        if (touchingNow != world->pairTouching[i])
        {
            int32_t a = (int32_t)(world->pairKeys[i] >> 32);
            int32_t b = (int32_t)(world->pairKeys[i] & 0xFFFFFFFFu);
            bool sensor = world->shapeSensor[a] != 0 || world->shapeSensor[b] != 0;
            if (touchingNow != 0)
            {
                if (sensor)
                {
                    EmitSensorBegin(world, a, b, i);
                }
                else
                {
                    EmitBegin(world, a, b, i);
                }
            }
            else if (sensor)
            {
                EmitSensorEnd(world, a, b);
            }
            else
            {
                EmitEnd(world, a, b);
            }
            world->pairTouching[i] = touchingNow;
        }
    }

    if (world->particleCount > 0)
    {
        // The whole fluid pass runs once per step before the rigid
        // solve, the reference schedule; pairs freeze at step start.
        // It runs BEFORE the island update so a body the water wakes
        // pulls its whole island awake, the island-coupled sleep law.
        m2SolveParticles(world, dt);
        // Lifetimes count down and expire in ascending slot order at
        // step end; derived from state, so no journal op and it
        // replays and rolls back by itself.
        for (int32_t i = 0; i < world->maxParticleIndex; ++i)
        {
            if (world->particleAlive[i] == 0 || world->particleLifetime[i] <= 0.0f)
            {
                continue;
            }
            world->particleLifetime[i] -= dt;
            if (world->particleLifetime[i] <= 0.0f)
            {
                m2ParticleId dying = {i + 1, worldId.index1, world->particleGenerations[i]};
                uint8_t journalWas = world->journalActive;
                world->journalActive = 0; // derived death is never recorded
                m2World_DestroyParticle(dying);
                world->journalActive = journalWas;
            }
        }
    }
    if (world->maxFvIndex > 0)
    {
        // Buoyancy feeds the force accumulators before the solve, so
        // it integrates alongside gravity and dies with the step.
        m2ApplyFluidVolumes(world, dt);
    }
    if (world->windLinearDrag > 0.0f)
    {
        // Global wind, after buoyancy and before the solve (canonical
        // fluid-then-wind order): an area-weighted linear drag toward
        // the wind velocity into the same force accumulators.
        m2ApplyWind(world, dt);
    }
    m2UpdateIslandsAndWake(world);
    uint64_t tIslands = m2TimeNowNs();
    m2SolveStep(world, dt, substepCount);
    uint64_t tSolve = m2TimeNowNs();
    m2UpdateSleep(world, dt);
#ifdef MAUL2D_VALIDATE
    // The validate build walks the invariants after every step.
    M2_ASSERT(m2World_Validate(worldId));
#endif
    // Freshness streak: two consecutive step-ends asleep guarantee the
    // stashed manifolds were computed from these exact transforms.
    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        if (world->alive[i] == 0 || world->types[i] != (uint8_t)m2_dynamicBody)
        {
            continue;
        }
        world->sleepStreak[i] =
            world->asleep[i] != 0
                ? (uint8_t)(world->sleepStreak[i] < 2 ? world->sleepStreak[i] + 1 : 2)
                : 0;
    }
    uint64_t tEnd = m2TimeNowNs();

    world->profile.stepMs = (float)((double)(tEnd - tStart) * 1.0e-6);
    world->profile.pairsMs = (float)((double)(tPairs - tStart) * 1.0e-6);
    world->profile.contactsMs = (float)((double)(tContacts - tPairs) * 1.0e-6);
    world->profile.solveMs = (float)((double)(tSolve - tIslands) * 1.0e-6);
    world->profile.sleepMs =
        (float)((double)(tIslands - tContacts) * 1.0e-6 + (double)(tEnd - tSolve) * 1.0e-6);

    // Forces live for exactly one step (reference lifetime).
    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        world->forces[i] = (m2Vec2){0.0f, 0.0f};
        world->torques[i] = 0.0f;
    }
    world->stepCount += 1;
}

uint64_t m2World_GetStepCount(m2WorldId worldId)
{
    m2World* world = GetWorld(worldId);
    return world != NULL ? world->stepCount : 0;
}

// --- Snapshot -------------------------------------------------------------------

typedef struct m2SnapshotHeader
{
    uint32_t magic;
    uint32_t version;
    int32_t bodyCapacity;
    int32_t maxBodyIndex;
    uint64_t stepCount;
    m2Vec2 gravity;
    m2Vec2 windVelocity;
    float windLinearDrag;
    int32_t windReserved; // keeps the header 8-aligned and padding-free
    int32_t freeHead;
    int32_t freeTail;
    int32_t freeCount;
    int32_t retiredCount;
    int32_t movedCount;
    int32_t pairCount;
    int32_t shapeCapacity;
    int32_t maxShapeIndex;
    int32_t shapeFreeHead;
    int32_t shapeFreeTail;
    int32_t shapeFreeCount;
    int32_t shapeRetiredCount;
} m2SnapshotHeader;

_Static_assert(sizeof(m2SnapshotHeader) == 96, "snapshot header must be padding-free");

static int32_t WalkBlocks(m2World* world, uint8_t* out, const uint8_t* in, int direction);

// Single source of truth: the size IS the walk (measure mode). The
// duplicated byte formula died here after its third drift (assert-caught
// every time; root cause now removed).
static int32_t BlockBytes(const m2World* world)
{
    return WalkBlocks((m2World*)world, NULL, NULL, 2);
}

int32_t m2World_SnapshotSize(m2WorldId worldId)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL)
    {
        return 0;
    }
    return (int32_t)sizeof(m2SnapshotHeader) + BlockBytes(world);
}

// The block walk is shared by Snapshot and Restore so the layouts can
// never drift apart (direction 0 = write, 1 = read).
static int32_t WalkBlocks(m2World* world, uint8_t* out, const uint8_t* in, int direction)
{
    int32_t cursor = 0;
    size_t cap = (size_t)world->bodyCapacity;
    size_t shapeCap = (size_t)world->shapeCapacity;
#define M2_BLOCK(ptr, bytes)                                                                       \
    do                                                                                             \
    {                                                                                              \
        if (direction == 0)                                                                        \
        {                                                                                          \
            memcpy(out + cursor, ptr, (size_t)(bytes));                                            \
        }                                                                                          \
        else if (direction == 1)                                                                   \
        {                                                                                          \
            memcpy(ptr, in + cursor, (size_t)(bytes));                                             \
        }                                                                                          \
        cursor += (int32_t)(bytes);                                                                \
    } while (0)
    M2_BLOCK(world->transforms, cap * sizeof(m2Transform));
    M2_BLOCK(world->linearVelocities, cap * sizeof(m2Vec2));
    M2_BLOCK(world->angularVelocities, cap * sizeof(float));
    M2_BLOCK(world->gravityScales, cap * sizeof(float));
    M2_BLOCK(world->invMass, cap * sizeof(float));
    M2_BLOCK(world->invInertia, cap * sizeof(float));
    M2_BLOCK(world->localCenters, cap * sizeof(m2Vec2));
    M2_BLOCK(world->asleep, cap * sizeof(uint8_t));
    M2_BLOCK(world->sleepTimes, cap * sizeof(float));
    M2_BLOCK(world->sleepStreak, cap * sizeof(uint8_t));
    M2_BLOCK(world->bullets, cap * sizeof(uint8_t));
    M2_BLOCK(world->userData, cap * sizeof(uint64_t));
    M2_BLOCK(world->types, cap * sizeof(uint8_t));
    M2_BLOCK(world->alive, cap * sizeof(uint8_t));
    M2_BLOCK(world->bodyShapeHead, cap * sizeof(int32_t));
    M2_BLOCK(world->generations, cap * sizeof(uint16_t));
    M2_BLOCK(world->freeQueue, cap * sizeof(int32_t));
    M2_BLOCK(world->shapeGeometry, shapeCap * sizeof(m2ShapeGeometry));
    M2_BLOCK(world->shapeDensity, shapeCap * sizeof(float));
    M2_BLOCK(world->shapeFriction, shapeCap * sizeof(float));
    M2_BLOCK(world->shapeRestitution, shapeCap * sizeof(float));
    M2_BLOCK(world->shapeTangentSpeed, shapeCap * sizeof(float));
    M2_BLOCK(world->shapeUserData, shapeCap * sizeof(uint64_t));
    M2_BLOCK(world->shapeBody, shapeCap * sizeof(int32_t));
    M2_BLOCK(world->shapeNext, shapeCap * sizeof(int32_t));
    M2_BLOCK(world->shapeAlive, shapeCap * sizeof(uint8_t));
    M2_BLOCK(world->shapeGenerations, shapeCap * sizeof(uint16_t));
    M2_BLOCK(world->shapeCategory, (size_t)world->shapeCapacity * sizeof(uint32_t));
    M2_BLOCK(world->shapeMask, (size_t)world->shapeCapacity * sizeof(uint32_t));
    M2_BLOCK(world->shapeGroup, (size_t)world->shapeCapacity * sizeof(int32_t));
    M2_BLOCK(world->shapeSensor, (size_t)world->shapeCapacity * sizeof(uint8_t));
    M2_BLOCK(world->shapeFreeQueue, shapeCap * sizeof(int32_t));
    M2_BLOCK(world->proxyIds, shapeCap * sizeof(int32_t));
    M2_BLOCK(world->inMoved, shapeCap * sizeof(uint8_t));
    M2_BLOCK(world->moved, shapeCap * sizeof(int32_t));
    M2_BLOCK(&world->maxJointIndex, sizeof(int32_t));
    M2_BLOCK(&world->jointFreeHead, sizeof(int32_t));
    M2_BLOCK(&world->jointFreeTail, sizeof(int32_t));
    M2_BLOCK(&world->jointFreeCount, sizeof(int32_t));
    M2_BLOCK(&world->jointRetiredCount, sizeof(int32_t));
    M2_BLOCK(world->jointType, (size_t)world->jointCapacity * sizeof(uint8_t));
    M2_BLOCK(world->jointAlive, (size_t)world->jointCapacity * sizeof(uint8_t));
    M2_BLOCK(world->jointBodyA, (size_t)world->jointCapacity * sizeof(int32_t));
    M2_BLOCK(world->jointBodyB, (size_t)world->jointCapacity * sizeof(int32_t));
    M2_BLOCK(world->jointLocalAnchorA, (size_t)world->jointCapacity * sizeof(m2Vec2));
    M2_BLOCK(world->jointLocalAnchorB, (size_t)world->jointCapacity * sizeof(m2Vec2));
    M2_BLOCK(world->jointLength, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointHertz, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointDamping, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointHertz2, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointDamping2, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointImpulse, (size_t)world->jointCapacity * sizeof(m2Vec2));
    M2_BLOCK(world->jointFlags, (size_t)world->jointCapacity * sizeof(uint8_t));
    M2_BLOCK(world->jointMotorSpeed, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointMaxMotor, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointLower, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointUpper, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointLocalAxisA, (size_t)world->jointCapacity * sizeof(m2Vec2));
    M2_BLOCK(world->jointRefAngle, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointMotorImpulse, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointLowerImpulse, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointUpperImpulse, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointSpringImpulse, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointBreakForce, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointCollide, (size_t)world->jointCapacity * sizeof(uint8_t));
    M2_BLOCK(world->jointTargets, (size_t)world->jointCapacity * sizeof(m2Pos2));
    M2_BLOCK(world->jointTargetsB, (size_t)world->jointCapacity * sizeof(m2Pos2));
    M2_BLOCK(world->jointUserData, (size_t)world->jointCapacity * sizeof(uint64_t));
    M2_BLOCK(world->jointBreakTorque, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointGenerations, (size_t)world->jointCapacity * sizeof(uint16_t));
    M2_BLOCK(world->jointFreeQueue, (size_t)world->jointCapacity * sizeof(int32_t));
    M2_BLOCK(&world->maxChainIndex, sizeof(int32_t));
    M2_BLOCK(&world->chainFreeHead, sizeof(int32_t));
    M2_BLOCK(&world->chainFreeTail, sizeof(int32_t));
    M2_BLOCK(&world->chainFreeCount, sizeof(int32_t));
    M2_BLOCK(&world->chainRetiredCount, sizeof(int32_t));
    M2_BLOCK(&world->lastInvH, sizeof(float));
    M2_BLOCK(&world->sleepEnabled, sizeof(uint8_t));
    M2_BLOCK(world->linearDampings, (size_t)world->bodyCapacity * sizeof(float));
    M2_BLOCK(world->angularDampings, (size_t)world->bodyCapacity * sizeof(float));
    M2_BLOCK(world->fixedRotations, (size_t)world->bodyCapacity * sizeof(uint8_t));
    M2_BLOCK(world->motionLocks, (size_t)world->bodyCapacity * sizeof(uint8_t));
    M2_BLOCK(world->sleepEnables, (size_t)world->bodyCapacity * sizeof(uint8_t));
    M2_BLOCK(world->forces, (size_t)world->bodyCapacity * sizeof(m2Vec2));
    M2_BLOCK(world->torques, (size_t)world->bodyCapacity * sizeof(float));
    M2_BLOCK(world->disabled, (size_t)world->bodyCapacity * sizeof(uint8_t));
    M2_BLOCK(world->dominances, (size_t)world->bodyCapacity * sizeof(int8_t));
    M2_BLOCK(world->shapeChain, (size_t)world->shapeCapacity * sizeof(int32_t));
    M2_BLOCK(world->chainAlive, (size_t)world->shapeCapacity * sizeof(uint8_t));
    M2_BLOCK(world->chainBody, (size_t)world->shapeCapacity * sizeof(int32_t));
    M2_BLOCK(world->chainGenerations, (size_t)world->shapeCapacity * sizeof(uint16_t));
    M2_BLOCK(world->chainFreeQueue, (size_t)world->shapeCapacity * sizeof(int32_t));
    M2_BLOCK(world->trees, M2_TREE_COUNT * sizeof(m2DynamicTree));
    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        M2_BLOCK(world->treeNodes[t], (size_t)world->treeNodeCapacity * sizeof(m2TreeNode));
    }
    M2_BLOCK(world->pairKeys, (size_t)world->pairCapacity * sizeof(uint64_t));
    M2_BLOCK(world->pairTouching, (size_t)world->pairCapacity * sizeof(uint8_t));
    M2_BLOCK(world->manifolds, (size_t)world->pairCapacity * sizeof(m2Manifold));
    if (world->particleCapacity > 0)
    {
        size_t particleCap = (size_t)world->particleCapacity;
        M2_BLOCK(world->particlePositions, particleCap * sizeof(m2Pos2));
        M2_BLOCK(world->particleVelocities, particleCap * sizeof(m2Vec2));
        M2_BLOCK(world->particleAlive, particleCap * sizeof(uint8_t));
        M2_BLOCK(world->particleGenerations, particleCap * sizeof(uint16_t));
        M2_BLOCK(world->particleFlags, particleCap * sizeof(uint32_t));
        M2_BLOCK(world->particleLifetime, particleCap * sizeof(float));
        M2_BLOCK(world->particleUserData, particleCap * sizeof(uint64_t));
        M2_BLOCK(world->particleFreeQueue, particleCap * sizeof(int32_t));
        M2_BLOCK(&world->particleFreeHead, sizeof(int32_t));
        M2_BLOCK(&world->particleFreeCount, sizeof(int32_t));
        M2_BLOCK(&world->particleCount, sizeof(int32_t));
        M2_BLOCK(&world->maxParticleIndex, sizeof(int32_t));
        M2_BLOCK(world->particleSpringA, (size_t)world->particleSpringCapacity * sizeof(int32_t));
        M2_BLOCK(world->particleSpringB, (size_t)world->particleSpringCapacity * sizeof(int32_t));
        M2_BLOCK(world->particleSpringRest, (size_t)world->particleSpringCapacity * sizeof(float));
        M2_BLOCK(&world->particleSpringCount, sizeof(int32_t));
        M2_BLOCK(world->particleTriadA, (size_t)world->particleTriadCapacity * sizeof(int32_t));
        M2_BLOCK(world->particleTriadB, (size_t)world->particleTriadCapacity * sizeof(int32_t));
        M2_BLOCK(world->particleTriadC, (size_t)world->particleTriadCapacity * sizeof(int32_t));
        M2_BLOCK(world->particleTriadPA, (size_t)world->particleTriadCapacity * sizeof(m2Vec2));
        M2_BLOCK(world->particleTriadPB, (size_t)world->particleTriadCapacity * sizeof(m2Vec2));
        M2_BLOCK(world->particleTriadPC, (size_t)world->particleTriadCapacity * sizeof(m2Vec2));
        M2_BLOCK(&world->particleTriadCount, sizeof(int32_t));
    }
    if (world->fvCapacity > 0)
    {
        size_t fvCap = (size_t)world->fvCapacity;
        M2_BLOCK(world->fvLower, fvCap * sizeof(m2Pos2));
        M2_BLOCK(world->fvUpper, fvCap * sizeof(m2Pos2));
        M2_BLOCK(world->fvSurface, fvCap * sizeof(double));
        M2_BLOCK(world->fvDensity, fvCap * sizeof(float));
        M2_BLOCK(world->fvLinearDrag, fvCap * sizeof(float));
        M2_BLOCK(world->fvAngularDrag, fvCap * sizeof(float));
        M2_BLOCK(world->fvFlow, fvCap * sizeof(m2Vec2));
        M2_BLOCK(world->fvUserData, fvCap * sizeof(uint64_t));
        M2_BLOCK(world->fvAlive, fvCap * sizeof(uint8_t));
        M2_BLOCK(world->fvGenerations, fvCap * sizeof(uint16_t));
        M2_BLOCK(world->fvFreeQueue, fvCap * sizeof(int32_t));
        M2_BLOCK(&world->fvFreeHead, sizeof(int32_t));
        M2_BLOCK(&world->fvFreeCount, sizeof(int32_t));
        M2_BLOCK(&world->maxFvIndex, sizeof(int32_t));
    }
#undef M2_BLOCK
    return cursor;
}

int32_t m2World_Snapshot(m2WorldId worldId, void* buffer, int32_t capacity)
{
    m2World* world = GetWorld(worldId);
    int32_t size = m2World_SnapshotSize(worldId);
    if (world == NULL || buffer == NULL || capacity < size)
    {
        return 0;
    }

    m2SnapshotHeader header;
    memset(&header, 0, sizeof(header));
    header.magic = M2_SNAPSHOT_MAGIC;
    header.version = M2_SNAPSHOT_VERSION;
    header.bodyCapacity = world->bodyCapacity;
    header.maxBodyIndex = world->maxBodyIndex;
    header.stepCount = world->stepCount;
    header.gravity = world->gravity;
    header.windVelocity = world->windVelocity;
    header.windLinearDrag = world->windLinearDrag;
    header.windReserved = 0;
    header.freeHead = world->freeHead;
    header.freeTail = world->freeTail;
    header.freeCount = world->freeCount;
    header.retiredCount = world->retiredCount;
    header.movedCount = world->movedCount;
    header.pairCount = world->pairCount;
    header.shapeCapacity = world->shapeCapacity;
    header.maxShapeIndex = world->maxShapeIndex;
    header.shapeFreeHead = world->shapeFreeHead;
    header.shapeFreeTail = world->shapeFreeTail;
    header.shapeFreeCount = world->shapeFreeCount;
    header.shapeRetiredCount = world->shapeRetiredCount;

    uint8_t* out = buffer;
    memcpy(out, &header, sizeof(header));
    int32_t cursor = (int32_t)sizeof(header) + WalkBlocks(world, out + sizeof(header), NULL, 0);
    M2_ASSERT(cursor == size);
    return cursor;
}

bool m2World_Restore(m2WorldId worldId, const void* buffer, int32_t size)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL || buffer == NULL || size < (int32_t)sizeof(m2SnapshotHeader))
    {
        return false;
    }
    m2SnapshotHeader header;
    memcpy(&header, buffer, sizeof(header));
    if (header.magic != M2_SNAPSHOT_MAGIC || header.version != M2_SNAPSHOT_VERSION ||
        header.bodyCapacity != world->bodyCapacity ||
        header.shapeCapacity != world->shapeCapacity ||
        size != (int32_t)sizeof(header) + BlockBytes(world))
    {
        return false;
    }

    world->maxBodyIndex = header.maxBodyIndex;
    world->stepCount = header.stepCount;
    world->gravity = header.gravity;
    world->windVelocity = header.windVelocity;
    world->windLinearDrag = header.windLinearDrag;
    world->freeHead = header.freeHead;
    world->freeTail = header.freeTail;
    world->freeCount = header.freeCount;
    world->retiredCount = header.retiredCount;
    world->movedCount = header.movedCount;
    world->pairCount = header.pairCount;
    world->maxShapeIndex = header.maxShapeIndex;
    world->shapeFreeHead = header.shapeFreeHead;
    world->shapeFreeTail = header.shapeFreeTail;
    world->shapeFreeCount = header.shapeFreeCount;
    world->shapeRetiredCount = header.shapeRetiredCount;

    const uint8_t* in = buffer;
    int32_t cursor = (int32_t)sizeof(header) + WalkBlocks(world, NULL, in + sizeof(header), 1);
    M2_ASSERT(cursor == size);
    (void)cursor;

    // Restores are first-class journal citizens: the tape carries the
    // snapshot itself, so rollback-heavy sessions replay bit-exactly.
    // The price is tape size, and that is the caller's tradeoff.
    m2JournalRecordRestore(world, buffer, size);

    // Events are an observer stream from an abandoned timeline: cleared
    // on restore, re-emitted by re-simulation (RT1-ROLL-3 / RT1-API-3).
    world->beginEventCount = 0;
    world->endEventCount = 0;
    world->pendingEndCount = 0;
    world->sensorBeginCount = 0;
    world->sensorEndCount = 0;
    world->pendingSensorEndCount = 0;
    world->jointBreakEventCount = 0;
    return true;
}

uint64_t m2World_Hash(m2WorldId worldId)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL)
    {
        return 0;
    }
    uint64_t h = M2_HASH_INIT;
    h = m2Hash64(h, &world->stepCount, (int32_t)sizeof(world->stepCount));
    h = m2Hash64(h, &world->gravity, (int32_t)sizeof(world->gravity));
    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        if (world->alive[i] == 0)
        {
            continue;
        }
        h = m2Hash64(h, &world->transforms[i], (int32_t)sizeof(m2Transform));
        h = m2Hash64(h, &world->linearVelocities[i], (int32_t)sizeof(m2Vec2));
        h = m2Hash64(h, &world->angularVelocities[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->invMass[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->invInertia[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->localCenters[i], (int32_t)sizeof(m2Vec2));
        h = m2Hash64(h, &world->types[i], (int32_t)sizeof(uint8_t));
        h = m2Hash64(h, &world->asleep[i], (int32_t)sizeof(uint8_t));
        h = m2Hash64(h, &world->sleepTimes[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->sleepStreak[i], 1);
        h = m2Hash64(h, &world->bullets[i], (int32_t)sizeof(uint8_t));
    }
    h = m2Hash64(h, world->pairKeys, world->pairCount * (int32_t)sizeof(uint64_t));
    h = m2Hash64(h, world->manifolds, world->pairCount * (int32_t)sizeof(m2Manifold));
    for (int32_t i = 0; i < world->maxJointIndex; ++i)
    {
        if (world->jointAlive[i] == 0)
        {
            continue;
        }
        h = m2Hash64(h, &world->jointImpulse[i], (int32_t)sizeof(m2Vec2));
        h = m2Hash64(h, &world->jointMotorImpulse[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->jointLowerImpulse[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->jointUpperImpulse[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->jointSpringImpulse[i], (int32_t)sizeof(float));
    }
    if (world->fvCapacity > 0)
    {
        for (int32_t i = 0; i < world->maxFvIndex; ++i)
        {
            h = m2Hash64(h, &world->fvAlive[i], 1);
            if (world->fvAlive[i] == 0)
            {
                continue;
            }
            h = m2Hash64(h, &world->fvSurface[i], (int32_t)sizeof(double));
        }
    }
    if (world->particleCapacity > 0)
    {
        h = m2Hash64(h, &world->particleCount, (int32_t)sizeof(int32_t));
        for (int32_t i = 0; i < world->maxParticleIndex; ++i)
        {
            h = m2Hash64(h, &world->particleAlive[i], 1);
            if (world->particleAlive[i] == 0)
            {
                continue;
            }
            h = m2Hash64(h, &world->particlePositions[i], (int32_t)sizeof(m2Pos2));
            h = m2Hash64(h, &world->particleVelocities[i], (int32_t)sizeof(m2Vec2));
            h = m2Hash64(h, &world->particleFlags[i], (int32_t)sizeof(uint32_t));
            h = m2Hash64(h, &world->particleLifetime[i], (int32_t)sizeof(float));
            h = m2Hash64(h, &world->particleUserData[i], (int32_t)sizeof(uint64_t));
        }
        h = m2Hash64(h, &world->particleSpringCount, (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particleSpringA,
                     world->particleSpringCount * (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particleSpringB,
                     world->particleSpringCount * (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particleSpringRest,
                     world->particleSpringCount * (int32_t)sizeof(float));
        h = m2Hash64(h, &world->particleTriadCount, (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particleTriadA,
                     world->particleTriadCount * (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particleTriadB,
                     world->particleTriadCount * (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particleTriadC,
                     world->particleTriadCount * (int32_t)sizeof(int32_t));
    }
    return h;
}

// --- Bodies ---------------------------------------------------------------------

m2BodyDef m2DefaultBodyDef(void)
{
    m2BodyDef def;
    memset(&def, 0, sizeof(def));
    def.type = m2_staticBody;
    def.rotation = (m2Rot){1.0f, 0.0f};
    def.gravityScale = 1.0f;
    def.enableSleep = true;
    def.isEnabled = true;
    def.internalValue = M2_BODY_COOKIE;
    return def;
}

m2BodyId m2CreateBody(m2WorldId worldId, const m2BodyDef* def)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_BODY_COOKIE)
    {
        M2_ASSERT(false);
        return m2_nullBodyId;
    }
    if (world->freeCount == 0)
    {
        return m2_nullBodyId;
    }

    int32_t index = world->freeQueue[world->freeHead];
    world->freeHead = (world->freeHead + 1) % world->bodyCapacity;
    world->freeCount -= 1;

    world->transforms[index].p = def->position;
    world->transforms[index].q = m2NormalizeRot(def->rotation);
    world->linearVelocities[index] = def->linearVelocity;
    // A rotation-locked body never spins, so a fixed-rotation (or
    // angularZ-locked) body drops any initial angular velocity at birth,
    // the same as the runtime setter does.
    world->angularVelocities[index] =
        (def->fixedRotation || def->motionLocks.angularZ) ? 0.0f : def->angularVelocity;
    world->gravityScales[index] = def->gravityScale;
    world->linearDampings[index] = def->linearDamping;
    world->angularDampings[index] = def->angularDamping;
    world->fixedRotations[index] = (def->fixedRotation || def->motionLocks.angularZ) ? 1 : 0;
    world->motionLocks[index] =
        (uint8_t)((def->motionLocks.linearX ? 1u : 0u) | (def->motionLocks.linearY ? 2u : 0u));
    world->sleepEnables[index] = def->enableSleep ? 1 : 0;
    world->forces[index] = (m2Vec2){0.0f, 0.0f};
    world->torques[index] = 0.0f;
    world->disabled[index] = def->isEnabled ? 0 : 1;
    world->dominances[index] = def->dominance;
    world->userData[index] = def->userData;
    world->types[index] = (uint8_t)def->type;
    world->alive[index] = 1;
    world->bodyShapeHead[index] = -1;
    world->invMass[index] = def->type == m2_dynamicBody ? 1.0f : 0.0f; // shapeless floor
    world->localCenters[index] = (m2Vec2){0.0f, 0.0f};
    world->invInertia[index] = 0.0f;
    world->asleep[index] = 0;
    world->sleepTimes[index] = 0.0f;
    world->sleepStreak[index] = 0;
    world->bullets[index] = def->isBullet ? 1 : 0;
    if (index + 1 > world->maxBodyIndex)
    {
        world->maxBodyIndex = index + 1;
    }

    m2BodyId id = {index + 1, worldId.index1, world->generations[index]};

    if (world->journalActive != 0)

    {

        struct

        {

            m2BodyDef def;

            m2BodyId expected;

        } record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = id;

        m2JournalRecord(world, m2_opCreateBody, &record, (int32_t)sizeof(record));
    }
    return id;
}

// M19 bookending shared by destroy and disable: end every touching
// contact of this shape, wake its riders, drop the proxy, prune pairs.
static void RetireShapeFromBroadphase(m2World* world, int32_t shapeIndex)
{
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        if (world->pairTouching[i] == 0)
        {
            continue;
        }
        int32_t a = (int32_t)(world->pairKeys[i] >> 32);
        int32_t b = (int32_t)(world->pairKeys[i] & 0xFFFFFFFFu);
        if (a != shapeIndex && b != shapeIndex)
        {
            continue;
        }
        // The same law as teleports and type changes: whoever was
        // resting on this shape must notice it vanish, or sleepers
        // float on a memory. (Caught by the floor-yank probe.)
        int32_t partner = world->shapeBody[a == shapeIndex ? b : a];
        if (world->types[partner] == (uint8_t)m2_dynamicBody)
        {
            world->asleep[partner] = 0;
            world->sleepTimes[partner] = 0.0f;
        }
        bool sensor = world->shapeSensor[a] != 0 || world->shapeSensor[b] != 0;
        m2ContactEndEvent* queue = sensor ? world->pendingSensorEnd : world->pendingEndEvents;
        int32_t* queueCount = sensor ? &world->pendingSensorEndCount : &world->pendingEndCount;
        if (*queueCount < world->pairCapacity)
        {
            m2ContactEndEvent* e = &queue[(*queueCount)++];
            e->shapeIdA = MakeShapeId(world, a);
            e->shapeIdB = MakeShapeId(world, b);
            e->step = world->stepCount;
        }
    }

    if (world->proxyIds[shapeIndex] != M2_NULL_NODE)
    {
        int32_t tree = ShapeTreeIndex(world, shapeIndex);
        m2Tree_Remove(&world->trees[tree], world->treeNodes[tree], world->proxyIds[shapeIndex]);
        world->proxyIds[shapeIndex] = M2_NULL_NODE;
    }
    PrunePairsOfShape(world, shapeIndex);
}

static void DestroyShapeInternal(m2World* world, int32_t shapeIndex)
{
    RetireShapeFromBroadphase(world, shapeIndex);
    world->shapeAlive[shapeIndex] = 0;
    if (world->shapeGenerations[shapeIndex] == UINT16_MAX)
    {
        world->shapeRetiredCount += 1;
        return;
    }
    world->shapeGenerations[shapeIndex] += 1;
    world->shapeFreeQueue[world->shapeFreeTail] = shapeIndex;
    world->shapeFreeTail = (world->shapeFreeTail + 1) % world->shapeCapacity;
    world->shapeFreeCount += 1;
}

void m2DestroyShape(m2ShapeId shapeId)
{
    m2World* world = WorldFromIndex(shapeId.world0);
    if (world == NULL)
    {
        return;
    }
    int32_t index = shapeId.index1 - 1;
    if (index < 0 || index >= world->shapeCapacity || world->shapeAlive[index] == 0 ||
        world->shapeGenerations[index] != shapeId.generation)
    {
        return;
    }
    m2JournalRecord(world, m2_opDestroyShape, &shapeId, (int32_t)sizeof(shapeId));

    int32_t bodyIndex = world->shapeBody[index];
    // Unlink from the body's shape list (insertion-ordered, singly
    // linked - the walk is canonical).
    if (world->bodyShapeHead[bodyIndex] == index)
    {
        world->bodyShapeHead[bodyIndex] = world->shapeNext[index];
    }
    else
    {
        for (int32_t s = world->bodyShapeHead[bodyIndex]; s != -1; s = world->shapeNext[s])
        {
            if (world->shapeNext[s] == index)
            {
                world->shapeNext[s] = world->shapeNext[index];
                break;
            }
        }
    }
    world->shapeNext[index] = -1;

    DestroyShapeInternal(world, index);
    RecomputeMass(world, bodyIndex);
    if (world->types[bodyIndex] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[bodyIndex] = 0;
        world->sleepTimes[bodyIndex] = 0.0f;
    }
}

static void RetireChainSlot(m2World* world, int32_t chainIndex)
{
    world->chainAlive[chainIndex] = 0;
    if (world->chainGenerations[chainIndex] == UINT16_MAX)
    {
        world->chainRetiredCount += 1;
        return;
    }
    world->chainGenerations[chainIndex] += 1;
    world->chainFreeQueue[world->chainFreeTail] = chainIndex;
    world->chainFreeTail = (world->chainFreeTail + 1) % world->shapeCapacity;
    world->chainFreeCount += 1;
}

void m2DestroyBody(m2BodyId bodyId)
{
    m2World* world = GetBodyWorld(bodyId);
    if (world == NULL)
    {
        return;
    }
    int32_t index = BodySlot(world, bodyId);
    if (index < 0)
    {
        return;
    }
    m2JournalRecord(world, m2_opDestroyBody, &bodyId, (int32_t)sizeof(bodyId));

    // Joints attached to this body die with it; the counterpart body
    // wakes (a support vanished).
    for (int32_t j = 0; j < world->maxJointIndex; ++j)
    {
        if (world->jointAlive[j] == 0 ||
            (world->jointBodyA[j] != index && world->jointBodyB[j] != index))
        {
            continue;
        }
        int32_t other = world->jointBodyA[j] == index ? world->jointBodyB[j] : world->jointBodyA[j];
        if (world->types[other] == (uint8_t)m2_dynamicBody)
        {
            world->asleep[other] = 0;
            world->sleepTimes[other] = 0.0f;
        }
        world->jointAlive[j] = 0;
        if (world->jointGenerations[j] == UINT16_MAX)
        {
            world->jointRetiredCount += 1;
        }
        else
        {
            world->jointGenerations[j] += 1;
            world->jointFreeQueue[world->jointFreeTail] = j;
            world->jointFreeTail = (world->jointFreeTail + 1) % world->jointCapacity;
            world->jointFreeCount += 1;
        }
    }

    // Chains die with their body; their slots retire so stale chain
    // ids miss on generation, same as every other id kind.
    for (int32_t c = 0; c < world->maxChainIndex; ++c)
    {
        if (world->chainAlive[c] != 0 && world->chainBody[c] == index)
        {
            RetireChainSlot(world, c);
        }
    }

    // Cascade: destroy the shape list (the contact-killing path that event
    // bookending will hang end-touch emission on, registry M19).
    int32_t s = world->bodyShapeHead[index];
    while (s != -1)
    {
        int32_t next = world->shapeNext[s];
        DestroyShapeInternal(world, s);
        s = next;
    }
    world->bodyShapeHead[index] = -1;

    world->alive[index] = 0;
    if (world->generations[index] == UINT16_MAX)
    {
        world->retiredCount += 1;
        return;
    }
    world->generations[index] += 1;
    world->freeQueue[world->freeTail] = index;
    world->freeTail = (world->freeTail + 1) % world->bodyCapacity;
    world->freeCount += 1;
}

bool m2Body_IsValid(m2BodyId bodyId)
{
    m2World* world = GetBodyWorld(bodyId);
    return world != NULL && BodySlot(world, bodyId) >= 0;
}

#define M2_BODY_GETTER(returnType, name, expr, fallback)                                           \
    returnType name(m2BodyId bodyId)                                                               \
    {                                                                                              \
        m2World* world = GetBodyWorld(bodyId);                                                     \
        int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;                              \
        if (index < 0)                                                                             \
        {                                                                                          \
            M2_ASSERT(false);                                                                      \
            return fallback;                                                                       \
        }                                                                                          \
        return expr;                                                                               \
    }

M2_BODY_GETTER(m2Transform, m2Body_GetTransform, world->transforms[index],
               ((m2Transform){{0.0, 0.0}, {1.0f, 0.0f}}))
M2_BODY_GETTER(m2Pos2, m2Body_GetPosition, world->transforms[index].p, ((m2Pos2){0.0, 0.0}))
M2_BODY_GETTER(m2Rot, m2Body_GetRotation, world->transforms[index].q, ((m2Rot){1.0f, 0.0f}))
M2_BODY_GETTER(m2Vec2, m2Body_GetLinearVelocity, world->linearVelocities[index],
               ((m2Vec2){0.0f, 0.0f}))
M2_BODY_GETTER(float, m2Body_GetAngularVelocity, world->angularVelocities[index], 0.0f)
M2_BODY_GETTER(uint64_t, m2Body_GetUserData, world->userData[index], 0)

float m2Body_GetMass(m2BodyId bodyId)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return 0.0f;
    }
    return world->invMass[index] > 0.0f ? 1.0f / world->invMass[index] : 0.0f;
}

void m2Body_SetLinearVelocity(m2BodyId bodyId, m2Vec2 velocity)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return;
    }
    world->linearVelocities[index] = velocity;
    world->asleep[index] = 0;
    world->sleepTimes[index] = 0.0f;
    if (world->journalActive != 0)
    {
        struct
        {
            m2BodyId body;
            m2Vec2 value;
        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.value = velocity;
        m2JournalRecord(world, m2_opSetLinearVelocity, &record, (int32_t)sizeof(record));
    }
}

void m2Body_SetAngularVelocity(m2BodyId bodyId, float velocity)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return;
    }
    world->angularVelocities[index] = velocity;
    world->asleep[index] = 0;
    world->sleepTimes[index] = 0.0f;
    if (world->journalActive != 0)
    {
        struct
        {
            m2BodyId body;
            float value;
        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.value = velocity;
        m2JournalRecord(world, m2_opSetAngularVelocity, &record, (int32_t)sizeof(record));
    }
}

void m2Body_ApplyLinearImpulse(m2BodyId bodyId, m2Vec2 impulse, m2Pos2 worldPoint)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0 || world->types[index] != (uint8_t)m2_dynamicBody)
    {
        M2_ASSERT(false);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2BodyId body;
            m2Vec2 impulse;
            m2Pos2 point;
        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.impulse = impulse;
        record.point = worldPoint;
        m2JournalRecord(world, m2_opApplyLinearImpulse, &record, (int32_t)sizeof(record));
    }
    // Arm from the center of mass; the single f64 crossing.
    m2Transform xf = world->transforms[index];
    m2Vec2 rlc = {xf.q.c * world->localCenters[index].x - xf.q.s * world->localCenters[index].y,
                  xf.q.s * world->localCenters[index].x + xf.q.c * world->localCenters[index].y};
    m2Vec2 r = {(float)(worldPoint.x - xf.p.x) - rlc.x, (float)(worldPoint.y - xf.p.y) - rlc.y};
    world->linearVelocities[index].x += world->invMass[index] * impulse.x;
    world->linearVelocities[index].y += world->invMass[index] * impulse.y;
    world->angularVelocities[index] +=
        world->invInertia[index] * (r.x * impulse.y - r.y * impulse.x);
    world->asleep[index] = 0;
    world->sleepTimes[index] = 0.0f;
}

void m2Body_ApplyForce(m2BodyId bodyId, m2Vec2 force, m2Pos2 worldPoint)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0 || world->types[index] != (uint8_t)m2_dynamicBody)
    {
        M2_ASSERT(false);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2BodyId body;
            m2Vec2 force;
            m2Pos2 point;
        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.force = force;
        record.point = worldPoint;
        m2JournalRecord(world, m2_opApplyForce, &record, (int32_t)sizeof(record));
    }
    // Arm from the center of mass, exactly like the impulse path.
    m2Transform xf = world->transforms[index];
    m2Vec2 rlc = {xf.q.c * world->localCenters[index].x - xf.q.s * world->localCenters[index].y,
                  xf.q.s * world->localCenters[index].x + xf.q.c * world->localCenters[index].y};
    m2Vec2 r = {(float)(worldPoint.x - xf.p.x) - rlc.x, (float)(worldPoint.y - xf.p.y) - rlc.y};
    world->forces[index].x += force.x;
    world->forces[index].y += force.y;
    world->torques[index] += r.x * force.y - r.y * force.x;
    world->asleep[index] = 0;
    world->sleepTimes[index] = 0.0f;
}

void m2Body_ApplyForceToCenter(m2BodyId bodyId, m2Vec2 force)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0 || world->types[index] != (uint8_t)m2_dynamicBody)
    {
        M2_ASSERT(false);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2BodyId body;
            m2Vec2 force;
        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.force = force;
        m2JournalRecord(world, m2_opApplyForceCenter, &record, (int32_t)sizeof(record));
    }
    world->forces[index].x += force.x;
    world->forces[index].y += force.y;
    world->asleep[index] = 0;
    world->sleepTimes[index] = 0.0f;
}

void m2Body_ApplyTorque(m2BodyId bodyId, float torque)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0 || world->types[index] != (uint8_t)m2_dynamicBody)
    {
        M2_ASSERT(false);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2BodyId body;
            float torque;
        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.torque = torque;
        m2JournalRecord(world, m2_opApplyTorque, &record, (int32_t)sizeof(record));
    }
    world->torques[index] += torque;
    world->asleep[index] = 0;
    world->sleepTimes[index] = 0.0f;
}

// One journaled channel for the body dynamics parameters, mirroring
// the shape and joint channels.
void m2SetBodyParamInternal(m2World* world, m2BodyId bodyId, uint8_t param, float value)
{
    int32_t index = BodySlot(world, bodyId);
    if (index < 0)
    {
        M2_ASSERT(false);
        return;
    }
    if (world->journalActive != 0)
    {
        struct m2OpBodyParam
        {
            m2BodyId body;
            float value;
            uint8_t param;
        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.value = value;
        record.param = param;
        m2JournalRecord(world, m2_opBodyParam, &record, (int32_t)sizeof(record));
    }
    switch (param)
    {
    case 0:
        world->linearDampings[index] = value;
        break;
    case 1:
        world->angularDampings[index] = value;
        break;
    case 2:
        world->gravityScales[index] = value;
        break;
    case 3:
        // Fixed rotation is a mass property: inertia recomputes, spin
        // stops now (a frozen axis with leftover spin is a lie).
        world->fixedRotations[index] = value != 0.0f ? 1 : 0;
        world->angularVelocities[index] = 0.0f;
        RecomputeMass(world, index);
        if (world->types[index] == (uint8_t)m2_dynamicBody)
        {
            world->asleep[index] = 0;
            world->sleepTimes[index] = 0.0f;
        }
        break;
    case 4:
        world->sleepEnables[index] = value != 0.0f ? 1 : 0;
        if (value == 0.0f && world->types[index] == (uint8_t)m2_dynamicBody)
        {
            world->asleep[index] = 0; // must not stay asleep illegally
            world->sleepTimes[index] = 0.0f;
        }
        break;
    case 5:
        // Lock linear X: a locked axis holds still, so stop it now and
        // wake the body (a frozen axis with leftover velocity is a lie,
        // same discipline as fixed rotation).
        world->motionLocks[index] = value != 0.0f ? (uint8_t)(world->motionLocks[index] | 1u)
                                                  : (uint8_t)(world->motionLocks[index] & ~1u);
        world->linearVelocities[index].x = value != 0.0f ? 0.0f : world->linearVelocities[index].x;
        if (world->types[index] == (uint8_t)m2_dynamicBody)
        {
            world->asleep[index] = 0;
            world->sleepTimes[index] = 0.0f;
        }
        break;
    case 6:
        // Lock linear Y.
        world->motionLocks[index] = value != 0.0f ? (uint8_t)(world->motionLocks[index] | 2u)
                                                  : (uint8_t)(world->motionLocks[index] & ~2u);
        world->linearVelocities[index].y = value != 0.0f ? 0.0f : world->linearVelocities[index].y;
        if (world->types[index] == (uint8_t)m2_dynamicBody)
        {
            world->asleep[index] = 0;
            world->sleepTimes[index] = 0.0f;
        }
        break;
    default:
        M2_ASSERT(false); // unknown body param
        break;
    }
}

void m2Body_SetLinearDamping(m2BodyId bodyId, float damping)
{
    m2World* world = GetBodyWorld(bodyId);
    if (world != NULL)
    {
        m2SetBodyParamInternal(world, bodyId, 0, m2MaxF(damping, 0.0f));
    }
}

float m2Body_GetLinearDamping(m2BodyId bodyId)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    return index >= 0 ? world->linearDampings[index] : 0.0f;
}

void m2Body_SetAngularDamping(m2BodyId bodyId, float damping)
{
    m2World* world = GetBodyWorld(bodyId);
    if (world != NULL)
    {
        m2SetBodyParamInternal(world, bodyId, 1, m2MaxF(damping, 0.0f));
    }
}

float m2Body_GetAngularDamping(m2BodyId bodyId)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    return index >= 0 ? world->angularDampings[index] : 0.0f;
}

void m2Body_SetGravityScale(m2BodyId bodyId, float scale)
{
    m2World* world = GetBodyWorld(bodyId);
    if (world != NULL)
    {
        m2SetBodyParamInternal(world, bodyId, 2, scale);
    }
}

void m2Body_SetFixedRotation(m2BodyId bodyId, bool flag)
{
    m2World* world = GetBodyWorld(bodyId);
    if (world != NULL)
    {
        m2SetBodyParamInternal(world, bodyId, 3, flag ? 1.0f : 0.0f);
    }
}

bool m2Body_IsFixedRotation(m2BodyId bodyId)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    return index >= 0 && world->fixedRotations[index] != 0;
}

void m2Body_SetMotionLocks(m2BodyId bodyId, m2MotionLocks locks)
{
    m2World* world = GetBodyWorld(bodyId);
    if (world != NULL)
    {
        // Each axis rides the journaled body-param channel so a replay
        // reproduces the locks. angularZ is the fixed-rotation lock.
        m2SetBodyParamInternal(world, bodyId, 5, locks.linearX ? 1.0f : 0.0f);
        m2SetBodyParamInternal(world, bodyId, 6, locks.linearY ? 1.0f : 0.0f);
        m2SetBodyParamInternal(world, bodyId, 3, locks.angularZ ? 1.0f : 0.0f);
    }
}

m2MotionLocks m2Body_GetMotionLocks(m2BodyId bodyId)
{
    m2MotionLocks locks = {false, false, false};
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index >= 0)
    {
        locks.linearX = (world->motionLocks[index] & 1u) != 0;
        locks.linearY = (world->motionLocks[index] & 2u) != 0;
        locks.angularZ = world->fixedRotations[index] != 0;
    }
    return locks;
}

void m2Body_EnableSleep(m2BodyId bodyId, bool flag)
{
    m2World* world = GetBodyWorld(bodyId);
    if (world != NULL)
    {
        m2SetBodyParamInternal(world, bodyId, 4, flag ? 1.0f : 0.0f);
    }
}

bool m2Body_IsSleepEnabled(m2BodyId bodyId)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    return index >= 0 && world->sleepEnables[index] != 0;
}

void m2World_EnableSleeping(m2WorldId worldId, bool flag)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL)
    {
        return;
    }
    uint8_t next = flag ? 1 : 0;
    if (world->sleepEnabled == next)
    {
        return; // no-op stays unjournaled, like SetGravity
    }
    if (world->journalActive != 0)
    {
        struct
        {
            uint8_t flag;
        } record;
        record.flag = next;
        m2JournalRecord(world, m2_opEnableSleeping, &record, (int32_t)sizeof(record));
    }
    world->sleepEnabled = next;
    if (next == 0)
    {
        // The rule that let them sleep is gone; wake everyone (the
        // SetGravity law).
        for (int32_t i = 0; i < world->maxBodyIndex; ++i)
        {
            if (world->alive[i] != 0 && world->types[i] == (uint8_t)m2_dynamicBody)
            {
                world->asleep[i] = 0;
                world->sleepTimes[i] = 0.0f;
            }
        }
    }
}

bool m2World_IsSleepingEnabled(m2WorldId worldId)
{
    m2World* world = GetWorld(worldId);
    return world != NULL && world->sleepEnabled != 0;
}

void m2Body_ApplyAngularImpulse(m2BodyId bodyId, float impulse)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0 || world->types[index] != (uint8_t)m2_dynamicBody)
    {
        M2_ASSERT(false);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2BodyId body;
            float value;
        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.value = impulse;
        m2JournalRecord(world, m2_opApplyAngularImpulse, &record, (int32_t)sizeof(record));
    }
    world->angularVelocities[index] += world->invInertia[index] * impulse;
    world->asleep[index] = 0;
    world->sleepTimes[index] = 0.0f;
}

void m2Body_SetTransform(m2BodyId bodyId, m2Pos2 position, m2Rot rotation)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2BodyId body;
            m2Pos2 position;
            m2Rot rotation;
        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.position = position;
        record.rotation = rotation;
        m2JournalRecord(world, m2_opSetTransform, &record, (int32_t)sizeof(record));
    }

    // Whatever this body was resting on - or holding up - must notice.
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        if (world->pairTouching[i] == 0)
        {
            continue;
        }
        int32_t a = world->shapeBody[(int32_t)(world->pairKeys[i] >> 32)];
        int32_t b = world->shapeBody[(int32_t)(world->pairKeys[i] & 0xFFFFFFFFu)];
        if (a != index && b != index)
        {
            continue;
        }
        int32_t other = a == index ? b : a;
        if (world->types[other] == (uint8_t)m2_dynamicBody)
        {
            world->asleep[other] = 0;
            world->sleepTimes[other] = 0.0f;
        }
    }

    world->transforms[index].p = position;
    world->transforms[index].q = rotation;
    if (world->types[index] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[index] = 0;
        world->sleepTimes[index] = 0.0f;
    }

    // Broadphase refresh right now: the next step's pair update must
    // see the new home, not the old one.
    for (int32_t shape = world->bodyShapeHead[index]; shape != -1; shape = world->shapeNext[shape])
    {
        if (world->proxyIds[shape] == M2_NULL_NODE)
        {
            continue;
        }
        m2AABB tight = ShapeTightAABB(world, shape);
        int32_t tree = ShapeTreeIndex(world, shape);
        if (!m2AABB_Contains(world->treeNodes[tree][world->proxyIds[shape]].aabb, tight))
        {
            m2Tree_Move(&world->trees[tree], world->treeNodes[tree], world->proxyIds[shape],
                        Fatten(tight));
        }
        PushMoved(world, shape);
    }
}

void m2Body_SetType(m2BodyId bodyId, m2BodyType type)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0 || (int32_t)type < 0 || (int32_t)type >= M2_TREE_COUNT)
    {
        M2_ASSERT(false);
        return;
    }
    if (world->types[index] == (uint8_t)type)
    {
        return; // no-op, and deliberately not journaled
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2BodyId body;
            uint8_t type;
        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.type = (uint8_t)type;
        m2JournalRecord(world, m2_opSetType, &record, (int32_t)sizeof(record));
    }

    // Contacts change meaning: wake every touching partner while the
    // old type is still in effect.
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        if (world->pairTouching[i] == 0)
        {
            continue;
        }
        int32_t a = world->shapeBody[(int32_t)(world->pairKeys[i] >> 32)];
        int32_t b = world->shapeBody[(int32_t)(world->pairKeys[i] & 0xFFFFFFFFu)];
        if (a != index && b != index)
        {
            continue;
        }
        int32_t other = a == index ? b : a;
        if (world->types[other] == (uint8_t)m2_dynamicBody)
        {
            world->asleep[other] = 0;
            world->sleepTimes[other] = 0.0f;
        }
    }

    // Proxies move between the per-type trees; marking them moved also
    // purges stale pairs and re-forms the valid ones, which is exactly
    // the M19 path for pairs that stop making sense.
    int32_t oldTree = world->types[index];
    world->types[index] = (uint8_t)type;
    for (int32_t shape = world->bodyShapeHead[index]; shape != -1; shape = world->shapeNext[shape])
    {
        if (world->proxyIds[shape] == M2_NULL_NODE)
        {
            continue;
        }
        m2Tree_Remove(&world->trees[oldTree], world->treeNodes[oldTree], world->proxyIds[shape]);
        world->proxyIds[shape] = m2Tree_Insert(&world->trees[type], world->treeNodes[type],
                                               Fatten(ShapeTightAABB(world, shape)), shape);
        M2_ASSERT(world->proxyIds[shape] != M2_NULL_NODE);
        PushMoved(world, shape);
    }

    RecomputeMass(world, index);
    if (type == m2_staticBody)
    {
        world->linearVelocities[index] = (m2Vec2){0.0f, 0.0f};
        world->angularVelocities[index] = 0.0f;
    }
    // Fresh start for the sleep ledger under the new identity.
    world->asleep[index] = 0;
    world->sleepTimes[index] = 0.0f;
    world->sleepStreak[index] = 0;
}

bool m2Body_IsAwake(m2BodyId bodyId)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        return false;
    }
    return world->types[index] == (uint8_t)m2_dynamicBody ? world->asleep[index] == 0 : true;
}

// --- Shapes ---------------------------------------------------------------------

m2ShapeDef m2DefaultShapeDef(void)
{
    m2ShapeDef def;
    memset(&def, 0, sizeof(def));
    def.density = 1.0f;
    def.friction = 0.6f;
    def.restitution = 0.0f;
    def.categoryBits = 1;
    def.maskBits = 0xFFFFFFFFu;
    def.internalValue = M2_SHAPE_COOKIE;
    return def;
}

static m2ShapeId CreateShape(m2BodyId bodyId, const m2ShapeDef* def,
                             const m2ShapeGeometry* geometry)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t bodyIndex = world != NULL ? BodySlot(world, bodyId) : -1;
    if (bodyIndex < 0 || def == NULL || def->internalValue != M2_SHAPE_COOKIE ||
        !(def->density >= 0.0f) || !(def->friction >= 0.0f) ||
        !(def->restitution >= 0.0f && def->restitution <= 1.0f))
    {
        M2_ASSERT(false);
        return m2_nullShapeId;
    }
    if (world->shapeFreeCount == 0)
    {
        return m2_nullShapeId;
    }

    int32_t index = world->shapeFreeQueue[world->shapeFreeHead];
    world->shapeFreeHead = (world->shapeFreeHead + 1) % world->shapeCapacity;
    world->shapeFreeCount -= 1;

    // memset first: deterministic union tail bytes in the snapshot.
    memset(&world->shapeGeometry[index], 0, sizeof(m2ShapeGeometry));
    world->shapeGeometry[index] = *geometry;
    world->shapeDensity[index] = def->density;
    world->shapeFriction[index] = def->friction;
    world->shapeRestitution[index] = def->restitution;
    world->shapeTangentSpeed[index] = def->tangentSpeed;
    world->shapeUserData[index] = def->userData;
    world->shapeCategory[index] = def->categoryBits;
    world->shapeMask[index] = def->maskBits;
    world->shapeGroup[index] = def->groupIndex;
    world->shapeSensor[index] = def->isSensor ? 1 : 0;
    world->shapeChain[index] = -1;
    world->shapeBody[index] = bodyIndex;
    world->shapeNext[index] = world->bodyShapeHead[bodyIndex];
    world->bodyShapeHead[bodyIndex] = index;
    world->shapeAlive[index] = 1;
    if (index + 1 > world->maxShapeIndex)
    {
        world->maxShapeIndex = index + 1;
    }

    if (world->disabled[bodyIndex] == 0)
    {
        int32_t tree = world->types[bodyIndex];
        world->proxyIds[index] = m2Tree_Insert(&world->trees[tree], world->treeNodes[tree],
                                               Fatten(ShapeTightAABB(world, index)), index);
        if (world->proxyIds[index] == M2_NULL_NODE)
        {
            // Node pool exhausted: undo everything; capacity error, not UB.
            world->bodyShapeHead[bodyIndex] = world->shapeNext[index];
            world->shapeAlive[index] = 0;
            world->shapeFreeHead =
                (world->shapeFreeHead + world->shapeCapacity - 1) % world->shapeCapacity;
            world->shapeFreeQueue[world->shapeFreeHead] = index;
            world->shapeFreeCount += 1;
            return m2_nullShapeId;
        }
        PushMoved(world, index);
    }
    // Dormant bodies keep the shape out of the trees until Enable, but
    // EVERYTHING else (mass, journaling, the id) proceeds normally so
    // replays mint identical worlds.
    RecomputeMass(world, bodyIndex);

    m2ShapeId id = {index + 1, bodyId.world0, world->shapeGenerations[index]};

    if (world->journalActive != 0)

    {

        struct

        {

            m2BodyId body;

            m2ShapeDef def;

            m2ShapeGeometry geometry;

            m2ShapeId expected;

        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.def = *def;
        record.geometry = *geometry;
        record.expected = id;

        m2JournalRecord(world, m2_opCreateShape, &record, (int32_t)sizeof(record));
    }
    return id;
}

#define M2_SHAPE_CTOR(name, geomType, enumValue, validator, member)                                \
    m2ShapeId name(m2BodyId bodyId, const m2ShapeDef* def, const geomType* geom)                   \
    {                                                                                              \
        if (!validator(geom))                                                                      \
        {                                                                                          \
            M2_ASSERT(false);                                                                      \
            return m2_nullShapeId;                                                                 \
        }                                                                                          \
        m2ShapeGeometry geometry;                                                                  \
        memset(&geometry, 0, sizeof(geometry));                                                    \
        geometry.type = enumValue;                                                                 \
        geometry.member = *geom;                                                                   \
        return CreateShape(bodyId, def, &geometry);                                                \
    }

M2_SHAPE_CTOR(m2CreateCircleShape, m2Circle, m2_circleShape, m2ValidateCircle, circle)
M2_SHAPE_CTOR(m2CreateCapsuleShape, m2Capsule, m2_capsuleShape, m2ValidateCapsule, capsule)
M2_SHAPE_CTOR(m2CreatePolygonShape, m2Polygon, m2_polygonShape, m2ValidatePolygon, polygon)
M2_SHAPE_CTOR(m2CreateSegmentShape, m2Segment, m2_segmentShape, m2ValidateSegment, segment)

m2ChainDef m2DefaultChainDef(void)
{
    m2ChainDef def;
    memset(&def, 0, sizeof(def));
    def.friction = 0.6f;
    def.categoryBits = 1;
    def.maskBits = 0xFFFFFFFFu;
    def.internalValue = M2_CHAIN_COOKIE;
    return def;
}

m2ChainId m2CreateChain(m2BodyId bodyId, const m2ChainDef* def)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t bodyIndex = world != NULL ? BodySlot(world, bodyId) : -1;
    if (bodyIndex < 0 || def == NULL || def->internalValue != M2_CHAIN_COOKIE ||
        def->points == NULL || (def->isLoop ? def->count < 3 : def->count < 4) ||
        world->chainFreeCount == 0)
    {
        M2_ASSERT(false);
        return m2_nullChainId;
    }

    // Claim the chain slot before making shapes so each segment can be
    // tagged with its owner as it is born.
    int32_t chainIndex = world->chainFreeQueue[world->chainFreeHead];
    world->chainFreeHead = (world->chainFreeHead + 1) % world->shapeCapacity;
    world->chainFreeCount -= 1;

    // One journal op describes the whole chain; the per-shape creates
    // below must not double-record.
    uint8_t journalWasActive = world->journalActive;
    world->journalActive = 0;

    m2ShapeDef shapeDef = m2DefaultShapeDef();
    shapeDef.density = 0.0f;
    shapeDef.friction = def->friction;
    shapeDef.restitution = def->restitution;
    shapeDef.categoryBits = def->categoryBits;
    shapeDef.maskBits = def->maskBits;
    shapeDef.groupIndex = def->groupIndex;
    shapeDef.userData = def->userData;

    int32_t segmentCount = def->isLoop ? def->count : def->count - 3;
    int32_t created = 0;
    for (int32_t i = 0; i < segmentCount; ++i)
    {
        m2ShapeGeometry geometry;
        memset(&geometry, 0, sizeof(geometry));
        geometry.type = m2_chainSegmentShape;
        if (def->isLoop)
        {
            int32_t n = def->count;
            geometry.chainSegment.ghost1 = def->points[(i + n - 1) % n];
            geometry.chainSegment.segment.point1 = def->points[i];
            geometry.chainSegment.segment.point2 = def->points[(i + 1) % n];
            geometry.chainSegment.ghost2 = def->points[(i + 2) % n];
        }
        else
        {
            geometry.chainSegment.ghost1 = def->points[i];
            geometry.chainSegment.segment.point1 = def->points[i + 1];
            geometry.chainSegment.segment.point2 = def->points[i + 2];
            geometry.chainSegment.ghost2 = def->points[i + 3];
        }
        m2ShapeId shape = CreateShape(bodyId, &shapeDef, &geometry);
        if (shape.index1 == 0)
        {
            break; // capacity: partial chain, loud via the segment count
        }
        world->shapeChain[shape.index1 - 1] = chainIndex;
        created += 1;
    }

    world->journalActive = journalWasActive;
    if (created == 0)
    {
        // Nothing was made: retire the claimed slot. The generation
        // burns, which keeps the id sequence append-only either way.
        RetireChainSlot(world, chainIndex);
        return m2_nullChainId;
    }
    world->chainAlive[chainIndex] = 1;
    world->chainBody[chainIndex] = bodyIndex;
    if (chainIndex + 1 > world->maxChainIndex)
    {
        world->maxChainIndex = chainIndex + 1;
    }
    m2JournalRecordChain(world, bodyId, def, created);
    m2ChainId id = {chainIndex + 1, world->worldIndex0, world->chainGenerations[chainIndex]};
    return id;
}

static int32_t ChainSlot(const m2World* world, m2ChainId chainId)
{
    int32_t index = chainId.index1 - 1;
    if (index < 0 || index >= world->shapeCapacity || world->chainAlive[index] == 0 ||
        world->chainGenerations[index] != chainId.generation)
    {
        return -1;
    }
    return index;
}

void m2DestroyChain(m2ChainId chainId)
{
    m2World* world = WorldFromIndex(chainId.world0);
    int32_t index = world != NULL ? ChainSlot(world, chainId) : -1;
    if (index < 0)
    {
        return;
    }
    m2JournalRecord(world, m2_opDestroyChain, &chainId, (int32_t)sizeof(chainId));

    // One canonical walk over the body's insertion-ordered shape list,
    // unlinking members in place; DestroyShapeInternal ends contacts
    // and wakes whoever was resting on each segment.
    int32_t bodyIndex = world->chainBody[index];
    int32_t prev = -1;
    int32_t s = world->bodyShapeHead[bodyIndex];
    while (s != -1)
    {
        int32_t next = world->shapeNext[s];
        if (world->shapeChain[s] == index)
        {
            if (prev == -1)
            {
                world->bodyShapeHead[bodyIndex] = next;
            }
            else
            {
                world->shapeNext[prev] = next;
            }
            world->shapeNext[s] = -1;
            DestroyShapeInternal(world, s);
        }
        else
        {
            prev = s;
        }
        s = next;
    }
    RetireChainSlot(world, index);
    RecomputeMass(world, bodyIndex);
    if (world->types[bodyIndex] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[bodyIndex] = 0;
        world->sleepTimes[bodyIndex] = 0.0f;
    }
}

bool m2Chain_IsValid(m2ChainId chainId)
{
    m2World* world = WorldFromIndex(chainId.world0);
    return world != NULL && ChainSlot(world, chainId) >= 0;
}

int32_t m2Chain_GetSegmentCount(m2ChainId chainId)
{
    m2World* world = WorldFromIndex(chainId.world0);
    int32_t index = world != NULL ? ChainSlot(world, chainId) : -1;
    if (index < 0)
    {
        return 0;
    }
    int32_t count = 0;
    for (int32_t s = world->bodyShapeHead[world->chainBody[index]]; s != -1;
         s = world->shapeNext[s])
    {
        count += world->shapeChain[s] == index ? 1 : 0;
    }
    return count;
}

bool m2Shape_IsValid(m2ShapeId shapeId)
{
    m2World* world = WorldFromIndex(shapeId.world0);
    return world != NULL && ShapeSlot(world, shapeId) >= 0;
}

m2BodyId m2Shape_GetBody(m2ShapeId shapeId)
{
    m2World* world = WorldFromIndex(shapeId.world0);
    int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return m2_nullBodyId;
    }
    int32_t bodyIndex = world->shapeBody[index];
    m2BodyId id = {bodyIndex + 1, shapeId.world0, world->generations[bodyIndex]};
    return id;
}

uint64_t m2Shape_GetUserData(m2ShapeId shapeId)
{
    m2World* world = WorldFromIndex(shapeId.world0);
    int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return 0;
    }
    return world->shapeUserData[index];
}

m2ContactEvents m2World_GetContactEvents(m2WorldId worldId)
{
    m2ContactEvents events = {NULL, NULL, 0, 0};
    m2World* world = GetWorld(worldId);
    if (world == NULL)
    {
        return events;
    }
    events.beginEvents = world->beginEvents;
    events.beginCount = world->beginEventCount;
    events.endEvents = world->endEvents;
    events.endCount = world->endEventCount;
    return events;
}

m2SensorEvents m2World_GetSensorEvents(m2WorldId worldId)
{
    m2SensorEvents events = {NULL, NULL, 0, 0};
    m2World* world = GetWorld(worldId);
    if (world == NULL)
    {
        return events;
    }
    events.beginEvents = world->sensorBeginEvents;
    events.beginCount = world->sensorBeginCount;
    events.endEvents = world->sensorEndEvents;
    events.endCount = world->sensorEndCount;
    return events;
}

m2JointEvents m2World_GetJointEvents(m2WorldId worldId)
{
    m2JointEvents events = {NULL, 0};
    m2World* world = GetWorld(worldId);
    if (world == NULL)
    {
        return events;
    }
    events.breakEvents = world->jointBreakEvents;
    events.breakCount = world->jointBreakEventCount;
    return events;
}

int32_t m2Shape_GetSensorOverlaps(m2ShapeId sensorShapeId, m2ShapeId* overlaps, int32_t capacity)
{
    m2World* world = WorldFromIndex(sensorShapeId.world0);
    if (world == NULL)
    {
        return 0;
    }
    int32_t sensor = sensorShapeId.index1 - 1;
    if (sensor < 0 || sensor >= world->shapeCapacity || world->shapeAlive[sensor] == 0 ||
        world->shapeGenerations[sensor] != sensorShapeId.generation ||
        world->shapeSensor[sensor] == 0)
    {
        return 0;
    }
    int32_t total = 0;
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        if (world->pairTouching[i] == 0)
        {
            continue;
        }
        int32_t a = (int32_t)(world->pairKeys[i] >> 32);
        int32_t b = (int32_t)(world->pairKeys[i] & 0xFFFFFFFFu);
        if (a != sensor && b != sensor)
        {
            continue;
        }
        if (total < capacity)
        {
            overlaps[total] = MakeShapeId(world, a == sensor ? b : a);
        }
        total += 1;
    }
    return total;
}

int32_t m2World_GetContactData(m2WorldId worldId, m2ContactData* data, int32_t capacity)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL)
    {
        return 0;
    }
    int32_t total = 0;
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        if (world->pairTouching[i] == 0)
        {
            continue;
        }
        {
            int32_t sa = (int32_t)(world->pairKeys[i] >> 32);
            int32_t sb = (int32_t)(world->pairKeys[i] & 0xFFFFFFFFu);
            if (world->shapeSensor[sa] != 0 || world->shapeSensor[sb] != 0)
            {
                continue; // sensors carry no physical contact
            }
        }
        if (total < capacity)
        {
            int32_t shapeA = (int32_t)(world->pairKeys[i] >> 32);
            int32_t shapeB = (int32_t)(world->pairKeys[i] & 0xFFFFFFFFu);
            m2Manifold* manifold = &world->manifolds[i];
            m2ContactData* out = data + total;
            out->shapeIdA = MakeShapeId(world, shapeA);
            out->shapeIdB = MakeShapeId(world, shapeB);
            m2Rot qA = world->transforms[world->shapeBody[shapeA]].q;
            out->normal = (m2Vec2){qA.c * manifold->normal.x - qA.s * manifold->normal.y,
                                   qA.s * manifold->normal.x + qA.c * manifold->normal.y};
            out->pointCount = manifold->pointCount;
            for (int32_t k = 0; k < 2; ++k)
            {
                bool live = k < manifold->pointCount;
                out->separations[k] = live ? manifold->points[k].separation : 0.0f;
                out->normalImpulses[k] = live ? manifold->points[k].normalImpulse : 0.0f;
                out->tangentImpulses[k] = live ? manifold->points[k].tangentImpulse : 0.0f;
            }
        }
        total += 1;
    }
    return total;
}

m2Profile m2World_GetProfile(m2WorldId worldId)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL)
    {
        m2Profile zero = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        return zero;
    }
    return world->profile;
}

double m2World_GetKineticEnergy(m2WorldId worldId)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL)
    {
        return 0.0;
    }
    double energy = 0.0;
    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        if (world->alive[i] == 0 || world->types[i] != (uint8_t)m2_dynamicBody ||
            world->invMass[i] == 0.0f)
        {
            continue;
        }
        double vx = (double)world->linearVelocities[i].x;
        double vy = (double)world->linearVelocities[i].y;
        energy += 0.5 * (1.0 / (double)world->invMass[i]) * (vx * vx + vy * vy);
        if (world->invInertia[i] > 0.0f)
        {
            double w = (double)world->angularVelocities[i];
            energy += 0.5 * (1.0 / (double)world->invInertia[i]) * w * w;
        }
    }
    return energy;
}

void m2World_SetGravity(m2WorldId worldId, m2Vec2 gravity)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL)
    {
        M2_ASSERT(false);
        return;
    }
    if (world->gravity.x == gravity.x && world->gravity.y == gravity.y)
    {
        return; // no-op, not journaled
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2Vec2 gravity;
        } record;
        memset(&record, 0, sizeof(record));
        record.gravity = gravity;
        m2JournalRecord(world, m2_opSetGravity, &record, (int32_t)sizeof(record));
    }
    world->gravity = gravity;
    // Honesty over precedent: a sleeping stack must feel the new
    // world. Wake every dynamic sleeper (deterministic, one pass).
    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        if (world->alive[i] != 0 && world->types[i] == (uint8_t)m2_dynamicBody &&
            world->asleep[i] != 0)
        {
            world->asleep[i] = 0;
            world->sleepTimes[i] = 0.0f;
        }
    }
}

m2Vec2 m2World_GetGravity(m2WorldId worldId)
{
    m2World* world = GetWorld(worldId);
    return world != NULL ? world->gravity : (m2Vec2){0.0f, 0.0f};
}

void m2World_SetWind(m2WorldId worldId, m2Vec2 velocity, float linearDrag)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL || !(linearDrag >= 0.0f) || !(velocity.x == velocity.x) ||
        !(velocity.y == velocity.y))
    {
        M2_ASSERT(false);
        return;
    }
    if (world->windVelocity.x == velocity.x && world->windVelocity.y == velocity.y &&
        world->windLinearDrag == linearDrag)
    {
        return; // no-op, not journaled
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2Vec2 velocity;
            float linearDrag;
        } record;
        memset(&record, 0, sizeof(record));
        record.velocity = velocity;
        record.linearDrag = linearDrag;
        m2JournalRecord(world, m2_opSetWind, &record, (int32_t)sizeof(record));
    }
    world->windVelocity = velocity;
    world->windLinearDrag = linearDrag;
    // Deliberate deviation from SetGravity, which wakes every sleeper:
    // wind is expected to change often (gusts), so waking all sleepers
    // on each change would defeat sleeping. Asleep bodies are frozen and
    // deterministically skip the wind pass, exactly as they skip gravity
    // integration; a settled pile stays settled until roused otherwise.
}

void m2World_GetWind(m2WorldId worldId, m2Vec2* velocity, float* linearDrag)
{
    m2World* world = GetWorld(worldId);
    if (velocity != NULL)
    {
        *velocity = world != NULL ? world->windVelocity : (m2Vec2){0.0f, 0.0f};
    }
    if (linearDrag != NULL)
    {
        *linearDrag = world != NULL ? world->windLinearDrag : 0.0f;
    }
}

static int32_t ShapeSlotChecked(m2ShapeId shapeId, m2World** outWorld)
{
    m2World* world = WorldFromIndex(shapeId.world0);
    *outWorld = world;
    if (world == NULL)
    {
        return -1;
    }
    int32_t index = shapeId.index1 - 1;
    if (index < 0 || index >= world->shapeCapacity || world->shapeAlive[index] == 0 ||
        world->shapeGenerations[index] != shapeId.generation)
    {
        return -1;
    }
    return index;
}

// One journaled channel for shape materials (op 22).
void m2SetShapeParamInternal(m2World* world, m2ShapeId shapeId, uint8_t param, float value)
{
    int32_t index = shapeId.index1 - 1;
    if (index < 0 || index >= world->shapeCapacity || world->shapeAlive[index] == 0 ||
        world->shapeGenerations[index] != shapeId.generation)
    {
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2ShapeId shape;
            float value;
            uint8_t param;
        } record;
        memset(&record, 0, sizeof(record));
        record.shape = shapeId;
        record.value = value;
        record.param = param;
        m2JournalRecord(world, m2_opShapeParam, &record, (int32_t)sizeof(record));
    }
    if (param == 0)
    {
        world->shapeFriction[index] = value;
    }
    else if (param == 2)
    {
        world->shapeTangentSpeed[index] = value;
        // A belt that changes speed must wake its riders, and the wake must
        // live HERE, inside the journaled channel, so a replay reproduces
        // it exactly. When it lived only in the public wrapper the replay
        // set the speed but left a sleeping rider asleep, and the recorded
        // and replayed worlds diverged (a fuzz seed caught this once the
        // velocity cap let it run far enough to reach the replay check).
        int32_t body = world->shapeBody[index];
        for (int32_t i = 0; i < world->pairCount; ++i)
        {
            int32_t a = (int32_t)(world->pairKeys[i] >> 32);
            int32_t b = (int32_t)(world->pairKeys[i] & 0xFFFFFFFFu);
            if (a != index && b != index)
            {
                continue;
            }
            int32_t otherBody = world->shapeBody[a == index ? b : a];
            if (world->types[otherBody] == (uint8_t)m2_dynamicBody)
            {
                world->asleep[otherBody] = 0;
                world->sleepTimes[otherBody] = 0.0f;
            }
        }
        if (world->types[body] == (uint8_t)m2_dynamicBody)
        {
            world->asleep[body] = 0;
            world->sleepTimes[body] = 0.0f;
        }
    }
    else
    {
        world->shapeRestitution[index] = value;
    }
}

void m2Shape_SetTangentSpeed(m2ShapeId shapeId, float speed)
{
    m2World* world = WorldFromIndex(shapeId.world0);
    if (world != NULL && speed == speed)
    {
        // The wake now rides inside the journaled channel, so the live call
        // and its replay leave identical sleep state.
        m2SetShapeParamInternal(world, shapeId, 2, speed);
    }
}

float m2Shape_GetTangentSpeed(m2ShapeId shapeId)
{
    m2World* world = WorldFromIndex(shapeId.world0);
    int32_t index = shapeId.index1 - 1;
    if (world == NULL || index < 0 || index >= world->shapeCapacity ||
        world->shapeAlive[index] == 0 || world->shapeGenerations[index] != shapeId.generation)
    {
        return 0.0f;
    }
    return world->shapeTangentSpeed[index];
}

void m2Shape_SetFriction(m2ShapeId shapeId, float friction)
{
    m2World* world = WorldFromIndex(shapeId.world0);
    if (world != NULL && friction >= 0.0f)
    {
        m2SetShapeParamInternal(world, shapeId, 0, friction);
    }
}

void m2Shape_SetRestitution(m2ShapeId shapeId, float restitution)
{
    m2World* world = WorldFromIndex(shapeId.world0);
    if (world != NULL && restitution >= 0.0f && restitution <= 1.0f)
    {
        m2SetShapeParamInternal(world, shapeId, 1, restitution);
    }
}

float m2Shape_GetFriction(m2ShapeId shapeId)
{
    m2World* world = NULL;
    int32_t index = ShapeSlotChecked(shapeId, &world);
    return index >= 0 ? world->shapeFriction[index] : 0.0f;
}

float m2Shape_GetRestitution(m2ShapeId shapeId)
{
    m2World* world = NULL;
    int32_t index = ShapeSlotChecked(shapeId, &world);
    return index >= 0 ? world->shapeRestitution[index] : 0.0f;
}

void m2Shape_SetFilter(m2ShapeId shapeId, uint32_t categoryBits, uint32_t maskBits,
                       int32_t groupIndex)
{
    m2World* world = NULL;
    int32_t index = ShapeSlotChecked(shapeId, &world);
    if (index < 0)
    {
        M2_ASSERT(false);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2ShapeId shape;
            uint32_t categoryBits;
            uint32_t maskBits;
            int32_t groupIndex;
        } record;
        memset(&record, 0, sizeof(record));
        record.shape = shapeId;
        record.categoryBits = categoryBits;
        record.maskBits = maskBits;
        record.groupIndex = groupIndex;
        m2JournalRecord(world, m2_opSetFilter, &record, (int32_t)sizeof(record));
    }

    // Whoever this shape was touching must notice its allegiance
    // change, exactly like a teleport or a type flip.
    int32_t body = world->shapeBody[index];
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        if (world->pairTouching[i] == 0)
        {
            continue;
        }
        int32_t sa = (int32_t)(world->pairKeys[i] >> 32);
        int32_t sb = (int32_t)(world->pairKeys[i] & 0xFFFFFFFFu);
        if (sa != index && sb != index)
        {
            continue;
        }
        int32_t other = world->shapeBody[sa == index ? sb : sa];
        if (world->types[other] == (uint8_t)m2_dynamicBody)
        {
            world->asleep[other] = 0;
            world->sleepTimes[other] = 0.0f;
        }
    }
    if (world->types[body] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[body] = 0;
        world->sleepTimes[body] = 0.0f;
    }

    world->shapeCategory[index] = categoryBits;
    world->shapeMask[index] = maskBits;
    world->shapeGroup[index] = groupIndex;
    PushMoved(world, index); // pair rebuild purges and re-collects (M19)
}

// One shared road for runtime geometry: validate outside, then swap
// the union (memset first: deterministic tail bytes), wake whoever
// was touching it, refresh mass and broadphase.
static void SetGeometryInternal(m2World* world, m2ShapeId shapeId, int32_t index,
                                const m2ShapeGeometry* geometry)
{
    if (world->journalActive != 0)
    {
        struct
        {
            m2ShapeId shape;
            m2ShapeGeometry geometry;
        } record;
        memset(&record, 0, sizeof(record));
        record.shape = shapeId;
        record.geometry = *geometry;
        m2JournalRecord(world, m2_opSetGeometry, &record, (int32_t)sizeof(record));
    }
    int32_t body = world->shapeBody[index];
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        if (world->pairTouching[i] == 0)
        {
            continue;
        }
        int32_t sa = (int32_t)(world->pairKeys[i] >> 32);
        int32_t sb = (int32_t)(world->pairKeys[i] & 0xFFFFFFFFu);
        if (sa != index && sb != index)
        {
            continue;
        }
        int32_t other = world->shapeBody[sa == index ? sb : sa];
        if (world->types[other] == (uint8_t)m2_dynamicBody)
        {
            world->asleep[other] = 0;
            world->sleepTimes[other] = 0.0f;
        }
    }
    memset(&world->shapeGeometry[index], 0, sizeof(m2ShapeGeometry));
    world->shapeGeometry[index] = *geometry;
    RecomputeMass(world, body);
    if (world->types[body] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[body] = 0;
        world->sleepTimes[body] = 0.0f;
    }
    if (world->proxyIds[index] != M2_NULL_NODE)
    {
        PushMoved(world, index);
    }
}

void m2Shape_SetCircle(m2ShapeId shapeId, const m2Circle* circle)
{
    m2World* world = NULL;
    int32_t index = ShapeSlotChecked(shapeId, &world);
    if (index < 0 || circle == NULL || !m2ValidateCircle(circle) ||
        world->shapeGeometry[index].type == (int32_t)m2_chainSegmentShape)
    {
        M2_ASSERT(false);
        return;
    }
    m2ShapeGeometry g;
    memset(&g, 0, sizeof(g));
    g.type = m2_circleShape;
    g.circle = *circle;
    SetGeometryInternal(world, shapeId, index, &g);
}

void m2Shape_SetCapsule(m2ShapeId shapeId, const m2Capsule* capsule)
{
    m2World* world = NULL;
    int32_t index = ShapeSlotChecked(shapeId, &world);
    if (index < 0 || capsule == NULL || !m2ValidateCapsule(capsule) ||
        world->shapeGeometry[index].type == (int32_t)m2_chainSegmentShape)
    {
        M2_ASSERT(false);
        return;
    }
    m2ShapeGeometry g;
    memset(&g, 0, sizeof(g));
    g.type = m2_capsuleShape;
    g.capsule = *capsule;
    SetGeometryInternal(world, shapeId, index, &g);
}

void m2Shape_SetPolygon(m2ShapeId shapeId, const m2Polygon* polygon)
{
    m2World* world = NULL;
    int32_t index = ShapeSlotChecked(shapeId, &world);
    if (index < 0 || polygon == NULL || !m2ValidatePolygon(polygon) ||
        world->shapeGeometry[index].type == (int32_t)m2_chainSegmentShape)
    {
        M2_ASSERT(false);
        return;
    }
    m2ShapeGeometry g;
    memset(&g, 0, sizeof(g));
    g.type = m2_polygonShape;
    g.polygon = *polygon;
    SetGeometryInternal(world, shapeId, index, &g);
}

void m2Shape_SetSegment(m2ShapeId shapeId, const m2Segment* segment)
{
    m2World* world = NULL;
    int32_t index = ShapeSlotChecked(shapeId, &world);
    if (index < 0 || segment == NULL || !m2ValidateSegment(segment) ||
        world->shapeGeometry[index].type == (int32_t)m2_chainSegmentShape)
    {
        M2_ASSERT(false);
        return;
    }
    m2ShapeGeometry g;
    memset(&g, 0, sizeof(g));
    g.type = m2_segmentShape;
    g.segment = *segment;
    SetGeometryInternal(world, shapeId, index, &g);
}

static void ChainMaterialInternal(m2World* world, m2ChainId chainId, int32_t chainIndex, uint8_t op,
                                  float value)
{
    if (world->journalActive != 0)
    {
        struct
        {
            m2ChainId chain;
            float value;
        } record;
        memset(&record, 0, sizeof(record));
        record.chain = chainId;
        record.value = value;
        m2JournalRecord(world, op, &record, (int32_t)sizeof(record));
    }
    int32_t body = world->chainBody[chainIndex];
    for (int32_t s = world->bodyShapeHead[body]; s != -1; s = world->shapeNext[s])
    {
        if (world->shapeChain[s] != chainIndex)
        {
            continue;
        }
        if (op == m2_opChainFriction)
        {
            world->shapeFriction[s] = value;
        }
        else
        {
            world->shapeRestitution[s] = value;
        }
    }
}

void m2Chain_SetFriction(m2ChainId chainId, float friction)
{
    m2World* world = WorldFromIndex(chainId.world0);
    int32_t index = world != NULL ? ChainSlot(world, chainId) : -1;
    if (index < 0 || !(friction >= 0.0f))
    {
        M2_ASSERT(false);
        return;
    }
    ChainMaterialInternal(world, chainId, index, m2_opChainFriction, friction);
}

void m2Chain_SetRestitution(m2ChainId chainId, float restitution)
{
    m2World* world = WorldFromIndex(chainId.world0);
    int32_t index = world != NULL ? ChainSlot(world, chainId) : -1;
    if (index < 0 || !(restitution >= 0.0f))
    {
        M2_ASSERT(false);
        return;
    }
    ChainMaterialInternal(world, chainId, index, m2_opChainRestitution, restitution);
}

m2Pos2 m2Body_GetWorldPoint(m2BodyId bodyId, m2Vec2 localPoint)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return (m2Pos2){0.0, 0.0};
    }
    m2Transform xf = world->transforms[index];
    m2Vec2 r = {xf.q.c * localPoint.x - xf.q.s * localPoint.y,
                xf.q.s * localPoint.x + xf.q.c * localPoint.y};
    return (m2Pos2){xf.p.x + (double)r.x, xf.p.y + (double)r.y};
}

m2Vec2 m2Body_GetLocalPoint(m2BodyId bodyId, m2Pos2 worldPoint)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return (m2Vec2){0.0f, 0.0f};
    }
    m2Transform xf = world->transforms[index];
    m2Vec2 rel = {(float)(worldPoint.x - xf.p.x), (float)(worldPoint.y - xf.p.y)};
    return (m2Vec2){xf.q.c * rel.x + xf.q.s * rel.y, -xf.q.s * rel.x + xf.q.c * rel.y};
}

m2Vec2 m2Body_GetWorldVector(m2BodyId bodyId, m2Vec2 localVector)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return (m2Vec2){0.0f, 0.0f};
    }
    m2Rot q = world->transforms[index].q;
    return (m2Vec2){q.c * localVector.x - q.s * localVector.y,
                    q.s * localVector.x + q.c * localVector.y};
}

m2Vec2 m2Body_GetLocalVector(m2BodyId bodyId, m2Vec2 worldVector)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return (m2Vec2){0.0f, 0.0f};
    }
    m2Rot q = world->transforms[index].q;
    return (m2Vec2){q.c * worldVector.x + q.s * worldVector.y,
                    -q.s * worldVector.x + q.c * worldVector.y};
}

m2Vec2 m2Body_GetWorldPointVelocity(m2BodyId bodyId, m2Pos2 worldPoint)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return (m2Vec2){0.0f, 0.0f};
    }
    // v + w x r, arm from the center of mass (one f64 crossing).
    m2Transform xf = world->transforms[index];
    m2Vec2 rlc = {xf.q.c * world->localCenters[index].x - xf.q.s * world->localCenters[index].y,
                  xf.q.s * world->localCenters[index].x + xf.q.c * world->localCenters[index].y};
    m2Vec2 r = {(float)(worldPoint.x - xf.p.x) - rlc.x, (float)(worldPoint.y - xf.p.y) - rlc.y};
    float w = world->angularVelocities[index];
    m2Vec2 v = world->linearVelocities[index];
    return (m2Vec2){v.x - w * r.y, v.y + w * r.x};
}

int32_t m2Body_GetJoints(m2BodyId bodyId, m2JointId* ids, int32_t capacity)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        return 0;
    }
    int32_t total = 0;
    for (int32_t j = 0; j < world->maxJointIndex; ++j)
    {
        if (world->jointAlive[j] == 0 ||
            (world->jointBodyA[j] != index && world->jointBodyB[j] != index))
        {
            continue;
        }
        if (ids != NULL && total < capacity)
        {
            m2JointId id = {j + 1, world->worldIndex0, world->jointGenerations[j]};
            ids[total] = id;
        }
        total += 1;
    }
    return total;
}

void m2Body_ApplyLinearImpulseToCenter(m2BodyId bodyId, m2Vec2 impulse)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0 || world->types[index] != (uint8_t)m2_dynamicBody)
    {
        M2_ASSERT(false);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2BodyId body;
            m2Vec2 impulse;
        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.impulse = impulse;
        m2JournalRecord(world, m2_opImpulseCenter, &record, (int32_t)sizeof(record));
    }
    world->linearVelocities[index].x += world->invMass[index] * impulse.x;
    world->linearVelocities[index].y += world->invMass[index] * impulse.y;
    world->asleep[index] = 0;
    world->sleepTimes[index] = 0.0f;
}

void m2Body_SetAwake(m2BodyId bodyId, bool awake)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0 || world->types[index] != (uint8_t)m2_dynamicBody || world->disabled[index] != 0)
    {
        return;
    }
    uint8_t sleeping = awake ? 0 : 1;
    if (world->asleep[index] == sleeping)
    {
        return; // no-op stays unjournaled
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2BodyId body;
            uint8_t awake;
        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.awake = awake ? 1 : 0;
        m2JournalRecord(world, m2_opSetAwake, &record, (int32_t)sizeof(record));
    }
    world->asleep[index] = sleeping;
    world->sleepTimes[index] = 0.0f;
    if (!awake)
    {
        // Forced sleep stills the body, reference-style.
        world->linearVelocities[index] = (m2Vec2){0.0f, 0.0f};
        world->angularVelocities[index] = 0.0f;
        world->forces[index] = (m2Vec2){0.0f, 0.0f};
        world->torques[index] = 0.0f;
    }
}

void m2Body_SetBullet(m2BodyId bodyId, bool flag)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return;
    }
    uint8_t next = flag ? 1 : 0;
    if (world->bullets[index] == next)
    {
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2BodyId body;
            uint8_t flag;
        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.flag = next;
        m2JournalRecord(world, m2_opSetBullet, &record, (int32_t)sizeof(record));
    }
    world->bullets[index] = next;
}

void m2Body_SetUserData(m2BodyId bodyId, uint64_t userData)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2BodyId body;
            uint64_t userData;
        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.userData = userData;
        m2JournalRecord(world, m2_opBodyUserData, &record, (int32_t)sizeof(record));
    }
    world->userData[index] = userData;
}

void m2Body_SetTargetTransform(m2BodyId bodyId, m2Pos2 position, m2Rot rotation, float dt)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0 || !(dt > 0.0f))
    {
        M2_ASSERT(false);
        return;
    }
    // Velocities that land the pose in one step; applied through the
    // journaled setters, so replays get this for free.
    float invDt = 1.0f / dt;
    m2Transform xf = world->transforms[index];
    m2Vec2 v = {(float)(position.x - xf.p.x) * invDt, (float)(position.y - xf.p.y) * invDt};
    float w = m2UnwindAngle(RelativeJointAngle(xf.q, rotation)) * invDt;
    m2Body_SetLinearVelocity(bodyId, v);
    m2Body_SetAngularVelocity(bodyId, w);
}

m2Vec2 m2Body_GetLocalPointVelocity(m2BodyId bodyId, m2Vec2 localPoint)
{
    return m2Body_GetWorldPointVelocity(bodyId, m2Body_GetWorldPoint(bodyId, localPoint));
}

m2Pos2 m2Body_GetWorldCenterOfMass(m2BodyId bodyId)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return (m2Pos2){0.0, 0.0};
    }
    return m2Body_GetWorldPoint(bodyId, world->localCenters[index]);
}

float m2Body_GetRotationalInertia(m2BodyId bodyId)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return 0.0f;
    }
    float invI = world->invInertia[index];
    return invI > 0.0f ? 1.0f / invI : 0.0f;
}

m2World* m2WorldFromIndex0(uint16_t index0)
{
    return WorldFromIndex(index0);
}

m2WorldId m2Body_GetWorld(m2BodyId bodyId)
{
    m2World* world = GetBodyWorld(bodyId);
    m2WorldId id = {0, 0};
    if (world == NULL)
    {
        M2_ASSERT(false);
        return id;
    }
    id.index1 = world->worldIndex0;
    id.generation = world->worldGeneration;
    return id;
}

m2AABBResult m2Body_ComputeAABB(m2BodyId bodyId)
{
    m2AABBResult result = {{0.0, 0.0}, {0.0, 0.0}};
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return result;
    }
    result.lowerBound = world->transforms[index].p;
    result.upperBound = world->transforms[index].p;
    bool first = true;
    for (int32_t s = world->bodyShapeHead[index]; s != -1; s = world->shapeNext[s])
    {
        m2AABB tight = m2ComputeShapeAABB(&world->shapeGeometry[s], world->transforms[index]);
        if (first)
        {
            result.lowerBound = tight.lowerBound;
            result.upperBound = tight.upperBound;
            first = false;
            continue;
        }
        result.lowerBound.x =
            tight.lowerBound.x < result.lowerBound.x ? tight.lowerBound.x : result.lowerBound.x;
        result.lowerBound.y =
            tight.lowerBound.y < result.lowerBound.y ? tight.lowerBound.y : result.lowerBound.y;
        result.upperBound.x =
            tight.upperBound.x > result.upperBound.x ? tight.upperBound.x : result.upperBound.x;
        result.upperBound.y =
            tight.upperBound.y > result.upperBound.y ? tight.upperBound.y : result.upperBound.y;
    }
    return result;
}

bool m2Shape_TestPoint(m2ShapeId shapeId, m2Pos2 point)
{
    m2World* world = WorldFromIndex(shapeId.world0);
    int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return false;
    }
    int32_t body = world->shapeBody[index];
    m2Transform xf = world->transforms[body];
    m2Vec2 rel = {(float)(point.x - xf.p.x), (float)(point.y - xf.p.y)};
    m2Vec2 local = {xf.q.c * rel.x + xf.q.s * rel.y, -xf.q.s * rel.x + xf.q.c * rel.y};
    m2DistanceProxy target = m2GeometryProxy(&world->shapeGeometry[index]);
    m2DistanceProxy probe;
    probe.points[0] = local;
    probe.count = 1;
    probe.radius = 0.0f;
    m2DistanceResult d = m2ShapeDistance(&target, &probe);
    // Touching within the slop skin counts (the overlap law).
    return d.distance - target.radius <= 0.005f;
}

m2Pos2 m2Shape_GetClosestPoint(m2ShapeId shapeId, m2Pos2 point)
{
    m2World* world = WorldFromIndex(shapeId.world0);
    int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return (m2Pos2){0.0, 0.0};
    }
    int32_t body = world->shapeBody[index];
    m2Transform xf = world->transforms[body];
    m2Vec2 rel = {(float)(point.x - xf.p.x), (float)(point.y - xf.p.y)};
    m2Vec2 local = {xf.q.c * rel.x + xf.q.s * rel.y, -xf.q.s * rel.x + xf.q.c * rel.y};
    m2DistanceProxy target = m2GeometryProxy(&world->shapeGeometry[index]);
    m2DistanceProxy probe;
    probe.points[0] = local;
    probe.count = 1;
    probe.radius = 0.0f;
    m2DistanceResult d = m2ShapeDistance(&target, &probe);
    if (d.distance - target.radius <= 0.0f)
    {
        return point; // inside: the query point is its own closest
    }
    m2Vec2 surf = {d.pointA.x + target.radius * d.normal.x,
                   d.pointA.y + target.radius * d.normal.y};
    m2Vec2 out = {xf.q.c * surf.x - xf.q.s * surf.y, xf.q.s * surf.x + xf.q.c * surf.y};
    return (m2Pos2){xf.p.x + (double)out.x, xf.p.y + (double)out.y};
}

m2WorldId m2Shape_GetWorld(m2ShapeId shapeId)
{
    m2World* world = WorldFromIndex(shapeId.world0);
    m2WorldId id = {0, 0};
    if (world == NULL)
    {
        M2_ASSERT(false);
        return id;
    }
    id.index1 = world->worldIndex0;
    id.generation = world->worldGeneration;
    return id;
}

m2ChainId m2Shape_GetParentChain(m2ShapeId shapeId)
{
    m2World* world = WorldFromIndex(shapeId.world0);
    int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;
    m2ChainId id = {0, 0, 0};
    if (index < 0)
    {
        M2_ASSERT(false);
        return id;
    }
    int32_t chain = world->shapeChain[index];
    if (chain < 0)
    {
        return id;
    }
    id.index1 = chain + 1;
    id.world0 = world->worldIndex0;
    id.generation = world->chainGenerations[chain];
    return id;
}

m2AABBResult m2Shape_GetAABB(m2ShapeId shapeId)
{
    m2AABBResult result = {{0.0, 0.0}, {0.0, 0.0}};
    m2World* world = WorldFromIndex(shapeId.world0);
    int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return result;
    }
    m2AABB tight = m2ComputeShapeAABB(&world->shapeGeometry[index],
                                      world->transforms[world->shapeBody[index]]);
    result.lowerBound = tight.lowerBound;
    result.upperBound = tight.upperBound;
    return result;
}

void m2Shape_SetDensity(m2ShapeId shapeId, float density)
{
    m2World* world = NULL;
    int32_t index = ShapeSlotChecked(shapeId, &world);
    if (index < 0 || !(density >= 0.0f))
    {
        M2_ASSERT(false);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2ShapeId shape;
            float density;
        } record;
        memset(&record, 0, sizeof(record));
        record.shape = shapeId;
        record.density = density;
        m2JournalRecord(world, m2_opSetDensity, &record, (int32_t)sizeof(record));
    }
    world->shapeDensity[index] = density;
    int32_t body = world->shapeBody[index];
    RecomputeMass(world, body);
    if (world->types[body] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[body] = 0;
        world->sleepTimes[body] = 0.0f;
    }
}

void m2Shape_SetUserData(m2ShapeId shapeId, uint64_t userData)
{
    m2World* world = NULL;
    int32_t index = ShapeSlotChecked(shapeId, &world);
    if (index < 0)
    {
        M2_ASSERT(false);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2ShapeId shape;
            uint64_t userData;
        } record;
        memset(&record, 0, sizeof(record));
        record.shape = shapeId;
        record.userData = userData;
        m2JournalRecord(world, m2_opShapeUserData, &record, (int32_t)sizeof(record));
    }
    world->shapeUserData[index] = userData;
}

m2WorldId m2Chain_GetWorld(m2ChainId chainId)
{
    m2World* world = WorldFromIndex(chainId.world0);
    m2WorldId id = {0, 0};
    if (world == NULL)
    {
        M2_ASSERT(false);
        return id;
    }
    id.index1 = world->worldIndex0;
    id.generation = world->worldGeneration;
    return id;
}

uint64_t m2Joint_GetUserData(m2JointId jointId)
{
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t index = jointId.index1 - 1;
    if (world == NULL || index < 0 || index >= world->jointCapacity ||
        world->jointAlive[index] == 0 || world->jointGenerations[index] != jointId.generation)
    {
        M2_ASSERT(false);
        return 0;
    }
    return world->jointUserData[index];
}

void m2Joint_SetUserData(m2JointId jointId, uint64_t userData)
{
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t index = jointId.index1 - 1;
    if (world == NULL || index < 0 || index >= world->jointCapacity ||
        world->jointAlive[index] == 0 || world->jointGenerations[index] != jointId.generation)
    {
        M2_ASSERT(false);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2JointId joint;
            uint64_t userData;
        } record;
        memset(&record, 0, sizeof(record));
        record.joint = jointId;
        record.userData = userData;
        m2JournalRecord(world, m2_opJointUserData, &record, (int32_t)sizeof(record));
    }
    world->jointUserData[index] = userData;
}

m2WorldId m2Joint_GetWorld(m2JointId jointId)
{
    m2World* world = WorldFromIndex(jointId.world0);
    m2WorldId id = {0, 0};
    if (world == NULL)
    {
        M2_ASSERT(false);
        return id;
    }
    id.index1 = world->worldIndex0;
    id.generation = world->worldGeneration;
    return id;
}

float m2Joint_GetLinearSeparation(m2JointId jointId)
{
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t j = jointId.index1 - 1;
    if (world == NULL || j < 0 || j >= world->jointCapacity || world->jointAlive[j] == 0 ||
        world->jointGenerations[j] != jointId.generation)
    {
        M2_ASSERT(false);
        return 0.0f;
    }
    int32_t bodyA = world->jointBodyA[j];
    int32_t bodyB = world->jointBodyB[j];
    m2Transform xfA = world->transforms[bodyA];
    m2Transform xfB = world->transforms[bodyB];
    m2Vec2 aA = world->jointLocalAnchorA[j];
    m2Vec2 aB = world->jointLocalAnchorB[j];
    m2Vec2 wA = {xfA.q.c * aA.x - xfA.q.s * aA.y, xfA.q.s * aA.x + xfA.q.c * aA.y};
    m2Vec2 wB = {xfB.q.c * aB.x - xfB.q.s * aB.y, xfB.q.s * aB.x + xfB.q.c * aB.y};
    float dx = (float)(xfB.p.x - xfA.p.x) + wB.x - wA.x;
    float dy = (float)(xfB.p.y - xfA.p.y) + wB.y - wA.y;
    switch (world->jointType[j])
    {
    case 0: // distance: length error along the rod
        return m2AbsF(sqrtf(dx * dx + dy * dy) - world->jointLength[j]);
    case 2: // prismatic: the off-axis gap
    case 4: // wheel: same slider geometry
    {
        m2Vec2 axis = world->jointLocalAxisA[j];
        m2Vec2 worldAxis = {xfA.q.c * axis.x - xfA.q.s * axis.y,
                            xfA.q.s * axis.x + xfA.q.c * axis.y};
        float perp = dx * -worldAxis.y + dy * worldAxis.x;
        return m2AbsF(perp);
    }
    case 5: // filter: pins nothing
        return 0.0f;
    case 6: // motor: distance from the commanded offset
    {
        m2Vec2 off = world->jointLocalAxisA[j];
        m2Vec2 worldOff = {xfA.q.c * off.x - xfA.q.s * off.y, xfA.q.s * off.x + xfA.q.c * off.y};
        float ex = dx - worldOff.x;
        float ey = dy - worldOff.y;
        return sqrtf(ex * ex + ey * ey);
    }
    case 7: // mouse: gap between grab point and target
    {
        m2Pos2 grab = m2Body_GetWorldPoint(
            (m2BodyId){bodyB + 1, jointId.world0, world->generations[bodyB]}, aB);
        float gx = (float)(grab.x - world->jointTargets[j].x);
        float gy = (float)(grab.y - world->jointTargets[j].y);
        return sqrtf(gx * gx + gy * gy);
    }
    default: // revolute, weld: the pinned point's gap
        return sqrtf(dx * dx + dy * dy);
    }
}

float m2Joint_GetAngularSeparation(m2JointId jointId)
{
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t j = jointId.index1 - 1;
    if (world == NULL || j < 0 || j >= world->jointCapacity || world->jointAlive[j] == 0 ||
        world->jointGenerations[j] != jointId.generation)
    {
        M2_ASSERT(false);
        return 0.0f;
    }
    uint8_t type = world->jointType[j];
    if (type != 3 && type != 6 && type != 2)
    {
        return 0.0f; // no angle is pinned
    }
    m2Rot qA = world->transforms[world->jointBodyA[j]].q;
    m2Rot qB = world->transforms[world->jointBodyB[j]].q;
    return m2AbsF(m2UnwindAngle(RelativeJointAngle(qA, qB) - world->jointRefAngle[j]));
}

m2Counters m2World_GetCounters(m2WorldId worldId)
{
    m2Counters counters;
    memset(&counters, 0, sizeof(counters));
    m2World* world = GetWorld(worldId);
    if (world == NULL)
    {
        return counters;
    }
    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        if (world->alive[i] == 0)
        {
            continue;
        }
        counters.bodies += 1;
        counters.awakeBodies += world->asleep[i] == 0 ? 1 : 0;
    }
    for (int32_t i = 0; i < world->maxShapeIndex; ++i)
    {
        counters.shapes += world->shapeAlive[i] != 0 ? 1 : 0;
    }
    for (int32_t i = 0; i < world->maxJointIndex; ++i)
    {
        counters.joints += world->jointAlive[i] != 0 ? 1 : 0;
    }
    counters.pairs = world->pairCount;
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        counters.touchingPairs += world->pairTouching[i] != 0 ? 1 : 0;
    }
    counters.constraints = world->lastConstraintCount;
    counters.graphColors = world->lastGraphColors;
    counters.overflowConstraints = world->lastOverflow;
    counters.stepCount = world->stepCount;
    counters.particlePairOverflow = world->particlePairOverflow;
    counters.particleBodyOverflow = world->particleBodyOverflow;
    counters.particlePoolFull = world->particlePoolFullCount;
    counters.misuse = world->misuseCount;
    return counters;
}

static int32_t AllocateJoint(m2World* world)
{
    if (world->jointFreeCount == 0)
    {
        return -1;
    }
    int32_t index = world->jointFreeQueue[world->jointFreeHead];
    world->jointFreeHead = (world->jointFreeHead + 1) % world->jointCapacity;
    world->jointFreeCount -= 1;
    if (index + 1 > world->maxJointIndex)
    {
        world->maxJointIndex = index + 1;
    }
    return index;
}

// Relative angle of B vs A from their rotations (own trig: ADR-0010).
static float RelativeJointAngle(m2Rot qA, m2Rot qB)
{
    float sin = qA.c * qB.s - qA.s * qB.c;
    float cos = qA.c * qB.c + qA.s * qB.s;
    return m2Atan2(sin, cos);
}

static m2JointId FinishJoint(m2World* world, m2WorldId worldId, int32_t index, uint8_t type,
                             int32_t bodyA, int32_t bodyB, m2Vec2 anchorA, m2Vec2 anchorB,
                             float length, float hertz, float damping)
{
    world->jointType[index] = type;
    world->jointBodyA[index] = bodyA;
    world->jointBodyB[index] = bodyB;
    world->jointLocalAnchorA[index] = anchorA;
    world->jointLocalAnchorB[index] = anchorB;
    world->jointLength[index] = length;
    world->jointHertz[index] = hertz;
    world->jointDamping[index] = damping;
    world->jointHertz2[index] = 0.0f;
    world->jointDamping2[index] = 0.0f;
    world->jointImpulse[index] = (m2Vec2){0.0f, 0.0f};
    world->jointFlags[index] = 0;
    world->jointMotorSpeed[index] = 0.0f;
    world->jointMaxMotor[index] = 0.0f;
    world->jointLower[index] = 0.0f;
    world->jointUpper[index] = 0.0f;
    world->jointLocalAxisA[index] = (m2Vec2){1.0f, 0.0f};
    world->jointRefAngle[index] = 0.0f;
    world->jointMotorImpulse[index] = 0.0f;
    world->jointLowerImpulse[index] = 0.0f;
    world->jointUpperImpulse[index] = 0.0f;
    world->jointSpringImpulse[index] = 0.0f;
    world->jointBreakForce[index] = 0.0f;
    world->jointBreakTorque[index] = 0.0f;
    world->jointCollide[index] = 1;
    world->jointTargets[index] = (m2Pos2){0.0, 0.0};
    world->jointTargetsB[index] = (m2Pos2){0.0, 0.0};
    world->jointUserData[index] = 0;
    world->jointAlive[index] = 1;
    // A new constraint wakes both ends.
    world->asleep[bodyA] = 0;
    world->sleepTimes[bodyA] = 0.0f;
    world->asleep[bodyB] = 0;
    world->sleepTimes[bodyB] = 0.0f;
    m2JointId id = {index + 1, worldId.index1, world->jointGenerations[index]};
    return id;
}

m2DistanceJointDef m2DefaultDistanceJointDef(void)
{
    m2DistanceJointDef def;
    memset(&def, 0, sizeof(def));
    def.internalValue = M2_DJOINT_COOKIE;
    return def;
}

m2RevoluteJointDef m2DefaultRevoluteJointDef(void)
{
    m2RevoluteJointDef def;
    memset(&def, 0, sizeof(def));
    def.internalValue = M2_RJOINT_COOKIE;
    return def;
}

m2JointId m2CreateDistanceJoint(m2WorldId worldId, const m2DistanceJointDef* def)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_DJOINT_COOKIE)
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    int32_t bodyA = BodySlot(world, def->bodyIdA);
    int32_t bodyB = BodySlot(world, def->bodyIdB);
    if (bodyA < 0 || bodyB < 0 || bodyA == bodyB)
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    int32_t index = AllocateJoint(world);
    if (index < 0)
    {
        return m2_nullJointId;
    }
    float length = def->length;
    if (!(length > 0.0f))
    {
        // Derive from spawn poses: the single f64 crossing.
        m2Transform xfA = world->transforms[bodyA];
        m2Transform xfB = world->transforms[bodyB];
        m2Vec2 wA = {xfA.q.c * def->localAnchorA.x - xfA.q.s * def->localAnchorA.y,
                     xfA.q.s * def->localAnchorA.x + xfA.q.c * def->localAnchorA.y};
        m2Vec2 wB = {xfB.q.c * def->localAnchorB.x - xfB.q.s * def->localAnchorB.y,
                     xfB.q.s * def->localAnchorB.x + xfB.q.c * def->localAnchorB.y};
        float dx = (float)(xfB.p.x - xfA.p.x) + wB.x - wA.x;
        float dy = (float)(xfB.p.y - xfA.p.y) + wB.y - wA.y;
        length = sqrtf(dx * dx + dy * dy);
    }
    m2JointId jointId = FinishJoint(world, worldId, index, 0, bodyA, bodyB, def->localAnchorA,
                                    def->localAnchorB, length, def->hertz, def->dampingRatio);
    // The hard range: off by default (0 .. huge); a def maxLength <= 0
    // means unbounded, mirroring "length <= 0 derives".
    world->jointLower[index] = def->minLength > 0.0f ? def->minLength : 0.0f;
    world->jointUpper[index] = def->maxLength > 0.0f ? def->maxLength : 3.4e38f;
    if (def->enableSpring)
    {
        world->jointFlags[index] |= 16u; // rope/rod: gate the rest-length row
    }
    world->jointUserData[index] = def->userData;
    world->jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        RefilterJointedBodies(world, bodyA, bodyB);
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2DistanceJointDef def;
            m2JointId expected;
        } record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateDistanceJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

m2JointId m2CreateRevoluteJoint(m2WorldId worldId, const m2RevoluteJointDef* def)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_RJOINT_COOKIE)
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    int32_t bodyA = BodySlot(world, def->bodyIdA);
    int32_t bodyB = BodySlot(world, def->bodyIdB);
    if (bodyA < 0 || bodyB < 0 || bodyA == bodyB)
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    int32_t index = AllocateJoint(world);
    if (index < 0)
    {
        return m2_nullJointId;
    }
    m2JointId jointId = FinishJoint(world, worldId, index, 1, bodyA, bodyB, def->localAnchorA,
                                    def->localAnchorB, 0.0f, def->hertz, def->dampingRatio);
    world->jointUserData[index] = def->userData;
    world->jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        RefilterJointedBodies(world, bodyA, bodyB);
    }
    world->jointFlags[index] = (def->enableMotor ? 1u : 0u) | (def->enableLimit ? 2u : 0u);
    world->jointMotorSpeed[index] = def->motorSpeed;
    world->jointMaxMotor[index] = def->maxMotorTorque;
    world->jointLower[index] = def->lowerAngle;
    world->jointUpper[index] = def->upperAngle;
    world->jointRefAngle[index] =
        RelativeJointAngle(world->transforms[bodyA].q, world->transforms[bodyB].q);
    world->jointHertz2[index] = def->springHertz;
    world->jointDamping2[index] = def->springDampingRatio;
    if (world->journalActive != 0)
    {
        struct
        {
            m2RevoluteJointDef def;
            m2JointId expected;
        } record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateRevoluteJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

m2PrismaticJointDef m2DefaultPrismaticJointDef(void)
{
    m2PrismaticJointDef def;
    memset(&def, 0, sizeof(def));
    def.localAxisA = (m2Vec2){1.0f, 0.0f};
    def.internalValue = M2_PJOINT_COOKIE;
    return def;
}

m2JointId m2CreatePrismaticJoint(m2WorldId worldId, const m2PrismaticJointDef* def)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_PJOINT_COOKIE)
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    int32_t bodyA = BodySlot(world, def->bodyIdA);
    int32_t bodyB = BodySlot(world, def->bodyIdB);
    if (bodyA < 0 || bodyB < 0 || bodyA == bodyB)
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    float axisLength =
        sqrtf(def->localAxisA.x * def->localAxisA.x + def->localAxisA.y * def->localAxisA.y);
    if (!(axisLength > 1.19209290e-7f))
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    int32_t index = AllocateJoint(world);
    if (index < 0)
    {
        return m2_nullJointId;
    }
    m2JointId jointId = FinishJoint(world, worldId, index, 2, bodyA, bodyB, def->localAnchorA,
                                    def->localAnchorB, 0.0f, def->hertz, def->dampingRatio);
    world->jointUserData[index] = def->userData;
    world->jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        RefilterJointedBodies(world, bodyA, bodyB);
    }
    world->jointFlags[index] = (def->enableMotor ? 1u : 0u) | (def->enableLimit ? 2u : 0u);
    world->jointMotorSpeed[index] = def->motorSpeed;
    world->jointMaxMotor[index] = def->maxMotorForce;
    world->jointLower[index] = def->lowerTranslation;
    world->jointUpper[index] = def->upperTranslation;
    world->jointLocalAxisA[index] =
        (m2Vec2){def->localAxisA.x / axisLength, def->localAxisA.y / axisLength};
    world->jointRefAngle[index] =
        RelativeJointAngle(world->transforms[bodyA].q, world->transforms[bodyB].q);
    if (world->journalActive != 0)
    {
        struct
        {
            m2PrismaticJointDef def;
            m2JointId expected;
        } record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreatePrismaticJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

m2WeldJointDef m2DefaultWeldJointDef(void)
{
    m2WeldJointDef def;
    memset(&def, 0, sizeof(def));
    def.internalValue = M2_WJOINT_COOKIE;
    return def;
}

m2JointId m2CreateWeldJoint(m2WorldId worldId, const m2WeldJointDef* def)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_WJOINT_COOKIE)
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    int32_t bodyA = BodySlot(world, def->bodyIdA);
    int32_t bodyB = BodySlot(world, def->bodyIdB);
    if (bodyA < 0 || bodyB < 0 || bodyA == bodyB)
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    int32_t index = AllocateJoint(world);
    if (index < 0)
    {
        return m2_nullJointId;
    }
    m2JointId jointId =
        FinishJoint(world, worldId, index, 3, bodyA, bodyB, def->localAnchorA, def->localAnchorB,
                    0.0f, def->linearHertz, def->linearDampingRatio);
    world->jointUserData[index] = def->userData;
    world->jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        RefilterJointedBodies(world, bodyA, bodyB);
    }
    world->jointHertz2[index] = def->angularHertz;
    world->jointDamping2[index] = def->angularDampingRatio;
    world->jointRefAngle[index] =
        RelativeJointAngle(world->transforms[bodyA].q, world->transforms[bodyB].q);
    if (world->journalActive != 0)
    {
        struct
        {
            m2WeldJointDef def;
            m2JointId expected;
        } record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateWeldJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

m2WheelJointDef m2DefaultWheelJointDef(void)
{
    m2WheelJointDef def;
    memset(&def, 0, sizeof(def));
    def.localAxisA = (m2Vec2){0.0f, 1.0f};
    def.enableSpring = true;
    def.hertz = 2.0f;
    def.dampingRatio = 0.7f;
    def.internalValue = M2_WHJOINT_COOKIE;
    return def;
}

m2JointId m2CreateWheelJoint(m2WorldId worldId, const m2WheelJointDef* def)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_WHJOINT_COOKIE)
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    int32_t bodyA = BodySlot(world, def->bodyIdA);
    int32_t bodyB = BodySlot(world, def->bodyIdB);
    if (bodyA < 0 || bodyB < 0 || bodyA == bodyB)
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    float axisLength =
        sqrtf(def->localAxisA.x * def->localAxisA.x + def->localAxisA.y * def->localAxisA.y);
    if (!(axisLength > 1.19209290e-7f))
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    int32_t index = AllocateJoint(world);
    if (index < 0)
    {
        return m2_nullJointId;
    }
    m2JointId jointId = FinishJoint(world, worldId, index, 4, bodyA, bodyB, def->localAnchorA,
                                    def->localAnchorB, 0.0f, def->hertz, def->dampingRatio);
    world->jointUserData[index] = def->userData;
    world->jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        RefilterJointedBodies(world, bodyA, bodyB);
    }
    world->jointFlags[index] =
        (def->enableMotor ? 1u : 0u) | (def->enableLimit ? 2u : 0u) | (def->enableSpring ? 4u : 0u);
    world->jointMotorSpeed[index] = def->motorSpeed;
    world->jointMaxMotor[index] = def->maxMotorTorque;
    world->jointLower[index] = def->lowerTranslation;
    world->jointUpper[index] = def->upperTranslation;
    world->jointLocalAxisA[index] =
        (m2Vec2){def->localAxisA.x / axisLength, def->localAxisA.y / axisLength};
    if (world->journalActive != 0)
    {
        struct
        {
            m2WheelJointDef def;
            m2JointId expected;
        } record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateWheelJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

// One journaled channel for every runtime joint parameter: replay
// re-drives the same setters through the same records.
void m2SetJointParamInternal(m2World* world, m2JointId jointId, uint8_t param, float value)
{
    int32_t index = jointId.index1 - 1;
    if (index < 0 || index >= world->jointCapacity || world->jointAlive[index] == 0 ||
        world->jointGenerations[index] != jointId.generation)
    {
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2JointId joint;
            float value;
            uint8_t param;
        } record;
        memset(&record, 0, sizeof(record));
        record.joint = jointId;
        record.value = value;
        record.param = param;
        m2JournalRecord(world, m2_opSetJointParam, &record, (int32_t)sizeof(record));
    }
    switch (param)
    {
    case m2_jointParamMotorSpeed:
        world->jointMotorSpeed[index] = value;
        break;
    case m2_jointParamMaxMotor:
        world->jointMaxMotor[index] = value;
        break;
    case m2_jointParamEnableMotor:
        world->jointFlags[index] =
            value != 0.0f ? (world->jointFlags[index] | 1u) : (world->jointFlags[index] & ~1u);
        break;
    case m2_jointParamEnableLimit:
        world->jointFlags[index] =
            value != 0.0f ? (world->jointFlags[index] | 2u) : (world->jointFlags[index] & ~2u);
        break;
    case m2_jointParamLower:
        world->jointLower[index] = value;
        break;
    case m2_jointParamBreakForce:
        world->jointBreakForce[index] = value;
        break;
    case m2_jointParamBreakTorque:
        world->jointBreakTorque[index] = value;
        break;
    case m2_jointParamHertz:
        world->jointHertz[index] = value;
        break;
    case m2_jointParamDamping:
        world->jointDamping[index] = value;
        break;
    case m2_jointParamAngularHertz:
        world->jointHertz2[index] = value;
        if (value == 0.0f)
        {
            world->jointSpringImpulse[index] = 0.0f; // disable drops memory
        }
        break;
    case m2_jointParamAngularDamping:
        world->jointDamping2[index] = value;
        break;
    case m2_jointParamLength:
        // Reference semantics: retargeting the rod drops its memory.
        world->jointLength[index] = value;
        world->jointImpulse[index] = (m2Vec2){0.0f, 0.0f};
        world->jointLowerImpulse[index] = 0.0f;
        world->jointUpperImpulse[index] = 0.0f;
        break;
    case m2_jointParamMinLength:
        world->jointLower[index] = value;
        world->jointImpulse[index] = (m2Vec2){0.0f, 0.0f};
        world->jointLowerImpulse[index] = 0.0f;
        world->jointUpperImpulse[index] = 0.0f;
        break;
    case m2_jointParamMaxLength:
        world->jointUpper[index] = value;
        world->jointImpulse[index] = (m2Vec2){0.0f, 0.0f};
        world->jointLowerImpulse[index] = 0.0f;
        world->jointUpperImpulse[index] = 0.0f;
        break;
    case m2_jointParamGearRatio:
        world->jointLength[index] = value; // phase carries over on purpose
        break;
    case m2_jointParamPulleyRatio:
        // Recapture the rope total from current geometry so the
        // machine does not snap; drop memory like a distance retarget.
        world->jointRefAngle[index] =
            PulleyLiveLength(world, index, 0) + value * PulleyLiveLength(world, index, 1);
        world->jointLength[index] = value;
        world->jointImpulse[index] = (m2Vec2){0.0f, 0.0f};
        break;
    default:
        world->jointUpper[index] = value;
        break;
    }
    // Any parameter change wakes both ends.
    int32_t bodyA = world->jointBodyA[index];
    int32_t bodyB = world->jointBodyB[index];
    if (world->types[bodyA] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[bodyA] = 0;
        world->sleepTimes[bodyA] = 0.0f;
    }
    if (world->types[bodyB] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[bodyB] = 0;
        world->sleepTimes[bodyB] = 0.0f;
    }
}

void m2Joint_SetMotorSpeed(m2JointId jointId, float speed)
{
    m2World* world = WorldFromIndex(jointId.world0);
    if (world != NULL)
    {
        m2SetJointParamInternal(world, jointId, m2_jointParamMotorSpeed, speed);
    }
}

void m2Joint_SetMaxMotor(m2JointId jointId, float maxTorqueOrForce)
{
    m2World* world = WorldFromIndex(jointId.world0);
    if (world != NULL)
    {
        m2SetJointParamInternal(world, jointId, m2_jointParamMaxMotor, maxTorqueOrForce);
    }
}

void m2Joint_EnableMotor(m2JointId jointId, bool enable)
{
    m2World* world = WorldFromIndex(jointId.world0);
    if (world != NULL)
    {
        m2SetJointParamInternal(world, jointId, m2_jointParamEnableMotor, enable ? 1.0f : 0.0f);
    }
}

void m2Joint_EnableLimit(m2JointId jointId, bool enable)
{
    m2World* world = WorldFromIndex(jointId.world0);
    if (world != NULL)
    {
        m2SetJointParamInternal(world, jointId, m2_jointParamEnableLimit, enable ? 1.0f : 0.0f);
    }
}

void m2Joint_SetLimits(m2JointId jointId, float lower, float upper)
{
    m2World* world = WorldFromIndex(jointId.world0);
    if (world != NULL)
    {
        m2SetJointParamInternal(world, jointId, m2_jointParamLower, lower);
        m2SetJointParamInternal(world, jointId, m2_jointParamUpper, upper);
    }
}

static bool JointHasSpringRow(const m2World* world, int32_t index)
{
    uint8_t type = world->jointType[index];
    return type != 5 && type != 6; // filter has no rows, motor no spring
}

void m2Joint_SetSpringHertz(m2JointId jointId, float hertz)
{
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t index = world != NULL ? JointSlotChecked(world, jointId) : -1;
    if (index < 0 || !JointHasSpringRow(world, index) || !(hertz >= 0.0f))
    {
        M2_ASSERT(false);
        return;
    }
    m2SetJointParamInternal(world, jointId, m2_jointParamHertz, hertz);
}

void m2Joint_SetSpringDampingRatio(m2JointId jointId, float dampingRatio)
{
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t index = world != NULL ? JointSlotChecked(world, jointId) : -1;
    if (index < 0 || !JointHasSpringRow(world, index) || !(dampingRatio >= 0.0f))
    {
        M2_ASSERT(false);
        return;
    }
    m2SetJointParamInternal(world, jointId, m2_jointParamDamping, dampingRatio);
}

void m2Joint_SetAngularSpringHertz(m2JointId jointId, float hertz)
{
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t index = world != NULL ? JointSlotChecked(world, jointId) : -1;
    if (index < 0 || (world->jointType[index] != 3 && world->jointType[index] != 1) ||
        !(hertz >= 0.0f))
    {
        M2_ASSERT(false);
        return;
    }
    m2SetJointParamInternal(world, jointId, m2_jointParamAngularHertz, hertz);
}

void m2Joint_SetAngularSpringDampingRatio(m2JointId jointId, float dampingRatio)
{
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t index = world != NULL ? JointSlotChecked(world, jointId) : -1;
    if (index < 0 || (world->jointType[index] != 3 && world->jointType[index] != 1) ||
        !(dampingRatio >= 0.0f))
    {
        M2_ASSERT(false);
        return;
    }
    m2SetJointParamInternal(world, jointId, m2_jointParamAngularDamping, dampingRatio);
}

void m2DistanceJoint_SetLength(m2JointId jointId, float length)
{
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t index = world != NULL ? JointSlotChecked(world, jointId) : -1;
    if (index < 0 || world->jointType[index] != 0 || !(length > 0.0f))
    {
        M2_ASSERT(false);
        return;
    }
    m2SetJointParamInternal(world, jointId, m2_jointParamLength, length);
}

void m2DistanceJoint_SetLengthRange(m2JointId jointId, float minLength, float maxLength)
{
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t index = world != NULL ? JointSlotChecked(world, jointId) : -1;
    if (index < 0 || world->jointType[index] != 0)
    {
        M2_ASSERT(false);
        return;
    }
    // Reference clamps: slop floor, ordered pair.
    float lo = minLength > 0.005f ? minLength : 0.005f;
    float hi = maxLength > 0.005f ? maxLength : 0.005f;
    float lower = lo < hi ? lo : hi;
    float upper = lo < hi ? hi : lo;
    m2SetJointParamInternal(world, jointId, m2_jointParamMinLength, lower);
    m2SetJointParamInternal(world, jointId, m2_jointParamMaxLength, upper);
}

void m2Joint_SetBreakLimits(m2JointId jointId, float maxForce, float maxTorque)
{
    m2World* world = WorldFromIndex(jointId.world0);
    if (world != NULL)
    {
        m2SetJointParamInternal(world, jointId, m2_jointParamBreakForce, maxForce);
        m2SetJointParamInternal(world, jointId, m2_jointParamBreakTorque, maxTorque);
    }
}

void m2DestroyJoint(m2JointId jointId)
{
    m2World* world = WorldFromIndex(jointId.world0);
    if (world == NULL)
    {
        return;
    }
    int32_t index = jointId.index1 - 1;
    if (index < 0 || index >= world->jointCapacity || world->jointAlive[index] == 0 ||
        world->jointGenerations[index] != jointId.generation)
    {
        return;
    }
    m2JournalRecord(world, m2_opDestroyJoint, &jointId, (int32_t)sizeof(jointId));
    m2DestroyJointInternal(world, index);
}

// The guts, shared with the solver's break pass (which must not
// journal: breaking is derived from state and replays by itself).
void m2DestroyJointInternal(m2World* world, int32_t index)
{
    // Both ends wake: a constraint vanished.
    int32_t bodyA = world->jointBodyA[index];
    int32_t bodyB = world->jointBodyB[index];
    if (world->types[bodyA] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[bodyA] = 0;
        world->sleepTimes[bodyA] = 0.0f;
    }
    if (world->types[bodyB] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[bodyB] = 0;
        world->sleepTimes[bodyB] = 0.0f;
    }
    world->jointAlive[index] = 0;
    if (world->jointCollide[index] == 0)
    {
        RefilterJointedBodies(world, bodyA, bodyB); // pairs may return
    }
    if (world->jointGenerations[index] == UINT16_MAX)
    {
        world->jointRetiredCount += 1;
        return;
    }
    world->jointGenerations[index] += 1;
    world->jointFreeQueue[world->jointFreeTail] = index;
    world->jointFreeTail = (world->jointFreeTail + 1) % world->jointCapacity;
    world->jointFreeCount += 1;
}

void m2Body_Disable(m2BodyId bodyId)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0 || world->disabled[index] != 0)
    {
        return; // already dormant: no-op stays unjournaled
    }
    m2JournalRecord(world, m2_opDisableBody, &bodyId, (int32_t)sizeof(bodyId));
    for (int32_t s = world->bodyShapeHead[index]; s != -1; s = world->shapeNext[s])
    {
        RetireShapeFromBroadphase(world, s);
    }
    world->disabled[index] = 1;
    world->asleep[index] = 0;
    world->sleepTimes[index] = 0.0f;
}

void m2Body_Enable(m2BodyId bodyId)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0 || world->disabled[index] == 0)
    {
        return;
    }
    m2JournalRecord(world, m2_opEnableBody, &bodyId, (int32_t)sizeof(bodyId));
    world->disabled[index] = 0;
    int32_t tree = world->types[index];
    for (int32_t s = world->bodyShapeHead[index]; s != -1; s = world->shapeNext[s])
    {
        world->proxyIds[s] = m2Tree_Insert(&world->trees[tree], world->treeNodes[tree],
                                           Fatten(ShapeTightAABB(world, s)), s);
        PushMoved(world, s);
    }
    world->asleep[index] = 0;
    world->sleepTimes[index] = 0.0f;
}

void m2Body_SetDominance(m2BodyId bodyId, int8_t dominance)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return;
    }
    if (world->dominances[index] == dominance)
    {
        return; // no-op stays unjournaled
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2BodyId body;
            int8_t dominance;
        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.dominance = dominance;
        m2JournalRecord(world, m2_opSetDominance, &record, (int32_t)sizeof(record));
    }
    world->dominances[index] = dominance;
    if (world->types[index] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[index] = 0;
        world->sleepTimes[index] = 0.0f;
    }
}

int8_t m2Body_GetDominance(m2BodyId bodyId)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return 0;
    }
    return world->dominances[index];
}

bool m2Body_IsEnabled(m2BodyId bodyId)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    return index >= 0 && world->disabled[index] == 0;
}

void m2Body_SetMassData(m2BodyId bodyId, m2MassData massData)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0 || world->types[index] != (uint8_t)m2_dynamicBody || !(massData.mass > 0.0f))
    {
        M2_ASSERT(false);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2BodyId body;
            m2MassData data;
        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.data = massData;
        m2JournalRecord(world, m2_opSetMassData, &record, (int32_t)sizeof(record));
    }
    world->invMass[index] = 1.0f / massData.mass;
    float inertiaCenter =
        massData.rotationalInertia - massData.mass * (massData.center.x * massData.center.x +
                                                      massData.center.y * massData.center.y);
    world->invInertia[index] =
        world->fixedRotations[index] == 0 && inertiaCenter > 0.0f ? 1.0f / inertiaCenter : 0.0f;
    world->localCenters[index] = massData.center;
    world->asleep[index] = 0;
    world->sleepTimes[index] = 0.0f;
}

m2MassData m2Body_GetMassData(m2BodyId bodyId)
{
    m2MassData data = {0.0f, {0.0f, 0.0f}, 0.0f};
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return data;
    }
    float invMass = world->invMass[index];
    data.mass = invMass > 0.0f ? 1.0f / invMass : 0.0f;
    data.center = world->localCenters[index];
    float invI = world->invInertia[index];
    float inertiaCenter = invI > 0.0f ? 1.0f / invI : 0.0f;
    data.rotationalInertia =
        inertiaCenter + data.mass * (data.center.x * data.center.x + data.center.y * data.center.y);
    return data;
}

void m2Body_ApplyMassFromShapes(m2BodyId bodyId)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return;
    }
    m2JournalRecord(world, m2_opMassFromShapes, &bodyId, (int32_t)sizeof(bodyId));
    RecomputeMass(world, index);
    if (world->types[index] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[index] = 0;
        world->sleepTimes[index] = 0.0f;
    }
}

m2ExplosionDef m2DefaultExplosionDef(void)
{
    m2ExplosionDef def;
    memset(&def, 0, sizeof(def));
    def.radius = 1.0f;
    def.falloff = 0.5f;
    def.impulse = 1.0f;
    def.maskBits = 0xFFFFFFFFu;
    def.internalValue = M2_EXPLODE_COOKIE;
    return def;
}

void m2World_Explode(m2WorldId worldId, const m2ExplosionDef* def)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_EXPLODE_COOKIE ||
        !(def->radius >= 0.0f) || !(def->falloff > 0.0f))
    {
        M2_ASSERT(false);
        return;
    }
    m2JournalRecord(world, m2_opExplode, def, (int32_t)sizeof(*def));
    // The blast applies raw impulses below; they must not be recorded
    // twice (the chain-create suppression pattern).
    uint8_t journalWasActive = world->journalActive;
    world->journalActive = 0;

    float reach = def->radius + def->falloff;
    for (int32_t s = 0; s < world->maxShapeIndex; ++s)
    {
        if (world->shapeAlive[s] == 0 || world->shapeSensor[s] != 0 ||
            (world->shapeCategory[s] & def->maskBits) == 0)
        {
            continue;
        }
        int32_t body = world->shapeBody[s];
        if (world->types[body] != (uint8_t)m2_dynamicBody || world->disabled[body] != 0)
        {
            continue;
        }
        // Closest point on the shape to the blast center, in the
        // shape's body frame (one f64 crossing).
        m2Transform xf = world->transforms[body];
        m2Vec2 rel = {(float)(def->position.x - xf.p.x), (float)(def->position.y - xf.p.y)};
        m2Vec2 local = {xf.q.c * rel.x + xf.q.s * rel.y, -xf.q.s * rel.x + xf.q.c * rel.y};
        m2DistanceProxy target = m2GeometryProxy(&world->shapeGeometry[s]);
        m2DistanceProxy point;
        point.points[0] = local;
        point.count = 1;
        point.radius = 0.0f;
        m2DistanceResult d = m2ShapeDistance(&target, &point);
        float dist = d.distance - target.radius;
        if (dist > reach)
        {
            continue;
        }
        // Blast direction: away from the center. Deep overlap falls
        // back to the body-center direction; a dead-centered blast on
        // a centered body has no direction and skips, loudly fair.
        m2Vec2 dir;
        if (dist > 0.0f)
        {
            dir = (m2Vec2){-d.normal.x, -d.normal.y}; // normal points shape->center
        }
        else
        {
            m2Vec2 lc = world->localCenters[body];
            dir = (m2Vec2){lc.x - local.x, lc.y - local.y};
            float len = sqrtf(dir.x * dir.x + dir.y * dir.y);
            if (!(len > 0.0f))
            {
                continue;
            }
            dir = (m2Vec2){dir.x / len, dir.y / len};
            dist = 0.0f;
        }
        float scale = dist <= def->radius ? 1.0f : 1.0f - (dist - def->radius) / def->falloff;
        float mag = def->impulse * scale;
        m2Vec2 hitLocal = {d.pointA.x + target.radius * d.normal.x,
                           d.pointA.y + target.radius * d.normal.y};
        m2Vec2 lc = world->localCenters[body];
        m2Vec2 arm = {hitLocal.x - lc.x, hitLocal.y - lc.y};
        m2Vec2 impulseLocal = {mag * dir.x, mag * dir.y};
        // Rotate impulse and arm out to world axes for the velocity
        // update (angular uses the local cross, identical either way).
        m2Vec2 impulseWorld = {xf.q.c * impulseLocal.x - xf.q.s * impulseLocal.y,
                               xf.q.s * impulseLocal.x + xf.q.c * impulseLocal.y};
        world->linearVelocities[body].x += world->invMass[body] * impulseWorld.x;
        world->linearVelocities[body].y += world->invMass[body] * impulseWorld.y;
        world->angularVelocities[body] +=
            world->invInertia[body] * (arm.x * impulseLocal.y - arm.y * impulseLocal.x);
        world->asleep[body] = 0;
        world->sleepTimes[body] = 0.0f;
    }
    world->journalActive = journalWasActive;
}

m2FilterJointDef m2DefaultFilterJointDef(void)
{
    m2FilterJointDef def;
    memset(&def, 0, sizeof(def));
    def.internalValue = M2_FJOINT_COOKIE;
    return def;
}

m2JointId m2CreateFilterJoint(m2WorldId worldId, const m2FilterJointDef* def)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_FJOINT_COOKIE)
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    int32_t bodyA = BodySlot(world, def->bodyIdA);
    int32_t bodyB = BodySlot(world, def->bodyIdB);
    if (bodyA < 0 || bodyB < 0 || bodyA == bodyB)
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    int32_t index = AllocateJoint(world);
    if (index < 0)
    {
        return m2_nullJointId;
    }
    m2Vec2 zero = {0.0f, 0.0f};
    m2JointId jointId =
        FinishJoint(world, worldId, index, 5, bodyA, bodyB, zero, zero, 0.0f, 0.0f, 0.0f);
    world->jointUserData[index] = def->userData;
    world->jointCollide[index] = 0; // its entire purpose
    RefilterJointedBodies(world, bodyA, bodyB);
    if (world->journalActive != 0)
    {
        struct
        {
            m2FilterJointDef def;
            m2JointId expected;
        } record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateFilterJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

m2GearJointDef m2DefaultGearJointDef(void)
{
    m2GearJointDef def;
    memset(&def, 0, sizeof(def));
    def.ratio = 1.0f;
    def.internalValue = M2_GJOINT_COOKIE;
    return def;
}

// Gear registry mapping: ratio rides jointLength; the two previous
// body rotations ride the anchor slots as (c, s) pairs so the phase
// accumulator in prepare survives any number of full turns; the
// accumulated phase itself rides jointRefAngle. All snapshot state.
m2JointId m2CreateGearJoint(m2WorldId worldId, const m2GearJointDef* def)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_GJOINT_COOKIE ||
        !(def->ratio != 0.0f))
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    int32_t bodyA = BodySlot(world, def->bodyIdA);
    int32_t bodyB = BodySlot(world, def->bodyIdB);
    if (bodyA < 0 || bodyB < 0 || bodyA == bodyB)
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    int32_t index = AllocateJoint(world);
    if (index < 0)
    {
        return m2_nullJointId;
    }
    m2Vec2 zero = {0.0f, 0.0f};
    m2JointId jointId =
        FinishJoint(world, worldId, index, 8, bodyA, bodyB, zero, zero, 0.0f, 0.0f, 0.0f);
    world->jointLength[index] = def->ratio;
    m2Rot qA = world->transforms[bodyA].q;
    m2Rot qB = world->transforms[bodyB].q;
    world->jointLocalAnchorA[index] = (m2Vec2){qA.c, qA.s};
    world->jointLocalAnchorB[index] = (m2Vec2){qB.c, qB.s};
    world->jointRefAngle[index] = 0.0f; // in phase by definition at birth
    world->jointUserData[index] = def->userData;
    world->jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        RefilterJointedBodies(world, bodyA, bodyB);
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2GearJointDef def;
            m2JointId expected;
        } record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateGearJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

void m2GearJoint_SetRatio(m2JointId jointId, float ratio)
{
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t index = world != NULL ? TypedJointSlot(world, jointId, 8) : -1;
    if (index < 0 || !(ratio != 0.0f))
    {
        M2_ASSERT(false);
        return;
    }
    m2SetJointParamInternal(world, jointId, m2_jointParamGearRatio, ratio);
}

float m2GearJoint_GetRatio(m2JointId jointId)
{
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t index = world != NULL ? TypedJointSlot(world, jointId, 8) : -1;
    return index >= 0 ? world->jointLength[index] : 0.0f;
}

m2PulleyJointDef m2DefaultPulleyJointDef(void)
{
    m2PulleyJointDef def;
    memset(&def, 0, sizeof(def));
    def.ratio = 1.0f;
    def.internalValue = M2_PLJOINT_COOKIE;
    return def;
}

// Live rope length for one pulley side: attach point (f64 body origin
// plus rotated local anchor) against the f64 ground anchor, a single
// f64 crossing like every other narrowphase entry.
static float PulleyLiveLength(m2World* world, int32_t index, int32_t side)
{
    int32_t body = side == 0 ? world->jointBodyA[index] : world->jointBodyB[index];
    m2Vec2 la = side == 0 ? world->jointLocalAnchorA[index] : world->jointLocalAnchorB[index];
    m2Pos2 g = side == 0 ? world->jointTargets[index] : world->jointTargetsB[index];
    m2Rot q = world->transforms[body].q;
    m2Vec2 arm = {q.c * la.x - q.s * la.y, q.s * la.x + q.c * la.y};
    float dx = (float)(world->transforms[body].p.x - g.x) + arm.x;
    float dy = (float)(world->transforms[body].p.y - g.y) + arm.y;
    return sqrtf(dx * dx + dy * dy);
}

// Pulley registry mapping: ratio rides jointLength, the rope total
// (constant) rides jointRefAngle, ground anchors ride jointTargets
// (A side, shared with mouse) and jointTargetsB. The total is
// CAPTURED from spawn geometry, the reference-angle convention: defs
// carry no length knobs. All snapshot state.
m2JointId m2CreatePulleyJoint(m2WorldId worldId, const m2PulleyJointDef* def)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_PLJOINT_COOKIE ||
        !(def->ratio > 0.0f))
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    int32_t bodyA = BodySlot(world, def->bodyIdA);
    int32_t bodyB = BodySlot(world, def->bodyIdB);
    if (bodyA < 0 || bodyB < 0 || bodyA == bodyB)
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    int32_t index = AllocateJoint(world);
    if (index < 0)
    {
        return m2_nullJointId;
    }
    m2JointId jointId = FinishJoint(world, worldId, index, 9, bodyA, bodyB, def->localAnchorA,
                                    def->localAnchorB, def->ratio, 0.0f, 0.0f);
    world->jointTargets[index] = def->groundAnchorA;
    world->jointTargetsB[index] = def->groundAnchorB;
    float lengthA = PulleyLiveLength(world, index, 0);
    float lengthB = PulleyLiveLength(world, index, 1);
    world->jointRefAngle[index] = lengthA + def->ratio * lengthB;
    world->jointUserData[index] = def->userData;
    world->jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        RefilterJointedBodies(world, bodyA, bodyB);
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2PulleyJointDef def;
            m2JointId expected;
        } record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreatePulleyJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

void m2PulleyJoint_SetRatio(m2JointId jointId, float ratio)
{
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t index = world != NULL ? TypedJointSlot(world, jointId, 9) : -1;
    if (index < 0 || !(ratio > 0.0f))
    {
        M2_ASSERT(false);
        return;
    }
    m2SetJointParamInternal(world, jointId, m2_jointParamPulleyRatio, ratio);
}

float m2PulleyJoint_GetRatio(m2JointId jointId)
{
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t index = world != NULL ? TypedJointSlot(world, jointId, 9) : -1;
    return index >= 0 ? world->jointLength[index] : 0.0f;
}

float m2PulleyJoint_GetLengthA(m2JointId jointId)
{
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t index = world != NULL ? TypedJointSlot(world, jointId, 9) : -1;
    return index >= 0 ? PulleyLiveLength(world, index, 0) : 0.0f;
}

float m2PulleyJoint_GetLengthB(m2JointId jointId)
{
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t index = world != NULL ? TypedJointSlot(world, jointId, 9) : -1;
    return index >= 0 ? PulleyLiveLength(world, index, 1) : 0.0f;
}

m2Pos2 m2PulleyJoint_GetGroundAnchorA(m2JointId jointId)
{
    m2Pos2 zero = {0.0, 0.0};
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t index = world != NULL ? TypedJointSlot(world, jointId, 9) : -1;
    return index >= 0 ? world->jointTargets[index] : zero;
}

m2Pos2 m2PulleyJoint_GetGroundAnchorB(m2JointId jointId)
{
    m2Pos2 zero = {0.0, 0.0};
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t index = world != NULL ? TypedJointSlot(world, jointId, 9) : -1;
    return index >= 0 ? world->jointTargetsB[index] : zero;
}

m2RatchetJointDef m2DefaultRatchetJointDef(void)
{
    m2RatchetJointDef def;
    memset(&def, 0, sizeof(def));
    def.ratchet = 0.5f;
    def.internalValue = M2_RTJOINT_COOKIE;
    return def;
}

// Ratchet registry mapping: tooth angle rides jointLength, phase
// rides jointRefAngle, the accumulated relative angle rides
// jointUpper (multi-turn exact via the gear trick: previous body
// rotations live in the anchor slots as (c, s) pairs), and the
// engaged tooth rides jointLower. All snapshot state.
m2JointId m2CreateRatchetJoint(m2WorldId worldId, const m2RatchetJointDef* def)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_RTJOINT_COOKIE ||
        !(def->ratchet != 0.0f))
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    int32_t bodyA = BodySlot(world, def->bodyIdA);
    int32_t bodyB = BodySlot(world, def->bodyIdB);
    if (bodyA < 0 || bodyB < 0 || bodyA == bodyB)
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    int32_t index = AllocateJoint(world);
    if (index < 0)
    {
        return m2_nullJointId;
    }
    m2Vec2 zero = {0.0f, 0.0f};
    m2JointId jointId =
        FinishJoint(world, worldId, index, 10, bodyA, bodyB, zero, zero, 0.0f, 0.0f, 0.0f);
    world->jointLength[index] = def->ratchet;
    world->jointRefAngle[index] = def->phase;
    m2Rot qA = world->transforms[bodyA].q;
    m2Rot qB = world->transforms[bodyB].q;
    world->jointLocalAnchorA[index] = (m2Vec2){qA.c, qA.s};
    world->jointLocalAnchorB[index] = (m2Vec2){qB.c, qB.s};
    world->jointUpper[index] = 0.0f; // accumulated relative angle
    // Engage the tooth at or behind the spawn angle (reference click).
    world->jointLower[index] =
        floorf((0.0f - def->phase) / def->ratchet) * def->ratchet + def->phase;
    world->jointUserData[index] = def->userData;
    world->jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        RefilterJointedBodies(world, bodyA, bodyB);
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2RatchetJointDef def;
            m2JointId expected;
        } record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateRatchetJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

float m2RatchetJoint_GetRatchet(m2JointId jointId)
{
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t index = world != NULL ? TypedJointSlot(world, jointId, 10) : -1;
    return index >= 0 ? world->jointLength[index] : 0.0f;
}

float m2RatchetJoint_GetPhase(m2JointId jointId)
{
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t index = world != NULL ? TypedJointSlot(world, jointId, 10) : -1;
    return index >= 0 ? world->jointRefAngle[index] : 0.0f;
}

m2MotorJointDef m2DefaultMotorJointDef(void)
{
    m2MotorJointDef def;
    memset(&def, 0, sizeof(def));
    def.maxForce = 1.0f;
    def.maxTorque = 1.0f;
    def.correctionFactor = 0.3f;
    def.internalValue = M2_MOJOINT_COOKIE;
    return def;
}

m2MouseJointDef m2DefaultMouseJointDef(void)
{
    m2MouseJointDef def;
    memset(&def, 0, sizeof(def));
    def.hertz = 4.0f;
    def.dampingRatio = 1.0f;
    def.maxForce = 35.0f;
    def.internalValue = M2_MSJOINT_COOKIE;
    return def;
}

// Registry mapping for the utility joints (documented deviations from
// the slot names): motor keeps linearOffset in jointLocalAxisA,
// angularOffset in jointRefAngle, maxForce in jointLength and
// correctionFactor in jointDamping; mouse keeps maxForce in
// jointLength and its world target in jointTargets.
m2JointId m2CreateMotorJoint(m2WorldId worldId, const m2MotorJointDef* def)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_MOJOINT_COOKIE)
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    int32_t bodyA = BodySlot(world, def->bodyIdA);
    int32_t bodyB = BodySlot(world, def->bodyIdB);
    if (bodyA < 0 || bodyB < 0 || bodyA == bodyB)
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    int32_t index = AllocateJoint(world);
    if (index < 0)
    {
        return m2_nullJointId;
    }
    m2Vec2 zero = {0.0f, 0.0f};
    m2JointId jointId =
        FinishJoint(world, worldId, index, 6, bodyA, bodyB, zero, zero, 0.0f, 0.0f, 0.0f);
    world->jointLocalAxisA[index] = def->linearOffset;
    world->jointRefAngle[index] = def->angularOffset;
    world->jointMaxMotor[index] = def->maxTorque;
    world->jointLength[index] = def->maxForce;
    world->jointDamping[index] = def->correctionFactor;
    // Spring drive rides the otherwise-idle secondary spring slots
    // (jointHertz2/jointDamping2 are only read for the angular spring of
    // the revolute and weld, which type 6 is not).
    world->jointHertz2[index] = def->hertz;
    world->jointDamping2[index] = def->dampingRatio;
    world->jointUserData[index] = def->userData;
    world->jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        RefilterJointedBodies(world, bodyA, bodyB);
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2MotorJointDef def;
            m2JointId expected;
        } record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateMotorJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

m2JointId m2CreateMouseJoint(m2WorldId worldId, const m2MouseJointDef* def)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_MSJOINT_COOKIE)
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    int32_t bodyA = BodySlot(world, def->bodyIdA);
    int32_t bodyB = BodySlot(world, def->bodyIdB);
    if (bodyA < 0 || bodyB < 0 || bodyA == bodyB)
    {
        M2_ASSERT(false);
        return m2_nullJointId;
    }
    int32_t index = AllocateJoint(world);
    if (index < 0)
    {
        return m2_nullJointId;
    }
    // The grab point is where the target sits at creation, in B's
    // local frame (the single f64 crossing).
    m2Transform xfB = world->transforms[bodyB];
    m2Vec2 rel = {(float)(def->target.x - xfB.p.x), (float)(def->target.y - xfB.p.y)};
    m2Vec2 grab = {xfB.q.c * rel.x + xfB.q.s * rel.y, -xfB.q.s * rel.x + xfB.q.c * rel.y};
    m2Vec2 zero = {0.0f, 0.0f};
    m2JointId jointId = FinishJoint(world, worldId, index, 7, bodyA, bodyB, zero, grab, 0.0f,
                                    def->hertz, def->dampingRatio);
    world->jointLength[index] = def->maxForce;
    world->jointTargets[index] = def->target;
    world->jointUserData[index] = def->userData;
    world->jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        RefilterJointedBodies(world, bodyA, bodyB);
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2MouseJointDef def;
            m2JointId expected;
        } record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateMouseJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

static int32_t TypedJointSlot(m2World* world, m2JointId jointId, uint8_t type)
{
    int32_t index = jointId.index1 - 1;
    if (index < 0 || index >= world->jointCapacity || world->jointAlive[index] == 0 ||
        world->jointGenerations[index] != jointId.generation || world->jointType[index] != type)
    {
        world->misuseCount += 1; // stale id or wrong type on a typed path
        return -1;
    }
    return index;
}

void m2MotorJoint_SetOffsets(m2JointId jointId, m2Vec2 linearOffset, float angularOffset)
{
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t index = world != NULL ? TypedJointSlot(world, jointId, 6) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2JointId joint;
            m2Vec2 linear;
            float angular;
        } record;
        memset(&record, 0, sizeof(record));
        record.joint = jointId;
        record.linear = linearOffset;
        record.angular = angularOffset;
        m2JournalRecord(world, m2_opMotorOffsets, &record, (int32_t)sizeof(record));
    }
    world->jointLocalAxisA[index] = linearOffset;
    world->jointRefAngle[index] = angularOffset;
    // Retargeting wakes both ends: the platform starts moving.
    int32_t bodyA = world->jointBodyA[index];
    int32_t bodyB = world->jointBodyB[index];
    if (world->types[bodyA] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[bodyA] = 0;
        world->sleepTimes[bodyA] = 0.0f;
    }
    if (world->types[bodyB] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[bodyB] = 0;
        world->sleepTimes[bodyB] = 0.0f;
    }
}

m2Vec2 m2MotorJoint_GetLinearOffset(m2JointId jointId)
{
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t index = world != NULL ? TypedJointSlot(world, jointId, 6) : -1;
    m2Vec2 zero = {0.0f, 0.0f};
    return index >= 0 ? world->jointLocalAxisA[index] : zero;
}

float m2MotorJoint_GetAngularOffset(m2JointId jointId)
{
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t index = world != NULL ? TypedJointSlot(world, jointId, 6) : -1;
    return index >= 0 ? world->jointRefAngle[index] : 0.0f;
}

float m2MotorJoint_GetMaxForce(m2JointId jointId)
{
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t index = world != NULL ? TypedJointSlot(world, jointId, 6) : -1;
    return index >= 0 ? world->jointLength[index] : 0.0f;
}

float m2MotorJoint_GetCorrectionFactor(m2JointId jointId)
{
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t index = world != NULL ? TypedJointSlot(world, jointId, 6) : -1;
    return index >= 0 ? world->jointDamping[index] : 0.0f;
}

void m2MouseJoint_SetTarget(m2JointId jointId, m2Pos2 target)
{
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t index = world != NULL ? TypedJointSlot(world, jointId, 7) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2JointId joint;
            m2Pos2 target;
        } record;
        memset(&record, 0, sizeof(record));
        record.joint = jointId;
        record.target = target;
        m2JournalRecord(world, m2_opMouseTarget, &record, (int32_t)sizeof(record));
    }
    world->jointTargets[index] = target;
    int32_t bodyB = world->jointBodyB[index];
    if (world->types[bodyB] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[bodyB] = 0;
        world->sleepTimes[bodyB] = 0.0f;
    }
}

m2Pos2 m2MouseJoint_GetTarget(m2JointId jointId)
{
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t index = world != NULL ? TypedJointSlot(world, jointId, 7) : -1;
    m2Pos2 zero = {0.0, 0.0};
    return index >= 0 ? world->jointTargets[index] : zero;
}

float m2MouseJoint_GetMaxForce(m2JointId jointId)
{
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t index = world != NULL ? TypedJointSlot(world, jointId, 7) : -1;
    return index >= 0 ? world->jointLength[index] : 0.0f;
}

bool m2Joint_GetCollideConnected(m2JointId jointId)
{
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t index = jointId.index1 - 1;
    if (world == NULL || index < 0 || index >= world->jointCapacity ||
        world->jointAlive[index] == 0 || world->jointGenerations[index] != jointId.generation)
    {
        M2_ASSERT(false);
        return false;
    }
    return world->jointCollide[index] != 0;
}

float m2Joint_GetReactionForce(m2JointId jointId)
{
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t index = jointId.index1 - 1;
    if (world == NULL || index < 0 || index >= world->jointCapacity ||
        world->jointAlive[index] == 0 || world->jointGenerations[index] != jointId.generation)
    {
        return 0.0f;
    }
    float force = 0.0f;
    float torque = 0.0f;
    m2JointReactionMagnitudes(world, index, world->lastInvH, &force, &torque);
    return force;
}

float m2Joint_GetReactionTorque(m2JointId jointId)
{
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t index = jointId.index1 - 1;
    if (world == NULL || index < 0 || index >= world->jointCapacity ||
        world->jointAlive[index] == 0 || world->jointGenerations[index] != jointId.generation)
    {
        return 0.0f;
    }
    float force = 0.0f;
    float torque = 0.0f;
    m2JointReactionMagnitudes(world, index, world->lastInvH, &force, &torque);
    return torque;
}

bool m2Joint_IsValid(m2JointId jointId)
{
    m2World* world = WorldFromIndex(jointId.world0);
    if (world == NULL)
    {
        return false;
    }
    int32_t index = jointId.index1 - 1;
    return index >= 0 && index < world->jointCapacity && world->jointAlive[index] != 0 &&
           world->jointGenerations[index] == jointId.generation;
}

// Introspection and enumeration (slice 55): pure readers for the
// editor and engine-integration walk. Every list is ascending slot
// order (the canonical order everywhere else in Maul) and returns the
// TRUE total even when it exceeds capacity, so callers can size and
// retry instead of silently missing objects.

static int32_t JointSlotChecked(const m2World* world, m2JointId jointId)
{
    int32_t index = jointId.index1 - 1;
    if (index < 0 || index >= world->jointCapacity || world->jointAlive[index] == 0 ||
        world->jointGenerations[index] != jointId.generation)
    {
        return -1;
    }
    return index;
}

m2JointType m2Joint_GetType(m2JointId jointId)
{
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t index = world != NULL ? JointSlotChecked(world, jointId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return m2_distanceJoint;
    }
    return (m2JointType)world->jointType[index];
}

m2BodyId m2Joint_GetBodyA(m2JointId jointId)
{
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t index = world != NULL ? JointSlotChecked(world, jointId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return m2_nullBodyId;
    }
    int32_t b = world->jointBodyA[index];
    m2BodyId id = {b + 1, jointId.world0, world->generations[b]};
    return id;
}

m2BodyId m2Joint_GetBodyB(m2JointId jointId)
{
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t index = world != NULL ? JointSlotChecked(world, jointId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return m2_nullBodyId;
    }
    int32_t b = world->jointBodyB[index];
    m2BodyId id = {b + 1, jointId.world0, world->generations[b]};
    return id;
}

// Joint parameter readback (slice 57): with these, a world is
// reconstructible from public getters alone; the mirror test in
// test_world.c holds that promise to hash equality.

static int32_t JointSlotLoud(m2JointId jointId, m2World** outWorld)
{
    m2World* world = WorldFromIndex(jointId.world0);
    int32_t index = world != NULL ? JointSlotChecked(world, jointId) : -1;
    *outWorld = world;
    M2_ASSERT(index >= 0);
    return index;
}

m2Vec2 m2Joint_GetLocalAnchorA(m2JointId jointId)
{
    m2World* world = NULL;
    int32_t index = JointSlotLoud(jointId, &world);
    m2Vec2 zero = {0.0f, 0.0f};
    return index >= 0 ? world->jointLocalAnchorA[index] : zero;
}

m2Vec2 m2Joint_GetLocalAnchorB(m2JointId jointId)
{
    m2World* world = NULL;
    int32_t index = JointSlotLoud(jointId, &world);
    m2Vec2 zero = {0.0f, 0.0f};
    return index >= 0 ? world->jointLocalAnchorB[index] : zero;
}

m2Vec2 m2Joint_GetLocalAxisA(m2JointId jointId)
{
    m2World* world = NULL;
    int32_t index = JointSlotLoud(jointId, &world);
    m2Vec2 zero = {0.0f, 0.0f};
    if (index < 0 || (world->jointType[index] != 2 && world->jointType[index] != 4))
    {
        M2_ASSERT(false);
        return zero;
    }
    return world->jointLocalAxisA[index];
}

float m2Joint_GetLength(m2JointId jointId)
{
    m2World* world = NULL;
    int32_t index = JointSlotLoud(jointId, &world);
    if (index < 0 || world->jointType[index] != 0)
    {
        M2_ASSERT(false);
        return 0.0f;
    }
    return world->jointLength[index];
}

float m2Joint_GetHertz(m2JointId jointId)
{
    m2World* world = NULL;
    int32_t index = JointSlotLoud(jointId, &world);
    return index >= 0 ? world->jointHertz[index] : 0.0f;
}

float m2Joint_GetDampingRatio(m2JointId jointId)
{
    m2World* world = NULL;
    int32_t index = JointSlotLoud(jointId, &world);
    return index >= 0 ? world->jointDamping[index] : 0.0f;
}

float m2Joint_GetAngularHertz(m2JointId jointId)
{
    m2World* world = NULL;
    int32_t index = JointSlotLoud(jointId, &world);
    if (index < 0 || (world->jointType[index] != 3 && world->jointType[index] != 1))
    {
        M2_ASSERT(false);
        return 0.0f;
    }
    return world->jointHertz2[index];
}

float m2Joint_GetAngularDampingRatio(m2JointId jointId)
{
    m2World* world = NULL;
    int32_t index = JointSlotLoud(jointId, &world);
    if (index < 0 || (world->jointType[index] != 3 && world->jointType[index] != 1))
    {
        M2_ASSERT(false);
        return 0.0f;
    }
    return world->jointDamping2[index];
}

float m2Joint_GetMotorSpeed(m2JointId jointId)
{
    m2World* world = NULL;
    int32_t index = JointSlotLoud(jointId, &world);
    return index >= 0 ? world->jointMotorSpeed[index] : 0.0f;
}

float m2Joint_GetMaxMotor(m2JointId jointId)
{
    m2World* world = NULL;
    int32_t index = JointSlotLoud(jointId, &world);
    return index >= 0 ? world->jointMaxMotor[index] : 0.0f;
}

bool m2Joint_IsMotorEnabled(m2JointId jointId)
{
    m2World* world = NULL;
    int32_t index = JointSlotLoud(jointId, &world);
    return index >= 0 && (world->jointFlags[index] & 1u) != 0;
}

bool m2Joint_IsLimitEnabled(m2JointId jointId)
{
    m2World* world = NULL;
    int32_t index = JointSlotLoud(jointId, &world);
    return index >= 0 && (world->jointFlags[index] & 2u) != 0;
}

bool m2Joint_IsSpringEnabled(m2JointId jointId)
{
    m2World* world = NULL;
    int32_t index = JointSlotLoud(jointId, &world);
    return index >= 0 && (world->jointFlags[index] & 4u) != 0;
}

void m2Joint_GetLimits(m2JointId jointId, float* lower, float* upper)
{
    m2World* world = NULL;
    int32_t index = JointSlotLoud(jointId, &world);
    if (lower != NULL)
    {
        *lower = index >= 0 ? world->jointLower[index] : 0.0f;
    }
    if (upper != NULL)
    {
        *upper = index >= 0 ? world->jointUpper[index] : 0.0f;
    }
}

void m2Joint_GetBreakLimits(m2JointId jointId, float* maxForce, float* maxTorque)
{
    m2World* world = NULL;
    int32_t index = JointSlotLoud(jointId, &world);
    if (maxForce != NULL)
    {
        *maxForce = index >= 0 ? world->jointBreakForce[index] : 0.0f;
    }
    if (maxTorque != NULL)
    {
        *maxTorque = index >= 0 ? world->jointBreakTorque[index] : 0.0f;
    }
}

m2BodyType m2Body_GetType(m2BodyId bodyId)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return m2_staticBody;
    }
    return (m2BodyType)world->types[index];
}

m2ShapeType m2Shape_GetType(m2ShapeId shapeId)
{
    m2World* world = WorldFromIndex(shapeId.world0);
    int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return m2_circleShape;
    }
    return (m2ShapeType)world->shapeGeometry[index].type;
}

bool m2Shape_IsSensor(m2ShapeId shapeId)
{
    m2World* world = WorldFromIndex(shapeId.world0);
    int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return false;
    }
    return world->shapeSensor[index] != 0;
}

void m2Shape_GetFilter(m2ShapeId shapeId, uint32_t* categoryBits, uint32_t* maskBits,
                       int32_t* groupIndex)
{
    m2World* world = WorldFromIndex(shapeId.world0);
    int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;
    uint32_t category = 0;
    uint32_t mask = 0;
    int32_t group = 0;
    if (index >= 0)
    {
        category = world->shapeCategory[index];
        mask = world->shapeMask[index];
        group = world->shapeGroup[index];
    }
    else
    {
        M2_ASSERT(false);
    }
    if (categoryBits != NULL)
    {
        *categoryBits = category;
    }
    if (maskBits != NULL)
    {
        *maskBits = mask;
    }
    if (groupIndex != NULL)
    {
        *groupIndex = group;
    }
}

int32_t m2World_GetBodies(m2WorldId worldId, m2BodyId* ids, int32_t capacity)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL)
    {
        return 0;
    }
    int32_t total = 0;
    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        if (world->alive[i] == 0)
        {
            continue;
        }
        if (ids != NULL && total < capacity)
        {
            m2BodyId id = {i + 1, world->worldIndex0, world->generations[i]};
            ids[total] = id;
        }
        total += 1;
    }
    return total;
}

int32_t m2World_GetJoints(m2WorldId worldId, m2JointId* ids, int32_t capacity)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL)
    {
        return 0;
    }
    int32_t total = 0;
    for (int32_t i = 0; i < world->maxJointIndex; ++i)
    {
        if (world->jointAlive[i] == 0)
        {
            continue;
        }
        if (ids != NULL && total < capacity)
        {
            m2JointId id = {i + 1, world->worldIndex0, world->jointGenerations[i]};
            ids[total] = id;
        }
        total += 1;
    }
    return total;
}

int32_t m2World_GetChains(m2WorldId worldId, m2ChainId* ids, int32_t capacity)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL)
    {
        return 0;
    }
    int32_t total = 0;
    for (int32_t i = 0; i < world->maxChainIndex; ++i)
    {
        if (world->chainAlive[i] == 0)
        {
            continue;
        }
        if (ids != NULL && total < capacity)
        {
            m2ChainId id = {i + 1, world->worldIndex0, world->chainGenerations[i]};
            ids[total] = id;
        }
        total += 1;
    }
    return total;
}

// Geometry readback: exact stored bits, loud on a type mismatch.
#define M2_GEOMETRY_GETTER(name, fieldType, field, enumValue)                                      \
    fieldType name(m2ShapeId shapeId)                                                              \
    {                                                                                              \
        fieldType zero;                                                                            \
        memset(&zero, 0, sizeof(zero));                                                            \
        m2World* world = WorldFromIndex(shapeId.world0);                                           \
        int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;                            \
        if (index < 0 || world->shapeGeometry[index].type != (int32_t)(enumValue))                 \
        {                                                                                          \
            M2_ASSERT(false);                                                                      \
            return zero;                                                                           \
        }                                                                                          \
        return world->shapeGeometry[index].field;                                                  \
    }

M2_GEOMETRY_GETTER(m2Shape_GetCircle, m2Circle, circle, m2_circleShape)
M2_GEOMETRY_GETTER(m2Shape_GetCapsule, m2Capsule, capsule, m2_capsuleShape)
M2_GEOMETRY_GETTER(m2Shape_GetPolygon, m2Polygon, polygon, m2_polygonShape)
M2_GEOMETRY_GETTER(m2Shape_GetSegment, m2Segment, segment, m2_segmentShape)
M2_GEOMETRY_GETTER(m2Shape_GetChainSegment, m2ChainSegment, chainSegment, m2_chainSegmentShape)
#undef M2_GEOMETRY_GETTER

float m2Shape_GetDensity(m2ShapeId shapeId)
{
    m2World* world = WorldFromIndex(shapeId.world0);
    int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return 0.0f;
    }
    return world->shapeDensity[index];
}

m2Vec2 m2Body_GetLocalCenter(m2BodyId bodyId)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        m2Vec2 zero = {0.0f, 0.0f};
        return zero;
    }
    return world->localCenters[index];
}

bool m2Body_IsBullet(m2BodyId bodyId)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return false;
    }
    return world->bullets[index] != 0;
}

float m2Body_GetGravityScale(m2BodyId bodyId)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t index = world != NULL ? BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        M2_ASSERT(false);
        return 0.0f;
    }
    return world->gravityScales[index];
}

int32_t m2Chain_GetShapes(m2ChainId chainId, m2ShapeId* ids, int32_t capacity)
{
    m2World* world = WorldFromIndex(chainId.world0);
    int32_t chainIndex = world != NULL ? ChainSlot(world, chainId) : -1;
    if (chainIndex < 0)
    {
        return 0;
    }
    int32_t total = 0;
    for (int32_t i = 0; i < world->maxShapeIndex; ++i)
    {
        if (world->shapeAlive[i] == 0 || world->shapeChain[i] != chainIndex)
        {
            continue;
        }
        if (ids != NULL && total < capacity)
        {
            ids[total] = MakeShapeId(world, i);
        }
        total += 1;
    }
    return total;
}

int32_t m2Body_GetShapes(m2BodyId bodyId, m2ShapeId* ids, int32_t capacity)
{
    m2World* world = GetBodyWorld(bodyId);
    int32_t bodyIndex = world != NULL ? BodySlot(world, bodyId) : -1;
    if (bodyIndex < 0)
    {
        return 0;
    }
    int32_t total = 0;
    for (int32_t i = 0; i < world->maxShapeIndex; ++i)
    {
        if (world->shapeAlive[i] == 0 || world->shapeBody[i] != bodyIndex)
        {
            continue;
        }
        if (ids != NULL && total < capacity)
        {
            ids[total] = MakeShapeId(world, i);
        }
        total += 1;
    }
    return total;
}

// --- Fluids: storage surface (the solver arrives in later slices) ------------------

static int32_t ParticleSlot(const m2World* world, m2ParticleId id)
{
    int32_t index = id.index1 - 1;
    if (world == NULL || index < 0 || index >= world->particleCapacity ||
        world->particleAlive[index] == 0 || world->particleGenerations[index] != id.generation)
    {
        return -1;
    }
    return index;
}

m2ParticleId m2World_EmitParticle(m2WorldId worldId, m2Pos2 position, m2Vec2 velocity,
                                  uint32_t flags)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL || world->particleCapacity == 0)
    {
        M2_ASSERT(false); // no particle system in this world: misuse
        return m2_nullParticleId;
    }
    if (!(position.x == position.x && position.y == position.y && velocity.x == velocity.x &&
          velocity.y == velocity.y))
    {
        M2_ASSERT(false); // NaN screen, the def-validation law
        return m2_nullParticleId;
    }
    if (world->particleFreeCount == 0)
    {
        // A full pool is a runtime fact, not misuse: pace emitters
        // off m2World_GetParticleCount; the counter keeps the score.
        world->particlePoolFullCount += 1;
        return m2_nullParticleId;
    }
    int32_t index = world->particleFreeQueue[world->particleFreeHead];
    world->particleFreeHead = (world->particleFreeHead + 1) % world->particleCapacity;
    world->particleFreeCount -= 1;
    if (index + 1 > world->maxParticleIndex)
    {
        world->maxParticleIndex = index + 1;
    }
    world->particlePositions[index] = position;
    world->particleVelocities[index] = velocity;
    world->particleFlags[index] = flags;
    world->particleLifetime[index] = 0.0f;
    world->particleUserData[index] = 0;
    world->particleAlive[index] = 1;
    world->particleCount += 1;
    m2ParticleId id = {index + 1, worldId.index1, world->particleGenerations[index]};
    if (world->journalActive != 0)
    {
        struct
        {
            m2Pos2 position;
            m2Vec2 velocity;
            uint32_t flags;
            m2ParticleId expected;
        } record;
        memset(&record, 0, sizeof(record));
        record.position = position;
        record.velocity = velocity;
        record.flags = flags;
        record.expected = id;
        m2JournalRecord(world, m2_opEmitParticle, &record, (int32_t)sizeof(record));
    }
    return id;
}

void m2World_DestroyParticle(m2ParticleId particleId)
{
    m2World* world = WorldFromIndex(particleId.world0);
    int32_t index = ParticleSlot(world, particleId);
    if (index < 0)
    {
        M2_ASSERT(false);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2ParticleId id;
        } record;
        memset(&record, 0, sizeof(record));
        record.id = particleId;
        m2JournalRecord(world, m2_opDestroyParticle, &record, (int32_t)sizeof(record));
    }
    world->particleAlive[index] = 0;
    world->particleGenerations[index] += 1; // retire under a fresh generation
    // Jelly bookkeeping: springs and triads die with their particle,
    // compacted in order so the lists stay canonical.
    if (world->particleSpringCount > 0)
    {
        int32_t keep = 0;
        for (int32_t k = 0; k < world->particleSpringCount; ++k)
        {
            if (world->particleSpringA[k] == index || world->particleSpringB[k] == index)
            {
                continue;
            }
            world->particleSpringA[keep] = world->particleSpringA[k];
            world->particleSpringB[keep] = world->particleSpringB[k];
            world->particleSpringRest[keep] = world->particleSpringRest[k];
            keep += 1;
        }
        world->particleSpringCount = keep;
    }
    if (world->particleTriadCount > 0)
    {
        int32_t keep = 0;
        for (int32_t k = 0; k < world->particleTriadCount; ++k)
        {
            if (world->particleTriadA[k] == index || world->particleTriadB[k] == index ||
                world->particleTriadC[k] == index)
            {
                continue;
            }
            world->particleTriadA[keep] = world->particleTriadA[k];
            world->particleTriadB[keep] = world->particleTriadB[k];
            world->particleTriadC[keep] = world->particleTriadC[k];
            world->particleTriadPA[keep] = world->particleTriadPA[k];
            world->particleTriadPB[keep] = world->particleTriadPB[k];
            world->particleTriadPC[keep] = world->particleTriadPC[k];
            keep += 1;
        }
        world->particleTriadCount = keep;
    }
    world->particleFreeQueue[(world->particleFreeHead + world->particleFreeCount) %
                             world->particleCapacity] = index;
    world->particleFreeCount += 1;
    world->particleCount -= 1;
}

bool m2Particle_IsValid(m2ParticleId particleId)
{
    m2World* world = WorldFromIndex(particleId.world0);
    return ParticleSlot(world, particleId) >= 0;
}

m2Pos2 m2Particle_GetPosition(m2ParticleId particleId)
{
    m2Pos2 zero = {0.0, 0.0};
    m2World* world = WorldFromIndex(particleId.world0);
    int32_t index = ParticleSlot(world, particleId);
    return index >= 0 ? world->particlePositions[index] : zero;
}

m2Vec2 m2Particle_GetVelocity(m2ParticleId particleId)
{
    m2Vec2 zero = {0.0f, 0.0f};
    m2World* world = WorldFromIndex(particleId.world0);
    int32_t index = ParticleSlot(world, particleId);
    return index >= 0 ? world->particleVelocities[index] : zero;
}

void m2Particle_SetVelocity(m2ParticleId particleId, m2Vec2 velocity)
{
    m2World* world = WorldFromIndex(particleId.world0);
    int32_t index = ParticleSlot(world, particleId);
    if (index < 0 || !(velocity.x == velocity.x && velocity.y == velocity.y))
    {
        M2_ASSERT(false);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2ParticleId id;
            m2Vec2 velocity;
        } record;
        memset(&record, 0, sizeof(record));
        record.id = particleId;
        record.velocity = velocity;
        m2JournalRecord(world, m2_opSetParticleVelocity, &record, (int32_t)sizeof(record));
    }
    world->particleVelocities[index] = velocity;
}

uint32_t m2Particle_GetFlags(m2ParticleId particleId)
{
    m2World* world = WorldFromIndex(particleId.world0);
    int32_t index = ParticleSlot(world, particleId);
    return index >= 0 ? world->particleFlags[index] : 0;
}

void m2Particle_SetLifetime(m2ParticleId particleId, float seconds)
{
    m2World* world = WorldFromIndex(particleId.world0);
    int32_t index = ParticleSlot(world, particleId);
    if (index < 0 || !(seconds >= 0.0f))
    {
        M2_ASSERT(false);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2ParticleId id;
            float seconds;
        } record;
        memset(&record, 0, sizeof(record));
        record.id = particleId;
        record.seconds = seconds;
        m2JournalRecord(world, m2_opSetParticleLifetime, &record, (int32_t)sizeof(record));
    }
    world->particleLifetime[index] = seconds;
}

float m2Particle_GetLifetime(m2ParticleId particleId)
{
    m2World* world = WorldFromIndex(particleId.world0);
    int32_t index = ParticleSlot(world, particleId);
    return index >= 0 ? world->particleLifetime[index] : 0.0f;
}

void m2Particle_SetUserData(m2ParticleId particleId, uint64_t userData)
{
    m2World* world = WorldFromIndex(particleId.world0);
    int32_t index = ParticleSlot(world, particleId);
    if (index < 0)
    {
        M2_ASSERT(false);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2ParticleId id;
            uint64_t userData;
        } record;
        memset(&record, 0, sizeof(record));
        record.id = particleId;
        record.userData = userData;
        m2JournalRecord(world, m2_opSetParticleUserData, &record, (int32_t)sizeof(record));
    }
    world->particleUserData[index] = userData;
}

uint64_t m2Particle_GetUserData(m2ParticleId particleId)
{
    m2World* world = WorldFromIndex(particleId.world0);
    int32_t index = ParticleSlot(world, particleId);
    return index >= 0 ? world->particleUserData[index] : 0;
}

int32_t m2World_GetParticleCount(m2WorldId worldId)
{
    m2World* world = GetWorld(worldId);
    return world != NULL ? world->particleCount : 0;
}

int32_t m2World_GetParticles(m2WorldId worldId, m2ParticleId* ids, int32_t capacity)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL)
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
        if (ids != NULL && total < capacity)
        {
            ids[total] = (m2ParticleId){i + 1, worldId.index1, world->particleGenerations[i]};
        }
        total += 1;
    }
    return total;
}

// Spring-named getter aliases: the setters say Spring, the readers
// now can too. Same slots, same validation.
float m2Joint_GetSpringHertz(m2JointId jointId)
{
    return m2Joint_GetHertz(jointId);
}

float m2Joint_GetSpringDampingRatio(m2JointId jointId)
{
    return m2Joint_GetDampingRatio(jointId);
}

float m2Joint_GetAngularSpringHertz(m2JointId jointId)
{
    return m2Joint_GetAngularHertz(jointId);
}

float m2Joint_GetAngularSpringDampingRatio(m2JointId jointId)
{
    return m2Joint_GetAngularDampingRatio(jointId);
}

// --- Shatter: the destruction road ------------------------------------------------

int32_t m2World_ShatterBody(m2BodyId bodyId, const m2Polygon* pieces, int32_t pieceCount,
                            m2BodyId* outBodies, int32_t capacity)
{
    m2World* world = WorldFromIndex(bodyId.world0);
    int32_t parent = world != NULL ? BodySlot(world, bodyId) : -1;
    if (parent < 0 || pieces == NULL || pieceCount < 1 || pieceCount > 64 ||
        world->types[parent] != (uint8_t)m2_dynamicBody)
    {
        M2_ASSERT(false);
        return 0;
    }
    for (int32_t i = 0; i < pieceCount; ++i)
    {
        if (pieces[i].count < 3)
        {
            M2_ASSERT(false);
            return 0;
        }
    }
    if (world->freeCount < pieceCount || world->shapeFreeCount < pieceCount)
    {
        return 0; // cannot seat every piece: a runtime fact, all or nothing
    }

    // The parent's rigid field, sampled before anything moves.
    m2Transform xf = world->transforms[parent];
    m2Vec2 vParent = world->linearVelocities[parent];
    float wParent = world->angularVelocities[parent];
    m2Vec2 lcParent = world->localCenters[parent];
    m2Vec2 comArm = {xf.q.c * lcParent.x - xf.q.s * lcParent.y,
                     xf.q.s * lcParent.x + xf.q.c * lcParent.y};

    // Materials and filter ride from the parent's first shape; a
    // shapeless parent hands out defaults (documented).
    m2ShapeDef pieceShape = m2DefaultShapeDef();
    int32_t firstShape = world->bodyShapeHead[parent];
    if (firstShape != -1)
    {
        pieceShape.density = world->shapeDensity[firstShape];
        pieceShape.friction = world->shapeFriction[firstShape];
        pieceShape.restitution = world->shapeRestitution[firstShape];
        pieceShape.tangentSpeed = world->shapeTangentSpeed[firstShape];
        pieceShape.categoryBits = world->shapeCategory[firstShape];
        pieceShape.maskBits = world->shapeMask[firstShape];
        pieceShape.groupIndex = world->shapeGroup[firstShape];
    }

    // Ids carry the 1-based world index; the generation lives in
    // the registry beside the world itself.
    m2WorldId worldId = {bodyId.world0, s_worldGenerations[bodyId.world0 - 1]};

    uint8_t journalWasActive = world->journalActive;
    world->journalActive = 0;

    int32_t firstIndex1 = 0;
    for (int32_t i = 0; i < pieceCount; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = xf.p;
        bd.rotation = xf.q;
        bd.gravityScale = world->gravityScales[parent];
        bd.linearDamping = world->linearDampings[parent];
        bd.angularDamping = world->angularDampings[parent];
        m2BodyId piece = m2CreateBody(worldId, &bd);
        m2CreatePolygonShape(piece, &pieceShape, &pieces[i]);
        int32_t pieceIndex = piece.index1 - 1;
        // The rigid field at this piece's own center of mass.
        m2Vec2 lc = world->localCenters[pieceIndex];
        m2Vec2 arm = {xf.q.c * lc.x - xf.q.s * lc.y, xf.q.s * lc.x + xf.q.c * lc.y};
        float rx = arm.x - comArm.x;
        float ry = arm.y - comArm.y;
        world->linearVelocities[pieceIndex] =
            (m2Vec2){vParent.x - wParent * ry, vParent.y + wParent * rx};
        world->angularVelocities[pieceIndex] = wParent;
        if (i == 0)
        {
            firstIndex1 = piece.index1;
        }
        if (outBodies != NULL && i < capacity)
        {
            outBodies[i] = piece;
        }
    }
    m2DestroyBody(bodyId); // joints die with it, touching sleepers wake

    world->journalActive = journalWasActive;
    m2JournalRecordShatter(world, bodyId, pieces, pieceCount, firstIndex1);
    return pieceCount;
}

// Subsystem hashes: independent seeds on purpose (the total is not
// a function of the parts), each loop mirroring the gated hash's
// coverage for its slice of the world.
m2WorldHashParts m2World_HashParts(m2WorldId worldId)
{
    m2WorldHashParts parts;
    memset(&parts, 0, sizeof(parts));
    m2World* world = GetWorld(worldId);
    if (world == NULL)
    {
        return parts;
    }

    uint64_t h = M2_HASH_INIT;
    h = m2Hash64(h, &world->stepCount, (int32_t)sizeof(world->stepCount));
    h = m2Hash64(h, &world->gravity, (int32_t)sizeof(world->gravity));
    parts.world = h;

    h = M2_HASH_INIT;
    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        if (world->alive[i] == 0)
        {
            continue;
        }
        h = m2Hash64(h, &world->transforms[i], (int32_t)sizeof(m2Transform));
        h = m2Hash64(h, &world->linearVelocities[i], (int32_t)sizeof(m2Vec2));
        h = m2Hash64(h, &world->angularVelocities[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->invMass[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->invInertia[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->localCenters[i], (int32_t)sizeof(m2Vec2));
        h = m2Hash64(h, &world->types[i], (int32_t)sizeof(uint8_t));
        h = m2Hash64(h, &world->asleep[i], (int32_t)sizeof(uint8_t));
        h = m2Hash64(h, &world->sleepTimes[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->sleepStreak[i], 1);
        h = m2Hash64(h, &world->bullets[i], (int32_t)sizeof(uint8_t));
    }
    parts.bodies = h;

    h = M2_HASH_INIT;
    h = m2Hash64(h, world->pairKeys, world->pairCount * (int32_t)sizeof(uint64_t));
    h = m2Hash64(h, world->manifolds, world->pairCount * (int32_t)sizeof(m2Manifold));
    parts.contacts = h;

    h = M2_HASH_INIT;
    for (int32_t i = 0; i < world->maxJointIndex; ++i)
    {
        if (world->jointAlive[i] == 0)
        {
            continue;
        }
        h = m2Hash64(h, &world->jointImpulse[i], (int32_t)sizeof(m2Vec2));
        h = m2Hash64(h, &world->jointMotorImpulse[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->jointLowerImpulse[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->jointUpperImpulse[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->jointSpringImpulse[i], (int32_t)sizeof(float));
    }
    parts.joints = h;

    h = M2_HASH_INIT;
    if (world->particleCapacity > 0)
    {
        h = m2Hash64(h, &world->particleCount, (int32_t)sizeof(int32_t));
        for (int32_t i = 0; i < world->maxParticleIndex; ++i)
        {
            h = m2Hash64(h, &world->particleAlive[i], 1);
            if (world->particleAlive[i] == 0)
            {
                continue;
            }
            h = m2Hash64(h, &world->particlePositions[i], (int32_t)sizeof(m2Pos2));
            h = m2Hash64(h, &world->particleVelocities[i], (int32_t)sizeof(m2Vec2));
            h = m2Hash64(h, &world->particleFlags[i], (int32_t)sizeof(uint32_t));
            h = m2Hash64(h, &world->particleLifetime[i], (int32_t)sizeof(float));
            h = m2Hash64(h, &world->particleUserData[i], (int32_t)sizeof(uint64_t));
        }
        h = m2Hash64(h, &world->particleSpringCount, (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particleSpringA,
                     world->particleSpringCount * (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particleSpringB,
                     world->particleSpringCount * (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particleSpringRest,
                     world->particleSpringCount * (int32_t)sizeof(float));
        h = m2Hash64(h, &world->particleTriadCount, (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particleTriadA,
                     world->particleTriadCount * (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particleTriadB,
                     world->particleTriadCount * (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particleTriadC,
                     world->particleTriadCount * (int32_t)sizeof(int32_t));
    }
    parts.particles = h;

    return parts;
}

// The invariant walk: everything a healthy world must be able to
// say about itself, checked loudly. Pure reader.
bool m2World_Validate(m2WorldId worldId)
{
    m2World* world = GetWorld(worldId);
    if (world == NULL)
    {
        M2_ASSERT(false);
        return false;
    }
#define M2_CHECK_INVARIANT(cond)                                                                   \
    do                                                                                             \
    {                                                                                              \
        if (!(cond))                                                                               \
        {                                                                                          \
            M2_ASSERT(false);                                                                      \
            return false;                                                                          \
        }                                                                                          \
    } while (0)

    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        if (world->alive[i] == 0)
        {
            continue;
        }
        m2Transform xf = world->transforms[i];
        M2_CHECK_INVARIANT(xf.p.x == xf.p.x && xf.p.y == xf.p.y);
        M2_CHECK_INVARIANT(xf.q.c == xf.q.c && xf.q.s == xf.q.s);
        m2Vec2 v = world->linearVelocities[i];
        M2_CHECK_INVARIANT(v.x == v.x && v.y == v.y);
        M2_CHECK_INVARIANT(world->angularVelocities[i] == world->angularVelocities[i]);
        M2_CHECK_INVARIANT(world->types[i] <= 2);
    }
    for (int32_t i = 0; i < world->maxShapeIndex; ++i)
    {
        if (world->shapeAlive[i] == 0)
        {
            continue;
        }
        int32_t body = world->shapeBody[i];
        M2_CHECK_INVARIANT(body >= 0 && body < world->bodyCapacity && world->alive[body] != 0);
    }
    for (int32_t i = 0; i < world->maxJointIndex; ++i)
    {
        if (world->jointAlive[i] == 0)
        {
            continue;
        }
        M2_CHECK_INVARIANT(world->jointType[i] <= 10);
        int32_t a = world->jointBodyA[i];
        int32_t b = world->jointBodyB[i];
        M2_CHECK_INVARIANT(a >= 0 && a < world->bodyCapacity && world->alive[a] != 0);
        M2_CHECK_INVARIANT(b >= 0 && b < world->bodyCapacity && world->alive[b] != 0);
    }
    for (int32_t i = 1; i < world->pairCount; ++i)
    {
        // The canonical ordering law, checked where it lives.
        M2_CHECK_INVARIANT(world->pairKeys[i - 1] < world->pairKeys[i]);
    }
    if (world->particleCapacity > 0)
    {
        int32_t alive = 0;
        for (int32_t i = 0; i < world->maxParticleIndex; ++i)
        {
            if (world->particleAlive[i] == 0)
            {
                continue;
            }
            alive += 1;
            m2Pos2 p = world->particlePositions[i];
            M2_CHECK_INVARIANT(p.x == p.x && p.y == p.y);
            m2Vec2 v = world->particleVelocities[i];
            M2_CHECK_INVARIANT(v.x == v.x && v.y == v.y);
        }
        M2_CHECK_INVARIANT(alive == world->particleCount);
        for (int32_t k = 0; k < world->particleSpringCount; ++k)
        {
            M2_CHECK_INVARIANT(world->particleAlive[world->particleSpringA[k]] != 0 &&
                               world->particleAlive[world->particleSpringB[k]] != 0);
        }
        for (int32_t k = 0; k < world->particleTriadCount; ++k)
        {
            M2_CHECK_INVARIANT(world->particleAlive[world->particleTriadA[k]] != 0 &&
                               world->particleAlive[world->particleTriadB[k]] != 0 &&
                               world->particleAlive[world->particleTriadC[k]] != 0);
        }
    }
#undef M2_CHECK_INVARIANT
    return true;
}
