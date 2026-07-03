// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Islands and sleeping (topic-06). v1 builds islands each step with a
// deterministic union-find over touching contacts in canonical pair
// order - persistence (topic-06 D2) is a recorded optimization for
// later; rebuilding from canonical inputs is history-free, so island
// structure needs no snapshot blocks. What DOES persist - and is
// snapshot state and hashed - is per-body sleep: the asleep flag and
// the sleep timer.
//
// Wake rules executed here (amended topic-06 §3): any awake member
// wakes its whole island; a moving kinematic touching the island
// disturbs it (the RT1-STAB-1 mechanism end to end); API setters wake
// their body directly, and the island coupling spreads it next step.

#include "world_internal.h"

#include "maul2d/base.h"

#define M2_SLEEP_LINEAR_TOLERANCE  0.05f // m/s (F-T6-2: harness-tuned later)
#define M2_SLEEP_ANGULAR_TOLERANCE 0.12f // rad/s
#define M2_TIME_TO_SLEEP           0.5f  // seconds under tolerance

static int32_t Find(int32_t* parent, int32_t i)
{
    while (parent[i] != i)
    {
        parent[i] = parent[parent[i]]; // halving; deterministic
        i = parent[i];
    }
    return i;
}

static void Union(int32_t* parent, int32_t a, int32_t b)
{
    int32_t ra = Find(parent, a);
    int32_t rb = Find(parent, b);
    // Min-root union: canonical regardless of processing order.
    if (ra < rb)
    {
        parent[rb] = ra;
    }
    else if (rb < ra)
    {
        parent[ra] = rb;
    }
}

static bool BodySlow(const m2World* world, int32_t i)
{
    m2Vec2 v = world->linearVelocities[i];
    float w = world->angularVelocities[i];
    return v.x * v.x + v.y * v.y < M2_SLEEP_LINEAR_TOLERANCE * M2_SLEEP_LINEAR_TOLERANCE &&
           w > -M2_SLEEP_ANGULAR_TOLERANCE && w < M2_SLEEP_ANGULAR_TOLERANCE;
}

// Pre-solve: build islands over touching contacts, wake islands that
// have any awake member or a moving-kinematic toucher.
void m2UpdateIslandsAndWake(m2World* world)
{
    int32_t* parent = world->islandParent;
    uint8_t* disturbed = world->islandDisturbed;
    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        parent[i] = i;
        disturbed[i] = 0;
    }

    // Union dynamic bodies over touching contacts (canonical order);
    // note moving-kinematic touches as disturbances.
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        if (world->manifolds[i].pointCount == 0)
        {
            continue;
        }
        int32_t bodyA = world->shapeBody[(int32_t)(world->pairKeys[i] >> 32)];
        int32_t bodyB = world->shapeBody[(int32_t)(world->pairKeys[i] & 0xFFFFFFFFu)];
        bool dynA = world->types[bodyA] == (uint8_t)m2_dynamicBody;
        bool dynB = world->types[bodyB] == (uint8_t)m2_dynamicBody;
        if (dynA && dynB)
        {
            Union(parent, bodyA, bodyB);
        }
        else if (dynA || dynB)
        {
            int32_t dynamic = dynA ? bodyA : bodyB;
            int32_t other = dynA ? bodyB : bodyA;
            if (world->types[other] == (uint8_t)m2_kinematicBody && !BodySlow(world, other))
            {
                disturbed[dynamic] = 1;
            }
        }
    }

    // Joints connect islands exactly like touching contacts do.
    for (int32_t j = 0; j < world->maxJointIndex; ++j)
    {
        if (world->jointAlive[j] == 0)
        {
            continue;
        }
        int32_t bodyA = world->jointBodyA[j];
        int32_t bodyB = world->jointBodyB[j];
        if (world->types[bodyA] == (uint8_t)m2_dynamicBody &&
            world->types[bodyB] == (uint8_t)m2_dynamicBody)
        {
            Union(parent, bodyA, bodyB);
        }
    }

    // Fold member-awake and disturbance flags up to the roots.
    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        if (world->alive[i] == 0 || world->types[i] != (uint8_t)m2_dynamicBody)
        {
            continue;
        }
        int32_t root = Find(parent, i);
        if (world->asleep[i] == 0 || disturbed[i] != 0)
        {
            disturbed[root] = 2; // root marker: island must be awake
        }
    }

    // Wake every member of awake islands (fixed body order).
    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        if (world->alive[i] == 0 || world->types[i] != (uint8_t)m2_dynamicBody)
        {
            continue;
        }
        if (disturbed[Find(parent, i)] == 2 && world->asleep[i] != 0)
        {
            world->asleep[i] = 0;
            world->sleepTimes[i] = 0.0f;
        }
    }
}

// Post-solve: island-coupled sleep accounting. An island sleeps only
// when every member has stayed under tolerance for the full window; one
// fast member resets the whole island (topic-06 D3).
void m2UpdateSleep(m2World* world, float dt)
{
    int32_t* parent = world->islandParent;
    uint8_t* islandFast = world->islandDisturbed; // reuse scratch

    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        islandFast[i] = 0;
    }
    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        if (world->alive[i] == 0 || world->types[i] != (uint8_t)m2_dynamicBody ||
            world->asleep[i] != 0)
        {
            continue;
        }
        if (!BodySlow(world, i))
        {
            islandFast[Find(parent, i)] = 1;
        }
    }

    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        if (world->alive[i] == 0 || world->types[i] != (uint8_t)m2_dynamicBody ||
            world->asleep[i] != 0)
        {
            continue;
        }
        if (islandFast[Find(parent, i)] != 0)
        {
            world->sleepTimes[i] = 0.0f;
            continue;
        }
        world->sleepTimes[i] += dt;
    }

    // Island sleeps when its minimum timer crosses the window. Two
    // passes keep it order-canonical: find sleepy roots, then apply.
    uint8_t* rootSleeps = islandFast; // reuse again: 1 = candidate
    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        rootSleeps[i] = 0;
    }
    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        if (world->alive[i] == 0 || world->types[i] != (uint8_t)m2_dynamicBody ||
            world->asleep[i] != 0)
        {
            continue;
        }
        int32_t root = Find(parent, i);
        if (rootSleeps[root] == 0)
        {
            rootSleeps[root] = 1; // assume sleepy until a member objects
        }
        if (world->sleepTimes[i] < M2_TIME_TO_SLEEP)
        {
            rootSleeps[root] = 2; // objection: someone is not ready
        }
    }
    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        if (world->alive[i] == 0 || world->types[i] != (uint8_t)m2_dynamicBody ||
            world->asleep[i] != 0)
        {
            continue;
        }
        if (rootSleeps[Find(parent, i)] == 1)
        {
            world->asleep[i] = 1;
            world->linearVelocities[i] = (m2Vec2){0.0f, 0.0f};
            world->angularVelocities[i] = 0.0f;
        }
    }
}
