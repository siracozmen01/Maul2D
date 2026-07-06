// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Fluid gate (storage slice): emit/destroy lifecycle with FIFO slot
// recycling under fresh generations, capacity overflow as a quiet
// fact, free-fall integration, rollback identity over particle
// state, journal replay of the particle ops, and the churn hash
// compared across CI cells as the 16th gated line.

#include "world_internal.h"

#include "maul2d/base.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int s_failures = 0;

#define CHECK(cond, msg)                                                                           \
    do                                                                                             \
    {                                                                                              \
        if (!(cond))                                                                               \
        {                                                                                          \
            printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__);                                 \
            s_failures += 1;                                                                       \
        }                                                                                          \
    } while (0)

static m2WorldDef FluidWorldDef(int32_t particleCapacity)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.jointCapacity = 4;
    def.particleCapacity = particleCapacity;
    return def;
}

static void TestLifecycle(void)
{
    m2WorldDef def = FluidWorldDef(64);
    m2WorldId world = m2CreateWorld(&def);

    // Emit a small grid; count and enumeration agree.
    m2ParticleId ids[64];
    for (int32_t i = 0; i < 20; ++i)
    {
        double x = (double)(i % 5) * 0.1;
        double y = 4.0 + (double)(i / 5) * 0.1;
        ids[i] = m2World_EmitParticle(world, (m2Pos2){x, y}, (m2Vec2){0.0f, 0.0f});
        CHECK(m2Particle_IsValid(ids[i]), "each emitted particle is live");
    }
    CHECK(m2World_GetParticleCount(world) == 20, "the count sees every emit");
    m2ParticleId listed[64];
    int32_t total = m2World_GetParticles(world, listed, 64);
    CHECK(total == 20, "enumeration total is truthful");
    CHECK(listed[0].index1 == ids[0].index1, "ascending slot order starts at the first emit");

    // Free fall: gravity moves every particle down, nothing else.
    m2Pos2 before = m2Particle_GetPosition(ids[7]);
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2Pos2 after = m2Particle_GetPosition(ids[7]);
    CHECK(after.y < before.y - 0.5, "particles free-fall under gravity");
    CHECK(after.x == before.x, "no lateral force exists yet");
    CHECK(m2Particle_GetVelocity(ids[7]).y < -4.0f, "velocity integrates");

    // Destroy one third; slots recycle FIFO under fresh generations.
    for (int32_t i = 0; i < 20; i += 3)
    {
        m2World_DestroyParticle(ids[i]);
    }
    CHECK(m2World_GetParticleCount(world) == 13, "destroys drop the count");
    CHECK(!m2Particle_IsValid(ids[0]), "a destroyed id goes stale");
    // FIFO promise (the round-7 lesson): freed slots return at the
    // END of the queue, never next; drain the 44 untouched slots and
    // the 45th emit lands on the first freed slot, fresh generation.
    m2ParticleId reborn = m2_nullParticleId;
    for (int32_t i = 0; i < 45; ++i)
    {
        reborn = m2World_EmitParticle(world, (m2Pos2){0.0, 9.0}, (m2Vec2){0.0f, 0.0f});
    }
    CHECK(m2Particle_IsValid(reborn), "the pool recycles");
    CHECK(reborn.index1 == ids[0].index1 && reborn.generation == ids[0].generation + 1,
          "the first freed slot returns last, under a fresh generation");
    CHECK(!m2Particle_IsValid(ids[0]), "the old id stays stale after rebirth");

    // Retargeting velocity is immediate (an isolated particle: the
    // coincident tower above is exactly where the velocity-limit law
    // dominates, by design).
    m2ParticleId lone = m2World_EmitParticle(world, (m2Pos2){20.0, 9.0}, (m2Vec2){0.0f, 0.0f});
    m2Particle_SetVelocity(lone, (m2Vec2){3.0f, 0.0f});
    m2World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m2Particle_GetPosition(lone).x > 20.04, "a set velocity moves the particle");

    m2DestroyWorld(world);
}

static void TestOverflowIsQuiet(void)
{
    m2WorldDef def = FluidWorldDef(8);
    m2WorldId world = m2CreateWorld(&def);
    int32_t granted = 0;
    for (int32_t i = 0; i < 12; ++i)
    {
        m2ParticleId id =
            m2World_EmitParticle(world, (m2Pos2){(double)i, 1.0}, (m2Vec2){0.0f, 0.0f});
        granted += id.index1 != 0 ? 1 : 0;
    }
    CHECK(granted == 8, "a full pool grants exactly its capacity");
    CHECK(m2World_GetParticleCount(world) == 8, "overflow leaves the count at capacity");
    m2DestroyWorld(world);
}

static void TestRollbackIdentity(void)
{
    m2WorldDef def = FluidWorldDef(128);
    m2WorldId world = m2CreateWorld(&def);
    for (int32_t i = 0; i < 60; ++i)
    {
        double x = (double)(i % 10) * 0.11;
        double y = 3.0 + (double)(i / 10) * 0.11;
        m2World_EmitParticle(world, (m2Pos2){x, y}, (m2Vec2){0.1f, 0.0f});
    }
    for (int32_t i = 0; i < 10; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    int32_t size = m2World_SnapshotSize(world);
    void* snap = malloc((size_t)size);
    CHECK(m2World_Snapshot(world, snap, size) == size, "snapshot writes its full size");
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    uint64_t first = m2World_Hash(world);
    CHECK(m2World_Restore(world, snap, size), "restore accepts its own bytes");
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    uint64_t second = m2World_Hash(world);
    CHECK(first == second, "the fluid timeline replays bit-exactly after rollback");
    free(snap);
    m2DestroyWorld(world);
}

static void TestJournalReplay(void)
{
    m2WorldDef def = FluidWorldDef(64);
    m2WorldId world = m2CreateWorld(&def);
    int32_t cap = 1 << 20;
    void* tape = malloc((size_t)cap);
    CHECK(m2World_StartJournal(world, tape, cap), "journal starts");

    m2ParticleId ids[32];
    for (int32_t i = 0; i < 16; ++i)
    {
        ids[i] = m2World_EmitParticle(world, (m2Pos2){(double)i * 0.09, 5.0}, (m2Vec2){0.0f, 0.0f});
    }
    for (int32_t i = 0; i < 20; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2Particle_SetVelocity(ids[3], (m2Vec2){-2.0f, 1.0f});
    m2World_DestroyParticle(ids[8]);
    m2World_DestroyParticle(ids[9]);
    m2World_EmitParticle(world, (m2Pos2){0.5, 8.0}, (m2Vec2){0.0f, -1.0f});
    for (int32_t i = 0; i < 20; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    int32_t used = m2World_StopJournal(world);
    CHECK(used > 0, "journal closed with bytes");
    uint64_t recorded = m2World_Hash(world);

    m2WorldId fresh = m2CreateWorld(&def);
    CHECK(m2World_ReplayJournal(fresh, tape, used), "replay accepts the tape");
    CHECK(m2World_Hash(fresh) == recorded, "the particle session replays onto the same bits");

    m2DestroyWorld(fresh);
    m2DestroyWorld(world);
    free(tape);
}

// The relaxation acceptance: a dense blob spreads toward rest
// spacing without exploding, the velocity limit holds, and the
// system calms down; viscosity kills tangential shear that plain
// water keeps.
static void TestRelaxation(void)
{
    m2WorldDef def = FluidWorldDef(64);
    def.gravity = (m2Vec2){0.0f, 0.0f};
    m2WorldId world = m2CreateWorld(&def);
    m2ParticleId blob[25];
    for (int32_t i = 0; i < 25; ++i)
    {
        double x = (double)(i % 5) * 0.04;
        double y = (double)(i / 5) * 0.04;
        blob[i] = m2World_EmitParticle(world, (m2Pos2){x, y}, (m2Vec2){0.0f, 0.0f});
    }
    float peakSpeed = 0.0f;
    for (int32_t step = 0; step < 120; ++step)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        if (step == 4)
        {
            for (int32_t i = 0; i < 25; ++i)
            {
                m2Vec2 v = m2Particle_GetVelocity(blob[i]);
                float speed = sqrtf(v.x * v.x + v.y * v.y);
                peakSpeed = peakSpeed > speed ? peakSpeed : speed;
            }
        }
    }
    double spreadSq = 0.0;
    float lateSpeed = 0.0f;
    for (int32_t i = 0; i < 25; ++i)
    {
        m2Pos2 p = m2Particle_GetPosition(blob[i]);
        CHECK(p.x == p.x && p.y == p.y, "positions stay finite");
        double dx = p.x - 0.08;
        double dy = p.y - 0.08;
        double d2 = dx * dx + dy * dy;
        spreadSq = d2 > spreadSq ? d2 : spreadSq;
        m2Vec2 v = m2Particle_GetVelocity(blob[i]);
        float speed = sqrtf(v.x * v.x + v.y * v.y);
        lateSpeed = lateSpeed > speed ? lateSpeed : speed;
    }
    CHECK(spreadSq > 0.013, "pressure spreads the blob past its packed extent");
    CHECK(peakSpeed > 0.0f, "the packed blob actually moves");
    // Free particles in zero gravity coast (walls arrive with the
    // rigid coupling slice); the promise here is the speed law.
    CHECK(peakSpeed <= 6.001f && lateSpeed <= 6.001f, "no speed ever beats one diameter per step");

    // Approach damping: two particles closing at 2 m/s inside one
    // diameter lose closing speed in a single step but do not stick.
    m2ParticleId left = m2World_EmitParticle(world, (m2Pos2){5.0, 5.0}, (m2Vec2){1.0f, 0.0f});
    m2ParticleId right = m2World_EmitParticle(world, (m2Pos2){5.08, 5.0}, (m2Vec2){-1.0f, 0.0f});
    m2World_Step(world, 1.0f / 60.0f, 4);
    float closing = m2Particle_GetVelocity(left).x - m2Particle_GetVelocity(right).x;
    CHECK(closing < 1.7f, "damping eats approach speed");
    CHECK(closing > 0.0f, "damping does not reverse the approach by itself");

    // The stability law: nothing crosses one diameter per step.
    m2ParticleId bullet = m2World_EmitParticle(world, (m2Pos2){3.0, 3.0}, (m2Vec2){1000.0f, 0.0f});
    m2World_Step(world, 1.0f / 60.0f, 4);
    m2Vec2 bv = m2Particle_GetVelocity(bullet);
    CHECK(sqrtf(bv.x * bv.x + bv.y * bv.y) <= 6.001f,
          "the velocity limit clamps to one diameter per step");
    m2DestroyWorld(world);

    // Tangential shear: plain water keeps it, viscosity eats it. The
    // pair normal is y, the shear is x, so pressure and damping never
    // touch it; only the viscous pass can.
    m2WorldDef syrupDef = FluidWorldDef(8);
    syrupDef.gravity = (m2Vec2){0.0f, 0.0f};
    m2WorldDef waterDef = syrupDef;
    syrupDef.particleViscousStrength = 0.5f;
    m2WorldId water = m2CreateWorld(&waterDef);
    m2WorldId syrup = m2CreateWorld(&syrupDef);
    m2ParticleId wSlide = m2World_EmitParticle(water, (m2Pos2){0.0, 0.0}, (m2Vec2){1.0f, 0.0f});
    m2World_EmitParticle(water, (m2Pos2){0.0, 0.06}, (m2Vec2){0.0f, 0.0f});
    m2ParticleId sSlide = m2World_EmitParticle(syrup, (m2Pos2){0.0, 0.0}, (m2Vec2){1.0f, 0.0f});
    m2World_EmitParticle(syrup, (m2Pos2){0.0, 0.06}, (m2Vec2){0.0f, 0.0f});
    m2World_Step(water, 1.0f / 60.0f, 4);
    m2World_Step(syrup, 1.0f / 60.0f, 4);
    CHECK(m2Particle_GetVelocity(wSlide).x > 0.95f, "plain water keeps tangential shear");
    CHECK(m2Particle_GetVelocity(sSlide).x < 0.85f, "viscosity eats tangential shear");
    m2DestroyWorld(water);
    m2DestroyWorld(syrup);
}

// White-box: the neighbor structure is canonical and complete.
static void TestPairStructure(void)
{
    m2WorldDef def = FluidWorldDef(64);
    def.gravity = (m2Vec2){0.0f, 0.0f}; // hold positions still
    m2WorldId world = m2CreateWorld(&def);
    m2World* w = m2World_GetInternal(world);

    // A triangle inside one diameter (0.1), a loner far away, and a
    // pair straddling the x=0 cell seam (the bias regression: the
    // reference's 32-bit tag would wrap there).
    m2World_EmitParticle(world, (m2Pos2){0.30, 0.30}, (m2Vec2){0.0f, 0.0f});
    m2World_EmitParticle(world, (m2Pos2){0.36, 0.30}, (m2Vec2){0.0f, 0.0f});
    m2World_EmitParticle(world, (m2Pos2){0.30, 0.36}, (m2Vec2){0.0f, 0.0f});
    m2World_EmitParticle(world, (m2Pos2){5.0, 5.0}, (m2Vec2){0.0f, 0.0f});
    m2World_EmitParticle(world, (m2Pos2){-0.02, 2.0}, (m2Vec2){0.0f, 0.0f});
    m2World_EmitParticle(world, (m2Pos2){0.02, 2.0}, (m2Vec2){0.0f, 0.0f});
    m2World_Step(world, 1.0f / 60.0f, 4);

    CHECK(w->particlePairCount == 4, "triangle gives three pairs, the seam pair the fourth");
    CHECK(w->particlePairOverflow == 0, "no truncation in a sparse scene");
    int32_t seamSeen = 0;
    for (int32_t i = 0; i < w->particlePairCount; ++i)
    {
        int32_t a = w->particlePairA[i];
        int32_t b = w->particlePairB[i];
        CHECK(a != b, "no self pairs");
        CHECK(w->particlePairWeight[i] > 0.0f && w->particlePairWeight[i] <= 1.0f,
              "weights live in (0, 1]");
        float nx = w->particlePairNormal[i].x;
        float ny = w->particlePairNormal[i].y;
        float len = nx * nx + ny * ny;
        CHECK(len > 0.99f && len < 1.01f, "normals are unit length");
        seamSeen += (a == 4 && b == 5) || (a == 5 && b == 4) ? 1 : 0;
    }
    CHECK(seamSeen == 1, "the pair across the x=0 seam is found exactly once");

    // Coincident particles: full weight, canonical normal, no NaN.
    m2ParticleId c1 = m2World_EmitParticle(world, (m2Pos2){8.0, 8.0}, (m2Vec2){0.0f, 0.0f});
    m2ParticleId c2 = m2World_EmitParticle(world, (m2Pos2){8.0, 8.0}, (m2Vec2){0.0f, 0.0f});
    m2World_Step(world, 1.0f / 60.0f, 4);
    int32_t coincident = -1;
    for (int32_t i = 0; i < w->particlePairCount; ++i)
    {
        if (w->particlePairA[i] == c1.index1 - 1 && w->particlePairB[i] == c2.index1 - 1)
        {
            coincident = i;
        }
    }
    CHECK(coincident >= 0, "coincident particles still pair");
    CHECK(w->particlePairWeight[coincident] == 1.0f, "full overlap reads full weight");
    CHECK(w->particlePairNormal[coincident].x == 0.0f &&
              w->particlePairNormal[coincident].y == 1.0f,
          "the canonical fallback normal, never NaN");
    m2DestroyWorld(world);

    // Twin determinism: identical emits give byte-identical pair lists.
    m2WorldId wa = m2CreateWorld(&def);
    m2WorldId wb = m2CreateWorld(&def);
    for (int32_t i = 0; i < 40; ++i)
    {
        m2Pos2 p = {(double)(i % 7) * 0.07, (double)(i / 7) * 0.07};
        m2World_EmitParticle(wa, p, (m2Vec2){0.0f, 0.0f});
        m2World_EmitParticle(wb, p, (m2Vec2){0.0f, 0.0f});
    }
    m2World_Step(wa, 1.0f / 60.0f, 4);
    m2World_Step(wb, 1.0f / 60.0f, 4);
    m2World* ia = m2World_GetInternal(wa);
    m2World* ib = m2World_GetInternal(wb);
    CHECK(ia->particlePairCount == ib->particlePairCount, "twins agree on the pair count");
    CHECK(ia->particlePairCount > 0, "the lattice actually pairs");
    CHECK(memcmp(ia->particlePairA, ib->particlePairA,
                 (size_t)ia->particlePairCount * sizeof(int32_t)) == 0 &&
              memcmp(ia->particlePairB, ib->particlePairB,
                     (size_t)ia->particlePairCount * sizeof(int32_t)) == 0,
          "twin pair lists are byte-identical");
    m2DestroyWorld(wa);
    m2DestroyWorld(wb);
}

// Overflow truncates deterministically and counts what it dropped.
static void TestPairOverflow(void)
{
    m2WorldDef def = FluidWorldDef(64);
    def.gravity = (m2Vec2){0.0f, 0.0f};
    m2WorldId world = m2CreateWorld(&def);
    m2World* w = m2World_GetInternal(world);
    for (int32_t i = 0; i < 64; ++i)
    {
        m2World_EmitParticle(world, (m2Pos2){1.0, 1.0}, (m2Vec2){0.0f, 0.0f});
    }
    m2World_Step(world, 1.0f / 60.0f, 4);
    CHECK(w->particlePairCount == w->particlePairCapacity, "the budget fills exactly");
    CHECK(w->particlePairOverflow == 64 * 63 / 2 - w->particlePairCapacity,
          "every dropped pair is counted");
    m2DestroyWorld(world);
}

// The coupling acceptance: water poured into a static basin stays
// inside, calms down, and stacks a surface; the slice-84 promise
// that needed walls.
static void TestBasin(void)
{
    m2WorldDef def = FluidWorldDef(128);
    m2WorldId world = m2CreateWorld(&def);

    m2ShapeDef sd = m2DefaultShapeDef();
    m2BodyDef floorDef = m2DefaultBodyDef();
    floorDef.position = (m2Pos2){100.0, 49.9};
    m2Polygon floorBox = m2MakeBox(0.7f, 0.1f);
    m2CreatePolygonShape(m2CreateBody(world, &floorDef), &sd, &floorBox);
    m2Polygon wallBox = m2MakeBox(0.1f, 0.8f);
    m2BodyDef leftDef = m2DefaultBodyDef();
    leftDef.position = (m2Pos2){99.3, 50.7};
    m2CreatePolygonShape(m2CreateBody(world, &leftDef), &sd, &wallBox);
    m2BodyDef rightDef = m2DefaultBodyDef();
    rightDef.position = (m2Pos2){100.7, 50.7};
    m2CreatePolygonShape(m2CreateBody(world, &rightDef), &sd, &wallBox);

    m2ParticleId drops[60];
    for (int32_t i = 0; i < 60; ++i)
    {
        double x = 100.0 - 0.44 + (double)(i % 10) * 0.09;
        double y = 50.3 + (double)(i / 10) * 0.09;
        drops[i] = m2World_EmitParticle(world, (m2Pos2){x, y}, (m2Vec2){0.0f, 0.0f});
    }
    for (int32_t step = 0; step < 360; ++step)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    float lateSpeed = 0.0f;
    double lowest = 1.0e9;
    double highest = -1.0e9;
    for (int32_t i = 0; i < 60; ++i)
    {
        m2Pos2 p = m2Particle_GetPosition(drops[i]);
        CHECK(p.x == p.x && p.y == p.y, "water stays finite");
        CHECK(p.x > 99.15 && p.x < 100.85, "no particle leaks through a wall");
        CHECK(p.y > 49.85, "no particle leaks through the floor");
        m2Vec2 v = m2Particle_GetVelocity(drops[i]);
        float speed = sqrtf(v.x * v.x + v.y * v.y);
        lateSpeed = lateSpeed > speed ? lateSpeed : speed;
        lowest = p.y < lowest ? p.y : lowest;
        highest = p.y > highest ? p.y : highest;
    }
    CHECK(lateSpeed < 1.0f, "the pool calms down");
    CHECK(lowest < 50.1, "water reaches the floor");
    CHECK(highest < 51.0, "water pools instead of climbing the walls");
    m2DestroyWorld(world);
}

// One-way platforms are one-way for water; sensors are invisible.
static void TestWaterChainAndSensor(void)
{
    m2WorldDef def = FluidWorldDef(64);
    m2WorldId world = m2CreateWorld(&def);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.position = (m2Pos2){0.0, 5.0};
    m2BodyId ground = m2CreateBody(world, &bd);
    // Solid side up: chain runs right to left (the winding law).
    m2Vec2 points[4] = {{2.0f, 0.0f}, {1.0f, 0.0f}, {-1.0f, 0.0f}, {-2.0f, 0.0f}};
    m2ChainDef cd = m2DefaultChainDef();
    cd.points = points;
    cd.count = 4;
    m2CreateChain(ground, &cd);

    // A sensor box hangs above the platform, in the fall path.
    m2BodyDef sensorBody = m2DefaultBodyDef();
    sensorBody.position = (m2Pos2){0.0, 6.5};
    m2ShapeDef sensorDef = m2DefaultShapeDef();
    sensorDef.isSensor = true;
    m2Polygon sensorBox = m2MakeBox(0.5f, 0.2f);
    m2CreatePolygonShape(m2CreateBody(world, &sensorBody), &sensorDef, &sensorBox);

    m2ParticleId rain[10];
    for (int32_t i = 0; i < 10; ++i)
    {
        rain[i] = m2World_EmitParticle(world, (m2Pos2){-0.45 + (double)i * 0.09, 7.5},
                                       (m2Vec2){0.0f, 0.0f});
    }
    m2ParticleId spray = m2World_EmitParticle(world, (m2Pos2){0.0, 3.0}, (m2Vec2){0.0f, 5.9f});
    for (int32_t step = 0; step < 240; ++step)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    for (int32_t i = 0; i < 10; ++i)
    {
        m2Pos2 p = m2Particle_GetPosition(rain[i]);
        CHECK(p.y > 4.95 && p.y < 5.6, "rain rests on the solid side, through the sensor");
    }
    m2Pos2 sp = m2Particle_GetPosition(spray);
    CHECK(sp.y < 4.0, "spray from below passed the ghost side and fell back through");
    m2DestroyWorld(world);
}

// The 16th gated line: an emit/fall/churn scenario far from origin.
static void TestFluidHash(void)
{
    m2WorldDef def = FluidWorldDef(256);
    m2WorldId world = m2CreateWorld(&def);
    m2BodyDef floorDef = m2DefaultBodyDef();
    floorDef.position = (m2Pos2){300.5, 39.0};
    m2BodyId floorBody = m2CreateBody(world, &floorDef);
    m2ShapeDef floorShape = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(2.0f, 0.2f);
    m2CreatePolygonShape(floorBody, &floorShape, &slab);
    m2ParticleId ids[256];
    int32_t made = 0;
    for (int32_t i = 0; i < 120; ++i)
    {
        double x = 300.0 + (double)(i % 12) * 0.12;
        double y = 40.0 + (double)(i / 12) * 0.12;
        ids[made] =
            m2World_EmitParticle(world, (m2Pos2){x, y}, (m2Vec2){0.05f * (float)(i % 3), 0.0f});
        made += ids[made].index1 != 0 ? 1 : 0;
    }
    for (int32_t step = 0; step < 90; ++step)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        if (step % 13 == 0 && step / 13 < made && m2Particle_IsValid(ids[step / 13]))
        {
            m2World_DestroyParticle(ids[step / 13]);
        }
        if (step % 17 == 0)
        {
            m2World_EmitParticle(world, (m2Pos2){301.0, 41.0 + (double)step * 0.01},
                                 (m2Vec2){0.0f, 0.5f});
        }
    }
    uint64_t hash = m2World_Hash(world);
    // Fold the neighbor structure into the gated line: the pair list
    // itself must agree bit-for-bit across every CI cell.
    m2World* w = m2World_GetInternal(world);
    hash = m2Hash64(hash, &w->particlePairCount, (int32_t)sizeof(int32_t));
    hash = m2Hash64(hash, w->particlePairA, w->particlePairCount * (int32_t)sizeof(int32_t));
    hash = m2Hash64(hash, w->particlePairB, w->particlePairCount * (int32_t)sizeof(int32_t));
    hash = m2Hash64(hash, w->particlePairWeight, w->particlePairCount * (int32_t)sizeof(float));
    hash = m2Hash64(hash, w->particlePairNormal, w->particlePairCount * (int32_t)sizeof(m2Vec2));
    printf("M2_FLUID_HASH=%016llx\n", (unsigned long long)hash);
    m2DestroyWorld(world);
}

int main(void)
{
    TestLifecycle();
    TestOverflowIsQuiet();
    TestPairStructure();
    TestPairOverflow();
    TestRelaxation();
    TestBasin();
    TestWaterChainAndSensor();
    TestRollbackIdentity();
    TestJournalReplay();
    TestFluidHash();
    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
