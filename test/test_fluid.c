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

    // Retargeting velocity is immediate.
    m2Particle_SetVelocity(reborn, (m2Vec2){3.0f, 0.0f});
    m2World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m2Particle_GetPosition(reborn).x > 0.04, "a set velocity moves the particle");

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

// The 16th gated line: an emit/fall/churn scenario far from origin.
static void TestFluidHash(void)
{
    m2WorldDef def = FluidWorldDef(256);
    m2WorldId world = m2CreateWorld(&def);
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
    printf("M2_FLUID_HASH=%016llx\n", (unsigned long long)hash);
    m2DestroyWorld(world);
}

int main(void)
{
    TestLifecycle();
    TestOverflowIsQuiet();
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
