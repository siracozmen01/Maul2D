// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Journal gate: a recorded session (spawns, shoves, joints, destroys,
// steps) replays into an identical world hash; recorded ids re-mint
// bit-identically; overflow is loud; and the journal BYTES themselves
// hash identically across CI cells - the wire format is part of the
// determinism contract (ADR-0010 rule 6b).

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

static m2WorldDef TestDef(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 64;
    def.shapeCapacity = 64;
    def.jointCapacity = 32;
    return def;
}

// A session with every op type: floor, rain of shapes, a pendulum, a
// shove, a destroy, steps in two dt flavors (markers carry dt verbatim).
static void RunSession(m2WorldId world, uint8_t* journal, int32_t capacity, int32_t* outSize,
                       uint64_t* outHash)
{
    if (journal != NULL)
    {
        CHECK(m2World_StartJournal(world, journal, capacity), "journal starts");
    }

    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = m2CreateBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(15.0f, 0.5f);
    m2CreatePolygonShape(floor, &fs, &slab);

    m2BodyId victim = m2_nullBodyId;
    for (int32_t i = 0; i < 8; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){-4.0 + 1.1 * (double)i, 2.0 + 0.4 * (double)i};
        m2BodyId body = m2CreateBody(world, &bd);
        m2ShapeDef sd = m2DefaultShapeDef();
        sd.restitution = 0.3f;
        if (i % 2 == 0)
        {
            m2Circle c = {{0.0f, 0.0f}, 0.3f};
            m2CreateCircleShape(body, &sd, &c);
        }
        else
        {
            m2Polygon box = m2MakeBox(0.3f, 0.25f);
            m2CreatePolygonShape(body, &sd, &box);
        }
        if (i == 3)
        {
            victim = body;
        }
        if (i == 5)
        {
            m2Body_SetLinearVelocity(body, (m2Vec2){-2.5f, 1.0f});
            m2Body_SetAngularVelocity(body, 3.0f);
        }
    }

    m2BodyDef ad = m2DefaultBodyDef();
    ad.position = (m2Pos2){5.0, 5.0};
    m2BodyId anchor = m2CreateBody(world, &ad);
    m2BodyDef bobDef = m2DefaultBodyDef();
    bobDef.type = m2_dynamicBody;
    bobDef.position = (m2Pos2){6.5, 5.0};
    m2BodyId bob = m2CreateBody(world, &bobDef);
    m2ShapeDef bs = m2DefaultShapeDef();
    m2Circle bc = {{0.0f, 0.0f}, 0.2f};
    m2CreateCircleShape(bob, &bs, &bc);
    m2DistanceJointDef jd = m2DefaultDistanceJointDef();
    jd.bodyIdA = anchor;
    jd.bodyIdB = bob;
    m2JointId rod = m2CreateDistanceJoint(world, &jd);

    // A motorized slider exercises the prismatic create op (op 10).
    m2BodyDef ramDef = m2DefaultBodyDef();
    ramDef.type = m2_dynamicBody;
    ramDef.position = (m2Pos2){9.0, 2.0};
    m2BodyId ram = m2CreateBody(world, &ramDef);
    m2CreateCircleShape(ram, &bs, &bc);
    m2BodyDef cylDef = m2DefaultBodyDef();
    cylDef.position = (m2Pos2){9.0, 2.0};
    m2BodyId cylinder = m2CreateBody(world, &cylDef);
    m2PrismaticJointDef press = m2DefaultPrismaticJointDef();
    press.bodyIdA = cylinder;
    press.bodyIdB = ram;
    press.localAxisA = (m2Vec2){0.0f, 1.0f};
    press.enableMotor = true;
    press.motorSpeed = -0.5f;
    press.maxMotorForce = 20.0f;
    press.enableLimit = true;
    press.lowerTranslation = -0.8f;
    press.upperTranslation = 0.3f;
    m2CreatePrismaticJoint(world, &press);

    for (int32_t i = 0; i < 45; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2DestroyBody(victim);
    m2DestroyJoint(rod);
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 120.0f, 2); // second dt flavor
    }

    if (journal != NULL)
    {
        *outSize = m2World_StopJournal(world);
        CHECK(*outSize > 0, "journal stops without overflow");
    }
    *outHash = m2World_Hash(world);
}

static void TestRecordReplay(void)
{
    enum
    {
        CAPACITY = 1 << 20
    };
    uint8_t* journal = malloc(CAPACITY);

    m2WorldDef def = TestDef();
    m2WorldId original = m2CreateWorld(&def);
    int32_t journalSize = 0;
    uint64_t originalHash = 0;
    RunSession(original, journal, CAPACITY, &journalSize, &originalHash);

    // Replay into a FRESH world of the same shape.
    m2WorldId fresh = m2CreateWorld(&def);
    CHECK(m2World_ReplayJournal(fresh, journal, journalSize), "replay accepts the journal");
    CHECK(m2World_Hash(fresh) == originalHash, "replayed world hash matches bit-exactly");
    CHECK(m2World_GetStepCount(fresh) == m2World_GetStepCount(original), "step counts match");

    // Replay is repeatable: run it again on the same world.
    CHECK(m2World_ReplayJournal(fresh, journal, journalSize), "replay is idempotent from t0");
    CHECK(m2World_Hash(fresh) == originalHash, "second replay matches too");

    // Wrong-shaped world refuses the journal.
    m2WorldDef other = TestDef();
    other.bodyCapacity = 128;
    m2WorldId mismatched = m2CreateWorld(&other);
    CHECK(!m2World_ReplayJournal(mismatched, journal, journalSize),
          "capacity mismatch is rejected");

    m2DestroyWorld(original);
    m2DestroyWorld(fresh);
    m2DestroyWorld(mismatched);
    free(journal);
}

static void TestOverflowIsLoud(void)
{
    m2WorldDef def = TestDef();
    m2WorldId world = m2CreateWorld(&def);
    int32_t snapshotSize = m2World_SnapshotSize(world);
    int32_t capacity = snapshotSize + 64; // room for the header + a crumb
    uint8_t* journal = malloc((size_t)capacity + sizeof(void*));
    CHECK(m2World_StartJournal(world, journal, capacity), "tiny journal starts");
    for (int32_t i = 0; i < 8; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        m2CreateBody(world, &bd); // blows past the crumb
    }
    CHECK(m2World_StopJournal(world) == 0, "overflowed journal reports zero, never truncates");
    m2DestroyWorld(world);
    free(journal);
}

static uint64_t JournalSweepHash(void)
{
    // The journal BYTES are the artifact here: bit-identical across
    // platforms, or the wire format has a leak.
    enum
    {
        CAPACITY = 1 << 20
    };
    uint8_t* journal = malloc(CAPACITY);
    m2WorldDef def = TestDef();
    m2WorldId world = m2CreateWorld(&def);
    int32_t size = 0;
    uint64_t worldHash = 0;
    RunSession(world, journal, CAPACITY, &size, &worldHash);

    uint64_t h = M2_HASH_INIT;
    h = m2Hash64(h, journal, size);
    h = m2Hash64(h, &size, (int32_t)sizeof(size));
    h = m2Hash64(h, &worldHash, (int32_t)sizeof(worldHash));

    m2DestroyWorld(world);
    free(journal);
    return h;
}

int main(void)
{
    TestRecordReplay();
    TestOverflowIsLoud();

    uint64_t hash = JournalSweepHash();
    printf("M2_JOURNAL_HASH=%016llx\n", (unsigned long long)hash);

    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
