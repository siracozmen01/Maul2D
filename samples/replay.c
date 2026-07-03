// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Record a session into the command journal, then replay it into a
// completely fresh world and compare the bits. This is the primitive
// that deterministic lockstep, replays and bug reports are built on.

#include "maul2d/maul2d.h"

#include <stdio.h>
#include <stdlib.h>

static void Shove(m2BodyId body, float impulseX)
{
    m2Body_ApplyLinearImpulse(body, (m2Vec2){impulseX, 0.0f}, m2Body_GetPosition(body));
}

int main(void)
{
    // The journal embeds a full snapshot up front, so size the tape
    // from m2World_JournalBaseSize plus room for the ops you expect.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 64;
    def.shapeCapacity = 64;
    def.jointCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);

    int tapeCapacity = m2World_JournalBaseSize(world) + (1 << 16);
    unsigned char* journal = malloc((size_t)tapeCapacity);
    if (!m2World_StartJournal(world, journal, tapeCapacity))
    {
        printf("journal would not start - tape too small?\n");
        return 1;
    }

    // Everything from here on is being recorded: creation, physics
    // steps, impulses, even destruction.
    m2BodyDef groundDef = m2DefaultBodyDef();
    groundDef.position = (m2Pos2){0.0, -0.5};
    m2BodyId ground = m2CreateBody(world, &groundDef);
    m2ShapeDef groundShape = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(20.0f, 0.5f);
    m2CreatePolygonShape(ground, &groundShape, &slab);

    m2BodyId boxes[5];
    for (int i = 0; i < 5; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){-2.0 + (double)i, 2.0 + 0.5 * (double)i};
        boxes[i] = m2CreateBody(world, &bd);
        m2ShapeDef sd = m2DefaultShapeDef();
        m2Polygon unit = m2MakeBox(0.4f, 0.4f);
        m2CreatePolygonShape(boxes[i], &sd, &unit);
    }

    for (int step = 0; step < 60; ++step)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    Shove(boxes[2], 3.0f);   // meddle mid-session
    m2DestroyBody(boxes[4]); // and destroy something, why not
    for (int step = 0; step < 60; ++step)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }

    int bytes = m2World_StopJournal(world);
    unsigned long long recorded = (unsigned long long)m2World_Hash(world);
    printf("recorded %d bytes of session, end hash %016llx\n", bytes, recorded);

    // A brand-new world, same capacities, replay the tape.
    m2WorldId fresh = m2CreateWorld(&def);
    bool ok = m2World_ReplayJournal(fresh, journal, bytes);
    unsigned long long replayed = (unsigned long long)m2World_Hash(fresh);
    printf("replayed into a fresh world,  end hash %016llx\n", replayed);
    printf(ok && recorded == replayed ? "bit-exact match.\n" : "MISMATCH - file a bug!\n");

    m2DestroyWorld(fresh);
    m2DestroyWorld(world);
    free(journal);
    return recorded == replayed ? 0 : 1;
}
