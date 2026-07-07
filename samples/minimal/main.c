// The ten-minute Maul2D: a ground, a tower, one journal, one
// rollback. Build against an installed maul2d (see README.md here).
#include "maul2d/maul2d.h"

#include <stdio.h>
#include <stdlib.h>

int main(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 64;
    def.shapeCapacity = 64;
    def.jointCapacity = 8;
    m2WorldId world = m2CreateWorld(&def);

    m2BodyDef groundDef = m2DefaultBodyDef();
    groundDef.position = (m2Pos2){0.0, -0.5};
    m2ShapeDef groundShape = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(10.0f, 0.5f);
    m2CreatePolygonShape(m2CreateBody(world, &groundDef), &groundShape, &slab);

    m2Polygon crate = m2MakeBox(0.5f, 0.5f);
    m2BodyId top = m2_nullBodyId;
    for (int i = 0; i < 5; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){0.0, 0.5 + (double)i * 1.02};
        top = m2CreateBody(world, &bd);
        m2ShapeDef sd = m2DefaultShapeDef();
        m2CreatePolygonShape(top, &sd, &crate);
    }

    // Simulate one second, snapshot, wreck the tower, then roll back:
    // the wreck never happened, bit for bit.
    for (int i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    int size = m2World_SnapshotSize(world);
    void* snap = malloc((size_t)size);
    m2World_Snapshot(world, snap, size);
    unsigned long long before = (unsigned long long)m2World_Hash(world);

    m2Body_ApplyLinearImpulseToCenter(top, (m2Vec2){8.0f, 4.0f});
    for (int i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    printf("wrecked hash %016llx\n", (unsigned long long)m2World_Hash(world));

    m2World_Restore(world, snap, size);
    printf("restored hash %016llx (matches the snapshot moment: %s)\n",
           (unsigned long long)m2World_Hash(world),
           (unsigned long long)m2World_Hash(world) == before ? "yes" : "NO");

    m2Pos2 p = m2Body_GetPosition(top);
    printf("tower top after rollback: (%.2f, %.2f)\n", p.x, p.y);
    free(snap);
    m2DestroyWorld(world);
    return 0;
}
