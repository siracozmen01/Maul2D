// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Command journal (topic-09 D3). Wire format, all little-endian raw
// structs (floats as IEEE-754 bit patterns by construction - ADR-0010
// rule 6b): header { magic, version, worldDef echo, snapshotSize },
// the embedded snapshot, then op records. Step markers carry dt and
// substepCount verbatim. Replay = restore + re-apply through the same
// public entry points; deterministic id re-minting is asserted.

#include "world_internal.h"

#include "maul2d/base.h"

#include <stdio.h>
#include <string.h>

#define M2_JOURNAL_MAGIC   0x4D324A4Eu // 'M2JN'
#define M2_JOURNAL_VERSION 3u

typedef struct m2JournalHeader
{
    uint32_t magic;
    uint32_t version;
    m2Vec2 gravity;
    int32_t bodyCapacity;
    int32_t shapeCapacity;
    int32_t jointCapacity;
    int32_t snapshotSize;
} m2JournalHeader;

_Static_assert(sizeof(m2JournalHeader) == 32, "journal header must be padding-free");

void m2JournalRecord(m2World* world, uint8_t op, const void* payload, int32_t bytes)
{
    if (world->journalActive == 0 || world->journalOverflow != 0)
    {
        return;
    }
    if (world->journalCursor + 1 + bytes > world->journalCapacity)
    {
        world->journalOverflow = 1; // loud at StopJournal, never silent
        return;
    }
    world->journal[world->journalCursor] = op;
    memcpy(world->journal + world->journalCursor + 1, payload, (size_t)bytes);
    world->journalCursor += 1 + bytes;
}

bool m2World_StartJournal(m2WorldId worldId, void* buffer, int32_t capacity)
{
    m2World* world = m2World_GetInternal(worldId);
    if (world == NULL || buffer == NULL || world->journalActive != 0)
    {
        return false;
    }
    int32_t snapshotSize = m2World_SnapshotSize(worldId);
    if (capacity < (int32_t)sizeof(m2JournalHeader) + snapshotSize)
    {
        return false;
    }

    m2JournalHeader header;
    memset(&header, 0, sizeof(header));
    header.magic = M2_JOURNAL_MAGIC;
    header.version = M2_JOURNAL_VERSION;
    header.gravity = world->gravity;
    header.bodyCapacity = world->bodyCapacity;
    header.shapeCapacity = world->shapeCapacity;
    header.jointCapacity = world->jointCapacity;
    header.snapshotSize = snapshotSize;

    uint8_t* out = buffer;
    memcpy(out, &header, sizeof(header));
    if (m2World_Snapshot(worldId, out + sizeof(header), snapshotSize) != snapshotSize)
    {
        return false;
    }

    world->journal = out;
    world->journalCapacity = capacity;
    world->journalCursor = (int32_t)sizeof(header) + snapshotSize;
    world->journalActive = 1;
    world->journalOverflow = 0;
    return true;
}

int32_t m2World_StopJournal(m2WorldId worldId)
{
    m2World* world = m2World_GetInternal(worldId);
    if (world == NULL || world->journalActive == 0)
    {
        return 0;
    }
    world->journalActive = 0;
    int32_t size = world->journalOverflow != 0 ? 0 : world->journalCursor;
    world->journal = NULL;
    world->journalCapacity = 0;
    world->journalCursor = 0;
    world->journalOverflow = 0;
    return size;
}

// Replay payload structs mirror the recorders in world.c exactly.
typedef struct m2OpStep
{
    float dt;
    int32_t substepCount;
} m2OpStep;

typedef struct m2OpCreateBody
{
    m2BodyDef def;
    m2BodyId expected;
} m2OpCreateBody;

typedef struct m2OpCreateShape
{
    m2BodyId body;
    m2ShapeDef def;
    m2ShapeGeometry geometry;
    m2ShapeId expected;
} m2OpCreateShape;

typedef struct m2OpCreateDistanceJoint
{
    m2DistanceJointDef def;
    m2JointId expected;
} m2OpCreateDistanceJoint;

typedef struct m2OpCreateRevoluteJoint
{
    m2RevoluteJointDef def;
    m2JointId expected;
} m2OpCreateRevoluteJoint;

typedef struct m2OpCreatePrismaticJoint
{
    m2PrismaticJointDef def;
    m2JointId expected;
} m2OpCreatePrismaticJoint;

typedef struct m2OpCreateWeldJoint
{
    m2WeldJointDef def;
    m2JointId expected;
} m2OpCreateWeldJoint;

typedef struct m2OpCreateWheelJoint
{
    m2WheelJointDef def;
    m2JointId expected;
} m2OpCreateWheelJoint;

typedef struct m2OpBodyVec
{
    m2BodyId body;
    m2Vec2 value;
} m2OpBodyVec;

typedef struct m2OpBodyFloat
{
    m2BodyId body;
    float value;
} m2OpBodyFloat;

bool m2World_ReplayJournal(m2WorldId worldId, const void* data, int32_t size)
{
    m2World* world = m2World_GetInternal(worldId);
    if (world == NULL || data == NULL || size < (int32_t)sizeof(m2JournalHeader) ||
        world->journalActive != 0)
    {
        return false;
    }
    m2JournalHeader header;
    memcpy(&header, data, sizeof(header));
    if (header.magic != M2_JOURNAL_MAGIC || header.version != M2_JOURNAL_VERSION ||
        header.bodyCapacity != world->bodyCapacity ||
        header.shapeCapacity != world->shapeCapacity ||
        header.jointCapacity != world->jointCapacity ||
        size < (int32_t)sizeof(header) + header.snapshotSize)
    {
        return false;
    }
    const uint8_t* in = data;
    if (!m2World_Restore(worldId, in + sizeof(header), header.snapshotSize))
    {
        return false;
    }

    // Recorded ids carry the recording world's registry index; rebind
    // every id to the target world so replay can never reach across the
    // registry into the original (the classic replay leak).
    uint16_t here = worldId.index1;

    int32_t cursor = (int32_t)sizeof(header) + header.snapshotSize;
    while (cursor < size)
    {
        uint8_t op = in[cursor];
        cursor += 1;
#define M2_READ_OP(type, name)                                                                     \
    type name;                                                                                     \
    if (cursor + (int32_t)sizeof(type) > size)                                                     \
    {                                                                                              \
        return false;                                                                              \
    }                                                                                              \
    memcpy(&name, in + cursor, sizeof(type));                                                      \
    cursor += (int32_t)sizeof(type)
        switch (op)
        {
        case m2_opStep:
        {
            M2_READ_OP(m2OpStep, step);
            m2World_Step(worldId, step.dt, step.substepCount);
            break;
        }
        case m2_opCreateBody:
        {
            M2_READ_OP(m2OpCreateBody, create);
            m2BodyId id = m2CreateBody(worldId, &create.def);
            M2_ASSERT(id.index1 == create.expected.index1 &&
                      id.generation == create.expected.generation);
            (void)id;
            break;
        }
        case m2_opDestroyBody:
        {
            M2_READ_OP(m2BodyId, id);
            id.world0 = here;
            m2DestroyBody(id);
            break;
        }
        case m2_opSetLinearVelocity:
        {
            M2_READ_OP(m2OpBodyVec, set);
            set.body.world0 = here;
            m2Body_SetLinearVelocity(set.body, set.value);
            break;
        }
        case m2_opSetAngularVelocity:
        {
            M2_READ_OP(m2OpBodyFloat, set);
            set.body.world0 = here;
            m2Body_SetAngularVelocity(set.body, set.value);
            break;
        }
        case m2_opCreateShape:
        {
            M2_READ_OP(m2OpCreateShape, create);
            create.body.world0 = here;
            m2ShapeId id = m2_nullShapeId;
            switch (create.geometry.type)
            {
            case m2_circleShape:
                id = m2CreateCircleShape(create.body, &create.def, &create.geometry.circle);
                break;
            case m2_capsuleShape:
                id = m2CreateCapsuleShape(create.body, &create.def, &create.geometry.capsule);
                break;
            case m2_polygonShape:
                id = m2CreatePolygonShape(create.body, &create.def, &create.geometry.polygon);
                break;
            default:
                id = m2CreateSegmentShape(create.body, &create.def, &create.geometry.segment);
                break;
            }
            M2_ASSERT(id.index1 == create.expected.index1 &&
                      id.generation == create.expected.generation);
            (void)id;
            break;
        }
        case m2_opCreateDistanceJoint:
        {
            M2_READ_OP(m2OpCreateDistanceJoint, create);
            create.def.bodyIdA.world0 = here;
            create.def.bodyIdB.world0 = here;
            m2JointId id = m2CreateDistanceJoint(worldId, &create.def);
            M2_ASSERT(id.index1 == create.expected.index1);
            (void)id;
            break;
        }
        case m2_opCreateRevoluteJoint:
        {
            M2_READ_OP(m2OpCreateRevoluteJoint, create);
            create.def.bodyIdA.world0 = here;
            create.def.bodyIdB.world0 = here;
            m2JointId id = m2CreateRevoluteJoint(worldId, &create.def);
            M2_ASSERT(id.index1 == create.expected.index1);
            (void)id;
            break;
        }
        case m2_opCreatePrismaticJoint:
        {
            M2_READ_OP(m2OpCreatePrismaticJoint, create);
            create.def.bodyIdA.world0 = here;
            create.def.bodyIdB.world0 = here;
            m2JointId id = m2CreatePrismaticJoint(worldId, &create.def);
            M2_ASSERT(id.index1 == create.expected.index1);
            (void)id;
            break;
        }
        case m2_opCreateWeldJoint:
        {
            M2_READ_OP(m2OpCreateWeldJoint, create);
            create.def.bodyIdA.world0 = here;
            create.def.bodyIdB.world0 = here;
            m2JointId id = m2CreateWeldJoint(worldId, &create.def);
            M2_ASSERT(id.index1 == create.expected.index1);
            (void)id;
            break;
        }
        case m2_opCreateWheelJoint:
        {
            M2_READ_OP(m2OpCreateWheelJoint, create);
            create.def.bodyIdA.world0 = here;
            create.def.bodyIdB.world0 = here;
            m2JointId id = m2CreateWheelJoint(worldId, &create.def);
            M2_ASSERT(id.index1 == create.expected.index1);
            (void)id;
            break;
        }
        case m2_opDestroyJoint:
        {
            M2_READ_OP(m2JointId, id);
            id.world0 = here;
            m2DestroyJoint(id);
            break;
        }
        default:
        {
            return false; // unknown op: corrupt stream, loud
        }
        }
#undef M2_READ_OP
    }
    return true;
}
