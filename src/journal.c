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
#define M2_JOURNAL_VERSION 32u

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

void m2JournalRecordRestore(m2World* world, const void* snapshot, int32_t size)
{
    if (world->journalActive == 0 || world->journalOverflow != 0)
    {
        return;
    }
    int32_t needed = 1 + (int32_t)sizeof(int32_t) + size;
    if (world->journalCursor + needed > world->journalCapacity)
    {
        world->journalOverflow = 1;
        return;
    }
    world->journal[world->journalCursor] = (uint8_t)m2_opRestore;
    memcpy(world->journal + world->journalCursor + 1, &size, sizeof(int32_t));
    memcpy(world->journal + world->journalCursor + 1 + sizeof(int32_t), snapshot, (size_t)size);
    world->journalCursor += needed;
}

typedef struct m2OpChainHeader
{
    m2BodyId body;
    int32_t count;
    int32_t createdCount;
    float friction;
    float restitution;
    uint32_t categoryBits;
    uint32_t maskBits;
    int32_t groupIndex;
    uint64_t userData;
    uint8_t isLoop;
} m2OpChainHeader;

void m2JournalRecordShatter(m2World* world, m2BodyId bodyId, const m2Polygon* pieces,
                            int32_t pieceCount, int32_t expectedFirst)
{
    if (world->journalActive == 0 || world->journalOverflow != 0)
    {
        return;
    }
    int32_t pieceBytes = pieceCount * (int32_t)sizeof(m2Polygon);
    int32_t needed = 1 + (int32_t)sizeof(m2OpShatterHeader) + pieceBytes;
    if (world->journalCursor + needed > world->journalCapacity)
    {
        world->journalOverflow = 1;
        return;
    }
    m2OpShatterHeader header;
    memset(&header, 0, sizeof(header));
    header.body = bodyId;
    header.pieceCount = pieceCount;
    header.expectedFirst = expectedFirst;
    world->journal[world->journalCursor] = (uint8_t)m2_opShatterBody;
    memcpy(world->journal + world->journalCursor + 1, &header, sizeof(header));
    memcpy(world->journal + world->journalCursor + 1 + sizeof(header), pieces, (size_t)pieceBytes);
    world->journalCursor += needed;
}

void m2JournalRecordChain(m2World* world, m2BodyId bodyId, const m2ChainDef* def,
                          int32_t createdCount)
{
    if (world->journalActive == 0 || world->journalOverflow != 0)
    {
        return;
    }
    int32_t pointBytes = def->count * (int32_t)sizeof(m2Vec2);
    int32_t needed = 1 + (int32_t)sizeof(m2OpChainHeader) + pointBytes;
    if (world->journalCursor + needed > world->journalCapacity)
    {
        world->journalOverflow = 1;
        return;
    }
    m2OpChainHeader header;
    memset(&header, 0, sizeof(header));
    header.body = bodyId;
    header.count = def->count;
    header.createdCount = createdCount;
    header.friction = def->friction;
    header.restitution = def->restitution;
    header.categoryBits = def->categoryBits;
    header.maskBits = def->maskBits;
    header.groupIndex = def->groupIndex;
    header.userData = def->userData;
    header.isLoop = def->isLoop ? 1 : 0;
    world->journal[world->journalCursor] = (uint8_t)m2_opCreateChain;
    memcpy(world->journal + world->journalCursor + 1, &header, sizeof(header));
    memcpy(world->journal + world->journalCursor + 1 + sizeof(header), def->points,
           (size_t)pointBytes);
    world->journalCursor += needed;
}

int32_t m2World_JournalBaseSize(m2WorldId worldId)
{
    int32_t snapshot = m2World_SnapshotSize(worldId);
    if (snapshot <= 0)
    {
        return 0;
    }
    return (int32_t)sizeof(m2JournalHeader) + snapshot;
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

typedef struct m2OpLinearImpulse
{
    m2BodyId body;
    m2Vec2 impulse;
    m2Pos2 point;
} m2OpLinearImpulse;

typedef struct m2OpJointParam
{
    m2JointId joint;
    float value;
    uint8_t param;
} m2OpJointParam;

typedef struct m2OpSetType
{
    m2BodyId body;
    uint8_t type;
} m2OpSetType;

typedef struct m2OpSetTransform
{
    m2BodyId body;
    m2Pos2 position;
    m2Rot rotation;
} m2OpSetTransform;

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
        case m2_opDestroyShape:
        {
            M2_READ_OP(m2ShapeId, id);
            id.world0 = here;
            m2DestroyShape(id);
            break;
        }
        case m2_opDestroyChain:
        {
            M2_READ_OP(m2ChainId, chain);
            chain.world0 = here;
            m2DestroyChain(chain);
            break;
        }
        case m2_opBodyParam:
        {
            struct m2OpBodyParam
            {
                m2BodyId body;
                float value;
                uint8_t param;
            };
            M2_READ_OP(struct m2OpBodyParam, bp);
            bp.body.world0 = here;
            m2SetBodyParamInternal(world, bp.body, bp.param, bp.value);
            break;
        }
        case m2_opEnableSleeping:
        {
            struct m2OpSleepFlag
            {
                uint8_t flag;
            };
            M2_READ_OP(struct m2OpSleepFlag, sf);
            m2World_EnableSleeping(worldId, sf.flag != 0);
            break;
        }
        case m2_opApplyForce:
        {
            struct m2OpForce
            {
                m2BodyId body;
                m2Vec2 force;
                m2Pos2 point;
            };
            M2_READ_OP(struct m2OpForce, fo);
            fo.body.world0 = here;
            m2Body_ApplyForce(fo.body, fo.force, fo.point);
            break;
        }
        case m2_opApplyForceCenter:
        {
            struct m2OpForceCenter
            {
                m2BodyId body;
                m2Vec2 force;
            };
            M2_READ_OP(struct m2OpForceCenter, fc);
            fc.body.world0 = here;
            m2Body_ApplyForceToCenter(fc.body, fc.force);
            break;
        }
        case m2_opCreateFilterJoint:
        {
            struct m2OpFilterJoint
            {
                m2FilterJointDef def;
                m2JointId expected;
            };
            M2_READ_OP(struct m2OpFilterJoint, fj);
            fj.def.bodyIdA.world0 = here;
            fj.def.bodyIdB.world0 = here;
            m2JointId made = m2CreateFilterJoint(worldId, &fj.def);
            M2_ASSERT(made.index1 == fj.expected.index1);
            (void)made;
            break;
        }
        case m2_opCreateMotorJoint:
        {
            struct m2OpMotorJoint
            {
                m2MotorJointDef def;
                m2JointId expected;
            };
            M2_READ_OP(struct m2OpMotorJoint, mj);
            mj.def.bodyIdA.world0 = here;
            mj.def.bodyIdB.world0 = here;
            m2JointId made = m2CreateMotorJoint(worldId, &mj.def);
            M2_ASSERT(made.index1 == mj.expected.index1);
            (void)made;
            break;
        }
        case m2_opCreateMouseJoint:
        {
            struct m2OpMouseJoint
            {
                m2MouseJointDef def;
                m2JointId expected;
            };
            M2_READ_OP(struct m2OpMouseJoint, sj);
            sj.def.bodyIdA.world0 = here;
            sj.def.bodyIdB.world0 = here;
            m2JointId made = m2CreateMouseJoint(worldId, &sj.def);
            M2_ASSERT(made.index1 == sj.expected.index1);
            (void)made;
            break;
        }
        case m2_opMotorOffsets:
        {
            struct m2OpMotorOffsets
            {
                m2JointId joint;
                m2Vec2 linear;
                float angular;
            };
            M2_READ_OP(struct m2OpMotorOffsets, mo);
            mo.joint.world0 = here;
            m2MotorJoint_SetOffsets(mo.joint, mo.linear, mo.angular);
            break;
        }
        case m2_opMouseTarget:
        {
            struct m2OpMouseTarget
            {
                m2JointId joint;
                m2Pos2 target;
            };
            M2_READ_OP(struct m2OpMouseTarget, mt);
            mt.joint.world0 = here;
            m2MouseJoint_SetTarget(mt.joint, mt.target);
            break;
        }
        case m2_opDisableBody:
        {
            M2_READ_OP(m2BodyId, body);
            body.world0 = here;
            m2Body_Disable(body);
            break;
        }
        case m2_opEnableBody:
        {
            M2_READ_OP(m2BodyId, body);
            body.world0 = here;
            m2Body_Enable(body);
            break;
        }
        case m2_opSetMassData:
        {
            struct m2OpMassData
            {
                m2BodyId body;
                m2MassData data;
            };
            M2_READ_OP(struct m2OpMassData, md);
            md.body.world0 = here;
            m2Body_SetMassData(md.body, md.data);
            break;
        }
        case m2_opMassFromShapes:
        {
            M2_READ_OP(m2BodyId, body);
            body.world0 = here;
            m2Body_ApplyMassFromShapes(body);
            break;
        }
        case m2_opExplode:
        {
            M2_READ_OP(m2ExplosionDef, boom);
            m2World_Explode(worldId, &boom);
            break;
        }
        case m2_opSetGeometry:
        {
            struct m2OpSetGeometry
            {
                m2ShapeId shape;
                m2ShapeGeometry geometry;
            };
            M2_READ_OP(struct m2OpSetGeometry, sg);
            sg.shape.world0 = here;
            switch (sg.geometry.type)
            {
            case m2_circleShape:
                m2Shape_SetCircle(sg.shape, &sg.geometry.circle);
                break;
            case m2_capsuleShape:
                m2Shape_SetCapsule(sg.shape, &sg.geometry.capsule);
                break;
            case m2_polygonShape:
                m2Shape_SetPolygon(sg.shape, &sg.geometry.polygon);
                break;
            default:
                m2Shape_SetSegment(sg.shape, &sg.geometry.segment);
                break;
            }
            break;
        }
        case m2_opChainFriction:
        case m2_opChainRestitution:
        {
            struct m2OpChainMaterial
            {
                m2ChainId chain;
                float value;
            };
            M2_READ_OP(struct m2OpChainMaterial, cm);
            cm.chain.world0 = here;
            if (op == m2_opChainFriction)
            {
                m2Chain_SetFriction(cm.chain, cm.value);
            }
            else
            {
                m2Chain_SetRestitution(cm.chain, cm.value);
            }
            break;
        }
        case m2_opImpulseCenter:
        {
            struct m2OpImpulseCenter
            {
                m2BodyId body;
                m2Vec2 impulse;
            };
            M2_READ_OP(struct m2OpImpulseCenter, ic);
            ic.body.world0 = here;
            m2Body_ApplyLinearImpulseToCenter(ic.body, ic.impulse);
            break;
        }
        case m2_opSetAwake:
        {
            struct m2OpSetAwake
            {
                m2BodyId body;
                uint8_t awake;
            };
            M2_READ_OP(struct m2OpSetAwake, sa);
            sa.body.world0 = here;
            m2Body_SetAwake(sa.body, sa.awake != 0);
            break;
        }
        case m2_opSetBullet:
        {
            struct m2OpSetBullet
            {
                m2BodyId body;
                uint8_t flag;
            };
            M2_READ_OP(struct m2OpSetBullet, sb);
            sb.body.world0 = here;
            m2Body_SetBullet(sb.body, sb.flag != 0);
            break;
        }
        case m2_opSetDensity:
        {
            struct m2OpSetDensity
            {
                m2ShapeId shape;
                float density;
            };
            M2_READ_OP(struct m2OpSetDensity, sd);
            sd.shape.world0 = here;
            m2Shape_SetDensity(sd.shape, sd.density);
            break;
        }
        case m2_opBodyUserData:
        {
            struct m2OpBodyUserData
            {
                m2BodyId body;
                uint64_t userData;
            };
            M2_READ_OP(struct m2OpBodyUserData, bu);
            bu.body.world0 = here;
            m2Body_SetUserData(bu.body, bu.userData);
            break;
        }
        case m2_opShapeUserData:
        {
            struct m2OpShapeUserData
            {
                m2ShapeId shape;
                uint64_t userData;
            };
            M2_READ_OP(struct m2OpShapeUserData, su);
            su.shape.world0 = here;
            m2Shape_SetUserData(su.shape, su.userData);
            break;
        }
        case m2_opJointUserData:
        {
            struct m2OpJointUserData
            {
                m2JointId joint;
                uint64_t userData;
            };
            M2_READ_OP(struct m2OpJointUserData, ju);
            ju.joint.world0 = here;
            m2Joint_SetUserData(ju.joint, ju.userData);
            break;
        }
        case m2_opSetDominance:
        {
            struct m2OpSetDominance
            {
                m2BodyId body;
                int8_t dominance;
            };
            M2_READ_OP(struct m2OpSetDominance, sd2);
            sd2.body.world0 = here;
            m2Body_SetDominance(sd2.body, sd2.dominance);
            break;
        }
        case m2_opCreateGearJoint:
        {
            struct m2OpGearJoint
            {
                m2GearJointDef def;
                m2JointId expected;
            };
            M2_READ_OP(struct m2OpGearJoint, gj);
            gj.def.bodyIdA.world0 = here;
            gj.def.bodyIdB.world0 = here;
            m2JointId made = m2CreateGearJoint(worldId, &gj.def);
            M2_ASSERT(made.index1 == gj.expected.index1);
            (void)made;
            break;
        }
        case m2_opCreatePulleyJoint:
        {
            struct m2OpPulleyJoint
            {
                m2PulleyJointDef def;
                m2JointId expected;
            };
            M2_READ_OP(struct m2OpPulleyJoint, pj);
            pj.def.bodyIdA.world0 = here;
            pj.def.bodyIdB.world0 = here;
            m2JointId made = m2CreatePulleyJoint(worldId, &pj.def);
            M2_ASSERT(made.index1 == pj.expected.index1);
            (void)made;
            break;
        }
        case m2_opEmitParticle:
        {
            struct m2OpEmit
            {
                m2Pos2 position;
                m2Vec2 velocity;
                uint32_t flags;
                m2ParticleId expected;
            };
            M2_READ_OP(struct m2OpEmit, em);
            m2ParticleId made = m2World_EmitParticle(worldId, em.position, em.velocity, em.flags);
            M2_ASSERT(made.index1 == em.expected.index1);
            (void)made;
            break;
        }
        case m2_opDestroyParticle:
        {
            struct m2OpKillParticle
            {
                m2ParticleId id;
            };
            M2_READ_OP(struct m2OpKillParticle, kp);
            kp.id.world0 = here;
            m2World_DestroyParticle(kp.id);
            break;
        }
        case m2_opSetParticleVelocity:
        {
            struct m2OpParticleVel
            {
                m2ParticleId id;
                m2Vec2 velocity;
            };
            M2_READ_OP(struct m2OpParticleVel, pv);
            pv.id.world0 = here;
            m2Particle_SetVelocity(pv.id, pv.velocity);
            break;
        }
        case m2_opSetParticleLifetime:
        {
            struct m2OpParticleLifetime
            {
                m2ParticleId id;
                float seconds;
            };
            M2_READ_OP(struct m2OpParticleLifetime, pl);
            pl.id.world0 = here;
            m2Particle_SetLifetime(pl.id, pl.seconds);
            break;
        }
        case m2_opSetParticleUserData:
        {
            struct m2OpParticleUserData
            {
                m2ParticleId id;
                uint64_t userData;
            };
            M2_READ_OP(struct m2OpParticleUserData, pu);
            pu.id.world0 = here;
            m2Particle_SetUserData(pu.id, pu.userData);
            break;
        }
        case m2_opCreateRatchetJoint:
        {
            struct m2OpRatchetJoint
            {
                m2RatchetJointDef def;
                m2JointId expected;
            };
            M2_READ_OP(struct m2OpRatchetJoint, rj);
            rj.def.bodyIdA.world0 = here;
            rj.def.bodyIdB.world0 = here;
            m2JointId made = m2CreateRatchetJoint(worldId, &rj.def);
            M2_ASSERT(made.index1 == rj.expected.index1);
            (void)made;
            break;
        }
        case m2_opFillParticles:
        {
            struct m2OpFill
            {
                m2Polygon polygon;
                m2Pos2 position;
                m2Vec2 velocity;
                uint32_t flags;
                int32_t expected;
            };
            M2_READ_OP(struct m2OpFill, fp);
            int32_t made = m2World_FillPolygonWithParticles(worldId, &fp.polygon, fp.position,
                                                            fp.velocity, fp.flags);
            M2_ASSERT(made == fp.expected);
            (void)made;
            break;
        }
        case m2_opApplyTorque:
        {
            struct m2OpTorque
            {
                m2BodyId body;
                float torque;
            };
            M2_READ_OP(struct m2OpTorque, tq);
            tq.body.world0 = here;
            m2Body_ApplyTorque(tq.body, tq.torque);
            break;
        }
        case m2_opApplyLinearImpulse:
        {
            M2_READ_OP(m2OpLinearImpulse, li);
            li.body.world0 = here;
            m2Body_ApplyLinearImpulse(li.body, li.impulse, li.point);
            break;
        }
        case m2_opApplyAngularImpulse:
        {
            M2_READ_OP(m2OpBodyFloat, ai);
            ai.body.world0 = here;
            m2Body_ApplyAngularImpulse(ai.body, ai.value);
            break;
        }
        case m2_opSetJointParam:
        {
            M2_READ_OP(m2OpJointParam, jp);
            jp.joint.world0 = here;
            m2World* target = m2World_GetInternal(worldId);
            if (target == NULL)
            {
                return false;
            }
            m2SetJointParamInternal(target, jp.joint, jp.param, jp.value);
            break;
        }
        case m2_opRestore:
        {
            M2_READ_OP(int32_t, restoreSize);
            if (restoreSize <= 0 || cursor + restoreSize > size)
            {
                return false; // truncated restore payload: loud
            }
            if (!m2World_Restore(worldId, in + cursor, restoreSize))
            {
                return false;
            }
            cursor += restoreSize;
            break;
        }
        case m2_opShatterBody:
        {
            M2_READ_OP(m2OpShatterHeader, sh);
            int32_t pieceBytes = sh.pieceCount * (int32_t)sizeof(m2Polygon);
            if (sh.pieceCount < 1 || cursor + pieceBytes > size)
            {
                return false;
            }
            // The stream is unaligned; copy pieces out before use.
            m2Polygon* pieces = m2AllocZeroed((size_t)pieceBytes);
            if (pieces == NULL)
            {
                return false;
            }
            memcpy(pieces, in + cursor, (size_t)pieceBytes);
            cursor += pieceBytes;
            sh.body.world0 = here;
            m2BodyId first[1];
            int32_t made = m2World_ShatterBody(sh.body, pieces, sh.pieceCount, first, 1);
            M2_ASSERT(made == sh.pieceCount);
            M2_ASSERT(made == 0 || first[0].index1 == sh.expectedFirst);
            (void)made;
            m2Free(pieces);
            break;
        }
        case m2_opCreateChain:
        {
            M2_READ_OP(m2OpChainHeader, ch);
            int32_t pointBytes = ch.count * (int32_t)sizeof(m2Vec2);
            if (ch.count < 3 || cursor + pointBytes > size)
            {
                return false;
            }
            // The stream is unaligned; copy points out before use.
            m2Vec2* pts = m2AllocZeroed((size_t)pointBytes);
            if (pts == NULL)
            {
                return false;
            }
            memcpy(pts, in + cursor, (size_t)pointBytes);
            m2ChainDef def = m2DefaultChainDef();
            def.points = pts;
            def.count = ch.count;
            def.isLoop = ch.isLoop != 0;
            def.friction = ch.friction;
            def.restitution = ch.restitution;
            def.categoryBits = ch.categoryBits;
            def.maskBits = ch.maskBits;
            def.groupIndex = ch.groupIndex;
            def.userData = ch.userData;
            ch.body.world0 = here;
            m2ChainId made = m2CreateChain(ch.body, &def);
            m2Free(pts);
            M2_ASSERT(m2Chain_GetSegmentCount(made) == ch.createdCount);
            (void)made;
            cursor += pointBytes;
            break;
        }
        case m2_opSetGravity:
        {
            struct GravityOp
            {
                m2Vec2 gravity;
            };
            M2_READ_OP(struct GravityOp, g);
            m2World_SetGravity(worldId, g.gravity);
            break;
        }
        case m2_opShapeParam:
        {
            struct ShapeParamOp
            {
                m2ShapeId shape;
                float value;
                uint8_t param;
            };
            M2_READ_OP(struct ShapeParamOp, sp);
            sp.shape.world0 = here;
            m2World* target = m2World_GetInternal(worldId);
            if (target == NULL)
            {
                return false;
            }
            m2SetShapeParamInternal(target, sp.shape, sp.param, sp.value);
            break;
        }
        case m2_opSetFilter:
        {
            struct FilterOp
            {
                m2ShapeId shape;
                uint32_t categoryBits;
                uint32_t maskBits;
                int32_t groupIndex;
            };
            M2_READ_OP(struct FilterOp, fo);
            fo.shape.world0 = here;
            m2Shape_SetFilter(fo.shape, fo.categoryBits, fo.maskBits, fo.groupIndex);
            break;
        }
        case m2_opSetType:
        {
            M2_READ_OP(m2OpSetType, tc);
            tc.body.world0 = here;
            m2Body_SetType(tc.body, (m2BodyType)tc.type);
            break;
        }
        case m2_opSetTransform:
        {
            M2_READ_OP(m2OpSetTransform, st);
            st.body.world0 = here;
            m2Body_SetTransform(st.body, st.position, st.rotation);
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
