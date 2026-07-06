// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The hostile integrator: seeded random walks over the ENTIRE public
// API (creates, destroys, setters, chains, joints, teleports, type
// flips), with the three Maul invariants enforced at the end of every
// scenario:
//   1. journal replay of the whole walk lands on the recorded hash,
//   2. snapshot -> divergent future -> restore returns the exact bits
//      and a re-snapshot is byte-equal,
//   3. a twin run of the same seed WITHOUT a journal reaches the same
//      hash (recording is observation-only, and no static state leaks
//      between worlds),
//   4. a THREADED twin (workerCount 3) of the same seed reaches the
//      same hash: parallel execution is a pure scheduling change even
//      under random API churn, not just on crafted scenes.
// The mixed end-state hash of all scenarios is the 15th CI gate line:
// eight platform cells must agree on the bits of several hundred
// randomized API calls. Hand-written tests walk the paths we thought
// of; this walks the ones we did not.

#include "maul2d/maul2d.h"

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

// xorshift64: fixed seeds, no libc rand, identical draws everywhere.
static uint64_t s_rng;

static uint64_t NextU64(void)
{
    uint64_t x = s_rng;
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    s_rng = x;
    return x;
}

static uint32_t Pick(uint32_t n)
{
    return (uint32_t)(NextU64() % (uint64_t)n);
}

// Quantized draws: every value is an exact float, so all platforms
// construct identical worlds from identical seeds.
static float PickUnit(void)
{
    return (float)Pick(101) * 0.01f;
}

static double PickCoord(void)
{
    return ((double)Pick(241) - 120.0) * 0.1;
}

static m2BodyId RandomBody(m2WorldId world)
{
    m2BodyId ids[16];
    int32_t n = m2World_GetBodies(world, ids, 16);
    if (n == 0)
    {
        m2BodyId nil = {0, 0, 0};
        return nil;
    }
    return ids[Pick((uint32_t)n)];
}

static m2JointId RandomJoint(m2WorldId world)
{
    m2JointId ids[8];
    int32_t n = m2World_GetJoints(world, ids, 8);
    if (n == 0)
    {
        m2JointId nil = {0, 0, 0};
        return nil;
    }
    return ids[Pick((uint32_t)n)];
}

static m2ShapeId RandomShape(m2WorldId world)
{
    m2BodyId body = RandomBody(world);
    m2ShapeId nil = {0, 0, 0};
    if (body.index1 == 0)
    {
        return nil;
    }
    m2ShapeId ids[16];
    int32_t n = m2Body_GetShapes(body, ids, 16);
    if (n == 0)
    {
        return nil;
    }
    return ids[Pick((uint32_t)n)];
}

static void DoRandomOp(m2WorldId world)
{
    m2Counters counters = m2World_GetCounters(world);
    uint32_t roll = Pick(100);

    if (roll < 24)
    {
        float dt = Pick(2) == 0 ? 1.0f / 60.0f : 1.0f / 120.0f;
        int32_t substeps = 2 << Pick(2); // 2, 4, or 8
        m2World_Step(world, dt, substeps);
        return;
    }
    if (roll < 30)
    {
        // Water ops: emit, retarget, destroy. Live targets come from
        // the enumeration (a reader, draw-free), the victim by draw.
        uint32_t which = Pick(3);
        if (which == 0)
        {
            double px = PickCoord();
            double py = (double)Pick(60) * 0.1;
            float vx = (float)((int32_t)Pick(7) - 3);
            float vy = (float)((int32_t)Pick(7) - 3);
            uint32_t kind = Pick(5);
            uint32_t flags =
                kind == 0 ? m2_tensileParticle
                          : (kind == 1 ? m2_viscousParticle : (kind == 2 ? m2_powderParticle : 0));
            m2World_EmitParticle(world, (m2Pos2){px, py}, (m2Vec2){vx, vy}, flags);
            return;
        }
        m2ParticleId targets[128];
        int32_t live = m2World_GetParticles(world, targets, 128);
        if (live == 0)
        {
            return;
        }
        uint32_t victim = Pick((uint32_t)live);
        if (which == 1)
        {
            float vx = (float)((int32_t)Pick(9) - 4);
            float vy = (float)((int32_t)Pick(9) - 4);
            m2Particle_SetVelocity(targets[victim], (m2Vec2){vx, vy});
            return;
        }
        m2World_DestroyParticle(targets[victim]);
        return;
    }
    if (roll < 35)
    {
        if (counters.bodies >= 15)
        {
            return;
        }
        m2BodyDef bd = m2DefaultBodyDef();
        uint32_t kind = Pick(100);
        bd.type = kind < 60 ? m2_dynamicBody : (kind < 85 ? m2_staticBody : m2_kinematicBody);
        // One draw per statement: evaluation order inside initializer
        // lists and argument lists is indeterminately sequenced in C,
        // and the fuzzer's first CI run proved compilers disagree.
        double px = PickCoord();
        double py = (double)Pick(80) * 0.1;
        bd.position = (m2Pos2){px, py};
        if (bd.type == m2_dynamicBody)
        {
            bd.isBullet = Pick(7) == 0;
            bd.gravityScale = 0.5f + (float)Pick(11) * 0.1f;
            float vx = (float)((int32_t)Pick(9) - 4);
            float vy = (float)((int32_t)Pick(9) - 4);
            bd.linearVelocity = (m2Vec2){vx, vy};
        }
        m2CreateBody(world, &bd);
        return;
    }
    if (roll < 45)
    {
        if (counters.shapes >= 26)
        {
            return;
        }
        m2BodyId body = RandomBody(world);
        if (body.index1 == 0)
        {
            return;
        }
        m2ShapeDef sd = m2DefaultShapeDef();
        sd.density = (float)(Pick(3) + 1) * 0.75f;
        sd.friction = PickUnit() * 0.9f;
        sd.restitution = PickUnit() * 0.8f;
        sd.isSensor = Pick(10) == 0;
        sd.categoryBits = 1u << Pick(4);
        sd.maskBits = Pick(5) == 0 ? 0x5u : 0xFFFFFFFFu;
        sd.groupIndex = (int32_t)Pick(5) - 2;
        uint32_t kind = Pick(3);
        if (kind == 0)
        {
            float cx = (float)((int32_t)Pick(11) - 5) * 0.1f;
            float cy = (float)((int32_t)Pick(11) - 5) * 0.1f;
            float radius = 0.2f + (float)Pick(25) * 0.01f;
            m2Circle circle = {{cx, cy}, radius};
            m2CreateCircleShape(body, &sd, &circle);
        }
        else if (kind == 1)
        {
            float half = 0.15f + (float)Pick(30) * 0.01f;
            m2Capsule capsule = {{-half, 0.0f}, {half, 0.0f}, 0.1f + (float)Pick(20) * 0.01f};
            m2CreateCapsuleShape(body, &sd, &capsule);
        }
        else
        {
            float halfW = 0.15f + (float)Pick(40) * 0.01f;
            float halfH = 0.15f + (float)Pick(40) * 0.01f;
            m2Polygon box = m2MakeBox(halfW, halfH);
            m2CreatePolygonShape(body, &sd, &box);
        }
        return;
    }
    if (roll < 50)
    {
        if (counters.joints >= 7)
        {
            return;
        }
        m2BodyId a = RandomBody(world);
        m2BodyId b = RandomBody(world);
        if (a.index1 == 0 || b.index1 == 0 || (a.index1 == b.index1))
        {
            return;
        }
        float aax = (float)((int32_t)Pick(5) - 2) * 0.1f;
        float aay = (float)((int32_t)Pick(5) - 2) * 0.1f;
        float abx = (float)((int32_t)Pick(5) - 2) * 0.1f;
        float aby = (float)((int32_t)Pick(5) - 2) * 0.1f;
        m2Vec2 anchorA = {aax, aay};
        m2Vec2 anchorB = {abx, aby};
        m2JointId made = {0, 0, 0};
        bool collide = Pick(4) == 0;
        uint32_t type = Pick(11);
        if (type == 10)
        {
            m2RatchetJointDef jd = m2DefaultRatchetJointDef();
            jd.bodyIdA = a;
            jd.bodyIdB = b;
            jd.collideConnected = collide;
            float tooth = 0.2f + (float)Pick(5) * 0.1f;
            jd.ratchet = Pick(2) == 0 ? tooth : -tooth;
            m2CreateRatchetJoint(world, &jd);
            return;
        }
        if (type == 9)
        {
            m2PulleyJointDef jd = m2DefaultPulleyJointDef();
            jd.bodyIdA = a;
            jd.bodyIdB = b;
            jd.collideConnected = collide;
            m2Pos2 pa = m2Body_GetPosition(a);
            m2Pos2 pb = m2Body_GetPosition(b);
            float sideA = (float)Pick(5) - 2.0f;
            float riseA = 2.0f + (float)Pick(4);
            float sideB = (float)Pick(5) - 2.0f;
            float riseB = 2.0f + (float)Pick(4);
            jd.groundAnchorA = (m2Pos2){pa.x + (double)sideA, pa.y + (double)riseA};
            jd.groundAnchorB = (m2Pos2){pb.x + (double)sideB, pb.y + (double)riseB};
            jd.ratio = 0.5f + (float)Pick(4) * 0.5f;
            m2CreatePulleyJoint(world, &jd);
            return;
        }
        if (type == 8)
        {
            m2GearJointDef jd = m2DefaultGearJointDef();
            jd.bodyIdA = a;
            jd.bodyIdB = b;
            jd.collideConnected = collide;
            float ratio = 0.5f + (float)Pick(4) * 0.5f;
            jd.ratio = Pick(2) == 0 ? ratio : -ratio;
            m2CreateGearJoint(world, &jd);
            return;
        }
        if (type == 6)
        {
            m2MotorJointDef jd = m2DefaultMotorJointDef();
            jd.bodyIdA = a;
            jd.bodyIdB = b;
            jd.collideConnected = collide;
            float ox = (float)((int32_t)Pick(7) - 3) * 0.5f;
            float oy = (float)((int32_t)Pick(7) - 3) * 0.5f;
            jd.linearOffset = (m2Vec2){ox, oy};
            jd.angularOffset = (float)((int32_t)Pick(5) - 2) * 0.2f;
            jd.maxForce = 10.0f + (float)Pick(50);
            jd.maxTorque = 5.0f + (float)Pick(20);
            m2CreateMotorJoint(world, &jd);
            return;
        }
        if (type == 7)
        {
            m2MouseJointDef jd = m2DefaultMouseJointDef();
            jd.bodyIdA = a;
            jd.bodyIdB = b;
            jd.collideConnected = collide;
            m2Pos2 bp = m2Body_GetPosition(b);
            double tx = (double)((int32_t)Pick(9) - 4) * 0.5;
            double ty = (double)((int32_t)Pick(9) - 4) * 0.5;
            jd.target = (m2Pos2){bp.x + tx, bp.y + ty};
            jd.maxForce = 20.0f + (float)Pick(60);
            m2CreateMouseJoint(world, &jd);
            return;
        }
        if (type == 5)
        {
            m2FilterJointDef jd = m2DefaultFilterJointDef();
            jd.bodyIdA = a;
            jd.bodyIdB = b;
            m2CreateFilterJoint(world, &jd);
            return;
        }
        if (type == 0)
        {
            m2DistanceJointDef jd = m2DefaultDistanceJointDef();
            jd.collideConnected = collide;
            jd.bodyIdA = a;
            jd.bodyIdB = b;
            jd.localAnchorA = anchorA;
            jd.localAnchorB = anchorB;
            jd.hertz = (float)Pick(3) * 2.0f;
            jd.dampingRatio = PickUnit();
            made = m2CreateDistanceJoint(world, &jd);
        }
        else if (type == 1)
        {
            m2RevoluteJointDef jd = m2DefaultRevoluteJointDef();
            jd.collideConnected = collide;
            jd.bodyIdA = a;
            jd.bodyIdB = b;
            jd.localAnchorA = anchorA;
            jd.localAnchorB = anchorB;
            uint32_t springOn = Pick(2);
            jd.springHertz = springOn == 1 ? 2.0f : 0.0f;
            jd.springDampingRatio = 0.5f;
            jd.enableMotor = Pick(2) == 0;
            jd.motorSpeed = (float)((int32_t)Pick(7) - 3);
            jd.maxMotorTorque = 5.0f + (float)Pick(20);
            jd.enableLimit = Pick(2) == 0;
            jd.lowerAngle = -0.1f - (float)Pick(10) * 0.05f;
            jd.upperAngle = 0.1f + (float)Pick(10) * 0.05f;
            made = m2CreateRevoluteJoint(world, &jd);
        }
        else if (type == 2)
        {
            m2PrismaticJointDef jd = m2DefaultPrismaticJointDef();
            jd.collideConnected = collide;
            jd.bodyIdA = a;
            jd.bodyIdB = b;
            jd.localAnchorA = anchorA;
            jd.localAnchorB = anchorB;
            jd.localAxisA = Pick(2) == 0 ? (m2Vec2){0.0f, 1.0f} : (m2Vec2){1.0f, 0.0f};
            jd.enableLimit = true;
            jd.lowerTranslation = -0.5f - (float)Pick(10) * 0.1f;
            jd.upperTranslation = 0.5f + (float)Pick(10) * 0.1f;
            made = m2CreatePrismaticJoint(world, &jd);
        }
        else if (type == 3)
        {
            m2WeldJointDef jd = m2DefaultWeldJointDef();
            jd.collideConnected = collide;
            jd.bodyIdA = a;
            jd.bodyIdB = b;
            jd.localAnchorA = anchorA;
            jd.localAnchorB = anchorB;
            jd.linearHertz = (float)Pick(3) * 2.0f;
            jd.linearDampingRatio = PickUnit();
            jd.angularHertz = (float)Pick(3) * 3.0f;
            jd.angularDampingRatio = PickUnit();
            made = m2CreateWeldJoint(world, &jd);
        }
        else
        {
            m2WheelJointDef jd = m2DefaultWheelJointDef();
            jd.collideConnected = collide;
            jd.bodyIdA = a;
            jd.bodyIdB = b;
            jd.localAnchorA = anchorA;
            jd.localAnchorB = anchorB;
            jd.localAxisA = (m2Vec2){0.0f, 1.0f};
            jd.enableSpring = Pick(2) == 0;
            jd.hertz = 1.0f + (float)Pick(4);
            jd.dampingRatio = PickUnit();
            jd.enableMotor = Pick(3) == 0;
            jd.motorSpeed = (float)((int32_t)Pick(9) - 4);
            jd.maxMotorTorque = 5.0f + (float)Pick(15);
            made = m2CreateWheelJoint(world, &jd);
        }
        if (made.index1 != 0 && Pick(5) == 0)
        {
            float bf = 10.0f + (float)Pick(60);
            float bt = 5.0f + (float)Pick(30);
            m2Joint_SetBreakLimits(made, bf, bt);
        }
        return;
    }
    if (roll < 54)
    {
        if (m2World_GetChains(world, NULL, 0) >= 5 || counters.shapes >= 24)
        {
            return;
        }
        m2BodyId body = RandomBody(world);
        if (body.index1 == 0)
        {
            return;
        }
        m2ChainDef cd = m2DefaultChainDef();
        cd.friction = 0.3f + PickUnit() * 0.6f;
        float y = (float)((int32_t)Pick(9) - 4);
        float x = (float)((int32_t)Pick(9) - 4);
        if (Pick(3) == 0)
        {
            m2Vec2 loop[4] = {{x, y - 1.0f}, {x + 1.0f, y}, {x, y + 1.0f}, {x - 1.0f, y}};
            cd.points = loop;
            cd.count = 4;
            cd.isLoop = true;
            m2CreateChain(body, &cd);
        }
        else
        {
            m2Vec2 open[5] = {{x + 4.0f, y}, {x + 2.0f, y}, {x, y}, {x - 2.0f, y}, {x - 4.0f, y}};
            cd.points = open;
            cd.count = 5;
            m2CreateChain(body, &cd);
        }
        return;
    }
    if (roll < 58)
    {
        m2BodyId body = RandomBody(world);
        if (body.index1 != 0)
        {
            m2DestroyBody(body);
        }
        return;
    }
    if (roll < 62)
    {
        m2ShapeId shape = RandomShape(world);
        if (shape.index1 != 0)
        {
            m2DestroyShape(shape); // chain links included: partial chains must survive
        }
        return;
    }
    if (roll < 65)
    {
        m2JointId joint = RandomJoint(world);
        if (joint.index1 != 0)
        {
            m2DestroyJoint(joint);
        }
        return;
    }
    if (roll < 67)
    {
        m2ChainId ids[8];
        int32_t n = m2World_GetChains(world, ids, 8);
        if (n > 0)
        {
            m2DestroyChain(ids[Pick((uint32_t)n)]);
        }
        return;
    }
    if (roll < 75)
    {
        m2BodyId body = RandomBody(world);
        if (body.index1 == 0)
        {
            return;
        }
        uint32_t which = Pick(13);
        if (which == 0)
        {
            float vx = (float)((int32_t)Pick(9) - 4);
            float vy = (float)((int32_t)Pick(9) - 4);
            m2Body_SetLinearVelocity(body, (m2Vec2){vx, vy});
        }
        else if (which == 1)
        {
            m2Body_SetAngularVelocity(body, (float)((int32_t)Pick(9) - 4));
        }
        else if (which == 2)
        {
            if (m2Body_GetType(body) == m2_dynamicBody)
            {
                m2Pos2 at = m2Body_GetPosition(body);
                float jx = (float)((int32_t)Pick(5) - 2);
                float jy = (float)((int32_t)Pick(5) - 2);
                m2Body_ApplyLinearImpulse(body, (m2Vec2){jx, jy}, at);
            }
        }
        else if (which == 3)
        {
            m2Rot identity = {1.0f, 0.0f};
            double tx = PickCoord();
            double ty = (double)Pick(60) * 0.1;
            m2Body_SetTransform(body, (m2Pos2){tx, ty}, identity);
        }
        else if (which == 4)
        {
            m2Body_SetType(body, (m2BodyType)Pick(3));
        }
        else if (which == 5)
        {
            if (m2Body_GetType(body) == m2_dynamicBody)
            {
                float fx = (float)((int32_t)Pick(9) - 4) * 3.0f;
                float fy = (float)((int32_t)Pick(9) - 4) * 3.0f;
                m2Body_ApplyForceToCenter(body, (m2Vec2){fx, fy});
            }
        }
        else if (which == 6)
        {
            if (m2Body_GetType(body) == m2_dynamicBody)
            {
                float torque = (float)((int32_t)Pick(9) - 4) * 2.0f;
                m2Body_ApplyTorque(body, torque);
            }
        }
        else if (which == 7)
        {
            m2Body_SetFixedRotation(body, Pick(2) == 0);
        }
        else if (which == 8)
        {
            float damping = (float)Pick(6) * 0.1f;
            m2Body_SetLinearDamping(body, damping);
        }
        else if (which == 9)
        {
            m2Body_EnableSleep(body, Pick(4) != 0);
        }
        else if (which == 10)
        {
            m2Body_SetAwake(body, Pick(3) != 0);
        }
        else if (which == 11)
        {
            int8_t dom = (int8_t)((int32_t)Pick(5) - 2);
            m2Body_SetDominance(body, dom);
        }
        else
        {
            uint64_t data = (uint64_t)Pick(1000);
            m2Body_SetUserData(body, data);
        }
        return;
    }
    if (roll == 75)
    {
        m2BodyId body = RandomBody(world);
        if (body.index1 != 0)
        {
            if (Pick(2) == 0)
            {
                m2Body_Disable(body);
            }
            else
            {
                m2Body_Enable(body);
            }
        }
        return;
    }
    if (roll == 76)
    {
        m2ExplosionDef boom = m2DefaultExplosionDef();
        double ex = PickCoord();
        double ey = (double)Pick(60) * 0.1;
        boom.position = (m2Pos2){ex, ey};
        boom.radius = 0.5f + (float)Pick(20) * 0.1f;
        boom.falloff = 0.5f + (float)Pick(10) * 0.1f;
        boom.impulse = (float)Pick(30) * 0.2f;
        m2World_Explode(world, &boom);
        return;
    }
    if (roll < 79)
    {
        float gx = (float)((int32_t)Pick(5) - 2) * 0.5f;
        float gy = -10.0f + (float)((int32_t)Pick(9) - 4) * 0.5f;
        m2World_SetGravity(world, (m2Vec2){gx, gy});
        return;
    }
    if (roll < 90)
    {
        m2ShapeId shape = RandomShape(world);
        if (shape.index1 == 0)
        {
            return;
        }
        uint32_t which = Pick(4);
        if (which == 0)
        {
            m2Shape_SetFriction(shape, PickUnit() * 0.9f);
            float belt = (float)((int32_t)Pick(5) - 2) * 0.5f;
            m2Shape_SetTangentSpeed(shape, belt);
        }
        else if (which == 1)
        {
            m2Shape_SetRestitution(shape, PickUnit() * 0.8f);
        }
        else if (which == 3)
        {
            float density = 0.5f + (float)Pick(4) * 0.5f;
            m2Shape_SetDensity(shape, density);
        }
        else
        {
            uint32_t category = 1u << Pick(4);
            uint32_t mask = Pick(4) == 0 ? 0x3u : 0xFFFFFFFFu;
            int32_t group = (int32_t)Pick(5) - 2;
            m2Shape_SetFilter(shape, category, mask, group);
        }
        return;
    }
    if (roll == 90)
    {
        m2ShapeId shape = RandomShape(world);
        if (shape.index1 != 0 && m2Shape_GetType(shape) != m2_chainSegmentShape)
        {
            if (Pick(2) == 0)
            {
                float radius = 0.2f + (float)Pick(25) * 0.01f;
                m2Circle c = {{0.0f, 0.0f}, radius};
                m2Shape_SetCircle(shape, &c);
            }
            else
            {
                float halfW = 0.15f + (float)Pick(30) * 0.01f;
                float halfH = 0.15f + (float)Pick(30) * 0.01f;
                m2Polygon box = m2MakeBox(halfW, halfH);
                m2Shape_SetPolygon(shape, &box);
            }
        }
        return;
    }
    {
        m2JointId joint = RandomJoint(world);
        if (joint.index1 == 0)
        {
            return;
        }
        uint32_t which = Pick(9);
        if (which == 0)
        {
            m2Joint_SetMotorSpeed(joint, (float)((int32_t)Pick(9) - 4));
        }
        else if (which == 1)
        {
            m2Joint_EnableMotor(joint, Pick(2) == 0);
        }
        else if (which == 2)
        {
            float lower = -0.2f - (float)Pick(8) * 0.1f;
            m2Joint_SetLimits(joint, lower, lower + 0.4f + (float)Pick(8) * 0.1f);
        }
        else if (which == 3)
        {
            float bf = 20.0f + (float)Pick(80);
            float bt = 10.0f + (float)Pick(40);
            m2Joint_SetBreakLimits(joint, bf, bt);
        }
        else if (which == 4)
        {
            float ox = (float)((int32_t)Pick(7) - 3) * 0.5f;
            float oy = (float)((int32_t)Pick(7) - 3) * 0.5f;
            float oa = (float)((int32_t)Pick(5) - 2) * 0.2f;
            if (m2Joint_GetType(joint) == m2_motorJoint)
            {
                m2MotorJoint_SetOffsets(joint, (m2Vec2){ox, oy}, oa);
            }
        }
        else if (which == 5)
        {
            double tx = (double)((int32_t)Pick(9) - 4) * 0.5;
            double ty = (double)((int32_t)Pick(9) - 4) * 0.5;
            if (m2Joint_GetType(joint) == m2_mouseJoint)
            {
                m2Pos2 base = m2Body_GetPosition(m2Joint_GetBodyB(joint));
                m2MouseJoint_SetTarget(joint, (m2Pos2){base.x + tx, base.y + ty});
            }
        }
        else if (which == 6)
        {
            uint64_t data = (uint64_t)Pick(1000);
            m2Joint_SetUserData(joint, data);
        }
        else if (which == 7)
        {
            float hertz = (float)Pick(30) * 0.5f;
            m2JointType type = m2Joint_GetType(joint);
            if (type != m2_motorJoint && type != m2_filterJoint)
            {
                m2Joint_SetSpringHertz(joint, hertz);
            }
        }
        else
        {
            float lo = 0.2f + (float)Pick(10) * 0.1f;
            float hi = lo + 0.2f + (float)Pick(15) * 0.1f;
            if (m2Joint_GetType(joint) == m2_distanceJoint)
            {
                m2DistanceJoint_SetLengthRange(joint, lo, hi);
            }
        }
    }
}

// One full walk. With a journal the walk is recorded and must replay
// onto its own bits; the rollback identity is enforced either way.
static uint64_t RunScenario(uint64_t seed, uint8_t* journal, int32_t journalCapacity,
                            int32_t workerCount)
{
    s_rng = seed;
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 32;
    def.jointCapacity = 8;
    def.particleCapacity = 128; // the walks drive water too
    def.workerCount = workerCount;
    m2WorldId world = m2CreateWorld(&def);
    if (journal != NULL)
    {
        m2World_StartJournal(world, journal, journalCapacity);
    }

    // A floor first, so the chaos has something to land on.
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -2.0};
    m2BodyId floor = m2CreateBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(14.0f, 0.5f);
    m2CreatePolygonShape(floor, &fs, &slab);

    for (int32_t i = 0; i < 300; ++i)
    {
        DoRandomOp(world);
    }
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    uint64_t endHash = m2World_Hash(world);

    if (journal != NULL)
    {
        int32_t bytes = m2World_StopJournal(world);
        CHECK(bytes > 0, "the walk fits the journal");
        m2WorldId fresh = m2CreateWorld(&def);
        CHECK(m2World_ReplayJournal(fresh, journal, bytes), "the walk replays");
        CHECK(m2World_Hash(fresh) == endHash, "replay lands on the recorded bits");
        m2DestroyWorld(fresh);
    }

    // Rollback identity from a randomized world: restore returns the
    // exact bits and a re-snapshot is byte-equal.
    int32_t size = m2World_SnapshotSize(world);
    void* snapA = malloc((size_t)size);
    void* snapB = malloc((size_t)size);
    CHECK(m2World_Snapshot(world, snapA, size) == size, "snapshot");
    for (int32_t i = 0; i < 15; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4); // a future to roll back from
    }
    CHECK(m2World_Restore(world, snapA, size), "restore");
    CHECK(m2World_Hash(world) == endHash, "restore returns the exact bits");
    CHECK(m2World_Snapshot(world, snapB, size) == size, "re-snapshot");
    CHECK(memcmp(snapA, snapB, (size_t)size) == 0, "re-snapshot is byte-equal");
    free(snapA);
    free(snapB);
    m2DestroyWorld(world);
    return endHash;
}

int main(void)
{
    enum
    {
        JOURNAL_CAPACITY = 1 << 21
    };
    uint8_t* journal = malloc(JOURNAL_CAPACITY);
    uint64_t fuzzHash = 0x9E3779B97F4A7C15ull;
    static const uint64_t seeds[10] = {
        0x0000000000C0FFEEull, 0x00000000BADC0DE5ull, 0x0000000012345678ull, 0x00000000DEADBEA7ull,
        0x00000000CAFEF00Dull, 0x00000000A11C0DE5ull, 0x000000005EED5EEDull, 0x00000000B16B00B5ull,
        0x00000000F005BA11ull, 0x000000007E57AB1Eull,
    };
    for (int32_t i = 0; i < 10; ++i)
    {
        uint64_t recorded = RunScenario(seeds[i], journal, JOURNAL_CAPACITY, 1);
        uint64_t twin = RunScenario(seeds[i], NULL, 0, 1);
        uint64_t threaded = RunScenario(seeds[i], NULL, 0, 3);
        CHECK(recorded == twin, "the unjournaled twin reaches the same bits");
        CHECK(recorded == threaded, "the threaded twin reaches the same bits");
        fuzzHash ^= recorded + 0x9E3779B97F4A7C15ull + (fuzzHash << 6) + (fuzzHash >> 2);
    }
    free(journal);

    printf("M2_FUZZ_HASH=%016llx\n", (unsigned long long)fuzzHash);
    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
