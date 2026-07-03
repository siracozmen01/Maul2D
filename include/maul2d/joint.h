// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#ifndef MAUL2D_JOINT_H
#define MAUL2D_JOINT_H

#include "maul2d/body.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct m2JointId
    {
        int32_t index1; // 1-based, 0 = null
        uint16_t world0;
        uint16_t generation;
    } m2JointId;

    /// Keeps two local anchor points at a fixed distance. hertz = 0 uses
    /// the stiff default; accumulated impulses are snapshot state.
    typedef struct m2DistanceJointDef
    {
        m2BodyId bodyIdA;
        m2BodyId bodyIdB;
        m2Vec2 localAnchorA; // body-local, f32
        m2Vec2 localAnchorB;
        float length; // meters; <= 0 derives from spawn poses
        float hertz;  // 0 = stiff default
        float dampingRatio;
        int32_t internalValue;
    } m2DistanceJointDef;

    /// Pins two local anchor points together (a hinge). The reference
    /// angle for limits is the relative angle at creation; limit angles
    /// and the motor are measured from there.
    typedef struct m2RevoluteJointDef
    {
        m2BodyId bodyIdA;
        m2BodyId bodyIdB;
        m2Vec2 localAnchorA;
        m2Vec2 localAnchorB;
        float hertz; // 0 = stiff default
        float dampingRatio;
        bool enableMotor;
        float motorSpeed;     // rad/s, positive = counter-clockwise B vs A
        float maxMotorTorque; // N*m budget per step
        bool enableLimit;
        float lowerAngle; // radians relative to the creation angle
        float upperAngle;
        int32_t internalValue;
    } m2RevoluteJointDef;

    /// A slider: body B translates along an axis fixed in body A's
    /// frame; relative rotation is locked to the creation angle. The
    /// motor drives translation speed, limits bound the translation
    /// measured from the creation separation.
    typedef struct m2PrismaticJointDef
    {
        m2BodyId bodyIdA;
        m2BodyId bodyIdB;
        m2Vec2 localAnchorA;
        m2Vec2 localAnchorB;
        m2Vec2 localAxisA; // body-A local; normalized at create
        float hertz;       // 0 = stiff default
        float dampingRatio;
        bool enableMotor;
        float motorSpeed;    // m/s along the axis
        float maxMotorForce; // N budget per step
        bool enableLimit;
        float lowerTranslation; // meters from the creation separation
        float upperTranslation;
        int32_t internalValue;
    } m2PrismaticJointDef;

    /// Welds two bodies into one rigid piece: relative position and
    /// rotation lock to their creation values. hertz softens the weld
    /// (0 = rigid via the stiff default).
    typedef struct m2WeldJointDef
    {
        m2BodyId bodyIdA;
        m2BodyId bodyIdB;
        m2Vec2 localAnchorA;
        m2Vec2 localAnchorB;
        float hertz; // 0 = stiff default
        float dampingRatio;
        int32_t internalValue;
    } m2WeldJointDef;

    /// A suspension wheel: body B slides on a body-A axis with a spring,
    /// spins freely, and the motor drives relative rotation. hertz and
    /// dampingRatio shape the spring when enableSpring is set.
    typedef struct m2WheelJointDef
    {
        m2BodyId bodyIdA;
        m2BodyId bodyIdB;
        m2Vec2 localAnchorA;
        m2Vec2 localAnchorB;
        m2Vec2 localAxisA; // body-A local; normalized at create
        bool enableSpring;
        float hertz; // suspension spring (default 2)
        float dampingRatio;
        bool enableMotor;
        float motorSpeed;     // rad/s of B relative to A
        float maxMotorTorque; // N*m budget per step
        bool enableLimit;
        float lowerTranslation; // meters from the creation separation
        float upperTranslation;
        int32_t internalValue;
    } m2WheelJointDef;

    m2DistanceJointDef m2DefaultDistanceJointDef(void);
    m2RevoluteJointDef m2DefaultRevoluteJointDef(void);
    m2PrismaticJointDef m2DefaultPrismaticJointDef(void);
    m2WeldJointDef m2DefaultWeldJointDef(void);
    m2WheelJointDef m2DefaultWheelJointDef(void);

    /// Joints join their bodies' sleep island: connected bodies sleep and
    /// wake together. Destroying either body destroys the joint.
    /// Thread class: writer.
    m2JointId m2CreateDistanceJoint(m2WorldId worldId, const m2DistanceJointDef* def);
    m2JointId m2CreateRevoluteJoint(m2WorldId worldId, const m2RevoluteJointDef* def);
    m2JointId m2CreatePrismaticJoint(m2WorldId worldId, const m2PrismaticJointDef* def);
    m2JointId m2CreateWeldJoint(m2WorldId worldId, const m2WeldJointDef* def);
    m2JointId m2CreateWheelJoint(m2WorldId worldId, const m2WheelJointDef* def);
    void m2DestroyJoint(m2JointId jointId);

    /// Runtime joint tuning. Motor speed is rad/s on revolute and
    /// wheel joints, m/s on prismatic; max motor is a torque or force
    /// budget accordingly. Every change wakes both bodies and is
    /// journaled. Distance joints ignore motor and limit parameters.
    void m2Joint_SetMotorSpeed(m2JointId jointId, float speed);
    void m2Joint_SetMaxMotor(m2JointId jointId, float maxTorqueOrForce);
    void m2Joint_EnableMotor(m2JointId jointId, bool enable);
    void m2Joint_EnableLimit(m2JointId jointId, bool enable);
    void m2Joint_SetLimits(m2JointId jointId, float lower, float upper);
    bool m2Joint_IsValid(m2JointId jointId);

    static const m2JointId m2_nullJointId = {0, 0, 0};

#ifdef __cplusplus
}
#endif

#endif // MAUL2D_JOINT_H
