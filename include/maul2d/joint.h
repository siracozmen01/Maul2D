// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#ifndef MAUL2D_JOINT_H
#define MAUL2D_JOINT_H

#include "maul2d/body.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum m2JointType
    {
        m2_distanceJoint = 0,
        m2_revoluteJoint = 1,
        m2_prismaticJoint = 2,
        m2_weldJoint = 3,
        m2_wheelJoint = 4,
        m2_filterJoint = 5,
        m2_motorJoint = 6,
        m2_mouseJoint = 7,
    } m2JointType;

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
        /// Hard length range, off by default (0 .. huge). When a
        /// bound is active it clamps regardless of the spring.
        float minLength;
        float maxLength; // <= 0 means unbounded
        float hertz;     // 0 = stiff default
        float dampingRatio;
        uint64_t userData;     // opaque, journaled through snapshots
        bool collideConnected; // default false: jointed bodies pass through
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
        uint64_t userData;     // opaque, journaled through snapshots
        bool collideConnected; // default false: jointed bodies pass through
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
        uint64_t userData;     // opaque, journaled through snapshots
        bool collideConnected; // default false: jointed bodies pass through
        int32_t internalValue;
    } m2PrismaticJointDef;

    /// Welds two bodies into one rigid piece: relative position and
    /// rotation lock to their creation values. The linear and angular
    /// rows soften independently, reference-style: zero hertz means
    /// rigid via the stiff default, nonzero turns that row into a real
    /// spring (biased even in the relax pass).
    typedef struct m2WeldJointDef
    {
        m2BodyId bodyIdA;
        m2BodyId bodyIdB;
        m2Vec2 localAnchorA;
        m2Vec2 localAnchorB;
        float linearHertz; // 0 = rigid
        float linearDampingRatio;
        float angularHertz; // 0 = rigid
        float angularDampingRatio;
        uint64_t userData;     // opaque, journaled through snapshots
        bool collideConnected; // default false: jointed bodies pass through
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
        uint64_t userData;     // opaque, journaled through snapshots
        bool collideConnected; // default false: jointed bodies pass through
        int32_t internalValue;
    } m2WheelJointDef;

    /// Drives body B's transform toward an offset from body A with
    /// force and torque budgets: moving platforms, magnetic grips,
    /// soft body-follow. Offsets are in body A's frame; correction
    /// pulls position error at correctionFactor per step.
    typedef struct m2MotorJointDef
    {
        m2BodyId bodyIdA;
        m2BodyId bodyIdB;
        m2Vec2 linearOffset;    // target of B minus A, in A's frame
        float angularOffset;    // target angle of B minus A, radians
        float maxForce;         // newtons
        float maxTorque;        // newton meters
        float correctionFactor; // [0,1] position correction per step
        uint64_t userData;
        bool collideConnected;
        int32_t internalValue;
    } m2MotorJointDef;

    /// A soft spring pulling one point of body B toward a world-space
    /// target: dragging. Body A is only a bookkeeping anchor. The grab
    /// point is where the target sits at creation.
    typedef struct m2MouseJointDef
    {
        m2BodyId bodyIdA;
        m2BodyId bodyIdB;
        m2Pos2 target; // world space, f64
        float hertz;
        float dampingRatio;
        float maxForce;
        uint64_t userData;
        bool collideConnected;
        int32_t internalValue;
    } m2MouseJointDef;

    /// No rows at all: its only effect is collideConnected=false,
    /// switching collision OFF between two bodies for its lifetime.
    typedef struct m2FilterJointDef
    {
        m2BodyId bodyIdA;
        m2BodyId bodyIdB;
        uint64_t userData;
        int32_t internalValue;
    } m2FilterJointDef;

    m2DistanceJointDef m2DefaultDistanceJointDef(void);
    m2RevoluteJointDef m2DefaultRevoluteJointDef(void);
    m2PrismaticJointDef m2DefaultPrismaticJointDef(void);
    m2WeldJointDef m2DefaultWeldJointDef(void);
    m2WheelJointDef m2DefaultWheelJointDef(void);
    m2FilterJointDef m2DefaultFilterJointDef(void);
    m2MotorJointDef m2DefaultMotorJointDef(void);
    m2MouseJointDef m2DefaultMouseJointDef(void);

    /// Joints join their bodies' sleep island: connected bodies sleep and
    /// wake together. Destroying either body destroys the joint.
    /// Thread class: writer.
    m2JointId m2CreateDistanceJoint(m2WorldId worldId, const m2DistanceJointDef* def);
    m2JointId m2CreateRevoluteJoint(m2WorldId worldId, const m2RevoluteJointDef* def);
    m2JointId m2CreatePrismaticJoint(m2WorldId worldId, const m2PrismaticJointDef* def);
    m2JointId m2CreateWeldJoint(m2WorldId worldId, const m2WeldJointDef* def);
    m2JointId m2CreateWheelJoint(m2WorldId worldId, const m2WheelJointDef* def);
    m2JointId m2CreateFilterJoint(m2WorldId worldId, const m2FilterJointDef* def);
    m2JointId m2CreateMotorJoint(m2WorldId worldId, const m2MotorJointDef* def);
    m2JointId m2CreateMouseJoint(m2WorldId worldId, const m2MouseJointDef* def);
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

    /// Break thresholds: reaction force or torque beyond these snaps
    /// the joint during the step, deterministically, and reports it in
    /// m2World_GetJointEvents. Zero (the default) means unbreakable.
    void m2Joint_SetBreakLimits(m2JointId jointId, float maxForce, float maxTorque);

    /// Runtime softness: the main row's spring (weld: linear row;
    /// mouse: the drag spring). Motor and filter joints have no
    /// spring and reject loudly. Angular variants are weld-only.
    /// Distance extras: retarget the rod length or clamp it into a
    /// hard range (accumulated impulses reset, reference-style);
    /// read the range back through m2Joint_GetLimits. All journaled.
    void m2Joint_SetSpringHertz(m2JointId jointId, float hertz);
    void m2Joint_SetSpringDampingRatio(m2JointId jointId, float dampingRatio);
    void m2Joint_SetAngularSpringHertz(m2JointId jointId, float hertz);
    void m2Joint_SetAngularSpringDampingRatio(m2JointId jointId, float dampingRatio);
    void m2DistanceJoint_SetLength(m2JointId jointId, float length);
    void m2DistanceJoint_SetLengthRange(m2JointId jointId, float minLength, float maxLength);

    /// Reaction load the joint carried on the last step, from the
    /// stored impulses times that step's inverse substep dt. This is
    /// the SAME computation the break pass compares against the
    /// limits, bit for bit, so tuning break thresholds against these
    /// readings is exact. Newtons and newton meters; zero before the
    /// first step and for invalid ids. Thread class: reader.
    float m2Joint_GetReactionForce(m2JointId jointId);
    float m2Joint_GetReactionTorque(m2JointId jointId);
    bool m2Joint_IsValid(m2JointId jointId);
    m2JointType m2Joint_GetType(m2JointId jointId);

    /// Parameter readback, completing the integrator surface: a world
    /// can be reconstructed from public getters alone (the mirror test
    /// proves it). Type-specific getters are loud on the wrong type;
    /// motor and limit reads on a distance joint return zero quietly,
    /// mirroring the setters that ignore them.
    m2Vec2 m2Joint_GetLocalAnchorA(m2JointId jointId);
    m2Vec2 m2Joint_GetLocalAnchorB(m2JointId jointId);
    m2Vec2 m2Joint_GetLocalAxisA(m2JointId jointId); // prismatic, wheel
    float m2Joint_GetLength(m2JointId jointId);      // distance
    float m2Joint_GetHertz(m2JointId jointId);       // weld: linear row
    float m2Joint_GetDampingRatio(m2JointId jointId);
    float m2Joint_GetAngularHertz(m2JointId jointId); // weld
    float m2Joint_GetAngularDampingRatio(m2JointId jointId);
    float m2Joint_GetMotorSpeed(m2JointId jointId);
    float m2Joint_GetMaxMotor(m2JointId jointId);
    bool m2Joint_IsMotorEnabled(m2JointId jointId);
    bool m2Joint_IsLimitEnabled(m2JointId jointId);
    bool m2Joint_IsSpringEnabled(m2JointId jointId); // wheel
    void m2Joint_GetLimits(m2JointId jointId, float* lower, float* upper);
    void m2Joint_GetBreakLimits(m2JointId jointId, float* maxForce, float* maxTorque);
    m2BodyId m2Joint_GetBodyA(m2JointId jointId);
    m2BodyId m2Joint_GetBodyB(m2JointId jointId);
    bool m2Joint_GetCollideConnected(m2JointId jointId);
    uint64_t m2Joint_GetUserData(m2JointId jointId);
    void m2Joint_SetUserData(m2JointId jointId, uint64_t userData); // journaled
    m2WorldId m2Joint_GetWorld(m2JointId jointId);

    /// Constraint drift right now: how far the joint currently is
    /// from what it pins. Point constraints report the anchor gap,
    /// the distance joint its length error, sliders their off-axis
    /// gap; angular drift is the unwound angle error where an angle
    /// is pinned and zero elsewhere. Thread class: reader.
    float m2Joint_GetLinearSeparation(m2JointId jointId);
    float m2Joint_GetAngularSeparation(m2JointId jointId);

    /// Motor joint runtime control (platforms retarget every frame)
    /// and readback; max torque rides m2Joint_SetMaxMotor/GetMaxMotor.
    /// Mouse joints retarget with SetTarget. All journaled.
    void m2MotorJoint_SetOffsets(m2JointId jointId, m2Vec2 linearOffset, float angularOffset);
    m2Vec2 m2MotorJoint_GetLinearOffset(m2JointId jointId);
    float m2MotorJoint_GetAngularOffset(m2JointId jointId);
    float m2MotorJoint_GetMaxForce(m2JointId jointId);
    float m2MotorJoint_GetCorrectionFactor(m2JointId jointId);
    void m2MouseJoint_SetTarget(m2JointId jointId, m2Pos2 target);
    m2Pos2 m2MouseJoint_GetTarget(m2JointId jointId);
    float m2MouseJoint_GetMaxForce(m2JointId jointId);

    /// Editor and integration walk: ascending slot order, truthful
    /// total (same contract as m2World_GetBodies). Thread class:
    /// reader.
    int32_t m2World_GetJoints(m2WorldId worldId, m2JointId* ids, int32_t capacity);
    int32_t m2Body_GetJoints(m2BodyId bodyId, m2JointId* ids, int32_t capacity);

    static const m2JointId m2_nullJointId = {0, 0, 0};

#ifdef __cplusplus
}
#endif

#endif // MAUL2D_JOINT_H
