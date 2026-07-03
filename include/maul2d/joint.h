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

    /// Pins two local anchor points together (a hinge). Motors and
    /// limits arrive in a later slice (F-T5-4).
    typedef struct m2RevoluteJointDef
    {
        m2BodyId bodyIdA;
        m2BodyId bodyIdB;
        m2Vec2 localAnchorA;
        m2Vec2 localAnchorB;
        float hertz; // 0 = stiff default
        float dampingRatio;
        int32_t internalValue;
    } m2RevoluteJointDef;

    m2DistanceJointDef m2DefaultDistanceJointDef(void);
    m2RevoluteJointDef m2DefaultRevoluteJointDef(void);

    /// Joints join their bodies' sleep island: connected bodies sleep and
    /// wake together. Destroying either body destroys the joint.
    /// Thread class: writer.
    m2JointId m2CreateDistanceJoint(m2WorldId worldId, const m2DistanceJointDef* def);
    m2JointId m2CreateRevoluteJoint(m2WorldId worldId, const m2RevoluteJointDef* def);
    void m2DestroyJoint(m2JointId jointId);
    bool m2Joint_IsValid(m2JointId jointId);

    static const m2JointId m2_nullJointId = {0, 0, 0};

#ifdef __cplusplus
}
#endif

#endif // MAUL2D_JOINT_H
