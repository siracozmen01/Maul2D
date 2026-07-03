// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#ifndef MAUL2D_BODY_H
#define MAUL2D_BODY_H

#include "maul2d/world.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum m2BodyType
    {
        m2_staticBody = 0,
        m2_kinematicBody = 1,
        m2_dynamicBody = 2,
    } m2BodyType;

    typedef struct m2BodyId
    {
        int32_t index1; // 1-based, 0 = null
        uint16_t world0;
        uint16_t generation;
    } m2BodyId;

    typedef struct m2BodyDef
    {
        m2BodyType type;
        m2Pos2 position; // world space, f64 (hybrid precision)
        m2Rot rotation;
        m2Vec2 linearVelocity;
        float angularVelocity; // radians/s
        float gravityScale;
        bool isBullet;     // continuous collision vs non-bullets (topic-07)
        uint64_t userData; // opaque, copied verbatim through snapshots
        int32_t internalValue;
    } m2BodyDef;

    /// Returns a def with pinned defaults and a valid cookie.
    m2BodyDef m2DefaultBodyDef(void);

    /// Create a body. Returns the null id on invalid def, stale world,
    /// or exhausted capacity (diagnostic in debug builds).
    /// Thread class: writer.
    m2BodyId m2CreateBody(m2WorldId worldId, const m2BodyDef* def);

    /// Destroy a body. The id becomes stale; the slot is recycled FIFO
    /// with a generation bump, and retires instead of wrapping.
    /// Thread class: writer.
    void m2DestroyBody(m2BodyId bodyId);

    /// Generation-checked liveness. Thread class: reader.
    bool m2Body_IsValid(m2BodyId bodyId);

    m2Transform m2Body_GetTransform(m2BodyId bodyId);
    m2Pos2 m2Body_GetPosition(m2BodyId bodyId);
    m2Rot m2Body_GetRotation(m2BodyId bodyId);
    m2Vec2 m2Body_GetLinearVelocity(m2BodyId bodyId);
    float m2Body_GetAngularVelocity(m2BodyId bodyId);
    uint64_t m2Body_GetUserData(m2BodyId bodyId);

    /// Sleep state (topic-06). Setters and new contacts wake bodies;
    /// waking is island-transitive at the next step.
    bool m2Body_IsAwake(m2BodyId bodyId);

    /// Setters wake nothing yet (no sleep system in this slice) but are
    /// already journal-shaped: every mutation is a discrete command.
    /// Thread class: writer.
    void m2Body_SetLinearVelocity(m2BodyId bodyId, m2Vec2 velocity);
    void m2Body_SetAngularVelocity(m2BodyId bodyId, float velocity);

    /// Impulses act instantly on the velocity; the world point's arm
    /// is measured from the center of mass. Dynamic bodies only; the
    /// body wakes. Thread class: writer.
    void m2Body_ApplyLinearImpulse(m2BodyId bodyId, m2Vec2 impulse, m2Pos2 worldPoint);
    void m2Body_ApplyAngularImpulse(m2BodyId bodyId, float impulse);

    static const m2BodyId m2_nullBodyId = {0, 0, 0};

#ifdef __cplusplus
}
#endif

#endif // MAUL2D_BODY_H
