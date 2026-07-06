// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Fluids: a fixed-capacity particle system living inside the world.
// The system exists for the world's whole lifetime when the world
// def asks for it (particleCapacity > 0), so the snapshot shape
// never changes and rollback across any point in history stays
// byte-exact. Storage is slot-stable SoA with FIFO recycling and
// generation-checked ids; particles are never compacted or
// reordered. Physics parameters are pinned at world creation like
// every other Maul knob.
//
// This is the storage slice of the fluids chapter: particles emit,
// free-fall, journal and roll back. The neighbor solver and rigid
// coupling arrive in the following slices.

#ifndef MAUL2D_PARTICLE_H
#define MAUL2D_PARTICLE_H

#include "maul2d/math.h"
#include "maul2d/world.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct m2ParticleId
    {
        int32_t index1; // 1-based, 0 = null
        uint16_t world0;
        uint16_t generation;
    } m2ParticleId;

    static const m2ParticleId m2_nullParticleId = {0, 0, 0};

    /// Per-particle behavior flags, set at emit. Plain water is 0.
    /// Tensile particles attract their tensile neighbors (surface
    /// tension): droplets bead up and cling instead of dispersing.
    /// Viscous particles drag their neighbors (honey, syrup);
    /// powder grains repel when packed tighter than the rest stride
    /// and never cohere (sand, rubble, dust).
    typedef enum m2ParticleFlags
    {
        m2_waterParticle = 0,
        m2_tensileParticle = 1u << 0,
        m2_viscousParticle = 1u << 1,
        m2_powderParticle = 1u << 2,
    } m2ParticleFlags;

    /// Emit one particle at a world position. Returns the null id
    /// when the world has no particle system (asserts in Debug: that
    /// is misuse) or when the system is full (silent: a full pool is
    /// a runtime fact, pace emitters off GetParticleCount). Journaled.
    /// Thread class: writer.
    m2ParticleId m2World_EmitParticle(m2WorldId worldId, m2Pos2 position, m2Vec2 velocity,
                                      uint32_t flags);

    /// Destroy one particle; its slot recycles FIFO under a fresh
    /// generation. Journaled. Thread class: writer.
    void m2World_DestroyParticle(m2ParticleId particleId);

    /// Generation-checked liveness. Thread class: reader.
    bool m2Particle_IsValid(m2ParticleId particleId);

    m2Pos2 m2Particle_GetPosition(m2ParticleId particleId);
    uint32_t m2Particle_GetFlags(m2ParticleId particleId);
    m2Vec2 m2Particle_GetVelocity(m2ParticleId particleId);

    /// Journaled. Thread class: writer.
    void m2Particle_SetVelocity(m2ParticleId particleId, m2Vec2 velocity);

    /// Live particle count. Thread class: reader.
    int32_t m2World_GetParticleCount(m2WorldId worldId);

    /// Fill a convex polygon (given in world space at position) with
    /// particles on the reference stride (0.75 diameters), row-major
    /// bottom-up, left to right: deterministic by construction. Stops
    /// quietly when the pool fills; returns the number emitted.
    /// Thread class: writer (a composition of journaled emits).
    int32_t m2World_FillPolygonWithParticles(m2WorldId worldId, const m2Polygon* polygon,
                                             m2Pos2 position, m2Vec2 velocity, uint32_t flags);

    /// Fill ids with live particles in ascending slot order; returns
    /// the truthful total even beyond capacity (the enumeration
    /// contract). NULL ids with zero capacity is a count query.
    /// Thread class: reader.
    int32_t m2World_GetParticles(m2WorldId worldId, m2ParticleId* ids, int32_t capacity);

#ifdef __cplusplus
}
#endif

#endif
