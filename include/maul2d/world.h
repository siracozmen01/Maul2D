// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#ifndef MAUL2D_WORLD_H
#define MAUL2D_WORLD_H

#include "maul2d/math.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

/// Def cookie base. Default functions set internalValue to
/// M2_COOKIE ^ (sizeof(def) << 8) ^ defTypeTag; create calls reject
/// anything else - including zero-initialized defs. The high byte is
/// always nonzero, so no legal cookie can collide with small garbage.
#define M2_COOKIE 0x6D32C0DE

    typedef struct m2WorldId
    {
        uint16_t index1; // 1-based, 0 = null
        uint16_t generation;
    } m2WorldId;

    typedef struct m2WorldDef
    {
        m2Vec2 gravity;       // meters/s^2, applied to dynamic bodies
        int32_t bodyCapacity; // fixed capacities for now (no growth yet)
        int32_t shapeCapacity;
        int32_t jointCapacity;
        /// Fluids: particleCapacity > 0 builds the particle system
        /// into the world for its whole lifetime (constant snapshot
        /// shape keeps rollback safe across every point in history).
        /// Parameters are pinned here like every other physics knob.
        int32_t particleCapacity; // 0 = no fluids
        float particleRadius;     // meters, floor 4x linear slop
        float particleDensity;    // mass per area
        float particleGravityScale;
        float particlePressureStrength; // reference water default 0.05
        float particleDampingStrength;  // reference default 1.0
        float particleViscousStrength;  // strength for viscous-FLAGGED particles (reference 0.25)
        float particlePowderStrength;   // strength for powder-flagged grains (reference 0.5)
        float particleTensilePressureStrength; // surface tension, reference 0.2
        float particleTensileNormalStrength;   // reference 0.2
        /// Solver worker threads including the caller (clamped to 8).
        /// NON-SEMANTIC by law: any worker count produces identical
        /// bits; it only changes how the same arithmetic is scheduled.
        int32_t workerCount;
        int32_t internalValue;
    } m2WorldDef;

    /// THREAD CLASSES. Every API is tagged reader or writer. Readers
    /// (queries, getters, diagnostics) may run concurrently with each
    /// other but never during m2World_Step or any writer. Writers
    /// (create/destroy, setters, impulses, Step itself) require
    /// exclusive access to their world. Distinct worlds are fully
    /// independent. The solver's own workers are internal and do not
    /// change any of this.

    /// Returns a def with pinned defaults and a valid cookie.
    /// Thread class: reader (pure).
    m2WorldDef m2DefaultWorldDef(void);

    /// Create a world. Returns the null id on invalid def (missing cookie,
    /// nonpositive capacity) or if the world registry is full.
    /// Thread class: writer (registry).
    m2WorldId m2CreateWorld(const m2WorldDef* def);

    /// Destroy a world and everything in it. Ids into it become stale.
    /// Thread class: writer.
    void m2DestroyWorld(m2WorldId worldId);

    /// Generation-checked liveness. Thread class: reader.
    bool m2World_IsValid(m2WorldId worldId);

    /// Changing gravity wakes every sleeping dynamic body: a stack
    /// must not float against a world that turned upside down. (The
    /// reference leaves sleepers floating; Maul picks honesty.) The
    /// change is journaled. Thread class: writer / reader.
    void m2World_SetGravity(m2WorldId worldId, m2Vec2 gravity);
    m2Vec2 m2World_GetGravity(m2WorldId worldId);

    /// World-wide sleep master switch, journaled. Disabling wakes
    /// every sleeping dynamic body (a sleeper must not outlive the
    /// rule that let it sleep). Thread class: writer / reader.
    /// One deterministic blast: every dynamic shape whose category
    /// passes maskBits and whose closest point lies inside
    /// radius+falloff gets an outward impulse at that point, full
    /// strength inside radius, fading linearly to zero across the
    /// falloff band. One journal op replays the whole thing.
    typedef struct m2ExplosionDef
    {
        m2Pos2 position;
        float radius;
        float falloff;
        float impulse; // newton seconds at full strength
        uint32_t maskBits;
        int32_t internalValue;
    } m2ExplosionDef;

    m2ExplosionDef m2DefaultExplosionDef(void);
    void m2World_Explode(m2WorldId worldId, const m2ExplosionDef* def);

    void m2World_EnableSleeping(m2WorldId worldId, bool flag);
    bool m2World_IsSleepingEnabled(m2WorldId worldId);

    /// Debug drawing: the engine walks its state and calls back; you
    /// render. Polygon vertices arrive BODY-LOCAL with the body's f64
    /// origin and rotation so your renderer can compose camera-relative
    /// doubles and stay steady far from the origin. Leave any callback
    /// NULL to skip that primitive. Colors are 0xRRGGBB hints.
    /// Read-only. Thread class: reader.
    typedef struct m2DebugDraw
    {
        void (*drawPolygon)(const m2Vec2* localVertices, int32_t count, m2Pos2 origin,
                            m2Rot rotation, uint32_t color, void* context);
        void (*drawCircle)(m2Pos2 center, float radius, m2Rot rotation, uint32_t color,
                           void* context);
        void (*drawCapsule)(m2Pos2 p1, m2Pos2 p2, float radius, uint32_t color, void* context);
        void (*drawSegment)(m2Pos2 p1, m2Pos2 p2, uint32_t color, void* context);
        void (*drawPoint)(m2Pos2 p, float size, uint32_t color, void* context);
        bool drawShapes;
        bool drawJoints;
        bool drawContacts;
        bool drawAABBs;
        void* context;
    } m2DebugDraw;

    void m2World_Draw(m2WorldId worldId, const m2DebugDraw* draw);

    /// Diagnostics. Counters are derived from simulation state and are
    /// therefore deterministic; profile times are wall-clock and are
    /// NOT - neither is ever snapshot state or hash input.
    typedef struct m2Profile
    {
        float stepMs;     // whole m2World_Step
        float pairsMs;    // broadphase + pair update
        float contactsMs; // narrowphase manifolds
        float solveMs;    // solver incl. CCD
        float sleepMs;    // islands + sleep accounting
    } m2Profile;

    typedef struct m2Counters
    {
        int32_t bodies;
        int32_t awakeBodies;
        int32_t shapes;
        int32_t joints;
        int32_t pairs;
        int32_t touchingPairs;
        int32_t constraints;         // solved last step
        int32_t graphColors;         // colors used last step
        int32_t overflowConstraints; // serial bucket last step
        uint64_t stepCount;
    } m2Counters;

    /// Snapshot of the last completed Step. Thread class: reader.
    /// Total kinetic energy of awake dynamic bodies, in joules.
    /// State-derived and deterministic (f64 accumulation in canonical
    /// body order) - twin worlds report identical bits. Sleeping
    /// bodies contribute zero by construction. Thread class: reader.
    double m2World_GetKineticEnergy(m2WorldId worldId);

    m2Profile m2World_GetProfile(m2WorldId worldId);
    m2Counters m2World_GetCounters(m2WorldId worldId);

    /// Advance the simulation. dt in seconds, substepCount >= 1.
    /// Deterministic: identical worlds and inputs produce bit-identical
    /// state on every supported platform. Thread class: writer.
    void m2World_Step(m2WorldId worldId, float dt, int32_t substepCount);

    /// Steps taken since creation (restored by m2World_Restore).
    /// Thread class: reader.
    uint64_t m2World_GetStepCount(m2WorldId worldId);

    /// Snapshot size in bytes for this world. Thread class: reader.
    int32_t m2World_SnapshotSize(m2WorldId worldId);

    /// Write a full snapshot into caller memory. Returns bytes written,
    /// or 0 if capacity is insufficient. Thread class: reader.
    int32_t m2World_Snapshot(m2WorldId worldId, void* buffer, int32_t capacity);

    /// Restore a snapshot taken from a world with the same def shape.
    /// Returns false on header mismatch. All sim state - bodies, id pools,
    /// step counter - returns to the snapshot instant, bit-exactly.
    /// Thread class: writer.
    bool m2World_Restore(m2WorldId worldId, const void* buffer, int32_t size);

    /// Deterministic state hash (alive bodies in index order + globals).
    /// Present in all builds: this is the desync-forensics primitive.
    /// Thread class: reader.
    uint64_t m2World_Hash(m2WorldId worldId);

    /// Command journal (the replay primitive). StartJournal embeds a
    /// full snapshot into the caller's buffer, then records every
    /// mutating call and step marker with raw IEEE-754 bit encoding.
    /// StopJournal returns the byte size (0 = overflow or not recording:
    /// loud, never truncated-silent). ReplayJournal restores the
    /// embedded snapshot and re-applies the stream; deterministic id
    /// re-minting is asserted along the way. Restore during recording
    /// stops the journal (recorded limitation). Thread class: writer.
    /// The journal's fixed cost: header plus the embedded snapshot.
    /// Size tapes as this plus room for your ops. Thread class: reader.
    int32_t m2World_JournalBaseSize(m2WorldId worldId);

    bool m2World_StartJournal(m2WorldId worldId, void* buffer, int32_t capacity);
    int32_t m2World_StopJournal(m2WorldId worldId);
    bool m2World_ReplayJournal(m2WorldId worldId, const void* data, int32_t size);

    static const m2WorldId m2_nullWorldId = {0, 0};

#ifdef __cplusplus
}
#endif

#endif // MAUL2D_WORLD_H
