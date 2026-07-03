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
        int32_t internalValue;
    } m2WorldDef;

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

    static const m2WorldId m2_nullWorldId = {0, 0};

#ifdef __cplusplus
}
#endif

#endif // MAUL2D_WORLD_H
