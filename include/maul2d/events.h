// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#ifndef MAUL2D_EVENTS_H
#define MAUL2D_EVENTS_H

#include "maul2d/shape.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /// Contact lifecycle events (polled, never called back). Begin/end
    /// pairing is an API contract: every path that kills a touching
    /// contact emits the end event - separation, pair loss, and shape or
    /// body destruction included. Events carry the index of the step
    /// they belong to; after a rollback, re-simulated steps re-emit
    /// their events (dedup by {step, shape ids} on the host side).
    typedef struct m2ContactBeginEvent
    {
        m2ShapeId shapeIdA; // lower shape index of the pair
        m2ShapeId shapeIdB;
        uint64_t step;
    } m2ContactBeginEvent;

    typedef struct m2ContactEndEvent
    {
        m2ShapeId shapeIdA; // ids as they were when touch ended
        m2ShapeId shapeIdB;
        uint64_t step;
    } m2ContactEndEvent;

    typedef struct m2ContactEvents
    {
        const m2ContactBeginEvent* beginEvents;
        const m2ContactEndEvent* endEvents;
        int32_t beginCount;
        int32_t endCount;
    } m2ContactEvents;

    /// Arrays are world-owned and valid until the next m2World_Step or
    /// m2World_Restore on this world; Restore clears them. Order is
    /// canonical (deterministic across platforms and replays).
    /// Thread class: reader.
    m2ContactEvents m2World_GetContactEvents(m2WorldId worldId);

    /// Sensor overlap stream: same shapes-and-step records, separate
    /// buffers, same bookending guarantees (a destroyed or filtered
    /// overlap always emits its end). Thread class: reader.
    typedef struct m2SensorEvents
    {
        const m2ContactBeginEvent* beginEvents;
        const m2ContactEndEvent* endEvents;
        int32_t beginCount;
        int32_t endCount;
    } m2SensorEvents;

    m2SensorEvents m2World_GetSensorEvents(m2WorldId worldId);

    /// A read-only view of the contacts touching right now, canonical
    /// pair order. Returns the total touching count even beyond
    /// capacity. Thread class: reader.
    typedef struct m2ContactData
    {
        m2ShapeId shapeIdA;
        m2ShapeId shapeIdB;
        m2Vec2 normal; // world frame, from A to B
        int32_t pointCount;
        float separations[2];
        float normalImpulses[2];
        float tangentImpulses[2];
    } m2ContactData;

    int32_t m2World_GetContactData(m2WorldId worldId, m2ContactData* data, int32_t capacity);

#ifdef __cplusplus
}
#endif

#endif // MAUL2D_EVENTS_H
