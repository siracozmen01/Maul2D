// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Internal world layout. Tests include this for white-box oracles; it is
// never installed. Every array below is a snapshot block.

#ifndef MAUL2D_WORLD_INTERNAL_H
#define MAUL2D_WORLD_INTERNAL_H

#include "dynamic_tree.h"
#include "maul2d/body.h"
#include "maul2d/world.h"

#define M2_TREE_COUNT 3 // one per body type (topic-02 D1)

typedef struct m2World
{
    // World-global mutable block (snapshot state).
    m2Vec2 gravity;
    uint64_t stepCount;

    // Body storage: parallel POD arrays, fixed capacity (slice 0).
    int32_t bodyCapacity;
    int32_t maxBodyIndex; // high-water mark of used slots
    m2Transform* transforms;
    m2Vec2* linearVelocities;
    float* angularVelocities;
    float* gravityScales;
    uint64_t* userData;
    uint8_t* types;
    uint8_t* alive;

    // Id pool: FIFO free queue + generations. Saturated slots retire.
    uint16_t* generations;
    int32_t* freeQueue;
    int32_t freeHead;
    int32_t freeTail;
    int32_t freeCount;
    int32_t retiredCount;

    // Broadphase (slice 1): per-type trees over one shared node capacity,
    // per-body proxies, persistent moved set, persistent canonical pairs.
    m2DynamicTree trees[M2_TREE_COUNT];
    m2TreeNode* treeNodes[M2_TREE_COUNT];
    int32_t treeNodeCapacity;
    int32_t* proxyIds; // per body slot; M2_NULL_NODE when absent
    uint8_t* inMoved;  // per body slot dedup flag
    int32_t* moved;    // body indices, consumed and cleared by step
    int32_t movedCount;
    uint64_t* pairKeys; // sorted, deduplicated (minIndex << 32 | maxIndex)
    int32_t pairCount;
    int32_t pairCapacity;
    uint64_t* pairScratch; // step-transient; not hashed, but snapshot-benign

    uint16_t worldGeneration;
} m2World;

// White-box accessor for tests and internal modules. Returns NULL for a
// stale or null id. Not part of the public ABI.
m2World* m2World_GetInternal(m2WorldId worldId);

#endif // MAUL2D_WORLD_INTERNAL_H
