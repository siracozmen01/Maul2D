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
#include "shape_internal.h"

#define M2_TREE_COUNT 3 // one per body type (topic-02 D1)

typedef struct m2World
{
    // World-global mutable block (snapshot state).
    m2Vec2 gravity;
    uint64_t stepCount;

    // Body storage: parallel POD arrays, fixed capacity.
    int32_t bodyCapacity;
    int32_t maxBodyIndex; // high-water mark of used slots
    m2Transform* transforms;
    m2Vec2* linearVelocities;
    float* angularVelocities;
    float* gravityScales;
    uint64_t* userData;
    uint8_t* types;
    uint8_t* alive;
    int32_t* bodyShapeHead; // head of the body's shape list (-1 = none)
    float* invMass;         // dynamic bodies: derived from shapes, floored
    float* invInertia;      // about the body origin; 0 = no rotation response

    // Body id pool: FIFO free queue + generations; saturated slots retire.
    uint16_t* generations;
    int32_t* freeQueue;
    int32_t freeHead;
    int32_t freeTail;
    int32_t freeCount;
    int32_t retiredCount;

    // Shape storage (slice 2): same id discipline as bodies.
    int32_t shapeCapacity;
    int32_t maxShapeIndex;
    m2ShapeGeometry* shapeGeometry;
    float* shapeDensity;
    uint64_t* shapeUserData;
    int32_t* shapeBody; // owning body index
    int32_t* shapeNext; // body's shape list linkage (-1 = end)
    uint8_t* shapeAlive;
    uint16_t* shapeGenerations;
    int32_t* shapeFreeQueue;
    int32_t shapeFreeHead;
    int32_t shapeFreeTail;
    int32_t shapeFreeCount;
    int32_t shapeRetiredCount;

    // Broadphase: per-type trees; leaves are SHAPE proxies (topic-02 D1).
    m2DynamicTree trees[M2_TREE_COUNT];
    m2TreeNode* treeNodes[M2_TREE_COUNT];
    int32_t treeNodeCapacity;
    int32_t* proxyIds; // per shape slot; M2_NULL_NODE when absent
    uint8_t* inMoved;  // per shape slot dedup flag
    int32_t* moved;    // shape indices, consumed and cleared by step
    int32_t movedCount;
    uint64_t* pairKeys; // sorted, deduplicated (minShape << 32 | maxShape)
    int32_t pairCount;
    int32_t pairCapacity;
    uint64_t* pairScratch; // step-transient; not hashed, snapshot-benign

    uint16_t worldGeneration;
} m2World;

// White-box accessor for tests and internal modules. Returns NULL for a
// stale or null id. Not part of the public ABI.
m2World* m2World_GetInternal(m2WorldId worldId);

#endif // MAUL2D_WORLD_INTERNAL_H
