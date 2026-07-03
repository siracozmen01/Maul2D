// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Internal world layout. Tests include this for white-box oracles; it is
// never installed. Every array below is a snapshot block.

#ifndef MAUL2D_WORLD_INTERNAL_H
#define MAUL2D_WORLD_INTERNAL_H

#include "dynamic_tree.h"
#include "maul2d/body.h"
#include "maul2d/events.h"
#include "maul2d/joint.h"
#include "maul2d/world.h"
#include "platform_thread.h"
#include "shape_internal.h"

#define M2_TREE_COUNT 3 // one per body type (topic-02 D1)

// Wide-lane contact solving (topic-08 phase 2). Lane width is a
// layout constant, never a semantics knob: every lane runs the same
// scalar IEEE sequence, so lane packing cannot move a bit.
#define M2_LANES 8
int32_t m2ContactBlockScratchBytes(int32_t pairCapacity);

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
    m2Vec2* localCenters;   // body-frame center of mass (snapshot state)
    uint8_t* asleep;        // sleep flag (snapshot + hashed: the sleep law)
    float* sleepTimes;      // seconds under tolerance (snapshot + hashed)
    uint8_t* sleepStreak;   // step-ends asleep, saturating at 2 (snapshot state)
    uint8_t* bullets;       // isBullet flag (snapshot + hashed)

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
    float* shapeFriction;
    float* shapeRestitution;
    uint64_t* shapeUserData;
    int32_t* shapeBody; // owning body index
    int32_t* shapeNext; // body's shape list linkage (-1 = end)
    uint8_t* shapeAlive;
    uint32_t* shapeCategory; // collision filter (snapshot state)
    uint32_t* shapeMask;
    int32_t* shapeGroup;
    uint16_t* shapeGenerations;
    int32_t* shapeFreeQueue;
    int32_t shapeFreeHead;
    int32_t shapeFreeTail;
    int32_t shapeFreeCount;
    int32_t shapeRetiredCount;

    // Joints (slice 8): same id discipline; impulses are warm-start
    // snapshot state. type: 0 = distance, 1 = revolute.
    int32_t jointCapacity;
    int32_t maxJointIndex;
    uint8_t* jointType;
    uint8_t* jointAlive;
    int32_t* jointBodyA;
    int32_t* jointBodyB;
    m2Vec2* jointLocalAnchorA;
    m2Vec2* jointLocalAnchorB;
    float* jointLength;
    float* jointHertz;
    float* jointDamping;
    m2Vec2* jointImpulse; // distance uses .x only
    uint8_t* jointFlags;  // bit0 enableMotor, bit1 enableLimit, bit2 enableSpring
    float* jointMotorSpeed;
    float* jointMaxMotor; // torque (revolute) or force (prismatic)
    float* jointLower;
    float* jointUpper;
    m2Vec2* jointLocalAxisA; // prismatic
    float* jointRefAngle;    // relative angle captured at create
    float* jointMotorImpulse;
    float* jointLowerImpulse;
    float* jointUpperImpulse;
    uint16_t* jointGenerations;
    int32_t* jointFreeQueue;
    int32_t jointFreeHead;
    int32_t jointFreeTail;
    int32_t jointFreeCount;
    int32_t jointRetiredCount;

    // Broadphase: per-type trees; leaves are SHAPE proxies (topic-02 D1).
    m2DynamicTree trees[M2_TREE_COUNT];
    m2TreeNode* treeNodes[M2_TREE_COUNT];
    int32_t treeNodeCapacity;
    int32_t* proxyIds; // per shape slot; M2_NULL_NODE when absent
    uint8_t* inMoved;  // per shape slot dedup flag
    int32_t* moved;    // shape indices, consumed and cleared by step
    int32_t movedCount;
    uint64_t* pairKeys;    // sorted, deduplicated (minShape << 32 | maxShape)
    uint8_t* pairTouching; // manifold had points last step (snapshot state)
    int32_t pairCount;
    int32_t pairCapacity;
    uint64_t* pairScratch; // step-transient; not hashed, snapshot-benign

    // Contacts (slice 3): manifolds[i] belongs to pairKeys[i]. Warm-start
    // impulses live here, so the block is snapshot state (topic-04 §4).
    m2Manifold* manifolds;
    int32_t oldPairCount;        // step-transient
    uint64_t* oldPairScratch;    // step-transient
    m2Manifold* manifoldScratch; // step-transient

    // Solver scratch (slice 4): all step-transient, zeroed at prepare.
    m2Vec2* deltaPositions; // f32 position deltas within the step
    m2Rot* deltaRotations;
    void* constraintScratch;  // m2ContactConstraint[pairCapacity]
    void* contactBlocks;      // wide SoA blocks (step-transient)
    int32_t* islandParent;    // union-find scratch (step-transient)
    uint8_t* islandDisturbed; // island flags scratch (step-transient)
    m2Pos2* ccdPrevPositions; // bullet substep origins (step-transient)
    uint8_t* touchingScratch; // pair-touching carry scratch (step-transient)
    int32_t* queryScratch;    // shapeCapacity ints (query-transient, never snapshot)
    m2ThreadPool* pool;       // NULL = serial; never snapshot state
    m2Profile profile;        // diagnostics only (never walked/hashed)
    int32_t lastConstraintCount;
    int32_t lastGraphColors;
    int32_t lastOverflow;
    uint32_t* colorMasks;      // per body: colors already used (step-transient)
    uint8_t* constraintColors; // per constraint (step-transient)
    int32_t* colorOrder;       // constraints sorted by color (step-transient)

    // Event buffers (world-owned observer stream; cleared at Step start
    // and by Restore; never snapshot state).
    m2ContactBeginEvent* beginEvents;
    int32_t beginEventCount;
    m2ContactEndEvent* endEvents;
    int32_t endEventCount;
    m2ContactEndEvent* pendingEndEvents; // between-step destroys, flushed at Step
    int32_t pendingEndCount;

    // Journal recorder (observer state; never snapshot state).
    uint8_t* journal;
    int32_t journalCapacity;
    int32_t journalCursor;
    uint8_t journalActive;
    uint8_t journalOverflow;

    uint16_t worldGeneration;
    uint16_t worldIndex0; // registry slot + 1, for building public ids
} m2World;

// Journal recording hook (src/journal.c): appends op + payload.
// Allocation goes through the user hooks (m2SetAllocator).
void* m2AllocZeroed(size_t bytes);
void m2Free(void* memory);

void m2JournalRecord(m2World* world, uint8_t op, const void* payload, int32_t bytes);
void m2JournalRecordRestore(m2World* world, const void* snapshot, int32_t size);
void m2SetJointParamInternal(m2World* world, m2JointId jointId, uint8_t param, float value);

// Journal ops (fixed-size payloads, little-endian raw structs).
enum
{
    m2_opStep = 1,
    m2_opCreateBody = 2,
    m2_opDestroyBody = 3,
    m2_opSetLinearVelocity = 4,
    m2_opSetAngularVelocity = 5,
    m2_opCreateShape = 6,
    m2_opCreateDistanceJoint = 7,
    m2_opCreateRevoluteJoint = 8,
    m2_opDestroyJoint = 9,
    m2_opCreatePrismaticJoint = 10,
    m2_opCreateWeldJoint = 11,
    m2_opCreateWheelJoint = 12,
    m2_opDestroyShape = 13,
    m2_opApplyLinearImpulse = 14,
    m2_opApplyAngularImpulse = 15,
    m2_opSetJointParam = 16,
    m2_opSetTransform = 17,
    m2_opSetType = 18,
    m2_opRestore = 19, // variable length: i32 size + snapshot bytes
};

// Journaled joint parameter channel (op 16).
enum
{
    m2_jointParamMotorSpeed = 0,
    m2_jointParamMaxMotor = 1,
    m2_jointParamEnableMotor = 2,
    m2_jointParamEnableLimit = 3,
    m2_jointParamLower = 4,
    m2_jointParamUpper = 5,
};

// Islands & sleep (src/island.c).
void m2UpdateIslandsAndWake(m2World* world);
void m2UpdateSleep(m2World* world, float dt);

// Bullet continuous collision, after each substep (src/ccd.c).
void m2SolveContinuous(m2World* world);

// Soft-step solve for one step (src/solver.c).
void m2SolveStep(m2World* world, float dt, int32_t substepCount);
// sizeof(m2ContactConstraint): the scratch block is sized by its owner.
int32_t m2ContactConstraintSize(void);

// White-box accessor for tests and internal modules. Returns NULL for a
// stale or null id. Not part of the public ABI.
m2World* m2World_GetInternal(m2WorldId worldId);

#endif // MAUL2D_WORLD_INTERNAL_H
