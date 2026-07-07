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
#include "maul2d/particle.h"
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
    uint8_t sleepEnabled; // world-wide sleep master switch (snapshot state)
    float lastInvH;       // inverse substep dt of the last solve (snapshot state)

    // Body storage: parallel POD arrays, fixed capacity.
    int32_t bodyCapacity;
    int32_t maxBodyIndex; // high-water mark of used slots
    m2Transform* transforms;
    m2Vec2* linearVelocities;
    float* angularVelocities;
    float* gravityScales;
    float* linearDampings;   // Pade-damped in integrate (snapshot state)
    float* angularDampings;  // (snapshot state)
    uint8_t* fixedRotations; // invInertia forced 0 (snapshot state)
    uint8_t* sleepEnables;   // 0 = this body never sleeps (snapshot state)
    m2Vec2* forces;          // accumulated, cleared at step end (snapshot state)
    uint8_t* disabled;       // 1 = outside simulation, shapes proxy-less (snapshot state)
    int8_t* dominances;      // contact priority (snapshot state)
    float* torques;          // (snapshot state)
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
    uint8_t* shapeSensor;     // overlap-only shapes (snapshot state)
    float* shapeTangentSpeed; // conveyor surface speed (snapshot state)
    int32_t* shapeChain;      // owning chain slot, -1 = free-standing (snapshot state)
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
    float* jointHertz2; // weld: angular row (linear rides jointHertz)
    float* jointDamping2;
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
    float* jointSpringImpulse; // revolute angular spring accumulator
    float* jointBreakForce;    // 0 = unbreakable (snapshot state)
    uint8_t* jointCollide;     // 0 = connected bodies never pair (snapshot state)
    m2Pos2* jointTargets;      // mouse joints: world target (snapshot state)
    m2Pos2* jointTargetsB;     // pulley: second ground anchor (snapshot state)
    uint64_t* jointUserData;   // opaque (snapshot state)
    float* jointBreakTorque;
    uint16_t* jointGenerations;
    // Fluids: slot-stable SoA, FIFO recycling, never compacted (the
    // LiquidFun recipe is adopted later; its reordering storage is
    // not). All snapshot state, walked only when capacity > 0.
    m2Pos2* particlePositions;
    m2Vec2* particleVelocities;
    uint8_t* particleAlive;
    uint16_t* particleGenerations;
    uint32_t* particleFlags;    // behavior bits (snapshot state)
    float* particleLifetime;    // seconds left, 0 = immortal (snapshot state)
    uint64_t* particleUserData; // opaque game data (snapshot state)
    int32_t* particleFreeQueue;
    int32_t particleCapacity;
    int32_t particleFreeHead;
    int32_t particleFreeCount;
    int32_t particleCount;
    int32_t maxParticleIndex;
    float particleRadius;
    float particleDensity;
    float particleGravityScale;
    float particlePressureStrength;
    float particleDampingStrength;
    float particleViscousStrength;
    float particleTensilePressure;
    float particleTensileNormal;
    float particlePowderStrength;
    float particleSpringStrength;
    float particleElasticStrength;
    // Step-transient fluid scratch: rebuilt from positions every
    // step, never walked, never hashed (the island precedent).
    void* particleProxies;    // capacity * 16 bytes (key, index, pad)
    void* particleProxiesTmp; // radix ping-pong buffer, same size
    int32_t* particlePairA;
    int32_t* particlePairB;
    float* particlePairWeight;
    uint32_t* particlePairFlags; // OR of both ends (transient)
    m2Vec2* particlePairNormal;
    int32_t particlePairCapacity; // 12 per particle of capacity
    int32_t particlePairCount;
    int32_t particlePairOverflow;  // deterministic truncation counter
    float* particleWeights;        // step-transient dimensionless density
    float* particleAccumulation;   // step-transient pressure accumulator
    m2Vec2* particleAccumulation2; // step-transient tensile normals
    uint32_t particleFlagsUnion;   // OR over live pairs (transient)
    // Jelly: persistent springs and shape-restoring triads captured
    // at fill time (snapshot state, unlike the transient neighbors).
    int32_t* particleSpringA;
    int32_t* particleSpringB;
    float* particleSpringRest;
    int32_t particleSpringCapacity; // 4 per particle of capacity
    int32_t particleSpringCount;
    int32_t* particleTriadA;
    int32_t* particleTriadB;
    int32_t* particleTriadC;
    m2Vec2* particleTriadPA; // rest offsets about the triad centroid
    m2Vec2* particleTriadPB;
    m2Vec2* particleTriadPC;
    int32_t particleTriadCapacity; // 2 per particle of capacity
    int32_t particleTriadCount;
    // Step-transient particle-vs-body contacts (one per particle and
    // touched shape, canonical particle-then-shape order).
    int32_t* particleBodyParticle;
    int32_t* particleBodyBody;
    float* particleBodyWeight;
    m2Vec2* particleBodyNormal; // outward, shape toward particle, world frame
    float* particleBodyMass;    // pair-effective mass (reference formula)
    // Per-particle staging slabs for the parallel contact build:
    // 4 candidate slots per particle, compacted serially in index
    // order, so any worker count produces the same flat list.
    int32_t* particleBodyStageBody;  // 4 * capacity, -1 = empty
    float* particleBodyStageWeight;  // 4 * capacity
    m2Vec2* particleBodyStageNormal; // 4 * capacity
    float* particleBodyStageMass;    // 4 * capacity
    int32_t* particlePairWorkCount;  // per-proxy pair counts (two-pass build)
    int32_t* particleBodyStageDrops; // per-particle candidates beyond 4
    int32_t particleBodyCapacity;    // 4 per particle of capacity
    int32_t particleBodyCount;
    int32_t particleBodyOverflow;
    uint64_t particlePoolFullCount; // cumulative quiet-full refusals
    uint64_t misuseCount;           // cumulative loud rejections
    int32_t* jointFreeQueue;
    int32_t jointFreeHead;
    int32_t jointFreeTail;
    int32_t jointFreeCount;
    int32_t jointRetiredCount;

    // Chains (slice 52): a chain is a named group of segment shapes
    // on one body, so a whole ground run can be destroyed by id.
    // Slots are bounded by shape capacity: every live chain owns at
    // least one shape. Same id discipline as bodies and joints.
    int32_t maxChainIndex;
    uint8_t* chainAlive;
    int32_t* chainBody;
    uint16_t* chainGenerations;
    int32_t* chainFreeQueue;
    int32_t chainFreeHead;
    int32_t chainFreeTail;
    int32_t chainFreeCount;
    int32_t chainRetiredCount;

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
    // Buoyancy volumes: particle-free water regions (snapshot state).
    m2Pos2* fvLower;
    m2Pos2* fvUpper;
    double* fvSurface;
    float* fvDensity;
    float* fvLinearDrag;
    float* fvAngularDrag;
    m2Vec2* fvFlow;
    uint64_t* fvUserData;
    uint8_t* fvAlive;
    uint16_t* fvGenerations;
    int32_t* fvFreeQueue;
    int32_t fvCapacity;
    int32_t fvFreeHead;
    int32_t fvFreeCount;
    int32_t maxFvIndex;
    m2Profile profile; // diagnostics only (never walked/hashed)
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
    m2ContactBeginEvent* sensorBeginEvents;
    int32_t sensorBeginCount;
    m2ContactEndEvent* sensorEndEvents;
    int32_t sensorEndCount;
    m2ContactEndEvent* pendingSensorEnd;
    int32_t pendingSensorEndCount;
    m2JointBreakEvent* jointBreakEvents;
    int32_t jointBreakEventCount;
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

// One-time SIMD backend capability guard (src/core.c): aborts loudly if
// an AVX2 binary is run on a CPU without AVX2.
void m2VerifyCpuBackend(void);

void m2JournalRecord(m2World* world, uint8_t op, const void* payload, int32_t bytes);
void m2JournalRecordRestore(m2World* world, const void* snapshot, int32_t size);
typedef struct m2OpShatterHeader
{
    m2BodyId body;
    int32_t pieceCount;
    int32_t expectedFirst; // index1 of the first piece body
} m2OpShatterHeader;
void m2JournalRecordShatter(m2World* world, m2BodyId bodyId, const struct m2Polygon* pieces,
                            int32_t pieceCount, int32_t expectedFirst);
void m2JournalRecordChain(m2World* world, m2BodyId bodyId, const m2ChainDef* def,
                          int32_t createdCount);
void m2SetJointParamInternal(m2World* world, m2JointId jointId, uint8_t param, float value);
void m2DestroyJointInternal(m2World* world, int32_t index);
void m2SetShapeParamInternal(m2World* world, m2ShapeId shapeId, uint8_t param, float value);
void m2SetBodyParamInternal(m2World* world, m2BodyId bodyId, uint8_t param, float value);

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
    m2_opRestore = 19,     // variable length: i32 size + snapshot bytes
    m2_opCreateChain = 20, // variable length: def echo + i32 count + points
    m2_opSetGravity = 21,
    m2_opShapeParam = 22, // friction (0) / restitution (1)
    m2_opSetFilter = 23,
    m2_opDestroyChain = 24,
    m2_opBodyParam = 25, // linDamp(0)/angDamp(1)/gravScale(2)/fixedRot(3)/enableSleep(4)
    m2_opEnableSleeping = 26,
    m2_opApplyForce = 27,
    m2_opApplyForceCenter = 28,
    m2_opApplyTorque = 29,
    m2_opCreateFilterJoint = 30,
    m2_opCreateMotorJoint = 31,
    m2_opCreateMouseJoint = 32,
    m2_opMotorOffsets = 33,
    m2_opMouseTarget = 34,
    m2_opDisableBody = 35,
    m2_opEnableBody = 36,
    m2_opSetMassData = 37,
    m2_opMassFromShapes = 38,
    m2_opExplode = 39,
    m2_opSetGeometry = 40, // shapeId + full geometry union
    m2_opChainFriction = 41,
    m2_opChainRestitution = 42,
    m2_opImpulseCenter = 43,
    m2_opSetAwake = 44,
    m2_opSetBullet = 45,
    m2_opSetDensity = 46,
    m2_opBodyUserData = 47,
    m2_opShapeUserData = 48,
    m2_opJointUserData = 49,
    m2_opSetDominance = 50,
    m2_opCreateGearJoint = 51,
    m2_opCreatePulleyJoint = 52,
    m2_opEmitParticle = 53,
    m2_opDestroyParticle = 54,
    m2_opSetParticleVelocity = 55,
    m2_opCreateRatchetJoint = 56,
    m2_opFillParticles = 57,
    m2_opShatterBody = 58,
    m2_opSetParticleLifetime = 59,
    m2_opSetParticleUserData = 60,
    m2_opCreateFluidVolume = 61,
    m2_opDestroyFluidVolume = 62,
    m2_opSetFluidSurface = 63,
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
    m2_jointParamBreakForce = 6,
    m2_jointParamBreakTorque = 7,
    m2_jointParamHertz = 8,
    m2_jointParamDamping = 9,
    m2_jointParamAngularHertz = 10,
    m2_jointParamAngularDamping = 11,
    m2_jointParamLength = 12,    // distance only; resets impulses
    m2_jointParamMinLength = 13, // distance only; resets impulses
    m2_jointParamMaxLength = 14,
    m2_jointParamGearRatio = 15,
    m2_jointParamPulleyRatio = 16,
};

// Convex distance and casts (src/distance.c, slice 63): one GJK
// kernel for every convex proxy; callers pre-transform both proxies
// into a single body-local float frame.
typedef struct m2DistanceProxy
{
    m2Vec2 points[8];
    int32_t count;
    float radius;
} m2DistanceProxy;

typedef struct m2DistanceResult
{
    m2Vec2 pointA;
    m2Vec2 pointB;
    m2Vec2 normal; // A toward B; (0,0) when cores overlap
    float distance;
} m2DistanceResult;

typedef struct m2CastResult
{
    m2Vec2 pointA; // surface point on A's inflated boundary
    m2Vec2 normal; // (0,0) on initial overlap
    float fraction;
    bool hit;
} m2CastResult;

m2World* m2WorldFromIndex0(uint16_t index0);
m2DistanceProxy m2GeometryProxy(const struct m2ShapeGeometry* g);
m2DistanceResult m2ShapeDistance(const m2DistanceProxy* proxyA, const m2DistanceProxy* proxyB);
m2CastResult m2ShapeCastProxy(const m2DistanceProxy* proxyA, const m2DistanceProxy* proxyB,
                              m2Vec2 translation, float maxFraction);

// Islands & sleep (src/island.c).
void m2UpdateIslandsAndWake(m2World* world);
void m2UpdateSleep(m2World* world, float dt);

// Bullet continuous collision, after each substep (src/ccd.c).
void m2SolveContinuous(m2World* world);

// Soft-step solve for one step (src/solver.c).
void m2SolveStep(m2World* world, float dt, int32_t substepCount);
// Reaction magnitudes from the stored impulses: the ONE mapping
// shared by the break pass and the public getters, so the number a
// game reads is bit-for-bit the number the scissors compare.
void m2JointReactionMagnitudes(const m2World* world, int32_t j, float invH, float* force,
                               float* torque);
// sizeof(m2ContactConstraint): the scratch block is sized by its owner.
int32_t m2ContactConstraintSize(void);

// White-box accessor for tests and internal modules. Returns NULL for a
// stale or null id. Not part of the public ABI.
m2World* m2World_GetInternal(m2WorldId worldId);
void m2UpdateParticlePairs(m2World* world);
void m2SolveParticles(m2World* world, float dt);
void m2ApplyFluidVolumes(m2World* world, float dt);
#define M2_FVOLUME_COOKIE (M2_COOKIE ^ ((int32_t)sizeof(m2FluidVolumeDef) << 8) ^ 17)
float m2ShapeArea(const struct m2ShapeGeometry* g);
struct m2CastHitInternal
{
    m2Vec2 point;
    m2Vec2 normal;
    float fraction;
    bool hit;
};
struct m2CastHitInternal m2RayCastShapeIndex(const m2World* world, int32_t shapeIndex,
                                             m2Pos2 origin, m2Vec2 translation, float maxFraction);

#endif // MAUL2D_WORLD_INTERNAL_H
