# Maul2D API reference

Generated from the public headers by tools/gen_api.py; the
headers are the source of truth and this file mirrors them.
For the why and how, read the [guide](guide.md).

## Base: versions, allocator, ids

```c
int32_t m2GetVersion(void);
```
Library version, encoded as major * 10000 + minor * 100 + patch. Thread class: reader (callable from any thread, no world required).

```c
const char* m2GetSimdBackend(void);
```
The SIMD backend this library was COMPILED against: "avx2", "neon" or "scalar". It is a compile-time choice, and every backend produces bit-identical results by contract; this only reports which kernels the binary carries. Thread class: reader.

```c
int32_t m2CpuSupportsBackend(void);
```
Whether the CPU running this call actually supports the compiled backend: 1 if it can run, 0 if not. An "avx2" binary needs AVX2 and FMA3 with OS wide-register support; "neon" is architectural on arm64 and "scalar" runs anywhere, so both return 1. Creating a world on a CPU that returns 0 aborts loudly rather than trapping on an illegal instruction; check this first for a graceful path, or build with -DMAUL2D_SIMD=scalar for a portable binary. Thread class: reader.

```c
void m2SetAllocator(m2AllocZeroedFn* allocZeroed, m2FreeFn* freeFn);
```

```c
uint64_t m2Hash64(uint64_t seed, const void* data, int32_t byteCount);
```
FNV-1a 64-bit hash over a byte range. This is the hash used by the determinism gates; its constants are frozen and will never change. Thread class: reader (pure function).

```c
void m2AssertFail(const char* condition, const char* file, int line);
```
Internal assertion failure sink (debug builds only). Prints and traps.

## Math: vectors, rotations, deterministic trig

```c
static inline float m2MinF(float a, float b) { #if defined(_MSC_VER) && defined(_M_X64) return _mm_cvtss_f32(_mm_min_ss(_mm_set_ss(a), _mm_set_ss(b)));
```
Pinned minimum: exactly (a < b ? a : b), in this operand order - including signed zero and NaN behavior. MSVC folds the plain ternary under value numbering (+0/-0 compare equal), so on MSVC x64 this is implemented with scalar MINSS, whose ISA-defined semantics are exactly the pinned form. NEON FMIN has different semantics and must never be used here; GCC/Clang compile the ternary faithfully (CI-verified).

```c
static inline float m2MaxF(float a, float b) { #if defined(_MSC_VER) && defined(_M_X64) return _mm_cvtss_f32(_mm_max_ss(_mm_set_ss(a), _mm_set_ss(b)));
```
Pinned maximum: exactly (a > b ? a : b), in this operand order. See m2MinF for the MSVC note (scalar MAXSS matches the pinned form).

```c
static inline float m2AbsF(float a) { union { float f;
```
Absolute value, pinned to IEEE |x| semantics: the sign bit is cleared, so m2AbsF(-0.0f) == +0.0f on every platform. Implemented as a bit mask because a ternary abs is compiler-foldable into divergent ±0 behavior (MSVC emits a sign-mask, GCC/Clang keep the branch - a real cross-cell hash break, caught by the gate).

```c
static inline float m2ClampF(float a, float lo, float hi) { return m2MaxF(lo, m2MinF(a, hi));
```
Clamp to [lo, hi] using the pinned min/max.

```c
float m2UnwindAngle(float radians);
```
Map an angle to [-pi, pi] (boundary within one rounding step of pi). Deterministic on every platform (uses only +, -, *, / and floorf). Valid input range is |radians| <= 1.0e6f (asserted in debug); beyond float precision limits an angle is meaningless anyway, and the deterministic fallback is the clamped boundary, never NaN.

```c
m2Rot m2MakeRot(float radians);
```
Build a rotation from an angle. Deterministic across platforms: does not call libm; never returns NaN (out-of-range input falls back deterministically, see m2UnwindAngle). Accuracy is a documented approximation, identical bits everywhere.

```c
float m2Atan2(float y, float x);
```
Deterministic atan2 replacement. Returns 0 for (0, 0) instead of NaN.

```c
m2Rot m2NormalizeRot(m2Rot q);
```
Renormalize a rotation. Every rotation composition site must call this immediately (drift control is part of the determinism contract). A degenerate input (zero or non-finite magnitude) returns the identity rotation - never NaN, never a non-unit result.

```c
m2Rot m2MulRot(m2Rot q, m2Rot r);
```
Compose two rotations (q followed by r), renormalized.

```c
int m2IsNormalizedRot(m2Rot q);
```
True if the rotation is unit length within tolerance.

## World: lifecycle, stepping, snapshots, queries, diagnostics

```c
m2WorldDef m2DefaultWorldDef(void);
```
THREAD CLASSES. Every API is tagged reader or writer. Readers (queries, getters, diagnostics) may run concurrently with each other but never during m2World_Step or any writer. Writers (create/destroy, setters, impulses, Step itself) require exclusive access to their world. Distinct worlds are fully independent. The solver's own workers are internal and do not change any of this. Returns a def with pinned defaults and a valid cookie. Thread class: reader (pure).

```c
m2WorldId m2CreateWorld(const m2WorldDef* def);
```
Create a world. Returns the null id on invalid def (missing cookie, nonpositive capacity) or if the world registry is full. Thread class: writer (registry).

```c
void m2DestroyWorld(m2WorldId worldId);
```
Destroy a world and everything in it. Ids into it become stale. Thread class: writer.

```c
bool m2World_IsValid(m2WorldId worldId);
```
Generation-checked liveness. Thread class: reader.

```c
bool m2World_Validate(m2WorldId worldId);
```
Walk the whole world and check its invariants: finiteness of every live transform, velocity and particle, endpoint liveness of joints and jelly nets, registry count consistency, pair-key ordering. Asserts loudly on the first violation and returns false; true means the world is sound. Pure reader; costs a full walk, so call it from debug paths. Building with -DMAUL2D_VALIDATE=ON runs it automatically after every step.

```c
void m2World_SetGravity(m2WorldId worldId, m2Vec2 gravity);
```
Changing gravity wakes every sleeping dynamic body: a stack must not float against a world that turned upside down. (The reference leaves sleepers floating; Maul picks honesty.) The change is journaled. Thread class: writer / reader.

```c
m2Vec2 m2World_GetGravity(m2WorldId worldId);
```

```c
m2ExplosionDef m2DefaultExplosionDef(void);
```

```c
void m2World_Explode(m2WorldId worldId, const m2ExplosionDef* def);
```

```c
void m2World_EnableSleeping(m2WorldId worldId, bool flag);
```

```c
bool m2World_IsSleepingEnabled(m2WorldId worldId);
```

```c
void m2World_Draw(m2WorldId worldId, const m2DebugDraw* draw);
```

```c
double m2World_GetKineticEnergy(m2WorldId worldId);
```
Snapshot of the last completed Step. Thread class: reader. Total kinetic energy of awake dynamic bodies, in joules. State-derived and deterministic (f64 accumulation in canonical body order) - twin worlds report identical bits. Sleeping bodies contribute zero by construction. Thread class: reader.

```c
m2Profile m2World_GetProfile(m2WorldId worldId);
```

```c
m2Counters m2World_GetCounters(m2WorldId worldId);
```

```c
void m2World_Step(m2WorldId worldId, float dt, int32_t substepCount);
```
Advance the simulation. dt in seconds, substepCount >= 1. Deterministic: identical worlds and inputs produce bit-identical state on every supported platform. Thread class: writer.

```c
uint64_t m2World_GetStepCount(m2WorldId worldId);
```
Steps taken since creation (restored by m2World_Restore). Thread class: reader.

```c
int32_t m2World_SnapshotSize(m2WorldId worldId);
```
Snapshot size in bytes for this world. Thread class: reader.

```c
int32_t m2World_Snapshot(m2WorldId worldId, void* buffer, int32_t capacity);
```
Write a full snapshot into caller memory. Returns bytes written, or 0 if capacity is insufficient. Thread class: reader.

```c
bool m2World_Restore(m2WorldId worldId, const void* buffer, int32_t size);
```
Restore a snapshot taken from a world with the same def shape. Returns false on header mismatch. All sim state - bodies, id pools, step counter - returns to the snapshot instant, bit-exactly. Thread class: writer.

```c
uint64_t m2World_Hash(m2WorldId worldId);
```
Deterministic state hash (alive bodies in index order + globals). Present in all builds: this is the desync-forensics primitive. Thread class: reader.

```c
m2WorldHashParts m2World_HashParts(m2WorldId worldId);
```

```c
m2FluidVolumeDef m2DefaultFluidVolumeDef(void);
```

```c
m2FluidVolumeId m2World_CreateFluidVolume(m2WorldId worldId, const m2FluidVolumeDef* def);
```
Thread class: writer. Journaled.

```c
void m2World_DestroyFluidVolume(m2FluidVolumeId volumeId);
```

```c
bool m2FluidVolume_IsValid(m2FluidVolumeId volumeId);
```

```c
void m2FluidVolume_SetSurface(m2FluidVolumeId volumeId, double surface);
```
Move the waterline at runtime (a rising tide, a draining tank). Journaled. Thread class: writer.

```c
double m2FluidVolume_GetSurface(m2FluidVolumeId volumeId);
```

```c
uint64_t m2FluidVolume_GetUserData(m2FluidVolumeId volumeId);
```

```c
int32_t m2World_JournalBaseSize(m2WorldId worldId);
```
Command journal (the replay primitive). StartJournal embeds a full snapshot into the caller's buffer, then records every mutating call and step marker with raw IEEE-754 bit encoding. StopJournal returns the byte size (0 = overflow or not recording: loud, never truncated-silent). ReplayJournal restores the embedded snapshot and re-applies the stream; deterministic id re-minting is asserted along the way. Restore during recording stops the journal (recorded limitation). Thread class: writer. The journal's fixed cost: header plus the embedded snapshot. Size tapes as this plus room for your ops. Thread class: reader.

```c
bool m2World_StartJournal(m2WorldId worldId, void* buffer, int32_t capacity);
```

```c
int32_t m2World_StopJournal(m2WorldId worldId);
```

```c
bool m2World_ReplayJournal(m2WorldId worldId, const void* data, int32_t size);
```

## Bodies: creation, dynamics, mass, readback

```c
m2BodyDef m2DefaultBodyDef(void);
```
Returns a def with pinned defaults and a valid cookie.

```c
m2BodyId m2CreateBody(m2WorldId worldId, const m2BodyDef* def);
```
Create a body. Returns the null id on invalid def, stale world, or exhausted capacity (diagnostic in debug builds). Thread class: writer.

```c
void m2DestroyBody(m2BodyId bodyId);
```
Destroy a body. The id becomes stale; the slot is recycled FIFO with a generation bump, and retires instead of wrapping. Thread class: writer.

```c
bool m2Body_IsValid(m2BodyId bodyId);
```
Generation-checked liveness. Thread class: reader.

```c
m2Transform m2Body_GetTransform(m2BodyId bodyId);
```

```c
m2Pos2 m2Body_GetPosition(m2BodyId bodyId);
```

```c
m2Rot m2Body_GetRotation(m2BodyId bodyId);
```

```c
m2Vec2 m2Body_GetLinearVelocity(m2BodyId bodyId);
```

```c
float m2Body_GetAngularVelocity(m2BodyId bodyId);
```

```c
uint64_t m2Body_GetUserData(m2BodyId bodyId);
```

```c
bool m2Body_IsAwake(m2BodyId bodyId);
```
Sleep state (topic-06). Setters and new contacts wake bodies; waking is island-transitive at the next step.

```c
void m2Body_SetAwake(m2BodyId bodyId, bool awake);
```
Manual sleep control: false forces the body to sleep NOW (velocities zero, like the reference), true wakes it. Journaled. Thread class: writer.

```c
void m2Body_SetBullet(m2BodyId bodyId, bool flag);
```

```c
void m2Body_SetUserData(m2BodyId bodyId, uint64_t userData);
```

```c
void m2Body_SetDominance(m2BodyId bodyId, int8_t dominance);
```
Contact dominance (a rival lesson worth keeping): in a pair, the higher-dominance body acts as unmovable toward the lower one. Statics outrank everything. Enemies stop pushing the player. Contacts only; joints are unaffected. Journaled.

```c
int8_t m2Body_GetDominance(m2BodyId bodyId);
```

```c
void m2Body_SetTargetTransform(m2BodyId bodyId, m2Pos2 position, m2Rot rotation, float dt);
```
Kinematic follow: sets the velocities that carry the body to the target pose over one step of the given dt. Applies via the journaled velocity setters, so replays are free.

```c
m2BodyType m2Body_GetType(m2BodyId bodyId);
```

```c
void m2Body_SetMassData(m2BodyId bodyId, m2MassData massData);
```

```c
m2MassData m2Body_GetMassData(m2BodyId bodyId);
```

```c
void m2Body_ApplyMassFromShapes(m2BodyId bodyId);
```

```c
void m2Body_Disable(m2BodyId bodyId);
```
Disable removes the body from simulation without destroying it: shapes leave the broadphase (contacts end, riders wake), joints stay attached but inert, queries no longer see it. Enable puts it back where it is. Both journaled. Thread class: writer.

```c
void m2Body_Enable(m2BodyId bodyId);
```

```c
bool m2Body_IsEnabled(m2BodyId bodyId);
```

```c
m2Vec2 m2Body_GetLocalCenter(m2BodyId bodyId); // body-frame center of mass bool m2Body_IsBullet(m2BodyId bodyId);
```

```c
float m2Body_GetGravityScale(m2BodyId bodyId);
```

```c
int32_t m2World_GetBodies(m2WorldId worldId, m2BodyId* ids, int32_t capacity);
```
Editor and integration walk: fills ids with up to capacity live body handles in ascending slot order and returns the TRUE total, even when it exceeds capacity. Thread class: reader.

```c
m2Pos2 m2Body_GetWorldPoint(m2BodyId bodyId, m2Vec2 localPoint);
```
Frame helpers (pure math on the body's pose) and the joint walk (ascending slot order, truthful total).

```c
m2Vec2 m2Body_GetLocalPoint(m2BodyId bodyId, m2Pos2 worldPoint);
```

```c
m2Vec2 m2Body_GetWorldVector(m2BodyId bodyId, m2Vec2 localVector);
```

```c
m2Vec2 m2Body_GetLocalVector(m2BodyId bodyId, m2Vec2 worldVector);
```

```c
m2Vec2 m2Body_GetWorldPointVelocity(m2BodyId bodyId, m2Pos2 worldPoint);
```

```c
m2Vec2 m2Body_GetLocalPointVelocity(m2BodyId bodyId, m2Vec2 localPoint);
```

```c
m2Pos2 m2Body_GetWorldCenterOfMass(m2BodyId bodyId);
```

```c
float m2Body_GetRotationalInertia(m2BodyId bodyId); // about the center of mass m2WorldId m2Body_GetWorld(m2BodyId bodyId);
```

```c
m2AABBResult m2Body_ComputeAABB(m2BodyId bodyId);
```
The tight AABB enclosing every shape on the body (fat tree margins excluded); a shapeless body returns a point at its origin. Thread class: reader.

```c
void m2Body_SetLinearVelocity(m2BodyId bodyId, m2Vec2 velocity);
```
Setters wake nothing yet (no sleep system in this slice) but are already journal-shaped: every mutation is a discrete command. Thread class: writer.

```c
void m2Body_SetAngularVelocity(m2BodyId bodyId, float velocity);
```

```c
void m2Body_SetTransform(m2BodyId bodyId, m2Pos2 position, m2Rot rotation);
```
Impulses act instantly on the velocity; the world point's arm is measured from the center of mass. Dynamic bodies only; the body wakes. Thread class: writer. Teleports the body: position and rotation snap, velocities are untouched, the broadphase refreshes immediately, and the body plus everything it was touching wake up (a sleeping stack must notice its support vanishing). Teleporting is not a sweep - no tunneling protection applies. Journaled. Thread class: writer.

```c
void m2Body_SetType(m2BodyId bodyId, m2BodyType type);
```
Converts the body's type in place. Becoming static zeroes the velocities; becoming dynamic recomputes mass from the shapes. The body and everything it touches wake, proxies migrate to the right tree, and stale pairs end with proper events. Journaled. Thread class: writer.

```c
void m2Body_ApplyLinearImpulse(m2BodyId bodyId, m2Vec2 impulse, m2Pos2 worldPoint);
```

```c
void m2Body_ApplyLinearImpulseToCenter(m2BodyId bodyId, m2Vec2 impulse);
```

```c
void m2Body_ApplyForce(m2BodyId bodyId, m2Vec2 force, m2Pos2 worldPoint);
```
Continuous forces: accumulated across calls, applied during the step, cleared when it ends. Waking is implied. Journaled. Thread class: writer.

```c
void m2Body_ApplyForceToCenter(m2BodyId bodyId, m2Vec2 force);
```

```c
void m2Body_ApplyTorque(m2BodyId bodyId, float torque);
```

```c
void m2Body_SetLinearDamping(m2BodyId bodyId, float damping);
```
Runtime body dynamics tuning, journaled. Fixed rotation zeroes the angular velocity and recomputes inertia; disabling sleep wakes the body so it cannot stay asleep illegally.

```c
float m2Body_GetLinearDamping(m2BodyId bodyId);
```

```c
void m2Body_SetAngularDamping(m2BodyId bodyId, float damping);
```

```c
float m2Body_GetAngularDamping(m2BodyId bodyId);
```

```c
void m2Body_SetGravityScale(m2BodyId bodyId, float scale);
```

```c
void m2Body_SetFixedRotation(m2BodyId bodyId, bool flag);
```

```c
bool m2Body_IsFixedRotation(m2BodyId bodyId);
```

```c
void m2Body_SetMotionLocks(m2BodyId bodyId, m2MotionLocks locks);
```
Set or read the per-axis motion locks. angularZ is the same lock as fixedRotation, so setting it here also fixes the rotation (and m2Body_IsFixedRotation reflects it). Changing a lock wakes the body and, for angularZ, recomputes the inertia. Journaled and snapshot state. Thread class: writer / reader.

```c
m2MotionLocks m2Body_GetMotionLocks(m2BodyId bodyId);
```

```c
void m2Body_EnableSleep(m2BodyId bodyId, bool flag);
```

```c
bool m2Body_IsSleepEnabled(m2BodyId bodyId);
```

```c
void m2Body_ApplyAngularImpulse(m2BodyId bodyId, float impulse);
```

## Shapes: geometry, filters, chains, hulls, decomposition

```c
m2ChainDef m2DefaultChainDef(void);
```

```c
m2ChainId m2CreateChain(m2BodyId bodyId, const m2ChainDef* def);
```
Creates the chain's segment shapes on the body and returns the chain's id (null on failure). The id names the whole group: m2DestroyChain removes every segment at once, ends their contacts, and wakes whoever was resting on them. Destroying the body also retires the chain id. Journaled. Thread class: writer.

```c
void m2DestroyChain(m2ChainId chainId);
```

```c
bool m2Chain_IsValid(m2ChainId chainId);
```

```c
int32_t m2Chain_GetSegmentCount(m2ChainId chainId);
```

```c
m2ShapeDef m2DefaultShapeDef(void);
```

```c
m2Polygon m2MakePolygon(const m2Vec2* points, int32_t count, float radius);
```
Validated constructors (topic-03 D4: relative thresholds, reject loudly). A returned polygon with count == 0 is invalid input.

```c
m2Polygon m2ComputeHull(const m2Vec2* points, int32_t count, float radius);
```
Convex hull of a loose point cloud (welding, collinear merging, deterministic quickhull): the doorway from sprite outlines to collision shapes. Degenerate input returns a polygon with count == 0, the loud-invalid convention.

```c
int32_t m2DecomposeOutline(const m2Vec2* points, int32_t count, m2Polygon* pieces, int32_t capacity);
```
Split a simple counter-clockwise outline (up to 64 points, no self-intersections, no holes) into convex pieces of at most 8 vertices each: the road from a sprite outline to destructible bodies. Fills up to capacity pieces and returns the truthful total (the enumeration contract). Returns 0 and asserts on invalid input (too few or too many points, clockwise winding, self-intersection, non-finite coordinates). Near-zero-area sliver pieces are welded away by validation and skipped; clean outlines lose nothing. Pure math, no world required. Thread class: reader (pure).

```c
int32_t m2World_ShatterBody(m2BodyId bodyId, const m2Polygon* pieces, int32_t pieceCount, m2BodyId* outBodies, int32_t capacity);
```
Break a dynamic body into one new dynamic body per piece, all at the parent's pose, each inheriting the parent's rigid velocity field evaluated at its own center of mass (v + w x r) plus the parent's spin, and the material and filter of the parent's first shape. Pieces are body-local polygons (pair with m2DecomposeOutline). The parent is destroyed: its joints die with it and touching sleepers wake, the ordinary destroy road. All-or-nothing: if body or shape capacity cannot seat every piece the call returns 0 and changes nothing (a full pool is a runtime fact). One journal op replays the whole shatter. Fills outBodies up to capacity, returns the piece count. Thread class: writer.

```c
m2Polygon m2MakeBox(float halfWidth, float halfHeight);
```

```c
m2ShapeId m2CreateCircleShape(m2BodyId bodyId, const m2ShapeDef* def, const m2Circle* circle);
```
Attach a shape to a body. Validation failure or exhausted capacity returns the null id; nothing is half-constructed. Dynamic bodies recompute mass from all attached shapes (explicit override comes with the solver slice). Thread class: writer.

```c
m2ShapeId m2CreateCapsuleShape(m2BodyId bodyId, const m2ShapeDef* def, const m2Capsule* capsule);
```

```c
m2ShapeId m2CreatePolygonShape(m2BodyId bodyId, const m2ShapeDef* def, const m2Polygon* polygon);
```

```c
m2ShapeId m2CreateSegmentShape(m2BodyId bodyId, const m2ShapeDef* def, const m2Segment* segment);
```

```c
bool m2Shape_IsValid(m2ShapeId shapeId);
```

```c
void m2DestroyShape(m2ShapeId shapeId);
```
Destroys one shape: touching contacts end (bookended into the next step's events), pairs are pruned, and the owning body's mass and center of mass are recomputed. Thread class: writer.

```c
void m2Shape_SetFriction(m2ShapeId shapeId, float friction);
```
Runtime material and filter tuning, journaled. Material changes apply the next time the contact is prepared; filter changes rebuild the shape's pairs immediately (ends are bookended) and wake whoever was touching it. Thread class: writer / reader.

```c
void m2Shape_SetRestitution(m2ShapeId shapeId, float restitution);
```

```c
void m2Shape_SetTangentSpeed(m2ShapeId shapeId, float speed);
```
Conveyor surface speed along the contact tangent; the pair value is the SUM of both shapes (reference mixing). Journaled.

```c
float m2Shape_GetTangentSpeed(m2ShapeId shapeId);
```

```c
void m2Shape_SetFilter(m2ShapeId shapeId, uint32_t categoryBits, uint32_t maskBits, int32_t groupIndex);
```

```c
float m2Shape_GetFriction(m2ShapeId shapeId);
```

```c
float m2Shape_GetRestitution(m2ShapeId shapeId);
```

```c
m2ShapeType m2Shape_GetType(m2ShapeId shapeId);
```

```c
bool m2Shape_IsSensor(m2ShapeId shapeId);
```

```c
void m2Shape_GetFilter(m2ShapeId shapeId, uint32_t* categoryBits, uint32_t* maskBits, int32_t* groupIndex);
```
Reads the collision filter; any out pointer may be NULL.

```c
m2Circle m2Shape_GetCircle(m2ShapeId shapeId);
```
Geometry readback for editors and gizmos: the getter must match the shape's type (checked loudly). Returned structs are the exact stored bits, in the shape's body-local frame.

```c
m2Capsule m2Shape_GetCapsule(m2ShapeId shapeId);
```

```c
m2Polygon m2Shape_GetPolygon(m2ShapeId shapeId);
```

```c
m2Segment m2Shape_GetSegment(m2ShapeId shapeId);
```

```c
m2ChainSegment m2Shape_GetChainSegment(m2ShapeId shapeId);
```

```c
float m2Shape_GetDensity(m2ShapeId shapeId);
```

```c
void m2Shape_SetCircle(m2ShapeId shapeId, const m2Circle* circle);
```
Runtime geometry: replace a shape's geometry in place, type changes included. The owner's mass recomputes, touching partners wake (a floor shrinking under a sleeper is a teleport-class change), and the broadphase refreshes. Journaled. Thread class: writer.

```c
void m2Shape_SetCapsule(m2ShapeId shapeId, const m2Capsule* capsule);
```

```c
void m2Shape_SetPolygon(m2ShapeId shapeId, const m2Polygon* polygon);
```

```c
void m2Shape_SetSegment(m2ShapeId shapeId, const m2Segment* segment);
```

```c
int32_t m2Body_GetShapes(m2BodyId bodyId, m2ShapeId* ids, int32_t capacity);
```
Enumeration walks, ascending slot order, truthful totals (same contract as m2World_OverlapAABB). Thread class: reader.

```c
int32_t m2World_GetChains(m2WorldId worldId, m2ChainId* ids, int32_t capacity);
```

```c
int32_t m2Chain_GetShapes(m2ChainId chainId, m2ShapeId* ids, int32_t capacity);
```

```c
m2WorldId m2Chain_GetWorld(m2ChainId chainId);
```
Runtime chain materials: applied to every link at once, one journal op each; takes effect at the next contact prepare, like the per-shape material setters. Thread class: writer.

```c
void m2Chain_SetFriction(m2ChainId chainId, float friction);
```

```c
void m2Chain_SetRestitution(m2ChainId chainId, float restitution);
```

```c
m2BodyId m2Shape_GetBody(m2ShapeId shapeId);
```

```c
m2WorldId m2Shape_GetWorld(m2ShapeId shapeId);
```

```c
m2ChainId m2Shape_GetParentChain(m2ShapeId shapeId); // null if free-standing m2AABBResult m2Shape_GetAABB(m2ShapeId shapeId); // tight, world space /// Point and ray queries against ONE shape. TestPoint counts /// touching within the engine's slop skin (the overlap law);
```

```c
bool m2Shape_TestPoint(m2ShapeId shapeId, m2Pos2 point);
```
GetClosestPoint returns the surface point nearest to the query, radius included; RayCast follows the world ray conventions including the one-sided chain law.

```c
m2Pos2 m2Shape_GetClosestPoint(m2ShapeId shapeId, m2Pos2 point);
```

```c
void m2Shape_SetDensity(m2ShapeId shapeId, float density); // journaled, mass recomputes void m2Shape_SetUserData(m2ShapeId shapeId, uint64_t userData); // journaled uint64_t m2Shape_GetUserData(m2ShapeId shapeId);
```

```c
float m2Body_GetMass(m2BodyId bodyId);
```
Body mass derived from attached shape densities (0 for non-dynamic).

```c
m2QueryFilter m2DefaultQueryFilter(void);
```

```c
m2RayCastResult m2World_CastRayClosest(m2WorldId worldId, m2Pos2 origin, m2Vec2 translation, m2QueryFilter filter);
```
Closest hit along origin + t * translation, t in [0, 1]. Thread class: reader.

```c
m2RayCastResult m2Shape_RayCast(m2ShapeId shapeId, m2Pos2 origin, m2Vec2 translation);
```

```c
int32_t m2World_CastRayAll(m2WorldId worldId, m2Pos2 origin, m2Vec2 translation, m2RayHit* hits, int32_t capacity, m2QueryFilter filter);
```

```c
int32_t m2World_CastCircleAll(m2WorldId worldId, const m2Circle* circle, m2Transform origin, m2Vec2 translation, m2RayHit* hits, int32_t capacity, m2QueryFilter filter);
```

```c
int32_t m2World_CastCapsuleAll(m2WorldId worldId, const m2Capsule* capsule, m2Transform origin, m2Vec2 translation, m2RayHit* hits, int32_t capacity, m2QueryFilter filter);
```

```c
int32_t m2World_CastPolygonAll(m2WorldId worldId, const m2Polygon* polygon, m2Transform origin, m2Vec2 translation, m2RayHit* hits, int32_t capacity, m2QueryFilter filter);
```

```c
int32_t m2World_CollideMover(m2WorldId worldId, const m2Capsule* mover, m2Transform origin, m2PlaneResult* results, int32_t capacity, m2QueryFilter filter);
```

```c
m2PlaneSolverResult m2SolvePlanes(m2Vec2 targetDelta, m2CollisionPlane* planes, int32_t count);
```

```c
m2Vec2 m2ClipVector(m2Vec2 vector, const m2CollisionPlane* planes, int32_t count);
```

```c
m2RayCastResult m2World_CastCircleClosest(m2WorldId worldId, const m2Circle* circle, m2Transform origin, m2Vec2 translation, m2QueryFilter filter);
```
Convex sweeps: the given shape (in its own local frame, posed by origin) slides along translation; the closest hit wins and ties break to the lower shape index. Chain segments stay one-sided: sweeps starting on the ghost side pass through. Initial overlap reports fraction 0 with a zero normal, like rays. Thread class: reader.

```c
m2RayCastResult m2World_CastCapsuleClosest(m2WorldId worldId, const m2Capsule* capsule, m2Transform origin, m2Vec2 translation, m2QueryFilter filter);
```

```c
m2RayCastResult m2World_CastPolygonClosest(m2WorldId worldId, const m2Polygon* polygon, m2Transform origin, m2Vec2 translation, m2QueryFilter filter);
```

```c
int32_t m2World_OverlapCircle(m2WorldId worldId, const m2Circle* circle, m2Transform origin, m2ShapeId* ids, int32_t capacity, m2QueryFilter filter);
```
Convex overlaps: live shapes touching the posed shape, in ascending slot order with a truthful total (the OverlapAABB contract). Chain segments are one-sided here too. Thread class: reader.

```c
int32_t m2World_OverlapCapsule(m2WorldId worldId, const m2Capsule* capsule, m2Transform origin, m2ShapeId* ids, int32_t capacity, m2QueryFilter filter);
```

```c
int32_t m2World_OverlapPolygon(m2WorldId worldId, const m2Polygon* polygon, m2Transform origin, m2ShapeId* ids, int32_t capacity, m2QueryFilter filter);
```

```c
int32_t m2World_OverlapAABB(m2WorldId worldId, m2Pos2 lower, m2Pos2 upper, m2ShapeId* results, int32_t capacity, m2QueryFilter filter);
```
Fills results with up to capacity alive shapes whose tight AABB overlaps [lower, upper], ascending shape order. Returns the total number of overlapping shapes even when it exceeds capacity. Thread class: reader.

## Joints: eleven types, motors, limits, springs, breaking

```c
m2DistanceJointDef m2DefaultDistanceJointDef(void);
```

```c
m2RevoluteJointDef m2DefaultRevoluteJointDef(void);
```

```c
m2PrismaticJointDef m2DefaultPrismaticJointDef(void);
```

```c
m2WeldJointDef m2DefaultWeldJointDef(void);
```

```c
m2WheelJointDef m2DefaultWheelJointDef(void);
```

```c
m2FilterJointDef m2DefaultFilterJointDef(void);
```

```c
m2GearJointDef m2DefaultGearJointDef(void);
```

```c
m2PulleyJointDef m2DefaultPulleyJointDef(void);
```

```c
m2RatchetJointDef m2DefaultRatchetJointDef(void);
```

```c
m2MotorJointDef m2DefaultMotorJointDef(void);
```

```c
m2MouseJointDef m2DefaultMouseJointDef(void);
```

```c
m2JointId m2CreateDistanceJoint(m2WorldId worldId, const m2DistanceJointDef* def);
```
Joints join their bodies' sleep island: connected bodies sleep and wake together. Destroying either body destroys the joint. Thread class: writer.

```c
m2JointId m2CreateRevoluteJoint(m2WorldId worldId, const m2RevoluteJointDef* def);
```

```c
m2JointId m2CreatePrismaticJoint(m2WorldId worldId, const m2PrismaticJointDef* def);
```

```c
m2JointId m2CreateWeldJoint(m2WorldId worldId, const m2WeldJointDef* def);
```

```c
m2JointId m2CreateWheelJoint(m2WorldId worldId, const m2WheelJointDef* def);
```

```c
m2JointId m2CreateFilterJoint(m2WorldId worldId, const m2FilterJointDef* def);
```

```c
m2JointId m2CreateGearJoint(m2WorldId worldId, const m2GearJointDef* def);
```

```c
m2JointId m2CreatePulleyJoint(m2WorldId worldId, const m2PulleyJointDef* def);
```

```c
m2JointId m2CreateRatchetJoint(m2WorldId worldId, const m2RatchetJointDef* def);
```

```c
m2JointId m2CreateMotorJoint(m2WorldId worldId, const m2MotorJointDef* def);
```

```c
m2JointId m2CreateMouseJoint(m2WorldId worldId, const m2MouseJointDef* def);
```

```c
void m2DestroyJoint(m2JointId jointId);
```

```c
void m2Joint_SetMotorSpeed(m2JointId jointId, float speed);
```
Runtime joint tuning. Motor speed is rad/s on revolute and wheel joints, m/s on prismatic; max motor is a torque or force budget accordingly. Every change wakes both bodies and is journaled. Distance joints ignore motor and limit parameters.

```c
void m2Joint_SetMaxMotor(m2JointId jointId, float maxTorqueOrForce);
```

```c
void m2Joint_EnableMotor(m2JointId jointId, bool enable);
```

```c
void m2Joint_EnableLimit(m2JointId jointId, bool enable);
```

```c
void m2Joint_SetLimits(m2JointId jointId, float lower, float upper);
```

```c
void m2Joint_SetBreakLimits(m2JointId jointId, float maxForce, float maxTorque);
```
Break thresholds: reaction force or torque beyond these snaps the joint during the step, deterministically, and reports it in m2World_GetJointEvents. Zero (the default) means unbreakable.

```c
void m2Joint_SetSpringHertz(m2JointId jointId, float hertz);
```
Runtime softness: the main row's spring (weld: linear row; mouse: the drag spring). Motor and filter joints have no spring and reject loudly. Angular variants are weld-only. Distance extras: retarget the rod length or clamp it into a hard range (accumulated impulses reset, reference-style); read the range back through m2Joint_GetLimits. All journaled.

```c
void m2Joint_SetSpringDampingRatio(m2JointId jointId, float dampingRatio);
```

```c
void m2Joint_SetAngularSpringHertz(m2JointId jointId, float hertz);
```

```c
void m2Joint_SetAngularSpringDampingRatio(m2JointId jointId, float dampingRatio);
```

```c
void m2DistanceJoint_SetLength(m2JointId jointId, float length);
```

```c
void m2DistanceJoint_SetLengthRange(m2JointId jointId, float minLength, float maxLength);
```

```c
float m2Joint_GetReactionForce(m2JointId jointId);
```
Reaction load the joint carried on the last step, from the stored impulses times that step's inverse substep dt. This is the SAME computation the break pass compares against the limits, bit for bit, so tuning break thresholds against these readings is exact. Newtons and newton meters; zero before the first step and for invalid ids. Thread class: reader.

```c
float m2Joint_GetReactionTorque(m2JointId jointId);
```

```c
bool m2Joint_IsValid(m2JointId jointId);
```

```c
m2JointType m2Joint_GetType(m2JointId jointId);
```

```c
m2Vec2 m2Joint_GetLocalAnchorA(m2JointId jointId);
```
Parameter readback, completing the integrator surface: a world can be reconstructed from public getters alone (the mirror test proves it). Type-specific getters are loud on the wrong type; motor and limit reads on a distance joint return zero quietly, mirroring the setters that ignore them.

```c
m2Vec2 m2Joint_GetLocalAnchorB(m2JointId jointId);
```

```c
m2Vec2 m2Joint_GetLocalAxisA(m2JointId jointId); // prismatic, wheel float m2Joint_GetLength(m2JointId jointId); // distance /// Spring-named getter aliases, symmetric with the setters; the /// short names remain and read the same registry slots. float m2Joint_GetSpringHertz(m2JointId jointId);
```

```c
float m2Joint_GetSpringDampingRatio(m2JointId jointId);
```

```c
float m2Joint_GetAngularSpringHertz(m2JointId jointId);
```

```c
float m2Joint_GetAngularSpringDampingRatio(m2JointId jointId);
```

```c
float m2Joint_GetHertz(m2JointId jointId); // weld: linear row float m2Joint_GetDampingRatio(m2JointId jointId);
```

```c
float m2Joint_GetAngularHertz(m2JointId jointId); // weld, revolute spring float m2Joint_GetAngularDampingRatio(m2JointId jointId);
```

```c
float m2Joint_GetMotorSpeed(m2JointId jointId);
```

```c
float m2Joint_GetMaxMotor(m2JointId jointId);
```

```c
bool m2Joint_IsMotorEnabled(m2JointId jointId);
```

```c
bool m2Joint_IsLimitEnabled(m2JointId jointId);
```

```c
bool m2Joint_IsSpringEnabled(m2JointId jointId); // wheel void m2Joint_GetLimits(m2JointId jointId, float* lower, float* upper);
```

```c
void m2Joint_GetBreakLimits(m2JointId jointId, float* maxForce, float* maxTorque);
```

```c
m2BodyId m2Joint_GetBodyA(m2JointId jointId);
```

```c
m2BodyId m2Joint_GetBodyB(m2JointId jointId);
```

```c
bool m2Joint_GetCollideConnected(m2JointId jointId);
```

```c
uint64_t m2Joint_GetUserData(m2JointId jointId);
```

```c
void m2Joint_SetUserData(m2JointId jointId, uint64_t userData); // journaled m2WorldId m2Joint_GetWorld(m2JointId jointId);
```

```c
float m2Joint_GetLinearSeparation(m2JointId jointId);
```
Constraint drift right now: how far the joint currently is from what it pins. Point constraints report the anchor gap, the distance joint its length error, sliders their off-axis gap; angular drift is the unwound angle error where an angle is pinned and zero elsewhere. Thread class: reader.

```c
float m2Joint_GetAngularSeparation(m2JointId jointId);
```

```c
void m2MotorJoint_SetOffsets(m2JointId jointId, m2Vec2 linearOffset, float angularOffset);
```
Motor joint runtime control (platforms retarget every frame) and readback; max torque rides m2Joint_SetMaxMotor/GetMaxMotor. Mouse joints retarget with SetTarget. All journaled.

```c
m2Vec2 m2MotorJoint_GetLinearOffset(m2JointId jointId);
```

```c
float m2MotorJoint_GetAngularOffset(m2JointId jointId);
```

```c
float m2MotorJoint_GetMaxForce(m2JointId jointId);
```

```c
float m2MotorJoint_GetCorrectionFactor(m2JointId jointId);
```

```c
void m2GearJoint_SetRatio(m2JointId jointId, float ratio); // journaled float m2GearJoint_GetRatio(m2JointId jointId);
```

```c
void m2PulleyJoint_SetRatio(m2JointId jointId, float ratio); // journaled float m2PulleyJoint_GetRatio(m2JointId jointId);
```
Retuning a pulley recaptures the rope total from the current geometry so the machine does not snap; accumulated impulse is dropped like a distance retarget. Lengths read live.

```c
float m2PulleyJoint_GetLengthA(m2JointId jointId);
```

```c
float m2PulleyJoint_GetLengthB(m2JointId jointId);
```

```c
m2Pos2 m2PulleyJoint_GetGroundAnchorA(m2JointId jointId);
```

```c
m2Pos2 m2PulleyJoint_GetGroundAnchorB(m2JointId jointId);
```

```c
float m2RatchetJoint_GetRatchet(m2JointId jointId);
```

```c
float m2RatchetJoint_GetPhase(m2JointId jointId);
```

```c
void m2MouseJoint_SetTarget(m2JointId jointId, m2Pos2 target);
```

```c
m2Pos2 m2MouseJoint_GetTarget(m2JointId jointId);
```

```c
float m2MouseJoint_GetMaxForce(m2JointId jointId);
```

```c
int32_t m2World_GetJoints(m2WorldId worldId, m2JointId* ids, int32_t capacity);
```
Editor and integration walk: ascending slot order, truthful total (same contract as m2World_GetBodies). Thread class: reader.

```c
int32_t m2Body_GetJoints(m2BodyId bodyId, m2JointId* ids, int32_t capacity);
```

## Events: contact, sensor and joint streams

```c
m2ContactEvents m2World_GetContactEvents(m2WorldId worldId);
```
Arrays are world-owned and valid until the next m2World_Step or m2World_Restore on this world; Restore clears them. Order is canonical (deterministic across platforms and replays). Thread class: reader.

```c
m2SensorEvents m2World_GetSensorEvents(m2WorldId worldId);
```

```c
m2JointEvents m2World_GetJointEvents(m2WorldId worldId);
```

```c
int32_t m2Shape_GetSensorOverlaps(m2ShapeId sensorShapeId, m2ShapeId* overlaps, int32_t capacity);
```
Who is inside this sensor right now. Fills up to capacity shape ids in canonical pair order and returns the true total even beyond capacity. Zero for anything that is not a live sensor. Thread class: reader.

```c
int32_t m2World_GetContactData(m2WorldId worldId, m2ContactData* data, int32_t capacity);
```

## Fluids: particles, behaviors, lifetime, fills, buoyancy

```c
m2ParticleId m2World_EmitParticle(m2WorldId worldId, m2Pos2 position, m2Vec2 velocity, uint32_t flags);
```
Emit one particle at a world position. Returns the null id when the world has no particle system (asserts in Debug: that is misuse) or when the system is full (silent: a full pool is a runtime fact, pace emitters off GetParticleCount). Journaled. Thread class: writer.

```c
void m2World_DestroyParticle(m2ParticleId particleId);
```
Destroy one particle; its slot recycles FIFO under a fresh generation. Journaled. Thread class: writer.

```c
bool m2Particle_IsValid(m2ParticleId particleId);
```
Generation-checked liveness. Thread class: reader.

```c
m2Pos2 m2Particle_GetPosition(m2ParticleId particleId);
```

```c
uint32_t m2Particle_GetFlags(m2ParticleId particleId);
```

```c
void m2Particle_SetLifetime(m2ParticleId particleId, float seconds);
```
Give a particle a finite lifetime in seconds: it counts down by the step's dt and auto-destroys at the end of the step it reaches zero, in ascending slot order, deterministically and without a journal op (the countdown is state, so it replays and rolls back by itself). Zero, the default, means immortal. Journaled. Thread class: writer.

```c
float m2Particle_GetLifetime(m2ParticleId particleId);
```

```c
void m2Particle_SetUserData(m2ParticleId particleId, uint64_t userData);
```
Opaque per-particle game data, copied verbatim through snapshots and journals. Journaled. Thread class: writer/reader.

```c
uint64_t m2Particle_GetUserData(m2ParticleId particleId);
```

```c
m2Vec2 m2Particle_GetVelocity(m2ParticleId particleId);
```

```c
void m2Particle_SetVelocity(m2ParticleId particleId, m2Vec2 velocity);
```
Journaled. Thread class: writer.

```c
int32_t m2World_GetParticleCount(m2WorldId worldId);
```
Live particle count. Thread class: reader.

```c
int32_t m2World_FillPolygonWithParticles(m2WorldId worldId, const m2Polygon* polygon, m2Pos2 position, m2Vec2 velocity, uint32_t flags);
```
Fill a convex polygon (given in world space at position) with particles on the reference stride (0.75 diameters), row-major bottom-up, left to right: deterministic by construction. Stops quietly when the pool fills; returns the number emitted. Spring and elastic flags make the batch a body: springs remember their spawn lengths, elastic triads remember their spawn shape, both captured here, journaled as one op, and carried by every snapshot. Thread class: writer.

```c
int32_t m2World_OverlapParticlesAABB(m2WorldId worldId, m2Pos2 lower, m2Pos2 upper, m2ParticleId* ids, int32_t capacity);
```
Live particles whose centers lie inside the box: ascending slot order, truthful total, NULL ids with zero capacity is a count query (the enumeration contract). Circular regions are one distance filter away on the caller's side. Thread class: reader.

```c
int32_t m2World_GetParticles(m2WorldId worldId, m2ParticleId* ids, int32_t capacity);
```
Fill ids with live particles in ascending slot order; returns the truthful total even beyond capacity (the enumeration contract). NULL ids with zero capacity is a count query. Thread class: reader.

---

271 functions across 8 headers.
