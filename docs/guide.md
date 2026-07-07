# Maul2D guide

This is the ten-minute tour: what the engine promises, how to hold it
right, and where the sharp edges are. The headers are the reference;
this document is the map.

## The contract

Maul2D promises three things no mainstream 2D engine promises together:

1. **Cross-platform bit determinism.** The same inputs produce the
   same bits on Linux, Windows, macOS and WebAssembly, on x64, arm64
   and wasm32, under GCC, Clang, MSVC and Emscripten, in Debug and
   Release. CI enforces it on every commit: sixteen scene hashes must
   match across ten platform cells, bit for bit.
2. **Bit-exact rollback.** `m2World_Snapshot` captures the whole
   simulation as one flat block; `m2World_Restore` brings it back
   exactly. Re-simulating from a snapshot reproduces the original
   trajectory to the last bit, events included.
3. **Replayable history.** The command journal records everything you
   do to a world (creation, steps, impulses, teleports, tuning, even
   restores) and replays it into a fresh world onto the same bits.

Everything else is a consequence of holding those three lines.

## Ten lines to a world

```c
m2WorldDef def = m2DefaultWorldDef();
m2WorldId world = m2CreateWorld(&def);

m2BodyDef bodyDef = m2DefaultBodyDef();
bodyDef.type = m2_dynamicBody;
bodyDef.position = (m2Pos2){0.0, 4.0};
m2BodyId ball = m2CreateBody(world, &bodyDef);

m2ShapeDef shapeDef = m2DefaultShapeDef();
m2Circle circle = {{0.0f, 0.0f}, 0.5f};
m2CreateCircleShape(ball, &shapeDef, &circle);

for (;;) { m2World_Step(world, 1.0f / 60.0f, 4); }
```

Fixed timestep, always. Determinism starts with feeding the same dt
every step; 1/60 with 4 substeps is the tuned default.

## Ids, not pointers

Every handle (`m2BodyId`, `m2ShapeId`, `m2JointId`, `m2ChainId`,
`m2ParticleId`) is an index plus a generation. Destroyed ids stay
dead forever, even after their slot is reused; `m2Body_IsValid` and
friends answer without asserting. Handles are values: copy them,
store them, send them over the network.

## Positions are doubles

World positions are f64 (`m2Pos2`); everything else is f32. Play
three hundred kilometers from the origin and contacts, casts and
rendering stay exact. The debug draw hands you body-local vertices
next to the f64 origin so your camera can subtract before dropping
to floats.

## Sleeping and hibernation

Islands sleep together and wake together. When every dynamic body
sleeps and nothing kinematic is moving, the world hibernates: a step
costs roughly nothing and changes exactly nothing. Anything that
should wake the world does: impulses, teleports, type changes,
filter changes, gravity changes, joints breaking, destroys, water
touching a sleeper, a conveyor changing speed under its cargo.

## Events are streams, not callbacks

Nothing calls back into your code mid-step. After a step, poll:

- `m2World_GetContactEvents`: begin events carry the impact facts
  (world normal, contact points, approach speed); ends are guaranteed
  bookends, even when the contact dies by destruction, filter change
  or type change.
- `m2World_GetSensorEvents`: the same discipline for trigger volumes.
- `m2World_GetJointEvents`: joints that broke this step, with the
  force and torque that broke them.

Buffers live until the next step or restore. A restore clears them;
re-simulated steps re-emit identical streams.

## Filters, sensors, chains

Shapes carry category bits, mask bits and a group index; two shapes
collide when each side's category intersects the other's mask, and a
shared non-zero group overrides that (positive: always, negative:
never). Queries take an `m2QueryFilter`.

Sensors observe without pushing back and never affect sleep. Ask
`m2Shape_GetSensorOverlaps` who is inside right now.

Chains build one-sided terrain that bodies cannot snag on. Wind them
so the solid side is on the right walking point1 to point2: for a
floor, list points right to left. Open chains use their first and
last points as ghosts. `m2CreateChain` returns an `m2ChainId` naming
the whole run: `m2DestroyChain` removes every segment at once and
wakes whoever was resting on them, and `m2Chain_GetSegmentCount`
tells you how many links a live chain has. Chains retune with
`m2Chain_SetFriction` and `m2Chain_SetRestitution`, one op for every
link.

## Joints

Five core types (distance, revolute, prismatic, weld, wheel) with
motors, limits and springs where they belong, all tunable at runtime
through journaled setters that wake both bodies. The distance joint
doubles as a rope: give it a hard length range and the stops stay
hard even on a soft spring. The revolute joint can carry an angular
spring (`springHertz`, `springDampingRatio`, zero means off) pulling
toward the creation angle; disabling it drops its stored impulse.

Joints report the load they carried on the last step through
`m2Joint_GetReactionForce` and `m2Joint_GetReactionTorque`. These are
the same numbers the break pass compares against
`m2Joint_SetBreakLimits`, bit for bit, so a threshold tuned against a
reading behaves exactly as read. Broken joints land in the joint
event stream with the force and torque that killed them.

## Utility joints

The motor joint drives one body's transform toward offsets from
another under force and torque budgets: moving platforms retarget
with `m2MotorJoint_SetOffsets` every frame and the physics stays
honest. The mouse joint is a soft spring pulling a grab point toward
a world target (`m2MouseJoint_SetTarget`): dragging, done
deterministically and journaled like everything else. The filter
joint carries no constraint at all: it switches collision off
between two bodies for its lifetime.

The gear joint couples two bodies' spins at a ratio (positive for
meshed cogs, negative for belts) with an accumulated phase that
stays exact across any number of full turns; pin the bodies with
their own joints, the gear only owns rotation.

The pulley joint runs a rope over two fixed world points: lengthA
plus ratio times lengthB stays at the total measured when you create
it, so there is no length knob to mis-set. The B side feels ratio
times the A-side tension (a ratio-2 hoist balances double the mass);
retuning the ratio recaptures the total from the current geometry so
the machine re-decides instead of snapping, and a side that fully
unspools goes limp near its ground anchor rather than pushing.

The ratchet joint lets relative rotation run free in its sign
direction, clicking tooth by tooth, and never gives back more than
the last engaged tooth: socket wrenches, winches, turnstiles. Pin
the bodies with their own joints; the ratchet only owns rotation and
stays exact across any number of turns.

## Dominance

Give a body a higher dominance and contacts stop pushing it around:
in any pair, the higher side acts as unmovable toward the lower one,
and statics outrank everything. Enemies stop bulldozing the player,
the boss stands firm in a crate avalanche, and joints stay perfectly
symmetric because dominance touches contacts only.

## Jointed bodies and collision

Jointed bodies do not collide with each other by default (the
reference convention ragdolls expect); set `collideConnected` on any
joint def to restore contact. Destroying a filter joint turns
collision back on, with end and begin events flowing as usual.

## Body dynamics

Forces accumulate across calls and live for exactly one step
(`m2Body_ApplyForce`, `ApplyForceToCenter`, `ApplyTorque`); impulses
remain for instant changes. Linear and angular damping decay
velocity with the reference's Pade form. `m2Body_SetFixedRotation`
makes rotation a mass property (inertia recomputes, spin stops).
Sleep has two scopes: `m2Body_EnableSleep(id, false)` keeps one body
alert forever, `m2World_EnableSleeping(world, false)` wakes and
holds everyone. Every mutation here is journaled.

## Conveyors

Give a shape a tangentSpeed and its surface slides along the contact
tangent: friction drags whatever rests on it toward belt speed.
Positive speed drives riders toward +x on an upward-facing floor; a
pair sums both shapes' speeds, the reference convention. Retuning is
journaled and wakes the riders, so a stopped belt lets its cargo
sleep and a restarted one picks it back up.

## Frame helpers and live geometry

`m2Body_GetWorldPoint`/`GetLocalPoint` and their vector and
point-velocity siblings do the pose math for you, f64-safe.
`m2Body_GetJoints` walks one body's joints the same way the world
walks everything. Shapes can be reshaped live
(`m2Shape_SetCircle`/`SetCapsule`/`SetPolygon`/`SetSegment`, type
changes included): mass recomputes, touching partners wake, and the
journal replays it.

## Dormancy, mass, and blasts

`m2Body_Disable` parks a body outside the simulation without
destroying it: contacts end, riders wake, queries stop seeing it,
joints wait; `m2Body_Enable` puts it back where it stood.
`m2Body_SetMassData` overrides shape-derived mass until the next
shape change recomputes it, and `m2Body_ApplyMassFromShapes` asks
for that recompute by name. `m2World_Explode` is one deterministic
blast: full impulse inside the radius, linear falloff to zero across
the band, filtered by category, replayed by one journal op.

## Building geometry from points

`m2ComputeHull` turns a loose point cloud into a valid convex
polygon (welding near-duplicates, dropping degenerates loudly), and
`m2DecomposeOutline` goes further: hand it a simple counter-clockwise
outline of up to 64 points (a sprite silhouette) and it returns
convex pieces of at most 8 vertices that keep the area, ready to
become the shapes of a destructible body. Bad outlines (clockwise,
self-intersecting) bounce loudly instead of producing garbage.

`m2World_ShatterBody` closes the loop at runtime: it breaks a
dynamic body into one body per piece, each carrying the parent's
rigid velocity field at its own center of mass plus the parent's
spin and first-shape material, as a single journaled operation that
replays and rolls back like everything else. All or nothing: a world
too full to seat every piece refuses without moving a bit.

## Water

Fluids are opt-in at world creation: set particleCapacity on the
world def (with the pinned physics knobs: radius, density, pressure,
damping, viscous, tension, spring, elastic and powder strengths,
gravity scale) and the particle system exists for the world's whole
lifetime, inside every snapshot, journal and hash the world
produces. Emit with a position, velocity and behavior flags, destroy
by id, read positions and velocities back, and enumerate with
truthful totals; a full pool quietly returns the null id, so pace
emitters off the count.

`m2World_FillPolygonWithParticles` pours a whole pool in one call
(row-major on the reference stride, deterministic layout), and
`m2World_OverlapParticlesAABB` reads a region back with truthful
totals; a circular region is one distance filter away on your side.

Behavior flags: tensile particles attract their tensile neighbors
(surface tension), so sparse spray beads up and clings instead of
drifting apart; viscous particles drag their neighbors (honey);
powder grains repel when packed tighter than the rest stride and
never cohere (sand, rubble). Spring and elastic flags turn a fill
into a body: springs remember their spawn lengths, elastic triads
remember their spawn shape, the whole batch journals as one op and
rides every snapshot, and the nets die with their particles. Jelly
stiffness scales with the def strengths: the reference runs several
particle iterations per step where Maul pins one, so firm small
blobs want higher strengths and big soft masses want the defaults.

Water obeys the house laws: sensors are invisible to it, one-way
chain platforms hold it only on their solid side, bodies push it and
it pushes back (light things float, dense things sink), and anything
it touches wakes up. The neighbor and body-contact builds thread
deterministically (workers own static particle ranges and write
disjoint staging slots, so any worker count reaches the same bits,
which the fuzzer's threaded twin proves on random fluid worlds); the
relaxation sweeps themselves stay serial because they are
order-dependent the way the reference's are. Threading the build
only pays above a few thousand particles, so small pools stay serial
and skip the fork-join tax, and a fifteen-hundred particle pool
steps in well under a millisecond either way. Two speed facts are part of the contract: no particle
ever moves more than one diameter per step (the stability law; the
reference raises its iteration count for fast flows, Maul pins one
iteration and keeps the law explicit), and a world with live
particles never hibernates.

## Buoyancy volumes

When a full particle pool is more than a scene needs, a fluid
volume is the cheap answer: an axis-aligned activation region and a
horizontal surface line. `m2World_CreateFluidVolume` places one,
and every awake dynamic body whose origin sits in the region feels
Archimedes lift on the part of each shape below the surface (exact
for circles and polygons, a bounding-box fraction for capsules and
segments), plus linear and angular drag, plus an optional flow
current. Light bodies float, dense ones sink, and a flow carries
whatever drifts in. The waterline moves at runtime with
`m2FluidVolume_SetSurface` for a rising tide or a draining tank,
and the whole thing is snapshot and journal state like everything
else. Forces feed the ordinary accumulators, so a body that
settles at the surface and sleeps simply floats.

## Walking the world

An editor or engine layer can enumerate everything from a bare world
id: `m2World_GetBodies`, `m2World_GetJoints` and `m2World_GetChains`
fill caller arrays in ascending slot order and return the true total
even when it exceeds capacity, and `m2Body_GetShapes` does the same
per body. Types, filters and sensor flags read back through
`m2Body_GetType`, `m2Shape_GetType`, `m2Shape_GetFilter`,
`m2Shape_IsSensor`, `m2Joint_GetType` and `m2Joint_GetBodyA`/`B`.
Geometry comes back exactly as stored (`m2Shape_GetCircle` and
friends, `m2Chain_GetShapes`), and every joint parameter has a
getter mirroring its def field. The surface is complete in a
provable sense: the mirror test rebuilds a world from public getters
alone and holds hash equality with the original through 90 steps.

## The character mover

`m2World_CollideMover` gathers the collision planes touching a posed
capsule (one-sided chains included), `m2SolvePlanes` turns your
desired step into a translation that respects every plane with the
reference's accumulated-push solver, and `m2ClipVector` strips the
velocity you just spent. The sweep half of a controller is
`m2World_CastCapsuleClosest`, which already is the mover cast. All
of it is pure reading and pure math: a kinematic character built on
these replays bit-exactly like everything else.

## Sweeps and overlaps

Character controllers live on `m2World_CastCircleClosest` and its
capsule and polygon siblings: pose a shape, give it a translation,
get the closest hit with the ray conventions (fraction, facing
normal, ties to the lower shape index, fraction 0 with a zero normal
on initial overlap). The all-hits variants list everything along the
path, ascending. `m2World_OverlapCircle`/`Capsule`/`Polygon` list
every shape touching a posed shape, ascending, with truthful totals;
touching within the engine's slop skin counts. Chain segments stay
one-sided for both: approach from the ghost side and you pass
through, exactly like rays and contacts.

## Fixed steps and smooth rendering

Step the world at a fixed dt and never tie it to the frame rate. For
presentation, keep last step's transforms application-side
(enumerate bodies, copy poses), and each frame draw the blend
between previous and current by alpha = accumulator / dt. The
readers are pure and cheap; the simulation stays bit-exact because
interpolation never feeds back. The testbed's platformer shows the
difference: press I to cycle 60 Hz, 30 Hz snapped, and 30 Hz
interpolated.

## Rollback netcode in one paragraph

Snapshot every confirmed frame. When a late input arrives, restore,
apply the corrected inputs, re-step to now. The engine guarantees
the re-simulation lands on the same bits the original would have, so
divergence can only come from your input handling. If you record a
journal while doing this, the tape includes your rollbacks and
replays them faithfully; size tapes from `m2World_JournalBaseSize`.

## Hunting a divergence

When your rollback game desyncs, the question is always the same:
which step, and whose fault. `m2World_HashParts` answers both. Run
the two simulations you believe should match and compare parts after
every step: the step where any field first splits names the moment,
and the field itself names the subsystem: bodies means a transform
or velocity moved, contacts means manifolds disagree, joints means
accumulator memory, particles means the fluid. If the engine's own
gates are green on your platforms, the split is almost always float
work in game code feeding the world different inputs; the journal
will confirm it, because two tapes of the same inputs replay to the
same bits by contract.

## Diagnostics

`m2World_GetCounters` carries the scoreboard: live object counts,
solver facts, and the quiet runtime truths that Release builds
cannot assert about: emits refused by a full pool, neighbor pairs
dropped by the budget, and stale-id or wrong-type rejections on the
typed paths. `m2World_Validate` walks every invariant the world
owns (finiteness, endpoint liveness, canonical pair order, registry
consistency) and answers loudly; build with -DMAUL2D_VALIDATE=ON
and it runs itself after every step of your debug sessions.

## Threads

Readers (queries, getters, diagnostics, draw) may run concurrently
with each other. Writers (anything that mutates, including Step)
need the world to themselves. The solver's internal workers are
invisible: `worldDef.workerCount` changes speed, never results.

## Versions and formats

Snapshots and journal tapes are exact-byte artifacts of one library
version. Every snapshot carries a version stamp and every tape
carries one plus an echo of the world def that recorded it; a
mismatch of either is rejected loudly before a single byte lands.
They are rollback and replay tools, not archival formats: when an
engine release bumps the snapshot or wire version (any new state
array, any grown def), old bytes refuse to load by design, because a
silently reinterpreted byte is a corrupted world. Persist game state
you care about at your own layer; persist Maul bytes only next to
the exact engine build that wrote them.

## What Maul leaves out, on purpose

Some rival features are absent by decision, not omission. Pre-solve
callbacks run user code mid-step, which is a replay hazard by
construction; chains, filters, sensors and the filter joint cover
the standard cases deterministically. World tuning toggles (contact
hertz, speculative distance, warm starting) stay pinned because the
physics itself is part of the determinism contract. Body names are
what userData is for. Per-shape event enable flags guard costs Maul
does not have. A per-step "moved bodies" event stream is also left
out: enumerate and read transforms instead, the readers are cheap
and the set of ids is yours to manage. Per-body sleep thresholds
stay pinned for the same reason the world tuning knobs do.

## The knobs you should not turn

There is no way to disable determinism, skip the sleep bookkeeping
or opt out of event bookending. Those are not features; they are the
contract. If a knob seems missing, that is usually why.
