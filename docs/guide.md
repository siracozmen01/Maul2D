#Maul2D guide

This is the ten - minute tour : what the engine promises, how to hold it right,
    and where the sharp edges are.The headers are the reference;
this document is the map.

## The contract

Maul2D promises three things no mainstream 2D engine promises together:

1. **Cross-platform bit determinism.** The same inputs produce the same
   bits on Linux, Windows and macOS, on x64 and arm64, under GCC, Clang
   and MSVC, in Debug and Release. This is enforced by CI on every
   commit: fourteen scene hashes must match across eight platform
   cells, bit for bit.
2. **Bit-exact rollback.** `m2World_Snapshot` captures the whole
   simulation as one flat block;
`m2World_Restore` brings it back exactly.Re -
    simulating from a snapshot reproduces the original trajectory to the last bit,
    events included.3. *
        *Replayable history.**The command journal records everything you do to a
                              world(creation, steps, impulses, teleports, tuning,
                                    even restores) and replays it into a fresh world onto the same
                              bits.

                              Everything else is a consequence of holding those three lines.

                              ##Ten lines to a world

```c m2WorldDef def = m2DefaultWorldDef();
m2WorldId world = m2CreateWorld(&def);

m2BodyDef bodyDef = m2DefaultBodyDef();
bodyDef.type = m2_dynamicBody;
bodyDef.position = (m2Pos2){0.0, 4.0};
m2BodyId ball = m2CreateBody(world, &bodyDef);

m2ShapeDef shapeDef = m2DefaultShapeDef();
m2Circle circle = {{0.0f, 0.0f}, 0.5f};
m2CreateCircleShape(ball, &shapeDef, &circle);

for (;;)
{
    m2World_Step(world, 1.0f / 60.0f, 4);
}
```

    Fixed timestep,
    always.Determinism starts with feeding the same dt every step;
1 / 60 with 4 substeps is the tuned default.

    ##Ids,
    not pointers

            Every handle(`m2BodyId`, `m2ShapeId`, `m2JointId`)
                is an index plus a generation.Destroyed ids stay dead forever,
    even after their slot is reused;
`m2Body_IsValid` and friends answer without asserting.Handles are values : copy them, store them,
    send them over the network.

    ##Positions are doubles

        World positions are f64(`m2Pos2`);
everything else is f32.Play three hundred kilometers from the origin and contacts,
    casts and rendering stay exact.The debug draw hands you body -
        local vertices next to the f64 origin so your camera can subtract before dropping to floats
            .

        ##Sleeping and hibernation

            Islands sleep together and wake together.When every dynamic body sleeps and
                nothing kinematic is moving,
    the world hibernates : a step costs roughly nothing and changes exactly nothing.Anything
                               that should wake the world does : impulses,
    teleports, type changes, filter changes, gravity changes, joints breaking,
    destroys.

    ##Events are streams,
    not callbacks

            Nothing calls back into your code mid -
        step.After a step,
    poll :

    - `m2World_GetContactEvents`: begin events carry the impact
                                  facts(world normal, contact points, approach speed); ends are guaranteed
  bookends, even when the contact dies by destruction, filter change
  or type change.
- `m2World_GetSensorEvents`: the same discipline for trigger volumes.
- `m2World_GetJointEvents`: joints that broke this step, with the
  force and torque that broke them.

Buffers live until the next step or restore. A restore clears them;
re - simulated steps re -
    emit identical streams.

    ##Filters,
    sensors,
    chains

        Shapes carry category bits,
    mask bits and a group index; two shapes
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
tells you how many links a live chain has.

Joints report the load they carried on the last step through
`m2Joint_GetReactionForce` and `m2Joint_GetReactionTorque`. These are
the same numbers the break pass compares against
`m2Joint_SetBreakLimits`, bit for bit, so a threshold tuned against a
reading behaves exactly as read.

## Walking the world

An editor or engine layer can enumerate everything from a bare world
id: `m2World_GetBodies`, `m2World_GetJoints` and `m2World_GetChains`
fill caller arrays in ascending slot order and return the true total
even when it exceeds capacity, and `m2Body_GetShapes` does the same
per body. Types, filters and sensor flags read back through
`m2Body_GetType`, `m2Shape_GetType`, `m2Shape_GetFilter`,
`m2Shape_IsSensor`, `m2Joint_GetType` and `m2Joint_GetBodyA`/`B`.
Geometry comes back exactly as stored (`m2Shape_GetCircle` and
friends, `m2Chain_GetShapes`), and every joint parameter has a getter
mirroring its def field. The surface is complete in a provable sense:
the mirror test rebuilds a world from public getters alone and holds
hash equality with the original through 90 steps.

## Rollback netcode in one paragraph

Snapshot every confirmed frame. When a late input arrives, restore,
apply the corrected inputs, re-step to now. The engine guarantees the
re-simulation lands on the same bits the original would have, so
divergence can only come from your input handling. If you record a
journal while doing this, the tape includes your rollbacks and
replays them faithfully;
size tapes from `m2World_JournalBaseSize`.

    ##Threads

    Readers(queries, getters, diagnostics, draw)
may run concurrently with each
    other.Writers(anything that mutates, including Step) need the world to
    themselves.The solver's internal workers are invisible:
`worldDef.workerCount` changes speed,
    never results.

    ##The knobs you should not turn

    There is no way to disable determinism,
    skip the sleep bookkeeping or opt out of event bookending.Those are not features;
they are the contract.If a knob seems missing, that is usually why.
