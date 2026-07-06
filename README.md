# Maul2D

[![ci](https://github.com/siracozmen01/Maul2D/actions/workflows/ci.yml/badge.svg)](https://github.com/siracozmen01/Maul2D/actions/workflows/ci.yml)

The 2D companion to [Maul3D](https://github.com/siracozmen01/Maul3D): same solver core, same C API style, two dimensions. This is where the solver, the determinism pipeline, and the rollback system get validated before the 3D engine takes them over.

Written in C17. MIT licensed.

## Status: 1.3

Baseline: x64 builds target AVX2+FMA (Haswell 2013+); arm64 uses
NEON. Configure with -DMAUL2D_SIMD=scalar for a portable fmaf
fallback that produces bit-identical results (CI enforces this).

The core engine is complete and gated:

- **Rigid bodies** - circles, capsules, segments, polygons (rounded too);
  static, kinematic, dynamic; proper center-of-mass handling.
- **Soft-step solver** with graph-colored parallel solving. Worker count is
  *non-semantic*: 1, 2 or 8 threads produce bit-identical trajectories, and
  a CI gate proves it.
- **Joints** - distance, revolute, prismatic, weld, wheel; motors, limits and
  suspension springs; runtime tuning that wakes and journals.
- **Sleeping** (island-coupled), **continuous collision** for bullets,
  **contact events** with strict begin/end bookending.
- **Rollback-native**: flat snapshots restore bit-exactly; a command journal
  records a session and replays it into a fresh world, byte for byte.
- **Queries** - closest ray cast and AABB overlap, canonical ordering, exact
  hundreds of kilometers from the origin (positions are f64).
- **Diagnostics** - deterministic counters, wall-clock phase profile,
  allocator hooks for engines that own their memory.

Determinism is the contract, not a feature flag: every commit must produce
bit-identical simulation on Linux/Windows/macOS, x64 and arm64,
GCC/Clang/MSVC. CI runs fourteen gated hash lines across six platform cells
and fails on a single differing bit; Debug and Release must agree too, and a
dual-backend harness holds behavior inside bands anchored to Box2D v3.

## Taste

```c
m2WorldDef def = m2DefaultWorldDef();
m2WorldId world = m2CreateWorld(&def);

// A car: two sprung, motorized wheels.
m2WheelJointDef ride = m2DefaultWheelJointDef();
ride.bodyIdA = chassis;
ride.bodyIdB = wheel;
ride.enableMotor = true;
ride.motorSpeed = -12.0f; // rad/s
ride.maxMotorTorque = 20.0f;
m2CreateWheelJoint(world, &ride);

// Rollback netcode in four lines.
int32_t size = m2World_SnapshotSize(world);
m2World_Snapshot(world, buffer, size);
/* ... mispredicted steps ... */
m2World_Restore(world, buffer, size); // bit-exact resimulation from here
```

Contributions: see [CONTRIBUTING.md](CONTRIBUTING.md).

## Learn more

The [guide](docs/guide.md) is the ten-minute tour: the determinism
contract, rollback netcode, events, filters, chains and threading.
The full Box2D license for the adapted kernels lives in
[THIRD_PARTY.md](THIRD_PARTY.md).

## Samples

Two tiny console programs under `samples/`:

- `sample_car` - a motorized, sprung two-wheeler drives itself down a
  road and prints telemetry; the world hash it ends on is identical
  on every supported platform.
- `sample_replay` - records a session into the command journal,
  replays it into a fresh world, and compares end hashes. Bit-exact,
  every time.

## Building

```
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/test_core
```

No dependencies. C17 compiler required (MSVC ≥ VS2022 17.0 - older versions
cannot disable FP contraction, which this engine requires).

## Acknowledgments

Maul2D stands on the shoulders of [Box2D](https://github.com/erincatto/box2d)
by Erin Catto (MIT). Several kernels - the polygon clipper, the soft-step
solver stage structure, joint constraint formulations, ray cast kernels and
the trigonometric approximations - are adapted from Box2D v3 sources, with
the adaptation noted in each file. The full Box2D license text lives in
[THIRD_PARTY.md](THIRD_PARTY.md). Box2D's license and copyright apply to
those portions; see its repository for the original work. The determinism
architecture (f64 hybrid positions, snapshot/rollback, the command journal,
cross-platform hash gating) is Maul's own.
