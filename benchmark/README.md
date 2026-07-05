# Dual-backend harness

The referee. Every scene here runs on the reference backend (Box2D v3) and,
once Maul2D has a world, on Maul2D through the same interface. Quality is a
measured comparison, never an impression: settle behavior, jitter amplitude,
and trajectory hashes are recorded per backend and compared against bands.

Until the Maul2D backend exists, the harness runs reference-only and its
output establishes the reference bands the engine will be measured against.

Build and run:

```
cmake -B build -DMAUL2D_BENCHMARKS=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/benchmark/harness
```

Metrics per scene: settle step (first step where every body sleeps), a
transform hash at settle, jitter amplitude over the 60 steps after settle
(max displacement of any body - settled stacks must not creep), and step
time (median / p99; report-only, no band: timing belongs to fixed reference
hardware, behavior belongs everywhere).

Note on scene format: scenes are currently code. The engine's command-journal
format (the single scene format the test constitution requires) does not
exist yet; when it lands, these scenes migrate to journals and this note is
deleted. This is a recorded, temporary deviation - not a second scene format.
