# Contributing to Maul2D

Contributions are welcome. A few ground rules keep the project sane:

- **The maintainer has final say on every merge.** Expect design questions on
  anything that changes simulation behavior.
- **All CI gates must be green.** The determinism gates are non-negotiable:
  the whole point of this engine is bit-identical simulation across
  platforms, compilers, and thread counts. A PR that trades determinism for
  speed or convenience will not merge, however fast it is.
- **Sign your commits with DCO** (`git commit -s`). Contributions are MIT,
  inbound = outbound. No CLA.
- **Maul's own laws come first.** Determinism across platforms, bit-exact
  rollback, canonical ordering, loud failure: these are the constitution, and
  no outside precedent overrides them. Where Box2D, Box3D or Jolt have already
  solved a problem well, we study them and take the lesson; where their
  approach conflicts with Maul's laws, Maul wins - our snapshot, journal and
  threading layers exist precisely because the references never promised what
  this engine promises. If your PR takes a different road on a solved problem,
  say why in the body. "It also works" is thin; "it also works, and here is
  what it buys Maul" is a case.
- Formatting is enforced by the checked-in `.clang-format` - run it, don't
  debate it. Commit messages: imperative subject, body explains why, no
  trailers.
- Bug reports with a reproducing scene are gold. Determinism bug reports
  should include the platform pair and the `M2_DET_HASH` outputs.
