# Changelog

## 0.1.1

- Restore the optional RPO `objective_evaluator` from Jakob's August 26 source
  through PSO search, every refinement operation and final reporting.
- Preserve default legacy/manuscript behavior and the later threaded path-binding fix.
- Test the pinned comparison consumer and retain reference outputs from 0.1.0.
- Require a SpaceAGORA revision explicitly accepting provider 0.1.1.

## 0.1.0

- Introduce standalone swarm state, search policies, serial search and particle updates.
- Preserve the existing RPO and robot-arm implementations in an optional SpaceAGORA extension.
- Retain the historical companion entry point through a separate compatibility shim.
- Add standalone examples, API documentation and tests.
- Refuse started or failed standalone search states, invalid positional settings,
  out-of-bounds initialization, mismatched or aliased buffers, and shared RNG objects
  before execution. Bounds must be vectors; tuple bounds are no longer accepted.
- Document callback mutation limits and caller-owned RNG stream independence.
- Pin accepted arithmetic and RNG consumption in HYPR-owned regression tests.
- Measure core and extension coverage together at 90% overall and 80% per file.
