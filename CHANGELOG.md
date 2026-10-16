# Changelog

## Unreleased

- Breaking: the SpaceAGORA extension's planners require an explicit `rng` (an
  `AbstractRNG`) and no longer draw from the global stream when it is omitted:
  `rpo_pso_plan_path`, the HYPR-configured `rpo_rrt_connect_plan_path`,
  `rpo_rrt_star_plan_path` and `rpo_rrt_connect_bezier_plan_path`, and
  `plan_robot_arm_motion_hypr`. This pairs with SpaceAGORA's required random
  streams in its GNC planning kernels; the release version is set with the
  provider contract update.

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
