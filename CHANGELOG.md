# Changelog

## 0.4.0 (RELEASE_DATE)

- SpaceAGORA compatibility moves to `0.4, 0.5`. SpaceAGORA 0.5.0 removes the
  names deprecated in 0.3.0; HYPR uses none of them, so no code changes.
- SpaceAGORA accepts HYPR 0.4.0, and only it, from the merge of its companion-pin
  pull request for 0.5.0 (`f7f3c0bd4`), which this release's integration CI pins.
  SpaceAGORA 0.5.0 is the first release that pairs with it.

## 0.3.0 (2026-10-19)

- SpaceAGORA compatibility moves to `0.3, 0.4`. SpaceAGORA 0.3.0 was the first
  release with `HYPRServices` contract 2.0.0, and 0.4.0 keeps that contract. No
  code changes; HYPR uses none of the SpaceAGORA names deprecated for removal in
  0.5.0.
- SpaceAGORA accepts HYPR 0.3.0, and only it, from the merge of its companion-pin
  pull request (`1618216e2`), which this release's integration CI pins. SpaceAGORA
  0.4.0 is the first release that pairs with it.

## 0.2.0 (2026-10-17)

- Breaking: the SpaceAGORA extension's planners require an explicit `rng` (an
  `AbstractRNG`) and no longer draw from the global stream when it is omitted:
  `rpo_pso_plan_path`, the HYPR-configured `rpo_rrt_connect_plan_path`,
  `rpo_rrt_star_plan_path` and `rpo_rrt_connect_bezier_plan_path`, and
  `plan_robot_arm_motion_hypr`. This pairs with SpaceAGORA's required random
  streams in its GNC planning kernels.
- Breaking: the extension requires SpaceAGORA `HYPRServices` contract 2.0.0, which
  accepts HYPR 0.2.0 only. A SpaceAGORA revision providing contract 1.0.0 is
  refused before the extension defines its methods.
- SpaceAGORA compatibility widens to `0.2, 0.3`: contract 2.0.0 is planned for
  SpaceAGORA 0.3.0, and the paired source revision still declares 0.2.0.

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
