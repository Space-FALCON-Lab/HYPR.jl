# Changelog

## 0.5.0 (unreleased)

- Breaking: the SpaceAGORA extension requires `HYPRServices` contract 3.0.0 and
  refuses 2.0.0 before it defines its methods. Contract 3.0.0 lists only the names
  SpaceAGORA's own code calls or exports and the names HYPR uses.
- The extension's helpers are HYPR's own functions. 75 RPO and robot-arm helpers
  that SpaceAGORA declared until contract 2.0.0 are now defined in the extension's
  `RPO` and `RobotArm` modules: path cost, path retiming and the
  acceleration-limited reference (`rpo_path_cost`, `rpo_retime_path`,
  `rpo_retimed_reference`), the PSO, refinement and warm-start helpers, and the
  robot-arm swarm, refinement, retiming and RRT helpers. Their code and results
  are unchanged. Reach them through `SpaceAGORAHYPR.RPO` and
  `SpaceAGORAHYPR.RobotArm`; `SpaceAGORA.HYPRServices` and SpaceAGORA's internal
  modules no longer have them.
- The extension's `SwarmPolicy` module names HYPR's own `hypr_iteration_weights`,
  `hypr_material_improvement` and `hypr_protected_particle_mask`. The methods
  that forwarded SpaceAGORA's copies of these functions to HYPR are removed.
- The extension no longer imports `_robot_arm_segment_distance`,
  `plan_robot_arm_motion` and `rpo_sample_path_bezier`, which it did not use and
  which contract 3.0.0 no longer lists.
- SpaceAGORA compatibility is `0.5, 0.6` while the extension is paired with
  SpaceAGORA's 0.6.0 development line, which still declares 0.5; the runtime
  contract check refuses SpaceAGORA 0.5 releases, which provide contract 2.0.0.
  The release narrows it to `0.6`, so that the resolver refuses that pair.
- Tests: the testsets SpaceAGORA ran on these helpers move to
  `test/extension_helpers.jl`, which also checks the `SpaceAGORAHYPR` access
  paths. `test/service_boundary.jl` checks that the names the extension imports or
  reaches are exactly the contract's lists, and that each is used outside its
  import. Tests build the station and chaser with SpaceAGORA's public
  `RPOStationGeometry` and `RPOCubeSatGeometry`.

## 0.4.0 (2026-10-20)

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
