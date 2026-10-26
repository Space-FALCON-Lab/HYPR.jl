# SpaceAGORA integration

HYPR supplies the planner; SpaceAGORA supplies the arm or spacecraft model, the
geometry and the simulation. Install both packages in one Julia environment and
load both to activate the adapter. The standalone search core needs only HYPR.

## Install with SpaceAGORA

From a Git clone of SpaceAGORA, use its optional-package command, then run the
short RPO example in the new environment:

```sh
julia --startup-file=no scripts/optional_packages.jl setup hypr /path/to/hypr-env
JULIA_LOAD_PATH='@:@stdlib' julia --startup-file=no --project=/path/to/hypr-env \
  examples/rpo_planner_env/smoke.jl
```

The setup command initializes the HYPR submodule at `packages/robotics/HYPR.jl`,
checks that it is clean and at the revision SpaceAGORA records, and installs it in
the separate environment. The example plans a short RPO maneuver and has the
controller track it. SpaceAGORA's
[installation guide](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/v0.7.1/docs/src/user/installation_environment.md#optional-package-commands)
explains the source checks and what to do when the checkout is missing or
modified. Use a Git clone: a GitHub source archive does not contain submodules.

Keep `/path/to/hypr-env` outside the SpaceAGORA checkout, and use the same
environment for the [robot-arm walkthrough](robot_arm.md). Both examples check
the installation and the interface; their settings are not a validated mission.

SpaceAGORA also keeps the older installer, `julia scripts/setup_hypr.jl
/path/to/hypr-env`. It installs the HYPR source named in
`packages/HYPRSource.toml`. To install your own HYPR checkout with that
installer, set `SPACEAGORA_HYPR_PATH` to it; the checkout must be a HYPR version
SpaceAGORA supports (0.5.0 for SpaceAGORA 0.6 and 0.7). Record the override with
your results.

## Compatibility and source records

Each SpaceAGORA release pins one HYPR revision. SpaceAGORA's setup commands
install that revision, and SpaceAGORA's adapter accepts only the HYPR version
listed for its service contract.

| SpaceAGORA | HYPR | HYPR revision | `HYPRServices` contract |
| --- | --- | --- | --- |
| v0.7.0, v0.7.1 | 0.5.0 | [`2816df11`](https://github.com/Space-FALCON-Lab/HYPR.jl/commit/2816df11c86206cc348ec2c4f82732f958f18ad2) | 3.0.0 |
| v0.6.0, v0.6.1 | 0.5.0 | `aa327b9f` (v0.6.0), tag `v0.5.0` (v0.6.1) | 3.0.0 |
| v0.5.0, v0.5.1 | 0.4.0 | `c434404d` | 2.0.0 |
| v0.4.0 | 0.3.0 | `2e0cb6ba` | 2.0.0 |
| v0.3.0, v0.3.1 | 0.2.0 | `3259e1de` | 2.0.0 |
| `388b8801` (before the first release) | 0.1.1 | `39e4cb96` | 1.0.0 |

The `v0.5.0` tag admits SpaceAGORA 0.6 only. The revision SpaceAGORA 0.7 pins,
`2816df11`, is the same HYPR 0.5.0 with its compatibility widened to 0.6 and 0.7;
there is no release tag for it. The release notes for each HYPR tag record the
pair it was released with. Save the resolved project and manifest with every run:
a version number alone does not identify a SpaceAGORA revision.

Since contract 2.0.0, every planner HYPR supplies takes an explicit `rng`. HYPR
0.1.0 and 0.1.1, whose planners used the global random stream, work only with
contract 1.0.0.

## Loading and ownership

`using HYPR` alone neither installs nor loads SpaceAGORA. When both packages are
loaded, the extension `HYPRSpaceAGORAExt` adds HYPR's RPO and robot-arm methods
to SpaceAGORA through the versioned `SpaceAGORA.HYPRServices` contract. The
extension imports exactly the bindings the contract lists, some of them with
leading underscores for historical reasons, rather than whole simulator modules.
The public configuration and result types stay in SpaceAGORA.

The extension's `RPO` and `RobotArm` modules hold HYPR's own helpers: path cost,
retiming and the acceleration-limited reference, the PSO, refinement and
warm-start helpers, and the robot-arm swarm, refinement, retiming and RRT
helpers. SpaceAGORA declared these until contract 2.0.0. Reach them with
`Base.get_extension(HYPR, :HYPRSpaceAGORAExt).RPO` and `.RobotArm`. The
extension's `SwarmPolicy` module holds HYPR's swarm-policy functions.

SpaceAGORA 0.6 also shipped a compatibility loader, `SpaceAGORAHYPR` 0.2.0, for
code written as `using SpaceAGORAHYPR`; it named the same `RPO` and `RobotArm`
modules. SpaceAGORA 0.7 removed it: load `SpaceAGORA` and `HYPR` directly. See
[Migration](migration.md).

Both load orders work. The contract and provider versions are checked before the
extension defines any method and again during initialization, and HYPR is
available to SpaceAGORA only after that activation succeeds. If a mixed load
fails, Julia does not undo the package initialization, so some methods may
already be defined. Discard that Julia process; SpaceAGORA keeps refusing HYPR
availability and planning in it, so do not catch the error and carry on. Each
worker process needs its own successful load.

SpaceAGORA's [service reference](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/v0.7.1/docs/src/reference/hypr_services.md)
lists the contract and every binding it consumes and implements. The
[motion-planning guide](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/v0.7.1/docs/src/user/hypr_planning.md)
covers configuration, coordinate conventions and how the research examples relate
to the paper. A change to the contract that breaks compatibility needs a version
update on both sides.

Shared geometry, metrics, RRT and retiming stay in SpaceAGORA. The standalone
search core does not depend on SpaceAGORA, but configured mission planning still
uses SpaceAGORA's geometry, dynamics and reference services.

## Optional RPO objective

`rpo_pso_plan_path(...; objective_evaluator=callback)` replaces the objective
that the ordinary HYPR PSO uses. The callback scores the initial particles, the
search updates, the proposals after refinement and the final report. Without it
(or with `nothing`), the planner uses the objective and policy of the selected
mode, legacy or manuscript. Standalone HYPR and robot-arm planning do not use it.

The callback is called as `callback(points, geometry, cfg, safe_distance, cutoff)`,
where the last two arguments are `Float64`. It must return an object with the
fields `total`, `J_obs` and `violation_count`; a missing field raises an
`ArgumentError`. Any other fields are kept in the planner result. You own the
objective and must report the collision and feasibility components correctly:
refinement still refuses a step that increases `J_obs`. The cutoff is an optional
pruning bound in your objective's units; use it only if pruning is valid for your
objective. Callbacks can run concurrently, so they must be thread-safe, must not
mutate their inputs, and should be deterministic if you want reproducible runs.
Exceptions propagate; the planner does not fall back to the standard objective.
Direct calls to `rpo_post_refine_path` accept the same keyword.

The callback comes from the comparison code in SpaceAGORA commit
`fa2e4a411c16c0587230b2020881a36065887b07`. That code runs ordinary HYPR with a
retimed fuel and wheel objective, then evaluates the returned trajectory with
coupled tracking; HYPR's CI runs it unchanged (see
[Validation and limits](validation.md)). It does not add reinforcement-learning
training to HYPR. The callback adds no SpaceAGORA service binding and no export.

## How CI checks the integration

HYPR keeps SpaceAGORA as a weak dependency. The workflow in
`.github/workflows/ci.yml` has these jobs:

| Job | What it runs |
| --- | --- |
| `core (1)`, `core (4)` | The standalone tests on one and four threads, strict precompilation and compiled loading, `examples/bounded_search.jl`, the arithmetic mutation controls and the coverage-gate failure cases. No SpaceAGORA. |
| `integration (core-first)`, `integration (hypr-first)` | HYPR with SpaceAGORA v0.6.0 (`0487e8226`), including the `SpaceAGORAHYPR` loader. Details below. |
| `core-release-pin` | Checks that the SpaceAGORA commit the integration job uses is a SpaceAGORA release tag and that HYPR's compatibility range admits it. |
| `docs` | Builds this documentation. |
| `contributor` | SpaceAGORA's shared contributor workflow with SpaceAGORA v0.7.0 (`bec28fcc3`, the revision also recorded in `SpaceAGORASource.toml`). Its entry point, `test/contributor_ci.jl`, runs `test/integration.jl` in both load orders and `test/extension_helpers.jl`, each in a fresh process. |

In each load order, the integration job:

- builds the SpaceAGORA, HYPR and loader package images and loads them in fresh
  processes with `--compiled-modules=strict --pkgimages=existing`, including
  loader-first loading and the conflict refusals;
- checks that Pkg will not resolve HYPR with a SpaceAGORA version older than
  the service contract (`test/incompatible_resolution.jl`);
- checks that the extension reaches SpaceAGORA only through names the contract
  lists, including through local aliases and inside function bodies, and that
  SpaceAGORA refuses HYPR planning before HYPR is loaded
  (`test/service_boundary.jl`);
- checks that a copy of the extension requiring another contract version is
  refused before it defines a method (`test/incompatible_contract.jl`);
- runs the comparison code from commit `fa2e4a41` (`test/jakob_comparison.jl`),
  the adapter without the loader (`test/integration.jl`) and HYPR's own
  SpaceAGORA helpers (`test/extension_helpers.jl`);
- in the `core-first` order only, measures coverage of `src/` and `ext/` and
  enforces the 90% overall and 80% per-file thresholds.

The integration job and the `contributor` job use different SpaceAGORA
revisions: v0.6.0 is the pair the `v0.5.0` tag was released with, and v0.7.0 is
the current core line. Keep the v0.6.0 job until a replacement pair has passed.
SpaceAGORA's own CI separately tests the HYPR revision it pins against each core
checkout. Neither pair establishes compatibility with every SpaceAGORA revision
in a version range, or reproduction of the paper.

### Updating shared support

`SharedSupport.toml` pins the SpaceAGORA revision that this repository's
generated support files come from; each file's header names its source file and
SHA-256. To change one, edit the source in SpaceAGORA, run
`scripts/sync_companion_support.jl --write` there against this checkout, then
update the pin. The Shared companion support workflow
(`.github/workflows/shared-support.yml`) compares the copies with the pinned
source. This pin is independent of the core revision used for the numerical tests.
