# SpaceAGORA integration

HYPR supplies the planner; SpaceAGORA supplies the arm or spacecraft model,
geometry and simulation. Install the packages in one Julia environment, then load
both to activate the adapter. The standalone search core needs only HYPR.

## Install with SpaceAGORA

From a SpaceAGORA Git clone, use the common optional-package command:

```sh
julia --startup-file=no scripts/optional_packages.jl setup hypr /path/to/hypr-env
JULIA_LOAD_PATH='@:@stdlib' julia --startup-file=no --project=/path/to/hypr-env \
  examples/rpo_planner_env/smoke.jl
```

The first command prepares a separate environment and uses the verified optional
checkout at `packages/robotics/HYPR.jl`. The second runs SpaceAGORA's short RPO
example through planning and controller tracking. The
[SpaceAGORA installation guide](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/41cd1f2ef7bcd4ef51f556487911e797ca411867/docs/src/user/installation_environment.md#optional-package-commands)
explains checkout verification and recovery when the optional source is missing
or modified. Use a Git clone for this route; an archive does not contain submodule
contents. The setup command initializes the requested optional checkout.

Keep `/path/to/hypr-env` outside the SpaceAGORA checkout. Use that same environment
for the [robot-arm walkthrough](robot_arm.md). Both examples are small installation
and interface checks; their settings do not define a validated mission.

The earlier `julia scripts/setup_hypr.jl /path/to/hypr-env` helper remains supported.
It installs the immutable source declared by
`packages/SpaceAGORAHYPR/HYPRSource.toml` directly. Developers can set
`SPACEAGORA_HYPR_PATH` when using that helper to select their own HYPR checkout.
Keep this override explicit in experiment records. Developing only the shim is
insufficient because Julia does not apply a dependency's `[sources]` transitively.

## Compatibility and source records

The following records identify the source pair qualified for HYPR 0.4.0 under
contract 2.0.0. Save the resolved project and manifest with a run; the version
number alone does not identify a SpaceAGORA research revision.

| Component | Version or revision | Meaning |
| --- | --- | --- |
| Julia | 1.12 | Package compatibility requirement |
| SpaceAGORA | 0.5.0 at [`41cd1f2ef`](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/commit/41cd1f2ef7bcd4ef51f556487911e797ca411867), tag `v0.5.0` | Core release that provides `HYPRServices` 2.0.0 and pins this HYPR runtime |
| HYPR | 0.4.0 at [`c434404d`](https://github.com/Space-FALCON-Lab/HYPR.jl/commit/c434404dd3e4ac3ba351b2a5e0bb77d9be95fb94), tag `v0.4.0` | Runtime pinned by that SpaceAGORA release; the tag adds only documentation and the CI pin |
| SpaceAGORAHYPR | 0.2.0, in the same SpaceAGORA checkout | Compatibility loader for the historical package name |
| HYPRServices | 2.0.0 | Adapter contract; accepts provider HYPR 0.4.0 only |

HYPR's integration CI pins the SpaceAGORA v0.5.1 release commit,
[`5c0d566f1`](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/commit/5c0d566f1042c446c610ffed3d8c92b64670a7f7), a patch
release after that one with the same `HYPRServices` contract and the same HYPR pin. Both
contain [`f7f3c0bd4`](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/commit/f7f3c0bd46c6faccb58a0d1dcb29213dc48c54bd),
the merge of the pull request that admitted HYPR 0.4.0.
That configured CI pair and the bounded walkthrough
have different scopes; neither establishes compatibility with every source revision
carrying a 0.5 version, or reproduction of the paper. See [Validation and limits](@ref)
for the retained evidence and release requirements. The earlier pairs are recorded
with their releases: HYPR 0.3.0 at `2e0cb6ba` with SpaceAGORA `c4bd7b6ab` (v0.4.0)
under contract 2.0.0 ([v0.3.0](https://github.com/Space-FALCON-Lab/HYPR.jl/releases/tag/v0.3.0)),
HYPR 0.2.0 at `3259e1de` with SpaceAGORA `d5fbe425a` under contract 2.0.0
([v0.2.0](https://github.com/Space-FALCON-Lab/HYPR.jl/releases/tag/v0.2.0)), and HYPR
0.1.1 at `39e4cb96` with SpaceAGORA `388b8801` under contract 1.0.0
([v0.1.1](https://github.com/Space-FALCON-Lab/HYPR.jl/releases/tag/v0.1.1)).

## Loading and ownership

`using HYPR` alone neither installs nor loads SpaceAGORA. When both packages are
loaded, `HYPRSpaceAGORAExt` provides the existing configured RPO and robot-arm
methods through `SpaceAGORA.HYPRServices` contract 2.0.0. It imports explicit
contract bindings, including historically underscored helper names, instead of
whole internal simulator modules. Public configuration/result identities remain
owned by SpaceAGORA.

HYPR 0.4.0 requires a SpaceAGORA revision providing service contract 2.0.0 that
accepts provider 0.4.0; SpaceAGORA 0.5.0 accepts 0.4.0 only. Contract 2.0.0
requires every planner HYPR supplies to take an explicit `rng`; HYPR 0.1.0 and
0.1.1, whose planners defaulted to the global stream, pair only with contract 1.0.0,
HYPR 0.2.0 pairs with SpaceAGORA 0.3, and HYPR 0.3.0 with SpaceAGORA 0.4. Legacy
`using SpaceAGORAHYPR` uses compatibility package 0.2.0. The old 0.1 companion is
excluded by compatibility metadata and refused by the runtime activation check.
Contract and provider versions are checked before source method definitions and
again during initialization. Only successful activation makes HYPR available.
Both load orders are supported.

A failed mixed load may already have defined methods because Julia does not roll
back package initialization. Discard that process. Availability and planning
preflight remain refused after a conflict; do not catch the error and continue
using imported planner methods. Each worker process needs its own supported load.

The contract and its complete consumed/implemented binding inventory are documented
in SpaceAGORA's [public service reference](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/41cd1f2ef7bcd4ef51f556487911e797ca411867/docs/src/reference/hypr_services.md).
The accompanying [motion-planning guide](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/41cd1f2ef7bcd4ef51f556487911e797ca411867/docs/src/user/hypr_planning.md)
introduces configuration, coordinate conventions and the paper's relationship to
the current research examples. The source records above identify the
installation separately from these guides. Aliases preserve original
function/type ownership. An incompatible contract change needs a coordinated
version update and review.

CI builds the core, extension and shim images, then runs fresh processes with
`--compiled-modules=strict --pkgimages=existing`. These witnesses require actual
image use and test both orders, shim-first loading and sticky conflict refusal.
A failed extension is refused before incomplete aliases are used. Release
acceptance requires installation from the published source and hosted checks on
the version pair; local tests alone do not supply that evidence.

The integration job also checks, in both load orders, that the extension reaches
SpaceAGORA only through names the contract lists (also through local aliases and in
function bodies) and that SpaceAGORA refuses HYPR
planning before HYPR loads (`test/service_boundary.jl`), and that a copy of the
extension requiring another contract version is refused before it defines a
method (`test/incompatible_contract.jl`). The `core-release-pin` job checks that
the SpaceAGORA commit the integration job uses is a SpaceAGORA release.

Shared geometry, metrics, RRT and retiming remain in SpaceAGORA. The standalone
HYPR search core is independent; configured mission planning still consumes the
simulator's geometry, dynamics and reference services.

## Optional RPO objective

`rpo_pso_plan_path(...; objective_evaluator=callback)` lets a caller choose the
objective used by ordinary HYPR PSO. The same callback scores initial particles,
search updates, post-refinement proposals and final reporting. Omitting it, or
passing `nothing`, retains the selected legacy or manuscript objective and policy.
Standalone HYPR and robot-arm planning are unchanged.

The callable signature is `callback(points, geometry, cfg, safe_distance, cutoff)`.
The last two arguments are `Float64`. Return a component object with `total`,
`J_obs` and `violation_count`; extra metadata is retained in the planner result.
The caller owns the objective and must report collision/feasibility components
accurately. Refinement still refuses an increase in `J_obs`. The cutoff is an
optional pruning bound in the callback's own objective units; use it only when
pruning is valid for that objective. Callbacks can run concurrently and must be
thread-safe, avoid mutating inputs, and be deterministic for reproducible runs.
Exceptions propagate rather than silently falling back to the standard objective.
Direct `rpo_post_refine_path` calls accept the same optional keyword.

This restores the callback route from Jakob's August 26 SpaceAGORA commit
`fa2e4a411c16c0587230b2020881a36065887b07`. Its comparison consumer uses ordinary
HYPR with a retimed fuel and wheel objective, then separately evaluates the
returned trajectory with coupled tracking. It does not add RL training to HYPR.
The internal dispatch helper is package-owned; no new SpaceAGORA service binding
or standalone public export is introduced.
