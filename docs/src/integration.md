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
[SpaceAGORA installation guide](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/0487e8226107592b19221618f0f21f500cf2aacf/docs/src/user/installation_environment.md#optional-package-commands)
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

The following records identify the source pair for HYPR 0.5.0 under contract
3.0.0. Save the resolved project and manifest with a run; the version number alone
does not identify a SpaceAGORA research revision.

| Component | Version or revision | Meaning |
| --- | --- | --- |
| Julia | 1.12 | Package compatibility requirement |
| SpaceAGORA | 0.6.0 at [`0487e8226`](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/commit/0487e8226107592b19221618f0f21f500cf2aacf), tag `v0.6.0` | Core release that provides `HYPRServices` 3.0.0 and pins HYPR 0.5.0 |
| HYPR | 0.5.0 | This release |
| SpaceAGORAHYPR | 0.2.0, in the same SpaceAGORA checkout | Compatibility loader for the historical package name |
| HYPRServices | 3.0.0 | Adapter contract; accepts provider HYPR 0.5.0 only |

HYPR's domain and compatibility-shim CI pins the SpaceAGORA v0.6.0 release commit,
[`0487e8226`](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/commit/0487e8226107592b19221618f0f21f500cf2aacf). That configured CI pair and the
bounded walkthrough have different scopes; neither establishes compatibility with
every source revision carrying a 0.6 version, or reproduction of the paper. See
[Validation and limits](@ref) for the retained evidence and release requirements.
The earlier pairs are recorded with their releases: HYPR 0.4.0 at `c434404d` with
SpaceAGORA `41cd1f2ef` (v0.5.0) under contract 2.0.0
([v0.4.0](https://github.com/Space-FALCON-Lab/HYPR.jl/releases/tag/v0.4.0)), HYPR 0.3.0 at `2e0cb6ba` with SpaceAGORA `c4bd7b6ab` (v0.4.0)
under contract 2.0.0 ([v0.3.0](https://github.com/Space-FALCON-Lab/HYPR.jl/releases/tag/v0.3.0)),
HYPR 0.2.0 at `3259e1de` with SpaceAGORA `d5fbe425a` under contract 2.0.0
([v0.2.0](https://github.com/Space-FALCON-Lab/HYPR.jl/releases/tag/v0.2.0)), and HYPR
0.1.1 at `39e4cb96` with SpaceAGORA `388b8801` under contract 1.0.0
([v0.1.1](https://github.com/Space-FALCON-Lab/HYPR.jl/releases/tag/v0.1.1)).

## Loading and ownership

`using HYPR` alone neither installs nor loads SpaceAGORA. When both packages are
loaded, `HYPRSpaceAGORAExt` provides the existing configured RPO and robot-arm
methods through `SpaceAGORA.HYPRServices` contract 3.0.0. It imports explicit
contract bindings, including historically underscored names, instead of whole
internal simulator modules, and imports exactly the names the contract lists.
Public configuration/result identities remain owned by SpaceAGORA.

The helpers behind those methods are HYPR's own functions, defined in the
extension's `RPO` and `RobotArm` modules: path cost, retiming and the
acceleration-limited reference, the PSO, refinement and warm-start helpers, and
the robot-arm swarm, refinement, retiming and RRT helpers. SpaceAGORA declared
them until contract 2.0.0. Reach them through the extension's modules,
`Base.get_extension(HYPR, :HYPRSpaceAGORAExt).RPO` and `.RobotArm`; that path
lasts. While SpaceAGORA ships its `SpaceAGORAHYPR` compatibility loader,
`SpaceAGORAHYPR.RPO` and `SpaceAGORAHYPR.RobotArm` name the same modules; the
loader goes after SpaceAGORA 0.6.0. The extension's `SwarmPolicy` module names
HYPR's own swarm-policy functions.

HYPR 0.5.0 requires a SpaceAGORA revision providing service contract 3.0.0 that
accepts provider 0.5.0; SpaceAGORA 0.6.0 accepts 0.5.0 only. Contract 2.0.0
introduced the rule that every planner HYPR supplies takes an explicit `rng`, and
3.0.0 keeps it; HYPR 0.1.0 and 0.1.1, whose planners defaulted to the global
stream, pair only with contract 1.0.0, HYPR 0.2.0 pairs with SpaceAGORA 0.3,
HYPR 0.3.0 with SpaceAGORA 0.4, and HYPR 0.4.0 with SpaceAGORA 0.5. Legacy
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
in SpaceAGORA's [public service reference](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/0487e8226107592b19221618f0f21f500cf2aacf/docs/src/reference/hypr_services.md).
The accompanying [motion-planning guide](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/0487e8226107592b19221618f0f21f500cf2aacf/docs/src/user/hypr_planning.md)
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


## Shared integration CI and the released compatibility pair

HYPR keeps SpaceAGORA as a weak dependency. Its standalone tests still run
without SpaceAGORA on one and four threads. The shared contributor workflow
uses `SpaceAGORASource.toml` to build a separate integration environment for the
current HYPR checkout; it does not change the package's dependencies.

The shared workflow and its core input are pinned to
`a3d786d80a1bce599c1c6415460d29ed268d906b`, a development core snapshot whose
package version is 0.6.1. This is separate from the released v0.6.0 pair at
`0487e8226107592b19221618f0f21f500cf2aacf` used by the existing domain and
compatibility-shim jobs. The release-pin gate certifies that legacy pair only.
The two revisions deliberately differ; when moving the shared pin, preserve
the legacy checkout until its own replacement pair has been qualified.

`test/contributor_ci.jl` runs both integration load orders and the extension
helper suite in fresh processes. The workflow also runs the service-boundary
checks in both load orders and the incompatible-contract refusal separately.
The existing mutation controls, coverage thresholds, compiled-image checks,
comparison checks and shim-compatibility tests remain required.
