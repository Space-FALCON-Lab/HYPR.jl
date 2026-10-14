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
[SpaceAGORA installation guide](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/388b8801b3e912c264b04a462b019acbaf1c0d1a/docs/src/user/installation_environment.md#optional-package-commands)
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

The following records identify the source pair used by this guide. Save the
resolved project and manifest with a run; the version number alone does not
identify a SpaceAGORA research revision.

| Component | Version or revision | Meaning |
| --- | --- | --- |
| Julia | 1.12 | Package compatibility requirement |
| SpaceAGORA | 0.2.0 at [`388b8801`](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/commit/388b8801b3e912c264b04a462b019acbaf1c0d1a) | Core revision used for the documented arm walkthrough |
| HYPR | 0.1.1 at [`39e4cb96`](https://github.com/Space-FALCON-Lab/HYPR.jl/commit/39e4cb9648fa94b15712d00017f12c3aad451e69), tag `v0.1.1` | Runtime pinned by that SpaceAGORA checkout |
| SpaceAGORAHYPR | 0.2.0, in the same SpaceAGORA checkout | Compatibility loader for the historical package name |
| HYPRServices | 1.0.0 | Adapter contract; explicitly accepts providers HYPR 0.1.0 and 0.1.1 |

HYPR's documentation update at `aa1b5b71` changes no runtime files from
`v0.1.1`. The arm walkthrough uses that runtime with the core revision above.
HYPR's own integration CI separately pins SpaceAGORA
[`8a979873`](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/commit/8a979873ad0ed17045a9080347b13f944c176079).
That configured CI pair and the bounded walkthrough have different scopes;
neither establishes compatibility with every source revision carrying version
0.2.0, or reproduction of the paper. See [Validation and limits](@ref) for the
retained evidence and release requirements.

## Loading and ownership

`using HYPR` alone neither installs nor loads SpaceAGORA. When both packages are
loaded, `HYPRSpaceAGORAExt` provides the existing configured RPO and robot-arm
methods through `SpaceAGORA.HYPRServices` contract 1.0.0. It imports explicit
contract bindings, including historically underscored helper names, instead of
whole internal simulator modules. Public configuration/result identities remain
owned by SpaceAGORA.

HYPR 0.1.1 requires a SpaceAGORA 0.2.0 revision explicitly accepting provider
0.1.1 under service contract 1.0.0. That consumer also accepts HYPR 0.1.0. Legacy
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
in SpaceAGORA's [public service reference](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/388b8801b3e912c264b04a462b019acbaf1c0d1a/docs/src/maintainer/hypr_services.md).
The accompanying [motion-planning guide](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/388b8801b3e912c264b04a462b019acbaf1c0d1a/docs/src/user/hypr_planning.md)
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
