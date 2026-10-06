# SpaceAGORA integration

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
in SpaceAGORA's `docs/src/maintainer/hypr_services.md`. Aliases preserve original
function/type ownership. An incompatible contract change needs a coordinated
version update and review.

Use a SpaceAGORA revision that carries this service contract and source pin,
as identified by its integration release or PR. SpaceAGORA's
`scripts/setup_hypr.jl` selects the full Git revision recorded in its
`packages/SpaceAGORAHYPR/HYPRSource.toml`. The same helper serves unit tests,
independent installation checks and the RPO example environment. An explicit
`SPACEAGORA_HYPR_PATH` selects a local development checkout. Julia does not use a
dependency's `[sources]` section transitively, so developing only the shim is not
a supported recipe.

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
