# SpaceAGORA integration

`using HYPR` alone neither installs nor loads SpaceAGORA. When both packages are
loaded, `HYPRSpaceAGORAExt` provides the existing configured RPO and robot-arm
methods through `SpaceAGORA.HYPRServices` contract 1.0.0. It imports explicit
contract bindings, including historically underscored helper names, instead of
whole internal simulator modules. Public configuration/result identities remain
owned by SpaceAGORA.

The version pair is HYPR 0.1.0 and SpaceAGORA 0.2.0 with contract 1.0.0. Legacy
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
