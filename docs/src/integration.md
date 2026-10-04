# SpaceAGORA integration

`using HYPR` alone neither installs nor loads SpaceAGORA. When both packages are
loaded, `HYPRSpaceAGORAExt` provides the existing configured RPO and robot-arm
methods through `SpaceAGORA.HYPRServices` contract 1.0.0. It imports explicit
contract bindings, including historically underscored helper names, instead of
whole internal simulator modules. Public configuration/result identities remain
owned by SpaceAGORA.

The local supported pair is HYPR 0.1.0 and SpaceAGORA with contract 1.0.0. Legacy
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

The candidates are local and have not been released. Repository source pins,
published installation recipes, moved-code coverage and precompiled-stack checks
remain release-integration requirements. A broad package version range alone is
not evidence of compatibility with historical SpaceAGORA revisions.

Shared geometry, metrics, RRT and retiming remain in SpaceAGORA. The standalone
HYPR search core is independent; configured mission planning still consumes the
simulator's geometry, dynamics and reference services.
