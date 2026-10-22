# Migration

Use this guide when moving an existing SpaceAGORA HYPR integration to the
standalone HYPR package and its optional adapter. If you only need numerical
search, start with the [standalone example](index.md) instead. The migration
keeps the public configuration and result types in SpaceAGORA while moving the
search implementation behind the package boundary.

## Owners

| Responsibility | Owner |
| --- | --- |
| Search state, serial search, particle update, swarm policy | HYPR core |
| RPO and robot-arm HYPR policy and execution | HYPR optional SpaceAGORA extension |
| Historical public config/result types and caller names | SpaceAGORA, preserved for compatibility |
| Geometry, frames, dynamics, shared RRT/metrics/retiming | Existing SpaceAGORA shared owners |
| Simulation initialization, callbacks and output | SpaceAGORA |
| Historical package load name | Thin SpaceAGORAHYPR compatibility package |

The shim re-exports the old companion module aliases. Function and type bindings
in SpaceAGORA retain their owners. Methods implemented by the companion now live
in the extension's corresponding modules. The shim stays through SpaceAGORA 0.6.0
and goes after it; the lasting path to the extension's helpers is
`Base.get_extension(HYPR, :HYPRSpaceAGORAExt).RPO` and `.RobotArm`. No general
serialized Julia object compatibility beyond the retained comparison is claimed.

## Supported migration

First select the compatible SpaceAGORA and HYPR source pair described in the
[integration guide](integration.md). From that SpaceAGORA checkout, create the
separate environment and start Julia in it:

```sh
julia scripts/optional_packages.jl setup hypr /path/to/hypr-env
julia --project=/path/to/hypr-env
```

Setup verifies the optional source at `packages/robotics/HYPR.jl`. The older
`scripts/setup_hypr.jl` route remains supported. In the Julia session, load the
core and the compatibility shim:

```julia
using SpaceAGORA, SpaceAGORAHYPR
```

Existing configured entry points remain accessible through their original
SpaceAGORA names. Run the [robot-arm walkthrough](robot_arm.md) or the applicable
spacecraft example before adapting a new configuration. Preserve the resolved
manifest and source revisions with the experiment, and check the
[validation limits](validation.md) when interpreting its results.

For older installations, HYPR 0.1 does not support SpaceAGORA or shim 0.1.
Applications using only the numerical search core load `HYPR` directly and do
not need this migration or a SpaceAGORA installation.

A repository publication alone does not establish integrated simulator acceptance.
Consult the release record and integration PR for the tested revision pair and
hosted results.

## Compatibility validation

The standalone keyword constructor validates its own settings. The historical
robot-arm adapter keeps its existing validation policy, including accepted edge
settings, through a private core entry point. The matched reference matrix covers
these differences. This compatibility path is internal, not a second public API.

HYPR owns arithmetic regression tests and a complete `src/` plus `ext/` coverage
gate, at 90% overall and 80% per executable file. Its core, documentation and
integration jobs exercise the versioned service provider. SpaceAGORA retains its
own full regression and coverage requirements. Releases require passing hosted
checks, published-source installation and integration verification; results from
a local checkout are recorded separately.
