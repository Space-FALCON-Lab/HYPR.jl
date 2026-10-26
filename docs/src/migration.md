# Migration

This page is for code that used HYPR from inside SpaceAGORA, before the search
moved into this package and its optional SpaceAGORA extension. If you only need
numerical search, start with the [standalone example](index.md) instead. The
public configuration and result types stay in SpaceAGORA; the search
implementation now sits behind the package boundary.

## Owners

| Responsibility | Owner |
| --- | --- |
| Search state, serial search, particle update, swarm policy | HYPR core |
| RPO and robot-arm HYPR policy and execution | HYPR's optional SpaceAGORA extension |
| Public configuration and result types, and the names callers use | SpaceAGORA |
| Geometry, frames, dynamics, shared RRT, metrics and retiming | SpaceAGORA |
| Simulation initialization, callbacks and output | SpaceAGORA |

Function and type bindings keep their owners in SpaceAGORA. The methods HYPR
implements live in the extension's `RPO` and `RobotArm` modules; reach them with
`Base.get_extension(HYPR, :HYPRSpaceAGORAExt).RPO` and `.RobotArm`. Serialized
Julia objects are not guaranteed to load across this change, beyond the comparison
cases the tests keep.

## Moving to SpaceAGORA 0.7

First pick a SpaceAGORA release and the HYPR revision it pins, from the table in
the [integration guide](integration.md). From that SpaceAGORA checkout, create a
separate environment and start Julia in it:

```sh
julia --startup-file=no scripts/optional_packages.jl setup hypr /path/to/hypr-env
JULIA_LOAD_PATH='@:@stdlib' julia --startup-file=no --project=/path/to/hypr-env
```

Setup checks the HYPR source at `packages/robotics/HYPR.jl` before installing it.
In Julia, load both packages:

```julia
using SpaceAGORA, HYPR
```

The configured entry points keep their SpaceAGORA names. Run the
[robot-arm walkthrough](robot_arm.md) or the matching spacecraft example before
adapting your own configuration, keep the resolved manifest and source revisions
with your results, and read the [validation limits](validation.md) before
interpreting them.

SpaceAGORA 0.6 shipped a compatibility loader, `SpaceAGORAHYPR` 0.2.0, so that
`using SpaceAGORA, SpaceAGORAHYPR` kept working and `SpaceAGORAHYPR.RPO` and
`SpaceAGORAHYPR.RobotArm` named the extension's modules. SpaceAGORA 0.7 removed
it. Replace `using SpaceAGORAHYPR` with `using HYPR`, and replace
`SpaceAGORAHYPR.RPO` and `SpaceAGORAHYPR.RobotArm` with the
`Base.get_extension` path above. If a package in your environment still depends
on `SpaceAGORAHYPR`, SpaceAGORA's setup stops and asks you to migrate that package
first.

HYPR 0.1.x works only with service contract 1.0.0, not with the current
SpaceAGORA adapter or the old loader. Code that uses only the numerical search
core loads `HYPR` directly and needs neither this migration nor SpaceAGORA.

## Preserving an existing environment

The current HYPR installer can copy an existing SpaceAGORA/HYPR environment into
a fresh destination. From the HYPR checkout, run:

```sh
julia --startup-file=no --project=@stdlib scripts/setup_env.jl --migrate-from /path/to/old-env /path/to/new-env
```

The old environment stays unchanged. Migration preserves its project, manifest
and local preferences, resolves relative source paths against their old location,
removes the retired `SpaceAGORAHYPR` loader, and selects this HYPR checkout and the
configured core together. It refuses environments containing another package
that still requires the retired loader; update that package first. The new
destination must be empty and outside both source checkouts and the old
environment. Failed package resolution is reported, with no fallback to another
source revision. Once migration has created the destination, ordinary setup can
retry an interrupted installation.

This preserves environment metadata, not simulation results or arbitrary files
stored beside it. Keep those in their original location or copy them separately.
For a core-only project, use SpaceAGORA's `scripts/setup_core.jl` instead.

## Compatibility validation

The keyword constructor of the standalone settings validates them. The older
robot-arm adapter keeps its own validation rules, including the edge settings it
always accepted, through a private entry point in the core; the reference
comparison covers those differences. That path is internal and is not a second
public API.

HYPR's arithmetic regression tests and its coverage gate (90% overall and 80% per
file across `src/` and `ext/`) are described in [Validation and limits](validation.md).
SpaceAGORA keeps its own regression and coverage requirements.
