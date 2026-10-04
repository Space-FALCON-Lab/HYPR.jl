# Migration

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
in the extension's corresponding modules. No general serialized Julia object
compatibility beyond the retained comparison is claimed.

## Supported migration

1. Use the SpaceAGORA 0.2 service-provider revision identified by the integration
   release or PR. HYPR 0.1 does not support SpaceAGORA or shim 0.1.
2. Run SpaceAGORA's `scripts/setup_hypr.jl` in a separate project. It installs the
   immutable HYPR revision declared by that SpaceAGORA checkout.
3. Load `SpaceAGORA` and the `SpaceAGORAHYPR` compatibility shim in that project.
   Existing configured entry points remain accessible through their original
   SpaceAGORA names. Applications using only the numerical search core load
   `HYPR` directly and do not install SpaceAGORA.
4. Preserve the resolved manifest with experiment records. Verify the supported
   example and the applicable scientific limits before using a new configuration.

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
