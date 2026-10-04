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

## Release sequence

1. Independently review this package boundary and the changed search loop.
2. Verify clean installation from a Git commit without sibling-path assumptions.
3. Publish the reviewed HYPR repository and a versioned prerelease.
4. Pin SpaceAGORA's shim, example setup and CI to that package identity.
5. Complete coverage ownership for moved files, preserving 90% overall and 80%
   per-file limits; run both repositories' CI and installation matrix. The
   package-applicable source-boundary rules already run in this candidate.
6. Merge the reviewed integration and verify combined main before closing H6.

The candidate is not ready to replace a production release until these gates pass.

## Compatibility validation

The standalone keyword constructor validates its own settings. The historical
robot-arm adapter keeps its existing validation policy, including accepted edge
settings, through a private core entry point. The matched reference matrix covers
these differences. This compatibility path is internal, not a second public API.

The CI definition is an unexecuted candidate until this repository is published.
Its standalone, docs and integration jobs do not yet replace SpaceAGORA's full
coverage campaign. HYPR now owns arithmetic regression tests and a complete
`src/` plus `ext/` coverage gate, at 90% overall and 80% per executable file.
The local gate uses the reviewed SpaceAGORA contract candidate; publication of
that pairing, precompiled loading checks and fresh hosted CI remain separate
release requirements.
