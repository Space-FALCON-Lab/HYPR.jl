# SpaceAGORA integration

The candidate uses a Julia package extension. `using HYPR` alone does not install
or load SpaceAGORA. When both packages are loaded, `HYPRSpaceAGORAExt` installs the
existing domain methods and activates SpaceAGORA's HYPR availability bridge.
Both load orders are supported. Loading HYPR in one process does not load it on
separate worker processes.

The integration is reviewed against SpaceAGORA commit
`9be384a20dfe28594a35341eed18e6fe945f1049`. Its current 0.1 compatibility range is
not evidence of compatibility with every historical 0.1 revision. The extension
imports existing internal contracts, so a released pairing policy and an explicit
mismatch check remain publication requirements.

The accompanying SpaceAGORA change makes `SpaceAGORAHYPR` a thin compatibility
package depending on HYPR and SpaceAGORA. Existing `using SpaceAGORAHYPR` users
retain their entry point and public configuration/result types. Do not load the
old implementation-bearing companion together with this candidate: it defines
the same methods. Use the matched compatibility shim or load HYPR directly with
the pinned SpaceAGORA core.

RPO geometry, dynamics, references and robot-arm models remain supplied by
SpaceAGORA. Its core keeps shared path, metric, RRT and retiming services for other
algorithms. This extraction does not make those domain models independently
available outside the simulator. A future domain-neutral path API is distinct
work requiring a consumer-driven contract and independent review.
