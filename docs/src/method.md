# HYPR and spacecraft motion planning

HYPR provides a numerical search library and an optional implementation of
spacecraft motion planning in SpaceAGORA. These serve different purposes. The
standalone library searches an objective supplied by its caller; the spacecraft
implementation gives that search a physical interpretation, builds a timed
reference and connects it to a tracking controller.

## The standalone search library

The core works with arrays of particle positions and velocities. A `SearchPolicy`
defines how candidates are evaluated, how weak particles are replaced, how the
swarm coefficients change and when an improvement is meaningful. `SwarmState`
holds one search's mutable state. The caller supplies the bounds, initial swarm
and random-number generator, together with the meaning and units of the objective.

This interface can be used without SpaceAGORA. It does not itself define a
vehicle, collision geometry, gravitational field or controller. The
`examples/bounded_search.jl` example minimizes a dimensionless quadratic to show
how these pieces fit together. Its result is the best evaluated candidate,
`gbest`, rather than the particles' final positions. See [Search API](@ref) for
ownership, failure and reproducibility requirements.

## The spacecraft method

The HyPR method described by Gorisek-Gazze and Falcone searches for a route near
a target spacecraft or station. An RRT-Connect path provides a feasible initial
guess, and particle-swarm optimization refines candidate Bézier paths. Candidate
evaluation considers both obstacle clearance and a fuel-related objective.
Retiming uses clearance, reaction time and acceleration-based speed limits to
predict the effort of following a path and to prepare its final reference.
An LQ-MPC controller then tracks that reference.

The planner's fuel proxy and propellant consumed during tracking answer different
questions. A lower proxy cost does not establish a particular fuel saving for a
vehicle with different dynamics or actuators. Likewise, sampled clearance of a
reference does not establish collision-free tracking under arbitrary disturbances.
The paper's replanning layer responds to changes in clearance, tracking error
and goal by retaining a reference, retiming it, holding or searching again.

In this repository, the optional SpaceAGORA extension owns the configured RPO
and robot-arm specializations. Its RPO schedule preserves per-particle random
streams and its domain-specific search order; it is not identical to the
standalone serial `search!` loop. SpaceAGORA supplies the shared geometry, frames,
dynamics, reference services and simulation lifecycle. Loading the extension
uses the versioned `SpaceAGORA.HYPRServices` contract, not arbitrary simulator
internals. The [integration guide](integration.md) describes installation and
the optional objective callback. For a complete spacecraft configuration, use
SpaceAGORA's [motion-planning guide](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/41cd1f2ef7bcd4ef51f556487911e797ca411867/docs/src/user/hypr_planning.md).

## Reading the paper alongside the software

The confirmed September 23 manuscript reports a 500-case static comparison and
900 dynamic cases. Those are manuscript results, not acceptance results for
every use of this package. The retained extraction, arithmetic and objective
compatibility tests establish the narrower properties described in
[Validation and limits](@ref).

The current SpaceAGORA research comparison helper differs from the manuscript
protocol. It tracks with a discrete linear Hill–Clohessy–Wiltshire model and
accounts for continuous commanded impulse; the paper describes nonlinear
two-body propagation with a 20% PWM activation threshold. The research defaults
also differ in controller settings, planner mode and random-seed mapping.
Selecting `hypr_mode=:manuscript` in a SpaceAGORA `RPOPSOConfig` selects that
objective formulation, not the complete experiment described in the paper.

An exact comparison still needs the original endpoint and geometry manifests,
dynamic event definitions, source revision and complete outcome records bound
to the experiment. That reconstruction has not been established for this release.
In particular, a successful generated case cannot reproduce an aggregate fuel
statistic. Static fuel comparisons must preserve the paper's successful-case
and matched-case definitions; the dynamic study uses its own static-map baseline.
Keep the source revision and resolved settings with any new results so they can
be distinguished from the manuscript's measurements.

## Citation

Jakob Gorisek-Gazze and Giusy Falcone, “HyPR: Safety- and Fuel-Aware Motion Planning
and Replanning for Autonomous SmallSat Servicing,” accepted to the IEEE
International Conference on Space Robotics (iSpaRo), 2026. The
[Space-FALCON Lab publication record](https://www.spacefalconlab.com/publications/)
confirms the citation and acceptance; it does not yet establish a DOI or
proceedings page range.

```bibtex
@inproceedings{gorisekgazze2026hypr,
  author = {Gorisek-Gazze, Jakob and Falcone, Giusy},
  title = {{HyPR}: Safety- and Fuel-Aware Motion Planning and Replanning for Autonomous {SmallSat} Servicing},
  booktitle = {IEEE International Conference on Space Robotics (iSpaRo)},
  year = {2026},
  note = {Accepted},
  url = {https://www.spacefalconlab.com/publications/}
}
```

Cite the paper for the method and record the exact HYPR and SpaceAGORA source
revisions for a software experiment. The paper citation does not replace the
software record. [PROVENANCE.md](https://github.com/Space-FALCON-Lab/HYPR.jl/blob/61fcee33ba4873a5f214490716e0a048e870ebf7/PROVENANCE.md)
describes the extraction history and retained license.
