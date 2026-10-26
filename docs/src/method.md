# HYPR and spacecraft motion planning

HYPR has two parts with different purposes: a numerical search library, and an
optional implementation of spacecraft motion planning in SpaceAGORA. The library
searches an objective that you supply. The spacecraft implementation gives that
search a physical meaning, builds a timed reference and hands it to a tracking
controller.

## The standalone search library

The core works on arrays of particle positions and velocities. A `SearchPolicy`
defines how candidates are evaluated, how weak particles are replaced, how the
swarm coefficients change and when an improvement counts. A `SwarmState` holds the
mutable state of one search. You supply the bounds, the initial swarm and the
random-number generator, and you decide what the objective means and its units.

This interface works without SpaceAGORA, and it defines no vehicle, collision
geometry, gravity field or controller. `examples/bounded_search.jl` minimizes a
dimensionless quadratic to show how the pieces fit together; its result is the
best candidate evaluated, `gbest`, not the particles' final positions. See
[Search API](api.md) for the rules on ownership, failures and reproducibility.

## The spacecraft method

The HyPR method of Gorisek-Gazze and Falcone searches for a route near a target
spacecraft or station. An RRT-Connect path gives a feasible initial guess, and
particle-swarm optimization refines candidate Bézier paths, scoring each on
obstacle clearance and on a fuel-related objective. Retiming applies clearance,
reaction time and acceleration-based speed limits, both to estimate the effort of
following a path and to build its final reference. An LQ-MPC controller then
tracks the reference.

The planner's fuel proxy and the propellant used during tracking are different
quantities. A lower proxy cost does not imply a particular fuel saving for a
vehicle with other dynamics or actuators, and clearance sampled along a reference
does not imply collision-free tracking under arbitrary disturbances. In the paper,
a replanning layer reacts to changes in clearance, tracking error and goal by
keeping the reference, retiming it, holding or searching again.

In this repository, the optional SpaceAGORA extension holds the configured RPO
and robot-arm versions. Its RPO schedule keeps one random stream per particle and
its own search order, so it differs from the standalone serial `search!` loop.
SpaceAGORA supplies the shared geometry, frames, dynamics, reference services and
the simulation lifecycle. The extension reaches SpaceAGORA only through the
versioned `SpaceAGORA.HYPRServices` contract. The [integration guide](integration.md)
covers installation and the optional objective callback; for a complete
spacecraft configuration, use SpaceAGORA's
[motion-planning guide](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/v0.7.1/docs/src/user/hypr_planning.md).

## Reading the paper alongside the software

The accepted manuscript (September 23 version) reports a 500-case static
comparison and 900 dynamic cases. Those are the paper's results, not acceptance
results for every use of this package. The extraction, arithmetic and objective
compatibility tests establish the narrower properties described in
[Validation and limits](validation.md).

SpaceAGORA's current research comparison helper does not follow the manuscript's
protocol. It tracks with a discrete linear Hill–Clohessy–Wiltshire model and
counts continuous commanded impulse, whereas the paper describes nonlinear
two-body propagation with a 20% PWM activation threshold. Its defaults also
differ in controller settings, planner mode and random-seed mapping. Setting
`hypr_mode=:manuscript` in a SpaceAGORA `RPOPSOConfig` selects the paper's
objective formulation, not the paper's complete experiment.

An exact comparison would need the original endpoint and geometry manifests, the
dynamic event definitions, the source revision and the complete outcome records of
the experiment. Those have not been recovered. A single successful generated case
cannot reproduce an aggregate fuel statistic. Static fuel comparisons must use the
paper's definitions of successful and matched cases; the dynamic study uses its
own static-map baseline. Keep the source revision and the resolved settings with
any new results, so they are not confused with the manuscript's measurements.

## Citation

Jakob Gorisek-Gazze and Giusy Falcone, “HyPR: Safety- and Fuel-Aware Motion Planning
and Replanning for Autonomous SmallSat Servicing,” accepted to the IEEE
International Conference on Space Robotics (iSpaRo), 2026. The
[Space-FALCON Lab publications page](https://www.spacefalconlab.com/publications/)
lists the paper as accepted; there is no DOI or page range yet.

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

Cite the paper for the method, and record the exact HYPR and SpaceAGORA
revisions for any software experiment; the citation does not replace that record. [PROVENANCE.md](https://github.com/Space-FALCON-Lab/HYPR.jl/blob/61fcee33ba4873a5f214490716e0a048e870ebf7/PROVENANCE.md)
describes the extraction history and retained license.
