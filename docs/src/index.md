# HYPR.jl

HYPR plans paths for spacecraft rendezvous and robot arms through its optional
SpaceAGORA integration. It uses particle-swarm search to improve candidate paths;
SpaceAGORA provides geometry, dynamics and the simulation that follows a planned
reference. The numerical search core can also minimize a caller's objective
without loading a simulator.

## Choose a first example

Use the standalone example to learn the search API. Use the robot-arm walkthrough
to connect a physical model and target to a timed reference. SpaceAGORA's
[spacecraft guide](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/41cd1f2ef7bcd4ef51f556487911e797ca411867/docs/src/user/hypr_planning.md)
continues from route planning to controller tracking.

| Goal | Start here |
| --- | --- |
| Search a numerical objective | Installation and first search below |
| Plan a robot-arm reference | [Robot-arm walkthrough](robot_arm.md) |
| Install the SpaceAGORA adapter | [SpaceAGORA integration](integration.md) |
| Check a version pair | [Compatibility and source records](integration.md#Compatibility-and-source-records) |
| Define an objective or policy | [Search API](@ref) |
| Understand the evidence and its limits | [Validation and limits](@ref) |
| Read or cite the spacecraft method | [Method and paper](method.md) |
| Migrate an existing project | [Migration](migration.md) |

## Installation and first search

HYPR 0.4.0 is a prerelease for Julia 1.12. It is not registered in General.
To use the standalone library in your own project:

```sh
julia --project=hypr-demo -e 'using Pkg; Pkg.add(url="https://github.com/Space-FALCON-Lab/HYPR.jl.git", rev="v0.4.0")'
```

Launch Julia with `--project=hypr-demo` and load `HYPR`. To run the repository's
executable examples, use a checkout instead:

```sh
git clone https://github.com/Space-FALCON-Lab/HYPR.jl.git
cd HYPR.jl
julia --project=. -e 'using Pkg; Pkg.instantiate()'
julia --project=. examples/bounded_search.jl
```

This example minimizes a dimensionless quadratic with an explicit RNG, objective,
bounds and policy. It prints `best_position` and `cost`. The returned state's
`gbest` is the best evaluated position and `gbest_cost` is its objective. Its final
particle positions can have moved after the final evaluation and are not the
reported solution.

Geometry, physical units, constraints and the meaning of success belong to the
application. Read [Search API](@ref) before defining a policy. The
[method guide](method.md) relates this numerical interface to the spacecraft
method, with the confirmed citation and the distinction between manuscript
results and current software validation.
