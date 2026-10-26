# HYPR.jl

HYPR plans paths for spacecraft rendezvous and robot arms through its optional
SpaceAGORA integration. It improves candidate paths with particle-swarm search;
SpaceAGORA provides the geometry, the dynamics and the simulation that follows a
planned reference. The numerical search core also works on its own, minimizing an
objective you supply, without loading a simulator.

## Choose a first example

The standalone example below shows the search API. The robot-arm walkthrough
connects a physical model and a target to a timed reference. For spacecraft,
SpaceAGORA's
[HYPR planning guide](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/v0.7.1/docs/src/user/hypr_planning.md)
goes from route planning to controller tracking.

| Goal | Start here |
| --- | --- |
| Search a numerical objective | Installation and first search below |
| Plan a robot-arm reference | [Robot-arm walkthrough](robot_arm.md) |
| Install the SpaceAGORA adapter | [SpaceAGORA integration](integration.md) |
| Check a version pair | [Compatibility and source records](integration.md#Compatibility-and-source-records) |
| Define an objective or policy | [Search API](api.md) |
| Understand the evidence and its limits | [Validation and limits](validation.md) |
| Read or cite the spacecraft method | [Method and paper](method.md) |
| Migrate an existing project | [Migration](migration.md) |

## Installation and first search

HYPR 0.5.0 is a prerelease for Julia 1.12 and is not registered in General.
To add the standalone library to your own project:

```sh
julia --project=hypr-demo -e 'using Pkg; Pkg.add(url="https://github.com/Space-FALCON-Lab/HYPR.jl.git", rev="v0.5.0")'
```

Start Julia with `--project=hypr-demo` and run `using HYPR`. The examples are in
the repository, so to run them use a clone:

```sh
git clone https://github.com/Space-FALCON-Lab/HYPR.jl.git
cd HYPR.jl
julia --project=. -e 'using Pkg; Pkg.instantiate()'
julia --project=. examples/bounded_search.jl
```

The example minimizes a dimensionless quadratic with an explicit RNG, objective,
bounds and policy, and prints `best_position` and `cost`. In the returned state,
`gbest` is the best position evaluated and `gbest_cost` its objective. The final
particle positions may have moved after the last evaluation, so they are not the
solution.

Geometry, physical units, constraints and what counts as success are up to the
application. Read [Search API](api.md) before writing a policy. The
[method guide](method.md) relates this numerical interface to the spacecraft
method, gives the citation, and separates the paper's results from what the
software's tests check.
