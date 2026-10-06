# HYPR.jl

HYPR owns the search algorithm. The core works with numerical particle arrays
and explicit policy functions. No simulator is installed or loaded by the core.
An optional extension provides the existing RPO and robot-arm domain adapters.

## Installation

HYPR 0.1.1 is an optional-objective prerelease for Julia 1.12. Install the versioned source
into a separate project:

```sh
julia --project=hypr-demo -e 'using Pkg; Pkg.add(url="https://github.com/Space-FALCON-Lab/HYPR.jl.git", rev="v0.1.1")'
```

Run Julia with `--project=hypr-demo` and load `HYPR`. The core does not need a
SpaceAGORA checkout. The package is not registered in General. For the executable
example and test suite, use the repository checkout instructions in the README.

## First example

Run `julia --project=. examples/bounded_search.jl`. The example supplies its own
RNG, objective, bounds and policy. The returned state's `gbest` is the best
evaluated position and `gbest_cost` is its objective. Its final particle positions
can have moved after the final evaluation and are not the reported solution.

Geometry, physical units, constraints, initialization and the meaning of success
belong to the application. See [Search API](@ref) before implementing a policy.
