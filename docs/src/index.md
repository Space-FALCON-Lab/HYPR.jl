# HYPR.jl

HYPR owns the search algorithm. The core works with numerical particle arrays
and explicit policy functions. No simulator is installed or loaded by the core.
An optional extension provides the existing RPO and robot-arm domain adapters.

## Installation status

This is a local extraction candidate. Follow the checkout instructions in the
README. Public URL and version-tag installation will be documented after review
and publication. Do not infer registry availability from the package name.

## First example

Run `julia --project=. examples/bounded_search.jl`. The example supplies its own
RNG, objective, bounds and policy. The returned state's `gbest` is the best
evaluated position and `gbest_cost` is its objective. Its final particle positions
can have moved after the final evaluation and are not the reported solution.

Geometry, physical units, constraints, initialization and the meaning of success
belong to the application. See [Search API](@ref) before implementing a policy.
