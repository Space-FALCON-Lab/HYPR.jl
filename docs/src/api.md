# Search API

`SwarmState` owns one run's mutable buffers. Do not share it between concurrent
runs or mutate particle views inside the objective. Use a separate explicit RNG
per run. Independent runs need newly initialized states.

`SearchPolicy` supplies evaluation, culling, coefficient schedule, feasibility and
material improvement. Culling can reset personal bests when replacing a particle.
Changing a policy can change the scientific algorithm and requires its own
validation; using the search API does not confer the RPO/robot-arm validations.

The serial schedule and the RPO schedule are deliberately distinct. Serial search
evaluates before movement and uses a shared sequential RNG plus velocity caps.
RPO maintains one random stream per particle and its existing learning,
reexploration, timeout and stopping order in the optional domain implementation.

```@docs
HYPR
SwarmState
SearchSettings
SearchPolicy
search!
advance_particles!
hypr_iteration_weights
hypr_material_improvement
hypr_protected_particle_mask
```

## Failure and restart

An objective may return an infinite cost for an infeasible point. If no finite
best is found, `gbest_components` remains `nothing`; the caller must report failure
and must not treat the zero-initialized best vector as a solution. Objective and
policy exceptions propagate. NaN costs never improve a best. The API does not
claim general checkpoint/restart support; a new search uses a new state.

## Checked standalone inputs

`search!` validates settings even when constructed positionally. All buffers must
have matching nonempty dimensions and must not alias; initial positions must be
finite and within the supplied bounds, and velocities must be finite. A state is
marked started before its first objective call, so an exception cannot make a
partially evaluated state reusable. Build a new state after any started search.
Callbacks must not resize buffers or change ownership while search is executing.
The early-stopping `feasible` callback must accept `nothing` when no finite best
exists.

`advance_particles!` checks dimensions, finite buffers and weights, ordered bounds
and one distinct RNG object per particle before mutation or draws. Separate RNGs
may have equal seeds; object-identity checks cannot prove statistical independence.
The caller owns stream selection. Initial positions may lie outside the update
bounds; this low-level movement clamps their next positions. The higher-level
`search!` requires in-bounds initialization.

The preserved legacy adapters use private kernels with their established
validation policy. Those kernels are not part of the standalone public API.
