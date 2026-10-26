# Search API

A `SwarmState` holds the mutable buffers of one run. Do not share it between
concurrent runs, and do not modify particle views inside the objective. Give each
run its own explicit RNG and a newly initialized state.

A `SearchPolicy` supplies evaluation, culling, the coefficient schedule, the
feasibility test and the test for a material improvement. Culling can reset
personal bests when it replaces a particle. A different policy can amount to a
different algorithm and needs its own validation: the RPO and robot-arm
validations do not carry over to your use of the search API.

The serial schedule and the RPO schedule are deliberately different. Serial
search evaluates before moving, draws from one sequential RNG and caps velocities.
The RPO search, in the optional SpaceAGORA extension, keeps one random stream per
particle and its own order of learning, re-exploration, timeout and stopping.

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
best is found, `gbest_components` stays `nothing`; report that as a failure, and do
not treat the zero-initialized best vector as a solution. Exceptions from the
objective or the policy propagate. A NaN cost never becomes a best. There is no
general checkpoint and restart: start a new search with a new state.

## Checked standalone inputs

`search!` validates the settings, even when they were constructed positionally.
All buffers must have matching, nonempty dimensions and must not alias each other;
initial positions must be finite and within the bounds, and velocities finite. A
state is marked as started before the first objective call, so a state left
half-evaluated by an exception cannot be reused: build a new state after any
search has started. Callbacks must not resize buffers or change who owns them
while the search runs. The early-stopping `feasible` callback must accept
`nothing`, which it receives while no finite best exists.

Before changing anything or drawing a random number, `advance_particles!` checks
the dimensions, that buffers and weights are finite, that the bounds are ordered,
and that each particle has its own RNG object. It checks object identity only:
separate RNGs with equal seeds pass, so choosing independent streams is up to you.
Positions may start outside the bounds; this low-level step clamps the next
positions into them. `search!`, the higher-level call, requires initial positions
inside the bounds.

The older adapters use private kernels that keep their original validation
rules. Those kernels are not part of the public standalone API.
