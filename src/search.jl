"""
    SwarmState(positions, velocities)

Mutable search buffers for a Float64 swarm, with particles in columns. The two
input matrices are used in place and must have matching nonempty dimensions.
Personal bests start at those positions with infinite costs. A state belongs to
one search; create a new state for an independent run.
"""
mutable struct SwarmState
    positions::Matrix{Float64}
    velocities::Matrix{Float64}
    pbest::Matrix{Float64}
    pbest_cost::Vector{Float64}
    pbest_components::Vector{Any}
    gbest::Vector{Float64}
    gbest_cost::Float64
    gbest_components::Any
    cost_history::Vector{Float64}
    early_stop_best_cost::Float64
    early_stop_stale_iters::Int
    early_stop_iter::Int
    cull_replacements::Int
end
function SwarmState(positions::Matrix{Float64}, velocities::Matrix{Float64})
    size(positions) == size(velocities) || throw(DimensionMismatch("Swarm positions and velocities must match."))
    all(>(0), size(positions)) || throw(ArgumentError("A swarm needs dimensions and particles."))
    positions === velocities && throw(ArgumentError("Positions and velocities must be distinct buffers."))
    d, n = size(positions)
    return SwarmState(positions, velocities, copy(positions), fill(Inf,n),
        Vector{Any}(undef,n), zeros(d), Inf, nothing, Float64[], Inf, 0, 0, 0)
end

"""
    SearchSettings(; iterations, early_stopping=false, min_iterations=1,
                   patience=1, max_velocity_fraction=0.2)

Serial HYPR search schedule. Each iteration evaluates particles in index order,
records the best cost, checks stopping, culls, then updates velocities/positions.
The final update is retained even though no further evaluation follows it, matching
the existing robot-arm variant. RPO uses its distinct evaluation/stopping schedule.
"""
struct SearchSettings
    iterations::Int
    early_stopping::Bool
    min_iterations::Int
    patience::Int
    max_velocity_fraction::Float64
end
function SearchSettings(; iterations::Integer, early_stopping::Bool=false,
        min_iterations::Integer=1, patience::Integer=1, max_velocity_fraction::Real=0.2)
    iterations > 0 || throw(ArgumentError("iterations must be positive."))
    min_iterations >= 0 || throw(ArgumentError("min_iterations must be nonnegative."))
    patience >= 0 || throw(ArgumentError("patience must be nonnegative."))
    isfinite(max_velocity_fraction) && max_velocity_fraction > 0 ||
        throw(ArgumentError("max_velocity_fraction must be finite and positive."))
    return SearchSettings(iterations,early_stopping,min_iterations,patience,max_velocity_fraction)
end

"""
    SearchPolicy(; evaluate, weights, cull!, feasible, materially_better)

Domain services for `search!`. `evaluate(position; cost_cutoff)` returns a bundle
with a numeric `total`; it must not mutate the position view. `weights(iter)`
returns `w_inertia`, `c1` and `c2`. `cull!(state, iter, rng)` may replace particles
and returns a replacement count. `feasible(components)` and
`materially_better(new_cost, reference_cost)` control early stopping. Domain
geometry, constraints, units and objective definitions belong to these services.
"""
struct SearchPolicy{E,W,C,F,M}
    evaluate::E
    weights::W
    cull!::C
    feasible::F
    materially_better::M
end
SearchPolicy(; evaluate, weights, cull!, feasible, materially_better) =
    SearchPolicy(evaluate,weights,cull!,feasible,materially_better)

# Core RPO update: one independent random stream per particle; no velocity cap.
"""
    advance_particles!(positions, velocities, pbest, gbest, lower, upper, weights, rngs)

Apply the RPO variant's bounded PSO update with one RNG per particle. This is a
low-level kernel: dimensions and independent streams are supplied by its caller.
The expression and draw order are preserved from the accepted RPO implementation.
"""
function advance_particles!(positions, velocities, pbest, gbest, lower, upper, weights, rngs)
    @threads for pidx in axes(positions,2)
        local_rng = rngs[pidx]
        for d in axes(positions,1)
            velocities[d,pidx] = weights.w_inertia * velocities[d,pidx] +
                weights.c1 * rand(local_rng) * (pbest[d,pidx] - positions[d,pidx]) +
                weights.c2 * rand(local_rng) * (gbest[d] - positions[d,pidx])
            positions[d,pidx] = clamp(positions[d,pidx] + velocities[d,pidx], lower[d], upper[d])
        end
    end
    return nothing
end

"""
    search!(state, lower, upper, settings, policy; rng)

Execute the serial domain-configured search and return the same state. The caller
owns initialization and an explicit RNG. `gbest` and `gbest_components` are the
best evaluated result; the final positions can be unevaluated. Infinite-cost
objectives can leave the best unavailable (`gbest_components === nothing`), which
the application must treat as failure. Exceptions propagate to the caller.
"""
function search!(state::SwarmState, lower, upper, settings::SearchSettings,
                 policy::SearchPolicy; rng::AbstractRNG)
    dim, n_particles = size(state.positions)
    length(lower)==length(upper)==dim || throw(DimensionMismatch("Bounds must match the swarm dimension."))
    all(isfinite,lower) && all(isfinite,upper) && all(lower .<= upper) ||
        throw(ArgumentError("Bounds must be finite and ordered."))
    isempty(state.cost_history) || throw(ArgumentError("Create a fresh SwarmState for each search."))
    return _search!(state, lower, upper, settings, policy; rng)
end

# The compatibility adapter already owns its historical validation policy.
# Keep newly introduced standalone-input checks out of that existing API.
function _search!(state::SwarmState, lower, upper, settings::SearchSettings,
                  policy::SearchPolicy; rng::AbstractRNG)
    dim, n_particles = size(state.positions)
    span = max.(upper .- lower, 1.0e-9)
    for iter in 1:settings.iterations
        @inbounds for pidx in 1:n_particles
            comps = policy.evaluate(@view(state.positions[:,pidx]); cost_cutoff=state.pbest_cost[pidx])
            curr_cost = comps.total
            if curr_cost < state.pbest_cost[pidx]
                state.pbest[:,pidx] .= state.positions[:,pidx]
                state.pbest_cost[pidx] = curr_cost
                state.pbest_components[pidx] = comps
            end
            if curr_cost < state.gbest_cost
                state.gbest .= state.positions[:,pidx]
                state.gbest_cost = curr_cost
                state.gbest_components = comps
            end
        end
        push!(state.cost_history,state.gbest_cost)
        if settings.early_stopping && iter >= settings.min_iterations &&
                policy.feasible(state.gbest_components)
            if policy.materially_better(state.gbest_cost,state.early_stop_best_cost)
                state.early_stop_best_cost = state.gbest_cost
                state.early_stop_stale_iters = 0
            else
                state.early_stop_stale_iters += 1
            end
            if state.early_stop_stale_iters >= settings.patience
                state.early_stop_iter = iter
                break
            end
        elseif iter == 1
            state.early_stop_best_cost = state.gbest_cost
        end
        state.cull_replacements += policy.cull!(state,iter,rng)
        weights = policy.weights(iter)
        @inbounds for pidx in 1:n_particles
            for d in 1:dim
                v = weights.w_inertia * state.velocities[d,pidx] +
                    weights.c1 * rand(rng) * (state.pbest[d,pidx] - state.positions[d,pidx]) +
                    weights.c2 * rand(rng) * (state.gbest[d] - state.positions[d,pidx])
                vmax = settings.max_velocity_fraction * span[d]
                state.velocities[d,pidx] = clamp(v,-vmax,vmax)
                state.positions[d,pidx] = clamp(state.positions[d,pidx]+state.velocities[d,pidx],lower[d],upper[d])
            end
        end
    end
    return state
end
