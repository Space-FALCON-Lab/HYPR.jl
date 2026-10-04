# Adapted from the accepted extraction review: include the now-private particle kernel.
# Positive controls: single-property mutations of the extracted HYPR kernels.
# Each mutant changes exactly one property named in the review question; the parity
# matrix must then differ from the parent for at least one record.
using HYPR
const MUTANTS = Dict(
    # evaluation order inside an iteration
    "eval_order" => ("@inbounds for pidx in 1:n_particles\n            comps = policy.evaluate(",
                     "@inbounds for pidx in reverse(1:n_particles)\n            comps = policy.evaluate("),
    # culling moved ahead of the stopping decision
    "cull_before_stop" => ("        push!(state.cost_history,state.gbest_cost)\n",
                           "        push!(state.cost_history,state.gbest_cost)\n        state.cull_replacements += policy.cull!(state,iter,rng)\n"),
    # the final, unevaluated movement removed (changes only positions and RNG consumption)
    "no_final_move" => ("        weights = policy.weights(iter)\n",
                        "        iter == settings.iterations && break\n        weights = policy.weights(iter)\n"),
    # floating-point association of the velocity update
    "fp_order" => ("v = weights.w_inertia * state.velocities[d,pidx] +\n                    weights.c1 * rand(rng) * (state.pbest[d,pidx] - state.positions[d,pidx]) +\n                    weights.c2 * rand(rng) * (state.gbest[d] - state.positions[d,pidx])",
                   "v = weights.w_inertia * state.velocities[d,pidx] +\n                    (weights.c1 * rand(rng) * (state.pbest[d,pidx] - state.positions[d,pidx]) +\n                    weights.c2 * rand(rng) * (state.gbest[d] - state.positions[d,pidx]))"),
    # RPO kernel: which draw multiplies which term
    "rpo_draw_swap" => ("weights.c1 * rand(local_rng) * (pbest[d,pidx] - positions[d,pidx]) +\n                weights.c2 * rand(local_rng) * (gbest[d] - positions[d,pidx])",
                        "weights.c2 * rand(local_rng) * (gbest[d] - positions[d,pidx]) +\n                weights.c1 * rand(local_rng) * (pbest[d,pidx] - positions[d,pidx])"),
    # RPO kernel: floating-point association
    "rpo_fp_order" => ("velocities[d,pidx] = weights.w_inertia * velocities[d,pidx] +\n                weights.c1 * rand(local_rng) * (pbest[d,pidx] - positions[d,pidx]) +\n                weights.c2 * rand(local_rng) * (gbest[d] - positions[d,pidx])",
                       "velocities[d,pidx] = weights.w_inertia * velocities[d,pidx] +\n                (weights.c1 * rand(local_rng) * (pbest[d,pidx] - positions[d,pidx]) +\n                weights.c2 * rand(local_rng) * (gbest[d] - positions[d,pidx]))"),
)
function function_defs(src)
    defs = Dict{Symbol, Expr}()
    for ex in Meta.parseall(src).args
        ex isa Expr || continue
        ex.head == :macrocall && (ex = ex.args[end]; ex isa Expr || continue)
        ex.head == :function || continue
        sig = ex.args[1]
        while sig isa Expr && sig.head in (:where, :(::)); sig = sig.args[1]; end
        name = sig.args[1]
        name in (:_search!, :_advance_particles!, :advance_particles!, :search!) && (defs[name] = ex)
    end
    return defs
end
const ORIGINAL_SRC = read(joinpath(dirname(pathof(HYPR)), "search.jl"), String)
function apply_mutant!(name)
    old, new = MUTANTS[name]
    count_found = length(findall(old, ORIGINAL_SRC))
    count_found == 1 || error("mutant $name: pattern found $count_found times")
    mutated = replace(ORIGINAL_SRC, old => new; count=1)
    for (_, ex) in function_defs(mutated)
        Core.eval(HYPR, ex)
    end
    println("MUTANT_APPLIED=", name)
end
function restore_original!()
    for (_, ex) in function_defs(ORIGINAL_SRC)
        Core.eval(HYPR, ex)
    end
end
