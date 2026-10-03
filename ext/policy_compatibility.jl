function hypr_iteration_weights(schedule_enable::Bool, n_iters::Int, iter::Int,
    w_inertia::Real, c1::Real, c2::Real, transition_fraction::Real,
    w_min::Real, w_end_fraction::Real, c1_end_fraction::Real, c2_end_fraction::Real,
    c_min::Real, c_max::Real)
    return HYPR.hypr_iteration_weights(schedule_enable, n_iters, iter, w_inertia,
        c1, c2, transition_fraction, w_min, w_end_fraction, c1_end_fraction,
        c2_end_fraction, c_min, c_max)
end
function hypr_material_improvement(new_cost::Real, reference_cost::Real,
    min_abs_improvement::Real, min_rel_improvement::Real)::Bool
    return HYPR.hypr_material_improvement(new_cost, reference_cost,
        min_abs_improvement, min_rel_improvement)
end
hypr_protected_particle_mask(costs, elite_fraction) =
    HYPR.hypr_protected_particle_mask(costs, elite_fraction)
