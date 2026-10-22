# HYPR's own SpaceAGORA helpers. SpaceAGORA's tests exercised them until its
# HYPRServices contract 3.0.0 moved them into this package's extension; these are
# those testsets, run on the extension's names. Their inputs come from SpaceAGORA's
# public names and the HYPRServices facade, so no private core name is needed.
# Run in the integration environment:
#   julia --project=<environment> test/extension_helpers.jl
using Test, HYPR, SpaceAGORA
@test Base.get_extension(HYPR, :HYPRSpaceAGORAExt).initialized()

# Since SpaceAGORA's HYPRServices 3.0.0 these are HYPR's own functions, defined in
# its extension. Code that reached them through the compatibility package keeps
# working: SpaceAGORAHYPR's namespaces are the extension's modules.
const OWN_RPO = (
    :_rpo_rrt_configured_result, :_rpo_sampling_settings, :rpo_adaptive_pso_config,
    :rpo_estimate_geometry_complexity, :rpo_fit_bezier_fixed_endpoints, :rpo_fuel_proxy_dt_s,
    :rpo_manuscript_adaptive_pso_config, :rpo_manuscript_exploration_score,
    :rpo_manuscript_path_cost_components, :rpo_path_cost, :rpo_position_to_path,
    :rpo_probe_geometry_metrics, :rpo_pso_bounds, :rpo_pso_cull_swarm!,
    :rpo_pso_early_stopping_feasible, :rpo_pso_effective_safe_distance,
    :rpo_pso_empty_warmstart_diagnostics, :rpo_pso_iteration_weights,
    :rpo_pso_material_improvement, :rpo_pso_project_to_segment,
    :rpo_pso_protected_particle_mask, :rpo_pso_rrt_warmstart_path,
    :rpo_pso_stagnation_count_after_learning, :rpo_pso_station_bounds,
    :rpo_pso_tapered_noise_scale, :rpo_pso_warmstart_bounds, :rpo_refine_lower_degree,
    :rpo_refine_shortcut_refit, :rpo_refine_tighten_handles, :rpo_refinement_bernstein,
    :rpo_refinement_better, :rpo_refinement_clamp_path, :rpo_refinement_config,
    :rpo_refinement_project_to_segment, :rpo_refinement_sample_params,
    :rpo_refinement_segment_is_safe, :rpo_refinement_segment_samples,
    :rpo_refinement_shortcut_samples, :rpo_retime_available_distance, :rpo_retime_path,
    :rpo_retime_pointwise_speed, :rpo_retimed_reference, :rpo_try_accept_refinement,
)
const OWN_ROBOT_ARM = (
    :_robot_arm_control_points, :_robot_arm_empty_rrt_warmstart_diagnostics,
    :_robot_arm_flatten_internal_points, :_robot_arm_hypr_base_wrench_ratios,
    :_robot_arm_hypr_cloth_base_wrench_ratios, :_robot_arm_hypr_cloth_state_for_reaction,
    :_robot_arm_hypr_cull_swarm!, :_robot_arm_hypr_early_stopping_feasible,
    :_robot_arm_hypr_iteration_weights, :_robot_arm_hypr_link_com_history,
    :_robot_arm_hypr_material_improvement, :_robot_arm_hypr_post_refine_points,
    :_robot_arm_hypr_reaction_scale, :_robot_arm_hypr_reference_times_from_scales,
    :_robot_arm_hypr_refinement_better, :_robot_arm_hypr_retime_reference,
    :_robot_arm_hypr_rigid_base_wrench_ratios, :_robot_arm_plan_from_q_reference,
    :_robot_arm_resample_polyline_points, :_robot_arm_rrt_connect!,
    :_robot_arm_rrt_connect_warmstart_path, :_robot_arm_rrt_extend!,
    :_robot_arm_rrt_join_paths, :_robot_arm_rrt_nearest_index, :_robot_arm_rrt_path_score,
    :_robot_arm_rrt_random_state, :_robot_arm_rrt_segment_is_safe,
    :_robot_arm_rrt_segment_samples, :_robot_arm_rrt_shortcut_path, :_robot_arm_rrt_steer,
    :_robot_arm_rrt_warmstart_fields, :_robot_arm_seed_control_points,
)
# SpaceAGORA releases before the loader's retirement install the SpaceAGORAHYPR
# compatibility loader beside HYPR; later ones load HYPR directly.
const LOADER = Base.find_package("SpaceAGORAHYPR") !== nothing
LOADER && @eval using SpaceAGORAHYPR
@testset "Earlier access paths reach HYPR's own helpers" begin
    extension = Base.get_extension(HYPR, :HYPRSpaceAGORAExt)
    if LOADER
        @test SpaceAGORAHYPR.RPO === extension.RPO
        @test SpaceAGORAHYPR.RobotArm === extension.RobotArm
        @test SpaceAGORAHYPR.SwarmPolicy === extension.SwarmPolicy
    end
    for name in (:hypr_iteration_weights, :hypr_material_improvement, :hypr_protected_particle_mask)
        @test getglobal(extension.SwarmPolicy, name) === getglobal(HYPR, name)
        @test !isdefined(SpaceAGORA.HYPRServices.RPO, name) && !isdefined(SpaceAGORA.HYPRServices.RobotArm, name)
    end
    @test length(OWN_RPO) + length(OWN_ROBOT_ARM) == 75
    for (namespace, names) in ((extension.RPO, OWN_RPO), (extension.RobotArm, OWN_ROBOT_ARM)), name in names
        f = getglobal(namespace, name)
        @test parentmodule(f) === namespace
        @test !isempty(methods(f))
        @test !isdefined(SpaceAGORA.HYPRServices.RPO, name) && !isdefined(SpaceAGORA.HYPRServices.RobotArm, name)
    end
end

# From SpaceAGORA's test/probes/hypr_search_probes.jl.
module SearchHelpers
using Test, LinearAlgebra, Random, StaticArrays, SpaceAGORA, HYPR
const GH = Base.get_extension(HYPR, :HYPRSpaceAGORAExt).RPO
const G = SpaceAGORA.HYPRServices.RPO

"Station geometry far away from the local RTN frame so paths near the origin are free."
function _hypr_far_geometry(; keepout_radius_m::Real=0.0)
    station = SpaceAGORA.RPOStationGeometry(
        reshape([100.0, 0.0, 0.0], 3, 1);
        keepout_radius_m=keepout_radius_m,
        name="hypr_far",
    )
    return G.RPOReferenceGeometry(station; chaser=SpaceAGORA.RPOCubeSatGeometry(dims_m=(0.02, 0.02, 0.02)))
end

"Single-point station at the origin acting as one sphere obstacle of given keepout radius."
function _hypr_origin_geometry(; keepout_radius_m::Real=0.0)
    station = SpaceAGORA.RPOStationGeometry(
        reshape([0.0, 0.0, 0.0], 3, 1);
        keepout_radius_m=keepout_radius_m,
        name="hypr_origin",
    )
    return G.RPOReferenceGeometry(station; chaser=SpaceAGORA.RPOCubeSatGeometry(dims_m=(0.02, 0.02, 0.02)))
end

"Tiny deterministic-ish PSO config with every optional subsystem off unless requested."
function _hypr_cfg(; kwargs...)
    return SpaceAGORA.RPOPSOConfig(;
        n_waypoints=1,
        n_particles=6,
        n_iters=3,
        sample_ds_m=0.25,
        refinement_sample_ds_m=0.1,
        curve_type=:polyline,
        search_margin_m=1.0,
        spread_scale=0.2,
        adaptive_enable=false,
        adaptive_sampling_enable=false,
        probe_enable=false,
        refinement_enable=false,
        refinement_rounds=2,
        refinement_waypoint_passes=4,
        rrt_warmstart_enable=false,
        cull_enable=false,
        reexplore_enable=false,
        stagnation_learning_enable=false,
        early_stopping_enable=false,
        schedule_enable=false,
        kwargs...,
    )
end

const _SV3 = SVector{3, Float64}

@testset "HYPR search helpers" begin
    @testset "adaptive policy: complexity and probe metrics" begin
        far = _hypr_far_geometry()
        blocked = _hypr_origin_geometry(; keepout_radius_m=0.5)

        # Free corridor: only the smooth clearance term contributes.
        c_free = GH.rpo_estimate_geometry_complexity((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), far)
        @test c_free ≈ 0.3 / (1.0 + 98.99) atol = 1.0e-9
        # Corridor through the keepout sphere saturates to full complexity.
        c_hit = GH.rpo_estimate_geometry_complexity((-1.0, 0.0, 0.0), (1.0, 0.0, 0.0), blocked)
        @test c_hit == 1.0
        # Positive clearance below the safety buffer adds the buffer term.
        c_buf = GH.rpo_estimate_geometry_complexity(
            (-1.0, 0.6, 0.0), (1.0, 0.6, 0.0), blocked; safe_distance_m=0.2)
        @test c_buf ≈ 0.7 + 0.3 / (1.0 + (0.6 - 0.51)) atol = 1.0e-9
        @test 0.0 <= c_buf <= 1.0

        ok = GH.rpo_probe_geometry_metrics((-1.0, 0.6, 0.0), (1.0, 0.6, 0.0), blocked)
        @test ok.detour_ratio == 1.0
        @test ok.min_clearance ≈ 0.6 - 0.51 atol = 1.0e-9
        @test ok.violation_fraction == 0.0
        @test ok.success  # safe_distance defaults to zero
        tight = GH.rpo_probe_geometry_metrics((-1.0, 0.6, 0.0), (1.0, 0.6, 0.0), blocked; safe_distance_m=0.2)
        @test !tight.success
        crossing = GH.rpo_probe_geometry_metrics((-1.0, 0.0, 0.0), (1.0, 0.0, 0.0), blocked)
        @test crossing.min_clearance < 0.0
        @test crossing.violation_fraction > 0.0
        @test !crossing.success
    end

    @testset "adaptive policy: config adaptation" begin
        far = _hypr_far_geometry()
        start = (0.0, 0.0, 0.0)
        goal = (1.0, 0.0, 0.0)

        base_off = _hypr_cfg()
        cfg_off, info_off = GH.rpo_adaptive_pso_config(base_off, start, goal, far)
        @test !info_off.enabled
        @test info_off.complexity == 0.0
        @test info_off.explore == 0.0
        @test info_off.distance_m ≈ 1.0
        @test cfg_off.n_particles == base_off.n_particles
        @test cfg_off.n_iters == base_off.n_iters

        # Downscale allowed with zero maxima triggers the raw fallback sizing.
        base_dn = _hypr_cfg(;
            adaptive_enable=true,
            adaptive_allow_downscale=true,
            adaptive_n_particles_max=0,
            adaptive_n_iters_max=0,
            adaptive_n_particles_min=2,
            adaptive_n_iters_min=1,
            adaptive_n_waypoints_min=1,
            adaptive_n_waypoints_max=2,
        )
        cfg_dn, info_dn = GH.rpo_adaptive_pso_config(base_dn, start, goal, far)
        @test info_dn.enabled
        @test 0.0 <= info_dn.complexity <= 1.0
        @test 0.0 <= info_dn.explore <= 1.0
        @test info_dn.distance_m ≈ 1.0
        @test 1 <= cfg_dn.n_waypoints <= 2
        @test 2 <= cfg_dn.n_particles <= max(20, 3 * base_dn.n_particles)
        @test 1 <= cfg_dn.n_iters <= max(5, 3 * base_dn.n_iters)
        @test base_dn.adaptive_c1_min <= cfg_dn.c1 <= base_dn.adaptive_c1_max
        @test base_dn.adaptive_c2_min <= cfg_dn.c2 <= base_dn.adaptive_c2_max
        @test base_dn.adaptive_w_inertia_min <= cfg_dn.w_inertia <= base_dn.adaptive_w_inertia_max
        @test base_dn.adaptive_spread_scale_min <= cfg_dn.spread_scale <= base_dn.adaptive_spread_scale_max
        @test base_dn.adaptive_w_len_min <= cfg_dn.w_len <= base_dn.adaptive_w_len_max

        # Downscale disallowed keeps at least the base sizing even with tiny maxima.
        base_up = _hypr_cfg(;
            adaptive_enable=true,
            adaptive_allow_downscale=false,
            adaptive_n_particles_max=4,
            adaptive_n_iters_max=1,
            adaptive_n_particles_min=2,
            adaptive_n_iters_min=1,
            adaptive_n_waypoints_min=1,
            adaptive_n_waypoints_max=1,
        )
        cfg_up, info_up = GH.rpo_adaptive_pso_config(base_up, start, goal, far)
        @test info_up.enabled
        @test cfg_up.n_particles >= base_up.n_particles
        @test cfg_up.n_iters >= base_up.n_iters
        @test cfg_up.n_waypoints >= base_up.n_waypoints
    end

    @testset "refinement primitives" begin
        far = _hypr_far_geometry()
        blocked = _hypr_origin_geometry(; keepout_radius_m=0.5)
        cfg = _hypr_cfg(; sample_ds_m=0.25, refinement_sample_ds_m=0.1)

        refine_cfg = GH.rpo_refinement_config(cfg)
        @test refine_cfg.sample_ds_m == 0.1
        @test refine_cfg.refinement_sample_ds_m == 0.1

        better = (J_obs=0.0, total=1.0)
        worse = (J_obs=0.0, total=2.0)
        @test GH.rpo_refinement_better(better, worse, cfg)
        @test !GH.rpo_refinement_better(worse, better, cfg)
        @test !GH.rpo_refinement_better(better, better, cfg)  # zero improvement fails thresholds
        obs_worse = (J_obs=5.0, total=0.1)
        @test !GH.rpo_refinement_better(obs_worse, worse, cfg)  # obstacle regression vetoes

        wild = [0.0 10.0 2.0; 0.0 -50.0 0.0; 0.0 0.0 0.0]
        clamped = GH.rpo_refinement_clamp_path(wild, cfg)
        lo, hi = GH.rpo_pso_bounds(wild[:, 1], wild[:, end], cfg)
        @test clamped[:, 1] == wild[:, 1]
        @test clamped[:, end] == wild[:, end]
        @test all(lo .<= clamped[:, 2] .<= hi)
        @test clamped[2, 2] == lo[2]

        seg = GH.rpo_refinement_segment_samples((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 0.4)
        @test size(seg, 2) == 4  # ceil(1/0.4)+1
        @test seg[:, 1] ≈ [0.0, 0.0, 0.0]
        @test seg[:, end] ≈ [1.0, 0.0, 0.0]
        @test issorted(seg[1, :])
        seg_one = GH.rpo_refinement_segment_samples((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 10.0)
        @test size(seg_one, 2) == 2
        seg_degen = GH.rpo_refinement_segment_samples((1.0, 1.0, 1.0), (1.0, 1.0, 1.0), 0.1)
        @test size(seg_degen, 2) == 2
        @test seg_degen[:, 1] ≈ seg_degen[:, 2]

        # Uniform-sampling safety check.
        @test GH.rpo_refinement_segment_is_safe((-1.0, 2.0, 0.0), (1.0, 2.0, 0.0), blocked, cfg)
        @test !GH.rpo_refinement_segment_is_safe((-1.0, 0.0, 0.0), (1.0, 0.0, 0.0), blocked, cfg)
        # Adaptive-sampling safety check hits the other branch.
        cfg_ad = _hypr_cfg(; adaptive_sampling_enable=true, sample_ds_m=0.1)
        @test GH.rpo_refinement_segment_is_safe((-1.0, 2.0, 0.0), (1.0, 2.0, 0.0), blocked, cfg_ad)
        @test !GH.rpo_refinement_segment_is_safe(
            (-1.0, 0.0, 0.0), (1.0, 0.0, 0.0), blocked, cfg_ad; safe_distance_m=0.05)

        @test GH.rpo_refinement_bernstein(3, 0, 0.0) == 1.0
        @test GH.rpo_refinement_bernstein(3, 3, 1.0) == 1.0
        @test sum(GH.rpo_refinement_bernstein(3, j, 0.3) for j in 0:3) ≈ 1.0

        @test GH.rpo_refinement_sample_params(reshape([1.0, 2.0, 3.0], 3, 1)) == [0.0]
        same = [1.0 1.0 1.0; 0.0 0.0 0.0; 0.0 0.0 0.0]
        @test GH.rpo_refinement_sample_params(same) == [0.0, 0.5, 1.0]  # degenerate arc length
        line3 = [0.0 1.0 3.0; 0.0 0.0 0.0; 0.0 0.0 0.0]
        u = GH.rpo_refinement_sample_params(line3)
        @test u ≈ [0.0, 1.0 / 3.0, 1.0]

        dense_line = GH.rpo_refinement_segment_samples((0.0, 0.0, 0.0), (2.0, 0.0, 0.0), 0.1)
        controls2 = GH.rpo_fit_bezier_fixed_endpoints(dense_line, 2, cfg)
        @test size(controls2) == (3, 2)
        @test controls2[:, 1] ≈ [0.0, 0.0, 0.0]
        @test controls2[:, 2] ≈ [2.0, 0.0, 0.0]
        controls4 = GH.rpo_fit_bezier_fixed_endpoints(dense_line, 4, cfg)
        @test size(controls4) == (3, 4)
        @test controls4[:, 1] ≈ dense_line[:, 1]
        @test controls4[:, end] ≈ dense_line[:, end]
        @test all(abs.(controls4[2, :]) .< 1.0e-6)  # collinear controls for straight data
        @test all(abs.(controls4[3, :]) .< 1.0e-6)
        # n_control below 2 clamps up to endpoints only.
        @test size(GH.rpo_fit_bezier_fixed_endpoints(dense_line, 1, cfg), 2) == 2

        a = _SV3(0.0, 0.0, 0.0)
        b = _SV3(1.0, 0.0, 0.0)
        @test GH.rpo_refinement_project_to_segment((0.5, 1.0, 0.0), a, b) ≈ _SV3(0.5, 0.0, 0.0)
        @test GH.rpo_refinement_project_to_segment((-2.0, 1.0, 0.0), a, b) ≈ a
        @test GH.rpo_refinement_project_to_segment((5.0, -1.0, 0.0), a, b) ≈ b
        @test GH.rpo_refinement_project_to_segment((0.3, 0.4, 0.0), a, a) ≈ a  # degenerate segment
    end

    @testset "refinement passes" begin
        far = _hypr_far_geometry()
        cfg = _hypr_cfg(; refinement_enable=true, sample_ds_m=0.1, refinement_sample_ds_m=0.1)

        zigzag = [0.0 1.0 2.0 3.0; 0.0 1.0 -1.0 0.0; 0.0 0.0 0.0 0.0]
        straightened = GH.rpo_refinement_shortcut_samples(zigzag, far, cfg)
        @test size(straightened, 2) == 2  # whole zigzag collapses in free space
        @test straightened[:, 1] ≈ zigzag[:, 1]
        @test straightened[:, end] ≈ zigzag[:, end]
        two = [0.0 1.0; 0.0 0.0; 0.0 0.0]
        @test GH.rpo_refinement_shortcut_samples(two, far, cfg) == two

        zig_comps = GH.rpo_normalized_path_cost_components(zigzag, far, cfg)
        straight = [0.0 1.0 2.0 3.0; 0.0 0.0 0.0 0.0; 0.0 0.0 0.0 0.0]
        accepted, comps, ok = GH.rpo_try_accept_refinement(straight, far, cfg, zig_comps)
        @test ok
        @test comps.total < zig_comps.total
        @test accepted[:, 1] ≈ zigzag[:, 1]
        rejected, kept, no = GH.rpo_try_accept_refinement(zigzag, far, cfg, zig_comps)
        @test !no
        @test rejected === nothing
        @test kept === zig_comps

        # Shortcut+refit accepts a straightened candidate in free space.
        cand, cand_comps, did = GH.rpo_refine_shortcut_refit(zigzag, far, cfg, zig_comps)
        @test did
        @test cand_comps.total < zig_comps.total
        @test cand[:, 1] ≈ zigzag[:, 1]
        @test cand[:, end] ≈ zigzag[:, end]

        # A blocked elbow with coarse refinement sampling cannot shortcut at all.
        blocked = _hypr_origin_geometry(; keepout_radius_m=0.9)
        elbow = [-1.0 0.0 1.0; 0.0 1.0 0.0; 0.0 0.0 0.0]
        cfg_coarse = _hypr_cfg(; sample_ds_m=0.05, refinement_sample_ds_m=5.0)
        elbow_comps = GH.rpo_normalized_path_cost_components(elbow, blocked, cfg_coarse)
        same_path, same_comps, changed = GH.rpo_refine_shortcut_refit(elbow, blocked, cfg_coarse, elbow_comps)
        @test !changed
        @test same_path == elbow
        @test same_comps === elbow_comps

        # Handle tightening pulls an off-chord handle toward the chord.
        bent = [0.0 1.5 3.0; 0.0 2.0 0.0; 0.0 0.0 0.0]
        bent_comps = GH.rpo_normalized_path_cost_components(bent, far, cfg)
        tight, tight_comps, timproved = GH.rpo_refine_tighten_handles(bent, far, cfg, bent_comps)
        @test timproved
        @test tight_comps.total < bent_comps.total
        @test tight[:, 1] ≈ bent[:, 1]
        @test tight[:, end] ≈ bent[:, end]
        @test abs(tight[2, 2]) < 2.0  # handle moved toward the chord
        short = [0.0 1.0; 0.0 0.0; 0.0 0.0]
        s_path, s_comps, s_improved = GH.rpo_refine_tighten_handles(short, far, cfg, bent_comps)
        @test !s_improved
        @test s_path == short

        # Degree lowering finds the straight 2-control representation.
        detour = [0.0 1.0 2.0 3.0; 0.0 2.0 2.0 0.0; 0.0 0.0 0.0 0.0]
        detour_comps = GH.rpo_normalized_path_cost_components(detour, far, cfg)
        lower, lower_comps, limproved = GH.rpo_refine_lower_degree(detour, far, cfg, detour_comps)
        @test limproved
        @test lower_comps.total < detour_comps.total
        @test size(lower, 2) < size(detour, 2)
        @test lower[:, 1] ≈ detour[:, 1]
        @test lower[:, end] ≈ detour[:, end]
        tri = [0.0 1.0 2.0; 0.0 1.0 0.0; 0.0 0.0 0.0]
        t_path, t_comps, t_improved = GH.rpo_refine_lower_degree(tri, far, cfg, detour_comps)
        @test !t_improved
        @test t_path == tri
    end

    @testset "post refine path" begin
        far = _hypr_far_geometry()
        zigzag = [0.0 1.0 2.0 3.0; 0.0 1.0 -1.0 0.0; 0.0 0.0 0.0 0.0]

        cfg_off = _hypr_cfg(; refinement_enable=false)
        path_off, cost_off, improved_off = GH.rpo_post_refine_path(zigzag, far, cfg_off)
        @test !improved_off
        @test path_off == zigzag
        @test cost_off ≈ GH.rpo_path_cost(zigzag, far, cfg_off)

        cfg_zero = _hypr_cfg(; refinement_enable=true, refinement_rounds=0)
        _, cost_zero, improved_zero = GH.rpo_post_refine_path(zigzag, far, cfg_zero)
        @test !improved_zero
        @test cost_zero ≈ GH.rpo_path_cost(zigzag, far, cfg_zero)

        cfg_on = _hypr_cfg(; refinement_enable=true, sample_ds_m=0.1)
        initial_cost = GH.rpo_path_cost(zigzag, far, cfg_on)
        refined, cost_on, improved_on = GH.rpo_post_refine_path(zigzag, far, cfg_on)
        @test improved_on
        @test cost_on < initial_cost
        @test refined[:, 1] ≈ zigzag[:, 1]
        @test refined[:, end] ≈ zigzag[:, end]
        # Refined free-space path is nearly straight.
        @test GH.rpo_path_length(refined) < GH.rpo_path_length(zigzag)
    end

    @testset "pso swarm helpers" begin
        far = _hypr_far_geometry()

        cfg_flat = _hypr_cfg(; schedule_enable=false)
        w = GH.rpo_pso_iteration_weights(cfg_flat, 2)
        @test w == (w_inertia=cfg_flat.w_inertia, c1=cfg_flat.c1, c2=cfg_flat.c2)
        cfg_sched = _hypr_cfg(; schedule_enable=true, n_iters=10)
        w_last = GH.rpo_pso_iteration_weights(cfg_sched, 10)
        @test w_last.w_inertia < cfg_sched.w_inertia

        a = _SV3(0.0, 0.0, 0.0)
        b = _SV3(2.0, 0.0, 0.0)
        @test GH.rpo_pso_project_to_segment((1.0, 3.0, 0.0), a, b) ≈ _SV3(1.0, 0.0, 0.0)
        @test GH.rpo_pso_project_to_segment((-9.0, 0.0, 0.0), a, b) ≈ a
        @test GH.rpo_pso_project_to_segment((1.0, 1.0, 1.0), a, a) ≈ a

        @test GH.rpo_pso_tapered_noise_scale(1, 1) == 1.0
        @test GH.rpo_pso_tapered_noise_scale(1, 5) == clamp(1.0 / 3.0, 0.25, 1.0)
        @test GH.rpo_pso_tapered_noise_scale(3, 5) == 1.0  # middle waypoint keeps full noise

        # Culling: disabled, warm-up, and unevaluated swarms are all no-ops.
        start = _SV3(0.0, 0.0, 0.0)
        goal = _SV3(1.0, 0.0, 0.0)
        mk_state() = (
            zeros(3, 4), ones(3, 4), zeros(3, 4), [1.0, 2.0, 3.0, 4.0],
            -ones(3), ones(3), zeros(3),
        )
        cfg_cull = _hypr_cfg(;
            n_particles=4, cull_enable=true, cull_start_iter=1, cull_fraction_max=0.5,
            cull_noise_scale=0.1, cull_arc_velocity_scale=0.2)
        pos, vel, pb, pbc, lo, hi, gb = mk_state()
        cfg_no = _hypr_cfg(; n_particles=4, cull_enable=false)
        @test GH.rpo_pso_cull_swarm!(pos, vel, pb, pbc, lo, hi, gb, start, goal, 1, cfg_no, 5, MersenneTwister(1)) == 0
        @test GH.rpo_pso_cull_swarm!(pos, vel, pb, pbc, lo, hi, gb, start, goal, 1, cfg_cull, 0, MersenneTwister(1)) == 0
        inf_costs = fill(Inf, 4)
        @test GH.rpo_pso_cull_swarm!(pos, vel, pb, inf_costs, lo, hi, gb, start, goal, 1, cfg_cull, 5, MersenneTwister(1)) == 0

        # Polyline branch replaces the two worst particles with jittered gbest copies.
        pos, vel, pb, pbc, lo, hi, gb = mk_state()
        n = GH.rpo_pso_cull_swarm!(pos, vel, pb, pbc, lo, hi, gb, start, goal, 1, cfg_cull, 3, MersenneTwister(1))
        @test n == 2
        @test pbc[1] == 1.0 && pbc[2] == 2.0
        @test pbc[3] == Inf && pbc[4] == Inf
        @test all(vel[:, 3] .== 0.0) && all(vel[:, 4] .== 0.0)
        @test all(lo .<= pos[:, 3] .<= hi) && all(lo .<= pos[:, 4] .<= hi)
        @test pb[:, 3] == pos[:, 3] && pb[:, 4] == pos[:, 4]

        # Bezier branch reseeds along corridor-biased targets with arc velocities.
        cfg_bez = _hypr_cfg(;
            n_particles=4, curve_type=:bezier, cull_enable=true, cull_start_iter=1,
            cull_fraction_max=0.5, cull_noise_scale=0.1, cull_arc_velocity_scale=0.2)
        pos, vel, pb, pbc, lo, hi, gb = mk_state()
        n = GH.rpo_pso_cull_swarm!(pos, vel, pb, pbc, lo, hi, gb, start, goal, 1, cfg_bez, 3, MersenneTwister(1))
        @test n == 2
        @test pbc[3] == Inf && pbc[4] == Inf
        @test all(lo .<= pos[:, 3] .<= hi) && all(lo .<= pos[:, 4] .<= hi)
        @test pb[:, 3] == pos[:, 3] && pb[:, 4] == pos[:, 4]

        # Effective safety distance resolution.
        cfg_sd = _hypr_cfg(; safe_distance_m=0.3, sample_ds_m=0.3)
        @test GH.rpo_pso_effective_safe_distance(cfg_sd, nothing) == 0.3
        @test GH.rpo_pso_effective_safe_distance(cfg_sd, 0.5) == 0.5
        @test GH.rpo_pso_effective_safe_distance(cfg_sd, 0.0) == 0.3  # zero defers to config
        cfg_sd0 = _hypr_cfg()
        @test GH.rpo_pso_effective_safe_distance(cfg_sd0, 0.0) == 0.0

        diag = GH.rpo_pso_empty_warmstart_diagnostics(_hypr_cfg(; rrt_warmstart_enable=true))
        @test diag.enabled && !diag.attempted && !diag.path_found
        @test diag.iterations == 0 && diag.cost == Inf && diag.n_points == 0

        # Warm start generation: disabled and free-space direct hit.
        p_off, d_off = GH.rpo_pso_rrt_warmstart_path(start, goal, far, _hypr_cfg(), 0.0, MersenneTwister(1))
        @test p_off === nothing
        @test !d_off.enabled && !d_off.attempted
        cfg_ws = _hypr_cfg(; rrt_warmstart_enable=true, rrt_warmstart_iters=50)
        p_on, d_on = GH.rpo_pso_rrt_warmstart_path(start, goal, far, cfg_ws, 0.0, MersenneTwister(1))
        @test d_on.attempted && d_on.path_found
        @test d_on.n_points == 2
        @test p_on == hcat(collect(start), collect(goal))
        @test isfinite(d_on.cost)

        mask = GH.rpo_pso_protected_particle_mask([3.0, 1.0, 2.0], 0.34)
        @test mask == [false, true, true]

        # Failed stagnation-learning attempts keep the particle eligible for another
        # attempt next iteration; only an accepted attempt clears the counter.
        @test GH.rpo_pso_stagnation_count_after_learning(3, false) == 3
        @test GH.rpo_pso_stagnation_count_after_learning(3, true) == 0
        @test GH.rpo_pso_stagnation_count_after_learning(0, false) == 0

        cfg_imp = _hypr_cfg()
        @test GH.rpo_pso_material_improvement(0.5, 1.0, cfg_imp)
        @test !GH.rpo_pso_material_improvement(1.0, 1.0, cfg_imp)
        @test GH.rpo_pso_material_improvement(1.0, Inf, cfg_imp)

        cfg_feas = _hypr_cfg(; early_stopping_require_feasible=true)
        @test !GH.rpo_pso_early_stopping_feasible(nothing, cfg_feas)
        @test GH.rpo_pso_early_stopping_feasible((violation_count=0,), cfg_feas)
        @test !GH.rpo_pso_early_stopping_feasible((violation_count=3,), cfg_feas)
        cfg_lax = _hypr_cfg(; early_stopping_require_feasible=false)
        @test GH.rpo_pso_early_stopping_feasible(nothing, cfg_lax)
    end
end
end

# From SpaceAGORA's test/probes/robot_arm_hypr_probes.jl.
module RobotArmHelpers
using Test, LinearAlgebra, Random, StaticArrays, SpaceAGORA, HYPR
const RA = Base.get_extension(HYPR, :HYPRSpaceAGORAExt).RobotArm

"Assert every column of a joint matrix respects the model's joint limits."
function _probe_within_joint_limits(model, q_matrix; tol=1.0e-9)
    lo = [joint.lower_rad for joint in model.joints]
    hi = [joint.upper_rad for joint in model.joints]
    for k in axes(q_matrix, 2)
        all(q_matrix[:, k] .>= lo .- tol) || return false
        all(q_matrix[:, k] .<= hi .+ tol) || return false
    end
    return true
end

@testset "HYPR robot-arm helpers" begin
    model = SpaceAGORA.default_cloth_arm_model()
    base = SpaceAGORA.ClothArmBasePose([0.0, 0.0, 0.0])
    planner_cfg = SpaceAGORA.RobotArmPlannerConfig(dt_s=0.1, duration_s=0.5, ik_tol_m=1.0e-3)
    q0 = [0.0, 0.3, -0.3]
    q_goal_truth = [0.6, -0.4, 0.4]
    target = SpaceAGORA.cloth_fk(model, base, q_goal_truth).end_effector_position
    q_goal_ik = SpaceAGORA.cloth_ik(
        model,
        base,
        SVector{3, Float64}(target);
        q_seed=q0,
        max_iters=planner_cfg.ik_max_iters,
        position_tol_m=planner_cfg.ik_tol_m,
        damping=planner_cfg.ik_damping,
    )
    no_obstacles = SpaceAGORA.RobotArmSphereObstacle[]

    @testset "refinement comparison and post refinement" begin
        better = RA._robot_arm_hypr_refinement_better
        cfg_ref = SpaceAGORA.RobotArmHYPRConfig()
        lower_obs = (J_obs=0.0, min_clearance=1.0, total=10.0)
        higher_obs = (J_obs=1.0, min_clearance=0.0, total=1.0)
        @test better(lower_obs, higher_obs, cfg_ref) === true
        @test better(higher_obs, lower_obs, cfg_ref) === false
        clr_a = (J_obs=1.0, min_clearance=0.5, total=3.0)
        clr_b = (J_obs=1.0, min_clearance=0.1, total=3.0)
        @test better(clr_a, clr_b, cfg_ref) === true
        cost_a = (J_obs=0.0, min_clearance=1.0, total=1.0)
        cost_b = (J_obs=0.0, min_clearance=1.0, total=2.0)
        @test better(cost_a, cost_b, cfg_ref) === true
        @test better(cost_b, cost_b, cfg_ref) === false

        # Post-refinement early return when refinement is disabled.
        cfg_noref = SpaceAGORA.RobotArmHYPRConfig(refinement_enable=false, n_samples=10)
        pts4 = hcat(q0, 0.7 .* q0 .+ 0.3 .* q_goal_truth, 0.3 .* q0 .+ 0.7 .* q_goal_truth, q_goal_truth)
        refined, refined_comps, improved, rounds = RA._robot_arm_hypr_post_refine_points(
            pts4, model, base, no_obstacles, cfg_noref,
        )
        @test refined == pts4
        @test improved === false
        @test rounds == 0
        @test refined_comps.total ≈ SpaceAGORA.robot_arm_hypr_path_cost_components(
            pts4, model, base, no_obstacles, cfg_noref,
        ).total

        # A strongly kinked midpoint gives refinement room to actually improve.
        cfg_refine = SpaceAGORA.RobotArmHYPRConfig(
            n_samples=12, curve_type=:polyline, refinement_rounds=4,
        )
        kink = 0.5 .* (q0 .+ q_goal_truth) .+ [1.5, -1.5, 1.5]
        pts_kinked = hcat(q0, kink, q_goal_truth)
        base_comps = SpaceAGORA.robot_arm_hypr_path_cost_components(
            pts_kinked, model, base, no_obstacles, cfg_refine,
        )
        better_pts, better_comps, did_improve, ran_rounds = RA._robot_arm_hypr_post_refine_points(
            pts_kinked, model, base, no_obstacles, cfg_refine,
        )
        @test did_improve === true
        @test ran_rounds >= 1
        @test better_comps.total < base_comps.total
        @test better_pts[:, 1] == q0 && better_pts[:, end] == q_goal_truth
        @test _probe_within_joint_limits(model, better_pts)
    end

    @testset "rrt warmstart" begin
        # Direct-path branch: no obstacles means the straight segment is safe.
        cfg_direct = SpaceAGORA.RobotArmHYPRConfig(
            n_waypoints=1,
            n_particles=6,
            n_iters=2,
            n_samples=10,
            rrt_warmstart_enable=true,
            rrt_warmstart_iters=8,
            early_stopping_enable=false,
            refinement_enable=false,
        )
        direct = SpaceAGORA.plan_robot_arm_motion_hypr(
            model, base, q0, target;
            planner_config=planner_cfg,
            hypr_config=cfg_direct,
            obstacles=no_obstacles,
            rng=MersenneTwister(46),
        )
        @test direct.components.rrt_warmstart_enabled === true
        @test direct.components.rrt_warmstart_attempted === true
        @test direct.components.rrt_warmstart_path_found === true
        @test direct.components.rrt_warmstart_iterations == 0
        @test direct.components.rrt_warmstart_n_points == 2
        @test isfinite(direct.components.rrt_warmstart_cost)
        @test direct.components.rrt_warmstart_cost == direct.components.rrt_warmstart_raw_cost
        @test direct.components.rrt_warmstart.path_found === true

        # Blocked direct path: the trees must grow and connect around the sphere.
        q_mid = 0.5 .* (q0 .+ q_goal_ik)
        blocker = SpaceAGORA.RobotArmSphereObstacle(
            SpaceAGORA.cloth_fk(model, base, q_mid).end_effector_position, 0.02,
        )
        cfg_rrt = SpaceAGORA.RobotArmHYPRConfig(
            n_waypoints=2,
            n_particles=6,
            n_iters=2,
            n_samples=12,
            safe_distance_m=0.004,
            rrt_warmstart_enable=true,
            rrt_warmstart_iters=600,
            rrt_warmstart_step_size_rad=0.35,
            rrt_warmstart_goal_sample_rate=0.2,
            rrt_warmstart_collision_sample_ds_rad=0.15,
            rrt_warmstart_shortcut_iters=15,
            early_stopping_enable=false,
            refinement_enable=false,
        )
        warm_path, warm_diag = RA._robot_arm_rrt_connect_warmstart_path(
            q0, q_goal_ik, model, base, [blocker], cfg_rrt, MersenneTwister(101),
        )
        @test warm_diag.attempted === true
        @test warm_diag.iterations >= 1
        @test warm_diag.path_found === true
        @test warm_path !== nothing
        @test size(warm_path, 1) == 3
        @test warm_diag.n_points == size(warm_path, 2) >= 2
        @test warm_path[:, 1] ≈ q0
        @test warm_path[:, end] ≈ q_goal_ik
        @test isfinite(warm_diag.cost) && isfinite(warm_diag.raw_cost)
        # The warm-started path must itself be collision free at the checked tolerance.
        warm_stats = SpaceAGORA.robot_arm_clearance_stats_from_samples(
            model,
            base,
            SpaceAGORA.robot_arm_sample_hypr_path(warm_path, 40; curve_type=:polyline),
            [blocker],
            cfg_rrt.safe_distance_m,
        )
        @test warm_stats.min_clearance > 0.0

        warmstarted = SpaceAGORA.plan_robot_arm_motion_hypr(
            model, base, q0, target;
            planner_config=planner_cfg,
            hypr_config=cfg_rrt,
            obstacles=[blocker],
            rng=MersenneTwister(101),
        )
        @test warmstarted.components.rrt_warmstart_path_found === true
        @test warmstarted.control_points[:, 1] ≈ q0
        @test warmstarted.control_points[:, end] ≈ warmstarted.plan.q_goal

        # Runtime-limited warm start bails out after the first iteration.
        cfg_budget = SpaceAGORA.RobotArmHYPRConfig(
            rrt_warmstart_enable=true,
            rrt_warmstart_iters=500,
            rrt_warmstart_step_size_rad=0.35,
            rrt_warmstart_goal_sample_rate=0.0,
            rrt_warmstart_collision_sample_ds_rad=0.15,
            rrt_warmstart_runtime_limit_s=0.0,
            rrt_warmstart_connect_max_steps=1,
            safe_distance_m=0.004,
        )
        budget_path, budget_diag = RA._robot_arm_rrt_connect_warmstart_path(
            q0, q_goal_ik, model, base, [blocker], cfg_budget, MersenneTwister(102),
        )
        @test budget_diag.attempted === true
        @test budget_diag.iterations <= 1
        @test budget_diag.path_found === false
        @test budget_path === nothing
        @test budget_diag.n_points == 2
        @test budget_diag.cost == budget_diag.raw_cost

        # Disabled warm start returns empty diagnostics.
        cfg_off = SpaceAGORA.RobotArmHYPRConfig(rrt_warmstart_enable=false)
        off_path, off_diag = RA._robot_arm_rrt_connect_warmstart_path(
            q0, q_goal_ik, model, base, [blocker], cfg_off, MersenneTwister(1),
        )
        @test off_path === nothing
        @test off_diag.enabled === false
        @test off_diag.attempted === false
        @test off_diag.cost == Inf

        # Segment sampling covers coarse and fine discretizations.
        seg = RA._robot_arm_rrt_segment_samples([0.0, 0.0, 0.0], [0.1, 0.0, 0.0], 1.0)
        @test size(seg) == (3, 2)
        seg_fine = RA._robot_arm_rrt_segment_samples([0.0, 0.0, 0.0], [0.1, 0.0, 0.0], 0.01)
        @test size(seg_fine, 2) == 11
        @test seg_fine[1, 1] == 0.0 && seg_fine[1, end] ≈ 0.1

        # Tree primitives.
        tree = RA.RobotArmRRTConnectTree([0.0, 0.0, 0.0])
        @test tree.nodes == [[0.0, 0.0, 0.0]]
        @test RA._robot_arm_rrt_nearest_index(tree, [1.0, 0.0, 0.0]) == 1
        status, idx = RA._robot_arm_rrt_extend!(
            tree, [1.0, 0.0, 0.0], model, base, no_obstacles,
            SpaceAGORA.RobotArmHYPRConfig(rrt_warmstart_step_size_rad=0.25),
        )
        @test status == :advanced
        @test idx == 2
        @test norm(tree.nodes[2] .- [0.25, 0.0, 0.0]) < 1.0e-9
        @test tree.costs[2] ≈ 0.25

        # Shortcutting a zigzag polyline with no obstacles shrinks or preserves length.
        zig = hcat(
            [0.0, 0.0, 0.0],
            [0.2, 0.4, 0.0],
            [0.4, -0.4, 0.0],
            [0.6, 0.0, 0.0],
        )
        cut = RA._robot_arm_rrt_shortcut_path(
            zig, model, base, no_obstacles, cfg_rrt, MersenneTwister(3),
        )
        @test RA._robot_arm_path_length(cut) <= RA._robot_arm_path_length(zig) + 1.0e-9
        @test cut[:, 1] == zig[:, 1] && cut[:, end] == zig[:, end]
        two = hcat([0.0, 0.0, 0.0], [1.0, 0.0, 0.0])
        @test RA._robot_arm_rrt_shortcut_path(
            two, model, base, no_obstacles, cfg_rrt, MersenneTwister(3),
        ) == two

        # Polyline resampling branches.
        @test_throws ArgumentError RA._robot_arm_resample_polyline_points(zig, 1)
        same = RA._robot_arm_resample_polyline_points(zig, 4)
        @test same == zig && same !== zig
        expanded = RA._robot_arm_resample_polyline_points(zig, 7)
        @test size(expanded) == (3, 7)
        @test expanded[:, 1] == zig[:, 1] && expanded[:, end] == zig[:, end]
        single = reshape([1.0, 2.0, 3.0], 3, 1)
        widened = RA._robot_arm_resample_polyline_points(single, 3)
        @test widened == hcat(single, zeros(3), single)
        degenerate = hcat([1.0, 1.0, 1.0], [1.0, 1.0, 1.0])
        deg = RA._robot_arm_resample_polyline_points(degenerate, 4)
        @test all(deg .== 1.0)
    end

    @testset "retiming and base wrench" begin
        # Velocity plus acceleration limited retime stretches the schedule.
        cfg_vel = SpaceAGORA.RobotArmHYPRConfig(
            n_waypoints=0,
            n_samples=8,
            retime_enable=true,
            retime_max_joint_velocity_rad_s=0.2,
            retime_max_joint_acceleration_rad_s2=0.8,
        )
        retimed = SpaceAGORA.plan_robot_arm_motion_hypr(
            model, base, q0, target;
            planner_config=planner_cfg,
            hypr_config=cfg_vel,
            obstacles=no_obstacles,
            rng=MersenneTwister(44),
        )
        @test retimed.plan.t_ref_s[end] > planner_cfg.duration_s
        @test issorted(retimed.plan.t_ref_s) && allunique(retimed.plan.t_ref_s)
        @test retimed.components.retime_enabled === true
        @test retimed.components.retime_duration_s == retimed.plan.t_ref_s[end]
        @test retimed.components.retime_scale_max ≈
            retimed.plan.t_ref_s[end] / planner_cfg.duration_s
        @test maximum(abs.(retimed.plan.dq_ref)) <=
            cfg_vel.retime_max_joint_velocity_rad_s + 1.0e-8

        # Reaction-time tapered retime (no wrench limits) stays within scale bounds.
        cfg_react = SpaceAGORA.RobotArmHYPRConfig(
            n_waypoints=0,
            n_samples=8,
            retime_enable=true,
            retime_max_joint_velocity_rad_s=0.2,
            retime_reaction_time_s=0.5,
            retime_reaction_gain=2.0,
            retime_max_scale=50.0,
        )
        reacted = SpaceAGORA.plan_robot_arm_motion_hypr(
            model, base, q0, target;
            planner_config=planner_cfg,
            hypr_config=cfg_react,
            obstacles=no_obstacles,
            rng=MersenneTwister(44),
        )
        @test reacted.plan.t_ref_s[end] >= retimed.plan.t_ref_s[end] - 1.0e-9
        @test reacted.plan.t_ref_s[end] <=
            cfg_react.retime_max_scale * planner_cfg.duration_s + 1.0e-9

        @test RA._robot_arm_hypr_reaction_scale(SpaceAGORA.RobotArmHYPRConfig(), 3.0, 2, 5) == 1.0
        cfg_scale = SpaceAGORA.RobotArmHYPRConfig(retime_reaction_time_s=1.0, retime_reaction_gain=1.0)
        @test RA._robot_arm_hypr_reaction_scale(cfg_scale, 1.0, 1, 3) == 1.0
        @test RA._robot_arm_hypr_reaction_scale(cfg_scale, 1.0, 2, 3) ≈ 2.0

        t_from_scales = RA._robot_arm_hypr_reference_times_from_scales(0.1, [1.0, 2.0, 1.0])
        @test t_from_scales ≈ [0.0, 0.1, 0.3, 0.4]

        # Rigid-model wrench ratios: disabled limits short-circuit to zero.
        q_ref = SpaceAGORA.robot_arm_sample_hypr_path(hcat(q0, q_goal_ik), 6; curve_type=:polyline)
        t_ref = collect(0.0:0.1:0.5)
        cfg_plain = SpaceAGORA.RobotArmHYPRConfig()
        zero_ratios = RA._robot_arm_hypr_rigid_base_wrench_ratios(model, base, q_ref, t_ref, cfg_plain)
        @test zero_ratios.force_ratio == 0.0
        @test zero_ratios.torque_ratio == 0.0

        cfg_rigid = SpaceAGORA.RobotArmHYPRConfig(
            retime_cloth_physics_enable=false,
            retime_max_base_force_n=0.5,
            retime_max_base_torque_nm=0.5,
        )
        rigid_ratios = RA._robot_arm_hypr_rigid_base_wrench_ratios(model, base, q_ref, t_ref, cfg_rigid)
        @test rigid_ratios.force_ratio > 0.0
        @test rigid_ratios.torque_ratio > 0.0
        @test length(rigid_ratios.node_ratio) == size(q_ref, 2)
        @test maximum(rigid_ratios.node_ratio) >= max(rigid_ratios.force_ratio, rigid_ratios.torque_ratio) - 1.0e-12

        single_ratios = RA._robot_arm_hypr_rigid_base_wrench_ratios(
            model, base, reshape(q0, 3, 1), [0.0], cfg_rigid,
        )
        @test single_ratios.force_ratio == 0.0
        @test single_ratios.node_ratio == [0.0]

        # Dispatch: cloth physics disabled routes to the rigid estimator.
        dispatch_ratios = RA._robot_arm_hypr_base_wrench_ratios(model, base, q_ref, t_ref, cfg_rigid)
        @test dispatch_ratios.force_ratio ≈ rigid_ratios.force_ratio
        @test dispatch_ratios.torque_ratio ≈ rigid_ratios.torque_ratio

        # Rigid wrench-limited retime iterates the scale loop.
        cfg_wrench = SpaceAGORA.RobotArmHYPRConfig(
            n_waypoints=0,
            n_samples=8,
            retime_enable=true,
            retime_cloth_physics_enable=false,
            retime_max_base_force_n=0.02,
            retime_max_base_torque_nm=0.05,
            retime_base_wrench_iters=4,
        )
        wrenched = SpaceAGORA.plan_robot_arm_motion_hypr(
            model, base, q0, target;
            planner_config=planner_cfg,
            hypr_config=cfg_wrench,
            obstacles=no_obstacles,
            rng=MersenneTwister(45),
        )
        @test wrenched.plan.t_ref_s[end] > planner_cfg.duration_s
        @test wrenched.components.retime_base_force_ratio <= 1.0 + 1.0e-6
        @test wrenched.components.retime_base_force_ratio >= 0.0
        @test isfinite(wrenched.components.retime_base_torque_ratio)

        # Cloth-coupled wrench estimation drives the coupled simulation path.
        cfg_cloth = SpaceAGORA.RobotArmHYPRConfig(
            n_waypoints=0,
            n_samples=8,
            retime_enable=true,
            retime_max_base_force_n=0.05,
            retime_max_base_torque_nm=0.25,
            retime_base_wrench_iters=2,
            retime_cloth_dt_s=0.05,
        )
        cloth_retimed = SpaceAGORA.plan_robot_arm_motion_hypr(
            model, base, [0.0, -0.05, 0.05],
            SpaceAGORA.cloth_fk(model, base, [0.08, -0.12, 0.08]).end_effector_position;
            planner_config=planner_cfg,
            hypr_config=cfg_cloth,
            obstacles=no_obstacles,
            rng=MersenneTwister(45),
        )
        @test cloth_retimed.plan.t_ref_s[end] > planner_cfg.duration_s
        @test isfinite(cloth_retimed.components.retime_base_force_ratio)
        @test isfinite(cloth_retimed.components.retime_base_torque_ratio)
        @test issorted(cloth_retimed.plan.t_ref_s)

        # Direct retime-reference branches.
        nominal = RA._robot_arm_hypr_retime_reference(
            model, base, reshape(q0, 3, 1), planner_cfg, cfg_vel,
        )
        @test nominal[1] == 0.0 && nominal[end] ≈ planner_cfg.duration_s
        off = RA._robot_arm_hypr_retime_reference(
            model, base, q_ref, planner_cfg, SpaceAGORA.RobotArmHYPRConfig(retime_enable=false),
        )
        @test off[end] ≈ planner_cfg.duration_s
        @test_throws ArgumentError RA._robot_arm_hypr_retime_reference(
            model, base, q_ref, SpaceAGORA.RobotArmPlannerConfig(duration_s=0.0), cfg_vel,
        )
    end

    @testset "swarm helpers" begin
        seeded = RA._robot_arm_seed_control_points(q0, q_goal_ik, 3)
        @test size(seeded) == (3, 5)
        @test seeded[:, 1] == q0
        @test seeded[:, end] == q_goal_ik
        @test seeded[:, 3] ≈ 0.5 .* (q0 .+ q_goal_ik)

        flat = RA._robot_arm_flatten_internal_points(seeded)
        @test length(flat) == 9
        rebuilt = RA._robot_arm_control_points(q0, q_goal_ik, flat, 3)
        @test rebuilt ≈ seeded

        pts = hcat(q0, q_goal_ik)
        poly = SpaceAGORA.robot_arm_sample_hypr_path(pts, 5; curve_type=:polyline)
        @test size(poly) == (3, 5)
        @test poly[:, 1] ≈ q0 && poly[:, end] ≈ q_goal_ik
        bez = SpaceAGORA.robot_arm_sample_hypr_path(seeded, 9; curve_type=:bezier)
        @test size(bez) == (3, 9)
        @test bez[:, 1] ≈ q0 && bez[:, end] ≈ q_goal_ik

        @test RA._robot_arm_path_length(poly) ≈ norm(q_goal_ik .- q0)
        @test RA._robot_arm_path_smoothness(hcat(q0, q_goal_ik)) == 0.0
        @test RA._robot_arm_path_smoothness(poly) ≈ 0.0 atol=1.0e-12
        kinked = hcat([0.0, 0.0, 0.0], [1.0, 1.0, 0.0], [2.0, 0.0, 0.0])
        @test RA._robot_arm_path_smoothness(kinked) > 0.0

        cfg_default = SpaceAGORA.RobotArmHYPRConfig()
        @test RA._robot_arm_hypr_material_improvement(1.0, 2.0, cfg_default) === true
        @test RA._robot_arm_hypr_material_improvement(2.0, 1.0, cfg_default) === false
        @test RA._robot_arm_hypr_early_stopping_feasible((J_obs=5.0,), cfg_default) === true
        cfg_feas = SpaceAGORA.RobotArmHYPRConfig(early_stopping_require_feasible=true)
        @test RA._robot_arm_hypr_early_stopping_feasible((J_obs=0.0,), cfg_feas) === true
        @test RA._robot_arm_hypr_early_stopping_feasible((J_obs=1.0,), cfg_feas) === false

        weights_sched = RA._robot_arm_hypr_iteration_weights(SpaceAGORA.RobotArmHYPRConfig(n_iters=10), 5)
        @test all(isfinite, (weights_sched.w_inertia, weights_sched.c1, weights_sched.c2))
        weights_flat = RA._robot_arm_hypr_iteration_weights(
            SpaceAGORA.RobotArmHYPRConfig(schedule_enable=false), 3,
        )
        @test weights_flat.w_inertia == SpaceAGORA.RobotArmHYPRConfig().w_inertia

        # Culling: replaces the worst particles and resets their bests.
        cull_cfg = SpaceAGORA.RobotArmHYPRConfig(cull_enable=true, cull_start_iter=1, cull_fraction_max=0.5)
        positions = [0.1 0.2 0.3 0.4; 0.1 0.2 0.3 0.4; 0.1 0.2 0.3 0.4]
        velocities = ones(3, 4)
        pbest = copy(positions)
        pbest_cost = [1.0, 2.0, 3.0, 4.0]
        lo = fill(-1.0, 3)
        hi = fill(1.0, 3)
        gbest = zeros(3)
        n_replaced = RA._robot_arm_hypr_cull_swarm!(
            positions, velocities, pbest, pbest_cost, lo, hi, gbest, cull_cfg, 5, MersenneTwister(9),
        )
        @test n_replaced == 2
        @test count(isinf, pbest_cost) == 2
        @test pbest_cost[1] == 1.0 && pbest_cost[2] == 2.0
        @test all(velocities[:, 3] .== 0.0) && all(velocities[:, 4] .== 0.0)
        @test all(-1.0 .<= positions .<= 1.0)
        @test pbest[:, 3] == positions[:, 3]

        off_cfg = SpaceAGORA.RobotArmHYPRConfig(cull_enable=false)
        @test RA._robot_arm_hypr_cull_swarm!(
            positions, velocities, pbest, pbest_cost, lo, hi, gbest, off_cfg, 50, MersenneTwister(9),
        ) == 0
        early_cfg = SpaceAGORA.RobotArmHYPRConfig(cull_enable=true, cull_start_iter=100)
        @test RA._robot_arm_hypr_cull_swarm!(
            positions, velocities, pbest, pbest_cost, lo, hi, gbest, early_cfg, 5, MersenneTwister(9),
        ) == 0
        all_inf_cost = fill(Inf, 4)
        @test RA._robot_arm_hypr_cull_swarm!(
            positions, velocities, pbest, all_inf_cost, lo, hi, gbest, cull_cfg, 5, MersenneTwister(9),
        ) == 0
        one_particle = reshape([0.0, 0.0, 0.0], 3, 1)
        @test RA._robot_arm_hypr_cull_swarm!(
            one_particle, zeros(3, 1), copy(one_particle), [1.0], lo, hi, gbest, cull_cfg, 5,
            MersenneTwister(9),
        ) == 0
    end
end
end

# From SpaceAGORA's test/probes/rpo_planning_probes.jl.
module PlanningHelpers
using Test, LinearAlgebra, Random, StaticArrays, SpaceAGORA, HYPR
const EXT = Base.get_extension(HYPR, :HYPRSpaceAGORAExt)
const GH = EXT.RPO
# The swarm policy is HYPR's own; the extension keeps it under its earlier name.
const HU = EXT.SwarmPolicy
const G = SpaceAGORA.HYPRServices.RPO

"Small station geometry far away from the RTN origin so it does not disturb local paths."
function _probe_far_geometry(; keepout_radius_m::Real=0.0)
    station = SpaceAGORA.RPOStationGeometry(
        reshape([100.0, 0.0, 0.0], 3, 1);
        keepout_radius_m=keepout_radius_m,
        name="probe_far",
    )
    return G.RPOReferenceGeometry(station; chaser=SpaceAGORA.RPOCubeSatGeometry(dims_m=(0.02, 0.02, 0.02)))
end

"Station geometry with a single point at the origin."
function _probe_origin_geometry(; keepout_radius_m::Real=0.0)
    station = SpaceAGORA.RPOStationGeometry(
        reshape([0.0, 0.0, 0.0], 3, 1);
        keepout_radius_m=keepout_radius_m,
        name="probe_origin",
    )
    return G.RPOReferenceGeometry(station; chaser=SpaceAGORA.RPOCubeSatGeometry(dims_m=(0.02, 0.02, 0.02)))
end

"Cheap deterministic-ish PSO config for fast plan builds."
function _probe_tiny_cfg(; kwargs...)
    return SpaceAGORA.RPOPSOConfig(;
        n_waypoints=1,
        n_particles=8,
        n_iters=2,
        sample_ds_m=0.25,
        curve_type=:polyline,
        adaptive_enable=false,
        adaptive_sampling_enable=false,
        probe_enable=false,
        refinement_enable=false,
        retime_a_max_mps2=0.1,
        retime_dt_s=1.0,
        rrt_warmstart_iters=120,
        rrt_warmstart_step_size_m=0.5,
        kwargs...,
    )
end

@testset "HYPR planning helpers" begin
    @testset "HYPRUtils PSO bookkeeping" begin
        base = HU.hypr_iteration_weights(false, 10, 5, 0.7, 1.4, 1.4, 0.5, 0.25, 0.65, 0.75, 1.25, 0.5, 2.5)
        @test base == (w_inertia=0.7, c1=1.4, c2=1.4)
        first_iter = HU.hypr_iteration_weights(true, 10, 1, 0.7, 1.4, 1.4, 0.5, 0.25, 0.65, 0.75, 1.25, 0.5, 2.5)
        @test first_iter.w_inertia ≈ 0.7
        last_iter = HU.hypr_iteration_weights(true, 10, 10, 0.7, 1.4, 1.4, 0.5, 0.25, 0.65, 0.75, 1.25, 0.5, 2.5)
        @test last_iter.w_inertia ≈ max(0.25, 0.65 * 0.7)
        @test last_iter.c1 ≈ clamp(0.75 * 1.4, 0.5, 2.5)
        @test last_iter.c2 ≈ clamp(1.25 * 1.4, 0.5, 2.5)
        # n_iters <= 1 short-circuits even with schedule on.
        @test HU.hypr_iteration_weights(true, 1, 1, 0.7, 1.4, 1.4, 0.5, 0.25, 0.65, 0.75, 1.25, 0.5, 2.5).w_inertia == 0.7

        @test !HU.hypr_material_improvement(NaN, 1.0, 1.0e-8, 1.0e-4)
        @test HU.hypr_material_improvement(1.0, Inf, 1.0e-8, 1.0e-4)
        @test HU.hypr_material_improvement(0.5, 1.0, 1.0e-8, 1.0e-4)
        @test !HU.hypr_material_improvement(1.0, 1.0, 1.0e-8, 1.0e-4)
        # Relative threshold dominates for large reference costs.
        @test !HU.hypr_material_improvement(999.99, 1000.0, 1.0e-8, 1.0e-3)

        @test isempty(HU.hypr_protected_particle_mask(Float64[], 0.5))
        mask = HU.hypr_protected_particle_mask([3.0, 1.0, 2.0], 0.34)
        @test mask == [false, true, true] # two lowest costs protected
        @test count(HU.hypr_protected_particle_mask([3.0, 1.0, 2.0], 0.0)) == 1
    end

    @testset "RPO path retiming" begin
        cfg = _probe_tiny_cfg(; sample_ds_m=0.5)
        far_geom = _probe_far_geometry()
        path = [0.0 4.0; 0.0 0.0; 0.0 0.0]
        mat, s_hist, v_hist = GH.rpo_retime_path(path, far_geom, cfg)
        @test mat[:, 1] ≈ path[:, 1] atol = 1.0e-9
        @test mat[:, end] ≈ path[:, 2] atol = 1.0e-9
        @test length(s_hist) == length(v_hist) == size(mat, 2)
        @test all(diff(s_hist) .> 0.0)  # retiming must advance strictly monotonically
        @test s_hist[end] ≈ 4.0 atol = 1.0e-6
        @test all(v_hist .> 0.0)

        # Zero-length path warns and returns a single-point reference.
        degenerate = [1.0 1.0; 2.0 2.0; 0.0 0.0]
        mat0, s0, v0 = @test_logs (:warn, r"zero-length path") match_mode = :any GH.rpo_retime_path(degenerate, far_geom, cfg)
        @test size(mat0) == (3, 1)
        @test s0 == [0.0] && v0 == [0.0]

        # Path passing exactly through a station point triggers the infeasible fallback.
        origin_geom = _probe_origin_geometry()
        crossing = [-1.0 1.0; 0.0 0.0; 0.0 0.0]
        matx, sx, vx = @test_logs (:warn, r"infeasible zero-speed") match_mode = :any GH.rpo_retime_path(crossing, origin_geom, cfg)
        @test sx[end] ≈ 2.0 atol = 1.0e-6
        @test all(vx .> 0.0)
        @test minimum(vx) <= 1.0e-3 + 1.0e-12  # fallback speed used near the intersection

        # A finite speed cap clamps both the local speed and the fallback speed.
        cfg_capped = _probe_tiny_cfg(; sample_ds_m=0.5, retime_max_speed_mps=0.05)
        matc, sc, vc = GH.rpo_retime_path(path, far_geom, cfg_capped)
        @test all(vc .<= 0.05 + 1.0e-12)
        @test sc[end] ≈ 4.0 atol = 1.0e-6

        # Step-count exhaustion still forces the endpoint into the result.
        cfg_short = _probe_tiny_cfg(; sample_ds_m=0.5, retime_max_steps=2, retime_speed_scale=0.05)
        matm, sm, vm = @test_logs (:warn, r"maximum step count") match_mode = :any GH.rpo_retime_path(path, far_geom, cfg_short)
        @test sm[end] ≈ 4.0 atol = 1.0e-9
        @test matm[:, end] ≈ path[:, 2] atol = 1.0e-9
    end
end
end

# From SpaceAGORA's test/unit/gnc/rpo_hypr_manuscript_tests.jl.
module ManuscriptHelpers
using Test, LinearAlgebra, Random, StaticArrays, SpaceAGORA, HYPR
const HM_GH = Base.get_extension(HYPR, :HYPRSpaceAGORAExt).RPO
const G = SpaceAGORA.HYPRServices.RPO

# A station reduced to one point far from the test paths, so clearance never limits them.
function _hm_far_geometry(point=(0.0, 0.0, 500.0); keepout=0.0)
    station = SpaceAGORA.RPOStationGeometry(reshape(collect(Float64, point), 3, 1); keepout_radius_m=keepout)
    return G.RPOReferenceGeometry(station; chaser=SpaceAGORA.RPOCubeSatGeometry(dims_m=(0.02, 0.02, 0.02)))
end

function _hm_manuscript_cfg(; kwargs...)
    return SpaceAGORA.rpo_pso_config(SpaceAGORA.RPOPSOConfig(;
        hypr_mode=:manuscript,
        retime_accel_limit_enable=true,
        retime_dt_s=0.1,
        retime_a_max_mps2=0.00625,
        retime_speed_scale=0.5,
        sample_ds_m=1.0,
        adaptive_sampling_max_ds_m=4.0,
        adaptive_sampling_far_clearance_m=20.0,
        obstacle_sigmoid_k=1.0e5,
        mean_motion_radps=0.00113,
        mass_kg=5.2,
        kwargs...,
    ))
end

@testset "RPO HyPR retimer boundary speeds and acceleration limit" begin
    a_max = 0.00625
    cfg = _hm_manuscript_cfg(safe_distance_m=1.0)
    geom = _hm_far_geometry()
    path = [0.0 10.0 20.0 30.0; -100.0 -40.0 40.0 100.0; 0.0 0.0 0.0 0.0]
    ref = HM_GH.rpo_retimed_reference(path, geom, cfg; safe_distance_m=1.0)
    @test ref.speed_mps[1] == 0.0
    @test ref.speed_mps[end] == 0.0
    @test ref.v_rtn[:, 1] == zeros(3)
    @test ref.v_rtn[:, end] == zeros(3)
    @test ref.r_rtn[:, 1] ≈ path[:, 1] atol = 1.0e-9
    @test ref.r_rtn[:, end] == path[:, end]
    @test ref.t_s[2] - ref.t_s[1] ≈ cfg.retime_dt_s
    @test maximum(abs.(ref.profile.a_seg)) <= a_max * (1.0 + 1.0e-9)
    @test all(ref.profile.v[2:(end - 1)] .<= ref.profile.v_point[2:(end - 1)] .+ 1.0e-12)
    # Speed changes between fixed steps stay within the limit.
    dv = diff(ref.speed_mps) ./ diff(ref.t_s)
    @test maximum(abs.(dv)) <= a_max * (1.0 + 1.0e-6)
    # The discrete reference itself: second differences of the positions stay
    # near the tangential limit (the path is almost straight here).
    dt = cfg.retime_dt_s
    acc = [norm(ref.r_rtn[:, k + 1] - 2.0 .* ref.r_rtn[:, k] + ref.r_rtn[:, k - 1]) / dt^2 for k in 2:(size(ref.r_rtn, 2) - 1)]
    @test maximum(acc) <= 1.05 * a_max
    # The velocity reference is the speed along the curve.
    @test maximum(abs.([norm(ref.v_rtn[:, k]) for k in eachindex(ref.speed_mps)] .- ref.speed_mps)) < 1.0e-12

    # A nonzero initial speed is honoured and the reference still ends at rest.
    moving = SpaceAGORA.rpo_pso_config(cfg; retime_initial_speed_mps=0.05)
    ref_moving = HM_GH.rpo_retimed_reference(path, geom, moving; safe_distance_m=1.0)
    @test ref_moving.speed_mps[1] ≈ 0.05
    @test ref_moving.speed_mps[end] == 0.0

    # Pointwise limits: near a station point the manuscript's d_avail = c - d_safe slows the reference.
    near = _hm_far_geometry((15.0, 0.0, 6.0); keepout=1.0)
    ref_near = HM_GH.rpo_retimed_reference(path, near, cfg; safe_distance_m=1.0)
    @test maximum(ref_near.speed_mps) <= maximum(ref.speed_mps)
    @test ref_near.t_s[end] > ref.t_s[end]
end

@testset "RPO HyPR manuscript exploration score and adaptive configuration" begin
    s = HM_GH.rpo_manuscript_exploration_score(200.0, 100.0, 100)
    @test s.detour_score == 1.0
    @test s.search_effort_score ≈ 1.0 - exp(-1.0)
    @test s.eta ≈ (2.0 + (1.0 - exp(-1.0))) / 3.0
    @test HM_GH.rpo_manuscript_exploration_score(100.0, 100.0, 0).eta == 0.0
    @test HM_GH.rpo_manuscript_exploration_score(500.0, 100.0, 0).detour_score == 1.0

    base = SpaceAGORA.rpo_pso_config(SpaceAGORA.RPOPSOConfig(
        hypr_mode=:manuscript,
        adaptive_n_waypoints_min=3, adaptive_n_waypoints_max=8,
        adaptive_n_particles_min=60, adaptive_n_particles_max=160,
        adaptive_n_iters_min=10, adaptive_n_iters_max=60,
        adaptive_w_inertia_min=0.4, adaptive_w_inertia_max=0.75,
        adaptive_c1_min=1.2, adaptive_c1_max=1.8,
        adaptive_c2_min=1.2, adaptive_c2_max=2.2,
    ))
    start = SVector(0.0, 0.0, 0.0)
    goal = SVector(0.0, 100.0, 0.0)
    warm(length_m, iterations) = (path_found=true, path_length_m=length_m, iterations=iterations)
    easy, easy_diag = HM_GH.rpo_manuscript_adaptive_pso_config(base, start, goal, warm(100.0, 0))
    hard, hard_diag = HM_GH.rpo_manuscript_adaptive_pso_config(base, start, goal, warm(250.0, 10_000))
    @test easy_diag.eta == 0.0
    @test hard_diag.eta ≈ 1.0
    @test (easy.w_inertia, easy.c1, easy.c2) == (0.4, 1.8, 1.2)
    @test hard.w_inertia ≈ 0.75
    @test hard.c1 ≈ 1.2          # cognitive attraction falls with eta
    @test hard.c2 ≈ 2.2          # social attraction rises with eta
    @test hard_diag.coefficient_direction === :c1_down_c2_up
    @test (easy.n_waypoints, easy.n_particles, easy.n_iters) == (3, 60, 10)
    @test (hard.n_waypoints, hard.n_particles, hard.n_iters) == (8, 160, 60)
    mid, mid_diag = HM_GH.rpo_manuscript_adaptive_pso_config(base, start, goal, warm(125.0, 0))
    @test mid_diag.detour_score ≈ 0.25
    @test mid_diag.eta ≈ 0.5 / 3.0
    @test mid.n_particles == round(Int, (1 - mid_diag.eta) * 60 + mid_diag.eta * 160)
    # No warm-start path counts as the largest detour.
    _, failed = HM_GH.rpo_manuscript_adaptive_pso_config(base, start, goal, (path_found=false, path_length_m=0.0, iterations=50))
    @test failed.detour_score == 1.0
    # The fuel-proxy step defaults to the retiming step unless set.
    @test HM_GH.rpo_fuel_proxy_dt_s(SpaceAGORA.RPOPSOConfig(hypr_mode=:manuscript, mean_motion_radps=0.001, fuel_proxy_dt_s=0.5)) == 0.5
    @test HM_GH.rpo_fuel_proxy_dt_s(SpaceAGORA.RPOPSOConfig(retime_dt_s=0.2)) == 0.2
end

@testset "RPO HyPR manuscript search box and culling" begin
    # B is the station's bounding box widened on each axis (Sec. III.C).
    points = [-2.0 3.0 0.5; -10.0 4.0 1.0; 1.0 -1.0 6.0]
    geom = G.RPOReferenceGeometry(SpaceAGORA.RPOStationGeometry(points; keepout_radius_m=0.5))
    cfg = _hm_manuscript_cfg(station_box_margin_m=(1.0, 2.0, 3.0))
    lo, hi = HM_GH.rpo_pso_station_bounds(geom, cfg)
    @test lo == SVector(-3.0, -12.0, -4.0)
    @test hi == SVector(4.0, 6.0, 9.0)
    lo2, hi2 = HM_GH.rpo_pso_station_bounds(geom, cfg; margin_scale=2.0)
    @test lo2 == SVector(-4.0, -14.0, -7.0)
    @test hi2 == SVector(5.0, 8.0, 12.0)

    # Culling (Sec. III.D): the worst particles move to q* + d_bez, perturbed by
    # σ_resp times the index taper in `:manuscript` mode.
    start, goal = SVector(0.0, 0.0, 0.0), SVector(0.0, 30.0, 0.0)
    nw, np = 2, 4
    gbest = [5.0, 10.0, 0.0, 5.0, 20.0, 0.0]
    lo_rep = fill(-100.0, 3 * nw)
    hi_rep = fill(100.0, 3 * nw)
    function culled(cull_cfg)
        positions = repeat(gbest, 1, np) .+ reshape(collect(1.0:(3 * nw * np)), 3 * nw, np)
        velocities = zeros(3 * nw, np)
        pbest = copy(positions)
        pbest_cost = [1.0, 4.0, 2.0, 3.0]
        n = HM_GH.rpo_pso_cull_swarm!(positions, velocities, pbest, pbest_cost, lo_rep, hi_rep, gbest,
            start, goal, nw, cull_cfg, 10, MersenneTwister(5))
        return n, positions, pbest_cost
    end
    # Targets: q* + 0.6 (q_line - q*) + 0.4 (q_loc - q*), with q_loc the
    # projection of q* on the chord of its neighbours.
    targets = zeros(3 * nw)
    for (j, q_line) in ((1, SVector(0.0, 10.0, 0.0)), (2, SVector(0.0, 20.0, 0.0)))
        q_star = SVector{3}(gbest[(3j - 2):(3j)])
        prev = j == 1 ? start : SVector{3}(gbest[1:3])
        next = j == nw ? goal : SVector{3}(gbest[4:6])
        q_loc = HM_GH.rpo_pso_project_to_segment(q_star, prev, next)
        targets[(3j - 2):(3j)] .= q_star + 0.6 * (q_line - q_star) + 0.4 * (q_loc - q_star)
    end
    exact = _hm_manuscript_cfg(n_particles=np, cull_fraction_max=0.5, cull_start_iter=10, cull_noise_abs_m=0.0)
    n, positions, costs = culled(exact)
    @test n == 2
    @test positions[:, 2] ≈ targets atol = 1.0e-12     # the two worst (costs 4 and 3)
    @test positions[:, 4] ≈ targets atol = 1.0e-12
    @test positions[:, 1] != targets                    # the best are kept
    @test isinf(costs[2]) && isinf(costs[4])
    noisy = SpaceAGORA.rpo_pso_config(exact; cull_noise_abs_m=0.3)
    _, positions, _ = culled(noisy)
    @test 0.0 < maximum(abs.(positions[:, 2] .- targets)) < 6 * 0.3
    # Before the start iteration nothing is culled.
    @test culled(SpaceAGORA.rpo_pso_config(exact; cull_start_iter=20))[1] == 0
end

@testset "RPO HyPR manuscript planner" begin
    rng = MersenneTwister(3)
    points = zeros(3, 1500)
    for j in 1:size(points, 2)
        d = randn(rng, 3)
        points[:, j] .= 8.0 .* d ./ norm(d)
    end
    geom = G.RPOReferenceGeometry(SpaceAGORA.RPOStationGeometry(points; keepout_radius_m=1.0);
        chaser=SpaceAGORA.RPOCubeSatGeometry(dims_m=(0.1, 0.1, 0.3)))
    cfg = _hm_manuscript_cfg(
        w_obs=1.0e6,
        w_fuel=1.0,
        obstacle_sigmoid_tol_m=0.5,
        adaptive_sampling_max_ds_m=3.0,
        adaptive_sampling_far_clearance_m=10.0,
        rrt_warmstart_enable=true,
        rrt_warmstart_iters=1000,
        rrt_warmstart_step_size_m=2.0,
        rrt_warmstart_collision_sample_ds_m=0.5,
        rrt_warmstart_shortcut_iters=40,
        rrt_warmstart_box_margin_m=10.0,
        search_margin_m=25.0,
        station_box_margin_m=(20.0, 40.0, 20.0),
        adaptive_n_waypoints_min=3, adaptive_n_waypoints_max=5,
        adaptive_n_particles_min=8, adaptive_n_particles_max=16,
        adaptive_n_iters_min=3, adaptive_n_iters_max=6,
        schedule_transition_fraction=0.5,
        early_stopping_enable=false,
    )
    start = SVector(0.0, -40.0, 0.0)
    goal = SVector(0.0, 40.0, 0.0)
    plan = G.rpo_pso_plan_path(start, goal, geom, cfg; safe_distance_m=1.0, rng=MersenneTwister(741))
    @test plan.warmstart.attempted
    @test plan.warmstart.path_found
    @test plan.warmstart.iterations > 0          # the straight segment is blocked
    @test plan.warmstart.path_length_m >= norm(goal - start)
    @test plan.adaptive.mode === :manuscript
    @test 0.0 <= plan.adaptive.eta <= 1.0
    @test plan.config.n_waypoints == plan.adaptive.n_waypoints
    @test plan.config.n_particles == plan.adaptive.n_particles
    @test length(plan.cost_history) == plan.config.n_iters
    @test issorted(plan.cost_history; rev=true)
    @test plan.components.violation_count == 0
    @test plan.cost ≈ cfg.w_obs * plan.components.J_obs + cfg.w_fuel * plan.components.J_fuel rtol = 1.0e-12
    @test plan.path[:, 1] == collect(start)
    @test plan.path[:, end] == collect(goal)
    # The swarm searched the station's box with the prescribed margins.
    box_lo, box_hi = HM_GH.rpo_pso_station_bounds(geom, plan.config)
    @test plan.initial_bounds.lo == collect(box_lo)
    @test plan.initial_bounds.hi == collect(box_hi)
    again = G.rpo_pso_plan_path(start, goal, geom, cfg; safe_distance_m=1.0, rng=MersenneTwister(741))
    @test again.path == plan.path
    @test again.cost == plan.cost
end
end

# From SpaceAGORA's test/unit/rpo_port_tests.jl, "RPO geometry and PSO basics".
module PortHelpers
using Test, LinearAlgebra, Random, StaticArrays, SpaceAGORA, HYPR
const R = Base.get_extension(HYPR, :HYPRSpaceAGORAExt).RPO
const G = SpaceAGORA.HYPRServices.RPO

@testset "RPO geometry and PSO basics: HYPR helpers" begin
    retime_geom = G.RPOReferenceGeometry(
        SpaceAGORA.RPOStationGeometry(reshape([0.0, 0.0, 0.0], 3, 1); keepout_radius_m=0.0);
        chaser=SpaceAGORA.RPOCubeSatGeometry(dims_m=(0.02, 0.02, 0.02)),
    )
    retime_cfg = SpaceAGORA.RPOPSOConfig(
        sample_ds_m=0.05,
        curve_type=:polyline,
        retime_dt_s=1.0,
        retime_reaction_time_s=0.0,
        retime_a_max_mps2=0.5,
        retime_speed_scale=1.0,
        retime_min_speed_mps=0.0,
        retime_max_speed_mps=Inf,
        retime_max_steps=100,
        adaptive_enable=false,
    )
    retime_path = [
        0.21 0.31;
        0.0  0.0;
        0.0  0.0
    ]
    _, _, v_zero_safe = R.rpo_retime_path(retime_path, retime_geom, retime_cfg; safe_distance_m=0.0)
    _, _, v_large_safe = R.rpo_retime_path(retime_path, retime_geom, retime_cfg; safe_distance_m=0.19)
    @test v_large_safe[1] ≈ v_zero_safe[1]
    @test v_zero_safe[1] ≈ sqrt(2.0 * retime_cfg.retime_a_max_mps2 * 0.21)

    # The configuration SpaceAGORA's test assembles from its settings groups, given
    # here by the same values through the public keyword constructor.
    modular_cfg = SpaceAGORA.rpo_pso_config(SpaceAGORA.RPOPSOConfig(
        n_waypoints=2, n_particles=9, n_iters=3, sample_ds_m=0.5, curve_type=:bezier,
        w_len=1.2, w_obs=1.0e5, cost_ref_distance_m=10.0, tf_s=20.0,
        adaptive_enable=false, adaptive_n_particles_min=6, adaptive_n_particles_max=12,
        adaptive_n_iters_min=2, adaptive_n_iters_max=5,
        cull_enable=true, cull_fraction_max=0.25, cull_start_iter=1,
        schedule_enable=true,
        early_stopping_enable=true, early_stopping_patience=1, early_stopping_min_iters=1,
        early_stopping_min_abs_improvement=1.0e99, early_stopping_require_feasible=false,
        rrt_warmstart_enable=true, rrt_warmstart_iters=4,
        refinement_enable=false,
        retime_a_max_mps2=0.1,
    ))
    warmstart_bounds_cfg = SpaceAGORA.rpo_pso_config(modular_cfg; pso_rrt_warmstart_box_margin=0.25)
    warmstart_lo, warmstart_hi = R.rpo_pso_warmstart_bounds(
        [
            -1.0 0.0 2.0;
             0.0 1.0 0.5;
             0.0 0.0 0.0
        ],
        warmstart_bounds_cfg,
    )
    @test warmstart_lo ≈ SVector{3, Float64}(-1.25, -0.25, -0.25)
    @test warmstart_hi ≈ SVector{3, Float64}(2.25, 1.25, 0.25)

    # SpaceAGORA's copy of this check needed its demo station point cloud, which no
    # checkout carries, so it never ran there. With adaptation disabled the
    # geometry is not consulted; a one-point station stands in for it.
    geom = G.RPOReferenceGeometry(
        SpaceAGORA.RPOStationGeometry(reshape([0.0, 10.0, 0.0], 3, 1); keepout_radius_m=0.25);
        chaser=SpaceAGORA.RPOCubeSatGeometry(dims_m=(0.2, 0.2, 0.3)),
    )
    fixed_cfg, adaptive_stats = R.rpo_adaptive_pso_config(
        modular_cfg,
        SVector{3, Float64}(-6.0, 0.0, 0.0),
        SVector{3, Float64}(6.0, 0.0, 0.0),
        geom;
        safe_distance_m=0.1,
    )
    @test fixed_cfg.n_particles == modular_cfg.n_particles
    @test adaptive_stats.enabled === false

    cull_cfg = SpaceAGORA.RPOPSOConfig(
        n_waypoints=3,
        n_particles=4,
        n_iters=1,
        cull_enable=true,
        cull_fraction_max=0.25,
        cull_start_iter=1,
        cull_noise_scale=0.0,
        cull_arc_velocity_scale=0.0,
        adaptive_enable=false,
    )
    start = SVector{3, Float64}(0.0, 0.0, 0.0)
    goal = SVector{3, Float64}(10.0, 0.0, 0.0)
    gbest_path = [
        0.0 2.5 5.0 7.5 10.0;
        0.0 2.0 3.0 2.0  0.0;
        0.0 0.0 0.0 0.0  0.0
    ]
    gbest = [gbest_path[:, j] for j in 2:4]
    gbest = reduce(vcat, gbest)
    dim = length(gbest)
    positions = repeat(gbest, 1, 4)
    velocities = fill(1.0, dim, 4)
    pbest = copy(positions)
    pbest_cost = [1.0, 2.0, 3.0, 4.0]
    lo_rep = fill(-20.0, dim)
    hi_rep = fill(20.0, dim)
    replaced = R.rpo_pso_cull_swarm!(
        positions,
        velocities,
        pbest,
        pbest_cost,
        lo_rep,
        hi_rep,
        gbest,
        start,
        goal,
        3,
        cull_cfg,
        1,
        MersenneTwister(3),
    )
    @test replaced == 1
    expected_cull = zeros(dim)
    for j in 1:3
        q_star = SVector{3, Float64}(gbest_path[:, j + 1])
        q_line = (1.0 - j / 4) * start + (j / 4) * goal
        q_loc = R.rpo_pso_project_to_segment(q_star, gbest_path[:, j], gbest_path[:, j + 2])
        expected_cull[(3 * (j - 1) + 1):(3 * j)] .= q_star + 0.6 * (q_line - q_star) + 0.4 * (q_loc - q_star)
    end
    @test positions[:, 4] ≈ expected_cull
    @test all(velocities[:, 4] .== 0.0)
end
end

# From SpaceAGORA's test/unit/gnc/shared_sampling_tests.jl.
module SamplingHelpers
using Test, SpaceAGORA, HYPR
const R = Base.get_extension(HYPR, :HYPRSpaceAGORAExt).RPO
const G = SpaceAGORA.HYPRServices.RPO

@testset "Configured sampling settings come from the configuration" begin
    for enabled in (false,true), curve in (:bezier,:polyline)
        cfg = G.RPOPSOConfig(adaptive_sampling_enable=enabled, curve_type=curve, sample_ds_m=0.17, safe_distance_m=0.23, adaptive_sampling_max_ds_m=0.41, adaptive_sampling_far_clearance_m=1.2, adaptive_sampling_power=0.7, adaptive_sampling_safe_distance_fraction=0.4, adaptive_sampling_obstacle_guard_fraction=0.6)
        settings = G.RPOAdaptiveSamplingSettings(enabled=enabled, max_ds_m=0.41, far_clearance_m=1.2, power=0.7, safe_distance_fraction=0.4, obstacle_guard_fraction=0.6)
        @test (@inferred R._rpo_sampling_settings(cfg)) === settings
    end
end
end
