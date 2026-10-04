# This file runs in its own process. The more-specific test method observes the
# real configured wrapper's callback, then delegates to the unchanged engine.
# Configured RRT* supplies refinement, so ordinary runs do not call evaluate_cost.
using HYPR, SpaceAGORA, Test, StaticArrays, Random
const S = SpaceAGORA.SimulationModel
const G = S.GuidanceHooks
const OBSERVED = Any[]
const PROBE_PATH = [-1.5 0.0 1.5; 0.0 0.25 0.0; 0.0 0.0 0.0]

function G.rpo_rrt_star_plan_path(start::SVector{3,Float64}, goal::SVector{3,Float64},
    geometry::S.RPOReferenceGeometry; evaluate_cost, kwargs...)
    push!(OBSERVED, evaluate_cost(PROBE_PATH))
    return invoke(G.rpo_rrt_star_plan_path, Tuple{Any,Any,Any},
        start, goal, geometry; evaluate_cost=evaluate_cost, kwargs...)
end

@testset "Configured RRT* fallback objective contract" begin
    start = SVector(-1.5, 0.0, 0.0)
    goal = SVector(1.5, 0.0, 0.0)
    for distance in (1.0, 10.0), safe_distance in (0.0, 0.2)
        geometry = S.RPOReferenceGeometry(
            S.RPOStationGeometry(reshape([0.,distance,0.],3,1); keepout_radius_m=0.1);
            chaser=S.RPOCubeSatGeometry(dims_m=(0.02,0.02,0.02)))
        cfg = S.RPOPSOConfig(curve_type=:bezier, w_obs=137.0, w_len=2.5,
            w_fuel=0.25, sample_ds_m=0.09, refinement_enable=false)
        polyline = G.rpo_pso_config(cfg; curve_type=:polyline)
        expected = G.rpo_path_cost(PROBE_PATH, geometry, polyline;
            safe_distance_m=safe_distance)
        previous = length(OBSERVED)
        plan = G.rpo_rrt_star_plan_path(start, goal, geometry, cfg;
            safe_distance_m=safe_distance, rng=MersenneTwister(172))
        @test length(OBSERVED) == previous + 1
        @test isfinite(expected)
        @test isequal(last(OBSERVED), expected)
        @test plan.path_found
        @test plan.iterations == 0
        @test plan.path == hcat(start, goal)
    end
end
