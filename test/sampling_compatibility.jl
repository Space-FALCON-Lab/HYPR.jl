using HYPR, SpaceAGORA, Test
@testset "Configured sampling forwards to shared kernels" begin
    S=SpaceAGORA.SimulationModel;G=S.GuidanceHooks
    points=[0. 0.5 1.;0. 0.4 0.;0. 0. 0.]
    for adaptive in (false,true), distance in (2.,50.)
        geometry=S.RPOReferenceGeometry(S.RPOStationGeometry(reshape([0.,distance,0.],3,1);keepout_radius_m=.1);
            chaser=S.RPOCubeSatGeometry(dims_m=(.02,.02,.02)))
        cfg=S.RPOPSOConfig(sample_ds_m=.09,adaptive_sampling_enable=adaptive)
        settings=G.RPOAdaptiveSamplingSettings(cfg.adaptive_sampling_enable,
            cfg.adaptive_sampling_max_ds_m,cfg.adaptive_sampling_far_clearance_m,
            cfg.adaptive_sampling_power,cfg.adaptive_sampling_safe_distance_fraction,
            cfg.adaptive_sampling_obstacle_guard_fraction)
        # The shared sampler intentionally returns NaN at the terminal clearance.
        # isequal preserves that sentinel comparison and distinguishes signed zeros.
        for f in (G.rpo_sample_path_polyline_adaptive,G.rpo_sample_path_bezier_adaptive,
                  G.rpo_sample_path_bezier_adaptive_with_params)
            @test isequal(f(points,geometry,cfg),f(points,geometry,settings;base_ds_m=.09))
            @test isequal(f(points,geometry,cfg;safe_distance_m=.2,base_ds_m=.07),
                f(points,geometry,settings;safe_distance_m=.2,base_ds_m=.07))
        end
    end
end
