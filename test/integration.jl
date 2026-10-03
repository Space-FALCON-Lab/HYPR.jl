using Test, Random
order=isempty(ARGS) ? "core-first" : only(ARGS)
if order=="core-first"
    using SpaceAGORA
    @test !SpaceAGORA.hypr_available()
    using HYPR
elseif order=="hypr-first"
    using HYPR
    @test all(id.name != "SpaceAGORA" for id in keys(Base.loaded_modules))
    using SpaceAGORA
else
    error("Expected core-first or hypr-first")
end
@testset "Optional SpaceAGORA integration" begin
    @test Base.get_extension(HYPR,:HYPRSpaceAGORAExt)!==nothing
    @test SpaceAGORA.hypr_available()
    S=SpaceAGORA.SimulationModel; G=S.GuidanceHooks
    @test SpaceAGORA.RPOPSOConfig===G.RPOPSOConfig
    @test SpaceAGORA.RobotArmHYPRConfig===S.RobotArmPlanning.RobotArmHYPRConfig
    geo=S.RPOReferenceGeometry(S.RPOStationGeometry(reshape([0.,0.,50.],3,1);keepout_radius_m=.25);
        chaser=S.RPOCubeSatGeometry(dims_m=(.1,.1,.3)))
    cfg=SpaceAGORA.RPOPSOConfig(n_waypoints=1,n_particles=4,n_iters=2,adaptive_enable=false,
        rrt_warmstart_enable=false,sample_ds_m=.2,iteration_runtime_limit_s=Inf)
    a=G.rpo_pso_plan_path([1.,0.,0.],[3.,1.,0.],geo,cfg;rng=MersenneTwister(1))
    b=G.rpo_pso_plan_path([1.,0.,0.],[3.,1.,0.],geo,cfg;rng=MersenneTwister(1))
    @test a.path==b.path
    @test a.cost==b.cost
    @test isfinite(a.cost)
    model=S.default_cloth_arm_model();base=S.ClothArmBasePose([0.,0.,0.]);q0=[0.,.3,-.3]
    target=S.cloth_fk(model,base,[.6,-.4,.4]).end_effector_position
    arm=S.plan_robot_arm_motion_hypr(model,base,q0,target;
        planner_config=S.RobotArmPlannerConfig(dt_s=.1,duration_s=.5),
        hypr_config=S.RobotArmHYPRConfig(n_particles=4,n_iters=2,n_waypoints=1,n_samples=20),rng=MersenneTwister(2))
    @test isfinite(arm.cost)
    @test all(isfinite,arm.plan.q_ref)
end
