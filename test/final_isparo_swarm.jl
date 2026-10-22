# Run in test/integration_env after developing this HYPR and compatible SpaceAGORA.
using Test, Random, StaticArrays, SpaceAGORA, HYPR
const G=SpaceAGORA.HYPRServices.RPO
const RPOPSOConfig=SpaceAGORA.RPOPSOConfig
function paper_config()
    cfg = RPOPSOConfig(hypr_mode=:manuscript, n_waypoints=5,n_particles=100,n_iters=60,
        mass_kg=5.,isp_s=60.,w_obs=1e6,w_fuel=1.,safe_distance_m=.5,
        obstacle_sigmoid_k=1e5,obstacle_sigmoid_tol_m=.001,
        adaptive_allow_downscale=true,adaptive_n_particles_min=60,adaptive_n_particles_max=160,
        adaptive_n_iters_min=10,adaptive_n_iters_max=60,
        adaptive_n_waypoints_min=3,adaptive_n_waypoints_max=8,
        retime_dt_s=.1,fuel_proxy_dt_s=.1,retime_reaction_time_s=.25,
        retime_a_max_mps2=.00625,retime_speed_scale=.5,retime_max_speed_mps=.25,
        retime_accel_limit_enable=false,rrt_warmstart_enable=true,
        rrt_warmstart_iters=1000,rrt_warmstart_step_size_m=.75,
        rrt_warmstart_shortcut_iters=80,cull_start_iter=10,cull_fraction_max=.25,
        stagnation_learning_threshold=10,schedule_transition_fraction=.5,
        refinement_start_iter=60,refinement_period=10,refinement_sample_ds_m=.025,
        early_stopping_enable=false,iteration_runtime_limit_s=Inf)
    earth=SpaceAGORA.make_no_gram_planet(:earth)
    n=sqrt(earth.μ/(earth.Rp_e+420000.0)^3)
    return SpaceAGORA.rpo_pso_config(cfg;mean_motion_radps=n, reexplore_max_waypoints=8)
end

cfg=paper_config()
@testset "Final paper swarm behavior" begin
    for eta in (0.,.5,1.)
        cc=SpaceAGORA.rpo_pso_config(cfg;w_inertia=(1-eta)*.4+eta*.75,c1=(1-eta)*1.8+eta*1.2,c2=(1-eta)*1.2+eta*2.2)
        @test collect(values(G.rpo_pso_iteration_weights(cc,cc.n_iters))) ≈ [.455,1.05,1.75]
        @test values(G.rpo_pso_iteration_weights(cc,1)) == (cc.w_inertia,cc.c1,cc.c2)
    end
    cc=SpaceAGORA.rpo_pso_config(cfg;n_particles=4,n_waypoints=1,adaptive_enable=false)
    for i in (9,10,11,19,20,21)
        p=fill(1.,3,4);v=zeros(3,4);best=copy(p);cost=[1.,2.,3.,4.]
        n=G.rpo_pso_cull_swarm!(p,v,best,cost,fill(-10.,3),fill(10.,3),zeros(3),SVector(-1.,0.,0.),SVector(1.,0.,0.),1,cc,i,MersenneTwister(7))
        @test n == (i%10==0 ? 1 : 0)
    end
end
