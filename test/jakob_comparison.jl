# Exercises unmodified source from the pinned August 26 comparison consumer.
using SpaceAGORA, HYPR, Test, SHA
const source=abspath(only(ARGS))
const files=sort!([joinpath(d,f) for (d,_,fs) in walkdir(source) for f in fs])
const inventory=join([replace(relpath(f,source),'\\'=>'/') * "\0" * bytes2hex(sha256(read(f))) * "\n" for f in files])
@test bytes2hex(sha256(inventory)) == "3521bb67d4bfa614b85f7aa47f0e9a938033c48d27625c3c512269a9f03dcb3e"
include(joinpath(source,"SpaceAGORA_RL.jl"))
const RL=SpaceAGORA_RL; const S=SpaceAGORA.SimulationModel
const geo=S.RPOReferenceGeometry(S.RPOStationGeometry(reshape([0.,0.,50.],3,1);keepout_radius_m=.25);chaser=S.RPOCubeSatGeometry(dims_m=(.1,.1,.3)))
@testset "Jakob August 26 paired PSO comparison" begin
 for waypoints in (0,1)
  cfg=SpaceAGORA.RPOPSOConfig(n_waypoints=waypoints,n_particles=4,n_iters=2,adaptive_enable=false,rrt_warmstart_enable=false,sample_ds_m=.2,iteration_runtime_limit_s=Inf,refinement_enable=true,refinement_rounds=1,retime_accel_limit_enable=true,retime_a_max_mps2=0.00625)
  scenario=RL.RPOHyPRRLScenario(start_rtn=[1.,0.,0.],goal_rtn=[3.,1.,0.],geometry=geo,pso_config=cfg)
  config=RL.RPOHyPRRLConfig(safe_distance_m=0.1)
  result=RL.evaluate_hypr_pso_comparison_case(config,scenario,17)
  for (name,planner) in ((:original,:hypr_pso_original_proxy),(:retimed_fuel,:hypr_pso_retimed_fuel))
   value=getproperty(result,name)
   @test !hasproperty(value,:error)
   @test value.plan !== nothing
   value.plan === nothing && continue
   plan=value.plan
   @test plan.valid
   @test isfinite(plan.cost) && isfinite(plan.propellant_used_kg)
   @test plan.diagnostics.planner === planner
   @test plan.diagnostics.edit_feasible
   @test plan.diagnostics.evaluator.evaluator_mode === :full_lqmpc
   @test plan.diagnostics.final_position_error_m <= S.GuidanceHooks.RPOLQMPCTrackingSettings().final_position_tol_m
   @test plan.path_rtn[:,1] == scenario.start_rtn && plan.path_rtn[:,end] == scenario.goal_rtn
   println((waypoints=waypoints,route=name,valid=plan.valid,objective=plan.cost,search_objective=plan.diagnostics.edit_cost,final_position_error_m=plan.diagnostics.final_position_error_m))
  end
  # The callback's search objective is separate from terminal coupled tracking.
  plan=result.retimed_fuel.plan
  if plan !== nothing
   evaluator=RL.RPOHyPRRLPSOObjectiveEvaluator(config,scenario)
   _,search_cfg,_,_=RL._rpo_spaceagora_settings(scenario,config)
   components=evaluator(plan.path_rtn,geo,search_cfg,config.safe_distance_m,Inf)
   @test components.retimed_feasible
   @test plan.diagnostics.edit_cost ≈ components.total rtol=1e-12
  end
 end
end
