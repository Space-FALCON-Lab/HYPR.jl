module CustomObjectiveTests
using HYPR, SpaceAGORA, Test, Random
const S=SpaceAGORA.SimulationModel
const G=S.GuidanceHooks
const R=Base.get_extension(HYPR,:HYPRSpaceAGORAExt).RPO
const geo=S.RPOReferenceGeometry(S.RPOStationGeometry(reshape([0.,0.,50.],3,1);keepout_radius_m=.25);chaser=S.RPOCubeSatGeometry(dims_m=(.1,.1,.3)))
const start=[1.,0.,0.]; const goal=[3.,1.,0.]
const defaults=include("fixtures/rpo_defaults_0_1_0.jl")
compatible(a,b)=isequal(a,b)
compatible(a::AbstractFloat,b::AbstractFloat)=isequal(a,b) || isapprox(a,b;rtol=1e-12,atol=1e-12)
compatible(a::AbstractArray,b::AbstractArray)=size(a)==size(b) && all(compatible(x,y) for (x,y) in zip(a,b))
compatible(a::Tuple,b::Tuple)=length(a)==length(b) && all(compatible(x,y) for (x,y) in zip(a,b))
compatible(a::NamedTuple,b::NamedTuple)=keys(a)==keys(b) && compatible(values(a),values(b))
summary(a,rng)=(a.path,a.cost,a.components,a.cost_history,a.refinement_improved,rand(rng))
@testset "Unchanged 0.1.0 legacy/manuscript defaults" begin
 for mode in (:legacy,:manuscript), refine in (false,true), seed in (1,17)
  cfg=SpaceAGORA.RPOPSOConfig(hypr_mode=mode,n_waypoints=2,n_particles=4,n_iters=2,adaptive_enable=false,rrt_warmstart_enable=false,sample_ds_m=.2,iteration_runtime_limit_s=Inf,refinement_enable=refine,refinement_rounds=1)
  for kwargs in ((;),(objective_evaluator=nothing,))
   rng=MersenneTwister(seed)
   a=G.rpo_pso_plan_path(start,goal,geo,cfg;rng,kwargs...)
   actual=summary(a,rng); expected=defaults[(mode,refine,seed)]
   @test compatible(actual,expected)
   @test actual[end] == expected[end] # RNG consumption must be exact.
  end
 end
end
@testset "Custom RPO component contract" begin
 cfg=SpaceAGORA.RPOPSOConfig(adaptive_enable=false,rrt_warmstart_enable=false)
 points=hcat(start,goal)
 for missing in (:total,:J_obs,:violation_count)
  value=(total=1.,J_obs=0.,violation_count=0)
  bad=(p,g,c,s,k)->NamedTuple{Tuple(x for x in keys(value) if x != missing)}(Tuple(value[x] for x in keys(value) if x != missing))
  @test_throws ArgumentError R.rpo_path_objective_components(points,geo,cfg;objective_evaluator=bad)
 end
 callback=(p,g,c,s,k)->begin
  @test p === points && g === geo && c === cfg
  @test s === 0.5 && k === 2.0
  (total=3.,J_obs=0.,violation_count=0,caller_metadata=:kept)
 end
 @test R.rpo_path_objective_components(points,geo,cfg;safe_distance_m=1//2,cost_cutoff=2,objective_evaluator=callback).caller_metadata === :kept
 @test_throws ErrorException R.rpo_path_objective_components(points,geo,cfg;objective_evaluator=(args...)->error("caller failure"))
end
@testset "Custom objective governs search and final reporting" begin
 # Deliberately reverse the standard preference; a constant offset would not
 # detect accidentally using the original objective during particle ranking.
 objective=(p,g,c,s,k)->merge(G.rpo_normalized_path_cost_components(p,g,c;safe_distance_m=s),
  (total=-1.0-sum(abs2,p[:,2:end-1]),marker=:custom))
 for refine in (false,true)
  cfg=SpaceAGORA.RPOPSOConfig(n_waypoints=2,n_particles=8,n_iters=3,adaptive_enable=false,rrt_warmstart_enable=false,refinement_enable=refine,refinement_rounds=1,iteration_runtime_limit_s=Inf)
  original=G.rpo_pso_plan_path(start,goal,geo,cfg;rng=MersenneTwister(31))
  result=G.rpo_pso_plan_path(start,goal,geo,cfg;rng=MersenneTwister(31),objective_evaluator=objective)
  @test result.path != original.path
  @test all(<(0),result.cost_history)
  @test result.cost == result.components.total == objective(result.path,geo,result.config,0.,Inf).total
  @test result.components.marker === :custom
 end
 cfg=SpaceAGORA.RPOPSOConfig(n_waypoints=0,adaptive_enable=false,rrt_warmstart_enable=false)
 result=G.rpo_pso_plan_path(start,goal,geo,cfg;objective_evaluator=objective)
 @test result.cost == -1.0
 @test result.components.marker === :custom
 @test result.cost_history == [-1.0]
end
@testset "Every refinement operation honors the objective" begin
 cfg=SpaceAGORA.RPOPSOConfig(curve_type=:bezier,adaptive_enable=false,rrt_warmstart_enable=false,refinement_enable=true,refinement_rounds=1,sample_ds_m=.1)
 path=[1. 1.5 2.5 3.; 0. 2. 2. 0.; 0. 0. 0. 0.]
 bend=(p,g,c,s,k)->(total=sum(abs2,p[2,:]),J_obs=0.,violation_count=0)
 current=bend(path,geo,cfg,0.,Inf)
 for operation in (R.rpo_refine_shortcut_refit,R.rpo_refine_tighten_handles)
  p,c,changed=operation(path,geo,cfg,current;objective_evaluator=bend)
  @test changed
  @test c.total < current.total
  @test c == bend(p,geo,cfg,0.,Inf)
 end
 degree=(p,g,c,s,k)->(total=Float64(size(p,2)),J_obs=0.,violation_count=0)
 p,c,changed=R.rpo_refine_lower_degree(path,geo,cfg,degree(path,geo,cfg,0.,Inf);objective_evaluator=degree)
 @test changed && size(p,2)<size(path,2)
 @test c == degree(p,geo,cfg,0.,Inf)
 # A lower custom total cannot override the existing obstacle-priority rule.
 unsafe=(args...)->(total=-10.,J_obs=1.,violation_count=1)
 p,c,changed=R.rpo_try_accept_refinement(path,geo,cfg,current;objective_evaluator=unsafe)
 @test !changed && p === nothing && c === current
 for enabled in (false,true)
  cf=G.rpo_pso_config(cfg;refinement_enable=enabled)
  p,c,changed=R.rpo_post_refine_path(path,geo,cf;objective_evaluator=bend)
  @test c == bend(p,geo,cf,0.,Inf).total
  @test changed == enabled
 end
end
end
