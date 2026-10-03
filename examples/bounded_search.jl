using HYPR, Random

"""A dimensionless demonstration, not a spacecraft or robot-arm acceptance case."""
function bounded_search(seed=741)
    rng=MersenneTwister(seed)
    lower=fill(-2.,2); upper=fill(2.,2)
    positions=4 .* rand(rng,2,12) .- 2
    velocities=zeros(2,12)
    state=SwarmState(positions,velocities)
    policy=SearchPolicy(
        evaluate=(x; cost_cutoff=Inf)->(total=sum(abs2,x),),
        weights=iter->(w_inertia=.5,c1=1.2,c2=1.2),
        cull! = (state,iter,rng)->0,
        feasible=comps->comps!==nothing,
        materially_better=(new,reference)->hypr_material_improvement(new,reference,1e-12,1e-9))
    settings=SearchSettings(iterations=40,max_velocity_fraction=.2)
    search!(state,lower,upper,settings,policy;rng)
    return state
end
if abspath(PROGRAM_FILE)==@__FILE__
    result=bounded_search()
    println("best_position=",result.gbest," cost=",result.gbest_cost)
end
