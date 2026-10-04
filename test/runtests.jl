using HYPR, Test, Random

@testset "Core installation" begin
    @test Base.find_package("SpaceAGORA")===nothing
    @test Base.get_extension(HYPR,:HYPRSpaceAGORAExt)===nothing
    @test all(id.name != "SpaceAGORA" for id in keys(Base.loaded_modules))
end
include(joinpath(@__DIR__,"..","examples","bounded_search.jl"))
@testset "Standalone optimization and reproducibility" begin
    a=bounded_search(); b=bounded_search()
    @test a.gbest_cost < 1e-8
    @test a.gbest==b.gbest
    @test a.cost_history==b.cost_history
    @test length(a.cost_history)==40
    @test all(diff(a.cost_history).<=0)
    @test all(-2 .<= a.positions .<= 2)
    @test a.gbest_cost==sum(abs2,a.gbest)
end
@testset "Search schedule and failure boundaries" begin
    makepolicy(;evaluate=(x;cost_cutoff=Inf)->(total=sum(abs2,x),),feasible=x->x!==nothing,cull! = (s,i,r)->0)=
        SearchPolicy(evaluate=evaluate,weights=i->(w_inertia=1.,c1=0.,c2=0.),
            cull! = cull!,feasible=feasible,materially_better=(a,b)->hypr_material_improvement(a,b,0.,0.))
    state=SwarmState(reshape([1.],1,1),reshape([.5],1,1))
    search!(state,[-2.],[2.],SearchSettings(iterations=1,max_velocity_fraction=1.),makepolicy();rng=MersenneTwister(1))
    @test state.gbest==[1.]
    @test state.positions==reshape([1.5],1,1) # final movement is not the evaluated best
    @test state.cost_history==[1.]
    @test_throws ArgumentError search!(state,[-2.],[2.],SearchSettings(iterations=1),makepolicy();rng=MersenneTwister(1))
    @test_throws DimensionMismatch SwarmState(zeros(2,3),zeros(2,4))
    @test_throws ArgumentError SwarmState(zeros(0,3),zeros(0,3))
    a=zeros(2,3);@test_throws ArgumentError SwarmState(a,a)
    @test_throws ArgumentError SearchSettings(iterations=0)
    @test_throws ArgumentError SearchSettings(iterations=1,max_velocity_fraction=NaN)
    fresh=()->SwarmState(ones(1,2),zeros(1,2))
    @test_throws DimensionMismatch search!(fresh(),[-1.,-1.],[1.],SearchSettings(iterations=1),makepolicy();rng=MersenneTwister(1))
    @test_throws ArgumentError search!(fresh(),[2.],[1.],SearchSettings(iterations=1),makepolicy();rng=MersenneTwister(1))
    # No finite objective is not silently turned into a successful best.
    bad=fresh();search!(bad,[-2.],[2.],SearchSettings(iterations=2),makepolicy(evaluate=(x;cost_cutoff=Inf)->(total=Inf,));rng=MersenneTwister(1))
    @test bad.gbest_components===nothing
    @test bad.gbest_cost==Inf
    # Stopping precedes culling and random movement.
    calls=Int[];rng=MersenneTwister(71);original=copy(rng)
    stop=fresh();search!(stop,[-2.],[2.],SearchSettings(iterations=8,early_stopping=true,min_iterations=1,patience=0),makepolicy(cull! = (s,i,r)->(push!(calls,i);0));rng)
    @test stop.early_stop_iter==1
    @test isempty(calls)
    @test rand(rng,UInt,4)==rand(original,UInt,4)
    @test stop.positions==ones(1,2)
    # An objective exception is not swallowed as an optimization result.
    @test_throws DomainError search!(fresh(),[-2.],[2.],SearchSettings(iterations=1),makepolicy(evaluate=(x;cost_cutoff=Inf)->throw(DomainError(x)));rng=MersenneTwister(1))
end
@testset "RPO movement consumes particle-owned streams" begin
    p=[1. 2.; 3. 4.];v=fill(.2,2,2); pb=copy(p);gb=[0.,0.];rngs=[MersenneTwister(1),MersenneTwister(2)]; witness=copy.(rngs)
    advance_particles!(p,v,pb,gb,[-5.,-5.],[5.,5.],(w_inertia=.5,c1=0.,c2=0.),rngs)
    @test p==[1.1 2.1;3.1 4.1]
    @test v==fill(.1,2,2)
    for i in 1:2
        rand(witness[i],4)
        @test rand(rngs[i],UInt,4)==rand(witness[i],UInt,4)
    end
end
@testset "Preserved shared swarm policy" begin
    @test hypr_iteration_weights(false,5,1,.6,1.,2.,.5,.1,.5,.5,.5,.1,3.)==(w_inertia=.6,c1=1.,c2=2.)
    @test hypr_material_improvement(1.,Inf,1e-9,1e-9)
    @test !hypr_material_improvement(NaN,1.,0.,0.)
    @test !hypr_material_improvement(1.,1.,0.,0.)
    @test hypr_protected_particle_mask([4.,1.,3.,2.],.25)==[false,true,false,false]
end

include("source_boundaries.jl")

include("input_safety.jl")

include("arithmetic_regressions.jl")

include("coverage_inventory.jl")
