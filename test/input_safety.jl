using Test, Random, HYPR
@testset "Standalone safety before unchecked arithmetic" begin
    fresh() = SwarmState([.1 .2; .3 .4], zeros(2,2))
    policy(; evaluate=(x;cost_cutoff=Inf)->(total=sum(abs2,x),), feasible=x->x!==nothing) =
        SearchPolicy(evaluate=evaluate, weights=i->(w_inertia=.5,c1=1.,c2=1.),
            cull! = (s,i,r)->0, feasible=feasible, materially_better=(a,b)->a<b)
    run(s, cfg=SearchSettings(iterations=2), p=policy(); rng=MersenneTwister(17)) =
        search!(s, [-1.,-1.], [1.,1.], cfg, p; rng)
    # A partial first iteration must not make a reusable state.
    state=fresh();calls=Ref(0)
    faulty=policy(evaluate=(x;cost_cutoff=Inf)->begin
        calls[]+=1;calls[]==2 && error("objective failed");(total=sum(abs2,x),)
    end)
    @test_throws ErrorException run(state,SearchSettings(iterations=2),faulty)
    @test isempty(state.cost_history)
    @test state.gbest_cost < Inf
    @test state.started
    @test_throws ArgumentError run(state)
    @test calls[] == 2
    for cfg in (SearchSettings(0,false,1,1,.2),SearchSettings(1,false,-1,1,.2),
                SearchSettings(1,false,1,-1,.2),SearchSettings(1,false,1,1,NaN),
                SearchSettings(1,false,1,1,-.2),SearchSettings(1,false,1,1,Inf))
        state=fresh();rng=MersenneTwister(19);before=copy(rng);positions=copy(state.positions)
        @test_throws ArgumentError run(state,cfg;rng)
        @test !state.started
        @test state.positions == positions
        @test rand(rng,UInt,8) == rand(before,UInt,8)
    end
    for (field,value) in ((:velocities,zeros(1,2)),(:pbest,zeros(2,1)),
                          (:gbest,zeros(1)),(:pbest_cost,zeros(1)),(:pbest_components,Any[nothing]))
        state=fresh();setproperty!(state,field,value)
        @test_throws DimensionMismatch run(state)
        @test !state.started
    end
    for field in (:velocities,:pbest)
        state=fresh();setproperty!(state,field,state.positions)
        @test_throws ArgumentError run(state)
    end
    for value in (2.,Inf,NaN)
        state=fresh();state.positions[1]=value
        @test_throws ArgumentError run(state)
        @test !state.started
    end
    state=fresh();state.velocities[1]=NaN
    @test_throws ArgumentError run(state)
    no_best=fresh();saw_nothing=Ref(false)
    run(no_best, SearchSettings(iterations=2,early_stopping=true),
        policy(evaluate=(x;cost_cutoff=Inf)->(total=Inf,),feasible=x->(saw_nothing[]|=x===nothing;false)))
    @test saw_nothing[]
    @test no_best.gbest_components === nothing
    @test_throws ArgumentError run(no_best)
end
@testset "Particle update validates streams and buffers without consuming randomness" begin
    function buffers()
        p=[.1 .2; .3 .4];(p,zeros(2,2),copy(p),[0.,0.],[-1.,-1.],[1.,1.],
            (w_inertia=.5,c1=1.,c2=1.),[MersenneTwister(1),MersenneTwister(2)])
    end
    a=buffers();shared=MersenneTwister(9);witness=copy(shared);before=copy(a[1])
    @test_throws ArgumentError advance_particles!(a[1:7]...,[shared,shared])
    @test a[1]==before
    @test rand(shared,UInt,8)==rand(witness,UInt,8)
    a=buffers();@test_throws DimensionMismatch advance_particles!(a[1:7]...,[a[8][1]])
    a=buffers();@test_throws DimensionMismatch advance_particles!(a[1],zeros(1,2),a[3:end]...)
    a=buffers();@test_throws ArgumentError advance_particles!(a[1],a[1],a[3:end]...)
    a=buffers();@test_throws ArgumentError advance_particles!(a[1:6]...,(w_inertia=NaN,c1=1.,c2=1.),a[8])
    a=buffers();@test_throws ArgumentError advance_particles!(a[1:7]...,[1,2])
    # Separate objects seeded alike are allowed: ownership, not seed uniqueness.
    a=buffers();@test advance_particles!(a[1:7]...,[MersenneTwister(9),MersenneTwister(9)])===nothing
end
