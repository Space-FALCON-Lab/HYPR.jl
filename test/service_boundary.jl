# Run in a fresh process in the integration environment:
#   julia --project=<environment> test/service_boundary.jl core-first|hypr-first
# Core-first first checks that SpaceAGORA refuses HYPR planning before HYPR loads.
# Both orders then check that the extension reaches SpaceAGORA only through names
# the HYPRServices contract lists: a contract change that drops a name HYPR uses,
# or a new use of a name the contract does not list, fails here, in this
# repository's CI and in SpaceAGORA's, which runs it against the core it changes.
# The check covers the extension's source in ext/, including local aliases and
# function bodies (test/core_access.jl); the tests' private fixtures are outside it.
using Test, Random
include(joinpath(@__DIR__, "core_access.jl"))
using .CoreAccess
length(ARGS) == 1 && only(ARGS) in ("core-first", "hypr-first") || error("Pass core-first or hypr-first")
const ORDER = only(ARGS)

if ORDER == "core-first"
    using SpaceAGORA
    @testset "SpaceAGORA refuses HYPR planning without HYPR" begin
        @test !SpaceAGORA.hypr_available()
        @test all(id.name != "HYPR" for id in keys(Base.loaded_modules))
        planner = SpaceAGORA.HYPRRPOPlanner(SpaceAGORA.RPOPSOConfig())
        @test_throws SpaceAGORA.HYPRUnavailableError SpaceAGORA.make_rpo_configuration(planner=planner)
        R = SpaceAGORA.HYPRServices.RPO
        for plan in (R.rpo_pso_plan_path, R.rpo_rrt_connect_plan_path, R.rpo_rrt_star_plan_path)
            @test_throws SpaceAGORA.HYPRUnavailableError plan(zeros(3), ones(3), nothing, SpaceAGORA.RPOPSOConfig();
                rng=MersenneTwister(1))
        end
        @test !SpaceAGORA.hypr_available()
    end
    using HYPR
else
    using HYPR
    @test all(id.name != "SpaceAGORA" for id in keys(Base.loaded_modules))
    using SpaceAGORA
end

@testset "HYPR uses only names the HYPRServices contract lists ($ORDER)" begin
    H = SpaceAGORA.HYPRServices
    extension = Base.get_extension(HYPR, :HYPRSpaceAGORAExt)
    @test extension !== nothing && extension.initialized()
    @test SpaceAGORA.hypr_available()
    # The facade's shape and its exact version refusal, as SpaceAGORA's conformance check defines them.
    SpaceAGORA.Conformance.test_service_facade(H; version=extension.REQUIRED_CONTRACT_VERSION)

    # From SpaceAGORA only the facade; from the facade its namespaces and the
    # provider checks; from each namespace its entries in the two inventories; from
    # Cloth the three listed services.
    entries = Set{Any}((H.IMPLEMENTED_FUNCTIONS..., H.CONSUMED_SERVICES...))
    namespaces = (:SwarmPolicy, :RPO, :RobotArm, :Planner)
    cloth = Set{Any}(H.CLOTH_SERVICES)
    cloth_modules = Dict(getglobal(H.Cloth, first(entry)) => first(entry) for entry in H.CLOTH_SERVICES)
    function allowed(m, name)
        m === SpaceAGORA && return name === :HYPRServices
        m === H && return name in (namespaces..., :Cloth, :check_provider, :activate!, :CompatibilityError, :require_version)
        m === H.Cloth && return any(entry -> first(entry) === name, H.CLOTH_SERVICES)
        haskey(cloth_modules, m) && return (cloth_modules[m], name) in cloth
        return any(ns -> m === getglobal(H, ns) && (ns, name) in entries, namespaces)
    end
    modules = (SpaceAGORA, H, map(ns -> getglobal(H, ns), namespaces)..., H.Cloth, keys(cloth_modules)...)
    policy = CorePolicy(SpaceAGORA, allowed, modules)

    @test isempty(runtime_violations(policy, extension))
    sources = String[]
    for (directory, _, files) in walkdir(joinpath(@__DIR__, "..", "ext")), file in files
        endswith(file, ".jl") && push!(sources, joinpath(directory, file))
    end
    found, used = source_violations(policy, sources)
    @test isempty(found)
    # A check that saw nothing would pass too: the source does name the
    # namespaces' entries, the provider checks and the Cloth services.
    @test length(sources) >= 15
    @test count(((m, _),) -> any(ns -> m === getglobal(H, ns), namespaces), used) >= 100
    @test (H, :check_provider) in used && (H, :activate!) in used
    @test count(((m, _),) -> haskey(cloth_modules, m), used) == length(H.CLOTH_SERVICES)
end
