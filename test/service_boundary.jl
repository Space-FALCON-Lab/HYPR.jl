# Run in a fresh process in the integration environment:
#   julia --project=<environment> test/service_boundary.jl core-first|hypr-first
# Core-first first checks that SpaceAGORA refuses HYPR planning before HYPR loads.
# Both orders then check that the extension reaches SpaceAGORA only through names
# the HYPRServices contract lists: a contract change that drops a name HYPR uses,
# or a new use of a name the contract does not list, fails here, in this
# repository's CI and in SpaceAGORA's, which runs it against the core it changes.
using Test, Random
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
    entries = Set{Any}((H.IMPLEMENTED_FUNCTIONS..., H.CONSUMED_SERVICES...))
    # Each extension namespace imports from one facade namespace.
    namespaces = (SwarmPolicy=:SwarmPolicy, RPO=:RPO, RobotArm=:RobotArm, PlannerAdapter=:Planner)
    from_core(value) = try
        Base.moduleroot(value isa Module ? value : parentmodule(value)) === SpaceAGORA
    catch
        false
    end
    imported = 0
    for (name, namespace) in pairs(namespaces)
        M = getproperty(extension, name)
        for binding in names(M; all=true, imported=true, usings=true)
            binding in (:SpaceAGORA, :HYPR) && continue
            isdefined(M, binding) && Base.binding_module(M, binding) !== M || continue
            value = getglobal(M, binding)
            from_core(value) || continue
            imported += 1
            @test (namespace, binding) in entries && getglobal(getproperty(H, namespace), binding) === value
        end
    end
    @test imported >= 100  # refuses a check that silently found no imports
    # Spelled paths: the extension's aliases of facade namespaces, and the Cloth services.
    # An alias defined in the extension's main file applies in the files it includes.
    sources = String[]
    for (directory, _, files) in walkdir(joinpath(@__DIR__, "..", "ext")), file in files
        endswith(file, ".jl") || continue
        push!(sources, join((first(split(line, '#'; limit=2)) for line in eachline(joinpath(directory, file))), '\n'))
    end
    @test length(sources) >= 15
    aliases = Dict(m.captures[1] => Symbol(m.captures[2]) for code in sources for m in
        eachmatch(r"const\s+([A-Z])\s*=\s*SpaceAGORA\.HYPRServices\.([A-Za-z]+)", code))
    for code in sources, m in eachmatch(r"const\s+([A-Z])\s*=\s*([A-Z])\s*$"m, code)
        haskey(aliases, m.captures[2]) && (aliases[m.captures[1]] = aliases[m.captures[2]])
    end
    @test aliases == Dict("P" => :Planner, "S" => :RPO, "G" => :RPO)
    for code in sources
        for (alias, namespace) in aliases, m in eachmatch(Regex("(?<![A-Za-z0-9_.])" * alias * "\\.([A-Za-z_][A-Za-z0-9_!]*)"), code)
            @test (namespace, Symbol(m.captures[1])) in entries
        end
        for m in eachmatch(r"SpaceAGORA\.HYPRServices\.([A-Za-z_][A-Za-z0-9_!]*)(?:\.([A-Za-z_][A-Za-z0-9_!]*))?", code)
            head = Symbol(m.captures[1])
            if head in (:check_provider, :activate!, :CompatibilityError, :require_version, :Cloth)
                continue
            end
            @test head in (:SwarmPolicy, :RPO, :RobotArm, :Planner)
            m.captures[2] === nothing || @test (head, Symbol(m.captures[2])) in entries
        end
        for m in eachmatch(r"getfield\(\s*(?:dynmod|mbmod)\s*,\s*:([A-Za-z_][A-Za-z0-9_!]*)\s*\)", code)
            @test any(entry -> last(entry) === Symbol(m.captures[1]), H.CLOTH_SERVICES)
        end
    end
end
