using Test
mode = only(ARGS)
# SpaceAGORA releases before the loader's retirement install the SpaceAGORAHYPR
# compatibility loader beside HYPR; later ones load HYPR directly.
const LOADER = Base.find_package("SpaceAGORAHYPR") !== nothing
    if mode == "core"
        using HYPR
        @test all(id.name != "SpaceAGORA" for id in keys(Base.loaded_modules))
        packages = [HYPR]
    elseif mode == "core-first"
        using SpaceAGORA
        @test !SpaceAGORA.hypr_available()
        using HYPR
        LOADER && @eval using SpaceAGORAHYPR
        packages = [SpaceAGORA, HYPR, Base.get_extension(HYPR, :HYPRSpaceAGORAExt)]
    elseif mode == "hypr-first"
        using HYPR
        @test all(id.name != "SpaceAGORA" for id in keys(Base.loaded_modules))
        using SpaceAGORA
        LOADER && @eval using SpaceAGORAHYPR
        packages = [SpaceAGORA, HYPR, Base.get_extension(HYPR, :HYPRSpaceAGORAExt)]
    elseif mode == "shim-first"
        LOADER || error("shim-first needs the SpaceAGORAHYPR compatibility loader in this environment")
        using SpaceAGORAHYPR, SpaceAGORA, HYPR
        packages = [SpaceAGORA, HYPR, Base.get_extension(HYPR, :HYPRSpaceAGORAExt)]
    else
        error("Unknown loading mode")
    end
if mode != "core" && LOADER
        push!(packages, SpaceAGORAHYPR)
        partial = Module(:PartiallyInitialized)
        Core.eval(partial, :(begin
            const SwarmPolicy=Main.SpaceAGORAHYPR.SwarmPolicy
            const RPO=Main.SpaceAGORAHYPR.RPO
            const RobotArm=Main.SpaceAGORAHYPR.RobotArm
            const PlannerAdapter=Main.SpaceAGORAHYPR.PlannerAdapter
            initialized()=false
        end))
end
@testset "Precompiled HYPR stack: $mode" begin
    for package in packages
        id = Base.PkgId(package)
        @test Base.isprecompiled(id)
        @test Base.pkgorigins[id].cachepath !== nothing
        cache = Base.compilecache_path(id)
        @test cache !== nothing && isfile(cache)
        @test isfile(Base.ocachefile_from_cachefile(cache))
        println("PACKAGE_IMAGE ", id.name, " ", cache)
    end
    if mode != "core"
        @test SpaceAGORA.hypr_available()
        @test Base.get_extension(HYPR, :HYPRSpaceAGORAExt).initialized()
    end
    if mode != "core" && LOADER
        @test SpaceAGORAHYPR.Adapter.initialized()
        @test SpaceAGORAHYPR.checked_adapter(SpaceAGORAHYPR.Adapter; require_initialized=true) === SpaceAGORAHYPR.Adapter
        @test_throws SpaceAGORA.HYPRServices.CompatibilityError SpaceAGORAHYPR.checked_adapter(nothing; require_initialized=false)
        @test_throws SpaceAGORA.HYPRServices.CompatibilityError SpaceAGORAHYPR.checked_adapter(Module(:Incomplete); require_initialized=false)
        @test SpaceAGORAHYPR.checked_adapter(partial;require_initialized=false) === partial
        @test_throws SpaceAGORA.HYPRServices.CompatibilityError SpaceAGORAHYPR.checked_adapter(partial;require_initialized=true)
    end
end
