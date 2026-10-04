using Test
mode = only(ARGS)
    if mode == "core"
        using HYPR
        @test all(id.name != "SpaceAGORA" for id in keys(Base.loaded_modules))
        packages = [HYPR]
    elseif mode == "core-first"
        using SpaceAGORA
        @test !SpaceAGORA.hypr_available()
        using HYPR
        using SpaceAGORAHYPR
        packages = [SpaceAGORA, HYPR, Base.get_extension(HYPR, :HYPRSpaceAGORAExt), SpaceAGORAHYPR]
    elseif mode == "hypr-first"
        using HYPR
        @test all(id.name != "SpaceAGORA" for id in keys(Base.loaded_modules))
        using SpaceAGORA
        using SpaceAGORAHYPR
        packages = [SpaceAGORA, HYPR, Base.get_extension(HYPR, :HYPRSpaceAGORAExt), SpaceAGORAHYPR]
    elseif mode == "shim-first"
        using SpaceAGORAHYPR, SpaceAGORA, HYPR
        packages = [SpaceAGORA, HYPR, Base.get_extension(HYPR, :HYPRSpaceAGORAExt), SpaceAGORAHYPR]
    else
        error("Unknown loading mode")
    end
if mode != "core"
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
        @test SpaceAGORAHYPR.Adapter.initialized()
        @test SpaceAGORAHYPR.checked_adapter(SpaceAGORAHYPR.Adapter; require_initialized=true) === SpaceAGORAHYPR.Adapter
        @test_throws SpaceAGORA.HYPRServices.CompatibilityError SpaceAGORAHYPR.checked_adapter(nothing; require_initialized=false)
        @test_throws SpaceAGORA.HYPRServices.CompatibilityError SpaceAGORAHYPR.checked_adapter(Module(:Incomplete); require_initialized=false)
        @test SpaceAGORAHYPR.checked_adapter(partial;require_initialized=false) === partial
        @test_throws SpaceAGORA.HYPRServices.CompatibilityError SpaceAGORAHYPR.checked_adapter(partial;require_initialized=true)
    end
end
