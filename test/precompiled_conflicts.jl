using Test, SpaceAGORA
mode=only(ARGS)
const Support=SpaceAGORA.SimulationModel.HYPRSupport
const CompatibilityError=SpaceAGORA.HYPRServices.CompatibilityError
if mode=="conflict-first"
    # This is the exact activation protocol of the unsupported 0.1 companion.
    @test_throws CompatibilityError Support.activate!()
    using HYPR
elseif mode=="hypr-first"
    using HYPR, SpaceAGORAHYPR
    @test SpaceAGORA.hypr_available()
    @test_throws CompatibilityError Support.activate!()
else
    error("Expected conflict-first or hypr-first")
end
@testset "Compiled provider conflict stays unavailable: $mode" begin
    @test Base.pkgorigins[Base.PkgId(SpaceAGORA)].cachepath !== nothing
    @test Base.pkgorigins[Base.PkgId(HYPR)].cachepath !== nothing
    ext=Base.get_extension(HYPR,:HYPRSpaceAGORAExt)
    # Failed __init__ is deliberately not exposed by get_extension, although
    # Julia has already restored the image. The origin records that restore.
    ext_ids = [id for id in keys(Base.pkgorigins) if id.name == "HYPRSpaceAGORAExt"]
    @test length(ext_ids) == 1
    @test Base.pkgorigins[only(ext_ids)].cachepath !== nothing
    @test (ext === nothing) == (mode == "conflict-first")
    @test !SpaceAGORA.hypr_available()
    @test_throws CompatibilityError Support.require_hypr()
    @test_throws CompatibilityError SpaceAGORA.make_rpo_configuration(
        planner=SpaceAGORA.HYPRRPOPlanner(SpaceAGORA.RPOPSOConfig()))
    if mode=="conflict-first"
        error_seen=try
            @eval using SpaceAGORAHYPR
            nothing
        catch e
            e
        end
        @test error_seen !== nothing
        @test occursin("did not initialize successfully",sprint(showerror,error_seen))
    else
        @test_throws CompatibilityError SpaceAGORAHYPR.checked_adapter(ext;require_initialized=true)
    end
end
