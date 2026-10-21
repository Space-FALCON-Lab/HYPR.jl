# Run in a fresh process in the integration environment. The extension must refuse
# a SpaceAGORA whose HYPRServices contract is not the one it implements, before it
# defines a module or a method. This copy of the extension changes only its required
# contract, to the previous one, 1.0.0, so the pinned core's 2.0.0 is refused.
using Test
using SpaceAGORA

@testset "Extension refuses another contract before defining methods" begin
    H = SpaceAGORA.HYPRServices
    path = joinpath(@__DIR__, "..", "ext", "HYPRSpaceAGORAExt.jl")
    source = read(path, String)
    expected = "const REQUIRED_CONTRACT_VERSION = v\"2.0.0\""
    @test count(expected, source) == 1
    incompatible = replace(source, expected => "const REQUIRED_CONTRACT_VERSION = v\"1.0.0\"")
    failure = try
        # Relative includes in the copy resolve from ext/, as they do for the real
        # extension, so a copy that checked late would define everything.
        task_local_storage(:SOURCE_PATH, path) do
            Base.include_string(Main, incompatible, path)
        end
        nothing
    catch error
        while error isa LoadError
            error = error.error
        end
        error
    end
    @test failure isa H.CompatibilityError
    message = failure === nothing ? "" : sprint(showerror, failure)
    @test occursin("1.0.0", message) && occursin("2.0.0", message)
    copy = Main.HYPRSpaceAGORAExt
    # Importing HYPR in the copy loaded the real extension; the copy is a separate module.
    @test copy !== Base.get_extension(Base.require(Main, :HYPR), :HYPRSpaceAGORAExt)
    for name in (:SwarmPolicy, :RPO, :RobotArm, :PlannerAdapter)
        @test !isdefined(copy, name)
    end
    @test !copy.initialized()
    within(m) = m === copy || (parentmodule(m) !== m && within(parentmodule(m)))
    for entry in H.IMPLEMENTED_FUNCTIONS
        f = foldl(getproperty, entry; init=H)
        @test !any(method -> within(method.module), methods(f))
    end
end
