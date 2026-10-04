using Pkg, TOML, Test
length(ARGS)==1 || error("usage: incompatible_resolution.jl SPACEAGORA_CHECKOUT")
sa=abspath(only(ARGS));hypr=normpath(joinpath(@__DIR__,".."))
@testset "Resolver refuses SpaceAGORA before the service release" begin
    mktempdir() do scratch
        old=joinpath(scratch,"old");mkpath(joinpath(old,"src"))
        open(joinpath(old,"Project.toml"),"w") do io
            TOML.print(io,Dict("name"=>"SpaceAGORA","uuid"=>"afbfb69f-5c0b-4832-b760-43725dff8540","version"=>"0.1.0"))
        end
        write(joinpath(old,"src","SpaceAGORA.jl"),"module SpaceAGORA; end\n")
        Pkg.activate(joinpath(scratch,"env"))
        failure=try
            Pkg.develop([PackageSpec(path=hypr),PackageSpec(path=old),PackageSpec(path=joinpath(sa,"packages","SpaceAGORAHYPR"))];preserve=Pkg.PRESERVE_ALL)
            nothing
        catch e
            e
        end
        @test failure !== nothing
        @test nameof(typeof(failure)) == :ResolverError
        message=sprint(showerror,failure);println(message)
        @test occursin("SpaceAGORA",message)
        @test occursin("0.1.0",message)
        @test occursin("restricted to versions 0.2 by HYPR",message)
    end
end
