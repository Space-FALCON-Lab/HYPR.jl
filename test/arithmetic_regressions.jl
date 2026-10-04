using Test, HYPR, TOML
include("arithmetic_cases.jl")
const ARITHMETIC_REFERENCE=TOML.parsefile(joinpath(@__DIR__,"fixtures","arithmetic_parent.toml"))
@testset "Accepted parent arithmetic and random draw order" begin
    actual=arithmetic_records(HYPR)
    @test Set(keys(actual))==Set(keys(ARITHMETIC_REFERENCE["records"]))
    for name in sort!(collect(keys(actual)))
        @testset "$name" begin
            reference=ARITHMETIC_REFERENCE["records"][name]
            @test Set(keys(actual[name]))==Set(keys(reference))
            for key in sort!(collect(keys(reference)))
                @testset "$key" begin
                    @test actual[name][key]==reference[key]
                end
            end
        end
    end
end
