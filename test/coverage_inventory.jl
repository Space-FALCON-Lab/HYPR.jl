module CoverageInventoryTests
using Test
include(joinpath(@__DIR__, "..", "scripts", "coverage_inventory.jl"))
@testset "Named and anonymous function coverage inventory" begin
    source = """
    function named(x)
        x + 1
    end
    short(x) = x + 2
    arrow = x -> x + 3
    block_arrow = x -> begin
        x + 4
    end
    map([1, 2]) do x
        x + 5
    end
    anonymous = function (x)
        x + 6
    end
    """
    spans = function_spans!(Dict{String,Any}[], JS.parseall(JS.SyntaxNode, source))
    @test length(spans) == 6
    @test count(s -> s["kind"] == "function", spans) == 3
    @test count(s -> s["kind"] == "->", spans) == 2
    @test count(s -> s["kind"] == "do", spans) == 1
    @test [(s["first"], s["last"]) for s in spans] == [(1, 3), (4, 4), (5, 5), (7, 7), (10, 10), (13, 13)]
    # The call and callback body have separate lines. Never credit the call's
    # execution to a callback whose body is wholly unmeasured.
    for (text, kind) in (("f = x ->\n    x + 1", "->"),
                         ("map(Int[]) do x\n    x + 1\nend", "do"))
        entry = only(function_spans!(Dict{String,Any}[], JS.parseall(JS.SyntaxNode, text)))
        @test entry["kind"] == kind
        @test entry["first"] >= 2
    end
end
end
