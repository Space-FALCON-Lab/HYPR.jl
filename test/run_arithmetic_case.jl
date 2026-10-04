using HYPR, Test, Random
if !isempty(ARGS) && only(ARGS)!="none"
    include("mutation_controls.jl")
    apply_mutant!(only(ARGS))
end
include("arithmetic_regressions.jl")
