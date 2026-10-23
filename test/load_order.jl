# Shared workflow names the provider-first order companion-first.
length(ARGS) == 1 && only(ARGS) in ("core-first", "companion-first") ||
    error("Pass core-first or companion-first")
ARGS[1] == "companion-first" && (ARGS[1] = "hypr-first")
include(joinpath(@__DIR__, "service_boundary.jl"))
