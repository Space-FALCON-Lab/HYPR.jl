# The shared workflow tests the current HYPR checkout in its isolated core pair.
# Keep each probe in a fresh process: integration.jl checks pre-load availability.
# Standalone runtests.jl remains in the separate 1/4-thread core jobs.
using Test
@test realpath(dirname(dirname(Base.find_package("HYPR")))) == realpath(joinpath(@__DIR__, ".."))
project = dirname(Base.active_project())
for (script, args) in (("integration.jl", ["core-first"]),
                       ("integration.jl", ["hypr-first"]),
                       ("extension_helpers.jl", String[]))
    run(`$(Base.julia_cmd()) --startup-file=no --project=$project --depwarn=error $(joinpath(@__DIR__, script)) $args`)
end
