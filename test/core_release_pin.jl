# The integration job runs against a released SpaceAGORA: the core commit it checks
# out must be the commit of a SpaceAGORA release tag, and HYPR's compatibility range
# must admit that release. Needs network access to the core repository; CI runs it
# as its own job.
#   julia --startup-file=no test/core_release_pin.jl
using Pkg, Test, TOML

const ROOT = joinpath(@__DIR__, "..")
const CORE = "https://github.com/Space-FALCON-Lab/SpaceAGORA.jl.git"
workflow = read(joinpath(ROOT, ".github", "workflows", "ci.yml"), String)
pins = [m.captures[1] for m in eachmatch(r"ref:\s*([0-9a-f]{40})\s*\n\s*path:\s*\.integration-spaceagora\s*\n", workflow)]

# Release versions whose tag points at `revision` (an annotated tag through its ^{} line).
function release_versions(url, revision)
    versions = VersionNumber[]
    for line in readlines(Cmd(["git", "ls-remote", "--tags", url]))
        commit, ref = split(line, '\t')
        tag = match(r"^refs/tags/v(\d+\.\d+\.\d+)(\^\{\})?$", ref)
        tag !== nothing && commit == revision && push!(versions, VersionNumber(tag.captures[1]))
    end
    return unique!(versions)
end

@testset "The integration job's core is a SpaceAGORA release" begin
    @test length(pins) == 1
    versions = length(pins) == 1 ? release_versions(CORE, only(pins)) : VersionNumber[]
    @test length(versions) == 1
    if length(versions) == 1
        release = only(versions)
        println("Integration core $(only(pins)) is SpaceAGORA v$release.")
        compat = TOML.parsefile(joinpath(ROOT, "Project.toml"))["compat"]["SpaceAGORA"]
        @test release in Pkg.Versions.semver_spec(compat)
    end
end
