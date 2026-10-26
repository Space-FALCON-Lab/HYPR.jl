using Test, TOML
include(joinpath(@__DIR__, "..", "scripts", "setup_env.jl"))
include(joinpath(@__DIR__, "support", "installation_contract.jl"))
CompanionInstallationContract.test_installation_contract(HYPRInstallation)
@testset "Optional core remains optional to the HYPR package" begin
    plan = HYPRInstallation.installation_plan(joinpath(mktempdir(), "environment"); core_path="", core_revision="")
    @test plan.core.revision == TOML.parsefile(joinpath(@__DIR__, "..", "SpaceAGORASource.toml"))["rev"]
    @test !haskey(plan.dependencies, "SpaceAGORA")
    @test !haskey(plan.dependencies, "SpaceAGORAHYPR")
    @test all(haskey(plan.dependencies, name) for name in ("Random", "StaticArrays", "Test"))
end

@testset "Legacy environment migration is explicit and preserves its source" begin
    mktempdir() do temporary
        core = mkpath(joinpath(temporary, "core"))
        write(joinpath(core, "Project.toml"), "name = \"SpaceAGORA\"\nuuid = \"afbfb69f-5c0b-4832-b760-43725dff8540\"\n")
        source = realpath(mkpath(joinpath(temporary, "old")))
        destination = joinpath(temporary, "new")
        core_uuid = "afbfb69f-5c0b-4832-b760-43725dff8540"
        loader_uuid = "4096c904-79d0-4d9a-9f5d-f1d6c7a2f1b4"
        hypr_uuid = "342c8629-b435-4bda-9d24-004c735fb297"
        project = Dict("deps" => Dict("SpaceAGORA" => core_uuid, "SpaceAGORAHYPR" => loader_uuid),
            "compat" => Dict("SpaceAGORAHYPR" => "0.2"),
            "sources" => Dict("SpaceAGORAHYPR" => Dict("path" => "deleted-loader"),
                "Other" => Dict("path" => "../other")), "preferences" => Dict("keep" => true))
        manifest = Dict("manifest_format" => "2.0", "julia_version" => string(VERSION), "deps" => Dict(
            "SpaceAGORA" => [Dict("uuid" => core_uuid, "version" => "0.7.0", "path" => core)],
            "HYPR" => [Dict("uuid" => hypr_uuid, "version" => "0.5.0", "weakdeps" => ["SpaceAGORA", "SpaceAGORAHYPR"])],
            "SpaceAGORAHYPR" => [Dict("uuid" => loader_uuid, "version" => "0.2.0", "path" => "deleted-loader")]))
        write_toml(name, data) = open(io -> TOML.print(io, data), joinpath(source, name), "w")
        write_toml("Project.toml", project); write_toml("Manifest.toml", manifest)
        write(joinpath(source, "LocalPreferences.toml"), "[Keep]\nvalue = 42\n")
        files = ["Project.toml", "Manifest.toml", "LocalPreferences.toml"]
        before = Dict(f => read(joinpath(source, f)) for f in files)
        plan(path=destination) = HYPRInstallation.installation_plan(path; core_path=core, core_revision="")
        migration = HYPRInstallation.migration_plan(source, plan())
        @test !haskey(migration.project["deps"], "SpaceAGORAHYPR")
        @test !haskey(migration.project["sources"], "SpaceAGORAHYPR")
        @test !haskey(migration.project["compat"], "SpaceAGORAHYPR")
        @test migration.project["preferences"]["keep"]
        @test migration.project["sources"]["Other"]["path"] == normpath(joinpath(source, "../other"))
        @test !haskey(migration.manifest["deps"], "SpaceAGORAHYPR")
        @test migration.manifest["deps"]["HYPR"][1]["weakdeps"] == Dict("SpaceAGORA" => core_uuid, "SpaceAGORAHYPR" => loader_uuid)
        @test Dict(f => read(joinpath(source, f)) for f in files) == before
        @test !ispath(destination)
        @test_throws ArgumentError HYPRInstallation.migration_plan(source, plan(joinpath(source, "nested")))
        for deps in (["SpaceAGORAHYPR"], Dict("SpaceAGORAHYPR" => loader_uuid))
            bad = deepcopy(manifest)
            bad["deps"]["Consumer"] = [Dict("uuid" => "7296cf7c-9f96-44d3-93c0-ef879936dd03", "deps" => deps)]
            write_toml("Manifest.toml", bad)
            current = Dict(f => read(joinpath(source, f)) for f in files)
            @test_throws ErrorException HYPRInstallation.migrate_environment(source, plan(); precompile=false)
            @test Dict(f => read(joinpath(source, f)) for f in files) == current
            @test !ispath(destination)
            delete!(bad["deps"], "SpaceAGORAHYPR")
            write_toml("Manifest.toml", bad)
            missing_before = read(joinpath(source, "Manifest.toml"))
            @test_throws ErrorException HYPRInstallation.migrate_environment(source, plan(); precompile=false)
            @test read(joinpath(source, "Manifest.toml")) == missing_before
            @test !ispath(destination)
        end
        write_toml("Manifest.toml", manifest)
        project["deps"]["HYPR"] = core_uuid
        write_toml("Project.toml", project)
        @test_throws ArgumentError HYPRInstallation.migration_plan(source, plan())
        @test !ispath(destination)
    end
end
