# Exercise actual offline package resolution, including removed historical paths.
using Test, TOML, Pkg
isdefined(@__MODULE__, :HYPRInstallation) || include(joinpath(@__DIR__, "..", "scripts", "setup_env.jl"))
@testset "Legacy migration resolves the selected pair together" begin
    mktempdir() do temporary
        temporary = realpath(temporary)
        core_uuid = "afbfb69f-5c0b-4832-b760-43725dff8540"
        hypr_uuid = "342c8629-b435-4bda-9d24-004c735fb297"
        loader_uuid = "4096c904-79d0-4d9a-9f5d-f1d6c7a2f1b4"
        local_uuid = "7296cf7c-9f96-44d3-93c0-ef879936dd03"
        write_toml(path, data) = open(io -> TOML.print(io, data), path, "w")
        function package(name, uuid, version, folder)
            path = mkpath(joinpath(temporary, folder)); mkpath(joinpath(path, "src"))
            package_metadata = Dict{String,Any}("name" => name, "uuid" => uuid, "version" => version)
            write_toml(joinpath(path, "Project.toml"), package_metadata)
            write(joinpath(path, "src", name * ".jl"), "module $name\nend\n")
            return path, package_metadata
        end
        core, _ = package("SpaceAGORA", core_uuid, "0.7.1", "selected-core")
        companion, project = package("HYPR", hypr_uuid, "0.5.0", "selected-hypr")
        support, _ = package("LocalSupport", local_uuid, "0.1.0", "local-support")
        project["weakdeps"] = Dict("SpaceAGORA" => core_uuid)
        write_toml(joinpath(companion, "Project.toml"), project)
        write_toml(joinpath(companion, "SpaceAGORASource.toml"), Dict(
            "url" => "https://github.com/Space-FALCON-Lab/SpaceAGORA.jl.git", "uuid" => core_uuid, "rev" => repeat("a", 40)))
        source = mkpath(joinpath(temporary, "old"))
        removed_core = joinpath(temporary, "removed-core")
        removed_hypr = joinpath(temporary, "removed-hypr")
        old_project = Dict("deps" => Dict("SpaceAGORA" => core_uuid, "HYPR" => hypr_uuid,
            "SpaceAGORAHYPR" => loader_uuid, "LocalSupport" => local_uuid),
            "sources" => Dict("SpaceAGORA" => Dict("path" => removed_core), "HYPR" => Dict("path" => removed_hypr),
                "LocalSupport" => Dict("path" => "../local-support")), "preferences" => Dict("keep" => true))
        old_manifest = Dict("manifest_format" => "2.0", "julia_version" => string(VERSION), "deps" => Dict(
            "SpaceAGORA" => [Dict("uuid" => core_uuid, "version" => "0.7.1", "path" => removed_core)],
            "HYPR" => [Dict("uuid" => hypr_uuid, "version" => "0.5.0", "path" => removed_hypr)],
            "LocalSupport" => [Dict("uuid" => local_uuid, "version" => "0.1.0", "path" => "../local-support")],
            "SpaceAGORAHYPR" => [Dict("uuid" => loader_uuid, "version" => "0.2.0", "path" => "removed-loader")]))
        write_toml(joinpath(source, "Project.toml"), old_project)
        write_toml(joinpath(source, "Manifest.toml"), old_manifest)
        write(joinpath(source, "LocalPreferences.toml"), "[LocalSupport]\nvalue = 42\n")
        source_before = Dict(f => read(joinpath(source, f)) for f in readdir(source))
        selected_before = (read(joinpath(core, "Project.toml")), read(joinpath(companion, "Project.toml")))
        destination = joinpath(temporary, "new")
        plan = HYPRInstallation.CompanionInstaller.installation_plan(companion, destination;
            pin_file="SpaceAGORASource.toml", marker_name=HYPRInstallation.ENVIRONMENT_MARKER,
            core_path=core, core_revision="")
        active = Base.active_project()
        offline = Pkg.OFFLINE_MODE[]
        try
            Pkg.offline(true)
            @test HYPRInstallation.migrate_environment(source, plan; precompile=false) == destination
            # A standard managed setup is retryable after migration, using the same pair.
            @test HYPRInstallation.setup(plan; precompile=false) == destination
        finally
            Pkg.offline(offline)
        end
        @test Base.active_project() == active
        migrated = TOML.parsefile(joinpath(destination, "Manifest.toml"))["deps"]
        @test realpath(migrated["SpaceAGORA"][1]["path"]) == core
        @test realpath(migrated["HYPR"][1]["path"]) == companion
        @test realpath(migrated["LocalSupport"][1]["path"]) == support
        @test !haskey(migrated, "SpaceAGORAHYPR")
        installed_project = TOML.parsefile(joinpath(destination, "Project.toml"))
        @test installed_project["preferences"]["keep"]
        @test installed_project["sources"]["SpaceAGORA"]["path"] == core
        @test installed_project["sources"]["HYPR"]["path"] == companion
        @test read(joinpath(destination, "LocalPreferences.toml")) == source_before["LocalPreferences.toml"]
        @test Dict(f => read(joinpath(source, f)) for f in readdir(source)) == source_before
        @test selected_before == (read(joinpath(core, "Project.toml")), read(joinpath(companion, "Project.toml")))
        @test !ispath(removed_core) && !ispath(removed_hypr)
    end
end
