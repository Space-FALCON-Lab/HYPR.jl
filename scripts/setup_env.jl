# Explicit standalone installation; shared policy is generated from core's
# scripts/companion_support/installation.jl. No core package is needed to plan.
module HYPRInstallation
using Pkg, TOML
include("support/installation.jl")
include("support/legacy_environment.jl")
using .CompanionInstaller: canonical_destination, inside, source_git, verify_core_checkout, acquire_core
const PACKAGE_ROOT = realpath(joinpath(@__DIR__, ".."))
const ENVIRONMENT_MARKER = ".spaceagora-hypr-environment.toml"

installation_plan(directory::AbstractString; kwargs...) =
    CompanionInstaller.installation_plan(PACKAGE_ROOT, directory;
        pin_file="SpaceAGORASource.toml", marker_name=ENVIRONMENT_MARKER, kwargs...)

setup(plan; precompile::Bool=true) = CompanionInstaller.setup(plan;
    marker_name=ENVIRONMENT_MARKER, package_name="HYPR", precompile)

"""Copy legacy project metadata into a fresh managed environment; leave the source untouched."""
function migration_plan(source::AbstractString, plan)
    source = realpath(source)
    destination = CompanionInstaller.validate_destination(plan.destination, plan.companion, plan.core, ENVIRONMENT_MARKER)
    destination == plan.destination || throw(ArgumentError("Environment destination changed after planning"))
    (inside(destination, source) || inside(source, destination)) &&
        throw(ArgumentError("Migration needs separate source and destination directories"))
    isdir(destination) && !isempty(readdir(destination)) &&
        throw(ArgumentError("Migration needs an empty destination; use ordinary setup to resume a prepared environment"))
    project_path = joinpath(source, "Project.toml")
    isfile(project_path) || throw(ArgumentError("Legacy source must contain Project.toml"))
    original = TOML.parsefile(project_path)
    dependencies = get(original, "deps", Dict())
    core_uuid = "afbfb69f-5c0b-4832-b760-43725dff8540"
    loader_uuid = "4096c904-79d0-4d9a-9f5d-f1d6c7a2f1b4"
    (get(dependencies, "SpaceAGORA", "") == core_uuid ||
        get(dependencies, "SpaceAGORAHYPR", "") == loader_uuid) ||
        throw(ArgumentError("Legacy project must declare SpaceAGORA or its retired loader by UUID"))
    for (name, uuid) in (("SpaceAGORA", core_uuid), ("SpaceAGORAHYPR", loader_uuid),
            ("HYPR", "342c8629-b435-4bda-9d24-004c735fb297"))
        !haskey(dependencies, name) || dependencies[name] == uuid ||
            throw(ArgumentError("Legacy project has an unexpected $name identity"))
    end
    records = LegacyEnvironmentMigration.migrated_records(project_path, joinpath(source, "Manifest.toml"))
    # Relative path dependencies retain their original meaning after copying.
    for spec in values(get(records.project, "sources", Dict()))
        spec isa AbstractDict && haskey(spec, "path") &&
            (spec["path"] = abspath(joinpath(source, spec["path"])))
    end
    if records.manifest !== nothing
        for entries in values(get(records.manifest, "deps", Dict())), entry in entries
            haskey(entry, "path") && (entry["path"] = abspath(joinpath(source, entry["path"])))
        end
    end
    return (; source, destination, project=records.project, manifest=records.manifest,
        preferences=joinpath(source, "LocalPreferences.toml"))
end

function migrate_environment(source::AbstractString, plan; precompile::Bool=true)
    migration = migration_plan(source, plan)
    destination = CompanionInstaller.validate_destination(plan.destination, plan.companion, plan.core, ENVIRONMENT_MARKER)
    destination == migration.destination || throw(ArgumentError("Environment destination changed after migration planning"))
    isdir(destination) && !isempty(readdir(destination)) && throw(ArgumentError("Migration destination is no longer empty"))
    mkpath(destination)
    open(joinpath(destination, ENVIRONMENT_MARKER), "w") do io
        TOML.print(io, Dict("companion" => plan.companion,
            "core" => (plan.core.path === nothing ? plan.core.revision : plan.core.path)))
    end
    open(io -> TOML.print(io, migration.project; sorted=true), joinpath(destination, "Project.toml"), "w")
    migration.manifest === nothing || open(io -> TOML.print(io, migration.manifest; sorted=true),
        joinpath(destination, "Manifest.toml"), "w")
    isfile(migration.preferences) && cp(migration.preferences, joinpath(destination, "LocalPreferences.toml"))
    return setup(plan; precompile)
end

function main(args=ARGS)
    migrate = length(args) == 3 && args[1] == "--migrate-from"
    (length(args) == 1 || migrate) || throw(ArgumentError("Usage: julia scripts/setup_env.jl [--migrate-from <old-project>] <environment-directory>"))
    plan = installation_plan(last(args))
    println("Companion: ", plan.companion)
    println("Core: ", plan.core.path === nothing ? "$(plan.core.url) @ $(plan.core.revision)" : plan.core.path)
    return migrate ? migrate_environment(args[2], plan) : setup(plan)
end
end
abspath(PROGRAM_FILE) == abspath(@__FILE__) && HYPRInstallation.main()
