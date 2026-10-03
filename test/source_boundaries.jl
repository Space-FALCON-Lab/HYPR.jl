# Package-applicable rules transferred from SpaceAGORA test/gates at 9be384a2.
# SpaceAGORA keeps its command-owner and canonical-aggregator assertions.
using Test

function source_boundary_violations(relative_path, text)
    active = join((strip(first(split(line, '#'; limit=2))) for line in split(text,'\n')), '\n')
    errors = String[]
    occursin(r"\b[a-zA-Z0-9_]*_stub\(\)\s*=\s*nothing\b", text) && push!(errors,"silent_stub")
    for token in ("Compatibility wrapper: canonical path forwarding to legacy implementation.","No-op ")
        occursin(token,text) && push!(errors,"incomplete_source")
    end
    occursin("__legacy_",active) && push!(errors,"legacy_include")
    occursin(r"include\(joinpath\(@__DIR__,\s*\"\.\.\",\s*\"\.\.\",\s*\"\.\.\",",active) && push!(errors,"parent_include")
    for line in split(active,'\n')
        if startswith(line,"include(")
            occursin("control",line) && push!(errors,"control_include")
            relative_path in ("src/HYPR.jl","ext/HYPRSpaceAGORAExt.jl") || push!(errors,"include_owner")
        end
    end
    for token in ("DynamicEffectors","maneuver_commands")
        occursin(token,active) && push!(errors,"command_owner")
    end
    for pattern in (r"\bBaseThrusterModel\b",r"\bAbstractThrusterModel\b",r"\bThrusterModels\b",
        r"\.\s*Δv\s*\[",r"\.\s*direction\s*\[",r"\.\s*start_burn_time\s*\[",
        r"\.\s*stop_burn_time\s*\[",r"\.\s*thrust\s*\[")
        occursin(pattern,active) && push!(errors,"thruster_state")
    end
    startswith(relative_path,"src/") && occursin(r"(?m)^\s*(?:using|import)\s+SpaceAGORA\b|\bSpaceAGORA\.",active) && push!(errors,"core_dependency")
    return unique(errors)
end

@testset "Source boundaries and negative controls" begin
    root=normpath(joinpath(@__DIR__,".."));count=0
    for folder in ("src","ext"), (dir,_,files) in walkdir(joinpath(root,folder)), file in files
        endswith(file,".jl") || continue
        path=joinpath(dir,file); rel=replace(relpath(path,root),'\\'=>'/')
        @test isempty(source_boundary_violations(rel,read(path,String)))
        count+=1
    end
    @test count >= 18 # refuses an accidentally empty/missing extracted tree
    for bad in ("lost_stub() = nothing", "No-op implementation", "__legacy_missing",
        "include(joinpath(@__DIR__, \"..\", \"..\", \"..\", \"file.jl\"))",
        "include(\"control.jl\")", "include(\"other.jl\")", "DynamicEffectors", "maneuver_commands",
        "BaseThrusterModel", "AbstractThrusterModel", "ThrusterModels", "x.Δv[1]", "x.direction[1]",
        "x.start_burn_time[1]", "x.stop_burn_time[1]", "x.thrust[1]", "import SpaceAGORA")
        @test !isempty(source_boundary_violations("src/search.jl",bad))
    end
    @test isempty(source_boundary_violations("src/HYPR.jl","include(\"search.jl\")"))
    @test isempty(source_boundary_violations("ext/HYPRSpaceAGORAExt.jl","import SpaceAGORA"))
end
