# Local pairing helper. Published source resolution remains release package 3.
using Pkg, TOML
length(ARGS)==2 || error("usage: setup_integration.jl SPACEAGORA_CHECKOUT ENVIRONMENT")
sa,env=abspath.(ARGS);root=normpath(joinpath(@__DIR__,".."))
isfile(joinpath(sa,"src","gnc","hypr","services.jl")) ||
    error("The selected SpaceAGORA must contain the reviewed HYPRServices contract.")
project=TOML.parsefile(joinpath(sa,"Project.toml"))
mkpath(env)
open(joinpath(env,"Project.toml"),"w") do io
    TOML.print(io,Dict("deps"=>project["deps"]);sorted=true)
end
cp(joinpath(sa,"Manifest.toml"),joinpath(env,"Manifest.toml");force=true)
Pkg.activate(env)
Pkg.develop([PackageSpec(path=root),PackageSpec(path=sa),PackageSpec(path=joinpath(sa,"packages","SpaceAGORAHYPR"))];preserve=Pkg.PRESERVE_ALL)
Pkg.instantiate()
