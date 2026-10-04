# Explicit local development pairing used by HYPR's own integration jobs.
length(ARGS)==2 || error("usage: setup_integration.jl SPACEAGORA_CHECKOUT ENVIRONMENT")
sa,env=abspath.(ARGS)
include(joinpath(sa, "scripts", "setup_hypr.jl"))
HYPRInstallation.setup(env; root=sa, local_path=normpath(joinpath(@__DIR__, "..")))
