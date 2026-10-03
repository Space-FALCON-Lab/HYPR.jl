"""HYPR search kernels and optional SpaceAGORA integration."""
module HYPR
using Random
using Base.Threads: @threads
export SwarmState, SearchSettings, SearchPolicy, search!, advance_particles!
export hypr_iteration_weights, hypr_material_improvement, hypr_protected_particle_mask
include("swarm_policy.jl")
include("search.jl")
end
