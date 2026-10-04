using HYPR, SpaceAGORA, Test
# Run maintained SpaceAGORA consumer tests from the selected integration pair.
# No test-source copies or native datasets are needed. Release pairing is R5.
const SA = dirname(dirname(pathof(SpaceAGORA)))
const INTEGRATION_CASES = [
    "probes/hypr_search_probes.jl", "probes/robot_arm_hypr_probes.jl",
    "probes/rpo_planning_probes.jl", "unit/gnc/rpo_planner_adapter_tests.jl",
    "unit/gnc/rpo_hypr_manuscript_tests.jl", "unit/robotics/runtests.jl",
    "unit/gnc/shared_metrics_tests.jl"]
for path in INTEGRATION_CASES
    println("COVERAGE_TEST=",path)
    m=Module(gensym(:HYPRCoverage))
    Core.eval(m, :(include(path)=Base.include(@__MODULE__,path)))
    Base.include(m, joinpath(SA,"test",path))
end

include("sampling_compatibility.jl")
