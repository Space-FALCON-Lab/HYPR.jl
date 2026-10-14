using SpaceAGORA, HYPR
using LinearAlgebra, Random

"""Plan a small, obstacle-free arm motion; return the reference and its settings."""
function robot_arm_example(seed=741)
    hypr_available() || error("Load compatible SpaceAGORA and HYPR packages first.")
    arm = default_robot_arm_model()
    base = RobotArmBasePose([0.0, 0.0, 0.0])
    q_start = [0.0, 0.3, -0.3]
    # A reachable Cartesian target, constructed from a different joint pose.
    target = robot_arm_fk(arm, base, [0.6, -0.4, 0.4]).end_effector_position
    planner = RobotArmPlannerConfig(dt_s=0.1, duration_s=2.0)
    search = RobotArmHYPRConfig(n_particles=8, n_iters=8, n_waypoints=1,
        n_samples=32, rrt_warmstart_enable=false, retime_enable=false)
    result = plan_robot_arm_motion_hypr(arm, base, q_start, target;
        planner_config=planner, hypr_config=search, rng=MersenneTwister(seed))
    return (; result, planner, q_start, target)
end

function check_robot_arm_example(example)
    result = example.result
    plan = result.plan
    isfinite(result.cost) || error("The planner did not return a finite cost.")
    all(isfinite, plan.q_ref) || error("The joint reference contains nonfinite values.")
    all(diff(plan.t_ref_s) .> 0) || error("Reference times must increase.")
    plan.q_ref[:, 1] == example.q_start || error("The reference lost its starting pose.")
    norm(plan.ee_ref[:, end] - example.target) <= example.planner.ik_tol_m ||
        error("The end-effector reference did not reach the requested IK tolerance.")
    return nothing
end

if abspath(PROGRAM_FILE) == @__FILE__
    example = robot_arm_example()
    check_robot_arm_example(example)
    plan = example.result.plan
    println("reference_samples=", length(plan.t_ref_s))
    println("reference_duration_s=", plan.t_ref_s[end])
    println("final_reference_error_m=", plan.final_error_m)
    println("planning_cost=", example.result.cost)
end
