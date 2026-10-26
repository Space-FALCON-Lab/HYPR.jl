# Plan a robot-arm reference

This walkthrough moves a three-joint arm toward a reachable Cartesian target.
HYPR searches joint-space paths and returns a reference indexed by time; you then
check the reference's start, its timing and its final end-effector error. The
example runs no actuator controller and no contact simulation.

## Prepare the environment

Follow [Install with SpaceAGORA](integration.md#Install-with-SpaceAGORA) first;
it creates `/path/to/hypr-env` with SpaceAGORA and the HYPR revision it pins.
Then, from a clone of this HYPR repository, run:

```sh
JULIA_LOAD_PATH='@:@stdlib' julia --startup-file=no --project=/path/to/hypr-env \
  examples/robot_arm_plan.jl
```

The example file comes from the clone, but the HYPR code that runs is the
version installed in the environment, which SpaceAGORA selected. The
[version table](integration.md#Compatibility-and-source-records) lists those
pairs.

## Define the arm and target

The example uses only SpaceAGORA's public interface:

```julia
using SpaceAGORA, HYPR
using Random

arm = default_robot_arm_model()
base = RobotArmBasePose([0.0, 0.0, 0.0])
q_start = [0.0, 0.3, -0.3]
target = robot_arm_fk(arm, base, [0.6, -0.4, 0.4]).end_effector_position
```

Joint angles are in radians and Cartesian positions in metres. The base keeps its
default orientation, so the target is in the same world frame as the base
position. Building the target from another joint pose guarantees that this model
can reach it. The planner still solves the inverse kinematics from `q_start`: it
receives only the Cartesian target, not the joints used to build it.

## Plan the motion

```julia
planner = RobotArmPlannerConfig(dt_s=0.1, duration_s=2.0)
search = RobotArmHYPRConfig(n_particles=8, n_iters=8, n_waypoints=1,
    n_samples=32, rrt_warmstart_enable=false, retime_enable=false)
result = plan_robot_arm_motion_hypr(arm, base, q_start, target;
    planner_config=planner, hypr_config=search, rng=MersenneTwister(741))
```

The small swarm keeps the example quick. It searches for one intermediate
joint-space waypoint, then samples the resulting path onto the requested
two-second reference. Retiming is turned off, so the nominal duration is kept, and
there are no obstacles. These settings are for the example only, not recommended
limits for a real arm; an application needs its own model, obstacle geometry, and
validated search and retiming settings.

## Read the result

`result.plan` holds the reference. Each column of `q_ref` is the joint
configuration at the matching time in `t_ref_s`, and `ee_ref` holds the
end-effector positions. The script checks that the results are finite, the times
increase, the reference starts at `q_start`, and the final Cartesian error is
within the planner's inverse-kinematics tolerance (`ik_tol_m`). It then prints:

| Output | Interpretation |
| --- | --- |
| `reference_samples` | Number of stored reference times; 21 for this example |
| `reference_duration_s` | Planned duration; 2 seconds here |
| `final_reference_error_m` | Distance between the reference endpoint and the requested target |
| `planning_cost` | The configured search objective, not measured actuator energy or propellant |

`result.components` and `result.cost_history` show the parts of the objective and
how the search progressed. A finite cost does not mean the motion can be executed.
With no obstacles, the example tests no collision avoidance, and a reference that
reaches the target says nothing yet about tracking error, contact forces or base
disturbance during execution.

Use the `RobotArm*` names for SpaceAGORA's robot model. The older `Cloth*` names
for the same compliant multibody services remain in the adapter contracts (1.0.0
through 3.0.0) for compatibility; the new names do not add MuJoCo contacts. Before
extending this example, read SpaceAGORA's
[public API policy](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/v0.7.1/docs/src/public_api_policy.md#compliant-multibody-and-robot-arm-names)
and the [validation guide](validation.md).
