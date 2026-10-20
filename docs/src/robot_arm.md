# Plan a robot-arm reference

This walkthrough moves a three-joint arm toward a reachable Cartesian target.
HYPR searches joint-space paths and returns a time-indexed reference. You will
inspect that reference's start, timing and final end-effector error. The example
does not run an actuator controller or a contact simulation.

## Prepare the environment

Follow [Install with SpaceAGORA](integration.md#Install-with-SpaceAGORA) first.
It creates `/path/to/hypr-env` with SpaceAGORA and its pinned HYPR package.
From a checkout of this HYPR repository, run:

```sh
JULIA_LOAD_PATH='@:@stdlib' julia --startup-file=no --project=/path/to/hypr-env \
  examples/robot_arm_plan.jl
```

The example file is part of the v0.2.0 source; the v0.1.1 archive does not contain
it. The installed runtime remains the version selected by SpaceAGORA.
The [source records](integration.md#Compatibility-and-source-records) identify the
pair used to check this walkthrough.

## Define the arm and target

The executable example uses only the public SpaceAGORA interface:

```julia
using SpaceAGORA, HYPR
using Random

arm = default_robot_arm_model()
base = RobotArmBasePose([0.0, 0.0, 0.0])
q_start = [0.0, 0.3, -0.3]
target = robot_arm_fk(arm, base, [0.6, -0.4, 0.4]).end_effector_position
```

Joint angles are in radians and Cartesian positions are in metres. The default
base orientation is unchanged, so the target above is expressed in the same world
frame as the base position. Constructing the target from another joint pose makes
it reachable for this model. The planner still solves inverse kinematics from
`q_start`; it receives the Cartesian target, not the joints used to construct it.

## Plan the motion

```julia
planner = RobotArmPlannerConfig(dt_s=0.1, duration_s=2.0)
search = RobotArmHYPRConfig(n_particles=8, n_iters=8, n_waypoints=1,
    n_samples=32, rrt_warmstart_enable=false, retime_enable=false)
result = plan_robot_arm_motion_hypr(arm, base, q_start, target;
    planner_config=planner, hypr_config=search, rng=MersenneTwister(741))
```

The small swarm keeps this example quick. It searches one internal joint-space
waypoint, then samples the resulting path onto the requested two-second reference.
Retiming is explicitly disabled, so the nominal duration is retained. No obstacles
are supplied. These settings explain the example; they are not recommended motion
limits for a real arm. Use your own model, obstacle geometry and validated search
and retiming settings for an application.

## Read the result

`result.plan` holds the reference. Each column of `q_ref` is a joint configuration
at the corresponding time in `t_ref_s`; `ee_ref` holds the end-effector positions.
The executable example verifies finite results, increasing times, the original
starting pose and a final Cartesian error within the configured inverse-kinematics
tolerance. It prints these quantities:

| Output | Interpretation |
| --- | --- |
| `reference_samples` | Number of stored reference times; 21 for this example |
| `reference_duration_s` | Planned duration; 2 seconds here |
| `final_reference_error_m` | Distance between the reference endpoint and the requested target |
| `planning_cost` | The configured search objective, not measured actuator energy or propellant |

You can inspect `result.components` and `result.cost_history` to understand the
objective and search progress. A finite cost alone does not establish a feasible
executed motion. This obstacle-free example exercises no collision-avoidance
claim, and reaching the endpoint in the reference says nothing yet about tracking
error, contact forces or base disturbance during execution.

The preferred `RobotArm*` names describe SpaceAGORA's robot model. Older `Cloth*`
names remain in adapter contracts 1.0.0 and 2.0.0 for compatibility with the same compliant
multibody services. The name change does not add MuJoCo contacts. See the
[SpaceAGORA public API policy](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/CORE_V050_SHA/docs/src/public_api_policy.md#compliant-multibody-and-robot-arm-names)
and [validation guide](validation.md) before extending this example.
