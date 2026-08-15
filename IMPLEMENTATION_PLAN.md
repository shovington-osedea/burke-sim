# Aircraft-Relative MiR Path Following Implementation Plan

## Objective

Build the first autonomous-motion behavior for the Burk-e simulation:

> Make the differential-drive MiR complete one clockwise, closed 2D loop whose
> centreline remains 1 m outside the aircraft geometry, using wheel odometry
> only and a fixed path expressed in an aircraft-relative frame.

The behavior must not use Gazebo model pose, world pose, Nav2, SLAM, an
occupancy map, or obstacle-aware replanning. The design must allow a future
aircraft-localization node to replace the manual aircraft transform without
changing the path publisher or follower.

## Confirmed Decisions

- Implementation language: Python.
- New ROS package: `aircraft_navigation`, using `ament_python`.
- Robot pose frame: existing `odom -> base_footprint` wheel-odometry TF.
- Aircraft frame origin: projected centre of the aircraft.
- Aircraft frame orientation: `+X` points toward the aircraft nose, `+Y` points
  to the aircraft's left, and `+Z` points up.
- Current nominal aircraft pose: centred near `(0, 12)` in the existing world.
  Because the current mesh nose points toward negative `odom` X, the provisional
  manual transform is approximately `odom -> aircraft = (x=0, y=12, yaw=pi)`.
  Task 2 must verify and freeze the exact value from configuration.
- Path direction: clockwise when viewed from above.
- Path clearance: path centreline must remain at least `1.0 m` outside the
  aircraft collision geometry projected onto the ground plane.
- First path point: at the aircraft nose.
- Initial robot placement: beside the first nose waypoint and aligned with the
  clockwise path tangent, so this milestone tests following rather than path
  acquisition from the current world origin.
- Runtime behavior: opt-in through a separate
  `fixed_perimeter_follow.launch.py`; normal `base_sim.launch.py` must not start
  autonomous motion.
- First execution: complete exactly one loop, publish zero velocity, and stop.

All geometric values derived from the imported aircraft mesh remain simulation
assumptions. Do not present them as surveyed aircraft dimensions.

## Existing Baseline to Preserve

The repository already provides:

- ROS 2 Jazzy and Gazebo Harmonic integration;
- the `burke_description` and `burke_gazebo` packages;
- a differential-drive MiR base with `+X` forward, `+Y` left, and `+Z` up;
- `/cmd_vel` as `geometry_msgs/msg/Twist`;
- `/odom` as `nav_msgs/msg/Odometry`;
- `/tf` containing `odom -> base_footprint`;
- a static Challenger aircraft model in `burke_empty`;
- independent LiftKit and UR8 Long controls;
- a fixed front-deck 3D lidar; and
- optional Foxglove visualization.

The new work must preserve those interfaces. It must not modify the base drive
plugin, replace wheel odometry, or require the lidar for this milestone.

## Architecture

```text
aircraft_pose.yaml
        |
        v
manual aircraft frame publisher
        |
        v
odom -----------------------> aircraft
 |                               |
 |                               v
 |                      perimeter_path.yaml
 |                               |
 v                               v
base_footprint             /aircraft_path
        \                       /
         \                     /
          v                   v
             pure pursuit
                  |
                  v
               /cmd_vel
                  |
                  v
        Gazebo differential drive
                  |
                  v
           wheel odometry only
```

The path follower may consume `/odom` for timestamps and velocity, but it must
obtain the robot pose and path transforms through TF. It must never subscribe
to Gazebo model-state, pose, or ground-truth topics.

## Frame Contract

Required TF relationships:

```text
odom
├── base_footprint
└── aircraft
```

- `odom -> base_footprint` is produced by the existing differential-drive wheel
  odometry.
- `odom -> aircraft` is initially published from `aircraft_pose.yaml`.
- The path is always stored and published in `aircraft`.
- The follower transforms path targets into `odom`, then transforms the active
  lookahead target into `base_footprint` for curvature calculation.
- No `world`, Gazebo entity, or ground-truth frame may be required by the
  follower.

## ROS Interface Contract

Required inputs:

| Topic / TF | Type | Purpose |
| --- | --- | --- |
| `/odom` | `nav_msgs/msg/Odometry` | Wheel-odometry observation and velocity evidence |
| `/aircraft_path` | `nav_msgs/msg/Path` | Closed path with `header.frame_id=aircraft` |
| `odom -> base_footprint` | TF | Robot pose estimate |
| `odom -> aircraft` | TF | Manual aircraft pose, later replaceable by localization |

Required output:

| Topic | Type | Constraint |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Only `linear.x` and `angular.z` may be non-zero |

Required debug outputs:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/path_follower/lookahead` | `geometry_msgs/msg/PointStamped` | Active lookahead target |
| `/path_follower/closest_point` | `geometry_msgs/msg/PointStamped` | Current closest point on the tracked path |
| `/path_follower/progress` | `std_msgs/msg/Float64` | Monotonic completed-loop fraction in `[0,1]` |
| `/path_follower/cross_track_error` | `std_msgs/msg/Float64` | Current path error in metres |

Publish `/aircraft_path` with transient-local durability so RViz and late
subscribers receive the fixed path without requiring republishing.

## Target Package Layout

```text
aircraft_navigation/
├── aircraft_navigation/
│   ├── __init__.py
│   ├── aircraft_frame_publisher.py
│   ├── path_geometry.py
│   ├── perimeter_path_publisher.py
│   ├── pure_pursuit.py
│   └── pure_pursuit_follower.py
├── config/
│   ├── aircraft_pose.yaml
│   ├── controller.yaml
│   └── perimeter_path.yaml
├── launch/
│   └── fixed_perimeter_follow.launch.py
├── resource/
│   └── aircraft_navigation
├── rviz/
│   └── perimeter_debug.rviz
├── test/
│   ├── test_aircraft_frame.py
│   ├── test_path_geometry.py
│   ├── test_pure_pursuit.py
│   ├── test_navigation_interfaces.py
│   └── test_perimeter_loop.py
├── package.xml
├── setup.cfg
└── setup.py
```

Add files only in the task that owns them. Do not create empty placeholders.

## Initial Controller Profile

Use the following as named, validated starting values in `controller.yaml`:

```yaml
control_rate_hz: 20.0
lookahead_distance_m: 1.0
nominal_linear_speed_mps: 0.3
maximum_linear_speed_mps: 0.5
minimum_linear_speed_mps: 0.05
maximum_angular_speed_radps: 0.5
maximum_linear_acceleration_mps2: 0.4
maximum_angular_acceleration_radps2: 0.8
completion_tolerance_m: 0.2
forward_search_window_points: 8
odom_timeout_s: 0.5
tf_timeout_s: 0.2
```

These are simulation tuning values from the supplied behavior plan, not MiR
hardware limits. Task 5 may tune them, but every change must be justified by
recorded metrics and retained in configuration rather than hidden in code.

## Controller Behavior

At each control cycle:

1. Confirm fresh `/odom`, valid TF, and a valid closed path.
2. Transform path geometry from `aircraft` into `odom` using the current
   `odom -> aircraft` transform.
3. Find the closest point or segment only within a bounded forward window from
   the retained progress position.
4. Advance along path arc length by the configured lookahead distance.
5. Transform the selected target into `base_footprint`.
6. For lookahead coordinates `(x_L, y_L)`, calculate:

   ```text
   L_d² = x_L² + y_L²
   curvature = 2 * y_L / L_d²
   angular_velocity = linear_velocity * curvature
   ```

7. Reduce linear speed as absolute curvature increases.
8. Clamp linear/angular velocity and acceleration to the active profile.
9. Publish a Twist with only `linear.x` and `angular.z` populated.
10. Publish debug targets, progress, and cross-track error.
11. Detect completion only after monotonic progress traverses one full loop and
    returns within the configured completion tolerance.
12. Publish an explicit zero Twist and remain stopped after completion.

The controller must also publish zero velocity when odometry is stale, TF is
missing, the path is invalid, shutdown begins, or an unhandled exception
escapes the control update.

## Agent Working Rules

Each implementation agent must:

1. Read `AGENTS.md`, this plan, and all prerequisite task handoffs.
2. Execute one task only.
3. Preserve all existing base, arm, lift, lidar, Foxglove, bridge, and world
   interfaces unless the task explicitly permits a narrow integration edit.
4. Use ROS simulation time for every navigation node.
5. Keep geometry and control math in pure Python functions with no ROS side
   effects so unit tests can exercise it directly.
6. Use the system Python associated with ROS 2 Jazzy; do not introduce Conda or
   an incompatible virtual environment.
7. Add bounded timeouts to all TF waits, topic waits, controller loops, launch
   tests, and cleanup.
8. Never validate path following from Gazebo world/model pose.
9. Update a task checkbox only after its acceptance criteria and available
   validation pass.
10. Leave a handoff with changed files, exact commands, results, tuning values,
    assumptions, and unresolved blockers.

## Stop-and-Ask Conditions

Stop and ask the project owner before continuing if:

- the imported aircraft nose or centre cannot be verified from the current
  model configuration;
- the verified `odom -> aircraft` transform differs materially from the
  provisional `(0, 12, pi)` contract;
- the 1 m path centreline cannot be kept outside aircraft collision geometry;
- the MiR footprint collides with the aircraft while its centre follows the
  requested 1 m-clearance path;
- the initial robot placement cannot be made beside the nose waypoint without
  changing the normal `base_sim.launch.py` default;
- wheel odometry or `odom -> base_footprint` is unavailable or requires Gazebo
  ground truth;
- another `/cmd_vel` publisher remains active during autonomous following;
- the lift cannot remain collapsed or the arm cannot be placed in its
  documented stow pose before base motion;
- a proposed fix would add Nav2, SLAM, a map, ground-truth pose, or obstacle
  replanning; or
- completion requires changing an existing public topic or frame contract.

## Dependency Order

```text
Task 1: Package and baseline odometry proof
    ↓
Task 2: Aircraft frame and navigation spawn contract
    ↓
Task 3: Clockwise 1 m perimeter path
    ↓
Task 4: Pure Pursuit math and unit tests
    ↓
Task 5: Follower node, safety, progress, and metrics
    ↓
Task 6: Opt-in integrated launch
    ↓
Task 7: Full-loop headless test and operator documentation
```

Tasks are intentionally sequential. Do not parallelize work that changes the
same package or relies on unverified geometry/TF decisions.

## Task 1 — Scaffold the Package and Prove the Odometry Baseline

- [ ] Complete

### Goal

Create the Python package and prove the existing wheel-odometry contract before
adding autonomous behavior.

### Allowed Scope

- New `aircraft_navigation` package metadata and Python module scaffold.
- Package-level lint/test configuration.
- A focused baseline test or diagnostic script.
- No aircraft TF, path, controller, RViz configuration, or `/cmd_vel` output.

### Work

- Create an `ament_python` package with explicit dependencies on `rclpy`,
  `geometry_msgs`, `nav_msgs`, `std_msgs`, `tf2_ros`, and ROS launch tooling.
- Confirm `/odom` uses `frame_id=odom` and `child_frame_id=base_footprint`.
- Confirm TF contains `odom -> base_footprint` and tracks the same translation
  and yaw as `/odom` while keyboard commands move the base.
- Confirm no Gazebo pose/model-state topic is needed for this observation.
- Add standard Python lint tests supported by the ROS environment.

### Acceptance Criteria

- `colcon build --symlink-install --packages-select aircraft_navigation`
  passes.
- The installed package is discoverable with `ros2 pkg prefix`.
- A bounded manual or automated check proves X/Y/yaw changes consistently in
  `/odom` and TF.
- The package contains no publisher for `/cmd_vel` yet.
- No ground-truth dependency is declared or consumed.

### Validation

```bash
colcon build --symlink-install --packages-select aircraft_navigation
source install/setup.bash
ros2 pkg prefix aircraft_navigation
ros2 launch burke_gazebo base_sim.launch.py gui:=false foxglove:=false
ros2 topic echo --once /odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

### Handoff

Record the observed odometry frame IDs, TF relationship, update behavior, and
proof that no ground-truth topic was consumed.

## Task 2 — Define the Aircraft Frame and Nose-Start Spawn Contract

- [ ] Complete

### Prerequisite

Task 1.

### Goal

Publish the manually configured aircraft frame and define a navigation-only
spawn pose beside the future nose waypoint.

### Allowed Scope

- `aircraft_navigation/config/aircraft_pose.yaml`
- `aircraft_frame_publisher.py`
- Focused frame tests.
- Launch arguments or a narrow reusable spawn-pose interface in
  `burke_gazebo/launch/base_sim.launch.py`.
- Normal base-simulation defaults must remain unchanged.
- No path or controller.

### Work

- Verify the aircraft projected centre and nose direction from the current
  world configuration and imported collision mesh.
- Define `aircraft` at the projected aircraft centre with `+X` toward the nose.
- Publish `odom -> aircraft` with a `StaticTransformBroadcaster` from validated
  YAML configuration.
- Freeze the exact provisional translation/yaw after verifying how wheel
  odometry initializes when the base is spawned away from the world origin.
- Add optional spawn arguments to the existing base launch only if needed;
  preserve `(0,0,0)` as its default.
- Define a navigation spawn contract that will place `base_footprint` beside P0
  and align the MiR with the clockwise tangent once P0 exists in Task 3.
- Do not read the Gazebo aircraft entity pose at runtime.

### Acceptance Criteria

- `odom -> aircraft` is available from configuration with simulation time.
- The aircraft frame origin and axes match the confirmed centre/nose contract.
- Changing YAML aircraft pose moves the TF without changing publisher code.
- The normal base launch still spawns at its original default pose.
- The navigation spawn can be configured without editing the world SDF.
- No path or follower depends on how the aircraft transform is produced.

### Validation

```bash
ros2 run tf2_ros tf2_echo odom aircraft
ros2 run tf2_ros tf2_echo aircraft base_footprint
```

### Handoff

Record the exact transform, axis convention, spawn interface, odometry-origin
behavior, and any remaining simulation assumptions.

## Task 3 — Define and Publish the Clockwise 1 m Perimeter Path

- [x] Complete

### Prerequisite

Task 2.

### Goal

Create a validated closed path in `aircraft` that starts at the nose, follows
the concave top-down aircraft footprint, and remains within the configured
1.0–1.5 m clearance band.

### Allowed Scope

- `perimeter_path.yaml`
- `path_geometry.py`
- `perimeter_path_publisher.py`
- Pure path/configuration tests.
- No controller or `/cmd_vel` publication.

### Work

- Project every collision-mesh triangle onto aircraft XY, rasterize the union,
  and extract its exterior contour without replacing it with a convex hull.
- Generate a circular buffered perimeter from that concave footprint using a
  1.2 m construction radius, then validate the resulting path against the
  stored footprint.
- Generate enough ordered waypoints that consecutive poses are no more than
  1.0 m apart.
- Place P0 at the positive-X nose and orient each pose toward the next pose;
  the final pose points back to P0.
- Order points clockwise when viewed from `+Z`.
- Close the loop explicitly without creating a zero-length final segment.
- Preserve concave fuselage, wing, and tail transitions from the exterior
  union boundary; add intermediate points where raster buffering requires it.
- Validate finite values, unique consecutive points, non-zero segments,
  clockwise signed area, closure, 1.0–1.5 m clearance, and maximum 1.0 m pose
  spacing.
- Calculate the P0 tangent from P0 to P1 for the navigation spawn.
- Publish a transient-local `nav_msgs/msg/Path` on `/aircraft_path` with every
  pose in `aircraft`, planar unit orientation, and yaw toward the next pose.

### Acceptance Criteria

- The path contains non-degenerate ordered points with no consecutive spacing
  greater than 1.0 m.
- P0 is the nose waypoint.
- Signed geometry verifies clockwise order.
- The projected triangle union has a concave exterior footprint; no convex-hull
  bridging is used.
- Every path segment stays at least 1.0 m and no farther than 1.5 m from the
  projected collision footprint within an explicitly documented numerical
  tolerance.
- The path is closed and has a stable total arc length.
- `/aircraft_path` has `header.frame_id=aircraft` and transient-local QoS.
- Each pose orientation points toward the next waypoint, including closure.
- Path publication requires no Gazebo pose topic or Nav2 dependency.

### Validation

```bash
ros2 topic info --verbose /aircraft_path
ros2 topic echo --once /aircraft_path
```

The footprint and perimeter can be regenerated from the collision mesh with:

```bash
python3 aircraft_navigation/scripts/generate_perimeter_path.py \
  burke_description/cad/stl/challenger_collision.stl \
  aircraft_navigation/config/perimeter_path.yaml
```

### Handoff

Record the projected-union method, footprint and waypoint counts, total length,
clearance range, maximum pose spacing, signed area, P0 coordinates, P0 tangent
yaw, and confirmation that waypoint orientations follow the next pose.

## Task 4 — Implement Pure Pursuit as Tested Pure Python Logic

- [x] Complete

### Prerequisite

Task 3.

### Goal

Implement path projection, progress, lookahead selection, curvature, speed
selection, and rate limiting without ROS side effects.

### Allowed Scope

- `path_geometry.py`
- `pure_pursuit.py`
- Unit tests and fixtures.
- No ROS node, TF listener, launch changes, or `/cmd_vel` publisher.

### Work

- Represent the closed path by segments and cumulative arc length.
- Project a robot point onto candidate segments.
- Track continuous progress modulo total loop length.
- Search for the closest point only in a bounded forward window around retained
  progress; never globally jump backward near adjacent sections.
- Select a lookahead point by advancing configured arc length with wraparound.
- Transform target coordinates into the robot frame through pure 2D math used
  by unit tests.
- Implement curvature `2*y/L²` with protection for near-zero lookahead distance.
- Reduce linear speed as curvature grows, then clamp to configured min/max.
- Clamp angular speed and linear/angular acceleration per control step.
- Return an explicit zero command for invalid, non-finite, or degenerate input.

### Required Unit Cases

- Straight path and centred robot.
- Left and right curves with correct angular sign.
- Near-zero lookahead distance.
- Closed-loop wraparound at P0.
- Robot near two spatially close but distant-in-progress segments.
- Forward-window tracking without backward jumps.
- Curvature-based speed reduction.
- Linear and angular acceleration limiting.
- Non-finite input rejection.
- One-loop progress and completion threshold logic.

### Acceptance Criteria

- Pure unit tests require no ROS graph, Gazebo, or display.
- Identical inputs produce identical outputs.
- Only forward linear velocity and yaw rate are produced.
- Progress does not jump backward or skip across nearby path sections.
- Every configured bound and invalid-input branch is covered.

### Validation

```bash
colcon test --packages-select aircraft_navigation --event-handlers console_direct+
colcon test-result --verbose
```

### Handoff

Record the math API, progress representation, wraparound rules, test cases, and
coverage of all safety bounds.

## Task 5 — Implement the Pure Pursuit Follower Node

- [x] Complete

### Prerequisite

Task 4.

### Goal

Connect the tested controller logic to ROS topics and TF while remaining
fail-closed.

### Allowed Scope

- `pure_pursuit_follower.py`
- `controller.yaml`
- Node-level tests and debug topics.
- No integrated autonomous launch yet.

### Work

- Load and validate every controller parameter at node construction.
- Subscribe to `/odom` and transient-local `/aircraft_path`.
- Use `tf2_ros.Buffer` and `TransformListener` for `odom`, `aircraft`, and
  `base_footprint` transforms.
- Run at the configured 20 Hz simulation-time rate.
- Preserve progress across path updates only when path identity/geometry is
  unchanged; otherwise stop and reset deterministically.
- Publish `/cmd_vel`, lookahead, closest point, progress, and cross-track error.
- Populate only `linear.x` and `angular.z`; all other Twist fields remain zero.
- Publish zero on stale odometry, missing/stale TF, invalid path, completion,
  shutdown, and controller exceptions.
- Detect exactly one completed loop using retained continuous progress, not
  nearest-point coincidence at P0.
- Record maximum and RMS cross-track error, maximum heading error, progress
  regressions, command bounds, acceleration bounds, execution time, and final
  completion state in a final structured log summary.
- Refuse to command while another unexpected `/cmd_vel` publisher is active if
  reliable publisher discovery can enforce this without race-prone behavior;
  otherwise document the exclusive-publisher operational requirement and test
  it in Task 7.

### Acceptance Criteria

- The node publishes no non-zero command before odometry, path, and TF are
  ready.
- Normal output never exceeds configured velocity or acceleration bounds.
- Loss of any required input produces zero within a bounded interval.
- One-loop completion produces zero and cannot restart without relaunch/reset.
- Debug topics match the controller's internal active targets and errors.
- Node tests use synthetic messages/TF and require no Gazebo ground truth.

### Validation

```bash
ros2 run aircraft_navigation pure_pursuit_follower --ros-args \
  --params-file $(ros2 pkg prefix aircraft_navigation)/share/aircraft_navigation/config/controller.yaml
```

### Handoff

Record parameter validation, freshness rules, stop behavior, completion state
machine, debug topics, metrics, and node-test results.

## Task 6 — Add the Opt-In Fixed-Perimeter Launch

- [x] Complete

### Prerequisite

Task 5.

### Goal

Provide a separate launch that starts the existing simulation at the nose and
runs the manual aircraft frame, path publisher, RViz option, and follower.

### Allowed Scope

- `fixed_perimeter_follow.launch.py`
- Narrow reusable launch arguments in `base_sim.launch.py`.
- Package installation metadata.
- No changes to the normal base-simulation defaults.

### Work

- Include `burke_gazebo/base_sim.launch.py` with Foxglove optional and with the
  navigation-specific spawn pose beside P0.
- Start the aircraft-frame publisher and perimeter-path publisher.
- Start RViz only behind a launch argument.
- Keep the follower opt-in within this separate launch; normal base simulation
  remains manually controlled.
- Ensure LiftKit remains collapsed and place the UR arm in its documented stow
  pose before allowing non-zero base motion. Use existing command interfaces;
  do not add arm/lift orchestration architecture in this milestone.
- Confirm no keyboard teleoperation or other `/cmd_vel` publisher is running.
- Sequence startup with readiness checks or a small dedicated gate rather than
  an unexplained sleep.
- On shutdown, publish zero `/cmd_vel` and leave lift/arm in their safe state.

### Operator Command

```bash
ros2 launch aircraft_navigation fixed_perimeter_follow.launch.py
```

Required launch arguments:

- `gui` default `true`;
- `rviz` default `true`;
- `foxglove` default `false`;
- `autostart` default `true` for this dedicated launch;
- aircraft pose/config paths;
- path/controller config paths; and
- spawn pose derived from P0 and its clockwise tangent.

### Acceptance Criteria

- The dedicated launch starts one simulation, one robot, one aircraft-frame
  publisher, one path publisher, and one follower.
- The MiR appears beside the nose P0 with the expected orientation.
- `base_sim.launch.py` alone still uses its original pose and starts no
  navigation nodes.
- Launch startup has no fixed timing race.
- Shutdown sends zero and terminates all owned processes.
- Existing arm, lift, lidar, bridge, and optional Foxglove interfaces remain
  available.

### Handoff

Record the complete launch graph, resolved spawn pose, readiness sequence,
arguments, and regression results for normal base simulation.

## Task 7 — Validate One Full Clockwise Loop and Document Operation

- [ ] Complete

### Prerequisite

Task 6.

### Goal

Prove the requested behavior end to end using only wheel odometry for robot
state.

### Allowed Scope

- `test_navigation_interfaces.py`
- `test_perimeter_loop.py`
- Test registration/dependencies.
- `README.md` navigation usage and troubleshooting.
- Minimal fixes to Tasks 2–7 only when a test proves a defect.

### Interface Test Requirements

Verify with bounded waits:

1. `/odom` and `odom -> base_footprint` agree on robot motion.
2. `odom -> aircraft` matches configuration.
3. `/aircraft_path` is closed, clockwise, transient-local, and expressed in
   `aircraft`.
4. P0 and spawn pose are at the nose with the expected tangent alignment.
5. Exactly one intended autonomous `/cmd_vel` publisher is active.
6. Debug topics use the documented types and frames.
7. No node in `aircraft_navigation` subscribes to a Gazebo ground-truth topic.

### Full-Loop Test Requirements

Run headlessly and:

1. Start the dedicated launch with RViz and Foxglove disabled.
2. Wait for all required topics and TF with explicit timeouts.
3. Confirm lift collapsed and arm stowed before first non-zero base command.
4. Observe progress monotonically from near zero to one full loop.
5. Record cross-track error, heading error, commands, acceleration, execution
   time, and progress regressions.
6. Confirm every command stays within the active profile.
7. Confirm the robot completes the loop clockwise without collision or
   oscillatory reversal.
8. Confirm completion publishes zero and the base remains stopped for a
   bounded observation period.
9. Confirm the controller does not begin a second loop.
10. Shut down cleanly and retain actionable logs on failure.

The test may use `/odom`, TF, commands, progress, and debug topics. It must not
use Gazebo world/model pose to score tracking or completion.

### Tuning and Metric Acceptance

- Start with the controller profile in this plan.
- Tune only configuration values, not hidden literals.
- Record before/after metrics for every tuning change.
- Freeze final maximum/RMS cross-track and heading-error acceptance thresholds
  from the first stable baseline, with explicit rationale in the test and
  README.
- If a stable loop requires changing the requested 1 m aircraft clearance,
  stop and ask rather than changing the path silently.

### Acceptance Criteria

- One full clockwise loop completes from the nose start.
- The path centreline remains the validated 1 m from aircraft geometry.
- Robot state comes only from wheel odometry and TF.
- The controller stops after one loop and stays stopped.
- Nav2, SLAM, maps, lidar localization, and Gazebo ground truth are absent.
- Repeated headless runs pass with the frozen metric thresholds.
- Manual GUI/RViz validation shows the path and lookahead geometry correctly.
- Existing base, lift, arm, lidar, and Foxglove tests still pass.
- README documents build, launch, configuration, path editing, metrics,
  shutdown, and troubleshooting.

### Validation

Run from the outer ROS workspace root:

```bash
colcon build --symlink-install --packages-select \
  burke_description burke_gazebo aircraft_navigation
source install/setup.bash
colcon test --packages-select \
  burke_description burke_gazebo aircraft_navigation \
  --event-handlers console_direct+
colcon test-result --verbose
ros2 launch aircraft_navigation fixed_perimeter_follow.launch.py
```

### Handoff

Record repeated-run count, final parameters, metric thresholds/results,
completion time, stop observation, absence of ground-truth subscriptions,
manual RViz result, and all regression results.

## Milestone Definition of Done

- [ ] `odom -> base_footprint` is the only robot-pose source used by the
      follower.
- [ ] A configurable manual `odom -> aircraft` transform is published.
- [ ] The aircraft frame is centred with `+X` toward the nose.
- [ ] A 10–30 point clockwise closed path starts at the nose and maintains a
      validated 1 m centreline clearance.
- [ ] The path is published as transient-local `nav_msgs/msg/Path` in
      `aircraft`.
- [ ] The MiR spawns beside P0 aligned with the clockwise tangent only in the
      dedicated navigation launch.
- [ ] RViz shows frames, robot, path, closest point, and lookahead.
- [ ] The Python Pure Pursuit follower commands only `linear.x` and
      `angular.z`.
- [ ] Progress cannot jump backward across nearby path sections.
- [ ] Curvature-based speed reduction and configured rate limits are active.
- [ ] Missing/stale inputs, exceptions, shutdown, and completion produce an
      explicit zero command.
- [ ] The MiR completes exactly one clockwise loop and remains stopped.
- [ ] Tracking and motion metrics are recorded with frozen acceptance
      thresholds.
- [ ] Nav2, SLAM, occupancy maps, obstacle replanning, lidar localization, and
      Gazebo ground truth are not used.
- [ ] Replacing the manual aircraft-frame publisher later will not require
      changes to the path publisher or follower.
- [ ] Normal `base_sim.launch.py` remains manually controlled and retains its
      original defaults.

## Deferred Follow-Up

After this milestone, plan separately:

1. Use the front-deck 3D lidar and/or depth cameras to estimate aircraft pose.
2. Replace the manual `odom -> aircraft` publisher with localization output.
3. Recenter the existing aircraft-relative path as localization updates.
4. Add obstacle observation and bounded path replanning.
5. Convert selected path locations into inspection stations.
6. Coordinate base motion with LiftKit, UR8L, and payload inspection behavior.

## Reference Sources

- ROS 2 Jazzy Python package and launch conventions:
  <https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-system.html>
- ROS 2 Jazzy Python package development:
  <https://docs.ros.org/en/jazzy/How-To-Guides/Developing-a-ROS-2-Package.html>
- ROS 2 Jazzy Python environment guidance:
  <https://docs.ros.org/en/jazzy/How-To-Guides/Using-Python-Packages.html>
- ROS 2 Jazzy `nav_msgs`, including `Path` and `Odometry`:
  <https://docs.ros.org/en/jazzy/p/nav_msgs/README.html>
- ROS 2 Jazzy Python TF listener implementation:
  <https://docs.ros.org/en/jazzy/p/tf2_ros_py/_modules/tf2_ros/transform_listener.html>
