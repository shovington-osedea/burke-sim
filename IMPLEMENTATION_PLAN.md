# Milestone 1 Implementation Plan: Keyboard-Controlled Robot Base

## Objective

Create the smallest useful Burk-e Gazebo simulation:

- an empty world with a ground plane and light;
- a simplified MiR1350-style differential-drive base;
- ROS 2 velocity commands on `/cmd_vel`;
- keyboard teleoperation; and
- odometry plus a headless motion smoke test.

This milestone is complete when a developer can launch the simulation, drive
the base with the keyboard, stop it predictably, and verify basic movement with
an automated test.

## Technical Baseline

Use this baseline unless the project owner explicitly changes it:

- Ubuntu 24.04 as the execution environment.
- ROS 2 Jazzy.
- Gazebo Harmonic.
- `colcon` with `ament_cmake` ROS packages.
- `ros_gz_sim` to launch and spawn models.
- `ros_gz_bridge` for ROS–Gazebo topics.
- `robot_state_publisher` and Xacro for the robot description.
- Gazebo's differential-drive system for base motion.
- `teleop_twist_keyboard` for manual control.

The repository host may be macOS, but the supported simulation environment for
this milestone is Ubuntu 24.04. Use a Linux VM or an approved container setup
when Ubuntu is not the host. Do not add a separate native-macOS architecture.

## Scope Boundaries

In scope:

- Reproducible ROS/Gazebo environment.
- Empty Gazebo world.
- Primitive-geometry mobile base.
- Two driven wheels and passive supports.
- `/cmd_vel`, `/odom`, TF, keyboard teleoperation, and smoke testing.

Out of scope:

- LiftKit, UR8L, payload, cameras, or other sensors.
- Aircraft model and inspection behavior.
- Nav2, SLAM, localization, or autonomous planning.
- Detailed MiR visual meshes.
- Real-device APIs, credentials, networking, or safety behavior.
- A custom base controller when Gazebo's differential-drive system is enough.

## Target Repository Layout

Use this structure unless an earlier task establishes an equivalent conventional
ROS layout:

```text
ros_ws/
└── src/
    └── burke-sim/                # This repository
        ├── AGENTS.md
        ├── IMPLEMENTATION_PLAN.md
        ├── README.md
        ├── compose.yaml          # Only if Task 0 selects a container runtime
        ├── docker/               # Only if Task 0 selects a container runtime
        ├── burke_description/
        │   ├── CMakeLists.txt
        │   ├── package.xml
        │   ├── urdf/
        │   │   ├── burke_base.urdf.xacro
        │   │   └── components/
        │   └── rviz/             # Optional; not required for this milestone
        └── burke_gazebo/
            ├── CMakeLists.txt
            ├── package.xml
            ├── config/
            │   └── bridge.yaml
            ├── launch/
            │   ├── empty_world.launch.py
            │   └── base_sim.launch.py
            ├── test/
            └── worlds/
                └── empty.sdf
```

Do not create empty placeholder directories. Add a directory only when its task
adds a real file that belongs there.

## Rules for Task Agents

Each agent should execute one task at a time.

1. Read `AGENTS.md`, this plan, and all files created by prerequisite tasks.
2. Confirm every prerequisite task is actually complete; do not rely only on
   checked boxes.
3. Limit changes to the task's allowed scope.
4. Do not add future-milestone functionality.
5. Use configurable Xacro properties for provisional physical values.
6. Mark assumed values as simulation assumptions in nearby documentation.
7. Run every available validation command listed for the task.
8. If a validation cannot run, report the exact missing dependency or
   environment condition. Do not claim it passed.
9. Update the task checkbox only after all acceptance criteria are met.
10. Leave a concise handoff containing changed files, validation evidence,
    assumptions, and remaining blockers.

## Dependency Order

```text
Task 0: Runtime environment
    ↓
Task 1: ROS workspace and packages
    ↓
Task 2: Empty Gazebo world
    ↓
Task 3: Simplified robot base
    ↓
Task 4: Differential drive and ROS bridge
    ↓
Task 5: Keyboard teleoperation and integrated launch
    ↓
Task 6: Automated smoke test and usage documentation
```

Do not parallelize Tasks 0–5. Task 6 depends on the integrated behavior from
all earlier tasks.

## Task 0 — Establish the Runtime Environment

- [x] Complete

### Goal

Provide a reproducible Ubuntu 24.04 environment containing ROS 2 Jazzy, Gazebo
Harmonic, and the milestone dependencies.

### Allowed Scope

- Root environment documentation.
- Container/VM support files if required for the chosen Ubuntu runtime.
- Dependency declarations and setup commands.
- No ROS packages, world files, or robot models yet.

### Required Dependencies

- ROS 2 Jazzy desktop or an equivalent package set containing required CLI and
  message packages.
- Gazebo Harmonic.
- `ros_gz`, `xacro`, `robot_state_publisher`, and `teleop_twist_keyboard`.
- `colcon` build tooling.

### Implementation Notes

- Prefer official binary packages.
- Pin the chosen ROS distribution and Gazebo release in project documentation.
- Because keyboard teleoperation requires an interactive terminal, ensure the
  selected runtime supports interactive `ros2 run` commands.
- If a container is selected, document how Gazebo's GUI is displayed. A
  headless-only container is insufficient for the manual milestone demo.
- Do not depend on host-global shell modifications that are absent in a clean
  terminal.

### Acceptance Criteria

- A clean supported environment can run `ros2 --help`.
- `gz sim --versions` or the supported equivalent reports Gazebo Harmonic.
- ROS can locate `ros_gz_sim`, `ros_gz_bridge`, `xacro`,
  `robot_state_publisher`, and `teleop_twist_keyboard`.
- Gazebo can open its GUI in the selected environment.
- Setup and entry commands are documented in the root `README.md`.

### Validation

```bash
ros2 pkg prefix ros_gz_sim
ros2 pkg prefix ros_gz_bridge
ros2 pkg prefix xacro
ros2 pkg prefix robot_state_publisher
ros2 pkg prefix teleop_twist_keyboard
gz sim --versions
```

### Handoff

State whether the runtime is a VM, container, or native Ubuntu environment and
record the exact command used to enter it.

## Task 1 — Scaffold the ROS Workspace

- [x] Complete

### Prerequisite

Task 0.

### Goal

Treat the parent `ros_ws/` directory as the ROS workspace and scaffold this
repository, already located at `ros_ws/src/burke-sim/`, with separate
description and simulation packages directly beneath it.

### Allowed Scope

- `burke_description/`
- `burke_gazebo/`
- Root build instructions in `README.md`
- No world content beyond package placeholders required for installation.
- No robot links, joints, plugins, or controllers.

### Package Responsibilities

`burke_description` owns:

- Xacro/URDF robot descriptions;
- robot geometry and kinematic frames; and
- future description assets.

`burke_gazebo` owns:

- worlds;
- simulation launch files;
- Gazebo-specific configuration;
- ROS–Gazebo bridge configuration; and
- simulation integration tests.

### Acceptance Criteria

- Both packages contain valid `package.xml` and `CMakeLists.txt` files.
- Runtime resources are installed into each package's share directory.
- A clean workspace builds successfully.
- Both packages are discoverable after sourcing the workspace.
- Package dependencies are minimal and explicit.

### Validation

Run from the workspace root `ros_ws/` (the parent of this repository):

```bash
colcon build --symlink-install
source install/setup.bash
ros2 pkg prefix burke_description
ros2 pkg prefix burke_gazebo
```

### Handoff

List the package responsibilities, declared dependencies, and build result.

## Task 2 — Launch an Empty Gazebo World

- [ ] Complete

### Prerequisite

Task 1.

### Goal

Launch a deterministic empty Gazebo world through ROS 2.

### Allowed Scope

- `burke_gazebo/worlds/empty.sdf`
- `burke_gazebo/launch/empty_world.launch.py`
- Package installation rules and focused tests/documentation.
- No robot spawning.

### World Requirements

- Ground plane.
- Directional light or sun.
- Earth gravity.
- Explicit physics engine/update settings.
- A useful initial GUI camera pose when supported.
- A stable world name used by later tasks.

### Acceptance Criteria

- One ROS launch command starts Gazebo with the correct world.
- The ground and lighting are visible.
- The world can pause, resume, reset, and close cleanly.
- The launch file resolves the installed world path rather than relying on the
  current working directory.
- No missing-resource or plugin errors appear during a normal launch.

### Validation

Run after building and sourcing the workspace:

```bash
ros2 launch burke_gazebo empty_world.launch.py
```

Also run a bounded headless startup if supported by the launch interface. The
agent must record the exact command because headless flags vary by integration
version.

### Handoff

Record the world name, launch command, physics choice, and GUI/headless results.

## Task 3 — Model the Simplified MiR1350 Base

- [ ] Complete

### Prerequisite

Task 2.

### Goal

Spawn a stable, primitive-geometry mobile base with the correct future-facing
frame and joint structure.

### Allowed Scope

- `burke_description/urdf/`
- Description package install rules.
- A spawn extension to `base_sim.launch.py` or a focused spawn launch file.
- Description validation tests.
- No drive plugin or ROS–Gazebo topic bridge.

### Required Frame and Link Structure

Minimum structure:

```text
base_footprint
└── base_link
    ├── left_drive_wheel_link
    ├── right_drive_wheel_link
    └── passive support links, if modeled as links
```

Use these joint names unless Gazebo integration requires a documented change:

- `left_drive_wheel_joint`
- `right_drive_wheel_joint`

### Modeling Requirements

- Use boxes, cylinders, and spheres rather than detailed meshes.
- Represent the base as a non-holonomic differential-drive platform.
- Use two driven wheels and enough passive support geometry to keep the chassis
  stable.
- Define visual, collision, and inertial properties.
- Keep wheel radius, wheel separation, chassis dimensions, ground clearance,
  mass, and caster properties as named Xacro properties.
- Make the forward direction `+X`, left `+Y`, and up `+Z`.
- Place `base_footprint` on the ground projection and `base_link` at the
  physical chassis reference.
- Choose conservative friction/contact values and document them as simulation
  assumptions.

### Acceptance Criteria

- Xacro expands into valid URDF.
- Gazebo spawns exactly one base at a named initial pose.
- The base rests on the ground without falling through, exploding, tipping, or
  continuously drifting.
- Wheel joints rotate about the expected axes.
- `robot_state_publisher` publishes the expected link tree.
- Collision geometry visibly matches the simplified robot footprint when
  collision visualization is enabled.
- Every provisional physical value is centralized and labeled as an
  assumption.

### Validation

```bash
xacro <path-to>/burke_base.urdf.xacro > /tmp/burke_base.urdf
check_urdf /tmp/burke_base.urdf
ros2 launch burke_gazebo base_sim.launch.py
ros2 run tf2_tools view_frames
```

If `check_urdf` or `tf2_tools` is not installed by Task 0, add the corresponding
runtime dependency or record a justified alternative.

### Handoff

Report the link/joint tree, assumed dimensions and inertial values, spawn pose,
and stability observation duration.

## Task 4 — Add Differential Drive and ROS–Gazebo Bridging

- [x] Complete

### Prerequisite

Task 3.

### Goal

Move the base with standard ROS `geometry_msgs/msg/Twist` commands and expose
odometry to ROS.

### Allowed Scope

- Gazebo differential-drive configuration in the robot description.
- `burke_gazebo/config/bridge.yaml`
- Integration launch changes.
- Focused command/odometry tests.
- No keyboard teleoperation yet.

### Interface Contract

ROS-facing topics:

- Command input: `/cmd_vel` using `geometry_msgs/msg/Twist`.
- Odometry output: `/odom` using `nav_msgs/msg/Odometry`.
- TF output should provide the appropriate `odom` to base transform if the
  selected differential-drive configuration publishes it.

Gazebo transport topic names may remain model-scoped internally. Hide them
behind `ros_gz_bridge` so later ROS components use the stable ROS-facing names.

### Configuration Requirements

- Bind the correct left and right wheel joints.
- Derive wheel radius/diameter and separation from the same description values
  used by the model; do not create conflicting unexplained copies.
- Set conservative maximum linear/angular velocity and acceleration values.
- Configure odometry frequency and frames explicitly.
- Use reliable command transport where the selected bridge/plugin requires it.
- Confirm the angular sign convention with a visible test.

### Acceptance Criteria

- A positive `linear.x` command moves the robot forward along its local `+X`.
- Positive and negative `angular.z` rotate in opposite directions with the
  expected ROS convention.
- A zero `Twist` stops commanded motion.
- `/odom` changes consistently with straight and rotational motion.
- No command is required on an internal Gazebo topic from the operator side.
- The base does not exhibit unacceptable wheel slip, chatter, or oscillation
  during the bounded manual tests.

### Validation

With `base_sim.launch.py` running:

```bash
ros2 topic info /cmd_vel --verbose
ros2 topic echo /odom
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.4}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

The agent may use a short publication rate instead of `--once` if the plugin
has an intentional command timeout. Record the exact bounded commands used.

### Handoff

Report ROS and Gazebo topic mappings, velocity limits, odometry frames, QoS,
and the observed straight/rotation results.

## Task 5 — Add Keyboard Teleoperation and Integrated Launch

- [x] Complete

### Prerequisite

Task 4.

### Goal

Provide the complete manual milestone workflow: launch the world and base, then
drive it from an interactive keyboard terminal.

### Allowed Scope

- `burke_gazebo/launch/base_sim.launch.py`
- Optional teleoperation parameter/configuration files.
- Root usage documentation.
- No custom keyboard node.

### Launch Requirements

The integrated simulation launch should start:

- Gazebo with `empty.sdf`;
- robot description publication;
- base spawning; and
- required ROS–Gazebo bridges.

Run `teleop_twist_keyboard` separately because it needs direct terminal input.
Remap its output only if the stable ROS-facing command topic differs from
`/cmd_vel`.

### Acceptance Criteria

- One command launches the complete base simulation.
- A second documented command starts interactive keyboard control.
- Keyboard commands drive forward, backward, left, and right.
- The keyboard stop command stops the robot.
- Exiting teleoperation is followed by an explicit zero command or another
  documented mechanism that prevents unintended continued motion.
- Initial keyboard speed and turn rate are conservative.
- Relaunching after shutdown does not leave duplicate nodes or models.

### Manual Validation

Terminal 1:

```bash
ros2 launch burke_gazebo base_sim.launch.py
```

Terminal 2:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p speed:=0.2 -p turn:=0.4
```

If remapping is required, append:

```text
--remap cmd_vel:=/cmd_vel
```

Validate forward, reverse, left turn, right turn, stop, and clean shutdown.

### Handoff

The integrated launch command is `ros2 launch burke_gazebo base_sim.launch.py`;
the second-terminal teleoperation command uses `speed:=0.2` and `turn:=0.4`.
The manual keys exercised by the documented workflow are forward (`i`),
reverse (`,`), left (`j`), right (`l`), and stop (`k`). After exiting
teleoperation, operators send an explicit zero `Twist`; `Ctrl-C` in the launch
terminal then shuts down Gazebo, the spawned model, and bridge processes.
Keyboard control requires an interactive terminal and the supported Ubuntu
24.04 / ROS 2 Jazzy runtime.

## Task 6 — Add the Motion Smoke Test and Final Documentation

- [ ] Complete

### Prerequisite

Task 5.

### Goal

Prove base spawning, command bridging, motion, stop behavior, and odometry in a
repeatable headless test.

### Allowed Scope

- `burke_gazebo/test/`
- Package test dependencies and CMake registration.
- Root `README.md` troubleshooting and usage updates.
- Small corrections to Tasks 2–5 only when required to make their documented
  interfaces testable.

### Test Scenario

The test must:

1. Start Gazebo headlessly with the empty world.
2. Spawn the Burk-e base.
3. Wait for `/cmd_vel` subscription and `/odom` publication with bounded
   timeouts.
4. Record the initial odometry pose.
5. Publish a low forward velocity for a bounded duration.
6. Publish zero velocity.
7. Confirm forward displacement exceeds a small tolerance.
8. Confirm lateral drift and unexpected yaw remain within documented generous
   simulation tolerances.
9. Confirm odometry speed settles near zero within a timeout.
10. Shut down all launched processes cleanly.

Do not assert exact floating-point poses or real MiR performance. The test is a
simulation integration check, not a hardware-fidelity test.

### Acceptance Criteria

- The headless smoke test passes repeatedly in the supported Ubuntu
  environment.
- Every wait and process has a bounded timeout.
- Failure output distinguishes launch, spawn, bridge, odometry, motion, and
  stop failures.
- Root documentation contains setup, build, manual launch, teleoperation,
  headless test, and common troubleshooting commands.
- The manual GUI workflow still works after the test is added.

### Validation

Run from the workspace root `ros_ws/` (the parent of this repository):

```bash
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select burke_description burke_gazebo \
  --event-handlers console_direct+
colcon test-result --verbose
```

Then repeat the Task 5 manual GUI test.

### Handoff

Report test names, timeouts, movement tolerances, repeat count, full test
result, and manual regression result.

## Milestone 1 Definition of Done

All of the following must be true:

- [ ] The supported Ubuntu 24.04 environment is reproducible.
- [ ] ROS 2 Jazzy and Gazebo Harmonic versions are documented.
- [ ] The ROS workspace builds from a clean shell.
- [ ] The empty Gazebo world launches in GUI and headless modes.
- [ ] The simplified Burk-e base spawns and remains physically stable.
- [ ] `/cmd_vel` controls forward, reverse, and rotational motion.
- [ ] `/odom` reports movement using documented frames.
- [ ] Keyboard teleoperation works from a second terminal.
- [ ] Stop and shutdown behavior are predictable.
- [ ] The headless motion smoke test passes repeatedly.
- [ ] All provisional physical values are clearly labeled as assumptions.
- [ ] No out-of-scope robot subsystems or real-hardware integrations were
      introduced.

## Deferred Follow-Up Milestones

After Milestone 1, plan these separately:

1. Improve base geometry and motion fidelity.
2. Add basic obstacle sensing.
3. Add the LiftKit prismatic structure.
4. Add the UR8L model and controllers.
5. Add the inspection payload and simulated sensors.
6. Add an aircraft model and predefined inspection stations.
7. Add autonomous station navigation and simplified inspection execution.
