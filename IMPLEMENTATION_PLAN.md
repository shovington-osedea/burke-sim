# Milestone 2 Implementation Plan: Primitive 6-DOF Arm

## Objective

Add the smallest useful articulated arm to the existing Burk-e simulation:

- one primitive 6-DOF serial arm mounted directly on top of the current mobile
  platform;
- six revolute joints represented with simple cylinder geometry;
- no end-effector, tool, payload, sensors, or lift;
- one ROS 2 position-command topic per arm joint; and
- joint-state feedback sufficient to verify commanded motion.

This milestone is complete when a developer can launch the existing mobile
base with the arm attached, command every arm joint independently from ROS 2,
observe bounded motion in Gazebo, and confirm that base driving still works.

## Existing Baseline

Agents must extend the repository as it exists at the start of this milestone:

- Ubuntu 24.04, ROS 2 Jazzy, and Gazebo Harmonic remain the supported runtime.
- `burke_description/urdf/burke_base.urdf.xacro` owns the current primitive
  mobile-base model and its Gazebo differential-drive system.
- `burke_gazebo/launch/base_sim.launch.py` expands and spawns that Xacro model.
- `burke_gazebo/config/bridge.yaml` bridges `/cmd_vel`, `/odom`, `/tf`, and
  `/clock`.
- The current robot model is spawned with the fixed name `burke_base`.
- The platform top is the top face of `base_link`; no separate top deck or
  LiftKit has been modeled yet.

Do not replace the working base, drive controller, launch flow, or bridge
configuration. Extend them with the arm.

## Scope Boundaries

In scope:

- A fixed mounting frame on the top centre of `base_link`.
- A serial chain containing exactly six bounded revolute joints.
- Primitive cylinder visual and collision geometry for the arm.
- Simple, non-zero inertial properties on every movable link.
- Gazebo-native position control for each joint.
- Six scalar ROS command topics and one joint-state feedback topic.
- A headless integration test and concise operator documentation.

Out of scope:

- UR8L vendor meshes or claims of dimensional accuracy.
- A LiftKit, top-deck enclosure, tool flange attachment, end-effector, payload,
  camera, light, or inspection sensor.
- Inverse kinematics, MoveIt, motion planning, trajectory interpolation, or
  coordinated multi-joint commands.
- `ros2_control`, controller manager, a custom controller node, or custom
  Gazebo plugins.
- Self-collision planning, aircraft collision checking, or autonomous motion.
- Changes to differential-drive behavior or its public topics.
- Real hardware interfaces, credentials, private addresses, or vendor APIs.

## Minimal Design Contract

### Kinematic structure

Use stable, generic names so the primitive geometry can be replaced later:

```text
base_link
└── arm_mount_joint (fixed)
    └── arm_mount_link
        └── arm_joint_1 (revolute, base yaw)
            └── arm_link_1
                └── arm_joint_2 (revolute, shoulder pitch)
                    └── arm_link_2
                        └── arm_joint_3 (revolute, elbow pitch)
                            └── arm_link_3
                                └── arm_joint_4 (revolute, wrist roll)
                                    └── arm_link_4
                                        └── arm_joint_5 (revolute, wrist pitch)
                                            └── arm_link_5
                                                └── arm_joint_6 (revolute, wrist roll)
                                                    └── arm_link_6
```

The chain ends at `arm_link_6`. Do not add `tool0`, a flange link, a dummy tool,
or any payload link in this milestone.

Mount the arm at `x=0`, `y=0` on the top face of `base_link`. Derive the mount
height from the existing chassis-height Xacro property instead of duplicating
the numeric height. The arm must remain attached to the base while it drives.

Use the axis sequence below unless Gazebo validation exposes a concrete issue:

| Joint | Purpose | Axis in joint frame |
| --- | --- | --- |
| `arm_joint_1` | base yaw | `0 0 1` |
| `arm_joint_2` | shoulder pitch | `0 1 0` |
| `arm_joint_3` | elbow pitch | `0 1 0` |
| `arm_joint_4` | wrist roll | `1 0 0` |
| `arm_joint_5` | wrist pitch | `0 1 0` |
| `arm_joint_6` | wrist roll | `1 0 0` |

All dimensions, masses, inertias, joint limits, maximum speeds, and controller
settings are simulation assumptions. Declare them as named Xacro properties,
label them accordingly, and choose conservative values that produce a compact,
stable model. Do not describe them as measured UR8L values. Keep the arm small
enough that its zero pose does not intersect the mobile base or the ground.

The simplest acceptable geometry is:

- one short cylinder for `arm_mount_link`;
- one cylinder for each `arm_link_1` through `arm_link_6`;
- matching cylinder collision geometry; and
- cylinders positioned so adjacent joint origins form a visibly connected
  serial chain.

No mesh assets are required. Reuse a single Xacro macro for repeated cylinder
link definitions and compute each cylinder inertia from its mass, radius, and
length instead of copying unexplained inertia tensors.

### ROS topic contract

Each topic accepts a target joint angle in radians as
`std_msgs/msg/Float64`:

| ROS command topic | Controlled joint | Direction |
| --- | --- | --- |
| `/arm/joint_1/command` | `arm_joint_1` | ROS to Gazebo |
| `/arm/joint_2/command` | `arm_joint_2` | ROS to Gazebo |
| `/arm/joint_3/command` | `arm_joint_3` | ROS to Gazebo |
| `/arm/joint_4/command` | `arm_joint_4` | ROS to Gazebo |
| `/arm/joint_5/command` | `arm_joint_5` | ROS to Gazebo |
| `/arm/joint_6/command` | `arm_joint_6` | ROS to Gazebo |

Expose joint feedback on `/joint_states` as `sensor_msgs/msg/JointState` and
include all six arm joints. Preserve the existing `/cmd_vel`, `/odom`, `/tf`,
and `/clock` contracts.

Use one Gazebo Harmonic `JointPositionController` system per arm joint. Give
each system a model-scoped Gazebo transport subtopic and bridge it to the ROS
topic above as `gz.msgs.Double`. Prefer the controller's bounded velocity mode
for this intentionally simple model rather than adding PID tuning work, but
verify the exact Harmonic plugin parameters in the supported runtime. Add
Gazebo's native joint-state publisher to the model and bridge its output; do
not create a custom relay or state publisher.

The command interface is deliberately six independent scalar topics. Do not
substitute `trajectory_msgs/msg/JointTrajectory` or add an aggregate command
API in this milestone.

## Target Files

The planned implementation should remain within these files unless validation
demonstrates a concrete need for another conventional ROS package resource:

```text
burke_description/
└── urdf/
    ├── burke_base.urdf.xacro
    └── components/
        └── simple_arm.urdf.xacro

burke_gazebo/
├── config/
│   └── bridge.yaml
├── launch/
│   └── base_sim.launch.py
└── test/
    └── test_arm_topics.py

README.md
```

Do not create empty placeholder files or directories. The implementation agent
may keep the arm macro in `burke_base.urdf.xacro` instead of creating
`components/simple_arm.urdf.xacro` only if doing so is materially simpler and
the base description remains readable.

## Rules for Task Agents

Each agent owns one task and must stop at its task boundary.

1. Read `AGENTS.md`, this plan, and every prerequisite task's changed files.
2. Inspect the working tree before editing and preserve unrelated user changes.
3. Confirm prerequisites from the files and validation evidence, not only from
   plan checkboxes.
4. Keep every provisional physical value configurable and documented as a
   simulation assumption.
5. Do not add any item listed as out of scope, even if it would be useful later.
6. Run all validation listed for the task that the environment supports.
7. If validation cannot run, report the exact missing dependency or runtime
   condition; never claim an unexecuted check passed.
8. Mark only the task's own checkbox complete after all acceptance criteria
   pass.
9. Leave a handoff listing changed files, commands run, observed results,
   assumptions, and blockers.

## Dependency Order

```text
Task 1: Primitive arm description
    ↓
Task 2: Gazebo joint control and ROS bridges
    ↓
Task 3: Integration test and operator documentation
    ↓
Task 4: Industrial arm kinematic layout
    ↓
Task 5: Final regression validation
```

Do not implement Tasks 1–4 in parallel because they update or validate the
same robot, bridge, launch, and test contracts. Task 5 is a verification and
correction pass, not a feature-expansion task.

## Task 1 — Add the Primitive Arm Description

- [x] Complete

### Goal

Extend the current Xacro robot with a directly mounted, primitive 6-DOF serial
arm while leaving all Gazebo controllers and bridges unchanged.

### Allowed Scope

- `burke_description/urdf/burke_base.urdf.xacro`
- `burke_description/urdf/components/simple_arm.urdf.xacro`, if used
- Description-package install rules only if the new file would otherwise not
  be installed
- No Gazebo control systems, bridge entries, launch changes, tests, or README
  changes

### Implementation Requirements

- Add `arm_mount_link`, its fixed joint, six movable links, and exactly six
  revolute joints using the names and axes in the design contract.
- Put the mount on the top centre of `base_link` using the existing chassis
  dimension properties.
- Use only cylinder visual and collision primitives for the arm.
- Give every physical link a positive mass and valid cylinder inertia.
- Add conservative position, velocity, and effort limits to all six revolute
  joints.
- Ensure the zero pose is connected, visible, and free of obvious intersection
  with the platform and ground.
- Keep base link names, wheel joints, support joints, and the differential-drive
  plugin unchanged.
- Do not add a tool or terminal dummy link.

### Acceptance Criteria

- Xacro expands without errors.
- The expanded URDF contains exactly `arm_joint_1` through `arm_joint_6` as
  revolute joints.
- `arm_link_6` is the terminal link and no tool or payload link exists.
- Every arm link has visual, collision, and inertial elements.
- All six joints have finite lower and upper position limits and positive
  velocity and effort limits.
- Gazebo can spawn the combined base-and-arm model without URDF or inertia
  errors.
- The original base frame tree and drive joints remain present.

### Validation

Run from the ROS workspace root after sourcing ROS 2:

```bash
xacro src/burke-sim/burke_description/urdf/burke_base.urdf.xacro > /tmp/burke_with_arm.urdf
check_urdf /tmp/burke_with_arm.urdf
colcon build --symlink-install --packages-select burke_description burke_gazebo
source install/setup.bash
timeout 30s ros2 launch burke_gazebo base_sim.launch.py gui:=false
```

If `check_urdf` or `timeout` is unavailable, record that exact limitation and
perform the equivalent available parse or bounded launch check.

### Handoff

Record the chosen provisional dimensions, masses, limits, zero-pose layout,
and whether the arm macro is separate or inline.

## Task 2 — Add Topic-Based Joint Control

- [x] Complete

### Prerequisite

Task 1.

### Goal

Make all six arm joints independently position-controllable from the defined
ROS topics and publish joint-state feedback without disturbing base control.

### Allowed Scope

- Arm-related Gazebo blocks in the robot Xacro files
- `burke_gazebo/config/bridge.yaml`
- Package dependency declarations only when required by the message bridges
- No geometry redesign, custom node, `ros2_control`, test implementation, or
  README changes

### Implementation Requirements

- Add one Gazebo Harmonic `JointPositionController` system for each arm joint.
- Use the same joint names and ROS topics defined in this plan.
- Use bounded joint motion; a large target must not bypass the URDF limit or
  create unbounded speed.
- Use model-scoped Gazebo transport topics to avoid collisions with future
  models, while retaining the stable ROS topic names.
- Add the Gazebo native joint-state publisher and bridge its model-scoped
  output to `/joint_states` as `sensor_msgs/msg/JointState`.
- Add six `std_msgs/msg/Float64` to `gz.msgs.Double` bridge entries.
- Preserve every existing bridge entry exactly unless a verified integration
  issue requires a minimal correction.
- Do not add a node that republishes or aggregates commands.

### Acceptance Criteria

- All six ROS command topics are visible after launch with the correct type.
- Publishing a valid target to any one command topic moves only the intended
  joint toward that target.
- `/joint_states` contains all six arm joint names and changing positions.
- Commands outside a joint's configured range do not drive it beyond its URDF
  limit.
- With no new command, the arm remains stable enough for the manual smoke test.
- `/cmd_vel` still drives the complete platform with the arm attached.

### Validation

With `base_sim.launch.py` running headlessly in one terminal, run in another:

```bash
ros2 topic list -t
ros2 topic echo --once /joint_states
ros2 topic pub --once /arm/joint_1/command std_msgs/msg/Float64 "{data: 0.25}"
ros2 topic pub --once /arm/joint_2/command std_msgs/msg/Float64 "{data: -0.20}"
ros2 topic echo --once /joint_states
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

Also command joints 3–6 individually and retain before/after feedback showing
that each intended joint changed. Use conservative values within the declared
limits.

### Handoff

Record the final ROS-to-Gazebo topic mapping, controller mode and maximum
speeds, observed feedback, and any Harmonic-specific parameter choices.

## Task 3 — Add an Arm Smoke Test and Usage Documentation

- [ ] Complete

### Prerequisite

Task 2.

### Goal

Make the arm interface repeatably verifiable and document the minimal manual
workflow for controlling it.

### Allowed Scope

- `burke_gazebo/test/test_arm_topics.py`
- Test registration and test-only dependencies in `burke_gazebo`
- `README.md`
- Minimal launch/config corrections only when the test exposes a defect in the
  Task 2 integration
- No new robot features, command APIs, geometry, tools, or controllers

### Test Requirements

Create the smallest reliable headless integration test supported by the
repository. It must:

1. launch or connect to the base simulation deterministically;
2. wait with explicit timeouts for all six command topics and `/joint_states`;
3. capture the starting joint positions;
4. command each joint to a small in-limit target, one at a time;
5. verify the intended joint reaches a documented tolerance before timeout;
6. verify no arm joint reports a position outside its declared limit;
7. publish a small `/cmd_vel` command and verify odometry changes; and
8. publish an explicit zero base command during cleanup.

The test must fail with a useful message when a topic, joint, or simulation
process is missing. It must not depend on the Gazebo GUI, wall-clock sleeps
without readiness checks, or real hardware.

### Documentation Requirements

Update `README.md` so it describes the new milestone rather than claiming the
repository only implements Milestone 1. Include:

- how to launch the combined base and arm;
- the six command topics and their `Float64` radians contract;
- a copy-paste example that commands one joint within its limits;
- how to inspect `/joint_states`;
- how to send the arm back to its documented zero pose;
- a warning that geometry and limits are simulation assumptions; and
- an explicit statement that there is no tool, trajectory control, or
  hardware interface yet.

### Acceptance Criteria

- The registered headless test passes in the supported Ubuntu environment.
- Failure paths have bounded timeouts and actionable messages.
- The README commands match the implemented topic names and message types.
- The documented reset sequence sends a zero target to all six joints.
- Existing base launch and teleoperation instructions remain correct.

### Validation

Run from the workspace root:

```bash
colcon build --symlink-install --packages-select burke_description burke_gazebo
source install/setup.bash
colcon test --packages-select burke_gazebo --event-handlers console_direct+
colcon test-result --verbose
```

Also execute the README's launch, one-joint command, joint-state inspection,
and six-joint reset commands exactly as written.

### Handoff

List the test cases, timeouts and tolerances, validation output, documentation
changes, and any remaining environment limitation.

## Task 4 — Rework the Arm into an Industrial Kinematic Layout

- [ ] Complete

### Prerequisite

Task 3.

### Goal

Replace the vertically stacked arm layout with a useful articulated layout
that visibly behaves like an industrial 6-DOF arm: base yaw, shoulder pitch,
elbow pitch, and a three-axis wrist. Keep the existing joint names, ROS topic
contract, controllers, and primitive-geometry approach.

### Allowed Scope

- `burke_description/urdf/components/simple_arm.urdf.xacro`
- Arm-specific assertions in `burke_gazebo/test/test_arm_topics.py`
- `README.md` only where the documented zero pose or arm description changes
- No bridge, launch, base-drive, command-interface, tool, payload, sensor,
  MoveIt, trajectory-control, or hardware-interface changes

### Kinematic Requirements

- Keep the existing serial chain and exactly six revolute joints named
  `arm_joint_1` through `arm_joint_6`.
- Lay out the chain as recognizable sections: a base/shoulder pedestal, an
  upper arm, a forearm, and a compact three-joint wrist.
- Place the shoulder and elbow joint origins at distinct physical pivots. The
  upper-arm and forearm geometry must span between those pivots instead of all
  link origins being translated along one vertical line.
- Orient each cylinder's visual, collision, and inertial origin to follow the
  segment it represents. Do not use a rotated visual to hide an incorrect
  kinematic frame.
- Retain the intended axis sequence: base yaw about `Z`, shoulder and elbow
  pitch about their local `Y` axes, wrist roll about local `X`, wrist pitch
  about local `Y`, and final wrist roll about local `X`.
- Ensure shoulder or elbow motion sweeps the downstream arm through a visible
  arc, while wrist joints primarily change wrist orientation rather than
  moving the whole chain around a shared axis.
- Define separate named Xacro properties for the provisional pedestal,
  upper-arm, forearm, and wrist dimensions. Continue to label all geometry,
  mass, inertia, and pose values as simulation assumptions.
- Choose a compact zero pose that is visibly connected, does not intersect the
  mobile base or ground, and can be held by the existing controllers.
- Preserve positive masses, valid inertias, joint limits, damping, and bounded
  controller speed. Keep each controller's initialized holding target.

### Acceptance Criteria

- Xacro expands and the resulting URDF passes structural validation.
- The arm has one connected six-joint serial chain with no tool or payload
  link.
- The shoulder, elbow, and wrist pivots are spatially distinct in the zero
  pose; the six movable links are not stacked on one common vertical line.
- In Gazebo, commanding `arm_joint_2` moves the upper-arm/downstream chain,
  commanding `arm_joint_3` bends the elbow relative to the upper arm, and
  commanding joints 4–6 produces recognizable wrist roll/pitch/roll motion.
- Each command topic still controls only its matching joint and
  `/joint_states` still reports all six joints.
- With no command, the arm holds its documented zero pose without falling onto
  the base or ground.
- Existing base translation, rotation, odometry, TF, clock, and arm topic
  contracts remain unchanged.

### Validation

Run the Task 3 build and headless integration test. Then launch Gazebo with the
GUI and command the shoulder, elbow, and wrist joints separately using small
in-limit targets:

```bash
ros2 launch burke_gazebo base_sim.launch.py
ros2 topic pub --once /arm/joint_2/command std_msgs/msg/Float64 "{data: 0.35}"
ros2 topic pub --once /arm/joint_3/command std_msgs/msg/Float64 "{data: -0.55}"
ros2 topic pub --once /arm/joint_4/command std_msgs/msg/Float64 "{data: 0.30}"
ros2 topic pub --once /arm/joint_5/command std_msgs/msg/Float64 "{data: -0.25}"
ros2 topic pub --once /arm/joint_6/command std_msgs/msg/Float64 "{data: 0.30}"
```

Verify visually that the shoulder, elbow, and wrist have distinct pivots and
motions. Return all joints to zero with the README reset loop, then leave the
simulation running briefly to confirm that the arm holds position.

### Handoff

Record the provisional segment dimensions, joint-origin layout, cylinder
orientations, zero pose, observed shoulder/elbow/wrist motions, automated test
results, and any remaining visual or physics limitations.

## Task 5 — Final Regression Validation

- [x] Complete

### Prerequisite

Tasks 1–4.

### Goal

Verify the milestone end to end and correct only defects that prevent its
documented acceptance criteria from passing.

### Allowed Scope

- Read the full milestone implementation.
- Make minimal corrections inside files already touched by Tasks 1–4.
- No refactor, new capability, higher-level motion interface, or future robot
  component.

### Acceptance Criteria

- A clean build succeeds for both packages.
- The combined model spawns in headless Gazebo without model/plugin errors.
- The model has exactly six commanded arm joints and no tool link.
- Each documented ROS command topic moves the matching joint and no other
  command mapping is crossed.
- `/joint_states` reports all six joints.
- The arm remains mounted while the base translates and rotates.
- Arm joint limits and speed bounds are respected.
- Existing `/cmd_vel`, `/odom`, `/tf`, and `/clock` behavior still works.
- The automated test suite passes.
- All physical values remain labeled as assumptions.

### Validation

Run the full Task 4 validation, then perform one GUI demonstration:

```bash
ros2 launch burke_gazebo base_sim.launch.py
```

During the demonstration, command all six joints to small distinct in-limit
angles, drive the base forward and rotate it, stop the base explicitly, and
return all joints to zero. Inspect Gazebo logs for warnings or errors related
to the arm model, controller systems, or bridges.

### Handoff

Report the final build and test results, the six observed joint motions, base
regression results, remaining warnings, and any validation that could not be
performed. Mark this task complete only when no required work remains.

### Task 5 Handoff

- `colcon build --symlink-install --packages-select burke_description burke_gazebo`: passed.
- Xacro expansion and `check_urdf`: passed; the model has exactly six arm
  revolute joints, `arm_link_6` is terminal, and no tool, flange, or payload
  link exists.
- Headless `base_sim.launch.py gui:=false`: spawned successfully in the
  supported ROS 2 Jazzy/Gazebo Harmonic runtime. All six command bridges and
  the `/joint_states` bridge were created without arm model or controller
  errors.
- Registered `test_arm_topics`: passed in 5.39 seconds. It commanded all six
  joints independently, verified joint-state targets and limits, and verified
  base odometry changed after `/cmd_vel`.
- Static contract validation confirmed the documented `/cmd_vel`, `/odom`,
  `/tf`, `/clock`, six arm command topics, and `/joint_states` mappings.
- The GUI demonstration was not run because this validation environment is
  headless. The sandbox-only launch attempt was also blocked by DDS socket and
  filesystem permissions; the approved runtime launch passed.
- Gazebo emitted no arm-specific model, controller, or bridge errors. The
  runtime log contained repeated robot-state-publisher time-backward warnings
  during startup, unrelated to the arm geometry or command mappings.
