# Vertical Lift Integration Plan

## Objective

Insert the supplied three-stage LiftKit telescoping mast between the MiR1350
platform and the existing UR8 Long arm:

- use `LIFTKIT_1.stl`, `LIFTKIT_2.stl`, and `LIFTKIT_3.stl` as three
  independent visual links;
- reproduce the collapsed state in which the upper stages are nested inside
  the stages below them;
- reproduce the fully extended state in which the stages are stacked
  vertically;
- retain simple primitive collision geometry rather than STL collisions;
- move the complete UR8 Long arm with the top lift stage;
- expose the telescoping joints through ROS 2 position-command topics; and
- preserve the existing MiR drive and six arm command interfaces.

This file is an implementation plan for sequential sub-agents. It does not
authorize adding the LiftKit code as part of the planning task.

## Existing Baseline

Agents must extend the repository as it exists when their task starts:

- Ubuntu 24.04, ROS 2 Jazzy, and Gazebo Harmonic remain the supported runtime.
- `burke_description/urdf/burke_base.urdf.xacro` owns the MiR model and invokes
  the UR8 Long arm macro.
- `burke_description/urdf/components/ur8long.urdf.xacro` currently fixes
  `arm_mount_link` directly to `base_link` with `arm_mount_joint`.
- The MiR visual top is `0.321230 m` above `base_link`.
- The UR8 Long arm, its six controllers, `/arm/joint_1/command` through
  `/arm/joint_6/command`, and `/joint_states` already exist.
- `burke_gazebo/config/bridge.yaml` also preserves `/cmd_vel`, `/odom`, `/tf`,
  and `/clock`.

The lift must be inserted into the current hierarchy. Do not rebuild the MiR,
replace the UR kinematics, rename existing arm joints, or change existing topic
contracts.

## CAD Evidence and Motion Derivation

STL files do not encode units. These LiftKit files use the same millimetre CAD
convention as the supplied UR meshes, so the provisional visual scale is
`0.001`.

### Raw bounds

| Asset | Raw minimum `(x,y,z)` | Raw maximum `(x,y,z)` | Raw size `(x,y,z)` | Role |
| --- | --- | --- | --- | --- |
| `LIFTKIT_1.stl` | `(-222.9, 0.0, -100.0)` | `(100.0, 487.6, 100.0)` | `(322.9, 487.6, 200.0)` | Bottom stage |
| `LIFTKIT_2.stl` | `(-74.2, 487.6, -74.2)` | `(74.2, 770.6, 74.2)` | `(148.4, 283.0, 148.4)` | Middle stage |
| `LIFTKIT_3.stl` | `(-98.0, 770.6, -98.0)` | `(98.0, 1055.0, 98.0)` | `(196.0, 284.4, 196.0)` | Top stage |
| `LIFTKIT-UR-500-1100-601R.stl` | `(-222.9, 0.0, -100.0)` | `(100.0, 555.0, 100.0)` | `(322.9, 555.0, 200.0)` | Collapsed reference only |

The three split STLs are stored in a common, fully extended assembly frame:

- stage 1 occupies raw lift-axis coordinates `0.0–487.6 mm`;
- stage 2 occupies `487.6–770.6 mm`; and
- stage 3 occupies `770.6–1055.0 mm`.

Rigid triangle matching against the collapsed reference produces these
collapsed transforms without rotation:

| Stage | Translation from extended CAD pose | Collapsed lift-axis range |
| --- | ---: | ---: |
| Stage 1 | `0 mm` | `0.0–487.6 mm` |
| Stage 2 | `-275 mm` | `212.6–495.6 mm` |
| Stage 3 | `-500 mm` | `270.6–555.0 mm` |

Therefore the CAD-supported telescoping motion is:

- stage 2 travel: `0.275 m` relative to stage 1;
- stage 3 travel: `0.225 m` relative to stage 2;
- total lift travel: `0.500 m`;
- collapsed mast top: `0.555 m` above the lift base; and
- fully extended mast top: `1.055 m` above the lift base.

The combined STL is a collapsed reference and must not be added as a runtime
visual, because it would duplicate the three articulated stage visuals.

## Required Design Contract

### Clarification about “three moving parts”

The supplied CAD establishes three mast stages but only two relative
telescoping motions: stage 1 does not change pose between the extended split
assembly and the collapsed reference, while stages 2 and 3 retract by `275 mm`
and `500 mm` in the common assembly frame.

Use the following CAD-grounded interpretation unless the project owner
explicitly changes it:

- all three STL files are independent links;
- stage 1 is rigidly mounted to the MiR and is the fixed lower mast stage;
- stage 2 moves relative to stage 1; and
- stage 3 moves relative to stage 2.

If stage 1 must also translate relative to the MiR, stop and ask for the
missing fixed housing geometry, stage-1 stroke, collapsed origin, and extended
origin. Do not invent a third prismatic travel from the existing files.

### Coordinate convention

- Robot `+Z` is the vertical lift direction.
- The LiftKit STL lift direction is raw `+Y`.
- Begin with mesh rotation `rpy=(pi/2, 0, 0)`, which maps raw `+Y` to robot
  `+Z` and raw `+Z` to robot `-Y`.
- Retain the CAD mast centreline at raw `x=0`, `z=0`; do not centre the
  asymmetric stage-1 AABB, because its negative-X extension represents the
  actuator housing.
- Mount the lift centreline at `x=0`, `y=0` on the MiR visual top,
  `z=0.321230 m` in `base_link`.
- Task 1 must verify the sign of the `pi/2` rotation in the GUI. If the
  actuator housing faces the wrong platform direction, record and correct only
  the lift-mount yaw; do not alter the telescoping axis.

### Kinematic hierarchy

Use this stable frame structure:

```text
base_link
└── lift_mount_joint (fixed at MiR top)
    └── lift_stage_1_link              # LIFTKIT_1.stl
        └── lift_stage_2_joint (prismatic +Z, 0.000–0.275 m)
            └── lift_stage_2_link      # LIFTKIT_2.stl
                └── lift_stage_3_joint (prismatic +Z, 0.000–0.225 m)
                    └── lift_stage_3_link  # LIFTKIT_3.stl
                        └── arm_mount_joint (fixed)
                            └── arm_mount_link
                                └── existing six-joint UR8 Long chain
```

Joint position zero is the fully collapsed mast. The visual origins must use
the CAD-derived retraction offsets:

- `lift_stage_1_link`: no lift-axis visual offset;
- `lift_stage_2_link`: `-0.275 m` along robot `Z` after orienting the mesh;
- `lift_stage_3_link`: `-0.500 m` along robot `Z` after orienting the mesh.

Because stage 3 is downstream of stage 2, its absolute displacement is
`q_stage_2 + q_stage_3`. At full extension this is
`0.275 + 0.225 = 0.500 m`, cancelling the stage-3 visual retraction offset and
reproducing the supplied extended CAD pose.

Fix the arm mount to the top-stage frame so the arm-base height relative to the
lift base is:

```text
arm_base_height = 0.555 + q_stage_2 + q_stage_3
```

The arm base therefore ranges from `0.555 m` to `1.055 m` above the LiftKit
base, or from `0.876230 m` to `1.376230 m` above `base_link` with the current
MiR geometry.

### Visual and collision geometry

- Use each stage STL exactly once as visual geometry through an installed
  `package://burke_description/...` URI.
- Apply scale `0.001` uniformly.
- Do not edit, rename, decimate, overwrite, or use the STL files for collision.
- Keep collision geometry primitive. Prefer a small set of boxes or cylinders
  per stage over one excessively broad AABB.
- Derive each collision primitive from the scaled stage bounds and record its
  centre, dimensions, and clearance margin as named Xacro properties.
- Collision geometry must follow its stage link during extension and
  retraction.
- Adjacent nested stages must not lock or destabilize each other because their
  simplified collisions overlap. Use Gazebo's supported same-model collision
  filtering or appropriately hollow/split primitive approximations; do not
  shrink the external mast envelope so far that environment clearance checks
  become misleading.
- Keep external collisions for the lift, arm, MiR, aircraft, and environment.
- Give every lift link a positive assumed mass and valid inertia derived from
  its primitive approximation unless verified LiftKit physical data becomes
  available. Label these values as simulation assumptions.

### Lift topic contract

Keep the first implementation minimal and expose the two physical prismatic
joints independently. Targets are metres in `std_msgs/msg/Float64`:

| ROS topic | Joint | Valid target |
| --- | --- | ---: |
| `/lift/stage_2/command` | `lift_stage_2_joint` | `0.000–0.275 m` |
| `/lift/stage_3/command` | `lift_stage_3_joint` | `0.000–0.225 m` |

Use model-scoped Gazebo topics and one native bounded position controller per
joint. Include both joints in the existing `/joint_states` feedback.

Do not add an aggregate lift-height node, action server, `ros2_control`, or
trajectory controller in this milestone. Document the total height formula
and provide paired command examples for collapsed, half-height, and fully
extended poses.

### Motion sequencing and safety rules

- The MiR must be stationary before either lift joint moves.
- The UR arm must be in the documented stow pose before the lift moves.
- Do not command arm motion while either lift joint is moving.
- Do not command base motion until the arm is stowed and the lift is fully
  collapsed.
- Lift controllers must start at `0.0 m` and hold the collapsed position under
  the simulated arm load.
- Use conservative assumed lift speed and effort limits, documented next to
  the joints. Do not present them as vendor ratings.
- A command beyond a joint limit must not move the stage beyond the URDF
  limit.
- A future interlock/controller may enforce sequencing automatically. This
  milestone validates and documents the protocol without adding unrelated
  orchestration infrastructure.

### Stop-and-ask conditions

An implementation agent must stop and request project-owner input if any of
these cannot be verified from the CAD or existing model:

- stage 1 is required to move relative to a missing fixed housing;
- the LiftKit mount is not centred at `x=0`, `y=0` on the MiR;
- the LiftKit requires a non-zero platform yaw;
- the arm attachment point differs from the `0.555–1.055 m` top-stage surface;
- the collapsed or extended stage positions disagree with the reference STL
  by more than `1 mm` at their mating interfaces;
- primitive collisions cannot represent the external envelope without
  blocking valid telescoping motion; or
- verified masses, centre-of-mass locations, effort limits, or speed limits
  are required instead of documented simulation assumptions.

Report the measured value, proposed interpretation, and exact missing value in
the question. Do not silently substitute unverified vendor dimensions.

## Agent Working Rules

1. Each agent owns exactly one task and must stop at its allowed boundary.
2. Read `AGENTS.md`, this plan, the current working tree, and all prerequisite
   handoffs before editing.
3. Preserve unrelated user changes and never modify the source STL files.
4. Confirm prerequisite behavior rather than relying only on plan checkboxes.
5. Keep physical assumptions configurable and documented near their use.
6. Validate resources from the installed ROS workspace, not only source paths.
7. Use bounded waits for every launch, controller, and feedback check.
8. Obey the stop-and-ask conditions instead of guessing.
9. Mark only the assigned task complete after all acceptance criteria pass.
10. Leave a handoff listing changed files, exact transforms and dimensions,
    commands, results, assumptions, and blockers.

## Dependency Order

```text
Task 1: Verify LiftKit CAD end states and mounting orientation
    ↓
Task 2: Add the three-stage telescoping mast description
    ↓
Task 3: Re-parent the UR8 Long arm onto the top stage
    ↓
Task 4: Add lift command topics and feedback
    ↓
Task 5: Add integration tests and operator documentation
    ↓
Task 6: Perform final visual, kinematic, and regression validation
```

Do not parallelize these tasks. They share the robot hierarchy, and each
downstream task depends on transforms validated by the preceding task.

## Task 1 — Verify LiftKit CAD End States and Mounting Orientation

- [ ] Complete

### Goal

Confirm the CAD-derived collapsed and extended transforms before modifying the
robot description.

### Allowed Scope

- Read-only inspection of the four LiftKit STL files
- This plan's Task 1 handoff section
- No Xacro, URDF, CMake, bridge, launch, test, README, or STL changes

### Work

- Recompute binary-STL validity, triangle counts, bounds, and `0.001` scale.
- Rigidly match each split stage against the collapsed reference.
- Confirm stage-1 translation `0`, stage-2 translation `-0.275 m`, and stage-3
  translation `-0.500 m` along the lift axis.
- Reassemble both end states numerically and verify their total heights are
  `0.555 m` and `1.055 m`.
- Confirm that `rpy=(pi/2,0,0)` maps raw `+Y` to robot `+Z`.
- Produce front, side, and top evidence for both end states.
- Confirm the centred MiR-top mounting assumption or trigger a stop-and-ask
  condition.

### Acceptance Criteria

- Every LiftKit STL imports successfully.
- The split meshes reproduce the collapsed reference within `1 mm` at mating
  interfaces and a documented surface tolerance elsewhere.
- The two serial strokes sum to exactly `0.500 m` within numerical tolerance.
- The proposed mount orientation is explicit and places no visual below the
  MiR top surface.
- Any ambiguity about a moving stage 1 is resolved before Task 2.

### Handoff

Record the four bounds, triangle counts, end-state transforms, overlay
tolerances, mount pose, screenshots or numeric evidence, and owner decisions.

## Task 2 — Add the Three-Stage Telescoping Mast Description

- [ ] Complete

### Prerequisite

Task 1.

### Goal

Add three independently visualized mast links with two bounded serial
prismatic joints, without moving the arm from its current parent yet.

### Allowed Scope

- A new focused component such as
  `burke_description/urdf/components/liftkit.urdf.xacro`
- `burke_description/urdf/burke_base.urdf.xacro`
- Description install rules only if required for existing CAD resources
- No arm-macro edits, Gazebo controllers, bridge changes, tests, or README
  changes

### Work

- Create `lift_stage_1_link`, `lift_stage_2_link`, and `lift_stage_3_link`.
- Fix stage 1 to the current MiR top through `lift_mount_joint`.
- Add `lift_stage_2_joint` and `lift_stage_3_joint` with axis `0 0 1` and limits
  `0–0.275 m` and `0–0.225 m`.
- Use the exact visual offsets defined in the design contract.
- Add primitive collisions, assumed masses, inertias, damping, friction,
  velocity limits, and effort limits as named properties.
- Ensure nested-stage collision handling does not destabilize the model.
- Add a fixed, massless `lift_top_mount` frame only if it materially clarifies
  the arm attachment; do not add a fourth visual mast body.

### Acceptance Criteria

- Xacro expands with exactly three lift visual links and two prismatic joints.
- Each runtime lift visual references one unique split STL at scale `0.001`.
- No lift STL is referenced from a collision element.
- At joint positions `(0,0)`, the mast top is `0.555 m` above its base.
- At joint positions `(0.275,0.225)`, the mast top is `1.055 m` above its base.
- Both end states match Task 1's CAD evidence.
- The mast remains attached and stable while the MiR is stationary.

### Validation

```bash
xacro src/burke-sim/burke_description/urdf/burke_base.urdf.xacro > /tmp/burke_lift.urdf
check_urdf /tmp/burke_lift.urdf
colcon build --symlink-install --packages-select burke_description burke_gazebo
source install/setup.bash
timeout 30s ros2 launch burke_gazebo base_sim.launch.py gui:=false
```

Also inspect collision geometry and both end poses in the Gazebo GUI.

### Handoff

Record the link tree, visual transforms, joint limits, primitive collision
dimensions, collision filtering, masses/inertias, and end-state measurements.

## Task 3 — Re-parent the UR8 Long Arm onto the Top Stage

- [ ] Complete

### Prerequisite

Task 2.

### Goal

Make the complete existing arm move rigidly with `lift_stage_3_link` while
preserving all arm geometry, kinematics, controllers, and public names.

### Allowed Scope

- `burke_description/urdf/components/ur8long.urdf.xacro`
- The lift component and `burke_base.urdf.xacro` only where required to pass
  the arm parent and attachment transform
- Focused static-description validation
- No arm CAD refit, arm joint-limit change, bridge change, new controller, or
  README edit

### Work

- Parameterize the UR8 Long macro's mount parent instead of hardcoding
  `base_link`.
- Parent `arm_mount_joint` to `lift_stage_3_link` or the verified
  `lift_top_mount` frame.
- Replace the old direct MiR mount height with the top-stage attachment pose.
- Preserve `arm_mount_link`, `arm_joint_1` through `arm_joint_6`, their
  controllers, initial pose, and existing topic names.
- Verify the arm-base transform follows
  `0.555 + q_stage_2 + q_stage_3` relative to the lift base.

### Acceptance Criteria

- There is exactly one kinematic path from `base_link` through all three lift
  links to the UR arm.
- The arm base is `0.876230 m` above `base_link` when collapsed and
  `1.376230 m` above `base_link` when fully extended.
- All arm links move rigidly with stage 3 during lift travel.
- Moving a lift joint does not change any arm joint position value.
- The arm's existing visual alignment, inertias, joint limits, initial pose,
  six controllers, and public topics remain unchanged.
- No direct `base_link` to `arm_mount_link` joint remains.

### Validation

Expand the Xacro and inspect the complete parent-child tree. In Gazebo, hold
the arm at its initial/stow pose, compare collapsed and extended arm-base
heights, and confirm all arm visuals remain connected during slow lift motion.

### Handoff

Record the macro-interface change, final arm-mount transform, measured minimum
and maximum arm-base heights, and arm regression results.

## Task 4 — Add Lift Command Topics and Joint Feedback

- [ ] Complete

### Prerequisite

Task 3.

### Goal

Control both telescoping motions independently from ROS 2 and publish their
positions through the existing joint-state interface.

### Allowed Scope

- Lift-related Gazebo system blocks
- `burke_gazebo/config/bridge.yaml`
- Required package dependency declarations
- No aggregate controller, arm-controller change, geometry refit, or motion
  orchestration node

### Work

- Add one bounded native position controller for each prismatic joint.
- Use model-scoped Gazebo topics and the ROS topics in the design contract.
- Use conservative assumed velocity/effort parameters that hold the complete
  arm load without high-speed extension.
- Initialize and hold both joints at `0.0 m`.
- Confirm the existing joint-state publisher includes both lift joints and all
  six arm joints.
- Verify out-of-range commands cannot exceed URDF travel limits.
- Preserve all base, arm, clock, odometry, and TF bridge entries.

### Acceptance Criteria

- Both ROS command topics exist as `std_msgs/msg/Float64` inputs.
- `/joint_states` reports both lift joints in metres.
- Commands `(0,0)` produce the collapsed state.
- Commands `(0.275,0.225)` produce the fully extended state.
- Each command moves only the intended relative stage.
- The controllers hold position under gravity with the stowed arm attached.
- Existing MiR and arm topics remain unchanged and operational.

### Validation

```bash
ros2 topic list -t
ros2 topic echo --once /joint_states
ros2 topic pub --once /lift/stage_2/command std_msgs/msg/Float64 "{data: 0.275}"
ros2 topic pub --once /lift/stage_3/command std_msgs/msg/Float64 "{data: 0.225}"
ros2 topic echo --once /joint_states
ros2 topic pub --once /lift/stage_3/command std_msgs/msg/Float64 "{data: 0.0}"
ros2 topic pub --once /lift/stage_2/command std_msgs/msg/Float64 "{data: 0.0}"
```

Use repeated bounded publication if bridge discovery makes one-shot delivery
unreliable. Record the actual position and settling tolerance for each target.

### Handoff

Record ROS/Gazebo topic mappings, controller parameters, measured travel,
holding behavior, limit behavior, and base/arm regression results.

## Task 5 — Add Integration Tests and Operator Documentation

- [ ] Complete

### Prerequisite

Task 4.

### Goal

Make lift resources, end states, command behavior, and safe transition protocol
repeatably verifiable.

### Allowed Scope

- `burke_gazebo/test/`
- Test registration and test-only dependencies
- `README.md`
- Minimal corrections to Tasks 2–4 only when a test proves a defect
- No new feature, controller architecture, or CAD modification

### Test Requirements

Add or extend bounded headless tests to verify:

1. all three installed LiftKit mesh URIs resolve;
2. no LiftKit STL is used for collision;
3. the model has three lift links and exactly two lift prismatic joints;
4. both lift topics and feedback become ready before commands are sent;
5. the arm is placed in its documented stow pose before lift motion;
6. `(0,0)` yields a `0.555 m` mast height within tolerance;
7. an intermediate paired target produces the expected summed height;
8. `(0.275,0.225)` yields a `1.055 m` mast height within tolerance;
9. each stage remains within its limit throughout the test;
10. the arm-link transforms move by the same total lift displacement;
11. the lift returns to `(0,0)` before any base command;
12. base odometry and all six arm topic mappings still work; and
13. cleanup explicitly stops the base and returns lift and arm to safe poses.

Every readiness wait, movement wait, and launched process must have a timeout
and actionable failure message.

### Documentation Requirements

Update the README with:

- the three-stage/two-prismatic-joint interpretation;
- the collapsed and extended dimensions;
- lift topic names, units, and ranges;
- commands for collapsed, half-height, and fully extended poses;
- the total arm-height formula;
- the arm-stow/base-stop/lift-motion sequence;
- the `0.001` scale and primitive-collision policy; and
- the fact that masses, actuator effort, and speeds are simulation assumptions.

### Acceptance Criteria

- A clean build and registered headless tests pass in the supported runtime.
- Tests detect missing assets, crossed topics, incorrect travel, bad end-state
  heights, arm detachment, and base/arm regressions.
- README commands exactly match the implemented interface.
- No GUI, real hardware, vendor controller, or private network is required by
  automated tests.

### Validation

```bash
colcon build --symlink-install --packages-select burke_description burke_gazebo
source install/setup.bash
colcon test --packages-select burke_description burke_gazebo --event-handlers console_direct+
colcon test-result --verbose
```

Execute every documented manual command exactly as written.

### Handoff

List test names, timeouts, tolerances, results, documentation changes, and any
environment-dependent validation that could not run.

## Task 6 — Final Visual, Kinematic, and Regression Validation

- [ ] Complete

### Prerequisites

Tasks 1–5.

### Goal

Perform an end-to-end review and correct only defects that prevent the lift
milestone's acceptance criteria.

### Allowed Scope

- Inspect the complete milestone.
- Make minimal corrections within files already touched by Tasks 2–5.
- No fourth lift stage, aggregate controller, autonomous sequencing, sensor,
  payload, or unrelated refactor.

### Acceptance Criteria

- Clean build and automated tests pass.
- Installed LiftKit visuals resolve with correct scale and orientation.
- The collapsed split model matches the combined collapsed reference.
- Full extension matches the supplied split CAD assembly.
- The mast top travels exactly `0.500 m` within documented tolerance.
- Primitive collisions move with their stages, approximate the external
  envelope, and do not block nesting.
- The stowed arm remains attached and stable at minimum, intermediate, and
  maximum lift heights.
- Lift joints hold position under gravity without unacceptable drift,
  oscillation, or explosive contact behavior.
- MiR drive, odometry, TF, clock, all arm topics, all lift topics, and joint
  feedback remain correct.
- The arm and lift are collapsed/stowed before base motion.
- The complete model contains three LiftKit runtime visuals and does not use
  the combined reference as a duplicate visual.
- All non-verified physical values remain labeled as assumptions.

### Validation

Run Task 5's full validation, then perform one GUI inspection from front, side,
top, and close telescoping views. Slowly exercise collapsed, intermediate, and
fully extended configurations with the arm stowed. Return the lift to zero,
drive and rotate the MiR, stop it explicitly, and regression-test small arm
commands. Review Gazebo logs for mesh, inertia, collision, controller, joint,
bridge, and TF warnings.

### Handoff

Report final transforms, travel measurements, visual/collision QA, holding and
stability observations, base/arm/lift regression results, build/test output,
remaining warnings, and any check that could not be performed. Mark complete
only when no required work remains.
