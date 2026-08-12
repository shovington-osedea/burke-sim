# CAD Visual Integration Plan: MiR1350 Platform and UR8L Arm

## Objective

Replace the current primitive visuals with the supplied CAD STL assets while
keeping simulation physics deliberately simple:

- use `MiR1350 1.0.stl` as the mobile-platform visual;
- use the split `UR8L_PART_*.stl` files as the visuals for an articulated
  6-DOF arm mounted directly on top of the platform;
- retain primitive boxes, cylinders, and spheres for collision geometry;
- resize and reposition collision primitives from the scaled STL bounds;
- preserve the existing differential-drive behavior; and
- expose each arm joint through an independent ROS 2 position-command topic.

There is no tool, camera, payload, LiftKit, control box, motion planner, or
hardware integration in this milestone.

## Existing Baseline

Agents must extend the repository as it exists when their task starts:

- Ubuntu 24.04, ROS 2 Jazzy, and Gazebo Harmonic are the supported runtime.
- `burke_description/urdf/burke_base.urdf.xacro` contains the current primitive
  differential-drive base.
- `burke_gazebo/launch/base_sim.launch.py` expands and spawns that description.
- `burke_gazebo/config/bridge.yaml` exposes `/cmd_vel`, `/odom`, `/tf`, and
  `/clock`.
- The model is named `burke_base` and uses `+X` forward, `+Y` left, `+Z` up.
- No arm implementation currently exists; this plan supersedes the previous
  primitive-arm plan.

Do not replace the working launch architecture or base controller. Make the
smallest changes required to introduce the CAD visuals, matching primitive
collisions, arm joints, topic control, and verification.

## Supplied CAD Inventory and Initial Measurements

STL files do not encode units. The values below are raw axis-aligned bounds
reported by Assimp and must not be treated as commissioned measurements.

The provisional scale is `0.001`, because the MiR bounds then become
`0.910 × 1.350 × 0.320963 m`, consistent with the expected physical class of a
MiR1350, and the complete UR model becomes approximately `0.702 × 0.943 ×
1.788 m`, consistent with the documented approximate `1.73 m` reach.

| Asset | Raw size X × Y × Z | Planned role |
| --- | --- | --- |
| `MiR1350 1.0.stl` | `910.000 × 1350.000 × 320.963` | Platform visual |
| `UR8 Long.stl` | `701.756 × 942.624 × 1788.369` | Assembled reference only |
| `UR8L_PART_1_C-2008883.stl` | `228.100 × 203.998 × 106.400` | Arm rigid body 1 |
| `UR8L_PART_2_C-2008884.stl` | `183.997 × 192.738 × 150.558` | Arm rigid body 2 |
| `UR8L_PART_3_C-2008885.stl` | `190.706 × 1038.652 × 150.000` | Arm rigid body 3 |
| `UR8L_PART_4_C-2008886.stl` | `150.718 × 824.542 × 120.000` | Arm rigid body 4 |
| `UR8L_PART_5_C-2008887.stl` | `111.607 × 119.888 × 90.002` | Arm rigid body 5 |
| `UR8L_PART_6_C-2008888.stl` | `134.323 × 109.298 × 90.393` | Arm rigid body 6 |
| `UR8L_PART_7_C-2008889.stl` | `57.400 × 95.474 × 89.989` | Arm rigid body 7 |

The assembled UR STL is a single rigid mesh and therefore cannot be the visual
for a moving 6-DOF chain. Use it only to validate the zero-pose assembly. Use
the seven part STLs as the runtime arm visuals: one fixed/base body plus six
bodies downstream of the six revolute joints. Confirm the exact part-to-link
order and origins by overlaying the assembled and split meshes before relying
on the numerical file order.

The following supplied assets are explicitly deferred and must not be added:

- `LIFTKIT-UR-500-1100-601R.stl`
- `LIFTKIT_1.stl`, `LIFTKIT_2.stl`, and `LIFTKIT_3.stl`
- `Extended Camera Module Assembly.stl`
- `UR_Control_Box.stl`

## Required Design Contract

### Asset handling

- Keep the source STL files under `burke_description/cad/stl/`; do not edit,
  decimate, convert, rename, or overwrite them in this milestone.
- Install the CAD directory through `burke_description` so Gazebo resolves
  visuals with `package://burke_description/cad/stl/...` URIs from an installed
  workspace.
- Quote or correctly encode filenames containing spaces. Do not depend on the
  repository working directory.
- Use one named Xacro property for the provisional STL scale and apply it
  consistently to every supplied mesh.
- CAD files are visual-only. No STL may appear in a `<collision>` element.

### MiR platform

- `MiR1350 1.0.stl` is the sole overall platform visual. Remove or suppress
  duplicate primitive chassis, wheel, and caster visuals that protrude through
  it, but preserve the existing drive-wheel and support link structure needed
  by physics.
- Orient the mesh so its `1.350 m` dimension follows robot `+X`, its `0.910 m`
  dimension follows `Y`, and its height follows `+Z`.
- Place the bottom of the scaled mesh at the ground-clearance/reference height
  appropriate to the existing `base_footprint` and wheel arrangement. Do not
  bury the visual below the ground plane.
- Replace the old chassis collision size with a conservative box or small set
  of boxes derived from the scaled MiR bounds. It must cover the main body
  without extending beyond the visual shell in the normal driving envelope.
- Preserve primitive wheel and ground-contact collision geometry. Adjust wheel
  separation, wheel position, caster/support position, and chassis height only
  as needed to align stable contact physics with the new `1.350 × 0.910 ×
  0.321 m` visual envelope.
- Keep all collision dimensions as named Xacro properties and state which STL
  bounds produced them. Collision geometry may include a documented clearance
  margin, but that margin must be small and explicit.

### UR8L arm

Use this stable kinematic naming contract:

```text
base_link
└── arm_mount_joint (fixed)
    └── arm_base_link              # one UR part visual
        └── arm_joint_1 (revolute)
            └── arm_link_1         # one UR part visual
                └── arm_joint_2 (revolute)
                    └── arm_link_2 # one UR part visual
                        └── arm_joint_3 (revolute)
                            └── arm_link_3
                                └── arm_joint_4 (revolute)
                                    └── arm_link_4
                                        └── arm_joint_5 (revolute)
                                            └── arm_link_5
                                                └── arm_joint_6 (revolute)
                                                    └── arm_link_6
```

- Mount the arm at the top centre of the MiR visual unless a supplied CAD
  reference unambiguously establishes another mount pose.
- The fixed mount height must be derived from the final platform visual and
  collision dimensions, not copied from the old primitive chassis.
- Each of the seven split UR STLs must appear on exactly one arm link. Do not
  use `UR8 Long.stl` as an additional runtime visual, because that would create
  a duplicate rigid arm over the articulated one.
- Preserve the mesh's local CAD origin where it corresponds to a joint axis.
  Use link visual origins and joint origins deliberately; do not recenter every
  STL merely to make its AABB symmetrical.
- Use exactly six bounded revolute joints. Determine joint axes and link-to-link
  origins by matching the split parts to the assembled zero-pose reference.
- The terminal body is `arm_link_6`. Do not add a tool, flange extension, dummy
  end link, camera, or payload.
- Use simple cylinders or boxes for arm collisions. Size and offset each
  primitive from its scaled part bounds, then shrink or split overly broad AABB
  collisions where necessary so adjacent links can articulate without false
  contact at their intended joints.
- Give every physical arm link positive mass and valid primitive-based inertia.
  Masses, inertias, limits, damping, friction, and controller speeds remain
  documented simulation assumptions; STL geometry alone does not supply them.

### Stop-and-ask conditions

Do not invent missing geometry. Stop the assigned task and ask the project
owner for the relevant dimension or transform if any of these cannot be
established by scaled mesh bounds and assembled/split overlay:

- STL unit scale;
- platform forward direction or ground-relative mesh offset;
- arm part ordering;
- any of the six joint pivot locations or axes;
- arm mounting `x`, `y`, `z`, roll, pitch, or yaw on the platform; or
- a collision dimension whose conservative primitive would materially exceed
  the corresponding visual.

Record the measured raw bound, proposed interpretation, and exact missing
value in the question. Do not silently substitute vendor dimensions from an
unverified third-party model.

### ROS interface

Keep the smallest topic-based control interface. Each target is radians in a
`std_msgs/msg/Float64`:

| ROS topic | Joint |
| --- | --- |
| `/arm/joint_1/command` | `arm_joint_1` |
| `/arm/joint_2/command` | `arm_joint_2` |
| `/arm/joint_3/command` | `arm_joint_3` |
| `/arm/joint_4/command` | `arm_joint_4` |
| `/arm/joint_5/command` | `arm_joint_5` |
| `/arm/joint_6/command` | `arm_joint_6` |

Use one Gazebo Harmonic `JointPositionController` per joint and bridge each
model-scoped Gazebo `gz.msgs.Double` topic to the stable ROS topic. Publish all
six joints on `/joint_states` as `sensor_msgs/msg/JointState`. Do not introduce
`ros2_control`, MoveIt, trajectory messages, a custom controller, or an
aggregate command node.

Preserve `/cmd_vel`, `/odom`, `/tf`, and `/clock` and their current message
types.

## Agent Working Rules

1. Take only one task at a time and respect its allowed scope.
2. Read `AGENTS.md`, this plan, the current working tree, and prerequisite task
   handoffs before editing.
3. Preserve unrelated user files, including untracked CAD assets and `.env`.
4. Never modify the source STL files.
5. Document every transform, scale, collision margin, mass, inertia, joint
   limit, and controller value that is assumed rather than measured.
6. Validate from the installed ROS workspace, not only from source paths.
7. Use bounded waits and headless validation where possible.
8. If a stop-and-ask condition is reached, do not continue by guessing.
9. Mark only the assigned task complete after its acceptance criteria pass.
10. Leave a concise handoff with changed files, exact measurements, commands,
    results, assumptions, and unresolved questions.

## Dependency Order

```text
Task 1: Audit and map the CAD assets
    ↓
Task 2: Integrate the MiR visual and refit base collisions
    ↓
Task 3: Build the articulated UR visual and primitive collisions
    ↓
Task 4: Add arm topic control and joint feedback
    ↓
Task 5: Add tests and operator documentation
    ↓
Task 6: Perform final visual and physics regression
```

Tasks are sequential because later transforms and collisions depend on the
validated scale, orientation, and link mapping established earlier.

## Task 1 — Audit and Map the CAD Assets

- [ ] Complete

### Goal

Produce a verified CAD mapping before changing the robot description.

### Allowed Scope

- Read-only inspection of `burke_description/cad/stl/`
- A concise CAD mapping/assumptions section added to this plan or an existing
  repository documentation file
- No URDF, Xacro, CMake, bridge, launch, or STL changes

### Work

- Record each relevant STL's raw minimum, maximum, AABB size, triangle count,
  and import result.
- Validate the provisional `0.001` scale against the MiR and assembled UR
  extents.
- Determine the MiR rotation required to map its long axis to robot `+X`.
- Overlay or otherwise compare the seven split UR meshes with `UR8 Long.stl`.
- Produce an explicit table mapping each split STL to `arm_base_link` or
  `arm_link_1` through `arm_link_6`.
- Record all six pivot origins and joint axes in parent-link coordinates.
- Identify the platform visual origin and the proposed top-centre arm mount.
- Trigger a stop-and-ask condition if any required mapping is ambiguous.

### Acceptance Criteria

- All selected STLs import successfully.
- Scale, platform orientation, part order, six pivot origins, and six axes are
  either verified and recorded or the task is explicitly blocked pending a
  project-owner measurement.
- The assembled arm reference and split-part zero pose agree within a stated
  visual/numerical tolerance.
- Deferred CAD files remain excluded.

### Handoff

Provide the full mesh-to-link table, all transforms in metres/radians, the
chosen comparison tolerance, and screenshots or numeric overlay evidence.

## Task 2 — Integrate the MiR Visual and Refit Base Collisions

- [ ] Complete

### Prerequisite

Task 1.

### Goal

Use the MiR1350 STL for platform visualization and align stable primitive
collisions and drive contact geometry to its verified size.

### Allowed Scope

- `burke_description/CMakeLists.txt`
- `burke_description/urdf/burke_base.urdf.xacro`
- A focused Xacro component under `burke_description/urdf/components/`, if it
  materially improves readability
- No arm links, controllers, bridge changes, tests, README edits, or STL edits

### Work

- Install `cad/` with the description package.
- Add the MiR mesh visual with the verified package URI, scale, rotation, and
  offset.
- Eliminate duplicate/protruding primitive base visuals.
- Resize and reposition chassis collision primitives from the scaled bounds.
- Align the existing driven-wheel and passive-support collisions with the new
  body envelope while preserving differential-drive joint names and behavior.
- Recalculate primitive inertias consistently with any changed dimensions.

### Acceptance Criteria

- The installed model resolves the STL without source-tree paths or missing
  resource warnings.
- The visual measures approximately `1.350 × 0.910 × 0.321 m` after scaling
  and points forward along robot `+X`.
- The model rests above the ground without continuous drift, tipping, or
  explosive contact behavior.
- Collision visualization shows the main collision inside or closely matching
  the visual, with each margin documented.
- Positive `/cmd_vel` linear motion remains robot-forward and odometry works.

### Validation

```bash
colcon build --symlink-install --packages-select burke_description burke_gazebo
source install/setup.bash
xacro src/burke-sim/burke_description/urdf/burke_base.urdf.xacro > /tmp/burke_mir.urdf
check_urdf /tmp/burke_mir.urdf
timeout 30s ros2 launch burke_gazebo base_sim.launch.py gui:=false
```

Also perform one GUI collision-visualization check and a bounded straight and
rotational drive test followed by an explicit zero `/cmd_vel` command.

### Handoff

Record the visual transform, final collision dimensions and margins, wheel and
support changes, inertial changes, and base regression results.

## Task 3 — Build the Articulated UR Visual and Primitive Collisions

- [ ] Complete

### Prerequisites

Tasks 1–2.

### Goal

Add the seven mapped UR part visuals as a fixed base body plus six revolute
links mounted on the MiR, with primitive collision and inertia only.

### Allowed Scope

- `burke_description/urdf/burke_base.urdf.xacro`
- `burke_description/urdf/components/simple_arm.urdf.xacro`, if used
- Description install rules only if not completed by Task 2
- No Gazebo arm controllers, bridge changes, tests, README edits, or STL edits

### Work

- Implement the stable arm naming contract exactly.
- Apply Task 1's part mapping, visual origins, joint origins, axes, and scale.
- Mount the fixed arm base at the verified top-centre platform transform.
- Use one and only one split STL visual per arm rigid body.
- Fit simple collision primitives to each scaled part's occupied volume.
- Use multiple primitives only where one AABB-shaped primitive would cause
  obvious false self-collision or materially exceed the visual.
- Add positive assumed masses, primitive-derived inertias, finite joint
  position limits, velocity/effort limits, damping, and friction.
- Confirm that `arm_link_6` is terminal and contains no tool.

### Acceptance Criteria

- Xacro expands and validates with exactly six arm revolute joints.
- The zero-pose split visuals align with the assembled UR reference within the
  Task 1 tolerance.
- No part is duplicated, missing, mirrored, or unexpectedly scaled.
- Visual and collision origins remain explicit and auditable.
- Collision visualization follows each rigid body without obvious gaps,
  protrusions, or false contact at joint pivots.
- The arm is mounted above the platform and the stationary model remains
  physically stable in headless Gazebo.

### Validation

Run the Task 2 build, Xacro, URDF, and bounded launch checks. In the GUI, inspect
the zero pose from front, side, and top views with collision visualization
enabled. Compare the articulated zero pose against `UR8 Long.stl` without
shipping the assembled reference in the runtime model.

### Handoff

Record the final part/link mapping, all mount/joint/visual transforms, collision
primitive dimensions, assumed dynamics, limits, and visual comparison result.

## Task 4 — Add Topic-Based Arm Control

- [ ] Complete

### Prerequisite

Task 3.

### Goal

Control every arm joint independently through the defined ROS topics and expose
joint feedback without changing the CAD or collision design.

### Allowed Scope

- Gazebo system blocks in the robot Xacro
- `burke_gazebo/config/bridge.yaml`
- Required package dependency declarations
- No geometry changes except a minimal correction proven necessary by motion
  validation; no custom nodes or higher-level controller stack

### Work

- Add one native `JointPositionController` for each arm joint.
- Use model-scoped Gazebo command topics and the six stable ROS topic names.
- Configure bounded controller speed and conservative initial positions.
- Add Gazebo's native joint-state publisher and bridge `/joint_states`.
- Preserve all existing base and clock bridge entries.
- Exercise one joint at a time to detect crossed mappings, incorrect axes,
  false self-collisions, and unexpected mesh detachment.

### Acceptance Criteria

- All six ROS topics exist as `std_msgs/msg/Float64` inputs.
- `/joint_states` contains all six arm joints with changing positions.
- Each command moves only its intended joint toward the target.
- All links and visuals remain attached throughout motion.
- Joint position and speed limits are respected.
- `/cmd_vel`, `/odom`, `/tf`, and `/clock` still work.

### Validation

```bash
ros2 topic list -t
ros2 topic echo --once /joint_states
ros2 topic pub --once /arm/joint_1/command std_msgs/msg/Float64 "{data: 0.25}"
ros2 topic pub --once /arm/joint_2/command std_msgs/msg/Float64 "{data: -0.20}"
ros2 topic echo --once /joint_states
```

Repeat with a distinct small in-limit target for joints 3–6, then return all
six joints to zero and regression-test bounded base translation and rotation.

### Handoff

Record the ROS/Gazebo mapping, controller parameters, observed axis directions,
limit behavior, mesh behavior during motion, and base regression result.

## Task 5 — Add Integration Tests and Operator Documentation

- [ ] Complete

### Prerequisite

Task 4.

### Goal

Make CAD loading, arm control, joint feedback, and base motion repeatably
verifiable in a headless environment and document the manual workflow.

### Allowed Scope

- `burke_gazebo/test/`
- Test registration and test-only dependencies
- `README.md`
- Minimal corrections to prior-task files only when a test proves a defect
- No new features, CAD conversion, geometry redesign, or controller stack

### Work

- Add a description-level test that verifies all required `package://` mesh
  URIs resolve from the installed package and no deferred mesh is referenced.
- Verify the expanded robot has one MiR visual, seven split UR visuals, exactly
  six revolute arm joints, primitive-only collisions, and no tool link.
- Add a bounded headless integration test that commands each joint, checks
  `/joint_states`, verifies limits, drives the base, checks odometry, and sends
  explicit zero/reset commands during cleanup.
- Update the README with build, launch, topic, joint reset, collision-visual,
  and test instructions.
- Clearly state the `0.001` scale assumption, CAD visual-only policy, primitive
  collision approximations, and absence of the LiftKit and tool.

### Acceptance Criteria

- Tests fail clearly on missing meshes, bad scale references, missing topics,
  crossed joints, limit violations, or base regression.
- Every wait has a timeout and tests require no GUI or real hardware.
- README commands exactly match the implemented interfaces.
- A clean installed workspace, not the source tree, supplies all mesh assets.

### Validation

```bash
colcon build --symlink-install --packages-select burke_description burke_gazebo
source install/setup.bash
colcon test --packages-select burke_description burke_gazebo --event-handlers console_direct+
colcon test-result --verbose
```

Execute the README's launch, per-joint command, six-joint reset, base stop, and
test commands exactly as written.

### Handoff

List tests, timeouts, tolerances, results, README changes, and any validation
that could not run.

## Task 6 — Final Visual and Physics Regression

- [ ] Complete

### Prerequisites

Tasks 1–5.

### Goal

Perform an end-to-end review and correct only defects preventing this
milestone's acceptance criteria.

### Allowed Scope

- Inspect the complete milestone.
- Make minimal corrections within files already touched by Tasks 1–5.
- No new robot subsystem, asset, controller API, or unrelated refactor.

### Acceptance Criteria

- A clean build and complete automated test run pass.
- Installed mesh URIs resolve with no missing-resource errors.
- MiR and UR visuals have the verified scale, orientation, and placement.
- Collision primitives closely approximate the scaled visuals and remain
  stable without using STL collision meshes.
- The articulated zero pose agrees with the complete UR reference.
- All six joints move correctly through safe representative angles without
  mesh detachment or unintended collision locking.
- The arm remains attached while the MiR translates and rotates.
- Base drive, odometry, TF, clock, joint feedback, stop, and reset behavior all
  remain correct.
- No deferred LiftKit, payload, camera, control-box, or tool asset is present.
- Every non-CAD physical value and transform assumption is documented.

### Validation

Run the full Task 5 validation, then launch the GUI and inspect visuals and
collisions from front, side, top, and close joint views. Command each arm joint
to a distinct in-limit pose, return it to zero, drive and rotate the base, and
send an explicit zero base command. Review Gazebo logs for mesh, inertia,
collision, controller, and bridge warnings.

### Handoff

Report final scale and transforms, visual/collision QA, all joint motion
results, base regression, build/test output, remaining warnings, and any check
that could not be performed. Mark complete only when no required work remains.
