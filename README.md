# Burk-e simulation

This repository implements the Milestone 2 simulation environment defined in
[`IMPLEMENTATION_PLAN.md`](IMPLEMENTATION_PLAN.md): native Ubuntu 24.04
(Noble), ROS 2 Jazzy, and Gazebo Harmonic. This repository is already checked
out under the ROS 2 workspace at `ros_ws/src/burke-sim/`.

## Supported runtime

The supported runtime is a native Ubuntu 24.04 installation with the official
ROS 2 Jazzy apt repository and Gazebo Harmonic packages. Source ROS in every
new terminal; no host-global shell modification is required:

```bash
source /opt/ros/jazzy/setup.bash
cd /path/to/ros_ws
colcon build --symlink-install
source install/setup.bash
```

The workspace currently contains:

- `burke_description`, which owns the mobile-base and UR8 Long URDF/Xacro
  descriptions, CAD visuals, parameter files, and frames.
- `burke_gazebo`, which owns Gazebo worlds, launch files, bridge configuration,
  and simulation tests.

The current milestone mounts a six-joint UR8 Long description directly on the
mobile base through a three-stage LiftKit mast. Its nominal kinematics, public joint limits, masses, centres of
mass, and inertia tensors come from Universal Robots' official ROS 2
description at the recorded rolling-branch commit. Link-local, metre-scale
derivatives of the seven supplied UR8L STL parts provide visual geometry;
inexpensive cylinders derived from their bounds remain the collision geometry. The upstream source
warns that the nominal 4 kg arm-base mass may be inaccurate, and the mounting
pose on the provisional mobile platform remains a simulation assumption.

### Empty-world launch

After building and sourcing the workspace, launch the deterministic empty
world with its ground plane and directional sun:

```bash
ros2 launch burke_gazebo empty_world.launch.py
```

For a bounded server-only startup (useful on CI or without a display):

```bash
ros2 launch burke_gazebo empty_world.launch.py gui:=false
```

The world name is `burke_empty`; its physics uses fixed Earth gravity and a
1 kHz ODE update rate. The launch defaults to Gazebo's `ogre` render engine;
it also enables software OpenGL (`LIBGL_ALWAYS_SOFTWARE=true`) by default for
reliable startup in VMs. Override the render engine with
`render_engine:=...` when needed.

Build and verify the workspace scaffold from the `ros_ws/` workspace root:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 pkg prefix burke_description
ros2 pkg prefix burke_gazebo
```

Install the milestone dependencies on a clean Ubuntu 24.04 host with:

```bash
sudo apt update
sudo apt install \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-teleop-twist-keyboard \
  python3-colcon-common-extensions
```

### Gazebo GUI

Gazebo runs directly on the Ubuntu desktop. For an SSH or VM session, use the
desktop's normal X11/Wayland forwarding and ensure `DISPLAY` is set before
launching. An interactive terminal is required for `teleop_twist_keyboard`.

```bash
source /opt/ros/jazzy/setup.bash
gz sim
```

For CI or a VM without a display, use Gazebo's supported headless rendering
options; this validates simulation startup but does not replace the
interactive GUI demonstration required for the manual milestone.

### Combined base and arm

Launch the headless simulation with the arm attached:

```bash
ros2 launch burke_gazebo base_sim.launch.py gui:=false
```

Omit `gui:=false` for the interactive Gazebo view. Each arm command accepts a
target angle in radians as `std_msgs/msg/Float64`:

The simulated position controllers cap shoulder motion at `0.75 rad/s`, elbow
motion at `1.0 rad/s`, and wrist motion at `1.5 rad/s`. These conservative
simulation speeds and joint-specific damping reduce overshoot; the larger
values in `joint_limits.yaml` remain the declared hardware limits.

| Topic | Joint |
| --- | --- |
| `/arm/joint_1/command` | `arm_joint_1` |
| `/arm/joint_2/command` | `arm_joint_2` |
| `/arm/joint_3/command` | `arm_joint_3` |
| `/arm/joint_4/command` | `arm_joint_4` |
| `/arm/joint_5/command` | `arm_joint_5` |
| `/arm/joint_6/command` | `arm_joint_6` |

For example, command joint 2 to a small in-limit target and inspect feedback:

```bash
ros2 topic pub --rate 10 --qos-reliability reliable \
  /arm/joint_2/command std_msgs/msg/Float64 "{data: -0.20}"
# Press Ctrl-C after the joint reaches the target.
ros2 topic echo --once /joint_states
```

Return the arm to its documented zero pose by sending zero to all six scalar
topics:

```bash
for joint in 1 2 3 4 5 6; do
  ros2 topic pub --once /arm/joint_${joint}/command std_msgs/msg/Float64 "{data: 0.0}"
done
```

The arm has no tool, payload, trajectory-control API, inverse kinematics, or
hardware interface yet. It is intentionally limited to six independent
Gazebo position commands and `/joint_states` feedback.

### LiftKit mast

The lift is modelled as three nested stages but has two actuated prismatic
joints: `lift_stage_2_joint` and `lift_stage_3_joint`. Stage 1 is the fixed
mounting frame; the two commanded strokes are 0–0.275 m and 0–0.225 m. The
collapsed mast is 0.555 m high and the fully extended mast is 1.055 m high.
Commands are `std_msgs/msg/Float64` in metres:

| Topic | Joint | Range |
| --- | --- | --- |
| `/lift/stage_2/command` | `lift_stage_2_joint` | 0.0–0.275 m |
| `/lift/stage_3/command` | `lift_stage_3_joint` | 0.0–0.225 m |

The mast height is `0.555 + q_stage_2 + q_stage_3` metres. The arm mount is
`0.876230 + q_stage_2 + q_stage_3` metres above `base_link`. Thus collapsed,
half-height, and full-height targets are respectively `(0.0, 0.0)`,
`(0.1375, 0.1125)`, and `(0.275, 0.225)`.

For example, hold the two commands at the half-height target while inspecting
feedback:

```bash
ros2 topic pub --rate 10 /lift/stage_2/command std_msgs/msg/Float64 "{data: 0.1375}"
ros2 topic pub --rate 10 /lift/stage_3/command std_msgs/msg/Float64 "{data: 0.1125}"
ros2 topic echo --once /joint_states
```

The LiftKit CAD visuals use link-local, metre-scale derivatives of the supplied
STL files. STL files are deliberately excluded from collision geometry;
primitive perimeter boxes provide the inexpensive simulation collision
envelope. Stage masses, the 0.05 m/s stroke-speed cap, and the 2500 N·m effort
cap are simulation assumptions, not a validated hardware model.

The original CAD exports remain unchanged in `burke_description/cad/stl`.
Regenerate and verify all derived `*_link.stl` visual assets from the repository
root with:

```bash
python3 burke_description/scripts/bake_link_local_meshes.py --write
python3 burke_description/scripts/bake_link_local_meshes.py
```

The baking step applies the former URDF scale, rotation, and translation to
the STL vertices. Consequently the MiR, LiftKit, and UR8L CAD visuals all use
identity origins and unit scale in the generated robot description.

Operate the mast in this order: stop `/cmd_vel`, return the arm to its stow
pose `[0, -2.0944, 2.0944, -1.5708, 1.5708, 0]` radians, command the lift while
the base is stationary, then return both lift commands to zero before sending
any new base motion. Send an explicit zero `/cmd_vel` on shutdown.

### Base model

Launch the empty world with one primitive-geometry Burk-e base:

```bash
ros2 launch burke_gazebo base_sim.launch.py
```

Use `gui:=false` for a server-only launch. The model uses `+X` forward, `+Y`
left, and `+Z` up. Its frame tree starts at `base_footprint`, followed by
`base_link`, the `left_drive_wheel_link` and `right_drive_wheel_link`, and
front/rear passive support links. The wheel joints are
`left_drive_wheel_joint` and `right_drive_wheel_joint`.

The primitive base dimensions, contact coefficients, and drive limits remain
simulation assumptions. Arm kinematics, public limits, and inertial data are
loaded from `burke_description/config/ur8long`, while its collision cylinders
and platform mounting transform are simulation approximations.
`base_sim.launch.py` starts the system and the YAML bridge. The stable
ROS-facing interfaces are:

- `/cmd_vel`: `geometry_msgs/msg/Twist` (ROS to Gazebo)
- `/odom`: `nav_msgs/msg/Odometry` (Gazebo to ROS), frames `odom` → `base_footprint`
- `/tf`: `tf2_msgs/msg/TFMessage` (Gazebo to ROS)

The current Gazebo transport topics are also global (`/cmd_vel`, `/odom`, and
`/tf`); operators should use the ROS-facing names above. Send a bounded command
and then an explicit stop with:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
ros2 topic echo /odom
```

### Keyboard teleoperation

Use two terminals. In the first terminal, launch the complete simulation:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/ros_ws/install/setup.bash
ros2 launch burke_gazebo base_sim.launch.py
```

In the second terminal, start the interactive teleoperator with conservative
initial speeds:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/ros_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p speed:=0.2 -p turn:=0.4
```

The teleoperator uses `i` for forward, `,` for reverse, `j` for left, and `l`
for right. Press `k` to send a zero command and stop the base. Press `Ctrl-C`
to exit teleoperation, then send an explicit zero command from either terminal
so the base cannot continue under a stale command:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

The simulation launch owns Gazebo, the robot description, spawning, and all
ROS–Gazebo bridges. Stop it with `Ctrl-C` in the first terminal before
relaunching; launch shutdown removes the spawned model and bridge processes.

### Front-deck 3D lidar

The generic spinning lidar is an obstacle-observation sensor fixed to the
MiR1350 front deck. It is separate from the TCP-mounted Gemini depth camera,
the front/rear obstacle cameras, and the MiR localization cameras. Its frame
tree is:

```text
base_link
└── mir_3d_lidar_mount_joint
    └── mir_3d_lidar_link
        └── mir_3d_lidar_frame_joint
            └── mir_3d_lidar_frame
```

The approved provisional mount is relative to `base_link`:

```text
translation: x=0.400 m, y=0.000 m, z=0.381230 m
rotation:    roll=0, pitch=0, yaw=0
```

The housing is a primitive cylinder with diameter `0.120 m`, height `0.100 m`,
and assumed mass `2.0 kg`. The lidar frame is zero-offset from the housing
link, scans along `+Z`, and remains fixed to the mobile base while the LiftKit
and arm move.

The initial simulation profile is 640 horizontal samples over `-pi..+pi`, 16
vertical samples over `-15..+15` degrees, 10 Hz, range `0.20..50.0 m`, range
resolution `0.010 m`, and Gaussian range noise with mean `0.0 m` and standard
deviation `0.010 m`. Visualization is disabled by default for headless
performance. These values and the housing dimensions are simulation
assumptions, not a commissioned physical lidar model or vendor profile.

The stable ROS interface is `/lidar/points` with type
`sensor_msgs/msg/PointCloud2`, frame `mir_3d_lidar_frame`, and sensor-data QoS
(`BEST_EFFORT`, `VOLATILE`). Because the publisher uses sensor-data QoS, use a
matching subscription when inspecting the cloud:

```bash
ros2 topic info --verbose /lidar/points
ros2 topic echo --qos-reliability best_effort --once /lidar/points
ros2 topic hz --qos-reliability best_effort /lidar/points
```

The Gazebo source topics are `/model/burke_base/lidar/points` for the native
range scan and `/model/burke_base/lidar/points/points` for
`gz.msgs.PointCloudPacked`; only the packed cloud is bridged to ROS. In RViz2,
add a `PointCloud2` display, select `/lidar/points`, set the Fixed Frame to
`mir_3d_lidar_frame` (or a connected world frame), and choose a suitable point
size and decay time.

If headless rendering is too slow, reduce `horizontal_samples`,
`vertical_samples`, or `update_rate` in the `mir_3d_lidar` instantiation in
`burke_description/urdf/burke_base.urdf.xacro`. Keep the ROS topic and frame
names unchanged when tuning performance. The generic profile does not provide
vendor-specific packets, ring timing, intensity calibration, or hardware
mounting guarantees.

### Foxglove robot-model configuration

Use one URDF custom layer for the robot. Delete older robot-model/URDF layers
before creating it so saved per-link offsets and transform history cannot be
reused. Configure the new layer as follows:

| Setting | Value |
| --- | --- |
| Source | Topic `/robot_description` |
| Control mode | Joint states |
| Joint states | `/joint_states` |
| Display mode | Visual |
| Frame prefix | empty |
| 3D display frame | `odom` |
| Mesh "up" axis | Z-up |

The Gazebo joint-state publisher supplies all ten movable joints in every
message: two wheels, two LiftKit stages, and six UR8L joints. Joint-state
control therefore lets Foxglove perform forward kinematics directly from the
URDF and avoids dependence on previously cached dynamic transforms. Switch the
same layer between `Visual` and `Collision` when checking alignment; do not use
separate layers with different control modes for that comparison.

The mesh up-axis setting is required because STL does not encode an up-axis.
Burk-e's link-local meshes use ROS coordinates (`+Z` up), while a Foxglove
panel configured for `Y-up` rotates mesh-backed visuals during loading. URDF
primitives such as the lidar housing and collision boxes do not pass through
that mesh conversion, so they can remain correct while every CAD visual is
misaligned.

Transform control remains available for diagnosing `/tf`, but a newly
connected client can have no history for the volatile movable-joint transforms.
The fixed transforms on `/tf_static` are transient-local and do not have that
limitation. If transform control is required, confirm the live tree first:

```bash
ros2 run tf2_ros tf2_echo base_footprint arm_link_6
ros2 topic info /tf --verbose
ros2 topic info /joint_states --verbose
```

One simulation should report one `/joint_states` publisher and two expected
`/tf` publishers: `robot_state_publisher` for robot joints and `ros_gz_bridge`
for `odom` to `base_footprint`.

## Runtime validation

Run these commands from a fresh terminal after sourcing ROS:

```bash
ros2 --help
ros2 pkg prefix ros_gz_sim
ros2 pkg prefix ros_gz_bridge
ros2 pkg prefix xacro
ros2 pkg prefix robot_state_publisher
ros2 pkg prefix teleop_twist_keyboard
gz sim --versions
```

The expected versions are ROS 2 `jazzy` and Gazebo `Harmonic`. The exact patch
versions are resolved by Ubuntu's package metadata; the ROS distribution and
Gazebo release are pinned at the distribution level.

## Assumptions and boundaries

- Native Ubuntu is the selected runtime; source `/opt/ros/jazzy/setup.bash` in
  each terminal.
- GUI availability depends on the Ubuntu desktop or its configured display
  forwarding.
- No real-device APIs, credentials, private network addresses, or hardware
  control are part of this environment.
