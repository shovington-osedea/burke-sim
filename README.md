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
mobile base. Its nominal kinematics, public joint limits, masses, centres of
mass, and inertia tensors come from Universal Robots' official ROS 2
description at the recorded rolling-branch commit. The seven supplied UR8L
STL parts are visual geometry at a millimetre-to-metre scale of `0.001`;
inexpensive cylinders
derived from their bounds remain the collision geometry. The upstream source
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
