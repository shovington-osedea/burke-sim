# Foxglove Remote UI Integration Plan

## Objective

Add an opt-in Foxglove connection to the Burk-e ROS 2 simulation running in
the Ubuntu 24.04 Parallels virtual machine. The completed integration must:

- run `foxglove_bridge` locally inside the Ubuntu VM;
- expose the bridge on a configurable TCP address and port, using
  `0.0.0.0:8765` for direct access from another machine;
- allow Foxglove Desktop or the Foxglove web application on another machine to
  connect with `ws://<reachable-vm-or-host-address>:8765`;
- visualize the robot model, TF, odometry, joint states, and 3D lidar point
  cloud produced by the existing simulation;
- avoid requiring ROS 2, Gazebo, or project message packages on the UI machine;
- keep external access disabled during ordinary simulation launches unless the
  operator deliberately selects the Foxglove launch;
- default to a visualization-only interface with no remote command, service,
  or parameter-write capability;
- preserve the existing Gazebo, ROS-Gazebo bridge, robot, lidar, arm, lift, and
  base interfaces; and
- clearly separate repository changes from Parallels, Ubuntu firewall, host
  firewall, router, VPN, and client-network configuration.

The primary scope is a direct connection over a trusted local network. Cloud
remote access, public Internet exposure, TLS termination, authentication,
teleoperation, and fleet management are not included unless explicitly
requested later.

## Architecture

```text
Gazebo Harmonic
    |
    | Gazebo Transport
    v
ros_gz_bridge
    |
    | ROS 2 Jazzy topics in the Ubuntu VM
    v
foxglove_bridge on 0.0.0.0:8765
    |
    | Foxglove WebSocket over TCP
    v
Parallels virtual network / host firewall / LAN
    |
    v
Foxglove Desktop or Chrome on another machine
```

The Foxglove UI does not connect to DDS directly. Only the WebSocket bridge
must be reachable from the UI machine.

## Existing Baseline

Agents must extend the repository as it exists when their task starts:

- The supported environment is Ubuntu 24.04, ROS 2 Jazzy, and Gazebo Harmonic
  running on ARM64 inside Parallels.
- `burke_gazebo/launch/base_sim.launch.py` starts Gazebo, publishes
  `robot_description`, spawns `burke_base`, and starts `ros_gz_bridge`.
- `burke_gazebo/config/bridge.yaml` exposes the simulation topics to ROS 2.
- The current visualization topics include `/robot_description`, `/tf`,
  `/clock`, `/odom`, `/joint_states`, and `/lidar/points`.
- `burke_gazebo/package.xml` does not currently depend on
  `foxglove_bridge`.
- `ros-jazzy-foxglove-bridge` is not installed in the current VM. An ARM64
  package candidate is available from the configured official ROS repository,
  so the apt package is the preferred installation path.
- Upstream Foxglove bridge Docker images are documented as Linux AMD64 only;
  do not introduce Docker for this ARM64 environment when the native Jazzy
  package is available.
- `scripts/stop_burke_sim.bash` stops the existing Gazebo and ROS processes but
  does not currently include the Foxglove bridge.
- The worktree contains ongoing lidar and simulation changes. Preserve them and
  do not revert, rewrite, or weaken them as part of this milestone.

Do not replace `ros_gz_bridge` with Foxglove Bridge. The first bridge converts
Gazebo messages into ROS 2 messages; Foxglove Bridge then streams selected ROS
2 messages to the remote UI.

## Connection and Security Defaults

Use the following initial interface contract:

| Setting | Default | Reason |
| --- | --- | --- |
| Launch | Dedicated `foxglove_sim.launch.py` | Avoid exposing a port during ordinary simulation runs |
| Bind address | `0.0.0.0` | Accept connections on any VM interface when explicitly launched |
| TCP port | `8765` | Foxglove Bridge default and documented client convention |
| Client URL | `ws://<reachable-address>:8765` | Direct local-network WebSocket connection |
| Mode | Visualization only | Do not allow remote control implicitly |
| Hidden topics | Disabled | Avoid exposing internal topics unnecessarily |
| Client publishing | Disabled | Prevent remote `/cmd_vel` or joint commands |
| Services | Disabled | Prevent remote service calls |
| Parameter writes | Disabled | Prevent remote configuration changes |
| Assets | Enabled with a narrow allowlist | Allow the 3D panel to retrieve Burk-e meshes |
| Client count | Enabled | Provide a simple connection diagnostic |

Use an explicit topic allowlist. The minimum visualization set is:

```text
/clock
/tf
/tf_static
/robot_description
/odom
/joint_states
/lidar/points
/foxglove_bridge/client_count
```

Add another topic only when a documented Foxglove panel needs it. Do not expose
command topics such as `/cmd_vel`, `/arm/.../command`, or `/lift/.../command`
under the default profile.

Retain only the Foxglove capabilities needed for connection-graph inspection
and robot-model asset retrieval. Configure `service_whitelist`,
`param_whitelist`, and `client_topic_whitelist` to match nothing. Restrict
`asset_uri_allowlist` to the installed `burke_description` resources required
by `robot_description`; do not allow arbitrary `file://` retrieval.

## Network Responsibility Boundary

Repository code can bind the bridge to a TCP socket and prove that it accepts a
local connection. It cannot guarantee that another physical machine can reach
that socket.

The current managed environment prevents inspection of network interfaces,
routes, listening sockets, and system firewall state through its sandbox. It
also has no access to the Parallels network-mode settings, the host operating
system firewall, the LAN router, Wi-Fi client-isolation policy, VPN policy, or
the intended Foxglove client machine. Therefore implementation must not claim
that port `8765` is externally reachable based only on a successful local
test.

The operator owns these external steps:

1. Select the appropriate Parallels networking mode.
2. Determine the reachable VM or host address.
3. Add a Parallels port-forwarding rule when NAT/shared networking requires it.
4. Allow TCP `8765` through Ubuntu and host firewalls when necessary.
5. Confirm that the client machine and selected address are mutually routable.
6. Test the port from the client machine.

Use this routing guide:

| Parallels/network state | Expected action | Client URL |
| --- | --- | --- |
| Bridged VM networking | Use the VM's LAN address | `ws://<VM_LAN_IP>:8765` |
| Shared/NAT networking with guest reachable from host only | Forward host TCP `8765` to guest TCP `8765` | `ws://<HOST_LAN_IP>:8765` |
| Host-only networking | Change network mode, add a host proxy/tunnel, or use Foxglove remote access | Depends on chosen route |
| Same machine as VM | Connect locally for diagnosis | `ws://127.0.0.1:8765` |
| Different routed network/VPN | Add an approved route or VPN rule | `ws://<ROUTABLE_IP>:8765` |

Do not automatically open a broad firewall rule, modify Parallels settings, or
configure router port forwarding. Provide exact diagnostic evidence and stop
at this boundary so the user can perform those actions.

Direct Foxglove WebSocket connections currently require a Foxglove developer
seat. If the account cannot open a direct connection, the user must choose
either an appropriate Foxglove plan or a separately approved remote-access
design. Foxglove remote access routes outbound through the Foxglove platform
and can work behind firewalls, but it requires account/device-token setup and
is not part of this local-LAN implementation.

## Stop-and-Ask Conditions

Stop and request direction instead of expanding scope if:

- the UI machine is outside the trusted LAN and no VPN is available;
- the user wants to expose port `8765` directly to the public Internet;
- Foxglove must publish commands, call services, modify parameters, or
  teleoperate the robot;
- TLS, authentication, a reverse proxy, or certificate management is required;
- a Foxglove device token or cloud remote access is requested;
- the direct WebSocket option is unavailable for the user's Foxglove account;
- port `8765` is already occupied;
- Parallels is configured in a mode that cannot route or forward to the guest;
- the external client cannot reach the selected address after local bridge and
  firewall checks succeed; or
- lidar bandwidth causes unacceptable simulation or UI performance and the
  acceptable fidelity reduction is unknown.

Report the observed error, the address and port tested, the test location, and
the exact external change or authorization required.

## Agent Working Rules

1. Read `AGENTS.md`, this plan, the current worktree, and prerequisite handoffs
   before editing.
2. Preserve ongoing lidar and simulation changes and all existing interfaces.
3. Keep Foxglove startup opt-in; do not expose a network listener from the
   ordinary base simulation launch without an explicit launch argument.
4. Never store account credentials, device tokens, private keys, host-specific
   addresses, or private network details in the repository.
5. Default to read-only visualization and least-privilege allowlists.
6. Use a configurable address and port; do not hard-code a discovered VM IP.
7. Use bounded waits and a non-default test port for automated tests so a stale
   process cannot make tests pass or block the normal operator port.
8. Validate installed package resources, not only source-tree files.
9. Do not modify firewall, Parallels, router, or remote-machine settings during
   repository implementation.
10. Mark a task complete only after its acceptance criteria pass and record
    exact commands, results, assumptions, and external blockers.

## Dependency Order

```text
Task 1: Install and prove the native Foxglove Bridge locally
    |
    v
Task 2: Add a secure opt-in launch and bridge configuration
    |
    v
Task 3: Add local automated validation and lifecycle handling
    |
    v
Task 4: Document and hand off Parallels/LAN reachability
    |
    v
Task 5: Validate the UI from another machine
```

Tasks 1–3 are repository and VM-local work. Tasks 4–5 require operator access
to Parallels, firewall settings, and the client machine and cannot be declared
complete solely from inside the VM.

## Task 1 — Install and Prove Foxglove Bridge Locally

- [x] Complete

### Goal

Confirm that the official ROS 2 Jazzy Foxglove Bridge package works on the
Ubuntu 24.04 ARM64 VM before changing project launch files.

### Allowed Scope

- Read-only environment and package inspection
- Installation of `ros-jazzy-foxglove-bridge` after normal package-manager
  approval
- Temporary logs under `/tmp`
- This task's handoff section
- No repository source changes

### Work

1. Source `/opt/ros/jazzy/setup.bash` and the Burk-e workspace.
2. Install the native apt package:

   ```bash
   sudo apt update
   sudo apt install ros-jazzy-foxglove-bridge
   ```

3. Confirm `ros2 pkg prefix foxglove_bridge` resolves.
4. Start the existing simulation headlessly.
5. Start Foxglove Bridge independently on loopback and a temporary port:

   ```bash
   ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
     address:=127.0.0.1 port:=8766
   ```

6. Confirm the process remains alive, reports the expected bind address and
   port, discovers the required ROS topics, and accepts a local TCP connection.
7. Connect Foxglove on the same machine when a usable GUI is available; this is
   optional for the headless smoke test.
8. Stop the bridge and simulation cleanly and verify the test port is released.

### Acceptance Criteria

- The native Jazzy package installs and resolves on ARM64.
- Foxglove Bridge starts without schema, DDS, or WebSocket initialization
  errors.
- A local client can connect to `127.0.0.1:8766`.
- The bridge discovers `/robot_description`, `/tf`, `/odom`, `/joint_states`,
  and `/lidar/points` while the simulation is running.
- No Docker container, source build, account credential, or external network
  change is required.

### Handoff

- [x] Complete (2026-08-15)
- Installed `ros-jazzy-foxglove-bridge` version
  `3.4.1-1noble.20260612.130454` from the configured official ROS 2 apt
  repository; the installed architecture is `arm64`.
- `ros2 pkg prefix foxglove_bridge` resolves to `/opt/ros/jazzy` and the
  installed launch resource is
  `/opt/ros/jazzy/share/foxglove_bridge/launch/foxglove_bridge_launch.xml`.
- Independent bridge command:

  ```bash
  ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
    address:=127.0.0.1 port:=8766
  ```

- Integrated headless validation used the existing
  `ros2 launch burke_gazebo base_sim.launch.py gui:=false` command and the
  bridge command above. The bridge reported `Server listening on port 8766`;
  a local TCP client connected successfully.
- While the simulation was running, the required topics were present:
  `/robot_description`, `/tf`, `/odom`, `/joint_states`, and `/lidar/points`.
  `/clock` and `/tf_static` were also present.
- Temporary logs are stored at `/tmp/burke-task1-bridge.log`,
  `/tmp/burke-task1-bridge-sim.log`, `/tmp/burke-task1-sim.log`, and under
  `/tmp/burke-task1-ros-log*`.
- The bridge shut down cleanly and port `8766` was successfully rebound after
  cleanup. Gazebo emitted a shutdown-time segmentation-fault message under
  the managed VM execution path; this is an existing simulation lifecycle
  observation and did not leave the bridge or test port running.
- No Docker container, source build, credential, or external network change
  was used. Foxglove GUI validation was not performed; the headless TCP smoke
  test is the applicable local proof for this task.

## Task 2 — Add the Opt-In Foxglove Launch and Configuration

- [x] Complete

### Prerequisite

Task 1.

### Goal

Provide one supported launch command that starts the complete Burk-e simulation
and a least-privilege Foxglove WebSocket server.

### Allowed Scope

- New `burke_gazebo/launch/foxglove_sim.launch.py`
- New `burke_gazebo/config/foxglove_bridge.yaml`
- `burke_gazebo/package.xml`
- Minimal launch/config installation corrections in `burke_gazebo/CMakeLists.txt`
- No robot description, Gazebo world, ROS-Gazebo topic mapping, controller, or
  sensor changes

### Work

1. Add `foxglove_bridge` as an execution dependency.
2. Create a dedicated launch file that includes `base_sim.launch.py` and starts
   Foxglove Bridge only from this opt-in entry point.
3. Declare launch arguments:
   - `gui`, forwarded to the base simulation;
   - `foxglove_address`, default `0.0.0.0`;
   - `foxglove_port`, default `8765`; and
   - an optional `foxglove_config` path for test overrides.
4. Load `config/foxglove_bridge.yaml` with `use_sim_time: true`.
5. Configure the exact topic allowlist from this plan.
6. Disable client publishing, services, and parameter access with allowlists
   that match nothing and remove the corresponding capabilities.
7. Retain connection-graph and asset access only as required by Foxglove's 3D
   panel.
8. Restrict assets to `package://burke_description/...` resources and supported
   robot visual file extensions.
9. Enable `/foxglove_bridge/client_count` for diagnosis.
10. Ensure the lidar's best-effort QoS is handled correctly and measure whether
    default send-buffer and compression settings are sufficient before tuning.
11. Keep the simulation interfaces unchanged when launched directly; the
    Foxglove bridge is enabled by default now that the operator has requested
    default exposure, with `foxglove:=false` available for local opt-out.

### Operator Command

```bash
source /opt/ros/jazzy/setup.bash
source /home/parallels/ros_ws/install/setup.bash
ros2 launch burke_gazebo foxglove_sim.launch.py \
  gui:=false foxglove_address:=0.0.0.0 foxglove_port:=8765
```

### Acceptance Criteria

- The dedicated launch starts Gazebo, the robot, `ros_gz_bridge`, and
  `foxglove_bridge` together.
- The server binds the requested address and port.
- Launching `base_sim.launch.py` directly starts Foxglove Bridge by default;
  `foxglove:=false` disables it explicitly.
- The UI can read only the allowlisted topics and required robot assets.
- A client cannot advertise command topics, invoke services, or modify
  parameters under the default configuration.
- No host-specific IP, credential, or secret is checked into the repository.
- Existing simulation launch and topic behavior remain unchanged.

### Handoff

- [x] Complete (2026-08-15)
- Changed files:
  `burke_gazebo/launch/foxglove_sim.launch.py`,
  `burke_gazebo/config/foxglove_bridge.yaml`, and
  `burke_gazebo/package.xml`. The existing CMake install rule already installs
  both `launch` and `config` directories, so no CMake change was required.
- Launch arguments are `gui` (default `true`), `foxglove_address` (default
  `0.0.0.0`), `foxglove_port` (default `8765`), and `foxglove_config` (the
  installed project YAML by default). The launch includes `base_sim.launch.py`
  and starts Foxglove Bridge only from this dedicated entry point.
- The default topic regex is
  `^/(?:clock|tf|tf_static|robot_description|odom|joint_states|lidar/points|foxglove_bridge/client_count)$`.
  `client_topic_whitelist`, `service_whitelist`, and `param_whitelist` are
  each `^$`, which matches no usable name. Capabilities are limited to
  `connectionGraph` and `assets`; hidden topics and sysinfo are disabled.
- Asset retrieval is limited to
  `^package://burke_description/cad/stl/[A-Za-z0-9_.%-]+[.]stl$`.
  Lidar best-effort QoS is explicitly allowed with
  `^/lidar/points$`; send-buffer and compression defaults were not tuned.
- Validation command:

  ```bash
  colcon build --symlink-install --packages-select burke_gazebo
  ros2 launch burke_gazebo foxglove_sim.launch.py \
    gui:=false foxglove_address:=127.0.0.1 foxglove_port:=8766
  ```

- The installed launch started Gazebo, `ros_gz_bridge`, and Foxglove Bridge;
  `/foxglove_bridge/client_count` plus all required visualization topics were
  advertised, and a TCP client connected to the `ws://127.0.0.1:8766` listener.
  The default `base_sim.launch.py gui:=false` behavior was separately checked
  on a test port and opened the Foxglove listener.
- This validates the VM-local listener and policy only. No Parallels, firewall,
  router, VPN, host-network, credential, or external reachability changes were
  made. Runtime logs are under `/tmp/burke-task2-foxglove.log` and
  `/tmp/burke-task2-base.log`.

## Task 3 — Add Local Tests and Lifecycle Handling

- [x] Complete

### Prerequisite

Task 2.

### Goal

Make the Foxglove launch, listener, topic policy, and cleanup behavior
repeatably verifiable from inside the VM.

### Allowed Scope

- New `burke_gazebo/test/test_foxglove_bridge.py`
- Test registration and test-only dependencies
- `scripts/stop_burke_sim.bash`
- `scripts/burke_sim_aliases.bash` only if a Foxglove-specific helper is useful
- Minimal Task 2 corrections when tests prove a defect
- No external network or firewall changes

### Test Requirements

Add bounded tests that:

1. verify `foxglove_bridge` resolves from the installed environment;
2. parse the installed configuration and assert the expected address, test
   port override, topic allowlist, disabled write paths, and asset restriction;
3. launch `foxglove_sim.launch.py` headlessly on `127.0.0.1:8766`;
4. wait for the Foxglove node and required ROS topics with explicit timeouts;
5. establish a local TCP connection to `127.0.0.1:8766`;
6. perform a Foxglove WebSocket handshake if it can be done with a stable,
   packaged dependency; otherwise leave protocol/UI validation to Task 5;
7. verify `/foxglove_bridge/client_count` changes when a compatible client is
   connected, when protocol validation is available;
8. prove that the ordinary `base_sim.launch.py` creates a listener by default
   and that `foxglove:=false` disables it;
9. terminate all launched processes and prove the test port can be rebound;
10. retain all existing arm, lift, base, lidar, and ROS-Gazebo tests; and
11. use actionable failure messages that distinguish bridge failure from
    external network reachability.

Update `scripts/stop_burke_sim.bash` so a Foxglove-enabled simulation shutdown
also terminates the bridge. Keep its scope narrow enough that it does not kill
unrelated WebSocket applications.

### Acceptance Criteria

- A clean build and all registered tests pass.
- Tests use loopback and a non-production port and require no LAN access.
- A stale bridge cannot make the test pass.
- Cleanup releases the port and leaves no Foxglove process from the test.
- Existing tests are not skipped, weakened, or reordered to hide regressions.
- Test output clearly says that local success does not prove remote reachability.

### Validation

```bash
colcon build --symlink-install --packages-select burke_description burke_gazebo
source install/setup.bash
colcon test --packages-select burke_description burke_gazebo --event-handlers console_direct+
colcon test-result --verbose
```

### Handoff

- [x] Complete (2026-08-15)
- Added `burke_gazebo/test/test_foxglove_bridge.py` and registered it in
  `burke_gazebo/CMakeLists.txt`; added `python3-yaml` as a test dependency.
- Tests use loopback port `8766` with a 30-second readiness timeout and
  bounded process cleanup. They parse the installed YAML, verify the package
  resolves, check the default base launch listener, verify all visualization
  topics, establish a local TCP connection, and rebind the port after cleanup.
  A WebSocket protocol handshake was not added because no stable packaged
  client dependency is required for this local milestone.
- Existing arm and lidar tests were retained and explicitly opt out with
  `foxglove:=false` so they do not contend for the production listener while
  testing unrelated simulation interfaces.
- `scripts/stop_burke_sim.bash` now stops the specific native
  `/foxglove_bridge/foxglove_bridge` executable in addition to the existing
  Gazebo process patterns; it does not target generic WebSocket processes.
- Validation passed:
  `colcon build --symlink-install --packages-select burke_gazebo`, complete
  `colcon test --packages-select burke_gazebo`, and
  `colcon test-result --verbose` reported `11 tests, 0 errors, 0 failures,
  0 skipped`. Local success does not prove Parallels, firewall, LAN, or
  remote Foxglove reachability; those remain deferred to Tasks 4–5.

## Task 4 — Document and Hand Off External Reachability

- [ ] Complete

### Prerequisite

Task 3.

### Goal

Give the user exact, non-destructive steps to make the VM's Foxglove listener
reachable without changing Parallels or firewall settings automatically.

### Allowed Scope

- `README.md`
- Optional read-only diagnostic script that prints commands and results without
  changing networking
- No firewall rule, Parallels preference, router, VPN, or remote-machine change

### Documentation Requirements

Document:

1. installation of `ros-jazzy-foxglove-bridge`;
2. the opt-in launch command and every launch argument;
3. local connection testing with `ws://127.0.0.1:8765`;
4. how the user can obtain the guest IP with `ip -brief address` and inspect the
   route with `ip route` outside the managed sandbox;
5. the bridged, shared/NAT, and host-only Parallels cases from this plan;
6. how to configure a host-to-guest TCP `8765` port forward in the appropriate
   Parallels version when shared networking is used;
7. a narrowly scoped Ubuntu firewall example, such as allowing TCP `8765` only
   from the intended client IP, while requiring the user to review and run it;
8. the need to check the host firewall and Wi-Fi/VPN client isolation;
9. how to test from the remote machine with `nc -vz <address> 8765` or an
   equivalent TCP client;
10. the Foxglove connection URL for direct guest and host-forwarded cases;
11. the difference between local listener success and end-to-end reachability;
12. the direct-connection account/seat requirement;
13. the security implications of unencrypted `ws://` and why it is limited to
    a trusted LAN; and
14. troubleshooting by failure layer: bridge process, VM listener, Ubuntu
    firewall, Parallels route/forward, host firewall, LAN, and Foxglove client.

Do not put a discovered private address into committed examples. Use
`<VM_LAN_IP>`, `<HOST_LAN_IP>`, and `<CLIENT_IP>` placeholders.

### Acceptance Criteria

- A user can identify which Parallels networking case applies.
- Every network-changing command is clearly labeled as an operator action.
- The documentation never claims that repository code opened the external
  path.
- Troubleshooting identifies the exact boundary at which traffic fails.
- No broad `0.0.0.0/0` firewall example, credential, or public exposure is
  recommended.

### Handoff

Report documentation changes, the detected limitation of the managed
environment, local diagnostic results, and the exact external steps left for
the user.

## Task 5 — Validate Foxglove from Another Machine

- [ ] Complete

### Prerequisites

Tasks 1–4 and user-completed network configuration.

### Goal

Prove the final experience from the actual Foxglove UI machine without
expanding the bridge's read-only permissions.

### User-Provided Inputs

- Confirmation that the client has Foxglove Desktop or Chrome access
- A Foxglove account/seat that supports the chosen connection type
- The reachable VM or host address
- Confirmation that Parallels and firewall configuration is complete

### Work

1. Start the opt-in Foxglove simulation in the VM.
2. From the UI machine, verify TCP reachability to the chosen address and port.
3. In Foxglove, select **Foxglove WebSocket** and connect to:

   ```text
   ws://<reachable-vm-or-host-address>:8765
   ```

4. Confirm the Topics panel contains only the intended visualization topics.
5. Configure a 3D panel with `odom` as the initial display frame.
6. Display the URDF robot model, TF frames, and `/lidar/points`.
7. Add plots or raw-message panels for `/odom` and `/joint_states`.
8. Drive the MiR and command the arm/lift locally in the VM; confirm the remote
   UI updates while remote command publication remains unavailable.
9. Observe latency, dropped updates, bridge send-buffer warnings, VM CPU/GPU
   load, and point-cloud responsiveness.
10. If lidar traffic is excessive, tune bridge compression/buffering or the
    simulated lidar rate only after recording the bottleneck and preserving the
    required visualization fidelity.
11. Disconnect the UI, stop the simulation with the repository cleanup helper,
    and confirm the connection closes and port is released.

### Acceptance Criteria

- The remote machine connects without installing ROS 2 or Gazebo.
- Robot model, TF, odometry, joint states, and 3D lidar data render correctly.
- Simulation-time updates remain coherent across panels.
- Remote clients cannot publish motion commands, call services, or modify
  parameters with the default profile.
- The UI remains responsive at the documented lidar rate on the intended LAN.
- Shutdown removes the Foxglove listener and all simulation-owned processes.
- Any Parallels, firewall, LAN, account, or client restriction is reported as
  an external blocker rather than hidden by repository changes.

### Handoff

Report the connection path used, address category without committing the
private address, client type, panels tested, observed latency and resource use,
read-only policy verification, shutdown result, external settings changed by
the user, and any remaining blocker.

## Final Validation Commands

Inside the Ubuntu VM:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/parallels/ros_ws
colcon build --symlink-install --packages-select burke_description burke_gazebo
source install/setup.bash
ros2 launch burke_gazebo foxglove_sim.launch.py \
  gui:=false foxglove_address:=0.0.0.0 foxglove_port:=8765
```

From the remote UI machine, after the user configures routing and firewalls:

```bash
nc -vz <reachable-vm-or-host-address> 8765
```

Then connect Foxglove to:

```text
ws://<reachable-vm-or-host-address>:8765
```

## Reference Sources

- Foxglove ROS 2 setup and direct connection instructions:
  <https://docs.foxglove.dev/docs/getting-started/frameworks/ros2>
- Foxglove Bridge installation, launch, address, port, allowlist, capability,
  and asset configuration:
  <https://github.com/foxglove/foxglove-sdk/blob/main/ros/src/foxglove_bridge/README.md>
- Foxglove direct WebSocket versus managed remote-access behavior:
  <https://docs.foxglove.dev/docs/visualization/connecting/live>
