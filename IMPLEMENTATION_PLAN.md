# Gazebo Integration Plan for `bombardier_burk-e_monorepo`

## Objective

Integrate the current Burk-e Gazebo simulation into
`Osedea/bombardier_burk-e_monorepo` while keeping Gazebo isolated and optional.

The monorepo must retain two explicit development paths:

1. **Deterministic simulation (default)** — the existing in-memory service
   adapters remain the default. ROS and Gazebo are not installed, started, or
   contacted.
2. **Gazebo-backed simulation (opt-in)** — MiR1350, UR8L, and LiftKit commands
   and observations are backed by native ROS 2 Jazzy/Gazebo Harmonic. Services
   without a Gazebo implementation remain on their deterministic adapters.

This plan was reevaluated on **2026-08-19** against monorepo `main` commit
`58caed59e7bfc92dbe0914b5f1f94c1b2db7eecc`. The previous plan was based on
`908e7bc`; implementation must not use that older tree as its baseline.

## Confirmed Decisions

- Gazebo source belongs in the monorepo under `simulation/gazebo/`.
- ROS 2 and Gazebo remain native on Ubuntu 24.04 for this milestone.
- The application stack continues to run through Docker Compose.
- Plain `docker compose up --build` keeps the deterministic backend and has no
  ROS/Gazebo dependency.
- A separate Compose override selects Gazebo explicitly.
- MiR1350, UR8L, and LiftKit are the first Gazebo-backed devices.
- Payload capture, surface waypoints, planning, defect detection, reporting,
  and Safe PLC projection remain deterministic until separately implemented.
- Both backends use `BURKE_HARDWARE_MODE=simulation`. Gazebo must never select
  or imply hardware mode.
- Existing Control API, Robot Orchestrator, device-service, Operational
  Supervision, command, actor, idempotency, and persistence boundaries remain
  authoritative.
- No GitHub repository, branch, or standalone simulator repository is deleted,
  archived, or rewritten as part of the import.

## Reevaluation Summary

The new `main` changes several assumptions from the prior plan.

| New `main` baseline | Effect on this integration plan |
| --- | --- |
| Raw MiR, LiftKit, UR8L, controller, and camera-module CAD now exists under root `assets/`. | Do not import duplicate raw CAD into `simulation/gazebo`. Use the monorepo assets as canonical source inputs and retain only simulator-derived meshes beside the ROS package. |
| The new raw CAD files are byte-identical to the corresponding standalone simulator sources, despite naming differences. | Create a checked manifest mapping source filename, SHA-256, units, transform, and generated simulator asset. Fail generation when a source digest changes. |
| V2 inspection hierarchy now models three stations and three elevation segments per station. | The Gazebo plan must validate against the existing V2 lifecycle rather than inventing a parallel demo workflow. |
| MiR placement evidence now comes from real MiR service command/snapshot calls in `SimulationLifecycleDriver`. | A Gazebo MiR adapter can feed measured station placement through the existing path; no Orchestrator-specific Gazebo client should be added. |
| Lift service now owns V2 `LiftPositionProfile` commands and measured completion evidence. | Gazebo Lift must implement both the legacy V1 seam and the V2 profile seam. The old plan's three hardcoded legacy preset mappings are insufficient. |
| Approved V2 Lift targets are `880`, `1240`, and `1600 mm`, with a provisional `0–2000 mm` range. | The current Gazebo model exposes only `0–500 mm` total joint travel. This is a blocking coordinate/range mismatch and must not be scaled, clamped, or offset without an explicit profile decision. |
| UR service now distinguishes wrist TCP, Inspection Payload TCP, and Gemini depth-camera transforms. | The Gazebo URDF/TF tree must use those distinct semantics. The Gemini offset is not the Inspection Payload TCP offset. |
| UR service owns provisional transform profile `inspection-policy-v1-simulation-transform-v2`. | Gazebo must carry the same profile identity/revision through a generated simulator manifest and parity tests rather than creating an independent transform. |
| Operational Supervision now has a deterministic Safe PLC status projection and a Lift decision seam. | Keep that projection deterministic in Gazebo mode. Gazebo collision/contact state is not PLC or permit evidence. |
| The monorepo now includes `compose.hardware.yaml`. | `compose.gazebo.yaml` must be a separate, simulation-only overlay and must reject or document combined use with the hardware overlay. |
| Robot Orchestrator now has a public simulation lifecycle and durable V2 report path. | End-to-end acceptance should use `/v1/simulations/inspection-lifecycles` and the V2 report, while accurately identifying any evidence still supplied by deterministic providers. |

## Terminology

Use the monorepo glossary without redefining its workflow terms:

- **Deterministic backend** — the existing network-free simulation adapters.
- **Gazebo backend** — the optional adapters whose measured MiR, UR, and Lift
  state comes from the native simulator.
- **Gazebo gateway** — the private simulation transport between containerized
  device services and native ROS. It is not a public device service.
- **Inspection station** — all inspection work at one achieved MiR placement.
- **Elevation segment** — inspection work at one commanded/measured Lift
  elevation while MiR and Lift remain stationary.
- **Safe PLC status projection** — the existing read-only, non-safety-rated
  software projection. It remains independent of Gazebo.

All Gazebo geometry, transforms, limits, and outcomes remain simulation
evidence. They are not commissioned calibration, collision freedom,
operational permission, or hardware validation.

## Current Baselines

### Standalone Gazebo repository

The current simulator provides:

- ROS 2 Jazzy and Gazebo Harmonic;
- `burke_description`, `burke_gazebo`, and `aircraft_navigation` packages;
- a differential-drive MiR base;
- a three-stage LiftKit with two actuated prismatic joints;
- a six-joint UR8 Long;
- a Challenger aircraft;
- odometry, TF, joint state, clock, and lidar outputs;
- scalar arm and Lift position commands;
- `/cmd_vel` mobile-base control;
- an opt-in aircraft-relative perimeter follower; and
- optional Foxglove visualization.

Current ROS-facing interfaces:

| Interface | ROS type | Direction |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS to Gazebo |
| `/odom` | `nav_msgs/msg/Odometry` | Gazebo to ROS |
| `/tf` | `tf2_msgs/msg/TFMessage` | Gazebo to ROS |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo to ROS |
| `/joint_states` | `sensor_msgs/msg/JointState` | Gazebo to ROS |
| `/arm/joint_1/command` … `/arm/joint_6/command` | `std_msgs/msg/Float64` | ROS to Gazebo |
| `/lift/stage_2/command` | `std_msgs/msg/Float64` | ROS to Gazebo |
| `/lift/stage_3/command` | `std_msgs/msg/Float64` | ROS to Gazebo |
| `/lidar/points` | `sensor_msgs/msg/PointCloud2` | Gazebo to ROS |

The simulator does not yet provide the Basler inspection camera, Gemini
surface-depth stream, localization cameras, obstacle cameras, ring light,
paint-thickness sensor, Nav2, or collision-aware route planning.

### Current monorepo `main`

The new baseline provides:

- deterministic device adapters as the normal Compose runtime;
- V1 MiR relative-move and UR named-preset command lifecycles;
- V1 legacy Lift preset commands;
- V2 Lift position profiles and measured completion evidence;
- V2 inspection stations, elevation segments, placement evidence, segment
  evidence, and reports;
- a three-station × three-segment simulation lifecycle;
- MiR station placement through MiR service commands and snapshots;
- Lift segment movement through the Lift V2 service boundary;
- UR candidate validation with distinct wrist/payload pose evidence;
- a read-only UR payload-geometry viewer;
- deterministic surface scans, capture, inference, and Safe PLC projection;
- Robot Orchestrator persistence and replay; and
- separate default and hardware Compose files.

Important current limitations to preserve honestly:

- `SimulationSegmentExecutionEvidenceProvider` still synthesizes UR-stow and
  segment-level Operational Supervision observations.
- The V2 lifecycle validates UR waypoint eligibility but does not command the
  Gazebo arm through each surface waypoint.
- Approved V2 Lift profile coordinates do not currently match the Gazebo Lift
  model's measured joint-travel range.
- The MiR simulation floor identifiers and ROS `map`/`odom` frames are not yet
  one explicit, versioned projection.

## Scope

### Included

- Import simulator code into an isolated ROS workspace under
  `simulation/gazebo/`.
- Deduplicate raw CAD against the new root `assets/` directory.
- Add a native integrated launch with rosbridge and optional Foxglove.
- Add a containerized gateway under `simulation/gazebo/gateway/`.
- Add explicit `deterministic|gazebo` backend selection to MiR, UR, and Lift
  services.
- Implement Gazebo adapters behind the existing service protocols.
- Feed measured Gazebo odometry, joint state, and TF-derived state into
  existing service responses.
- Execute existing MiR relative moves and station placements in Gazebo.
- Execute existing UR named presets in Gazebo and expose the corrected
  wrist/payload/depth-camera frame geometry.
- Execute legacy Lift presets and, after profile reconciliation, V2 Lift
  position profiles in Gazebo.
- Preserve deterministic adapters and scenarios as the default.
- Run the existing V2 lifecycle with measured Gazebo evidence where an
  existing service seam supports it.
- Add host-compatible gateway/adapter tests and marked native Gazebo tests.
- Update root/service documentation, project planning, and contract delivery
  state when implementation changes them.

### Excluded from the initial integration

- Hardware connectivity or control.
- Containerizing Gazebo.
- Replacing the deterministic default.
- Treating Gazebo contacts as collision-free planning or safety evidence.
- Gazebo-backed payload capture, surface scanning, defect detection, or Safe
  PLC status.
- Moving the UR through every generated inspection waypoint before an owned,
  command-bound execution contract exists.
- Nav2, SLAM, obstacle-camera integration, or finalized aircraft routing.
- Automatic archival of the standalone simulator repository.
- Silent compatibility shims for mismatched Lift coordinates or UR frames.

## Mandatory Compatibility Gates

These gates precede adapter implementation.

### Gate A — CAD and generated-asset ownership

Root `assets/` is the canonical source for the duplicated MiR, LiftKit, UR8L,
controller, and camera-module CAD. The simulator package owns only:

- generated link-local/metre-scale derivatives;
- reduced collision/display derivatives;
- aircraft assets not present at the root; and
- a machine-readable generation manifest.

The manifest records root source path, SHA-256, source units, applied rotation,
translation, scale, generated path, and generation tool revision. Build/test
must fail when source digests change without regenerated outputs and review.

Do not reference source files through brittle relative paths at Gazebo runtime.
Install generated assets into the ROS package share directory.

### Gate B — Lift position reference and reachable range

Before V2 Gazebo Lift commands are enabled, determine what
`LiftPositionProfile.target_position_mm` measures:

- actuator/stage travel;
- LiftKit top height;
- UR mounting-plane height;
- controller encoder position; or
- another calibrated reference.

Current evidence is incompatible:

- Gazebo joint travel totals `0–500 mm`;
- the Gazebo UR mount is approximately `876.230 mm` above `base_link` when
  collapsed and approximately `1376.230 mm` at full simulated travel; and
- current V2 profile targets are `880`, `1240`, and `1600 mm` in a provisional
  `0–2000 mm` range.

The gate must choose and document one of these outcomes:

1. revise the Gazebo model and its verified geometry/range so the existing V2
   profiles are physically representable; or
2. add explicit Gazebo-only Lift profiles and a matching Planner
   station/segment profile whose targets are representable by the current
   model.

Never scale `880/1240/1600` into `0–500`, clamp targets, or hide an offset in
the adapter. Profile ID, revision, calibration identity, commanded coordinate,
measured coordinate, units, and transform must remain traceable.

Legacy V1 `stowed/inspection_low/inspection_high` commands may be integrated
before this gate because their `0/250/500 mm` simulation mapping already fits
the current model. They do not satisfy V2 lifecycle acceptance.

### Gate C — UR frame and transform parity

The monorepo UR service now owns distinct provisional transforms:

- wrist/UR TCP to Inspection Payload TCP:
  translation `(0.0, -0.261, 0.063) m`, rotation vector
  `(1.571, 0.0, 0.0) rad`;
- wrist/UR TCP to Gemini depth camera:
  translation `(-0.023750, 0.085453, -0.029006) m`, with its separate Z/X
  rotation sequence and current simulation correction; and
- transform revision `inspection-policy-v1-simulation-transform-v2`.

The Gemini translation must not be reused as the payload TCP transform.

Gazebo needs explicit frames for:

```text
ur8l_base
└── ur_tcp / wrist
    ├── inspection_payload_tcp
    └── gemini_depth_camera
```

`URInspectionProfile` remains the application owner. A generated simulator
manifest carries its profile ID, revision, numeric transforms, frame names,
and provenance into Xacro/TF. CI compares that manifest with the service-owned
profile. An unreviewed mismatch makes Gazebo UR readiness unavailable.

### Gate D — simulation floor projection

Define one versioned mapping among:

- Planner floor profile `floor-demo-v1` revision `1`;
- existing MiR evidence identifiers such as `simulation-floor` and
  `simulation-v1`;
- ROS `map`; and
- ROS wheel-odometry frame `odom`.

The initial mapping may be identity at reset, but it must be named and reported
as simulation-only. Do not merely relabel `odom` as achieved Planner-frame
evidence.

### Gate E — shared-world reset semantics

The current application exposes a MiR-only simulation reset, while Gazebo owns
one shared world containing MiR, Lift, and UR.

Before wiring reset:

- decide whether Gazebo mode provides subsystem reset, coordinated world reset,
  or restart-only reset;
- require all device commands to be idle or terminal;
- preserve Orchestrator replay/idempotency semantics;
- increment a world/reset epoch; and
- reconcile service command state after reset or restart.

Do not make the existing MiR reset silently reset UR and Lift without changing
and documenting its application semantics.

## Isolation Rules

1. ROS/Gazebo source, generated assets, launch files, gateway code, and native
   scripts live under `simulation/gazebo/`.
2. Root `assets/` owns raw CAD that already exists there; the simulator does
   not duplicate it.
3. Normal device-service images do not install ROS, Gazebo, or `rclpy`.
4. Device services access Gazebo only through service-local adapter clients to
   the gateway.
5. The gateway translates low-level execution and state. It does not own
   actors, public idempotency, inspection workflow, permits, or durable reports.
6. Default Compose remains runnable without ROS, Gazebo, rosbridge, or the
   gateway.
7. Gazebo selection fails closed. Loss or stale state never falls back to the
   deterministic adapter.
8. Routes above device-service application layers do not branch on Gazebo.
9. Deterministic Safe PLC and unimplemented sensor providers remain clearly
   labelled deterministic; they are not derived from Gazebo.
10. After accepted import, the monorepo copy is the implementation source of
    truth. Any later standalone-repository disposition requires a separate
    decision.

## Target Architecture

```text
Default mode
============

Browser -> Control API -> Robot Orchestrator -> MiR / UR / Lift services
                                                |      |      |
                                                v      v      v
                                      deterministic service adapters


Gazebo-backed mode
==================

Browser -> Control API -> Robot Orchestrator -> MiR / UR / Lift services
                                                |      |      |
                                                v      v      v
                                       service-local Gazebo adapters
                                                  |
                                                  | private HTTP
                                                  v
                            +----------------------------------------+
Docker Compose              | Gazebo gateway                         |
                            | state, execution, timeout, normal stop |
                            +-------------------+--------------------+
                                                |
                                                | rosbridge WebSocket
============================= native boundary ==|============================
                                                v
Native Ubuntu 24.04                    rosbridge_server
                                                |
                                       ROS 2 topics / TF
                                                |
                                         ros_gz_bridge
                                                |
                                      Gazebo Harmonic world
```

The gateway belongs to the optional Compose overlay. Gazebo and ROS remain
native.

## Target Monorepo Layout

```text
bombardier_burk-e_monorepo/
├── assets/                              # canonical raw CAD already on main
├── compose.yaml                         # deterministic default
├── compose.hardware.yaml                # existing hardware commissioning overlay
├── compose.gazebo.yaml                  # new, mutually exclusive simulation overlay
├── simulation/
│   └── gazebo/
│       ├── README.md
│       ├── .gitignore
│       ├── config/
│       │   ├── asset_manifest.yaml
│       │   ├── frames.yaml
│       │   ├── gateway.yaml
│       │   ├── lift_profiles.yaml
│       │   └── ur_geometry_profile.yaml
│       ├── ros_ws/
│       │   └── src/
│       │       ├── aircraft_navigation/
│       │       ├── burke_description/
│       │       └── burke_gazebo/
│       ├── gateway/
│       │   ├── Dockerfile
│       │   ├── pyproject.toml
│       │   ├── src/burke_gazebo_gateway/
│       │   └── tests/
│       └── scripts/
│           ├── check_native_prerequisites.sh
│           ├── generate_assets.sh
│           ├── build_native.sh
│           ├── run_native.sh
│           └── stop_native.sh
└── backend/services/
    ├── mir_service/.../adapters/gazebo.py
    ├── ur_service/.../adapters/gazebo.py
    └── lift_service/.../adapters/gazebo.py
```

Do not commit ROS `build/`, `install/`, or `log/`, runtime databases, captured
output, generated reports, or duplicate raw CAD.

## Runtime Selection

Keep the existing outer mode and add a selector only to the three affected
device services:

```text
BURKE_HARDWARE_MODE=simulation
BURKE_SIMULATION_BACKEND=deterministic   # default
```

Gazebo override:

```text
BURKE_HARDWARE_MODE=simulation
BURKE_SIMULATION_BACKEND=gazebo
GAZEBO_GATEWAY_BASE_URL=http://gazebo_gateway:8091
```

Rules:

- Allowed backends are `deterministic` and `gazebo`.
- Omission selects `deterministic` and preserves current behavior.
- `gazebo` is invalid outside `BURKE_HARDWARE_MODE=simulation`.
- Gateway URL, timeout, and freshness settings are required only for Gazebo.
- Values are validated before adapter construction.
- Adapter construction makes no network call; connection occurs during service
  lifespan startup.
- Public readiness remains `simulated`; detail/evidence identifies the backend
  and active simulation profile/revision.
- Only MiR, UR, and Lift receive the Gazebo selector in the override.
- Combining `compose.gazebo.yaml` with `compose.hardware.yaml` is rejected or
  documented as unsupported. It must not produce a mixed implicit runtime.

## Native ROS Contract

Add `integrated_stack.launch.py` under `burke_gazebo`. It starts:

1. the existing world and robot;
2. `robot_state_publisher` with simulation time;
3. existing ROS/Gazebo topic bridges;
4. rosbridge WebSocket on a configured host port;
5. optional Foxglove on its separate port;
6. the reviewed simulation floor-frame projection; and
7. no autonomous movement.

Required launch arguments:

| Argument | Purpose |
| --- | --- |
| `gui` | enable/disable Gazebo client |
| `rosbridge_address`, `rosbridge_port` | container-to-native transport |
| `foxglove`, `foxglove_port` | independent visualization |
| `spawn_x`, `spawn_y`, `spawn_z`, `spawn_yaw` | deterministic initial base pose |
| `aircraft_frame` | preserve explicit aircraft transform |
| `simulation_profile` | bind floor, UR geometry, Lift, and reset profile revisions |

Verify the installed Jazzy/Harmonic reset and entity-state service types before
adding service bridges. If reliable reset is unavailable, use an explicit
restart-based workflow. Do not guess a `ros_gz` service mapping.

Expected frame tree after compatibility gates:

```text
map                                  simulation floor projection
└── odom                             wheel odometry, identity at reset only
    ├── base_footprint
    │   └── base_link
    │       └── Lift and UR hierarchy
    │           └── ur_tcp
    │               ├── inspection_payload_tcp
    │               └── gemini_depth_camera
    └── aircraft
```

The application floor profile may map to `map`; it is not necessarily a TF
frame. That mapping must carry the simulation profile and revision.

## Gazebo Gateway Boundary

The gateway owns:

- rosbridge connection lifecycle;
- subscriptions to `/clock`, `/odom`, `/tf`, `/tf_static`, and
  `/joint_states`;
- publishers for `/cmd_vel`, six UR joint targets, and two Lift stage targets;
- bounded state caches and sequence/reset epochs;
- one active execution per subsystem;
- MiR relative-motion control;
- measured completion, dwell, no-progress, and wall-time timeout logic;
- normal-stop/hold behavior; and
- capability/readiness reporting.

It does not own:

- public actor or idempotency policy;
- Planner station/segment selection;
- device-service position or geometry profiles;
- Orchestrator persistence/replay;
- Operational Supervision or Safe PLC decisions;
- inspection capture/reporting; or
- hardware safety.

Private resources:

| Resource | Purpose |
| --- | --- |
| `GET /healthz` | process health |
| `GET /readyz` | rosbridge and required-state freshness |
| `GET /v1/capabilities` | supported operations, frames, scenarios, and profile revisions |
| `GET /v1/world` | world/reset epoch and simulator time |
| `PUT /v1/world` | coordinated idle-only reset when Gate E permits it |
| `GET /v1/mir/state` | measured pose/twist and active execution |
| `POST/GET/DELETE /v1/mir/executions...` | bounded relative motion/status/normal stop |
| `GET /v1/ur/state` | measured joints and TF-derived wrist/payload/camera poses |
| `POST/GET/DELETE /v1/ur/executions...` | joint target/status/hold |
| `GET /v1/lift/state` | measured stages plus resolved profile coordinate |
| `POST/GET/DELETE /v1/lift/executions...` | stage target/status/hold |

The gateway receives resolved numeric targets and profile identity from device
services. It does not accept Planner profile IDs as command policy.

### State and time rules

- Use `/clock` as simulator time.
- Use monotonic wall time for freshness and bounded liveness because Gazebo can
  pause.
- Use UTC gateway-reception timestamps for monorepo `observed_at`; retain
  simulator time separately in gateway-only evidence.
- Reject non-finite values, unknown/duplicate joints, missing TF, stale state,
  profile mismatch, and out-of-range targets.
- Never present cached state as fresh after its limit.
- Increment a reset epoch after every accepted coordinated reset/restart.
- Publish zero `/cmd_vel` on completion, cancellation, timeout, shutdown, and
  connection loss when publication is possible.
- Hold latest measured UR/Lift positions on normal cancellation.
- Completion requires measured in-tolerance state for the profile-owned dwell
  period, not successful publication.
- Disconnection, restart, unowned motion, timeout, and ambiguous stop become
  explicit failed or unknown outcomes.

## Device Mapping

### MiR1350

Implement the existing MiR adapter protocol:

| Service operation | Gazebo behavior |
| --- | --- |
| readiness | fresh gateway, clock, odometry, and required TF |
| enqueue | validate service-owned MiR motion profile and queue command |
| start | create one gateway relative execution |
| status | map measured execution to existing mission/command states |
| abort | zero velocity and confirm stopped or unknown |
| snapshot | measured pose, target, trail, world ID, and lifecycle state |
| placement evidence | existing `MiRServiceClient.execute_simulation_placement` consumes the snapshot automatically |

The controller supports existing robot-relative `x_m`, `y_m`, and
`orientation_deg` requests. Differential-drive lateral displacement is a
bounded rotate/drive/rotate or path-following sequence, never strafing.

Initial Gazebo scenario support is `normal`. Existing deterministic
`blocked/failure/stale` scenarios remain available only on the deterministic
backend until deliberate Gazebo fault injection exists. Gazebo collision
physics and no-progress timeout are limited blocked evidence, not obstacle-aware
planning.

The aircraft perimeter follower remains an opt-in ROS demonstration and does
not implement the application relative-move contract.

### UR8L

Implement existing `URAdapter`, `URCommandAdapter`, and simulation reset
capabilities:

- service owns named presets and speed/acceleration policy;
- adapter resolves `home` and `inspection_start` into numeric targets;
- gateway publishes targets only after explicit command start;
- status uses all six measured joint names and dwell tolerance;
- state uses TF-derived wrist TCP;
- payload geometry uses the Gate C manifest and reports separate wrist,
  Inspection Payload TCP, and Gemini frames;
- queue/start/status/cancel, actor, and idempotency semantics remain unchanged;
- unowned motion or transform/profile mismatch makes readiness unavailable;
  and
- hardware RTDE mode remains unchanged and read-only.

The current Gazebo position controllers use fixed limits. Do not claim a
requested speed/acceleration was dynamically enforced unless controller support
is implemented and observed.

The current V2 lifecycle does not command each surface waypoint. Initial
acceptance proves named UR commands and corrected measured geometry through the
existing viewer/API. A future waypoint-execution slice requires an owned public
contract, permit binding, measured arrival, collision evidence, and
Orchestrator reconciliation.

### LiftKit

Implement both legacy and V2 protocols:

- V1 named presets remain supported for viewer/manual compatibility;
- V2 accepts only a Lift-service-resolved `LiftPositionProfile`;
- gateway receives stage targets plus profile/calibration identity;
- state observes both prismatic joints;
- adapter converts measured joints into the reviewed profile coordinate from
  Gate B;
- completion uses V2 tolerance, settling time, velocity, and freshness;
- public evidence retains commanded/measured millimetres, profile
  ID/revision, calibration ID, and absence reason; and
- reset/cancel/restart preserve unknown-outcome semantics.

Do not keep the prior plan's unconditional V2 mapping:

```text
stowed=0 mm, low=250 mm, high=500 mm
```

Those are legacy simulator presets only. They are not substitutes for the new
`fuselage-lower/mid/upper` V2 profiles.

### Existing V2 lifecycle

Gazebo integration should reuse the existing lifecycle:

```text
Planner V2 station/segment profile
  -> MiR service placement
  -> station evidence
  -> current UR-stow/operational evidence provider
  -> Lift V2 profile command
  -> deterministic surface scan + UR candidate validation
  -> deterministic capture/inference
  -> durable V2 station/segment report
```

After adapter integration:

- MiR placement evidence comes from Gazebo odometry through the MiR service;
- Lift evidence comes from Gazebo joint state after Gate B;
- UR named-command/viewer evidence can come from Gazebo;
- surface scan/capture/inference/Safe PLC projection remain deterministic; and
- current synthetic UR-stow/segment supervision evidence remains explicitly
  identified until replaced through an owned service-backed seam.

Do not call the lifecycle “fully Gazebo-driven” while any of those providers
remain deterministic.

## Compose Integration

Add `compose.gazebo.yaml` without changing default `compose.yaml` semantics.
The overlay:

- builds `gazebo_gateway` from `simulation/gazebo/gateway/`;
- connects the gateway to native rosbridge via `host.docker.internal`;
- adds Linux `host-gateway` explicitly;
- selects Gazebo only for MiR, UR, and Lift;
- leaves Planner, Surface Waypoint, Payload, Defect Detection, reporting, and
  Operational Supervision simulation providers deterministic;
- does not add hardware endpoints, credentials, or controller identities;
- starts the UI/API even when native Gazebo is temporarily unavailable so
  readiness can report the failure; and
- is not combined with `compose.hardware.yaml`.

Operator flow:

```bash
# Terminal 1 — native Ubuntu
./simulation/gazebo/scripts/check_native_prerequisites.sh
./simulation/gazebo/scripts/build_native.sh
./simulation/gazebo/scripts/run_native.sh --headless

# Terminal 2 — application stack
docker compose -f compose.yaml -f compose.gazebo.yaml up --build
```

Default mode remains:

```bash
docker compose up --build
```

## Ordered Implementation Tasks

Complete a task only after its acceptance criteria pass.

### Task 0 — Rebase the implementation branch on evaluated `main`

- [ ] Start from `main` at or after `58caed59` on a non-main branch/worktree.
- [ ] Re-read root `AGENTS.md`, `README.md`, `GLOSSARY.md`, and
      `PROJECT_PLANNING.md`.
- [ ] Recheck Lift, UR, MiR, Orchestrator, Planner, and Operational Supervision
      plans/tests for changes after this document's evaluated commit.
- [ ] Record current source and destination SHAs in the handoff.

Acceptance:

- No implementation is based on the stale `908e7bc` tree.
- Existing working-tree changes are preserved.
- Any newer contract/profile change is reflected before import begins.

### Task 1 — Import the ROS workspace and deduplicate assets

- [ ] Create `simulation/gazebo/ros_ws/src/`.
- [ ] Import the three ROS packages without changing their baseline behavior.
- [ ] Create the root-asset mapping/generation manifest.
- [ ] Remove duplicated raw MiR/Lift/UR/controller/payload CAD from the imported
      package.
- [ ] Generate link-local/metre-scale simulator assets from root `assets/`.
- [ ] Keep aircraft assets inside the simulator package.
- [ ] Add local ignore rules and update simulator README paths.
- [ ] Record standalone source commit and generated-asset digests.

Acceptance:

- Every duplicated raw CAD source resolves to the root asset with matching
  SHA-256.
- Generated assets reproduce current visuals/collisions.
- Native `colcon build --symlink-install` and existing ROS tests pass.
- Existing default Compose remains valid.

### Task 2 — Resolve frame, Lift, and reset compatibility gates

- [ ] Resolve Gate B's Lift coordinate reference and choose model revision or
      Gazebo-specific V2 profiles.
- [ ] Freeze the Lift simulation calibration/profile identity and propagation.
- [ ] Generate and validate Gate C's UR geometry manifest.
- [ ] Define Gate D's floor-frame projection and placement evidence IDs.
- [ ] Define Gate E's subsystem/shared reset behavior.
- [ ] Update affected service/Planner profiles and contract delivery state only
      after owner approval.
- [ ] Add parity tests that fail on profile/revision/transform mismatch.

Acceptance:

- No conversion depends on an unexplained scale, clamp, or offset.
- All three V2 Lift targets selected for Gazebo are reachable by the chosen
  model/profile.
- Wrist, payload, Gemini, floor, map, and odom semantics are explicit.
- Reset behavior does not silently change the MiR-only public resource.

This task is a hard prerequisite for V2 Lift/lifecycle acceptance.

### Task 3 — Add native integrated launch and scripts

- [ ] Add `integrated_stack.launch.py` with rosbridge and optional Foxglove.
- [ ] Apply the reviewed floor and UR frame manifests.
- [ ] Keep perimeter/autonomous motion off by default.
- [ ] Verify port conflicts and native host binding.
- [ ] Add prerequisite, asset generation, build, run, and stop scripts with
      bounded waits.
- [ ] Implement only the reset mechanism approved in Task 2.

Acceptance:

- One native command starts a headless graph with fresh required topics/TF.
- Startup causes no motion.
- rosbridge is reachable from Docker's host gateway.
- Shutdown sends best-effort zero Twist and ends boundedly.

### Task 4 — Implement the isolated gateway

- [ ] Add gateway package/container and injected rosbridge transport.
- [ ] Implement health, readiness, capability, state, execution, cancel, and
      approved reset resources.
- [ ] Add bounded state caches and strict schema/freshness validation.
- [ ] Add single active execution per subsystem.
- [ ] Add measured completion/dwell and zero/hold behavior.
- [ ] Add unit tests with a fake rosbridge; they must not require ROS.

Acceptance:

- Process health and ROS readiness are separate.
- No endpoint moves a device before explicit start.
- Missing/stale/profile-mismatched data never appears successful.
- Connection loss cannot leave an unbounded velocity command without unknown
  outcome evidence.

### Task 5 — Add backend selection to MiR, UR, and Lift services

- [ ] Add validated `SimulationBackend` to each service.
- [ ] Preserve `deterministic` as default.
- [ ] Require Gazebo URL/timeouts/freshness only for Gazebo.
- [ ] Reject Gazebo outside simulation mode.
- [ ] Construct adapters without network I/O.
- [ ] Propagate backend/profile identity into readiness detail/evidence.
- [ ] Test defaults, valid overrides, malformed/non-finite values,
      contradictory modes, and final adapter construction.

Acceptance:

- Existing deterministic tests pass unchanged.
- Default startup opens no gateway/ROS connection.
- Gazebo configuration fails clearly when incomplete.
- Gateway loss never triggers deterministic fallback.

### Task 6 — Implement Gazebo-backed MiR

- [ ] Add gateway client/adapter behind the existing MiR protocol.
- [ ] Map fresh odometry into snapshot/readiness/viewer state.
- [ ] Implement bounded differential-drive relative motion.
- [ ] Implement completion, no-progress, timeout, cancellation, and restart
      reconciliation.
- [ ] Preserve command registry, actor, idempotency, and Orchestrator permits.
- [ ] Map the approved floor projection into V2 placement evidence.
- [ ] Keep non-normal scenarios deterministic-only initially.

Acceptance:

- Existing viewer/Orchestrator commands visibly move Gazebo MiR.
- Viewer pose/trail and station placement evidence come from Gazebo odometry.
- Completion requires measured pose and stop tolerance.
- Cancellation/failure publishes zero velocity.
- V2 station placement uses the existing MiR service client path.

### Task 7 — Implement Gazebo-backed UR8L

- [ ] Add gateway client/adapter behind existing UR protocols.
- [ ] Resolve named presets from service-owned configuration.
- [ ] Use six measured joints for execution status.
- [ ] Derive wrist pose from TF and payload/Gemini poses from the reviewed
      transform manifest.
- [ ] Implement queue/start/status/cancel/reset reconciliation.
- [ ] Detect unowned motion and profile/TF mismatch.
- [ ] Preserve the read-only hardware RTDE path.

Acceptance:

- Existing named UR commands visibly move Gazebo UR8L.
- Public state equals measured joints and wrist TF.
- Payload geometry reports distinct wrist/payload/Gemini frames with current
  profile/revision.
- Completion waits for measured tolerance/dwell.
- Cancellation holds measured joints and reports cancelled or unknown.

### Task 8 — Implement Gazebo-backed LiftKit

- [ ] Add gateway client/adapter behind V1 and V2 Lift protocols.
- [ ] Implement reviewed coordinate conversion from Task 2.
- [ ] Observe both physical stage joints.
- [ ] Implement legacy preset and approved V2 profile commands.
- [ ] Apply V2 tolerance, settling time, velocity, bounds, and freshness.
- [ ] Preserve profile/calibration identity and absence reasons.
- [ ] Detect inconsistent stages, out-of-range state, stale data, and unowned
      motion.

Acceptance:

- Legacy presets visibly move Gazebo LiftKit.
- Approved V2 profile commands reach their actual commanded coordinate.
- Measured V2 evidence comes from Gazebo joint state.
- The three-segment profile is either truthfully supported or explicitly
  unavailable; no scaled synthetic success is allowed.
- Hardware mode remains unchanged and unavailable.

### Task 9 — Add the optional Compose topology

- [ ] Add `compose.gazebo.yaml` and gateway container.
- [ ] Add Linux host-gateway access.
- [ ] Set Gazebo backend only on MiR, UR, and Lift.
- [ ] Keep all other simulation providers deterministic.
- [ ] Add a bounded preflight for rosbridge, topics, frames, and profile
      revisions.
- [ ] Verify UI/API remain available during native simulator outage.
- [ ] Reject/document simultaneous Gazebo and hardware overlays.

Acceptance:

- Default and Gazebo Compose configurations validate.
- Plain Compose requires no ROS.
- Gazebo overlay selects exactly three device adapters.
- Native outage reports stale/disconnected without fallback.
- Restart requires explicit reconciliation before motion.

### Task 10 — Integrate the existing V2 lifecycle

- [ ] Run the existing three-station × three-segment lifecycle through the
      Gazebo-backed services.
- [ ] Verify MiR placement evidence is measured from Gazebo.
- [ ] Verify Lift V2 evidence is measured from Gazebo after Task 2.
- [ ] Keep deterministic surface/capture/inference/PLC evidence labelled as
      such.
- [ ] Do not claim waypoint UR execution; separately verify Gazebo UR named
      commands and viewer geometry.
- [ ] Preserve Orchestrator actor/idempotency persistence and completed-run
      replay without re-actuation.
- [ ] Test outage, unknown command, partial segment, restart, and wrong-owner
      report behavior.
- [ ] Consider replacing synthetic UR-stow/segment supervision providers only
      through their owning service-backed contracts; do not embed ROS in the
      Orchestrator.

Acceptance:

- `/v1/simulations/inspection-lifecycles` completes or explicitly blocks based
  on real Gazebo device evidence.
- V2 reports retain correct station, segment, profile, commanded/measured Lift,
  and MiR placement evidence.
- Replay returns persisted results and does not move Gazebo again.
- Deterministic evidence remains distinguishable from Gazebo evidence.

### Task 11 — Documentation, validation, and handoff

- [ ] Update root README with both modes and exact prerequisites.
- [ ] Update `PROJECT_PLANNING.md` contract/delivery state.
- [ ] Update service READMEs for every new environment setting and backend.
- [ ] Update `GLOSSARY.md` only if a new cross-service term is necessary.
- [ ] Add host-compatible gateway/adapter tests to normal CI.
- [ ] Add a separate native Ubuntu Gazebo test job when a suitable runner is
      available.
- [ ] Record versions, commands, timeouts, source SHAs, asset digests, profile
      revisions, and unresolved assumptions.

Acceptance:

- A developer can run deterministic mode without reading Gazebo setup.
- An Ubuntu developer can reach the Gazebo-backed V2 smoke from a clean
  checkout.
- CI distinguishes host-compatible, Compose, and native Gazebo tests.
- Documentation never presents simulation evidence as commissioning.

## Validation Matrix

| Layer | Deterministic default | Gazebo opt-in |
| --- | --- | --- |
| Existing backend tests | required | required |
| New adapter tests | fake deterministic transport | fake gateway |
| Gateway tests | not started | required without ROS |
| Default Compose config | required | required unchanged |
| Gazebo Compose config | not used | required |
| Hardware overlay config | required unchanged | must not be combined |
| Native `colcon build/test` | not required | required |
| Headless Gazebo launch | not required | required |
| MiR evidence | deterministic world | odometry + TF |
| UR evidence | deterministic interpolation | joint state + TF + profile parity |
| Lift V1 evidence | deterministic presets | joint state |
| Lift V2 evidence | deterministic profile state | reviewed coordinate + joint state |
| V2 lifecycle | deterministic providers | mixed, with each source labelled |
| Hardware access | forbidden | forbidden |

Minimum monorepo checks after implementation:

```bash
cd backend
uv sync --all-packages --group dev
uv run pytest
uv run ruff check .
uv run ruff format --check .
uv run mypy packages/burke_contracts/src packages/burke_observability/src \
  services/mir_service/src services/ur_service/src services/lift_service/src \
  services/inspection_planner/src services/operational_supervision/src \
  services/robot_orchestrator/src services/control_api/src

cd ../frontend
npm ci
npm run typecheck
npm run test
npm run build

cd ..
docker compose config
docker compose -f compose.yaml -f compose.gazebo.yaml config
docker compose -f compose.yaml -f compose.hardware.yaml config
```

Native checks:

```bash
source /opt/ros/jazzy/setup.bash
cd simulation/gazebo/ros_ws
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
ros2 launch burke_gazebo integrated_stack.launch.py gui:=false
```

Run the existing opt-in lifecycle merge gate against the Gazebo overlay after
the stack reports ready:

```bash
cd backend
BURKE_HOST_LIFECYCLE_SMOKE=1 \
  uv run pytest services/robot_orchestrator/tests/test_host_lifecycle_smoke.py
```

All native tests need bounded startup, execution, and cleanup and must leave
`/cmd_vel` at zero.

## Stop-and-Ask Conditions

Stop and ask the project owner if:

- the implementation baseline is newer than `58caed59` and changes affected
  contracts/profiles;
- ROS or Gazebo would enter normal application-service images;
- default Compose behavior would change;
- raw CAD would be duplicated instead of using root `assets/`;
- the Lift position reference remains undefined;
- existing Lift V2 targets would need scaling, clamping, or a hidden offset;
- supporting `1600 mm` requires unverified Lift geometry or joint changes;
- UR wrist, payload, or Gemini transforms disagree with the service profile;
- a transform would be relabelled without explicit frame/profile evidence;
- MiR Planner-frame placement cannot be derived from a named floor projection;
- shared reset would silently change MiR-only reset semantics;
- a public contract must break rather than gain an explicit compatible field or
  version;
- Gazebo controller limits cannot truthfully satisfy a requested motion
  profile;
- collision-free planning is required for the first milestone;
- the V2 lifecycle must physically move the UR through every waypoint;
- deterministic Safe PLC or sensor evidence is being presented as Gazebo or
  safety evidence;
- command ownership, permits, idempotency, replay, or fail-closed behavior
  would be weakened;
- hardware credentials, private endpoints, or hardware control appear; or
- standalone-repository deletion/archival is requested implicitly.

## Completion Definition

The integration is complete when:

1. the monorepo contains the isolated native workspace under
   `simulation/gazebo/` without duplicate raw CAD;
2. default Compose remains fully deterministic and ROS-free;
3. the Gazebo overlay selects MiR, UR, and Lift only;
4. existing MiR and UR commands visibly move their Gazebo models and report
   measured state through existing APIs;
5. Lift V1 and the owner-approved V2 profiles move Gazebo LiftKit and return
   truthful measured completion evidence;
6. UR wrist/payload/Gemini and MiR floor-frame semantics match current
   service-owned profiles;
7. missing, stale, mismatched, or restarted Gazebo state fails closed without
   deterministic fallback;
8. the existing V2 lifecycle and report path consume measured Gazebo MiR/Lift
   evidence while every remaining deterministic source stays labelled;
9. completed lifecycle replay does not re-actuate Gazebo; and
10. host, Compose, native, documentation, and handoff validation are current.
