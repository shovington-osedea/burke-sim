# AGENTS

## Repository Goal

Build a Gazebo simulation of Burk-e, a mobile robot for aircraft exterior
inspection. The simulation should recreate the major physical parts of the real
robot, place them in the correct kinematic hierarchy, and perform a simplified
inspection around a 3D aircraft model.

This repository is simulation-only. Do not add real-device credentials,
private network addresses, vendor controller connections, or hardware-control
code. When a physical dimension, joint limit, transform, or sensor property is
unknown, document the assumption instead of presenting it as a measured fact.

## Physical Robot

Build the robot from the ground up in this order:

1. **MiR1350 mobile base** — drives the complete platform around the aircraft.
2. **MiR top deck** — supports the lift and the onboard-computer enclosure.
3. **Ewellix LiftKit** — provides vertical positioning for the arm.
4. **UR8L arm** — moves the inspection payload over the aircraft surface. Its
   documented maximum reach is approximately `1.73 m`.
5. **TCP-mounted inspection payload** — contains the inspection camera,
   surface depth camera, ring light, and paint-thickness sensor.

### Sensors and Payload

- **Inspection camera:** Basler a2A4504-18ucPRO with a C12-1224-25M lens.
  Simulate image capture from the arm's tool centre point.
- **Surface depth camera:** Orbbec Gemini 335L mounted near the UR flange/TCP.
  It observes the aircraft surface and generates candidate inspection
  waypoints.
- **Ring light:** mounted with the inspection camera and enabled during image
  capture.
- **Paint-thickness sensor:** mounted on the inspection payload and used only
  at selected contact inspection poses.
- **MiR localization cameras:** fixed front-facing RealSense cameras used for
  coarse anchor localization.
- **Obstacle cameras:** one forward-facing and one rear-facing depth camera on
  the cobot base plate. They observe clearance around the MiR, lift, and arm
  envelope before base or lift movement.

The obstacle cameras, MiR localization cameras, and TCP-mounted Gemini are
separate sensors with different poses and purposes.

### Useful Simulation Seeds

These values came from planning and proof-of-concept documentation. Treat them
as initial simulation values, not commissioned measurements:

- Gemini flange-to-camera translation:
  `[-0.023750, 0.085453, -0.029006] m`.
- Gemini flange-to-camera rotation: Euler sequence `zx`, angles `[180, 147]°`.
- Inspection footprint: `0.30 m × 0.30 m`.
- Planned overlap between neighbouring inspection views: `15%`.
- Nominal surface-waypoint spacing: `0.255 m`.
- Initial localization marker: AprilTag `DICT_APRILTAG_36h11`, size `0.140 m`.

Prefer verified vendor geometry, meshes, joint limits, and inertial properties
when they become available. Keep the Gazebo link and frame structure stable so
better geometry can replace simplified models later.

## Planned Inspection Flow

1. **Start and preflight**
   - Spawn Burk-e near its dock and place the aircraft model in the world.
   - Confirm that the MiR base, lift, UR arm, cameras, light, and simulated
     inspection sensors are available.

2. **Undock and localize**
   - Move the MiR away from its dock.
   - Use the front localization cameras and an AprilTag anchor to estimate the
     robot's position relative to the inspection area.

3. **Follow the aircraft route**
   - Use a coarse predefined route around the selected aircraft model.
   - Treat this route as a sequence of candidate inspection stations, not as a
     final collision-free trajectory.
   - Before each MiR movement, check the simulated obstacle observations and
     plan a collision-free base motion.

4. **Establish an inspection station**
   - Stop the MiR at a stable pose beside the aircraft.
   - Keep the MiR stationary while the lift, arm, and payload perform inspection
     work at this station.

5. **Select an elevation segment**
   - Stow the UR arm before moving the LiftKit.
   - Move the lift to a named inspection height.
   - Keep both the MiR and lift stationary while the segment is inspected.

6. **Scan the aircraft surface**
   - Move the UR arm through safe scan poses.
   - Use the Gemini depth camera to observe the local aircraft surface.
   - Generate a grid of candidate inspection poses using the configured
     footprint and overlap.
   - Reject candidates that are unreachable or collide with the robot,
     payload, aircraft, or environment.

7. **Inspect each reachable waypoint**
   - Move the inspection payload to a reachable pose facing the surface.
   - Turn on the ring light.
   - Capture a simulated inspection image.
   - When required, move to the approved contact pose and simulate a
     paint-thickness reading.
   - Turn off the ring light and record the waypoint outcome.

8. **Continue coverage**
   - Repeat waypoint inspection until the reachable surface area for the
     current elevation is covered.
   - Preserve skipped, unreachable, collided, or failed waypoints for the final
     coverage result. A simulated defect should be recorded but should not stop
     inspection of the remaining waypoints.

9. **Move to the next segment or station**
   - Stow the UR arm before moving the lift.
   - After completing all elevations at a station, stow the arm and lift before
     moving the MiR.
   - Continue around the aircraft until all planned stations are complete.

10. **Finish the inspection**
    - Return the robot to a safe final or docked pose.
    - Produce a simulation result containing visited stations, elevation
      segments, captured views, rejected waypoints, coverage, and simulated
      findings.

## Motion and Modeling Rules

- An **inspection station** is all inspection work performed at one fixed MiR
  pose.
- An **elevation segment** is the work performed at one fixed LiftKit height
  within an inspection station.
- A **capture view** is one camera observation at one surface waypoint.
- Do not move the MiR or LiftKit while the UR arm is executing an inspection
  segment.
- Stow the UR arm before any LiftKit or MiR transition.
- Model collision geometry for the mobile base, lift, arm, payload, aircraft,
  and nearby environment even when visual meshes are simplified.
- Keep simulated sensor frames, robot frames, and aircraft/world frames
  explicit and consistent.
