# Robotar development log

Last updated: 2026-10-20

## Purpose

This document records Robotar experiments, design decisions, verified results and open problems. It is a working engineering log, not a marketing claim. New trials should be added with the goal, change, verification evidence, result and next action.

## Current product direction

The product direction is now a **branch clamp-shaker robot** for supervised olive harvesting:

```text
stabilized mobile rover
+ RGB-D branch perception
+ MoveIt/admittance arm control
+ soft branch clamp
+ adaptive-frequency shaker
+ force and tilt safety interlocks
```

The intended farmer workflow is:

1. Farmer places a collection net.
2. Robot drives to the tree.
3. Robot scans and selects a reachable branch.
4. Arm clamps the branch with controlled force.
5. Shaker vibrates the branch.
6. Olives fall onto the net.
7. Robot releases the branch and continues.

The current rotating comb is only a software placeholder. It is not the intended product mechanism.

## Trial and decision log

### 2026-10-15 — Environment and existing simulations

**Goal:** establish a supported simulation environment and evaluate existing agricultural robot simulations.

**Work performed:**

- Installed Ubuntu 24.04-compatible ROS 2 Jazzy and Gazebo Harmonic.
- Evaluated a Panther + Franka tomato-harvesting simulation.
- Evaluated an Amiga + Kinova orchard simulation.
- Reviewed ROS 2 topics, actions, services, parameters, TF, launch files, workspaces and Gazebo integration.

**Result:**

- ROS 2 Jazzy and Gazebo Harmonic passed communication and simulation smoke tests.
- The tomato demo demonstrated navigation, detection and MoveIt harvesting logic.
- The Amiga demo demonstrated orchard, agricultural base, sensors, arm and Docker isolation.

**Decision:** use ROS 2 Jazzy + Gazebo Harmonic for the first-party Robotar prototype. Treat third-party projects as references, not final designs.

### 2026-10-16 — GPU acceleration

**Goal:** enable the GTX 1650 for simulation and inference.

**Work performed:**

- Diagnosed an NVIDIA kernel-module mismatch after a kernel update.
- Installed the recommended NVIDIA 595 open driver and DKMS headers.
- Installed CUDA-enabled PyTorch.
- Tested CUDA tensor operations and YOLO inference in the reference tomato project.
- Verified Gazebo graphics and YOLO compute processes through `nvidia-smi`.

**Result:** GPU acceleration works. The GTX 1650 is sufficient for current simulation and initial perception experiments, but it is not a guarantee of real-time performance for larger future models.

### 2026-10-17 — Initial Robotar vertical slice

**Goal:** create an original ROS 2/Gazebo olive-platform prototype rather than another patched demo.

**Work performed:**

- Created `robotar_description`, `robotar_gazebo` and `robotar_mission` packages.
- Built a custom four-wheel rover, 6-DOF arm, mast RGB-D camera, IMU, comb and gripper interfaces.
- Generated an olive orchard with target branch, dynamic olives and catch net.
- Implemented odometry-based row driving, arm trajectories, comb control and contact-gated olive detachment.

**Verified result:**

- Rover drove approximately 2.952/3.000 m.
- Comb/branch contact was detected.
- Six targeted olives detached.
- All six olives settled inside the catch region.
- Arm retracted successfully.
- No-contact mode blocked detachment and failed safely.

**Important boundary:** this proved software orchestration, not physical olive-harvest effectiveness.

### 2026-10-18 — Startup race and controller readiness

**Problem:** the mission sometimes sent the first arm trajectory before `arm_controller` became active.

**Fix:** `harvest_demo.py` now queries `/controller_manager/list_controllers` and waits until drive, arm, comb and joint-state controllers are actually active.

**Result:** repeated visible and headless missions reached `MISSION DONE` after the fix.

### 2026-10-18 — Gazebo reset limitation

**Problem:** Gazebo world/model reset removes the runtime-spawned robot without rerunning the ROS spawn/controller bringup.

**Investigation:** a respawn supervisor was tested but could create duplicate respawns and controller races under load.

**Decision:** do not use Gazebo reset in this MVP. Restart the launch script. A safe lifecycle reset supervisor remains future work.

### 2026-10-18 — Geometry and connectedness corrections

**Problems found:**

- Wheel collision cylinders overlapped the chassis.
- Wheels had no visible axles.
- Mast pole and camera had visual gaps.
- IMU had no mounting bracket.
- Arm had no support pedestal between chassis and arm base.
- Arm joints lacked visible bearing caps.

**Fixes:**

- Increased wheel track and adjusted the catch net.
- Added wheel axles, mast neck, IMU bracket, arm pedestal and joint caps.
- Added a flange adapter for tool connection.
- Added an RViz RGB camera display.

**Verification:** generated-URDF adjacency checks report that visible parent/child bounds touch or overlap for both comb and gripper variants.

### 2026-10-19 — Mission-mode confusion and successful visible run

**Problem:** `robotar_run_sim.sh` was used and the rover did not drive.

**Explanation:** `robotar_run_sim.sh` starts only the robot, orchard and controllers. It does not start `odom_driver` or `harvest_demo`.

**Correct command:**

```bash
ROBOTAR_GUI=1 ROBOTAR_GPU_RENDER=1 ROBOTAR_TOOL=comb \
  ./scripts/robotar_run_harvest.sh
```

**Result:** the visible mission drove to the tree, detected contact, detached all six olives and completed.

### 2026-10-19 — Realism review

**Conclusion:** the robot is a functional ROS 2/Gazebo prototype, not a field-realistic harvesting machine.

Current limitations include:

- Rigid branch.
- Contact-triggered detachment rather than stem-force physics.
- No fruit, leaf or bark damage model.
- No slope, traction, rollover or stabilizer model.
- No camera-based branch targeting in the Robotar mission.
- No actuator, thermal, dust, water or structural validation.
- Robot self-collision remains disabled.
- Foliage and comb contact geometry remain coarse.

### 2026-10-20 — Product-design correction

**User feedback:** the rotating comb is unclear and not farmer-ready.

**Decision:** accepted. The comb remains only a placeholder/demo tool. The product mechanism will be a **branch clamp-shaker**:

```text
soft padded clamp
+ enclosed vibration unit
+ force/torque sensing
+ clamp IMU/accelerometer
+ controlled frequency and amplitude
```

**Reason:** this has a clear farmer workflow and a defensible relationship to existing hand-held and tractor-mounted shaking tools.

## Current verified capabilities

- ROS 2 Jazzy and Gazebo Harmonic run on Ubuntu 24.04.
- NVIDIA GTX 1650 works for Gazebo graphics and CUDA experiments.
- Custom Robotar packages build successfully.
- Base, arm, comb and gripper controllers run.
- The placeholder mission completes drive, contact, detachment, collection and retraction.
- Negative no-contact safety behavior works.
- Robot visual connectedness has been corrected and structurally checked.
- README documents verified commands and realism boundaries.

## Current control and perception state

### Control now

- Rover: proportional odometry controller.
- Arm: `FollowJointTrajectory` through `joint_trajectory_controller`.
- Comb: open-loop counter-rotating velocity command.
- Mission: finite-state machine with controller, stationary and contact interlocks.

### Perception now

- RGB-D and IMU streams are available.
- Robotar mission does not yet use image processing for target selection.
- Targets and arm poses are currently scenario-configured.

## Professional development plan

### Phase 1 — Clamp-shaker requirements

Investigate:

- Existing hand-held olive hook shakers.
- Tractor trunk shakers.
- Canopy harvesters.
- Robotic branch-shaking and pruning systems.
- Branch diameter, clamp force, vibration frequency, amplitude, duration and damage constraints.

Deliverable: a requirement range for branch clamp force, tool mass, vibration settings, sensors and safety limits.

### Phase 2 — New end-effector

Design a `shaker` tool with:

- Parallel-jaw soft clamp.
- Replaceable compliant pads.
- Enclosed vibration actuator.
- Force/torque sensor frame.
- Clamp IMU/accelerometer.
- Bark-slip and force-limit detection.

### Phase 3 — Perception

Implement:

1. RGB-D point-cloud generation.
2. Ground and distance filtering.
3. Branch clustering.
4. RANSAC/PCA branch-axis fitting.
5. Shake-point ranking.
6. YOLO branch/trunk/olive segmentation for field conditions.

Deep learning is appropriate for robust orchard perception. LLMs should not be used in real-time safety control.

### Phase 4 — Control

Upgrade from demo controls to:

- EKF localization using wheel odometry and IMU.
- Nav2 with MPPI for local navigation.
- MoveIt 2 for arm approach planning.
- Admittance control for compliant branch contact.
- Force-limited clamp control.
- Adaptive frequency shaking using measured acceleration response.
- IMU/stability interlocks for slope operation.

### Phase 5 — Stability and farmer workflow

Add:

- Sloped terrain simulation.
- Stabilizer legs.
- Self-leveling arm platform.
- Tilt, slip and force safety interlocks.
- Supervised farmer mode before full autonomy.

## Investigation sources to review

Prioritized reading and comparison topics:

- Mechanical harvesting of olive orchards and trunk-shaker reviews.
- Olive-tree forced-vibration dynamics.
- Pneumatic branch-shaker field studies.
- Fruit detachment force by cultivar and maturity.
- Clamp and bark-damage studies.
- RGB-D branch/trunk segmentation.
- Shake-point selection in fruit trees.
- Adaptive impedance/admittance control for agricultural grasping.
- Nav2 MPPI and mobile-manipulator stability.

Existing solutions to compare:

- Hand-held electric/pneumatic olive hook shakers.
- Tractor-mounted trunk shakers.
- Canopy and over-the-row harvesters.
- Robotic cherry/fruit branch-shaking research.
- Tree pruning and branch-localization robots.

## Acceptance criteria for a farmer-ready design

The product should be evaluated on:

- Detachment percentage on reachable branches.
- No visible bark damage.
- Olives collected in nets.
- Time per branch and per tree.
- Reliability over a full harvest day.
- Safe stop on tilt, slip, excessive force or obstacle detection.
- Operation on slopes where tractors are impractical.
- Tool swapping for spraying, pruning and inspection.
- Maintenance and cost that a small or medium farm can justify.

## Trial-entry template

Use this format for future entries:

```text
### YYYY-MM-DD — Short title

Goal:
Change made:
Verification:
Result:
Limitations discovered:
Decision/next action:
Evidence/log path:
```

### 2026-10-20 — Analysis paper library

**Goal:** collect open-access references for the branch clamp-shaker investigation.

**Work performed:**

- Created `analysis-papers/`.
- Downloaded and verified seven open-access PDFs covering olive mechanization, trunk-shaker power, branch vibration, fruit-stem dynamics, olive detection, branch segmentation and orchard 3D segmentation.
- Created `analysis-papers/README.md` with reading order, source links, relevance and papers still needed from the user or library access.

**Result:** the research base for the clamp-shaker requirement freeze is now version-controlled.

### 2026-10-20 — Difficult-terrain mobility research

**Goal:** find literature for stable motion and forceful work on slopes and rough orchard terrain.

**Work performed:**

- Reviewed open literature on agricultural UGV stability, hilly orchard path planning, orchard MPC and anti-slip control, adaptive platform leveling, rollover margins, tracked locomotion, Nav2 and MPPI.
- Downloaded and verified twelve additional open-access PDFs.
- Verified the user-provided Hoshyarmanesh olive-tree vibration paper and Ji adaptive-impedance paper.
- Expanded `analysis-papers/README.md` with a terrain-reading order, source links and browser-access papers that were bot-blocked for automated download.

**Result:** the next platform phase can be based on documented stability metrics, traction/slip modeling, active leveling and terrain-aware navigation instead of ad-hoc slope handling.

### 2026-10-20 — Library-access review list

**Goal:** identify important agricultural-robotics reviews and platform studies that are not available as legal open PDFs.

**Work performed:**

- Added exact titles, publisher/DOI links and relevance notes to `analysis-papers/README.md`.
- Covered foundational agricultural field-robot reviews, autonomous mobility, vision-based navigation, machinery management, labor/economics, human-robot interaction, safety/ergonomics, harvesting robots, multi-robot systems and side-slope/slip studies.

**Result:** missing papers can be requested through a university/library or from authors without using unofficial mirrors.

### 2026-10-20/21 — Branch clamp-shaker tool implemented (validation in progress)

**Goal:** replace the placeholder rotating comb with a professional enclosed branch clamp-shaker behind the `tool0` flange, per the approved plan (padded parallel jaws clamp the branch; counter-rotating eccentric rotors inside a housing provide the excitation, f = rotor_speed/2pi, sweep 10-25 Hz, shake 3-5 s — simulation tuning values anchored in the Messina 2025 review and Camposeo 2023, not hardware ratings).

**Work performed:**

- Added `tools.shaker` to the SSOT `physical.yaml` (housing, clamp jaws/pads, eccentric rotors, clamp IMU, topics/controllers), new `urdf/tools/shaker.xacro`, `tool:=shaker` wiring in `robotar.urdf.xacro`, `inc/ros2_control.xacro`, `scripts/robotar_common.sh` and `scripts/robotar_run_shaker.sh`. The comb remains as the legacy demo.
- Added `shaker_clamp_controller` (GripperActionController, max_effort as the force-limit proxy after Ji et al.) and `shaker_rotor_controller` (JointGroupVelocityController) to `controllers.yaml` and the spawner wiring.
- `generate_orchard.py`: the target branch is now a dynamic body on a revolute SDF spring/damper joint (stiffness/damping tuned so the 0.9 m ~2 kg branch resonates inside the 10-25 Hz sweep envelope); jaw pad contact sensors publish `/shaker/clamp_contact`.
- New mission `robotar_mission/shaker_demo.py` implementing the full state machine WAIT_FOR_CONTROLLERS -> STOW -> DRIVE_TO_TREE -> STOP -> PRE_POSITION -> APPROACH -> CLAMP -> VERIFY_CLAMP -> FREQUENCY_SWEEP -> CONTROLLED_SHAKE -> DETACH -> RELEASE -> RETRACT -> DONE, with base-stationary interlocks, clamp verification gating olive release, and a `miss_contact` negative mode. Arm poses were solved numerically (FK + swept-path collision checks).

**Debugging so far:**

- A misleading mid-run pass turned out to be contamination from an orphaned second `gz sim` instance (two Gazebo servers shared GZ transport topics and both hosted gz_ros2_control controller managers: spawners failed with "Controller already loaded / Failed to configure"). All runs must kill leftover sims first.
- Clean runs then failed deterministically in APPROACH: `goal_time_tolerance exceeded`, error_code -5. Live joint-state sampling showed `arm_wrist_pitch_joint` slamming to the URDF lower limit -2.10 (clamp pose was only 0.04 rad from it; quintic spline overshoot latched the joint on DART's limit constraint) and `arm_tool_pitch_joint` being knocked off track during the fast shoulder/elbow swing and not recovering.
- Fix in progress: wrist_pitch URDF range widened to +/-2.35 with effort 70 -> 120 Nm (tuning values), all mission poses re-solved with >= 0.15 rad joint-limit margins, and the APPROACH slide now passes through a joint-space midpoint so no spline segment carries the full ~1 rad shoulder/elbow swing at once.

**Status (updated 2026-10-21):** the wrist-limit latch was a CONSEQUENCE, not the cause — the next entry has the validated root cause (tool/olive contact wedge) and the final fix; the mission now passes end-to-end (positive DONE, negative clean). The wrist-limit widening and margins stay as defense-in-depth.

### 2026-10-21 — Shaker APPROACH aborts: root cause and fix

**Goal:** fix the deterministic `arm_controller` abort (`goal_time_tolerance` exceeded, error_code -5) in the shaker mission's APPROACH/slide.

**Diagnosis (multiple red herrings eliminated):**

- *Dual-sim contamination is real but was not the root cause.* Leftover Gazebo instances and mission nodes from parallel diagnosis sessions repeatedly poisoned validation (foreign action goals preempting mission trajectories; interleaved `/joint_states` from two sims). Also: `gz sim` sometimes ignores SIGTERM — kill with SIGKILL by PID and verify with `pgrep` before every run.
- *Not spline overshoot:* the cubic spline through the slide waypoints never undershoots its endpoints (verified numerically); the JTC reference stayed on-goal while joints diverged.
- *Not servo gain/effort/step size:* the stall persisted across `position_proportional_gain` 0.05/0.1/0.3, wrist/tool effort 70→400 Nm, and physics steps 4 ms → 1 ms.
- **Root cause: the tool wedged against the olives.** The housing (jaw_length 0.12) penetrated olive_1 by ~1 cm along the slide corridor. Olives are welded to the tree via DetachableJoint until the mission detaches them, so tool/olive contact anchors against the world; DART contact constraints starve the joint servos in the coupled LCP — joints crawl at ~0.005 rad/s under full servo command, get pushed off path, and in the first runs the wrist was driven onto its joint limit where DART's limit constraint latched it. Pad/branch contact topics stayed empty because the contact was with the olives (no sensors). Proven by detaching the olives in a live wedged sim: a previously stuck joint tracked a 0.5 rad probe immediately and cleanly (error_code 0).

**Fix:**

- `physical.yaml`: shaker `jaw_length` 0.12 → 0.30 (long hook jaws; housing stays below the olive cluster, deeper jaw mouth). Wrist/tool effort kept at 120 Nm (the 400/250 Nm experiment reverted — irrelevant to the wedge). `position_proportional_gain` set to 0.05 with corrected rationale; note this is the ONLY position-servo gain gz_ros2_control Jazzy supports (no per-joint PID; verified in the package source).
- `orchard.yaml`: olive_2 nudged (3.58,-2.50,1.36) → (3.62,-2.42,1.36) — the jaw plate grazed it at the miss-slide corridor start.
- `shaker_demo.yaml`: pose family re-solved numerically for the new pad height (FK + swept-path clearance against the branch cylinder and all six olives; every tool collision box ≥ 1 cm clear along the mission path; every joint ≥ 0.20 rad, wrist ≥ 0.30 rad from URDF limits). approach/clamp share the exact clamp orientation (pure-translation slide). New `miss_approach_pose`: miss mode shifts the whole maneuver +0.22 m in +x so the branch never enters the jaw mouth.
- `shaker_demo.py`: miss mode approaches via `miss_approach_pose`.

**Verification:**

- Isolated arm sequence at the tree (stow→pre→approach→clamp, jaws open): all segments track to goal (final joint error < 1e-6 rad), zero wedge stalls.
- Positive run (`ROBOTAR_TOOL=shaker ROBOTAR_GUI=0 ./scripts/robotar_run_shaker.sh`, log `data/logs/robotar_shaker_20261021_092928.log`): drive to x=2.980; pad/branch clamp contact confirmed on `/shaker/clamp_contact` (`clamp verified stalled=True`); sweep 10→25 Hz in 3 Hz steps (housing IMU response 0.8→14.7 m/s², peak 0.61 mm at 22 Hz); 4.0 s controlled shake at 22 Hz; 6/6 olives detached on command; all six olives settled inside the catch-net region (poses checked via `gz model`); release, retract, `MISSION DONE` with final odom (2.980, ~0).
- Negative run (`ROBOTAR_MISS_CONTACT=1`, log `robotar_shaker_20261021_094621.log`): miss approach/clamp on air, VERIFY_CLAMP timed out with no pad/branch contact, no sweep, no shake, **zero detach commands** (olives still attached afterwards), jaws released, clean `MISSION FAILED` exit 1.

**Limitations discovered:** DART's JointConstraint gives the servo lowest priority after position/velocity limits, and joints driven onto a position limit can latch (the jaw `limit_margin` pattern mitigates); keep pose margins ≥ 0.2 rad. The wrist-limit widening (±2.35) stays as defense-in-depth.

**Decision/next action:** shaker mission validated end-to-end. Next: branch-compliant shake physics realism (branch spring response vs frequency) and RGB-D shake-point selection.

## Current next action

Shaker demo validated end-to-end on 2026-10-21 (positive run reached DONE with clamp verified, 10-25 Hz sweep, 4 s shake, 6/6 olives detached and landed in the catch region; `ROBOTAR_MISS_CONTACT=1` failed cleanly with zero detach commands — see the 2026-10-21 entry). Next: shake-physics realism (branch spring response vs frequency, pad force limits) and RGB-D shake-point selection.
