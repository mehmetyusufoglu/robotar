# Robotar agricultural simulation workspace

This workspace contains the original Robotar olive-harvesting simulation plus two isolated third-party agricultural references. Project-owned files live under `/home/memo/projects/robotar`; ROS and other APT dependencies remain in their standard system locations.

## Verified status

| Simulation | Environment | Verified result |
|---|---|---|
| **Robotar olive platform** | Native ROS 2 Jazzy, Gazebo Harmonic, NVIDIA GTX 1650 | Builds successfully; drive, arm, comb and gripper controllers run; the scripted mission drives 2.952/3.000 m, confirms comb/branch contact, detaches all 6 targeted olives, collects all 6 in the catch region and retracts. The no-contact mode blocks detachment and fails safely. GUI and headless runs passed. |
| Panther + Franka FR3 tomato harvesting | Native ROS 2 Jazzy, Gazebo Harmonic | Builds successfully; robot/controllers and sensors run; NVIDIA Gazebo rendering and CUDA YOLO were confirmed by `nvidia-smi`; tomato mapping, Nav2 and MoveIt goals run. A CPU run reached the first harvest-approach trajectory. A completed pick and basket drop were not tested. |
| Amiga + Kinova orchard | Dockerized ROS 2 Humble, Gazebo Fortress | Minimal image and packages build; one robot spawns; base/arm/gripper controllers and sensors run; MoveIt homing completes; Gazebo GUI is available through noVNC. |

All verification workloads were stopped after testing. No demo containers or ROS/Gazebo demo processes should be left running.

## Layout

```text
config/third_party.repos     Canonical upstream URLs and base revisions
docker/amiga/compose.yaml    Isolated Amiga/noVNC runtime
scripts/                     Dependency, build, run and stop entry points
third_party/                 Third-party source checkouts
venvs/yolo/                  Project-local CPU/CUDA inference environment
ws/src/robotar_description/  Original rover, 6-DOF arm, sensors and tools
ws/src/robotar_gazebo/       Generated olive orchard, controllers and bridges
ws/src/robotar_mission/      Closed-loop drive and harvest state machine
data/logs/                   Build and runtime evidence
```

Scripts locate the project from their own paths, so they can be launched from any current directory. The upstream base revisions are recorded only in `config/third_party.repos`.

## Host preparation

The following steps have already been completed on this machine:

```bash
sudo bash scripts/install_ros2_jazzy.sh
sudo ./scripts/install_tomato_dependencies.sh
sudo bash scripts/install_nvidia_driver.sh
TOMATO_DEVICE=cuda:0 ./scripts/tomato_setup_yolo.sh
```

The tomato dependency installer requires root and finishes with a `rosdep check`; it must report that all system dependencies are satisfied. The NVIDIA installer handles DKMS and the currently running kernel's headers. Verify the driver with `nvidia-smi` before selecting CUDA.

The YOLO setup supports CPU and CUDA PyTorch 2.14.1 builds and uses a NumPy/OpenCV combination compatible with Jazzy's `cv_bridge`. The current environment uses `torch 2.14.1+cu130` on the GTX 1650. To select a CPU environment instead, run `TOMATO_DEVICE=cpu ./scripts/tomato_setup_yolo.sh`.

## Robotar olive-harvesting platform

The first-party MVP is an original four-wheel skid-steer field rover with a side-mounted 6-DOF arm, mast RGB-D camera, chassis IMU and launch-selectable tools. The arm has a chassis-connected pedestal and visible joint bearing caps; wheels, mast, camera, flange and tool are visually connected. The `comb` tool has twin counter-rotating tine banks visible through an open guard frame; the `gripper` tool proves the interchangeable flange/controller interface. The generated orchard contains two rows of low-poly olive trees, one instrumented branch, six dynamic olives and a catch net.

Single sources of truth:

- `ws/src/robotar_description/config/physical.yaml` — robot geometry, joints, limits, sensors and topic names.
- `ws/src/robotar_gazebo/config/orchard.yaml` — orchard layout, target branch, olives, catch region and detach topics.
- `ws/src/robotar_mission/config/harvest_demo.yaml` — mission timing, tolerances, arm poses and comb motion.

Build:

```bash
./scripts/robotar_build.sh
```

Watch the complete scripted harvest in Gazebo and RViz:

```bash
ROBOTAR_GUI=1 ROBOTAR_GPU_RENDER=1 ROBOTAR_TOOL=comb \
  ./scripts/robotar_run_harvest.sh
```

The rover drives approximately 3 m, stops, positions the arm, spins the comb only while stationary, requires physical comb/branch contact, releases the six targeted olives, and retracts. Gazebo shows the orchard and open comb guard; RViz shows the robot model, TF and the live `/camera/mast/image_raw` RGB image. Stop the remaining simulation windows with **Ctrl-C** in the launch terminal.

Do not use Gazebo's world/model **Reset** button in this MVP. The robot is spawned from `robot_description` after the world starts; Gazebo reset removes that spawned model without rerunning the ROS spawn and controller bringup. Restart with Ctrl-C and the run script instead. A safe lifecycle reset supervisor remains future work.

Robot links do not self-collide by default, canopy foliage is not modeled as individual collision bodies, and comb tines use coarse swept collision cylinders. These are deliberate MVP simplifications that prevent adjacent joints and dense foliage from locking the simulation. Visual axles, mast connections, the arm pedestal, joint bearing caps and the tool flange adapter are modeled; a generated-URDF adjacency check confirms that visible parent/child bounds touch or overlap for both tools. Selective self-collision and contact filtering are future work.

Run headless instead with `ROBOTAR_GUI=0`. Run the safety-gate check, which is expected to fail cleanly without publishing detach commands:

```bash
ROBOTAR_MISS_CONTACT=1 ROBOTAR_GUI=0 ROBOTAR_TOOL=comb \
  ./scripts/robotar_run_harvest.sh
```

Inspect the gripper variant without the harvest mission:

```bash
ROBOTAR_GUI=1 ROBOTAR_GPU_RENDER=1 ROBOTAR_TOOL=gripper \
  ./scripts/robotar_run_sim.sh
```

With that simulation running, an open/close smoke command is:

```bash
source scripts/robotar_common.sh
robotar_source_overlay
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  '{command: {position: 0.04, max_effort: 5.0}}'
```

## Tomato harvesting: build and run

Build the three overlays in their required order:

```bash
./scripts/tomato_build.sh
```

The default is two build workers. Override it when memory is constrained, for example:

```bash
TOMATO_BUILD_WORKERS=1 ./scripts/tomato_build.sh
```

Start only the greenhouse, robot, controllers and MoveIt with NVIDIA rendering:

```bash
TOMATO_GPU_RENDER=1 TOMATO_GUI=0 ./scripts/tomato_run_gazebo.sh
```

Stop it with **Ctrl-C** before starting the autonomous mission.

To watch the autonomous mission on the Ubuntu desktop, open Gazebo and RViz while using NVIDIA rendering and CUDA inference:

```bash
TOMATO_GPU_RENDER=1 TOMATO_DEVICE=cuda:0 TOMATO_GUI=1 \
  ./scripts/tomato_run_harvest.sh
```

The integrated GPU verification used reduced GUI mode:

```bash
TOMATO_GPU_RENDER=1 TOMATO_DEVICE=cuda:0 TOMATO_GUI=0 \
  ./scripts/tomato_run_harvest.sh
```

For CPU inference and the default renderer, use:

```bash
TOMATO_GPU_RENDER=0 TOMATO_DEVICE=cpu TOMATO_GUI=0 \
  ./scripts/tomato_run_harvest.sh
```

Stop a mission with **Ctrl-C**. The wrapper supplies the lower-cost world, project-local YOLO model and Python site-packages automatically. `TOMATO_GUI=1` opens Gazebo and RViz but consumes more resources. While a mission is running, `nvidia-smi` should show Gazebo as a graphics (`G`) process and YOLO as a compute (`C`) process.

Useful inspection commands in another terminal:

```bash
source scripts/tomato_common.sh
tomato_source_overlays
ros2 control list_controllers
ros2 topic list -t
```

Expected controllers are `fr3_arm_controller`, `fr3_gripper_controller`, `drive_controller` and `joint_state_broadcaster`.

### Local tomato adaptations

The tomato checkout remains at its pinned base revision but is intentionally not clean. The verified setup includes:

- A required symlink exposing the pinned FR3 MoveIt configuration to `combined_ws`.
- Directory-symlink asset installation and relative plant-mesh paths instead of developer-specific absolute paths.
- Basket visuals generated from existing collision geometry because the original external mesh URIs were unavailable.
- One-Hz RGB/depth rendering, introduced originally for CPU inference and retained for the verified GPU configuration.
- `tomato_farm_sera_cpu.sdf`, which retains model names, poses, collisions and physics while using lower-cost tomato visuals.

Do not reset or clean this checkout without preserving these adaptations. Existing upstream shutdown defects can print Gazebo memory errors or duplicate rclpy shutdown messages after Ctrl-C; they were observed during teardown, not during the verified runtime.

## Amiga orchard: build and run

The first build downloads a large container image (approximately 18 GB) and builds the minimal required workspace packages:

```bash
./scripts/amiga_build.sh
```

Start the one-robot orchard in terminal 1:

```bash
./scripts/amiga_sim.sh gazebo
```

Open the Gazebo desktop through noVNC:

```text
http://127.0.0.1:8080/vnc.html
```

After the controllers are active, start the MoveIt arm layer in terminal 2:

```bash
./scripts/amiga_sim.sh arm
```

Check controllers from the host:

```bash
docker compose -f docker/amiga/compose.yaml exec -T ros \
  bash -ic 'ros2 control list_controllers -c /controller_manager'
```

Software rendering can make initial controller transitions time out. If startup leaves a controller inactive, retry only that transition, for example:

```bash
docker compose -f docker/amiga/compose.yaml exec -T ros bash -ic \
  'ros2 control set_controller_state joint_trajectory_controller active -c /controller_manager'
```

Stop all Amiga containers from a host terminal:

```bash
./scripts/amiga_stop.sh
```

The built images and named build/install/log volumes remain available for the next run. The noVNC page is unauthenticated but is bound only to host loopback. The Compose runtime deliberately has no SSH mount, host devices, privileged mode, GPU passthrough or host networking. NVIDIA Container Toolkit is not installed, so the verified Amiga configuration remains software-rendered.

## Verification boundaries

- The Robotar mission is a scripted vertical slice: it uses scenario-configured drive and arm poses, not camera perception, Nav2 or MoveIt. Detachment is contact-gated task logic through Gazebo detachable joints, not a stem-force, branch-vibration, fruit-damage or harvest-efficiency model. The 6/6 catch result is a functional simulation check, not a validated field rate.
- Robot dimensions, mass and arm/tool parameters are simulation design assumptions, not validated hardware ratings or a ruggedness certification.
- The Amiga run used one robot without Nav2, behavior trees, coordination, agents, VLM or real hardware. Its orchard harvest action remains a placeholder; no fruit was simulated.
- The CPU tomato run received real simulated camera frames and detections, completed navigation goals, selected a ripe tomato and began executing its first MoveIt harvest-approach trajectory. The CUDA run independently confirmed NVIDIA Gazebo rendering, real camera frames, non-empty detections, tomato mapping and successful Nav2/MoveIt goals. Neither run completed a pick or basket drop.
- A warmed synthetic CUDA benchmark measured approximately 14.4 ms mean end-to-end YOLO prediction time. This was not a controlled CPU-versus-GPU or complete-mission benchmark.
- Tomato transport uses the upstream contact-triggered attachment approach. Neither result is a validated olive-detachment or plant-contact physics model.
- Review each upstream project's code and asset licenses before reusing its content in a derived product.

## Logs

Latest build and runtime logs are under `data/logs/`, including Robotar positive/negative harvest runs, GUI harvest evidence, Amiga controller/sensor/MoveIt checks, and tomato build/Gazebo/YOLO/CUDA runs. Files with `gpu` in their names contain the NVIDIA process and runtime evidence. These logs support the verification boundaries above, not benchmarks for future olive-harvesting performance.
