# Unitree Go2 Ground-Reaction-Force MPC + Kalman Filter

This repository contains a ROS 2 controller for the Unitree Go2 in MuJoCo. The main goal of the current implementation is to bring the robot into a known crouched posture, initialize a leg-kinematics-based Kalman filter, and then use a convex Model Predictive Controller (MPC) to calculate ground-reaction forces for standing and body stabilization.

This package implements a ground force-based Convex Model Predictive Control (MPC) for the Unitree Go2 quadruped robot. The controller architecture and state-space dynamics are derived from the MIT Cheetah 3 paper:

> Di Carlo, J., Wensing, P. M., Katz, B., Bledt, G., & Kim, S. (2018). **"Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control."** *IEEE International Conference on Intelligent Robots and Systems (IROS).*

This repository is still a research and learning project rather than a finished locomotion stack. It currently focuses on a four-foot standing case; gait generation, swing-leg control, contact handling, and hardware safety are still under development.

> **Safety note:** Run this in simulation only. The current controller is not ready for a physical Go2. In particular, the emergency-stop, contact switching, and several whole-body safety checks are not complete.

---

## What the repository contains

The workspace contains two ROS 2 packages:

```text
src/
├── controller/       # Estimator, MPC, high-level supervisor and Go2 model
└── go2_interfaces/   # Custom ROS 2 messages shared between the nodes
```

The `controller` package builds three executable nodes:

| Node | Executable | Main job |
|---|---|---|
| State estimator | `state_estimator_node` | Uses IMU, joint states and leg kinematics to estimate body position and velocity |
| MPC | `mpc_node` | Builds and solves the ground-reaction-force QP using OSQP |
| High-level controller | `controller_node` | State machines to ensure a proper control flow |

All three nodes are started by:

```bash
ros2 launch controller mpc.launch.py
```

---

## Controller data flow

In normal operation:

1. MuJoCo publishes the robot sensor data on `/lowstate`.
2. The high-level controller smoothly moves the joints to the configured crouched posture.
3. The high-level controller requests Kalman-filter initialization through `/kf_initialize`.
4. The estimator publishes `/estimated_state` after initialization.
5. After the estimator warm-up period, the controller enables the MPC through `/mpc_initialize`.
6. The MPC builds a reference, predicts the body dynamics, solves the QP, and publishes twelve foot-force values on `/mpc_command`.
7. The high-level controller maps each world-frame foot force to joint torque using `-Jᵀf`, adds posture PD control, and publishes `/lowcmd` to MuJoCo.

---

## Current startup sequence

The high-level controller contains the following state-machine states:

```text
BOOT
  ↓
IDLE
  ↓
SMOOTH_RAISE
  ↓
KF_INITIALIZE
  ↓
MPC_INITIALIZE
```

The enum also contains `MPC_RUNNING`, `STOP`, and `EMERGENCY_STOP`, but the current source does not yet transition into a separate `MPC_RUNNING` state. MPC commands are presently applied inside `MPC_INITIALIZE`.

The current sequence is:

- **BOOT:** Wait for the first `/lowstate` message.
- **IDLE:** Check that the twelve joint measurements and the configured reference are finite and valid.
- **SMOOTH_RAISE:** Use quintic interpolation to move from the measured startup posture to `q_ref`.
- **KF_INITIALIZE:** Initialize the Kalman filter and keep holding the joint posture during the warm-up time.
- **MPC_INITIALIZE:** Enable the MPC, receive foot forces, convert them to torque, and publish the motor command.

The configured `q_ref` currently represents the initial PD-controlled crouched posture.

Check the REPOGUIDE.md to understand the full repository's code structure and functionalities. 
---

## Dependencies

The code is currently set up for the following environment:

- ROS 2 Humble
- C++17
- Eigen3
- Unitree ROS 2 messages (`unitree_go`)
- The local `go2_interfaces` package
- `osqp-cpp`
- Unitree MuJoCo simulator
- Cyclone DDS/your project DDS loopback configuration

The controller CMake file currently expects `osqp-cpp` at:

```text
/opt/osqp-cpp
```

This path is hard-coded in `controller/CMakeLists.txt`:

```cmake
add_subdirectory(
  /opt/osqp-cpp
  ${CMAKE_BINARY_DIR}/osqp-cpp
  EXCLUDE_FROM_ALL
)
```

If `osqp-cpp` is installed elsewhere, update this path before building.

You can also use the DockerFile and devcontainer to load the environment without installing anything extra / from the source. 
---

# Build tutorial

The commands below assume the ROS 2 workspace is `/workspace` and the packages are located in `/workspace/src`.

## 1. Enter the workspace

```bash
cd /workspace
```

## 2. Source ROS 2

```bash
source /opt/ros/humble/setup.bash
```

## 3. Build the message and controller packages

```bash
colcon build \
  --symlink-install \
  --packages-select go2_interfaces controller
```

`go2_interfaces` must be built because the three controller nodes use its custom messages.

## 4. Source the workspace

```bash
source install/setup.bash
```

You must repeat this in every new terminal, unless your shell setup does it automatically.

After changing a `.cpp`, `.h`, `.msg`, `CMakeLists.txt`, or `package.xml` file, rebuild the affected packages and source the workspace again.

---

# Run tutorial

The controller and simulator are normally run in two terminal windows.

## Terminal 1 — Start the ROS 2 controller

```bash
cd /workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
source dds-loopback
ros2 launch controller mpc.launch.py
```

In this project, `source dds-loopback` applies the local DDS configuration used for communication between the ROS 2 controller and the simulator.

The launch command starts:

```text
/state_estimator
/high_level_control
/mpc_node
```

It is normal for the nodes to wait for `/lowstate` before the simulator starts.

## Terminal 2 — Start MuJoCo

Open a second terminal window and go to the simulator directory containing `start.sh`:

```bash
cd <path-to-the-unitree-mujoco-directory>
./start.sh
```

After MuJoCo starts publishing `/lowstate`, the controller should automatically proceed through its startup sequence.

## Expected terminal progression

You should see messages similar to:

```text
Controller active! Waiting for simulation ticks...
Control supervisor started
BOOT successful. IDLING and checking sensors/references.
Checks successful. Starting PD smooth raise.
Starting 5.00-second smooth crouch transition.
Smooth crouch transition completed.
PD smooth raise completed. Initializing Kalman filter.
KF initialization requested
Kalman filter initialized.
KF warmup completed. Initializing MPC.
Starting 5.00-second smooth stand transition.
```

The MPC node also prints the summed optimized forces and the current/reference body position.

---

## Useful ROS 2 checks

Run these commands in a terminal where ROS, the workspace, and DDS loopback have been sourced.

### Confirm that all nodes are running

```bash
ros2 node list
```

Expected controller nodes:

```text
/high_level_control
/mpc_node
/state_estimator
```

### Inspect the available topics

```bash
ros2 topic list
```

### Check whether the simulator is publishing sensor data

```bash
ros2 topic hz /lowstate
```

The expected rate in the current setup is approximately 500 Hz.

### Check the estimator output rate

```bash
ros2 topic hz /estimated_state
```

### View the current estimated state

```bash
ros2 topic echo /estimated_state
```

### View the MPC forces

```bash
ros2 topic echo /mpc_command
```

The force order is FR, FL, RR, RL, with x/y/z components for each foot.

### View detailed estimator diagnostics

```bash
ros2 topic echo /estimator
```

For longer tests, PlotJuggler or Foxglove is usually easier than printing this message directly.

### Inspect connections

```bash
ros2 node info /state_estimator
ros2 node info /mpc_node
ros2 node info /high_level_control
```

---

## Main parameters to tune

For most experiments, begin with `controller/config/mpc_params.yaml`.

| Parameter | Current value | Purpose |
|---|---:|---|
| `obs_dt` | `0.002` | Estimator timestep; intended for 500 Hz `/lowstate` |
| `joint_dt` | `0.002` | High-level motor-command loop period |
| `mpc_dt` | `0.02` | MPC solve period; 50 Hz |
| `crouch_transition_time` | `5.0` | Time used to interpolate from startup joints to `q_ref` |
| `stand_transition_time` | `5.0` | Time used to interpolate body height toward `z_ref` |
| `kf_warmup_time` | `4.0` | Estimator settling time before enabling MPC |
| `z_ref` | `0.3` | Desired body-height reference |
| `gains.kp` | `35.0` | PD position gain during the initial smooth transition |
| `gains.kd` | `1.5` | PD velocity gain during the initial smooth transition |
| `q_ref` | 12 values | Initial joint reference in FR, FL, RR, RL order |

---

## How the MPC works

At every MPC tick:

1. Read the latest estimated body state, foot positions, and Jacobians.
2. Hold the x, y, and yaw values captured when MPC starts.
3. Generate a smooth height reference from the initial estimated height to `z_ref`.
4. Build the continuous simplified rigid-body A and B matrices.
5. Discretize them using zero-order hold.
6. Unroll the model over five prediction steps.
7. Build a quadratic state-tracking and force-effort cost.
8. Add friction-pyramid, normal-force, and joint-torque constraints.
9. Solve the QP with OSQP.
10. Publish only the first force vector from the optimized horizon.

The optimized forces are not motor torques. The high-level controller performs the conversion using each leg Jacobian.

---

## Known limitations and unfinished work

This is the most important section for anyone modifying the repository.

### Standing only

All four feet are currently treated as contacts. There is no gait planner, swing-foot trajectory, or contact schedule.

### No swing-leg force constraint

The MPC TODO explicitly notes that forces for swing legs still need to be forced to zero when gait support is added.

### Contact flags are not yet dynamic

The estimator publishes contact information, but the current planned-contact vector is initialized to four `true` values and is not updated from measured contact forces.

### Incomplete state machine

`MPC_RUNNING`, `STOP`, and `EMERGENCY_STOP` exist in the enum, but the current active path remains in `MPC_INITIALIZE`. `runEmergencyStop()` is empty.

### Solver-failure fallback is incomplete

The MPC sets its local force vector to zero when OSQP fails, but `mpc_node.cpp` currently returns without publishing a replacement command. A command timestamp, validity flag, timeout, and safe fallback should be added.

### Fixed model and limits

Robot mass, inertia, link dimensions, friction, force limits, and torque limits are defined in source files rather than loaded from YAML or lib like pinocchio.

### No hardware-ready command safety

Final motor torque clamping, LowCmd mode initialization, CRC handling, stale-command checks, and tested physical emergency behaviour must be completed before hardware use.

---

## Reference

The controller structure is inspired by:

> J. Di Carlo, P. M. Wensing, B. Katz, G. Bledt, and S. Kim, “Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control,” IEEE/RSJ International Conference on Intelligent Robots and Systems, 2018.

---

## Quick command summary

### Build

```bash
cd /workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select go2_interfaces controller
source install/setup.bash
```

### Run the controller

```bash
cd /workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
source dds-loopback
ros2 launch controller mpc.launch.py
```

### Run the simulator in a new terminal

```bash
cd <path-to-the-unitree-mujoco-directory>
./start.sh
```
