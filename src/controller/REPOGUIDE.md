
## Repository guide

### `controller/config/mpc_params.yaml`

This is the first file to edit for normal experiments. It contains the parameters that are expected to change between runs:

- Estimator timestep
- High-level joint-control timestep
- MPC timestep
- Initial joint posture
- Joint PD gains
- Kalman-filter warm-up duration
- Smooth transition durations
- Desired body height

The joint order is:

```text
FR hip, FR thigh, FR calf,
FL hip, FL thigh, FL calf,
RR hip, RR thigh, RR calf,
RL hip, RL thigh, RL calf
```

Treat `q_ref` as an experimental robot-model-dependent value. Verify the joint order and left/right hip signs before changing it or using a different Go2 model.

### `controller/launch/mpc.launch.py`

Starts all three controller nodes and loads `mpc_params.yaml` into each node:

- `state_estimator_node`
- `controller_node`
- `mpc_node`

Modify this file when adding node remappings, namespaces, extra nodes, or a different parameter file.

### `controller/src/controller_node.cpp`

Contains the high-level supervisor and the final motor-command path.

Important functions:

- `controlLoop()` — startup state machine
- `runSmoothRaise()` — quintic joint interpolation and PD command
- `runMpcCommand()` — converts the four MPC forces into twelve joint torques
- `changeState()` — state-transition logging
- `runEmergencyStop()` — currently unfinished

This is the file to modify when changing:

- Startup behaviour
- State-machine transitions
- Joint posture control during MPC
- Ground-force-to-torque mapping
- Motor command limits and safety handling
- MPC command timeout or fallback behaviour

The current MPC torque command uses:

```text
τleg = -Jworldᵀ fworld
```

inside `runMpcCommand()`.

### `controller/src/mpc_node.cpp`

ROS 2 wrapper around the MPC implementation.

It:

- Receives `/estimated_state`
- Waits for `/mpc_initialize`
- Stores the initial x, y, and yaw as hold references
- Generates a quintic body-height trajectory toward `z_ref` (height)
- Builds the 13-state MPC vector
- Calls the dynamics, cost, constraints, and OSQP solver
- Publishes the first optimized force input on `/mpc_command`

Modify this file when changing:

- The standing reference
- Desired roll, pitch, yaw, position, or velocity
- Reference trajectory generation
- MPC initialization behaviour
- What is published when the solver fails

The 13-state MPC order is:

```text
[roll, pitch, yaw,
 position x, position y, position z,
 angular velocity x, angular velocity y, angular velocity z,
 linear velocity x, linear velocity y, linear velocity z,
 gravity state]
```

The 12-input order is:

```text
[FR fx, FR fy, FR fz,
 FL fx, FL fy, FL fz,
 RR fx, RR fy, RR fz,
 RL fx, RL fy, RL fz]
```

### `controller/src/mpc.cpp`

Contains the actual QP formulation.

Main responsibilities:

- Define state, input, and terminal weights
- Unroll the discrete dynamics over the prediction horizon
- Construct the quadratic cost
- Construct friction and joint-torque constraints
- Convert the matrices to sparse Eigen matrices
- Solve the QP through `osqp-cpp`
- Return the first twelve force components

Modify `setWeights()` when tuning the MPC cost.

Current state weights are larger for:

- Roll and pitch
- Body height
- Vertical velocity

Current input weights are `1e-3` for each force component.

### `controller/include/controller/mpc.h`

Contains the MPC dimensions and limits.

The current values include:

```text
Prediction horizon hp = 5
Control horizon    hc = 5
Friction coefficient  = 0.4
Normal force range    = 0 to 100 N per foot
Hip/thigh torque      = ±23.7 Nm
Calf torque           = ±45.43 Nm
```

With `mpc_dt = 0.05 s`, the current five-step prediction horizon covers approximately `0.25 s`.

Modify this file when changing:

- Prediction/control horizon
- Friction coefficient
- Normal-force bounds
- Joint-torque limits
- MPC matrix dimensions

### `controller/src/dynamics.cpp`

Implements the simplified 13-state rigid-body dynamics used by the MPC.

It contains:

- Robot mass
- Body inertia
- Roll, pitch, and yaw rotation matrices
- Skew-symmetric matrix generation
- Continuous A and B matrix construction
- Zero-order-hold discretization using a matrix exponential

Modify this file when changing robot physical parameters or the simplified rigid-body model.

### `controller/src/kinematics.cpp`

Contains analytical leg kinematics:

- Forward kinematics
- Inverse kinematics
- Leg Jacobian

### `controller/include/controller/kinematics.h`

Contains leg link lengths, joint limits, and the left/right leg-side definition.

Modify this header when adapting the package to a different Go2 model or correcting geometric parameters. Changes here affect the estimator, foot positions, Jacobians, torque constraints, and force-to-torque mapping.

### `controller/src/state_estimator_node.cpp`

Connects the Kalman filter to ROS 2 and the Go2 sensor messages.

It:

- Reads IMU orientation, angular velocity, and acceleration
- Reads joint positions and velocities
- Computes body- and world-frame foot positions
- Computes world-frame foot velocities and Jacobians
- Initializes the Kalman filter after receiving `/kf_initialize`
- Publishes `/estimated_state` and `/estimator`

The current implementation assumes all four feet are planned contacts:

```cpp
planned_contacts = std::vector<bool>(4, true);
```

Modify this file when adding:

- Contact detection
- Gait-dependent stance/swing handling
- Different frame transformations
- Additional sensor validation
- A different estimator initialization procedure

### `controller/src/state_estimator.cpp`

Contains the 18-state linear Kalman filter.

The estimator state is:

```text
[body position (3),
 body velocity (3),
 FR foot position (3),
 FL foot position (3),
 RR foot position (3),
 RL foot position (3)]
```

It uses:

- IMU acceleration for prediction
- Relative foot positions
- Foot-velocity constraints
- Zero foot-height assumptions for stance feet
- Normalized Innovation Squared (NIS) for debugging

Modify this file when tuning:

- Initial covariance `P`
- Process covariance `R1`
- Measurement covariance `R2`
- Measurement model
- NIS calculation

### `controller/include/controller/foot_index.h`

Provides the shared FR/FL/RR/RL and hip/thigh/calf indexing helpers.

Use this instead of manually inventing another motor order.

### `controller/src/*_main.cpp`

Small ROS 2 entry points that initialize ROS, create the corresponding node, spin it, and shut ROS down.

You normally do not need to modify these files.

### `go2_interfaces/msg/EstimatedState.msg`

Carries the estimator output to the MPC and high-level controller. It includes:

- Estimated state
- RPY
- Position and velocity
- World angular velocity
- Four foot positions
- Four world-frame Jacobians
- Contact flags
- NIS
- Initialization and validity flags

### `go2_interfaces/msg/EstimatorDebug.msg`

Carries detailed estimator diagnostics for PlotJuggler, Foxglove, logging, or manual inspection:

- Raw and transformed IMU acceleration
- Predicted and filtered state
- Measurement and predicted measurement
- Innovation
- Covariance diagonals
- NIS

### `go2_interfaces/msg/MpcCommand.msg`

Carries the twelve optimized ground-force components from the MPC to the high-level controller.

---