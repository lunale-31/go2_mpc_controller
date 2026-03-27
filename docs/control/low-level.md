# Low-level Control

In the low-level control mode, all joints can be controlled manually.
To do so, you must **turn off** the robot's internal controller (see [this document](motion-switcher.md) for details), as it will otherwise interfere with your control inputs.

In Unitree's terminology, the low-level control is called the [Basic Services Interface](https://support.unitree.com/home/en/developer/Basic_services).
We provide a wrapper this interface in the [`LowLevelControl` class](../../src/interface/LowLevelControl.h).

When instanciated, am `LowLevelControl` object `llc` provides methods to access all joints via their corresponding legs (e.g., `llc.frontRightLeg().hip()` returns a reference to the front-right hip joint).
In that, we follow [Unitree's naming convention](https://support.unitree.com/home/en/developer/about_Go2#heading-10) for joints.
When looking at the robot from the top, we consider the LiDAR sensor to point to the front and define the left and right sides accordingly.
Each leg consists of three joints, which we call, going from the torso to the feet, _hip_, _thigh_, and _calf_.

Each joint object `j` offers access to its current state via the `j.state()` method.
To control a joint, there exist two options: Torque control, or a pre-provided PD controller.
- For torque control, a torque value can be set with the `j.tau(float torque)` method.
Torque control requires the internal PD controller to be turned off.
This can be accomplished by setting its gains to zero (i.e., calling `j.kp(0.0f)` and `j.kd(0.0f)`).
- To use the pre-provided PD controller, its gains can be set by calling `j.kp(float prop_gain)` and `j.kd(float deriv_gain)`.
Then, a desired target angle can be provided by calling `j.q(float angle)`, or a desired angular velocity can be provided by calling `j.dq(float ang_velo)`.

To work, the joint motors must be turned on.
This can be done using the `j.mode(int mode)` method.
Passing it the value `0x0` deactivates the motor, while passing it `0x1` will activate it.