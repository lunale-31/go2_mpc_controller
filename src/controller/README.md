# Controller Package

This package implements a ground force-based Convex Model Predictive Control (MPC) for the Unitree Go2 quadruped robot. The controller architecture and state-space dynamics are derived from the MIT Cheetah 3 paper:

> Di Carlo, J., Wensing, P. M., Katz, B., Bledt, G., & Kim, S. (2018). **"Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control."** *IEEE International Conference on Intelligent Robots and Systems (IROS).*

## Overview
The controller computes optimal ground reaction forces using a convex MPC formulation and tracks a reference state trajectory for stand-up/locomotion. 
The package is implemented using ROS2 (majorly C++ except launch files) and consists of dynamics, kinematics, MPC and ROS2 interface for controller callback. 

## Walkthrough 

### 1) Config (config/mpc_params.yaml) 
 - Contains all the parameters that is required for the controller to run. (For example: Reference signal)
 
### 2) Launch (launch/mpc_launch.py) 
 - Contains the code to launch the "go2_mpc_controller" node with config param file, which contains the main callback loop. 

### 3) Source code (src)
- **dynamics**: Contains the code related to the state-space model that has been discussed mathematically in the paper. 
- **go2_mpc**: Contains the code related to the controller command loop. 
- **kinematics**: Contains the code related to the go2 robot's forward and inverse kinematics. 
- **main**: Entry point for the ros2 node.  

