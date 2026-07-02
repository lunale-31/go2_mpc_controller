#include "go2_mpc.h"

Go2MPC::Go2MPC() : rclcpp::Node("go2_mpc_controller")
{
    publisher_ = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd",10);
    subscriber_ = this->create_subscription<unitree_go::msg::LowState>("/lowstate", 10, std::bind(&Go2MPC::controller_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Controller active! Waiting for simulation ticks...");

    loadParams(); 

    // Kinematics parameters 
    leg_sides = {
        common::LegSide::RIGHT, // FR
        common::LegSide::LEFT,  // FL
        common::LegSide::RIGHT, // RR
        common::LegSide::LEFT   // RL
    };

    // Hip_offset because kinematics vector is from hip to calf. We need to consider the transformation from COM to the hip attachment too.  
    hip_offset = {
        Eigen::Vector3d(0.1934, -0.0465, 0.0), // FR
        Eigen::Vector3d(0.1934,  0.0465, 0.0), // FL
        Eigen::Vector3d(-0.1934, -0.0465, 0.0), // RR
        Eigen::Vector3d(-0.1934,  0.0465, 0.0)  // RL
    };
}

void Go2MPC::loadParams(){
    /* Declare parameter is called only when there is no YAML file is provided for loading the parameters. */
    // Get sampling time
    this->declare_parameter<double>("dt", 0.01); 
    this->get_parameter("dt", dt); 

    // Reference joint positions
    this->declare_parameter<std::vector<double>>("q_ref", std::vector<double>());
    this->get_parameter("q_ref", q_ref); 

    // State reference 
    this->declare_parameter<std::vector<double>>("x_ref", std::vector<double>());
    this->get_parameter("x_ref", x_ref);

    // Gains
    this->declare_parameter<double>("gains.kp", 10.0);
    this->declare_parameter<double>("gains.kd", 1.0);

    this->get_parameter("gains.kp", kp_);
    this->get_parameter("gains.kd", kd_);
    
    // Sanity check
    RCLCPP_INFO(this->get_logger(), "Loaded dt: %f", dt);
    RCLCPP_INFO(this->get_logger(), "Loaded q_ref size: %zu", q_ref.size());
}

void Go2MPC::controller_callback(const unitree_go::msg::LowState::SharedPtr msg){
    /*TODO: 1) Work on logging the data and plotting them to debug properly. 
        2) Work on calculating r_i every time step and get B with it every step (Use forward kinematics function)
        3) Work on Developing MPC.h and MPC.cpp 
        3) Keep a single reference for stand up, use a TrajectorGeneration.cpp file for generating variety of trajectories. 
        4) Find the jacobian matrix to compute tau from forces obtained from optimal control
        5) Create a Gait planner/scheduler to compute which legs are contacted in the ground (Contact flag to know which legs are in stance)
        6) State machine to switch between MPC ground force control and Swing-leg control using PID*/

    // Logger to see the sensor data (Test)
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
        "IMU Gyro [x: %.3f, y: %.3f, z: %.3f] | Accel [x: %.3f, y: %.3f, z: %.3f]", 
        msg->imu_state.gyroscope[0],
        msg->imu_state.gyroscope[1],
        msg->imu_state.gyroscope[2],
        msg->imu_state.accelerometer[0],
        msg->imu_state.accelerometer[1],
        msg->imu_state.accelerometer[2]);

    // Current yaw
    double current_yaw = msg->imu_state.rpy[2]; 

    // jacobian calculation
    std::vector<Eigen::Matrix3f> leg_jacobians(4); 

    // r_i calculation
    std::vector<Eigen::Vector3d> foot_positions_world;
    Eigen::Matrix3d Rz = go2.getRotationMatrix(current_yaw);

    for(int i=0; i<4; i++){ // i = legs 
        // Joint angles vector for sending it to forward kinematics function
        Eigen::Vector3f joint_angles;
        joint_angles << msg->motor_state[i*3 + 0].q, // i*3 + 0 because we need first out of 3 joints (index 0) of the current leg (i)
                        msg->motor_state[i*3 + 1].q,  
                        msg->motor_state[i*3 + 2].q; 

        // Forward kinematics
        auto fk_result = common::forwards_kinematics(joint_angles, leg_sides[i]); 
        
        // r_i calc
        if (fk_result.has_value()) {
            Eigen::Vector3d d_fk = fk_result.value().cast<double>(); // Type cast to double from float
            Eigen::Vector3d r_body = hip_offset[i] + d_fk; 
            Eigen::Vector3d r_world = Rz * r_body; 
            foot_positions_world.push_back(r_world);

            // Store the leg jacobians
            leg_jacobians[i] = common::jacobian_matrix(joint_angles, leg_sides[i]);
        } 
        else {
            RCLCPP_WARN_ONCE(this->get_logger(), "Leg %d returned invalid kinematics on startup!.", i);
            leg_jacobians[i] = Eigen::Matrix3f::Identity(); // Safety to avoid crashes 
        } 
    }

    // Contact foot count 

    // Dynamics computations 
    go2.computeA(current_yaw);
    go2.computeB(foot_positions_world, current_yaw);

    go2.discretize(dt); 

    // Initialize Controller Command
    unitree_go::msg::LowCmd cmd_msg;

    // Controller lines

    // PD trial from plugin
    for(int i=0;i<12;i++)
    {
        cmd_msg.motor_cmd[i].q  = q_ref[i];
        cmd_msg.motor_cmd[i].dq = 0.0;
        cmd_msg.motor_cmd[i].kp = kp_;
        cmd_msg.motor_cmd[i].kd = kd_;
        cmd_msg.motor_cmd[i].tau = 0.0;
    }
    // TODO: I dont know if this is useful: Clean up the unused spots in the array (12-20) to prevent garbage values
    for (int i = 12; i < 20; ++i) {
        cmd_msg.motor_cmd[i].q = 0.0;
        cmd_msg.motor_cmd[i].dq = 0.0;
        cmd_msg.motor_cmd[i].kp = 0.0;
        cmd_msg.motor_cmd[i].kd = 0.0;
        cmd_msg.motor_cmd[i].tau = 0.0;
    }

    publisher_->publish(cmd_msg);
}