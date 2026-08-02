#include "controller/mpc_node.h"

MPCNode::MPCNode() : rclcpp::Node("mpc_node"){
    x_sub_ = this->create_subscription<go2_interfaces::msg::EstimatedState>("/estimated_state", 10, 
                    std::bind(&MPCNode::stateCallback, this, std::placeholders::_1));

    mpc_initialize_sub = this->create_subscription<std_msgs::msg::Bool>("/mpc_initialize", 10,
                    std::bind(&MPCNode::mpcInitializeCallback, this, std::placeholders::_1));

    mpc_cmd_pub_ = this->create_publisher<go2_interfaces::msg::MpcCommand>("/mpc_command", 10);

    loadParams(); 

    force_control_timer_ = this->create_wall_timer(std::chrono::duration<double>(mpc_dt_),
                                std::bind(&MPCNode::mpcControlLoop, this));
}   

void MPCNode::loadParams(){
    this->declare_parameter<double>("mpc_dt", 0.05);
    this->get_parameter("mpc_dt", mpc_dt_);

    this->declare_parameter<double>("stand_transition_time", 5.0);
    this->get_parameter("stand_transition_time", stand_transition_time_);

    this->declare_parameter<double>("z_ref", 0.3);
    this->get_parameter("z_ref", z_ref);

    RCLCPP_INFO(this->get_logger(), "Loaded mpc_dt: %f", mpc_dt_);
}

void MPCNode::stateCallback(go2_interfaces::msg::EstimatedState::SharedPtr msg){
    latest_state_msg = msg; 
}

void MPCNode::mpcInitializeCallback(std_msgs::msg::Bool::SharedPtr msg){
    if(msg->data && !mpc_initialize_){
        mpc_initialize_request = true;
    }
}

std::pair<double,double> MPCNode::computeSmoothHeight(Eigen::Matrix<double, 13,1> x_curr){

    if(!stand_initialized_){
        z_start = x_curr[5];

        stand_start_time_ = this->now(); 
        stand_initialized_ = true;

        RCLCPP_INFO(
            this->get_logger(),
            "Starting %.2f-second smooth stand transition.",
            stand_transition_time_);
    }

    const rclcpp::Time now = this->now();

    // Elapsed time since crouch starting time
    const double elapsed_time = (now - stand_start_time_.value()).seconds();

    // Normalized transition time (phase), 0 at start and 1 at the end. 
    const double phase = std::clamp(
        elapsed_time / stand_transition_time_,
        0.0,
        1.0);

    // Powers of phase
    const double phase_2 = phase * phase;
    const double phase_3 = phase_2 * phase;
    const double phase_4 = phase_3 * phase;
    const double phase_5 = phase_4 * phase;

    // Quintic interpolation, coefficients are obtained using => s(0) = 0, s(1) = 1, velocity and acceleration are zero at both ends.
    // Now S is a smoothly increasing function from 0 to 1, with the help of phases/time
    const double s = 10.0 * phase_3 - 15.0 * phase_4 +  6.0 * phase_5;

    // Derivative ds/dt
    double s_dot = 0.0;

    if (phase < 1.0 && stand_transition_time_ > 0.0) {
        const double ds_dphase = 30.0 * phase_2 - 60.0 * phase_3 + 30.0 * phase_4;
        s_dot = ds_dphase / stand_transition_time_;
    }

    const double z_command = z_start + s * (z_ref - z_start);
    const double dz_command = s_dot * (z_ref - z_start);

    return {z_command, dz_command}; 
}

void MPCNode::mpcControlLoop(){
    go2_interfaces::msg::MpcCommand mpc_cmd; 
    if (!latest_state_msg) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Waiting for /estimated_state...");
        return;
    }

    if (!mpc_initialize_request) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Waiting for /mpc_initialize...");
        return;
    }

    /* Read latest state */
    double current_yaw = latest_state_msg->rpy[2];
    // r_foot in base frame wrt world frame
    std::vector<Eigen::Vector3d> foot_pos_world(4);

    // Jacobians
    std::vector<Eigen::Matrix3d> leg_jacobians_world(4);

    if(!mpc_initialize_){
        if(!mpc_initialize_request){
            return;
        }
        for(int leg = 0; leg<4; leg++){
            for(int i=0; i<3; i++){
                foot_pos_world[leg][i] = latest_state_msg->foot_positions_world[leg*3 + i];
            }
        }
        for (int leg = 0; leg < 4; leg++) {
            for (int row = 0; row < 3; row++) {
                for (int col = 0; col < 3; col++) {
                    const int index =
                        leg * 9 + row * 3 + col;

                    leg_jacobians_world[leg](row, col) = latest_state_msg->leg_jacobians_world[index];
                }
            }
        }
        
        // Initialization code for MPC
        mpc_initialize_ = true; 

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
        "FOOT POSITIONS DEBUG: %.3f", foot_pos_world[0][2]);
    }
    /* Reference generator */
    Eigen::Matrix<double, 13,1> x_curr; 
    x_curr <<   latest_state_msg->rpy[0],
                latest_state_msg->rpy[1],
                latest_state_msg->rpy[2],
                latest_state_msg->position[0],
                latest_state_msg->position[1],
                latest_state_msg->position[2],
                latest_state_msg->omega_world[0],
                latest_state_msg->omega_world[1],
                latest_state_msg->omega_world[2],
                latest_state_msg->velocity[0],
                latest_state_msg->velocity[1],
                latest_state_msg->velocity[2],
                1.0; 

    const auto [z_traj, zdot_traj] = computeSmoothHeight(x_curr);
    
    x_ref_ <<   0.0,        // desired roll
                0.0,        // desired pitch
                x_curr(2),  // desired yaw, or fixed initial yaw
                x_curr(3),  // desired x position
                x_curr(4),  // desired y position
                z_traj,  // desired body height
                0.0,        // desired wx
                0.0,        // desired wy
                0.0,        // desired wz
                0.0,        // desired vx
                0.0,        // desired vy
                zdot_traj, // desired vertical velocity
                1.0;        // gravity state

    /*Dynamics computations*/  
    go2.computeA(current_yaw);
    go2.computeB(foot_pos_world, current_yaw);
    go2.discretize(mpc_dt_);

    auto A_d = go2.getA();
    auto B_d = go2.getB();
    auto C_d = go2.getC();

    /* Matrix building */

    /* Solve */
    mpc_cmd.ground_force[0] = 0.0;
    mpc_cmd.ground_force[1] = 0.0;
    mpc_cmd.ground_force[2] = 0.0;
    mpc_cmd.ground_force[3] = 0.0;
    mpc_cmd.ground_force[4] = 0.0;
    mpc_cmd.ground_force[5] = 0.0;
    mpc_cmd.ground_force[6] = 0.0;
    mpc_cmd.ground_force[7] = 0.0;
    mpc_cmd.ground_force[8] = 0.0;
    mpc_cmd.ground_force[9] = 0.0;
    mpc_cmd.ground_force[10] = 0.0;
    mpc_cmd.ground_force[11] = 0.0;

    mpc_cmd_pub_->publish(mpc_cmd);
}