#include "controller/mpc_node.h"

MPCNode::MPCNode() : rclcpp::Node("mpc_node"){
    x_sub_ = this->create_subscription<go2_interfaces::msg::EstimatedState>("/estimated_state", 10, 
                    std::bind(&MPCNode::stateCallback, this, std::placeholders::_1));

    mpc_initialize_sub = this->create_subscription<std_msgs::msg::Bool>("/mpc_initialize", 10,
                    std::bind(&MPCNode::mpcInitializeCallback, this, std::placeholders::_1));

    mpc_cmd_pub_ = this->create_publisher<go2_interfaces::msg::MpcCommand>("/mpc_command", 10);

    loadParams(); 

    mpc_ = std::make_unique<MPC>(mpc_dt_);

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

    if(!mpc_initialize_){
        if(!mpc_initialize_request){
            return;
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

    if (!hold_reference_initialized_) {
        x_hold_ = x_curr(3);
        y_hold_ = x_curr(4);
        yaw_hold_ = x_curr(2);

        hold_reference_initialized_ = true;

        RCLCPP_INFO(
            this->get_logger(),
            "Holding reference: x=%.3f, y=%.3f, yaw=%.3f",
            x_hold_,
            y_hold_,
            yaw_hold_);
    }

    const auto [z_traj, zdot_traj] = computeSmoothHeight(x_curr);
    // if (!stand_initialized_) {
    //     z_start = x_curr(5);
    //     stand_initialized_ = true;
    // }

    // const double z_traj = z_start;
    // const double zdot_traj = 0.0;

    x_ref_ <<   0.0,        // desired roll
                0.0,        // desired pitch
                yaw_hold_,  // desired yaw, or fixed initial yaw
                x_hold_,  // desired x position
                y_hold_,  // desired y position
                z_traj,  // desired body height
                0.0,        // desired wx
                0.0,        // desired wy
                0.0,        // desired wz
                0.0,        // desired vx
                0.0,        // desired vy
                zdot_traj, // desired vertical velocity
                1.0;        // gravity state

    /* MPC Set reference */
    mpc_->setReference(x_ref_);

    /*Dynamics computations*/  
    mpc_->computeDynamics(go2, foot_pos_world, current_yaw);

    /* Cost building */
    mpc_->buildCost(x_curr);

    /* Constraints building */
    mpc_->buildConstraints(leg_jacobians_world);

    /* Solve */
    const bool solved = mpc_->solve();

    if(!solved){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "MPC Solver failed.");
        return;
    }

    const Eigen::Matrix<double, 12,1> u_opt = mpc_->getControlSignal();

    // Debug msg
    const double total_fx =
        u_opt(0) + u_opt(3) + u_opt(6) + u_opt(9);

    const double total_fy =
        u_opt(1) + u_opt(4) + u_opt(7) + u_opt(10);

    const double total_fz =
        u_opt(2) + u_opt(5) + u_opt(8) + u_opt(11);

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        500,
        "MPC force sum: Fx=%.2f, Fy=%.2f, Fz=%.2f | "
        "position=[%.3f %.3f %.3f] ref=[%.3f %.3f %.3f]",
        total_fx,
        total_fy,
        total_fz,
        x_curr(3),
        x_curr(4),
        x_curr(5),
        x_ref_(3),
        x_ref_(4),
        x_ref_(5));
        
    for(int i=0; i<12; i++){
        mpc_cmd.ground_force[i] = u_opt[i];
    }
    mpc_cmd_pub_->publish(mpc_cmd);
}