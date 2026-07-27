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

}

void MPCNode::stateCallback(go2_interfaces::msg::EstimatedState::SharedPtr msg){
    latest_state_msg = msg; 
}

void MPCNode::mpcInitializeCallback(std_msgs::msg::Bool::SharedPtr msg){
    if(msg->data && !mpc_initialize_){
        mpc_initialize_request = true;
    }
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

    double current_yaw = latest_state_msg->rpy[2];
    // r_foot in base frame wrt world frame
    std::vector<Eigen::Vector3d> foot_pos_world(4);

    if(!mpc_initialize_){
        if(!mpc_initialize_request){
            return;
        }
        for(int leg = 0; leg<4; leg++){
            for(int i=0; i<3; i++){
                foot_pos_world[leg][i] = latest_state_msg->foot_positions_world[leg*3 + i];
            }
        }

        // Initialization code for MPC

        mpc_initialize_ = true; 

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
        "FOOT POSITIONS DEBUG: %.3f", foot_pos_world[0][2]);
    }

    // /*Dynamics computations*/  
    go2.computeA(current_yaw);
    go2.computeB(foot_pos_world, current_yaw);
    go2.discretize(mpc_dt_);

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