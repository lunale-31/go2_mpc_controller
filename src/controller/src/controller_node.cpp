#include "controller/controller_node.h"


HighLevelControl::HighLevelControl() : rclcpp::Node("high_level_control"){
    
    cmd_pub_ = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd",10);
    low_state_sub_ = this->create_subscription<unitree_go::msg::LowState>("/lowstate", 10, 
                     std::bind(&HighLevelControl::lowStateCallback, this, std::placeholders::_1));
    est_sub_ = this->create_subscription<go2_interfaces::msg::EstimatedState>("/estimated_state", 10, 
                     std::bind(&HighLevelControl::estimatedStateCallback, this, std::placeholders::_1));
    mpc_command_sub_ = this->create_subscription<go2_interfaces::msg::MpcCommand>("/mpc_command", 10, 
                     std::bind(&HighLevelControl::mpcCommandCallback, this, std::placeholders::_1));

    kf_initialize_pub_ = this->create_publisher<std_msgs::msg::Bool>("/kf_initialize", 10);     
    mpc_initialize_pub_ = this->create_publisher<std_msgs::msg::Bool>("/mpc_initialize", 10); 

    loadParams(); 

    // PD Controller
    control_timer_ = this->create_wall_timer(std::chrono::duration<double>(joint_dt_), std::bind(&HighLevelControl::controlLoop, this));
    RCLCPP_INFO(this->get_logger(), "Control supervisor started");
}

void HighLevelControl::loadParams(){
    /* Declare parameter is called only when there is no YAML file is provided for loading the parameters. */
    this->declare_parameter<double>("joint_dt", 0.001); 
    this->get_parameter("joint_dt", joint_dt_); 

    this->declare_parameter<double>("kf_warmup_time", 4.0);
    this->get_parameter<double>("kf_warmup_time", kf_warmup_time_);

    // Reference joint positions
    this->declare_parameter<std::vector<double>>("q_ref", std::vector<double>());
    this->get_parameter("q_ref", q_ref); 

    // Gains
    this->declare_parameter<double>("gains.kp", 10.0);
    this->declare_parameter<double>("gains.kd", 1.0);

    this->get_parameter("gains.kp", kp_);
    this->get_parameter("gains.kd", kd_);
    
    //  Crouch transition time
    this->declare_parameter<double>("crouch_transition_time", 3.0);
    this->get_parameter("crouch_transition_time",crouch_transition_time_);
}

void HighLevelControl::lowStateCallback(unitree_go::msg::LowState::SharedPtr msg){
    latest_low_state_ = msg; 
    low_state_received_ = true; 
}

void HighLevelControl::estimatedStateCallback(go2_interfaces::msg::EstimatedState::SharedPtr msg){
    latest_est_state_ = msg;
    estimated_state_received_ = true; 
}

void HighLevelControl::mpcCommandCallback(go2_interfaces::msg::MpcCommand::SharedPtr msg){
    latest_mpc_cmd_ = msg;
    mpc_command_received_ = true; 
}

void HighLevelControl::controlLoop(){
    // Controller command
    unitree_go::msg::LowCmd command;

    // State executions
    switch(m_s){
        case ControlState::BOOT: 
        {
            if(low_state_received_){
                boot_complete_=true; 

                changeState(ControlState::IDLE);
            }
            break;
        }

        case ControlState::IDLE:
        {
            if(sensorsValid()){
                crouch_initialized_ = false;
                crouch_complete_ = false; 
                
                idle_complete_ = true;

                changeState(ControlState::SMOOTH_RAISE);
            }
            break;
        }

        case ControlState::SMOOTH_RAISE:
        {
            runSmoothRaise(command);

            if(crouch_initialized_ && crouch_complete_){                
                std_msgs::msg::Bool init_msg;
                init_msg.data = true;
                
                kf_initialize_pub_->publish(init_msg);

                kf_start_time = this->now();
                
                smooth_raise_complete_ = true; 
                changeState(ControlState::KF_INITIALIZE);
            }
            break;
        }

        case ControlState::KF_INITIALIZE:
        {
            runSmoothRaise(command);

            if (estimated_state_received_ &&
                latest_est_state_ &&
                latest_est_state_->initialized &&
                latest_est_state_->valid){
                    
                    auto elapsed_time = (this->now() - kf_start_time.value()).seconds(); 
                    
                    if(elapsed_time >= kf_warmup_time_){
                        mpc_start_time = this->now(); 
                        kf_complete_ = true;
                        changeState(ControlState::MPC_INITIALIZE);
                    }
            }

            break;
        }

    case ControlState::MPC_INITIALIZE:
    {
        if (boot_complete_ &&
            idle_complete_ &&
            smooth_raise_complete_ &&
            kf_complete_)
        {
            // Publish initialization only once.
            if (!mpc_initialize_) {
                std_msgs::msg::Bool mpc_init_msg;
                mpc_init_msg.data = true;
                mpc_initialize_pub_->publish(mpc_init_msg);

                mpc_initialize_ = true;
            }

            if (latest_mpc_cmd_ &&
                latest_est_state_ &&
                latest_low_state_)
            {
                std::array<double, 12> test_force{};

                const double force_per_leg =
                    15.206 * 9.81 / 4.0;

                for (int leg = 0; leg < 4; leg++) {
                    test_force[leg * 3 + 0] = 0.0;
                    test_force[leg * 3 + 1] = 0.0;
                    test_force[leg * 3 + 2] = force_per_leg;
                }

                // runMpcCommand(
                //     command,
                //     test_force,
                //     latest_est_state_);
                runMpcCommand(
                    command,
                    latest_mpc_cmd_->ground_force,
                    latest_est_state_);
            }
            else {
                // Continue holding the crouch until the first MPC result arrives.
                runSmoothRaise(command);
            }
        }

        break;
    }
        
        default:{
            runSmoothRaise(command);
            break;
        }
    }
}

bool HighLevelControl::sensorsValid(){
    if (q_ref.size() != 12) {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "q_ref must contain 12 values.");

        return false;
    }

    if (!latest_low_state_) {
        return false;
    }

    for (int i = 0; i < 12; ++i) {
        if (!std::isfinite(latest_low_state_->motor_state[i].q) ||
            !std::isfinite(latest_low_state_->motor_state[i].dq))
        {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Invalid joint data detected at motor %d.",
                i);

            return false;
        }
    }
    return true; 
}

void HighLevelControl::runSmoothRaise(unitree_go::msg::LowCmd &cmd_msg){
    /* Smooth crouch controller */
    // Prevent q_ref[i] from accessing invalid memory.
    if (q_ref.size() != 12) {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "q_ref must contain exactly 12 joint positions. Current size: %zu",
            q_ref.size());

        return;
    }

    // Record the actual joint posture only once, when control starts.
    if (!crouch_initialized_) {
        for (int i = 0; i < 12; ++i) {
            q_start_[i] = latest_low_state_->motor_state[i].q;
        }

        crouch_start_time_ = this->now();
        crouch_initialized_ = true;

        RCLCPP_INFO(
            this->get_logger(),
            "Starting %.2f-second smooth crouch transition.",
            crouch_transition_time_);
    }

    const rclcpp::Time now = this->now();

    // Elapsed time since crouch starting time
    const double elapsed_time = (now - crouch_start_time_.value()).seconds();

    // Normalized transition time, 0 at start and 1 at the end. 
    const double phase = std::clamp(
        elapsed_time / crouch_transition_time_,
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

    if (phase < 1.0 && crouch_transition_time_ > 0.0) {
        const double ds_dphase = 30.0 * phase_2 - 60.0 * phase_3 + 30.0 * phase_4;
        s_dot = ds_dphase / crouch_transition_time_;
    }

    for (int i = 0; i < 12; ++i) {

        // Interpolation (Qstart at s=0 and Qref at s=1)
        const double q_command = q_start_[i] + s * (q_ref[i] - q_start_[i]);
        const double dq_command = s_dot * (q_ref[i] - q_start_[i]);

        cmd_msg.motor_cmd[i].q = q_command;
        cmd_msg.motor_cmd[i].dq = dq_command;
        cmd_msg.motor_cmd[i].kp = kp_;
        cmd_msg.motor_cmd[i].kd = kd_;
        cmd_msg.motor_cmd[i].tau = 0.0;
    }

    // Report transition completion only once.
    if (phase >= 1.0 && !crouch_complete_) {
        crouch_complete_ = true;

        RCLCPP_INFO(
            this->get_logger(),
            "Smooth crouch transition completed.");
    }

    // TODO: I dont know if this is useful: Clean up the unused spots in the array (12-20) to prevent garbage values
    for (int i = 12; i < 20; ++i) {
        cmd_msg.motor_cmd[i].q = 0.0;
        cmd_msg.motor_cmd[i].dq = 0.0;
        cmd_msg.motor_cmd[i].kp = 0.0;
        cmd_msg.motor_cmd[i].kd = 0.0;
        cmd_msg.motor_cmd[i].tau = 0.0;
    }
    cmd_pub_->publish(cmd_msg);
}

void HighLevelControl::runMpcCommand(unitree_go::msg::LowCmd &cmd, std::array<double,12> ground_force,  const go2_interfaces::msg::EstimatedState::SharedPtr msg){

    if (!msg || !latest_low_state_) {
        return;
    }

    // Capture the joint posture once when MPC begins
    if (!mpc_posture_initialized_) {
        for (int motor = 0; motor < 12; motor++) {
            mpc_q_hold_[motor] =
                latest_low_state_->motor_state[motor].q;
        }

        mpc_posture_initialized_ = true;

        RCLCPP_INFO(
            this->get_logger(),
            "Captured MPC stance posture.");
    }

    // Low-gain posture stabilization + MPC feedforward torque
    for (int motor = 0; motor < 12; motor++) {
        cmd.motor_cmd[motor].q =
            mpc_q_hold_[motor];

        cmd.motor_cmd[motor].dq = 0.0;

        cmd.motor_cmd[motor].kp = 30.0;
        cmd.motor_cmd[motor].kd = 1.5;

        cmd.motor_cmd[motor].tau = 0.0;
    }
    
    // Jacobians
    std::vector<Eigen::Matrix3d> leg_jacobians_world(4);

    for (int leg = 0; leg < 4; leg++) {
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                const int index =
                    leg * 9 + row * 3 + col;

                leg_jacobians_world[leg](row, col) = msg->leg_jacobians_world[index];
            }
        }
    }

    // MPC force command to torque
    for(int leg=0; leg<4; leg++){
        Eigen::Vector3d force_world;
        
        force_world << ground_force[leg*3 + 0],
                       ground_force[leg*3 + 1],
                       ground_force[leg*3 + 2];

        const Eigen::Vector3d tau_leg = -leg_jacobians_world[leg].transpose() * force_world;

        // Copy the three leg torques into LowCmd
        for (int joint = 0; joint < 3; joint++) {
            const int motor_index = leg * 3 + joint;

            cmd.motor_cmd[motor_index].tau =
                tau_leg(joint);
        }
    }
    cmd_pub_->publish(cmd);
}

void HighLevelControl::runEmergencyStop(unitree_go::msg::LowCmd &cmd){

}

void HighLevelControl::changeState(ControlState new_state)
{
    // Avoids repeatedly entering and printing the same state
    if (m_s == new_state) {
        return;
    }

    const ControlState old_state = m_s;
    m_s = new_state;

    // Print a useful message when entering each state
    switch (m_s) {
        case ControlState::IDLE:
            RCLCPP_INFO(
                this->get_logger(),
                "BOOT successful. IDLING and checking sensors/references.");
            break;

        case ControlState::SMOOTH_RAISE:
            RCLCPP_INFO(
                this->get_logger(),
                "Checks successful. Starting PD smooth raise.");
            break;

        case ControlState::KF_INITIALIZE:
            RCLCPP_INFO(
                this->get_logger(),
                "PD smooth raise completed. Initializing Kalman filter.");
            break;

        case ControlState::MPC_INITIALIZE:
            RCLCPP_INFO(
                this->get_logger(),
                "KF warmup completed. Initializing MPC.");
            break;

        case ControlState::MPC_RUNNING:
            RCLCPP_INFO(
                this->get_logger(),
                "MPC initialized. MPC control is running.");
            break;

        case ControlState::STOP:
            RCLCPP_WARN(
                this->get_logger(),
                "Normal stop requested.");
            break;

        case ControlState::EMERGENCY_STOP:
            RCLCPP_ERROR(
                this->get_logger(),
                "EMERGENCY STOP ACTIVE.");
            break;

        default:
            break;
    }
}

ControlState HighLevelControl::getCurrentState() const{
    return m_s; 
}

std::string HighLevelControl::getStateName() const{
    switch(m_s){
        case ControlState::BOOT: return "BOOT"; 
        case ControlState::EMERGENCY_STOP: return "EMERGENCY_STOP";
        case ControlState::SMOOTH_RAISE: return "SMOOTH_RAISE"; 
        case ControlState::KF_INITIALIZE: return "KF_INITALIZE"; 
        case ControlState::MPC_INITIALIZE: return "MPC_INITALIZE";
        case ControlState::MPC_RUNNING: return "MPC_RUNNING";
        default: return "Unknown";
    }
}