#include "controller/state_estimator_node.h"
#include "controller/foot_index.h"

StateEstimatorNode::StateEstimatorNode() : rclcpp::Node("state_estimator")
{
    publisher_ = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd",10);
    subscriber_ = this->create_subscription<unitree_go::msg::LowState>("/lowstate", 10, std::bind(&StateEstimatorNode::controller_callback, this, std::placeholders::_1));
    
    est_debug_ = this->create_publisher<go2_interfaces::msg::EstimatorDebug>("/estimator", 10);
    est_state_pub = this->create_publisher<go2_interfaces::msg::EstimatedState>("/estimated_state", 10);

    kf_initialize_sub = this->create_subscription<std_msgs::msg::Bool>("/kf_initialize",10, std::bind(&StateEstimatorNode::initialize_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Controller active! Waiting for simulation ticks...");

    loadParams(); 

    filter_ = std::make_unique<KalmanFilter>(obs_dt); // dt = 0.002, because lowstate publishes at 500hz

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

    // Gait planner. 
    // TODO: Later, build a gait planner that changes the contacts for every timestep. 
    planned_contacts = std::vector<bool>(4, true);
}

void StateEstimatorNode::loadParams(){
    /* Declare parameter is called only when there is no YAML file is provided for loading the parameters. */
    // Get sampling time
    this->declare_parameter<double>("obs_dt", 0.002); 
    this->get_parameter("obs_dt", obs_dt); 

    // Sanity check
    RCLCPP_INFO(this->get_logger(), "Loaded obs_dt: %f", obs_dt);
}

void StateEstimatorNode::initialize_callback(const std_msgs::msg::Bool::SharedPtr msg){
    if(msg->data && !kf_initialized){
        kf_init_requested = true;

        RCLCPP_INFO(this->get_logger(), "KF initialization requested");
    }
}

void StateEstimatorNode::controller_callback(const unitree_go::msg::LowState::SharedPtr msg){

    /*Kinematics related calculation*/
    // Current RPY
    double current_yaw = msg->imu_state.rpy[2]; 
    double current_pitch = msg->imu_state.rpy[1];
    double current_roll = msg->imu_state.rpy[0];

    // jacobian calculation
    std::vector<Eigen::Matrix3f> leg_jacobians(4); 

    // r_body calculation (position vector from body to foot wrt to the body frame)
    std::vector<Eigen::Vector3d> foot_positions_body(4); 

    // r_i calculation 
    std::vector<Eigen::Vector3d> foot_positions_world(4); //position vector from body to foot wrt world frame (small angle approximation - MPC)
    std::vector<Eigen::Vector3d> est_foot_positions_world(4);//position vector from body to foot wrt world frame (No approximations - KF)

    // r_i dot calculation variable
    std::vector<Eigen::Vector3d> est_foot_velocities_world(4); 

    Eigen::Matrix3d Rz = go2.getRotationMatrix(current_yaw);
    Eigen::Matrix3d Ry = go2.getRotationMatrixPitch(current_pitch);
    Eigen::Matrix3d Rx = go2.getRotationMatrixRoll(current_roll);

    // Computes the transformation matrix for changing body frame to world frame (Rotation matrix in Euler Angle representation)
    Eigen::Matrix3d R_w = Rz * Ry * Rx;  

    // r_i calculation 
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
            if(planned_contacts[i]){
                Eigen::Vector3d d_fk = fk_result.value().cast<double>(); // Type cast to double from float
                Eigen::Vector3d r_body = hip_offset[i] + d_fk; 
                foot_positions_body[i] = r_body;

                Eigen::Vector3d r_world = Rz * r_body; 
                foot_positions_world[i] = r_world;
                
                // TODO: Build filter functions that accounts for foot contacts. If no contact, need to rebuild C and R2 matrix.  
                Eigen::Vector3d r_fullworld = R_w * r_body; 
                est_foot_positions_world[i] = r_fullworld;
            }
            // Store the leg jacobians
            leg_jacobians[i] = common::jacobian_matrix(joint_angles, leg_sides[i]);
        }
        else {
            // TODO: compute fk and r_i based on contacts here. 
            RCLCPP_WARN_ONCE(this->get_logger(), "Leg %d returned invalid kinematics on startup!.", i);
            leg_jacobians[i] = Eigen::Matrix3f::Identity(); // Safety to avoid crashes 
        }
    }

    // r_i dot calculation
    std::vector<Eigen::Vector3d> joint_dq;

    for(int leg = 0; leg<4; leg++){
        Eigen::Vector3d motor_dq; 
        motor_dq << msg->motor_state[leg*3 + 0].dq,
                    msg->motor_state[leg*3 + 1].dq, 
                    msg->motor_state[leg*3 + 2].dq; 
        
        joint_dq.push_back(motor_dq);
    }

    Eigen::Vector3d joint_omegas_body; 
    joint_omegas_body << msg->imu_state.gyroscope[0],
                         msg->imu_state.gyroscope[1],
                         msg->imu_state.gyroscope[2];

    for(int j=0; j<4; j++){
        est_foot_velocities_world[j] = R_w * (go2.getSkewSymMatrix(joint_omegas_body) * foot_positions_body[j] + 
                                              leg_jacobians[j].cast<double>() * joint_dq[j]);
    }
    
    // /*Dynamics computations*/  
    // go2.computeA(current_yaw);
    // go2.computeB(foot_positions_world, current_yaw);
    // go2.discretize(obs_dt); 
    
    /*State Estimator*/
    Eigen::Vector3d omega_body{msg->imu_state.gyroscope[0],
                            msg->imu_state.gyroscope[1],
                            msg->imu_state.gyroscope[2]}; 

    Eigen::Vector3d omega_world = Rz * omega_body;

    Eigen::Vector3d imu_accln; 
    Eigen::Vector3d accln_world; 
    Eigen::Vector3d gravity(0.0,0.0, -9.81); 
    imu_accln << msg->imu_state.accelerometer[0],
                 msg->imu_state.accelerometer[1],
                 msg->imu_state.accelerometer[2];
    
    accln_world = R_w*imu_accln + gravity; 
    
    if (!kf_initialized) {
        if (!kf_init_requested) {
            return;
        }

        const bool success = filter_->initializeKF(est_foot_positions_world);

        if (!success) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Waiting for valid kinematics before KF initialization.");

            return;
        }

        kf_initialized = true;
        kf_init_requested = false;

        RCLCPP_INFO(
            this->get_logger(),
            "Kalman filter initialized.");
    }

    // Prediction state
    filter_->predictionStep(accln_world);
    Eigen::Matrix<double, 18,1> x_hat_predicted = filter_->getXhat();

    // Filtering state
    filter_->filteringStep(est_foot_positions_world, est_foot_velocities_world);
    Eigen::Matrix<double, 18,1> x_hat = filter_->getXhat(); 

    // Reading values 
    Eigen::Matrix<double, 28, 1> y_measurement = filter_->getMeasurement();
    Eigen::Matrix<double, 28, 1> y_predicted = filter_->getMeasurementPrediction();
    Eigen::Matrix<double, 28, 1> innovation = filter_->getInnovation();
    Eigen::Matrix<double, 28, 1> residual = filter_->getResidualError(); 
    
    Eigen::Matrix<double, 18,1> p_diag_filtered = filter_->getStateCovarianceDiag();
    Eigen::Matrix<double, 28,1> s_diagonal = filter_->getInnovationCovarianceDiag();

    double nis = filter_->getNIS(); 

    // Estimated state publisher
    go2_interfaces::msg::EstimatedState est_state_msg; 

    est_state_msg.header.stamp = this->get_clock()->now();

    est_state_msg.rpy[0] = current_roll;
    est_state_msg.rpy[1] = current_pitch;
    est_state_msg.rpy[2] = current_yaw;

    for(int i = 0; i<3; i++){
        est_state_msg.position[i] = x_hat[i]; 
        est_state_msg.velocity[i] = x_hat[i+3];
        est_state_msg.omega_world[i] = omega_world[i];  
    }

    for(int leg = 0; leg<4; leg++){ // Legs
        est_state_msg.contact[leg] = planned_contacts[leg]; 

        for(int i = 0; i<3; i++){ // Hip, thigh, calf
            est_state_msg.foot_positions_world[leg*3 + i] = foot_positions_world[leg][i]; 
        } 
    }

    std::vector<Eigen::Matrix3d> leg_jacobians_world(4);

    for (int leg = 0; leg < 4; leg++) {
        leg_jacobians_world[leg] =
            R_w * leg_jacobians[leg].cast<double>();
    }

    for (int leg = 0; leg < 4; leg++) {
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                const int index =
                    leg * 9 + row * 3 + col;

                est_state_msg.leg_jacobians_world[index] =
                    leg_jacobians_world[leg](row, col);
            }
        }
    }
    
    est_state_msg.nis = nis; 

    const bool estimator_valid =
        x_hat.allFinite() &&
        p_diag_filtered.allFinite() &&
        std::isfinite(nis);

    est_state_msg.initialized = kf_initialized;
    est_state_msg.valid = estimator_valid;

    est_state_pub->publish(est_state_msg);

    // Debug message
    go2_interfaces::msg::EstimatorDebug est_debug_msg;

    est_debug_msg.header.stamp = this->get_clock()->now();
    est_debug_msg.dt = obs_dt; 
    
    est_debug_msg.rpy[0] = current_roll;
    est_debug_msg.rpy[1] = current_pitch;
    est_debug_msg.rpy[2] = current_yaw;
     
    for(int i=0; i<3; i++){
        est_debug_msg.imu_accel_body[i] = imu_accln[i];
        est_debug_msg.accel_world[i] = accln_world[i]; 
    }

    for(int i=0; i<18; i++){
        est_debug_msg.x_predicted[i] = x_hat_predicted[i];
        est_debug_msg.x_filtered[i] = x_hat[i]; 
    }

    for(int i=0; i<28; i++){
        est_debug_msg.measurement[i] = y_measurement[i];
        est_debug_msg.measurement_predicted[i] = y_predicted[i]; 
        est_debug_msg.innovation[i] = innovation[i]; 
    }

    for(int i=0; i<18; i++){
        est_debug_msg.state_covariance_diagonal[i] = p_diag_filtered[i]; 
    }

    for(int i=0; i<28; i++){
        est_debug_msg.innovation_covariance_diagonal[i] = s_diagonal[i];
    }

    est_debug_msg.nis = nis; 
    
    est_debug_->publish(est_debug_msg);
    
    /* TODO: Switch to MPC thread */
    // Creating the current state vector for MPC
    Eigen::Matrix<double, 13,1> x_curr; 
    x_curr <<   current_roll,
                current_pitch,
                current_yaw,
                x_hat[0],
                x_hat[1],
                x_hat[2],
                omega_world[0],
                omega_world[1],
                omega_world[2],
                x_hat[3],
                x_hat[4],
                x_hat[5],
                1.0; 
        
    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
    //     "x_curr | RPY: [%.3f, %.3f, %.3f] | Pos: [%.3f, %.3f, %.3f] | Omg: [%.3f, %.3f, %.3f] | Vel: [%.3f, %.3f, %.3f] | g: %.1f",
    //     x_curr(0), x_curr(1), x_curr(2),   // Roll, Pitch, Yaw
    //     x_curr(3), x_curr(4), x_curr(5),   // X, Y, Z Position
    //     x_curr(6), x_curr(7), x_curr(8),   // X, Y, Z Angular Velocity
    //     x_curr(9), x_curr(10), x_curr(11), // X, Y, Z Linear Velocity
    //     x_curr(12));                       // Gravity constant

}