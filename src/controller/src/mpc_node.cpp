#include "controller/mpc_node.h"

MPCNode::MPCNode() : rclcpp::Node("mpc_node"){
    x_sub_ = this->create_subscription<go2_interfaces::msg::EstimatedState>("/estimated_state", 10, 
                    std::bind(&MPCNode::stateCallback, this, std::placeholders::_1));

    lowstate_sub = this->create_subscription<unitree_go::msg::LowState>("/lowstate", 10, 
                    std::bind(&MPCNode::lowStateCallback, this, std::placeholders::_1));
                
    mpc_initialize_sub = this->create_subscription<std_msgs::msg::Bool>("/mpc_initialize", 10,
                    std::bind(&MPCNode::mpcInitializeCallback, this, std::placeholders::_1));

    mpc_cmd_pub_ = this->create_publisher<go2_interfaces::msg::MpcCommand>("/mpc_command", 10);

    loadParams(); 

    initializePinocchio();

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

void MPCNode::lowStateCallback(unitree_go::msg::LowState::SharedPtr msg){
    latest_low_state = msg; 
}

void MPCNode::mpcInitializeCallback(std_msgs::msg::Bool::SharedPtr msg){
    if(msg->data && !mpc_initialize_){
        mpc_initialize_request = true;
    }
}

std::pair<double,double> MPCNode::computeSmoothHeight(double current_base_height){

    if(!stand_initialized_){
        z_start = current_base_height;

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

    if (!latest_low_state) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Waiting for /lowstate...");
        return;
    }

    /* Read latest state */
    double current_yaw = latest_state_msg->rpy[2];

    // Jacobians
    std::vector<Eigen::Matrix3d> leg_jacobians_world(4);
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
    }

    // Constructing Rotation matrix to change from Base frame to World frame
    const double roll  = latest_state_msg->rpy[0];
    const double pitch = latest_state_msg->rpy[1];
    const double yaw   = latest_state_msg->rpy[2];

    Eigen::AngleAxisd Rx(roll,Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd Ry(pitch,Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd Rz(yaw,Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond quat = Rz * Ry * Rx;
    quat.normalize();

    Eigen::Matrix3d R_WB = quat.toRotationMatrix();

    // Pinocchio calculations
    pin_q_.setZero();

    // BASE POSITION expressed in world coordinates
    pin_q_[0] = latest_state_msg->position[0];
    pin_q_[1] = latest_state_msg->position[1];
    pin_q_[2] = latest_state_msg->position[2];

    // BASE QUATERNION
    // x y z w
    pin_q_[3] = quat.x();
    pin_q_[4] = quat.y();
    pin_q_[5] = quat.z();
    pin_q_[6] = quat.w();

    // Estimator velocity = base velocity relative to world velocity expressed in world frame
    Eigen::Vector3d v_world(
        latest_state_msg->velocity[0],
        latest_state_msg->velocity[1],
        latest_state_msg->velocity[2]);

    Eigen::Vector3d omega_world(
        latest_state_msg->omega_world[0],
        latest_state_msg->omega_world[1],
        latest_state_msg->omega_world[2]);
    
    // With this, velocity is expressed in base frame because pinocchio only accepts base frame velocities to its arguments. 
    Eigen::Vector3d v_body = R_WB.transpose() * v_world;
    Eigen::Vector3d omega_body = R_WB.transpose() * omega_world;

    pin_v_.setZero();

    pin_v_.segment<3>(0) = v_body;
    pin_v_.segment<3>(3) = omega_body;

    // Filling joints pos and vel
    for (int motor = 0; motor < 12; ++motor)
    {
        const pinocchio::JointIndex jid = pin_joint_ids_[motor];

        const int idx_q = pin_model_.joints[jid].idx_q();
        const int idx_v = pin_model_.joints[jid].idx_v();

        pin_q_[idx_q] = latest_low_state->motor_state[motor].q;
        pin_v_[idx_v] = latest_low_state->motor_state[motor].dq;
    }

    // COM calculation
    // Compute the current world pose and spatial velocity of every joint
    // from the robot configuration q and velocity v. Updates data.oMi (pose of joint i wrt world/origin)
    pinocchio::forwardKinematics(pin_model_,*pin_data_,pin_q_,pin_v_);

    // Centroidal Composite Rigid Body Algorithm. Computes the centroidal momentum matrix, 
    // composite rigid-body inertia (Ig) and centroidal momentum for the current q,v 
    pinocchio::ccrba(pin_model_,*pin_data_,pin_q_,pin_v_);

    // Computes full robot's COM (com and vcom)
    pinocchio::centerOfMass(pin_model_,*pin_data_,pin_q_,pin_v_);

    // Pinocchio computes a kinematic tree of joints first, this function update world poses of all 
    // robot frames from the current joint kinematics. updates data.oMf (pose of frame f wrt world/origin)
    pinocchio::updateFramePlacements(pin_model_,*pin_data_);

    const Eigen::Vector3d p_com = pin_data_->com[0];
    const Eigen::Vector3d v_com = pin_data_->vcom[0];

    // Rotational inertia about COM
    const Eigen::Matrix3d I_com_world = pin_data_->Ig.inertia();

    const double robot_mass = pin_data_->Ig.mass();
    
    // Calculating I_body to do small-angle assumption similar to the MIT stack 
    Eigen::Matrix3d I_body = R_WB.transpose() * I_com_world * R_WB;

    // COM to contact vector 
    std::vector<Eigen::Vector3d> foot_pos_com_world(4);

    for (int leg = 0; leg < 4; ++leg)
    {
        Eigen::Vector3d p_foot = pin_data_->oMf[pin_foot_frame_ids_[leg]].translation();

        // Flat-ground contact is 22 mm underneath.
        // p_foot.z() -= 0.022;

        foot_pos_com_world[leg] = p_foot - p_com;
    }

    /* Reference generator */
    Eigen::Matrix<double, 13,1> x_curr; 
    x_curr <<
        latest_state_msg->rpy[0],
        latest_state_msg->rpy[1],
        latest_state_msg->rpy[2],

        p_com.x(),
        p_com.y(),
        p_com.z(),

        latest_state_msg->omega_world[0],
        latest_state_msg->omega_world[1],
        latest_state_msg->omega_world[2],

        v_com.x(),
        v_com.y(),
        v_com.z(),

        1.0;

    if (!hold_reference_initialized_) {
        x_hold_ = x_curr(3);
        y_hold_ = x_curr(4);
        yaw_hold_ = x_curr(2);
        roll_hold_  = x_curr(0);
        pitch_hold_ = x_curr(1);

        hold_reference_initialized_ = true;

        RCLCPP_INFO(
            this->get_logger(),
            "Holding reference: x=%.3f, y=%.3f, yaw=%.3f",
            x_hold_,
            y_hold_,
            yaw_hold_);
    }

    // Base position obtained from KF
    Eigen::Vector3d p_base(
        latest_state_msg->position[0],
        latest_state_msg->position[1],
        latest_state_msg->position[2]);

    // Base to COM offset
    Eigen::Vector3d base_to_com_world = p_com - p_base;

    const auto [z_traj, zdot_traj] = computeSmoothHeight(p_base.z());
    const double z_com_ref = z_traj + base_to_com_world.z();

    const double stand_elapsed = (this->now() - stand_start_time_.value()).seconds();

    const bool steady_state = stand_elapsed >= stand_transition_time_;

    x_ref_ <<   roll_hold_,        // desired roll
                pitch_hold_,        // desired pitch
                yaw_hold_,  // desired yaw, or fixed initial yaw
                x_hold_,  // desired x position
                y_hold_,  // desired y position
                z_com_ref,  // desired body height
                0.0,        // desired wx
                0.0,        // desired wy
                0.0,        // desired wz
                0.0,        // desired vx
                0.0,        // desired vy
                zdot_traj, // desired vertical velocity
                1.0;        // gravity state
    
    // RCLCPP_INFO_THROTTLE(
    //     this->get_logger(),
    //     *this->get_clock(),
    //     1000,

    //     "PINO | mass=%.4f COM=[%.4f %.4f %.4f] "
    //     "Idiag=[%.4f %.4f %.4f]",

    //     robot_mass,

    //     p_com.x(),
    //     p_com.y(),
    //     p_com.z(),

    //     I_com_world(0,0),
    //     I_com_world(1,1),
    //     I_com_world(2,2));

    /* MPC Set reference */
    mpc_->setReference(x_ref_);

    const auto t0 = std::chrono::steady_clock::now();

    mpc_->computeDynamics(go2,foot_pos_com_world,current_yaw,I_body,robot_mass);

    const auto t1 = std::chrono::steady_clock::now();

    mpc_->buildCost(x_curr);

    const auto t2 = std::chrono::steady_clock::now();

    mpc_->buildConstraints(
        leg_jacobians_world
    );

    const auto t3 = std::chrono::steady_clock::now();

    const bool solved = mpc_->solve();

    const auto t4 = std::chrono::steady_clock::now();

    /* MPC timing debug */
    const double dynamics_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    const double cost_ms =std::chrono::duration<double, std::milli>(t2 - t1).count();

    const double constraints_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();

    const double solve_ms = std::chrono::duration<double, std::milli>(t4 - t3).count();

    const double total_ms = std::chrono::duration<double, std::milli>(t4 - t0).count();

    // RCLCPP_INFO_THROTTLE(
    //     this->get_logger(),
    //     *this->get_clock(),
    //     500,
    //     "MPC TIMING: dyn=%.2f | cost=%.2f | "
    //     "constraints=%.2f | solve=%.2f | TOTAL=%.2f ms",
    //     dynamics_ms,
    //     cost_ms,
    //     constraints_ms,
    //     solve_ms,
    //     total_ms
    // );

    if(!solved){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "MPC Solver failed.");
        return;
    }

    // Validation
    validation_.updateConstraints(
        mpc_->getConstraintMatrix(),
        mpc_->getLowerBounds(),
        mpc_->getUpperBounds(),
        mpc_->getFullSolution()
    );

    const Eigen::Matrix<double, 12,1> u_opt = mpc_->getControlSignal();

    Eigen::Vector3d total_moment =
        Eigen::Vector3d::Zero();

    for (int leg = 0; leg < 4; ++leg) {

        Eigen::Vector3d force =
            u_opt.segment<3>(leg * 3);

        total_moment +=
            foot_pos_com_world[leg].cross(force);
    }

    // RCLCPP_INFO_THROTTLE(
    //     this->get_logger(),
    //     *this->get_clock(),
    //     1000,

    //     "PITCH DEBUG | "
    //     "pitch=%.2f deg | "
    //     "wy=%.3f rad/s | "
    //     "My=%.3f Nm | "
    //     "Fz=[%.1f %.1f %.1f %.1f] | "
    //     "rx=[%.3f %.3f %.3f %.3f]",

    //     x_curr(1) * 180.0 / M_PI,
    //     x_curr(7),
    //     total_moment.y(),

    //     u_opt(2),
    //     u_opt(5),
    //     u_opt(8),
    //     u_opt(11),

    //     foot_pos_com_world[0].x(),
    //     foot_pos_com_world[1].x(),
    //     foot_pos_com_world[2].x(),
    //     foot_pos_com_world[3].x()
    // );

    validation_.updateGRF(u_opt);

    const auto metrics =
        validation_.getMPCSummary();

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,

        "MPC VALIDATION | "
        "load=[%.1f %.1f %.1f %.1f]%% | "
        "imbalance=%.4f | "
        "viol=%zu/%zu | "
        "maxViol=%.5f",

        metrics.mean_load_fraction(0) * 100.0,
        metrics.mean_load_fraction(1) * 100.0,
        metrics.mean_load_fraction(2) * 100.0,
        metrics.mean_load_fraction(3) * 100.0,

        metrics.grf_load_imbalance_rms,

        metrics.constraint_violations,
        metrics.constraint_rows_checked,

        metrics.max_constraint_violation
    );
    // Debug msg
    const double total_fx =
        u_opt(0) + u_opt(3) + u_opt(6) + u_opt(9);

    const double total_fy =
        u_opt(1) + u_opt(4) + u_opt(7) + u_opt(10);

    const double total_fz =
        u_opt(2) + u_opt(5) + u_opt(8) + u_opt(11);

    const double mg =
        robot_mass * 9.81;

    const double predicted_az =
        total_fz / robot_mass - 9.81;

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        500,

        "VERTICAL | "
        "COMz=%.4f refCOMz=%.4f err=%.4f | "
        "vCOMz=%.4f refVz=%.4f | "
        "baseZ=%.4f baseRef=%.4f | "
        "Fz=%.2f mg=%.2f azPred=%.3f",

        p_com.z(),
        z_com_ref,
        p_com.z() - z_com_ref,

        v_com.z(),
        zdot_traj,

        p_base.z(),
        z_traj,   // if this is your new variable

        total_fz,
        mg,
        predicted_az
    );

    // RCLCPP_INFO_THROTTLE(
    //     this->get_logger(),
    //     *this->get_clock(),
    //     500,
    //     "MPC force sum: Fx=%.2f, Fy=%.2f, Fz=%.2f | "
    //     "position=[%.3f %.3f %.3f] ref=[%.3f %.3f %.3f]",
    //     total_fx,
    //     total_fy,
    //     total_fz,
    //     x_curr(3),
    //     x_curr(4),
    //     x_curr(5),
    //     x_ref_(3),
    //     x_ref_(4),
    //     x_ref_(5));

    for(int i=0; i<12; i++){
        mpc_cmd.ground_force[i] = u_opt[i];
    }

    mpc_cmd.reference_height = z_traj;
    mpc_cmd.steady_state = steady_state;

    mpc_cmd_pub_->publish(mpc_cmd);
}


void MPCNode::initializePinocchio()
{
    mjcf_path_ = "/workspace/src/controller/go2.xml";

    // Build model with go2.xml
    pinocchio::mjcf::buildModel(
        mjcf_path_,
        pin_model_,
        false);

    // Allocate the working memory/results object required to perform calculations on this model.
    pin_data_ = std::make_unique<pinocchio::Data>(pin_model_);

    // Neutral helps to initialize quaternion (x,y,z,w) as (0,0,0,1) instead of w being 0 (which makes the quat invalid).
    pin_q_ = pinocchio::neutral(pin_model_);
    // Create a correctly sized velocity vector and initially assume zero velocity.
    pin_v_ = Eigen::VectorXd::Zero(pin_model_.nv);

    // LowState order in our controller:
    // FR, FL, RR, RL
    const std::array<std::string, 12> joint_names = {
        "FR_hip_joint",
        "FR_thigh_joint",
        "FR_calf_joint",

        "FL_hip_joint",
        "FL_thigh_joint",
        "FL_calf_joint",

        "RR_hip_joint",
        "RR_thigh_joint",
        "RR_calf_joint",

        "RL_hip_joint",
        "RL_thigh_joint",
        "RL_calf_joint"
    };

    for (int i = 0; i < 12; ++i) {
        // Find Pinocchio's internal joint ID corresponding to this named MJCF joint.
        pin_joint_ids_[i] = pin_model_.getJointId(joint_names[i]);
    }

    const std::array<std::string, 4> foot_names = {
        "FR_foot",
        "FL_foot",
        "RR_foot",
        "RL_foot"
    };

    for (int i = 0; i < 4; ++i) {
        // Find Pinocchio's internal frame ID corresponding to this named foot frame.
        pin_foot_frame_ids_[i] = pin_model_.getFrameId(foot_names[i]);
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Pinocchio loaded: nq=%d nv=%d joints=%d frames=%d",
        static_cast<int>(pin_model_.nq),
        static_cast<int>(pin_model_.nv),
        static_cast<int>(pin_model_.njoints),
        static_cast<int>(pin_model_.nframes));
}