#include "controller/mpc.h"

MPC::MPC(double dt){
    mpc_dt_ = dt;
    Q1_.setZero();
    Q2_.setZero();
    Qf_.setZero();

    m_x.setZero();
    m_xref.setZero();

    m_u.setZero();

    // QP matrices initialization
    m_Aqp.resize(states_nr * hp, states_nr);
    m_Bqp.resize(states_nr * hp, inputs_nr * hc);
    m_Xqp.resize(states_nr * hp, 1);
    m_Uqp.resize(inputs_nr * hc); 
    
    m_Xqpref.resize(states_nr * hp, 1);
    m_Q1qp.resize(states_nr * hp, states_nr * hp);
    m_Q2qp.resize(inputs_nr * hc, inputs_nr * hc);

    Hqp.resize(inputs_nr * hc, inputs_nr * hc);
    gqp.resize(inputs_nr * hc);

    Hqp.setZero();
    gqp.setZero();

    m_Xqpref.setZero();
    m_Q1qp.setZero();
    m_Q2qp.setZero();

    m_Aqp.setZero();
    m_Bqp.setZero();
    m_Xqp.setZero();
    m_Uqp.setZero();

    setWeights();
}

void MPC::setWeights(){
    // Q1_.diagonal() <<  50.0,   // roll
    //     100.0,   // pitch
    //     25.0,    // yaw

    //     25.0,    // position x
    //     25.0,    // position y
    //     200.0,  // position z

    //     5.0,    // omega x
    //     5.0,    // omega y
    //     1.0,    // omega z

    //     10.0,   // linear velocity x
    //     10.0,   // linear velocity y
    //     20.0,   // linear velocity z

    //     0.0;    // gravity state

    Q2_.diagonal().setConstant(1e-3);

    // Bryson's rule
    Q1_.diagonal() <<  2500.0, 2500.0, 111111.0,  
           3460.0, 3460.0,   2500.0,  
            100.0,  100.0,   100.0, 
            100.0,  100.0,  100.0;

    // Q2_.diagonal() << 0.01, 0.01, 0.0016,  
    //        0.01, 0.01, 0.0016,  
    //        0.01, 0.01, 0.0016,  
    //        0.01, 0.01, 0.0016;

    Qf_ = Q1_;
    Qf_(0,0) *= 2.0; // Roll
    Qf_(1,1) *= 2.0; // Pitch
    Qf_(3,3) *= 2.0; // x
    Qf_(4,4) *= 2.0; // y
    Qf_(5,5) *= 3.0; // Height
    Qf_(11,11) *=  2.0; // Vertical velocity   
}

void MPC::setReference(Eigen::Matrix<double,13,1>& xref){
    m_xref = xref;
}

void MPC::computeDynamics(Dynamics& go2,const std::vector<Eigen::Vector3d>& foot_pos_com_world,
                          double current_yaw,const Eigen::Matrix3d& inertia_body,double mass)
{
    go2.computeA(current_yaw);
    go2.computeB(foot_pos_com_world,inertia_body,mass,current_yaw);
    go2.discretize(mpc_dt_);

    m_Ad = go2.getA();
    m_Bd = go2.getB();
    m_C = go2.getC();   

    buildMatrix(); 
}

void MPC::buildMatrix(){
    // Unroll dynamics over prediction horizon
    std::vector<Eigen::Matrix<double, states_nr, states_nr>> A_powers(hp + 1);
    A_powers[0] = Eigen::Matrix<double, states_nr, states_nr>::Identity();
    
    for(int power = 1; power<=hp; power++){
        A_powers[power] = A_powers[power - 1] * m_Ad; 
    }

    for(int pred_step = 0; pred_step<hp; pred_step++){
        m_Aqp.block(pred_step * states_nr, 0, states_nr, states_nr) = A_powers[pred_step + 1];

        for(int con_step = 0; con_step<=pred_step && con_step < hc; con_step++){
            // Bqp unroll
            int exponent = pred_step - con_step;
            m_Bqp.block(pred_step * states_nr, con_step * inputs_nr, states_nr, inputs_nr) = A_powers[exponent] * m_Bd;
        }
    }

    // Xref, Q1 unroll
    for(int pred_step = 0; pred_step<hp; pred_step++){
        m_Xqpref.block(pred_step * states_nr, 0, states_nr, 1) = m_xref;

        if(pred_step == hp-1){
            m_Q1qp.block(pred_step * states_nr, pred_step * states_nr, states_nr, states_nr) = Qf_;
        }
        else{
            m_Q1qp.block(pred_step*states_nr, pred_step * states_nr, states_nr, states_nr) = Q1_;
        } 
    }
    
    // Q2 unroll
    for(int con_step = 0; con_step < hc; con_step++){
        m_Q2qp.block(con_step*inputs_nr, con_step * inputs_nr, inputs_nr, inputs_nr) = Q2_; 
    }
}

void MPC::buildCost(Eigen::Matrix<double,13,1>& x_curr){
    Hqp.setZero();
    Hqp = m_Bqp.transpose() * m_Q1qp * m_Bqp + m_Q2qp;
    gqp = m_Bqp.transpose() * m_Q1qp * (m_Aqp * x_curr - m_Xqpref);
    rho_qp = (m_Aqp * x_curr - m_Xqpref).transpose() * m_Q1qp * (m_Aqp * x_curr - m_Xqpref);
}

void MPC::buildConstraints(std::vector<Eigen::Matrix3d>& leg_jacobians_world, std::array<double, 12>& gravity_tau){
    /* 2 Constraints: Friction and Torque.*/
    /* TODO: Add Swing-leg force constraint after adding gait, contact and trajectory planners.*/

    // 1) Friction
    Eigen::Matrix<double, 5,3> C_f_leg;
    Eigen::Vector<double, 5> l_f_leg;
    Eigen::Vector<double, 5> u_f_leg;
    
    C_f_leg.setZero();
    C_f_leg << 1.0, 0.0, -mu_,
               -1.0, 0.0, -mu_,
               0.0, 1.0, -mu_,
               0.0, -1.0, -mu_,
               0.0, 0.0, 1.0;

    l_f_leg.setZero();
    l_f_leg << negInf,
               negInf,
               negInf,
               negInf,
               fz_min;

    u_f_leg.setZero();
    u_f_leg << 0.0,
               0.0,
               0.0,
               0.0,
               fz_max;
               
    Eigen::Matrix<double, 20, 12> C_f; 
    Eigen::Vector<double, 20> l_f;
    Eigen::Vector<double, 20> u_f;

    C_f.setZero();
    l_f.setZero();
    u_f.setZero();
    
    for(int leg = 0; leg<4; leg++){
        C_f.block(leg*5, leg*3, 5, 3) = C_f_leg;
        l_f.block(leg*5, 0, 5, 1) = l_f_leg;
        u_f.block(leg*5, 0, 5, 1) = u_f_leg; 
    }

    // Over the prediction/control horizon, 
    // 20 x 5 and 12 x 5 as hc=5
    Eigen::MatrixXd A_friction; 
    Eigen::VectorXd l_friction;
    Eigen::VectorXd u_friction;

    A_friction.resize(20 * hc, inputs_nr * hc);
    l_friction.resize(20 * hc);
    u_friction.resize(20 * hc);

    A_friction.setZero();
    l_friction.setZero();
    u_friction.setZero();

    for(int i=0; i<hc; i++){
        A_friction.block(i*20, i* inputs_nr, 20, inputs_nr) = C_f; 
        l_friction.block(i*20, 0, 20, 1) = l_f;
        u_friction.block(i*20, 0, 20, 1) = u_f;
    }

    // Torque
    Eigen::Matrix<double, 12, 12> torque_jacobian_legs; 
    Eigen::Vector<double, 3> torque_l_;
    Eigen::Vector<double, 3> torque_u_;
    
    torque_l_ << tau_min, 
                 tau_min, 
                 tau_min_calf; 

    torque_u_ << tau_max,
                 tau_max,
                 tau_max_calf;

    torque_jacobian_legs.setZero();

    Eigen::Vector<double, 12> torque_l_legs;
    Eigen::Vector<double, 12> torque_u_legs;

    for(int leg=0; leg<4; leg++){
        torque_jacobian_legs.block(leg*3, leg*3, 3, 3) = -leg_jacobians_world[leg].transpose();
        
        const Eigen::Map<const Eigen::Matrix<double,12,1>> g_tau(gravity_tau.data());

        torque_l_legs.block(leg*3, 0, 3, 1) = torque_l_ - g_tau.segment<3>(leg*3);;
        torque_u_legs.block(leg*3, 0, 3, 1) = torque_u_ - g_tau.segment<3>(leg*3);; 
    }

    Eigen::MatrixXd A_torque;
    Eigen::VectorXd l_torque;
    Eigen::VectorXd u_torque;

    A_torque.resize(12*hc, inputs_nr*hc);
    l_torque.resize(12*hc); 
    u_torque.resize(12*hc); 
    
    A_torque.setZero();
    l_torque.setZero();
    u_torque.setZero();

    for(int i=0; i<hc; i++){
        A_torque.block(12*i, 12*i, 12, 12) = torque_jacobian_legs;
        l_torque.block(12*i, 0, 12,1) = torque_l_legs;
        u_torque.block(12*i, 0, 12,1) = torque_u_legs;
    }

    const int friction_rows = 20 * hc;
    const int torque_rows = 12 * hc;
    const int variable_count = inputs_nr * hc;
    const int constraint_count = friction_rows + torque_rows;

    m_constraint_matrix.resize(constraint_count, variable_count);
    m_constraint_matrix.setZero();

    m_constraint_matrix.block(0, 0, friction_rows, variable_count) = A_friction;
    m_constraint_matrix.block(friction_rows, 0, torque_rows, variable_count) = A_torque;

    m_lower_bounds.resize(constraint_count);
    m_upper_bounds.resize(constraint_count);

    m_lower_bounds.setZero();
    m_upper_bounds.setZero();

    m_lower_bounds.segment(0, friction_rows) = l_friction;
    m_lower_bounds.segment(friction_rows, torque_rows) = l_torque;

    m_upper_bounds.segment(0, friction_rows) = u_friction;
    m_upper_bounds.segment(friction_rows, torque_rows) = u_torque;
}

bool MPC::solve(){
    // Values stored as double, column major sparse storage, and integer type used for row/column index. 
    using SparseMatrix = Eigen::SparseMatrix<double, Eigen::ColMajor, osqp::c_int>;

    Eigen::MatrixXd P_dense = Hqp; 

    // Remove small numerical asymmetry. 
    P_dense = 0.5 * (P_dense + P_dense.transpose());

    // Convert dense matrices to sparse matrices.
    SparseMatrix P_sparse = P_dense.sparseView();
    SparseMatrix A_sparse = m_constraint_matrix.sparseView();

    // Required by osqp-cpp.
    P_sparse.makeCompressed();
    A_sparse.makeCompressed();
    
    // Check official git repo to see how to use the solver step-by-step. 
    // Create the solver instance
    osqp::OsqpInstance instance;

    // Update the vars
    instance.constraint_matrix = A_sparse;
    instance.lower_bounds = m_lower_bounds; 
    instance.upper_bounds = m_upper_bounds;

    instance.objective_matrix = P_sparse;
    instance.objective_vector = gqp;

    // Solver settings
    osqp::OsqpSettings settings;
    settings.verbose = false;

    // Initialize solver
    osqp::OsqpSolver solver;

    const auto init_status = solver.Init(instance, settings);

    if(!init_status.ok()){
        std::cerr << "OSQP initialization failed"<< init_status.ToString() << std::endl;
        m_u.setZero();
        return false;
    }

    // Solve the QP
    const osqp::OsqpExitCode exit_code = solver.Solve();
    if(exit_code != osqp::OsqpExitCode::kOptimal){
        std::cerr<< "Osqp solver failed" << osqp::ToString(exit_code) << std::endl;
        m_u.setZero();
        return false;
    }

    // Full solution
    Eigen::VectorXd solution = solver.primal_solution();

    // Check
    if (solution.size() != inputs_nr * hc ||
        !solution.allFinite())
    {
        std::cerr
            << "OSQP returned an invalid solution."
            << std::endl;

        m_u.setZero();
        return false;
    }

    m_Uqp = solution;

    // MPC applying first output
    m_u = solution.head<inputs_nr>();

    return true;
}

Eigen::Matrix<double, 12, 1> MPC::getControlSignal(){
    return m_u;
}

const Eigen::MatrixXd&
MPC::getConstraintMatrix() const
{
    return m_constraint_matrix;
}

const Eigen::VectorXd&
MPC::getLowerBounds() const
{
    return m_lower_bounds;
}

const Eigen::VectorXd&
MPC::getUpperBounds() const
{
    return m_upper_bounds;
}

const Eigen::VectorXd&
MPC::getFullSolution() const
{
    return m_Uqp;
}
