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
    m_Uqp.resize(inputs_nr * hc, 1); 
    
    m_Xqpref.resize(states_nr * hp, 1);
    m_Q1qp.resize(states_nr * hp, states_nr * hp);
    m_Q2qp.resize(inputs_nr * hc, inputs_nr * hc);

    m_Aqp.setZero();
    m_Bqp.setZero();
    m_Xqp.setZero();
    m_Uqp.setZero();

    setWeights();
}

void MPC::setWeights(){
    Q1_.diagonal() <<  50.0,   // roll
        50.0,   // pitch
        5.0,    // yaw

        2.0,    // position x
        2.0,    // position y
        200.0,  // position z

        5.0,    // omega x
        5.0,    // omega y
        1.0,    // omega z

        10.0,   // linear velocity x
        10.0,   // linear velocity y
        20.0,   // linear velocity z

        0.0;    // gravity state

    Q2_.diagonal() << 1e-3, 1e-3, 1e-3, // FR forces
                      1e-3, 1e-3, 1e-3, // FL forces
                      1e-3, 1e-3, 1e-3, // RR forces
                      1e-3, 1e-3, 1e-3; // RL forces

    Qf_ = Q1_;
    Qf_(0,0) *= 2.0; // Roll
    Qf_(1,1) *= 2.0; // Pitch
    Qf_(5,5) *= 3.0; // Height
    Qf_(11,11) *=  2.0; // Vertical velocity   
}

void MPC::setReference(Eigen::Matrix<double,13,1>& xref){
    m_xref = xref;
}

void MPC::computeDynamics(double dt, Dynamics& go2, std::vector<Eigen::Vector3d> foot_pos_world, double current_yaw){
    go2.computeA(current_yaw);
    go2.computeB(foot_pos_world, current_yaw);
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