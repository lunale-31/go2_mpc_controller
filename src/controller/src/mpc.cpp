#include "controller/mpc.h"

MPC::MPC(double dt){
    mpc_dt_ = dt;
    Q1_.setZero();
    Q2_.setZero();

    m_x.setZero();
    m_xref.setZero();

    m_u.setZero();

    setWeights();
}

void MPC::setWeights(){
    Q1_.setZero();
    Q2_.setZero();
}

void MPC::setReference(Eigen::Matrix<double,13,1>& xref){
    m_xref = xref;
}

void MPC::computeLTVMatrices(double dt, Dynamics& go2, std::vector<Eigen::Vector3d> foot_pos_world, double current_yaw){
    go2.computeA(current_yaw);
    go2.computeB(foot_pos_world, current_yaw);
    go2.discretize(mpc_dt_);

    auto A_d = go2.getA();
    auto B_d = go2.getB();
    auto C_d = go2.getC();

    
}
