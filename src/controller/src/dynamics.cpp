#include "controller/dynamics.h"

Dynamics::Dynamics(){
    mass = 15.206;
    body_inertia = Eigen::Matrix3d::Zero();

    body_inertia(0,0) = 0.0244531;
    body_inertia(1,1) = 0.0980771;
    body_inertia(2,2) = 0.107027; 

    m_C.setZero();
    m_C.block<12,12>(0,0) = Eigen::Matrix<double, 12, 12>::Identity();    
}

Eigen::Matrix3d Dynamics::getRotationMatrix(double yaw){
    Eigen::Matrix3d Rz; 
    double c = std::cos(yaw);
    double s = std::sin(yaw); 

    Rz << c, -s, 0.0,
          s, c, 0.0,
          0.0, 0.0, 1.0; 

    return Rz; 
}

Eigen::Matrix3d Dynamics::getRotationMatrixRoll(double roll){
    Eigen::Matrix3d Rx; 
    double c = std::cos(roll);
    double s = std::sin(roll); 

    Rx << 1, 0.0, 0.0,
          0.0, c, -s,
          0.0, s, c; 

    return Rx; 
}

Eigen::Matrix3d Dynamics::getRotationMatrixPitch(double pitch){
    Eigen::Matrix3d Ry; 
    double c = std::cos(pitch);
    double s = std::sin(pitch); 

    Ry << c, 0.0, s,
          0.0, 1.0, 0.0,
          -s, 0.0, c; 

    return Ry; 
}

Eigen::Matrix3d Dynamics::getSkewSymMatrix(const Eigen::Vector3d& r){
    Eigen::Matrix3d s; 
    s << 0, -r(2), r(1),
         r(2), 0, -r(0),
         -r(1), r(0), 0; 
    
    return s; 
}

void Dynamics::computeA(double yaw){
    Eigen::Vector3d g;
    g << 0, 0, -9.81; 

    m_A.setZero();

    m_A.block<3,3>(0,6) = getRotationMatrix(yaw).transpose(); 
    m_A.block<3,3>(3,9) = Eigen::Matrix3d::Identity(); 
    m_A.block<3,1>(9,12) = g; 
}

void Dynamics::computeB(std::vector<Eigen::Vector3d>& foot_positions_world, double yaw){
    int n = foot_positions_world.size(); // number of legs that has contact on ground

    m_B.resize(13, 3 * n); // resizing column to number of foot contact because originally declared as Xd 
    m_B.setZero();
    Eigen::Matrix3d Rz = getRotationMatrix(yaw); 

    Eigen::Matrix3d I_world = Rz*body_inertia*(Rz.transpose());
    Eigen::Matrix3d I_inv = I_world.inverse();

    for(int i = 0; i<n; i++){
        Eigen::Matrix3d r_skew = getSkewSymMatrix(foot_positions_world[i]);
        m_B.block<3,3>(6,i*3) = I_inv*r_skew; // 6, i*3 : for 1st leg, i = 0 , and it takes 3 columns from 0(0,1,2).
        // for 2nd leg, i = 1, it takes 3 columns from 3 (3,4,5). These 3 columns are basically related to 3 forces per foot (fx, fy, fz)
        m_B.block<3,3>(9,i*3) = Eigen::Matrix3d::Identity()/mass; 
    }
}

void Dynamics::discretize(double dt){
    Eigen::MatrixXd M;
    
    M.resize(m_A.rows() + m_B.cols(), m_A.rows() + m_B.cols()); 
    M.setZero();

    M.block<13,13>(0,0) = m_A;
    M.block(0,13,13,m_B.cols()) = m_B; 

    M *= dt;
    Eigen::MatrixXd M_exp = M.exp();

    m_Ad = M_exp.block<13,13>(0,0); // <> dimensions must be known compile-time
    m_Bd = M_exp.block(0,13,13,m_B.cols()); // () n is determined in run-time, so use this bracket for it
}


Eigen::Matrix<double,13,13> Dynamics::getA() const{
    return m_Ad; 
} 
Eigen::Matrix<double, 12,13> Dynamics::getC() const{
    return m_C; 
}
Eigen::MatrixXd Dynamics::getB() const{
    return m_Bd; 
}