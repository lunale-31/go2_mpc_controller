#include "controller/state_estimator.h"

KalmanFilter::KalmanFilter(double dt){
    // A matrix and B matrix initializations 
    m_A = Eigen::Matrix<double, 18, 18>::Identity(); 
    m_A.block<3,3>(0, 3) = Eigen::Matrix3d::Identity() * dt;

    m_B.setZero();
    m_B.block<3,3>(0, 0) = Eigen::Matrix3d::Identity() * (dt * dt * 0.5);
    m_B.block<3,3>(3, 0) = Eigen::Matrix3d::Identity() * dt;

    // C matrix initializations
    Eigen::Vector3d ez; 
    ez << 0.0, 0.0, 1.0; 

    m_C.setZero(); 
    
    m_C.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity(); 
    m_C.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity(); 
    m_C.block<3, 3>(6, 0) = -Eigen::Matrix3d::Identity(); 
    m_C.block<3, 3>(9, 0) = -Eigen::Matrix3d::Identity(); 

    m_C.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity(); 
    m_C.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity(); 
    m_C.block<3, 3>(6, 12) = Eigen::Matrix3d::Identity(); 
    m_C.block<3, 3>(9, 15) = Eigen::Matrix3d::Identity(); 


    m_C.block<3, 3>(12, 3) = Eigen::Matrix3d::Identity(); 
    m_C.block<3, 3>(15, 3) = Eigen::Matrix3d::Identity(); 
    m_C.block<3, 3>(18, 3) = Eigen::Matrix3d::Identity(); 
    m_C.block<3, 3>(21, 3) = Eigen::Matrix3d::Identity(); 

    m_C.block<1, 3>(24, 6) = ez.transpose(); 
    m_C.block<1, 3>(25, 9) = ez.transpose(); 
    m_C.block<1, 3>(26, 12) = ez.transpose(); 
    m_C.block<1, 3>(27, 15) = ez.transpose(); 
    
    // Initial state
    m_xhat.setZero(); 

    // Input signal
    m_u.setZero(); 
    
    // Measurement signal
    m_y.setZero(); 
    m_yhat.setZero();
    m_innovation.setZero();

    // Innovation covariance
    S.setZero();

    // State covariance
    P.setZero();

    // NIS
    NIS = std::numeric_limits<double>::quiet_NaN();

    // Body position 
    P.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * 0.25;
    // Body velocity (almost always starts up while being completely stationary on the ground)
    P.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * 0.001; 
    // Foot positions
    P.block<12,12>(6,6) = Eigen::Matrix<double, 12, 12>::Identity() * 0.01; 

    // Process and measurement noise covariance
    R1.setZero(); 
    R2.setZero(); 

    // Position noise (Just an integrator using velocity, so the noise can be less)
    R1.block<3,3>(0, 0) = Eigen::Matrix3d::Identity() * 1e-3;
    // Higher because leg impacts break constant-accel rules
    R1.block<3,3>(3, 3) = Eigen::Matrix3d::Identity() * 1e-3;
    // Less assuming stance foot shouldn't move.
    // TODO: Dynamically increase this for swing legs later when implementing other gaits 
    R1.block<12,12>(6, 6) = Eigen::Matrix<double, 12, 12>::Identity() * 1e-4;

    // Relative foot position measurement noise
    // Lower because measurement is from kinematics calculator
    R2.block<12,12>(0,0) =
        Eigen::Matrix<double,12,12>::Identity() * 0.000625;

    // Velocity measurement noise
    R2.block<12,12>(12,12) =
        Eigen::Matrix<double,12,12>::Identity() * 0.01;

    // Foot height measurement noise
    R2.block<4,4>(24,24) =
        Eigen::Matrix<double,4,4>::Identity() * 0.000025;

    // Kalman Gain 
    L.setZero();
}

bool KalmanFilter::initializeKF(const std::vector<Eigen::Vector3d>& foot_positions_world){
    if(foot_positions_world.size() != 4){
        return false;
    }

    double body_height = 0.0;

    for(int i = 0; i<4; i++){
        if(!foot_positions_world[i].allFinite()){
            return false; 
        }

        body_height += 0.022 -foot_positions_world[i][2];
    }
    // Initially the position of the z coordinate is 1/4th of summation of all the foot positions vector. 
    body_height = body_height / 4.0;

    // Reset all values
    m_xhat.setZero();

    // Position
    m_xhat[0] = 0.0;
    m_xhat[1] = 0.0;
    m_xhat[2] = body_height;

    // Velocity
    m_xhat[3] = 0.0;
    m_xhat[4] = 0.0;
    m_xhat[5] = 0.0;

    for(int leg=0; leg<4; leg++){
        // Pf = P + r_w (Foot position in world frame = Body position wrt world frame + foot position from body wrt world frame)
        m_xhat.segment<3>(6 + 3*leg) = m_xhat.segment<3>(0) + foot_positions_world[leg]; 
    }

    return true;
}

void KalmanFilter::predictionStep(const Eigen::Vector3d& u){
    m_u = u; 

    // Prediction step 
    m_xhat = m_A*m_xhat + m_B*m_u; 
    P = m_A*P*(m_A.transpose()) + R1;   
}

void KalmanFilter::filteringStep(const std::vector<Eigen::Vector3d>& foot_positions_world, const std::vector<Eigen::Vector3d>& foot_velocities_world){
    // Measurement vector
    if(foot_positions_world.size() == 4 && foot_velocities_world.size() == 4){
        for(int i = 0; i<4; i++){
            m_y.block<3,1>(i*3, 0) = foot_positions_world[i]; 
            m_y.block<3,1>((i*3)+12, 0) = -foot_velocities_world[i];
            
            // 4 foot height = 0, assuming foot are in stance. 
            m_y(24 + i) = 0.022;
        } 
    }
    else{
        // TODO: Later, setup the update for moments when foot is not in contact. (height can be found using normal z 
        // coordinate formula: m_xhat(2) + foot_positions_world[i].z() (pb + ri)
        m_y.setZero();
    }

    // Compute L, S -> innovation covariance
    S = m_C * P * m_C.transpose() + R2;
    const auto PCt = P * m_C.transpose(); 

    // Theory L = P*m_C.transpose() * ((m_C * P * m_C.transpose()) + R2).inverse(), but the below one is more stable and faster numerically.
    L = S.ldlt().solve(PCt.transpose()).transpose();  

    // Measurement prediction
    m_yhat = m_C * m_xhat;

    // Innovation vector
    m_innovation = m_y - m_yhat;

    // NIS computation
    NIS = computeNIS(m_innovation, S); 

    // Filtering update
    m_xhat = m_xhat + L*m_innovation; 

    const Eigen::Matrix<double, 18, 18> I = Eigen::Matrix<double, 18, 18>::Identity();
    const auto A = I - (L* m_C); 

    // Theoretically P = P - (L * m_C * P), but numerically stable is given below (Joseph covariance update)
    P = A * P * A.transpose() + L * R2 * L.transpose();  
}

double KalmanFilter::computeNIS(Eigen::Matrix<double, 28,1> innovation, Eigen::Matrix<double, 28,28> s){
    // NIS = i.Transpose * S.inverse() * i 
    const auto ldlt = s.ldlt();

    if (ldlt.info() != Eigen::Success) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    double nis = innovation.dot(ldlt.solve(innovation));

    return nis; 
}

Eigen::Matrix<double, 18, 1> KalmanFilter::getXhat() const{ 
    return m_xhat; 
}  

Eigen::Matrix<double, 28, 1> KalmanFilter::getResidualError() const{
    return m_y - (m_C * m_xhat);
}

Eigen::Matrix<double, 28, 1> KalmanFilter::getInnovation() const{
    return m_innovation;
}

Eigen::Matrix<double, 28, 1> KalmanFilter::getMeasurement() const{
    return m_y; 
}

Eigen::Matrix<double, 28, 1> KalmanFilter::getMeasurementPrediction() const{
    return m_yhat;
}

Eigen::Matrix<double, 18, 1> KalmanFilter::getStateCovarianceDiag() const{
    return P.diagonal(); 
}

Eigen::Matrix<double, 28, 1> KalmanFilter::getInnovationCovarianceDiag() const{
    return S.diagonal();
}

double KalmanFilter::getNIS() const{
    return NIS; 
}