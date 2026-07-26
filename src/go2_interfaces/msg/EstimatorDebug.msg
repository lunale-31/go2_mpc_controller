std_msgs/Header header

float64 dt

float64[3] rpy
float64[3] imu_accel_body
float64[3] accel_world

float64[18] x_predicted
float64[18] x_filtered

float64[28] measurement
float64[28] measurement_predicted
float64[28] innovation

float64[18] state_covariance_diagonal
float64[28] innovation_covariance_diagonal

# Validation
float64 nis 