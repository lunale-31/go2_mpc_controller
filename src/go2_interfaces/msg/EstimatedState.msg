std_msgs/Header header

float64[18] xhat

float64[3] rpy
float64[3] position
float64[3] velocity
float64[3] omega_world

float64[12] foot_positions_world
float64[36] leg_jacobians_world # 4 legs * 9 dimension

bool[4] contact

float64 nis

bool initialized
bool valid