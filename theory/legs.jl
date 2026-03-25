using LinearAlgebra

# Given the angles j1 .. j3, compute the position of the feet relative to the base
# This function models right-side legs. For the left side, l1 must be inverted
# j1: hip; j2: thigh; j3: calf
function m(j1, j2, j3)
    R1 = [1 0 0; 0 cos(j1) -sin(j1); 0 sin(j1) cos(j1)]
    R2 = [cos(j2) 0 sin(j2); 0 1 0; -sin(j2) 0 cos(j2)]
    R3 = [cos(j3) 0 sin(j3); 0 1 0; -sin(j3) 0 cos(j3)]
    l1 = 0.0955 # hip length
    l2 = 0.213  # thigh length
    l3 = 0.23   # calf length
    return R1 * ([0; -l1; 0] + R2 * ([0; 0; -l2] + R3 * [0; 0; -l3]))
end

# Compute leg vector symbolically
using Symbolics
@variables j1 j2 j3
m(j1, j2, j3)

# Returns:
# 3-element Vector{Num}:
#                            -sin(-j2)*(-0.213 - 0.23cos(-j3)) + 0.23cos(-j2)*sin(-j3)
#  -0.0955cos(j1) - (0.23sin(-j2)*sin(-j3) + cos(-j2)*(-0.213 - 0.23cos(-j3)))*sin(j1)
#  -0.0955sin(j1) + (0.23sin(-j2)*sin(-j3) + cos(-j2)*(-0.213 - 0.23cos(-j3)))*cos(j1)

L1 = 0.095; L2 = 0.213; L3 = 0.23

# T1 transformation matrix
function T1(theta)
    return [
        1 0 0 0;
        0 cos(theta) -sin(theta) 0;
        0 sin(theta) cos(theta) 0;
        0 0 0 1
    ]
end

# T2 transformation matrix
function T2(theta)
    return [
       cos(theta) 0 -sin(theta) 0;
       0 1 0 L1;
       sin(theta) 0 cos(theta) 0;
       0 0 0 1
    ]
end

# T3 transformation matrix
function T3(theta)
    return [
       cos(theta) 0 -sin(theta) 0;
       0 1 0 0;
       sin(theta) 0 cos(theta) -L2;
       0 0 0 1
    ]
end

# Complete transformation matrix
function T(j1, j2, j3)
    return T1(j1) * T2(j2) * T3(j3) * [0; 0; L3; 1]


T1(j1) * T2(j2) * T3(j3) * [0; 0; -l3; 1]
# Returns:    
# 4-element Vector{Num}:
#                           l3*sin(j3)*cos(j2) - (-0.213 - l3*cos(j3))*sin(j2)
#  0.095cos(j1) - (l3*sin(j3)*sin(j2) + (-0.213 - l3*cos(j3))*cos(j2))*sin(j1)
#  0.095sin(j1) + (l3*sin(j3)*sin(j2) + (-0.213 - l3*cos(j3))*cos(j2))*cos(j1)
#                                                                         1.0
