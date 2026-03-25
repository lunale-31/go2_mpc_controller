#!/bin/env python3
'''
Requires Sympy: sudo apt install python3 python3-sympy

This Python script calculates the Inverse Kinematics (IK) of a 3-DOF robotic arm with 
three rotational joints (angles θ1, θ2, θ3) using # SymPy for symbolic mathematics. 
The input is the Cartesian coordinates # (x, y, z) of the end effector.
                   ________
                  |        |
    (x, y, z) ==> |   IK   | ==> (θ1, θ2, θ3)
                  |________|

'''

from sympy import *

# Define symbols for Cartesian coordinates, joint angles, and link lengths
x, y, z, θ1, θ2, θ3 = symbols('x y z θ1 θ2 θ3')
L1, L2, L3, L4 = symbols('L1 L2 L3 L4')

# ANSI escape codes for text styling
COLOR = '\033[38;2;255;165;0m'  # Orange color
ITALIC = '\033[3m'  # Italic text
RESET = '\033[0m'  # Reset text formatting

# Define the transformation matrices for each joint
T01 = Matrix([
    [cos(θ1), -sin(θ1), 0, 0],
    [sin(θ1), cos(θ1), 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

T12 = Matrix([
    [1, 0, 0, 0],
    [0, cos(θ2), -sin(θ2), 0],
    [0, sin(θ2), cos(θ2), L1],
    [0, 0, 0, 1]
])

T23 = Matrix([
    [1, 0, 0, 0],
    [0, cos(θ3), -sin(θ3), L2],
    [0, sin(θ3), cos(θ3), 0],
    [0, 0, 0, 1]
])

T34 = Matrix([
    [1, 0, 0, L4],
    [0, 1, 0, 0],
    [0, 0, 1, L3],
    [0, 0, 0, 1]
])

# Compute the overall transformation matrix from base to end-effector
T04 = simplify(T01 * T12 * T23 * T34)

# Compute Cartesian coordinates (x, y, z) from the transformation matrix
V = T04 * Matrix([0, 0, 0, 1])
x_expr, y_expr, z_expr = V[0], V[1], V[2]

# Display Forward Kinematics results
print(f"{COLOR}\nForward Kinematics: (θ1, θ2, θ3) ==> (x, y, z){RESET}")
print("  φ = θ2 + θ3")
print(f"  x = {x_expr.subs({θ2 + θ3: symbols('φ')})}")
print(f"  y = {y_expr.subs({θ2 + θ3: symbols('φ')})}")
print(f"  z = {z_expr.subs({θ2 + θ3: symbols('φ')})}\n")

# Inverse Kinematics Solutions
print(f"{COLOR}                   ________                      {RESET}")
print(f"{COLOR}                  |        |                     {RESET}")
print(f"{COLOR}    (x, y, z) ==> |   IK   | ==> (θ1, θ2, θ3)    {RESET}")
print(f"{COLOR}                  |________|                     {RESET}\n")

# Solution for θ1
resultForθ1 = x_expr * cos(θ1) + y_expr * sin(θ1)
print(f"{COLOR}\nIK solution for θ1:  θ1 = f(x, y){RESET}")
print(f"  x*cos(θ1) + y*sin(θ1) = {simplify(resultForθ1)}")

# Solution for θ3
resultForθ3 = x_expr**2 + y_expr**2 + (z_expr - L1)**2
print(f"{COLOR}\nIK solution for θ3:  θ3 = f(x, y, z){RESET}")
print(f"  x**2 + y**2 + (z - L1)**2 = {simplify(resultForθ3)}")

# Solution for θ2
A = L3 * cos(θ3)
B = L2 - L3 * sin(θ3)
S_expr = sqrt(A**2 + B**2)
ψ_expr = atan2(B, A)
z_minus_L1 = S_expr * cos(θ2 - ψ_expr)

print(f"{COLOR}\nIK solution for θ2:  θ2 = f(z, θ3) = f(x, y, z){RESET}")
print(f"  z - L1 = {expand_trig(z_expr - L1)}")
print(f"  ↳ {ITALIC}This expression can be rewritten as:{RESET}")
print(f"     z - L1 = {z_minus_L1.subs({S_expr: symbols('S'), ψ_expr: symbols('ψ')})}")
print(f"     with S = {simplify(S_expr)}")
print(f"     and  ψ = {ψ_expr}")
print(f"  ↳ {ITALIC}Verification with SymPy:{RESET}")
print(f"     z - L1 - S*cos(θ2 - ψ) = {simplify(z_expr - L1 - z_minus_L1)}\n")
