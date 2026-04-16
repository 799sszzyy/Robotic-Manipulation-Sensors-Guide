import numpy as np
from scipy.spatial.transform import Rotation
def hand_eye_calibration(A_list, B_list):
"""
Solve AX = XB for hand-eye calibration.
Args:
A_list: List of 4x4 camera motion matrices
B_list: List of 4x4 robot motion matrices
Returns:
X: 4x4 hand-eye transformation matrix
"""
assert len(A_list) == len(B_list) >= 2
# Tsai-Lenz algorithm
# Step 1: Extract rotation components
n = len(A_list)
S = np.zeros((3*n, 3))
v = np.zeros(3*n)
for i in range(n):
Ra = A_list[i][:3, :3]
Rb = B_list[i][:3, :3]
# Skew-symmetric part
A_rot = Ra - np.eye(3)
b_rot = Rb - np.eye(3)
# Solve for rotation axis
S[3*i:3*i+3, :] = A_rot
v[3*i:3*i+3] = (Rb - Ra).reshape(9)[:3] # Simplified
# Step 2: Solve for rotation
# Using rotation averaging for robustness
# (Full implementation would use more sophisticated methods)
# Step 3: Solve for translation
# t_x = (R_a - I)ˆ+ @ (R_b @ t_b - t_a)
return X # 4x4 transformation matrix
def dual_quaternion_calibration(A_list, B_list):
"""Alternative using dual quaternions for better accuracy."""
from scipy.optimize import least_squares
def residual(x):
# x = [qx, qy, qz, qw, tx, ty, tz]
X = params_to_matrix(x)
error = 0
for A, B in zip(A_list, B_list):
# ||AX - XB||ˆ2
error += np.sum((A @ X - X @ B)**2)
return error.flatten()
# Initial guess
x0 = np.array([0, 0, 0, 1, 0, 0, 0])
result = least_squares(residual, x0)
return params_to_matrix(result.x)
