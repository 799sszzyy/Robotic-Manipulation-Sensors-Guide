import numpy as np
from scipy.linalg import expm, inv
class MultiModalEKF:
"""
EKF for fusing vision, IMU, and tactile data.
State: [position, velocity, quaternion, gyro_bias, accel_bias]
"""
def __init__(self):
self.x = np.zeros(16) # state vector
self.x[6:10] = [0, 0, 0, 1] # initial quaternion
self.P = np.eye(16) * 0.1 # covariance
# Process noise
self.Q = np.diag([0.01]*3 + [0.001]*3 + [0.001]*4 +
[0.0001]*3 + [0.0001]*3)
# Measurement noise (tunable per modality)
self.R_imu = np.diag([0.01, 0.01, 0.01, # gyro
0.1, 0.1, 0.1]) # accel
self.R_vision = np.eye(6) * 0.01 # pose
self.R_tactile = np.eye(3) * 0.001 # contact position
def predict(self, gyro, accel, dt):
"""Prediction step using IMU."""
# State extraction
p = self.x[0:3] # position
v = self.x[3:6] # velocity
q = self.x[6:10] # quaternion (w, x, y, z)
bg = self.x[10:13] # gyro bias
ba = self.x[13:16] # accel bias
# Compensate biases
gyro_corrected = gyro - bg
accel_corrected = accel - ba
# Quaternion integration
omega = np.array([0, *gyro_corrected])
q_dot = 0.5 * self.quaternion_multiply(q, omega)
q_new = q + q_dot * dt
q_new = q_new / np.linalg.norm(q_new) # normalize
# Rotate acceleration to world frame
R = self.quaternion_to_matrix(q)
accel_world = R @ accel_corrected
# Update position and velocity
p_new = p + v * dt + 0.5 * accel_world * dt**2
v_new = v + accel_world * dt
# Update state
self.x[0:3] = p_new
self.x[3:6] = v_new
self.x[6:10] = q_new
# biases remain constant in prediction
# Update covariance (simplified - full implementation
# would compute Jacobian F)
F = self.compute_state_jacobian(gyro_corrected,
accel_corrected, dt)
self.P = F @ self.P @ F.T + self.Q
def update_vision(self, z_pose):
"""Update with vision pose measurement."""
H = np.zeros((6, 16))
H[0:3, 0:3] = np.eye(3) # position
H[3:6, 6:10] = self.quaternion_jacobian() # orientation
self._update_common(z_pose, H, self.R_vision)
def update_tactile(self, z_contact, contact_model):
"""Update with tactile contact position."""
# contact_model maps state to expected contact position
H = contact_model.jacobian(self.x)
self._update_common(z_contact, H, self.R_tactile)
def _update_common(self, z, H, R):
"""Common update step."""
y = z - H @ self.x # residual
S = H @ self.P @ H.T + R
K = self.P @ H.T @ inv(S) # Kalman gain
self.x = self.x + K @ y
self.P = (np.eye(16) - K @ H) @ self.P
@staticmethod
def quaternion_to_matrix(q):
"""Convert quaternion (w, x, y, z) to rotation matrix."""
w, x, y, z = q
return np.array([
[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
[2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
[2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]
])
