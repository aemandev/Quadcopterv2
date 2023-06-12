import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Rotation

def measurement_function(state):
    q, omega = state[:4], state[4:]

    # Normalize quaternion
    q = q / np.linalg.norm(q)

    # Compute expected accelerometer and gyroscope measurements
    gravity = np.array([0, 0, -9.81])  # Gravity vector in the body frame
    a_expected = Rotation.from_quat(q).apply(gravity, inverse=True)
    g_expected = omega

    return np.concatenate((a_expected, g_expected))

def ukf_update(state_pred, P_pred, measurements, R):
    n = len(state_pred)
    m = len(measurements)
    kappa = 3 - n

    # Generate sigma points
    L = np.linalg.cholesky(P_pred + (n + kappa) * np.identity(n))
    sigma_points = np.zeros((2 * n + 1, n))
    sigma_points[0] = state_pred
    for i in range(n):
        sigma_points[i + 1] = state_pred + L[i]
        sigma_points[n + i + 1] = state_pred - L[i]

    # Compute predicted measurements
    W = np.full(2 * n + 1, 0.5 / (n + kappa))
    W[0] = kappa / (n + kappa)
    predicted_measurements = np.zeros((2 * n + 1, m))
    for i, sigma_point in enumerate(sigma_points):
        predicted_measurements[i] = measurement_function(sigma_point)

    # Compute measurement mean and covariance
    measurement_mean = np.sum(W[:, None] * predicted_measurements, axis=0)
    P_yy = np.zeros((m, m))
    for i in range(2 * n + 1):
        delta = predicted_measurements[i] - measurement_mean
        P_yy += W[i] * np.outer(delta, delta)
    P_yy += R

    # Compute cross-covariance
    P_xy = np.zeros((n, m))
    for i in range(2 * n + 1):
        delta_x = sigma_points[i] - state_pred
        delta_y = predicted_measurements[i] - measurement_mean
        P_xy += W[i] * np.outer(delta_x, delta_y)

    # Compute Kalman gain
    K = np.linalg.solve(P_yy.T, P_xy.T).T

    # Update state and covariance
    measurement_residual = measurements - measurement_mean
    state_updated = state_pred + K @ measurement_residual
    P_updated = P_pred - K @ P_yy @ K.T

    return state_updated, P_updated



def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return np.array([w, x, y, z])

def state_transition_function(state, dt):
    q, omega = state[:4], state[4:]

    # Normalize quaternion
    q = q / np.linalg.norm(q)

    # Convert angular velocity to quaternion derivative
    omega_quat = np.concatenate(([0], omega))
    q_dot = 0.5 * quaternion_multiply(q, omega_quat)

    # Integrate quaternion using simple forward Euler integration
    q_new = q + q_dot * dt

    # Normalize the updated quaternion
    q_new = q_new / np.linalg.norm(q_new)

    # Assume constant angular velocity (no dynamics model for omega)
    omega_new = omega

    return np.concatenate((q_new, omega_new))




def markley_quaternion_average(Q, weights):
    Q_matrix = np.dot((weights[:, None] * Q).T, Q)
    eigenvalues, eigenvectors = np.linalg.eigh(Q_matrix)
    return eigenvectors[:, np.argmax(eigenvalues)]

def ukf_predict_with_markley(state, P, Q, dt):
    n = len(state)
    kappa = 3 - n

    # Generate sigma points
    L = np.linalg.cholesky(P + (n + kappa) * Q)
    sigma_points = np.zeros((2 * n + 1, n))
    sigma_points[0] = state
    for i in range(n):
        sigma_points[i + 1] = state + L[i]
        sigma_points[n + i + 1] = state - L[i]

    # Propagate sigma points through the state transition function
    sigma_points_pred = np.zeros_like(sigma_points)
    for i, sigma_point in enumerate(sigma_points):
        sigma_points_pred[i] = state_transition_function(sigma_point, dt)

    # Compute predicted state mean and covariance
    W = np.full(2 * n + 1, 0.5 / (n + kappa))
    W[0] = kappa / (n + kappa)

    # Use Markley averaging for quaternion
    q_pred = markley_quaternion_average(sigma_points_pred[:, :4], W)

    # Use weighted mean for angular velocity
    omega_pred = np.sum(W[:, None] * sigma_points_pred[:, 4:], axis=0)

    state_pred = np.concatenate((q_pred, omega_pred))

    P_pred = np.zeros((n, n))
    for i in range(2 * n + 1):
        delta = sigma_points_pred[i] - state_pred
        P_pred += W[i] * np.outer(delta, delta)

    return state_pred, P_pred


import matplotlib.pyplot as plt

# Simulate measurements
num_steps = 100
true_state = np.array([1, 0, 0, 0, 0.1, -0.1, 0.05])  # Initial true state
measurement_noise = np.diag([0.01, 0.01, 0.01, 0.001, 0.001, 0.001])

measurements = []
for _ in range(num_steps):
    true_measurement = measurement_function(true_state)
    noisy_measurement = true_measurement + np.random.multivariate_normal(np.zeros(6), measurement_noise)
    measurements.append(noisy_measurement)

measurements = np.array(measurements)

# Run the UKF
state_estimates = []
state = np.array([1, 0, 0, 0, 0.1, -0.1, 0.05])  # Initial estimated state
P = np.identity(7)  # Initial state covariance
Q = np.diag([1e-4, 1e-4, 1e-4, 1e-4, 1e-3, 1e-3, 1e-3])  # Process noise covariance matrix
R = np.diag([1e-2, 1e-2, 1e-2, 1e-4, 1e-4, 1e-4])  # Measurement noise covariance matrix
dt = 0.1  # Time step

for measurement in measurements:
    state_pred, P_pred = ukf_predict_with_markley(state, P, Q, dt)
    state, P = ukf_update(state_pred, P_pred, measurement, R)
    state_estimates.append(state)

state_estimates = np.array(state_estimates)

# Plot the results
time = np.arange(num_steps) * dt

plt.figure(figsize=(12, 8))

# Quaternion components
plt.subplot(2, 2, 1)
plt.plot(time, state_estimates[:, 0], label='qw')
plt.plot(time, state_estimates[:, 1], label='qx')
plt.plot(time, state_estimates[:, 2], label='qy')
plt.plot(time, state_estimates[:, 3], label='qz')
plt.xlabel('Time (s)')
plt.ylabel('Quaternion')
plt.legend()

# Angular velocity components
plt.subplot(2, 2, 2)
plt.plot(time, state_estimates[:, 4], label='ωx')
plt.plot(time, state_estimates[:, 5], label='ωy')
plt.plot(time, state_estimates[:, 6], label='ωz')
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (rad/s)')
plt.legend()

plt.tight_layout()
plt.show()
