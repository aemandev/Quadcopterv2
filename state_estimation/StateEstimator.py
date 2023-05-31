from state_estimation.UKF import UKF
import csv
import numpy as np
import pandas as pd
import math


def normalizeQuat(q):
    mag = (q[0] ** 2 + q[1] ** 2 + q[2] ** 2 + q[3] ** 2) ** 0.5
    return q / mag
def q_rot_body_to_inertial(q):
    C_body_to_inertial = np.array([[q[0] ** 2 + q[1] ** 2 - q[2] ** 2 - q[3] ** 2, 2 * (
    q[1] * q[2] - q[0] * q[3]), 2 * (q[1] * q[3] + q[0] * q[2])],
                                   [2 * (q[1] * q[2] + q[0] * q[3]),
                                    q[0] ** 2 - q[1] ** 2 + q[2] ** 2 - q[3] ** 2,
                                    2 * (q[2] * q[3] - q[0] * q[1])],
                                   [2 * (q[1] * q[3] - q[0] * q[2]),
                                    2 * (q[2] * q[3] + q[0] * q[1]),
                                    q[0] ** 2 - q[1] ** 2 - q[2] ** 2 + q[3] ** 2]])
    return C_body_to_inertial

def acc_measurement_function(q_in):
    # Make negative if necessary (rotating inertial acc. ref to body frame)
    acc_ref = np.array([[2 * (q_in[1] * q_in[3] - q_in[0] * q_in[2])],
                        [2 * (q_in[2] * q_in[3] + q_in[0] * q_in[1])],
                        [(q_in[0] ** 2 - q_in[1] ** 2 - q_in[2] ** 2 + q_in[3] ** 2)]])
    return np.concatenate([np.array([0]),acc_ref[:,0]])

def mag_measurement_function(quat):
    mag_sf = np.array([23.0142, 22.4117, 5.2315])
    mag_sf[2] = 0
    mag_sf = mag_sf / (mag_sf[0] ** 2 + mag_sf[1] ** 2) ** 0.5
    mag_ref_est = np.matmul(q_rot_body_to_inertial(quat).transpose(), mag_sf)

    return np.concatenate([np.array([0]),mag_ref_est])


def iterate_x(x, dt, inputs):
    '''this function is based on the x_dot and can be nonlinear as needed'''
    # Inputs = wx, wy, wz
    wx = inputs[0]
    wy = inputs[1]
    wz = inputs[2]

    ret = np.zeros(len(x))
    ret[0] = x[0] - (dt/2)*(-x[1]) - (dt/2)*(-x[2]) - (dt/2)*(-x[3]) + (dt/2)*(-x[1]*wx-x[2]*wy-x[3]*wz)
    ret[1] = x[1] - (dt / 2) * (x[0]) - (dt / 2) * (-x[3]) - (dt / 2) * (x[2]) + (dt/2)*(x[0]*wx-x[3]*wy+x[2]*wz)
    ret[2] = x[2] - (dt / 2) * (x[3]) - (dt / 2) * (x[0]) - (dt / 2) * (-x[1]) + (dt/2)*(x[3]*wx+x[0]*wy-x[1]*wz)
    ret[3] = x[2] - (dt / 2) * (-x[2]) - (dt / 2) * (x[1]) - (dt / 2) * (x[0]) + (dt/2)*(-x[2]*wx+x[1]*wy+x[0]*wz)
    ret[0:4] = normalizeQuat(ret[0:4])
    ret[4] = x[4]
    ret[5] = x[5]
    ret[6] = x[6]

    return ret


def main():

    # Process Noise
    q = np.eye(7)
    q[0][0] = 0.0001
    q[1][1] = 0.0001
    q[2][2] = 0.0001
    q[3][3] = 0.0001
    q[3][3] = 0.0025
    q[4][4] = 0.0025
    q[5][5] = 0.0025

    # create measurement noise covariance matrices
    r_imu = np.zeros([2, 2])
    r_imu[0][0] = 0.01
    r_imu[1][1] = 0.03

    r_compass = np.zeros([1, 1])
    r_compass[0][0] = 0.02

    alpha = 0.04
    K = 0
    beta = 2.0
    data = pd.read_csv('flights.csv')
    with open('flights.csv', 'r') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)
        i = 0
        for row in reader:
            if i == 0:
                quaternion = np.array([1, 0, 0, 0])  # Initial estimate of the quaternion
                bias = np.array([0, 0, 0])
                x0 = np.concatenate((quaternion, bias)).transpose()
                # pass all the parameters into the UKF!
                # number of state variables, process noise, initial state, initial coariance, three tuning paramters, and the iterate function
                state_estimator = UKF(np.identity(7) * 0.001, x0, 0.01 * np.eye(7), alpha, K, beta, iterate_x)
                last_time = 0
                i = 1
                continue

            cur_time = np.asarray(float(row[1]))
            dt = cur_time-last_time
            true_att = np.asarray([float(row[x]) for x in [12, 9, 10, 11]])
            gyro = np.asarray([float(row[x]) for x in [16, 17, 18]])
            accel_measure = np.asarray([float(row[x]) for x in [19, 20, 21]])
            accel_measure[2] = -accel_measure[2]

            last_time = cur_time

            state_estimator.predict(dt, gyro)
            # accel_measure = 9.81 * np.array([.3, .5, .812403])
            accel_mag = (accel_measure[0] ** 2 + accel_measure[1] ** 2 + accel_measure[2] ** 2) ** 0.5
            acc_data = accel_measure / accel_mag
            r_acc = .1 * np.eye(4)
            states_acc = state_estimator.update([0, 1, 2, 3], acc_data, r_acc, update_eq=acc_measurement_function)
            a = 5
            # Turn mag measurment to inertial, take out the z component, normalize, turn back to body
            # mag_measurement = np.array([23.0142, 22.4117, 5.2315])
            # mag_measurement = np.atleast_2d(mag_measurement)
            # mag = np.matmul(q_rot_body_to_inertial(state_estimator.x), mag_measurement.T)
            # mag[2] = 0
            # mag = mag / (mag[0] ** 2 + mag[1] ** 2) ** 0.5
            # mag_reading_B = np.matmul(q_rot_body_to_inertial(state_estimator.x).transpose(), mag)
            # mag_reading_B = mag_reading_B.reshape(3, )
            # r_mag = .005 * np.eye(4)
            # states_mag = state_estimator.update([0, 1, 2, 3], mag_reading_B, r_mag, update_eq=mag_measurement_function)
            #
            # print("--------------------------------------------------------")
            # print("Real state: ", real_state)
            # print("Estimated state: ", state_estimator.get_state())
            # print("Difference: ", real_state - state_estimator.get_state())

if __name__ == "__main__":
    main()