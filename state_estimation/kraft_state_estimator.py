import numpy as np
import filterpy
from filterpy.kalman import MerweScaledSigmaPoints, UnscentedKalmanFilter
from filterpy.common import Q_discrete_white_noise
from Quaternion import Quaternion
# I want to import the module "UKF_general" from the "state_estimation" folder
import sys
sys.path.append('../')
# sys.path.insert(0, r'C:\Users\kraft\Documents\GitHub\state_estimation\state_estimation')
from UKF_general import UKF
import pandas as pd
import csv
import math
from plotly.subplots import make_subplots
import plotly.graph_objects as go
import pandas as pd
import csv
import math
from plotly.subplots import make_subplots
import plotly.graph_objects as go




def euler_from_quaternion(q):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    w,x,y,z = q
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return np.rad2deg(roll_x), np.rad2deg(pitch_y), np.rad2deg(yaw_z)  # in radians


def vec2quat(vec, dt=[]):
        if dt:
            a = np.linalg.norm(vec) * dt
            e = vec / np.linalg.norm(vec) * np.sin(a / 2)
            q_new = np.array([np.cos(a / 2), e[0], e[1], e[2]])
        else:
            if np.any(vec):
                a = np.linalg.norm(vec)
                e = vec / np.linalg.norm(vec) * np.sin(a / 2)
                q_new = np.array([np.cos(a / 2), e[0], e[1], e[2]])
            else:
                q_new = np.array([1, 0, 0, 0])
        q_new[np.isnan(q_new)] = 0
        return q_new / np.linalg.norm(q_new)

def fx(x, dt, w):
    # Does not even have to be the same dim as x0
    q = Quaternion(x[0:4])

    # w_q = w__k[0:3]
    # w_w = w__k[3:7]

    # Calculate angle and axis
    q_delta = vec2quat(w, dt)

    q_disturbed = Quaternion(q.quatProd(q_delta))
    w_disturbed = x[4::] + w


    x_k1 = np.concatenate((q_disturbed.q, w_disturbed))
    return x_k1

def H_gyro(x, v):
    # v: Measurement noise of the angular velocity
    w_k = x[5:7]
    z_rot = w_k + v
    return z_rot

def H_accel(x, v):
    q = Quaternion(x[0:4]) 
    g = np.array([0,0,1]) + v   
    # Concatenate a 0 to the beginning of the the g vector
    g_prime = Quaternion(np.concatenate([np.array([0]),g]))
    return q.quatProd(g_prime.quatProd(q.conj()))


def H_mag(x, b, v):
    q = Quaternion(x[0:4]) 
    g = b + v 
    g_prime = Quaternion(np.concatenate([np.array([0]),g]))

    return q.quatProd(g_prime.quatProd(q.conj()))


# def H_mag(x, b, v):
#     q = x[0:4]
#     g_prime = qut_rot(q, b)
#     z_mag = g_prime + v
#     return z_mag

def hx(qin,w):
    qin = Quaternion(qin)
    g = Quaternion(np.array([0,0,0,1]))
    q_out= qin.quatProd(g.quatProd(qin.conj()))
    return np.concatenate((q_out.q[1:4],w))

# q must always be a unit quaternion
# This means that the first 4 componentso f the state vector are n o longer independent
# Causes conflicts with concept of Kalman Filter and how now is treated
q0 = np.array([1, 0, 0, 0])
w0 = np.array([0, 0, 0])
x0 = np.concatenate((q0, w0))

# These work but I cant say if the update step is doing much
Q0 = np.diag(np.concatenate([np.array([.01,.01,.01])*1E-3,np.array([1,1,1])*1E10]))
R = np.diag(np.concatenate([np.array([.1,.1,.1])*1E1,np.array([1,1,1])*1E-11]))
R_mag = np.diag(np.concatenate([np.array([.1,.1,.1])*1E-2,np.array([1,1,1])*1E-11]))

# Q0 = np.diag(np.concatenate([np.array([1,50,1])*1E-8,np.array([1,1,1])*1E-6]))
# R = np.diag(np.concatenate([np.array([1E-5,1,1])*1E3,np.array([1,1,1])*1E-11]))
# R_mag = np.diag(np.concatenate([np.array([1,1,1])*1E5,np.array([1,1,1])*1]))
# Q0 = np.identity(6) * 0.001
P0 = np.eye(6)*1E0
alpha = 0.04
K = 0
beta = 2.0


# UKF(np.identity(7) * 0.001, x0, 0.01 * np.eye(7), alpha, K, beta, iterate_x)
# --------------------READY FOR MESUREMENT UPDATE CODE-------------------

# Initial process noise vector. Vector has 6 states because there are only 6 degrees of freedom because
# the quaternion states are normalized. Will need to convert w_q into a quaternion to add it to the quat est
w_q = np.array([0.1, 0.1, 0.1])
w_w = np.array([0.1, 0.1, 0.1])
w__k = np.vstack((w_q, w_w))

# Process noise covariance matrix, Q_q
# Q_q = np.zeros(3,3)

# Need 3 measurement models -> zrot, zacc, zmag
# Since w_k is already part of the prediction step...

# data = pd.read_csv('flights.csv')
true_mat = np.array([0,0,0])
approx_att = np.array([0,0,0])
t_mat = np.array([0])

data = pd.read_csv('TStick_Test08_Trial3.csv')



#  # measurements
# for z in zs:
#     kf.predict()
#     kf.update(z)
#     print(kf.x, 'log-likelihood', kf.log_likelihood)

acc_save = np.array([0,0,9.81])
w_stack = np.array([0,0,0])
with open('TStick_Test08_Trial3.csv') as csvfile:
    reader = csv.reader(csvfile)
    next(reader)
    next(reader)
    i = 0
    for row in reader:
        data = [float(x) for x in row[0].split(';')]
        if i == 0:
            # Write code to split row into a list of ints
            quaternion = np.array([data[1], data[2], data[3], data[4]])  # Initial estimate of the quaternion
            bias = np.array([data[8],data[9],data[10]])
            mag_0 = np.asarray([data[11],data[12],data[13]])
            x0 = np.concatenate((quaternion, bias)).transpose()
            # pass all the parameters into the UKF!
            # number of state variables, process noise, initial state, initial coariance, three tuning paramters, and the iterate function
            state_estimator = UKF(x0, P0, Q0, alpha, K, beta, fx, H_accel, x_mean_fn=None, z_mean_fn=None)
            last_time = 0
            i = 1
            continue

        cur_time = np.asarray(data[0])
        t_mat = np.vstack([t_mat,cur_time])
        dt = cur_time-last_time
        true_att = np.asarray([data[1], data[2], data[3], data[4]])
        gyro = np.asarray([data[8],data[9],data[10]])
        accel_measure = np.asarray([data[5],data[6],data[7]])
        mag_measure = np.asarray([data[11],data[12],data[13]])
        # Take the norm of the magnetometer measurement
        # mag_measure = mag_measure / np.linalg.norm(mag_measure)            

        acc_save = np.vstack([acc_save,accel_measure])
        w_stack = np.vstack([w_stack,gyro])
        accel_mag = (accel_measure[0] ** 2 + accel_measure[1] ** 2 + accel_measure[2] ** 2) ** 0.5
        acc_data = accel_measure / accel_mag
        last_time = cur_time

        # Predict
        state_estimator.predict(dt, fx=fx, w=gyro)
        # Update
        state_estimator.update(z=acc_data, z_gyro=gyro, R=R, hx=H_accel, v=np.array([0,0,0]))
        # Update mag
        state_estimator.update(z=mag_measure, z_gyro=gyro, R=R_mag, hx=H_mag, b = mag_0, v=np.array([0,0,0]))


        true_r, true_p, true_y = euler_from_quaternion(true_att)
        true_mat = np.vstack([true_mat,np.array([true_r,true_p,true_y])])
        [r, p, y] = euler_from_quaternion(state_estimator.x[0:4])
        approx_att = np.vstack([approx_att, np.array([r, p, y])])
        if len(true_mat) > 1338:
            break


fig = make_subplots(
    rows=3, cols=1,
    subplot_titles=("Roll", "Pitch", "Yaw"), shared_xaxes=True)
fig.add_trace(go.Scatter(mode='lines+markers',x=t_mat[:,0],y=true_mat[:,0],name='Roll Real',marker=dict(color='red',size=4),
                         legendgroup='Roll',legendgrouptitle=dict(text='Roll')),row=1,col=1)
fig.add_trace(go.Scatter(mode='lines+markers',x=t_mat[:,0],y=approx_att[:,0],name='Roll Est', marker=dict(color='black'),
legendgroup='Roll',legendgrouptitle=dict(text='Roll')),row=1,col=1)

# Label x axis time and y axis roll
fig.update_xaxes(title_text="Time (s)", row=1, col=1)

fig.add_trace(go.Scatter(mode='lines+markers',x=t_mat[:,0],y=true_mat[:,1],name='Pitch Real', marker=dict(color='red',size=4),
                         legendgroup='Pitch',legendgrouptitle=dict(text='Pitch')),row=2,col=1)
fig.add_trace(go.Scatter(mode='lines+markers',x=t_mat[:,0],y=approx_att[:,1],name='Pitch Est', marker=dict(color='black'),
                         legendgroup='Pitch',legendgrouptitle=dict(text='Pitch')),row=2,col=1)
# Label x axis time and y axis pitch
fig.update_xaxes(title_text="Time (s)", row=2, col=1)

fig.add_trace(go.Scatter(mode='lines+markers',x=t_mat[:,0],y=true_mat[:,2],name='Yaw Real', marker=dict(color='red',size=4),
                         legendgroup='Yaw',legendgrouptitle=dict(text='Yaw')),row=3,col=1)
fig.add_trace(go.Scatter(mode='lines+markers',x=t_mat[:,0],y=approx_att[:,2],name='Yaw Est', marker=dict(color='black'),
                         legendgroup='Yaw',legendgrouptitle=dict(text='Yaw')),row=3,col=1)
# Label x axis time and y axis yaw
fig.update_xaxes(title_text="Time (s)", row=3, col=1)
fig.show()


# # create a new plotly figure subplot with two rows. Name the plots "Accel Data" and "Gyro Data"
# fig = make_subplots(
#     rows=2, cols=1,
#     subplot_titles=("Accel Data", "Gyro Data"), shared_xaxes=True)

# # plot accel data vs time
# fig.add_trace(go.Scatter(mode='lines+markers',x=t_mat[:,0],y=acc_save[:,0],name='Accel X',marker=dict(color='red',size=4),
#                             legendgroup='Accel',legendgrouptitle=dict(text='Accel')),row=1,col=1)
# fig.add_trace(go.Scatter(mode='lines+markers',x=t_mat[:,0],y=acc_save[:,1],name='Accel Y',marker=dict(color='green',size=4),
#                             legendgroup='Accel',legendgrouptitle=dict(text='Accel')),row=1,col=1)
# fig.add_trace(go.Scatter(mode='lines+markers',x=t_mat[:,0],y=acc_save[:,2],name='Accel Z',marker=dict(color='blue',size=4),
#                             legendgroup='Accel',legendgrouptitle=dict(text='Accel')),row=1,col=1)

# # Do the same for gyro data
# fig.add_trace(go.Scatter(mode='lines+markers',x=t_mat[:,0],y=w_stack[:,0],name='Gyro X',marker=dict(color='red',size=4),
#                             legendgroup='Gyro',legendgrouptitle=dict(text='Gyro')),row=2,col=1)
# fig.add_trace(go.Scatter(mode='lines+markers',x=t_mat[:,0],y=w_stack[:,1],name='Gyro Y',marker=dict(color='green',size=4),
#                             legendgroup='Gyro',legendgrouptitle=dict(text='Gyro')),row=2,col=1)
# # Label the axes
# fig.update_xaxes(title_text="Time (s)", row=1, col=1)
# fig.update_xaxes(title_text="Time (s)", row=2, col=1)
# fig.update_yaxes(title_text="Accel (m/s^2)", row=1, col=1)
# fig.update_yaxes(title_text="Gyro (rad/s)", row=2, col=1)
# fig.show()
