import sys
# sys.path.insert(0, r'C:\Users\kraft\Documents\GitHub\state_estimation\state_estimation')
import numpy as np
import scipy
from unscented_transform import unscented_transform
from Quaternion import Quaternion
from scipy.spatial.transform import Rotation

import copy
class UKF:
    def __init__(self, x0, P0, Q0, alpha, kappa, beta, fx, hx, x_mean_fn=None, z_mean_fn=None):
        self.x = x0
        self.num_states = len(x0)
        self.num_measure = 3
        self.P = P0
        self.Q = Q0
        self.x_prior = np.copy(self.x)
        self.P_prior = np.copy(self.P)
        self.R = np.eye(self.num_measure)

        self.alpha = alpha
        self.kappa = kappa
        self.beta = beta
        self.weights = self.generate_weights()
        self.sigmas = self.kraftSigmaPnts()
        self.num_sig = len(self.sigmas)
        self.fx = fx
        self.hx = hx
        self.x_mean = x_mean_fn
        self.z_mean = z_mean_fn

        self.sigmas_f = np.zeros((self.num_sig, self.num_states))
        self.sigmas_h = np.zeros((self.num_sig, self.num_measure)) # Change this to be  the number of measurements

        self.K = np.zeros((self.num_states, self.num_measure))  # Kalman gain
        self.y = np.zeros((self.num_measure))  # residual
        self.z = np.array([[None] * self.num_measure]).T  # measurement

        # these will always be a copy of x,P after predict() is called
        self.x_prior = self.x.copy()
        self.P_prior = self.P.copy()

        # these will always be a copy of x,P after update() is called
        self.x_post = self.x.copy()
        self.P_post = self.P.copy()

    def generate_weights(self):
        # Generates covariance and mean weights
        lambda_ = self.alpha ** 2 * (self.num_states + self.kappa) - self.num_states
        c = .5 / (self.num_states + lambda_)

        # Generate rows all with the value c
        self.Wc = np.full(2 * self.num_states+ 1, c)
        self.Wm = np.full(2 * self.num_states + 1, c)

        # Update the first value of the states
        self.Wc[0] = lambda_ / (self.num_states + lambda_) + (1 - self.alpha ** 2 + self.beta)
        self.Wm[0] = lambda_ / (self.num_states + lambda_)

    def generate_sigma_pts(self, x, P):
        lambda_ = self.alpha ** 2 * (self.num_states + self.kappa) - self.num_states
        U = np.linalg.cholesky((lambda_ + self.num_states) * P)
        sigmas = np.zeros((2 * self.num_states + 1, self.num_states))
        sigmas[0] = x
        for k in range(self.num_states):
            # pylint: disable=bad-whitespace
            sigmas[k + 1] = np.subtract(x, -U[k])
            sigmas[self.num_states + k + 1] = np.subtract(x, U[k])
        return sigmas

    def vec2quat(self, vec, dt=[]):
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

    def quat2vec(self, q):
        q = q/np.linalg.norm(q)
        angles = np.arccos(q[0])*2
        sins = np.sin(angles)
        vec = sins*q[1:4]/sins
        if np.any(np.isnan(vec)) | np.any(np.isinf(vec)):
            vec = np.array([0,0,0])
        return vec
    
    def kraftSigmaPnts(self):
        sigmas = np.zeros((2 * (self.num_states-1), self.num_states))
        s = np.linalg.cholesky(self.P+self.Q)

        # Do n-1 since n is 7x1 but we only want the vector components 6x1
        s = np.sqrt(2*(self.num_states-1))*s
        W = np.asarray(np.bmat([s,-s]))
        temp_quat = Quaternion(self.x[0:4])
        for k in range(2*(self.num_states-1)):
            quat_vec = Quaternion(self.vec2quat(W[0:3,k], dt=[]))
            quat_vec = temp_quat.quatProd(quat_vec.q)
            sigmas[k,0:4] = quat_vec
            sigmas[k, 4::] = W[3::,k]
        return sigmas


    def predict_full_state(self, dt, fx=None, **fx_args):
        """
        State: [p, v, q, w]
        """
        # calculate sigma points for given mean and covariance
        self.sigmas_f = self.compute_process_sigmas(dt, fx, **fx_args)
        # and pass sigmas through the unscented transform to compute prior
        quat_av = Quaternion(self.markleyAverage(self.sigmas_f[:,6:9]))
        w_av = np.mean(self.sigmas_f[:,9::],0)
        x_rot_out = np.concatenate((quat_av.q, w_av))
        x_pos_vel_out = np.mean(self.sigmas_f[:,0:6],0)

        w_W = self.sigmas_f[:,9::] - w_av
        quat_conj = quat_av.conj()
        r_W = np.zeros((12,3))
        for i, s in enumerate(self.sigmas_f):
            quat = Quaternion(s[6:9])
            r_W[i,0:3] = self.quat2vec(quat.quatProd(quat_conj))
        W_P = np.append(r_W, w_W, axis=1)
        self.W_P = W_P

        x_out = np.concatenate((x_pos_vel_out, x_rot_out))

        self.x = x_out
        self.P_prior = 1/(len(self.sigmas_f))*np.dot(W_P.T,W_P)
        self.x_prior = np.copy(x_out)



    def predict(self, dt, fx=None, **fx_args):

        # calculate sigma points for given mean and covariance
        self.sigmas_f = self.compute_process_sigmas(dt, fx, **fx_args)
        # and pass sigmas through the unscented transform to compute prior
        quat_av = Quaternion(self.markleyAverage(self.sigmas_f[:,0:4]))
        w_av = np.mean(self.sigmas_f[:,4::],0)
        x_out = np.concatenate((quat_av.q, w_av))

        w_W = self.sigmas_f[:,4::] - w_av
        quat_conj = quat_av.conj()
        r_W = np.zeros((self.num_sig,3))
        for i, s in enumerate(self.sigmas_f):
            quat = Quaternion(s[0:4])
            r_W[i,0:3] = self.quat2vec(quat.quatProd(quat_conj))
        W_P = np.append(r_W, w_W, axis=1)
        self.W_P = W_P
        self.x = x_out
        self.P_prior = 1/(len(self.sigmas_f))*np.dot(W_P.T,W_P)
        self.x_prior = np.copy(x_out)

    # def update(self, z, R=None, UT=None, hx=None, **hx_args):
    # Currentlly we require the updates at the same step but can be coded to be required at different intervals
    def update(self, z, z_gyro, R=None, hx=None, **hx_args):

        if z is None:
            self.z = np.array([[None] * self.num_measure]).T
            self.x_post = self.x.copy()
            self.P_post = self.P.copy()
            return


        if R is None:
            R = self.R
        elif np.isscalar(R):
            R = np.eye(self.num_measure) * R

        # Pass the sigma points through the measurement function
        sigmas_h = []
        for s in self.sigmas_f:
            sigmas_h.append(np.append(hx(s,**hx_args)[1::],s[4:7]))

        self.sigmas_h = np.atleast_2d(sigmas_h)

        z_ = np.mean(self.sigmas_h,axis=0)
        z = np.append(z,z_gyro)
        # Calculate innovation
        v = z-z_
        Wz = self.sigmas_h - z_
        Pzz = np.dot(Wz.T, Wz) / (len(self.sigmas_f))
        Pvv = Pzz + R

        # compute cross variance of the state and the measurements
        # Pxz = self.cross_variance(self.x, v, self.sigmas_f, self.sigmas_h)
        Pxz = np.dot(self.W_P.T,Wz)/(len(self.sigmas_f))

        # self.K = np.dot(Pxz, np.linalg.inv(Pvv))  # Kalman gain
        self.K = Pxz @ np.linalg.inv(Pvv)
        self.y = v.T  # residual


        # update Gaussian state estimate (x, P)
        Kv = np.dot(self.K, self.y)
        self.P = self.P_prior - np.dot(self.K, np.dot(Pvv, self.K.T))
        q_mult = Quaternion(self.x[0:4])
        self.x = np.append(q_mult.quatProd(self.vec2quat(Kv[0:3])), self.x[4::]+Kv[3::])

        # # update Gaussian state estimate (x, P)
        # self.x = self.x + np.dot(self.K, self.y)
        # self.P = self.P - np.dot(self.K, np.dot(self.S, self.K.T))

        # save measurement and posterior state
        self.z = copy.deepcopy(z)
        self.x_post = self.x.copy()
        self.P_post = self.P.copy()
        return z, z_


    def compute_process_sigmas(self, dt, fx=None, **fx_args):
        """
        computes the values of sigmas_f. Normally a user would not call
        this, but it is useful if you need to call update more than once
        between calls to predict (to update for multiple simultaneous
        measurements), so the sigmas correctly reflect the updated state
        x, P.
        """
        # calculate sigma points for given mean and covariance
        sigmas = self.kraftSigmaPnts()
        sigmas_f = np.zeros_like(self.sigmas)
        # Run sigma through the function fx
        for i, s in enumerate(sigmas):
            sigmas_f[i] = fx(s, dt, **fx_args)
        return sigmas_f

    def markleyAverage(self, Qmat):
        Q = Qmat
        # Number of quaternions to average
        M = Q.shape[0]
        A = np.zeros(shape=(4, 4))

        for i in range(0, M):
            q = Q[i, :]
            # multiply q with its transposed version q' and add A
            A = np.outer(q, q) + A

        # scale
        A = (1.0 / M) * A
        # compute eigenvalues and -vectors
        eigenValues, eigenVectors = np.linalg.eig(A)
        # Sort by largest eigenvalue
        eigenVectors = eigenVectors[:, eigenValues.argsort()[::-1]]
        # return the real part of the largest eigenvector (has only real part)
        return np.real(eigenVectors[:, 0])

