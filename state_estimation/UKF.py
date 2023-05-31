import numpy as np
import scipy
from copy import deepcopy
from state_estimation.Quaternion import *
from state_estimation.kraft_state_estimator import vec2quat
class UKF:
    def __init__(self, Q, x0, P0, alpha, K, beta, state_transition_function):
        self.n_dim = len(x0)
        self.Q = Q  # Process noise
        self.x = x0  # Initial states
        self.P = P0  # Initial state covariance
        self.alpha = alpha  # Spread of UKF covariance
        self.K = K  # UKF kalman gain
        self.beta = beta  # UKF tuning parameter
        self.state_transition = \
            state_transition_function  # State transition function

        self.num_sig = 1 + len(self.x) * 2  # Number of sigma points

        self.lam = self.alpha**2 * (self.K + len(self.x)) - len(self.x)

        self.covar_weights = np.zeros(self.num_sig)
        self.mean_weights = np.zeros(self.num_sig)

        self.covar_weights[0] = (self.lam / (self.n_dim + self.lam)) + (1 - pow(self.alpha, 2) + self.beta)
        self.mean_weights[0] = (self.lam / (self.n_dim + self.lam))


        self.sigma_pts = self.generate_sigma_pnts()


    def generate_sigma_pnts(self):
        """
        * generates sigma points of size (n_states)x(num_sigma_pts)
        * See https://fjp.at/blog/ukf
        """
        ret = np.zeros((self.num_sig, self.n_dim))

        tmp_mat = (self.n_dim + self.lam) * self.P

        # print spr_mat
        spr_mat = scipy.linalg.sqrtm(tmp_mat)

        ret[0] = self.x
        for i in range(self.n_dim):
            ret[i + 1] = self.x + spr_mat[i]
            ret[i + 1 + self.n_dim] = self.x - spr_mat[i]

        return ret.T

    def kraftSigmaPnts(self):
        s = np.linalg.cholesky(self.P+self.Q)

        # Do n-1 since n is 7x1 but we only want the vector components 6x1
        s = np.sqrt(2*(self.n_dim-1)*s)
        W = np.assarray(np.bmat([s,s]))
        # X[:,0:3] = vec2quat(W[0:2,:])
        # X[:,4:6] = []
        # X[:,0:3] = []

    def predict(self, dt, w):
        """
        performs a prediction step
        :param timestep: float, amount of time since last prediction
        """

        # Run each sigma point through the state transition update
        sigmas_out = np.array([self.state_transition(x, dt,w) for x in self.sigma_pts.T]).T

        x_out = np.zeros(self.n_dim)

        # for each variable in X
        # For each state, basically take weighted average over all sigma points :)
        for i in range(self.n_dim):
            # the mean of that variable is the sum of
            # the weighted values of that variable for each iterated sigma point
            x_out[i] = sum((self.mean_weights[j] * sigmas_out[i][j] for j in range(self.num_sig)))

        p_out = np.zeros((self.n_dim, self.n_dim))
        # for each sigma point
        for i in range(self.num_sig):
            # take the distance from the mean
            # make it a covariance by multiplying by the transpose
            # weight it using the calculated weighting factor
            # and sum
            # Take sigma points that have been run through the state transition equations. Subtract out
            # the current intermediate x estimate (which is currently the weighted average of all sigma points)
            diff = sigmas_out.T[i] - x_out
            diff = np.atleast_2d(diff)
            # P out would be equivalent to dx**2 multiplied by the covariance weights. Dot product is a
            # 7x7 matrix
            # Doing the sum of the weighted covariances (sum of weighted square difference)
            # That is to say, a higher covariance would mean that we are penalyzing a state for being
            # off from the predicted state
            p_out += self.covar_weights[i] * np.dot(diff.T, diff)

        # add process noise
        p_out += dt * self.Q

        # Update sigma points after running them through the state transition
        self.sigma_pts = sigmas_out
        # Update predicted x before the measurement
        self.x = x_out
        # Update p; the state covariance
        self.P = p_out

    def update(self, states, data, r_matrix, update_eq):
        """
        performs a measurement update
        :param states: list of indices (zero-indexed) of which states were measured, that is, which are being updated
        :param data: list of the data corresponding to the values in states
        :param r_matrix: error matrix for the data, again corresponding to the values in states
        """

        num_states = len(states)

        # Create row of sigma points length of len(self.sigma_pts)
        # Each row will represent the concatenation of all sigmas from a particular state
        sigmas_split = np.split(self.sigma_pts, self.n_dim)

        # Basically just creating a y vector where we are stacking all the sigma points into column
        # vectors len(states) by len(self.sigma_pts). The idea is to isolate only the sigma states we want
        # for a measurement update
        y = np.concatenate([sigmas_split[i] for i in states])
        z = np.array([update_eq(x) for x in y.T]).T
        if len(z.shape)>2:
            z = z[0,:,:]
        z_mean = np.mean(z,1)
        # Split each state estimate into it's own array index
        x_split = np.split(self.x, self.n_dim)

        # Concat indices back together that you want
        y_mean = np.concatenate([x_split[i] for i in states])
################################################################################################
        # differences in y from y mean
        y_diff = deepcopy(y)
        x_diff = deepcopy(self.sigma_pts)
        for i in range(self.num_sig):
            for j in range(num_states):
                # Subtract the mean (current state estimate at this point) from each sigma point
                y_diff[j][i] -= y_mean[j]
            for j in range(self.n_dim):
                # From each sigma point., subtract off the state
                x_diff[j][i] -= self.x[j]

        # covariance of measurement
        p_yy = np.zeros((num_states, num_states))
        for i, val in enumerate(np.array_split(y_diff, self.num_sig, 1)):
            p_yy += self.covar_weights[i] * val.dot(val.T)

        # add measurement noise
        p_yy += r_matrix

        # covariance of measurement with states
        p_xy = np.zeros((self.n_dim, num_states))
        for i, val in enumerate(zip(np.array_split(y_diff, self.num_sig, 1), np.array_split(x_diff, self.num_sig, 1))):
            p_xy += self.covar_weights[i] * val[1].dot(val[0].T)

        k = np.dot(p_xy, np.linalg.inv(p_yy))

        y_actual = data

        self.x += np.dot(k, (np.concatenate([np.array([0]),y_actual]) - z_mean))
        self.P -= np.dot(k, np.dot(p_yy, k.T))
        self.sigmas = self.generate_sigma_pnts()


