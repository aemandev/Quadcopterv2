import numpy as np
from abc import ABC


class MUKF:

    def __init__(self, RwIn, RaIn):
        self.q0 = np.array([0, 0, 0, 1])
        self.e = np.array([0, 0, 0])
        self.q = np.array([0, 0, 0, 1])
        self.w = np.array([0, 0, 0])
        self.P = 1e2 * np.eye(6)
        self.P[3, 3] = 1e-16
        self.Qw = 1.0e0 * np.eye(3, 3)
        self.Qa = 1.0e-2 * np.eye(3, 3)
        self.Rw = np.eye(3) * 1e-3
        self.Ra = np.eye(3) * 1e-3

    def get_q(self):
        q = self.q
        return q

    def fM2C(self,qm, q):
        delta = np.array([qm[4] * q[1:3] - q[4] * qm[1:3] - np.cross(qm[1:3], q[1:3]),
                          np.matmul(qm.T, q)])
        if delta[4] < 0:
            delta = -delta

        e = 4 * delta[1:3] / (1 + delta[4])
        return e

    def fC2M(self,qm, e):
        enorm = np.linalg.norm(e)
        if enorm > 4:
            e = 4 * e / enorm

        delta = np.array([[8 * e], [16 - e.T * e]]) / (16 + e.T * e)
        q = np.array([qm[4] * delta[1:3] + delta[4] * qm[1:3] + np.cross(qm[1:3], delta[1:3]),
                      qm[4] * delta[4] - qm[1:3].T * delta[1:3]])
        return q

    def updateIMU(self, am, wm, dt):

        # we define the extended covariance matrix
        Pe = np.bmat([[self.P, np.zeros((6, 3)), np.zeros((6, 3))],
                       [np.zeros((3, 6)), self.Qw * dt, np.zeros((3, 3))],
                       [np.zeros((3, 6)), np.zeros((3, 3)), self.Qa]])
        Pe = np.asarray(Pe)

        # Cholesky factorization
        W0 = 1.0 / 25.0  # [0,1]
        Wi = (1.0 - W0) / (2 * 12)  # size[Pe] = 12x12
        alpha = 1.0 / np.sqrt(2.0 * Wi)
        Pe = alpha * np.linalg.cholesky(Pe)

        # we set up the sigma points
        X = np.zeros((13, 25))
        Xp = np.zeros((13, 25))
        Yp = np.zeros((6, 25))
        # first we propagate the mean value
        X[:, 0] = np.asarray(np.bmat([[self.q], [self.w], [np.zeros((6,))]]))
        Xp[:, 0] = self.statePrediction(X[:, 0], dt)
        Yp[:, 0] = self.IMU_MeasurementPrediction(Xp[:, 0])
        # self.q0 = self.q self.e = 0 # with this we can test if the Chart update is good or not
        # second we generate the sigma points from the P matrix
        for j in range(12):
            # first, the +sigma point
            #  we do this because P is expressed in the q0 chart, but we need
            #  to express it in the q chart for the next time step
            X[:, j + 1] = np.array([self.fC2M(self.q0, self.e + Pe[1:3, j]),  self.w + Pe[4:6, j],  Pe[7:12, j]])
            Xp[:, j + 1] = self.statePrediction(X[:, j + 1], dt)
            # we make sure that all quaternion are in the same semisphere
            if np.transpose(Xp[1:4, j + 1]) * Xp[1:4, 1] < 0:
                Xp[1:4, j + 1] = -Xp[1:4, j + 1]

            Yp[:, j + 1] = self.IMU_MeasurementPrediction(Xp[:, j + 1])
            # second, the -sigma point
            #  we do this because P is expressed in the q0 chart, but we need
            #  to express it in the q chart for the next time step
            X[:, j + 13] = [self.fC2M(self.q0, self.e - Pe[1:3, j]), self.w - Pe[4:6, j] - Pe[7:12, j]]
            Xp[:, j + 13] = self.statePrediction(X[:, j + 13], dt)
            # we make sure that all quaternion are in the same semisphere
            if Xp[1:4, j + 13].T*Xp[1:4,1] < 0:
                Xp[1:4, j + 13] = -Xp[1:4, j + 13]

            Yp[:, j + 13] = self.IMU_MeasurementPrediction[Xp[:, j + 13]]

        # we compute the means
        xpm = W0 * Xp[:, 1] + Wi * sum(Xp[:, 2::], 2)
        xpm[1:4] = xpm[1:4] / np.linalg.norm(xpm[1:4])
        ypm = W0 * Yp[:, 1] + Wi * sum(Yp[:, 2::], 2)

        # matrices we will need
        dX = np.array([self.fM2C(xpm[1:4], Xp[1:4, 1]), Xp[5:7, 1] - xpm[5:7]])
        dY = Yp[:, 1] - ypm
        Pxx = W0 * np.array([dX * dX.T])
        Pxy = W0 * np.array([dX * dY.T])
        Pyy = W0 * np.array([dY * dY.T])
        # now we end calculating the matrices
        for j in range(1, 24):
            dX = np.array([self.fM2C(xpm[1:4], Xp[1:4, j]), Xp[5:7, j] - xpm[5:7]])
            dY = Yp[:, j] - ypm
            Pxx = Pxx + Wi * np.array([dX * dX.T])
            Pxy = Pxy + Wi * np.array([dX * dY.T])
            Pyy = Pyy + Wi * np.array([dY * dY.T])

        # now we can compute the gain [ K*Pyy = Pxy ]
        K = Pxy / Pyy

        # and update the state in the chart
        d = K * np.array([np.array([am, wm]) - ypm])

        self.q0 = xpm[1:4]
        self.e = d[1:3]

        # the updated point in the chart is mapped to a quaternion
        self.q = self.fC2M(xpm[1:4], self.e)
        # and the angular velocity is updated in the usual way
        self.w = xpm[5:7] + d[4:6]

        # the covariance matrix is updated in the chart centered in self.q0
        self.P = Pxx - K * Pyy * K


        # we avoid numerical instabilities
        self.q = self.q / np.linalg.norm(self.q)
        self.P = 0.5 * (self.P + self.P.T)

        # this covariance matrix is expressed in the xpm[1:4] chart [self.q0]
        # we will have to update it to the new self.q chart
        # that is why we do what we do at the beginning

        # def: statePrediction
        # this method predicts the state given the previous state, and the time increment
        # inputs:
        #  x: previous state [q,w,n,a]
        #  dt: time step
        # outputs:
        #  xp: predicted state [qp,wp,np,ap]


    def statePrediction(self, x, dt):
        # first we predict the angular velocity
        wp = x[4:7] + x[7:10]
        # w norm computation
        wnorm = np.linalg.norm(wp)
        # we compute qw
        if wnorm != 0.0:
            wdt05 = 0.5 * wnorm * dt
            swdt = np.sin(wdt05) / wnorm
            qw = np.array([wp * swdt, np.cos(wdt05)])
        else:
            qw = np.array([0, 0, 0, 1])

        # we compute the predicted state [q*qw,w,n,a]
        xp = np.asarray(np.bmat([[x[3] * qw[0:3] + qw[3] * x[0:3] + np.cross(x[0:3], qw[0:3])],
                                 [x[3] * qw[3] - x[0:3].T * qw[0:3]],
                                 [wp],
                                 [x[7:11]]]))
        return xp

        # def: IMU_MeasurementPrediction


    # this method predicts the measurement given a state
    # inputs:
    #  xp: state for which the measure is to be predicted
    # outputs:
    #  yp: predicted measurement
    def IMU_MeasurementPrediction(self, xp):
        # we build the transposed rotation matrix
        RT = np.array([[-xp[1] * xp[1] - xp[2] * xp[2],   xp[0] * xp[1] + xp[2] * xp[3],   xp[0] * xp[2] - xp[1] * xp[3]],
              [xp[0] * xp[1] - xp[2] * xp[3], -xp[0] * xp[0] - xp[2] * xp[2],   xp[1] * xp[2] + xp[0] * xp[3]],
              [xp[0] * xp[2] + xp[1] * xp[3],   xp[1] * xp[2] - xp[0] * xp[3], -xp[0] * xp[0] - xp[1] * xp[1]]])
        RT = RT + RT + np.eye(3)
        # we compute the predicted measurement
        yp = np.concatenate([np.dot(RT, xp[10:13] + np.array([0, 0, 1])), xp[4:7]])
        return yp


