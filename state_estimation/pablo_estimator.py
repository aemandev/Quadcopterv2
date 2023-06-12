import numpy as np
from state_estimation.quaternion_manifold import MUKF

def IMU_Measurement(q, w, na, ra, rw):
    RT = np.array([[-q[1]*q[1]-q[2]*q[2], q[0]*q[1]+q[2]*q[3], q[0]*q[1]-q[2]*q[3]],
                    [q[0]*q[1]-q[2]*q[3],  -q[0]*q[0]-q[2]*q[2],   q[1]*q[2]+q[0]*q[3]],
                    [q[0]*q[2]+q[1]*q[3],   q[1]*q[2]-q[0]*q[3],  -q[0]*q[0]-q[1]*q[1]]])
    RT = RT + RT + np.eye(3)
    a = np.random.normal(0 , na, size=(3,))
    am = np.dot(RT,(a + np.array([0, 0, 1]))) + np.random.normal(0, ra, size=(3,))
    wm = w + np.random.normal(0, rw, size=(3,))
    return am, wm


convergenceThreshold = 3*np.pi/180.0
maxConvergenceUpdates = 10000
Tsim = 1
dtdtsim = 10
Ntimes = 100
Qw = 1E0
Qa = 1E-2
Ra = 1E-4
Rw = 1E-4
ra = np.sqrt(Ra)
rw = np.sqrt(Rw)
RwIn = np.eye(3, 3) * Rw
RaIn = np.eye(3, 3) * Ra
q0 = np.random.normal( 0 , 1 , size=(4,))
q0 = q0/np.linalg.norm(q0)
w0 = np.array([0, 0, 0])
na = abs( np.random.normal( 0, np.sqrt(Qa)))


dt = .01
myEstimator = MUKF(RwIn, RaIn)
am, wm = IMU_Measurement(q0, w0, na, ra, rw)
myEstimator.updateIMU( am, wm, dt)