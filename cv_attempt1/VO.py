import cv2
import numpy as np
import os

class VO:
    def __init__(self, K):
        self.K = K
        self.K_inv = np.linalg.inv(K)
        # detector=cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True
        # self.detector = cv2.FastFeatureDetector_create(
        #     threshold=25, nonmaxSuppression=True
        # )
        self.num_features = 100
        self.detector = cv2.ORB_create(self.num_features)
        # self.detector = cv2.goodFeaturesToTrack()
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.dist = np.array(
            [0.171537e00, -2.875925e-01, -1.874744e-03, -3.985333e-05, 6.887928e-02]
        )
        self.lk_params = dict(
            winSize=(21, 21),
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
        )
        self.R = np.eye(3)
        self.t = np.zeros((3, 1))
        self.prev_img = None
        self.prev_kp = None
        self.prev_des = None
        self.prev_pts = None
        self.curr_img = None
        self.curr_kp = None
        self.curr_des = None
        self.curr_pts = None
        self.good_matches = None
        self.good_prev_pts = None
        self.good_curr_pts = None
        self.E = None
        self.Rt = None
        self.initiliazed = False
        self.keyImg = None
        self.st = None
        self.frame_num = 0
        self.good_new = None
        self.good_old = None
        self.color = np.random.randint(0, 255, (100, 3))
        self.angles = np.array([0, 0, 0])
        self.matches = []
        self.reinitialized = False

    def match(self):
        if not self.reinitialized:
            matches =  self.matcher.match(self.prev_des, self.curr_des)
            matches = sorted(matches, key=lambda x: x.distance)
            top_matches = matches
            self.matches = top_matches
            self.curr_pts = np.float32([self.curr_pts[m.trainIdx] for m in top_matches]).reshape(-1, 1, 2)
            self.curr_des = np.float32([self.curr_des[m.trainIdx] for m in top_matches])


    def debug_frame(self):
        # draw the tracks
        mask = np.zeros_like(self.curr_img)
        frame = self.curr_img
        for i, (new, old) in enumerate(zip(self.good_new[0:100], self.good_old[0:100])):
            a, b = new.ravel()
            c, d = old.ravel()
            mask = cv2.line(
                mask, (int(a), int(b)), (int(c), int(d)), self.color[i].tolist(), 2
            )
            frame = cv2.circle(frame, (int(a), int(b)), 5, self.color[i].tolist(), -1)
        self.keyImg = cv2.add(frame, mask)

    def detect(self):
        # feature_params = dict( maxCorners = 100,
        #                 qualityLevel = 0.3,
        #                 minDistance = 7,
        #                 blockSize = 7 )
        # self.curr_img  = cv2.cvtColor(self.curr_img, cv2.COLOR_BGR2GRAY)
        # self.curr_pts = cv2.goodFeaturesToTrack(self.curr_img, mask = None, **feature_params)
        if self.initiliazed:
            points_needed = self.num_features -len(self.curr_pts)
            self.detector = cv2.ORB_create(points_needed)
        points, des = self.detector.detectAndCompute(self.curr_img, None)
        new_points = np.array([x.pt for x in points], dtype=np.float32).reshape(
            -1, 1, 2
        )
        if not new_points.size ==0 :
            if self.initiliazed:
                cat_points = np.concatenate((self.curr_pts, new_points), axis=0)
                cat_des = np.concatenate((self.curr_des, des), axis=0)
                self.curr_des = cat_des
                self.curr_pts = cat_points
                
            else:
                self.curr_des = des
                self.curr_pts = new_points  
            

    def flow(self):
        # self.prev_pts = self.curr_pts
        if not self.reinitialized:
            self.curr_pts, st, err = cv2.calcOpticalFlowPyrLK(
                self.prev_img, self.curr_img, self.prev_pts, None, **self.lk_params
            )
            self.good_new = self.curr_pts[st == 1]
            self.good_old = self.prev_pts[st == 1]
            self.prev_pts = self.good_new.reshape(-1, 1, 2)
            self.curr_des = self.curr_des[st[:, 0]==1]
            self.debug_frame()
        else:
            self.reinitialized = False

    def estimate_R_t_initial(self):
        # self.detect()
        self.flow()
        # self.keyImg = cv2.drawKeypoints(self.curr_img, self.curr_kp, None, color=(255,0,0))
        a = 5

    def run_vo(self):
        if len(self.good_new) < self.num_features - 50:
            self.reinitialized = True
            self.detect()
            self.match()
        self.flow()
        try:
            self.E, mask = cv2.findEssentialMat(
                self.good_new, self.good_old, self.K, cv2.RANSAC, 0.999, 1.0, None
            )
        except:
            self.detect()
            self.match()
            self.flow()
            return
        try:
            points, R, t, mask = cv2.recoverPose(
                self.E, self.good_new, self.good_old, self.K
            )
        except:
            self.detect()
            self.match()
            return
        # Convert R to euler angles using 3-2-1 parameterization
        self.R = np.matmul(self.R, R)
        self.angles = self.rotation_matrix_to_euler_angles()
        self.t = self.t + self.R.dot(t)

    def rotation_matrix_to_euler_angles(self):
        # Extract the rotation matrix elements
        R = self.R
        r11, r12, r13 = R[0]
        r21, r22, r23 = R[1]
        r31, r32, r33 = R[2]

        # Calculate theta_x (around x-axis)
        theta_x = np.arctan2(r32, r33)

        # Calculate theta_y (around y-axis)
        sin_theta_y = -r31
        cos_theta_y = np.sqrt(r11 ** 2 + r21 ** 2)
        theta_y = np.arctan2(sin_theta_y, cos_theta_y)

        # Calculate theta_z (around z-axis)
        sin_theta_z = r21
        cos_theta_z = r11
        theta_z = np.arctan2(sin_theta_z, cos_theta_z)

        # Return the euler angles in degrees
        return np.rad2deg([theta_x, theta_y, theta_z])

    def process_fame(self, img):
        self.curr_img = img
        if self.initiliazed:
            self.run_vo()
            self.prev_img = self.curr_img
            # self.prev_kp = self.curr_kp
            # self.prev_des = self.curr_des
            # self.prev_pts = self.curr_pts
            self.prev_des = self.curr_des
        else:
            self.detect()
            self.prev_img = self.curr_img
            # self.prev_kp = self.curr_kp
            # self.prev_des = self.curr_des
            self.prev_pts = self.curr_pts
            self.prev_des = self.curr_des
            self.frame_num += 1
            if self.frame_num > 1:
                self.estimate_R_t_initial()
                self.match()
                self.initiliazed = True
