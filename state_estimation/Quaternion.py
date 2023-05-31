import numpy as np

class Quaternion:
    def __init__(self, q0):
        self.q = q0
    def conj(self):
        return np.array([self.q[0], -self.q[1], -self.q[2], -self.q[3]])

    # def quatProd(self, q2):
    #     """
    #     Performs quaternion product in the scalar-first format
    #     :return:
    #     """
    #     q1 = self.q
    #
    #
    #     # Eventually make quaternion into a class and make this a method
    #     # MAY NEED TO MAKES SURE THIS IS CORRECT TOO
    #     qmat = np.array([
    #         [q1[0], -q1[1], -q1[2], -q1[3]],
    #         [q1[1], q1[0], -q1[3], q1[2]],
    #         [q1[2], q1[3], q1[0], -q1[1]],
    #         [q1[3], -q1[2], q1[1], q1[0]]
    #     ])
    #     q_out = np.matmul(qmat, q2)
    #     return q_out

    def quatProd(self, q2):
        """
        Multiply two quaternions.

        Parameters:
        q1 (list): a list representing the first quaternion, in the format [w, x, y, z].
        q2 (list): a list representing the second quaternion, in the format [w, x, y, z].

        Returns:
        A list representing the result quaternion, in the format [w, x, y, z].

        """
        q1 = self.q
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2

        return [w, x, y, z]

    def vec2quat(vec, dt=[]):
        if dt:
            a = np.linalg.norm(vec) * dt
            e = vec / np.linalg.norm(vec) * np.sin(a / 2)
        else:
            a = np.linalg.norm(vec)
            e = vec / np.linalg.norm(vec) * np.sin(a / 2)
        q_new = np.array([np.cos(a / 2), e[0], e[1], e[2]])
        return q_new / np.linalg.norm(q_new)

    def quat_inv(self):

        return np.array([q0, -q[1:4]]) / np.linalg.norm(q)

    def qut_rot(self, r):
        """
        Rotation vector by quaternion
        :param r:
        :return:
        """
        return quat_prod(q, quat_prod(np.concatenate((0, r)), quat_inv(q)))