import math
from cvxopt import solvers, matrix, spmatrix
import numpy as np

def qpSolver(P, q, G=None, h=None, A=None, b=None, initvals=None):
    if G is None and h is None and A is None and b is None:
        args = [matrix(P), matrix(q)]
    elif A is None and b is None:
        args = [matrix(P), matrix(q), matrix(G), matrix(h)]
    else:
        args = [matrix(P), matrix(q), matrix(G), matrix(h), matrix(A), matrix(b)]

    solvers.options['show_progress'] = False

    try:
        sol = solvers.qp(*args)
    except ValueError:
        print("QP is infeasible")
        return -1
    if 'optimal' not in sol['status']:
        print("QP fails, the status are: %s", sol)
        return -1
    jointVel = np.array(sol['x']).reshape((q.shape[0], 1))

    return jointVel

class controller():
    def __init__(self, robot):
        self.robot = robot
        self.Q = np.identity(6)
        self.position_k = 3
        self.orientation_k = np.ones((2, 1))*1.5
        self.sigularity_k = 0.1
        self.position_b = 0.0
        self.orientation_b = np.ones((2, 1))*math.cos(0./180.0*math.pi)
        self.sigularity_b = -0.5
        self.Fp = 0
        self.Fo = np.zeros((2, 1))
        self.Fs = 0
        self.dFp_dq = np.zeros((1, 6))
        self.dFo_dq = np.zeros((2, 6))
        self.dFs_dq = np.zeros((1, 6))
        self.dFp_dt = 0
        self.dFo_dt = np.zeros((2, 1))
        self.dFs_dt = 0
        self.desired_pose = 0

    def update_task_state(self, desired_pose):
        desired_position = np.array([[desired_pose.pose.position.x], 
                                    [desired_pose.pose.position.y], 
                                    [desired_pose.pose.position.z]])
        desired_quaternion = (desired_pose.pose.orientation.x, 
                               desired_pose.pose.orientation.y, 
                               desired_pose.pose.orientation.z, 
                               desired_pose.pose.orientation.w)
        self.robot.update_states()
        # position task
        p_error = self.robot.position-desired_position
        self.dFp_dq = 2*np.dot(p_error.T, self.robot.jacobian[0:3,:])
        self.Fp = np.dot(p_error.T, p_error)
        # orietation task
        R_d = self.robot.q2R(desired_quaternion)
        x_d = R_d[0:3, 0]
        y_d = R_d[0:3, 1]
        R = self.robot.q2R(self.robot.quaternion)
        x = R[0:3, 0]
        y = R[0:3, 1]
        x_hat = self.robot.hat(x)
        y_hat = self.robot.hat(y)
        self.dFo_dq[0, :] = np.dot(x_d.T, -np.dot(x_hat, self.robot.jacobian[3:6,:]))
        self.Fo[0,0] = np.dot(x.T, x_d)
        self.dFo_dq[1, :] = np.dot(y_d.T, -np.dot(y_hat, self.robot.jacobian[3:6,:]))
        self.Fo[1,0] = np.dot(y.T, y_d)
        # sigularities avoidance task
        self.Fs = -0.5*np.linalg.det(np.dot(self.robot.jacobian.T, self.robot.jacobian))
        self.dFs_dq = self.robot.cal_dJ_dq(self.Fs)


    def calcualte_output(self, desired_pose):
        self.update_task_state(desired_pose)
        P = self.Q
        q = np.zeros((6, 1))
        A = self.dFp_dq
        b = self.position_k*(self.position_b-self.Fp) - self.dFp_dt
        '''G = self.dFo_dq
        h = self.orientation_k*(self.orientation_b-self.Fo) - self.dFo_dt'''
        G = np.row_stack((self.dFo_dq, self.dFs_dq))
        h = np.row_stack((self.orientation_k*(self.orientation_b-self.Fo) - self.dFo_dt, 
                        self.sigularity_k*(self.sigularity_b-self.Fs) - self.dFs_dt))
        return qpSolver(P, q, -G, -h, A, b)
        


