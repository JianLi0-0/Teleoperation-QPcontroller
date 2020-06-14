import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
import numpy as np
from tf import transformations

class robot:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('qp_ros_interface',
                        anonymous=True)
        group_name = "manipulator"
        robot_commander = moveit_commander.RobotCommander()
        group = moveit_commander.MoveGroupCommander(group_name)

        self.robot_commander = robot_commander
        self.group = group
        self.pose = self.group.get_current_pose().pose
        self.position = np.array([self.pose.position.x, self.pose.position.y, self.pose.position.z])
        self.quaternion = np.array([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])
        self.joint_status = self.group.get_current_joint_values()
        robot = URDF.from_parameter_server()
        self.kdl_kin = KDLKinematics(robot, "base_link", "ee_link")
        self.jacobian = self.kdl_kin.jacobian(self.joint_status)
 
    def update_joint_status(self):
        self.joint_status = self.group.get_current_joint_values()

    def update_jacobian(self):
        self.joint_status = self.group.get_current_joint_values()
        self.jacobian = self.kdl_kin.jacobian(self.joint_status)

    def update_pose(self):
        self.pose = self.group.get_current_pose().pose
        self.position = np.array([[self.pose.position.x], 
                                    [self.pose.position.y], 
                                    [self.pose.position.z]])
        self.quaternion = (self.pose.orientation.x, 
                            self.pose.orientation.y, 
                            self.pose.orientation.z, 
                            self.pose.orientation.w)

    def hat(self, x):
        x_hat = np.array([[0.,   -x[2],  x[1]],
                          [x[2],    0., -x[0]],
                          [-x[1], x[0],    0.]])
        return x_hat

    def q2R(self, quaternion):
        R = transformations.quaternion_matrix(quaternion)
        return R

    def cal_dJ_dq(self, Fs):
        dJ_dq = np.zeros((1, 6))
        dq = 0.05
        for i in range(6):
            joint_status_tmp = self.joint_status
            joint_status_tmp[i] += dq
            jacobian_new = self.kdl_kin.jacobian(joint_status_tmp)
            Fs_new = -0.5*np.linalg.det(np.dot(jacobian_new.T, jacobian_new))
            dJ_dq[0,i] = (Fs_new - Fs)/dq
        return dJ_dq

    def update_states(self):
        # self.update_joint_status()
        self.update_jacobian()
        self.update_pose()
        