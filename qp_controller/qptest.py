import robot
import numpy as np
import controller
import touch
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import rospy
import tf

ur5_manipulator = robot.robot()
controller = controller.controller(ur5_manipulator)
tele_info = touch.teleoperation_info(ur5_manipulator)

desired_pose = PoseStamped()
desired_pose.pose = ur5_manipulator.pose
joint_vel = Float64MultiArray()
last_joint_vel = Float64MultiArray()
frequency = 125
r = rospy.Rate(frequency)

while rospy.is_shutdown() is False:

    if tele_info.white_button is 1:
        desired_pose.pose.position.x = tele_info.ref_ur5[0] + (tele_info.touch_pose.pose.position.x - tele_info.ref_touch[0])
        desired_pose.pose.position.y = tele_info.ref_ur5[1] + (tele_info.touch_pose.pose.position.y - tele_info.ref_touch[1])
        desired_pose.pose.position.z = tele_info.ref_ur5[2] + (tele_info.touch_pose.pose.position.z - tele_info.ref_touch[2])
        desired_pose.pose.orientation = tele_info.touch_pose.pose.orientation

    cal_vel = controller.calcualte_output(desired_pose)
    if cal_vel is -1:
        rospy.logerr("joint_vel error"+str(cal_vel))
    else:
        joint_vel.data = cal_vel
        str_joint_vel = "speedj([" + str(float(joint_vel.data[0])) + "," \
                        + str(float(joint_vel.data[1])) + "," \
                        + str(float(joint_vel.data[2])) + "," \
                        + str(float(joint_vel.data[3])) + "," \
                        + str(float(joint_vel.data[4])) + "," \
                        + str(float(joint_vel.data[5])) + "],2.0)"
        # str_joint_vel = "speedj([" + str(float(joint_vel.data[0])) + "," \
        #                 + str(float(joint_vel.data[1])) + "," \
        #                 + str(float(joint_vel.data[2])) + "," \
        #                 + str(float(joint_vel.data[3])) + "," \
        #                 + str(float(joint_vel.data[4])) + "," \
        #                 + str(float(joint_vel.data[5])) + "]," \
        #                 + str(5.0) + "," + str(1.0/frequency) + ")"

    # print("joint_vel"+str(joint_vel.data))
    tele_info.gazebo_vel_cmd_pub.publish(joint_vel)
    # tele_info.script_vel_cmd_pub.publish(str_joint_vel)
    tele_info.real_vel_cmd_pub.publish(joint_vel)
    last_joint_vel.data = joint_vel.data
    r.sleep()
rospy.spin()
