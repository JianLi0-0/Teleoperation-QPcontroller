from omni_msgs.msg import OmniButtonEvent
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float64MultiArray
import rospy
import time
class teleoperation_info():
    def __init__(self, robot):
        self.robot = robot
        self.white_button = 0
        self.touch_pose = PoseStamped()
        self.ref_touch = (0., 0., 0.)
        self.ref_ur5 = (0., 0., 0.)
        self.eef_position = (0., 0., 0.)

        self.gazebo_vel_cmd_pub = rospy.Publisher('/arm_controller/command', Float64MultiArray, queue_size=10)
        self.script_vel_cmd_pub = rospy.Publisher('/ur_hardware_interface/script_command', String, queue_size=10)
        self.real_vel_cmd_pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=10)

        rospy.Subscriber("/phantom/button", OmniButtonEvent, self.touch_button_callback)
        rospy.Subscriber("/touch/stylus_pose", PoseStamped, self.touch_pose_callback)

    def touch_button_callback(self, msg):
        self.white_button = msg.white_button
        if self.white_button is 1:
            self.ref_touch = (self.touch_pose.pose.position.x, self.touch_pose.pose.position.y, self.touch_pose.pose.position.z)
            self.robot.update_pose()
            self.ref_ur5 = (self.robot.position[0,0], self.robot.position[1,0], self.robot.position[2,0])
            print("white button is pressed")
            print("ref touch: "+str(self.ref_touch))
            print("ref ur5: "+str(self.ref_ur5))

    def touch_pose_callback(self, msg):
        self.touch_pose = msg


