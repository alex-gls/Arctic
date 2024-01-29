#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Vector3, Point, Pose, Quaternion
from std_msgs.msg import Header, Bool
import numpy as np
from math import asin, acos, sin, cos

import tf
from tf import transformations

class Rviz():
    def __init__(self) -> None:
        self.mat = np.array([1, 1, 1, -1, -1, -1, -1, 1, 1, 1, -1, -1]) # array for inverse something joint
        
        self.global_pos_robot = Pose()
        self.cmd = Twist()

        self.cmd_vel = Twist()
        self.move = Vector3()

        self.enable_move = Bool()

        pass

    def main(self):
        rospy.Subscriber("/global_pos_robot", Pose, self.callbackPos, queue_size=1)
        self.rate = rospy.Rate(45)
        while not rospy.is_shutdown():
            self.movingRobot()
        # rospy.spin()
            self.rate.sleep()

    def callbackPos(self, data: Pose):
        self.global_pos_robot = data


    def jointstate(self, matrix_angle):
        """
        The method sends an array of data to the jointstate for rviz 
        """
        mat = np.array([-1, -1, -1, -1, 1, 1, 1, -1, -1, 1, 1, 1])
        position = mat * matrix_angle

        pub = rospy.Publisher("joint_states", JointState, queue_size=1)

        js = JointState()
        js.header = Header()
        js.header.stamp = rospy.Time.now()
        js.header.frame_id = "root"
        js.name = ['foot_1_', 'foot_2_', 'foot_3_', 'foot_4_', 'foot_5_', 'foot_6_', 'foot_7_', 'foot_8_', 'foot_9_', 'foot_10_', 'foot_11_', 'foot_12_']

        js.position = position
        js.velocity = []
        js.effort = []
        pub.publish(js)
        pass

    def movingRobot(self):
       
        self.global_pos_robot.position.x /= 100
        self.global_pos_robot.position.y /= 100
        self.global_pos_robot.position.z /= 100

        br = tf.TransformBroadcaster()
        br.sendTransform((self.global_pos_robot.position.x, self.global_pos_robot.position.y, self.global_pos_robot.position.z), 
                         transformations.quaternion_from_euler(self.global_pos_robot.orientation.x, self.global_pos_robot.orientation.y, self.global_pos_robot.orientation.z), rospy.Time.now(), "root", "dummy")
        # br.sendTransform(((self.global_pos_robot.position.x), (self.global_pos_robot.position.y), (self.global_pos_robot.position.z)), 
        # transformations.quaternion_from_euler(x, y, z), rospy.Time.now(), "root", "dummy")

if __name__ == "__main__":
    rospy.init_node("Rviz_node")
    node = Rviz()
    node.main()