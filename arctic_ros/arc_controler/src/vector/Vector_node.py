#!/usr/bin/env python3
import rospy
from arc_msgs.msg import Vector
from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion
from std_msgs.msg import Header
from sensor_msgs.msg import Joy
from math import * 

class Vector_node():
    def __init__(self) -> None:

        self.x = 0
        self.y = 0
        self.z = 1

        self.rx = 0
        self.ry = 0
        self.rz = 0
        self.rw = 1

        self.len = 1
        
        pass

    def main(self):         
        rospy.Subscriber("/joy", Joy, self.callback_joystick, queue_size=10)
        rospy.spin()
        pass

    
    def callback_joystick(self, data):
        rospy.loginfo("vec") 
        vec = Vector()
        vec.header = Header()
        vec.header.stamp =  rospy.Time.now()
        
        vec.L_angle, vec.L_power = angle_vec(-data.axes[0], -data.axes[1])
        vec.R_angle, vec.R_power = angle_vec(-data.axes[3], -data.axes[4])
        
        pub = rospy.Publisher("/vector", Vector, queue_size=1)
        pub.publish(vec)

        self.send_Pose(vec)
        
    def send_Pose(self, vec):
        
        x, y, z = 0, 0, 0.075

        # ---------------------------Left stick---------------------------
        pubL = rospy.Publisher('/Vector_Pose_L', PoseStamped, queue_size=10)

        poseStamped = PoseStamped()

        poseStamped.header = Header()
        poseStamped.header.stamp = rospy.Time.now()
        poseStamped.header.frame_id = "root"

        poseStamped.pose = Pose()
        
        poseStamped.pose.position = Point()
        poseStamped.pose.position.x = x
        poseStamped.pose.position.y = y
        poseStamped.pose.position.z = z

        poseStamped.pose.orientation = Quaternion()
        poseStamped.pose.orientation.w = cos(-vec.L_angle/2)
        poseStamped.pose.orientation.x = 0
        poseStamped.pose.orientation.y = 0
        poseStamped.pose.orientation.z = sin(-vec.L_angle/2)

        pubL.publish(poseStamped)

        # ---------------------------Right stick---------------------------
        pubR = rospy.Publisher('/Vector_Pose_R', PoseStamped, queue_size=10)

        poseStamped = PoseStamped()

        poseStamped.header = Header()
        poseStamped.header.stamp = rospy.Time.now()
        poseStamped.header.frame_id = "root"

        poseStamped.pose = Pose()
        
        poseStamped.pose.position = Point()
        poseStamped.pose.position.x = x
        poseStamped.pose.position.y = y
        poseStamped.pose.position.z = z

        poseStamped.pose.orientation = Quaternion()
        poseStamped.pose.orientation.w = cos(-vec.R_angle/2)
        poseStamped.pose.orientation.x = 0
        poseStamped.pose.orientation.y = 0
        poseStamped.pose.orientation.z = sin(-vec.R_angle/2)

        pubR.publish(poseStamped)
    
def angle_vec(dx: float, dy: float):
    
    dy *= -1
    leng = sqrt(dx**2 + dy**2)
    
    if dx > 0 and dy == 0.0: angle = 1.57
    elif dx < 0 and dy == 0.0: angle = -1.57
    elif dy < 0 and dx == 0: angle = 3.14
    elif dy < 0 and dx > 0: angle = pi - atan(dx / -dy)
    elif dy < 0 and dx < 0: angle = - pi - atan(dx / -dy)
    elif dx != 0 and dy != 0: angle = atan(dx / dy)
    else: angle = 0

    angle = round(angle, 2)
    leng = round(leng, 2)

    if leng > 1: leng = 1

    return angle, leng


if __name__ == "__main__":
    rospy.init_node('vector_node')
    node = Vector_node()
    node.main()
    pass