#!/usr/bin/env python3
import rospy
from arc_msgs.msg import Vector, Gamepad

angle = 0
def main():
    global angle
    
    while not rospy.is_shutdown():
        vec = Vector()
        game = Gamepad()
        game.R3_x = 1.0
        #---------Settings----------  
        vec.name = "Arctic"
        if angle > 6.28: angle = 0
        angle += 0.01
        vec.angle = angle
        vec.power = 0.8
        # --------------------------
        pub = rospy.Publisher("/vec", Vector, queue_size=10)
        pub2 = rospy.Publisher("/Game", Gamepad, queue_size=10)
        pub.publish(vec)
        pub2.publish(game)
        rospy.sleep(0.01)

if __name__ == "__main__":
    rospy.init_node("Talker_node")
    main()

