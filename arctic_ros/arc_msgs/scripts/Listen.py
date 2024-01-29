#!/usr/bin/env python3
import rospy
from arc_msgs.msg import Gamepad

joystick = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
def callbackGame(data):
    global joystick
    joystick[0] = data.R3_x
    joystick[1] = data.R3_y
    joystick[2] = data.L3_x
    joystick[3] = data.L3_y
    joystick[4] = data.X
    joystick[5] = data.T
    joystick[6] = data.O
    joystick[7] = data.Sq
    joystick[8] = data.L1
    joystick[9] = data.R1

    print(joystick)

def main():
    rospy.Subscriber("/Game", Gamepad, queue_size=10, callback=callbackGame)
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("Listen_node")
    main()

