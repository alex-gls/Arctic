import rospy
from geometry_msgs.msg import Twist

import tf
from tf import transformations

import numpy as np

class Twist_node():
    def __init__(self) -> None:

        self.linear = np.array([0.0, 0.0, 0.0])
        self.angular = np.array([0.0, 0.0, 0.0])
        pass

    def main(self):
        rospy.Subscriber("/cmd_vel", Twist)
        pass

if __name__ == "__main__":
    rospy.init_node("twist_node")
    node = Twist_node()
    node.main()