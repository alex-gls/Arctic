#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import numpy as np
import math

class Control():
    def __init__(self) -> None:
        # Value for dynamixels
        self.value_matrix = np.full((12), 0.0)
        self.matrix = np.array([1, 1, 1, -1, -1, -1, -1, 1, 1, 1, -1, -1])

        self.publish_list = []
        self.topics_list = ["/Arctic/foot_1_/command",
                            "/Arctic/foot_2_/command",
                            "/Arctic/foot_3_/command",
                            "/Arctic/foot_4_/command",
                            "/Arctic/foot_5_/command",
                            "/Arctic/foot_6_/command",
                            "/Arctic/foot_7_/command",
                            "/Arctic/foot_8_/command",
                            "/Arctic/foot_9_/command",
                            "/Arctic/foot_10_/command",
                            "/Arctic/foot_11_/command",
                            "/Arctic/foot_12_/command",
                            ]
        pass
    
    def sin(self, val):
        val = val + 0.1
        return math.sin(val)
        pass

    def multi_matrix(self, list: list, matrix: list) -> list:
        matrix = np.multiply(list, matrix)
        return matrix
    
    def main(self):
        self.init_topic()
        while not rospy.is_shutdown():        
            self.message_topics(self.value_matrix)
            rospy.sleep(self.time)
        pass
    
    def init_topic(self):
        for i, item in enumerate(self.topics_list):
            self.publish_list.append(rospy.Publisher(item, Float64, queue_size = 10))
        pass

    def message_topics(self, list: list):
        # for i, item in enumerate(self.publish_list):
        #     item.publish(list[i])
        pass

if __name__ == "__main__":
    rospy.init_node('control_lane')
    node = Control()
    node.main()
