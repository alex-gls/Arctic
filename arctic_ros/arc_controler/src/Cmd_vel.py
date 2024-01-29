import rospy
from geometry_msgs.msg import Twist, PointStamped, Point
from sensor_msgs.msg import Joy
from std_msgs.msg import Header, Bool
from pid.PID import PID
import numpy as np

class Cmd_vel:
    def __init__(self) -> None:
        self.cmd_vel = Twist()
        self.xPID = PID(0.1, 0.2, 0)
        self.yPID = PID(0.1, 0.2, 0)
        self.axPID = PID(0.1, 0.2, 0)

        self.enable_move = Bool()

        self.pointStamped = PointStamped()
        pass

    def callbackJoy(self, data):
        if self.enable_move.data == True:
            pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
            pub_sphere = rospy.Publisher("/MoveSphere", PointStamped, queue_size=1)

            self.pointStamped.header = Header()
            self.pointStamped.header.seq = rospy.Time.now()
            self.pointStamped.header.frame_id = "root"
            
            self.pointStamped.point = Point()
            self.pointStamped.point.x = -0.02
            self.pointStamped.point.y = 0
            self.pointStamped.point.z = -0.15

            self.max = 4

            if (data.axes[7] == 0.0):
                self.cmd_vel.linear.x = self.xPID.calculate(self.cmd_vel.linear.x, data.axes[1] * self.max)
            else: 
                self.cmd_vel.linear.x = self.xPID.calculate(self.cmd_vel.linear.x, data.axes[7] * self.max)
            
            if (data.axes[6] == 0.0):
                self.cmd_vel.linear.y = self.yPID.calculate(self.cmd_vel.linear.y, data.axes[0] * self.max * 0.5)
            else: 
                self.cmd_vel.linear.y = self.yPID.calculate(self.cmd_vel.linear.y, data.axes[6] * self.max * 0.5)

            # self.cmd_vel.linear.x = self.map(self.cmd_vel.linear.x, -1, 1, -self.max, self.max)
            # self.cmd_vel.linear.y = self.map(self.cmd_vel.linear.y, -1, 1, -self.max, self.max)
            self.cmd_vel.angular.z = self.axPID.calculate(self.cmd_vel.angular.z, data.axes[3] * self.max * 0.5)

            self.pointStamped.point.x += self.cmd_vel.linear.x / 100
            self.pointStamped.point.y += self.cmd_vel.linear.y / 100

            # print(self.cmd_vel.linear.y)

            pub_cmd.publish(self.cmd_vel)
            pub_sphere.publish(self.pointStamped)

            #---------------------------------------------------------------------------------------
    
    def map(self, x: float, in_min: float, in_max: float, out_min: float, out_max: float) -> float:
        return np.around((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min, 2)
    
    def callbackEnable(self, data: Bool):
        self.enable_move = data
        rospy.loginfo(self.enable_move)
        pass


    def main(self):
        rospy.Subscriber("/joy", Joy, self.callbackJoy, queue_size=1)
        rospy.Subscriber("/enable_move", Bool, self.callbackEnable, queue_size=1)
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node("Cmd_vel_node")
    node = Cmd_vel()
    node.main()