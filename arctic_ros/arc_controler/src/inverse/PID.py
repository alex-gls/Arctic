import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
import numpy as np

class PID:
    def __init__(self, Kp: float, Ki: float, Kd: float) -> None:
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.last_time = rospy.Time()

        self.rate = rospy.Rate(100)

        self.dt = 1

        self.error = 0.0
        self.integral = 0.00001
        self.prevErr = 0.0

        self.X = 0.0

        self.out = 0.0
        self.input = 0.0

        pass

    def main(self):
        rospy.Subscriber("/joy", Joy, self.callbackJoy, queue_size=1)

        while not rospy.is_shutdown():

            pub = rospy.Publisher('/X_input', Float32, queue_size=1)
            pub.publish(self.X)

            self.out = self.calculate(self.input, self.X)
            
            self.input = self.out
            
            pub1 = rospy.Publisher("/PID_out", Float32, queue_size=1)
            pub1.publish(self.input)

            self.rate.sleep()
        pass
    
    def callbackJoy(self, data):
        self.X = data.axes[0]

    def calculate(self, input: float, setpoint: float):

        self.error = np.around(setpoint - input, 2)
        self.integral = self.error * self.dt
        # D = (self.error - self.prevErr) / self.dt
        # self.prevErr = self.error
        return np.around((self.error * self.Kp + self.integral * self.Ki), 2)
    

# if __name__ == "__main__":
#     rospy.init_node("PID_node")
#     node = PID(0.01, 0.1, 0)
#     node.main()