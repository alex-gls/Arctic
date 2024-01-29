import rospy
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header

class DrawPoint():
    def __init__(self) -> None:
        pass

    def main(self):
        pub = rospy.Publisher("center_robot", PointStamped, queue_size=10)

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            pointstamped = PointStamped()

            pointstamped.header = Header()
            pointstamped.header.stamp = rospy.Time.now()
            pointstamped.header.frame_id = "root"

            pointstamped.point = Point()
            pointstamped.point.x = -0.02
            pointstamped.point.y = 0
            pointstamped.point.z = -0.15
            
            pub.publish(pointstamped)

            rate.sleep()
        pass

if __name__ == "__main__":
    rospy.init_node("draw_center")
    node = DrawPoint()
    node.main()