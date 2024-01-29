#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header, Float32MultiArray, Float64, Bool
from sensor_msgs.msg import Joy, Imu
from geometry_msgs.msg import PolygonStamped, Polygon, Point32, Twist, Point, PointStamped, Quaternion, Pose, Point, PoseArray, PoseStamped
from jsk_footstep_msgs.msg import FootstepArray, Footstep
from nav_msgs.msg import Path

import numpy as np
from math import *

from control.Control_foot import Control
from dxl.DynamixelTTL import MX_28AT
from inverse.Inverse_Kinematics import Kinematics
from pid.PID import PID

import tf
from tf import transformations

from vector.Vector_node import angle_vec

from rviz.Rviz_sender import Rviz

class Main_node():
    def __init__(self) -> None:

        # DYNAMIXEL
        self.enable_dynamixel = False
        if self.enable_dynamixel:
            self.DXL = MX_28AT()
            self.DXL.Connect()

        self.IK = Kinematics()

        self.speed = 100

        self.matrix_angle = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

        self.modes = [False for i in range(8)]

        self.L3_x = 0.0
        self.L3_y = 0.0
        self.R3_x = 0.0
        self.R3_y = 0.0

        self.vec_L = [0, 0]
        self.vec_R = [0, 0]

        self.step = 0
        self.stand_i = 0


        #Gait
        self.gait_trigger = False
        self.gait_tick = 0
        self.gait_step = 0
        self.gait_start = False
        self.gait_start_end = False
        self.gait_start_tick = True
        self.gait_FR_tick = 0
        self.gait_FR = False
        self.gait_FL_tick = 0
        self.gait_FL = False

        self.gaitFR = Point(0, 0, 0)
        self.gaitFL = Point(0, 0, 0)
        self.gaitBR = Point(0, 0, 0)
        self.gaitBL = Point(0, 0, 0)

        self.take_point = False

        #Trot
        self.trot_tick = 0
        self.trot_start = False
        self.trot_step = 0
        self.trot_end = False
        self.trot_move = Point(0, 0, 15)

        self.trot_dx_move = 0
        self.trot_dy_move = 0
        self.tror_take = False

        self.trotFR = Point(0, 0, 0)
        self.trotFL = Point(0, 0, 0)
        self.trotBR = Point(0, 0, 0)
        self.trotBL = Point(0, 0, 0)

        # self.dx1, self.dy1, self.dz1 = 0, 0, 0
        # self.dx2, self.dy2, self.dz2 = 0, 0, 0
        # self.dx3, self.dy3, self.dz3 = 0, 0, 0
        # self.dx4, self.dy4, self.dz4 = 0, 0, 0
        
        self.dy1_base = 0
        self.dy2_base = 0
        self.dy3_base = 0
        self.dy4_base = 0

        self.dx1_base = 0
        self.dx2_base = 0
        self.dx3_base = 0
        self.dx4_base = 0

        self.trot_FR_tick = 0
        self.trot_FL_tick = 0
        self.trot_BR_tick = 0
        self.trot_BL_tick = 0

        self.poseFR_0 = Pose()
        self.poseFL_0 = Pose()
        self.poseBR_0 = Pose()
        self.poseBL_0 = Pose()

        self.trot_pointFR = Point()
        self.trot_pointFL = Point()
        self.trot_pointBR = Point()
        self.trot_pointBL = Point()

        #March
        self.march_tick = 0
        self.march_change = False
        self.enable_march = False
        self.enable_move = False

        self.button_press = [False for i in range(13)]
        self.button_release = [True for i in range(13)]

        self.USE_IMU = False
        self.stabilization = False

        self.cmd_vel = Twist()
        self.sphere_pos = PointStamped()

        self.rviz = Rviz()

        self.PID_x = PID(0.02, 0.05, 0)
        self.PID_y = PID(0.02, 0.05, 0)
        self.PID_z = PID(0.02, 0.05, 0)

        self.PID_roll = PID(0.01, 0.1, 0)
        self.PID_pitch = PID(0.01, 0.1, 0)

        self.PID_xstab = PID(0.01, 0.01, 0)
        self.PID_ystab = PID(0.01, 0.01, 0)

        self.posesFR = PoseArray()
        self.posesFR.header = Header()
        self.posesFR.header.frame_id = "root"
        self.posesFR.header.seq = rospy.Time.now()

        self.posesFL = PoseArray()
        self.posesFL.header = Header()
        self.posesFL.header.frame_id = "root"
        self.posesFL.header.seq = rospy.Time.now()

        self.posesBR = PoseArray()
        self.posesBR.header = Header()
        self.posesBR.header.frame_id = "root"
        self.posesBR.header.seq = rospy.Time.now()

        self.posesBL = PoseArray()
        self.posesBL.header = Header()
        self.posesBL.header.frame_id = "root"
        self.posesBL.header.seq = rospy.Time.now()

        self.save_global_pos = Pose()

        # PID LEGs
        # FR
        self.P, self.I, self.D = 0.05, 0.2, 0
        self.PID_FRx = PID(self.P, self.I, self.D)
        self.PID_FRy = PID(self.P, self.I, self.D)
        self.PID_FRz = PID(self.P, self.I, self.D)

        # FL
        self.PID_FLx = PID(self.P, self.I, self.D)
        self.PID_FLy = PID(self.P, self.I, self.D)
        self.PID_FLz = PID(self.P, self.I, self.D)

        # BR
        self.PID_BRx = PID(self.P, self.I, self.D)
        self.PID_BRy = PID(self.P, self.I, self.D)
        self.PID_BRz = PID(self.P, self.I, self.D)
        
        # BL
        self.PID_BLx = PID(self.P, self.I, self.D)
        self.PID_BLy = PID(self.P, self.I, self.D)
        self.PID_BLz = PID(self.P, self.I, self.D)

        self.dx = 0
        self.dy = 0
        self.dz = 0

        self.pitch = 0
        self.roll = 0

        self.imu_dx = 0.0
        self.imu_dy = 0.0

        self.ex = 0
        self.ey = 0

        self.acc_rx = 0.0
        self.acc_ry = 0.0

        self.pointFR = np.array([])
        self.pointFL = np.array([])
        self.pointBR = np.array([])
        self.pointBL = np.array([])

        self.FootprintFR = None
        self.FootprintFL = None
        self.FootprintBR = None
        self.FootprintBL = None

        self.global_pos_robot = Pose()

        print("........................Ready........................")
        pass

    def main(self):   
        self.rate = rospy.Rate(self.speed)
        rospy.Subscriber("/joy", Joy, self.callback_joystick, queue_size=1)
        rospy.Subscriber("/arctic_imu/base_link_orientation", Imu, self.imu, queue_size=1)
        rospy.Subscriber("/cmd_vel", Twist, self.callbackCmdVel, queue_size=1)

        rospy.Subscriber("/FootprintFR", PoseArray, self.callbackFootprintFR, queue_size=1)
        rospy.Subscriber("/FootprintFL", PoseArray, self.callbackFootprintFL, queue_size=1)
        rospy.Subscriber("/FootprintBR", PoseArray, self.callbackFootprintBR, queue_size=1)
        rospy.Subscriber("/FootprintBL", PoseArray, self.callbackFootprintBL, queue_size=1)
        
        # for i in range(31):
        #     dx, dy, dz = self.IK.moveLegEllipce(Point(5, 0, 1), i, 30, 5)
        #     print(f"{dx}, {dy}, {dz}")

        while not rospy.is_shutdown():
            # try:

                if self.modes[0]:
                    self.moving()
                    self.stabilization = False
            
                if self.modes[1]:
                    self.rotating()
                    self.stabilization = False

                if self.modes[2]:
                    self.stand()

                if self.modes[3]:
                    self.trot()
                    self.enable_move = True
                
                if self.modes[4]:
                    self.gait()
                    self.enable_move = True

                if self.modes[5]:
                    self.default_modes()
                    if self.enable_dynamixel:
                        self.DXL.SET_TORQUE(0)

                if self.modes[6]:
                    self.leg_up()

                if self.enable_march:
                    self.march()

                if self.stabilization:
                    self.stabileBase()

                self.drawPolygon()
                self.publishing()

                self.enable_move = False
                # print(self.save_global_pos)

                self.IK.updatePosLegs()
                self.IK.FR_angles = self.IK.calculate_angles(self.IK.pos_FR)
                self.IK.FL_angles = self.IK.calculate_angles(self.IK.pos_FL, -1)
                self.IK.BR_angles = self.IK.calculate_angles(self.IK.pos_BR)
                self.IK.BL_angles = self.IK.calculate_angles(self.IK.pos_BL, -1)

                matrix_angle = np.concatenate((self.IK.FR_angles, self.IK.FL_angles, self.IK.BR_angles, self.IK.BL_angles), axis=None)
                
                self.messages(matrix_angle)
                self.rate.sleep()
            # except:
            #    pass

    def callbackFootprintFR(self, data: PoseArray):
        if self.FootprintFR == None:
            self.FootprintFR = data.poses
        self.tror_take = True
        # print(self.FootprintFR[1])
    
    def callbackFootprintFL(self, data: PoseArray):
        if self.FootprintFL == None:
            self.FootprintFL = data.poses

    def callbackFootprintBR(self, data: PoseArray):
        if self.FootprintBR == None:
            self.FootprintBR = data.poses

    def callbackFootprintBL(self, data: PoseArray):
        if self.FootprintBL == None:
            self.FootprintBL = data.poses

    def publishing(self):
        # Legs position
        # SetPosFR
        pubSetPosFR = rospy.Publisher('/SetPosFR', PoseStamped, queue_size=1)
        poseSTFR = PoseStamped()
        poseSTFR.header = Header()
        poseSTFR.header = Header()
        poseSTFR.header.stamp = rospy.Time.now()
        poseSTFR.header.frame_id = "root"
        poseFR = Pose()
        poseFR.orientation = Quaternion(0.707, 0, 0.707, 0)
        poseFR.position.x = (-self.IK.pos_FR.x + self.IK.bodyFR.x - 1.251) / 100
        poseFR.position.y = -(self.IK.pos_FR.y + self.IK.bodyFR.y - 0.35) / 100
        poseFR.position.z = (-self.IK.pos_FR.z - 1.522) / 100
        poseSTFR.pose = poseFR
        pubSetPosFR.publish(poseSTFR)


        pubSetPosFL = rospy.Publisher('/SetPosFL', PoseStamped, queue_size=1)
        poseSTFL = PoseStamped()
        poseSTFL.header = Header()
        poseSTFL.header = Header()
        poseSTFL.header.stamp = rospy.Time.now()
        poseSTFL.header.frame_id = "root"
        poseFL = Pose()
        poseFL.orientation = Quaternion(0.707, 0, 0.707, 0)
        poseFL.position.x = (-self.IK.pos_FL.x + self.IK.bodyFL.x - 1.251) / 100
        poseFL.position.y = -(self.IK.pos_FL.y + self.IK.bodyFL.y - 0.35) / 100
        poseFL.position.z = (-self.IK.pos_FL.z - 1.522) / 100
        poseSTFL.pose = poseFL
        pubSetPosFL.publish(poseSTFL)

        pubSetPosBR = rospy.Publisher('/SetPosBR', PoseStamped, queue_size=1)
        poseSTBR = PoseStamped()
        poseSTBR.header = Header()
        poseSTBR.header = Header()
        poseSTBR.header.stamp = rospy.Time.now()
        poseSTBR.header.frame_id = "root"
        poseBR = Pose()
        poseBR.orientation = Quaternion(0.707, 0, 0.707, 0)
        poseBR.position.x = (-self.IK.pos_BR.x + self.IK.bodyBR.x - 1) / 100
        poseBR.position.y = -(self.IK.pos_BR.y + self.IK.bodyBR.y - 0.35) / 100
        poseBR.position.z = (-self.IK.pos_BR.z - 1.522) / 100
        poseSTBR.pose = poseBR
        pubSetPosBR.publish(poseSTBR)

        pubSetPosBL = rospy.Publisher('/SetPosBL', PoseStamped, queue_size=1)
        poseSTBL = PoseStamped()
        poseSTBL.header = Header()
        poseSTBL.header = Header()
        poseSTBL.header.stamp = rospy.Time.now()
        poseSTBL.header.frame_id = "root"
        poseBL = Pose()
        poseBL.orientation = Quaternion(0.707, 0, 0.707, 0)
        poseBL.position.x = (-self.IK.pos_BL.x + self.IK.bodyBL.x - 1) / 100
        poseBL.position.y = -(self.IK.pos_BL.y + self.IK.bodyBL.y - 0.35) / 100
        poseBL.position.z = (-self.IK.pos_BL.z - 1.522) / 100
        poseSTBL.pose = poseBL
        pubSetPosBL.publish(poseSTBL)

        # Publish PosArray for legs
        pubFR = rospy.Publisher('/FootprintFR', PoseArray, queue_size=1)
        pubFR.publish(self.posesFR)
        pubFL = rospy.Publisher('/FootprintFL', PoseArray, queue_size=1)
        pubFL.publish(self.posesFL)
        pubBR = rospy.Publisher('/FootprintBR', PoseArray, queue_size=1)
        pubBR.publish(self.posesBR)
        pubBL = rospy.Publisher('/FootprintBL', PoseArray, queue_size=1)
        pubBL.publish(self.posesBL)

        self.posesFR.poses.clear()
        self.posesFL.poses.clear()
        self.posesBR.poses.clear()
        self.posesBL.poses.clear()
        
        #Robot position
        pub_global_pos = rospy.Publisher("/global_pos_robot", Pose, queue_size=1)
        self.global_pos_robot.position.z = self.IK.robot_pos.z
        pub_global_pos.publish(self.global_pos_robot)

        # Enable move robot
        pub_move = rospy.Publisher("/enable_move", Bool, queue_size=10)
        pub_move.publish(self.enable_move)

    def drawPolygon(self):
        polygon_pub = rospy.Publisher('/polygon_topic', PolygonStamped, queue_size=1)

        polygonStamped = PolygonStamped()

        polygonStamped.header = Header()
        polygonStamped.header.stamp = rospy.Time.now()
        polygonStamped.header.frame_id = "root"

        width = (self.IK.body_collision.y) / 50
        depth = -(self.IK.body_collision.x) / 50
        x_offset = 0
        y_offset = -0.025
        z_offset = -0.01
        
        polygonStamped.polygon = Polygon()

        # print(self.IK.pos_FR)
        # print(self.IK.pointFR)
        
        if (self.IK.pointFR.z < 0.5):
            point = Point32()
            point.x = (-self.IK.pos_FR.x + self.IK.bodyFR.x - 1.451) / 100
            point.y = -(self.IK.pos_FR.y + self.IK.bodyFR.y) / 100
            point.z = -self.IK.robot_pos.z / 100 + z_offset
            polygonStamped.polygon.points.append(point)
        
        if (self.IK.pointFL.z < 0.5):
            point = Point32()
            point.x = (-self.IK.pos_FL.x + self.IK.bodyFL.x - 1.451) / 100
            point.y = -(self.IK.pos_FL.y + self.IK.bodyFL.y) / 100
            point.z = -self.IK.robot_pos.z / 100 + z_offset
            polygonStamped.polygon.points.append(point)
        
        if (self.IK.pointBR.z < 0.5):
            point = Point32()
            point.x = (-self.IK.pos_BR.x + self.IK.bodyBR.x - 1.451) / 100
            point.y = -(self.IK.pos_BR.y + self.IK.bodyBR.y) / 100
            point.z = -self.IK.robot_pos.z / 100 + z_offset
            polygonStamped.polygon.points.append(point)
        
        if (self.IK.pointBL.z < 0.5):
            point = Point32()
            point.x = (-self.IK.pos_BL.x + self.IK.bodyBL.x - 1.451) / 100
            point.y = -(self.IK.pos_BL.y + self.IK.bodyBL.y) / 100
            point.z = -self.IK.robot_pos.z / 100 + z_offset
            polygonStamped.polygon.points.append(point)

        polygon_pub.publish(polygonStamped)

    def default_modes(self):
        for i in range(len(self.modes)):
            self.modes[i] = False

    def callback_joystick(self, data: Joy):
        
        self.R3_x = -data.axes[3]
        self.R3_y = -data.axes[4]
        self.L3_x = -data.axes[0]
        self.L3_y = -data.axes[1]

        self.arrow_x = -data.axes[6]
        self.arrow_y = data.axes[7]

        self.vec_L = angle_vec(self.L3_x, self.L3_y)
        self.vec_R = angle_vec(self.R3_x, self.R3_y)

        # Button
        # 0 1 2 3  4  5  6  7    8    9  10 11 12
        # X O T Sq L1 R1 L2 R2 Share Opt PS L3 R3

        # Axes
        #  0    1   2   3    4   5    6        7   
        # L3_x L3_y L2 R3_x R3_y R2 arrow_x arrow_y

        for i, state in enumerate(data.buttons):
            if state and not self.button_press[i]:
                self.button_press[i] = True

            if not state and self.button_press[i]:
                self.button_press[i] = False

                if (i == 3 and not self.button_press[i]): # Move
                    self.default_modes()
                    self.modes[0] = True

                    self.save_global_pos = Pose()
                    
                    self.save_global_pos.orientation.x = self.global_pos_robot.orientation.x
                    self.save_global_pos.orientation.y = self.global_pos_robot.orientation.y
                    self.save_global_pos.orientation.z = self.global_pos_robot.orientation.z

                    self.save_global_pos.position.x = self.global_pos_robot.position.x
                    self.save_global_pos.position.y = self.global_pos_robot.position.y
                    self.save_global_pos.position.z = self.global_pos_robot.position.z
                    print("Move")
                
                if (i == 1 and not self.button_press[i]): # Rotate
                    self.default_modes()
                    self.modes[1] = True
                    print("Rotate")

                if (i == 10 and not self.button_press[i]): # Up
                    self.default_modes()
                    self.modes[2] = True
                    self.stand_i = 0
                    print("UP")

                if (i == 0 and not self.button_press[i]): # Trot
                    self.default_modes()
                    self.modes[3] = True
                    print("Trot")

                if (i == 2 and not self.button_press[i]): # Gait
                    self.default_modes()
                    self.modes[4] = True
                    print("Gait")

                if (i == 5 and not self.button_press[i]): # Reset
                    self.default_modes()
                    self.modes[5] = True
                    print("Reset")

                if (i == 7 and not self.button_press[i]): # Leg up
                    self.default_modes()
                    self.modes[6] = True
                    print("Leg up")

                if (i == 8 and not self.button_press[i]): # March
                    self.enable_march = not self.enable_march
                    if (self.enable_march): 
                        print("Enable march")
                    else:
                        self.march_change = False
                        self.IK.pointFR = Point()
                        self.IK.pointFL = Point()
                        self.IK.pointBR = Point()
                        self.IK.pointBL = Point()
                        print("Disable march")
                
                if (i == 4 and not self.button_press[i]): # Used IMU
                    self.USE_IMU = not self.USE_IMU
                    if (self.USE_IMU):
                        print("Enable IMU")
                    else:  
                        print("Disable IMU")
                    
                if (i == 6 and not self.button_press[i]):
                    self.stabilization = not self.stabilization
                    if (self.stabilization):
                        print("Enable stabilization mode")
                    else:  
                        print("Disable stabilization mode")

                if (i == 9 and not self.button_press[i]):
                    self.take_point = not self.take_point
                    # if (self.take_point):
                        # print("Enable acc")
                    # else:  
                        # print("Disable acc")

        pass

    def callbackCmdVel(self, data):
        self.cmd_vel = data

    # -------------------------------------------------MODES-----------------------------------------------------
    def march(self):
        
        sec = 0.2

        if self.march_change:
            if self.march_tick <= sec * self.speed / 2:
                dz = self.map(self.march_tick, 0, sec * self.speed / 2, 0, 8)
            elif self.march_tick <= sec * self.speed: 
                dz = self.map(self.march_tick, sec * self.speed / 2, sec * self.speed, 8, 0)
            self.IK.pointFR = Point(0, 0, dz)
            self.IK.pointBL = Point(0, 0, dz)
        else:
            if self.march_tick <= sec * self.speed / 2:
                dz = self.map(self.march_tick, 0, sec * self.speed / 2, 0, 8)
            elif self.march_tick <= sec * self.speed: 
                dz = self.map(self.march_tick, sec * self.speed / 2, sec * self.speed, 8, 0)
            self.IK.pointFL = Point(0, 0, dz)
            self.IK.pointBR = Point(0, 0, dz)

        if self.march_tick == sec * self.speed:
            self.march_tick = 0
            self.march_change = not self.march_change

        self.IK.setBase(Point(1, 0, 15), Point(0, 0, 0), self.roll, self.pitch)

        self.march_tick += 1
        pass

    def map_step(self, start: float, end: float, count_step = 1):
        return np.around((end - start) / count_step, 2)
    
    def ellipce(self, x: float, a: float, b: float) -> float:
        if (a != 0):
            if x**2 / a**2 > 1:
                x = 1
                a = 1
            y = b * sqrt(1 - ((x)**2 / (a)**2))
        else: y = 0
        return -y
    
    def foot_path(self, percent: float, power: float, h: int, start, end):
        """Percent 0-1"""

        steps = 1 / 2

        if (percent <= steps):
            dy = self.map(percent, start, start + steps, -power, power)
            dz = self.ellipce(dy, power, h)

        if (percent > steps):
            dy = self.map(percent, start + steps, end, power, -power)
            dz = 0

        return dy, dz   

    def LegsPoints(self, dx, dy):
        offset = Point(11.5, -11.4, -self.IK.robot_pos.z - 1.522)

        #FR_leg
        pose = Pose()
        pose.orientation = Quaternion(0.707, 0, 0, 0.707)
        pose.position.x = offset.x / 100 - dx / 100
        pose.position.y = offset.y / 100 - dy / 100
        pose.position.z = offset.z / 100
        self.posesFR.poses.append(pose)

        #second
        pose = Pose()
        pose.orientation = Quaternion(0.707, 0, 0, 0.707)
        pose.position.x = offset.x / 100
        pose.position.y = offset.y / 100
        pose.position.z = offset.z / 100
        self.posesFR.poses.append(pose)

        #tree
        pose = Pose()
        pose.orientation = Quaternion(0.707, 0, 0, 0.707)
        pose.position.x = offset.x / 100 + self.gaitFR.x / 100
        pose.position.y = offset.y / 100 + self.gaitFR.y / 100
        pose.position.z = offset.z / 100
        self.posesFR.poses.append(pose)

        #FL_leg
        pose = Pose()
        pose.orientation = Quaternion(0.707, 0, 0, 0.707)
        pose.position.x = offset.x / 100 - dx / 100
        pose.position.y = -offset.y / 100 - dy / 100
        pose.position.z = offset.z / 100
        self.posesFL.poses.append(pose)

        #second
        pose = Pose()
        pose.orientation = Quaternion(0.707, 0, 0, 0.707)
        pose.position.x = offset.x / 100
        pose.position.y = -offset.y / 100
        pose.position.z = offset.z / 100
        self.posesFL.poses.append(pose)

        #tree
        pose = Pose()
        pose.orientation = Quaternion(0.707, 0, 0, 0.707)
        pose.position.x = offset.x / 100 + self.gaitFL.x / 100
        pose.position.y = -offset.y / 100 + self.gaitFL.y / 100
        pose.position.z = offset.z / 100
        self.posesFL.poses.append(pose)

        #BR_leg
        pose = Pose()
        pose.orientation = Quaternion(0.707, 0, 0, 0.707)
        pose.position.x = -offset.x / 100 - dx / 100 - 0.075
        pose.position.y = offset.y / 100 - dy / 100
        pose.position.z = offset.z / 100
        self.posesBR.poses.append(pose)

        #second
        pose = Pose()
        pose.orientation = Quaternion(0.707, 0, 0, 0.707)
        pose.position.x = -offset.x / 100 - 0.075
        pose.position.y = offset.y / 100
        pose.position.z = offset.z / 100
        self.posesBR.poses.append(pose)

        #tree
        pose = Pose()
        pose.orientation = Quaternion(0.707, 0, 0, 0.707)
        pose.position.x = -offset.x / 100 + self.gaitBR.x / 100 - 0.075
        pose.position.y = offset.y / 100 + self.gaitBR.y / 100
        pose.position.z = offset.z / 100
        self.posesBR.poses.append(pose)

        #BL_leg
        pose = Pose()
        pose.orientation = Quaternion(0.707, 0, 0, 0.707)
        pose.position.x = -offset.x / 100 - dx / 100 - 0.075
        pose.position.y = -offset.y / 100 - dy / 100
        pose.position.z = offset.z / 100
        self.posesBL.poses.append(pose)

        #second
        pose = Pose()
        pose.orientation = Quaternion(0.707, 0, 0, 0.707)
        pose.position.x = -offset.x / 100 - 0.075
        pose.position.y = -offset.y / 100
        pose.position.z = offset.z / 100
        self.posesBL.poses.append(pose)

        #tree
        pose = Pose()
        pose.orientation = Quaternion(0.707, 0, 0, 0.707)
        pose.position.x = -offset.x / 100 + self.gaitBL.x / 100 - 0.075
        pose.position.y = -offset.y / 100 + self.gaitBL.y / 100
        pose.position.z = offset.z / 100
        self.posesBL.poses.append(pose)

    def imu(self, data: Imu):
        if self.USE_IMU:
            q = data.orientation
            rpy_angles = transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
            
            
            local_roll = np.around(rpy_angles[0], 5) * 2
            local_pitch = np.around(rpy_angles[1], 5) * 2

            a = data.angular_velocity

            # self.acc_rx = np.around(a.x)
            # self.acc_ry = np.around(a.y)

            # print(local_pitch)

            sum = 0.02
            limit = 0.1
            if (abs(local_pitch - self.pitch) > 0.01):
                if local_pitch > 0:
                    self.pitch += sum
                elif local_pitch < 0:
                    self.pitch -= sum
                
            if (abs(local_roll - self.roll) > 0.01):
                if local_roll > 0:
                    self.roll += sum
                elif local_roll < 0:
                    self.roll -= sum

            if self.pitch < -limit:
                self.pitch = -limit
            elif self.pitch > limit:
                self.pitch = limit

            if self.pitch < -limit:
                self.roll = -limit
            elif self.pitch > limit:
                self.roll = limit
        else:
            self.roll = 0.0
            self.pitch = 0.0

    def stabileBase(self):

        Ex = 0
        Ey = 0
        count_leg = 0

        # print(self.IK.planeFR)

        for i, item in enumerate([self.IK.pos_FR, self.IK.pos_FL, self.IK.pos_BR, self.IK.pos_BL]):
            # print(f"{i}: {item}")
            if (item.z > self.IK.robot_pos.z - 0.1):
                if i < 2:
                    Ex += item.x + self.IK.body_collision.x
                else:
                    Ex += item.x - self.IK.body_collision.x
                if item.y < 0:
                    Ey += item.y - self.IK.len_leg.x
                else:
                    Ey += item.y + self.IK.len_leg.y
                count_leg += 1

        if count_leg != 0:
            Ex /= count_leg
            Ey /= count_leg

        # print(f"Ex: {Ex}, Ey: {Ey}")

        self.ex = self.PID_xstab.calculate(self.ex, Ex)
        self.ey = self.PID_ystab.calculate(self.ey, Ey * 10)

        self.IK.setBase(Point(0, -self.ey, self.IK.robot_pos.z), Point(0, 0, 0), self.roll, self.pitch)

    def gait(self):
        ####################################################WORK############################################ ### # ### ## ##
        
        sec = 15 / self.speed # time circle move 0.1

        time_stop = 4

        h = 4

        #--------------------GAMEPAD---------------------------
        dyL = self.cmd_vel.linear.y
        dxL = self.cmd_vel.linear.x
        dyR = -self.cmd_vel.angular.z
                
        leng = sqrt(dyR**2 + dxL**2)
        if leng != 0:
            angle = asin(dyR / leng)
        else:
            angle = 0

        dx = dxL * cos(angle)
        dy = dyL * cos(angle)
        dy2 =  dyR


        if dx < 0.1 and dx > -0.1 and dy < 0.1 and dy > -0.1:
            dx = self.acc_rx
            dy = self.acc_ry       

        if self.USE_IMU:
            if self.pitch != 0:
                dz = sqrt(dx**2 + dy**2) * sin(self.pitch)
            else:
                dz = 0
        else:
            dz = 0
        
        if dz < 0:
            dz = -dz

        self.gaitFR = Point(dx, dy - dyR, dz)
        self.gaitFL = Point(dx, dy - dyR, dz)
        self.gaitBR = Point(dx, dy + dyR, dz)
        self.gaitBL = Point(dx, dy + dyR, dz)
        #-------------------------------------------------------
            
        power = sqrt(dx**2 + (dy + dy2)**2)

        # print(power)

        if (power > 0.5):
            if not self.gait_start_end:
                self.gait_start = True
        else:
            self.gait_start = False
            self.gait_start_tick = 0
            self.gait_FR = False
            self.gait_FL = False
            self.gait_FR_tick = 0
            self.gait_FL_tick = 0
            self.gait_start_end = False

            self.IK.pointFR = Point(0, 0, 0)
            self.IK.pointFL = Point(0, 0, 0)
            self.IK.pointBR = Point(0, 0, 0)
            self.IK.pointBL = Point(0, 0, 0)
 
        
        self.IK.setBase(Point(dx / 2, dy, self.IK.robot_pos.z), Point(0, 0, -dy2 / 30), self.roll, self.pitch)

        if self.gait_start and not self.gait_start_end:
            if self.gait_start_tick <= self.speed * sec + 1:

                # -----------------------POSITION-----------------------
                robot_x = self.map_step(0, (self.gaitFR.x + self.gaitBR.x) / 2, self.speed * sec)
                robot_y = self.map_step(0, (self.gaitFR.y + self.gaitBR.y) / 2, self.speed * sec)

                self.global_pos_robot.position.x += robot_x * cos(self.global_pos_robot.orientation.z) - robot_y * sin(self.global_pos_robot.orientation.z)
                self.global_pos_robot.position.y += robot_y * cos(self.global_pos_robot.orientation.z) + robot_x * sin(self.global_pos_robot.orientation.z)

                if dy2 != 0:
                    angle = -atan((dy2 / self.IK.body_collision.x) / 30)
                else: 
                    angle = 0
                self.global_pos_robot.orientation.z += angle
                # -------------------------------------------------------

                dx11_leg, dy11_leg, dz11_leg = self.IK.moveLegEllipce(self.gaitFR, self.gait_start_tick, self.speed * sec, h)
                dx12_leg, dy12_leg, dz12_leg = self.IK.moveLegEllipce(self.gaitBL, self.gait_start_tick, self.speed * sec, h)
                # print(dx, dy)
                # print(f"1: {dx_leg}, {dy_leg}, {dz_leg}")
                
                self.IK.pointFR = Point(dx11_leg, dy11_leg, dz11_leg)
                self.IK.pointBL = Point(dx12_leg, dy12_leg, dz12_leg)

            if self.gait_start_tick == self.speed * sec + 1 + time_stop:
                self.gait_start_end = True
                self.gait_start_tick = 0
            else: 
                self.gait_start_tick += 1

            #--------------------------------------FOOTPRINT----------------------------------------------------
            self.LegsPoints(dx, dy)
            # #FR_LEG
            # #first
            # pose = Pose()
            # pose.orientation = Quaternion(0.707, 0, 0, 0.707)
            # pose.position.x = offset.x / 100 - dx / 100
            # pose.position.y = offset.y / 100 - dy / 100
            # pose.position.z = offset.z / 100
            # self.posesFR.poses.append(pose)

            # #second
            # pose = Pose()
            # pose.orientation = Quaternion(0.707, 0, 0, 0.707)
            # pose.position.x = offset.x / 100
            # pose.position.y = offset.y / 100
            # pose.position.z = offset.z / 100
            # self.posesFR.poses.append(pose)

            # #tree
            # pose = Pose()
            # pose.orientation = Quaternion(0.707, 0, 0, 0.707)
            # pose.position.x = offset.x / 100 + self.gaitFR.x / 100
            # pose.position.y = offset.y / 100 + self.gaitFR.y / 100
            # pose.position.z = offset.z / 100
            # self.posesFR.poses.append(pose)
        
        # print(self.IK.pointFR)

        if self.gait_start and self.gait_start_end:
            if not self.gait_FR:
                if self.gait_FR_tick <= self.speed * sec + 1:

                    # -----------------------POSITION-----------------------
                    robot_x = self.map_step(0, (self.gaitFR.x + self.gaitBR.x) / 2, self.speed * sec)
                    robot_y = self.map_step(0, (self.gaitFR.y + self.gaitBR.y) / 2, self.speed * sec)

                    self.global_pos_robot.position.x += robot_x * cos(self.global_pos_robot.orientation.z) - robot_y * sin(self.global_pos_robot.orientation.z)
                    self.global_pos_robot.position.y += robot_y * cos(self.global_pos_robot.orientation.z) + robot_x * sin(self.global_pos_robot.orientation.z)

                    if dy2 != 0:
                        angle = -atan((dy2 / self.IK.body_collision.x) / 30)
                    else: 
                        angle = 0
                    self.global_pos_robot.orientation.z += angle
                    # -------------------------------------------------------

                    dx11_leg = self.map(self.gait_FR_tick, 0, self.speed * sec, self.gaitFR.x, 0)
                    dy11_leg = self.map(self.gait_FR_tick, 0, self.speed * sec, self.gaitFR.y, 0)
                    dz11_leg = self.map(self.gait_FR_tick, 0, self.speed * sec, self.gaitFR.z, 0)

                    dx12_leg = self.map(self.gait_FR_tick, 0, self.speed * sec, self.gaitBL.x, 0)
                    dy12_leg = self.map(self.gait_FR_tick, 0, self.speed * sec, self.gaitBL.y, 0)
                    dz12_leg = self.map(self.gait_FR_tick, 0, self.speed * sec, self.gaitBL.z, 0)

                    dx21_leg, dy21_leg, dz21_leg = self.IK.moveLegEllipce(self.gaitFL, self.gait_FR_tick, self.speed * sec, h)
                    dx22_leg, dy22_leg, dz22_leg = self.IK.moveLegEllipce(self.gaitBR, self.gait_FR_tick, self.speed * sec, h)

                    # print(f"st1: {dx}, {dy}, {dy2}")

                    self.IK.pointFR = Point(dx11_leg, dy11_leg, dz11_leg)
                    self.IK.pointBL = Point(dx12_leg, dy12_leg, dz12_leg)
                    self.IK.pointFL = Point(dx21_leg, dy21_leg, dz21_leg)
                    self.IK.pointBR = Point(dx22_leg, dy22_leg, dz22_leg)

                if self.gait_FR_tick == self.speed * sec + 1 + time_stop:
                    self.gait_FR_tick = 0
                    self.gait_FR = True 
                else:
                    self.gait_FR_tick += 1

                #--------------------------------------FOOTPRINT----------------------------------------------------
                self.LegsPoints(dx, dy)
                # #FR_LEG
                # #first
                # pose = Pose()
                # pose.orientation = Quaternion(0.707, 0, 0, 0.707)
                # pose.position.x = offset.x / 100 - dx / 100
                # pose.position.y = offset.y / 100 - dy / 100
                # pose.position.z = offset.z / 100
                # self.posesFR.poses.append(pose)

                # #second
                # pose = Pose()
                # pose.orientation = Quaternion(0.707, 0, 0, 0.707)
                # pose.position.x = offset.x / 100
                # pose.position.y = offset.y / 100
                # pose.position.z = offset.z / 100
                # self.posesFR.poses.append(pose)

                # #tree
                # pose = Pose()
                # pose.orientation = Quaternion(0.707, 0, 0, 0.707)
                # pose.position.x = offset.x / 100 + self.gaitFR.x / 100
                # pose.position.y = offset.y / 100 + self.gaitFR.y / 100
                # pose.position.z = offset.z / 100
                # self.posesFR.poses.append(pose)
            
            else:
                if self.gait_FR_tick <= self.speed * sec + 1:

                    # -----------------------POSITION-----------------------
                    robot_x = self.map_step(0, (self.gaitFR.x + self.gaitBR.x) / 2, self.speed * sec)
                    robot_y = self.map_step(0, (self.gaitFR.y + self.gaitBR.y) / 2, self.speed * sec)

                    self.global_pos_robot.position.x += robot_x * cos(self.global_pos_robot.orientation.z) - robot_y * sin(self.global_pos_robot.orientation.z)
                    self.global_pos_robot.position.y += robot_y * cos(self.global_pos_robot.orientation.z) + robot_x * sin(self.global_pos_robot.orientation.z)

                    if dy2 != 0:
                        angle = -atan((dy2 / self.IK.body_collision.x) / 30)
                    else: 
                        angle = 0
                    self.global_pos_robot.orientation.z += angle
                    # -------------------------------------------------------


                    dx11_leg, dy11_leg, dz11_leg = self.IK.moveLegEllipce(self.gaitFR, self.gait_FR_tick, self.speed * sec, h)
                    dx12_leg, dy12_leg, dz12_leg = self.IK.moveLegEllipce(self.gaitBL, self.gait_FR_tick, self.speed * sec, h)
                    # print(dx, dy)

                    dx21_leg = self.map(self.gait_FR_tick, 0, self.speed * sec, self.gaitFL.x, 0)
                    dy21_leg = self.map(self.gait_FR_tick, 0, self.speed * sec, self.gaitFL.y, 0)
                    dz21_leg = self.map(self.gait_FR_tick, 0, self.speed * sec, self.gaitFL.z, 0)

                    dx22_leg = self.map(self.gait_FR_tick, 0, self.speed * sec, self.gaitBR.x, 0)
                    dy22_leg = self.map(self.gait_FR_tick, 0, self.speed * sec, self.gaitBR.y, 0)
                    dz22_leg = self.map(self.gait_FR_tick, 0, self.speed * sec, self.gaitBR.z, 0)

                    self.IK.pointFR = Point(dx11_leg, dy11_leg, dz11_leg)
                    self.IK.pointBL = Point(dx12_leg, dy12_leg, dz12_leg)
                    self.IK.pointFL = Point(dx21_leg, dy21_leg, dz21_leg)
                    self.IK.pointBR = Point(dx22_leg, dy22_leg, dz22_leg)
                
                    
                if self.gait_FR_tick == self.speed * sec + 1 + time_stop:
                    self.gait_FR_tick = 0
                    self.gait_FR = False 
                else:
                    self.gait_FR_tick += 1

                #--------------------------------------FOOTPRINT----------------------------------------------------
                self.LegsPoints(dx, dy)

                #FR_LEG
                # #first
                # pose = Pose()
                # pose.orientation = Quaternion(0.707, 0, 0, 0.707)
                # pose.position.x = offset.x / 100 - dx / 100
                # pose.position.y = offset.y / 100 - dy / 100
                # pose.position.z = offset.z / 100
                # self.posesFR.poses.append(pose)

                # #second
                # pose = Pose()
                # pose.orientation = Quaternion(0.707, 0, 0, 0.707)
                # pose.position.x = offset.x / 100
                # pose.position.y = offset.y / 100
                # pose.position.z = offset.z / 100
                # self.posesFR.poses.append(pose)

                # #tree
                # pose = Pose()
                # pose.orientation = Quaternion(0.707, 0, 0, 0.707)
                # pose.position.x = offset.x / 100 + self.gaitFR.x / 100
                # pose.position.y = offset.y / 100 + self.gaitFR.y / 100
                # pose.position.z = offset.z / 100
                # self.posesFR.poses.append(pose)
               
        self.gait_tick += 1

    def trot(self):
        self.LegsPoints()
    
        if (self.pitch < 1 and self.pitch > -1):
            h = 4 * acos(self.pitch)
        else:
            h = 4

        sec = 6 # time circle move 0.1
        
        dyL = self.cmd_vel.linear.y
        dxL = self.cmd_vel.linear.x
        dyR = -self.cmd_vel.angular.z
        
        leng = sqrt(dyR**2 + dxL**2)
        if leng != 0:
            angle = asin(dyR / leng)
        else:
            angle = 0

        dx = dxL * cos(angle)
        dy = dyL * cos(angle)
        dy2 =  dyR

        mode = -1 
        
        # print(f"m: {dyR}, {dy2}, {angle}")
        # print(f"cmd: {dx}, {dy}")

        power = sqrt(dx**2 + (dy + dy2)**2)
        
        if power > 0.5:
            self.trot_start = True       
        # elif self.trot_tick == 0:
        #     self.trot_start = False
        else:
            self.trot_start = False

        if self.trot_start:
            if self.trot_tick <= sec * self.speed:
                if self.trot_tick <= sec * self.speed * 1 / 12:
                    if self.trot_tick == 0:
                        self.dy1_base = -((self.IK.pointFL.y - self.IK.len_leg.x + self.IK.bodyFL.y) + (self.IK.pointBR.y + self.IK.len_leg.x + self.IK.bodyBR.y) + (self.IK.pointBL.y - self.IK.len_leg.x + self.IK.bodyBL.y)) * 0.33
                    self.trot_move.y = self.map(self.trot_tick, 0, sec * self.speed * 1 / 12, 0, self.dy1_base)

                elif self.trot_tick <= sec * self.speed * 2 / 12:
                    self.trotFR.x, self.trotFR.y, self.trotFR.z = self.IK.moveLegEllipce(Point(dx, dy, 0), self.trot_tick - sec * self.speed * 1 / 12, sec * self.speed * 1 / 12, h) 
                    self.IK.pointFR = Point(self.trotFR.x, self.trotFR.y, self.trotFR.z)
                
                elif self.trot_tick <= sec * self.speed * 4 / 12:
                    if self.trot_tick == sec * self.speed * 2 / 12 + 1:
                        self.dy2_base = -((self.IK.pointFR.y + self.IK.len_leg.x + self.IK.bodyFR.y) + (self.IK.pointBR.y + self.IK.len_leg.x + self.IK.bodyBR.y) + (self.IK.pointBL.y - self.IK.len_leg.x + self.IK.bodyBL.y)) * 0.33
                    self.trot_move.y = self.map(self.trot_tick, sec * self.speed * 2 / 12, sec * self.speed * 4 / 12, self.dy1_base, self.dy2_base)

                elif self.trot_tick <= sec * self.speed * 5 / 12:
                    self.trotFL.x, self.trotFL.y, self.trotFL.z = self.IK.moveLegEllipce(Point(dx, dy, 0), self.trot_tick - sec * self.speed * 4 / 12, sec * self.speed * 1 / 12, h) 
                    self.IK.pointFL = Point(self.trotFL.x, self.trotFL.y, self.trotFL.z)

                elif self.trot_tick <= sec * self.speed * 7 / 12:
                    if self.trot_tick == sec * self.speed * 5 / 12 + 1:
                        self.dy1_base = -((self.IK.pointFL.y - self.IK.len_leg.x + self.IK.bodyFL.y) + (self.IK.pointFR.y + self.IK.len_leg.x + self.IK.bodyFR.y) + (self.IK.pointBL.y - self.IK.len_leg.x + self.IK.bodyBL.y)) * 0.33    
                    self.trot_move.y = self.map(self.trot_tick, sec * self.speed * 5 / 12, sec * self.speed * 7 / 12, self.dy2_base, self.dy1_base)

                elif self.trot_tick <= sec * self.speed * 8 / 12:
                    self.trotBR.x, self.trotBR.y, self.trotBR.z = self.IK.moveLegEllipce(Point(dx, dy, 0), self.trot_tick - sec * self.speed * 7 / 12, sec * self.speed * 1 / 12, h) 
                    self.IK.pointBR = Point(self.trotBR.x, self.trotBR.y, self.trotBR.z)

                elif self.trot_tick <= sec * self.speed * 10 / 12:
                    if self.trot_tick == sec * self.speed * 8 / 12 + 1:
                        self.dy2_base = -((self.IK.pointFL.y - self.IK.len_leg.x + self.IK.bodyFL.y) + (self.IK.pointFR.y + self.IK.len_leg.x + self.IK.bodyFR.y) + (self.IK.pointBR.y + self.IK.len_leg.x + self.IK.bodyBR.y)) * 0.33  
                    self.trot_move.y = self.map(self.trot_tick, sec * self.speed * 8 / 12, sec * self.speed * 10 / 12, self.dy1_base, self.dy2_base)

                elif self.trot_tick <= sec * self.speed * 11 / 12:
                    self.trotBL.x, self.trotBL.y, self.trotBL.z = self.IK.moveLegEllipce(Point(dx, dy, 0), self.trot_tick - sec * self.speed * 10 / 12, sec * self.speed * 1 / 12, h) 
                    self.IK.pointBL = Point(self.trotBL.x, self.trotBL.y, self.trotBL.z)
                
                elif self.trot_tick <= sec * self.speed * 12 / 12:
                    self.trot_move.y = self.map(self.trot_tick, sec * self.speed * 11 / 12, sec * self.speed * 12 / 12, self.dy2_base, 0)

                if (self.trot_tick > sec * self.speed * 5 / 12 and self.trot_tick < sec * self.speed * 7 / 12):
                    self.IK.pointFR.x = self.map(self.trot_tick, sec * self.speed * 5 / 12, sec * self.speed * 7 / 12, self.trotFR.x, 0)
                    self.IK.pointFR.y = self.map(self.trot_tick, sec * self.speed * 5 / 12, sec * self.speed * 7 / 12, self.trotFR.y, 0)
                    self.IK.pointFR.z = self.map(self.trot_tick, sec * self.speed * 5 / 12, sec * self.speed * 7 / 12, self.trotFR.z, 0)

                    self.IK.pointFL.x = self.map(self.trot_tick, sec * self.speed * 5 / 12, sec * self.speed * 7 / 12, self.trotFL.x, 0)
                    self.IK.pointFL.y = self.map(self.trot_tick, sec * self.speed * 5 / 12, sec * self.speed * 7 / 12, self.trotFL.y, 0)
                    self.IK.pointFL.z = self.map(self.trot_tick, sec * self.speed * 5 / 12, sec * self.speed * 7 / 12, self.trotFL.z, 0)

                    self.IK.pointBR.x = self.map(self.trot_tick, sec * self.speed * 5 / 12, sec * self.speed * 7 / 12, 0, -self.trotFL.x)
                    self.IK.pointBR.y = self.map(self.trot_tick, sec * self.speed * 5 / 12, sec * self.speed * 7 / 12, 0, -self.trotFL.y)
                    self.IK.pointBR.z = self.map(self.trot_tick, sec * self.speed * 5 / 12, sec * self.speed * 7 / 12, 0, -self.trotFL.z)

                    self.IK.pointBL.x = self.map(self.trot_tick, sec * self.speed * 5 / 12, sec * self.speed * 7 / 12, 0, -self.trotFR.x)
                    self.IK.pointBL.y = self.map(self.trot_tick, sec * self.speed * 5 / 12, sec * self.speed * 7 / 12, 0, -self.trotFR.y)
                    self.IK.pointBL.z = self.map(self.trot_tick, sec * self.speed * 5 / 12, sec * self.speed * 7 / 12, 0, -self.trotFR.z)
                
                # if (self.trot_tick < sec * self.speed * 10 / 12):
                #     self.IK.pointBL.x = self.map(self.trot_tick, 0, sec * self.speed * 10 / 12, dx, 0)
                #     self.IK.pointBL.y = self.map(self.trot_tick, 0, sec * self.speed * 10 / 12, dy, 0)
                #     self.IK.pointBL.z = self.map(self.trot_tick, 0, sec * self.speed * 10 / 12, 0, 0)


                # if self.trot_tick <= sec * self.speed * 1 / 12


            # elif self.trot_tick <= sec * self.speed:
            #     self.trot_move.y = 0
            #     pass
        else:
            self.trot_move = Point(0, 0, 15)
        
        print(f"{self.trot_tick} - {self.trot_move}")

        self.trot_move.x = 0

        self.IK.setBase(self.trot_move, Point(), self.roll, self.pitch)

        if self.trot_tick <= sec * self.speed and self.trot_start:
            self.trot_tick += 1
        else:
            self.trot_tick = 0

        pass


    def stand(self):

        if self.stand_i == 0:
            self.down()

        sec = 20

        steps = self.speed * sec

        if (self.stand_i < steps):

            dz = self.map(self.stand_i, 0, steps, self.IK.robot_pos.z, 18)
            dy = self.map(self.stand_i, 0, steps, self.IK.robot_pos.y, 0)
            dx = self.map(self.stand_i, 0, steps, self.IK.robot_pos.x, 0)

            self.IK.setBase(Point(0, dy, dz), Point(), self.roll, self.pitch)

            self.stand_i += 1
            # rospy.sleep(0.05)
        else:
            self.modes[2] = False

    def leg_up(self):
        if (-self.R3_y > 0):
            dz = self.map(-self.R3_y, 0, 1, 0, 5)
        else: dz = 0

        dx = self.map(-self.L3_y, -1, 1, -5, 5)
        dy = self.map(-self.L3_x, -1, 1, -5, 5) 

        self.IK.pointFR = Point(dx, dy, dz)

        # print(f"pFR_lu: {self.IK.pointFR}")
    
    
    def down(self):
        self.IK.setBase(Point(0, 0, 10), Point(0, 0, 0), self.roll, self.pitch)
        return 0

    def moving(self):

        limitX = [-5, 5]
        limitY = [-5, 5]
        limitZ = [-7, 2.5]

        mapY = self.map(self.L3_x, -1, 1, limitX[0], limitX[1])
        mapX = self.map(self.L3_y, -1, 1, limitY[0], limitY[1])

        if (-self.R3_y < 0): mapZ = self.map(-self.R3_y, 0, -1, 0, limitZ[0])
        else: mapZ = self.map(-self.R3_y, 0, 1, 0, limitZ[1])

        # self.dx = mapX
        # self.dy = mapY
        # self.dz = mapZ

        self.dx = self.PID_x.calculate(self.dx, mapX)
        self.dy = self.PID_y.calculate(self.dy, mapY)
        self.dz = self.PID_z.calculate(self.dz, mapZ)
        
        self.global_pos_robot.position.x = self.save_global_pos.position.x - (self.dx * cos(self.save_global_pos.orientation.z) - self.dy * sin(self.save_global_pos.orientation.z))
        self.global_pos_robot.position.y = self.save_global_pos.position.y - (self.dy * cos(self.save_global_pos.orientation.z) + self.dx * sin(self.save_global_pos.orientation.z))

        # print(self.global_pos_robot.position)
        # print(self.save_global_pos)

        # print(self.dx, self.dy, self.dz)
        self.IK.setBase(Point(-self.dx, -self.dy, 18 + self.dz), Point(), self.roll, self.pitch)
        
        pass


    def rotating(self):

        limitX = [-0.3, 0.3]
        limitY = [-0.3, 0.3]
        limitZ = [-0.3, 0.3]
        limitMZ = [-7, 3]

        mapX = self.map(-self.L3_x, 1, -1, limitX[0], limitX[1])
        mapY = self.map(self.L3_y, 1, -1, limitY[0], limitY[1])
        mapZ = self.map(self.R3_x, 1, -1, limitZ[0], limitZ[1])

        if (-self.R3_y < 0): mapMZ = self.map(-self.R3_y, 0, -1, 0, limitMZ[0])
        else: mapMZ = self.map(-self.R3_y, 0, 1, 0, limitMZ[1])

        self.dx = self.PID_x.calculate(self.dx, mapX)
        self.dy = self.PID_y.calculate(self.dy, mapY)
        self.dz = self.PID_z.calculate(self.dz, mapZ)
        self.dmz = mapMZ

        self.global_pos_robot.position.z = self.IK.robot_pos.z

        print(f"dx: {self.dx}, dt: {self.dy}, dz: {self.dz},")

        # self.IK.movePosRobot(Point(0, 0, 0), Point(-self.dx, self.dy, self.dz), self.roll, self.pitch)
        self.IK.setBase(Point(0, 0, self.save_global_pos.position.z + self.dmz), Point(-self.dx, self.dy, self.dz), self.roll, self.pitch)

    # ---------------------------------------------SEND MESSAGE-----------------------------------------------------
    
    def messages(self, matrix_angle):
        self.send_gazebo(matrix_angle) # Gazebo
        self.rviz.jointstate(matrix_angle)
        self.send_dynamixel(matrix_angle) # Arctic


    def send_gazebo(self, matrix):
        # mat = np.array([1, 1, 1, -1, -1, -1, -1, 1, 1, 1, -1, -1])
        mat = np.array([-1, -1, -1, -1, 1, 1, 1, -1, -1, 1, 1, 1])
        mat = mat * matrix

        pub = rospy.Publisher("/Arctic/foot_1_/command", Float64, queue_size=10)
        pub.publish(mat[0])

        pub = rospy.Publisher("/Arctic/foot_2_/command", Float64, queue_size=10)
        pub.publish(mat[1])

        pub = rospy.Publisher("/Arctic/foot_3_/command", Float64, queue_size=10)
        pub.publish(mat[2])

        pub = rospy.Publisher("/Arctic/foot_4_/command", Float64, queue_size=10)
        pub.publish(mat[3])

        pub = rospy.Publisher("/Arctic/foot_5_/command", Float64, queue_size=10)
        pub.publish(mat[4])

        pub = rospy.Publisher("/Arctic/foot_6_/command", Float64, queue_size=10)
        pub.publish(mat[5])

        pub = rospy.Publisher("/Arctic/foot_7_/command", Float64, queue_size=10)
        pub.publish(mat[6])

        pub = rospy.Publisher("/Arctic/foot_8_/command", Float64, queue_size=10)
        pub.publish(mat[7])

        pub = rospy.Publisher("/Arctic/foot_9_/command", Float64, queue_size=10)
        pub.publish(mat[8])

        pub = rospy.Publisher("/Arctic/foot_10_/command", Float64, queue_size=10)
        pub.publish(mat[9])

        pub = rospy.Publisher("/Arctic/foot_11_/command", Float64, queue_size=10)
        pub.publish(mat[10])

        pub = rospy.Publisher("/Arctic/foot_12_/command", Float64, queue_size=10)
        pub.publish(mat[11])

    def send_dynamixel(self, matrix):
        map_matrix = []
        mul = [1, 1, 1, 1, -1, -1, -1, 1, 1, -1, -1, -1]
        matrix = mul * matrix

        # print(matrix)
        for item in matrix:
            if item != None:
                map_matrix.append(int(self.map(item, -3.14, 3.14, 0, 4095)))

        pub = rospy.Publisher("dynamixel", Float32MultiArray, queue_size=10)
        data = Float32MultiArray()
        data.data = map_matrix
        pub.publish(data)

        if self.enable_dynamixel:
            self.DXL.sendvalue(map_matrix)
        pass

    # ---------------------------------------------------------------------------------------------------------------

    def map(self, x: float, in_min: float, in_max: float, out_min: float, out_max: float) -> float:
        return np.around((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min, 2)

if __name__ == "__main__":
    rospy.init_node('inverce_node')
    node = Main_node()
    node.down()
    node.main()
    pass    