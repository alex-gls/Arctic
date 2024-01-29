#!/usr/bin/env python3
import rospy
import numpy as np
from . Transformations import * # this file have a rotation matrix
from . PID import * 
# from Transformations import *
from math import *

from geometry_msgs.msg import Point, Point

class Kinematics():
    def __init__(self, **kwargs) -> None:
        
        # self.len_leg = [0.66 * 2, 0.87 * 2, 1.07 * 2]  # arctic
        # self.body = np.array([2.28, 2.24, 0])
    
        # self.len_leg = [6.55, 8.65, 12.25]  # arctic
        # self.body = np.array([4.84, 11.15, 0])

        self.body_collision = Point(13.65, 4.84, 0)

        self.robot_pos = Point(0, 0, 0)
        self.robot_move_pos = Point(0, 0, 0)
        self.robot_local_pos = Point(0, 0, 0)

        # self.len_leg = Point(8.65, 6.55, 12.25)
        self.len_leg = Point(6.482, 6.547, 14.33) # 6.482 6.712, 8.649 6.547 7.467

        self.start_y = 0
        self.start_z = 5

        self.joint_FR = Point(0, 0, 0)
        self.joint_FL = Point(0, 0, 0)
        self.joint_BR = Point(0, 0, 0)
        self.joint_BL = Point(0, 0, 0)

        self.pos_FR = Point(0, 0, 0)
        self.pos_FL = Point(0, 0, 0)
        self.pos_BR = Point(0, 0, 0)
        self.pos_BL = Point(0, 0, 0)

        self.set_FR = Point()
        self.set_FL = Point()
        self.set_BR = Point()
        self.set_BL = Point()

        self.pointFR = Point()
        self.pointFL = Point()
        self.pointBR = Point()
        self.pointBL = Point()

        self.local_pos_FR = Point(0, 0, 0)
        self.local_pos_FL = Point(0, 0, 0)
        self.local_pos_BR = Point(0, 0, 0)
        self.local_pos_BL = Point(0, 0, 0)

        self.move_pos_FR = Point(0, 0, 0)
        self.move_pos_FL = Point(0, 0, 0)
        self.move_pos_BR = Point(0, 0, 0)
        self.move_pos_BL = Point(0, 0, 0)

        self.P = 0.01
        self.I = 0.05
        self.D = 0.0

        self.xPID_FR = PID(self.P, self.I, self.D)
        self.yPID_FR = PID(self.P, self.I, self.D)
        self.zPID_FR = PID(self.P, self.I, self.D)

        self.coordinate_FR = Point(self.body_collision.x, self.body_collision.y + self.len_leg.x, 15)
        self.coordinate_FL = Point(self.body_collision.x, -self.body_collision.y - self.len_leg.x, 15)
        self.coordinate_BR = Point(-self.body_collision.x, self.body_collision.y + self.len_leg.x, 15)
        self.coordinate_BL = Point(-self.body_collision.x, -self.body_collision.y - self.len_leg.x, 15)

        self.bodyFR = Point(0, 0, 0)
        self.bodyFL = Point(0, 0, 0)
        self.bodyBR = Point(0, 0, 0)
        self.bodyBL = Point(0, 0, 0)
        
        self.planeFR = Point()
        self.planeFL = Point()
        self.planeBR = Point()
        self.planeBL = Point()

        self.FR_angles = np.array([0.0, 0.0, 0.0])
        self.FL_angles = np.array([0.0, 0.0, 0.0])
        self.BR_angles = np.array([0.0, 0.0, 0.0])
        self.BL_angles = np.array([0.0, 0.0, 0.0])

        self.dx = 0.0
        self.dy = 0.0

        self.step = 0
        
        self.alpha = 0
        self.beta  = 0
        self.gamma = 0
        pass

    def updatePosLegs(self): 
        offset = Point(-0.5,-1, 0)

        # FR_LEG
        self.pos_FR.x = self.local_pos_FR.x - self.pointFR.x + offset.x
        self.pos_FR.y = self.local_pos_FR.y + self.len_leg.x - self.pointFR.y + offset.y
        self.pos_FR.z = self.local_pos_FR.z - self.pointFR.z + offset.z
        
        # FL_LEG
        self.pos_FL.x = self.local_pos_FL.x - self.pointFL.x + offset.x
        self.pos_FL.y = self.local_pos_FL.y - self.len_leg.x - self.pointFL.y - offset.y
        self.pos_FL.z = self.local_pos_FL.z - self.pointFL.z + offset.z

        # BR_LEG
        self.pos_BR.x = self.local_pos_BR.x - self.pointBR.x - offset.x + 2
        self.pos_BR.y = self.local_pos_BR.y + self.len_leg.x - self.pointBR.y + offset.y
        self.pos_BR.z = self.local_pos_BR.z - self.pointBR.z + offset.z

        # BL_LEG
        self.pos_BL.x = self.local_pos_BL.x - self.pointBL.x - offset.x + 2
        self.pos_BL.y = self.local_pos_BL.y - self.len_leg.x - self.pointBL.y - offset.y
        self.pos_BL.z = self.local_pos_BL.z - self.pointBL.z + offset.z
    

    def updateLocalPosLegs(self):
        self.local_pos_FR = Point(self.planeFR.x - self.bodyFR.x, self.planeFR.y - self.bodyFR.y, self.bodyFR.z - self.planeFR.z)
        self.local_pos_FL = Point(self.planeFL.x - self.bodyFL.x, -self.planeFL.y + self.bodyFL.y, self.bodyFL.z - self.planeFL.z)
        self.local_pos_BR = Point(self.planeBR.x - self.bodyBR.x, self.planeBR.y - self.bodyBR.y, self.bodyBR.z - self.planeBR.z)
        self.local_pos_BL = Point(self.planeBL.x - self.bodyBL.x, -self.planeBL.y + self.bodyBL.y, self.bodyBL.z - self.planeBL.z)


    def updatePlaneLegs(self, point_rotate: Point, roll: float, pitch: float):

        self.planeFR = Point(self.bodyFR.x + self.robot_pos.x, self.bodyFR.y + self.robot_pos.y, self.bodyFR.z - self.robot_pos.z)
        self.planeFR = self.transferPos(self.planeFR, 0, 0, 0, point_rotate.x - roll, point_rotate.y + pitch, point_rotate.z)

        self.planeFL = Point(self.bodyFL.x + self.robot_pos.x, self.bodyFL.y - self.robot_pos.y, self.bodyFL.z - self.robot_pos.z)
        self.planeFL = self.transferPos(self.planeFL, 0, 0, 0, point_rotate.x - roll, point_rotate.y + pitch, -point_rotate.z)

        self.planeBR = Point(self.bodyBR.x + self.robot_pos.x, self.bodyBR.y + self.robot_pos.y, self.bodyBR.z - self.robot_pos.z)
        self.planeBR = self.transferPos(self.planeBR, 0, 0, 0, point_rotate.x - roll, point_rotate.y + pitch, point_rotate.z)

        self.planeBL = Point(self.bodyBL.x + self.robot_pos.x, self.bodyBL.y - self.robot_pos.y, self.bodyBL.z - self.robot_pos.z)
        self.planeBL = self.transferPos(self.planeBL, 0, 0, 0, point_rotate.x - roll, point_rotate.y + pitch, -point_rotate.z)
        
        # print(f"pl: {self.planeFL}")
        
        self.updateLocalPosLegs()

    def setBase(self, point_move: Point, point_rotate: Point, roll: float, pitch: float):
        """
        Установить центр робота в точку
        """

        offset = Point(2.5, 0, 0)

        self.robot_pos.x = point_move.x + offset.x
        self.robot_pos.y = point_move.y + offset.y
        self.robot_pos.z = point_move.z + offset.z

        self.bodyFR = Point(self.body_collision.x - 0.495, self.body_collision.y, self.robot_pos.z)
        self.bodyFL = Point(self.body_collision.x - 0.495, -self.body_collision.y, self.robot_pos.z)
        self.bodyBR = Point(-self.body_collision.x + 0.495, self.body_collision.y, self.robot_pos.z)
        self.bodyBL = Point(-self.body_collision.x + 0.495, -self.body_collision.y, self.robot_pos.z)

        self.updatePlaneLegs(point_rotate, roll, pitch)
    
    def ellipce(self, x: float, a: float, c: float) -> float:
        if a == 0 and c == 0: return 0

        if a != 0:
            first = ((x)**2 / (a)**2)
            if first > 1:
                first = 1
        else: first = 0
        
        z = c * sqrt(1 - first)
        return z
    
    def moveLegEllipce(self, point: Point, step: int, count_step: int, h: float):

        dx = self.map(step, 0, count_step, 0, point.x)
        dy = self.map(step, 0, count_step, 0, point.y)
        

        lengs = sqrt(point.x**2 + point.y**2 + point.z**2)
        edx = self.map(step, 0, count_step, -lengs / 2, lengs / 2)

        if lengs != 0:
            beta = asin(point.z / lengs)
            gamma = asin(point.y / lengs)
        else:
            beta = 0
            gamma = 0
            
        if lengs > 0.1:
            dz = self.ellipce(edx, lengs / 2, h)
            point_1 = self.transferPos(Point(dx, dy, dz), 0, 0, 0, 0, -beta, gamma)
            dz = point_1.z
        else:
            dz = 0

        return dx, dy, dz

    def transferPos(self, pos_leg: Point, dx: float, dy: float, dz: float, alpha: float, beta: float, gamma: float):
        """
        Transfer position leg, update your position
        """
        return_vector = Point()
        matrix = np.array([pos_leg.x, pos_leg.y, pos_leg.z])
        
        matrix = np.append(matrix, 1) # Add element
        matrix = np.around(np.dot(homog_transform(dx, dy, dz, alpha, beta, gamma), matrix), 2)
        
        return_vector.x = matrix[0]
        return_vector.y = matrix[1]
        return_vector.z = matrix[2]

        return return_vector

    def map(self, x: float, in_min: float, in_max: float, out_min: float, out_max: float):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min        
    
    def calculate_angles(self, leg_pos: Point, mode = 1) -> list:
        pi = 3.14

        legs = np.array([self.len_leg.x, self.len_leg.y, self.len_leg.z])

        dx = leg_pos.x
        dy = leg_pos.y
        dz = leg_pos.z
        
        # print(dx, dy, dz)
        
        # Calculate angles
        alpha = 0
        beta = 0
        gamma = 0
        # -------------------------------------------------

        # alpha
        D = np.around(sqrt(dx**2 + dy**2 + dz**2), 2)

        if D != 0:
            acalpha = legs[0] / D 
            if acalpha > 1:
                acalpha = 1
            elif acalpha < -1:
                acalpha = -1
        else:
            acalpha = 1

        alpha1 = np.around(acos(acalpha), 2)
        
        if sqrt(dy**2 + dz**2) != 0:
            asalpha = dy / sqrt(dy**2 + dz**2) 
            if asalpha > 1:
                asalpha = 1
            elif asalpha < -1:
                asalpha = 1
        else:
            alpha2 = 0
            
        alpha2 = np.around(asin(asalpha), 2)
        alpha = ((alpha1 + alpha2 * mode) - pi / 2)
        # print(alpha1, alpha2, alpha, D)

        # -------------------------------------------------

        # beta
        if (dz != 0):
            beta1 = atan(dx / dz)
        else: beta1 = 0

        if D**2 - legs[0]**2 > 0: 
            H = sqrt(D**2 - legs[0]**2)
        else:
            H = 0
            
        abeta = (H**2 + legs[1]**2 - legs[2]**2) / (2 * H * legs[1])
        
        if abeta > 1:
            abeta = 1
        elif abeta < -1:
            abeta = -1
        # print(f"abeta: {abeta}")
        beta2 = acos(abeta)
        beta_sum = beta2 + beta1 - pi / 4
        if beta_sum > 1.3:
            beta_sum = 1.3
        elif beta_sum < -1.3:
            beta_sum = -1.3
        beta = beta_sum
        # print(H, beta1, beta2)
        
        # -------------------------------------------------

        # gamma
        agamma = (legs[1]**2 + legs[2]**2 - H**2) / (2 * legs[1] * legs[2])
        if agamma > 1:
            agamma = 1
        elif agamma < -1:
            agamma = -1
        gamma = acos(agamma) - pi * 0.6
        # print(H, gamma)

        return np.around(np.array([alpha * mode, beta, gamma]), 2)

# if __name__ == "__main__":
#     node = Kinematics()

#     print(f"1 - b:{node.body_collision}")
#     print(f"FR:{node.pos_FR}")
#     print(f"JFR:{node.joint_FR}")

#     node.move_base(Point(0, 0, 5))

#     print(f"2 - b:{node.body_collision}")
#     print(f"FR:{node.pos_FR}")
#     print(f"JFR:{node.joint_FR}")

#     node.move_foot(node.pos_FR, Point(0, 1, 0))

#     print(f"3 - b:{node.body_collision}")
#     print(f"FR:{node.pos_FR}")
#     print(f"JFR:{node.joint_FR}")