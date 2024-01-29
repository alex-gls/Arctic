#!/usr/bin/env python3

import rospy
from dynamixel_sdk import *

class MX_28AT():

    def __init__(self) -> None:

        self.ADDR_TORQUE_ENABLE          = 64
        self.ADDR_GOAL_POSITION          = 116
        self.ADDR_LED                    = 65
        self.LEN_GOAL_POSITION           = 4         # Data Byte Length
        self.ADDR_PRESENT_POSITION       = 132
        self.LEN_PRESENT_POSITION        = 4         # Data Byte Length
        self.DXL_MINIMUM_POSITION_VALUE  = 1500      # Refer to the Minimum Position Limit of product eManual
        self.DXL_MAXIMUM_POSITION_VALUE  = 2500      # Refer to the Maximum Position Limit of product eManual
        self.BAUDRATE                    = 1_000_000

        self.PROTOCOL_VERSION            = 2.0

        self.TORQUE_ENABLE               = 1                 # Value for enabling the torque
        self.TORQUE_DISABLE              = 0                 # Value for disabling the torque
        self.DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

        self.DXL_LIST = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
        # self.DXL_LIST = [1, 2, 3]

        self.DEVICENAME                  = '/dev/ttyS2'

        self.start = False

        pass

    def Connect(self):
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        # Initialize GroupSyncWrite instance
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_GOAL_POSITION, self.LEN_GOAL_POSITION)

        # Initialize GroupSyncRead instace for Present Position
        self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate")
            self.start = True
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            quit()

        print("initialization dynamixels")


    def sendvalue(self, value_list: list) -> None:
        """
        This metod send syncronaze message for dynamixels in class
        """
        if self.start:
            self.SET_TORQUE(1)
            print(len(self.DXL_LIST))
            for index, ID in enumerate(self.DXL_LIST):
                # # Add parameter storage for Dynamixel#1 present position value
                # dxl_addparam_result = self.groupSyncRead.addParam(ID)
                # if dxl_addparam_result != True:
                #     print("[ID:%03d] groupSyncRead addparam failed" % ID)
                #     break

                # Allocate goal position value into byte array
                param_goal_position = [DXL_LOBYTE(DXL_LOWORD(value_list[index])), DXL_HIBYTE(DXL_LOWORD(value_list[index])), DXL_LOBYTE(DXL_HIWORD(value_list[index])), DXL_HIBYTE(DXL_HIWORD(value_list[index]))]
                
                # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
                dxl_addparam_result = self.groupSyncWrite.addParam(ID, param_goal_position)
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % ID)
                    quit()

            # Syncwrite goal position
            dxl_comm_result = self.groupSyncWrite.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            
            # Clear syncwrite parameter storage
            self.groupSyncWrite.clearParam()
    
    def closePort(self) -> None:
        """
        This metod is close port for dynamixel
        """
        if self.portHandler.openPort():
            self.portHandler.closePort()

    def SET_TORQUE(self, value: int) -> None:
        """
        This metod is install value torque (0/1) for dynamixels in class
        """
        if self.start:
            self.SET_LED(value)
            for ID in self.DXL_LIST:
                # Enable Dynamixel's Torque
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_TORQUE_ENABLE, value)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                    break
                elif dxl_error != 0:
                    print("%s" % self.packetHandler.getRxPacketError(dxl_error))
                    break
                else:
                    pass
                    # print("Dynamixel#%d has been successfully connected" % ID)
    
    def SET_LED(self, value: int) -> None:
        """
        This metod is install value torque (0/1) for dynamixels in class
        """
        if self.start:
            for ID in self.DXL_LIST:
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_LED, value)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                    break
                elif dxl_error != 0:
                    print("%s" % self.packetHandler.getRxPacketError(dxl_error))
                    break
                else:
                    pass

    def SET_LIMIT(self, min: int, max: int, ID: int) -> None:
        """
        This metod is install limit 0~4095 (min | max) for dynamixel in class
        """
        if self.start:
            
            self.ADDR_MIN_LIMIT = 52
            self.ADDR_MAX_LIMIT = 48

            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, ID, self.ADDR_MIN_LIMIT, min)
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, ID, self.ADDR_MAX_LIMIT, max)
            
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                pass



