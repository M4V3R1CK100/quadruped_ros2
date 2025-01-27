#!/usr/bin/env python
# -*- coding: utf-8 -*-

# With this program we read and write the Dynamixel Motores using SDK
import rclpy
from rclpy.node import Node
from dynamixel_sdk import *
import math
import os

from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from quadruped_interfaces.msg import MotorData

class DynamixelMotors(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f"{name} creado")
        self.joint_subscription   = self.create_subscription(JointState, '/joint_goals', self.callback, 10)
        self.publisher_motor_data = self.create_publisher   (MotorData,  '/motor_data' , 10)
        # Control table address
        #READ AND WRITE
        self.ADDR_PRO_ACCELERATION_LIMIT = 40
        self.ADDR_PRO_VELOCITY_LIMIT     = 44
        self.ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
        self.ADDR_PRO_LED                = 65
        self.ADDR_PRO_POSITION_D_GAIN    = 80
        self.ADDR_PRO_POSITION_I_GAIN    = 82
        self.ADDR_PRO_POSITION_P_GAIN    = 84

        self.ADDR_PRO_GOAL_VELOCITY      = 104
        self.ADDR_PRO_PROFILE_ACCELERATION = 108
        self.ADDR_PRO_PROFILE_VELOCITY   = 112
        self.ADDR_PRO_GOAL_POSITION      = 116

        #ONLY READ
        self.ADDR_PRO_PRESENT_CURRENT    = 126
        self.ADDR_PRO_PRESENT_VELOCITY   = 128
        self.ADDR_PRO_PRESENT_POSITION   = 132
        self.ADDR_PRO_PRESENT_TEMPERATURE= 146

        # Protocol version
        self.PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel
        # Default setting
        self.DXL_ID1                      = [0,1,2,5,6]                 # Dynamixel ID : 1
        self.DXL_ID0                      = [3,4,7,8]
        self.DXL_ID                       = [0,1,2,3,4,5,6,7,8]

        # BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
        self.BAUDRATE                    = 57600

        self.DEVICENAME0                 = '/dev/ttyUSB0'    # Check which port is being used on your controller
        self.DEVICENAME1                 = '/dev/ttyUSB1'    # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler0  = PortHandler(self.DEVICENAME0)
        self.portHandler1  = PortHandler(self.DEVICENAME1)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        self.groupSyncReadPos0 = GroupSyncRead(self.portHandler0, self.packetHandler, self.ADDR_PRO_PRESENT_POSITION, 4)
        self.groupSyncReadPos1 = GroupSyncRead(self.portHandler1, self.packetHandler, self.ADDR_PRO_PRESENT_POSITION, 4)



        self.theta0=0
        self.theta1=0
        self.theta2=0
        self.theta3=0
        self.theta4=0
        self.theta5=0
        self.theta6=0
        self.theta7=0
        self.theta8=0

        self.motors = MotorData()

        self.create_timer(0.1, self.read_positions)

    def comunication(self, n: int = 2):
        """
        Establece comunicación con los motores.

        Args:
            n (int): Indica los puertos a los que se aplicará el cambio (0, 1, o 2 para ambos).
        """
        def setup_port(portHandler: PortHandler, dxl_ids: list, groupSyncReadPos: GroupSyncRead):
            if portHandler.openPort():
                self.get_logger().info("Succeeded to open the port HOLA")
            else:
                self.get_logger().error("Failed to open the port")

            if portHandler.setBaudRate(self.BAUDRATE):
                self.get_logger().info("Succeeded to change the baudrate")
            else:
                self.get_logger().error("Failed to change the baudrate")

            for i in dxl_ids:
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(portHandler, i, self.ADDR_PRO_TORQUE_ENABLE, 0)
                if dxl_comm_result != COMM_SUCCESS:
                    self.get_logger().error(f"Error j{self.packetHandler.getTxRxResult(dxl_comm_result)} on Dynamixel {i}")
                elif dxl_error != 0:
                    self.get_logger().error(f"Packet error {self.packetHandler.getRxPacketError(dxl_error)} on Dynamixel {i}")
                else:
                    self.get_logger().info(f"Dynamixel {i} has been successfully connected")

            for id in dxl_ids:
                # Add parameter storage for Dynamixel present position value
                dxl_addparam_resultPos = groupSyncReadPos.addParam(id)
                if dxl_addparam_resultPos != True:
                    print("[ID:%03d] groupSyncReadPos addparam failed" % id)

        if n == 2:
            setup_port(self.portHandler0, self.DXL_ID0, self.groupSyncReadPos0)
            setup_port(self.portHandler1, self.DXL_ID1, self.groupSyncReadPos1)
        elif n == 1:
            setup_port(self.portHandler1, self.DXL_ID1, self.groupSyncReadPos1)
        else:
            setup_port(self.portHandler0, self.DXL_ID0, self.groupSyncReadPos0)

    def current(self, DXL_ID,portHandler):
        # print("current:")
        for i in DXL_ID:
            dxl_present_current, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(portHandler, i, self.ADDR_PRO_PRESENT_CURRENT)
            if int(dxl_present_current) >= 32767:
                if 65535-int(dxl_present_current) >= 50:
                    print("Motor ",i," is off",65535-int(dxl_present_current))
                    self.packetHandler.write1ByteTxRx(portHandler, i, self.ADDR_PRO_TORQUE_ENABLE, 0)
            else:
                if int(dxl_present_current) >= 50:
                    print("Motor ",i," is off",int(dxl_present_current))
                    self.packetHandler.write1ByteTxRx(portHandler, i, self.ADDR_PRO_TORQUE_ENABLE, 0)

    def torque(self, turn_on: bool, n: int = 2):
        """
        Enciende/apaga el torque de los motores.

        Args:
            turn_on (bool):  True para encender torques  y False para apagarlos
            n (int): Indica los puertos a los que se aplicará el cambio (0, 1, o 2 para ambos).
        """

        def port_torque(portHandler: PortHandler, dxl_ids: list):
            for i in dxl_ids:
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(portHandler, i, self.ADDR_PRO_TORQUE_ENABLE, turn_on)
                if dxl_comm_result != COMM_SUCCESS:
                    self.get_logger().error(f"{self.packetHandler.getTxRxResult(dxl_comm_result)} on motor {i}")
                elif dxl_error != 0:
                    self.get_logger().error(f"{self.packetHandler.getRxPacketError(dxl_error)} on motor {i}")
                else:
                    state = "on" if turn_on else "off"
                    self.get_logger().warn(f"Torque of Motor {i} is {state}")

        if n == 2:
            port_torque(self.portHandler0, self.DXL_ID0)
            port_torque(self.portHandler1, self.DXL_ID1)
        elif n == 1:
            port_torque(self.portHandler1, self.DXL_ID1)
        else:
            port_torque(self.portHandler0, self.DXL_ID0)

    def pid_gain_position_loop(self):
        # set_P_Gain = 500
        # set_I_Gain = 100
        # set_D_Gain = 4700
        set_P_Gain = 2000
        set_I_Gain = 10
        set_D_Gain = 10
        for i in self.DXL_ID0:
            if i!=0:
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler0, i, self.ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler0, i, self.ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler0, i, self.ADDR_PRO_POSITION_D_GAIN, set_D_Gain)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % self.packetHandler.getRxPacketError(dxl_error))
                else:
                    print("Dynamixel: ",i," has been successfully PID configuration")
        for i in self.DXL_ID1:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler1, i, self.ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler1, i, self.ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler1, i, self.ADDR_PRO_POSITION_D_GAIN, set_D_Gain)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel: ",i," has been successfully PID configuration")

    def pid_gain_velocity_loop(self):
        set_A_l = 80
        set_V_l = 80

        set_A_PRFL = 10
        set_V_PRFL = 10
        for i in self.DXL_ID0:
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler0, i, self.ADDR_PRO_VELOCITY_LIMIT, int(set_V_l))
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler0, i, self.ADDR_PRO_ACCELERATION_LIMIT, int(set_A_l))
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler0, i, self.ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler0, i, self.ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel: ",i," has been successfully velocity and acceleration configuration")
        for i in self.DXL_ID1:
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler1, i, self.ADDR_PRO_VELOCITY_LIMIT, int(set_V_l))
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler1, i, self.ADDR_PRO_ACCELERATION_LIMIT, int(set_A_l))
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler1, i, self.ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler1, i, self.ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel: ",i," has been successfully velocity and acceleration configuration")

    def convert_motor_to_pos(self, motor_positions: list):
        #1 degree ~ 90
        offset1 = 50
        offset2 = -80
        offset3 = -300 + 120
        offset4 = 10 - 320
        offset5 = 0
        offset6 = -200
        offset7 = 150 - 150
        offset8 = -120

        motor_positions_converted = [0,0,0,0,0,0,0,0,0]
        motor_positions_converted[0] = motor_positions[0] * (360.0 / 4095.0)
        motor_positions_converted[1] = (((20475.0 - 4095*3 + offset1) - motor_positions[1]) * (15.0 / 120.0) * (360.0 / 4095.0)) - 90.0
        motor_positions_converted[2] = (((16380.0 - 4095*4 + offset2) - motor_positions[2]) * (15.0 / 120.0) * (360.0 / 4095.0))
        motor_positions_converted[3] = (((motor_positions[3] - offset3 + 4095) * (15.0 / 120.0) * (360.0 / 4095.0))) - 90.0
        motor_positions_converted[4] = ((motor_positions[4] - offset4) * (15.0 / 120.0) * (360.0 / 4095.0)) - 45.0
        motor_positions_converted[5] = (((20475.0 - 4095*3 + offset5) - motor_positions[5]) * (15.0 / 120.0) * (360.0 / 4095.0)) - 90.0
        motor_positions_converted[6] = (((16380.0 - 4095*4 + offset6) - motor_positions[6]) * (15.0 / 120.0) * (360.0 / 4095.0))
        motor_positions_converted[7] = (((motor_positions[7] - offset7 + 4095) * (15.0 / 120.0) * (360.0 / 4095.0))) - 90.0
        motor_positions_converted[8] = ((motor_positions[8] - offset8) * (15.0 / 120.0) * (360.0 / 4095.0)) - 45.0

        return motor_positions_converted

    def read_positions(self):
        # Read Dynamixel present positions
        dxl0_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler1, 0, self.ADDR_PRO_PRESENT_POSITION)
        dxl1_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler1, 1, self.ADDR_PRO_PRESENT_POSITION)
        dxl2_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler1, 2, self.ADDR_PRO_PRESENT_POSITION)
        dxl3_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler0, 3, self.ADDR_PRO_PRESENT_POSITION)
        dxl4_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler0, 4, self.ADDR_PRO_PRESENT_POSITION)
        dxl5_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler1, 5, self.ADDR_PRO_PRESENT_POSITION)
        dxl6_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler1, 6, self.ADDR_PRO_PRESENT_POSITION)
        dxl7_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler0, 7, self.ADDR_PRO_PRESENT_POSITION)
        dxl8_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler0, 8, self.ADDR_PRO_PRESENT_POSITION)

        motor_position_converted = self.convert_motor_to_pos([dxl0_present_position, dxl1_present_position, dxl2_present_position, dxl3_present_position, dxl4_present_position, dxl5_present_position, dxl6_present_position, dxl7_present_position, dxl8_present_position])

        motors = MotorData()
        motors.goal_position = [self.theta0, self.theta1, self.theta2, self.theta3, self.theta4, self.theta5, self.theta6, self.theta7, self.theta8]
        motors.pres_position = [motor_position_converted[0], motor_position_converted[1], motor_position_converted[2], motor_position_converted[3], motor_position_converted[4], motor_position_converted[5], motor_position_converted[6], motor_position_converted[7], motor_position_converted[8]]
        self.publisher_motor_data.publish(motors)

    def read_positions_sync(self):
        present_positions = [0,0,0,0,0,0,0,0,0]
        # Syncread present position
        dxl_comm_result = self.groupSyncReadPos0.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("Falla en get_positions: %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        
        for id in self.DXL_ID0:
            # Check if groupsyncread data of Dynamixel#1 is available
            dxl_getdata_result = self.groupSyncReadPos0.isAvailable(id, self.ADDR_PRO_PRESENT_POSITION , 4)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncReadPos getdata failed" % id)
            # Get Dynamixel present position value
            dxl_present_position = self.groupSyncReadPos0.getData(id, self.ADDR_PRO_PRESENT_POSITION , 4)
            present_positions[id]=dxl_present_position

        for id in self.DXL_ID1:
            # Check if groupsyncread data of Dynamixel#1 is available
            dxl_getdata_result = self.groupSyncReadPos1.isAvailable(id, self.ADDR_PRO_PRESENT_POSITION , 4)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncReadPos getdata failed" % id)
            # Get Dynamixel present position value
            dxl_present_position = self.groupSyncReadPos1.getData(id, self.ADDR_PRO_PRESENT_POSITION , 4)
            present_positions[id]=dxl_present_position

        motor_position_converted = self.convert_motor_to_pos(present_positions)

        
        self.motors.goal_position = [self.theta0, self.theta1, self.theta2, self.theta3, self.theta4, self.theta5, self.theta6, self.theta7, self.theta8]
        self.motors.pres_position = [motor_position_converted[0], motor_position_converted[1], motor_position_converted[2], motor_position_converted[3], motor_position_converted[4], motor_position_converted[5], motor_position_converted[6], motor_position_converted[7], motor_position_converted[8]]
        self.publisher_motor_data.publish(self.motors)

    def movement(self):

        #1 degree ~ 90 /i did repair the motor 1 and we change for motor 1 and 7 position
        # offset1 = -410
        offset1 = 50
        offset2 = -80
        offset3 = -300 + 120
        offset4 = 10 - 320
        offset5 = 0
        offset6 = -200
        offset7 = 150 - 150
        offset8 = -120

        # print("[1]%.2f\t[2]%.2f\t[3]%.2f\t[4]%.2f\t[5]%.2f\t[6]%.2f\t[7]%.2f\t[8]%.2f " % (self.theta1,self.theta2,self.theta3,self.theta4,self.theta5,self.theta6,self.theta7,self.theta8))

        dxl0_goal_position = int(self.theta0/(360.0/4095.0))
        dxl1_goal_position = int(20475.0 - 4095*3 + offset1-((self.theta1+90.0)/((15.0/120.0)*(360.0/4095.0))))
        dxl2_goal_position = int(16380.0 - 4095*4 + offset2-((self.theta2)/((15.0/120.0)*(360.0/4095.0))))
        dxl3_goal_position = int(-4095 + ((self.theta3+90)/((15.0/120.0)*(360.0/4095.0)))+offset3)
        dxl4_goal_position = int(((self.theta4+45)/((15.0/120.0)*(360.0/4095.0)))+offset4)
        dxl5_goal_position = int(20475.0  - 4095*3 +  offset5-((self.theta5+90.0)/((15.0/120.0)*(360.0/4095.0))))
        dxl6_goal_position = int(16380.0 - 4095*4 + offset6-((self.theta6)/((15.0/120.0)*(360.0/4095.0))))
        dxl7_goal_position = int(-4095 + ((self.theta7+90)/((15.0/120.0)*(360.0/4095.0)))+offset7)
        dxl8_goal_position = int(((self.theta8+45)/((15.0/120.0)*(360.0/4095.0)))+offset8)

        # print("[1]%.2f\t[2]%.2f\t[3]%.2f\t[4]%.2f\t[5]%.2f\t[6]%.2f\t[7]%.2f\t[8]%.2f " % (dxl1_goal_position,dxl2_goal_position,dxl3_goal_position,dxl4_goal_position,dxl5_goal_position,dxl6_goal_position,dxl7_goal_position,dxl8_goal_position))
        print(self.theta0)

        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler1, 0, self.ADDR_PRO_GOAL_POSITION, dxl0_goal_position)
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler1, 1, self.ADDR_PRO_GOAL_POSITION, dxl1_goal_position)
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler1, 2, self.ADDR_PRO_GOAL_POSITION, dxl2_goal_position)
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler0, 3, self.ADDR_PRO_GOAL_POSITION, dxl3_goal_position)
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler0, 4, self.ADDR_PRO_GOAL_POSITION, dxl4_goal_position)
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler1, 5, self.ADDR_PRO_GOAL_POSITION, dxl5_goal_position)
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler1, 6, self.ADDR_PRO_GOAL_POSITION, dxl6_goal_position)
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler0, 7, self.ADDR_PRO_GOAL_POSITION, dxl7_goal_position)
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler0, 8, self.ADDR_PRO_GOAL_POSITION, dxl8_goal_position)

        # self.current(self.DXL_ID0,self.portHandler0)
        # self.current(self.DXL_ID1,self.portHandler1)

    def callback(self, data: JointState):
        #esta configuracion es con lo siguiente mensaje joint1 front and backs y despues los dos

        self.theta0 = data.position[0]*180/3.1416 + 180
        self.theta1 = data.position[1]*180/3.1416
        self.theta2 = data.position[2]*180/3.1416
        self.theta3 = data.position[3]*180/3.1416
        self.theta4 = data.position[4]*180/3.1416
        self.theta5 = data.position[5]*180/3.1416
        self.theta6 = data.position[6]*180/3.1416
        self.theta7 = data.position[7]*180/3.1416
        self.theta8 = data.position[8]*180/3.1416

        self.movement()
        self.get_logger().error(str(data._position))


def main(args=None):

    rclpy.init(args=args)
    node = DynamixelMotors("dynamixel_node")
    node.comunication()
    node.torque(turn_on=True)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.torque(turn_on=False)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()









