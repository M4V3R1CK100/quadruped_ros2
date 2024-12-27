#!/usr/bin/env python3

from time import sleep
from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node
from dynamixel_sdk import *
import math


class DynamixelMotors(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f"{name} creado")

        self.joint_subscription = self.create_subscription(JointState, '/joint_goals', self.joint_callback, 10)

        # Control table address
        #READ AND WRITE
        self.ADDR_PRO_ACCELERATION_LIMIT   = 40
        self.ADDR_PRO_VELOCITY_LIMIT       = 44
        self.ADDR_PRO_TORQUE_ENABLE        = 64               # Control table address is different in Dynamixel model
        self.ADDR_PRO_LED                  = 65
        self.ADDR_PRO_POSITION_D_GAIN      = 80
        self.ADDR_PRO_POSITION_I_GAIN      = 82
        self.ADDR_PRO_POSITION_P_GAIN      = 84

        self.ADDR_PRO_GOAL_VELOCITY        = 104
        self.ADDR_PRO_PROFILE_ACCELERATION = 108
        self.ADDR_PRO_PROFILE_VELOCITY     = 112
        self.ADDR_PRO_GOAL_POSITION        = 116

        #ONLY READ
        self.ADDR_PRO_PRESENT_VELOCITY     = 128
        self.ADDR_PRO_PRESENT_POSITION     = 132
        self.ADDR_PRO_PRESENT_TEMPERATURE  = 146

        # Protocol version
        self.PROTOCOL_VERSION              = 2.0               # See which protocol version is used in the Dynamixel
        # Default setting
        self.DXL_ID0                       = [0,1,2,5,6]         # Dynamixel ID : 1
        self.DXL_ID1                       = [3,4,7,8]
        self.DXL_ID                        = [1,2,3,4,5,6,7,8]

        self.BAUDRATE                      = 1000000           # Dynamixel default baudrate : 57600

        self.DEVICENAME0                   = '/dev/ttyUSB0'    # Check which port is being used on your controller
        self.DEVICENAME1                   = '/dev/ttyUSB1'    # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
        
        self.ALL_PORTS = 2
        self.PORT_0    = 0
        self.PORT_1    = 1

        self.present_positions = [0,0,0,0,0,0,0,0,0]


        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler0  = PortHandler  (self.DEVICENAME0)
        self.portHandler1  = PortHandler  (self.DEVICENAME1)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        # Initialize GroupSyncWrite/Read instance
        self.groupSyncWritePos0 = GroupSyncWrite(self.portHandler0, self.packetHandler, self.ADDR_PRO_PRESENT_POSITION, 4)
        self.groupSyncWritePos1 = GroupSyncWrite(self.portHandler1, self.packetHandler, self.ADDR_PRO_PRESENT_POSITION, 4)
        self.groupSyncReadPos0  = GroupSyncRead (self.portHandler0, self.packetHandler, self.ADDR_PRO_PRESENT_POSITION, 4)
        self.groupSyncReadPos1  = GroupSyncRead (self.portHandler1, self.packetHandler, self.ADDR_PRO_PRESENT_POSITION, 4)


        for id in self.DXL_ID0:
            # Add parameter storage for Dynamixel present position value
            dxl_addparam_resultPos = self.groupSyncReadPos0.addParam(id)

            if dxl_addparam_resultPos != True:
                self.get_logger().error(f"[ID: {id}] groupSyncReadPos0 addparam failed")

        for id in self.DXL_ID1:
            # Add parameter storage for Dynamixel present position value
            dxl_addparam_resultPos = self.groupSyncReadPos1.addParam(id)

            if dxl_addparam_resultPos != True:
                self.get_logger().error(f"[ID: {id}] groupSyncReadPos1 addparam failed")


    def comunication(self, n: int = 2):
        """
        Establece comunicación con los motores.

        Args:
            n (int): Indica los puertos a los que se aplicará el cambio (0, 1, o 2 para ambos).
        """
        def setup_port(portHandler: PortHandler, dxl_ids: list):
            if portHandler.openPort():
                self.get_logger().info("Succeeded to open the port")
            else:
                self.get_logger().error("Failed to open the port")

            if portHandler.setBaudRate(self.BAUDRATE):
                self.get_logger().info("Succeeded to change the baudrate")
            else:
                self.get_logger().error("Failed to change the baudrate")

            for i in dxl_ids:
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(portHandler, i, self.ADDR_PRO_TORQUE_ENABLE, 0)
                if dxl_comm_result != COMM_SUCCESS:
                    self.get_logger().error(f"Error {self.packetHandler.getTxRxResult(dxl_comm_result)} on Dynamixel {i}")
                elif dxl_error != 0:
                    self.get_logger().error(f"Packet error {self.packetHandler.getRxPacketError(dxl_error)} on Dynamixel {i}")
                else:
                    self.get_logger().info(f"Dynamixel {i} has been successfully connected")

        if n == self.ALL_PORTS:
            setup_port(self.portHandler0, self.DXL_ID0)
            setup_port(self.portHandler1, self.DXL_ID1)
        elif n == self.PORT_1:
            setup_port(self.portHandler1, self.DXL_ID1)
        else:
            setup_port(self.portHandler0, self.DXL_ID0)


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

        if n == self.ALL_PORTS:
            port_torque(self.portHandler0, self.DXL_ID0)
            port_torque(self.portHandler1, self.DXL_ID1)
        elif n == self.PORT_1:
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
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler0, i, self.ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler0, i, self.ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler0, i, self.ADDR_PRO_POSITION_D_GAIN, set_D_Gain)
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f"{self.packetHandler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                self.get_logger().error(f"{self.packetHandler.getRxPacketError(dxl_error)}")
            else:
                self.get_logger().info(f"Dynamixel: {i} has been successfully PID configuration")
        for i in self.DXL_ID1:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler1, i, self.ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler1, i, self.ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler1, i, self.ADDR_PRO_POSITION_D_GAIN, set_D_Gain)
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f"{self.packetHandler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                self.get_logger().error(f"{self.packetHandler.getRxPacketError(dxl_error)}")
            else:
                self.get_logger().info(f"Dynamixel: {i} has been successfully PID configuration")


    def read_positions(self):

        def process_sync_read(groupSyncRead: GroupSyncRead, dxl_ids: list, present_positions: list):
            for id in dxl_ids:
                # Check if groupsyncread data is available
                if not groupSyncRead.isAvailable(id, self.ADDR_PRO_PRESENT_POSITION, 4):
                    self.get_logger().error(f"[ID:{id}] groupSyncRead getdata failed")
                    continue
                # Get Dynamixel present position value
                dxl_present_position = motor_angle_to_radian(
                    groupSyncRead.getData(id, self.ADDR_PRO_PRESENT_POSITION, 4)
                )
                present_positions[id] = dxl_present_position
            return present_positions
        
        def process_sync_read2(groupSyncRead: GroupSyncRead, dxl_ids: list, present_positions: list):
            for id in dxl_ids:
                # Check if groupsyncread data is available
                if not groupSyncRead.isAvailable(id, self.ADDR_PRO_PRESENT_POSITION, 4):
                    self.get_logger().error(f"[ID:{id}] groupSyncRead getdata failed")
                    continue
                # Get Dynamixel present position value
                dxl_present_position = groupSyncRead.getData(id, self.ADDR_PRO_PRESENT_POSITION, 4)
                present_positions[id] = dxl_present_position
            return present_positions

        def motor_angle_to_radian(motor_angle_val):

            min_value_angle = 0
            max_value_angle = 4095 
            min_angle_deg = 0
            max_angle_deg = 360

            zero_value_angle = (max_value_angle-min_value_angle)//2 + 1
            deg_movement_span_2 = (max_angle_deg-min_angle_deg)//2
            rad_movement_span_2 = float(math.radians(deg_movement_span_2))        

            if (motor_angle_val == zero_value_angle):
                return 0.0
            else:
                return ((float)(motor_angle_val-zero_value_angle)/(float)(zero_value_angle))*rad_movement_span_2

        def motor_angle_to_radian2(motor_positions: list):
                #1 degree ~ 90
                offset1 = -410
                offset2 = -80
                offset3 = 167
                offset4 = -300
                offset5 = 0
                offset6 = -300
                offset7 = -168
                offset8 = -120

                motor_positions[1] = math.radians((20475.0 + offset1 -motor_positions[1])*(15.0/120.0)*(360.0/4095.0)-90.0)
                motor_positions[2] = math.radians((16380.0 + offset2 -motor_positions[2])*(15.0/120.0)*(360.0/4095.0))
                motor_positions[3] = math.radians(((motor_positions[3]-offset3)*(15.0/120.0)*(360.0/4095.0))-90.0)
                motor_positions[4] = math.radians(((motor_positions[4]-offset4)*(15.0/120.0)*(360.0/4095.0))-45.0)
                motor_positions[5] = math.radians((20475.0 + offset5 -motor_positions[5])*(15.0/120.0)*(360.0/4095.0)-90.0)
                motor_positions[6] = math.radians((16380.0 + offset6 -motor_positions[6])*(15.0/120.0)*(360.0/4095.0))
                motor_positions[7] = math.radians(((motor_positions[7]-offset7)*(15.0/120.0)*(360.0/4095.0))-90.0)
                motor_positions[8] = math.radians(((motor_positions[8]-offset8)*(15.0/120.0)*(360.0/4095.0))-45.0)

                return motor_positions

        # Syncread present position
        dxl_comm_result = self.groupSyncReadPos0.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f"Falla en get_positions: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        dxl_comm_result = self.groupSyncReadPos1.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f"Falla en get_positions: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        
        self.present_positions = process_sync_read(self.groupSyncReadPos0, self.DXL_ID0, self.present_positions)
        self.present_positions = process_sync_read(self.groupSyncReadPos1, self.DXL_ID1, self.present_positions)

        # self.present_positions = motor_angle_to_radian2(self.present_positions)
        return self.present_positions


    def set_motor_positions(self, goal_positions: list):
        def map_range(x, in_min, in_max, out_min, out_max):
            return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

        def move_sync_motors(goal_positions: list, groupSyncWritePos: GroupSyncWrite, dxl_ids: list):
            for id in dxl_ids:
                new_pos = map_range(goal_positions[id], -math.pi/2, math.pi/2, 0, 4095)
                param_goal_pos = [DXL_LOBYTE(DXL_LOWORD(new_pos)), 
                                  DXL_HIBYTE(DXL_LOWORD(new_pos)), 
                                  DXL_LOBYTE(DXL_HIWORD(new_pos)), 
                                  DXL_HIBYTE(DXL_HIWORD(new_pos))]
                # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
                dxl_addparam_result = groupSyncWritePos.addParam(id, param_goal_pos)
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupSyncWritePWM addparam failed" % id)
                self.get_logger().error(f"Goal: {goal_positions[id]}, Param: {new_pos}")


            # dxl_comm_result = groupSyncWritePos.txPacket()
            # if dxl_comm_result != COMM_SUCCESS:
            #     print("Hubo un fallo en envío de PWM")
            #     print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

            # Clear syncwrite parameter storage
            groupSyncWritePos.clearParam()

        move_sync_motors(goal_positions, self.groupSyncReadPos0, self.DXL_ID0)
        move_sync_motors(goal_positions, self.groupSyncReadPos1, self.DXL_ID1)

    def joint_callback(self, msg: JointState):

        motor_goals = list(msg.position)

        self.set_motor_positions(motor_goals)




def main(args=None):
    rclpy.init(args=args)
    node = DynamixelMotors("vrep_communication")
    node.comunication()
    node.torque(turn_on=True)
    node.pid_gain_position_loop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.torque(turn_on=False)

    node.destroy_node()

    rclpy.shutdown()













