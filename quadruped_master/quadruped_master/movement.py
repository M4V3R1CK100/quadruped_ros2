#!/usr/bin/env python3
import rclpy
import sys
import threading
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np
import math
from math import pi, cos, sin, acos, atan2
from rclpy.node import Node
import time
from gait_functions import *

global current_pos
current_pos = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
global quit
quit = 0

class MyNode(Node):
    def __init__(self):
        super().__init__("movement_node")
        self.get_logger().info("Nodo creado")
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.01, self.send_joint_states)
        self.node_joint_states = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def send_joint_states(self):
        joint_state = JointState()
        joint_state.position = self.node_joint_states
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['front_right_joint1', 'front_right_joint2', 'front_left_joint1', 'front_left_joint2', 
                            'back_left_joint1', 'back_left_joint2', 'back_right_joint1', 'back_right_joint2']
        self.joint_state_pub.publish(joint_state)

def movement(speed):
    min_speed = 0
    max_speed = 5
    delay_min = 0.1  # en s
    delay_max = 0.5  # en s
    length = 0.15

    if speed < 0:
        length = -length

    time_delay = delay_max - ((abs(speed) - min_speed) / (max_speed - min_speed)) * (delay_max - delay_min)
    print(time_delay)
    print(speed)

    while speed != 0:
        print("Traslation")
        dummy_traslation(-length/2, 0, time_delay)
        time.sleep(0.05)
        print("Front Gait")
        if speed > 0:  # Cuando Avanza
            gait(1, length, time_delay)
            gait(2, length, time_delay)
            dummy_traslation(4 * length / 2, 0, time_delay)
            time.sleep(1)
            gait(3, length, time_delay)
            gait(4, length, time_delay)
        if speed < 0:  # Cuando retrocede
            gait(4, length, time_delay)
            gait(3, length, time_delay)
            dummy_traslation(4 * length / 2, 0, time_delay)
            time.sleep(1)
            gait(2, length, time_delay)
            gait(1, length, time_delay)
        dummy_traslation(-length/2, 0, time_delay)

def user_input_loop(node):
    global quit, current_position
    stand_1 = 0.3
    stand_2 = 1.4
    while quit == 0:
        imp = input("Enter number: ")
        number = int(imp)
        if number == 1:
            print("Stand")
            joint_position_state = [stand_1, stand_2, stand_1, stand_2, stand_1, stand_2, stand_1, stand_2]
            node.node_joint_states = joint_position_state
            current_position = joint_position_state
        elif number == 2:
            print("Up position")
            joint_position_state = [-1, 2.2, -1, 2.2, -1, 2.2, -1, 2.2]
            node.node_joint_states = joint_position_state
            time.sleep(1)
            joint_position_state = [stand_1, stand_2, stand_1, stand_2, stand_1, stand_2, stand_1, stand_2]
            node.node_joint_states = joint_position_state
            current_position = joint_position_state
        elif number == 3:
            time.sleep(1)
            user = input("Enter Speed: ")
            velocity = int(user)
            movement(velocity)
        elif number == 4:
            time.sleep(1)
            dummy_traslation(0.1, 0, 0.1)
        elif number == 5:
            quit = 1

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    print(dummy_traslation(0.0,0.1,0,node.node_joint_states))
    # Spin the node to handle ROS messages
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    
    main()
