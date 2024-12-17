#!/usr/bin/env python3
import rclpy
import sys
import threading
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
from math import pi, cos, sin, acos, atan2
from rclpy.node import Node
import time
from quadruped_master.gait_functions import *
from quadruped_interfaces.msg import MotionParams

global quit
quit = 0

class MyNode(Node):
    def __init__(self):
        super().__init__("movement_node")
        self.get_logger().info("Nodo creado")
        self.joint_states_pub = self.create_publisher(JointState, '/joint_goals', 10)
        self.motion_sub = self.create_subscription(MotionParams, "motion_params", self.update_motion_params, 10)
        self.motion_params = MotionParams()
        self.rotation = 0.0
        self.motion = 0
        self.traslx = 0.0
        self.traslz = 0.0

        #Joint states del nodo
        self.node_joint_states = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.joint_states = JointState()
        self.joint_states.position = self.node_joint_states
        self.joint_states.header = Header()
        self.joint_states.name = ['camera_joint','front_right_joint1', 'front_right_joint2', 'front_left_joint1', 'front_left_joint2', 
                                  'back_left_joint1', 'back_left_joint2','back_right_joint1', 'back_right_joint2']
        
        # Timer para procesar movimientos a intervalos regulares
        self.timer = self.create_timer(0.1, self.process_movement)
        self.joint_states.header.stamp = self.get_clock().now().to_msg()
        self.joint_states_pub.publish(self.joint_states)

    def update_motion_params(self, msg):
        self.motion_params = msg

    def process_movement(self):
        # Procesar movimiento en función de los parámetros
        if self.motion_params.motion == 1.0 and self.motion_params.motion!=self.motion:
            initial_position(self)
            self.motion =1
        if self.motion==1:
            if self.motion_params.speed != 0.0:
                gaits(self, self.motion_params.speed)

            elif self.motion_params.traslation_x != self.traslx or self.motion_params.traslation_z != self.traslz or self.motion_params.rotation!= self.rotation:
                plan = dummy_traslation(self.motion_params.traslation_x-self.traslx, self.motion_params.traslation_z-self.traslz,self.motion_params.rotation - self.rotation, self.node_joint_states, self.rotation)
                self.rotation = self.motion_params.rotation 
                self.traslx = self.motion_params.traslation_x
                self.traslz = self.motion_params.traslation_z
                self.send_joint_states(plan, 0.2)

    def send_joint_states(self, joint_goals_list, time_delay):
        for goals in joint_goals_list:
            self.node_joint_states = goals
            self.joint_states.position = goals
            self.joint_states.header.stamp = self.get_clock().now().to_msg()  # Actualiza el timestamp antes de publicar
            self.joint_states_pub.publish(self.joint_states)
            time.sleep(time_delay)

    def publish_joint_states(self, joint_positions: list):
        self.node_joint_states = joint_positions
        self.joint_states.position = joint_positions
        self.joint_states.header.stamp = self.get_clock().now().to_msg()
        self.joint_states_pub.publish(self.joint_states)
        
def initial_position(node: MyNode):
    stand_1=0.3
    stand_2=1.4
    # up
    joint_position_state=[0.0, -1.0, 2.2, -1.0, 2.2, -1.0, 2.2, -1.0, 2.2]
    node.publish_joint_states(joint_position_state)

    time.sleep(4)
    joint_position_state=[0.0, stand_1, stand_2,stand_1,stand_2,stand_1,stand_2,stand_1,stand_2] # stand up principal
    node.publish_joint_states(joint_position_state)

def gaits(node: MyNode, f_speed):
    plan       = []
    min_speed  = 0
    max_speed  = 1
    delay_min  = 0.1  # en s
    delay_max  = 0.5  # en s
    length     = 0.15
    if f_speed < 0:   
        length = -length

    time_delay = delay_max - ((abs(f_speed) - min_speed) / (max_speed - min_speed)) * (delay_max - delay_min)
    print("Traslation")
    plan = dummy_traslation(-length/4, 0, 0, node.node_joint_states, node.motion_params.rotation)
    node.send_joint_states(plan,time_delay)

    time.sleep(0.05)
    print("Front Gait")
    if f_speed > 0:  # Cuando Avanza
        plan = gait(1, length, node.node_joint_states)
        node.send_joint_states(plan,time_delay)

        plan = gait(2, length, node.node_joint_states)
        node.send_joint_states(plan,time_delay)

        plan = dummy_traslation( 2*length / 4 + length, 0, 0 , node.node_joint_states, node.motion_params.rotation)
        node.send_joint_states(plan,time_delay)

        time.sleep(1)
        plan = gait(3, length, node.node_joint_states)
        node.send_joint_states(plan,time_delay)
        plan = gait(4, length, node.node_joint_states)
        node.send_joint_states(plan,time_delay)
    if f_speed < 0:  # Cuando retrocede
        plan = gait(4, length, node.node_joint_states)
        node.send_joint_states(plan,time_delay)

        plan = gait(3, length, node.node_joint_states)
        node.send_joint_states(plan,time_delay)

        plan = dummy_traslation(2*length/4 + length, 0, 0, node.node_joint_states, node.motion_params.rotation)
        node.send_joint_states(plan,time_delay)

        time.sleep(1)
        plan = gait(2, length, node.node_joint_states)
        node.send_joint_states(plan,time_delay)

        plan = gait(1, length, node.node_joint_states)
        node.send_joint_states(plan,time_delay)

    plan = dummy_traslation(-length/4, 0, 0,node.node_joint_states, node.motion_params.rotation)
    node.send_joint_states(plan,time_delay)


def main(args=None):
    rclpy.init(args=args)
    movement_node = MyNode()

    #Home(movement_node)
    
    # Crear un hilo para user_input_loop
    # user_input_loop(movement_node)

    # Ejecutar el nodo ROS
    rclpy.spin(movement_node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()