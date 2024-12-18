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
        self.speed = 0.0
        self.camera = 0.0
        
        self.is_executing = False   # Bandera para controlar la ejecución de gaits()


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
        print(msg)
        self.motion_params = msg
        self.speed = self.motion_params.speed

    def process_movement(self):
        """Controla el procesamiento del movimiento basado en los parámetros actuales."""
        # Si ya se está ejecutando un movimiento, no hacer nada
        if self.is_executing:
            return

        if self.motion_params.motion == 2:
            self.get_logger().info("Paro de emergencia detectado.")
            return

        if self.motion_params.motion != self.motion:
            # Cambió el tipo de movimiento
            if self.motion_params.motion == 0.0:
                rest(self)
            elif self.motion_params.motion == 1.0:
                initial_position(self)
            self.motion = self.motion_params.motion

        # Ejecutar gaits si la velocidad es distinta de 0
        if self.motion_params.motion == 1.0 and self.speed != 0.0:
            self.is_executing = True  # Marcar que estamos ejecutando
            gaits(self, self.speed)
            self.is_executing = False  # Liberar bandera al finalizar

        elif self.motion_params.traslation_x != self.traslx or self.motion_params.traslation_z != self.traslz or self.motion_params.rotation!= self.rotation:
            plan = dummy_traslation(self.motion_params.traslation_x-self.traslx, self.motion_params.traslation_z-self.traslz,self.motion_params.rotation - self.rotation, self.node_joint_states, self.rotation)
            self.rotation = self.motion_params.rotation 
            self.traslx = self.motion_params.traslation_x
            self.traslz = self.motion_params.traslation_z
            self.send_joint_states(plan, 0.2)
        
        if self.motion_params.camera!=self.camera:
            self.camera = self.motion_params.camera
            new_joints = self.node_joint_states
            new_joints[0] = self.camera
            self.publish_joint_states(new_joints)

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

def rest(node: MyNode):
    joint_position_state=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    node.publish_joint_states(joint_position_state)

def gaits(node: MyNode, f_speed):
    if f_speed == 0.0:
        return
    plan       = []
    min_speed  = 0
    max_speed  = 1
    delay_min  = 0.1  # en s
    delay_max  = 0.5  # en s
    length     = 0.15
    if f_speed < 0:   
        length = -length

    time_delay = delay_max - ((abs(f_speed) - min_speed) / (max_speed - min_speed)) * (delay_max - delay_min)
    plan = dummy_traslation(-length/4, 0, 0, node.node_joint_states, node.motion_params.rotation)
    node.send_joint_states(plan,time_delay)

    time.sleep(0.05)
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

    rclpy.spin(movement_node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()