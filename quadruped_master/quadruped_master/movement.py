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
sys.path.append('/home/erick/quad_ws/src/quadruped_master/quadruped_master')
from gait_functions import *
from quadruped_interfaces.msg import MotionParams

global quit
quit = 0

class MyNode(Node):
    def __init__(self):
        super().__init__("movement_node")
        self.get_logger().info("Nodo creado")
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.motion_sub = self.create_subscription(MotionParams,"motion_params", self.movement_manage)
        self.motion_params = MotionParams()
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
    
    def adjust_goals(self,joint_goals_list,time_delay):
        for goals in joint_goals_list:
            self.node_joint_states = goals
            time.sleep(time_delay)

    def movement_manage(self):
        if self.motion_params.speed != 0:
            movement(self, self.motion_params.speed)
        if self.motion_params.rotation !=0:
            plan = dummy_traslation(0,0,self.motion_params.rotation,self.node_joint_states)
            self.adjust_goals(plan,0.2)
        if self.motion_params.traslation_z!=0 or self.motion_params.traslation_x!=0:
            plan = dummy_traslation(self.motion_params.traslation_x, self.motion_params.traslation_z , 0,self.node_joint_states)
            self.adjust_goals(plan,0.2)




def movement(node, speed):
    global quit
    plan = []
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
        plan = dummy_traslation(-length/2, 0, node.node_joint_states)
        node.adjust_goals(plan,time_delay)

        time.sleep(0.05)
        print("Front Gait")
        if speed > 0:  # Cuando Avanza
            plan = gait(1, length, node.node_joint_states)
            node.adjust_goals(plan,time_delay)

            plan = gait(2, length, node.node_joint_states)
            node.adjust_goals(plan,time_delay)

            plan = dummy_traslation(4 * length / 2, 0, 0 , node.node_joint_states)
            node.adjust_goals(plan,time_delay)

            time.sleep(1)
            plan = gait(3, length, node.node_joint_states)
            node.adjust_goals(plan,time_delay)
            plan = gait(4, length, node.node_joint_states)
            node.adjust_goals(plan,time_delay)
        if speed < 0:  # Cuando retrocede
            plan = gait(4, length, node.node_joint_states)
            node.adjust_goals(plan,time_delay)

            plan = gait(3, length, node.node_joint_states)
            node.adjust_goals(plan,time_delay)

            plan = dummy_traslation(4 * length / 2, 0, 0, node.node_joint_states)
            node.adjust_goals(plan,time_delay)

            time.sleep(1)
            plan = gait(2, length, node.node_joint_states)
            node.adjust_goals(plan,time_delay)

            plan = gait(1, length, node.node_joint_states)
            node.adjust_goals(plan,time_delay)

        plan = dummy_traslation(-length/2, 0, 0,node.node_joint_states)
        node.adjust_goals(plan,time_delay)


def user_input_loop(node):
    global quit
    stand_1 = 0.3
    stand_2 = 1.4

    while quit == 0:
        print("\nMenu:")
        print("1. Stand")
        print("2. Up position")
        print("3. Move (forward/backward)")
        print("4. Translate")
        print("5. Exit")
        
        try:
            imp = input("Enter your choice: ")
            number = int(imp)
            if number == 1:
                print("Setting to Stand position")
                joint_position_state = [stand_1, stand_2, stand_1, stand_2, stand_1, stand_2, stand_1, stand_2]
                node.node_joint_states = joint_position_state
            elif number == 2:
                print("Setting to Up position")
                joint_position_state = [-1, 2.2, -1, 2.2, -1, 2.2, -1, 2.2]
                node.node_joint_states = joint_position_state
                time.sleep(1)
                joint_position_state = [stand_1, stand_2, stand_1, stand_2, stand_1, stand_2, stand_1, stand_2]
                node.node_joint_states = joint_position_state
            elif number == 3:
                user = input("Enter Speed (positive for forward, negative for backward): ")
                velocity = int(user)
                movement(node, velocity)  # Llama a la función de movimiento
            elif number == 4:
                print("Translating")
                dummy_traslation(0.1, 0, 0,node.node_joint_states)  # Ajusta según tu función de traducción
            elif number == 5:
                print("Exiting...")
                quit = 1
            else:
                print("Invalid choice, please try again.")
        except ValueError:
            print("Please enter a valid number.")

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    
    # Crear un hilo para user_input_loop
    user_input_loop()

    # Ejecutar el nodo ROS
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()