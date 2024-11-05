#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np
from math import pi, cos, sin, acos, atan2

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        self.current_position = [0, 0, 0, 0, 0, 0, 0, 0]
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.joints_states = JointState()
        self.timer = self.create_timer(0.1, self.main_loop)
        self.get_logger().info("Robot Control Node initialized")

    def FK(self, theta1, theta2, base_on_dummy):
        if base_on_dummy:
            theta1 = -theta1
            theta2 = -theta2
        l1 = 0.4
        l2 = 0.4
        x = l1 * cos(theta1) + l2 * cos(theta1 + theta2)
        z = l1 * sin(theta1) + l2 * sin(theta1 + theta2)
        return x, z

    def IK(self, pos_x, pos_z, base_on_dummy):
        l1 = 0.4
        l2 = 0.4
        distancia = np.sqrt(pos_x**2 + pos_z**2)
        if distancia > (l1 + l2) or distancia < abs(l1 - l2):
            self.get_logger().warning("El punto objetivo está fuera del alcance del robot.")
            return None, None
        else:
            cosbeta = (pos_x ** 2 + pos_z ** 2 - l1 ** 2 - l2 ** 2) / (2.0 * l1 * l2)
            beta1 = acos(cosbeta)
            A, B = l1 + l2 * cosbeta, l2 * sin(beta1)
            if base_on_dummy:
                alpha1 = -atan2(pos_z * A + pos_x * B, pos_x * A - pos_z * B)
            else:
                alpha1 = atan2(pos_z * A - pos_x * B, pos_x * A + pos_z * B)
            return alpha1, beta1

    def send_joints(self, list_joints):
        self.joints_states.position = list_joints
        self.joints_states.header = Header()
        self.joints_states.header.stamp = self.get_clock().now().to_msg()
        self.joints_states.name = [
            'front_right_joint1', 'front_right_joint2', 
            'front_left_joint1', 'front_left_joint2',
            'back_left_joint1', 'back_left_joint2', 
            'back_right_joint1', 'back_right_joint2'
        ]
        self.pub.publish(self.joints_states)
        self.current_position = list_joints

    def plan_circle(self, center_x, center_y, r, theta_o, theta_f, sentido_x, sentido_y, steps):
        pos = []
        for theta in range(theta_o, theta_f + 1, steps):
            position_z = center_y + r * sin(theta * pi / 180)
            position_x = center_x + (r * cos(theta * pi / 180) if sentido_x else -r * cos(theta * pi / 180))
            pos.append((position_x, position_z) if sentido_y else (position_x, -position_z))
        return pos

    def gait(self, foot_number, length, time_delay):
        steps = 10
        length = length / 2
        n = 2 * foot_number - 1
        joint_position_state = self.current_position
        first_position = self.FK(self.current_position[n - 1], self.current_position[n], True)
        if foot_number > 2:
            posd = self.plan_circle(first_position[0] - length, first_position[1], length, 0, 180, True, foot_number <= 2, steps)
        else:
            posd = self.plan_circle(first_position[0] + length, first_position[1], length, 0, 180, foot_number > 2, True, steps)

        for p in posd:
            j = self.IK(p[0], p[1], True)
            if j[0] is not None:
                joint_position_state[n] = j[1]
                joint_position_state[n - 1] = j[0]
                self.send_joints(joint_position_state)
                rclpy.sleep(time_delay)

    def main_loop(self):
        # Aquí colocas la lógica que quieras correr en un bucle, similar a la función main de ROS 1
        self.get_logger().info("Running main loop")

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
