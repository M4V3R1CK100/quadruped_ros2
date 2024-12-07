import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class QuadrupedJointCommander(Node):
    def __init__(self):
        super().__init__('quadruped_joint_commander')
        
        # Publicador al tópico relevante del controlador
        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        # Temporizador para enviar comandos periódicamente
        self.timer = self.create_timer(1.0, self.send_joint_trajectory)
        
        # Nombres de los joints y posiciones objetivo
        self.joint_names = [
            'camera_joint',
            'front_right_joint1',
            'front_right_joint2',
            'front_left_joint1',
            'front_left_joint2',
            'back_right_joint1',
            'back_right_joint2',
            'back_left_joint1',
            'back_left_joint2'
        ]
        self.target_positions = [0.0, 0.3, 1.4, 0.3, 1.4, 0.3, 1.4, 0.3, 1.4]

    def send_joint_trajectory(self):
        # Crear mensaje JointTrajectory
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        # Configurar las posiciones deseadas y el tiempo de movimiento
        point = JointTrajectoryPoint()
        point.positions = self.target_positions
        point.time_from_start.sec = 2  # Movimiento en 2 segundos
        
        msg.points.append(point)
        
        # Publicar el mensaje
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent trajectory to {self.joint_names}')

def main(args=None):
    rclpy.init(args=args)
    
    node = QuadrupedJointCommander()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
