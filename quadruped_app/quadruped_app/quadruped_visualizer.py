import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import numpy as np
import io
from PIL import Image as PILImage

class QuadrupedImagePublisher(Node):
    def __init__(self):
        super().__init__('quadruped_image_publisher')
        self.subscription = self.create_subscription(
            JointState, '/joint_goals', self.listener_callback, 10)
        
        # Publicador de imágenes
        self.publisher = self.create_publisher(Image, '/image_raw', 10)
        self.bridge = CvBridge()

        # Longitudes del cuadrúpedo
        self.link_lengths = [0.4, 0.4]  # hombro a codo, codo a pie
        self.angles = [0.0, 0.0, 0.0, 0.0]  # Inicialización de los ángulos

        # Configuración de Matplotlib (sin ventana y con estética mejorada)
        self.fig, self.ax = plt.subplots(figsize=(10, 10))  # Aumentar el tamaño del gráfico
        self.ax.set_xlim(-1.2, 1.2)
        self.ax.set_ylim(-1, 0.5)
        self.ax.set_aspect('equal')
        self.ax.set_facecolor('black')  # Fondo negro
        plt.ioff()  # Desactiva la interfaz gráfica interactiva

    def listener_callback(self, msg):
        if len(msg.position) >= 4:
            # Invertimos el sentido de los ángulos
            self.angles = [-angle for angle in msg.position[3:7]]
            self.update_plot_and_publish_image()

    def update_plot_and_publish_image(self):
        self.ax.clear()
        self.ax.set_facecolor('black')  # Asegurar fondo negro tras limpiar el gráfico

        # Separación inicial entre las patas
        front_offset = 0.11
        rear_offset = -0.11

        # Coordenadas de la pata delantera
        x1_front = front_offset + self.link_lengths[0] * np.cos(self.angles[0])
        y1_front = self.link_lengths[0] * np.sin(self.angles[0])
        x2_front = x1_front + self.link_lengths[1] * np.cos(self.angles[0] + self.angles[1])
        y2_front = y1_front + self.link_lengths[1] * np.sin(self.angles[0] + self.angles[1])

        # Coordenadas de la pata trasera
        x1_rear = rear_offset - self.link_lengths[0] * np.cos(self.angles[2])
        y1_rear = self.link_lengths[0] * np.sin(self.angles[2])
        x2_rear = x1_rear - self.link_lengths[1] * np.cos(self.angles[2] + self.angles[3])
        y2_rear = y1_rear + self.link_lengths[1] * np.sin(self.angles[2] + self.angles[3])

        # Dibuja las patas
        self.ax.plot([front_offset, x1_front, x2_front], [0, y1_front, y2_front], 'o-', color='cyan', linewidth=3, label='Pata Delantera')
        self.ax.plot([rear_offset, x1_rear, x2_rear], [0, y1_rear, y2_rear], 'o-', color='yellow', linewidth=3, label='Pata Trasera')

        # Dibuja el torso (línea que conecta las patas)
        self.ax.plot([front_offset, rear_offset], [0, 0], 'o-', color='white', linewidth=3, label='Torso')

        # Añadir puntos circulares en las articulaciones
        self.ax.scatter([front_offset, x1_front, x2_front], [0, y1_front, y2_front], color='cyan', s=150, zorder=5)
        self.ax.scatter([rear_offset, x1_rear, x2_rear], [0, y1_rear, y2_rear], color='yellow', s=150, zorder=5)

        # Mejora la estética del gráfico
        self.ax.axis('off')  # Quita los ejes, las etiquetas y los bordes
        self.ax.set_xlim(-1.2, 1.2)
        self.ax.set_ylim(-1, 0.5)
        self.ax.set_aspect('equal')

        # Convierte la gráfica en una imagen
        buf = io.BytesIO()
        self.fig.savefig(buf, format='png', bbox_inches='tight', pad_inches=0, facecolor='black')
        buf.seek(0)
        img = PILImage.open(buf)
        cv_image = np.array(img)

        # Publica la imagen como un mensaje de tipo Image
        ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='rgba8')
        self.publisher.publish(ros_image)
        buf.close()

def main(args=None):
    rclpy.init(args=args)
    node = QuadrupedImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
