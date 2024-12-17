import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(Image,'/image_mat_raw', self.listener_callback,10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        # Convertir el mensaje de ROS a una imagen de OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Mostrar la imagen usando OpenCV
        cv2.imshow("Imagen de la Cámara", cv_image)
        cv2.waitKey(1)  # Refresca la ventana con la imagen

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
