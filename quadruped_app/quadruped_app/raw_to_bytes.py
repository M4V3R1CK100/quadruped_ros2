import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import base64

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        self.declare_parameter('camera_topic', '/camera_link/image_raw')
        camera_topic       = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.subscription  = self.create_subscription( Image, camera_topic, self.listener_callback, 10)
        self.subscription  = self.create_subscription( Image, '/image_mat_raw', self.mat_listener_callback, 10)
        self.publisher_cam = self.create_publisher   (String, '/image_bytes_camera', 10)
        self.publisher_mat = self.create_publisher   (String, '/image_bytes_mat', 10)

        self.bridge = CvBridge()

        self.get_logger().info('raw_to_bytes_node initialized')

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        _, buffer = cv2.imencode('.jpg', cv_image)
        image_bytes = base64.b64encode(buffer).decode('utf-8')
        self.publisher_cam.publish(String(data=image_bytes))

    def mat_listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        _, buffer = cv2.imencode('.jpg', cv_image)
        image_bytes = base64.b64encode(buffer).decode('utf-8')
        self.publisher_mat.publish(String(data=image_bytes))
        print("mat image")

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()