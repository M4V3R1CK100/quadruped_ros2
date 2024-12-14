import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('camera_to_image_raw')
        self.publisher_ = self.create_publisher(Image,'/camera_link/image_raw',10)
        self.cv_bridge     = CvBridge()
        self.timer      = self.create_timer(0.1, self.timer_callback)                
        self.cap        = cv2.VideoCapture(0)



    def timer_callback(self):
        ret, frame = self.cap.read()

        if ret == True:
            self.publisher_.publish(self.cv_bridge.cv2_to_imgmsg(frame, 'bgr8'))

        self.get_logger().info('Piblishing video frame')

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
