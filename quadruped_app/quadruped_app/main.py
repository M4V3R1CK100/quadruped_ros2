import flet as ft
import base64
from io import BytesIO
from PIL import Image
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

class FletApp(Node):
    def __init__(self):
        super().__init__('flet_app')
        self.subscription = self.create_subscription(
            String,
            '/image_bytes',
            self.listener_callback,
            10)
        self.latest_image = None

    def listener_callback(self, msg):
        self.latest_image = msg.data

def update_image(page: ft.Page, app: FletApp):
    img_component = ft.Image(width=500, height=500)
    page.controls.append(img_component)
    page.update()

    while True:
        if app.latest_image:
            img_bytes = base64.b64decode(app.latest_image)
            img = Image.open(BytesIO(img_bytes))
            img_component.src_base64 = app.latest_image
            page.update()

def main():
    rclpy.init()
    app = FletApp()
    thread = threading.Thread(target=rclpy.spin, args=(app,))
    thread.start()

    ft.app(target=lambda page: update_image(page, app))

    app.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()