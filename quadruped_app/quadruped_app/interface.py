#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String
from io import BytesIO
from PIL import Image
import base64
from quadruped_interfaces.msg import MotionParams
import flet as ft
import threading

class my_node(Node):
    def __init__(self):
        super().__init__('interface_node')
        
        self.pub          = self.create_publisher(MotionParams, 'motion_params', 10)
        self.subscription = self.create_subscription(String,'/image_bytes',self.listener_image_callback,10)
        self.latest_image = None

        self.motion_params = MotionParams()

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("Interface Node initialized")

        self.speed        = 0.0  # Por ejemplo, velocidad de 1.0 m/s
        self.rotation     = 0.0  # Por ejemplo, rotación de 0.5 radianes
        self.traslation_x = 0.0  # Por ejemplo, traslación de 2.0 metros
        self.traslation_z = 0.0  # Por ejemplo, traslación de 2.0 metros
        self.motion       = 0.0  # Por ejemplo, traslación de 2.0 metros

    def listener_image_callback(self, msg):
        self.latest_image = msg.data

    
    def timer_callback(self):

        # Asignar valores a los campos de MotionParams (ajusta según la estructura del mensaje)
        self.motion_params.speed        = self.speed  # Por ejemplo, velocidad de 1.0 m/s
        self.motion_params.rotation     = self.rotation # Por ejemplo, rotación de 0.5 radianes
        self.motion_params.traslation_x = self.traslation_x  # Por ejemplo, traslación de 2.0 metros
        self.motion_params.traslation_z = self.traslation_z  # Por ejemplo, traslación de 2.0 metros
        self.motion_params.motion       = self.motion  # Por ejemplo, traslación de 2.0 metros

        # Publicar el mensaje en el tema 'motion_params'
        # self.pub.publish(self.motion_params)

    def update_params(self, speed, rotation, traslation_x, traslation_z, motion):
        print(speed, rotation, traslation_x, traslation_z, motion, "jota")
        # Método para actualizar las variables del nodo
        # Asignar valores a los campos de MotionParams (ajusta según la estructura del mensaje)
        self.motion_params.speed        = speed  # Por ejemplo, velocidad de 1.0 m/s
        self.motion_params.rotation     = rotation # Por ejemplo, rotación de 0.5 radianes
        self.motion_params.traslation_x = traslation_x  # Por ejemplo, traslación de 2.0 metros
        self.motion_params.traslation_z = traslation_z  # Por ejemplo, traslación de 2.0 metros
        self.motion_params.motion       = motion  # Por ejemplo, traslación de 2.0 metros

        self.pub.publish(self.motion_params)

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

def ros_spin(node: my_node):
    # Este método se ejecutará en un hilo separado
    rclpy.spin(node)

def interface(page: ft.Page, node: my_node):
    page.title = "Quadruped Controller"
    page.vertical_alignment = ft.MainAxisAlignment.CENTER
    page.window.height      = 750
    page.window.width       = 900
    page.auto_scroll        = True
    page.scroll             = ft.ScrollMode.HIDDEN
    page.theme_mode         = "dark" 
    page.window.min_width   = 700
    page.window.min_height  = 700

    page. theme = ft. Theme(
        color_scheme_seed=ft.colors.BLUE,
        visual_density=ft.VisualDensity.COMFORTABLE,
        color_scheme=ft.ColorScheme(
            primary=ft.colors.BLUE,
            secondary=ft.colors.WHITE54,
            background=ft.colors.GREY_900,
            surface=ft.colors.GREY_400
        )
    )

    txt_velocity = ft.TextField(value="0%",  read_only=True, text_align=ft.TextAlign.CENTER, width=80 )
    txt_z_pos    = ft.TextField(value="0.3", read_only=True, text_align=ft.TextAlign.CENTER, width=100, prefix_icon=ft.icons.SWAP_VERT)#label="Z displacement"
    txt_x_pos    = ft.TextField(value="0.0", read_only=True, text_align=ft.TextAlign.CENTER, width=100, prefix_icon=ft.icons.SWAP_HORIZ_OUTLINED)#label="X displacement"
    txt_angle    = ft.TextField(value="0°",  read_only=True, text_align=ft.TextAlign.CENTER, width=80)
    global motion
    motion = 0.0

    title        = ft.Text(value="Quadruped Controller", size=28, weight=ft.FontWeight.BOLD, color=ft.colors.BLUE_200)
    
    def minus_click_gait_vel(e):
        if not(float(txt_velocity.value.split("%")[0]) == float(-100)):
            txt_velocity.value = str(round(float(txt_velocity.value.split("%")[0]) - 10)) + "%"
            page.update()
        else:
            page.snack_bar = ft.SnackBar(content=ft.Text(f"The min gait velocity is  {txt_velocity.value}", weight=ft.FontWeight.BOLD, size=16, color=ft.colors.WHITE),bgcolor=ft.colors.RED_700)
            page.snack_bar.open = True
            page.update()

    def plus_click_gait_vel(e):
        if not(float(txt_velocity.value.split("%")[0]) == float(100)):
            txt_velocity.value = str(round(float(txt_velocity.value.split("%")[0]) + 10)) + "%"
            page.update()
        else:
            page.snack_bar = ft.SnackBar(ft.Text(f"The max gait velocity is  {txt_velocity.value}", weight=ft.FontWeight.BOLD, size=16, color=ft.colors.WHITE),bgcolor=ft.colors.RED_700)
            page.snack_bar.open = True
            page.update()

    def minus_click_z_pos(e):
        if not(float(txt_z_pos.value) == float(-0.20)): # Modificar según altura mínima permitida para el robot
            txt_z_pos.value = str(round(float(txt_z_pos.value) - 0.05, 2))
            page.update()
        else:
            page.snack_bar = ft.SnackBar(ft.Text(f"The min z traslation is  {txt_z_pos.value}", weight=ft.FontWeight.BOLD, size=16, color=ft.colors.WHITE),bgcolor=ft.colors.RED_700)
            page.snack_bar.open = True
            page.update()

    def plus_click_z_pos(e):
        if not(float(txt_z_pos.value) == float(0.20)): # Modificar según altura máxima permitida para el robot
            txt_z_pos.value = str(round(float(txt_z_pos.value) + 0.05, 2))
            page.update()
        else:
            page.snack_bar = ft.SnackBar(ft.Text(f"The max z traslation is  {txt_z_pos.value}", weight=ft.FontWeight.BOLD, size=16, color=ft.colors.WHITE),bgcolor=ft.colors.RED_700)
            page.snack_bar.open = True
            page.update()

    def minus_click_x_pos(e):
        if not(float(txt_x_pos.value) == float(-0.20)): # Modificar según altura mínima permitida para el robot
            txt_x_pos.value = str(round(float(txt_x_pos.value) - 0.05, 2))
            page.update()
        else:
            page.snack_bar = ft.SnackBar(ft.Text(f"The min x traslation is  {txt_x_pos.value}", weight=ft.FontWeight.BOLD, size=16, color=ft.colors.WHITE),bgcolor=ft.colors.RED_700)
            page.snack_bar.open = True
            page.update()

    def plus_click_x_pos(e):
        if not(float(txt_x_pos.value) == float(0.20)): # Modificar según altura máxima permitida para el robot
            txt_x_pos.value = str(round(float(txt_x_pos.value) + 0.05, 2))
            page.update()
        else:
            page.snack_bar = ft.SnackBar(ft.Text(f"The max x traslation is  {txt_x_pos.value}", weight=ft.FontWeight.BOLD, size=16, color=ft.colors.WHITE),bgcolor=ft.colors.RED_700)
            page.snack_bar.open = True
            page.update()

    def minus_click_angle(e):
        if not(float(txt_angle.value.split("°")[0]) == float(-30)):
            txt_angle.value = str(round(float(txt_angle.value.split("°")[0]) - 5)) + "°"
            page.update()
        else:
            page.snack_bar = ft.SnackBar(content=ft.Text(f"The min rotation angle is  {txt_angle.value}", weight=ft.FontWeight.BOLD, size=16, color=ft.colors.WHITE),bgcolor=ft.colors.RED_700)
            page.snack_bar.open = True
            page.update()

    def plus_click_angle(e):
        if not(float(txt_angle.value.split("°")[0]) == float(30)):
            txt_angle.value = str(round(float(txt_angle.value.split("°")[0]) + 5)) + "°"
            page.update()
        else:
            page.snack_bar = ft.SnackBar(ft.Text(f"The max rotation angle is  {txt_angle.value}", weight=ft.FontWeight.BOLD, size=16, color=ft.colors.WHITE),bgcolor=ft.colors.RED_700)
            page.snack_bar.open = True
            page.update()

    def send_data(e):
        global motion
        page.snack_bar = ft.SnackBar(ft.Text(value=f"The information has been sent", weight=ft.FontWeight.BOLD, size=16, color=ft.colors.WHITE),bgcolor=ft.colors.GREEN_700)
        page.snack_bar.open = True
        page.update()
        node.update_params(float(txt_velocity.value.split("%")[0]), float(txt_angle.value.split("°")[0]), float(txt_x_pos.value), float(txt_z_pos.value), motion)
        motion = 0.0


    def home_position(e):
        global motion
        page.snack_bar = ft.SnackBar(ft.Text(value=f"The information has been sent", weight=ft.FontWeight.BOLD, size=16, color=ft.colors.WHITE),bgcolor=ft.colors.GREEN_700)
        page.snack_bar.open = True
        motion = 1.0
        page.update()

    movement_control = ft.Container(
        content=ft.Column(
            [
                ft.Row(
                    [
                        ft.Text(value = "Gait velocity", size=18, weight=ft.FontWeight.BOLD, text_align=ft.TextAlign.CENTER, width=180), 
                        ft.Text(value = "Rotation"     , size=18, weight=ft.FontWeight.BOLD, text_align=ft.TextAlign.CENTER, width=180),
                        ft.Text(value = "Traslation"   , size=18, weight=ft.FontWeight.BOLD, text_align=ft.TextAlign.CENTER, width=180),
                        
                    ], alignment=ft.MainAxisAlignment.SPACE_EVENLY
                ),
                ft.Row(
                    [
                        ft.Column(
                            [
                                
                                ft.Row(
                                    [
                                        ft.IconButton(ft.icons.REMOVE, on_click=minus_click_gait_vel, icon_size=40),
                                        txt_velocity,
                                        ft.IconButton(ft.icons.ADD, on_click=plus_click_gait_vel, icon_size=40),
                                    ], alignment=ft.MainAxisAlignment.CENTER, 
                                ) 
                            ], horizontal_alignment=ft.CrossAxisAlignment.CENTER, width=200
                        ),
                        ft.Column(
                            [
                                 
                                ft.Container(
                                    content=(
                                        ft.Row(
                                            [
                                                ft.IconButton(icon=ft.icons.ROTATE_RIGHT, on_click=minus_click_angle, icon_size=40),
                                                txt_angle, 
                                                ft.IconButton(icon=ft.icons.ROTATE_LEFT , on_click=plus_click_angle , icon_size=40),
                                            ], alignment=ft.MainAxisAlignment.SPACE_EVENLY, 
                                        )
                                    ),height=110,
                                )
                            ], horizontal_alignment=ft.CrossAxisAlignment.CENTER, width=200
                        ), 
                        ft.Column(
                            [
                                ft.Row(
                                    [
                                        
                                        ft.IconButton(ft.icons.ARROW_BACK,on_click=minus_click_x_pos, icon_size=40),
                                        ft.Column(
                                            [
                                                ft.IconButton(ft.icons.ARROW_UPWARD, on_click=plus_click_z_pos, icon_size=40),
                                                ft.Container(content=ft.Text(value=""),width=40, height=40),
                                                ft.IconButton(ft.icons.ARROW_DOWNWARD, on_click=minus_click_z_pos, icon_size=40),
                                            ]
                                        ),
                                        ft.IconButton(ft.icons.ARROW_FORWARD, on_click=plus_click_x_pos, icon_size=40),
                                    ], alignment=ft.MainAxisAlignment.CENTER,
                                ), 
                                ft.Row(
                                    [
                                        txt_x_pos, txt_z_pos
                                    ], alignment=ft.MainAxisAlignment.CENTER
                                )
                            ], horizontal_alignment=ft.CrossAxisAlignment.CENTER, width=200
                        ),
                    ], alignment=ft.MainAxisAlignment.SPACE_EVENLY
                )     
            ], horizontal_alignment=ft.CrossAxisAlignment.CENTER, alignment=ft.MainAxisAlignment.SPACE_EVENLY, spacing=200
        ), 
    )

    video = ft.Row(
        [
            ft.Image(
                src="quadruped.png",
                width=200,
                height=200,
                fit=ft.ImageFit.CONTAIN,
            ),
            ft.Column(
                [
                    ft.FloatingActionButton(icon=ft.icons.HOME_ROUNDED, on_click=home_position, bgcolor=ft.colors.BLUE, text="Home Position", width=150),
                    ft.FloatingActionButton(icon=ft.icons.POWER_SETTINGS_NEW_OUTLINED, on_click=home_position, bgcolor=ft.colors.BLUE, text="Rest Position", width=150),
                    ft.FloatingActionButton(icon=ft.icons.SEND, on_click=send_data, bgcolor=ft.colors.GREEN_700, text="Send Data", width=150),
                ], width=150, height=300, horizontal_alignment=ft.CrossAxisAlignment.CENTER, alignment=ft.MainAxisAlignment.SPACE_AROUND, expand=True
            )
        ], vertical_alignment=ft.CrossAxisAlignment.CENTER
    )

    page.add(
        ft.Column(
            [
                title,
                video, 
                movement_control        
            ], spacing=25, horizontal_alignment=ft.CrossAxisAlignment.CENTER
        )
    )


def main():
    # Inicializar rclpy antes de crear el nodo
    rclpy.init()

    try:
        # Crear una instancia del nodo ROS2
        node = my_node()

        # Crear un hilo separado para ejecutar ros_spin
        ros_thread = threading.Thread(target=ros_spin, args=(node,))
        ros_thread.start()

        # Iniciar la interfaz Flet
        ft.app(target=lambda page: interface(page, node))

        # Esperar a que el hilo de ROS termine (opcional)
        ros_thread.join()
    finally:
        # Cerrar rclpy correctamente
        rclpy.shutdown()


if __name__ == "__main__":
    main()