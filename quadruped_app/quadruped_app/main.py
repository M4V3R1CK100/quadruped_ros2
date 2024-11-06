import flet as ft

def interface(page: ft.Page):
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
    title = ft.Text(value="Quadruped Controller", size=28, weight=ft.FontWeight.BOLD, color=ft.colors.BLUE_200)

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
        if not(float(txt_z_pos.value) == float(0.10)): # Modificar según altura mínima permitida para el robot
            txt_z_pos.value = str(round(float(txt_z_pos.value) - 0.05, 2))
            page.update()
        else:
            page.snack_bar = ft.SnackBar(ft.Text(f"The min z traslation is  {txt_z_pos.value}", weight=ft.FontWeight.BOLD, size=16, color=ft.colors.WHITE),bgcolor=ft.colors.RED_700)
            page.snack_bar.open = True
            page.update()

    def plus_click_z_pos(e):
        if not(float(txt_z_pos.value) == float(0.40)): # Modificar según altura máxima permitida para el robot
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
        page.snack_bar = ft.SnackBar(ft.Text(value=f"The information has been sent", weight=ft.FontWeight.BOLD, size=16, color=ft.colors.WHITE),bgcolor=ft.colors.GREEN_700)
        page.snack_bar.open = True
        page.update()

    def home_position(e):
        page.snack_bar = ft.SnackBar(ft.Text(value=f"The information has been sent", weight=ft.FontWeight.BOLD, size=16, color=ft.colors.WHITE),bgcolor=ft.colors.GREEN_700)
        page.snack_bar.open = True
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
            ft.Container(
                    content=ft.Text(value="Aqui va el video", style=ft.FontWeight.BOLD),
                    border=ft.border.all(width=2, color=ft.colors.BLUE_400), 
                    border_radius=10, width=500, height=300, alignment=ft.Alignment(0, 0),
                    padding=20, margin=ft.margin.only(top=10),bgcolor=ft.colors.GREY_800, expand=True
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


if __name__ == "__main__":

    ft.app(target=interface)
