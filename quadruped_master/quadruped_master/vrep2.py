#!/usr/bin/env python3
from time import sleep
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# Crear cliente y conectarse al servidor
client = RemoteAPIClient()
sim = client.getObject('sim')

# Puerto y dirección del servidor
server_ip = "127.0.0.1"
server_port = 23050  # Asegúrate de que coincide con el puerto del servidor

try:
    # Comprobar conexión
    sim.startSimulation()
    print("Conexión establecida con el servidor.")

    # Obtener handles de las juntas
    joint_names = ['/camera_joint','/front_right_joint1', '/front_right_joint2', '/front_left_joint1', '/front_left_joint2', 
                    '/back_left_joint1', '/back_left_joint2','/back_right_joint1', '/back_right_joint2']
    joint_handles = []

    for joint in joint_names:
        handle = sim.getObject(joint)
        if handle != -1:
            print(f"Handle encontrado para {joint}: {handle}")
            joint_handles.append(handle)
        else:
            print(f"No se pudo encontrar el handle para {joint}.")

    # Controlar las juntas (moverlas a posiciones aleatorias como prueba)
    for handle in joint_handles:
        sim.setJointTargetPosition(handle, 0.5)  # Cambia este valor para probar
        sleep(0.1)

    print("Movimientos aplicados a las juntas.")

    # Finalizar la simulación
    sim.stopSimulation()
    print("Simulación finalizada.")
except Exception as e:
    print(f"Error durante la ejecución: {e}")
