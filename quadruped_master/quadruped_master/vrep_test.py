#!/usr/bin/env python3


#este archivo lo uso en windows para pruebas, no lo he probado en ubuntu
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

    # Lista de posiciones deseadas para las juntas
    joint_positions = [-1.57, 0.4, 0.3, 0.2, 0.1, 0.0, -0.1, -0.2, -0.3]

    scriptHandle = sim.getObject('/moveJoints')
    print(scriptHandle)

    # Llamar a la función del script en CoppeliaSim
    success = sim.callScriptFunction('sysCall_joint', scriptHandle, joint_positions)


except Exception as e:
    print(f"Error durante la ejecución: {e}")
