# Sistema de Control de Movimiento para Robot Cuadrúpedo (ROS2 Humble)

![Quadruped Robot](https://via.placeholder.com/800x400.png?text=Imagen+del+Robot+Cuadrúpedo+Real+y+Simulación)  
*(Reemplaza esta imagen con una captura de tu robot en simulación y en la vida real)*

## Descripción del Proyecto

Este proyecto es parte de una tesis titulada **"Diseño e Implementación de un Sistema de Control de Movimiento para Robot Cuadrúpedo"**, desarrollado por **Erick Mendoza** y **Juan Saeteros**. El objetivo principal es implementar un sistema de control para un robot cuadrúpedo con 8 grados de libertad (2 DOF por pata) y un motor adicional para controlar una cámara montada en la cabeza del robot.

El robot es capaz de:
- Caminar hacia adelante y hacia atrás.
- Mover el centro de masa vertical y horizontalmente.
- Rotar el centro de masa para ajustarse a distintas posiciones.
- Mantener posiciones específicas mientras se mueve.
- Utilizar una cámara para monitorear el entorno y un motor adicional para ajustar su ángulo.

El proyecto está desarrollado en **ROS2 Humble**, utilizando **Gazebo Classic 11** y **V-REP** para simulaciones, y se puede controlar mediante una interfaz gráfica o un mando de PS4.

---

## Arquitectura del Sistema

El sistema está diseñado con una arquitectura modular que facilita la integración y pruebas de cada componente. Los módulos principales son:

1. **`interface_node`**: Responsable de la interacción con el usuario y la configuración de parámetros.
2. **`movement_node`**: Realiza cálculos cinemáticos para determinar la posición de las articulaciones en función de los parámetros requeridos.
3. **`dynamixel_communication_node`**: Envía comandos a los actuadores físicos del robot basándose en los datos generados por el `movement_node`.
4. **`write_motor_data`**: Registra y transmite en tiempo real la información del estado del robot (posiciones de los joints, parámetros del sistema, etc.).
5. **`image_node`**: Visualiza la cámara en la interfaz, tanto en simulaciones (Gazebo/V-REP) como en el robot real.
6. **`visualizer_node`**: Muestra los ángulos actuales de las articulaciones del robot utilizando `matplotlib`.

---

## Instalación

### Dependencias
- **ROS2 Humble**: Asegúrate de tener ROS2 Humble instalado en tu sistema.
- **Gazebo Classic 11**: Para simulaciones en Gazebo.
- **V-REP**: Para simulaciones en V-REP.
- **Dynamixel SDK**: Para controlar los motores Dynamixel del robot.

### Pasos de Instalación
1. Clona el repositorio en tu espacio de trabajo de ROS2:
   ```bash
   git clone https://github.com/M4V3R1CK100/quadruped_ros2.git
   cd quadruped_ros2