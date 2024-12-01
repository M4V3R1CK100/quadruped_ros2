cmake_minimum_required(VERSION 3.8)
project(quadruped_vrep)

# Configuración de compilación
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encontrar dependencias de ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Incluir las bibliotecas y los encabezados de la API remota de CoppeliaSim
set(SOURCES
  remoteApi/extApi.c
  remoteApi/extApiPlatform.c
)

set(HEADERS
  remoteApi/extApi.h
  remoteApi/extApiInternal.h
  remoteApi/extApiPlatform.h
  include/v_repConst.h
  include/v_repLib.h
)

# Definiciones específicas de CoppeliaSim
add_definitions(-DNON_MATLAB_PARSING)
add_definitions(-DMAX_EXT_API_CONNECTIONS=255)
add_definitions(-DDO_NOT_USE_SHARED_MEMORY)

# Incluir directorios adicionales
include_directories(
  include
  remoteApi
)

# Declarar el ejecutable de tu nodo
add_executable(communication src/vrep_communication.cpp ${SOURCES})

# Especificar las bibliotecas para enlazar con el ejecutable
target_link_libraries(communication ${rclcpp_LIBRARIES})

# Dependencias de compilación de ament
ament_target_dependencies(communication
  rclcpp
  std_msgs
  sensor_msgs
)

# Instalación del ejecutable
install(TARGETS communication
  DESTINATION lib/${PROJECT_NAME}
)

add_compile_options(-Wno-unused-variable -Wno-unused-parameter)

ament_package()

