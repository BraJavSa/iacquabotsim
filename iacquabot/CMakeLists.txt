cmake_minimum_required(VERSION 3.0.2)
project(iacquabot)

# Buscar dependencias
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gazebo_ros
  xacro
)

# Buscar Eigen3 (se asegura de que esté instalado)
find_package(Eigen3 REQUIRED)

# Declarar el paquete
catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs gazebo_ros xacro
)

# Crear el ejecutable
add_executable(velocity_controller src/velocity_controller.cpp)
add_executable(position_controller src/position_controller.cpp)
add_executable(thruster_signals src/thruster_signals.cpp)
add_executable(world2body src/world2body.cpp)


# Vincular el ejecutable con las bibliotecas necesarias
target_link_libraries(velocity_controller
  ${catkin_LIBRARIES}
)
target_link_libraries(position_controller
  ${catkin_LIBRARIES}
)
target_link_libraries(thruster_signals
  ${catkin_LIBRARIES}
)
target_link_libraries(world2body
  ${catkin_LIBRARIES}
)
# Incluir directorios (agregar la ruta a Eigen)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Instalar los archivos de launch
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Instalar los programas de scripts
install(PROGRAMS
  scripts/mi_script.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
