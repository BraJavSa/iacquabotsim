cmake_minimum_required(VERSION 3.10)  # Actualizar versión mínima
project(usv_description)

find_package(catkin REQUIRED COMPONENTS
  xacro
  gazebo_dev
  geographic_msgs
  message_generation
  roscpp
  rospy
  std_msgs
)

catkin_package()

find_package(gazebo REQUIRED)
find_package(xacro REQUIRED)  # Asegurar que xacro está disponible

# Variable para compatibilidad con Kinetic (Noetic no la necesita)
set(XACRO_INORDER "--inorder")
if(DEFINED ENV{ROS_DISTRO})
  if($ENV{ROS_DISTRO} STREQUAL "kinetic")
    set(XACRO_INORDER "")
  endif()
endif()

# Verifica que el archivo Xacro existe antes de compilar
add_custom_target(${PROJECT_NAME}_check_xacro ALL
  COMMAND ${CATKIN_ENV} xacro ${XACRO_INORDER} ${CMAKE_CURRENT_SOURCE_DIR}/urdf/wamv_base.urdf.xacro
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

# Instalar archivos Xacro y modelos
install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)

install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)
