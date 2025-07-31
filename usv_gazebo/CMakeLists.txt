cmake_minimum_required(VERSION 2.8.3)
project(usv_gazebo)

if(POLICY CMP0054)
  cmake_policy(SET CMP0054 OLD)
endif()

find_package(catkin REQUIRED COMPONENTS
  usv_description
  xacro
  gazebo_ros
  roscpp
)

catkin_package(
  CATKIN_DEPENDS usv_description xacro gazebo_ros roscpp
)

set(XACRO_INORDER)
if(DEFINED ENV{ROS_DISTRO})
  if($ENV{ROS_DISTRO} STREQUAL "kinetic")
    set(XACRO_INORDER INORDER)
  endif()
endif()

xacro_add_files(
  urdf/wamv_gazebo.urdf.xacro
  ${XACRO_INORDER} INSTALL DESTINATION urdf
)

install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)

install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

