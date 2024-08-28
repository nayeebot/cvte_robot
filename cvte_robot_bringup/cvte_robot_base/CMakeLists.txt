cmake_minimum_required(VERSION 3.0.2)
project(cvte_robot_base)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf
  tf2_ros
)

catkin_package()

catkin_package(
  CATKIN_DEPENDS roscpp tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(cvte_robot_base 
  cvte_robot_base.cpp
)

target_link_libraries(cvte_robot_base
  ${catkin_LIBRARIES}
)

install(TARGETS cvte_robot_base
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)