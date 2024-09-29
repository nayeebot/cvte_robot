cmake_minimum_required(VERSION 3.0.2)
project(cvte_robot_ir_charge)

set(CMAKE_CXX_STANDARD 17)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package()

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(cvte_robot_ir_charge 
  src/charge.cpp
)

target_link_libraries(cvte_robot_ir_charge
  ${catkin_LIBRARIES}
)

install(TARGETS cvte_robot_ir_charge
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)