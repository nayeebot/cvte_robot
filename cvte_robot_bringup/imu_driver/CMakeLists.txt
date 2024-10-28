cmake_minimum_required(VERSION 3.0.2)
project(imu_driver)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

catkin_package()

catkin_package(
  CATKIN_DEPENDS roscpp tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu_driver 
  imu_driver.cpp
)

target_link_libraries(imu_driver
  ${catkin_LIBRARIES}
)

install(TARGETS imu_driver
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
