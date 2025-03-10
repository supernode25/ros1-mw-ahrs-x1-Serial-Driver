cmake_minimum_required(VERSION 3.0.2)
project(serial_x1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  tf2
)

find_package(serial REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${serial_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

add_executable(imu_x1_node
  src/imu_x1.cpp 
  
  #src/imu_x1_node.cpp
)
target_link_libraries(imu_x1_node
  ${catkin_LIBRARIES}
  ${serial_LIBRARIES}
)

