cmake_minimum_required(VERSION 2.8.3)
project(magic_arm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  roboy_communication_middleware
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  src/
)

add_executable(${PROJECT_NAME} src/keyboardUI.cpp src/motorCommand.cpp)
add_dependencies(${PROJECT_NAME} roboy_communication_middleware_gencpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
