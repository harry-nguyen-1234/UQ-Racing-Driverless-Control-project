cmake_minimum_required(VERSION 3.0.2)
project(car_controller_cpp)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  fssim_messages
  geometry_msgs
  visualization_msgs
)

catkin_package()

include_directories(include 
${catkin_INCLUDE_DIRS}
${roscpp_INCLUDE_DIRS}
)

add_executable(car_controller_cpp src/car_controller.cpp)
target_link_libraries(car_controller_cpp ${catkin_LIBRARIES})
