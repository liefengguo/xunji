cmake_minimum_required(VERSION 3.0.2)
project(xunji)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  message_generation
  nav_msgs
  pcl_conversions
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  tf
)

## System dependencies are found with CMake's conventions
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system timer)


add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
  ctrl_cmd.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
    DEPENDS PCL GTSAM
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

link_directories(
    ${PCL_LIBRARY_DIRS}
    ${OpenCV_LIBRARY_DIRS}
)

# src/main.cpp src/loc_ang.cpp
add_executable(${PROJECT_NAME} src/main.cpp src/loc_ang.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(roadside_stone src/roadside_stone.cpp)
 
add_dependencies(roadside_stone ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 
target_link_libraries(roadside_stone
  ${catkin_LIBRARIES}
)