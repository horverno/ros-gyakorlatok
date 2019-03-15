cmake_minimum_required(VERSION 2.8.3)
project(gyakorlo_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gyakorlo_pkg
#  CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/gyakorlo_pkg.cpp
# )

add_executable(tavolsag_szamitas_node src/szamol.cpp)
target_link_libraries(tavolsag_szamitas_node ${catkin_LIBRARIES})

add_executable(csak_kiiras_node src/kiir.cpp)
target_link_libraries(csak_kiiras_node ${catkin_LIBRARIES})

