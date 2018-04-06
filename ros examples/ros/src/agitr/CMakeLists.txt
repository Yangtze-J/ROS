cmake_minimum_required(VERSION 2.8.3)
project(agitr)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs turtlesim)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package ( )

include_directories( include ${catkin_INCLUDE_DIRS} )


add_executable(hello hello.cpp)
add_executable(sec sec.cpp)
add_executable(pubvel pubvel.cpp)
add_executable(subpose subpose.cpp)

target_link_libraries(hello ${catkin_LIBRARIES})
target_link_libraries(sec ${catkin_LIBRARIES})
target_link_libraries(pubvel ${catkin_LIBRARIES})
target_link_libraries(subpose ${catkin_LIBRARIES})
