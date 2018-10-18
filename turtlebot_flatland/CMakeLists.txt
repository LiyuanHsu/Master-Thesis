cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_flatland)

# Ensure we're using c++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
)

## System dependencies are found with CMake's conventions
find_package(PkgConfig REQUIRED)


###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)
