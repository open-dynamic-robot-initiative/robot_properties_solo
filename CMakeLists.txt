cmake_minimum_required(VERSION 2.8.3)

set(ROBOT hopper)

project(robot_properties_${ROBOT})

# find dependencies using catkin
find_package(catkin REQUIRED COMPONENTS)

catkin_package(
  INCLUDE_DIRS include
)
catkin_python_setup()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")


