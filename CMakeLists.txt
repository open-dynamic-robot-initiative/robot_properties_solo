######################
# set up the project #
######################
cmake_minimum_required(VERSION 2.8.3)

set(ROBOT quadruped)
project(robot_properties_${ROBOT})

# required to use std::shared_ptr in code AND to link the DG plugins
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wl,--no-as-needed")

# ensuring path to libraries are set during install
set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

############################
# Search for dependencies. #
############################

# find dependencies using catkin
find_package(catkin REQUIRED COMPONENTS
    mpi_cmake_modules
)

##########################
# Build the xacro files. #
##########################

build_xacro_files()

##############################
# Define the Catkin Package. #
##############################

catkin_package()

#####################################
# Define the Catkin Python Package. #
#####################################

catkin_python_setup()

