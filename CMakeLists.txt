cmake_minimum_required(VERSION 2.8.3)

set(ROBOT quadruped)

project(robot_properties_${ROBOT})

# find dependencies using catkin
find_package(catkin REQUIRED COMPONENTS
  xacro
)

# Xacro files of the quadruped
file(GLOB  xacro_file_names RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/xacro
     ${CMAKE_CURRENT_SOURCE_DIR}/xacro/*.xacro)

foreach(xacro_file_name ${xacro_file_names})
  # remove .xacro extension
  string(REGEX MATCH "(.*)[.]xacro$" unused ${xacro_file_name})
  set(urdf_file_name ${CMAKE_MATCH_1})

  set(xacro_file_full_path ${CMAKE_CURRENT_SOURCE_DIR}/xacro/${xacro_file_name})
  set(urdf_file_full_path ${CMAKE_CURRENT_SOURCE_DIR}/urdf/${urdf_file_name})

  # create a rule to generate ${output_filename} from {it}
  xacro_add_xacro_file(
    ${xacro_file_full_path}
    ${urdf_file_full_path}
  )

  list(APPEND urdf_files ${urdf_file_full_path})

endforeach(xacro_file_name)

# add an abstract target to actually trigger the builds
add_custom_target(urdf_build_files ALL DEPENDS ${urdf_files})

catkin_package()
catkin_python_setup()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")


