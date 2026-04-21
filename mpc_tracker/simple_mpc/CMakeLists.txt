cmake_minimum_required(VERSION 3.0.2)
project(simple_mpc)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  std_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(casadi REQUIRED)

if(TARGET casadi::casadi)
  set(CASADI_TARGET casadi::casadi)
elseif(TARGET casadi)
  set(CASADI_TARGET casadi)
else()
  set(CASADI_TARGET casadi)
endif()

function(configure_casadi_target_runtime target_name)
  if(NOT TARGET ${CASADI_TARGET})
    return()
  endif()

  get_target_property(_casadi_location ${CASADI_TARGET} IMPORTED_LOCATION_RELEASE)
  if(NOT _casadi_location)
    get_target_property(_casadi_location ${CASADI_TARGET} IMPORTED_LOCATION_RELWITHDEBINFO)
  endif()
  if(NOT _casadi_location)
    get_target_property(_casadi_location ${CASADI_TARGET} IMPORTED_LOCATION_DEBUG)
  endif()
  if(NOT _casadi_location)
    get_target_property(_casadi_location ${CASADI_TARGET} IMPORTED_LOCATION)
  endif()

  if(NOT _casadi_location)
    return()
  endif()

  get_filename_component(_casadi_lib_dir "${_casadi_location}" DIRECTORY)

  set_property(TARGET ${target_name} APPEND_STRING PROPERTY
    LINK_FLAGS " -Wl,-rpath,${_casadi_lib_dir} -Wl,-rpath-link,${_casadi_lib_dir}")

  if(EXISTS "${_casadi_lib_dir}/libstdc++.so.6")
    target_link_libraries(${target_name} "${_casadi_lib_dir}/libstdc++.so.6")
  endif()

  if(EXISTS "${_casadi_lib_dir}/libgcc_s.so.1")
    target_link_libraries(${target_name} "${_casadi_lib_dir}/libgcc_s.so.1")
  endif()
endfunction()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
)

add_executable(mpc_node
  src/mpc_node.cpp
  src/mpc_tracker.cpp
)

target_link_libraries(mpc_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CASADI_TARGET}
)
configure_casadi_target_runtime(mpc_node)
