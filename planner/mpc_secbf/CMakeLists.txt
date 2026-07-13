cmake_minimum_required(VERSION 3.0.2)
project(mpc_secbf)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  semantic_guard
  semantic_fusion
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(casadi REQUIRED)

if(TARGET casadi::casadi)
  set(CASADI_TARGET casadi::casadi)
else()
  set(CASADI_TARGET casadi)
endif()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp nav_msgs std_msgs geometry_msgs visualization_msgs semantic_guard semantic_fusion
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${semantic_guard_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(mpc_secbf_node
  src/mpc_secbf_node.cpp
  src/mpc_secbf.cpp
)
add_dependencies(mpc_secbf_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${semantic_guard_EXPORTED_TARGETS}
)
target_link_libraries(mpc_secbf_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CASADI_TARGET}
)

# Set rpath for CasADi
get_target_property(_casadi_loc ${CASADI_TARGET} IMPORTED_LOCATION_RELEASE)
if(NOT _casadi_loc)
  get_target_property(_casadi_loc ${CASADI_TARGET} IMPORTED_LOCATION)
endif()
if(_casadi_loc)
  get_filename_component(_casadi_dir "${_casadi_loc}" DIRECTORY)
  set_property(TARGET mpc_secbf_node APPEND_STRING PROPERTY
    LINK_FLAGS " -Wl,-rpath,${_casadi_dir}")
endif()
