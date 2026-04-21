cmake_minimum_required(VERSION 2.8.3)
project(traj_planner)

# set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(PkgConfig QUIET)
find_package(nlopt QUIET CONFIG)

if(TARGET nlopt::nlopt)
  set(NLOPT_TARGET nlopt::nlopt)
elseif(TARGET nlopt)
  set(NLOPT_TARGET nlopt)
else()
  find_path(NLOPT_INCLUDE_DIRS
    NAMES nlopt.hpp
  )
  find_library(NLOPT_LIBRARIES
    NAMES nlopt nlopt_cxx
  )

  if(NOT NLOPT_INCLUDE_DIRS OR NOT NLOPT_LIBRARIES)
    message(FATAL_ERROR "nlopt not found. Install libnlopt-cxx-dev or set nlopt_DIR.")
  endif()
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  plan_env
  cv_bridge
  message_generation
  dynamic_simulator
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES traj_planner
 CATKIN_DEPENDS plan_env dynamic_simulator message_runtime
#  DEPENDS system_lib
)

include_directories( 
  SYSTEM 
  include 
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)


add_executable( dynamic_planner_node
    example/dynamic_planner_node.cpp
    src/theta_astar.cpp
    src/bspline_opt.cpp
    src/plan_manager.cpp
    src/dynamic_planner_fsm.cpp
)
target_link_libraries( dynamic_planner_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${NLOPT_TARGET}
  ${NLOPT_LIBRARIES}
)

add_executable( obs_Manager_node
    example/obs_manager_node.cpp
)
# add_dependencies(obs_Manager_node 
#   ${${PROJECT_NAME}_EXPORTED_TARGETS} 
#   ${catkin_EXPORTED_TARGETS}
#   dynamic_simulator
# )
target_link_libraries( obs_Manager_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable( global_path_by_adsm
    example/global_path_by_adsm.cpp
    src/theta_astar.cpp
    src/bspline_opt.cpp
    src/plan_manager.cpp
)
target_link_libraries( global_path_by_adsm
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${NLOPT_TARGET}
  ${NLOPT_LIBRARIES}
)

add_executable( globalFsm_by_adsm
    example/globalFsm_by_adsm.cpp
    src/theta_astar.cpp
    src/bspline_opt.cpp
    src/plan_manager.cpp
)
target_link_libraries( globalFsm_by_adsm
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${NLOPT_TARGET}
  ${NLOPT_LIBRARIES}
)
