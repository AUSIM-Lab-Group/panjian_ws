cmake_minimum_required(VERSION 3.0.2)
project(semantic_guard)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  semantic_fusion
  message_generation
)

find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  GuardLog.msg
  AppliedMarginArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs semantic_fusion message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(beta_guard_node
  src/beta_guard_node.cpp
)
add_dependencies(beta_guard_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(beta_guard_node
  ${catkin_LIBRARIES}
)

add_executable(beta_ground_truth_node
  src/beta_ground_truth_node.cpp
)
add_dependencies(beta_ground_truth_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(beta_ground_truth_node
  ${catkin_LIBRARIES}
)
