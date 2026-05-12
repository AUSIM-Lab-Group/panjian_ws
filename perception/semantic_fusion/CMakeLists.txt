cmake_minimum_required(VERSION 3.0.2)
project(semantic_fusion)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  vision_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  jsk_recognition_msgs
  message_generation
  cv_bridge
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  SemanticObstacle.msg
  SemanticObstacleArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs vision_msgs
                 nav_msgs tf2_ros jsk_recognition_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(semantic_fusion_node
  src/fusion_node.cpp
  src/projection.cpp
)
add_dependencies(semantic_fusion_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(semantic_fusion_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
