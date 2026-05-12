cmake_minimum_required(VERSION 3.0.2)
project(semantic_detection)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  vision_msgs
  cv_bridge
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/yolo_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
