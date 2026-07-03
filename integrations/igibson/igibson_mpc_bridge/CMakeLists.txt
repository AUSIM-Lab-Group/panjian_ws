cmake_minimum_required(VERSION 3.0.2)
project(igibson_mpc_bridge)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  topic_tools
  visualization_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/cmd_vel_relay.py
  scripts/corridor_metrics_logger.py
  scripts/igibson_corridor_crowd_sim.py
  scripts/keyboard_cmd_vel.py
  scripts/pedestrian_state_publisher.py
  scripts/scripted_pedestrian_manager.py
  scripts/topic_health_check.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
