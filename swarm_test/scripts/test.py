source /opt/ros/noetic/setup.bash

rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "header:
  stamp: {secs: 0, nsecs: 0}
  frame_id: 'camera_init'
pose:
  position: {x: 0.5, y: 0.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"