# iGibson Docker 隔离 MPC-SECBF 集成

此可选分支将 iGibson、其 ROS 桥接器和最小 MPC-SECBF 适配器与主 RViz、Gazebo 和真实机器人工作流隔离。

## 阶段 0：主机检查

```bash
cd /home/lxr20/lxr/panjian_ws
bash integrations/igibson/scripts/00_check_host.sh
```

如果缺少 Docker，请单独安装：

```bash
bash integrations/igibson/scripts/01_install_docker_prereqs.sh
newgrp docker
```

## 阶段 1：iGibson ROS 演示

```bash
bash integrations/igibson/scripts/02_prepare_igibson_vendor.sh
bash integrations/igibson/scripts/03_build_igibson_ros_image.sh
bash integrations/igibson/scripts/04_run_igibson_ros.sh
```

如果当前 shell 尚未识别 `docker` 组，请使用 `sg docker -c '...'` 包装 Docker 命令或运行一次 `newgrp docker`。

在容器内，可以先启动官方 RGB-D 启动程序做基础检查：

```bash
source /opt/ros/noetic/setup.bash
source /opt/catkin_ws/devel/setup.bash
roslaunch igibson-ros turtlebot_rgbd.launch
```

预期的 iGibson 话题包括 `/odom`、`/ground_truth_odom`、`/gibson_ros/camera/rgb/image`、`/gibson_ros/camera/rgb/depth` 和 `/gibson_ros/lidar/points`。

## 阶段 2：最小 MPC-SECBF 桥接器

准备隔离的 catkin 工作空间：

```bash
bash integrations/igibson/scripts/05_prepare_isolated_mpc_ws.sh
```

从隔离工作空间启动桥接器：

```bash
source /opt/ros/noetic/setup.bash
source /home/lxr20/lxr/igibson_mpc_ws/devel/setup.bash
roslaunch igibson_mpc_bridge igibson_minimal_bridge.launch
```

该桥接器将 iGibson 传感器话题映射到现有控制层名称，并将 `/cmd_vel_secbf_dryrun` 中继到 `/mobile_base/commands/velocity`。隔离工作空间仅链接干运行规划器所需的最小包，包括 `semantic_fusion`、`mpc_secbf`、`semantic_guard`、`plan_env`、`dynamic_simulator`、`traj_planner`、`map_generator`、`robot_simulator`、`swarm_test` 和 `igibson_mpc_bridge`。

## 阶段 3：corridor crowd demo

该 demo 复用现有 `igibson_mpc_bridge`，不新增单独 package。第一版目标是可运行、可视化、可记录：用 `8m x 2m` corridor 配置和 3 个脚本行人模拟论文风格的人群场景。RViz 中的 `/pedestrian_markers` 仍保留为逻辑行人对照层；iGibson GUI / RGB-D / LiDAR 中会优先加载 iGibson `Pedestrian` mesh，若当前镜像缺少该资产或接口不兼容，则自动退回到头、躯干、手臂、腿组成的简化人形几何体。ORCA / rvo2 / Sim-BL / Sim-Int / Sim-Ext 留到下一阶段。

配置文件：

```bash
integrations/igibson/igibson_mpc_bridge/config/corridor_crowd.yaml
```

先重新准备并编译隔离工作空间：

```bash
bash integrations/igibson/scripts/05_prepare_isolated_mpc_ws.sh
cd /home/lxr20/lxr/igibson_mpc_ws
source /opt/ros/noetic/setup.bash
catkin_make \
  -DCATKIN_WHITELIST_PACKAGES="semantic_fusion;mpc_secbf;semantic_guard;plan_env;dynamic_simulator;traj_planner;map_generator;robot_simulator;swarm_test;igibson_mpc_bridge" \
  -DCMAKE_CXX_STANDARD=14 \
  -DCMAKE_PREFIX_PATH="/home/lxr20/lxr/local/casadi-3.7.0;/opt/ros/noetic"
```

终端 1：启动 iGibson ROS demo 容器。脚本会把当前 `integrations/igibson` 挂载到容器 `/workspace/integrations/igibson`，不修改 vendor 源码：

```bash
cd /home/lxr20/lxr/panjian_ws
bash integrations/igibson/scripts/04_run_igibson_ros.sh
```

容器内启动带可见人体对象的 iGibson corridor crowd sim。该节点会订阅 `/pedestrian_states`，在 iGibson 场景里生成 3 个人体对象，并继续发布 RGB、depth、LiDAR point cloud 和 odom：

```bash
source /opt/ros/noetic/setup.bash
source /opt/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=/workspace/integrations/igibson/igibson_mpc_bridge:$ROS_PACKAGE_PATH
roslaunch igibson_mpc_bridge igibson_corridor_crowd_sim.launch
```

启动日志里应看到类似：

```text
iGibson optimized_renderer disabled so dynamic pedestrian meshes can be rendered
Loaded iGibson Pedestrian mesh 0 for pedestrian 1
Loaded iGibson Pedestrian mesh 1 for pedestrian 2
Loaded iGibson Pedestrian mesh 2 for pedestrian 3
Disabled robot-pedestrian physical collision ...
Corridor pedestrians are placed in iGibson world frame with odom offset ...
```

人形对象会与机器人禁用物理碰撞，避免脚本行人被每帧重置位置时把机器人硬推飞；RGB-D、LiDAR 和 `/pedestrian_states` 仍保留用于视觉效果和安全距离/碰撞指标记录。

如果日志里出现下面这句，说明人形对象只进了物理世界，未进入 RGB 渲染器，需要确认当前代码已重新加载并且 `optimized_renderer` 已关闭：

```text
Using optimized renderer and optimization process is already excuted, cannot add new objects
```

终端 2：启动 corridor crowd bridge、脚本行人、状态发布和日志：

```bash
source /opt/ros/noetic/setup.bash
source /home/lxr20/lxr/igibson_mpc_ws/devel/setup.bash
roslaunch igibson_mpc_bridge corridor_crowd_demo.launch
```

如果你在已经同时包含 `igibson-ros` 和 `igibson_mpc_bridge` 的容器工作空间内启动，也可以使用：

```bash
roslaunch igibson_mpc_bridge corridor_crowd_demo.launch start_igibson_demo:=true
```

常用检查：

```bash
rostopic list | grep -E "pedestrian|Odometry|cmd_vel|mobile_base|gibson_ros"
rostopic echo -n 1 /pedestrian_states
rostopic hz /pedestrian_states
rostopic echo -n 1 /pedestrian_markers
rostopic hz /gibson_ros/camera/rgb/image
rostopic hz /gibson_ros/camera/depth/image
rostopic hz /gibson_ros/lidar/points
rostopic echo -n 1 /mobile_base/commands/velocity
```

图像检查：

```bash
rqt_image_view /gibson_ros/camera/rgb/image
rqt_image_view /gibson_ros/camera/depth/image
```

在 RViz 中建议同时显示 `/pedestrian_markers` 和 `/gibson_ros/lidar/points`，用 marker 对照 iGibson 内真实人体对象的 LiDAR 轮廓。

注意：如果只启动宿主机上的 `corridor_crowd_demo.launch`，RViz 里只会看到圆柱 marker 在动，这是正常的逻辑状态显示；要看到“人”，必须同时在 Docker 容器内运行 `igibson_corridor_crowd_sim.launch`，然后看 iGibson GUI 或 `rqt_image_view /gibson_ros/camera/rgb/image`。

如果修改了 `igibson_corridor_crowd_sim.py` 后正在运行旧 launch，需要在容器里 `Ctrl-C` 停掉并重新执行 `roslaunch igibson_mpc_bridge igibson_corridor_crowd_sim.launch`，否则旧 Python 进程不会加载新代码。

手动发布速度，检查 relay 和 iGibson 机器人运动：

```bash
rostopic pub -r 10 /cmd_vel_secbf_dryrun geometry_msgs/Twist "linear:
  x: 0.15
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.2"
```

键盘控制可以开一个新的宿主机终端。该节点不依赖 `teleop_twist_keyboard`，默认发布到 `/cmd_vel_secbf_dryrun`，继续经过 relay 限幅后进入 `/mobile_base/commands/velocity`：

```bash
source /opt/ros/noetic/setup.bash
source /home/lxr20/lxr/igibson_mpc_ws/devel/setup.bash
rosrun igibson_mpc_bridge keyboard_cmd_vel.py
```

按键：

```text
w/s       增大/减小 linear.x
a/d       增大/减小 angular.z
x 或空格  停车
q         停车并退出
```

停止机器人：

```bash
rostopic pub -1 /cmd_vel_secbf_dryrun geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

查看 CSV 日志：

```bash
cd /home/lxr20/lxr/panjian_ws
ls -lt integrations/igibson/logs/
tail -n 5 integrations/igibson/logs/corridor_metrics_*.csv
```

日志字段包括 `time`、机器人位姿、`pedestrian_states`、`min_distance_to_pedestrian`、`personal_space_violation` 和 `collision_flag`。

## 阶段 4：现有 MPC-SECBF 控制层

在另一个终端中：

```bash
source /opt/ros/noetic/setup.bash
source /home/lxr20/lxr/igibson_mpc_ws/devel/setup.bash
roslaunch swarm_test exp_secbf_planner.launch \
  cmd_vel_topic:=/cmd_vel_secbf_dryrun \
  v_max:=0.3 \
  front_adsm:=false
```

发送一个简短的测试目标：

```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "header:
  stamp: {secs: 0, nsecs: 0}
  frame_id: 'odom'
pose:
  position: {x: 2.0, y: 0.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
```

## 隔离说明

该支线只新增和修改 `integrations/igibson/` 下的文件。`integrations/igibson/CATKIN_IGNORE` 会阻止它被原 `/home/lxr20/lxr/panjian_ws` 默认 catkin 编译；真正编译发生在 `/home/lxr20/lxr/igibson_mpc_ws`。原 RViz、Gazebo 和实车 launch 链路保持不变。
