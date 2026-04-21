# MPC-A-CBF

## 最小闭环

当前推荐的复现目标是跑通两条主链，不追求整个工作区无白名单全量编译：

- 数值仿真：`acbf0_planner.launch + start_test.launch`
- Gazebo 仿真：`start_gazebo_env.launch + exp_hardware.launch + exp_acbf_planner.launch`

## Docker 环境

镜像假设为 `luxuran/noeticgym:with_conda_isaacgym`，并且宿主机 `/home/lxr` 已挂载到容器 `/workspace`。

```bash
docker exec -it noeticgym bash --noprofile --norc

rosversion -d
nvidia-smi
ls /workspace/panjian_ws
```

如果你当前容器 shell 继承了宿主机的 conda、ROS 2 或其他工作空间环境，先清理再 source Noetic：

```bash
unset LD_LIBRARY_PATH PYTHONPATH CMAKE_PREFIX_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CONDA_PREFIX CONDA_DEFAULT_ENV
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
```

工作空间初始化：

```bash
mkdir -p /catkin_ws
ln -s /workspace/panjian_ws /catkin_ws/src
cd /catkin_ws
```

## 依赖安装

先退出 conda，再 source ROS：

```bash
conda deactivate || true
source /opt/ros/noetic/setup.bash
```

主链最小依赖：

```bash
apt-get update && apt-get install -y \
  build-essential cmake pkg-config git \
  libeigen3-dev libpcl-dev libopencv-dev \
  libnlopt-cxx-dev libnlopt-dev libyaml-cpp-dev libapr1-dev \
  python3-dev python3-pip \
  ros-noetic-pcl-ros ros-noetic-cv-bridge \
  ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs \
  ros-noetic-rviz ros-noetic-rviz-visual-tools \
  ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
  ros-noetic-controller-manager ros-noetic-controller-interface \
  ros-noetic-control-toolbox ros-noetic-hardware-interface \
  ros-noetic-joint-limits-interface ros-noetic-velocity-controllers \
  ros-noetic-realtime-tools ros-noetic-robot-state-publisher \
  ros-noetic-joint-state-publisher ros-noetic-joint-state-publisher-gui \
  ros-noetic-xacro ros-noetic-message-filters \
  ros-noetic-eigen-conversions ros-noetic-laser-geometry \
  ros-noetic-angles ros-noetic-urdf
```

### CasADi 检查顺序

```bash
python3 -c "import casadi; print(casadi.__version__)"
cmake --find-package -DNAME=casadi -DCOMPILER_ID=GNU -DLANGUAGE=CXX -DMODE=EXIST
```

如果 Python 能导入但 CMake 找不到：

1. 先尝试安装系统开发包。
2. 如果镜像里没有可用系统包，再在 conda 环境安装 CasADi，并在编译前导出：

```bash
export CMAKE_PREFIX_PATH="$CONDA_PREFIX:$CMAKE_PREFIX_PATH"
```

## 两阶段编译

不要执行 `CATKIN_WHITELIST_PACKAGES=""` 的全量编译。使用固定白名单：

```bash
source /opt/ros/noetic/setup.bash
cd /catkin_ws

catkin_make \
  -DCATKIN_WHITELIST_PACKAGES="livox_ros_driver;fast_lio;dynamic_simulator" \
  -DCMAKE_CXX_STANDARD=14 \
  -j"$(nproc)"

catkin_make \
  -DCATKIN_WHITELIST_PACKAGES="livox_ros_driver;fast_lio;dynamic_simulator;plan_env;map_generator;laser_simulator;robot_simulator;traj_planner;mpc_dcbf;swarm_test;scout_description;scout_control;scout_gazebo_sim;velodyne_description;velodyne_gazebo_plugins" \
  -DCMAKE_CXX_STANDARD=14 \
  -j"$(nproc)"

source /catkin_ws/devel/setup.bash
```

可选持久化：

```bash
echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc
```

## 运行

### 数值仿真

```bash
unset LD_LIBRARY_PATH PYTHONPATH CMAKE_PREFIX_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CONDA_PREFIX CONDA_DEFAULT_ENV
source /catkin_ws/devel/setup.bash

# 终端 1
roslaunch swarm_test acbf0_planner.launch show_rviz:=false

# 终端 2
unset LD_LIBRARY_PATH PYTHONPATH CMAKE_PREFIX_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CONDA_PREFIX CONDA_DEFAULT_ENV
source /catkin_ws/devel/setup.bash
roslaunch swarm_test start_test.launch
```

验收重点：

- `/global_path` 持续更新
- `/local_path` 持续更新
- `/cmd_vel1` 持续输出
- 结果文件写入 `swarm_test/output/`

### Gazebo 仿真

宿主机先允许 GUI：

```bash
xhost +local:docker
```

容器内启动 3 个终端：

```bash
unset LD_LIBRARY_PATH PYTHONPATH CMAKE_PREFIX_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CONDA_PREFIX CONDA_DEFAULT_ENV
source /catkin_ws/devel/setup.bash

# 终端 1
roslaunch swarm_test start_gazebo_env.launch gzclient:=true

# 终端 2
unset LD_LIBRARY_PATH PYTHONPATH CMAKE_PREFIX_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CONDA_PREFIX CONDA_DEFAULT_ENV
source /catkin_ws/devel/setup.bash
roslaunch swarm_test exp_hardware.launch

# 终端 3
unset LD_LIBRARY_PATH PYTHONPATH CMAKE_PREFIX_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CONDA_PREFIX CONDA_DEFAULT_ENV
source /catkin_ws/devel/setup.bash
roslaunch swarm_test exp_acbf_planner.launch show_rviz:=true
```

默认主流程里不再并行启动 `roslaunch mpc_dcbf mpc_adsm_c.launch`，因为 `exp_acbf_planner.launch` 已内嵌 `local_planner`。

验收重点：

- Gazebo GUI 和 RViz 都能打开
- `/robot1/odom`、`/fastLIO/non_ground_points`、`/robot1/cmd_vel` 正常
- 发布 `/move_base_simple/goal` 后，`/global_path` 和 `/local_path` 更新

## 全局路径数据处理器

`global_path_data_processor.py` 默认输出目录已经改为 `swarm_test/output/`，也可以在 launch 中通过 `output_dir` 覆盖。

详细说明见 [swarm_test/scripts/README_global_path_processor.md](/home/lxr/panjian_ws/swarm_test/scripts/README_global_path_processor.md)。

## 实车相关说明

`launch_exp/` 下仍保留实车实验脚本。若用于实车，请按需把 `use_sim_time:=false` 传给对应 launch，并按底盘通信要求先配置 CAN。
