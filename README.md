# MPC-A-CBF

动态避障运动规划系统，支持数值仿真、Gazebo 仿真和实车部署。

## 最小闭环

三条主链：

- **数值仿真**：`acbf0_planner.launch + start_test.launch`
- **Gazebo 仿真**：`start_gazebo_env.launch + exp_hardware.launch + exp_acbf_planner.launch`
- **实车**：`exp_hardware.launch + start_perception.launch + exp_acbf_planner_use.launch`

---

## 宿主机环境（Ubuntu 20.04）

### 系统要求

| 项目 | 版本 |
|------|------|
| OS | Ubuntu 20.04 |
| ROS | Noetic (desktop-full) |
| GCC | 9.4+ (C++14) |
| CasADi | 3.7.x (本地编译，新 ABI) |

### 依赖安装

```bash
sudo apt-get update && sudo apt-get install -y \
  build-essential cmake pkg-config git \
  libeigen3-dev libpcl-dev libopencv-dev \
  libnlopt-cxx-dev libnlopt-dev libyaml-cpp-dev libapr1-dev \
  libpcap-dev libasio-dev \
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
  ros-noetic-angles ros-noetic-urdf \
  ros-noetic-teleop-twist-keyboard \
  ros-noetic-jsk-recognition-msgs ros-noetic-autoware-msgs
```

### CasADi（关键）

使用本地编译的 CasADi 3.7.0（新 C++ ABI，兼容 ROS Noetic）：

```
/home/lxr20/lxr/local/casadi-3.7.0/
├── include/casadi/
└── lib/
    ├── libcasadi.so.3.7
    └── cmake/casadi/casadi-config.cmake
```

编译时通过 `CMAKE_PREFIX_PATH` 传入，无需修改 CMakeLists.txt：

```bash
-DCMAKE_PREFIX_PATH="/home/lxr20/lxr/local/casadi-3.7.0;/opt/ros/noetic"
```

> **注意**：pip 安装的 CasADi 会强制 `-D_GLIBCXX_USE_CXX11_ABI=0`，与 ROS Noetic 的新 ABI 不兼容，链接时会报 `ros::console::initializeLogLocation` undefined reference。必须使用本地编译版本。

### 工作空间初始化

```bash
mkdir -p ~/catkin_ws
ln -sf ~/lxr/panjian_ws ~/catkin_ws/src
```

### 编译

两阶段白名单编译：

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws

# 阶段 1：基础包
catkin_make \
  -DCATKIN_WHITELIST_PACKAGES="livox_ros_driver;fast_lio;dynamic_simulator" \
  -DCMAKE_CXX_STANDARD=14 \
  -DCMAKE_PREFIX_PATH="/home/lxr20/lxr/local/casadi-3.7.0;/opt/ros/noetic" \
  -j$(nproc)

# 阶段 2：算法 + 仿真
catkin_make \
  -DCATKIN_WHITELIST_PACKAGES="livox_ros_driver;fast_lio;dynamic_simulator;plan_env;map_generator;laser_simulator;robot_simulator;traj_planner;mpc_dcbf;swarm_test;scout_description;scout_control;scout_gazebo_sim;velodyne_description;velodyne_gazebo_plugins" \
  -DCMAKE_CXX_STANDARD=14 \
  -DCMAKE_PREFIX_PATH="/home/lxr20/lxr/local/casadi-3.7.0;/opt/ros/noetic" \
  -j$(nproc)

# 阶段 3：实车硬件驱动
catkin_make \
  -DCATKIN_WHITELIST_PACKAGES="ugv_sdk;scout_msgs;scout_base;scout_bringup;rslidar_sdk;rs_to_velodyne;sbg_driver;dynamic_perception;ddynamic_reconfigure;mapping;simple_mpc" \
  -DCMAKE_CXX_STANDARD=14 \
  -DCMAKE_PREFIX_PATH="/home/lxr20/lxr/local/casadi-3.7.0;/opt/ros/noetic" \
  -j$(nproc)

source ~/catkin_ws/devel/setup.bash
```

持久化：

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

---

## 运行

### 数值仿真

```bash
source ~/catkin_ws/devel/setup.bash

# 终端 1
roslaunch swarm_test acbf0_planner.launch show_rviz:=false

# 终端 2
roslaunch swarm_test start_test.launch
```

验收：`/global_path`、`/local_path`、`/cmd_vel1` 持续输出，`swarm_test/output/` 有结果文件。

### Gazebo 仿真

```bash
source ~/catkin_ws/devel/setup.bash

# 终端 1
roslaunch swarm_test start_gazebo_env.launch gzclient:=true

# 终端 2
roslaunch swarm_test exp_hardware.launch

# 终端 3
roslaunch swarm_test exp_acbf_planner.launch show_rviz:=true
```

验收：Gazebo + RViz 正常，发布 `/move_base_simple/goal` 后 `/global_path` 和 `/local_path` 更新。

### 实车

```bash
source ~/catkin_ws/devel/setup.bash

# 0. CAN 底盘初始化
sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 500000

# 终端 1：状态估计
roslaunch swarm_test exp_hardware.launch use_sim_time:=false

# 终端 2：动态感知
roslaunch swarm_test start_perception.launch

# 终端 3：规划 + 控制
roslaunch swarm_test exp_acbf_planner_use.launch
```

验收：`/Odometry`、`/fastLIO/non_ground_points`、`/cmd_vel1` 正常输出。

---

## 包结构

```
panjian_ws/
├── hardware/                    # 实车硬件驱动
│   ├── ugv_sdk/                 # AgileX 底盘 CAN SDK
│   ├── scout_ros-master/        # Scout ROS 封装 (scout_msgs, scout_base, scout_bringup)
│   ├── rslidar_sdk/             # 速腾 LiDAR 驱动
│   ├── rs_to_velodyne-master/   # 点云格式转换
│   ├── sbg_ros_driver-master/   # SBG IMU 驱动
│   ├── ouster_example-20220927/ # Ouster LiDAR (可选)
│   ├── realsense-ros/           # Intel RealSense (可选, 需 librealsense2)
│   ├── ddynamic_reconfigure/    # 动态参数 (realsense 依赖)
│   └── all_demo/                # 一键启动全部硬件
├── state_estimation/
│   ├── FAST_LIO/                # FAST-LIO 状态估计
│   └── livox_ros_driver/        # Livox LiDAR 驱动
├── perception/
│   ├── dynamic_perception/      # DBSCAN + L-shape + 轨迹预测
│   ├── mapping/                 # 局部建图
│   └── plan_env/                # SDF 地图
├── planner/
│   ├── vomp_planner/traj_planner/  # 全局路径搜索 (Theta*, B-spline)
│   └── mpc_dcbf/                # MPC-CBF 局部规划 (C++)
├── mpc_tracker/simple_mpc/      # 独立 MPC 跟踪器
├── simulation_tools/
│   ├── dynamic_simulator/       # 动态障碍物仿真
│   ├── map_generator/           # 随机地图生成
│   ├── robot_simulator/         # 差速机器人数值仿真
│   └── laser_simulator/         # 激光仿真
├── simulation_models/
│   ├── scout/                   # Scout Gazebo 模型
│   ├── hunter/                  # Hunter Gazebo 模型
│   └── velodyne/                # Velodyne Gazebo 插件
└── swarm_test/                  # 主测试/启动编排
    ├── launch/                  # 数值仿真 + Gazebo launch
    ├── launch_exp/              # 实车实验 launch
    ├── config/                  # RViz 配置
    ├── scripts/                 # 数据处理脚本
    └── output/                  # 实验结果输出
```

## Controller 类型

launch 文件中 `controller` 参数：

| 值 | 方法 |
|----|------|
| 0 | None (无约束) |
| 1 | MPC-DC (距离约束) |
| 2 | MPC-SCBF (静态 CBF) |
| 3 | MPC-DCBF (动态 CBF) |
| 4 | MPC-ACBF (自适应 CBF) |

## 全局路径数据处理器

`global_path_data_processor.py` 输出目录为 `swarm_test/output/`，可在 launch 中通过 `output_dir` 覆盖。

---

## Docker 环境（备用）

镜像：`luxuran/noeticgym:with_conda_isaacgym`，宿主机 `/home/lxr` 挂载到容器 `/workspace`。

```bash
docker exec -it noeticgym bash --noprofile --norc

# 清理 conda/ROS2 环境污染
unset LD_LIBRARY_PATH PYTHONPATH CMAKE_PREFIX_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CONDA_PREFIX CONDA_DEFAULT_ENV
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

# 初始化工作空间
mkdir -p /catkin_ws
ln -s /workspace/panjian_ws /catkin_ws/src
cd /catkin_ws
source /opt/ros/noetic/setup.bash
```

编译和运行流程同上，路径替换为 `/catkin_ws`。

---

## 注意事项

1. **CasADi ABI**：必须使用本地编译版本（`/home/lxr20/lxr/local/casadi-3.7.0`），pip 版本 ABI 不兼容
2. **obs_manager.hpp** 在 3 处存在，修改时需保持一致：
   - `planner/vomp_planner/traj_planner/include/obs_manager/obs_manager.hpp`
   - `swarm_test/include/obs_manager_for_data_process/obs_manager.hpp`
   - `perception/dynamic_perception/include/dynamic_perception/obs_manager/obs_manager.hpp`
3. **实车 CAN**：启动底盘前需 `sudo modprobe gs_usb && sudo ip link set can0 up type can bitrate 500000`
4. **Gazebo 仿真**：`exp_acbf_planner.launch` 已内嵌 `local_planner`，不需要额外启动 `mpc_dcbf mpc_adsm_c.launch`
5. **障碍物配置**：`simulation_tools/dynamic_simulator/config/obstacles_param.yaml`
