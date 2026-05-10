# MPC-A-CBF

动态避障运动规划系统，支持数值仿真、Gazebo 仿真和实车部署。

## 目录

- [最小闭环](#最小闭环)
- [宿主机环境（Ubuntu 20.04）](#宿主机环境ubuntu-2004)
  - [系统要求](#系统要求)
  - [依赖安装](#依赖安装)
  - [CasADi（关键）](#casadi关键)
  - [工作空间初始化](#工作空间初始化)
  - [编译](#编译)
- [运行](#运行)
  - [数值仿真](#数值仿真)
  - [Gazebo 仿真](#gazebo-仿真)
  - [实车](#实车)
- [包结构](#包结构)
- [Controller 类型](#controller-类型)
- [避障调参经验](#避障调参经验)
- [全局路径数据处理器](#全局路径数据处理器)
- [Docker 环境（备用）](#docker-环境备用)
- [注意事项](#注意事项)

---

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
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch swarm_test acbf0_planner.launch show_rviz:=true


# 终端 2
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch swarm_test start_test.launch

```

验收：`/global_path`、`/local_path`、`/cmd_vel1` 持续输出，`swarm_test/output/` 有结果文件。

### Gazebo 仿真

```bash
source ~/catkin_ws/devel/setup.bash

# 终端 1 — Gazebo 环境（机器人 + 动态障碍物）
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch swarm_test start_gazebo_env.launch gzclient:=true

# 等 Gazebo 窗口出现、机器人模型加载完毕（看到 scout 车和障碍物圆柱体）。

# 终端 2 — FAST-LIO 状态估计
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch swarm_test exp_hardware.launch


# 终端 3 — 规划 + MPC 控制
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch swarm_test exp_acbf_planner.launch show_rviz:=true

```
操作：在 RViz 用 2D Nav Goal 点击目标点，观察避障效果。


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

## 避障调参经验

### 关键参数速查

`exp_acbf_planner.launch` 和 `acbf0_planner.launch` 的参数必须一致，否则避障效果天差地别。

| 参数 | 位置 | 推荐值 | 含义 |
|------|------|--------|------|
| `controller` | launch arg | **4** (ACBF) | 0=None, 1=DC, 2=SCBF, 3=DCBF, 4=ACBF |
| `front_adsm` | launch arg | **true** | 前端 Theta* 考虑动态障碍物速度 |
| `front_dis` | launch arg | false | 与 `front_adsm` 互斥，二选一 |
| `mpc/gamma` | node param | **0.35** | CBF 衰减率；越小越保守（避障更积极）；默认 0 = 没有避障 |
| `mpc/tau_scale` | node param | **0.30** | ACBF 时间尺度；0 = 不用预测 |
| `mpc/use_initiguess` | node param | **true** | 热启动 MPC 求解器，提升稳定性 |
| `mpc/ahead` | node param | **true** | 使用提前预测 |
| `safe_dist` | mpc_cbf.cpp | 0.3 + 0.4 | 安全余量 + 机器人半径（硬编码） |

### 常见坑

1. **障碍物话题 remap 错误**（最致命）
   - MPC 订阅 `/obs_Manager_node/obs_predict_pub`，但在 Gazebo 链路里实际发布者是 `/globalFsm_by_adsm/obs_predict_pub`（前端全局规划器）
   - 如果 remap 写成 `from="/obs_Manager_node/obs_predict_pub" to="/obs_Manager_node/obs_predict_pub"`（自己映射到自己），MPC 收不到任何障碍物信息，CBF 约束失效，车会直接撞
   - 正确写法：`<remap from="/obs_Manager_node/obs_predict_pub" to="/globalFsm_by_adsm/obs_predict_pub" />`

2. **Gazebo 场景 `_perception_GroundTruth` 配错**（同样致命）
   - `exp_acbf_planner.launch` 里的 `_perception_GroundTruth` 必须配合实际启动的链路
   - 如果设为 `false`（用感知），必须额外启动 `start_perception.launch`，否则 `globalFsm_by_adsm` 的订阅源 `/obstacle_prediction_node/...` 没人发布，前端不发 `obs_predict_pub`，MPC 依然收不到障碍物
   - 3 终端方案：设为 **`true`**（用真值，调试首选）
   - 4 终端方案：设为 `false`（用感知），额外起 `start_perception.launch`

3. **`gamma` 和 `tau_scale` 没设**
   - 两个参数默认值是 0，会让 CBF 约束退化成 `h_{k+1} >= h_k`（只要求不变差，不强制拉开距离）
   - 必须显式设置为 `gamma=0.35, tau_scale=0.30`（经验值）

4. **controller 选错**
   - DCBF (3) 不考虑障碍物轨迹预测，只用当前位置
   - ACBF (4) 基于预测，在动态障碍物场景下避障效果显著更好

5. **前端全局路径穿过障碍物**
   - `front_adsm=false` 时 Theta* 只看静态栅格，找到的路径可能穿过动态障碍物的未来位置
   - MPC 再聪明也救不回来，必须 `front_adsm=true`

6. **Python 节点缺可执行权限**
   - `scripts/*.py` 如果没 `chmod +x`，`roslaunch` 会报 `Cannot locate node of type`
   - 修复：`chmod +x /path/to/scripts/*.py`

### 调试三板斧

```bash
# 1. 确认 MPC 真的收到了障碍物数据
rostopic echo /globalFsm_by_adsm/obs_predict_pub | head -30

# 2. 确认 MPC 订阅话题正确
rosnode info /local_planner | grep -A5 "Subscriptions"

# 3. 看 MPC 重规划耗时（正常 30-60ms）
# 启动 planner 的终端会持续打印：
#   MPC replan_time =: 36.5ms
```

### 如果还是容易撞

按以下顺序调：

1. **gamma 调小**：`0.35 -> 0.20`（避障更积极，但会更保守、速度变慢）
2. **增加安全余量**：`planner/mpc_dcbf/src/mpc_cbf.cpp` 第 54 行 `safe_dist = 0.3 + 0.4` 改成 `0.5 + 0.4`（需重编 `mpc_dcbf`）
3. **降低 v_max**：`mpc_cbf.cpp` 第 42 行 `v_max = 1.3` 改小（需重编）
4. **增加预测步数**：`pre_step = 20 -> 30`（预测更远，但 MPC 耗时增加）

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
