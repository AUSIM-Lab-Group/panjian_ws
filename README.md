# MPC-A-CBF / MPC-SECBF

动态避障运动规划系统，支持传统 ACBF baseline、本文 MPC-SECBF 方法、数值仿真、Gazebo 仿真和实车部署。

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
- [可选仿真环境与修改入口](#可选仿真环境与修改入口)
- [包结构](#包结构)
  - [MPC-SECBF 论文代码目录](#mpc-secbf-论文代码目录)
- [Controller 类型](#controller-类型)
- [避障调参经验](#避障调参经验)
- [全局路径数据处理器](#全局路径数据处理器)
- [Docker 环境（备用）](#docker-环境备用)
- [注意事项](#注意事项)

---

## 最小闭环

三条主链：

- **数值仿真 baseline**：`acbf0_planner.launch + start_test.launch`
- **数值仿真 MPC-SECBF**：`secbf_planner.launch + start_test.launch`
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
  -DCATKIN_WHITELIST_PACKAGES="livox_ros_driver;fast_lio;dynamic_simulator;plan_env;map_generator;laser_simulator;robot_simulator;traj_planner;mpc_dcbf;mpc_secbf;semantic_guard;swarm_test;scout_description;scout_control;scout_gazebo_sim;velodyne_description;velodyne_gazebo_plugins" \
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

#### ACBF baseline

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

#### MPC-SECBF 论文代码

批量数值实验会自动从 `swarm_test/config/secbf_scenarios.yaml` 读取场景，生成动态障碍物参数，并启动本文方法：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

python3 swarm_test/scripts/run_secbf_sim_experiments.py \
  --scenario S4_mixed \
  --baseline B3_SECBF_with_guard \
  --duration-sec 60 \
  --roscore auto
```

#### 完整终端启动顺序：MPC-SECBF + RViz + 数据记录

这个流程用于“像 baseline 一样看 RViz 动态障碍物场景，同时记录实验数据”。日常跑通只需要两个终端；第一个 `roslaunch` 会自动拉起 ROS master。`RUN_DIR` 是本次实验输出目录，planner 和 `start_test.launch` 必须使用同一个 `RUN_DIR`。

**终端 1：启动本文 planner、semantic β、Guard 和 RViz**

```bash
cd /home/lxr20/lxr/panjian_ws
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

export RUN_DIR=/home/lxr20/lxr/panjian_ws/swarm_test/output/secbf_runs/rviz_s4

roslaunch swarm_test secbf_planner.launch \
  show_rviz:=true \
  scenario_id:=S4_mixed \
  baseline_id:=B3_SECBF_with_guard \
  guard_enabled:=true \
  output_dir:="$RUN_DIR" \
  obstacle_classes:='[pedestrian,vehicle,box]'
```

**终端 2：启动动态障碍物、目标触发器和数据记录**

```bash
cd /home/lxr20/lxr/panjian_ws
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

export RUN_DIR=/home/lxr20/lxr/panjian_ws/swarm_test/output/secbf_runs/rviz_s4
mkdir -p "$RUN_DIR"

cat > "$RUN_DIR/obstacles_param.yaml" <<'YAML'
obstacle_params:
- {x: 6.0, y: -1.0, z: 0.75, offset: 5.0, slower: 8.0, scale_x: 15.0, scale_y: 0.0, scale_z: 0.0}
- {x: 12.0, y: 1.5, z: 0.75, offset: 0.0, slower: 6.0, scale_x: -12.0, scale_y: 0.0, scale_z: 0.0}
- {x: 8.0, y: 0.0, z: 0.3, offset: 0.0, slower: 999.0, scale_x: 0.0, scale_y: 0.0, scale_z: 0.0}
YAML

roslaunch swarm_test start_test.launch \
  scenario_index:=4 \
  controller_index:=6 \
  num_of_obs:=3 \
  output_dir:="$RUN_DIR" \
  obstacle_params_file:="$RUN_DIR/obstacles_param.yaml" \
  record_data:=true
```

**终端 3：运行时检查**

```bash
cd /home/lxr20/lxr/panjian_ws
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

export RUN_DIR=/home/lxr20/lxr/panjian_ws/swarm_test/output/secbf_runs/rviz_s4

rosnode list | grep -E "mpc_secbf|beta_ground_truth|globalFsm|dynamic_corridor|data_processor"
rostopic info /cmd_vel1 | grep mpc_secbf
rostopic hz /cmd_vel1
tail -f "$RUN_DIR/guard_log.csv"
```

看到 `/cmd_vel1` 的发布者是 `mpc_secbf_node`，且 `guard_log.csv` 中有 `pedestrian/vehicle/box` 和不同的 `beta_applied`，就说明当前跑的是本文 MPC-SECBF + semantic_guard 链路。

如果你想长期调试、多次重启终端 1/2，也可以额外开一个终端先手动启动 ROS master：

```bash
source /opt/ros/noetic/setup.bash
roscore
```

如果提示 `roscore cannot run as another roscore/master is already running`，说明已有 ROS master。用 `rostopic list` 能看到 `/rosout` 时，直接继续终端 1/2 即可。

#### ACBF baseline 完整终端顺序

如果要跑原 ACBF baseline，终端结构相同，但 planner 换成 `acbf0_planner.launch`，数据记录标签换成 `controller_index:=4`。ACBF 没有 `guard_log.csv`，主要看 `data_processor_summary.csv` 和 `data_processor_distance.csv`。

**终端 1：ACBF planner + RViz**

```bash
cd /home/lxr20/lxr/panjian_ws
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

roslaunch swarm_test acbf0_planner.launch \
  show_rviz:=true
```

`acbf0_planner.launch` 当前把 `controller=4` 和 `front_adsm=true` 写成固定 `value`，命令行不能覆盖；如果要切换 legacy controller，需要编辑 `swarm_test/launch/acbf0_planner.launch` 中对应 `<arg ... value="..."/>`。

**终端 2：动态障碍物 + 数据记录**

```bash
cd /home/lxr20/lxr/panjian_ws
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

export RUN_DIR=/home/lxr20/lxr/panjian_ws/swarm_test/output/acbf_runs/rviz_s4
mkdir -p "$RUN_DIR"

roslaunch swarm_test start_test.launch \
  scenario_index:=4 \
  controller_index:=4 \
  num_of_obs:=5 \
  output_dir:="$RUN_DIR" \
  obstacle_params_file:=/home/lxr20/lxr/panjian_ws/simulation_tools/dynamic_simulator/config/obstacles_param.yaml \
  record_data:=true
```

#### 数据记录怎么看

手动 RViz 运行时，数据写到你传给 `output_dir` 的目录。上面的 MPC-SECBF 示例是：

```bash
export RUN_DIR=/home/lxr20/lxr/panjian_ws/swarm_test/output/secbf_runs/rviz_s4
ls -lh "$RUN_DIR"
```

常见文件：

| 文件 | 谁生成 | 用途 |
|------|--------|------|
| `obstacles_param.yaml` | 手动准备或批量脚本自动生成 | 本次动态障碍物轨迹配置 |
| `guard_log.csv` | `beta_ground_truth_node` / `beta_guard_node` | SECBF β、Guard、`h_EE` 记录 |
| `data_processor_summary.csv` | `data_processor_node` | 到达、耗时、轨迹统计等汇总 |
| `data_processor_distance.csv` | `data_processor_node` | 障碍物距离相关记录 |
| `global_path_analysis_*.json` | `global_path_data_processor.py` | 全局路径重规划与路径质量分析 |
| `global_path_summary_*.yaml` | `global_path_data_processor.py` | 全局路径分析摘要 |
| `summary.md` / `summary.csv` | `run_secbf_sim_experiments.py` | 批量实验摘要，手动 RViz 运行不会自动生成 |
| `planner.log` / `start_test.log` | `run_secbf_sim_experiments.py` | 批量实验日志，手动 RViz 运行不会自动生成 |

查看 Guard 和 β：

```text
timestamp, obstacle_id, semantic_class, beta_requested, beta_applied, h_ee, guard_passed
```

```bash
head -n 5 "$RUN_DIR/guard_log.csv"
tail -n 20 "$RUN_DIR/guard_log.csv"

# 只看类别、β 和 Guard 是否通过
awk -F, 'NR==1 || NR>1 {print $3, $4, $5, $6, $7}' "$RUN_DIR/guard_log.csv" | head -n 20
```

验证理论安全下界：

```bash
python3 swarm_test/scripts/verify_safety_bound.py \
  --csv "$RUN_DIR/guard_log.csv" \
  --gamma 0.35 \
  --eps_max 0.05 \
  --delta_bar_beta 0.3
```

查看数据处理结果：

```bash
head -n 20 "$RUN_DIR/data_processor_summary.csv"
head -n 20 "$RUN_DIR/data_processor_distance.csv"
find "$RUN_DIR" -maxdepth 1 -type f | sort
```

批量实验输出目录按时间戳自动命名，例如：

```bash
ls -td swarm_test/output/secbf_runs/*S4_mixed_B3_SECBF_with_guard | head -n 1
```

进入该目录后重点看：

```bash
cat summary.md
cat verify_safety_bound.txt
tail -n 30 planner.log
tail -n 30 start_test.log
```

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

#### 网口配置（首次或重装系统后必做）

LiDAR 通过有线网口通信，需要配置静态 IP：

```bash
# 配置静态 IP（雷达 IP: 192.168.4.200，上位机需配: 192.168.4.102）
sudo nmcli connection modify "有线连接 1" \
  connection.interface-name enp89s0 \
  ipv4.method manual \
  ipv4.addresses 192.168.4.102/24 \
  ipv4.gateway "" \
  ipv4.dns "" \
  ipv4.never-default yes \
  ipv6.method disabled

sudo nmcli connection up "有线连接 1" ifname enp89s0

# 验证连通性
ping 192.168.4.200
```

> **注意**：如果有线连接使用 DHCP（默认），会因为雷达不提供 DHCP 服务而显示"以太网连接激活失败"。必须手动配置为静态 IP。

#### 启动流程

当前 Teacher-v1 MPC-SECBF 实车工程链优先使用独立入口。默认命令只发布到
`/cmd_vel_secbf_dryrun`，不会直接驱动 Scout：

```bash
source /opt/ros/noetic/setup.bash
source /home/lxr20/lxr/SEESM_lxr/panjian_ws/devel_current/setup.bash
roslaunch swarm_test scout_secbf_real.launch
```

离线或不连接底盘时：

```bash
roslaunch swarm_test scout_secbf_real.launch \
  start_base:=false \
  cmd_vel_topic:=/cmd_vel_secbf_dryrun
```

只有完成 dry-run 和架空轮检查后，现场才显式使用：

```bash
roslaunch swarm_test scout_secbf_real.launch \
  cmd_vel_topic:=/cmd_vel \
  v_max:=0.15 \
  reverse_v_max:=0.15 \
  o_max:=0.30
```

下面的五终端流程保留为旧 ACBF 实车链调试参考。

# 终端 1：底盘
sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 500000
source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash
roslaunch scout_bringup scout_robot_base.launch

# 终端 2：LiDAR + IMU + FAST-LIO
source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash
roslaunch all_demo all_demo_no_camera.launch

# 终端 3：感知
source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash
roslaunch swarm_test start_perception.launch use_sim_time:=false show_rviz:=false

# 终端 4：规划
source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash
roslaunch swarm_test exp_acbf_planner_use.launch

# 终端 5：RViz + 急停
source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash
rviz -d ~/catkin_ws/src/swarm_test/config/exp_real_bag.rviz



---

## 可选仿真环境与修改入口

这里的“仿真环境”指论文实验里看到的 RViz/Gazebo 场景，而不是底层 simulator 包。优先按下面的场景环境选择；只有要改地图、障碍物轨迹或机器人仿真细节时，再进入 `simulation_tools`。

### 场景环境总览

| 场景环境 | 用途 | 启动入口 | 场景配置/修改点 |
|----------|------|----------|-----------------|
| 原论文 RViz 动态障碍物环境 | 复现师兄 ACBF/DCBF baseline，在 RViz 中看动态障碍物避障 | `swarm_test/launch/acbf0_planner.launch` + `swarm_test/launch/start_test.launch` | `simulation_tools/dynamic_simulator/config/obstacles_param.yaml` |
| 本文 MPC-SECBF RViz 动态语义环境 | 在同类动态障碍物 RViz 环境中跑本文方法，比较不同语义 β 和 Guard | `swarm_test/launch/secbf_planner.launch` + `swarm_test/launch/start_test.launch` | `swarm_test/config/secbf_scenarios.yaml`，或手动传 `obstacle_classes` + `obstacle_params_file` |
| 本文批量数值实验环境 | 自动跑 S1-S4、B1/B2/B3，并生成日志和摘要 | `swarm_test/scripts/run_secbf_sim_experiments.py` | `swarm_test/config/secbf_scenarios.yaml` |
| 本文静态一字排开障碍物环境 | 测试不同类别静态障碍物的避让距离差异，直观看 β 的作用 | `swarm_test/launch/secbf_static_test.launch` | `simulation_tools/dynamic_simulator/config/obstacles_param_static_beta.yaml` |
| Gazebo 动态障碍物环境 | 更接近实车的 Gazebo 模型环境，启动 Scout 和动态障碍物模型 | `swarm_test/launch/start_gazebo_env.launch` | `simulation_tools/dynamic_simulator/config/obstacles_param_gazebo.yaml`、Gazebo world |

### 1. 原论文 RViz 动态障碍物环境

这是原 baseline 主要使用的 RViz 数值仿真环境：随机地图 + 差速车 + 动态障碍物 + ACBF/DCBF planner。完整运行见上面的 “ACBF baseline 完整终端顺序”。

核心文件：

```text
swarm_test/launch/acbf0_planner.launch
swarm_test/launch/start_test.launch
simulation_tools/dynamic_simulator/config/obstacles_param.yaml
```

环境特点：

- `acbf0_planner.launch` 启动 legacy planner、随机地图、机器人仿真、全局路径搜索和 RViz。
- `start_test.launch` 启动动态障碍物、目标触发器和数据记录。
- 动态障碍物轨迹来自 `obstacles_param.yaml`；每一行是一个障碍物的初始位置和运动轨迹参数。
- 原论文 baseline 的 `controller=4`、`front_adsm=true` 当前写在 `acbf0_planner.launch` 里，命令行不能直接覆盖。

### 2. 本文 MPC-SECBF RViz 动态语义环境

这是本文方法在 RViz 中最常用的动态障碍物环境：仍然使用原来的动态障碍物和全局路径链路，但局部控制器换成 `mpc_secbf_node`，并增加语义 β 和 Guard。完整运行见上面的 “MPC-SECBF + RViz + 数据记录”。

核心文件：

```text
swarm_test/launch/secbf_planner.launch
swarm_test/launch/start_test.launch
swarm_test/config/secbf_scenarios.yaml
planner/mpc_secbf/launch/mpc_secbf.launch
planner/semantic_guard/launch/beta_ground_truth.launch
planner/semantic_guard/config/semantic_safety_margin.yaml
```

可选场景在 `swarm_test/config/secbf_scenarios.yaml`：

| 场景 | 含义 |
|------|------|
| `S1_pedestrian_crossing` | 单个行人对向/迎面穿越 |
| `S2_child_sudden` | 儿童从侧向突然进入路径 |
| `S3_feasibility_critical` | Guard 可行性关键场景，用来比较 B2 无 Guard 和 B3 有 Guard |
| `S4_mixed` | 混合动态场景：pedestrian + vehicle + box |

批量运行时脚本会自动把这些场景写成每次实验目录里的 `obstacles_param.yaml`。手动 RViz 运行时，需要保证：

- `start_test.launch num_of_obs` 等于障碍物数量。
- `secbf_planner.launch obstacle_classes` 的顺序和 `obstacles_param.yaml` 的障碍物顺序一致。
- `secbf_planner.launch output_dir` 和 `start_test.launch output_dir` 指向同一个目录。

### 3. 本文静态一字排开障碍物环境

这个环境用于测试“语义 β 改变避让距离”的效果。障碍物静止排成一条线，机器人沿全局路径经过，观察不同类别的 `β_bar(c)` 让 MPC-SECBF 产生不同避让距离。

核心文件：

```text
swarm_test/launch/secbf_static_test.launch
simulation_tools/dynamic_simulator/config/obstacles_param_static_beta.yaml
planner/semantic_guard/config/semantic_safety_margin.yaml
```

当前 `obstacles_param_static_beta.yaml` 配置了 5 个静态障碍物：

| 顺序 | 类别 | 位置 | 说明 |
|------|------|------|------|
| 0 | `pedestrian` | `(5.0, 1.5)` | 行人，β 中等 |
| 1 | `child` | `(9.0, 1.5)` | 儿童，β 最大 |
| 2 | `cyclist` | `(13.0, 1.5)` | 骑行者，β 较大 |
| 3 | `vehicle` | `(17.0, 1.5)` | 车辆，β 较大 |
| 4 | `box` | `(21.0, 1.5)` | 纸箱，β 最小 |

启动：

```bash
cd /home/lxr20/lxr/panjian_ws
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

roslaunch swarm_test secbf_static_test.launch show_rviz:=true
```

静态的关键写法是：

```yaml
slower: 999.0
scale_x: 0.0
scale_y: 0.0
scale_z: 0.0
```

如果只想做“三个一条线上的静态障碍物”，按下面三处同步改：

1. 在 `obstacles_param_static_beta.yaml` 只保留 3 行障碍物，例如 pedestrian、child、box。
2. 在 `secbf_static_test.launch` 中把 `args="5 false"` 改成 `args="3 false"`。
3. 在 `secbf_static_test.launch` 中把 `obstacle_classes` 改成和三行障碍物一致，例如 `[pedestrian, child, box]`。

输出日志：

```text
swarm_test/output/guard_log_static.csv
```

### 4. Gazebo 动态障碍物环境

Gazebo 环境用于更接近实车的模型验证。它启动 Gazebo world、Scout 模型和动态障碍物模型。完整流程见 “Gazebo 仿真”。

核心文件：

```text
swarm_test/launch/start_gazebo_env.launch
simulation_tools/dynamic_simulator/config/obstacles_param_gazebo.yaml
simulation_models/scout/scout_gazebo_sim/worlds/simple_env.world
```

常改位置：

| 想改什么 | 改哪里 |
|----------|--------|
| Gazebo world | `start_gazebo_env.launch` 的 `world_name` |
| 是否打开 Gazebo 界面 | `gzclient:=true/false` |
| Gazebo 动态障碍物数量 | `start_gazebo_env.launch` include `spawn_dynamic_obstacle1.launch` 时的 `num_of_obs` |
| Gazebo 障碍物轨迹 | `simulation_tools/dynamic_simulator/config/obstacles_param_gazebo.yaml` |
| Scout 初始位姿 | `start_gazebo_env.launch` include `spawn_scout_v2_use1.launch` 的 `x1/y1/z1/yaw1` |

### 底层组件入口

### 数值仿真怎么改

完整数值仿真一般是“两终端结构”：一个 planner launch，另一个环境和数据 launch。

```bash
# ACBF baseline planner
roslaunch swarm_test acbf0_planner.launch show_rviz:=true

# 或本文 MPC-SECBF planner
roslaunch swarm_test secbf_planner.launch show_rviz:=true

# 环境、动态障碍物、触发器、数据记录
roslaunch swarm_test start_test.launch
```

常改位置：

| 想改什么 | 改哪里 |
|----------|--------|
| 地图范围 | `acbf0_planner.launch` / `secbf_planner.launch` 里的 `map_size_x`、`map_size_y`、`map_size_z` |
| 地图障碍物数量 | include `map_generator/launch/map.launch` 时传入的 `p_num`、`c_num` |
| 动态障碍物数量 | `start_test.launch` 的 `num_of_obs`，必须和障碍物 YAML 中启用条目数量一致 |
| 动态障碍物轨迹 | `obstacle_params_file` 指向的 YAML，默认是 `simulation_tools/dynamic_simulator/config/obstacles_param.yaml` |
| 数据输出目录 | `start_test.launch output_dir:=...`；SECBF 还要让 `secbf_planner.launch output_dir:=...` 指向同一个目录 |
| legacy 控制器类型 | `acbf0_planner.launch` 中 `controller` 目前是固定 `value`；要切换 0/1/2/3/4 需编辑 launch，或使用其它允许覆盖的 legacy planner launch |
| SECBF 语义类别 | `secbf_planner.launch obstacle_classes:='[pedestrian,vehicle,box]'`，顺序必须和障碍物 YAML 一致 |
| SECBF Guard 开关 | `secbf_planner.launch guard_enabled:=true/false` |
| SECBF β 参数 | `planner/semantic_guard/config/semantic_safety_margin.yaml` |

### 动态障碍物 YAML 怎么写

数值仿真默认读取：

```text
simulation_tools/dynamic_simulator/config/obstacles_param.yaml
```

Gazebo 动态障碍物默认读取：

```text
simulation_tools/dynamic_simulator/config/obstacles_param_gazebo.yaml
```

每个障碍物是一行：

```yaml
- {x: 6.0, y: -1.0, z: 0.75, offset: 5.0, slower: 8.0, scale_x: 15.0, scale_y: 0.0, scale_z: 0.0}
```

字段含义：

| 字段 | 含义 | 常见改法 |
|------|------|----------|
| `x/y/z` | 初始位置 | 改障碍物出现位置；`z` 通常 0.75 或 1.0 |
| `offset` | 轨迹相位/时间偏移 | 改障碍物进入冲突区域的时机 |
| `slower` | 轨迹时间尺度 | 数值越大运动越慢 |
| `scale_x/y/z` | 轨迹振幅和方向 | `scale_x` 控制横向/前后移动，`scale_y` 控制侧向移动；正负号决定方向 |

修改建议：不要直接覆盖默认文件做一次性实验。更稳的做法是复制一个 YAML 到输出目录，然后通过 launch 参数指定：

```bash
mkdir -p swarm_test/output/custom_runs/test1
cp simulation_tools/dynamic_simulator/config/obstacles_param.yaml \
  swarm_test/output/custom_runs/test1/obstacles_param.yaml

roslaunch swarm_test start_test.launch \
  num_of_obs:=3 \
  obstacle_params_file:=$(pwd)/swarm_test/output/custom_runs/test1/obstacles_param.yaml \
  output_dir:=$(pwd)/swarm_test/output/custom_runs/test1 \
  record_data:=true
```

### 单独调试仿真组件

只看随机地图：

```bash
roslaunch map_generator map.launch \
  rviz_vis:=true \
  map_size_x_:=50.0 \
  map_size_y_:=50.0 \
  map_size_z_:=3.0 \
  p_num:=20 \
  c_num:=0
```

只看动态障碍物：

```bash
roslaunch dynamic_simulator spawn_dynamic_obstacle.launch \
  rviz:=true \
  num_of_obs:=5 \
  obstacle_params_file:=$(pwd)/simulation_tools/dynamic_simulator/config/obstacles_param.yaml
```

只看机器人、地图和键盘控制：

```bash
roslaunch robot_simulator vis_car.launch rviz_vis:=true
```

启动 Gazebo 环境：

```bash
roslaunch swarm_test start_gazebo_env.launch \
  gzclient:=true \
  use_sim_time:=true
```

Gazebo 常改位置：

| 想改什么 | 改哪里 |
|----------|--------|
| world 文件 | `swarm_test/launch/start_gazebo_env.launch` 的 `world_name`，默认 `simulation_models/scout/scout_gazebo_sim/worlds/simple_env.world` |
| 是否打开 Gazebo 界面 | `gzclient:=true/false` |
| 动态障碍物数量 | `start_gazebo_env.launch` include `spawn_dynamic_obstacle1.launch` 时的 `num_of_obs` |
| Gazebo 障碍物轨迹 | `simulation_tools/dynamic_simulator/config/obstacles_param_gazebo.yaml` |
| Scout 初始位姿 | `start_gazebo_env.launch` include `spawn_scout_v2_use1.launch` 的 `x1/y1/z1/yaw1` |

### 推荐修改流程

1. 先确定要改的是“完整实验”还是“组件单测”。完整实验优先改 `swarm_test/launch/*.launch` 和 `swarm_test/config/*.yaml`。
2. 动态障碍物实验优先复制 YAML 到 `swarm_test/output/...`，再用 `obstacle_params_file:=...` 指定，避免污染默认场景。
3. 改 `num_of_obs` 时同步检查 YAML 启用条目数；SECBF 还要同步检查 `obstacle_classes` 数量和顺序。
4. 改 RViz 显示时直接在 RViz 中调整并 Save 到对应 `.rviz`；常用入口是 `swarm_test/config/acbf_plan.rviz`。
5. 如果 launch 能启动但车不避障，优先检查 `/Dyn_Obs_trajs`、`/globalFsm_by_adsm/obs_predict_pub`、`/cmd_vel1`、`/safety_margin/beta` 这几个话题。

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
│   ├── dwa_planner/             # 传统 DWA baseline
│   ├── mpc_dcbf/                # 师兄原 MPC-DCBF/ACBF baseline 代码
│   ├── mpc_secbf/               # 本文 MPC-SECBF 控制器
│   ├── semantic_guard/          # 本文 β 计算、Guard 与日志
│   └── vomp_planner/traj_planner/  # 全局路径搜索 (Theta*, B-spline)
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

### MPC-SECBF 论文代码目录

本文代码按“控制器”和“语义安全余量”拆成两个 ROS 包，和已有 baseline 并列：

```text
planner/
├── dwa_planner/                 # 传统局部规划 baseline
├── mpc_dcbf/                    # 原 MPC-DCBF/ACBF baseline
├── mpc_secbf/
│   ├── include/mpc_secbf/       # MPC-SECBF 类定义
│   ├── launch/mpc_secbf.launch  # 独立控制器 launch
│   └── src/                     # h_SEE、SECBF 约束、β 订阅
└── semantic_guard/
    ├── config/semantic_safety_margin.yaml  # β_bar、μ(φ)、Guard 参数
    ├── launch/beta_guard.launch
    ├── msg/GuardLog.msg
    └── src/                     # beta_ground_truth_node / beta_guard_node
```

实验编排仍放在 `swarm_test`，因为它同时服务 baseline 和本文方法：

```text
swarm_test/
├── config/secbf_scenarios.yaml          # S1-S4 场景 + B1/B2/B3 baseline 定义
├── launch/secbf_planner.launch          # 数值仿真：MPC-SECBF + semantic_guard
├── launch_exp/exp_secbf_planner.launch  # 实车：MPC-SECBF + perception/fusion/guard
└── scripts/run_secbf_sim_experiments.py # 批量实验脚本
```

对应论文公式：

```text
h_EE  = ||l + τv|| - R_obs - R_robot      -> mpc_secbf + semantic_guard Guard
h_SEE = h_EE - β_i                        -> mpc_secbf
β_i   = β_bar(c_i) × μ(φ_i)               -> semantic_guard
Guard: β_i <= h_EE - η                    -> semantic_guard
CBF: h_SEE(k+1) >= (1-γ) h_SEE(k)         -> mpc_secbf
```

## Controller 类型

`controller` 参数是 `swarm_test` 数据记录器使用的实验标签，也是 legacy `mpc_dcbf` baseline 的方法编号。本文 MPC-SECBF 不塞进 `mpc_dcbf` 内部，而是独立运行 `planner/mpc_secbf`；在 SECBF 实验里 `controller_index:=6` 仅用于 CSV 标记“本文方法”。

legacy launch 文件中 `controller` 参数：

| 值 | 方法 |
|----|------|
| 0 | None (无约束) |
| 1 | MPC-DC (距离约束) |
| 2 | MPC-SCBF (静态 CBF) |
| 3 | MPC-DCBF (动态 CBF) |
| 4 | MPC-ACBF (自适应 CBF) |
| 6 | MPC-SECBF (本文方法，数据记录标签) |

## 避障调参经验

### 关键参数速查

以下经验主要针对 `planner/mpc_dcbf` 里的 legacy ACBF/DCBF baseline。本文 MPC-SECBF 的安全余量由 `planner/semantic_guard/config/semantic_safety_margin.yaml` 和 `/safety_margin/beta` 决定，不再手动修改固定 `safe_dist`。

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
| `safe_dist` | mpc_cbf.cpp | 0.3 + 0.4 | legacy ACBF 固定安全余量 + 机器人半径（硬编码） |

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
2. **增加 legacy 安全余量**：`planner/mpc_dcbf/src/mpc_cbf.cpp` 第 54 行 `safe_dist = 0.3 + 0.4` 改成 `0.5 + 0.4`（仅影响 `mpc_dcbf`，需重编）
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
6. **LiDAR 网口配置**：雷达 IP `192.168.4.200`，上位机必须配 `192.168.4.102/24`（静态），DHCP 会导致连接失败
7. **PCD 保存目录**：FAST-LIO 的 `ROOT_DIR` 在编译时硬编码为 CMake 源码路径，需确保 `~/catkin_ws/src/state_estimation/FAST_LIO/PCD/` 目录存在，否则保存时报错
