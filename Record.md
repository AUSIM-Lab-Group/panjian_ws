# 修改记录
快速使用
```bash
source devel/setup.bash
roslaunch swarm_test acbf0_planner.launch  # 启动仿真脚本: 包含全局规划，局部规划
roslaunch swarm_test start_test.launch # 启动测试脚本: 包含动态障碍物启动节点，数据记录节点
```

障碍物设置文件: `src/simulation_tools/dynamic_simulator/config/obstacles_param.yaml`  
数据记录节点文件: `src/swarm_test/src/data_processor.cpp`
全局规划碰撞检查函数文件: `src/planner/vomp_planner/traj_planner/include/obs_manager/obs_manager.hpp`

需要注意：
1. 设计障碍物运动形式
2. 数据记录文件中，添加全局路径的指标评估数据获取函数：选择至少4个指标



## 时间线

- 24-01-23 修改主CBF函数 关于MPC问题的格式修改-增加controller标志位
- 24-01-24 复现MPC-DC，MPC-SCBF，MPC-DCBF，MPC-ACBF方法
- 24-01-24 对MPC预测步数，步长时间的修改同步至mpc_acbf, global_path_publish, obstacle_manager，统一在launch文件参数修改
- 24-02-25 增加前端全局路径搜索部分 节点文件是`global_path_by_adsm.cpp`
- 24-03-15 修改障碍物的运动形式-修改为匀速往返运动形式;
- 24-03-19 增加MPC-ACBF软约束的约束形式;
- 24-04-22 增加Gazebo仿真环境实验 `start_gazebo_env.launch`启动gazebo的环境;
- 24-05-13 增加mpc-cbf C++版本，以及全局规划的状态机;
- 24-07-10 增加mpc-cbf one-horizen实验脚本 `acbf0_one_horizen.launch`启动仿真环境； 
- 24-09-15 增加实车实验脚本 `exp_hardware.launch; start_perception.launch; exp_acbf_planner.launch`;
- 24-10-10 增加实车数据bag播放脚本 `exp_play_bag_use.launch`；

## 注意

### 数值实验

需要注意：
```bash
source devel/setup.bash

roslaunch swarm_test acbf0_one_horizen.launch  # 启动同一基准实验脚本

roslaunch swarm_test acbf0_planner.launch  # 启动总体的运动规划实验脚本
roslaunch swarm_test start_test.launch
roslaunch swarm_test data_process.launch  # 启动数据处理脚本，录制bag

```

播放bag时：
```bash
roslaunch swarm_test show_mpc_traj_single.launch  # 显示单个MPC运动轨迹
roslaunch swarm_test show_mpc_traj.launch  # 显示多个MPC运动轨迹

```

### Gazebo仿真实验

需要注意：
1. `start_gazebo_env.launch`、`exp_hardware.launch`、`exp_acbf_planner.launch` 现在统一通过 `use_sim_time` 控制仿真时间，Gazebo 主流程默认使用 `true`。
2. `start_gazebo_env.launch` 默认把机器人启动节点里的 `use_fast_lio_` 置为 `true`，避免 Gazebo 真值 TF 与 Fast-LIO TF 冲突。
3. Gazebo 默认主流程使用 3 个终端，不再额外并行启动 `roslaunch mpc_dcbf mpc_adsm_c.launch`，因为 `exp_acbf_planner.launch` 已经内嵌 `local_planner`。

推荐启动顺序：
```bash
source devel/setup.bash

# 终端 1
roslaunch swarm_test start_gazebo_env.launch gzclient:=true

# 终端 2
roslaunch swarm_test exp_hardware.launch

# 终端 3
roslaunch swarm_test exp_acbf_planner.launch show_rviz:=true
```

播放bag时：
```bash
roslaunch swarm_test exp_play_bag_use.launch  # 显示实车实验bag数据

```
