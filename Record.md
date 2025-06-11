# 修改记录
快速使用
```bash
source develop/setup.bash
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
source develop/setup.bash

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
1.播放bag时,需要在 `start_perception.launch`和 `exp_start_test.launch`文件中的 `/use_sim_time`设置为 `true`。这样才会使bag里的系统时间与本机的gazebo仿真时间匹配上。
2.在 `start_gazebo_env.launch`中启动gazebo仿真环境，其中加载机器人模型时会发布机器人在gazebo中的odometry真值 `/Odometry`以及发布TF坐标(odom-basklink)，但因为fast-lio模块也会发布TF坐标变换信息（world-camera, body-basklink）可能会有冲突，所以需要在机器人启动节点那设置 `use_fast_lio_`符号位设置为 `true`

播放bag时：
```bash
roslaunch swarm_test exp_play_bag_use.launch  # 显示实车实验bag数据

```