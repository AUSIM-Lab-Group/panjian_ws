在ubuntu20.04 docker中复现算法仓库，有挺多问题，工具：claude code opus4.7，codex5.4，插件superpower

# MPC-A-CBF


## 依赖安装

```bash
nlopt
ipopt
casadi

# 编译过程中报错确实某些msg消息头文件，需要使用catkin_make先编指定package，在恢复默认编译
catkin_make -DCATKIN_WHITELIST_PACKAGES="livox_ros_driver;fast_lio;dynamic_simulator" -DCMAKE_CXX_STANDARD=14
catkin_make -DCATKIN_WHITELIST_PACKAGES="" -DCMAKE_CXX_STANDARD=14
```

## 启动

数值仿真启动
```bash
source devel/setup.bash
# 启动全局规划
roslaunch swarm_test acbf_planning1.launch # FSM
# 启动mpc (包含全局路径数据处理器)
roslaunch swarm_test start_test.launch
```

Gazebo仿真启动
```bash
source devel/setup.bash
# 启动Gazebo环境
roslaunch swarm_test start_gazebo_env.launch # 在机器人启动节点那设置 `use_fast_lio_`符号位设置为 `true`-这与fast-lio发布里程相关
# 启动状态估计模块-fast lio
roslaunch swarm_test exp_hardware.launch
# 启动全局规划
# obs_manager部分可以修改感知部分是否使用真值-；
# 不使用真值时，需要将start_gazebo_env.launch中arg /use_sim_time变量的设置为true，以及exp_acbf_planning1.launch的 param /use_sim_time参数的设置为true
roslaunch swarm_test exp_acbf_planning1.launch # FSM
# 启动mpc
roslaunch mpc_dcbf mpc_adsm_c.launch

```

## 全局路径数据处理器

系统新增了全局路径数据处理器(`global_path_data_processor.py`)，用于记录和分析全局路径规划的性能指标。

### 功能特性
- 自动监听 `/global_path` 和 `/globalFsm_by_adsm/obs_predict_pub` 话题
- 记录前3次路径重规划的详细数据
- 计算路径搜索时间、速度方差、角速度方差等指标  
- 分析轨迹与障碍物的安全距离和ISAD评分
- 自动保存分析结果到 `swarm_test/output/` 目录

### 测试使用
```bash
# 启动数据处理器
roslaunch swarm_test start_test.launch

# 或者单独测试
rosrun swarm_test global_path_data_processor.py

# 运行测试脚本(模拟发布路径数据)
rosrun swarm_test test_global_path_processor.py
```

### 输出数据
分析结果保存为JSON格式，包含：
- 元数据信息(时间戳、参数配置等)
- 汇总统计(平均搜索时间、安全指标等)
- 每次重规划的详细数据

详细使用说明请参考：`src/swarm_test/scripts/README_global_path_processor.md`

## 实车实验

启动脚本在launch_exp文件夹中

```bash
# 启动底盘前需要 联通地盘通信
sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 500000
```
实车实验需要记录的话题名:

```bash
# 原始点云,图像,里程计信息
rosbag record /fastLIO/points_world  /camera/color/image_raw /Odometry

rosbag record /fastLIO/points_world /Odometry /l_shape_fitting/jsk_bbox_array /obstacle_prediction_node/obstacle_prediction/trajs_predicted /move_base_simple/goal /globalFsm_by_adsm/obs_traj_vis /globalFsm_by_adsm/obs_predict_pub /global_path /local_path /cmd_vel
# 需要进行可视化的话题信息 图像,原始点云,里程计,障碍物预测轨迹(球体),global_path,local_path,/cmd_vel,
rosbag record /fastLIO/points_world /camera/color/image_raw /Odometry /sdf_map/occupancy_inflate /obstacle_prediction_node/obstacle_prediction/trajs_predicted /globalFsm_by_adsm/obs_traj_vis /global_path /local_path /cmd_vel /cmd_vel1
```

