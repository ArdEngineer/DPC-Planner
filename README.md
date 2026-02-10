# DPC-Planner 自主无人机路径规划器

[![ROS Version](https://img.shields.io/badge/ROS-Melodic%2FNoetic-blue)](http://wiki.ros.org/)
[![C++](https://img.shields.io/badge/C%2B%2B-11%2F14-orange)](https://isocpp.org/)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)

一个基于ROS的无人机实时路径规划器，采用**前端A*搜索 + 后端B样条优化**的经典架构，支持动态避障和实时重规划。

---

## 目录

- [功能特性](#功能特性)
- [系统架构](#系统架构)
- [安装指南](#安装指南)
- [快速开始](#快速开始)
- [参数配置](#参数配置)
- [ROS接口](#ros接口)
- [可视化](#可视化)
- [性能指标](#性能指标)
- [常见问题](#常见问题)

---

## 功能特性

### 核心功能
- **实时3D路径规划**：基于OctoMap构建三维占据地图
- **混合规划策略**：前端A*搜索 + 后端B样条轨迹优化
- **动态重规划**：支持飞行过程中的实时重规划
- **碰撞检测**：基于八叉树的高效碰撞检测
- **轨迹平滑**：使用NLOpt优化器生成平滑的B样条轨迹

### 技术特点
- **多维度搜索**：支持2D（8邻域）和3D（26邻域）A*搜索模式
- **安全机制**：自动寻找安全临时目标点
- **轨迹约束**：考虑速度、加速度、Jerk约束
- **可视化支持**：完整的RViz可视化接口

---

## 系统架构

### 核心模块

#### 1. MapUtils (`map_utils.h/cpp`)
- **功能**：基于OctoMap的三维环境地图管理
- **主要职责**：
  - 点云数据接收与处理
  - 占据栅格地图更新
  - 碰撞检测与边界检查
  - 地图可视化发布

#### 2. AStarSearch (`dyn_a_star.h/cpp`)
- **功能**：动态A*路径搜索算法
- **搜索模式**：
  - 2D模式：8邻域搜索（水平面）
  - 3D模式：26邻域搜索（全空间）
- **特性**：支持最大迭代次数、时间限制、启发函数调节

#### 3. TrajOptimizer (`traj_optimizer.h/cpp`)
- **功能**：基于NLOpt的轨迹优化器
- **优化目标**：
  - 平滑性惩罚（加速度、Jerk）
  - 碰撞惩罚（安全距离约束）
  - 动态约束（速度、加速度限制）
- **输出**：B样条曲线采样点

#### 4. Planner (`planner_utils.h/cpp`)
- **功能**：核心规划逻辑协调器
- **主要职责**：
  - 规划状态机管理
  - 引导路径生成
  - 碰撞段检测与处理
  - 优化问题构造

---

## 安装指南

### 环境要求

| 依赖项 | 版本要求 |
|--------|----------|
| ROS | Melodic / Noetic |
| Ubuntu | 18.04 / 20.04 |
| C++ | 11 或更高 |
| CMake | 3.0.2+ |

### 依赖库

```bash
# ROS依赖
sudo apt-get install ros-$ROS_DISTRO-octomap*
sudo apt-get install ros-$ROS_DISTRO-pcl*
sudo apt-get install ros-$ROS_DISTRO-tf

# 系统依赖
sudo apt-get install libnlopt-dev
sudo apt-get install libceres-dev
sudo apt-get install libeigen3-dev
sudo apt-get install libpcl-dev
```

### 编译安装

```bash
# 1. 进入工作空间
cd ~/catkin_ws/src

# 2. 克隆仓库
git clone <repository_url> planner_manage

# 3. 编译
cd ~/catkin_ws
catkin_make

# 4.  source 环境
source devel/setup.bash
```

### 添加到.bashrc（可选）

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

---

## 快速开始

### 1. 启动仿真环境

确保已配置好无人机仿真环境（如Gazebo + PX4 / MAVROS）。

### 2. 启动规划器

```bash
roslaunch planner_manage planner.launch
```

### 3. 设置目标点

```bash
# 方式1：使用rostopic
rostopic pub /goal geometry_msgs/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 10.0, y: 5.0, z: 2.0}}}'

# 方式2：使用RViz的2D Nav Goal工具
# 订阅话题：/goal
```

### 4. 查看可视化结果

打开RViz，添加以下显示项：
- `/visualized_guidance_path` - 引导路径（黑色）
- `/visualized_planning_path` - 优化后路径（蓝色）
- `/visualized_astar_path` - A*搜索结果（红色）
- `/map_utils/octo_map` - 八叉树地图

---

## 参数配置

### 基本参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `test_mode` | bool | false | 测试模式开关 |
| `planner_freq` | double | 10.0 | 规划器运行频率(Hz) |
| `plan_len` | double | 7.0 | 单次规划段长度(m) |
| `reached_threshold` | double | 0.4 | 到达目标判断阈值(m) |

### A*搜索参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `astar_search_mode` | int | 2 | 搜索维度(2=2D, 3=3D) |
| `astar_step_size` | double | 0.1 | 搜索步长(m) |
| `astar_max_iter` | int | 5000 | 最大迭代次数 |
| `astar_max_time` | double | 0.2 | 最大搜索时间(s) |
| `astar_hfunc_factor` | double | 10.0 | 启发函数缩放因子 |

### 优化器参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `dt` | double | 0.2 | 时间步长(s) |
| `sf` | double | 0.8 | 安全距离(m) |
| `lambda_s` | double | 1.0 | 平滑性权重 |
| `lambda_c` | double | 80.0 | 碰撞惩罚权重 |
| `lambda_d` | double | 2.0 | 动态约束权重 |
| `opt_xtol_rel` | double | 1e-6 | 优化容差 |
| `opt_maxeval` | int | 50 | 最大优化迭代次数 |

### 地图参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `map_resolution` | double | 0.1 | 地图分辨率(m) |
| `map_z_min` | double | 0.1 | 地图Z轴最小值(m) |
| `map_z_max` | double | 5.0 | 地图Z轴最大值(m) |
| `map_r` | double | 20.0 | 地图有效半径(m) |
| `pro_hit` | double | 0.7 | 命中概率更新值 |
| `pro_miss` | double | 0.4 | 未命中概率更新值 |

### 运动参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `max_v` | double | 2.5 | 最大速度(m/s) |
| `average_v` | double | 1.0 | 平均速度(m/s) |
| `replan_threshold_dist` | double | 3.5 | 重规划触发距离(m) |

---

## ROS接口

### 订阅话题 (Subscribers)

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/iris_0/mavros/vision_pose/pose` | `geometry_msgs/PoseStamped` | 无人机当前位姿 |
| `/goal` | `geometry_msgs/PoseStamped` | 目标点设置 |
| `/points_stable_inflate` | `sensor_msgs/PointCloud2` | 点云地图数据 |

### 发布话题 (Publishers)

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/mission_planning_path` | `geometry_msgs/PoseArray` | 规划路径（发送给控制器） |
| `/visualized_planning_path` | `visualization_msgs/Marker` | 优化后路径可视化 |
| `/visualized_guidance_path` | `visualization_msgs/Marker` | 引导路径可视化 |
| `/visualized_astar_path` | `visualization_msgs/Marker` | A*路径可视化 |
| `/map_utils/octo_map` | `octomap_msgs/Octomap` | 八叉树地图 |
| `/interpret_signal` | `std_msgs/Char` | 飞行中断信号 |
| `/emergency_hover_cmd` | `geometry_msgs/Twist` | 紧急悬停指令 |

---

## 可视化

### RViz配置

```bash
# 启动RViz
rosrun rviz rviz

# 添加显示项：
# 1. Marker - /visualized_guidance_path (黑色引导路径)
# 2. Marker - /visualized_planning_path (蓝色优化路径)
# 3. Marker - /visualized_astar_path (红色A*路径)
# 4. OctoMap - /map_utils/octo_map (占据地图)
# 5. PoseArray - /mission_planning_path (任务点)
```

### 路径颜色说明

| 颜色 | 路径类型 | 说明 |
|------|----------|------|
| 黑色 | 引导路径 | 初始生成的粗略路径 |
| 红色 | A*路径 | 碰撞段的绕行路径 |
| 蓝色 | 优化路径 | 最终平滑的B样条轨迹 |

---

## 性能指标

### 规划性能

| 指标 | 典型值 | 说明 |
|------|--------|------|
| 规划频率 | 10 Hz | 主循环运行频率 |
| A*搜索时间 | < 200 ms | 单次搜索最大时间限制 |
| 优化时间 | < 100 ms | 典型优化求解时间 |
| 重规划延迟 | < 50 ms | 从触发到输出路径 |

### 轨迹质量

| 指标 | 说明 |
|------|------|
| 连续性 | C2连续（位置、速度、加速度） |
| 约束满足 | 速度、加速度、Jerk约束 |
| 安全距离 | 默认0.8m障碍物距离 |

---

## 常见问题

### Q1: 编译错误 - 找不到NLopt

**解决方案：**
```bash
sudo apt-get install libnlopt-dev
# 如果仍有问题，检查CMakeLists.txt中的NLOPT_INCLUDE_DIR
```

### Q2: A*搜索失败，返回空路径

**可能原因：**
- 起点或终点在障碍物内
- 搜索步长过大
- 最大迭代次数不足

**解决方案：**
- 减小 `astar_step_size`
- 增加 `astar_max_iter`
- 检查地图数据

### Q3: 轨迹不平滑

**解决方案：**
- 增加 `lambda_s`（平滑性权重）
- 减小 `dt`（时间步长）
- 增加 `num_of_control_points`

### Q4: 规划器不接收目标点

**检查项：**
- 确认 `/goal` 话题有数据发布
- 检查 `set_goal_topic` 参数配置
- 查看ROS日志是否有错误信息

---

## 开发计划

- [ ] 支持动态障碍物预测
- [ ] 添加多无人机协同规划
- [ ] 集成更高效的优化器（如MPC）
- [ ] 支持非结构化环境（如森林）

---

## 许可证

本项目采用 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件

---

## 联系方式

如有问题或建议，欢迎提交Issue或Pull Request。

---

**致谢**：本项目开发过程中参考了以下开源项目：
- [OctoMap](https://octomap.github.io/)
- [NLopt](https://nlopt.readthedocs.io/)
- [Ceres Solver](http://ceres-solver.org/)
