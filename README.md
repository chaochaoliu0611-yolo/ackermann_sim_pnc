# SAYE — Ackermann 小车自定义规划控制系统

> 基于 ROS2 Humble + Gazebo Harmonic 仿真，从直接调用 Nav2 标准控制器重构为**自定义 pluginlib 插件架构**的自主导航系统。

---

## 与原项目的区别

本项目 fork 自 [alitekes1/ackermann-vehicle-gzsim-ros2](https://github.com/alitekes1/ackermann-vehicle-gzsim-ros2)，进行了深度改造：

| 维度 | 原项目 | 本项目 |
|------|--------|--------|
| 导航架构 | 直接调用 Nav2 标准控制器 | **自定义 pluginlib 插件** — Hybrid A\* + PurePursuit/MPC/Stanley |
| 全局规划器 | Nav2 SmacPlannerHybrid | **Hybrid A\* ** — 18 运动原语 + OMPL Reeds-Shepp + Dijkstra 启发式 |
| 局部控制器 | Nav2 MPPI / RPP | **PurePursuit / MPC(OSQP) / Stanley** 三合一可热切换 |
| 路径坐标系 | 依赖标准控制器内部变换 | **自行实现 `transformGlobalPlan()`** — 修复 map/odom 帧不匹配 |
| 转向语义 | 有误（angular.z 作为转角） | **修正为横摆角速度** — 回溯 Gazebo 插件源码确认 |
| 模型渲染 | RViz 只显示绿框 | **SDF→URDF 转换 + 4层问题修复** — 完整 3D 模型显示 |
| costmap 范围 | obstacle_max_range=2.5m | **25m** — 实现远距离提前绕障 |
| 文档 | 英文 | **中英双语** + 面试素材 + 问题修复记录 |

---

## 项目概述

### 车辆参数

| 参数 | 值 |
|------|-----|
| 车身尺寸 | 0.30m × 0.09m × 0.12m |
| 轴距 | 0.2255 m |
| 最大转向角 | 0.4 rad |
| 最小转弯半径 | ≈ 0.53 m |
| 质量 | 3.0 kg |
| 驱动方式 | Ackermann 前轮转向，后轮差速驱动 |

### 传感器

| 传感器 | 型号/类型 | 参数 |
|--------|----------|------|
| 3D LiDAR | GPU LiDAR | 1024 × 128 采样，360°水平，±30°垂直，50m 范围 |
| IMU | 6 轴 | 250 Hz，高斯噪声模拟 |
| 相机 × 4 | RGB | 前/后/左/右（默认仅前相机桥接） |
| 里程计 | OdometryPublisher | 车身坐标系 odom→saye |

### 技术栈

| 组件 | 版本/配置 |
|------|----------|
| ROS2 | Humble |
| Gazebo | Sim Harmonic (gz-sim8) |
| Nav2 | Humble 内置，自定义插件替换规划器和控制器 |
| 规划器 | 自定义 Hybrid A\* (`saye_nav2_plugins/HybridAStarPlanner`) |
| 控制器 | 自定义 PurePursuit / MPC / Stanley（可切换） |
| QP 求解器 | OSQP（预编译静态库） |
| 运动规划库 | OMPL 1.7（Reeds-Shepp 状态空间） |
| 线性代数 | Eigen3 |

---

## 系统架构

```
┌──────────────────────────────────────────────────────────────────┐
│                      Gazebo Sim Harmonic                         │
│  ┌──────────┐  ┌───────────┐  ┌───────────────────────────────┐  │
│  │Ackermann │  │  Sensors  │  │   Warehouse World             │  │
│  │Steering  │  │ LiDAR     │  │   + A/I/L Letter Landmarks    │  │
│  │Plugin    │  │ IMU       │  │                               │  │
│  │          │  │ 4x Camera │  │                               │  │
│  └────┬─────┘  └─────┬─────┘  └───────────────────────────────┘  │
│       │ cmd_vel      │ /odom, /scan, /imu, /cloud, /camera      │
│       ▼              ▼                                            │
│       ┌────────────────────────────────────────────────────────┐  │
│       │           ros_gz_bridge                                │  │
│       │  /cmd_vel ↔ Gazebo | /odom /scan /imu → ROS           │  │
│       │  /joint_states → ROS  | /clock → ROS                  │  │
│       └───────────────────────┬────────────────────────────────┘  │
└──────────────────────────────┼───────────────────────────────────┘
                               │
      ┌────────────────────────┼────────────────────────────┐
      │                    ROS2 Network                      │
      │                                                      │
      ▼                        ▼                             ▼
┌──────────┐          ┌────────────────┐          ┌──────────────────┐
│ odom_to_ │          │  Nav2 Stack     │          │  RViz2           │
│ tf.py    │          │                 │          │  ─ 3D 小车模型   │
│ odom→saye│          │ ┌─────────────┐ │          │  ─ costmap       │
└──────────┘          │ │  AMCL /     │ │          │  ─ 路径          │
                      │ │  map_server │ │          │  ─ LiDAR 点云    │
                      │ └──────┬──────┘ │          └──────────────────┘
                      │        │ map    │
                      │        ▼        │
                      │ ┌─────────────┐ │
                      │ │HybridAStar  │ │    自定义 Nav2 plugins:
                      │ │ Planner     │ │    saye_nav2_plugins/
                      │ └──────┬──────┘ │
                      │        │ path   │
                      │        ▼        │
                      │ ┌─────────────┐ │
                      │ │ Controller  │ │    ┌─────────────────┐
                      │ │(可切换:)    │ │    │ PurePursuit     │
                      │ │             │ │    │ MPC (OSQP)      │
                      │ │             │ │    │ Stanley         │
                      │ └──────┬──────┘ │    └─────────────────┘
                      │        │ cmd_vel│
                      │        ▼        │
                      │ velocity_smoother│  (angular.z 直通)
                      │        │        │
                      │  costmaps(global │
                      │  +local, 25m)   │
                      └────────┼────────┘
                               │ /cmd_vel
                               ▼
                      Gazebo AckermannSteering
                      r = v/ω → δ = atan(L/r)
```

---

## 快速开始

### 环境要求

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Sim Harmonic
- 已安装 `ros-humble-ros-gz` 和 `ros-humble-ros-gzharmonic`

### 安装

```bash
# 1. 克隆
mkdir -p ~/ackermann_sim/src && cd ~/ackermann_sim/src
git clone <your-repo-url> ackermann-vehicle-gzsim-ros2
cd ..

# 2. 设置环境变量
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/src/ackermann-vehicle-gzsim-ros2/
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/src/ackermann-vehicle-gzsim-ros2/

# 3. 编译
colcon build --packages-up-to saye_nav2_plugins
source install/setup.bash
```

### 运行

**终端 1 — 仿真：**
```bash
source install/setup.bash
ros2 launch saye_bringup saye_spawn.launch.py rviz:=false
```

**终端 2 — 导航 + RViz：**
```bash
source install/setup.bash
ros2 launch saye_nav2_plugins saye_nav_bringup.launch.py
```

### 切换控制器

修改 `saye_nav2_plugins/config/nav2_params_saye.yaml` 第 157 行：

```yaml
FollowPath:
  plugin: "saye_nav2_plugins/PurePursuitController"    # 当前默认
  # plugin: "saye_nav2_plugins/MPCController"          # MPC 模式
  # plugin: "saye_nav2_plugins/StanleyController"       # Stanley 模式
```

改完后重启导航终端即可生效。

---

## 自定义插件详解

### Hybrid A\* 全局规划器

**文件**：`saye_nav2_plugins/src/hybrid_astar_planner.cpp`

| 特性 | 说明 |
|------|------|
| 运动原语 | 18 个（9 个离散转向角 × 前进/后退） |
| 状态空间 | SE2 (x, y, θ)，64-bit 离散 key |
| 碰撞检测 | 13 点车辆 footprint 采样 + 弧段采样 |
| 启发式 | Dijkstra BFS wavefront（从目标向外展开）替代欧氏距离 |
| 解析扩展 | OMPL ReedsSheppStateSpace，每 30 次 A\* 迭代尝试跳帧到目标 |
| 路径平滑 | Nav2 SimpleSmoother（1000 次梯度下降迭代）替代原五次多项式 |

### PurePursuit 控制器

**公式**：`δ = arctan(2L·sin(α) / Ld)`

| 参数 | 值 |
|------|-----|
| 前视距离 | 0.3~1.0m（速度自适应，`lookahead_time=1.5s`） |
| 前视点搜索 | 弧长法（避免最近点弯道拉扯） |
| 前馈 | 参考路径曲率（30% 权重） |
| 巡航速度 | 0.35 m/s |
| 转向滤波 | 一阶低通 (α=0.4) |

### MPC 控制器

**模型**：线性化运动学自行车模型，状态 `[x, y, θ]`，控制量 `δ`（方向盘转角）

**QP 形式**：
```
min  Σ(Δzᵀ·Q·Δz + R·δ²)
s.t. |δ| ≤ δ_max
```

| 参数 | 值 |
|------|-----|
| 预测时域 | 15 步 |
| 时间步长 | 0.05 s |
| Q_xy | 10.0 |
| Q_theta | 5.0 |
| R_steer | 0.5 |
| 求解器 | OSQP（400 最大迭代，1e-4 精度） |

### Stanley 控制器

**公式**：`δ = θ_err + arctan(k·cte / v)`

| 参数 | 值 |
|------|-----|
| Stanley k | 0.5 |
| 前馈 | 参考路径曲率（40% 权重） |
| 滤波 | 一阶低通 (α=0.4) |

---

## 关键配置

### nav2_params_saye.yaml

主要参数位置：`saye_nav2_plugins/config/nav2_params_saye.yaml`

| 参数 | 值 | 说明 |
|------|-----|------|
| `obstacle_max_range` | 25.0 m | 全局+局部 costmap 障碍检测范围 |
| `raytrace_max_range` | 8.0 m | 射线追踪清除范围（防 costmap 抖动） |
| `controller_frequency` | 20.0 Hz | 控制器运行频率 |
| `progress_checker.movement_time_allowance` | 8.0 s | 进度检测超时 |
| `progress_checker.required_movement_radius` | 0.2 m | 进度检测最小移动距离 |
| `velocity_smoother.max_accel[2]` | 1000.0 | angular.z 直通（保持 v/ω 比率） |
| `inflation_radius` | 0.35 m | 膨胀层半径 |
| `cost_scaling_factor` | 3.0 | 膨胀衰减因子 |
| `yaw_goal_tolerance` | 0.25 rad | 目标朝向容差 |

### ros_gz_bridge.yaml

主要桥接条目：`saye_bringup/config/ros_gz_bridge.yaml`

| 话题 | 方向 | 说明 |
|------|------|------|
| `/cmd_vel` | BIDIRECTIONAL | 速度命令 ↔ Gazebo Ackermann 插件 |
| `/odom` | GZ→ROS | 里程计（`lazy: false` 确保启动即发布） |
| `/scan` | GZ→ROS | 2D LaserScan（`lazy: false`） |
| `/joint_states` | GZ→ROS | 关节状态（world-scoped topic，**必须使用完整路径**） |
| `/imu` | GZ→ROS | IMU 数据 |
| `/cloud` | GZ→ROS | 3D 点云 |

> **注意**：`/joint_states` 的 Gazebo topic 必须使用 world-scoped 路径 `/world/my_world/model/saye/joint_state`。使用 model-scoped 路径 `/model/saye/joint_state` 会因命名空间不匹配导致桥接收不到数据。

---

## 技术要点

### 1. Gazebo AckermannSteering 转向语义

Gazebo Harmonic 的 `AckermannSteering` 插件源码核心逻辑：

```cpp
turningRadius = linVel / angVel;        // r = v / ω
phi = atan(wheelBase / turningRadius);  // δ = atan(L / r)
```

**结论**：`cmd_vel.angular.z` = 横摆角速度 (rad/s)，**不是方向盘转角**。

因此控制器的正确输出公式为：

```cpp
cmd.twist.angular.z = v * std::tan(steer) / vp_.wheelbase;
```

其中 `v` 与 `cmd.twist.linear.x` 同源，保证插件反算：`δ = atan(L / (v/(v·tanδ/L))) = δ` 恒成立。

> 详细分析见 [20260511.md](20260511.md) 第一章。

### 2. 路径坐标系变换（帧不匹配）

Nav2 的 `controller_server` 在调用 `setPlan()` 时**不进行坐标变换**。所有内置控制器（DWB、RPP、MPPI）都在 `computeVelocityCommands()` 内部自行调用 `transformGlobalPlan()`。自定义控制器必须自己完成此变换。

本项目三个控制器统一添加了 `transformGlobalPlan()` 方法：

```cpp
// 获取 map→odom 变换（1次 TF lookup）
tf_stamped = tf_->lookupTransform(odom_frame, map_frame, tf2::TimePointZero);
// 手动对所有路径点做 R·p + t（避免逐点 TF 查询的性能开销）
```

> 详细分析见 [20260511.md](20260511.md) 第二章。

### 3. RViz 3D 模型显示修复

RViz 不显示小车模型的问题经排查为**四个独立配置缺失**的叠加：

| 层次 | 问题 | 修复 |
|------|------|------|
| 1 | `robot_description` 内容是 SDF 而非 URDF | 写 Python 脚本做 SDF→URDF 转换 |
| 2 | 缺少 `/joint_states` 桥接 | 新增 ros_gz_bridge 桥接条目 |
| 3 | 桥接订阅了错误命名空间 | 使用 world-scoped 完整路径 |
| 4 | URDF mesh 格式不符规范 | `<mesh><uri>` → `<mesh filename="">` |
| 附 | inertia 缺 ixy 属性 | 补默认值 "0" |
| 附 | RViz TF Prefix 与帧名冲突 | 清空 TF Prefix |

> 详细分析见 [20260511.md](20260511.md) 第七章。

### 4. velocity_smoother 对 Ackermann 的影响

`velocity_smoother` 默认独立平滑 `linear.x`（±0.5 m/s²）和 `angular.z`（±1.5 m/s²），暂态期间 `v/ω` 比率错误直接导致转弯半径错误。Ackermann 车辆的 `angular.z` 本身即为横摆角速度，比率错误无法通过差速绕行补偿。解决方案：将 `max_accel[2]` 设为极大值使 `angular.z` 直通。

### 5. SDF→URDF 自动转换

`saye_description/scripts/sdf_to_urdf.py` 在仿真启动时自动将 SDF 模型转换为 URDF，关键转换规则：

| SDF 格式 | URDF 格式 |
|----------|----------|
| `<mesh><uri>...</uri></mesh>` | `<mesh filename="..."/>` |
| `<parent>link_name</parent>` | `<parent link="link_name"/>` |
| `<pose>x y z r p y</pose>` | `<origin xyz="x y z" rpy="r p y"/>` |
| `<inertia><ixx>...</ixx></inertia>` | `<inertia ixx="..." .../>`（全部 6 属性） |
| `<gz_frame_id>` | 剥离 |
| `<sensor>` | 剥离 |
| `<plugin>` | 剥离 |

---

## 目录结构

```
ackermann_sim/
└── src/ackermann-vehicle-gzsim-ros2/
    ├── saye_bringup/           # 仿真启动 + 上位机桥接
    │   ├── launch/             # 启动文件
    │   ├── config/             # ros_gz_bridge.yaml
    │   ├── maps/               # 仓库地图 (752×1251 @ 0.04m/cell)
    │   └── rviz/               # RViz 配置
    ├── saye_description/        # 车辆模型 + 世界描述
    │   ├── models/saye/        # SDF 车辆模型 + COLLADA 网格
    │   ├── worlds/             # 仓库世界 SDF
    │   ├── scripts/            # sdf_to_urdf.py 转换脚本
    │   └── hooks/              # Gazebo 资源路径钩子
    ├── saye_control/           # 手柄控制 + 地图服务客户端
    ├── saye_msgs/              # 自定义消息和服务
    └── saye_nav2_plugins/      # ← 核心：自定义 Nav2 插件
        ├── include/saye_nav2_plugins/
        │   ├── common.hpp               # 共享工具函数
        │   ├── hybrid_astar_planner.hpp  # Hybrid A* 规划器
        │   ├── pure_pursuit_controller.hpp
        │   ├── mpc_controller.hpp       # MPC (OSQP)
        │   └── stanley_controller.hpp
        ├── src/                         # 实现文件
        ├── config/nav2_params_saye.yaml # 全部 Nav2 参数
        ├── launch/                      # 导航启动文件
        ├── scripts/odom_to_tf.py        # TF 修复节点
        └── plugins.xml                  # pluginlib 注册
```

---

## 已知问题

- **动态避障效果待优化**：避障后卡住问题已缓解，但远未完全解决。Spin 恢复行为对 Ackermann 车辆无效（Spin 发送角速度，Ackermann 插件解为转角）。
- **目标朝向偏差**：Reeds-Shepp 扩展缓解了大部分场景，偶见终点附近滑过。
- **DDS 生命周期超时**：`default_server_timeout=60` 为临时解决方案，根因在 RMW/DDS 层。

---

## 参考文档

| 文档 | 说明 |
|------|------|
| [20260511.md](20260511.md) | 全部问题修复的详细技术分析（685 行） |
| [面试问答素材.md](.opencode_archive/面试问答素材.md) | 面试准备用问答材料 |
| [interview_resume.md](interview_resume.md) | 简历条目 + 面试话术 |

---

## 许可证

Apache License 2.0

原项目：https://github.com/alitekes1/ackermann-vehicle-gzsim-ros2


