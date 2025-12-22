# HyperplanePlanner - 基于超平面分离的全局规划器

## 概述

`HyperplanePlanner` 是一个基于 CasADi + IPOPT 的 ROS2 全局路径规划器，实现了超平面分离定理的避障策略。该规划器将路径规划问题建模为非线性优化问题，能够为麦克纳姆轮差速驱动机器人生成平滑、无碰撞的轨迹。

## 核心算法思想

### 1. 超平面分离定理
- 对于两个不相交的凸集（车辆和障碍物），必定存在一个超平面将它们分离
- 超平面定义: `λᵀp = μ`，其中 λ 是法向量，μ 是偏移量
- 约束形式：
  - 车辆顶点在超平面一侧: `λᵀv_vehicle > μ`
  - 障碍物顶点在另一侧: `λᵀv_obstacle < μ`

### 2. 差速驱动动力学模型
```
dx/dt = v * cos(θ)
dy/dt = v * sin(θ)
dθ/dt = ω
dv/dt = a_x
dω/dt = a_θ
```
状态向量: `x = [x, y, θ, v, ω]ᵀ`
控制向量: `u = [a_x, a_θ]ᵀ`

### 3. 优化目标
最小化：`J = Σ(weight_smooth * (a_x² + a_θ²)) + weight_time * T_final`
- 平滑性：减少加速度变化
- 时间最优：尽快到达目标

## 文件结构

```
src/dcs_nav_plugin/
├── include/dcs_nav_plugin/
│   ├── hyperplane_planner.hpp    # 头文件
│   └── hybrid_a_star.hpp         # Hybrid A* 规划器（备用）
├── src/
│   ├── hyperplane_planner.cpp    # 实现文件
│   └── hybrid_a_star.cpp         # Hybrid A* 实现
├── CMakeLists.txt
├── package.xml
└── global_planner_plugin.xml     # 插件描述文件
```

## 安装依赖

### 1. CasADi C++ 库（必需）
```bash
# Ubuntu/Debian
sudo apt-get install libcasadi-dev

# 或从源码安装
git clone https://github.com/casadi/casadi.git
cd casadi
mkdir build && cd build
cmake -DWITH_IPOPT=ON ..
make -j4
sudo make install
```

### 2. 其他依赖（通过 rosdep 自动安装）
```bash
cd /root/yahboomcar_ros2_ws/yahboomcar_ws
rosdep install --from-paths src --ignore-src -r -y
```

## 编译

```bash
cd /root/yahboomcar_ros2_ws/yahboomcar_ws
colcon build --packages-select dcs_nav_plugin --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 配置使用

### 方法 1：修改导航参数文件

编辑 `src/yahboomcar_nav/params/dwa_nav_params.yaml` 或 `teb_nav_params.yaml`:

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: False
    planner_plugins: ["HyperplanePlanner"]  # 取消注释
    HyperplanePlanner:                       # 取消注释整个配置块
      plugin: "dcs_nav_plugin/HyperplanePlanner"
      
      # 车辆几何参数（根据 yahboomcar_X3.urdf）
      vehicle_length: 0.20      # 车辆长度 (m)
      vehicle_width: 0.17       # 车辆宽度 (m)
      vehicle_wheelbase: 0.16   # 轴距 (m)
      
      # 优化设置
      N: 20                     # 时域预测点数
      M: 5                      # RK4 积分子步数
      max_vel_x: 0.5            # 最大线速度 (m/s)
      max_vel_theta: 1.0        # 最大角速度 (rad/s)
      max_acc_x: 1.0            # 最大线加速度 (m/s²)
      max_acc_theta: 2.0        # 最大角加速度 (rad/s²)
      tf_max: 60.0              # 最大时间 (s)
      
      # 代价权重
      weight_smooth: 100.0      # 平滑性权重
      weight_time: 1.0          # 时间权重
      
      # 求解器参数
      solver_max_iter: 3000     # IPOPT 最大迭代次数
      solver_tol: 1e-4          # 容差
      solver_time_limit: 10.0   # 求解时间限制 (s)
      
      # 障碍物处理
      obstacle_inflation: 0.05  # 障碍物膨胀 (m)
      lambda_max: 1e5           # 超平面参数上限
      mu_max: 1e5
      eps: 1e-4                 # 数值稳定性参数
```

### 方法 2：使用现有配置启动

配置文件中已包含 `HyperplanePlanner` 配置（默认注释掉）。启动导航：

```bash
# 使用 DWA 局部规划器 + HyperplanePlanner 全局规划器
ros2 launch yahboomcar_nav navigation_dwa_launch.py

# 或使用 TEB 局部规划器 + HyperplanePlanner 全局规划器
ros2 launch yahboomcar_nav navigation_teb_launch.py
```

## 参数说明

### 车辆几何参数
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `vehicle_length` | 0.20 | 车辆长度（前后方向），单位：米 |
| `vehicle_width` | 0.17 | 车辆宽度（左右方向），单位：米 |
| `vehicle_wheelbase` | 0.16 | 前后轮轴距，单位：米 |

### 优化参数
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `N` | 20 | 预测时域离散点数，值越大轨迹越精细但计算量越大 |
| `M` | 5 | RK4 积分子步数，影响动力学精度 |
| `max_vel_x` | 0.5 | 最大线速度，单位：m/s |
| `max_vel_theta` | 1.0 | 最大角速度，单位：rad/s |
| `max_acc_x` | 1.0 | 最大线加速度，单位：m/s² |
| `max_acc_theta` | 2.0 | 最大角加速度，单位：rad/s² |
| `tf_max` | 60.0 | 最大允许时间，单位：秒 |

### 代价权重
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `weight_smooth` | 100.0 | 平滑性权重（越大轨迹越平滑但可能更慢）|
| `weight_time` | 1.0 | 时间权重（越大越倾向于快速到达）|

### 求解器参数
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `solver_max_iter` | 3000 | IPOPT 最大迭代次数 |
| `solver_tol` | 1e-4 | 优化容差 |
| `solver_time_limit` | 10.0 | 单次规划最大时间，单位：秒 |

### 障碍物处理
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `obstacle_inflation` | 0.05 | 障碍物膨胀距离，单位：米 |
| `lambda_max` | 1e5 | 超平面法向量参数上限 |
| `mu_max` | 1e5 | 超平面偏移参数上限 |
| `eps` | 1e-4 | 防止数值除零的小量 |

## 性能调优

### 提高规划速度
- 减小 `N`（如 15）：减少优化变量数量
- 减小 `M`（如 3）：降低积分精度但加快计算
- 降低 `solver_max_iter`（如 1000）
- 增加 `solver_tol`（如 1e-3）

### 提高轨迹质量
- 增大 `N`（如 30）：更精细的轨迹
- 增大 `M`（如 7）：更精确的动力学模型
- 增大 `weight_smooth`：更平滑的加速度变化
- 减小 `solver_tol`（如 1e-5）

### 提高安全性
- 增大 `obstacle_inflation`：更大的安全边距
- 减小 `max_vel_x` 和 `max_vel_theta`：降低速度
- 减小 `max_acc_x` 和 `max_acc_theta`：更平缓的加速

## 故障排查

### 编译错误：找不到 CasADi
```bash
# 检查 CasADi 是否安装
pkg-config --modversion casadi

# 手动指定 CasADi 路径
colcon build --packages-select dcs_nav_plugin \
  --cmake-args -DCMAKE_PREFIX_PATH=/usr/local
```

### 运行时错误：优化失败
- **现象**: 日志显示 "Optimization failed"
- **原因**: 
  1. 起点或终点在障碍物内
  2. 路径无解（目标不可达）
  3. 时间限制太短
- **解决**:
  1. 检查起点终点位置
  2. 增加 `solver_time_limit`
  3. 增加 `tf_max`
  4. 调整 `obstacle_inflation`

### 规划太慢
- **现象**: 每次规划耗时超过 10 秒
- **解决**: 参考"提高规划速度"章节

### 轨迹不平滑
- **现象**: 机器人运动有抖动
- **解决**: 增大 `weight_smooth` 和 `N`

## 与 Hybrid A* 的对比

| 特性 | HyperplanePlanner | Hybrid A* |
|------|-------------------|-----------|
| 算法类型 | 非线性优化 | 搜索算法 |
| 轨迹质量 | 最优（考虑动力学）| 可行（基于运动原语）|
| 计算时间 | 较慢（秒级）| 较快（毫秒级）|
| 依赖库 | CasADi + IPOPT | OMPL |
| 适用场景 | 静态环境精确规划 | 动态环境快速规划 |
| 动力学约束 | 严格满足 | 近似满足 |

## 已知限制

1. **计算密集**: 优化求解比搜索算法慢，不适合高频率重规划
2. **依赖 CasADi**: 需要额外安装 CasADi C++ 库
3. **障碍物表示**: 当前使用矩形近似，复杂形状可能不精确
4. **局部最优**: IPOPT 可能陷入局部最优解

## 未来改进方向

- [ ] 实现凸分解算法，精确处理复杂障碍物
- [ ] 添加初始猜测热启动，加速求解
- [ ] 支持动态障碍物预测
- [ ] 实现分层规划：A* 粗规划 + 优化精细化
- [ ] 添加轨迹跟踪性能评估

## 参考

- Python 原型: `src/dcs_nav_plugin/src/hyperplane_based methods.py`
- CasADi 文档: https://web.casadi.org/
- Nav2 全局规划器接口: https://navigation.ros.org/plugin_tutorials/docs/writing_new_nav2planner_plugin.html

## 作者

基于 `hyperplane_based methods.py` 的算法思想实现

## 许可证

TODO: 添加许可证信息
