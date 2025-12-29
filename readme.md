# DCS Nav Plugin 项目文档

## 1. 项目架构 (Architecture)

本项目是基于 ROS 2 Navigation 2 (Nav2) 的自定义插件集合，专注于高精度的轨迹规划与基于模型预测控制 (MPC) 的避障跟踪。

### 核心模块
*   **全局规划器 (Global Planner)**: `dcs_nav_plugin/HybridAStar`
    *   **算法**: Hybrid A* (混合 A 星)
    *   **核心机制**: 结合栅格地图搜索与 Reeds-Shepp 曲线，生成符合运动学约束的光滑路径。
    *   **适用场景**: 非完整约束机器人 (Car-like) 及全向移动机器人。

*   **局部控制器 (Local Controller)**: `dcs_nav_plugin/DcsShtMpcController`
    *   **算法**: 基于 CasADi 的非线性 MPC (NMPC)
    *   **避障核心**: 分离超平面定理 (SHT - Separating Hyperplane Theorem)。将避障问题转化为求解分隔凸多边形的超平面参数，而非简单的距离约束。
    *   **动力学模型**: 支持全向移动 (Mecanum) 运动学模型。

*   **几何引擎 (Geometry Engine)**: `src/geometry/geometry_engine.cpp`
    *   **功能**: 负责从 Nav2 Costmap 中提取精确的障碍物几何信息。
    *   **流程**: Costmap -> 二值化图像 -> 轮廓提取 (OpenCV) -> 多边形简化 -> 凸多边形分解 (Convex Decomposition)。

*   **求解器接口 (MPC Solver)**: `src/mpc/mpc_solver_casadi.cpp`
    *   **功能**: 封装 CasADi 库，构建 NLP (非线性规划) 问题并在每帧实时求解控制律。

## 2. 目前已完成工作 (Completed Work)

### A. 混合 A* 规划器 (Done)
- [x] 完成基于 `nav2_core::GlobalPlanner` 的插件封装。
- [x] 实现 Reeds-Shepp 曲线扩展逻辑，支持倒车与原地转向。
- [x] 启发式函数优化 (Euclidean Distance + RS Path Length)。

### B. SHT-MPC 控制器 (In Progress / Optimization)
- [x] **CasADi 集成**: 完成符号变量定义、代价函数构建与约束生成。
- [x] **坐标系对齐 (关键修复)**: 实现了 `transformPlan`，在计算控制量前将全局路径从 Map 坐标系精确转换至 Odom/Control 坐标系，解决了轨迹跟踪的大幅偏移问题。
- [x] **参考轨迹平滑**:
    - 实现了基于切线方向 (`atan2`) 的朝向计算。
    - 实现了 **Yaw Rate Limiting** (朝向变化率限制)，消除了参考轨迹朝向突变导致的控制量震荡 (Twitching)。
- [x] **麦克纳姆轮适配**:
    - 调整了 Q/R 权重矩阵，降低了航向误差 (`Q_theta`) 的权重 (50.0 -> 10.0)，允许机器人更灵活地利用平移能力进行纠偏。
    - 保持了较高的角速度惩罚 (`R_omega`) 以保证运行平稳。

### C. 几何处理 (Done)
- [x] 完成障碍物提取流水线，支持动态 Top-K 障碍物筛选以保证求解实时性。
- [x] 实现了可视化 Marker 发布 (`all_obstacles` vs `selected_obstacles`)，便于在 RViz 中调试 SHT 约束平面。

## 3. 依赖库 (Dependencies)
- **ROS 2 Humble**
- **Nav2 Stack**
- **CasADi** (C++ Interface)
- **OpenCV** (用于 Costmap 处理)
- **tf2_geometry_msgs**
