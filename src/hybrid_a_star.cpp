#include "dcs_nav_plugin/hybrid_a_star.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"

// 注册插件，使其可以被 Nav2 动态加载
PLUGINLIB_EXPORT_CLASS(dcs_nav_plugin::HybridAStar, nav2_core::GlobalPlanner)

namespace dcs_nav_plugin
{

HybridAStar::HybridAStar() : costmap_(nullptr)
{
}

HybridAStar::~HybridAStar()
{
}

/**
 * @brief 插件配置函数，在插件加载时被调用
 * 
 * @param parent 父节点指针，用于获取参数和日志
 * @param name 插件名称
 * @param tf TF变换缓冲
 * @param costmap_ros 代价地图ROS接口
 */
void HybridAStar::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap(); // 获取底层代价地图对象
  global_frame_ = costmap_ros_->getGlobalFrameID(); // 获取全局坐标系（通常是 map）

  auto node = node_;
  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }

  // --- 参数声明与获取 ---

  // ========== 读取运动模型类型 ==========
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".motion_model", rclcpp::ParameterValue("ACKERMANN"));
  node->get_parameter(name + ".motion_model", motion_model_);
  
  // 验证参数合法性
  if (motion_model_ != "ACKERMANN" && motion_model_ != "OMNIDIRECTIONAL") {
    RCLCPP_ERROR(node->get_logger(), 
      "[HybridAStar] Invalid motion_model: %s. Using ACKERMANN.", 
      motion_model_.c_str());
    motion_model_ = "ACKERMANN";
  }
  
  RCLCPP_INFO(node->get_logger(), 
    "[HybridAStar] 正在激活 HybridAStar 插件: %s", name.c_str());
  RCLCPP_INFO(node->get_logger(), 
    "[HybridAStar] 运动模型: %s", motion_model_.c_str());

  // ========== 通用参数 ==========
  
  // 插值分辨率：路径点的间隔距离 (米)
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
  node->get_parameter(name + ".interpolation_resolution", interpolation_resolution_);
  
  // 射击距离：当距离目标小于此值时，尝试直接用 Reeds-Shepp 曲线连接（暂未使用，保留参数）
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".shot_distance", rclcpp::ParameterValue(5.0));
  node->get_parameter(name + ".shot_distance", shot_distance_);

  // 角度离散化：将 360 度分为多少份，用于构建 3D 网格 (x, y, theta)
  // 72 表示每 5 度一份
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".theta_discretization", rclcpp::ParameterValue(72)); 
  node->get_parameter(name + ".theta_discretization", theta_discretization_);

  // ========== Ackermann 模式专用参数 ==========
  
  // 最小转弯半径：车辆运动学的核心约束 (米)
  // 根据 X3 机器人 URDF 估算，约为 0.4m
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".min_turning_radius", rclcpp::ParameterValue(0.4)); 
  node->get_parameter(name + ".min_turning_radius", min_turning_radius_);

  // 初始化 OMPL 的 Reeds-Shepp 状态空间
  // 用于计算考虑转弯半径和倒车的最短路径距离，作为启发式函数
  reeds_shepp_space_ = std::make_shared<ompl::base::ReedsSheppStateSpace>(min_turning_radius_);

  // 倒车惩罚因子：值越大，规划器越倾向于向前行驶
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".reverse_penalty", rclcpp::ParameterValue(2.0));
  node->get_parameter(name + ".reverse_penalty", reverse_penalty_);

  // 换向惩罚因子：防止路径中出现频繁的前进/后退切换
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".direction_change_penalty", rclcpp::ParameterValue(2.0));
  node->get_parameter(name + ".direction_change_penalty", direction_change_penalty_);
  
  // ========== Omnidirectional 模式专用参数 ==========
  
  if (motion_model_ == "OMNIDIRECTIONAL") {
    // 横向移动步长（米）
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".lateral_step_size", rclcpp::ParameterValue(0.2));
    node->get_parameter(name + ".lateral_step_size", lateral_step_size_);
    
    // 原地旋转步长（弧度）
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".rotation_step", rclcpp::ParameterValue(0.26));  // ~15°
    node->get_parameter(name + ".rotation_step", rotation_step_);
    
    // 旋转代价惩罚因子
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".rotation_penalty", rclcpp::ParameterValue(0.5));
    node->get_parameter(name + ".rotation_penalty", rotation_penalty_);
    
    // 对角线移动代价惩罚因子
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".diagonal_penalty", rclcpp::ParameterValue(1.2));
    node->get_parameter(name + ".diagonal_penalty", diagonal_penalty_);
    
    RCLCPP_INFO(node->get_logger(), 
      "  横向步长=%.2fm, 旋转步长=%.1f°", 
      lateral_step_size_, rotation_step_ * 180.0 / M_PI);
  }

  // Debug Utils
  std::string debug_log_dir;
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".debug.log_dir", rclcpp::ParameterValue("~/ros2_ws/src/dcs_nav_plugin/planner_log"));
  node->get_parameter(name + ".debug.log_dir", debug_log_dir);

  debug_utils_ = std::make_shared<DebugUtils>();
  debug_utils_->initialize(debug_log_dir, "planner");
}

void HybridAStar::cleanup()
{
  RCLCPP_INFO(
    rclcpp::get_logger("HybridAStar"), "正在清理 HybridAStar 插件: %s",
    name_.c_str());
}

void HybridAStar::activate()
{
  RCLCPP_INFO(
    rclcpp::get_logger("HybridAStar"), "正在激活 HybridAStar 插件: %s",
    name_.c_str());
}

void HybridAStar::deactivate()
{
  RCLCPP_INFO(
    rclcpp::get_logger("HybridAStar"), "正在停用 HybridAStar 插件: %s",
    name_.c_str());
}

/**
 * @brief 核心规划函数，计算从起点到终点的路径
 * 
 * @param start 起点位姿
 * @param goal 终点位姿
 * @return nav_msgs::msg::Path 规划出的路径
 */
nav_msgs::msg::Path HybridAStar::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  RCLCPP_INFO(rclcpp::get_logger("HybridAStar"), "==========================================");
  RCLCPP_INFO(rclcpp::get_logger("HybridAStar"), "开始 Hybrid A* 路径规划...");
  
  if (debug_utils_) {
      debug_utils_->log("HybridAStar::createPlan called.");
  }
  
  nav_msgs::msg::Path global_path;
  global_path.poses.clear();
  global_path.header.stamp = start.header.stamp;
  global_path.header.frame_id = global_frame_;

  if (!costmap_) {
    RCLCPP_ERROR(rclcpp::get_logger("HybridAStar"), "代价地图未初始化!");
    return global_path;
  }

  // 1. 初始化 Open Set (优先队列) 和 Closed Set (哈希表)
  // Open Set 存储待扩展的节点，按总代价 (f = g + h) 排序
  std::priority_queue<Node3D*, std::vector<Node3D*>, Node3DComparator> open_list;
  
  // Closed Set 存储已访问过的节点，防止重复搜索
  // Key: 离散化后的索引 (x_idx, y_idx, theta_idx), Value: 节点指针
  std::unordered_map<long, Node3D*> closed_list;
  
  // All Nodes 用于内存管理，存储所有创建的节点指针
  std::unordered_map<long, Node3D*> all_nodes; 

  // 2. 设置起点和终点
  double start_x = start.pose.position.x;
  double start_y = start.pose.position.y;
  double start_theta = tf2::getYaw(start.pose.orientation);

  double goal_x = goal.pose.position.x;
  double goal_y = goal.pose.position.y;
  double goal_theta = tf2::getYaw(goal.pose.orientation);
  
  std::stringstream ss_start;
  ss_start << "[HybridAStar] 起点: (" << start_x << ", " << start_y << ", " << start_theta * 180.0 / PI << "°) | "
           << "终点: (" << goal_x << ", " << goal_y << ", " << goal_theta * 180.0 / PI << "°)";
  debug_utils_->log(ss_start.str());
  
  double straight_line_dist = std::hypot(goal_x - start_x, goal_y - start_y);
  // debug_utils_->log("直线距离: " + std::to_string(straight_line_dist) + " 米");

  // 创建起点节点
  Node3D* start_node = new Node3D(start_x, start_y, start_theta, 0.0, 0.0, nullptr, 0, false);
  Node3D goal_node(goal_x, goal_y, goal_theta, 0.0, 0.0, nullptr, 0, false);

  // 检查起点和终点是否有效
  RCLCPP_INFO(rclcpp::get_logger("HybridAStar"), "正在验证起点和终点...");
  if (isCollision(start_x, start_y, start_theta)) {
    RCLCPP_ERROR(rclcpp::get_logger("HybridAStar"), "起点位置碰撞或超出地图!");
    delete start_node;
    return global_path;
  }
  if (isCollision(goal_x, goal_y, goal_theta)) {
    RCLCPP_ERROR(rclcpp::get_logger("HybridAStar"), "终点位置碰撞或超出地图!");
    delete start_node;
    return global_path;
  }
  RCLCPP_INFO(rclcpp::get_logger("HybridAStar"), "起点和终点验证通过。");

  // 计算起点的启发式代价 (h值)
  start_node->h = getHeuristic(*start_node, goal_node);
  start_node->cost = start_node->g + start_node->h;
  
  // 计算起点的离散化索引
  int sx, sy, st;
  if (!getIndex(*start_node, sx, sy, st)) {
    RCLCPP_ERROR(rclcpp::get_logger("HybridAStar"), "起点位置超出地图范围!");
    delete start_node;
    return global_path;
  }
  start_node->setIndex(sx, sy, st);

  // 将起点加入 Open List
  open_list.push(start_node);
  
  // 简单的哈希函数，生成唯一 Key
  auto get_index_key = [&](int x, int y, int t) -> long {
      return (long)x + (long)y * 100000 + (long)t * 10000000000; 
      // 注意：这是一个简单的哈希，假设地图尺寸在 x, y 方向不超过 100000
  };
  
  all_nodes[get_index_key(sx, sy, st)] = start_node;

  Node3D* current_node = nullptr;
  int iterations = 0;
  int max_iterations = 100000; // 最大迭代次数，防止死循环
  
  RCLCPP_INFO(rclcpp::get_logger("HybridAStar"), 
    "开始搜索,初始启发值: %.3f", start_node->h);

  // --- 主搜索循环 ---
  while (!open_list.empty() && iterations < max_iterations) {
    // 取出代价最小的节点
    current_node = open_list.top();
    open_list.pop();
    iterations++;

    long current_key = get_index_key(current_node->x_idx, current_node->y_idx, current_node->theta_idx);

    // 如果已经在 Closed List 中，说明已经找到过更优或等价的路径到达此状态，跳过
    if (closed_list.find(current_key) != closed_list.end()) {
      continue;
    }
    closed_list[current_key] = current_node;

    // 3. 检查是否到达终点
    // 同时判断位置和角度
    double dist_to_goal = std::hypot(current_node->x - goal_x, current_node->y - goal_y);
    double angle_diff = std::abs(current_node->theta - goal_theta);
    // 归一化角度差到 [0, PI]
    while (angle_diff > PI) angle_diff = std::abs(angle_diff - 2 * PI);
    
    if (dist_to_goal < 0.01 && angle_diff < 0.05) { 
        // 找到路径,回溯生成完整路径
        RCLCPP_INFO(rclcpp::get_logger("HybridAStar"), 
          "已到达终点! 最终误差: 距离=%.3fm, 角度=%.1f°",
          dist_to_goal, angle_diff * 180.0 / PI);
        global_path = reconstructPath(current_node, start, goal);
        break;
    }
    
    // 4. Analytic Expansion: 尝试直接连接目标
    if (dist_to_goal < shot_distance_) {
        nav_msgs::msg::Path analytic_path;
        
        if (motion_model_ == "OMNIDIRECTIONAL") {
            // 全向模式:使用直线扩展(不改变朝向,直接移动)
            analytic_path = tryLinearExpansion(current_node, goal_node);
            if (!analytic_path.poses.empty()) {
                RCLCPP_INFO(rclcpp::get_logger("HybridAStar"), 
                  "全向直线扩展成功! 生成 %zu 个路由点。",
                  analytic_path.poses.size());
            }
        } else {
            // Ackermann模式:使用Reeds-Shepp曲线
            analytic_path = tryAnalyticExpansion(current_node, goal_node);
            if (!analytic_path.poses.empty()) {
                RCLCPP_INFO(rclcpp::get_logger("HybridAStar"), 
                  "RS曲线扩展成功! 生成 %zu 个路由点。",
                  analytic_path.poses.size());
            }
        }
        
        if (!analytic_path.poses.empty()) {
            // 从当前节点的父节点开始回溯（因为 analytic_path 已包含 current_node 的位置）
            nav_msgs::msg::Path backtrack_path;
            backtrack_path.header.stamp = start.header.stamp;
            backtrack_path.header.frame_id = global_frame_;
            
            // 回溯到起点
            Node3D* curr = current_node->parent;
            std::vector<geometry_msgs::msg::PoseStamped> temp_poses;
            while (curr != nullptr) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = backtrack_path.header;
                pose.pose.position.x = curr->x;
                pose.pose.position.y = curr->y;
                pose.pose.position.z = 0.0;
                tf2::Quaternion q;
                q.setRPY(0, 0, curr->theta);
                pose.pose.orientation = tf2::toMsg(q);
                temp_poses.push_back(pose);
                curr = curr->parent;
            }
            // 反转得到从起点到当前节点父节点的路径
            std::reverse(temp_poses.begin(), temp_poses.end());
            backtrack_path.poses = temp_poses;
            
            // 将解析路径添加到回溯路径后面
            for (const auto& pose : analytic_path.poses) {
                backtrack_path.poses.push_back(pose);
            }
            global_path = backtrack_path;
            break;
        }
    }

    // 5. 扩展邻居节点 (Motion Primitives)
    std::vector<Node3D*> neighbors = getNeighbors(current_node, goal_node);
    for (Node3D* neighbor : neighbors) {
        int nx, ny, nt;
        if (!getIndex(*neighbor, nx, ny, nt)) {
            // 邻居在地图外，跳过
            delete neighbor;
            continue;
        }
        neighbor->setIndex(nx, ny, nt);
        long neighbor_key = get_index_key(nx, ny, nt);

        // 如果邻居已经在 Closed List 中，跳过
        if (closed_list.find(neighbor_key) != closed_list.end()) {
            delete neighbor;
            continue;
        }

        // 检查是否在 All Nodes 中（即之前生成过但未处理完，或在 Open List 中）
        if (all_nodes.find(neighbor_key) != all_nodes.end()) {
            // 如果新路径代价更高，则丢弃新节点
            if (all_nodes[neighbor_key]->g <= neighbor->g) {
                delete neighbor;
                continue;
            }
            // 如果新路径更好，理论上应该更新 Open List 中的节点
            // 这里简化处理：直接覆盖 all_nodes 中的记录，并加入 Open List
            // (旧节点会留在 Open List 中，但在弹出时会被 Closed List 检查过滤掉)
        }
        
        all_nodes[neighbor_key] = neighbor;
        open_list.push(neighbor);
    }
  }

  // 输出搜索统计
  RCLCPP_INFO(rclcpp::get_logger("HybridAStar"), 
    "搜索完成: %d 次迭代, %zu 个节点已扩展, %zu 个节点在开放列表",
    iterations, closed_list.size(), open_list.size());
  
  if (global_path.poses.empty()) {
      std::stringstream ss_fail;
      ss_fail << "[HybridAStar] 路径规划失败! 迭代: " << iterations 
              << ", 已探索: " << closed_list.size() << ", 开放列表: " << open_list.size();
      debug_utils_->log(ss_fail.str());
  } else {
      double path_length = 0.0;
      for (size_t i = 1; i < global_path.poses.size(); ++i) {
          double dx = global_path.poses[i].pose.position.x - 
                      global_path.poses[i-1].pose.position.x;
          double dy = global_path.poses[i].pose.position.y - 
                      global_path.poses[i-1].pose.position.y;
          path_length += std::hypot(dx, dy);
      }
      
      std::stringstream ss_succ;
      ss_succ << "[HybridAStar] ✓ 成功! 长度: " << path_length << " m (" 
              << (path_length / straight_line_dist) * 100.0 << "%), 点数: " << global_path.poses.size()
              << ", 迭代: " << iterations << ", 探索: " << closed_list.size();
      debug_utils_->log(ss_succ.str());
  }
  // RCLCPP_INFO(rclcpp::get_logger("HybridAStar"), "==========================================");

  // 5. 清理内存
  for (auto& pair : all_nodes) {
      delete pair.second;
  }

  return global_path;
}

/**
 * @brief 碰撞检测函数
 * 
 * @param x 世界坐标 x
 * @param y 世界坐标 y
 * @param theta 角度 (目前未使用，未来可用于足迹检测)
 * @return true 发生碰撞
 * @return false 无碰撞
 */
bool HybridAStar::isCollision(double x, double y, double theta)
{
    unsigned int mx, my;
    // 检查是否超出地图范围
    if (!costmap_->worldToMap(x, y, mx, my)) {
        return true; 
    }
    // 检查该栅格的代价值
    unsigned char cost = costmap_->getCost(mx, my);
    // 使用 INSCRIBED_INFLATED_OBSTACLE (253) 作为阈值
    // 这表示如果机器人中心处于此位置，机器人边缘必然碰撞
    if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        return true;
    }
    return false;
}

/**
 * @brief 计算启发式代价 (Heuristic Cost)
 * 使用 Reeds-Shepp 曲线距离，考虑了车辆的最小转弯半径和倒车能力
 * 
 * @param node 当前节点
 * @param goal 目标节点
 * @return double 估计的剩余代价
 */
double HybridAStar::getHeuristic(const Node3D& node, const Node3D& goal)
{
    ompl::base::ScopedState<ompl::base::SE2StateSpace> from(reeds_shepp_space_);
    ompl::base::ScopedState<ompl::base::SE2StateSpace> to(reeds_shepp_space_);
    
    from->setX(node.x);
    from->setY(node.y);
    from->setYaw(node.theta);
    
    to->setX(goal.x);
    to->setY(goal.y);
    to->setYaw(goal.theta);
    
    // 返回 Reeds-Shepp 距离
    return reeds_shepp_space_->distance(from.get(), to.get());
}

/**
 * @brief 将连续的世界坐标转换为离散的网格索引
 * 
 * @param node 节点
 * @param x_idx 输出 x 索引
 * @param y_idx 输出 y 索引
 * @param theta_idx 输出 theta 索引
 * @return true 转换成功
 * @return false 点在地图外
 */
bool HybridAStar::getIndex(const Node3D& node, int& x_idx, int& y_idx, int& theta_idx)
{
    unsigned int mx, my;
    if (!costmap_->worldToMap(node.x, node.y, mx, my)) {
        return false;  // 点在地图外
    }
    x_idx = static_cast<int>(mx);
    y_idx = static_cast<int>(my);
    
    // 归一化角度到 [0, 2*PI)
    double t = node.theta;
    while (t < 0) t += 2 * PI;
    while (t >= 2 * PI) t -= 2 * PI;
    
    // 将角度离散化
    theta_idx = static_cast<int>(t / (2 * PI / theta_discretization_));
    if (theta_idx >= theta_discretization_) theta_idx = 0;  // 防止越界
    
    return true;
}

/**
 * @brief 生成邻居节点 (Motion Primitives) - 调度器
 * 根据运动模型选择对应的实现
 * 
 * @param current 当前节点
 * @param goal 目标节点 (用于计算 h 值)
 * @return std::vector<Node3D*> 邻居节点列表
 */
std::vector<Node3D*> HybridAStar::getNeighbors(Node3D* current, const Node3D& goal)
{
    if (motion_model_ == "OMNIDIRECTIONAL") {
        return getNeighborsOmnidirectional(current, goal);
    } else {
        return getNeighborsAckermann(current, goal);
    }
}

/**
 * @brief Ackermann 运动模型邻居生成
 * 类汽车运动学：前进/后退 × 直行/左转/右转
 * 
 * @param current 当前节点
 * @param goal 目标节点
 * @return std::vector<Node3D*> 邻居节点列表
 */
std::vector<Node3D*> HybridAStar::getNeighborsAckermann(Node3D* current, const Node3D& goal)
{
    std::vector<Node3D*> neighbors;
    double step_size = interpolation_resolution_ * 2.0; // 步长略大于分辨率
    
    // 运动原语 (Motion Primitives): 
    // 0: 前进直行
    // 1: 前进左转
    // 2: 前进右转
    // 3: 后退直行
    // 4: 后退左转
    // 5: 后退右转
    
    for (int i = 0; i < 6; ++i) {
        double next_x, next_y, next_theta;
        bool reverse = (i >= 3); // 是否倒车
        double direction = reverse ? -1.0 : 1.0;
        double move_dist = direction * step_size;
        
        if (i % 3 == 0) { // 直行
            next_theta = current->theta;
            next_x = current->x + move_dist * cos(next_theta);
            next_y = current->y + move_dist * sin(next_theta);
        } else { // 转弯
            // 计算转弯角度：弧长 / 半径
            double turn_angle = step_size / min_turning_radius_;
            if (i % 3 == 2) turn_angle = -turn_angle; // 右转为负
            
            // 简单的运动学更新 (近似圆弧)
            // 这里的计算是基于圆心的旋转
            next_theta = current->theta + direction * turn_angle;
            // 使用半角公式近似计算位移，比直接用 current->theta 更准
            next_x = current->x + move_dist * cos(current->theta + direction * turn_angle / 2.0);
            next_y = current->y + move_dist * sin(current->theta + direction * turn_angle / 2.0);
        }
        
        // 碰撞检测
        if (!isCollision(next_x, next_y, next_theta)) {
            double g_cost = current->g + step_size;
            
            // 应用惩罚
            if (reverse) g_cost += reverse_penalty_ * step_size; // 倒车惩罚
            if (current->is_reverse != reverse && current->parent != nullptr) g_cost += direction_change_penalty_; // 换向惩罚
            
            Node3D* next_node = new Node3D(next_x, next_y, next_theta, g_cost, 0, current, i, reverse);
            next_node->h = getHeuristic(*next_node, goal);
            next_node->cost = next_node->g + next_node->h;
            neighbors.push_back(next_node);
        }
    }
    return neighbors;
}

/**
 * @brief Omnidirectional 运动模型邻居生成
 * 全向轮运动学：8方向平移 + 原地旋转
 * 
 * @param current 当前节点
 * @param goal 目标节点
 * @return std::vector<Node3D*> 邻居节点列表
 */
std::vector<Node3D*> HybridAStar::getNeighborsOmnidirectional(Node3D* current, const Node3D& goal)
{
    std::vector<Node3D*> neighbors;
    double step = lateral_step_size_;
    
    // ========== Part 1: 8方向平移（朝向保持不变） ==========
    struct MoveAction {
        double dx, dy;
        bool is_diagonal;
    };
    
    std::vector<MoveAction> moves = {
        // 主方向（4个）
        {1, 0, false},   // 前
        {-1, 0, false},  // 后
        {0, 1, false},   // 左
        {0, -1, false},  // 右
        // 对角线（4个）
        {1, 1, true},    // 左前
        {1, -1, true},   // 右前
        {-1, 1, true},   // 左后
        {-1, -1, true}   // 右后
    };
    
    for (auto& move : moves) {
        double next_x = current->x + move.dx * step;
        double next_y = current->y + move.dy * step;
        double next_theta = current->theta;  // ← 朝向保持不变！
        
        if (!isCollision(next_x, next_y, next_theta)) {
            double dist = step;
            if (move.is_diagonal) {
                dist = step * std::sqrt(2.0);  // 对角线距离
                dist *= diagonal_penalty_;     // 应用惩罚
            }
            
            double g_cost = current->g + dist;
            
            Node3D* next = new Node3D(
                next_x, next_y, next_theta, 
                g_cost, 0, current, -1, false
            );
            next->h = getHeuristic(*next, goal);
            next->cost = next->g + next->h;
            neighbors.push_back(next);
        }
    }
    
    // ========== Part 2: 原地旋转（位置不变） ==========
    for (int dir : {-1, 1}) {  // 左转、右转
        double next_theta = current->theta + dir * rotation_step_;
        
        if (!isCollision(current->x, current->y, next_theta)) {
            double g_cost = current->g + rotation_step_ * rotation_penalty_;
            
            Node3D* next = new Node3D(
                current->x, current->y, next_theta,
                g_cost, 0, current, -1, false
            );
            next->h = getHeuristic(*next, goal);
            next->cost = next->g + next->h;
            neighbors.push_back(next);
        }
    }
    
    return neighbors;
}

/**
 * @brief 从终点回溯到起点，构建路径
 * 
 * @param node 终点节点
 * @param start 起点位姿 (用于添加精确起点)
 * @param goal 终点位姿 (用于添加精确终点)
 * @return nav_msgs::msg::Path 
 */
nav_msgs::msg::Path HybridAStar::reconstructPath(Node3D* node, const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal)
{
    nav_msgs::msg::Path path;
    path.header.stamp = start.header.stamp;
    path.header.frame_id = global_frame_;
    
    // 回溯收集所有节点
    std::vector<geometry_msgs::msg::PoseStamped> temp_poses;
    Node3D* curr = node;
    while (curr != nullptr) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = curr->x;
        pose.pose.position.y = curr->y;
        pose.pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, curr->theta);
        pose.pose.orientation = tf2::toMsg(q);
        
        temp_poses.push_back(pose);
        curr = curr->parent;
    }
    
    // 反转得到正向路径
    std::reverse(temp_poses.begin(), temp_poses.end());
    
    // 添加精确的起点（如果第一个点不够接近）
    if (!temp_poses.empty()) {
        double dist_to_start = std::hypot(
            temp_poses.front().pose.position.x - start.pose.position.x,
            temp_poses.front().pose.position.y - start.pose.position.y);
        if (dist_to_start > 0.01) {
            geometry_msgs::msg::PoseStamped start_pose = start;
            start_pose.header = path.header;
            path.poses.push_back(start_pose);
        }
    }
    
    // 添加回溯的路径点
    for (const auto& pose : temp_poses) {
        path.poses.push_back(pose);
    }
    
    // 添加精确的终点(如果最后一个点与目标不够接近)
    if (!path.poses.empty()) {
        double dist_to_goal = std::hypot(
            path.poses.back().pose.position.x - goal.pose.position.x,
            path.poses.back().pose.position.y - goal.pose.position.y);
            
        double last_theta = tf2::getYaw(path.poses.back().pose.orientation);
        double goal_theta = tf2::getYaw(goal.pose.orientation);
        double ang_to_goal = std::abs(last_theta - goal_theta);
        // Normalize angle difference
        while (ang_to_goal > PI) ang_to_goal -= 2.0 * PI;
        while (ang_to_goal < -PI) ang_to_goal += 2.0 * PI;
        ang_to_goal = std::abs(ang_to_goal);

        if (dist_to_goal > 0.01 || ang_to_goal > 0.01) {
            geometry_msgs::msg::PoseStamped goal_pose = goal;
            goal_pose.header = path.header;
            path.poses.push_back(goal_pose);
        }
    }
    
    return path;
}

/**
 * @brief 尝试使用 Reeds-Shepp 曲线直接连接到目标
 * 
 * @param current 当前节点
 * @param goal 目标节点
 * @return nav_msgs::msg::Path 如果无碰撞则返回路径，否则返回空路径
 */
nav_msgs::msg::Path HybridAStar::tryAnalyticExpansion(Node3D* current, const Node3D& goal)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = global_frame_;
    
    // 使用 OMPL 计算 Reeds-Shepp 路径
    ompl::base::ScopedState<ompl::base::SE2StateSpace> from(reeds_shepp_space_);
    ompl::base::ScopedState<ompl::base::SE2StateSpace> to(reeds_shepp_space_);
    
    from->setX(current->x);
    from->setY(current->y);
    from->setYaw(current->theta);
    
    to->setX(goal.x);
    to->setY(goal.y);
    to->setYaw(goal.theta);
    
    double rs_length = reeds_shepp_space_->distance(from.get(), to.get());
    
    // 沿着 Reeds-Shepp 曲线采样并检查碰撞
    int num_samples = static_cast<int>(rs_length / interpolation_resolution_) + 1;
    if (num_samples < 2) num_samples = 2;
    
    std::vector<geometry_msgs::msg::PoseStamped> sampled_poses;
    
    for (int i = 0; i <= num_samples; ++i) {
        double t = static_cast<double>(i) / num_samples;
        
        ompl::base::State* state = reeds_shepp_space_->allocState();
        reeds_shepp_space_->interpolate(from.get(), to.get(), t, state);
        
        auto* se2state = state->as<ompl::base::SE2StateSpace::StateType>();
        double x = se2state->getX();
        double y = se2state->getY();
        double theta = se2state->getYaw();
        
        reeds_shepp_space_->freeState(state);
        
        // 碰撞检测
        if (isCollision(x, y, theta)) {
            return nav_msgs::msg::Path(); // 返回空路径表示失败
        }
        
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = global_frame_;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        pose.pose.orientation = tf2::toMsg(q);
        
        sampled_poses.push_back(pose);
    }
    
    path.poses = sampled_poses;
}

/**
 * @brief 全向模式专用：尝试直线连接到目标
 * 直接从当前点到目标点画直线，朝向在最后才转向目标朝向
 * 
 * @param current 当前节点
 * @param goal 目标节点
 * @return nav_msgs::msg::Path 如果无碰撞则返回路径，否则返回空路径
 */
nav_msgs::msg::Path HybridAStar::tryLinearExpansion(Node3D* current, const Node3D& goal)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = global_frame_;
    
    // 计算直线距离
    double dx = goal.x - current->x;
    double dy = goal.y - current->y;
    double linear_dist = std::hypot(dx, dy);
    
    // 计算采样点数
    int num_samples = static_cast<int>(linear_dist / interpolation_resolution_) + 1;
    if (num_samples < 2) num_samples = 2;
    
    std::vector<geometry_msgs::msg::PoseStamped> sampled_poses;
    
    // 沿直线采样
    for (int i = 0; i <= num_samples; ++i) {
        double t = static_cast<double>(i) / num_samples;
        
        double x = current->x + t * dx;
        double y = current->y + t * dy;
        
        // 朝向：在路径的最后20%逐渐转向目标朝向
        double theta;
        if (t < 0.8) {
            // 前80%路径：保持当前朝向
            theta = current->theta;
        } else {
            // 后20%路径：从当前朝向平滑过渡到目标朝向
            double blend = (t - 0.8) / 0.2;  // 0到1
            double angle_diff = goal.theta - current->theta;
            // 归一化角度差
            while (angle_diff > PI) angle_diff -= 2 * PI;
            while (angle_diff < -PI) angle_diff += 2 * PI;
            theta = current->theta + blend * angle_diff;
        }
        
        // 碰撞检测
        if (isCollision(x, y, theta)) {
            return nav_msgs::msg::Path(); // 返回空路径表示失败
        }
        
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = global_frame_;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        pose.pose.orientation = tf2::toMsg(q);
        
        sampled_poses.push_back(pose);
    }
    
    path.poses = sampled_poses;
    return path;
}

}  // namespace dcs_nav_plugin
